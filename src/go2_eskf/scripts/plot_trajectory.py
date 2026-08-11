#!/usr/bin/env python3
"""Live trajectory + error plot: ESKF estimates vs. Gazebo ground truth.

Two panels, updated in real time:

  * Left  — top-down XY paths of every estimate and the Gazebo ground truth,
            with current-position dots and heading arrows. The view is *clamped*
            to a fixed window (default +/-10 m) so a filter blow-up doesn't zoom
            the whole plot out to a useless scale; when a path leaves the window
            an "OUT OF VIEW" banner appears with the offending position.
  * Right — position error (‖estimate - truth‖) vs. time for each estimate, so
            you can see exactly when divergence starts and whether the
            slip-adaptive arm actually beats the baseline.

Subscribes to up to three ``nav_msgs/Odometry`` topics:
  estimate  (default /eskf/odom)         — baseline go2_eskf_node (fixed R_leg)
  slip      (default /eskf_slip/odom)    — slip-adaptive arm, i.e. the second
                                            go2_eskf_node started by
                                            ``eskf.launch.py slip:=true``
  truth     (default /ground_truth/odom) — bridged gz OdometryPublisher
                                            (see ground_truth.launch.py)

The slip curve is drawn only if that topic is actually publishing (it is
skipped with a log line if the arm isn't running), so the old two-curve
behaviour still works unchanged.

Typical use (see run_go2_teleop.sh --square for the whole workflow):

  ros2 launch unitree_go2_sim unitree_go2_launch.py
  ros2 launch go2_eskf ground_truth.launch.py
  ros2 launch go2_eskf eskf.launch.py use_sim_time:=true slip:=true
  ros2 run go2_eskf plot_trajectory.py --ros-args -p use_sim_time:=true

CLI (after a ``--`` separator, since --ros-args consumes ROS flags):
  ros2 run go2_eskf plot_trajectory.py --ros-args -p use_sim_time:=true -- \
      --view 20 --no-truth --no-slip
"""

import argparse
import math
import sys
import threading
import time

import matplotlib
import matplotlib.pyplot as plt
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_from_quat(q):
    """Yaw (rad) from a geometry_msgs quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class Series:
    """One trajectory: its path, its latest pose, and its error-vs-truth trace."""

    def __init__(self, key, label, color, short=None):
        self.key = key
        self.label = label          # legend text
        self.short = short or label  # title text (kept short — the title is one line)
        self.color = color
        self.xs, self.ys = [], []
        self.pose = None          # (x, y, yaw)
        self.err_t, self.err_v = [], []

    def snapshot(self):
        return {
            "xs": list(self.xs), "ys": list(self.ys), "pose": self.pose,
            "err_t": list(self.err_t), "err_v": list(self.err_v),
            "label": self.label, "short": self.short, "color": self.color,
        }


class TrajectoryPlotter(Node):
    def __init__(self, est_topic, slip_topic, truth_topic, show_truth, show_slip):
        super().__init__("eskf_trajectory_plotter")
        self.show_truth = show_truth
        self._t0 = None  # wall-clock start, for the error-vs-time axis

        # Sensor-ish QoS: best-effort keeps up with a fast stream without
        # blocking on a mismatched reliable publisher.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Shared state, guarded so the ROS thread and the GUI thread don't race.
        self._lock = threading.Lock()
        self.truth = Series("truth", "Gazebo ground truth", "#2e86ab")
        # Estimates, in draw order. Errors are computed against self.truth.
        self.estimates = [
            Series("est", "ESKF estimate (fixed R)", "#d1495b", short="fixed R")]
        if show_slip:
            self.estimates.append(
                Series("slip", "ESKF slip-adaptive", "#f0a202", short="slip-adaptive"))

        self.create_subscription(
            Odometry, est_topic,
            lambda m: self._store(m, self.estimates[0]), qos)
        self.get_logger().info(f"Estimate topic:      {est_topic}")
        if show_slip:
            self.create_subscription(
                Odometry, slip_topic,
                lambda m: self._store(m, self.estimates[1]), qos)
            self.get_logger().info(f"Slip-adaptive topic: {slip_topic}")
        if show_truth:
            self.create_subscription(
                Odometry, truth_topic,
                lambda m: self._store(m, self.truth), qos)
            self.get_logger().info(f"Ground-truth topic:  {truth_topic}")

    def _store(self, msg, series):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            if self._t0 is None:
                self._t0 = time.monotonic()
            series.xs.append(p.x)
            series.ys.append(p.y)
            series.pose = (p.x, p.y, yaw)
            # Log an error sample whenever both sides are known (driven off each
            # estimate's own stream, so every arm gets one point per update).
            if (series is not self.truth and self.show_truth
                    and self.truth.pose is not None):
                err = math.hypot(p.x - self.truth.pose[0],
                                 p.y - self.truth.pose[1])
                series.err_t.append(time.monotonic() - self._t0)
                series.err_v.append(err)

    def snapshot(self):
        """Thread-safe copy of everything the plot needs this frame."""
        with self._lock:
            return {
                "estimates": [s.snapshot() for s in self.estimates],
                "truth": self.truth.snapshot(),
            }


def _arrow(ax, pose, color):
    x, y, yaw = pose
    return ax.annotate(
        "", xy=(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw)),
        xytext=(x, y),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=2))


def run_gui(node, view, follow, interval):
    """Matplotlib render loop on the main thread; ROS spins in a side thread."""
    show_truth = node.show_truth
    plt.ion()
    if show_truth:
        fig, (ax, axe) = plt.subplots(1, 2, figsize=(13, 6.5))
    else:
        fig, ax = plt.subplots(figsize=(7.5, 7.5))
        axe = None

    # One line + dot + (rebuilt each frame) arrow per estimate, plus the truth.
    est_artists = []   # [(line, dot, err_line, arrow_holder)]
    for s in node.estimates:
        (line,) = ax.plot([], [], "-", color=s.color, lw=2, label=s.label)
        (dot,) = ax.plot([], [], "o", color=s.color, ms=9)
        est_artists.append({"line": line, "dot": dot, "arrow": None,
                            "err": None})
    truth_line = truth_dot = None
    truth_arrow = None
    if show_truth:
        (truth_line,) = ax.plot([], [], "-", color=node.truth.color, lw=2,
                                label=node.truth.label)
        (truth_dot,) = ax.plot([], [], "o", color=node.truth.color, ms=9)

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper left")
    # Out-of-view banner (hidden until something leaves the clamped window).
    oob = ax.text(0.5, 0.97, "", transform=ax.transAxes, ha="center", va="top",
                  color="white", fontsize=10, weight="bold",
                  bbox=dict(boxstyle="round", fc="#c1121f", ec="none", alpha=0.9))
    oob.set_visible(False)

    if axe is not None:
        for a, s in zip(est_artists, node.estimates):
            (a["err"],) = axe.plot([], [], "-", color=s.color, lw=1.8,
                                   label=s.label)
        axe.set_xlabel("time [s]")
        axe.set_ylabel("position error ‖est − truth‖ [m]")
        axe.grid(True, alpha=0.3)
        axe.legend(loc="upper left")
        axe.set_title("Localization error vs. time")

    max_err = [0.0] * len(est_artists)

    def clamp_view(cx, cy):
        """Fixed +/-view window, optionally re-centred on the truth (follow)."""
        ax.set_xlim(cx - view, cx + view)
        ax.set_ylim(cy - view, cy + view)

    while rclpy.ok() and plt.fignum_exists(fig.number):
        s = node.snapshot()
        ests, truth = s["estimates"], s["truth"]

        for a, e in zip(est_artists, ests):
            a["line"].set_data(e["xs"], e["ys"])
            if e["pose"]:
                a["dot"].set_data([e["pose"][0]], [e["pose"][1]])
                if a["arrow"]:
                    a["arrow"].remove()
                a["arrow"] = _arrow(ax, e["pose"], e["color"])
        if show_truth:
            truth_line.set_data(truth["xs"], truth["ys"])
            if truth["pose"]:
                truth_dot.set_data([truth["pose"][0]], [truth["pose"][1]])
                if truth_arrow:
                    truth_arrow.remove()
                truth_arrow = _arrow(ax, truth["pose"], truth["color"])

        # --- clamp the XY view; centre on truth if following, else on origin.
        if follow and truth["pose"]:
            cx, cy = truth["pose"][0], truth["pose"][1]
        elif follow and ests[0]["pose"]:
            cx, cy = ests[0]["pose"][0], ests[0]["pose"][1]
        else:
            cx, cy = 0.0, 0.0
        clamp_view(cx, cy)

        # --- out-of-view detection (current markers outside the window).
        offenders = []
        marks = [(e["label"], e["pose"]) for e in ests]
        if show_truth:
            marks.append(("truth", truth["pose"]))
        for name, pose in marks:
            if pose and (abs(pose[0] - cx) > view or abs(pose[1] - cy) > view):
                offenders.append(f"{name} @ ({pose[0]:.0f}, {pose[1]:.0f}) m")
        if offenders:
            oob.set_text("OUT OF VIEW — " + "; ".join(offenders))
            oob.set_visible(True)
        else:
            oob.set_visible(False)

        # --- title + error panel.
        if show_truth and truth["pose"]:
            parts = []
            redraw_err = False
            for i, (a, e) in enumerate(zip(est_artists, ests)):
                if not e["pose"]:
                    continue
                err = math.hypot(e["pose"][0] - truth["pose"][0],
                                 e["pose"][1] - truth["pose"][1])
                max_err[i] = max(max_err[i], err)
                parts.append(f"{e['short']} {err:.3f} m (max {max_err[i]:.3f})")
                if a["err"] is not None and e["err_t"]:
                    a["err"].set_data(e["err_t"], e["err_v"])
                    redraw_err = True
            if parts:
                ax.set_title("error vs truth — " + " | ".join(parts), fontsize=9)
            if redraw_err:
                axe.relim()
                axe.autoscale_view()
        elif ests[0]["pose"]:
            p = ests[0]["pose"]
            ax.set_title(f"Go2 ESKF estimate — x={p[0]:.2f} y={p[1]:.2f} m")

        fig.canvas.draw_idle()
        plt.pause(interval)  # pump the GUI event loop (default ~10 Hz)

    plt.ioff()


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--estimate-topic", default="/eskf/odom")
    parser.add_argument("--slip-topic", default="/eskf_slip/odom",
                        help="Odometry from the slip-adaptive ESKF arm "
                             "(eskf.launch.py slip:=true).")
    parser.add_argument("--truth-topic", default="/ground_truth/odom")
    parser.add_argument("--no-truth", action="store_true",
                        help="Plot only the estimates (no ground-truth overlay).")
    parser.add_argument("--no-slip", action="store_true",
                        help="Don't plot the slip-adaptive arm even if it is up.")
    parser.add_argument("--wait-slip", type=float, default=0.0,
                        help="Seconds to wait for the slip topic to appear "
                             "before giving up on it (0 = don't wait; the curve "
                             "is added anyway and simply stays empty).")
    parser.add_argument("--view", type=float, default=10.0,
                        help="Half-width of the clamped XY window in metres "
                             "(default 10 -> a 20x20 m box).")
    parser.add_argument("--follow", action="store_true",
                        help="Re-centre the XY window on the robot instead of "
                             "the world origin (keeps it in frame while driving).")
    parser.add_argument("--interval", type=float, default=0.1,
                        help="Redraw period in seconds (default 0.1 = 10 Hz). "
                             "Raise it (e.g. 0.2) to lower CPU/GPU load.")
    args = parser.parse_args(argv[1:])

    # Force a GUI-capable backend; error clearly if only Agg is available.
    if matplotlib.get_backend().lower() == "agg":
        for be in ("TkAgg", "Qt5Agg", "GTK3Agg"):
            try:
                matplotlib.use(be, force=True)
                break
            except Exception:
                continue

    rclpy.init()

    show_slip = not args.no_slip
    if show_slip and args.wait_slip > 0.0:
        # Optional: drop the third curve entirely when nobody is publishing it,
        # so the legend doesn't advertise an arm that isn't running.
        probe = rclpy.create_node("eskf_plot_slip_probe")
        deadline = time.monotonic() + args.wait_slip
        topic = args.slip_topic.lstrip("/")
        while time.monotonic() < deadline:
            names = [n.lstrip("/") for n, _ in probe.get_topic_names_and_types()]
            if topic in names:
                break
            time.sleep(0.2)
        else:
            show_slip = False
            probe.get_logger().warn(
                f"{args.slip_topic} not published after {args.wait_slip:.0f}s — "
                "plotting without the slip-adaptive curve.")
        probe.destroy_node()

    node = TrajectoryPlotter(args.estimate_topic, args.slip_topic,
                             args.truth_topic,
                             show_truth=not args.no_truth,
                             show_slip=show_slip)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_gui(node, view=args.view, follow=args.follow, interval=args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
