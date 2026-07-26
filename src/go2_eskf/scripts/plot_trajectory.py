#!/usr/bin/env python3
"""Live trajectory + error plot: ESKF estimate vs. Gazebo ground truth.

Two panels, updated in real time:

  * Left  — top-down XY paths of the ESKF estimate and the Gazebo ground truth,
            with current-position dots and heading arrows. The view is *clamped*
            to a fixed window (default +/-10 m) so a filter blow-up doesn't zoom
            the whole plot out to a useless scale; when either path leaves the
            window an "OUT OF VIEW" banner appears with the offending position.
  * Right — position error (‖estimate - truth‖) vs. time, so you can see exactly
            when divergence starts and whether GPS reins it back in.

Subscribes to two ``nav_msgs/Odometry`` topics:
  estimate  (default /eskf/odom)         — output of go2_eskf_node
  truth     (default /ground_truth/odom) — bridged gz OdometryPublisher
                                            (see ground_truth.launch.py)

Typical use (see run_go2_teleop.sh --plot for the whole workflow):

  ros2 launch unitree_go2_sim unitree_go2_launch.py
  ros2 launch go2_eskf ground_truth.launch.py
  ros2 launch go2_eskf eskf.launch.py use_sim_time:=true use_gps:=true
  ros2 run go2_eskf plot_trajectory.py --ros-args -p use_sim_time:=true

CLI (after a ``--`` separator, since --ros-args consumes ROS flags):
  ros2 run go2_eskf plot_trajectory.py --ros-args -p use_sim_time:=true -- \
      --view 20 --no-truth
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


class TrajectoryPlotter(Node):
    def __init__(self, est_topic, truth_topic, show_truth):
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
        self.est_xy = ([], [])       # (xs, ys)
        self.truth_xy = ([], [])
        self.est_pose = None         # (x, y, yaw)
        self.truth_pose = None
        self.err_t = []              # seconds since first message
        self.err_v = []              # position error [m]

        self.create_subscription(Odometry, est_topic, self._est_cb, qos)
        self.get_logger().info(f"Estimate topic:     {est_topic}")
        if show_truth:
            self.create_subscription(Odometry, truth_topic, self._truth_cb, qos)
            self.get_logger().info(f"Ground-truth topic: {truth_topic}")

    def _est_cb(self, msg):
        self._store(msg, self.est_xy, "est")

    def _truth_cb(self, msg):
        self._store(msg, self.truth_xy, "truth")

    def _store(self, msg, xy, which):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            if self._t0 is None:
                self._t0 = time.monotonic()
            xy[0].append(p.x)
            xy[1].append(p.y)
            if which == "est":
                self.est_pose = (p.x, p.y, yaw)
            else:
                self.truth_pose = (p.x, p.y, yaw)
            # Log an error sample whenever both sides are known (drive off the
            # estimate stream so the series has one point per estimate update).
            if which == "est" and self.show_truth and self.truth_pose is not None:
                err = math.hypot(p.x - self.truth_pose[0],
                                 p.y - self.truth_pose[1])
                self.err_t.append(time.monotonic() - self._t0)
                self.err_v.append(err)

    def snapshot(self):
        """Thread-safe copy of everything the plot needs this frame."""
        with self._lock:
            return {
                "ex": list(self.est_xy[0]), "ey": list(self.est_xy[1]),
                "epose": self.est_pose,
                "tx": list(self.truth_xy[0]), "ty": list(self.truth_xy[1]),
                "tpose": self.truth_pose,
                "err_t": list(self.err_t), "err_v": list(self.err_v),
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

    EST_C, TRU_C = "#d1495b", "#2e86ab"
    (est_line,) = ax.plot([], [], "-", color=EST_C, lw=2, label="ESKF estimate")
    (est_dot,) = ax.plot([], [], "o", color=EST_C, ms=9)
    est_arrow = None
    truth_line = truth_dot = truth_arrow = None
    if show_truth:
        (truth_line,) = ax.plot([], [], "-", color=TRU_C, lw=2,
                                label="Gazebo ground truth")
        (truth_dot,) = ax.plot([], [], "o", color=TRU_C, ms=9)

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

    err_line = None
    if axe is not None:
        (err_line,) = axe.plot([], [], "-", color="#6a4c93", lw=1.8)
        axe.set_xlabel("time [s]")
        axe.set_ylabel("position error ‖est − truth‖ [m]")
        axe.grid(True, alpha=0.3)
        axe.set_title("Localization error vs. time")

    max_err = 0.0

    def clamp_view(cx, cy):
        """Fixed +/-view window, optionally re-centred on the truth (follow)."""
        ax.set_xlim(cx - view, cx + view)
        ax.set_ylim(cy - view, cy + view)

    while rclpy.ok() and plt.fignum_exists(fig.number):
        s = node.snapshot()
        ex, ey, epose = s["ex"], s["ey"], s["epose"]
        tx, ty, tpose = s["tx"], s["ty"], s["tpose"]

        est_line.set_data(ex, ey)
        if epose:
            est_dot.set_data([epose[0]], [epose[1]])
            if est_arrow:
                est_arrow.remove()
            est_arrow = _arrow(ax, epose, EST_C)
        if show_truth:
            truth_line.set_data(tx, ty)
            if tpose:
                truth_dot.set_data([tpose[0]], [tpose[1]])
                if truth_arrow:
                    truth_arrow.remove()
                truth_arrow = _arrow(ax, tpose, TRU_C)

        # --- clamp the XY view; centre on truth if following, else on origin.
        if follow and tpose:
            cx, cy = tpose[0], tpose[1]
        elif follow and epose:
            cx, cy = epose[0], epose[1]
        else:
            cx, cy = 0.0, 0.0
        clamp_view(cx, cy)

        # --- out-of-view detection (current markers outside the window).
        offenders = []
        for name, pose in (("estimate", epose), ("truth", tpose)):
            if pose and (abs(pose[0] - cx) > view or abs(pose[1] - cy) > view):
                offenders.append(f"{name} @ ({pose[0]:.0f}, {pose[1]:.0f}) m")
        if offenders:
            oob.set_text("OUT OF VIEW — " + "; ".join(offenders))
            oob.set_visible(True)
        else:
            oob.set_visible(False)

        # --- title + error panel.
        if show_truth and epose and tpose:
            err = math.hypot(epose[0] - tpose[0], epose[1] - tpose[1])
            max_err = max(max_err, err)
            ax.set_title(f"Go2 localization — error {err:.3f} m (max {max_err:.3f} m)")
            if err_line is not None and s["err_t"]:
                err_line.set_data(s["err_t"], s["err_v"])
                axe.relim()
                axe.autoscale_view()
        elif epose:
            ax.set_title(f"Go2 ESKF estimate — x={epose[0]:.2f} y={epose[1]:.2f} m")

        fig.canvas.draw_idle()
        plt.pause(interval)  # pump the GUI event loop (default ~10 Hz)

    plt.ioff()


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--estimate-topic", default="/eskf/odom")
    parser.add_argument("--truth-topic", default="/ground_truth/odom")
    parser.add_argument("--no-truth", action="store_true",
                        help="Plot only the estimate (no ground-truth overlay).")
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
    node = TrajectoryPlotter(args.estimate_topic, args.truth_topic,
                             show_truth=not args.no_truth)

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
