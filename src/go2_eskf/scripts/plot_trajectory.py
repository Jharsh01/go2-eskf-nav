#!/usr/bin/env python3
"""Live trajectory + error plot: ESKF estimates vs. Gazebo ground truth.

Up to three panels, updated in real time:

  * Left  — top-down XY paths of every estimate and the Gazebo ground truth,
            with current-position dots and heading arrows. The view is *clamped*
            to a fixed window (default +/-10 m) so a filter blow-up doesn't zoom
            the whole plot out to a useless scale; when a path leaves the window
            an "OUT OF VIEW" banner appears with the offending position.
  * Middle — position error (‖estimate - truth‖) vs. time for each estimate, so
            you can see exactly when divergence starts and whether the
            slip-adaptive arm actually beats the baseline.
  * Right — the RAW leg-odometry signal itself: CHAMP's body twist (v_x, ω_z)
            from /odom/raw against the ground-truth twist, plus the fraction of
            samples that came through degenerate (all-zero).

Subscribes to up to four ``nav_msgs/Odometry`` topics:
  estimate  (default /eskf/odom)         — baseline go2_eskf_node (fixed R_leg)
  slip      (default /eskf_slip/odom)    — slip-adaptive arm, i.e. the second
                                            go2_eskf_node started by
                                            ``eskf.launch.py slip:=true``
  truth     (default /ground_truth/odom) — bridged gz OdometryPublisher
                                            (see ground_truth.launch.py)
  leg       (default /odom/raw)          — CHAMP leg odometry, the ESKF's only
                                            velocity measurement

/odom/raw's POSE is meaningless (a vel_dt unit bug in CHAMP's
state_estimation.cpp), so the XY curve for it is NOT that pose: it is the twist
dead-reckoned here (∫ Rz(ψ)·v dt, ψ = ∫ ω_z dt), anchored on the ground-truth
pose at the first sample. That is the honest "leg odometry alone" baseline — the
open-loop track the ESKF exists to beat — and the gap between it and the ESKF
curves is what the IMU and the covariance model are buying you. Note it is drawn
UNSCALED by default: the filter multiplies linear velocity by leg_odom_scale
(1.111 in eskf_params.yaml) to undo CHAMP's odom_scaler, so the dead-reckon runs
~10% short of the estimates on purpose. Pass --leg-scale to match the filter.

The slip curve is drawn only if that topic is actually publishing (it is
skipped with a log line if the arm isn't running), so the old two-curve
behaviour still works unchanged.

The window is a 2x2 grid: XY trajectory | error vs time on top, yaw vs time |
slip score below. The yaw panel draws ground truth, every estimate and the raw
magnetometer heading (/eskf/mag_heading, once it publishes), unwrapped in degrees. The slip panel plots /eskf_slip/slip_score (in [0, 1]) against
time, with the R_leg inflation it implies, (1 + lambda*s)^2, on a second axis. A
flat score means the model is de-weighting leg odometry everywhere rather than
detecting slip — the panel makes that visible while the run is still going.
Panels whose data is switched off (--no-truth, --no-leg, --no-slip,
--no-slip-score) are left out and the grid shrinks to fit.

Typical use (see run_go2_teleop.sh --square for the whole workflow):

  ros2 launch unitree_go2_sim unitree_go2_launch.py
  ros2 launch go2_eskf ground_truth.launch.py
  ros2 launch go2_eskf eskf.launch.py use_sim_time:=true slip:=true
  ros2 run go2_eskf plot_trajectory.py --ros-args -p use_sim_time:=true

CLI (after a ``--`` separator, since --ros-args consumes ROS flags):
  ros2 run go2_eskf plot_trajectory.py --ros-args -p use_sim_time:=true -- \
      --view 20 --no-truth --no-slip --no-leg
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
from std_msgs.msg import Float64
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def stamp_sec(stamp):
    """Header stamp in seconds (sim time when use_sim_time is set)."""
    return stamp.sec + stamp.nanosec * 1e-9


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
        # Heading history for the yaw panel, UNWRAPPED (so a full turn reads as a
        # staircase rather than a jump at +-180 deg) and in degrees.
        self.yaw_t, self.yaw_deg = [], []
        self._yaw_last = None

    def add_yaw(self, t, yaw):
        """Append a heading sample, unwrapping it against the previous one."""
        if self._yaw_last is None:
            u = math.degrees(yaw)
        else:
            u = self.yaw_deg[-1] + math.degrees(
                math.atan2(math.sin(yaw - self._yaw_last),
                           math.cos(yaw - self._yaw_last)))
        self._yaw_last = yaw
        self.yaw_t.append(t)
        self.yaw_deg.append(u)

    def snapshot(self):
        return {
            "xs": list(self.xs), "ys": list(self.ys), "pose": self.pose,
            "err_t": list(self.err_t), "err_v": list(self.err_v),
            "yaw_t": list(self.yaw_t), "yaw_deg": list(self.yaw_deg),
            "key": self.key, "label": self.label, "short": self.short,
            "color": self.color,
        }


class TrajectoryPlotter(Node):
    def __init__(self, est_topic, slip_topic, truth_topic, leg_topic,
                 show_truth, show_slip, show_leg, leg_scale=1.0,
                 leg_anchor_wait=5.0, score_topic=None,
                 mag_topic="/eskf/mag_heading"):
        super().__init__("eskf_trajectory_plotter")
        self.show_truth = show_truth
        self.show_leg = show_leg
        self.leg_scale = leg_scale
        self.leg_anchor_wait = leg_anchor_wait
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
        # Leg odometry, dead-reckoned. Listed with the estimates so it gets the
        # same XY curve and the same error-vs-truth trace for free.
        self.leg = Series("leg", "leg odom, dead-reckoned", "#6a4c93",
                          short="leg DR")
        self._leg_t = None        # previous sample time, for dt
        self._leg_src = None      # which clock that time came from
        self._leg_seen = None     # wall time of the first sample (anchor timeout)
        if show_leg:
            self.estimates.append(self.leg)
        # Slip score of the slip-adaptive arm, on the same elapsed-time axis.
        self.show_score = score_topic is not None
        # Raw magnetometer heading (use_mag), drawn on the yaw panel only once it
        # actually publishes — so runs without --mag look exactly as before.
        self.mag = Series("mag", "magnetometer heading (raw)", "#2a9d8f",
                          short="mag")
        self.score_t, self.score_v = [], []

        self.create_subscription(
            Odometry, est_topic,
            lambda m: self._store(m, self.estimates[0]), qos)
        self.get_logger().info(f"Estimate topic:      {est_topic}")
        if show_slip:
            self.create_subscription(
                Odometry, slip_topic,
                lambda m: self._store(m, self.estimates[1]), qos)
            self.get_logger().info(f"Slip-adaptive topic: {slip_topic}")
        if mag_topic:
            self.create_subscription(Float64, mag_topic, self._store_mag, qos)
        if self.show_score:
            self.create_subscription(Float64, score_topic, self._store_score, qos)
            self.get_logger().info(f"Slip-score topic:    {score_topic}")
        if show_truth:
            self.create_subscription(
                Odometry, truth_topic,
                lambda m: self._store(m, self.truth), qos)
            self.get_logger().info(f"Ground-truth topic:  {truth_topic}")
        if show_leg:
            self.create_subscription(Odometry, leg_topic, self._store_leg, qos)
            self.get_logger().info(
                f"Leg-odometry topic:  {leg_topic} (twist dead-reckoned, "
                f"scale {leg_scale:g})")

    def _store(self, msg, series):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            self._append(series, p.x, p.y, yaw)

    def _store_mag(self, msg):
        with self._lock:
            self.mag.add_yaw(self._elapsed(), msg.data)

    def _store_score(self, msg):
        with self._lock:
            self.score_t.append(self._elapsed())
            self.score_v.append(msg.data)

    def _store_leg(self, msg):
        """Dead-reckon /odom/raw's TWIST (its pose is a known-broken field).

        Integrates v_body and omega_z with midpoint heading, starting from the
        ground-truth pose at the first sample so the curve is directly
        comparable with the estimates.
        """
        tw = msg.twist.twist
        vx, vy, wz = tw.linear.x, tw.linear.y, tw.angular.z
        t, src = stamp_sec(msg.header.stamp), "stamp"
        now = time.monotonic()
        if t <= 0.0:            # unstamped publisher: fall back to the wall clock
            t, src = now, "wall"
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            if self.leg.pose is None:
                # Anchor on ground truth if it is coming; otherwise (or if it never
                # shows up) start at the origin rather than dropping the curve.
                if self._leg_seen is None:
                    self._leg_seen = now
                anchor = self.truth.pose
                if anchor is None:
                    if (self.show_truth
                            and now - self._leg_seen < self.leg_anchor_wait):
                        self._leg_t, self._leg_src = t, src
                        return
                    anchor = (0.0, 0.0, 0.0)
                self._leg_t, self._leg_src = t, src
                self._append(self.leg, *anchor)
                return

            # Never difference two different clocks: a stream that starts unstamped
            # and then gets real stamps (or a /clock that shows up late) would
            # otherwise produce one huge or negative dt.
            dt = t - self._leg_t if src == self._leg_src else 0.0
            self._leg_t, self._leg_src = t, src
            if not 0.0 < dt < 0.5:         # a stall, a clock jump, or a source change
                return
            x, y, yaw = self.leg.pose
            mid = yaw + 0.5 * wz * dt      # midpoint heading over the step
            c, s = math.cos(mid), math.sin(mid)
            x += (vx * c - vy * s) * self.leg_scale * dt
            y += (vx * s + vy * c) * self.leg_scale * dt
            self._append(self.leg, x, y, yaw + wz * dt)

    def _elapsed(self):
        """Seconds since the first sample of any series (caller holds the lock)."""
        if self._t0 is None:
            self._t0 = time.monotonic()
        return time.monotonic() - self._t0

    def _append(self, series, x, y, yaw):
        """Add one pose to a series (caller holds the lock)."""
        series.xs.append(x)
        series.ys.append(y)
        series.pose = (x, y, yaw)
        series.add_yaw(self._elapsed(), yaw)
        # Log an error sample whenever both sides are known (driven off each
        # estimate's own stream, so every arm gets one point per update).
        if (series is not self.truth and self.show_truth
                and self.truth.pose is not None):
            err = math.hypot(x - self.truth.pose[0], y - self.truth.pose[1])
            series.err_t.append(self._elapsed())
            series.err_v.append(err)

    def snapshot(self):
        """Thread-safe copy of everything the plot needs this frame."""
        with self._lock:
            return {
                "estimates": [s.snapshot() for s in self.estimates],
                "truth": self.truth.snapshot(),
                "leg": self.leg.snapshot() if self.show_leg else None,
                "mag": self.mag.snapshot(),
                "score_t": list(self.score_t),
                "score_v": list(self.score_v),
            }


def _arrow(ax, pose, color):
    x, y, yaw = pose
    return ax.annotate(
        "", xy=(x + 0.35 * math.cos(yaw), y + 0.35 * math.sin(yaw)),
        xytext=(x, y),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=2))


def _make_score_panel(ax, lam):
    """Slip score vs time, with the R_leg inflation it implies."""
    (raw,) = ax.plot([], [], "-", color="#f0a202", lw=1.0, alpha=0.55,
                     label="slip score")
    (smooth,) = ax.plot([], [], "-", color="#b86b00", lw=2.2,
                        label="2 s moving average")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("slip score  (0 = trust leg odom, 1 = max slip)")
    ax.set_ylim(-0.02, 1.02)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower left", fontsize=8)
    # Second axis: what the score DOES — R_leg is scaled by (1 + lambda*s)^2
    # (SlipModel::adaptLegCovariance). Same data, relabelled, so it stays aligned.
    if lam > 0.0:
        axr = ax.secondary_yaxis("right", functions=(
            lambda sc: (1.0 + lam * sc) ** 2,
            lambda r: (abs(r) ** 0.5 - 1.0) / lam))
        axr.set_ylabel(f"R_leg inflation  (1 + {lam:g}·s)²")
    ax.set_title("waiting for /eskf_slip/slip_score …", fontsize=9)
    return ax, raw, smooth


def _update_score_panel(panel, t, v):
    ax, raw, smooth = panel
    if not t:
        return
    raw.set_data(t, v)
    # Trailing 2 s mean, O(n) with a running sum over a sliding window.
    avg, j, acc = [], 0, 0.0
    for i, ti in enumerate(t):
        acc += v[i]
        while t[j] < ti - 2.0:
            acc -= v[j]
            j += 1
        avg.append(acc / (i - j + 1))
    smooth.set_data(t, avg)
    ax.set_xlim(0.0, max(t[-1], 10.0))
    n = len(v)
    mean = sum(v) / n
    ax.set_title(
        f"slip score now {v[-1]:.3f} | mean {mean:.3f} | range "
        f"{min(v):.3f}..{max(v):.3f} | {n} samples", fontsize=9)


def run_gui(node, view, follow, interval, score_lambda=1.0):
    """Matplotlib render loop on the main thread; ROS spins in a side thread."""
    show_truth = node.show_truth
    show_leg = node.show_leg
    plt.ion()
    # Panels, in reading order: XY always; error only against truth; yaw always;
    # slip score only with the slip arm. Four fill a 2x2 grid; fewer shrink it
    # (2 -> 1x2, 3 -> 2x2 with one blank).
    wanted = ["xy"] + (["err"] if show_truth else []) + ["yaw"] + \
        (["score"] if node.show_score else [])
    n = len(wanted)
    rows, cols = (1, n) if n <= 2 else (2, 2)
    fig, grid = plt.subplots(
        rows, cols, figsize={(1, 1): (7.5, 7.5), (1, 2): (13, 6.5),
                             (2, 2): (14, 11)}[(rows, cols)], squeeze=False)
    flat = [a for r in grid for a in r]
    for extra in flat[n:]:
        extra.set_visible(False)
    panels = dict(zip(wanted, flat))
    ax = panels["xy"]
    axe = panels.get("err")
    axy = panels["yaw"]
    axs_ = panels.get("score")

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

    # Heading vs time: truth, every estimate (incl. leg-odom dead-reckoning), and
    # the raw magnetometer heading. Unwrapped degrees, so each 90 deg corner of the
    # square is a step; the gap between a curve and truth IS the heading error,
    # which drives position error ~1:1 (skills.md §0).
    yaw_lines = {}
    for srs, style, lw, alpha in (
            [(node.truth, "-", 2.4, 0.9)] if show_truth else []) + \
            [(e, "-", 1.5, 1.0) for e in node.estimates] + \
            [(node.mag, ".", 1.0, 0.35)]:
        (yaw_lines[srs.key],) = axy.plot([], [], style, color=srs.color, lw=lw,
                                         ms=2.5, alpha=alpha, label=srs.label)
    axy.set_xlabel("time [s]")
    axy.set_ylabel("yaw [deg], unwrapped")
    axy.grid(True, alpha=0.3)
    axy.legend(loc="upper left", fontsize=8)
    axy.set_title("Yaw vs. time", fontsize=9)

    max_err = [0.0] * len(est_artists)
    score_panel = _make_score_panel(axs_, score_lambda) if axs_ is not None else None
    fig.tight_layout()

    def clamp_view(cx, cy):
        """Fixed +/-view window, optionally re-centred on the truth (follow)."""
        ax.set_xlim(cx - view, cx + view)
        ax.set_ylim(cy - view, cy + view)

    while rclpy.ok() and plt.fignum_exists(fig.number):
        s = node.snapshot()
        ests, truth, leg = s["estimates"], s["truth"], s["leg"]

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
                # Wrapped two-per-line: with three arms a single line runs off the axes.
                lines = [" | ".join(parts[i:i + 2]) for i in range(0, len(parts), 2)]
                ax.set_title("error vs truth — " + "\n".join(lines), fontsize=9)
            if redraw_err:
                axe.relim()
                axe.autoscale_view()
        elif ests[0]["pose"]:
            p = ests[0]["pose"]
            ax.set_title(f"Go2 ESKF estimate — x={p[0]:.2f} y={p[1]:.2f} m")

        # --- yaw panel.
        mag = s["mag"]
        drew = False
        for srs in [truth] + ests + [mag]:
            line = yaw_lines.get(srs["key"])
            if line is not None and srs["yaw_t"]:
                line.set_data(srs["yaw_t"], srs["yaw_deg"])
                drew = True
        if drew:
            axy.relim()
            axy.autoscale_view()
        if show_truth and truth["yaw_deg"]:
            gt = truth["yaw_deg"][-1]
            parts = [f"{e['short']} {e['yaw_deg'][-1] - gt:+.1f}°"
                     for e in ests + [mag] if e["yaw_deg"]]
            axy.set_title("Yaw vs. time — error now: " + " | ".join(parts),
                          fontsize=9)

        # --- slip-score panel.
        if score_panel is not None:
            _update_score_panel(score_panel, s["score_t"], s["score_v"])

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
    parser.add_argument("--score-topic", default="/eskf_slip/slip_score",
                        help="Slip score of the slip-adaptive arm (bottom-right "
                             "panel).")
    parser.add_argument("--no-slip-score", action="store_true",
                        help="Drop the slip-score panel.")
    parser.add_argument("--slip-lambda", type=float, default=1.0,
                        help="slip_lambda from eskf_params.yaml, used only to "
                             "label the R_leg inflation axis (default 1.0).")
    parser.add_argument("--truth-topic", default="/ground_truth/odom")
    parser.add_argument("--leg-topic", default="/odom/raw",
                        help="CHAMP leg odometry. Only its TWIST is used — its "
                             "pose field is a known-broken value.")
    parser.add_argument("--no-truth", action="store_true",
                        help="Plot only the estimates (no ground-truth overlay).")
    parser.add_argument("--no-slip", action="store_true",
                        help="Don't plot the slip-adaptive arm even if it is up.")
    parser.add_argument("--no-leg", action="store_true",
                        help="Drop the dead-reckoned leg-odometry curve and the "
                             "raw-twist panel.")
    parser.add_argument("--leg-scale", type=float, default=1.0,
                        help="Multiply leg linear velocity by this before "
                             "dead-reckoning (default 1.0 = raw). The filter uses "
                             "leg_odom_scale from eskf_params.yaml (1.111) to undo "
                             "CHAMP's odom_scaler; pass that to compare like for "
                             "like.")
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
                             args.truth_topic, args.leg_topic,
                             show_truth=not args.no_truth,
                             show_slip=show_slip,
                             show_leg=not args.no_leg,
                             leg_scale=args.leg_scale,
                             score_topic=(args.score_topic
                                          if show_slip and not args.no_slip_score
                                          else None))

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        run_gui(node, view=args.view, follow=args.follow, interval=args.interval,
                score_lambda=args.slip_lambda)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
