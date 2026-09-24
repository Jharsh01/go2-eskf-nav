#!/usr/bin/env python3
"""Autonomous square drift test for the go2_eskf estimator.

Drives the Go2 around a square (default 5 m sides) by publishing /cmd_vel,
closing the loop on GROUND TRUTH pose (/ground_truth/odom) — so the true path
is a clean square regardless of estimator quality, and the gap between the
ESKF estimate and truth is pure estimator drift. Replaces manual teleop for
repeatable drift evaluation.

Per corner it prints truth vs estimate vs error; at the end it prints a drift
summary (final error, max error, error as % of the 4*side distance travelled).

Side length matters for more than duration: on terrain.sdf the four `mu=0.3`
patches sit at the midpoints of the **10 m** square's legs — (5,0), (10,-5),
(5,-10), (0,-5) — so a 5 m square passes over only two of them, and at its
CORNERS rather than mid-leg. If the friction patches are the point of the run,
edit SQUARE / DEFAULT_PATCHES in scripts/make_terrain_world.py to match the side
being driven and regenerate the world (that also moves the flat start pad's
relief blend, so re-read the slope numbers it prints).

Needs: the sim (CHAMP walking), ground_truth.launch.py, and the ESKF running.
Respects the gait limits in gait.yaml (vx<=0.3, wz<=0.5).

  ros2 run go2_eskf square_test.py --ros-args -p use_sim_time:=true
  # options after --:  --side 5 --speed 0.25 --ccw
"""

import argparse
import collections
import math
import sys

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class SquareTest(Node):
    # Gait limits (gait.yaml): max vx 0.3, max wz 0.5 — stay under them.
    V_MAX = 0.25
    W_MAX = 0.4
    ARRIVE_TOL = 0.25       # [m] corner acceptance radius
    HEAD_TOL = 0.10         # [rad] heading good enough to start driving
    HEAD_REDO = 0.60        # [rad] heading bad enough to stop and re-turn
    # Base height below which the Go2 is NOT standing, as an ABSOLUTE world z. It
    # spawns at z=0.375 and stands at ~0.30; lying on its belly it reads ~0.06. If
    # ros2_control never activated joint_group_effort_controller the robot collapses,
    # and driving a prone robot silently produces a "frozen at origin" trace that
    # looks like an estimator bug. Refuse to drive until it is actually standing.
    #
    # This test is only valid BEFORE the robot has stood: every world spawns it on
    # ground at elevation 0 (terrain.sdf has a deliberately flat start pad), so the
    # absolute comparison is sound there. It is NOT a valid fall detector afterwards
    # on uneven ground — a robot collapsed at terrain elevation 0.4 m reads z~0.46,
    # far above this threshold, so the fall goes unnoticed and the test keeps driving
    # a prone robot whose feet scrabble in place. Leg odometry then reports forward
    # motion that is not happening and the estimate runs away by metres. MEASURED on
    # terrain.sdf: a fall at (9,-3) produced 50 s of phantom motion and a bogus
    # 8.8 m "drift" figure. Post-standing falls are caught by TILT_MAX / stall below.
    STAND_Z = 0.18
    # ...minus the crouch terrain_adapt.py (--adapt) is COMMANDING right now on
    # /body_pose (position.z is a delta on nominal_height, negative = lower). Without
    # this the check contradicts the adapter by construction: nominal 0.225 - its
    # max_crouch 0.06 = 0.165 < 0.18. MEASURED 2026-09-23: a spurious 3.3 cm crouch
    # plus normal trot bob put an UPRIGHT robot at z=0.180 and aborted the run as a
    # "fall". With no adapter nothing publishes /body_pose, the credit stays 0, and
    # stock runs keep exactly the 0.18 they were validated with. The credit is capped
    # so a rogue publisher cannot switch the check off (real belly flops: z~0.06).
    MAX_CROUCH_CREDIT = 0.08
    # Body tilt beyond which it has certainly gone over. Must clear the terrain
    # slope a STANDING robot legitimately sits at: terrain.sdf reaches 19.5 deg at
    # the default --relief 0.7 and 38.9 deg at 1.6, hence 60.
    TILT_MAX = math.radians(60.0)
    # Stall = we are commanding motion but ground truth is not moving, in EITHER
    # position or heading. Catches a belly flop (which can be perfectly level, so
    # TILT_MAX misses it) and getting wedged on terrain. At 0.25 m/s the window
    # should cover ~1.5 m, so 0.15 m is a 10x margin against a slow crawl.
    STALL_WIN = 6.0         # [s] look-back window
    STALL_DIST = 0.15       # [m] displacement below this counts as no progress
    STALL_YAW = 0.15        # [rad] yaw change below this counts as no progress
    STALL_CMD = 0.02        # [m/s, rad/s] commanded motion above this counts as "asked to move"

    # --- Command shaping (2026-09-23). The 23:20 run stumbled ~2 s after /cmd_vel
    # stepped 0 -> 0.25 m/s in one tick while cmd_wz flickered 0 -> 0.25 -> 0.04 ->
    # 0.17 in 0.6 s, the steering chasing a truth yaw that wobbles ~+-10 deg with each
    # trot step. So: rate-limit both channels, and steer off a low-passed heading.
    # Aborts bypass the limiter (they publish zero directly). --no-shaping = old.
    # Only SPEEDING UP is limited. Slowing down, stopping and reversing (via zero)
    # are immediate: the 23:28 run showed a symmetric limit is harmful — told to stop
    # and turn in place at a missed corner, the robot kept walking for 2 s and needed
    # 2 s more to reverse its turn, looping round the corner until the stall detector
    # fired (skills.md §0).
    ACCEL_MAX = 0.125       # [m/s^2]   0 -> 0.25 m/s in 2 s
    ALPHA_MAX = 0.4         # [rad/s^2] 0 -> W_MAX in 1 s
    YAW_TAU = 0.25          # [s] heading low-pass for steering (~0.1 rad lag at W_MAX)

    # --- Stumble recovery (2026-09-23). A fall condition (height below stand_z() or
    # tilt above TILT_MAX) used to abort on the FIRST sample. The 23:20 robot buckled
    # to z=0.139 for one sample, bounced, rolled 48.7 deg, and was standing upright
    # 2 s later — a run thrown away over a recoverable stumble. Now a fall condition
    # PAUSES the square (zero command, so the gait can re-plant) and it resumes, from a
    # fresh ramp, once the robot has been settled for RECOVER_HOLD. It aborts only if
    # the fall condition persists FALL_HOLD continuously (a belly flop or a flip stays
    # down) or it has not settled within RECOVER_MAX.
    FALL_HOLD = 0.5         # [s]
    SETTLE_TILT = math.radians(25.0)   # settled = standing AND tilt below this
    RECOVER_HOLD = 1.0      # [s]
    RECOVER_MAX = 5.0       # [s]

    def __init__(self, side, speed, ccw, shaping=True):
        super().__init__("square_test")
        self.shaping = shaping
        self.cmd_v = 0.0        # last published (shaped) command
        self.cmd_w = 0.0
        self.yaw_f = None       # low-passed heading used for steering
        self.last_t = None
        self.recovering = None  # None, or dict(start, bad_since, settled_since, why)
        self.stumbles = 0
        self.V_MAX = min(speed, 0.3)
        self.side = side
        s = side
        # Corner list; robot starts at (0,0) facing +x.
        if ccw:
            self.waypoints = [(s, 0.0), (s, s), (0.0, s), (0.0, 0.0)]
        else:
            self.waypoints = [(s, 0.0), (s, -s), (0.0, -s), (0.0, 0.0)]
        self.wp_i = 0
        self.mode = "TURN"
        self.truth = None       # (x, y, yaw)
        self.truth_z = None     # base height [m], for the standing check
        self.crouch = 0.0       # commanded crouch from /body_pose [m], >= 0
        self.truth_tilt = 0.0   # angle between base z and world z [rad]
        self.prone_warned = False
        self.stood_once = False  # distinguishes "never stood" from "fell mid-run"
        # (t, x, y, yaw, cmd_moving) samples for the stall detector.
        self.history = collections.deque()
        self.est = None         # (x, y)
        self.max_err = 0.0
        self.err_sum = 0.0
        self.err_n = 0
        self.done = False

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(Odometry, "/ground_truth/odom", self._gt_cb, qos)
        self.create_subscription(Odometry, "/eskf/odom", self._est_cb, qos)
        self.create_subscription(Pose, "/body_pose", self._body_pose_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self._step)  # 20 Hz
        self.get_logger().info(
            f"Square test: {side} m sides, v={self.V_MAX} m/s, "
            f"{'CCW' if ccw else 'CW'}. Waiting for /ground_truth/odom...")

    def _gt_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.truth = (p.x, p.y, yaw_from_quat(q))
        self.truth_z = p.z
        # Angle between the base z axis and world up: R*(0,0,1) has z-component
        # 1 - 2(qx^2 + qy^2). Independent of terrain height, unlike truth_z.
        self.truth_tilt = math.acos(
            max(-1.0, min(1.0, 1.0 - 2.0 * (q.x * q.x + q.y * q.y))))

    def _body_pose_cb(self, msg):
        self.crouch = min(max(0.0, -msg.position.z), self.MAX_CROUCH_CREDIT)

    def stand_z(self):
        """Standing threshold, lowered by whatever crouch is being commanded."""
        return self.STAND_Z - self.crouch

    def _est_cb(self, msg):
        p = msg.pose.pose.position
        self.est = (p.x, p.y)
        if self.truth is not None:
            e = math.hypot(p.x - self.truth[0], p.y - self.truth[1])
            self.max_err = max(self.max_err, e)
            self.err_sum += e
            self.err_n += 1

    def _report_corner(self):
        tx, ty, _ = self.truth
        line = f"CORNER {self.wp_i + 1}/4 truth=({tx:+.2f},{ty:+.2f})"
        if self.est:
            ex, ey = self.est
            line += (f"  eskf=({ex:+.2f},{ey:+.2f})"
                     f"  err={math.hypot(ex - tx, ey - ty):.3f} m")
        self.get_logger().info(line)

    def _finish(self):
        self.cmd_pub.publish(Twist())  # stop
        self.done = True
        dist = 4.0 * self.side
        mean = self.err_sum / max(self.err_n, 1)
        final = 0.0
        if self.est and self.truth:
            final = math.hypot(self.est[0] - self.truth[0],
                               self.est[1] - self.truth[1])
        self.get_logger().info(
            "\n===== SQUARE DRIFT SUMMARY =====\n"
            f"  distance travelled : ~{dist:.0f} m\n"
            f"  final error        : {final:.3f} m ({100.0 * final / dist:.2f}% of distance)\n"
            f"  max error          : {self.max_err:.3f} m\n"
            f"  mean error         : {mean:.3f} m\n"
            f"  stumbles recovered : {self.stumbles}\n"
            "================================")

    def _stalled(self):
        """Commanded to move for a whole window, but ground truth never went anywhere.

        Progress is the MAXIMUM excursion from the window's first sample — how far
        the robot ever got from there, in position or heading — not the endpoint
        difference. MEASURED 2026-09-23: circling a missed corner, the 23:28 robot
        walked a ~0.6 m loop that ended 4 cm from where it began, and the endpoint
        test called that a 9 s stall. A robot scrabbling in place still never gets
        STALL_DIST from its start, so it is still caught.
        """
        if len(self.history) < 2:
            return None
        t0, x0, y0, yaw0, _ = self.history[0]
        t1 = self.history[-1][0]
        if t1 - t0 < self.STALL_WIN:
            return None                       # not enough history yet
        if not all(moving for _, _, _, _, moving in self.history):
            return None                       # we were not asking it to move
        moved = max(math.hypot(x - x0, y - y0) for _, x, y, _, _ in self.history)
        turned = max(abs(wrap(yw - yaw0)) for _, _, _, yw, _ in self.history)
        if moved < self.STALL_DIST and turned < self.STALL_YAW:
            return (f"no progress for {t1 - t0:.1f} s while commanded to move "
                    f"(never more than {moved * 100:.0f} cm / "
                    f"{math.degrees(turned):.0f} deg from where it was)")
        return None

    @staticmethod
    def _ramp(cur, tgt, step):
        """Limit growth of |cmd| to `step`; shrinking it (or crossing zero) is instant."""
        if cur * tgt < 0.0:
            cur = 0.0                      # reversing: drop to zero at once...
        if abs(tgt) <= abs(cur):
            return tgt                     # slowing down: immediate
        return cur + max(-step, min(step, tgt - cur))   # ...then speed up gradually

    def _shape(self, cmd, dt):
        """Rate-limit speed-ups (see ACCEL_MAX). Identity with --no-shaping."""
        if self.shaping:
            cmd.linear.x = self._ramp(self.cmd_v, cmd.linear.x, self.ACCEL_MAX * dt)
            cmd.angular.z = self._ramp(self.cmd_w, cmd.angular.z, self.ALPHA_MAX * dt)
        self.cmd_v, self.cmd_w = cmd.linear.x, cmd.angular.z
        return cmd

    def _filtered_yaw(self, yaw, dt):
        """Heading low-pass on the unit circle (wrap-safe). Raw with --no-shaping."""
        if not self.shaping:
            return yaw
        if self.yaw_f is None:
            self.yaw_f = [math.cos(yaw), math.sin(yaw)]
        else:
            a = dt / (self.YAW_TAU + dt)
            self.yaw_f[0] += a * (math.cos(yaw) - self.yaw_f[0])
            self.yaw_f[1] += a * (math.sin(yaw) - self.yaw_f[1])
        return math.atan2(self.yaw_f[1], self.yaw_f[0])

    def _recover(self, now, fall):
        """One tick of stumble recovery. True while still recovering (or aborted)."""
        rec = self.recovering
        self.cmd_pub.publish(Twist())
        self.cmd_v = self.cmd_w = 0.0            # resume from a fresh ramp
        if fall is not None:
            rec["bad_since"] = rec["bad_since"] or now
            rec["why"] = fall
            if now - rec["bad_since"] >= self.FALL_HOLD:
                self._abort_fallen(f"{fall}, for {self.FALL_HOLD} s")
                return True
        else:
            rec["bad_since"] = None
        settled = fall is None and self.truth_tilt < self.SETTLE_TILT
        if settled:
            rec["settled_since"] = rec["settled_since"] or now
            if now - rec["settled_since"] >= self.RECOVER_HOLD:
                self.stumbles += 1
                self.recovering = None
                self.history.clear()             # the pause is not a stall
                self.get_logger().info(
                    f"RECOVERED from stumble #{self.stumbles} after "
                    f"{now - rec['start']:.1f} s — resuming the square.")
                return False
        else:
            rec["settled_since"] = None
        if now - rec["start"] > self.RECOVER_MAX:
            self._abort_fallen(f"not settled {self.RECOVER_MAX} s after: {rec['why']}")
        return True

    def _abort_fallen(self, why):
        self.cmd_pub.publish(Twist())
        self.get_logger().error(
            f"ROBOT FELL/STUCK at truth=({self.truth[0]:+.2f},{self.truth[1]:+.2f}) "
            f"after {self.wp_i}/4 corners — {why}. This is a CHAMP gait failure in "
            "sim, not an estimator fault. Drift numbers past this point are "
            "MEANINGLESS: a prone robot's feet scrabble in place, leg odometry "
            "reports motion that is not happening, and the estimate runs away. "
            "Partial summary:")
        self._finish()

    def _step(self):
        if self.done:
            return
        if self.truth is None:
            return  # waiting for ground truth bridge

        # Before it has ever stood, the absolute STAND_Z test is the right one: every
        # world spawns the robot on ground at elevation 0. See STAND_Z.
        if not self.stood_once:
            if self.truth_z is not None and self.truth_z < self.stand_z():
                self.cmd_pub.publish(Twist())
                if not self.prone_warned:
                    self.get_logger().warn(
                        f"Robot base is at z={self.truth_z:.3f} m (< {self.stand_z():.3f} m) and it has "
                        "never stood, so /cmd_vel is held at zero. The leg controller most likely "
                        "never activated: check `ros2 control list_controllers` for "
                        "joint_group_effort_controller.")
                    self.prone_warned = True
                return
            self.stood_once = True
            if self.prone_warned:
                self.get_logger().info("Robot is standing now — starting the square.")
                self.prone_warned = False

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.05 if self.last_t is None else min(max(now - self.last_t, 0.0), 0.2)
        self.last_t = now

        # It has walked. Now detect going down WITHOUT assuming flat ground: absolute
        # height (valid on flat only) and body tilt pause-then-maybe-abort (see
        # FALL_HOLD); a stall aborts outright.
        fall = None
        if self.truth_z is not None and self.truth_z < self.stand_z():
            fall = (f"base z={self.truth_z:.3f} m below {self.stand_z():.3f} m "
                    f"(STAND_Z {self.STAND_Z} - commanded crouch {self.crouch:.3f})")
        elif self.truth_tilt > self.TILT_MAX:
            fall = f"body tilted {math.degrees(self.truth_tilt):.0f} deg"
        if fall is not None and self.recovering is None:
            self.recovering = dict(start=now, bad_since=None, settled_since=None,
                                   why=fall)
            self.get_logger().warn(
                f"STUMBLE at truth=({self.truth[0]:+.2f},{self.truth[1]:+.2f}): {fall} "
                f"— holding zero command; aborting only if it persists "
                f"{self.FALL_HOLD} s or it has not settled in {self.RECOVER_MAX} s.")
        if self.recovering is not None:
            if self._recover(now, fall):
                return
        stall = self._stalled()
        if stall is not None:
            self._abort_fallen(stall)
            return

        x, y, yaw_raw = self.truth
        yaw = self._filtered_yaw(yaw_raw, dt)
        gx, gy = self.waypoints[self.wp_i]
        dist = math.hypot(gx - x, gy - y)

        if dist < self.ARRIVE_TOL:
            self._report_corner()
            self.wp_i += 1
            if self.wp_i >= len(self.waypoints):
                self._finish()
                return
            self.mode = "TURN"
            gx, gy = self.waypoints[self.wp_i]

        bearing = math.atan2(gy - y, gx - x)
        herr = wrap(bearing - yaw)
        cmd = Twist()

        if self.mode == "TURN":
            if abs(herr) < self.HEAD_TOL:
                self.mode = "DRIVE"
            else:
                cmd.angular.z = max(-self.W_MAX, min(self.W_MAX, 1.5 * herr))
        if self.mode == "DRIVE":
            if abs(herr) > self.HEAD_REDO:
                self.mode = "TURN"   # drifted off heading — stop and re-aim
            else:
                cmd.linear.x = min(self.V_MAX, max(0.08, 0.8 * dist))
                cmd.angular.z = max(-self.W_MAX, min(self.W_MAX, 1.2 * herr))
        cmd = self._shape(cmd, dt)
        self.cmd_pub.publish(cmd)

        # Feed the stall detector: what we asked for, and where truth actually is.
        moving = (abs(cmd.linear.x) > self.STALL_CMD or
                  abs(cmd.angular.z) > self.STALL_CMD)
        self.history.append((now, x, y, yaw_raw, moving))
        # Keep MORE than one window, so the oldest sample is genuinely >= STALL_WIN
        # old and _stalled()'s span test can actually be satisfied.
        while self.history and now - self.history[0][0] > self.STALL_WIN * 1.5:
            self.history.popleft()


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--side", type=float, default=5.0, help="square side [m]")
    ap.add_argument("--speed", type=float, default=0.25, help="forward speed [m/s]")
    ap.add_argument("--ccw", action="store_true", help="counter-clockwise square")
    ap.add_argument("--no-shaping", action="store_true",
                    help="publish the raw controller output (step commands, steering "
                         "off unfiltered truth yaw) — the pre-2026-09-23 behaviour")
    args = ap.parse_args(argv[1:])

    rclpy.init()
    node = SquareTest(args.side, args.speed, args.ccw, shaping=not args.no_shaping)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Only touch the node if the context is still valid — on Ctrl-C/SIGTERM rclpy
        # may already have torn it down, and publishing then raises RCLError over the
        # real exit. A run cut short still prints the drift numbers it did collect:
        # the summary is the whole point of the test, so don't lose it to an interrupt.
        if rclpy.ok():
            if node.done:
                node.cmd_pub.publish(Twist())  # never leave the robot walking
            else:
                node.get_logger().info(
                    f"Interrupted after {node.wp_i}/4 corners — partial drift summary:")
                node._finish()                 # stops the robot and prints the summary
            rclpy.shutdown()


if __name__ == "__main__":
    main()
