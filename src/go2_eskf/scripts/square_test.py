#!/usr/bin/env python3
"""Autonomous 10 m square drift test for the go2_eskf estimator.

Drives the Go2 around a square (default 10 m sides) by publishing /cmd_vel,
closing the loop on GROUND TRUTH pose (/ground_truth/odom) — so the true path
is a clean square regardless of estimator quality, and the gap between the
ESKF estimate and truth is pure estimator drift. Replaces manual teleop for
repeatable drift evaluation.

Per corner it prints truth vs estimate vs error; at the end it prints a drift
summary (final error, max error, error as % of ~40 m distance travelled).

Needs: the sim (CHAMP walking), ground_truth.launch.py, and the ESKF running.
Respects the gait limits in gait.yaml (vx<=0.3, wz<=0.5).

  ros2 run go2_eskf square_test.py --ros-args -p use_sim_time:=true
  # options after --:  --side 10 --speed 0.25 --ccw
"""

import argparse
import collections
import math
import sys

import rclpy
from geometry_msgs.msg import Twist
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

    def __init__(self, side, speed, ccw):
        super().__init__("square_test")
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
            "================================")

    def _stalled(self):
        """Commanded to move for a whole window, but ground truth did not move."""
        if len(self.history) < 2:
            return None
        t0, x0, y0, yaw0, _ = self.history[0]
        t1, x1, y1, yaw1, _ = self.history[-1]
        if t1 - t0 < self.STALL_WIN:
            return None                       # not enough history yet
        if not all(moving for _, _, _, _, moving in self.history):
            return None                       # we were not asking it to move
        moved = math.hypot(x1 - x0, y1 - y0)
        turned = abs(wrap(yaw1 - yaw0))
        if moved < self.STALL_DIST and turned < self.STALL_YAW:
            return (f"no progress for {t1 - t0:.1f} s while commanded to move "
                    f"({moved * 100:.0f} cm, {math.degrees(turned):.0f} deg)")
        return None

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
            if self.truth_z is not None and self.truth_z < self.STAND_Z:
                self.cmd_pub.publish(Twist())
                if not self.prone_warned:
                    self.get_logger().warn(
                        f"Robot base is at z={self.truth_z:.3f} m (< {self.STAND_Z} m) and it has "
                        "never stood, so /cmd_vel is held at zero. The leg controller most likely "
                        "never activated: check `ros2 control list_controllers` for "
                        "joint_group_effort_controller.")
                    self.prone_warned = True
                return
            self.stood_once = True
            if self.prone_warned:
                self.get_logger().info("Robot is standing now — starting the square.")
                self.prone_warned = False

        # It has walked. Now detect going down WITHOUT assuming flat ground: absolute
        # height (valid on flat only), body tilt, and a stall. Any one aborts.
        if self.truth_z is not None and self.truth_z < self.STAND_Z:
            self._abort_fallen(f"base z={self.truth_z:.3f} m below {self.STAND_Z} m")
            return
        if self.truth_tilt > self.TILT_MAX:
            self._abort_fallen(f"body tilted {math.degrees(self.truth_tilt):.0f} deg")
            return
        stall = self._stalled()
        if stall is not None:
            self._abort_fallen(stall)
            return

        x, y, yaw = self.truth
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
        self.cmd_pub.publish(cmd)

        # Feed the stall detector: what we asked for, and where truth actually is.
        moving = (abs(cmd.linear.x) > self.STALL_CMD or
                  abs(cmd.angular.z) > self.STALL_CMD)
        now = self.get_clock().now().nanoseconds * 1e-9
        self.history.append((now, x, y, yaw, moving))
        # Keep MORE than one window, so the oldest sample is genuinely >= STALL_WIN
        # old and _stalled()'s span test can actually be satisfied.
        while self.history and now - self.history[0][0] > self.STALL_WIN * 1.5:
            self.history.popleft()


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--side", type=float, default=10.0, help="square side [m]")
    ap.add_argument("--speed", type=float, default=0.25, help="forward speed [m/s]")
    ap.add_argument("--ccw", action="store_true", help="counter-clockwise square")
    args = ap.parse_args(argv[1:])

    rclpy.init()
    node = SquareTest(args.side, args.speed, args.ccw)
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
