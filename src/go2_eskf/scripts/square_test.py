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
    # Base height below which the Go2 is NOT standing. It spawns at z=0.375 and
    # stands at ~0.30; lying on its belly it reads ~0.06. If ros2_control never
    # activated joint_group_effort_controller the robot collapses, and driving a
    # prone robot silently produces a "frozen at origin" trace that looks like an
    # estimator bug. Refuse to drive until it is actually standing.
    STAND_Z = 0.18

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
        self.prone_warned = False
        self.stood_once = False  # distinguishes "never stood" from "fell mid-run"
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
        self.truth = (p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))
        self.truth_z = p.z

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

    def _step(self):
        if self.done:
            return
        if self.truth is None:
            return  # waiting for ground truth bridge

        # Don't drive a robot that isn't on its feet — see STAND_Z.
        if self.truth_z is not None and self.truth_z < self.STAND_Z:
            self.cmd_pub.publish(Twist())
            if self.stood_once:
                # It walked and then went down: a gait failure, not a setup problem.
                # Abort rather than idle — the drift measurement is void from here,
                # and a silent stall wastes the whole run.
                self.get_logger().error(
                    f"ROBOT FELL at truth=({self.truth[0]:+.2f},{self.truth[1]:+.2f}) "
                    f"after {self.wp_i}/4 corners (base z={self.truth_z:.3f} m). "
                    "This is a CHAMP gait failure in sim, not an estimator fault — "
                    "drift numbers past this point are meaningless. Partial summary:")
                self._finish()
                return
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
