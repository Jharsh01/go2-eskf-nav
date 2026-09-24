#!/usr/bin/env python3
"""Did the Go2 actually stand up? Answered from the IMU's gravity direction.

run_go2_teleop.sh used to print "Leg controller ACTIVE — the Go2 is standing." the
moment joint_group_effort_controller went active, without ever checking. Two of four
launches on 2026-09-24 then ended with the robot on its side (00:31) or its back
(00:50) before any command was sent, and the whole stack sat until the 300 s cap.

Run it right after the controller activates. It waits --settle seconds of SIM time
(so a slow stand-up, or a robot that stands and then tips over, is still caught),
then averages the accelerometer over --window seconds. At rest the accelerometer
reads the gravity reaction, so the angle between it and the IMU's +z axis is the
body tilt: ~0 deg standing, ~90 on its side, ~180 on its back. It prints one line:

    STAND_CHECK UPRIGHT tilt=2.1deg
    STAND_CHECK NOT_UPRIGHT tilt=179.7deg

and exits 0 / 1. The launcher greps that line. Only the magnitude of the accel
vector's direction is used, so it is independent of the IMU's roll/pitch sign
conventions (the orientation field is unreliable in this sim, DESIGN.md §6).

  stand_check.py --ros-args -p use_sim_time:=true -- [--settle 15] [--max-tilt 45]
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class StandCheck(Node):
    def __init__(self, settle, window):
        super().__init__("stand_check")
        self.settle, self.window = settle, window
        self.t0 = None
        self.sum = [0.0, 0.0, 0.0]
        self.n = 0
        self.done = False
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)

    def on_imu(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        age = t - self.t0
        if age < self.settle:
            return
        a = msg.linear_acceleration
        self.sum[0] += a.x
        self.sum[1] += a.y
        self.sum[2] += a.z
        self.n += 1
        if age >= self.settle + self.window:
            self.done = True

    def tilt_deg(self):
        x, y, z = (s / max(self.n, 1) for s in self.sum)
        norm = math.sqrt(x * x + y * y + z * z)
        if norm < 1e-6:
            return float("nan")
        return math.degrees(math.acos(max(-1.0, min(1.0, z / norm))))


def main():
    argv = rclpy.utilities.remove_ros_args(sys.argv)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--settle", type=float, default=15.0, help="[s] sim time to wait")
    ap.add_argument("--window", type=float, default=1.0, help="[s] averaging window")
    ap.add_argument("--max-tilt", type=float, default=45.0, help="[deg] upright below this")
    ap.add_argument("--wall-timeout", type=float, default=120.0,
                    help="[s] give up (and report NOT_UPRIGHT) if no IMU arrives")
    args = ap.parse_args([a for a in argv[1:] if a != "--"])

    rclpy.init()
    node = StandCheck(args.settle, args.window)
    deadline = time.monotonic() + args.wall_timeout   # bounded on WALL time
    while rclpy.ok() and not node.done and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
    if not node.done:
        print(f"STAND_CHECK NOT_UPRIGHT tilt=nan (no usable /imu/data within "
              f"{args.wall_timeout:.0f} s)", flush=True)
        rc = 1
    else:
        tilt = node.tilt_deg()
        ok = tilt == tilt and tilt < args.max_tilt
        print(f"STAND_CHECK {'UPRIGHT' if ok else 'NOT_UPRIGHT'} tilt={tilt:.1f}deg",
              flush=True)
        rc = 0 if ok else 1
    node.destroy_node()
    rclpy.shutdown()
    return rc


if __name__ == "__main__":
    sys.exit(main())
