#!/usr/bin/env python3
"""Measure CHAMP leg-odometry velocity against ground truth, straight vs turning.

Drives the robot straight, then rotates in place, and reports the ratios
  vx_leg / vx_truth      (expect ~0.9 raw: CHAMP's odom_scaler)
  wz_leg / wz_truth      (expect ~1.0 if correct; >1 => theta_sum not averaged)
  wz_gyro / wz_truth     (expect ~1.0; sign shows whether gyro_z_sign is right)
and the residual that eskf_node feeds to correctGyroBias:
  gyro_wz - wz_leg       (should be ~b_g, i.e. ~constant and ~0 in BOTH phases)

DEGENERATE SAMPLES. champ::Odometry::getVelocities early-returns hard zeros for
linear.x, linear.y AND angular.z whenever all four or zero feet are in contact
("nothing to calculate") — a NO-INFORMATION flag, not a measurement. MEASURED
2026-07-26: this never happens while walking or turning (0/23,000 samples);
noFootInContact() does not fire on a trot, and standing (all four feet planted)
produces unbroken runs of 3-5 s. Every ratio is still reported over ALL and over
VALID samples separately, because an all-sample median would hide such a
bimodality completely if the gait ever changed.
"""
import math
import statistics as st
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Probe(Node):
    def __init__(self):
        super().__init__("probe_yaw")
        self.truth_v = None      # (vx_body, wz)
        self.leg = None          # (vx, vy, wz)
        self.gyro = None         # wz
        self.samples = []
        self.phase = "settle"
        self.create_subscription(Odometry, "/ground_truth/odom", self.gt_cb, 10)
        self.create_subscription(Odometry, "/odom/raw", self.leg_cb, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_cb, 10)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.yaw = 0.0
        # Degenerate-run tracking, measured per MESSAGE (50 Hz) rather than in the
        # ~20 Hz polling loop, so the durations are honest. eskf_node's
        # degenerate_hold_sec must sit above the longest run seen while WALKING and
        # below the runs seen while STOPPED; these are the numbers to set it from.
        self.runs = {}           # phase -> list of completed run durations [s]
        self.run_start = None    # stamp [s] the current run of zeros began
        self.run_last = None     # stamp [s] of the most recent zero in that run
        self.run_phase = None    # phase the run STARTED in (a stopped-phase run
                                 # only ends once the next phase is driving, so
                                 # attributing it to the end phase would be wrong)

    def gt_cb(self, m):
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # gz OdometryPublisher reports twist in the child (body) frame.
        self.truth_v = (m.twist.twist.linear.x, m.twist.twist.angular.z)

    def leg_cb(self, m):
        # linear.y is captured only so the all-zero degenerate sample can be
        # identified exactly the way champ::Odometry emits it.
        self.leg = (m.twist.twist.linear.x, m.twist.twist.linear.y,
                    m.twist.twist.angular.z)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if is_degenerate(self.leg):
            if self.run_start is None:
                self.run_start = stamp
                self.run_phase = self.phase
            self.run_last = stamp
        elif self.run_start is not None:
            self.runs.setdefault(self.run_phase, []).append(
                self.run_last - self.run_start)
            self.run_start = None
            self.run_last = None
            self.run_phase = None

    def imu_cb(self, m):
        self.gyro = m.angular_velocity.z

    def record(self):
        if None in (self.truth_v, self.leg, self.gyro):
            return
        self.samples.append((self.phase, self.truth_v[0], self.truth_v[1],
                             self.leg[0], self.leg[2], self.gyro,
                             is_degenerate(self.leg)))

    def drive(self, vx, wz, secs, phase):
        self.phase = phase
        t_end = time.time() + secs
        cmd = Twist()
        cmd.linear.x = vx
        cmd.angular.z = wz
        while time.time() < t_end:
            self.pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            self.record()
        self.pub.publish(Twist())


def is_degenerate(leg):
    """True for champ::Odometry's all-zero 'nothing to calculate' sample.

    Exact == 0.0 is deliberate: champ writes literal zeros in that branch, so this
    is not a near-zero measurement being thresholded.
    """
    return leg[0] == 0.0 and leg[1] == 0.0 and leg[2] == 0.0


def _ratios(sel, indent):
    """Print truth/leg/gyro medians and ratios for one bucket of samples."""
    tv = [r[1] for r in sel]
    tw = [r[2] for r in sel]
    lv = [r[3] for r in sel]
    lw = [r[4] for r in sel]
    gw = [r[5] for r in sel]
    resid = [r[5] - r[4] for r in sel]
    print(f"{indent}truth   vx={st.median(tv):+.3f} m/s   wz={st.median(tw):+.3f} rad/s")
    print(f"{indent}leg     vx={st.median(lv):+.3f} m/s   wz={st.median(lw):+.3f} rad/s")
    print(f"{indent}gyro                       wz={st.median(gw):+.3f} rad/s")
    if abs(st.median(tv)) > 0.05:
        print(f"{indent}RATIO vx_leg/vx_truth  = {st.median(lv)/st.median(tv):+.3f}   (0.9 = odom_scaler)")
    if abs(st.median(tw)) > 0.05:
        print(f"{indent}RATIO wz_leg/wz_truth  = {st.median(lw)/st.median(tw):+.3f}   (>1 => theta_sum not averaged)")
        print(f"{indent}RATIO wz_gyro/wz_truth = {st.median(gw)/st.median(tw):+.3f}   (sign => gyro_z_sign)")
    print(f"{indent}correctGyroBias input (gyro_wz - wz_leg) = {st.median(resid):+.4f} rad/s")


def summarize_runs(runs, phase, label):
    """Report the duration of unbroken runs of degenerate samples for one phase."""
    r = runs.get(phase, [])
    if not r:
        print(f"  {label}: no completed degenerate runs")
        return
    print(f"  {label}: {len(r)} runs, median {st.median(r):.3f} s, "
          f"max {max(r):.3f} s")


def summarize(rows, phase, label):
    sel = [r for r in rows if r[0] == phase]
    # drop the first 20% (transient) and require real motion
    sel = sel[len(sel) // 5:]
    if not sel:
        print(f"  {label}: no samples")
        return

    degen = [r for r in sel if r[6]]
    valid = [r for r in sel if not r[6]]
    frac = len(degen) / len(sel)

    print(f"  {label}  (n={len(sel)})")
    print(f"    DEGENERATE /odom/raw samples: {len(degen)}/{len(sel)} = {frac:6.1%}"
          f"   [all-zero 'no information'; expected 0% while moving]")

    print("    -- ALL samples (comparable with the numbers recorded in skills.md) --")
    _ratios(sel, "      ")

    if valid:
        print("    -- VALID samples only (degenerate excluded: real sensor behaviour) --")
        _ratios(valid, "      ")
    else:
        print("    -- VALID samples only: NONE (every sample was degenerate) --")

    # The bias pseudo-measurement is the channel that corrupts heading: on a
    # degenerate sample wz_leg is 0, so the residual is the WHOLE gyro rate and
    # correctGyroBias is told all of it is bias.
    if degen:
        d = st.median([r[5] - r[4] for r in degen])
        print(f"    correctGyroBias input on DEGENERATE samples = {d:+.4f} rad/s"
              f"   [== gyro_wz; should have been REJECTED, not fused as bias]")
    if valid:
        v = st.median([r[5] - r[4] for r in valid])
        print(f"    correctGyroBias input on VALID      samples = {v:+.4f} rad/s"
              f"   [this is the honest ~b_g: ~0 and EQUAL in both phases]")


def main():
    rclpy.init()
    n = Probe()
    print("settling 5 s ...", flush=True)
    n.drive(0.0, 0.0, 5.0, "settle")
    print("driving STRAIGHT 12 s ...", flush=True)
    n.drive(0.25, 0.0, 12.0, "straight")
    n.drive(0.0, 0.0, 3.0, "pause")
    print("TURNING in place 12 s ...", flush=True)
    n.drive(0.0, 0.4, 12.0, "turn")
    n.pub.publish(Twist())

    print("\n================ RESULTS ================")
    summarize(n.samples, "straight", "STRAIGHT (vx=0.25, wz=0)")
    print()
    summarize(n.samples, "turn", "TURN (vx=0, wz=+0.4)")
    print("\n--- degenerate-run DURATIONS (sets eskf_node's degenerate_hold_sec) ---")
    print("  Pick a threshold ABOVE the max seen while walking and BELOW the runs")
    print("  seen while stopped; the two must be well separated for the gate to")
    print("  tell a flight phase apart from a genuine stop.")
    summarize_runs(n.runs, "straight", "walking straight")
    summarize_runs(n.runs, "turn", "turning in place")
    summarize_runs(n.runs, "settle", "STOPPED (settle)")
    summarize_runs(n.runs, "pause", "STOPPED (pause)")
    print("=========================================")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
