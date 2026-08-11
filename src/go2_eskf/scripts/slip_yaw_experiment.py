#!/usr/bin/env python3
"""Why inflating R_leg costs YAW — measured offline, no ROS, no Gazebo.

The first dual-arm terrain square (2026-08-11) ended with the slip-adaptive arm at
13.6 deg of yaw error against the baseline's 2.6 deg, and the position error
followed 1:1. The obvious objection is that leg odometry cannot see heading at all
(``skills.md`` §0 HEADLINE 2: the direction ``dv = dpsi*(-v_y, v_x)`` is exactly
unobservable), so scaling its covariance should not touch yaw.

That objection is wrong, and this script shows why in ~10 seconds with zero
run-to-run variance. ``correctLegOdom``'s Jacobian is

    H[:, PSI] = dRz(-psi)/dpsi @ v_world

which is nonzero whenever the robot is MOVING. Only one direction in the
(v_x, v_y, psi) subspace is unobservable; the orthogonal complement is measured,
so every leg-odom update applies a restoring pull on psi toward the heading
implied by the measured body velocity. Inflating R_leg weakens that spring —
it does not remove information the filter never had, it stops using information
it did have.

The experiment: two filters, identical inputs, differing only in R_leg. Walk
straight, inject a burst of gyro error to knock heading off (this is what a corner
does — see the |wz| scale error in skills.md §0), then watch the recovery.

    python3 src/go2_eskf/scripts/slip_yaw_experiment.py

MEASURED (40 seeds, 2026-08-11) — and the result is NOT the tidy one:

  * The restoring pull is real but CONDITIONAL. It works only while v_world is
    still anchored near truth. Once (v_world, psi) have rotated together into the
    unobservable direction the residual vanishes and NOTHING pulls heading back,
    for either arm. Baseline recovers in only 16/40 seeds.
  * Inflating R_leg makes recovery steadily LESS likely — 16/40 (1.00x), 15/40
    (1.44x), 12/40 (1.82x), 12/40 (2.25x), 10/40 (4.00x).
  * But the FINAL yaw error is statistically indistinguishable: mean 11.90 deg at
    1.00x vs 11.66 deg at 1.44x and 11.47 deg at 1.82x, with the inflated arm
    worse in 19-20 seeds out of 40 — a coin flip, in BOTH directions.

So do not read a single seed (or a single square run) as evidence that slip
adaptation costs heading. What R_leg reliably changes is the PROBABILITY of
recovering from a heading kick, not the size of the error you end up with. The
size is set by which basin the run falls into, and a small R change is enough to
flip the basin either way.
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eskf_reference import EskfReference, PSI, VX, VY, BG  # noqa: E402

# Matches config/eskf_params.yaml: leg_odom_vel_noise 0.2 -> R = 0.04*I.
LEG_STD = 0.2
# accel_noise 1.0 in the node — the sim IMU is dominated by contact spikes, which
# is exactly why leg odometry is meant to carry the velocity estimate.
ACCEL_NOISE = 1.0
IMU_HZ, LEG_HZ = 100.0, 50.0
SPEED = 0.25          # m/s, the square test's commanded speed
DURATION = 60.0
KICK_T, KICK_LEN, KICK_RATE = 10.0, 4.0, 0.05   # rad/s of false yaw rate


def run(r_scale, seed=0):
    """One filter. r_scale = (1 + lambda*slip) — the slip model's R_leg factor."""
    rng = np.random.default_rng(seed)
    f = EskfReference(accel_noise=ACCEL_NOISE)
    R_leg = np.eye(2) * (LEG_STD * r_scale) ** 2

    dt = 1.0 / IMU_HZ
    leg_every = int(IMU_HZ / LEG_HZ)
    n = int(DURATION * IMU_HZ)
    ts, yaw_err = [], []

    for k in range(n):
        t = k * dt
        # --- truth: walking straight along +x, true yaw fixed at 0.
        yaw_true = 0.0
        v_world_true = np.array([SPEED, 0.0])

        # --- IMU. Gravity is already cancelled upstream (gravity_lp), so what
        # reaches the filter is contact noise plus, during the kick, a yaw-rate
        # error standing in for the corner scale error.
        accel_body = rng.normal(0.0, ACCEL_NOISE, 3)
        accel_body[2] = 0.0
        gyro_z = rng.normal(0.0, 2.0e-3)
        if KICK_T <= t < KICK_T + KICK_LEN:
            gyro_z += KICK_RATE
        f.predict_imu(accel_body, gyro_z, 0.0, 0.0, dt)

        # --- leg odometry: body-frame velocity, honest, at 50 Hz.
        if k % leg_every == 0:
            c, s = math.cos(yaw_true), math.sin(yaw_true)
            Rz_T = np.array([[c, s], [-s, c]])
            v_body = Rz_T @ v_world_true + rng.normal(0.0, 0.02, 2)
            f.correct_leg_odom(v_body, R_leg)

        ts.append(t)
        yaw_err.append(f.x[PSI] - yaw_true)

    return np.array(ts), np.array(yaw_err)


def summarize(ts, err):
    peak_i = int(np.argmax(np.abs(err)))
    peak, peak_t = err[peak_i], ts[peak_i]
    # Recovery half-life: time from the peak until |error| first halves.
    half = None
    for t, e in zip(ts[peak_i:], err[peak_i:]):
        if abs(e) <= abs(peak) / 2.0:
            half = t - peak_t
            break
    return peak, err[-1], half


ARMS = (("baseline (fixed R)", 1.0),
        ("slip 0.2 (lambda=1)", 1.2),
        ("slip 0.35", 1.35),
        ("slip 0.5", 1.5),
        ("slip 1.0", 2.0))


def main():
    seeds = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    print(__doc__.split("\n\n")[0])
    print(f"\nStraight walk at {SPEED} m/s, {KICK_RATE} rad/s of false yaw rate "
          f"for {KICK_LEN:.0f}s at t={KICK_T:.0f}s.")
    print(f"{seeds} seeds per arm; within a seed the inputs are IDENTICAL across "
          "arms — R_leg is the only difference.\n")

    base = None
    print(f"{'arm':22s} {'R_leg':>7s} {'mean|final|':>12s} {'median':>8s} "
          f"{'recovered':>10s} {'vs base':>16s}")
    print("-" * 80)
    for label, scale in ARMS:
        finals, recovered = [], 0
        for s in range(seeds):
            _, final, half = summarize(*run(scale, seed=s))
            finals.append(abs(math.degrees(final)))
            recovered += half is not None
        if base is None:
            base, cmp = finals, ""
        else:
            worse = sum(a > b for a, b in zip(finals, base))
            cmp = f"worse in {worse}/{seeds}"
        print(f"{label:22s} {scale ** 2:6.2f}x {np.mean(finals):11.2f}° "
              f"{np.median(finals):7.2f}° {recovered:6d}/{seeds} {cmp:>16s}")

    print("\nRead the 'recovered' column, not the error columns. Inflating R_leg\n"
          "monotonically lowers the chance of recovering from a heading kick, but\n"
          "leaves the FINAL error a coin flip in both directions — the size of that\n"
          "error is set by whether the run fell into the recovering basin at all.\n"
          "One seed here, like one square run, proves nothing.")


if __name__ == "__main__":
    main()
