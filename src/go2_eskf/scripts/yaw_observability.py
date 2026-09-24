#!/usr/bin/env python3
"""Does the GPS we now fuse actually fix YAW? Measured offline, no ROS, no Gazebo.

``skills.md`` §0 HEADLINE 2 says heading is *exactly* unobservable in this ESKF —
``correctLegOdom`` cannot see the direction ``dv = dpsi*(-v_y, v_x)``. That result
is TRUE ONLY WITH GPS OFF, which is the condition it was derived under. GPS was
switched on by default on 2026-09-04, and ``correctGps`` pins world POSITION, which
reaches psi through two covariance couplings the filter already builds:

    P(p, v)   from  F[PX, VX] = I*dt          (every predict)
    P(v, psi) from  H[:, PSI] = dRz(-psi)/dpsi @ v_world   (every leg-odom update)

So yaw should now be *weakly* observable whenever the robot is MOVING. Weakly is not
a number, and one square run cannot produce one (identical configs have given 4 deg
and 97 deg). This script produces it in seconds with zero run-to-run variance, the
same way ``leg_odom_model.py`` settled the leg-odom twist bug and
``slip_yaw_experiment.py`` settled the R_leg-vs-yaw question.

The experiment: drive the 5 m square that ``square_test.py`` drives — turn in place
at up to 0.4 rad/s, then drive the side at 0.25 m/s — with a GYRO SCALE ERROR that
injects heading error at every corner and nowhere else, which is the measured shape
of the real failure. Truth is exact and identical across arms within a seed; only
the sensor set differs.

Arms:

    gps off                 the archived baseline: psi runs open-loop on the gyro
    gps on                  position fusion only — the configuration shipped today
    gps on + course heading  Stage 2a of the plan, emulated here WITHOUT touching
                            eskf_core: heading from GPS position deltas over a 2 m
                            window, fused as a direct psi measurement

The third arm is the point of doing this offline: it evaluates a fix that has not
been written yet, so the decision to write it is made on a measurement.

Usage:
    python3 src/go2_eskf/scripts/yaw_observability.py [seeds] [--straight]

MEASURED (40 seeds, 5 m square, 2026-09-04) — and the answer is unusually clean:

    arm                          mean|final|   median    worst   mean|err| over run
    gps off (archived baseline)      23.41°   23.17°   56.52°              11.46°
    gps on (shipped today)            2.00°    1.96°    4.10°               2.68°
    gps on + course heading           4.00°    3.76°   10.38°               5.59°

  * GPS POSITION FUSION ALONE FIXES YAW: 23.4 deg -> 2.0 deg, better in 39/40
    seeds, worst case 4.1 deg (the GPS-off worst case is 56.5 deg). No new
    measurement, no new sensor, no tuning — this is the configuration already
    shipped. The §0 unobservability result is not wrong; its precondition is gone.
  * ADDING A COURSE-OVER-GROUND HEADING MAKES IT WORSE (2.00 -> 4.00 deg), and
    this is not a tuning artifact. The course heading is computed FROM GPS fixes
    that were already fused as positions, so fusing it again double-counts the
    same measurement: the filter becomes over-confident in a lagged, window-
    averaged heading and pulls psi toward where the robot WAS. Any Stage-2 fix
    built on GPS-derived heading inherits this defect. A genuinely independent
    heading source (magnetometer/AHRS) would not, but on these numbers nothing
    needs one.
  * Straddling matters: gating the window on the instantaneous turn rate is not
    enough, the buffer must be CLEARED when turning, or the window reports the
    previous side's heading (4.81 -> 4.00 deg mean final).

  * Control, --straight (20 seeds, 300 s of continuous walking, no corners):
    51.02 deg -> 0.46 deg, better in 20/20 seeds. Sustained motion is exactly the
    condition H[:, PSI] needs, so the effect is even stronger there. The corollary
    is that the square's TURNS are where GPS cannot help — turning in place has
    v_world = 0, so H[:, PSI] = 0 and the ZUPT carries no heading information.
    Heading is corrected on the sides, not in the corners.

So: do not add a yaw measurement. Verify this in sim instead.

UPDATE 2026-09-23 — MAGNETOMETER ARMS (correctYaw now exists in eskf_core; the
twin's correct_yaw replaces this script's local copy, same math). Heading noise:
0.98 deg white (MEASURED on the gz sensor) + an ASSUMED Gauss-Markov tilt error
(tau 0.5 s, sigma MAG_TILT_STD) standing in for gait rock under the low-passed
"up". Archived arms reproduce bit-for-bit (the mag noise has its own RNG).

    tilt sigma   route      gps only   gps + mag   mag better   gps OFF + mag
      2 deg     square       2.00°       0.61°       37/40          0.61°
      5 deg     square       2.00°       1.48°       29/40          1.49°
     10 deg     square       2.00°       2.93°       17/40          2.95°
      2 deg     straight     0.46°       0.58°       10/20
      5 deg     straight     0.46°       1.44°        2/20

  * The magnetometer is the one heading source that works in the CORNERS, which
    is where GPS cannot help. It pays on the square when the tilt error is small,
    and it rescues the GPS-OFF case outright (23.4 deg -> 0.6-3.0 deg).
  * With GPS on and straight walking it adds nothing: GPS already pins heading
    there, and the mag's correlated error becomes the floor.
  * So the decision hinges on ONE unmeasured number: the live tilt-compensation
    error. Measure /eskf/mag_heading against ground-truth yaw on a walking run
    before turning use_mag on by default — run_report.py's magnetometer section
    reports exactly this std (and its correlation time) for any --mag run.
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eskf_reference import EskfReference, PX, PY, PSI, VX, VY  # noqa: E402

# --- Sensor / filter constants. Every one of these matches a real measurement or
#     a config value; none is a convenient guess.
LEG_STD = 0.2          # eskf_params.yaml leg_odom_vel_noise
ZUPT_STD = 0.02        # eskf_params.yaml zupt_vel_noise (turn in place -> ZUPT)
ACCEL_NOISE = 1.0      # eskf_params.yaml accel_noise (contact spikes dominate)
GYRO_STD = 2.0e-3
GPS_STD = 0.5          # MEASURED 2026-08-28: 0.542 m N/S, 0.491 m E/W
LEG_MEAS_STD = 0.02    # honest leg-odom noise on top of the true body twist

IMU_HZ, LEG_HZ, GPS_HZ = 100.0, 50.0, 10.0

# --- Trajectory: square_test.py's controller, driven off TRUTH (as it really is).
SIDE = 5.0
V_MAX, W_MAX = 0.25, 0.4
HEAD_TOL = 0.10
ARRIVE_TOL = 0.25
MAX_T = 300.0

# The corner error. CHAMP's per-foot bearing sum over-read yaw rate by ~8 % before
# the least-squares twist solver (skills.md §0); that solver is NOT yet validated
# live, so 8 % is the honest disturbance to design against.
GYRO_SCALE_ERR = 0.08

# --- Magnetometer arm. White part MEASURED 2026-09-23 on the gz sensor (0.98 deg
#     heading std at 0.005 G/axis). The correlated part is an ASSUMPTION standing in
#     for tilt-compensation error — "up" is the low-passed accelerometer and the gait
#     rocks the body under it — modelled as a first-order Gauss-Markov process. The
#     filter uses eskf_params.yaml's mag_heading_noise, fused at mag_min_interval.
MAG_HZ = 10.0
MAG_WHITE_STD = math.radians(0.98)
MAG_TILT_STD = math.radians(2.0)
MAG_TILT_TAU = 0.5      # [s]
MAG_R_STD = 0.05        # eskf_params.yaml mag_heading_noise

# --- Course-heading arm (Stage 2a, emulated).
COURSE_MIN_DIST = 2.0   # [m] window displacement before a heading is formed
COURSE_MAX_WZ = 0.05    # [rad/s] only while going straight


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def corners(side):
    return [(side, 0.0), (side, -side), (0.0, -side), (0.0, 0.0)]


def run(seed, use_gps, use_course, straight=False, use_mag=False):
    """One filter over one trajectory. Returns (t, yaw_err) arrays."""
    rng = np.random.default_rng(seed)
    f = EskfReference(accel_noise=ACCEL_NOISE)

    dt = 1.0 / IMU_HZ
    leg_every = int(IMU_HZ / LEG_HZ)
    gps_every = int(IMU_HZ / GPS_HZ)
    mag_every = int(IMU_HZ / MAG_HZ)
    # Its own generator, so adding the mag arm leaves every other arm's noise
    # draws — and therefore their archived numbers — bit-for-bit unchanged.
    mag_rng = np.random.default_rng(10_000 + seed)
    mag_tilt = 0.0
    mag_phi = math.exp(-1.0 / (IMU_HZ * MAG_TILT_TAU))
    R_leg = np.eye(2) * LEG_STD ** 2
    R_zupt = np.eye(2) * ZUPT_STD ** 2
    R_gps = np.eye(2) * GPS_STD ** 2

    # Truth.
    p = np.zeros(2)
    yaw = 0.0
    goals = corners(SIDE)
    gi, mode = 0, "TURN"

    course_buf = []          # (p_gps, t) for the course-heading arm
    ts, errs = [], []
    k = 0
    while k * dt < MAX_T:
        t = k * dt

        # --- Trajectory controller, exactly square_test.py's, run on truth.
        if straight:
            v_cmd, w_cmd = V_MAX, 0.0
        else:
            if gi >= len(goals):
                break
            gx, gy = goals[gi]
            dx, dy = gx - p[0], gy - p[1]
            dist = math.hypot(dx, dy)
            if dist < ARRIVE_TOL:
                gi += 1
                mode = "TURN"
                continue
            herr = wrap(math.atan2(dy, dx) - yaw)
            v_cmd, w_cmd = 0.0, 0.0
            if mode == "TURN":
                if abs(herr) < HEAD_TOL:
                    mode = "DRIVE"
                else:
                    w_cmd = max(-W_MAX, min(W_MAX, 1.5 * herr))
            if mode == "DRIVE":
                v_cmd = min(V_MAX, max(0.08, 0.8 * dist))
                w_cmd = max(-W_MAX, min(W_MAX, 1.2 * herr))

        v_world = np.array([v_cmd * math.cos(yaw), v_cmd * math.sin(yaw)])

        # --- IMU. The gyro over-reads turn rate by GYRO_SCALE_ERR, so heading
        #     error is injected at the corners and nowhere else.
        accel_body = rng.normal(0.0, ACCEL_NOISE, 3)
        accel_body[2] = 0.0
        gyro_z = w_cmd * (1.0 + GYRO_SCALE_ERR) + rng.normal(0.0, GYRO_STD)
        f.predict_imu(accel_body, gyro_z, 0.0, 0.0, dt)

        # --- Advance truth.
        p = p + v_world * dt
        yaw = wrap(yaw + w_cmd * dt)

        # --- Leg odometry at 50 Hz. Turning in place produces CHAMP's all-zero
        #     samples, which the node treats as a ZUPT (tight R), not as data.
        if k % leg_every == 0:
            c, s = math.cos(yaw), math.sin(yaw)
            Rz_T = np.array([[c, s], [-s, c]])
            if abs(v_cmd) < 1e-9:
                f.correct_leg_odom(np.zeros(2), R_zupt)
            else:
                v_body = Rz_T @ v_world + rng.normal(0.0, LEG_MEAS_STD, 2)
                f.correct_leg_odom(v_body, R_leg)

        # --- GPS at 10 Hz.
        if use_gps and k % gps_every == 0:
            z = p + rng.normal(0.0, GPS_STD, 2)
            f.correct_gps(z, R_gps)

            if use_course:
                # A window that STRADDLES a corner reports the heading of the
                # previous side, so turning must clear it, not merely gate it.
                if abs(w_cmd) >= COURSE_MAX_WZ:
                    course_buf.clear()
                course_buf.append(z.copy())
                # Trim to the shortest window that still spans COURSE_MIN_DIST.
                while len(course_buf) > 2 and \
                        np.linalg.norm(course_buf[-1] - course_buf[1]) >= COURSE_MIN_DIST:
                    course_buf.pop(0)
                d = np.linalg.norm(course_buf[-1] - course_buf[0])
                if d >= COURSE_MIN_DIST and abs(w_cmd) < COURSE_MAX_WZ:
                    delta = course_buf[-1] - course_buf[0]
                    psi_meas = math.atan2(delta[1], delta[0])
                    # Crab angle: heading = course - atan2(v_y, v_x) in body frame.
                    # Modelled as zero here (no lateral leg-odom bias); the real
                    # node must subtract it from the measured body twist.
                    sigma = GPS_STD * math.sqrt(2.0) / d
                    f.correct_yaw(psi_meas, sigma ** 2)

        # --- Magnetometer heading at 10 Hz (EskfCore::correctYaw).
        if use_mag:
            mag_tilt = mag_phi * mag_tilt + mag_rng.normal(
                0.0, MAG_TILT_STD * math.sqrt(1.0 - mag_phi ** 2))
            if k % mag_every == 0:
                z = wrap(yaw + mag_tilt + mag_rng.normal(0.0, MAG_WHITE_STD))
                f.correct_yaw(z, MAG_R_STD ** 2)

        ts.append(t)
        errs.append(wrap(f.x[PSI] - yaw))
        k += 1

    return np.array(ts), np.array(errs)


ARMS = (
    ("gps off (archived baseline)", dict(use_gps=False, use_course=False)),
    ("gps on (shipped today)", dict(use_gps=True, use_course=False)),
    ("gps on + course heading", dict(use_gps=True, use_course=True)),
    ("gps off + magnetometer", dict(use_gps=False, use_course=False, use_mag=True)),
    ("gps on + magnetometer", dict(use_gps=True, use_course=False, use_mag=True)),
)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seeds", nargs="?", type=int, default=40)
    ap.add_argument("--straight", action="store_true",
                    help="straight walk instead of the square (no corners, so no "
                         "injected heading error — a control)")
    args = ap.parse_args()

    print(__doc__.split("\n\n")[0])
    route = "straight walk" if args.straight else f"{SIDE:.0f} m square"
    print(f"\nRoute: {route}, v={V_MAX} m/s, gyro yaw-rate scale error "
          f"{GYRO_SCALE_ERR:+.0%} (injected at the corners).")
    print(f"GPS: {GPS_HZ:.0f} Hz, sigma={GPS_STD} m (MEASURED, 2026-08-28). "
          f"{args.seeds} seeds; inputs identical across arms within a seed.\n")

    print(f"{'arm':30s} {'mean|final|':>12s} {'median':>9s} {'worst':>9s} "
          f"{'mean|err| over run':>19s}")
    print("-" * 84)
    base_finals = None
    results = {}
    for label, kw in ARMS:
        finals, means = [], []
        for s in range(args.seeds):
            _, err = run(s, straight=args.straight, **kw)
            finals.append(abs(math.degrees(err[-1])))
            means.append(np.mean(np.abs(np.degrees(err))))
        results[label] = (finals, means)
        if base_finals is None:
            base_finals = finals
        print(f"{label:30s} {np.mean(finals):11.2f}° {np.median(finals):8.2f}° "
              f"{np.max(finals):8.2f}° {np.mean(means):18.2f}°")

    print()
    base = results[ARMS[0][0]][0]
    for label, _ in ARMS[1:]:
        finals = results[label][0]
        better = sum(a < b for a, b in zip(finals, base))
        print(f"{label:30s} beats the GPS-off baseline in {better}/{args.seeds} "
              f"seeds ({np.mean(base) / max(np.mean(finals), 1e-9):.1f}x lower "
              f"mean final error)")

    print("\nRead the seed-paired comparison, not one row. Per skills.md §0 a single\n"
          "square proves nothing — this script exists so the question is settled\n"
          "before any sim time is booked, and before any filter code is written.")


if __name__ == "__main__":
    main()
