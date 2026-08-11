#!/usr/bin/env python3
"""Offline model of CHAMP's leg odometry, with an exactly known body twist.

Why this exists: square-test drift numbers in this workspace are NOT repeatable
(4-97 deg of yaw error for identical configs -- see skills.md §0), so a live run
cannot resolve an 8% sensor error. This harness simulates a Go2 trot in the world
frame from a COMMANDED twist, generates the base-frame foot positions
`champ::Odometry::getVelocities` would see, and runs the estimator on them. The
truth is exact and the result has zero run-to-run variance.

It is how the bearing-sum yaw-rate defect was found and how the least-squares
replacement (champ/include/champ/odometry/odometry.h) was verified. Keep the
`champ_lsq` port here in step with that header -- it is the tripwire if the two
ever drift apart.

    python3 src/go2_eskf/scripts/leg_odom_model.py

Estimators:
  bearing      -- the original vendored formula (x_sum/2, theta_sum/total_contact)
  lsq          -- least-squares body twist, but fusing touchdown samples
  lsq+td       -- ... and ignoring feet not in contact on the PREVIOUS sample
  champ_lsq    -- exactly what the C++ now does (lsq+td, odom_scaler, beta_)
"""
import sys

import numpy as np

DT = 0.02          # 50 Hz, champ's state_estimation timer
CYCLE = 0.5        # gait cycle [s]
DUTY = 0.5         # stance fraction (trot: diagonal pairs alternate)
BETA = 0.1         # champ's output smoothing
ODOM_SCALER = 0.9  # gait.yaml -- a real-robot slip fudge, undone filter-side by
                   # go2_eskf's leg_odom_scale: 1.111

# Nominal stance foot positions in the base frame (Go2-ish).
R_NOM = np.array([[+0.19, +0.14],   # FL
                  [+0.19, -0.14],   # FR
                  [-0.19, +0.14],   # RL
                  [-0.19, -0.14]])  # RR
PHASE_OFF = np.array([0.0, 0.5, 0.5, 0.0])  # trot: FL+RR vs FR+RL


def rot(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s], [s, c]])


def simulate(vx, vy, wz, T=6.0):
    """Walk the base on an exact twist; return per-sample (foot_base, contacts)."""
    n = int(T / DT)
    pb, psi = np.zeros(2), 0.0
    foot_b = R_NOM.copy()
    foot_w = np.array([pb + rot(psi) @ foot_b[i] for i in range(4)])
    v_body = np.array([vx, vy])
    lead = 0.5 * DUTY * CYCLE * v_body   # half a stride

    out = []
    prev_contact = np.ones(4, dtype=bool)
    for k in range(n):
        phase = (k * DT / CYCLE + PHASE_OFF) % 1.0
        contact = phase < DUTY
        for i in range(4):
            if contact[i]:
                if not prev_contact[i]:
                    # Touchdown: plant half a stride ahead so stance sweeps
                    # symmetrically about the nominal position.
                    foot_w[i] = pb + rot(psi) @ (R_NOM[i] + lead)
                foot_b[i] = rot(-psi) @ (foot_w[i] - pb)
            else:
                u = (phase[i] - DUTY) / (1.0 - DUTY)
                s = u * u * (3 - 2 * u)              # smoothstep swing
                foot_b[i] = (R_NOM[i] - lead) + s * (2 * lead)

        out.append((foot_b.copy(), contact.copy()))
        prev_contact = contact.copy()
        pb = pb + rot(psi) @ v_body * DT
        psi = psi + wz * DT
    return out


def bearing(frames):
    """The original vendored formula, line for line (post the theta_sum/n fix)."""
    prev_pos = frames[0][0].copy()
    prev_theta = np.arctan2(prev_pos[:, 0], prev_pos[:, 1])  # atan2f(X, Y) as written
    prev_vel = np.zeros(3)
    est = []
    for pos, contact in frames[1:]:
        theta = np.arctan2(pos[:, 0], pos[:, 1])
        if contact.all() or (~contact).all():
            prev_vel = np.zeros(3)
            prev_pos, prev_theta = pos.copy(), theta
            est.append(np.zeros(3))
            continue
        x_sum = y_sum = th_sum = 0.0
        n_contact = 0
        for i in range(4):
            if contact[i]:
                n_contact += 1
                th_sum += theta[i] - prev_theta[i]
                x_sum += (prev_pos[i, 0] - pos[i, 0]) / 2.0
                y_sum += (prev_pos[i, 1] - pos[i, 1]) / 2.0
        n = max(n_contact, 1)
        v = np.array([
            (1 - BETA) * (x_sum * ODOM_SCALER / DT) + BETA * prev_vel[0],
            (1 - BETA) * (y_sum * ODOM_SCALER / DT) + BETA * prev_vel[1],
            (1 - BETA) * ((th_sum / n) / DT) + BETA * prev_vel[2],
        ])
        est.append(v)
        prev_vel, prev_pos, prev_theta = v, pos.copy(), theta
    return np.array(est)


def lsq(frames, require_prev_contact=True, scale=1.0, early_return=False):
    """Body twist from stance feet: -dr_i/dt = v + w x r_i, centroid-reduced.

    This is the algorithm now in champ/odometry.h; `champ_lsq` below is the
    faithful configuration of it.
    """
    prev_pos = frames[0][0].copy()
    prev_contact = frames[0][1].copy()
    prev_vel = np.zeros(3)
    est = []
    for pos, contact in frames[1:]:
        if early_return and (contact.all() or (~contact).all()):
            # champ's untouched "nothing to calculate" branch. It is a
            # no-information FLAG that go2_eskf's ZUPT path keys on, so the
            # least-squares change deliberately leaves it alone.
            prev_vel = np.zeros(3)
            prev_pos, prev_contact = pos.copy(), contact.copy()
            est.append(np.zeros(3))
            continue

        use = contact & prev_contact if require_prev_contact else contact
        r_mid = 0.5 * (pos + prev_pos)
        q = -(pos - prev_pos) / DT
        if not use.any():
            est.append(prev_vel.copy())
            prev_pos, prev_contact = pos.copy(), contact.copy()
            continue

        r_bar, q_bar = r_mid[use].mean(axis=0), q[use].mean(axis=0)
        p_, q_ = r_mid[use] - r_bar, q[use] - q_bar
        num = float((p_[:, 0] * q_[:, 1] - p_[:, 1] * q_[:, 0]).sum())
        den = float((p_ ** 2).sum())
        w = num / den if den > 1e-9 else prev_vel[2]
        sol = np.array([(q_bar[0] + w * r_bar[1]) * scale,
                        (q_bar[1] - w * r_bar[0]) * scale,
                        w])
        v = (1 - BETA) * sol + BETA * prev_vel
        est.append(v)
        prev_vel, prev_pos, prev_contact = v, pos.copy(), contact.copy()
    return np.array(est)


def champ_lsq(frames):
    return lsq(frames, require_prev_contact=True, scale=ODOM_SCALER,
               early_return=True)


CASES = (("STRAIGHT", 0.25, 0.0, 0.0),
         ("TURN", 0.0, 0.0, 0.4),
         ("ARC", 0.25, 0.0, 0.4),
         ("SLOW ARC", 0.15, 0.0, -0.2))

ESTIMATORS = (("bearing", bearing),
              ("lsq", lambda f: lsq(f, require_prev_contact=False)),
              ("lsq+td", lambda f: lsq(f, require_prev_contact=True)),
              ("champ_lsq", champ_lsq))


def main():
    skip = int(1.0 / DT)   # let the beta_ smoother settle
    worst = 0.0
    for label, vx, vy, wz in CASES:
        frames = simulate(vx, vy, wz)
        print(f"\n=== {label}: truth vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} ===")
        print(f"{'estimator':10s} {'vx mean':>9s} {'vx/tru':>7s} "
              f"{'wz mean':>9s} {'wz/tru':>7s} {'wz std':>8s}")
        for name, fn in ESTIMATORS:
            e = fn(frames)[skip:]
            vxm, wzm = e[:, 0].mean(), e[:, 2].mean()
            vr = vxm / vx if abs(vx) > 1e-9 else float('nan')
            wr = wzm / wz if abs(wz) > 1e-9 else float('nan')
            print(f"{name:10s} {vxm:+9.4f} {vr:7.3f} "
                  f"{wzm:+9.4f} {wr:7.3f} {e[:, 2].std():8.4f}")
            if name == "champ_lsq":
                # The shipped estimator must recover the twist exactly, up to
                # odom_scaler on the linear channel.
                if abs(vx) > 1e-9:
                    worst = max(worst, abs(vr - ODOM_SCALER))
                if abs(wz) > 1e-9:
                    worst = max(worst, abs(wr - 1.0))
                worst = max(worst, float(e[:, 2].std()))

    print(f"\nchamp_lsq worst deviation from exact: {worst:.2e}")
    if worst > 1e-3:
        print("FAIL: the port no longer matches the C++ contract", file=sys.stderr)
        return 1
    print("PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
