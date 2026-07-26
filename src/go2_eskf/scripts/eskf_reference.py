#!/usr/bin/env python3
"""NumPy reference implementation of the 8-state ESKF.

This is the "twin" of the C++ EskfCore (src/eskf_core.cpp). It exists so we can
cross-validate the C++ filter to ~1e-11: both consume the *same* sensor-stream
CSV and must produce the same state trajectory. Keeping the two in lock-step is
the cheapest, strongest regression test for the filter math.

State:  x = [px py pz  vx vy vz  psi  b_g]
Indices match go2_eskf::idx in types.hpp.
"""
import sys
import numpy as np

GRAVITY = 9.80665
PX, PY, PZ, VX, VY, VZ, PSI, BG = range(8)

# Measurement covariances — MUST match tools/replay_eskf.cpp.
LEG_R = np.diag([0.04, 0.04])
GPS_R = np.diag([0.25, 0.25])


def wrap_angle(a):
    return np.arctan2(np.sin(a), np.cos(a))


def rot_body_to_world(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def drot_dyaw(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    dRz = np.array([[-sy, -cy, 0], [cy, -sy, 0], [0, 0, 0]])
    return dRz @ Ry @ Rx


class EskfReference:
    def __init__(self, accel_noise=0.10, gyro_noise=2.0e-3,
                 gyro_bias_noise=1.0e-4, p0=1e-3):
        self.x = np.zeros(8)
        self.P = np.eye(8) * p0
        self.sa = accel_noise
        self.sg = gyro_noise
        self.sbg = gyro_bias_noise

    def predict_imu(self, accel_body, gyro_z, roll, pitch, dt):
        if dt <= 0.0:
            raise ValueError("dt must be > 0")
        psi = self.x[PSI]
        R = rot_body_to_world(roll, pitch, psi)
        a_world = R @ accel_body + np.array([0.0, 0.0, -GRAVITY])

        p = self.x[PX:PZ + 1]
        v = self.x[VX:VZ + 1]
        xp = self.x.copy()
        xp[PX:PZ + 1] = p + v * dt + 0.5 * a_world * dt * dt
        xp[VX:VZ + 1] = v + a_world * dt
        xp[PSI] = wrap_angle(psi + (gyro_z - self.x[BG]) * dt)
        xp[BG] = self.x[BG]

        F = np.eye(8)
        F[PX:PZ + 1, VX:VZ + 1] = np.eye(3) * dt
        da_dpsi = drot_dyaw(roll, pitch, psi) @ accel_body
        F[VX:VZ + 1, PSI] = da_dpsi * dt
        F[PX:PZ + 1, PSI] = 0.5 * da_dpsi * dt * dt
        F[PSI, BG] = -dt

        sa2, sg2, sbg2 = self.sa ** 2, self.sg ** 2, self.sbg ** 2
        Q = np.zeros((8, 8))
        Q[PX:PZ + 1, PX:PZ + 1] = np.eye(3) * (0.25 * sa2 * dt ** 4)
        Q[VX:VZ + 1, VX:VZ + 1] = np.eye(3) * (sa2 * dt ** 2)
        Q[PSI, PSI] = sg2 * dt ** 2
        Q[BG, BG] = sbg2 * dt

        self.P = F @ self.P @ F.T + Q
        self.x = xp

    def _joseph_update(self, y, H, R):
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[PSI] = wrap_angle(self.x[PSI])
        I_KH = np.eye(8) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

    def correct_leg_odom(self, v_body_meas, R=LEG_R):
        psi = self.x[PSI]
        c, s = np.cos(psi), np.sin(psi)
        Rz_T = np.array([[c, s], [-s, c]])
        v_world_xy = self.x[VX:VY + 1]
        h = Rz_T @ v_world_xy
        H = np.zeros((2, 8))
        H[:, VX:VY + 1] = Rz_T
        dRzT_dpsi = np.array([[-s, c], [-c, -s]])
        H[:, PSI] = dRzT_dpsi @ v_world_xy
        self._joseph_update(v_body_meas - h, H, R)

    def correct_gps(self, pos_xy, R=GPS_R):
        H = np.zeros((2, 8))
        H[0, PX] = 1.0
        H[1, PY] = 1.0
        h = self.x[PX:PY + 1]
        self._joseph_update(pos_xy - h, H, R)


def run_csv(input_csv, output_csv):
    """Replay an input stream and dump the state after every event."""
    f = EskfReference()
    rows = []
    data = np.loadtxt(input_csv, delimiter=",", skiprows=1, ndmin=2)
    for r in data:
        t = int(r[0])
        if t == 0:
            f.predict_imu(r[2:5], r[5], r[6], r[7], r[1])
        elif t == 1:
            f.correct_leg_odom(r[8:10])
        elif t == 2:
            f.correct_gps(r[8:10])
        rows.append(f.x.copy())
    np.savetxt(output_csv, np.array(rows), delimiter=",", fmt="%.17g")


def generate_input(path, n=600, seed=0):
    """Write a deterministic, plausible sensor stream for cross-validation."""
    rng = np.random.default_rng(seed)
    dt = 0.01
    lines = ["type,dt,a0,a1,a2,gz,roll,pitch,m0,m1"]
    for k in range(n):
        t = k * dt
        ax = 0.4 * np.sin(0.3 * t) + 0.02 * rng.standard_normal()
        ay = 0.15 * np.cos(0.2 * t) + 0.02 * rng.standard_normal()
        az = GRAVITY + 0.02 * rng.standard_normal()
        gz = 0.2 * np.sin(0.1 * t)
        roll = 0.03 * np.sin(0.5 * t)
        pitch = 0.05 * np.cos(0.4 * t)
        lines.append(f"0,{dt},{ax:.17g},{ay:.17g},{az:.17g},{gz:.17g},"
                     f"{roll:.17g},{pitch:.17g},0,0")
        if k % 5 == 0:  # leg odometry at 20 Hz
            vbx = 0.25 + 0.05 * np.sin(0.3 * t)
            vby = 0.02 * np.cos(0.2 * t)
            lines.append(f"1,0,0,0,0,0,0,0,{vbx:.17g},{vby:.17g}")
        if k % 50 == 0:  # GPS at 2 Hz
            gx = 0.25 * t + 0.1 * rng.standard_normal()
            gy = 0.05 * t + 0.1 * rng.standard_normal()
            lines.append(f"2,0,0,0,0,0,0,0,{gx:.17g},{gy:.17g}")
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    if len(sys.argv) == 3:
        run_csv(sys.argv[1], sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == "gen":
        generate_input(sys.argv[2])
    else:
        print("usage: eskf_reference.py <input.csv> <output.csv>", file=sys.stderr)
        sys.exit(1)
