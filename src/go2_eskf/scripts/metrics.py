#!/usr/bin/env python3
"""Phase 4 trajectory-accuracy metrics for the ESKF.

Operates on the CSV the node writes when `log_path` is set:

    t,est_x,est_y,est_z,est_yaw,est_vx,est_vy,est_bg,gt_x,gt_y,gt_yaw

Columns are resolved by NAME from the file's own header, so logs written before
est_bg existed still load.

Provides:
  * ATE  — Absolute Trajectory Error: RMSE of estimated vs ground-truth
           position after a rigid SE(2) (Umeyama, no scale) alignment. Global
           accuracy.
  * RPE  — Relative Pose Error over a fixed index gap: local drift rate.
  * final drift and drift-as-%-of-path-length.

Run as a module (imported by run_benchmark.py) or standalone:

    metrics.py <log.csv>          # print a metrics block for one run
    metrics.py --selftest         # synthetic sanity check (no ROS/sim needed)
"""
import sys

import numpy as np

LOG_COLUMNS = ["t", "est_x", "est_y", "est_z", "est_yaw",
               "est_vx", "est_vy", "est_bg", "gt_x", "gt_y", "gt_yaw"]


def _column_index(path):
    """Map column name -> index using the log's own header.

    Reading the header rather than assuming LOG_COLUMNS' order means adding a
    column to the node's log cannot silently shift which values get read as
    ground truth. Falls back to LOG_COLUMNS for a headerless file.
    """
    with open(path) as f:
        header = f.readline().strip().split(",")
    names = [h.strip() for h in header]
    if "gt_x" not in names:  # headerless / unrecognised: assume canonical order
        names = LOG_COLUMNS
    return {name: i for i, name in enumerate(names)}


def load_log(path):
    """Load a node log; drop rows whose ground truth is nan (not yet received)."""
    col = _column_index(path)
    rows = np.genfromtxt(path, delimiter=",", skip_header=1)
    if rows.ndim == 1:
        rows = rows[None, :]
    gt_cols = [col["gt_x"], col["gt_y"], col["gt_yaw"]]
    keep = ~np.isnan(rows[:, gt_cols]).any(axis=1)
    rows = rows[keep]
    if rows.shape[0] < 2:
        raise ValueError(f"{path}: fewer than 2 rows with valid ground truth")
    out = {
        "t": rows[:, col["t"]],
        "est_xy": rows[:, [col["est_x"], col["est_y"]]],
        "est_yaw": rows[:, col["est_yaw"]],
        "gt_xy": rows[:, [col["gt_x"], col["gt_y"]]],
        "gt_yaw": rows[:, col["gt_yaw"]],
    }
    # Gyro bias is absent from logs written before it was recorded; callers that
    # want it should check for the key rather than assume it.
    if "est_bg" in col and col["est_bg"] < rows.shape[1]:
        out["est_bg"] = rows[:, col["est_bg"]]
    return out


def align_se2(src, dst):
    """Rigid 2-D alignment (rotation+translation, no scale) mapping src->dst.

    Umeyama / Kabsch in 2-D. Returns the aligned src. Used so ATE measures shape
    error, not the arbitrary initial offset/heading of the estimator frame.
    """
    src = np.asarray(src)
    dst = np.asarray(dst)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    s0, d0 = src - mu_s, dst - mu_d
    H = s0.T @ d0
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:  # reflection guard
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = mu_d - R @ mu_s
    return (R @ src.T).T + t


def ate(est_xy, gt_xy, align=True):
    """Absolute Trajectory Error (position RMSE) after optional SE(2) alignment."""
    est = align_se2(est_xy, gt_xy) if align else np.asarray(est_xy)
    err = np.linalg.norm(est - gt_xy, axis=1)
    return float(np.sqrt(np.mean(err ** 2)))


def _se2(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0, 0, 1]])


def rpe(est_xy, est_yaw, gt_xy, gt_yaw, delta=10):
    """Relative Pose Error over a fixed index gap `delta`.

    Returns (trans_rmse [m], rot_rmse [rad]) of the relative motion error —
    a drift-rate measure that, unlike ATE, needs no global alignment.
    """
    n = len(est_xy)
    if n <= delta:
        raise ValueError("RPE delta exceeds trajectory length")
    trans, rot = [], []
    for i in range(n - delta):
        gt_rel = np.linalg.inv(_se2(*gt_xy[i], gt_yaw[i])) @ _se2(*gt_xy[i + delta], gt_yaw[i + delta])
        est_rel = np.linalg.inv(_se2(*est_xy[i], est_yaw[i])) @ _se2(*est_xy[i + delta], est_yaw[i + delta])
        err = np.linalg.inv(gt_rel) @ est_rel
        trans.append(np.hypot(err[0, 2], err[1, 2]))
        rot.append(abs(np.arctan2(err[1, 0], err[0, 0])))
    return float(np.sqrt(np.mean(np.square(trans)))), float(np.sqrt(np.mean(np.square(rot))))


def path_length(xy):
    return float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))


def final_drift(est_xy, gt_xy, align=True):
    est = align_se2(est_xy, gt_xy) if align else np.asarray(est_xy)
    return float(np.linalg.norm(est[-1] - gt_xy[-1]))


def compute_all(log, rpe_delta=10):
    est_xy, gt_xy = log["est_xy"], log["gt_xy"]
    rpe_t, rpe_r = rpe(est_xy, log["est_yaw"], gt_xy, log["gt_yaw"],
                       delta=min(rpe_delta, len(est_xy) - 1))
    plen = path_length(gt_xy)
    fdrift = final_drift(est_xy, gt_xy)
    return {
        "ate_m": ate(est_xy, gt_xy),
        "rpe_trans_m": rpe_t,
        "rpe_rot_rad": rpe_r,
        "final_drift_m": fdrift,
        "path_length_m": plen,
        "drift_pct": 100.0 * fdrift / plen if plen > 1e-6 else float("nan"),
        "n_samples": int(len(est_xy)),
    }


def format_metrics(name, m):
    return (f"[{name}]\n"
            f"  ATE (RMSE)        : {m['ate_m']:.4f} m\n"
            f"  RPE trans (RMSE)  : {m['rpe_trans_m']:.4f} m\n"
            f"  RPE rot   (RMSE)  : {m['rpe_rot_rad']:.4f} rad\n"
            f"  final drift       : {m['final_drift_m']:.4f} m "
            f"({m['drift_pct']:.2f}% of {m['path_length_m']:.2f} m path)\n"
            f"  samples           : {m['n_samples']}")


def _selftest():
    """Synthesize a path + a rotated/translated/noisy estimate; check metrics."""
    rng = np.random.default_rng(0)
    t = np.linspace(0, 20, 500)
    gt_xy = np.column_stack([4 * np.cos(0.3 * t), 4 * np.sin(0.3 * t)])
    gt_yaw = 0.3 * t + np.pi / 2

    theta = 0.5  # estimator started with a different heading + offset
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    noise = rng.normal(0, 0.05, gt_xy.shape)
    est_xy = (R @ gt_xy.T).T + np.array([2.0, -1.0]) + noise
    est_yaw = gt_yaw + theta

    log = {"est_xy": est_xy, "est_yaw": est_yaw, "gt_xy": gt_xy, "gt_yaw": gt_yaw}
    m = compute_all(log)
    print(format_metrics("selftest", m))

    ate_unaligned = ate(est_xy, gt_xy, align=False)
    assert m["ate_m"] < 0.1, f"aligned ATE too high: {m['ate_m']}"
    assert ate_unaligned > 1.0, "alignment should remove the large offset"
    assert m["rpe_trans_m"] < 0.1, f"RPE trans too high: {m['rpe_trans_m']}"
    print(f"\nunaligned ATE = {ate_unaligned:.3f} m (offset present, as expected)")
    print("SELFTEST PASS")
    return 0


def main():
    if "--selftest" in sys.argv:
        return _selftest()
    if len(sys.argv) < 2:
        print("usage: metrics.py <log.csv> | metrics.py --selftest", file=sys.stderr)
        return 1
    log = load_log(sys.argv[1])
    print(format_metrics(sys.argv[1], compute_all(log)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
