#!/usr/bin/env python3
"""Phase 4 benchmark: compare ESKF configurations and emit a report + plots.

Consumes node CSV logs (written when `log_path` is set) for one or more named
scenarios — typically:
  * fixed     : fixed leg covariance (use_slip_model:=false)
  * adaptive  : slip-adaptive covariance (use_slip_model:=true)
  * gps_denied: GPS disabled mid-run (use_gps:=false)

and produces:
  * a markdown table of ATE / RPE / drift for every scenario,
  * a trajectory-overlay plot (each estimate SE(2)-aligned to ground truth),
  * a position-error-over-time plot.

Collecting the logs needs a running sim (see launch/benchmark.launch.py); this
script is the offline analysis stage and runs anywhere matplotlib is installed.

Usage:
  run_benchmark.py --log fixed=fixed.csv --log adaptive=adaptive.csv \
                   --out docs/benchmark
  run_benchmark.py --demo --out /tmp/benchmark_demo   # synthetic, no sim needed
"""
import argparse
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402


def write_report(results, out_md):
    lines = [
        "# go2_eskf — Phase 4 benchmark",
        "",
        "Absolute Trajectory Error (ATE), Relative Pose Error (RPE) over a fixed",
        "index gap, and terminal drift. ATE/drift use a rigid SE(2) alignment of",
        "the estimate to ground truth.",
        "",
        "| scenario | ATE [m] | RPE trans [m] | RPE rot [rad] | final drift [m] | drift % | samples |",
        "|----------|--------:|--------------:|--------------:|----------------:|--------:|--------:|",
    ]
    for name, m in results.items():
        lines.append(
            f"| {name} | {m['ate_m']:.4f} | {m['rpe_trans_m']:.4f} | "
            f"{m['rpe_rot_rad']:.4f} | {m['final_drift_m']:.4f} | "
            f"{m['drift_pct']:.2f} | {m['n_samples']} |")
    lines.append("")
    with open(out_md, "w") as f:
        f.write("\n".join(lines) + "\n")
    return "\n".join(lines)


def plot_trajectories(logs, out_png):
    plt.figure(figsize=(7, 7))
    first = next(iter(logs.values()))
    plt.plot(first["gt_xy"][:, 0], first["gt_xy"][:, 1],
             "k-", lw=2.5, label="ground truth")
    for name, log in logs.items():
        est = M.align_se2(log["est_xy"], log["gt_xy"])
        plt.plot(est[:, 0], est[:, 1], "--", lw=1.5, label=f"{name} (aligned)")
    plt.axis("equal")
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.title("Trajectory: ground truth vs estimates")
    plt.legend(); plt.grid(True, alpha=0.3)
    plt.tight_layout(); plt.savefig(out_png, dpi=120); plt.close()


def plot_error_over_time(logs, out_png):
    plt.figure(figsize=(9, 4))
    for name, log in logs.items():
        est = M.align_se2(log["est_xy"], log["gt_xy"])
        err = np.linalg.norm(est - log["gt_xy"], axis=1)
        plt.plot(log["t"] - log["t"][0], err, lw=1.3, label=name)
    plt.xlabel("time [s]"); plt.ylabel("position error [m]")
    plt.title("Aligned position error over time")
    plt.legend(); plt.grid(True, alpha=0.3)
    plt.tight_layout(); plt.savefig(out_png, dpi=120); plt.close()


def make_demo_logs(out_dir):
    """Synthesize node-format logs for three scenarios (no sim required).

    Encodes the expected qualitative story: adaptive covariance tracks better
    than fixed through a slippery stretch, and GPS-denied drifts away with no
    global anchor. Lets the full report/plot pipeline run end-to-end.
    """
    rng = np.random.default_rng(0)
    t = np.linspace(0, 30, 900)
    gt_xy = np.column_stack([5 * np.cos(0.2 * t), 5 * np.sin(0.2 * t)])
    gt_yaw = M.np.unwrap(0.2 * t + np.pi / 2)
    slip_window = (t > 12) & (t < 18)  # leg odometry unreliable here

    def write(name, est_xy, est_yaw):
        path = os.path.join(out_dir, f"{name}.csv")
        with open(path, "w") as f:
            f.write(",".join(M.LOG_COLUMNS) + "\n")
            for i in range(len(t)):
                # est_z, est_vx, est_vy, est_bg are unused by the metrics.
                f.write(f"{t[i]:.6f},{est_xy[i,0]:.6f},{est_xy[i,1]:.6f},0,"
                        f"{est_yaw[i]:.6f},0,0,0,"
                        f"{gt_xy[i,0]:.6f},{gt_xy[i,1]:.6f},{gt_yaw[i]:.6f}\n")
        return path

    base_noise = rng.normal(0, 0.03, gt_xy.shape)

    # Fixed covariance: trusts bad leg odometry during the slip window -> bias.
    fixed = gt_xy + base_noise.copy()
    bump = np.cumsum(slip_window[:, None] * np.array([0.012, 0.008]), axis=0)
    fixed += bump
    # Adaptive: down-weights leg odometry during slip -> much smaller bump.
    adaptive = gt_xy + base_noise.copy() + 0.12 * bump
    # GPS-denied: slow unbounded drift (no global anchor).
    drift = np.cumsum(np.full(gt_xy.shape, 0.004), axis=0)
    gps_denied = gt_xy + base_noise.copy() + drift

    return {
        "fixed": M.load_log(write("fixed", fixed, gt_yaw)),
        "adaptive": M.load_log(write("adaptive", adaptive, gt_yaw)),
        "gps_denied": M.load_log(write("gps_denied", gps_denied, gt_yaw)),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log", action="append", default=[],
                    metavar="NAME=PATH", help="scenario label and CSV path")
    ap.add_argument("--out", default="benchmark_out", help="output directory")
    ap.add_argument("--rpe-delta", type=int, default=10)
    ap.add_argument("--demo", action="store_true",
                    help="synthesize logs instead of reading them")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)

    if args.demo:
        logs = make_demo_logs(args.out)
    else:
        if not args.log:
            print("provide --log NAME=PATH (or --demo)", file=sys.stderr)
            return 1
        logs = {}
        for spec in args.log:
            name, _, path = spec.partition("=")
            logs[name] = M.load_log(path)

    results = {n: M.compute_all(l, rpe_delta=args.rpe_delta)
               for n, l in logs.items()}

    report = write_report(results, os.path.join(args.out, "benchmark_report.md"))
    plot_trajectories(logs, os.path.join(args.out, "trajectories.png"))
    plot_error_over_time(logs, os.path.join(args.out, "error_over_time.png"))

    print(report)
    print(f"\nwrote report + plots to {os.path.abspath(args.out)}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
