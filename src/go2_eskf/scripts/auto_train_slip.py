#!/usr/bin/env python3
"""Retrain the slip MLP after every run, and deploy it only if it does not regress.

run_go2_teleop.sh calls this from cleanup() once every node has stopped (so the
live slip log is closed). One invocation:

  1. ARCHIVE the run's slip-feature log into a persistent dataset directory.
     run_report/ is overwritten every run, so without this each run's training
     data was thrown away. Rows without ground truth carry no label and are
     dropped; so are rows where the robot is tipped over (gt_tilt > --max-tilt):
     a fallen robot's feet scrabble and its leg odometry is garbage, which is not
     the distribution the model runs on. Runs with fewer than --min-rows usable
     rows are not archived.
  2. EVALUATE ON A HELD-OUT RUN. The rows are a time series, so a random row
     split is optimistic (skills.md §0: "train on one run, evaluate on others").
     A candidate is trained on every archived run EXCEPT the newest and scored on
     the newest, as is the deployed model — neither has seen it. The score is the
     BCE the trainer minimises, against the soft label clip(|v_leg - v_truth| /
     0.3, 0, 1).
  3. DEPLOY only if the candidate's held-out BCE is no worse than the deployed
     model's (+ --tol). The deployed weights are then retrained on ALL archived
     runs, checked in C++ (tools/slip_infer must parse the file and agree with
     NumPy to 1e-9, per CLAUDE.md), the previous file is copied to
     config/slip_model_history/, and config/slip_model.txt is replaced atomically.
     The slip arm loads it at the NEXT run's startup.
  4. REPORT: a markdown summary, printed and (with --report) appended to REPORT.md.

Needs at least two archived runs to evaluate; with one it only archives.

  auto_train_slip.py --run-log run_report/slip_features.csv \\
      [--dataset slip_dataset] [--model config/slip_model.txt] \\
      [--report run_report/REPORT.md] [--context run_report/context.txt]
"""
import argparse
import datetime
import glob
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slip_reference as ref  # noqa: E402
import train_slip_model as tsm  # noqa: E402

WS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def predict_batch(model, X):
    """Vectorised twin of SlipModel.predict (same maths, all rows at once)."""
    h = ((X - model.mean) / model.std).T
    for W, b, act in model.layers:
        h = ref._ACT[act](W @ h + b[:, None])
    return h[0]


def bce(p, y):
    p = np.clip(p, 1e-7, 1.0 - 1e-7)
    return float(-np.mean(y * np.log(p) + (1.0 - y) * np.log(1.0 - p)))


def quality(model, X, y, err):
    p = predict_batch(model, X)
    corr = float(np.corrcoef(p, err)[0, 1]) if p.std() > 0 else float("nan")
    return dict(bce=bce(p, y), corr=corr, spread=float(p.max() - p.min()),
                mean=float(p.mean()))


def fit(paths, label_scale, hidden, epochs):
    X, y, _ = tsm.load_runlogs(paths, label_scale)
    mean, std = tsm.standardize(X)
    layers = tsm.train_torch((X - mean) / std, y, hidden, epochs)
    return ref.SlipModel(tsm.DIM, mean, std, layers), len(y)


def archive(run_log, dataset, max_tilt, min_rows, context):
    """Copy the usable rows of this run's log into the dataset. Returns (path, note)."""
    if not os.path.exists(run_log):
        return None, f"no slip log at `{run_log}` (no ground truth this run?)"
    d = np.genfromtxt(run_log, delimiter=",", names=True, invalid_raise=False)
    d = np.atleast_1d(d)
    if d.size == 0:
        return None, "slip log is empty"
    names = d.dtype.names
    keep = np.isfinite(d["gt_vx"]) & np.isfinite(d["gt_vy"])
    n_nolabel = int((~keep).sum())
    n_tilt = 0
    if "gt_tilt" in names:
        tipped = keep & ~(d["gt_tilt"] <= np.radians(max_tilt))
        n_tilt = int(tipped.sum())
        keep &= ~tipped
    n = int(keep.sum())
    if n < min_rows:
        return None, (f"only {n} usable rows (< {min_rows}; {n_nolabel} without "
                      f"truth, {n_tilt} tipped over) — not archived")
    os.makedirs(dataset, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    out = os.path.join(dataset, f"run_{stamp}.csv")
    with open(run_log) as src:
        header = src.readline().rstrip("\n")
    rows = d[keep]
    with open(out, "w") as f:
        f.write(header + "\n")
        for r in rows:
            f.write(",".join(repr(float(r[c])) for c in names) + "\n")
    cmd = ""
    if context and os.path.exists(context):
        for line in open(context):
            if line.startswith("Command line:"):
                cmd = line.split(":", 1)[1].strip()
    with open(out[:-4] + ".meta", "w") as f:
        f.write(f"command: {cmd}\nrows: {n}\nno_truth_dropped: {n_nolabel}\n"
                f"tipped_dropped: {n_tilt}\n")
    return out, (f"archived {n} rows as `{os.path.relpath(out, WS)}` "
                 f"({n_nolabel} without truth and {n_tilt} tipped over dropped)")


def cpp_check(weights, X):
    """The C++ runtime must parse the file and agree with NumPy (CLAUDE.md)."""
    binary = None
    for c in (os.path.join(WS, "install/go2_eskf/lib/go2_eskf/slip_infer"),
              os.path.join(WS, "install/lib/go2_eskf/slip_infer"),
              os.path.join(WS, "build/go2_eskf/slip_infer")):
        if os.path.exists(c):
            binary = c
            break
    if binary is None:
        return False, "slip_infer binary not found — build go2_eskf"
    tmp = tempfile.mkdtemp(prefix="slip_auto_")
    feats, out = os.path.join(tmp, "f.csv"), os.path.join(tmp, "o.csv")
    sample = X[:: max(1, len(X) // 500)]
    np.savetxt(feats, sample, delimiter=",", fmt="%.17g",
               header=",".join(ref.SLIP_FEATURES), comments="")
    r = subprocess.run([binary, weights, feats, out], capture_output=True, text=True)
    if r.returncode != 0:
        return False, f"slip_infer failed: {r.stderr.strip()}"
    cpp = np.loadtxt(out, ndmin=1)
    py = predict_batch(ref.SlipModel.load(weights), sample)
    diff = float(np.abs(cpp - py).max())
    return diff <= 1e-9, f"C++ vs NumPy max diff {diff:.1e} over {len(sample)} rows"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-log", required=True)
    ap.add_argument("--dataset", default=os.path.join(WS, "slip_dataset"))
    ap.add_argument("--model", default=os.path.join(HERE, "..", "config", "slip_model.txt"))
    ap.add_argument("--report", default=None, help="markdown file to append to")
    ap.add_argument("--context", default=None, help="run_report/context.txt")
    ap.add_argument("--label-scale", type=float, default=0.3)
    ap.add_argument("--hidden", type=int, default=16)
    ap.add_argument("--epochs", type=int, default=1500)   # as the 2026-08-11 model
    ap.add_argument("--max-tilt", type=float, default=30.0, help="[deg]")
    ap.add_argument("--min-rows", type=int, default=500)
    ap.add_argument("--tol", type=float, default=1e-3, help="BCE slack for deploy")
    args = ap.parse_args()
    model_path = os.path.abspath(args.model)

    try:
        import torch
        torch.set_num_threads(max(1, (os.cpu_count() or 2) // 2))
    except ImportError:
        print("auto_train_slip: PyTorch not installed — skipping (the NumPy "
              "fallback is a linear model, not a replacement for the MLP).")
        return 0

    lines = ["", "## Slip model training (after this run)", ""]
    out, note = archive(args.run_log, args.dataset, args.max_tilt, args.min_rows,
                        args.context)
    lines.append(f"- data: {note}")
    runs = sorted(glob.glob(os.path.join(args.dataset, "run_*.csv")))
    lines.append(f"- dataset: {len(runs)} archived run(s) in "
                 f"`{os.path.relpath(args.dataset, WS)}/`")

    def finish(verdict):
        lines.append(f"- **{verdict}**")
        lines.append("")
        text = "\n".join(lines)
        print(text)
        if args.report:
            with open(args.report, "a") as f:
                f.write(text + "\n")
        return 0

    if out is None:
        return finish("NOT retrained — no new data. Deployed model unchanged.")
    if len(runs) < 2:
        return finish("NOT retrained — need >= 2 archived runs to evaluate on a "
                      "held-out run. Data archived; the next run can train.")

    newest, others = runs[-1], runs[:-1]
    Xh, yh, eh = tsm.load_runlogs([newest], args.label_scale)
    cand, n_fit = fit(others, args.label_scale, args.hidden, args.epochs)
    qc = quality(cand, Xh, yh, eh)
    try:
        qd = quality(ref.SlipModel.load(model_path), Xh, yh, eh)
    except Exception as e:  # missing / unreadable deployed model: anything beats it
        qd = dict(bce=float("inf"), corr=float("nan"), spread=float("nan"),
                  mean=float("nan"))
        lines.append(f"- deployed model unreadable ({e}) — treated as worst")
    lines.append(f"- held-out run `{os.path.basename(newest)}` ({len(yh)} rows), "
                 f"never seen by either model:")
    lines.append("")
    lines.append("  | model | BCE (lower = better) | corr(score, leg-odom error) "
                 "| score spread | mean score |")
    lines.append("  |---|---|---|---|---|")
    for name, q in ((f"candidate (trained on {len(others)} other run(s), "
                     f"{n_fit} rows)", qc), ("deployed", qd)):
        lines.append(f"  | {name} | {q['bce']:.4f} | {q['corr']:+.3f} | "
                     f"{q['spread']:.3f} | {q['mean']:.3f} |")
    lines.append("")

    if not qc["bce"] <= qd["bce"] + args.tol:
        return finish(f"NOT deployed — the candidate is worse on the held-out run "
                      f"(BCE {qc['bce']:.4f} vs {qd['bce']:.4f}). Deployed model "
                      "unchanged; data archived.")

    final, n_all = fit(runs, args.label_scale, args.hidden, args.epochs)
    tmp = model_path + ".new"
    final.save(tmp, [
        f"trained {datetime.datetime.now().isoformat(timespec='seconds')} by "
        f"auto_train_slip.py",
        f"hidden={args.hidden} epochs={args.epochs} rows={n_all} runs={len(runs)}",
        "source: " + " ".join(os.path.basename(r) for r in runs),
        f"label = clip(|v_leg - v_truth| / {args.label_scale}, 0, 1)",
        f"held-out check on {os.path.basename(newest)}: candidate BCE "
        f"{qc['bce']:.4f} vs previous {qd['bce']:.4f}",
    ])
    ok, msg = cpp_check(tmp, Xh)
    lines.append(f"- C++ runtime check: {msg}")
    if not ok:
        os.remove(tmp)
        return finish("NOT deployed — the C++ check failed. Deployed model unchanged.")
    if os.path.exists(model_path):
        hist = os.path.join(os.path.dirname(model_path), "slip_model_history")
        os.makedirs(hist, exist_ok=True)
        backup = os.path.join(hist, "slip_model_" +
                              datetime.datetime.now().strftime("%Y%m%d-%H%M%S") + ".txt")
        shutil.copy2(model_path, backup)
        lines.append(f"- previous weights backed up to "
                     f"`{os.path.relpath(backup, WS)}`")
    os.replace(tmp, model_path)
    return finish(f"DEPLOYED — retrained on all {len(runs)} runs ({n_all} rows); "
                  "the slip arm loads it at the next run's startup.")


if __name__ == "__main__":
    sys.exit(main())
