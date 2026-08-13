#!/usr/bin/env python3
"""Train the Phase 3 slip detector and export it for the C++ runtime.

The model maps the locomotion feature vector (see slip_reference.SLIP_FEATURES)
to a slip score s in [0, 1]. It is exported in the plain-text weights format
read by include/go2_eskf/slip_model.hpp, so inference in the live node is a few
Eigen matmuls with no ML runtime dependency.

Two training backends, picked automatically:
  * PyTorch (if installed): a 2-hidden-layer MLP trained with BCEWithLogitsLoss.
  * NumPy fallback: logistic regression by gradient descent. Same exported
    format, so the pipeline runs anywhere (CI, machines without torch) and still
    yields an adaptive model — just linear instead of an MLP.

Data:
  * --runlog <csv>...: RECOMMENDED. The slip-feature logs written live by
                    eskf_node (`slip_log:=...`, which run_go2_teleop.sh sets to
                    run_report/slip_features.csv whenever the ground-truth bridge
                    is up). Rows are logged from exactly the code path that runs
                    inference, and are labelled here by how wrong leg odometry
                    actually was:

                        label = clip(|v_leg - v_truth| / label_scale, 0, 1)

                    That is the quantity R_leg should be inflated for, so the
                    model is trained on its real job rather than on a proxy.
  * --data <csv>  : pre-labelled features. Columns = SLIP_FEATURES plus a final
                    'slip' label column in [0,1].
  * default       : a physics-inspired synthetic dataset (synthesize_dataset),
                    so the pipeline is runnable before real logs exist. A model
                    trained this way has never seen the robot and should not be
                    trusted in a live A/B.

Usage:
  train_slip_model.py --runlog run_report/slip_features.csv [more.csv ...]
                      [--label-scale 0.3] [--out config/slip_model.txt]
                      [--hidden 16] [--epochs 300]
"""
import argparse
import datetime
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slip_reference as ref  # noqa: E402

DIM = ref.DIM
F = {name: i for i, name in enumerate(ref.SLIP_FEATURES)}


def synthesize_dataset(n=8000, seed=0):
    """Physics-inspired synthetic slip data.

    A latent slip state drives the observable disagreements: under slip the
    commanded/joint motion persists while the measured body velocity and foot
    contact collapse, and horizontal acceleration spikes. Real training should
    replace this with logged sim/hardware data via --data.
    """
    rng = np.random.default_rng(seed)
    X = np.zeros((n, DIM))
    y = rng.random(n) < 0.4  # ~40% slipping
    for k in range(n):
        slip = y[k]
        cmd_v = rng.uniform(0.0, 0.6)          # commanded forward speed
        cmd_w = rng.uniform(-0.4, 0.4)
        # Under slip the body moves far less than commanded.
        track = rng.uniform(0.0, 0.3) if slip else rng.uniform(0.7, 1.0)
        leg_v = cmd_v * track + rng.normal(0, 0.02)
        gyro = cmd_w * track + rng.normal(0, 0.02)
        joint_mean = 0.5 + 4.0 * cmd_v + rng.normal(0, 0.1)  # legs keep cycling
        joint_max = joint_mean * rng.uniform(2.0, 3.0)
        accel = (rng.uniform(2.0, 6.0) if slip else rng.uniform(0.0, 1.5))
        contact = (rng.uniform(0.0, 0.5) if slip else rng.uniform(0.75, 1.0))

        X[k, F["cmd_minus_leg_vx"]] = cmd_v - leg_v
        X[k, F["cmd_minus_leg_vy"]] = rng.normal(0, 0.03)
        X[k, F["cmd_minus_gyro_wz"]] = cmd_w - gyro
        X[k, F["leg_speed"]] = abs(leg_v)
        X[k, F["joint_vel_mean"]] = joint_mean
        X[k, F["joint_vel_max"]] = joint_max
        X[k, F["accel_horiz"]] = accel
        X[k, F["contact_frac"]] = contact
    return X, y.astype(float)


def load_dataset(path):
    data = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    if data.shape[1] != DIM + 1:
        raise SystemExit(f"--data must have {DIM} feature columns + 1 label "
                         f"column; got {data.shape[1]}")
    return data[:, :DIM], data[:, DIM]


def load_runlogs(paths, label_scale):
    """Live slip-feature logs -> (X, y, err) with truth-derived soft labels.

    The label is how badly leg odometry disagreed with ground truth at that
    instant, normalized by --label-scale and clipped to [0, 1]: a direct proxy
    for the leg-odom measurement error whose variance R_leg is supposed to
    describe. Rows without ground truth carry no label and are dropped.
    """
    Xs, errs = [], []
    for p in paths:
        # invalid_raise=False: the log is written live and the process is killed
        # at teardown, so the final line is routinely a half-flushed fragment.
        d = np.genfromtxt(p, delimiter=",", names=True, invalid_raise=False)
        if d.size == 0:
            print(f"  {p}: EMPTY, skipped")
            continue
        d = np.atleast_1d(d)
        missing = [f for f in ref.SLIP_FEATURES if f not in d.dtype.names]
        if missing:
            raise SystemExit(f"{p}: missing feature columns {missing}")
        X = np.column_stack([d[f] for f in ref.SLIP_FEATURES])
        err = np.hypot(d["leg_vx"] - d["gt_vx"], d["leg_vy"] - d["gt_vy"])
        keep = np.isfinite(err) & np.isfinite(X).all(axis=1)
        n_drop = len(err) - int(keep.sum())
        print(f"  {p}: {len(err)} rows, {n_drop} unlabelled (no ground truth) "
              f"dropped")
        Xs.append(X[keep])
        errs.append(err[keep])
    if not Xs:
        raise SystemExit("no usable rows in the --runlog files")
    X = np.vstack(Xs)
    err = np.concatenate(errs)
    y = np.clip(err / label_scale, 0.0, 1.0)
    print(f"leg-odom velocity error |v_leg - v_truth| [m/s]: "
          f"mean {err.mean():.3f}, median {np.median(err):.3f}, "
          f"p90 {np.percentile(err, 90):.3f}, max {err.max():.3f}")
    print(f"labels (err/{label_scale} clipped): mean {y.mean():.3f}, "
          f"median {np.median(y):.3f}, frac>0.5 {(y > 0.5).mean():.3f}")
    if not np.isfinite(X[:, F["contact_frac"]]).any() or \
            X[:, F["contact_frac"]].std() < 1e-9:
        print("WARNING: contact_frac is constant in this data — either "
              "/foot_contacts was not connected or the gait never lifted a "
              "foot. That feature carries no information here.")
    return X, y, err


def standardize(X):
    mean = X.mean(axis=0)
    std = X.std(axis=0)
    std[std < 1e-6] = 1.0
    return mean, std


def train_torch(Xn, y, hidden, epochs, val=None):
    import torch
    import torch.nn as nn

    torch.manual_seed(0)
    Xt = torch.tensor(Xn, dtype=torch.float32)
    yt = torch.tensor(y, dtype=torch.float32).unsqueeze(1)
    net = nn.Sequential(
        nn.Linear(DIM, hidden), nn.ReLU(),
        nn.Linear(hidden, hidden), nn.ReLU(),
        nn.Linear(hidden, 1),  # logits; sigmoid applied at export/inference
    )
    opt = torch.optim.Adam(net.parameters(), lr=1e-2)
    # Soft targets in [0,1] are fine here: BCEWithLogitsLoss is cross-entropy
    # against the target, minimized when the sigmoid output equals the label —
    # so the trained score reproduces the normalized leg-odom error itself.
    loss_fn = nn.BCEWithLogitsLoss()
    for ep in range(epochs):
        opt.zero_grad()
        loss = loss_fn(net(Xt), yt)
        loss.backward()
        opt.step()
        if val is not None and (ep + 1) % max(1, epochs // 5) == 0:
            with torch.no_grad():
                vX = torch.tensor(val[0], dtype=torch.float32)
                vy = torch.tensor(val[1], dtype=torch.float32).unsqueeze(1)
                vloss = loss_fn(net(vX), vy).item()
            print(f"  epoch {ep + 1:5d}  train {loss.item():.4f}  "
                  f"val {vloss:.4f}")
    print(f"[torch] final BCE loss = {loss.item():.4f}")

    layers, acts = [], ["relu", "relu", "sigmoid"]
    linears = [m for m in net if isinstance(m, nn.Linear)]
    for lin, act in zip(linears, acts):
        W = lin.weight.detach().numpy()
        b = lin.bias.detach().numpy()
        layers.append((W, b, act))
    return layers


def train_numpy(Xn, y, epochs, lr=0.1, seed=0):
    """Logistic regression by full-batch gradient descent (torch-free fallback)."""
    rng = np.random.default_rng(seed)
    w = rng.standard_normal(DIM) * 0.01
    b = 0.0
    n = len(y)
    for _ in range(epochs * 5):
        z = Xn @ w + b
        p = 1.0 / (1.0 + np.exp(-z))
        g = p - y
        w -= lr * (Xn.T @ g) / n
        b -= lr * g.mean()
    z = Xn @ w + b
    p = 1.0 / (1.0 + np.exp(-z))
    bce = -np.mean(y * np.log(p + 1e-9) + (1 - y) * np.log(1 - p + 1e-9))
    print(f"[numpy] final BCE loss = {bce:.4f}")
    return [(w.reshape(1, DIM), np.array([b]), "sigmoid")]


def report_quality(model, X, y, tag=""):
    """Threshold-free separation: mean predicted slip on slip vs non-slip rows."""
    preds = np.array([model.predict(r) for r in X])
    pos = preds[y > 0.5].mean() if (y > 0.5).any() else float("nan")
    neg = preds[y <= 0.5].mean() if (y <= 0.5).any() else float("nan")
    acc = ((preds > 0.5).astype(float) == (y > 0.5).astype(float)).mean()
    print(f"{tag}mean slip score:  slipping={pos:.3f}  planted={neg:.3f}")
    print(f"{tag}accuracy@0.5   :  {acc:.3f}")
    return preds


def report_runlog_quality(model, X, y, err, tag=""):
    """The two questions a live A/B actually turns on.

    1. Does the score DISCRIMINATE, or does it just de-weight leg odometry
       everywhere? A model whose score barely varies inflates R uniformly, which
       is a retune of leg_odom_vel_noise wearing a neural network as a hat.
    2. Does it rank the genuinely bad leg-odom samples above the good ones?
       Reported as the score gap between the worst and best error deciles, and
       as the correlation with the true error.
    """
    preds = report_quality(model, X, y, tag)
    lo, hi = np.percentile(err, 10), np.percentile(err, 90)
    best, worst = preds[err <= lo], preds[err >= hi]
    spread = preds.max() - preds.min()
    corr = np.corrcoef(preds, err)[0, 1]
    print(f"{tag}score range    :  {preds.min():.3f} .. {preds.max():.3f} "
          f"(median {np.median(preds):.3f}, spread {spread:.3f})")
    print(f"{tag}decile gap     :  best-10% err -> {best.mean():.3f}, "
          f"worst-10% err -> {worst.mean():.3f} "
          f"(gap {worst.mean() - best.mean():+.3f})")
    print(f"{tag}corr(score,err):  {corr:+.3f}")
    if spread < 0.1:
        print(f"{tag}VERDICT: the score is nearly CONSTANT — this model does not "
              "detect slip, it just scales R_leg. Do not read a live A/B as "
              "evidence about slip detection.")
    return preds


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=None, help="pre-labelled feature CSV")
    ap.add_argument("--runlog", nargs="+", default=None,
                    help="live slip-feature logs from eskf_node (slip_log:=...); "
                         "labelled here from the ground-truth twist")
    ap.add_argument("--label-scale", type=float, default=0.3,
                    help="leg-odom velocity error [m/s] mapped to slip score 1.0")
    ap.add_argument("--val-frac", type=float, default=0.2,
                    help="held-out fraction (--runlog only)")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "config",
                                                  "slip_model.txt"))
    ap.add_argument("--hidden", type=int, default=16)
    ap.add_argument("--epochs", type=int, default=300)
    args = ap.parse_args()

    err = None
    if args.runlog:
        print(f"reading {len(args.runlog)} run log(s):")
        X, y, err = load_runlogs(args.runlog, args.label_scale)
    elif args.data:
        X, y = load_dataset(args.data)
        print(f"loaded {len(y)} rows from {args.data}")
    else:
        X, y = synthesize_dataset()
        print(f"synthesized {len(y)} rows (no --data given) — SYNTHETIC, the "
              "model has never seen the robot")

    mean, std = standardize(X)
    Xn = (X - mean) / std

    # Held-out split, only where it is meaningful (real logged data). The rows are
    # a time series, so shuffle before splitting is optimistic about
    # generalisation across runs — it still catches a model that cannot fit at all.
    val = None
    if err is not None and 0.0 < args.val_frac < 0.5 and len(y) > 100:
        rng = np.random.default_rng(0)
        perm = rng.permutation(len(y))
        n_val = int(len(y) * args.val_frac)
        vi, ti = perm[:n_val], perm[n_val:]
        val = (Xn[vi], y[vi])
        Xn_fit, y_fit = Xn[ti], y[ti]
    else:
        Xn_fit, y_fit = Xn, y

    try:
        import torch  # noqa: F401
        layers = train_torch(Xn_fit, y_fit, args.hidden, args.epochs, val)
    except ImportError:
        print("PyTorch not found — using NumPy logistic-regression fallback.")
        layers = train_numpy(Xn_fit, y_fit, args.epochs)

    model = ref.SlipModel(DIM, mean, std, layers)
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    notes = [f"trained {datetime.datetime.now().isoformat(timespec='seconds')}",
             f"hidden={args.hidden} epochs={args.epochs} rows={len(y)}"]
    if args.runlog:
        notes.append("source: run logs " + " ".join(args.runlog))
        notes.append(f"label = clip(|v_leg - v_truth| / {args.label_scale}, 0, 1)")
    elif args.data:
        notes.append(f"source: {args.data}")
    else:
        notes.append("source: SYNTHETIC (synthesize_dataset) — never saw the robot")
    model.save(out, notes)
    print(f"wrote {out}")
    if err is not None:
        report_runlog_quality(model, X, y, err)
        if val is not None:
            report_runlog_quality(model, X[vi], y[vi], err[vi], tag="[val] ")
    else:
        report_quality(model, X, y)


if __name__ == "__main__":
    main()
