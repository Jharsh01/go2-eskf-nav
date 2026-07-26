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
  * --data <csv>  : real recorded features. Columns = SLIP_FEATURES plus a final
                    'slip' label column in [0,1]. Collect this from sim/hardware
                    runs (e.g. label = foot-velocity-disagreement threshold, or
                    commanded-vs-measured mismatch under known slippery patches).
  * default       : a physics-inspired synthetic dataset (synthesize_dataset),
                    so the pipeline is runnable before real logs exist.

Usage:
  train_slip_model.py [--data data.csv] [--out config/slip_model.txt]
                      [--hidden 16] [--epochs 300]
"""
import argparse
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


def standardize(X):
    mean = X.mean(axis=0)
    std = X.std(axis=0)
    std[std < 1e-6] = 1.0
    return mean, std


def train_torch(Xn, y, hidden, epochs):
    import torch
    import torch.nn as nn

    Xt = torch.tensor(Xn, dtype=torch.float32)
    yt = torch.tensor(y, dtype=torch.float32).unsqueeze(1)
    net = nn.Sequential(
        nn.Linear(DIM, hidden), nn.ReLU(),
        nn.Linear(hidden, hidden), nn.ReLU(),
        nn.Linear(hidden, 1),  # logits; sigmoid applied at export/inference
    )
    opt = torch.optim.Adam(net.parameters(), lr=1e-2)
    loss_fn = nn.BCEWithLogitsLoss()
    for ep in range(epochs):
        opt.zero_grad()
        loss = loss_fn(net(Xt), yt)
        loss.backward()
        opt.step()
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


def report_quality(model, X, y):
    """Threshold-free separation: mean predicted slip on slip vs non-slip rows."""
    preds = np.array([model.predict(r) for r in X])
    pos = preds[y > 0.5].mean() if (y > 0.5).any() else float("nan")
    neg = preds[y <= 0.5].mean() if (y <= 0.5).any() else float("nan")
    acc = ((preds > 0.5).astype(float) == y).mean()
    print(f"mean slip score:  slipping={pos:.3f}  planted={neg:.3f}")
    print(f"accuracy@0.5   :  {acc:.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=None, help="labelled feature CSV")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "config",
                                                  "slip_model.txt"))
    ap.add_argument("--hidden", type=int, default=16)
    ap.add_argument("--epochs", type=int, default=300)
    args = ap.parse_args()

    if args.data:
        X, y = load_dataset(args.data)
        print(f"loaded {len(y)} rows from {args.data}")
    else:
        X, y = synthesize_dataset()
        print(f"synthesized {len(y)} rows (no --data given)")

    mean, std = standardize(X)
    Xn = (X - mean) / std

    try:
        import torch  # noqa: F401
        layers = train_torch(Xn, y, args.hidden, args.epochs)
    except ImportError:
        print("PyTorch not found — using NumPy logistic-regression fallback.")
        layers = train_numpy(Xn, y, args.epochs)

    model = ref.SlipModel(DIM, mean, std, layers)
    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    model.save(out)
    print(f"wrote {out}")
    report_quality(model, X, y)


if __name__ == "__main__":
    main()
