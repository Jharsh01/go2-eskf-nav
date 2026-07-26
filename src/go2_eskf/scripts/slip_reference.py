#!/usr/bin/env python3
"""NumPy twin of the C++ SlipModel (include/go2_eskf/slip_model.hpp).

Mirrors the MLP forward pass and the plain-text weights format exactly, so the
two can be cross-validated to machine precision on the same feature stream —
the same C++<->NumPy discipline used for the ESKF core (eskf_reference.py).

Also provides:
  * the feature-vector layout (SLIP_FEATURES) shared with slip_model.hpp,
  * read/write of the weights file format,
  * make_random_weights(): a deterministic, untrained network used purely to
    exercise the forward-pass cross-validation (training lives in
    train_slip_model.py, which writes the identical format).

Weights file format (lines beginning with '#' are comments):

    input_dim <D>
    mean  <m_0> ... <m_{D-1}>
    std   <s_0> ... <s_{D-1}>
    layers <L>
    layer <in> <out> <relu|sigmoid|none>
    <out rows of W, each <in> values>
    <one row of b, <out> values>
    ...
"""
import sys

import numpy as np

# Feature layout — MUST match slip_feat:: in slip_model.hpp.
SLIP_FEATURES = [
    "cmd_minus_leg_vx",   # 0  commanded - measured body vx   [m/s]
    "cmd_minus_leg_vy",   # 1  commanded - measured body vy   [m/s]
    "cmd_minus_gyro_wz",  # 2  commanded - measured yaw rate  [rad/s]
    "leg_speed",          # 3  |leg-odom body velocity|       [m/s]
    "joint_vel_mean",     # 4  mean |joint velocity|          [rad/s]
    "joint_vel_max",      # 5  max  |joint velocity|          [rad/s]
    "accel_horiz",        # 6  horizontal motion-accel magn.  [m/s^2]
    "contact_frac",       # 7  fraction of feet in contact    [0..1]
]
DIM = len(SLIP_FEATURES)


def _relu(x):
    return np.maximum(x, 0.0)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


_ACT = {"relu": _relu, "sigmoid": _sigmoid, "none": lambda x: x}


class SlipModel:
    """NumPy MLP matching the C++ forward pass."""

    def __init__(self, input_dim, mean, std, layers):
        self.input_dim = input_dim
        self.mean = np.asarray(mean, float)
        self.std = np.asarray(std, float).copy()
        self.std[np.abs(self.std) < 1e-12] = 1.0  # guard, as in C++
        self.layers = layers  # list of (W [out,in], b [out], act_name)

    def predict(self, features):
        x = (np.asarray(features, float) - self.mean) / self.std
        for W, b, act in self.layers:
            x = _ACT[act](W @ x + b)
        return float(x[0])

    # --- file IO -----------------------------------------------------------
    def save(self, path):
        with open(path, "w") as f:
            f.write("# go2_eskf slip MLP weights\n")
            f.write(f"input_dim {self.input_dim}\n")
            f.write("mean " + " ".join(f"{v:.17g}" for v in self.mean) + "\n")
            f.write("std " + " ".join(f"{v:.17g}" for v in self.std) + "\n")
            f.write(f"layers {len(self.layers)}\n")
            for W, b, act in self.layers:
                out_dim, in_dim = W.shape
                f.write(f"layer {in_dim} {out_dim} {act}\n")
                for r in range(out_dim):
                    f.write(" ".join(f"{v:.17g}" for v in W[r]) + "\n")
                f.write(" ".join(f"{v:.17g}" for v in b) + "\n")

    @staticmethod
    def load(path):
        with open(path) as f:
            toks = []
            for line in f:
                line = line.split("#", 1)[0]
                toks.extend(line.split())
        it = iter(toks)

        def nxt():
            return next(it)

        assert nxt() == "input_dim"
        input_dim = int(nxt())
        assert nxt() == "mean"
        mean = [float(nxt()) for _ in range(input_dim)]
        assert nxt() == "std"
        std = [float(nxt()) for _ in range(input_dim)]
        assert nxt() == "layers"
        n_layers = int(nxt())
        layers = []
        for _ in range(n_layers):
            assert nxt() == "layer"
            in_dim, out_dim, act = int(nxt()), int(nxt()), nxt()
            W = np.array([[float(nxt()) for _ in range(in_dim)]
                          for _ in range(out_dim)])
            b = np.array([float(nxt()) for _ in range(out_dim)])
            layers.append((W, b, act))
        return SlipModel(input_dim, mean, std, layers)


def make_random_weights(path, hidden=16, seed=0):
    """Deterministic untrained net for forward-pass cross-validation only."""
    rng = np.random.default_rng(seed)
    mean = rng.standard_normal(DIM) * 0.1
    std = 0.5 + rng.random(DIM)  # strictly positive
    W1 = rng.standard_normal((hidden, DIM)) * 0.5
    b1 = rng.standard_normal(hidden) * 0.1
    W2 = rng.standard_normal((1, hidden)) * 0.5
    b2 = rng.standard_normal(1) * 0.1
    SlipModel(DIM, mean, std,
              [(W1, b1, "relu"), (W2, b2, "sigmoid")]).save(path)


def generate_features(path, n=400, seed=1):
    """Deterministic, plausible raw feature stream for cross-validation."""
    rng = np.random.default_rng(seed)
    header = ",".join(SLIP_FEATURES)
    lines = [header]
    for _ in range(n):
        row = rng.standard_normal(DIM)
        row[SLIP_FEATURES.index("leg_speed")] = abs(row[3])     # >= 0
        row[SLIP_FEATURES.index("joint_vel_max")] = abs(row[5])
        row[SLIP_FEATURES.index("contact_frac")] = rng.random()  # 0..1
        lines.append(",".join(f"{v:.17g}" for v in row))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def run_csv(weights, features_csv, output_csv):
    """Predict a slip score for every feature row (twin of slip_infer.cpp)."""
    model = SlipModel.load(weights)
    data = np.loadtxt(features_csv, delimiter=",", skiprows=1, ndmin=2)
    out = np.array([model.predict(r) for r in data])
    np.savetxt(output_csv, out, fmt="%.17g")


if __name__ == "__main__":
    if len(sys.argv) == 5 and sys.argv[1] == "run":
        run_csv(sys.argv[2], sys.argv[3], sys.argv[4])
    elif len(sys.argv) == 3 and sys.argv[1] == "gen-weights":
        make_random_weights(sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == "gen-features":
        generate_features(sys.argv[2])
    else:
        print("usage:\n"
              "  slip_reference.py run <weights> <features.csv> <out.csv>\n"
              "  slip_reference.py gen-weights <weights>\n"
              "  slip_reference.py gen-features <features.csv>", file=sys.stderr)
        sys.exit(1)
