#!/usr/bin/env python3
"""Cross-validate the C++ SlipModel against the NumPy twin.

Generates a deterministic untrained network and a feature stream, runs both the
C++ inference driver (slip_infer) and the NumPy twin (slip_reference.py) on
them, and reports the max absolute difference in slip score. Mirrors
cross_validate.py for the ESKF core.

    cross_validate_slip.py [path-to-slip_infer] [--tol 1e-9]
"""
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import slip_reference as ref  # noqa: E402


def find_cpp_binary(explicit):
    if explicit and os.path.exists(explicit):
        return explicit
    found = shutil.which("slip_infer")
    if found:
        return found
    ws = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
    for sub in ("install/go2_eskf/lib/go2_eskf/slip_infer",
                "build/go2_eskf/slip_infer"):
        cand = os.path.join(ws, sub)
        if os.path.exists(cand):
            return cand
    return None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    tol = 1e-9
    for a in sys.argv[1:]:
        if a.startswith("--tol"):
            tol = float(a.split("=")[1]) if "=" in a else 1e-9

    cpp_bin = find_cpp_binary(args[0] if args else None)
    if not cpp_bin:
        print("ERROR: slip_infer binary not found. Build go2_eskf first "
              "(colcon build --packages-select go2_eskf).", file=sys.stderr)
        return 2

    tmp = tempfile.mkdtemp(prefix="slip_xval_")
    weights = os.path.join(tmp, "weights.txt")
    feats = os.path.join(tmp, "features.csv")
    py_out = os.path.join(tmp, "py.csv")
    cpp_out = os.path.join(tmp, "cpp.csv")

    ref.make_random_weights(weights)
    ref.generate_features(feats)
    ref.run_csv(weights, feats, py_out)
    subprocess.run([cpp_bin, weights, feats, cpp_out], check=True)

    a = np.loadtxt(py_out, ndmin=1)
    b = np.loadtxt(cpp_out, ndmin=1)
    if a.shape != b.shape:
        print(f"ERROR: shape mismatch py={a.shape} cpp={b.shape}", file=sys.stderr)
        return 1

    max_abs = float(np.abs(a - b).max())
    print(f"rows compared : {a.shape[0]}")
    print(f"binary        : {cpp_bin}")
    print(f"OVERALL max abs diff : {max_abs:.3e}   (tol {tol:.0e})")
    if max_abs <= tol:
        print("PASS — C++ and NumPy slip scores agree to machine precision.")
        return 0
    print("FAIL — divergence exceeds tolerance.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
