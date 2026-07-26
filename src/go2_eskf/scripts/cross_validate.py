#!/usr/bin/env python3
"""Cross-validate the C++ ESKF against the NumPy twin.

Both filters consume one deterministic sensor stream and must agree to within
machine precision accumulated over the run (~1e-11). Prints the max absolute
state difference and exits non-zero if it exceeds the threshold.

    cross_validate.py [path-to-replay_eskf] [--tol 1e-9]
"""
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import eskf_reference as ref  # noqa: E402


def find_cpp_binary(explicit):
    if explicit and os.path.exists(explicit):
        return explicit
    # Try PATH, then the colcon install/build trees relative to this file.
    found = shutil.which("replay_eskf")
    if found:
        return found
    candidates = []
    ws = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
    for sub in ("install/go2_eskf/lib/go2_eskf/replay_eskf",
                "build/go2_eskf/replay_eskf"):
        candidates.append(os.path.join(ws, sub))
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    tol = 1e-9
    for a in sys.argv[1:]:
        if a.startswith("--tol"):
            tol = float(a.split("=")[1]) if "=" in a else 1e-9

    cpp_bin = find_cpp_binary(args[0] if args else None)
    if not cpp_bin:
        print("ERROR: replay_eskf binary not found. Build go2_eskf first "
              "(colcon build --packages-select go2_eskf).", file=sys.stderr)
        return 2

    tmp = tempfile.mkdtemp(prefix="eskf_xval_")
    inp = os.path.join(tmp, "input.csv")
    py_out = os.path.join(tmp, "py.csv")
    cpp_out = os.path.join(tmp, "cpp.csv")

    ref.generate_input(inp)
    ref.run_csv(inp, py_out)
    subprocess.run([cpp_bin, inp, cpp_out], check=True)

    a = np.loadtxt(py_out, delimiter=",", ndmin=2)
    b = np.loadtxt(cpp_out, delimiter=",", ndmin=2)
    if a.shape != b.shape:
        print(f"ERROR: shape mismatch py={a.shape} cpp={b.shape}", file=sys.stderr)
        return 1

    diff = np.abs(a - b)
    max_abs = diff.max()
    names = ["px", "py", "pz", "vx", "vy", "vz", "psi", "b_g"]
    per_state = diff.max(axis=0)

    print(f"steps compared : {a.shape[0]}")
    print(f"binary         : {cpp_bin}")
    print("max abs diff per state:")
    for n, d in zip(names, per_state):
        print(f"    {n:>4} : {d:.3e}")
    print(f"OVERALL max abs diff : {max_abs:.3e}   (tol {tol:.0e})")

    if max_abs <= tol:
        print("PASS — C++ and NumPy agree to machine precision.")
        return 0
    print("FAIL — divergence exceeds tolerance.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
