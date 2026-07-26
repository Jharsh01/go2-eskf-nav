# go2_eskf

8-state **error-state EKF** for Unitree Go2 localization — fuses **IMU + leg
odometry + GPS**, with a slip-adaptive measurement-covariance model.

Like `ekf_estimator`, the math is a **ROS-free C++/Eigen core** (`eskf_core`)
so it unit-tests and cross-validates in isolation, wrapped by a thin ROS node.

## Status
- **Phase 1 ✅** — ESKF core, 16 GTest unit tests, NumPy reference +
  cross-validation harness (C++ ≡ NumPy to ~1e-14).
- **Phase 2 ✅** — live ROS node `eskf_node` (IMU predict, leg-odom + GPS
  corrections, ENU conversion, ground-truth logging). Robust `gravity_lp`
  attitude handling for the sim IMU; validated bounded on live data.
- **Phase 3 ✅** — slip-adaptive leg covariance: PyTorch trainer (+ NumPy
  fallback) → exported weights → dependency-free Eigen MLP (`slip_model.hpp`)
  inflating `R_leg`; 12 unit tests + C++≡NumPy slip cross-validation (~3e-16).
- **Phase 4 ✅** — benchmark suite: ATE/RPE/drift metrics with SE(2) alignment,
  fixed-vs-adaptive-vs-GPS-denied scenarios, auto-generated report + plots.

The remaining manual step is collecting real logs from a healthy walking-sim run
to populate the benchmark report with on-robot numbers. See
[`docs/DESIGN.md`](docs/DESIGN.md) §7–8.

## Slip model & benchmark
```bash
# Train the slip detector (PyTorch if installed, else NumPy fallback) -> weights
python3 src/go2_eskf/scripts/train_slip_model.py --out src/go2_eskf/config/slip_model.txt

# Cross-validate the C++ Eigen MLP against the NumPy twin (~3e-16)
python3 src/go2_eskf/scripts/cross_validate_slip.py

# Run the ESKF with the slip model enabled
ros2 run go2_eskf go2_eskf_node --ros-args -p use_slip_model:=true \
  -p slip_model_path:=$PWD/install/go2_eskf/share/go2_eskf/config/slip_model.txt

# Offline benchmark (analyse logged scenarios, or --demo for synthetic)
python3 src/go2_eskf/scripts/run_benchmark.py --demo --out /tmp/benchmark_demo
```

## Run live (with unitree_go2_sim)
```bash
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true
# GPS off until the sim world gets a spherical_coordinates datum:
ros2 run go2_eskf go2_eskf_node --ros-args -p use_sim_time:=true -p use_gps:=false
```

## Build & test
```bash
colcon build --packages-select go2_eskf --symlink-install
source install/setup.bash

# unit tests
./build/go2_eskf/test_eskf_core

# cross-validate C++ against the NumPy twin
python3 src/go2_eskf/scripts/cross_validate.py
```

## Layout
```
include/go2_eskf/   eskf_core.hpp, types.hpp   (pure Eigen, ROS-free)
                    slip_model.hpp             (header-only Eigen MLP, Phase 3)
src/                eskf_core.cpp, eskf_node.cpp
tools/              replay_eskf.cpp            (deterministic ESKF replay)
                    slip_infer.cpp             (slip inference driver)
scripts/            eskf_reference.py          (ESKF NumPy twin)
                    cross_validate.py          (ESKF C++ vs NumPy)
                    slip_reference.py          (slip NumPy twin)
                    cross_validate_slip.py     (slip C++ vs NumPy)
                    train_slip_model.py        (PyTorch / NumPy trainer)
                    metrics.py                 (ATE/RPE/drift, Phase 4)
                    run_benchmark.py           (scenarios -> report + plots)
test/               test_eskf_core.cpp         (16 GTest cases)
                    test_slip_model.cpp        (12 GTest cases)
config/             eskf_params.yaml, slip_model.txt
launch/             eskf.launch.py, benchmark.launch.py
docs/               DESIGN.md, REFERENCE.md
```

## Docs
- [`docs/DESIGN.md`](docs/DESIGN.md) — state/Jacobian/sensor-model derivation,
  design rationale, roadmap, and how the package fits the CHAMP control stack.
- [`docs/REFERENCE.md`](docs/REFERENCE.md) — ROS node interface (topics, frames,
  TF), full parameter tables, the C++ `EskfCore` API, and tooling/tests.
