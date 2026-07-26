# Ground-Truth Wiring & Benchmark Enablement — Session Report

**Date:** 2026-07-12
**Package:** `go2_eskf` (Unitree Go2 error-state EKF) + one edit to vendored `unitree_go2_description`
**Goal:** Run the Unitree pipeline, exercise slip-model training + Phase-4 metrics, and
**wire up a ground-truth source** so the benchmark can score ATE/RPE/drift against the sim.

---

## 1. Summary

The Phase-4 benchmark was blocked: the Go2 sim publishes **no world-frame
`nav_msgs/Odometry` ground truth**, so the estimator's CSV log had nothing to score
against. This session:

1. Walked through **running the full Unitree pipeline** (sim + ESKF).
2. Ran the **slip-model training** and **benchmark metrics** offline to confirm they work.
3. Diagnosed why ground truth was missing, tried a shim approach, found a hard blocker in
   `ros_gz_bridge`, and landed the correct fix: a Gazebo **`OdometryPublisher`** bridged to
   `/ground_truth/odom`.
4. **Verified the whole chain live** against a running sim, ending with a real ATE/RPE/drift
   table computed from an on-robot log.

Result: `ros2 launch go2_eskf ground_truth.launch.py` now provides ground truth, and the
real benchmark runs end-to-end.

---

## 2. Running the Unitree pipeline (reference)

Two parts run together, each terminal sourced with
`source /opt/ros/jazzy/setup.bash && source install/setup.bash`:

```bash
# Terminal 1 — Go2 Gazebo sim (CHAMP stack)
ros2 launch unitree_go2_sim unitree_go2_launch.py         # rviz:=true optional

# Terminal 2 — error-state EKF against the sim
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true
```

Confirmed live topic names (correcting earlier assumptions): IMU `imu/data`, leg odometry
`odom/raw`, GPS `gps/fix` (keep off — has a datum but left disabled), drive `/cmd_vel`
(sim remaps `/cmd_vel/smooth`→`/cmd_vel`), `joint_states`, ESKF output `eskf/odom`, slip
score `eskf/slip`. Drive the robot with:
`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10`.

---

## 3. Slip-model training & metrics (verified offline)

All three ran clean (PyTorch 2.12 + matplotlib 3.6 present):

| Step | Command | Result |
|------|---------|--------|
| Train slip model | `python3 src/go2_eskf/scripts/train_slip_model.py` | wrote `config/slip_model.txt`; BCE≈0, acc 1.000 (synthetic data is trivially separable — placeholder until real `--data` logs) |
| Verify export | `python3 src/go2_eskf/scripts/cross_validate_slip.py` | **PASS** — C++ runtime ≡ NumPy to `3.3e-16` |
| Demo benchmark | `python3 src/go2_eskf/scripts/run_benchmark.py --demo --out /tmp/benchmark_demo` | table + `trajectories.png` + `error_over_time.png` |

Demo metrics (synthetic, confirms pipeline + shows intended story):

| scenario | ATE [m] | RPE trans [m] | final drift [m] |
|----------|--------:|--------------:|----------------:|
| fixed | 1.0143 | 0.0877 | 0.9667 |
| adaptive | 0.1312 | 0.0603 | 0.1459 |
| gps_denied | 1.3555 | 0.0824 | 2.0556 |

**Key clarification:** training is an *offline* step producing `slip_model.txt`; the live
node only *loads* those weights and runs inference. Nothing "trains" during simulation.

---

## 4. Ground-truth wiring (main deliverable)

### 4.1 The problem
- `/odom` is the `robot_localization` **estimate**, not truth.
- The sim's Gazebo odometry plugin was deliberately removed; only a link-relative
  `PosePublisher` remains (robot-internal TF, not world truth).

### 4.2 Dead ends (all confirmed against the running sim)
- **`ros_gz_bridge` strips entity names.** Bridging `gz.msgs.Pose_V → tf2_msgs/TFMessage`
  from `/world/default/dynamic_pose/info` **and** `/world/default/pose/info` yields
  **empty `frame_id`/`child_frame_id`** for every transform, so the robot cannot be matched
  by name. (Raw gz messages *do* carry names, e.g. `base_link`, `go2`; the bridge drops them.)
- `dynamic_pose/info` is also **silent at rest** (only moving entities), and its poses are
  identified by numeric id, not name.
- **gz Transport Python bindings are not installed** (`gz.transport*`, `gz.msgs*` all absent),
  so reading gz directly from Python was not an option.
- `/model/go2/pose` carries only **model-internal** link poses (`go2::rh_lower_leg_link`, …),
  not world truth.

A first attempt — a Python shim (`ground_truth_odom.py`) subscribing to the bridged
TFMessage and picking the base link — was built, unit-tested offline, then **abandoned**
because the name-stripping makes name matching impossible. It was removed.

### 4.3 The fix
Add a dedicated Gazebo **`OdometryPublisher`** to the model that emits the true base
pose+twist on a **new** topic (so it never conflicts with the `/odom` chain), and bridge it
straight to `nav_msgs/Odometry`:

```
gz OdometryPublisher  ->  /model/go2/ground_truth/odometry  (gz.msgs.Odometry)
      --ros_gz_bridge-->  /ground_truth/odom                (nav_msgs/Odometry)
```

Ground truth is relative to the spawn pose, which is exactly what the estimator is compared
against (both start at the origin). Metrics SE(2)-align anyway.

---

## 5. Files changed

| File | Change |
|------|--------|
| `src/unitree_go2_ros2/unitree_go2_description/urdf/unitree_go2_gazebo.xacro` | **+`OdometryPublisher`** plugin → `/model/go2/ground_truth/odometry`, `odom_frame=ground_truth_odom`, `robot_base_frame=base_link`, 50 Hz, 3-D. The **one vendored edit** — additive, non-conflicting, harmless if the bridge is never run. |
| `src/go2_eskf/launch/ground_truth.launch.py` | **New** — runs a `parameter_bridge` for the gz Odometry topic → `/ground_truth/odom` (args `gz_odom_topic`, `output_topic`, `use_sim_time`). |
| `src/go2_eskf/package.xml` | `+<exec_depend>ros_gz_bridge</exec_depend>`. |
| `src/go2_eskf/docs/DESIGN.md` (§8) | Documented the true GT source, the bridge name-stripping gotcha, and the plugin approach. |
| `CLAUDE.md` (go2_eskf section) | Documented `ground_truth.launch.py`, the gotcha, and flagged the deliberate vendored edit. |
| `src/go2_eskf/scripts/ground_truth_odom.py` | Created then **removed** (abandoned shim). `CMakeLists.txt` net-unchanged. |

**Rebuilds:** `unitree_go2_description` (its installed xacro was a stale **copy**, not a
symlink — the reason the first relaunch didn't pick up the plugin) and `go2_eskf`. Both use
the workspace's `--merge-install` layout.

---

## 6. Live verification (fresh sim, robot walking)

| Check | Result |
|-------|--------|
| gz plugin publishes | `/model/go2/ground_truth/odometry` present ~3 s after spawn |
| Bridged topic type | `/ground_truth/odom` = `nav_msgs/msg/Odometry`, **~48 Hz** |
| At rest | position ≈ (0.02, 0.003) — origin |
| After ~8 s @ cmd 0.4 m/s | position → (1.78, 0.17) — true world-frame motion |
| Twist | `vx ≈ 0.38 m/s` ≈ commanded 0.4 |
| ESKF CSV logging | `gt_x, gt_y, gt_yaw` populated; only **4 of 8760** rows `nan` (startup, pre-first-GT) |

**Real end-to-end benchmark** (ESKF node vs `/ground_truth/odom`, 8755 samples):

| scenario | ATE [m] | RPE trans [m] | RPE rot [rad] | final drift [m] | drift % |
|----------|--------:|--------------:|--------------:|----------------:|--------:|
| live_sim | 1.1714 | 0.0258 | 0.0191 | 1.3124 | 6.17 |

The sim was torn down cleanly afterward.

---

## 7. How to use it

Three terminals (each sourced):

```bash
# 1. Sim
ros2 launch unitree_go2_sim unitree_go2_launch.py

# 2. Ground truth (new)
ros2 launch go2_eskf ground_truth.launch.py

# 3. One benchmark scenario, pointed at the GT topic
ros2 launch go2_eskf benchmark.launch.py scenario:=adaptive \
     log_path:=/tmp/adaptive.csv ground_truth_topic:=/ground_truth/odom
```

Drive the robot so there is motion to estimate, repeat T3 for `scenario:=fixed` and
`scenario:=gps_denied`, then:

```bash
python3 src/go2_eskf/scripts/run_benchmark.py \
   --log fixed=/tmp/fixed.csv --log adaptive=/tmp/adaptive.csv \
   --log gps_denied=/tmp/gps_denied.csv --out docs/benchmark
```

Sanity check: `ros2 topic echo /ground_truth/odom --once`.

---

## 8. Known gaps / suggested follow-ups

- **`metrics.load_log` is brittle to a truncated final CSV line.** It uses
  `np.genfromtxt`, which raises if the ESKF node is `Ctrl-C`'d mid-write and leaves a
  partial last row. A one-line guard to skip malformed rows would harden it. (Not fixed —
  out of scope for this task.)
- **Real slip model needs real data.** `train_slip_model.py` currently trains on a
  physics-inspired *synthetic* dataset (hence the perfect scores). Collect labelled
  locomotion features from sim/hardware and pass via `--data` for a meaningful model.
- **Estimator vs GT frame offset.** The raw CSV shows the estimate in its own start frame
  (large raw offset vs GT); this is expected and removed by the benchmark's SE(2) alignment.
  The ATE ≈ 1.17 m result is a legitimate first on-robot number, not a wiring artifact.
