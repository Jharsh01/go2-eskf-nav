# go2_eskf — Reference

Interface and API reference for the `go2_eskf` package: the ROS 2 node's topics,
frames, and parameters; the ROS-free C++ core API; and the developer tooling.

- For a **quickstart**, see [`../README.md`](../README.md).
- For the **math derivation, design rationale, and roadmap**, see
  [`DESIGN.md`](DESIGN.md).

This document describes runtime/API surfaces; it does not re-derive the filter.

---

## 1. Package at a glance

8-state error-state EKF (ESKF) localizing a Unitree Go2, fusing **IMU + leg
odometry + GPS**. The math lives in a ROS-free C++/Eigen core (`EskfCore`),
wrapped by a thin ROS node (`EskfNode`), and mirrored line-for-line by a NumPy
twin used for cross-validation.

```
include/go2_eskf/eskf_core.hpp   EskfCore — predict/correct/inject ESKF math (ROS-free)
include/go2_eskf/types.hpp       Vec8/Mat8, state indices, kGravity
src/eskf_core.cpp                EskfCore implementation
src/eskf_node.cpp                EskfNode — ROS 2 wrapper (subs/pubs/TF/logging)
config/eskf_params.yaml          default parameters
launch/eskf.launch.py            launch with params_file / use_sim_time / log_path args
scripts/eskf_reference.py        NumPy twin of EskfCore
scripts/cross_validate.py        feeds one stream to C++ + NumPy, compares
tools/replay_eskf.cpp            
replay driver (replay_eskf binary)
test/test_eskf_core.cpp          16 GTest cases
```

State vector `x = [px py pz  vx vy vz  psi  b_g]ᵀ` — position (world/ENU, m),
velocity (world, m/s), yaw (rad), yaw-rate gyro bias (rad/s). Roll & pitch are
read from the IMU, not estimated (hence 8 states, not 15).

---

## 2. ROS node interface (`go2_eskf_node`)

Node name: `eskf_node`. Run it via the launch file or directly:

```bash
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true
ros2 run go2_eskf go2_eskf_node --ros-args -p use_sim_time:=true -p use_gps:=false
```

### 2.1 Subscriptions

| Topic (default) | Param | Type | QoS | Role |
|-----------------|-------|------|-----|------|
| `imu/data` | `imu_topic` | `sensor_msgs/Imu` | SensorData (best-effort) | Drives **prediction** (accel + yaw-rate gyro; roll/pitch for gravity) |
| `odom/raw` | `leg_odom_topic` | `nav_msgs/Odometry` | depth 10 (reliable) | CHAMP leg odometry — body-frame velocity in `twist.twist.linear.{x,y}`; **corrects velocity** |
| `gps/fix` | `gps_topic` | `sensor_msgs/NavSatFix` | SensorData (best-effort) | **Corrects world position** (only if `use_gps:=true`) |
| `ground_truth_topic` (empty) | `ground_truth_topic` | `nav_msgs/Odometry` | depth 10 | Optional; logged for benchmarking only — not fused |
| `cmd_vel` | `cmd_vel_topic` | `geometry_msgs/Twist` | depth 10 | Slip feature (commanded body twist) — only if `use_slip_model:=true` |
| `joint_states` | `joint_states_topic` | `sensor_msgs/JointState` | SensorData (best-effort) | Slip feature (joint-velocity stats) — only if `use_slip_model:=true` |

> The IMU and GPS subs **must** be best-effort (`SensorDataQoS`) because the
> Gazebo→ROS bridge publishes best-effort; a reliable sub silently receives
> nothing.

### 2.2 Publications

| Topic (default) | Param | Type | Notes |
|-----------------|-------|------|-------|
| `eskf/odom` | `output_odom_topic` | `nav_msgs/Odometry` | Pose in `world_frame`, twist in body frame; `pose.covariance` carries x/y/yaw variances, `twist.covariance` carries vx/vy. Published every IMU tick. |
| `eskf/slip` | — | `std_msgs/Float64` | Latest slip score in [0,1] — only if `use_slip_model:=true`. Published each leg-odom correction. |
| TF `world_frame → base_frame` | — | `tf2` | Broadcast only if `publish_tf:=true` (off by default to avoid clashing with the stock `robot_localization` TF tree). |

### 2.3 Frames

- `world_frame` (default `map`) — output pose/TF parent frame.
- `base_frame` (default `base_link`) — child frame.

### 2.4 Behavior notes

- **Initialization:** the first IMU message seeds the gravity low-pass and (for
  `attitude_source: orientation`) the initial yaw, then returns — two stamps are
  needed before the first `dt`. Leg-odom/GPS corrections are ignored until
  initialized.
- **dt guarding:** non-positive `dt` (duplicate/out-of-order stamps) is dropped;
  `dt` is clamped to `max_imu_dt` to survive pauses/dropouts.
- **GPS datum:** the first valid fix sets the ENU origin (`lat0/lon0`) and is not
  used as a correction; subsequent fixes are converted to local ENU (equirect-
  angular approximation about the datum) and fused. `STATUS_NO_FIX` messages are
  ignored.
- **Logging:** if `log_path` is non-empty, a CSV
  `t,est_x,est_y,est_z,est_yaw,est_vx,est_vy,gt_x,gt_y,gt_yaw` is written each
  IMU tick (ground-truth columns are `nan` until a GT message arrives).

---

## 3. Parameters

Defaults below match `config/eskf_params.yaml`. See [`DESIGN.md`](DESIGN.md) §2/§6
for why the noise defaults are set the way they are (large `accel_noise` so leg
odometry dominates velocity; `gravity_lp` attitude to survive the sim IMU).

### Topics & frames
| Parameter | Default | Description |
|-----------|---------|-------------|
| `imu_topic` | `imu/data` | IMU input |
| `leg_odom_topic` | `odom/raw` | CHAMP leg odometry (body-frame twist) |
| `gps_topic` | `gps/fix` | NavSatFix input |
| `ground_truth_topic` | `""` | Optional GT to log (empty = disabled) |
| `output_odom_topic` | `eskf/odom` | Fused odometry output |
| `world_frame` | `map` | Output parent frame |
| `base_frame` | `base_link` | Output child frame |
| `publish_tf` | `false` | Broadcast `world→base` TF |

### Attitude / gravity
| Parameter | Default | Description |
|-----------|---------|-------------|
| `attitude_source` | `gravity_lp` | `gravity_lp` \| `accel` \| `orientation` (see below) |
| `grav_lp_beta` | `0.995` | Gravity low-pass factor (closer to 1 = slower tracking) |
| `accel_clip` | `40.0` | Clip motion-accel magnitude [m/s²] to reject contact spikes |

`attitude_source` modes:
- **`gravity_lp`** (default) — low-pass the accelerometer to track gravity+mount,
  subtract it for motion accel, rotate by yaw only. Robust to the sim IMU's frame
  flip and contact spikes; discards sustained horizontal accel (leg odom is the
  primary velocity source).
- **`accel`** — per-sample roll/pitch from the gravity direction.
- **`orientation`** — trust the IMU's `orientation` quaternion (unreliable on the
  sim IMU; see DESIGN §6).

### Sensor switches
| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_gps` | `true` | Subscribe to and fuse GPS (set `false` until the sim world has a `spherical_coordinates` datum) |
| `max_imu_dt` | `0.05` | Clamp prediction `dt` [s] across pauses/dropouts |

### IMU process noise (continuous-time std-devs)
| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `accel_noise` | `1.0` | m/s² | Accelerometer white noise (large by design) |
| `gyro_noise` | `0.002` | rad/s | Yaw-rate white noise |
| `gyro_bias_noise` | `0.0001` | rad/s/√s | Gyro-bias random walk |

### Measurement noise (std-devs)
| Parameter | Default | Units | Description |
|-----------|---------|-------|-------------|
| `leg_odom_vel_noise` | `0.2` | m/s | Leg-odom velocity noise (the slip model inflates this) |
| `gps_pos_noise` | `0.5` | m | GPS horizontal noise (~u-blox) |

### Slip model (Phase 3)
| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_slip_model` | `false` | Enable slip-adaptive leg covariance (subscribes `cmd_vel` + `joint_states`, publishes `eskf/slip`) |
| `slip_model_path` | `""` | Path to the exported weights (e.g. `config/slip_model.txt`); if empty/unloadable, falls back to fixed `R_leg` |
| `slip_lambda` | `1.0` | Inflation gain: `R_leg ← R_leg · (1 + λ·s)²` |
| `cmd_vel_topic` | `cmd_vel` | Commanded body twist (slip feature) |
| `joint_states_topic` | `joint_states` | Joint velocities (slip feature) |

### Initial covariance (diagonal)
| Parameter | Default | Applies to |
|-----------|---------|-----------|
| `init_pos_cov` | `1.0` | px, py, pz |
| `init_vel_cov` | `0.5` | vx, vy, vz |
| `init_yaw_cov` | `0.2` | psi |
| `init_bias_cov` | `0.001` | b_g |

### Logging
| Parameter | Default | Description |
|-----------|---------|-------------|
| `log_path` | `""` | CSV path for estimate-vs-GT logging (empty = off) |

### Launch arguments (`eskf.launch.py`)
| Arg | Default | Description |
|-----|---------|-------------|
| `params_file` | `config/eskf_params.yaml` | Parameter YAML to load |
| `use_sim_time` | `true` | Use the Gazebo `/clock` |
| `log_path` | `""` | Overrides `log_path` param |

---

## 4. C++ core API (`go2_eskf::EskfCore`)

Header: `include/go2_eskf/eskf_core.hpp`. Pure Eigen, no ROS — link against
`go2_eskf_core`. Types come from `types.hpp` (`Vec8`, `Mat8`, `idx::*`,
`kStateDim = 8`, `kGravity`).

### Construction
```cpp
EskfCore::Config cfg;          // initial_state/covariance + IMU noise densities
cfg.accel_noise     = 1.0;     // m/s²
cfg.gyro_noise      = 2.0e-3;  // rad/s
cfg.gyro_bias_noise = 1.0e-4;  // rad/s/√s
go2_eskf::EskfCore eskf(cfg);
```

`Config` fields: `initial_state` (`Vec8`, default 0), `initial_covariance`
(`Mat8`, default `1e-3·I`), `accel_noise`, `gyro_noise`, `gyro_bias_noise`.

### Predict & correct
```cpp
// IMU-driven prediction. accel_body = specific force [m/s²], gyro_z = yaw rate
// [rad/s], roll/pitch [rad] (only used to cancel gravity), dt > 0 [s].
eskf.predictImu(accel_body, gyro_z, roll, pitch, dt);

// Leg odometry: body-frame planar velocity [vx, vy] with 2×2 covariance R.
// R is the knob the slip model will scale (Phase 3).
eskf.correctLegOdom(v_body_meas, R_leg);

// GPS: world-frame position [x, y] with 2×2 covariance R.
eskf.correctGps(pos_xy, R_gps);
```

Both corrections use the Joseph-form covariance update and inject additively into
the nominal state. `predictImu` throws `std::invalid_argument` on `dt <= 0`.

### Accessors & utilities
```cpp
const Vec8& x = eskf.state();        // current nominal state
const Mat8& P = eskf.covariance();   // current covariance
eskf.reset(x0, P0);                  // re-seed state and covariance

EskfCore::wrapAngle(a);                       // wrap to (-π, π]
EskfCore::rotBodyToWorld(roll,pitch,yaw);     // R = Rz·Ry·Rx
EskfCore::dRotDYaw(roll,pitch,yaw);           // ∂R/∂yaw (Jacobian term)
```

Read state components with the named indices, never magic numbers:
```cpp
double x_pos = eskf.state()(go2_eskf::idx::PX);
double yaw   = eskf.state()(go2_eskf::idx::PSI);
```

### Minimal usage
```cpp
#include "go2_eskf/eskf_core.hpp"
using namespace go2_eskf;

EskfCore eskf({});                                  // default config
eskf.predictImu({0,0,kGravity}, 0.0, 0,0, 0.01);    // standing still
eskf.correctLegOdom({0.0, 0.0}, Eigen::Matrix2d::Identity()*0.04);
const Vec8& x = eskf.state();
```

---

## 5. Tooling & tests

```bash
# Build just this package
colcon build --packages-select go2_eskf --symlink-install
source install/setup.bash

# Unit tests (28 GTest cases: 16 ESKF + 12 slip) — via colcon or the binaries
colcon test --packages-select go2_eskf && colcon test-result --all --verbose
./build/go2_eskf/test_eskf_core              # add --gtest_filter=... for one case
./build/go2_eskf/test_slip_model

# Cross-validate C++ vs NumPy twins (both must agree to ~1e-14 / ~3e-16).
# Strongest regression test — run after any change to the core or its twin.
python3 src/go2_eskf/scripts/cross_validate.py       # ESKF core
python3 src/go2_eskf/scripts/cross_validate_slip.py  # slip MLP

# Train / re-export the slip model (PyTorch if installed, else NumPy fallback)
python3 src/go2_eskf/scripts/train_slip_model.py --out src/go2_eskf/config/slip_model.txt

# Phase 4 metrics + benchmark (self-checks need no sim)
python3 src/go2_eskf/scripts/metrics.py --selftest
python3 src/go2_eskf/scripts/run_benchmark.py --demo --out /tmp/benchmark_demo
```

- `scripts/eskf_reference.py` / `scripts/slip_reference.py` — NumPy twins; keep
  them line-for-line in sync with `eskf_core.cpp` / `slip_model.hpp`.
- `scripts/cross_validate.py` / `scripts/cross_validate_slip.py` — feed one
  deterministic stream to both implementations and report the max abs difference.
- `tools/replay_eskf.cpp` / `tools/slip_infer.cpp` — deterministic C++ drivers
  used by the cross-validation scripts.
- `scripts/train_slip_model.py` — trains the slip detector and exports the
  weights format read by `slip_model.hpp`.
- `scripts/metrics.py` — ATE/RPE/drift from a node CSV log (`--selftest` for a
  synthetic sanity check). `scripts/run_benchmark.py` — multi-scenario report +
  plots (`--demo` synthesizes logs so it runs without the sim).

> **Testing gotcha:** never `pkill -f go2_eskf_node` from a script whose own text
> contains `go2_eskf_node` — `-f` matches the script's command line and kills the
> script. Use a bracketed path pattern, e.g. `pkill -f '[l]ib/go2_eskf/go2_eskf_node'`.

---

## 6. Status & limitations

- **Phase 1 ✅** — ESKF core, 16 unit tests, NumPy cross-validation (~1e-14).
- **Phase 2 ✅** — live ROS node; sim-integration handling (`gravity_lp` attitude,
  large `accel_noise`).
- **Phase 3 ✅** — slip-adaptive `R_leg`: Eigen MLP (`slip_model.hpp`), trainer,
  12 unit tests, C++≡NumPy slip cross-validation (~3e-16), node integration.
- **Phase 4 ✅** — ATE/RPE/drift metrics with SE(2) alignment, multi-scenario
  benchmark report + plots.

Remaining manual step: collect real logs from a healthy walking-sim run to
populate the benchmark with on-robot numbers (needs working `cmd_vel`).

Known limitations (by design, for now): `gravity_lp` discards sustained
horizontal acceleration, so leg odometry is the primary linear-velocity source.
The slip model ships trained on synthetic data (`config/slip_model.txt`) — retrain
with `--data` on logged runs for on-robot performance. See [`DESIGN.md`](DESIGN.md)
§6–8 for details.
