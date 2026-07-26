# go2_eskf — Design & Report

An 8-state **error-state EKF (ESKF)** for Unitree Go2 localization, fusing
**IMU + leg odometry + GPS**, with a **slip-adaptive measurement-covariance**
model. This document is both the design spec and the running write-up.

---

## 0. Where this fits in the robot

The Go2 is driven by **CHAMP**:

```
/cmd_vel → body posture → gait generator → inverse kinematics → joint_trajectory
         → joint_trajectory_controller (PID, effort) → gz_ros2_control → Gazebo
```

For *localization*, CHAMP produces **leg odometry** (`odom/raw`): it assumes a
foot in stance is stationary in the world, so the apparent backward motion of
stance feet (from forward kinematics on the joint encoders) reveals body
velocity. The stock stack fuses that with IMU via two `robot_localization`
EKFs, and **never uses GPS**.

`go2_eskf` replaces that with a single, transparent filter that **also fuses
GPS** (global drift correction) and **adapts its trust in leg odometry when
feet slip** — the exact failure mode of the stance-foot assumption.

---

## 1. State

Nominal state (8):

| idx | symbol | meaning | units |
|-----|--------|---------|-------|
| 0–2 | `px py pz` | position, world (ENU) | m |
| 3–5 | `vx vy vz` | velocity, world | m/s |
| 6   | `psi` | yaw / heading | rad |
| 7   | `b_g` | yaw-rate gyro bias | rad/s |

**Why 8 and not 15?** Roll and pitch are directly observable from the IMU
(gravity defines "down"), so we read them from the IMU rather than estimating
them. We keep only the slowly-drifting, weakly-observable quantities as states.
This is the standard reduction for a mostly-planar legged robot with GPS.

**Why "error-state"?** We propagate the full nonlinear nominal state directly,
and the Kalman filter tracks the small *error* `δx` about it, injecting `δx`
back after each correction (`x ← x ⊞ δx`, then reset `δx = 0`). For a yaw-only
rotation the error is additive (SO(2)), so injection is a simple add — but the
predict→correct→inject→reset structure is the ESKF recipe and extends cleanly
to a full 3-D attitude (SO(3) multiplicative error) later.

---

## 2. Prediction — IMU driven

Inputs each IMU tick: specific force `a_b` (body), yaw rate `ω_z`, roll `φ` &
pitch `θ` (from IMU), step `dt`.

Body→world rotation `R = Rz(ψ)Ry(θ)Rx(φ)`. World acceleration removes gravity:

```
a_w = R · a_b + [0, 0, −g],     g = 9.80665
```

Nominal propagation (constant-acceleration over the step):

```
p ← p + v·dt + ½·a_w·dt²
v ← v + a_w·dt
ψ ← wrap(ψ + (ω_z − b_g)·dt)
b_g ← b_g                       (random walk)
```

Error-state transition `F = ∂δx_{k+1}/∂δx_k` (non-identity blocks):

```
∂p/∂v   = I₃·dt
∂v/∂ψ   = (∂R/∂ψ · a_b)·dt
∂p/∂ψ   = ½·(∂R/∂ψ · a_b)·dt²
∂ψ/∂b_g = −dt
```

Discrete process noise `Q` from IMU noise densities (`σ_a`, `σ_g`, `σ_bg`):

```
Q_pp = I₃·(¼ σ_a² dt⁴)   Q_vv = I₃·(σ_a² dt²)   Q_ψψ = σ_g² dt²   Q_bb = σ_bg² dt
```

Covariance: `P ← F P Fᵀ + Q`.

---

## 3. Corrections

Both use the **Joseph form** `P ← (I−KH)P(I−KH)ᵀ + KRKᵀ` (keeps `P` symmetric
and positive-definite) and inject additively into the nominal state.

**Leg odometry** — measures body-frame planar velocity `[vx_b, vy_b]`:

```
h(x)   = Rz(−ψ) · [vx, vy]ᵀ
H_v    = Rz(−ψ)            (∂h/∂v_xy)
H_ψ    = (∂Rz(−ψ)/∂ψ) · [vx, vy]ᵀ
```

`R_leg` is the knob the **slip model** turns: nominal when feet are planted,
inflated when slip is detected, so the filter leans on IMU+GPS instead.

**GPS** — measures world position `[x, y]`: `h = [px, py]`, `H` selects them.
This is the only globally-anchored sensor; it bounds long-term drift.

---

## 4. Cross-validation (résumé bullet 1)

`scripts/eskf_reference.py` is a NumPy twin of the C++ filter, line-for-line.
`scripts/cross_validate.py` feeds **one deterministic sensor stream** to both
and compares state trajectories:

```
colcon build --packages-select go2_eskf
python3 src/go2_eskf/scripts/cross_validate.py
# → OVERALL max abs diff : ~1e-14   PASS
```

Agreement is limited only by BLAS-vs-NumPy rounding (matrix inverse ordering),
which accumulates to the ~1e-11 range on longer aggressive runs. This is the
cheapest, strongest regression test for the math: any bug in either side breaks
agreement immediately.

---

## 5. Roadmap

| Phase | Deliverable | Résumé bullet |
|-------|-------------|---------------|
| **1 ✅** | ESKF core + 16 unit tests + NumPy cross-validation | #1 |
| **2 ✅** | ROS node (`eskf_node`): subs `imu/data`, `odom/raw`, `gps/fix`; GPS lat/lon→local ENU; publishes `eskf/odom` (+ optional TF); ground-truth CSV logging | #1 |
| **3 ✅** | Slip model: PyTorch trainer (+ NumPy fallback) → exported weights → dependency-free Eigen MLP (`slip_model.hpp`) → adaptive `R_leg`; 12 unit tests + C++≡NumPy slip cross-validation (~3e-16) | #2 |
| **4 ✅** | Benchmark: ATE / RPE / drift metrics with SE(2) alignment; fixed vs adaptive vs GPS-denied scenarios; auto-generated markdown report + plots | #3 |

## 6. Phase 2 — live ROS node & sim-integration findings

`eskf_node` (`src/eskf_node.cpp`, config `config/eskf_params.yaml`, launch
`launch/eskf.launch.py`) drives prediction from the IMU and corrects with leg
odometry and GPS. Run it against the live sim with:

```bash
ros2 launch go2_eskf eskf.launch.py            # alongside unitree_go2_sim
# or standalone:
ros2 run go2_eskf go2_eskf_node --ros-args -p use_sim_time:=true
```

Bringing it up against the real Gazebo Go2 surfaced three concrete issues that
the design now handles — each verified on live data via the NumPy twin:

1. **The sim gz-IMU `orientation` is unreliable.** For an upright robot
   (`odom→base_link` identity, z≈0.43) the IMU reported roll≈173° and a
   negative accel-z, and `R·accel` did *not* recover vertical gravity. Trusting
   it leaks a fake horizontal acceleration → position runs to ∞.
   **Fix:** `attitude_source` default `gravity_lp` — low-pass the accelerometer
   to track the gravity+mount vector, subtract it for motion acceleration, and
   rotate by yaw only. Gravity cancels by construction, independent of the IMU
   frame flip. (`accel` per-sample leveling and `orientation` are also
   selectable.)

2. **Contact impacts spike the accelerometer to ~215 m/s²** (median ≈ g). With
   a small `accel_noise` the filter believes them and diverges.
   **Fix:** model the IMU honestly with a large `accel_noise` (default 1.0) so
   **leg odometry dominates the velocity estimate**, plus `accel_clip` on the
   motion-acceleration magnitude. On a standing robot the estimate then stays
   pinned at the origin (validated live).

3. **The sim `navsat` originally had no world datum**, so early `gps/fix` lat/lon
   were mutually inconsistent run-to-run (≈0.5° ⇒ tens of km of ENU error). This
   is now fixed: `unitree_go2_description/worlds/default.sdf` sets a
   `<spherical_coordinates>` origin, so fixes are self-consistent and GPS fusion
   (`use_gps:=true`) can be enabled. The node ignores the bridged (all-zero)
   `position_covariance` and uses the `gps_pos_noise` parameter instead.

**Validation status:** core math proven by 16 unit tests + C++≡NumPy
cross-validation (~1e-14); the live-data behaviour (bounded estimate, leg-odom
dominance) validated by replaying the real sim IMU/leg streams through the
NumPy twin, which shares the node's algorithm exactly. A standing robot yields
`pos≈(0,0,0)`. End-to-end C++-against-sim tracking with a *walking* robot is the
first task of Phase 4's benchmark (needs a healthy sim run + working `cmd_vel`).

### Known limitation (by design, for now)
`gravity_lp` discards sustained horizontal acceleration along with gravity, so
the IMU contributes attitude + yaw-rate + transients, while **leg odometry is
the primary linear-velocity source** — a deliberate, defensible choice given
the sim IMU quality. A future upgrade is a proper complementary/Mahony attitude
filter (gyro-integrated, accel-corrected) so the accelerometer can contribute
true horizontal motion.

## 7. Phase 3 — slip-adaptive leg covariance

A small MLP maps an 8-feature per-step vector to a slip score `s ∈ [0,1]`, and
we scale `R_leg ← R_leg · (1 + λ·s)²`: planted → trust leg odometry; slipping →
distrust it so the filter leans on IMU + GPS.

**Feature vector** (`SlipFeatures` in `include/go2_eskf/slip_model.hpp`, mirrored
in `scripts/slip_reference.py`): commanded-minus-measured body velocity (vx, vy),
commanded-minus-measured yaw rate, leg-odometry speed, joint-velocity mean and
max, horizontal motion-acceleration magnitude, and feet-in-contact fraction.
The disagreement terms are the signal — under slip the commanded/joint motion
persists while the measured body velocity and contact collapse.

**Inference** is a dependency-free Eigen MLP (`SlipModel`, header-only): it loads
a plain-text weights file (input standardization + Linear/ReLU/Sigmoid layers)
and runs a few matmuls. No ML runtime in the node.

**Training** (`scripts/train_slip_model.py`) uses PyTorch for the full 2-hidden-
layer MLP when available, with a NumPy logistic-regression fallback so the
pipeline runs anywhere. It trains on logged feature/label CSVs (`--data`) or a
physics-inspired synthetic dataset, and exports the same weights format.

**Verification** (the project's signature method): `scripts/slip_reference.py` is
a line-for-line NumPy twin of the C++ forward pass, and
`scripts/cross_validate_slip.py` confirms C++ ≡ NumPy slip scores to ~3e-16,
plus 12 GTest cases in `test/test_slip_model.cpp`.

**Node integration:** with `use_slip_model:=true` and `slip_model_path` set, the
node subscribes to `cmd_vel` and `joint_states`, assembles the feature vector
each leg-odom correction, runs the model, inflates `R_leg`, and republishes the
score on `eskf/slip`. Live end-to-end validation on slippery terrain needs a sim
run; the inference math is proven by the cross-validation and unit tests.

## 8. Phase 4 — benchmark

`scripts/metrics.py` computes, from the node's estimate-vs-ground-truth CSV log:
- **ATE** — RMSE of position after a rigid **SE(2) (Umeyama) alignment**, so it
  measures trajectory shape, not the estimator's arbitrary start frame.
- **RPE** — relative pose error (translation + rotation) over a fixed index gap;
  a local drift-rate measure needing no global alignment.
- **final drift** — terminal position error, absolute and as a % of path length.

`scripts/run_benchmark.py` runs these across scenarios (**fixed** vs **adaptive**
covariance, **GPS-denied**), writes a markdown table, and plots the trajectory
overlay and aligned position-error-over-time. `launch/benchmark.launch.py` runs
the node per scenario with logging enabled. `run_benchmark.py --demo` synthesizes
the three scenarios so the report/plot pipeline runs without the sim. Each script
has a self-contained check (`metrics.py --selftest`).

**Ground-truth source.** The Go2 sim publishes no world-frame `nav_msgs/Odometry`
on its own: `/odom` is the `robot_localization` estimate (not truth), and the gz
pose-vector topics (`/world/<world>/pose/info`, `dynamic_pose/info`,
`/model/go2/pose`) lose their entity names through `ros_gz_bridge` — the
`Pose_V → tf2_msgs/TFMessage` conversion blanks `frame_id`/`child_frame_id`, so
the robot can't be matched by name (verified live: all child ids come through
empty). The clean fix is a dedicated Gazebo **`OdometryPublisher`** on the model
(`unitree_go2_gazebo.xacro`) that emits the true base pose+twist on
`/model/go2/ground_truth/odometry` — a *new* gz topic, so it doesn't touch the
`/odom` chain the removed plugin used to own. `launch/ground_truth.launch.py`
bridges it straight to `/ground_truth/odom` (`gz.msgs.Odometry → nav_msgs/Odometry`,
no name matching, twist included). Ground truth is relative to the spawn pose,
which is exactly what the estimator is compared against (both start at the origin).
Point the benchmark at it with `ground_truth_topic:=/ground_truth/odom`.

The remaining manual step is collecting real logs from a healthy walking-sim run
(needs working `cmd_vel`) to populate the report with on-robot numbers.
