# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What's in this workspace

A ROS 2 Jazzy + Gazebo Harmonic workspace (Ubuntu 24.04) holding **two independent
robot pipelines** plus vendored upstream code:

1. **Diff-drive demo** (first-party): `ekf_estimator` + `motion_planner` — a 5-state
   EKF feeding a swappable-planner navigation stack on a wheeled robot.
2. **Unitree Go2 quadruped** (first-party `go2_eskf` + vendored `unitree_go2_ros2`):
   an 8-state error-state EKF localizing the CHAMP-driven Go2.
3. **Vendored**: `src/unitree_go2_ros2/` (CHAMP + Go2 sim; has its own `.git`) and
   `src/ros2_control_demos/` (upstream reference, not built into the demos). Treat
   these as third-party — prefer not to edit them; changes belong in the first-party
   packages.

The two pipelines are **separate** — `ekf_estimator`/`motion_planner` target a
diff-drive robot, `go2_eskf` targets the quadruped. They don't run together.

## Build Commands

```bash
# Full workspace build
colcon build --symlink-install

# First-party packages individually
colcon build --packages-select ekf_estimator --symlink-install
colcon build --packages-select motion_planner --symlink-install
colcon build --packages-select go2_eskf --symlink-install

# Clean rebuild
rm -rf build install log && colcon build --symlink-install

# NOTE: the existing install/ uses the MERGED layout, so single-package builds need
# --merge-install or colcon refuses — and then any test run silently uses the OLD binary.
colcon build --packages-select go2_eskf --symlink-install --merge-install

# Source workspace (required before running nodes)
source install/setup.bash
```

## Testing

```bash
# colcon-driven (all first-party tests)
colcon test --packages-select ekf_estimator motion_planner go2_eskf
colcon test-result --all --verbose

# Single package
colcon test --packages-select ekf_estimator

# go2_eskf also exposes its GTest binary directly (fast, no colcon):
./build/go2_eskf/test_eskf_core

# go2_eskf cross-validation: C++ core vs the NumPy twin (must agree to ~1e-14).
# This is the strongest regression test for the ESKF math — run it after any
# change to eskf_core.cpp or eskf_reference.py.
python3 src/go2_eskf/scripts/cross_validate.py

# Same discipline for the slip MLP (C++ Eigen forward pass vs the NumPy twin).
# Run it after retraining — it also proves the new weights file parses in C++.
python3 src/go2_eskf/scripts/cross_validate_slip.py
```

**Never build while a sim run is in flight.** `colcon build` and Gazebo together
OOM-killed the compiler and starved the sim mid-gait; CHAMP fell 19 s into the
square and the "drift" logged past that point was 9964 m of scrabbling-feet
nonsense. Wait for the run to finish.

GTest binaries live at `build/<pkg>/<target>` (e.g. `build/motion_planner/test_planners`)
and accept `--gtest_filter=...` to run a single case.

## Running the System

### Diff-drive demo
```bash
# Full navigation demo (Gazebo + EKF + Navigation + RViz)
ros2 launch motion_planner nav_demo.launch.py
ros2 launch motion_planner nav_demo.launch.py planner_type:=rrt_star
#   planner_type ∈ astar | dijkstra | rrt | rrt_star | prm
#   also switchable at runtime: ros2 param set /navigation_node planner_type rrt_star

# Headless benchmark — compares all 5 planners, prints metrics table
ros2 launch motion_planner benchmark.launch.py

# Standalone EKF only
ros2 launch ekf_estimator ekf_sim.launch.py

# Hardware mode: EKF + nav only, NO Gazebo/bridge (uses ekf_params_hardware.yaml).
# Expects external IMU (/imu), LiDAR (/scan), and diff-drive (/odom, /cmd_vel) drivers.
ros2 launch motion_planner hardware_demo.launch.py

# One-shot build + launch
./quickstart.sh [planner_type]
```

### Unitree Go2 / ESKF
```bash
# Bring up the Go2 Gazebo sim (CHAMP stack)
ros2 launch unitree_go2_sim unitree_go2_launch.py            # add rviz:=true for RViz

# Run the error-state EKF against the sim. The navsat sensor is FIXED (2026-08-28):
# its <stddev> was applied in DEGREES, not metres, so 0.5 meant ~55 km of scatter.
# It is now written in metres and converted at the datum (unitree_go2_gazebo.xacro),
# and MEASURED at 0.54 m N/S, 0.49 m E/W on a stationary robot. GPS is therefore
# safe to fuse, so it is now ON BY DEFAULT (2026-09-04): the filter fuses
# IMU + leg odometry + GPS. use_gps is still a launch arg, so the old
# leg-odom-only behaviour is one flag away (and it is what every pre-2026-09-04
# drift number in skills.md / CLAUDE.md was measured under).
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true                 # GPS on (default)
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true use_gps:=false  # leg odom + IMU only

# Run BOTH estimator arms at once against the same sensor stream: the baseline
# (fixed R_leg -> /eskf/odom) and the slip-adaptive one (use_slip_model -> /eskf_slip/odom).
# Same params, same inputs, so the difference between them IS the slip model — and the
# run-to-run variance that makes two separate runs incomparable cancels out.
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true slip:=true

# Convenience launcher: opens gnome-terminals for sim, keyboard teleop
# (teleop_twist_keyboard → /cmd_vel), and the ESKF. The ESKF waits for /odom/raw
# (leg odom) AND /gps/fix before starting, so it never dead-reckons before its
# corrections exist (skipping this lets position run away to tens of km — position
# is unobservable without a velocity+position anchor from t=0).
#
# TWO estimator arms run by default (baseline + slip-adaptive, see eskf.launch.py
# slip:=true above), so --plot/--square shows all three curves and REPORT.md scores
# both. `--no-slip` goes back to a single estimator.
./run_go2_teleop.sh                    # rviz on, GPS on, NVIDIA rendering
./run_go2_teleop.sh --plot             # + ground-truth bridge + live XY/error plot
./run_go2_teleop.sh --software-render   # CPU (llvmpipe) rendering fallback (see gotcha below)

# Live trajectory plot (clamped XY view + error-vs-time + raw leg twist). Draws FOUR
# curves — ground truth, baseline estimate (/eskf/odom), slip-adaptive estimate
# (/eskf_slip/odom), and leg odometry (/odom/raw) dead-reckoned — with one error trace
# per non-truth curve. Needs ground_truth.launch.py for the truth overlay (--no-truth
# otherwise) and eskf.launch.py slip:=true for the slip curve (--no-slip, or
# --wait-slip N to drop it from the legend if it never publishes).
#
# The /odom/raw curve is NOT its pose field (that one is meaningless — see the CHAMP
# note below); it is the TWIST integrated in the plotter, anchored on ground truth at
# the first sample, so it shows what leg odometry alone would give you. It is drawn
# unscaled by default (the filter applies leg_odom_scale 1.111, so the curve runs ~10%
# short on purpose — pass --leg-scale 1.111 to compare like for like). A third panel
# plots the raw v_x/omega_z against the true body twist and reports the degenerate
# (all-zero) sample share. --no-leg drops both. The window is a 2x2 grid: XY | error
# on top, leg twist | slip score below. The slip panel plots /eskf_slip/slip_score vs
# time (+ 2 s mean, + R_leg inflation axis); --no-slip-score drops it, --slip-lambda
# labels the inflation axis. Switched-off panels are left out and the grid shrinks. The twist
# panel's y-range is the central 99 % of samples, so truth omega_z spikes (up to ~500 rad/s)
# cannot flatten it; the title counts the off-scale samples.
ros2 run go2_eskf plot_trajectory.py --ros-args -p use_sim_time:=true
```

**GPU gotcha — Gazebo sensor-render segfault on NVIDIA Optimus laptops.** Gazebo's
offscreen *sensor* render thread (Ogre2) is separate from the on-screen GUI/RViz
context. On a hybrid NVIDIA+Intel laptop it wrongly picks the Mesa/DRI2 path for the
NVIDIA card (`libEGL warning: ... failed to create dri2 screen`) and **segfaults in
`SensorsPrivate::RenderThread()`**. That kills the whole `gz` server — and with it
the `controller_manager` it hosts — so every controller spawner then hangs on
`Could not contact service /controller_manager/list_controllers` and `/odom/raw`,
GPS, and joint states never appear. It is *not* an ESKF bug. Fix: force the offscreen
renderer onto NVIDIA's own EGL/GLX (what `run_go2_teleop.sh` does automatically):
```bash
export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
```
If PRIME offload still crashes, fall back to CPU rasterisation with
`export LIBGL_ALWAYS_SOFTWARE=1` (the `--software-render` flag). Software rendering is
fine for the ESKF workflow — it consumes IMU, joint states, GPS, and leg odom, none of
which are rendering sensors; the cameras/LiDAR only render for RViz.

## Architecture

### Core design principle (applies to all first-party packages)
Each package splits a **ROS-free math library** (`*_core`, `types.hpp`, planners)
from a **thin ROS 2 node wrapper**. The math is pure C++/Eigen so it unit-tests and
cross-validates with no ROS infrastructure, and stays portable. When adding
functionality, keep algorithm code in the core and ROS plumbing in the node.

### ekf_estimator (diff-drive)
5-state EKF (`[x, y, θ, v, ω]`) fusing IMU gyro, wheel odometry, and LiDAR
scan-matching (ICP). Publishes `/ekf/pose` (PoseWithCovarianceStamped) and
broadcasts `odom → base_link` TF.
- `ekf_core.hpp/.cpp` — pure math, fully unit-tested
- `ekf_node.hpp/.cpp` — ROS wrapper; executable `ekf_estimator_node`
- Tests: `test_ekf_core` (GTest) + `test_ekf_integration.py` (launch_testing)

### motion_planner (diff-drive)
Goal-to-velocity orchestration with swappable planners (Strategy pattern).
- `NavigationNode` (navigation_node.cpp) — orchestrator: `/goal_pose` → build map →
  plan → Pure Pursuit → `/cmd_vel`. Executable `navigation_node`.
- `OccupancyGrid` — 2D collision grid with robot-radius inflation
- `PlannerBase` — abstract interface; five implementations: **A\***, **Dijkstra**,
  **RRT**, **RRT\***, **PRM** (selected via `planner_type` param)
- `PurePursuit` — waypoints → cmd_vel follower
- `benchmark_node` — runs all planners on one start/goal for the metrics table
- Tests: `test_planners`, `test_pure_pursuit`

### go2_eskf (Unitree Go2)
8-state **error-state EKF** (`[p(3), v(3), ψ, b_g]`) fusing **IMU + leg odometry + GPS**
with a slip-adaptive measurement-covariance model. Replaces CHAMP's stock
`robot_localization` dual-EKF and adds GPS-anchored drift correction.
- `eskf_core.hpp/.cpp` + `types.hpp` — pure Eigen ESKF (predict→correct→inject→reset,
  Joseph-form covariance updates)
- `eskf_node.cpp` — ROS wrapper; executable `go2_eskf_node`. IMU-driven predict, leg-odom +
  GPS corrections, lat/lon→ENU conversion, ground-truth CSV logging.
- `scripts/eskf_reference.py` — line-for-line NumPy twin; `cross_validate.py` compares it
  to the C++ core. `tools/replay_eskf.cpp` is the deterministic replay driver.
- Tests: `test_eskf_core` (22 GTest cases)
- **Magnetometer heading (2026-09-23), OFF by default — `use_mag:=true` / `./run_go2_teleop.sh --mag`.**
  `EskfCore::correctYaw` (direct ψ update, wrapped innovation) fed by `EskfCore::magHeading`
  (vector tilt compensation: field + low-passed accelerometer "up", both in the IMU frame, so
  the flipped sim-IMU frame is irrelevant). The gz magnetometer sits on `imu_link` (third
  deliberate vendored edit, additive, in `unitree_go2_gazebo.xacro`); `eskf.launch.py` bridges
  it (`/imu/mag`), and the raw heading is published on `/eskf/mag_heading` for checking against
  truth. Measured on a headless gz world: with `<spherical_coordinates>` set, gz uses its **WMM
  field in GAUSS** (|B| 0.433, 4.0° from world +x) and ignores the SDF `<magnetic_field>`
  default, and the noise `<stddev>` is in gauss too; heading recovered yaw to 0.000° at four
  poses incl. an upside-down mount. Default reference is calibrated at startup against the
  filter's initial yaw (`mag_calibrate_heading`). Offline it beats GPS alone on the square
  (2.00°→0.61°) **only if** the gait's tilt-compensation error is ≲5°; at 10° it is worse —
  measure `/eskf/mag_heading` vs `/ground_truth/odom` yaw on a walking run before enabling it.
  MEASURED since (2026-09-23/24): std 4.1–4.3° on terrain squares, heavy-tailed — the error grows
  with the body's tilt LAG behind the low-passed "up", so R is now tilt-dependent
  (`mag_tilt_gain` 0.60, from a fit) with a 3σ innovation gate (`mag_gate_sigma`) and a 5 s
  lock-out escape. Still not A/B'd (`--mag` vs none) —
  `REPORT.md`'s "Magnetometer heading vs ground truth" section does exactly that (`--mag` + `--square`/`--plot`).
- **Slip model (Phase 3): trained on REAL run data, end to end.** `eskf_node` taps a
  training set at exactly the point inference runs — `slip_log:=<csv>` writes one row per
  *fused* leg-odom update (post-scale, post-gate, ZUPT rows excluded) with the eight
  features plus the ground-truth body twist. `run_go2_teleop.sh` sets it to
  `run_report/slip_features.csv` whenever the ground-truth bridge is up (`--plot`/`--square`).
  Then `train_slip_model.py --runlog <csv>...` labels each row
  `clip(|v_leg − v_truth| / 0.3, 0, 1)` — how wrong leg odometry actually was, which is the
  quantity `R_leg` exists to describe — trains the MLP, and writes `config/slip_model.txt`
  with a provenance header. Ground truth is used for the LABEL only, never fed to the filter.
  `config/slip_model_synthetic.txt` keeps the old never-saw-the-robot weights for A/B.
  Always train on one run and evaluate on others; the rows are a time series.
- **The slip model now retrains after EVERY run** (2026-09-24, `--no-train` to skip):
  `run_go2_teleop.sh`'s `cleanup()` calls `scripts/auto_train_slip.py` once every node has
  stopped. It archives the run's usable rows to `slip_dataset/` (gitignored; rows without truth
  or with `gt_tilt` > 30°, a new slip-log column, are dropped; runs under 500 rows are skipped),
  trains a candidate on every OTHER archived run, and scores it and the deployed model on this run
  (held out, BCE). Only if the candidate is not worse does it retrain on all runs, check the file
  in C++ (`slip_infer` vs NumPy ≤ 1e-9), back the old weights up to
  `config/slip_model_history/` (gitignored), and replace `config/slip_model.txt`. The verdict is
  appended to `REPORT.md`; the full log is `run_report/slip_training.log`. Needs ≥ 2 archived runs
  (seeded with 2). Only runs with the ground-truth bridge (`--plot`/`--square`) produce data.
  `config/slip_model.txt` is git-tracked, so it shows as modified after each deploy.
- **`contact_frac` is a structurally dead feature — measured, not assumed.** It is now
  wired to `/foot_contacts` (optional `champ_msgs` dependency, `find_package(... QUIET)`,
  so the package still builds without the vendored tree). At the instants the slip model
  runs it is **0.5 in 13467/13467 samples**: the degenerate gate drops every 0-and-4-feet
  sample, and a trot in between always stands on exactly one diagonal pair. A useful contact
  feature would have to be sub-gait-cycle (stance duty over a window), not instantaneous.
- `launch/ground_truth.launch.py` — benchmark ground truth: the sim has no world-frame
  Odometry (its `/odom` is the robot_localization estimate; gz pose-vector topics lose entity
  names through `ros_gz_bridge`). A dedicated gz `OdometryPublisher` on the model (added to
  the vendored `unitree_go2_gazebo.xacro`, publishing the *new* topic `/model/go2/ground_truth/odometry`)
  provides true base pose+twist; this launch bridges it to `/ground_truth/odom` for
  `benchmark.launch.py`. (The one deliberate vendored edit — additive, non-conflicting.)
- **Read `src/go2_eskf/docs/DESIGN.md` before touching this package** — it has the full
  state/Jacobian/sensor-model derivation, the phased roadmap (Phase 1–2 done; Phase 3 neural
  slip model + Phase 4 ATE/RPE benchmark implemented, live walking-sim run pending), and the
  sim-integration gotchas (unreliable gz-IMU orientation → `gravity_lp` attitude default;
  contact-impact accel spikes → large `accel_noise` so leg odometry dominates; the world has
  a `<spherical_coordinates>` datum so the `/gps/fix` mean is correct, BUT the Gazebo navsat
  emitted ~0.5° (~55 km) of position noise — its `<stddev>0.5</stddev>` was applied in degrees,
  not metres — **fixed 2026-08-28**, now 0.5 m nominal and measured at 0.54/0.49 m, so GPS is
  fusable and is ON by default since 2026-09-04 (`use_gps:=false` restores the old
  leg-odom-only arm); no sim ground-truth
  Odometry → use `ground_truth.launch.py`). With GPS off, position is unobservable and drifts
  slowly on leg odometry — that is expected in sim; the GPS path is validated by the replay /
  NumPy cross-checks, not the live sim.
- **CHAMP's `/odom/raw` all-zero samples are a no-information flag, not a measurement —
  and they occur only while the robot is STOPPED.** `champ::Odometry::getVelocities`
  early-returns hard zeros for `linear.x`, `linear.y` *and* `angular.z` whenever all four
  or zero feet are in contact. Measured: 0/23,000 samples while walking or turning;
  standing yields unbroken 3–5 s runs. `eskf_node` gates them
  (`leg_odom_gate_degenerate`) and treats a sustained run as a zero-velocity update
  (`degenerate_hold_sec`); set `leg_odom_gate_degenerate: false` for the old
  fuse-everything behaviour. `/odom/raw`'s **pose** is separately meaningless (a `vel_dt`
  unit bug in `state_estimation.cpp`); only its twist is usable.
- **CHAMP's leg odometry now solves the body twist by least squares** (vendored edit #6 in
  `champ/include/champ/odometry/odometry.h`): `-dr_i/dt = v + ω × r_i` over stance feet, in
  centroid-reduced closed form. It replaces a per-foot *bearing* sum that mistook body
  translation for rotation (±0.59 rad/s per foot at 0.25 m/s, cancelling only for a
  perfectly symmetric diagonal pair), and it now ignores touchdown samples whose position
  delta spans the swing rather than the stance. Offline-verified against an exact twist:
  `vx/truth` 0.821→0.900 (= `odom_scaler`, which `leg_odom_scale: 1.111` undoes),
  `wz/truth` 0.923→1.000, false yaw-rate noise on a straight walk 0.0675→0.0000 rad/s.
  **Not yet validated live.** Rebuild `champ` AND `champ_base` (header-only).
- **The drift is a HEADING problem, and square results are NOT repeatable.** Across six
  runs, position error tracks yaw error 1:1 (4°→2.3 m … 65°→12.7 m), but identical
  configurations give anywhere from 4° to 97° of yaw error. **Never conclude from one
  square run** — budget ~5+ runs per arm, and relaunch the sim per run (a `gz` world
  reset wedges `controller_manager`). No filter change to date is proven to help. Read
  `skills.md` §0 before touching the yaw path or quoting a drift number.
- **GPS is a working sensor again (2026-08-28), and it is the ONLY absolute heading
  reference in this stack.** The navsat `<stddev>` unit bug is fixed in
  `unitree_go2_gazebo.xacro` (a deliberate vendored edit, additive: `gps_noise_m` in metres, converted
  by metres-per-degree at the datum); measured on a stationary robot over 400 fixes at
  **0.542 m N/S, 0.491 m E/W**, altitude 0.805 m (vertical was never affected — altitude is
  natively metres). The ENU datum is now the **average of the first `gps_datum_samples: 10`
  fixes**, not one; datum noise is a constant offset on every position the filter reports and
  is the one GPS error later fusion cannot average away. It is fused BY DEFAULT since
  2026-09-04 (`use_gps: true` in `eskf_params.yaml`, `use_gps` default `true` in
  `eskf.launch.py`, `USE_GPS=true` in `run_go2_teleop.sh`); `./run_go2_teleop.sh --no-gps`
  goes back to leg odom + IMU only. **Not yet A/B'd on a square** — per §0's own rule, budget ~5+
  runs per arm before quoting a number, and note `run_report/timeseries.csv` has no GPS
  columns yet, so GPS quality is currently invisible in `REPORT.md`.
- **With GPS off, yaw is EXACTLY unobservable — no filter tuning can fix heading drift.**
  `correctLegOdom` predicts `h = Rz(-ψ)·v_world`, whose Jacobian has the null direction
  `δv = δψ·(-v_y, v_x)`: rotating heading and world velocity together is invisible to leg
  odometry, which constrains body-frame velocity only. Heading is observable only through
  something that pins `v_world` in the world frame — the IMU accel (gutted by `gravity_lp`)
  or GPS position (off). So `ψ` runs open-loop on `∫(gyro_z − b_g)dt`. Only two things can
  help: a smaller yaw-rate disturbance, or an absolute heading reference (fix the navsat
  `<stddev>` and enable GPS, or fuse an AHRS yaw). Don't retune `Q`/`R` at it.
- **Leg-odometry defects are best found OFFLINE.** `scripts/leg_odom_model.py` simulates a
  trot from an exactly known body twist, feeds the estimator its own inputs, and self-checks
  — no ROS, no Gazebo, zero variance. It resolved an 8% yaw-rate error and a 9% speed error
  that six square runs could not, and it is the tripwire for the vendored CHAMP twist
  solver. Run it before booking sim time on any leg-odom question.
- **Perception sensors (cameras + LiDARs) are DISABLED in the vendored model** — commented out
  in `unitree_go2_robot.xacro` (the 3 velodyne/4D-lidar/D455 includes) and `unitree_go2_gazebo.xacro`
  (the `rgb_camera` block). They are the only GPU-rendered sensors, so they (a) triggered a Gazebo
  Ogre2 offscreen-render **segfault** on NVIDIA-Optimus laptops (which killed gz + controller_manager)
  and (b) dominated sim GPU/CPU load. The go2_eskf stack doesn't use them (IMU + leg odometry only).
  Re-enable by uncommenting those blocks. NOTE: `<xacro:include>` can't be gated by `xacro:if` (the
  include pass precedes conditionals), hence plain comments. After editing, rebuild
  `unitree_go2_description` (its installed `urdf/` were stale copies until rebuilt with
  `--symlink-install`). This is the second deliberate vendored edit (after the ground-truth publisher).

### Pipeline (diff-drive demo)
```
Gazebo sensors (/imu, /odom, /scan)
  → ekf_estimator  → /ekf/pose
  → motion_planner → /planned_path → /cmd_vel → Gazebo
```

## Configuration

| File | Purpose |
|------|---------|
| `src/ekf_estimator/config/ekf_params.yaml` | EKF predict rate (50 Hz), Q, per-sensor R (sim) |
| `src/ekf_estimator/config/ekf_params_hardware.yaml` | Real-robot tuning: ground truth off, loosened noise; used by `hardware_demo.launch.py` |
| `src/motion_planner/config/planner_params.yaml` | Active planner, map resolution (0.1 m/cell), robot radius (0.25 m), Pure Pursuit gains |
| `src/go2_eskf/config/eskf_params.yaml` | IMU noise densities, attitude source, GPS toggle, slip params |
| `src/unitree_go2_ros2/unitree_go2_sim/config/gait/gait.yaml` | CHAMP gait tuning (velocity limits, stance/swing) |

**Keep in sync:** the diff-drive Gazebo world (`src/motion_planner/worlds/obstacle_course.sdf`)
must match `NavigationNode::buildDemoMap()`. The Go2 GPS origin lives in
`unitree_go2_description/worlds/default.sdf` under `<spherical_coordinates>`.

### Go2 worlds (three, switched by flag — no file editing, trivially reversible)

| flag | world | use |
|------|-------|-----|
| *(none)* | `go2_eskf/worlds/flat.sdf` | **default** — obstacle-free flat floor |
| `--obstacles` | vendored `default.sdf` | the five boxes/cylinders |
| `--terrain` | `go2_eskf/worlds/terrain.sdf` | uneven heightmap + low-friction patches |

`terrain.sdf` is **generated** by `scripts/make_terrain_world.py` (a fractal heightmap plus
four `mu=0.3` patches at the leg midpoints of the **10 m** square: (5,0), (10,-5), (5,-10),
(0,-5)). **`square_test.py --side` now defaults to 5 m**, so the driven route is half that
box and touches only two of the four patches, at its corners rather than mid-leg — halve
`SQUARE`/`DEFAULT_PATCHES` in the generator and regenerate if the friction patches are the
point of the run, or drive `-- --side 10`. It exists because the flat world has a rigid
no-slip floor and so produces *no slip at all* — the slip model has nothing to detect there.
Difficulty is set either by `--relief` (peak elevation) or, preferably, by **`--max-slope DEG`**,
which caps the steepest cell anywhere and solves the required relief in closed form (slope is
exactly linear in relief). The **current world is `--max-slope 11.3`** → relief 0.40 m, slope
median 2.0° / max 11.3°, square path median 2.1° / max 9.6°. For reference by relief:
0.6 m → 3.2°/14.4°, 0.7 m → 3.7°/16.6°, 1.0 m → 5.2°/23.1°, 1.6 m → 8.4°/34.3° (square-path
median/max). CHAMP's blind gait already falls occasionally on flat ground, so raise it
gradually — **it stalled outright on a sustained 11.9° climb** (see `skills.md` §0 "Terrain
BLOCKER"), which is why the cap exists. The start pad is flat at elevation 0, so the robot
spawns exactly as it does over `flat.sdf` and flat-vs-terrain runs start from an identical
pose; note the pad **blend** (`--pad-radius`/`--pad-blend`) concentrates the flat→relief
transition into a narrow annulus, so the steepest part of the route can sit a couple of metres
from spawn. **Reverting to flat is just dropping the flag** — `flat.sdf` is never touched.

### Run report — read this instead of asking for screenshots

**Every `run_go2_teleop.sh` run writes `run_report/REPORT.md`** (`--no-report` disables).
Everything in `run_report/` is **overwritten each run**, so it costs a fixed ~0.5 MB
regardless of how many runs happen. `run_report/` is gitignored.

| file | contents |
|------|----------|
| `REPORT.md` | config (flags/world/gains/gait), outcome + **stall detector** (where progress stopped while still commanded), terrain elevation & slope under the ground-truth path, gait health (per-leg contact duty, **joint tracking error** = the stance-sag metric, leg-odom degenerate %), estimator ATE/final/yaw error **with one column per estimator arm** (baseline vs slip-adaptive), topic rates with **MISSING topics flagged**, and de-duplicated WARN/ERROR lines from every node |
| `timeseries.csv` | merged series at 5 Hz, hard row cap (decimates itself on overflow); last two columns `mag_heading`/`mag_err` are empty unless `--mag` |
| `context.txt` | launcher flags/world/gains (written by the shell) |
| `node_logs/` | per-node stdout, copied by `cleanup()` **before** it deletes its scratch dir — otherwise this is lost |
| `slip_features.csv` | slip-model training set (~2 MB), written when the ground-truth bridge is up. Feed to `train_slip_model.py --runlog` |
| `outcome.txt` | how the run ENDED (`square test COMPLETE` / `FAILSAFE TIMEOUT after Ns`), written by the launcher just before teardown; `REPORT.md` reproduces it under **Outcome** |

**`square_test.py` ramps its commands and survives stumbles** (2026-09-23): `cmd_vx`/`cmd_wz` are
rate-limited on SPEED-UP only (0→0.25 m/s over 2 s; slowing/stopping is immediate) and steering
uses a 0.25 s low-passed truth heading; a stall is judged by the maximum excursion over the window
(so circling a corner is not a stall); a fall
condition pauses the square and it resumes once the robot is settled for 1 s, aborting only if the
condition persists 0.5 s or it has not settled in 5 s. `-- --no-shaping` restores step commands.

**Runs stop themselves.** `run_go2_teleop.sh` supervises two automatic exits: the square's
drift summary appearing, and a `--timeout SEC` wall-clock cap — **300 s by default under
`--square`**, sized for the 5 m route (~185 s end to end, ~90 s of it boot). A 10 m route
(`-- --side 10`) takes ~370 s and **will be cut off** unless `--timeout` is raised;
`--no-timeout` opts out. Both exits call the same `cleanup()` as Ctrl-C, so a timed-out run
still copies `node_logs/` and lets `run_report.py` write its final report — a wedged run
stays diagnosable rather than sitting on the GPU indefinitely.

`REPORT.md`'s config table also carries a **Slip model** row mined from the ESKF node's own
stdout (`loaded — <path>` / `**FAILED TO LOAD**` / `**NOT LOADED**`). Read it before
interpreting any A/B: an empty or unparseable `slip_model_path` makes the slip arm fall back
to the fixed `R_leg` *silently*, so the two arms become identical and the comparison is a
null test that looks like a result.

**The launcher's Ctrl-C handler cannot be triggered from a script.** `run_go2_teleop.sh`
traps `INT`, but a command started asynchronously from a non-interactive shell has SIGINT
set to IGNORE, and bash cannot trap a signal ignored on entry — so `kill -INT` on a
backgrounded launcher does nothing, it gets SIGKILLed, `cleanup()` never runs, and the whole
stack is orphaned with STALE `node_logs/` (which then get read as if they were this run's).
Use `kill -TERM`, which the same trap catches.

Implemented by `scripts/run_report.py` (passive subscriber, starts at t=0, rewrites the
report every 20 s as well as at shutdown). The flush timer runs on **wall** time on
purpose: if gz never starts, `/clock` never advances and sim-time timers never fire — the
run that most needs a report. Verified: with no `/clock` at all it still lands a report
whose topic table reads `0 — NEVER RECEIVED` for everything.

### Slope locomotion: the two CHAMP adaptations (both first-party, both A/B flags)

CHAMP is **blind and has zero terrain adaptation**: `quadruped_controller.cpp` sets
`req_pose_.position.z = nominal_height` once and never touches orientation, `req_pose_` moves
only via the `/body_pose` topic, and nothing in the vendored launch publishes it. So all four
feet are held on one plane in the base frame forever. Two first-party, independently
switchable fixes exist; **neither is validated in sim yet**:

| flag | what it does |
|------|--------------|
| `--adapt` | **Pitch sign FIXED 2026-09-23** — before that `com_shift_x` shifted the CoM DOWNHILL on every slope (`skills.md` §0), so no earlier `--adapt` result is valid. Runs `scripts/terrain_adapt.py`, which publishes `/body_pose`: shifts the body uphill (`com_shift_x`), crouches on slopes (`crouch`), and optionally levels the body against the ground (`level_pitch`, default 0 = stock). Attitude comes from a **gyro + accelerometer complementary filter** (`attitude_mode: complementary`, default since 2026-09-23; `cf_tau` 10 s; `accel` = the old accel-only low-pass, which read a spurious 19° at gait start), not the gz IMU orientation (unreliable) or the ESKF (yaw-only). It publishes `/terrain_adapt/pitch`, which `REPORT.md` scores against truth. All gains 0 ⇒ bit-for-bit stock. |
| `--stiff` | points the launch's `ros_control_file` arg at `go2_eskf/config/ros_control_stiff.yaml` (p 100→300, d 1.0→3.5). The stock effort-mode PID gives away ~3 cm of stance sag under the 15.1 kg robot — stroke the climb never gets. Actuators have 2.4× headroom and gz ignores URDF command limits, so the torque is really applied. Vendored `ros_control.yaml` untouched. |

`--climb` is shorthand for `--terrain --adapt --stiff`. A/B them **one at a time**.
**Read `src/go2_eskf/docs/SLOPE_POSTURE.md` before touching either** — it derives the
posture geometry (CoM projection `Δ = h·tan γ`, the ideal `com_shift_x = h = 0.225`, why
`level_pitch = 1.0` only halves the pitch, why swing height and ground friction were never
the constraint, and why 11.3° is exactly the walkable limit of the μ=0.3 patches).

Measured facts about gz heightmaps (gz sim 8.11), all established by experiment:
- Heightmap collision **works** under `bullet-featherstone`, and `<surface><friction><ode><mu>`
  **is** honoured (a box slid off a 10° `mu=0.08` ramp and held on an identical `mu=1.0` one).
- Image column → **+X**, row 0 → **+Y** (max Y).
- Elevation = `pixel / (MAX pixel in the image) * size_z` — gz normalises by the image's own
  maximum, **not** by 255. The generator always emits a 255 maximum so it is just `pixel/255`.
- A heightmap `<uri>` resolves **only** as an absolute `file://` path — neither a
  world-relative path nor `GZ_SIM_RESOURCE_PATH` works. So `terrain.sdf` is machine-specific;
  re-run the generator after moving the workspace. If the path goes stale gz loads a world
  with *no ground* and the robot falls forever, so `run_go2_teleop.sh --terrain` checks the
  referenced PNG exists and fails loudly instead.

## Dependencies

ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04.

```bash
# Diff-drive demo
sudo apt install ros-jazzy-ros-gz ros-jazzy-tf2-geometry-msgs \
                 ros-jazzy-rviz2 libeigen3-dev

# Unitree Go2 sim (CHAMP) additionally needs:
sudo apt install ros-jazzy-gazebo-ros2-control ros-jazzy-xacro \
                 ros-jazzy-robot-localization ros-jazzy-ros2-controllers \
                 ros-jazzy-ros2-control ros-jazzy-velodyne ros-jazzy-velodyne-description
```
