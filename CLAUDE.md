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
```

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

# Run the error-state EKF against the sim. GPS is OFF by default: the world has a
# <spherical_coordinates> datum (so the /gps/fix MEAN is right), but the Gazebo
# navsat emits ~0.5 deg (~55 km) of position NOISE — its <stddev>0.5</stddev> is
# applied in degrees, not metres — so fusing it wrecks the estimate. Enable only
# with a real/fixed GPS. use_gps is a launch arg.
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true                # GPS off (default)
ros2 launch go2_eskf eskf.launch.py use_sim_time:=true use_gps:=true  # only if navsat is fixed

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

# Live trajectory plot (clamped XY view + error-vs-time). Draws THREE curves —
# ground truth, baseline estimate (/eskf/odom), slip-adaptive estimate (/eskf_slip/odom) —
# with one error trace per estimator arm. Needs ground_truth.launch.py for the truth
# overlay (--no-truth otherwise) and eskf.launch.py slip:=true for the third curve
# (--no-slip, or --wait-slip N to drop it from the legend if it never publishes).
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
- Tests: `test_eskf_core` (16 GTest cases)
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
  emits ~0.5° (~55 km) of position noise — its `<stddev>0.5</stddev>` is applied in degrees,
  not metres — so GPS is OFF by default (fusing it wrecks the estimate); no sim ground-truth
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
- **With GPS off, yaw is EXACTLY unobservable — no filter tuning can fix heading drift.**
  `correctLegOdom` predicts `h = Rz(-ψ)·v_world`, whose Jacobian has the null direction
  `δv = δψ·(-v_y, v_x)`: rotating heading and world velocity together is invisible to leg
  odometry, which constrains body-frame velocity only. Heading is observable only through
  something that pins `v_world` in the world frame — the IMU accel (gutted by `gravity_lp`)
  or GPS position (off). So `ψ` runs open-loop on `∫(gyro_z − b_g)dt`. Only two things can
  help: a smaller yaw-rate disturbance, or an absolute heading reference. Don't retune
  `Q`/`R` at it.
- **There is now an absolute-heading option: `use_magnetometer:=true`.** `EskfCore::correctYaw`
  measures `ψ` directly (`H` is a single 1 in the `PSI` column), which is the only thing that
  breaks the null space above while standing still. The sim carries a non-rendering
  `magnetometer` sensor on `imu_link` (vendored edit #3) with `<magnetic_field>` pinned in
  `flat.sdf`/`terrain.sdf`; `magnetic_declination` in `eskf_params.yaml` **must match that
  field** (13.67°) or you get a constant heading bias. The innovation is angle-wrapped — the
  only correction here that needs it; see `docs/DESIGN.md` §3 and §9. **OFF by default and
  NOT yet validated on a live run**: the math is cross-validated (C++≡NumPy 1.1e-15 across
  the ±π seam) but the sim magnetometer has no hard/soft iron, so expect the first real run
  to need `mag_yaw_sign`/`mag_yaw_offset`. Calibrate against `/eskf/mag_yaw`, which publishes
  the heading **pre-fusion** for exactly that purpose.
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
four `mu=0.3` patches on the 10 m square). It exists because the flat world has a rigid
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
| `timeseries.csv` | 29-column merged series at 5 Hz, hard row cap (decimates itself on overflow) |
| `context.txt` | launcher flags/world/gains (written by the shell) |
| `node_logs/` | per-node stdout, copied by `cleanup()` **before** it deletes its scratch dir — otherwise this is lost |

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
| `--adapt` | runs `scripts/terrain_adapt.py`, which publishes `/body_pose`: shifts the body uphill (`com_shift_x`), crouches on slopes (`crouch`), and optionally levels the body against the ground (`level_pitch`, default 0 = stock). Attitude comes from a low-passed accelerometer, not the gz IMU orientation (unreliable) or the ESKF (yaw-only). All gains 0 ⇒ bit-for-bit stock. |
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
