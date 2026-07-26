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

# Convenience launcher: opens gnome-terminals for sim, keyboard teleop
# (teleop_twist_keyboard → /cmd_vel), and the ESKF. The ESKF waits for /odom/raw
# (leg odom) AND /gps/fix before starting, so it never dead-reckons before its
# corrections exist (skipping this lets position run away to tens of km — position
# is unobservable without a velocity+position anchor from t=0).
./run_go2_teleop.sh                    # rviz on, GPS on, NVIDIA rendering
./run_go2_teleop.sh --plot             # + ground-truth bridge + live XY/error plot
./run_go2_teleop.sh --software-render   # CPU (llvmpipe) rendering fallback (see gotcha below)

# Live estimate-vs-ground-truth trajectory plot (clamped XY view + error-vs-time).
# Needs ground_truth.launch.py running for the truth overlay; --no-truth otherwise.
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
- **The drift is a HEADING problem, and square results are NOT repeatable.** Across six
  runs, position error tracks yaw error 1:1 (4°→2.3 m … 65°→12.7 m), but identical
  configurations give anywhere from 4° to 97° of yaw error. **Never conclude from one
  square run** — budget ~5+ runs per arm, and relaunch the sim per run (a `gz` world
  reset wedges `controller_manager`). No filter change to date is proven to help. Read
  `skills.md` §0 before touching the yaw path or quoting a drift number.
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
