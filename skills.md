# skills.md — go2_eskf sim-integration knowledge & session handoff

Working log of the go2_eskf ↔ Unitree Go2 sim integration debugging. Read this
first when resuming — it captures hard-won findings that aren't obvious from the
code. (Companion to `CLAUDE.md`; this file is the narrative + current state.)

Last updated: 2026-07-25.

---

## 0. CURRENT STATE (start here)

**LATEST (2026-07-25 eve):** leg odometry FIXED — the estimate now moves and tracks
DISTANCE (~6 m, matched truth). New/last issue: **heading was mirrored in y** (estimate
curved −y, truth +y). Diagnosed as the sim IMU's **negated yaw rate** (IMU ~180° flipped,
DESIGN §6.1; gravity_lp fixes the accel but gyro-z is used directly). **Fix applied:**
`gyro_z_sign` param (default +1) applied to `gyro_wz_` in `eskf_node.cpp`, set to **-1**
in `eskf_params.yaml`. Rebuilt go2_eskf. **VERIFY:** re-run `--plot`, drive a curve — the
red path should now follow blue (same turn direction). If still mirrored, the flip is in
leg-odom `vy` instead; if rotated-but-not-mirrored, it's yaw drift (gyro bias) during the
initial stand — consider fusing leg-odom yaw rate (`/odom/raw` twist.angular.z, currently
UNUSED) or seeding/holding yaw while stationary.

---

**Earlier story:** the ESKF estimate was **frozen at the origin** in XY while the
robot walked away. Confirmed CHAMP leg odometry `/odom/raw` twist.linear.x = **0.0
even while walking**.

**Root cause (confirmed):** `champ_base/src/state_estimation.cpp` computes leg-odom
velocity in a **message_filters synchronized callback needing BOTH `/joint_states`
AND `/foot_contacts`**. The sim launched `quadruped_controller` with
`publish_foot_contacts: False`, so `/foot_contacts` was never published → sync never
fired → velocity stayed 0. (`/foot_contacts` appeared in `ros2 topic list` only as a
subscription, no publisher.)

**FIX (two parts — the launch flag alone did NOTHING):**
1. `publish_foot_contacts: True` in `unitree_go2_sim/launch/unitree_go2_launch.py`
   (rebuild `unitree_go2_sim`).
2. **The real blocker:** `quadruped_controller.cpp` gated contact publishing behind
   `if(publish_foot_contacts_ && !in_gazebo_)` at lines 84 (publisher creation) and
   190 (publish) — so it **refused to publish `/foot_contacts` in gazebo mode**. The
   contacts are the gait-phase stance estimate (`base_.legs[i]->gait_phase()`), a valid
   fallback the code comment endorses. Removed `&& !in_gazebo_` in both places; rebuilt
   `champ_base` (C++). The sync policy is ApproximateTime, so stamps needn't match exactly.
This is the **3rd deliberate vendored edit** (see §5).

**NEXT STEP — verify:** re-run `./run_go2_teleop.sh --plot`, drive forward, and:
```bash
ros2 topic echo /odom/raw --field twist.twist.linear.x   # expect ~0.15 while walking now
```
- If `/odom/raw` is now non-zero → the estimate should TRACK the robot (accel_noise=50
  already makes leg odom dominate; vz pseudo-measurement keeps Z bounded). Send the plot.
- If STILL 0.0 → `/foot_contacts` may be published but not sync'd, or contacts are all
  false / `getVelocities` returns 0. Then check: `ros2 topic hz /foot_contacts`, echo it,
  and confirm `state_estimation`'s `synchronized_callback_` is firing. Fallback: feed
  `/cmd_vel` as the ESKF velocity source (open-loop; defeats slip model).

---

## 1. What was built (first-party, keep)

- **`run_go2_teleop.sh`** (workspace root) — one-shot launcher: sim + keyboard
  teleop + ESKF (+ ground-truth bridge + live plot with `--plot`).
  - Each component runs as a background child in its **own process group**
    (`set -m`); a single **Ctrl-C in the launching shell** tears everything down
    (SIGINT→SIGKILL the groups + `pkill teleop_twist_keyboard`). Verified: 6 procs → 0.
  - **Separate windows per component** (NOT tabs — see §4 gotcha). Teleop runs live
    in its own window.
  - Flags: `--rviz` (RViz is OFF by default — heavy), `--plot`, `--lite` (lowest
    load), `--software-render` (CPU rendering fallback). Non-critical nodes are
    `nice`d; plot is throttled and started only after the sim is up.
  - Sets NVIDIA offscreen-render env for Gazebo (see §3) — now mostly moot since
    perception sensors are disabled.
- **`src/go2_eskf/scripts/plot_trajectory.py`** — live matplotlib: left = top-down
  XY of `/eskf/odom` (red) vs `/ground_truth/odom` (blue), clamped ±10 m view with
  an OUT-OF-VIEW banner; right = position-error vs time. Flags via `-- --no-truth
  --view N --follow --interval S`. Runs its own ROS spin thread. Registered in
  `CMakeLists.txt`; `ros2 run go2_eskf plot_trajectory.py`.

## 2. Bugs found & fixes applied (chronological)

1. **Gazebo sensor-render SEGFAULT on NVIDIA Optimus** (`SensorsPrivate::RenderThread`
   → Ogre2 GL3Plus → `failed to create dri2 screen`). Killed gz → controller_manager
   → everything hung on "Could not contact service /controller_manager/list_controllers".
   - Interim fix: force NVIDIA EGL/GLX offscreen (`__NV_PRIME_RENDER_OFFLOAD=1`,
     `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `__EGL_VENDOR_LIBRARY_FILENAMES=.../10_nvidia.json`),
     wired into the launcher; `--software-render` = `LIBGL_ALWAYS_SOFTWARE=1` fallback.
   - **Permanent fix:** disabled the GPU-rendered sensors entirely (see §5). No render
     sensors → no render thread → crash is gone in any mode.

2. **ESKF diverged to ~55 km with GPS ON.** The sim navsat emits **~0.5° (~55 km) of
   position NOISE** while stationary — its `<stddev>0.5</stddev>` is applied in
   **degrees, not metres** (mean is correct at the datum, noise is ~100,000× too big).
   The node trusted GPS at R=(0.5 m)², so each fix yanked the estimate tens of km.
   - **Fix:** `use_gps: false` by default (yaml + `eskf.launch.py` arg + launcher).
     Can't be tuned around. To actually use GPS in sim, fix the navsat `<stddev>` to
     ~`4.5e-6` (= 0.5 m ÷ 111320 m/°) — vendored `unitree_go2_gazebo.xacro`.

3. **Estimate Z exploded to 1601 m.** `pz`/`vz` are **unobservable** — leg odom
   corrects only planar velocity, GPS off. IMU/gravity residual double-integrates.
   - **Fix (code):** `EskfCore::correctVerticalVel(vz_meas=0, r)` — a `vz≈0` pseudo-
     measurement applied with every leg-odom update (ground robot doesn't climb).
     `vz_zero_noise=0.3` allows the gait's bob. Added `josephUpdate<1>` instantiation.

4. **Estimate frozen in XY (leg odom ignored).** With `accel_noise=1.0`, the leg-odom
   Kalman gain was ~0.06, so the filter trusted its zero-mean IMU-noise velocity.
   - **Fix (tuning):** `accel_noise 1→50` (leg odom dominates), `leg_odom_vel_noise
     0.2→0.1` (trust it), `accel_clip 40→5` (reject 215 m/s² contact spikes).
   - …which then exposed that leg odom itself is ~0 → §0 open issue.

5. **Terminal UX.** gnome-terminal 3.52 can't reliably open multiple tabs in one
   window from the CLI → switched to separate windows + single-Ctrl-C teardown.

## 3. Key architecture / behaviour facts

- **ESKF pipeline** (`eskf_node.cpp` + `eskf_core.cpp`): state `x=[p(3),v(3),ψ,b_g]`.
  - Predict (IMU ~100 Hz): `gravity_lp` low-passes accel to cancel gravity →
    `a_world=Rz(ψ)·motion`; propagate p,v,ψ; `P=FPFᵀ+Q`.
  - Correct `/odom/raw` (leg odom, body-frame velocity) → `correctLegOdom` — ONLY
    velocity constraint.
  - Correct `/gps/fix` (world x,y) → `correctGps` — ONLY absolute-position anchor
    (off in sim).
  - Correct `vz≈0` → `correctVerticalVel` — anchors the vertical channel.
  - `gravity_lp` **discards sustained horizontal accel**, so **leg odometry is the
    primary linear-velocity source** (DESIGN §6). If leg odom is bad, XY is bad.
- **Ground truth** = sim's true `base_link` pose from a `gz OdometryPublisher` plugin
  (added to `unitree_go2_gazebo.xacro`) → gz `/model/go2/ground_truth/odometry` →
  `ground_truth.launch.py` bridges to `/ground_truth/odom`. Never touches the filter.
- Node prints one-shot diagnostics: `First /odom/raw ... received`,
  `First /gps/fix ... received`, `GPS datum set` — watch the ESKF window/log.

## 4. Gotchas learned (save future time)

- **xacro PRESERVES XML comments in its text output.** `grep`ping the xacro output
  for a commented-out `<sensor>` gives a FALSE positive. Validate with an XML parser
  and `getElementsByTagName('sensor')` (comments excluded), not grep.
- **`<xacro:include>` cannot be gated by `<xacro:if>`** — the include pass runs
  before conditionals are evaluated. Use plain comments to disable includes.
- **Installed `unitree_go2_description/urdf/` were stale COPIES**, not symlinks.
  Editing `src` had no effect until `colcon build --packages-select
  unitree_go2_description --merge-install --symlink-install` (now symlinked).
- **`go2_eskf` config/launch ARE symlink-installed** — yaml/launch edits take effect
  without rebuild; only C++ changes need `colcon build`.
- The workspace install layout is **merged** → always build with `--merge-install`
  (plain `colcon build` errors on layout mismatch).
- `ros2 topic hz` takes **one** topic at a time.
- Sim controllers spawn ~20–30 s in; nodes that need `/odom/raw` must wait for it.

## 5. Deliberate vendored edits (2)

The workspace prefers first-party changes, but three vendored edits are intentional:
1. **Ground-truth OdometryPublisher** in `unitree_go2_gazebo.xacro` (for benchmarking).
2. **Perception sensors DISABLED** — commented out in `unitree_go2_robot.xacro` (the
   velodyne / 4D-lidar / D455 includes) and `unitree_go2_gazebo.xacro` (`rgb_camera`
   block). They were the only GPU-rendered sensors → caused the render segfault (§2.1)
   and dominated load. ESKF doesn't use them. Re-enable by uncommenting + rebuild
   `unitree_go2_description`.
3. **Leg-odometry revival (two files)** — CHAMP produced zero leg-odom velocity (§0):
   (a) `publish_foot_contacts: True` in `unitree_go2_sim/launch/unitree_go2_launch.py`
   (was False, twice); (b) removed the `&& !in_gazebo_` gate in `champ_base/src/
   quadruped_controller.cpp` (lines 84, 190) so `/foot_contacts` (gait-phase estimate)
   is published in sim. Rebuild `unitree_go2_sim` and `champ_base`.

## 6. Files changed this session

- `run_go2_teleop.sh` (new), `src/go2_eskf/scripts/plot_trajectory.py` (new)
- `src/go2_eskf/config/eskf_params.yaml` — use_gps false, accel_noise 50, accel_clip 5,
  leg_odom_vel_noise 0.1, vz_zero_noise 0.3
- `src/go2_eskf/launch/eskf.launch.py` — `use_gps` arg (default false)
- `src/go2_eskf/include/go2_eskf/eskf_core.hpp` + `src/eskf_core.cpp` — `correctVerticalVel`
- `src/go2_eskf/src/eskf_node.cpp` — vz constraint call, one-shot arrival diagnostics
- `src/go2_eskf/CMakeLists.txt` — install `plot_trajectory.py`
- vendored: `unitree_go2_robot.xacro`, `unitree_go2_gazebo.xacro` (perception disabled)
- `CLAUDE.md` — GPS/navsat, NVIDIA render, perception-disabled notes

## 7. How to run / build / test

```bash
# Build (merged layout!)
colcon build --packages-select go2_eskf --merge-install --symlink-install
source install/setup.bash

# Run everything (crash-proof now; RViz off by default)
./run_go2_teleop.sh --plot            # + --lite / --rviz / --software-render

# Tests
./build/go2_eskf/test_eskf_core                       # 16 GTest
python3 src/go2_eskf/scripts/cross_validate.py         # C++≡NumPy ~1e-14

# Read before touching the filter: src/go2_eskf/docs/DESIGN.md
```
