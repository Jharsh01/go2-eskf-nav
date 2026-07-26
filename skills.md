# skills.md — go2_eskf sim-integration knowledge & session handoff

Working log of the go2_eskf ↔ Unitree Go2 sim integration debugging. Read this
first when resuming — it captures hard-won findings that aren't obvious from the
code. (Companion to `CLAUDE.md`; this file is the narrative + current state.)

Last updated: 2026-07-26.

---

## 0. CURRENT STATE (start here)

**LATEST (2026-07-26, first REAL square numbers):** two environment/infrastructure bugs
were masking the estimator entirely — neither was in go2_eskf. Both are fixed, and the
square test now produces genuine drift measurements for the first time.

- **The robot wasn't walking — it was COLLAPSED.** `run_go2_teleop.sh` used `/odom/raw`
  as its "sim is ready" guard, but CHAMP's `state_estimation_node` publishes `/odom/raw`
  (all zeros) **~2 s** after launch, long before `controller_manager` exists. The guard
  fired instantly, so ESKF + ground-truth bridge + plot + square test all piled onto
  Gazebo's boot and starved the `controller_manager` that lives INSIDE the gz process.
  Spawners failed (`Failed to acquire lock in 20 seconds`), no controller held the legs,
  and the Go2 fell from its z=0.375 spawn to **z≈0.057** (belly on the floor, legs
  splayed). `square_test.py` then commanded 0.25 m/s at a prone robot: truth frozen at
  the origin, error flat at 1.5 cm. **This was previously misread as an ESKF bug** — the
  filter was tracking a stationary robot correctly the whole time. It also explains the
  intermittency: when the machine had spare CPU the controllers won the race and the
  robot walked (the ~6 m teleop run); when they lost it, "frozen at origin" again.
  **Fix:** `WAIT_READY` in the launcher waits for `ros2 control list_controllers` to
  report `joint_group_effort_controller ... active` (~37 s), gating both the ESKF and the
  ground-truth bridge; plus `square_test.py` now refuses to drive while base `z < 0.18`
  (`STAND_Z`) and says why. See §2.7 / §4.
- **FIRST REAL DRIFT NUMBERS** (clean run, 10 m square, GPS off, flat world):
  ```
  CORNER 1/4  truth=(+9.76,+0.00)  eskf=( +8.75, +0.02)  err=1.003 m
  CORNER 2/4  truth=(+9.98,-9.75)  eskf=(+15.08, -9.65)  err=5.096 m
  CORNER 3/4  truth=(+0.25,-9.99)  eskf=( +2.45,-12.90)  err=3.652 m
  ```
  Note the SHAPE: at corner 2 the estimate tracks **y** almost perfectly (−9.65 vs −9.75)
  while **x** overshoots by 5.1 m — the estimate keeps advancing along +x through a turn
  the filter under-registers. That is a heading/yaw-rate error, not isotropic drift, and
  it is the live lead for `gyro_z_sign` and the slip model. (Corner 4 not reached: the
  test run was capped at 200 s.)
- **Gazebo GUI was segfaulting from VRAM exhaustion** — looked like a clean shutdown,
  killed the whole sim. See §2.6. Check `nvidia-smi --query-gpu=memory.free` FIRST if the
  sim dies seconds after start.
- **Obstacles removed from the sim world** (new first-party `go2_eskf/worlds/flat.sdf`);
  `box1` sat at (5,0), directly on the square path. Nothing deleted — `--obstacles`
  selects the vendored `default.sdf`. Needed wiring the launch file's declared-but-unused
  `world` arg (§5.4).
- **Next step (unchanged):** slip model — but now on top of a baseline that actually moves.

---

**Earlier (2026-07-25 night, drift-minimization pass):** two systematic drift sources
found and fixed, plus an autonomous square test replacing manual teleop:
- **CHAMP `odom_scaler: 0.9`** (gait.yaml) scales leg-odom velocity by 0.9 — a real-robot
  slip fudge that under-reports speed 10% on sim's no-slip floor (~1 m per 10 m; matches
  the earlier 0.17-vs-0.19 leg/truth ratio). Compensated filter-side: `leg_odom_scale:
  1.111` param in eskf_node (keeps vendored gait.yaml untouched).
- **Gyro bias now strongly observable:** new `EskfCore::correctGyroBias` fuses
  `z = gyro_wz − leg_odom.angular.z` (leg yaw rate is bias-free) → `h = b_g`. Params
  `use_leg_yaw_bias: true`, `leg_yaw_bias_noise: 0.05`. Main lever against long-horizon
  yaw drift without any absolute heading. (Leg angular.z was previously unused.)
- **`square_test.py`** (new, installed): drives a 10 m square via /cmd_vel closed-loop on
  GROUND TRUTH (true path = perfect square ⇒ plot gap = pure estimator drift). Respects
  gait limits (v=0.25 ≤0.3, wz ≤0.4 ≤0.5). Prints per-corner truth/est/error + final
  summary (final/max/mean error, % of 40 m). Launcher: `./run_go2_teleop.sh --square`
  (implies plot+ground-truth, replaces teleop, widens plot view to ±13 m).
- All still verified: 16/16 GTest, C++≡NumPy 8e-15. Yaw-sign fix (`gyro_z_sign=-1`) also
  still pending live confirmation — the square run will confirm both at once.
- **Next step (user-stated):** slip model on top of this baseline.

---

**Earlier (2026-07-25 eve):** leg odometry FIXED — the estimate now moves and tracks
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

6. **Gazebo GUI segfault from VRAM exhaustion — disguised as a clean shutdown.**
   The whole sim died ~3 s after launch; `ros2 launch` printed
   `[gazebo-1]: process has finished cleanly`, so it looked like a normal stop.
   - **Chain:** the GUI's Ogre2 render target allocation fails when VRAM is exhausted,
     Ogre doesn't check it, and it null-derefs in
     `Ogre::ForwardClustered::collectLightForSlice` (via `GzRenderer::Render` in
     `libMinimalScene.so`) on the FIRST rendered frame. The `gz` ruby wrapper then
     SIGINTs the *server*, which exits 0 — hence "finished cleanly". `controller_manager`
     dies with gz, so spawners hang on `/controller_manager/list_controllers` and both
     `robot_localization` ekf_nodes abort with `exit code -6`.
   - **Cause here:** a `carla-server` docker container (`carlasim/carla:0.9.15`,
     restart=unless-stopped, RPC 2000) holding **2890 MiB of the 4096 MiB** RTX 3050,
     leaving **94 MiB free**. Not a code regression: no package changed, and the NVIDIA
     580.173.02 update predates the working runs.
   - **Fix:** `docker stop carla-server` → free VRAM 94 MB → 3120 MB, GPU 100% → 1%.
     Restart with `docker start carla-server`. The two cannot coexist on 4 GB.
   - **NOT the fix:** `--software-render` / `LIBGL_ALWAYS_SOFTWARE=1` crashed too. This is
     also NOT the Optimus *offscreen sensor* segfault of §2.1 — that's a different thread.

7. **Robot collapsed on its belly → "estimate frozen at origin" (the big one).**
   Full chain in §0. Root cause: the launcher's readiness guard waited on `/odom/raw`,
   which CHAMP publishes (zeros) ~2 s after launch — useless as a signal. Everything
   started during Gazebo's boot, starved `controller_manager`, spawners failed
   (`Failed to acquire lock in 20 seconds`), legs went limp, base fell to z≈0.057.
   - **Fix (launcher):** `WAIT_READY` waits for
     `ros2 control list_controllers | grep 'joint_group_effort_controller.*active'`
     (~37 s on this machine) before starting the ESKF or the ground-truth bridge.
   - **Fix (test):** `square_test.py` holds `/cmd_vel` at zero while base `z < STAND_Z`
     (0.18 m) and warns once with the exact diagnostic command, so a prone robot can
     never again masquerade as estimator drift. It also prints a partial drift summary
     on Ctrl-C/SIGTERM instead of an `RCLError` traceback.
   - **Diagnosis recipe:** if the estimate looks frozen, check the ROBOT first —
     `ros2 topic echo /ground_truth/odom --once --field pose.pose.position`. `z≈0.22`
     = standing, `z≈0.06` = collapsed. Then `ros2 control list_controllers`.

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
- **`/odom/raw` existing does NOT mean the sim is ready.** CHAMP's `state_estimation_node`
  publishes it (all zeros) ~2 s after launch, before `controller_manager` exists. The only
  honest readiness signal is `ros2 control list_controllers` reporting
  `joint_group_effort_controller ... active` (~37 s). Waiting on the topic instead lets
  every other node pile onto Gazebo's boot and starve the controller spawners (§2.7).
- **A collapsed robot looks exactly like a broken estimator.** With no active leg
  controller the Go2 lies on its belly at base `z≈0.057` (standing ≈0.22, spawn 0.375) and
  ignores `/cmd_vel`. Always check base `z` before blaming the filter.
- **`[gazebo-1]: process has finished cleanly` can be a GUI CRASH.** The `gz` wrapper
  SIGINTs the server when the GUI dies, and the server exits 0. The real evidence is in
  `~/.gz/sim/log/<ts>/server_console.log`: `Received signal[2]` only *seconds* after
  start = something killed it. `gz sim -s` (server) surviving while `gz sim -g` exits 139
  isolates it to the GUI. Also: `nvidia-smi --query-gpu=memory.free` (§2.6).
- Each gz run logs its world: `grep -ao "Loading SDF world file\[[^]]*\]"
  ~/.gz/sim/log/<ts>/server_console.log` — settles "which world did that run use?".
- **Killing a `ros2 launch` with SIGKILL ORPHANS its children** (gz, champ nodes, bridges),
  which then fight the next run over `/world/default/...` topics and produce nonsense
  (two robots answering one `/ground_truth/odom`, GUI rendering garbage). SIGINT the
  launch parent and let it reap. Beware: no process is literally named `gz-sim-server` —
  `pkill -f gz-sim-server` matches NOTHING. The real names are `gz sim`, `gz sim server`,
  `gz sim gui`.

## 5. Deliberate vendored edits (4)

The workspace prefers first-party changes, but four vendored edits are intentional:
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
4. **`world` launch arg actually honoured** — `unitree_go2_sim/launch/unitree_go2_launch.py`
   DECLARED a `world` argument but hardcoded `worlds/default.sdf` in `gz_args`, so
   `world:=...` was silently ignored (same dead-arg pattern as `gui`, still unused). Now
   passes `LaunchConfiguration('world')`; the default is unchanged. This is what lets the
   obstacle-free world live in first-party `go2_eskf` instead of editing the vendored SDF.

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

**2026-07-26:**
- `src/go2_eskf/worlds/flat.sdf` (new) — obstacle-free world; `CMakeLists.txt` installs
  `worlds/`
- `run_go2_teleop.sh` — `WAIT_READY` controller-active guard (replaces the useless
  `/odom/raw` wait) on the ESKF + ground-truth bridge; `--obstacles` flag; world selection
  + `World:` line in the banner
- `src/go2_eskf/scripts/square_test.py` — `STAND_Z` refuse-to-drive-while-prone check,
  partial drift summary on interrupt, `ExternalShutdownException` handling
- vendored: `unitree_go2_sim/launch/unitree_go2_launch.py` — honour the `world` arg (§5.4)

## 7. How to run / build / test

```bash
# Build (merged layout!)
colcon build --packages-select go2_eskf --merge-install --symlink-install
source install/setup.bash

# Run everything (RViz off by default; obstacle-free world by default)
./run_go2_teleop.sh --plot            # + --lite / --rviz / --software-render
./run_go2_teleop.sh --square          # autonomous 10 m square drift test
./run_go2_teleop.sh --square --obstacles   # ...with the boxes/cylinders back

# BEFORE running the sim: free the GPU (4 GB card — CARLA and Gazebo can't share it)
nvidia-smi --query-gpu=memory.free --format=csv   # <500 MB free => GUI will segfault
docker stop carla-server                          # docker start carla-server to restore

# If the estimate looks frozen, check the ROBOT before the filter:
ros2 topic echo /ground_truth/odom --once --field pose.pose.position  # z~0.22 ok, ~0.06 collapsed
ros2 control list_controllers          # joint_group_effort_controller must be 'active'

# Tests
./build/go2_eskf/test_eskf_core                       # 16 GTest
python3 src/go2_eskf/scripts/cross_validate.py         # C++≡NumPy ~1e-14

# Read before touching the filter: src/go2_eskf/docs/DESIGN.md
```
