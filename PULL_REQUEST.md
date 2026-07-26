# PR: Make `go2_eskf` track the Go2 in the live Gazebo sim

## Summary

The `go2_eskf` error-state EKF ran cleanly against replay/NumPy tests but did **not
work against the live CHAMP Gazebo sim** — the estimate blew up to tens of km, then
(once that was fixed) sat frozen at the origin. This PR takes it end-to-end from
"diverges to 55 km" to "tracks the robot," fixing five independent root causes
across the estimator, its config, and the vendored sim, plus adds tooling
(one-command launcher + live estimate-vs-truth plot) to see it working.

Along the way it also fixes a recurring Gazebo **segfault** on NVIDIA-Optimus
laptops and cuts sim load.

**Status:** estimate now moves and tracks distance; a final yaw-sign fix is in and
under verification.

---

## Changes by theme

### 1. Estimator correctness & tuning (`go2_eskf`)

| File | Change | Why |
|------|--------|-----|
| `src/eskf_core.{hpp,cpp}` | Add `correctVerticalVel(vz_meas, r)` (1-D update) + `josephUpdate<1>` instantiation | `pz`/`vz` were **unobservable** (leg odom is planar, GPS off) → Z double-integrated to **1601 m**. A `vz≈0` pseudo-measurement (ground robot doesn't climb) bounds the vertical channel. |
| `src/eskf_node.cpp` | Call `correctVerticalVel(0, vz_zero_noise²)` on each leg-odom update | Applies the vertical anchor whenever the robot is walking. |
| `src/eskf_node.cpp` | `gyro_z_sign` param applied to `gyro_wz_` | The sim IMU is ~180° flipped (DESIGN §6.1): `gravity_lp` cancels the flipped accel, but the **yaw rate was used raw and came out negated** → estimate turned the wrong way. |
| `src/eskf_node.cpp` | `RCLCPP_INFO_ONCE` on first `/odom/raw` and `/gps/fix`; warn on GPS `NO_FIX` | Runtime visibility into which corrections actually reach the filter — this is what localized the leg-odom-dead and GPS-noise bugs. |
| `include/.../eskf_node.hpp` | New members `vz_zero_noise_`, `gyro_z_sign_` | Backing storage for the two new params. |
| `config/eskf_params.yaml` | See tuning table below | Make leg odometry dominate, disable unusable GPS, add the new params. |
| `launch/eskf.launch.py` | Add `use_gps` launch arg (default **false**) | Was only settable via yaml; now `use_gps:=true/false` works from the CLI (typed bool). |
| `CMakeLists.txt` | Install `scripts/plot_trajectory.py` | Ships the new visual-aid as a `ros2 run` executable. |

**Tuning (`eskf_params.yaml`):**

| Param | Old | New | Reason |
|-------|-----|-----|--------|
| `use_gps` | `true` | `false` | Sim navsat emits ~0.5°(~55 km) noise (stddev applied in degrees) — fusing it wrecked the estimate. |
| `accel_noise` | `1.0` | `50.0` | At 1.0 the leg-odom Kalman gain was ~0.06 → filter ignored leg odom and froze. Big value ⇒ leg odom dominates velocity (sim IMU spikes to 215 m/s²). |
| `accel_clip` | `40.0` | `5.0` | Reject contact-impact spikes at the source; a walking base's true CoM accel is only a few m/s². |
| `leg_odom_vel_noise` | `0.2` | `0.1` | Leg odom tracks truth to ~10%; trust it more. |
| `vz_zero_noise` | — | `0.3` | New: std-dev of the `vz≈0` vertical pseudo-measurement (allows gait bob). |
| `gyro_z_sign` | — | `-1.0` | New: un-flips the negated yaw rate from the 180°-mounted sim IMU. |

### 2. Vendored sim fixes (deliberate, documented)

| File | Change | Why |
|------|--------|-----|
| `champ_base/src/quadruped_controller.cpp` | Remove `&& !in_gazebo_` from the two `publish_foot_contacts_` guards (lines 84, 190) | **The core "frozen estimate" bug.** The controller refused to publish `/foot_contacts` in gazebo mode, so `state_estimation`'s synchronized callback never fired and leg-odom velocity stayed 0. It now publishes the gait-phase stance estimate (the code's own documented fallback). |
| `unitree_go2_sim/launch/unitree_go2_launch.py` | `publish_foot_contacts: True` (both occurrences) | Enables the above so leg odometry produces velocity. |
| `unitree_go2_description/urdf/unitree_go2_robot.xacro` | Comment out velodyne / 4D-lidar / D455 includes | These GPU-rendered sensors caused a Gazebo Ogre2 offscreen-render **segfault** on NVIDIA-Optimus laptops (killing gz + controller_manager) and dominated load. The ESKF doesn't use them. |
| `unitree_go2_description/urdf/unitree_go2_gazebo.xacro` | Comment out the `rgb_camera` sensor block | Same as above (4th GPU-rendered sensor). |

> Note: `<xacro:include>` can't be gated by `<xacro:if>` (the include pass precedes
> conditionals), so plain comments are used. Re-enable by uncommenting.

### 3. New tooling (first-party)

| File | What |
|------|------|
| `run_go2_teleop.sh` (new, 200 LoC) | One-command launcher: sim + keyboard teleop + ESKF (+ ground-truth bridge + live plot with `--plot`). Each component runs in its own process group (`set -m`); a single **Ctrl-C tears everything down**. Flags: `--rviz` (off by default — heavy), `--plot`, `--lite`, `--software-render`. Forces NVIDIA offscreen-render env; nices non-critical nodes; staggers startup so nothing piles onto Gazebo's boot. |
| `src/go2_eskf/scripts/plot_trajectory.py` (new, 274 LoC) | Live matplotlib: top-down XY of `/eskf/odom` vs `/ground_truth/odom` (clamped view + out-of-view banner) and a position-error-vs-time panel. Runs its own ROS spin thread. `ros2 run go2_eskf plot_trajectory.py`. |

### 4. Docs

| File | What |
|------|------|
| `CLAUDE.md` | Corrected GPS guidance (off + why), NVIDIA render gotcha, perception-sensors-disabled note, `run_go2_teleop.sh` usage. |
| `skills.md` (new) | Full session handoff: current state, all bugs/fixes, gotchas, files changed, run/build/test. |
| `PULL_REQUEST.md` (this file) | This report. |

---

## Root causes fixed (in the order found)

1. **Gazebo sensor-render segfault** (NVIDIA Optimus) → force NVIDIA EGL/GLX offscreen;
   then permanently disable the GPU-rendered sensors.
2. **Divergence to ~55 km with GPS on** → sim navsat noise is ~0.5°/~55 km (stddev in
   degrees, not metres); `use_gps` OFF by default.
3. **Z position exploding to 1601 m** → vertical channel unobservable; add `vz≈0`
   pseudo-measurement.
4. **Estimate frozen at origin** → (a) `accel_noise` too small so leg odom was ignored;
   (b) the real blocker: CHAMP never published `/foot_contacts` in sim → leg-odom
   velocity was literally 0.
5. **Heading mirrored** → sim IMU 180°-flipped ⇒ negated yaw rate; `gyro_z_sign=-1`.

---

## Testing

- `./build/go2_eskf/test_eskf_core` — **16/16 GTest pass** after the core change.
- `python3 src/go2_eskf/scripts/cross_validate.py` — **C++ ≡ NumPy to ~8e-15**
  (the `vz` addition is additive; parity preserved).
- `xacro` on the model generates valid URDF with only `imu` + `navsat` live sensors
  (validated via XML element parse, not grep — xacro preserves comments in output).
- End-to-end launcher teardown verified: 6 procs → 0 on one Ctrl-C, no orphans.
- Live sim: estimate went from 55 km error → frozen → **tracking distance** (~6 m
  matched truth); yaw-sign fix under verification.

## How to run

```bash
colcon build --merge-install --symlink-install   # merged layout is required
source install/setup.bash
./run_go2_teleop.sh --plot                        # + --lite / --rviz / --software-render
```

## Build/deploy notes (gotchas that bit us)

- Workspace install layout is **merged** → always `--merge-install`.
- `go2_eskf` config/launch are **symlink-installed** (yaml/launch edits are live; only
  C++ needs a rebuild). The **vendored** `unitree_go2_*` packages had **stale copies**
  installed — they must be rebuilt with `--symlink-install` for src edits to take effect
  (`unitree_go2_description`, `unitree_go2_sim`, `champ_base` were rebuilt here).

## Files changed

```
 new    run_go2_teleop.sh
 new    skills.md
 new    PULL_REQUEST.md
 new    src/go2_eskf/scripts/plot_trajectory.py
 edit   src/go2_eskf/config/eskf_params.yaml
 edit   src/go2_eskf/launch/eskf.launch.py
 edit   src/go2_eskf/include/go2_eskf/eskf_core.hpp
 edit   src/go2_eskf/src/eskf_core.cpp
 edit   src/go2_eskf/include/go2_eskf/eskf_node.hpp
 edit   src/go2_eskf/src/eskf_node.cpp
 edit   src/go2_eskf/CMakeLists.txt
 edit   CLAUDE.md
 edit   src/unitree_go2_ros2/champ_base/src/quadruped_controller.cpp        (vendored)
 edit   src/unitree_go2_ros2/unitree_go2_sim/launch/unitree_go2_launch.py    (vendored)
 edit   src/unitree_go2_ros2/unitree_go2_description/urdf/unitree_go2_robot.xacro  (vendored)
 edit   src/unitree_go2_ros2/unitree_go2_description/urdf/unitree_go2_gazebo.xacro (vendored)
```
