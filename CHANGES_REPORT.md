# Changes report — debugging the navigation pipeline

Two distinct problems were investigated and fixed in this session. Each is
explained below with the evidence that pinned it down and the file diff that
fixed it. End-to-end test at the bottom shows the robot driving from start
to goal.

---

## Problem 1 — `/cmd_vel` did not move the robot in Gazebo

**Symptom.** Publishing `/cmd_vel.linear.x = 0.5` left the chassis stationary
in physics, while RViz displayed it gliding forward at constant velocity.

### Root cause — two SDF bugs in `models/diff_drive_bot/model.sdf`

1. **Wheel joint axis resolved to vertical.** The wheel link is rotated
   `−π/2` about X (cylinder lying on its side). The joint axis was
   `<xyz>0 1 0</xyz>`, which SDFormat 1.7+ interprets in the **child-link
   frame**. After the link's `R_x(−π/2)`, child-frame `(0,1,0)` maps to
   world `(0, 0, −1)` — vertical. Wheels spun around the vertical axis like
   turntables.
2. **Caster sat lower than the wheels.** Caster pose z = `−0.05`, radius
   `0.04` ⇒ bottom at `−0.09`. Wheel center at z = 0, radius `0.08` ⇒
   bottom at `−0.08`. Caster was 1 cm below the wheel contact line, so the
   chassis pitched ~2.5° forward, rested on the caster (penetrating ground
   by ~7 mm), and the wheels hovered ~3 mm above ground. **No traction.**

### Why RViz "saw" motion

The `gz-sim-diff-drive-system` plugin computes `/odom` from the **commanded**
wheel velocity, not from physics. So with vx = 0.5 it reported
`/odom.twist.x = 0.5` and integrated pose forever. That fed:
- `/tf` (`odom → base_link`) via the diff-drive plugin's TF publisher → RViz
- `/ekf/pose` via the EKF (which fused the lying odom with IMU)

### File changed: `src/motion_planner/models/diff_drive_bot/model.sdf`

| Element | Before | After |
|---|---|---|
| caster `<pose>` | `-0.15 0 -0.05 0 0 0` | `-0.15 0 -0.03 0 0 0` |
| left_wheel_joint axis | `<xyz>0 1 0</xyz>` | `<xyz>0 0 1</xyz>` |
| right_wheel_joint axis | `<xyz>0 1 0</xyz>` | `<xyz>0 0 1</xyz>` |

Comments were added inline explaining the geometry rationale.

### Verification

| Test | Before | After |
|---|---|---|
| `gz topic /cmd_vel x=0.5` for 5 s, ground-truth Δx | 4 µm | ≈ 2.4 m |
| Steady-state forward speed (Gazebo) | — | ≈ 0.52 m/s |
| ROS `/cmd_vel` 0.5 m/s × 4 s, `/odom` Δx vs gz Δx | — | 2.91 m vs 2.41 m (matches at steady state; gap is the acceleration ramp) |
| Reverted axis to `0 1 0` keeping the caster fix | Δx = 0.02 m in 3 s | — (confirms the axis fix is independently necessary) |

---

## Problem 2 — Robot circles around the start of the path instead of reaching the goal

**Symptom.** With a goal published, the robot spun in place and made tight
forward bursts in a small region near the path start, never approaching the
goal.

### Root cause — Pure Pursuit search anchored at index 0

`src/motion_planner/src/pure_pursuit.cpp:26-33` searched the path from
`i = 0` every tick:
```cpp
size_t la_idx = 0;
for (size_t i = 0; i < path.size(); ++i) {
  if (robot.point().distanceTo(path[i].point()) >= cfg_.lookahead_m) {
    la_idx = i;
    break;          // first hit wins
  }
  la_idx = i;
}
```
Once the robot moved more than `lookahead_m` (= 0.5 m) past `path[0]`, the
very first iteration satisfied the `>=` test and `path[0]` was picked as
the lookahead target — i.e. the controller permanently asked the robot to
return to the path's starting point. Combined with the "lookahead behind
me" guard
```cpp
if (x_r < 0.0) v = 0.0;
...
if (v_out < 1e-3)
  omega_out = std::copysign(min(max_omega, 0.8), y_r);
```
the robot rotated in place until the start point was in front, sprinted
toward it, overshot, and repeated. That is the circling.

### Evidence

- Path[0] = `(7.75, −5.45)`; robot oscillating in `(8.2…9.1, −8.3…−9.0)`
  (~3.3 m south of path[0]); goal `(8.5, −2.0)` never approached.
- 8 s of `/cmd_vel`, only ~6 distinct values; dominant pairs:
  `(0.0, 0.8)` (rotate-in-place CCW, 93 samples) and `(0.6, 1.5)` (full fwd
  + saturated turn, 35 samples). No "drive straight" samples even though
  the goal was almost due north.

### Why it is not a tuning issue

Increasing `lookahead_m` only delays the trap (you advance further before
`path[0]` falls into range). Decreasing it brings the trap on sooner. The
problem is the search shape, not its parameter.

### File changed: `src/motion_planner/src/pure_pursuit.cpp`

Added `<limits>` header, replaced the lookahead block with a
**closest-waypoint anchor + forward walk**:

```cpp
// Step 1: closest waypoint to the robot
size_t closest_idx = 0;
double closest_d2  = std::numeric_limits<double>::infinity();
for (size_t i = 0; i < path.size(); ++i) {
  const double dx_i = path[i].x - robot.x;
  const double dy_i = path[i].y - robot.y;
  const double d2   = dx_i * dx_i + dy_i * dy_i;
  if (d2 < closest_d2) { closest_d2 = d2; closest_idx = i; }
}
// Step 2: walk forward from there until distance >= lookahead_m
size_t la_idx = closest_idx;
for (size_t i = closest_idx; i < path.size(); ++i) {
  if (robot.point().distanceTo(path[i].point()) >= cfg_.lookahead_m) {
    la_idx = i;
    break;
  }
  la_idx = i;
}
```

This makes progress monotonic: waypoints behind the robot are skipped, and
the lookahead always lies on the unvisited part of the path.

### Verification

Goal at `(8.5, −2.0)` from start `(7.0, −7.0)`:

| t (s) | gz x | gz y |
|---:|---:|---:|
| 0 | 6.998 | −7.000 |
| 1 | 7.468 | −6.471 |
| 3 | 7.493 | −5.155 |
| 5 | 7.855 | −3.913 |
| 7 | 8.595 | −2.835 |
| 9 | 8.674 | −2.206 |
| 10 | 8.678 | −2.181 — `Goal reached.` logged |
| 11+ | 8.678 | −2.181 (stopped) |

A* path length 5.62 m, planned in 0.5 ms; traversal in ≈ 10 s at the
configured 0.6 m/s max ⇒ behavior matches expectation.

---

## Summary of files changed

| File | Edit |
|---|---|
| `src/motion_planner/models/diff_drive_bot/model.sdf` | caster z `-0.05 → -0.03`; wheel-joint axes `0 1 0 → 0 0 1` (×2); explanatory comments |
| `src/motion_planner/src/pure_pursuit.cpp` | added `#include <limits>`; replaced lookahead search with closest-waypoint anchor + forward walk |

Reports produced this session:
- `CMD_VEL_FIX_REPORT.md` — detail on Problem 1
- `CHANGES_REPORT.md` — this file (covers Problems 1 and 2)

## Notes / known follow-ups

- Steady-state slip during acceleration is normal and absorbed by the EKF.
  Not a bug.
- `launch_rviz` arg in `nav_demo.launch.py:93` is a no-op
  (`condition=None if launch_rviz else None` — both branches return
  `None`). RViz launches regardless. Cosmetic.
- No replanning loop — if a goal is sent while the robot is mid-track, the
  navigation node replans on every new `/goal_pose`, but it does not detect
  drift or path infeasibility on its own.
