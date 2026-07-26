# cmd_vel discrepancy — investigation and fix

**Symptom.** With `/cmd_vel.linear.x = 0.5`, the robot was stationary in Gazebo
but appeared to glide forward at constant velocity in RViz.

## Diagnosis

### What I checked, in order

1. **Bridge sanity** — confirmed the issue reproduced when publishing directly
   to Gazebo's `/cmd_vel` (bypassing `ros_gz_bridge`), so the bridge was not
   the cause.
2. **Joint state** (`/world/obstacle_course/model/diff_drive_bot/joint_state`)
   — wheels were spinning at exactly 6.25 rad/s (= 0.5 m/s ÷ 0.08 m wheel
   radius), proving the diff-drive plugin received the command and was
   actuating the joints.
3. **Gazebo `/odom`** — reported `pose.x` increasing as if the robot were
   moving (`x = 40.08` while ground-truth pose was unchanged). This is the
   "fake motion" RViz was visualizing.
4. **Ground-truth pose** (`/world/obstacle_course/dynamic_pose/info`) —
   confirmed the chassis was not translating in physics.
5. **Sim time** (`/stats`) — advancing normally with RTF ≈ 1.0, so physics
   was stepping.
6. **Direct push** — applied an external wrench to the chassis; still no
   motion. This ruled out a "wheels-don't-grip" surface-friction problem
   alone and pointed toward the chassis being unable to translate at all.
7. **Wheel orientation over time** — sampled the left-wheel quaternion while
   driving. The y and z components drifted with the **same sign**, which
   matches a rotation around world Y (rolling axis), not world Z (steering
   axis). So once contact was restored, the rolling direction was correct.
8. **Geometry math** — computed the world-z of the wheel bottom vs the caster
   bottom from the SDF, which exposed Bug 2 below.

### Root cause — two bugs in `models/diff_drive_bot/model.sdf`

**Bug 1: wheel joint axis resolved to vertical.**

The wheel link is rotated `-π/2` about X so the cylinder lies on its side.
The joint axis was `<xyz>0 1 0</xyz>`, which SDFormat 1.7+ interprets in the
**child-link frame**. After the link's `R_x(-π/2)`, child-frame `(0,1,0)`
maps to world `(0, 0, -1)` — vertical. The wheels were spinning around the
vertical axis like turntables. I first tried `expressed_in="__model__"`;
SDFormat did not honor it for this joint. The fix that worked was
`<xyz>0 0 1</xyz>` (child-frame Z, which the link rotation maps to world Y).

**Bug 2: caster lower than the wheels.**

Caster sphere at base_link z = `-0.05`, radius 0.04 → bottom at `-0.09`.
Wheel center at base_link z = 0, radius 0.08 → bottom at `-0.08`.
Caster sat 1 cm below the wheel contact. The chassis settled on the caster
(penetrating the ground), pitched ~2.5° forward, and the wheels hovered
~3 mm in the air. No ground contact, no traction. Fix: caster z `-0.05` →
`-0.03`, putting its bottom 1 cm **above** the wheel contact line.

### Why RViz "saw" the robot move

The `gz-sim-diff-drive-system` plugin synthesizes `/odom` from the
**commanded** wheel velocity, not from a ground-contact measurement. So
with `vx = 0.5` it published `/odom.twist.x = 0.5` and integrated pose
indefinitely. That `/odom` feeds two things RViz cares about:

- `/tf` (`odom → base_link`) via the diff-drive plugin's TF publisher
- `/ekf/pose` via the EKF (which fuses the lying wheel odom with IMU)

Both reported motion that the physics never delivered.

## What I changed

`src/motion_planner/models/diff_drive_bot/model.sdf`:

| Line | Before | After |
|------|--------|-------|
| caster `<pose>` | `-0.15 0 -0.05 0 0 0` | `-0.15 0 -0.03 0 0 0` |
| left_wheel_joint axis | `<xyz>0 1 0</xyz>` | `<xyz>0 0 1</xyz>` |
| right_wheel_joint axis | `<xyz>0 1 0</xyz>` | `<xyz>0 0 1</xyz>` |

(Plus inline comments explaining the geometry.)

## Verification

| Test | Result |
|------|--------|
| `gz topic /cmd_vel x=0.5` for 5 s — ground-truth Δx | **Before:** 4 µm. **After:** ≈ 2.4 m |
| Steady-state speed (Gazebo) | ≈ 0.52 m/s (commanded 0.5) |
| ROS `/cmd_vel` 0.5 m/s × 4 s — `/odom` Δx vs gz Δx | 2.91 m vs 2.41 m; matches at steady state, the ~0.5 m gap is the acceleration ramp + small slip |
| `angular.z = 1.0` for 2 s | Yaw advanced ~1.07 rad (slightly under because each `pub --once` is a momentary command) |
| Reverted axis to `0 1 0`, kept caster fix | Robot moved 0.02 m in 3 s — confirms axis fix is independently necessary |
| Full `nav_demo.launch.py` + goal at (8.5, −2.0) | Robot drove; `/ekf/pose` tracked Gazebo ground truth within ~0.3 m |

## Notes / follow-ups

- Steady-state slip during acceleration is normal (commanded wheel speed
  ramps faster than the chassis can accelerate). This is exactly the kind
  of error the EKF is meant to absorb when fused with IMU — not a bug.
- The pure-pursuit follower oscillated near goals during the integration
  test (robot circled around the start area). Independent of this fix —
  it's a controller-tuning matter (lookahead, goal tolerance).
