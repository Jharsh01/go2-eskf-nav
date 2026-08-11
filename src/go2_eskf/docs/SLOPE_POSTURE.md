# Slope posture — theory and the numbers for this robot

Why a blind quadruped needs to change its **body height, position and attitude** on a
slope, what the right values are, and how the two knobs in `scripts/terrain_adapt.py`
(`com_shift_x`, `crouch`, `level_pitch`) follow from the geometry.

Companion to `DESIGN.md` (the estimator). This document is about **locomotion**, and it
exists because CHAMP does none of this: `quadruped_controller.cpp` sets
`req_pose_.position.z = nominal_height` once at startup, never touches orientation, and
`req_pose_` moves only when something publishes the `/body_pose` topic — which nothing in
the vendored launch does. So stock CHAMP holds all four feet on one plane in the base
frame forever, and that is why it stalls on a sustained climb (`skills.md` §0,
"Terrain BLOCKER").

Every number below is for **this** robot, from `const.xacro` and `gait.yaml`:

| symbol | meaning | value |
|--------|---------|-------|
| `m` | total mass (6.921 trunk + 4 × 2.044 leg) | 15.10 kg → `W` = 148.1 N |
| `a` | hip offset along body x (half wheelbase) | 0.1934 m |
| `L` | hip-to-hip, front to rear | 0.3762 m |
| `h` | `nominal_height`, hip plane → foot plane | 0.225 m |
| `l` | thigh = calf link length | 0.213 m |
| `Λ` | step length = `(stance_duration/2)·v·2` at v=0.25 | 0.0625 m |
| `γ` | ground slope angle | — |
| `β_w` | body pitch **in the world** | — |
| `β_r` | body pitch **relative to the ground** = `β_w − γ` | — |

---

## 1. Frames, and what "height" even means on a slope

`nominal_height` is measured **along the body's −z axis**, not vertically. So the moment
the body tilts, "height" splits into three different quantities that are equal only on
flat ground:

- **leg extension** `ℓ` — hip to foot, what the IK actually commands;
- **perpendicular clearance** `h_c` — body plane to ground plane, along the ground normal;
- **vertical clearance** — body to ground along gravity, `= h_c/cos γ` … or `h_c·cos γ`
  depending on which you tilt.

If the body stays **parallel to the ground** (`β_r = 0`), then `ℓ = h_c = h` for all four
legs and only the vertical projection changes — by `h(1 − cos γ)`, which is **4.4 mm at
11.3°**. Negligible. This is the key result of the section: *keeping the body parallel to
the ground makes the height problem disappear.* Stock CHAMP gets this for free, because
its foot plane is body-parallel and the body settles onto the terrain by force balance.

The height problem only appears when you deliberately break `β_r = 0` — see §4.

---

## 2. Leg-extension asymmetry: the cost of levelling the body

Put the body at pitch `β_r` relative to the ground. A hip at body-x offset `u` then sits a
perpendicular distance

```
    ℓ(u) = h_c + u · sin(β_r)
```

above the ground, so front and rear legs must differ by `2a·sin(β_r)`:

| γ (body held level in world, `β_r = −γ`) | front leg | rear leg | spread |
|---|---|---|---|
| 5.0° | 0.208 m | 0.242 m | ±1.69 cm |
| 9.6° | 0.193 m | 0.257 m | ±3.23 cm |
| **11.3°** | **0.187 m** | **0.263 m** | **±3.79 cm** |
| 16.6° | 0.170 m | 0.280 m | ±5.53 cm |

Is that reachable? Yes, easily — two 0.213 m links reach 0.426 m, and `BodyController`
clamps translation at `0.65 · zero_stance`. Kinematics is **not** the limit.

What about torque? For a symmetric two-link leg with foot at distance `ℓ` from the hip, the
half-angle is `α = acos(ℓ/2l)` and the knee torque for an axial force `F` is `τ = F·l·sin α`:

| leg extension | half-angle α | knee τ at F = 74 N | % of `calf_torque_max` (35.55) | sag at p=100 | sag at p=300 |
|---|---|---|---|---|---|
| 0.187 m (front, levelled) | 64.0° | 14.17 N·m | 39.9 % | 3.02 cm | 1.01 cm |
| 0.205 m (crouched) | 61.2° | 13.82 N·m | 38.9 % | 2.94 cm | 0.98 cm |
| **0.225 m (nominal)** | 58.1° | **13.39 N·m** | 37.7 % | **2.85 cm** | **0.95 cm** |
| 0.263 m (rear, levelled) | 51.9° | 12.40 N·m | 34.9 % | 2.64 cm | 0.88 cm |

Two conclusions that matter:

1. **Torque barely varies with extension** — 12.4 to 14.2 N·m across the whole usable
   range, ±7 %. So crouching is cheap and levelling is cheap, *in torque terms*.
2. **The sag column is the real story.** `joint_group_effort_controller` is a
   JointTrajectoryController on the **effort** interface: commanded torque is a PID on
   joint *position* error, so a leg only pushes as hard as `p ×` its own tracking error.
   At the stock `p = 100 N·m/rad`, holding 74 N needs 0.134 rad of error ⇒ **2.85 cm of
   foot droop, 12.7 % of `nominal_height`, given away before the leg pushes at all.**
   That droop is stance stroke the climb never receives. It is why `ros_control_stiff.yaml`
   exists (p = 300 ⇒ 0.95 cm), and it is a *controller* problem, not an actuator one — the
   calf has 2.4× torque headroom and gz logs `Enforcing command limits is disabled`.

---

## 3. The real slope problem: the CoM projection, not the height

Gravity is vertical; the ground normal is tilted by `γ`. The gravity line through the CoM,
at perpendicular height `h_c` above the ground, therefore pierces the ground plane at a
point displaced **downhill** by

```
    Δ = h_c · tan γ
```

That is the whole story of slope posture in one term. With `h_c ≈ h = 0.225`:

| γ | Δ (downhill) | as % of half-base `a` | downhill static margin `a − Δ` |
|---|---|---|---|
| 5.0° | 1.97 cm | 10.2 % | 17.4 cm (90 %) |
| 9.6° (new path max) | 3.81 cm | 19.7 % | 15.5 cm (80 %) |
| **11.3° (new terrain cap)** | **4.50 cm** | **23.2 %** | **14.8 cm (77 %)** |
| 16.6° (old path max) | 6.71 cm | 34.7 % | 12.6 cm (65 %) |

Outright tip-over needs `Δ = a`, i.e. `γ = atan(a/h) = 40.7°` — nowhere near. **The robot
does not fall over on these slopes; it loses traction and stroke.** The mechanism is load
transfer. For a four-foot stance the normal force splits as `(a ∓ Δ)/2a`:

| γ | front pair | rear pair | rear:front |
|---|---|---|---|
| 0° | 50.0 % (74.0 N) | 50.0 % (74.0 N) | 1.00 : 1 |
| 9.6° | 40.2 % (58.6 N) | 59.8 % (87.4 N) | 1.49 : 1 |
| **11.3°** | **38.4 % (55.7 N)** | **61.6 % (89.5 N)** | **1.61 : 1** |
| 16.6° | 32.7 % (46.4 N) | 67.3 % (95.6 N) | 2.06 : 1 |

Both ends suffer, differently: the **front** feet lose 23 % of their normal force and with
it 23 % of their available friction (`F_friction ≤ μN`), so they are the first to slip;
the **rear** feet carry 61 % of the weight, so they sag most and their swing must clear the
most while working hardest. In a trot only a diagonal pair is in stance, so this is a
*time-averaged load* argument, not a tip-over polygon — but load transfer is exactly what
degrades a blind trot into stepping in place.

### 3.1 `com_shift_x` — the correction, and its exact value

Shifting the body uphill by `d` moves the CoM's gravity projection back toward the
geometric centre, so choose `d = Δ`. `terrain_adapt.py` commands

```
    position.x = com_shift_x · sin(γ)          (+x = body forward = uphill on a climb)
```

Setting that equal to `h·tan γ` gives the **ideal gain**

```
    com_shift_x_ideal = h / cos γ  ≈  h  =  0.225
```

— a pleasingly clean result: *the ideal CoM-shift gain is just the body height.*

| gain | shift at 11.3° | compensation | front-pair load restored to |
|---|---|---|---|
| 0 (stock) | 0 cm | 0 % | 38.4 % |
| **0.15 (default)** | **2.94 cm** | **65 %** | **46.0 %** |
| 0.225 (ideal) | 4.41 cm | 98 % | 49.8 % |

The default is deliberately **2/3 of ideal**. The CoM is not exactly at the body centre nor
exactly at height `h`, the estimate lags (§5), and over-shifting during a trot's flight
phase throws load onto a front pair that is about to swing. Two-thirds recovers most of the
load transfer with margin for those errors. If a climb still fails with `--adapt`, raising
`com_shift_x` toward 0.225 is the first thing to try, not the last.

### 3.2 `crouch` — what it does and does not buy

Lowering `h_c` by `δ` reduces `Δ` by `δ·tan γ`. At 11.3° a 2 cm crouch buys back **4 mm**
of the 4.5 cm shift — under 10 %. **Crouching is not a fix for the CoM projection**; if
that is what you want, use `com_shift_x`, which addresses it directly and at 10× the
leverage.

What crouching actually buys is *dynamic*: the overturning moment from a lateral
disturbance scales with CoM height, so a lower body rejects the roll/pitch kicks that a
blind trot takes on uneven ground. It costs ~3 % more knee torque (§2) and, importantly,
spends leg workspace that terrain irregularity needs. Hence the modest default
(`crouch = 0.10` ⇒ 2.0 cm at 11.3°) and the `max_crouch = 0.06` clamp.

---

## 4. `level_pitch` — and why gain 1.0 only levels halfway

`BodyController::poseCommand` applies `RotateY(-req_pose.orientation.pitch)` to the foot
targets, so commanding pitch `β_c` makes the body sit at `β_r = β_c` **relative to the foot
plane** (≈ the ground). `terrain_adapt.py` commands `β_c = −k·β_w` from the *measured world
pitch*, which makes this a feedback loop, not a feedforward one:

```
    β_w = γ + β_r = γ − k·β_w        ⇒        β_w = γ / (1 + k)
```

| k (`level_pitch`) | body keeps | at γ = 11.3°: body in world | body vs ground |
|---|---|---|---|
| **0 (default)** | **100 %** | **11.30°** | **0.00°** |
| 0.5 | 66.7 % | 7.53° | −3.77° |
| 1.0 | 50.0 % | 5.65° | −5.65° |
| 2.0 | 33.3 % | 3.77° | −7.53° |
| 4.0 | 20.0 % | 2.26° | −9.04° |

So **`level_pitch = 1.0` does not level the body — it halves the pitch.** True levelling
needs `k → ∞` or an integrator, neither of which is wise here: the loop's DC gain is
`k/(1+k) < 1` so it is unconditionally stable in this algebra, but the real loop also
contains the 0.7 s attitude low-pass and the body's own pendulum dynamics, and high `k`
will ring. **Keep `k ≤ 1`.**

Should you level at all? §1 and §2 argue mostly **no** — `β_r = 0` equalises leg extension,
equalises torque, keeps all four legs in the same part of their workspace, and keeps the
swing trajectory (planned in the body plane) perpendicular to the *ground*, which is what
"clearance" is supposed to mean. Tilting the foot plane by `β_r` steals
`(Λ/2)·sin|β_r| = 0.6 cm` of effective clearance at touchdown for 11.3° of levelling.

The case *for* levelling is entirely about what is bolted to the trunk — a levelled body
keeps cameras and LiDAR horizontal and keeps the IMU near its calibrated attitude. Since
perception is disabled in this workspace, the default is 0. It is exposed because the
question is worth measuring, not because it is expected to help walking.

---

## 5. Estimating the slope: why the accelerometer, and what it costs

Roll/pitch cannot come from the sources you would expect: the gz IMU's **orientation**
field is unreliable (`DESIGN.md` — it is why the ESKF defaults to `gravity_lp`), and the
ESKF's own state is 8-dimensional, carrying **yaw only**. So attitude is recovered from
gravity, as everywhere else in this stack. At rest an accelerometer reads `+g` expressed in
the body frame, so a nose-up pitch `θ` gives `f = (−g sin θ, 0, g cos θ)`, hence

```
    pitch = atan2(−f_x, ‖(f_y, f_z)‖)          roll = atan2(f_y, f_z)
```

Three consequences the implementation has to respect:

1. **This measures the BODY's attitude, not the GROUND's slope.** They coincide only when
   `β_r = 0` — i.e. at `level_pitch = 0`, the default. Turn levelling on and the two
   diverge by exactly the amount §4 computes, which is why the `com_shift_x` term (which
   really wants `γ`) and the `level_pitch` term (which really wants `β_w`) should not both
   be pushed hard at once.
2. **Contact impacts carry no attitude information.** A trot's touchdown spikes the
   accelerometer far past `g` (the same spikes that force a huge `accel_noise` in the
   ESKF). They are rejected by magnitude — `|f|` outside `(1 ± 0.35)g` is dropped rather
   than averaged in — before the low-pass sees them.
3. **Lag is the price.** τ = 0.7 s at 0.25 m/s is **18 cm** of travel. Against terrain
   whose slope changes over metres (the current world's steepest ramp runs ~1.5 m) the
   posture trails by roughly a tenth of the feature, which is fine. Against a step change —
   a curb, a stair — it is useless, and no amount of tuning fixes that, because the robot
   is blind: this is a *reactive* controller with no lookahead.

---

## 6. What was never the problem

Worth stating explicitly, because both were tried or assumed first:

- **Swing height.** Per-step rise is `Λ·tan γ`: **1.25 cm at 11.3°**, 1.06 cm at 9.6°,
  0.76 cm at 6.9° — against a swing apex of 8 cm (and 4 cm before it was doubled). A 6×
  margin at the *old* setting. Clearance was never the binding constraint on a slope; it
  is a step/obstacle concern, not a gradient one.
- **Ground friction.** Standing needs `μ ≥ tan γ` = 0.20 at 11.3°; walking needs roughly
  `μ ≥ 1.5·tan γ` = 0.30. Neither `flat.sdf` nor `terrain.sdf` sets
  `<surface><friction>` on the ground, so both take the gz default `μ = 1.0` — good for
  45° standing, 33.7° walking. The bare terrain is not slippery.

  The **slip patches** are another matter, and this is where the terrain cap comes from:

  | μ | can stand up to | can walk up to |
  |---|---|---|
  | 1.0 (bare ground) | 45.0° | 33.7° |
  | **0.3 (slip patch)** | 16.7° | **11.31°** |

  `atan(0.3 / 1.5) = 11.31°`. **The 11.3° terrain cap is exactly the slope at which the
  μ = 0.3 patches remain walkable.** Above it the patches stop being a *slip* experiment
  and become an unconditional fall, which tests nothing. The generator already encodes the
  `μ > 1.5·tan(slope)` rule as a warning; `--max-slope 11.3` makes the world satisfy it by
  construction.

---

## 7. Parameter summary

`scripts/terrain_adapt.py`, publishing `/body_pose` at 50 Hz. **All gains at 0 reproduce
stock CHAMP bit-for-bit**, which is what makes this a clean A/B arm.

| param | default | ideal / limit | derived in |
|---|---|---|---|
| `com_shift_x` | 0.15 | **0.225** (`= h`, full CoM compensation) | §3.1 |
| `com_shift_y` | 0.0 | `h` by the same argument, for roll | §3.1 |
| `crouch` | 0.10 | no optimum — a dynamic-margin trade | §3.2 |
| `level_pitch` | 0.0 | keep ≤ 1.0; retains `γ/(1+k)` | §4 |
| `tau` | 0.7 s | 18 cm of lag at 0.25 m/s | §5 |
| `accel_gate` | 0.35 | reject `\|f\|` outside `(1±gate)g` | §5 |
| `max_shift` / `max_crouch` / `max_tilt` | 0.06 m / 0.06 m / 0.35 rad | safety clamps | — |

And in the joint controller, `config/ros_control_stiff.yaml`:

| param | stock | stiff | why |
|---|---|---|---|
| `p` | 100 | **300** | 2.85 cm → 0.95 cm of stance sag (§2) |
| `d` | 1.0 | **3.5** | `2√(pI)` with `I ≈ 0.01 kg·m²`; stock was under-damped even at p=100 |

**Status: none of this is validated in the live sim.** The geometry and the sign
conventions are verified offline (a synthetic tilted IMU reproduces the table in §3.1
exactly), but whether it gets CHAMP up a real slope is unmeasured. Per `skills.md` §0,
A/B the arms **one at a time** and budget several runs each — this gait's run-to-run
variance is large enough to fake any result you like from a single run.
