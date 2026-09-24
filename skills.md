# skills.md — go2_eskf sim-integration knowledge & session handoff

Working log of the go2_eskf ↔ Unitree Go2 sim integration debugging. Read this
first when resuming — it captures hard-won findings that aren't obvious from the
code. (Companion to `CLAUDE.md`; this file is the narrative + current state.)

Last updated: 2026-08-25 (terrain position error = leg-odom SPEED BIAS, §0) — see §0 "resume here". Current headline: the slip model is trained
on real logged runs and A/B'd within single terrain squares. It mostly *calibrates* `R_leg`
(the fixed value was 2.6x too small in variance) and does NOT respond to the µ=0.3 patches —
because those patches raise the leg-odom error by only ~10%, so there is little slip to
detect. Earlier headlines still stand: CHAMP's leg odometry mis-derived the body twist
(found and fixed analytically), and yaw is provably UNOBSERVABLE with GPS off.

---

## 0. CURRENT STATE (start here)

---

### 🧭 2026-09-23 — MAGNETOMETER HEADING INTEGRATED (off by default, not yet run live)

**What exists now.** `EskfCore::correctYaw(psi, r)` — direct ψ update, `H = e_ψ`, wrapped
innovation — and `EskfCore::magHeading(B, up, field_heading)`, a tilt compensation built from
VECTORS (horizontal projections of the field and of body +x about "up"), not roll/pitch angles.
"Up" is the node's `grav_lp_` (now updated for every `attitude_source`), in the same IMU frame as
the magnetometer, so the sim IMU's flipped frame never enters. Node: `use_mag`, `mag_topic`,
`mag_heading_noise` 0.05 rad, `mag_min_interval` 0.1 s, `mag_norm_gate` 0.25, `mag_ref_samples` 50,
`mag_calibrate_heading` true (reference = the filter's own initial yaw, i.e. spawn yaw 0 in ENU).
Raw heading on `/eskf/mag_heading` (`/eskf_slip/mag_heading` for the slip arm). Sim: gz
magnetometer on `imu_link` at 20 Hz (vendored edit #3, additive); `eskf.launch.py use_mag:=true`
starts a first-party bridge; `run_go2_teleop.sh --mag` passes it through.

**Green:** 22/22 GTest (6 new: pull + P shrink, ±π seam, b_g observable from heading alone,
level/tilted/upside-down heading, degenerate geometry), C++≡NumPy **2.498e-15** on BOTH the
original stream and a new heading stream that spins through ±π.

**Measured on a headless gz world** (static boxes at known poses, no Go2 — cheap and exact):
1. **gz ignores the SDF `<magnetic_field>` when `<spherical_coordinates>` is set** and uses its
   WMM tables instead, reported in **GAUSS** despite the field being called `field_tesla`:
   |B| 0.4326 G, horizontal 0.3096 G, pointing **4.0°** from world +x (not the SDF default's 76°).
2. **The noise `<stddev>` is in those same units** — the first draft's 5e-7 "tesla" would have
   been ~1000× too small. 0.005 G/axis measured **0.98°** heading std.
3. `magHeading` recovered true yaw to **0.000°** at yaw 0, 0.7, −2.0 (tilted 0.2/−0.15) and 0.6
   with **roll = π** (upside-down mount).
4. Node end to end on the upside-down box (fixed reference): ψ 0 → **0.598** (truth 0.600),
   fused at exactly 10 Hz.

**Offline price (`yaw_observability.py`, archived arms reproduce bit-for-bit):** GPS on, 5 m
square: 2.00° → **0.61°** (37/40) at an assumed 2° tilt-compensation error, 1.48° (29/40) at 5°,
**2.93° — worse — at 10°**. GPS off: 23.4° → 0.6–3.0°. Straight walking with GPS on: no gain
(0.46° → 0.58°). The magnetometer is the only heading source that works in the CORNERS.

**Next (this decides the default):** one walking `--square --mag` run, then compare
`/eskf/mag_heading` against ground-truth yaw (reject |gt_wz|>1.5 first). That error's std and
correlation time are the one unmeasured input; ≲5° → turn `use_mag` on, ~10° → raise
`mag_heading_noise` or leave it off. Note `timeseries.csv` has no mag column yet.

**First live run (2026-09-23 21:25, `--square --terrain --adapt --mag`, GPS on, both arms):**
CHAMP FELL after corner 3 at t=120.6 s, (−0.25, −2.40) — pitch rose 5°→20° over ~4 s on the
route's steepest cell (9.6°) and then it rolled 54°. Not the estimator (square_test steers off
truth). Before the fall: yaw error mean **1.1°**, p90 2.3–2.9°, max 7.4° (both arms); position
error mean 0.28 / 0.31 m. Magnetometer: 0 rejections by the |B| gate, and the raw mag heading tracked
filter ψ to a few degrees through all three corners. **One run, n=1, and no GPS-only run in this
config to pair it with** — the only archived terrain square is GPS-off (33° mean yaw error), not a
fair baseline. `/eskf/mag_heading` was not in `timeseries.csv` for this run, so the raw mag-vs-truth error
(the number that decides the default) is still unmeasured. **Fixed right after:** `run_report.py`
now records `mag_heading` and `mag_err` (signed, wrapped mag − truth yaw) as the last two CSV
columns, and `REPORT.md` gets a "Magnetometer heading vs ground truth" section: bias, de-biased
std, p90/max, std split straight vs turning, correlation time, and a verdict against the offline
thresholds. Upright samples only (tilt < 45°), and a heading older than 1 s counts as missing.
Checked on synthetic input with a known 2° bias / 3° white error: reported +2.02° / 3.05°, τ one
sample.

**WHY IT FELL — `terrain_adapt.py`'s pitch has the WRONG SIGN, so `--adapt` shifts the CoM
DOWNHILL on every slope (found 2026-09-23).** `attitude()` computes `atan2(−f_x, hypot(f_y,f_z))`
and treats it as +nose-UP. It is +nose-DOWN: nose-up tilts body +x toward the sky, so the gravity
reaction has f_x = +g·sin θ_up. (The docstring's "a nose-up pitch θ gives f = (−g sin θ, 0,
g cos θ)" is backwards — that is REP-103 pitch, which is +nose-down.) Evidence, three ways:
1. **Known pose, headless gz:** a model tilted 8.6° nose-up read **−8.6°** through the adapter's
   formula; `atan2(+f_x, …)` gives +8.6°. Roll `atan2(f_y, f_z)` is correct (+11.5° vs +11.5°).
2. **This run:** adapter pitch vs gt_pitch (REP-103) correlates **+0.42** (low-passed; fit slope
   +0.70). A correct nose-up pitch must correlate NEGATIVELY. §0 "MEASURED 3" found the same
   +0.25 and read it as noise; it is sign-flipped as well as noisy.
3. **The command:** `body_pose_x` vs gt_pitch **+0.73**. Nose-up samples got −0.027 m (body
   back), nose-down +0.023 m (body forward): downhill both ways. `com_shift_x` has been a
   destabilising gain since it was written; `crouch` uses |sin| and is unaffected.

**The fall itself fits that as a positive-feedback loop.** Last side, descending the start-pad
blend (terrain z 0.27 → 0.12 m, slope up to 9.6°): body pitch 7° → 23° nose-down over 111.6–115.4 s
while `body_pose_x` climbed +0.009 → +0.051 m (clamp 0.06), i.e. more nose-down → CoM pushed
further over the front feet → more front sag → more nose-down. Body pitch reached ~2× the terrain
slope; worst joint error 20–34° from 114.2 s; roll −48° at 115.4 s while `cmd_wz` was saturating
at 0.37–0.40 (steering). n = 1: CHAMP also falls on terrain without `--adapt` (~40 % completion),
so this is the leading cause for THIS fall, not proven to be the only one.

**FIXED (same day):** `attitude()` now returns `atan2(+f_x, …)`, documented as +nose-up (the
opposite of REP-103). The `level_pitch`/`level_roll` command was checked against CHAMP itself —
`getRPY` on `/body_pose`, then feet rotated by a standard right-handed `RotateY(−pitch)`, so a
commanded REP-103 +pitch puts the body nose-DOWN relative to the feet. The old leveling command
was therefore already right (inverted measurement × inverted comment cancelled); it is now written
as `cmd_pitch = +k·pitch_up` and is numerically identical. Verified through the node's own
`on_timer`: the gz known pose reads +8.6°/+11.5° (truth +8.6/+11.5); a 10° climb gives body x
**+0.026 m** (forward = uphill), a 10° descent **−0.026 m**, crouch −0.017 both ways, and
`level_pitch=1` on the climb commands +10° REP-103 (nose-down, levels it). **Every `--adapt` /
`--climb` result before this date ran with `com_shift_x` pointing downhill — none of them measures
the design; re-A/B from scratch.**

**First live run after the fix (22:56, `--square --terrain --adapt --mag`) — a FALSE "fall".**
`square_test` aborted 6 s into driving: "base z=0.180 m below 0.18 m", at (+0.42, +0.07) on the
FLAT start pad (slope 0.0°). The robot never went down: roll ≤ 11°, and it stood at z 0.22–0.24
afterwards. Chain of cause:
1. **The adapter misread the gait start as a 19–20° slope.** Its low-passed accelerometer pitch
   went +1.1° (standing) → **+19.3° nose-up** during the first ~5 s of trotting, against a ground
   truth of |pitch| ~1°, then settled (−1.3°, +2.9°). 142 of ~360 samples in that window were
   impact-rejected. This is §0 "MEASURED 3" (accel-only attitude is mostly noise) in its worst form:
   a transient bias of ~20°, not just scatter.
2. **So it crouched 3.3 cm and shifted 5 cm forward on flat ground** (clamps 0.06/0.06).
3. **Trotting base height is ~0.20 m** (standing 0.231); minus the crouch → 0.180, the threshold.
4. **The threshold is inconsistent with the adapter by construction:** `nominal_height 0.225 −
   max_crouch 0.06 = 0.165 < STAND_Z 0.18`, so a legitimately commanded crouch alone can trip it.
   Real belly flops sit at z ≈ 0.06–0.08.
The sign fix did not cause this: the crouch uses |sin|, so the old code crouched identically;
only the (spurious) shift direction changed.

**THRESHOLD FIXED (same day):** `square_test.py` now subscribes to `/body_pose` and uses
`stand_z() = STAND_Z − commanded crouch` (crouch = −position.z, capped at 0.08 m so a rogue
publisher cannot disable the check). No adapter → nothing on `/body_pose` → threshold stays exactly
0.18, so stock runs are unchanged. Checked: stock z 0.180 ok / 0.179 abort; the 22:56 case (z 0.180,
crouch 0.033 → threshold 0.147) ok; max crouch + deep bob (0.130 vs 0.120) ok; on its back (0.080)
abort; belly flop under a rogue −0.5 m crouch (0.060 vs 0.100) abort. The adapter's ~20° pitch
transient at gait start is NOT fixed — it still crouches and shifts on flat ground (needs a gyro
complementary filter, per "MEASURED 3").

**GYRO COMPLEMENTARY FILTER ADDED (same day) — `attitude_mode: complementary`, now the default.**
The gyro's body rates are integrated through the ZYX Euler kinematics (so turning while pitched
couples correctly), and pulled toward the gated accelerometer only over `cf_tau` = 10 s. The
effective time constant bootstraps from 0 up to `cf_tau`, so the ~10 s the robot stands before
walking give a running mean rather than one noisy sample. The posture still low-passes the result
over `tau` 0.7 s, to follow the slope rather than the stride. `attitude_mode: accel` keeps the old
estimator for A/B. Offline, through the node's own `on_imu` — synthetic IMU at 100 Hz, 2e-4 rad/s
gyro noise (the gz value), 0.3 m/s² accel noise, 30 % impact spikes, ±4.6° 2 Hz gait rock; error vs
the 0.7 s low-pass of true pitch, max:

| scenario | accel-only | complementary |
|---|---|---|
| static 8° up, 5° roll | 0.50° | **0.06°** |
| **flat + an accel bias equal to 19° for 5 s (the 22:56 gait-start failure)** | **18.18°** | **4.65°** |
| 0→10° ramp over 6 s | 0.98° | 0.11° |
| pitched 10° (and rolled 6°), turning 0.4 rad/s | 1.5–1.8° | 0.11° |

`cf_tau` trade-off: that bias case gives 7.90 / 4.65 / 3.92° max at 5 / 10 / 30 s, but a **0.01 rad/s
pitch-gyro bias** (real-IMU class; NOT modelled by this sim's gz gyro) gives 4.2 / 8.1 / 17.4° max.
10 s is the compromise; on hardware, lower it or estimate gyro bias while standing. Signs re-checked
through `on_timer` in both modes (gz known pose +8.6°; climb → body x +0.026 m, descent −0.026 m).
**Live check is now built in:** the adapter publishes `/terrain_adapt/pitch` (+nose-up, rad),
`timeseries.csv` gains `adapt_pitch_up`, and `REPORT.md` gets a "terrain_adapt attitude vs ground
truth" line (bias, mean/p90/max error). The 0.7 s smoothing is included in that error by design.

**First live run with the complementary filter (23:20, `--square --terrain --adapt --mag`) — a
real STUMBLE at gait start on the flat pad, then recovery.** Timeline: standing 11 s; `cmd_vx` steps
0 → 0.25 at 11.2 s; `cmd_wz` jitters 0 → 0.25 → 0.04 → 0.17 in 0.6 s (steering off a truth yaw that
wobbles ±10° per step); roll −19.5° at 13.0 s, then base z 0.139 at 13.2 s with all four feet down
and 24° joint error (legs buckled). `square_test` aborted (0.146 < 0.174) and zeroed the command.
AFTER the abort the body was thrown UP to z 0.345 (11 cm above standing), rolled +48.7°, spun 80° and
slid 0.5 m, then settled upright at z 0.231 — consistent with the under-damped stock PD (p 100,
d 1.0; SLOPE_POSTURE §6) rebounding from the collapse. **Not terrain** (slope 0.0°), **not the
adapter** (commands ≤ 1.1 cm shift / 0.7 cm crouch, and `cmdPoseCallback_` only sets the pose),
**not the estimator** (square_test steers off truth). The initiator cannot be pinned down from 5 Hz
data; the step velocity command plus steering jitter at gait start are the candidates. n = 1: the
archived 2026-08-28 run (no `--adapt`) started cleanly (first 3 s: min z 0.214, |roll| ≤ 7.8°).
**The complementary filter works live:** standing, adapter pitch − truth = **+0.01°** (max 0.02°);
walking before the trip **−1.46° mean, 4.6° max** — versus the accel-only estimator's +19° at the
previous run's gait start. Magnetometer in the same windows: +0.03° / max 2.0° standing, −0.38° /
max 4.8° walking; the report's 8.4° std is dominated by the tumble and 52 samples — not a
measurement of the sensor yet.

**`square_test.py` changed in response (same day):**
1. **Command shaping** (default; `-- --no-shaping` restores the old behaviour for A/B):
   - `cmd_vx` is rate-limited to 0.125 m/s² (0 → 0.25 in 2 s) and `cmd_wz` to 0.4 rad/s².
   - Steering and the TURN/DRIVE decisions use a 0.25 s unit-circle low-pass of truth yaw, so
     the ±10°/step gait wobble no longer flickers `cmd_wz`. The stall detector still uses raw yaw.
   - Aborts publish zero directly, bypassing the limiter.
   - Verified: cmd_vx 0.006 → 0.069 → 0.131 → 0.194 → 0.250 over 2 s, where it used to be
     0.25 on the first tick.
2. **Stumble recovery instead of abort-on-first-sample:**
   - A fall condition (z < `stand_z()` or tilt > 60°) now PAUSES the square with a zero command.
   - It aborts only if the condition holds **0.5 s** continuously, or the robot is not settled
     (standing and tilt < 25°) within **5 s**.
   - After **1 s** settled, the square resumes from a fresh ramp and the stall history is
     cleared. The summary now reports "stumbles recovered".
   - Replayed the real 23:20 truth through it: STUMBLE at 13.2 s → **RECOVERED after 2.0 s**,
     run continues (the old code aborted).
   - Synthetic checks: a persistent belly flop (z 0.06) aborts; flipped on its side (90°)
     aborts; a 0.1 s dip then 40° tilt aborts "not settled 5.0 s after"; a 40° scramble with
     no dip still aborts via the stall detector.
   - Cost: roughly +1 s per acceleration, about 5 s per 5 m square — far inside the 300 s cap.
   - Not yet run live. Whether the ramp reduces gait-start stumbles needs paired runs:
     `-- --no-shaping` vs default.

**First live run with shaping (23:28, `--square --terrain --adapt --mag`) — stopped by a FALSE
STALL at corner 1, no fall.** The robot went through three stages:
1. **Drifted off the line on the μ=0.3 patch at (5, 0).** Heading for (5, 0), it drifted to
   y +0.44 m with `cmd_wz` saturated at −0.4 from 34.8 s, while forward speed fell to ~0.1 m/s.
2. **Circled the corner.** A 0.44 m lateral offset (> `ARRIVE_TOL` 0.25) swings the bearing to
   the waypoint fast; `herr` passed `HEAD_REDO`, so the controller switched to TURN. **The new
   ramp made this worse:** in TURN it wanted v = 0, but the limiter decelerates at the same
   0.125 m/s² it accelerates at, so the robot kept walking for 2 s while turning. `cmd_wz` also
   needed 2 s to reverse from −0.4 to +0.4. Net effect: a loop out to y 0.71 m and back over
   38–46 s, before it got within 0.25 m of (5, 0) at ~46 s.
3. **The stall detector read the loop as no progress.** `_stalled()` compares only the window's
   first and last samples: 4 cm and 8° over 9 s, although the robot had walked a ~0.6 m loop
   in between. It aborted at ~48 s.

Slip score on the patch was 0.78 vs 0.60 off it (n = 82 / 105). That is **confounded**: the
on-patch samples are the slow, turning corner loop. Do not read it as slip detection.

Fixes proposed: (a) limit only speed-UP, and let |v| and |w| drop quickly; (b) define a stall as
"never got more than STALL_DIST from the window's start (and never turned STALL_YAW)", i.e. the
maximum excursion, not the endpoint difference, so a loop is not a stall.

**Both fixed (same day):**
- **`_ramp()` limits only growth of |cmd|.** Slowing and stopping are immediate; a reversal
  drops to zero at once and then ramps. Checked: 0 → 0.4 gives 0.02 on the first tick;
  0.4 → 0.1 and 0.25 → 0 are immediate; −0.4 → +0.4 gives 0.02 on the first tick.
- **`_stalled()` uses the maximum excursion** from the window's first sample, in position and
  in yaw.
  - The real 23:28 loop (39–48 s): the endpoint difference was 1 cm (old test: STALL); the new
    test says not a stall.
  - A robot scrabbling in place (±2 cm / ±2° jitter for 10 s) is still caught: "never more
    than 9 cm / 7 deg from where it was".
  - Not yet run live.

**23:35 run (`--square --terrain --adapt --mag`) — COMPLETED 4/4, first full terrain square since
the fixes.** 28.8 m of truth path, 0 stumbles, worst 9 s progress 55.9 cm (so no stall), max
slope 9.8°.
- **Estimator:** ATE mean 0.534 m (baseline) / 0.314 m (slip); max 1.386 / 0.775 m; final
  0.182 / 0.185 m; yaw error mean 1.9° / final 0.1° for both arms.
- **Magnetometer (first real measurement):** 794 upright samples; bias +0.60°, **std 4.07°**
  (straight 3.78°, turning 4.46°), p90 6.34°, max 21.0°, correlation time 0.2 s. By the offline
  thresholds (≲5° pays on the square) that is on the right side, but it is one run and NOT an
  A/B: both arms fused the magnetometer. Paired `--mag` vs no-`--mag` runs are next.
- **Adapter pitch has a +7–8° nose-up bias while walking.**
  - Per 20 s window: +6.6 / +8.3 / +8.1 / +7.2 / +8.7 / +6.8 / +7.4°.
  - Standing at the start: +0.7°; after stopping at the end it decays (+4.7°), and the log
    shows the accelerometer 6° off the complementary estimate while standing still.
  - **Not the accelerometer:** its gated mean while walking reads −2.6° (nose-down; 5 Hz
    samples, 44 % kept by the gate).
  - So it is the **gyro path**: a steady error of 7° against `cf_tau` 10 s implies ~0.7°/s of
    effective pitch drift from gyro integration while trotting, and none while standing.
  - Leading suspect (unverified): impulsive contact-impact rates sampled instantaneously at
    100 Hz. The ground-truth angular rate carries similar spikes (see above), and yaw gyro
    integration already showed 6–52° of error per run.
  - **Effect:** the posture sits ~2 cm forward with ~1.4 cm of crouch on flat ground. The run
    still completed.
  - **Candidate fix:** a PI (Mahony-style) complementary filter that estimates the gyro pitch
    bias, so a steady drift is integrated out. The floor would then be the accelerometer mean
    (−2.6° here). Needs `timeseries.csv` to log gyro x/y to verify the mechanism first.

**Yaw spikes (23:47 run, `--square --terrain --adapt --mag`, completed) come from the
MAGNETOMETER.**
- **Ground truth itself jumps up to 17° per 0.2 s.** The trot's yaw wobble is real body motion,
  and is not what is wrong.
- **Errors vs truth** (upright samples):
  - estimator: std 2.91°, p90 4.6°, max 15.3° (slip arm the same);
  - raw magnetometer heading: std 4.31°, p90 6.9°, max 19.4°.
- **The estimator's error tracks the magnetometer's:** corr(est err, mag err) = **+0.69**, vs
  +0.10 with truth yaw rate. The largest estimator errors (+9 to +15° around 59 s and 65 s)
  coincide with mag errors of +10 to +17°.
- **Mechanism — tilt compensation.** `magHeading` uses `grav_lp_` (accelerometer low-passed,
  τ ≈ 2 s) as "up". It cannot follow the gait's roll/pitch rocking, so the heading error
  scales with the body's instantaneous tilt (corr +0.48):

  | instantaneous tilt | mean \|mag err\| |
  |---|---|
  | 0–5° | 2.17° (n 450) |
  | 5–10° | 3.75° (n 266) |
  | 10–20° | 5.48° (n 110) |
  | 20–45° | 7.66° (n 12) |

  This is exactly the "correlated tilt error" `yaw_observability.py` had to ASSUME. It is now
  measured, and it is heavy-tailed.
- **`mag_heading_noise` 0.05 rad (2.9°) is too tight for it.** The filter believes a 15° spike.
- **Fix candidates:** (a) an innovation gate on `correctYaw` (reject |y| > k·√(P_ψψ + R));
  (b) R inflated by instantaneous tilt / gyro roll-pitch rate; (c) a better "up" vector
  (gyro-propagated, but note the adapter's complementary filter shows a +7° walking drift);
  (d) raising `mag_heading_noise` toward the measured 4.3°.

**(a) and (b) IMPLEMENTED (2026-09-24), default ON.**
- **Tilt lag feature.** `tilt_lag` = the high-pass of the body tilt with `grav_lp_`'s own time
  constant, i.e. how far the real tilt has run ahead of "up". Computed from gyro x/y as
  `tilt_hp_ = β·(tilt_hp_ + ω_xy·dt)`, which exactly mirrors the discrete low-pass. Only |·| is
  used, so a flipped gyro axis would not matter.
- **It predicts the mag error better than raw tilt:** corr +0.57 vs +0.48 (truth tilt, 5 Hz).
  RMS mag error by lag: 2.67 / 4.12 / 6.19 / 8.76° for 0–3 / 3–6 / 6–10 / 10+°.
- **Fit:** σ² = (2.96°)² + (0.60·lag)². Hence `mag_tilt_gain: 0.60`; the 2.96° floor matches
  `mag_heading_noise` 0.05 (2.86°), which stays.
- **Innovation gate:** `mag_gate_sigma: 3.0`, rejecting |y| > 3·√(P_ψψ + R_eff).
  `mag_gate_reset_sec: 5.0` fuses one sample after 5 s of unbroken rejections (lock-out escape).
- **Offline replay of the REAL mag headings from the 23:47 run** (NumPy twin; gyro = truth ×
  0.981 + drift noise; 10 seeds; lag from truth tilt). Yaw error, mean / p90 / max:

  | arm | mean | p90 | max |
  |---|---|---|---|
  | current (constant R) | 2.03° | 4.37° | 12.38° |
  | gate only | 2.09° | 4.49° | 10.03° |
  | **tilt R only** | **1.82°** | **3.89°** | **7.77°** |
  | both | 1.83° | 3.89° | 7.95° |

  The tilt-dependent R does the work. With it on, the gate never fires; it is kept as a safety
  net for non-tilt disturbances.
- **Caveats:** the gain was fitted on the same run it is scored on (in-sample, optimistic), and
  the live lag comes from the 100 Hz gyro rather than 5 Hz truth tilt.
- **Node smoke test** (headless gz, static upside-down box): lag 0.0° → σ 2.9°; converged 0 →
  0.601 rad (truth 0.600); the first, 33°-off update was accepted (P0 large), so no startup
  lock-out.
- **The node log line now reports** gated / forced counts and the current lag / σ.
- **Plotter:** the twist panel's y-range is set from the central 99 % of samples (+15 % margin),
  with a clipped-sample count in the title. Rendered from the 23:47 log, which contains a
  **519 rad/s** truth ω_z spike, it stays at ±1.7.
- **Also seen in that render:** the baseline arm's position error peaked at **2.3 m around
  60 s**, the same window as the yaw spikes.

**The slip model retrains after every run (2026-09-24): `scripts/auto_train_slip.py`, called
from the launcher's `cleanup()`.**
- **What it does:** archive the run's rows → train a candidate on all OTHER runs → score it and
  the deployed model on this run (held out) → if not worse, retrain on all runs, C++-check, back
  up, and replace. Details in CLAUDE.md.
- **Tested in scratch** (1.3 s for archive-only, 5.8 s to train twice and deploy):
  - First run: archive only.
  - Second run: held-out BCE candidate **0.6566** vs deployed **0.6626** (corr +0.527 vs +0.508)
    → C++ check 6.7e-16 → deployed, with a backup.
  - `--tol −1` forces the reject path: model unchanged.
  - 1000 rows with `gt_tilt` 1.2 rad dropped; a 199-row log and a missing log skipped.
- **Stale-data guard:** the launcher now deletes `run_report/slip_features.csv` at startup
  (like `outcome.txt`). It was only rewritten when the truth bridge was up, so a teleop run would
  otherwise have re-archived the previous run's data.
- **`slip_dataset/` seeded, archive only (no deploy yet):**
  - the 2026-08-28 `--square --terrain` run: 7889 rows;
  - the latest `--square --terrain --adapt --mag` run: 6847 rows.
- **Honest limits:**
  - BCE on soft labels is the trainer's own objective. A lower held-out BCE says the score
    tracks leg-odom error better, not that the ESKF localises better; that still needs the
    in-run A/B.
  - The gate compares against the deployed model on ONE new run, so run-to-run variance can
    flip it.

**00:31 run (`--square --terrain --adapt --mag`) — the robot NEVER STOOD. A boot failure; no
first-party code involved.**
- **Timeline** (launcher / gz log clock):
  - entity spawned 258.7 s;
  - `joint_states_controller` 278.7 s;
  - `joint_group_effort_controller` activated **289.5 s** — **31 s limp**, which is the vendored
    launch's spawner timing;
  - the adapter's first attitude reading at 299.9 s was already at its −20° roll clamp, with
    |f| ≈ g steady (0 impacts rejected, |accel − cf| 0.1°): lying still on its side.
- **Ground truth**, once its bridge came up: roll −80° → −94° → settled −89.6°, z 0.162 m, 0.4 m
  from the spawn point. `square_test` correctly held zero ("never stood").
- **Nothing first-party was acting on it:** the adapter's roll gains are 0 (identity pose), the
  ESKF is passive, and `square_test` held zero.
- **The launcher is where the gap is:** `WAIT_READY` prints "Leg controller ACTIVE — the Go2 is
  standing." on controller activation alone, never checking the robot actually stood. So a
  sideways robot boots the whole stack and sits until Ctrl-C or the 300 s cap.
- The auto-trainer handled it correctly: the slip log was empty (every leg-odom sample was
  degenerate while lying still), so nothing was archived and the deployed model was unchanged.
- **Proposed fix:** after activation, confirm the robot is upright from `/imu/data` (gravity
  direction), then either fail fast with "FAILED TO STAND" or relaunch the sim automatically.

**00:39 run (`--square --terrain --adapt --mag`) — COMPLETED; the first live run of the magnetometer
tilt-R + gate.** 26.8 m, 0 stumbles, worst 9 s progress 21.4 cm.
- **Estimator yaw error vs the 23:47 run (spikes)** — n = 1 each, and the mag sensor itself
  was similar (std 3.89° vs 4.31°), so the comparison is fair-ish:

  | | std | p90 | max |
  |---|---|---|---|
  | 23:47 (constant R) | 2.91° | 4.6° | 15.3° |
  | **00:39 (tilt R + gate)** | **2.09°** | **3.41°** | **8.61°** |

  Slip arm: 2.23 / 3.82 / 7.57°. The offline replay had predicted a max of 12.4 → 7.8°.
- **The gate did not fire:** 0 gated, 0 forced, over 1405 fused headings.
- **Position:** ATE mean 0.672 m (baseline) / 0.385 m (slip); final 0.360 / 0.277 m; yaw final
  0.9° for both.
- **Adapter pitch bias** is unchanged at **+7.24°** (open issue above).
- **Auto-train:** the 4th run was archived (6650 rows) and deployed, retrained on 4 runs /
  27802 rows.
- **DESIGN FLAW in the deploy gate, found in that result:** candidate and deployed scored
  IDENTICALLY (BCE 0.6194, corr +0.609).
  - After any deploy, the deployed model WAS trained on exactly "all runs but the newest", so
    the candidate (same data, same seed, same epochs) is the same network. The gate then always
    passes, and it never evaluates the model it actually deploys (which includes the new run).
  - It only discriminates right after a rejection.
  - Fix: leave-one-run-out cross-validation over the whole dataset. For each run k, train on the
    others and score on k. Compare the mean CV score against the deployed model's, or against the
    previous dataset's CV. Costs ~N trainings (N × ~2 s).

**00:50 run — second boot failure in four launches: flipped onto its BACK before the square
started.**
- **Timeline:**
  - spawn 436.7 s; effort controller 468.6 s (32 s limp);
  - the adapter's complementary filter initialised from a gravity-like accel sample at 473.4 s
    (so the robot was upright then), and at 478.4 s logged roll +0.1°, pitch-peak 6.5°;
  - ground truth from its first sample: roll ±180°, z 0.075 m, x −0.30 m;
  - raw accelerometer roll median **179.7°** (|f| 9.80): it really is on its back;
  - joint error 0.7° (legs unloaded, vs ~4.5–5° when standing), zero command throughout.
- So it stood, or was at least upright, at 473 s, and was on its back by ~483 s with no command.
  The flip itself is in no log. The timeseries starts with ground truth; the adapter logs every
  5 s, and its estimate is unreliable here (see below).
- `--adapt`'s commands were ~2 mm, so it is unlikely but not ruled out. A few boots without
  `--adapt` would settle it.
- **Bug found — the adapter's complementary filter does not wrap the accelerometer
  correction.** Near ±180° the accelerometer roll alternates +179.7 / −179.7, the corrections
  cancel, and the estimate sat at ~0–6° for the whole run while truth was 180°. It also never
  integrated the flip from the gyro. Harmless for posture (a robot on its back has none), but it
  needs `atan2(sin, cos)` on the innovation.
- **Shutdown:** the launcher was stopped with SIGTERM; `cleanup()` ran and the trainer skipped
  the empty slip log. A `gz sim` process briefly outlived the cleanup and then exited, which is
  how the 22:56 orphan could have happened.
- **2 of the last 4 launches failed at boot.** The stand-up check (fail fast or auto-relaunch)
  is now the most valuable fix for unattended runs.

**Both fixed (same day):**
- **`terrain_adapt.py`:** the complementary filter's accelerometer correction now uses a
  wrapped innovation (`atan2(sin, cos)`), and roll is wrapped after each step.
- **`stand_check.py` + in-place relaunch in `run_go2_teleop.sh`:** details in CLAUDE.md.
  - Verified headless: an upright model reads **tilt 0.0° → UPRIGHT (exit 0)**; an upside-down
    one reads **180.0° → NOT_UPRIGHT (exit 1)**.
  - That test also caught a bug: a bare `--` reached argparse when the script was run without
    `--ros-args`. It is now filtered.
  - **End to end, forced** (`GO2_STAND_MAX_TILT=-1 --square --boot-retries 1`), three runs:
    - **Run 1:** attempt 1 relaunched in place (PID unchanged, 1:44 elapsed across both), then
      attempt 2 gave up: FAILED TO STAND, exit 3; `REPORT.md` shows "Boot attempt 2 of 2
      (earlier: attempt 1 tilt 1.8deg)". **But attempt 2 measured 180.0° — a genuine flip.**
      Teardown had not waited for the gz server, which was repeatedly seen alive seconds after
      `cleanup()`, so attempt 2 booted alongside a dying gz on the same topics. That is a
      plausible cause of that flip, and of the 22:56 orphan. Fixed: teardown waits up to 15 s
      for gz, then SIGKILLs it.
    - **Run 2** exposed a bug in that fix: unanchored `pgrep -f "gz sim"` matched any SHELL
      whose command line contained the text. Teardown then waited 15 s on it and SIGKILLed it,
      twice killing my own test shells. Now anchored to `'^gz sim'`.
    - **Run 3, clean:** relaunch with no spurious wait → give up → exit 3 → **no gz left**.
      Both boots stood (tilt 1.2°, 2.1°), and so did run 2's second attempt (2.0°) once the gz
      wait existed.
**01:16 run (`--square --terrain --adapt --mag`), first real launch with the stand check:**
- Stand check passed on the first boot (tilt 2.5°); the square completed 4/4 with 0 stumbles;
  exit 0.
- Estimator yaw error std **1.82°** / p90 3.11° / max **7.16°** (slip arm 1.87 / 3.22 /
  5.94°). That is the lowest so far, after 2.91 / 4.6 / 15.3° (23:47) and 2.09 / 3.41 / 8.61°
  (00:39).
- Magnetometer std 3.48°.
- Position ATE mean 0.472 / 0.291 m; final 0.170 / 0.222 m; yaw final 0.2 / 0.1°.
- Slip model auto-deployed (7 runs, 49561 rows).
- n = 1 per configuration; the downward yaw trend is consistent but unproven.

  - The boot flips (00:31, 00:50, run 1 attempt 2) are therefore partly unexplained. The last
    one coincided with overlapping gz servers; the first two did not, since each was a fresh
    launch.

**Gotcha found on the way:** `install/` is a MERGED layout, so `colcon build --packages-select`
without `--merge-install` refuses — and the cross-validator then silently runs the OLD binary
(it failed at 1.456 until rebuilt, which is the check doing its job).

---

### ⏱ SESSION HANDOFF — 2026-08-12 (RESUME HERE)

Branch `fix/sim-readiness-guard-and-drift-baseline`. **The slip model is now trained on real
robot data and A/B'd on terrain across multiple same-run comparisons.** Headline: it makes
`R_leg` honest, it does NOT detect the low-friction patches — and the measurement below says
that is the *right* answer, because those patches barely produce any extra leg-odom error.

**HEADLINE 0 — is `R` the right place for the slip model? Measured 2026-08-12: mostly NO**

Asked after a `--square --terrain --adapt` run reported the slip arm as far worse (raw ATE mean
0.477 → 2.221 m). Two separate answers came out of it.

*First, that run was NOT a slip loss.* `REPORT.md`'s "position error (ATE mean)" is the RAW
mean error, which a constant heading offset dominates. Recomputed with `scripts/metrics.py`
on the same `timeseries.csv`, both arms:

| | ATE (SE2-aligned) | ATE raw | RPE trans / 2 s | RPE rot / 2 s | final drift |
|---|---|---|---|---|---|
| baseline | **0.362 m** | 0.568 m | 0.155 m | 4.50° | 1.211 m |
| slip | **0.405 m** | 2.509 m | 0.148 m | 4.51° | 1.250 m |

Identical trajectory SHAPE and identical LOCAL rotation error; the entire 4.6× raw gap is one
low-frequency heading offset (yaw error mean 5.8° vs 21.5°). Consistent with the known
mechanism — inflating `R` removes what little heading pull the leg-odom update has
(`slip_yaw_experiment.py`, 16/40 → 10/40). **`REPORT.md`'s ATE column is raw; do not read it as
ATE.** Fixing that column is on the list.

*Second, and more useful — the diagnostics on `slip_features.csv` (5512 usable rows,
`|gt_wz|<1.5` filtered) say `R` is conceptually admissible but a weak and mis-shaped lever:*

1. **The error IS zero-mean, so `R` is not the wrong object.** Bias accounts for 0.8 % (vx) /
   1.8 % (vy) of the error energy. There is no systematic offset for an `h`-side correction to
   remove. Score one for the current design.
2. **But 86 % of the model's output is a constant.** Inflation factor `1+s`: mean 1.603,
   std/mean **13.6 %**. Fixed σ=0.10 gives a normalised innovation variance of **4.17** where a
   2-D update wants 2.0; the optimal *constant* σ is **0.144**, and the model's mean lands 0.160.
   It is a calibration wearing an MLP.
3. **There is heteroscedasticity to exploit, and the features can't see it.** |e| spans
   p10 0.057 → p90 0.291 (**5.1×**), but held-out R² for |e| is **0.182** with all 8 features and
   **0.063** without the three `cmd_minus_*` ones. Strip the command and the gait features
   explain ~6 % of the spread. That is the honest ceiling of this feature set.
4. **What the model actually learned is "`cmd_vel` beats leg odometry".** Signed-error held-out
   R² is 0.530 (ex) / 0.412 (ey) with `cmd_*`, and **0.010 / −0.008** without. Directly:
   RMSE against ground-truth body velocity is leg odom 0.124 / 0.163 vs `cmd_vel` **0.110 / 0.117**
   — and `cmd_vy` is identically zero. Beware: in sim the command tracks truth; on a real robot
   slip is precisely what decorrelates them, so this signal does not transfer.
5. **`R` is isotropic and shouldn't be.** σx 0.123, σy **0.161**, corr −0.09. Worse, the lateral
   channel carries no information at all: least-squares `leg_vy/gt_vy` = **0.089**, corr +0.081,
   while `leg_vy` std (0.123) exceeds truth's (0.114). CHAMP's `vy` is noise, fused at σ=0.10.
6. **The gain is saturated, so `R` barely moves the velocity update.** With `accel_noise: 5.0`,
   `P_vv ≈ 0.5` between leg-odom updates, so `K` goes 0.980 → 0.926 across the model's ENTIRE
   range. The residual effect lands on the covariance/heading coupling — i.e. the one place it
   hurts.
7. **Structural: with GPS off there is nowhere for the trust to go.** Adaptive `R` pays when a
   competing information source can take over. Here leg odometry is the only velocity anchor and
   the IMU is deliberately gutted (`accel_noise: 5.0`, `gravity_lp`), so de-weighting it means
   free-running integration, not "lean on the other sensor".

**Ranked consequences (none implemented yet):** (a) make `R_leg` anisotropic — σx 0.123,
σy 0.161 — and consider treating `vy` as near-uninformative; free, no model needed. (b) An online
NIS/covariance-matching estimator would deliver the calibration in point 2 without any MLP, which
is ~86 % of what the model currently delivers. (c) If the learned model stays, train it with a
Gaussian NLL on a per-axis log-variance head instead of regressing `clip(|e|/0.3,0,1)` through a
sigmoid — that label discards sign AND axis AND saturates (p90 |e| = 0.291 vs the 0.3 clip).
(d) Harsher `--patch-mu` is still the experiment that would create slip worth detecting.

**HEADLINE 1 — the fixed `R_leg` was simply wrong, and that is most of what the model fixes**

Measured on terrain from 13.5 k live samples (`slip_features.csv`, labelled against
ground-truth body twist):

| quantity | value |
|---|---|
| leg-odom velocity error \|v_leg − v_truth\| | median **0.162**, p90 0.342 m/s |
| least-squares scale `v_leg / v_truth` | **0.95** — so the error is RANDOM, not a scale bias |
| `leg_odom_vel_noise` (the fixed `R_leg` std) | 0.10 m/s |

The fixed `R_leg` was **~2.6× too small in variance**. The trained model's mean score 0.68
inflates the std to `0.10·(1+0.68) = 0.168` — i.e. almost exactly the measured residual. So
its first-order effect is a *calibration* it learned from data, not slip detection. Note
`leg_odom_scale: 1.111` is vindicated on terrain: the scale comes out at 0.95, not 0.82.

**HEADLINE 2 — the model does not respond to the µ=0.3 patches, and it shouldn't**

Slip score inside vs outside the four `mu=0.3` patches, three runs: 0.714/0.659,
0.682/0.687, 0.686/0.680 — **flat**. Before blaming the model, the ground truth was checked:
the leg-odom error itself is only ~10 % higher on the patches (median 0.204 vs 0.180 m/s,
consistent across all three runs). **The patches barely break the stance-foot assumption at
this gait**, so there is almost no friction signal to detect and the error is dominated by
gait/roughness noise everywhere. Do not read "the model failed to detect slip" from this —
read "this world does not produce much slip". Making the patches harsher (lower `--patch-mu`)
is the experiment that would actually test detection.

**FINAL BENCHMARK — proper ATE/RPE, 7 paired runs, 297 m of ground truth**

Computed with the package's own `scripts/metrics.py` (ATE = position RMSE after a rigid
SE(2)/Umeyama alignment; RPE over a 2 s gap) on every valid run's `timeseries.csv`, both arms:

| metric | baseline (fixed R) | slip-adaptive | slip better in |
|---|---|---|---|
| **ATE** (SE(2)-aligned RMSE) | median **2.066 m** (0.54–4.36) | median **1.246 m** (0.27–2.52) | **6/7** |
| RPE translation / 2 s | 0.130 m (0.10–0.21) | 0.119 m (0.11–0.19) | 4/7 |
| RPE rotation / 2 s | **2.10°** (1.4–8.3) | 3.34° (2.3–3.8) | 2/7 (baseline better) |
| final drift, aligned | 6.85 % of path | 4.97 % of path | 5/7 |

**Read the alignment carefully before quoting any of this.** ATE and the drift % are computed
AFTER an SE(2) alignment, which absorbs a constant heading offset — so they measure trajectory
SHAPE. On shape the slip arm wins consistently (6/7, −40% median). On RAW, unaligned final
error it is still a coin flip (5/8 including the run below), because that metric is dominated
by the unobservable heading. Both statements are true; they measure different things, and
"ATE improved 40%" is only honest alongside "absolute drift did not".

Caveat on n: one 5 m run (baseline 2.202 m / slip 4.317 m final — a slip LOSS) was overwritten
before it was archived, so it is in the table above's 5/8 but not the 7-run metrics. Excluding
it flatters the slip arm slightly.

RPE rotation is the one metric where the baseline clearly wins (2.10 vs 3.34 °/2 s): inflating
`R_leg` costs local heading stability, consistent with `slip_yaw_experiment.py`'s 16/40 → 10/40.

**THE A/B ITSELF — 5 valid runs, no proven effect on RAW final error**

Both arms run in ONE `--terrain --square`, so each row is a *paired* sample and the
run-to-run variance cancels. `run1` used the old synthetic weights; the rest the trained
model. Runs where CHAMP fell are excluded (see the ~40 % completion rate below).

| run | ATE base | ATE slip | final base | final slip | yaw base | yaw slip | slip score |
|---|---|---|---|---|---|---|---|
| run1 (synthetic) | 3.658 | 3.113 | 4.707 | 3.065 | 7.5° | 22.9° | 0.34 [0.00,1.00] |
| run2 | 2.325 | 3.520 | 5.192 | 3.099 | 15.7° | 15.7° | 0.68 [0.24,1.00] |
| run4 | 4.385 | 4.706 | 13.540 | 5.801 | 125.6° | 60.2° | 0.69 [0.18,1.00] |
| run6 | 7.546 | 2.766 | 5.374 | 9.177 | 30.9° | 72.9° | 0.68 [0.24,1.00] |
| run10 | 4.516 | 4.660 | 5.760 | 3.824 | 38.4° | 4.4° | 0.64 [0.20,1.00] |

* **final error**: baseline 6.92 m vs slip 4.99 m, paired diff **−1.92 m**, slip better in
  **4/5** runs, t = −1.05.
* **ATE**: baseline 4.49 m vs slip 3.75 m, paired diff −0.73 m, slip better in only **2/5**.
* **yaw**: split 3/2 the other way, and the baseline's 125.6° in run4 shows the spread.

**Verdict: not proven.** The final-error sign is consistently in the slip arm's favour but
ATE disagrees with it, n = 5, and t ≈ −1. This matches `slip_yaw_experiment.py`'s offline
result (inflating `R` is a coin flip on final error). Do not quote a slip-model improvement
from this. The ablation in "Next steps" is what would settle it.

**HEADLINE 3 — the gyro scale error is 1.9%, not 4–17%, and the old number was an artifact**

Measured across 6 runs / 146 k logged samples, `wz_gyro` regressed on ground-truth `wz`:
**0.976–0.984, mean 0.981**, correlation 0.92, bias < 0.003 rad/s — remarkably consistent.
That is only **−0.43 °/s** at a `wz=0.4` corner, ~7° over four corners. It cannot explain the
15–125° yaw errors, so **the gyro scale is not the heading culprit** and there is nothing
there for a model to learn.

§0's earlier "0.830 and 0.964, i.e. a 4–17% scale error" is **retracted**. The cause is a
data defect found while checking it: **the gz `OdometryPublisher`'s angular velocity emits
enormous spikes** — up to **583 rad/s** (~93 rev/s), in 0.3–0.5% of samples, in 4 of 6 runs.
Regressing against unfiltered `gt_wz` collapses the slope to ~0.03. **Always reject
`|gt_wz| > 1.5` before using ground-truth yaw rate.** The LINEAR channel is clean (max
|gt_vx| 0.94 m/s, zero samples above 1 m/s), so the slip training labels — which use linear
velocity only — are unaffected.

What is left is integration of zero-mean gyro noise: `∫(gyro − truth)dt` over a run comes to
**6–52°**, the same order as the observed final yaw errors (7–126°) and with no consistent
sign. Heading drift here is a random walk, not a bias or scale that any feature can predict.

**A "robot is turning" feature would NOT help the slip model** (asked and tested 2026-08-12).
Two independent reasons:
1. **Turning barely changes what `R_leg` predicts.** corr(|wz|, leg-odom velocity error) =
   **0.006–0.145**; median error turning vs straight = **0.96–1.10x**. Almost no signal.
   (The current 8 features indeed cannot see turning — best proxy is `cmd_minus_leg_vx` at
   corr 0.29 — but per the above it does not matter.)
2. **It would push the wrong way.** `R_leg` scales the leg-odom VELOCITY update, whose `H`
   has a nonzero ψ column ∝ speed, so inflating `R` during turns *removes* what little
   heading pull exists — which is exactly what `slip_yaw_experiment.py` measured
   (heading-kick recovery 16/40 → 10/40 when `R` is inflated).

The effect the intuition is reaching for already exists, in the right place: `Q(ψ,ψ)` gets
`(gyro_scale_noise·ω_z)²·dt`, so heading uncertainty already grows while turning (at ω=0.4
that term is ~400x the white-noise term), and `bias_update_max_wz: 0.10` already gates the
bias pseudo-measurement out of turns.

**Then the square moved to 5 m sides, and two runs promptly disagreed**

`square_test.py --side` now defaults to **5 m** (20 m of travel, ~185 s end to end vs ~370 s).
Two runs on it, same config, same trained model:

| run | final base | final slip | verdict |
|---|---|---|---|
| sq5m #1 | 2.699 m | 0.526 m | slip **2.17 m better** |
| sq5m #2 | 2.202 m | 4.317 m | slip **2.11 m worse** |

Equal and opposite. Treat that as the calibration for how much any single square is worth.
Note the 5 m route clips only two of the four `mu=0.3` patches, at its CORNERS (the patches
were laid out for the 10 m square's leg midpoints), so it exercises the slip model *less* —
and its percentages are not comparable to the 10 m numbers above (20 m travelled, not 40 m).

**`/eskf/slip` renamed to `/eskf/slip_score`** (slip arm: `/eskf_slip/slip_score`). It is a
Float64 diagnostic and was one character away from the slip ARM's namespace
(`/eskf_slip/odom`), which got it read as a second trajectory more than once. The plot's
third curve has always come from `/eskf_slip/odom`; nothing about the plot changed.

**Runs now stop themselves** — `run_go2_teleop.sh` has a supervisor loop with two automatic
exits: the square's drift summary appearing, and a `--timeout` wall-clock cap (300 s default
under `--square`, sized for the 5 m route — a 10 m route needs it raised). Both call the same
`cleanup()` as Ctrl-C, so a capped run still writes `REPORT.md` and `node_logs`, and the
outcome line appears at the top of the report's Outcome section. Both paths verified live:
`--timeout 60` fired at 60 s and left nothing running; the default run self-terminated at
193 s on completion.

**What changed**

1. **`slip_log_path`** on `eskf_node` — one CSV row per *fused* leg-odom update (same call
   site as inference, so train and test distributions match), features + ground-truth body
   twist. The launcher writes `run_report/slip_features.csv` whenever the GT bridge is up.
2. **`train_slip_model.py --runlog`** labels rows `clip(|v_leg − v_truth|/0.3, 0, 1)` and
   reports the two things that matter: score spread, and score-vs-error decile gap /
   correlation. Trained on run 1, val **corr +0.54**, worst-decile 0.83 vs best-decile 0.43.
   `config/slip_model.txt` is now that model (provenance in its header comment);
   `config/slip_model_synthetic.txt` keeps the old never-saw-a-robot weights for A/B.
3. **`contact_frac` wired to `/foot_contacts`** — and thereby *measured dead*: 0.5 in
   13467/13467 samples, structurally (the degenerate gate drops 0-and-4-feet samples; a trot
   in between always stands on one diagonal pair). Optional `champ_msgs` dep.
4. **`REPORT.md` now names the loaded slip model** (path, or `**FAILED TO LOAD**`). The old
   failure mode — an empty `slip_model_path` silently making both arms identical — is now
   visible instead of looking like a null result.

**Traps that cost time this session — read before automating a run**

* **`kill -INT` on a backgrounded `run_go2_teleop.sh` does nothing.** A command started
  asynchronously from a non-interactive shell has SIGINT set to IGNORE, and bash cannot trap
  an ignored-on-entry signal. The launcher gets SIGKILLed, `cleanup()` never runs, and the
  next report is written against **stale `node_logs/`** — the archived square summary was
  literally the previous session's numbers. Use `kill -TERM`.
* **Orphans from that failure keep running.** A `square_test` node from a dead run survived
  45 min and burned ~30 % of a core through five later runs. It no longer publishes
  `/cmd_vel` once `done`, but check for orphans between runs.
* **Never `colcon build` during a run.** The compiler was OOM-killed and the sim starved;
  CHAMP fell 19 s in and logged 9964 m of scrabbling-feet "drift".

**The terrain square only completes ~40 % of the time.** 5 of 8 honest attempts stalled
("no progress for 9 s while commanded"), three of them at **x ≈ 2.1 m on the first leg** —
the pad-blend annulus CLAUDE.md warns about. Budget ~2.5 launches per usable run.

**Next steps**

1. Lower `--patch-mu` (0.30 → ~0.1) and regenerate the world, then re-A/B. That is the only
   way to find out whether the model detects slip, as opposed to calibrating `R`.
2. Raise `leg_odom_vel_noise` from 0.10 to ~0.17 in the BASELINE arm and re-run. If the
   baseline then matches the slip arm, the model is worth nothing beyond that one constant —
   this is the cheapest possible ablation and it has not been run.
3. `--adapt` / `--stiff` remain unvalidated. A/B one at a time.

---

### YAW: where the 12.3° actually comes from, and the two things worth fixing (2026-08-22)

Measured on the same terrain square (`run_report/`, baseline arm: final yaw error **12.3°**,
mean 5.4°, +5.7 °/min; slip arm 31.4°). §0's unobservability result stands and is the right
frame: with GPS off nothing corrects `ψ`, so `ψ = k·Δψ_true − ∫b̂ dt` and **all** heading
error is accumulated yaw-RATE error. That makes this a rate-error budget, not a tuning problem.

**⚠️ TOOLING TRAP — `timeseries.csv` and `slip_features.csv` are on DIFFERENT CLOCKS.**
`timeseries.csv` `t` is **0-based from run_report's start**; `slip_features.csv` `t` is **raw
sim time**. On this run the offset is **+28.80 s** (recovered by cross-correlating `cmd_wz`,
confirmed on `gt_vx` +0.883 and `gt_wz` +0.755). Joining them naively silently produces
garbage — it flipped a `leg_wz`↔`gyro_wz` correlation to **−0.38** when the true value is
**+0.82**. Always align before any cross-file join. Also in this file set:
`err_yaw`/`slip_err_yaw` are **RADIANS** (`REPORT.md` converts); `leg_degenerate` was **all
NaN**; and `gt_wz` carries the ±599 rad/s truth spikes §0 warns about — reject `|gt_wz|>1.5`.

**THE BUDGET (this is the useful part)**

| term | contribution | note |
|---|---|---|
| gyro scale error | **+2.5°** | `k=0.986` this run × **net** Δψ = −176.1° |
| everything else (bias + random walk) | **~10°** | the actual problem |
| *measured final* | **12.3°** | |

A scale error integrates against the **NET** heading change, not total \|turning\| — 176°
here, not 497°. Do not budget it against total turn (I did, first pass, and got 7.0°; wrong).
This also **confirms §0's dismissal of the scale term for the right reason**: at ~1.4–1.9%
it is worth 2.5–3.4° and cannot explain 12.3°. A `gyro_z_scale` param is cheap and correct
but is **not** the fix.

**MEASURED — `leg_wz` reads ~40% LOW live, against an offline prediction of 1.000.**
Properly aligned, degenerate zeros dropped (n=501):

```
leg_wz = 0.606 * gyro_wz      corr +0.816
leg_wz = 0.718 * gt_wz        corr +0.877   (5 Hz, |gt_wz|<1.5)
gyro_wz = 0.986 * gt_wz       corr +0.935
```

Two independent comparisons agree `leg_wz/truth ≈ 0.60–0.72`. **`leg_odom_model.py` predicts
1.000 after vendored edit #6, and `odom_scaler` is NOT applied to `angular.z` in
`odometry.h` (only `vx_raw`/`vy_raw`), so no scaler explains this.** skills.md flags edit #6
as "offline-proven, LIVE-UNPROVEN" — this is the first live measurement of its yaw rate and
**it does not reproduce.** Caveat: 5 Hz-decimated `leg_wz` vs 50 Hz gyro, and CHAMP's
`beta_=0.1` output smoothing both bias the estimate low, but not from 1.0 to 0.6.
**Resolve this before any further yaw work** — and offline, via `leg_odom_model.py`, per §0.

**MEASURED — the gyro-bias pseudo-measurement is over-trusted by ~32× in variance.**
`correctGyroBias` is fed `(gyro_wz − leg_wz)` with `leg_yaw_bias_noise: 0.05` rad/s. Inside
the `bias_update_max_wz: 0.10` gate the residual's **actual** std is **0.283 rad/s** — 5.7×
the assumed σ, **32× in variance** — with mean +0.0229 rad/s (+1.31 °/s). Held for the run
that mean alone is +152° of false heading. Outside the gate the std reaches 0.55 rad/s.

Worse, the residual is **structurally scale-shaped**: `leg_wz = 0.606·gyro_wz` ⇒
`(gyro − leg) ≈ 0.394·wz`, i.e. proportional to `wz`. **A gyro bias is constant by
definition**, so fusing a `wz`-proportional residual as bias is exactly the failure
`bias_update_max_wz` was added to prevent — and the gate only *shrinks* it (at |wz|<0.10 it
still admits up to 0.039 rad/s = 2.3 °/s of false bias). The per-bin residual means
(+1.31, −1.39, +1.80, −0.14 °/s) show no clean monotone trend only because the 0.28–0.55
rad/s noise swamps it at these sample counts.

**RANKED, and note what is NOT on the list**

1. **Fix `leg_wz`'s 0.6 scale, offline, first.** It is the input to the only yaw-adjacent
   update in the filter. Everything below is unsafe while it is wrong.
2. **Re-tune `leg_yaw_bias_noise` from 0.05 to the measured ~0.28 rad/s**, or drop the
   pseudo-measurement entirely. One-line change, and the measurement says the current value
   is indefensible. **Do NOT open `bias_update_max_wz`** — with a scale-shaped residual the
   gate is doing real work; opening it (0.10→0.40 admits 24%→51% of samples) makes it worse,
   which is the opposite of what I first assumed.
3. **Estimate gyro bias where it is actually observable: standing still.** A genuine ZUPT
   gives `gyro_wz = bias` directly, with no leg odometry in the loop and no scale error to
   confound it. The machinery already exists (`degenerate_hold_sec`, 14% of `leg_wz` samples
   are degenerate zeros) — this is the *clean* bias observation, and it is currently mixed in
   with the dirty one.
4. `gyro_z_scale ≈ 1.015` — correct, cheap, worth ~2.5–3.4°. Do it, but do not expect a fix.
5. **The only structural cure remains an absolute heading reference** (§0 HEADLINE 2): fix
   the navsat `<stddev>` and enable GPS, or fuse an AHRS/magnetometer yaw. 1–4 slow the
   drift; only this bounds it.

Not on the list, deliberately: `Q`/`R` tuning on the yaw states, and anything touching the
slip model. Neither can correct an unobservable state.

---

### "ON A GRADE" vs "SLIPPING": which is observable, and from what (2026-08-22)

Asked how the estimator could tell that the robot is *walking a grade* from that it is
*slipping*. Answer: they are separable, but **not in the channel the workspace currently
uses for either.** Both were measured — grade on the last terrain square's
`run_report/` (582 rows @ 5 Hz + 5056 slip rows @ 50 Hz), slip offline on
`leg_odom_model.py`.

**The confound to avoid.** A grade tilts gravity into the body's x-axis; slip shows up as a
kinematics/inertial disagreement. Both therefore look like "unexplained horizontal
acceleration", and an accelerometer cannot tell a sustained tilt from a sustained
acceleration at all (the classic specific-force ambiguity). So do **not** try to separate
them in the accel channel. They separate cleanly in two *orthogonal* observables:

| | observable | why it is orthogonal |
|---|---|---|
| grade | the vertical/geometric channel — Δz over arclength, or a plane fit through touchdown points | slip does not change the contact-plane normal |
| slip | the inter-foot kinematic-consistency residual | a grade does not change the distance between two planted feet |

**MEASURED 1 — grade is recoverable from the z-trajectory, but only over an arclength
window longer than one gait cycle.** Regress `gt_z` on path length over a sliding window,
`grade = atan(dz/ds)`, and correlate against the heightmap's own slope under the path:

| window | corr(\|grade\|, heightmap slope) |
|---|---|
| 0.25 m | +0.597 |
| **0.50 m** | **+0.730** |
| **1.00 m** | **+0.737** |
| 2.00 m | +0.686 |
| 3.00 m | +0.636 |

Per-sample `dz/ds` at 5 Hz is **useless** — it reports a median \|grade\| of **10.5°** on
terrain whose true median slope is 2.9°, because gait bob (~5 cm per ~0.5 m stride) swamps
the grade. Under 0.25 m the window is inside one gait cycle and the bob leaks straight
through; over ~2 m it smears real slope changes. **0.5–1.0 m is the operating point.**

**MEASURED 2 — the body does carry the grade: `gt_pitch = −0.736·grade − 3.46°`.** The
body follows ~**74%** of the terrain grade (sign per REP-103: +grade ⇒ nose up), so pitch is
a legitimate grade proxy *if you can measure pitch*. The constant −3.46° nose-up offset is
consistent with the stance-sag asymmetry the same report shows — hind lower-leg joints track
worst (lh 6.93°, rh 6.51°) vs front (4.74°, 4.65°), so the rear sags and the nose rides up.
That is `docs/SLOPE_POSTURE.md` §2's argument showing up in an independent measurement.

**MEASURED 3 — the accel-only attitude estimator does NOT recover the grade.** This is what
`terrain_adapt.py` runs on. Causal low-pass of the IMU pitch, with and without its ±35%
magnitude gate:

| τ | RMS vs gt_pitch | corr | RMS vs grade | corr |
|---|---|---|---|---|
| 0.5 s | 7.35° | +0.25 | 9.29° | +0.03 |
| 1.0 s | 6.50° | +0.23 | 8.29° | +0.05 |
| 2.0 s | 5.67° | +0.20 | 7.23° | +0.09 |

The gate (which keeps 52% of samples) removes the bias (+0.63° → −0.26°) but **does not
improve correlation**. The damning number is the per-sample SNR: instantaneous accel pitch
has p90 **29.3°** against a ground truth of 3.0°. **CAVEAT — this is measured on 5 Hz
decimated, which aliases 2–4 Hz gait content; the real node low-passes at 200 Hz, so treat
the RMS as pessimistic.** But scale-free correlation never exceeds +0.25, and no amount of
averaging fixes a signal that is not there. **Consequence: on this world `terrain_adapt.py`
is posturing against something much closer to noise than to slope.** Before A/B-ing
`--adapt` again, either give it a gyro/accel complementary filter (the gyro carries pitch
*rate* at full bandwidth; the accel only needs to stop long-term drift) or feed it a
proprioceptive plane fit. Testing that needs a log with gyro y — `timeseries.csv` has none,
so this is NOT settled, only bounded.

**MEASURED 4 — slip has an exact, ground-truth-free observable that CHAMP already computes
and throws away.** The least-squares twist solver (vendored edit #6) fits
`−dr_i/dt = v + ω×r_i` over stance feet. In centroid-reduced form it keeps only the
*perpendicular* part (`num = Σ(p'_x q'_y − p'_y q'_x)` → ω) and never forms the *radial*
part. Two feet on the ground **cannot change their separation**, so

```
sep_rate = Σ(p'_x q'_x + p'_y q'_y) / sqrt(Σ|p'|²)      [m/s]
```

is identically zero for planted feet and non-zero exactly when the stance constraint breaks.
A trot has 2 stance feet = 4 equations in 3 unknowns, so there is **exactly 1 DOF of
redundancy at every sample** — enough for this one scalar. Offline on `leg_odom_model.py`:

| condition | leg-odom \|v_err\| | sep_rate median |
|---|---|---|
| straight 0.25 / fast 0.50 / turn wz=0.4 / spin wz=0.8 / crab vy=0.15, **no slip** | 0.000–0.001 | **0.0000** |
| one stance foot slips 0.02 m/s | 0.009 | 0.0113 |
| one stance foot slips 0.05 m/s | 0.023 | 0.0282 |
| one stance foot slips 0.10 m/s | 0.045 | 0.0556 |
| one stance foot slips 0.20 m/s | 0.090 | 0.1084 |
| one stance foot slips 0.10 m/s **laterally** | 0.041 | 0.0396 |
| **all** stance feet slip together, 0.05–0.20 m/s | 0.050–0.200 | **0.0000** |

Linear in the slip rate, `sep_rate ≈ 1.2 × ` the resulting leg-odom velocity error, and a
**zero false-alarm floor** across every no-slip gait/twist condition tested — including
turning and spinning in place, which is where the old bearing-sum estimator generated its
false yaw rate. Compare the deployed 8-feature MLP: held-out R² for \|e\| is **0.182**, and
**0.063** without the `cmd_minus_*` features that do not transfer to hardware.

**The hard limit, and it is the same one as yaw.** *Uniform* slip — every stance foot
sliding together — leaves `sep_rate` at exactly **0.0000** while the velocity error is the
full slip rate. Inter-foot consistency can only see **differential** slip. Uniform slip is
indistinguishable from motion without an external anchor, which with GPS off does not exist.
So `sep_rate` is a genuine slip *detector*, not a slip *correction*, and it does not repeal
[the unobservability argument](#headline-2-yaw-is-exactly-unobservable-in-the-eskf-with-gps-off).

**What this costs to implement** (none of it done):
1. `sep_rate` is **one extra accumulator in the loop that already exists** in
   `champ/include/champ/odometry/odometry.h` — the `px,py,qx,qy` deviations are in hand;
   add `rad += px*qx + py*qy` beside `num`, publish `rad/sqrt(den)`. Vendored edit, additive.
2. **Measure its floor on `flat.sdf` first.** The 0.0000 above is a noiseless model; joint
   noise, leg compliance and body flex will put a real floor under it. The flat world's rigid
   no-slip floor is exactly the instrument for that — it is the one place the true answer is
   known to be zero.
3. Only then feed it to `R_leg`. It is a far better-shaped input than the current feature
   set, and it would also answer the open ablation from the handoff above (is the MLP doing
   anything beyond a constant?) — a detector that is provably zero when nothing is slipping
   cannot be a calibration in disguise.
4. Grade: a plane fit through the last cycle's touchdown points (`foot_from_base()`, already
   computed) gives the terrain normal in the base frame from proprioception alone — no accel,
   no drift, and it is precisely the geometric signal CHAMP is blind to.

Scripts for both measurements are throwaway (scratchpad, not committed); the numbers above
are reproducible from `run_report/` and `leg_odom_model.py`.

---

### YAW, terrain-only term: the filter integrates ω_z as ψ̇, which is FALSE on a slope (2026-08-25)

Found while asking what is terrain-*specific* about the estimation error. `predictImu`
(`eskf_core.cpp:71`) advances heading as `ψ += (gyro_z − b_g)·dt`, and with
`attitude_source: gravity_lp` the node hardcodes `roll_ = pitch_ = 0`
(`eskf_node.cpp:254`). So the body-frame gyro z is used as the WORLD yaw rate. The
correct ZYX relation is

```
ψ̇ = (ω_y·sin φ + ω_z·cos φ) / cos θ
```

On flat ground φ=θ=0 and `ψ̇ = ω_z` exactly — which is why this never showed up before.
On `terrain.sdf` it does not hold. Measured on the archived square (`timeseries.csv`,
n=536 @ 5 Hz, `|gt_wz|<1.5`):

| quantity | value |
|---|---|
| \|roll\| | median **2.3°**, p90 7.6°, max 20.2° |
| \|pitch\| | median **2.9°**, p90 8.9°, max 16.2° |
| ∫(ω_z·cos φ/cos θ − ω_z)dt over the run | **−3.1°** |

**−3.1° from the second-order term alone**, i.e. *larger* than the gyro scale error
(+2.5°) that the 2026-08-22 budget lists as worth fixing. It is also **not zero-mean**:
`cos φ/cos θ − 1` has a fixed sign wherever |φ|>|θ|, so it accumulates rather than
random-walking.

**The first-order `ω_y·sin φ` term is UNMEASURED and is the bigger unknown.** Nothing in
the workspace logs gyro x/y — `timeseries.csv` has `imu_pitch`/`imu_roll` (derived from
accel) but no rates, and `slip_features.csv` carries `gyro_wz` only. It cancels only if
ω_y is uncorrelated with roll; during a trot on a side-slope it need not be, since roll
and pitch-rate are both gait-locked. Upper bounds at median roll over this 116 s run:
26° per 0.1 rad/s of coherent ω_y. Do not quote those as estimates — they are what a
*fully* coherent ω_y would give, and the true figure is somewhere between 0 and that.

**Cheapest test, no sim time needed beyond one logged run:** add `gyro_x`/`gyro_y` to the
run-report series (or to `slip_features.csv`, which is already at 50 Hz and avoids the
5 Hz gait aliasing), then integrate `ψ̇` both ways over an archived run and difference
them. If it is worth >5°, tilt-compensating the yaw propagation is a ~3-line change in
`predictImu` and it is the only item on the yaw list that is *specific to terrain*.

Caveat: this needs a roll/pitch estimate to apply, and 2026-08-22 measured the accel-only
attitude at corr +0.25 vs truth — so it is blocked behind the same complementary filter
that `terrain_adapt.py` needs. That makes attitude the shared dependency of both, which
raises its priority above where §0 currently has it.

---

### GPS is a WORKING SENSOR now — the navsat `<stddev>` was in degrees (2026-08-28)

Asked to add GPS as a sensor. It was already wired end to end in the ESKF (`use_gps`,
`correctGps`, lat/lon->ENU); what was broken was the **sim sensor**, and the fix is one unit
conversion.

**The bug.** `unitree_go2_gazebo.xacro`'s navsat carried `<stddev>0.5</stddev>` with a comment
claiming "~0.5 m horizontal noise". gz-sensors adds that number straight onto latitude and
longitude **in degrees**, so it meant ~0.5 deg = **~55 km** of scatter. That is the whole
reason GPS was off by default.

**The fix** (vendored, additive): express it in metres and convert at the world datum
(lat 30.0444), using the SAME spherical earth model `eskf_node::gpsToEnu` uses (R = 6371 km):
1 deg lat = 111195 m, 1 deg lon = 111195*cos(30.0444) = 96269 m. SDF allows only ONE
horizontal noise for both axes, so the divisor is their geometric mean, 103459 m/deg.
`gps_noise_m = 0.5` therefore expands to `<stddev>4.83e-06</stddev>`.

**MEASURED, stationary robot, 400 fixes on `flat.sdf`:**

| axis | predicted | measured |
|---|---|---|
| latitude (N/S) | 0.537 m | **0.542 m** |
| longitude (E/W) | 0.465 m | **0.491 m** |
| altitude | 0.8 m (unchanged) | **0.805 m** |
| datum offset of the MEAN from the world origin | 0 | +0.02 m lat, +0.04 m lon |

Three things confirmed at once: the degrees hypothesis was right, the conversion is right, and
**the vertical channel was never affected** — altitude is natively metres, so there was no unit
confusion possible there. The mean landing within 4 cm of the world datum re-confirms the
long-standing note that the `/gps/fix` MEAN was always correct; only the noise was broken.

**Also fixed: the ENU datum was one noisy fix.** `gpsCallback` set `lat0_/lon0_` from the
FIRST fix, so that single sample's noise became a constant offset on every position the filter
ever reported — the one GPS error no amount of later fusion can average away. It now averages
`gps_datum_samples: 10` fixes (1 s at 10 Hz, offset down by sqrt(10)).

**ON BY DEFAULT since 2026-09-04 — still not A/B'd.** The ESKF now fuses IMU + leg odometry +
GPS out of the box: `use_gps: true` in `config/eskf_params.yaml`, `use_gps` default `true` in
`eskf.launch.py`, `USE_GPS=true` in `run_go2_teleop.sh`. `--gps` is kept as a no-op for
compatibility and **`./run_go2_teleop.sh --no-gps` (or `use_gps:=false`) restores the old
leg-odom-only arm** — which is what EVERY drift/yaw number recorded in this file before
2026-09-04 was measured under, so do not compare a new run against them directly. Two cautions
before anyone quotes a number:
1. §0's own rule applies — **never conclude from one square**; budget ~5+ runs per arm.
2. `run_report/timeseries.csv` has **no GPS columns**, so GPS quality is currently invisible in
   `REPORT.md`. Add `gps_x/gps_y/gps_err` before running the A/B, or the run cannot be
   diagnosed when it goes wrong.

### THE YAW FIX IS GPS POSITION, AND IT NEEDS NO NEW CODE (measured offline, 2026-09-04)

`scripts/yaw_observability.py` (new; same offline method as `leg_odom_model.py` and
`slip_yaw_experiment.py`) drives square_test's own 5 m route in the NumPy twin with an +8 %
gyro yaw-rate scale error injected at the corners, 40 seed-paired trials per arm:

| arm | mean \|final yaw\| | median | worst | mean over run |
|---|---|---|---|---|
| gps off (the archived baseline) | 23.41° | 23.17° | 56.52° | 11.46° |
| **gps on (shipped 2026-09-04)** | **2.00°** | **1.96°** | **4.10°** | **2.68°** |
| gps on + GPS course heading | 4.00° | 3.76° | 10.38° | 5.59° |

Better in **39/40** seeds, 11.7x lower mean final error. On a 300 s straight walk
(`--straight`, 20 seeds) it is 51.02° → 0.46°, better in 20/20.

**§0 HEADLINE 2 is not wrong — its precondition is gone.** Yaw is exactly unobservable
*with GPS off*. `correctGps` pins world position, which reaches ψ through `P(p,v)` (from
`F[PX,VX]=I·dt`) and `P(v,ψ)` (from `correctLegOdom`'s `H[:,PSI]`), so ψ is observable
whenever the robot is MOVING. Corners are the exception: turning in place has `v_world=0`,
so `H[:,PSI]=0` and the ZUPT carries no heading information — heading is corrected on the
sides, not in the corners.

⚠️ **Do NOT add a GPS course-over-ground heading measurement.** It measured *worse* (2.00°
→ 4.00°), and the reason is structural, not tuning: the course is computed from fixes that
were already fused as positions, so fusing it again double-counts the same measurement and
makes the filter over-confident in a lagged, window-averaged heading. Any GPS-derived
heading inherits this. (Clearing the window when turning is still required — gating on the
instantaneous turn rate alone lets it straddle a corner: 4.81° → 4.00°.) An independent
source (magnetometer/AHRS) would not have the defect, but on these numbers nothing needs one.

**Still to do: confirm in sim.** These are offline numbers with an exact truth model; they
say where to spend sim time, not what the robot will do. Budget ~5+ completed runs per arm
and add GPS columns to `run_report/timeseries.csv` first (there are none).

**First run with GPS on (2026-09-04, `--square --terrain`) proves only the plumbing.** Both
arms logged `gps=gps/fix(on)` and set the ENU datum from 10 fixes; then CHAMP fell over on the
FLAT start pad (slope max 0.0° under the whole path) 3 s after square_test began, 0/4 corners,
roll -180°, `/cmd_vel` never published. Its drift numbers are void by square_test's own rule.
No accuracy claim about GPS is supported by any run yet — see the report artifact
`GPS and the Heading Problem` (2026-09-04) for the full "what is / is not established" split.

**Why this is the big one.** Per §0, yaw is *exactly* unobservable with GPS off, and the
2026-08-28 decomposition puts 5.16 m of the 6.59 m position error in episodic slip that
proprioception structurally cannot see. GPS is the only absolute reference in this stack that
addresses either. It is the honest answer to "improve results without GPS": you mostly cannot,
and this is why.

⚠️ **One run was archived BY HAND before this work overwrote it** —
`run_archive/2026-08-28_terrain_square_baseline/`. The launcher still overwrites `run_report/`
every run and **nothing automates archiving yet**; every earlier session's square is already
lost this way (see the section below).

---

### The leg-odom speed bias is EPISODIC SLIP, not a scale error — do NOT re-fit `leg_odom_scale` (2026-08-28)

Re-measured the mean signed leg-odom error to put 2026-08-25's one-run `leg_odom_scale`
recommendation on more runs. **It could not be done across runs: `run_report/` is gitignored
and overwritten every run, so every earlier square is GONE.** n = 1 (a fresh
`--square --terrain`, 7889 rows, 147.7 s of fused updates, `|gt_wz|<1.5`), plus 2026-08-25's
numbers as an independent second point. Archive `run_report/` per run before this question
can ever get a real n.

**The headline number reproduces.** Time-weighted mean signed error of the *as-fused* `leg_vx`:
**+0.0446 m/s → +6.59 m** injected over the log (2026-08-25: +0.061 → +6.1 m). Same sign, same
order. Lateral: **−0.0181 m/s → −2.67 m** (2026-08-25: −2.3 m). Both prior findings stand.

**But the decomposition kills the fix.** Smoothed truth vs leg speed splits the run cleanly:

| regime | share of time | mean err vx | contributes | scale wanted |
|---|---|---|---|---|
| normal walking | 74 % | +0.0131 m/s | **+1.44 m** | **1.029** |
| slip episodes (t≈50–62, 122–158) | 26 % | +0.1343 m/s | **+5.16 m** | 0.318 |
| whole run | 100 % | +0.0446 m/s | +6.59 m | 0.837 |

**78 % of the position error comes from 26 % of the time.** During normal walking leg odometry
is nearly unbiased and the true correction is `leg_odom_scale ≈ 1.03` — i.e. the configured
1.111 is only ~8 % fast, worth ~1.4 m. During the slip episodes leg odom reports ~0.20 m/s
against ~0.11 m/s of truth; no scale factor describes that.

⚠️ **So 2026-08-25's ranked fix #1 ("the filter wants ~0.86, one param, biggest lever") is
withdrawn.** 0.837 zeroes the mean *on this run* by making normal walking under-report by 19 %
— trading an episodic error for a permanent one, fitted to n=1. Constant-scale sweep
(mean signed / RMS): 1.111 → +6.59 m / 0.1405; 1.000 → +3.92 m / 0.1280; **1.029 → the
normal-walking optimum**; 0.837 → 0.00 m / 0.1154; 0.800 → −0.89 m / 0.1137. RMS keeps falling
past the drift-optimal point, which is the tell that a single scale is the wrong object.

**Honest, transferable change: `leg_odom_scale` 1.111 → ~1.03**, justified on the 74 % of time
the gait is working. Worth ~1.4 m. The other 5.16 m is not a calibration problem.

**And the remaining 5.16 m is structurally invisible to proprioception.** During slip, `leg_vx`
reads ~0.20 both when the robot is moving and when it is not, so `cmd_minus_leg_vx` barely
moves — this is the same wall as the 2026-08-22 `sep_rate` result (uniform stance slip leaves
inter-foot consistency at exactly 0.0000). It is also the *right* shape for an `h`-side
scale/bias correction gated on a slip detector, and the wrong shape for `R` inflation — which
is §0 HEADLINE 0's conclusion, now with metres attached: 5.16 m of the 6.59 m sits in the term
`R` cannot touch. A detector needs an exteroceptive reference or a harsher-patch world where
slip is large enough to leave a proprioceptive trace.

**Lateral confirmed independently: `leg_vy` costs −2.67 m** (−1.54 m normal, −1.14 m slip).
Downweighting/dropping it (§0 HEADLINE 0 item (a)) is still free and still the best
cost/benefit item on the list — and unlike the scale, it needs no per-world fitting.

⚠️ **Data-handling correction.** `slip_features.csv`'s `leg_vx`/`leg_vy` are **POST**-`leg_odom_scale`
(`eskf_node.cpp:326` computes `vx`, `:332` logs it). 2026-08-25's table labels one column "raw"
and the other "after `leg_odom_scale` 1.111", differing by exactly 1.111x — so the "after"
column **double-applied the scale** and its slope 1.154 is spurious; 1.039 was already the
as-fused value. (This run: as-fused slope 0.990, mean ratio 1.234.) The section's conclusions
rest on the mean signed error, which was computed on the as-fused column and is unaffected.

---

### POSITION on terrain: the dominant term is a leg-odom SPEED BIAS, not heading (2026-08-25)

Asked "why is position estimation so bad on slip terrain". Decomposed the archived terrain
square (`run_report/timeseries.csv`, 561 rows, final error **3.759 m** over a 17.91 m path)
by splitting the integrated velocity error into a pure-rotation part and a body-frame part:
`v_est − v_gt = [R(ψ_err) − I]·v_gt + [v_est − R(ψ_err)·v_gt]`.

| term | contribution to the 3.75 m |
|---|---|
| heading-induced (yaw error rotating a correct velocity) | **1.32 m** |
| body-frame velocity error (rotation removed) | **2.86 m** — along-body 3.51, lateral 2.12, partly cancelling |

**So on this run heading is the MINORITY term.** `REPORT.md`'s boilerplate "position error
tracks yaw error ~1:1" is a static sentence, not a per-run measurement — do not read it as one.
Corroboration: the estimated path is **7.8 % longer** than truth (19.31 vs 17.91 m), which a
heading error cannot produce.

**Cause, measured on the same run's `slip_features.csv` (n=4815 fused updates, `|gt_wz|<1.5`):**

| quantity | raw `leg_vx` | after `leg_odom_scale: 1.111` |
|---|---|---|
| least-squares slope vs `gt_vx` | 1.039 | **1.154** |
| mean ratio | 1.287 | — |
| median | 0.203 vs truth 0.149 | — |
| **mean signed error** | — | **+0.061 m/s → +6.1 m over the 100 s log** |

Speed-magnitude ratio `|v_leg|/|v_gt|` = **1.095** (1.092 moving-only). The raw injection
(6.1 m) exceeds the observed error because `K < 1` and the square's four headings partly
cancel it — but the sign and the order of magnitude are unambiguous: **leg odometry runs
FAST on terrain and `leg_odom_scale: 1.111` makes it faster.**

⚠️ **This contradicts §0 HEADLINE 1's "least-squares scale 0.95, so `leg_odom_scale: 1.111`
is vindicated on terrain."** Two reasons, both worth internalising:
1. Different runs. Run-to-run variance in the leg-odom scale is real and unquantified.
2. **The least-squares slope is the WRONG statistic for position drift.** It is dominated by
   the high-speed samples; position integrates the **MEAN** error. Here the slope says 1.039
   while the mean ratio says 1.287. Always quote the mean signed error when the question is
   drift.

**`leg_vy` is noise fused at σ=0.10 as if it were a measurement.** Slope vs truth **0.118**,
corr **+0.10**, `std(leg_vy)` 0.124 > `std(gt_vy)` 0.106 — and its mean is **−0.023 m/s**,
which integrates to **−2.3 m** of lateral drift over the log. This reproduces §0 HEADLINE 0
point 5 (`leg_vy/gt_vy = 0.089`) on an independent run and shows what it costs in metres.

**Why nothing in the filter catches it.** The bias is in the only velocity anchor: GPS is off,
`accel_noise: 5.0` deliberately guts the IMU, so there is no second opinion. A velocity bias
integrates straight into position with gain 1. And it is *structurally* invisible — the
2026-08-22 `sep_rate` measurement shows uniform stance slip leaves inter-foot consistency at
exactly 0.0000 while the velocity error is the full slip rate.

**The slip model cannot fix this and is not shaped to.** It only inflates `R`, which (a) moves
`K` from 0.980 to 0.926 across its entire output range, and (b) de-weights a *biased* source
toward free-running integration rather than toward a better one. An `h`-side correction (a
scale/bias on `v_leg`) is the right object for a bias; `R` is the right object for a variance.

**Ranked, and it is cheap:**
1. **Re-measure `leg_odom_scale` per world from the MEAN, not the slope** — on this run the
   filter wants ~0.86, not 1.111. One param. Biggest single lever on terrain position error.
2. **Make `R_leg` anisotropic and treat `vy` as near-uninformative** (σy → 0.3–0.5, or drop
   the lateral channel). Already ranked (a) in §0 HEADLINE 0; this quantifies it as ~2 m.
3. Then the `sep_rate` detector, which is the only thing that can separate "leg odom is fast
   because the gait is mis-solved" from "leg odom is fast because the feet are sliding".

Scratchpad scripts only; every number reproduces from the archived `run_report/`.

---

### TOOLING — the live plot now shows `/odom/raw` itself (2026-08-19)

The plot compared three *filtered* curves and never showed the measurement they all come
from. `plot_trajectory.py` now subscribes to `/odom/raw` as well and adds two things:

* **A fourth XY curve: leg odometry dead-reckoned.** Not its pose field — that is the
  known-broken `vel_dt` value — but its TWIST integrated in the plotter
  (`p += Rz(psi_mid)*v*dt`, `psi += wz*dt`, midpoint heading), anchored on the ground-truth
  pose at the first sample so it starts co-located with everything else. This is the
  open-loop "leg odometry alone" baseline the ESKF exists to beat; the gap between it and
  the two ESKF curves is what the IMU + covariance model actually buy. It gets an error
  trace in the middle panel like any other arm.
* **A third panel: the raw body twist**, `leg v_x`/`omega_z` against the ground-truth
  `v_x`/`omega_z`, with the **degenerate (all-zero) sample share in the title**. That
  number is the live version of the `leg_odom_gate_degenerate` statistic — previously only
  visible post-hoc in `REPORT.md`/`timeseries.csv`.

Details that matter:

* **The curve is UNSCALED by default** (`--leg-scale 1.0`), so it runs ~10 % short of the
  estimates on purpose: the filter applies `leg_odom_scale: 1.111` to undo CHAMP's
  `odom_scaler`. Pass `--leg-scale 1.111` for a like-for-like comparison. The default is
  raw so the plot cannot silently drift out of sync with `eskf_params.yaml`.
* `--no-leg` drops both the curve and the panel; panel count is now 1-3 depending on
  `--no-truth`/`--no-leg`.
* Anchoring waits up to 5 s for ground truth, then falls back to the origin, so the curve
  still appears when the truth bridge is down.
* **Never difference two clocks.** The dead-reckoner uses `header.stamp` (sim time) and
  falls back to the wall clock only for unstamped messages — and it *skips* the step where
  the source changes. Without that guard the first stamped sample after an unstamped one
  produces a huge negative `dt`; the offline test caught exactly one lost integration step
  from it.

Verified offline (no sim): a straight 0.5 m/s twist integrates to the exact expected
distance and anchors on the truth pose; a pure yaw rate rotates without translating and
`leg_scale` correctly does **not** touch omega; degenerate samples are counted and move
nothing; the wall-clock fallback and the anchor timeout both work; all three panels render.
**Not yet seen against the live sim.**

---

### ⏱ SESSION HANDOFF — 2026-08-11 (previous)

Branch `fix/sim-readiness-guard-and-drift-baseline`, pushed through `df74c58`, clean tree.
Newer entries in this section are ordered oldest-first, so read this block, then jump to
"BOTH estimator arms now run in ONE run" and "FIRST dual-arm terrain square" below.

**What changed this session**

1. **Both estimator arms run simultaneously** off one sensor stream — `eskf_node` →
   `/eskf/odom` (fixed `R_leg`) and `eskf_slip_node` → `/eskf_slip/odom`
   (`use_slip_model:=true`). The slip model is now an A/B *inside* one run, which is the
   only version of that comparison worth anything here. `eskf.launch.py slip:=true`;
   `run_go2_teleop.sh` does it by default (`--no-slip` reverts); `plot_trajectory.py`
   draws three curves; `REPORT.md` scores both arms. (`deeaef0`)
2. **First dual-arm terrain square ran and completed all four corners** — baseline 2.591 m
   / 2.6° yaw, slip-adaptive 6.121 m / 13.6°. (`9edfac6`)
3. **That result was then cut down to size.** `scripts/slip_yaw_experiment.py` (offline,
   40 seeds) shows inflating `R_leg` lowers the *probability* of recovering from a heading
   kick (16/40 → 10/40) but leaves the final error a **coin flip** (worse in ~19/40). The
   earlier "inflating R_leg costs yaw stability" was **retracted**. (`b972133`)
4. **`slip_score` is now recorded** (`/eskf_slip/slip` → CSV column + a `REPORT.md` line
   with the implied `R_leg` inflation). (`9edfac6`)
5. **The estimator is fully derived in `src/go2_eskf/README.md`** — physics, `F`/`H`
   Jacobians with the leg-odom `H` worked out, `Q`/`R`, Joseph update, flow chart, each
   equation carrying a `file:line`. `DESIGN.md` §2's stale `Q` (per-sample convention)
   fixed at the same time. (`df74c58`)

**Next steps, in order**

1. **Run `--terrain --square` again and read the slip-score line in `REPORT.md` first.**
   A narrow score range = the model de-weights leg odometry everywhere rather than
   detecting slip. That one line decides whether the model or the `R`↔yaw coupling is at
   fault. **It has never been captured live** — the metric postdates the only dual-arm run.
2. Get the dual-arm comparison to ~5 runs per arm. n=1 today; §0 variance rules apply.
3. `--adapt` and `--stiff` (the two slope fixes) are still **unvalidated in sim**. A/B one
   at a time, never together.

**Do not re-derive**: leg odometry's `H` has a nonzero ψ column proportional to speed, so
it *does* pull on heading — only `δv = δψ·(-v_y, v_x)` is blind, and at rest there is no
heading information at all. Full derivation in `src/go2_eskf/README.md` §4.

---

**PREVIOUS (2026-07-28 — CHAMP leg odometry re-derived as a least-squares body twist; two
defects found and fixed OFFLINE with zero run-to-run variance. Plus the observability
result that explains why six runs of filter tuning did nothing.)**

Triggered by another square screenshot (final **2.735 m**, max 2.786 m). The error ramps on
the STRAIGHT legs and plateaus through the corners, and the estimated square is rotated and
shrunk relative to truth — the same heading signature as every earlier run.

### HEADLINE 1: the yaw-rate error is a DERIVATION BUG in CHAMP, proven without the sim

§0's previous entry ended blocked: "the variance looks SENSOR-level, chase it there, but a
square run is too noisy to resolve anything." That block is now lifted — **the sensor error
is analytic, so it can be measured with no Gazebo, no GPU and no variance at all.**
`src/go2_eskf/scripts/leg_odom_model.py` (new, permanent, `PASS`/`FAIL` self-checking)
simulates a Go2 trot from an EXACTLY known body twist, generates the base-frame foot
positions `champ::Odometry::getVelocities` would see, and runs the estimator on them.

| estimator | vx/truth | wz/truth (turn) | wz std, truth wz=0 |
|---|---|---|---|
| `bearing` (as vendored) | 0.821 | 0.923 | **0.0675 rad/s** |
| `lsq` (twist solve, touchdown fused) | 0.913 | 0.923 | 0.0951 |
| `lsq+td` (twist solve + touchdown gate) | 1.000 | 1.000 | 0.0000 |
| `champ_lsq` (**shipped**) | 0.900 | 1.000 | 0.0000 |

`0.900` is exactly `odom_scaler`, which `leg_odom_scale: 1.111` already undoes — so the
shipped estimator recovers the twist **exactly** (worst deviation 5.3e-06).

**Two independent defects, isolated by that ablation:**

1. **Yaw rate was derived from a per-foot BEARING sum.** `vel.angular.z` came from
   `theta_sum`, the change in `atan2f(X, Y)` of each stance foot, attributing the *whole*
   bearing change to rotation. But body TRANSLATION also swings a planted foot's bearing:
   `d(theta)/dx = y/(x^2+y^2)` = 2.36 rad/m on Go2 geometry, i.e. **±0.59 rad/s per foot at
   0.25 m/s** — larger than the 0.4 rad/s turn signal itself. Left and right feet give equal
   and opposite terms that cancel *only* when the diagonal stance pair is exactly symmetric.
   So the reported yaw rate was a difference of large near-cancelling numbers whose residual
   moved with gait phase and contact timing. Measured cost: **±0.0675 rad/s (±3.9°/s) of
   gait-synchronous false yaw rate while walking dead straight.**
2. **Touchdown samples were fused.** `prev_foot_contacts_` was maintained and **never read**
   (the same "computed then unused" pattern as `total_contact` in §5.5). On the sample a
   foot lands, its position delta spans the SWING, not the stance, and that was counted as
   body motion. Cost: **~9% of forward speed** and the 0.923 turn-phase yaw-rate under-read.

**Defect 2 explains the long-standing open puzzle** "`leg_odom_scale: 1.111` does not appear
to reach the estimate" (§0, 2026-07-26 late). It reaches it fine. There were simply TWO
multiplicative errors: `odom_scaler` 0.9, which `leg_odom_scale` undoes, and a ~0.91
structural error, which it does not. 0.9 x 0.91 = 0.82, and the model reproduces exactly
that (0.821). **Close that item.**

**Defect 1 quantitatively reproduces the measured `correctGyroBias` residual.** The model
predicts `wz_leg` under-reads a wz=0.4 turn by 7.7%, so `gyro_wz - wz_leg` should sit near
+0.031 rad/s during a turn and near zero straight. **Measured live: -0.004/-0.006 straight,
+0.045/+0.052 turning.** Same sign, same order, same turn-correlation — the mechanism §0
measured but could not explain is this derivation bug. (The live numbers are larger; the
model uses idealised contact timing, so treat it as the mechanism, not the magnitude.)

### HEADLINE 2: yaw is EXACTLY UNOBSERVABLE in the ESKF with GPS off

This is why six square runs of filter tuning moved nothing, and it should stop anyone from
trying again. `correctLegOdom` predicts `h = Rz(-psi) * v_world`. Differentiating,

    dh = Rz(-psi) * (dv - J*v*dpsi),  J = [[0,-1],[1,0]]

so **any** perturbation with `dv = dpsi * (-v_y, v_x)` leaves `h` unchanged: rotating the
heading and rotating the world velocity together is invisible to leg odometry. Leg odom
constrains the body-frame velocity only. Heading is observable solely through whatever pins
`v_world` in the WORLD frame — the IMU accel (gutted by `gravity_lp`, which discards
sustained horizontal accel) or GPS position (off in sim). With GPS off there is no absolute
heading information anywhere in the system, so `psi` runs pure open-loop on
`integral(gyro_z - b_g) dt`.

**Consequences, and they are sharp:**
- The `eskf_core.cpp` comment claiming the old `Q(PSI,PSI)` convention made the filter
  "refuse the yaw information leg odometry carries in its body-frame vy residual" is
  **wrong** — there is no such information to refuse. Growing `P(psi)` cannot help;
  nothing corrects psi. (Keep the `Q` fix anyway: it is the correct convention.)
- No `R`, `Q`, or gating change can bound heading drift. Only two things can: **reduce the
  yaw-rate disturbance** (what this session did) or **add an absolute heading reference**
  (fix the navsat `<stddev>` and enable GPS, or fuse an AHRS/magnetometer yaw).
- Position error tracking yaw error 1:1 is not a coincidence to be tuned away; it is the
  structure of the problem.

### What was SHIPPED (vendored edit #6, `champ/include/champ/odometry/odometry.h`)

`getVelocities` now solves the rigid-body twist directly. A planted foot is fixed in the
world, so its base-frame position obeys `-dr_i/dt = v + w x r_i`. Two equations per stance
foot, three unknowns, so >=2 stance feet over-determine it and least squares is the correct
estimator. Solved in centroid-reduced closed form (no matrix inverse — champ targets
embedded):

    w = sum(r'_x q'_y - r'_y q'_x) / sum(|r'|^2),   v = q_bar + w x r_bar

with `q_i = -dr_i/dt` and primes deviations from the stance mean. A foot contributes only if
it was in contact on the PREVIOUS sample too. Falls back to the previous value when the twist
is unobservable (one usable foot, or a trot swapping both diagonal pairs at once).

**Deliberately NOT changed: the `allFeetInContact() || noFootInContact()` early return.**
Least squares would handle all-four-planted best of all (8 equations), but those literal
zeros are the no-information FLAG that `eskf_node`'s degenerate gate and ZUPT path key on
with an exact `== 0.0` test. Routing them through the solver would emit small non-zero noise
instead, silently killing the standing ZUPT — the cleanest gyro-bias observation available.
Measured 0/23,000 zeros while walking, so leaving it costs nothing in motion.

### Status: offline-proven, LIVE-UNPROVEN — do not quote a drift improvement yet

- **Green:** `leg_odom_model.py` PASS (5.3e-06), 16/16 GTest, C++=NumPy **2.498e-15**,
  `champ`/`champ_base`/`go2_eskf` all rebuild clean.
- **No sim run has been done.** Per this file's own rule, nothing here may be claimed as a
  drift improvement until measured — and given 4–97° run-to-run variance, **not from one
  square run either.**
- **Predicted, so it can be falsified:** `probe_leg_odom.py` should now show
  `wz_leg/wz_truth` ~1.0 (was 0.734/0.901), `vx_leg/vx_truth` ~0.9 in BOTH phases (was
  0.894/0.903 straight vs 0.773/0.781 turning), and the `gyro_wz - wz_leg` residual roughly
  EQUAL straight vs turning (was -0.005 vs +0.05). If the residual equalises, the false
  turn-only bias injection is gone.
- **`leg_odom_scale: 1.111` is now correct** rather than under-compensating. Do not retune it
  before re-measuring — the ~11% shortfall it was chasing was the touchdown bug.
- If the residual equalises, `bias_update_max_wz` loses its motivation (it was a symptom
  guard for exactly this). Keep it for now — it is physically right — but it becomes an
  A/B candidate rather than a fix.

### Terrain world for the slip model (2026-07-28, second half of the session)

Phase 3's blocker was never the model — it is that **the flat world produces no slip**
("No slip exists: rigid no-slip floor", assessment below). `--terrain` fixes the sim
conditions:

    ./run_go2_teleop.sh --terrain --square      # uneven ground + mu=0.08 patches
    ./run_go2_teleop.sh --square                # unchanged flat baseline (default)

`src/go2_eskf/worlds/terrain.sdf` is GENERATED by
`src/go2_eskf/scripts/make_terrain_world.py`: a 40x40 m fractal heightmap (five octaves,
finest at 0.9 m so it perturbs foot contacts, not just the view) plus four 3x3 m `mu=0.08`
patches on the midpoint of each square leg. **`flat.sdf` is untouched and remains the
default — reverting is dropping the flag, nothing else.**

Design points worth keeping:
- **Flat start pad at elevation 0** (r=1.5 m, blended to 4.0 m). The robot spawns exactly as
  over `flat.sdf` (`world_init_z` 0.375), so flat-vs-terrain runs start from an identical
  pose and are comparable. VERIFIED in-sim: a probe rests at exactly z=0.000 on the pad.
- **`--relief` is the difficulty knob.** Square-path median/max slope: 0.6 -> 3.2/14.4 deg,
  **0.7 (default) -> 3.7/16.6**, 1.0 -> 5.2/23.1, 1.6 -> 8.4/34.3. Defaulted LOW on purpose:
  the gait already falls occasionally on flat ground, and terrain the robot cannot cross
  yields zero slip data. Raise it gradually.
- The generator models the QUANTISED image exactly as gz will, so patch placement is
  millimetre-accurate. VERIFIED: predicted vs measured terrain height agreed to **0.2 mm**
  at two independent probe points.

**gz heightmap facts, all MEASURED here (gz sim 8.11) — none of this is in the docs:**
- Heightmap collision **works** under `bullet-featherstone` (the engine these worlds use).
- `<surface><friction><ode><mu>` **is** honoured: an identical box slid off a 10 deg
  `mu=0.08` ramp (x -> 27.4 m) and did not move on a `mu=1.0` one (x -> 6e-5 m). So the slip
  patches produce genuine slip, which is the whole point.
- Image **column -> +X, row 0 -> +Y** (max Y).
- **Elevation = pixel / (MAX pixel in the image) * size_z** — gz normalises by the image's
  own maximum, NOT by 255. Measured: pixels 220/20 with `size_z=1.5` gave 1.500/0.136 m,
  i.e. 20/220*1.5. The generator always emits a 255 max so the scaling is plain `pixel/255`.
- A heightmap `<uri>` resolves **ONLY** as an absolute `file://` path. Tested and FAILED:
  a path relative to the world file, and `GZ_SIM_RESOURCE_PATH`. Both fail SILENTLY — gz
  loads a world with no ground and the robot falls forever. Hence `terrain.sdf` is
  machine-specific (re-run the generator after moving the workspace) and the launcher
  verifies the referenced PNG exists before starting.

**FIRST TERRAIN RUN (2026-07-29): the world works, but the run was VOID — and it
exposed a real bug in `square_test.py`.** Reported final 8.776 m / max 8.799 m.
**Do not record that as a terrain drift number.** What actually happened:

- The heightmap, its texture and the tilted `mu=0.08` patches all render and simulate
  correctly, and the robot walks on them. The world itself is validated.
- Ground truth completed leg 1 along y=0, wandered near (6,+1.4), then **stopped at
  about (9,-3)** — the robot went down. Meanwhile the estimate carried on to (9,-12).
- Error was ~flat until t≈20 s, reached ~3 m during leg 1 (`slip_patch_0` sits at
  (5,0), right on it), plateaued ~3.5 m, then climbed steeply from t≈95 s to 8.8 m.
- That final climb is **phantom motion**: ~50 s of commanded driving against a prone
  robot. 9 m of estimate travel over 50 s ≈ 0.18 m/s of false velocity, exactly what
  scrabbling feet feed a leg-odometry estimator (a foot sliding backwards under a
  stationary base is indistinguishable from the base moving forwards).

**Root cause — `STAND_Z` is an ABSOLUTE world-z test and so is terrain-blind.** It
compares base z against 0.18 m, which only means "prone" when the floor is at z=0. On
`terrain.sdf` the robot walks at elevations up to 0.56 m, so a robot collapsed at
elevation 0.4 m reads z≈0.46 and the fall is never detected — the test drives a prone
robot indefinitely and reports the resulting runaway as drift. **Every drift number
from an uneven-ground run predating this fix is suspect for the same reason.**

Fixed by splitting the two jobs the check was doing:
- **Before the robot has ever stood**, the absolute test is kept and is correct: every
  world spawns it on ground at elevation 0 (`terrain.sdf` has a flat start pad for
  exactly this reason). This preserves the original "controller never activated"
  diagnostic unchanged.
- **After it has stood**, three terrain-agnostic tests, any of which aborts:
  absolute z (still valid on flat, harmless elsewhere); **body tilt > 60 deg** from
  the ground-truth quaternion (`acos(1 - 2(qx^2+qy^2))`, independent of terrain
  height — 60 clears the 38.9 deg a *standing* robot reaches at `--relief 1.6`); and
  a **stall** — commanded to move for 6 s while ground truth moved < 15 cm AND turned
  < 0.15 rad. The stall test is what catches a belly flop, which can be perfectly
  level and so invisible to a tilt test.
- Unit-tested with no sim (8 cases): walking, in-place turning, a 5 cm/s crawl and a
  yaw wrap across +-pi do NOT abort; a fall while commanded, and a 2 cm/s crawl, do;
  a fall while NOT commanded does not (we may be deliberately holding zero).

**SECOND TERRAIN FINDING (2026-07-29): the robot got wedged, and BOTH causes were
defects in the generated world, not robot limitations.**

1. **A slip patch was a 27 cm ICE CURB.** Each patch was one 3x3 m rigid plate tilted
   to the gradient at its centre. Laid on curved terrain such a plate only touches
   near the middle and its edges float — measured **11.7 / 13.1 / 18.9 / 27.0 cm**
   above the ground for the four patches. 27 cm is taller than the Go2 is (~22 cm),
   and CHAMP's blind gait has no step-up reflex.
   **Fix:** build each patch from small tiles (`--patch-tile`, default 0.40 m), each
   at its OWN local height and gradient. Worst step onto a patch drops to **2.8 cm**,
   worst tile-to-tile seam to 1.6 cm, and terrain never pokes above a tile (0.0 cm) —
   all below the terrain's own fine roughness. Tile-size sweep (step onto patch /
   seam / poke): 1.00 m -> 6.5/3.1/4.2 cm, 0.50 -> 4.3/2.1/0.9, **0.40 ->
   2.7/1.6/0.6**, 0.30 -> 2.1/0.9/0.0. 0.40 m gives 64 tiles per patch, 256 total;
   the world still loads and simulates clean.
2. **`mu=0.08` made two patches physically unstandable.** A foot holds only if
   `mu > tan(slope)`, and walking needs roughly twice that. `slip_patch_2` and
   `slip_patch_3` sit on 6.5 and 5.5 deg (up to 9.7/10.3 deg across the footprint),
   needing mu > 0.114 / 0.096 just to STAND. At 0.08 the robot slides no matter what
   it does.
   **Fix:** default `--patch-mu` 0.08 -> **0.30** (gravel/wet grass: real slip against
   terrain's ~1.0, still traversable), and the generator now WARNS per patch when
   `mu < 1.5*tan(max slope on the footprint)`, printing the mu actually needed.

**Lesson worth keeping: a rigid plate cannot represent a surface property on uneven
ground.** Anything laid on a heightmap has to conform to it or it becomes an
obstacle, and any friction value has to clear the slope it is placed on. Both are
now checked and reported by the generator rather than discovered by a wedged robot.

**Also fixed: the generated SDF was not well-formed XML.** XML forbids `--` inside a
comment and the header comment was full of CLI flags. gz's TinyXML2 accepted it, so
it went unnoticed; `xml.dom.minidom` rejects it. The command/knobs now live in a
plain-text sidecar `terrain_params.txt` (which also records the slope/step numbers
for the run), and the generator validates its own output with a strict parser and
aborts rather than writing bad XML.

**What terrain does to the ESKF — expect these, they are not bugs:**
- `correctVerticalVel(0, ·)` is a `vz≈0` pseudo-measurement. Climbing at 0.25 m/s on a
  10 deg slope gives a true `vz` of 0.043 m/s, inside `vz_zero_noise: 0.3`, so it is
  tolerable at the default relief — but it becomes a real bias as `--relief` rises. Loosen
  `vz_zero_noise` before blaming the filter.
- The leg-odom twist solve (§5.6) is **planar** — it uses only foot X/Y. On a slope it
  under-reads along-slope speed by roughly `cos(pitch)` (1.5% at 10 deg, 6% at 20 deg).
  Real, expected, and exactly the kind of error the slip model is meant to flag.
- **Still open before Phase 3 can be claimed:** `contact_frac_` is dead (never assigned, so
  the slip feature is a frozen 1.0 — needs a `/foot_contacts` subscription), and the model
  must be RETRAINED on terrain logs. The existing weights were trained where no slip exists,
  so they learned to fire on gait tracking error; do not carry them over. Enable with
  `use_slip_model: true` + `slip_model_path` once retrained.

### FIRST COMPLETED TERRAIN SQUARE + slip is REAL but NOT DETECTABLE (2026-08-02, 15:41 run)

`./run_go2_teleop.sh --terrain --square`, **stock gains, no `--adapt`**, on the re-cut
11.3 deg-capped terrain. 257.6 s, 46.34 m, all four corners:
(0,0) -> (9.74,+0.08) -> (9.41,-9.79) -> (0.46,-7.58) -> (0.03,-0.22). **No fall, no
stall.** The previous terrain run died at 2.55 m. **The terrain re-cut ALONE unblocked
locomotion — `--stiff` and `--adapt` were not needed and remain unvalidated.**

Estimator: ATE mean **1.94 m**, max **3.22 m**, yaw mean 10.9 / final 15.0 deg. Final
position error 0.30 m (0.66% of path) is MEANINGLESS — the square returns to its start and
a heading-rotated trajectory comes back with it. **Quote ATE/max, never final.** 15 deg ->
~3 m sits exactly on the historical yaw-vs-position line, so this run is in-family.

Gait: stance duty 60% and symmetric (59.9/60.8/60.8/59.9). Joint tracking error mean
**9.31 deg**, max 21.9 -> ~16 Nm -> ~3 cm of continuous foot sag at p=100. Worst joints are
the **REAR calves** (rh 6.79, lh 6.41) vs front (rf 4.90, lf 4.78) = **1.4:1 rear bias**,
against the 1.6:1 load transfer SLOPE_POSTURE.md section 3 predicts. Best evidence yet that
`--stiff` is worth a run. Ground-truth attitude |pitch| median 2.3 max 14.8 deg.

**SLIP IS REAL, and measured.** Path crossed the mu=0.3 patches for 66.8 s of 228.8 s
commanded (29%). Signed leg-odom velocity bias vs truth:

    OFF patch  -13.3 mm/s   (under-reads)
    ON  patch   +5.7 mm/s   (OVER-reads)   <-- the sign FLIPS

That flip is the stance-foot assumption breaking: a foot sliding backward under load is
read by forward kinematics as body translation that never happened. Accumulated, the
on-patch segments contributed +0.38 m where the off-patch bias would have given -0.89 m,
so **slip injected ~1.3 m of position error — ~40% of the 3.22 m peak, from 29% of the
run.**

**BLOCKER (negative result): slip is NOT detectable from any currently-logged onboard
signal.** Patch vs non-patch separation, Cohen's d:

    feet in contact       2.000 vs 2.000    d = 0.00   <-- information-free
    joint tracking error   9.73 vs 10.01    d = -0.11
    |accel|               14.04 vs 13.22    d = +0.12
    terrain slope          2.42 vs  2.39    d = +0.02
    |leg wz|              0.405 vs 0.571    d = -0.44  (confounded by turning)
    |leg vx|              0.187 vs 0.170    d = +0.18

`feet in contact` is EXACTLY 2.000 on and off. CHAMP's `/foot_contacts` is a gait-phase
estimate from the trajectory planner, not a contact sensor, so **`contact_frac_` — the
planned slip feature, currently dead code frozen at 1.0 — cannot work even once it is
wired up.** Do not spend more effort there.

**The one untested signal is the filter's own INNOVATION.** `nu = z_leg - h(x) =
v_body,legodom - Rz(-psi) v_world_hat` is already computed every `correctLegOdom` and is
exactly "leg odom says moving, the IMU says we did not accelerate that way". Normalised by
`S = H P H' + R` it is the NIS: chi-2 distributed when the model holds, spiking when it
does not. Next steps, in order:
1. **Log `nu` and NIS** from `eskf_node` into the CSV and run_report, re-run this same
   course, and test patch-vs-non-patch separation. This is the GO/NO-GO — if the innovation
   does not separate, no slip model works with these sensors.
2. If it separates, **use it directly**: NIS-gated covariance inflation / a robust (Huber)
   update beats a learned model — no training set, no retraining when the gait changes,
   graceful degradation.
3. Only then revisit the neural model, and retrain from scratch (existing weights learned
   gait tracking error on a world with no slip) with an innovation feature, not
   `contact_frac_`.

**Ceiling to remember:** the slip model can only address the ~1.3 m velocity-bias
component. Yaw is exactly unobservable with GPS off, so the 10.9 deg heading drift is
untouched by anything done to leg-odom covariance. Slip gating is the cheap win; an
absolute heading reference is still the big one.

**Three run_report bugs this run exposed (all fixed, re-verified against its CSV):**
- Stall detector armed on a single commanded sample at the window's END -> tripped on the
  ~10 s warm-up every run. Now requires the whole window commanded.
- Stall detector measured POSITION only -> `square_test` turning in place at a corner
  (measured: 260 deg of yaw for 35 cm of travel) read as a stall. Progress is now
  `displacement + 0.1934 * |dyaw|`. Fixed detector on this run: worst 9 s progress 36.5 cm,
  **never stalled**.
- Reported "body pitch from IMU max 67.8 deg" against a ground truth of **14.8 deg** —
  instantaneous accel during a trot is rotating and impact-spiking, so single samples are
  not attitude. Report now prints GROUND-TRUTH attitude and caveats the IMU number.

### RUN REPORT: stop debugging from screenshots (2026-08-02)

`scripts/run_report.py`, on by default in `run_go2_teleop.sh` (`--no-report` disables).
**Read `run_report/REPORT.md` first, every time.** Everything in `run_report/` is
OVERWRITTEN each run — fixed ~0.5 MB forever, gitignored.

It exists because this session burned two rounds of digging: `~/.ros/log` had to be
hand-read, the terrain slope at the stall point had to be re-derived from the PNG by hand,
and the launcher's own per-node logs were being `rm -rf`'d on exit. All three are now
captured automatically.

What it answers without a screenshot:
- **did it stall, when, and where** — sliding-window progress detector, only armed while
  `/cmd_vel` is non-zero (so standing still on purpose is not a stall)
- **was it the hill** — elevation AND slope sampled from the world's own heightmap at the
  ground-truth (x,y), with the steepest point and its timestamp
- **are the gains too soft** — joint tracking error, commanded `joint_trajectory` vs
  measured `joint_states`. This is the stance-sag metric (SLOPE_POSTURE.md §2): 0.134 rad
  = 7.7 deg = ~74 N of push = ~2.9 cm of sag at p=100
- **per-leg contact duty**, leg-odom degenerate %, IMU |accel| peaks (contact impacts)
- **estimator** ATE / max / final position error, yaw error, error as % of path
- **which topic never appeared** — the single most common silent failure here
- **WARN/ERROR from every node**, de-duplicated

Design points worth not re-litigating:
- The flush timer runs on **WALL** time, not sim time. With `use_sim_time:=true` and gz
  dead, `/clock` never advances and sim-time timers never fire — i.e. no report for
  exactly the run that needs one. Verified: with no `/clock` at all it still writes a
  report whose topic table is all `0 — NEVER RECEIVED`.
- `cleanup()` copies `$LOGDIR/*.log` into `run_report/node_logs/` **before** signalling
  anything, so run_report's shutdown write can mine them. Order matters.
- CSV is row-capped: on overflow it drops every other row and halves the rate, so a
  runaway run cannot fill the disk.
- `Node.context` is a real rclpy attribute — do not name a method `context()` on a Node
  (cost one crash: `'function' object has no attribute 'handle'` from `create_timer`).

### BOTH estimator arms now run in ONE run — three curves in one plot (2026-08-11)

The slip-adaptive estimator used to be a *separate run* (`benchmark.launch.py
scenario:=adaptive`), so comparing it with the baseline meant comparing two different
square runs — worthless here, because run-to-run variance dominates (§0 HEADLINE, 4°–97°
of yaw error from identical configs). Now both arms run **simultaneously against the same
sensor stream**:

| arm | node | topic | difference |
|---|---|---|---|
| baseline | `eskf_node` | `/eskf/odom` | fixed `R_leg` |
| slip-adaptive | `eskf_slip_node` | `/eskf_slip/odom` | `use_slip_model:=true`, `R_leg ← R_leg·(1+λ·slip)²` |

Everything else — params file, IMU, leg odom, cmd_vel, joint states, GPS setting — is
identical, so the difference between the two curves **is** the slip model, on one run,
with the variance cancelled. That is the only way this comparison is worth anything.

- `eskf.launch.py slip:=true` starts the second arm (`slip_model_path`, `slip_odom_topic`
  are args). It is `false` by default, so nothing else changes.
- `run_go2_teleop.sh` passes `slip:=true` **by default**; `--no-slip` reverts to one arm.
- `plot_trajectory.py` draws **three** curves (truth, baseline, slip-adaptive) and one
  error trace per arm in the right panel. `--wait-slip N` drops the third curve from the
  legend if the topic never appears (the launcher passes 30 s), `--no-slip` disables it.
- `REPORT.md`'s estimator table has **one column per arm**, plus a one-line verdict; the
  CSV gains `slip_x, slip_y, slip_yaw, slip_err_pos, slip_err_yaw`. `/eskf_slip/odom` is
  registered as *optional*, so with `--no-slip` it reads `not running (optional)` instead
  of the loud `NEVER RECEIVED`.
- Diagnostic topics of the second arm are remapped (`eskf_slip/slip`,
  `eskf_slip/gyro_bias`), otherwise both arms would publish onto the same names. Its
  `publish_tf` is forced false — the baseline owns `map→base_link`.

Cost: one extra ESKF instance (IMU-rate predict + an 8-input MLP), negligible next to gz.
Verified offline: both nodes come up and the slip weights load; the plot renders all three
curves in truth/no-truth/no-slip modes; the report writes a 34-column CSV and a two-column
estimator table. **Not yet run against the live sim** — and one run still proves nothing.

### FIRST dual-arm terrain square: slip-adaptive is 2.4x WORSE (2026-08-11, n=1)

First run of the two-arms-at-once setup. `./run_go2_teleop.sh --terrain --square`, stock
gains, no `--adapt`, GPS off, commit `deeaef0`. **The robot completed all four corners** on
the 11.3-deg-capped terrain — no stall, no fall (worst 9 s progress 52.8 cm, terrain under
the path median 2.1 deg / max 9.8 deg, |pitch| median 2.4 deg / max 18.1 deg).

| | baseline (fixed R) | slip-adaptive |
|---|---|---|
| ATE mean | 1.898 m | 2.917 m |
| max / final position error | 3.506 / **2.591 m** | 6.126 / **6.121 m** |
| yaw error mean / final | 5.9 deg / 2.6 deg | 13.5 deg / **13.6 deg** |
| final error as % of 46.75 m path | 5.54 % | 13.09 % |

Square-test's own corner errors (baseline arm): 0.254, 3.068, 3.501, 2.583 m.

Read the YAW row, not the position row: the arms track each other to ~0.1 m for the first
40 s and then separate exactly as their yaw errors separate (t=66 s: 3.6 vs 14.3 deg → 0.32
vs 0.63 m; t=237 s: 5.3 vs 15.0 deg → 2.54 vs 5.82 m). Position error is the yaw error
integrated along the path, as always here.

**Why an R_leg change moves YAW at all** (the thing to not re-derive): `correctLegOdom`'s
Jacobian has `H[:,PSI] = dRz(-psi)/dpsi @ v_world`, which is nonzero whenever the robot is
MOVING. Only the direction `δv = δψ·(-v_y, v_x)` is unobservable (§0 HEADLINE 2); its
orthogonal complement IS measured, so each leg-odom update applies a restoring pull on psi
toward the heading implied by the measured body velocity. Inflating `R_leg` turns that
spring down. It is NOT the gyro-bias path — `r_leg_yaw_bias` is not scaled by slip (checked
`legCovarianceForUpdate`; only `R_leg` is).

**But that mechanism does NOT explain a 2.6 vs 13.6 deg gap.** `scripts/slip_yaw_experiment.py`
(offline, NumPy twin, no ROS/gz) injects one identical heading kick into filters differing
only in `R_leg`, over 40 seeds:

| R_leg | mean \|final yaw err\| | recovered from the kick | worse than baseline |
|---|---|---|---|
| 1.00x | 11.90 deg | **16/40** | — |
| 1.44x | 11.66 deg | 15/40 | 19/40 |
| 1.82x | 11.47 deg | 12/40 | 19/40 |
| 2.25x | 11.41 deg | 12/40 | 19/40 |
| 4.00x | 11.66 deg | **10/40** | 20/40 |

The restoring pull is **conditional**: it only works while `v_world` is still anchored near
truth. Once `(v_world, psi)` rotate together into the unobservable direction the residual
vanishes and nothing pulls heading back — for EITHER arm; baseline fails to recover in
24/40 seeds. Inflating `R_leg` monotonically lowers the recovery *probability*, but leaves
the final error a **coin flip in both directions** (worse in ~19/40 — noise).

So the honest reading of the run: the sim realization was identical for both arms (same
sensor stream), so the 2.6-vs-13.6 gap IS downstream of `R_leg` — but via which *basin* the
filter fell into, not via a systematic penalty. A small R change flips that basin either
way. **An earlier version of this entry said "inflating R_leg costs yaw stability" — that
overstated a single seed and a single run; the 40-seed sweep does not support it.**

What the run DOES establish: the plumbing works (both arms 25,982 msgs at 100.0 Hz off one
sensor stream, 1,298 CSV rows, both columns populated).

**Gap found and fixed the same day**: the run recorded the slip model's *effect* but not its
*score*, so "detects slip" and "de-weights leg odometry everywhere" were indistinguishable.
`run_report.py` now subscribes `/eskf_slip/slip` (optional), logs a `slip_score` CSV column,
and prints mean/median/range plus the implied `R_leg` inflation factor. **A narrow score
range = a blanket de-weighting, which would explain the yaw cost with none of the benefit.**
Check that line first on the next terrain run — it is the cheapest way to decide whether the
model or the coupling is at fault.

### Terrain BLOCKER: CHAMP stalls on the start-pad blend ramp at ~12 deg (2026-08-02)

`square_test` aborted 84 s into a terrain run, 0/4 corners done:

    ROBOT FELL/STUCK at truth=(+2.55,+0.42) after 0/4 corners
    — no progress for 9.0 s while commanded to move (14 cm, 8 deg)

Nothing crashed. gz, `controller_manager`, both controllers, `quadruped_controller`,
`state_estimation` and `eskf_node` all ran healthy through to a clean SIGINT 48 s later.
`eskf_node`'s degenerate (all-zero leg odom = stopped) fraction climbing back 27.9% ->
40.2 -> 48.9 -> 55.3 -> 60.4% after the abort confirms the robot was standing still, not
that the sim died. **The stall detector was right; this is a gait failure.**

**What is at (2.55, 0.42)** — sampled straight out of `terrain_height.png`
(col->+X, row0->+Y max, z = px/255 * 0.7):

    x[m]   1.50  1.75  2.00  2.25  2.50  2.75  3.00  3.25  3.50
    z[m]  0.002 0.019 0.051 0.099 0.151 0.204 0.254 0.294 0.316
    slope  0.4   3.9   7.5  10.8  11.9  11.9  11.3   9.0   5.0   deg

It died at the steepest point of a sustained 11-12 deg climb — 3.2x the square path's
median slope (3.7 deg) and near its max (16.6 deg). **This is NOT the ice**:
`slip_patch_0` spans x in [3.5, 6.5] at y=0, ~1 m further on. The robot never reached a
patch.

**The ramp is an artifact of the start pad, not of `--relief`.** `--pad-radius 1.5
--pad-blend 2.5` compresses the whole flat-pad -> full-relief transition into a 2.5 m
annulus, so the hardest climb on the entire route sits 2 m from spawn and is hit in the
first ~12 s, before the trot settles.

**DISPROVED: swing height is not the fix.** `swing_height: 0.08` (edit §5.7) was already
live for this run — gait.yaml mtime 13:36:19, sim launched 13:38:47, config is
symlink-installed. Doubling clearance did not get it up the ramp, and the arithmetic says
it never could: step length = `raibertHeuristic * 2` = `(stance_duration/2)*v*2` =
0.25*0.25 = **6.25 cm/step**, so an 11.9 deg slope rises **1.3 cm per step** against an
8 cm swing apex — 6x margin even at the old 0.04.

**Friction is not it either.** Neither `flat.sdf` nor `terrain.sdf` sets
`<surface><friction>` on the ground, so both take the gz default mu=1.0, far above the
tan(11.9 deg)=0.21 needed to stand. Terrain is no more slippery than flat outside the
deliberate mu=0.3 patches.

**The actual cause: CHAMP has ZERO terrain adaptation.**
`quadruped_controller.cpp:100` sets `req_pose_.position.z = nominal_height` once and never
touches orientation; `req_pose_` changes only via the `/body_pose` topic, and **nothing in
the launch publishes it**. So `BodyController::poseCommand` holds all four feet on a single
plane 0.225 m below the hips in the BASE frame at zero roll/pitch for the whole run — no
IMU feedback into foot placement, no per-leg height offset, no pitch compensation, no
balance term. On a sustained incline the front feet contact early and the rear reach into
air, the body pitches up, stance shortens, and the trot degenerates into stepping in place.
That is exactly the "14 cm in 9 s while commanded to move" the detector saw.

### What was SHIPPED for it (2026-08-02, later) — all three UNVALIDATED in sim

**1. Terrain re-cut with a hard slope cap.** New generator option `--max-slope DEG`. Slope
is *exactly* linear in relief — the quantised heightmap shape does not depend on relief and
elevation is `pixel/255 * relief` — so the required relief is `tan(cap)/max|grad(pix/255)|`,
solved in closed form on the QUANTISED image (so the reported number is the one physics
sees), not searched. Regenerated with:

    python3 src/go2_eskf/scripts/make_terrain_world.py --max-slope 11.3

    relief resolved 0.70 -> 0.40 m
    slope        median 3.6 -> 2.0 deg, 95th 7.5 -> 4.3, max 19.5 -> 11.3
    square path  median 3.7 -> 2.1 deg, max 16.6 -> 9.6
    patch slopes 9.8/7.3/9.7/10.3 -> 5.6/4.1/5.5/5.9 deg (mu=0.3 now comfortable)
    THE KILLER RAMP at x=2.0..3.1: 11.9 -> 6.9 deg peak

**2. `--adapt`: slope-adaptive body posture** — `scripts/terrain_adapt.py`, first-party, new.
Publishes the `/body_pose` topic CHAMP already subscribes to but nobody ever fed, at 50 Hz:

    position.x  = com_shift_x * sin(pitch)        shift body UPHILL (CoM into support)
    position.z  = -crouch * |sin(pitch)|          crouch on slopes (DELTA on nominal_height,
                                                  cmdPoseCallback_ adds it back — no collapse trap)
    orientation = RPY(-level_roll*roll, -level_pitch*pitch, 0)

Defaults `com_shift_x 0.15`, `crouch 0.10`, `level_pitch 0.0`. **All gains 0 == bit-for-bit
stock**, so it is a clean A/B arm. `level_pitch` is real feedback on measured attitude (0 =
body follows terrain, 1 = body level in world) and is OFF by default — raise it deliberately
and watch for oscillation. Attitude comes from a low-passed accelerometer (tau 0.7 s) with
impact rejection by |f|, because the gz IMU orientation is unreliable and the ESKF is
yaw-only. Cost: ~18 cm of lag at 0.25 m/s — fine for slopes that change over metres.

Signs verified offline against a synthetic tilted IMU (no sim), which is the risky part:

    +11.3 deg climb  -> body +2.94 cm forward, -1.96 cm height, cmd pitch  0.0 deg
    +11.3, level=1.0 -> body +2.94 cm forward, -1.96 cm height, cmd pitch -11.3 deg
    -11.3 deg descent-> body -2.94 cm forward, -1.96 cm height, cmd pitch  0.0 deg

**3. `--stiff`: 3x joint PD** — `go2_eskf/config/ros_control_stiff.yaml` (p 100->300,
d 1.0->3.5), selected via the launch's existing `ros_control_file` arg, which feeds both
`controller_manager` and the xacro's `gz_ros2_control <parameters>`. **No vendored edit.**
The arithmetic: 15.10 kg (6.921 trunk + 4x2.044 leg) = 148 N; a trot puts ~74 N on each of
two feet; ~0.2 m moment arm => ~15 Nm at the knee; at p=100 that needs 0.15 rad of tracking
error = **~3.2 cm of foot sag, 14% of nominal_height** — stroke the stance never delivers,
which is exactly "steps but does not advance". Not an actuator limit: `calf_torque_max` is
35.55 Nm (2.4x headroom) and gz logs "Enforcing command limits is disabled". p=300 -> ~1.1 cm.
d follows sqrt(p) with light links (I~0.01 kg m^2): 2*sqrt(300*0.01) ~ 3.5; the stock d=1.0
was under-damped even at p=100. CAVEAT: trades sim realism for gait robustness — a real Go2
cannot hold 300 Nm/rad at 100 Hz, so leg-odom tracking here is optimistic. Keep A/B arms on
ONE gain set.

`./run_go2_teleop.sh --climb` = `--terrain --adapt --stiff`. **A/B them ONE AT A TIME.**

**Theory written up: `src/go2_eskf/docs/SLOPE_POSTURE.md`** — read it before retuning any
of this. Results worth knowing without opening it:
- The slope problem is the **CoM gravity projection** `Δ = h·tan(gamma)`, not height.
  4.50 cm downhill at 11.3 deg = 23% of the half-base, which transfers the four-foot load
  from 50/50 to **38/62 front/rear** (1.61:1). Front feet lose 23% of their friction, rear
  work hardest. Tip-over is at `atan(a/h)` = 40.7 deg — never the issue.
- **Ideal `com_shift_x` = h/cos(gamma) ~ h = 0.225**, i.e. the ideal CoM-shift gain is just
  the body height. The 0.15 default is deliberately 2/3 of it (65% compensation, front load
  back to 46%). Raise it toward 0.225 FIRST if a climb still fails.
- **`crouch` barely touches the CoM problem** — 2 cm of crouch buys back 4 mm of the 4.5 cm
  shift (<10%). It is a dynamic-margin knob, not a slope-compensation one.
- **`level_pitch = k` retains `gamma/(1+k)` of the ground slope**, so k=1.0 only HALVES the
  pitch, it does not level. Keep k <= 1 (the 0.7 s attitude lag + body dynamics will ring).
  Default 0 is also the geometrically better choice: `beta_r = 0` equalises leg extension
  and torque and keeps swing clearance perpendicular to the ground.
- Stance sag at the stock p=100 is **2.85 cm (12.7% of nominal_height)** at 74 N/foot, vs
  0.95 cm at p=300. Knee torque only varies 12.4-14.2 Nm across the whole leg-extension
  range, so crouching/levelling are cheap in torque; the sag is the expensive part.
- **11.3 deg is exactly `atan(mu/1.5)` for mu=0.3** — the walkable limit of the slip
  patches. Above it the patches stop being a slip experiment and become an unconditional
  fall. That is the principled justification for the cap, not just "less steep".

Untried, still open, in order of cheapness:
- **Slow down**: 0.25 m/s is 83% of `max_linear_velocity_x` (0.3). Try v=0.15 in `square_test`.
- **Lower/lengthen the gait**: `nominal_height` 0.225 -> 0.20, `stance_duration` 0.25 -> 0.30.
- **Widen the pad blend** (`--pad-blend 6.0`) if the ramp is still what kills it — but the
  slope cap already took that ramp to 6.9 deg, so this is probably redundant now.

Confirm the stall location over >=2 runs before drawing conclusions: the *terrain* is
deterministic, but whether the trot survives a given ramp is not.

### Next steps (in order)

1. **Re-run `probe_leg_odom.py`** (~1 min of sim, not 6.5) and check the four predictions
   above. This is a sensor-level check, so it needs far fewer runs than a square.
2. **Then** square runs, still >=5 per arm, comparing against the recorded distribution
   (not against any single historical number).
3. If heading still drifts after the sensor fix, stop tuning the filter and go for an
   absolute reference — the observability result says nothing else can work. Cheapest path:
   fix the navsat `<stddev>` to `4.5e-6` in the vendored `unitree_go2_gazebo.xacro` (§2.2)
   and turn GPS on.
4. Record the numbers here **whether or not they improve.**

---

**Earlier (2026-07-26 — NEGATIVE RESULT: square drift is not repeatable, so none of this
session's filter changes can be shown to help. The durable finding is that drift is a
HEADING problem.)**

A screenshot of a full 4-corner square run (final **5.945 m**, max 6.144 m — worse than
the 2.172 m run below) started this. Read the HEADLINE section first; everything else in
this entry is supporting detail.

Session outcome in four lines:
- One hypothesis (champ's all-zero leg-odom samples fused as measurements) was written up
  confidently from a code read and then **disproved by the sim** (0.0% zeros while walking).
- A real, reproducible turn-correlated defect WAS measured: the `correctGyroBias` input is
  ~10x larger while turning. A gate for it was implemented; **its A/B is inconclusive.**
- The deferred core `Q` convention bug was fixed (constraint lifted), cross-validated in
  lockstep with the NumPy twin. **Also unproven** against the sim.
- Six square runs proved the measurement method itself is too noisy to judge any of it.
  **That is the main thing to fix before more tuning.**

### HEADLINE: six square runs, three configs — RUN-TO-RUN VARIANCE DOMINATES

**No filter change made here can be shown to help.** Every configuration produced one good
run and one bad run; the within-config spread is far larger than any between-config
difference. Do not tune further off single runs — and treat every drift number recorded
earlier in this file (including the 2.172 m "baseline" and the 5.945 m screenshot) as one
draw from a wide distribution, not a measurement of a configuration.

| config | run 1 (yaw mean / pos max) | run 2 (yaw mean / pos max) |
|---|---|---|
| old Q, bias gate ON  | 9.4 deg / 2.14 m | 24.5 deg / 3.40 m |
| old Q, bias gate OFF | 65.5 deg / 12.70 m | **4.3 deg / 2.32 m** |
| new Q + gyro scale noise | 8.2 deg / 1.55 m | 34.9 deg / 7.08 m |

Best single run overall: `newQ+scale` at **final 0.439 m (1.10% of 40 m), max 1.546 m** —
but its sibling run was 7.045 m, so that number means nothing on its own.

**What IS solid (consistent across all six runs): position error is a direct consequence of
YAW error.** The two track each other 1:1 — 4.3 deg -> 2.3 m, 8.2 deg -> 1.6 m,
24.5 deg -> 3.4 m, 34.9 deg -> 7.1 m, 65.5 deg -> 12.7 m. **Stop treating this as position
drift; it is a heading problem, and any fix must be argued in degrees of yaw.**

**b_g is never the culprit**: |b_g| <= 1.07e-2 rad/s in every run, because
`Q(BG,BG) = sbg2*dt ~ 2e-10` keeps `P(BG)` and hence the bias gain tiny. The bias channel
is effectively frozen — which is also why gating it changes little.

**The real open question is now the VARIANCE, not the mean.** Identical configs give
4-97 deg of yaw error. Note the probe's `wz_gyro/wz_truth` also swung **0.830 -> 0.964**
between two runs — the yaw-rate scale error itself is not repeatable, which would explain
non-repeatable heading error. Next step is sensor-level, not filter-level: find out why
that ratio moves. Check whether `/ground_truth/odom`'s `twist.angular.z` is trustworthy
(frame/derivation) before trusting the ratio at all; the yaw ERROR numbers above come from
truth *pose* orientation and are solid regardless.

**Statistical discipline for whoever picks this up:** ~5+ runs per arm minimum, report the
distribution (median + spread), and prefer paired runs in one session. Each run costs
~6.5 min wall clock (sim relaunch is mandatory — see the world-reset trap in §4).

### MEASURED (two probe runs, n≈11,500 samples each — this is the ground truth)

**The degenerate-zero hypothesis is WRONG while walking: 0.0% of `/odom/raw` samples are
all-zero during either straight walking or turning.** The degenerate branch fires **only
while stopped**, in unbroken runs of **3.1–5.0 s**. So:

- Zeros are **not** the drift mechanism, and they do **not** explain why
  `leg_odom_scale: 1.111` appears not to reach the estimate. That remains OPEN.
- The speed under-read is just `odom_scaler`: `vx_leg/vx_truth` = **0.894 / 0.903** on
  straight legs across the two runs — exactly the 0.9 already known.

**What the measurement DID find — reproducible and turn-correlated.** The residual
`eskf_node` feeds to `correctGyroBias`:

| phase | run 1 | run 2 |
|---|---|---|
| STRAIGHT (`vx=0.25, wz=0`) | −0.0039 rad/s | −0.0057 rad/s |
| TURN (`vx=0, wz=+0.4`)     | **+0.0452 rad/s** | **+0.0520 rad/s** |

A gyro bias is a slowly-varying constant, so this residual must be ~equal in both phases
(the probe's own docstring says so). It is **~10× larger while turning**, consistently. So
`correctGyroBias` is injecting a **false bias of ~+0.05 rad/s (≈2.9°/s) only during
turns**, at a tight `R=(0.05)²`. `predictImu` integrates `psi += (gyro_z − b_g)·dt`, so
over a ~4 s corner that is **~10° of heading error per corner** — which is the shape seen
in the screenshot (fine until a turn, then monotonic divergence) and the corner-correlated
errors below. (This motivated the `|wz|` gate below; note the gate's A/B could not confirm
it mattered, and `b_g` never grew enough for this mechanism to be the dominant one.)

**Correction to the previous entry:** the table below records
`gyro_wz − wz_leg (turning) = +0.042` under "after" and treats the turn residual as fixed.
**It was not fixed — +0.042 *was already this bug*,** and it reproduces now at
+0.045/+0.052.

Second turn-correlated error, also reproducible: `vx_leg/vx_truth` = **0.773 / 0.781 while
turning** vs 0.894/0.903 straight — leg odometry under-reads translation ~13% more while
rotating, which `leg_odom_scale` (a constant) cannot track.

**Do not trust a single-run yaw ratio.** `wz_leg/wz_truth` measured 0.734 then 0.901, and
`wz_gyro/wz_truth` 0.830 then 0.964 — too noisy to conclude from one run. The *residual*
above is the stable statistic; use it.

### The `|wz|` bias gate: IMPLEMENTED, and the A/B is INCONCLUSIVE

`bias_update_max_wz: 0.10` now skips `correctGyroBias` while rotating. Four square runs,
paired, same session, only that parameter changed:

| run | final [m] | max [m] | mean [m] | yaw err mean / max [deg] | max abs b_g [rad/s] |
|---|---|---|---|---|---|
| gate ON  | 1.055 | 2.136 | 1.058 | 9.4 / 14.8 | 3.8e-3 |
| gate ON  | 3.180 | 3.403 | 1.906 | 24.5 / 36.2 | 4.7e-3 |
| gate OFF | 12.584 | 12.695 | 4.126 | 65.5 / 97.1 | 1.07e-2 |
| gate OFF | **0.695** | 2.325 | 1.332 | **4.3 / 7.1** | 6.7e-3 |

**Do not claim the gate fixes the drift.** The best run of the four is a gate-OFF run.
n=2 per arm against this much gait variance establishes nothing; the gate does look like it
reduces the b_g excursion (3.8–4.7e-3 with vs 6.7–10.7e-3 without) and the tail risk, but
that needs many more runs to assert. Keep it — it is physically correct (a bias is only
observable when not rotating) and cheap — but it is not the fix.

### Leading hypothesis now: yaw runs OPEN-LOOP on a gyro with a turn-dependent scale error

Two measured facts combine:
1. `wz_gyro/wz_truth` measured **0.830 and 0.964** during turns — a 4–17% yaw-rate scale
   error. Over four 90-degree corners (360 deg of rotation) that alone yields **14–61 deg**
   of yaw error, which is the observed range.
2. `Q(PSI,PSI) = sg2*dt^2` treats `gyro_noise` as a per-sample std although the config
   documents these as continuous-time densities — **100x too small at dt=0.01**. So
   `P(psi)` barely grows, and the filter effectively REFUSES the yaw information that leg
   odometry does carry (`correctLegOdom`'s `H` has a real `PSI` column,
   `eskf_core.cpp:129`, via the body-frame `vy` residual).

i.e. the gyro scale error is the disturbance, and the over-confident `P(psi)` is why
nothing corrects it.

### CORE CHANGE SHIPPED (Q convention + gyro scale noise) — correct, but unproven

The node-only constraint was lifted, so this was implemented in `eskf_core.cpp` and
`eskf_reference.py` **in lockstep** (cross-validation re-passes at **2.498e-15**, 16/16
GTest):

1. **Q now uses the continuous-time convention throughout.** It was
   `Q(v,v)=sa^2*dt^2`, `Q(psi,psi)=sg^2*dt^2` (per-SAMPLE) while `Q(b,b)=sbg^2*dt`
   (density) — internally inconsistent, and 100x too small for psi at dt=0.01. Now
   `sa^2*dt`, `sg^2*dt`, `sbg^2*dt`, plus the `p-v` cross-covariance blocks the
   diagonal-only form omitted (standard discrete white-noise-acceleration model).
   **`accel_noise` retuned 50.0 -> 5.0** so `Q(v,v)=sa^2*dt=0.25` per step is numerically
   identical to the old value — velocity behaviour is deliberately unchanged, only psi
   moves.
2. **New `gyro_scale_noise` (default 0.10).** `Q(psi,psi) = (sg^2 + (0.1*wz)^2)*dt`, so
   heading goes uncertain exactly while turning — modelling the measured 4-17% yaw-rate
   scale error, which no white-noise term can represent. Set 0 for white-noise-only.

**Both are defensible on first principles and neither is proven to help** (see the variance
headline). They are kept because they are more correct, not because they measured better.

### What WAS shipped, and what it is actually worth

The degenerate-sample gate was implemented and kept — but **be honest about its value: it
is nearly inert while walking** (0% of samples), so it is **not** a drift fix and no
improvement should be claimed for it. What it does buy, on measurement:
- The 3.1–5.0 s runs of standing zeros are now fused as a proper tight ZUPT
  (`zupt_vel_noise: 0.02`) instead of at `R=(0.1)²` — a better anchor while idle, and the
  cleanest gyro-bias observation available.
- `degenerate_hold_sec: 0.3` separates the two classes with huge margin (walking: **no
  degenerate runs at all**; stopped: 3.1–5.0 s), so the threshold is safe as-is.
- It is insurance if gait params ever change to produce genuine flight phases.

### The hypothesis that was DISPROVED (kept so it is not re-derived)

`champ::Odometry::getVelocities` (`champ/include/champ/odometry/odometry.h:97-108`)
early-returns **hard zeros for `linear.x`, `linear.y` AND `angular.z`** whenever
`allFeetInContact()` or `noFootInContact()` — its "nothing to calculate" branch. With
`state_estimation.cpp:148` calls `getVelocities` from a **fixed 50 Hz timer** and publishes
the result straight into `/odom/raw`'s twist (`:189-195`). All of that is true, and those
zeros genuinely are a no-information flag rather than a measurement.

**The reasoning error** was inferring from `gait.yaml`'s `stance_duration: 0.25` that a
trot spends much of each cycle with no foot planted. It does not: measurement shows
`noFootInContact()` never fires while walking (0/23,000 samples), so `allFeetInContact()`
while stopped is the only path that reaches the branch. `stance_duration` is not the duty
factor that inference assumed. **Lesson: this file's own rule — measure before theorising —
was skipped, and a code-read hypothesis got written up as a root cause.**

Consequently both claimed mechanisms are void: velocity was never dragged toward zero while
walking, and `correctGyroBias` was never fed `gyro_wz − 0`. The real bias corruption is the
*magnitude* disagreement measured above, not a zero.

### The gate as actually implemented (first-party, node-level; core untouched)

`legOdomCallback` classifies each message before fusing. The subtlety: **a genuinely
stationary robot also reports `(0,0,0)`** (all four feet planted), and there the zeros are
real information — a ZUPT, and the best gyro-bias observation available. Gating all zeros
would remove the zero-velocity anchor while standing and let velocity random-walk under
`accel_noise: 50`. The discriminator is **duration**: a flight phase lasts a fraction of a
gait cycle, standing produces an unbroken run. Hence `degenerate_hold_sec`.

| sample | condition | action |
|---|---|---|
| non-degenerate | any | fuse as before |
| `(0,0,0)` | zero-run ≥ `degenerate_hold_sec` and no fresh moving `cmd_vel` | fuse as a **ZUPT** (tight `zupt_vel_noise`) + fuse gyro bias |
| `(0,0,0)` | otherwise | **skip** both `correctLegOdom` and `correctGyroBias` |

`correctVerticalVel(0,·)` still runs on every message — it is what bounds `pz`.

**Do NOT key this off `/cmd_vel` going quiet.** `teleop_twist_keyboard` publishes only on
keypress, so a stale command does not mean a stopped robot; keying on staleness would fuse
flight-phase zeros as a *tight* ZUPT while walking — worse than the original bug.
`cmd_vel` is used only as a **veto** (a fresh command asking for motion rules out a ZUPT).

`leg_odom_gate_degenerate: false` reproduces the old behaviour **exactly** (the ZUPT
tightening is also gated on it), so it is a valid A/B baseline. Config is
symlink-installed → no rebuild needed to flip it.

### Next steps (in order) — all filter work is BLOCKED on step 1

1. **Fix the measurement before tuning anything else.** Six runs showed 4–97° of yaw error
   for identical configs, so a single square run carries almost no information. Either
   run ~5+ per arm and compare distributions (≈6.5 min each, so ~35 min per arm), or
   reduce the variance at the source. Until then, no tuning change can be evaluated and
   any number quoted from one run is noise.
2. **Chase the variance, which looks SENSOR-level, not filter-level.** `wz_gyro/wz_truth`
   itself swung 0.830 → 0.964 between two probe runs. First check whether
   `/ground_truth/odom`'s `twist.angular.z` is even trustworthy (frame convention /
   derivation in the gz `OdometryPublisher`) — if it is not, every yaw-*rate* ratio in this
   file is suspect. The yaw *error* numbers are safe: they come from truth pose orientation.
3. Then re-evaluate the two unproven changes from this session (`bias_update_max_wz`,
   `gyro_scale_noise`) with adequate n, and A/B them properly. Both default ON.
4. Read-outs to use: `/eskf/gyro_bias` (new topic) and the `est_bg` CSV column for the bias
   channel; `probe_leg_odom.py` for the STRAIGHT-vs-TURN residual, which should converge if
   the bias gate is doing what it claims.
5. Record the numbers here **whether or not they improve.**

### State at handoff (git / build / artifacts)

- **All green:** 16/16 GTest, C++≡NumPy **2.498e-15**, `metrics.py --selftest` and
  `run_benchmark.py --demo` pass. Sim torn down, VRAM 3074/4096 MiB free.
- **Branch** `fix/sim-readiness-guard-and-drift-baseline`, still 4 commits, **nothing from
  this session committed.** Uncommitted: `CLAUDE.md`, `skills.md`,
  `go2_eskf/{CMakeLists.txt, config/eskf_params.yaml, include/go2_eskf/eskf_core.hpp,
  include/go2_eskf/eskf_node.hpp, src/eskf_core.cpp, src/eskf_node.cpp,
  scripts/{eskf_reference.py, metrics.py, run_benchmark.py, square_test.py}}`,
  vendored `champ/.../odometry.h`; untracked `go2_eskf/scripts/probe_leg_odom.py`.
  **Commit `eskf_core.cpp` and `eskf_reference.py` together** — they are a lockstep pair
  and `cross_validate.py` is the tripwire if they ever drift apart.
- **The six run CSVs and the `full_run.sh` A/B harness were written to an ephemeral session
  scratchpad and are GONE.** Only the aggregate numbers above survive. If per-sample
  trajectories matter next time, log to a path inside the repo (or `~/`) via the node's
  `log_path` parameter, and keep the harness in `src/go2_eskf/scripts/`. Rebuilding the
  harness is ~20 lines; the recipe is in §7.

### Still open / deliberately not done

- ~~`Q(PSI,PSI) = sg2·dt²` convention bug~~ — **FIXED this session**, see "CORE CHANGE
  SHIPPED" above. Left here as a pointer because it was listed as deferred for a while.
- **No innovation (Mahalanobis) gating** on any correction. That is the general defence
  against this whole bug class, but `S` is computed inside `EskfCore::josephUpdate`, so it
  is a core change. Pair it with the `Q` fix.
- **`contact_frac_` is dead** (`eskf_node.hpp`): never assigned, so the slip model's
  `contact_frac` feature is a frozen 1.0. Needs a `/foot_contacts`
  (`champ_msgs/ContactsStamped`) subscription — a first-party→vendored dependency — for a
  model that is off by default. Marked with a comment at the declaration; left unfixed.
- **CHAMP `vel_dt` bug:** `state_estimation.cpp:147` computes
  `(current_time - last_vel_time_).nanoseconds()/1e-9`, i.e. ×1e9 instead of ×1e-9, so
  **`/odom/raw`'s POSE is meaningless.** We consume only the twist, so the ESKF is
  unaffected — but never trust that pose.
- **Raising `stance_duration`** would shrink the degenerate window at the source, but it
  is a vendored gait change that alters dynamics and risks the gait instability already
  seen. Experiment, not a fix.

Offline regressions after the change: **16/16 GTest**, C++≡NumPy **7.994e-15**,
`metrics.py --selftest` and `run_benchmark.py --demo` pass. `metrics.py` now resolves CSV
columns **by name from the header** (so the new `est_bg` column cannot silently shift
which columns are read as ground truth, and pre-`est_bg` logs still load).

---

**Earlier (2026-07-26 late, yaw-error hunt — PARTIAL).**

The sim pipeline works end-to-end and the square test completes all four corners.
Two real sensor-level bugs were found by DIRECT MEASUREMENT against ground truth and
fixed. **They did NOT fix the position error** — be honest about this when resuming.

### Where the error stands (full 4-corner run, after both fixes)
```
CORNER 1/4  truth=(+9.75,+0.00)  eskf=( +8.55, +1.28)  err=1.756 m
CORNER 2/4  truth=(+9.99,-9.75)  eskf=(+11.91, -6.33)  err=3.924 m
CORNER 3/4  truth=(+0.25,-9.99)  eskf=( +4.05, -9.61)  err=3.816 m
CORNER 4/4  truth=(+0.02,-0.25)  eskf=( -0.17, -2.41)  err=2.172 m
final 2.172 m (5.43% of 40 m) · max 4.085 m · mean 2.431 m
```
vs BEFORE the fixes (run cut off at corner 3): 1.003 / 5.096 / 3.652 m.
Corner 2 improved (5.10→3.92), corner 1 got WORSE (1.00→1.76), corner 3 unchanged.
Peak error over the run dropped ~12.0 → 4.09 m. **Net: not solved.**

### The two fixes (both verified by measurement, both keep)
Measured with `ros2 run go2_eskf probe_leg_odom.py` (drives straight, then rotates in
place, and prints leg/gyro/truth ratios per phase — the tool to re-run first tomorrow):

| metric | before | after |
|---|---|---|
| `wz_leg / wz_truth`            | +1.832 | +1.023 |
| `wz_gyro / wz_truth`           | +0.925 | +0.997 |
| `gyro_wz - wz_leg` (straight)  | +0.030 | −0.003 |
| `gyro_wz - wz_leg` (turning)   | **−0.255** | +0.042 |

1. **`gyro_z_sign` reverted −1 → +1.** The raw sim IMU yaw rate ALREADY matches truth in
   sign and magnitude (+0.925 during a commanded wz=+0.4 turn). The earlier "mirrored
   heading" reading that motivated −1 was confounded by bug 2 below. With −1 the filter
   turned the wrong way: invisible while wz≈0, ruinous at every corner.
2. **CHAMP `theta_sum` now averaged by `total_contact`** (`champ/odometry.h`, 5th vendored
   edit, §5.5). It was a raw SUM over stance feet, so reported yaw rate scaled with the
   number of feet on the ground (1.83× truth on a trot). That poisoned `correctGyroBias`
   with a false bias ONLY while turning — which is why straight legs looked fine.

### The two residuals to chase tomorrow (NOT yet explained)
1. **Yaw drifts on STRAIGHT legs.** At corner 1 the estimate is at y=+1.28 where truth is
   y=0.00 — an ~8.5° heading error with no turn involved. Not turn-induced; bias or
   initial alignment during ordinary walking.
2. **Speed under-read ~11%, and `leg_odom_scale: 1.111` does not appear to reach the
   estimate.** Estimated path length to corner 1 is 8.64 m vs 9.75 m truth; the 8.5°
   heading error only accounts for ~1% of that. (An earlier note claiming heading
   explained the shortfall was WRONG.)
   **Leading hypothesis:** `gravity_lp` discards sustained horizontal accel, so the
   PREDICT step pulls v toward zero; with a finite Kalman gain the estimate sits
   systematically below the leg-odom measurement no matter what scale is applied to it.
   **Next measurement (cheap, do this first):** log `/eskf/odom` twist.linear.x against
   truth AND raw `/odom/raw` during steady straight walking. If the estimate sits below
   BOTH, it is prediction-step drag → fix the gain/noise balance, not a scale factor.
   The same lag mechanism would also explain residual turn spikes, so it is one test for
   both residuals.

### Slip model (Phase 3) — assessment before investing more
**Do not expect significant improvement in THIS sim.** Reasons, all measured:
- No slip exists: rigid no-slip floor; `vx_leg/vx_truth = 0.899` is exactly `odom_scaler`,
  a known constant, not stochastic slip.
- Its only lever is inflating `R_leg` so the filter leans on IMU+GPS — but GPS is off and
  `gravity_lp` discards sustained horizontal accel, so **there is no better fallback**;
  firing the slip model makes the estimate worse, not better. Benefit is structurally
  capped near zero until the fallback path is fixed.
- Its lead feature (commanded − measured body velocity) is contaminated: with feet
  perfectly planted, cmd=0.25, truth=0.196, leg=0.177 → a persistent ~0.073 m/s
  disagreement from gait tracking error + `odom_scaler`, with ZERO slip present. Trained
  on sim data the MLP learns to fire when leg odometry is fine.
- None of the measured errors are slip-shaped, so a slip model layered on now risks
  ABSORBING them — apparent gain in-distribution, bad generalisation to hardware.
- To make Phase 3 meaningful in sim: add low-friction patches (`<mu>0.1</mu>`) to
  `go2_eskf/worlds/flat.sdf` to create real slip, AND fix the fallback path first.
- Phase 3 remains a strong ENGINEERING artifact (PyTorch → Eigen MLP, C++≡NumPy 3e-16,
  12 GTests). Fair to present as implemented+validated; not fair to claim accuracy gains.

### Other state
- **Robot fell mid-run once** at x=8.16 on a STRAIGHT leg (base z 0.225→0.162). CHAMP gait
  instability in sim, unrelated to any of our changes (`champ::Odometry` is referenced
  only by `state_estimation.cpp`; `quadruped_controller.cpp` never includes it).
  `square_test.py` now distinguishes "never stood" from "fell mid-run" via `stood_once`
  and ABORTS with a partial summary instead of idling. If falls become frequent, drop
  `--speed` to ~0.2 (gait.yaml allows 0.3 but the sim is not robust at 0.25).
- **Git:** branch `fix/sim-readiness-guard-and-drift-baseline` pushed with 3 commits; PR
  not opened yet (`gh` is NOT installed — `sudo apt install -y gh` then `gh auth login`).
  PR body draft was written but lives in a session scratchpad and is gone; regenerate.
  **UNCOMMITTED:** `eskf_params.yaml` (gyro_z_sign), `square_test.py` (fall handling),
  `champ/odometry.h` (theta_sum), `probe_leg_odom.py` + its CMakeLists entry. Commit these
  as a follow-up WITHOUT claiming they fix the turning error — they do not.
- **Before any sim run:** `nvidia-smi --query-gpu=memory.free --format=csv`. If CARLA is
  up, `docker stop carla-server` (§2.6). The 4 GB card cannot host both.

---

**Earlier (2026-07-26, first REAL square numbers):** two environment/infrastructure bugs
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
- **CHAMP's `/odom/raw` zeros are a NO-INFORMATION FLAG, not a measurement** — but they
  only occur **while the robot is STOPPED.** `getVelocities` early-returns hard zeros for
  vx, vy AND wz whenever all four or zero feet are in contact. MEASURED 2026-07-26:
  **0/23,000 samples are zero while walking or turning**; `noFootInContact()` never fires
  on a trot, and standing (all four planted) produces unbroken runs of 3.1–5.0 s.
  `stance_duration: 0.25` is **not** the duty factor — do not infer flight phases from it.
  `eskf_node` gates them anyway (`leg_odom_gate_degenerate`) and treats a sustained run as
  a ZUPT, which matters only while idle.
- **`stance_duration: 0.25` does not mean a 25% stance duty factor.** A plausible-looking
  chain of reasoning from that value to "half the gait cycle has no foot planted" was
  written up as a root cause and then disproved by a 30-second probe run (§0). Measure.
- **`gz service .../control --req 'reset: {all: true}'` WEDGES THE SIM — do not use it to
  reset between A/B runs.** It rewinds `/clock`, after which `controller_manager`,
  `/ground_truth/odom` and `ros2 control list_controllers` all stop responding and the
  run is unrecoverable. **Relaunch the whole sim per run** (~60 s to controller-active,
  ~6.5 min per square run including the drive). `full_run.sh` in the session scratchpad
  did this; the pattern is: teardown -> launch sim + ground_truth -> poll
  `ros2 control list_controllers` for `joint_group_effort_controller.*active` -> start a
  FRESH `go2_eskf_node` (params and filter state clean) -> `square_test.py`.
- **A sensor error can often be measured OFFLINE, with zero variance — try that before
  booking sim time.** `leg_odom_model.py` resolved an 8% yaw-rate error and a 9% speed
  error in seconds, on a question six square runs (~40 min of sim) could not touch, by
  simulating the estimator's own inputs from a known twist. If a defect is in a
  *derivation*, a live run is the wrong instrument.
- **With GPS off, yaw is EXACTLY unobservable in this ESKF** — `correctLegOdom`'s Jacobian
  has a null direction `dv = dpsi*(-v_y, v_x)`, i.e. rotating heading and world velocity
  together is invisible to it. No `Q`/`R`/gating change can bound heading drift; only a
  smaller yaw-rate disturbance or an absolute heading reference can. See §0.
- **A fallen robot inflates drift instead of zeroing it — and an absolute-height fall
  test is terrain-blind.** `square_test.py`'s `STAND_Z` compares base z against 0.18 m,
  which only means "prone" over a floor at z=0; on `terrain.sdf` a robot collapsed at
  elevation 0.4 m reads 0.46 m and passes. The test then drives a prone robot whose feet
  scrabble, leg odometry reports motion that is not happening, and the run reports a
  large bogus drift (measured: 8.8 m, of which ~9 m was phantom travel after the fall).
  Post-standing falls are now caught by body tilt and by a stall test instead. **Always
  check that ground truth actually completed the square before quoting a number.**
- **Square drift numbers are NOT repeatable.** Identical configs give 4-97 deg of yaw
  error and 0.4-12.6 m of final position error. Never conclude from one run, and never
  compare a new run against a single historical number. ~5+ runs per arm.
- **`ros2 run go2_eskf go2_eskf_node --ros-args --params-file <yaml> -p k:=v`** overrides
  any single parameter without editing the yaml — the clean way to A/B, since the node
  name in `eskf_params.yaml` (`eskf_node`) matches the executable's default node name.
- **`/odom/raw`'s POSE is meaningless** — `state_estimation.cpp:147` divides a nanosecond
  count by `1e-9` instead of multiplying, so its dead-reckoned pose integrates with a
  1e9-scaled dt. The twist is fine; the pose is not. (Vendored, unfixed — we don't use it.)
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

## 5. Deliberate vendored edits (7)

The workspace prefers first-party changes, but seven vendored edits are intentional:
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
5. **CHAMP yaw rate averaged by stance count** — `champ/include/champ/odometry/odometry.h`
   built `vel.angular.z` from `theta_sum`, a raw SUM of each contacting foot's rotation
   about the base, with `total_contact` computed and then never used (`x_sum`/`y_sum` are
   at least averaged, by a hardcoded 2.0). Reported yaw rate therefore scaled with the
   number of feet in stance — measured **1.83× truth** on the Go2's trot. Now divided by
   `total_contact`. Rebuild `champ` AND `champ_base` (header-only change, so champ_base
   must be rebuilt to pick it up). **Superseded by edit 6**, which removes the bearing
   derivation entirely — including its unwrapped `delta_theta` (a foot crossing the atan2
   branch cut would have injected a 2π spike; latent only, because with `atan2f(X, Y)` the
   cut sits at x=0 on the right-side feet and the ±0.19 m hip offsets keep foot x away
   from 0).
6. **CHAMP leg odometry re-derived as a least-squares body twist** — same file,
   `getVelocities`. Replaces the bearing-sum yaw rate (which mistook translation for
   rotation, ±0.59 rad/s per foot at 0.25 m/s) and starts ignoring touchdown samples via
   the already-maintained-but-never-read `prev_foot_contacts_` (which cost ~9% of forward
   speed). Verified offline against an exactly known twist: `vx/truth` 0.821 → 0.900
   (= `odom_scaler`, as intended), `wz/truth` 0.923 → 1.000, false yaw-rate noise on a
   straight walk 0.0675 → 0.0000 rad/s. See §0 and
   `src/go2_eskf/scripts/leg_odom_model.py`. The `allFeetInContact()` early return is
   deliberately left alone — its literal zeros are the flag `eskf_node`'s ZUPT path keys
   on. Rebuild `champ` AND `champ_base`.
7. **Swing height doubled, 0.04 → 0.08 m** — `unitree_go2_sim/config/gait/gait.yaml`
   (2026-08-02). On the terrain world's low-friction patches the rear feet were seen
   dragging/scuffing rather than clearing (screenshot, 22:02 run). CHAMP scales foot
   clearance linearly in this parameter — `TrajectoryPlanner::updateControlPointsHeight`
   sets `height_ratio = swing_height / 0.15` and multiplies every Bézier control point by
   it — so 0.04 → 0.08 doubles the swing apex; the endpoints stay on the stance plane, so
   step length and stance are untouched. The same YAML feeds BOTH `quadruped_controller`
   and `state_estimation`, so leg odometry's foot model stays consistent with the
   commanded gait. Config is symlink-installed: **no rebuild, just relaunch the sim.**
   **It does NOT fix hill climbing** — see §0 "Terrain BLOCKER": the 13:38 run already had
   0.08 and still stalled on the 12 deg ramp, and 6.25 cm steps only rise 1.3 cm per step,
   so clearance was never the binding constraint. Whether it helps the ice-patch scuffing
   that motivated it is still UNMEASURED. Watch for the opposite failure (higher apex in
   the same swing time = faster descent = harder touchdown → more contact-impact accel
   spikes and more slip on landing). Revert = set it back to 0.04.

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

**2026-07-26 late (uncommitted at handoff):**
- `src/go2_eskf/config/eskf_params.yaml` — `gyro_z_sign` −1 → **+1** (measured; see §0)
- `src/go2_eskf/scripts/probe_leg_odom.py` (new, installed) — drives straight then rotates
  in place and prints `vx_leg/vx_truth`, `wz_leg/wz_truth`, `wz_gyro/wz_truth` and the
  `gyro_wz − wz_leg` residual per phase. **Run this first when resuming**; it is how both
  of today's bugs were found and how any yaw/scale claim should be checked.
- `src/go2_eskf/scripts/square_test.py` — `stood_once` fall detection, abort-with-partial-
  summary on a fall, `ExternalShutdownException` handling
- vendored: `champ/include/champ/odometry/odometry.h` — average `theta_sum` (§5.5)

**2026-07-26 (degenerate leg-odom gate — all first-party, core untouched):**
- `src/go2_eskf/src/eskf_node.cpp` — classify/gate degenerate leg-odom samples in
  `legOdomCallback`; new `looksGenuinelyStationary()`; `cmd_vel` subscribed
  unconditionally (the gate's veto needs it, previously slip-model-only); publish
  `/eskf/gyro_bias`; throttled degenerate-fraction + `b_g` log line; `est_bg` in the CSV
- `src/go2_eskf/include/go2_eskf/eskf_node.hpp` — gate members/params, `R_zupt_`,
  degenerate-run tracking, comment marking `contact_frac_` as never assigned
- `src/go2_eskf/config/eskf_params.yaml` — `leg_odom_gate_degenerate`,
  `degenerate_hold_sec`, `zupt_vel_noise`, `stationary_cmd_eps`, `cmd_vel_stale_sec`
- `src/go2_eskf/scripts/probe_leg_odom.py` — degenerate-sample fraction, all-vs-valid
  ratio buckets, `correctGyroBias` input per bucket, degenerate-run duration distribution
- `src/go2_eskf/scripts/metrics.py` — resolve CSV columns BY NAME from the header (adding
  a column can no longer shift ground truth; pre-`est_bg` logs still load); `est_bg` in
  `LOG_COLUMNS`
- `src/go2_eskf/scripts/run_benchmark.py` — emit the extra column in its synthetic logs

**2026-07-26 (|wz| bias gate + core Q convention — node-only constraint lifted):**
- `src/go2_eskf/src/eskf_node.cpp` / `include/go2_eskf/eskf_node.hpp` — `bias_update_max_wz`
  gate on `correctGyroBias` (skip while rotating) + skipped-count in the throttled log;
  declare `gyro_scale_noise`
- **`src/go2_eskf/src/eskf_core.cpp` + `include/go2_eskf/eskf_core.hpp`** — Q switched to
  the continuous-time convention (`sigma^2*dt`) with `p-v` cross terms; new
  `Config::gyro_scale_noise` adding `(gyro_scale_noise*wz)^2` to `Q(psi,psi)`
- **`src/go2_eskf/scripts/eskf_reference.py`** — same change, in lockstep (cross-validation
  re-passes at 2.498e-15; if these two ever diverge, cross_validate.py is the tripwire)
- `src/go2_eskf/config/eskf_params.yaml` — `bias_update_max_wz: 0.10`,
  `gyro_scale_noise: 0.10`, **`accel_noise: 50.0 -> 5.0`** (compensates the dt^2 -> dt
  convention change; velocity behaviour intentionally unchanged)

**2026-07-28 (CHAMP leg-odom twist re-derivation — the sensor-level fix):**
- vendored `champ/include/champ/odometry/odometry.h` — `getVelocities` solves the body
  twist by centroid-reduced least squares over stance feet; touchdown samples gated via
  `prev_foot_contacts_`; bearing/`prev_theta_` derivation removed (§5.6). Rebuild `champ`
  AND `champ_base`.
- `src/go2_eskf/scripts/leg_odom_model.py` (new, installed) — offline trot model with an
  exactly known twist; ablates `bearing` / `lsq` / `lsq+td` / `champ_lsq` and self-checks
  the shipped algorithm to 1e-3. **No ROS, no Gazebo, no variance.** Run it after any
  change to that header — it is the tripwire if the Python port and the C++ diverge.
- `src/go2_eskf/CMakeLists.txt` — install `leg_odom_model.py`, `make_terrain_world.py`

**2026-07-28 (uneven-terrain world for the slip model — all first-party):**
- `src/go2_eskf/scripts/make_terrain_world.py` (new, installed) — generates the heightmap,
  its textures and `terrain.sdf`; `--relief` is the difficulty knob, `--patch-mu` /
  `--patch-size` / `--no-patches` control the slip patches
- `src/go2_eskf/worlds/terrain.sdf` + `terrain_height.png` + `terrain_diffuse.png` +
  `terrain_normal.png` (generated; regenerate after moving the workspace — the heightmap
  URI is necessarily absolute)
- `run_go2_teleop.sh` — `--terrain` flag, three-way world selection, mutual-exclusion check
  with `--obstacles`, and a stale-heightmap-path guard (a missing PNG loads a world with no
  ground, silently)

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

# Offline leg-odometry check — NO sim, NO GPU, zero run-to-run variance. Do this
# BEFORE booking sim time on any leg-odom/yaw-rate question (§0).
python3 src/go2_eskf/scripts/leg_odom_model.py    # expects PASS

# Sensor-level truth check (straight phase + in-place rotation, prints ratios).
# This is how the 1.83x leg yaw rate and the wrong gyro_z_sign were found — measure
# before theorising. Needs the sim + ground_truth.launch.py up first.
ros2 run go2_eskf probe_leg_odom.py

# Rebuild after the CHAMP odometry edit (header-only -> champ_base must rebuild too):
colcon build --packages-select champ champ_base go2_eskf --merge-install --symlink-install

# --- A/B a single parameter (the ONLY sound way to evaluate a tuning change) ---
# `run_go2_teleop.sh --square` opens gnome-terminals, so its output is awkward to
# capture. For scripted A/B, drive the pieces directly and override one parameter:
#
#   1) teardown, then launch sim + ground truth, logging to files:
#        ros2 launch unitree_go2_sim unitree_go2_launch.py use_sim_time:=true \
#          rviz:=false world:=$PWD/install/share/go2_eskf/worlds/flat.sdf &
#        ros2 launch go2_eskf ground_truth.launch.py &
#   2) poll until the leg controller is ACTIVE (~25-30 s; the only honest signal):
#        until ros2 control list_controllers | grep -q 'joint_group_effort_controller.*active'
#   3) start a FRESH node (clean params AND filter state) with the override + a log:
#        ros2 run go2_eskf go2_eskf_node --ros-args \
#          --params-file install/share/go2_eskf/config/eskf_params.yaml \
#          -p use_sim_time:=true -p ground_truth_topic:=/ground_truth/odom \
#          -p log_path:=$HOME/ab_<label>.csv -p <param>:=<value> &
#   4) ros2 run go2_eskf square_test.py --ros-args -p use_sim_time:=true
#   5) RELAUNCH THE WHOLE SIM for the next run. Do NOT use the gz world-reset
#      service to recycle it — it rewinds /clock and wedges controller_manager (§4).
#
# ~6.5 min per run. Budget 5+ runs per arm: identical configs give 4-97 deg of yaw
# error, so fewer runs than that cannot distinguish anything (§0).
#
# Post-process (yaw error is the quantity that matters — position error follows it):
#   python3 -c "import sys; sys.path.insert(0,'src/go2_eskf/scripts'); import metrics as M; \
#     import numpy as np; L=M.load_log('ab_x.csv'); \
#     print(np.degrees(np.abs(np.arctan2(np.sin(L['est_yaw']-L['gt_yaw']), \
#                                        np.cos(L['est_yaw']-L['gt_yaw'])))).mean())"

# Tests
./build/go2_eskf/test_eskf_core                       # 16 GTest
python3 src/go2_eskf/scripts/cross_validate.py         # C++≡NumPy ~1e-14
python3 src/go2_eskf/scripts/metrics.py --selftest     # metrics, no ROS/sim needed

# Read before touching the filter: src/go2_eskf/docs/DESIGN.md
```
