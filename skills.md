# skills.md — go2_eskf sim-integration knowledge & session handoff

Working log of the go2_eskf ↔ Unitree Go2 sim integration debugging. Read this
first when resuming — it captures hard-won findings that aren't obvious from the
code. (Companion to `CLAUDE.md`; this file is the narrative + current state.)

Last updated: 2026-07-28 — see §0 "resume here". Current headline: the sensor-level yaw
defect was FOUND AND FIXED analytically (CHAMP's leg odometry mis-derived the body twist);
and yaw is provably UNOBSERVABLE in the ESKF with GPS off, so no filter tuning could ever
have fixed the heading drift. Live A/B still pending.

---

## 0. CURRENT STATE (start here)

**LATEST (2026-07-28 — CHAMP leg odometry re-derived as a least-squares body twist; two
defects found and fixed OFFLINE with zero run-to-run variance. Plus the observability
result that explains why six runs of filter tuning did nothing. Resume here.)**

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

**Why an R_leg change moves YAW at all** (the thing to not re-derive): inflating `R_leg`
weakens `correctLegOdom`, and although the yaw direction `δv = δψ·(-v_y, v_x)` is exactly
unobservable to it (§0 HEADLINE 2), the *rest* of that update is not — a weaker leg
correction means less of everything, so psi is left running closer to open-loop gyro
integration. The slip arm's `r_leg_yaw_bias` is NOT scaled by slip (checked
`legCovarianceForUpdate` — only `R_leg` is), so this is not the bias path.

**Do not conclude the slip model is bad from this.** n=1, and run-to-run variance is 4-97
deg of yaw error at fixed config. What this run DOES establish: the plumbing works (both
arms 25,982 msgs at 100.0 Hz off one sensor stream, 1,298 CSV rows, both columns populated),
and inflating `R_leg` on this terrain costs yaw stability.

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
