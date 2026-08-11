#!/usr/bin/env python3
"""One fixed, self-overwriting diagnostic snapshot per run — so a run can be debugged
from text instead of screenshots.

Writes into ONE directory (default `<workspace>/run_report/`), overwriting every file
each run, so disk use is bounded no matter how many runs you do:

    run_report/REPORT.md      <- read this. Everything below, rendered.
    run_report/timeseries.csv <- decimated merged time series (5 Hz, row-capped)
    run_report/context.txt    <- flags/world/gains, written by run_go2_teleop.sh
    run_report/node_logs/     <- per-node stdout, copied by run_go2_teleop.sh before
                                 it deletes its scratch log dir

WHAT IT CAPTURES, and why each item earned its place — these are the things that
previously had to be dug out of ~/.ros/log by hand or guessed at from a screenshot:

  outcome      distance travelled, and a sliding-window STALL detector (where the robot
               stopped advancing while still being commanded to move) — the single most
               common failure and the one a screenshot cannot timestamp.
  terrain      elevation AND slope under the ground-truth path, sampled from the world's
               own heightmap. Answers "was it the hill?" without re-deriving the mapping.
  gait health  per-leg contact duty, and JOINT TRACKING ERROR (commanded trajectory vs
               measured joint_states). Tracking error is the stance-sag metric: at the
               stock p=100 N*m/rad the legs give away ~3 cm of stroke before pushing
               (docs/SLOPE_POSTURE.md section 2), and that is invisible in any screenshot.
  leg odom     degenerate (all-zero) fraction — CHAMP's no-information flag, which also
               reads as "the robot is standing still".
  estimator    ATE / final / max position error and yaw error vs ground truth, one
               column PER ARM: the baseline /eskf/odom and, when it is running, the
               slip-adaptive /eskf_slip/odom (same inputs, slip-scaled R_leg).
  topics       message counts and rates, with MISSING topics called out loudly. Half the
               historical failures here are "a topic never appeared".

Runs alongside everything else and touches nothing:

    ros2 run go2_eskf run_report.py --ros-args -p use_sim_time:=true
    ./run_go2_teleop.sh --terrain --square          # on by default; --no-report disables

The report is rewritten every `flush_sec` as well as at shutdown, so a hard kill (or a
gz segfault) still leaves a usable snapshot of everything up to that moment.
"""

import csv
import math
import os
import subprocess
import sys
from collections import deque

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
from trajectory_msgs.msg import JointTrajectory

try:                                            # optional: only CHAMP publishes this
    from champ_msgs.msg import ContactsStamped
except ImportError:
    ContactsStamped = None

G = 9.80665
LEGS = ("lf", "rf", "lh", "rh")


def rpy(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(sinr, cosr), math.asin(sinp), math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stat(xs):
    """mean / median / max of a list, tolerant of empty."""
    if not xs:
        return None
    s = sorted(xs)
    n = len(s)
    med = s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])
    return sum(xs) / n, med, s[-1]


class Heightmap:
    """The world's own heightmap, sampled the way gz does.

    Mapping (measured, see CLAUDE.md): image column -> +X, row 0 -> +Y (max Y),
    elevation = pixel / (image max pixel) * size_z.
    """

    def __init__(self, png, extent, size_z):
        import numpy as np
        from PIL import Image
        a = np.asarray(Image.open(png).convert("L")).astype(float)
        self.z = a / max(a.max(), 1.0) * size_z
        self.n = a.shape[0]
        self.ext = extent
        self.cell = extent / (self.n - 1)
        drow, dcol = np.gradient(self.z, self.cell)
        self.gx, self.gy = dcol, -drow

    def _sample(self, grid, x, y):
        c = (x + self.ext / 2.0) / self.ext * (self.n - 1)
        r = (self.ext / 2.0 - y) / self.ext * (self.n - 1)
        if not (0 <= c <= self.n - 1 and 0 <= r <= self.n - 1):
            return None
        c0, r0 = int(c), int(r)
        c1, r1 = min(c0 + 1, self.n - 1), min(r0 + 1, self.n - 1)
        fc, fr = c - c0, r - r0
        return (grid[r0, c0] * (1 - fc) * (1 - fr) + grid[r0, c1] * fc * (1 - fr) +
                grid[r1, c0] * (1 - fc) * fr + grid[r1, c1] * fc * fr)

    def height(self, x, y):
        return self._sample(self.z, x, y)

    def slope_deg(self, x, y):
        gx, gy = self._sample(self.gx, x, y), self._sample(self.gy, x, y)
        if gx is None or gy is None:
            return None
        return math.degrees(math.atan(math.hypot(gx, gy)))

    @staticmethod
    def from_world(world_path):
        """Pull the heightmap out of an SDF. Returns None for flat/obstacle worlds."""
        import re
        try:
            sdf = open(world_path).read()
        except OSError:
            return None
        m = re.search(r"<heightmap>.*?<uri>file://(.*?)</uri>.*?"
                      r"<size>([\d.eE+-]+)\s+([\d.eE+-]+)\s+([\d.eE+-]+)</size>",
                      sdf, re.S)
        if not m or not os.path.isfile(m.group(1)):
            return None
        try:
            return Heightmap(m.group(1), float(m.group(2)), float(m.group(4)))
        except Exception:
            return None


class RunReport(Node):
    def __init__(self):
        super().__init__("run_report")

        here = os.path.dirname(os.path.abspath(__file__))
        ws_guess = os.path.abspath(os.path.join(here, *([os.pardir] * 3)))
        self.out_dir = self.declare_parameter(
            "out_dir", os.path.join(ws_guess, "run_report")).value
        self.csv_hz = float(self.declare_parameter("csv_hz", 5.0).value)
        self.max_rows = int(self.declare_parameter("max_rows", 20000).value)
        self.flush_sec = float(self.declare_parameter("flush_sec", 20.0).value)
        self.stall_window = float(self.declare_parameter("stall_window", 9.0).value)
        self.stall_dist = float(self.declare_parameter("stall_dist", 0.15).value)
        os.makedirs(self.out_dir, exist_ok=True)

        # --- latest sample of everything, merged into one CSV row at csv_hz
        self.gt = self.est = self.leg = None
        self.est_slip = None            # slip-adaptive arm (/eskf_slip/odom)
        self.imu = None
        self.cmd = (0.0, 0.0)
        self.body_pose = None
        self.contacts = None
        self.cmd_joints = None          # name -> commanded position
        self.meas_joints = None         # name -> measured position

        # --- accumulators
        self.rows = []
        self.row_stride = 1             # bumped when max_rows is hit (halves the rate)
        self.n_row = 0
        self.counts = {}
        self.first_t = {}
        self.last_t = {}
        self.t0 = None
        self.path_len = 0.0
        self.prev_xy = None
        self.err_pos, self.err_yaw = [], []
        self.err_pos_slip, self.err_yaw_slip = [], []
        self.optional = set()           # topics whose absence is not a fault
        self.slopes, self.elevs = [], []
        self.steepest = None            # (slope_deg, t, x, y)
        self.imu_pitch, self.accel_mag = [], []
        self.gt_att = []                # (|roll|, |pitch|) from ground truth
        self.jt_err = []                # per-sample max |commanded - measured|
        self.jt_err_by_joint = {}
        self.contact_on = [0] * 4
        self.contact_n = 0
        self.leg_degen = 0
        self.leg_n = 0
        self.track = deque()            # (t, x, y) for the stall detector
        self.worst_stall = None         # (displacement, t, x, y)
        self.hmap_tried = False
        self.hmap = None

        qd = 20
        self.sub(Odometry, "/ground_truth/odom", self.on_gt, qd)
        self.sub(Odometry, "/eskf/odom", self.on_est, qd)
        # Second estimator arm (slip-adaptive leg covariance), only up when the
        # launcher started it — optional, so its absence isn't flagged as a fault.
        self.sub(Odometry, "/eskf_slip/odom", self.on_est_slip, qd, optional=True)
        self.sub(Odometry, "/odom/raw", self.on_leg, qd)
        self.sub(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.sub(Twist, "/cmd_vel", self.on_cmd, qd)
        self.sub(Pose, "/body_pose", self.on_body_pose, qd)
        self.sub(JointState, "/joint_states", self.on_joints, qd)
        self.sub(JointTrajectory,
                 "/joint_group_effort_controller/joint_trajectory", self.on_jt_cmd, qd)
        if ContactsStamped is not None:
            self.sub(ContactsStamped, "/foot_contacts", self.on_contacts, qd)

        # Sampling runs on the (sim) clock so rows line up with the data. The FLUSH
        # timer deliberately runs on wall time: if gz dies or never starts, /clock never
        # advances, sim-time timers never fire, and the run that most needs a report
        # would produce none. This way the topic-health table still lands on disk.
        self.create_timer(1.0 / self.csv_hz, self.on_sample)
        self.create_timer(self.flush_sec, self.write,
                          clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self.get_logger().info(
            f"run_report -> {self.out_dir}/REPORT.md  (overwritten every run; "
            f"csv {self.csv_hz:g} Hz, flush {self.flush_sec:g}s)")

    def sub(self, typ, topic, cb, qos, optional=False):
        self.counts[topic] = 0
        if optional:
            self.optional.add(topic)

        def wrapped(msg, _t=topic, _cb=cb):
            now = self.now()
            self.counts[_t] += 1
            self.first_t.setdefault(_t, now)
            self.last_t[_t] = now
            _cb(msg)

        self.create_subscription(typ, topic, wrapped, qos)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- callbacks ---------------------------------------------------------
    def on_gt(self, m):
        p, q, t = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist
        r, pi, y = rpy(q)
        self.gt = (p.x, p.y, p.z, r, pi, y, t.linear.x, t.angular.z)

    def on_est(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.est = (p.x, p.y, rpy(q)[2])

    def on_est_slip(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.est_slip = (p.x, p.y, rpy(q)[2])

    def on_leg(self, m):
        t = m.twist.twist
        degen = (t.linear.x == 0.0 and t.linear.y == 0.0 and t.angular.z == 0.0)
        self.leg = (t.linear.x, t.angular.z, degen)
        self.leg_n += 1
        self.leg_degen += 1 if degen else 0

    def on_imu(self, m):
        a = m.linear_acceleration
        mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self.imu = (math.atan2(-a.x, math.hypot(a.y, a.z)),
                    math.atan2(a.y, a.z), mag)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_body_pose(self, m):
        self.body_pose = (m.position.x, m.position.z)

    def on_contacts(self, m):
        c = list(m.contacts)[:4]
        if len(c) == 4:
            self.contacts = c
            self.contact_n += 1
            for i, on in enumerate(c):
                self.contact_on[i] += 1 if on else 0

    def on_jt_cmd(self, m):
        if m.points and m.points[0].positions:
            self.cmd_joints = dict(zip(m.joint_names, m.points[0].positions))

    def on_joints(self, m):
        self.meas_joints = dict(zip(m.name, m.position))

    # ---- periodic merge ----------------------------------------------------
    def on_sample(self):
        t = self.now()
        if self.t0 is None:
            if self.gt is None and self.est is None:
                return                        # nothing alive yet; don't log dead air
            self.t0 = t
        rel = t - self.t0

        # joint tracking error: commanded trajectory vs measured joint_states.
        jmax = jmean = None
        if self.cmd_joints and self.meas_joints:
            errs = {k: abs(v - self.meas_joints[k])
                    for k, v in self.cmd_joints.items() if k in self.meas_joints}
            if errs:
                jmax, jmean = max(errs.values()), sum(errs.values()) / len(errs)
                self.jt_err.append(jmax)
                for k, v in errs.items():
                    self.jt_err_by_joint.setdefault(k, []).append(v)

        terr_z = terr_s = None
        if self.gt is not None:
            x, y = self.gt[0], self.gt[1]
            if self.prev_xy is not None:
                self.path_len += math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
            self.prev_xy = (x, y)

            hm = self.heightmap()
            if hm is not None:
                terr_z, terr_s = hm.height(x, y), hm.slope_deg(x, y)
                if terr_s is not None:
                    self.slopes.append(terr_s)
                    self.elevs.append(terr_z)
                    # Carry the location with the slope. Indexing back into self.rows
                    # would be wrong: rows are skipped before gt arrives and decimated
                    # on overflow, so the two lists are not aligned.
                    if self.steepest is None or terr_s > self.steepest[0]:
                        self.steepest = (terr_s, rel, x, y)

            # Stall detector: displacement over a trailing window. The window must be
            # commanded THROUGHOUT, not merely at its final sample — otherwise every run
            # trips on the warm-up, where the robot legitimately stands still for the
            # ~10 s before square_test starts driving and then gets one commanded sample.
            moving = abs(self.cmd[0]) > 1e-3 or abs(self.cmd[1]) > 1e-3
            self.track.append((rel, x, y, moving, self.gt[5]))
            while self.track and rel - self.track[0][0] > self.stall_window:
                self.track.popleft()
            if (self.track and rel - self.track[0][0] >= self.stall_window * 0.95
                    and all(s[3] for s in self.track)):
                # Progress is translation OR rotation. square_test turns IN PLACE at every
                # corner (measured: 260° of yaw for 35 cm of travel), so a position-only
                # test calls each corner a stall. Yaw is converted to an equivalent arc at
                # the hip offset so the two are comparable.
                d = math.hypot(x - self.track[0][1], y - self.track[0][2])
                dyaw = abs(wrap(self.gt[5] - self.track[0][4]))
                prog = d + 0.1934 * dyaw
                if self.worst_stall is None or prog < self.worst_stall[0]:
                    self.worst_stall = (prog, rel, x, y)

        if self.gt is not None and self.est is not None:
            self.err_pos.append(math.hypot(self.est[0] - self.gt[0],
                                           self.est[1] - self.gt[1]))
            self.err_yaw.append(abs(wrap(self.est[2] - self.gt[5])))
        if self.gt is not None and self.est_slip is not None:
            self.err_pos_slip.append(math.hypot(self.est_slip[0] - self.gt[0],
                                                self.est_slip[1] - self.gt[1]))
            self.err_yaw_slip.append(abs(wrap(self.est_slip[2] - self.gt[5])))
        if self.imu is not None:
            self.imu_pitch.append(self.imu[0])
            self.accel_mag.append(self.imu[2])
        if self.gt is not None:
            self.gt_att.append((abs(self.gt[3]), abs(self.gt[4])))

        g = self.gt or (None,) * 8
        e = self.est or (None,) * 3
        es = self.est_slip or (None,) * 3
        lg = self.leg or (None, None, None)
        im = self.imu or (None, None, None)
        bp = self.body_pose or (None, None)
        row = [round(rel, 3), *g, *e,
               self.err_pos[-1] if self.err_pos else None,
               self.err_yaw[-1] if self.err_yaw else None,
               *es,
               self.err_pos_slip[-1] if self.err_pos_slip else None,
               self.err_yaw_slip[-1] if self.err_yaw_slip else None,
               *lg, *im, *self.cmd, *bp,
               "".join("1" if c else "0" for c in self.contacts) if self.contacts else None,
               jmax, jmean, terr_z, terr_s]

        # Row cap: never grow without bound. On overflow, throw away every other row
        # and halve the effective rate — the run stays fully represented, at half detail.
        self.n_row += 1
        if self.n_row % self.row_stride:
            return
        self.rows.append(row)
        if len(self.rows) >= self.max_rows:
            self.rows = self.rows[::2]
            self.row_stride *= 2

    def heightmap(self):
        if not self.hmap_tried:
            self.hmap_tried = True
            world = self.run_context().get("World file")
            if world:
                try:
                    self.hmap = Heightmap.from_world(world)
                except Exception as ex:
                    self.get_logger().warn(f"heightmap unavailable: {ex}")
        return self.hmap

    def run_context(self):
        """key: value pairs the launcher dropped in context.txt (may be absent)."""
        if not hasattr(self, "_ctx"):
            self._ctx = {}
            try:
                for line in open(os.path.join(self.out_dir, "context.txt")):
                    if ":" in line:
                        k, v = line.split(":", 1)
                        self._ctx[k.strip()] = v.strip()
            except OSError:
                pass
        return self._ctx

    # ---- output ------------------------------------------------------------
    def write(self):
        try:
            self.write_csv()
        except Exception as ex:
            self.get_logger().error(f"csv write failed: {ex}")
        try:
            self.write_report()
        except Exception as ex:
            self.get_logger().error(f"report write failed: {ex}")

    def write_csv(self):
        head = ["t", "gt_x", "gt_y", "gt_z", "gt_roll", "gt_pitch", "gt_yaw",
                "gt_vx", "gt_wz", "est_x", "est_y", "est_yaw", "err_pos", "err_yaw",
                "slip_x", "slip_y", "slip_yaw", "slip_err_pos", "slip_err_yaw",
                "leg_vx", "leg_wz", "leg_degenerate", "imu_pitch", "imu_roll",
                "accel_mag", "cmd_vx", "cmd_wz", "body_pose_x", "body_pose_z",
                "contacts_lf_rf_lh_rh", "joint_err_max", "joint_err_mean",
                "terrain_z", "terrain_slope_deg"]
        path = os.path.join(self.out_dir, "timeseries.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(head)
            w.writerows(self.rows)

    def git(self):
        try:
            # The repo is where the SOURCE lives, not where the report is written —
            # out_dir can be pointed anywhere.
            root = os.path.dirname(os.path.abspath(__file__))
            sha = subprocess.run(["git", "-C", root, "log", "-1", "--format=%h %s"],
                                 capture_output=True, text=True, timeout=5).stdout.strip()
            dirty = subprocess.run(["git", "-C", root, "status", "--porcelain"],
                                   capture_output=True, text=True, timeout=5).stdout
            n = len([x for x in dirty.splitlines() if x.strip()])
            return f"{sha}" + (f"  (+{n} uncommitted file{'s' * (n != 1)})" if n else "  (clean)")
        except Exception:
            return "unavailable"

    def log_issues(self):
        """WARN/ERROR lines from the per-node logs the launcher copied in."""
        d = os.path.join(self.out_dir, "node_logs")
        if not os.path.isdir(d):
            return []
        out = []
        for fn in sorted(os.listdir(d)):
            try:
                lines = open(os.path.join(d, fn), errors="replace").read().splitlines()
            except OSError:
                continue
            hits = [ln for ln in lines
                    if "[ERROR]" in ln or "[FATAL]" in ln or "[WARN]" in ln
                    or "Segmentation" in ln or "segfault" in ln]
            seen, uniq = set(), []
            for ln in hits:                       # collapse repeats, keep order
                key = ln[ln.find("]") + 1:][:110]
                if key not in seen:
                    seen.add(key)
                    uniq.append(ln.strip())
            if uniq:
                out.append((fn, len(hits), uniq[:8]))
        return out

    def write_report(self):
        L = []
        A = L.append
        dur = (self.rows[-1][0] if self.rows else 0.0)

        A("# Go2 run report")
        A("")
        A("Auto-generated, **overwritten every run**. Raw series: `timeseries.csv`.")
        A("")
        A(f"- generated : {subprocess.run(['date'], capture_output=True, text=True).stdout.strip()}")
        A(f"- duration  : {dur:.1f} s of data, {len(self.rows)} rows "
          f"@ {self.csv_hz / self.row_stride:g} Hz")
        A(f"- git       : {self.git()}")
        ctx = self.run_context()
        if ctx:
            A("")
            A("## Configuration")
            A("")
            A("| setting | value |")
            A("|---|---|")
            for k, v in ctx.items():
                A(f"| {k} | {v} |")
        A("")

        # ---- outcome
        A("## Outcome")
        A("")
        A("| metric | value |")
        A("|---|---|")
        A(f"| ground-truth path length | {self.path_len:.2f} m |")
        if self.gt:
            A(f"| final truth pose | ({self.gt[0]:+.2f}, {self.gt[1]:+.2f}, "
              f"{self.gt[2]:+.2f}) m, yaw {math.degrees(self.gt[5]):+.1f}° |")
            A(f"| final truth attitude | roll {math.degrees(self.gt[3]):+.1f}°, "
              f"pitch {math.degrees(self.gt[4]):+.1f}° |")
        if self.worst_stall:
            d, t, x, y = self.worst_stall
            verdict = "**STALLED**" if d < self.stall_dist else "ok"
            A(f"| worst {self.stall_window:.0f} s progress (while commanded) | "
              f"{d * 100:.1f} cm at t={t:.1f}s, ({x:+.2f}, {y:+.2f}) — {verdict} |")
        else:
            A(f"| worst {self.stall_window:.0f} s progress | never commanded to move "
              f"for a full window |")
        A("")

        # ---- terrain
        if self.slopes:
            s = stat(self.slopes)
            e = stat(self.elevs)
            A("## Terrain under the ground-truth path")
            A("")
            A(f"- slope     : median {s[1]:.1f}°, mean {s[0]:.1f}°, **max {s[2]:.1f}°**")
            A(f"- elevation : {min(self.elevs):.2f} .. {max(self.elevs):.2f} m "
              f"(median {e[1]:.2f})")
            if self.steepest:
                sl, t, x, y = self.steepest
                A(f"- steepest point : {sl:.1f}° at t={t:.1f}s, ({x:+.2f}, {y:+.2f})")
            A("")
        if self.gt_att:
            r = stat([a[0] for a in self.gt_att])
            p = stat([a[1] for a in self.gt_att])
            A(f"Body attitude (**ground truth**): |pitch| median {math.degrees(p[1]):.1f}°, "
              f"max {math.degrees(p[2]):.1f}°; |roll| median {math.degrees(r[1]):.1f}°, "
              f"max {math.degrees(r[2]):.1f}°.")
            A("")
        if self.imu_pitch:
            p = stat([abs(x) for x in self.imu_pitch])
            A(f"Body pitch from **instantaneous** IMU accel: median "
              f"{math.degrees(p[1]):.1f}°, max {math.degrees(p[2]):.1f}°. Treat the max as "
              f"an artifact, not attitude — during a trot the accelerometer is rotating "
              f"and spiking on contact, so single samples read far past the true tilt "
              f"(measured: 67.8° here against a ground truth of 14.8°). Attitude needs the "
              f"low-pass `terrain_adapt.py` applies; only the median is meaningful.")
            A("")

        # ---- gait health
        A("## Gait health")
        A("")
        if self.contact_n:
            A("| leg | stance duty |")
            A("|---|---|")
            for i, leg in enumerate(LEGS):
                A(f"| {leg} | {self.contact_on[i] / self.contact_n * 100:.1f} % |")
            A("")
        if self.leg_n:
            A(f"- leg odom `/odom/raw`: {self.leg_n} msgs, "
              f"**{self.leg_degen / self.leg_n * 100:.1f} % degenerate** (all-zero). "
              f"CHAMP emits hard zeros only when all four or zero feet are in contact, "
              f"so a high fraction means the robot spent that time standing still.")
            A("")
        if self.jt_err:
            j = stat(self.jt_err)
            A(f"- joint tracking error (commanded trajectory vs measured), worst joint "
              f"per sample: mean {math.degrees(j[0]):.2f}°, median {math.degrees(j[1]):.2f}°, "
              f"max {math.degrees(j[2]):.2f}°")
            per = sorted(((stat(v)[0], k) for k, v in self.jt_err_by_joint.items()),
                         reverse=True)[:4]
            A("- worst joints by mean error: " +
              ", ".join(f"`{k}` {math.degrees(m):.2f}°" for m, k in per))
            A(f"- at p=100 N·m/rad a 0.134 rad (7.7°) error is ~74 N of push and ~2.9 cm "
              f"of foot sag; see `docs/SLOPE_POSTURE.md` §2.")
            A("")
        if self.accel_mag:
            a = stat(self.accel_mag)
            A(f"- IMU |accel|: median {a[1]:.2f}, max {a[2]:.1f} m/s² "
              f"(g = {G:.2f}; large peaks are contact impacts).")
            A("")

        # ---- estimator. Two arms when the slip-adaptive instance is running
        # (/eskf_slip/odom): same inputs, same tuning, slip-scaled R_leg — so the
        # columns are a within-run A/B and the difference IS the slip model.
        if self.err_pos:
            arms = [("baseline (fixed R)", self.err_pos, self.err_yaw)]
            if self.err_pos_slip:
                arms.append(("slip-adaptive", self.err_pos_slip, self.err_yaw_slip))
            A("## Estimator vs ground truth")
            A("")
            A("| metric | " + " | ".join(n for n, _, _ in arms) + " |")
            A("|---|" + "---|" * len(arms))

            def row(label, fn):
                A(f"| {label} | " + " | ".join(fn(ep, ey) for _, ep, ey in arms) + " |")

            row("position error (ATE mean)", lambda ep, ey: f"{stat(ep)[0]:.3f} m")
            row("position error max / final",
                lambda ep, ey: f"{stat(ep)[2]:.3f} m / {ep[-1]:.3f} m")
            row("yaw error mean / final",
                lambda ep, ey: f"{math.degrees(stat(ey)[0]):.1f}° / "
                               f"{math.degrees(ey[-1]):.1f}°")
            if self.path_len > 1.0:
                row("final error as % of path",
                    lambda ep, ey: f"{ep[-1] / self.path_len * 100:.2f} %")
            A("")
            if len(arms) == 1:
                A("Only one arm ran — start the slip-adaptive one with "
                  "`eskf.launch.py slip:=true` (the launcher does it unless "
                  "`--no-slip`) to get the comparison column.")
                A("")
            else:
                d = self.err_pos_slip[-1] - self.err_pos[-1]
                A(f"Slip-adaptive final position error is {abs(d):.3f} m "
                  f"{'WORSE' if d > 0 else 'BETTER'} than baseline on this run. "
                  "One run proves nothing — square results are not repeatable "
                  "(`skills.md` §0); budget ~5 runs per arm.")
                A("")
            A("Position error tracks yaw error ~1:1 with GPS off — yaw is exactly "
              "unobservable there, so read the yaw row first (`skills.md` §0).")
            A("")

        # ---- topic health
        A("## Topic health")
        A("")
        A("| topic | msgs | rate |")
        A("|---|---|---|")
        for t, n in self.counts.items():
            if n == 0:
                if t in self.optional:
                    A(f"| `{t}` | 0 — not running (optional) | — |")
                else:
                    A(f"| `{t}` | **0 — NEVER RECEIVED** | — |")
            else:
                span = self.last_t[t] - self.first_t[t]
                A(f"| `{t}` | {n} | {n / span if span > 0.1 else float('nan'):.1f} Hz |")
        A("")
        if ContactsStamped is None:
            A("> `champ_msgs` not importable — `/foot_contacts` was not recorded.")
            A("")

        # ---- log issues
        issues = self.log_issues()
        if issues:
            A("## Warnings & errors from node logs")
            A("")
            for fn, total, lines in issues:
                A(f"**{fn}** ({total} lines, unique shown):")
                A("```")
                for ln in lines:
                    A(ln[:200])
                A("```")
            A("")

        path = os.path.join(self.out_dir, "REPORT.md")
        with open(path, "w") as f:
            f.write("\n".join(L) + "\n")


def main():
    rclpy.init()
    node = RunReport()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write()
        node.get_logger().info(f"wrote {node.out_dir}/REPORT.md")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
