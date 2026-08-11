#!/usr/bin/env python3
"""Slope-adaptive body posture for the CHAMP-driven Go2 — publishes /body_pose.

WHY THIS EXISTS
CHAMP is blind and has ZERO terrain adaptation. `quadruped_controller.cpp` sets
`req_pose_.position.z = nominal_height` once at startup and never touches orientation;
`req_pose_` changes only when something publishes the `/body_pose` topic, and nothing in
`unitree_go2_launch.py` does. So `BodyController::poseCommand` holds all four feet on a
single plane 0.225 m below the hips in the BASE frame at zero roll/pitch, forever. On a
sustained incline the front feet contact early and the rear reach into air, the body
pitches, stance stroke is lost, and the trot degenerates into stepping in place — the
"no progress for 9.0 s while commanded to move" stall `square_test` reports on terrain.

This node closes that gap using the interface CHAMP already exposes. It estimates the
body's attitude against gravity and asks for a posture that suits the slope:

  position.x  shift the body UPHILL so the CoM stays inside the support polygon.
              On a climb the CoM drifts rearward, unloading the front feet until they
              slip or the robot rears. sin(pitch) carries the sign, so a descent
              shifts the body back automatically.
  position.z  crouch on slopes. Lower CoM, shorter overturning moment. This is a DELTA
              on nominal_height (`cmdPoseCallback_` adds gait_config_.nominal_height),
              so publishing 0.0 is the stock stance — there is no collapse trap here.
  orientation pitch/roll the body relative to the ground. 0 (the default) leaves the
              body parallel to the terrain, which is stock behaviour; 1.0 holds it level
              in the world. This one is genuine FEEDBACK on the measured attitude, so it
              is off by default — raise it deliberately and watch for oscillation.

With every gain at 0 the published pose is identity and behaviour is bit-for-bit stock,
so this is a strict superset of the current controller. That also makes it a clean A/B
arm: run it or don't.

ATTITUDE SOURCE. The gz IMU's ORIENTATION field is unreliable (see DESIGN.md — it is why
the ESKF defaults to `gravity_lp`), and the ESKF's own state is 8-dimensional and carries
yaw only, no roll/pitch. So attitude comes from the accelerometer here too: low-pass the
specific force to recover the gravity direction. Contact impacts are rejected by
magnitude before they enter the filter. The cost is lag — at tau=0.7 s and 0.25 m/s the
posture trails the ground by ~18 cm, which is fine for terrain whose slope changes over
metres and useless for a step change.

  ros2 run go2_eskf terrain_adapt.py --ros-args -p use_sim_time:=true
  ros2 run go2_eskf terrain_adapt.py --ros-args -p use_sim_time:=true \
      -p com_shift_x:=0.20 -p crouch:=0.12 -p level_pitch:=0.5
"""

import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

G = 9.80665


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (sr * cp * cy - cr * sp * sy,      # x
            cr * sp * cy + sr * cp * sy,      # y
            cr * cp * sy - sr * sp * cy,      # z
            cr * cp * cy + sr * sp * sy)      # w


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


class TerrainAdapt(Node):
    def __init__(self):
        super().__init__("terrain_adapt")

        # --- posture gains. All zero == stock CHAMP.
        self.declare_parameter("com_shift_x", 0.15)   # [m per unit sin(pitch)] + = uphill
        self.declare_parameter("com_shift_y", 0.00)   # [m per unit sin(roll)]  lateral
        self.declare_parameter("crouch", 0.10)        # [m per unit |sin(pitch)|] lower body
        self.declare_parameter("level_pitch", 0.0)    # 0 = follow terrain, 1 = level in world
        self.declare_parameter("level_roll", 0.0)
        # --- filter / safety
        self.declare_parameter("tau", 0.7)            # [s] gravity low-pass time constant
        self.declare_parameter("rate", 50.0)          # [Hz] /body_pose publish rate
        self.declare_parameter("accel_gate", 0.35)    # reject |f| outside (1+-gate)*g
        self.declare_parameter("max_shift", 0.06)     # [m] clamp on position.x/y
        self.declare_parameter("max_crouch", 0.06)    # [m] clamp on downward position.z
        self.declare_parameter("max_tilt", 0.35)      # [rad] clamp on commanded roll/pitch
        self.declare_parameter("report_sec", 5.0)     # 0 disables the periodic log

        g = self.get_parameter
        self.k_x = g("com_shift_x").value
        self.k_y = g("com_shift_y").value
        self.k_z = g("crouch").value
        self.k_pitch = g("level_pitch").value
        self.k_roll = g("level_roll").value
        self.tau = max(float(g("tau").value), 1e-3)
        self.rate = float(g("rate").value)
        self.gate = float(g("accel_gate").value)
        self.max_shift = float(g("max_shift").value)
        self.max_crouch = float(g("max_crouch").value)
        self.max_tilt = float(g("max_tilt").value)
        self.report_sec = float(g("report_sec").value)

        # Low-passed specific force, seeded to "level and at rest" so the first
        # published pose is identity rather than a lurch.
        self.f = [0.0, 0.0, G]
        self.have_imu = False
        self.n_imu = 0
        self.n_rejected = 0
        self.last_t = None
        self.last_report = None
        self.peak_pitch = 0.0

        self.pub = self.create_publisher(Pose, "/body_pose", 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_timer(1.0 / self.rate, self.on_timer)

        self.get_logger().info(
            f"terrain_adapt up -> /body_pose at {self.rate:.0f} Hz. "
            f"com_shift_x={self.k_x} crouch={self.k_z} level_pitch={self.k_pitch} "
            f"tau={self.tau}s. Waiting for /imu/data...")

    # ---- attitude from low-passed gravity -----------------------------------
    def on_imu(self, msg):
        a = msg.linear_acceleration
        mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        # Contact impacts spike the accelerometer far past g (DESIGN.md: this is why
        # accel_noise is huge in the ESKF). They carry no attitude information, so they
        # are dropped rather than averaged in.
        if not (G * (1.0 - self.gate) < mag < G * (1.0 + self.gate)):
            self.n_rejected += 1
            return

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.0 if self.last_t is None else t - self.last_t
        self.last_t = t
        if not (0.0 < dt < 0.5):        # first sample, or a clock jump
            dt = 1.0 / 200.0

        alpha = dt / (self.tau + dt)
        self.f[0] += alpha * (a.x - self.f[0])
        self.f[1] += alpha * (a.y - self.f[1])
        self.f[2] += alpha * (a.z - self.f[2])
        self.have_imu = True
        self.n_imu += 1

    def attitude(self):
        """Body roll/pitch against gravity, from the low-passed specific force.

        At rest the accelerometer reads +g expressed in the body frame, so a nose-up
        pitch theta gives f = (-g sin(theta), 0, g cos(theta)).
        """
        fx, fy, fz = self.f
        pitch = math.atan2(-fx, math.hypot(fy, fz))
        roll = math.atan2(fy, fz)
        return roll, pitch

    # ---- posture command ----------------------------------------------------
    def on_timer(self):
        if not self.have_imu:
            return

        roll, pitch = self.attitude()
        roll = clamp(roll, -self.max_tilt, self.max_tilt)
        pitch = clamp(pitch, -self.max_tilt, self.max_tilt)
        self.peak_pitch = max(self.peak_pitch, abs(pitch))

        p = Pose()
        # position.x/y are body offsets; poseCommand negates them onto the feet, so
        # +x moves the BODY forward. sin() keeps the sign right on a descent.
        p.position.x = clamp(self.k_x * math.sin(pitch), -self.max_shift, self.max_shift)
        p.position.y = clamp(self.k_y * math.sin(roll), -self.max_shift, self.max_shift)
        # Delta on nominal_height, negative = crouch.
        p.position.z = clamp(-self.k_z * abs(math.sin(pitch)), -self.max_crouch, 0.0)
        # poseCommand applies RotateY(-pitch), i.e. commanding +pitch pitches the body
        # nose-up RELATIVE TO THE FOOT PLANE. To hold the body flatter than the ground
        # we must therefore command the negative of the measured world attitude, scaled.
        qx, qy, qz, qw = quat_from_rpy(-self.k_roll * roll, -self.k_pitch * pitch, 0.0)
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        self.pub.publish(p)

        if self.report_sec > 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.last_report is None:
                self.last_report = now
            elif now - self.last_report >= self.report_sec:
                self.last_report = now
                self.get_logger().info(
                    f"pitch={math.degrees(pitch):+5.1f} deg (peak "
                    f"{math.degrees(self.peak_pitch):4.1f}) roll={math.degrees(roll):+5.1f} "
                    f"-> body x={p.position.x:+.3f} z={p.position.z:+.3f} m, "
                    f"cmd pitch={math.degrees(-self.k_pitch * pitch):+5.1f} deg | "
                    f"imu {self.n_imu} used / {self.n_rejected} impact-rejected")


def main():
    rclpy.init()
    node = TerrainAdapt()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
