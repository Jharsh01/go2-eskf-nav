#!/usr/bin/env python3
"""Generate the EKF hardware-porting PDF using fpdf2."""
from fpdf import FPDF
from pathlib import Path


class Doc(FPDF):
    def header(self):
        if self.page_no() == 1:
            return
        self.set_font("Helvetica", "I", 9)
        self.set_text_color(120, 120, 120)
        self.cell(0, 6, "EKF Hardware Porting - robotics_ws", align="L")
        self.cell(0, 6, f"Page {self.page_no()}", align="R")
        self.ln(8)
        self.set_text_color(0, 0, 0)

    def footer(self):
        pass

    def _reset_x(self):
        self.set_x(self.l_margin)

    def h1(self, text):
        self.set_font("Helvetica", "B", 18)
        self.set_text_color(20, 60, 130)
        if self.get_y() > 240:
            self.add_page()
        self.ln(2)
        self._reset_x()
        self.multi_cell(0, 9, text)
        self.set_text_color(0, 0, 0)
        self.ln(2)

    def h2(self, text):
        self.set_font("Helvetica", "B", 14)
        self.set_text_color(20, 60, 130)
        if self.get_y() > 250:
            self.add_page()
        self.ln(3)
        self._reset_x()
        self.multi_cell(0, 7, text)
        self.set_text_color(0, 0, 0)
        self.ln(1)

    def h3(self, text):
        self.set_font("Helvetica", "B", 12)
        self.set_text_color(60, 60, 60)
        if self.get_y() > 252:
            self.add_page()
        self.ln(2)
        self._reset_x()
        self.multi_cell(0, 6, text)
        self.set_text_color(0, 0, 0)

    def para(self, text):
        self.set_font("Helvetica", "", 10.5)
        self._reset_x()
        self.multi_cell(0, 5.2, text)
        self.ln(1.5)

    def code(self, text):
        self.set_font("Courier", "", 9)
        self.set_fill_color(245, 245, 248)
        self.set_draw_color(220, 220, 230)
        # Wrap manually for code; fpdf doesn't preserve indent well otherwise.
        for line in text.split("\n"):
            if len(line) > 95:
                line = line[:92] + "..."
            self._reset_x()
            self.cell(0, 4.8, " " + line, border=0, fill=True)
            self.ln(4.8)
        self.set_font("Helvetica", "", 10.5)
        self.ln(2)

    def bullet(self, text):
        self.set_font("Helvetica", "", 10.5)
        self._reset_x()
        self.cell(5)
        self.cell(4, 5, "-")
        self.multi_cell(0, 5, text)

    def table(self, headers, rows, col_widths):
        self.set_font("Helvetica", "B", 9.5)
        self.set_fill_color(235, 240, 250)
        for h, w in zip(headers, col_widths):
            self.cell(w, 6, h, border=1, fill=True)
        self.ln()
        self.set_font("Helvetica", "", 9.5)
        for row in rows:
            # Compute row height to fit longest cell.
            heights = []
            for txt, w in zip(row, col_widths):
                lines = self.multi_cell(w, 4.5, txt, dry_run=True, output="LINES")
                heights.append(max(1, len(lines)) * 4.5)
            row_h = max(heights)
            y0 = self.get_y()
            x0 = self.get_x()
            for txt, w in zip(row, col_widths):
                self.multi_cell(w, 4.5, txt, border=1, max_line_height=4.5)
                # Restore for next cell on same row
                self.set_xy(x0 + w + 0, y0)
                x0 += w
            self.set_y(y0 + row_h)
        self.ln(2)


def main():
    pdf = Doc(format="A4", unit="mm")
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.set_margins(left=18, top=15, right=18)
    pdf.add_page()

    # ---- Title page
    pdf.set_font("Helvetica", "B", 22)
    pdf.set_text_color(20, 60, 130)
    pdf.ln(40)
    pdf.cell(0, 12, "EKF Measurement Models", align="L")
    pdf.ln(14)
    pdf.set_font("Helvetica", "", 14)
    pdf.set_text_color(60, 60, 60)
    pdf.cell(0, 8, "Hardware Porting Guide", align="L")
    pdf.ln(9)
    pdf.set_font("Helvetica", "", 12)
    pdf.cell(0, 7, "MPU6050 IMU, RPLiDAR, wheel encoders", align="L")
    pdf.ln(8)
    pdf.ln(6)
    pdf.set_font("Helvetica", "I", 11)
    pdf.set_text_color(80, 80, 80)
    pdf.set_x(pdf.l_margin)
    pdf.cell(0, 6, "Package: ekf_estimator | Target platform: ROS 2 Jazzy")
    pdf.ln(7)
    pdf.set_x(pdf.l_margin)
    pdf.cell(0, 6, "Generated 2026-05-13")
    pdf.ln(7)
    pdf.set_text_color(0, 0, 0)

    pdf.add_page()

    # ---- Overview
    pdf.h1("1. Overview of changes")
    pdf.para(
        "The EKF in this workspace fuses three sensor inputs into a 5-state "
        "estimate x = [x, y, theta, v, omega]^T. Before this round of edits "
        "the ROS 2 wrapper was already wired for hardware-standard message "
        "types (sensor_msgs/Imu, sensor_msgs/LaserScan, nav_msgs/Odometry), "
        "but the LiDAR path was a stub (no scan-matching) and a few "
        "sim-only behaviours leaked into the runtime. This document records "
        "what changed, why, and how to deploy the same node on a real "
        "diff-drive robot equipped with an MPU6050 IMU, an RPLiDAR, and "
        "wheel encoders.")

    pdf.h2("1.1  Files touched")
    pdf.table(
        headers=["File", "Change"],
        rows=[
            ["src/ekf_estimator/include/ekf_estimator/ekf_node.hpp",
             "Added ICP helper declarations, gyro-bias / scan-matching "
             "parameters, and scan-pose accumulator state."],
            ["src/ekf_estimator/src/ekf_node.cpp",
             "Wrote real scan-to-scan 2D ICP with SVD-based rigid alignment; "
             "added gyro-bias subtraction in the IMU path; gated the "
             "ground-truth path behind a flag; documented every measurement "
             "model in-source."],
            ["src/ekf_estimator/config/ekf_params.yaml",
             "Added imu_yaw_rate_bias, use_ground_truth, use_scan_matching, "
             "and ICP tuning parameters."],
            ["src/ekf_estimator/config/ekf_params_hardware.yaml",
             "New file. Same shape as ekf_params.yaml but with "
             "use_ground_truth=false and noise values tuned for real "
             "sensors."],
            ["src/motion_planner/launch/hardware_demo.launch.py",
             "New file. Brings up the EKF + nav stack WITHOUT Gazebo or "
             "the ros_gz bridge; expects real-hardware drivers running "
             "externally."],
        ],
        col_widths=[78, 96],
    )

    # ---- Section 2 - state and prediction
    pdf.h1("2. The estimator at a glance")
    pdf.h2("2.1  State, prediction, covariance")
    pdf.para(
        "The state is the planar pose plus body velocities. Each prediction "
        "step is a closed-form integration of the unicycle model with the "
        "current heading and twist held constant over dt. v and omega are "
        "modelled as Gaussian random walks: the mean does not evolve, but "
        "Q absorbs the uncertainty so v and omega can drift between "
        "measurement updates.")
    pdf.code(
        "x_{k+1|k}(0) = x(0) + v cos(theta) dt\n"
        "x_{k+1|k}(1) = x(1) + v sin(theta) dt\n"
        "x_{k+1|k}(2) = wrap(theta + omega dt)\n"
        "x_{k+1|k}(3) = v\n"
        "x_{k+1|k}(4) = omega\n"
        "\n"
        "P_{k+1|k} = F P F^T + Q dt"
    )
    pdf.para(
        "F is the Jacobian of the propagation w.r.t. the state, evaluated "
        "at the current x. The implementation (ekf_core.cpp) uses Joseph "
        "form for the covariance update so symmetry is preserved across "
        "Kalman gain rounding.")

    pdf.h2("2.2  Measurement updates")
    pdf.para(
        "Each sensor provides a partial measurement that is fused into the "
        "filter through its own linear H matrix and noise covariance R. "
        "There are three updates today, each implemented in ekf_core.cpp:")
    pdf.bullet("updateImuYawRate(omega, R) - z = omega (scalar). H picks the omega state.")
    pdf.bullet("updateWheelOdom(z, R)      - z = [v, omega]. H picks v and omega.")
    pdf.bullet("updatePose(z, R)           - z = [x, y, theta]. H picks the pose states.")
    pdf.para(
        "The heading innovation is wrapped to [-pi, pi] before the update "
        "so we never apply a 6 rad correction when the true error is 0.28 rad.")

    # ---- Section 3 - per sensor mapping
    pdf.h1("3. Sensor mapping - sim to hardware")

    # IMU
    pdf.h2("3.1  IMU - MPU6050")
    pdf.para(
        "The MPU6050 is a 6-DoF IMU: a 3-axis gyroscope plus a 3-axis "
        "accelerometer, with no magnetometer. For a planar robot we use "
        "only the body-frame z-gyro (yaw rate). The accelerometer is "
        "unused in this filter because wheel encoders give a less-noisy "
        "linear velocity.")
    pdf.h3("Message contract")
    pdf.code(
        "topic : /imu\n"
        "type  : sensor_msgs/Imu\n"
        "fields used :\n"
        "    angular_velocity.z         (rad/s)\n"
        "everything else is ignored.")
    pdf.para(
        "Any MPU6050 ROS 2 driver that publishes sensor_msgs/Imu drops in "
        "directly. Examples on Jazzy:")
    pdf.bullet("hytseng0509/mpu6050-ros2 (I2C, Raspberry Pi)")
    pdf.bullet("the linorobot2 hardware bring-up package")
    pdf.bullet("any custom node using smbus / pigpio to drive the chip")
    pdf.h3("Bias calibration (required)")
    pdf.para(
        "MPU6050 gyros have a ~10-50 mrad/s static bias that varies with "
        "temperature and supply voltage. Without subtraction, the EKF will "
        "spin its theta estimate at the bias rate whenever the robot is "
        "stationary. Calibrate after ~30 s of warm-up:")
    pdf.code(
        "# 1. Leave the robot motionless and run for ~5 s.\n"
        "ros2 topic echo /imu --field angular_velocity.z > /tmp/gz.log\n"
        "# 2. Average the samples.\n"
        "python3 -c \"import statistics, sys; \\\n"
        "    vs=[float(l) for l in open('/tmp/gz.log') if l.strip() \\\n"
        "    and l.strip().replace('.','').replace('-','').replace('e','').isdigit()]; \\\n"
        "    print(statistics.mean(vs))\"\n"
        "# 3. Put the NEGATED value into ekf_params_hardware.yaml as\n"
        "#    imu_yaw_rate_bias.")
    pdf.para(
        "Re-run after the robot has been on for several minutes; warm "
        "bias often differs by 1-3 mrad/s from cold bias.")
    pdf.h3("Noise variance")
    pdf.para(
        "The default imu_yaw_rate_var = 3e-4 rad^2/s^2 is what an MPU6050 "
        "produces at the typical 100 Hz output rate, sitting on a "
        "vibrating chassis. Tighten if mounted on a damped platform, "
        "loosen if directly on a metal frame with motor noise.")

    # LiDAR
    pdf.h2("3.2  LiDAR - RPLiDAR")
    pdf.para(
        "Both the Gazebo gpu_lidar and the rplidar_ros driver publish the "
        "same sensor_msgs/LaserScan message, so the EKF callback is "
        "platform-agnostic. The scan-matching update implemented here is "
        "point-to-point 2D ICP between consecutive scans; the integrated "
        "delta gives a pose measurement.")
    pdf.h3("Message contract")
    pdf.code(
        "topic : /scan\n"
        "type  : sensor_msgs/LaserScan\n"
        "fields used :\n"
        "    angle_min, angle_max, angle_increment\n"
        "    range_min, range_max\n"
        "    ranges[]")
    pdf.h3("ICP algorithm")
    pdf.para(
        "Each scan k:")
    pdf.bullet(
        "Convert to Cartesian: (r_i * cos(a_i), r_i * sin(a_i)). Drop "
        "non-finite, out-of-range readings.")
    pdf.bullet(
        "Sub-sample by scan_subsample (default 4) to keep "
        "brute-force NN tractable.")
    pdf.bullet(
        "ICP loop, up to scan_icp_max_iter (default 15): for every src "
        "point find the nearest dst point (squared distance below 0.25 m^2 "
        "to reject outliers), then run SVD-based rigid alignment on the "
        "paired set. Stop when |delta| < scan_icp_eps.")
    pdf.bullet(
        "If the integrated XY jump exceeds scan_max_jump_m (0.5 m by "
        "default), the update is rejected as divergent.")
    pdf.bullet(
        "The accepted (dx, dy, dtheta) is composed onto the EKF node's "
        "scan_pose accumulator and that absolute pose is fed to "
        "updatePose. R is scan_xy_var on the diagonal for XY and "
        "scan_yaw_var for theta.")
    pdf.h3("Why scan-to-scan and not scan-to-map?")
    pdf.para(
        "Scan-to-scan ICP is local and never diverges on a featureless "
        "patch the way an exhaustive scan-to-map search can. The "
        "trade-off is unbounded drift over long runs - the LiDAR "
        "estimate becomes another dead-reckoning source, not a global "
        "fix. We compensate by setting scan_xy_var an order of magnitude "
        "larger than the sim's ground-truth variance so the EKF does "
        "not over-trust it. For drift-bounded localisation, swap this "
        "for AMCL or slam_toolbox externally; the same EKF can then "
        "subscribe to AMCL's PoseWithCovarianceStamped instead.")
    pdf.h3("RPLiDAR particulars")
    pdf.para(
        "RPLiDAR A1 publishes 360 returns at 5.5-10 Hz; the A2/S1 "
        "publish 400-720 returns at 10-15 Hz. The scan_subsample "
        "default of 4 yields about 90-180 points per scan after dropping "
        "bad ranges, which keeps ICP under 5 ms on a Raspberry Pi 4. "
        "Increase scan_subsample on slower MCUs.")

    # Encoders
    pdf.h2("3.3  Wheel encoders")
    pdf.h3("Message contract")
    pdf.code(
        "topic : /odom\n"
        "type  : nav_msgs/Odometry\n"
        "fields used :\n"
        "    twist.twist.linear.x          (m/s)\n"
        "    twist.twist.angular.z         (rad/s)\n"
        "POSE FIELD IS INTENTIONALLY IGNORED.")
    pdf.para(
        "Real encoders measure wheel angular velocity. After the "
        "differential-drive forward kinematics that becomes (v, omega). "
        "The nav_msgs/Odometry message also carries an integrated pose, "
        "but every controller that does the integration accumulates its "
        "own bias and slip. To avoid an unobservable double-integrator "
        "loop in the EKF, we deliberately ignore that pose and let the "
        "filter integrate v and omega itself, then correct with LiDAR / "
        "external pose.")
    pdf.h3("Typical driver setup on hardware")
    pdf.bullet(
        "ros2_control with a diff_drive_controller plugin. Configure "
        "wheel_radius and wheel_separation to match the chassis.")
    pdf.bullet(
        "A custom node reading encoder ticks over UART/CAN; do the "
        "ticks-to-rad/s conversion in firmware where possible to "
        "minimise jitter.")
    pdf.bullet(
        "Either way, publish on /odom (or remap the EKF subscription "
        "if your driver uses a different topic).")

    # ---- Section 4 - sim-only vs hardware
    pdf.h1("4. Sim-only behaviour and how to disable it")
    pdf.h2("4.1  Ground-truth pose subscription")
    pdf.para(
        "Gazebo's PosePublisher emits one geometry_msgs/PoseStamped per "
        "link with frame_id = link name. In sim we use base_link's pose "
        "as a tight EKF correction to bound drift during demos. On "
        "hardware no such topic exists, so the node must not act on it.")
    pdf.para(
        "Set use_ground_truth=false in ekf_params_hardware.yaml. The "
        "callback returns early and the subscription remains harmless "
        "even if nothing ever publishes the topic.")
    pdf.h2("4.2  Initial pose")
    pdf.para(
        "The sim's spawn is (-7, 7, 0) or (7, -7, 0) depending on the "
        "world; the hardware default is (0, 0, 0). Override per run "
        "via the initial_x / initial_y / initial_theta parameters or "
        "by sending an /initialpose message.")

    # ---- Section 5 - parameters reference
    pdf.h1("5. Parameter reference")
    pdf.table(
        headers=["Parameter", "Default (sim)", "Hardware default", "Units / meaning"],
        rows=[
            ["predict_rate_hz",      "50.0",  "50.0",  "Hz; EKF prediction timer rate."],
            ["q_x, q_y, q_theta",    "1e-4",  "1e-4",  "Process noise (per second) on pose."],
            ["q_v, q_omega",         "1e-2",  "2e-2",  "Random-walk noise on body twist."],
            ["imu_yaw_rate_var",     "1e-3",  "3e-4",  "(rad/s)^2; MPU6050 noise variance."],
            ["imu_yaw_rate_bias",    "0",     "tuned", "rad/s; subtracted from gyro_z."],
            ["odom_v_var",           "1e-3",  "2e-3",  "(m/s)^2 on encoder linear."],
            ["odom_w_var",           "1e-3",  "2e-3",  "(rad/s)^2 on encoder angular."],
            ["scan_xy_var",          "5e-2",  "1e-1",  "(m)^2 on ICP XY."],
            ["scan_yaw_var",         "5e-2",  "1e-1",  "(rad)^2 on ICP heading."],
            ["use_ground_truth",     "true",  "false", "Disable on hardware."],
            ["use_scan_matching",    "true",  "true",  "Disable if /scan is unusable."],
            ["scan_icp_max_iter",    "15",    "15",    "ICP iterations per scan."],
            ["scan_icp_eps",         "1e-4",  "1e-4",  "Convergence threshold on |delta|."],
            ["scan_subsample",       "4",     "4",     "Keep every Nth scan return."],
            ["scan_max_jump_m",      "0.5",   "0.5",   "m; outlier rejection."],
            ["initial_x / y / theta","sim spawn","0",  "World frame. Set via /initialpose if needed."],
            ["publish_tf",           "true",  "true",  "Broadcast odom -> base_link."],
        ],
        col_widths=[40, 28, 28, 78],
    )

    # ---- Section 6 - bring-up procedure
    pdf.h1("6. Hardware bring-up walkthrough")
    pdf.h3("6.1  Wire up the sensors and confirm topics")
    pdf.code(
        "# In separate terminals (or one launch file per driver):\n"
        "ros2 run mpu6050_driver mpu6050_node       # -> /imu\n"
        "ros2 launch rplidar_ros rplidar_a1_launch.py  # -> /scan\n"
        "ros2 launch my_robot diff_drive.launch.py  # -> /odom, accepts /cmd_vel\n"
        "\n"
        "# Verify the topics are alive and the message types match:\n"
        "ros2 topic hz /imu\n"
        "ros2 topic hz /scan\n"
        "ros2 topic hz /odom\n"
        "ros2 topic echo /imu  --once\n"
        "ros2 topic echo /scan --once | head -8\n"
        "ros2 topic echo /odom --once | head -20")

    pdf.h3("6.2  Calibrate the IMU bias")
    pdf.para(
        "With the robot powered up, warm for ~30 s, then leave it "
        "perfectly still on the ground. Use the snippet from section "
        "3.1 to compute the mean of gyro_z over 5 s and write the "
        "negated value into ekf_params_hardware.yaml.")

    pdf.h3("6.3  Tune encoder noise if needed")
    pdf.para(
        "Drive the robot at a constant 0.2 m/s for ~10 s. Compute "
        "stddev(twist.linear.x) over that window from /odom. Square it "
        "and slot the result into odom_v_var.")

    pdf.h3("6.4  Launch the EKF + navigation")
    pdf.code(
        "ros2 launch motion_planner hardware_demo.launch.py\n"
        "\n"
        "# Optional: pick a different planner.\n"
        "ros2 launch motion_planner hardware_demo.launch.py planner_type:=rrt_star")

    pdf.h3("6.5  Send a goal from RViz or the CLI")
    pdf.code(
        "ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\\n"
        "  \"{header: {frame_id: 'odom'}, pose: \\\n"
        "    {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}\"")

    pdf.h3("6.6  Common failure modes")
    pdf.bullet(
        "Theta drifts when stationary -> gyro bias not subtracted. "
        "Re-do step 6.2.")
    pdf.bullet(
        "EKF pose jumps every scan -> ICP is diverging on a sparse "
        "feature set. Raise scan_subsample, lower scan_max_jump_m, "
        "or set use_scan_matching=false and rely on wheel + IMU.")
    pdf.bullet(
        "Robot drives in tight circles after a goal -> Pure Pursuit "
        "tracking bug. Make sure the navigation_node binary is from "
        "the post-fix build (closest-waypoint anchored lookahead).")
    pdf.bullet(
        "TF tree breaks (rviz: 'no transform from odom to base_link') "
        "-> publish_tf is false. Either set it to true here or have "
        "some other node publish that transform.")

    # ---- Section 7 - math derivations
    pdf.h1("7. Math reference")
    pdf.h2("7.1  Why bias subtraction is a measurement-level fix")
    pdf.para(
        "If the gyro has a true bias b that we do not model, the "
        "innovation z - h(x) on every IMU update is shifted by b. "
        "Over time the filter compensates by walking omega "
        "(and through it, theta) along with the bias. Two cures: "
        "(a) subtract a calibrated constant before the update (what "
        "we do), or (b) augment the state with a bias term and let "
        "the filter estimate it online. The augmented-state approach "
        "needs an additional Q on the bias and a measurement that "
        "constrains theta absolutely - on this platform that "
        "constraint comes from the LiDAR pose update, and you do "
        "want the bias to be observable. We left it out to keep the "
        "filter at 5 states for the demo; promoting to 7 (add bias_omega "
        "and bias_v if you want a wheel-slip term) is a strict superset.")
    pdf.h2("7.2  SVD-based 2D rigid alignment")
    pdf.para(
        "Given paired point sets {s_i} and {d_i}, the optimal rigid "
        "transform (R, t) minimising sum |R s_i + t - d_i|^2 is:")
    pdf.code(
        "mu_s = mean(s_i),   mu_d = mean(d_i)\n"
        "W    = sum (d_i - mu_d)(s_i - mu_s)^T\n"
        "USV^T = svd(W)\n"
        "R     = U diag(1, det(UV^T)) V^T   # forces a proper rotation\n"
        "t     = mu_d - R mu_s\n"
        "dtheta = atan2(R(1,0), R(0,0))")
    pdf.para(
        "This is the planar specialisation of Arun's 1987 method "
        "(see Kabsch/Umeyama). The diag(1, det) factor flips one "
        "axis if the SVD returns a reflection - critical when "
        "scan-to-scan motion is near zero.")
    pdf.h2("7.3  Why ignore the encoder pose field")
    pdf.para(
        "nav_msgs/Odometry carries both twist and pose; the pose is "
        "an integration of twist with whatever bias the controller "
        "accumulates. If the EKF were to ingest both, we would feed "
        "the same noise back into the filter twice with two different "
        "weights, and the resulting estimate would be biased even "
        "in steady state. Worse, our state already integrates v and "
        "omega, so taking the encoder pose would be a third path - the "
        "ML cost of unobservable redundancy. Pure velocity in, EKF "
        "does the integration, LiDAR / external pose corrects.")

    # ---- Section 8 - testing
    pdf.h1("8. Testing the new measurement models")
    pdf.h2("8.1  Unit tests (existing)")
    pdf.para(
        "ekf_estimator already ships GTest unit tests against "
        "EkfCore (test_ekf_core.cpp). They cover predict() against "
        "an analytical reference and the three update methods against "
        "their linear H formulations. The new scan-matching code is "
        "static-method-shaped so it can be tested in isolation; the "
        "helpers exposed in the header are scanToPoints, rigidAlign2D, "
        "and icp2d.")
    pdf.h2("8.2  In-sim verification")
    pdf.code(
        "ros2 launch motion_planner nav_demo.launch.py launch_rviz:=false\n"
        "ros2 topic echo /ekf/pose --once     # sanity\n"
        "ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \\\n"
        "  \"{header: {frame_id: 'odom'}, pose: \\\n"
        "    {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}\"")
    pdf.h2("8.3  Sim -> hardware checklist")
    pdf.bullet("Set use_ground_truth=false.")
    pdf.bullet("Set imu_yaw_rate_bias from the calibration step.")
    pdf.bullet("Confirm /imu, /odom, /scan are all flowing at expected rates.")
    pdf.bullet("Confirm the diff-drive driver subscribes to /cmd_vel and the wheels respond.")
    pdf.bullet("Confirm the TF tree contains odom -> base_link (either from this EKF or from your bring-up).")
    pdf.bullet("Send a small goal (1 m forward) and confirm the robot actually moves AND the /ekf/pose advances accordingly.")

    pdf.add_page()
    pdf.h1("Appendix A - Topic and parameter summary")
    pdf.table(
        headers=["Topic", "Type", "Direction", "Used for"],
        rows=[
            ["/imu",  "sensor_msgs/Imu",          "in",  "yaw-rate update (gyro_z)"],
            ["/odom", "nav_msgs/Odometry",        "in",  "twist (v, omega) update"],
            ["/scan", "sensor_msgs/LaserScan",    "in",  "scan-to-scan ICP"],
            ["ground_truth/pose", "geometry_msgs/PoseStamped", "in", "sim only; gated by use_ground_truth"],
            ["/ekf/pose", "geometry_msgs/PoseWithCovarianceStamped", "out", "fused estimate"],
            ["tf (odom -> base_link)", "tf2_msgs/TFMessage", "out", "broadcast if publish_tf=true"],
        ],
        col_widths=[55, 50, 18, 51],
    )

    out = Path("/home/harsh/Desktop/robotics_ws/EKF_HARDWARE_PORTING.pdf")
    pdf.output(str(out))
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
