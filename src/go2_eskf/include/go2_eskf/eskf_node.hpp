// eskf_node.hpp
// ROS 2 wrapper around EskfCore. Drives prediction from the IMU and applies
// leg-odometry (body velocity) and GPS (world position) corrections, then
// publishes eskf/odom and (optionally) a world->base TF. Optionally logs the
// estimate alongside a ground-truth topic for the Phase 4 benchmark.

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "go2_eskf/eskf_core.hpp"
#include "go2_eskf/slip_model.hpp"

namespace go2_eskf {

class EskfNode : public rclcpp::Node {
 public:
  EskfNode();

 private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void legOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);

  // Tilt-compensated compass: body-frame field -> ENU heading in the state's own
  // convention (psi = 0 is body-x-East). Returns false when the reading is
  // rejected (implausible magnitude, or a horizontal component too small for
  // atan2 to mean anything), leaving yaw_out untouched.
  bool magYawFromField(const Eigen::Vector3d& m_body, double& yaw_out) const;

  // Roll/pitch for tilt compensation, taken from the gravity low-pass rather
  // than roll_/pitch_ — under attitude_source "gravity_lp" those are forced to
  // zero, which would silently disable tilt compensation on sloped terrain.
  void bodyTilt(double& roll, double& pitch) const;

  // Phase 3: run the slip model on the latest features and return the
  // (possibly inflated) leg-odometry covariance for this correction.
  Eigen::Matrix2d legCovarianceForUpdate(double leg_vx, double leg_vy);

  // Distinguishes a genuine stop from champ's mid-gait "no information" leg-odom
  // sample. Both look like (0,0,0), but they differ in DURATION: a flight phase
  // lasts a fraction of a gait cycle, whereas standing still (all four feet
  // planted) produces an unbroken run of zeros. `stamp` must come from the
  // leg-odom header so the test uses one consistent clock.
  bool looksGenuinelyStationary(const rclcpp::Time& stamp) const;

  void publishEstimate(const rclcpp::Time& stamp);
  void logRow(const rclcpp::Time& stamp);

  // Local-tangent-plane (ENU) conversion of a GPS fix relative to the datum.
  void gpsToEnu(double lat, double lon, double& east, double& north) const;

  std::unique_ptr<EskfCore> eskf_;

  // Latest IMU attitude (roll/pitch are inputs to the filter, yaw is a state).
  double roll_ = 0.0, pitch_ = 0.0;
  bool initialized_ = false;
  rclcpp::Time last_imu_time_;

  // Gravity low-pass (for attitude_source == "gravity_lp"): tracks the slowly
  // varying gravity+mount component of the accelerometer so we can subtract it
  // and keep only motion acceleration — robust to the sim IMU's frame flip and
  // contact-impact spikes.
  Eigen::Vector3d grav_lp_ = Eigen::Vector3d::Zero();
  bool have_grav_lp_ = false;
  double grav_lp_beta_ = 0.995;
  double accel_clip_ = 40.0;
  double gyro_z_sign_ = 1.0;  // -1 un-flips the sim IMU's negated yaw rate

  // GPS datum (first valid fix defines the ENU origin).
  bool gps_datum_set_ = false;
  double lat0_ = 0.0, lon0_ = 0.0;

  // Latest ground truth (for logging only).
  bool have_gt_ = false;
  double gt_x_ = 0.0, gt_y_ = 0.0, gt_yaw_ = 0.0;

  // Parameters.
  std::string world_frame_, base_frame_;
  // "gravity_lp" (low-pass gravity removal) | "accel" (per-sample leveling) |
  // "orientation" (trust the IMU quaternion).
  std::string attitude_source_ = "gravity_lp";
  bool publish_tf_ = false;
  bool use_gps_ = true;
  double max_imu_dt_ = 0.05;
  Eigen::Matrix2d R_leg_;
  Eigen::Matrix2d R_gps_;
  Eigen::Matrix2d R_zupt_;      // tight R for a genuine zero-velocity update
  double vz_zero_noise_ = 0.3;  // std-dev of the vz≈0 vertical pseudo-measurement
  double leg_odom_scale_ = 1.0; // undoes CHAMP's odom_scaler velocity fudge
  bool use_leg_yaw_bias_ = true;   // observe gyro bias via (gyro_wz - leg wz)
  double r_leg_yaw_bias_ = 2.5e-3; // variance of that bias pseudo-measurement
  // The bias pseudo-measurement is only valid when NOT rotating: wz_leg has a
  // rotation-dependent scale error, so a turning residual is not bias. Measured
  // ~-0.005 rad/s straight vs +0.045..+0.052 during a wz=0.4 turn. Above this
  // |wz| the bias update is skipped. Set very large to disable the gate.
  double bias_update_max_wz_ = 0.10;  // [rad/s]
  uint64_t n_bias_skipped_ = 0;

  // --- Absolute heading from a magnetometer -------------------------------
  // The ONLY sensor here that makes yaw observable. Everything else constrains
  // body-frame velocity, which is blind to the null direction
  // dv = dpsi*(-v_y, v_x); GPS position breaks that too but only while moving,
  // whereas a compass works standing still. Off by default — it needs a
  // declination that is correct for the deployment site, and a sim magnetometer
  // is far kinder than a real one near motors and steel.
  bool use_magnetometer_ = false;
  double r_mag_yaw_ = 0.0225;        // [rad^2] variance of the heading measurement
  double mag_declination_rad_ = 0.0; // magnetic north east of true north
  double mag_yaw_offset_rad_ = 0.0;  // sensor mounting yaw, added after the sign
  double mag_yaw_sign_ = 1.0;        // -1 mirrors a flipped-mount compass
  bool mag_tilt_compensate_ = true;
  // Plausibility gate on |B|. Hard/soft-iron disturbances and nearby motors show
  // up first as a magnitude that no longer matches the ambient field, which is
  // the cheapest way to spot a reading that must not be fused. <= 0 disables it.
  double mag_norm_ref_ = 0.0;        // [T] expected ambient field magnitude
  double mag_norm_tol_ = 0.25;       // accept |‖B‖-ref| <= tol*ref
  // Below this the levelled horizontal component is numerical noise and atan2
  // returns an essentially random heading (the degenerate case: field straight
  // down, i.e. standing at a magnetic pole, or a dead sensor reading zeros).
  double mag_min_horiz_ = 1.0e-9;    // [T]
  // Absolute-heading updates do not need to run at sensor rate: fusing a 100 Hz
  // compass with an optimistic R crushes P(psi) and lets a biased heading
  // dominate the gyro entirely. Throttle so R stays the honest trust knob.
  double mag_min_interval_ = 0.1;    // [s]
  bool mag_seed_initial_yaw_ = true; // start psi at the compass, not at 0
  bool have_last_mag_update_ = false;
  rclcpp::Time last_mag_update_time_;
  uint64_t n_mag_msgs_ = 0, n_mag_rejected_ = 0, n_mag_applied_ = 0;

  // champ::Odometry::getVelocities early-returns hard zeros for vx, vy AND wz
  // whenever all four or zero feet are in contact ("nothing to calculate"), and
  // gait.yaml's stance_duration 0.25 makes that a large fraction of a trot. Those
  // samples are a NO-INFORMATION flag, not a measurement: fusing them drags
  // velocity toward zero (which no leg_odom_scale_ can undo, since it multiplies
  // zero) and tells correctGyroBias that the entire gyro yaw rate is bias — worst
  // exactly during turns. Gate them, but keep them when the robot is genuinely
  // commanded to stand still, where (0,0,0) is real information (a ZUPT).
  bool leg_odom_gate_degenerate_ = true;
  // Standing still and a flight phase both report (0,0,0); they differ in how
  // LONG the zeros persist. A trot's no-contact window is a fraction of a gait
  // cycle, so an unbroken degenerate run past this threshold means the robot is
  // genuinely stopped (all four feet planted) and the zeros are a real ZUPT.
  // Deliberately not based on /cmd_vel freshness: teleop_twist_keyboard publishes
  // only on keypress, so a stale command does NOT imply a stationary robot.
  double degenerate_hold_sec_ = 0.3;
  bool in_degenerate_run_ = false;
  rclcpp::Time degenerate_run_start_;
  // /cmd_vel is used only as a veto: a fresh command asking for motion rules out
  // a ZUPT. It can never on its own promote zeros to a measurement.
  double stationary_cmd_eps_ = 0.01;   // |cmd| below this => not commanding motion
  double cmd_vel_stale_sec_ = 0.5;     // older than this => no opinion
  bool have_cmd_vel_ = false;
  rclcpp::Time last_cmd_vel_time_;
  // Running tally so a run's degenerate fraction is visible in the node's log.
  uint64_t n_leg_msgs_ = 0, n_leg_degenerate_ = 0, n_leg_skipped_ = 0;

  // Phase 3: slip-adaptive leg covariance. When use_slip_model_ is true, the
  // model maps the latest locomotion features to a slip score that inflates
  // R_leg_ via SlipModel::adaptLegCovariance (lambda = slip_lambda_).
  SlipModel slip_model_;
  bool use_slip_model_ = false;
  double slip_lambda_ = 1.0;
  double last_slip_ = 0.0;

  // Latest inputs to the slip feature vector, refreshed by their callbacks.
  double cmd_vx_ = 0.0, cmd_vy_ = 0.0, cmd_wz_ = 0.0;  // commanded body twist
  double gyro_wz_ = 0.0;                                // measured yaw rate
  double joint_vel_mean_ = 0.0, joint_vel_max_ = 0.0;   // joint velocity stats
  double accel_horiz_ = 0.0;                            // horizontal motion accel
  // NOT WIRED UP: nothing assigns this, so the slip model's contact_frac feature
  // is a frozen 1.0. Populating it needs a /foot_contacts
  // (champ_msgs/ContactsStamped) subscription, i.e. a first-party -> vendored
  // dependency, for a model that is off by default. Do not read it as live data.
  double contact_frac_ = 1.0;                           // feet-in-contact fraction

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_pub_;
  // b_g is the state that corrupts heading (predict integrates gyro_z - b_g), so
  // publish it: a poisoned bias is invisible in a position-error plot.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gyro_bias_pub_;
  // The raw compass heading, published BEFORE it is fused. Calibrating
  // mag_yaw_sign/mag_yaw_offset means comparing this against ground-truth yaw;
  // without it that is guesswork, exactly as it was for b_g.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mag_yaw_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::ofstream log_file_;
};

}  // namespace go2_eskf
