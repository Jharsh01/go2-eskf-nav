// eskf_node.hpp
// ROS 2 wrapper around EskfCore. Drives prediction from the IMU and applies
// leg-odometry (body velocity), GPS (world position) and, optionally,
// magnetometer (heading) corrections, then
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

// champ_msgs lives in the vendored CHAMP stack. It carries the only live
// foot-contact signal (/foot_contacts), which is a slip feature, but a
// first-party package must still build without the vendored tree present — so
// the dependency is optional and CMake defines this only when it is found.
#ifdef GO2_ESKF_HAS_CHAMP_MSGS
#include <champ_msgs/msg/contacts_stamped.hpp>
#endif

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
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
  void groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
#ifdef GO2_ESKF_HAS_CHAMP_MSGS
  void contactsCallback(const champ_msgs::msg::ContactsStamped::SharedPtr msg);
#endif

  // Phase 3: run the slip model on the latest features and return the
  // (possibly inflated) leg-odometry covariance for this correction.
  Eigen::Matrix2d legCovarianceForUpdate(double leg_vx, double leg_vy);

  // The feature vector the slip model consumes, built from the latest sensor
  // callbacks. Split out so training data is logged from EXACTLY the same code
  // path that inference uses — a training set built any other way silently
  // drifts from what the live model sees.
  SlipFeatures currentSlipFeatures(double leg_vx, double leg_vy) const;

  // One CSV row per fused leg-odom update: the features, plus the ground-truth
  // body twist when a truth topic is connected. train_slip_model.py turns the
  // (leg odom - truth) velocity error into the training label, so this is the
  // whole supervision signal for the slip model.
  void logSlipFeatures(const rclcpp::Time& stamp, const SlipFeatures& feat);

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

  // GPS datum: the ENU origin, averaged over the first gps_datum_samples_ fixes.
  // A SINGLE fix used to define it, which pinned the whole world frame to that
  // one sample's noise — a constant offset on every position the filter ever
  // reports. Averaging N fixes shrinks it by sqrt(N) and costs only N/10 s of
  // startup at the sensor's 10 Hz.
  bool gps_datum_set_ = false;
  int gps_datum_samples_ = 10;
  int gps_datum_count_ = 0;
  double lat_sum_ = 0.0, lon_sum_ = 0.0;
  double lat0_ = 0.0, lon0_ = 0.0;

  // Magnetometer heading (use_mag). The one INDEPENDENT absolute heading source:
  // GPS reaches psi only through velocity (so not while turning in place) and
  // leg odometry not at all. Tilt compensation uses grav_lp_ as "up" — the same
  // sensor frame as the magnetometer when both sit on imu_link — so no roll/
  // pitch angles (unreliable in this sim) are involved.
  bool use_mag_ = false;
  double r_mag_ = 2.5e-3;           // variance of the heading measurement [rad^2]
  double mag_min_interval_ = 0.1;   // [s] fuse at most this often (noise is
                                    // time-correlated through the tilt estimate)
  double mag_norm_gate_ = 0.25;     // reject |B| off the reference by > this fraction
  // Reference = world angle of the field's horizontal part. With
  // mag_calibrate_heading_ it is calibrated from the first mag_ref_samples_
  // readings against the filter's own initial heading (sim: the robot spawns at
  // yaw 0 in ENU, and the gz field direction is then irrelevant); otherwise
  // mag_field_heading_ is used as given — e.g. from declination on hardware.
  // The field-norm reference for the disturbance gate is averaged over the same
  // samples either way.
  int mag_ref_samples_ = 50;
  bool mag_calibrate_heading_ = true;
  double mag_field_heading_ = 0.0699;
  bool mag_ref_set_ = false;
  int mag_ref_count_ = 0;
  double mag_ref_c_ = 0.0, mag_ref_s_ = 0.0, mag_norm_sum_ = 0.0;
  double mag_ref_norm_ = 0.0;
  rclcpp::Time last_mag_fuse_time_;
  bool have_mag_fused_ = false;
  uint64_t n_mag_msgs_ = 0, n_mag_fused_ = 0, n_mag_norm_rejected_ = 0;

  // Tilt-dependent R (2026-09-24). The heading error comes from "up" (grav_lp_)
  // lagging the gait's roll/pitch rocking. That lag is exactly the HIGH-pass of the
  // body tilt with grav_lp_'s own time constant, which the gyro measures directly:
  // tilt_hp_ = beta * (tilt_hp_ + w_xy * dt) mirrors grav_lp_'s discrete low-pass.
  // Only |tilt_hp_| is used, so a flipped gyro x/y axis would not matter.
  // MEASURED on the 23:47 run (5 Hz, truth tilt): mag error rms 2.7 / 4.1 / 6.2 /
  // 8.8 deg for tilt lag 0-3 / 3-6 / 6-10 / 10+ deg, fit
  // sigma^2 = (2.96 deg)^2 + (0.60 * lag)^2 — hence mag_tilt_gain 0.60.
  Eigen::Vector2d tilt_hp_ = Eigen::Vector2d::Zero();
  double mag_tilt_gain_ = 0.60;
  // Innovation gate: skip a heading whose wrapped innovation exceeds
  // mag_gate_sigma_ * sqrt(P_psi + R). A gate alone can lock out a filter whose
  // heading really has gone wrong, so after mag_gate_reset_sec_ of unbroken
  // rejections the next sample is fused regardless.
  double mag_gate_sigma_ = 3.0;       // 0 disables
  double mag_gate_reset_sec_ = 5.0;
  bool mag_rejecting_ = false;
  rclcpp::Time mag_reject_start_;
  uint64_t n_mag_gate_rejected_ = 0, n_mag_gate_forced_ = 0;

  // Latest ground truth (for logging only). The twist is what labels the slip
  // training set: the gz OdometryPublisher reports it in the CHILD (body) frame,
  // the same frame as CHAMP's leg odometry, so the two subtract directly.
  bool have_gt_ = false;
  double gt_x_ = 0.0, gt_y_ = 0.0, gt_yaw_ = 0.0;
  double gt_vx_ = 0.0, gt_vy_ = 0.0, gt_wz_ = 0.0;
  // Angle between the base z axis and world up [rad]. Logged with the slip
  // training rows so auto_train_slip.py can drop a fallen robot's rows: its
  // feet scrabble, leg odometry is garbage, and that is not the distribution the
  // model runs on.
  double gt_tilt_ = 0.0;

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
  // Fraction of feet in stance, from /foot_contacts (champ_msgs/ContactsStamped).
  // Only live when the vendored champ_msgs was found at build time AND the topic
  // is publishing; otherwise it stays at the neutral 1.0 and have_contacts_ is
  // false, which the feature logger records so a training set can never mistake
  // the placeholder for data.
  //
  // MEASURED 2026-08-11, once it was wired up: at the instants the slip model
  // actually runs it is 0.5 in 13467/13467 samples, i.e. it carries ZERO
  // information. That is structural, not a bug — the degenerate gate drops every
  // sample with 0 or 4 feet down (champ returns hard zeros there), and a trot in
  // between always stands on one diagonal pair. Wiring it up was still worth it:
  // the feature is now known-dead rather than assumed-useful. A useful contact
  // feature would have to be sub-gait-cycle (e.g. stance duty over a window).
  double contact_frac_ = 1.0;                           // feet-in-contact fraction
  bool have_contacts_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
#ifdef GO2_ESKF_HAS_CHAMP_MSGS
  rclcpp::Subscription<champ_msgs::msg::ContactsStamped>::SharedPtr contacts_sub_;
#endif
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_pub_;
  // b_g is the state that corrupts heading (predict integrates gyro_z - b_g), so
  // publish it: a poisoned bias is invisible in a position-error plot.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gyro_bias_pub_;
  // The raw tilt-compensated magnetometer heading, before fusion — compare it
  // to ground-truth yaw to validate the sensor/frame chain independently of
  // the filter.
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mag_heading_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::ofstream log_file_;
  std::ofstream slip_log_file_;
};

}  // namespace go2_eskf
