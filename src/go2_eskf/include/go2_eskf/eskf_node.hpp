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

  // Phase 3: run the slip model on the latest features and return the
  // (possibly inflated) leg-odometry covariance for this correction.
  Eigen::Matrix2d legCovarianceForUpdate(double leg_vx, double leg_vy);

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
  double vz_zero_noise_ = 0.3;  // std-dev of the vz≈0 vertical pseudo-measurement
  double leg_odom_scale_ = 1.0; // undoes CHAMP's odom_scaler velocity fudge
  bool use_leg_yaw_bias_ = true;   // observe gyro bias via (gyro_wz - leg wz)
  double r_leg_yaw_bias_ = 2.5e-3; // variance of that bias pseudo-measurement

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
  double contact_frac_ = 1.0;                           // feet-in-contact fraction

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr slip_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::ofstream log_file_;
};

}  // namespace go2_eskf
