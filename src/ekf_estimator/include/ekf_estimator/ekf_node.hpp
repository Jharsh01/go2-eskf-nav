// ekf_node.hpp
#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "ekf_estimator/ekf_core.hpp"

namespace ekf_estimator {

class EkfNode : public rclcpp::Node {
 public:
  EkfNode();

 private:
  // Callbacks
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void groundTruthCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void predictTimerCallback();

  // Helpers
  void publishPose(const rclcpp::Time& stamp);
  void broadcastTf(const rclcpp::Time& stamp);
  double dtSinceLastPredict(const rclcpp::Time& now);

  // Scan-matching helpers (header-public for unit testing).
  // Converts a LaserScan into 2D points in the lidar frame, dropping bad ranges.
  static std::vector<Eigen::Vector2d> scanToPoints(
      const sensor_msgs::msg::LaserScan& scan);
  // SVD-based 2D rigid alignment of two equal-length point sets (one-to-one).
  // Returns (dx, dy, dtheta) that maps src -> dst.
  static Eigen::Vector3d rigidAlign2D(
      const std::vector<Eigen::Vector2d>& src,
      const std::vector<Eigen::Vector2d>& dst);
  // Full scan-to-scan ICP loop: nearest-neighbor + rigid alignment, iterated.
  // Returns (dx, dy, dtheta) that aligns src onto dst (in dst's frame).
  static Eigen::Vector3d icp2d(
      const std::vector<Eigen::Vector2d>& src,
      const std::vector<Eigen::Vector2d>& dst,
      int max_iter, double convergence_eps);

  // EKF core
  std::unique_ptr<EkfCore> ekf_;

  // ROS 2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr      scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr  gt_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr predict_timer_;

  // Params (loaded from YAML)
  std::string base_frame_;
  std::string odom_frame_;
  double      imu_yaw_rate_var_;
  double      imu_yaw_rate_bias_;    // static gyro_z bias (rad/s) — subtracted before use
  double      odom_v_var_;
  double      odom_w_var_;
  double      scan_xy_var_;
  double      scan_yaw_var_;
  double      gt_xy_var_;
  double      gt_yaw_var_;
  bool        publish_tf_;
  bool        use_ground_truth_;     // off on real hardware
  bool        use_scan_matching_;    // disable if /scan is noisy or absent
  int         scan_icp_max_iter_;
  double      scan_icp_eps_;
  int         scan_subsample_;       // keep every Nth scan return
  double      scan_max_jump_m_;      // reject ICP delta if it exceeds this

  // Scan-matching state. We accumulate scan-to-scan deltas into an absolute
  // pose (scan_pose_) and feed that to EkfCore::updatePose. This drifts (it's
  // dead-reckoning on top of LiDAR), but the EKF fuses it with wheel + IMU so
  // short-term wheel slip is mostly absorbed.
  std::vector<Eigen::Vector2d> last_scan_points_;
  Eigen::Vector3d              scan_pose_{Eigen::Vector3d::Zero()};
  bool                         scan_pose_initialised_{false};

  std::optional<rclcpp::Time> last_predict_time_;
};

}  // namespace ekf_estimator
