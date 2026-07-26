// ekf_node.cpp
//
// ROS 2 node wrapping the EKF core. Subscribes to:
//   /imu  (sensor_msgs/Imu)             — MPU6050 / any IMU. Uses gyro_z only.
//   /odom (nav_msgs/Odometry)           — wheel encoder odometry. Uses twist only
//                                         (linear.x, angular.z); pose is ignored.
//   /scan (sensor_msgs/LaserScan)       — RPLiDAR or equivalent. Scan-to-scan
//                                         ICP produces a relative motion that
//                                         is integrated into an absolute pose
//                                         measurement.
//   ground_truth/pose                   — sim-only; disabled by use_ground_truth.
//
// All inbound message types and field choices are deliberately the ones a
// real-hardware driver (rplidar_ros, any mpu6050 ROS 2 driver, ros2_control
// diff_drive_controller) already publishes, so the same node runs on
// hardware after flipping use_ground_truth=false.
#include "ekf_estimator/ekf_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ekf_estimator {

namespace {
// Extract yaw from a geometry_msgs Quaternion.
double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tq(q.x, q.y, q.z, q.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
  return yaw;
}

double wrapAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}
}  // namespace

EkfNode::EkfNode() : rclcpp::Node("ekf_estimator") {
  // ---- Declare and load parameters
  base_frame_         = declare_parameter<std::string>("base_frame", "base_link");
  odom_frame_         = declare_parameter<std::string>("odom_frame", "odom");
  imu_yaw_rate_var_   = declare_parameter<double>("imu_yaw_rate_var", 1e-3);
  imu_yaw_rate_bias_  = declare_parameter<double>("imu_yaw_rate_bias", 0.0);
  odom_v_var_         = declare_parameter<double>("odom_v_var", 1e-3);
  odom_w_var_         = declare_parameter<double>("odom_w_var", 1e-3);
  scan_xy_var_        = declare_parameter<double>("scan_xy_var", 1e-2);
  scan_yaw_var_       = declare_parameter<double>("scan_yaw_var", 1e-2);
  gt_xy_var_          = declare_parameter<double>("gt_xy_var", 1e-3);
  gt_yaw_var_         = declare_parameter<double>("gt_yaw_var", 1e-3);
  publish_tf_         = declare_parameter<bool>("publish_tf", true);
  use_ground_truth_   = declare_parameter<bool>("use_ground_truth", true);
  use_scan_matching_  = declare_parameter<bool>("use_scan_matching", true);
  scan_icp_max_iter_  = declare_parameter<int>("scan_icp_max_iter", 15);
  scan_icp_eps_       = declare_parameter<double>("scan_icp_eps", 1e-4);
  scan_subsample_     = declare_parameter<int>("scan_subsample", 4);
  scan_max_jump_m_    = declare_parameter<double>("scan_max_jump_m", 0.5);
  const double predict_rate = declare_parameter<double>("predict_rate_hz", 50.0);

  // Process noise from params (diagonal).
  EkfCore::Config cfg;
  cfg.process_noise_Q.diagonal() <<
      declare_parameter<double>("q_x",     1e-4),
      declare_parameter<double>("q_y",     1e-4),
      declare_parameter<double>("q_theta", 1e-4),
      declare_parameter<double>("q_v",     1e-2),
      declare_parameter<double>("q_omega", 1e-2);
  // Initial belief — set to robot's known spawn pose to avoid a startup jump
  // when the first ground-truth / scan-match update arrives. On hardware,
  // set these to (0,0,0) or use /initialpose.
  cfg.initial_state(0) = declare_parameter<double>("initial_x",     -7.0);
  cfg.initial_state(1) = declare_parameter<double>("initial_y",     7.0);
  cfg.initial_state(2) = declare_parameter<double>("initial_theta", 0.0);
  ekf_ = std::make_unique<EkfCore>(cfg);

  // Initial pose for the scan-matching accumulator — match EKF init so the
  // first scan-pose update doesn't jolt the filter.
  scan_pose_ << cfg.initial_state(0), cfg.initial_state(1), cfg.initial_state(2);

  // ---- ROS 2 wiring
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::SensorDataQoS(),
      std::bind(&EkfNode::imuCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&EkfNode::odomCallback, this, std::placeholders::_1));
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&EkfNode::scanCallback, this, std::placeholders::_1));
  gt_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "ground_truth/pose", 10,
      std::bind(&EkfNode::groundTruthCallback, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ekf/pose", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  predict_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / predict_rate),
      std::bind(&EkfNode::predictTimerCallback, this));

  RCLCPP_INFO(get_logger(),
              "EKF estimator started @ %.1f Hz | use_ground_truth=%s | "
              "use_scan_matching=%s | gyro_bias=%.4f rad/s",
              predict_rate,
              use_ground_truth_ ? "true" : "false",
              use_scan_matching_ ? "true" : "false",
              imu_yaw_rate_bias_);
}

double EkfNode::dtSinceLastPredict(const rclcpp::Time& now) {
  if (!last_predict_time_) {
    last_predict_time_ = now;
    return 0.0;
  }
  const double dt = (now - *last_predict_time_).seconds();
  last_predict_time_ = now;
  return dt;
}

void EkfNode::predictTimerCallback() {
  const auto now = this->now();
  const double dt = dtSinceLastPredict(now);
  if (dt <= 0.0) return;            // first tick, nothing to do
  ekf_->predict(dt);
  publishPose(now);
  if (publish_tf_) broadcastTf(now);
}

// ----------------------------------------------------------------------------
// IMU update (MPU6050-compatible).
//
// The MPU6050 outputs 3-axis gyro and 3-axis accel; standard ROS 2 drivers
// publish that as sensor_msgs/Imu with:
//   angular_velocity.{x,y,z}     [rad/s]
//   linear_acceleration.{x,y,z}  [m/s^2]
// For a planar diff-drive robot, only gyro_z (yaw rate) is informative for
// the EKF's omega state. We subtract a static bias (calibrated offline by
// averaging gyro_z while the robot is stationary).
void EkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const double omega_meas = msg->angular_velocity.z - imu_yaw_rate_bias_;
  ekf_->updateImuYawRate(omega_meas, imu_yaw_rate_var_);
}

// ----------------------------------------------------------------------------
// Wheel-encoder update.
//
// Real encoders measure wheel angular velocity. A typical diff-drive driver
// (ros2_control's diff_drive_controller or any custom encoder node) publishes
// nav_msgs/Odometry whose twist.twist.{linear.x, angular.z} is the
// instantaneous body velocity. The pose field is an integrated estimate that
// we deliberately IGNORE — pose comes out of the EKF, not the encoders, to
// keep dead-reckoning loops single-source.
void EkfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Vector2d z;
  z << msg->twist.twist.linear.x, msg->twist.twist.angular.z;
  Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
  R(0, 0) = odom_v_var_;
  R(1, 1) = odom_w_var_;
  ekf_->updateWheelOdom(z, R);
}

// ----------------------------------------------------------------------------
// Ground-truth update (sim only).
//
// Gazebo's gz-sim-pose-publisher-system emits one Pose per link, with
// frame_id = link name. We filter for base_link and use it to bound EKF
// drift in simulation. Disabled on hardware via use_ground_truth=false.
void EkfNode::groundTruthCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!use_ground_truth_) return;

  RCLCPP_INFO_ONCE(get_logger(),
                   "Ground-truth pose received with frame_id='%s' at (%.2f, %.2f)",
                   msg->header.frame_id.c_str(),
                   msg->pose.position.x, msg->pose.position.y);

  if (msg->header.frame_id.find("base_link") == std::string::npos) return;

  Eigen::Vector3d z;
  z << msg->pose.position.x,
       msg->pose.position.y,
       yawFromQuat(msg->pose.orientation);
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = gt_xy_var_;
  R(1, 1) = gt_xy_var_;
  R(2, 2) = gt_yaw_var_;
  ekf_->updatePose(z, R);
}

// ----------------------------------------------------------------------------
// Scan-matching update (RPLiDAR / Gazebo gpu_lidar — same message type).
//
// Algorithm: 2D point-to-point ICP between consecutive scans.
//   1. Convert each scan to (x, y) points in the lidar frame, dropping bad
//      ranges (NaN, out of [range_min, range_max]).
//   2. Sub-sample by scan_subsample to keep the per-update cost bounded.
//   3. ICP iterates: for each src point find the nearest dst point
//      (brute-force; OK for ~360 points), then solve the SVD-based rigid
//      alignment, apply it, repeat until the delta is < scan_icp_eps.
//   4. The accumulated delta is the relative motion between scan k-1 and k.
//      Compose it onto an absolute "scan-matched pose" and feed that to the
//      EKF as a pose measurement.
//   5. Reject the update if the per-step jump exceeds scan_max_jump_m
//      (catches divergent ICP solutions on featureless scans).
void EkfNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  if (!use_scan_matching_) return;

  std::vector<Eigen::Vector2d> pts = scanToPoints(*msg);
  // Subsample for performance — at 360 returns/scan and 10 Hz, brute-force
  // NN at every iteration is OK but we don't need every point.
  if (scan_subsample_ > 1) {
    std::vector<Eigen::Vector2d> sub;
    sub.reserve(pts.size() / scan_subsample_ + 1);
    for (size_t i = 0; i < pts.size(); i += scan_subsample_) {
      sub.push_back(pts[i]);
    }
    pts.swap(sub);
  }
  if (pts.size() < 10) return;  // not enough returns to align

  if (!scan_pose_initialised_) {
    last_scan_points_ = std::move(pts);
    scan_pose_initialised_ = true;
    return;
  }

  // ICP between previous and current scan.
  Eigen::Vector3d d =
      icp2d(last_scan_points_, pts, scan_icp_max_iter_, scan_icp_eps_);
  const double jump = std::hypot(d(0), d(1));
  if (jump > scan_max_jump_m_) {
    RCLCPP_DEBUG(get_logger(), "Rejecting ICP delta of %.2f m as outlier", jump);
    last_scan_points_ = std::move(pts);
    return;
  }

  // Compose: scan_pose_new = scan_pose_old * delta (delta in local frame).
  const double c = std::cos(scan_pose_(2));
  const double s = std::sin(scan_pose_(2));
  scan_pose_(0) += c * d(0) - s * d(1);
  scan_pose_(1) += s * d(0) + c * d(1);
  scan_pose_(2) = wrapAngle(scan_pose_(2) + d(2));

  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  R(0, 0) = scan_xy_var_;
  R(1, 1) = scan_xy_var_;
  R(2, 2) = scan_yaw_var_;
  ekf_->updatePose(scan_pose_, R);

  last_scan_points_ = std::move(pts);
}

// ----------------------------------------------------------------------------
// Scan-matching helpers
// ----------------------------------------------------------------------------
std::vector<Eigen::Vector2d> EkfNode::scanToPoints(
    const sensor_msgs::msg::LaserScan& scan) {
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(scan.ranges.size());
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double a = scan.angle_min + i * scan.angle_increment;
    pts.emplace_back(r * std::cos(a), r * std::sin(a));
  }
  return pts;
}

Eigen::Vector3d EkfNode::rigidAlign2D(
    const std::vector<Eigen::Vector2d>& src,
    const std::vector<Eigen::Vector2d>& dst) {
  // Both vectors are expected to be the same length, with src[i] paired to
  // dst[i] (i.e. caller already did correspondences). 2D point-to-point
  // rigid alignment via SVD (Arun's method, planar).
  const size_t N = std::min(src.size(), dst.size());
  if (N == 0) return Eigen::Vector3d::Zero();

  Eigen::Vector2d mu_s = Eigen::Vector2d::Zero();
  Eigen::Vector2d mu_d = Eigen::Vector2d::Zero();
  for (size_t i = 0; i < N; ++i) { mu_s += src[i]; mu_d += dst[i]; }
  mu_s /= static_cast<double>(N);
  mu_d /= static_cast<double>(N);

  Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < N; ++i) {
    W += (dst[i] - mu_d) * (src[i] - mu_s).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d U = svd.matrixU();
  Eigen::Matrix2d V = svd.matrixV();
  // Force a proper rotation (det = +1).
  Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
  if ((U * V.transpose()).determinant() < 0.0) D(1, 1) = -1.0;
  Eigen::Matrix2d Rm = U * D * V.transpose();
  Eigen::Vector2d t  = mu_d - Rm * mu_s;
  const double dtheta = std::atan2(Rm(1, 0), Rm(0, 0));
  return Eigen::Vector3d(t.x(), t.y(), dtheta);
}

Eigen::Vector3d EkfNode::icp2d(
    const std::vector<Eigen::Vector2d>& src_in,
    const std::vector<Eigen::Vector2d>& dst,
    int max_iter, double convergence_eps) {
  if (src_in.empty() || dst.empty()) return Eigen::Vector3d::Zero();

  // Work in src's frame; we accumulate (R, t) that takes src -> dst.
  std::vector<Eigen::Vector2d> src = src_in;
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();

  for (int it = 0; it < max_iter; ++it) {
    // Brute-force nearest-neighbor matching, src -> dst.
    std::vector<Eigen::Vector2d> matched_src;
    std::vector<Eigen::Vector2d> matched_dst;
    matched_src.reserve(src.size());
    matched_dst.reserve(src.size());
    for (const auto& p : src) {
      double best = std::numeric_limits<double>::infinity();
      size_t best_j = 0;
      for (size_t j = 0; j < dst.size(); ++j) {
        const double d2 = (dst[j] - p).squaredNorm();
        if (d2 < best) { best = d2; best_j = j; }
      }
      // Drop very-far matches (gross outliers) — keep correspondences within 0.5 m.
      if (best < 0.25) {
        matched_src.push_back(p);
        matched_dst.push_back(dst[best_j]);
      }
    }
    if (matched_src.size() < 6) break;

    Eigen::Vector3d step = rigidAlign2D(matched_src, matched_dst);
    // Apply step.
    const double c = std::cos(step(2));
    const double s = std::sin(step(2));
    Eigen::Matrix2d dR; dR << c, -s, s, c;
    Eigen::Vector2d dt; dt << step(0), step(1);
    for (auto& p : src) p = dR * p + dt;
    R = dR * R;
    t = dR * t + dt;

    if (std::hypot(step(0), step(1)) + std::abs(step(2)) < convergence_eps) break;
  }
  return Eigen::Vector3d(t.x(), t.y(), std::atan2(R(1, 0), R(0, 0)));
}

// ----------------------------------------------------------------------------
// Publishers
// ----------------------------------------------------------------------------
void EkfNode::publishPose(const rclcpp::Time& stamp) {
  const auto& x = ekf_->state();
  const auto& P = ekf_->covariance();

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp    = stamp;
  msg.header.frame_id = odom_frame_;
  msg.pose.pose.position.x = x(0);
  msg.pose.pose.position.y = x(1);
  msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, x(2));
  msg.pose.pose.orientation = tf2::toMsg(q);

  // Map 5-state covariance into the 6-DoF (x,y,z,roll,pitch,yaw) layout.
  // Indices: 0=x, 1=y, 5=yaw. Everything else is 0 (planar robot).
  auto& C = msg.pose.covariance;
  std::fill(C.begin(), C.end(), 0.0);
  C[0]      = P(0, 0);  C[1]      = P(0, 1);  C[5]      = P(0, 2);
  C[6]      = P(1, 0);  C[7]      = P(1, 1);  C[11]     = P(1, 2);
  C[30]     = P(2, 0);  C[31]     = P(2, 1);  C[35]     = P(2, 2);

  pose_pub_->publish(msg);
}

void EkfNode::broadcastTf(const rclcpp::Time& stamp) {
  const auto& x = ekf_->state();
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp    = stamp;
  t.header.frame_id = odom_frame_;
  t.child_frame_id  = base_frame_;
  t.transform.translation.x = x(0);
  t.transform.translation.y = x(1);
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, x(2));
  t.transform.rotation = tf2::toMsg(q);

  tf_broadcaster_->sendTransform(t);
}

}  // namespace ekf_estimator

// ---- main ------------------------------------------------------------------
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ekf_estimator::EkfNode>());
  rclcpp::shutdown();
  return 0;
}
