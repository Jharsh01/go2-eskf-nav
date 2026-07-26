// eskf_node.cpp
#include "go2_eskf/eskf_node.hpp"

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace go2_eskf {

using idx::BG;
using idx::PSI;
using idx::PX;
using idx::PY;
using idx::PZ;
using idx::VX;
using idx::VY;

namespace {
constexpr double kEarthRadius = 6378137.0;  // WGS-84 equatorial radius [m]
}  // namespace

EskfNode::EskfNode() : rclcpp::Node("eskf_node") {
  // --- Topics & frames
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "imu/data");
  const auto leg_topic = declare_parameter<std::string>("leg_odom_topic", "odom/raw");
  const auto gps_topic = declare_parameter<std::string>("gps_topic", "gps/fix");
  const auto gt_topic = declare_parameter<std::string>("ground_truth_topic", "");
  const auto out_topic = declare_parameter<std::string>("output_odom_topic", "eskf/odom");
  world_frame_ = declare_parameter<std::string>("world_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  attitude_source_ = declare_parameter<std::string>("attitude_source", "gravity_lp");
  grav_lp_beta_ = declare_parameter<double>("grav_lp_beta", 0.995);
  accel_clip_ = declare_parameter<double>("accel_clip", 40.0);
  // The sim IMU is mounted ~180° flipped (roll≈173°, negative accel-z; DESIGN §6.1).
  // gravity_lp cancels the flipped accel, but the gyro z (yaw rate) is used directly
  // and is therefore negated — set this to -1 to un-flip it so yaw integrates the
  // correct way. +1 for a correctly-mounted IMU.
  gyro_z_sign_ = declare_parameter<double>("gyro_z_sign", 1.0);
  publish_tf_ = declare_parameter<bool>("publish_tf", false);
  use_gps_ = declare_parameter<bool>("use_gps", true);
  max_imu_dt_ = declare_parameter<double>("max_imu_dt", 0.05);
  const auto log_path = declare_parameter<std::string>("log_path", "");

  // --- Filter configuration
  EskfCore::Config cfg;
  // Default reflects the sim IMU's contact-spike reality: large enough that
  // leg odometry dominates the velocity estimate. See docs/DESIGN.md.
  cfg.accel_noise = declare_parameter<double>("accel_noise", 1.0);
  cfg.gyro_noise = declare_parameter<double>("gyro_noise", 2.0e-3);
  cfg.gyro_bias_noise = declare_parameter<double>("gyro_bias_noise", 1.0e-4);
  const double p_pos = declare_parameter<double>("init_pos_cov", 1.0);
  const double p_vel = declare_parameter<double>("init_vel_cov", 0.5);
  const double p_yaw = declare_parameter<double>("init_yaw_cov", 0.2);
  const double p_bias = declare_parameter<double>("init_bias_cov", 1.0e-3);
  cfg.initial_covariance = Mat8::Zero();
  cfg.initial_covariance.diagonal() << p_pos, p_pos, p_pos, p_vel, p_vel, p_vel,
      p_yaw, p_bias;
  eskf_ = std::make_unique<EskfCore>(cfg);

  const double leg_std = declare_parameter<double>("leg_odom_vel_noise", 0.2);
  const double gps_std = declare_parameter<double>("gps_pos_noise", 0.5);
  R_leg_ = Eigen::Matrix2d::Identity() * (leg_std * leg_std);
  R_gps_ = Eigen::Matrix2d::Identity() * (gps_std * gps_std);
  // Std-dev of the vz≈0 vertical pseudo-measurement applied with each leg-odom
  // update (loose enough to permit the gait's vertical bob).
  vz_zero_noise_ = declare_parameter<double>("vz_zero_noise", 0.3);

  // --- Phase 3: slip-adaptive leg covariance
  use_slip_model_ = declare_parameter<bool>("use_slip_model", false);
  slip_lambda_ = declare_parameter<double>("slip_lambda", 1.0);
  const auto slip_model_path = declare_parameter<std::string>("slip_model_path", "");
  const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  const auto joint_topic = declare_parameter<std::string>("joint_states_topic", "joint_states");
  if (use_slip_model_) {
    if (slip_model_path.empty()) {
      RCLCPP_WARN(get_logger(),
                  "use_slip_model is true but slip_model_path is empty; "
                  "falling back to fixed R_leg.");
      use_slip_model_ = false;
    } else {
      try {
        slip_model_ = SlipModel::loadFromFile(slip_model_path);
        RCLCPP_INFO(get_logger(), "slip model loaded from %s (lambda=%.2f)",
                    slip_model_path.c_str(), slip_lambda_);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "failed to load slip model (%s); using "
                     "fixed R_leg.", e.what());
        use_slip_model_ = false;
      }
    }
  }

  // --- I/O
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&EskfNode::imuCallback, this, std::placeholders::_1));
  leg_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leg_topic, 10,
      std::bind(&EskfNode::legOdomCallback, this, std::placeholders::_1));
  if (use_gps_) {
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic, rclcpp::SensorDataQoS(),
        std::bind(&EskfNode::gpsCallback, this, std::placeholders::_1));
  }
  if (!gt_topic.empty()) {
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        gt_topic, 10,
        std::bind(&EskfNode::groundTruthCallback, this, std::placeholders::_1));
  }
  if (use_slip_model_) {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic, 10,
        std::bind(&EskfNode::cmdVelCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::SensorDataQoS(),
        std::bind(&EskfNode::jointStateCallback, this, std::placeholders::_1));
    slip_pub_ = create_publisher<std_msgs::msg::Float64>("eskf/slip", 10);
  }
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic, 10);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  if (!log_path.empty()) {
    log_file_.open(log_path);
    log_file_ << "t,est_x,est_y,est_z,est_yaw,est_vx,est_vy,gt_x,gt_y,gt_yaw\n";
    RCLCPP_INFO(get_logger(), "Logging estimate vs ground truth to %s",
                log_path.c_str());
  }

  RCLCPP_INFO(get_logger(),
              "go2_eskf node up. IMU=%s leg=%s gps=%s(%s) -> %s",
              imu_topic.c_str(), leg_topic.c_str(), gps_topic.c_str(),
              use_gps_ ? "on" : "off", out_topic.c_str());
}

void EskfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  const Eigen::Vector3d accel(msg->linear_acceleration.x,
                              msg->linear_acceleration.y,
                              msg->linear_acceleration.z);
  const rclcpp::Time stamp(msg->header.stamp);

  if (!initialized_) {
    double yaw_seed = 0.0;
    if (attitude_source_ == "orientation") {
      tf2::Quaternion q;
      tf2::fromMsg(msg->orientation, q);
      double r, p;
      tf2::Matrix3x3(q).getRPY(r, p, yaw_seed);
    }
    grav_lp_ = accel;  // seed the gravity low-pass with the first sample
    have_grav_lp_ = true;
    Vec8 x0 = Vec8::Zero();
    x0(PSI) = yaw_seed;
    eskf_->reset(x0, eskf_->covariance());
    last_imu_time_ = stamp;
    initialized_ = true;
    return;  // need two stamps before the first dt
  }                         

  double dt = (stamp - last_imu_time_).seconds();
  last_imu_time_ = stamp;
  if (dt <= 0.0) return;                  // out-of-order / duplicate stamp
  if (dt > max_imu_dt_) dt = max_imu_dt_; // guard against pauses / dropouts

  // Resolve the effective acceleration and attitude fed to the filter. roll_/
  // pitch_ are only used to cancel gravity (and to label the output odom).
  Eigen::Vector3d accel_eff = accel;
  if (attitude_source_ == "gravity_lp") {
    // Track gravity+mount with a slow low-pass; subtract it to leave motion
    // acceleration, then re-add +g on z so EskfCore's gravity term cancels and
    // a_world = Rz(yaw) * motion. Robust to the IMU frame flip and to spikes.
    grav_lp_ = grav_lp_beta_ * grav_lp_ + (1.0 - grav_lp_beta_) * accel;
    Eigen::Vector3d motion = accel - grav_lp_;
    if (motion.norm() > accel_clip_) motion = motion.normalized() * accel_clip_;
    accel_eff = motion + Eigen::Vector3d(0.0, 0.0, kGravity);
    accel_horiz_ = std::hypot(motion.x(), motion.y());  // slip feature
    roll_ = 0.0;
    pitch_ = 0.0;
  } else if (attitude_source_ == "accel") {
    const double n = accel.norm();
    if (n > 1e-3) {
      const Eigen::Vector3d u = accel / n;
      roll_ = std::atan2(u.y(), u.z());
      pitch_ = std::atan2(-u.x(), std::sqrt(u.y() * u.y() + u.z() * u.z()));
    }
  } else {  // "orientation"
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);
    double yaw;
    tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw);
  }

  gyro_wz_ = gyro_z_sign_ * msg->angular_velocity.z;  // slip feature; sign un-flips the sim IMU
  eskf_->predictImu(accel_eff, gyro_wz_, roll_, pitch_, dt);

  publishEstimate(stamp);
  logRow(stamp);
}

void EskfNode::legOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(),
      "First /odom/raw (leg odom) received: v_body=(%.3f, %.3f) m/s — "
      "velocity corrections are active.",
      msg->twist.twist.linear.x, msg->twist.twist.linear.y);
  if (!initialized_) return;
  // CHAMP publishes body-frame velocity in the twist.
  const double vx = msg->twist.twist.linear.x;
  const double vy = msg->twist.twist.linear.y;
  const Eigen::Vector2d v_body(vx, vy);
  eskf_->correctLegOdom(v_body, legCovarianceForUpdate(vx, vy));
  // A leg-odom message means the robot is walking on the ground, so anchor the
  // otherwise-unobservable vertical channel with vz≈0 (prevents pz drifting to
  // infinity when GPS is off). R is loose enough to allow the gait's vertical bob.
  eskf_->correctVerticalVel(0.0, vz_zero_noise_ * vz_zero_noise_);
}

void EskfNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_wz_ = msg->angular.z;
}

void EskfNode::jointStateCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->velocity.empty()) return;
  double sum = 0.0, mx = 0.0;
  for (double w : msg->velocity) {
    const double a = std::abs(w);
    sum += a;
    if (a > mx) mx = a;
  }
  joint_vel_mean_ = sum / static_cast<double>(msg->velocity.size());
  joint_vel_max_ = mx;
}

Eigen::Matrix2d EskfNode::legCovarianceForUpdate(double leg_vx, double leg_vy) {
  if (!use_slip_model_ || !slip_model_.loaded()) return R_leg_;

  SlipFeatures feat;
  feat.cmd_vx = cmd_vx_;
  feat.cmd_vy = cmd_vy_;
  feat.cmd_wz = cmd_wz_;
  feat.leg_vx = leg_vx;
  feat.leg_vy = leg_vy;
  feat.gyro_wz = gyro_wz_;
  feat.joint_vel_mean = joint_vel_mean_;
  feat.joint_vel_max = joint_vel_max_;
  feat.accel_horiz = accel_horiz_;
  feat.contact_frac = contact_frac_;

  try {
    last_slip_ = slip_model_.predict(feat);
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "slip inference failed (%s); using fixed R_leg.",
                         e.what());
    return R_leg_;
  }

  if (slip_pub_) {
    std_msgs::msg::Float64 m;
    m.data = last_slip_;
    slip_pub_->publish(m);
  }
  return SlipModel::adaptLegCovariance(R_leg_, last_slip_, slip_lambda_);
}

void EskfNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
  // Logged before any gate so it reveals a topic that is alive but being
  // rejected (e.g. status==NO_FIX) vs. a topic that never publishes at all.
  RCLCPP_INFO_ONCE(get_logger(),
      "First /gps/fix received: status=%d lat=%.6f lon=%.6f — GPS is publishing.",
      static_cast<int>(msg->status.status), msg->latitude, msg->longitude);
  if (!initialized_) return;
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN_ONCE(get_logger(),
        "/gps/fix reports STATUS_NO_FIX — GPS corrections are being SKIPPED, so "
        "position is unobservable and will drift. Check the sim navsat sensor.");
    return;
  }

  if (!gps_datum_set_) {
    lat0_ = msg->latitude;
    lon0_ = msg->longitude;
    gps_datum_set_ = true;
    RCLCPP_INFO(get_logger(), "GPS datum set: lat=%.7f lon=%.7f", lat0_, lon0_);
    return;  // datum maps to ENU origin; nothing to correct on the first fix
  }
  double east, north;
  gpsToEnu(msg->latitude, msg->longitude, east, north);
  eskf_->correctGps(Eigen::Vector2d(east, north), R_gps_);
}

void EskfNode::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  gt_x_ = msg->pose.pose.position.x;
  gt_y_ = msg->pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double r, p;
  tf2::Matrix3x3(q).getRPY(r, p, gt_yaw_);
  have_gt_ = true;
}

void EskfNode::gpsToEnu(double lat, double lon, double& east,
                        double& north) const {
  const double deg2rad = M_PI / 180.0;
  const double lat0r = lat0_ * deg2rad;
  east = (lon - lon0_) * deg2rad * kEarthRadius * std::cos(lat0r);
  north = (lat - lat0_) * deg2rad * kEarthRadius;
}

void EskfNode::publishEstimate(const rclcpp::Time& stamp) {
  const Vec8& x = eskf_->state();
  const Mat8& P = eskf_->covariance();

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = world_frame_;
  odom.child_frame_id = base_frame_;

  odom.pose.pose.position.x = x(PX);
  odom.pose.pose.position.y = x(PY);
  odom.pose.pose.position.z = x(PZ);
  tf2::Quaternion q;
  q.setRPY(roll_, pitch_, x(PSI));
  odom.pose.pose.orientation = tf2::toMsg(q);

  // Twist is expressed in the child (body) frame by convention.
  const double c = std::cos(x(PSI)), s = std::sin(x(PSI));
  odom.twist.twist.linear.x = c * x(VX) + s * x(VY);
  odom.twist.twist.linear.y = -s * x(VX) + c * x(VY);

  // Map the relevant state covariances into the 6x6 ROS blocks (x,y,yaw).
  odom.pose.covariance[0] = P(PX, PX);
  odom.pose.covariance[7] = P(PY, PY);
  odom.pose.covariance[35] = P(PSI, PSI);
  odom.twist.covariance[0] = P(VX, VX);
  odom.twist.covariance[7] = P(VY, VY);
  odom_pub_->publish(odom);

  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = x(PX);
    tf.transform.translation.y = x(PY);
    tf.transform.translation.z = x(PZ);
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }
}

void EskfNode::logRow(const rclcpp::Time& stamp) {
  if (!log_file_.is_open()) return;
  const Vec8& x = eskf_->state();
  log_file_ << stamp.seconds() << ',' << x(PX) << ',' << x(PY) << ',' << x(PZ)
            << ',' << x(PSI) << ',' << x(VX) << ',' << x(VY) << ',';
  if (have_gt_)
    log_file_ << gt_x_ << ',' << gt_y_ << ',' << gt_yaw_ << '\n';
  else
    log_file_ << "nan,nan,nan\n";
}

}  // namespace go2_eskf

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_eskf::EskfNode>());
  rclcpp::shutdown();
  return 0;
}
