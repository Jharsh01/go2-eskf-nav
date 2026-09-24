// eskf_node.cpp
#include "go2_eskf/eskf_node.hpp"

#include <algorithm>
#include <cstdint>
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
  cfg.gyro_scale_noise = declare_parameter<double>("gyro_scale_noise", 0.10);
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
  // Zero-velocity update: when the robot is genuinely commanded to stand still,
  // leg odom's (0,0) is a real and very precise measurement, so trust it far more
  // than a walking sample.
  const double zupt_std = declare_parameter<double>("zupt_vel_noise", 0.02);
  R_zupt_ = Eigen::Matrix2d::Identity() * (zupt_std * zupt_std);
  // See the header for why champ's all-zero leg-odom samples must not be fused.
  // Set false to restore the previous (fuse-everything) behaviour for an A/B run.
  leg_odom_gate_degenerate_ =
      declare_parameter<bool>("leg_odom_gate_degenerate", true);
  degenerate_hold_sec_ = declare_parameter<double>("degenerate_hold_sec", 0.3);
  stationary_cmd_eps_ = declare_parameter<double>("stationary_cmd_eps", 0.01);
  cmd_vel_stale_sec_ = declare_parameter<double>("cmd_vel_stale_sec", 0.5);
  // Std-dev of the vz≈0 vertical pseudo-measurement applied with each leg-odom
  // update (loose enough to permit the gait's vertical bob).
  vz_zero_noise_ = declare_parameter<double>("vz_zero_noise", 0.3);
  // CHAMP multiplies its leg-odom velocities by gait.yaml's odom_scaler (0.9 in
  // this sim) — a real-robot slip fudge that systematically under-reports speed
  // by 10% on sim's no-slip floor (~1 m of drift per 10 m walked). This scale
  // undoes it (1/0.9 ≈ 1.111). Set 1.0 when the source odometry is unscaled.
  leg_odom_scale_ = declare_parameter<double>("leg_odom_scale", 1.0);
  // Gyro-bias observability from leg yaw rate: z = gyro_wz - wz_leg = b_g + n.
  use_leg_yaw_bias_ = declare_parameter<bool>("use_leg_yaw_bias", true);
  const double byaw_std = declare_parameter<double>("leg_yaw_bias_noise", 0.05);
  r_leg_yaw_bias_ = byaw_std * byaw_std;
  // Only fuse the bias pseudo-measurement when barely rotating (see legOdomCallback).
  // A huge value disables the gate, restoring the previous always-fuse behaviour.
  bias_update_max_wz_ = declare_parameter<double>("bias_update_max_wz", 0.10);

  // --- Phase 3: slip-adaptive leg covariance
  use_slip_model_ = declare_parameter<bool>("use_slip_model", false);
  slip_lambda_ = declare_parameter<double>("slip_lambda", 1.0);
  const auto slip_model_path = declare_parameter<std::string>("slip_model_path", "");
  const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  const auto joint_topic = declare_parameter<std::string>("joint_states_topic", "joint_states");
  const auto contacts_topic =
      declare_parameter<std::string>("foot_contacts_topic", "foot_contacts");
  // Training-data tap: one CSV row of slip features per fused leg-odom update.
  // Independent of use_slip_model_ — the features are all sensor-derived, so the
  // BASELINE arm can record a training set while the slip arm is still running
  // whatever model exists today.
  gps_datum_samples_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int>("gps_datum_samples", 10)));
  const auto slip_log_path = declare_parameter<std::string>("slip_log_path", "");

  // --- Magnetometer heading (see the header for the reference/gate design)
  use_mag_ = declare_parameter<bool>("use_mag", false);
  const auto mag_topic = declare_parameter<std::string>("mag_topic", "imu/mag");
  const double mag_std = declare_parameter<double>("mag_heading_noise", 0.05);
  r_mag_ = mag_std * mag_std;
  mag_min_interval_ = declare_parameter<double>("mag_min_interval", 0.1);
  mag_tilt_gain_ = declare_parameter<double>("mag_tilt_gain", 0.60);
  mag_gate_sigma_ = declare_parameter<double>("mag_gate_sigma", 3.0);
  mag_gate_reset_sec_ = declare_parameter<double>("mag_gate_reset_sec", 5.0);
  mag_norm_gate_ = declare_parameter<double>("mag_norm_gate", 0.25);
  mag_ref_samples_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int>("mag_ref_samples", 50)));
  mag_calibrate_heading_ = declare_parameter<bool>("mag_calibrate_heading", true);
  mag_field_heading_ = declare_parameter<double>("mag_field_heading", 0.0699);
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
  if (use_mag_) {
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
        mag_topic, rclcpp::SensorDataQoS(),
        std::bind(&EskfNode::magCallback, this, std::placeholders::_1));
    mag_heading_pub_ =
        create_publisher<std_msgs::msg::Float64>("eskf/mag_heading", 10);
  }
  if (!gt_topic.empty()) {
    gt_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        gt_topic, 10,
        std::bind(&EskfNode::groundTruthCallback, this, std::placeholders::_1));
  }
  // Always needed: the degenerate-leg-odom gate uses the commanded twist to tell a
  // genuine stop apart from a mid-gait "no information" sample. (It doubles as a
  // slip-model feature.)
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, 10,
      std::bind(&EskfNode::cmdVelCallback, this, std::placeholders::_1));
  // The slip features are needed for inference AND for recording a training set,
  // so subscribe whenever either is on.
  const bool need_slip_features = use_slip_model_ || !slip_log_path.empty();
  if (need_slip_features) {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, rclcpp::SensorDataQoS(),
        std::bind(&EskfNode::jointStateCallback, this, std::placeholders::_1));
#ifdef GO2_ESKF_HAS_CHAMP_MSGS
    contacts_sub_ = create_subscription<champ_msgs::msg::ContactsStamped>(
        contacts_topic, rclcpp::SensorDataQoS(),
        std::bind(&EskfNode::contactsCallback, this, std::placeholders::_1));
#else
    RCLCPP_WARN(get_logger(),
                "built without champ_msgs: /%s is not subscribed, so the slip "
                "model's contact_frac feature stays at 1.0.",
                contacts_topic.c_str());
#endif
  }
  if (use_slip_model_) {
    // "slip_score", not "slip": this is a Float64 diagnostic in [0,1], NOT the
    // slip arm's odometry. The old name sat one character away from the slip
    // ARM's namespace (/eskf_slip/odom) and was repeatedly read as if it were
    // the second trajectory. It is not — plot_trajectory.py's third curve comes
    // from /eskf_slip/odom; this topic only feeds REPORT.md's slip-score line.
    slip_pub_ = create_publisher<std_msgs::msg::Float64>("eskf/slip_score", 10);
  }
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(out_topic, 10);
  gyro_bias_pub_ = create_publisher<std_msgs::msg::Float64>("eskf/gyro_bias", 10);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  if (!log_path.empty()) {
    log_file_.open(log_path);
    log_file_ << "t,est_x,est_y,est_z,est_yaw,est_vx,est_vy,est_bg,"
                 "gt_x,gt_y,gt_yaw\n";
    RCLCPP_INFO(get_logger(), "Logging estimate vs ground truth to %s",
                log_path.c_str());
  }

  if (!slip_log_path.empty()) {
    slip_log_file_.open(slip_log_path);
    // Feature columns first, in slip_feat:: order, so the file is readable by
    // scripts/slip_reference.py's SLIP_FEATURES directly; then the raw inputs and
    // the ground-truth twist that the trainer needs to build the label.
    slip_log_file_ << "t,cmd_minus_leg_vx,cmd_minus_leg_vy,cmd_minus_gyro_wz,"
                      "leg_speed,joint_vel_mean,joint_vel_max,accel_horiz,"
                      "contact_frac,leg_vx,leg_vy,gyro_wz,cmd_vx,cmd_vy,cmd_wz,"
                      "have_contacts,gt_vx,gt_vy,gt_wz\n";
    RCLCPP_INFO(get_logger(),
                "Logging slip features%s to %s",
                gt_topic.empty()
                    ? " (NO ground_truth_topic set — rows will have no label)"
                    : "",
                slip_log_path.c_str());
  }

  RCLCPP_INFO(get_logger(),
              "go2_eskf node up. IMU=%s leg=%s gps=%s(%s) mag=%s(%s) -> %s",
              imu_topic.c_str(), leg_topic.c_str(), gps_topic.c_str(),
              use_gps_ ? "on" : "off", mag_topic.c_str(),
              use_mag_ ? "on" : "off", out_topic.c_str());
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
  // Track gravity+mount with a slow low-pass. Updated for EVERY attitude source:
  // besides gravity_lp's motion extraction below, it is the magnetometer's "up"
  // vector (in the IMU's own frame, so the frame flip is irrelevant there too).
  grav_lp_ = grav_lp_beta_ * grav_lp_ + (1.0 - grav_lp_beta_) * accel;
  // How far the real tilt has run ahead of grav_lp_: the same first-order filter's
  // high-pass, driven by the gyro's roll/pitch rates (see the header).
  tilt_hp_ = grav_lp_beta_ *
             (tilt_hp_ + Eigen::Vector2d(msg->angular_velocity.x,
                                         msg->angular_velocity.y) * dt);
  if (attitude_source_ == "gravity_lp") {
    // Subtract the low-passed gravity to leave motion acceleration, then re-add
    // +g on z so EskfCore's gravity term cancels and a_world = Rz(yaw) * motion.
    // Robust to the IMU frame flip and to spikes.
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
  const double wz_leg = msg->twist.twist.angular.z;

  // Classify the sample BEFORE leg_odom_scale_ is applied. champ writes literal
  // zeros in its "nothing to calculate" branch (all four or zero feet in contact),
  // so an exact comparison is the right test — this is a flag, not a small reading.
  const bool degenerate = msg->twist.twist.linear.x == 0.0 &&
                          msg->twist.twist.linear.y == 0.0 && wz_leg == 0.0;
  ++n_leg_msgs_;
  if (degenerate) ++n_leg_degenerate_;

  // Track how long the current unbroken run of degenerate samples has lasted —
  // that duration is what separates standing still from a flight phase.
  const rclcpp::Time stamp(msg->header.stamp);
  if (degenerate) {
    if (!in_degenerate_run_) {
      in_degenerate_run_ = true;
      degenerate_run_start_ = stamp;
    }
  } else {
    in_degenerate_run_ = false;
  }

  // A degenerate sample carries information only if the robot really is still:
  // then (0,0,0) is a genuine zero-velocity update, and a stationary robot is also
  // the single best gyro-bias observation available. Mid-gait it means nothing, and
  // fusing it corrupts both velocity and b_g.
  const bool stationary = degenerate && looksGenuinelyStationary(stamp);
  const bool skip = degenerate && leg_odom_gate_degenerate_ && !stationary;
  // Tighten R only when the gate is enabled, so leg_odom_gate_degenerate:=false
  // reproduces the previous behaviour exactly and stays a valid A/B baseline.
  const bool zupt = degenerate && leg_odom_gate_degenerate_ && stationary;
  if (skip) ++n_leg_skipped_;

  // The vz≈0 anchor applies either way: it is what keeps the unobservable vertical
  // channel from diverging (pz reached 1601 m without it), and vz_zero_noise_ is
  // loose enough to tolerate both the gait's bob and a flight phase.
  eskf_->correctVerticalVel(0.0, vz_zero_noise_ * vz_zero_noise_);

  if (!skip) {
    // CHAMP publishes body-frame velocity in the twist, pre-scaled by its
    // odom_scaler fudge; leg_odom_scale_ undoes that so speed is unbiased. (Note
    // it cannot undo a degenerate zero — scaling zero gives zero, which is why
    // gating matters more than the scale factor.)
    const double vx = msg->twist.twist.linear.x * leg_odom_scale_;
    const double vy = msg->twist.twist.linear.y * leg_odom_scale_;
    const Eigen::Vector2d v_body(vx, vy);
    // Record training data at exactly the point where R_leg is chosen, so the
    // logged distribution is the inference distribution: post-scale, post-gate,
    // ZUPT samples excluded (those use R_zupt_, not the slip model).
    if (slip_log_file_.is_open() && !zupt) {
      logSlipFeatures(stamp, currentSlipFeatures(vx, vy));
    }
    const Eigen::Matrix2d R = zupt ? R_zupt_ : legCovarianceForUpdate(vx, vy);
    eskf_->correctLegOdom(v_body, R);
    // Leg yaw rate is bias-free, so (gyro - leg) observes the gyro bias directly —
    // the main lever against long-horizon yaw drift
    // the main lever against long-horizon yaw drift with no absolute heading.
    //
    // BUT only while the robot is NOT rotating. MEASURED 2026-07-26: the residual
    // (gyro_wz - wz_leg) is ~-0.005 rad/s walking straight but +0.045..+0.052 rad/s
    // during a wz=+0.4 turn, reproducibly — because wz_leg carries a
    // rotation-dependent scale error (wz_leg/wz_truth and vx_leg/vx_truth both drop
    // while turning). A gyro bias is a slowly-varying constant, so a residual that
    // tracks |wz| is not bias: fusing it injects ~+0.05 rad/s (~2.9 deg/s) of FALSE
    // bias at every corner, and predict then integrates psi += (gyro_z - b_g)*dt.
    // Over a ~4 s corner that is ~10 deg of heading error per corner.
    if (use_leg_yaw_bias_) {
      const double wz_mag = std::max(std::abs(gyro_wz_), std::abs(wz_leg));
      if (wz_mag <= bias_update_max_wz_) {
        eskf_->correctGyroBias(gyro_wz_ - wz_leg, r_leg_yaw_bias_);
      } else {
        ++n_bias_skipped_;
      }
    }
  }

  // Surface the two quantities that made this bug hard to see: how much of the leg
  // odometry is no-information, and where the gyro bias has ended up.
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "leg odom: %lu msgs, %.1f%% degenerate (all-zero), %lu skipped, "
      "%lu bias updates skipped while turning | b_g=%+.5f rad/s",
      static_cast<unsigned long>(n_leg_msgs_),
      100.0 * static_cast<double>(n_leg_degenerate_) /
          static_cast<double>(n_leg_msgs_),
      static_cast<unsigned long>(n_leg_skipped_),
      static_cast<unsigned long>(n_bias_skipped_), eskf_->state()(BG));
}

bool EskfNode::looksGenuinelyStationary(const rclcpp::Time& stamp) const {
  // Primary test: the zeros have persisted longer than any gait flight phase.
  if (!in_degenerate_run_) return false;
  if ((stamp - degenerate_run_start_).seconds() < degenerate_hold_sec_) {
    return false;
  }
  // Veto: a FRESH command asking for motion contradicts "stopped", so stay
  // conservative and treat the sample as no-information instead of a ZUPT. A
  // stale command expresses no opinion either way (teleop only publishes on
  // keypress), so it cannot veto.
  if (have_cmd_vel_ &&
      (stamp - last_cmd_vel_time_).seconds() <= cmd_vel_stale_sec_) {
    const bool commanding_motion = std::abs(cmd_vx_) >= stationary_cmd_eps_ ||
                                   std::abs(cmd_vy_) >= stationary_cmd_eps_ ||
                                   std::abs(cmd_wz_) >= stationary_cmd_eps_;
    if (commanding_motion) return false;
  }
  return true;
}

void EskfNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vx_ = msg->linear.x;
  cmd_vy_ = msg->linear.y;
  cmd_wz_ = msg->angular.z;
  // Twist is unstamped, so timestamp arrival ourselves; this clock follows
  // use_sim_time, matching the leg-odom header stamps the gate compares against.
  last_cmd_vel_time_ = now();
  have_cmd_vel_ = true;
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

#ifdef GO2_ESKF_HAS_CHAMP_MSGS
void EskfNode::contactsCallback(
    const champ_msgs::msg::ContactsStamped::SharedPtr msg) {
  if (msg->contacts.empty()) return;
  size_t n = 0;
  for (bool c : msg->contacts) n += c ? 1u : 0u;
  contact_frac_ = static_cast<double>(n) /
                  static_cast<double>(msg->contacts.size());
  have_contacts_ = true;
}
#endif

SlipFeatures EskfNode::currentSlipFeatures(double leg_vx, double leg_vy) const {
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
  return feat;
}

void EskfNode::logSlipFeatures(const rclcpp::Time& stamp,
                               const SlipFeatures& feat) {
  if (!slip_log_file_.is_open()) return;
  const Eigen::VectorXd f = feat.toVector();
  slip_log_file_ << stamp.seconds();
  for (int i = 0; i < f.size(); ++i) slip_log_file_ << ',' << f(i);
  slip_log_file_ << ',' << feat.leg_vx << ',' << feat.leg_vy << ','
                 << feat.gyro_wz << ',' << feat.cmd_vx << ',' << feat.cmd_vy
                 << ',' << feat.cmd_wz << ',' << (have_contacts_ ? 1 : 0)
                 << ',';
  // No truth => no label. Write NaN rather than a zero the trainer could mistake
  // for "leg odometry was perfect here".
  if (have_gt_)
    slip_log_file_ << gt_vx_ << ',' << gt_vy_ << ',' << gt_wz_ << '\n';
  else
    slip_log_file_ << "nan,nan,nan\n";
}

Eigen::Matrix2d EskfNode::legCovarianceForUpdate(double leg_vx, double leg_vy) {
  if (!use_slip_model_ || !slip_model_.loaded()) return R_leg_;

  const SlipFeatures feat = currentSlipFeatures(leg_vx, leg_vy);

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
    // Average the first N fixes rather than trusting one. The datum defines the
    // ENU origin, so its noise becomes a CONSTANT offset on every reported
    // position — the one GPS error no amount of later fusion can average away.
    lat_sum_ += msg->latitude;
    lon_sum_ += msg->longitude;
    if (++gps_datum_count_ < gps_datum_samples_) return;
    lat0_ = lat_sum_ / gps_datum_count_;
    lon0_ = lon_sum_ / gps_datum_count_;
    gps_datum_set_ = true;
    RCLCPP_INFO(get_logger(),
                "GPS datum set from %d fixes: lat=%.7f lon=%.7f",
                gps_datum_count_, lat0_, lon0_);
    return;  // datum maps to ENU origin; nothing to correct on the datum fixes
  }
  double east, north;
  gpsToEnu(msg->latitude, msg->longitude, east, north);
  eskf_->correctGps(Eigen::Vector2d(east, north), R_gps_);
}

void EskfNode::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
  RCLCPP_INFO_ONCE(get_logger(),
      "First magnetometer sample received: |B|=%.3e — heading corrections "
      "start after %d reference samples.",
      std::sqrt(msg->magnetic_field.x * msg->magnetic_field.x +
                msg->magnetic_field.y * msg->magnetic_field.y +
                msg->magnetic_field.z * msg->magnetic_field.z),
      mag_ref_samples_);
  if (!initialized_ || !have_grav_lp_) return;
  ++n_mag_msgs_;

  const Eigen::Vector3d B(msg->magnetic_field.x, msg->magnetic_field.y,
                          msg->magnetic_field.z);
  const double norm = B.norm();
  // Heading of the body relative to the field's horizontal direction.
  double rel = 0.0;
  if (!EskfCore::magHeading(B, grav_lp_, 0.0, &rel)) return;

  if (!mag_ref_set_) {
    // Circular mean, not an arithmetic one: the offsets may straddle +-pi.
    if (mag_calibrate_heading_) {
      const double off = eskf_->state()(PSI) - rel;
      mag_ref_c_ += std::cos(off);
      mag_ref_s_ += std::sin(off);
    }
    mag_norm_sum_ += norm;
    if (++mag_ref_count_ < mag_ref_samples_) return;
    if (mag_calibrate_heading_) {
      mag_field_heading_ = std::atan2(mag_ref_s_, mag_ref_c_);
    }
    mag_ref_norm_ = mag_norm_sum_ / mag_ref_count_;
    mag_ref_set_ = true;
    RCLCPP_INFO(get_logger(),
                "magnetometer reference set from %d samples: field heading "
                "%.4f rad (%s), |B| %.3e",
                mag_ref_count_, mag_field_heading_,
                mag_calibrate_heading_ ? "calibrated against initial yaw"
                                       : "from mag_field_heading",
                mag_ref_norm_);
    return;
  }

  const double psi_meas = EskfCore::wrapAngle(mag_field_heading_ + rel);
  std_msgs::msg::Float64 hm;
  hm.data = psi_meas;
  mag_heading_pub_->publish(hm);

  // A field whose strength has moved off the reference is being distorted
  // (motors, steel, rebar) — and a distorted field points the wrong way too.
  if (mag_norm_gate_ > 0.0 && mag_ref_norm_ > 0.0 &&
      std::abs(norm - mag_ref_norm_) > mag_norm_gate_ * mag_ref_norm_) {
    ++n_mag_norm_rejected_;
    return;
  }
  // Rate-limit: consecutive headings share the tilt estimate's error, so
  // fusing every sample at a white-noise R would be over-confident.
  const rclcpp::Time stamp(msg->header.stamp);
  if (have_mag_fused_ &&
      (stamp - last_mag_fuse_time_).seconds() < mag_min_interval_) {
    return;
  }
  last_mag_fuse_time_ = stamp;
  have_mag_fused_ = true;

  // Tilt-dependent R: a heading taken while "up" lags the body is worth less.
  const double tilt_sigma = mag_tilt_gain_ * tilt_hp_.norm();
  const double r_eff = r_mag_ + tilt_sigma * tilt_sigma;

  // Innovation gate, with a lock-out escape (see the header).
  if (mag_gate_sigma_ > 0.0) {
    const double y = EskfCore::wrapAngle(psi_meas - eskf_->state()(PSI));
    const double s = eskf_->covariance()(PSI, PSI) + r_eff;
    if (y * y > mag_gate_sigma_ * mag_gate_sigma_ * s) {
      if (!mag_rejecting_) {
        mag_rejecting_ = true;
        mag_reject_start_ = stamp;
      }
      if ((stamp - mag_reject_start_).seconds() < mag_gate_reset_sec_) {
        ++n_mag_gate_rejected_;
        return;
      }
      ++n_mag_gate_forced_;
      RCLCPP_WARN(get_logger(),
                  "mag innovation gate rejected every heading for %.1f s "
                  "(innovation %+.1f deg) — fusing this one to avoid lock-out.",
                  mag_gate_reset_sec_, y * 180.0 / M_PI);
    }
    mag_rejecting_ = false;
  }

  eskf_->correctYaw(psi_meas, r_eff);
  ++n_mag_fused_;
  RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "mag: %lu msgs, %lu fused, %lu gated (%lu forced), %lu rejected by the "
      "|B| gate | tilt lag %.1f deg -> sigma %.1f deg | mag heading %+.3f rad, "
      "filter psi %+.3f rad",
      static_cast<unsigned long>(n_mag_msgs_),
      static_cast<unsigned long>(n_mag_fused_),
      static_cast<unsigned long>(n_mag_gate_rejected_),
      static_cast<unsigned long>(n_mag_gate_forced_),
      static_cast<unsigned long>(n_mag_norm_rejected_),
      tilt_hp_.norm() * 180.0 / M_PI, std::sqrt(r_eff) * 180.0 / M_PI,
      psi_meas, eskf_->state()(PSI));
}

void EskfNode::groundTruthCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  gt_x_ = msg->pose.pose.position.x;
  gt_y_ = msg->pose.pose.position.y;
  tf2::Quaternion q;
  tf2::fromMsg(msg->pose.pose.orientation, q);
  double r, p;
  tf2::Matrix3x3(q).getRPY(r, p, gt_yaw_);
  // Body-frame twist (nav_msgs/Odometry convention, and what gz's
  // OdometryPublisher emits) — directly comparable to CHAMP's leg-odom twist,
  // which is what makes it usable as the slip label.
  gt_vx_ = msg->twist.twist.linear.x;
  gt_vy_ = msg->twist.twist.linear.y;
  gt_wz_ = msg->twist.twist.angular.z;
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

  // b_g drives yaw integration (predict uses gyro_z - b_g), so a corrupted bias
  // shows up as heading error long before anything else reveals it.
  std_msgs::msg::Float64 bias_msg;
  bias_msg.data = x(BG);
  gyro_bias_pub_->publish(bias_msg);

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
            << ',' << x(PSI) << ',' << x(VX) << ',' << x(VY) << ',' << x(BG)
            << ',';
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
