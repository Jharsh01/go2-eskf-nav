// eskf_core.hpp
// 8-state error-state EKF for a quadruped (Unitree Go2), fusing:
//   * IMU            (prediction: accelerometer + yaw-rate gyro)
//   * leg odometry   (correction: body-frame velocity from CHAMP)
//   * GPS            (correction: world-frame x,y position)
//
// "Error-state" means: the nonlinear nominal state is propagated directly,
// while the Kalman filter tracks the small *error* dx about that nominal and
// injects it back after every correction. For a planar-yaw robot the yaw
// error is additive (SO(2)), so injection is a simple add — but the structure
// (predict nominal -> correct error -> inject -> reset) is the standard ESKF
// recipe and extends cleanly to full 3-D attitude later.
//
// Pure C++/Eigen — no ROS. Mirrored exactly by scripts/eskf_reference.py.

#pragma once

#include <Eigen/Dense>
#include <stdexcept>

#include "go2_eskf/types.hpp"

namespace go2_eskf {

class EskfCore {
 public:
  struct Config {
    Vec8 initial_state = Vec8::Zero();
    Mat8 initial_covariance = Mat8::Identity() * 1e-3;

    // Continuous-time IMU noise densities (std-devs). Used to build the
    // discrete process noise Q each predict step.
    double accel_noise = 0.10;      // [m/s^2]   accelerometer white noise
    double gyro_noise = 2.0e-3;     // [rad/s]   yaw-rate white noise
    double gyro_bias_noise = 1.0e-4;// [rad/s/sqrt(s)] gyro-bias random walk
    // Fractional yaw-rate SCALE uncertainty. The dominant yaw-rate error in the
    // Go2 sim is not white noise but a scale error — wz_gyro/wz_truth measured
    // 0.830 and 0.964 during turns (2026-07-26) — and integrating that
    // open-loop is what produced 4-97 deg of heading error over a 10 m square.
    // Q(psi,psi) therefore carries an extra (gyro_scale_noise * wz)^2 term, so
    // heading becomes uncertain exactly while turning, which is when the error
    // enters, letting leg odometry's body-frame vy residual pull yaw back.
    // Set 0 for white-noise-only behaviour.
    double gyro_scale_noise = 0.10;  // [-] fraction of |wz|
  };

  explicit EskfCore(const Config& cfg);

  // --- Prediction (IMU-driven) ----------------------------------------------
  // accel_body : specific force from the accelerometer, body frame [m/s^2]
  // gyro_z     : yaw-rate from the gyro, body frame                [rad/s]
  // roll,pitch : body attitude taken directly from the IMU         [rad]
  // dt         : time step                                          [s]  (>0)
  void predictImu(const Eigen::Vector3d& accel_body, double gyro_z,
                  double roll, double pitch, double dt);

  // --- Corrections ----------------------------------------------------------
  // Leg odometry: body-frame planar velocity [vx, vy]. R is the (adaptive)
  // measurement covariance — the slip model scales this up when feet slip.
  void correctLegOdom(const Eigen::Vector2d& v_body_meas,
                      const Eigen::Matrix2d& R);

  // GPS: world-frame position [x, y].
  void correctGps(const Eigen::Vector2d& pos_xy, const Eigen::Matrix2d& R);

  // Gyro-bias pseudo-measurement. Leg odometry's yaw rate is an independent,
  // bias-free measurement of the true yaw rate, so (gyro_wz - wz_leg) measures
  // the gyro bias directly: h = b_g. This makes b_g strongly observable (it is
  // otherwise only weakly coupled through the leg-velocity direction), which is
  // the main lever against long-horizon yaw drift when no absolute heading
  // sensor exists.
  void correctGyroBias(double bias_meas, double r);

  // Vertical pseudo-measurement: world-frame vertical velocity vz. On flat
  // ground the base's mean vertical velocity is ~0 (it bobs but does not
  // climb), so feeding vz_meas=0 keeps the OTHERWISE UNOBSERVABLE vertical
  // channel (pz, vz) from double-integrating IMU/gravity residuals to infinity.
  // R should be loose enough to permit gait bob (~0.3 m/s).
  void correctVerticalVel(double vz_meas, double r);

  // --- Accessors ------------------------------------------------------------
  const Vec8& state() const { return x_; }
  const Mat8& covariance() const { return P_; }
  void reset(const Vec8& x0, const Mat8& P0);

  // --- Utilities ------------------------------------------------------------
  static double wrapAngle(double a);
  // Body->world rotation R = Rz(yaw) Ry(pitch) Rx(roll).
  static Eigen::Matrix3d rotBodyToWorld(double roll, double pitch, double yaw);
  // Derivative of the above w.r.t. yaw (needed for the Jacobian).
  static Eigen::Matrix3d dRotDYaw(double roll, double pitch, double yaw);

 private:
  template <int MeasDim>
  void josephUpdate(const Eigen::Matrix<double, MeasDim, 1>& y,
                    const Eigen::Matrix<double, MeasDim, kStateDim>& H,
                    const Eigen::Matrix<double, MeasDim, MeasDim>& R);

  Vec8 x_;
  Mat8 P_;
  Config cfg_;
};

}  // namespace go2_eskf
