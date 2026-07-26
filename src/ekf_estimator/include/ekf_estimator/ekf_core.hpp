// ekf_core.hpp
// 5-state EKF for differential-drive ground robot.
// State: x = [x, y, theta, v, omega]^T
// Pure C++/Eigen — no ROS dependency so it can be unit tested in isolation.

#pragma once

#include <Eigen/Dense>
#include <stdexcept>

namespace ekf_estimator {

constexpr int kStateDim = 5;

using StateVec   = Eigen::Matrix<double, kStateDim, 1>;
using StateMat   = Eigen::Matrix<double, kStateDim, kStateDim>;

class EkfCore {
 public:
  struct Config {
    StateVec initial_state    = StateVec::Zero();
    StateMat initial_covariance = StateMat::Identity() * 1e-3;
    StateMat process_noise_Q    = StateMat::Identity() * 1e-3;
  };

  explicit EkfCore(const Config& cfg);

  // Time update — propagate state forward by dt seconds.
  // Throws std::invalid_argument if dt <= 0.
  void predict(double dt);

  // Measurement updates ----------------------------------------------------

  // IMU yaw-rate measurement: z = omega (scalar).
  void updateImuYawRate(double omega_meas, double R);

  // Wheel odometry measurement: z = [v, omega]^T.
  void updateWheelOdom(const Eigen::Vector2d& z, const Eigen::Matrix2d& R);

  // Full pose from scan matching: z = [x, y, theta]^T.
  // Innovation handles angle wrap on theta.
  void updatePose(const Eigen::Vector3d& z, const Eigen::Matrix3d& R);

  // Accessors --------------------------------------------------------------
  const StateVec& state()      const { return x_; }
  const StateMat& covariance() const { return P_; }

  // Reset estimator to a known state (e.g. on init from /initialpose).
  void reset(const StateVec& x0, const StateMat& P0);

  // Utility: wrap angle to [-pi, pi].
  static double wrapAngle(double a);

 private:
  // Joseph-form covariance update — symmetric and numerically stable.
  template <int MeasDim>
  void josephUpdate(const Eigen::Matrix<double, MeasDim, 1>& y,
                    const Eigen::Matrix<double, MeasDim, kStateDim>& H,
                    const Eigen::Matrix<double, MeasDim, MeasDim>& R);

  StateVec x_;
  StateMat P_;
  StateMat Q_;
};

}  // namespace ekf_estimator
