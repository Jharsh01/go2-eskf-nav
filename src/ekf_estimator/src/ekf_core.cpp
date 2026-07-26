// ekf_core.cpp
#include "ekf_estimator/ekf_core.hpp"

#include <cmath>

namespace ekf_estimator {

EkfCore::EkfCore(const Config& cfg)
    : x_(cfg.initial_state), P_(cfg.initial_covariance), Q_(cfg.process_noise_Q) {}

double EkfCore::wrapAngle(double a) {
  // Map any angle to [-pi, pi]. Using atan2 is more robust than fmod for
  // edge cases near the boundary.
  return std::atan2(std::sin(a), std::cos(a));
}

void EkfCore::reset(const StateVec& x0, const StateMat& P0) {
  x_ = x0;
  P_ = P0;
}

void EkfCore::predict(double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument("EkfCore::predict — dt must be > 0");
  }

  const double theta = x_(2);
  const double v     = x_(3);
  const double omega = x_(4);
  const double c     = std::cos(theta);
  const double s     = std::sin(theta);

  // --- Nonlinear state propagation: constant-velocity / turn-rate model.
  StateVec x_pred;
  x_pred(0) = x_(0) + v * c * dt;
  x_pred(1) = x_(1) + v * s * dt;
  x_pred(2) = wrapAngle(theta + omega * dt);
  x_pred(3) = v;       // random-walk: mean unchanged, Q absorbs uncertainty
  x_pred(4) = omega;

  // --- Jacobian F = df/dx evaluated at current x.
  StateMat F = StateMat::Identity();
  F(0, 2) = -v * s * dt;
  F(0, 3) =  c * dt;
  F(1, 2) =  v * c * dt;
  F(1, 3) =  s * dt;
  F(2, 4) =  dt;

  // --- Covariance propagation. Q scales with dt so tuning is rate-agnostic.
  P_ = F * P_ * F.transpose() + Q_ * dt;
  x_ = x_pred;
}

template <int MeasDim>
void EkfCore::josephUpdate(const Eigen::Matrix<double, MeasDim, 1>& y,
                           const Eigen::Matrix<double, MeasDim, kStateDim>& H,
                           const Eigen::Matrix<double, MeasDim, MeasDim>& R) {
  // Innovation covariance.
  Eigen::Matrix<double, MeasDim, MeasDim> S = H * P_ * H.transpose() + R;
  // Kalman gain via solve (avoid explicit inverse).
  Eigen::Matrix<double, kStateDim, MeasDim> K =
      P_ * H.transpose() * S.inverse();

  x_ += K * y;
  x_(2) = wrapAngle(x_(2));   // keep heading bounded after every update

  // Joseph form: P = (I - KH) P (I - KH)^T + K R K^T
  StateMat I_KH = StateMat::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EkfCore::updateImuYawRate(double omega_meas, double R_scalar) {
  Eigen::Matrix<double, 1, kStateDim> H;
  H << 0, 0, 0, 0, 1;

  Eigen::Matrix<double, 1, 1> y, R;
  y(0) = omega_meas - x_(4);
  R(0) = R_scalar;
  josephUpdate<1>(y, H, R);
}

void EkfCore::updateWheelOdom(const Eigen::Vector2d& z, const Eigen::Matrix2d& R) {
  Eigen::Matrix<double, 2, kStateDim> H;
  H << 0, 0, 0, 1, 0,
       0, 0, 0, 0, 1;

  Eigen::Vector2d y;
  y(0) = z(0) - x_(3);
  y(1) = z(1) - x_(4);
  josephUpdate<2>(y, H, R);
}

void EkfCore::updatePose(const Eigen::Vector3d& z, const Eigen::Matrix3d& R) {
  Eigen::Matrix<double, 3, kStateDim> H;
  H << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0,
       0, 0, 1, 0, 0;

  Eigen::Vector3d y;
  y(0) = z(0) - x_(0);
  y(1) = z(1) - x_(1);
  y(2) = wrapAngle(z(2) - x_(2));   // critical: wrap heading innovation
  josephUpdate<3>(y, H, R);
}

// Explicit instantiations so the linker is happy.
template void EkfCore::josephUpdate<1>(
    const Eigen::Matrix<double, 1, 1>&,
    const Eigen::Matrix<double, 1, kStateDim>&,
    const Eigen::Matrix<double, 1, 1>&);
template void EkfCore::josephUpdate<2>(
    const Eigen::Matrix<double, 2, 1>&,
    const Eigen::Matrix<double, 2, kStateDim>&,
    const Eigen::Matrix<double, 2, 2>&);
template void EkfCore::josephUpdate<3>(
    const Eigen::Matrix<double, 3, 1>&,
    const Eigen::Matrix<double, 3, kStateDim>&,
    const Eigen::Matrix<double, 3, 3>&);

}  // namespace ekf_estimator
