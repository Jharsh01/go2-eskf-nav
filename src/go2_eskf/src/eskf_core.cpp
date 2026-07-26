// eskf_core.cpp
#include "go2_eskf/eskf_core.hpp"

#include <cmath>

namespace go2_eskf {

using idx::BG;
using idx::PSI;
using idx::PX;
using idx::VX;

EskfCore::EskfCore(const Config& cfg)
    : x_(cfg.initial_state), P_(cfg.initial_covariance), cfg_(cfg) {}

double EskfCore::wrapAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

Eigen::Matrix3d EskfCore::rotBodyToWorld(double roll, double pitch,
                                         double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);

  Eigen::Matrix3d Rx, Ry, Rz;
  Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
  Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
  Rz << cy, -sy, 0, sy, cy, 0, 0, 0, 1;
  return Rz * Ry * Rx;
}

Eigen::Matrix3d EskfCore::dRotDYaw(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);

  Eigen::Matrix3d Rx, Ry, dRz;
  Rx << 1, 0, 0, 0, cr, -sr, 0, sr, cr;
  Ry << cp, 0, sp, 0, 1, 0, -sp, 0, cp;
  // d/dyaw of Rz.
  dRz << -sy, -cy, 0, cy, -sy, 0, 0, 0, 0;
  return dRz * Ry * Rx;
}

void EskfCore::reset(const Vec8& x0, const Mat8& P0) {
  x_ = x0;
  P_ = P0;
}

void EskfCore::predictImu(const Eigen::Vector3d& accel_body, double gyro_z,
                          double roll, double pitch, double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument("EskfCore::predictImu — dt must be > 0");
  }

  const double psi = x_(PSI);
  const Eigen::Matrix3d R = rotBodyToWorld(roll, pitch, psi);

  // Specific force -> world linear acceleration (remove gravity).
  const Eigen::Vector3d g_world(0.0, 0.0, -kGravity);
  const Eigen::Vector3d a_world = R * accel_body + g_world;

  const Eigen::Vector3d p = x_.segment<3>(PX);
  const Eigen::Vector3d v = x_.segment<3>(VX);

  // --- Nominal-state propagation (nonlinear, full).
  Vec8 x_pred = x_;
  x_pred.segment<3>(PX) = p + v * dt + 0.5 * a_world * dt * dt;
  x_pred.segment<3>(VX) = v + a_world * dt;
  x_pred(PSI) = wrapAngle(psi + (gyro_z - x_(BG)) * dt);
  x_pred(BG) = x_(BG);  // random walk: mean unchanged

  // --- Error-state transition Jacobian F = d(dx_{k+1})/d(dx_k).
  Mat8 F = Mat8::Identity();
  F.block<3, 3>(PX, VX) = Eigen::Matrix3d::Identity() * dt;          // dp/dv
  const Eigen::Vector3d da_dpsi = dRotDYaw(roll, pitch, psi) * accel_body;
  F.block<3, 1>(VX, PSI) = da_dpsi * dt;                             // dv/dpsi
  F.block<3, 1>(PX, PSI) = 0.5 * da_dpsi * dt * dt;                  // dp/dpsi
  F(PSI, BG) = -dt;                                                  // dpsi/db

  // --- Discrete process noise Q.
  const double sa2 = cfg_.accel_noise * cfg_.accel_noise;
  const double sg2 = cfg_.gyro_noise * cfg_.gyro_noise;
  const double sbg2 = cfg_.gyro_bias_noise * cfg_.gyro_bias_noise;
  Mat8 Q = Mat8::Zero();
  Q.block<3, 3>(PX, PX) = Eigen::Matrix3d::Identity() * (0.25 * sa2 * dt * dt * dt * dt);
  Q.block<3, 3>(VX, VX) = Eigen::Matrix3d::Identity() * (sa2 * dt * dt);
  Q(PSI, PSI) = sg2 * dt * dt;
  Q(BG, BG) = sbg2 * dt;

  P_ = F * P_ * F.transpose() + Q;
  x_ = x_pred;
}

template <int MeasDim>
void EskfCore::josephUpdate(
    const Eigen::Matrix<double, MeasDim, 1>& y,
    const Eigen::Matrix<double, MeasDim, kStateDim>& H,
    const Eigen::Matrix<double, MeasDim, MeasDim>& R) {
  Eigen::Matrix<double, MeasDim, MeasDim> S = H * P_ * H.transpose() + R;
  Eigen::Matrix<double, kStateDim, MeasDim> K =
      P_ * H.transpose() * S.inverse();

  // Inject the error into the nominal state (additive for this state def).
  x_ += K * y;
  x_(PSI) = wrapAngle(x_(PSI));

  // Joseph form keeps P symmetric and positive-definite.
  Mat8 I_KH = Mat8::Identity() - K * H;
  P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
}

void EskfCore::correctLegOdom(const Eigen::Vector2d& v_body_meas,
                              const Eigen::Matrix2d& R) {
  const double psi = x_(PSI);
  const double c = std::cos(psi), s = std::sin(psi);

  // Predicted body-frame planar velocity: v_body = Rz(-psi) * v_world_xy.
  Eigen::Matrix2d Rz_T;
  Rz_T << c, s, -s, c;
  const Eigen::Vector2d v_world_xy(x_(VX), x_(VX + 1));
  const Eigen::Vector2d h = Rz_T * v_world_xy;

  Eigen::Matrix<double, 2, kStateDim> H = Eigen::Matrix<double, 2, kStateDim>::Zero();
  H.block<2, 2>(0, VX) = Rz_T;                       // d h / d v_world_xy
  Eigen::Matrix2d dRzT_dpsi;
  dRzT_dpsi << -s, c, -c, -s;
  H.block<2, 1>(0, PSI) = dRzT_dpsi * v_world_xy;    // d h / d psi

  josephUpdate<2>(v_body_meas - h, H, R);
}

void EskfCore::correctGps(const Eigen::Vector2d& pos_xy,
                          const Eigen::Matrix2d& R) {
  Eigen::Matrix<double, 2, kStateDim> H = Eigen::Matrix<double, 2, kStateDim>::Zero();
  H(0, PX) = 1.0;
  H(1, PX + 1) = 1.0;
  const Eigen::Vector2d h(x_(PX), x_(PX + 1));
  josephUpdate<2>(pos_xy - h, H, R);
}

void EskfCore::correctVerticalVel(double vz_meas, double r) {
  // h = vz (world-frame vertical velocity), state index VX+2.
  Eigen::Matrix<double, 1, kStateDim> H =
      Eigen::Matrix<double, 1, kStateDim>::Zero();
  H(0, VX + 2) = 1.0;
  Eigen::Matrix<double, 1, 1> y, R;
  y(0) = vz_meas - x_(VX + 2);
  R(0) = r;
  josephUpdate<1>(y, H, R);
}

// Explicit instantiations.
template void EskfCore::josephUpdate<2>(
    const Eigen::Matrix<double, 2, 1>&,
    const Eigen::Matrix<double, 2, kStateDim>&,
    const Eigen::Matrix<double, 2, 2>&);
template void EskfCore::josephUpdate<1>(
    const Eigen::Matrix<double, 1, 1>&,
    const Eigen::Matrix<double, 1, kStateDim>&,
    const Eigen::Matrix<double, 1, 1>&);

}  // namespace go2_eskf
