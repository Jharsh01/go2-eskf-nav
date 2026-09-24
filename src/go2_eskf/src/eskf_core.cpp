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
  // accel_noise / gyro_noise / gyro_bias_noise are CONTINUOUS-TIME densities, so
  // every term integrates as sigma^2 * dt. This previously used dt^2 for the accel
  // and gyro terms (the per-SAMPLE convention) while the bias term already used
  // dt — internally inconsistent, and 100x too small for psi at dt=0.01. The
  // consequence was that P(psi) barely grew, so the filter effectively refused the
  // yaw information leg odometry carries in its body-frame vy residual and heading
  // ran open-loop off the gyro. Position error then tracked yaw error 1:1
  // (measured: 4.3 deg -> 2.3 m, 65 deg -> 12.7 m over a 10 m square).
  const double sa2 = cfg_.accel_noise * cfg_.accel_noise;
  const double sg2 = cfg_.gyro_noise * cfg_.gyro_noise;
  const double sbg2 = cfg_.gyro_bias_noise * cfg_.gyro_bias_noise;
  // Yaw-rate scale error, the dominant heading disturbance here (see Config).
  const double scale_sigma = cfg_.gyro_scale_noise * gyro_z;
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  Mat8 Q = Mat8::Zero();
  // Standard discrete white-noise-acceleration model, including the p-v
  // cross-covariance the previous diagonal-only form omitted.
  Q.block<3, 3>(PX, PX) = I3 * (sa2 * dt * dt * dt / 3.0);
  Q.block<3, 3>(PX, VX) = I3 * (sa2 * dt * dt / 2.0);
  Q.block<3, 3>(VX, PX) = I3 * (sa2 * dt * dt / 2.0);
  Q.block<3, 3>(VX, VX) = I3 * (sa2 * dt);
  Q(PSI, PSI) = (sg2 + scale_sigma * scale_sigma) * dt;
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

void EskfCore::correctGyroBias(double bias_meas, double r) {
  // h = b_g (state index BG); z = gyro_wz - wz_leg = b_g + noise.
  Eigen::Matrix<double, 1, kStateDim> H =
      Eigen::Matrix<double, 1, kStateDim>::Zero();
  H(0, BG) = 1.0;
  Eigen::Matrix<double, 1, 1> y, R;
  y(0) = bias_meas - x_(BG);
  R(0) = r;
  josephUpdate<1>(y, H, R);
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

void EskfCore::correctYaw(double psi_meas, double r) {
  // h = psi (state index PSI). The innovation is wrapped: psi lives on a circle,
  // and an unwrapped 3.1 - (-3.1) = 6.2 rad residual would spin the estimate
  // the long way round.
  Eigen::Matrix<double, 1, kStateDim> H =
      Eigen::Matrix<double, 1, kStateDim>::Zero();
  H(0, PSI) = 1.0;
  Eigen::Matrix<double, 1, 1> y, R;
  y(0) = wrapAngle(psi_meas - x_(PSI));
  R(0) = r;
  josephUpdate<1>(y, H, R);
}

bool EskfCore::magHeading(const Eigen::Vector3d& mag_sensor,
                          const Eigen::Vector3d& up_sensor,
                          double field_heading, double* psi) {
  const double up_n = up_sensor.norm();
  const double mag_n = mag_sensor.norm();
  if (up_n < 1e-9 || mag_n < 1e-12) return false;
  const Eigen::Vector3d u = up_sensor / up_n;

  // Project the field and the sensor's forward axis onto the horizontal plane.
  const Eigen::Vector3d m_h = mag_sensor - mag_sensor.dot(u) * u;
  const Eigen::Vector3d fwd = Eigen::Vector3d::UnitX();
  const Eigen::Vector3d f_h = fwd - fwd.dot(u) * u;
  // Near-vertical field (magnetic pole) or forward axis (robot on its nose):
  // the horizontal projection is noise, so there is no heading to report.
  if (m_h.norm() < 1e-3 * mag_n || f_h.norm() < 0.1) return false;

  // Signed angle from the horizontal field to the horizontal forward axis,
  // measured about up. Level check: m_body = Rz(-psi) m_world, so this angle is
  // psi - field_heading.
  const double rel = std::atan2(u.dot(m_h.cross(f_h)), m_h.dot(f_h));
  *psi = wrapAngle(field_heading + rel);
  return true;
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
