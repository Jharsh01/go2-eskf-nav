// test_eskf_core.cpp
// Unit tests for the 8-state ESKF: rotation helpers, IMU prediction (gravity
// cancellation, acceleration/yaw integration, bias), leg-odom & GPS
// corrections, and covariance health (symmetry / positive-definiteness).

#include <gtest/gtest.h>

#include <cmath>

#include "go2_eskf/eskf_core.hpp"

using namespace go2_eskf;
using idx::BG;
using idx::PSI;
using idx::PX;
using idx::PY;
using idx::VX;
using idx::VY;

namespace {

EskfCore makeFresh(double p0 = 1e-3) {
  EskfCore::Config cfg;
  cfg.initial_state = Vec8::Zero();
  cfg.initial_covariance = Mat8::Identity() * p0;
  return EskfCore(cfg);
}

// A still, upright IMU reads +g on its z axis (reaction to gravity).
const Eigen::Vector3d kRestAccel(0.0, 0.0, kGravity);

bool isSymmetric(const Mat8& M, double tol = 1e-9) {
  return (M - M.transpose()).cwiseAbs().maxCoeff() < tol;
}
bool isPosDef(const Mat8& M, double tol = 1e-12) {
  Eigen::SelfAdjointEigenSolver<Mat8> es(M);
  return es.info() == Eigen::Success && es.eigenvalues().minCoeff() > -tol;
}

}  // namespace

// === Rotation helpers ======================================================

TEST(EskfRotation, IdentityAtZero) {
  EXPECT_TRUE(EskfCore::rotBodyToWorld(0, 0, 0).isApprox(
      Eigen::Matrix3d::Identity()));
}

TEST(EskfRotation, YawRotatesXIntoY) {
  auto R = EskfCore::rotBodyToWorld(0, 0, M_PI / 2);
  Eigen::Vector3d v = R * Eigen::Vector3d(1, 0, 0);
  EXPECT_NEAR(v(0), 0.0, 1e-12);
  EXPECT_NEAR(v(1), 1.0, 1e-12);
}

TEST(EskfRotation, YawDerivativeMatchesFiniteDiff) {
  const double r = 0.1, p = -0.2, y = 0.7, h = 1e-6;
  Eigen::Matrix3d num =
      (EskfCore::rotBodyToWorld(r, p, y + h) - EskfCore::rotBodyToWorld(r, p, y - h)) /
      (2 * h);
  EXPECT_TRUE(num.isApprox(EskfCore::dRotDYaw(r, p, y), 1e-6));
}

// === IMU prediction ========================================================

TEST(EskfPredict, RejectsNonPositiveDt) {
  auto e = makeFresh();
  EXPECT_THROW(e.predictImu(kRestAccel, 0, 0, 0, 0.0), std::invalid_argument);
}

TEST(EskfPredict, GravityIsCancelledAtRest) {
  auto e = makeFresh();
  for (int i = 0; i < 100; ++i) e.predictImu(kRestAccel, 0, 0, 0, 0.01);
  // A still robot must not drift in velocity or position.
  EXPECT_NEAR(e.state()(VX), 0.0, 1e-9);
  EXPECT_NEAR(e.state().segment<3>(PX).norm(), 0.0, 1e-9);
}

TEST(EskfPredict, ForwardAccelBuildsVelocityAndPosition) {
  auto e = makeFresh();
  // 1 m/s^2 forward (body x) on top of gravity, for 1 s at 100 Hz.
  Eigen::Vector3d a(1.0, 0.0, kGravity);
  for (int i = 0; i < 100; ++i) e.predictImu(a, 0, 0, 0, 0.01);
  EXPECT_NEAR(e.state()(VX), 1.0, 1e-6);     // v = a*t
  EXPECT_NEAR(e.state()(PX), 0.5, 1e-3);     // x = 0.5*a*t^2
}

TEST(EskfPredict, GyroIntegratesYaw) {
  auto e = makeFresh();
  for (int i = 0; i < 100; ++i) e.predictImu(kRestAccel, 0.5, 0, 0, 0.01);
  EXPECT_NEAR(e.state()(PSI), 0.5, 1e-9);    // yaw = omega*t
}

TEST(EskfPredict, GyroBiasIsSubtracted) {
  EskfCore::Config cfg;
  cfg.initial_state = Vec8::Zero();
  cfg.initial_state(BG) = 0.2;               // a 0.2 rad/s bias
  cfg.initial_covariance = Mat8::Identity() * 1e-3;
  EskfCore e(cfg);
  for (int i = 0; i < 100; ++i) e.predictImu(kRestAccel, 0.2, 0, 0, 0.01);
  // Measured rate equals the bias -> true rate ~0 -> yaw stays put.
  EXPECT_NEAR(e.state()(PSI), 0.0, 1e-9);
}

TEST(EskfPredict, CovarianceGrowsAndStaysHealthy) {
  auto e = makeFresh();
  double before = e.covariance()(VX, VX);
  for (int i = 0; i < 50; ++i) e.predictImu(kRestAccel, 0, 0, 0, 0.01);
  EXPECT_GT(e.covariance()(VX, VX), before);
  EXPECT_TRUE(isSymmetric(e.covariance()));
  EXPECT_TRUE(isPosDef(e.covariance()));
}

// === Corrections ===========================================================

TEST(EskfGps, PullsPositionTowardMeasurement) {
  auto e = makeFresh(1.0);                    // loose prior -> trust GPS
  e.correctGps(Eigen::Vector2d(2.0, -1.0), Eigen::Matrix2d::Identity() * 0.01);
  EXPECT_NEAR(e.state()(PX), 2.0, 0.1);
  EXPECT_NEAR(e.state()(PY), -1.0, 0.1);
  EXPECT_TRUE(isPosDef(e.covariance()));
}

TEST(EskfGps, ReducesPositionUncertainty) {
  auto e = makeFresh(1.0);
  double before = e.covariance()(PX, PX);
  e.correctGps(Eigen::Vector2d(0.5, 0.5), Eigen::Matrix2d::Identity() * 0.01);
  EXPECT_LT(e.covariance()(PX, PX), before);
}

TEST(EskfLegOdom, BodyVelocityUpdatesWorldVelocityAtZeroYaw) {
  auto e = makeFresh(1.0);
  e.correctLegOdom(Eigen::Vector2d(0.3, 0.0), Eigen::Matrix2d::Identity() * 0.01);
  EXPECT_NEAR(e.state()(VX), 0.3, 0.05);     // at yaw=0, body==world
  EXPECT_NEAR(e.state()(VY), 0.0, 0.05);
}

TEST(EskfLegOdom, RespectsYawRotation) {
  // Robot facing +90deg: a forward body velocity maps to +world-y.
  EskfCore::Config cfg;
  cfg.initial_state = Vec8::Zero();
  cfg.initial_state(PSI) = M_PI / 2;
  cfg.initial_covariance = Mat8::Identity() * 1.0;
  EskfCore e(cfg);
  e.correctLegOdom(Eigen::Vector2d(0.4, 0.0), Eigen::Matrix2d::Identity() * 0.01);
  EXPECT_NEAR(e.state()(VX), 0.0, 0.05);
  EXPECT_NEAR(e.state()(VY), 0.4, 0.05);
}

TEST(EskfLegOdom, HigherCovarianceMeansSmallerUpdate) {
  // This is the knob the slip model turns: bigger R -> the filter trusts
  // leg odometry less -> the state moves less.
  auto tight = makeFresh(1.0);
  auto loose = makeFresh(1.0);
  tight.correctLegOdom(Eigen::Vector2d(0.5, 0), Eigen::Matrix2d::Identity() * 0.01);
  loose.correctLegOdom(Eigen::Vector2d(0.5, 0), Eigen::Matrix2d::Identity() * 100.0);
  EXPECT_GT(tight.state()(VX), loose.state()(VX));
}

// === Utility ===============================================================

TEST(EskfUtil, WrapAngleBounds) {
  // +-pi are the same angle; atan2 may return either sign, so compare |.|.
  EXPECT_NEAR(std::abs(EskfCore::wrapAngle(3 * M_PI)), M_PI, 1e-9);
  EXPECT_NEAR(std::abs(EskfCore::wrapAngle(-3 * M_PI)), M_PI, 1e-9);
  EXPECT_NEAR(EskfCore::wrapAngle(0.5), 0.5, 1e-12);
  EXPECT_NEAR(EskfCore::wrapAngle(2 * M_PI + 0.3), 0.3, 1e-9);
}

TEST(EskfUtil, ResetRestoresState) {
  auto e = makeFresh();
  Vec8 x0 = Vec8::Ones();
  e.reset(x0, Mat8::Identity() * 0.5);
  EXPECT_TRUE(e.state().isApprox(x0));
  EXPECT_NEAR(e.covariance()(0, 0), 0.5, 1e-12);
}
