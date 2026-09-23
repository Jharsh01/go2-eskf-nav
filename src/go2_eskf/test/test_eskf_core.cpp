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

// === Absolute heading (magnetometer) =======================================

namespace {
// A filter started at a known heading, for the yaw-correction tests.
EskfCore makeAtYaw(double psi, double p0 = 1.0) {
  EskfCore::Config cfg;
  cfg.initial_state = Vec8::Zero();
  cfg.initial_state(PSI) = psi;
  cfg.initial_covariance = Mat8::Identity() * p0;
  return EskfCore(cfg);
}
}  // namespace

TEST(EskfYaw, PullsYawTowardMeasurement) {
  auto e = makeAtYaw(0.0);
  e.correctYaw(0.8, 0.01);
  EXPECT_NEAR(e.state()(PSI), 0.8, 0.05);
}

TEST(EskfYaw, ReducesYawUncertainty) {
  auto e = makeAtYaw(0.0);
  const double before = e.covariance()(PSI, PSI);
  e.correctYaw(0.3, 0.01);
  EXPECT_LT(e.covariance()(PSI, PSI), before);
}

TEST(EskfYaw, HigherCovarianceMeansSmallerUpdate) {
  auto tight = makeAtYaw(0.0);
  auto loose = makeAtYaw(0.0);
  tight.correctYaw(1.0, 0.01);
  loose.correctYaw(1.0, 100.0);
  EXPECT_GT(tight.state()(PSI), loose.state()(PSI));
}

// THE test for this update. psi lives on a circle, so the innovation must be
// wrapped; every other correct*() here differences a linear quantity, and
// copying their pattern would be the bug.
//
// State +3.10 rad, measurement -3.10 rad: the same heading to within 0.083 rad
// the short way round, but 6.20 rad apart if differenced naively. The gain is
// deliberately held LOW (small P, large R) — at gain ~1 both the wrapped and
// unwrapped forms happen to land in the same place, so only a partial update
// tells them apart.
TEST(EskfYaw, InnovationWrapsAcrossThePiSeam) {
  auto e = makeAtYaw(3.10, 0.01);
  e.correctYaw(-3.10, 0.09);  // K = 0.01/(0.01+0.09) = 0.1

  // Short way: 3.10 + 0.1*0.0832 ~= 3.108, i.e. barely moved.
  // Naive difference would give 3.10 + 0.1*(-6.20) ~= 2.48.
  EXPECT_NEAR(e.state()(PSI), 3.108, 0.02);
  EXPECT_GT(e.state()(PSI), 3.0) << "innovation was not wrapped: yaw was "
                                    "driven the long way round the circle";
}

TEST(EskfYaw, MeasurementNearSeamStillConverges) {
  // Same seam, but now trusting the compass: the estimate should end up AT the
  // measurement, expressed in wrapped form.
  auto e = makeAtYaw(3.13, 1.0);
  e.correctYaw(-3.13, 1e-6);
  EXPECT_NEAR(std::abs(EskfCore::wrapAngle(e.state()(PSI) - (-3.13))), 0.0, 1e-3);
}

TEST(EskfYaw, OutputStaysWrapped) {
  auto e = makeAtYaw(3.0);
  e.correctYaw(-3.0, 1e-6);
  EXPECT_LE(std::abs(e.state()(PSI)), M_PI + 1e-9);
}

TEST(EskfYaw, LeavesPositionAndVelocityAloneWhenUncorrelated) {
  // With a diagonal P, H touches only the PSI column, so K is non-zero only in
  // the PSI row: a heading fix must not silently move position or velocity.
  auto e = makeAtYaw(0.0);
  e.correctYaw(1.2, 0.01);
  EXPECT_NEAR(e.state()(PX), 0.0, 1e-12);
  EXPECT_NEAR(e.state()(PY), 0.0, 1e-12);
  EXPECT_NEAR(e.state()(VX), 0.0, 1e-12);
  EXPECT_NEAR(e.state()(VY), 0.0, 1e-12);
}

TEST(EskfYaw, CovarianceStaysHealthy) {
  auto e = makeAtYaw(0.5);
  for (int i = 0; i < 50; ++i) {
    e.predictImu(kRestAccel, 0.05, 0, 0, 0.01);
    e.correctYaw(0.5, 0.0225);
  }
  EXPECT_TRUE(isSymmetric(e.covariance()));
  EXPECT_TRUE(isPosDef(e.covariance()));
}

TEST(EskfYaw, ObservesYawThatLegOdomCannot) {
  // The point of the whole feature. Walking straight, leg odometry leaves yaw
  // uncertainty growing (its Jacobian is blind to the dpsi/dv null direction);
  // an absolute heading collapses it.
  auto no_mag = makeAtYaw(0.0, 0.2);
  auto with_mag = makeAtYaw(0.0, 0.2);
  for (int i = 0; i < 100; ++i) {
    no_mag.predictImu(kRestAccel, 0.0, 0, 0, 0.01);
    with_mag.predictImu(kRestAccel, 0.0, 0, 0, 0.01);
    no_mag.correctLegOdom(Eigen::Vector2d(0.3, 0.0),
                          Eigen::Matrix2d::Identity() * 0.04);
    with_mag.correctLegOdom(Eigen::Vector2d(0.3, 0.0),
                            Eigen::Matrix2d::Identity() * 0.04);
    with_mag.correctYaw(0.0, 0.0225);
  }
  EXPECT_LT(with_mag.covariance()(PSI, PSI), no_mag.covariance()(PSI, PSI));
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
