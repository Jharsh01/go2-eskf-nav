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

TEST(EskfYaw, PullsHeadingAndShrinksItsUncertainty) {
  auto e = makeFresh(1.0);
  const double before = e.covariance()(PSI, PSI);
  e.correctYaw(0.4, 0.01);
  EXPECT_NEAR(e.state()(PSI), 0.4, 0.01);
  EXPECT_LT(e.covariance()(PSI, PSI), before);
  EXPECT_TRUE(isPosDef(e.covariance()));
}

TEST(EskfYaw, InnovationWrapsAcrossThePiSeam) {
  // psi = 3.1 and a measurement of -3.1 are 0.083 rad apart, not 6.2. An
  // unwrapped innovation would drag psi through zero the long way round.
  EskfCore::Config cfg;
  cfg.initial_state = Vec8::Zero();
  cfg.initial_state(PSI) = 3.1;
  cfg.initial_covariance = Mat8::Identity() * 1.0;
  EskfCore e(cfg);
  e.correctYaw(-3.1, 0.01);
  const double moved = EskfCore::wrapAngle(e.state()(PSI) - 3.1);
  EXPECT_GT(moved, 0.0);                      // went up through +pi...
  EXPECT_LT(moved, 2 * M_PI - 6.2 + 1e-6);    // ...by at most the short arc
  EXPECT_NEAR(EskfCore::wrapAngle(e.state()(PSI) + 3.1), 0.0, 0.01);
}

TEST(EskfYaw, ObservesGyroBiasWithoutAnyOtherSensor) {
  // A stationary robot with a biased gyro, heading pinned by a magnetometer:
  // the yaw update must pull b_g onto the bias (via F(PSI,BG) = -dt) — this is
  // what bounds heading drift with GPS off, which leg odometry alone cannot.
  auto e = makeFresh(1e-2);
  const double bias = 0.02;  // [rad/s], ~69 deg of drift per minute
  for (int k = 0; k < 6000; ++k) {           // 60 s at 100 Hz
    e.predictImu(kRestAccel, bias, 0, 0, 0.01);
    if (k % 10 == 0) e.correctYaw(0.0, 0.05 * 0.05);
  }
  EXPECT_NEAR(e.state()(BG), bias, 2e-3);
  EXPECT_NEAR(e.state()(PSI), 0.0, 0.02);
  EXPECT_TRUE(isPosDef(e.covariance()));
}

TEST(EskfMag, LevelHeadingMatchesYaw) {
  const Eigen::Vector3d B_world(5.5645e-6, 22.8758e-6, -42.3884e-6);  // gz default
  const double field_heading = std::atan2(B_world.y(), B_world.x());
  for (double yaw : {0.0, 0.7, -2.0, 3.0}) {
    const Eigen::Matrix3d R = EskfCore::rotBodyToWorld(0, 0, yaw);
    double psi = 99.0;
    ASSERT_TRUE(EskfCore::magHeading(R.transpose() * B_world,
                                     R.transpose() * Eigen::Vector3d::UnitZ(),
                                     field_heading, &psi));
    EXPECT_NEAR(EskfCore::wrapAngle(psi - yaw), 0.0, 1e-9) << "yaw " << yaw;
  }
}

TEST(EskfMag, TiltCompensatedAndMountIndependent) {
  // Roll/pitch must not leak into heading — including a sensor mounted upside
  // down (roll = pi), as the sim IMU's frame appears to be (DESIGN.md §6).
  const Eigen::Vector3d B_world(5.5645e-6, 22.8758e-6, -42.3884e-6);
  const double field_heading = std::atan2(B_world.y(), B_world.x());
  const double cases[][3] = {{0.2, -0.15, 1.0}, {-0.3, 0.25, -2.5},
                             {M_PI, 0.0, 0.6}, {M_PI, 0.2, -1.2}};
  for (const auto& c : cases) {
    const Eigen::Matrix3d R = EskfCore::rotBodyToWorld(c[0], c[1], c[2]);
    double psi = 99.0;
    ASSERT_TRUE(EskfCore::magHeading(R.transpose() * B_world,
                                     R.transpose() * Eigen::Vector3d::UnitZ(),
                                     field_heading, &psi));
    EXPECT_NEAR(EskfCore::wrapAngle(psi - c[2]), 0.0, 1e-9)
        << "roll " << c[0] << " pitch " << c[1] << " yaw " << c[2];
  }
}

TEST(EskfMag, RejectsDegenerateGeometry) {
  double psi = 99.0;
  // Purely vertical field (at a magnetic pole): no horizontal direction.
  EXPECT_FALSE(EskfCore::magHeading(Eigen::Vector3d(0, 0, -4e-5),
                                    Eigen::Vector3d::UnitZ(), 0.0, &psi));
  // Forward axis pointing straight up: the body has no heading.
  EXPECT_FALSE(EskfCore::magHeading(Eigen::Vector3d(1e-5, 2e-5, -4e-5),
                                    Eigen::Vector3d::UnitX(), 0.0, &psi));
  EXPECT_FALSE(EskfCore::magHeading(Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d::UnitZ(), 0.0, &psi));
  EXPECT_EQ(psi, 99.0);  // untouched on failure
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
