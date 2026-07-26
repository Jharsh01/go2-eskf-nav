// test_ekf_core.cpp
// 17 unit tests covering motion model, Jacobians, all three update types,
// covariance properties, angle wrapping, and reset.

#include <gtest/gtest.h>
#include "ekf_estimator/ekf_core.hpp"

#include <cmath>

using namespace ekf_estimator;

namespace {

EkfCore makeFreshEkf(double p0 = 0.1, double q = 1e-4) {
  EkfCore::Config cfg;
  cfg.initial_state      = StateVec::Zero();
  cfg.initial_covariance = StateMat::Identity() * p0;
  cfg.process_noise_Q    = StateMat::Identity() * q;
  return EkfCore(cfg);
}

bool isPosDef(const StateMat& M, double tol = 1e-9) {
  Eigen::SelfAdjointEigenSolver<StateMat> es(M);
  if (es.info() != Eigen::Success) return false;
  return es.eigenvalues().minCoeff() > -tol;
}

bool isSymmetric(const StateMat& M, double tol = 1e-9) {
  return (M - M.transpose()).cwiseAbs().maxCoeff() < tol;
}

}  // namespace

// === Motion model tests (4) ================================================

TEST(EkfCoreMotion, StationaryRobotDoesNotMove) {
  auto ekf = makeFreshEkf();
  ekf.predict(0.1);
  EXPECT_NEAR(ekf.state()(0), 0.0, 1e-12);
  EXPECT_NEAR(ekf.state()(1), 0.0, 1e-12);
  EXPECT_NEAR(ekf.state()(2), 0.0, 1e-12);
}

TEST(EkfCoreMotion, ForwardVelocityMovesAlongX) {
  auto ekf = makeFreshEkf();
  StateVec x0 = StateVec::Zero();
  x0(3) = 1.0;                       // v = 1 m/s, theta = 0
  ekf.reset(x0, StateMat::Identity() * 0.1);
  ekf.predict(2.0);
  EXPECT_NEAR(ekf.state()(0), 2.0, 1e-9);
  EXPECT_NEAR(ekf.state()(1), 0.0, 1e-9);
}

TEST(EkfCoreMotion, PureRotationRotatesHeading) {
  auto ekf = makeFreshEkf();
  StateVec x0 = StateVec::Zero();
  x0(4) = 1.0;                       // omega = 1 rad/s
  ekf.reset(x0, StateMat::Identity() * 0.1);
  ekf.predict(1.0);
  EXPECT_NEAR(ekf.state()(2), 1.0, 1e-9);
  EXPECT_NEAR(ekf.state()(0), 0.0, 1e-9);
}

TEST(EkfCoreMotion, NegativeDtThrows) {
  auto ekf = makeFreshEkf();
  EXPECT_THROW(ekf.predict(-0.01), std::invalid_argument);
  EXPECT_THROW(ekf.predict(0.0),   std::invalid_argument);
}

// === Jacobian / covariance growth tests (3) ================================

TEST(EkfCoreJacobian, CovarianceGrowsDuringPredict) {
  auto ekf = makeFreshEkf(0.01, 1e-3);
  const double trace_before = ekf.covariance().trace();
  ekf.predict(0.5);
  EXPECT_GT(ekf.covariance().trace(), trace_before);
}

TEST(EkfCoreJacobian, CovarianceStaysPositiveDefinite) {
  auto ekf = makeFreshEkf(0.05, 1e-3);
  StateVec x0;
  x0 << 0, 0, 0.5, 1.0, 0.3;
  ekf.reset(x0, StateMat::Identity() * 0.05);
  for (int i = 0; i < 100; ++i) ekf.predict(0.05);
  EXPECT_TRUE(isPosDef(ekf.covariance()));
  EXPECT_TRUE(isSymmetric(ekf.covariance()));
}

TEST(EkfCoreJacobian, CovarianceSymmetryAfterUpdate) {
  auto ekf = makeFreshEkf();
  ekf.predict(0.1);
  ekf.updateImuYawRate(0.2, 1e-3);
  EXPECT_TRUE(isSymmetric(ekf.covariance()));
  EXPECT_TRUE(isPosDef(ekf.covariance()));
}

// === IMU update tests (3) ==================================================

TEST(EkfCoreImu, YawRateMeasurementPullsOmega) {
  auto ekf = makeFreshEkf(0.1, 1e-4);
  for (int i = 0; i < 50; ++i) {
    ekf.predict(0.02);
    ekf.updateImuYawRate(0.5, 1e-4);
  }
  EXPECT_NEAR(ekf.state()(4), 0.5, 0.05);
}

TEST(EkfCoreImu, HighMeasurementNoiseIgnoresMeasurement) {
  auto ekf = makeFreshEkf(0.01, 1e-6);     // tight prior
  ekf.updateImuYawRate(10.0, 1e6);          // useless measurement
  EXPECT_NEAR(ekf.state()(4), 0.0, 1e-3);
}

TEST(EkfCoreImu, OmegaCovarianceShrinksAfterUpdate) {
  auto ekf = makeFreshEkf(0.1, 1e-4);
  const double var_before = ekf.covariance()(4, 4);
  ekf.updateImuYawRate(0.0, 1e-4);
  EXPECT_LT(ekf.covariance()(4, 4), var_before);
}

// === Wheel odom update tests (2) ===========================================

TEST(EkfCoreOdom, BothVelocitiesPulled) {
  auto ekf = makeFreshEkf(0.1, 1e-4);
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 1e-4;
  for (int i = 0; i < 50; ++i) {
    ekf.predict(0.02);
    ekf.updateWheelOdom(Eigen::Vector2d(1.0, 0.2), R);
  }
  EXPECT_NEAR(ekf.state()(3), 1.0, 0.05);
  EXPECT_NEAR(ekf.state()(4), 0.2, 0.05);
}

TEST(EkfCoreOdom, PositionDoesNotJumpFromVelocityOnly) {
  auto ekf = makeFreshEkf(0.1, 1e-4);
  const double x_before = ekf.state()(0);
  ekf.updateWheelOdom(Eigen::Vector2d(1.0, 0.0), Eigen::Matrix2d::Identity() * 1e-4);
  EXPECT_NEAR(ekf.state()(0), x_before, 1e-9);
}

// === Pose update tests (3) =================================================

TEST(EkfCorePose, FullPoseMeasurementCorrectsPosition) {
  auto ekf = makeFreshEkf(0.5, 1e-4);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1e-4;
  ekf.updatePose(Eigen::Vector3d(2.0, 1.0, 0.3), R);
  EXPECT_NEAR(ekf.state()(0), 2.0, 0.05);
  EXPECT_NEAR(ekf.state()(1), 1.0, 0.05);
  EXPECT_NEAR(ekf.state()(2), 0.3, 0.05);
}

TEST(EkfCorePose, AngleWrapInInnovation) {
  // Prior heading at +pi - 0.05; measurement at -pi + 0.05.
  // Naive subtraction would yield ~ -2pi + 0.10 (huge, wraps the wrong way);
  // correct innovation is ~ +0.10.
  auto ekf = makeFreshEkf(0.01, 1e-4);
  StateVec x0;
  x0 << 0, 0, M_PI - 0.05, 0, 0;
  ekf.reset(x0, StateMat::Identity() * 0.01);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 1e-4;
  ekf.updatePose(Eigen::Vector3d(0.0, 0.0, -M_PI + 0.05), R);
  // After update, heading should sit very close to the wrap boundary —
  // crucially NOT swing all the way back through 0.
  EXPECT_GT(std::fabs(ekf.state()(2)), 3.0);   // |theta| close to pi
  EXPECT_LE(std::fabs(ekf.state()(2)), M_PI + 1e-9);
}

TEST(EkfCorePose, PositionVarianceShrinksAfterUpdate) {
  auto ekf = makeFreshEkf(0.5, 1e-4);
  const double var_x_before = ekf.covariance()(0, 0);
  ekf.updatePose(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity() * 1e-4);
  EXPECT_LT(ekf.covariance()(0, 0), var_x_before);
}

// === Utility tests (2) =====================================================

TEST(EkfCoreUtil, AngleWrapBoundary) {
  EXPECT_NEAR(EkfCore::wrapAngle(0.0),               0.0,   1e-12);
  EXPECT_NEAR(EkfCore::wrapAngle(M_PI),              M_PI,  1e-12);
  // -pi maps to either +pi or -pi at the wrap boundary; both are valid.
  EXPECT_NEAR(std::fabs(EkfCore::wrapAngle(-M_PI)),  M_PI,  1e-12);
  EXPECT_NEAR(std::fabs(EkfCore::wrapAngle(3 * M_PI)), M_PI, 1e-12);
  EXPECT_NEAR(EkfCore::wrapAngle(-3 * M_PI / 2),     M_PI / 2, 1e-12);
}

TEST(EkfCoreUtil, ResetRestoresState) {
  auto ekf = makeFreshEkf();
  ekf.predict(0.5);
  ekf.updateImuYawRate(1.0, 1e-3);
  StateVec x0; x0 << 5, 5, 0.5, 0, 0;
  StateMat P0 = StateMat::Identity() * 0.2;
  ekf.reset(x0, P0);
  EXPECT_TRUE(ekf.state().isApprox(x0));
  EXPECT_TRUE(ekf.covariance().isApprox(P0));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
