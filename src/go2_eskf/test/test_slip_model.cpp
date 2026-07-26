// test_slip_model.cpp
// Unit tests for the Phase 3 slip model: feature assembly, weights parsing,
// input normalization, the MLP forward pass (vs hand-computed values), sigmoid
// range, and the leg-covariance inflation rule.

#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "go2_eskf/slip_model.hpp"

using namespace go2_eskf;

namespace {

// Build a SlipModel from an in-memory weights string (same format as the file).
SlipModel fromString(const std::string& s) {
  std::stringstream ss(s);
  return SlipModel::loadFromStream(ss);
}

}  // namespace

// === Feature assembly ======================================================

TEST(SlipFeatures, LayoutAndDisagreementTerms) {
  SlipFeatures f;
  f.cmd_vx = 0.5; f.leg_vx = 0.1;     // slip: commanded >> measured
  f.cmd_vy = 0.0; f.leg_vy = -0.2;
  f.cmd_wz = 0.3; f.gyro_wz = 0.1;
  f.joint_vel_mean = 1.5; f.joint_vel_max = 4.0;
  f.accel_horiz = 2.0; f.contact_frac = 0.5;

  Eigen::VectorXd v = f.toVector();
  ASSERT_EQ(v.size(), slip_feat::DIM);
  EXPECT_NEAR(v[slip_feat::CMD_MINUS_LEG_VX], 0.4, 1e-12);
  EXPECT_NEAR(v[slip_feat::CMD_MINUS_LEG_VY], 0.2, 1e-12);
  EXPECT_NEAR(v[slip_feat::CMD_MINUS_GYRO_WZ], 0.2, 1e-12);
  EXPECT_NEAR(v[slip_feat::LEG_SPEED], std::hypot(0.1, -0.2), 1e-12);
  EXPECT_NEAR(v[slip_feat::JOINT_VEL_MEAN], 1.5, 1e-12);
  EXPECT_NEAR(v[slip_feat::JOINT_VEL_MAX], 4.0, 1e-12);
  EXPECT_NEAR(v[slip_feat::ACCEL_HORIZ], 2.0, 1e-12);
  EXPECT_NEAR(v[slip_feat::CONTACT_FRAC], 0.5, 1e-12);
}

// === Forward pass ==========================================================

TEST(SlipModel, LinearForwardMatchesHandComputation) {
  // 2-input single linear neuron, no normalization: y = w.x + b.
  auto m = fromString(
      "input_dim 2\n"
      "mean 0 0\n"
      "std 1 1\n"
      "layers 1\n"
      "layer 2 1 none\n"
      "1 2\n"      // W = [[1, 2]]
      "0.5\n");    // b = [0.5]
  Eigen::VectorXd x(2);
  x << 3.0, 4.0;
  EXPECT_NEAR(m.predict(x), 1 * 3.0 + 2 * 4.0 + 0.5, 1e-12);
}

TEST(SlipModel, AppliesInputStandardization) {
  // mean/std should be applied before the first layer.
  auto m = fromString(
      "input_dim 2\n"
      "mean 1 1\n"
      "std 2 2\n"
      "layers 1\n"
      "layer 2 1 none\n"
      "1 1\n"
      "0\n");
  Eigen::VectorXd x(2);
  x << 3.0, 5.0;                          // -> ((3-1)/2, (5-1)/2) = (1, 2)
  EXPECT_NEAR(m.predict(x), 1.0 + 2.0, 1e-12);  // 1*1 + 1*2 = 3
}

TEST(SlipModel, ZeroStdIsTreatedAsOne) {
  // A constant feature (std 0) must not produce inf/nan.
  auto m = fromString(
      "input_dim 1\n"
      "mean 0\n"
      "std 0\n"
      "layers 1\n"
      "layer 1 1 none\n"
      "1\n"
      "0\n");
  Eigen::VectorXd x(1);
  x << 7.0;
  EXPECT_NEAR(m.predict(x), 7.0, 1e-12);
}

TEST(SlipModel, ReluThenSigmoidTwoLayer) {
  // Layer1: 2->2 relu, Layer2: 2->1 sigmoid. Hand-compute the chain.
  auto m = fromString(
      "input_dim 2\n"
      "mean 0 0\n"
      "std 1 1\n"
      "layers 2\n"
      "layer 2 2 relu\n"
      "1 0\n"      // W1 row0
      "0 -1\n"     // W1 row1
      "0 0\n"      // b1
      "layer 2 1 sigmoid\n"
      "1 1\n"      // W2
      "0\n");      // b2
  Eigen::VectorXd x(2);
  x << 2.0, 3.0;
  // h = relu([2, -3]) = [2, 0]; z = 2 + 0 = 2; sigmoid(2).
  const double expect = 1.0 / (1.0 + std::exp(-2.0));
  EXPECT_NEAR(m.predict(x), expect, 1e-12);
}

TEST(SlipModel, SigmoidOutputInUnitInterval) {
  auto m = fromString(
      "input_dim 1\n"
      "mean 0\n"
      "std 1\n"
      "layers 1\n"
      "layer 1 1 sigmoid\n"
      "1000\n"     // large weight -> drive toward the rails
      "0\n");
  Eigen::VectorXd lo(1), hi(1);
  lo << -50.0;
  hi << 50.0;
  EXPECT_GE(m.predict(lo), 0.0);
  EXPECT_LE(m.predict(lo), 1.0);
  EXPECT_GE(m.predict(hi), 0.0);
  EXPECT_LE(m.predict(hi), 1.0);
  EXPECT_GT(m.predict(hi), m.predict(lo));  // monotone in input
}

// === Error handling ========================================================

TEST(SlipModel, RejectsFeatureDimMismatch) {
  auto m = fromString(
      "input_dim 2\nmean 0 0\nstd 1 1\nlayers 1\nlayer 2 1 none\n1 1\n0\n");
  Eigen::VectorXd x(3);
  x << 1, 2, 3;
  EXPECT_THROW(m.predict(x), std::invalid_argument);
}

TEST(SlipModel, RejectsNonChainingLayers) {
  EXPECT_THROW(fromString("input_dim 2\nmean 0 0\nstd 1 1\nlayers 2\n"
                          "layer 2 3 relu\n1 1\n1 1\n1 1\n0 0 0\n"
                          "layer 2 1 sigmoid\n1 1\n0\n"),  // expects in=3, gets 2
               std::runtime_error);
}

TEST(SlipModel, RejectsNonScalarOutput) {
  EXPECT_THROW(fromString("input_dim 2\nmean 0 0\nstd 1 1\nlayers 1\n"
                          "layer 2 2 none\n1 0\n0 1\n0 0\n"),
               std::runtime_error);
}

TEST(SlipModel, UnloadedModelThrows) {
  SlipModel m;
  EXPECT_FALSE(m.loaded());
  Eigen::VectorXd x(1);
  x << 0.0;
  EXPECT_THROW(m.predict(x), std::runtime_error);
}

// === Covariance inflation ==================================================

TEST(SlipModel, AdaptLegCovarianceScalesWithSlip) {
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * 0.04;

  // No slip -> unchanged.
  EXPECT_TRUE(SlipModel::adaptLegCovariance(R, 0.0, 1.0).isApprox(R));
  // lambda 0 -> unchanged regardless of slip.
  EXPECT_TRUE(SlipModel::adaptLegCovariance(R, 1.0, 0.0).isApprox(R));

  // s=1, lambda=2 -> factor (1+2)^2 = 9.
  Eigen::Matrix2d big = SlipModel::adaptLegCovariance(R, 1.0, 2.0);
  EXPECT_TRUE(big.isApprox(R * 9.0));

  // Monotone increasing in slip.
  double a = SlipModel::adaptLegCovariance(R, 0.2, 1.0)(0, 0);
  double b = SlipModel::adaptLegCovariance(R, 0.8, 1.0)(0, 0);
  EXPECT_GT(b, a);
}

TEST(SlipModel, CommentsAndBlankLinesAreSkipped) {
  auto m = fromString(
      "# a comment\n"
      "input_dim 1\n"
      "# mean line\n"
      "mean 0\n"
      "std 1\n"
      "layers 1\n"
      "layer 1 1 none\n"
      "2\n"
      "1\n");
  Eigen::VectorXd x(1);
  x << 3.0;
  EXPECT_NEAR(m.predict(x), 2 * 3.0 + 1.0, 1e-12);
}
