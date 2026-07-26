// test_pure_pursuit.cpp
#include <gtest/gtest.h>
#include "motion_planner/pure_pursuit.hpp"

using namespace motion_planner;

TEST(PurePursuit, EmptyPathDoesNothing) {
  PurePursuit pp;
  Pose2D r{0, 0, 0};
  Path empty;
  double v, w; bool reached;
  pp.compute(r, empty, v, w, reached);
  EXPECT_DOUBLE_EQ(v, 0.0);
  EXPECT_DOUBLE_EQ(w, 0.0);
  EXPECT_FALSE(reached);
}

TEST(PurePursuit, ReachedFlagSetWithinTolerance) {
  PurePursuitConfig cfg; cfg.goal_tol_m = 0.2;
  PurePursuit pp(cfg);
  Pose2D r{0, 0, 0};
  Path path = {{0.05, 0.0, 0.0}};
  double v, w; bool reached;
  pp.compute(r, path, v, w, reached);
  EXPECT_TRUE(reached);
}

TEST(PurePursuit, StraightAheadGivesPositiveLinear) {
  PurePursuit pp;
  Pose2D r{0, 0, 0};
  Path path = {{0, 0, 0}, {1, 0, 0}, {2, 0, 0}, {3, 0, 0}};
  double v, w; bool reached;
  pp.compute(r, path, v, w, reached);
  EXPECT_GT(v, 0.0);
  EXPECT_NEAR(w, 0.0, 0.05);
  EXPECT_FALSE(reached);
}

TEST(PurePursuit, LeftTurnGivesPositiveOmega) {
  PurePursuit pp;
  Pose2D r{0, 0, 0};
  // Path runs to the left of the robot.
  Path path = {{0, 0, 0}, {0.3, 0.5, 0}, {0.6, 1.0, 0}};
  double v, w; bool reached;
  pp.compute(r, path, v, w, reached);
  EXPECT_GT(w, 0.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
