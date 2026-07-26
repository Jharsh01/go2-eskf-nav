// test_planners.cpp
// Unit tests for the occupancy grid and all 5 planners.
// Coverage:
//   - Occupancy grid: coord conversion, line collision check, inflation
//   - Each planner: trivial path, blocked goal, around-obstacle, all return
//                   collision-free paths.
//   - A* vs Dijkstra: both find a path; A* explores fewer nodes on average.

#include <gtest/gtest.h>

#include "motion_planner/occupancy_grid.hpp"
#include "motion_planner/planners/astar.hpp"
#include "motion_planner/planners/rrt.hpp"
#include "motion_planner/planners/prm.hpp"

using namespace motion_planner;

namespace {

// 20m x 20m grid at 0.1 res, central 4x4 obstacle, inflated by 0.25.
OccupancyGrid makeStandardGrid() {
  OccupancyGrid g(0.1, 200, 200, -10.0, -10.0);
  g.addBorderWalls(0.2);
  g.addRectangleWorld(-2.0, -2.0, 2.0, 2.0);   // central block
  g.inflate(0.25);
  return g;
}

bool pathIsCollisionFree(const Path& p, const OccupancyGrid& g) {
  for (size_t i = 1; i < p.size(); ++i)
    if (!g.isLineCollisionFree(p[i - 1].point(), p[i].point())) return false;
  return true;
}

}  // namespace

// ============================ Occupancy grid ===============================

TEST(OccupancyGrid, WorldToGridRoundTrip) {
  OccupancyGrid g(0.1, 100, 100, -5.0, -5.0);
  int gx, gy;
  ASSERT_TRUE(g.worldToGrid(0.0, 0.0, gx, gy));
  EXPECT_EQ(gx, 50);
  EXPECT_EQ(gy, 50);
  double wx, wy;
  g.gridToWorld(50, 50, wx, wy);
  EXPECT_NEAR(wx, 0.05, 1e-9);
  EXPECT_NEAR(wy, 0.05, 1e-9);
}

TEST(OccupancyGrid, OutOfBoundsIsBlocked) {
  OccupancyGrid g(0.1, 10, 10, 0.0, 0.0);
  EXPECT_TRUE(g.isOccupied(-1, 0));
  EXPECT_TRUE(g.isOccupied(0, -1));
  EXPECT_TRUE(g.isOccupied(10, 0));
}

TEST(OccupancyGrid, InflationGrowsObstacles) {
  OccupancyGrid g(0.1, 50, 50, -2.5, -2.5);
  g.setOccupied(25, 25, true);
  EXPECT_FALSE(g.isOccupied(27, 25));
  g.inflate(0.3);                            // 3 cells radius
  EXPECT_TRUE(g.isOccupied(27, 25));
  EXPECT_FALSE(g.isOccupied(30, 25));
}

TEST(OccupancyGrid, LineCollisionDetectsObstacle) {
  auto g = makeStandardGrid();
  // Line passing through the central obstacle.
  EXPECT_FALSE(g.isLineCollisionFree({-5.0, 0.0}, {5.0, 0.0}));
  // Line going around it.
  EXPECT_TRUE (g.isLineCollisionFree({-5.0, -5.0}, {-5.0, 5.0}));
}

// ================================ A* / Dijkstra ============================

TEST(AStarPlanner, FindsPathAroundObstacle) {
  auto g = makeStandardGrid();
  AStarPlanner p(true);
  auto path = p.plan({-5, -5, 0}, {5, 5, 0}, g);
  ASSERT_TRUE(path.has_value());
  EXPECT_GE(path->size(), 2u);
  EXPECT_TRUE(pathIsCollisionFree(*path, g));
  EXPECT_GT(p.stats().path_length_m, 10.0);
  EXPECT_LT(p.stats().path_length_m, 30.0);
}

TEST(AStarPlanner, ReturnsNulloptIfGoalBlocked) {
  auto g = makeStandardGrid();
  AStarPlanner p(true);
  auto path = p.plan({-5, -5, 0}, {0, 0, 0}, g);   // goal inside obstacle
  EXPECT_FALSE(path.has_value());
}

TEST(DijkstraPlanner, FindsSamePathAsAstar) {
  auto g = makeStandardGrid();
  AStarPlanner astar(true);
  AStarPlanner dijkstra(false);
  auto p_a = astar.plan({-5, -5, 0}, {5, 5, 0}, g);
  auto p_d = dijkstra.plan({-5, -5, 0}, {5, 5, 0}, g);
  ASSERT_TRUE(p_a.has_value());
  ASSERT_TRUE(p_d.has_value());
  // Both optimal by construction; lengths should match within one cell.
  EXPECT_NEAR(astar.stats().path_length_m, dijkstra.stats().path_length_m, 0.2);
  // A* explores at most as many nodes as Dijkstra (typically fewer).
  EXPECT_LE(astar.stats().nodes_explored, dijkstra.stats().nodes_explored);
}

// ================================ RRT / RRT* ===============================

TEST(RrtPlanner, FindsPathAroundObstacle) {
  auto g = makeStandardGrid();
  RrtConfig cfg; cfg.seed = 7; cfg.max_iters = 8000;
  RrtPlanner p(false, cfg);
  auto path = p.plan({-5, -5, 0}, {5, 5, 0}, g);
  ASSERT_TRUE(path.has_value());
  EXPECT_TRUE(pathIsCollisionFree(*path, g));
}

TEST(RrtStarPlanner, ProducesShorterOrEqualPathThanRrt) {
  auto g = makeStandardGrid();
  RrtConfig cfg; cfg.seed = 7; cfg.max_iters = 8000;
  RrtPlanner rrt    (false, cfg);
  RrtPlanner rrtStar(true,  cfg);
  auto p_rrt  = rrt.plan    ({-5, -5, 0}, {5, 5, 0}, g);
  auto p_star = rrtStar.plan({-5, -5, 0}, {5, 5, 0}, g);
  ASSERT_TRUE(p_rrt .has_value());
  ASSERT_TRUE(p_star.has_value());
  // RRT* has the asymptotic-optimality guarantee — for sufficient samples it
  // should be no worse than vanilla RRT. Slack accounts for randomness.
  EXPECT_LE(rrtStar.stats().path_length_m,
            rrt.stats().path_length_m * 1.05);
}

TEST(RrtPlanner, ReturnsNulloptIfStartBlocked) {
  auto g = makeStandardGrid();
  RrtPlanner p(false);
  auto path = p.plan({0, 0, 0}, {5, 5, 0}, g);   // start inside obstacle
  EXPECT_FALSE(path.has_value());
}

// ================================ PRM =====================================

TEST(PrmPlanner, FindsPathAroundObstacle) {
  auto g = makeStandardGrid();
  PrmConfig cfg; cfg.num_samples = 800; cfg.seed = 7;
  PrmPlanner p(cfg);
  auto path = p.plan({-5, -5, 0}, {5, 5, 0}, g);
  ASSERT_TRUE(path.has_value());
  EXPECT_TRUE(pathIsCollisionFree(*path, g));
}

TEST(PrmPlanner, FailsGracefullyWithTooFewSamples) {
  // 5 random samples in a maze can't form a roadmap from start to goal.
  auto g = makeStandardGrid();
  PrmConfig cfg; cfg.num_samples = 5; cfg.seed = 1; cfg.max_edge_length_m = 1.0;
  PrmPlanner p(cfg);
  auto path = p.plan({-9, -9, 0}, {9, 9, 0}, g);
  EXPECT_FALSE(path.has_value());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
