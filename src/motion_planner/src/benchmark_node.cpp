// benchmark_node.cpp
// Runs all 5 planners on the same start/goal pair and prints a comparison
// table. Useful for the evaluation section of a project report.

#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "motion_planner/occupancy_grid.hpp"
#include "motion_planner/planner_base.hpp"
#include "motion_planner/planners/astar.hpp"
#include "motion_planner/planners/rrt.hpp"
#include "motion_planner/planners/prm.hpp"

using namespace motion_planner;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("planner_benchmark");

  // ---- Build the same demo map the navigation node uses ----------------
  const double res = 0.1;
  const int W = 200, H = 200;
  OccupancyGrid grid(res, W, H, -10.0, -10.0);
  grid.addBorderWalls(0.2);
  grid.addRectangleWorld(-6.0, -6.0, -4.0, -2.0);
  grid.addRectangleWorld( 2.0,  2.0,  5.0,  4.0);
  grid.addRectangleWorld(-2.0,  3.0,  0.0,  6.0);
  grid.addRectangleWorld( 3.0, -5.0,  6.0, -3.0);
  grid.addRectangleWorld(-1.0, -1.0,  1.0,  1.0);
  grid.inflate(0.25);

  Pose2D start{-8.0, -8.0, 0.0};
  Pose2D goal { 8.0,  8.0, 0.0};

  // ---- Build planners --------------------------------------------------
  std::vector<std::unique_ptr<PlannerBase>> planners;
  planners.emplace_back(std::make_unique<AStarPlanner>(true));
  planners.emplace_back(std::make_unique<AStarPlanner>(false));
  planners.emplace_back(std::make_unique<RrtPlanner>(false));
  planners.emplace_back(std::make_unique<RrtPlanner>(true));
  planners.emplace_back(std::make_unique<PrmPlanner>());

  std::cout << "\n===== Motion Planner Benchmark =====\n";
  std::cout << std::left
            << std::setw(12) << "Planner"
            << std::setw(14) << "Time (ms)"
            << std::setw(16) << "Length (m)"
            << std::setw(12) << "Nodes"
            << "Status\n";
  std::cout << std::string(60, '-') << "\n";

  for (auto& p : planners) {
    auto path = p->plan(start, goal, grid);
    const auto& s = p->stats();
    std::cout << std::left
              << std::setw(12) << p->name()
              << std::setw(14) << std::fixed << std::setprecision(2) << s.plan_time_ms
              << std::setw(16) << std::fixed << std::setprecision(2) << s.path_length_m
              << std::setw(12) << s.nodes_explored
              << (path ? "OK" : "FAILED") << "\n";
  }
  std::cout << std::string(60, '-') << "\n";
  std::cout << "Notes: A*/Dijkstra are deterministic; RRT/RRT*/PRM use seed=42.\n";
  std::cout << "       Lower time + shorter length is better.\n\n";

  rclcpp::shutdown();
  return 0;
}
