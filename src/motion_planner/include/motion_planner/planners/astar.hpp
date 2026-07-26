// astar.hpp
// 8-connected grid search. Set use_heuristic_=false to get pure Dijkstra.

#pragma once

#include "motion_planner/planner_base.hpp"

namespace motion_planner {

class AStarPlanner : public PlannerBase {
 public:
  explicit AStarPlanner(bool use_heuristic = true)
      : use_heuristic_(use_heuristic) {}

  std::optional<Path> plan(const Pose2D& start,
                           const Pose2D& goal,
                           const OccupancyGrid& grid) override;

  std::string name() const override {
    return use_heuristic_ ? "astar" : "dijkstra";
  }

 private:
  bool use_heuristic_;
};

}  // namespace motion_planner
