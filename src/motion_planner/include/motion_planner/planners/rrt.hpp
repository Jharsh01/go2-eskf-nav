// rrt.hpp
// Sampling-based planner. star_rewire_=true upgrades it to RRT*.

#pragma once

#include <random>
#include "motion_planner/planner_base.hpp"

namespace motion_planner {

struct RrtConfig {
  int    max_iters     = 5000;
  double step_size_m   = 0.4;
  double goal_bias     = 0.10;     // probability of sampling the goal
  double goal_radius_m = 0.3;
  double rewire_radius_m = 1.2;    // RRT* only
  unsigned seed        = 42;
};

class RrtPlanner : public PlannerBase {
 public:
  explicit RrtPlanner(bool star_rewire, RrtConfig cfg = {})
      : star_rewire_(star_rewire), cfg_(cfg) {}

  std::optional<Path> plan(const Pose2D& start,
                           const Pose2D& goal,
                           const OccupancyGrid& grid) override;

  std::string name() const override {
    return star_rewire_ ? "rrt_star" : "rrt";
  }

 private:
  bool      star_rewire_;
  RrtConfig cfg_;
};

}  // namespace motion_planner
