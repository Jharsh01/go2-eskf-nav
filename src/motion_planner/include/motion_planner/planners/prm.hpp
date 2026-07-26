// prm.hpp
// Probabilistic Roadmap. Builds a graph by sampling N collision-free points
// and connecting each to its k nearest neighbors, then runs Dijkstra for
// the start->goal query.

#pragma once

#include "motion_planner/planner_base.hpp"

namespace motion_planner {

struct PrmConfig {
  int    num_samples       = 500;
  int    k_neighbors       = 8;
  double max_edge_length_m = 2.0;
  unsigned seed            = 42;
};

class PrmPlanner : public PlannerBase {
 public:
  explicit PrmPlanner(PrmConfig cfg = {}) : cfg_(cfg) {}

  std::optional<Path> plan(const Pose2D& start,
                           const Pose2D& goal,
                           const OccupancyGrid& grid) override;

  std::string name() const override { return "prm"; }

 private:
  PrmConfig cfg_;
};

}  // namespace motion_planner
