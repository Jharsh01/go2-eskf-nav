// planner_base.hpp
// Common interface so the node can swap planners by string name at runtime.

#pragma once

#include <optional>
#include <string>

#include "motion_planner/types.hpp"
#include "motion_planner/occupancy_grid.hpp"

namespace motion_planner {

struct PlannerStats {
  double plan_time_ms{0.0};
  double path_length_m{0.0};
  int    nodes_explored{0};
};

class PlannerBase {
 public:
  virtual ~PlannerBase() = default;
  virtual std::optional<Path> plan(const Pose2D& start,
                                   const Pose2D& goal,
                                   const OccupancyGrid& grid) = 0;
  virtual std::string name() const = 0;
  const PlannerStats& stats() const { return stats_; }

 protected:
  PlannerStats stats_;
};

}  // namespace motion_planner
