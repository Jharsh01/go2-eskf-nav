// pure_pursuit.hpp
// Standard geometric path follower: pick a lookahead point on the path,
// compute curvature to reach it, command (v, omega).

#pragma once

#include "motion_planner/types.hpp"

namespace motion_planner {

struct PurePursuitConfig {
  double lookahead_m   = 0.5;
  double max_v         = 0.6;
  double max_omega     = 1.5;
  double goal_tol_m    = 0.15;
  double slow_radius_m = 1.0;     // start slowing within this distance of goal
};

class PurePursuit {
 public:
  explicit PurePursuit(PurePursuitConfig cfg = {}) : cfg_(cfg) {}

  // Returns (v, omega). reached=true when within goal_tol of last waypoint.
  void compute(const Pose2D& robot, const Path& path,
               double& v_out, double& omega_out, bool& reached_out) const;

  void setConfig(const PurePursuitConfig& cfg) { cfg_ = cfg; }

 private:
  PurePursuitConfig cfg_;
};

}  // namespace motion_planner
