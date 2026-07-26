// pure_pursuit.cpp
#include "motion_planner/pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace motion_planner {

void PurePursuit::compute(const Pose2D& robot, const Path& path,
                          double& v_out, double& omega_out,
                          bool& reached_out) const {
  v_out       = 0.0;
  omega_out   = 0.0;
  reached_out = false;
  if (path.empty()) return;

  // ---- Goal proximity ---------------------------------------------------
  const Pose2D& goal = path.back();
  const double dist_to_goal = robot.point().distanceTo(goal.point());
  if (dist_to_goal < cfg_.goal_tol_m) {
    reached_out = true;
    return;
  }

  // ---- Find lookahead point.
  // Step 1: find the path waypoint closest to the robot (where we are NOW).
  // Step 2: walk forward from there to the first waypoint at >= lookahead_m.
  // Without step 1 the search starts at path[0], so once we are >lookahead_m
  // past the start of the path, path[0] satisfies the >= test on the very
  // first iteration and gets picked — pulling the robot back to the start
  // forever. The closest-waypoint anchor makes progress monotonic.
  size_t closest_idx = 0;
  double closest_d2  = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.size(); ++i) {
    const double dx_i = path[i].x - robot.x;
    const double dy_i = path[i].y - robot.y;
    const double d2   = dx_i * dx_i + dy_i * dy_i;
    if (d2 < closest_d2) { closest_d2 = d2; closest_idx = i; }
  }
  size_t la_idx = closest_idx;
  for (size_t i = closest_idx; i < path.size(); ++i) {
    if (robot.point().distanceTo(path[i].point()) >= cfg_.lookahead_m) {
      la_idx = i;
      break;
    }
    la_idx = i;     // fall through to last point if none is far enough
  }
  const Pose2D& la = path[la_idx];

  // ---- Geometry: convert lookahead to robot frame ----------------------
  const double dx = la.x - robot.x;
  const double dy = la.y - robot.y;
  const double cos_t = std::cos(robot.theta);
  const double sin_t = std::sin(robot.theta);
  const double x_r =  cos_t * dx + sin_t * dy;
  const double y_r = -sin_t * dx + cos_t * dy;

  // Curvature from pure pursuit: kappa = 2 * y_r / L^2
  const double L2 = x_r * x_r + y_r * y_r;
  if (L2 < 1e-9) return;
  const double kappa = 2.0 * y_r / L2;

  // ---- Speed: ramp down near the goal ----------------------------------
  double v = cfg_.max_v;
  if (dist_to_goal < cfg_.slow_radius_m) {
    v *= std::max(0.2, dist_to_goal / cfg_.slow_radius_m);
  }
  // Reverse-driving avoidance: if lookahead is behind us, rotate in place.
  if (x_r < 0.0) v = 0.0;

  v_out     = v;
  omega_out = std::clamp(v * kappa, -cfg_.max_omega, cfg_.max_omega);
  // If we zeroed v but still need to turn, give it a minimum rotational rate.
  if (v_out < 1e-3) {
    omega_out = std::copysign(std::min(cfg_.max_omega, 0.8), y_r);
  }
  
}

}  // namespace motion_planner
