// types.hpp
// Common 2D primitives shared by all planners.

#pragma once

#include <vector>
#include <cmath>

namespace motion_planner {

struct Point2D {
  double x{0.0};
  double y{0.0};
  double distanceTo(const Point2D& other) const {
    const double dx = x - other.x;
    const double dy = y - other.y;
    return std::sqrt(dx * dx + dy * dy);
  }
};

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  Point2D point() const { return {x, y}; }
};

using Path = std::vector<Pose2D>;

}  // namespace motion_planner
