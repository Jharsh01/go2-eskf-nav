// occupancy_grid.cpp
#include "motion_planner/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>

namespace motion_planner {

OccupancyGrid::OccupancyGrid(double resolution,
                             int width_cells, int height_cells,
                             double origin_x, double origin_y)
    : data_(width_cells * height_cells, 0),
      width_(width_cells),
      height_(height_cells),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y) {}

void OccupancyGrid::setOccupied(int gx, int gy, bool occupied) {
  if (!inBounds(gx, gy)) return;
  data_[idx(gx, gy)] = occupied ? 100 : 0;
}

bool OccupancyGrid::isOccupied(int gx, int gy) const {
  if (!inBounds(gx, gy)) return true;     // out-of-bounds = blocked
  return data_[idx(gx, gy)] >= 50;
}

bool OccupancyGrid::inBounds(int gx, int gy) const {
  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

bool OccupancyGrid::worldToGrid(double wx, double wy, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
  return inBounds(gx, gy);
}

void OccupancyGrid::gridToWorld(int gx, int gy, double& wx, double& wy) const {
  wx = origin_x_ + (gx + 0.5) * resolution_;
  wy = origin_y_ + (gy + 0.5) * resolution_;
}

bool OccupancyGrid::isFreeWorld(double wx, double wy) const {
  int gx, gy;
  if (!worldToGrid(wx, wy, gx, gy)) return false;
  return !isOccupied(gx, gy);
}

bool OccupancyGrid::isLineCollisionFree(const Point2D& a, const Point2D& b) const {
  // Sample along the line at half-cell resolution. Cheaper than full Bresenham
  // and accurate enough for sampling-based planners.
  const double dist = a.distanceTo(b);
  const int steps = std::max(1, static_cast<int>(dist / (resolution_ * 0.5)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const double x = a.x + t * (b.x - a.x);
    const double y = a.y + t * (b.y - a.y);
    int gx, gy;
    if (!worldToGrid(x, y, gx, gy)) return false;
    if (isOccupied(gx, gy)) return false;
  }
  return true;
}

void OccupancyGrid::inflate(double radius_m) {
  if (radius_m <= 0.0) return;
  const int r_cells = static_cast<int>(std::ceil(radius_m / resolution_));
  if (r_cells == 0) return;

  // Snapshot the original obstacles, then expand around them.
  std::vector<uint8_t> original = data_;
  for (int gy = 0; gy < height_; ++gy) {
    for (int gx = 0; gx < width_; ++gx) {
      if (original[idx(gx, gy)] < 50) continue;
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          if (dx * dx + dy * dy > r_cells * r_cells) continue;
          const int nx = gx + dx;
          const int ny = gy + dy;
          if (inBounds(nx, ny)) data_[idx(nx, ny)] = 100;
        }
      }
    }
  }
}

void OccupancyGrid::addRectangleWorld(double x_min, double y_min,
                                      double x_max, double y_max) {
  int gx_min, gy_min, gx_max, gy_max;
  worldToGrid(x_min, y_min, gx_min, gy_min);
  worldToGrid(x_max, y_max, gx_max, gy_max);
  gx_min = std::clamp(gx_min, 0, width_  - 1);
  gx_max = std::clamp(gx_max, 0, width_  - 1);
  gy_min = std::clamp(gy_min, 0, height_ - 1);
  gy_max = std::clamp(gy_max, 0, height_ - 1);
  for (int gy = gy_min; gy <= gy_max; ++gy)
    for (int gx = gx_min; gx <= gx_max; ++gx)
      data_[idx(gx, gy)] = 100;
}

void OccupancyGrid::addBorderWalls(double thickness_m) {
  const int t = std::max(1, static_cast<int>(std::ceil(thickness_m / resolution_)));
  for (int gy = 0; gy < height_; ++gy)
    for (int gx = 0; gx < width_; ++gx)
      if (gx < t || gx >= width_ - t || gy < t || gy >= height_ - t)
        data_[idx(gx, gy)] = 100;
}

}  // namespace motion_planner
