// occupancy_grid.hpp
// Simple 2D occupancy grid used by every planner. Origin is the world coord
// of the (0,0) cell's lower-left corner.

#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include "motion_planner/types.hpp"

namespace motion_planner {

class OccupancyGrid {
 public:
  OccupancyGrid(double resolution,
                int width_cells,
                int height_cells,
                double origin_x,
                double origin_y);

  // ---- Cell access ------------------------------------------------------
  void setOccupied(int gx, int gy, bool occupied);
  bool isOccupied(int gx, int gy) const;
  bool inBounds(int gx, int gy) const;
  bool isFreeWorld(double wx, double wy) const;

  // ---- Coordinate conversion -------------------------------------------
  bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
  void gridToWorld(int gx, int gy, double& wx, double& wy) const;

  // ---- Geometric queries ------------------------------------------------
  // Bresenham-style line check between two world-frame points.
  bool isLineCollisionFree(const Point2D& a, const Point2D& b) const;

  // Inflate every occupied cell by `radius_m` meters. Standard nav practice
  // so the planner reasons about the robot footprint instead of a point.
  void inflate(double radius_m);

  // ---- Obstacle helpers used by the world-builder ----------------------
  void addRectangleWorld(double x_min, double y_min,
                         double x_max, double y_max);
  void addBorderWalls(double thickness_m = 0.2);

  // ---- Accessors --------------------------------------------------------
  int width()           const { return width_; }
  int height()          const { return height_; }
  double resolution()   const { return resolution_; }
  double originX()      const { return origin_x_; }
  double originY()      const { return origin_y_; }
  const std::vector<uint8_t>& data() const { return data_; }

 private:
  int idx(int gx, int gy) const { return gy * width_ + gx; }

  std::vector<uint8_t> data_;   // 0=free, 100=occupied (nav_msgs convention)
  int    width_{0};
  int    height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
};

}  // namespace motion_planner
