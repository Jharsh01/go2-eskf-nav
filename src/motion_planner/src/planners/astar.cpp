// astar.cpp
#include "motion_planner/planners/astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>

namespace motion_planner {

namespace {

struct Node {
  int gx, gy;
  double g;       // cost from start
  double f;       // g + h
  bool operator>(const Node& o) const { return f > o.f; }
};

inline int packKey(int gx, int gy, int width) { return gy * width + gx; }

double octileDistance(int x1, int y1, int x2, int y2) {
  const int dx = std::abs(x1 - x2);
  const int dy = std::abs(y1 - y2);
  return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
}

}  // namespace

std::optional<Path> AStarPlanner::plan(const Pose2D& start,
                                       const Pose2D& goal,
                                       const OccupancyGrid& grid) {
  const auto t0 = std::chrono::steady_clock::now();
  stats_ = {};

  int sx, sy, gx, gy;
  if (!grid.worldToGrid(start.x, start.y, sx, sy)) return std::nullopt;
  if (!grid.worldToGrid(goal.x,  goal.y,  gx, gy)) return std::nullopt;
  if (grid.isOccupied(sx, sy) || grid.isOccupied(gx, gy)) return std::nullopt;

  const int W = grid.width();

  std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
  std::unordered_map<int, double> g_score;
  std::unordered_map<int, int>    came_from;

  const int start_key = packKey(sx, sy, W);
  g_score[start_key] = 0.0;
  open.push({sx, sy, 0.0, use_heuristic_ ? octileDistance(sx, sy, gx, gy) : 0.0});

  // 8-connected neighborhood with costs (sqrt2 for diagonals).
  static const int   dx_[8] = {1, -1,  0,  0,  1,  1, -1, -1};
  static const int   dy_[8] = {0,  0,  1, -1,  1, -1,  1, -1};
  static const double cost_[8] = {1, 1, 1, 1, std::sqrt(2.0), std::sqrt(2.0),
                                  std::sqrt(2.0), std::sqrt(2.0)};

  while (!open.empty()) {
    const Node cur = open.top();
    open.pop();
    ++stats_.nodes_explored;

    if (cur.gx == gx && cur.gy == gy) {
      // Reconstruct path by walking parents back to start.
      Path path;
      int key = packKey(cur.gx, cur.gy, W);
      while (key != start_key) {
        int cx = key % W, cy = key / W;
        double wx, wy;
        grid.gridToWorld(cx, cy, wx, wy);
        path.push_back({wx, wy, 0.0});
        key = came_from[key];
      }
      double swx, swy;
      grid.gridToWorld(sx, sy, swx, swy);
      path.push_back({swx, swy, start.theta});
      std::reverse(path.begin(), path.end());
      if (!path.empty()) path.back().theta = goal.theta;

      // Path length in meters.
      double len = 0.0;
      for (size_t i = 1; i < path.size(); ++i)
        len += path[i].point().distanceTo(path[i - 1].point());
      stats_.path_length_m = len;
      stats_.plan_time_ms  = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - t0).count();
      return path;
    }

    const int cur_key = packKey(cur.gx, cur.gy, W);
    if (cur.g > g_score[cur_key]) continue;     // stale entry

    for (int i = 0; i < 8; ++i) {
      const int nx = cur.gx + dx_[i];
      const int ny = cur.gy + dy_[i];
      if (!grid.inBounds(nx, ny) || grid.isOccupied(nx, ny)) continue;
      // Block diagonal "squeezes" through corners.
      if (i >= 4 && (grid.isOccupied(cur.gx + dx_[i], cur.gy)
                  || grid.isOccupied(cur.gx, cur.gy + dy_[i]))) continue;

      const double tentative = cur.g + cost_[i];
      const int    nkey      = packKey(nx, ny, W);
      auto it = g_score.find(nkey);
      if (it == g_score.end() || tentative < it->second) {
        g_score[nkey]   = tentative;
        came_from[nkey] = cur_key;
        const double h  = use_heuristic_ ? octileDistance(nx, ny, gx, gy) : 0.0;
        open.push({nx, ny, tentative, tentative + h});
      }
    }
  }

  stats_.plan_time_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - t0).count();
  return std::nullopt;
}

}  // namespace motion_planner
