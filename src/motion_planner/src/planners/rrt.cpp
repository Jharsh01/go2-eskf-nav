// rrt.cpp
#include "motion_planner/planners/rrt.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace motion_planner {

namespace {

struct TreeNode {
  Point2D pt;
  int     parent{-1};
  double  cost{0.0};       // cumulative cost from start (RRT* only)
};

int nearestIndex(const std::vector<TreeNode>& tree, const Point2D& p) {
  int    best     = 0;
  double best_d2  = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < tree.size(); ++i) {
    const double dx = tree[i].pt.x - p.x;
    const double dy = tree[i].pt.y - p.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) { best_d2 = d2; best = static_cast<int>(i); }
  }
  return best;
}

Point2D steer(const Point2D& from, const Point2D& to, double step) {
  const double d = from.distanceTo(to);
  if (d <= step) return to;
  const double t = step / d;
  return {from.x + t * (to.x - from.x), from.y + t * (to.y - from.y)};
}

}  // namespace

std::optional<Path> RrtPlanner::plan(const Pose2D& start,
                                     const Pose2D& goal,
                                     const OccupancyGrid& grid) {
  const auto t0 = std::chrono::steady_clock::now();
  stats_ = {};

  if (!grid.isFreeWorld(start.x, start.y)) return std::nullopt;
  if (!grid.isFreeWorld(goal.x,  goal.y))  return std::nullopt;

  std::mt19937 rng(cfg_.seed);
  std::uniform_real_distribution<double> ux(grid.originX(),
      grid.originX() + grid.width()  * grid.resolution());
  std::uniform_real_distribution<double> uy(grid.originY(),
      grid.originY() + grid.height() * grid.resolution());
  std::uniform_real_distribution<double> ub(0.0, 1.0);

  std::vector<TreeNode> tree;
  tree.push_back({{start.x, start.y}, -1, 0.0});

  int  goal_idx = -1;
  double best_goal_cost = std::numeric_limits<double>::infinity();

  for (int iter = 0; iter < cfg_.max_iters; ++iter) {
    // 1) Sample a target point (with goal bias).
    Point2D sample;
    if (ub(rng) < cfg_.goal_bias) {
      sample = {goal.x, goal.y};
    } else {
      sample = {ux(rng), uy(rng)};
    }

    // 2) Find nearest tree node and steer toward sample.
    const int near_idx = nearestIndex(tree, sample);
    const Point2D new_pt = steer(tree[near_idx].pt, sample, cfg_.step_size_m);
    if (!grid.isLineCollisionFree(tree[near_idx].pt, new_pt)) continue;

    // 3) Standard RRT: attach to nearest. RRT*: pick best parent in radius.
    int    best_parent = near_idx;
    double best_cost   = tree[near_idx].cost
                       + tree[near_idx].pt.distanceTo(new_pt);

    std::vector<int> neighbors;
    if (star_rewire_) {
      for (size_t i = 0; i < tree.size(); ++i) {
        if (tree[i].pt.distanceTo(new_pt) <= cfg_.rewire_radius_m
         && grid.isLineCollisionFree(tree[i].pt, new_pt)) {
          neighbors.push_back(static_cast<int>(i));
          const double c = tree[i].cost + tree[i].pt.distanceTo(new_pt);
          if (c < best_cost) { best_cost = c; best_parent = static_cast<int>(i); }
        }
      }
    }

    TreeNode new_node{new_pt, best_parent, best_cost};
    const int new_idx = static_cast<int>(tree.size());
    tree.push_back(new_node);
    ++stats_.nodes_explored;

    // 4) RRT*: rewire neighbors that would benefit from going through new_node.
    if (star_rewire_) {
      for (int n : neighbors) {
        if (n == best_parent) continue;
        const double thru = new_node.cost + new_node.pt.distanceTo(tree[n].pt);
        if (thru < tree[n].cost
         && grid.isLineCollisionFree(new_node.pt, tree[n].pt)) {
          tree[n].parent = new_idx;
          tree[n].cost   = thru;
        }
      }
    }

    // 5) Goal check.
    if (new_pt.distanceTo({goal.x, goal.y}) <= cfg_.goal_radius_m) {
      const double goal_cost =
          new_node.cost + new_pt.distanceTo({goal.x, goal.y});
      if (goal_cost < best_goal_cost) {
        best_goal_cost = goal_cost;
        goal_idx       = new_idx;
        if (!star_rewire_) break;     // plain RRT stops at first solution
      }
    }
  }

  if (goal_idx < 0) {
    stats_.plan_time_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - t0).count();
    return std::nullopt;
  }

  // Reconstruct path: goal_idx -> ... -> root, then add the actual goal.
  Path path;
  int cur = goal_idx;
  while (cur != -1) {
    path.push_back({tree[cur].pt.x, tree[cur].pt.y, 0.0});
    cur = tree[cur].parent;
  }
  std::reverse(path.begin(), path.end());
  if (!path.empty()) path.front().theta = start.theta;
  path.push_back({goal.x, goal.y, goal.theta});

  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    len += path[i].point().distanceTo(path[i - 1].point());
  stats_.path_length_m = len;
  stats_.plan_time_ms  = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - t0).count();
  return path;
}

}  // namespace motion_planner
