// prm.cpp
#include "motion_planner/planners/prm.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <random>
#include <vector>

namespace motion_planner {

namespace {

struct Edge { int to; double cost; };

// k smallest distances using partial_sort. Linear search is fine for the
// sample sizes typical in PRM demos (a few hundred to a few thousand nodes).
std::vector<int> kNearest(const std::vector<Point2D>& nodes,
                          const Point2D& q, int k, int self_idx = -1) {
  std::vector<std::pair<double, int>> dists;
  dists.reserve(nodes.size());
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (static_cast<int>(i) == self_idx) continue;
    dists.emplace_back(q.distanceTo(nodes[i]), static_cast<int>(i));
  }
  k = std::min<int>(k, dists.size());
  std::partial_sort(dists.begin(), dists.begin() + k, dists.end());
  std::vector<int> out;
  out.reserve(k);
  for (int i = 0; i < k; ++i) out.push_back(dists[i].second);
  return out;
}

// Dijkstra over an adjacency list. Returns parents and total cost to goal.
std::optional<std::vector<int>> dijkstraGraph(
    const std::vector<std::vector<Edge>>& adj,
    int start, int goal) {
  const int N = adj.size();
  std::vector<double> dist(N, std::numeric_limits<double>::infinity());
  std::vector<int>    parent(N, -1);
  using QItem = std::pair<double, int>;
  std::priority_queue<QItem, std::vector<QItem>, std::greater<>> pq;
  dist[start] = 0.0;
  pq.emplace(0.0, start);
  while (!pq.empty()) {
    auto [d, u] = pq.top(); pq.pop();
    if (u == goal) break;
    if (d > dist[u]) continue;
    for (const Edge& e : adj[u]) {
      const double nd = d + e.cost;
      if (nd < dist[e.to]) {
        dist[e.to]   = nd;
        parent[e.to] = u;
        pq.emplace(nd, e.to);
      }
    }
  }
  if (parent[goal] == -1 && start != goal) return std::nullopt;
  return parent;
}

}  // namespace

std::optional<Path> PrmPlanner::plan(const Pose2D& start,
                                     const Pose2D& goal,
                                     const OccupancyGrid& grid) {
  const auto t0 = std::chrono::steady_clock::now();
  stats_ = {};

  if (!grid.isFreeWorld(start.x, start.y)) return std::nullopt;
  if (!grid.isFreeWorld(goal.x,  goal.y))  return std::nullopt;

  // ---- 1) Sample collision-free nodes -----------------------------------
  std::mt19937 rng(cfg_.seed);
  std::uniform_real_distribution<double> ux(grid.originX(),
      grid.originX() + grid.width()  * grid.resolution());
  std::uniform_real_distribution<double> uy(grid.originY(),
      grid.originY() + grid.height() * grid.resolution());

  std::vector<Point2D> nodes;
  nodes.reserve(cfg_.num_samples + 2);
  nodes.push_back({start.x, start.y});
  nodes.push_back({goal.x,  goal.y});
  while (static_cast<int>(nodes.size()) < cfg_.num_samples + 2) {
    Point2D s{ux(rng), uy(rng)};
    if (grid.isFreeWorld(s.x, s.y)) nodes.push_back(s);
  }

  // ---- 2) Build adjacency by k-NN with collision-free edges -------------
  std::vector<std::vector<Edge>> adj(nodes.size());
  for (size_t i = 0; i < nodes.size(); ++i) {
    auto neighbors = kNearest(nodes, nodes[i], cfg_.k_neighbors,
                              static_cast<int>(i));
    for (int n : neighbors) {
      const double d = nodes[i].distanceTo(nodes[n]);
      if (d > cfg_.max_edge_length_m) continue;
      if (!grid.isLineCollisionFree(nodes[i], nodes[n])) continue;
      adj[i].push_back({n, d});
    }
  }
  stats_.nodes_explored = static_cast<int>(nodes.size());

  // ---- 3) Query: Dijkstra from start (idx 0) to goal (idx 1) ------------
  auto parents = dijkstraGraph(adj, 0, 1);
  if (!parents) {
    stats_.plan_time_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - t0).count();
    return std::nullopt;
  }

  // Reconstruct path.
  Path path;
  int cur = 1;
  while (cur != -1) {
    path.push_back({nodes[cur].x, nodes[cur].y, 0.0});
    cur = (*parents)[cur];
  }
  std::reverse(path.begin(), path.end());
  if (!path.empty()) {
    path.front().theta = start.theta;
    path.back().theta  = goal.theta;
  }

  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
    len += path[i].point().distanceTo(path[i - 1].point());
  stats_.path_length_m = len;
  stats_.plan_time_ms  = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - t0).count();
  return path;
}

}  // namespace motion_planner
