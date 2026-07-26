#!/usr/bin/env bash
# quickstart.sh — one-shot build + launch for the demo.
# Usage: ./quickstart.sh [planner_type]
#   planner_type ∈ {astar, dijkstra, rrt, rrt_star, prm}, default: astar

set -e
PLANNER=${1:-astar}

cd "$(dirname "$0")"
echo "[quickstart] Building workspace…"
colcon build --symlink-install
echo "[quickstart] Sourcing install/setup.bash"
# shellcheck disable=SC1091
source install/setup.bash
echo "[quickstart] Launching nav_demo with planner_type=${PLANNER}"
ros2 launch motion_planner nav_demo.launch.py planner_type:="${PLANNER}"
