# Robotics Workspace — EKF + Motion Planning Demo

ROS 2 Jazzy + Gazebo Harmonic workspace that demonstrates a complete
localization-and-navigation pipeline on a differential-drive robot:

```
                   +-------------+
                   |   Gazebo    |  publishes /imu, /odom, /scan
                   +------+------+
                          |
                ros_gz_bridge (bidirectional)
                          |
         +---------+      v       +----------------+
  /imu   |         |              |                |  /planned_path
  /odom  |   EKF   | --/ekf/pose->|  Navigation    |--->  RViz
  /scan  | (Jazzy) |              |  (5 planners)  |
         +---------+              +-------+--------+
                                          |
                                       /cmd_vel
                                          v
                                       Gazebo
```

## Packages

| Package | Role |
|---------|------|
| `ekf_estimator`  | 5-state EKF fusing IMU + wheel odom + LiDAR scan-matching |
| `motion_planner` | A*, Dijkstra, RRT, RRT*, PRM + Pure Pursuit follower      |
| `go2_eskf`       | 8-state error-state EKF for the Unitree Go2 quadruped (IMU + leg odometry + GPS, slip-adaptive `R`) |

The Go2 estimator's full derivation — the strapdown physics, the `F` and `H` Jacobians,
`Q`/`R`, the Joseph update, and an annotated flow chart — is in
[`src/go2_eskf/README.md`](src/go2_eskf/README.md#how-the-estimator-works--physics-flow-and-every-equation).

## Build

```bash
cd ~/robotics_ws        # or wherever you unzipped this
rosdep install --from-paths src --ignore-src -r -y    # optional sanity check
colcon build --symlink-install
source install/setup.bash
```

Required system packages (Ubuntu 24.04 + ROS 2 Jazzy):
```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-tf2-geometry-msgs \
                 ros-jazzy-rviz2 libeigen3-dev
```

## Run the full demo

```bash
ros2 launch motion_planner nav_demo.launch.py
```

This brings up Gazebo with the obstacle course, the EKF, the navigation node,
and RViz. To send a goal:

- **From RViz**: click the `2D Goal Pose` tool, then click anywhere on the map.
- **From the CLI**:
  ```bash
  ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: 'odom'}, pose: {position: {x: 7.0, y: 7.0}, \
      orientation: {w: 1.0}}}"
  ```

## Switch planners at launch

```bash
ros2 launch motion_planner nav_demo.launch.py planner_type:=astar
ros2 launch motion_planner nav_demo.launch.py planner_type:=dijkstra
ros2 launch motion_planner nav_demo.launch.py planner_type:=rrt
ros2 launch motion_planner nav_demo.launch.py planner_type:=rrt_star
ros2 launch motion_planner nav_demo.launch.py planner_type:=prm
```

You can also change the planner at runtime:
```bash
ros2 param set /navigation_node planner_type rrt_star
```
The next goal you publish will use it.

## Benchmark all five planners

Headless comparison on the same start/goal — useful for project reports:
```bash
ros2 launch motion_planner benchmark.launch.py
```

Sample output:
```
===== Motion Planner Benchmark =====
Planner     Time (ms)     Length (m)      Nodes       Status
------------------------------------------------------------
astar       12.34         22.85           1854        OK
dijkstra    18.91         22.85           4217        OK
rrt         9.42          26.31           312         OK
rrt_star    142.07        23.94           5000        OK
prm         24.55         23.71           502         OK
```

## Tests

```bash
colcon test --packages-select ekf_estimator motion_planner
colcon test-result --verbose
```

- `ekf_estimator`: 17 unit tests + 4 integration tests.
- `motion_planner`: 13 unit tests covering grid, all five planners, follower.

## Topic map

| Topic | Type | Direction |
|-------|------|-----------|
| `/imu`           | `sensor_msgs/Imu`                          | Gazebo -> EKF      |
| `/odom`          | `nav_msgs/Odometry`                        | Gazebo -> EKF      |
| `/scan`          | `sensor_msgs/LaserScan`                    | Gazebo -> EKF/RViz |
| `/ekf/pose`      | `geometry_msgs/PoseWithCovarianceStamped`  | EKF -> nav         |
| `/goal_pose`     | `geometry_msgs/PoseStamped`                | RViz -> nav        |
| `/planned_path`  | `nav_msgs/Path`                            | nav -> RViz        |
| `/map`           | `nav_msgs/OccupancyGrid`                   | nav -> RViz        |
| `/cmd_vel`       | `geometry_msgs/Twist`                      | nav -> Gazebo      |

## Algorithm summary

| Algorithm | Type            | Optimal? | Strengths              | Weaknesses          |
|-----------|-----------------|----------|------------------------|---------------------|
| A*        | Grid search     | Yes      | Fast, deterministic    | Resolution-limited  |
| Dijkstra  | Grid search     | Yes      | No heuristic needed    | Explores everything |
| RRT       | Sampling        | No       | Fast in open space     | Jagged paths        |
| RRT*      | Sampling        | Asymp.   | Improves over time     | Slower than RRT     |
| PRM       | Roadmap         | Yes*     | Good for multi-query   | Costly to build     |

(*PRM is optimal over the roadmap, not the continuous space.)

## Tuning notes

- `motion_planner/config/planner_params.yaml` — robot radius, follower gains, planner.
- `ekf_estimator/config/ekf_params.yaml` — process noise Q, sensor noise R.
- Obstacle layout lives in two places: `worlds/obstacle_course.sdf` for Gazebo
  and `NavigationNode::buildDemoMap()` for the planner. Keep them in sync if
  you edit one.

## What to extend next

1. Replace the placeholder `EkfNode::scanCallback` with real PCL ICP scan matching.
2. Swap the hand-rolled occupancy grid for a SLAM-built map (`slam_toolbox`).
3. Add dynamic obstacles + replanning trigger when the path becomes infeasible.
4. Replace Pure Pursuit with the MPC controller from your existing project list
   for a tighter follower.
