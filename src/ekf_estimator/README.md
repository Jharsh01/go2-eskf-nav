# ekf_estimator

5-state Extended Kalman Filter for differential-drive ground robots, fusing
IMU, wheel odometry, and LiDAR scan-matching. Built for ROS 2 Jazzy +
Gazebo Harmonic.

State: `x = [x, y, theta, v, omega]^T`

## Build

```bash
cd ~/ekf_ws
colcon build --packages-select ekf_estimator --symlink-install
source install/setup.bash
```

## Run in simulation

```bash
ros2 launch ekf_estimator ekf_sim.launch.py
```

In another terminal, drive the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Topics

| Direction | Topic | Type |
|-----------|-------|------|
| in | `/imu` | `sensor_msgs/Imu` |
| in | `/odom` | `nav_msgs/Odometry` |
| in | `/scan` | `sensor_msgs/LaserScan` |
| out | `/ekf/pose` | `geometry_msgs/PoseWithCovarianceStamped` |
| tf | `odom -> base_link` | broadcast at `predict_rate_hz` |

## Tests

```bash
colcon test --packages-select ekf_estimator
colcon test-result --verbose
```

- 17 unit tests (`gtest`) on the EKF math: motion model, Jacobians, all three
  measurement updates, covariance positive-definiteness, angle wrapping, reset.
- 4 integration tests (`launch_testing`) that spin up the node and validate
  behaviour for stationary / rotation-only / straight-line / sensor-dropout
  cases.

## What's left to wire up

1. **Real scan matching.** `EkfNode::scanCallback` is a placeholder. Drop in
   PCL ICP or NDT between `last_scan_` and the new scan, compose with the
   previous estimate, and call `updatePose`.
2. **Validation node.** Subscribe to `/ground_truth/pose` and `/ekf/pose`,
   compute ATE/RPE, log to CSV, plot with matplotlib.
3. **Tuning.** Defaults in `config/ekf_params.yaml` are starting points —
   adjust `q_*` and `*_var` against your specific sensors and Gazebo world.
