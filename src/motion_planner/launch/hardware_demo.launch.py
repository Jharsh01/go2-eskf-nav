# hardware_demo.launch.py
# Hardware-mode counterpart to nav_demo.launch.py. Brings up:
#   - ekf_estimator   (with ekf_params_hardware.yaml — ground_truth off)
#   - navigation_node (with planner_params.yaml)
#   - RViz            (optional)
#
# Does NOT bring up Gazebo, the ros_gz bridge, or the diff-drive Gazebo
# plugin. You are expected to start the following nodes externally before
# launching this file:
#
#   1. IMU driver publishing sensor_msgs/Imu on  /imu
#        e.g. ros2 run mpu6050_driver mpu6050_node
#   2. LiDAR driver publishing sensor_msgs/LaserScan on  /scan
#        e.g. ros2 launch rplidar_ros rplidar_a1_launch.py
#   3. Wheel-encoder/diff-drive controller publishing nav_msgs/Odometry on /odom
#        and subscribing to /cmd_vel (geometry_msgs/Twist)
#        e.g. ros2 launch your_robot_bringup diff_drive.launch.py
#
# Usage:
#   ros2 launch motion_planner hardware_demo.launch.py
#   ros2 launch motion_planner hardware_demo.launch.py planner_type:=rrt_star

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mp_share  = get_package_share_directory("motion_planner")
    ekf_share = get_package_share_directory("ekf_estimator")
    planner_cfg = os.path.join(mp_share,  "config", "planner_params.yaml")
    ekf_cfg     = os.path.join(ekf_share, "config", "ekf_params_hardware.yaml")
    rviz_cfg    = os.path.join(mp_share,  "rviz",   "nav_demo.rviz")

    planner_type = LaunchConfiguration("planner_type", default="astar")
    launch_rviz  = LaunchConfiguration("launch_rviz",  default="true")

    # The EKF owns odom -> base_link TF on hardware (no Gazebo to provide it).
    ekf = Node(
        package="ekf_estimator",
        executable="ekf_estimator_node",
        name="ekf_estimator",
        parameters=[ekf_cfg, {"publish_tf": True}],
        output="screen",
    )

    nav = Node(
        package="motion_planner",
        executable="navigation_node",
        name="navigation_node",
        parameters=[planner_cfg, {"planner_type": planner_type}],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        condition=None,           # cosmetic: always on; set launch_rviz:=false to skip externally
    )

    return LaunchDescription([
        DeclareLaunchArgument("planner_type", default_value="astar",
            description="astar | dijkstra | rrt | rrt_star | prm"),
        DeclareLaunchArgument("launch_rviz",  default_value="true"),
        ekf,
        nav,
        rviz,
    ])
