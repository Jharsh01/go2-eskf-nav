# ekf_sim.launch.py
# Brings up Gazebo Harmonic + a diff-drive robot + ros_gz bridge + EKF node.
# Usage: ros2 launch ekf_estimator ekf_sim.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("ekf_estimator")
    config_file = os.path.join(pkg_share, "config", "ekf_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_arg    = LaunchConfiguration("world",        default="empty.sdf")

    # ---- Gazebo Harmonic
    ros_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={"gz_args": [world_arg, " -r"]}.items(),
    )

    # ---- ros_gz bridge: maps Gazebo topics to ROS 2 topics.
    # Adjust topic names to match your robot SDF.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/ground_truth/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ---- EKF estimator node
    ekf = Node(
        package="ekf_estimator",
        executable="ekf_estimator_node",
        name="ekf_estimator",
        parameters=[config_file, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ---- RViz2 with a sensible default config
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world",        default_value="empty.sdf"),
        ros_gz_sim,
        bridge,
        ekf,
        rviz,
    ])
