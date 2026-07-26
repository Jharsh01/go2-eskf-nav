# nav_demo.launch.py
# One-shot launch: Gazebo Harmonic with the obstacle world, ros_gz bridge,
# the EKF estimator, the navigation node, and RViz.
#
# Usage:
#   ros2 launch motion_planner nav_demo.launch.py
#   ros2 launch motion_planner nav_demo.launch.py planner_type:=rrt_star

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share        = get_package_share_directory("motion_planner")
    ekf_share        = get_package_share_directory("ekf_estimator")
    world_path       = os.path.join(pkg_share, "worlds", "obstacle_course.sdf")
    planner_cfg      = os.path.join(pkg_share, "config", "planner_params.yaml")
    ekf_cfg          = os.path.join(ekf_share, "config", "ekf_params.yaml")
    rviz_cfg         = os.path.join(pkg_share, "rviz",   "nav_demo.rviz")

    use_sim_time   = LaunchConfiguration("use_sim_time",   default="true")
    planner_type   = LaunchConfiguration("planner_type",   default="astar")
    launch_rviz    = LaunchConfiguration("launch_rviz",    default="true")

    # Tell Gazebo where to find our custom model.
    set_model_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_share, "models")
              + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    )

    # ---- Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={"gz_args": [world_path, " -r"]}.items(),
    )

    # ---- ros_gz bridge: maps Gazebo topics to ROS 2.
    # Note: '[' = GZ -> ROS, ']' = ROS -> GZ, '@' = bidirectional.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/model/diff_drive_bot/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ---- EKF estimator
    ekf = Node(
        package="ekf_estimator",
        executable="ekf_estimator_node",
        name="ekf_estimator",
        parameters=[ekf_cfg, {"use_sim_time": use_sim_time,
                              "publish_tf": True}],   # EKF owns odom->base_link
        remappings=[("ground_truth/pose", "/model/diff_drive_bot/pose")],
        output="screen",
    )

    # ---- Motion planner / navigation
    nav = Node(
        package="motion_planner",
        executable="navigation_node",
        name="navigation_node",
        parameters=[planner_cfg,
                    {"use_sim_time": use_sim_time,
                     "planner_type": planner_type}],
        output="screen",
    )

    # ---- RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        condition=None if launch_rviz else None,
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time",   default_value="true"),
        DeclareLaunchArgument("planner_type",   default_value="astar",
            description="astar | dijkstra | rrt | rrt_star | prm"),
        DeclareLaunchArgument("launch_rviz",    default_value="true"),
        set_model_path,
        gazebo,
        bridge,
        ekf,
        nav,
        rviz,
    ])
