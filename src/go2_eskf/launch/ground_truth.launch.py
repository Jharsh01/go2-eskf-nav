# ground_truth.launch.py
# Phase 4 helper — provides the nav_msgs/Odometry ground truth the benchmark needs.
#
# The Go2 sim publishes no world-frame ground-truth Odometry on its own (its /odom
# is the robot_localization estimate, not truth; the gz pose-vector topics lose
# entity names through ros_gz_bridge so they can't be matched). Instead, a dedicated
# gz OdometryPublisher on the model (see unitree_go2_gazebo.xacro) emits the true
# base pose+twist on /model/go2/ground_truth/odometry; this launch just bridges that
# straight to /ground_truth/odom. Ground truth is relative to the spawn pose, which
# is what the estimator is compared against (both start at the origin).
#
# Run it alongside the sim (order-independent), then point the benchmark at it:
#
#   ros2 launch unitree_go2_sim unitree_go2_launch.py
#   ros2 launch go2_eskf ground_truth.launch.py
#   ros2 launch go2_eskf benchmark.launch.py scenario:=adaptive \
#        log_path:=/tmp/adaptive.csv ground_truth_topic:=/ground_truth/odom
#
# Sanity check:  ros2 topic echo /ground_truth/odom --once

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_odom_topic = LaunchConfiguration("gz_odom_topic")
    output_topic = LaunchConfiguration("output_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ground_truth_odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            # gz -> ros only ("[").  gz.msgs.Odometry maps to nav_msgs/Odometry
            # with pose, twist and frame ids populated by the OdometryPublisher.
            [gz_odom_topic, "@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        ],
        remappings=[(gz_odom_topic, output_topic)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "gz_odom_topic", default_value="/model/go2/ground_truth/odometry",
            description="gz Odometry topic from the OdometryPublisher plugin"),
        DeclareLaunchArgument("output_topic", default_value="/ground_truth/odom"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        bridge,
    ])
