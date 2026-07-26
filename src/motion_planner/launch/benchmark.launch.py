# benchmark.launch.py
# Runs the planner benchmark in isolation (no Gazebo). Prints a comparison
# table of all 5 planners on the same start/goal pair.
#
# Usage: ros2 launch motion_planner benchmark.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="motion_planner",
            executable="benchmark_node",
            name="planner_benchmark",
            output="screen",
        ),
    ])
