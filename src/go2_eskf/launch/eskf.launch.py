import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("go2_eskf")
    default_params = os.path.join(pkg, "config", "eskf_params.yaml")

    declare_params = DeclareLaunchArgument(
        "params_file", default_value=default_params,
        description="Path to the ESKF parameter YAML",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true",
        description="Use the Gazebo /clock",
    )
    declare_log_path = DeclareLaunchArgument(
        "log_path", default_value="",
        description="CSV path to log estimate vs ground truth (empty = off)",
    )
    declare_use_gps = DeclareLaunchArgument(
        "use_gps", default_value="false",
        description="Fuse /gps/fix for absolute position. OFF by default: the "
                    "Gazebo navsat sensor emits ~0.5 DEGREES (~55 km) of position "
                    "noise (its <stddev>0.5</stddev> is applied in degrees, not "
                    "metres), so fusing it destroys the estimate. Enable only with "
                    "a real/fixed GPS. Without it, position relies on leg odometry "
                    "(bounded drift, no global anchor).",
    )

    eskf_node = Node(
        package="go2_eskf",
        executable="go2_eskf_node",
        name="eskf_node",
        output="screen",
        parameters=[
            LaunchConfiguration("params_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"log_path": LaunchConfiguration("log_path")},
            # Typed override so use_gps:=true/false on the CLI actually reaches
            # the bool parameter (a raw substitution would arrive as a string).
            {"use_gps": ParameterValue(
                LaunchConfiguration("use_gps"), value_type=bool)},
        ],
    )

    return LaunchDescription(
        [declare_params, declare_use_sim_time, declare_log_path,
         declare_use_gps, eskf_node]
    )
