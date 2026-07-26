# benchmark.launch.py
# Phase 4 — launches eskf_node configured for one benchmark scenario, logging
# estimate-vs-ground-truth to CSV for offline analysis with scripts/metrics.py
# and scripts/run_benchmark.py.
#
# Run alongside the Go2 sim (ros2 launch unitree_go2_sim unitree_go2_launch.py),
# once per scenario, then analyse:
#
#   ros2 launch go2_eskf benchmark.launch.py scenario:=fixed     log_path:=/tmp/fixed.csv
#   ros2 launch go2_eskf benchmark.launch.py scenario:=adaptive  log_path:=/tmp/adaptive.csv \
#        slip_model_path:=<ws>/install/go2_eskf/share/go2_eskf/config/slip_model.txt
#   ros2 launch go2_eskf benchmark.launch.py scenario:=gps_denied log_path:=/tmp/gps_denied.csv
#
#   python3 src/go2_eskf/scripts/run_benchmark.py \
#        --log fixed=/tmp/fixed.csv --log adaptive=/tmp/adaptive.csv \
#        --log gps_denied=/tmp/gps_denied.csv --out docs/benchmark
#
# scenario presets:
#   fixed      -> use_slip_model:=false, use_gps:=true
#   adaptive   -> use_slip_model:=true,  use_gps:=true  (needs slip_model_path)
#   gps_denied -> use_slip_model:=true,  use_gps:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory("go2_eskf")
    params_file = LaunchConfiguration("params_file").perform(context)
    scenario = LaunchConfiguration("scenario").perform(context)
    log_path = LaunchConfiguration("log_path").perform(context)
    slip_model_path = LaunchConfiguration("slip_model_path").perform(context)
    ground_truth_topic = LaunchConfiguration("ground_truth_topic").perform(context)

    presets = {
        "fixed": {"use_slip_model": False, "use_gps": True},
        "adaptive": {"use_slip_model": True, "use_gps": True},
        "gps_denied": {"use_slip_model": True, "use_gps": False},
    }
    if scenario not in presets:
        raise RuntimeError(f"unknown scenario '{scenario}'; "
                           f"choose one of {list(presets)}")

    overrides = dict(presets[scenario])
    overrides["slip_model_path"] = slip_model_path
    overrides["ground_truth_topic"] = ground_truth_topic
    overrides["log_path"] = log_path
    overrides["use_sim_time"] = True

    return [Node(
        package="go2_eskf",
        executable="go2_eskf_node",
        name="eskf_node",
        output="screen",
        parameters=[params_file, overrides],
    )]


def generate_launch_description():
    pkg = get_package_share_directory("go2_eskf")
    default_params = os.path.join(pkg, "config", "eskf_params.yaml")
    default_model = os.path.join(pkg, "config", "slip_model.txt")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("scenario", default_value="fixed",
                              description="fixed | adaptive | gps_denied"),
        DeclareLaunchArgument("log_path", default_value="/tmp/eskf_benchmark.csv"),
        DeclareLaunchArgument("slip_model_path", default_value=default_model),
        DeclareLaunchArgument("ground_truth_topic", default_value="/ground_truth/odom",
                              description="nav_msgs/Odometry ground-truth topic from the sim"),
        OpaqueFunction(function=_setup),
    ])
