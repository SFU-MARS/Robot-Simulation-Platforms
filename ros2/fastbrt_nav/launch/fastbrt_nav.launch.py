#!/usr/bin/env python3
"""
Gazebo world + TurtleBot3 (turtlebot3_controls/hj_launch_world.launch.py) and
the fastbrt_nav node.  The spawn pose is passed to both, since odometry starts
at the spawn pose.

    ros2 launch fastbrt_nav fastbrt_nav.launch.py                       # hj_reach_target, defaults below
    ros2 launch fastbrt_nav fastbrt_nav.launch.py world:=<share>/worlds/occluded_4x4.world \
        x_pose:=-0.9 y_pose:=-1.4 yaw_pose:=0.0 goal:="[0.9, -0.3]" \
        map_origin:="[-2.1, -2.1]" map_size:="[4.2, 4.2]"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def floats(text):
    return [float(v) for v in text.strip().strip("[]").split(",") if v.strip()]


def nav_node(context):
    a = lambda name: LaunchConfiguration(name).perform(context)
    params = {
        "robot": a("robot"),
        "goal": floats(a("goal")),
        "spawn": [float(a("x_pose")), float(a("y_pose")), float(a("yaw_pose"))],
        "nominal": a("nominal"),
        "filter": a("filter"),
        "unknown": a("unknown"),
        "primitives": a("primitives"),
        "map_origin": floats(a("map_origin")),
        "map_size": floats(a("map_size")),
        "log_path": a("log_path"),
        "use_sim_time": a("use_sim_time").lower() == "true",
    }
    return [Node(package="fastbrt_nav", executable="nav_node", name="fastbrt_nav", output="screen",
                 parameters=[params])]


def generate_launch_description():
    controls_share = get_package_share_directory("turtlebot3_controls")
    args = [
        ("world", os.path.join(controls_share, "worlds", "hj_reach_target.world")),
        ("x_pose", "-0.8"), ("y_pose", "-0.8"), ("yaw_pose", "0.2"),
        ("robot", "burger"), ("goal", "[0.7, 0.2]"),
        ("nominal", "mppi"), ("filter", "gatekeeper"), ("unknown", "occupied"), ("primitives", "faces"),
        ("map_origin", "[0.0, 0.0]"), ("map_size", "[0.0, 0.0]"),       # [0, 0]: the profile's map
        ("log_path", ""), ("use_sim_time", "true"),
    ]
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(controls_share, "launch", "hj_launch_world.launch.py")),
        launch_arguments={k: LaunchConfiguration(k) for k in ("world", "x_pose", "y_pose", "yaw_pose",
                                                              "use_sim_time")}.items())
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args]
                             + [world, OpaqueFunction(function=nav_node)])
