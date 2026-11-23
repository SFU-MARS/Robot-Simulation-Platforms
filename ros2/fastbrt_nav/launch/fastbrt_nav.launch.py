#!/usr/bin/env python3
"""
Gazebo world + TurtleBot3 and the fastbrt_nav node.  The spawn pose is passed to
both, since odometry starts at the spawn pose.  ``gazebo``: "classic" (Gazebo
Classic, ROS 2 Humble / Ubuntu 22.04: turtlebot3_controls/hj_launch_world_classic
.launch.py) or "gz" (the new Gazebo, ROS 2 Jazzy: hj_launch_world.launch.py);
default: classic on Humble, gz otherwise.

    ros2 launch fastbrt_nav fastbrt_nav.launch.py                       # hj_reach_target, defaults below
    ros2 launch fastbrt_nav fastbrt_nav.launch.py filter:=hj log_path:=/tmp/hj.csv   # hj, corners on (thread)
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
        "v0": a("v0"),
        "corners": a("corners"),
        "corner_mode": a("corner_mode"),
        "margin": float(a("margin")),
        "map_origin": floats(a("map_origin")),
        "map_size": floats(a("map_size")),
        "log_path": a("log_path"),
        "use_sim_time": a("use_sim_time").lower() == "true",
    }
    return [Node(package="fastbrt_nav", executable="nav_node", name="fastbrt_nav", output="screen",
                 parameters=[params])]


def world_launch(context):
    a = lambda name: LaunchConfiguration(name).perform(context)
    gazebo = a("gazebo").lower()
    if gazebo not in ("classic", "gz"):
        raise ValueError(f"gazebo must be classic or gz, got {gazebo!r}")
    name = "hj_launch_world_classic.launch.py" if gazebo == "classic" else "hj_launch_world.launch.py"
    path = os.path.join(get_package_share_directory("turtlebot3_controls"), "launch", name)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={k: LaunchConfiguration(k) for k in ("world", "x_pose", "y_pose", "yaw_pose",
                                                              "use_sim_time")}.items())]


def generate_launch_description():
    controls_share = get_package_share_directory("turtlebot3_controls")
    args = [
        ("gazebo", "classic" if os.environ.get("ROS_DISTRO", "") == "humble" else "gz"),
        ("world", os.path.join(controls_share, "worlds", "hj_reach_target.world")),
        ("x_pose", "-0.8"), ("y_pose", "-0.8"), ("yaw_pose", "0.2"),
        ("robot", "burger"), ("goal", "[0.7, 0.2]"),
        ("nominal", "mppi"), ("filter", "backup"), ("unknown", "occupied"), ("primitives", "faces"), ("v0", ""),
        ("corners", "auto"), ("corner_mode", "thread"), ("margin", "-1.0"),
        ("map_origin", "[0.0, 0.0]"), ("map_size", "[0.0, 0.0]"),       # [0, 0]: the profile's map
        ("log_path", ""), ("use_sim_time", "true"),
    ]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args]
                             + [OpaqueFunction(function=world_launch), OpaqueFunction(function=nav_node)])
