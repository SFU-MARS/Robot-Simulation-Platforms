#!/usr/bin/env python3
"""
Gazebo Classic (ROS 2 Humble, Ubuntu 22.04) version of hj_launch_world.launch.py: a world of this package and a
TurtleBot3 at (x_pose, y_pose, yaw_pose), with the gazebo_ros plugins of the turtlebot3_gazebo models (humble
branch): /scan, /odom (and the odom -> base_footprint tf), /cmd_vel (geometry_msgs/Twist), /imu, /joint_states.

The worlds are written for the new Gazebo (gz sim) and name its world plugins (gz-sim-physics-system, ...), which
Gazebo Classic cannot load: they are left out of a copy of the world in the temporary folder. Everything else in
the worlds (boxes, ground, light, ODE physics) is plain SDF 1.6.

Needs ros-humble-gazebo-ros-pkgs and ros-humble-turtlebot3-gazebo (apt), and TURTLEBOT3_MODEL (e.g. burger).

    ros2 launch turtlebot3_controls hj_launch_world_classic.launch.py x_pose:=-0.8 y_pose:=-0.8 yaw_pose:=0.2
"""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GZ_SIM_PLUGIN = re.compile(r'\s*<plugin\s+filename="gz-sim-[^"]*"[^>]*/>\s*\n')


def classic_world(path: str) -> str:
    """A copy of the world without the plugins of the new Gazebo, in the temporary folder."""
    with open(path) as f:
        text = GZ_SIM_PLUGIN.sub("\n", f.read())
    out = os.path.join(tempfile.gettempdir(), "classic_" + os.path.basename(path))
    with open(out, "w") as f:
        f.write(text)
    return out


def launch_setup(context):
    a = lambda name: LaunchConfiguration(name).perform(context)
    gazebo_ros = get_package_share_directory("gazebo_ros")
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    model = "turtlebot3_" + os.environ["TURTLEBOT3_MODEL"]
    use_sim_time = a("use_sim_time")
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": classic_world(a("world"))}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tb3_gazebo, "launch", "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
        Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
             arguments=["-entity", os.environ["TURTLEBOT3_MODEL"],
                        "-file", os.path.join(tb3_gazebo, "models", model, "model.sdf"),
                        "-x", a("x_pose"), "-y", a("y_pose"), "-z", "0.01", "-Y", a("yaw_pose")]),
    ]
    if a("gui").lower() == "true":
        actions.insert(1, IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzclient.launch.py"))))
    return actions


def generate_launch_description():
    controls_share = get_package_share_directory("turtlebot3_controls")
    args = [
        ("world", os.path.join(controls_share, "worlds", "hj_reach_target.world")),
        ("x_pose", "-0.8"), ("y_pose", "-0.8"), ("yaw_pose", "0.0"),
        ("use_sim_time", "true"), ("gui", "true"),
    ]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args]
                             + [OpaqueFunction(function=launch_setup)])
