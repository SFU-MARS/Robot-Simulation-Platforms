# This is the launch file that starts up the basic QBot Platform nodes

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode, Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution


def exit_qbot_platform_driver_interface_cb(context):
    # We're using a callback function and wrap it in OpaqueFunction() to be
    # passsed to the qbot_platform_driver_interface node's OnProcessExit
    # event handler, instead of using an ExecuteProcess() to stop the driver
    # model. This is because when an user uses Ctrl-C to end the launch file
    # it would cause the entire launch system to close, and during launch
    # system closing, no new ExecuteProcess() will be created, and thus cannot
    # use it to call "quarc_run -q".
    # However, OpaqueFunction callback would still work even during launch
    # system shutdown, and we can call the subprocess.call() to run the
    # "quarc_run -q" command to stop the model.

    # Use "run(capture_output=True)" so that if the rt-model somehow stopped
    # for whatever reason, we won't show the error message, because it doesn't
    # really matter at this point.
    subprocess.run(
        "quarc_run" + " -q -t tcpip://localhost:17000 qbot_platform_driver_physical",
        shell=True,
        capture_output=True,
    )


def generate_launch_description():
    driver_model_rt_executable = PathJoinSubstitution(
        [
            FindPackageShare("qbot_driver"),
            "rt_models",
            "qbot_platform_driver_physical.rt-linux_qbot_platform",
        ]
    )

    rt_model_start = ExecuteProcess(
        cmd=[
            "quarc_run",
            "-r -t tcpip://localhost:17000",
            driver_model_rt_executable,
            "-d %d -uri tcpip://%m:17001",
        ],
        name="QBotPlatformDriverModelStart",
        shell=True,
    )

    lidar_node = Node(package="qbot_driver", executable="lidar", name="Lidar")

    # realsense_camera_node = Node(
    #         package='qbot_driver',
    #         executable='rgbd',
    #         name='RealsenseCamera'
    #     )

    realsense_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"), "launch/rs_launch.py"
            )
        ),
        launch_arguments={
            "depth_module.profile": "640x480x30",
            # "pointcloud.enable": "True",
        }.items(),
    )

    downward_facing_camera_node = Node(
        package="qbot_driver", executable="csi", name="DownwardFacingCamera"
    )

    qbot_platform_driver_node = Node(
        package="qbot_driver",
        executable="qbot_platform_driver_interface",
        name="QBotPlatformDriver",
        parameters=[{"arm_robot": True}],
    )

    description_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            [
                get_package_share_directory("qbot_driver"),
                "/launch/description.launch.py",
            ]
        ),
    )

    return LaunchDescription(
        [
            rt_model_start,
            lidar_node,
            realsense_camera_node,
            downward_facing_camera_node,
            description_launch,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=qbot_platform_driver_node,
                    on_exit=[
                        OpaqueFunction(function=exit_qbot_platform_driver_interface_cb),
                        LogInfo(
                            msg="QBotPlatformDriver exiting, so stopping the driver model as well."
                        ),
                    ],
                )
            ),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=rt_model_start,
                    on_start=[
                        LogInfo(
                            msg="QBotPlatformDriverModelStart started. Wait for 2 sec, then start the qbot_platform_driver_interface node"
                        ),
                        TimerAction(period=2.0, actions=[qbot_platform_driver_node]),
                    ],
                )
            ),
        ]
    )
