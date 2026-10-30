#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    gazebo_share = get_package_share_directory('turtlebot3_gazebo')
    controls_share = get_package_share_directory('turtlebot3_controls')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    turtlebot3_model = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    world = LaunchConfiguration('world')

    gz_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
    gazebo_launch_dir = os.path.join(gazebo_share, 'launch')
    robot_model = 'turtlebot3_' + turtlebot3_model
    robot_sdf = os.path.join(gazebo_share, 'models', robot_model, 'model.sdf')
    bridge_params = os.path.join(
        controls_share, 'params', f'{robot_model}_bridge.yaml'
    )

    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(gazebo_share, 'models'),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(controls_share, 'worlds', 'hj_reach_target.world'),
            description='Path to the Gazebo world SDF file',
        ),
        DeclareLaunchArgument('x_pose', default_value='-0.8'),
        DeclareLaunchArgument('y_pose', default_value='-0.8'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        set_model_path,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world],
                'on_exit_shutdown': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': '-g -v2 '}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', turtlebot3_model,
                '-file', robot_sdf,
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01',
            ],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
            output='screen',
        ),
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=['/camera/image_raw'],
            condition=IfCondition(
                PythonExpression(["'", turtlebot3_model, "' != 'burger'"])
            ),
            output='screen',
        ),
    ])