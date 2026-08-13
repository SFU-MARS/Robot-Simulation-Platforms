# Launch file to see the robot description

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Paths
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("qbot_driver"), "description", "robot.urdf.xacro"]
    )
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare("qbot_driver"), "params", "description.rviz"]
    )

    # Launch arguments
    urdf_path_arg = DeclareLaunchArgument(
        name="urdf_path", default_value=urdf_path, description="URDF path"
    )
    pub_joints_arg = DeclareLaunchArgument(
        name="publish_joints",
        default_value="true",
        description="Launch joint_states_publisher",
    )
    rviz_arg = DeclareLaunchArgument(
        name="rviz", default_value="true", description="Run rviz"
    )
    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        description="Use simulation time",
    )

    # Nodes
    joint_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": Command(
                    ["xacro ", LaunchConfiguration("urdf_path")]
                ),
            }
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription(
        [
            urdf_path_arg,
            pub_joints_arg,
            rviz_arg,
            sim_time_arg,
            joint_pub_node,
            state_pub_node,
            rviz_node,
        ]
    )
