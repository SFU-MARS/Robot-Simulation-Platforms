import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Paths and files
    nav2_launch_file = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )
    map_file = PathJoinSubstitution(
        [FindPackageShare("hunavis"), "maps", "empty_room.yaml"]
    )
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("qbot_driver"), "params", "qbot_nav2.yaml"]
    )
    ekf_params_file = os.path.join(
        get_package_share_directory("qbot_driver"), "params", "ekf.yaml"
    )

    # Settings
    declare_sim = DeclareLaunchArgument(
        name="sim", default_value="false", description="Whether to use simulation time"
    )

    declare_map = DeclareLaunchArgument(
        name="map", default_value=map_file, description="Map"
    )

    # Launch files
    qbot_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("qbot_driver"),
                    "launch",
                    "qbot_platform_launch.py",
                )
            ]
        )
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("sim"),
            "params_file": nav2_params_file,
        }.items(),
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file],
    )

    """Generate launch description with multiple components."""
    depth_to_ptcloud_container = ComposableNodeContainer(
        name="obstacle_detection",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="rtabmap_util",
                plugin="rtabmap_util::PointCloudXYZ",
                name="points_xyz_rt",
                remappings=[
                    ("depth/image", "/camera/camera/depth/image_rect_raw"),
                    ("depth/camera_info", "/camera/camera/depth/camera_info"),
                    ("cloud", "/camera/camera/depth/color/points"),
                ],
                parameters=[
                    {"decimation": 4},
                    {"voxel_size": 0.05},
                    {"approx_sync": False},
                ],
            ),
            ComposableNode(
                package="rtabmap_util",
                plugin="rtabmap_util::ObstaclesDetection",
                name="obstacle_detection_rt",
                remappings=[("cloud", "/cropped_pointcloud")],
                parameters=[
                    {"frame_id": "camera_link"},
                    {"wait_for_transform": 1.0},
                    {"Grid/MaxGroundHeight": "0.04"},
                ],
            ),
        ],
        output="screen",
    )

    # Launch everything
    ld = LaunchDescription()
    ld.add_action(declare_sim)
    ld.add_action(declare_map)
    ld.add_action(qbot_platform_launch)
    ld.add_action(nav2_launch)
    ld.add_action(ekf_node)
    ld.add_action(depth_to_ptcloud_container)
    return ld
