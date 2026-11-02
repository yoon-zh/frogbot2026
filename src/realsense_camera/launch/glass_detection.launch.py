import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("realsense_camera")
    default_config = os.path.join(pkg_share, "config", "glass_detection.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to glass detection parameter YAML",
    )
    launch_driver_arg = DeclareLaunchArgument(
        "launch_driver",
        default_value="false",
        description="Launch the native pyrealsense2 camera driver node",
    )

    glass_node = Node(
        package="realsense_camera",
        executable="glass_detection_node",
        name="glass_detection_node",
        output="screen",
        parameters=[LaunchConfiguration("config_file")],
    )

    realsense_driver = Node(
        package="realsense_camera",
        executable="realsense_driver_node",
        name="realsense_driver_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_driver")),
    )

    return LaunchDescription(
        [
            config_file_arg,
            launch_driver_arg,
            glass_node,
            realsense_driver,
        ]
    )

