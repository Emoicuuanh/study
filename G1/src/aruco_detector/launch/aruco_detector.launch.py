import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("aruco_detector")
    default_config_path = os.path.join(pkg_share, "config", "aruco_params.yaml")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config_path,
        description="Path to YAML configuration file for aruco_detector."
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="/camera/color/image_raw",
        description="Camera image topic."
    )

    camera_info_topic_arg = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/color/camera_info",
        description="Camera info topic."
    )

    marker_size_arg = DeclareLaunchArgument(
        "marker_size",
        default_value="0.1",
        description="Size of ArUco marker side in meters."
    )

    show_image_arg = DeclareLaunchArgument(
        "show_image",
        default_value="false",
        description="Show live OpenCV GUI preview window."
    )

    aruco_detector_node = Node(
        package="aruco_detector",
        executable="aruco_detector_node.py",
        name="aruco_detector",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "marker_size": LaunchConfiguration("marker_size"),
                "show_image": LaunchConfiguration("show_image"),
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        image_topic_arg,
        camera_info_topic_arg,
        marker_size_arg,
        show_image_arg,
        aruco_detector_node,
    ])
