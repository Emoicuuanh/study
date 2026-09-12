import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('sdk_bridge')
    default_config_file = os.path.join(pkg_share, 'config', 'sdk_bridge.yaml')

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to configuration file for sdk_bridge'
    )

    cmd_vel_to_sdk_node = Node(
        package='sdk_bridge',
        executable='cmd_vel_to_sdk_node',
        name='cmd_vel_to_sdk_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )

    return LaunchDescription([
        config_file_arg,
        cmd_vel_to_sdk_node
    ])
