import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('g1_leg_odometry')
    default_urdf = os.path.join(
        get_package_share_directory('g1_description'), 'urdf', 'g1_29dof.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value=''),
        DeclareLaunchArgument('enable_sdk', default_value='true'),
        DeclareLaunchArgument('urdf_path', default_value=default_urdf),
        Node(
            package='g1_leg_odometry',
            executable='leg_odometry_node',
            name='leg_odometry_node',
            output='screen',
            parameters=[
                os.path.join(pkg, 'config', 'leg_odometry.yaml'),
                {
                    'urdf_path': LaunchConfiguration('urdf_path'),
                    'network_interface': LaunchConfiguration('network_interface'),
                    'enable_sdk': LaunchConfiguration('enable_sdk'),
                },
            ],
        ),
    ])
