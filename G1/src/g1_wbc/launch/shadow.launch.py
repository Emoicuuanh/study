import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('g1_wbc')
    urdf = os.path.join(get_package_share_directory('g1_description'), 'urdf', 'g1_29dof.urdf')
    pay = os.path.join(get_package_share_directory('g1_leg_odometry'), 'config', 'payload.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value='wlp0s20f3'),
        DeclareLaunchArgument('csv_path', default_value=''),
        DeclareLaunchArgument('urdf_path', default_value=urdf),
        DeclareLaunchArgument('payload_path', default_value=pay),
        Node(package='g1_wbc', executable='wbc_shadow_node', name='wbc_shadow_node',
             output='screen',
             parameters=[os.path.join(pkg, 'config', 'wbc.yaml'),
                         {'urdf_path': LaunchConfiguration('urdf_path'),
                          'payload_path': LaunchConfiguration('payload_path'),
                          'network_interface': LaunchConfiguration('network_interface'),
                          'csv_path': LaunchConfiguration('csv_path')}]),
    ])
