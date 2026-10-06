"""Chay leg_odometry_node + leg_odom_check_node de do tren ROBOT THAT.

Ca hai node deu CHI DOC - khong node nao gui lenh den robot.

    ros2 launch g1_leg_odometry check_on_robot.launch.py \
        network_interface:=eth0 csv_path:=/tmp/legodom.csv
"""
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
    iface = LaunchConfiguration('network_interface')

    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value=''),
        DeclareLaunchArgument('urdf_path', default_value=default_urdf),
        DeclareLaunchArgument('csv_path', default_value=''),
        DeclareLaunchArgument('sdk_ref', default_value='true',
                              description='doi chieu voi rt/odommodestate qua DDS'),
        DeclareLaunchArgument('enable_sdk', default_value='true',
                              description='false = khong doc robot, chi kiem tra duong ong'),
        Node(
            package='g1_leg_odometry', executable='leg_odometry_node',
            name='leg_odometry_node', output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=[os.path.join(pkg, 'config', 'leg_odometry.yaml'),
                        {'urdf_path': LaunchConfiguration('urdf_path'),
                         'network_interface': iface,
                         'enable_sdk': LaunchConfiguration('enable_sdk')}],
        ),
        Node(
            package='g1_leg_odometry', executable='leg_odom_check_node',
            name='leg_odom_check_node', output='screen',
            parameters=[{'network_interface': iface,
                         'sdk_ref': LaunchConfiguration('sdk_ref'),
                         'csv_path': LaunchConfiguration('csv_path')}],
        ),
    ])
