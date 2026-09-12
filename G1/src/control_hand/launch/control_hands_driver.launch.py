from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('control_hand')

    headless_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'headless_driver_hands.launch.py')
        )
    )

    hand_node = Node(
        package='control_hand',
        executable='hand_node',
        name='hand_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return LaunchDescription([
        headless_launch,
        hand_node,
    ])
