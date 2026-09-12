from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    headless_driver = Node(
        package='control_hand',
        executable='init_driver',
        name='init_driver',
        output='screen'
    )

    return LaunchDescription([
        headless_driver,
    ])
