import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('g1_footstep_planner')
    param_file = os.path.join(pkg_dir, 'config', 'planner_params.yaml')

    footstep_planner_action_node = Node(
        package='g1_footstep_planner',
        executable='footstep_planner_action_node',
        name='g1_footstep_planner_node',
        output='screen',
        parameters=[param_file]
    )

    return LaunchDescription([
        footstep_planner_action_node
    ])
