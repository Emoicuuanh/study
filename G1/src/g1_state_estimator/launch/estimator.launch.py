import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get config directory
    pkg_share = get_package_share_directory('g1_state_estimator')
    config_file = os.path.join(pkg_share, 'config', 'estimator.yaml')
    loop_config_file = os.path.join(pkg_share, 'config', 'gtsam_loop_closure.yaml')

    # State Estimator Node (Local EKF)
    estimator_node = Node(
        package='g1_state_estimator',
        executable='state_estimator_node',
        name='g1_state_estimator',
        output='screen',
        parameters=[config_file]
    )

    # GTSAM Loop Closure Node (Global Backend)
    loop_closure_node = Node(
        package='g1_state_estimator',
        executable='gtsam_loop_closure_node',
        name='gtsam_loop_closure_node',
        output='screen',
        parameters=[loop_config_file]
    )

    return LaunchDescription([
        estimator_node,
        loop_closure_node
    ])

