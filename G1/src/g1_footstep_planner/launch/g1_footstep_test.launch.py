import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sdk_bridge = get_package_share_directory('sdk_bridge')
    pkg_footstep_planner = get_package_share_directory('g1_footstep_planner')

    default_sdk_config = os.path.join(pkg_sdk_bridge, 'config', 'sdk_bridge.yaml')
    default_planner_config = os.path.join(pkg_footstep_planner, 'config', 'planner_params.yaml')

    # Launch Arguments for Standalone Testing
    network_interface_arg = DeclareLaunchArgument(
        'network_interface',
        default_value='wlp2s0',
        description='Network interface connected to Unitree G1 (e.g. wlp2s0, eth0)'
    )

    enable_sdk_arg = DeclareLaunchArgument(
        'enable_sdk',
        default_value='true',
        description='Enable real SDK communication with robot'
    )

    auto_stand_arg = DeclareLaunchArgument(
        'auto_stand_on_start',
        default_value='true',
        description='Automatically send BalanceStand on node start'
    )

    # Launch Configurations
    network_interface = LaunchConfiguration('network_interface')
    enable_sdk = LaunchConfiguration('enable_sdk')
    auto_stand = LaunchConfiguration('auto_stand_on_start')

    # 1. SDK Bridge Node (cmd_vel_to_sdk_node) -> Subscribes to /cmd_vel
    cmd_vel_to_sdk_node = Node(
        package='sdk_bridge',
        executable='cmd_vel_to_sdk_node',
        name='cmd_vel_to_sdk_node',
        output='screen',
        parameters=[
            default_sdk_config,
            {
                'network_interface': network_interface,
                'enable_sdk': PythonExpression(["'", enable_sdk, "'.lower() == 'true'"]),
                'auto_stand_on_start': PythonExpression(["'", auto_stand, "'.lower() == 'true'"]),
                'cmd_vel_topic': '/cmd_vel'
            }
        ]
    )

    # 2. G1 Twist Mux Node (Lightweight zero-dependency multiplexer from g1_navigation_nav2)
    twist_mux_node = Node(
        package='g1_navigation_nav2',
        executable='g1_twist_mux.py',
        name='g1_twist_mux',
        output='screen',
        parameters=[{
            'out_topic': '/cmd_vel',
            'timeout_sec': 0.5
        }]
    )

    # 3. Footstep Planner Action Node -> Publishes to /cmd_vel_footstep
    footstep_planner_action_node = Node(
        package='g1_footstep_planner',
        executable='footstep_planner_action_node',
        name='g1_footstep_planner_node',
        output='screen',
        parameters=[default_planner_config]
    )

    return LaunchDescription([
        network_interface_arg,
        enable_sdk_arg,
        auto_stand_arg,
        cmd_vel_to_sdk_node,
        twist_mux_node,
        footstep_planner_action_node,
    ])
