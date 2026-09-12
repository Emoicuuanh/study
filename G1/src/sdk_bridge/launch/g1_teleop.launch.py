import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_sdk_bridge = get_package_share_directory('sdk_bridge')
    default_config_file = os.path.join(pkg_sdk_bridge, 'config', 'sdk_bridge.yaml')

    # Launch Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to configuration file for sdk_bridge'
    )

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

    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='true',
        description='Whether to launch the keyboard teleop node'
    )

    use_watchdog_arg = DeclareLaunchArgument(
        'use_watchdog',
        default_value='false',
        description='Whether to use safety watchdog for acceleration/speed smoothing'
    )

    use_bringup_arg = DeclareLaunchArgument(
        'use_bringup',
        default_value='false',
        description='Whether to launch g1_bringup (URDF / TFs)'
    )

    # Launch configurations
    config_file = LaunchConfiguration('config_file')
    network_interface = LaunchConfiguration('network_interface')
    enable_sdk = LaunchConfiguration('enable_sdk')
    auto_stand = LaunchConfiguration('auto_stand_on_start')
    launch_teleop = LaunchConfiguration('launch_teleop')
    use_watchdog = LaunchConfiguration('use_watchdog')
    use_bringup = LaunchConfiguration('use_bringup')

    # Determine command topic for teleop:
    # If watchdog is enabled, teleop publishes to /cmd_vel_raw, watchdog outputs to /cmd_vel
    # If watchdog is disabled, teleop publishes directly to /cmd_vel
    teleop_cmd_topic = PythonExpression([
        "'/cmd_vel_raw' if '", use_watchdog, "'.lower() == 'true' else '/cmd_vel'"
    ])

    # 1. SDK Bridge Node (cmd_vel_to_sdk_node)
    cmd_vel_to_sdk_node = Node(
        package='sdk_bridge',
        executable='cmd_vel_to_sdk_node',
        name='cmd_vel_to_sdk_node',
        output='screen',
        parameters=[
            config_file,
            {
                'network_interface': network_interface,
                'enable_sdk': PythonExpression(["'", enable_sdk, "'.lower() == 'true'"]),
                'auto_stand_on_start': PythonExpression(["'", auto_stand, "'.lower() == 'true'"]),
                'cmd_vel_topic': '/cmd_vel'
            }
        ]
    )

    # 2. Keyboard Teleop Node (launched in a separate popup terminal window for keyboard interaction)
    keyboard_teleop_node = Node(
        package='sdk_bridge',
        executable='keyboard_teleop_node',
        name='keyboard_teleop_node',
        output='screen',
        prefix=['gnome-terminal --'],
        parameters=[{
            'cmd_vel_topic': teleop_cmd_topic,
            'speed_linear_step': 0.02,
            'speed_angular_step': 0.02,
            'max_linear': 1.2,
            'max_angular': 0.5
        }],
        condition=IfCondition(launch_teleop)
    )

    # 3. Optional Safety Watchdog Node (from g1_navigation_nav2)
    safety_watchdog_node = Node(
        package='g1_navigation_nav2',
        executable='g1_safety_watchdog.py',
        name='g1_safety_watchdog',
        output='screen',
        parameters=[{
            'cmd_vel_in_topic': '/cmd_vel_raw',
            'cmd_vel_out_topic': '/cmd_vel',
            'timeout_sec': 0.5,
            'publish_rate_hz': 20.0,
            'max_vx': 0.8,
            'min_vx': -0.4,
            'max_vy': 0.4,
            'min_vy': -0.4,
            'max_vtheta': 0.5,
            'min_vtheta': -0.5,
            'acc_lim_x': 0.4,
            'acc_lim_y': 0.3,
            'acc_lim_theta': 0.8
        }],
        condition=IfCondition(use_watchdog)
    )

    ld = LaunchDescription([
        config_file_arg,
        network_interface_arg,
        enable_sdk_arg,
        auto_stand_arg,
        launch_teleop_arg,
        use_watchdog_arg,
        use_bringup_arg,
        cmd_vel_to_sdk_node,
        keyboard_teleop_node,
        safety_watchdog_node,
    ])

    # 4. Optional Bringup (g1_bringup.launch.py)
    try:
        g1_bringup_dir = get_package_share_directory('g1_bringup')
        g1_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(g1_bringup_dir, 'launch', 'g1_bringup.launch.py')
            ),
            condition=IfCondition(use_bringup)
        )
        ld.add_action(g1_bringup_launch)
    except Exception:
        pass

    return ld
