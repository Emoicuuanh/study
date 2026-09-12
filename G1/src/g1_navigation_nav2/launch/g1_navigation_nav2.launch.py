import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Get the package share directory
    nav_share_dir = get_package_share_directory('g1_navigation_nav2')
    
    # 2. Paths to default parameters and map files
    default_params_file = os.path.join(nav_share_dir, 'config', 'nav2_params.yaml')
    default_map_file = '/home/hoangdc/ROS2/unitree_G1/maps/g1_office/g1_office.yaml'

    # 3. Declare launch configurations
    map_yaml_file = LaunchConfiguration('map', default=default_map_file)
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    autostart = LaunchConfiguration('autostart', default='true')

    # Lifecycle nodes to manage in Nav2 stack
    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'recoveries_server',
        'bt_navigator'
    ]

    # 4. Create Node actions
    
    # Map Server node (reads 2D static map)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )

    # Planner Server node (computes global path)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # Controller Server node (follows global path and outputs cmd_vel_nav)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )

    # Recoveries Server node (performs spin/backup recovery behaviors)
    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/cmd_vel_nav')]
    )

    # BT Navigator node (behavior tree coordination)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    # Twist Mux node (prioritizes teleop /cmd_vel_teleop over nav /cmd_vel_nav -> /cmd_vel_raw)
    twist_mux_node = Node(
        package='g1_navigation_nav2',
        executable='g1_twist_mux.py',
        name='g1_twist_mux',
        output='screen',
        parameters=[{'out_topic': '/cmd_vel_raw', 'timeout_sec': 0.5}]
    )

    # G1 Safety Watchdog & Velocity Smoother node (ramping acceleration, clamping, watchdog timeout)
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
            'max_vx': 0.4,
            'min_vx': -0.2,
            'max_vy': 0.15,
            'min_vy': -0.15,
            'max_vtheta': 0.5,
            'min_vtheta': -0.5,
            'acc_lim_x': 0.4,
            'acc_lim_y': 0.3,
            'acc_lim_theta': 0.8
        }]
    )

    # Lifecycle Manager node (activates/deactivates Nav2 servers)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument('map', default_value=default_map_file, description='Full path to map yaml file to load'),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),

        map_server_node,
        planner_server_node,
        controller_server_node,
        recoveries_server_node,
        bt_navigator_node,
        twist_mux_node,
        safety_watchdog_node,
        lifecycle_manager_node
    ])
