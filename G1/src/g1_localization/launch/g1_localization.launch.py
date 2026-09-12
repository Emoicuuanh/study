import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_localization = get_package_share_directory('g1_localization')
    pkg_estimator = get_package_share_directory('g1_state_estimator')
    pkg_fast_lio = get_package_share_directory('fast_lio')

    # 1. Khai báo các đối số Launch
    map_pcd_arg = DeclareLaunchArgument(
        'map_pcd_path',
        default_value='/home/hoangdc/ROS2/unitree_G1/maps/g1_office.pcd',
        description='Path to the static global PCD map'
    )
    run_voxblox_local_arg = DeclareLaunchArgument(
        'run_voxblox_local', default_value='true', # Bật ESDF local để tránh vật cản khi di chuyển
        description='Launch Voxblox Local Node for obstacle avoidance'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )

    # 2. Đường dẫn các file cấu hình
    config_estimator = os.path.join(pkg_estimator, 'config', 'estimator.yaml')
    config_localization = os.path.join(pkg_localization, 'config', 'localization.yaml')

    # Đọc cấu hình Voxblox Local từ g1_state_estimator
    local_config_path = os.path.join(pkg_estimator, 'config', 'voxblox_local.yaml')
    with open(local_config_path, 'r') as f:
        local_yaml = yaml.safe_load(f)
    local_key = list(local_yaml.keys())[0]
    local_params = local_yaml[local_key]['ros__parameters']

    # 3. FAST-LIO2 (Chạy chế độ local odom, không phát map TF)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fast_lio, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'rviz': 'false',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_file': 'mid360.yaml'
        }.items()
    )

    # 4. State Estimator Node (EKF 500Hz)
    estimator_node = Node(
        package='g1_state_estimator',
        executable='state_estimator_node',
        name='g1_state_estimator',
        output='screen',
        parameters=[config_estimator, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 5. NDT Localization Node mới (pcl_localization_ros2)
    pcl_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pcl_localization_ros2'),
                'launch',
                'pcl_localization.launch.py'
            )
        )
    )

    # NDT Localization Node cũ (đã comment lại)
    # ndt_localization_node = Node(
    #     package='g1_localization',
    #     executable='ndt_localization_node',
    #     name='ndt_localization_node',
    #     output='screen',
    #     parameters=[
    #         config_localization,
    #         {
    #             'map_pcd_path': LaunchConfiguration('map_pcd_path'),
    #             'use_sim_time': LaunchConfiguration('use_sim_time')
    #         }
    #     ]
    # )

    # 6. Voxblox Local Node (ESDF collision mapping - odom frame)
    voxblox_local_node = Node(
        package='voxblox_ros',
        executable='esdf_server',
        name='voxblox_local',
        output='screen',
        parameters=[local_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/voxblox_local/pointcloud', '/utlidar/cloud_livox_mid360_sync')
        ],
        condition=IfCondition(LaunchConfiguration('run_voxblox_local'))
    )

    # 7. RViz2 chuyên dụng cho Định vị
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_localization, 'rviz', 'localization.rviz')]
    )

    return LaunchDescription([
        map_pcd_arg,
        # run_voxblox_local_arg,
        use_sim_time_arg,
      
        fast_lio_launch,
        estimator_node,
        pcl_localization_launch,
        # voxblox_local_node,
        # rviz_node
    ])
