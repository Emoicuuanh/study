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
    # 1. Tìm đường dẫn đến các package
    pkg_estimator = get_package_share_directory('g1_state_estimator')
    pkg_fast_lio = get_package_share_directory('fast_lio')

    # 2. Khai báo các đối số Launch (Launch Arguments)
    run_voxblox_global_arg = DeclareLaunchArgument(
        'run_voxblox_global', default_value='true',
        description='Launch Voxblox Global Node (TSDF mapping)'
    )
    run_voxblox_local_arg = DeclareLaunchArgument(
        'run_voxblox_local', default_value='false',  # Mặc định tắt để tiết kiệm CPU, bật khi cần tránh vật cản
        description='Launch Voxblox Local Node (ESDF collision mapping)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock if true'
    )

    # 3. Đọc cấu hình cho EKF State Estimator và GTSAM Loop Closure
    config_estimator = os.path.join(pkg_estimator, 'config', 'estimator.yaml')
    config_loop_closure = os.path.join(pkg_estimator, 'config', 'gtsam_loop_closure.yaml')

    # 4. Đọc tham số Voxblox (đọc trực tiếp qua python dict do hạn chế của parser Foxy)
    global_config_path = os.path.join(pkg_estimator, 'config', 'voxblox_global.yaml')
    local_config_path = os.path.join(pkg_estimator, 'config', 'voxblox_local.yaml')

    with open(global_config_path, 'r') as f:
        global_yaml = yaml.safe_load(f)
    global_key = list(global_yaml.keys())[0]
    global_params = global_yaml[global_key]['ros__parameters']

    with open(local_config_path, 'r') as f:
        local_yaml = yaml.safe_load(f)
    local_key = list(local_yaml.keys())[0]
    local_params = local_yaml[local_key]['ros__parameters']

    # ----------------------------------------------------
    # KHAI BÁO CÁC NODE VÀ LAUNCH CHI TIẾT
    # ----------------------------------------------------

    # 1. FAST-LIO2 mapping (Gọi file launch gốc của nó, tắt rviz mặc định)
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

    # 2. State Estimator Node (Local EKF - 500Hz)
    estimator_node = Node(
        package='g1_state_estimator',
        executable='state_estimator_node',
        name='g1_state_estimator',
        output='screen',
        parameters=[config_estimator, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 3. GTSAM Loop Closure Node (Global Backend)
    loop_closure_node = Node(
        package='g1_state_estimator',
        executable='gtsam_loop_closure_node',
        name='gtsam_loop_closure_node',
        output='screen',
        parameters=[config_loop_closure, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 4. Voxblox Global Node (TSDF mapping - map frame)
    voxblox_global_node = Node(
        package='voxblox_ros',
        executable='tsdf_server',
        name='voxblox_global',
        output='screen',
        parameters=[global_params, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/voxblox_global/pointcloud', '/utlidar/cloud_livox_mid360_sync')
        ],
        condition=IfCondition(LaunchConfiguration('run_voxblox_global'))
    )

    # 5. Voxblox Local Node (ESDF mapping - odom frame)
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

    return LaunchDescription([
        run_voxblox_global_arg,
        run_voxblox_local_arg,
        use_sim_time_arg,
        
        # Chạy toàn bộ các thành phần
        fast_lio_launch,
        estimator_node,
        loop_closure_node,
        voxblox_global_node,
        voxblox_local_node
    ])
