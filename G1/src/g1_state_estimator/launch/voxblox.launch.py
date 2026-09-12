import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Tìm đường dẫn đến package g1_state_estimator
    share_dir = get_package_share_directory('g1_state_estimator')
  
    # Đường dẫn đến các file cấu hình yaml
    global_config_path = os.path.join(share_dir, 'config', 'voxblox_global.yaml')
    local_config_path = os.path.join(share_dir, 'config', 'voxblox_local.yaml')

    # Đọc tham số từ file yaml trong python để truyền trực tiếp dưới dạng dict
    with open(global_config_path, 'r') as f:
        global_yaml = yaml.safe_load(f)
    global_key = list(global_yaml.keys())[0]
    global_params = global_yaml[global_key]['ros__parameters']

    with open(local_config_path, 'r') as f:
        local_yaml = yaml.safe_load(f)
    local_key = list(local_yaml.keys())[0]
    local_params = local_yaml[local_key]['ros__parameters']

    # 1. Node Voxblox Global: Dựng bản đồ Mesh 3D toàn cục (hệ tọa độ 'map')
    # Nhận đám mây điểm đã được đăng ký toàn cục để tự động sửa sai khi có Loop Closure.
    global_node = Node(
        package='voxblox_ros',
        executable='tsdf_server',
        name='voxblox_global',
        output='screen',
        parameters=[global_params],
        remappings=[
            ('/voxblox_global/pointcloud', '/utlidar/cloud_livox_mid360_sync')
        ]
    )

    # 2. Node Voxblox Local: Né vật cản thời gian thực (hệ tọa độ 'odom')
    # Nhận đám mây điểm LiDAR đồng bộ tần số cao để tính khoảng cách vật cản (ESDF) mượt mà liên tục.
    local_node = Node(
        package='voxblox_ros',
        executable='esdf_server',
        name='voxblox_local',
        output='screen',
        parameters=[local_params],
        remappings=[
            ('/voxblox_local/pointcloud', '/utlidar/cloud_livox_mid360_sync')
        ]
    )

    return LaunchDescription([
        global_node
        # local_node
    ])

