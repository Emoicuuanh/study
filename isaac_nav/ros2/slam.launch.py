"""Chay SLAM 2D tu du lieu Isaac Sim.

Luong du lieu:
  Isaac Sim --/points (3D)--> pointcloud_to_laserscan --/scan (2D)--> slam_toolbox --> /map

Vi sao ep 3D xuong 2D o tang nay: slam_toolbox lam viec voi LaserScan.
Muc dich la kiem chung duong ong + tao costmap 2D cho Nav2 (tang 4).
Ban do 3D that (elevation map cho cau thang) se lam o TANG 3.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 3D point cloud -> 2D laser scan
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pc_to_scan",
            remappings=[("cloud_in", "/points"), ("scan", "/scan")],
            parameters=[{
                "target_frame": "pelvis",
                "transform_tolerance": 0.05,
                "min_height": -0.60,      # bo diem sat san (nhieu tu mat dat)
                "max_height": 0.50,       # bo diem tren cao (tran nha)
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.0087,   # ~0.5 do
                "scan_time": 0.1,
                "range_min": 0.5,
                "range_max": 30.0,
                "use_inf": True,
                "use_sim_time": True,
            }],
        ),
        # SLAM 2D
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            parameters=[{
                "use_sim_time": True,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "pelvis",
                "scan_topic": "/scan",
                "mode": "mapping",
                "resolution": 0.05,
                "max_laser_range": 25.0,
                "minimum_travel_distance": 0.2,
                "minimum_travel_heading": 0.2,
                "transform_publish_period": 0.02,
            }],
        ),
    ])
