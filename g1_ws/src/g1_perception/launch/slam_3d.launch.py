"""CHANG B - SLAM 3D bang FAST-LIO (LiDAR-Inertial Odometry).

    ros2 launch g1_perception slam_3d.launch.py

Duong ong:
    /points (x,y,z)  +  /imu (500Hz)
        |
        | lidar_add_fields  - them `time` va `ring` cho tung diem
        v
    /points_ts  +  /imu
        |
        | fast_lio  - go meo bang IMU, ghep diem, uoc luong tu the
        v
    /cloud_registered (ban do 3D)  +  /Odometry  +  TF camera_init -> body
        |
        | lio_eval  - neo camera_init vao odom va DO SAI SO so voi ground truth
        v
    log sai so theo thoi gian

KHAC GI CHANG A (SLAM 2D):
    chang A: ep 3D xuong 2D roi khop anh 2D. Mat het thong tin do cao, va
             rat nhay voi goc nghieng cua robot (da do: nghieng 4.43 do lam
             dai chieu cao cat vao san -> ban do hong).
    chang B: giu nguyen 3D. Dung IMU 500Hz de biet tu the tai TUNG THOI
             DIEM trong vong quet 100ms -> go meo. Day moi la thu chay duoc
             tren robot chan that.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG_DIR, "config")


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim = {"use_sim_time": use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        # de chay THI NGHIEM DOI CHUNG ve cach suy thoi gian tung diem:
        #   ros2 launch ... slam_3d.launch.py time_mode:=none
        DeclareLaunchArgument("time_mode", default_value="azimuth",
                              description="azimuth | azimuth_rev | none"),

        # Bo sung truong `time` + `ring` - FAST-LIO khong chay duoc thieu no
        Node(
            package="g1_perception", executable="lidar_add_fields.py",
            name="lidar_add_fields",
            parameters=[sim, {"time_mode": LaunchConfiguration("time_mode")}],
            output="screen",
        ),

        Node(
            package="fast_lio", executable="fastlio_mapping",
            name="fast_lio",
            parameters=[os.path.join(CONFIG, "fast_lio_g1_os0.yaml"), sim],
            output="screen",
        ),

        # Neo camera_init vao odom + do sai so so voi ground truth
        Node(
            package="g1_perception", executable="lio_eval.py",
            name="lio_eval", parameters=[sim], output="screen",
        ),
    ])
