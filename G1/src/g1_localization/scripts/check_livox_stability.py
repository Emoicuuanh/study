#!/usr/bin/env python3
"""
Livox LiDAR & IMU Stability Diagnostic Script for ROS 2
Analyzes frequency, jitter, point count, packet drops, and IMU noise in real-time.
"""

import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from sensor_msgs.msg import PointCloud2, Imu

class LivoxStabilityChecker(Node):
    def __init__(self):
        super().__init__('livox_stability_checker')
        
        self.declare_parameter('lidar_topic', '/utlidar/cloud_livox_mid360_sync')
        self.declare_parameter('imu_topic', '/utlidar/imu_livox_mid360_sync')
        
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        
        self.get_logger().info(f"Starting Livox Stability Checker...")
        self.get_logger().info(f"  LiDAR Topic: {lidar_topic}")
        self.get_logger().info(f"  IMU Topic:   {imu_topic}")

        # Subscriptions
        self.sub_lidar = self.create_subscription(PointCloud2, lidar_topic, self.lidar_callback, 10)
        self.sub_imu = self.create_subscription(Imu, imu_topic, self.imu_callback, 100)

        # LiDAR Stats
        self.lidar_times = []
        self.lidar_point_counts = []
        self.lidar_drops = 0
        self.last_lidar_time = None

        # IMU Stats
        self.imu_times = []
        self.acc_norms = []
        self.gyro_norms = []
        self.imu_drops = 0
        self.last_imu_time = None

        # Timer for summary report every 2 seconds
        self.create_timer(2.0, self.print_summary)

    def lidar_callback(self, msg: PointCloud2):
        now = time.time()
        
        # Calculate point count
        # PointCloud2 width * height
        num_points = msg.width * msg.height
        self.lidar_point_counts.append(num_points)

        if self.last_lidar_time is not None:
            dt = now - self.last_lidar_time
            self.lidar_times.append(dt)
            # Flag drop if frame interval > 200ms (expected ~100ms for 10Hz)
            if dt > 0.20:
                self.lidar_drops += 1
                self.get_logger().warn(f"⚠️ [LiDAR DROP DETECTED] Delay: {dt*1000:.1f} ms | Points: {num_points}")
        
        self.last_lidar_time = now

        # Warn if scan is empty or abnormally low
        if num_points < 1000:
            self.get_logger().warn(f"⚠️ [LiDAR LOW POINTS] Point count: {num_points}")

    def imu_callback(self, msg: Imu):
        now = time.time()
        
        # Accelerometer norm
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        acc_norm = math.sqrt(ax*ax + ay*ay + az*az)
        self.acc_norms.append(acc_norm)

        # Gyroscope norm
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        gyro_norm = math.sqrt(gx*gx + gy*gy + gz*gz)
        self.gyro_norms.append(gyro_norm)

        if self.last_imu_time is not None:
            dt = now - self.last_imu_time
            self.imu_times.append(dt)
            # Flag drop if IMU interval > 15ms (expected ~5ms for 200Hz)
            if dt > 0.015:
                self.imu_drops += 1
                self.get_logger().warn(f"⚠️ [IMU DROP DETECTED] Delay: {dt*1000:.1f} ms")

        self.last_imu_time = now

        # Flag severe acceleration spikes when stationary (> 25 m/s^2)
        if acc_norm > 25.0:
            self.get_logger().warn(f"⚠️ [IMU ACC SPIKE] Acceleration norm: {acc_norm:.2f} m/s^2")

    def print_summary(self):
        print("\n" + "="*60)
        print("                LIVOX DATA STABILITY REPORT                ")
        print("="*60)
        
        # 1. LiDAR Analysis
        if self.lidar_times:
            mean_dt = np.mean(self.lidar_times)
            freq = 1.0 / mean_dt if mean_dt > 0 else 0
            std_dt_ms = np.std(self.lidar_times) * 1000
            min_dt_ms = np.min(self.lidar_times) * 1000
            max_dt_ms = np.max(self.lidar_times) * 1000
            avg_pts = int(np.mean(self.lidar_point_counts))
            min_pts = int(np.min(self.lidar_point_counts))
            max_pts = int(np.max(self.lidar_point_counts))

            print(f"📦 [LiDAR - PointCloud2]")
            print(f"   Rate / Frequency : {freq:.2f} Hz (Expected ~10 Hz)")
            print(f"   Jitter (StdDev)  : ±{std_dt_ms:.2f} ms")
            print(f"   Min / Max Delay  : {min_dt_ms:.1f} ms / {max_dt_ms:.1f} ms")
            print(f"   Point Count      : Avg {avg_pts} pts (Min: {min_pts}, Max: {max_pts})")
            print(f"   Dropped Frames   : {self.lidar_drops} drops")
        else:
            print("📦 [LiDAR - PointCloud2] Waiting for messages...")

        print("-" * 60)

        # 2. IMU Analysis
        if self.imu_times:
            mean_dt_imu = np.mean(self.imu_times)
            freq_imu = 1.0 / mean_dt_imu if mean_dt_imu > 0 else 0
            std_dt_imu_ms = np.std(self.imu_times) * 1000
            avg_acc = np.mean(self.acc_norms)
            std_acc = np.std(self.acc_norms)
            avg_gyro = np.mean(self.gyro_norms)

            print(f"🌀 [IMU - Inertial Measurement]")
            print(f"   Rate / Frequency : {freq_imu:.2f} Hz (Expected ~200 Hz)")
            print(f"   Jitter (StdDev)  : ±{std_dt_imu_ms:.2f} ms")
            print(f"   Acc Norm (Gravity): Avg {avg_acc:.3f} m/s² (StdDev: ±{std_acc:.3f})")
            print(f"   Gyro Norm (Rotation): Avg {avg_gyro:.3f} rad/s")
            print(f"   Dropped Messages : {self.imu_drops} drops")
        else:
            print("🌀 [IMU - Inertial Measurement] Waiting for messages...")

        print("="*60 + "\n")

        # Keep rolling window of last 200 samples
        if len(self.lidar_times) > 200:
            self.lidar_times = self.lidar_times[-200:]
            self.lidar_point_counts = self.lidar_point_counts[-200:]
        if len(self.imu_times) > 500:
            self.imu_times = self.imu_times[-500:]
            self.acc_norms = self.acc_norms[-500:]
            self.gyro_norms = self.gyro_norms[-500:]

def main():
    rclpy.init()
    node = LivoxStabilityChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
