#!/usr/bin/env python3

import ctypes
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

try:
    _rcutils = ctypes.CDLL('librcutils.so')
except OSError:
    try:
        _rcutils = ctypes.CDLL('/opt/ros/foxy/lib/librcutils.so')
    except OSError:
        _rcutils = None

def reset_rcutils_error():
    if _rcutils is not None:
        try:
            _rcutils.rcutils_reset_error()
        except Exception:
            pass

class TimestampCorrector(Node):
    def __init__(self):
        super().__init__('timestamp_corrector')
        
        # Subscribe to raw LiDAR pointcloud with depth=2 to prevent PointCloud queue congestion
        self.sub = self.create_subscription(
            PointCloud2,
            '/utlidar/cloud_livox_mid360',
            self.callback,
            2
        )
        
        # Publish synced LiDAR pointcloud with depth=2
        self.pub = self.create_publisher(
            PointCloud2,
            '/utlidar/cloud_livox_mid360_sync',
            2
        )
        
        self.get_logger().info("Timestamp Corrector node started.")
        self.get_logger().info("Subscribing to: /utlidar/cloud_livox_mid360")
        self.get_logger().info("Publishing to: /utlidar/cloud_livox_mid360_sync")

    def callback(self, msg: PointCloud2):
        reset_rcutils_error()
        # Overwrite the message header stamp with the current host system time
        msg.header.stamp = self.get_clock().now().to_msg()
        # Publish the corrected message
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TimestampCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

