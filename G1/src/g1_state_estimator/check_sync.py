import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class SyncChecker(Node):
    def __init__(self):
        super().__init__('sync_checker')
        self.latest_odom_time = None
        self.latest_cloud_time = None

        self.create_subscription(
            Odometry,
            '/state_estimator/odom',
            self.odom_callback,
            10
        )
        self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.cloud_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("SyncChecker Node Started. Listening to topics...")

    def odom_callback(self, msg):
        self.latest_odom_time = msg.header.stamp

    def cloud_callback(self, msg):
        self.latest_cloud_time = msg.header.stamp

    def timer_callback(self):
        if self.latest_odom_time is None:
            self.get_logger().warn("Waiting for /state_estimator/odom...")
            return
        if self.latest_cloud_time is None:
            self.get_logger().warn("Waiting for /cloud_registered...")
            return

        t_odom = self.latest_odom_time.sec + self.latest_odom_time.nanosec * 1e-9
        t_cloud = self.latest_cloud_time.sec + self.latest_cloud_time.nanosec * 1e-9
        dt = t_cloud - t_odom

        self.get_logger().info(
            f"\nOdom timestamp:  {t_odom:.4f}\n"
            f"Cloud timestamp: {t_cloud:.4f}\n"
            f"Difference (Cloud - Odom): {dt:.4f} seconds"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SyncChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
