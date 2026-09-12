#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import time

class OdomCheckerNode(Node):
    def __init__(self):
        super().__init__('odom_checker')
        self.declare_parameter('odom_topic', '/state_estimator/odom')
        odom_topic = self.get_parameter('odom_topic').value

        self.sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )

        self.msg_count = 0
        self.start_time = time.time()
        self.last_print_time = time.time()
        self.prev_pos = None
        self.total_distance = 0.0
        self.pos_window = []

        self.get_logger().info(f"Đang lắng nghe topic Odometry: {odom_topic}")
        self.get_logger().info("=========================================================")
        self.get_logger().info(" BẮT ĐẦU ĐÁNH GIÁ CHẤT LƯỢNG ODOMETRY")
        self.get_logger().info("=========================================================")

    def odom_callback(self, msg: Odometry):
        self.msg_count += 1
        pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Tính tổng quãng đường di chuyển (lọc bỏ vi nhiễu tần số 1000Hz khi đứng yên)
        if self.prev_pos is not None:
            dist = np.linalg.norm(pos - self.prev_pos)
            if dist > 0.003:  # Chỉ tính khi thực sự di chuyển > 3mm
                self.total_distance += dist

        self.prev_pos = pos
        self.pos_window.append(pos)
        if len(self.pos_window) > 500:  # Giữ cửa sổ 500 mẫu gần nhất
            self.pos_window.pop(0)

        now = time.time()
        # In thống kê mỗi 1.0 giây
        if now - self.last_print_time >= 1.0:
            elapsed = now - self.start_time
            rate = self.msg_count / (now - self.last_print_time)
            self.msg_count = 0
            self.last_print_time = now

            pos_arr = np.array(self.pos_window)
            std_dev = np.std(pos_arr, axis=0) * 1000.0  # Chuyển sang mm

            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            vz = msg.twist.twist.linear.z
            v_norm = np.sqrt(vx**2 + vy**2 + vz**2)

            print(f"[Odom Check] Tần số: {rate:6.1f} Hz | "
                  f"Vị trí (X,Y,Z): [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}]m | "
                  f"Vận tốc: {v_norm:4.2f} m/s | "
                  f"Rung tĩnh: [{std_dev[0]:3.1f}, {std_dev[1]:3.1f}, {std_dev[2]:3.1f}]mm | "
                  f"Tổng QĐ: {self.total_distance:5.2f}m")

def main():
    rclpy.init()
    node = OdomCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nĐã dừng kiểm tra Odometry.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
