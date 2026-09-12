#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class G1SafetyWatchdog(Node):
    """
    Combined Velocity Smoother, Clamping, and Safety Watchdog Node for Unitree G1.
    - Applies smooth acceleration and deceleration limits.
    - Clamps velocity values within absolute hardware limits.
    - Zeroes velocity output if input command times out (watchdog).
    - Supports E-Stop emergency stop signal.
    """
    def __init__(self):
        super().__init__('g1_safety_watchdog')

        # Declare parameters
        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_raw')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel')
        self.declare_parameter('e_stop_topic', '/e_stop')
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)

        # Velocity limits
        self.declare_parameter('max_vx', 0.4)
        self.declare_parameter('min_vx', -0.2)
        self.declare_parameter('max_vy', 0.15)
        self.declare_parameter('min_vy', -0.15)
        self.declare_parameter('max_vtheta', 0.5)
        self.declare_parameter('min_vtheta', -0.5)

        # Acceleration limits (m/s^2 or rad/s^2) - increased for faster responsiveness
        self.declare_parameter('acc_lim_x', 0.8)
        self.declare_parameter('acc_lim_y', 0.6)
        self.declare_parameter('acc_lim_theta', 1.2)


        # Retrieve parameters
        in_topic = self.get_parameter('cmd_vel_in_topic').value
        out_topic = self.get_parameter('cmd_vel_out_topic').value
        e_stop_topic = self.get_parameter('e_stop_topic').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.max_vx = float(self.get_parameter('max_vx').value)
        self.min_vx = float(self.get_parameter('min_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.min_vy = float(self.get_parameter('min_vy').value)
        self.max_vtheta = float(self.get_parameter('max_vtheta').value)
        self.min_vtheta = float(self.get_parameter('min_vtheta').value)

        self.acc_lim_x = float(self.get_parameter('acc_lim_x').value)
        self.acc_lim_y = float(self.get_parameter('acc_lim_y').value)
        self.acc_lim_theta = float(self.get_parameter('acc_lim_theta').value)

        # State tracking for smoothing & watchdog
        self.last_msg_time = 0.0
        self.target_twist = Twist()
        self.current_twist = Twist()
        self.e_stop_active = False

        # Subscriptions
        self.sub_vel = self.create_subscription(Twist, in_topic, self.cmd_vel_callback, 10)
        self.sub_estop = self.create_subscription(Bool, e_stop_topic, self.estop_callback, 10)

        # Publisher
        self.pub_vel = self.create_publisher(Twist, out_topic, 10)

        # Timer loop
        self.dt = 1.0 / self.rate_hz
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f"G1 Safety Watchdog & Smoother initialized: listening on {in_topic}, publishing to {out_topic} (Timeout: {self.timeout_sec}s)"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.target_twist = msg
        self.last_msg_time = time.time()

    def estop_callback(self, msg: Bool):
        self.e_stop_active = msg.data
        if self.e_stop_active:
            self.get_logger().warn("EMERGENCY STOP (E-STOP) ACTIVATED! Zeroing all velocities.")

    def clamp(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def ramp_value(self, current, target, max_change):
        diff = target - current
        if abs(diff) <= max_change:
            return target
        return current + max_change if diff > 0 else current - max_change

    def control_loop(self):
        now = time.time()

        # Target velocities (zeroed if E-stop or timed out)
        if self.e_stop_active or (now - self.last_msg_time) > self.timeout_sec:
            target_x = 0.0
            target_y = 0.0
            target_theta = 0.0
        else:
            target_x = self.clamp(self.target_twist.linear.x, self.min_vx, self.max_vx)
            target_y = self.clamp(self.target_twist.linear.y, self.min_vy, self.max_vy)
            target_theta = self.clamp(self.target_twist.angular.z, self.min_vtheta, self.max_vtheta)

        # Smooth acceleration / deceleration
        max_dx = self.acc_lim_x * self.dt
        max_dy = self.acc_lim_y * self.dt
        max_dtheta = self.acc_lim_theta * self.dt

        self.current_twist.linear.x = self.ramp_value(self.current_twist.linear.x, target_x, max_dx)
        self.current_twist.linear.y = self.ramp_value(self.current_twist.linear.y, target_y, max_dy)
        self.current_twist.angular.z = self.ramp_value(self.current_twist.angular.z, target_theta, max_dtheta)

        # Publish smooth safe twist
        self.pub_vel.publish(self.current_twist)

def main(args=None):
    rclpy.init(args=args)
    node = G1SafetyWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
