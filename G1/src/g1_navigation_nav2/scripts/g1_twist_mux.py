#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class G1TwistMux(Node):
    """
    Lightweight, zero-dependency Twist Multiplexer Node for Unitree G1 ROS 2.
    Prioritizes input velocity topics:
    Priority 1 (100): /cmd_vel_teleop (Keyboard / Joystick Manual Override)
    Priority 2 (75) : /cmd_vel_footstep (Footstep Planner Action Server)
    Priority 3 (50) : /cmd_vel_nav (Nav2 Autonomous Navigation)
    Output: /cmd_vel (or configured out_topic) -> sent to sdk_bridge
    """
    def __init__(self):
        super().__init__('g1_twist_mux')

        self.declare_parameter('out_topic', '/cmd_vel')
        self.declare_parameter('timeout_sec', 0.5)

        out_topic = self.get_parameter('out_topic').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        # Output publisher to sdk_bridge
        self.pub_out = self.create_publisher(Twist, out_topic, 10)

        # Priority 1: Teleop (high priority = 100)
        self.teleop_twist = Twist()
        self.teleop_time = 0.0
        self.sub_teleop = self.create_subscription(
            Twist, '/cmd_vel_teleop', self.teleop_callback, 10
        )

        # Priority 2: Footstep Planner (medium-high priority = 75)
        self.footstep_twist = Twist()
        self.footstep_time = 0.0
        self.sub_footstep = self.create_subscription(
            Twist, '/cmd_vel_footstep', self.footstep_callback, 10
        )

        # Priority 3: Nav2 (default priority = 50)
        self.nav_twist = Twist()
        self.nav_time = 0.0
        self.sub_nav = self.create_subscription(
            Twist, '/cmd_vel_nav', self.nav_callback, 10
        )

        # Timer loop at 20Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(
            f"G1 Twist Mux running: prioritizing /cmd_vel_teleop (100) > /cmd_vel_footstep (75) > /cmd_vel_nav (50) -> publishing to {out_topic}"
        )

    def teleop_callback(self, msg: Twist):
        self.teleop_twist = msg
        self.teleop_time = time.time()

    def footstep_callback(self, msg: Twist):
        self.footstep_twist = msg
        self.footstep_time = time.time()

    def nav_callback(self, msg: Twist):
        self.nav_twist = msg
        self.nav_time = time.time()

    def control_loop(self):
        now = time.time()
        out_msg = Twist()

        # Check Priority 1 (Teleop)
        if (now - self.teleop_time) <= self.timeout_sec:
            out_msg = self.teleop_twist
        # Check Priority 2 (Footstep Planner)
        elif (now - self.footstep_time) <= self.timeout_sec:
            out_msg = self.footstep_twist
        # Check Priority 3 (Nav2)
        elif (now - self.nav_time) <= self.timeout_sec:
            out_msg = self.nav_twist

        self.pub_out.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = G1TwistMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
