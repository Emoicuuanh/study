#!/usr/bin/env python3

import os
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Keyboard controls instructions banner
BANNER = """
================================================================
          Unitree G1 Teleop Keyboard Control Node
================================================================
   W : Tiến (Forward)            Q : Xoay trái (Turn Left)
   S : Lùi  (Backward)           E : Xoay phải (Turn Right)
   A : Sang trái (Side Left)     D : Sang phải (Side Right)

   SPACE / K : Dừng khẩn cấp (Emergency Stop 0 m/s)
   
   U / J : Tăng / Giảm tốc độ di chuyển (Linear speed +/-)
   I / M : Tăng / Giảm tốc độ xoay     (Angular speed +/-)

   CTRL-C để thoát (Quit)
================================================================
"""

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_teleop')
        self.declare_parameter('speed_linear_step', 0.01)
        self.declare_parameter('speed_angular_step', 0.1)
        self.declare_parameter('max_linear', 1.2)

        self.declare_parameter('max_angular', 0.5)

        topic_name = self.get_parameter('cmd_vel_topic').value
        self.linear_step = float(self.get_parameter('speed_linear_step').value)
        self.angular_step = float(self.get_parameter('speed_angular_step').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.pub_vel = self.create_publisher(Twist, topic_name, 10)

        # Current target speeds
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vtheta = 0.0

        # Current speed limits
        self.speed_linear = 0.2    # Default linear speed step (m/s)
        self.speed_angular = 0.3   # Default angular speed step (rad/s)

        self.get_logger().info(f"Keyboard Teleop initialized! Publishing to {topic_name}")

    def print_status(self):
        sys.stdout.write(
            f"\rCurrent target: vx={self.target_vx:+.2f} m/s | vy={self.target_vy:+.2f} m/s | vtheta={self.target_vtheta:+.2f} rad/s   "
        )
        sys.stdout.flush()

    def clamp(self, val, limit):
        return max(-limit, min(val, limit))

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = float(self.target_vx)
        msg.linear.y = float(self.target_vy)
        msg.angular.z = float(self.target_vtheta)
        self.pub_vel.publish(msg)
        self.print_status()

    def getKey(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        print(BANNER)
        self.print_status()

        try:
            while rclpy.ok():
                key = self.getKey(settings)
                if key != '':
                    k = key.lower()
                    if k == 'w':
                        self.target_vx = self.clamp(self.target_vx + self.linear_step, self.max_linear)
                    elif k == 's':
                        self.target_vx = self.clamp(self.target_vx - self.linear_step, self.max_linear)
                    elif k == 'a':
                        self.target_vy = self.clamp(self.target_vy + self.linear_step, self.max_linear)
                    elif k == 'd':
                        self.target_vy = self.clamp(self.target_vy - self.linear_step, self.max_linear)
                    elif k == 'q':
                        self.target_vtheta = self.clamp(self.target_vtheta + self.angular_step, self.max_angular)
                    elif k == 'e':
                        self.target_vtheta = self.clamp(self.target_vtheta - self.angular_step, self.max_angular)
                    elif k == ' ' or k == 'k':
                        # Dừng khẩn cấp
                        self.target_vx = 0.0
                        self.target_vy = 0.0
                        self.target_vtheta = 0.0
                    elif k == 'u':
                        self.speed_linear = min(self.max_linear, self.speed_linear + 0.05)
                        print(f"\n[Info] Linear Speed step increased to {self.speed_linear:.2f} m/s")
                    elif k == 'j':
                        self.speed_linear = max(0.05, self.speed_linear - 0.05)
                        print(f"\n[Info] Linear Speed step decreased to {self.speed_linear:.2f} m/s")
                    elif k == '\x03':  # CTRL-C
                        break

                    self.publish_twist()
                else:
                    # Continuous publishing current target twist
                    self.publish_twist()

        except Exception as e:
            print(f"\nError reading keyboard: {e}")
        finally:
            # Stop robot on exit
            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_vtheta = 0.0
            self.publish_twist()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            print("\nTeleop node terminated gracefully.")


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        node.run_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
