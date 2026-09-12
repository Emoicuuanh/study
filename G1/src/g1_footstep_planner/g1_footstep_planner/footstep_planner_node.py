#!/usr/bin/env python3
"""
ROS 2 Footstep Planner Node for Unitree G1.
Receives Goal Pose, calculates footsteps & 3-phase execution impulses above SDK deadband,
and sends precise cmd_vel velocity pulses to sdk_bridge.
"""

import time
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf2_ros import TransformException

from g1_footstep_planner.footstep_calculator import FootstepCalculator, normalize_angle


class FootstepPlannerNode(Node):
    """
    ROS 2 Node for Pose-Based Footstep Planning and Impulse Execution on Unitree G1.
    """

    def __init__(self):
        super().__init__('g1_footstep_planner_node')

        # Declare ROS 2 parameters
        self.declare_parameter('vx_exec', 0.12)
        self.declare_parameter('vy_exec', 0.12)
        self.declare_parameter('wz_exec', 0.12)
        self.declare_parameter('v_min', 0.10)
        self.declare_parameter('w_min', 0.10)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('g1_mode_topic', '/g1_mode')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Retrieve parameter values
        self.vx_exec = float(self.get_parameter('vx_exec').value)
        self.vy_exec = float(self.get_parameter('vy_exec').value)
        self.wz_exec = float(self.get_parameter('wz_exec').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.w_min = float(self.get_parameter('w_min').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.g1_mode_topic = str(self.get_parameter('g1_mode_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        # Instantiate Footstep Calculator
        self.calculator = FootstepCalculator(
            vx_exec=self.vx_exec,
            vy_exec=self.vy_exec,
            wz_exec=self.wz_exec,
            v_min=self.v_min,
            w_min=self.w_min,
        )

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.pub_g1_mode = self.create_publisher(String, self.g1_mode_topic, 10)
        self.pub_status = self.create_publisher(String, '/g1_footstep_status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/g1_footstep_markers', 10)

        # Subscribers
        self.sub_goal_pose = self.create_subscription(
            PoseStamped, '/g1_footstep_goal', self.goal_pose_callback, 10
        )
        # Also subscribe to standard RViz /goal_pose for convenience
        self.sub_rviz_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10
        )

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State Variables
        self.is_executing = False
        self.current_impulses = []
        self.current_impulse_idx = 0
        self.impulse_start_time = None
        self.status = "IDLE"

        # Control Loop Timer (20Hz = 0.05s)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("==============================================")
        self.get_logger().info("      G1 Footstep Planner Node Initialized    ")
        self.get_logger().info("==============================================")
        self.get_logger().info(f"Execution Speeds : vx={self.vx_exec}m/s, vy={self.vy_exec}m/s, wz={self.wz_exec}rad/s")
        self.get_logger().info(f"SDK Deadbands    : v_min={self.v_min}m/s, w_min={self.w_min}rad/s")
        self.get_logger().info(f"Subscribing Goals: /g1_footstep_goal, /goal_pose")
        self.get_logger().info(f"Publishing CmdVel: {self.cmd_vel_topic}")

        self.publish_status("IDLE")

    def publish_status(self, status_msg: str):
        self.status = status_msg
        msg = String()
        msg.data = status_msg
        self.pub_status.publish(msg)

    def get_current_robot_pose(self) -> Tuple[float, float, float]:
        """Reads current robot (x, y, yaw) from TF relative to odom frame."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                now,
                timeout=Duration(seconds=0.2)
            )
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y

            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w

            # Yaw from quaternion
            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            return tx, ty, yaw

        except TransformException as ex:
            self.get_logger().warn(f"TF Lookup failed ({self.odom_frame} -> {self.base_frame}): {ex}")
            return 0.0, 0.0, 0.0

    def goal_pose_callback(self, msg: PoseStamped):
        """Callback triggered when a new target Pose is received."""
        if self.is_executing:
            self.get_logger().warn("Already executing a footstep plan! Overwriting with new goal...")

        # Extract goal coordinates
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        goal_yaw = math.atan2(siny_cosp, cosy_cosp)

        self.get_logger().info("----------------------------------------------")
        self.get_logger().info(f"Received New Goal Pose: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(goal_yaw):.1f}°")

        # Get current robot pose
        curr_x, curr_y, curr_yaw = self.get_current_robot_pose()
        self.get_logger().info(f"Current Robot Pose   : x={curr_x:.3f}, y={curr_y:.3f}, yaw={math.degrees(curr_yaw):.1f}°")

        # Calculate relative pose in robot frame
        dx, dy, dyaw = self.calculator.calculate_relative_pose(
            curr_x, curr_y, curr_yaw, goal_x, goal_y, goal_yaw
        )
        self.get_logger().info(f"Relative Offset (Local): dx={dx:.3f}m, dy={dy:.3f}m, dyaw={math.degrees(dyaw):.1f}°")

        # Plan 3-phase execution impulses
        self.current_impulses = self.calculator.plan_impulses(dx, dy, dyaw)
        self.current_impulse_idx = 0
        self.impulse_start_time = self.get_clock().now()

        if not self.current_impulses:
            self.get_logger().info("Goal position already reached within tolerance! No footsteps required.")
            self.publish_status("COMPLETED")
            return

        self.get_logger().info(f"Generated {len(self.current_impulses)} Impulse Phases:")
        for imp in self.current_impulses:
            self.get_logger().info(f"  -> {imp}")

        # Visualize footsteps in RViz
        discrete_steps = self.calculator.generate_discrete_footsteps(dx, dy, dyaw)
        self.publish_footstep_markers(discrete_steps)

        self.is_executing = True
        self.publish_status(f"EXECUTING_{self.current_impulses[0].phase_name}")

    def publish_footstep_markers(self, footsteps):
        """Publishes RViz MarkerArray visualizing left/right footsteps."""
        marker_array = MarkerArray()

        for step in footsteps:
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "g1_footsteps"
            marker.id = step.step_index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = step.x
            marker.pose.position.y = step.y
            marker.pose.position.z = 0.02  # Slight elevation on ground

            # Orientation
            cy = math.cos(step.yaw * 0.5)
            sy = math.sin(step.yaw * 0.5)
            marker.pose.orientation.w = cy
            marker.pose.orientation.z = sy

            # Foot dimensions
            marker.scale.x = 0.16  # Foot length
            marker.scale.y = 0.08  # Foot width
            marker.scale.z = 0.02  # Foot height

            # Color: Green for LEFT, Blue for RIGHT
            if step.foot == 'LEFT':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.2
            else:
                marker.color.r = 0.0
                marker.color.g = 0.4
                marker.color.b = 1.0
            marker.color.a = 0.8

            marker_array.markers.append(marker)

        self.pub_markers.publish(marker_array)

    def control_loop(self):
        """20Hz Control Loop executing active impulse phase."""
        if not self.is_executing or not self.current_impulses:
            return

        current_impulse = self.current_impulses[self.current_impulse_idx]
        now = self.get_clock().now()
        elapsed_sec = (now - self.impulse_start_time).nanoseconds / 1e9

        if elapsed_sec < current_impulse.duration:
            # Publish impulse velocity
            cmd = Twist()
            cmd.linear.x = current_impulse.vx
            cmd.linear.y = current_impulse.vy
            cmd.angular.z = current_impulse.wz
            self.pub_cmd_vel.publish(cmd)
        else:
            # Current phase completed, stop velocity briefly
            cmd = Twist()
            self.pub_cmd_vel.publish(cmd)

            self.get_logger().info(f"Finished Phase {current_impulse.phase_name} (Duration: {elapsed_sec:.2f}s)")
            self.current_impulse_idx += 1

            if self.current_impulse_idx < len(self.current_impulses):
                # Move to next phase
                next_impulse = self.current_impulses[self.current_impulse_idx]
                self.impulse_start_time = self.get_clock().now()
                self.get_logger().info(f"Starting Phase {next_impulse.phase_name}: {next_impulse}")
                self.publish_status(f"EXECUTING_{next_impulse.phase_name}")
            else:
                # All phases completed!
                self.is_executing = False
                self.get_logger().info("==============================================")
                self.get_logger().info("  Footstep Impulse Execution COMPLETED!       ")
                self.get_logger().info("==============================================")

                # Send BalanceStand command
                mode_msg = String()
                mode_msg.data = "balance_stand"
                self.pub_g1_mode.publish(mode_msg)

                self.publish_status("COMPLETED")


def main(args=None):
    rclpy.init(args=args)
    node = FootstepPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
