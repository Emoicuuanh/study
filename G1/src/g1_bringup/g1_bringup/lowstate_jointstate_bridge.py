#!/usr/bin/env python3

import ctypes
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_hg.msg import LowState

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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

# Define the standard motor mapping for Unitree G1.
# G1's low-level controller publishes joint angles in this specific index order.
G1_JOINT_INDEX_MAP = {
    # Left Leg (0-5)
    'left_hip_pitch_joint': 0,
    'left_hip_roll_joint': 1,
    'left_hip_yaw_joint': 2,
    'left_knee_joint': 3,
    'left_ankle_pitch_joint': 4,
    'left_ankle_roll_joint': 5,
    
    # Right Leg (6-11)
    'right_hip_pitch_joint': 6,
    'right_hip_roll_joint': 7,
    'right_hip_yaw_joint': 8,
    'right_knee_joint': 9,
    'right_ankle_pitch_joint': 10,
    'right_ankle_roll_joint': 11,
    
    # Waist (12-14)
    'waist_yaw_joint': 12,
    'waist_roll_joint': 13,    # Fixed (locked) on 23-DoF model, active on 29-DoF
    'waist_pitch_joint': 14,   # Fixed (locked) on 23-DoF model, active on 29-DoF
    
    # Left Arm (15-21)
    'left_shoulder_pitch_joint': 15,
    'left_shoulder_roll_joint': 16,
    'left_shoulder_yaw_joint': 17,
    'left_elbow_joint': 18,
    'left_wrist_roll_joint': 19,
    'left_wrist_pitch_joint': 20, # Only present on 29-DoF
    'left_wrist_yaw_joint': 21,   # Only present on 29-DoF
    
    # Right Arm (22-28)
    'right_shoulder_pitch_joint': 22,
    'right_shoulder_roll_joint': 23,
    'right_shoulder_yaw_joint': 24,
    'right_elbow_joint': 25,
    'right_wrist_roll_joint': 26,
    'right_wrist_pitch_joint': 27, # Only present on 29-DoF
    'right_wrist_yaw_joint': 28,   # Only present on 29-DoF
}

class LowStateJointStateBridge(Node):
    def __init__(self):
        super().__init__('lowstate_jointstate_bridge')

        # Declare parameters
        self.declare_parameter('robot_description', '')
        self.declare_parameter('lowstate_topic', 'lowstate')
        self.declare_parameter('joint_states_topic', 'joint_states')

        # Retrieve topics names
        lowstate_topic = self.get_parameter('lowstate_topic').get_parameter_value().string_value
        joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value

        self.get_logger().info(f"Subscribing to LowState topic: {lowstate_topic}")
        self.get_logger().info(f"Publishing JointState topic: {joint_states_topic}")

        # Active joints found in URDF
        self.active_joints = []
        self.cached_motor_indices = []
        self.urdf_parsed = False

        # Create Publisher
        self.joint_state_pub = self.create_publisher(JointState, joint_states_topic, 50)

        # Create Subscriber with generous queue depth (200) to prevent 500Hz buffer drops
        self.lowstate_sub = self.create_subscription(
            LowState,
            lowstate_topic,
            self.lowstate_callback,
            200
        )

        # Timer to check parameter robot_description if not loaded initially
        self.create_timer(1.0, self.check_urdf_parameter)

    def check_urdf_parameter(self):
        if self.urdf_parsed:
            return
        
        urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value
        if urdf_string:
            self.parse_urdf(urdf_string)

    def parse_urdf(self, urdf_string):
        try:
            root = ET.fromstring(urdf_string)
            self.active_joints = []
            self.cached_motor_indices = []
            
            # Find all revolute and continuous joints
            for joint in root.findall('joint'):
                jname = joint.get('name')
                jtype = joint.get('type')
                if jtype in ['revolute', 'continuous']:
                    if jname in G1_JOINT_INDEX_MAP:
                        self.active_joints.append(jname)
                        self.cached_motor_indices.append(G1_JOINT_INDEX_MAP[jname])
                    else:
                        self.get_logger().warn(
                            f"Joint '{jname}' of type '{jtype}' found in URDF, "
                            f"but it is not mapped to any low-level motor index in our mapping table."
                        )
            
            self.get_logger().info(
                f"Successfully parsed robot_description URDF. "
                f"Found {len(self.active_joints)} active joints matching G1 motor mappings."
            )
            self.urdf_parsed = True
        except Exception as e:
            self.get_logger().error(f"Error parsing robot_description XML parameter: {e}")

    def lowstate_callback(self, msg: LowState):
        # Parse URDF dynamically if not parsed yet
        if not self.urdf_parsed:
            urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value
            if urdf_string:
                self.parse_urdf(urdf_string)
            else:
                self.get_logger().warn(
                    "Waiting for 'robot_description' parameter to parse joint list. "
                    "Cannot publish joint states yet."
                )
                return

        motor_state = msg.motor_state
        motor_len = len(motor_state)
        
        # Fast path: Prepare JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.active_joints

        positions = []
        velocities = []
        efforts = []
        
        # Populate values using pre-cached indices
        for motor_idx in self.cached_motor_indices:
            if motor_idx < motor_len:
                m = motor_state[motor_idx]
                positions.append(float(m.q))
                velocities.append(float(m.dq))
                efforts.append(float(m.tau_est))

        joint_state_msg.position = positions
        joint_state_msg.velocity = velocities
        joint_state_msg.effort = efforts

        # Publish the state
        if joint_state_msg.position:
            self.joint_state_pub.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LowStateJointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

