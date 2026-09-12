#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult


# Dynamically import Unitree SDK 2 python modules
try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    UNITREE_SDK_AVAILABLE = True
except ImportError:
    try:
        # Alternative module structure for sport/loco client in unitree_sdk2py
        from unitree_sdk2py.go2.sport.sport_client import SportClient as LocoClient
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        UNITREE_SDK_AVAILABLE = True
    except ImportError:
        UNITREE_SDK_AVAILABLE = False


class CmdVelToSDKNode(Node):
    """
    ROS 2 Node bridging safe /cmd_vel messages to Unitree G1 SDK 2 Motion Client.
    """
    def __init__(self):
        super().__init__('cmd_vel_to_sdk_node')

        # Declare parameters
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('enable_sdk', True)
        self.declare_parameter('auto_stand_on_start', True)
        self.declare_parameter('cmd_send_rate_hz', 20.0)
        self.declare_parameter('motion_mode_topic', '/g1_mode')
        self.declare_parameter('speed_mode', 0)
        self.declare_parameter('swing_height', 0.0)

        # Retrieve parameter values
        self.net_if = self.get_parameter('network_interface').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.enable_sdk = self.get_parameter('enable_sdk').value
        self.auto_stand = self.get_parameter('auto_stand_on_start').value
        self.motion_mode_topic = self.get_parameter('motion_mode_topic').value
        self.current_speed_mode = int(self.get_parameter('speed_mode').value)
        self.current_swing_height = float(self.get_parameter('swing_height').value)
        rate_hz = float(self.get_parameter('cmd_send_rate_hz').value)

        self.latest_twist = Twist()
        self.last_cmd_time = 0.0
        self.sdk_ready = False
        self.sport_client = None

        self.get_logger().info("==========================================")
        self.get_logger().info("   G1 SDK Bridge: cmd_vel_to_sdk_node     ")
        self.get_logger().info("==========================================")
        self.get_logger().info(f"Target Network Interface: {self.net_if}")
        self.get_logger().info(f"Subscribing Twist Topic:  {self.cmd_vel_topic}")
        self.get_logger().info(f"Subscribing Mode Topic:   {self.motion_mode_topic}")

        # Initialize Unitree SDK 2 if enabled
        if self.enable_sdk:
            if UNITREE_SDK_AVAILABLE:
                try:
                    ChannelFactoryInitialize(0, self.net_if)
                    self.sport_client = LocoClient()
                    self.sport_client.SetTimeout(0.1)  # Ultra-short RPC timeout (0.1s for real-time responsiveness)
                    self.sport_client.Init()
                    self.sdk_ready = True
                    self.get_logger().info("Successfully initialized Unitree SDK 2 Channel & LocoClient!")

                    if self.auto_stand:
                        self.get_logger().info("Sending BalanceStand command to G1...")
                        self.safe_balance_stand()

                    # Apply initial speed mode and swing height if set
                    if self.current_speed_mode != 0:
                        self.set_speed_mode(self.current_speed_mode)
                    if self.current_swing_height > 0.0:
                        self.set_swing_height(self.current_swing_height)

                except Exception as e:
                    self.get_logger().error(f"Failed to initialize Unitree SDK 2: {e}")
                    self.sdk_ready = False
            else:
                self.get_logger().warn(
                    "unitree_sdk2py library not found on this system. "
                    "Running in SIMULATION / LOG-ONLY mode."
                )
        else:
            self.get_logger().info("SDK disabled via parameter 'enable_sdk: false'. Running in DRY-RUN mode.")

        # State tracking for delta sending
        self.last_sent_vx = 0.0
        self.last_sent_vy = 0.0
        self.last_sent_vyaw = 0.0
        self.last_send_time = 0.0

        # Subscriber with QoS depth=1 for zero queue buffering
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            qos
        )

        # Mode topic subscriber
        self.sub_motion_mode = self.create_subscription(
            String,
            self.motion_mode_topic,
            self.motion_mode_callback,
            10
        )

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Timer loop for heartbeat updates (5Hz = 0.2s)
        timer_period = 1.0 / rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'speed_mode':
                self.set_speed_mode(int(param.value))
            elif param.name == 'swing_height':
                self.set_swing_height(float(param.value))
        return SetParametersResult(successful=True)

    def set_speed_mode(self, mode: int):
        self.current_speed_mode = mode
        mode_desc = "Fast/Run Mode (1)" if mode == 1 else f"Normal/Walk Mode ({mode})"
        self.get_logger().info(f"Setting G1 Speed Mode -> {mode_desc}")
        if self.sdk_ready and self.sport_client is not None:
            try:
                self.sport_client.SetSpeedMode(mode)
            except Exception as e:
                self.get_logger().error(f"Failed to set speed mode via SDK: {e}")

    def set_swing_height(self, height: float):
        self.current_swing_height = height
        self.get_logger().info(f"Setting G1 Swing Height -> {height:.3f} m")
        if self.sdk_ready and self.sport_client is not None:
            try:
                if hasattr(self.sport_client, 'SetSwingHeight'):
                    self.sport_client.SetSwingHeight(height)
                else:
                    import json
                    p = {"data": height}
                    self.sport_client._Call(7103, json.dumps(p))
            except Exception as e:
                self.get_logger().error(f"Failed to set swing height via SDK: {e}")

    def motion_mode_callback(self, msg: String):
        mode_str = msg.data.strip().lower()
        self.get_logger().info(f"Received G1 Motion Mode command: '{mode_str}'")

        if mode_str in ['walk', 'normal', '0']:
            self.set_speed_mode(0)
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.SetBalanceMode(0)
                except Exception as e:
                    self.get_logger().warn(f"SetBalanceMode error: {e}")
        elif mode_str in ['run', 'fast', '1']:
            self.set_speed_mode(1)
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.SetBalanceMode(1)  # Enable Continuous Gait for fast trotting/running
                    self.get_logger().info("Set G1 BalanceMode -> 1 (Continuous Gait / Run Gait active)")
                except Exception as e:
                    self.get_logger().warn(f"SetBalanceMode error: {e}")
            self.get_logger().info("Note: G1 requires forward velocity vx >= 0.8 m/s to trigger physical running stride.")

        elif mode_str in ['stand', 'balance_stand']:
            self.get_logger().info("Executing BalanceStand posture...")
            self.safe_balance_stand()
        elif mode_str in ['high_stand']:
            self.get_logger().info("Executing HighStand posture...")
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.HighStand()
                except Exception as e:
                    self.get_logger().error(f"Failed to execute HighStand: {e}")
        elif mode_str in ['low_stand']:
            self.get_logger().info("Executing LowStand posture...")
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.LowStand()
                except Exception as e:
                    self.get_logger().error(f"Failed to execute LowStand: {e}")
        elif mode_str in ['sit']:
            self.get_logger().info("Executing Sit posture...")
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.Sit()
                except Exception as e:
                    self.get_logger().error(f"Failed to execute Sit: {e}")
        elif mode_str in ['damp']:
            self.get_logger().info("Executing Damp state...")
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.Damp()
                except Exception as e:
                    self.get_logger().error(f"Failed to execute Damp: {e}")
        elif mode_str in ['wave']:
            self.get_logger().info("Executing WaveHand gesture...")
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.WaveHand()
                except Exception as e:
                    self.get_logger().error(f"Failed to execute WaveHand: {e}")
        elif mode_str in ['shake']:
            self.get_logger().info("Executing ShakeHand gesture...")
            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.ShakeHand()
                except Exception as e:
                    self.get_logger().error(f"Failed to execute ShakeHand: {e}")
        else:
            self.get_logger().warn(
                f"Unknown G1 motion mode: '{mode_str}'. "
                "Supported modes: walk, run, stand, high_stand, low_stand, sit, damp, wave, shake"
            )

    def safe_balance_stand(self):

        if self.sport_client is not None:
            try:
                self.sport_client.BalanceStand(0)
            except TypeError:
                try:
                    self.sport_client.BalanceStand()
                except Exception as e:
                    self.get_logger().warn(f"Could not send BalanceStand: {e}")
            except Exception as e:
                self.get_logger().warn(f"Could not send BalanceStand: {e}")

    def cmd_vel_callback(self, msg: Twist):
        self.latest_twist = msg
        self.last_cmd_time = time.time()
        # Immediately dispatch RPC call on incoming velocity message (0ms timer delay)
        self.send_move_cmd_if_needed()

    def control_loop(self):
        # Heartbeat check
        self.send_move_cmd_if_needed(force_heartbeat=True)

    def send_move_cmd_if_needed(self, force_heartbeat=False):
        vx = float(self.latest_twist.linear.x)
        vy = float(self.latest_twist.linear.y)
        vyaw = float(self.latest_twist.angular.z)

        now = time.time()
        is_moving = (abs(vx) > 0.001 or abs(vy) > 0.001 or abs(vyaw) > 0.001)

        vel_changed = (
            abs(vx - self.last_sent_vx) > 0.005 or
            abs(vy - self.last_sent_vy) > 0.005 or
            abs(vyaw - self.last_sent_vyaw) > 0.005
        )
        time_expired = (now - self.last_send_time) > 0.2

        if vel_changed or (force_heartbeat and time_expired):
            self.last_sent_vx = vx
            self.last_sent_vy = vy
            self.last_sent_vyaw = vyaw
            self.last_send_time = now

            if self.sdk_ready and self.sport_client is not None:
                try:
                    self.sport_client.Move(vx, vy, vyaw, continous_move=is_moving)
                except Exception as e:
                    self.get_logger().error(f"Error calling sport_client.Move(): {e}")
            else:
                if is_moving and vel_changed:
                    self.get_logger().info(
                        f"[DRY-RUN G1 Move] vx={vx:+.3f} m/s, vy={vy:+.3f} m/s, vyaw={vyaw:+.3f} rad/s"
                    )



    def stop_robot(self):
        self.get_logger().info("Stopping robot motion & restoring BalanceStand...")
        if self.sdk_ready and self.sport_client is not None:
            try:
                self.sport_client.Move(0.0, 0.0, 0.0, False)
                time.sleep(0.1)
                self.safe_balance_stand()
            except Exception as e:
                self.get_logger().error(f"Error during stop_robot(): {e}")




def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSDKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
