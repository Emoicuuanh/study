#!/usr/bin/env python3
"""
ROS 2 Image Compressor Relay Node for Robot-to-Laptop Streaming
Subscribes to uncompressed /camera/color/image_raw on Robot.
Publishes JPEG compressed sensor_msgs/msg/CompressedImage to /camera/color/image_raw/compressed for ultra-fast Wi-Fi streaming.
Includes real-time FPS & bandwidth debug logging.
"""

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__("realsense_publisher")

        # ROS 2 Parameters for fine-tuning bandwidth & quality
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("output_topic", "/camera/color/compressed")
        self.declare_parameter("jpeg_quality", 75)   # Quality 75% reduces size by ~90% (e.g. 500KB -> 35KB)
        self.declare_parameter("resize_scale", 1.0)  # Scale 1.0 = original 640x480, 0.5 = 320x240
        self.declare_parameter("debug_fps", True)    # Enable FPS debug logs



        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.jpeg_quality = self.get_parameter("jpeg_quality").value
        self.resize_scale = self.get_parameter("resize_scale").value
        self.debug_fps = self.get_parameter("debug_fps").value

        self.bridge = CvBridge()

        # FPS & Debug Statistics Counters
        self.frame_count = 0
        self.total_frames_received = 0
        self.last_fps_time = time.time()
        self.last_frame_received_time = time.time()
        self.last_compressed_size_kb = 0.0
        self.last_raw_size_kb = 0.0
        self.img_shape = (0, 0)

        # High-Speed Sensor QoS Profile:
        # depth=1 with KEEP_LAST & BEST_EFFORT guarantees zero Wi-Fi retransmit storms and lowest latency!
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber & Publisher (Both using BEST_EFFORT for maximum Wi-Fi streaming performance)
        self.sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, sensor_qos
        )
        self.pub = self.create_publisher(
            CompressedImage, self.output_topic, sensor_qos
        )


        # Timer to check if frames are arriving (Health Check every 5.0s)
        self.health_timer = self.create_timer(5.0, self.health_check_callback)


        self.get_logger().info(
            f"Image Compressor Started!\n"
            f"  Subscribing to : [{self.input_topic}]\n"
            f"  Publishing to  : [{self.output_topic}]\n"
            f"  JPEG Quality   : {self.jpeg_quality}%\n"
            f"  Resize Scale   : {self.resize_scale}"
        )

    def image_callback(self, msg: Image):
        current_time = time.time()
        self.last_frame_received_time = current_time
        self.frame_count += 1
        self.total_frames_received += 1

        try:
            # Convert ROS Image msg to OpenCV BGR image
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.img_shape = cv_img.shape[:2]  # (height, width)

            # Calculate raw payload size
            self.last_raw_size_kb = len(msg.data) / 1024.0

            # Optional downscaling for weak Wi-Fi networks
            if self.resize_scale < 1.0 and self.resize_scale > 0.0:
                h, w = self.img_shape
                new_w, new_h = int(w * self.resize_scale), int(h * self.resize_scale)
                cv_img = cv2.resize(cv_img, (new_w, new_h), interpolation=cv2.INTER_AREA)

            # Compress to JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            success, encoded_img = cv2.imencode(".jpg", cv_img, encode_param)

            if not success:
                self.get_logger().warn("Failed to encode frame to JPEG")
                return

            # Construct ROS 2 CompressedImage message
            encoded_bytes = np.array(encoded_img).tobytes()
            self.last_compressed_size_kb = len(encoded_bytes) / 1024.0

            comp_msg = CompressedImage()
            comp_msg.header = msg.header
            comp_msg.format = "jpeg"
            comp_msg.data = encoded_bytes

            self.pub.publish(comp_msg)

            # FPS Debug Logging (Every 1.0 second)
            elapsed = current_time - self.last_fps_time
            if self.debug_fps and elapsed >= 1.0:
                fps = self.frame_count / elapsed
                h, w = self.img_shape
                self.get_logger().info(
                    f"[DEBUG FPS] Receiving {fps:.1f} FPS | Resolution: {w}x{h} | "
                    f"Raw: {self.last_raw_size_kb:.1f} KB -> Compressed: {self.last_compressed_size_kb:.1f} KB "
                    f"(Total frames: {self.total_frames_received})"
                )
                self.frame_count = 0
                self.last_fps_time = current_time

        except Exception as e:
            self.get_logger().error(f"Error compressing image frame: {e}")

    def health_check_callback(self):
        elapsed_since_last_frame = time.time() - self.last_frame_received_time
        if elapsed_since_last_frame >= 3.0:
            self.get_logger().warn(
                f"[WAITING FOR CAMERA] No image received from [{self.input_topic}] in {elapsed_since_last_frame:.1f}s. "
                f"Please ensure camera driver (rs_launch.py) is running!"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
