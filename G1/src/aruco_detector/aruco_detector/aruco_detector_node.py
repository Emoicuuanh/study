#!/usr/bin/env python3
"""
ROS 2 ArUco Marker Detector Node.
Detects ArUco markers, estimates 3D poses using camera intrinsics,
publishes ROS 2 topics (/aruco/markers, /aruco/marker_poses, /aruco/markers_visualization),
and broadcasts TFs for each detected marker.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros

from aruco_detector.msg import ArucoMarker, ArucoMarkerArray
from aruco_detector.aruco_utils import (
    get_aruco_dictionary,
    detect_aruco_markers,
    estimate_marker_pose,
    rotation_matrix_to_quaternion,
    draw_aruco_debug
)


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_detector")

        # Declare ROS 2 Parameters
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("use_camera_info", True)
        self.declare_parameter("marker_size", 0.1)  # 0.1m = 10cm
        self.declare_parameter("aruco_dictionary", "DICT_5X5_100")
        self.declare_parameter("tf_prefix", "aruco_marker_")
        self.declare_parameter("camera_frame_override", "")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_image_result", True)
        self.declare_parameter("publish_rviz_markers", True)
        self.declare_parameter("show_image", False)  # Open OpenCV GUI window for live preview

        # Helper to safely parse string or bool parameters from ROS 2 launch
        def to_bool(val):
            if isinstance(val, bool):
                return val
            if isinstance(val, str):
                return val.lower() in ("true", "1", "yes")
            return bool(val)

        # Get parameter values
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.use_camera_info = to_bool(self.get_parameter("use_camera_info").value)
        self.marker_size = float(self.get_parameter("marker_size").value)
        self.dict_name = str(self.get_parameter("aruco_dictionary").value)
        self.tf_prefix = str(self.get_parameter("tf_prefix").value)
        self.camera_frame_override = str(self.get_parameter("camera_frame_override").value)
        self.publish_tf = to_bool(self.get_parameter("publish_tf").value)
        self.publish_image_result = to_bool(self.get_parameter("publish_image_result").value)
        self.publish_rviz_markers = to_bool(self.get_parameter("publish_rviz_markers").value)
        self.show_image = to_bool(self.get_parameter("show_image").value)

        self.bridge = CvBridge()
        self.dictionary = get_aruco_dictionary(self.dict_name)

        # Camera intrinsic matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame_id = ""

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # QoS configuration for high throughput & reliability compatibility
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Publishers
        self.markers_pub = self.create_publisher(ArucoMarkerArray, "/aruco/markers", 10)
        self.poses_pub = self.create_publisher(PoseArray, "/aruco/marker_poses", 10)

        if self.publish_rviz_markers:
            self.rviz_pub = self.create_publisher(MarkerArray, "/aruco/markers_visualization", 10)

        if self.publish_image_result:
            self.image_pub = self.create_publisher(Image, "/aruco/image_result", sensor_qos)

        # Subscribers
        if self.use_camera_info:
            self.cam_info_sub = self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.camera_info_callback,
                sensor_qos
            )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos
        )

        # Health check & debug frame counter
        import time
        self._last_frame_time = time.time()
        self._has_received_frame = False
        self.health_timer = self.create_timer(5.0, self.health_check_callback)

        self.get_logger().info(
            f"ArUco Detector Node Started!\n"
            f"  Subscribed Image Topic      : [{self.image_topic}]\n"
            f"  Subscribed CameraInfo Topic  : [{self.camera_info_topic}]\n"
            f"  ArUco Dictionary            : {self.dict_name}\n"
            f"  Marker Size                 : {self.marker_size}m\n"
            f"  Publishing TFs              : {self.publish_tf} (Prefix: '{self.tf_prefix}')\n"
            f"  Show GUI Preview Window     : {self.show_image}"
        )

    def health_check_callback(self):
        import time
        elapsed = time.time() - self._last_frame_time
        if not self._has_received_frame or elapsed > 5.0:
            self.get_logger().warn(
                f"[WAITING FOR CAMERA] Chưa nhận được ảnh từ topic [{self.image_topic}] ({elapsed:.1f}s). "
                f"Vui lòng kiểm tra lại node driver camera (Realsense / Unitree camera) đã chạy chưa!"
            )

    def camera_info_callback(self, msg: CameraInfo):
        """Receive and update camera intrinsic parameters."""
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)
        if not self.camera_frame_id:
            self.camera_frame_id = msg.header.frame_id

    def image_callback(self, msg: Image):
        """Process image frame, detect markers, compute 3D poses, and publish topics/TFs."""
        import time
        self._last_frame_time = time.time()
        self._has_received_frame = True

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Conversion Error: {e}")
            return

        frame_id = self.camera_frame_override if self.camera_frame_override else (
            self.camera_frame_id if self.camera_frame_id else msg.header.frame_id
        )

        if not frame_id:
            frame_id = "camera_optical_frame"

        # Detect 2D markers
        corners, ids = detect_aruco_markers(cv_img, self.dictionary)

        marker_array_msg = ArucoMarkerArray()
        marker_array_msg.header = msg.header
        marker_array_msg.header.frame_id = frame_id

        pose_array_msg = PoseArray()
        pose_array_msg.header = msg.header
        pose_array_msg.header.frame_id = frame_id

        rviz_marker_array = MarkerArray()

        valid_rvecs = []
        valid_tvecs = []

        if ids is not None and len(ids) > 0:
            for idx, marker_id_arr in enumerate(ids):
                marker_id = int(marker_id_arr[0])
                corner_pts = corners[idx]

                single_marker_msg = ArucoMarker()
                single_marker_msg.id = marker_id
                single_marker_msg.marker_size = self.marker_size

                # 2D Corner Points
                for c in corner_pts.reshape((4, 2)):
                    pt = Point()
                    pt.x = float(c[0])
                    pt.y = float(c[1])
                    pt.z = 0.0
                    single_marker_msg.corners_2d.append(pt)

                # Pose Estimation (if intrinsics available)
                if self.camera_matrix is not None and self.dist_coeffs is not None:
                    success, rvec, tvec = estimate_marker_pose(
                        corner_pts, self.marker_size, self.camera_matrix, self.dist_coeffs
                    )

                    if success:
                        valid_rvecs.append(rvec)
                        valid_tvecs.append(tvec)

                        # Convert rvec (Rodrigues) -> 3x3 R -> Quaternion
                        R, _ = cv2.Rodrigues(rvec)
                        qx, qy, qz, qw = rotation_matrix_to_quaternion(R)

                        tx, ty, tz = float(tvec[0, 0]), float(tvec[1, 0]), float(tvec[2, 0])

                        pose = Pose()
                        pose.position.x = tx
                        pose.position.y = ty
                        pose.position.z = tz
                        pose.orientation.x = qx
                        pose.orientation.y = qy
                        pose.orientation.z = qz
                        pose.orientation.w = qw

                        single_marker_msg.pose = pose
                        pose_array_msg.poses.append(pose)

                        # Broadcast TF
                        if self.publish_tf:
                            t_tf = TransformStamped()
                            t_tf.header.stamp = msg.header.stamp
                            t_tf.header.frame_id = frame_id
                            t_tf.child_frame_id = f"{self.tf_prefix}{marker_id}"
                            t_tf.transform.translation.x = tx
                            t_tf.transform.translation.y = ty
                            t_tf.transform.translation.z = tz
                            t_tf.transform.rotation.x = qx
                            t_tf.transform.rotation.y = qy
                            t_tf.transform.rotation.z = qz
                            t_tf.transform.rotation.w = qw
                            self.tf_broadcaster.sendTransform(t_tf)

                        # Build RViz visualization markers
                        if self.publish_rviz_markers:
                            # 3D Cube Marker
                            cube_marker = Marker()
                            cube_marker.header = msg.header
                            cube_marker.header.frame_id = frame_id
                            cube_marker.ns = "aruco_cubes"
                            cube_marker.id = marker_id
                            cube_marker.type = Marker.CUBE
                            cube_marker.action = Marker.ADD
                            cube_marker.pose = pose
                            cube_marker.scale.x = self.marker_size
                            cube_marker.scale.y = self.marker_size
                            cube_marker.scale.z = 0.005
                            cube_marker.color.r = 0.0
                            cube_marker.color.g = 0.8
                            cube_marker.color.b = 1.0
                            cube_marker.color.a = 0.8
                            rviz_marker_array.markers.append(cube_marker)

                            # Text Label Marker
                            text_marker = Marker()
                            text_marker.header = msg.header
                            text_marker.header.frame_id = frame_id
                            text_marker.ns = "aruco_text"
                            text_marker.id = marker_id + 10000
                            text_marker.type = Marker.TEXT_VIEW_FACING
                            text_marker.action = Marker.ADD
                            text_marker.pose = pose
                            text_marker.pose.position.z += (self.marker_size * 0.8)
                            text_marker.scale.z = 0.05  # Text height
                            text_marker.color.r = 1.0
                            text_marker.color.g = 1.0
                            text_marker.color.b = 1.0
                            text_marker.color.a = 1.0
                            text_marker.text = f"ID: {marker_id}"
                            rviz_marker_array.markers.append(text_marker)

                marker_array_msg.markers.append(single_marker_msg)

        # Publish topics
        self.markers_pub.publish(marker_array_msg)
        self.poses_pub.publish(pose_array_msg)

        if self.publish_rviz_markers and len(rviz_marker_array.markers) > 0:
            self.rviz_pub.publish(rviz_marker_array)

        # Publish debug image / Show GUI window
        if self.publish_image_result or self.show_image:
            debug_img = draw_aruco_debug(
                cv_img,
                corners,
                ids,
                self.camera_matrix,
                self.dist_coeffs,
                valid_rvecs,
                valid_tvecs,
                self.marker_size
            )
            if self.publish_image_result:
                try:
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                    debug_msg.header = msg.header
                    debug_msg.header.frame_id = frame_id
                    self.image_pub.publish(debug_msg)
                except Exception as e:
                    self.get_logger().error(f"Failed to publish debug image: {e}")

            if self.show_image:
                cv2.imshow("ArUco Detector Live Preview", debug_img)
                cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
