# ArUco Detector ROS 2 Package

A ROS 2 package for detecting ArUco markers, estimating 3D poses, publishing ROS 2 topics (`/aruco/markers`, `/aruco/marker_poses`, `/aruco/markers_visualization`), and broadcasting 3D Transform Frames (TF) for each detected marker.

## Features

- **ArUco Detection**: Utilizes OpenCV 4.x (`cv2.aruco.ArucoDetector`) to identify markers across configurable dictionaries.
- **3D Pose Estimation**: Computes position $(X, Y, Z)$ and orientation $(q_x, q_y, qz, q_w)$ using `cv2.solvePnP` and camera calibration parameters.
- **Custom ROS 2 Message**: `ArucoMarker` and `ArucoMarkerArray` containing marker IDs, 3D poses, sizes, and 2D image coordinates.
- **Standard Topics**:
  - `/aruco/markers` (`aruco_detector/msg/ArucoMarkerArray`)
  - `/aruco/marker_poses` (`geometry_msgs/msg/PoseArray`)
  - `/aruco/markers_visualization` (`visualization_msgs/msg/MarkerArray` for RViz rendering)
  - `/aruco/image_result` (`sensor_msgs/msg/Image` with overlay 2D borders & 3D axes)
- **TF Broadcast**: Broadcasts frame `camera_frame` -> `aruco_marker_<id>` via `tf2_ros`.

---

## Installation & Build

From your ROS 2 workspace root (`dev_unitreeg1_ws`):

```bash
cd /home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws
colcon build --packages-select aruco_detector
source install/setup.bash
```

---

## How to Run

### Launching with default parameters:
```bash
ros2 launch aruco_detector aruco_detector.launch.py
```

### Launching with custom topic and marker size:
```bash
ros2 launch aruco_detector aruco_detector.launch.py image_topic:=/camera/color/image_raw marker_size:=0.05
```

---

## ROS 2 Topics & Interfaces

| Topic | Message Type | Description |
|---|---|---|
| `/aruco/markers` | `aruco_detector/msg/ArucoMarkerArray` | Full detection data (IDs, 3D Poses, 2D Corners) |
| `/aruco/marker_poses` | `geometry_msgs/msg/PoseArray` | 3D Poses of detected markers in camera frame |
| `/aruco/markers_visualization` | `visualization_msgs/msg/MarkerArray` | 3D Cubes & ID Labels for RViz visualization |
| `/aruco/image_result` | `sensor_msgs/msg/Image` | Processed camera image with drawn axes and IDs |
| `TF` | `tf2_msgs/msg/TFMessage` | Broadcast transform `camera_frame` -> `aruco_marker_<id>` |

---

## Parameters Config (`config/aruco_params.yaml`)

```yaml
aruco_detector:
  ros__parameters:
    image_topic: "/camera/color/image_raw"
    camera_info_topic: "/camera/color/camera_info"
    use_camera_info: true
    marker_size: 0.05           # Size of marker side in meters
    aruco_dictionary: "DICT_5X5_100"
    tf_prefix: "aruco_marker_"
    publish_tf: true
    publish_image_result: true
    publish_rviz_markers: true
```
