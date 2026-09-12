# Hướng dẫn Thiết kế và Triển khai Chi tiết Hệ thống Navigation (Nav2) cho Robot Unitree G1

Tài liệu này chứa toàn bộ thiết kế kiến trúc, mã nguồn các file cấu hình, file launch và kịch bản chi tiết để xây dựng hệ thống dẫn đường (Navigation) tự động né tránh vật cản cho robot Unitree G1.

---

## 1. Nguyên lý Hoạt động của Hệ thống

Hệ thống dẫn đường sử dụng bộ khung **Nav2 (Navigation 2)** của ROS 2 kết hợp với dữ liệu định vị từ **NDT Localization** và chướng ngại vật từ **Voxblox/LiDAR 3D**:

```
 ┌───────────┐      ┌─────────────────────────┐
 │   RViz2   ├─────►│  BT Navigator (Behavior) │
 └───────────┘      └────────────┬────────────┘
                                 │
                 ┌───────────────┴───────────────┐
                 ▼                               ▼
      ┌────────────────────┐          ┌────────────────────┐
      │   Global Planner   │          │  Local Controller  │
      │   (NavFn / A*)     │          │  (DWB Holonomic)   │
      └──────────▲─────────┘          └──────────▲─────────┘
                 │                               │
       ┌─────────┴────────┐            ┌─────────┴────────┐
       │  Global Costmap  │            │  Local Costmap   │
       │ (Static 2D Map)  │            │ (ESDF + LiDAR3D) │
       └─────────▲────────┘            └─────────▲────────┘
                 │                               │
          ┌──────┴──────┐                ┌───────┴───────┐
          │ Map Server  │                │ /esdf_slice   │
          │  (2D YAML)  │                │ /registered   │
          └─────────────┘                └───────────────┘
```

1.  **Lập kế hoạch toàn cục (Global Plan):** Bộ nạp bản đồ (`map_server`) nạp bản đồ tĩnh 2D (đã chuyển đổi từ bản đồ 3D `.pcd`). `Global Planner` tính toán đường đi dài tối ưu dạng các tọa độ liên tục nối từ vị trí hiện tại đến đích.
2.  **Tránh vật cản thời gian thực (Local Avoidance):** `Local Costmap` dựng một lưới ô cờ nhỏ (ví dụ $3\text{m} \times 3\text{m}$) di chuyển theo robot. Lưới này gộp dữ liệu từ **lát cắt vật cản sát đất (ESDF slice)** của Voxblox và **mây điểm thô (LiDAR 3D)** để xác định vật cản tĩnh và động xung quanh chân robot.
3.  **Tính toán lệnh vận tốc:** `Local Controller` (thuật toán DWB) chạy ở chế độ **Holonomic** (chuyển động đa hướng) để tính toán ra vận tốc tiến/lùi ($v_x$), đi ngang ($v_y$) và tốc độ xoay góc ($\omega_z$) an toàn nhất, xuất ra topic `/cmd_vel`.

---

## 2. Các File Cần Thiết lập trong Hệ thống

Để triển khai hệ thống dẫn đường, ta sẽ tạo một package chuyên dụng mang tên `g1_navigation_nav2` với cấu trúc như sau:

```text
src/g1_navigation_nav2/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── nav2_params.yaml          # Định cấu hình toàn bộ tham số Nav2
└── launch/
    └── g1_navigation_nav2.launch.py   # Launch file khởi chạy toàn bộ Nav2
```

### 2.1. File `package.xml`
Khai báo các gói phụ thuộc cần thiết cho Nav2:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>g1_navigation_nav2</name>
  <version>1.0.0</version>
  <description>Navigation package for Unitree G1 humanoid robot</description>
  <maintainer email="hoangdc@todo.todo">hoangdc</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>nav2_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>

  <exec_depend>nav2_bringup</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_recoveries</exec_depend>
  <exec_depend>nav2_bt_navigator</exec_depend>
  <exec_depend>nav2_waypoint_follower</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav2_lifecycle_manager</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 2.2. File `CMakeLists.txt`
Chỉ thị biên dịch và cài đặt các tài nguyên cấu hình, launch:

```cmake
cmake_minimum_required(VERSION 3.5)
project(g1_navigation_nav2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

---

## 3. Bản thiết kế File Cấu hình `config/nav2_params.yaml`

Đây là file tham số trung tâm. Được thiết kế đặc biệt cho robot **Unitree G1** để tối ưu hóa di chuyển đa hướng (Holonomic) và nhận dạng vật cản từ 2 nguồn:

```yaml
amcl:
  ros__parameters:
    use_sim_time: false

bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_frame: base_link
    odom_frame: odom
    default_bt_xml_filename: "/opt/ros/foxy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node

controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Cấu hình Progress Checker
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    # Cấu hình Goal Checker (Kiểm tra tới đích)
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.15          # Sai số khoảng cách đích (15cm)
      yaw_goal_tolerance: 0.20         # Sai số góc quay đích (~11 độ)
      stateful: true

    # Cấu hình DWB Local Controller (Chế độ Đa Hướng - Holonomic)
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      debug_trajectory_details: false
      min_vel_x: -0.2                  # Tốc độ lùi tối đa (m/s)
      max_vel_x: 0.4                   # Tốc độ tiến tối đa (m/s)
      min_vel_y: -0.2                  # Tốc độ đi ngang trái (m/s)
      max_vel_y: 0.2                   # Tốc độ đi ngang phải (m/s)
      max_vel_trans: 0.4               # Tổng vận tốc di chuyển tối đa
      min_vel_trans: 0.01
      max_vel_theta: 0.6               # Tốc độ xoay tối đa (rad/s)
      min_vel_theta: -0.6
      acc_lim_x: 0.8                   # Giới hạn gia tốc để tránh ngã robot
      acc_lim_y: 0.8
      acc_lim_theta: 1.5
      decel_lim_x: -0.8
      decel_lim_y: -0.8
      decel_lim_theta: -1.5

      # Các hệ số chấm điểm quỹ đạo (Trajectory Scoring)
      critics: [
        "RotateToGoalDist",
        "Oscillation",
        "BaseObstacle",
        "GoalAlign",
        "PathAlign",
        "PathDist",
        "GoalDist"
      ]
      BaseObstacle.scale: 0.02
      PathAlign.scale: 32.0
      GoalAlign.scale: 24.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      Oscillation.scale: 1.0

      # Số lượng mẫu thử vận tốc để tính toán
      vx_samples: 20
      vy_samples: 20
      vtheta_samples: 20
      sim_time: 1.7

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: false
      robot_radius: 0.28               # Bán kính an toàn của G1 (rộng hơn vai thực tế)
      obstacle_max_range: 3.0
      obstacle_min_range: 0.0
      publish_frequency: 1.0
      update_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      resolution: 0.05
      transform_tolerance: 1.5         # Dung sai tra cứu TF từ NDT (1Hz)
      always_send_full_costmap: True   # Luôn xuất bản bản đồ đầy đủ để RViz không bị trắng
      plugins: ["static_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.6            # Khoảng cách thổi phồng an toàn xung quanh tường

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: false
      robot_radius: 0.28
      obstacle_max_range: 3.0
      obstacle_min_range: 0.0
      publish_frequency: 5.0
      update_frequency: 5.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 4.0                       # Kích thước bản đồ cục bộ trượt (4m x 4m)
      height: 4.0
      resolution: 0.05
      transform_tolerance: 0.5
      always_send_full_costmap: True
      plugins: ["obstacle_layer", "inflation_layer"]

      # Cấu hình đa cảm biến nhận dạng chướng ngại vật
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: voxblox_esdf_slice lidar_3d_high
        
        # Nguồn 1: Lát cắt chướng ngại vật thấp từ Voxblox (sát đất)
        voxblox_esdf_slice:
          topic: /voxblox_local/esdf_slice
          sensor_frame: odom
          data_type: PointCloud2
          clearing: true
          marking: true
          max_obstacle_height: 0.20
          min_obstacle_height: -0.10

        # Nguồn 2: Mây điểm LiDAR 3D đồng bộ để tránh chướng ngại vật cao / thanh ngang người
        lidar_3d_high:
          topic: /utlidar/cloud_livox_mid360_sync
          sensor_frame: livox_frame
          data_type: PointCloud2
          clearing: true
          marking: true
          max_obstacle_height: 1.30      # Giới hạn chiều cao đỉnh đầu G1
          min_obstacle_height: 0.15      # Bỏ qua mặt sàn phẳng để tránh nhiễu

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 4.0
        inflation_radius: 0.5

planner_server:
  ros__parameters:
    use_sim_time: false
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavFnPlanner"
      tolerance: 0.1
      use_astar: true                  # Sử dụng thuật toán A*

recoveries_server:
  ros__parameters:
    use_sim_time: false
    recovery_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_recoveries/Spin"
    backup:
      plugin: "nav2_recoveries/BackUp"
    wait:
      plugin: "nav2_recoveries/Wait"
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: false
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

---

## 4. File Launch Khởi chạy: `launch/g1_navigation_nav2.launch.py`

File launch này chịu trách nhiệm khởi chạy toàn bộ các thành phần của Nav2 và quản lý vòng đời (lifecycle manager) của chúng:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Định nghĩa các đường dẫn thư mục share
    nav_share_dir = get_package_share_directory('g1_navigation_nav2')
    
    # 2. Định nghĩa các file cấu hình mặc định
    default_params_file = os.path.join(nav_share_dir, 'config', 'nav2_params.yaml')
    default_map_file = '/home/hoangdc/ROS2/unitree_G1/maps/g1_office.yaml'

    # 3. Khai báo các tham số Launch
    map_yaml_file = LaunchConfiguration('map', default=default_map_file)
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    autostart = LaunchConfiguration('autostart', default='true')

    # Danh sách các node cần quản lý vòng đời
    lifecycle_nodes = [
        'map_server',
        'planner_server',
        'controller_server',
        'recoveries_server',
        'bt_navigator'
    ]

    # 4. Tạo các hành động khởi chạy Nodes
    
    # Node nạp bản đồ 2D Gridmap tĩnh
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}]
    )

    # Node lập kế hoạch toàn cục
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # Node tính toán bám đường đi và sinh ra cmd_vel
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/cmd_vel_nav')] # Đưa ra kênh Nav
    )

    # Node cứu hộ
    recoveries_server_node = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[params_file]
    )

    # Node quản lý hành vi cao cấp (Behavior Tree)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    # Node quản lý vòng đời hoạt động (Active/Inactive) của toàn hệ thống Nav2
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ]
    )

    return LaunchDescription([
        # Khai báo biến môi trường đầu ra log
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        DeclareLaunchArgument('map', default_value=default_map_file, description='Full path to map yaml file to load'),
        DeclareLaunchArgument('params_file', default_value=default_params_file, description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack'),

        map_server_node,
        planner_server_node,
        controller_server_node,
        recoveries_server_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])
```

---

## 5. Quy trình Chạy thử nghiệm và Căn chỉnh Thực tế (Tuning & Troubleshooting)

### 5.1. Khởi chạy Hệ thống dẫn đường
1.  Khởi chạy Robot G1 Bringup và LiDAR.
2.  Khởi chạy hệ thống định vị NDT Localization.
3.  Trong một Terminal mới, chạy lệnh khởi tạo Nav2:
    ```bash
    ros2 launch g1_navigation_nav2 g1_navigation_nav2.launch.py
    ```

### 5.2. Điều khiển trên RViz2
1.  Bật giao diện RViz2.
2.  Thêm hiển thị **Map** (Topic: `/map`), **Path** (Topic: `/plan`) để xem quỹ đạo di chuyển.
3.  Thêm hiển thị **Costmap** (Topic: `/local_costmap/costmap` và `/global_costmap/costmap`) để quan sát vùng an toàn bao quanh các chướng ngại vật.
4.  Dùng công cụ **2D Goal Pose** nhấp chuột trên bản đồ để chỉ định điểm đích cần đến cho robot.
5.  Theo dõi output của topic `/cmd_vel_nav` để kiểm tra các giá trị điều khiển:
    ```bash
    ros2 topic echo /cmd_vel_nav
    ```

### 5.3. Căn chỉnh thông số tránh ngã (Safety Tuning)
Đối với robot humanoid như G1, quán tính cơ thể rất lớn, tăng tốc hay dừng quá đột ngột sẽ khiến robot bị ngã chúi đầu (faceplant) hoặc ngã ngửa:
*   **Nếu robot phanh gấp quá bị chúi đầu:** Hãy giảm giá trị hãm phanh tối đa `decel_lim_x` từ `-0.8` xuống `-0.4` hoặc `-0.5` trong cấu hình `FollowPath`.
*   **Nếu robot đi quá sát vật cản:** Hãy tăng tham số `inflation_radius` trong `local_costmap` từ `0.5` lên `0.6` hoặc `0.7` để ép robot đi xa vật cản hơn.
*   **Nếu robot không lách qua được các khe cửa nhỏ:** Hãy giảm nhẹ `robot_radius` xuống khoảng `0.25` (phải đảm bảo tốc độ của robot được kiểm soát rất chậm khi đi qua khe hẹp để tránh va chạm).
