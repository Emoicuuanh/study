# Kế Hoạch Triển Khai Master (Master Execution Plan)

# Điều Khiển Bước Chân (Footstep Navigation) Dùng Whole-Body Control (WBC) Ở Chế Độ Develop Mode Cho Unitree G1

- **Robot Model:** Unitree G1 Humanoid
- **ROS Version:** ROS 2 Foxy Fitzroy (Ubuntu 20.04 LTS)
- **C++ Standard:** C++17
- **Giao thức điều khiển:** Unitree SDK 2 Low-Level DDS (`rt/lowcmd` & `rt/lowstate` @ 500Hz)
- **Kiến trúc chính:** SLAM / Nav2 $\rightarrow$ Footstep Planner $\rightarrow$ Walking Pattern Generator (LIPM + Bézier) $\rightarrow$ Hierarchical QP WBC $\rightarrow$ Unitree LowCmd DDS

---

## 1. Sơ Đồ Kiến Trúc Dạng ROS 2 Node Graph (Rõ Ràng & Dễ Quan Sát)

```mermaid
graph TD
    %% Node 1: Input & Cảm biến
    N1["<b>[1] RVIZ2 & CẢM BIẾN</b><br/>• Node: /rviz2 & /livox_ros_driver2<br/>• Input: 2D Nav Goal & PointCloud2 3D"]

    %% Node 2: SLAM & Costmaps
    N2["<b>[2] SLAM & COSTMAPS</b><br/>• Node: /g1_localization_node (FAST-LIO)<br/>• Node: /local_costmap_node<br/>• Output: Pose TF (map -> odom) & Map vật cản"]

    %% Node 3: Global Planner
    N3["<b>[3] GLOBAL PLANNER</b><br/>• Node: /planner_server (Nav2)<br/>• Output: Đường đi tổng thể /plan (nav_msgs/Path)"]

    %% Node 4: Footstep Planner & Né vật cản
    N4["<b>[4] FOOTSTEP PLANNER (NÉ VẬT CẢN)</b><br/>• Node: /footstep_planner_action_node<br/>• Chức năng: Chia dấu chân & tính lại né vật cản @ 5Hz<br/>• Output: /g1_footstep_plan (unitree_msgs/FootstepPlan)"]

    %% Node 5: WPG & WBC Controller
    N5["<b>[5] WHOLE-BODY CONTROLLER (WBC 500Hz)</b><br/>• Node: /g1_wbc_walk_node<br/>• Chức năng: WPG (LIPM + Bézier) + HQP QP Solver<br/>• Output: Lệnh mô-mơ & góc khớp qua DDS rt/lowcmd"]

    %% Node 6: Hardware / Sim & Feedback Loop
    N6["<b>[6] ROBOT THỰC THI & FEEDBACK</b><br/>• Component: Unitree G1 Hardware / MuJoCo Sim<br/>• Feedback: LowState (IMU, Encoders, Contact) @ 500Hz qua rt/lowstate"]

    %% Connections
    N1 -->|1. Goal Pose & Pointcloud| N2
    N2 -->|2. Position TF & Costmap| N3
    N3 -->|3. Global Path| N4
    N2 -->|4. Local Costmap (Né vật cản)| N4
    N4 -->|5. Danh sách dấu chân| N5
    N5 -->|6. LowCmd DDS (500Hz)| N6
    N6 -.->|7. Vòng phản hồi LowState IMU/Joints (500Hz)| N5
    N6 -.->|8. Phản hồi Pose / Odom| N2

    %% Styling
    style N1 fill:#e0f7fa,stroke:#006064,stroke-width:2px
    style N2 fill:#fff3e0,stroke:#e65100,stroke-width:2px
    style N3 fill:#fff8e1,stroke:#f57f17,stroke-width:2px
    style N4 fill:#ede7f6,stroke:#4a148c,stroke-width:2px
    style N5 fill:#e8f5e9,stroke:#1b5e20,stroke-width:3px
    style N6 fill:#ffebee,stroke:#b71c1c,stroke-width:2px
```

---

## 2. Quy Trình 6 Giai Đoạn Triển Khai Chi Tiết (Phase 1 - Phase 6)

### PHASE 1: Cài Đặt Môi Trường, Thư Viện C++ & Mô Phỏng MuJoCo

#### 1.1 Cài đặt thư viện phụ thuộc cho ROS 2 Foxy

- **Pinocchio (Rigid Body Dynamics):**
  ```bash
  sudo apt update
  sudo apt install ros-foxy-pinocchio
  ```
- **ProxSuite / qpOASES (Real-time QP Solver):**
  ```bash
  sudo apt install libproxsuite-dev || sudo apt install robotpkg-qpOASES
  ```

#### 1.2 Thiết lập Môi trường Mô phỏng MuJoCo

- Nạp mô hình URDF/MJCF của G1 từ package `g1_description`.
- Xây dựng script kiểm thử mô phỏng C++/Python kết nối qua DDS channel `rt/lowcmd` và `rt/lowstate`.

#### 1.3 Kiểm tra Động lực học với Pinocchio

- Viết script C++ load URDF `g1.urdf` qua Pinocchio để kiểm tra Forward Kinematics (FK), ma trận khối lượng $M(q)$, và các Ma trận Jacobian $J_L, J_R$ của 2 chân.

---

### PHASE 2: Custom Messages & Nâng Cấp `g1_footstep_planner`

#### 2.1 Tạo Custom Messages (Trong `unitree_msgs/msg/`)

- **`Footstep.msg`:**
  ```text
  uint8 LEFT=0
  uint8 RIGHT=1

  uint8 foot_side              # Chân trái (0) hoặc chân phải (1)
  geometry_msgs/Pose pose      # Tọa độ 3D bước chân (x, y, z, yaw) trong odom frame
  float64 step_duration        # Thời gian thực hiện bước (vd: 0.4s)
  float64 swing_height         # Độ cao nhấc chân lăng (vd: 0.08m)
  ```
- **`FootstepPlan.msg`:**
  ```text
  std_msgs/Header header
  unitree_msgs/Footstep[] footsteps
  ```

#### 2.2 Nâng cấp Action Node Footstep Planner

- File sửa đổi: `src/g1_footstep_planner/src/footstep_planner_action_node.cpp`
- Nhận đường đi `nav_msgs/Path` từ Nav2.
- Chia nhỏ thành danh sách các pose bước chân $S_0, S_1, \dots, S_N$.
- Publish message `FootstepPlan` lên topic `/g1_footstep_plan`.

---

### PHASE 3: Phát Triển Package `g1_wbc_controller` & Bài Test Đứng Thăng Bằng (Balance Stand)

#### 3.1 Cấu trúc Package C++ `g1_wbc_controller`

Tạo package mới tại `src/g1_wbc_controller` với cấu trúc file:

- `include/g1_wbc_controller/pinocchio_model_handler.hpp`
- `include/g1_wbc_controller/hqp_wbc_solver.hpp`
- `include/g1_wbc_controller/low_level_dds_bridge.hpp`
- `src/pinocchio_model_handler.cpp`
- `src/hqp_wbc_solver.cpp`
- `src/low_level_dds_bridge.cpp`
- `src/g1_wbc_stand_node.cpp`

#### 3.2 Lập trình HQP WBC Solver (500Hz Loop) - Chế độ Đứng 2 Chân

* **Input:** Góc khớp $q, \dot{q}$ và dữ liệu IMU (Roll, Pitch, Yaw, Gyro) từ `rt/lowstate`.
* **Cấu trúc bài toán QP:**
  - **Ràng buộc cứng (Equality Constraints):**
    - Phương trình động lực học: $M(q)\ddot{q} + h(q,\dot{q}) = S^T \tau + J_L^T f_L + J_R^T f_R$
    - Chân cố định trên sàn: $J_L \ddot{q} + \dot{J}_L \dot{q} = 0$, $J_R \ddot{q} + \dot{J}_R \dot{q} = 0$
  - **Ràng buộc bất đẳng thức (Inequality Constraints):**
    - Nón ma sát (Friction Cone): $|f_{x,y}| \le \mu f_z$ và $f_z \ge 0$.
    - Giới hạn mô-men khớp: $\tau_{min} \le \tau \le \tau_{max}$.
  - **Task Priorities:**
    1. *Task 1:* Cân bằng Torso dựa trên phản hồi IMU.
    2. *Task 2:* Giữ tư thế mặc định cho thân trên và tay.
* **Output:** $\tau_{ff}, q_d, \dot{q}_d, K_p, K_d$ gửi qua `rt/lowcmd`.

#### 3.3 Chạy Test Phase 3 trên Mô phỏng MuJoCo & Robot Thật G1

- **MuJoCo Sim:** Test thăng bằng 60 giây, dùng chuột đẩy lực ngang (Disturbance Test).
- **Robot Thật G1 (Treo dây cáp an toàn):** Chạy `g1_wbc_stand_node` ở Develop Mode. Đẩy nhẹ kiểm tra phản hồi lực thăng bằng.

---

### PHASE 4: Xây Dựng Walking Pattern Generator (LIPM + Bézier Curve)

#### 4.1 Cấu tạo Lớp `walking_pattern_generator.cpp`

- **LIPM CoM Planner:**
  - Nhận `FootstepPlan`.
  - Tính toán ZMP tham chiếu $p_{zmp}(t)$ và giải phương trình LIPM: $\ddot{x}_{com} = \frac{g}{z_0}(x_{com} - p_{zmp})$ để tạo đường cong Khối tâm (CoM) mượt mà.
- **Swing Foot Trajectory Generator:**
  - Tạo đường cong 3D dạng **Bézier Polynomial 5th-order** cho chân lăng:
    * $t = 0$: Chân ở mặt đất ($z = 0, v = 0$).
    * $t = T_{step}/2$: Chân ở đỉnh cao nhất ($z = h_{swing}$).
    * $t = T_{step}$: Chân đáp xuống đất ($z = 0, v = 0$).

---

### PHASE 5: Tích Hợp Dynamic Walking Vào WBC (`g1_wbc_walk_node`)

#### 5.1 Xây dựng State Machine Chu kỳ Bước Đi

1. **Double Support Phase (0.1s):** Phân bổ trọng lượng 50%-50%.
2. **Weight Shift Phase (0.05s):** Dồn ZMP về chân trụ.
3. **Single Support Phase (0.4s):**
   - Chân trụ: Chịu 100% trọng lượng + ràng buộc nón ma sát.
   - Chân lăng: Thêm **Swing Foot Task** vào QP Solver bám theo đường cong Bézier.

#### 5.2 Kiểm thử Dynamic Walking trong MuJoCo

- Gửi lệnh phát 10 bước tiến thẳng và 5 bước xoay góc.
- Kiểm tra lực tiếp xúc $f_z$ chân lăng bằng $0$ khi nhấc lên.

---

### PHASE 6: Tích Hợp Hệ Thống End-to-End Navigation & Robot Thật

#### 6.1 Tích hợp Launch File tổng hợp (`g1_wbc_navigation.launch.py`)

1. `FAST_LIO_ROS2` / `g1_localization`: Khởi chạy SLAM & Định vị Pose.
2. `g1_navigation_nav2`: Tạo Global Path 2D.
3. `g1_footstep_planner`: Tạo `FootstepPlan`.
4. `g1_wbc_walk_node`: Chạy WPG + WBC QP Solver @ 500Hz $\rightarrow$ Unitree `LowCmd`.

#### 6.2 Quy trình Thử nghiệm Thực tế trên G1 (Hardware Deployment)

1. Đeo cáp treo an toàn cho G1.
2. Chạy `g1_wbc_stand_node` (Kiểm tra đứng thăng bằng).
3. Đặt 2D Goal Pose trên RViz2 và quan sát G1 di chuyển theo từng bước chân.

---

## 3. Tiêu Chí Nghiệm Thu (Acceptance Criteria)

| Phase             | Môi trường     | Bài Test Đạt Yêu Cầu (Pass Criteria)                                                     |
| :---------------- | :---------------- | :-------------------------------------------------------------------------------------------- |
| **Phase 1** | Terminal / C++    | Load URDF G1 vào Pinocchio không lỗi; MuJoCo render robot G1 chuẩn.                       |
| **Phase 2** | ROS 2 Foxy        | Node`g1_footstep_planner` xuất ra topic `/g1_footstep_plan` danh sách Pose chuẩn xác. |
| **Phase 3** | MuJoCo & G1 Thật | Robot đứng 2 chân ổn định ở Low-Level Mode; bị đẩy nhẹ không ngã.                |
| **Phase 4** | C++ Unit Test     | Đồ thị CoM & Bézier trajectory mượt mà, vận tốc không bị giật.                    |
| **Phase 5** | MuJoCo Sim        | Robot bước đi liên tục 10m trong mô phỏng không sập, chạm đất mượt.             |
| **Phase 6** | Hardware G1       | Robot di chuyển tự động từ A đến B theo vị trí chỉ định trên RViz2 thành công. |
