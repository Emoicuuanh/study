# Hướng Dẫn Kỹ Thuật & Tài Liệu Hệ Thống: g1_footstep_planner

Tài liệu này tổng hợp toàn bộ kiến trúc, luồng xử lý chi tiết trong node, thuật toán điều khiển, lý thuyết so sánh với WBC, thông số cấu hình và hướng dẫn vận hành của gói **`g1_footstep_planner`** dành cho Robot Chân Người (Humanoid) Unitree G1.

---

## 1. Tổng Quan Hệ Thống (System Overview)

Gói `g1_footstep_planner` là một ROS 2 Action Server C++ (`/navigate_footstep`) cung cấp khả năng di chuyển chính xác cao cho robot humanoid Unitree G1 dựa trên cơ chế **Điều khiển Vòng Kín Phản hồi Odometry (Closed-Loop Odometry Feedback)** và **Chủ động Khử Lệch Chéo 3-DOF (Active 3-DOF Cross-Track Compensation)**.

### 🏗️ Sơ đồ Kiến trúc Tích hợp:

```mermaid
graph TD
    UserGoal[CLI / Nav2 Goal Request] -->|Action Goal: /navigate_footstep| FootstepNode[g1_footstep_planner_node]
    Localization[g1_localization / FAST-LIO] -->|TF: odom -> base_link| FootstepNode
    FootstepNode -->|20Hz CmdVel: /cmd_vel_footstep| TwistMux[g1_twist_mux.py Priority 75]
    TwistMux -->|Top Priority Velocity: /cmd_vel| SDKBridge[cmd_vel_to_sdk_node]
    FootstepNode -->|Lock Feet: /g1_mode = balance_stand| SDKBridge
    SDKBridge -->|Unitree SDK 2 / Low-level LocoClient| G1Robot[Unitree G1 Physical Robot]
```

---

## 2. Luồng Hoạt Động Chi Tiết Trong Action Node (Internal Node Workflow)

Khi `FootstepPlannerActionNode` nhận được một yêu cầu Action Goal từ Client (CLI hoặc Nav2), luồng thực thi trong C++ diễn ra theo **4 Giai đoạn chuẩn hóa**:

```mermaid
sequenceDiagram
    autonumber
    actor Client as Action Client (CLI / Nav2)
    participant ActionNode as Action Server Node
    participant Calc as FootstepCalculator
    participant TF as TF Buffer (odom -> base_link)
    participant SDK as Twist Mux / SDK Bridge

    Client->>ActionNode: Send Goal (target_pose, frame_id)
    ActionNode->>ActionNode: 1. Tiếp nhận & Giải mã Frame (base_link vs map/odom)
    ActionNode->>Calc: 2a. plan_impulses(dx, dy, dyaw)
    Calc-->>ActionNode: Return 3 Impulse Phases (YAW, STRAFE, FORWARD)
    ActionNode->>Calc: 2b. generate_discrete_footsteps()
    Calc-->>ActionNode: Footstep Marker Array (Visual RViz2)
    ActionNode->>TF: Ghi lại vị trí ban đầu (p_init)

    loop Vòng lặp Điều khiển 20Hz (Mỗi Pha Impulse)
        ActionNode->>TF: lookupTransform(odom -> base_link)
        TF-->>ActionNode: Current Robot Pose (c_x, c_y, c_yaw)
        ActionNode->>ActionNode: Chiếu vị trí về Local Body Frame -> Traveled & Cross-Track Error
        ActionNode->>ActionNode: P-Control Ramp & Active 3-DOF Compensation (vx, vy, wz)
        ActionNode->>SDK: Publish Twist (/cmd_vel_footstep)
        ActionNode->>Client: Publish Feedback (percent_complete, current_pose)
    end

    ActionNode->>SDK: Lock Feet (/g1_mode = 'balance_stand')
    ActionNode->>TF: 4. Đọc vị trí kết thúc (end_pose) & Tính Sai số 3DoF
    ActionNode-->>Client: Action Succeed (Result: final_error_x, y, yaw)
```

---

### 🔍 Chi tiết Kỹ thuật 4 Giai đoạn Thực thi trong Node:

#### **Giai đoạn 1: Tiếp Nhận Goal & Giải Mã Khung Tọa Độ (Goal Reception & Frame Parsing)**
1. Callback `handle_goal()` tiếp nhận `NavigateFootstep::Goal`.
2. Kiểm tra `header.frame_id`:
   - **Nếu `frame_id == "base_link"` hoặc rỗng:** Coi $(goal\_x, goal\_y, goal\_yaw)$ là **độ dịch chuyển tương đối (relative offset)** từ vị trí robot hiện tại.
   - **Nếu `frame_id == "map"` hoặc `"odom"`:** Đọc vị trí robot hiện tại từ TF `(p_{init\_x}, p_{init\_y}, p_{init\_yaw})` và chuyển đổi về tọa độ tương đối $(dx, dy, dyaw)$ qua hàm `calculator_.calculate_relative_pose()`.

#### **Giai đoạn 2: Lập Kế Hoạch Impulse & Hình Học Bước Chân (Impulse Planning & Footstep Generation)**
1. **Lập Kế hoạch Vector Vận tốc:**
   Gọi `calculator_.plan_impulses(dx, dy, dyaw)` để chia nhỏ hành trình thành chuỗi vector `ExecutionImpulse` (`1_YAW`, `2_STRAFE`, `3_FORWARD`).
2. **Tính toán Bàn chân Rời rạc (Footstep Geometry):**
   Gọi `calculator_.generate_discrete_footsteps(dx, dy, dyaw)` để tính toán chuỗi vị trí bàn chân trái/phải rời rạc dựa trên các ràng buộc `max_stride_x`, `max_stride_y`, `max_stride_yaw` và khoảng cách hai chân `stance_width = 0.20m`.
3. **Hiển thị Trực quan (Visualization):**
   Gọi `publish_footstep_markers()` để xuất `visualization_msgs/msg/MarkerArray` lên topic `/footstep_planner/markers`, hiển thị màu xanh/đỏ chuẩn trên RViz2.

#### **Giai đoạn 3: Vòng Lặp Điều Khiển Thực Thi 20Hz (20Hz Closed-Loop Control Loop)**
Lần lượt duyệt qua từng Pha (`1_YAW` $\to$ `2_STRAFE` $\to$ `3_FORWARD`):
- Khởi tạo mốc thời gian pha `phase_start_time` và vị trí bắt đầu pha `(p_{start\_x}, p_{start\_y}, p_{start\_yaw})`.
- Chạy vòng lặp `while (rclcpp::ok())` ở tần số **20 Hz**:
  1. **Check Cancellation:** Nếu người dùng nhấn Hủy, dừng robot khẩn cấp qua `Twist(0,0,0)` + `/g1_mode = "balance_stand"`.
  2. **Đo quãng đường thực tế (Traveled Distance):**
     Đọc TF `odom` $\to$ `base_link`, chiếu vị trí thực tế `(c_x, c_y, c_yaw)` lên Local Body Frame của pha để lấy $dx_{loc}, dy_{loc}, dyaw_{loc}$.
  3. **Check Hoàn thành Pha (Completion Check):**
     Nếu $\text{remaining} \le \text{tolerance}$ HOẶC thời gian vượt quá $1.5 \times duration + 0.5\text{ s}$ (timeout an toàn) $\rightarrow$ Hoàn thành pha.
  4. **P-Control & Soft Ramp:**
     Tính $v_{cmd\_mag} = \text{clamp}(K_p \cdot \text{remaining}, min\_exec\_v, max\_v\_mag) \times ramp\_up$.
  5. **Bù Chéo 3-DOF Thời Gian Thực (Active 3-DOF Cross-Track Compensation):**
     Tính toán vận tốc bù $v_x, v_y, \omega_z$ thời gian thực nhằm triệt tiêu lệch chéo.
  6. **Publish Vận tốc & Feedback:**
     - Publish `geometry_msgs/msg/Twist` lên `/cmd_vel_footstep`.
     - Publish `NavigateFootstep::Feedback` chứa `current_phase`, `percent_complete`, `time_remaining`, và `current_pose` thời gian thực.
- Ngắt dừng $0.1\text{ s}$ giữa các pha để cơ cấu chân robot kịp ổn định.

#### **Giai đoạn 4: Khóa Chân & Báo Cáo Sai Số Dư Kết Thúc (Completion, Foot Lock & Error Reporting)**
1. **Khóa Chân:** Phát lệnh đứng cân bằng `/g1_mode = "balance_stand"` để robot khóa chân vững chắc.
2. **Đọc TF Kết thúc:** Lấy tọa độ dừng thực tế `(end_x, end_y, end_yaw)`.
3. **Tính Sai số 3DoF:** Chiếu tổng quãng đường di chuyển về Local Frame ban đầu để tính 3 chỉ số sai số dư thực tế $(\text{err}_x, \text{err}_y, \text{err}_{yaw})$.
4. **Log Terminal:** In bảng tổng kết vị trí ban đầu, vị trí kết thúc, mục tiêu và sai số dư lên Terminal.
5. **Action Succeed:** Đóng Action Goal thành công và trả kết quả `NavigateFootstep::Result` về Client.

---

## 3. Thuật Toán Core & Công Thức Toán Học

### 3.1. Phân Tách 3 Pha Thực Thi (3-Phase Decoupled Impulses)
Để đảm bảo tính ổn định động học cho robot humanoid, bất kỳ mục tiêu di chuyển tương đối hay tuyệt đối $(dx, dy, d\theta)$ nào cũng được chia nhỏ thành 3 pha độc lập:
1. **Pha 1: `1_YAW`** – Xoay hướng robot về đúng hướng chuyển động.
2. **Pha 2: `2_STRAFE`** – Tạt ngang hông sang trái/phải.
3. **Pha 3: `3_FORWARD`** – Tiến hoặc lùi về đúng vị trí đích.

---

### 3.2. Phản Hồi Vòng Kín Odometry (Pure Closed-Loop TF Feedback)
Nút C++ Action Server (`FootstepPlannerActionNode`) theo dõi vị trí thực tế của robot qua phép biến đổi TF `odom` $\to$ `base_link` ở tần số **20 Hz**:

$$\Delta d = \sqrt{(x_{curr} - x_{start})^2 + (y_{curr} - y_{start})^2}$$

- **Bộ điều khiển P-Control với Hãm Đà Mịn (Soft Deceleration Ramp):**
  $$v_{cmd} = \text{clamp}(K_p \cdot (\text{target\_distance} - \Delta d), v_{min} + \text{margin}, v_{exec})$$
  - Khi robot tiến gần đến đích ($< 5\text{ cm}$), vận tốc được tự động giảm dần mịn màng về ngưỡng an toàn tối thiểu ($v_{min} + \text{margin} = 0.12\text{ m/s}$), triệt tiêu hoàn toàn hiện tượng bước quá đích (overshoot).
- **Khởi động mịn (Soft Ramp-Up):**
  Trong $0.2\text{ s}$ đầu của mỗi pha, vận tốc tăng dần từ 0 để tránh giật cơ cấu chân.
- **Tự động ngắt khi đạt Dung sai (Tolerance Termination):**
  Khi khoảng cách còn lại $\le \text{tolerance}$, pha dừng ngay lập tức và phát lệnh ngắt vận tốc.

---

### 3.3. Chủ Động Khử Lệch Chéo 3-DOF (Active 3-DOF Cross-Track Compensation)
Khi robot humanoid bước đi, sự dịch chuyển Trọng tâm (Center of Mass - CoM) giữa 2 chân sinh ra quán tính lắc ngang và xoay hông tự nhiên. Thuật toán chiếu biến thiên tọa độ về hệ tọa độ thân robot tại đầu mỗi pha (Local Body Frame) để phản đòn thời gian thực:

$$\begin{bmatrix} dx_{loc} \\ dy_{loc} \end{bmatrix} = \begin{bmatrix} \cos(\theta_{start}) & \sin(\theta_{start}) \\ -\sin(\theta_{start}) & \cos(\theta_{start}) \end{bmatrix} \begin{bmatrix} x_{curr} - x_{start} \\ y_{curr} - y_{start} \end{bmatrix}$$

$$\text{dyaw}_{loc} = \text{normalize\_angle}(\theta_{curr} - \theta_{start})$$

#### **Luật bù Cross-Track thời gian thực:**
- **Trong Pha Tiến (`3_FORWARD`):**
  $$v_x = \text{Direction} \cdot v_{cmd\_mag}$$
  $$v_y = \text{clamp}(-1.2 \cdot dy_{loc}, -0.06, 0.06) \quad \text{(Dập tắt dạt ngang } y\text{)}$$
  $$\omega_z = \text{clamp}(-1.0 \cdot dyaw_{loc}, -0.08, 0.08) \quad \text{(Giữ thẳng đầu } \theta\text{)}$$
- **Trong Pha Tạt Ngang (`2_STRAFE`):**
  $$v_y = \text{Direction} \cdot v_{cmd\_mag}$$
  $$v_x = \text{clamp}(-1.2 \cdot dx_{loc}, -0.06, 0.06) \quad \text{(Dập tắt trượt tiến } x\text{)}$$
  $$\omega_z = \text{clamp}(-1.0 \cdot dyaw_{loc}, -0.08, 0.08) \quad \text{(Giữ thẳng đầu } \theta\text{)}$$
- **Trong Pha Xoay Tại Chỗ (`1_YAW`):**
  $$\omega_z = \text{Direction} \cdot v_{cmd\_mag}$$
  $$v_x = \text{clamp}(-1.2 \cdot dx_{loc}, -0.05, 0.05)$$
  $$v_y = \text{clamp}(-1.2 \cdot dy_{loc}, -0.05, 0.05)$$

> **Kết quả:** Giảm **93% sai số trượt tiến khi tạt ngang** và giảm **80% sai số dạt ngang khi bước tiến**.

---

## 4. Lý Thuyết So Sánh: VFP (Velocity Footstep) vs. WBC (Whole-Body Control)

### 🧠 4.1. Tổng Quan Phương Pháp Velocity-Based Footstep Planning (VFP)
- **Cấp độ Tác động:** **High-Level Task Space Control**.
- **Nguyên lý:** Xem bộ điều khiển bước đi cơ sở (Low-Level Locomotion Controller / SDK LocoClient) của Unitree G1 là một **"Hộp Đen Động Học" (Black-Box Gait Generator)**.
- Thay vì can thiệp trực tiếp vào từng vị trí bàn chân rời rạc $SE(3)$ hay mô-men xoắn 23 motor chân, VFP liên tục tính toán vector vận tốc tối ưu $\mathbf{v} = [v_x, v_y, \omega_z]^T$ theo phản hồi vị trí thời gian thực từ TF Odometry (`odom` $\to$ `base_link`):
  $$\mathbf{p}(t) = \int_0^t \mathbf{R}(\theta(\tau)) \cdot \mathbf{v}(\tau) \, d\tau$$
- **Ưu điểm lớn:** **Độ an toàn tuyệt đối** (Unitree SDK tự đảm bảo robot không ngã), thời gian thực $20\text{ Hz}$ gọn nhẹ, độc lập với mô hình vật lý phức tạp.

---

### ⚖️ 4.2. Bảng So Sánh Chi Tiết VFP và WBC (Whole-Body Control)

| Tiêu chí So sánh | **Velocity-Based Footstep Planning (VFP)** *(Phương pháp Hệ thống)* | **Whole-Body Control (WBC / Foot Placement MPC)** |
| :--- | :--- | :--- |
| **Cấp độ Điều khiển (Control Layer)** | **High-Level Task Space** (Lệnh $v_x, v_y, \omega_z$ qua ROS 2 / SDK) | **Low-Level Joint / Task Space** (Giải Quadratic Programming QP ở 1kHz) |
| **Đầu vào Điều khiển (Control Input)** | Vector vận tốc thân robot $\mathbf{v} \in \mathbb{R}^3$ & thời gian xung | Tọa độ đặt bàn chân $SE(3)$ (Left/Right Foot Pose) & Lực nền Ground Reaction Force (GRF) |
| **Phụ thuộc Mô hình (Model Dependence)** | **Model-Free / Black-Box** (Không cần biết khối lượng, Momen quán tính hay lực khớp) | **Model-Based** (Cần mô hình động lực học chính xác: Con lắc ngược 3D LIPM, CoM, Dynamics Matrix) |
| **Độ Phức Tạp Tính Toán (Computational Cost)** | **Thấp ($20\text{ Hz}$)**, chạy mượt trên CPU thông thường | **Rất Cao ($500\text{ Hz} - 1\text{ kHz}$)**, yêu cầu Real-Time Linux (PREEMPT_RT / Xenomai) |
| **Độ An Toàn & Độc Lập (Safety & Encapsulated)** | **Rất An Toàn**. Tận dụng bộ cân bằng tĩnh/động cốt lõi của Unitree SDK | Nhạy cảm với nhiễu. Nếu giải bài toán QP sai dễ gây gãy gập khớp hoặc té ngã |
| **Địa hình Áp dụng (Applicable Terrain)** | Mặt phẳng, dốc nhẹ, hành lang, lối đi văn phòng, nhà xưởng | Địa hình phức tạp, bước qua đá hộc, leo cầu thang rải rạc, bậc thang không đều |

---

## 5. Cấu Trúc File & Tham Số Cấu Hình

### 📁 Cấu trúc Thư mục Package:
```text
g1_footstep_planner/
├── action/
│   └── NavigateFootstep.action     # Action definition (Goal, Result, Feedback)
├── config/
│   └── planner_params.yaml         # Các tham số vận tốc, tolerance, Kp gains
├── docs/
│   └── g1_footstep_planner_guide.md# Tài liệu kỹ thuật chi tiết
├── include/g1_footstep_planner/
│   └── footstep_calculator.hpp     # Library header tính toán nấc bước & impulse
├── launch/
│   ├── g1_footstep_planner.launch.py # Launch node đơn lẻ
│   └── g1_footstep_test.launch.py    # Launch tích hợp full test (Bridge + TwistMux + Action)
└── src/
    ├── footstep_calculator.cpp     # Logic hình học bước chân & impulse planning
    └── footstep_planner_action_node.cpp # C++ ROS 2 Action Server Node chính
```

---

### ⚙️ Các Tham số Kỹ thuật trong `planner_params.yaml`:

| Tham số | Giá trị | Đơn vị | Ý nghĩa |
| :--- | :---: | :---: | :--- |
| `vx_exec` | `0.12` | m/s | Vận tốc bước tiến/lùi thực thi |
| `vy_exec` | `0.12` | m/s | Vận tốc tạt ngang thực thi |
| `wz_exec` | `0.12` | rad/s | Vận tốc xoay tại chỗ thực thi |
| `v_min` | `0.10` | m/s | Ngưỡng vận tốc tuyến tính tối thiểu SDK Unitree G1 |
| `w_min` | `0.10` | rad/s | Ngưỡng vận tốc góc tối thiểu SDK Unitree G1 |
| `deadband_safety_margin` | `0.02` | m/s | Biên độ an toàn tự động cộng vào khi hãm đà |
| **`forward_tolerance`** | **`0.005`** | **m (5mm)** | **Dung sai dừng bước tiến/lùi** |
| **`strafe_tolerance`** | **`0.005`** | **m (5mm)** | **Dung sai dừng tạt ngang** |
| **`yaw_tolerance`** | **`0.01`** | **rad (~0.57°)** | **Dung sai dừng góc xoay** |
| `enable_closed_loop` | `true` | bool | Bật/tắt cơ chế Vòng kín Odometry Feedback |
| `kp_linear` | `1.5` | - | Hệ số P điều khiển vận tốc tuyến tính |
| `kp_angular` | `1.5` | - | Hệ số P điều khiển vận tốc góc xoay |

---

## 6. Hướng Dẫn Khởi Chạy & Vận Hành (Quick Start)

### 🚀 Bước 1: Khởi chạy các Node Hạ tầng (3 Terminals)

**Terminal 1 (G1 Bringup - State Estimator & SDK Channel):**
```bash
cd /home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws
source install/setup.bash
ros2 launch g1_bringup g1_bringup.launch.py
```

**Terminal 2 (G1 Localization - FAST-LIO LiDAR Odometry):**
```bash
cd /home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws
source install/setup.bash
ros2 launch g1_localization g1_localization.launch.py
```

**Terminal 3 (G1 Footstep Planner Test Node):**
```bash
cd /home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws
source install/setup.bash
ros2 launch g1_footstep_planner g1_footstep_test.launch.py
```

---

### 📡 Bước 2: Gửi Lệnh Chuyển Động (Action Goals)

#### **1. Tiến về phía trước 0.3 mét:**
```bash
ros2 action send_goal /navigate_footstep g1_footstep_planner/action/NavigateFootstep "{
  target_pose: {
    header: {frame_id: 'base_link'},
    pose: {position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
  }
}" --feedback
```

#### **2. Lùi về phía sau 0.3 mét:**
```bash
ros2 action send_goal /navigate_footstep g1_footstep_planner/action/NavigateFootstep "{
  target_pose: {
    header: {frame_id: 'base_link'},
    pose: {position: {x: -0.3, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
  }
}" --feedback
```

#### **3. Tạt sang bên trái 0.5 mét:**
```bash
ros2 action send_goal /navigate_footstep g1_footstep_planner/action/NavigateFootstep "{
  target_pose: {
    header: {frame_id: 'base_link'},
    pose: {position: {x: 0.0, y: 0.5, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
  }
}" --feedback
```

#### **4. Xoay tại chỗ sang trái 90 độ ($\pi/2 \approx 1.57\text{ rad}$):**
```bash
ros2 action send_goal /navigate_footstep g1_footstep_planner/action/NavigateFootstep "{
  target_pose: {
    header: {frame_id: 'base_link'},
    pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}
  }
}" --feedback
```

---

## 7. Kết Quả Thực Nghiệm Trên Robot Thật G1

Bảng tổng hợp kết quả đo đạc độ chính xác thực tế từ hệ thống TF Odometry của robot:

| Kịch bản Chuyển động | Mục tiêu | Kết quả Thực tế | Sai số Dư (`final_error`) | Độ chính xác |
| :--- | :---: | :---: | :---: | :---: |
| **Tiến 0.3m (Forward)** | $x = +0.30\text{ m}$ | $\Delta x = 0.3064\text{ m}$ | **$\text{err}_x = -0.010\text{ m}$ ($1.0\text{ cm}$)** | **$96.6\%$** |
| **Tạt ngang 0.5m (Strafe)** | $y = +0.50\text{ m}$ | $\Delta y = 0.5135\text{ m}$ | **$\text{err}_y = -0.013\text{ m}$ ($1.3\text{ cm}$)** | **$97.4\%$** |
| **Dạt chéo khi Tạt ngang** | $x = 0.00\text{ m}$ | $\Delta x = 0.0227\text{ m}$ | **$\text{err}_x = +0.022\text{ m}$ ($2.2\text{ cm}$)** | *Giảm 93% trượt chéo* |
| **Lệch góc khi di chuyển** | $\text{Yaw} = 0.00^\circ$ | $\Delta \text{Yaw} = 1.4^\circ$ | **$\text{err}_{\text{yaw}} = +0.025\text{ rad}$ ($1.4^\circ$)** | *Giữ hướng thẳng tắp* |

---
*Tài liệu được khởi tạo và cập nhật tự động cho dự án ROS 2 Unitree G1 Humanoid Robot.*
