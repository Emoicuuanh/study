 

# Kế hoạch Triển khai: Lập kế hoạch Bước chân & Bộ điều khiển Động lực học Toàn thân (Low-Level WBC) cho Unitree G1

Tài liệu này mô tả chi tiết kiến trúc kỹ thuật, thuật toán, công nghệ và lộ trình từng bước để triển khai **Bộ lập kế hoạch bước chân (Search-Based Footstep Planner)** và **Bộ điều khiển Động lực học toàn thân (Low-Level Whole-Body Controller - WBC)** cho robot humanoid Unitree G1 trong môi trường ROS 2.

---

## Yêu cầu Kiểm duyệt từ Người dùng (User Review Required)

> [!IMPORTANT]
> **Cảnh báo Quy trình An toàn cho Điều khiển Khớp Low-Level (`rt/lowcmd`)**
> Việc điều khiển trực tiếp mô-men xoắn/góc khớp ở tầng Low-Level sẽ bỏ qua bộ điều khiển thăng bằng mặc định của Unitree. **TẤT CẢ các thuật toán BẮT BUỘC phải được kiểm thử 100% trên phần mềm mô phỏng (MuJoCo / Gazebo)** trước khi chạy trên robot thật. Khi kiểm thử trên robot thật, robot **BẮT BUỘC phải được treo trên khung giá bảo hộ an toàn**.

> [!NOTE]
> **Tích hợp Tái sử dụng Tài nguyên Workspace**
> Chúng ta sẽ tận dụng các gói ROS 2 đã có sẵn trong workspace của bạn:
>
> - [g1_description](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/g1_description): Chứa file mô hình động lực học URDF & hình học 3D của G1.
> - [g1_state_estimator](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/g1_state_estimator): Chứa bộ ước lượng vị trí/vận tốc thân robot (Floating-Base Odometry).
> - [sdk_bridge](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/sdk_bridge): Cầu nối giao tiếp ROS 2 sang Unitree SDK 2.

---

## Câu hỏi Mở cần Xác nhận (Open Questions)

> [!IMPORTANT]
> 1. **Môi trường Mô phỏng**: Bạn hiện đã có sẵn môi trường **MuJoCo** hoặc **Isaac Sim** cài trên máy cho G1 chưa, hay chúng ta sẽ dựng một Node mô phỏng MuJoCo / Gazebo nhẹ trong ROS 2 trước?
> 2. **Lựa chọn Ngôn ngữ (Python vs C++)**:
>    - **Python** (dùng `Pinocchio` + `proxsuite`): Phát triển nhanh, dễ debug (tần số 200-500Hz).
>    - **C++**: Tối ưu hiệu năng thời gian thực tuyệt đối (tần số 1000Hz).
>      Bạn muốn bắt đầu bằng Python để kiểm thử thử nghiệm trước hay làm thẳng bằng C++?

---

## Kiến trúc Hệ thống & Luồng Dữ liệu (Architecture & Flow)

```
[Nhận diện Target (AprilTag / Camera)] 
                   │
                   ▼
┌─────────────────────────────────────────────────────────────┐
│ 1. Search-Based Footstep Planner (A* / Kinematic Planner)   │
│    - Tính toán chuỗi bước chân rời rạc: {S_1, ..., S_N}     │
│    - Tần số: 1 - 5 Hz                                       │
└──────────────────────────────┬──────────────────────────────┘
                               │ Chuỗi dấu chân rời rạc S_k (x, y, z, yaw)
                               ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. Centroidal & Swing Trajectory Generator                  │
│    - Tính quỹ đạo Trọng tâm (CoM) & Đường cong chân lăng 3D │
│    - Tần số: 50 - 100 Hz                                    │
└──────────────────────────────┬──────────────────────────────┘
                               │ Gia tốc & Quỹ đạo mong muốn (x_ddot, F_des)
                               ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. Hierarchical Whole-Body Controller (HQP WBC)             │
│    - Thiết lập bài toán tối ưu QP với Pinocchio Dynamics    │
│    - Giải: Min ||x_ddot - x_ddot_des|| s.t. Friction Cone   │
│    - Tần số: 500 - 1000 Hz                                  │
└──────────────────────────────┬──────────────────────────────┘
                               │ Lệnh khớp (q_des, dq_des, Kp, Kd, tau_ff)
                               ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Low-Level DDS Bridge (`rt/lowcmd` / `rt/lowstate`)       │
│    - Gửi lệnh mô-men/góc khớp trực tiếp tới 29 động cơ G1   │
└─────────────────────────────────────────────────────────────┘
```

---

## Đề xuất Các Gói phần mềm Mới & Thay đổi (Proposed Changes)

### Thành phần 1: Gói Nhận diện & Lập kế hoạch Bước chân (`g1_footstep_planner`)

#### [NEW] [g1_footstep_planner](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/g1_footstep_planner)

Tạo gói ROS 2 mới chịu trách nhiệm xác định vị trí mốc và lập kế hoạch đường bước chân.

- **`target_perception_node.py`**: Tính tọa độ tương đối $(dx, dy, d\theta)$ giữa Camera và AprilTag/Target Docking.
- **`footstep_planner_node.py`**: Thuật toán A* / Kinematic Planner tính toán danh sách nấc bước chân tuân thủ giới hạn động học G1 ($L_{max} = 0.12\text{m}$, $W_{stance} = 0.20\text{m}$, $\theta_{max} = 0.15\text{rad}$).

---

### Thành phần 2: Gói Bộ điều khiển Động lực học Toàn thân (`g1_wbc_controller`)

#### [NEW] [g1_wbc_controller](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/g1_wbc_controller)

Tạo gói điều khiển Low-Level WBC chứa mô hình động lực học và bộ giải tối ưu QP.

- **`g1_kinematics.py` / `g1_kinematics.cpp`**:
  - Dùng `Pinocchio` nạp file [g1.urdf](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/g1_description/urdf/g1.urdf).
  - Tính ma trận quán tính $M(q)$, Coriolis/Gravity $h(q, \dot{q})$, và Jacobian bàn chân $J_L(q), J_R(q)$.
- **`swing_trajectory.py`**:
  - Sinh đường cong Bézier bậc 5 tạo quỹ đạo nhấc/hạ chân lăng 3D ($h_{swing} = 0.04\text{m}$).
- **`hqp_wbc_solver.py` / `hqp_wbc_solver.cpp`**:
  - Giải bài toán tối ưu HQP bằng `ProxQP` hoặc `OSQP`.
  - Task 1: Phương trình động lực học thân ngực & Ràng buộc nón ma sát ($|F_{xy}| \le \mu F_z$).
  - Task 2: Theo đuổi vị trí điểm đáp bàn chân lăng.
  - Task 3: Giữ thăng bằng độ cao và tư thế thân người (Pelvis / CoM height).
- **`g1_low_level_node.py`**:
  - Kết nối channel DDS của Unitree SDK 2 (`rt/lowstate` @ 500Hz, `rt/lowcmd` @ 500Hz).
  - Tích hợp **Cơ chế An toàn Ngắt Khẩn cấp (Watchdog & Emergency Damping Fallback)**: Tự động chuyển $K_p \to 0, K_d \to 5.0$ nếu robot nghiêng quá $25^\circ$ hoặc rớt gói tin DDS.

---

### Thành phần 3: Cầu nối SDK & Chuyển đổi Chế độ (SDK Bridge)

#### [MODIFY] [cmd_vel_to_sdk_node.py](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/sdk_bridge/sdk_bridge/cmd_vel_to_sdk_node.py)

- Bổ sung Service/Topic hỗ trợ chuyển đổi mượt mà giữa chế độ High-Level `LocoClient` (Nav2) và chế độ Low-Level WBC (`rt/lowcmd`).

---

## Kế hoạch Kiểm thử & Xác minh (Verification Plan)

### Pha 1: Kiểm thử Mô phỏng (Simulation Testing)

1. Nạp file URDF `g1_description` vào môi trường mô phỏng MuJoCo / Gazebo.
2. Kiểm thử gói `g1_wbc_controller` trên mô phỏng:
   - Kiểm tra khả năng thăng bằng đứng 1 chân.
   - Kiểm tra đường cong chân lăng nhấc/hạ theo đúng quỹ đạo.
   - Kiểm tra độ chính xác đặt chân tiếp cận mục tiêu trong khoảng $\pm 5\text{mm}$.

### Pha 2: Quy trình An toàn & Kiểm thử trên Robot Thật

1. **Thiết lập Khung Giá Bảo hộ**: Treo robot Unitree G1 lên khung dây cáp bảo hộ chống ngã.
2. **Pha 2.1 (Kiểm tra Thả lỏng Damping)**: Đọc dữ liệu `rt/lowstate` (encoder 29 khớp, IMU) và thử nghiệm chế độ thả lỏng Damping.
3. **Pha 2.2 (Thăng bằng Đứng trọng trường)**: Thử nghiệm bù trọng trường và thăng bằng CoM trên robot thật.
4. **Pha 2.3 (Docking Bước chân Chính xác)**: Thực thi chuỗi bước chân tiếp cận AprilTag mục tiêu.
