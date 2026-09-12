# Humanoid Robotics — Unitree G1

Nhật ký học và code thực hành về robot humanoid, từ điều khiển cổ điển
đến SLAM/navigation, dùng Unitree G1 làm đối tượng.

Mỗi phần đều có số đo thật kèm theo — xem `g1_ws/docs/sotay.html` (sổ tay
tổng hợp) và `INTERVIEW_QUESTIONS.md` (32 câu hỏi ôn phỏng vấn).

---

## Nội dung

### Phần 1 — Điều khiển cổ điển trong MuJoCo (`week*.py`)

| File | Nội dung |
|---|---|
| `week1_inspect_g1.py` | Floating base, quaternion, khảo sát model |
| `week2_pd_stand.py` | Ba chế độ actuator, PD đứng thẳng |
| `week3_squat*.py` | Sinh quỹ đạo mượt, quy tắc −θ/+2θ/−θ |
| `week4_ik_reach.py` | Động học nghịch, damped least squares |
| `week4_nullspace.py` | Không gian rỗng, xử lý bậc tự do dư |
| `week5_lqr_*.py` | LQR, phương trình Riccati (+ `cartpole.xml`) |
| `week6_capture_point.py` | Capture point — thành phần phân kỳ của LIPM |
| `week7_lipm_steps.py` | LIPM, hoạch định bước hồi phục |
| `week8_mpc_lipm.py` | MPC so với greedy |
| `week9_wbc.py` | Whole-Body Control bằng QP có trọng số |

### Phần 2 — Mô phỏng Isaac Sim + ROS2 (`g1_ws/`)

```
g1_ws/
├── isaac/              mô phỏng — Python 3.11 (conda isaaclab)
│   ├── run_g1_sim.py     điểm chạy chính
│   ├── scene/            dựng cảnh, phòng kín 14×14m
│   ├── sensors/          LiDAR OS0, camera RGB-D, IMU
│   ├── control/          slide · walk · rl (policy Unitree)
│   ├── bridge/           cầu nối ROS2 qua OmniGraph
│   └── tests/            script đo kiểm độc lập
├── src/                ROS2 — Python 3.12 (hệ thống)
│   ├── g1_perception/    SLAM 2D + 3D, frame ổn định trọng lực
│   ├── g1_bringup/       cấu hình RViz
│   ├── fast_lio/         FAST-LIO đã vá 3 chỗ
│   └── livox_ros_driver2/  chỉ message, không có driver
└── docs/sotay.html     sổ tay tổng hợp lý thuyết + số đo
```

### Phần 3 — Thử nghiệm Isaac ban đầu (`isaac_nav/`)

Các bước a→e dựng dần đường ống, giữ lại để đối chiếu.

---

## Hai môi trường Python tách biệt

Đây là điểm dễ nhầm nhất khi dựng lại:

| | Dùng cho | Python | Cài bằng |
|---|---|---|---|
| `.venv` | MuJoCo, `week*.py` | 3.13 | `requirements.txt` |
| conda `isaaclab` | Isaac Sim | 3.11 | Isaac Sim installer |
| hệ thống | ROS2 Jazzy | 3.12 | apt |

Ba cái **không import được lẫn nhau** — `rclpy` biên dịch riêng cho 3.12.
Isaac Sim và ROS2 gặp nhau ở tầng DDS, không phải tầng Python.

```bash
# moi truong MuJoCo
python3.13 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

---

## Repo phụ thuộc — clone riêng

Ba thư mục sau **không nằm trong repo này**:

```bash
git clone https://github.com/unitreerobotics/unitree_rl_gym.git
git clone https://github.com/google-deepmind/mujoco_menagerie.git
git clone https://github.com/Emoicuuanh/G1.git
```

- `unitree_rl_gym` — policy `motion.pt` đã train và URDF 12 khớp
- `mujoco_menagerie` — model MuJoCo của G1
- `G1` — workspace ROS2 riêng (SDK bridge, kế hoạch navigation)

---

## Chạy mô phỏng

```bash
cd g1_ws
./build.sh                                                # build ROS2

# T1 — mô phỏng
./run_sim.sh --gait rl --scene room --no-camera --teleop
# T2 — SLAM 2D
./run_slam.sh
# T3 — hiển thị
./run_rviz_slam.sh
# T4 — điều khiển   (i tiến · k dừng · j/l xoay)
./run_teleop.sh
```

Vài cờ hữu ích:

```bash
--drive position       PD chạy trong PhysX, giống kiến trúc robot thật
--gait slide|walk|rl   mức độ trung thực vật lý tăng dần
SIM_THREADS=4          giảm CPU (mặc định 6; Isaac mặc định 20 → chậm hơn)
```

---

## Vài kết quả đo được

```
SLAM 2D, sau khi sửa frame ổn định trọng lực
  TF map→odom lệch        9.3 m / −96°   →   0.19 m / 1.7°

Policy RL, lệnh = 0, độ trôi
  model 43 khớp (sai)     0.212 m/s
  model 12 khớp (đúng)    0.026 m/s
  MuJoCo tham chiếu       0.011 m/s

Hiệu năng, threadCount 20 → 6
  CPU 503% → 222%   và mô phỏng NHANH HƠN (0.78× → 1.00× real-time)
```

Nguyên nhân và cách tìm ra từng con số ghi trong `g1_ws/docs/sotay.html`.

---

## Ghi chú về `g1_ws/isaac/assets/`

File `configuration/g1_12dof_base.usd` (34 MB) được giữ trong repo dù là
tài sản dẫn xuất, vì **không có script tái tạo**: nó được nhập từ
`g1_12dof.urdf` của `unitree_rl_gym` bằng URDF importer của Isaac Sim,
với hai tham số dễ sai:

```
default_drive_type    = UrdfJointTargetType.JOINT_DRIVE_POSITION
collision_from_visuals = True
```

Thiếu file này thì `--gait rl` không chạy được.

Lưu ý thêm: drive trong file USD nhập ra mặc định là `acceleration`,
stiffness 625, damping 0 — cờ `--drive position` sẽ tự chuyển sang `force`
với gain đúng của policy. Xem `isaac/control/rl_walk.py`.
