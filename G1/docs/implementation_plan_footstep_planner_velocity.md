# Tài liệu Kiến trúc Closed-Loop Odometry Feedback: C++ ROS 2 Action Server `g1_footstep_planner` cho Unitree G1

Tài liệu này tổng hợp kiến trúc kỹ thuật và thuật toán **Closed-Loop Odometry Feedback (Vòng kín Phản hồi Odometry)** trực tiếp qua TF của gói **`g1_footstep_planner`**.

---

## 🏛️ 1. Nguyên lý Vòng Kín Odometry Feedback (Closed-Loop Control)

1. **Đọc Vị trí Thực tế qua TF Odometry (Real-Time TF Tracking):**
   Khi bắt đầu mỗi Pha (Xoay Yaw, Tạt ngang Strafe, Tiến/Lùi Forward), hệ thống ghi lại vị trí xuất phát $p_{start}$. Tại mỗi chu kỳ $20\text{ Hz}$, hệ thống đọc vị trí hiện tại $p_{curr}$ qua TF `odom` $\to$ `base_link` để tính quãng đường thực tế đã đi:
   $$d_{traveled} = \sqrt{(x_{curr} - x_{start})^2 + (y_{curr} - y_{start})^2}$$

2. **Bộ điều khiển P-Control & Giảm tốc Mềm (Soft Ramp-Down):**
   Tính khoảng cách thực tế còn lại: $\Delta d_{remaining} = d_{target} - d_{traveled}$.
   Khi gần đến đích, vận tốc được điều chỉnh mềm mại theo bộ điều khiển Proportional (P-Controller):
   $$v_{cmd} = K_p \cdot \Delta d_{remaining}$$
   Vận tốc được giới hạn an toàn trong khoảng $[v_{min} + \text{margin}, v_{exec}]$.

3. **Gia tốc Mềm (Ramp-Up):**
   Trong $0.2\text{ s}$ đầu của mỗi pha, vận tốc tăng dần mềm mại để tránh hiện tượng chân robot bị giật giật trên mặt sàn.

4. **Ngắt Pha Chính xác (Phase Termination):**
   Pha sẽ kết thúc ngay lập tức khi $\Delta d_{remaining} \le \text{tolerance}$ ($0.005\text{ m}$ / $5\text{ mm}$), loại bỏ sai số tích lũy khi tiến lùi lặp lại.

---

## ⚙️ 2. Danh sách Tham số Cấu hình trong `planner_params.yaml`

```yaml
g1_footstep_planner_node:
  ros__parameters:
    # -------------------------------------------------------------
    # 1. Vận tốc thực thi (Execution Velocities)
    # -------------------------------------------------------------
    vx_exec: 0.12         # Vận tốc tiến/lùi thực thi (m/s)
    vy_exec: 0.12         # Vận tốc tạt ngang thực thi (m/s)
    wz_exec: 0.12         # Vận tốc xoay tại chỗ thực thi (rad/s)

    # -------------------------------------------------------------
    # 2. Ngưỡng Vận tốc tối thiểu & Biên độ An toàn (SDK Deadbands)
    # -------------------------------------------------------------
    v_min: 0.10                   # Ngưỡng vận tốc tuyến tính tối thiểu (m/s)
    w_min: 0.10                   # Ngưỡng vận tốc góc tối thiểu (rad/s)
    deadband_safety_margin: 0.02  # Biên độ an toàn cộng thêm (m/s)

    # -------------------------------------------------------------
    # 3. Dung sai Dừng (Target Tolerances)
    # -------------------------------------------------------------
    forward_tolerance: 0.005  # Dung sai tiến/lùi (m) [5mm]
    strafe_tolerance: 0.005   # Dung sai tạt ngang (m) [5mm]
    yaw_tolerance: 0.01       # Dung sai góc xoay (rad) [~0.57 độ]

    # -------------------------------------------------------------
    # 4. Điều khiển Vòng kín Odometry (Closed-Loop Feedback)
    # -------------------------------------------------------------
    enable_closed_loop: true      # Bật cơ chế vòng kín Odometry Feedback
    kp_linear: 1.5                # Hệ số P điều khiển vận tốc tuyến tính
    kp_angular: 1.5               # Hệ số P điều khiển vận tốc góc
```

---

## 🚀 3. Hướng dẫn Biên dịch & Chạy Kiểm thử

### Biên dịch gói ROS 2 C++:
```bash
cd /home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws
colcon build --packages-select g1_footstep_planner
source install/setup.bash
```

### Khởi chạy Test trên Robot Thật:
```bash
ros2 launch g1_footstep_planner g1_footstep_test.launch.py
```

### Gửi Lệnh Tiến 0.3m & Lùi -0.3m:
```bash
# Lệnh 1: Tiến 0.3m
ros2 action send_goal /navigate_footstep g1_footstep_planner/action/NavigateFootstep "{
  target_pose: {
    header: {frame_id: 'base_link'},
    pose: {position: {x: 0.3, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
  }
}" --feedback

# Lệnh 2: Lùi -0.3m
ros2 action send_goal /navigate_footstep g1_footstep_planner/action/NavigateFootstep "{
  target_pose: {
    header: {frame_id: 'base_link'},
    pose: {position: {x: -0.3, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}
  }
}" --feedback
```
