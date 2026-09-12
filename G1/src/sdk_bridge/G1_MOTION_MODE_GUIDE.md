# Hướng Dẫn Sử Dụng & Bộ Lệnh Đã Tạo Sẵn Cho Unitree G1 SDK Bridge

Tài liệu này chứa danh sách các lệnh ROS 2 đã được chuẩn bị sẵn (Copy & Paste) để khởi chạy và điều khiển các Mode di chuyển, tư thế và bước chân cho Robot Humanoid Unitree G1.

---

## 1. Khởi chạy Node Bridge (`cmd_vel_to_sdk_node`)

### Khởi chạy chính thức (Kết nối Robot G1 qua Card Mạng):
```bash
ros2 launch sdk_bridge sdk_bridge.launch.py
```
*Hoặc chạy trực tiếp:*
```bash
ros2 run sdk_bridge cmd_vel_to_sdk_node --ros-args -p network_interface:=wlp2s0
```

### Khởi chạy ở chế độ Mô phỏng / Test (Không cần robot thật):
```bash
ros2 run sdk_bridge cmd_vel_to_sdk_node --ros-args -p enable_sdk:=false
```

---

## 2. Các Lệnh Đổi Mode Di Chuyển & Tư Thế Qua Topic `/g1_mode`

Bạn mở một terminal mới và copy-paste các lệnh bên dưới để điều khiển robot:

### 🚶‍♂️ Mode Đi bộ thông thường (Normal Walk Mode):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'walk'" --once
```

### 🏃‍♂️ Mode Chạy / Đi nhanh (Run / Fast Mode):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'run'" --once
```

### 🧍‍♂️ Mode Đứng cân bằng (Balance Stand):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'stand'" --once
```

### 🦒 Mode Nâng cao trọng tâm (High Stand):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'high_stand'" --once
```

### 🧘‍♂️ Mode Hạ thấp trọng tâm (Low Stand):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'low_stand'" --once
```

### 🪑 Mode Ngồi xuống (Sit):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'sit'" --once
```

### 🛑 Mode Thả lỏng / Giảm chấn (Damp Mode):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'damp'" --once
```

### 👋 Mode Vẫy tay chào (Wave Hand Gesture):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'wave'" --once
```

### 🤝 Mode Bắt tay (Shake Hand Gesture):
```bash
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'shake'" --once
```

---

## 3. Các Lệnh Thay Đổi Dynamic Parameter Thời Gian Thực

### Chuyển Speed Mode sang Chạy (1) hoặc Đi bộ (0):
- **Chạy/Đi nhanh (Speed Mode = 1):**
  ```bash
  ros2 param set /cmd_vel_to_sdk_node speed_mode 1
  ```
- **Đi bộ bình thường (Speed Mode = 0):**
  ```bash
  ros2 param set /cmd_vel_to_sdk_node speed_mode 0
  ```

### Thay đổi độ cao bước chân (Swing Height):
- **Nhấc chân cao 10 cm (0.1 m):**
  ```bash
  ros2 param set /cmd_vel_to_sdk_node swing_height 0.1
  ```
- **Nhấc chân cao 15 cm (0.15 m):**
  ```bash
  ros2 param set /cmd_vel_to_sdk_node swing_height 0.15
  ```
- **Trở về độ cao nhấc chân mặc định (0.0 m):**
  ```bash
  ros2 param set /cmd_vel_to_sdk_node swing_height 0.0
  ```

---

## 4. Lệnh Phát Vận Tốc Thử Nghiệm (`/cmd_vel`)

### Di chuyển tiến chậm (Đi bộ - 0.3 m/s):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

### Di chuyển tiến nhanh (Chạy - 1.0 m/s):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

### Xoay tại chỗ (0.5 rad/s):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" -r 10
```

### Dừng khẩn cấp:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```
