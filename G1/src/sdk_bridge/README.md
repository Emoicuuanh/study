# Unitree G1 ROS 2 SDK Bridge (`sdk_bridge`)

Package này là cầu nối (Bridge) giữa ROS 2 (Navigation 2 / Safety Controller) và Unitree SDK 2 của Robot Humanoid **Unitree G1**.

## Tính năng chính
1. **Chuyển đổi `/cmd_vel` sang lệnh SDK `LocoClient.Move()`**: Phát lệnh vận tốc chuyển động 3 DOF ($v_x, v_y, v_{\omega}$) theo thời gian thực tới G1.
2. **Hỗ trợ chọn Mode di chuyển linh hoạt**:
   - Chuyển giữa các chế độ **Đi bộ (Walk)** và **Chạy (Run / Fast Mode)**.
   - Thay đổi độ cao bước chân (**Swing Height**).
   - Thực hiện các tư thế & hành động: **BalanceStand**, **HighStand**, **LowStand**, **Sit**, **Damp**, **WaveHand**, **ShakeHand**.
3. **Đổi Mode thời gian thực**: Qua ROS 2 Topic `/g1_mode` (`std_msgs/msg/String`) hoặc Dynamic Parameters.

---

## Hướng dẫn chi tiết & Bộ lệnh sẵn

Vui lòng xem file tài liệu hướng dẫn đầy đủ các lệnh copy-paste sẵn tại:
👉 **[G1_MOTION_MODE_GUIDE.md](file:///home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws/src/sdk_bridge/G1_MOTION_MODE_GUIDE.md)**

---

## Các lệnh cơ bản nhanh

### 1. Khởi chạy Node:
```bash
ros2 launch sdk_bridge sdk_bridge.launch.py
```

### 2. Đổi Mode di chuyển nhanh qua Topic:
```bash
# Chuyển sang Mode Chạy
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'run'" --once

# Chuyển sang Mode Đi bộ
ros2 topic pub /g1_mode std_msgs/msg/String "data: 'walk'" --once
```

### 3. Đổi Mode qua Parameter:
```bash
ros2 param set /cmd_vel_to_sdk_node speed_mode 1   # Mode chạy
ros2 param set /cmd_vel_to_sdk_node swing_height 0.1 # Nhấc chân cao 10cm
```
