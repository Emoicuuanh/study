# Hướng Dẫn Điều Khiển Tay Robot Inspire (control_hand)

Package này cung cấp driver kết nối và Action Server trong ROS 2 để điều khiển hai tay robot Inspire của robot Unitree G1 qua giao thức Modbus TCP và DDS.

---

## 🛠️ Kiến trúc Hệ thống

Hệ thống điều khiển tay robot gồm 2 thành phần chính:
1. **`init_driver.py`**: Driver tầng thấp, thiết lập kết nối Modbus TCP tới tay robot (Tay Trái: `192.168.123.210`, Tay Phải: `192.168.123.211`). Nó đọc trạng thái từ tay và tương tác với các DDS Topic của Unitree.
2. **`hand_node.py`**: Chạy Action Server ROS 2 trên topic `/hand_command` (sử dụng action message `unitree_msgs/action/HandCommand`). Khi nhận được goal, node này sẽ chuyển đổi và gửi lệnh điều khiển tới driver thông qua DDS.

---

## 🚀 Các Bước Chạy và Điều Khiển

### 1. Build Workspace
Trước khi chạy lần đầu hoặc khi có thay đổi trong code, bạn cần build lại workspace:
```bash
cd /home/hoangdc/ROS2/unitree_G1/dev_unitreeg1_ws
colcon build --packages-select control_hand
source install/setup.bash
```

### 2. Khởi chạy Driver và Node (Terminal 1)
Chạy file launch để kích hoạt đồng thời cả Driver Modbus và ROS 2 Action Server:
```bash
source install/setup.bash
ros2 launch control_hand control_hands_driver.launch.py
```

---

## 🎮 Cách Gửi Lệnh Điều Khiển (Terminal 2)

Sau khi chạy file launch, mở một terminal mới và chạy các lệnh sau:

### Sử dụng trực tiếp bằng ROS 2 Action CLI

#### A. Các tư thế cơ bản (Standard Mode)
* **Mở cả 2 tay:**
  ```bash
  ros2 action send_goal /hand_command unitree_msgs/action/HandCommand "{hand: 'both', type: 'standard', name_action: 'open_hand'}"
  ```
* **Đóng cả 2 tay:**
  ```bash
  ros2 action send_goal /hand_command unitree_msgs/action/HandCommand "{hand: 'both', type: 'standard', name_action: 'close_hand'}"
  ```
* **Gắp bảng (Take board):**
  ```bash
  ros2 action send_goal /hand_command unitree_msgs/action/HandCommand "{hand: 'both', type: 'standard', name_action: 'take_board'}"
  ```
* **Đặt bảng (Place board):**
  ```bash
  ros2 action send_goal /hand_command unitree_msgs/action/HandCommand "{hand: 'both', type: 'standard', name_action: 'place_board'}"
  ```
* **Chỉ điều khiển tay trái hoặc tay phải:** Thay `both` bằng `left` hoặc `right`:
  ```bash
  ros2 action send_goal /hand_command unitree_msgs/action/HandCommand "{hand: 'left', type: 'standard', name_action: 'open_hand'}"
  ```

---

#### B. Điều khiển góc quay chi tiết (Custom Mode)
Để điều khiển chính xác các ngón tay bằng cách truyền góc (`angles`), lực tối đa (`forces`), tốc độ (`speeds`):
* Cần chọn `type: 'custom'`.
* Đặt `mode` (Ví dụ: `13` ứng với bitmask của ANGLE, FORCE, VELOCITY).
* Truyền danh sách 6 phần tử tương ứng với 6 khớp ngón tay (0 đến 1000).

Ví dụ khép chặt tay bằng Custom Mode:
```bash
ros2 action send_goal /hand_command unitree_msgs/action/HandCommand "{hand: 'both', type: 'custom', mode: 13, angles: [1000, 1000, 1000, 1000, 1000, 0], forces: [500, 500, 500, 500, 500, 500], speeds: [400, 400, 400, 400, 400, 400], timeout: 3.0}"
```

---

## 🔍 Chẩn Đoán Lỗi Tay (Stuck/Non-Responsive Finger)

Nếu có ngón tay bị đứng im, kẹt cứng hoặc không phản hồi điều khiển, bạn có thể chạy công cụ chẩn đoán để đọc trực tiếp trạng thái cảm biến, dòng điện, nhiệt độ và mã lỗi phần cứng từ tay robot:

```bash
# Sourcing workspace
source install/setup.bash

# Khởi chạy chẩn đoán lỗi
ros2 run control_hand diagnose_hands
```

### Cách đọc kết quả:
Công cụ sẽ in ra bảng trạng thái của từng khớp ngón tay bao gồm:
* **Pos / Angle**: Vị trí & góc thực tế.
* **Force / Curr**: Lực phản hồi & Dòng điện (mA) của motor.
* **Temp**: Nhiệt độ motor (nếu quá nóng, motor tự ngắt để bảo vệ).
* **Status / Errors**: Hiển thị rõ các lỗi như `Stall / Jammed` (kẹt cơ), `Over-temperature` (quá nhiệt), `Over-current` (quá dòng bảo vệ), `Communication Failure` (lỗi giao tiếp ngón tay).
