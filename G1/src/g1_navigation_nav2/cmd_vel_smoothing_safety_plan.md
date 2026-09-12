# Kế Hoạch Triển Khai Bộ Làm Mượt & An Toàn Vận Tốc (/cmd_vel) Cho Unitree G1

Tài liệu này chi tiết từng bước cài đặt và tích hợp 4 thành phần làm mượt và bảo vệ an toàn vận tốc cho robot chân người **Unitree G1** trong ROS 2 (Nav2).

---

## 🏗 Kiến Trúc Luồng Dữ Liệu (Dataflow Architecture)

```
                       [ Joystick / Teleop ]
                                |
                         (/cmd_vel_teleop)
                                |
[ Nav2 Controller ]             v
        |                +---------------+
 (/cmd_vel_nav) -------->|   twist_mux   |
                         +---------------+
                                |
                          (/cmd_vel_raw)
                                |
                                v
                     +---------------------+
                     |  velocity_smoother  |
                     +---------------------+
                                |
                        (/cmd_vel_smoothed)
                                |
                                v
                     +---------------------+
                     |   safety_watchdog   |  (Kẹp giới hạn tuyệt đối &
                     +---------------------+   Tự ngắt về 0 khi mất mạng)
                                |
                            (/cmd_vel)
                                |
                                v
                     [ Unitree G1 SDK Bridge ]
```

---

## 📋 Chi Tiết 4 Bước Thực Hiện

### Bước 1: Cấu hình Bộ trộn Ưu tiên (`twist_mux`)
Tạo file cấu hình xếp thứ tự ưu tiên điều khiển (Manual Teleop luôn được ưu tiên hơn Nav2 Tự động).

- **Tệp tạo mới**: `src/g1_navigation_nav2/config/twist_mux.yaml`
- **Nội dung cấu hình**:
  - Nguồn 1: `/cmd_vel_teleop` (Ưu tiên: 100, Timeout: 0.5s)
  - Nguồn 2: `/cmd_vel_nav` (Ưu tiên: 50, Timeout: 0.5s)
  - Đầu ra: `/cmd_vel_raw`

---

### Bước 2: Cấu hình Bộ làm mượt Vận tốc (`nav2_velocity_smoother`)
Bổ sung node `velocity_smoother` vào file `nav2_params.yaml` để nội suy mượt gia tốc và vận tốc trước khi truyền tới robot.

- **Tệp chỉnh sửa**: `src/g1_navigation_nav2/config/nav2_params.yaml`
- **Các thông số an toàn khuyến nghị cho Unitree G1**:
  - `smoothing_frequency`: `20.0` Hz
  - `scale_velocities`: `false`
  - `max_velocities`: `[0.4, 0.15, 0.5]` (lần lượt vx, vy, vtheta)
  - `min_velocities`: `[-0.2, -0.15, -0.5]`
  - `max_accelerations`: `[0.4, 0.3, 0.8]` (gia tốc tăng tốc mượt)
  - `max_decelerations`: `[-0.5, -0.4, -1.0]` (giảm tốc an toàn)

---

### Bước 3: Tạo Node Giám Sát An Toàn (`g1_safety_watchdog_node`)
Xây dựng một ROS 2 Python/C++ node nhẹ đóng vai trò là lớp phòng thủ cuối cùng (Last-line Defense) trước khi lệnh đi vào SDK hardware của Unitree.

- **Nhiệm vụ chính**:
  1. **Watchdog Timeout**: Nếu không có lệnh mới trong `0.5s`, tự phát lệnh dừng (`vx=0, vy=0, vtheta=0`).
  2. **Soft Limit Clamping**: Giới hạn cứng giá trị cực đại để phòng trường hợp lỗi phần mềm phía Nav2.
  3. **E-Stop Check**: Tích hợp công tắc dừng khẩn cấp mềm (service/topic e-stop).
- **Tệp tạo mới**: `src/g1_navigation_nav2/scripts/g1_safety_watchdog.py`

---

### Bước 4: Tích Hợp Vào Launch File (`g1_navigation_nav2.launch.py`)
Cập nhật file launch để khởi chạy đồng bộ các thành phần trên.

- **Tệp chỉnh sửa**: `src/g1_navigation_nav2/launch/g1_navigation_nav2.launch.py`
- **Nội dung thay đổi**:
  - Khởi chạy node `twist_mux` (gói `twist_mux`).
  - Khởi chạy node `velocity_smoother` (gói `nav2_velocity_smoother`).
  - Đưa `velocity_smoother` vào danh sách `lifecycle_nodes` để Lifecycle Manager tự động quản lý trạng thái (`Configure` -> `Activate`).
  - Khởi chạy node `g1_safety_watchdog`.

---

## 🎯 Kết Quả Sau Khi Hoàn Thành
- Robot Unitree G1 di chuyển **mượt mà, không bị giật** khi xuất phát hoặc dừng lại.
- Dễ dàng **can thiệp điều khiển bằng tay** qua Joystick/Teleop bất kỳ lúc nào Nav2 đang tự hành.
- **An toàn tuyệt đối** trước các nguy cơ rớt WiFi hay mất kết nối tín hiệu điều khiển.
