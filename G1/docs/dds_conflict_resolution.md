# Hướng dẫn xử lý xung đột mạng ROS 2 DDS (Unitree G1)

Tài liệu này ghi lại chi tiết về lỗi xung đột mạng DDS xảy ra trên mạng robot Unitree G1 (`192.168.123.X`) và các bước xử lý triệt để bằng tường lửa mà không cần thay đổi tên frame TF hay ảnh hưởng đến multicast nội bộ.

---

## 🔍 Mô tả lỗi xung đột

Khi có nhiều máy tính cùng kết nối vào mạng của robot Unitree G1 (sử dụng cơ chế tự động tìm kiếm multicast mặc định và chung `ROS_DOMAIN_ID=0`), các gói tin chủ đề (topics) và biến đổi tọa độ `/tf` giữa các máy sẽ bị nhận lẫn chéo nhau.

### Triệu chứng cụ thể:
* Trên RViz, khung xương (TF tree) của robot bị giật, nhấp nháy liên tục giữa trạng thái thực tế của robot bạn điều khiển và robot của máy khác.
* Các frame như `pelvis`, `torso_link` và các khớp động bị ghi đè, kéo lệch trục do nhận nhầm gốc tọa độ từ máy tính khác.
* Các lệnh ROS 2 như `ros2 node list` hoặc `ros2 topic list` có thể bị lỗi crash `UnicodeDecodeError` do nhận phải các chuỗi dữ liệu rác không kết thúc bằng ký tự null từ SDK của robot hoặc các máy khác gửi lên mạng DDS.

---

## 🛠️ Phân tích địa chỉ IP trong mạng

Qua quét lưu lượng tìm kiếm DDS Discovery trên dải mạng `192.168.123.X`, các thiết bị đang hoạt động được xác định như sau:

| Địa chỉ IP | Vai trò thiết bị | Trạng thái |
| :--- | :--- | :--- |
| `192.168.123.100` | **Máy tính của bạn (Host PC)** | Cục bộ |
| `192.168.123.161` | **CPU điều khiển chuyển động của G1 (Onboard)** | An toàn (Bắt buộc giữ) |
| `192.168.123.164` | **Nvidia Orin PC của G1 (Onboard)** | An toàn (Bắt buộc giữ) |
| `192.168.123.150` | **Máy tính đang xung đột (Của đồng nghiệp/máy khác)** | **Cần chặn (Target IP)** |

---

## 🛡️ Giải pháp khắc phục: Lọc mạng bằng tường lửa hệ thống

Vì việc đặt tiền tố tên khớp (ví dụ: `g1/pelvis`) sẽ làm hỏng khả năng tương thích với nhiều thư viện và thuật toán xử lý khác, cách tốt nhất là **chặn toàn bộ gói tin đến từ IP gây xung đột (`192.168.123.150`)** thông qua tường lửa hệ thống `iptables`.

Việc này giúp máy bạn hoàn toàn bỏ qua các gói tin quảng bá DDS từ máy tính kia mà vẫn giao tiếp bình thường với robot.

### Bước 1: Áp dụng lệnh chặn IP xung đột
Chạy lệnh sau trên terminal của bạn:
```bash
sudo iptables -A INPUT -s 192.168.123.150 -j DROP
```

### Bước 2: Xác nhận luật chặn đã hoạt động
Để liệt kê danh sách quy tắc tường lửa và đảm bảo IP đã bị chặn thành công:
```bash
sudo iptables -L INPUT -v -n
```
Bạn sẽ thấy một dòng quy tắc có hành động `DROP` trỏ tới nguồn `192.168.123.150`.

### Bước 3: Rebuild lại workspace và khởi chạy bình thường
Sau khi thiết lập tường lửa, khởi động lại các nút của bạn:
```zsh
colcon build --symlink-install --packages-select g1_bringup
sr2
ros2 launch g1_bringup g1_bringup.launch.py
```
Lúc này, cây TF trên máy của bạn sẽ hoàn toàn độc lập, sạch sẽ và không còn bị giật lắc.

---

## 🔓 Cách gỡ chặn IP trong tương lai

Nếu sau này bạn cần kết nối hoặc nhận dữ liệu trực tiếp từ máy tính đó, bạn có thể xóa luật chặn bằng lệnh sau:
```bash
sudo iptables -D INPUT -s 192.168.123.150 -j DROP
```
