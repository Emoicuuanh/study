# Hướng dẫn thiết lập và chạy FAST-LIO với Livox Mid-360 trên Unitree G1 (ROS 2 Foxy)

Tài liệu này hướng dẫn cách kết nối phần cứng, cấu hình phần mềm và chạy thuật toán **FAST-LIO** với thiết bị LiDAR **Livox Mid-360** trên robot thật Unitree G1.

---

## 1. Kết nối phần cứng và cấu hình Mạng

### Kết nối vật lý
- Kết nối cáp Ethernet của LiDAR Mid-360 vào máy tính điều khiển (máy chạy ROS 2 Foxy).
- Cấp nguồn cho LiDAR (sử dụng nguồn DC 9-30V phù hợp).

### Cấu hình địa chỉ IP tĩnh trên Máy tính (Host)
Mặc định LiDAR Mid-360 trên robot Unitree thường có địa chỉ IP tĩnh là `192.168.123.120`. Để kết nối được, bạn cần cấu hình IP tĩnh trên cổng mạng Ethernet của Máy tính:
- **IP Address (Địa chỉ IP)**: `192.168.123.99`
- **Netmask (Mặt nạ mạng)**: `255.255.255.0`
- **Gateway**: Để trống hoặc `192.168.123.1`

> **Lưu ý**: Bạn có thể kiểm tra kết nối mạng bằng cách ping thử tới LiDAR:
> ```bash
> ping 192.168.123.120
> ```

---

## 2. Kiểm tra Cấu hình Driver (`livox_ros_driver2`)

Trước khi chạy mapping, đảm bảo driver LiDAR được cấu hình đúng để tìm thấy thiết bị và phát ra định dạng dữ liệu tối ưu cho FAST-LIO.

### Kiểm tra File cấu hình của Driver:
Mở file `src/livox_ros_driver2/config/MID360_config.json` và kiểm tra các địa chỉ IP:
```json
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.123.99",
      "point_data_ip": "192.168.123.99",
      "imu_data_ip": "192.168.123.99"
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.123.120",
      "pcl_data_type": 1
    }
  ]
}
```
- **`cmd_data_ip` / `point_data_ip`**: Phải khớp với IP tĩnh của Máy tính (`192.168.123.99`).
- **`ip`**: Khớp với IP tĩnh của LiDAR Mid-360 (`192.168.123.120`).

---

## 3. Cấu hình Tham số FAST-LIO (`mid360.yaml`)

File cấu hình chính của thuật toán nằm tại: `src/FAST_LIO_ROS2/config/mid360.yaml`.

Các thông số quan trọng cần lưu ý:
1. **Topic nhận dữ liệu**:
   - `common.lid_topic`: `"/livox/lidar"` (Topic Point Cloud dạng CustomMsg).
   - `common.imu_topic`: `"/livox/imu"` (Topic dữ liệu IMU tích hợp).
2. **Kiểu LiDAR**:
   - `preprocess.lidar_type`: `1` (Định dạng CustomMsg của Livox, hoạt động tối ưu nhất cho FAST-LIO).
3. **Vùng mù (`blind`)**:
   - `preprocess.blind`: `0.5` (Bỏ qua các điểm trong bán kính 0.5m để tránh quét trúng chính cơ thể robot Unitree G1). Tăng lên `0.6` hoặc `0.7` nếu robot tự nhận nhầm cơ thể mình là vật cản.
4. **Tọa độ tương quan (Extrinsics)**:
   - `mapping.extrinsic_T`: `[ -0.011, -0.02329, 0.04412 ]` (Vị trí tương quan vật lý giữa LiDAR và IMU bên trong Mid-360).
   - `mapping.extrinsic_est_en`: `true` (Cho phép tự động tối ưu hóa sai lệch tọa độ trong khi chạy).

---

## 4. Các bước Khởi chạy trên Robot Thật

Thực hiện theo trình tự sau (sử dụng các terminal riêng biệt):

### Bước 1: Khởi động Driver LiDAR Mid-360
Mở terminal mới, nạp môi trường ROS 2 Foxy và khởi chạy driver để phát dữ liệu Point Cloud và IMU:
```bash
cd ~/ROS2/unitree_G1/dev_unitreeg1_ws
source /opt/ros/foxy/setup.zsh
source install/setup.zsh

# Chạy driver phát CustomMsg
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Bước 2: Khởi chạy FAST-LIO Mapping
Mở một terminal khác, nạp môi trường và chạy node định vị và xây dựng bản đồ:
```bash
cd ~/ROS2/unitree_G1/dev_unitreeg1_ws
source /opt/ros/foxy/setup.zsh
source install/setup.zsh

# Khởi chạy thuật toán mapping
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```
*Node này cũng sẽ tự động mở giao diện **RViz** để bạn trực quan hóa đám mây điểm và quỹ đạo di chuyển của robot.*

---

## 5. Lưu bản đồ Point Cloud (PCD)

Khi đã quét xong khu vực mong muốn và muốn lưu lại bản đồ dưới dạng file `.pcd`:

### Cách 1: Lưu tự động khi tắt Node (Mặc định)
Khi bạn nhấn `Ctrl + C` để tắt terminal chạy **FAST-LIO**, node sẽ tự động lưu bản đồ tích lũy vào đường dẫn được chỉ định ở tham số `map_file_path` trong file `mid360.yaml` (mặc định là `./test.pcd`).

### Cách 2: Gọi Service để lưu bản đồ thủ công
Mở một terminal mới và chạy lệnh gọi Service:
```bash
ros2 service call /map_save std_srvs/srv/Trigger {}
```
Hệ thống sẽ ghi file bản đồ ngay lập tức mà không cần tắt chương trình.

---

> **Cảnh báo**: Luôn đảm bảo bạn đang ở môi trường terminal **sạch** (chỉ source ROS 2 Foxy, **không** source ROS 1 Noetic chung một terminal) để tránh xung đột môi trường Python và thư viện liên kết.
