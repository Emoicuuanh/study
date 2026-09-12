# Unitree G1 Camera Package (`unitree_camera`)

Gói ROS 2 chuyên dụng giao tiếp camera Intel RealSense D435/D435i trên Robot bipedal **Unitree G1**, tối ưu nén ảnh truyền siêu tốc qua mạng Wi-Fi về Laptop điều khiển.

---

## 🚀 1. Hướng dẫn sử dụng (Usage)

### Bước 1: Khởi động Driver RealSense C++ (Trên Robot Unitree G1)
Mở Terminal trên Robot và chạy lệnh khởi động driver phần cứng:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=false \
  enable_gyro:=false \
  enable_accel:=false \
  align_depth.enable:=false \
  pointcloud.enable:=false \
  color_width:=640 \
  color_height:=480 \
  color_fps:=15
```

### Bước 2: Khởi động Node Nén ảnh truyền qua Wi-Fi (Trên Robot Unitree G1)
Mở Terminal thứ 2 trên Robot và chạy node nén ảnh siêu nhẹ:
```bash
ros2 run unitree_camera camera_node
```

### Bước 3: Xem hình ảnh trực quan (Trên Laptop)
* **Kiểm tra topic nén trên Laptop**:
  ```bash
  ros2 topic hz /camera/color/compressed --qos-reliability best_effort
  ```
* **Xem trên RViz2**:
  1. Mở **RViz2** trên Laptop.
  2. Thêm Display loại **Image**.
  3. Chọn Topic: **`/camera/color/compressed`**.

---

## 🛠️ 2. Chẩn đoán Lỗi & Cách Khắc Phục (Troubleshooting)

### ❌ Lỗi 1: Tràn bộ nhớ `bad_alloc` do cờ `FORCE_RSUSB_BACKEND=true`
* **Mô tả lỗi**: Node camera chạy được vài giây thì báo `bad_alloc caught: std::bad_alloc` liên tục và bị Linux Kernel tự động ngắt (`process has died [exit code -9]`).
* **Nguyên nhân kĩ thuật**:
  File cài đặt ban đầu biên dịch thư viện `librealsense2` với cờ `-DFORCE_RSUSB_BACKEND=true`, ép camera chạy qua thư viện người dùng `libusb` thay vì V4L2 kernel driver chuẩn (`/dev/video*`).
  Trên chip xử lý ARM (NVIDIA Tegra/Jetson của Unitree G1), `RSUSB_BACKEND` làm kẹt vòng lặp bộ đệm USB, gây ra lỗi `bad_alloc caught: std::bad_alloc` liên tục và làm Linux Kernel tự động diệt tiến trình (`kill -9`).
* **Cách khắc phục**:
  Biên dịch lại thư viện `librealsense2` trực tiếp trên Robot với cờ **`-DFORCE_RSUSB_BACKEND=false`**:
  ```bash
  cd ~/librealsense/build
  cmake ../ -DCMAKE_BUILD_TYPE=Release \
            -DBUILD_EXAMPLES=false \
            -DBUILD_GRAPHICAL_EXAMPLES=false \
            -DFORCE_RSUSB_BACKEND=false \
            -DBUILD_WITH_CUDA=false
  make -j4
  sudo make install
  sudo ldconfig
  ```
  Sau đó rebuild lại workspace ROS 2:
  ```bash
  cd ~/realsense_ws
  rm -rf build install log
  colcon build --symlink-install
  ```

---

### ❌ Lỗi 2: Tràn số thời gian RTC năm 1970 (Nanosecond Overflow)
* **Mô tả lỗi**: Node bị sập ngay khi bật hoặc DDS báo lỗi thời gian.
* **Nguyên nhân**: Khi Robot cạn pin hoàn toàn, đồng hồ RTC trên Robot bị trả về năm 1970 (`Wed Mar 4 1970`). Trong ROS 2 Foxy, tính toán nanosecond `rclcpp::Clock::now()` từ mốc 1970 bị lệch dải số so với mốc thời gian hiện tại, làm FastDDS bị sập.
* **Cách khắc phục**: Đồng bộ lại giờ hệ thống Robot về thời gian hiện tại:
  ```bash
  echo 123 | sudo -S date -s "2026-07-28 08:08:00"
  ```

---

### ❌ Lỗi 3: Lệch màu xanh nước biển (Blue Tint) trên RViz2
* **Nguyên nhân**: Thư viện `pyrealsense2` / V4L2 xuất luồng dữ liệu byte ở dạng **BGR**, trong khi RViz2 mặc định giải mã theo định dạng **RGB**.
* **Cách khắc phục**: Thêm bước chuyển đổi hệ màu bằng OpenCV trong `camera_node.py`:
  ```python
  color_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
  msg_color = self.bridge.cv2_to_imgmsg(color_rgb, encoding="rgb8")
  ```

---

### ❌ Lỗi 4: Lỗi Xung đột kiểu dữ liệu Topic (`Topic Type Mismatch`)
* **Mô tả lỗi**: `Cannot echo topic '/camera/color/image_raw/compressed', as it contains more than one type: [sensor_msgs/msg/CompressedImage, sensor_msgs/msg/Image]`.
* **Nguyên nhân**: Plugin `image_transport` mặc định của ROS 2 đã đăng ký sẵn tên `/camera/color/image_raw/compressed` dưới kiểu `sensor_msgs/msg/Image`. Việc đăng ký trùng tên khác Message Type gây xung đột ROS 2 DDS.
* **Cách khắc phục**: Đổi tên topic nén phát ra thành duy nhất: **`/camera/color/compressed`** (`sensor_msgs/msg/CompressedImage`).

---

### ❌ Lỗi 5: Hz bị tụt khi truyền qua mạng Wi-Fi (NACK Retransmit Storm)
* **Nguyên nhân**: Publisher cũ dùng QoS `RELIABLE`. Khi truyền dữ liệu nén qua sóng Wi-Fi (có nhiễu không dây), chỉ cần rớt 1 gói UDP, `RELIABLE` sẽ bắt phát lại (NACK storm), làm dồn đệm bộ nhớ và tụt tốc độ trên Laptop từ 15 Hz xuống còn 5 Hz.
* **Cách khắc phục**: Cấu hình `QoSProfile` chuẩn **`BEST_EFFORT` + `depth=1`** cho cả Publisher và Subscriber:
  ```python
  sensor_qos = QoSProfile(
      reliability=ReliabilityPolicy.BEST_EFFORT,
      durability=DurabilityPolicy.VOLATILE,
      history=HistoryPolicy.KEEP_LAST,
      depth=1  # Luôn chỉ bắt khung hình mới nhất, loại bỏ hàng đợi dồn toa
  )
  ```

---

## 📌 3. Thông số kỹ thuật của Node Nén (`camera_node.py`)

* **Input Topic**: `/camera/color/image_raw` (`sensor_msgs/msg/Image`)
* **Output Topic**: `/camera/color/compressed` (`sensor_msgs/msg/CompressedImage`)
* **Định dạng nén**: JPEG (`quality = 75%`)
* **Hiệu quả nén**: Dung lượng giảm từ **~900 KB/frame** xuống còn **~18 - 35 KB/frame** (Giảm 95% băng thông, truyền Wi-Fi ổn định 15 FPS liên tục với độ trễ xấp xỉ 0ms).
