# g1_state_estimator

Gói ROS 2 ước lượng trạng thái (State Estimator) sử dụng thuật toán **Error-State Iterative Kalman Filter (ES-IEKF)** thiết kế riêng cho robot humanoid **Unitree G1**. Bộ lọc thực hiện dung hợp dữ liệu từ:
1. **IMU Hông (`/dog_imu_raw`)**: Tần số cao (~768 Hz) dùng cho bước dự báo (Prediction).
2. **Vận tốc chân (`/dog_odom`)**: Dùng cho bước cập nhật đo đạc vận tốc tuyến tính trong hệ trục robot (Leg Odometry).
3. **FAST-LIO (`/Odometry`)**: Dùng để hiệu chỉnh trôi (drift) vị trí và hướng (LIO Pose Update) qua cơ chế rollback (History Buffer) để bù trừ trễ thời gian xử lý của LiDAR.

---

## 1. Cấu trúc Package

```text
g1_state_estimator/
├── CMakeLists.txt
├── package.xml
├── README.md                 <-- (Tài liệu này)
├── config/
│   └── estimator.yaml        <-- Cấu hình tham số (noise, topics, frames)
├── launch/
│   └── estimator.launch.py   <-- Launch file khởi chạy node
├── include/g1_state_estimator/
│   ├── state.h               <-- Cấu trúc State vector 15D & phép toán Boxplus
│   ├── estimator.h           <-- Interface lõi lọc IEKF
│   └── estimator_node.h      <-- Interface Node ROS 2 wrapper
└── src/
    ├── estimator.cpp         <-- Chi tiết toán học IEKF & Rollback Buffer
    ├── estimator_node.cpp    <-- Xử lý callback, tra cứu TF & phát xuất dữ liệu
    └── main.cpp              <-- Entry point khởi chạy
```

---

## 2. Cấu hình tham số (`config/estimator.yaml`)

File cấu hình chính nằm tại `config/estimator.yaml`. Bạn có thể chỉnh sửa các tham số sau mà không cần biên dịch lại code:

| Tên tham số | Kiểu dữ liệu | Giá trị mặc định | Giải thích |
| :--- | :--- | :--- | :--- |
| **`imu_topic`** | string | `/dog_imu_raw` | Topic IMU chính từ hông robot (kiểu `sensor_msgs/msg/Imu`). |
| **`lio_topic`** | string | `/Odometry` | Topic chứa pose ước lượng từ FAST-LIO (kiểu `nav_msgs/msg/Odometry`). |
| **`leg_topic`** | string | `/dog_odom` | Topic chứa vận tốc chân của robot (kiểu `nav_msgs/msg/Odometry`). |
| **`map_frame`** | string | `map` | Gốc hệ tọa độ bản đồ toàn cầu (FAST-LIO). |
| **`odom_frame`** | string | `odom` | Hệ tọa độ Odometry xuất phát của bộ lọc. |
| **`base_frame`** | string | `pelvis` | Khung tọa độ gốc của robot (khung chậu). |
| **`gyro_noise`** | double | `1e-4` | Độ nhiễu gyroscope (độ lớn phương sai). |
| **`acc_noise`** | double | `1e-3` | Độ nhiễu accelerometer (độ lớn phương sai). |
| **`gyro_bias_noise`** | double | `1e-6` | Tốc độ trôi sai số (bias random walk) của Gyroscope. |
| **`acc_bias_noise`** | double | `1e-5` | Tốc độ trôi sai số (bias random walk) của Accelerometer. |
| **`lio_pos_noise`** | double | `1e-3` | Phương sai đo đạc vị trí từ FAST-LIO (càng nhỏ bộ lọc càng tin LIO). |
| **`lio_ori_noise`** | double | `1e-3` | Phương sai đo đạc góc quay từ FAST-LIO. |
| **`leg_vel_noise`** | double | `1e-2` | Phương sai đo đạc vận tốc chân (càng nhỏ bộ lọc càng tin Leg Odom). |
| **`history_length_sec`**| double | `2.0` | Khoảng thời gian lưu trữ lịch sử trạng thái để rollback (giây). |
| **`extrinsic_x`** | double | `0.03314` | Khoảng cách dịch chuyển trục X từ LiDAR IMU tới Pelvis (Fallback). |
| **`extrinsic_y`** | double | `0.02332` | Khoảng cách dịch chuyển trục Y từ LiDAR IMU tới Pelvis (Fallback). |
| **`extrinsic_z`** | double | `0.41554` | Khoảng cách dịch chuyển trục Z từ LiDAR IMU tới Pelvis (Fallback). |
| **`extrinsic_roll`** | double | `-3.14159` | Góc lệch xoay Roll giữa LiDAR IMU và Pelvis (Fallback). |
| **`extrinsic_pitch`**| double | `-0.04014` | Góc lệch xoay Pitch giữa LiDAR IMU và Pelvis (Fallback). |
| **`extrinsic_yaw`** | double | `0.0` | Góc lệch xoay Yaw giữa LiDAR IMU và Pelvis (Fallback). |

*Lưu ý: Các thông số `extrinsic_*` chỉ được dùng làm giá trị dự phòng nếu hệ thống không tra cứu được TF động giữa frame đầu ra của FAST-LIO (ví dụ: `body`) và `pelvis` thông qua TF tree.*

---

## 3. Cách biên dịch và chạy

### Biên dịch:
Di chuyển đến thư mục workspace và chạy lệnh:
```bash
colcon build --packages-select g1_state_estimator --symlink-install
```

### Khởi chạy:
1. Mở terminal và source workspace:
   ```bash
   source install/setup.bash
   ```
2. Chạy file launch:
   ```bash
   ros2 launch g1_state_estimator estimator.launch.py
   ```

---

## 4. Dữ liệu đầu vào và đầu ra (Inputs & Outputs)

### Đầu vào (Subscribed Topics):
* `/dog_imu_raw` (`sensor_msgs/msg/Imu`): Dữ liệu gia tốc và vận tốc góc của pelvis.
* `/dog_odom` (`nav_msgs/msg/Odometry`): Vận tốc tuyến tính của pelvis trong body-frame.
* `/Odometry` (`nav_msgs/msg/Odometry`): Trạng thái pose từ FAST-LIO (vị trí/hướng của LiDAR).

### Đầu ra (Published Topics & TFs):
* **`/state_estimator/odom`** (`nav_msgs/msg/Odometry`):
  * **Tần số**: Bằng tần số IMU (~768 Hz).
  * **Vị trí và Hướng (`pose.pose`)**: Ước lượng của gốc `pelvis` trong hệ tọa độ `odom` toàn cầu (đã fusion trơn tru không bị giật lùi).
  * **Vận tốc (`twist.twist.linear`)**: Vận tốc tuyến tính 3 chiều của robot được biểu diễn trong **body-frame** (`pelvis` frame), phù hợp với tiêu chuẩn ROS 2 điều khiển di chuyển.
* **TF động (`odom -> pelvis`)**:
  * Phát hành liên tục phép biến đổi tọa độ động giữa hệ odom và khung chậu robot để hiển thị chính xác vị trí robot trên RViz.

---

## 5. Nguyên lý bù trễ thời gian (Rollback & Re-propagate)

Do quá trình xử lý LiDAR của FAST-LIO mất từ **30ms - 80ms**, dữ liệu pose nhận được tại thời điểm thực tế luôn đại diện cho trạng thái của robot trong quá khứ ($t_{meas} < t_{current}$). Nếu sử dụng phương pháp EKF thông thường nạp trực tiếp, bộ lọc sẽ bị lỗi tính toán và gây ra hiện tượng nhảy vọt trạng thái (state jumps).

Bộ ước lượng này giải quyết triệt để bằng cách:
1. Lưu giữ một hàng đợi lịch sử (`std::map`) gồm các trạng thái (State), ma trận hiệp phương sai (Covariance) và dữ liệu IMU trong vòng 2 giây gần nhất.
2. Khi nhận được tin nhắn `/Odometry` trễ tại thời điểm $t_{meas}$:
   * Tìm trạng thái lịch sử gần nhất với $t_{meas}$ trong bộ đệm.
   * Quay ngược thời gian bộ lọc về thời điểm đó (Rollback).
   * Thực hiện cập nhật đo đạc IEKF với pose của LIO để sửa lỗi sai lệch vị trí/hướng tại thời điểm quá khứ.
   * Lấy toàn bộ các gói IMU từ $t_{meas}$ đến $t_{current}$ để tính toán tích phân dự báo đẩy nhanh trạng thái về thời điểm hiện tại (Re-propagate).
3. Cập nhật lại giá trị hiện tại và tiếp tục chạy dự báo thời gian thực.

---

## 6. Bộ hiệu chỉnh nhãn thời gian (Timestamp Corrector Node)

Do LiDAR Livox Mid-360 và IMU của robot G1 xuất bản dữ liệu sử dụng nhãn thời gian từ đồng hồ phần cứng riêng (Hardware Clock), chúng có thể bị lệch đáng kể so với đồng hồ của máy tính nhúng (Host PC) (đặc biệt khi khởi động hoặc reset driver). Nếu độ lệch này vượt quá thời gian lưu đệm TF (10 giây), các gói dữ liệu mây điểm và IMU sẽ bị lỗi `Unknown frame` hoặc `Transform data too old` trên RViz và Nav2.

Node `timestamp_corrector_node` giải quyết vấn đề này bằng cách:
1. **Đăng ký nhận dữ liệu từ các cảm biến:**
   * LiDAR: `/utlidar/cloud_livox_mid360`
   * IMU: `/utlidar/imu_livox_mid360`
2. **Ước lượng độ lệch thời gian (time offset) giữa hệ thống và cảm biến bằng thuật toán:**
   * **Bù trừ tức thời khi khởi tạo:** Đặt offset ban đầu bằng chênh lệch thời gian nhận tin nhắn đầu tiên.
   * **Nhận diện bước nhảy thời gian (Jump Detection):** Nếu độ lệch nhảy đột ngột vượt quá `100ms`, lập tức đặt lại offset để đồng bộ ngay lập tức.
   * **Theo dõi độ trôi nhiệt siêu chậm:** Khi hoạt động bình thường, độ lệch được cập nhật qua bộ lọc EMA cực kỳ chậm (hệ số $\alpha = 0.0001$ ở tần số 200Hz) để theo dõi độ trôi của đồng hồ phần cứng mà không gây ra bất kỳ jitter nào cho khoảng cách thời gian ($dt$) giữa các gói tin.
3. **Xuất bản dữ liệu đã hiệu chỉnh thời gian sang các topic đồng bộ:**
   * LiDAR: `/utlidar/cloud_livox_mid360_sync`
   * IMU: `/utlidar/imu_livox_mid360_sync`
