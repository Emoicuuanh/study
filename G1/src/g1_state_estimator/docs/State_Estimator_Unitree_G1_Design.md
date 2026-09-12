# State Estimator cho Unitree G1 (ES-IEKF)

> Tài liệu tóm tắt thiết kế bộ State Estimator sử dụng Error-State Iterated Kalman Filter (ES-IEKF) cho robot humanoid Unitree G1.

---

## 1. Mục tiêu

State Estimator có nhiệm vụ hợp nhất (Sensor Fusion) dữ liệu từ ba nguồn chính:

* **IMU (Pelvis):** Phục vụ bước dự đoán (Prediction) ở tần số cao (~900Hz).
* **FAST-LIO (/Odometry):** Cung cấp vị trí và tư thế toàn cục, chạy ở tần số thấp (~10Hz) và có độ trễ.
* **Leg Odometry (/dog_odom):** Cung cấp vận tốc tuyến tính trong hệ trục robot (Body-frame velocity) ở tần số cao (~900Hz).

Mục tiêu là ước lượng liên tục trạng thái robot bao gồm: Vị trí (Position), Vận tốc (Velocity), Tư thế (Orientation) và Sai số IMU (Bias Gyro/Acc) mà không bị trôi trục đứng (Z-drift).

```
                 IMU (Pelvis) ~900Hz
                      │
                 Prediction
                      │
          ┌───────────┴───────────┐
          │                       │
     FAST-LIO (Trễ)          /dog_odom (Chân)
          │                       │
          └──────────Update────────┘
                      │
                  ES-IEKF State
                      │
         ┌────────────┴────────────┐
         ▼                         ▼
   /state_estimator/odom     /state_estimator/path (10Hz)
```

---

## 2. Kiến trúc tổng thể

Bộ ước lượng hoạt động theo mô hình dự đoán và hiệu chỉnh liên tục:

1. **Prediction Step (900Hz):** Khi có IMU mới, hệ thống tích phân trạng thái danh nghĩa và lan truyền hiệp phương sai sai số $P$.
2. **Correction Step (Tức thời - 900Hz):** Cập nhật ngay lập tức vận tốc đo từ chân robot (`UpdateLeg`) để "khóa" vận tốc và chiều cao của robot.
3. **Rollback & Correction Step (Trễ - 10Hz):** Khi nhận được gói tin định vị từ LIO (FAST-LIO), quay ngược thời gian (Rollback) về thời điểm của gói tin, áp dụng cập nhật LIO (`UpdateLIO`) để sửa sai số tích lũy của vị trí, sau đó dự đoán lại (Re-propagate) trạng thái tới thời điểm hiện tại.

---

## 3. Trạng thái của bộ lọc

Vector trạng thái danh nghĩa (Nominal State) gồm 16 thành phần:

$$
x = \begin{bmatrix} p \\ v \\ q \\ b_g \\ b_a \end{bmatrix}
$$

Trong đó:

* **Position ($p$):** Tọa độ 3D của pelvis trong khung `odom` ($3 \times 1$).
* **Velocity ($v$):** Vận tốc tuyến tính 3D của pelvis trong khung `odom` ($3 \times 1$).
* **Orientation ($q$):** Quaternion biểu diễn tư thế pelvis ($4 \times 1$).
* **Gyroscope Bias ($b_g$):** Sai số dịch của con quay hồi chuyển ($3 \times 1$).
* **Accelerometer Bias ($b_a$):** Sai số dịch của cảm biến gia tốc ($3 \times 1$).

---

## 4. Trạng thái lỗi (Error State)

Để tránh điểm kỳ dị (singularity) khi biểu diễn quaternion, ES-IEKF sử dụng vector trạng thái lỗi 15 chiều đại lượng vector tối thiểu:

$$
\delta x = \begin{bmatrix} \delta p \\ \delta v \\ \delta \theta \\ \delta b_g \\ \delta b_a \end{bmatrix}
$$

* $\delta \theta \in \mathbb{R}^3$ biểu diễn sai số góc nhỏ xoay quanh các trục.
* Phép cộng trạng thái (Boxplus $\oplus$) được định nghĩa như sau:
  $$
  x \oplus \delta x = \begin{bmatrix} p + \delta p \\ v + \delta v \\ \exp(\delta \theta) \otimes q \\ b_g + \delta b_g \\ b_a + \delta b_a \end{bmatrix}
  $$

---

## 5. Mô hình dự đoán (Prediction Model - IMU)

Tích phân Euler cải tiến từ dữ liệu IMU:

* **Tư thế:** $\omega_k = \omega_{\text{imu}} - b_{g,k} \implies q_{k+1} = q_k \otimes \text{Exp}(\omega_k \Delta t)$
* **Vận tốc:** $v_{k+1} = v_k + (R(q_k)(a_{\text{imu}} - b_{a,k}) + g)\Delta t$
* **Vị trí:** $p_{k+1} = p_k + v_k \Delta t + \frac{1}{2}(R(q_k)(a_{\text{imu}} - b_{a,k}) + g)\Delta t^2$

Lan truyền ma trận hiệp phương sai sai số:

$$
P_{k+1} = F_k P_k F_k^T + G_k Q G_k^T
$$

---

## 6. Mô hình đo LIO và cơ chế lọc trôi Z (Anisotropic Noise Model)

FAST-LIO cung cấp vị trí và tư thế pelvis dưới dạng phép đo tuyệt đối trong hệ tọa độ `camera_init` (được ánh xạ sang khung `odom` thông qua transform tĩnh):

$$
z_{\text{lio}} = \begin{bmatrix} p_{\text{lio}} \\ q_{\text{lio}} \end{bmatrix}
$$

### Mô hình nhiễu không đẳng hướng (Anisotropic / Per-Axis Noise)

Để giải quyết triệt để lỗi trôi trục đứng (Z-drift) đặc trưng của LiDAR trong môi trường phẳng, bộ lọc đã cấu hình ma trận hiệp phương sai nhiễu đo $R_{\text{lio}}$ độc lập cho từng trục bằng cách sử dụng cấu trúc ma trận đường chéo từ vector 3 chiều:

$$
R_{\text{pos}} = \text{diag}(\sigma_{px}^2, \sigma_{py}^2, \sigma_{pz}^2)
$$

$$
R_{\text{ori}} = \text{diag}(\sigma_{r\_roll}^2, \sigma_{r\_pitch}^2, \sigma_{r\_yaw}^2)
$$

* **Cấu hình chống trôi Z:** Đặt độ nhiễu trục đứng $\sigma_{pz}^2$ rất lớn (ví dụ: `0.1` so với X, Y là `0.001`). Khi đó, EKF coi dữ liệu Z từ LiDAR là không đáng tin cậy, hầu như bỏ qua nó và ưu tiên dùng ràng buộc trọng lực (gravity) từ IMU kết hợp với động học chân để giữ Z ổn định ở đúng chiều cao đứng thực tế.

---

## 7. Mô hình đo Động học chân (Leg Odometry)

Vận tốc tuyến tính trong hệ trục robot (Body-frame velocity) được trích xuất trực tiếp từ topic `/dog_odom` (đã được bộ ước lượng onboard của Unitree lọc tiếp xúc sẵn):

$$
z_{\text{leg}} = v_{\text{body}}
$$

Mô hình đo lường:

$$
h(x) = R(q)^T v
$$

Residual (Sai số đo):

$$
r_{\text{leg}} = v_{\text{body}} - R(q)^T v
$$

Hiệp phương sai nhiễu đo của chân cũng được cấu hình theo từng trục để phản ánh chính xác sai số trượt chân:

$$
R_{\text{leg}} = \text{diag}(\sigma_{vx}^2, \sigma_{vy}^2, \sigma_{vz}^2)
$$

---

## 8. Cơ chế History Buffer xử lý trễ từ FAST-LIO

Dữ liệu đầu ra của FAST-LIO có độ trễ lớn (~30-80ms). Quy trình xử lý trễ:

1. **Lưu trữ lịch sử:** Bộ lọc liên tục lưu các trạng thái danh nghĩa `x`, hiệp phương sai `P`, và dữ liệu IMU vào `std::map` sắp xếp theo thời gian (giới hạn thời gian lưu là 2.0 giây).
2. **Rollback (Quay ngược thời gian):** Khi có gói tin LIO tại thời điểm $t_{\text{meas}}$, tìm trạng thái $x(t_{\text{meas}})$ và $P(t_{\text{meas}})$ tương ứng trong bộ nhớ đệm.
3. **Update:** Thực hiện cập nhật ES-IEKF trên trạng thái lịch sử đó bằng dữ liệu LIO mới nhận.
4. **Re-propagate (Dự đoán lại):** Từ thời điểm $t_{\text{meas}}$, lấy các gói tin IMU đã lưu chạy lại bước dự đoán liên tục đến thời điểm hiện tại $t_{\text{current}}$, cập nhật lại trạng thái và hiệp phương sai mới nhất.

---

## 9. Xuất bản dữ liệu & Hiển thị quỹ đạo (Throttling)

Bộ ước lượng xuất bản các thông tin chính:

* **Odometry (`/state_estimator/odom`):** Tần số cao (~900Hz) phục vụ trực tiếp cho bộ điều khiển chuyển động của robot.
* **TF (`odom -> base_link`):** Phục vụ xây dựng cây tọa độ của ROS 2.
* **Path (`/state_estimator/path`):** Xuất bản danh sách quỹ đạo di chuyển của robot (`nav_msgs/msg/Path`).
  * **Cơ chế giới hạn tần số (Throttling):** Để tránh quá tải CPU và nghẽn băng thông do tần số bộ lọc quá cao (~900Hz), việc xuất bản Path được giới hạn ở tần số **10Hz**.
  * **Giới hạn bộ nhớ:** Danh sách điểm quỹ đạo được khống chế ở giới hạn tối đa **5000 điểm** nhằm tránh rò rỉ bộ nhớ (memory leak).

---

## 10. File cấu hình mẫu (`estimator.yaml`)

```yaml
g1_state_estimator_node:
  ros__parameters:
    # Frame names
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    publish_tf: true
    use_host_time: true

    # ES-IEKF continuous noise
    gyro_noise: 0.0001
    acc_noise: 0.001
    gyro_bias_noise: 0.000001
    acc_bias_noise: 0.00001

    # Discrete measurement noise per-axis [X, Y, Z]
    lio_pos_noise: [0.001, 0.001, 0.1]       # Trục Z của LIO đặt nhiễu 0.1 để chống trôi Z
    lio_ori_noise: [0.001, 0.001, 0.001]
    leg_vel_noise: [0.01, 0.01, 0.01]

    # Buffer length
    history_length_sec: 2.0
```

---

## 11. Trạng thái dự án & Lộ trình phát triển

### Giai đoạn 1: Cơ bản (Đã hoàn thành)

* Tích hợp IMU + FAST-LIO.
* Xây dựng bộ đệm lịch sử (History Buffer) xử lý trễ LIO.
* Chuyển đổi và xuất bản TF.

### Giai đoạn 2: Kết hợp động học chân (Đã hoàn thành)

* Hợp nhất vận tốc tuyến tính Body-frame từ `/dog_odom` của Unitree G1.
* Chuyển đổi cấu trúc nhiễu đo sang dạng vectơ từng trục (Vector3d Anisotropic Noise Model) giúp triệt tiêu trôi trục Z.
* Tích hợp hiển thị Path trực quan hóa trong RViz (Giới hạn 10Hz, 5000 điểm).

### Giai đoạn 3: Ràng buộc nâng cao (Kế hoạch)

* Thiết lập Innovation Gate (ngưỡng loại bỏ điểm nhảy dị thường khi mất gói tin LiDAR đột ngột).
* Tự động kích hoạt chế độ Dead Reckoning chỉ bằng IMU + Chân khi mất tín hiệu LIO quá 0.5s.
* Tích hợp ràng buộc vận tốc đứng bằng 0 (ZUPT) khi robot đứng yên.
