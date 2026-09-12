# Hướng Dẫn Sử Dụng Voxblox Chi Tiết Cho Robot Unitree G1

Tài liệu này cung cấp hướng dẫn chi tiết cách vận hành, cấu hình, trực quan hóa và thao tác dữ liệu với Voxblox trong hệ thống robot Unitree G1 của bạn.

---

## 1. Kiến Trúc Voxblox Trong Hệ Thống G1

Hệ thống của bạn được thiết kế với hai node Voxblox hoạt động song song để giải quyết hai nhiệm vụ độc lập:

```mermaid
graph TD
    LiDAR[Livox Mid-360] -->|Raw Pointcloud| Bringup[g1_bringup]
    Bringup -->|/utlidar/cloud_livox_mid360_sync| LocalNode[voxblox_local]
    Bringup -->|Raw LiDAR/IMU| FastLIO[FAST_LIO SLAM]
    FastLIO -->|/cloud_registered| GlobalNode[voxblox_global]
  
    subgraph voxblox_local [Node Cục Bộ - odom frame]
        LocalNode -->|Publish| LocalMesh[/voxblox_local/mesh]
        LocalNode -->|Publish| LocalSlice[/voxblox_local/esdf_slice]
    end

    subgraph voxblox_global [Node Toàn Cục - map frame]
        GlobalNode -->|Publish| GlobalMesh[/voxblox_global/mesh]
        GlobalNode -->|Publish| GlobalOccupied[/voxblox_global/occupied_nodes]
    end
```

### 1.1. Voxblox Global (`voxblox_global`)

- **Khung tọa độ:** `map`
- **Đầu vào (Remap):** `/voxblox_global/pointcloud` $\rightarrow$ `/cloud_registered` (đám mây điểm đã qua xử lý SLAM/đăng ký toàn cục).
- **Mục tiêu:** Xây dựng mô hình 3D toàn cảnh không gian lớn (TSDF & Mesh). Dùng để **Lập bản đồ (Mapping)** và **Định vị (Localization)**.
- **Tần số cập nhật:** Thấp (1.0 Hz) để tiết kiệm CPU.

### 1.2. Voxblox Local (`voxblox_local`)

- **Khung tọa độ:** `odom`
- **Đầu vào (Remap):** `/voxblox_local/pointcloud` $\rightarrow$ `/utlidar/cloud_livox_mid360_sync` (pointcloud đồng bộ tần số cao trực tiếp từ cảm biến).
- **Mục tiêu:** Tính toán trường khoảng cách Euclid ký hiệu (ESDF) để phục vụ **Tránh vật cản thời gian thực (Collision Avoidance)**.
- **Tần số cập nhật:** Cao (5.0 Hz - `update_mesh_every_n_sec: 0.2`) trong phạm vi ngắn (bán kính 2.0m - 5.0m quanh robot).

---

## 2. Quy Trình Khởi Chạy Hệ Thống

Để Voxblox hoạt động đúng, toàn bộ các thành phần hệ thống cần được khởi chạy theo thứ tự sau:

### Bước 1: Khởi chạy phần cứng & robot bridge

```bash
ros2 launch g1_bringup g1_bringup.launch.py
```

### Bước 2: Khởi chạy SLAM (Odometry)

```bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

### Bước 3: Khởi chạy bộ lọc ước lượng trạng thái EKF

```bash
ros2 launch g1_state_estimator estimator.launch.py
```

### Bước 4: Khởi chạy hệ thống Voxblox

```bash
ros2 launch g1_state_estimator voxblox.launch.py
```

---

## 3. Các Tham Số Cấu Hình Quan Trọng

Các file cấu hình nằm tại thư mục `src/g1_state_estimator/config/`.

### 3.1. Cấu hình Bản đồ Toàn cục (`voxblox_global.yaml`)

```yaml
tsdf_voxel_size: 0.10          # Kích thước của mỗi ô voxel (0.1m = 10cm). Tăng lên nếu muốn nhẹ máy.
tsdf_voxels_per_side: 16       # Số voxel trên mỗi cạnh của 1 block.
world_frame: "map"             # Tọa độ neo của bản đồ toàn cục.
max_ray_length_m: 8.0          # Khoảng cách tối đa tích hợp tia quét LiDAR.
min_ray_length_m: 0.2          # Khoảng cách tối thiểu loại bỏ nhiễu/khung thân robot.
update_mesh_every_n_sec: 1.0   # Chu kỳ cập nhật Mesh hiển thị (1 giây/lần).
publish_tsdf_map: false
publish_slices: false
publish_pointclouds: true      # [THÊM] Bật/tắt xuất bản PointCloud2 để xem trên RViz.
```

### 3.2. Cấu hình Bản đồ Cục bộ tránh vật cản (`voxblox_local.yaml`)

```yaml
tsdf_voxel_size: 0.10
tsdf_voxels_per_side: 16
world_frame: "odom"            # Tọa độ neo cục bộ.
max_ray_length_m: 5.0          # Giới hạn ngắn hơn để giảm tải tính toán.
generate_esdf: true            # Bắt buộc bật để tính khoảng cách vật cản.
publish_esdf_map: true
esdf_max_distance_m: 2.0       # Khoảng cách tối đa cần tính toán ESDF (thường robot chỉ cần tránh vật cản < 2m).
update_mesh_every_n_sec: 0.2   # Cập nhật nhanh (5 Hz) để bám theo vật cản động.
publish_slices: true           # Xuất bản lát cắt 2D của trường khoảng cách.
slice_level: 0.0               # Lát cắt ngang mặt đất (z = 0m).
publish_pointclouds: true      # [THÊM] Bật để xuất bản esdf_pointcloud và esdf_slice.
```

---

## 4. Hướng Dẫn Trực Quan Hóa Trên RViz2

Mở RViz2 lên (`rviz2`), thay đổi **Fixed Frame** thành `map` (hoặc `odom`), sau đó add các kiểu hiển thị sau:

### 4.1. Hiển thị Mesh 3D trực quan

- Nhấp **Add** $\rightarrow$ Chọn **`VoxbloxMesh`** (yêu cầu cài đặt plugin Voxblox RViz).
- Chọn Topic: `/voxblox_global/mesh` hoặc `/voxblox_local/mesh`.

### 4.2. Hiển thị dạng hình hộp voxel (Không cần Plugin)

- Nhấp **Add** $\rightarrow$ Chọn **`MarkerArray`**.
- Chọn Topic: `/voxblox_global/occupied_nodes`.
- *Mô tả:* Bạn sẽ thấy các khối lập phương nhỏ 3D biểu diễn các ô không gian đã bị chiếm bởi vật cản.

### 4.3. Hiển thị lát cắt 2D phục vụ né vật cản (Slices)

- Nhấp **Add** $\rightarrow$ Chọn **`PointCloud2`**.
- Chọn Topic: `/voxblox_local/esdf_slice`.
- *Mẹo cấu hình:* Thay đổi **Color Transformer** thành `AxisColor` hoặc `Intensity` để nhìn thấy thang màu sắc thể hiện khoảng cách đến vật cản (màu đỏ là rất gần vật cản, màu xanh là vùng an toàn).

---

## 5. Thao Tác Lưu/Mở Bản Đồ (Services)

Khi đang chạy Voxblox, bạn có thể tương tác với bản đồ thông qua các ROS 2 Service sau:

### 5.1. Lưu bản đồ thể tích Voxblox để chạy lại lần sau

Lưu toàn bộ bản đồ voxel (định dạng nhị phân `.vxblx`) để phục vụ định vị hoặc tránh vật cản mà không cần quét lại:

```bash
ros2 service call /voxblox_global/save_map voxblox_msgs/srv/FilePath "{file_path: '/home/hoangdc/ROS2/unitree_G1/maps/g1_office.vxblx'}"
```

### 5.2. Tải bản đồ đã quét lên hệ thống

Nạp lại file `.vxblx` đã lưu trước đó vào hệ thống:

```bash
ros2 service call /voxblox_global/load_map voxblox_msgs/srv/FilePath "{file_path: '/home/hoangdc/ROS2/unitree_G1/maps/g1_office.vxblx'}"
```

### 5.3. Xuất mô hình lưới 3D (Mesh) thành file `.ply`

Lưu bản đồ lưới đa giác màu sắc để mở trong Blender, MeshLab, CloudCompare hoặc nạp vào phần mềm mô phỏng:

1. Đảm bảo cấu hình file `voxblox_global.yaml` chứa dòng: `mesh_filename: "/home/hoangdc/ROS2/unitree_G1/maps/g1_office.ply"`
2. Gọi lệnh sau để xuất file:
   ```bash
   ros2 service call /voxblox_global/generate_mesh std_srvs/srv/Empty {}
   ```

### 5.4. Xóa sạch bản đồ hiện tại (Reset)

Xóa toàn bộ các block voxel để tiến hành quét lại từ đầu:

```bash
ros2 service call /voxblox_global/clear_map std_srvs/srv/Empty {}
```

---

## 6. Giải Quyết Sự Cố Thường Gặp (Troubleshooting)

### 6.1. Xuất hiện hàng loạt cảnh báo `Could not get transform...` khi khởi động

- **Triệu chứng:** Node Voxblox liên tục in ra các dòng màu vàng thông báo không tìm thấy transform từ `camera_init` sang `map` hoặc `livox_frame` sang `odom`.
- **Giải thích:** Đây là hiện tượng bình thường khi các node SLAM/EKF chưa khởi động xong hoặc chưa xuất bản cây tọa độ TF. Voxblox sẽ tạm thời giữ pointcloud trong hàng đợi.
- **Cách xử lý:** Đợi khoảng 5-10 giây để hệ thống SLAM chạy ổn định, dữ liệu TF bắt đầu truyền đến, cảnh báo sẽ tự động biến mất và dữ liệu sẽ được tích hợp.

### 6.2. Node bị crash khi nhấn `Ctrl+C` hoặc treo tắt mất 5 giây

- **Giải thích:** Hiện tượng này xảy ra do hàng đợi điểm quá tải khiến node bận xử lý không phản hồi tín hiệu tắt, hoặc bộ nhớ ROS 2 bị hủy sai thứ tự.
- **Giải pháp:** Lỗi này **đã được chúng tôi vá triệt để** bằng cách thêm kiểm tra trạng thái hủy `rclcpp::ok()` và gọi hàm giải phóng bộ nhớ `rclcpp::shutdown();` chuẩn mực trước khi thoát chương trình.
