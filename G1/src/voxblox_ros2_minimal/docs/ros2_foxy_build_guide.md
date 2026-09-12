# Hướng dẫn Cài đặt & Build Voxblox trên ROS 2 Foxy

Tài liệu này ghi lại chi tiết các gói phụ thuộc (dependencies) đã cài đặt và toàn bộ các chỉnh sửa mã nguồn cần thiết để biên dịch thành công bộ thư viện bản đồ thể tích **Voxblox** (`voxblox_ros2_minimal`) trên hệ điều hành **Ubuntu 20.04** và **ROS 2 Foxy**.

---

## 1. Cài đặt các gói phụ thuộc hệ thống (System Dependencies)

Trước khi build, các thư viện hệ thống và trình biên dịch protobuf cần được cài đặt đầy đủ thông qua lệnh sau:

```bash
sudo apt update
sudo apt install -y protobuf-compiler libprotobuf-dev libgoogle-glog-dev libgflags-dev
```

---

## 2. Các chỉnh sửa mã nguồn & Cấu hình Build

Để khắc phục các lỗi tương thích hệ thống của bản port ROS 2 ban đầu khi chạy trên Foxy, các thay đổi sau đã được thực hiện:

### 2.1. Cấu hình định vị Glog & Gflags (`cmake/Findglog.cmake`)
Do thư viện `libgoogle-glog-dev` trên Ubuntu 20.04 không đi kèm file cấu hình CMake (`glog-config.cmake`), ta đã tạo một file tìm kiếm tùy chỉnh:
* **Tệp tạo mới**: `cmake/Findglog.cmake`
* **Nội dung**: Định nghĩa vị trí header và thư viện của `glog`, đồng thời liên kết trực tiếp với thư viện `gflags` để tránh các lỗi liên kết DSO khi chạy kiểm thử (gtest).
* **Tích hợp**: Inject thư mục `cmake/` vào `CMAKE_MODULE_PATH` của tất cả các file `CMakeLists.txt` trong dự án.

### 2.2. Khắc phục lỗi INTERFACE Library của Downstream (`minikindr_ament`)
Gói `minkindr` là một thư viện header-only (INTERFACE). Việc sử dụng `ament_auto_add_library` với thư viện INTERFACE thường gây ra lỗi CMake.
* **Giải pháp**: 
  - Khai báo target `minkindr` thủ công bằng lệnh CMake chuẩn: `add_library(minkindr INTERFACE)`.
  - Cấu hình export các thư mục header (`include/`) và đăng ký target alias `minkindr::minkindr` để các gói tiêu thụ phía sau (như `voxblox`) có thể tìm thấy dễ dàng.

### 2.3. Loại bỏ gói `backward_ros` không sử dụng
Gói `backward_ros` (dùng để in stack trace khi crash) được khai báo tìm kiếm bắt buộc trong CMake nhưng không thực sự được include hay sử dụng trong code.
* **Giải pháp**: Comment out/Xóa bỏ khai báo `backward_ros` trong:
  - `voxblox_ros/CMakeLists.txt`
  - `voxblox_skeleton/CMakeLists.txt`
  - `voxblox_skeleton/package.xml`

### 2.4. Sửa đổi Callback Signature cho chuẩn ROS 2 Subscriber
Trong ROS 2, việc đăng ký hàm callback nhận tham chiếu const (ví dụ `const Msg&`) sẽ khiến trình biên dịch cố gắng tạo kiểu `std::shared_ptr<const Msg&>`, dẫn đến lỗi biên dịch nghiêm trọng.
* **Giải pháp**: Thay đổi kiểu đối số nhận vào của callback thành con trỏ thông minh dạng `ConstSharedPtr` hoặc `SharedPtr`:
  - **`tsdfMapCallback`** (trong `voxblox_ros/src/tsdf_server.cc` & `tsdf_server.h`):
    - Đổi thành `const voxblox_msgs::msg::Layer::SharedPtr layer_msg`.
    - Dereference pointer khi deserialize: `deserializeMsgToLayer<TsdfVoxel>(*layer_msg, ...)`.
  - **`esdfMapCallback`** (trong `voxblox_ros/src/esdf_server.cc` & `esdf_server.h`):
    - Đổi thành `const voxblox_msgs::msg::Layer::SharedPtr layer_msg`.
    - Dereference pointer: `deserializeMsgToLayer<EsdfVoxel>(*layer_msg, ...)`.
  - **`transformCallback`** (trong `voxblox_ros/src/transformer.cc` & `transformer.h`):
    - Đổi thành `const geometry_msgs::msg::TransformStamped::SharedPtr transform_msg`.
    - Dereference pointer khi đẩy vào queue: `transform_queue_.push_back(*transform_msg)`.

### 2.5. Thay đổi các Header và Include không tương thích
* **`cv_bridge`**: Đổi `#include <cv_bridge/cv_bridge.hpp>` thành `#include <cv_bridge/cv_bridge.h>` (trong `intensity_server.h`).
* **`tf2_geometry_msgs`**: Đổi `#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>` thành `#include <tf2_geometry_msgs/tf2_geometry_msgs.h>` (trong `transformer.cc` và `voxblox_eval.cc`).
* **`tf2`**: Xóa bỏ/Comment out include lỗi thời `#include <tf2/transform_datatypes.hpp>` trong `tsdf_server.cc` và `kindr_tf.h`.

---

## 3. Hướng dẫn Biên dịch (Compilation Instructions)

Để biên dịch toàn bộ các package trong repo `voxblox_ros2_minimal`, bạn chuyển về thư mục gốc của workspace ROS 2:

```bash
cd ~/ROS2/unitree_G1/dev_unitreeg1_ws

# Biên dịch toàn bộ các package liên quan tới Voxblox
colcon build --symlink-install --packages-up-to voxblox_skeleton voxblox_rviz_plugin
```

Sau khi hoàn thành, bạn chỉ cần source lại môi trường:
```bash
source install/setup.bash
```
 Dữ liệu bản đồ TSDF/ESDF lúc này đã sẵn sàng để tích hợp vào quy trình tránh vật cản của robot G1.
