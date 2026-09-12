# Đề xuất Kiến trúc Phần mềm `g1_state_estimator`

Để phát triển bộ ước lượng trạng thái Unitree G1 hoạt động ổn định, dễ bảo trì và kiểm thử độc lập, chương trình được thiết kế theo mô hình **tách biệt phần lõi toán học (Core Math Engine) khỏi lớp bọc phần cứng ROS 2 (Wrapper Node)**.

---

## 1. Cấu trúc Thư mục Package

```text
g1_state_estimator/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── g1_state_estimator/
│       ├── state.h                # Định nghĩa cấu trúc dữ liệu (State, IMUData,...)
│       ├── estimator.h            # Lớp lõi thuật toán ES-IEKF, nhiễu 3 chiều và History Buffer
│       └── estimator_node.h       # Lớp bọc ROS 2 Node (Pub/Sub, TF, Path, Parameter Parser)
├── src/
│   ├── estimator.cpp              # Chi tiết toán học IEKF và logic Rollback với diagonal noise
│   ├── estimator_node.cpp         # ROS 2 Pub/Sub, Throttled Path (10Hz), và quản lý Parameters
│   └── main.cpp                   # Điểm chạy chương trình
├── config/
│   └── estimator.yaml             # File cấu hình nhiễu (dạng mảng [x,y,z]) và frames
└── launch/
    └── estimator.launch.py        # Launch file khởi chạy
```

---

## 2. Thiết kế các Thành phần Cốt lõi (Class & Struct Design)

### 2.1. Cấu trúc dữ liệu (`state.h`)
Sử dụng thư viện `Eigen` cho tính toán ma trận.

```cpp
#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct State {
    double timestamp;
    Eigen::Vector3d p;         // Vị trí (Position) toàn cục
    Eigen::Vector3d v;         // Vận tốc (Velocity) toàn cục
    Eigen::Quaterniond q;      // Hướng (Orientation) toàn cục
    Eigen::Vector3d bg;        // Gyroscope bias
    Eigen::Vector3d ba;        // Accelerometer bias

    // Hàm toán tử cộng Boxplus (⊕) cho hệ thống Lie Group
    void boxplus(const Eigen::Matrix<double, 15, 1>& dx) {
        p += dx.segment<3>(0);
        v += dx.segment<3>(3);
        // Quaternion multiplication đại diện cho Exp(dtheta) * q
        Eigen::Vector3d dtheta = dx.segment<3>(6);
        if (dtheta.norm() > 1e-6) {
            Eigen::Quaterniond dq(Eigen::AngleAxisd(dtheta.norm(), dtheta.normalized()));
            q = (dq * q).normalized();
        }
        bg += dx.segment<3>(9);
        ba += dx.segment<3>(12);
    }
};

struct IMUData {
    double timestamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};

struct LIOMeasurement {
    double timestamp;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
};

struct LegMeasurement {
    double timestamp;
    Eigen::Vector3d v_body; // Vận tốc tuyến tính trong body frame
};
```

---

### 2.2. Lớp ước lượng lõi (`estimator.h` & `estimator.cpp`)
Lớp này **hoàn toàn độc lập với ROS 2**, giúp dễ viết Unit Test độc lập. Hệ thống hỗ trợ hiệp phương sai nhiễu đo dạng vector để cấu hình độc lập từng trục.

```cpp
#pragma once
#include "state.h"
#include <map>
#include <vector>
#include <mutex>

class Estimator {
public:
    Estimator();
    ~Estimator() = default;

    // Khởi tạo trạng thái ban đầu
    void Initialize(const State& initial_state, const Eigen::Matrix<double, 15, 15>& initial_cov);

    // Cấu hình các thông số nhiễu (sử dụng Vector3d cho các tham số đo lường)
    void SetNoiseParameters(double gyro_noise, double acc_noise, 
                            double gyro_bias_noise, double acc_bias_noise,
                            const Eigen::Vector3d& lio_pos_noise,
                            const Eigen::Vector3d& lio_ori_noise,
                            const Eigen::Vector3d& leg_vel_noise);

    // Bước dự đoán IMU (chạy liên tục ở tần số cao)
    void Predict(const IMUData& imu, double dt);

    // Bước cập nhật vận tốc chân (tức thời, độ trễ thấp)
    void UpdateLeg(const LegMeasurement& leg);

    // Bước cập nhật dữ liệu trễ từ FAST-LIO (sử dụng Rollback)
    bool UpdateLIO(const LIOMeasurement& lio);

    // Lấy trạng thái hiện tại
    State GetState() const;
    Eigen::Matrix<double, 15, 15> GetCovariance() const;

private:
    State state_;
    Eigen::Matrix<double, 15, 15> P_; // Hiệp phương sai sai số

    // Bộ tham số nhiễu
    double q_gyro_, q_acc_, q_bg_, q_ba_;
    Eigen::Vector3d r_lio_pos_; // Hiệp phương sai nhiễu vị trí LIO [x, y, z]
    Eigen::Vector3d r_lio_ori_; // Hiệp phương sai nhiễu tư thế LIO [roll, pitch, yaw]
    Eigen::Vector3d r_leg_vel_; // Hiệp phương sai nhiễu vận tốc chân [vx, vy, vz]

    // Bộ đệm lịch sử phục vụ Rollback
    std::map<double, State> state_history_;
    std::map<double, Eigen::Matrix<double, 15, 15>> cov_history_;
    std::map<double, IMUData> imu_history_;

    // Độ dài tối đa của history buffer (ví dụ: 2.0 giây)
    double max_history_duration_ = 2.0;

    mutable std::mutex mutex_;

    // Hàm dọn dẹp bộ đệm vượt quá max_history_duration_
    void PruneHistory(double latest_time);

    // Logic cốt lõi để rollback và chạy lại IMU prediction
    bool RollbackAndRePropagate(double t_meas);
};
```

---

### 2.3. Lớp bọc ROS 2 Node (`estimator_node.h` & `estimator_node.cpp`)
Quản lý giao tiếp với ROS 2: Pub/Sub, TF, Path (tần số 10Hz để chống nghẽn), và phân tích cấu hình mảng/vector.

```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "estimator.h"

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode();

private:
    // Callbacks nhận dữ liệu cảm biến
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void LioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void DogOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Hàm đọc cấu hình ROS 2 parameters
    void LoadParameters();
    
    // Hàm helper đọc tham số dạng Vector3d từ mảng hoặc số đơn trong YAML
    Eigen::Vector3d ParseVector3Parameter(const std::string& name, double default_val);

    // Các Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_sub_;

    // Publisher & TF Broadcaster
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Quản lý hiển thị Path
    nav_msgs::msg::Path path_msg_;
    double last_path_pub_time_ = 0.0;

    // Lõi ước lượng
    Estimator estimator_;

    // Biến trạng thái khởi tạo
    bool is_initialized_ = false;
    double last_imu_time_ = -1.0;

    // Tên các frame tọa độ
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    // Cấu hình đồng bộ thời gian và TF
    bool use_host_time_;
    bool publish_tf_;

    // Extrinsics tĩnh dự phòng: body (LIO internal IMU) to base_link
    Eigen::Vector3d t_body_baselink_;
    Eigen::Quaterniond q_body_baselink_;
};
```

---

## 3. Phân tích Luồng Xử lý Dữ liệu Chi tiết

### 3.1. Luồng IMU (Prediction - ~900Hz)
1. Callback nhận IMU $\mathbf{u}_k = [\mathbf{a}_k, \boldsymbol{\omega}_k]^T$ tại thời điểm $t_k$.
2. Đồng bộ thời gian: Nếu `use_host_time` bật, gán $t_k$ bằng thời gian hiện tại của hệ thống.
3. Tính toán $\Delta t = t_k - t_{k-1}$ và chạy `Predict(imu_data, dt)`.
4. Dự đoán trạng thái danh nghĩa và lan truyền ma trận hiệp phương sai sai số $P_k = F P_{k-1} F^T + G Q G^T$.
5. Lưu trữ lịch sử trạng thái, hiệp phương sai và IMU vào hàng đợi buffer.
6. Quảng bá tin `/state_estimator/odom` ở tần số cao.
7. **Throttled Path:** Cập nhật quỹ đạo robot và phát hành tin `/state_estimator/path` với tần số giới hạn **10Hz** và số lượng điểm giới hạn dưới **5000** để chống tràn bộ nhớ.

### 3.2. Luồng Leg Odometry (Correction - ~900Hz)
1. Callback nhận gói `/dog_odom` chứa vận tốc hông (pelvis velocity) từ SDK.
2. Trích xuất vận tốc tuyến tính trong hệ trục robot (Body-frame velocity) $v^B_{leg}$.
3. Gọi hàm `estimator_.UpdateLeg(leg_data)`.
4. Vì dữ liệu này có độ trễ cực thấp, ta thực hiện cập nhật trực tiếp lên trạng thái hiện tại bằng mô hình đo đường chéo:
   - Sai số đo (Residual): $\mathbf{r} = v^B_{leg} - R(q)^T v_{est}$.
   - Hiệp phương sai nhiễu đo: $R_{\text{leg}} = \text{diag}(\sigma_{vx}^2, \sigma_{vy}^2, \sigma_{vz}^2)$.
   - Tính toán ma trận Jacobian đo $H_v$ và thực hiện cập nhật trạng thái lỗi $\delta x$.

### 3.3. Luồng FAST-LIO (Correction trễ - ~10Hz)
1. Callback nhận dữ liệu `/odom_livox` tại thời điểm $t_{now}$, tương ứng với gói đo trong quá khứ $t_{meas}$.
2. Gọi hàm `estimator_.UpdateLIO(lio_data)` để thực hiện **Rollback & Re-propagate**:
   - **Bước 1**: Tìm kiếm mốc $t_{match}$ gần nhất với $t_{meas}$ trong bộ đệm lịch sử.
   - **Bước 2**: Đặt trạng thái tạm thời của bộ lọc về $x(t_{match})$ và $P(t_{match})$.
   - **Bước 3**: Tính toán sai số đo lường. Sử dụng hiệp phương sai không đẳng hướng ($R_{\text{pos}} = \text{diag}(\sigma_{px}^2, \sigma_{py}^2, \sigma_{pz}^2)$) với giá trị nhiễu Z rất lớn (`0.1`) để lọc hoàn toàn hiện tượng trôi Z của LiDAR.
   - **Bước 4**: Thực hiện các vòng lặp cập nhật IEKF để hiệu chỉnh trạng thái lịch sử $x(t_{match})$ và ma trận hiệp phương sai $P(t_{match})$.
   - **Bước 5 (Re-propagate)**: Quét toàn bộ dữ liệu IMU trong `imu_history_` từ sau mốc $t_{match}$ đến thời điểm hiện tại $t_{now}$, chạy dự đoán lại để cập nhật trạng thái hiện thời lên thực tại.

---

## 4. Tổng kết các điểm Thiết kế đã Giải quyết

1. **Đồng bộ thời gian tuyệt đối:**
   * Hệ thống sử dụng tham số cấu hình `use_host_time = true`. Toàn bộ dữ liệu IMU, LIO, và chân robot khi đi vào Node ước lượng sẽ được gán nhãn thời gian theo đồng hồ của Host PC để loại bỏ sai lệch thời gian truyền nhận không dây hoặc khác biệt đồng hồ giữa các máy tính.
2. **Ngăn chặn trôi đứng (Z-drift):**
   * Đã chuyển đổi hoàn chỉnh sang mô hình nhiễu đường chéo (diagonal covariance). Cấu hình độ nhiễu đo vị trí trục Z của LIO lên `0.1` (gấp 100 lần X, Y) làm bộ lọc bỏ qua các cú trôi đứng từ LiDAR, giúp giữ chiều cao robot ổn định tuyệt đối dựa trên động học chân.
3. **Cấu hình Extrinsic tĩnh:**
   * Các transform tĩnh giữa các khung hình được cấu hình tập trung trong file `g1_bringup.yaml`. Đồng thời bộ ước lượng `g1_state_estimator` hỗ trợ nạp cấu hình extrinsics mặc định trùng khớp thông số vật lý của robot G1 để dự phòng khi TF tree lookup bị chậm.
4. **Outlier Rejection (Kế hoạch Giai đoạn 3):**
   * Khi mất gói tin LiDAR khiến LIO bị nhảy (jump), chúng ta sẽ thiết lập một Innovation Gate (ngưỡng khoảng cách Euclidean) trong bước `UpdateLIO`. Nếu phép đo vị trí LIO lệch quá $0.5\text{ m}$ so với trạng thái dự đoán, bộ lọc sẽ từ chối phép đo này và tiếp tục chạy ổn định bằng động học chân (Dead Reckoning).
