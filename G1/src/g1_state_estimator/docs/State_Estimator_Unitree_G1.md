# Thiết kế State Estimator cho Unitree G1

## 1. Mục tiêu

Xây dựng một **State Estimator** trung tâm cho robot humanoid Unitree
G1, thay vì chỉ fusion hai topic odometry bằng `robot_localization`.

Nguồn dữ liệu hiện có:

-   `/imu` (\~1000 Hz)
-   `/dog_odom` (200--500 Hz, odometry từ Unitree SDK)
-   `/odom_livox` (50--100 Hz, FAST-LIO)

Estimator sử dụng **Error-State IEKF**.

------------------------------------------------------------------------

## 2. Kiến trúc tổng thể

``` text
                 IMU (/imu)
               1000 Hz
                   │
                   ▼
            Prediction Model
                   │
                   ▼
       ┌─────────────────────────┐
       │     IEKF Estimator      │
       │-------------------------│
       │ Internal State x̂        │
       │                         │
       │ R   : Rotation          │
       │ p   : Position          │
       │ v   : Velocity          │
       │ bg  : Gyro bias         │
       │ ba  : Acc bias          │
       └─────────────────────────┘
          ▲                  ▲
          │                  │
          │                  │
   Leg Measurement     LiDAR Measurement
          │                  │
     /dog_odom         /odom_livox
   (200~500 Hz)        (50~100 Hz)
          │                  │
          └────────┬─────────┘
                   │
          Measurement Update
                   │
                   ▼
        Internal State được cập nhật
                   │
                   ▼
        Publish /state_estimation
```

> Chỉ tồn tại **một Internal State** trong IEKF. `/dog_odom` và
> `/odom_livox` chỉ là measurement để hiệu chỉnh state.

------------------------------------------------------------------------

## 3. State của bộ lọc

State được lựa chọn:

    x = [R, p, v, bg, ba]

Trong đó:

-   `R`: Orientation
-   `p`: Position
-   `v`: Linear velocity
-   `bg`: Gyroscope bias
-   `ba`: Accelerometer bias

Đây là state 15 chiều điển hình của Error-State EKF/IEKF.

------------------------------------------------------------------------

## 4. Prediction

Prediction chỉ sử dụng IMU.

Chu kỳ:

-   IMU: \~1000 Hz

Luồng:

    gyro
       ↓
    Orientation

    acc
       ↓
    Velocity
       ↓
    Position

Prediction được thực hiện liên tục dù chưa có LiDAR hoặc Leg Odometry.

------------------------------------------------------------------------

## 5. Measurement Update

### 5.1 Leg Update

Nguồn:

    /dog_odom

Measurement:

-   Position
-   Orientation
-   Velocity

Residual:

    r_leg = z_leg - h(x)

Sau đó IEKF thực hiện correction.

------------------------------------------------------------------------

### 5.2 LiDAR Update

Nguồn:

    /odom_livox

Measurement:

-   Position
-   Orientation
-   Velocity

Residual:

    r_lio = z_lio - h(x)

Sau đó IEKF cập nhật lại Internal State.

------------------------------------------------------------------------

## 6. Chu trình hoạt động

### IMU callback

    predict()

    ↓

    Internal State

    ↓

    publish(/state_estimation)

### FAST-LIO callback

    updateLidar()

    ↓

    Internal State

    ↓

    publish(/state_estimation)

### dog_odom callback

    updateLeg()

    ↓

    Internal State

    ↓

    publish(/state_estimation)

------------------------------------------------------------------------

## 7. Pseudo-code

``` cpp
State x;
Covariance P;

imuCallback()
{
    predict(x, P);
    publish(x);
}

lioCallback()
{
    updateLidar(x, P);
    publish(x);
}

dogCallback()
{
    updateLeg(x, P);
    publish(x);
}
```

------------------------------------------------------------------------

## 8. ROS2 Node

    g1_state_estimator

Subscribe:

-   `/imu`
-   `/dog_odom`
-   `/odom_livox`

Publish:

-   `/state_estimation`
-   `tf`

------------------------------------------------------------------------

## 9. Khả năng mở rộng

Khi lấy được dữ liệu thô từ Unitree SDK:

-   Joint Encoder
-   Foot Contact
-   Joint Torque

thì chỉ cần thay thế `updateLeg()` bằng mô hình động học chân.

State có thể mở rộng thành:

    [R, p, v, bg, ba, LeftFootPose, RightFootPose, ContactState]

mà không cần thay đổi kiến trúc tổng thể.

------------------------------------------------------------------------

## 10. So sánh với robot_localization

  robot_localization                 IEKF đề xuất
  ---------------------------------- --------------------------------
  Fusion topic                       Duy trì Internal State
  Tổng quát                          Thiết kế cho humanoid
  Không có Prediction theo mô hình   Prediction IMU
  Không dễ mở rộng                   Dễ thêm measurement mới
  Không phù hợp whole-body           Hướng tới whole-body estimator

## 11. Lộ trình phát triển

1.  Xây dựng IEKF với IMU + `/dog_odom` + `/odom_livox`.
2.  Kiểm thử độ ổn định và covariance.
3.  Thay `/dog_odom` bằng encoder + foot contact khi có dữ liệu.
4.  Thêm camera/VIO.
5.  Mở rộng thành Whole-body InEKF.
