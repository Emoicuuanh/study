#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <mutex>
#include <chrono>

class TimestampCorrectorNode : public rclcpp::Node {
public:
  TimestampCorrectorNode() : Node("timestamp_corrector") {
    // QoS matching Unitree Livox driver
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    
    // Estimate clock offset from IMU (high frequency, 200Hz)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/utlidar/imu_livox_mid360", qos,
      [this](sensor_msgs::msg::Imu::UniquePtr msg) {
        update_offset(rclcpp::Time(msg->header.stamp));
        msg->header.stamp = correct_time(rclcpp::Time(msg->header.stamp));
        imu_pub_->publish(std::move(msg));
      });
      
    // Correct PointCloud timestamps using the same hardware-clock offset
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_livox_mid360", qos,
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
        // Fallback in case IMU hasn't started yet
        update_offset(rclcpp::Time(msg->header.stamp));
        msg->header.stamp = correct_time(rclcpp::Time(msg->header.stamp));
        cloud_pub_->publish(std::move(msg));
      });
      
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "/utlidar/imu_livox_mid360_sync", qos);
      
    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/utlidar/cloud_livox_mid360_sync", qos);
      
    RCLCPP_INFO(this->get_logger(), "C++ High-Performance Jitter-Free Timestamp Corrector started.");
    RCLCPP_INFO(this->get_logger(), "LiDAR & IMU hardware clocks will be synchronized to Host PC time.");
  }

private:
  void update_offset(const rclcpp::Time& hw_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    double system_sec = this->now().seconds();
    double hw_sec = hw_time.seconds();
    double current_offset = system_sec - hw_sec;
    
    if (!offset_initialized_) {
      time_offset_ = current_offset;
      offset_initialized_ = true;
    } else {
      // If clock jumped (e.g. driver re-sync > 100ms), reset offset instantly.
      if (std::abs(current_offset - time_offset_) > 0.10) {
        time_offset_ = current_offset;
      } else {
        // Ultra-slow tracking of thermal/clock drift without packet receipt jitter
        time_offset_ = 0.0001 * current_offset + 0.9999 * time_offset_;
      }
    }
  }
  
  rclcpp::Time correct_time(const rclcpp::Time& hw_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    int64_t offset_ns = static_cast<int64_t>(time_offset_ * 1e9);
    return hw_time + rclcpp::Duration(std::chrono::nanoseconds(offset_ns));
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  
  std::mutex mutex_;
  double time_offset_ = 0.0;
  bool offset_initialized_ = false;
  const double alpha_ = 0.005; // Very slow filtering to maintain ultra-smooth relative timing
};

int main(int argc, char** argv) {
  setvbuf(stdout, NULL, _IONBF, 0);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimestampCorrectorNode>());
  rclcpp::shutdown();
  return 0;
}
