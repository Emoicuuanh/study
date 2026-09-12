#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "estimator.h"

class StateEstimatorNode : public rclcpp::Node {
public:
    StateEstimatorNode();
    ~StateEstimatorNode() = default;

private:
    // Subscriber callbacks
    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void LioCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void DogOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Helper functions
    void LoadParameters();
    void PublishState(const State& state, const rclcpp::Time& stamp);
    Eigen::Vector3d ParseVector3Parameter(const std::string& name, double default_val);

    // ROS 2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leg_sub_;

    // ROS 2 Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Core Estimator
    Estimator estimator_;

    // State initialization helper variables
    bool is_initialized_ = false;
    double last_imu_time_ = -1.0;

    // TF management
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string imu_topic_;
    std::string lio_topic_;
    std::string leg_topic_;

    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;
    bool use_host_time_;

    // Extrinsics fallback (if TF lookup fails)
    Eigen::Vector3d t_imu_pelvis_ = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_imu_pelvis_ = Eigen::Quaterniond::Identity();
};
