#include "g1_state_estimator/estimator_node.h"
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

StateEstimatorNode::StateEstimatorNode() : Node("g1_state_estimator") {
    // 1. Load Parameters
    LoadParameters();

    // 2. Initialize TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 3. Setup Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimator/odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/state_estimator/path", 10);

    // 4. Setup Subscribers
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 20, std::bind(&StateEstimatorNode::ImuCallback, this, std::placeholders::_1));
    
    lio_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        lio_topic_, 10, std::bind(&StateEstimatorNode::LioCallback, this, std::placeholders::_1));
        
    leg_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        leg_topic_, 10, std::bind(&StateEstimatorNode::DogOdomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "State Estimator Node Initialized.");
    RCLCPP_INFO(this->get_logger(), "Subscriptions: IMU -> %s, LIO -> %s, Leg -> %s", 
                imu_topic_.c_str(), lio_topic_.c_str(), leg_topic_.c_str());
}

void StateEstimatorNode::LoadParameters() {
    // Topics
    this->declare_parameter<std::string>("imu_topic", "/dog_imu_raw");
    this->declare_parameter<std::string>("lio_topic", "/Odometry");
    this->declare_parameter<std::string>("leg_topic", "/dog_odom");
    
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    lio_topic_ = this->get_parameter("lio_topic").as_string();
    leg_topic_ = this->get_parameter("leg_topic").as_string();

    // Frames
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "pelvis");

    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    this->declare_parameter<bool>("publish_tf", true);
    publish_tf_ = this->get_parameter("publish_tf").as_bool();

    this->declare_parameter<bool>("use_host_time", true);
    use_host_time_ = this->get_parameter("use_host_time").as_bool();

    // Noise parameters
    this->declare_parameter<double>("gyro_noise", 1e-4);
    this->declare_parameter<double>("acc_noise", 1e-3);
    this->declare_parameter<double>("gyro_bias_noise", 1e-6);
    this->declare_parameter<double>("acc_bias_noise", 1e-5);
    double gyro_n = this->get_parameter("gyro_noise").as_double();
    double acc_n = this->get_parameter("acc_noise").as_double();
    double gyro_bn = this->get_parameter("gyro_bias_noise").as_double();
    double acc_bn = this->get_parameter("acc_bias_noise").as_double();
    Eigen::Vector3d lio_pn = ParseVector3Parameter("lio_pos_noise", 1e-3);
    Eigen::Vector3d lio_on = ParseVector3Parameter("lio_ori_noise", 1e-3);
    Eigen::Vector3d leg_vn = ParseVector3Parameter("leg_vel_noise", 1e-2);

    estimator_.SetNoiseParameters(gyro_n, acc_n, gyro_bn, acc_bn, lio_pn, lio_on, leg_vn);

    // Rollback buffer length
    this->declare_parameter<double>("history_length_sec", 2.0);
    double hist_len = this->get_parameter("history_length_sec").as_double();
    estimator_.SetMaxHistoryDuration(hist_len);

    // Fallback static extrinsics: body to base_link
    // default matches g1_bringup.yaml: [0.03314, 0.02332, 0.41554, rpy: -3.14159, -0.04014, 0.0]
    this->declare_parameter<double>("extrinsic_x", 0.03314);
    this->declare_parameter<double>("extrinsic_y", 0.02332);
    this->declare_parameter<double>("extrinsic_z", 0.41554);
    this->declare_parameter<double>("extrinsic_roll", -3.14159);
    this->declare_parameter<double>("extrinsic_pitch", -0.04014);
    this->declare_parameter<double>("extrinsic_yaw", 0.0);

    double ex = this->get_parameter("extrinsic_x").as_double();
    double ey = this->get_parameter("extrinsic_y").as_double();
    double ez = this->get_parameter("extrinsic_z").as_double();
    double eroll = this->get_parameter("extrinsic_roll").as_double();
    double epitch = this->get_parameter("extrinsic_pitch").as_double();
    double eyaw = this->get_parameter("extrinsic_yaw").as_double();

    t_imu_pelvis_ = Eigen::Vector3d(ex, ey, ez);
    
    // Euler to quaternion
    Eigen::AngleAxisd rollAngle(eroll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(epitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(eyaw, Eigen::Vector3d::UnitZ());
    q_imu_pelvis_ = yawAngle * pitchAngle * rollAngle;
}

void StateEstimatorNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    if (use_host_time_) {
        timestamp = this->get_clock()->now().seconds();
    }

    IMUData imu;
    imu.timestamp = timestamp;
    imu.acc = Eigen::Vector3d(msg->linear_acceleration.x, 
                               msg->linear_acceleration.y, 
                               msg->linear_acceleration.z);
    imu.gyro = Eigen::Vector3d(msg->angular_velocity.x, 
                                msg->angular_velocity.y, 
                                msg->angular_velocity.z);

    if (!estimator_.IsInitialized()) {
        return;
    }

    double dt = timestamp - last_imu_time_;
    last_imu_time_ = timestamp;

    // Run prediction step
    estimator_.Predict(imu, dt);

    // Publish high-rate odometry and TF
    if (estimator_.IsInitialized()) {
        State current_state = estimator_.GetState();
        PublishState(current_state, msg->header.stamp);
    }
}

void StateEstimatorNode::DogOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!estimator_.IsInitialized()) return;

    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    if (use_host_time_) {
        timestamp = this->get_clock()->now().seconds();
    }

    LegMeasurement leg;
    leg.timestamp = timestamp;
    leg.v_body = Eigen::Vector3d(msg->twist.twist.linear.x, 
                                  msg->twist.twist.linear.y, 
                                  msg->twist.twist.linear.z);

    // Call update step
    estimator_.UpdateLeg(leg);
}

void StateEstimatorNode::LioCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Always use original header stamp for LIO measurement to match exact scan time in state history
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();

    // 1. Extract raw LIO pose (LiDAR sensor pose in camera_init/map frame)
    Eigen::Vector3d p_lidar_raw(msg->pose.pose.position.x, 
                                msg->pose.pose.position.y, 
                                msg->pose.pose.position.z);
    Eigen::Quaterniond q_lidar_raw(msg->pose.pose.orientation.w, 
                                   msg->pose.pose.orientation.x, 
                                   msg->pose.pose.orientation.y, 
                                   msg->pose.pose.orientation.z);

    // Transform LiDAR pose to odom frame to align with gravity (correct 180 deg roll of camera_init)
    Eigen::Vector3d p_lidar = p_lidar_raw;
    Eigen::Quaterniond q_lidar = q_lidar_raw;
    try {
        geometry_msgs::msg::TransformStamped tf_odom_map = tf_buffer_->lookupTransform(
            odom_frame_, msg->header.frame_id, tf2::TimePointZero);
        
        Eigen::Vector3d t_odom_map(tf_odom_map.transform.translation.x,
                                   tf_odom_map.transform.translation.y,
                                   tf_odom_map.transform.translation.z);
        Eigen::Quaterniond q_odom_map(tf_odom_map.transform.rotation.w,
                                      tf_odom_map.transform.rotation.x,
                                      tf_odom_map.transform.rotation.y,
                                      tf_odom_map.transform.rotation.z);
        
        p_lidar = t_odom_map + q_odom_map * p_lidar_raw;
        q_lidar = (q_odom_map * q_lidar_raw).normalized();
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Could not transform LIO pose to odom frame: %s. Using raw LIO pose.", ex.what());
    }

    // 2. Look up the extrinsic transform: child_frame_id -> base_frame_ (pelvis)
    Eigen::Vector3d t_ext = t_imu_pelvis_;
    Eigen::Quaterniond q_ext = q_imu_pelvis_;

    if (!msg->child_frame_id.empty()) {
        try {
            // Find transform from LIO sensor frame to robot pelvis frame
            geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(
                base_frame_, msg->child_frame_id, tf2::TimePointZero);
            
            t_ext = Eigen::Vector3d(tf_msg.transform.translation.x, 
                                    tf_msg.transform.translation.y, 
                                    tf_msg.transform.translation.z);
            q_ext = Eigen::Quaterniond(tf_msg.transform.rotation.w, 
                                       tf_msg.transform.rotation.x, 
                                       tf_msg.transform.rotation.y, 
                                       tf_msg.transform.rotation.z);
        } catch (tf2::TransformException &ex) {
            // Use fallback static params if TF tree is not fully available yet
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "LIO TF lookup failed: %s. Using default static extrinsic.", ex.what());
        }
    }

    // 3. Compute pelvis pose: T_map_pelvis = T_map_lidar * T_lidar_pelvis
    LIOMeasurement lio;
    lio.timestamp = timestamp;
    lio.p = p_lidar + q_lidar * t_ext;
    lio.q = (q_lidar * q_ext).normalized();

    // 4. Update or initialize the estimator
    if (!estimator_.IsInitialized()) {
        State init_state;
        init_state.timestamp = timestamp;
        init_state.p = lio.p;
        init_state.q = lio.q;
        init_state.v.setZero();
        init_state.bg.setZero();
        init_state.ba.setZero();
        
        Eigen::Matrix<double, 15, 15> init_cov = Eigen::Matrix<double, 15, 15>::Identity() * 1e-2;
        estimator_.Initialize(init_state, init_cov);
        
        last_imu_time_ = timestamp;
        RCLCPP_INFO(this->get_logger(), "State Estimator Initialized from first LIO pose: [%.3f, %.3f, %.3f]", 
                    init_state.p.x(), init_state.p.y(), init_state.p.z());
        return;
    }

    if (!estimator_.UpdateLIO(lio)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "LIO update failed (possibly due to timestamp mismatch).");
    }
}

void StateEstimatorNode::PublishState(const State& state, const rclcpp::Time& stamp) {
    rclcpp::Time pub_stamp = use_host_time_ ? this->get_clock()->now() : stamp;

    // 1. Publish Odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = pub_stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Set position and orientation
    odom_msg.pose.pose.position.x = state.p.x();
    odom_msg.pose.pose.position.y = state.p.y();
    odom_msg.pose.pose.position.z = state.p.z();

    odom_msg.pose.pose.orientation.w = state.q.w();
    odom_msg.pose.pose.orientation.x = state.q.x();
    odom_msg.pose.pose.orientation.y = state.q.y();
    odom_msg.pose.pose.orientation.z = state.q.z();

    // Set velocity (represented in body-frame per ROS standards for child_frame_id)
    Eigen::Vector3d v_body = state.q.toRotationMatrix().transpose() * state.v;
    odom_msg.twist.twist.linear.x = v_body.x();
    odom_msg.twist.twist.linear.y = v_body.y();
    odom_msg.twist.twist.linear.z = v_body.z();

    // Publish
    odom_pub_->publish(odom_msg);

    // 2. Publish dynamic TF transform: odom -> pelvis
    if (publish_tf_) {
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = pub_stamp;
        tf_msg.header.frame_id = odom_frame_;
        tf_msg.child_frame_id = base_frame_;

        tf_msg.transform.translation.x = state.p.x();
        tf_msg.transform.translation.y = state.p.y();
        tf_msg.transform.translation.z = state.p.z();

        tf_msg.transform.rotation.w = state.q.w();
        tf_msg.transform.rotation.x = state.q.x();
        tf_msg.transform.rotation.y = state.q.y();
        tf_msg.transform.rotation.z = state.q.z();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    // 3. Publish path (throttled at 10Hz to save CPU/network bandwidth)
    static rclcpp::Time last_path_pub_time(0, 0, RCL_ROS_TIME);
    double elapsed = (pub_stamp - last_path_pub_time).seconds();
    if (elapsed >= 0.1 || elapsed < 0.0) {
        last_path_pub_time = pub_stamp;
        
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = pub_stamp;
        pose_stamped.header.frame_id = odom_frame_;
        pose_stamped.pose = odom_msg.pose.pose;
        
        path_msg_.header.stamp = pub_stamp;
        path_msg_.header.frame_id = odom_frame_;
        path_msg_.poses.push_back(pose_stamped);
        
        // Keep the path size bounded to prevent memory leakage
        if (path_msg_.poses.size() > 5000) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }
        
        path_pub_->publish(path_msg_);
    }
}

Eigen::Vector3d StateEstimatorNode::ParseVector3Parameter(const std::string& name, double default_val) {
    this->declare_parameter<std::vector<double>>(name, {default_val, default_val, default_val});
    auto param = this->get_parameter(name);
    if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        double val = param.as_double();
        return Eigen::Vector3d(val, val, val);
    } else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        std::vector<double> vals = param.as_double_array();
        if (vals.size() == 3) {
            return Eigen::Vector3d(vals[0], vals[1], vals[2]);
        } else if (vals.size() == 1) {
            return Eigen::Vector3d(vals[0], vals[0], vals[0]);
        }
    }
    return Eigen::Vector3d(default_val, default_val, default_val);
}
