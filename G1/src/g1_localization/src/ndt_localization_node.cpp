#include "g1_localization/ndt_localization_node.hpp"
#include <pcl/io/pcd_io.h>

NdtLocalizationNode::NdtLocalizationNode() : Node("ndt_localization_node") {
    // 1. Declare and Load Parameters
    map_pcd_path_ = this->declare_parameter<std::string>("map_pcd_path", "");
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_ = this->declare_parameter<double>("ndt_step_size", 0.1);
    ndt_epsilon_ = this->declare_parameter<double>("ndt_epsilon", 1e-6);
    ndt_max_iterations_ = this->declare_parameter<int>("ndt_max_iterations", 35);
    live_voxel_size_ = this->declare_parameter<double>("live_voxel_size", 0.3);
    max_fitness_score_ = this->declare_parameter<double>("max_fitness_score", 0.5);

    // 2. Load Static Map
    loadStaticMap();

    // 3. Configure NDT Solvers (Coarse for multi-angle snapping, Fine for precision)
    ndt_coarse_.setTransformationEpsilon(ndt_epsilon_);
    ndt_coarse_.setStepSize(1.0);
    ndt_coarse_.setResolution(2.5);
    ndt_coarse_.setMaximumIterations(30);
    ndt_coarse_.setInputTarget(static_map_);

    ndt_fine_.setTransformationEpsilon(ndt_epsilon_);
    ndt_fine_.setStepSize(ndt_step_size_);
    ndt_fine_.setResolution(ndt_resolution_);
    ndt_fine_.setMaximumIterations(ndt_max_iterations_);
    ndt_fine_.setInputTarget(static_map_);

    // 4. Configure Voxel Filter
    voxel_filter_.setLeafSize(live_voxel_size_, live_voxel_size_, live_voxel_size_);

    // 5. Setup Interfaces
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 5, std::bind(&NdtLocalizationNode::cloudCallback, this, std::placeholders::_1));
  
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10, std::bind(&NdtLocalizationNode::initialPoseCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimator/odom", 10, std::bind(&NdtLocalizationNode::odomCallback, this, std::placeholders::_1));

    // Transient Local QoS configuration for the static map
    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/state_estimator/localization_map", map_qos);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/state_estimator/localized_pose", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/state_estimator/localized_path", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publish static map once
    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*static_map_, map_msg);
    map_msg.header.frame_id = map_frame_;
    map_msg.header.stamp = this->get_clock()->now();
    map_pub_->publish(map_msg);

    // 6. Optional Auto-initialization from parameters
    bool auto_init = this->declare_parameter<bool>("auto_initialize", false);
    double init_x = this->declare_parameter<double>("init_pose_x", 0.0);
    double init_y = this->declare_parameter<double>("init_pose_y", 0.0);
    double init_z = this->declare_parameter<double>("init_pose_z", 0.0);
    double init_yaw = this->declare_parameter<double>("init_pose_yaw", 0.0);

    if (auto_init) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        T_map_base_current_ = Eigen::Affine3d::Identity();
        T_map_base_current_.translation() = Eigen::Vector3d(init_x, init_y, init_z);
        T_map_base_current_.linear() = Eigen::AngleAxisd(init_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        is_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Auto-initialized pose from parameters: [%.2f, %.2f, %.2f, yaw: %.2f]",
                    init_x, init_y, init_z, init_yaw);
    }

    RCLCPP_INFO(this->get_logger(), "NDT 3D Localization Node fully initialized.");
}

void NdtLocalizationNode::loadStaticMap() {
    static_map_.reset(new pcl::PointCloud<PointT>());
    if (map_pcd_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'map_pcd_path' is empty! Cannot run localization.");
        return;
    }
    if (pcl::io::loadPCDFile<PointT>(map_pcd_path_, *static_map_) == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to read PCD map from: %s", map_pcd_path_.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Static map loaded successfully with %lu points.", static_map_->size());
}

void NdtLocalizationNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.push_back(*msg);
    if (odom_buffer_.size() > 2000) {
        odom_buffer_.pop_front();
    }
    has_odom_ = true;
}

void NdtLocalizationNode::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    T_map_base_current_.translation() = Eigen::Vector3d(
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
  
    T_map_base_current_.linear() = Eigen::Quaterniond(
        msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z).toRotationMatrix();

    is_initialized_ = true;
    initial_pose_needed_reinit_ = true; // Trigger multi-angle coarse search on next LiDAR frame
    last_cloud_time_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Manual initial pose set from RViz: [%.2f, %.2f, %.2f]",
                T_map_base_current_.translation().x(),
                T_map_base_current_.translation().y(),
                T_map_base_current_.translation().z());
}

bool NdtLocalizationNode::getRelativeOdom(double t_prev, double t_curr, Eigen::Affine3d& delta_odom) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_buffer_.size() < 2) return false;

    // Find closest odoms in time
    int idx_prev = -1, idx_curr = -1;
    double min_dt_prev = 1e9, min_dt_curr = 1e9;

    for (size_t i = 0; i < odom_buffer_.size(); ++i) {
        double t_odom = rclcpp::Time(odom_buffer_[i].header.stamp).seconds();
        double dt_prev = std::abs(t_odom - t_prev);
        double dt_curr = std::abs(t_odom - t_curr);

        if (dt_prev < min_dt_prev) { min_dt_prev = dt_prev; idx_prev = i; }
        if (dt_curr < min_dt_curr) { min_dt_curr = dt_curr; idx_curr = i; }
    }

    if (idx_prev != -1 && idx_curr != -1 && min_dt_prev < 0.1 && min_dt_curr < 0.1) {
        Eigen::Affine3d T_odom_prev, T_odom_curr;
        tf2::fromMsg(odom_buffer_[idx_prev].pose.pose, T_odom_prev);
        tf2::fromMsg(odom_buffer_[idx_curr].pose.pose, T_odom_curr);
    
        delta_odom = T_odom_prev.inverse() * T_odom_curr;
        return true;
    }
    return false;
}

void NdtLocalizationNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!is_initialized_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Localization is not initialized yet. Set 2D Pose Estimate on RViz.");
        return;
    }

    double current_cloud_time = rclcpp::Time(msg->header.stamp).seconds();

    // 1. Transform raw point cloud from source frame (e.g. camera_init) to robot base frame (base_link)
    pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*msg, *raw_cloud);
  
    pcl::PointCloud<PointT>::Ptr cloud_in_base(new pcl::PointCloud<PointT>());
    try {
        geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(
            base_frame_, msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.05));
        
        Eigen::Affine3d T_base_cloud = tf2::transformToEigen(tf_msg);
        pcl::transformPointCloud(*raw_cloud, *cloud_in_base, T_base_cloud.cast<float>());
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Could not transform cloud from %s to %s: %s. Skipping this frame.",
                             msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        return;
    }

    // 2. Voxel filter the transformed cloud to save CPU
    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
    voxel_filter_.setInputCloud(cloud_in_base);
    voxel_filter_.filter(*filtered_cloud);

    // 3. Compute motion guess from Odometry (T_map_base_guess)
    Eigen::Affine3d T_map_base_guess = Eigen::Affine3d::Identity();
    bool is_reinit = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (initial_pose_needed_reinit_) {
            is_reinit = true;
            initial_pose_needed_reinit_ = false;
        }

        Eigen::Affine3d delta_odom;
        if (!is_reinit && last_cloud_time_ > 0.0 && getRelativeOdom(last_cloud_time_, current_cloud_time, delta_odom)) {
            T_map_base_guess = T_map_base_current_ * delta_odom;
        } else {
            T_map_base_guess = T_map_base_current_;
        }
    }

    // 4. NDT Alignment
    ndt_fine_.setInputSource(filtered_cloud);

    Eigen::Affine3d initial_guess = T_map_base_guess;
    rclcpp::Time start_time = this->get_clock()->now();

    if (is_reinit) {
        RCLCPP_INFO(this->get_logger(), "New initial pose set. Running multi-angle coarse NDT alignment...");
        ndt_coarse_.setInputSource(filtered_cloud);
        double best_score = 1e9;
        // Search angles: -60, -45, -30, -15, 0, 15, 30, 45, 60 degrees around user's initial guess
        std::vector<double> angle_offsets = {-1.047, -0.785, -0.523, -0.261, 0.0, 0.261, 0.523, 0.785, 1.047};
        for (double d_yaw : angle_offsets) {
            Eigen::Affine3d candidate_guess = T_map_base_guess * Eigen::AngleAxisd(d_yaw, Eigen::Vector3d::UnitZ());
            pcl::PointCloud<PointT> aligned_coarse;
            ndt_coarse_.align(aligned_coarse, candidate_guess.matrix().cast<float>());
            if (ndt_coarse_.hasConverged()) {
                double score = ndt_coarse_.getFitnessScore();
                if (score < best_score) {
                    best_score = score;
                    initial_guess.matrix() = ndt_coarse_.getFinalTransformation().cast<double>();
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Multi-angle coarse alignment completed. Best coarse score: %.3f", best_score);
    }

    // Fine Stage (Direct tracking on odometry guess during normal operation)
    pcl::PointCloud<PointT> aligned_fine;
    ndt_fine_.align(aligned_fine, initial_guess.matrix().cast<float>());
    rclcpp::Time end_time = this->get_clock()->now();
    double solve_time_ms = (end_time - start_time).seconds() * 1000.0;

    if (ndt_fine_.hasConverged()) {
        double score = ndt_fine_.getFitnessScore();
        if (score < max_fitness_score_) {
            Eigen::Matrix4d T_ndt = ndt_fine_.getFinalTransformation().cast<double>();
        
            std::lock_guard<std::mutex> lock(state_mutex_);
            T_map_base_current_.matrix() = T_ndt;
            last_cloud_time_ = current_cloud_time;
        
            updateMapOdomTransform(T_map_base_current_);
            publishTFAndPose(msg->header.stamp);
        
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "NDT matched successfully: time: %.1fms, score: %.3f", solve_time_ms, score);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "NDT match rejected: fitness score too high (score: %.3f > limit: %.3f)", 
                                 score, max_fitness_score_);
            
            // Fallback: publish TF using predicted odometry guess pose
            std::lock_guard<std::mutex> lock(state_mutex_);
            T_map_base_current_ = T_map_base_guess;
            last_cloud_time_ = current_cloud_time;
            updateMapOdomTransform(T_map_base_current_);
            publishTFAndPose(msg->header.stamp);
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "NDT Solver failed to converge! Using odometry guess.");
        
        // Fallback: publish TF using predicted odometry guess pose
        std::lock_guard<std::mutex> lock(state_mutex_);
        T_map_base_current_ = T_map_base_guess;
        last_cloud_time_ = current_cloud_time;
        updateMapOdomTransform(T_map_base_current_);
        publishTFAndPose(msg->header.stamp);
    }
}

void NdtLocalizationNode::updateMapOdomTransform(const Eigen::Affine3d& T_map_base) {
    std::lock_guard<std::mutex> lock_odom(odom_mutex_);
    if (odom_buffer_.empty()) return;

    // Get the latest EKF odometry pose
    Eigen::Affine3d T_odom_base;
    tf2::fromMsg(odom_buffer_.back().pose.pose, T_odom_base);

    // T_map_odom = T_map_base * T_odom_base^-1
    T_map_odom_ = T_map_base * T_odom_base.inverse();
}

void NdtLocalizationNode::publishTFAndPose(const rclcpp::Time& stamp) {
    // 1. Publish TF map -> odom
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;

    tf_msg.transform = tf2::eigenToTransform(T_map_odom_).transform;
    tf_broadcaster_->sendTransform(tf_msg);

    // 2. Publish Localized Pose
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose = tf2::toMsg(T_map_base_current_);
    pose_pub_->publish(pose_msg);

    // 3. Publish Localized Path in map frame
    path_msg_.header.stamp = stamp;
    path_msg_.header.frame_id = map_frame_;
    path_msg_.poses.push_back(pose_msg);
    if (path_msg_.poses.size() > 2000) {
        path_msg_.poses.erase(path_msg_.poses.begin());
    }
    path_pub_->publish(path_msg_);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NdtLocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
