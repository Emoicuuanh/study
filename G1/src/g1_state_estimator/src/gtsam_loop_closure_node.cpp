#include "g1_state_estimator/gtsam_loop_closure_node.h"
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

GtsamLoopClosureNode::GtsamLoopClosureNode() : Node("gtsam_loop_closure_node") {
    // 1. Declare and Load Parameters
    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    kf_dist_threshold_ = this->declare_parameter<double>("keyframe_distance", 1.0);
    kf_angle_threshold_ = this->declare_parameter<double>("keyframe_angle", 0.2);

    loop_closure_enable_ = this->declare_parameter<bool>("loop_closure_enable", true);
    loop_search_radius_ = this->declare_parameter<double>("loop_search_radius", 8.0);
    loop_search_time_diff_ = this->declare_parameter<double>("loop_search_time_diff", 30.0);
    loop_frequency_ = this->declare_parameter<double>("loop_frequency", 1.0);
    min_history_keyframes_ = this->declare_parameter<int>("min_history_keyframes", 30);

    icp_fitness_score_ = this->declare_parameter<double>("icp_fitness_score", 0.2);
    gicp_max_distance_ = this->declare_parameter<double>("gicp_max_distance", 1.0);
    gicp_max_iterations_ = this->declare_parameter<int>("gicp_max_iterations", 50);
    voxel_grid_leaf_size_ = this->declare_parameter<double>("voxel_grid_leaf_size", 0.4);

    map_publish_frequency_ = this->declare_parameter<double>("map_publish_frequency", 0.1);
    save_directory_ = this->declare_parameter<std::string>("save_directory", "/home/hoangdc/ROS2/unitree_G1/maps/");

    // Parse Noise parameters
    std::vector<double> odom_noise_sig = this->declare_parameter<std::vector<double>>(
        "odom_noise_sigmas", {0.001, 0.001, 0.001, 0.01, 0.01, 0.01});
    std::vector<double> loop_noise_sig = this->declare_parameter<std::vector<double>>(
        "loop_noise_sigmas", {0.0001, 0.0001, 0.0001, 0.001, 0.001, 0.001});

    // 2. Initialize GTSAM ISAM2
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam_ = std::make_unique<gtsam::ISAM2>(parameters);

    // Initialize GTSAM noise models
    if (odom_noise_sig.size() == 6) {
        odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << odom_noise_sig[0], odom_noise_sig[1], odom_noise_sig[2],
                                 odom_noise_sig[3], odom_noise_sig[4], odom_noise_sig[5]).finished());
    } else {
        odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-2, 1e-2, 1e-2).finished());
    }

    if (loop_noise_sig.size() == 6) {
        loop_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << loop_noise_sig[0], loop_noise_sig[1], loop_noise_sig[2],
                                 loop_noise_sig[3], loop_noise_sig[4], loop_noise_sig[5]).finished());
    } else {
        loop_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3).finished());
    }

    T_map_odom_ = Eigen::Affine3d::Identity();

    // 3. Setup Subscribers, Publishers, and Services
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimator/odom", 10, std::bind(&GtsamLoopClosureNode::OdomCallback, this, std::placeholders::_1));
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", 5, std::bind(&GtsamLoopClosureNode::CloudCallback, this, std::placeholders::_1));
        
    global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/state_estimator/global_map", 1);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/state_estimator/optimized_path", 5);
    
    save_map_service_ = this->create_service<g1_state_estimator::srv::SaveMap>(
        "/state_estimator/save_map", std::bind(&GtsamLoopClosureNode::SaveMapCallback, this, std::placeholders::_1, std::placeholders::_2));
        
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 4. Start Background threads
    if (loop_closure_enable_) {
        loop_thread_ = std::thread(&GtsamLoopClosureNode::LoopClosureThread, this);
    }
    map_pub_thread_ = std::thread(&GtsamLoopClosureNode::MapPublishThread, this);

    RCLCPP_INFO(this->get_logger(), "Decoupled Backend GTSAM Loop Closure Node Initialized successfully.");
}

GtsamLoopClosureNode::~GtsamLoopClosureNode() {
    run_threads_ = false;
    if (loop_thread_.joinable()) loop_thread_.join();
    if (map_pub_thread_.joinable()) map_pub_thread_.join();
}

void GtsamLoopClosureNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buf_mutex_);
    odom_buf_.push_back(*msg);
    
    // Periodically prune odom buffer if it gets too large
    while (odom_buf_.size() > 1000) {
        odom_buf_.pop_front();
    }
    
    // Publish TF map -> odom continuously
    PublishTF();
}

void GtsamLoopClosureNode::CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buf_mutex_);
    cloud_buf_.push_back(*msg);
    
    while (cloud_buf_.size() > 100) {
        cloud_buf_.pop_front();
    }

    // Try to match cloud and odom to create keyframes
    if (!odom_buf_.empty()) {
        double cloud_time = rclcpp::Time(msg->header.stamp).seconds();
        
        // Find closest odom in time
        int best_idx = -1;
        double min_dt = 1e9;
        for (size_t i = 0; i < odom_buf_.size(); ++i) {
            double odom_time = rclcpp::Time(odom_buf_[i].header.stamp).seconds();
            double dt = std::abs(cloud_time - odom_time);
            if (dt < min_dt) {
                min_dt = dt;
                best_idx = i;
            }
        }
        
        if (best_idx != -1 && min_dt < 0.1) {
            CheckAndCreateKeyframe(odom_buf_[best_idx], *msg);
        }
    }
}

bool GtsamLoopClosureNode::CheckAndCreateKeyframe(const nav_msgs::msg::Odometry& odom, const sensor_msgs::msg::PointCloud2& cloud) {
    std::lock_guard<std::recursive_mutex> lock_kf(kf_mutex_);
    
    gtsam::Pose3 pose_curr(
        gtsam::Rot3::Quaternion(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
                                odom.pose.pose.orientation.y, odom.pose.pose.orientation.z),
        gtsam::Point3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    double timestamp = rclcpp::Time(odom.header.stamp).seconds();

    // 1. Convert ROS PointCloud2 to PCL
    pcl::PointCloud<PointT>::Ptr raw_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(cloud, *raw_cloud);

    // 2. Transform raw_cloud to base_frame_ (base_link) at the cloud's timestamp using TF
    pcl::PointCloud<PointT>::Ptr cloud_in_base(new pcl::PointCloud<PointT>());
    try {
        geometry_msgs::msg::TransformStamped tf_msg = tf_buffer_->lookupTransform(
            base_frame_, cloud.header.frame_id, cloud.header.stamp, rclcpp::Duration::from_seconds(0.05));
        
        Eigen::Affine3d T_base_cloud = tf2::transformToEigen(tf_msg);
        pcl::transformPointCloud(*raw_cloud, *cloud_in_base, T_base_cloud.cast<float>());
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform cloud from %s to %s: %s. Using raw cloud.",
                    cloud.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        cloud_in_base = raw_cloud; // fallback
    }

    if (keyframes_.empty()) {
        Keyframe first_kf;
        first_kf.id = 0;
        first_kf.timestamp = timestamp;
        first_kf.pose_odom = pose_curr;
        first_kf.pose_optimized = pose_curr;
        
        first_kf.cloud.reset(new pcl::PointCloud<PointT>());
        
        // Voxel grid filter downsampling
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        voxel_grid.setInputCloud(cloud_in_base);
        voxel_grid.filter(*first_kf.cloud);
        
        keyframes_.push_back(first_kf);
        AddKeyframeToGraph(first_kf);
        RCLCPP_INFO(this->get_logger(), "Created initial keyframe 0");
        return true;
    }

    auto last_kf = keyframes_.back();
    gtsam::Pose3 diff = last_kf.pose_odom.between(pose_curr);
    double dist = diff.translation().norm();
    auto q_diff = diff.rotation().toQuaternion();
    double angle = 2.0 * std::acos(std::min(1.0, std::max(-1.0, q_diff.w())));

    if (dist > kf_dist_threshold_ || angle > kf_angle_threshold_) {
        Keyframe kf;
        kf.id = keyframes_.size();
        kf.timestamp = timestamp;
        kf.pose_odom = pose_curr;
        kf.pose_optimized = pose_curr;
        
        kf.cloud.reset(new pcl::PointCloud<PointT>());
        
        // Voxel grid filter downsampling
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setLeafSize(voxel_grid_leaf_size_, voxel_grid_leaf_size_, voxel_grid_leaf_size_);
        voxel_grid.setInputCloud(cloud_in_base);
        voxel_grid.filter(*kf.cloud);

        keyframes_.push_back(kf);
        AddKeyframeToGraph(kf);
        
        RCLCPP_INFO(this->get_logger(), "Created keyframe %zu (dist: %.2f m, angle: %.2f rad)", keyframes_.size() - 1, dist, angle);
        return true;
    }

    return false;
}

void GtsamLoopClosureNode::AddKeyframeToGraph(const Keyframe& kf) {
    if (kf.id == 0) {
        gt_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, kf.pose_odom, odom_noise_));
        gt_initial_values_.insert(0, kf.pose_odom);
    } else {
        auto prev_kf = keyframes_[kf.id - 1];
        gtsam::Pose3 rel_pose = prev_kf.pose_odom.between(kf.pose_odom);
        gt_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(kf.id - 1, kf.id, rel_pose, odom_noise_));
        gt_initial_values_.insert(kf.id, kf.pose_odom);
    }
    
    isam_->update(gt_graph_, gt_initial_values_);
    isam_->update();
    
    gt_graph_.resize(0);
    gt_initial_values_.clear();
    
    gt_optimized_values_ = isam_->calculateEstimate();
    
    // Update poses of keyframes
    for (auto& frame : keyframes_) {
        if (gt_optimized_values_.exists(frame.id)) {
            frame.pose_optimized = gt_optimized_values_.at<gtsam::Pose3>(frame.id);
        }
    }
    
    UpdateMapOdomTransform();
    PublishPath();
}

void GtsamLoopClosureNode::LoopClosureThread() {
    rclcpp::Rate rate(loop_frequency_);
    while (rclcpp::ok() && run_threads_) {
        rate.sleep();
        
        int curr_id;
        gtsam::Pose3 curr_pose;
        double curr_time;
        {
            std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
            if (keyframes_.size() < static_cast<size_t>(min_history_keyframes_)) continue;
            curr_id = keyframes_.size() - 1;
            curr_pose = keyframes_.back().pose_optimized;
            curr_time = keyframes_.back().timestamp;
        }

        // 1. Search loop closure candidates
        auto candidates = SearchLoopCandidates(curr_pose, curr_time);
        
        for (int candidate_id : candidates) {
            pcl::PointCloud<PointT>::Ptr cloud_curr(new pcl::PointCloud<PointT>());
            pcl::PointCloud<PointT>::Ptr cloud_cand(new pcl::PointCloud<PointT>());
            gtsam::Pose3 cand_pose;
            
            {
                std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
                cloud_curr = keyframes_[curr_id].cloud;
                cloud_cand = keyframes_[candidate_id].cloud;
                cand_pose = keyframes_[candidate_id].pose_optimized;
            }

            // 2. Align point clouds using GICP
            gtsam::Pose3 guess = cand_pose.between(curr_pose);
            gtsam::Pose3 relative_pose_corrected;
            
            if (AlignCloudsGICP(cloud_curr, cloud_cand, guess, relative_pose_corrected)) {
                RCLCPP_INFO(this->get_logger(), "🎯 Loop Closure Detected! Connecting Keyframe %d to %d", curr_id, candidate_id);
                
                // 3. Add Loop closure factor
                std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
                gt_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(candidate_id, curr_id, relative_pose_corrected, loop_noise_));
                
                isam_->update(gt_graph_);
                isam_->update();
                isam_->update();
                isam_->update();
                isam_->update();
                gt_graph_.resize(0);
                
                gt_optimized_values_ = isam_->calculateEstimate();
                
                // Propagate optimized values to all keyframes
                for (auto& frame : keyframes_) {
                    if (gt_optimized_values_.exists(frame.id)) {
                        frame.pose_optimized = gt_optimized_values_.at<gtsam::Pose3>(frame.id);
                    }
                }
                
                UpdateMapOdomTransform();
                PublishPath();
                break; // Process one loop closure per iteration to avoid CPU spikes
            }
        }
    }
}

std::vector<int> GtsamLoopClosureNode::SearchLoopCandidates(const gtsam::Pose3& curr_pose, double curr_time) {
    std::vector<int> candidates;
    std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
    int curr_id = keyframes_.size() - 1;

    for (int i = 0; i < curr_id - min_history_keyframes_; ++i) {
        double time_diff = curr_time - keyframes_[i].timestamp;
        if (time_diff < loop_search_time_diff_) continue;

        double dist = (keyframes_[i].pose_optimized.translation() - curr_pose.translation()).norm();
        if (dist < loop_search_radius_) {
            candidates.push_back(i);
        }
    }
    return candidates;
}

bool GtsamLoopClosureNode::AlignCloudsGICP(const pcl::PointCloud<PointT>::Ptr& source, 
                                          const pcl::PointCloud<PointT>::Ptr& target,
                                          const gtsam::Pose3& guess,
                                          gtsam::Pose3& result_transform) {
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setMaxCorrespondenceDistance(gicp_max_distance_);
    gicp.setMaximumIterations(gicp_max_iterations_);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);

    gicp.setInputSource(source);
    gicp.setInputTarget(target);

    pcl::PointCloud<PointT> aligned;
    gicp.align(aligned, guess.matrix().cast<float>());

    if (gicp.hasConverged()) {
        double score = gicp.getFitnessScore();
        if (score < icp_fitness_score_) {
            Eigen::Matrix4d T = gicp.getFinalTransformation().cast<double>();
            result_transform = gtsam::Pose3(T);
            return true;
        }
    }
    return false;
}

void GtsamLoopClosureNode::UpdateMapOdomTransform() {
    std::lock_guard<std::recursive_mutex> lock_kf(kf_mutex_);
    std::lock_guard<std::mutex> lock_tf(tf_mutex_);
    
    if (keyframes_.empty()) return;
    
    auto latest_kf = keyframes_.back();
    Eigen::Affine3d T_map_pelvis;
    T_map_pelvis.matrix() = latest_kf.pose_optimized.matrix();
    
    Eigen::Affine3d T_odom_pelvis;
    T_odom_pelvis.matrix() = latest_kf.pose_odom.matrix();
    
    T_map_odom_ = T_map_pelvis * T_odom_pelvis.inverse();
}

void GtsamLoopClosureNode::PublishTF() {
    std::lock_guard<std::mutex> lock(tf_mutex_);
    geometry_msgs::msg::TransformStamped tf_msg;
    
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;
    
    Eigen::Translation3d translation(T_map_odom_.translation());
    Eigen::Quaterniond rotation(T_map_odom_.rotation());
    
    tf_msg.transform.translation.x = translation.x();
    tf_msg.transform.translation.y = translation.y();
    tf_msg.transform.translation.z = translation.z();
    tf_msg.transform.rotation.x = rotation.x();
    tf_msg.transform.rotation.y = rotation.y();
    tf_msg.transform.rotation.z = rotation.z();
    tf_msg.transform.rotation.w = rotation.w();
    
    tf_broadcaster_->sendTransform(tf_msg);
}

void GtsamLoopClosureNode::PublishPath() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = map_frame_;
    
    {
        std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
        for (const auto& kf : keyframes_) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = rclcpp::Time(static_cast<uint64_t>(kf.timestamp * 1e9));
            pose_stamped.header.frame_id = map_frame_;
            
            auto t = kf.pose_optimized.translation();
            auto r = kf.pose_optimized.rotation().toQuaternion();
            
            pose_stamped.pose.position.x = t.x();
            pose_stamped.pose.position.y = t.y();
            pose_stamped.pose.position.z = t.z();
            pose_stamped.pose.orientation.x = r.x();
            pose_stamped.pose.orientation.y = r.y();
            pose_stamped.pose.orientation.z = r.z();
            pose_stamped.pose.orientation.w = r.w();
            
            path_msg.poses.push_back(pose_stamped);
        }
    }
    
    path_pub_->publish(path_msg);
}

void GtsamLoopClosureNode::MapPublishThread() {
    rclcpp::Rate rate(map_publish_frequency_);
    while (rclcpp::ok() && run_threads_) {
        rate.sleep();
        
        pcl::PointCloud<PointT>::Ptr global_map(new pcl::PointCloud<PointT>());
        
        {
            std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
            if (keyframes_.empty()) continue;
            
            for (const auto& kf : keyframes_) {
                pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>());
                Eigen::Affine3f T;
                T.matrix() = kf.pose_optimized.matrix().cast<float>();
                pcl::transformPointCloud(*kf.cloud, *transformed_cloud, T);
                *global_map += *transformed_cloud;
            }
        }
        
        if (!global_map->empty()) {
            sensor_msgs::msg::PointCloud2 map_msg;
            pcl::toROSMsg(*global_map, map_msg);
            map_msg.header.stamp = this->get_clock()->now();
            map_msg.header.frame_id = map_frame_;
            global_map_pub_->publish(map_msg);
        }
    }
}

void GtsamLoopClosureNode::SaveMapCallback(const std::shared_ptr<g1_state_estimator::srv::SaveMap::Request> request,
                                         std::shared_ptr<g1_state_estimator::srv::SaveMap::Response> response) {
    std::lock_guard<std::recursive_mutex> lock(kf_mutex_);
    
    // Choose save directory: use request path if provided, else fall back to parameter
    std::string save_dir = request->destination_path;
    if (save_dir.empty()) {
        this->get_parameter("save_directory", save_directory_);
        save_dir = save_directory_;
    }
    if (!save_dir.empty() && save_dir.back() != '/') {
        save_dir += "/";
    }
    
    // Create directory if it doesn't exist
    std::string make_dir_cmd = "mkdir -p " + save_dir;
    int system_res = std::system(make_dir_cmd.c_str());
    (void)system_res;

    pcl::PointCloud<PointT>::Ptr global_map(new pcl::PointCloud<PointT>());
    for (const auto& kf : keyframes_) {
        pcl::PointCloud<PointT>::Ptr temp(new pcl::PointCloud<PointT>());
        Eigen::Affine3f T;
        T.matrix() = kf.pose_optimized.matrix().cast<float>();
        pcl::transformPointCloud(*kf.cloud, *temp, T);
        *global_map += *temp;
    }
    
    // Choose file prefix: use request prefix if provided, else fall back to default
    std::string prefix = request->file_prefix;
    if (prefix.empty()) {
        prefix = "global_map";
    }
    
    std::string pcd_path = save_dir + prefix + ".pcd";
    std::string traj_path = save_dir + prefix + "_trajectory.txt";

    try {
        if (!global_map->empty()) {
            pcl::io::savePCDFileBinary(pcd_path, *global_map);
        }
        
        std::ofstream traj_file(traj_path);
        if (traj_file.is_open()) {
            for (const auto& kf : keyframes_) {
                auto t = kf.pose_optimized.translation();
                auto r = kf.pose_optimized.rotation().toQuaternion();
                traj_file << std::fixed << std::setprecision(6) << kf.timestamp << " "
                          << t.x() << " " << t.y() << " " << t.z() << " "
                          << r.x() << " " << r.y() << " " << r.z() << " " << r.w() << "\n";
            }
            traj_file.close();
        }
        
        response->success = true;
        response->message = "Saved map to " + pcd_path + " and trajectory to " + traj_path;
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to save map: ") + e.what();
    }
}
