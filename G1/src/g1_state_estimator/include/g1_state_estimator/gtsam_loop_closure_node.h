#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <g1_state_estimator/srv/save_map.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// PCL (Point Cloud Library)
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <mutex>
#include <thread>
#include <vector>
#include <deque>
#include <fstream>
#include <iomanip>

class GtsamLoopClosureNode : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    
    GtsamLoopClosureNode();
    ~GtsamLoopClosureNode();

private:
    // ROS 2 Subscriptions, Publishers, Services
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Service<g1_state_estimator::srv::SaveMap>::SharedPtr save_map_service_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS Parameters
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    double kf_dist_threshold_;
    double kf_angle_threshold_;
    bool loop_closure_enable_;
    double loop_search_radius_;
    double loop_search_time_diff_;
    double loop_frequency_;
    int min_history_keyframes_;
    double icp_fitness_score_;
    double gicp_max_distance_;
    int gicp_max_iterations_;
    double voxel_grid_leaf_size_;
    double map_publish_frequency_;
    std::string save_directory_;

    // Buffers for raw ROS messages
    std::deque<nav_msgs::msg::Odometry> odom_buf_;
    std::deque<sensor_msgs::msg::PointCloud2> cloud_buf_;
    std::mutex buf_mutex_;

    // Keyframe data structure
    struct Keyframe {
        int id;
        double timestamp;
        gtsam::Pose3 pose_odom;
        gtsam::Pose3 pose_optimized;
        pcl::PointCloud<PointT>::Ptr cloud;
    };
    std::vector<Keyframe> keyframes_;
    std::recursive_mutex kf_mutex_;

    // GTSAM Graph & Values
    gtsam::NonlinearFactorGraph gt_graph_;
    gtsam::Values gt_initial_values_;
    gtsam::Values gt_optimized_values_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr loop_noise_;

    // Dynamic Correction Transform
    Eigen::Affine3d T_map_odom_;
    std::mutex tf_mutex_;

    // Thread management
    std::thread loop_thread_;
    std::thread map_pub_thread_;
    bool run_threads_ = true;

    // Callbacks
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void CloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void SaveMapCallback(const std::shared_ptr<g1_state_estimator::srv::SaveMap::Request> request,
                         std::shared_ptr<g1_state_estimator::srv::SaveMap::Response> response);

    // Frame & Graph processing
    bool CheckAndCreateKeyframe(const nav_msgs::msg::Odometry& odom, const sensor_msgs::msg::PointCloud2& cloud);
    void AddKeyframeToGraph(const Keyframe& kf);

    // Backend threads
    void LoopClosureThread();
    void MapPublishThread();

    // Helper functions
    std::vector<int> SearchLoopCandidates(const gtsam::Pose3& curr_pose, double curr_time);
    bool AlignCloudsGICP(const pcl::PointCloud<PointT>::Ptr& source, 
                         const pcl::PointCloud<PointT>::Ptr& target,
                         const gtsam::Pose3& guess,
                         gtsam::Pose3& result_transform);
    void UpdateMapOdomTransform();
    void PublishTF();
    void PublishPath();
};
