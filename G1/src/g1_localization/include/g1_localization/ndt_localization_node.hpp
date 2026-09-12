#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>
#include <deque>

class NdtLocalizationNode : public rclcpp::Node {
public:
    using PointT = pcl::PointXYZI;
    NdtLocalizationNode();
    ~NdtLocalizationNode() = default;

private:
    void loadStaticMap();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  
    bool getRelativeOdom(double t_prev, double t_curr, Eigen::Affine3d& delta_odom);
    void updateMapOdomTransform(const Eigen::Affine3d& T_map_base);
    void publishTFAndPose(const rclcpp::Time& stamp);

    // ROS 2 Interfaces
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string map_pcd_path_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
  
    double ndt_resolution_;
    double ndt_step_size_;
    double ndt_epsilon_;
    int ndt_max_iterations_;
    double live_voxel_size_;
    double max_fitness_score_;
  
    // Core Maps & Algorithms
    pcl::PointCloud<PointT>::Ptr static_map_;
    pcl::NormalDistributionsTransform<PointT, PointT> ndt_coarse_;
    pcl::NormalDistributionsTransform<PointT, PointT> ndt_fine_;
    pcl::VoxelGrid<PointT> voxel_filter_;

    // States
    std::mutex state_mutex_;
    Eigen::Affine3d T_map_base_current_ = Eigen::Affine3d::Identity();
    Eigen::Affine3d T_map_odom_ = Eigen::Affine3d::Identity();
    double last_cloud_time_ = 0.0;
    bool is_initialized_ = false;
    bool initial_pose_needed_reinit_ = false;

    // Odometry Buffer
    std::mutex odom_mutex_;
    std::deque<nav_msgs::msg::Odometry> odom_buffer_;
    bool has_odom_ = false;
};
