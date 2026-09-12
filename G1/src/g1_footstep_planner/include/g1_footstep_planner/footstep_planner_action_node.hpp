#ifndef G1_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_ACTION_NODE_HPP_
#define G1_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav_msgs/msg/odometry.hpp"
#include "g1_footstep_planner/action/navigate_footstep.hpp"
#include "g1_footstep_planner/footstep_calculator.hpp"

namespace g1_footstep_planner
{

class FootstepPlannerActionNode : public rclcpp::Node
{
public:
  using NavigateFootstep = g1_footstep_planner::action::NavigateFootstep;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateFootstep>;

  explicit FootstepPlannerActionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~FootstepPlannerActionNode() = default;

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateFootstep::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void execute(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  std::tuple<double, double, double> get_current_robot_pose();

  void publish_footstep_markers(const std::vector<Footstep> & footsteps);

  void dog_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // ROS 2 Action Server
  rclcpp_action::Server<NavigateFootstep>::SharedPtr action_server_;

  // Publishers & Subscribers & TF
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_g1_mode_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_dog_odom_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters & Calculator
  FootstepCalculator calculator_;
  std::string cmd_vel_topic_;
  std::string g1_mode_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  bool use_odom_topic_{false};
  std::string odom_topic_{"/dog_odom"};

  // Topic Odometry Storage
  nav_msgs::msg::Odometry latest_dog_odom_;
  bool has_dog_odom_{false};
};


}  // namespace g1_footstep_planner

#endif  // G1_FOOTSTEP_PLANNER__FOOTSTEP_PLANNER_ACTION_NODE_HPP_
