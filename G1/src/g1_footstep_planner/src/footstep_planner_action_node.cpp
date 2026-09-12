#include "g1_footstep_planner/footstep_planner_action_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

namespace g1_footstep_planner
{

FootstepPlannerActionNode::FootstepPlannerActionNode(const rclcpp::NodeOptions & options)
: Node("g1_footstep_planner_node", options)
{
  // 1. Execution Velocities
  this->declare_parameter<double>("vx_exec", 0.12);
  this->declare_parameter<double>("vy_exec", 0.12);
  this->declare_parameter<double>("wz_exec", 0.12);

  // 2. SDK Velocity Deadbands & Safety Margin
  this->declare_parameter<double>("v_min", 0.10);
  this->declare_parameter<double>("w_min", 0.10);
  this->declare_parameter<double>("deadband_safety_margin", 0.02);

  // 3. Tolerances
  this->declare_parameter<double>("forward_tolerance", 0.005);
  this->declare_parameter<double>("strafe_tolerance", 0.005);
  this->declare_parameter<double>("yaw_tolerance", 0.01);

  // 4. Footstep Stride & Geometry Limits
  this->declare_parameter<double>("max_stride_x", 0.10);
  this->declare_parameter<double>("max_stride_y", 0.05);
  this->declare_parameter<double>("max_stride_yaw", 0.12);
  this->declare_parameter<double>("stance_width", 0.20);

  // 5. Topics & Frames
  this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_footstep");
  this->declare_parameter<std::string>("g1_mode_topic", "/g1_mode");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_link");

  // 6. Closed-Loop Odometry Feedback & PI Gains
  this->declare_parameter<bool>("enable_closed_loop", true);
  this->declare_parameter<double>("kp_linear", 1.5);
  this->declare_parameter<double>("kp_angular", 1.5);
  this->declare_parameter<double>("ki_linear", 0.3);
  this->declare_parameter<double>("ki_angular", 0.2);

  // 7. Active Cross-Track Damping & Limits
  this->declare_parameter<double>("kp_crosstrack_linear", 1.2);
  this->declare_parameter<double>("kp_crosstrack_angular", 1.0);
  this->declare_parameter<double>("max_crosstrack_linear_vel", 0.06);
  this->declare_parameter<double>("max_crosstrack_angular_vel", 0.08);
  this->declare_parameter<double>("anti_windup_limit", 0.03);

  // 8. Execution Ramp & Safety Timeout Parameters
  this->declare_parameter<double>("ramp_up_duration", 0.2);
  this->declare_parameter<double>("safety_timeout_multiplier", 3.0);
  this->declare_parameter<double>("safety_timeout_offset", 1.0);
  this->declare_parameter<double>("inter_phase_pause", 0.1);

  // 9. Terminal Square-Up Phase Parameters
  this->declare_parameter<bool>("enable_square_up", true);
  this->declare_parameter<double>("square_up_duration", 0.45);

  // 10. Odometry Source Parameters
  this->declare_parameter<bool>("use_odom_topic", false);
  this->declare_parameter<std::string>("odom_topic", "/dog_odom");

  use_odom_topic_ = this->get_parameter("use_odom_topic").as_bool();
  odom_topic_ = this->get_parameter("odom_topic").as_string();

  // Get parameter values
  double vx_exec = this->get_parameter("vx_exec").as_double();
  double vy_exec = this->get_parameter("vy_exec").as_double();
  double wz_exec = this->get_parameter("wz_exec").as_double();

  double v_min = this->get_parameter("v_min").as_double();
  double w_min = this->get_parameter("w_min").as_double();
  double deadband_safety_margin = this->get_parameter("deadband_safety_margin").as_double();

  double forward_tolerance = this->get_parameter("forward_tolerance").as_double();
  double strafe_tolerance = this->get_parameter("strafe_tolerance").as_double();
  double yaw_tolerance = this->get_parameter("yaw_tolerance").as_double();

  double max_stride_x = this->get_parameter("max_stride_x").as_double();
  double max_stride_y = this->get_parameter("max_stride_y").as_double();
  double max_stride_yaw = this->get_parameter("max_stride_yaw").as_double();
  double stance_width = this->get_parameter("stance_width").as_double();

  cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
  g1_mode_topic_ = this->get_parameter("g1_mode_topic").as_string();
  odom_frame_ = this->get_parameter("odom_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  bool enable_closed_loop = this->get_parameter("enable_closed_loop").as_bool();
  double kp_linear = this->get_parameter("kp_linear").as_double();
  double kp_angular = this->get_parameter("kp_angular").as_double();
  double ki_linear = this->get_parameter("ki_linear").as_double();
  double ki_angular = this->get_parameter("ki_angular").as_double();


  // Instantiate Calculator
  calculator_ = FootstepCalculator(
    vx_exec, vy_exec, wz_exec,
    v_min, w_min, deadband_safety_margin,
    forward_tolerance, strafe_tolerance, yaw_tolerance,
    max_stride_x, max_stride_y, max_stride_yaw, stance_width,
    enable_closed_loop, kp_linear, kp_angular, ki_linear, ki_angular
  );


  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  pub_g1_mode_ = this->create_publisher<std_msgs::msg::String>(g1_mode_topic_, 10);
  pub_status_ = this->create_publisher<std_msgs::msg::String>("/g1_footstep_status", 10);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/g1_footstep_markers", 10);

  // Subscribe to Odometry topic if requested
  if (use_odom_topic_) {
    sub_dog_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&FootstepPlannerActionNode::dog_odom_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "Selected Topic Odometry Feedback Source: '%s'", odom_topic_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Selected TF Odometry Feedback Source: '%s' -> '%s'", odom_frame_.c_str(), base_frame_.c_str());
  }


  // Create Action Server
  action_server_ = rclcpp_action::create_server<NavigateFootstep>(
    this,
    "navigate_footstep",
    std::bind(&FootstepPlannerActionNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&FootstepPlannerActionNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&FootstepPlannerActionNode::handle_accepted, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "==========================================================");
  RCLCPP_INFO(this->get_logger(), "  G1 Closed-Loop Odometry C++ Action Footstep Planner     ");
  RCLCPP_INFO(this->get_logger(), "==========================================================");
  RCLCPP_INFO(this->get_logger(), "Action Server Name    : /navigate_footstep");
  RCLCPP_INFO(this->get_logger(), "CmdVel Topic Output   : %s (Priority 75)", cmd_vel_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Closed-Loop Odom Mode : %s", enable_closed_loop ? "ENABLED" : "DISABLED");
  RCLCPP_INFO(this->get_logger(), "Kp Gains              : Kp_linear=%.2f, Kp_angular=%.2f", kp_linear, kp_angular);
}

rclcpp_action::GoalResponse FootstepPlannerActionNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateFootstep::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received Footstep Action Goal Request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FootstepPlannerActionNode::handle_cancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  (void)goal_handle;
  RCLCPP_WARN(this->get_logger(), "Received Goal Cancellation Request! Aborting active step execution.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FootstepPlannerActionNode::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::thread{std::bind(&FootstepPlannerActionNode::execute, this, goal_handle)}.detach();
}

void FootstepPlannerActionNode::dog_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_dog_odom_ = *msg;
  has_dog_odom_ = true;
}

std::tuple<double, double, double> FootstepPlannerActionNode::get_current_robot_pose()
{
  if (use_odom_topic_ && has_dog_odom_) {
    double tx = latest_dog_odom_.pose.pose.position.x;
    double ty = latest_dog_odom_.pose.pose.position.y;
    tf2::Quaternion q(
      latest_dog_odom_.pose.pose.orientation.x,
      latest_dog_odom_.pose.pose.orientation.y,
      latest_dog_odom_.pose.pose.orientation.z,
      latest_dog_odom_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return std::make_tuple(tx, ty, yaw);
  }

  std::string err_msg;
  if (!tf_buffer_->canTransform(odom_frame_, base_frame_, tf2::TimePointZero, &err_msg)) {
    return std::make_tuple(0.0, 0.0, 0.0);
  }

  try {
    geometry_msgs::msg::TransformStamped trans = tf_buffer_->lookupTransform(
      odom_frame_, base_frame_, tf2::TimePointZero);

    double tx = trans.transform.translation.x;
    double ty = trans.transform.translation.y;

    tf2::Quaternion q(
      trans.transform.rotation.x,
      trans.transform.rotation.y,
      trans.transform.rotation.z,
      trans.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return std::make_tuple(tx, ty, yaw);
  } catch (const tf2::TransformException & ex) {
    return std::make_tuple(0.0, 0.0, 0.0);
  }
}


void FootstepPlannerActionNode::publish_footstep_markers(const std::vector<Footstep> & footsteps)
{
  visualization_msgs::msg::MarkerArray marker_array;

  for (const auto & step : footsteps) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_;
    marker.header.stamp = this->now();
    marker.ns = "g1_footsteps";
    marker.id = step.step_index;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = step.x;
    marker.pose.position.y = step.y;
    marker.pose.position.z = 0.02;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, step.yaw);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 0.16;
    marker.scale.y = 0.08;
    marker.scale.z = 0.02;

    if (step.foot == "LEFT") {
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.2f;
    } else {
      marker.color.r = 0.0f;
      marker.color.g = 0.4f;
      marker.color.b = 1.0f;
    }
    marker.color.a = 0.8f;

    marker_array.markers.push_back(marker);
  }

  pub_markers_->publish(marker_array);
}

void FootstepPlannerActionNode::execute(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing Footstep Action Goal...");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<NavigateFootstep::Feedback>();
  auto result = std::make_shared<NavigateFootstep::Result>();

  auto [p_init_x, p_init_y, p_init_yaw] = get_current_robot_pose();

  double goal_x = goal->target_pose.pose.position.x;
  double goal_y = goal->target_pose.pose.position.y;

  tf2::Quaternion q(
    goal->target_pose.pose.orientation.x,
    goal->target_pose.pose.orientation.y,
    goal->target_pose.pose.orientation.z,
    goal->target_pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, goal_yaw;
  m.getRPY(roll, pitch, goal_yaw);

  double dx = 0.0;
  double dy = 0.0;
  double dyaw = 0.0;

  std::string frame_id = goal->target_pose.header.frame_id;
  if (frame_id.empty() || frame_id == base_frame_) {
    dx = goal_x;
    dy = goal_y;
    dyaw = goal_yaw;
    RCLCPP_INFO(this->get_logger(), "Target Pose interpreted as RELATIVE offset in '%s': dx=%.3fm, dy=%.3fm, dyaw=%.3frad",
      base_frame_.c_str(), dx, dy, dyaw);
  } else {
    std::tie(dx, dy, dyaw) = calculator_.calculate_relative_pose(p_init_x, p_init_y, p_init_yaw, goal_x, goal_y, goal_yaw);
    RCLCPP_INFO(this->get_logger(), "Target Pose transformed from '%s': dx=%.3fm, dy=%.3fm, dyaw=%.3frad",
      frame_id.c_str(), dx, dy, dyaw);
  }

  // Plan 3-phase impulses
  auto impulses = calculator_.plan_impulses(dx, dy, dyaw);
  auto footsteps = calculator_.generate_discrete_footsteps(dx, dy, dyaw);
  publish_footstep_markers(footsteps);

  if (impulses.empty()) {
    RCLCPP_INFO(this->get_logger(), "Target goal already reached!");
    result->success = true;
    result->message = "Target already within tolerance";
    result->final_error_x = 0.0f;
    result->final_error_y = 0.0f;
    result->final_error_yaw = 0.0f;
    goal_handle->succeed(result);
    return;
  }

  double total_duration = 0.0;
  for (const auto & imp : impulses) {
    total_duration += imp.duration;
  }

  double elapsed_total = 0.0;
  rclcpp::Rate rate(20);  // 20Hz Closed-Loop Control

  for (size_t i = 0; i < impulses.size(); ++i) {
    const auto & impulse = impulses[i];
    RCLCPP_INFO(this->get_logger(), "Executing Phase %zu/%zu (%s): Target Distance = %.3f",
      i+1, impulses.size(), impulse.phase_name.c_str(), impulse.target_distance);

    auto phase_start_time = this->now();
    bool odom_active = tf_buffer_->canTransform(odom_frame_, base_frame_, tf2::TimePointZero);
    auto [p_start_x, p_start_y, p_start_yaw] = get_current_robot_pose();

    double tolerance = (impulse.phase_name == "1_YAW") ? calculator_.get_yaw_tolerance() :
                       (impulse.phase_name == "2_STRAFE") ? calculator_.get_strafe_tolerance() :
                       calculator_.get_forward_tolerance();

    // Fetch dynamic parameters
    double kp_ct_lin = this->get_parameter("kp_crosstrack_linear").as_double();
    double kp_ct_ang = this->get_parameter("kp_crosstrack_angular").as_double();
    double max_ct_lin = this->get_parameter("max_crosstrack_linear_vel").as_double();
    double max_ct_ang = this->get_parameter("max_crosstrack_angular_vel").as_double();
    double windup_lim = this->get_parameter("anti_windup_limit").as_double();

    double ramp_up_dur = this->get_parameter("ramp_up_duration").as_double();
    double timeout_mult = this->get_parameter("safety_timeout_multiplier").as_double();
    double timeout_off = this->get_parameter("safety_timeout_offset").as_double();
    double pause_dur = this->get_parameter("inter_phase_pause").as_double();

    // Reset PI Integral Accumulators for the phase
    double int_err_x = 0.0;
    double int_err_y = 0.0;
    double int_err_yaw = 0.0;
    double ki_lin = calculator_.get_ki_linear();
    double ki_ang = calculator_.get_ki_angular();

    while (rclcpp::ok()) {
      // Check Cancellation
      if (goal_handle->is_canceling()) {
        geometry_msgs::msg::Twist stop_cmd;
        pub_cmd_vel_->publish(stop_cmd);
        std_msgs::msg::String mode_msg;
        mode_msg.data = "balance_stand";
        pub_g1_mode_->publish(mode_msg);

        result->success = false;
        result->message = "Action Goal Canceled by User";
        goal_handle->canceled(result);
        RCLCPP_WARN(this->get_logger(), "Action Goal Canceled & Robot Locked.");
        return;
      }

      auto now = this->now();
      double phase_elapsed = (now - phase_start_time).seconds();

      // Check current traveled distance via TF Odometry Feedback if available
      double traveled = 0.0;
      if (odom_active && tf_buffer_->canTransform(odom_frame_, base_frame_, tf2::TimePointZero)) {
        auto [c_x, c_y, c_yaw] = get_current_robot_pose();
        double dx_g = c_x - p_start_x;
        double dy_g = c_y - p_start_y;

        if (impulse.phase_name == "1_YAW") {
          traveled = std::abs(FootstepCalculator::normalize_angle(c_yaw - p_start_yaw));
        } else if (impulse.phase_name == "2_STRAFE") {
          double dy_loc = -std::sin(p_start_yaw) * dx_g + std::cos(p_start_yaw) * dy_g;
          traveled = std::abs(dy_loc);
        } else { // 3_FORWARD
          double dx_loc = std::cos(p_start_yaw) * dx_g + std::sin(p_start_yaw) * dy_g;
          traveled = std::abs(dx_loc);
        }
      } else {
        // Fallback to dead-reckoning time-integration
        double base_v = (impulse.phase_name == "1_YAW") ? std::abs(impulse.wz) :
                        (impulse.phase_name == "2_STRAFE") ? std::abs(impulse.vy) : std::abs(impulse.vx);
        traveled = base_v * phase_elapsed;
      }

      double remaining = impulse.target_distance - traveled;

      // Completion check: Reached target distance OR safety timeout
      if (remaining <= tolerance || phase_elapsed >= (impulse.duration * timeout_mult + timeout_off)) {
        RCLCPP_INFO(this->get_logger(), "Phase '%s' Completed! Traveled: %.3f / %.3f (rem: %.3f)",
          impulse.phase_name.c_str(), traveled, impulse.target_distance, remaining);
        break;
      }

      // Closed-Loop Velocity Control with Soft Deceleration Ramp & Active Cross-Track Compensation
      double min_v = (impulse.phase_name == "1_YAW") ? calculator_.get_w_min() : calculator_.get_v_min();
      double min_exec_v = min_v + calculator_.get_deadband_margin();
      double kp = (impulse.phase_name == "1_YAW") ? calculator_.get_kp_angular() : calculator_.get_kp_linear();

      double v_cmd_mag = std::abs(kp * remaining);
      double max_v_mag = (impulse.phase_name == "1_YAW") ? std::abs(impulse.wz) :
                         (impulse.phase_name == "2_STRAFE") ? std::abs(impulse.vy) : std::abs(impulse.vx);
      
      // Clamp velocity magnitude
      v_cmd_mag = std::min(max_v_mag, std::max(min_exec_v, v_cmd_mag));

      // Soft Ramp-Up
      double ramp_up = (ramp_up_dur > 0.0) ? std::min(1.0, phase_elapsed / ramp_up_dur) : 1.0;
      v_cmd_mag *= ramp_up;

      // Apply Direction with Active 3-DOF Cross-Track PI Compensation (Neutralizing cross-axis drift)
      geometry_msgs::msg::Twist cmd;
      if (odom_active && tf_buffer_->canTransform(odom_frame_, base_frame_, tf2::TimePointZero)) {
        auto [c_x, c_y, c_yaw] = get_current_robot_pose();
        double dx_g = c_x - p_start_x;
        double dy_g = c_y - p_start_y;
        double dx_loc = std::cos(p_start_yaw) * dx_g + std::sin(p_start_yaw) * dy_g;
        double dy_loc = -std::sin(p_start_yaw) * dx_g + std::cos(p_start_yaw) * dy_g;
        double dyaw_loc = FootstepCalculator::normalize_angle(c_yaw - p_start_yaw);

        // Accumulate integrals with anti-windup clamping
        double dt = 0.05; // 20Hz loop step
        int_err_x = std::clamp(int_err_x + (-dx_loc) * dt, -windup_lim, windup_lim);
        int_err_y = std::clamp(int_err_y + (-dy_loc) * dt, -windup_lim, windup_lim);
        int_err_yaw = std::clamp(int_err_yaw + (-dyaw_loc) * dt, -windup_lim, windup_lim);

        if (impulse.phase_name == "1_YAW") {
          cmd.angular.z = (impulse.wz > 0 ? 1.0 : -1.0) * v_cmd_mag;
          // Active PI damping of X and Y drift during in-place turning
          cmd.linear.x = std::clamp(-kp_ct_lin * dx_loc + ki_lin * int_err_x, -max_ct_lin, max_ct_lin);
          cmd.linear.y = std::clamp(-kp_ct_lin * dy_loc + ki_lin * int_err_y, -max_ct_lin, max_ct_lin);
        } else if (impulse.phase_name == "2_STRAFE") {
          cmd.linear.y = (impulse.vy > 0 ? 1.0 : -1.0) * v_cmd_mag;
          // Active PI Cross-Track: Damping forward/backward (X) & Yaw drift during lateral strafe
          cmd.linear.x = std::clamp(-kp_ct_lin * dx_loc + ki_lin * int_err_x, -max_ct_lin, max_ct_lin);
          cmd.angular.z = std::clamp(-kp_ct_ang * dyaw_loc + ki_ang * int_err_yaw, -max_ct_ang, max_ct_ang);
        } else if (impulse.phase_name == "3_FORWARD") {
          cmd.linear.x = (impulse.vx > 0 ? 1.0 : -1.0) * v_cmd_mag;
          // Active PI Cross-Track: Damping lateral sway (Y) & Yaw drift during forward walk
          cmd.linear.y = std::clamp(-kp_ct_lin * dy_loc + ki_lin * int_err_y, -max_ct_lin, max_ct_lin);
          cmd.angular.z = std::clamp(-kp_ct_ang * dyaw_loc + ki_ang * int_err_yaw, -max_ct_ang, max_ct_ang);
        }
      } else {
        if (impulse.phase_name == "1_YAW") {
          cmd.angular.z = (impulse.wz > 0 ? 1.0 : -1.0) * v_cmd_mag;
        } else if (impulse.phase_name == "2_STRAFE") {
          cmd.linear.y = (impulse.vy > 0 ? 1.0 : -1.0) * v_cmd_mag;
        } else if (impulse.phase_name == "3_FORWARD") {
          cmd.linear.x = (impulse.vx > 0 ? 1.0 : -1.0) * v_cmd_mag;
        }
      }

      pub_cmd_vel_->publish(cmd);

      // Publish Feedback
      elapsed_total += 0.05;
      feedback->current_phase = impulse.phase_name;
      feedback->percent_complete = std::min(100.0f, static_cast<float>((traveled / impulse.target_distance) * 100.0));
      feedback->time_remaining = std::max(0.0f, static_cast<float>(total_duration - elapsed_total));

      if (odom_active && tf_buffer_->canTransform(odom_frame_, base_frame_, tf2::TimePointZero)) {
        auto [c_x, c_y, c_yaw] = get_current_robot_pose();
        feedback->current_pose.header.frame_id = odom_frame_;
        feedback->current_pose.header.stamp = now;
        feedback->current_pose.pose.position.x = c_x;
        feedback->current_pose.pose.position.y = c_y;
        tf2::Quaternion q_curr;
        q_curr.setRPY(0.0, 0.0, c_yaw);
        feedback->current_pose.pose.orientation.x = q_curr.x();
        feedback->current_pose.pose.orientation.y = q_curr.y();
        feedback->current_pose.pose.orientation.z = q_curr.z();
        feedback->current_pose.pose.orientation.w = q_curr.w();
      }

      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    // Inter-phase pause
    geometry_msgs::msg::Twist stop_cmd;
    pub_cmd_vel_->publish(stop_cmd);
    if (pause_dur > 0.0) {
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(pause_dur * 1000.0)));
    }
  }

  // Terminal Micro-Square-Up Settling Phase:
  bool enable_sq = this->get_parameter("enable_square_up").as_bool();
  double sq_dur = this->get_parameter("square_up_duration").as_double();
  if (enable_sq && sq_dur > 0.0) {
    RCLCPP_INFO(this->get_logger(), "Executing Terminal Micro-Square-Up Phase (%.2fs settling feet side-by-side)...", sq_dur);
    geometry_msgs::msg::Twist square_up_cmd;
    rclcpp::Rate square_up_rate(20); // 20Hz
    int sq_steps = static_cast<int>(sq_dur * 20.0);
    for (int step = 0; step < sq_steps; ++step) {
      pub_cmd_vel_->publish(square_up_cmd);
      square_up_rate.sleep();
    }
  }

  // Lock Feet / BalanceStand on Completion
  std_msgs::msg::String mode_msg;
  mode_msg.data = "balance_stand";
  pub_g1_mode_->publish(mode_msg);




  // Final Pose Verification & Logging
  float final_err_x = 0.0f;
  float final_err_y = 0.0f;
  float final_err_yaw = 0.0f;

  if (tf_buffer_->canTransform(odom_frame_, base_frame_, tf2::TimePointZero)) {
    auto [end_x, end_y, end_yaw] = get_current_robot_pose();
    
    // Project global displacement into local frame at start of execution
    double dx_global = end_x - p_init_x;
    double dy_global = end_y - p_init_y;
    double cos_init = std::cos(p_init_yaw);
    double sin_init = std::sin(p_init_yaw);

    double dx_achieved = cos_init * dx_global + sin_init * dy_global;
    double dy_achieved = -sin_init * dx_global + cos_init * dy_global;
    double dyaw_achieved = FootstepCalculator::normalize_angle(end_yaw - p_init_yaw);

    final_err_x = static_cast<float>(dx - dx_achieved);
    final_err_y = static_cast<float>(dy - dy_achieved);
    final_err_yaw = static_cast<float>(FootstepCalculator::normalize_angle(dyaw - dyaw_achieved));

    RCLCPP_INFO(this->get_logger(), "==========================================================");
    RCLCPP_INFO(this->get_logger(), "📍 INITIAL ODOM POSE : x=%.4fm, y=%.4fm, yaw=%.4frad", p_init_x, p_init_y, p_init_yaw);
    RCLCPP_INFO(this->get_logger(), "🏁 FINAL ODOM POSE   : x=%.4fm, y=%.4fm, yaw=%.4frad", end_x, end_y, end_yaw);
    RCLCPP_INFO(this->get_logger(), "🎯 TARGET RELATIVE   : dx=%.4fm, dy=%.4fm, dyaw=%.4frad", dx, dy, dyaw);
    RCLCPP_INFO(this->get_logger(), "📏 ACHIEVED MOVEMENT : dx=%.4fm, dy=%.4fm, dyaw=%.4frad", dx_achieved, dy_achieved, dyaw_achieved);
    RCLCPP_INFO(this->get_logger(), "❌ REMAINING ERRORS  : err_x=%.4fm, err_y=%.4fm, err_yaw=%.4frad", final_err_x, final_err_y, final_err_yaw);
    RCLCPP_INFO(this->get_logger(), "==========================================================");
  }

  result->success = true;
  result->message = "Footstep Action Goal Successfully Executed with Closed-Loop Precision!";
  result->final_error_x = final_err_x;
  result->final_error_y = final_err_y;
  result->final_error_yaw = final_err_yaw;




  result->success = true;
  result->message = "Footstep Action Goal Successfully Executed with Closed-Loop Precision!";
  result->final_error_x = final_err_x;
  result->final_error_y = final_err_y;
  result->final_error_yaw = final_err_yaw;

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "==========================================================");
  RCLCPP_INFO(this->get_logger(), "  Action Goal Succeeded! Closed-Loop Feedback Completed.  ");
  RCLCPP_INFO(this->get_logger(), "==========================================================");
}

}  // namespace g1_footstep_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<g1_footstep_planner::FootstepPlannerActionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
