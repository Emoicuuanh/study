#include "g1_footstep_planner/footstep_calculator.hpp"
#include <algorithm>
#include <cmath>

namespace g1_footstep_planner
{

FootstepCalculator::FootstepCalculator(
  double vx_exec,
  double vy_exec,
  double wz_exec,
  double v_min,
  double w_min,
  double deadband_safety_margin,
  double forward_tolerance,
  double strafe_tolerance,
  double yaw_tolerance,
  double max_stride_x,
  double max_stride_y,
  double max_stride_yaw,
  double stance_width,
  bool enable_closed_loop,
  double kp_linear,
  double kp_angular,
  double ki_linear,
  double ki_angular)
: vx_exec_(std::abs(vx_exec)),
  vy_exec_(std::abs(vy_exec)),
  wz_exec_(std::abs(wz_exec)),
  v_min_(std::abs(v_min)),
  w_min_(std::abs(w_min)),
  deadband_safety_margin_(std::abs(deadband_safety_margin)),
  forward_tolerance_(forward_tolerance),
  strafe_tolerance_(strafe_tolerance),
  yaw_tolerance_(yaw_tolerance),
  max_stride_x_(max_stride_x),
  max_stride_y_(max_stride_y),
  max_stride_yaw_(max_stride_yaw),
  stance_width_(stance_width),
  enable_closed_loop_(enable_closed_loop),
  kp_linear_(kp_linear),
  kp_angular_(kp_angular),
  ki_linear_(ki_linear),
  ki_angular_(ki_angular)
{

  // Apply configurable safety margin if velocity is at or below deadband
  if (vx_exec_ <= v_min_) {
    vx_exec_ = v_min_ + deadband_safety_margin_;
  }
  if (vy_exec_ <= v_min_) {
    vy_exec_ = v_min_ + deadband_safety_margin_;
  }
  if (wz_exec_ <= w_min_) {
    wz_exec_ = w_min_ + deadband_safety_margin_;
  }
}

double FootstepCalculator::normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

std::tuple<double, double, double> FootstepCalculator::calculate_relative_pose(
  double curr_x, double curr_y, double curr_yaw,
  double goal_x, double goal_y, double goal_yaw) const
{
  double dx_global = goal_x - curr_x;
  double dy_global = goal_y - curr_y;

  double cos_yaw = std::cos(curr_yaw);
  double sin_yaw = std::sin(curr_yaw);

  double dx_local = cos_yaw * dx_global + sin_yaw * dy_global;
  double dy_local = -sin_yaw * dx_global + cos_yaw * dy_global;
  double dyaw_local = normalize_angle(goal_yaw - curr_yaw);

  return std::make_tuple(dx_local, dy_local, dyaw_local);
}

std::vector<ExecutionImpulse> FootstepCalculator::plan_impulses(
  double dx, double dy, double dyaw) const
{
  std::vector<ExecutionImpulse> impulses;

  // Phase 1: Yaw Alignment
  if (std::abs(dyaw) > yaw_tolerance_) {
    double wz_cmd = (dyaw > 0) ? wz_exec_ : -wz_exec_;
    double duration_yaw = std::abs(dyaw) / wz_exec_;
    impulses.push_back({"1_YAW", 0.0, 0.0, wz_cmd, duration_yaw, std::abs(dyaw)});
  }

  // Phase 2: Lateral Strafe
  if (std::abs(dy) > strafe_tolerance_) {
    double vy_cmd = (dy > 0) ? vy_exec_ : -vy_exec_;
    double duration_y = std::abs(dy) / vy_exec_;
    impulses.push_back({"2_STRAFE", 0.0, vy_cmd, 0.0, duration_y, std::abs(dy)});
  }

  // Phase 3: Forward / Backward
  if (std::abs(dx) > forward_tolerance_) {
    double vx_cmd = (dx > 0) ? vx_exec_ : -vx_exec_;
    double duration_x = std::abs(dx) / vx_exec_;
    impulses.push_back({"3_FORWARD", vx_cmd, 0.0, 0.0, duration_x, std::abs(dx)});
  }

  return impulses;
}

std::vector<Footstep> FootstepCalculator::generate_discrete_footsteps(
  double dx, double dy, double dyaw) const
{
  std::vector<Footstep> footsteps;

  int num_steps_x = (std::abs(dx) > 0) ? static_cast<int>(std::ceil(std::abs(dx) / max_stride_x_)) : 0;
  int num_steps_y = (std::abs(dy) > 0) ? static_cast<int>(std::ceil(std::abs(dy) / max_stride_y_)) : 0;
  int num_steps_w = (std::abs(dyaw) > 0) ? static_cast<int>(std::ceil(std::abs(dyaw) / max_stride_yaw_)) : 0;

  int total_steps = std::max(std::max(num_steps_x, num_steps_y), num_steps_w);
  if (total_steps % 2 != 0) {
    total_steps += 1;
  }
  total_steps = std::max(2, total_steps);

  std::string curr_foot = "LEFT";
  for (int i = 1; i <= total_steps; ++i) {
    double alpha = static_cast<double>(i) / static_cast<double>(total_steps);
    double cx = alpha * dx;
    double cy = alpha * dy;
    double cyaw = alpha * dyaw;

    double sign = (curr_foot == "LEFT") ? 1.0 : -1.0;
    double fx = cx - sign * (stance_width_ / 2.0) * std::sin(cyaw);
    double fy = cy + sign * (stance_width_ / 2.0) * std::cos(cyaw);

    footsteps.push_back({curr_foot, fx, fy, cyaw, i});
    curr_foot = (curr_foot == "LEFT") ? "RIGHT" : "LEFT";
  }

  return footsteps;
}

}  // namespace g1_footstep_planner
