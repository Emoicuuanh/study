#ifndef G1_FOOTSTEP_PLANNER__FOOTSTEP_CALCULATOR_HPP_
#define G1_FOOTSTEP_PLANNER__FOOTSTEP_CALCULATOR_HPP_

#include <cmath>
#include <string>
#include <vector>
#include <tuple>
#include <iostream>

namespace g1_footstep_planner
{

struct Footstep
{
  std::string foot;  // "LEFT" or "RIGHT"
  double x;
  double y;
  double yaw;
  int step_index;
};

struct ExecutionImpulse
{
  std::string phase_name;
  double vx;
  double vy;
  double wz;
  double duration;
  double target_distance;  // Target displacement for phase (m or rad)
};

class FootstepCalculator
{
public:
  FootstepCalculator(
    double vx_exec = 0.12,
    double vy_exec = 0.12,
    double wz_exec = 0.12,
    double v_min = 0.10,
    double w_min = 0.10,
    double deadband_safety_margin = 0.02,
    double forward_tolerance = 0.005,
    double strafe_tolerance = 0.005,
    double yaw_tolerance = 0.01,
    double max_stride_x = 0.10,
    double max_stride_y = 0.05,
    double max_stride_yaw = 0.12,
    double stance_width = 0.20,
    bool enable_closed_loop = true,
    double kp_linear = 1.5,
    double kp_angular = 1.5,
    double ki_linear = 0.3,
    double ki_angular = 0.2
  );

  static double normalize_angle(double angle);

  std::tuple<double, double, double> calculate_relative_pose(
    double curr_x, double curr_y, double curr_yaw,
    double goal_x, double goal_y, double goal_yaw
  ) const;

  std::vector<ExecutionImpulse> plan_impulses(double dx, double dy, double dyaw) const;

  std::vector<Footstep> generate_discrete_footsteps(double dx, double dy, double dyaw) const;

  // Getters
  bool is_closed_loop_enabled() const { return enable_closed_loop_; }
  double get_kp_linear() const { return kp_linear_; }
  double get_kp_angular() const { return kp_angular_; }
  double get_ki_linear() const { return ki_linear_; }
  double get_ki_angular() const { return ki_angular_; }
  double get_v_min() const { return v_min_; }
  double get_w_min() const { return w_min_; }
  double get_deadband_margin() const { return deadband_safety_margin_; }
  double get_forward_tolerance() const { return forward_tolerance_; }
  double get_strafe_tolerance() const { return strafe_tolerance_; }
  double get_yaw_tolerance() const { return yaw_tolerance_; }


private:
  double vx_exec_;
  double vy_exec_;
  double wz_exec_;
  double v_min_;
  double w_min_;
  double deadband_safety_margin_;
  double forward_tolerance_;
  double strafe_tolerance_;
  double yaw_tolerance_;
  double max_stride_x_;
  double max_stride_y_;
  double max_stride_yaw_;
  double stance_width_;
  bool enable_closed_loop_;
  double kp_linear_;
  double kp_angular_;
  double ki_linear_;
  double ki_angular_;
};


}  // namespace g1_footstep_planner

#endif  // G1_FOOTSTEP_PLANNER__FOOTSTEP_CALCULATOR_HPP_
