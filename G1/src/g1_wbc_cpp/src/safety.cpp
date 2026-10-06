#include "g1_wbc/safety.hpp"

#include <algorithm>
#include <cmath>

namespace g1_wbc
{

const char * toString(SafetyState s)
{
  switch (s) {
    case SafetyState::Disarmed: return "KHOA";
    case SafetyState::Ramping: return "DANG TANG";
    case SafetyState::Active: return "DANG CHAY";
    case SafetyState::Fault: return "LOI";
  }
  return "?";
}

SafetyGate::SafetyGate(const Eigen::VectorXd & effort_limit, const SafetyLimits & lim)
: lim_(lim)
{
  const int n = static_cast<int>(effort_limit.size());
  tau_max_ = (effort_limit.array() * lim_.tau_scale).cwiseMin(lim_.tau_abs_max);
  tau_out_ = Eigen::VectorXd::Zero(n);
  tau_prev_ = Eigen::VectorXd::Zero(n);
}

void SafetyGate::arm()
{
  // Co y KHONG cho arm() thoat khoi Fault: sau moi loi phai reset() mot cach
  // tuong minh, va reset() dua ve KHOA chu khong phai dang chay.
  if (rep_.state == SafetyState::Fault) {return;}
  rep_.state = SafetyState::Ramping;
  t_ramp_ = 0.0;
}

void SafetyGate::disarm()
{
  if (rep_.state != SafetyState::Fault) {rep_.state = SafetyState::Disarmed;}
  rep_.alpha = 0.0;
  t_ramp_ = 0.0;
}

void SafetyGate::reset()
{
  rep_.state = SafetyState::Disarmed;
  rep_.fault.clear();
  rep_.alpha = 0.0;
  t_ramp_ = 0.0;
  n_overrun_ = 0;
  n_qp_fail_ = 0;
  tau_prev_.setZero();
}

void SafetyGate::trip(const std::string & why)
{
  if (rep_.state == SafetyState::Fault) {return;}
  rep_.state = SafetyState::Fault;
  rep_.fault = why;
  rep_.alpha = 0.0;
  ++n_fault_;
  if (first_fault_.empty()) {first_fault_ = why;}
}

bool SafetyGate::checkFaults(const SafetyInputs & in, const Eigen::VectorXd & tau_des)
{
  // Thu tu kiem tra di tu "phep do khong dung duoc" den "tu the da hong": neu
  // du lieu vao da sai thi moi ket luan sau do deu vo nghia.
  if (in.state_age_ms > lim_.state_timeout_ms) {
    trip("mat lowstate " + std::to_string(in.state_age_ms) + " ms");
    return true;
  }
  if (!tau_des.allFinite() || !in.dq.allFinite() || !in.v_body.allFinite()) {
    trip("co gia tri khong huu han (NaN/Inf)");
    return true;
  }
  // Quaternion phai la don vi; lech nhieu = IMU hoac duong truyen co van de.
  const double qn = in.quat.norm();
  if (std::abs(qn - 1.0) > 0.05) {
    trip("quaternion khong chuan hoa (|q| = " + std::to_string(qn) + ")");
    return true;
  }

  n_qp_fail_ = in.qp_ok ? 0 : n_qp_fail_ + 1;
  if (n_qp_fail_ >= lim_.qp_fail_max) {
    trip("QP khong giai duoc " + std::to_string(n_qp_fail_) + " chu ky lien tiep");
    return true;
  }
  n_overrun_ = (in.cycle_ms > lim_.cycle_budget_ms) ? n_overrun_ + 1 : 0;
  if (n_overrun_ >= lim_.cycle_overrun_max) {
    trip("tre chu ky " + std::to_string(n_overrun_) + " lan lien tiep");
    return true;
  }

  // Goc nghieng so voi phuong thang dung, suy tu quaternion (w,x,y,z):
  // truc z cua than trong he the gioi la cot thu 3 cua R.
  const double w = in.quat[0], x = in.quat[1], y = in.quat[2], z = in.quat[3];
  const double zz = 1.0 - 2.0 * (x * x + y * y);        // R(2,2)
  const double tilt = std::acos(std::clamp(zz, -1.0, 1.0));
  if (tilt > lim_.tilt_max_rad) {
    trip("nghieng " + std::to_string(tilt * 57.2958) + " do");
    return true;
  }
  (void)w; (void)z;

  if (in.com_err.norm() > lim_.com_err_max) {
    trip("CoM lech " + std::to_string(in.com_err.norm() * 1000.0) + " mm");
    return true;
  }
  if (in.dq.cwiseAbs().maxCoeff() > lim_.dq_max) {
    trip("van toc khop " + std::to_string(in.dq.cwiseAbs().maxCoeff()) + " rad/s");
    return true;
  }
  if (in.v_body.norm() > lim_.v_body_max) {
    trip("van toc than " + std::to_string(in.v_body.norm()) + " m/s");
    return true;
  }
  if (lim_.require_both_feet && !(in.contact_left && in.contact_right)) {
    trip("mat tiep xuc mot ban chan");
    return true;
  }
  return false;
}

const Eigen::VectorXd & SafetyGate::filter(
  const Eigen::VectorXd & tau_des, const SafetyInputs & in, double dt)
{
  ++n_cycles_;
  rep_.n_clipped = 0;
  rep_.n_rate_limited = 0;
  rep_.max_clip = 0.0;

  // Kiem tra loi NGAY CA khi dang KHOA: chay o che do kho thi day chinh la thu
  // ta muon biet - "neu luc nay dang cam lai thi da ngat chua".
  checkFaults(in, tau_des);

  if (rep_.state == SafetyState::Fault) {
    rep_.alpha = 0.0;
    if (lim_.fault_action == FaultAction::Damping) {
      tau_out_ = (-lim_.kd_fault * in.dq).cwiseMax(-tau_max_).cwiseMin(tau_max_);
    } else {
      tau_out_.setZero();
    }
    tau_prev_ = tau_out_;
    return tau_out_;
  }
  if (rep_.state == SafetyState::Disarmed) {
    rep_.alpha = 0.0;
    tau_out_.setZero();
    tau_prev_.setZero();
    return tau_out_;
  }

  if (rep_.state == SafetyState::Ramping) {
    t_ramp_ += dt;
    rep_.alpha = (lim_.ramp_secs <= 0.0) ? 1.0 : std::min(1.0, t_ramp_ / lim_.ramp_secs);
    if (rep_.alpha >= 1.0) {rep_.state = SafetyState::Active;}
  } else {
    rep_.alpha = 1.0;
  }

  tau_out_ = rep_.alpha * tau_des;

  // Tran mo-men rieng.
  for (int i = 0; i < tau_out_.size(); ++i) {
    const double lo = -tau_max_[i], hi = tau_max_[i];
    if (tau_out_[i] > hi || tau_out_[i] < lo) {
      rep_.max_clip = std::max(rep_.max_clip, std::abs(tau_out_[i]) - tau_max_[i]);
      tau_out_[i] = std::clamp(tau_out_[i], lo, hi);
      ++rep_.n_clipped;
    }
  }
  // Chan toc do bien thien. Phai dat SAU tran: neu cat tran truoc thi buoc nhay
  // con lai van co the lon.
  const double dmax = lim_.tau_rate_max * std::max(dt, 1e-6);
  for (int i = 0; i < tau_out_.size(); ++i) {
    const double d = tau_out_[i] - tau_prev_[i];
    if (std::abs(d) > dmax) {
      tau_out_[i] = tau_prev_[i] + std::copysign(dmax, d);
      ++rep_.n_rate_limited;
    }
  }
  tau_prev_ = tau_out_;
  if (rep_.n_clipped || rep_.n_rate_limited) {++n_clipped_cycles_;}
  return tau_out_;
}

}  // namespace g1_wbc
