#include "g1_wbc/leg_odometry.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <stdexcept>

namespace g1_wbc
{
namespace
{
constexpr int kNMotorOdom = 29;
const std::array<std::string, 2> kFoot = {"left_ankle_roll_link", "right_ankle_roll_link"};
// Thu tu khop cua Unitree SDK (G1JointIndex): 0-5 chan trai, 6-11 chan phai.
inline int legStart(int side) {return side * 6;}
}  // namespace

LegOdometry::LegOdometry(
  const std::string & urdf_path, double dls_lambda, double contact_on,
  double contact_off, bool fusion_force)
: dls_lambda_(dls_lambda), contact_on_(contact_on), contact_off_(contact_off),
  fusion_force_(fusion_force)
{
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
  if (model_.nv != 6 + kNMotorOdom) {
    throw std::runtime_error("URDF khong phai g1_29dof (nv khong khop)");
  }
  data_ = pinocchio::Data(model_);
  for (int s = 0; s < 2; ++s) {fids_[s] = model_.getFrameId(kFoot[s]);}
  q_ = pinocchio::neutral(model_);
}

void LegOdometry::perFrame(
  const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
  const Eigen::Vector3d & gyro, const Eigen::VectorXd & tau_est,
  const Eigen::Vector4d & quat_wxyz, OdomResult & out)
{
  // FK voi than GHIM TAI GOC, huong don vi - co y khong dung pose that cua than,
  // vi robot that khong biet pose do.
  q_.head<3>().setZero();
  q_.segment<4>(3) << 0.0, 0.0, 0.0, 1.0;   // pinocchio dung (x,y,z,w)
  q_.tail(kNMotorOdom) = q_motor;
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeJointJacobians(model_, data_, q_);

  // Mo-men trong luc cua chan: trong luc quay ve he than (cho nay CAN huong IMU).
  const Eigen::Matrix3d R =
    Eigen::Quaterniond(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3])
    .normalized().toRotationMatrix();
  model_.gravity.linear(R.transpose() * Eigen::Vector3d(0.0, 0.0, -9.81));
  pinocchio::computeGeneralizedGravity(model_, data_, q_);
  const Eigen::VectorXd tau_g = data_.g;

  Eigen::MatrixXd J(6, model_.nv);
  for (int s = 0; s < 2; ++s) {
    const int m0 = legStart(s), v0 = m0 + 6;
    const Eigen::Vector3d p_BF = data_.oMf[fids_[s]].translation();
    J.setZero();
    pinocchio::getFrameJacobian(model_, data_, fids_[s], pinocchio::LOCAL_WORLD_ALIGNED, J);
    const Eigen::Matrix<double, 6, 6> J6 = J.block(0, v0, 6, 6);
    out.v_foot[s] = -(gyro.cross(p_BF) + J6.topRows<3>() * dq_motor.segment<6>(m0));

    // Wrench tiep xuc 6D. Luc diem 3D tai goc co chan khong sinh noi mo-men
    // co chan = fz*(CoP - co chan) -> he mau thuan, nghiem rac.
    const Eigen::Matrix<double, 6, 1> rhs =
      tau_g.segment<6>(v0) - tau_est.segment<6>(m0);
    const Eigen::Matrix<double, 6, 6> A =
      J6 * J6.transpose() + dls_lambda_ * dls_lambda_ * Eigen::Matrix<double, 6, 6>::Identity();
    const Eigen::Matrix<double, 6, 1> wr = A.lu().solve(Eigen::Matrix<double, 6, 1>(J6 * rhs));
    out.wrench[s].head<3>() = R * wr.head<3>();
    out.wrench[s].tail<3>() = R * wr.tail<3>();
    out.fz[s] = out.wrench[s][2];
  }
}

const OdomResult & LegOdometry::update(
  const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
  const Eigen::Vector3d & gyro, const Eigen::VectorXd & tau_est,
  const Eigen::Vector4d & quat_wxyz)
{
  perFrame(q_motor, dq_motor, gyro, tau_est, quat_wxyz, res_);

  // Tre nguong (hysteresis) de khong nhay trang thai.
  for (int s = 0; s < 2; ++s) {
    in_contact_[s] = in_contact_[s] ? (res_.fz[s] > contact_off_)
      : (res_.fz[s] > contact_on_);
    res_.contact[s] = in_contact_[s];
  }

  std::array<int, 2> stance{{0, 0}};
  int n = 0;
  for (int s = 0; s < 2; ++s) {if (in_contact_[s]) {stance[n++] = s;}}
  if (n == 0) {stance = {{0, 1}}; n = 2;}      // bay ca hai chan: giu phep do cu
  res_.n_contact = n;

  double w[2] = {1.0, 1.0}, sum = 0.0;
  if (fusion_force_) {
    for (int i = 0; i < n; ++i) {w[i] = std::max(res_.fz[stance[i]], 0.0);}
    double s2 = 0.0;
    for (int i = 0; i < n; ++i) {s2 += w[i];}
    if (s2 < 1e-6) {for (int i = 0; i < n; ++i) {w[i] = 1.0;}}
  }
  for (int i = 0; i < n; ++i) {sum += w[i];}
  res_.v_body.setZero();
  for (int i = 0; i < n; ++i) {res_.v_body += (w[i] / sum) * res_.v_foot[stance[i]];}
  return res_;
}

}  // namespace g1_wbc
