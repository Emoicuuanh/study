// Ban C++ cua g1_leg_odometry/leg_odometry.py.
//
// v_B = -(w_B x p_BF + J_BF qdot_leg)   - KHONG co ma tran quay o ve phai, nen
// sai so huong cua IMU khong lot vao phep do nay.
//
// Hai gioi han giu nguyen tu ban Python:
//   1. Goi duoi thang la diem ky di (cond ~2e6) -> luon dung goi chung.
//   2. Uoc luong luc BO QUA quan tinh M*qddot -> chi dung o che do tua tinh.
//      Van toc KHONG bi anh huong (phep chieu thuan, khong nghich dao gi).
#pragma once

#include <array>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody.hpp>

namespace g1_wbc
{

struct OdomResult
{
  Eigen::Vector3d v_body{Eigen::Vector3d::Zero()};
  std::array<Eigen::Vector3d, 2> v_foot;     // 0 = trai, 1 = phai
  std::array<double, 2> fz{{0.0, 0.0}};
  std::array<Eigen::Matrix<double, 6, 1>, 2> wrench;
  std::array<bool, 2> contact{{true, true}};
  int n_contact = 2;
};

class LegOdometry
{
public:
  // fusion: true = trong so theo fz ("force"), false = trung binh cong ("mean").
  // "force" la GIA THUYET tu du lieu robot that, chua chung minh; trong mo phong
  // "mean" tot hon (RMS 2.64 so voi 2.95 mm/s). Giu dung mac dinh cua ban Python.
  explicit LegOdometry(
    const std::string & urdf_path, double dls_lambda = 1e-3,
    double contact_on = 60.0, double contact_off = 30.0, bool fusion_force = true);

  const OdomResult & update(
    const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
    const Eigen::Vector3d & gyro, const Eigen::VectorXd & tau_est,
    const Eigen::Vector4d & quat_wxyz);

  // Tinh lai KHONG dung trang thai tre nguong - de doi chieu tung khung mot
  // voi ban Python (tre nguong phu thuoc ca lich su nen khong doi chieu duoc).
  void perFrame(
    const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
    const Eigen::Vector3d & gyro, const Eigen::VectorXd & tau_est,
    const Eigen::Vector4d & quat_wxyz, OdomResult & out);

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::array<pinocchio::FrameIndex, 2> fids_;
  double dls_lambda_, contact_on_, contact_off_;
  bool fusion_force_;
  std::array<bool, 2> in_contact_{{true, true}};
  Eigen::VectorXd q_;
  OdomResult res_;
};

}  // namespace g1_wbc
