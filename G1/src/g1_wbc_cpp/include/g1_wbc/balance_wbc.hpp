// Ban C++ cua g1_wbc/wbc.py (BalanceWBC).
//
// KHONG phai viet lai cho dep. La DICH TUNG BUOC MOT, co y giu nguyen thu tu
// goi ham Pinocchio va thu tu cong ma tran cua ban Python, de khi lech thi
// lech do LOI DICH chu khong phai do "minh lam theo cach khac". Toi uu toc do
// de sau - luc do da co test vector giu lung.
//
// Vi sao phai co ban C++: do tren robot that, ban Python p99 3.39 ms, xau nhat
// 13.58 ms, trong khi ngan sach o 500 Hz la 2.0 ms. Khong phai vi Python cham
// mot cach mo ho, ma vi bo thu gom rac dung khong bao truoc.
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody.hpp>
#include <proxsuite/proxqp/dense/dense.hpp>

namespace g1_wbc
{

inline constexpr int kNMotor = 29;
inline constexpr int kNContact = 8;   // 4 diem x 2 ban chan

// Vi tri diem tiep xuc trong he ban chan, lay tu mujoco_menagerie g1.xml.
// Phai TRUNG TUNG SO voi CONTACT_LOCAL trong wbc.py.
inline const std::array<Eigen::Vector3d, 4> kContactLocal = {
  Eigen::Vector3d(-0.05, 0.025, -0.035),
  Eigen::Vector3d(-0.05, -0.025, -0.035),
  Eigen::Vector3d(0.12, 0.030, -0.035),
  Eigen::Vector3d(0.12, -0.030, -0.035)};

inline const std::array<std::string, 2> kFootFrames = {
  "left_ankle_roll_link", "right_ankle_roll_link"};

struct Gains
{
  double mu = 0.6;
  double fz_min = 1.0;
  double kd_contact = 30.0;
  double kp_com = 60.0, kd_com = 15.0;
  double kp_ori = 250.0, kd_ang = 40.0;
  double kp_q = 100.0, kd_q = 20.0;
  double w_com = 60.0, w_ang = 12.0, w_post = 1.0, w_f = 1e-3, w_qacc = 1e-4;
  // Phat mo-men khop: (w_tau/2)||tau||^2. 0 = tat.
  // Dung hai chan la bai toan VO DINH TINH HOC - nhieu cach phan bo luc cho
  // cung mot hop luc nhung mo-men khop khac han. Da TACH duoc tren du lieu
  // that: mo-men hong 27.6 Nm thi 28.8 den tu -Jc^T f, chi 5.5 tu M*qacc va
  // 3.1 tu trong luc. Tuc QP dang chon cach phan bo luc doi mo-men hong lon.
  // Do danh doi (MuJoCo, nguong day / mo-men hong TB tren du lieu that):
  //    w_tau=0 -> 65 N / 26.6 Nm      w_tau=3  -> 58 N
  //    w_tau=1 -> 62 N / 17.5 Nm      w_tau=10 -> 58 N / 12.7 Nm
  //    w_tau=30 -> 55 N
  double w_tau = 1.0;
};

// Tach rieng khoi Gains: day la lua chon ve CACH GIAI, khong phai ve bai toan.
// Doi chieu test vector can chinh xac cao (eps chat, khong khoi dong nong);
// chay that can nhanh (eps long hon, khoi dong nong tu nghiem chu ky truoc).
enum class QpSolver
{
  ProxQp,       // ADMM, co san trong ros-jazzy-proxsuite
  EiQuadProg    // tap tich cuc (Goldfarb-Idnani) - CUNG thuat toan voi
                // quadprog cua ban Python, nen nghiem chinh xac chu khong xap xi
};

struct SolverOpts
{
#ifdef G1_WBC_HAS_EIQUADPROG
  // Do tren 55 tu the that (41 o giai doan mot chan): eiquadprog nhanh hon
  // ProxQP 3.1 lan o phan QP (0.187 so voi 0.585 ms) VA khop voi ban Python den
  // 1.0e-09 thay vi 1.7e-06 - vi no dung chinh thuat toan cua quadprog.
  QpSolver solver = QpSolver::EiQuadProg;
#else
  QpSolver solver = QpSolver::ProxQp;
#endif
  // 1e-9: siet hon nua khong giup gi (do lech chung lai o 1.7e-06 vi thang
  // cua bai toan, xem TOL trong test_vectors.py) ma ton them vong lap.
  double eps_abs = 1e-9;
  bool warm_start = false;
  int max_iter = 20000;
  // ProxSuite mac dinh tu can bang ma tran (Ruiz). Tat di de DO xem no dang
  // lam duoc bao nhieu - neu tat ma khong te hon may thi can bang bang tay
  // cung se khong duoc gi.
  bool precond = true;
};

struct SolveResult
{
  Eigen::VectorXd tau;     // 29
  Eigen::VectorXd qacc;    // nv
  Eigen::VectorXd f;       // 3 * nc
  Eigen::Vector3d com, com_err, ori_err, L;
  double fz_left = 0.0, fz_right = 0.0;
  bool ok = false;
  std::string error;
};

class BalanceWBC
{
public:
  BalanceWBC(const std::string & urdf_path, const std::string & payload_yaml, const Gains & g);

  void setSolverOpts(const SolverOpts & o);

  // Chot tu the / CoM / huong hien tai lam muc tieu.
  void captureReference(const Eigen::VectorXd & q_motor, const Eigen::Vector4d & quat_wxyz);

  SolveResult solve(
    const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
    const Eigen::Vector4d & quat_wxyz, const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & v_body, bool contact_left, bool contact_right);

  int qpIters() const {return last_iters_;}
  // Chia nho thoi gian lan giai gan nhat (ms). Do gop ca ba thi khong biet
  // nen toi uu cho nao - va toi uu nham cho la mat cong vo ich.
  struct Timing {double dyn = 0, build = 0, qp = 0;};
  const Timing & timing() const {return t_;}
  double mass() const {return mass_;}
  int nv() const {return nv_;}
  int nc() const {return nc_;}
  const Gains & gains() const {return gains_;}
  const Eigen::VectorXd & qRef() const {return q_ref_;}
  const Eigen::Vector3d & comDes() const {return com_des_;}
  const Eigen::Vector3d & anchorRef() const {return anchor_ref_;}
  const Eigen::Quaterniond & quatDes() const {return quat_des_;}
  const Eigen::VectorXd & tauLo() const {return tau_lo_;}
  const Eigen::VectorXd & tauHi() const {return tau_hi_;}

private:
  void fill(
    SolveResult & r, const Eigen::VectorXd & z, const Eigen::Vector3d & com,
    const Eigen::Vector3d & ori_err, const Eigen::Vector3d & L,
    const Eigen::VectorXd & h_a) const;

  void assemble(
    const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
    const Eigen::Quaterniond & qq, const Eigen::Vector3d & gyro,
    const Eigen::Vector3d & v_body);

  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<pinocchio::FrameIndex> contact_fids_, foot_fids_;
  Gains gains_;
  int nv_ = 0, nc_ = 0, nf_ = 0, nz_ = 0, n_eq_ = 0, n_in_ = 0;
  double mass_ = 0.0;
  Eigen::Vector3d gravity_{0.0, 0.0, -9.81};
  Eigen::VectorXd tau_lo_, tau_hi_;

  bool ref_done_ = false;
  bool qp_inited_ = false;
  SolverOpts opts_;
  int last_iters_ = 0;
  Timing t_;
  Eigen::VectorXd q_ref_;
  Eigen::Vector3d com_des_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d anchor_ref_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond quat_des_{Eigen::Quaterniond::Identity()};

  Eigen::VectorXd q_, v_;
  // vung nho cap phat san - khong cap phat trong vong dieu khien
  Eigen::MatrixXd H_, A_eq_, C_in_, Jc_, Jf_, M_, A_lin_, A_ang_, A_post_, T_;
  Eigen::VectorXd g_, b_eq_, l_in_, u_in_, h_, drift_f_, f_nom_;
  Eigen::MatrixXd pts_;
  // QP phai biet kich thuoc luc tao, ma kich thuoc lai phu thuoc model doc tu
  // URDF -> khong dat duoc trong danh sach khoi tao, phai dung con tro.
  std::unique_ptr<proxsuite::proxqp::dense::QP<double>> qp_;
  // Dang rang buoc MOT PHIA cho eiquadprog: CI z + ci0 >= 0, CE z + ce0 = 0.
  // ProxQP nhan duoc hai phia nen it hang hon; eiquadprog thi khong.
  int n_in_eiq_ = 0;
  Eigen::MatrixXd Ci_, Ce_;
  Eigen::VectorXd ci0_, ce0_, x_eiq_;
  void * eiq_ = nullptr;   // eiquadprog::solvers::EiquadprogFast* (an di de
                           // header nay khong phu thuoc eiquadprog)
};

}  // namespace g1_wbc
