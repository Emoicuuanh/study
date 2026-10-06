#include "g1_wbc/balance_wbc.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/spatial.hpp>
#include <yaml-cpp/yaml.h>

#ifdef G1_WBC_HAS_EIQUADPROG
// LGPL-3: lien ket DONG (eiquadprog la thu vien chia se rieng), khong chep
// nguon vao repo nay.
#include <eiquadprog/eiquadprog-fast.hpp>
using Eiq = eiquadprog::solvers::EiquadprogFast;
#endif

namespace g1_wbc
{

namespace
{
constexpr double kInf = std::numeric_limits<double>::infinity();

// Ban dich cua g1_leg_odometry/payload.py. Phai giong tung buoc: quan tinh
// cau ban kinh 0.05 m (de ma tran quan tinh khong suy bien ve mat so), roi
// appendBodyToJoint qua he cua khop cha.
void applyPayload(pinocchio::Model & model, const std::string & yaml_path)
{
  YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root["payload"]) {return;}
  for (const auto & it : root["payload"]) {
    const auto frame = it["frame"].as<std::string>();
    if (!model.existFrame(frame)) {
      throw std::runtime_error("payload: khong co frame '" + frame + "'");
    }
    const auto fid = model.getFrameId(frame);
    const pinocchio::Frame & fr = model.frames[fid];
    const double m = it["mass"].as<double>();
    Eigen::Vector3d lever = Eigen::Vector3d::Zero();
    if (it["xyz"]) {
      for (int i = 0; i < 3; ++i) {lever[i] = it["xyz"][i].as<double>();}
    }
    const auto Ic = pinocchio::Inertia::FromSphere(m, 0.05).inertia();
    model.appendBodyToJoint(fr.parentJoint, pinocchio::Inertia(m, lever, Ic), fr.placement);
  }
}
}  // namespace

BalanceWBC::BalanceWBC(
  const std::string & urdf_path, const std::string & payload_yaml, const Gains & g)
: gains_(g)
{
  pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);
  if (!payload_yaml.empty()) {applyPayload(model_, payload_yaml);}

  // Them 8 frame diem tiep xuc -> Pinocchio tu tinh Jacobian va so hang troi.
  // PHAI lam sau payload, truoc createData - dung thu tu nhu ban Python.
  for (const auto & fname : kFootFrames) {
    const auto fid = model_.getFrameId(fname);
    const pinocchio::Frame fr = model_.frames[fid];
    for (size_t k = 0; k < kContactLocal.size(); ++k) {
      pinocchio::Frame nf(
        fname + "_contact" + std::to_string(k), fr.parentJoint, fid,
        fr.placement * pinocchio::SE3(Eigen::Matrix3d::Identity(), kContactLocal[k]),
        pinocchio::OP_FRAME);
      contact_fids_.push_back(model_.addFrame(nf));
    }
  }
  for (const auto & fname : kFootFrames) {foot_fids_.push_back(model_.getFrameId(fname));}
  data_ = pinocchio::Data(model_);

  nv_ = model_.nv;
  nc_ = static_cast<int>(contact_fids_.size());
  nf_ = 3 * nc_;
  nz_ = nv_ + nf_;
  mass_ = 0.0;
  for (const auto & I : model_.inertias) {mass_ += I.mass();}

  tau_hi_ = model_.effortLimit.segment(6, kNMotor);
  tau_lo_ = -tau_hi_;

  // Rang buoc dang thuc: 6 hang than noi + 6 hang moi ban chan.
  n_eq_ = 6 + 12;
  // Bat dang thuc: moi diem tiep xuc 1 hang fz (hai phia) + 4 hang non ma sat,
  // roi 29 hang gioi han mo-men (hai phia).
  //
  // Ban Python them/bot hang tuy theo chan co cham dat khong. O day so hang
  // CO DINH, chan bay thi doi CAN cua hang fz thanh [0, 1e-6]. Tuong duong ve
  // toan hoc, nhung kich thuoc QP khong doi -> khong cap phat lai moi chu ky.
  n_in_ = nc_ * 5 + kNMotor;

  q_ = pinocchio::neutral(model_);
  v_ = Eigen::VectorXd::Zero(nv_);
  q_ref_ = Eigen::VectorXd::Zero(kNMotor);

  H_ = Eigen::MatrixXd::Zero(nz_, nz_);
  g_ = Eigen::VectorXd::Zero(nz_);
  A_eq_ = Eigen::MatrixXd::Zero(n_eq_, nz_);
  b_eq_ = Eigen::VectorXd::Zero(n_eq_);
  C_in_ = Eigen::MatrixXd::Zero(n_in_, nz_);
  l_in_ = Eigen::VectorXd::Zero(n_in_);
  u_in_ = Eigen::VectorXd::Zero(n_in_);
  Jc_ = Eigen::MatrixXd::Zero(nf_, nv_);
  Jf_ = Eigen::MatrixXd::Zero(12, nv_);
  M_ = Eigen::MatrixXd::Zero(nv_, nv_);
  A_lin_ = Eigen::MatrixXd::Zero(3, nz_);
  A_ang_ = Eigen::MatrixXd::Zero(3, nz_);
  A_post_ = Eigen::MatrixXd::Zero(kNMotor, nz_);
  T_ = Eigen::MatrixXd::Zero(kNMotor, nz_);
  h_ = Eigen::VectorXd::Zero(nv_);
  drift_f_ = Eigen::VectorXd::Zero(12);
  f_nom_ = Eigen::VectorXd::Zero(nf_);
  pts_ = Eigen::MatrixXd::Zero(nc_, 3);

  // A_post va phan hang so cua A_lin khong doi theo thoi gian -> dat mot lan.
  A_post_.block(0, 6, kNMotor, kNMotor).setIdentity();
  for (int i = 0; i < nc_; ++i) {
    A_lin_.block<3, 3>(0, nv_ + 3 * i).setIdentity();
  }

  // Dang mot phia: moi diem 1 hang fz duoi + 1 hang fz tren + 4 hang ma sat,
  // roi 29 x 2 hang gioi han mo-men.
  n_in_eiq_ = nc_ * 6 + 2 * kNMotor;
  Ci_ = Eigen::MatrixXd::Zero(n_in_eiq_, nz_);
  ci0_ = Eigen::VectorXd::Zero(n_in_eiq_);
  Ce_ = Eigen::MatrixXd::Zero(n_eq_, nz_);
  ce0_ = Eigen::VectorXd::Zero(n_eq_);
  x_eiq_ = Eigen::VectorXd::Zero(nz_);
#ifdef G1_WBC_HAS_EIQUADPROG
  auto * e = new Eiq();
  e->reset(static_cast<size_t>(nz_), static_cast<size_t>(n_eq_),
    static_cast<size_t>(n_in_eiq_));
  eiq_ = e;
#endif

  qp_ = std::make_unique<proxsuite::proxqp::dense::QP<double>>(nz_, n_eq_, n_in_);
  // Chat hon mac dinh nhieu. Bai toan chi 59 bien, va muc dich la doi chieu
  // duoc voi ban Python (quadprog la phuong phap tap tich cuc, nghiem chinh
  // xac); de mac dinh 1e-9 thi lech con nam o muc sai so giai chu khong phai
  // sai so dich, va nhu the test vector mat y nghia.
  qp_->settings.eps_rel = 0.0;
  qp_->settings.verbose = false;
  setSolverOpts(opts_);
}

void BalanceWBC::setSolverOpts(const SolverOpts & o)
{
  opts_ = o;
  qp_->settings.eps_abs = o.eps_abs;
  qp_->settings.max_iter = o.max_iter;
  // NO_INITIAL_GUESS = moi lan giai cho ra dung mot ket qua voi cung dau vao,
  // khong phu thuoc thu tu cac vector - can cho doi chieu test vector.
  qp_->settings.initial_guess =
    o.warm_start ? proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT
    : proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
}

void BalanceWBC::assemble(
  const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
  const Eigen::Quaterniond & qq, const Eigen::Vector3d & gyro,
  const Eigen::Vector3d & v_body)
{
  q_.head<3>().setZero();
  q_.segment<4>(3) = qq.coeffs();      // pinocchio dung (x,y,z,w)
  q_.tail(kNMotor) = q_motor;
  v_.head<3>() = v_body;               // free-flyer: LOCAL = he than
  v_.segment<3>(3) = gyro;
  v_.tail(kNMotor) = dq_motor;
}

void BalanceWBC::captureReference(
  const Eigen::VectorXd & q_motor, const Eigen::Vector4d & quat_wxyz)
{
  const Eigen::Quaterniond qq =
    Eigen::Quaterniond(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]).normalized();
  assemble(q_motor, Eigen::VectorXd::Zero(kNMotor), qq, Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero());
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::centerOfMass(model_, data_, q_);
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int i = 0; i < nc_; ++i) {mean += data_.oMf[contact_fids_[i]].translation();}
  anchor_ref_ = mean / static_cast<double>(nc_);
  q_ref_ = q_motor;
  com_des_ = data_.com[0];
  quat_des_ = qq;
  ref_done_ = true;
}

SolveResult BalanceWBC::solve(
  const Eigen::VectorXd & q_motor, const Eigen::VectorXd & dq_motor,
  const Eigen::Vector4d & quat_wxyz, const Eigen::Vector3d & gyro,
  const Eigen::Vector3d & v_body, bool contact_left, bool contact_right)
{
  SolveResult r;
  if (!ref_done_) {
    r.error = "phai goi captureReference() truoc";
    return r;
  }
  const Eigen::Quaterniond qq =
    Eigen::Quaterniond(quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]).normalized();
  assemble(q_motor, dq_motor, qq, gyro, v_body);

  const auto t0 = std::chrono::steady_clock::now();
  // THU TU GOI giu y nhu ban Python: cac ham nay co tac dung phu len data_,
  // doi thu tu la doi ket qua.
  pinocchio::computeAllTerms(model_, data_, q_, v_);
  pinocchio::forwardKinematics(model_, data_, q_, v_, Eigen::VectorXd::Zero(nv_));
  pinocchio::updateFramePlacements(model_, data_);

  // computeAllTerms chi dien nua tren cua M.
  M_ = data_.M;
  M_.triangularView<Eigen::StrictlyLower>() = M_.transpose().triangularView<Eigen::StrictlyLower>();
  h_ = data_.nle;
  Eigen::Vector3d com = data_.com[0];
  const Eigen::Vector3d vcom = data_.vcom[0];
  const Eigen::Vector3d L = pinocchio::computeCentroidalMomentum(model_, data_, q_, v_).angular();

  const auto t_dyn = std::chrono::steady_clock::now();
  Eigen::MatrixXd Jtmp(6, nv_);
  for (int i = 0; i < nc_; ++i) {
    pts_.row(i) = data_.oMf[contact_fids_[i]].translation().transpose();
    Jtmp.setZero();
    pinocchio::getFrameJacobian(
      model_, data_, contact_fids_[i], pinocchio::LOCAL_WORLD_ALIGNED, Jtmp);
    Jc_.block(3 * i, 0, 3, nv_) = Jtmp.topRows<3>();
  }
  for (int i = 0; i < 2; ++i) {
    Jtmp.setZero();
    pinocchio::getFrameJacobian(
      model_, data_, foot_fids_[i], pinocchio::LOCAL_WORLD_ALIGNED, Jtmp);
    Jf_.block(6 * i, 0, 6, nv_) = Jtmp;
    const auto a = pinocchio::getFrameClassicalAcceleration(
      model_, data_, foot_fids_[i], pinocchio::LOCAL_WORLD_ALIGNED);
    drift_f_.segment<3>(6 * i) = a.linear();
    drift_f_.segment<3>(6 * i + 3) = a.angular();
  }

  // --- NEO VAO BAN CHAN ---
  // q[:3]=0 nen moi vi tri deu tinh trong he gan voi than. Dich tat ca sao cho
  // trung binh diem tiep xuc trung voi luc chot moc. Neu bo buoc nay thi tac vu
  // CoM khong thay sai lech khi robot nghieng quanh co chan - da tung lam robot
  // do sau 3.8 s trong MuJoCo du moi dai luong dong luc hoc deu khop den 1e-5.
  const Eigen::Vector3d offset = anchor_ref_ - pts_.colwise().mean().transpose();
  pts_.rowwise() += offset.transpose();
  com += offset;

  // tau = T z + h_a. Tinh som vi CA ham muc tieu lan rang buoc deu dung.
  T_.leftCols(nv_) = M_.bottomRows(kNMotor);
  T_.rightCols(nf_) = -Jc_.transpose().bottomRows(kNMotor);
  const Eigen::VectorXd h_a = h_.tail(kNMotor);

  // ================= ham muc tieu =================
  H_.setZero();
  g_.setZero();

  const Eigen::Vector3d a_com_des =
    gains_.kp_com * (com_des_ - com) - gains_.kd_com * vcom;
  const Eigen::Vector3d b_lin = mass_ * (a_com_des - gravity_);
  H_.noalias() += gains_.w_com * A_lin_.transpose() * A_lin_;
  g_.noalias() += gains_.w_com * A_lin_.transpose() * b_lin;

  const Eigen::Vector3d ori_err = pinocchio::log3((quat_des_ * qq.inverse()).toRotationMatrix());
  A_ang_.setZero();
  for (int i = 0; i < nc_; ++i) {
    A_ang_.block<3, 3>(0, nv_ + 3 * i) =
      pinocchio::skew(Eigen::Vector3d(pts_.row(i).transpose() - com));
  }
  const Eigen::Vector3d b_ang = gains_.kp_ori * ori_err - gains_.kd_ang * L;
  H_.noalias() += gains_.w_ang * A_ang_.transpose() * A_ang_;
  g_.noalias() += gains_.w_ang * A_ang_.transpose() * b_ang;

  const Eigen::VectorXd b_post =
    gains_.kp_q * (q_ref_ - q_motor) - gains_.kd_q * dq_motor;
  H_.noalias() += gains_.w_post * A_post_.transpose() * A_post_;
  g_.noalias() += gains_.w_post * A_post_.transpose() * b_post;

  if (gains_.w_tau > 0.0) {
    // (w/2)||T z + h_a||^2  ->  H += w T'T ,  g += w T'(-h_a)
    H_.noalias() += gains_.w_tau * T_.transpose() * T_;
    g_.noalias() -= gains_.w_tau * T_.transpose() * h_a;
  }

  for (int i = 0; i < nc_; ++i) {
    f_nom_.segment<3>(3 * i) << 0.0, 0.0, mass_ * 9.81 / static_cast<double>(nc_);
  }
  H_.bottomRightCorner(nf_, nf_).diagonal().array() += gains_.w_f;
  g_.tail(nf_).noalias() += gains_.w_f * f_nom_;
  H_.topLeftCorner(nv_, nv_).diagonal().array() += gains_.w_qacc;
  H_.diagonal().array() += 1e-8;

  // ================= rang buoc dang thuc =================
  // (1) 6 hang dau khong co mo-men truyen dong: M[0:6] qacc + h[0:6] = Jc^T[0:6] f
  A_eq_.setZero();
  A_eq_.block(0, 0, 6, nv_) = M_.topRows<6>();
  A_eq_.block(0, nv_, 6, nf_) = -Jc_.transpose().topRows<6>();
  b_eq_.head<6>() = -h_.head<6>();
  // (2) ban chan khong truot, 6D MOI BAN CHAN (khong phai 3D moi diem: 4 diem
  //     tren mot ban chan cung chi cho hang 6)
  A_eq_.block(6, 0, 12, nv_) = Jf_;
  b_eq_.segment<12>(6) = -drift_f_ - gains_.kd_contact * (Jf_ * v_);

  // ================= bat dang thuc: l <= C z <= u =================
  C_in_.setZero();
  int row = 0;
  for (int i = 0; i < nc_; ++i) {
    const int s = nv_ + 3 * i;
    const bool on = (i < 4) ? contact_left : contact_right;
    C_in_(row, s + 2) = 1.0;                       // fz
    l_in_[row] = on ? gains_.fz_min : 0.0;
    u_in_[row] = on ? kInf : 1e-6;                 // chan bay: ep fz ve 0
    ++row;
    // non ma sat tuyen tinh hoa: mu*fz +- fx >= 0, mu*fz +- fy >= 0.
    // Tu dong ep CoP nam trong da giac do vi 8 diem chi day duoc.
    for (int k = 0; k < 2; ++k) {
      for (double sgn : {1.0, -1.0}) {
        C_in_(row, s + 2) = gains_.mu;
        C_in_(row, s + k) = sgn;
        l_in_[row] = 0.0;
        u_in_[row] = kInf;
        ++row;
      }
    }
  }
  // gioi han mo-men: tau = T z + h_a
  C_in_.block(row, 0, kNMotor, nz_) = T_;
  l_in_.segment(row, kNMotor) = tau_lo_ - h_a;
  u_in_.segment(row, kNMotor) = tau_hi_ - h_a;
  row += kNMotor;

  const auto t_build = std::chrono::steady_clock::now();
  using ms = std::chrono::duration<double, std::milli>;
  // ================= giai =================
  // ProxSuite toi thieu 0.5 z'Hz + g'z; ban Python (quadprog) toi thieu
  // 0.5 z'Hz - a'z. Nen dau cua g phai dao.
  if (opts_.solver == QpSolver::EiQuadProg) {
#ifdef G1_WBC_HAS_EIQUADPROG
    // CE z + ce0 = 0 ; CI z + ci0 >= 0  (dau nguoc voi dang "C z >= b")
    Ce_ = A_eq_;
    ce0_ = -b_eq_;
    int er = 0;
    for (int i = 0; i < nc_; ++i) {
      const int s = nv_ + 3 * i;
      const bool on = (i < 4) ? contact_left : contact_right;
      Ci_.row(er).setZero(); Ci_(er, s + 2) = 1.0;
      ci0_[er] = -(on ? gains_.fz_min : 0.0); ++er;                 // fz >= can duoi
      Ci_.row(er).setZero(); Ci_(er, s + 2) = -1.0;
      // Chan dang cham dat thi hang tren vo hieu (1e9), khong phai bo hang -
      // giu so hang co dinh de khong cap phat lai moi chu ky.
      ci0_[er] = on ? 1e9 : 1e-6; ++er;
      for (int k = 0; k < 2; ++k) {
        for (double sgn : {1.0, -1.0}) {
          Ci_.row(er).setZero();
          Ci_(er, s + 2) = gains_.mu;
          Ci_(er, s + k) = sgn;
          ci0_[er] = 0.0; ++er;
        }
      }
    }
    Ci_.block(er, 0, kNMotor, nz_) = T_;
    ci0_.segment(er, kNMotor) = -(tau_lo_ - h_a);
    er += kNMotor;
    Ci_.block(er, 0, kNMotor, nz_) = -T_;
    ci0_.segment(er, kNMotor) = tau_hi_ - h_a;
    er += kNMotor;

    auto * e = static_cast<Eiq *>(eiq_);
    const auto st = e->solve_quadprog(H_, -g_, Ce_, ce0_, Ci_, ci0_, x_eiq_);
    last_iters_ = e->getIteratios();
    const auto t_qp2 = std::chrono::steady_clock::now();
    t_.dyn = ms(t_dyn - t0).count();
    t_.build = ms(t_build - t_dyn).count();
    t_.qp = ms(t_qp2 - t_build).count();
    if (st != eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
      r.error = "eiquadprog khong giai duoc (status " +
        std::to_string(static_cast<int>(st)) + ")";
      return r;
    }
    fill(r, x_eiq_, com, ori_err, L, h_a);
    return r;
#else
    r.error = "ban build nay khong co eiquadprog";
    return r;
#endif
  }

  if (!qp_inited_) {
    qp_->init(H_, -g_, A_eq_, b_eq_, C_in_, l_in_, u_in_, opts_.precond);
    qp_inited_ = true;
  } else {
    qp_->update(H_, -g_, A_eq_, b_eq_, C_in_, l_in_, u_in_);
  }
  qp_->solve();
  const auto t_qp = std::chrono::steady_clock::now();
  t_.dyn = ms(t_dyn - t0).count();
  t_.build = ms(t_build - t_dyn).count();
  t_.qp = ms(t_qp - t_build).count();
  const auto status = qp_->results.info.status;
  if (status != proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
    r.error = "ProxQP khong giai duoc (status " + std::to_string(static_cast<int>(status)) + ")";
    return r;
  }
  last_iters_ = static_cast<int>(qp_->results.info.iter);
  fill(r, qp_->results.x, com, ori_err, L, h_a);
  return r;
}

// Dien ket qua tu nghiem z. Dung chung cho ca hai bo giai de khong co kha nang
// hai duong di cho ra hai cach tinh tau khac nhau.
void BalanceWBC::fill(
  SolveResult & r, const Eigen::VectorXd & z, const Eigen::Vector3d & com,
  const Eigen::Vector3d & ori_err, const Eigen::Vector3d & L,
  const Eigen::VectorXd & h_a) const
{
  r.qacc = z.head(nv_);
  r.f = z.tail(nf_);
  r.tau = M_.bottomRows(kNMotor) * r.qacc + h_a - Jc_.transpose().bottomRows(kNMotor) * r.f;
  r.com = com;
  r.com_err = com - com_des_;
  r.ori_err = ori_err;
  r.L = L;
  r.fz_left = 0.0;
  r.fz_right = 0.0;
  for (int i = 0; i < 4; ++i) {r.fz_left += r.f[3 * i + 2];}
  for (int i = 4; i < nc_; ++i) {r.fz_right += r.f[3 * i + 2];}
  r.ok = true;
}

}  // namespace g1_wbc
