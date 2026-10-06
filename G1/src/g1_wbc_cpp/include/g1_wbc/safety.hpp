// Cong an toan: dung giua WBC va dong co.
//
// WBC tinh ra mo-men mong muon. Cong nay quyet dinh co cho qua khong, cho qua
// bao nhieu, va khi nao thi cat han. No KHONG biet gi ve DDS hay rt/lowcmd -
// co y nhu vay, de kiem thu duoc ma khong can robot.
//
// NGUYEN TAC
//   1. Mac dinh la KHOA. Phai goi arm() mot cach tuong minh moi mo.
//   2. Loi thi CHOT (latch). Khong tu phuc hoi. Ra khoi loi chi bang reset(),
//      va reset() dua ve trang thai KHOA chu khong phai dang chay - nghia la
//      sau moi loi deu phai tang dan lai tu 0.
//   3. Cat bot (mo-men, toc do bien thien, he so tang dan) KHONG phai la loi -
//      chuyen binh thuong. Loi la nhung dieu kien noi rang "mo hinh hoac phep
//      do dang sai", va luc do mo-men cua ta vo nghia.
//
// MOT DIEU PHAI NOI RO VE "HANH DONG KHI LOI"
//   Mo-men bang 0 tren mot humanoid dang dung KHONG phai la an toan - no do.
//   Cho giam chan (-kd*dq) do cham hon nhung van do. Khong co lua chon nao
//   trong file nay la an toan that su; thu an toan that su la GIAN TREO va nut
//   dung khan do nguoi cam. Cong nay chi lam cho cu do bot bao lieu hon.
#pragma once

#include <string>

#include <Eigen/Core>

namespace g1_wbc
{

enum class SafetyState
{
  Disarmed,   // mac dinh - khong cho gi qua
  Ramping,    // dang tang he so tu 0 len 1
  Active,     // he so = 1
  Fault       // da chot loi, chi reset() moi ra duoc
};

const char * toString(SafetyState s);

enum class FaultAction
{
  ZeroTorque,   // buong han
  Damping       // -kd*dq, do cham hon nhung van do
};

struct SafetyLimits
{
  // --- gioi han mo-men RIENG, chat hon gioi han dong co trong URDF ---
  // Do tren robot that khi BI DAY (Ethernet, 31054 chu ky, CoM lech toi 56.9 mm):
  // WBC doi toi 88.0 Nm o hong (dung bang gioi han dong co - QP bao hoa) va
  // 76.6 Nm o goi, trong khi bo dieu khien cua Unitree dinh 61.8 Nm.
  //
  //   tau_scale 0.25 (cu) -> cat 87.4% chu ky   <- bop nghet, khong giu duoc
  //   tau_scale 0.50      -> cat 15.7%
  //   tau_scale 0.70      -> cat  5.8%
  //
  // Noi rieng tran tuyet doi KHONG giai quyet duoc: 30 hay 80 Nm deu cat ~87%,
  // vi thu dang chan la tau_scale (22 Nm o hong, 12.5 o co chan).
  //
  // PHAI NOI RO: mot tran du chat de that su bao ve thi cung du chat de bo dieu
  // khien khong giu duoc thang bang. Bien an toan that su den tu GIAN TREO, nut
  // dung khan va viec gioi han do manh cu day - khong den tu cho nay.
  // 0.5 / 60 Nm la lua chon cho lan treo gian DAU TIEN, se xem lai sau do.
  double tau_scale = 0.5;
  double tau_abs_max = 60.0;
  // Nm/s. 150 la con so DOAN ban dau va sai hanh: do tren robot that (59 s,
  // 30279 chu ky) phan bo |dtau/dt| cua WBC la p50 417, p90 2172, p99 11395,
  // max 166600 Nm/s. Nguong 150 chan 89% chu ky - khong phai chan dot bien ma
  // la bop nghet bo dieu khien. 4000 Nm/s chan ~5%, dung vai tro "chan dot
  // bien" ma khong can thiep vao hoat dong binh thuong.
  double tau_rate_max = 4000.0;

  double ramp_secs = 3.0;           // thoi gian tang he so 0 -> 1
  FaultAction fault_action = FaultAction::Damping;
  double kd_fault = 2.0;            // Nm/(rad/s) khi fault_action = Damping

  // --- dieu kien CHOT LOI ---
  double cycle_budget_ms = 2.0;
  int cycle_overrun_max = 5;        // so chu ky LIEN TIEP duoc phep vuot
  // Giu 10 ms, CO Y khong noi. Do tren wifi: p50 1.88 ms, p90 2.15, nhung
  // p99 = 10.33 va max 32 ms -> ngat 1.25% so chu ky. Noi len 15 ms thi chi
  // con 1 lan, NHUNG do la lam bo phat hien im lang chu khong sua duoc gi:
  // mot bo dieu khien 500 Hz mu 10-12 ms la mu mat 5-6 chu ky.
  // Ket luan dung la DIEU KHIEN THAT PHAI QUA ETHERNET. De nguyen 10 ms de
  // con so nay tu noi len dieu do.
  double state_timeout_ms = 10.0;
  int qp_fail_max = 3;              // so lan LIEN TIEP QP khong giai duoc
  double tilt_max_rad = 0.35;       // ~20 do so voi phuong thang dung
  double com_err_max = 0.12;        // m
  double dq_max = 12.0;             // rad/s, bat ky khop nao
  double v_body_max = 1.5;          // m/s
  // Dung thang bang HAI CHAN: cong thuc WBC rang buoc ca hai ban chan, nen
  // mot chan roi khoi dat la ra NGOAI mo hinh - khong phai "kho hon mot chut"
  // ma la gia thiet bi vi pham. Do tren robot that: khi bi day, 24.9% thoi
  // gian o giai doan mot chan, va cong se ngat dung 24.4%.
  // Day la gioi han THAT cua bo dieu khien hien nay, khong phai nguong dat sai.
  bool require_both_feet = true;
};

struct SafetyInputs
{
  Eigen::VectorXd dq;               // 29
  Eigen::Vector4d quat{1, 0, 0, 0}; // w,x,y,z
  Eigen::Vector3d v_body{Eigen::Vector3d::Zero()};
  Eigen::Vector3d com_err{Eigen::Vector3d::Zero()};
  bool contact_left = true, contact_right = true;
  bool qp_ok = true;
  double cycle_ms = 0.0;
  double state_age_ms = 0.0;
};

struct SafetyReport
{
  SafetyState state = SafetyState::Disarmed;
  double alpha = 0.0;               // he so tang dan dang ap dung
  std::string fault;                // ly do, rong neu chua loi
  int n_clipped = 0;                // so khop cham tran mo-men o chu ky nay
  int n_rate_limited = 0;           // so khop bi chan toc do bien thien
  double max_clip = 0.0;            // Nm bi cat nhieu nhat
};

class SafetyGate
{
public:
  // effort_limit: gioi han mo-men cua dong co (29), lay tu URDF.
  SafetyGate(const Eigen::VectorXd & effort_limit, const SafetyLimits & lim = {});

  void arm();                       // Disarmed -> Ramping (khong ra duoc tu Fault)
  void disarm();                    // ve Disarmed, he so ve 0
  void reset();                     // cach DUY NHAT ra khoi Fault, ve Disarmed

  // Tra ve mo-men duoc phep gui. dt tinh bang giay.
  const Eigen::VectorXd & filter(
    const Eigen::VectorXd & tau_des, const SafetyInputs & in, double dt);

  const SafetyReport & report() const {return rep_;}
  const Eigen::VectorXd & tauMax() const {return tau_max_;}
  SafetyState state() const {return rep_.state;}
  // Thong ke tich luy - de chay o che do kho roi xem "neu that thi sao".
  long nFault() const {return n_fault_;}
  long nCycles() const {return n_cycles_;}
  long nClippedCycles() const {return n_clipped_cycles_;}
  const std::string & firstFault() const {return first_fault_;}

private:
  void trip(const std::string & why);
  bool checkFaults(const SafetyInputs & in, const Eigen::VectorXd & tau_des);

  SafetyLimits lim_;
  Eigen::VectorXd tau_max_, tau_out_, tau_prev_;
  SafetyReport rep_;
  double t_ramp_ = 0.0;
  int n_overrun_ = 0, n_qp_fail_ = 0;
  long n_fault_ = 0, n_cycles_ = 0, n_clipped_cycles_ = 0;
  std::string first_fault_;
};

}  // namespace g1_wbc
