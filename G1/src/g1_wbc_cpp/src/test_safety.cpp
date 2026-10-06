// Kiem tra cong an toan. Khong can robot, khong can ROS, khong can mang.
//
// Cong an toan la thu duy nhat trong he nay ma loi cua no KHONG duoc phep xay
// ra: moi thu khac sai thi cung chi la so lieu xau, con cai nay sai thi mo-men
// sai di thang vao dong co. Nen moi luat deu co mot phep thu rieng, va co ca
// phep thu cho dieu NGUOC LAI (vd: da chot loi thi arm() phai khong lam gi).
#include <cmath>
#include <iomanip>
#include <iostream>

#include "g1_wbc/safety.hpp"

using g1_wbc::SafetyGate;
using g1_wbc::SafetyInputs;
using g1_wbc::SafetyLimits;
using g1_wbc::SafetyState;

namespace
{
int g_fail = 0, g_run = 0;

void check(bool ok, const std::string & what)
{
  ++g_run;
  if (!ok) {
    ++g_fail;
    std::cout << "  FAIL: " << what << "\n";
  }
}

constexpr int kN = 29;
const double kDt = 0.002;

Eigen::VectorXd limits() {return Eigen::VectorXd::Constant(kN, 100.0);}

SafetyInputs goodInput()
{
  SafetyInputs in;
  in.dq = Eigen::VectorXd::Zero(kN);
  in.quat = Eigen::Vector4d(1, 0, 0, 0);
  in.cycle_ms = 0.8;
  in.state_age_ms = 1.0;
  return in;
}

// Chay n chu ky voi cung dau vao, tra ve mo-men chu ky cuoi.
Eigen::VectorXd run(SafetyGate & g, const Eigen::VectorXd & des, SafetyInputs in, int n)
{
  Eigen::VectorXd out;
  for (int i = 0; i < n; ++i) {out = g.filter(des, in, kDt);}
  return out;
}

void section(const char * s) {std::cout << "\n-- " << s << "\n";}
}  // namespace

int main()
{
  const Eigen::VectorXd big = Eigen::VectorXd::Constant(kN, 500.0);

  section("mac dinh la KHOA");
  {
    SafetyGate g(limits(), {});
    const auto out = run(g, big, goodInput(), 10);
    check(g.state() == SafetyState::Disarmed, "chua arm() thi phai o trang thai KHOA");
    check(out.cwiseAbs().maxCoeff() == 0.0, "chua arm() thi mo-men phai bang 0");
  }

  section("tang dan he so tu 0");
  {
    SafetyLimits L; L.ramp_secs = 1.0; L.tau_scale = 1.0; L.tau_abs_max = 1e9;
    L.tau_rate_max = 1e9;
    SafetyGate g(limits(), L);
    g.arm();
    const Eigen::VectorXd des = Eigen::VectorXd::Constant(kN, 10.0);
    run(g, des, goodInput(), 1);
    check(g.report().alpha > 0.0 && g.report().alpha < 0.01, "chu ky dau he so phai gan 0");
    run(g, des, goodInput(), 249);                   // tong 250 chu ky = 0.5 s
    check(std::abs(g.report().alpha - 0.5) < 0.02, "nua thoi gian thi he so ~0.5");
    check(g.state() == SafetyState::Ramping, "chua het thoi gian thi van DANG TANG");
    run(g, des, goodInput(), 300);
    check(g.state() == SafetyState::Active, "het thoi gian thi chuyen DANG CHAY");
    check(std::abs(g.report().alpha - 1.0) < 1e-12, "he so phai bang 1");
  }

  section("tran mo-men rieng");
  {
    SafetyLimits L; L.ramp_secs = 0.0; L.tau_scale = 0.25; L.tau_abs_max = 1e9;
    L.tau_rate_max = 1e9;
    SafetyGate g(limits(), L);
    g.arm();
    const auto out = run(g, big, goodInput(), 5);
    check(std::abs(out[0] - 25.0) < 1e-9, "100 Nm x 0.25 -> cat o 25 Nm");
    check(g.report().n_clipped == kN, "tat ca khop deu phai bao bi cat");
  }
  {
    SafetyLimits L; L.ramp_secs = 0.0; L.tau_scale = 1.0; L.tau_abs_max = 12.0;
    L.tau_rate_max = 1e9;
    SafetyGate g(limits(), L);
    g.arm();
    const auto out = run(g, big, goodInput(), 5);
    check(std::abs(out[0] - 12.0) < 1e-9, "tran tuyet doi phai duoc ap dung");
  }

  section("chan toc do bien thien");
  {
    SafetyLimits L; L.ramp_secs = 0.0; L.tau_scale = 1.0; L.tau_abs_max = 1e9;
    L.tau_rate_max = 100.0;                          // 100 Nm/s -> 0.2 Nm moi chu ky
    SafetyGate g(limits(), L);
    g.arm();
    const Eigen::VectorXd des = Eigen::VectorXd::Constant(kN, 50.0);
    const auto o1 = g.filter(des, goodInput(), kDt);
    check(std::abs(o1[0] - 0.2) < 1e-9, "chu ky dau chi duoc tang 0.2 Nm");
    const auto o2 = g.filter(des, goodInput(), kDt);
    check(std::abs(o2[0] - 0.4) < 1e-9, "chu ky hai la 0.4 Nm");
    check(g.report().n_rate_limited == kN, "phai bao da chan toc do bien thien");
  }

  section("tung dieu kien chot loi");
  {
    struct Case {const char * ten; void (* set)(SafetyInputs &);};
    const Case cases[] = {
      {"mat lowstate", [](SafetyInputs & i) {i.state_age_ms = 50.0;}},
      {"NaN trong dq", [](SafetyInputs & i) {i.dq[3] = std::nan("");}},
      {"quaternion lech chuan", [](SafetyInputs & i) {i.quat = Eigen::Vector4d(1.3, 0, 0, 0);}},
      {"nghieng qua", [](SafetyInputs & i) {
         i.quat = Eigen::Vector4d(std::cos(0.3), std::sin(0.3), 0, 0);}},   // 0.6 rad
      {"CoM lech qua", [](SafetyInputs & i) {i.com_err = Eigen::Vector3d(0.3, 0, 0);}},
      {"van toc khop qua", [](SafetyInputs & i) {i.dq[7] = 30.0;}},
      {"van toc than qua", [](SafetyInputs & i) {i.v_body = Eigen::Vector3d(3, 0, 0);}},
      {"mat tiep xuc chan", [](SafetyInputs & i) {i.contact_left = false;}},
    };
    for (const auto & c : cases) {
      SafetyLimits L; L.ramp_secs = 0.0;
      SafetyGate g(limits(), L);
      g.arm();
      SafetyInputs in = goodInput();
      c.set(in);
      g.filter(Eigen::VectorXd::Constant(kN, 5.0), in, kDt);
      check(g.state() == SafetyState::Fault, std::string("phai chot loi khi: ") + c.ten);
    }
  }
  {   // QP hong va tre chu ky phai can NHIEU LAN LIEN TIEP moi chot
    SafetyLimits L; L.ramp_secs = 0.0; L.qp_fail_max = 3;
    SafetyGate g(limits(), L);
    g.arm();
    SafetyInputs bad = goodInput(); bad.qp_ok = false;
    const Eigen::VectorXd des = Eigen::VectorXd::Constant(kN, 5.0);
    g.filter(des, bad, kDt);
    g.filter(des, bad, kDt);
    check(g.state() != SafetyState::Fault, "QP hong 2 lan thi CHUA duoc chot loi");
    g.filter(des, goodInput(), kDt);              // mot lan tot -> dem ve 0
    g.filter(des, bad, kDt);
    g.filter(des, bad, kDt);
    check(g.state() != SafetyState::Fault, "mot chu ky tot phai dat lai bo dem");
    g.filter(des, bad, kDt);
    check(g.state() == SafetyState::Fault, "QP hong 3 lan lien tiep thi phai chot loi");
  }
  {
    SafetyLimits L; L.ramp_secs = 0.0; L.cycle_overrun_max = 5; L.cycle_budget_ms = 2.0;
    SafetyGate g(limits(), L);
    g.arm();
    SafetyInputs slow = goodInput(); slow.cycle_ms = 3.0;
    const Eigen::VectorXd des = Eigen::VectorXd::Constant(kN, 5.0);
    for (int i = 0; i < 4; ++i) {g.filter(des, slow, kDt);}
    check(g.state() != SafetyState::Fault, "tre 4 chu ky thi CHUA chot");
    g.filter(des, slow, kDt);
    check(g.state() == SafetyState::Fault, "tre 5 chu ky lien tiep thi phai chot");
  }

  section("loi la CHOT, khong tu phuc hoi");
  {
    // ramp_secs > 0 o day la CO Y: phep thu cuoi cung kiem tra rang sau reset
    // thi phai tang dan LAI TU DAU, ma dieu do chi quan sat duoc khi co giai
    // doan tang. (Dat 0 thi cong nhay thang sang DANG CHAY - dung nhu thiet ke,
    // va lan dau viet bai thu nay minh da doi nham.)
    SafetyLimits L; L.ramp_secs = 1.0;
    SafetyGate g(limits(), L);
    g.arm();
    SafetyInputs bad = goodInput(); bad.com_err = Eigen::Vector3d(0.3, 0, 0);
    g.filter(Eigen::VectorXd::Constant(kN, 5.0), bad, kDt);
    check(g.state() == SafetyState::Fault, "da chot loi");
    const std::string why = g.report().fault;
    run(g, Eigen::VectorXd::Constant(kN, 5.0), goodInput(), 100);
    check(g.state() == SafetyState::Fault, "dau vao tot tro lai cung KHONG duoc tu phuc hoi");
    check(g.report().fault == why, "ly do loi phai duoc giu nguyen");
    g.arm();
    check(g.state() == SafetyState::Fault, "arm() khi dang loi phai khong lam gi");
    g.reset();
    check(g.state() == SafetyState::Disarmed, "reset() phai ve KHOA, khong phai DANG CHAY");
    g.arm();
    g.filter(Eigen::VectorXd::Constant(kN, 5.0), goodInput(), kDt);
    check(g.state() == SafetyState::Ramping, "sau reset phai tang dan lai tu dau");
  }

  section("hanh dong khi loi");
  {
    SafetyLimits L; L.ramp_secs = 0.0; L.fault_action = g1_wbc::FaultAction::Damping;
    L.kd_fault = 2.0; L.tau_scale = 0.25;
    SafetyGate g(limits(), L);
    g.arm();
    SafetyInputs in = goodInput();
    in.com_err = Eigen::Vector3d(0.3, 0, 0);
    in.dq = Eigen::VectorXd::Constant(kN, 3.0);
    const auto out = g.filter(Eigen::VectorXd::Constant(kN, 50.0), in, kDt);
    check(std::abs(out[0] + 6.0) < 1e-9, "giam chan phai la -kd*dq = -6 Nm");
    in.dq = Eigen::VectorXd::Constant(kN, 100.0);
    const auto out2 = g.filter(Eigen::VectorXd::Constant(kN, 50.0), in, kDt);
    check(std::abs(out2[0] + 25.0) < 1e-9, "giam chan cung phai nam trong tran mo-men");
  }
  {
    SafetyLimits L; L.ramp_secs = 0.0; L.fault_action = g1_wbc::FaultAction::ZeroTorque;
    SafetyGate g(limits(), L);
    g.arm();
    SafetyInputs in = goodInput();
    in.com_err = Eigen::Vector3d(0.3, 0, 0);
    in.dq = Eigen::VectorXd::Constant(kN, 3.0);
    const auto out = g.filter(Eigen::VectorXd::Constant(kN, 50.0), in, kDt);
    check(out.cwiseAbs().maxCoeff() == 0.0, "che do buong han phai ra mo-men 0");
  }

  std::cout << "\n" << (g_run - g_fail) << "/" << g_run << " phep thu dat\n"
            << (g_fail ? "FAIL\n" : "PASS\n");
  return g_fail ? 1 : 0;
}
