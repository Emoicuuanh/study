// Ban giao quyen dieu khien tu bo dieu khien cua Unitree sang WBC cua ta.
//
// CHUONG TRINH NAY CO THE LAM ROBOT CHUYEN DONG. Mac dinh thi khong:
//   - khong co --send thi chi tinh toan va in, khong publish gi
//   - co --send thi van phai go dung mot cau de xac nhan
//   - co --release-mode moi tat bo dieu khien cua hang
//
// BA GIAI DOAN, lam theo dung thu tu. Hai giai doan dau ROBOT PHAI DANG TREO
// tren gian, chan khong cham dat - luc do du co sai gi cung khong nga duoc.
//
//   --stage damp   Lenh nho nhat co the: tau=0, kp=0, kd nho. Robot treo
//                  mem nhung co giam chan. Dung de tra loi BON cau hoi chua
//                  biet: (a) robot co nhan ban tin khong (CRC, mode_machine),
//                  (b) nut giam chan tren tay cam con tac dung sau khi
//                  ReleaseMode khong, (c) ROBOT LAM GI KHI LENH NGUNG DEN -
//                  khong tim thay watchdog nao trong SDK nen day la an so
//                  that su, (d) rut cap Ethernet thi sao.
//
//   --stage hold   Giu nguyen tu the hien tai bang servo vi tri, kp NHO.
//                  Dung de kiem tra anh xa chi so khop: neu nham thu tu khop
//                  thi o day se thay ngay, va robot dang treo nen vo hai.
//                  CANH BAO: o giai doan nay mo-men do DONG CO sinh ra tu kp,
//                  nen CONG AN TOAN KHONG CHAN DUOC. Thu bao ve duy nhat la
//                  kp nho va robot dang treo.
//
//   --stage wbc    Mo-men tu WBC, di qua cong an toan. CHI chay giai doan nay
//                  khi chan da chiu tai va day gian da chung - vi cong thuc
//                  WBC gia thiet hai chan cham dat.
#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "g1_wbc/balance_wbc.hpp"
#include "g1_wbc/leg_odometry.hpp"
#include "g1_wbc/lowcmd.hpp"
#include "g1_wbc/safety.hpp"

namespace
{
constexpr int kN = g1_wbc::kNMotor;
std::atomic<bool> g_stop{false};
void onSignal(int) {g_stop = true;}

enum class Stage {Damp, Hold, Wbc};

const char * stageName(Stage s)
{
  switch (s) {
    case Stage::Damp: return "damp (chi giam chan)";
    case Stage::Hold: return "hold (giu tu the)";
    case Stage::Wbc: return "wbc  (mo-men tu WBC)";
  }
  return "?";
}
}  // namespace

struct Handover
{
  g1_wbc::LegOdometry odom;
  g1_wbc::BalanceWBC wbc;
  g1_wbc::SafetyGate gate;
  g1_wbc::LowCmdSender tx;
  Stage stage;

  std::mutex mu;
  std::atomic<long> n_msg{0};
  long n_cycle = 0;
  int divider = 2;
  double settle_secs = 2.0;
  bool ref_done = false, armed = false;
  std::chrono::steady_clock::time_point t_first;
  double t_prev = -1.0;
  Eigen::VectorXd q_hold, zero;
  // Lech MOT khop so voi tu the dang treo. Giu dung cho robot dang nam thi
  // khong can mo-men nao - va nhu the khong phan biet duoc voi giam chan.
  // Phai co sai lech thi dong co moi buoc phai sinh luc on dinh.
  int off_joint = -1;
  double off_rad = 0.0;
  // Tang dan do lech thay vi ap tuc thi. O giai doan hold, cong an toan
  // KHONG nam tren duong lenh (mo-men do kp cua dong co sinh ra), nen khong
  // co gi chan cu giat luc bat. Lech cang lon thi cang phai tang cham.
  double off_ramp_secs = 3.0;
  double t_arm = -1.0;
  double tau_max_seen = 0.0;
  std::string last_fault;

  Handover(
    const std::string & urdf, const std::string & payload, Stage st,
    const g1_wbc::LowCmdOpts & copts, const g1_wbc::SafetyLimits & lim)
  : odom(urdf), wbc(urdf, payload, g1_wbc::Gains{}),
    gate(wbc.tauHi(), lim), tx(copts), stage(st),
    q_hold(Eigen::VectorXd::Zero(kN)), zero(Eigen::VectorXd::Zero(kN)) {}

  void onLowState(const void * message)
  {
    const auto * m = static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
    const auto now = std::chrono::steady_clock::now();
    const long k = ++n_msg;
    if (k % divider) {return;}
    if (k == divider) {t_first = now;}
    const double t = std::chrono::duration<double>(now - t_first).count();

    tx.setModeMachine(m->mode_machine());

    Eigen::VectorXd q(kN), dq(kN), te(kN);
    for (int i = 0; i < kN; ++i) {
      q[i] = m->motor_state()[i].q();
      dq[i] = m->motor_state()[i].dq();
      te[i] = m->motor_state()[i].tau_est();
    }
    const Eigen::Vector3d gyro(
      m->imu_state().gyroscope()[0], m->imu_state().gyroscope()[1],
      m->imu_state().gyroscope()[2]);
    const Eigen::Vector4d quat(
      m->imu_state().quaternion()[0], m->imu_state().quaternion()[1],
      m->imu_state().quaternion()[2], m->imu_state().quaternion()[3]);

    if (!ref_done) {
      if (t < settle_secs) {return;}
      q_hold = q;
      wbc.captureReference(q, quat);
      ref_done = true;
      std::cout << "da chot tu the tham chieu. Go ARM roi Enter de mo cong an toan.\n";
      return;
    }
    if (!armed) {
      if (!tx.dry()) {tx.sendDamping();}     // giu giam chan trong luc cho
      return;
    }

    const double dt = (t_prev >= 0.0) ? std::max(t - t_prev, 1e-5) : 0.002;
    const double age_ms = (t_prev >= 0.0) ? (t - t_prev) * 1e3 : 0.0;
    t_prev = t;

    Eigen::VectorXd tau_ff = Eigen::VectorXd::Zero(kN);
    Eigen::VectorXd q_tgt = Eigen::VectorXd::Zero(kN);

    if (stage == Stage::Hold) {
      // Mo-men do DONG CO sinh ra tu kp*(q_tgt - q). Cong an toan khong nam
      // tren duong nay - no chi chan tau_ff. Bao ve o day la kp nho + gian treo.
      q_tgt = q_hold;
      if (off_joint >= 0 && off_joint < kN) {
        if (t_arm < 0.0) {t_arm = t;}
        const double a = (off_ramp_secs <= 0.0) ? 1.0 :
          std::min(1.0, (t - t_arm) / off_ramp_secs);
        q_tgt[off_joint] += a * off_rad;
      }
    } else if (stage == Stage::Wbc) {
      const auto & od = odom.update(q, dq, gyro, te, quat);
      auto r = wbc.solve(q, dq, quat, gyro, od.v_body, od.contact[0], od.contact[1]);
      g1_wbc::SafetyInputs si;
      si.dq = dq;
      si.quat = quat;
      si.v_body = od.v_body;
      si.com_err = r.com_err;
      si.contact_left = od.contact[0];
      si.contact_right = od.contact[1];
      si.qp_ok = r.ok;
      si.cycle_ms = wbc.timing().dyn + wbc.timing().build + wbc.timing().qp;
      si.state_age_ms = age_ms;
      tau_ff = gate.filter(r.ok ? r.tau : Eigen::VectorXd::Zero(kN), si, dt);
      if (gate.state() == g1_wbc::SafetyState::Fault) {
        std::lock_guard<std::mutex> lk(mu);
        if (last_fault.empty()) {
          last_fault = gate.report().fault;
          g_stop = true;                     // CHOT: dung han, khong tu dat lai
        }
      }
    }
    // Stage::Damp: tau_ff = 0, q_tgt = 0, kp = 0 -> chi con kd cua dong co.

    tx.send(tau_ff, q_tgt, zero);
    std::lock_guard<std::mutex> lk(mu);
    ++n_cycle;
    tau_max_seen = std::max(tau_max_seen, tau_ff.cwiseAbs().maxCoeff());
  }
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr <<
      "dung: handover_node <card_mang> --stage damp|hold|wbc [--send]\n"
      "      [--release-mode] [--kd N] [--kp N] [--secs N]\n"
      "      [--urdf U] [--payload P]\n\n"
      "Khong co --send thi KHONG gui gi den robot.\n";
    return 2;
  }
  std::string iface = argv[1];
  std::string urdf = "G1/src/g1_description/urdf/g1_29dof.urdf";
  std::string payload = "G1/src/g1_leg_odometry/config/payload.yaml";
  Stage stage = Stage::Damp;
  g1_wbc::LowCmdOpts copts;          // dry = true, kp = 0, kd = 1
  bool release = false;
  double secs = 0.0;
  int off_joint = -1;
  double off_rad = 0.0, off_ramp = 3.0;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--stage" && i + 1 < argc) {
      const std::string s = argv[++i];
      if (s == "damp") {stage = Stage::Damp;} else if (s == "hold") {stage = Stage::Hold;} else if
      (s == "wbc") {stage = Stage::Wbc;} else {
        std::cerr << "giai doan la: " << s << "\n"; return 2;
      }
    } else if (a == "--send") {copts.dry = false;} else if (a == "--release-mode") {
      release = true;
    } else if (a == "--kd" && i + 1 < argc) {copts.kd = std::stod(argv[++i]);} else if (a ==
      "--kp" && i + 1 < argc) {copts.kp = std::stod(argv[++i]);} else if (a == "--secs" &&
      i + 1 < argc) {secs = std::stod(argv[++i]);} else if (a == "--urdf" && i + 1 < argc) {
      urdf = argv[++i];
    } else if (a == "--payload" && i + 1 < argc) {payload = argv[++i];} else if (a ==
      "--offset-joint" && i + 1 < argc) {off_joint = std::stoi(argv[++i]);} else if (a ==
      "--offset-rad" && i + 1 < argc) {off_rad = std::stod(argv[++i]);} else if (a ==
      "--offset-ramp" && i + 1 < argc) {off_ramp = std::stod(argv[++i]);} else {
      std::cerr << "tham so la: " << a << "\n"; return 2;
    }
  }
  if (stage == Stage::Hold && copts.kp <= 0.0) {copts.kp = 5.0;}
  if (stage != Stage::Hold && copts.kp != 0.0) {
    std::cerr << "chi giai doan 'hold' moi duoc dat kp khac 0\n";
    return 2;
  }

  g1_wbc::SafetyLimits lim;
  if (off_joint >= 0 && stage != Stage::Hold) {
    std::cerr << "--offset-joint chi dung voi --stage hold\n";
    return 2;
  }
  // Tran 1.0 rad (57 do): co tran de mot lan go nham so khong thanh mot cu
  // vung tay. Gia tri lon van vao tu tu nho off_ramp_secs.
  if (std::abs(off_rad) > 1.0) {
    std::cerr << "--offset-rad qua lon (toi da 1.0 rad)\n";
    return 2;
  }
  Handover h(urdf, payload, stage, copts, lim);
  h.off_joint = off_joint;
  h.off_rad = off_rad;
  h.off_ramp_secs = off_ramp;
  h.tx.setModeMachine(0);

  std::cout << "\n==================================================\n"
            << " giai doan : " << stageName(stage) << "\n"
            << " gui that  : " << (copts.dry ? "KHONG (che do kho)" : "CO") << "\n"
            << " kp / kd   : " << copts.kp << " / " << copts.kd << "\n"
            << " tat bo dk cua hang: " << (release ? "CO" : "KHONG") << "\n"
            << (off_joint >= 0 ?
    " LECH KHOP   : khop " + std::to_string(off_joint) + " lech " +
    std::to_string(off_rad) + " rad (" + std::to_string(off_rad * 57.2958) +
    " do), tang dan trong " + std::to_string(off_ramp) + " s\n" : "")
            << "==================================================\n";
  if (!copts.dry) {
    std::cout
      << "\nCHUONG TRINH NAY SE GUI LENH DEN ROBOT.\n"
      << (stage == Stage::Wbc ?
      "Giai doan 'wbc' gia thiet HAI CHAN CHAM DAT. Chi chay khi day gian da chung.\n" :
      "Robot PHAI DANG TREO, chan khong cham dat.\n")
      << "Tay cam Unitree phai trong tay nguoi dieu khien.\n"
      << "\nGo dung: TOI DONG Y\n> ";
    std::string line;
    std::getline(std::cin, line);
    if (line != "TOI DONG Y") {
      std::cout << "khong xac nhan - thoat, chua gui gi.\n";
      return 1;
    }
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, iface);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub(
    new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
  h.tx.init();            // sau ChannelFactory::Init, truoc khi gui bat cu gi
  sub->InitChannel([&h](const void * m) {h.onLowState(m);}, 10);
  std::cout << "nghe rt/lowstate tren '" << iface << "'\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  if (h.n_msg.load() == 0) {
    std::cerr << "khong nhan duoc lowstate - sai card mang? Thoat.\n";
    return 2;
  }

  if (release) {
    std::cout << "\n*** SAP TAT BO DIEU KHIEN CUA HANG ***\n"
              << "Sau lenh nay robot MEM RA ngay lap tuc.\n"
              << "Go dung: TAT DI\n> ";
    std::string line;
    std::getline(std::cin, line);
    if (line != "TAT DI") {std::cout << "khong xac nhan - thoat.\n"; return 1;}
    if (!h.tx.releaseMode()) {std::cerr << "khong tat duoc - thoat.\n"; return 2;}
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  // Doi chot tu the, roi doi nguoi dieu khien go ARM.
  std::thread arm_thread([&h]() {
      std::string line;
      while (!g_stop && std::getline(std::cin, line)) {
        if (line == "ARM") {
          // KHONG duoc arm truoc khi co moc tham chieu: o giai doan wbc thi
          // solve() se nem loi, con o moi giai doan thi nguoi van hanh se
          // tuong da mo cong trong khi chua co gi de bam vao.
          if (!h.ref_done) {
            std::cout << "chua chot duoc tu the tham chieu - cho roi go lai ARM.\n";
            continue;
          }
          h.gate.arm();
          h.armed = true;
          std::cout << "DA ARM - he so tang dan tu 0.\n";
          return;
        }
        std::cout << "(go ARM de mo, hoac Ctrl-C de thoat)\n";
      }
    });

  const auto t0 = std::chrono::steady_clock::now();
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (secs > 0.0 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >= secs)
    {
      break;
    }
    std::lock_guard<std::mutex> lk(h.mu);
    std::cout << "chu ky " << h.n_cycle << " | "
              << (h.armed ? "DA ARM" : "chua arm") << " | ";
    if (stage == Stage::Wbc) {
      std::cout << "cong an toan " << toString(h.gate.state())
                << " he so " << h.gate.report().alpha;
    } else {
      // Noi thang: o hai giai doan nay cong an toan KHONG nam tren duong lenh.
      // Giai doan damp khong co mo-men nao de chan; giai doan hold thi mo-men
      // do dong co sinh ra tu kp, cong khong voi toi.
      std::cout << (stage == Stage::Hold ?
        "cong an toan KHONG AP DUNG (mo-men do kp cua dong co sinh ra)" :
        "cong an toan KHONG AP DUNG (giai doan nay khong phat mo-men)");
    }
    std::cout << " | |tau| max da gui " << h.tau_max_seen << " Nm\n";
  }

  std::cout << "\ndung lai - gui giam chan.\n";
  h.armed = false;
  for (int i = 0; i < 50; ++i) {
    h.tx.sendDamping();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  if (!h.last_fault.empty()) {
    std::cout << "CONG AN TOAN DA NGAT: " << h.last_fault << "\n";
  }
  std::cout << "da gui " << h.tx.nSent() << " ban tin"
            << (h.tx.dry() ? " (che do kho - khong ban tin nao ra khoi may)" : "") << "\n";
  arm_thread.detach();
  return 0;
}
