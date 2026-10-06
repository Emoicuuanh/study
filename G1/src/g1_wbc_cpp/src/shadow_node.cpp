// CHE DO BONG ban C++: chay leg odometry + WBC tren du lieu robot that, theo
// thoi gian thuc, nhung KHONG GUI LENH NAO.
//
// AN TOAN. Chuong trinh nay CHI tao MOT ChannelSubscriber tren rt/lowstate.
// Khong co ChannelPublisher nao, khong co LocoClient, khong co rt/lowcmd.
// Bo dieu khien cua Unitree van giu robot, khong he biet chuong trinh nay ton tai.
//
// Vi sao can dung ban C++ chay song: doi chieu test vector da chung minh ban dich
// DUNG, nhung moi do duoc thoi gian tren 125 tu the roi rac. Cai chua biet la
// cai DUOI: p99 va xau nhat tren hang chuc nghin chu ky lien tuc, co ca cap phat
// bo nho, lo cache va lich trinh cua he dieu hanh chen vao. Do chinh la thu da
// giet ban Python (p99 3.39 ms, xau nhat 13.58 ms tren ngan sach 2.0 ms).
//
//   shadow_node <card_mang> [--secs N] [--div N] [--csv FILE] [--warm]
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <thread>
#include <string>
#include <vector>

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "g1_wbc/balance_wbc.hpp"
#include "g1_wbc/leg_odometry.hpp"
#include "g1_wbc/safety.hpp"
#include <map>

namespace
{
constexpr int kN = g1_wbc::kNMotor;
std::atomic<bool> g_stop{false};
void onSignal(int) {g_stop = true;}

// Kiem tra libddsc va libddscxx co DEN TU CUNG MOT NOI khong.
// Da mat nhieu gio vi cho nay: /opt/ros/jazzy cung co libddsc.so.0, va source
// setup.bash cua ROS (de lay pinocchio) dat no len truoc tren LD_LIBRARY_PATH.
// Chuong trinh chay binh thuong vai chuc nghin goi roi moi sap o mot free() bat
// ky. Khong co dau hieu nao chi ve nguyen nhan. Nen chan ngay tu dau.
std::string ddsLibDirs(bool & mismatch)
{
  std::ifstream f("/proc/self/maps");
  std::string line, dir_c, dir_cxx;
  while (std::getline(f, line)) {
    const auto sp = line.rfind(' ');
    if (sp == std::string::npos) {continue;}
    const std::string path = line.substr(sp + 1);
    const auto slash = path.rfind('/');
    if (slash == std::string::npos) {continue;}
    const std::string base = path.substr(slash + 1), dir = path.substr(0, slash);
    if (base.rfind("libddscxx.so", 0) == 0 && dir_cxx.empty()) {dir_cxx = dir;} else if (base.rfind(
        "libddsc.so", 0) == 0 && dir_c.empty()) {dir_c = dir;}
  }
  mismatch = !dir_c.empty() && !dir_cxx.empty() && dir_c != dir_cxx;
  return "  libddsc  : " + (dir_c.empty() ? "(chua nap)" : dir_c) + "\n" +
         "  libddscxx: " + (dir_cxx.empty() ? "(chua nap)" : dir_cxx) + "\n";
}

double pct(std::vector<double> v, double p)
{
  if (v.empty()) {return 0.0;}
  std::sort(v.begin(), v.end());
  return v[static_cast<size_t>(p / 100.0 * (v.size() - 1))];
}
}  // namespace

struct Shadow
{
  g1_wbc::LegOdometry odom;
  g1_wbc::BalanceWBC wbc;
  int divider = 2;
  double settle_secs = 1.0;
  // Loc thong thap bac 1 10 Hz tren dau vao - GIONG ban Python. Do tren robot
  // that: khong loc thi rung 1.289 Nm, loc 10 Hz con 0.568 Nm, va kha nang
  // chong day KHONG giam (dao dong thang bang chi 1.6 Hz nen tre pha khong dang ke).
  double vel_fc = 10.0, dq_fc = 10.0;
  // --norun: chi nhan va dem, khong chay odometry/WBC. De tach xem cho
  // hong nam o phan toan cua ta hay o tang SDK/DDS.
  bool no_compute = false;
  // CHE DO KHO: cong an toan chay that moi chu ky, nhung dau ra cua no chi de
  // bao cao. Khong co duong nao tu day den rt/lowcmd.
  std::unique_ptr<g1_wbc::SafetyGate> gate;
  std::map<std::string, long> fault_counts;
  long n_gate = 0, n_gate_clip = 0, n_gate_rate = 0;
  double gate_max_clip = 0.0, tau_sent_max = 0.0;
  double t_prev_msg = -1.0;

  // timed_mutex, KHONG phai mutex: bao cao dinh ky tuyet doi khong duoc phep
  // treo duong thoat. Da gap - node chay mai khong dung duoc, SIGINT vo hieu.
  std::timed_mutex mu;
  // Luong DDS van goi callback sau khi main thoat vong lap. Khong chan lai
  // thi Shadow bi huy trong khi callback con dung -> hong vung nho.
  std::atomic<bool> stopping{false};
  // Dem so callback dang chay CUNG LUC. BalanceWBC/LegOdometry deu co bo dem
  // dung chung (H_, A_eq_, data_...) nen hai callback chong nhau se pha vung
  // nho. Do that chu khong doan: neu max_cb > 1 thi dung la the.
  std::atomic<int> in_cb{0};
  std::atomic<int> max_cb{0};
  std::atomic<long> n_msg{0};
  long n_ok = 0, n_fail = 0, n_odom_bad = 0;
  bool ref_done = false;
  std::chrono::steady_clock::time_point t_first;
  std::vector<double> ms, ms_wbc;
  size_t report_from = 0;
  Eigen::VectorXd dq_filt = Eigen::VectorXd::Zero(kN);
  Eigen::Vector3d v_filt = Eigen::Vector3d::Zero();
  bool v_init = false;
  double t_prev = -1.0;
  Eigen::VectorXd tau_sum = Eigen::VectorXd::Zero(kN);
  Eigen::VectorXd tau_sq = Eigen::VectorXd::Zero(kN);
  Eigen::VectorXd te_sum = Eigen::VectorXd::Zero(kN);
  Eigen::VectorXd te_sq = Eigen::VectorXd::Zero(kN);
  long n_stat = 0, n_sign = 0, n_sign_tot = 0, n_single = 0;
  double t_dyn = 0, t_build = 0, t_qp = 0;
  long n_split = 0, qp_iters = 0;
  double com_err_max = 0.0;
  std::ofstream csv;

  Shadow(const std::string & urdf, const std::string & payload)
  : odom(urdf), wbc(urdf, payload, g1_wbc::Gains{}) {}

  void onLowState(const void * message)
  {
    if (stopping) {return;}
    const int depth = ++in_cb;
    int prev = max_cb.load();
    while (depth > prev && !max_cb.compare_exchange_weak(prev, depth)) {}
    struct Leave {std::atomic<int> & c; ~Leave() {--c;}} leave{in_cb};
    // KHONG khoa ca callback. Da DO duoc max_cb = 1 (callback khong bao gio
    // chong nhau) nen bo dem tinh toan khong can khoa. Con khoa ca phan tinh
    // toan thi luong DDS giu mutex ~40% thoi gian va khoa lai ngay sau khi nha,
    // lam luong chinh BI BO DOI: node khong thoat duoc va SIGINT cung vo hieu.
    // Chi khoa quanh so lieu dung chung, that ngan.
    const auto * m = static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
    const auto t_now = std::chrono::steady_clock::now();
    const long k = ++n_msg;
    if (k % divider) {return;}
    if (k == divider) {t_first = t_now;}
    const double t = std::chrono::duration<double>(t_now - t_first).count();

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

    if (no_compute) {++n_ok; return;}

    const auto t0 = std::chrono::steady_clock::now();

    Eigen::VectorXd dq_ctrl = dq;
    if (dq_fc > 0.0) {
      const double dt = (t_prev >= 0.0) ? std::max(t - t_prev, 1e-5) : 0.002;
      t_prev = t;
      const double a = dt / (1.0 / (2 * M_PI * dq_fc) + dt);
      dq_filt += a * (dq - dq_filt);
      dq_ctrl = dq_filt;
    }

    const auto & od = odom.update(q, dq, gyro, te, quat);
    if (!std::isfinite(od.v_body[0]) || od.v_body.norm() > 10.0) {
      ++n_odom_bad;
      return;
    }

    if (!ref_done) {
      if (t < settle_secs) {return;}
      wbc.captureReference(q, quat);
      ref_done = true;
      if (gate) {
        gate->arm();
        std::cout << "cong an toan: da arm (CHE DO KHO - khong co lenh nao duoc gui)\n";
      }
      std::cout << "da chot moc tham chieu (tu the / CoM / huong hien tai)\n";
      return;
    }

    Eigen::Vector3d v_ctrl = od.v_body;
    if (vel_fc > 0.0) {
      if (!v_init) {v_filt = od.v_body; v_init = true;}
      const double dtv = 0.002;
      const double av = dtv / (1.0 / (2 * M_PI * vel_fc) + dtv);
      v_filt += av * (od.v_body - v_filt);
      v_ctrl = v_filt;
    }

    const auto tw0 = std::chrono::steady_clock::now();
    auto r = wbc.solve(q, dq_ctrl, quat, gyro, v_ctrl, od.contact[0], od.contact[1]);
    const auto t1 = std::chrono::steady_clock::now();

    std::lock_guard<std::timed_mutex> lk(mu);
    if (!r.ok) {++n_fail; return;}
    ++n_ok;
    // Do ca "toan bo chu ky" (loc + odometry + WBC) lan "rieng QP": con so dau
    // la cai phai vua ngan sach 2 ms, con so sau de biet cat o dau neu khong vua.
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    ms_wbc.push_back(std::chrono::duration<double, std::milli>(t1 - tw0).count());
    tau_sum += r.tau; tau_sq += r.tau.cwiseProduct(r.tau);
    te_sum += te; te_sq += te.cwiseProduct(te);
    ++n_stat;
    if (od.contact[0] != od.contact[1]) {++n_single;}
    for (int i = 0; i < 12; ++i) {
      ++n_sign_tot;
      if ((r.tau[i] >= 0) == (te[i] >= 0)) {++n_sign;}
    }
    com_err_max = std::max(com_err_max, r.com_err.norm());

    if (gate) {
      g1_wbc::SafetyInputs si;
      si.dq = dq;                      // dq THO: gioi han an toan phai nhin so
      si.quat = quat;                  // lieu that, khong phai so da loc
      si.v_body = od.v_body;
      si.com_err = r.com_err;
      si.contact_left = od.contact[0];
      si.contact_right = od.contact[1];
      si.qp_ok = r.ok;
      si.cycle_ms = ms.back();
      si.state_age_ms = (t_prev_msg >= 0.0) ? (t - t_prev_msg) * 1e3 : 0.0;
      const double dt_gate = (t_prev_msg >= 0.0) ? std::max(t - t_prev_msg, 1e-5) : 0.002;
      const auto & tau_sent = gate->filter(r.tau, si, dt_gate);
      ++n_gate;
      tau_sent_max = std::max(tau_sent_max, tau_sent.cwiseAbs().maxCoeff());
      if (gate->report().n_clipped) {++n_gate_clip;}
      if (gate->report().n_rate_limited) {++n_gate_rate;}
      gate_max_clip = std::max(gate_max_clip, gate->report().max_clip);
      if (gate->state() == g1_wbc::SafetyState::Fault) {
        // Gom ly do bo phan so de dem duoc theo nhom.
        std::string key = gate->report().fault;
        const auto d = key.find_first_of("0123456789");
        if (d != std::string::npos) {key = key.substr(0, d);}
        ++fault_counts[key];
        // Chi o CHE DO KHO moi tu dat lai: muc dich la dem xem trong ca buoi se
        // ngat bao nhieu lan va vi sao. Khi dieu khien that thi TUYET DOI khong.
        gate->reset();
        gate->arm();
      }
    }
    t_prev_msg = t;
    t_dyn += wbc.timing().dyn;
    t_build += wbc.timing().build;
    t_qp += wbc.timing().qp;
    qp_iters += wbc.qpIters();
    ++n_split;
    if (csv.is_open()) {
      csv << std::setprecision(9) << t;
      for (int i = 0; i < kN; ++i) {csv << "," << r.tau[i];}
      for (int i = 0; i < kN; ++i) {csv << "," << te[i];}
      csv << "," << r.com_err[0] << "," << r.com_err[1] << "," << r.com_err[2]
          << "," << r.fz_left << "," << r.fz_right
          << "," << od.contact[0] << "," << od.contact[1]
          << "," << od.v_body[0] << "," << od.v_body[1] << "," << od.v_body[2]
          << "," << ms.back() << "\n";
    }
  }

  void report(bool final_)
  {
    std::unique_lock<std::timed_mutex> lk(mu, std::chrono::milliseconds(100));
    if (!lk.owns_lock()) {
      std::cout << "(bo qua mot bao cao: khong lay duoc khoa trong 100 ms)\n";
      return;
    }
    // Bao cao dinh ky: chi phan tu lan truoc den gio. Bao cao cuoi: TOAN BO.
    // (Ban dau xoa sach ms moi lan bao cao dinh ky -> tong ket ra rong.)
    const size_t lo = final_ ? 0 : report_from;
    std::vector<double> w(ms.begin() + lo, ms.end());
    std::vector<double> ww(ms_wbc.begin() + lo, ms_wbc.end());
    report_from = ms.size();
    if (no_compute) {
      std::cout << "chi nhan (--norun): " << n_msg.load() << " goi, " << n_ok << " chu ky\n";
      return;
    }
    if (w.empty()) {
      std::cout << (n_msg.load() ? "chua co chu ky nao giai duoc\n"
        : "chua nhan duoc lowstate nao - sai card mang?\n");
      return;
    }
    const auto sd = [&](const Eigen::VectorXd & s, const Eigen::VectorXd & s2) {
        double acc = 0.0;
        for (int i = 0; i < 12; ++i) {
          acc += std::sqrt(std::max(s2[i] / n_stat - std::pow(s[i] / n_stat, 2), 0.0));
        }
        return acc / 12.0;
      };
    std::cout << std::fixed << std::setprecision(3)
              << (final_ ? "=== TONG KET ===\n" : "")
              << "chu ky " << n_ok << " | QP fail " << n_fail
              << " | odom bo " << n_odom_bad
              << " | callback chong nhau toi da " << max_cb.load()
              << " | mot chan " << std::setprecision(1)
              << (100.0 * n_single / std::max(1L, n_stat)) << "%\n"
              << std::setprecision(3)
              << "  ca chu ky: TB " << std::accumulate(w.begin(), w.end(), 0.0) / w.size()
              << " | p99 " << pct(w, 99) << " | p999 " << pct(w, 99.9)
              << " | max " << *std::max_element(w.begin(), w.end())
              << "  (ngan sach 2.0 ms)\n"
              << "  rieng QP : TB "
              << std::accumulate(ww.begin(), ww.end(), 0.0) / ww.size()
              << " | p99 " << pct(ww, 99)
              << " | max " << *std::max_element(ww.begin(), ww.end()) << "\n"
              << "  chia nho : pinocchio " << t_dyn / std::max(1L, n_split)
              << " | dung ma tran " << t_build / std::max(1L, n_split)
              << " | giai QP " << t_qp / std::max(1L, n_split)
              << " ms | vong lap TB " << std::setprecision(1)
              << static_cast<double>(qp_iters) / std::max(1L, n_split)
              << std::setprecision(3) << "\n"
              << (gate ?
    "  cong an toan (KHO): " + std::to_string(fault_counts.size()) + " loai loi | cat tran " +
    std::to_string(n_gate ? 100 * n_gate_clip / n_gate : 0) + "% chu ky | chan toc do " +
    std::to_string(n_gate ? 100 * n_gate_rate / n_gate : 0) + "% | |tau| gui max " +
    std::to_string(tau_sent_max) + " Nm\n" : "")
              << "  rung mo-men chan: ta " << sd(tau_sum, tau_sq)
              << " vs Unitree " << sd(te_sum, te_sq) << " Nm"
              << " | cung dau " << std::setprecision(0)
              << (100.0 * n_sign / std::max(1L, n_sign_tot)) << "%"
              << " | CoM lech max " << std::setprecision(1) << com_err_max * 1e3 << " mm\n";
  }
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "dung: shadow_node <card_mang> [--secs N] [--div N] [--csv F]\n"
              << "      [--urdf U] [--payload P] [--eps X] [--warm]\n";
    return 2;
  }
  std::string iface = argv[1], csv_path;
  std::string urdf = "G1/src/g1_description/urdf/g1_29dof.urdf";
  std::string payload = "G1/src/g1_leg_odometry/config/payload.yaml";
  double secs = 0.0;
  int div = 2;
  bool sh_no_compute = false;
  bool use_gate = true;
  g1_wbc::SolverOpts opts;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--secs" && i + 1 < argc) {secs = std::stod(argv[++i]);} else if (a == "--div" &&
      i + 1 < argc) {div = std::stoi(argv[++i]);} else if (a == "--csv" && i + 1 < argc) {
      csv_path = argv[++i];
    } else if (a == "--urdf" && i + 1 < argc) {urdf = argv[++i];} else if (a == "--payload" &&
      i + 1 < argc) {payload = argv[++i];} else if (a == "--eps" && i + 1 < argc) {
      opts.eps_abs = std::stod(argv[++i]);
    } else if (a == "--warm") {opts.warm_start = true;} else if (a == "--eiq") {
      opts.solver = g1_wbc::QpSolver::EiQuadProg;
    } else if (a == "--nogate") {
      use_gate = false;
    } else if (a == "--norun") {
      sh_no_compute = true;
    } else {
      std::cerr << "tham so la: " << a << "\n"; return 2;
    }
  }

  std::cout << "\n*** CHE DO BONG - CHI DOC rt/lowstate, KHONG GUI LENH NAO ***\n"
            << "    Bo dieu khien cua Unitree van dang giu robot.\n\n";

  Shadow sh(urdf, payload);
  if (use_gate) {
    sh.gate = std::make_unique<g1_wbc::SafetyGate>(
      sh.wbc.tauHi(), g1_wbc::SafetyLimits{});
  }
  sh.no_compute = sh_no_compute;
  sh.divider = div;
  sh.wbc.setSolverOpts(opts);
  if (!csv_path.empty()) {
    sh.csv.open(csv_path);
    sh.csv << "t";
    for (int i = 0; i < kN; ++i) {sh.csv << ",tau" << i;}
    for (int i = 0; i < kN; ++i) {sh.csv << ",tauest" << i;}
    sh.csv << ",cex,cey,cez,fzl,fzr,cl,cr,vx,vy,vz,ms\n";
  }
  std::cout << "mo hinh WBC: " << std::fixed << std::setprecision(3) << sh.wbc.mass()
            << " kg | chia tan so 1/" << div << " | eps " << opts.eps_abs << "\n";

  {
    bool mismatch = false;
    const std::string info = ddsLibDirs(mismatch);
    if (mismatch) {
      std::cerr << "\n*** DUNG: libddsc va libddscxx den tu HAI noi khac nhau ***\n"
                << info
                << "Hai ban CycloneDDS trong mot tien trinh se pha heap sau vai\n"
                << "chuc nghin goi. Dat thu muc thirdparty cua unitree_sdk2 len\n"
                << "TRUOC tren LD_LIBRARY_PATH (xem test/run_shadow.sh).\n";
      return 3;
    }
    std::cout << info;
  }
  unitree::robot::ChannelFactory::Instance()->Init(0, iface);
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub(
    new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>("rt/lowstate"));
  sub->InitChannel([&sh](const void * msg) {sh.onLowState(msg);}, 10);
  std::cout << "nghe rt/lowstate tren '" << iface << "'\n";

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  const auto t_start = std::chrono::steady_clock::now();
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    // Kiem tra dieu kien dung TRUOC khi bao cao: bao cao la viec phu, khong
    // duoc dung truoc duong thoat.
    if (secs > 0.0 &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count() >= secs)
    {
      break;
    }
    sh.report(false);
  }
  // Chan callback RO I moi dong kenh, roi moi doc so lieu lan cuoi.
  sh.stopping = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  sub->CloseChannel();
  std::cout << "\n";
  sh.report(true);
  if (sh.gate && !sh.fault_counts.empty()) {
    std::cout << "\n--- cong an toan se da NGAT o nhung truong hop nay ---\n";
    long tot = 0;
    for (const auto & kv : sh.fault_counts) {tot += kv.second;}
    for (const auto & kv : sh.fault_counts) {
      std::cout << "  " << std::setw(6) << kv.second << " lan ("
                << std::setw(5) << std::setprecision(2)
                << (100.0 * kv.second / std::max(1L, sh.n_gate)) << "%)  " << kv.first << "\n";
    }
    std::cout << "  tong " << tot << " lan tren " << sh.n_gate << " chu ky\n"
              << "  (che do kho: moi lan ngat deu tu dat lai de dem tiep;\n"
              << "   khi dieu khien that thi lan dau tien la dung han)\n";
  }
  if (sh.csv.is_open()) {sh.csv.close(); std::cout << "CSV: " << csv_path << "\n";}
  return 0;
}
