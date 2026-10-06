// Doc file test vector chuan (sinh tu ban Python), chay BalanceWBC ban C++ tren
// DUNG dau vao do, ghi ket qua ra file cung dinh dang. Doi chieu bang:
//
//   python G1/src/g1_wbc/g1_wbc/test_vectors.py check CHUAN.txt RA_CPP.txt
//
// Day la doi ung C++ cua test_vectors.rerun() ben Python.
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "g1_wbc/balance_wbc.hpp"
#include "g1_wbc/leg_odometry.hpp"

using Row = std::vector<double>;

struct Vectors
{
  std::map<std::string, Row> head;
  std::vector<std::map<std::string, Row>> vecs;
};

Vectors read(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {throw std::runtime_error("khong mo duoc " + path);}
  Vectors out;
  std::string line;
  bool in_vec = false;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream ss(line);
    std::string key;
    ss >> key;
    if (key == "END") {break;}
    if (key == "VEC") {
      out.vecs.emplace_back();
      in_vec = true;
      continue;
    }
    Row v;
    double x;
    while (ss >> x) {v.push_back(x);}
    (in_vec ? out.vecs.back() : out.head)[key] = v;
  }
  return out;
}

Eigen::VectorXd vec(const Row & r) {return Eigen::Map<const Eigen::VectorXd>(r.data(), r.size());}

void writeRow(std::ostream & o, const std::string & k, const Eigen::VectorXd & v)
{
  o << k;
  for (int i = 0; i < v.size(); ++i) {o << " " << std::setprecision(17) << v[i];}
  o << "\n";
}

int main(int argc, char ** argv)
{
  if (argc < 5) {
    std::cerr << "dung: run_test_vectors CHUAN.txt RA.txt URDF [PAYLOAD.yaml]"
              << " [--eps X] [--warm] [--bench N]\n";
    return 2;
  }
  const std::string gold = argv[1], out_path = argv[2], urdf = argv[3];
  std::string payload;
  g1_wbc::SolverOpts opts;
  int bench = 1;
  for (int i = 4; i < argc; ++i) {
    const std::string a = argv[i];
    const auto next = [&]() {return std::string(argv[++i]);};
    if (a == "--eps" && i + 1 < argc) {
      opts.eps_abs = std::stod(next());
    } else if (a == "--bench" && i + 1 < argc) {
      bench = std::stoi(next());
    } else if (a == "--warm") {
      opts.warm_start = true;
    } else if (a == "--noprecond") {
      opts.precond = false;
    } else if (a == "--eiq") {
      opts.solver = g1_wbc::QpSolver::EiQuadProg;
    } else if (a.rfind("--", 0) != 0) {
      payload = a;
    } else {
      std::cerr << "tham so la: " << a << "\n";
      return 2;
    }
  }

  Vectors g = read(gold);
  const Row & gn = g.head.at("GAINS");
  g1_wbc::Gains gains;
  gains.mu = gn[0]; gains.fz_min = gn[1]; gains.kd_contact = gn[2];
  gains.kp_com = gn[3]; gains.kd_com = gn[4];
  gains.kp_ori = gn[5]; gains.kd_ang = gn[6];
  gains.kp_q = gn[7]; gains.kd_q = gn[8];
  gains.w_com = gn[9]; gains.w_ang = gn[10]; gains.w_post = gn[11];
  gains.w_f = gn[12]; gains.w_qacc = gn[13];
  gains.w_tau = (gn.size() > 14) ? gn[14] : 0.0;   // dinh dang 3 khong co w_tau

  g1_wbc::BalanceWBC wbc(urdf, payload, gains);
  wbc.setSolverOpts(opts);
  // Leg odometry dung URDF GOC (khong ap tai trong): khoi luong khong vao phep
  // tinh van toc, va uoc luong luc chi dung quan tinh cac khau CHAN. Giong ban
  // Python, noi LegOdometry(urdf) khong truyen payload.
  g1_wbc::LegOdometry odom(urdf);
  g1_wbc::OdomResult od;
  std::cout << "mo hinh C++: " << std::setprecision(9) << wbc.mass() << " kg"
            << " | chuan: " << g.head.at("MASS")[0] << " kg\n";

  const Eigen::VectorXd ref_q = vec(g.head.at("REF_IN_Q"));
  const Row & rq = g.head.at("REF_IN_QUAT");
  wbc.captureReference(ref_q, Eigen::Vector4d(rq[0], rq[1], rq[2], rq[3]));

  std::ofstream o(out_path);
  if (!o) {std::cerr << "khong ghi duoc " << out_path << "\n"; return 2;}
  o << "# g1_wbc test vectors (ban C++)\n";
  o << "FORMAT " << static_cast<int>(g.head.at("FORMAT")[0]) << "\n";
  o << "NVEC " << g.vecs.size() << "\n";
  o << "NMOTOR " << g1_wbc::kNMotor << "\nNV " << wbc.nv() << "\nNC " << wbc.nc() << "\n";
  writeRow(o, "MASS", Eigen::Matrix<double, 1, 1>(wbc.mass()));
  writeRow(o, "GAINS", vec(gn));
  writeRow(o, "REF_IN_Q", ref_q);
  writeRow(o, "REF_IN_QUAT", vec(rq));
  writeRow(o, "REF_QREF", wbc.qRef());
  writeRow(o, "REF_COMDES", wbc.comDes());
  const auto & qd = wbc.quatDes();
  writeRow(o, "REF_QUATDES", Eigen::Vector4d(qd.w(), qd.x(), qd.y(), qd.z()));
  writeRow(o, "REF_ANCHOR", wbc.anchorRef());

  int n_fail = 0;
  std::vector<double> ms;
  std::vector<int> iters;
  double t_dyn = 0, t_build = 0, t_qp = 0;
  long n_t = 0;
  // --bench N: chay LAI CA DAY vector N lan, khong phai giai lai cung mot
  // vector N lan. Khac biet quan trong: voi khoi dong nong, giai lai cung mot
  // vector thi QP hoi tu trong 0 vong lap va con so do duoc la gia. Chay ca
  // day thi moi lan giai bat dau tu nghiem cua mot tu the KHAC - do la tinh
  // huong that. (Van bi quan hon that: cac vector duoc chon co y CACH XA nhau,
  // con hai chu ky lien tiep tren robot thi gan nhau.)
  for (int pass = 0; pass < bench; ++pass) {
    const bool last = (pass == bench - 1);
    const bool measure = (bench == 1) || (pass > 0);   // bo lan dau: nap cache
    for (size_t k = 0; k < g.vecs.size(); ++k) {
      const auto & v = g.vecs[k];
      const Row & qu = v.at("IN_QUAT");
      const Row & ct = v.at("IN_CONTACT");
      // Chay leg odometry MOI LUOT, khong chi luot ghi: neu chi chay o luot
      // cuoi thi --bench 3000 van chi goi no 55 lan, khong kiem duoc gi.
      const bool has_te = v.count("IN_TAUEST") > 0;
      Eigen::VectorXd dqr;
      if (has_te) {
        dqr = vec(v.count("IN_DQRAW") ? v.at("IN_DQRAW") : v.at("IN_DQ"));
        odom.perFrame(
          vec(v.at("IN_Q")), dqr,
          Eigen::Vector3d(v.at("IN_GYRO")[0], v.at("IN_GYRO")[1], v.at("IN_GYRO")[2]),
          vec(v.at("IN_TAUEST")),
          Eigen::Vector4d(qu[0], qu[1], qu[2], qu[3]), od);
      }
      const auto t0 = std::chrono::steady_clock::now();
      auto r = wbc.solve(
        vec(v.at("IN_Q")), vec(v.at("IN_DQ")),
        Eigen::Vector4d(qu[0], qu[1], qu[2], qu[3]),
        Eigen::Vector3d(v.at("IN_GYRO")[0], v.at("IN_GYRO")[1], v.at("IN_GYRO")[2]),
        Eigen::Vector3d(v.at("IN_VBODY")[0], v.at("IN_VBODY")[1], v.at("IN_VBODY")[2]),
        ct[0] > 0.5, ct[1] > 0.5);
      const double dt = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();
      if (measure) {
        ms.push_back(dt);
        iters.push_back(wbc.qpIters());
        t_dyn += wbc.timing().dyn;
        t_build += wbc.timing().build;
        t_qp += wbc.timing().qp;
        ++n_t;
      }
      if (!last) {continue;}
      if (!r.ok) {
        std::cerr << "  vector " << k << " FAIL: " << r.error << "\n";
        ++n_fail;
        r.tau = Eigen::VectorXd::Zero(g1_wbc::kNMotor);
        r.qacc = Eigen::VectorXd::Zero(wbc.nv());
        r.f = Eigen::VectorXd::Zero(3 * wbc.nc());
      }
      o << "VEC " << k << "\n";
      writeRow(o, "IN_Q", vec(v.at("IN_Q")));
      writeRow(o, "IN_DQ", vec(v.at("IN_DQ")));
      writeRow(o, "IN_QUAT", vec(qu));
      writeRow(o, "IN_GYRO", vec(v.at("IN_GYRO")));
      writeRow(o, "IN_VBODY", vec(v.at("IN_VBODY")));
      writeRow(o, "IN_CONTACT", vec(ct));
      if (has_te) {
        writeRow(o, "IN_TAUEST", vec(v.at("IN_TAUEST")));
        writeRow(o, "IN_DQRAW", dqr);
      }
      writeRow(o, "OUT_TAU", r.tau);
      writeRow(o, "OUT_QACC", r.qacc);
      writeRow(o, "OUT_F", r.f);
      if (has_te) {
        Eigen::Matrix<double, 6, 1> vf;
        vf << od.v_foot[0], od.v_foot[1];
        writeRow(o, "OUT_VFOOT", vf);
        writeRow(o, "OUT_FZ", Eigen::Vector2d(od.fz[0], od.fz[1]));
      }
    }
  }
  o << "END\n";

  double sum = 0.0, mx = 0.0;
  for (double x : ms) {sum += x; mx = std::max(mx, x);}
  std::sort(ms.begin(), ms.end());
  double it_sum = 0; int it_max = 0;
  for (int x : iters) {it_sum += x; it_max = std::max(it_max, x);}
  std::cout << "da giai " << g.vecs.size() << " vector, that bai " << n_fail
            << " | vong lap QP: TB " << static_cast<int>(it_sum / iters.size())
            << " max " << it_max
            << " | eps_abs " << opts.eps_abs << (opts.warm_start ? " + khoi dong nong" : "")
            << "\n"
            << "thoi gian giai: TB " << std::setprecision(4) << sum / ms.size()
            << " ms | p99 " << ms[static_cast<size_t>(0.99 * (ms.size() - 1))]
            << " | max " << mx << "   (ngan sach 500 Hz = 2.0 ms)\n"
            << "  chia nho: pinocchio " << t_dyn / n_t << " | dung ma tran "
            << t_build / n_t << " | giai QP " << t_qp / n_t << " ms"
            << "  (" << static_cast<int>(100 * t_qp / (t_dyn + t_build + t_qp))
            << "% o QP)\n"
            << "da ghi " << out_path << "\n";
  return n_fail ? 1 : 0;
}
