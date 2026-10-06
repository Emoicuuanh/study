// Duong PHAT LENH den robot: rt/lowcmd.
//
// Day la file duy nhat trong ca du an co the lam robot chuyen dong. Moi thu
// khac - WBC, leg odometry, cong an toan, node bong - deu chi doc.
//
// BA LOP CHAN, phai mo TUNG CAI MOT bang tay:
//   1. `dry` mac dinh TRUE  -> dung ban tin nhung KHONG publish.
//   2. Cong an toan mac dinh KHOA -> mo-men ra bang 0 du co publish.
//   3. Phai goi releaseMode() tuong minh -> truoc do robot van do bo dieu
//      khien cua hang giu, va lenh cua ta bi no lan at.
//
// MOT LUA CHON THIET KE DANG NOI: mo-men cua WBC di vao `tau_ff`, con `kp`
// de BANG 0 va `kd` de mot gia tri nho KHAC 0. Nghia la bo giam chan nam o
// TRONG dong co, khong phai trong vong lap cua ta. Neu tien trinh nay chet,
// treo, hay bi he dieu hanh cho ngu, dong co van tu giam chan - khong phu
// thuoc vao viec ma cua ta co chay nua hay khong.
//
// VAN CON MOT AN SO: khong tim thay watchdog nao trong SDK, nen CHUA BIET
// robot lam gi khi lenh ngung den. Phai thu khi robot dang TREO truoc.
#pragma once

#include <array>
#include <memory>
#include <string>

#include <Eigen/Core>

namespace g1_wbc
{

inline constexpr int kNCmdMotor = 29;

struct LowCmdOpts
{
  // Giam chan o muc DONG CO. Khac 0 de khi ma cua ta ngung chay thi dong co
  // van ghi lai chuyen dong. Don vi Nm/(rad/s).
  double kd = 1.0;
  // Mac dinh 0: KHONG dung servo vi tri. Chi stage "hold" moi dat khac 0.
  double kp = 0.0;
  bool dry = true;        // true = dung ban tin, khong gui
};

class LowCmdSender
{
public:
  explicit LowCmdSender(const LowCmdOpts & opts);
  ~LowCmdSender();

  // PHAI goi SAU ChannelFactory::Instance()->Init(). Tao publisher truoc khi
  // factory san sang thi sap ngay (SIGSEGV) - da gap o lan chay dau tien.
  // Tach rieng khoi ham khoi tao de thu tu nay la BAT BUOC, khong phai quy uoc.
  void init();

  // Tat bo dieu khien cua hang. SAU LENH NAY ROBOT MEM RA va chi dung duoc
  // neu lenh cua ta tiep quan kip - hoac neu no dang duoc gian treo do.
  // Tra ve true neu da tat duoc.
  bool releaseMode();

  // mode_machine phai COPY tu lowstate; robot tu choi ban tin neu sai.
  void setModeMachine(uint8_t m) {mode_machine_ = m;}

  // Gui mot chu ky. q_target/dq_target chi co tac dung khi kp/kd khac 0.
  void send(
    const Eigen::VectorXd & tau_ff, const Eigen::VectorXd & q_target,
    const Eigen::VectorXd & dq_target);

  // Chi giam chan: tau=0, kp=0, kd giu nguyen. Dung khi loi va khi thoat.
  void sendDamping();

  long nSent() const {return n_sent_;}
  bool dry() const {return opts_.dry;}

private:
  struct Impl;
  std::unique_ptr<Impl> p_;
  LowCmdOpts opts_;
  uint8_t mode_machine_ = 0;
  long n_sent_ = 0;
};

}  // namespace g1_wbc
