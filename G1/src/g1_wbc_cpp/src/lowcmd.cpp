#include "g1_wbc/lowcmd.hpp"

#include <chrono>
#include <iostream>
#include <thread>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

namespace g1_wbc
{
namespace
{
// Sao y tu example/g1/low_level/g1_ankle_swing_example.cpp cua SDK.
// Robot TU CHOI ban tin neu CRC sai, nen day cung la mot lop bao ve: ban tin
// hong tren duong truyen se bi bo chu khong duoc thi hanh.
inline uint32_t crc32Core(uint32_t * ptr, uint32_t len)
{
  uint32_t xbit = 0, data = 0, CRC32 = 0xFFFFFFFF;
  const uint32_t poly = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1u << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (CRC32 & 0x80000000) {CRC32 <<= 1; CRC32 ^= poly;} else {CRC32 <<= 1;}
      if (data & xbit) {CRC32 ^= poly;}
      xbit >>= 1;
    }
  }
  return CRC32;
}
}  // namespace

struct LowCmdSender::Impl
{
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> pub;
  std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> msc;
  unitree_hg::msg::dds_::LowCmd_ msg;
};

LowCmdSender::LowCmdSender(const LowCmdOpts & opts)
: p_(std::make_unique<Impl>()), opts_(opts)
{
  if (opts_.dry) {
    std::cout << "LowCmdSender: CHE DO KHO - dung ban tin nhung KHONG gui.\n";
    return;
  }
  std::cout << "\n*** LowCmdSender: CHE DO GUI THAT - se publish rt/lowcmd ***\n\n";
}

void LowCmdSender::init()
{
  if (opts_.dry || p_->pub) {return;}
  p_->pub.reset(
    new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>("rt/lowcmd"));
  p_->pub->InitChannel();
  std::cout << "publisher rt/lowcmd da san sang.\n";
}

LowCmdSender::~LowCmdSender()
{
  // Loi thoat cuoi cung: du thoat kieu gi cung de lai lenh giam chan.
  if (!opts_.dry && p_->pub) {
    try {sendDamping();} catch (...) {}
  }
}

bool LowCmdSender::releaseMode()
{
  if (opts_.dry) {
    std::cout << "(kho) bo qua releaseMode\n";
    return false;
  }
  p_->msc.reset(new unitree::robot::b2::MotionSwitcherClient());
  p_->msc->SetTimeout(5.0f);
  p_->msc->Init();
  std::string form, name;
  for (int attempt = 0; attempt < 5; ++attempt) {
    p_->msc->CheckMode(form, name);
    if (name.empty()) {
      std::cout << "bo dieu khien cua hang DA TAT.\n";
      return true;
    }
    std::cout << "dang tat che do '" << name << "' (lan " << attempt + 1 << ")...\n";
    const int32_t ret = p_->msc->ReleaseMode();
    std::cout << (ret == 0 ? "  ReleaseMode OK\n" : "  ReleaseMode loi, ma " +
      std::to_string(ret) + "\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  std::cerr << "KHONG tat duoc bo dieu khien cua hang.\n";
  return false;
}

void LowCmdSender::send(
  const Eigen::VectorXd & tau_ff, const Eigen::VectorXd & q_target,
  const Eigen::VectorXd & dq_target)
{
  auto & m = p_->msg;
  m.mode_pr() = 0;
  m.mode_machine() = mode_machine_;
  for (int i = 0; i < kNCmdMotor; ++i) {
    auto & c = m.motor_cmd().at(i);
    c.mode() = 1;                       // 1 = cho phep, 0 = tat
    c.tau() = static_cast<float>(tau_ff[i]);
    c.q() = static_cast<float>(q_target[i]);
    c.dq() = static_cast<float>(dq_target[i]);
    c.kp() = static_cast<float>(opts_.kp);
    c.kd() = static_cast<float>(opts_.kd);
  }
  m.crc() = crc32Core(reinterpret_cast<uint32_t *>(&m), (sizeof(m) >> 2) - 1);
  ++n_sent_;
  if (opts_.dry) {return;}
  if (!p_->pub) {
    std::cerr << "LowCmdSender::send goi khi chua init() - KHONG gui.\n";
    return;
  }
  p_->pub->Write(m);
}

void LowCmdSender::sendDamping()
{
  const Eigen::VectorXd z = Eigen::VectorXd::Zero(kNCmdMotor);
  const double kp_save = opts_.kp;
  opts_.kp = 0.0;                        // giam chan thuan, khong servo vi tri
  send(z, z, z);
  opts_.kp = kp_save;
}

}  // namespace g1_wbc
