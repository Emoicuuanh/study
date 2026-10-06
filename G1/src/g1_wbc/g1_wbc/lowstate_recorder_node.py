"""Ghi rt/lowstate THO ra file, KHONG xu ly gi, KHONG gui lenh gi.

AN TOAN: node nay CHI subscribe rt/lowstate. No KHONG publish rt/lowcmd,
KHONG goi LocoClient, KHONG gui bat cu gi den robot. Bo dieu khien cua Unitree
van giu robot binh thuong, khong he biet node nay ton tai.

Dung de lam gi: mot ban ghi 40 giay la du lieu goc cho ba viec sau day, ma sau
do deu lam duoc KHI KHONG CO ROBOT:
  1. phat lai (replay.py)       - chay lai leg odometry + WBC tren dung du lieu
                                  do, doi he so, xem ket qua doi the nao
  2. sinh test vector           - chot dau vao/dau ra lam moc cho ban C++
  3. do nhieu, do tan so        - phan tich lai bao nhieu lan tuy thich

    ros2 run g1_wbc lowstate_recorder_node --ros-args \
        -p network_interface:=wlp0s20f3 -p label:="day vai + di vai buoc"

Danh dau su kien trong luc ghi (tu terminal khac, tuy chon):
    ros2 topic pub --once /rec_mark std_msgs/String "{data: 'day lan 1'}"

Ctrl-C de dung: file duoc ghi trong luc thoat (da chan SIGINT nen khong mat).
"""
import os
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from g1_wbc.recording import Buffer, N_MOTOR_RAW, default_dir, find_robot_interface

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    SDK_OK = True
except ImportError:
    SDK_OK = False


class RecorderNode(Node):
    def __init__(self):
        super().__init__("lowstate_recorder_node")
        self.declare_parameter("network_interface", "auto")
        self.declare_parameter("robot_subnet", "192.168.123.")
        self.declare_parameter("lowstate_topic", "rt/lowstate")
        self.declare_parameter("out_dir", "")
        self.declare_parameter("name", "")            # rong = dat theo thoi gian
        self.declare_parameter("label", "")           # mo ta viec dang lam
        self.declare_parameter("max_secs", 120.0)     # chan bo nho
        self.declare_parameter("expect_hz", 1052.0)
        self.declare_parameter("stop_after_secs", 0.0)   # 0 = ghi den khi Ctrl-C

        self.max_secs = float(self.get_parameter("max_secs").value)
        cap = int(self.max_secs * float(self.get_parameter("expect_hz").value) * 1.2)
        self.buf = Buffer(cap)
        self._lock = threading.Lock()
        self.t0 = None
        self.t_node = time.monotonic()
        self.n_drop = 0
        self.warned_full = False
        self.saved = False

        out_dir = self.get_parameter("out_dir").value or default_dir()
        name = self.get_parameter("name").value or time.strftime("%Y%m%d_%H%M%S")
        self.out_path = os.path.join(out_dir, f"lowstate_{name}.npz")
        if os.path.exists(self.out_path):
            raise RuntimeError(f"file da ton tai, khong ghi de: {self.out_path}")
        os.makedirs(out_dir, exist_ok=True)

        self.create_subscription(String, "/rec_mark", self.on_mark, 10)

        iface = self.get_parameter("network_interface").value
        if not SDK_OK:
            raise RuntimeError("khong import duoc unitree_sdk2py")
        subnet = self.get_parameter("robot_subnet").value
        if iface == "auto":
            iface, addr = find_robot_interface(subnet)
            if not iface:
                raise RuntimeError(
                    f"khong card nao o mang robot {subnet}x - robot da bat chua? "
                    "day/wifi da noi chua?")
            self.get_logger().info(f"tu chon card '{iface}' ({addr}) o mang robot")
        elif iface:
            base = f"/sys/class/net/{iface}"
            if not os.path.isdir(base):
                raise RuntimeError(f"khong co card mang '{iface}'")
            if open(f"{base}/carrier").read().strip() != "1":
                raise RuntimeError(f"card '{iface}' khong co ket noi - kiem tra cap/wifi")
            good, addr = find_robot_interface(subnet)
            if good and good != iface:
                # Canh bao chu khong chan: co the nguoi dung co y chi dinh.
                self.get_logger().warn(
                    f"card '{iface}' KHONG o mang robot, ma '{good}' ({addr}) thi co. "
                    "Chi dinh sai card thi node chay im lang nhung khong nhan duoc gi.")
        self.iface = iface
        ChannelFactoryInitialize(0, iface) if iface else ChannelFactoryInitialize(0)
        self.sub = ChannelSubscriber(self.get_parameter("lowstate_topic").value, LowState_)
        self.sub.Init(self.on_lowstate, 20)

        self.create_timer(2.0, self.report)
        self.get_logger().warn("CHI GHI - KHONG gui lenh nao den robot.")
        self.get_logger().info(
            f"dich: {self.out_path} | suc chua {cap} khung (~{self.max_secs:.0f}s)")

    def on_mark(self, msg):
        with self._lock:
            t = time.monotonic() - (self.t0 or time.monotonic())
            self.buf.mark(t, msg.data)
        self.get_logger().info(f"moc: {msg.data}")

    def on_lowstate(self, msg):
        now = time.monotonic()
        if self.t0 is None:
            self.t0 = now
        ms = msg.motor_state
        with self._lock:
            ok = self.buf.append(
                t=now - self.t0,
                tick=msg.tick,
                mode_pr=msg.mode_pr,
                mode_machine=msg.mode_machine,
                crc=msg.crc,
                imu_temp=msg.imu_state.temperature,
                quat=msg.imu_state.quaternion,
                gyro=msg.imu_state.gyroscope,
                accel=msg.imu_state.accelerometer,
                rpy=msg.imu_state.rpy,
                q=np.fromiter((ms[i].q for i in range(N_MOTOR_RAW)), np.float32, N_MOTOR_RAW),
                dq=np.fromiter((ms[i].dq for i in range(N_MOTOR_RAW)), np.float32, N_MOTOR_RAW),
                ddq=np.fromiter((ms[i].ddq for i in range(N_MOTOR_RAW)), np.float32, N_MOTOR_RAW),
                tau_est=np.fromiter((ms[i].tau_est for i in range(N_MOTOR_RAW)),
                                    np.float32, N_MOTOR_RAW),
                motor_mode=np.fromiter((ms[i].mode for i in range(N_MOTOR_RAW)),
                                       np.uint8, N_MOTOR_RAW),
                wireless_remote=np.frombuffer(
                    bytes(msg.wireless_remote), dtype=np.uint8),
            )
            if not ok:
                self.n_drop += 1

    def report(self):
        with self._lock:
            n, drop, t0 = self.buf.n, self.n_drop, self.t0
        stop = float(self.get_parameter("stop_after_secs").value)
        if t0 is None:
            self.get_logger().warn("chua nhan duoc lowstate nao - kiem tra card mang/robot")
            # Van phai tu dung: neu khong co du lieu thi t0 mai la None, va truoc
            # day node se treo mai khong bao gio cham duoc moc stop_after_secs.
            if stop > 0 and time.monotonic() - self.t_node >= stop + 5.0:
                self.get_logger().error("khong co du lieu sau khi het gio - dung")
                raise KeyboardInterrupt
            return
        el = time.monotonic() - t0
        self.get_logger().info(
            f"da ghi {n} khung | {el:.0f}s | {n/max(el,1e-3):.0f} Hz | "
            f"bo nho {100*n/self.buf.capacity:.0f}%")
        if drop and not self.warned_full:
            self.warned_full = True
            self.get_logger().error("BO DEM DAY - dung lai, tang max_secs neu can")
        if stop > 0 and el >= stop:
            self.get_logger().info(f"du {stop:.0f}s - dung")
            raise KeyboardInterrupt

    def finish(self):
        if self.saved:
            return
        self.saved = True
        with self._lock:
            n = self.buf.n
        if n < 10:
            print(f"\nCHI co {n} khung - khong ghi file.")
            return
        meta = dict(
            label=self.get_parameter("label").value,
            iface=self.iface,
            topic=self.get_parameter("lowstate_topic").value,
            host=os.uname().nodename,
            source="robot",
        )
        p = self.buf.save(self.out_path, meta)
        sz = os.path.getsize(p) / 1e6
        print(f"\nDA GHI {n} khung -> {p}  ({sz:.1f} MB)")
        print("Phat lai (khong can robot):")
        print(f"  .venv-real/bin/python G1/src/g1_wbc/g1_wbc/replay.py {p}")


def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Da tung mat bao cao vi Ctrl-C thu hai giet tien trinh giua chung.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.finish()
        except Exception as e:
            print(f"loi khi ghi file: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
