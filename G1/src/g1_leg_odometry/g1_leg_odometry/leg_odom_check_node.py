"""Do chat luong leg odometry TREN ROBOT THAT. Chi doc, khong dieu khien gi.

Muc dich chinh: do NEN NHIEU cua van toc uoc luong khi robot dung yen. Luc do
van toc that = 0, nen do lech chuan cua /leg_odom CHINH LA nhieu phep do. Con
so do di thang vao hai cho:
    g1_leg_odometry/config/leg_odometry.yaml   -> vel_std
    g1_state_estimator/config/estimator.yaml   -> leg_vel_noise

Muc dich phu: doi chieu voi mot nguon van toc DOC LAP, neu co.
  - /dog_odom (ROS): LUU Y - khong co node nao trong workspace nay publish
    topic do. No den tu mot bridge ben ngoai. Neu bridge khong chay thi bo qua.
  - rt/odommodestate (DDS): truong velocity[3] la uoc luong cua CHINH Unitree,
    lay thang qua SDK, khong can bridge nao. DA KIEM CHUNG tren robot that:
    500 Hz, kieu unitree_go::msg::dds_::SportModeState_.
    (KHONG dung rt/lf/sportmodestate: robot publish no voi kieu
     unitree_hg::msg::dds_::SportModeState_, ma unitree_sdk2py ban nay KHONG co
     dinh nghia IDL do -> reader khong khop, khong nhan duoc gi.
     rt/lf/odommodestate cung dung kieu unitree_go nhung chi 20 Hz.)
    PHAI TU KIEM CHUNG he quy chieu cua truong nay (he than hay he the gioi)
    bang cach cho robot xoay tai cho.

An toan: node nay va leg_odometry_node deu CHI DOC. Khong node nao publish
rt/lowcmd hay goi LocoClient. Chay duoc song song voi bo dieu khien co san
cua Unitree - do la ly do buoc nay lam duoc TRUOC khi co bat ky mo-men nao
do ta phat ra.

Chay:
    ros2 run g1_leg_odometry leg_odom_check_node --ros-args \
        -p sdk_ref:=true -p network_interface:=eth0 -p csv_path:=/tmp/legodom.csv
"""
import csv
import math
import signal
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
    SDK_OK = True
except ImportError:
    SDK_OK = False


class Check(Node):
    def __init__(self):
        super().__init__("leg_odom_check_node")
        self.declare_parameter("leg_odom_topic", "/leg_odom")
        self.declare_parameter("contact_topic", "/leg_contact")
        self.declare_parameter("ref_odom_topic", "/dog_odom")
        self.declare_parameter("sdk_ref", False)
        self.declare_parameter("sdk_ref_topic", "rt/odommodestate")
        self.declare_parameter("network_interface", "")
        self.declare_parameter("static_speed", 0.05)     # m/s, nguong coi la dung yen
        self.declare_parameter("ref_max_age", 0.05)      # s, gioi han ghep cap theo thoi gian
        self.declare_parameter("csv_path", "")
        self.declare_parameter("report_period", 5.0)

        self.static_speed = float(self.get_parameter("static_speed").value)
        self.ref_max_age = float(self.get_parameter("ref_max_age").value)

        self._lock = threading.Lock()
        self.v_samples = []          # tat ca mau van toc tu /leg_odom
        self.static_samples = []     # chi nhung mau khi dung yen
        self.pairs = []              # (v_leg, v_ref) da ghep theo thoi gian
        self.fz = []
        self.stamps = []
        self.ref_last = None         # (t_mono, v_ref)
        self.per_foot = [math.nan] * 6
        self.n_leg = 0
        self.n_ref = 0
        self.t0 = time.monotonic()

        self.create_subscription(Odometry, self.get_parameter("leg_odom_topic").value,
                                 self.on_leg, 50)
        self.create_subscription(Float32MultiArray, self.get_parameter("contact_topic").value,
                                 self.on_contact, 50)
        self.create_subscription(Odometry, self.get_parameter("ref_odom_topic").value,
                                 self.on_ref_ros, 50)

        self.csv = None
        path = self.get_parameter("csv_path").value
        if path:
            self._csv_f = open(path, "w", newline="")
            self.csv = csv.writer(self._csv_f)
            self.csv.writerow(["t", "vx", "vy", "vz", "ref_vx", "ref_vy", "ref_vz",
                               "fz_l", "fz_r", "c_l", "c_r",
                               "lvx", "lvy", "lvz", "rvx", "rvy", "rvz"])
            self.get_logger().info(f"ghi CSV: {path}")

        if self.get_parameter("sdk_ref").value:
            if not SDK_OK:
                self.get_logger().error("khong import duoc unitree_sdk2py, bo qua sdk_ref")
            else:
                iface = self.get_parameter("network_interface").value
                ChannelFactoryInitialize(0, iface) if iface else ChannelFactoryInitialize(0)
                topic = self.get_parameter("sdk_ref_topic").value
                self._sub_sdk = ChannelSubscriber(topic, SportModeState_)
                self._sub_sdk.Init(self.on_ref_sdk, 10)
                self.get_logger().info(f"doi chieu voi DDS {topic} (SportModeState.velocity)")

        self.create_timer(float(self.get_parameter("report_period").value), self.report)
        self.get_logger().info(
            "dang do. De robot DUNG YEN it nhat 30s de lay nen nhieu, "
            "roi day nhe / cho di de xem phan dong. Ctrl-C de ra bao cao.")

    # ---------------- callbacks ----------------
    def on_leg(self, msg):
        v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        now = time.monotonic()
        with self._lock:
            self.n_leg += 1
            self.v_samples.append(v)
            self.stamps.append(now)
            if np.linalg.norm(v) < self.static_speed:
                self.static_samples.append(v)
            ref = None
            if self.ref_last is not None and now - self.ref_last[0] < self.ref_max_age:
                ref = self.ref_last[1]
                self.pairs.append((v, ref))
            if self.csv:
                r = ref if ref is not None else [math.nan] * 3
                f = self.fz[-1] if self.fz else [math.nan] * 4
                self.csv.writerow([f"{now - self.t0:.4f}", *(f"{x:.5f}" for x in v),
                                   *(f"{x:.5f}" for x in r), *(f"{x:.2f}" for x in f),
                                   *(f"{x:.5f}" for x in self.per_foot)])

    def on_contact(self, msg):
        # /leg_contact mang 10 gia tri: fz_l, fz_r, c_l, c_r, roi van toc TUNG
        # CHAN (lvx,lvy,lvz, rvx,rvy,rvz). Phai tach dung, neu khong CSV se
        # thua cot so voi header.
        d = list(msg.data)
        with self._lock:
            self.fz.append(d[:4])
            self.per_foot = d[4:10] if len(d) >= 10 else [math.nan] * 6

    def on_ref_ros(self, msg):
        v = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        with self._lock:
            self.n_ref += 1
            self.ref_last = (time.monotonic(), v)

    def on_ref_sdk(self, msg):
        v = np.array(msg.velocity, dtype=float)
        with self._lock:
            self.n_ref += 1
            self.ref_last = (time.monotonic(), v)

    # ---------------- bao cao ----------------
    def report(self):
        with self._lock:
            n_leg, n_ref, n_st = self.n_leg, self.n_ref, len(self.static_samples)
            self.n_leg = self.n_ref = 0
            v = self.v_samples[-1] if self.v_samples else None
        p = float(self.get_parameter("report_period").value)
        if v is None:
            self.get_logger().warn(
                f"chua nhan duoc /leg_odom - leg_odometry_node da chay chua?")
            return
        self.get_logger().info(
            f"/leg_odom {n_leg/p:5.0f} Hz | doi chieu {n_ref/p:5.0f} Hz | "
            f"mau dung yen {n_st:6d} | v=[{v[0]:+.3f} {v[1]:+.3f} {v[2]:+.3f}]")

    def final(self):
        with self._lock:
            allv = np.array(self.v_samples)
            st = np.array(self.static_samples)
            pairs = list(self.pairs)
            stamps = np.array(self.stamps)
        print("\n" + "=" * 68)
        if len(allv) == 0:
            print("KHONG nhan duoc mau nao tu /leg_odom.")
            return
        dur = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
        print(f"Tong {len(allv)} mau trong {dur:.1f}s ({len(allv)/max(dur,1e-9):.0f} Hz)")
        if len(stamps) > 2:
            gaps = np.diff(stamps)
            lost = gaps[gaps > 0.05].sum()
            print(f"Khoang cach mau: trung vi {np.median(gaps)*1e3:.2f} ms, "
                  f"lon nhat {gaps.max()*1e3:.1f} ms")
            if lost > 0.5:
                print(f"  CANH BAO: mat {lost:.1f}s du lieu trong {dur:.1f}s "
                      f"({100*lost/dur:.0f}%) - xem lai cap ethernet")

        print("\n--- TACH NHIEU CAM BIEN KHOI DAO DONG THAT ---")
        print("  Bo can bang cua hang lien tuc chinh tu the, nen robot dung yen VAN")
        print("  dao dong that. Do lech chuan tho = nhieu + dao dong, khong dung lam")
        print("  vel_std duoc. Dao dong that muot o thang 1ms, nhieu trang thi khong:")
        print("      sigma_nhieu = std(hieu 2 mau lien tiep) / sqrt(2)")
        dt = np.diff(stamps)
        good = dt < 0.005
        noise = np.zeros(3)
        for i, ax in enumerate("xyz"):
            d = np.diff(allv[:, i])[good]
            noise[i] = d.std() / np.sqrt(2)
            raw = allv[:, i].std()
            sway = np.sqrt(max(raw ** 2 - noise[i] ** 2, 0.0))
            print(f"  {ax}: tong {raw*1000:7.2f} = nhieu {noise[i]*1000:6.2f} "
                  f"+ dao dong that {sway*1000:7.2f}  (mm/s)")
        rec_n = float(np.ceil(noise.max() * 1.5 * 1000) / 1000)
        print(f"\n  => vel_std        = {rec_n:.3f}   (leg_odometry.yaml)")
        print(f"  => leg_vel_noise  = [{rec_n**2:.2e}, {rec_n**2:.2e}, {rec_n**2:.2e}]"
              f"   (estimator.yaml, IEKF dung PHUONG SAI)")
        print("  (Gia dinh nhieu trang. Neu encoder da duoc loc san trong firmware,")
        print("   cach nay se DANH GIA THAP nhieu - coi day la can duoi.)")

        print("\n--- SO LIEU THO KHI DUNG YEN (de tham khao) ---")
        if len(st) < 500:
            print(f"  CHI CO {len(st)} mau dung yen - CHUA DU. Cho robot dung yen "
                  f">=30s roi do lai; dung con so duoi day.")
        if len(st) > 0:
            for i, ax in enumerate("xyz"):
                print(f"  {ax}: trung binh {st[:,i].mean()*1000:+7.2f} mm/s (do lech he thong)   "
                      f"do lech chuan {st[:,i].std()*1000:6.2f} mm/s")
            bias = np.abs(st.mean(axis=0))
            if bias.max() > 0.02:
                print(f"\n  CANH BAO: do lech he thong {bias.max()*1000:.0f} mm/s la LON. "
                      f"Kha nang: gyro chua tru bias, hoac chan bi truot, "
                      f"hoac nguong tiep xuc sai (xem fz o log).")

        print("\n--- DOI CHIEU NGUON DOC LAP ---")
        if not pairs:
            print("  Khong co mau doi chieu nao. /dog_odom khong co node nao trong")
            print("  workspace nay publish; thu -p sdk_ref:=true de lay thang tu DDS.")
        else:
            a = np.array([p[0] for p in pairs]); b = np.array([p[1] for p in pairs])
            d = a - b
            print(f"  {len(pairs)} cap da ghep")
            for i, ax in enumerate("xyz"):
                print(f"  {ax}: lech trung binh {d[:,i].mean()*1000:+7.1f} mm/s   "
                      f"RMS {np.sqrt((d[:,i]**2).mean())*1000:7.1f} mm/s")
            mov = np.linalg.norm(b, axis=1) > 0.1
            if mov.sum() > 100:
                dm = d[mov]
                print(f"  chi khi DANG DI CHUYEN ({mov.sum()} mau): "
                      f"RMS {np.sqrt((dm**2).sum(axis=1).mean())*1000:.1f} mm/s")
            print("  LUU Y he quy chieu: leg odometry tra ve van toc trong HE THAN.")
            print("  Neu nguon doi chieu la he THE GIOI, lech se lon khi robot xoay -")
            print("  do khong phai loi uoc luong. Kiem chung bang cach cho robot xoay tai cho.")
        print("=" * 68)


def main(args=None):
    rclpy.init(args=args)
    node = Check()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Khoa SIGINT: bao cao la san pham duy nhat cua buoi do, Ctrl-C lan hai
        # KHONG duoc phep giet no giua chung.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.final()
        except Exception as e:
            print(f"loi khi dung bao cao: {e}")
        if node.csv:
            node._csv_f.close()
            print(f"CSV: {node.get_parameter('csv_path').value}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
