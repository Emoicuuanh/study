"""CHE DO BONG: chay WBC tren du lieu robot that nhung KHONG GUI LENH.

Giua "WBC chay tot trong mo phong" va "WBC cam lai robot that" con bon an so
ma mo phong khong tra loi duoc. Node nay tra loi ca bon, rui ro vat ly BANG
KHONG - dung logic da dung cho leg odometry:

  1. QP co LUON giai duoc o moi tu the that khong, hay co luc vo nghiem?
  2. Thoi gian giai tren du lieu that bao nhieu, co vuot ngan sach khong?
  3. Mo-men WBC tinh ra co CUNG DAU va CUNG BAC DO LON voi mo-men bo dieu
     khien cua Unitree dang thuc su phat (motor_state.tau_est) khong?
     Lech hoan toan = sai co ban, va biet truoc khi treo gian thi re hon nhieu.
  4. Uoc luong luc tiep xuc lech anh huong the nao den nghiem QP?

AN TOAN: node nay subscribe rt/lowstate va publish ROS topic. No KHONG publish
rt/lowcmd, KHONG goi LocoClient, KHONG gui bat cu gi den robot. Chay song song
voi bo dieu khien cua hang - robot van do bo dieu khien do giu.

KHONG PHAI so sanh "phai giong nhau". Bo dieu khien cua Unitree giai bai toan
khac (co the la RL policy), voi muc tieu khac. Dieu dang xem la: cung dau,
cung bac do lon o cac khop CHAN, va khong co khop nao WBC doi mo-men vo ly.

    ros2 run g1_wbc wbc_shadow_node --ros-args -p network_interface:=wlp0s20f3
"""
import os
import signal
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from g1_leg_odometry.leg_odometry import LegOdometry, N_MOTOR
from g1_wbc.wbc import BalanceWBC

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    SDK_OK = True
except ImportError:
    SDK_OK = False


class ShadowNode(Node):
    def __init__(self):
        super().__init__("wbc_shadow_node")
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("payload_path", "")
        self.declare_parameter("network_interface", "")
        self.declare_parameter("enable_sdk", True)
        self.declare_parameter("lowstate_topic", "rt/lowstate")
        self.declare_parameter("csv_path", "")
        self.declare_parameter("rate_divider", 2)       # 1052Hz / 2 ~ 500Hz
        self.declare_parameter("settle_secs", 1.0)      # cho on dinh roi moi chot moc
        # --- he so, de quet tim nguon rung mo-men ---
        # Do tren robot that: mo-men CUA TA dao dong 1.217 Nm trong khi bo dieu
        # khien cua Unitree chi 0.076 Nm (gap 16 lan) tren robot DUNG YEN.
        # Mo phong dung dq hoan hao nen khong bao gio lo ra.
        self.declare_parameter("kp_q", 100.0)
        self.declare_parameter("kd_q", 20.0)
        self.declare_parameter("w_tau", 1.0)
        self.declare_parameter("kp_com", 60.0)
        self.declare_parameter("kd_com", 15.0)
        self.declare_parameter("kp_ori", 250.0)
        self.declare_parameter("kd_ang", 40.0)
        self.declare_parameter("dq_filter_hz", 0.0)     # 0 = khong loc
        # Quet tren robot that cho thay nguon rung KHONG phai tac vu tu the:
        #   goc        1.257 Nm
        #   kd_q   = 0 1.277  <- khong lien quan
        #   kd_ang = 0 0.870
        #   kd_com = 0 0.467  <- thu pham chinh
        # Tuc la nhieu VAN TOC THAN (6 mm/s) nhan kd_com roi qua canh tay don
        # tiep xuc thanh ~1 Nm. Dao dong thang bang chi 1-4 Hz nen loc thong
        # thap 15-25 Hz cat duoc phan lon nhieu ma tre pha khong dang ke.
        self.declare_parameter("vel_filter_hz", 0.0)    # loc v_body, 0 = khong loc

        urdf = self.get_parameter("urdf_path").value
        if not urdf:
            raise RuntimeError("phai dat urdf_path")
        pay = self.get_parameter("payload_path").value or None
        self.odom = LegOdometry(urdf)
        gp = lambda n: float(self.get_parameter(n).value)
        self.wbc = BalanceWBC(urdf, payload_yaml=pay,
                              kp_q=gp("kp_q"), kd_q=gp("kd_q"),
                              kp_com=gp("kp_com"), kd_com=gp("kd_com"),
                              kp_ori=gp("kp_ori"), kd_ang=gp("kd_ang"),
                              w_tau=gp("w_tau"))
        self.dq_fc = gp("dq_filter_hz")
        self.v_fc = gp("vel_filter_hz")
        self.dq_filt = np.zeros(N_MOTOR)
        self.v_filt = np.zeros(3)
        self._v_init = False
        self._t_prev = None
        self.get_logger().info(
            f"mo hinh WBC: {self.wbc.mass:.3f} kg"
            f"{' (da ap tai trong)' if pay else ' (URDF goc, CHUA ap tai trong)'}")

        self.pub = self.create_publisher(Float32MultiArray, "/wbc_shadow", 10)
        self._lock = threading.Lock()
        self.k = 0
        self.n_ok = self.n_fail = 0
        self.t_start = None
        self.ref_done = False
        self.solve_ms = []
        self.tau_hist = []
        self.te_hist = []
        self.rows = []
        self.last = None
        self.fail_msgs = {}

        if self.get_parameter("enable_sdk").value:
            if not SDK_OK:
                raise RuntimeError("khong import duoc unitree_sdk2py")
            iface = self.get_parameter("network_interface").value
            if iface:
                base = f"/sys/class/net/{iface}"
                if not os.path.isdir(base):
                    raise RuntimeError(f"khong co card mang '{iface}'")
                if open(f"{base}/carrier").read().strip() != "1":
                    raise RuntimeError(f"card '{iface}' khong co ket noi - kiem tra cap/wifi")
            ChannelFactoryInitialize(0, iface) if iface else ChannelFactoryInitialize(0)
            self.sub = ChannelSubscriber(self.get_parameter("lowstate_topic").value, LowState_)
            self.sub.Init(self.on_lowstate, 10)
            self.get_logger().info(f"nghe DDS rt/lowstate tren {iface or '(mac dinh)'}")
        else:
            self.get_logger().warn("enable_sdk=false: khong doc robot")

        self.create_timer(2.0, self.report)
        self.get_logger().warn(
            "CHE DO BONG - KHONG gui lenh nao den robot. "
            "Bo dieu khien cua Unitree van dang giu robot.")

    def on_lowstate(self, msg):
        self.k += 1
        if self.k % int(self.get_parameter("rate_divider").value):
            return
        ms = msg.motor_state
        q = np.fromiter((ms[i].q for i in range(N_MOTOR)), float, N_MOTOR)
        dq = np.fromiter((ms[i].dq for i in range(N_MOTOR)), float, N_MOTOR)
        tau_est = np.fromiter((ms[i].tau_est for i in range(N_MOTOR)), float, N_MOTOR)
        gyro = np.array(msg.imu_state.gyroscope, dtype=float)
        quat = np.array(msg.imu_state.quaternion, dtype=float)

        now = time.monotonic()
        if self.t_start is None:
            self.t_start = now
        # loc thong thap bac 1 cho van toc khop (0 = tat)
        if self.dq_fc > 0:
            dt = (now - self._t_prev) if self._t_prev else 0.002
            self._t_prev = now
            a = dt / (1.0 / (2 * np.pi * self.dq_fc) + dt)
            self.dq_filt += a * (dq - self.dq_filt)
            dq_ctrl = self.dq_filt.copy()
        else:
            dq_ctrl = dq
        try:
            out = self.odom.update(q, dq, gyro, tau_est, quat)
        except Exception as e:
            self.get_logger().error(f"leg odometry loi: {e}", throttle_duration_sec=5.0)
            return

        if not self.ref_done:
            if now - self.t_start < float(self.get_parameter("settle_secs").value):
                return
            self.wbc.capture_reference(q, quat)
            self.ref_done = True
            self.get_logger().info("da chot moc tham chieu (tu the / CoM / huong hien tai)")
            return

        v_body = out["v_body"]
        if self.v_fc > 0:
            if not self._v_init:
                self.v_filt[:] = v_body; self._v_init = True
            dtv = 0.002
            av = dtv / (1.0 / (2 * np.pi * self.v_fc) + dtv)
            self.v_filt += av * (v_body - self.v_filt)
            v_ctrl = self.v_filt.copy()
        else:
            v_ctrl = v_body
        contact = (out["contact"]["left"], out["contact"]["right"])
        try:
            t0 = time.perf_counter()
            tau, info = self.wbc.solve(q, dq_ctrl, quat, gyro, v_ctrl, contact)
            dt_ms = (time.perf_counter() - t0) * 1e3
        except Exception as e:
            with self._lock:
                self.n_fail += 1
                key = str(e)[:60]
                self.fail_msgs[key] = self.fail_msgs.get(key, 0) + 1
            return

        sat = np.abs(tau) > 0.98 * self.wbc.tau_hi
        m = Float32MultiArray()
        m.data = [float(x) for x in np.concatenate([tau, tau_est, info["com_err"]])]
        self.pub.publish(m)
        with self._lock:
            self.n_ok += 1
            self.solve_ms.append(dt_ms)
            self.last = (tau.copy(), tau_est.copy(), info, sat.sum(), contact)
            self.tau_hist.append(tau[:12].copy())
            self.te_hist.append(tau_est[:12].copy())
            if self.get_parameter("csv_path").value:
                self.rows.append(np.concatenate(
                    [[now - self.t_start], tau, tau_est, info["com_err"],
                     [info["fz_left"], info["fz_right"], float(contact[0]), float(contact[1]), dt_ms],
                     info["F_left"], info["F_right"],
                     out["wrench"]["left"][:3], out["wrench"]["right"][:3]]))

    def report(self):
        with self._lock:
            ok, fail, last = self.n_ok, self.n_fail, self.last
            ms = np.array(self.solve_ms[-2000:]) if self.solve_ms else None
            th = np.array(self.tau_hist[-1000:]) if len(self.tau_hist) > 50 else None
            te_h = np.array(self.te_hist[-1000:]) if len(self.te_hist) > 50 else None
            self.n_ok = self.n_fail = 0
        if last is None:
            self.get_logger().warn("chua co ket qua - da nhan lowstate chua?")
            return
        tau, tau_est, info, nsat, contact = last
        leg = slice(0, 12)
        self.get_logger().info(
            f"{ok/2:.0f} Hz | QP fail {fail} | giai {ms.mean():.2f}ms (p99 {np.percentile(ms,99):.2f}) | "
            f"CoM err {np.linalg.norm(info['com_err'])*1000:5.1f}mm | "
            f"|tau|max {np.abs(tau).max():5.1f} vs Unitree {np.abs(tau_est).max():5.1f} Nm | "
            f"chan cung dau {100*np.mean(np.sign(tau[leg])==np.sign(tau_est[leg])):3.0f}% | "
            f"RUNG ta {th.std(0).mean():.3f} vs Unitree {te_h.std(0).mean():.3f} Nm | "
            f"tiep xuc {int(contact[0])}{int(contact[1])}"
            if th is not None else f"tiep xuc {int(contact[0])}{int(contact[1])}")

    def final(self):
        print("\n" + "=" * 70)
        with self._lock:
            rows = np.array(self.rows) if self.rows else None
            ms = np.array(self.solve_ms)
            fails = dict(self.fail_msgs)
        if ms.size == 0:
            print("KHONG co chu ky nao giai thanh cong.")
        else:
            print(f"Giai thanh cong {ms.size} chu ky | that bai {sum(fails.values())}")
            print(f"Thoi gian giai: TB {ms.mean():.3f} ms | p99 {np.percentile(ms,99):.3f} | max {ms.max():.3f}")
            print(f"  ngan sach 500Hz = 2.0 ms -> dung {ms.mean()/2*100:.1f}% TB, {ms.max()/2*100:.1f}% xau nhat")
        for k, v in fails.items():
            print(f"  [QP FAIL x{v}] {k}")
        if rows is not None and len(rows) > 10:
            tau = rows[:, 1:30]; te = rows[:, 30:59]; ce = rows[:, 59:62]
            print(f"\nMO-MEN: WBC vs Unitree (12 khop chan)")
            names = ["L_hip_p", "L_hip_r", "L_hip_y", "L_knee", "L_ank_p", "L_ank_r",
                     "R_hip_p", "R_hip_r", "R_hip_y", "R_knee", "R_ank_p", "R_ank_r"]
            for i, nm in enumerate(names):
                a, b = tau[:, i], te[:, i]
                cc = np.corrcoef(a, b)[0, 1] if a.std() > 1e-9 and b.std() > 1e-9 else float("nan")
                print(f"  {nm:9s} WBC {a.mean():+7.2f}+-{a.std():5.2f} | "
                      f"Unitree {b.mean():+7.2f}+-{b.std():5.2f} Nm | tuong quan {cc:+.3f}")
            print(f"\nCoM err: {np.abs(ce).max(axis=0)*1000} mm (lon nhat moi truc)")
            p = self.get_parameter("csv_path").value
            if p:
                hdr = "t," + ",".join(f"tau{i}" for i in range(29)) + "," + \
                      ",".join(f"tauest{i}" for i in range(29)) + \
                      ",cex,cey,cez,fzl,fzr,cl,cr,ms" + \
                      ",qFlx,qFly,qFlz,qFrx,qFry,qFrz" + \
                      ",uFlx,uFly,uFlz,uFrx,uFry,uFrz"
                np.savetxt(p, rows, delimiter=",", header=hdr, comments="")
                print(f"\nCSV: {p}")
        print("=" * 70)


def main(args=None):
    rclpy.init(args=args)
    node = ShadowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        try:
            node.final()
        except Exception as e:
            print(f"loi khi dung bao cao: {e}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
