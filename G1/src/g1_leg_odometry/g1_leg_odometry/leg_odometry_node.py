"""ROS 2 node: publish van toc than tu leg odometry, thay cho /dog_odom.

NODE NAY CHI DOC. No subscribe rt/lowstate va publish nav_msgs/Odometry.
No KHONG publish rt/lowcmd, KHONG goi LocoClient, KHONG gui bat cu lenh nao
den robot. Chay song song voi bo dieu khien co san cua Unitree la an toan -
day chinh la ly do no la buoc DAU TIEN chay tren robot that: cho phep kiem
chung uoc luong bang du lieu that TRUOC khi co bat ky mo-men nao do ta phat ra.

Ghep vao stack hien co (khong phai sua estimator.cpp):
    g1_state_estimator/config/estimator.yaml
        leg_topic: "/dog_odom"     ->     leg_topic: "/leg_odom"
estimator.cpp:143 UpdateLeg() chi doc msg->twist.twist.linear, dung dang
nav_msgs/Odometry ma node nay publish.

Chay:
    # co robot that
    ros2 run g1_leg_odometry leg_odometry_node --ros-args -p network_interface:=eth0
    # khong co robot (kiem tra duong ong ROS)
    ros2 run g1_leg_odometry leg_odometry_node --ros-args -p enable_sdk:=false
"""
import os
import signal
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from g1_leg_odometry.leg_odometry import LegOdometry, N_MOTOR

try:
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    SDK_OK = True
except ImportError:
    SDK_OK = False


class LegOdometryNode(Node):
    def __init__(self):
        super().__init__("leg_odometry_node")

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("network_interface", "")
        self.declare_parameter("enable_sdk", True)
        self.declare_parameter("output_topic", "/leg_odom")
        self.declare_parameter("contact_topic", "/leg_contact")
        self.declare_parameter("base_frame", "pelvis")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("vel_std", 0.02)          # m/s, do trong covariance
        self.declare_parameter("contact_on", 60.0)       # N
        self.declare_parameter("contact_off", 30.0)      # N
        self.declare_parameter("fusion", "force")        # "force" | "mean"
        self.declare_parameter("lowstate_topic", "rt/lowstate")
        # Cap ethernet chop -> CycloneDDS mat multicast va KHONG tu khoi phuc:
        # ca hai node cung mat du lieu vinh vien du robot van phat binh thuong
        # (da gap that: carrier_changes tang, 46/72s khong co du lieu).
        # Thoat han de launch respawn - tien trinh moi discovery lai tu dau.
        self.declare_parameter("watchdog_timeout", 5.0)   # s, 0 = tat

        urdf = self.get_parameter("urdf_path").value
        if not urdf:
            raise RuntimeError("phai dat tham so urdf_path (vd g1_29dof.urdf)")
        self.odom = LegOdometry(
            urdf,
            contact_on=self.get_parameter("contact_on").value,
            contact_off=self.get_parameter("contact_off").value,
            fusion=self.get_parameter("fusion").value,
        )
        self.base_frame = self.get_parameter("base_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        var = float(self.get_parameter("vel_std").value) ** 2

        self.pub = self.create_publisher(Odometry, self.get_parameter("output_topic").value, 10)
        self.pub_contact = self.create_publisher(
            Float32MultiArray, self.get_parameter("contact_topic").value, 10)

        self.cov = [0.0] * 36
        for i in range(3):
            self.cov[i * 6 + i] = var
        for i in range(3, 6):
            self.cov[i * 6 + i] = 1e6          # khong do van toc goc -> vo hieu

        self._lock = threading.Lock()
        self._n_msg = 0
        self._n_pub = 0
        self._last_rx = 0.0
        self._last_state = None

        if self.get_parameter("enable_sdk").value:
            if not SDK_OK:
                raise RuntimeError(
                    "khong import duoc unitree_sdk2py. Cai dat no, hoac chay voi "
                    "enable_sdk:=false de kiem tra duong ong ROS.")
            iface = self.get_parameter("network_interface").value
            problem = check_interface(iface)
            if problem:
                raise RuntimeError(problem)
            ChannelFactoryInitialize(0, iface) if iface else ChannelFactoryInitialize(0)
            self.sub = ChannelSubscriber(self.get_parameter("lowstate_topic").value, LowState_)
            self.sub.Init(self.on_lowstate, 10)
            self.get_logger().info(f"dang nghe DDS {self.get_parameter('lowstate_topic').value}"
                                   f" tren {iface or '(mac dinh)'}")
        else:
            self.get_logger().warn("enable_sdk=false: KHONG doc robot, chi kiem tra duong ong")

        self.create_timer(1.0, self.report)
        self.get_logger().info("leg_odometry_node san sang - CHI DOC, khong gui lenh nao den robot")

    # ---------------- DDS callback (chay o thread cua SDK, ~500Hz) ----------------
    def on_lowstate(self, msg):
        ms = msg.motor_state
        q = np.fromiter((ms[i].q for i in range(N_MOTOR)), float, N_MOTOR)
        dq = np.fromiter((ms[i].dq for i in range(N_MOTOR)), float, N_MOTOR)
        tau = np.fromiter((ms[i].tau_est for i in range(N_MOTOR)), float, N_MOTOR)
        gyro = np.array(msg.imu_state.gyroscope, dtype=float)
        quat = np.array(msg.imu_state.quaternion, dtype=float)      # w,x,y,z

        try:
            out = self.odom.update(q, dq, gyro, tau, quat)
        except Exception as e:                       # khong duoc lam chet thread DDS
            self.get_logger().error(f"leg odometry that bai: {e}", throttle_duration_sec=2.0)
            return

        now = self.get_clock().now().to_msg()
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = self.odom_frame
        od.child_frame_id = self.base_frame
        v = out["v_body"]
        od.twist.twist.linear.x = float(v[0])
        od.twist.twist.linear.y = float(v[1])
        od.twist.twist.linear.z = float(v[2])
        od.twist.covariance = self.cov
        # pose de trong CO Y: leg odometry khong quan sat duoc vi tri tuyet doi,
        # tich phan se troi. estimator.cpp UpdateLeg() chi doc twist.linear.
        self.pub.publish(od)

        c = Float32MultiArray()
        c.data = [float(out["fz"]["left"]), float(out["fz"]["right"]),
                  float(out["contact"]["left"]), float(out["contact"]["right"]),
                  *(float(x) for x in out["v_per_foot"]["left"]),
                  *(float(x) for x in out["v_per_foot"]["right"])]
        self.pub_contact.publish(c)

        with self._lock:
            self._n_msg += 1
            self._n_pub += 1
            self._last_rx = time.monotonic()
            self._last_state = out

    # ---------------- chan doan 1Hz ----------------
    def report(self):
        with self._lock:
            n, last, st = self._n_msg, self._last_rx, self._last_state
            self._n_msg = 0
        if n == 0:
            if self.get_parameter("enable_sdk").value:
                age = time.monotonic() - last if last else -1.0
                self.get_logger().warn(
                    f"khong nhan duoc lowstate ({age:.1f}s) - kiem tra network_interface"
                    if last else "chua nhan duoc lowstate nao - kiem tra network_interface")
                wd = float(self.get_parameter("watchdog_timeout").value)
                if wd > 0 and last and age > wd:
                    self.get_logger().fatal(
                        f"mat lowstate {age:.1f}s > watchdog {wd:.0f}s -> thoat de "
                        f"khoi tao lai DDS (kiem tra cap ethernet)")
                    os._exit(1)
            return
        v = st["v_body"]
        self.get_logger().info(
            f"{n} Hz | v_body=[{v[0]:+.3f} {v[1]:+.3f} {v[2]:+.3f}] m/s | "
            f"fz L={st['fz']['left']:6.1f} R={st['fz']['right']:6.1f} N | "
            f"tiep xuc L={int(st['contact']['left'])} R={int(st['contact']['right'])}")


def check_interface(name):
    """Kiem tra card mang truoc khi khoi tao DDS.

    unitree_sdk2py nem "channel factory init error" khi card khong ton tai hoac
    dang DOWN - thong bao do khong noi len duoc van de that (thuong la tuot cap).
    """
    import os
    if not name:
        return None
    base = f"/sys/class/net/{name}"
    if not os.path.isdir(base):
        return f"khong co card mang '{name}'. Cac card dang co: " + \
               ", ".join(sorted(os.listdir("/sys/class/net")))
    try:
        with open(f"{base}/operstate") as f:
            state = f.read().strip()
        with open(f"{base}/carrier") as f:
            carrier = f.read().strip()
    except OSError:
        return f"khong doc duoc trang thai cua '{name}'"
    if carrier != "1" or state != "up":
        return (f"card '{name}' dang {state} (carrier={carrier}) - "
                f"KIEM TRA CAP ETHERNET da cam chac chua")
    return None


def main(args=None):
    rclpy.init(args=args)
    node = LegOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
