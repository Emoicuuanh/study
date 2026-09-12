#!/usr/bin/python3
# shebang PHAI la /usr/bin/python3 - xem ghi chu trong stabilize_frame.py
"""Neo FAST-LIO vao cay TF va DO TROI so voi ground truth.

Lam hai viec:

1. PHAT TF TINH  odom -> camera_init
   FAST-LIO dung hai frame co dinh trong ma nguon:
       camera_init = he the gioi rieng cua no, neo tai tu the DAU TIEN
       body        = he than robot (thuc chat la he IMU)
   Hai frame nay khong noi voi cay TF cua Isaac (odom -> pelvis -> imu),
   nen RViz khong ve duoc chung cung mot cho. Ta lay tu the odom->imu o
   THOI DIEM DAU rồi phat lam TF tinh odom -> camera_init.
   Sau do `body` (uoc luong) va `imu` (that) nam cung he, xe lech bao nhieu
   la thay ngay bang mat.

2. DO SAI SO
   Trong Isaac Sim, /odom la GROUND TRUTH (lay truc tiep tu vat ly), khong
   he troi. Day la loi the lon cua mo phong: co the do CHINH XAC sai so cua
   SLAM - tren robot that thi khong biet dau la dung.
       sai so = | vi tri(odom->imu)  -  vi tri(odom->camera_init->body) |
   In ra theo thoi gian de thay sai so co TICH LUY hay khong.

VI SAO CAN BAI DO NAY:
   O chang A da hoc bai: ban do "trong co ve duoc" nhung TF map->odom lech
   9.3 m, va 7 khoi vat can bi ve thanh ~20 ban sao. Anh ban do mot minh de
   gay nham. Con so sai so thi khong.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


class LioEval(Node):
    def __init__(self):
        super().__init__("lio_eval")
        self.declare_parameter("truth_frame", "imu")     # frame that cua IMU
        self.declare_parameter("est_frame", "body")      # frame FAST-LIO uoc luong
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("lio_world", "camera_init")
        self.declare_parameter("report_period", 10.0)

        self.truth = self.get_parameter("truth_frame").value
        self.est = self.get_parameter("est_frame").value
        self.odom = self.get_parameter("odom_frame").value
        self.lio_w = self.get_parameter("lio_world").value

        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.static_br = StaticTransformBroadcaster(self)
        self.anchored = False
        self.t0 = None
        self.worst = 0.0

        # /odom chi dung de biet mo phong da chay - moc neo lay tu TF
        self.create_subscription(
            Odometry, "/odom", self.on_odom,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(1.0, self.try_anchor)
        self.create_timer(float(self.get_parameter("report_period").value),
                          self.report)
        self.have_odom = False

    def on_odom(self, _msg):
        self.have_odom = True

    def try_anchor(self):
        """Phat TF tinh odom -> camera_init, mot lan duy nhat."""
        if self.anchored or not self.have_odom:
            return
        try:
            tr = self.buf.lookup_transform(
                self.odom, self.truth, rclpy.time.Time()).transform
        except Exception:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom
        t.child_frame_id = self.lio_w
        t.transform = tr
        self.static_br.sendTransform(t)
        self.anchored = True
        self.get_logger().info(
            f"da neo {self.odom} -> {self.lio_w} tai "
            f"({tr.translation.x:+.3f}, {tr.translation.y:+.3f}, {tr.translation.z:+.3f})")

    def report(self):
        if not self.anchored:
            self.get_logger().info("chua neo duoc - cho TF odom->imu va /odom")
            return
        try:
            a = self.buf.lookup_transform(
                self.odom, self.truth, rclpy.time.Time()).transform.translation
            b = self.buf.lookup_transform(
                self.odom, self.est, rclpy.time.Time()).transform.translation
        except Exception as e:
            self.get_logger().info(f"chua doc duoc TF: {e}")
            return
        d = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        self.worst = max(self.worst, d)
        self.get_logger().info(
            f"that=({a.x:+.2f},{a.y:+.2f},{a.z:+.2f})  "
            f"LIO=({b.x:+.2f},{b.y:+.2f},{b.z:+.2f})  "
            f"sai so={d*100:6.1f} cm  (lon nhat {self.worst*100:.1f} cm)")


def main():
    rclpy.init()
    n = LioEval()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
