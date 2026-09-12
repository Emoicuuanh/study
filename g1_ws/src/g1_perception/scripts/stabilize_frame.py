#!/usr/bin/python3
# CHU Y SHEBANG: PHAI la /usr/bin/python3, KHONG duoc dung /usr/bin/env python3.
# Tren may nay `python3` dau tien tren PATH la /home/hungvd/miniconda3 (3.13),
# con rclpy cua ROS2 Jazzy build cho 3.12. Dung env python3 thi node chet ngay:
#     ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
#     The C extension '.../_rclpy_pybind11.cpython-313-...so' isn't present
# Va vi node chet im lang trong launch, trieu chung thay duoc lai la o CHO KHAC:
#     pointcloud_to_laserscan: "discarding message because the queue is full"
# (thieu TF pelvis_stab -> moi quet bi xep hang roi bi bo).
"""Phat frame ON DINH THEO TRONG LUC: odom -> pelvis_stab

VAN DE NAY LA RIENG CUA ROBOT CHAN - xe AGV khong gap.

pointcloud_to_laserscan lay mot DAI CHIEU CAO trong frame dich roi ep xuong
mat phang. Neu frame dich la `pelvis` thi:

    robot dung yen          robot dang di bo (pelvis nghieng ~3 do)
    ------------------      --------------------------------------
    dai cat NGANG           dai cat NGHIENG THEO
    san luon o -0.777       san o xa BI KEO VAO trong dai

Cu the: san cao -0.777 m so voi pelvis. Voi dai bat dau tu -0.55 thi can
nghieng bao nhieu de san "loi" vao dai o khoang cach r?
        r * sin(theta) > 0.227
    r = 6 m  ->  theta > 2.17 do    <- robot di bo nghieng HON the nay
    r = 4 m  ->  theta > 3.25 do

HAU QUA DA DO DUOC (xem anh ban do trong bai):
  - mot VANH DEN TRON o ban kinh ~ range_max: do la mat san
  - vanh tron DOI XUNG XOAY hoan toan -> scan matching khong con rang buoc
    ve goc quay -> tu the troi -> 7 khoi vat can bi ve thanh ~20 ban sao
  - TF map->odom lech 6.1 m roi 9.3 m (le ra phai gan 0 vi odom cua Isaac
    la ground truth)

CACH SUA: tao mot frame moi cung VI TRI voi pelvis nhung BO ROLL va PITCH,
chi giu YAW. Dai chieu cao trong frame nay luon thuc su nam ngang, nen san
o -0.777 m khong bao gio loi vao dai du robot lac the nao.

Tren robot THAT, roll/pitch nay den tu bo loc IMU (state estimator). O day
ta lay tu /odom cua Isaac cho don gian. Chinh y tuong "on dinh theo trong
luc" nay se quay lai o CHANG B duoi dang de-skew bang IMU cua FAST-LIO.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Stabilizer(Node):
    def __init__(self):
        super().__init__("stabilize_frame")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "pelvis_stab")

        self.parent = self.get_parameter("parent_frame").value
        self.child = self.get_parameter("child_frame").value

        self.br = TransformBroadcaster(self)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.on_odom,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.n = 0
        self.max_tilt = 0.0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"phat {self.parent} -> {self.child} (bo roll/pitch)")

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        # yaw giu lai, roll/pitch bo di
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        # do lai do nghieng that de bao cao - day la con so bien minh cho
        # su ton tai cua node nay
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                          1 - 2 * (q.x * q.x + q.y * q.y))
        sp = max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sp)
        self.max_tilt = max(self.max_tilt, math.hypot(roll, pitch))
        self.n += 1

        t = TransformStamped()
        t.header.stamp = msg.header.stamp      # GIU NGUYEN dau thoi gian cua
        t.header.frame_id = self.parent        # /odom - dat lai bang now() se
        t.child_frame_id = self.child          # lam TF lech pha voi du lieu
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.br.sendTransform(t)

    def report(self):
        if self.n:
            self.get_logger().info(
                f"{self.n} khung, do nghieng pelvis lon nhat "
                f"{math.degrees(self.max_tilt):.2f} do")


def main():
    rclpy.init()
    n = Stabilizer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
