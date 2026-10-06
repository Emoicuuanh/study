"""Kiem tra duong ong ROS cua leg_odometry_node - KHONG can robot, KHONG can DDS.

Bom mot ban tin lowstate gia (duck-typed) thang vao on_lowstate(), roi kiem tra
co message ra dung tren /leg_odom va /leg_contact, dung dang nav_msgs/Odometry
ma estimator.cpp:143 UpdateLeg() doc.

Chay (sau khi source ROS + install):
    python3 G1/src/g1_leg_odometry/test/test_node_pipeline.py
"""
import sys
import types

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from g1_leg_odometry.leg_odometry_node import LegOdometryNode

import os
URDF = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "g1_description", "urdf", "g1_29dof.urdf"))
N = 29


def fake_lowstate(q, dq, tau, gyro, quat):
    ms = []
    for i in range(35):
        m = types.SimpleNamespace(q=0.0, dq=0.0, tau_est=0.0)
        if i < N:
            m.q, m.dq, m.tau_est = float(q[i]), float(dq[i]), float(tau[i])
        ms.append(m)
    return types.SimpleNamespace(
        motor_state=ms,
        imu_state=types.SimpleNamespace(gyroscope=list(gyro), quaternion=list(quat)))


def main():
    rclpy.init(args=["--ros-args", "-p", "enable_sdk:=false", "-p", f"urdf_path:={URDF}"])
    node = LegOdometryNode()

    got = {"odom": None, "contact": None}
    node.create_subscription(Odometry, "/leg_odom",
                             lambda m: got.__setitem__("odom", m), 10)
    node.create_subscription(Float32MultiArray, "/leg_contact",
                             lambda m: got.__setitem__("contact", m), 10)

    # tu the goi chung 0.3 rad, dung yen -> v_body phai ~ 0
    q = np.zeros(N)
    for base in (0, 6):
        q[base + 0], q[base + 3], q[base + 4] = -0.15, 0.3, -0.15
    tau = np.zeros(N)
    tau[3] = tau[9] = -5.6          # mo-men goi khi doi trong luong
    tau[0] = tau[6] = -0.65
    tau[4] = tau[10] = 1.2

    for _ in range(5):
        node.on_lowstate(fake_lowstate(q, np.zeros(N), tau, np.zeros(3), [1.0, 0, 0, 0]))
        rclpy.spin_once(node, timeout_sec=0.05)

    ok = True
    if got["odom"] is None:
        print("FAIL: khong co message tren /leg_odom"); ok = False
    else:
        v = got["odom"].twist.twist.linear
        print(f"  /leg_odom  twist.linear = [{v.x:+.4f} {v.y:+.4f} {v.z:+.4f}] m/s")
        print(f"             frame: {got['odom'].header.frame_id} -> {got['odom'].child_frame_id}")
        print(f"             covariance duong cheo = {[got['odom'].twist.covariance[i*6+i] for i in range(6)]}")
        if np.linalg.norm([v.x, v.y, v.z]) > 1e-6:
            print("FAIL: dung yen ma v_body != 0"); ok = False
    if got["contact"] is None:
        print("FAIL: khong co message tren /leg_contact"); ok = False
    else:
        d = got["contact"].data
        print(f"  /leg_contact  fz=[{d[0]:.1f} {d[1]:.1f}] N  tiep xuc=[{int(d[2])} {int(d[3])}]")
        if not (d[2] and d[3]):
            print("FAIL: dung hai chan ma khong bao tiep xuc"); ok = False

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{'PASS' if ok else 'FAIL'}: duong ong ROS")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
