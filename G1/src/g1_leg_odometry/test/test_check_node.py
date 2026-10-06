"""Kiem chung PHAN TOAN cua leg_odom_check_node bang du lieu tong hop.

Cong cu do chi co ich neu con so no in ra dung - con so do se duoc chep thang
vao vel_std va leg_vel_noise. Test nay bom nhieu DA BIET vao roi kiem tra no
khoi phuc lai dung.
"""
import io
import re
import sys
from contextlib import redirect_stdout

import numpy as np
import rclpy
from nav_msgs.msg import Odometry

from g1_leg_odometry.leg_odom_check_node import Check

TRUE_STD = np.array([0.015, 0.012, 0.004])      # m/s
TRUE_BIAS = np.array([0.003, -0.002, 0.001])
N_STATIC = 4000
N_MOVING = 1000


def odom(v):
    m = Odometry()
    m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.linear.z = map(float, v)
    return m


def main():
    rclpy.init(args=["--ros-args", "-p", "static_speed:=0.05"])
    node = Check()
    rng = np.random.default_rng(1)

    # --- pha 1: robot dung yen, van toc that = 0 + bias + nhieu ---
    for _ in range(N_STATIC):
        node.on_leg(odom(TRUE_BIAS + rng.normal(0, TRUE_STD)))

    # --- pha 2: dang di chuyen, nguon doi chieu lech mot luong da biet ---
    REF_OFFSET = np.array([0.02, 0.0, 0.0])
    for k in range(N_MOVING):
        v_true = np.array([0.3 * np.sin(k * 0.01), 0.1, 0.0])
        node.on_ref_ros(odom(v_true))
        node.on_leg(odom(v_true + REF_OFFSET + rng.normal(0, TRUE_STD)))

    buf = io.StringIO()
    with redirect_stdout(buf):
        node.final()
    out = buf.getvalue()
    print(out)

    node.destroy_node()
    rclpy.shutdown()

    # --- kiem tra ---
    ok = True

    def check(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok &= good
        print(f"  {'ok ' if good else 'SAI'} {name}: {got:.2f} (mong doi {want:.2f} +-{tol})")

    print("Kiem tra:")
    for i, ax in enumerate("xyz"):
        line = [l for l in out.splitlines() if l.strip().startswith(f"{ax}: trung binh")][0]
        mean = float(line.split("trung binh")[1].split("mm/s")[0])
        std = float(line.split("do lech chuan")[1].split("mm/s")[0])
        check(f"nen nhieu {ax} do lech chuan", std, TRUE_STD[i] * 1000, 1.5)
        check(f"nen nhieu {ax} do lech he thong", mean, TRUE_BIAS[i] * 1000, 1.5)

    rec = float(re.search(r"vel_std\s*=\s*([0-9.]+)", out).group(1))
    want_rec = np.ceil(TRUE_STD.max() * 1.5 * 1000) / 1000
    check("vel_std de xuat", rec, want_rec, 0.002)

    dx = float(re.search(r"x: lech trung binh\s*([+-][0-9.]+)", out).group(1))
    check("lech so voi nguon doi chieu (x)", dx, REF_OFFSET[0] * 1000, 2.0)

    print(f"\n{'PASS' if ok else 'FAIL'}: phan toan cua cong cu do")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
