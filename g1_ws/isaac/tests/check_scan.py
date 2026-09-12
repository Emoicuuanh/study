"""Kham /scan: xem LaserScan co DUNG la mot vanh vat can, hay chi la MAT SAN.

    unset VIRTUAL_ENV PYTHONPATH; source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 isaac/tests/check_scan.py

VI SAO CAN BAI KIEM TRA NAY:
  Neu dai chieu cao cua pointcloud_to_laserscan cham vao MAT SAN thi moi tia
  deu tra ve mot khoang cach GAN NHU BANG NHAU -> LaserScan thanh mot VONG
  TRON. Vong tron thi doi xung xoay hoan toan: scan matching khong con thong
  tin gi ve goc quay, va no se tra ve goc bat ky. Trieu chung o tang tren la
  ban do bi xoay/bi nhoe, va TF map->odom lech rat lon.

  Cach phan biet:
    vat can that -> so tia huu han IT (vai %), khoang cach TAN MAC
    mat san      -> so tia huu han NHIEU (~100%), khoang cach TAP TRUNG
                    quanh mot gia tri gan range_max
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

N_SAMPLE = 20


class ScanCheck(Node):
    def __init__(self):
        super().__init__("scan_check")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos)
        self.rows = []

    def on_scan(self, m):
        if len(self.rows) >= N_SAMPLE:
            return
        r = np.asarray(m.ranges, dtype=np.float32)
        ok = np.isfinite(r) & (r > m.range_min) & (r < m.range_max * 0.999)
        self.rows.append((len(r), ok.sum(), r[ok]))

    def report(self):
        if not self.rows:
            print("KHONG nhan duoc /scan")
            return
        n_beam = self.rows[0][0]
        fr = np.array([c / n_beam * 100 for _, c, _ in self.rows])
        allr = np.concatenate([v for _, _, v in self.rows if v.size])
        print(f"so tia moi quet     : {n_beam}")
        print(f"ty le tia co vat    : {fr.mean():5.1f} %  (min {fr.min():.1f}, max {fr.max():.1f})")
        print(f"khoang cach: trung binh {allr.mean():.2f} m, do lech chuan {allr.std():.2f} m")
        print(f"             nho nhat  {allr.min():.2f} m, lon nhat {allr.max():.2f} m")
        h, edges = np.histogram(allr, bins=12)
        print("phan bo khoang cach:")
        for c, lo, hi in zip(h, edges[:-1], edges[1:]):
            print(f"  {lo:4.1f}-{hi:4.1f} m | {'#' * int(60 * c / h.max())} {c}")
        # ket luan tu dong
        if fr.mean() > 60 and allr.std() < 1.0:
            print("\n=> NGHI VAN: dang quet MAT SAN (nhieu tia + khoang cach tap trung)")
        else:
            print("\n=> Trong hinh dang cua vat can (it tia, khoang cach tan mac)")


def main():
    rclpy.init()
    n = ScanCheck()
    for _ in range(600):
        rclpy.spin_once(n, timeout_sec=0.1)
        if len(n.rows) >= N_SAMPLE:
            break
    n.report()
    rclpy.shutdown()


main()
