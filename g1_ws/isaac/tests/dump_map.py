"""Xuat /map ra anh PNG va theo doi TF map->odom theo thoi gian.

    /usr/bin/python3 isaac/tests/dump_map.py <duong_dan_png> [so_giay]

VI SAO PHAI XEM ANH:
  Con so "map->odom lech 6m" mot minh khong noi duoc SLAM sai o dau. Hai
  truong hop cho ra cung con so:
    (a) ban do DUNG (cac vat can ro net) nhung frame map bi dat lech
    (b) ban do SAI  (vat can bi nhoe thanh cung tron, nhieu ban sao)
  Chi co nhin ban do moi phan biet duoc. Da hoc bai nay o vu camera: khi
  so do va quan sat mau thuan, hay tin quan sat truoc.

VI SAO THEO DOI map->odom THEO THOI GIAN:
  co dinh   -> chi la lech khoi tao, khong phai loi tich luy
  tang dan  -> scan matching dang PHAN KY, day moi la loi that
"""
import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/map.png"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0


class Dumper(Node):
    def __init__(self):
        super().__init__("map_dumper")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.create_subscription(
            OccupancyGrid, "/map", self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.last = None
        self.track = []

    def on_map(self, m):
        self.last = m

    def sample_tf(self):
        try:
            t = self.buf.lookup_transform("map", "odom", rclpy.time.Time()).transform
            q = t.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            self.track.append((t.translation.x, t.translation.y, math.degrees(yaw)))
        except Exception:
            pass


def main():
    rclpy.init()
    n = Dumper()
    t_end = SECS / 0.1
    i = 0
    while i < t_end:
        rclpy.spin_once(n, timeout_sec=0.1)
        i += 1
        if i % 10 == 0:
            n.sample_tf()

    if n.last is None:
        print("KHONG nhan duoc /map")
        rclpy.shutdown()
        return

    m = n.last
    g = np.asarray(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    # anh xam: chua biet = 128, trong = 255, chiem dung = 0
    img = np.full(g.shape, 128, dtype=np.uint8)
    img[g >= 0] = 255
    img[g >= 65] = 0
    img = np.flipud(img)                       # y cua ban do huong LEN
    try:
        from PIL import Image
        Image.fromarray(img).resize(
            (img.shape[1] * 2, img.shape[0] * 2), Image.NEAREST).save(OUT)
        print(f"da luu {OUT}  ({m.info.width}x{m.info.height} o)")
    except ImportError:
        pgm = OUT.rsplit(".", 1)[0] + ".pgm"
        with open(pgm, "wb") as f:
            f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
            f.write(img.tobytes())
        print(f"khong co PIL - da luu {pgm}")

    print("\nmap->odom theo thoi gian (moi 1 giay):")
    for k, (x, y, yw) in enumerate(n.track):
        print(f"  t+{k:2d}s  dx={x:+.3f} dy={y:+.3f}  yaw={yw:+7.2f} deg"
              f"  |d|={math.hypot(x, y):5.2f} m")
    if len(n.track) >= 2:
        d0 = math.hypot(*n.track[0][:2])
        d1 = math.hypot(*n.track[-1][:2])
        print(f"\n|d| dau {d0:.2f} m -> cuoi {d1:.2f} m"
              f"  ({'TANG - dang phan ky' if d1 > d0 + 0.05 else 'on dinh - chi la lech khoi tao'})")
    rclpy.shutdown()


main()
