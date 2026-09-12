"""Kiem tra /points cua Isaac Sim co xep diem THEO THU TU QUET khong.

    /usr/bin/python3 isaac/tests/probe_scan_order.py

VI SAO PHAI KIEM TRA:
  FAST-LIO can biet MOI DIEM duoc do vao LUC NAO trong vong quet 100 ms
  de go meo. Isaac Sim chi phat x,y,z - khong co truong thoi gian.
  Co hai cach suy ra thoi gian, va chung KHAC NHAU:
    (1) theo CHI SO diem: t = i/N * 0.1s   - dung NEU mang xep theo thu tu do
    (2) theo GOC PHUONG VI: t = (az0 - az)/360 * 0.1s - can biet CHIEU QUAY
  Chon sai chieu quay o cach (2) thi go meo se ap nguoc dau, lam SAI HON la
  khong go. Nen phai do truoc, khong duoc doan.

Bai kiem tra: neu mang xep theo thu tu quet thi goc phuong vi phai BIEN
THIEN DON DIEU theo chi so (cho phep mot lan nhay khi vong qua 360 do).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Probe(Node):
    def __init__(self):
        super().__init__("probe_scan_order")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.create_subscription(
            PointCloud2, "/points", self.on_pc,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.done = False

    def on_pc(self, m):
        if self.done:
            return
        self.done = True
        n = m.width * m.height
        buf = np.frombuffer(m.data, dtype=np.uint8).reshape(n, m.point_step)
        xyz = buf[:, 0:12].copy().view(np.float32).reshape(n, 3)
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        az = np.degrees(np.arctan2(y, x))               # -180..180
        el = np.degrees(np.arctan2(z, np.hypot(x, y)))
        print(f"so diem       : {n}  (point_step={m.point_step})")
        print(f"goc doc (el)  : {el.min():+.1f} .. {el.max():+.1f} do")
        print(f"goc ngang (az): {az.min():+.1f} .. {az.max():+.1f} do")

        # === thu tu quet? ===
        azu = np.unwrap(np.radians(az))
        d = np.diff(azu)
        pos = (d > 0).mean() * 100
        print(f"\nbien thien az theo chi so: {pos:.1f}% buoc TANG"
              f"  (trung binh {np.degrees(d).mean():+.4f} do/diem)")
        if pos > 95:
            print("=> mang XEP THEO THU TU QUET, chieu NGUOC kim dong ho (az tang)")
        elif pos < 5:
            print("=> mang XEP THEO THU TU QUET, chieu THEO kim dong ho (az giam)")
        else:
            print("=> mang KHONG xep theo goc phuong vi - kiem tra theo tia (ring)")

        # === KIEM TRA O MUC THO ===
        # Trong mot cot doc, 128 tia co goc phuong vi lech nhau chut it, nen
        # so sanh tung diem lien tiep bi nhieu. Nhung neu mang xep theo thu tu
        # quet thi o MUC THO (chia 10 doan) goc phai giam/tang deu ~36 do moi
        # doan. Day la bai kiem tra dang tin.
        print("\ngoc phuong vi (da bo cuon) tai 11 moc deu nhau:")
        idx = np.linspace(0, n - 1, 11).astype(int)
        vals = np.degrees(azu[np.minimum(idx, len(azu) - 1)])
        for i, v in zip(idx, vals):
            print(f"  i={i:6d}  az={v:+8.1f} do")
        step = np.diff(vals)
        mono = (step < 0).all() or (step > 0).all()
        print(f"tong bien thien: {vals[-1] - vals[0]:+.1f} do"
              f"  (mot vong = +-360)")
        print(f"=> {'DON DIEU o muc tho - xep theo thu tu quet' if mono else 'KHONG don dieu'}"
              f", chieu {'GIAM (theo kim dong ho)' if step.mean() < 0 else 'TANG'}")

        # === xep theo tia truoc hay theo goc truoc? ===
        # OS0 128 tia. Neu xep theo COT (moi goc mot lan quet du 128 tia) thi
        # 128 diem dau phai co az gan nhu bang nhau va el trai deu.
        k = 128
        print(f"\n128 diem dau : az {az[:k].min():+.2f}..{az[:k].max():+.2f}"
              f"   el {el[:k].min():+.1f}..{el[:k].max():+.1f}")
        print(f"128 diem sau : az {az[k:2*k].min():+.2f}..{az[k:2*k].max():+.2f}"
              f"   el {el[k:2*k].min():+.1f}..{el[k:2*k].max():+.1f}")
        if az[:k].ptp() < 2.0 and el[:k].ptp() > 40:
            print("=> xep THEO COT: moi 128 diem la mot cot doc cung goc ngang")
        else:
            print("=> KHONG xep theo cot 128 - can xem lai gia thiet")


def main():
    rclpy.init()
    n = Probe()
    for _ in range(400):
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.done:
            break
    if not n.done:
        print("khong nhan duoc /points")
    rclpy.shutdown()


main()
