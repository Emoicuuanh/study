#!/usr/bin/python3
# shebang PHAI la /usr/bin/python3 - xem ghi chu trong stabilize_frame.py
"""Bo sung truong `time` va `ring` vao /points de FAST-LIO dung duoc.

    /points  (x,y,z)                -> /points_ts  (x,y,z,intensity,time,ring)

=== VAN DE ===
Isaac Sim phat PointCloud2 CHI CO x, y, z (do duoc: point_step = 12).
Nhung SLAM 3D kieu LIO can biet MOI DIEM duoc do vao THOI DIEM NAO trong
vong quet 100 ms, de go meo (de-skew) theo du lieu IMU. Khong co truong
thoi gian thi khong go meo duoc, va tren robot chan dieu do rat dat:
    do tan mat san  che do truot 41.9 mm  ->  che do di bo 58.5 mm

FAST-LIO co san duong du phong suy thoi gian tu goc phuong vi, NHUNG no
can truong `ring` de tach tung tia. Isaac cung khong cho `ring`. Nen ta
tu tinh ca hai.

=== SUY RA THOI GIAN: DA DO, KHONG DOAN ===
Da kiem tra thuc te bang isaac/tests/probe_scan_order.py:

    so diem       : 33593   (point_step = 12, chi x/y/z)
    goc doc       : -44.5 .. +13.7 do
    goc phuong vi : mot vong tron day du
    goc phuong vi (da bo cuon) tai 11 moc deu nhau:
        i=0      -144.8 do
        i=6718   -201.1 do
        i=16796  -298.1 do
        i=33592  -498.5 do
    tong bien thien: -353.7 do   => DUNG MOT VONG, chieu GIAM

Ket luan 1: mang xep theo THU TU QUET, cam bien quay theo chieu goc GIAM.

Ket luan 2 (quan trong hon): KHONG duoc dung chi so de suy thoi gian.
Cac doan 10% deu nhau ve chi so lai ung voi goc rat khac nhau:
    -31, -25, -27, -36, -34, -53, -33, -64, -22, -28 do
Ly do: nhieu tia khong tra ve gi (bay ra ngoai troi vi phong khong co
tran), va mang chi chua diem CO tra ve. Mat do diem theo goc khong deu.
Nhung cam bien QUAY DEU, nen thoi gian ti le voi GOC DA QUET, khong phai
voi so diem da doc. Dung chi so se sai toi 2 lan o mot so doan.

Cong thuc dung:
    delta_i = (az_0 - az_i) mod 360        (do, tang theo thoi gian)
    t_i     = delta_i / 360 * 0.1 s

=== RING ===
Suy tu goc doc: OS0 co 128 tia phu +-45 do.
    ring = clip(round((el + 45) / 90 * 127), 0, 127)
FAST-LIO khong dung ring khi da co `time`, nhung truong nay phai TON TAI
vi kieu diem velodyne_ros::Point dang ky ca 6 truong.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

# kieu diem khop CHINH XAC velodyne_ros::Point cua FAST-LIO:
#   PCL_ADD_POINT4D; float intensity; float time; uint16_t ring;
OUT_DTYPE = np.dtype([
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("intensity", "<f4"), ("time", "<f4"), ("ring", "<u2"),
])
OUT_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
    PointField(name="ring", offset=20, datatype=PointField.UINT16, count=1),
]

N_BEAMS = 128
EL_MIN, EL_MAX = -45.0, 45.0


class AddFields(Node):
    def __init__(self):
        super().__init__("lidar_add_fields")
        self.declare_parameter("in_topic", "/points")
        self.declare_parameter("out_topic", "/points_ts")
        self.declare_parameter("scan_period", 0.1)      # LiDAR 10 Hz
        # time_mode - de LAM THI NGHIEM DOI CHUNG, khong phai de dung lau dai:
        #   azimuth     t = (az0 - az) mod 360 / 360 * T   (chieu da do duoc)
        #   azimuth_rev t = (az - az0) mod 360 / 360 * T   (chieu NGUOC lai)
        #   none        t = 1e-6 cho MOI diem -> FAST-LIO khong go meo duoc
        # Muc dich: neu 'none' cho sai so nho hon 'azimuth' thi loi nam o
        # cach suy thoi gian, chu khong phai o khoi tao IMU. Khong doan duoc
        # dieu nay bang mat - phai do.
        self.declare_parameter("time_mode", "azimuth")
        # Isaac dat dau thoi gian o dau hay cuoi vong quet? Chua xac dinh
        # duoc chac. Tham so nay de dich dau thoi gian khi can hieu chinh;
        # 0.0 = coi nhu dau thoi gian la DAU vong quet.
        self.declare_parameter("stamp_shift", 0.0)

        self.period = float(self.get_parameter("scan_period").value)
        self.shift = float(self.get_parameter("stamp_shift").value)
        self.mode = str(self.get_parameter("time_mode").value)

        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter("out_topic").value,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            PointCloud2, self.get_parameter("in_topic").value, self.on_pc,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.n = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"{self.get_parameter('in_topic').value} -> "
            f"{self.get_parameter('out_topic').value}  "
            f"(them time + ring, time_mode={self.mode})")

    def on_pc(self, m):
        n = m.width * m.height
        if n == 0:
            return
        buf = np.frombuffer(m.data, dtype=np.uint8).reshape(n, m.point_step)
        xyz = buf[:, 0:12].copy().view(np.float32).reshape(n, 3)
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]

        az = np.degrees(np.arctan2(y, x))
        if self.mode == "none":
            # cung mot thoi diem cho moi diem -> FAST-LIO khong go meo.
            # Phai > 0, neu = 0 thi FAST-LIO coi la "khong co thoi gian" va
            # tu suy lai bang thuat toan rieng cua no (khong phai dieu ta muon
            # kiem chung).
            t = np.full(n, 1e-6, dtype=np.float32)
        else:
            # (az_0 - az) mod 360 -> tang dan theo thoi gian vi cam bien quay
            # theo chieu goc GIAM (da do, xem docstring)
            d = az[0] - az if self.mode == "azimuth" else az - az[0]
            t = (np.mod(d, 360.0) / 360.0 * self.period).astype(np.float32)

        el = np.degrees(np.arctan2(z, np.hypot(x, y)))
        ring = np.clip(
            np.rint((el - EL_MIN) / (EL_MAX - EL_MIN) * (N_BEAMS - 1)),
            0, N_BEAMS - 1).astype(np.uint16)

        out = np.empty(n, dtype=OUT_DTYPE)
        out["x"], out["y"], out["z"] = x, y, z
        out["intensity"] = 0.0
        out["time"] = t
        out["ring"] = ring

        msg = PointCloud2()
        msg.header = m.header
        if self.shift:
            total = (msg.header.stamp.sec * 1_000_000_000
                     + msg.header.stamp.nanosec + int(self.shift * 1e9))
            msg.header.stamp.sec = total // 1_000_000_000
            msg.header.stamp.nanosec = total % 1_000_000_000
        msg.height = 1
        msg.width = n
        msg.fields = OUT_FIELDS
        msg.is_bigendian = False
        msg.point_step = OUT_DTYPE.itemsize
        msg.row_step = OUT_DTYPE.itemsize * n
        msg.is_dense = True
        msg.data = out.tobytes()
        self.pub.publish(msg)

        self.n += 1
        self.last = (n, float(t.max()), int(ring.min()), int(ring.max()))

    def report(self):
        if self.n:
            n, tmax, r0, r1 = self.last
            self.get_logger().info(
                f"{self.n} vong quet | {n} diem | time toi da {tmax*1000:.1f} ms "
                f"| ring {r0}..{r1}")


def main():
    rclpy.init()
    node = AddFields()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()
