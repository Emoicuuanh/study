"""Do ban do /map va do lech map->odom. Chay bang python3 he thong:

    unset VIRTUAL_ENV PYTHONPATH; source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 isaac/tests/check_map.py

In ra:
  - kich thuoc / do phan giai / goc ban do
  - so o CHIEM DUNG, TRONG, CHUA BIET  (o luoi chiem dung: 100 / 0 / -1)
  - do lech cua TF map->odom = luong sai so tich luy ma SLAM da chinh lai.
    Neu no LUON bang 0 thi SLAM khong he sua gi (dang chi tin odometry).
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener


class MapCheck(Node):
    def __init__(self):
        super().__init__("map_check")
        self.set_parameters([rclpy.parameter.Parameter(
            "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        # /map dung Transient Local - phai khop, khong thi khong nhan duoc gi
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.buf = Buffer()
        self.tfl = TransformListener(self.buf, self)
        self.got = False

    def on_map(self, m):
        if self.got:
            return
        self.got = True
        d = m.data
        occ = sum(1 for v in d if v >= 65)
        free = sum(1 for v in d if 0 <= v < 25)
        unk = sum(1 for v in d if v < 0)
        res, w, h = m.info.resolution, m.info.width, m.info.height
        print(f"ban do   : {w} x {h} o, do phan giai {res:.3f} m/o"
              f"  ({w*res:.1f} x {h*res:.1f} m)")
        print(f"goc (map): ({m.info.origin.position.x:+.2f}, {m.info.origin.position.y:+.2f})")
        print(f"chiem dung: {occ:6d} o = {occ*res*res:7.2f} m2")
        print(f"trong     : {free:6d} o = {free*res*res:7.2f} m2  <- vung robot da di qua")
        print(f"chua biet : {unk:6d} o")

        try:
            t = self.buf.lookup_transform("map", "odom", rclpy.time.Time()).transform
            dx, dy = t.translation.x, t.translation.y
            q = t.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            print(f"map->odom : dx={dx:+.3f} dy={dy:+.3f} m, yaw={math.degrees(yaw):+.2f} deg"
                  f"  |sai so da chinh| = {math.hypot(dx, dy)*100:.1f} cm")
        except Exception as e:
            print(f"map->odom : CHUA CO ({e})")


def main():
    rclpy.init()
    n = MapCheck()
    for _ in range(400):                 # toi da ~40s
        rclpy.spin_once(n, timeout_sec=0.1)
        if n.got and n.buf.can_transform("map", "odom", rclpy.time.Time()):
            break
    if not n.got:
        print("KHONG nhan duoc /map - kiem tra slam_toolbox da 'active' chua")
    rclpy.shutdown()


main()
