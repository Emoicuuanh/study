"""TANG 0, Buoc C - Gan LiDAR 3D (Ouster OS0) len G1, doc du lieu point cloud.

Vi sao chon OS0:
  - FOV doc +-45 do (rong nhat trong cac model co san) -> nhin duoc CA
    mat san ngay truoc chan (can cho elevation map o tang 3) LAN vat can
    tren cao. G1 that dung Livox Mid-360 (doc -7 den +52 do) - tuong duong.
  - 360 do ngang, 128 tia, 10Hz, tam xa 75m

LiDAR RTX dung ray tracing tren GPU -> BAT BUOC render=True khi step.
"""
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path
from lidar_utils import create_lidar, read_scan
import omni.replicator.core as rep

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root = get_assets_root_path()
add_reference_to_stage(assets_root + "/Isaac/Robots/Unitree/G1/g1.usd", "/World/G1")

# --- Vai vat can de LiDAR co gi ma quet ---
for i, (px, py) in enumerate([(3.0, 0.0), (0.0, 3.0), (-3.0, 1.0), (2.0, -2.5)]):
    rep.create.cube(position=(px, py, 0.5), scale=(0.6, 0.6, 1.0), semantics=[("class", "obstacle")])

robot = Articulation(prim_paths_expr="/World/G1", name="g1")
world.scene.add(robot)

# --- LiDAR gan tren dau robot (~0.45m tren pelvis) ---
# create_lidar() da xu ly bug double-translation (xem lidar_utils.py)
lidar = create_lidar("/World/G1/pelvis/lidar", "os0", [0.0, 0.0, 0.45])
# THU TU QUAN TRONG: phai reset + initialize TRUOC roi moi attach annotator.
# (Goi attach truoc khi render product san sang -> crash trong omni.syntheticdata)
world.reset()
lidar.initialize()
lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer")

n = robot.num_dof
robot.set_gains(kps=np.full((1, n), 400.0), kds=np.full((1, n), 40.0))
robot.set_joint_position_targets(np.zeros((1, n)))

DT, BASE_HEIGHT = 1.0 / 60.0, 0.80
x, y, yaw = 0.0, 0.0, 0.0
vx, omega = 0.4, 0.3

print(">>> Bat dau quet (robot truot vong tron giua cac vat can)", flush=True)
so_frame_co_data = 0
for i in range(300):        # 5 giay
    yaw += omega * DT
    x += vx * np.cos(yaw) * DT
    y += vx * np.sin(yaw) * DT
    robot.set_world_poses(
        positions=np.array([[x, y, BASE_HEIGHT]]),
        orientations=np.array([[np.cos(yaw/2), 0.0, 0.0, np.sin(yaw/2)]]),
    )
    robot.set_velocities(np.zeros((1, 6)))

    world.step(render=True)     # BAT BUOC render=True cho LiDAR RTX

    arr = read_scan(lidar)
    if arr is not None:
        so_frame_co_data += 1
        if so_frame_co_data in (1, 20, 40):
            print(f">>> frame {i}: {arr.shape[0]} diem | z_san={np.median(arr[:,2]):.2f}m "
                  f"(mong doi -1.25 = do cao lidar tren san)", flush=True)

print(f">>> Tong so frame co du lieu: {so_frame_co_data}/300", flush=True)
print(">>> BUOC C THANH CONG" if so_frame_co_data > 10 else ">>> LOI: khong nhan duoc point cloud", flush=True)

app.close()
