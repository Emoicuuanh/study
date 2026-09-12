"""TANG 0, Buoc B - Cho G1 DUNG THANG va TRUOT theo lenh van toc (nhu AGV).

Vi sao truot ma khong di bo that:
  - Tang 0 (SLAM 3D) khong phu thuoc dang di - thuat toan ghep point cloud
    chi can biet cam bien o dau, khong quan tam chan lam gi
  - Tach bach de debug: neu map sai thi biet la loi SLAM, khong phai loi nga
  - Sang tang 2 (footstep planning) se BAT BUOC dung chan that

Cach lam: bien robot thanh KINEMATIC (vat ly khong keo no nga), roi tu
dat vi tri moi buoc theo van toc (vx, vy, omega) - giong cmd_vel cua AGV.
"""
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics, PhysxSchema
import omni.usd

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root = get_assets_root_path()
add_reference_to_stage(assets_root + "/Isaac/Robots/Unitree/G1/g1.usd", "/World/G1")

# --- Bien articulation thanh FIX BASE: vat ly khong lam no nga ---
stage = omni.usd.get_context().get_stage()
root_prim = stage.GetPrimAtPath("/World/G1")
for prim in [root_prim] + list(root_prim.GetChildren()):
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        art_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
        art_api.CreateArticulationEnabledAttr(True)
        print(f">>> Tim thay articulation root: {prim.GetPath()}", flush=True)
        break

robot = Articulation(prim_paths_expr="/World/G1", name="g1")
world.scene.add(robot)
world.reset()

# --- Tu the dung: dat muc tieu khop = 0 (tu the mac dinh cua USD) ---
n = robot.num_dof
robot.set_gains(kps=np.full((1, n), 400.0), kds=np.full((1, n), 40.0))
robot.set_joint_position_targets(np.zeros((1, n)))

# --- Dieu khien truot: cmd_vel (giong AGV) ---
DT = 1.0 / 60.0
BASE_HEIGHT = 0.80          # do cao than robot khi dung (m)
x, y, yaw = 0.0, 0.0, 0.0
max_err = 0.0
vx, vy, omega = 0.4, 0.0, 0.3    # m/s, m/s, rad/s

print(">>> Bat dau truot: vx=0.4 m/s, omega=0.3 rad/s (di vong tron)", flush=True)
for i in range(600):        # 10 giay
    # tich phan van toc -> vi tri (giong odometry cua AGV)
    yaw += omega * DT
    x += (vx * np.cos(yaw) - vy * np.sin(yaw)) * DT
    y += (vx * np.sin(yaw) + vy * np.cos(yaw)) * DT

    pos = np.array([[x, y, BASE_HEIGHT]])
    quat = np.array([[np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)]])   # (w,x,y,z)
    robot.set_world_poses(positions=pos, orientations=quat)
    robot.set_velocities(np.zeros((1, 6)))   # triet tieu van toc: khong cho trong luc keo lech

    world.step(render=False)

    p_now, _ = robot.get_world_poses()
    err = np.linalg.norm(np.array([x, y, BASE_HEIGHT]) - p_now[0])
    max_err = max(max_err, err)
    if i % 150 == 0:
        print(f">>> t={i*DT:4.1f}s  lenh=({x:+.2f},{y:+.2f})  thuc te=({p_now[0][0]:+.2f},{p_now[0][1]:+.2f},{p_now[0][2]:+.2f})  sai so={err*1000:.1f}mm", flush=True)

p, _ = robot.get_world_poses()
sai_so = np.linalg.norm(np.array([x, y, BASE_HEIGHT]) - p[0])
print(f">>> Sai so cuoi: {sai_so*1000:.1f} mm | Sai so LON NHAT trong 10s: {max_err*1000:.1f} mm", flush=True)
print(">>> BUOC B THANH CONG" if max_err < 0.02 else ">>> CAN CHINH: robot khong bam lenh", flush=True)

app.close()
