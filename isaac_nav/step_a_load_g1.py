"""TANG 0, Buoc A - Nap G1 vao Isaac Sim, chay vat ly, in thong tin co ban.

BOI CANH LO TRINH (5 tang cua humanoid navigation):
  Tang 4  Global path planning        <- giong AGV
  Tang 3  Elevation/traversability    <- MOI so voi AGV
  Tang 2  Footstep planning           <- MOI, khong co o AGV
  Tang 1  Locomotion (di/leo)         <- tuan 1-9 + RL
  Tang 0  SLAM 3D + localization      <- DANG O DAY

O tang 0-3-4, robot se TRUOT (nhu AGV) vi thuat toan SLAM/mapping khong
phu thuoc dang di. Sang tang 2-1 (footstep planning, leo cau thang) BAT
BUOC phai co chan that + dia hinh that - se doi cach lam luc do.

Chay:  ~/miniconda3/envs/isaaclab/bin/python step_a_load_g1.py
"""
from isaacsim import SimulationApp

# headless=True: khong mo cua so (nhanh, de debug). Doi False de xem 3D.
app = SimulationApp({"headless": True})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

assets_root = get_assets_root_path()
g1_usd = assets_root + "/Isaac/Robots/Unitree/G1/g1.usd"
print(f">>> Nap model tu: {g1_usd}", flush=True)

add_reference_to_stage(usd_path=g1_usd, prim_path="/World/G1")
robot = Articulation(prim_paths_expr="/World/G1", name="g1")
world.scene.add(robot)

world.reset()

print(f">>> So khop (DOF): {robot.num_dof}", flush=True)
print(f">>> 6 khop dau: {robot.dof_names[:6]}", flush=True)
pos, _ = robot.get_world_poses()
print(f">>> Vi tri ban dau: {pos[0]}", flush=True)

for _ in range(200):          # ~3.3 giay mo phong
    world.step(render=False)

pos, _ = robot.get_world_poses()
print(f">>> Vi tri sau 200 buoc: {pos[0]}", flush=True)
print(">>> BUOC A THANH CONG", flush=True)

app.close()
