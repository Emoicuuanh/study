"""Kiem tra policy RL tren model G1 12 khop (dung model policy duoc train).

    python test_policy_12dof.py <vx>

So sanh voi model 43 khop cua Isaac Sim: policy duoc train tren G1 12 khop
(chi hai chan, 32.11 kg) nen chay tren model 43 khop (them eo + 2 tay +
ban tay) se troi 0.212 m/s du lenh bang 0. Model 12 khop nhap tu
unitree_rl_gym/resources/robots/g1_description/g1_12dof.urdf phai khop
dung va het troi.
"""
import sys
import numpy as np
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

sys.path.insert(0, "/home/hungvd/study/g1_ws/isaac")
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
import control.rl_walk as RW

PHYS = 0.002
STEPS = 5000                        # = 10 giay
VX = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
POLICY = "/home/hungvd/study/unitree_rl_gym/deploy/pre_train/g1/motion.pt"
USD = "/home/hungvd/study/g1_ws/isaac/assets/g1_12dof.usd"

world = World(stage_units_in_meters=1.0, physics_dt=PHYS, rendering_dt=PHYS * 10)
world.scene.add_default_ground_plane()
add_reference_to_stage(usd_path=USD, prim_path="/World/G1")
robot = Articulation(prim_paths_expr="/World/G1", name="g1")
world.scene.add(robot)
world.reset()
print(f">>> so khop = {robot.num_dof}", flush=True)

robot.set_world_poses(positions=np.array([[0.0, 0.0, 0.80]]),
                      orientations=np.array([[1.0, 0.0, 0.0, 0.0]]))
robot.set_velocities(np.zeros((1, 6)))

ctrl = RW.RLWalkController(robot, POLICY, loop_dt=PHYS)
print(f">>> chi so khop chan = {ctrl.leg_idx.tolist()}  decimation = {ctrl.decimation}", flush=True)
ctrl.setup_gains()
# KHONG cho robot "on dinh" thu dong: voi tu the mac dinh (goi gap 0.3 rad),
# PD mot minh khong du do trong luong -> robot sup TRUOC khi policy kip vao.
# Ban goc MuJoCo cung khong co giai doan nay - policy chay ngay tu buoc dau.
for _ in range(200):
    ctrl.step(cmd=(0.0, 0.0, 0.0))
    world.step(render=False)

p0, _ = robot.get_world_poses()
start = np.array(p0[0][:2])
print(f">>> sau khi on dinh: z = {p0[0][2]:.3f} m", flush=True)

for i in range(STEPS):
    ctrl.step(cmd=(VX, 0.0, 0.0))
    world.step(render=False)
    if i % 1000 == 0:
        p, _ = robot.get_world_poses()
        print(f">>>   t={i*PHYS:4.1f}s  z={p[0][2]:.3f}m  x={p[0][0]:+.2f}m", flush=True)

p, _ = robot.get_world_poses()
d = float(np.linalg.norm(np.array(p[0][:2]) - start))
print(f">>> lenh vx={VX}: di {d:.2f} m / {STEPS*PHYS:.0f}s = {d/(STEPS*PHYS):.3f} m/s"
      f"  z cuoi = {p[0][2]:.3f} m", flush=True)
if VX == 0.0:
    print(">>> " + ("DUNG YEN OK" if d / (STEPS * PHYS) < 0.05 and p[0][2] > 0.5
                    else f"VAN TROI ({d/(STEPS*PHYS):.3f} m/s)"), flush=True)
app.close()
