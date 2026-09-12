"""Doc cau hinh DRIVE that su cua 12 khop chan trong g1_12dof.usd.

    cd isaac && ~/miniconda3/envs/isaaclab/bin/python -u tests/probe_drive.py

VI SAO CAN BAI NAY:
  Muon cho Isaac Sim dong vai driver dong co (giong robot that) thi phai
  biet drive dang duoc cau hinh the nao. Diem quyet dinh la DRIVE TYPE:

    type = "force"        stiffness tinh bang N*m/rad   -> giong MuJoCo,
                          giong kp gui xuong driver that
    type = "acceleration" stiffness tinh bang 1/s^2     -> Isaac tu nhan
                          them QUAN TINH cua khop vao
                              tau = I * (kp*(target-q) - kd*dq)
                          Cung mot so kp=150 se cho mo-men KHAC HAN,
                          lech theo ty le quan tinh cua tung khop.

  Trinh nhap URDF cua Isaac mac dinh dat "acceleration". Neu dung vay thi
  do la nguyen nhan lan truoc thu position drive robot SUP (0.80 -> 0.06 m)
  - khong phai vi position drive sai, ma vi don vi gain bi hieu khac.

  maxForce cung quan trong: neu no nho hon mo-men can thiet, drive bi cat
  bot lang le, khong bao loi.
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import sys
sys.path.insert(0, __file__.rsplit("/", 2)[0])

from scene.world import build_scene
from control.rl_walk import POLICY_JOINT_ORDER, KPS, KDS

G1_12DOF_USD = "/home/hungvd/study/g1_ws/isaac/assets/g1_12dof.usd"

world, robot = build_scene(add_dome_light=False, physics_dt=0.002,
                           rendering_dt=0.02, robot_usd=G1_12DOF_USD)
world.reset()

import omni.usd
from pxr import UsdPhysics, PhysxSchema

stage = omni.usd.get_context().get_stage()
names = list(robot.dof_names)
print(f"\n>>> so khop = {len(names)}")
print(f">>> ten khop: {names}\n")

print(f"{'khop':26s} {'type':13s} {'stiffness':>11s} {'damping':>9s} "
      f"{'maxForce':>10s}   {'kp mong':>8s} {'kd mong':>8s}")
print("-" * 96)

found = 0
for prim in stage.Traverse():
    d = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not d:
        continue
    nm = prim.GetName()
    if nm not in POLICY_JOINT_ORDER:
        continue
    i = POLICY_JOINT_ORDER.index(nm)
    found += 1
    print(f"{nm:26s} {str(d.GetTypeAttr().Get()):13s} "
          f"{str(d.GetStiffnessAttr().Get()):>11s} "
          f"{str(d.GetDampingAttr().Get()):>9s} "
          f"{str(d.GetMaxForceAttr().Get()):>10s}   "
          f"{KPS[i]:8.0f} {KDS[i]:8.0f}")

print("-" * 96)
print(f"tim thay {found}/12 khop chan co DriveAPI")

# gain thuc te ma Isaac dang dung (qua API cua Articulation)
import numpy as np
kp_now, kd_now = robot.get_gains()
print(f"\nget_gains() tra ve:")
print(f"  kps = {np.asarray(kp_now[0]).round(2)}")
print(f"  kds = {np.asarray(kd_now[0]).round(2)}")

# quan tinh tung khop - de uoc luong sai lech neu drive la 'acceleration'
try:
    m = robot.get_body_masses()
    print(f"\nkhoi luong cac khau: {np.asarray(m[0]).round(3)}")
    print(f"tong khoi luong    : {float(np.asarray(m[0]).sum()):.2f} kg")
except Exception as e:
    print(f"(khong doc duoc khoi luong: {e})")

app.close()
