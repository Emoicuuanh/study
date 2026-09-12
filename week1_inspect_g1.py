"""Tuan 1 - Script dau tien: load Unitree G1 va doc thong tin co ban."""
import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene.xml")
data = mujoco.MjData(model)

print(f"So bac tu do (nv):        {model.nv}")
print(f"So bien vi tri (nq):      {model.nq}")
print(f"So actuator (dong co):    {model.nu}")
print(f"Khoi luong robot:         {sum(model.body_mass):.2f} kg")
print(f"Timestep mo phong:        {model.opt.timestep*1000:.1f} ms")

print("\nDanh sach khop (joint):")
for i in range(model.njnt):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    jtype = ["free", "ball", "slide", "hinge"][model.jnt_type[i]]
    print(f"  {i:2d}  {name:35s} {jtype}")

# Chay thu 1000 buoc mo phong (robot roi tu do vi chua co controller)
mujoco.mj_resetDataKeyframe(model, data, 0)  # tu the dung san co trong model
for _ in range(1000):
    mujoco.mj_step(model, data)
print(f"\nMo phong 1000 buoc OK. Chieu cao thân robot sau 2s: {data.qpos[2]:.3f} m")
