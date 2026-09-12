"""Tuan 3 - Bai tap 2: vua squat vua VAY TAY.

Them so voi week3_squat.py:
- Tra them index 2 khop shoulder_pitch bang aid()
- Tinh them wave = A_arm * sin(2*pi*f_arm*t)
- De thu vi: 2 tay vay NGUOC PHA nhau (trai truoc thi phai sau)
  nhu dong tac chay bo -> chi can dao dau: trai -wave, phai +wave

Quan sat thu: wave dung sin nen tai t=0 van toc muc tieu != 0
=> tay bi "nay" nhe luc khoi dong (khac voi squat dung (1-cos)/2
khoi dong em). Day chinh la ly do bai truoc chon (1-cos)/2!
"""
import time
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
q_ref = data.qpos[7:].copy()

def aid(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

joints = {n: aid(n) for n in [
    "left_hip_pitch_joint",  "right_hip_pitch_joint",
    "left_knee_joint",       "right_knee_joint",
    "left_ankle_pitch_joint","right_ankle_pitch_joint",
    "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",   # <-- MOI
]}
print("Chi so actuator:", joints)

# Chan (squat)
A = 0.8
f = 0.4
# Tay (vay)
A_arm = 0.7   # bien do vay (rad)
f_arm = 1.4   # tay vay nhanh gap doi nhip squat

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        t = data.time

        theta = A * (1 - np.cos(2*np.pi*f*t)) / 2       # nhip squat
        wave  = A_arm * np.sin(2*np.pi*f_arm*t)         # nhip vay tay  <-- MOI

        ctrl = q_ref.copy()
        # --- chan: squat (nhu bai truoc) ---
        ctrl[joints["left_hip_pitch_joint"]]    -= theta
        ctrl[joints["right_hip_pitch_joint"]]   -= theta
        ctrl[joints["left_knee_joint"]]         += 2*theta
        ctrl[joints["right_knee_joint"]]        += 2*theta
        ctrl[joints["left_ankle_pitch_joint"]]  -= theta
        ctrl[joints["right_ankle_pitch_joint"]] -= theta
        # --- tay: vay nguoc pha nhau (nhu chay bo) ---   <-- MOI
        ctrl[joints["left_shoulder_pitch_joint"]]  -= wave
        ctrl[joints["right_shoulder_pitch_joint"]] += wave
        data.ctrl[:] = ctrl

        mujoco.mj_step(model, data)
        viewer.sync()

        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
