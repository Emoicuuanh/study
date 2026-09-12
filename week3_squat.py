"""Tuan 3 - Squat: tu the muc tieu THAY DOI THEO THOI GIAN.

Y tuong: van la position control nhu bai truoc, nhung thay vi
ctrl = q_ref (hang so), ta cap nhat ctrl = q_ref(t) moi vong lap.

Quy tac hinh hoc de squat ma ban chan phang + than thang dung:
    hip_pitch  = -theta
    knee       = +2*theta
    ankle_pitch= -theta
(tong -theta + 2theta - theta = 0 => than // ban chan)

theta(t) = A * (1 - cos(2*pi*f*t)) / 2
  - bat dau tu 0, tang dan => khong bi giat luc khoi dong
  - A = do sau squat (rad), f = tan so (lan squat / giay)
"""
import time
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene.xml")
data = mujoco.MjData(model)

mujoco.mj_resetDataKeyframe(model, data, 0)
q_ref = data.qpos[7:].copy()          # tu the dung goc (29 goc khop)

# Tim chi so actuator cua 6 khop chan can dieu khien (theo ten)
def aid(name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

joints = {n: aid(n) for n in [
    "left_hip_pitch_joint",  "right_hip_pitch_joint",
    "left_knee_joint",       "right_knee_joint",
    "left_ankle_pitch_joint","right_ankle_pitch_joint",
]}
print("Chi so actuator:", joints)

A = 0.8   # do sau squat (rad) ~ goi gap toi da 1.6 rad ~ 92 do
f = 0.4   # 0.4 lan squat moi giay (1 nhip ~ 2.5s)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        t = data.time                              # thoi gian mo phong (s)

        theta = A * (1 - np.cos(2*np.pi*f*t)) / 2  # 0 -> A -> 0 -> A ...

        ctrl = q_ref.copy()                        # bat dau tu tu the dung
        ctrl[joints["left_hip_pitch_joint"]]   = q_ref[joints["left_hip_pitch_joint"]]   - theta
        ctrl[joints["right_hip_pitch_joint"]]  = q_ref[joints["right_hip_pitch_joint"]]  - theta
        ctrl[joints["left_knee_joint"]]        = q_ref[joints["left_knee_joint"]]        + 2*theta
        ctrl[joints["right_knee_joint"]]       = q_ref[joints["right_knee_joint"]]       + 2*theta
        ctrl[joints["left_ankle_pitch_joint"]] = q_ref[joints["left_ankle_pitch_joint"]] - theta
        ctrl[joints["right_ankle_pitch_joint"]]= q_ref[joints["right_ankle_pitch_joint"]]- theta
        data.ctrl[:] = ctrl

        mujoco.mj_step(model, data)
        viewer.sync()

        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
