"""So sanh BUG vs FIX cua week4_nullspace.py tren model G1 THAT, khong mo viewer.

Chay:  ~/miniconda3/envs/g1-real/bin/python week4_nullspace_ab.py
"""
import mujoco, numpy as np

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene_target.xml")
ARM_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",   "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
jid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
aid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
arm_dofs = [model.jnt_dofadr[jid(n)] for n in ARM_JOINTS]
arm_acts = [aid(n) for n in ARM_JOINTS]
hand_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")

LAMBDA, GAIN, MAX_STEP = 0.05, 0.10, 0.003
SECONDARY_AMP  = np.array([0, 0, 0.8, 0.4, 0, 0, 0])
SECONDARY_FREQ, NULL_GAIN = 0.25, 0.05

def run(BUG, T=16.0):
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    q_ref = data.qpos[7:].copy()
    q_des_arm = np.array([q_ref[a] for a in arm_acts])
    q_base = q_des_arm.copy()
    lo, hi = model.actuator_ctrlrange[arm_acts, 0], model.actuator_ctrlrange[arm_acts, 1]
    errs, sat, elbow, n = [], 0, [], 0
    while data.time < T:
        t = data.time
        data.mocap_pos[0] = [0.30, -0.20, 0.85]
        err = data.mocap_pos[0] - data.body(hand_id).xpos
        jacp = np.zeros((3, model.nv)); mujoco.mj_jacBody(model, data, jacp, None, hand_id)
        J = jacp[:, arm_dofs]
        A = J @ J.T + LAMBDA**2*np.eye(3)
        dq_ik = J.T @ np.linalg.solve(A, err)
        N = np.eye(7) - (J.T @ np.linalg.solve(A, np.eye(3))) @ J
        wave = SECONDARY_AMP * np.sin(2*np.pi*SECONDARY_FREQ*t)
        dq_null = N @ wave if BUG else N @ (NULL_GAIN * ((q_base + wave) - q_des_arm))
        dq = GAIN*dq_ik + dq_null
        sat += bool(np.any(np.abs(dq) > MAX_STEP)); n += 1
        q_des_arm = np.clip(q_des_arm + np.clip(dq, -MAX_STEP, MAX_STEP), lo, hi)
        ctrl = q_ref.copy(); ctrl[arm_acts] = q_des_arm; data.ctrl[:] = ctrl
        mujoco.mj_step(model, data)
        errs.append(np.linalg.norm(err)); elbow.append(q_des_arm[3])
    e = np.array(errs[int(2.0/model.opt.timestep):])
    return e.mean()*1000, e.max()*1000, 100*sat/n, np.array(elbow)

print(f"model G1 that | timestep = {model.opt.timestep}s | mujoco {mujoco.__version__}")
lo3, hi3 = model.actuator_ctrlrange[arm_acts[3]]
print(f"gioi han khop khuyu: {lo3:+.3f} .. {hi3:+.3f} rad\n")
print(f'{"che do":>6} {"sai so tay TB":>14} {"sai so tay MAX":>15} {"buoc kep tran":>15} {"khuyu min..max":>22}')
print('-'*80)
for BUG in (True, False):
    a, b, c, el = run(BUG)
    print(f'{"BUG" if BUG else "FIX":>6} {a:11.1f} mm {b:12.1f} mm {c:14.0f}% '
          f'{el.min():+9.2f} ..{el.max():+7.2f} rad')
