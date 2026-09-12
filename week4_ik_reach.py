"""Tuan 4 - Inverse Kinematics: tay phai G1 voi theo qua bong di chuyen.

Thuat toan (resolved-rate IK voi damped least squares), moi vong lap:
  1. err = vi_tri_bong - vi_tri_tay            (vector 3D)
  2. J   = Jacobian cua co tay (3 x 7 cot khop tay phai)
  3. dq  = J^T (J J^T + lambda^2 I)^-1 err     (DLS: "chia cho J" an toan)
  4. q_des_tay += k * dq                        (nhich muc tieu khop 1 buoc nho)
  5. gui q_des cho position actuator

Chan + eo + tay trai: giu nguyen tu the dung (nhu bai 2).
Qua bong do chay theo vong tron -> tay phai bam theo.
"""
import time
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene_target.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
q_ref = data.qpos[7:].copy()

# --- Thong tin 7 khop tay phai ---
ARM_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",   "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
def jid(n):  return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n):  return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)

arm_dofs = [model.jnt_dofadr[jid(n)] for n in ARM_JOINTS]   # cot trong Jacobian
arm_acts = [aid(n) for n in ARM_JOINTS]                     # o trong data.ctrl
hand_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")

# Muc tieu khop cua tay phai (se duoc IK cap nhat dan)
q_des_arm = np.array([q_ref[a] for a in arm_acts])

# GIOI HAN KHOP (fix bug windup: q_des khong duoc vuot qua tam khop that)
lo = model.actuator_ctrlrange[arm_acts, 0]
hi = model.actuator_ctrlrange[arm_acts, 1]

LAMBDA = 0.05    # damping chong singularity
GAIN   = 0.10    # ty le sai so duoc "an" moi vong
MAX_STEP = 0.003 # gioi han toc do nhich muc tieu (0.003rad/2ms = 1.5rad/s)
# Bai hoc debug: GAIN=0.15 + MAX_STEP=0.02 lam tay dao dong du doi
# (overshoot qua lai quanh muc tieu) va quat nga ca robot!

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        t = data.time

        # --- Qua bong chay theo vong tron truoc mat robot ---
        data.mocap_pos[0] = [0.30,
                             -0.25 + 0.15*np.cos(2*np.pi*0.15*t),
                              0.90 + 0.15*np.sin(2*np.pi*0.15*t)]

        # ============ IK: 4 buoc ============
        # (1) sai so vi tri
        err = data.mocap_pos[0] - data.body(hand_id).xpos            # (3,)

        # (2) Jacobian vi tri cua co tay: 3 x nv, roi cat 7 cot tay phai
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, None, hand_id)
        J = jacp[:, arm_dofs]                                        # (3,7)

        # (3) damped least squares
        dq = J.T @ np.linalg.solve(J @ J.T + LAMBDA**2 * np.eye(3), err)  # (7,)

        # (4) nhich muc tieu khop: gioi han toc do + kep trong tam khop
        q_des_arm = np.clip(q_des_arm + np.clip(GAIN * dq, -MAX_STEP, MAX_STEP), lo, hi)
        # ====================================

        ctrl = q_ref.copy()          # toan than giu tu the dung...
        ctrl[arm_acts] = q_des_arm   # ...rieng tay phai theo IK
        data.ctrl[:] = ctrl

        mujoco.mj_step(model, data)
        viewer.sync()

        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
