"""Tuan 4 - Bai tap: NHIN THAY 4 bac tu do "du" cua tay 7 khop.

Y tuong (giong thi nghiem ngoay khuyu tay ma ngon tay van dinh 1 diem):
  - Bong DUNG YEN tai 1 diem co dinh
  - Tay phai bam chat diem do bang IK (nhu bai truoc)
  - DONG THOI, ta dinh nghia mot TU THE PHU dao dong theo thoi gian
    (q_null_target), roi KEO NHE q_des ve tu the do, nhung da LOC
    qua null space cua J truoc khi cong vao.

Cong thuc null space projection:
    N        = I - J^+ J                    (J^+ = pinv(J), ma tran chieu)
    delta    = q_null_target - q_des        (con thieu bao nhieu de dat tu the phu)
    dq_total = GAIN*dq_IK  +  N @ (NULL_GAIN * delta)

Bai hoc debug quan trong: LAN DAU viet demo nay, minh cho dq_null la
mot "van toc" tuy y roi clip theo MAX_STEP moi vong -> gap dung windup
(cau 4.3!): dq_null gan nhu luon bi kep tran MAX_STEP trong nua chu ky
dao dong -> tich luy thanh mot duong doc khong lo (khuyu tay lech ~2.2
rad, sai so tay len den 140mm). Fix: doi dq_null thanh MOT TY LE NHO
cua "khoang cach den tu the phu mong muon" (kieu P-controller) thay vi
dong thang mot van toc lon vao bo tich luy. Sau fix, sai so tay ~5mm.

Neu thay dung: sai so tay gan nhu khong doi, du khuyu tay + vai dao
dong ro ret.
"""
import time
import mujoco
import mujoco.viewer
import numpy as np

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene_target.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
q_ref = data.qpos[7:].copy()

ARM_JOINTS = [
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",   "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
def jid(n):  return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n):  return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)

arm_dofs = [model.jnt_dofadr[jid(n)] for n in ARM_JOINTS]
arm_acts = [aid(n) for n in ARM_JOINTS]
hand_id  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")

q_des_arm = np.array([q_ref[a] for a in arm_acts])
lo = model.actuator_ctrlrange[arm_acts, 0]
hi = model.actuator_ctrlrange[arm_acts, 1]

LAMBDA, GAIN, MAX_STEP = 0.05, 0.10, 0.003
q_base = q_des_arm.copy()   # tu the goc, lam tam cho dao dong phu

# Tu the phu: dao dong quanh tu the goc (index: 2=shoulder_yaw, 3=elbow)
SECONDARY_AMP  = np.array([0, 0, 0.8, 0.4, 0, 0, 0])
SECONDARY_FREQ = 0.25    # Hz
NULL_GAIN      = 0.05    # ty le "keo nhe" ve tu the phu (KHONG phai van toc truc tiep)

# Doi thanh True de TAI HIEN LOI windup trong null space (xem docstring).
# True : dq_null la mot VAN TOC tuy y -> bi kep tran MAX_STEP gan nhu ca nua
#        chu ky -> an het ngan sach cua IK -> tay troi khoi diem bam.
# False: dq_null = ty le nho cua "con thieu bao nhieu" -> tu tat khi dat.
BUG = False
err_max, n_sat, n_step = 0.0, 0, 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        t = data.time

        # Bong DUNG YEN - de de quan sat sai so tay co tang khong
        data.mocap_pos[0] = [0.30, -0.20, 0.85]

        err = data.mocap_pos[0] - data.body(hand_id).xpos
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, None, hand_id)
        J = jacp[:, arm_dofs]                                   # (3,7)

        # --- Uu tien 1: IK bam diem (nhu bai truoc) ---
        A = J @ J.T + LAMBDA**2*np.eye(3)
        dq_ik = J.T @ np.linalg.solve(A, err)

        # --- Uu tien 2: keo NHE ve mot TU THE PHU dao dong, loc qua null space ---
        Jpinv = J.T @ np.linalg.solve(A, np.eye(3))
        N = np.eye(7) - Jpinv @ J                                     # (7,7) ma tran chieu
        wave = SECONDARY_AMP * np.sin(2*np.pi*SECONDARY_FREQ*t)
        if BUG:
            dq_null = N @ wave                            # SAI: dung thang lam van toc
        else:
            delta = (q_base + wave) - q_des_arm           # con thieu bao nhieu
            dq_null = N @ (NULL_GAIN * delta)             # DUNG: ty le nho, tu tat khi dat

        dq = GAIN * dq_ik + dq_null
        if np.any(np.abs(dq) > MAX_STEP):
            n_sat += 1
        n_step += 1
        q_des_arm = np.clip(q_des_arm + np.clip(dq, -MAX_STEP, MAX_STEP), lo, hi)

        ctrl = q_ref.copy()
        ctrl[arm_acts] = q_des_arm
        data.ctrl[:] = ctrl

        mujoco.mj_step(model, data)
        viewer.sync()

        err_max = max(err_max, np.linalg.norm(err))
        if int(t*500) % 250 == 0:   # in moi 0.5s
            print(f"[{'BUG' if BUG else 'FIX'}] t={t:5.1f}s  "
                  f"sai_so_tay={np.linalg.norm(err)*1000:6.1f}mm  max={err_max*1000:6.1f}mm  "
                  f"buoc_kep_tran={100*n_sat/max(n_step,1):3.0f}%  "
                  f"elbow={q_des_arm[3]:+.2f}rad  shoulder_yaw={q_des_arm[2]:+.2f}rad")

        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
