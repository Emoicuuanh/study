"""Tuan 9 - Whole-Body Control (WBC): NHIEU muc tieu dong thoi, MOT bai toan QP.

PHAM VI (thu hep sau 3 lan debug that bai - xem duoi): WBC nay dieu khien
CHI 14 khop 2 TAY. Chan + eo giu nguyen tu the dung (PD tuan 2, da kiem
chung on dinh xuyen suot khoa hoc). Hai tay CUNG LUC voi toi 2 qua bong
di chuyen doc lap, tay phai duoc UU TIEN CAO hon tay trai (trong so).

Ca hai muc tieu giai CHUNG trong MOT bai toan binh phuong toi thieu co
RANG BUOC (QP), dung scipy.optimize.lsq_linear:

    minimize  || W_R * (J_R dq - v_R_target) ||^2      (tay phai, uu tien cao)
            + || W_L * (J_L dq - v_L_target) ||^2      (tay trai, uu tien thap)
            + || W_REG * dq                  ||^2       (giam chan, on dinh so)
    subject to  -DQ_MAX <= dq <= DQ_MAX                 (GIOI HAN dua vao
                                                    NGAY TU DAU bai toan,
                                                    khong clip SAU nhu tuan 4)

Khac null-space tuan 4 (1 muc tieu chinh + 1 muc tieu phu qua chieu):
o day 2 muc tieu THUC SU canh tranh (ca 2 tay dung chung vai khop qua vai
Jacobian truncate) va duoc CAN BANG bang TRONG SO trong 1 QP, khong phai
uu tien cung nhac (hard priority). Day la kieu WBC "soft priority" pho
bien trong thuc te.

=== BA LAN DEBUG THAT BAI TRUOC KHI RA BAN NAY (dang ke vao ho so) ===
1. Dinh dua "giu CoM" lam uu tien 1 (dieu khien qua tay+than tren) +
   "voi tay" lam uu tien 2. THAT BAI: dung mj_jacSubtreeCom voi body=
   torso_id -> Jacobian cua khoi tam SUBTREE CUA TORSO (chi tay+dau),
   nhung do luong com_now lai dung subtree_com[0] (khoi tam TOAN ROBOT,
   gom ca chan) -> hai dai luong LECH NHAU, dq tinh sai huong hoan toan.
   Bai hoc: giong loi MPC tuan 8 - luon kiem tra 2 dai luong dang so
   sanh co dung CHUNG dinh nghia vat ly khong.
2. Sua loi (1) roi van nga: dung "err/DT" lam van toc muc tieu = doi
   sua HET loi trong dung 1 chu ky 2ms (gain ~500!) -> QP luon kich
   DQ_MAX theo huong sai, danh ca 2 tac vu. Sua thanh gain P vua phai
   (KP=4.0) thay vi 1/DT.
3. Van nga (dung hon, sau ~2-5s): dieu khien CA khop eo (waist) de giup
   2 tay voi 2 vat doi nghich -> eo VAN QUA MANH de "giup" tay, tao phan
   luc xoay than -> chan PD tinh (khong biet than dang xoay) khong giu
   duoc -> do. Day KHONG phai bug code - la GIOI HAN THAT: WBC dieu
   khien dong hoc (khong co dong luc hoc + tiep xuc) khong duoc dung de
   "giu can bang qua than tren" khi chan khong dong bo - can QP toan
   than co rang buoc tiep xuc (ngoai pham vi 1 buoi). Fix: BO khop eo
   khoi WBC, chi con 2 tay - da kiem chung on dinh 12s.

=> Bai hoc tong quat: WBC ban dau tuong "de" (chi la QP + Jacobian) nhung
that ra RAT DE tao ra chuyen dong "dung ve toan hoc, sai ve vat ly" khi
bo qua dong luc hoc/tiep xuc. Day chinh la ly do WBC san xuat thuc te
(vd trong cac paper Unitree/Boston Dynamics) luon giai dong thoi VOI
dong luc hoc + rang buoc luc tiep xuc, khong chi Jacobian dong hoc don.
"""
import time
import numpy as np
import mujoco
import mujoco.viewer
from scipy.optimize import lsq_linear

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene_wbc.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
q_ref = data.qpos[7:].copy()

ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
def jid(n): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
def aid(n): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)

u_dofs = [model.jnt_dofadr[jid(n)] for n in ARM_JOINTS]
u_acts = [aid(n) for n in ARM_JOINTS]
n_u = len(u_dofs)

rhand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_wrist_yaw_link")
lhand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_wrist_yaw_link")

q_des_u = np.array([q_ref[a] for a in u_acts])
lo_u = model.actuator_ctrlrange[u_acts, 0]
hi_u = model.actuator_ctrlrange[u_acts, 1]

KP = 4.0                         # gain P (KHONG dung 1/DT - bai hoc debug #2)
W_R, W_L, W_REG = 20.0, 1.0, 0.5  # tay phai uu tien gap 3 lan tay trai
DQ_MAX = 0.8                      # rad/s - dua vao QP TU DAU (bai hoc MPC)
DT_CTRL = model.opt.timestep

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        t = data.time

        ball_r = np.array([0.30, -0.25 + 0.15*np.cos(2*np.pi*0.15*t), 0.90 + 0.15*np.sin(2*np.pi*0.15*t)])
        ball_l = np.array([0.30,  0.25 + 0.15*np.cos(2*np.pi*0.20*t + 1.5), 0.90 + 0.15*np.sin(2*np.pi*0.20*t + 1.5)])
        data.mocap_pos[0] = ball_r   # ve qua bong DO (muc tieu tay phai) len man hinh
        data.mocap_pos[1] = ball_l   # ve qua bong XANH (muc tieu tay trai) len man hinh

        # ============ WBC: xay dung va giai QP ============
        r_now = data.body(rhand_id).xpos
        l_now = data.body(lhand_id).xpos
        err_r = KP * (ball_r - r_now)
        err_l = KP * (ball_l - l_now)

        jr = np.zeros((3, model.nv)); mujoco.mj_jacBody(model, data, jr, None, rhand_id)
        jl = np.zeros((3, model.nv)); mujoco.mj_jacBody(model, data, jl, None, lhand_id)
        J_r = jr[:, u_dofs]
        J_l = jl[:, u_dofs]

        A = np.vstack([W_R * J_r, W_L * J_l, W_REG * np.eye(n_u)])
        b = np.concatenate([W_R * err_r, W_L * err_l, np.zeros(n_u)])

        res = lsq_linear(A, b, bounds=(-DQ_MAX, DQ_MAX), max_iter=30)
        dq = res.x
        # ====================================================

        q_des_u = np.clip(q_des_u + dq * DT_CTRL, lo_u, hi_u)

        ctrl = q_ref.copy()
        ctrl[u_acts] = q_des_u
        data.ctrl[:] = ctrl

        mujoco.mj_step(model, data)
        viewer.sync()

        if int(t*500) % 250 == 0:
            print(f"t={t:5.1f}s  err_R={np.linalg.norm(ball_r-r_now)*1000:5.1f}mm  "
                  f"err_L={np.linalg.norm(ball_l-l_now)*1000:5.1f}mm  cao={data.qpos[2]:.3f}m")

        dt = DT_CTRL - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
