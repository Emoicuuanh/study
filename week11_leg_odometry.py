"""Tuan 11 - BUOC 0: LEG ODOMETRY, thay the /dog_odom.

VAN DE. File G1/src/g1_state_estimator/src/estimator.cpp:97 UpdateLeg() lay
van toc than tu topic /dog_odom (config estimator.yaml:6). /dog_odom do SDK
HIGH-LEVEL sinh ra - tuc la do CHINH BO DIEU KHIEN MA WBC SE TAT. Khi chuyen
sang low-level rt/lowcmd, nhanh cap nhat do hoac chet han, hoac te hon la tra
so cu ma IEKF van tin. Con FAST-LIO chi 50-100Hz, qua cham cho vong 500Hz.

=> Phai tu tinh van toc than tu DUY NHAT nhung gi rt/lowstate co:
   encoder khop (q, dq) + IMU (gyro) + trang thai tiep xuc.

NGUYEN LY. Goi B = he pelvis, W = he the gioi, chan f dang cham dat:
    v_WF = v_WB + R(w_B x p_BF) + R(J_BF qdot_leg)
Chan dang tua (khong truot) => v_WF = 0 =>
    v_WB = -R(w_B x p_BF + J_BF qdot_leg)
Nhan R^T hai ve:
    v_B = -(w_B x p_BF + J_BF qdot_leg)          <-- KHONG CO R !

Day la diem mau chot: van toc than TRONG HE THAN khong can biet huong robot,
chi can gyro + encoder. Ma UpdateLeg() dang can dung dai luong do (leg.v_body).
Sai so huong cua IMU khong lot vao phep do nay.

FK/Jacobian tinh tren mot BAN SAO mo hinh voi than co dinh tai goc, huong don
vi - dung y la KHONG dung pose that cua than (robot that khong biet pose do).

Cam bien mo phong DUNG cua MuJoCo: gyro + accelerometer tai site imu_in_pelvis
(g1.xml:342-343, noise 0.0005 / 0.01) + nhieu encoder tu them.

Chay:
  .venv-real/bin/python week11_leg_odometry.py              # sach, 8s
  .venv-real/bin/python week11_leg_odometry.py --noise      # bat nhieu cam bien
  .venv-real/bin/python week11_leg_odometry.py --noise --secs 20
"""
import sys
import numpy as np
import mujoco

import week10_wbc_balance as wbc

model, data = wbc.model, wbc.data

NOISE = "--noise" in sys.argv
SECS = float(sys.argv[sys.argv.index("--secs") + 1]) if "--secs" in sys.argv else 8.0
DT = model.opt.timestep
rng = np.random.default_rng(0)

# Nhieu encoder (khong co san trong mo hinh - tu them, muc do thuc te)
ENC_Q_STD = 1e-3 if NOISE else 0.0       # rad
ENC_DQ_STD = 2e-2 if NOISE else 0.0      # rad/s
DLS_LAMBDA = 1e-3                        # dap cho nghich dao J6 gan ky di
# MuJoCo 3.10 da BO co mjENBL_SENSORNOISE, thuoc tinh noise= trong XML chi con la
# khai bao thong so -> tu cong nhieu, lay dung do lech chuan model da khai bao.
GYRO_BIAS = np.array([0.002, -0.0015, 0.001]) if NOISE else np.zeros(3)

# ---------- id cam bien / vat the ----------
def sid(n): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, n)
GYRO = sid("imu-pelvis-angular-velocity")
ACC = sid("imu-pelvis-linear-acceleration")
GYRO_ADR, ACC_ADR = model.sensor_adr[GYRO], model.sensor_adr[ACC]
GYRO_STD = float(model.sensor_noise[GYRO]) if NOISE else 0.0
ACC_STD = float(model.sensor_noise[ACC]) if NOISE else 0.0
pelvis_id = wbc.pelvis_id
foot_bodies = wbc.foot_bodies
leg_dofs = [np.arange(6, 12), np.arange(12, 18)]     # trai, phai

# ---------- mo hinh rieng cho FK: than co dinh tai goc ----------
fk_model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene.xml")
fk_data = mujoco.MjData(fk_model)
_jacp = np.zeros((3, model.nv))
_jacr = np.zeros((3, model.nv))
_tau_g = np.zeros(model.nv)


def leg_odometry(q_meas, dq_meas, gyro, R_imu, tau_meas):
    """Chi dung q, dq, gyro, tau do duoc. Tra ve (v_body, fz_est[2])."""
    # --- FK voi than tai goc, huong don vi ---
    fk_data.qpos[:3] = 0.0
    fk_data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    fk_data.qpos[7:] = q_meas
    fk_data.qvel[:] = 0.0
    mujoco.mj_kinematics(fk_model, fk_data)
    mujoco.mj_comPos(fk_model, fk_data)

    # mo-men trong luc cua chan, tinh voi trong luc quay ve he than (can huong IMU)
    fk_model.opt.gravity[:] = R_imu.T @ model.opt.gravity
    mujoco.mj_rne(fk_model, fk_data, 0, _tau_g)

    v_est, fz_est = [], []
    for k, (b, dofs) in enumerate(zip(foot_bodies, leg_dofs)):
        p_BF = fk_data.xpos[b].copy()                    # vi tri chan trong he than
        _jacp[:] = 0.0; _jacr[:] = 0.0
        mujoco.mj_jacBody(fk_model, fk_data, _jacp, _jacr, b)
        J_lin = _jacp[:, dofs]                           # 3x6
        # v_B = -(w x p + J qdot)
        v_est.append(-(np.cross(gyro, p_BF) + J_lin @ dq_meas[dofs - 6]))
        # --- uoc luong tiep xuc: WRENCH 6D, khong phai luc diem 3D ---
        # Luc tiep xuc that phan bo tren 4 diem de ban chan, lech khoi goc co
        # chan -> sinh MO-MEN. Mo hinh bang luc diem tai goc co chan thi canh
        # tay don = 0, khong sinh noi mo-men co chan (= fz * (CoP - co chan)),
        # he 6 pt/3 an tro nen mau thuan -> nghiem rac. Dung wrench 6D: 6/6.
        # Goi duoi thang (keyframe) -> J6 ky di (cond ~2e6): nghich dao truc tiep
        # cho nghiem rac hang chuc nghin N. Dung binh phuong toi thieu CO DAP.
        J6 = np.vstack([J_lin, _jacr[:, dofs]])          # 6x6
        rhs = _tau_g[dofs] - tau_meas[dofs - 6]
        w = np.linalg.solve(J6 @ J6.T + DLS_LAMBDA ** 2 * np.eye(6), J6 @ rhs)
        fz_est.append((R_imu @ w[:3])[2])
    return np.array(v_est), np.array(fz_est)


def true_foot_fz():
    """Luc phap tuyen that o moi ban chan (chi dung de CHAM DIEM, khong vao uoc luong)."""
    fz = np.zeros(2)
    frc = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        for g in (c.geom1, c.geom2):
            b = model.geom_bodyid[g]
            if b in foot_bodies:
                mujoco.mj_contactForce(model, data, i, frc)
                n = c.frame[:3]
                fz[foot_bodies.index(b)] += abs(frc[0] * n[2])
                break
    return fz


# ============================ CHAY ============================
def reset_pose():
    if wbc.KNEE > 0:
        wbc.set_stand_pose(wbc.KNEE)
    else:
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)


reset_pose()

# KIEM TRA GIA DINH: qvel[0:3] cua free joint = van toc goc than trong he THE GIOI?
# (MuJoCo Euler nua an: x_{t+1} = x_t + dt*v_{t+1}, nen sai phan = qvel SAU buoc.
#  mj_step khong cap nhat lai xpos sau khi tich phan -> phai goi mj_kinematics.)
p0 = data.xpos[pelvis_id].copy()
mujoco.mj_step(model, data)
mujoco.mj_kinematics(model, data)
v_fd = (data.xpos[pelvis_id] - p0) / DT
assert np.allclose(v_fd, data.qvel[:3], atol=1e-6), f"gia dinh sai: {v_fd} vs {data.qvel[:3]}"
print(f"[check] qvel[0:3] dung la van toc than he the gioi (lech {np.abs(v_fd-data.qvel[:3]).max():.2e})")
reset_pose()

print(f"nhieu cam bien: {'BAT' if NOISE else 'TAT'}  | {SECS:.0f}s  | goi={wbc.KNEE:.2f}rad")
if NOISE:
    print(f"  gyro sigma={GYRO_STD} rad/s (+bias {GYRO_BIAS[0]}) | acc sigma={ACC_STD} m/s^2 | "
          f"encoder q={ENC_Q_STD} rad, dq={ENC_DQ_STD} rad/s")

WARMUP = 1.0          # bo qua giai doan on dinh ban dau
err, v_true_all, fz_err, fz_true_all, is_dyn = [], [], [], [], []
v_W_cf = data.qvel[:3].copy()     # loc bu: van toc the gioi
p_W_cf = data.xpos[pelvis_id].copy()
K_CF = 20.0                        # he so keo ve phep do chan (1/s)
n_steps = int(SECS / DT)

for k in range(n_steps):
    mujoco.mj_step1(model, data)

    t = data.time
    data.xfrc_applied[:] = 0.0
    pushing = 1.0 < t and (t % 4.0) < 0.15
    if pushing:
        data.xfrc_applied[wbc.torso_id, :3] = [-40.0, 25.0, 0.0]

    try:
        tau, _ = wbc.solve_wbc()
        tau = np.clip(tau, wbc.tau_lo, wbc.tau_hi)
        wbc.tau_last[:] = tau
    except Exception:
        tau = wbc.tau_last

    # ---------- DOC "rt/lowstate" ----------
    q_meas = data.qpos[7:] + rng.normal(0, ENC_Q_STD, model.nu)
    dq_meas = data.qvel[6:] + rng.normal(0, ENC_DQ_STD, model.nu)
    gyro = data.sensordata[GYRO_ADR:GYRO_ADR + 3] + GYRO_BIAS + rng.normal(0, GYRO_STD, 3)
    acc = data.sensordata[ACC_ADR:ACC_ADR + 3] + rng.normal(0, ACC_STD, 3)
    R_imu = data.xmat[pelvis_id].reshape(3, 3).copy()      # IMU cung cap quaternion

    v_feet, fz_est = leg_odometry(q_meas, dq_meas, gyro, R_imu, tau)

    # ---------- hop nhat: chi lay chan dang cham ----------
    in_contact = fz_est > 30.0
    v_body = v_feet[in_contact].mean(axis=0) if in_contact.any() else v_feet.mean(axis=0)

    # ---------- loc bu IMU + leg odometry (he the gioi) ----------
    a_W = R_imu @ acc + model.opt.gravity        # accelerometer do luc rieng
    v_W_cf += a_W * DT
    v_W_cf += K_CF * (R_imu @ v_body - v_W_cf) * DT
    p_W_cf += v_W_cf * DT

    # ---------- cham diem ----------
    v_body_true = R_imu.T @ data.qvel[:3]
    if t >= WARMUP:
        err.append(v_body - v_body_true)
        v_true_all.append(v_body_true)
        fzt = true_foot_fz()
        fz_err.append(fz_est - fzt)
        fz_true_all.append(fzt)
        # "dong": trong 0.6s sau moi cu day -> qdd lon, so hang quan tinh dang ke
        is_dyn.append((t % 4.0) < 0.6)

    data.qfrc_applied[6:] = tau
    mujoco.mj_step2(model, data)

    if k % 1000 == 0 and err:
        e = err[-1]
        print(f"t={t:5.1f}s  v_body_that=[{v_body_true[0]:+.3f} {v_body_true[1]:+.3f} {v_body_true[2]:+.3f}]  "
              f"sai so=[{e[0]*1000:+6.1f} {e[1]*1000:+6.1f} {e[2]*1000:+6.1f}] mm/s"
              f"{'  <-- DAY' if pushing else ''}")

err = np.array(err); v_true_all = np.array(v_true_all)
fz_err = np.array(fz_err); fz_true_all = np.array(fz_true_all)
is_dyn = np.array(is_dyn); stat = ~is_dyn

print(f"\n=== VAN TOC THAN TRONG HE THAN (v_body), bo {WARMUP:.0f}s dau ===")
for i, ax in enumerate("xyz"):
    print(f"  {ax}: RMS {np.sqrt((err[:,i]**2).mean())*1000:6.2f} mm/s   "
          f"max |e| {np.abs(err[:,i]).max()*1000:7.2f} mm/s   "
          f"(bien do that +-{np.abs(v_true_all[:,i]).max()*1000:.0f} mm/s)")
print(f"  RMS chuan 3 truc: {np.sqrt((err**2).sum(axis=1).mean())*1000:.2f} mm/s")

print("\n=== UOC LUONG LUC PHAP TUYEN TU MO-MEN KHOP ===")
print("  (bo qua so hang quan tinh M*qddot -> chi dung khi tinh/gan tinh)")
for k, n in enumerate(("trai", "phai")):
    print(f"  chan {n}: fz that TB {fz_true_all[stat,k].mean():6.1f} N | "
          f"RMS tinh {np.sqrt((fz_err[stat,k]**2).mean()):6.1f} N | "
          f"RMS luc bi day {np.sqrt((fz_err[is_dyn,k]**2).mean()):7.1f} N")
print(f"  phat hien tiep xuc (nguong 30N): dung {100*((fz_true_all-fz_err > 30) == (fz_true_all > 30)).mean():.1f}% so buoc")

drift = np.linalg.norm(p_W_cf - data.xpos[pelvis_id])
print(f"\n=== LOC BU (IMU + leg odom) ===")
print(f"  troi vi tri sau {SECS:.0f}s: {drift*1000:.1f} mm  ({drift/SECS*1000:.1f} mm/s)")
print("  (vi tri VAN troi - do la viec cua FAST-LIO. Can bang chi can VAN TOC.)")
