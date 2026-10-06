"""Tuan 10 - WBC DONG LUC HOC giu thang bang (BUOC 1 cua lo trinh len G1 EDU).

KHAC HOAN TOAN week9_wbc.py. Tuan 9 giai theo VAN TOC khop (dq) va gia dinh
"ra lenh dq thi khop di dung the" - dung voi tay vay trong khong khi, SAI khi
robot dung tren chan, vi thu quyet dinh nga hay khong la LUC TIEP XUC o ban
chan, ma bai toan dong hoc khong he co bien do. Do la ly do lan debug #3 cua
tuan 9 (them khop eo -> nga sau 2-5s) KHONG sua duoc bang cach chinh trong so.

O day an so cua QP gom CA gia toc khop LAN luc tiep xuc:

    z = [ q_ddot (nv=35) ; f (8 diem x 3 = 24) ]        -> 59 bien

RANG BUOC CUNG (dieu tuan 9 khong co):
  (1) Dong luc hoc phan than noi - 6 hang DAU khong co mo-men truyen dong:
          M[0:6,:] q_ddot + h[0:6] = J_c^T[0:6,:] f
      Day chinh la Newton-Euler toan robot: chi co LUC TIEP XUC moi lam
      thay doi dong luong cua robot. Khong co f thi khong the giu thang bang.
  (2) Ban chan khong truot - rang buoc 6D (3 tinh tien + 3 quay) cho MOI ban
      chan, KHONG phai 3D cho moi diem: 4 diem tren 1 ban chan CUNG chi cho
      hang 6, dat 12 rang buoc diem se phu thuoc tuyen tinh va Goldfarb-Idnani
      bao "constraints are inconsistent". Luc van dat o 8 diem (can cho non
      ma sat / CoP), chi rang buoc DONG HOC la gop theo vat the.
          J_foot q_ddot + Jdot_foot q_dot = -Kd_c * v_foot
  (3) Non ma sat (tuyen tinh hoa kim tu thap): |fx|,|fy| <= mu*fz,  fz >= 0
      -> rang buoc nay TU DONG ep CoP nam trong da giac do, vi 8 diem tiep
         xuc chi day duoc (fz>=0) chu khong keo duoc.
  (4) Gioi han mo-men: tau = M[6:,:] q_ddot + h[6:] - J_c^T[6:,:] f  in [tau_min, tau_max]

MUC TIEU (ham muc tieu) - dung dang CENTROIDAL, dat thang len luc:
  - Tinh tien:  sum(f_i)            = m*(a_com_des - g)
  - Quay:       sum((p_i - c) x f_i) = Ldot_des = -Kp_ori*theta_err - Kd_L*L
  - Tu the:     q_ddot[khop] = Kp_q*(q_ref - q) - Kd_q*q_dot
  (dat muc tieu CoM len f thay vi len J_com q_ddot tranh phai tinh Jdot_com,
   va hai cach la TUONG DUONG vi rang buoc (1) da noi f voi q_ddot)

MO PHONG <-> ROBOT THAT: o day tat actuator cua MuJoCo (mjDSBL_ACTUATION) va
ghi mo-men thang vao qfrc_applied. Tren G1 EDU that, tuong duong voi gui
LowCmd moi khop kp=0, kd=0, tau=tau_wbc qua rt/lowcmd @500Hz.

CANH BAO: file nay CHI chay mo phong. KHONG nap len robot that truoc khi qua
buoc 0 (leg odometry doc lap voi /dog_odom) va phai treo gian - xem lo trinh.

Chay:  .venv-real/bin/python week10_wbc_balance.py
Phim:  cua so viewer; day nhieu tu dong moi PUSH_PERIOD giay (xem log).
"""
import sys
import time
import numpy as np
import mujoco
import mujoco.viewer
from quadprog import solve_qp

# ============================ THAM SO ============================
MU = 0.6                 # he so ma sat (khop voi friction="0.6" trong g1.xml)
FZ_MIN = 1.0             # N - giu tiep xuc "song", tranh f=0 lam QP suy bien

KP_COM, KD_COM = 60.0, 15.0     # bam CoM
KP_ORI, KD_ANG = 250.0, 40.0    # giu than thang + dap dong luong goc
KP_Q,   KD_Q   = 100.0, 20.0    # tu the khop

W_COM, W_ANG, W_POST, W_F, W_QACC = 60.0, 12.0, 1.0, 1e-3, 1e-4

KD_CONTACT = 30.0        # dap van toc truot ban chan (Baumgarte)

PUSH_PERIOD, PUSH_DUR = 4.0, 0.15
PUSH_FORCE = np.array([70.0, 40.0, 0.0])      # ghi de bang --push FX FY

# ============================ MO HINH ============================
model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene.xml")
data = mujoco.MjData(model)

# Tat actuator vi tri cua MuJoCo - WBC ghi mo-men thang vao qfrc_applied
model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_ACTUATION

nv, nu = model.nv, model.nu
mujoco.mj_resetDataKeyframe(model, data, 0)
mujoco.mj_forward(model, data)

pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")

# --- cac dof duoc truyen dong: kiem tra dung thu tu voi actuator ---
act_dofs = []
for a in range(nu):
    j = model.actuator_trnid[a, 0]
    act_dofs.append(model.jnt_dofadr[j])
act_dofs = np.array(act_dofs)
assert np.array_equal(act_dofs, np.arange(6, 6 + nu)), "thu tu dof khong lien tuc"
tau_lo = np.array([model.jnt_actfrcrange[model.actuator_trnid[a, 0], 0] for a in range(nu)])
tau_hi = np.array([model.jnt_actfrcrange[model.actuator_trnid[a, 0], 1] for a in range(nu)])

# --- 8 diem tiep xuc: cac geom sphere r=0.005 tren hai ban chan ---
contact_geoms = [g for g in range(model.ngeom)
                 if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
                 and abs(model.geom_size[g, 0] - 0.005) < 1e-9]
contact_bodies = [model.geom_bodyid[g] for g in contact_geoms]
contact_radius = [model.geom_size[g, 0] for g in contact_geoms]
NC = len(contact_geoms)
NF = 3 * NC
NZ = nv + NF
assert NC == 8, f"mong doi 8 diem tiep xuc, thay {NC}"

foot_bodies = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
               for n in ("left_ankle_roll_link", "right_ankle_roll_link")]
NFEET = len(foot_bodies)
NEQ_C = 6 * NFEET

mass_total = model.body_mass.sum()
gravity = model.opt.gravity.copy()
q_ref = data.qpos[7:].copy()
com_des = data.subtree_com[1].copy()
quat_des = data.xquat[pelvis_id].copy()

def set_stand_pose(knee):
    """Dat tu the dung voi goi chung 'knee' rad, ha than cho ban chan cham dat,
    roi cap nhat moc tham chieu (q_ref, com_des, quat_des).

    Keyframe 'stand' cua menagerie de MOI khop chan = 0 rad, tuc goi DUOI THANG
    -> Jacobian chan KY DI (cond ~ 2e6, sigma_min ~ 9e-7). O do khong nghich dao
    duoc J, moi thu can nghich dao (uoc luong luc tiep xuc tu mo-men khop, IK,
    admittance) deu vo. Robot humanoid that luon dung goi chung vi ly do nay.
    """
    global q_ref, com_des, quat_des
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for base in (0, 6):                       # chan trai, chan phai
        data.qpos[7 + base + 0] = -knee / 2   # hip_pitch
        data.qpos[7 + base + 3] = knee        # knee
        data.qpos[7 + base + 4] = -knee / 2   # ankle_pitch
    mujoco.mj_forward(model, data)
    low = min(data.geom_xpos[g][2] - r for g, r in zip(contact_geoms, contact_radius))
    data.qpos[2] -= low
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    q_ref = data.qpos[7:].copy()
    com_des = data.subtree_com[1].copy()
    quat_des = data.xquat[pelvis_id].copy()


KNEE = float(sys.argv[sys.argv.index("--bend") + 1]) if "--bend" in sys.argv else 0.0
if KNEE > 0:
    set_stand_pose(KNEE)

print(f"nv={nv} nu={nu} | {NC} diem tiep xuc | bien QP={NZ} | m={mass_total:.2f}kg | goi={KNEE:.2f}rad")
print(f"CoM muc tieu = {com_des}")

# ==================== BO NHO TAM (tranh cap phat moi vong) ====================
M_full = np.zeros((nv, nv))
Jc = np.zeros((NF, nv))          # Jacobian 8 diem - CHI dung de truyen luc J_c^T f
Jfeet = np.zeros((NEQ_C, nv))    # Jacobian 6D hai ban chan - dung lam rang buoc
Jdotqdot_feet = np.zeros(NEQ_C)
jacp = np.zeros((3, nv))
jacr = np.zeros((3, nv))
jacp_dot = np.zeros((3, nv))
jacr_dot = np.zeros((3, nv))
tau_last = np.zeros(nu)


def build_contacts():
    """Dien Jc, Jdotqdot_c va tra ve vi tri the gioi cua 8 diem tiep xuc."""
    pts = np.zeros((NC, 3))
    for i, (g, b, r) in enumerate(zip(contact_geoms, contact_bodies, contact_radius)):
        p = data.geom_xpos[g] - np.array([0.0, 0.0, r])   # day qua cau = diem cham
        pts[i] = p
        jacp[:] = 0.0
        mujoco.mj_jac(model, data, jacp, None, p, b)
        Jc[3 * i:3 * i + 3, :] = jacp
        jacp_dot[:] = 0.0
        jacr_dot[:] = 0.0
    for i, b in enumerate(foot_bodies):
        jacp[:] = 0.0; jacr[:] = 0.0
        mujoco.mj_jacBody(model, data, jacp, jacr, b)
        Jfeet[6 * i:6 * i + 3, :] = jacp
        Jfeet[6 * i + 3:6 * i + 6, :] = jacr
        jacp_dot[:] = 0.0; jacr_dot[:] = 0.0
        mujoco.mj_jacDot(model, data, jacp_dot, jacr_dot, data.xpos[b], b)
        Jdotqdot_feet[6 * i:6 * i + 3] = jacp_dot @ data.qvel
        Jdotqdot_feet[6 * i + 3:6 * i + 6] = jacr_dot @ data.qvel
    return pts


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def solve_wbc():
    """Dung va giai QP. Tra ve (tau, info)."""
    mujoco.mj_fullM(model, data, M_full)
    h = data.qfrc_bias.copy()
    mujoco.mj_subtreeVel(model, data)

    pts = build_contacts()
    com = data.subtree_com[1].copy()
    v_com = data.subtree_linvel[1].copy()
    L_ang = data.subtree_angmom[1].copy()

    # ---- sai so huong cua than (vector quay tu hien tai -> mong muon) ----
    qneg = np.zeros(4); qerr = np.zeros(4); ori_err = np.zeros(3)
    mujoco.mju_negQuat(qneg, data.xquat[pelvis_id])
    mujoco.mju_mulQuat(qerr, quat_des, qneg)
    mujoco.mju_quat2Vel(ori_err, qerr, 1.0)

    # ================= HAM MUC TIEU =================
    H = np.zeros((NZ, NZ))
    g_lin = np.zeros(NZ)

    # (a) tinh tien: sum(f_i) = m*(a_com_des - g)
    a_com_des = KP_COM * (com_des - com) - KD_COM * v_com
    A_lin = np.zeros((3, NZ))
    for i in range(NC):
        A_lin[:, nv + 3 * i: nv + 3 * i + 3] = np.eye(3)
    b_lin = mass_total * (a_com_des - gravity)
    H += W_COM * A_lin.T @ A_lin
    g_lin += W_COM * A_lin.T @ b_lin

    # (b) quay: sum((p_i - c) x f_i) = Ldot_des
    A_ang = np.zeros((3, NZ))
    for i in range(NC):
        A_ang[:, nv + 3 * i: nv + 3 * i + 3] = skew(pts[i] - com)
    b_ang = KP_ORI * ori_err - KD_ANG * L_ang
    H += W_ANG * A_ang.T @ A_ang
    g_lin += W_ANG * A_ang.T @ b_ang

    # (c) tu the khop
    A_post = np.zeros((nu, NZ))
    A_post[:, 6:6 + nu] = np.eye(nu)
    b_post = KP_Q * (q_ref - data.qpos[7:]) - KD_Q * data.qvel[6:]
    H += W_POST * A_post.T @ A_post
    g_lin += W_POST * A_post.T @ b_post

    # (d) chinh hoa: luc gan phan bo deu trong luong, q_ddot gan 0
    f_nom = np.tile([0.0, 0.0, mass_total * 9.81 / NC], NC)
    H[nv:, nv:] += W_F * np.eye(NF)
    g_lin[nv:] += W_F * f_nom
    H[:nv, :nv] += W_QACC * np.eye(nv)
    H += 1e-8 * np.eye(NZ)        # bao dam xac dinh duong

    # ================= RANG BUOC DANG THUC =================
    JcT = Jc.T
    # (1) than noi: M[0:6] qddot - J_c^T[0:6] f = -h[0:6]
    Ceq1 = np.zeros((6, NZ)); Ceq1[:, :nv] = M_full[:6, :]; Ceq1[:, nv:] = -JcT[:6, :]
    beq1 = -h[:6]
    # (2) ban chan khong truot - 6D moi ban chan (hang doc lap)
    Ceq2 = np.zeros((NEQ_C, NZ)); Ceq2[:, :nv] = Jfeet
    beq2 = -Jdotqdot_feet - KD_CONTACT * (Jfeet @ data.qvel)
    Ceq = np.vstack([Ceq1, Ceq2]); beq = np.concatenate([beq1, beq2])

    # ================= RANG BUOC BAT DANG THUC (dang >= 0) =================
    rows, rhs = [], []
    for i in range(NC):
        s = nv + 3 * i
        e_fz = np.zeros(NZ); e_fz[s + 2] = 1.0
        rows.append(e_fz);              rhs.append(FZ_MIN)          # fz >= FZ_MIN
        for k, sgn in ((0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0)):
            r = np.zeros(NZ); r[s + 2] = MU; r[s + k] = sgn
            rows.append(r); rhs.append(0.0)                          # mu*fz +- f_t >= 0
    # gioi han mo-men
    T = np.zeros((nu, NZ)); T[:, :nv] = M_full[6:, :]; T[:, nv:] = -JcT[6:, :]
    h_a = h[6:]
    for k in range(nu):
        rows.append(T[k]);   rhs.append(tau_lo[k] - h_a[k])
        rows.append(-T[k]);  rhs.append(h_a[k] - tau_hi[k])
    Cin = np.array(rows); bin_ = np.array(rhs)

    # ================= GIAI =================
    C = np.vstack([Ceq, Cin]).T           # quadprog: C.T z >= b, meq dau la dang thuc
    b = np.concatenate([beq, bin_])
    z = solve_qp(H, g_lin, C, b, meq=Ceq.shape[0])[0]

    qacc, f = z[:nv], z[nv:]
    tau = M_full[6:, :] @ qacc + h_a - JcT[6:, :] @ f
    return tau, dict(com=com, f=f, fz_min=f[2::3].min(), L=np.linalg.norm(L_ang))


# ============================ VONG LAP ============================
HEADLESS = "--headless" in sys.argv
SIM_SECONDS = float(sys.argv[sys.argv.index("--headless") + 1]) if HEADLESS else None
if "--push" in sys.argv:
    i = sys.argv.index("--push")
    PUSH_FORCE = np.array([float(sys.argv[i + 1]), float(sys.argv[i + 2]), 0.0])
QUIET = "--quiet" in sys.argv
n_fail = 0


def control_step():
    """Mot chu ky dieu khien. Tra ve (tau, info, pushing)."""
    global n_fail
    mujoco.mj_step1(model, data)
    t = data.time
    data.xfrc_applied[:] = 0.0
    pushing = (t % PUSH_PERIOD) < PUSH_DUR and t > 1.0
    if pushing:
        sign = 1.0 if int(t / PUSH_PERIOD) % 2 == 0 else -1.0
        data.xfrc_applied[torso_id, :3] = sign * PUSH_FORCE
    try:
        tau, info = solve_wbc()
        tau = np.clip(tau, tau_lo, tau_hi)
        tau_last[:] = tau
    except Exception as e:
        n_fail += 1
        tau, info = tau_last, None
        if n_fail <= 5:
            print(f"  [QP FAIL] t={t:.2f}s: {e}")
    data.qfrc_applied[6:] = tau
    mujoco.mj_step2(model, data)
    return tau, info, pushing


def log(tau, info, pushing):
    if info is None:
        return
    dc = info["com"] - com_des
    print(f"t={data.time:5.1f}s  CoM err=[{dc[0]*1000:6.1f} {dc[1]*1000:6.1f} {dc[2]*1000:6.1f}]mm  "
          f"|L|={info['L']:5.2f}  fz_min={info['fz_min']:6.1f}N  "
          f"|tau|max={np.abs(tau).max():6.1f}Nm  cao={data.qpos[2]:.3f}m"
          f"{'  <-- DAY' if pushing else ''}")


def main():
    global n_fail, max_com_err
    if HEADLESS:
        t_wall = time.time()
        max_com_err = 0.0
        n_steps = int(SIM_SECONDS / model.opt.timestep)
        for k in range(n_steps):
            tau, info, pushing = control_step()
            if info is not None:
                max_com_err = max(max_com_err, np.linalg.norm(info["com"][:2] - com_des[:2]))
            if k % 250 == 0 and not QUIET:
                log(tau, info, pushing)
            if data.qpos[2] < 0.4:
                print(f"\n*** NGA o t={data.time:.2f}s (cao={data.qpos[2]:.3f}m) ***")
                break
        else:
            print(f"\n*** DUNG VUNG {SIM_SECONDS:.0f}s ***")
        print(f"CoM lech max: {max_com_err*1000:.1f} mm")
        el = time.time() - t_wall
        print(f"QP fail: {n_fail}  |  toc do: {data.time/el:.2f}x thoi gian thuc "
              f"({el/max(data.time/model.opt.timestep,1)*1e3:.3f} ms/buoc)")
    else:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                step_start = time.time()
                tau, info, pushing = control_step()
                viewer.sync()
                if int(data.time * 500) % 250 == 0:
                    log(tau, info, pushing)
                dt = model.opt.timestep - (time.time() - step_start)
                if dt > 0:
                    time.sleep(dt)


if __name__ == "__main__":
    main()
