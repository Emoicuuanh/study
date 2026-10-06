"""Leg odometry cho Unitree G1: van toc than tu encoder + gyro + mo-men khop.

THAY THE /dog_odom. Topic /dog_odom do SDK HIGH-LEVEL sinh ra, tuc la do chinh
bo dieu khien ma WBC low-level se tat. g1_state_estimator/src/estimator.cpp:97
UpdateLeg() dang lay leg.v_body tu do (config/estimator.yaml:6). Khi chuyen
sang rt/lowcmd, nguon do hoac chet, hoac te hon la tra so cu ma IEKF van tin.

NGUYEN LY. B = he pelvis, W = he the gioi, chan f dang tua (khong truot):
    v_WF = v_WB + R(w_B x p_BF) + R(J_BF qdot_leg) = 0
=>  v_B = R^T v_WB = -(w_B x p_BF + J_BF qdot_leg)

KHONG CO R trong ve phai. Van toc than TRONG HE THAN chi can gyro + encoder,
khong can biet huong robot => sai so huong cua IMU khong lot vao phep do nay.
Dung la dai luong ma UpdateLeg() can.

FK/Jacobian tinh voi than GHIM TAI GOC, huong don vi - co y khong dung pose
that cua than, vi robot that khong biet pose do.

DA KIEM CHUNG doi chieu ground truth trong MuJoCo (test/test_against_mujoco.py):
van toc RMS ~2.6 mm/s (sach) / ~14 mm/s (co nhieu cam bien) tren bien do 257 mm/s.

HAI GIOI HAN PHAI BIET TRUOC KHI DUNG:

1. TU THE GOI DUOI THANG LA DIEM KY DI. Jacobian chan o goi = 0 rad co
   cond ~ 2e6, sigma_min ~ 9e-7. Uoc luong luc tiep xuc (can nghich dao J)
   sai RMS 99 N; gap goi 0.3 rad thi con 1.4 N. LUON dung goi chung.

2. UOC LUONG LUC BO QUA QUAN TINH (M*qddot). Chi dung o che do tua tinh /
   gan tinh. Luc bi day manh sai so len ~1600 N. Khi DI BO phai dung cam bien
   luc ban chan hoac dua qddot vao. Van toc (v_body) KHONG bi anh huong -
   no la phep chieu thuan, khong nghich dao gi.
"""
import numpy as np
import pinocchio as pin

# Thu tu khop cua Unitree SDK (G1JointIndex) - trung khop thu tu trong URDF
LEG_JOINT_IDX = {"left": list(range(0, 6)), "right": list(range(6, 12))}
FOOT_FRAMES = {"left": "left_ankle_roll_link", "right": "right_ankle_roll_link"}
N_MOTOR = 29


class LegOdometry:
    def __init__(self, urdf_path, dls_lambda=1e-3,
                 contact_on=60.0, contact_off=30.0, fusion="force"):
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        if self.model.nv != 6 + N_MOTOR:
            raise ValueError(f"URDF co nv={self.model.nv}, mong doi {6 + N_MOTOR} "
                             f"(g1_29dof). Kiem tra lai duong dan URDF.")
        self.sides = ("left", "right")
        self.fids = {s: self.model.getFrameId(FOOT_FRAMES[s]) for s in self.sides}
        # chi so trong vector nv: 6 dof than noi truoc, roi 29 khop theo thu tu SDK
        self.vidx = {s: np.array(LEG_JOINT_IDX[s]) + 6 for s in self.sides}
        self.midx = {s: np.array(LEG_JOINT_IDX[s]) for s in self.sides}
        self.dls_lambda = dls_lambda
        self.contact_on, self.contact_off = contact_on, contact_off
        if fusion not in ("force", "mean"):
            raise ValueError(f"fusion phai la 'force' hoac 'mean', nhan '{fusion}'")
        self.fusion = fusion
        self.in_contact = {s: True for s in self.sides}
        self._q = pin.neutral(self.model)

    def update(self, q_motor, dq_motor, gyro, tau_est, quat_wxyz):
        """Tat ca dau vao lay tu rt/lowstate.

        q_motor, dq_motor, tau_est : (29,) theo thu tu G1JointIndex
        gyro                       : (3,) rad/s, he than (imu_state.gyroscope)
        quat_wxyz                  : (4,) w,x,y,z (imu_state.quaternion)

        Tra ve dict: v_body (3,), fz {side: N}, contact {side: bool}, n_contact
        """
        q_motor = np.asarray(q_motor, dtype=float)
        dq_motor = np.asarray(dq_motor, dtype=float)
        gyro = np.asarray(gyro, dtype=float)
        tau_est = np.asarray(tau_est, dtype=float)

        # --- cau hinh FK: than tai goc, huong don vi (pinocchio dung x,y,z,w) ---
        self._q[:3] = 0.0
        self._q[3:7] = [0.0, 0.0, 0.0, 1.0]
        self._q[7:] = q_motor
        pin.forwardKinematics(self.model, self.data, self._q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, self._q)

        # --- mo-men trong luc cua chan: trong luc quay ve he than (can huong IMU) ---
        w, x, y, z = quat_wxyz
        R = pin.Quaternion(float(w), float(x), float(y), float(z)).normalized().matrix()
        self.model.gravity.linear = R.T @ np.array([0.0, 0.0, -9.81])
        pin.computeGeneralizedGravity(self.model, self.data, self._q)
        tau_g = self.data.g.copy()

        v_est, fz, wrench = {}, {}, {}
        for s in self.sides:
            fid, vi, mi = self.fids[s], self.vidx[s], self.midx[s]
            p_BF = self.data.oMf[fid].translation
            J = pin.getFrameJacobian(self.model, self.data, fid, pin.LOCAL_WORLD_ALIGNED)
            J6 = J[:, vi]                                   # 6x6
            # v_B = -(w x p + J_lin qdot)
            v_est[s] = -(np.cross(gyro, p_BF) + J6[:3] @ dq_motor[mi])
            # wrench tiep xuc 6D (luc diem 3D tai goc co chan khong sinh noi
            # mo-men co chan = fz*(CoP - co chan) -> he mau thuan, nghiem rac)
            rhs = tau_g[vi] - tau_est[mi]
            A = J6 @ J6.T + self.dls_lambda ** 2 * np.eye(6)
            wr = np.linalg.solve(A, J6 @ rhs)
            fz[s] = float((R @ wr[:3])[2])
            # wrench day du trong he THE GIOI: luc 3 + mo-men 3.
            # Dung de doi chieu voi nghiem QP cua WBC - chi so fz khong du de
            # biet hai ben co dang tao LUC NOI (hai chan ep/keo nhau) hay khong.
            wrench[s] = np.concatenate([R @ wr[:3], R @ wr[3:]])

        # --- tre nguong (hysteresis) de khong nhay trang thai ---
        for s in self.sides:
            if self.in_contact[s]:
                self.in_contact[s] = fz[s] > self.contact_off
            else:
                self.in_contact[s] = fz[s] > self.contact_on

        # --- hop nhat hai chan ---
        # fusion="mean" : trung binh cong. Toi uu khi CA HAI chan deu tua tot -
        #   trung binh hai uoc luong doc lap giam nhieu sqrt(2) lan.
        #   Do trong mo phong: RMS 2.64 mm/s (so voi 2.95 khi dung "force").
        # fusion="force": trong so theo fz. GIA THUYET, chua chung minh.
        #   Do tren robot that (26s, dung bang bo can bang cua hang): robot dao
        #   ngang 1.63Hz, trong luong dich qua lai hai chan bien do 75N dinh-dinh.
        #   tuong quan(chenh luc, v_y cua Unitree) = -0.69
        #   tuong quan(chenh luc, v_y cua ta)      = -0.06   <- khong lien quan
        #   tuong quan(chenh luc, SAI LECH hai ben)= +0.59
        #   Nghi ngo: chan dang NHE TAI bap benh mep -> goc co chan khong dung
        #   yen, gia dinh "chan tua khong truot" bi vi pham, nhung trung binh
        #   cong van coi no ngang chan dang chiu tai.
        #   CHUA KIEM CHUNG duoc vi thieu van toc TUNG CHAN trong log. Da them
        #   vao /leg_contact + CSV; buoi do sau chay ca hai che do roi so.
        stance = [s for s in self.sides if self.in_contact[s]]
        if not stance:                       # bay ca hai chan: giu phep do cu
            stance = list(self.sides)
        if self.fusion == "force":
            w = np.array([max(fz[s], 0.0) for s in stance])
            if w.sum() < 1e-6:
                w = np.ones(len(stance))
        else:
            w = np.ones(len(stance))
        w = w / w.sum()
        v_body = np.einsum("i,ij->j", w, np.array([v_est[s] for s in stance]))

        return dict(v_body=v_body, fz=fz, wrench=wrench, contact=dict(self.in_contact),
                    n_contact=len(stance), v_per_foot=v_est, weights=dict(zip(stance, w)))
