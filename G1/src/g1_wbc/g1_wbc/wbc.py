"""WBC dong luc hoc giu thang bang cho G1, dung Pinocchio.

Cung cong thuc voi week10_wbc_balance.py (ban MuJoCo, da kiem chung doi chieu
ly thuyet capture point), nhung chay tren mo hinh URDF + tai trong that, va
nhan trang thai tu rt/lowstate thay vi tu mo phong.

An so cua QP gom CA gia toc khop LAN luc tiep xuc:

    z = [ q_ddot (nv=35) ; f (8 diem x 3 = 24) ]      -> 59 bien

RANG BUOC CUNG:
  (1) Dong luc hoc than noi - 6 hang dau khong co mo-men truyen dong:
          M[0:6,:] q_ddot + h[0:6] = J_c^T[0:6,:] f
      Chi luc tiep xuc moi doi duoc dong luong toan robot.
  (2) Ban chan khong truot - rang buoc 6D MOI BAN CHAN (khong phai 3D moi diem:
      4 diem tren mot ban chan CUNG chi cho hang 6, dat 12 rang buoc diem se
      phu thuoc tuyen tinh va Goldfarb-Idnani bao "constraints are inconsistent").
  (3) Non ma sat tuyen tinh hoa: |fx|,|fy| <= mu*fz, fz >= 0.
      Tu dong ep CoP nam trong da giac do vi 8 diem chi day duoc.
  (4) Gioi han mo-men.

MUC TIEU dat dang centroidal, thang len luc (tranh phai tinh Jdot_com):
    tinh tien:  sum(f_i)             = m*(a_com_des - g)
    quay:       sum((p_i - c) x f_i) = Ldot_des
    tu the:     q_ddot[khop] = Kp(q_ref - q) - Kd*qdot

TRANG THAI lay tu rt/lowstate, KHONG can biet vi tri tuyet doi:
  - q[0:3]  = 0, NHUNG cac dai luong VI TRI (com, diem tiep xuc) duoc NEO VAO
      BAN CHAN truoc khi dua vao tac vu CoM. Xem _anchor().

      Vi sao: M va h khong phu thuoc vi tri than (trong luc deu), nen ghim
      q[:3]=0 la vo hai VOI DONG LUC HOC. Nhung tac vu CoM thi KHONG. Khi robot
      nghieng ve truoc quanh co chan, trong mo hinh ghim goc thi ca than lan CoM
      deu "dung yen" con BAN CHAN moi la thu dich chuyen -> CoM so voi goc than
      gan nhu khong doi -> tac vu CoM khong thay sai lech va khong keo lai ->
      robot do tu tu. (Da gap that: nga sau 3.8s trong MuJoCo du moi dai luong
      dong luc hoc deu khop den 1e-5.)

      Neo vao ban chan thi khung tham chieu dung yen cung mat dat, giong het
      truong hop MuJoCo co vi tri than that.
  - q[3:7]  = quaternion IMU
  - v[0:3]  = van toc than HE THAN tu leg odometry  <- dung convention LOCAL
                cua free-flyer Pinocchio, khong phai doi he
  - v[3:6]  = gyro (he than)
"""
import numpy as np
import pinocchio as pin
from quadprog import solve_qp

# Vi tri diem tiep xuc trong he ban chan, lay tu mujoco_menagerie g1.xml
# (geom class="foot", sphere r=0.005 -> diem cham = tam - 0.005 theo z).
# DA KIEM CHUNG frame ankle_roll_link cua URDF va MJCF trung nhau den 1e-6 m.
CONTACT_LOCAL = np.array([
    [-0.05,  0.025, -0.035],
    [-0.05, -0.025, -0.035],
    [ 0.12,  0.030, -0.035],
    [ 0.12, -0.030, -0.035],
])
FOOT_FRAMES = ("left_ankle_roll_link", "right_ankle_roll_link")
N_MOTOR = 29


class BalanceWBC:
    def __init__(self, urdf_path, payload_yaml=None,
                 mu=0.6, fz_min=1.0, dq_max=None,
                 kp_com=60.0, kd_com=15.0, kp_ori=250.0, kd_ang=40.0,
                 kp_q=100.0, kd_q=20.0,
                 w_com=60.0, w_ang=12.0, w_post=1.0, w_f=1e-3, w_qacc=1e-4,
                 w_tau=1.0,
                 kd_contact=30.0):
        self.model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        if payload_yaml:
            from g1_leg_odometry.payload import apply_payload
            apply_payload(self.model, payload_yaml)

        # Them 8 frame diem tiep xuc -> Pinocchio tu tinh Jacobian va so hang troi
        self.contact_fids = []
        for fname in FOOT_FRAMES:
            fid = self.model.getFrameId(fname)
            fr = self.model.frames[fid]
            for k, p in enumerate(CONTACT_LOCAL):
                nf = pin.Frame(f"{fname}_contact{k}", fr.parentJoint, fid,
                               fr.placement * pin.SE3(np.eye(3), p),
                               pin.FrameType.OP_FRAME)
                self.contact_fids.append(self.model.addFrame(nf))
        self.foot_fids = [self.model.getFrameId(f) for f in FOOT_FRAMES]
        self.data = self.model.createData()

        self.nv = self.model.nv
        self.nc = len(self.contact_fids)
        self.nf = 3 * self.nc
        self.nz = self.nv + self.nf
        self.mass = sum(I.mass for I in self.model.inertias)
        self.g = np.array([0.0, 0.0, -9.81])

        lim = self.model.effortLimit[6:]
        self.tau_lo, self.tau_hi = -lim, lim

        self.mu, self.fz_min, self.kd_contact = mu, fz_min, kd_contact
        self.kp_com, self.kd_com = kp_com, kd_com
        self.kp_ori, self.kd_ang = kp_ori, kd_ang
        self.kp_q, self.kd_q = kp_q, kd_q
        self.w_com, self.w_ang = w_com, w_ang
        self.w_post, self.w_f, self.w_qacc = w_post, w_f, w_qacc
        # Phat mo-men khop. 0 = tat (giu nguyen hanh vi cu).
        # Vi sao can: dung HAI CHAN la bai toan VO DINH TINH HOC - nhieu cach
        # phan bo luc giua hai chan cho cung mot hop luc, nhung mo-men khop
        # khac han nhau. Ham muc tieu cu khong he phat mo-men nen QP chon bua
        # mot nghiem trong ho do. Do tren robot that: ta giu 33 Nm o hong
        # trong khi Unitree chi ~2 Nm luc dung yen.
        self.w_tau = w_tau

        self.keep_qp = False
        self.last_qp = None
        self.q_ref = None
        self.com_des = None
        self.quat_des = None
        self.anchor_ref = None      # vi tri trung binh diem tiep xuc luc chot moc
        self._q = pin.neutral(self.model)
        self._v = np.zeros(self.nv)

    # ---------------- moc tham chieu ----------------
    def capture_reference(self, q_motor, quat_wxyz):
        """Chot tu the / CoM / huong hien tai lam muc tieu."""
        self._assemble(q_motor, np.zeros(N_MOTOR), quat_wxyz, np.zeros(3), np.zeros(3))
        pin.forwardKinematics(self.model, self.data, self._q)
        pin.updateFramePlacements(self.model, self.data)
        pin.centerOfMass(self.model, self.data, self._q)
        pts = np.array([self.data.oMf[f].translation for f in self.contact_fids])
        self.anchor_ref = pts.mean(axis=0).copy()
        self.q_ref = np.asarray(q_motor, dtype=float).copy()
        self.com_des = self.data.com[0].copy()
        w, x, y, z = quat_wxyz
        self.quat_des = pin.Quaternion(float(w), float(x), float(y), float(z)).normalized()

    def _assemble(self, q_motor, dq_motor, quat_wxyz, gyro, v_body):
        w, x, y, z = quat_wxyz
        qq = pin.Quaternion(float(w), float(x), float(y), float(z)).normalized()
        self._q[:3] = 0.0
        self._q[3:7] = qq.coeffs()          # pinocchio dung (x,y,z,w)
        self._q[7:] = q_motor
        self._v[:3] = v_body                # free-flyer: LOCAL = he than
        self._v[3:6] = gyro
        self._v[6:] = dq_motor
        return qq

    def update_reference(self, q_motor, dt, tau_secs):
        """Cho moc tu the TROI CHAM theo tu the hien tai (hang so thoi gian tau_secs).

        Vi sao can: q_ref duoc chot MOT LAN luc khoi dong. Do tren robot that,
        sau vai cu day robot on dinh o tu the lech han -0.085 rad o hong va
        +0.07 rad o goi, VA KHONG BAO GIO TRO LAI. Bo dieu khien cua hang chap
        nhan tu the moi; con tac vu tu the cua ta thi chong lai do lech vinh
        vien do mai mai - 100 x 0.085 = 8.5 rad/s^2 lien tuc, ra ~35 Nm o hong
        trong khi Unitree chi 1.3 Nm, va KHONG he tuong quan voi sai lech CoM
        (-0.08 so voi +0.51 cua Unitree).

        Khong phai sai HE SO ma sai cai MOC.

        Co y tach khoi solve(): solve() phai la ham THUAN voi moc da cho, neu
        khong thi test vector (chay 55 tu the roi rac khong theo thu tu thoi
        gian) se khong tai lap duoc.
        """
        if tau_secs <= 0.0:
            return
        a = min(dt / tau_secs, 1.0)
        self.q_ref += a * (np.asarray(q_motor, dtype=float) - self.q_ref)

    # ---------------- giai ----------------
    def solve(self, q_motor, dq_motor, quat_wxyz, gyro, v_body, contact=(True, True)):
        if self.q_ref is None:
            raise RuntimeError("phai goi capture_reference() truoc")
        m, d, nv, nz = self.model, self.data, self.nv, self.nz
        qq = self._assemble(q_motor, dq_motor, quat_wxyz, gyro, v_body)
        q, v = self._q, self._v

        pin.computeAllTerms(m, d, q, v)
        pin.forwardKinematics(m, d, q, v, np.zeros(nv))   # qacc=0 -> lay so hang troi
        pin.updateFramePlacements(m, d)
        M = d.M.copy()
        M = np.triu(M) + np.triu(M, 1).T
        h = d.nle.copy()
        com = d.com[0].copy()
        vcom = d.vcom[0].copy()
        L = pin.computeCentroidalMomentum(m, d, q, v).angular.copy()

        # --- Jacobian tiep xuc (8 diem, 3D) va rang buoc ban chan (2 x 6D) ---
        Jc = np.zeros((self.nf, nv))
        pts = np.zeros((self.nc, 3))
        for i, fid in enumerate(self.contact_fids):
            pts[i] = d.oMf[fid].translation
            Jc[3 * i:3 * i + 3] = pin.getFrameJacobian(
                m, d, fid, pin.LOCAL_WORLD_ALIGNED)[:3]
        # --- NEO VAO BAN CHAN ---
        # q[:3]=0 nen moi vi tri deu tinh trong he gan voi than. Dich tat ca sao
        # cho trung binh diem tiep xuc trung voi luc chot moc -> khung tham chieu
        # dung yen cung mat dat. Chi dich VI TRI; M, h, Jacobian khong doi.
        Jf = np.zeros((12, nv))
        drift_f = np.zeros(12)
        for i, fid in enumerate(self.foot_fids):
            Jf[6 * i:6 * i + 6] = pin.getFrameJacobian(m, d, fid, pin.LOCAL_WORLD_ALIGNED)
            a = pin.getFrameClassicalAcceleration(m, d, fid, pin.LOCAL_WORLD_ALIGNED)
            drift_f[6 * i:6 * i + 3] = a.linear
            drift_f[6 * i + 3:6 * i + 6] = a.angular

        offset = self.anchor_ref - pts.mean(axis=0)
        pts = pts + offset
        com = com + offset

        JcT = Jc.T
        # tau = T z + h_a. Tinh som vi CA ham muc tieu lan rang buoc deu dung.
        T = np.zeros((N_MOTOR, nz))
        T[:, :nv] = M[6:]
        T[:, nv:] = -JcT[6:]
        h_a = h[6:]

        # ================= ham muc tieu =================
        H = np.zeros((nz, nz))
        g_lin = np.zeros(nz)

        a_com_des = self.kp_com * (self.com_des - com) - self.kd_com * vcom
        A_lin = np.zeros((3, nz))
        for i in range(self.nc):
            A_lin[:, nv + 3 * i: nv + 3 * i + 3] = np.eye(3)
        b_lin = self.mass * (a_com_des - self.g)
        H += self.w_com * A_lin.T @ A_lin
        g_lin += self.w_com * A_lin.T @ b_lin

        ori_err = pin.log3((self.quat_des * qq.inverse()).matrix())
        A_ang = np.zeros((3, nz))
        for i in range(self.nc):
            A_ang[:, nv + 3 * i: nv + 3 * i + 3] = pin.skew(pts[i] - com)
        b_ang = self.kp_ori * ori_err - self.kd_ang * L
        H += self.w_ang * A_ang.T @ A_ang
        g_lin += self.w_ang * A_ang.T @ b_ang

        A_post = np.zeros((N_MOTOR, nz))
        A_post[:, 6:6 + N_MOTOR] = np.eye(N_MOTOR)
        b_post = self.kp_q * (self.q_ref - q_motor) - self.kd_q * dq_motor
        H += self.w_post * A_post.T @ A_post
        g_lin += self.w_post * A_post.T @ b_post

        if self.w_tau > 0.0:
            # (w/2)||T z + h_a||^2  -> H += w T'T ,  g += w T'(-h_a)
            H += self.w_tau * T.T @ T
            g_lin += self.w_tau * T.T @ (-h_a)

        f_nom = np.tile([0.0, 0.0, self.mass * 9.81 / self.nc], self.nc)
        H[nv:, nv:] += self.w_f * np.eye(self.nf)
        g_lin[nv:] += self.w_f * f_nom
        H[:nv, :nv] += self.w_qacc * np.eye(nv)
        H += 1e-8 * np.eye(nz)

        # ================= rang buoc dang thuc =================
        Ceq1 = np.zeros((6, nz)); Ceq1[:, :nv] = M[:6]; Ceq1[:, nv:] = -JcT[:6]
        beq1 = -h[:6]
        Ceq2 = np.zeros((12, nz)); Ceq2[:, :nv] = Jf
        beq2 = -drift_f - self.kd_contact * (Jf @ v)
        Ceq = np.vstack([Ceq1, Ceq2]); beq = np.concatenate([beq1, beq2])

        # ================= bat dang thuc (dang >= 0) =================
        rows, rhs = [], []
        for i in range(self.nc):
            s = nv + 3 * i
            foot_on = contact[0] if i < 4 else contact[1]
            e = np.zeros(nz); e[s + 2] = 1.0
            rows.append(e); rhs.append(self.fz_min if foot_on else 0.0)
            if not foot_on:                      # chan bay: ep fz = 0
                rows.append(-e); rhs.append(-1e-6)
            for k, sgn in ((0, 1.0), (0, -1.0), (1, 1.0), (1, -1.0)):
                r = np.zeros(nz); r[s + 2] = self.mu; r[s + k] = sgn
                rows.append(r); rhs.append(0.0)
        for k in range(N_MOTOR):
            rows.append(T[k]);  rhs.append(self.tau_lo[k] - h_a[k])
            rows.append(-T[k]); rhs.append(h_a[k] - self.tau_hi[k])
        Cin = np.array(rows); bin_ = np.array(rhs)

        C = np.vstack([Ceq, Cin]).T
        b = np.concatenate([beq, bin_])
        if self.keep_qp:
            # Giu lai bai toan de kiem tra mot nghiem BEN NGOAI co that su toi uu
            # khong - dung khi doi chieu ban C++ ma lech khong chiu nho di.
            self.last_qp = dict(H=H.copy(), g=g_lin.copy(), Ceq=Ceq.copy(),
                                beq=beq.copy(), Cin=Cin.copy(), bin=bin_.copy())
        z = solve_qp(H, g_lin, C, b, meq=Ceq.shape[0])[0]

        qacc, f = z[:nv], z[nv:]
        tau = M[6:] @ qacc + h_a - JcT[6:] @ f
        F = f.reshape(self.nc, 3)
        return tau, dict(qacc=qacc, f=f, com=com, com_err=com - self.com_des,
                         ori_err=ori_err, L=L, fz=f[2::3],
                         fz_left=f[2:12:3].sum(), fz_right=f[14::3].sum(),
                         F_left=F[:4].sum(axis=0), F_right=F[4:].sum(axis=0))
