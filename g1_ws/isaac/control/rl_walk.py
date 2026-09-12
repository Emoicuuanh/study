"""Chay policy RL di bo cua Unitree (motion.pt) trong Isaac Sim.

Policy nay lay tu repo chinh thuc unitree_rl_gym:
    deploy/pre_train/g1/motion.pt
Da train san, la TorchScript nen chay duoc o bat ky moi truong nao - chi
can dung DINH DANG DAU VAO va DUNG THU TU KHOP.

DAY LA DI BO THAT: chan dap san, co luc tiep xuc, tu giu thang bang, bi
day thi loang choang roi hoi phuc. Khac hoan toan mo hinh dong hoc trong
control/walk.py (chi ap dat dao dong len than).

=== BON DIEU PHAI KHOP CHINH XAC ===

1. THU TU KHOP. Policy duoc train voi thu tu cua model MuJoCo:
       left:  hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll
       right: (y het thu tu tren)
   Isaac Sim xep khop theo thu tu KHAC (xen ke hai chan + eo). Nen phai
   tra theo TEN, khong duoc dung chi so co dinh.

2. OBSERVATION 47 chieu, dung thu tu:
       [ 0: 3]  van toc goc than, TRONG HE THAN     x 0.25
       [ 3: 6]  huong trong luc chieu vao he than
       [ 6: 9]  lenh (vx, vy, omega)                x (2.0, 2.0, 0.25)
       [ 9:21]  goc khop - goc mac dinh             x 1.0
       [21:33]  van toc khop                        x 0.05
       [33:45]  action cua buoc TRUOC
       [45:47]  sin/cos cua pha buoc (chu ky 0.8s)

3. TAN SO. Policy train voi vat ly 500Hz, dieu khien 50Hz (decimation 10).
   Chay sai tan so -> policy hanh xu khac han.

4. TRONG LUC PHAI BAT. Che do truot truoc day tat trong luc; o day robot
   phai chiu trong luc that moi di duoc.

CHU Y VE HE QUY CHIEU:
   MuJoCo tra van toc goc cua free joint trong HE THAN.
   Isaac Sim tra trong HE THE GIOI.
   -> phai quay ve he than: omega_than = R^T . omega_the_gioi
   Neu bo qua buoc nay, policy nhan sai dau vao va robot nga ngay.
"""
import numpy as np
import torch

# thu tu khop dung nhu luc train (xem docstring)
POLICY_JOINT_ORDER = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]

DEFAULT_ANGLES = np.array([-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
                           -0.1, 0.0, 0.0, 0.3, -0.2, 0.0], dtype=np.float32)
KPS = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float32)
KDS = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float32)

ANG_VEL_SCALE = 0.25
DOF_POS_SCALE = 1.0
DOF_VEL_SCALE = 0.05
ACTION_SCALE = 0.25
CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
GAIT_PERIOD = 0.8
# DA THU va THAT BAI: dong bang pha buoc khi lenh~0 lam TROI TE HON
#   khong dong bang: 0.212 m/s     co dong bang: 0.441 m/s
# Nen de -1.0 = TAT co che nay. Giu lai code de biet da thu.
STAND_THRESHOLD = -1.0
CONTROL_HZ = 50.0                # policy duoc train o 50Hz - PHAI dung dung
DEG_PER_RAD = 57.29577951308232  # 180/pi - xem configure_force_drives()


def configure_force_drives(robot_prim="/World/G1", verbose=True):
    """Cau hinh drive cua Isaac Sim de DONG VAI DRIVER DONG CO nhu robot that.

    GOI TRUOC world.reset(). Sau reset thi physics view da tao xong, sua
    thuoc tinh USD khong con tac dung.

    === VI SAO LAN TRUOC THU POSITION DRIVE THI ROBOT SUP (0.80 -> 0.06 m) ===
    Da do lai bang tests/probe_drive.py, phat hien BA van de cung luc:

      1. DRIVE TYPE = "acceleration" (mac dinh cua trinh nhap URDF).
         Voi loai nay PhysX nhan them QUAN TINH cua khop:
             tau = I * (kp*(target-q) - kd*dq)
         Cung mot kp=150 se cho mo-men khac han o moi khop, tuy quan tinh.
         Loai "force" moi dung: stiffness tinh thang bang N*m/rad, giong
         MuJoCo va giong kp gui xuong driver dong co that.

      2. SAI DON VI. Thuoc tinh USD physics:stiffness cua khop quay tinh
         theo DO, con Articulation.set_gains() tinh theo RADIAN:
             USD 625.0  <->  get_gains() 35809.86 = 625 * 180/pi
         Lan truoc dat kp=150 trong khi nen dang la 35809 -> yeu hon 238
         lan -> khong du do trong luong -> sup.

      3. damping = 0.0 trong USD. Khong co giam chan thi khop dao dong
         khong tat.

    So sanh voi robot that (deploy_real/configs/g1.yaml): may tinh onboard
    gui xuong driver (q, kp, kd) roi driver tu tinh PD o ~1-4 kHz. Dat drive
    o che do "force" voi dung kp/kd chinh la tai hien co che do - Isaac chay
    PD ben trong buoc vat ly, 500 Hz.
    """
    import omni.usd
    from pxr import UsdPhysics
    stage = omni.usd.get_context().get_stage()
    done = 0
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(robot_prim):
            continue
        d = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not d:
            continue
        name = prim.GetName()
        if name not in POLICY_JOINT_ORDER:
            continue
        i = POLICY_JOINT_ORDER.index(name)
        # DEG_PER_RAD: thuoc tinh USD tinh theo do, KPS/KDS cua ta theo radian
        d.GetTypeAttr().Set("force")
        d.GetStiffnessAttr().Set(float(KPS[i]) / DEG_PER_RAD)
        d.GetDampingAttr().Set(float(KDS[i]) / DEG_PER_RAD)
        done += 1
    if verbose:
        print(f">>> drive: da chuyen {done}/12 khop chan sang che do 'force' "
              f"(Isaac dong vai driver dong co)", flush=True)
    return done


def _quat_to_R(q_wxyz):
    """Quaternion (w,x,y,z) -> ma tran quay 3x3."""
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def gravity_orientation(q_wxyz):
    """Huong trong luc chieu vao he than - dung dung cong thuc cua Unitree."""
    qw, qx, qy, qz = q_wxyz
    return np.array([
        2 * (-qz * qx + qw * qy),
        -2 * (qz * qy + qw * qx),
        1 - 2 * (qw * qw + qz * qz),
    ], dtype=np.float32)


class RLWalkController:
    """Dieu khien G1 di bo bang policy RL da train san cua Unitree."""

    def __init__(self, robot, policy_path, loop_dt=0.002,
                 other_joints_kp=200.0, other_joints_kd=20.0,
                 drive_mode="torque"):
        """loop_dt = khoang thoi gian MOI LAN goi step() lam sim tien len.

        BAY QUAN TRONG: trong Isaac Sim, world.step(render=True) tien mot
        khoang RENDERING_DT (khong phai physics_dt) va tu chay nhieu buoc
        vat ly ben trong. Neu tinh decimation theo physics_dt thi policy
        se chay sai tan so.
        Trieu chung da gap: rendering_dt=0.02 nhung decimation van de 10
        -> policy chay 5Hz thay vi 50Hz -> robot NGA ngay, du cung policy
        do chay tot khi step(render=False).
        """
        self.robot = robot
        self.dt = loop_dt
        # drive_mode:
        #   "torque"   - ta tu tinh PD trong Python 500Hz roi ap mo-men.
        #                Giong ban tham chieu MuJoCo cua Unitree.
        #   "position" - Isaac Sim dong vai DRIVER DONG CO: ta chi gui goc
        #                muc tieu, drive tu tinh PD ben trong buoc vat ly.
        #                Giong kien truc robot THAT (may chinh gui q/kp/kd,
        #                driver tu tinh). Can goi configure_force_drives()
        #                TRUOC world.reset().
        if drive_mode not in ("torque", "position"):
            raise ValueError(f"drive_mode phai la torque hoac position, nhan: {drive_mode}")
        self.drive_mode = drive_mode
        self.decimation = max(1, int(round((1.0 / CONTROL_HZ) / loop_dt)))
        self.policy = torch.jit.load(policy_path)
        self.policy.eval()

        names = list(robot.dof_names)
        missing = [n for n in POLICY_JOINT_ORDER if n not in names]
        if missing:
            raise RuntimeError(f"Model thieu khop: {missing}")
        self.leg_idx = np.array([names.index(n) for n in POLICY_JOINT_ORDER])

        self.n_dof = robot.num_dof
        self.action = np.zeros(12, dtype=np.float32)
        self.obs = np.zeros(47, dtype=np.float32)
        self.target_leg = DEFAULT_ANGLES.copy()
        self.counter = 0
        self.other_kp, self.other_kd = other_joints_kp, other_joints_kd

    def setup_gains(self):
        """Chuan bi dieu khien MO-MEN cho 12 khop chan.

        VI SAO DUNG MO-MEN chu khong dung position drive:
          Ban goc cua Unitree (deploy_mujoco.py) dung actuator <motor> tuc
          MO-MEN TRUC TIEP, va tinh tay:
              tau = kp*(target - q) - kd*dq      roi ghi vao d.ctrl
          Neu ta dung position drive cua Isaac Sim thi ve LY THUYET tuong
          duong, nhung thuc te phu thuoc cach Isaac hieu don vi gain va
          gioi han maxForce cua tung khop -> da thu va robot SUP xuong
          (z tu 0.80 xuong 0.06 m).
          Ap mo-men truc tiep loai bo moi mo ho do -> trung thuc voi ban goc.

        Cach lam: dat stiffness/damping cua drive = 0 (de drive khong "danh"
        lai mo-men ta ap), roi moi buoc goi set_joint_efforts().
        """
        kps = np.zeros((1, self.n_dof), dtype=np.float32)
        kds = np.zeros((1, self.n_dof), dtype=np.float32)
        if self.drive_mode == "position":
            # Isaac dong vai driver: BAT drive voi dung kp/kd cua policy.
            # set_gains() nhan don vi RADIAN (khac thuoc tinh USD tinh theo
            # do - xem configure_force_drives), nen truyen thang KPS/KDS.
            kps[0, self.leg_idx] = KPS
            kds[0, self.leg_idx] = KDS
        # cac khop KHONG thuoc chan van giu bang PD (neu model co)
        other = np.setdiff1d(np.arange(self.n_dof), self.leg_idx)
        kps[0, other] = self.other_kp
        kds[0, other] = self.other_kd
        self.robot.set_gains(kps=kps, kds=kds)
        got_kp, got_kd = self.robot.get_gains()
        print(f">>> drive_mode = {self.drive_mode}  |  gain thuc te khop chan: "
              f"kp={np.asarray(got_kp[0])[self.leg_idx].round(1)} "
              f"kd={np.asarray(got_kd[0])[self.leg_idx].round(1)}", flush=True)

        q0 = np.zeros((1, self.n_dof), dtype=np.float32)
        q0[0, self.leg_idx] = DEFAULT_ANGLES
        self.robot.set_joint_positions(q0)
        self.robot.set_joint_position_targets(q0)

    def step(self, cmd=(0.5, 0.0, 0.0)):
        """Goi MOI vong lap. Policy suy dien o 50Hz (xem self.decimation).

        VE VIEC ROBOT KHONG DUNG YEN KHI LENH = 0 (troi 0.212 m/s):
        Policy nay khong co che do dung - pha buoc luon chay nen no buoc
        lien tuc. Da thu hai cach chua, CA HAI DEU THAT BAI:
            dong bang pha buoc  -> troi 0.441 m/s (TE HON 2 lan)
            giu goc khop co dinh -> robot SUP xuong (chi PD khong du do
                                    trong luong; can thang bang chu dong)
        Nguyen nhan goc KHONG nam o observation ma o SAI LECH MODEL:
            policy train tren G1 12 khop (chi chan, 32 kg)
            Isaac Sim dung G1 43 khop (co eo + 2 tay + ban tay)
        Trong MuJoCo voi model 12 khop dung: troi 0.011 m/s (it hon 20 lan).
        Khong co meo observation nao sua duoc chuyen nay - phai dung model
        khop dung, hoac train lai policy cho model 43 khop.
        """
        self.counter += 1
        if self.counter % self.decimation == 0:
            self._infer(np.asarray(cmd, dtype=np.float32))

        if self.drive_mode == "position":
            # Isaac dong vai DRIVER DONG CO: ta chi gui goc muc tieu, drive
            # tu tinh PD ben trong buoc vat ly. Giong may tinh onboard cua
            # robot that gui (q, kp, kd) xuong driver roi driver tu lo.
            # Khong can doc q/dq, khong can tinh tau.
            tgt = np.zeros((1, self.n_dof), dtype=np.float32)
            tgt[0, self.leg_idx] = self.target_leg
            self.robot.set_joint_position_targets(tgt)
            return

        # PD tinh TAY roi ap mo-men - dung nhu ban goc MuJoCo:
        #     tau = kp*(target - q) - kd*dq
        q = np.asarray(self.robot.get_joint_positions()[0])[self.leg_idx]
        dq = np.asarray(self.robot.get_joint_velocities()[0])[self.leg_idx]
        tau = KPS * (self.target_leg - q) - KDS * dq

        eff = np.zeros((1, self.n_dof), dtype=np.float32)
        eff[0, self.leg_idx] = tau
        self.robot.set_joint_efforts(eff)

    def _infer(self, cmd):
        q_all = self.robot.get_joint_positions()[0]
        dq_all = self.robot.get_joint_velocities()[0]
        pos, quat = self.robot.get_world_poses()
        vel = self.robot.get_velocities()[0]          # [lin(3), ang(3)] he THE GIOI

        q_wxyz = np.asarray(quat[0], dtype=np.float64)
        R = _quat_to_R(q_wxyz)
        omega_body = R.T @ np.asarray(vel[3:6], dtype=np.float64)   # -> he THAN

        qj = (np.asarray(q_all)[self.leg_idx] - DEFAULT_ANGLES) * DOF_POS_SCALE
        dqj = np.asarray(dq_all)[self.leg_idx] * DOF_VEL_SCALE

        # dong bang pha khi lenh ~ 0 -> policy khong nhan nhip buoc -> dung yen
        standing = float(np.abs(cmd).max()) < STAND_THRESHOLD
        if standing:
            sin_p, cos_p = 0.0, 1.0
        else:
            phase = (self.counter * self.dt) % GAIT_PERIOD / GAIT_PERIOD
            sin_p, cos_p = np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)
        o = self.obs
        o[0:3]   = omega_body * ANG_VEL_SCALE
        o[3:6]   = gravity_orientation(q_wxyz)
        o[6:9]   = cmd * CMD_SCALE
        o[9:21]  = qj
        o[21:33] = dqj
        o[33:45] = self.action
        o[45:47] = [sin_p, cos_p]

        with torch.no_grad():
            self.action = self.policy(torch.from_numpy(o).unsqueeze(0)).numpy().squeeze()
        self.target_leg = self.action * ACTION_SCALE + DEFAULT_ANGLES
