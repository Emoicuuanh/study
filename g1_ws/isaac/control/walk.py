"""Mo hinh DANG DI DONG HOC - tai tao nhieu cua buoc chan len cam bien.

DAY KHONG PHAI BO DIEU KHIEN DI BO THAT. Noi ro de khong nham:
  KHONG co: luc tiep xuc chan-san, giu thang bang, phan ung khi bi day,
            dong luc hoc toan than.
  CO:       than nhun len xuong, lac truoc-sau va trai-phai, xoay nhe
            theo nhip buoc, chan cu dong theo dang di.

VI SAO VAN HUU ICH: voi SLAM, thu quyet dinh chat luong ban do la
CHUYEN DONG CUA CAM BIEN, khong phai co che sinh ra chuyen dong do.
Mot LiDAR bi nhun 3cm va lac 3 do theo nhip buoc se gay meo vong quet
y het nhau, du chuyen dong den tu buoc chan that hay tu mo hinh nay.
Nho vay co the do duoc SLAM xau di bao nhieu, va thu cach chua (IMU +
go meo), TRUOC khi phai giai bai toan di bo.

BIEN DO LAY THEO SO LIEU DI BO CUA HUMANOID CO G1:
  nhun doc      1.5 cm bien do (3 cm dinh-dinh)   tan so = f_buoc
  lac ngang     2.0 cm                             tan so = f_buoc / 2
  lac trai-phai (roll)  2.5 do                     tan so = f_buoc / 2
  chuc truoc-sau (pitch) 1.5 do                    tan so = f_buoc
  xoay (yaw)    1.5 do                             tan so = f_buoc / 2

Vi sao tan so khac nhau: moi BUOC lam than nhun mot lan (nen nhun theo
f_buoc), nhung phai di het HAI buoc than moi lac sang trai roi ve phai
mot vong (nen lac theo f_buoc/2).
"""
import numpy as np


class WalkController:
    """Cho robot di theo lenh van toc, KEM nhieu dao dong cua dang di.

    So sanh voi SlideController: cung nhan (vx, vy, omega), nhung them
    dao dong nhun/lac/xoay theo nhip buoc, va cu dong khop chan.
    """

    def __init__(self, robot, base_height=0.78, dt=1.0 / 60.0,
                 step_freq=1.8,           # so buoc moi giay
                 bob_amp=0.015,           # nhun doc (m)
                 sway_amp=0.020,          # lac ngang (m)
                 roll_amp=np.deg2rad(2.5),
                 pitch_amp=np.deg2rad(1.5),
                 yaw_amp=np.deg2rad(1.5),
                 leg_swing=0.35,          # bien do vung chan (rad)
                 disturb=1.0,             # he so nhieu: 0 = tat het, 1 = nhu that
                 x=0.0, y=0.0, yaw=0.0):
        self.robot = robot
        self.h0 = base_height
        self.dt = dt
        self.f = step_freq
        self.k = disturb
        self.bob, self.sway = bob_amp, sway_amp
        self.roll_a, self.pitch_a, self.yaw_a = roll_amp, pitch_amp, yaw_amp
        self.leg_swing = leg_swing
        self.x, self.y, self.yaw = x, y, yaw
        self.t = 0.0
        self._joint_idx = None

    # ---- tim chi so khop chan (goi sau world.reset()) ----
    def _find_leg_joints(self):
        names = list(self.robot.dof_names)
        want = {
            "lhp": "left_hip_pitch_joint",   "rhp": "right_hip_pitch_joint",
            "lk":  "left_knee_joint",        "rk":  "right_knee_joint",
            "lap": "left_ankle_pitch_joint", "rap": "right_ankle_pitch_joint",
        }
        self._joint_idx = {k: names.index(v) for k, v in want.items() if v in names}

    def hold_pose(self, kp=400.0, kd=40.0):
        n = self.robot.num_dof
        self.robot.set_gains(kps=np.full((1, n), kp), kds=np.full((1, n), kd))
        self.robot.set_joint_position_targets(np.zeros((1, n)))
        self._find_leg_joints()

    def step(self, vx=0.0, vy=0.0, omega=0.0):
        self.t += self.dt
        # --- di chuyen theo lenh van toc (giong SlideController) ---
        self.yaw += omega * self.dt
        self.x += (vx * np.cos(self.yaw) - vy * np.sin(self.yaw)) * self.dt
        self.y += (vx * np.sin(self.yaw) + vy * np.cos(self.yaw)) * self.dt

        # --- dao dong theo nhip buoc ---
        w_step = 2 * np.pi * self.f            # nhip BUOC
        w_gait = np.pi * self.f                # nhip CHU KY (2 buoc)
        k = self.k

        z = self.h0 - k * self.bob * (1 - np.cos(w_step * self.t))     # nhun, luon <= h0
        lateral = k * self.sway * np.sin(w_gait * self.t)               # lac ngang
        roll = k * self.roll_a * np.sin(w_gait * self.t)
        pitch = k * self.pitch_a * np.sin(w_step * self.t)
        yaw_osc = k * self.yaw_a * np.sin(w_gait * self.t)

        # lac ngang la theo huong NGANG cua robot (vuong goc huong di)
        yaw_tot = self.yaw + yaw_osc
        px = self.x - lateral * np.sin(yaw_tot)
        py = self.y + lateral * np.cos(yaw_tot)

        # --- ghep roll/pitch/yaw thanh quaternion (w,x,y,z) ---
        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw_tot / 2), np.sin(yaw_tot / 2)
        quat = np.array([[
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]])
        self.robot.set_world_poses(positions=np.array([[px, py, z]]), orientations=quat)
        self.robot.set_velocities(np.zeros((1, 6)))

        # --- cu dong khop chan cho giong dang di ---
        if self._joint_idx:
            n = self.robot.num_dof
            tgt = np.zeros((1, n))
            s = self.leg_swing * np.sin(w_gait * self.t)     # hai chan nguoc pha
            # chan tru duoi, chan vung nhac len (dau gia tri duong)
            swing_l = max(0.0, np.sin(w_gait * self.t))
            swing_r = max(0.0, -np.sin(w_gait * self.t))
            J = self._joint_idx
            if "lhp" in J: tgt[0, J["lhp"]] = -s
            if "rhp" in J: tgt[0, J["rhp"]] = +s
            if "lk" in J:  tgt[0, J["lk"]] = 1.0 * swing_l
            if "rk" in J:  tgt[0, J["rk"]] = 1.0 * swing_r
            if "lap" in J: tgt[0, J["lap"]] = -0.5 * swing_l
            if "rap" in J: tgt[0, J["rap"]] = -0.5 * swing_r
            self.robot.set_joint_position_targets(tgt)

        return dict(z=z, roll=roll, pitch=pitch, yaw_osc=yaw_osc, lateral=lateral)
