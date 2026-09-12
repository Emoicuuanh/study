"""Dieu khien robot TRUOT theo lenh van toc (nhu AGV).

VI SAO TRUOT MA KHONG DI BO THAT - va gioi han cua no:

  Dung duoc cho:
    TANG 0  SLAM 3D           - thuat toan ghep point cloud khong phu
                                thuoc dang di, chi can biet cam bien o dau
    TANG 3  Elevation map     - chi can LiDAR quet dia hinh
    TANG 4  Global planning   - lam viec tren ban do, khong lien quan chan

  KHONG dung duoc cho:
    TANG 2  Footstep planning - bat buoc co chan that, dia hinh that
    TANG 1  Leo cau thang     - perception va locomotion rang buoc chat

  Ngoai ra robot di bo that co LiDAR NHUN LEN XUONG theo nhip buoc, gay
  meo point cloud (motion distortion) - van de that ma SLAM phai xu ly
  bang IMU. Robot truot khong co hien tuong nay -> de hon thuc te.

Cach lam: dat vi tri robot moi buoc theo tich phan van toc, dong thoi
triet tieu van toc de trong luc khong keo lech (do 2.7mm sai so).
"""
import numpy as np


def disable_gravity(robot_prim="/World/G1"):
    """Tat trong luc cho robot dang o che do truot.

    VI SAO CAN - bug that da gap:
      Vong lap la: dat tu the -> chay vat ly 1/60s -> do odometry.
      Trong 1/60s do, trong luc keo hai chan lung lang tao MO-MEN lam
      nghieng hong ~3 do. Odometry doc trang thai SAU khi vat ly chay nen
      ghi nhan goc nghieng do.

      3 do nghe nho nhung o khoang cach 20m se lech hon 1m -> mat san
      trong ban do bi "doc", SLAM hong.

      Trieu chung rat de nham: kiem tra point cloud trong he PELVIS thi
      dep (88% diem dung mat san), nhung qua he ODOM thi hong (10%).
      => Loi khong o cam bien ma o tu the robot bao cao qua odometry.

      Robot dang truot (khong di bo), khong can trong luc -> tat di la
      sach nhat. Khi sang TANG 1-2 (di bo that) thi PHAI bat lai.
    """
    import omni.usd
    from pxr import UsdPhysics, PhysxSchema
    stage = omni.usd.get_context().get_stage()
    n = 0
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(robot_prim) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateDisableGravityAttr(True)
            n += 1
    return n


class SlideController:
    """Tich phan (vx, vy, omega) -> vi tri, roi dat robot vao vi tri do."""

    def __init__(self, robot, base_height=0.80, dt=1.0 / 60.0, x=0.0, y=0.0, yaw=0.0):
        self.robot = robot
        self.h = base_height
        self.dt = dt
        self.x, self.y, self.yaw = x, y, yaw

    def step(self, vx=0.0, vy=0.0, omega=0.0):
        """Tien mot buoc theo lenh van toc (m/s, m/s, rad/s)."""
        self.yaw += omega * self.dt
        self.x += (vx * np.cos(self.yaw) - vy * np.sin(self.yaw)) * self.dt
        self.y += (vx * np.sin(self.yaw) + vy * np.cos(self.yaw)) * self.dt

        self.robot.set_world_poses(
            positions=np.array([[self.x, self.y, self.h]]),
            orientations=np.array([[np.cos(self.yaw / 2), 0.0, 0.0, np.sin(self.yaw / 2)]]),
        )
        # triet tieu van toc: khong cho trong luc keo lech giua cac buoc
        self.robot.set_velocities(np.zeros((1, 6)))

    def hold_pose(self, kp=400.0, kd=40.0):
        """Giu cac khop o tu the mac dinh (robot dung thang, khong xoai)."""
        n = self.robot.num_dof
        self.robot.set_gains(kps=np.full((1, n), kp), kds=np.full((1, n), kd))
        self.robot.set_joint_position_targets(np.zeros((1, n)))
