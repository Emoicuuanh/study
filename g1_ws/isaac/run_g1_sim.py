"""Diem chay chinh - mo phong G1 voi LiDAR 3D + camera 3D, phat ra ROS2.

    python run_g1_sim.py              # co cua so 3D (mac dinh, de quan sat)
    python run_g1_sim.py --headless   # khong cua so (nhanh hon)
    python run_g1_sim.py --duration 60
    python run_g1_sim.py --preset v1  # tai hien trang thai loi cu (xem duoi)

Chay bang Python cua conda env isaaclab (KHONG phai python he thong):
    ~/miniconda3/envs/isaaclab/bin/python run_g1_sim.py

CAC PRESET CAMERA - giu lai de doi chieu khi can:
  v1  Trang thai LOI DAU TIEN: camera dat o z=+0.50 -> NAM TRONG torso_link
      (link nay chiem z tu +0.03 den +0.53). Camera nhin thay mat trong
      luoi 3D cua robot => anh RGB la mot khoi den. Khong co den vom.
  v2  Trang thai LOI THU HAI: camera da ra ngoai than (z=+0.62) nhung goc
      xoay (90,0,-90) lam ANH NGHIENG 90 do. Point cloud 3D thi DUNG vi
      frame TF cung nghieng cung kieu -> hai cai triet tieu nhau.
      Khong co den vom nen nua khung hinh toi den.
  fix Trang thai da sua (mac dinh): camera (0,0,180) cho anh thang, frame
      quang hoc rieng cho point cloud, va co den vom.
"""
import argparse
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true", help="khong mo cua so 3D")
parser.add_argument("--duration", type=float, default=300.0, help="thoi gian chay (giay)")
parser.add_argument("--vx", type=float, default=0.4, help="van toc tien (m/s)")
parser.add_argument("--omega", type=float, default=0.3, help="van toc xoay (rad/s)")
parser.add_argument("--no-camera", action="store_true", help="tat camera (nhe hon)")
parser.add_argument("--preset", choices=["v1", "v2", "fix", "roll90"], default="fix",
                    help="tai hien trang thai camera cu de doi chieu")
parser.add_argument("--gait", choices=["slide", "walk", "rl"], default="slide",
                    help="slide = truot muot; walk = mo hinh dong hoc; rl = DI BO THAT bang policy Unitree")
parser.add_argument("--disturb", type=float, default=1.0,
                    help="he so nhieu dang di: 0 = tat, 1 = bien do that, 2 = gap doi")
parser.add_argument("--step-freq", type=float, default=1.8, help="so buoc moi giay")
parser.add_argument("--teleop", action="store_true",
                    help="nhan lenh tu /cmd_vel (teleop ban phim hoac Nav2) thay vi --vx/--omega")
parser.add_argument("--scene", choices=["open", "room"], default="open",
                    help="open = bai dat trong voi vai khoi; room = PHONG KIN 14x14m (cho SLAM)")
parser.add_argument("--no-imu", action="store_true",
                    help="tat IMU (SLAM 3D can IMU - chi tat khi go loi)")
parser.add_argument("--gpu-physics", action="store_true",
                    help="chay duong ong vat ly tren GPU. CHU Y: voi MOT robot thi "
                         "thuong KHONG nhanh hon, tham chi cham hon - moi lan doc "
                         "get_joint_positions() thanh mot lan dong bo GPU->CPU. "
                         "Ket hop voi --drive position (khong doc trang thai) moi co ly.")
parser.add_argument("--drive", choices=["torque", "position"], default="torque",
                    help="torque = Python tu tinh PD 500Hz (giong ban MuJoCo cua Unitree); "
                         "position = Isaac Sim dong vai DRIVER DONG CO, tu tinh PD "
                         "trong buoc vat ly (giong kien truc robot THAT)")
parser.add_argument("--start-delay", type=float, default=0.0,
                    help="so giay DUNG TAI CHO truoc khi chay lenh van toc. "
                         "Can cho SLAM 3D: FAST-LIO khoi tao huong trong luc va "
                         "bias con quay tu IMU luc dau, neu luc do robot dang di "
                         "va dang xoay thi no ghi luon toc do xoay thanh bias.")
# parse_known_args (khong phai parse_args): cac co dang --/... la cua Kit,
# vd --/plugins/carb.tasking.plugin/threadCount=6. Chung phai duoc GIU LAI
# trong sys.argv de SimulationApp chuyen tiep xuong Kit. parse_args() se
# bao "unrecognized arguments" va thoat.
args, _kit_args = parser.parse_known_args()

# (vi tri camera, goc xoay, quaternion frame quang hoc xyzw, den vom, dung frame quang hoc)
# v1/v2 KHONG dung frame quang hoc - dung thang frame cua prim camera,
# giong het code luc do, de tai hien trung thuc.
PRESETS = {
    "v1":  ((0.10, 0.0, 0.50), (0.0, -10.0, 0.0),  None, False, False),
    "v2":  ((0.08, 0.0, 0.62), (90.0, 0.0, -90.0), None, False, False),
    "fix": ((0.08, 0.0, 0.62), (0.0, 0.0, 180.0),  (0.5, -0.5, 0.5, -0.5), True, True),
    # roll90: giong het "fix" nhung XOAY THEM 90 do quanh truc nhin.
    # Dung de thay RIENG tac dong cua goc nghieng - huong nhin van la phia truoc.
    "roll90": ((0.08, 0.0, 0.62), (90.0, 0.0, 180.0), None, True, False),
}
CAM_POS, CAM_ROT, OPTICAL_Q, DOME, USE_OPTICAL = PRESETS[args.preset]

# SimulationApp PHAI khoi tao truoc moi import cua Isaac Sim
from isaacsim import SimulationApp
app = SimulationApp({"headless": args.headless})

sys.path.insert(0, __file__.rsplit("/", 1)[0])

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

from scene.world import build_scene
from sensors.lidar import create_lidar
from sensors.camera import create_camera
from sensors.imu import create_imu
from bridge.ros2_graph import build_ros2_graph, build_imu_graph, read_cmd_vel
from control.slide import SlideController, disable_gravity
from control.walk import WalkController
from control.rl_walk import RLWalkController, configure_force_drives

# Policy RL duoc train voi vat ly 500Hz -> phai chay dung tan so do.
# Che do slide/walk thi 60Hz la du.
DT = 0.002 if args.gait == "rl" else 1.0 / 60.0
# Che do RL: vat ly 500Hz nhung chi dung hinh moi 10 buoc (xem RENDER_EVERY).
# rendering_dt PHAI bang khoang thoi gian THUC giua hai lan dung hinh, vi
# dong ho cua cam bien RTX chay theo rendering_dt.
#   Da sai truoc day: rendering_dt = 0.002 nhung thuc te dung hinh moi 0.02s
#   -> cam bien tuong moi khung chi troi 0.002s -> can 50 khung cho mot vong
#      quet 0.1s -> 500 buoc vat ly = 1.0s sim moi ra MOT quet.
#   Do duoc: /points chi 0.98 Hz thay vi 10 Hz (cham dung 10 lan).
RENDER_EVERY = 10 if args.gait == "rl" else 1
RENDER_DT = DT * RENDER_EVERY
CHASSIS = "/World/G1/pelvis"
POLICY_PATH = "/home/hungvd/study/unitree_rl_gym/deploy/pre_train/g1/motion.pt"
# Che do RL dung model 12 KHOP - DUNG model policy duoc train.
# Dung model 43 khop cua NVIDIA thi policy troi 0.212 m/s du lenh=0
# (sai lech sim-to-sim), con model 12 khop chi troi 0.026 m/s.
G1_12DOF_USD = "/home/hungvd/study/g1_ws/isaac/assets/g1_12dof.usd"

print(f">>> Dung canh... (preset camera: {args.preset}, den vom: {'co' if DOME else 'KHONG'})", flush=True)
world, robot = build_scene(add_dome_light=DOME, physics_dt=DT, rendering_dt=RENDER_DT,
                          robot_usd=(G1_12DOF_USD if args.gait == "rl" else None),
                          room=(args.scene == "room"),
                          gpu_physics=args.gpu_physics)

print(">>> Gan cam bien...", flush=True)
lidar = create_lidar(f"{CHASSIS}/lidar", "os0", [0.0, 0.0, 0.45])
# IMU dat CUNG cho voi LiDAR -> extrinsic IMU->LiDAR la ma tran don vi
imu_path = None
if not args.no_imu:
    imu_path = f"{CHASSIS}/imu"
    create_imu(imu_path, "imu", [0.0, 0.0, 0.45])
    print("    imu: cung vi tri LiDAR, 500Hz -> /imu", flush=True)
cam_rp = cam_prim = None
if not args.no_camera:
    cam_rp, cam_prim = create_camera(CHASSIS, position=CAM_POS, rotation=CAM_ROT)
    print(f"    camera: vi tri {CAM_POS}, goc xoay {CAM_ROT}", flush=True)

# Trong luc: TAT khi robot truot (tranh nghieng ~3 do, xem control/slide.py),
# nhung PHAI BAT khi di bo that - khong co trong luc thi khong the di.
# Luu y: disable_gravity() phai goi TRUOC world.reset(), goi sau se lam
# hong physics view cua Articulation (set_gains bao loi get_dof_stiffnesses).
if args.gait != "rl":
    print(f">>> Tat trong luc cho {disable_gravity()} rigid body", flush=True)
else:
    print(">>> Trong luc BAT (can thiet de di bo that)", flush=True)
    # PHAI goi TRUOC world.reset(): sau reset thi physics view da tao xong,
    # sua thuoc tinh USD cua drive khong con tac dung.
    if args.drive == "position":
        configure_force_drives()

world.reset()
lidar.initialize()

print(">>> Dung do thi ROS2...", flush=True)
TWIST_NODE = build_ros2_graph(
    lidar_rp_path=lidar.get_render_product_path(),
    cam_rp_path=cam_rp,
    cam_prim_path=cam_prim,
    chassis_prim=CHASSIS,
    use_optical_frame=USE_OPTICAL,
    optical_rot_xyzw=OPTICAL_Q or (0.0, 0.0, 0.0, 1.0),
    cam_offset=CAM_POS,
    imu_prim_path=imu_path,
)
# IMU o do thi RIENG, on-demand, 500Hz - xem docstring build_imu_graph()
if imu_path:
    build_imu_graph(imu_path)

if args.gait == "rl":
    import numpy as _np
    robot.set_world_poses(positions=_np.array([[0.0, 0.0, 0.80]]),
                          orientations=_np.array([[1.0, 0.0, 0.0, 0.0]]))
    robot.set_velocities(_np.zeros((1, 6)))
    # loop_dt = DT vi ta goi ctrl.step() MOI BUOC VAT LY (500Hz), giong
    # ban goc MuJoCo. Policy van suy dien 50Hz (decimation = 10).
    ctrl = RLWalkController(robot, POLICY_PATH, loop_dt=DT, drive_mode=args.drive)
    ctrl.setup_gains()
    # policy phai chay NGAY - khong cho on dinh thu dong (voi tu the goi gap,
    # PD mot minh khong du do trong luong -> robot sup truoc khi policy vao)
    for _ in range(50):
        ctrl.step(cmd=(0.0, 0.0, 0.0))
        world.step(render=False)
    print(">>> Dang di: RL (policy da train cua Unitree - DI BO THAT)", flush=True)
elif args.gait == "walk":
    ctrl = WalkController(robot, base_height=0.78, dt=DT,
                          step_freq=args.step_freq, disturb=args.disturb)
    ctrl.hold_pose()
    print(f">>> Dang di: WALK (mo hinh dong hoc, {args.step_freq} buoc/giay)", flush=True)
else:
    ctrl = SlideController(robot, base_height=0.80, dt=DT)
    ctrl.hold_pose()
    print(">>> Dang di: SLIDE (truot muot)", flush=True)

n_steps = int(args.duration / DT)
print(f">>> Chay {args.duration:.0f}s. Topic ROS2: /points /odom /tf /clock"
      + ("" if args.no_imu else " /imu")
      + ("" if args.no_camera else " /camera/rgb /camera/depth_pcl"), flush=True)

LOOP = DT
report_every = int(10.0 / LOOP)
# RENDER_EVERY dat o dau file (can cho rendering_dt). Nhac lai ly do:
# mo-men phai cap nhat MOI BUOC VAT LY (500Hz) voi q/dq tuoi. Neu chi cap
# nhat 50Hz (mot lan moi world.step(render=True)) thi PD dung so lieu cu 10
# lan lien -> mat on dinh -> robot NGA.
# Nen tach: chay vat ly render=False, roi dung hinh rieng moi 10 buoc.
if args.teleop:
    print(">>> TELEOP BAT: dang cho lenh tu /cmd_vel", flush=True)
    print(">>>   terminal khac chay: ./run_teleop.sh", flush=True)

for i in range(n_steps):
    if args.teleop:
        vx, vy, wz = read_cmd_vel(TWIST_NODE)
    else:
        vx, vy, wz = args.vx, 0.0, args.omega
    # Giai doan DUNG TAI CHO dau tien: de SLAM 3D khoi tao trong luc/bias
    # tren du lieu IMU khong co van toc tien va khong co van toc xoay.
    if i * DT < args.start_delay:
        vx, vy, wz = 0.0, 0.0, 0.0
    if args.gait == "rl":
        ctrl.step(cmd=(vx, vy, wz))
        world.step(render=False)                 # chi vat ly, 500Hz
        if i % RENDER_EVERY == 0:
            world.render()                        # dung hinh 50Hz cho cam bien RTX
    else:
        ctrl.step(vx=vx, vy=vy, omega=wz)
        world.step(render=True)
    if i % report_every == 0:
        p, _ = robot.get_world_poses()
        print(f">>> t={i*LOOP:5.0f}s  robot tai ({p[0][0]:+.2f}, {p[0][1]:+.2f}) cao {p[0][2]:.2f}m"
              f"  lenh=({vx:+.2f},{vy:+.2f},{wz:+.2f})", flush=True)

print(">>> KET THUC", flush=True)
app.close()
