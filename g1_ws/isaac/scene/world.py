"""Dung canh mo phong: san, robot G1, vat can.

Ve sau (TANG 3) se them cau thang / dia hinh cao thap vao day de test
elevation map va footstep planning.
"""
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import Articulation
from isaacsim.storage.native import get_assets_root_path
import omni.replicator.core as rep

G1_USD = "/Isaac/Robots/Unitree/G1/g1.usd"

# Vat can de LiDAR co gi ma quet (x, y)
DEFAULT_OBSTACLES = [
    (3.0, 0.0), (0.0, 3.0), (-3.0, 1.0), (2.0, -2.5), (-2.0, -3.0),
    (4.0, 3.0), (-4.0, -1.5),
]

# === PHONG KIN 14 x 14 m ===
# VI SAO CAN PHONG KIN (thay vi bai dat trong voi vai khoi hop):
#   Canh cu chi co 7 khoi 0.6x0.6 giua khong gian mo. Do la moi truong
#   RAT NGHEO DAC TRUNG cho SLAM:
#     - phan lon tia LiDAR bay ra vo cuc, khong tra ve gi (do duoc: chi
#       19% tia co vat)
#     - khong co mat phang lon nao de rang buoc goc quay
#   Tuong phong thi khac han: moi huong deu co mat phang thang dung, cho
#   scan matching rang buoc CA vi tri LAN goc quay. Day cung dung voi thuc
#   te - robot humanoid lam viec trong nha.
#
# Kich thuoc chon theo duong di cua robot:
#   vx=0.4, omega=0.15 -> di vong tron ban kinh 0.4/0.15 = 2.67 m,
#   tam (0, 2.67), tuc x thuoc [-2.67, 2.67], y thuoc [0, 5.34].
#   Tuong o +-7 m => con it nhat 1.66 m khoang trong. Vat can ben trong
#   deu dat NGOAI vanh duong di de robot khong huc vao.
ROOM_HALF = 7.0          # nua chieu rong phong (m)
WALL_T = 0.2             # be day tuong
WALL_H = 2.5             # chieu cao tuong

# (x, y, z, sx, sy, sz) - toa do la TAM khoi, scale la kich thuoc DAY DU
ROOM_PARTS = [
    # 4 tuong bao
    ( ROOM_HALF + WALL_T / 2, 0.0, WALL_H / 2, WALL_T, 2 * ROOM_HALF + 2 * WALL_T, WALL_H),
    (-ROOM_HALF - WALL_T / 2, 0.0, WALL_H / 2, WALL_T, 2 * ROOM_HALF + 2 * WALL_T, WALL_H),
    (0.0,  ROOM_HALF + WALL_T / 2, WALL_H / 2, 2 * ROOM_HALF + 2 * WALL_T, WALL_T, WALL_H),
    (0.0, -ROOM_HALF - WALL_T / 2, WALL_H / 2, 2 * ROOM_HALF + 2 * WALL_T, WALL_T, WALL_H),
    # vach lung trong phong - tao ra goc va vung bi che, de loop closure
    # co viec that ma lam
    (-3.0, -5.0, WALL_H / 2, WALL_T, 4.0, WALL_H),
    ( 3.5, -4.0, WALL_H / 2, 5.0, WALL_T, WALL_H),
    # cot
    (-5.5, -4.0, WALL_H / 2, 0.4, 0.4, WALL_H),
    ( 5.5, -4.0, WALL_H / 2, 0.4, 0.4, WALL_H),
    (-5.5,  5.0, WALL_H / 2, 0.4, 0.4, WALL_H),
    ( 5.5,  5.0, WALL_H / 2, 0.4, 0.4, WALL_H),
    # do vat thap - kiem tra dai chieu cao cua pointcloud_to_laserscan
    (-4.5,  0.5, 0.50, 1.2, 1.2, 1.0),
    ( 4.5,  1.0, 0.40, 1.0, 2.0, 0.8),
    ( 0.0, -6.0, 0.60, 2.0, 0.8, 1.2),
]


def build_room():
    """Dung tuong + cot + do vat cua phong kin. Tra ve so khoi da tao."""
    for x, y, z, sx, sy, sz in ROOM_PARTS:
        rep.create.cube(position=(x, y, z), scale=(sx, sy, sz))
    return len(ROOM_PARTS)


def build_scene(obstacles=None, robot_prim="/World/G1", add_dome_light=True,
                physics_dt=None, rendering_dt=None, robot_usd=None,
                room=False, gpu_physics=False):
    """Tao World + san + den + robot G1 + vat can.

    Tra ve (world, robot). Chua goi world.reset() - de ben ngoai goi sau
    khi da gan xong cam bien.

    VE DEN VOM (add_dome_light) - dung bo qua:
      add_default_ground_plane() chi tao MOT SphereLight chieu tu mot phia.
      Hau qua: camera RGB co mot nua khung hinh DEN THUI (do sang = 0.0).
      Rat de nham la "camera bi xoay" hoac "camera hong" - minh da mat
      nhieu vong debug vi hieu nham nhu vay.
      Kiem chung: do do sang trung binh 4 vung khung hinh
          chi co SphereLight : trai 47.0  phai  0.0   <- lech han
          them DomeLight     : trai 168   phai 180    <- can bang
      LiDAR khong bi anh huong (no dung ray tracing hinh hoc, khong can
      anh sang) - nen chi camera moi lo van de nay.
    """
    kw = {}
    if physics_dt is not None:   kw["physics_dt"] = physics_dt
    if rendering_dt is not None: kw["rendering_dt"] = rendering_dt
    if gpu_physics:
        # device="cuda" chuyen duong ong vat ly sang GPU. Xem canh bao ve
        # chi phi dong bo GPU<->CPU trong docstring cua run_g1_sim.py.
        kw["device"] = "cuda"
    world = World(stage_units_in_meters=1.0, **kw)
    world.scene.add_default_ground_plane()

    # In ra trang thai THUC TE, khong tin vao co dong lenh.
    # (Bai hoc lap lai: Isaac Sim nuot loi cau hinh, phai doc nguoc lai.)
    from isaacsim.core.simulation_manager import SimulationManager
    if gpu_physics:
        SimulationManager.enable_gpu_dynamics(True)
    try:
        print(f">>> vat ly: device={SimulationManager.get_physics_sim_device()}  "
              f"gpu_dynamics={SimulationManager.is_gpu_dynamics_enabled()}", flush=True)
    except Exception as e:
        print(f">>> vat ly: khong doc duoc trang thai ({e})", flush=True)

    if add_dome_light:
        import omni.usd
        from pxr import UsdLux, Sdf
        stage = omni.usd.get_context().get_stage()
        dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/DomeLight"))
        dome.CreateIntensityAttr(1000.0)

    # robot_usd: duong dan USD tuy chon. Mac dinh dung G1 43 khop cua NVIDIA.
    # Che do RL dung model 12 KHOP nhap tu unitree_rl_gym - xem run_g1_sim.py.
    if robot_usd:
        add_reference_to_stage(usd_path=robot_usd, prim_path=robot_prim)
    else:
        add_reference_to_stage(usd_path=get_assets_root_path() + G1_USD, prim_path=robot_prim)

    if room:
        # phong kin: KHONG dat them cac khoi cu, vi chung nam dung tren
        # duong di vong tron cua robot
        print(f">>> Phong kin {2*ROOM_HALF:.0f}x{2*ROOM_HALF:.0f} m - {build_room()} khoi", flush=True)
    else:
        for px, py in (DEFAULT_OBSTACLES if obstacles is None else obstacles):
            rep.create.cube(position=(px, py, 0.5), scale=(0.6, 0.6, 1.0))

    robot = Articulation(prim_paths_expr=robot_prim, name="g1")
    world.scene.add(robot)
    return world, robot
