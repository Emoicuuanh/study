"""Camera 3D (RGB + do sau) cho G1 trong Isaac Sim.

Camera bo tro cho LiDAR:
  - LiDAR: chinh xac ve hinh hoc, 360 do, nhung KHONG co mau/kết cấu
  - Camera: co mau (nhan dien vat the, bien bao), do sau day dac o vung
    truoc mat - huu ich cho phat hien vat can thap, be mat trong suot

Tren G1 that thuong dung RealSense D435i hoac camera cua Unitree.

BUG DA GAP - camera nam TRONG than robot:
  Dat camera o z=+0.50 so voi pelvis => nam GON trong torso_link (link nay
  chiem z tu +0.03 den +0.53). Camera nhin thay mat TRONG cua luoi 3D
  => anh RGB chi la mot khoi den, point cloud la cuc diem sat mat kinh.
  Trieu chung de nham voi "camera hong" nhung that ra chi la dat sai cho.
  Do bang UsdGeom.BBoxCache de biet kich thuoc that cua tung link:
      pelvis      z[-0.15, +0.00]
      torso_link  z[+0.03, +0.53]   <- vung cam
      toan robot  z[-0.79, +0.53]
  Fix: dat z=+0.62 (tren dinh torso), giong camera gan tren dau.

HAI QUY UOC TRUC KHAC NHAU - GOC RE CUA MOI RAC ROI:

  (a) Prim camera trong USD:  nhin theo -Z cua chinh no, +Y la huong len.
  (b) Point cloud do sau ma ROS2CameraHelper phat ra: theo QUY UOC QUANG
      HOC cua ROS - x sang phai, y xuong duoi, z ve phia truoc.

  Hai quy uoc lech nhau. Neu ep MOT frame lam ca hai viec thi luon hong
  mot ben - da do bang thuc nghiem:

      goc xoay prim   anh RGB        hinh hoc 3D (diem tren mat san)
      (90, 0, -90)    xoay 90 do     DUNG  (26.5%)
      (90, 0,   0)    DUNG           sai   (0%)

  Cach phat hien anh bi xoay: bau troi trong canh nay render MAU DEN (khong
  co den vom), nen chi can do do sang tung vung anh RGB:
      binh thuong: TREN sang (troi), DUOI toi hon (san), TRAI ~ PHAI
      bi xoay:     PHAI = 0.0 (troi don het sang mot ben)

  GIAI PHAP CHUAN CUA ROS - tach lam hai frame:
      Camera          : prim that, dat (90,0,0) de ANH thang
      camera_optical  : frame ao, xoay them (-90,0,-90) so voi Camera,
                        dung cho POINT CLOUD
  Day chinh la cap camera_link / camera_optical_frame ma moi driver
  camera trong ROS deu phat. Khong phai chuyen rieng cua Isaac Sim.
"""
import omni.usd
from pxr import UsdGeom
import omni.replicator.core as rep


# Phep xoay tu frame prim camera sang frame quang hoc cua ROS.
# Quaternion (w, x, y, z), tuong duong RPY (-90, 0, -90) do.
OPTICAL_ROT_WXYZ = (0.5, -0.5, 0.5, -0.5)
OPTICAL_FRAME = "camera_optical"


def create_camera(parent_prim: str, position=(0.08, 0.0, 0.62), rotation=(0.0, 0.0, 180.0),
                  resolution=(640, 480), focal_length=18.0, clipping=(0.1, 20.0)):
    """Tao camera gan vao `parent_prim`, nhin ve phia truoc robot.

    position mac dinh (0.08, 0, 0.62): ngay TREN dinh torso_link (z=0.53)
             -> khong bi luoi 3D cua robot che.
    rotation mac dinh (90, 0, -90): da kiem chung cho HINH HOC 3D dung
             (point cloud dat dung cho trong the gioi).

    Tra ve (render_product_path, camera_prim_path).
    camera_prim_path can cho node phat TF - neu thieu, RViz khong biet dat
    point cloud cua camera o dau.
    """
    cam = rep.create.camera(
        focal_length=focal_length,
        clipping_range=clipping,
        parent=parent_prim,
    )
    with cam:
        rep.modify.pose(position=position, rotation=rotation)

    render_product = rep.create.render_product(cam, resolution)
    rp_path = render_product.path if hasattr(render_product, "path") else str(render_product)

    # rep.create.camera tao prim long trong Xform -> phai tim prim Camera that
    stage = omni.usd.get_context().get_stage()
    cam_prim_path = None
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if prim.IsA(UsdGeom.Camera) and p.startswith(parent_prim):
            cam_prim_path = p
            break

    return rp_path, cam_prim_path
