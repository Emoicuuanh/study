"""LiDAR 3D cho G1 trong Isaac Sim.

CHON MODEL - Ouster OS0:
  FOV doc +-45 do (rong nhat trong cac model Isaac Sim co san). Quan trong
  voi humanoid vi can nhin CA mat san ngay truoc chan (de dung elevation
  map cho buoc chan) LAN vat can tren cao. G1 that dung Livox Mid-360
  (doc -7 den +52 do) - OS0 tuong duong, nhin xuong con tot hon.
  Thong so: 360 do ngang, 128 tia, 10Hz, tam 0.5-75m.

BUG DA PHAT HIEN VA SUA - double translation:
  LidarRtx(prim_path=..., translation=[0,0,H]) gan translation cho CA HAI:
      /path/lidar          (Xform cha)    translate = H
      /path/lidar/sensor   (OmniLidar)    translate = H   <-- TRUNG
  => cam bien THUC TE nam o do cao 2H.

  Trieu chung: point cloud bao mat san o z = -2H thay vi -H.
  Nguy hiem vi ban do SLAM van "trong co ve dung", chi sai ty le do cao
  - rat kho truy nguon neu khong kiem tra tu dau.

  Cach phat hien: dat 2 LiDAR o 2 do cao khac nhau (1m va 2m), do z cua
  mat san -> ty le z/H = -2.01 o CA HAI -> loi he thong, khong phai nhieu.

  Fix: xoa xformOp:translate cua prim con 'sensor'.
"""
import numpy as np
import omni.usd
from pxr import UsdGeom, Gf
from isaacsim.sensors.rtx import LidarRtx

DEFAULT_CONFIG = "OS0_REV7_128ch10hz512res"


def create_lidar(prim_path: str, name: str, translation, config: str = DEFAULT_CONFIG):
    """Tao LiDAR RTX o DUNG vi tri yeu cau (da xu ly bug double-translation)."""
    lidar = LidarRtx(
        prim_path=prim_path,
        name=name,
        translation=np.asarray(translation, dtype=float),
        config_file_name=config,
    )

    stage = omni.usd.get_context().get_stage()
    child = stage.GetPrimAtPath(f"{prim_path}/sensor")
    if child:
        for op in UsdGeom.Xformable(child).GetOrderedXformOps():
            if op.GetOpName() == "xformOp:translate":
                op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    return lidar


def read_scan(lidar):
    """Doc point cloud dang numpy (N,3), hoac None neu frame nay chua co
    du lieu moi.

    LiDAR chay 10Hz nhung mo phong chay 60Hz -> chi ~1/6 so frame co du
    lieu. Doc ma khong kiem tra se gap mang rong va crash.
    """
    pc = lidar.get_current_frame().get("IsaacCreateRTXLidarScanBuffer", {})
    if not isinstance(pc, dict):
        return None
    data = pc.get("data")
    if data is None:
        return None
    arr = np.asarray(data)
    return arr if (arr.ndim == 2 and arr.shape[0] > 0) else None
