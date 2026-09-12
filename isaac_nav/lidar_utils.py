"""Tien ich tao LiDAR RTX - co xu ly BUG double-translation cua Isaac Sim 5.0.

BUG PHAT HIEN DUOC (dang ghi vao ho so phong van):
  LidarRtx(prim_path=..., translation=[0,0,H]) gan translation cho CA:
    /World/lidar        (Xform cha)   translate=(0,0,H)
    /World/lidar/sensor (OmniLidar)   translate=(0,0,H)   <-- TRUNG!
  -> cam bien THUC TE nam o do cao 2H, khong phai H.

  Trieu chung: point cloud bao mat san o z = -2H thay vi -H. Neu khong
  phat hien, moi thu SLAM sau nay se sai do cao gap doi - va rat kho
  truy nguon vi ban do van "trong co ve dung", chi lech ty le.

  Cach phat hien: dat 2 LiDAR o 2 do cao khac nhau (1m va 2m), do z cua
  mat san -> ty le z/H = -2.01 o CA HAI -> khang dinh loi he thong,
  khong phai nhieu ngau nhien.

  Fix: xoa xformOp:translate cua prim con 'sensor'.
"""
import numpy as np
import omni.usd
from pxr import UsdGeom, Gf
from isaacsim.sensors.rtx import LidarRtx


def create_lidar(prim_path: str, name: str, translation, config: str = "OS0_REV7_128ch10hz512res"):
    """Tao LiDAR RTX o dung vi tri yeu cau (da fix bug double-translation)."""
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
    """Doc point cloud. Tra ve None neu frame nay chua co du lieu moi
    (LiDAR 10Hz nhung sim chay 60Hz -> chi ~1/6 so frame co du lieu)."""
    pc = lidar.get_current_frame().get("IsaacCreateRTXLidarScanBuffer", {})
    if not isinstance(pc, dict):
        return None
    d = pc.get("data")
    if d is None:
        return None
    arr = np.asarray(d)
    return arr if (arr.ndim == 2 and arr.shape[0] > 0) else None
