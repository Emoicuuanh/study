"""IMU cho G1 - PHAN BAT BUOC de SLAM 3D chay duoc tren robot chan.

=== VI SAO ROBOT CHAN CAN IMU MA AGV THI KHONG ===

LiDAR quay het mot vong mat 100 ms. Trong 100 ms do robot van dang di:
    xe AGV      : chuyen dong muot, gan nhu deu -> odometry banh xe du de
                  bu, hoac bo qua cung khong sao
    robot chan  : nhun, lac, va dap khi dat chan. Da do duoc do nghieng
                  pelvis len tos 4.43 do trong luc di bo.

Hau qua: diem DAU vong quet va diem CUOI vong quet duoc do o HAI TU THE
KHAC NHAU. Ghep chung nhu cung mot tu the -> vong quet BI XE LECH. Hien
tuong nay goi la MEO TRONG VONG QUET (intra-scan motion distortion).

Da do duoc anh huong o bai truoc: do tan mat san
    che do truot (than khong lac) : 41.9 mm
    che do di bo                  : 58.5 mm    <- tang 40%
du odometry hoan toan chinh xac trong ca hai truong hop.

Cach chua la GO MEO (de-skew): IMU chay 500 Hz cho biet tu the tai TUNG
THOI DIEM trong vong quet, roi tinh lai vi tri tung diem ve mot tu the
tham chieu chung. Do la chu "I" trong LIO (LiDAR-Inertial Odometry) va la
ly do FAST-LIO ton tai.

=== TAN SO: VI SAO DUNG OnPhysicsStep CHU KHONG OnPlaybackTick ===

Do thi ROS2 hien tai chay tren OnPlaybackTick, tuc moi lan world.render().
O che do rl ta chi render moi 10 buoc vat ly => 50 Hz. IMU 50 Hz LA QUA
CHAM cho FAST-LIO (no can >= 100 Hz, thuc te thuong 200-500 Hz).
Nhanh IMU vi vay duoc noi vao OnPhysicsStep => 500 Hz, dung tan so vat ly.
"""
import numpy as np
from isaacsim.sensors.physics import IMUSensor


def create_imu(prim_path: str, name: str, translation=(0.0, 0.0, 0.45)):
    """Tao IMU tai `prim_path`.

    Dat CUNG VI TRI voi LiDAR (mac dinh cao 0.45 m tren pelvis). Ly do:
    FAST-LIO can biet phep bien doi IMU -> LiDAR (extrinsic). Neu hai cam
    bien trung vi tri thi extrinsic la ma tran don vi, bot mot nguon sai.
    Tren robot that hai cai lech nhau vai cm va phai do/hieu chuan that.
    """
    return IMUSensor(
        prim_path=prim_path,
        name=name,
        translation=np.asarray(translation, dtype=float),
        frequency=500,
    )
