"""Tuan 6 - Tinh CAPTURE POINT THAT tu robot G1, khong con la mo hinh toan.

Cong thuc (da hieu qua vi du cai choi/thi nghiem day nguoi):
    x_cp = x_com + v_com / omega0
    omega0 = sqrt(g / z_com)

Khac voi bai LQR (mo hinh don gian M, m, l tu tay chon), o day ta LAY
TRUC TIEP tu MuJoCo:
  - x_com, z_com   : tam khoi cua CA ROBOT (data.subtree_com[0] -
                     MuJoCo tu tinh san, cong tat ca 30+ than robot
                     theo dung ty le khoi luong)
  - v_com          : uoc luong bang sai phan (com_now - com_prev)/dt
                     (don gian hoa - he thong that dung bo loc Kalman,
                     nhung sai phan la du de "nhin thay" capture point)

Robot dung yen bang PD (nhu tuan 2). Moi 3 giay, mot cu day ngang duoc
ap vao than -> quan sat: mocap marker (dia xanh) tren san TU DONG chay
ra xa theo dung huong bi day, dung boi phep tinh nay.
"""
import time
import numpy as np
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene_capture.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)
q_ref = data.qpos[7:].copy()

torso_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
G = 9.81
DT = model.opt.timestep

com_prev = None
push_schedule = [  # (thoi_diem_bat_dau, luc_x, luc_y, thoi_gian_keo_dai)
    (2.0,  35.0,   0.0, 0.15),   # day theo +x
    (5.0,   0.0,  20.0, 0.15),   # day theo +y
    (8.0, -18.0,  12.0, 0.15),   # day xien
]
# Luu y: PD giu tu the (tuan 2) rat "cung" nen robot phan ung nhanh,
# capture point chi lech vai cm - nhung dung HUONG vat ly. Neu day
# manh hon ~30N robot se nga (dung nhu nguong da do o tuan 2), vi
# controller nay CHUA BUOC CHAN - chi moi TINH capture point de xem.

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()
        t = data.time

        # --- Ap luc day theo lich, giong thi nghiem tuan 2 ---
        data.xfrc_applied[torso_id, :3] = 0
        for t0, fx, fy, dur in push_schedule:
            if t0 <= t < t0 + dur:
                data.xfrc_applied[torso_id, 0] = fx
                data.xfrc_applied[torso_id, 1] = fy

        # --- PD giu tu the dung (nhu tuan 2) ---
        data.ctrl[:] = q_ref

        mujoco.mj_step(model, data)

        # --- Tinh capture point tu trang thai THAT cua robot ---
        com = data.subtree_com[0].copy()          # (x, y, z) tam khoi toan robot
        if com_prev is None:
            com_prev = com.copy()
        com_vel = (com - com_prev) / DT
        com_prev = com.copy()

        omega0 = np.sqrt(G / com[2])
        cp_x = com[0] + com_vel[0] / omega0
        cp_y = com[1] + com_vel[1] / omega0
        data.mocap_pos[0] = [cp_x, cp_y, 0.01]      # ve marker tai capture point

        viewer.sync()

        if int(t*500) % 250 == 0:
            print(f"t={t:5.1f}s  com=({com[0]:+.3f},{com[1]:+.3f},{com[2]:.3f})  "
                  f"v_com=({com_vel[0]:+.2f},{com_vel[1]:+.2f})  cp=({cp_x:+.3f},{cp_y:+.3f})")

        dt = DT - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
