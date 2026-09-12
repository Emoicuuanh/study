"""Tuan 5 - Xem LQR can bang con lac nguoc, tu tay day thu.

Thu trong viewer:
  - Nhap dup chon qua cau tren dinh gay, Ctrl+chuot phai KEO de day
  - Tha ra: xem xe TU DONG chay theo huong dung de "bat" gay lai
  - Day mot cai THAT MANH: > ~35 do se qua kha nang LQR (vi day la
    tuyen tinh hoa QUANH diem can bang, khong con dung khi lech qua xa)
"""
import time
import numpy as np
import mujoco
import mujoco.viewer
from week5_lqr_cartpole import K   # tai lai gain da tinh san

model = mujoco.MjModel.from_xml_path("cartpole.xml")
data = mujoco.MjData(model)
data.qpos[1] = 0.1   # nhieu nho luc bat dau cho vui

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        state = np.array([data.qpos[0], data.qvel[0], data.qpos[1], data.qvel[1]])
        u = -K @ state
        data.ctrl[0] = np.clip(u[0], -25, 25)

        mujoco.mj_step(model, data)
        viewer.sync()

        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
