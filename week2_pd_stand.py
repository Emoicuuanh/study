"""Tuan 2 - Controller dau tien: giu Unitree G1 dung thang.

BAI HOC QUAN TRONG VE ACTUATOR INTERFACE:
Model G1 nay dung actuator loai <position> - MuJoCo da tich hop san
PD controller ben trong (gain Kp/Kd dinh nghia trong file XML).
=> data.ctrl nhan GOC KHOP MUC TIEU (rad), khong phai torque!

  tau = Kp*(ctrl - q) - Kd*qd   <- MuJoCo tu tinh, ta chi can dua ctrl

(Phien ban dau cua script nay bom torque vao ctrl -> robot hieu nham
thanh goc muc tieu -> nga. Loi kinh dien: hieu sai actuator interface.)

Thu nghiem: chay script, doi robot dung on dinh, roi dung
Ctrl + chuot phai (sau khi nhap dup chon than robot) de day no
=> robot se chong lai va giu thang bang (den mot muc luc nao do).
"""
import time
import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("mujoco_menagerie/unitree_g1/scene.xml")
data = mujoco.MjData(model)

# Tu the muc tieu: keyframe "stand" co san trong model
mujoco.mj_resetDataKeyframe(model, data, 0)
# data.qpos[2] += 1.0  # dat tat ca khop co ban dau = 0 (robot dung thang)
q_ref = data.qpos[7:].copy()   # bo 7 phan tu dau (vi tri + quaternion cua floating base)

print(f"Dieu khien {model.nu} dong co (position servo, PD gain nam trong XML)")
print("Mo viewer... Thu day robot bang Ctrl + chuot phai (nhap dup chon than truoc).")

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # Position actuator: ctrl = goc khop muc tieu
        data.ctrl[:] = q_ref

        mujoco.mj_step(model, data)
        viewer.sync()

        # Giu mo phong chay dung toc do thoi gian thuc
        dt = model.opt.timestep - (time.time() - step_start)
        if dt > 0:
            time.sleep(dt)
