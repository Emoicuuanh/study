"""Tuan 5 - LQR giu con lac nguoc thang dung: mang ghep thieu tu tuan 2.

Trang thai: x = [vi_tri_xe, van_toc_xe, goc_gay, van_toc_goc]
Dieu khien: u = luc day xe (N)

Buoc 1 - Tuyen tinh hoa quanh diem can bang (gay thang dung, moi thu = 0):
  Voi con lac nguoc chuan (M=khoi luong xe, m=khoi luong gay, l=nua chieu
  dai gay, g=trong truong), phuong trinh dong luc hoc tuyen tinh hoa la:

    x_dot = A x + B u

  A, B duoc dan ra tu Modern Robotics / Underactuated Robotics chuong
  ve con lac nguoc - day la CONG THUC CHUAN, khong can tu suy, chi can
  hieu Y NGHIA tung so hang.

Buoc 2 - Giai LQR: cho scipy giai phuong trinh Riccati, ra ma tran K.
  u = -K x   (luc toi uu, can bang giua "bam sat 0" va "do it luc")

Buoc 3 - Ap dung K vao mo phong MuJoCo (phi tuyen tinh, KHONG xap xi)
  de kiem tra ly thuyet tuyen tinh co thuc su giu duoc con lac THAT.
"""
import numpy as np
import scipy.linalg
import mujoco

# ---- Thong so vat ly (khop voi cartpole.xml) ----
M = 1.0      # khoi luong xe (kg)
m = 0.15     # khoi luong gay ~ que(0.1) + cau(0.05) (kg)
l = 0.5      # chieu dai gay tinh tu khop (m) -- xap xi trong tam ~ l/2 nhung
             # dung mo hinh diem-o-cuoi don gian hoa cho de hieu
g = 9.81

# ---- Buoc 1: A, B cua he tuyen tinh hoa (cong thuc chuan cart-pole) ----
# state = [x, xdot, theta, thetadot], theta=0 la gay thang dung
A = np.array([
    [0, 1, 0,               0],
    [0, 0, -m*g/M,          0],
    [0, 0, 0,               1],
    [0, 0, (M+m)*g/(M*l),   0],
])
B = np.array([
    [0],
    [1/M],
    [0],
    [-1/(M*l)],
])

# ---- Buoc 2: giai LQR ----
# Q: phat loi trang thai (duong cheo = phat rieng tung bien)
# R: phat "ton luc" -- R nho = san sang dung nhieu luc de bam sat muc tieu
Q = np.diag([1.0, 0.1, 10.0, 0.5])   # phat GOC GAY (10.0) manh nhat: uu tien khong ngua
R = np.array([[0.05]])                # cho phep dung luc kha manh

P = scipy.linalg.solve_continuous_are(A, B, Q, R)
K = np.linalg.solve(R, B.T @ P)       # K = R^-1 B^T P  (cong thuc LQR chuan)
print("Gain K =", np.round(K, 3))
print("  (dien giai: luc = -[",
      f"{K[0,0]:.2f}*x + {K[0,1]:.2f}*xdot + {K[0,2]:.2f}*theta + {K[0,3]:.2f}*thetadot ])")

# ---- Buoc 3: kiem tra tren mo phong MuJoCo THAT (phi tuyen) ----
if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path("cartpole.xml")
    data = mujoco.MjData(model)

    # Tao nhieu ban dau: gay lech 0.15 rad (~8.6 do) khoi thang dung
    data.qpos[1] = 0.15
    mujoco.mj_forward(model, data)

    print(f"\nBat dau: goc gay lech {np.degrees(data.qpos[1]):.1f} do")
    max_theta = 0.0
    for step in range(5000):   # 10 giay
        state = np.array([data.qpos[0], data.qvel[0], data.qpos[1], data.qvel[1]])
        u = -K @ state
        data.ctrl[0] = np.clip(u[0], -25, 25)
        mujoco.mj_step(model, data)
        max_theta = max(max_theta, abs(data.qpos[1]))
        if step % 1000 == 0:
            print(f"t={data.time:4.1f}s  x={data.qpos[0]:+.3f}m  theta={np.degrees(data.qpos[1]):+5.1f}do  u={u[0]:+6.2f}N")

    print(f"\nGoc lech lon nhat trong qua trinh: {np.degrees(max_theta):.1f} do")
    print(f"Trang thai cuoi: x={data.qpos[0]:.4f}m, theta={np.degrees(data.qpos[1]):.4f} do")
    ok = abs(data.qpos[1]) < 0.02 and abs(data.qpos[0]) < 0.3
    print("=> LQR GIU DUOC CAN BANG" if ok else "=> CHUA ON, CAN CHINH Q/R")
