"""Kiem chung BalanceWBC (Pinocchio/URDF) bang cach cho no dieu khien robot
trong MuJoCo. Day la ban VIET LAI cua week10_wbc_balance.py - phai chung minh
no thuc su giu duoc thang bang truoc khi cho chay canh robot that.

Dong thoi do luon KHA NANG CHIU SAI MO HINH: mo phong MJCF nang 33.34 kg,
con mo hinh WBC dung (URDF, va URDF + tai trong) nang 35.1 / 38.6 kg. Tuc la
WBC bi cho mot mo hinh SAI 5% hoac 16% - dung tinh huong se gap tren robot that.

    .venv-real/bin/python G1/src/g1_wbc/test/test_wbc_in_mujoco.py [--payload] [--push 50]
"""
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "G1", "src", "g1_wbc"))
sys.path.insert(0, os.path.join(ROOT, "G1", "src", "g1_leg_odometry"))
os.chdir(ROOT)

USE_PAYLOAD = "--payload" in sys.argv
PUSH = float(sys.argv[sys.argv.index("--push") + 1]) if "--push" in sys.argv else 50.0
SECS = float(sys.argv[sys.argv.index("--secs") + 1]) if "--secs" in sys.argv else 10.0
_keep = list(sys.argv)
sys.argv = [sys.argv[0], "--bend", "0.3"]

import mujoco                                    # noqa: E402
import week10_wbc_balance as sim                 # noqa: E402
from g1_wbc.wbc import BalanceWBC                # noqa: E402

model, data = sim.model, sim.data
DT = model.opt.timestep
URDF = os.path.join(ROOT, "G1/src/g1_description/urdf/g1_29dof.urdf")
PAY = os.path.join(ROOT, "G1/src/g1_leg_odometry/config/payload.yaml")

def _arg(name, default):
    return float(_keep[_keep.index(name) + 1]) if name in _keep else default


KD_COM = _arg("--kdcom", 15.0)
KD_ANG = _arg("--kdang", 40.0)
# Bo loc thong thap NHU TREN ROBOT THAT. Mo phong khong co nhieu nen loc chi
# THEM TRE PHA chu khong loi gi - day chinh la cai gia can do: no cho biet
# viec loc lam giam kha nang chong day bao nhieu.
# Mac dinh phai BAM THEO thu vien, khong phai 0.0: neu khong thi chay bai thu
# ma khong kem co se am tham kiem chung mot cau hinh KHAC voi cau hinh thuc chay.
# Da gap: "PASS o 65 N" hoa ra la cua w_tau=0 trong khi mac dinh da la 1.0.
import inspect as _inspect
_W_DEF = _inspect.signature(BalanceWBC.__init__).parameters["w_tau"].default
W_TAU = _arg("--wtau", _W_DEF)
VEL_FC = _arg("--velfc", 0.0)
DQ_FC = _arg("--dqfc", 0.0)
wbc = BalanceWBC(URDF, payload_yaml=PAY if USE_PAYLOAD else None,
                 kd_com=KD_COM, kd_ang=KD_ANG, w_tau=W_TAU)
print(f"mo phong MJCF: {model.body_mass.sum():.3f} kg | mo hinh WBC: {wbc.mass:.3f} kg"
      f"  -> sai mo hinh {100*(wbc.mass-model.body_mass.sum())/model.body_mass.sum():+.1f}%")

sim.set_stand_pose(0.3)
pelvis = sim.pelvis_id
wbc.capture_reference(data.qpos[7:].copy(), data.xquat[pelvis].copy())

n_fail = 0
v_filt = np.zeros(3)
dq_filt = np.zeros(29)
tau_last = np.zeros(29)
ts = []
max_com = 0.0
n_steps = int(SECS / DT)
import time
for k in range(n_steps):
    mujoco.mj_step1(model, data)
    t = data.time
    data.xfrc_applied[:] = 0.0
    pushing = 1.0 < t and (t % 4.0) < 0.15
    if pushing:
        data.xfrc_applied[sim.torso_id, :3] = [-PUSH, 0.0, 0.0]

    R = data.xmat[pelvis].reshape(3, 3)
    v_meas = R.T @ data.qvel[:3]
    dq_meas = data.qvel[6:].copy()
    if VEL_FC > 0:
        a = DT / (1.0 / (2 * np.pi * VEL_FC) + DT)
        v_filt += a * (v_meas - v_filt)
        v_meas = v_filt.copy()
    if DQ_FC > 0:
        a = DT / (1.0 / (2 * np.pi * DQ_FC) + DT)
        dq_filt += a * (dq_meas - dq_filt)
        dq_meas = dq_filt.copy()
    try:
        t0 = time.perf_counter()
        tau, info = wbc.solve(
            q_motor=data.qpos[7:].copy(), dq_motor=dq_meas,
            quat_wxyz=data.xquat[pelvis].copy(), gyro=data.qvel[3:6].copy(),
            v_body=v_meas)
        ts.append(time.perf_counter() - t0)
        tau = np.clip(tau, wbc.tau_lo, wbc.tau_hi)
        tau_last[:] = tau
        max_com = max(max_com, np.linalg.norm(info["com_err"][:2]))
    except Exception as e:
        n_fail += 1
        tau = tau_last
        if n_fail <= 3:
            print(f"  [QP FAIL] t={t:.2f}s: {e}")

    data.qfrc_applied[6:] = tau
    mujoco.mj_step2(model, data)

    if k % 500 == 0 and n_fail == 0:
        e = info["com_err"]
        print(f"t={t:5.1f}s  CoM err=[{e[0]*1000:6.1f} {e[1]*1000:6.1f} {e[2]*1000:6.1f}]mm  "
              f"fz L={info['fz_left']:6.1f} R={info['fz_right']:6.1f} N  "
              f"|tau|max={np.abs(tau).max():6.1f}Nm  cao={data.qpos[2]:.3f}m"
              f"{'  <-- DAY' if pushing else ''}")
    if data.qpos[2] < 0.35:
        print(f"\n*** NGA o t={t:.2f}s ***")
        break
else:
    print(f"\n*** DUNG VUNG {SECS:.0f}s ***")

ts = np.array(ts) * 1e3
print(f"CoM lech max: {max_com*1000:.1f} mm | QP fail: {n_fail}")
print(f"thoi gian giai: TB {ts.mean():.3f} ms | p99 {np.percentile(ts,99):.3f} ms | max {ts.max():.3f} ms")
ok = n_fail == 0 and data.qpos[2] > 0.35
print(f"\n{'PASS' if ok else 'FAIL'}")
sys.exit(0 if ok else 1)
