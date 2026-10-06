"""Kiem chung LegOdometry (Pinocchio/URDF) doi chieu ground truth trong MuJoCo.

Chay WBC dong luc hoc tuan 10 de robot dung + bi day, moi chu ky lay dung
nhung gi rt/lowstate se cho (q, dq, tau, gyro, quaternion), day qua LegOdometry,
roi so v_body voi van toc that cua MuJoCo.

Test nay dong thoi kiem chung URDF g1_29dof.urdf CO KHOP mo hinh MJCF khong:
neu offset link hai ben lech nhau thi sai so se bung ra ngay.

Chay:  .venv-real/bin/python G1/src/g1_leg_odometry/test/test_against_mujoco.py [--noise]
"""
import os
import sys

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "G1", "src", "g1_leg_odometry"))
os.chdir(ROOT)

NOISE = "--noise" in sys.argv
FUSION_ARG = sys.argv[sys.argv.index("--fusion") + 1] if "--fusion" in sys.argv else "force"
sys.argv = [sys.argv[0], "--bend", "0.3", "--fusion", FUSION_ARG]          # goi chung: tranh diem ky di

import mujoco                                       # noqa: E402
import week10_wbc_balance as wbc                    # noqa: E402
from g1_leg_odometry.leg_odometry import LegOdometry  # noqa: E402

model, data = wbc.model, wbc.data
DT = model.opt.timestep
rng = np.random.default_rng(0)

URDF = os.path.join(ROOT, "G1/src/g1_description/urdf/g1_29dof.urdf")
FUSION = sys.argv[sys.argv.index("--fusion") + 1] if "--fusion" in sys.argv else "force"
odom = LegOdometry(URDF, fusion=FUSION)
print(f"URDF: nv={odom.model.nv}  | MJCF: nv={model.nv}")
assert odom.model.nv == model.nv

GYRO_ADR = model.sensor_adr[mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_SENSOR, "imu-pelvis-angular-velocity")]
GYRO_STD = 5e-4 if NOISE else 0.0
GYRO_BIAS = np.array([0.002, -0.0015, 0.001]) if NOISE else np.zeros(3)
ENC_Q_STD = 1e-3 if NOISE else 0.0
ENC_DQ_STD = 2e-2 if NOISE else 0.0

wbc.set_stand_pose(0.3)
SECS, WARMUP = 12.0, 1.0
err, v_true_all, fz_err, dyn = [], [], [], []

for k in range(int(SECS / DT)):
    mujoco.mj_step1(model, data)
    t = data.time
    data.xfrc_applied[:] = 0.0
    pushing = 1.0 < t and (t % 4.0) < 0.15
    if pushing:
        data.xfrc_applied[wbc.torso_id, :3] = [-40.0, 25.0, 0.0]
    try:
        tau = np.clip(wbc.solve_wbc()[0], wbc.tau_lo, wbc.tau_hi)
        wbc.tau_last[:] = tau
    except Exception:
        tau = wbc.tau_last

    # ---------- gia lap rt/lowstate ----------
    q_motor = data.qpos[7:] + rng.normal(0, ENC_Q_STD, 29)
    dq_motor = data.qvel[6:] + rng.normal(0, ENC_DQ_STD, 29)
    gyro = data.sensordata[GYRO_ADR:GYRO_ADR + 3] + GYRO_BIAS + rng.normal(0, GYRO_STD, 3)
    quat_wxyz = data.xquat[wbc.pelvis_id].copy()     # IMU tra ve quaternion
    tau_est = tau.copy()                             # robot that: motor_state.tau_est

    out = odom.update(q_motor, dq_motor, gyro, tau_est, quat_wxyz)

    R = data.xmat[wbc.pelvis_id].reshape(3, 3)
    v_true = R.T @ data.qvel[:3]
    if t >= WARMUP:
        err.append(out["v_body"] - v_true)
        v_true_all.append(v_true)
        fz_true = np.zeros(2)
        frc = np.zeros(6)
        for i in range(data.ncon):
            c = data.contact[i]
            for g in (c.geom1, c.geom2):
                b = model.geom_bodyid[g]
                if b in wbc.foot_bodies:
                    mujoco.mj_contactForce(model, data, i, frc)
                    fz_true[wbc.foot_bodies.index(b)] += frc[0] * abs(c.frame[2])
                    break
        fz_err.append([out["fz"]["left"] - fz_true[0], out["fz"]["right"] - fz_true[1]])
        dyn.append((t % 4.0) < 0.6)

    data.qfrc_applied[6:] = tau
    mujoco.mj_step2(model, data)

err = np.array(err); v_true_all = np.array(v_true_all)
fz_err = np.array(fz_err); dyn = np.array(dyn); stat = ~dyn

rms = np.sqrt((err ** 2).sum(axis=1).mean()) * 1000
amp = np.abs(v_true_all).max() * 1000
print(f"\nnhieu cam bien: {'BAT' if NOISE else 'TAT'} | fusion={FUSION}")
for i, ax in enumerate("xyz"):
    print(f"  v_body {ax}: RMS {np.sqrt((err[:,i]**2).mean())*1000:6.2f} mm/s  "
          f"max {np.abs(err[:,i]).max()*1000:7.2f} mm/s")
print(f"  RMS chuan 3 truc: {rms:.2f} mm/s   (bien do that {amp:.0f} mm/s -> {rms/amp*100:.1f}%)")
print(f"  fz sai so RMS khi tua tinh: trai {np.sqrt((fz_err[stat,0]**2).mean()):.1f} N  "
      f"phai {np.sqrt((fz_err[stat,1]**2).mean()):.1f} N")

LIMIT = 25.0 if NOISE else 8.0
ok = rms < LIMIT
print(f"\n{'PASS' if ok else 'FAIL'}: RMS {rms:.2f} mm/s {'<' if ok else '>='} nguong {LIMIT} mm/s")
sys.exit(0 if ok else 1)
