"""Kiem chung TOAN BO day chuyen ghi -> phat lai -> test vector, KHONG CAN ROBOT.

Van de: bo ghi chi chay duoc khi co robot that. Neu doi den luc co robot moi
biet no hong thi mat ca buoi. Nen o day minh dung MuJoCo sinh ra mot ban ghi
DUNG DINH DANG ma bo ghi se tao, co them nhieu cam bien cho giong robot that,
roi cho no di het day chuyen. Hom co robot chi con viec bat len 40 giay.

Bon buoc, buoc nao hong bao buoc do:
  1. sinh ban ghi gia lap, doc lai, kiem tra so lieu con nguyen
  2. phat lai (replay) - QP phai giai duoc, thoi gian phai hop ly
  3. sinh test vector, chay lai ban Python tren chinh dau vao do -> phai
     KHOP TUYET DOI (neu lech thi ban than dinh dang hoac ma da khong on dinh)
  4. TIEM LOI CO Y (dao dau pin.skew, dung cai loi minh hay lay lam vi du) ->
     bo doi chieu BAT BUOC phai bao lech. Bo doi chieu khong bat duoc loi thi
     no vo dung, va dieu do phai duoc kiem tra chu khong duoc tin.

    .venv-real/bin/python G1/src/g1_wbc/test/test_record_replay_pipeline.py
"""
import os
import sys
import time

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "G1", "src", "g1_wbc"))
sys.path.insert(0, os.path.join(ROOT, "G1", "src", "g1_leg_odometry"))
os.chdir(ROOT)

SECS = float(sys.argv[sys.argv.index("--secs") + 1]) if "--secs" in sys.argv else 8.0
NOISE = "--no-noise" not in sys.argv
KEEP = "--keep" in sys.argv
sys.argv = [sys.argv[0], "--bend", "0.3"]

import mujoco                                                      # noqa: E402
import week10_wbc_balance as sim                                   # noqa: E402
from g1_wbc.recording import Buffer, N_MOTOR, N_MOTOR_RAW, Recording   # noqa: E402
from g1_wbc.wbc import BalanceWBC                                  # noqa: E402

URDF = os.path.join(ROOT, "G1/src/g1_description/urdf/g1_29dof.urdf")
PAY = os.path.join(ROOT, "G1/src/g1_leg_odometry/config/payload.yaml")
OUT = os.path.join(ROOT, "G1", "data", "recordings", "lowstate_sim_pipeline.npz")
VEC = os.path.join(ROOT, "G1", "data", "test_vectors", "sim_pipeline")
TMP = os.environ.get("TMPDIR", "/tmp")

# Muc nhieu lay theo do luong tren robot that (phien 2026-09): nhieu van toc
# than ~6 mm/s, rung mo-men bo dieu khien hang 0.076 Nm.
NZ = dict(q=1.5e-4, dq=1.2e-2, gyro=2.0e-3, quat=2.0e-4, tau=0.30)

fails = []


def step(name):
    print(f"\n{'='*68}\n{name}\n{'='*68}")


# ------------------------------------------------- 1. sinh ban ghi gia lap
step("1/4  sinh ban ghi gia lap tu MuJoCo (dung dinh dang bo ghi tao ra)")
model, data = sim.model, sim.data
DT = model.opt.timestep
pelvis = sim.pelvis_id
wbc_ctrl = BalanceWBC(URDF, payload_yaml=PAY)
sim.set_stand_pose(0.3)
wbc_ctrl.capture_reference(data.qpos[7:].copy(), data.xquat[pelvis].copy())

rng = np.random.default_rng(0)
buf = Buffer(int(SECS / DT) + 100)
tau_last = np.zeros(N_MOTOR)
n_qp_fail = 0
for k in range(int(SECS / DT)):
    mujoco.mj_step1(model, data)
    t = data.time
    data.xfrc_applied[:] = 0.0
    pushing = 1.0 < t and (t % 3.0) < 0.15
    if pushing:
        data.xfrc_applied[sim.torso_id, :3] = [-45.0, 12.0, 0.0]

    R = data.xmat[pelvis].reshape(3, 3)
    try:
        tau, _ = wbc_ctrl.solve(q_motor=data.qpos[7:].copy(), dq_motor=data.qvel[6:].copy(),
                                quat_wxyz=data.xquat[pelvis].copy(),
                                gyro=data.qvel[3:6].copy(), v_body=R.T @ data.qvel[:3])
        tau = np.clip(tau, wbc_ctrl.tau_lo, wbc_ctrl.tau_hi)
        tau_last[:] = tau
    except Exception:
        n_qp_fail += 1
        tau = tau_last
    data.qfrc_applied[6:] = tau
    mujoco.mj_step2(model, data)

    # --- dong goi y het cai bo ghi se nhan tu rt/lowstate ---
    q = np.zeros(N_MOTOR_RAW); q[:N_MOTOR] = data.qpos[7:]
    dq = np.zeros(N_MOTOR_RAW); dq[:N_MOTOR] = data.qvel[6:]
    te = np.zeros(N_MOTOR_RAW); te[:N_MOTOR] = tau
    quat = data.xquat[pelvis].copy()
    gyro = data.qvel[3:6].copy()
    if NOISE:
        q[:N_MOTOR] += rng.normal(0, NZ["q"], N_MOTOR)
        dq[:N_MOTOR] += rng.normal(0, NZ["dq"], N_MOTOR)
        te[:N_MOTOR] += rng.normal(0, NZ["tau"], N_MOTOR)
        gyro += rng.normal(0, NZ["gyro"], 3)
        quat += rng.normal(0, NZ["quat"], 4)
        quat /= np.linalg.norm(quat)
    buf.append(t=t, tick=int(t * 1000) & 0xFFFFFFFF, mode_pr=0, mode_machine=5, crc=0,
               imu_temp=35, quat=quat, gyro=gyro,
               accel=R.T @ (data.cacc[pelvis][3:] + np.array([0, 0, 9.81]))
               if hasattr(data, "cacc") else np.zeros(3),
               rpy=np.zeros(3), q=q, dq=dq, ddq=np.zeros(N_MOTOR_RAW),
               tau_est=te, motor_mode=np.ones(N_MOTOR_RAW))
    if pushing and k % 500 == 0:
        buf.mark(t, "day")
    if data.qpos[2] < 0.35:
        fails.append(f"robot nga trong mo phong o t={t:.2f}s")
        break

buf.save(OUT, dict(label=f"mo phong MuJoCo{' + nhieu' if NOISE else ''}",
                   source="mujoco", noise=NOISE, push_N=45.0))
rec = Recording(OUT)
print(rec.summary())
print(f"kich thuoc file: {os.path.getsize(OUT)/1e6:.2f} MB | QP fail luc sinh: {n_qp_fail}")
if len(rec) < int(SECS / DT) * 0.9:
    fails.append("ban ghi ngan hon mong doi")
if not np.allclose(rec.frame(10)["q"], buf.buf["q"][10, :N_MOTOR], atol=1e-6):
    fails.append("doc lai khong khop voi luc ghi")

# ------------------------------------------------- 2. phat lai
step("2/4  phat lai qua leg odometry + WBC (day la buoc lam duoc khi khong co robot)")
from g1_wbc.replay import DEFAULTS, _print, run                    # noqa: E402

t0 = time.time()
st = run(rec, dict(DEFAULTS), URDF, PAY)
_print(st)
print(f"  (phat lai het {time.time()-t0:.1f} s thuc te cho {rec.duration:.1f} s du lieu)")
if st["n_ok"] < 100:
    fails.append(f"phat lai chi giai duoc {st['n_ok']} chu ky")
if st["n_fail"] > 0.02 * max(st["n_ok"], 1):
    fails.append(f"qua nhieu QP that bai khi phat lai: {st['n_fail']}")

# ------------------------------------------------- 3. test vector
step("3/4  sinh test vector va chay lai ban Python tren chinh dau vao do")
from g1_wbc import test_vectors as tv                              # noqa: E402


class A:
    recording, out, n = OUT, VEC, 30
    urdf, payload, no_payload = URDF, PAY, False


if tv.make(A) != 0:
    fails.append("khong sinh duoc test vector")
gold = VEC + ".txt"
cand = os.path.join(TMP, "tv_rerun")
tv.rerun(gold, cand, URDF, PAY)
print()
if not tv.compare(gold, cand + ".txt"):
    fails.append("chay lai ban Python KHONG khop voi chinh no - dinh dang hoac ma khong on dinh")

# ------------------------------------------------- 4. tiem loi co y
step("4/4  tiem loi co y - bo doi chieu BAT BUOC phai bat duoc")
import pinocchio as pin                                            # noqa: E402
import g1_wbc.wbc as wbcmod                                        # noqa: E402


class _PinDaoDau:
    """Gia lap dung loi da lay lam vi du: khi dich sang C++, viet nham
    skew(com - pts[i]) thay vi skew(pts[i] - com). Khong crash, khong NaN,
    QP van giai ra mo-men bac 10-15 Nm - chi dau mo-men goc la nguoc."""

    def __getattr__(self, n):
        return (lambda v: -pin.skew(v)) if n == "skew" else getattr(pin, n)


real_pin = wbcmod.pin
wbcmod.pin = _PinDaoDau()
try:
    buggy = os.path.join(TMP, "tv_bug")
    tv.rerun(gold, buggy, URDF, PAY)
finally:
    wbcmod.pin = real_pin
print()
caught = not tv.compare(gold, buggy + ".txt")
print(f"\n-> bo doi chieu {'BAT DUOC' if caught else 'KHONG bat duoc'} loi dao dau")
if not caught:
    fails.append("tiem loi dao dau ma bo doi chieu van bao KHOP - bo doi chieu vo dung")

# loi thu hai: chi sai o ba khop eo, de xem bao cao co khoanh dung vung khong
lines = open(gold).read().splitlines()
with open(os.path.join(TMP, "tv_eo.txt"), "w") as f:
    for ln in lines:
        if ln.startswith("OUT_TAU "):
            v = np.array([float(x) for x in ln.split()[1:]])
            v[12:15] += 8.4
            ln = "OUT_TAU " + " ".join(f"{x:.17g}" for x in v)
        f.write(ln + "\n")
print("\n--- loi thu hai: chi ba khop eo lech 8.4 Nm ---")
ok2 = tv.compare(gold, os.path.join(TMP, "tv_eo.txt"))
if ok2:
    fails.append("loi chi o khop eo ma bo doi chieu khong thay")

if not KEEP:
    for p in (OUT, gold, VEC + ".json"):
        if os.path.exists(p) and "sim_pipeline" in p:
            os.remove(p)
    print(f"\n(da xoa file tam; chay lai voi --keep de giu)")

step("KET QUA")
if fails:
    for f in fails:
        print(f"  FAIL: {f}")
print("FAIL" if fails else "PASS - day chuyen ghi/phat lai/test vector san sang cho robot that")
sys.exit(1 if fails else 0)
