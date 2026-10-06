"""Phan tich offline CSV do tu robot that.

Tach nhieu cam bien khoi dao dong that, xem pho tan so, va - quan trong nhat -
doi chieu voi uoc luong DOC LAP cua chinh Unitree (rt/odommodestate) xem hai ben
co cung nhin thay mot chuyen dong hay khong.

    .venv-real/bin/python G1/src/g1_leg_odometry/test/analyze_log.py /tmp/legodom_dung.csv
"""
import sys

import numpy as np
from scipy import signal

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/legodom_dung.csv"
d = np.genfromtxt(path, delimiter=",", names=True)
t = d["t"]
v = np.column_stack([d["vx"], d["vy"], d["vz"]])
ref = np.column_stack([d["ref_vx"], d["ref_vy"], d["ref_vz"]])
fz = np.column_stack([d["fz_l"], d["fz_r"]])
ok_ref = ~np.isnan(ref).any(axis=1)
fs = 1.0 / np.median(np.diff(t))

print(f"File: {path}")
print(f"{len(t)} mau | {t[-1]-t[0]:.1f}s | {fs:.0f} Hz | co tham chieu: {ok_ref.sum()} mau")
gaps = np.diff(t)
print(f"Khoang cach: trung vi {np.median(gaps)*1e3:.2f} ms, max {gaps.max()*1e3:.1f} ms, "
      f"mat {gaps[gaps>0.05].sum():.2f}s")

# ---------- 1. tach nhieu / dao dong ----------
print("\n=== 1. NHIEU CAM BIEN vs DAO DONG THAT ===")
good = np.diff(t) < 0.005
noise = np.zeros(3)
for i, ax in enumerate("xyz"):
    dv = np.diff(v[:, i])[good]
    noise[i] = dv.std() / np.sqrt(2)
    raw = v[:, i].std()
    sway = np.sqrt(max(raw ** 2 - noise[i] ** 2, 0))
    print(f"  {ax}: tong {raw*1000:7.2f} = nhieu {noise[i]*1000:6.2f} "
          f"+ dao dong {sway*1000:7.2f} mm/s   (ty le dao dong/nhieu = {sway/noise[i]:5.1f}x)")

# ---------- 2. pho tan so ----------
print("\n=== 2. PHO TAN SO (Welch) ===")
print("  Dao dong that nam o tan thap. Nhieu trang thi pho PHANG.")
for i, ax in enumerate("xyz"):
    f, P = signal.welch(v[:, i] - v[:, i].mean(), fs=fs, nperseg=4096)
    lo = P[(f > 0.2) & (f < 3)].mean()
    hi = P[f > 100].mean()
    fpk = f[1:][np.argmax(P[1:])]
    print(f"  {ax}: dinh o {fpk:5.2f} Hz | mat do 0.2-3Hz = {lo:.2e} | >100Hz = {hi:.2e}"
          f" | ty le = {lo/hi:8.0f}x")

# ---------- 3. doi chieu voi uoc luong cua Unitree ----------
print("\n=== 3. DOI CHIEU VOI rt/odommodestate (uoc luong cua Unitree) ===")
if ok_ref.sum() < 100:
    print("  khong du mau tham chieu")
else:
    a, b = v[ok_ref], ref[ok_ref]
    # loc thong thap 5Hz de so phan TIN HIEU THAT, bo nhieu bang rong
    sos = signal.butter(4, 5.0, fs=fs, output="sos")
    af = signal.sosfiltfilt(sos, a, axis=0)
    bf = signal.sosfiltfilt(sos, b, axis=0)
    for i, ax in enumerate("xyz"):
        r = np.corrcoef(af[:, i], bf[:, i])[0, 1]
        # he so ty le bang hoi quy qua goc
        k = (af[:, i] @ bf[:, i]) / max(bf[:, i] @ bf[:, i], 1e-12)
        print(f"  {ax}: tuong quan (<5Hz) = {r:+.3f} | he so ty le = {k:5.2f} | "
              f"bien do ta {af[:,i].std()*1000:6.1f} vs Unitree {bf[:,i].std()*1000:6.1f} mm/s")
    print("  tuong quan cao = hai uoc luong DOC LAP cung thay mot chuyen dong")

# ---------- 4. luc tiep xuc ----------
print("\n=== 4. LUC PHAP TUYEN UOC LUONG ===")
m = ~np.isnan(fz).any(axis=1)
if m.sum():
    tot = fz[m].sum(axis=1)
    MG = 33.34 * 9.81
    print(f"  trai {fz[m,0].mean():6.1f} +- {fz[m,0].std():5.1f} N | "
          f"phai {fz[m,1].mean():6.1f} +- {fz[m,1].std():5.1f} N")
    print(f"  tong {tot.mean():6.1f} N  vs  mg(URDF 33.34kg) = {MG:.1f} N  "
          f"-> lech {100*(tot.mean()-MG)/MG:+.1f}%")
    print(f"  chenh lech trai-phai: {abs(fz[m,0].mean()-fz[m,1].mean()):.1f} N")

print("\n=== KHUYEN NGHI ===")
rec = float(np.ceil(noise.max() * 1.5 * 1000) / 1000)
print(f"  vel_std        = {rec:.3f}")
print(f"  leg_vel_noise  = [{rec**2:.2e}, {rec**2:.2e}, {rec**2:.2e}]")
