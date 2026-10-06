"""Giai thich LIPM bang so: hai che do, capture point, vung cuu duoc.

Chay:  ~/miniconda3/envs/g1-real/bin/python week6_lipm_explain.py
"""
import numpy as np
from week7_lipm_steps import OMEGA0, Z_COM, G, T_STEP, MAX_STEP, propagate, capture_point

print(f"z_com = {Z_COM} m   ->   omega0 = sqrt({G}/{Z_COM}) = {OMEGA0:.3f} rad/s")
print(f"hang so thoi gian 1/w0 = {1/OMEGA0:.3f} s     thoi gian NHAN DOI = ln2/w0 = {np.log(2)/OMEGA0:.3f} s\n")

print("="*78); print("1. HAI CHE DO: tach r = x - chan  thanh phan ky + hoi tu"); print("="*78)
print("  r(t) = A*exp(+w0 t)  +  B*exp(-w0 t)")
print("         ^PHAN KY         ^tu tat\n")
print(f'{"tinh huong":>34} {"A (phan ky)":>12} {"B (hoi tu)":>11} | {"r sau 1s":>9} {"sau 2s":>9}')
print('-'*84)
for x0, v0, p, ten in [(0.0, 0.5, 0.00, 'dat chan NGAY duoi nguoi'),
                       (0.0, 0.5, 0.10, 'dat chan truoc 10cm'),
                       (0.0, 0.5, 0.1336, 'dat chan dung CAPTURE POINT'),
                       (0.0, 0.5, 0.20, 'dat chan qua xa 20cm')]:
    r0 = x0 - p
    A = (r0 + v0/OMEGA0)/2; B = (r0 - v0/OMEGA0)/2
    r = lambda t: A*np.exp(OMEGA0*t) + B*np.exp(-OMEGA0*t)
    print(f'{ten:>34} {A:12.4f} {B:11.4f} | {r(1.0):8.3f}m {r(2.0):8.3f}m')
print("\n  A = 0  <=>  chan dat dung capture point  <=>  het thanh phan phan ky.")

print("\n"+"="*78); print("2. VI SAO A = 0 KHI DAT CHAN VAO x_cp"); print("="*78)
x0, v0 = 0.0, 0.5
print(f"  A = (r0 + v0/w0)/2 = ((x0 - p) + v0/w0)/2 = (x_cp - p)/2")
print(f"  voi x0={x0}, v0={v0}: x_cp = {x0} + {v0}/{OMEGA0:.3f} = {capture_point(x0,v0):.4f} m")
print(f"  dat p = x_cp = {capture_point(x0,v0):.4f}  ->  A = 0")
print(f"  ({v0} m/s chia cho {OMEGA0:.3f} 1/s = {v0/OMEGA0:.4f} METRE - w0 doi van toc thanh khoang cach)")

print("\n"+"="*78); print("3. DAT CHAN LECH MOT CHUT THI SAO  (vi sao dung 2 chan KHO)"); print("="*78)
print(f'{"lech (cm)":>10} {"sau 0.5s":>10} {"sau 1s":>10} {"sau 2s":>10} {"sau 3s":>10}')
print('-'*56)
for e_cm in [0.1, 1.0, 5.0]:
    e = e_cm/100
    print(f'{e_cm:9.1f} ' + ''.join(f'{e*np.exp(OMEGA0*t)*100:9.1f}cm' for t in [0.5,1,2,3]))
print(f"\n  Sai {np.log(2)/OMEGA0*1000:.0f} ms khong sua thi sai so NHAN DOI. Khong co che do dao dong de tu quay ve.")

print("\n"+"="*78); print("4. VUNG CUU DUOC (capturability region)"); print("="*78)
v_ideal = MAX_STEP*OMEGA0
print(f"  Neu dat chan TUC THI:  x_cp = v/w0 <= MAX_STEP={MAX_STEP}m  ->  v <= {v_ideal:.3f} m/s")
lo, hi = 0.0, 2.0
for _ in range(50):
    mid = (lo+hi)/2
    xe, ve = propagate(0.0, mid, 0.0, T_STEP)          # khoi tam nga duoi chan CU suot T_STEP
    (lo, hi) = (mid, hi) if capture_point(xe, ve) <= MAX_STEP else (lo, mid)
print(f"  Nhung chan can T_STEP={T_STEP}s moi dat xuong duoc -> nguong thuc: v <= {lo:.3f} m/s")
print(f"  -> do TRE {T_STEP}s lam mat {100*(1-lo/v_ideal):.0f}% vung cuu duoc.")
print(f"\n  (tuan 8 do duoc ~0.507 m/s cho CHUOI nhieu buoc - cao hon {lo:.3f} vi duoc buoc NHIEU lan)")
