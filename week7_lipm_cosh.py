"""Vi sao cosh/sinh chu khong phai cos/sin - va gia dinh bi vi pham ton bao nhieu.

Chay:  ~/miniconda3/envs/g1-real/bin/python week7_lipm_cosh.py
"""
import time, numpy as np
from week7_lipm_steps import OMEGA0, Z_COM, T_STEP, propagate, capture_point
w = OMEGA0

print("="*78); print("1. cosh/sinh CHI LA e^t CHA DOI LOT"); print("="*78)
print("   cosh(t) = (e^t + e^-t)/2      sinh(t) = (e^t - e^-t)/2\n")
print(f'{"t":>5} {"e^(+t)":>9} {"e^(-t)":>9} {"cosh(t)":>9} {"sinh(t)":>9} {"cos(t)":>8} {"sin(t)":>8}')
print('-'*64)
for t in [0,0.5,1,1.5,2,3]:
    print(f'{t:5.1f} {np.exp(t):9.2f} {np.exp(-t):9.3f} {np.cosh(t):9.2f} {np.sinh(t):9.2f} {np.cos(t):8.2f} {np.sin(t):8.2f}')
print("\n  cosh,sinh -> LON MAI (khong bao gio quay lai)")
print("  cos,sin   -> quanh quan trong [-1,+1] MAI MAI")

print("\n"+"="*78); print("2. DAU TRONG PHUONG TRINH QUYET DINH TAT CA"); print("="*78)
x0, v0, T = 0.10, 0.0, 2.5
def hyper(t): return x0*np.cosh(w*t) + (v0/w)*np.sinh(w*t)      # xdd = +w^2 x
def trig(t):  return x0*np.cos(w*t)  + (v0/w)*np.sin(w*t)       # xdd = -w^2 x
W, H = 62, 9
for f, ten, eq in [(hyper,'CON LAC NGUOC  xdd = +w0^2 x','tri rieng THUC +-w0  -> cosh/sinh'),
                   (trig ,'CON LAC THUONG xdd = -w0^2 x','tri rieng AO +-i*w0  -> cos/sin')]:
    ys = np.array([f(t) for t in np.linspace(0,T,W)])
    hi = max(abs(ys).max(), 0.2)
    print(f"\n  {ten}    ({eq})")
    for r in range(H,-H-1,-1):
        lvl = hi*r/H
        row = "".join("#" if (y>=lvl>0) or (y<=lvl<0) or (r==0) else " " for y in ys)
        print(f"  {lvl:+7.2f}m |{row}")
    print(f"          +{'-'*W}   0 -> {T}s")

print("\n"+"="*78); print("3. NGHIEM DONG NHANH HON MO PHONG BAO NHIEU LAN"); print("="*78)
N = 20000
t0=time.perf_counter()
for _ in range(N): propagate(0.0, 0.5, 0.0, T_STEP)
t_analytic = time.perf_counter()-t0
def numeric(x,v,p,T,dt=0.001):
    for _ in range(int(T/dt)):
        a = w*w*(x-p); v += a*dt; x += v*dt
    return x,v
t0=time.perf_counter()
for _ in range(N): numeric(0.0, 0.5, 0.0, T_STEP)
t_numeric = time.perf_counter()-t0
xa,va = propagate(0.0,0.5,0.0,T_STEP); xn,vn = numeric(0.0,0.5,0.0,T_STEP)
print(f"  {N} lan du doan 'khoi tam o dau sau {T_STEP}s':")
print(f"    cong thuc dong (cosh/sinh) : {t_analytic*1000:8.1f} ms   -> x={xa:.5f}")
print(f"    mo phong tung buoc 1ms     : {t_numeric*1000:8.1f} ms   -> x={xn:.5f}")
print(f"    NHANH HON {t_numeric/t_analytic:.0f} lan")
print(f"\n  MPC horizon=2 goi propagate() ~{2*30*15} lan moi vong dieu khien.")
print(f"  O 1kHz: cong thuc dong ton {t_analytic/N*900*1000*1000:.2f} us/vong. Mo phong ton {t_numeric/N*900*1000*1000:.0f} us -> KHONG KIP.")

print("\n"+"="*78); print("4. GIA DINH BI VI PHAM TON BAO NHIEU"); print("="*78)
print(f"  Gia dinh: z_com = {Z_COM} m co dinh. That ra robot NHUN khi di.\n")
print(f'{"z_com that":>11} {"omega0":>8} {"x_cp (v=0.5)":>13} {"lech so voi danh nghia":>23}')
print('-'*60)
base = 0.5/w
for z in [0.65, 0.68, 0.70, 0.72, 0.75]:
    ww = np.sqrt(9.81/z); cp = 0.5/ww
    print(f'{z:10.2f}m {ww:8.3f} {cp*100:11.1f}cm {(cp-base)*1000:+18.1f} mm')
print(f"\n  Nhun +-5cm  ->  dat chan sai +-5mm.")
print(f"  Ma sai so nhan doi moi {np.log(2)/w*1000:.0f}ms -> tang dieu khien BEN DUOI phai lien tuc hap thu.")
