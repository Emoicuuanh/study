"""Tuan 5 - Giai thich LQR bang so: K la gi, Q/R lam gi, va gioi han tuyen tinh hoa.

Chay:  ~/miniconda3/envs/g1-real/bin/python week5_lqr_explain.py
Chi can numpy + scipy (khong can MuJoCo).
"""
import numpy as np, scipy.linalg
np.set_printoptions(precision=3, suppress=True)

M, m, l, g = 1.0, 0.15, 0.5, 9.81            # giong week5_lqr_cartpole.py
A = np.array([[0,1,0,0],[0,0,-m*g/M,0],[0,0,0,1],[0,0,(M+m)*g/(M*l),0]])
B = np.array([[0],[1/M],[0],[-1/(M*l)]])

def lqr(Q, R):
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)

# ---- dong luc hoc THAT (phi tuyen, khong xap xi) ----
def f_nonlinear(s, u):
    x, xd, th, thd = s
    st, ct = np.sin(th), np.cos(th)
    den = M + m*st**2
    xdd  = (u + m*l*thd**2*st - m*g*st*ct) / den
    thdd = (-u*ct - m*l*thd**2*st*ct + (M+m)*g*st) / (l*den)
    return np.array([xd, xdd, thd, thdd])

def sim(K, th0, T=8.0, dt=0.001, u_max=None):
    s = np.array([0.0, 0.0, th0, 0.0]); peak_u = 0.0; peak_th = abs(th0); ts = None
    for i in range(int(T/dt)):
        u = float((-K @ s).item())
        if u_max: u = np.clip(u, -u_max, u_max)
        peak_u = max(peak_u, abs(u)); peak_th = max(peak_th, abs(s[2]))
        k1=f_nonlinear(s,u); k2=f_nonlinear(s+dt/2*k1,u)
        k3=f_nonlinear(s+dt/2*k2,u); k4=f_nonlinear(s+dt*k3,u)
        s = s + dt/6*(k1+2*k2+2*k3+k4)
        if abs(s[2]) > np.pi/2: return False, peak_u, peak_th, None   # nga hang
        if ts is None and abs(s[2]) < np.radians(0.5) and abs(s[3]) < 0.05: ts = i*dt
    return True, peak_u, peak_th, ts

Q0 = np.diag([1.0, 0.1, 10.0, 0.5]); R0 = np.array([[0.05]])
K0 = lqr(Q0, R0)

print("="*78); print("1. K LA GI  -  bien 4 con so thanh 1 con so"); print("="*78)
print(f"K = {K0[0]}")
print(f"luc = -({K0[0,0]:.2f}*x  +  {K0[0,1]:.2f}*xdot  +  {K0[0,2]:.2f}*theta  +  {K0[0,3]:.2f}*thetadot)")
print("       ^xe lech      ^xe chay       ^gay nghieng    ^gay dang do")
for st, ten in [([0.1,0,0,0],'xe lech 10cm'), ([0,0,np.radians(5),0],'gay nghieng 5 do'),
                ([0,0,0,0.5],'gay dang do 0.5 rad/s')]:
    print(f"  {ten:24} -> luc = {float((-K0@np.array(st)).item()):+8.2f} N")

print("\n"+"="*78); print("2. CHI TI LE Q/R CO Y NGHIA"); print("="*78)
for scale in [1, 100, 0.01]:
    K = lqr(Q0*scale, R0*scale)
    print(f"  Q*{scale:<6g} R*{scale:<6g} -> K = {K[0]}")
print("  ^ nhan ca hai cung mot so: K KHONG DOI.")

print("\n"+"="*78); print("3. DOI R  -  danh doi giua nhanh va ton luc"); print("="*78)
print(f'{"R":>8} {"Q/R":>8} {"K_theta":>9} {"luc dinh":>10} {"t on dinh":>11}  tinh cach')
print('-'*72)
for R in [0.005, 0.05, 0.5, 5.0, 50.0]:
    K = lqr(Q0, np.array([[R]]))
    ok, pu, pth, ts = sim(K, np.radians(10))
    tag = 'hung hang' if R < 0.05 else ('tiet kiem, cham' if R > 1 else 'can bang')
    print(f'{R:8.3f} {10.0/R:8.1f} {K[0,2]:9.1f} {pu:8.1f} N {str(round(ts,2))+" s" if ts else "   >8 s":>11}  {tag}')

print("\n"+"="*78); print("4. GIOI HAN: sin(theta) ~ theta sai tu luc nao"); print("="*78)
print(f'{"goc":>8} {"sin(theta)":>11} {"theta (rad)":>12} {"sai lech":>10}')
print('-'*46)
for d in [1,5,10,20,30,45,60,80]:
    r = np.radians(d)
    print(f'{d:6d}d {np.sin(r):11.3f} {r:12.3f} {100*abs(r-np.sin(r))/np.sin(r):9.1f}%')

print("\n"+"="*78); print("5. LQR CUU DUOC TU GOC NAO?  (mo phong PHI TUYEN)"); print("="*78)
print(f'{"goc ban dau":>12} {"cuu duoc?":>11} {"luc dinh":>10} {"goc vot len":>13}')
print('-'*54)
for d in [5,10,20,30,40,50,60,70,80]:
    ok, pu, pth, ts = sim(K0, np.radians(d))
    print(f'{d:10d} d {("CUU DUOC" if ok else "NGA"):>11} {pu:8.0f} N {np.degrees(pth):11.0f} d')

lo, hi = 5.0, 90.0
for _ in range(40):
    mid = (lo+hi)/2
    (lo, hi) = (mid, hi) if sim(K0, np.radians(mid))[0] else (lo, mid)
print(f"\n  Nguong (khong gioi han luc): ~{lo:.1f} do")
for umax in [200, 100, 50, 20, 10]:
    lo2, hi2 = 0.5, 90.0
    for _ in range(40):
        mid = (lo2+hi2)/2
        (lo2, hi2) = (mid, hi2) if sim(K0, np.radians(mid), u_max=umax)[0] else (lo2, mid)
    print(f"  Nguong khi dong co chi co {umax:3d} N : ~{lo2:.1f} do")
