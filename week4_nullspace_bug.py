"""Tuan 4 - MO PHONG LOI: windup trong null space (khong can MuJoCo).

Chay:  python3 week4_nullspace_bug.py

Dung dung logic dieu khien cua week4_nullspace.py, nhung thay MuJoCo bang
mot canh tay 7 khop tu viet -> chay headless, in ra bang so sanh.

Hai che do:
  BUG  : dq_null la mot VAN TOC tuy y  -> bi kep tran MAX_STEP nua chu ky
  FIX  : dq_null = NULL_GAIN * (tu_the_phu - q)   (kieu P-controller)
"""
import numpy as np

# ---------------------------------------------------------------- canh tay 7 khop
X, Y, Z = np.eye(3)
AXES    = [Y, X, Z, Y, Z, Y, Z]                     # truc xoay cua tung khop
OFFSETS = [np.array(v, float) for v in [            # doan noi TRUOC moi khop (m)
    (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, -0.30),  # vai(3 khop) -> khuyu
    (0, 0, -0.25), (0, 0, 0), (0, 0, 0)]]            # khuyu -> co tay(3 khop)
TIP = np.array([0, 0, -0.10])                        # co tay -> ban tay

def rot(axis, a):
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a)*K + (1-np.cos(a))*(K @ K)

def fk_full(q):
    """Tra ve (vi tri tay, danh sach vi tri khop, danh sach truc khop) trong he the gioi."""
    R, p = np.eye(3), np.zeros(3)
    ps, zs = [], []
    for i in range(7):
        p = p + R @ OFFSETS[i]
        ps.append(p.copy()); zs.append(R @ AXES[i])
        R = R @ rot(AXES[i], q[i])
    return p + R @ TIP, ps, zs

def jac(q):
    """Jacobian 3x7 cho khop xoay:  cot i = z_i x (p_tay - p_i)."""
    p_end, ps, zs = fk_full(q)
    return np.column_stack([np.cross(zs[i], p_end - ps[i]) for i in range(7)])

# ---------------------------------------------------------------- tham so (giong file that)
LAMBDA, GAIN, MAX_STEP = 0.05, 0.10, 0.003
SECONDARY_AMP  = np.array([0, 0, 0.8, 0.4, 0, 0, 0])
SECONDARY_FREQ = 0.25
NULL_GAIN      = 0.05
DT, T_END      = 0.002, 16.0
LO = np.array([-3.0, -1.6, -2.6, -1.0472, -2.0, -1.6, -1.6])
HI = np.array([ 3.0,  2.3,  2.6,  2.0944,  2.0,  1.6,  1.6])

def run(mode):
    q_base = np.array([0.2, -0.3, 0.0, 0.9, 0.0, 0.0, 0.0])
    q_des  = q_base.copy()
    target = fk_full(q_base)[0] + np.array([0.05, 0.02, 0.03])   # diem co dinh, voi toi duoc
    errs, sat, elbow, satmask = [], 0, [], []
    n = int(T_END/DT)
    for i in range(n):
        t = i*DT
        hand = fk_full(q_des)[0]
        err  = target - hand
        errs.append(np.linalg.norm(err)); elbow.append(q_des[3])

        J = jac(q_des)
        A = J @ J.T + LAMBDA**2*np.eye(3)
        dq_ik = J.T @ np.linalg.solve(A, err)
        Jpinv = J.T @ np.linalg.solve(A, np.eye(3))
        N = np.eye(7) - Jpinv @ J

        wave = SECONDARY_AMP * np.sin(2*np.pi*SECONDARY_FREQ*t)
        if mode == 'BUG':
            dq_null = N @ wave                                   # <-- dung thang lam VAN TOC
        else:
            dq_null = N @ (NULL_GAIN * (q_base + wave - q_des))  # <-- P-controller ve tu the

        dq = GAIN*dq_ik + dq_null
        hit = bool(np.any(np.abs(dq) > MAX_STEP))
        satmask.append(hit); sat += hit
        q_des = np.clip(q_des + np.clip(dq, -MAX_STEP, MAX_STEP), LO, HI)

    e = np.array(errs[int(2.0/DT):])            # bo 2s dau (khoi dong)
    return dict(mean=e.mean()*1000, max=e.max()*1000, sat=100*sat/n,
                swing=np.ptp(elbow), errs=np.array(errs), elbow=np.array(elbow),
                satmask=np.array(satmask))


# ---------------------------------------------------------------- mo xe MOT buoc
def dissect(mode, t=3.0):
    """In ra tung thanh phan cua dq tai mot thoi diem, de thay ai an het ngan sach."""
    q_base = np.array([0.2, -0.3, 0.0, 0.9, 0.0, 0.0, 0.0])
    q_des  = q_base.copy()
    target = fk_full(q_base)[0] + np.array([0.05, 0.02, 0.03])
    for i in range(int(t/DT)):                      # chay toi thoi diem t
        tt = i*DT
        J = jac(q_des); A = J @ J.T + LAMBDA**2*np.eye(3)
        err = target - fk_full(q_des)[0]
        dq_ik = J.T @ np.linalg.solve(A, err)
        N = np.eye(7) - (J.T @ np.linalg.solve(A, np.eye(3))) @ J
        wave = SECONDARY_AMP * np.sin(2*np.pi*SECONDARY_FREQ*tt)
        dq_null = N @ wave if mode == 'BUG' else N @ (NULL_GAIN*(q_base + wave - q_des))
        dq = GAIN*dq_ik + dq_null
        q_des = np.clip(q_des + np.clip(dq, -MAX_STEP, MAX_STEP), LO, HI)

    after = np.clip(dq, -MAX_STEP, MAX_STEP)
    ik_part, null_part = GAIN*dq_ik, dq_null
    over = np.linalg.norm(dq)/MAX_STEP                          # vuot ngan sach may lan
    cos  = float(dq @ after)/(np.linalg.norm(dq)*np.linalg.norm(after) + 1e-12)
    lech = np.degrees(np.arccos(min(max(cos, -1.0), 1.0)))      # clip be cong huong bao nhieu
    print(f"\n  ===== che do {mode}, tai t = {t}s, MAX_STEP = {MAX_STEP} =====")
    np.set_printoptions(precision=4, suppress=True, floatmode='fixed')
    print(f"  GAIN*dq_ik  (nhiem vu CHINH) = {ik_part}   do lon {np.linalg.norm(ik_part):.4f}")
    print(f"  dq_null     (nhiem vu PHU)   = {null_part}   do lon {np.linalg.norm(null_part):.4f}")
    print(f"  dq = chinh + phu             = {dq}   do lon {np.linalg.norm(dq):.4f}")
    print(f"  sau np.clip(+-{MAX_STEP})        = {after}   do lon {np.linalg.norm(after):.4f}")
    print(f"  -> phu to gap CHINH             : {np.linalg.norm(null_part)/max(np.linalg.norm(ik_part),1e-12):6.1f} lan")
    print(f"  -> dq vuot ngan sach MAX_STEP   : {over:6.1f} lan  {'<-- TRAN' if over > 1 else '(con cho)'}")
    print(f"  -> np.clip be cong huong di     : {lech:6.1f} do   {'<-- IK bi pha' if lech > 5 else '(khong dang ke)'}")

print("="*78)
print("MO XE MOT BUOC: ai dang an het ngan sach MAX_STEP?")
print("="*78)
for m in ['BUG', 'FIX']:
    dissect(m)
np.set_printoptions(precision=8, suppress=False, floatmode='maxprec')

# ---------------------------------------------------------------- ve bang ky tu
W, H = 68, 11
def chart(y, title, unit, hi=None):
    hi = hi or max(y.max(), 1e-9)
    col = [y[int(i*len(y)/W):max(int((i+1)*len(y)/W), int(i*len(y)/W)+1)].max() for i in range(W)]
    print(f"\n  {title}   (truc doc 0..{hi:.0f} {unit})")
    for r in range(H, 0, -1):
        lvl = hi*r/H
        line = "".join("#" if c >= lvl else " " for c in col)
        print(f"  {lvl:6.0f} |{line}")
    print(f"  {0:6.0f} +" + "-"*W)
    print("         0s" + " "*(W-10) + f"{T_END:.0f}s")

def strip(mask, title):
    col = [mask[int(i*len(mask)/W):max(int((i+1)*len(mask)/W), int(i*len(mask)/W)+1)].mean() for i in range(W)]
    print(f"\n  {title}")
    print("         |" + "".join("#" if c > 0.5 else ("+" if c > 0 else ".") for c in col) + "|   (# = bi kep tran)")

R = {m: run(m) for m in ['BUG', 'FIX']}

print(f'{"che do":>6} {"sai so tay TB":>14} {"sai so tay MAX":>15} {"buoc bi kep tran":>17} {"khuyu dao dong":>15}')
print('-'*76)
for m in ['BUG', 'FIX']:
    r = R[m]
    print(f'{m:>6} {r["mean"]:11.1f} mm {r["max"]:12.1f} mm {r["sat"]:16.0f}% {r["swing"]:12.2f} rad')

print("\n" + "="*78)
print("SAI SO VI TRI TAY THEO THOI GIAN  (muc tieu: cang thap cang tot)")
print("="*78)
top = R['BUG']['max']
chart(R['BUG']['errs']*1000, "[BUG] van toc phu", "mm", top)
chart(R['FIX']['errs']*1000, "[FIX] tu the phu + gain nho", "mm", top)
print("\n  ^ cung mot thang do. FIX gan nhu dinh sat truc hoanh.")

print("\n" + "="*78)
print("GOC KHUYU TAY  (BUG: quet het tam roi DINH gioi han. FIX: dao dong nhe)")
print("="*78)
for m in ['BUG', 'FIX']:
    e = R[m]['elbow']
    print(f"  [{m}] min={e.min():+.2f}  max={e.max():+.2f}  bien do={np.ptp(e):.2f} rad"
          f"   (gioi han khop: {LO[3]:+.2f} .. {HI[3]:+.2f})")

print("\n" + "="*78)
print("BUOC NAO BI KEP TRAN MAX_STEP")
print("="*78)
for m in ['BUG', 'FIX']:
    strip(R[m]['satmask'], f"[{m}]")

print("\nBUG: nhiem vu phu an het ngan sach MAX_STEP -> IK khong con cho -> tay troi.")
print("FIX: nhiem vu phu tu tat khi dat tu the -> IK giu duoc tay.")
