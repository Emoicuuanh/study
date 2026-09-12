"""Tuan 7 - LIPM: CHUOI BUOC DI de bat lai thang bang sau cu day manh.

Khac voi capture point tuan 6 (tinh 1 lan, gia dinh dat chan NGAY duoc):
o day them 1 rang buoc THAT: chan chi buoc duoc toi da MAX_STEP moi lan
(khong the mot buoc nhay toi ngay diem cuu neu bi day qua manh).

Mo hinh LIPM (Linear Inverted Pendulum Model): gia dinh chieu cao khoi
tam CO DINH trong luc buoc (z_com = const) -> phuong trinh dong luc hoc
tro thanh TUYEN TINH, co NGHIEM GIAI TICH dep (khong can mo phong tung
buoc nho nhu MuJoCo):

    x(t)  = (x0-p)*cosh(w0 t) + (v0/w0)*sinh(w0 t) + p
    v(t)  = (x0-p)*w0*sinh(w0 t) + v0*cosh(w0 t)

  (p = vi tri chan tru/pivot, w0 = sqrt(g/z_com) nhu tuan 6)

Thuat toan moi buoc (lap lai capture point, co gioi han buoc):
  1. Cho khoi tam "roi tu do" quanh chan tru trong THOI_GIAN_BUOC (Tstep)
     bang cong thuc giai tich tren
  2. Cuoi buoc, tinh capture point = x(T) + v(T)/w0  (dung cong thuc tuan 6)
  3. Chan moi dat tai: pivot + clip(capture_point - pivot, -MAX_STEP, MAX_STEP)
     -> neu capture point qua xa, CHI buoc duoc toi da MAX_STEP -> chua du
        de dung han -> phai buoc THEM 1 lan nua (lap lai tu buoc 1)
  4. Dung lai khi |v| va |x - pivot| deu nho -> da "bat" duoc thang bang
"""
import numpy as np

G = 9.81
Z_COM = 0.70          # chieu cao khoi tam khi di (thap hon dung yen ~0.79
                       # - giong nguoi/robot hoi khuy goi khi chuan bi buoc)
OMEGA0 = np.sqrt(G / Z_COM)
T_STEP = 0.30          # thoi gian moi buoc (s) - swing duration
MAX_STEP = 0.28        # chieu dai buoc toi da (m) - rang buoc VAT LY cua chan


def propagate(x0, v0, pivot, t):
    """Nghiem giai tich LIPM: vi tri/van toc khoi tam sau thoi gian t,
    voi chan tru dat tai `pivot`."""
    rel = x0 - pivot
    x = rel * np.cosh(OMEGA0 * t) + (v0 / OMEGA0) * np.sinh(OMEGA0 * t) + pivot
    v = rel * OMEGA0 * np.sinh(OMEGA0 * t) + v0 * np.cosh(OMEGA0 * t)
    return x, v


def capture_point(x, v):
    return x + v / OMEGA0


def run_recovery(v_push, max_steps=8):
    """Gia lap: bi day toi van toc v_push tai x=0, chan tru ban dau tai
    x=0. Tra ve danh sach cac buoc (vi tri chan) cho den khi dung han."""
    x, v = 0.0, v_push
    pivot = 0.0
    steps = []
    for i in range(max_steps):
        x_end, v_end = propagate(x, v, pivot, T_STEP)
        cp = capture_point(x_end, v_end)
        desired_step = cp - pivot
        actual_step = np.clip(desired_step, -MAX_STEP, MAX_STEP)
        new_pivot = pivot + actual_step
        clipped = abs(actual_step) < abs(desired_step) - 1e-6

        steps.append(dict(i=i, x_end=x_end, v_end=v_end, cp=cp,
                           pivot_old=pivot, pivot_new=new_pivot, clipped=clipped))

        pivot = new_pivot
        x, v = x_end, v_end

        if abs(v_end) < 0.05 and abs(x_end - new_pivot) < 0.02:
            break
        if abs(v_end) > 20 or abs(x_end) > 10:   # da phat no ro rang -> khong the cuu
            steps[-1]["diverged"] = True
            break
    return steps


if __name__ == "__main__":
    # Nguong toi da co the cuu (voi MAX_STEP=0.28m, T_STEP=0.30s) la ~0.505 m/s.
    # Ba muc day duoc chon de thay ro CA BA vung: de cuu / gan nguong / vuot nguong.
    for v_push in [0.3, 0.48, 0.60]:
        print(f"\n=== Bi day toi van toc {v_push} m/s ===")
        steps = run_recovery(v_push, max_steps=12)
        for s in steps:
            tag = " (CLIP - chua du, con phai buoc tiep!)" if s["clipped"] else " (du - dung han)"
            print(f"  Buoc {s['i']+1}: cuoi buoc x={s['x_end']:+.3f}m v={s['v_end']:+.3f}m/s "
                  f"cp={s['cp']:+.3f}m -> chan moi tai {s['pivot_new']:+.3f}m{tag}")
        if steps and steps[-1].get("diverged"):
            print(f"  => VUOT QUA KHA NANG CUU (capturability region) - khong chuoi buoc "
                  f"nao (voi MAX_STEP={MAX_STEP}m) cuu duoc cu day nay. Robot se nga.")
        else:
            print(f"  => Can {len(steps)} buoc de dung han.")
