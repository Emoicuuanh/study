"""Tuan 8 - MPC cho chuoi buoc LIPM: nhin truoc NHIEU buoc, toi uu dong thoi.

So sanh voi tuan 7 (greedy: moi buoc chi nham DUNG capture point, clip
neu vuot MAX_STEP):

  GREEDY:  step_i = clip(capture_point(propagate(...)) - pivot, -MAX, MAX)
           -> chi nhin 1 buoc, khong "biet" buoc sau se the nao

  MPC:     toi uu CA MOT CHUOI [step_1, ..., step_H] cung luc, muc tieu
           "van toc cuoi cung nho nhat", roi CHI THUC HIEN step_1, sau
           do tinh lai tu dau (receding horizon) - dung y tuong ban da
           thay o demo "2 xe dua toi tuong".

QUAN TRONG: ca hai phai dung CHUNG mot mo hinh vat ly (ngay dau tien
minh viet MPC bi sai cho nay - xem INTERVIEW_QUESTIONS.md muc 5.5):
  - Khoi tam NGA DUOI CHAN TRU HIEN TAI suot ca buoc (T_STEP giay)
  - Chi o CUOI buoc, chan moi moi dat xuong va tro thanh tru cho buoc sau
"""
import numpy as np
from scipy.optimize import minimize
from week7_lipm_steps import propagate, capture_point, OMEGA0, T_STEP, MAX_STEP


def greedy_recover(v_push, max_steps=15):
    x, v, pivot = 0.0, v_push, 0.0
    log = []
    for i in range(max_steps):
        xe, ve = propagate(x, v, pivot, T_STEP)
        cp = capture_point(xe, ve)
        step = np.clip(cp - pivot, -MAX_STEP, MAX_STEP)
        pivot += step
        x, v = xe, ve
        log.append((pivot, x, v))
        if abs(v) < 0.05 and abs(x - pivot) < 0.02:
            return log, True
        if abs(v) > 20:
            return log, False
    return log, False


def mpc_recover(v_push, horizon=2, max_steps=15):
    x, v, pivot = 0.0, v_push, 0.0
    log = []
    for i in range(max_steps):
        def cost(steps):
            xx, vv, pp = x, v, pivot
            for s in steps:
                xx, vv = propagate(xx, vv, pp, T_STEP)      # nga duoi tru CU
                pp = pp + np.clip(s, -MAX_STEP, MAX_STEP)   # roi moi doi tru
            return vv**2 + 0.01 * (xx - pp)**2               # phat van toc cuoi + lech vi tri

        res = minimize(cost, np.zeros(horizon), method="Nelder-Mead",
                        options={"xatol": 1e-4, "fatol": 1e-6, "maxiter": 800})
        s1 = np.clip(res.x[0], -MAX_STEP, MAX_STEP)

        xe, ve = propagate(x, v, pivot, T_STEP)   # THUC HIEN dung buoc dau (giong greedy)
        pivot += s1
        x, v = xe, ve
        log.append((pivot, x, v))
        if abs(v) < 0.05 and abs(x - pivot) < 0.02:
            return log, True
        if abs(v) > 20:
            return log, False
    return log, False


if __name__ == "__main__":
    print(f"{'v_push':>8} | {'greedy':>16} | {'MPC (horizon=2)':>16}")
    print("-" * 50)
    for v in [0.30, 0.48, 0.50, 0.503, 0.505, 0.507, 0.52]:
        g_log, g_ok = greedy_recover(v)
        m_log, m_ok = mpc_recover(v, horizon=2)
        g_str = f"{len(g_log)} buoc" if g_ok else "NGA"
        m_str = f"{len(m_log)} buoc" if m_ok else "NGA"
        print(f"{v:8.3f} | {g_str:>16} | {m_str:>16}")

    print("\n--- Chi tiet chuoi buoc tai v=0.503 m/s (gan nguong) ---")
    for name, (log, ok) in [("Greedy", greedy_recover(0.503)), ("MPC   ", mpc_recover(0.503, horizon=2))]:
        positions = [f"{p:.2f}" for p, x, v in log]
        print(f"{name}: {len(log)} buoc -> vi tri chan: {positions}")
