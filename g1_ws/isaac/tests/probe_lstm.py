"""Kiem tra trang thai LSTM cua policy co lien quan den hien tuong TROI khong.

    ~/study/.venv/bin/python isaac/tests/probe_lstm.py

CACH LAM - chay NGOAI TUYEN, khong can Isaac Sim:
  Dua vao policy mot observation "DUNG THANG HOAN HAO, lenh = 0":
      van toc goc than  = 0        (khong lac)
      huong trong luc   = (0,0,-1) (thang dung tuyet doi)
      lenh              = 0        (khong di dau ca)
      goc khop          = goc mac dinh  -> qj = 0
      van toc khop      = 0
      action truoc      = vong lap nguoc (dau ra buoc truoc)
      pha buoc          = chay theo thoi gian, chu ky 0.8s

  Day la trang thai LY TUONG ma robot NEN o do khi lenh = 0. Neu policy
  khoe manh, hanh dong tra ve phai:
      (a) tuan hoan, bien do bo chan
      (b) TRUNG BINH mot chu ky ~ 0, tuc khong co xu huong nghieng ve
          mot phia -> khong co ly do gi de troi

  Neu trung binh KHAC 0 mot cach he thong -> do la thien lech co san cua
  policy, va no giai thich truc tiep hien tuong troi.

BA CAU HOI CAN TRA LOI:
  1. |hidden_state| co bi phinh to theo thoi gian khong (phan ky)?
  2. Hanh dong co hoi tu ve chu trinh gioi han on dinh khong?
  3. Trung binh hanh dong qua mot chu ky buoc co bang 0 khong?

=== GIOI HAN CUA BAI TEST NAY - DOC TRUOC KHI DIEN GIAI ===

CHI CAU HOI 1 LA TRA LOI DUOC. Cau 2 va 3 KHONG.

Ly do: bai test giu co dinh
    o[0:3] = 0            (than khong xoay)
    o[3:6] = (0, 0, -1)   (than thang dung tuyet doi)
trong khi chan dang vung. Do la trang thai VAT LY KHONG THE TON TAI - chan
vung ma than khong hề nghiêng, khong hề xoay.

Policy vi the nhan tin hieu "co the hoan toan on dinh", khong co ly do gi
de tu sua, va troi vao vung vo nghia. Do duoc: bien do hanh dong len toi
14.3 (= 3.6 rad = 205 do cho mot khop) o kich ban B.

So voi quan sat THAT trong Isaac Sim: robot dung vung o 0.777 m, troi
0.026 m/s. Hai ket qua mau thuan nhau -> PHEP DO SAI, khong phai mo phong
sai. (Dung bai hoc da ghi trong INTERVIEW_QUESTIONS.md: khi so do mau thuan
voi quan sat truc tiep, nghi ngo phep do truoc.)

Muon tra loi cau 2-3 cho dung thi phai do TRONG mo phong that, noi than
robot phan ung lai chuyen dong cua chan.
"""
import numpy as np
import torch

POLICY = "/home/hungvd/study/unitree_rl_gym/deploy/pre_train/g1/motion.pt"

# giong het rl_walk.py
JOINTS = ["L_hip_p", "L_hip_r", "L_hip_y", "L_knee", "L_ank_p", "L_ank_r",
          "R_hip_p", "R_hip_r", "R_hip_y", "R_knee", "R_ank_p", "R_ank_r"]
ACTION_SCALE = 0.25
GAIT_PERIOD = 0.8
CONTROL_DT = 1.0 / 50.0          # policy chay 50Hz
N_STEPS = 500                     # 10 giay = 12.5 chu ky buoc


DOF_VEL_SCALE = 0.05


def build_obs(action_prev, t, qj=None, dqj=None):
    """Observation cua trang thai 'than thang dung, lenh = 0'.

    qj/dqj = None  -> KICH BAN A: gia vo khop LUON o goc mac dinh.
                      Khong thuc te: nhu the la giu chat robot, khong cho
                      no phan hoi lai lenh cua chinh policy.
    qj/dqj co gia tri -> KICH BAN B: khop DI THEO lenh (gia thiet PD bam
                      hoan hao). Gan voi vong kin that hon nhieu.
    """
    o = np.zeros(47, dtype=np.float32)
    o[0:3] = 0.0                                    # van toc goc = 0
    o[3:6] = [0.0, 0.0, -1.0]                       # trong luc: thang dung
    o[6:9] = 0.0                                    # lenh = 0
    o[9:21] = 0.0 if qj is None else qj             # goc khop - goc mac dinh
    o[21:33] = 0.0 if dqj is None else dqj          # van toc khop
    o[33:45] = action_prev                          # vong lap nguoc
    phase = (t % GAIT_PERIOD) / GAIT_PERIOD
    o[45:47] = [np.sin(2 * np.pi * phase), np.cos(2 * np.pi * phase)]
    return o


def run(closed_loop):
    """Chay policy N_STEPS buoc. Tra ve (mang hanh dong, mang |h|).

    closed_loop=True: gia thiet PD bam hoan hao, tuc q = target.
        Khi do qj = (target - mac dinh) = action * ACTION_SCALE.
        Day la xap xi hop ly vi PD chay 500Hz voi kp 100-150 bam kha sat.
    """
    m = torch.jit.load(POLICY)     # nap lai -> trang thai LSTM ve 0
    m.eval()
    action = np.zeros(12, dtype=np.float32)
    qj = np.zeros(12, dtype=np.float32)
    hist_a, hist_h = [], []
    for i in range(N_STEPS):
        if closed_loop:
            qj_new = action * ACTION_SCALE          # khop da di den lenh truoc
            dqj = (qj_new - qj) / CONTROL_DT * DOF_VEL_SCALE
            qj = qj_new
            o = build_obs(action, i * CONTROL_DT, qj, dqj)
        else:
            o = build_obs(action, i * CONTROL_DT)
        with torch.no_grad():
            action = m(torch.from_numpy(o).unsqueeze(0)).numpy().squeeze()
        hist_a.append(action.copy())
        hist_h.append(float(np.linalg.norm(m.hidden_state.numpy())))
    return np.array(hist_a), np.array(hist_h)


def main():
    m0 = torch.jit.load(POLICY)
    print(f"trang thai LSTM ban dau: |h| = {float(m0.hidden_state.abs().max()):.6f}")
    print()

    A_open, _ = run(closed_loop=False)
    A, H = run(closed_loop=True)
    print(">>> Ket qua duoi day la KICH BAN B (vong kin: khop di theo lenh).")
    print(">>> Kich ban A (giu chat khop) chi dung de doi chieu o phan 5.")
    print()

    # ---- CAU HOI 1: trang thai LSTM co phinh khong ----
    print("=== 1. Do lon trang thai LSTM theo thoi gian ===")
    for k in [0, 24, 49, 99, 199, 299, 399, 499]:
        print(f"  buoc {k:3d} (t={k*CONTROL_DT:5.2f}s)  |h| = {H[k]:8.4f}")
    growth = H[-50:].mean() / max(H[50:100].mean(), 1e-9)
    print(f"  |h| trung binh 1s dau  : {H[50:100].mean():.4f}")
    print(f"  |h| trung binh 1s cuoi : {H[-50:].mean():.4f}")
    print(f"  ty le cuoi/dau = {growth:.3f}  "
          f"({'ON DINH' if 0.8 < growth < 1.25 else 'CO XU HUONG TRAM TRONG'})")
    print()

    # ---- CAU HOI 2: hanh dong co hoi tu ve chu trinh gioi han khong ----
    print("=== 2. Bien do hanh dong theo thoi gian ===")
    for a, b in [(0, 50), (50, 100), (200, 250), (450, 500)]:
        seg = A[a:b]
        print(f"  buoc {a:3d}-{b:3d}: |action| lon nhat = {np.abs(seg).max():6.3f}, "
              f"trung binh = {np.abs(seg).mean():6.3f}")
    print()

    # ---- CAU HOI 3: trung binh mot chu ky co bang 0 khong ----
    # 0.8s / 0.02s = 40 buoc moi chu ky. Lay 5 chu ky cuoi (da on dinh).
    per = int(round(GAIT_PERIOD / CONTROL_DT))
    tail = A[-5 * per:]
    mean_a = tail.mean(axis=0)
    print(f"=== 3. TRUNG BINH hanh dong qua 5 chu ky cuoi ({per} buoc/chu ky) ===")
    print("   (neu policy khong thien lech, cac so nay phai ~ 0)")
    print(f"   {'khop':10s} {'tb action':>10s} {'-> goc (rad)':>14s} {'bien do':>10s}")
    for j, name in enumerate(JOINTS):
        amp = tail[:, j].max() - tail[:, j].min()
        flag = "  <<<" if abs(mean_a[j] * ACTION_SCALE) > 0.02 else ""
        print(f"   {name:10s} {mean_a[j]:10.4f} {mean_a[j]*ACTION_SCALE:14.4f} "
              f"{amp:10.4f}{flag}")
    print()

    # ---- doi xung trai/phai ----
    print("=== 4. Doi xung trai-phai (chan trai vs chan phai) ===")
    print("   Di thang thi hai chan phai DOI XUNG. Lech nhieu -> robot re/troi.")
    for j in range(6):
        l, r = mean_a[j], mean_a[j + 6]
        # hip_roll, hip_yaw, ankle_roll doi dau khi doi xung; con lai cung dau
        mirror = j in (1, 2, 5)
        expect = -l if mirror else l
        d = abs(r - expect)
        flag = "  <<< LECH" if d > 0.05 else ""
        print(f"   {JOINTS[j][2:]:8s}  trai={l:+.4f}  phai={r:+.4f}  "
              f"(mong doi {expect:+.4f})  lech={d:.4f}{flag}")
    print()

    # ---- 5. doi chieu hai kich ban ----
    print("=== 5. Doi chieu A (giu chat khop) vs B (vong kin) ===")
    print("   Neu B cho thien lech NHO HON HAN A, nghia la phan hoi tu khop")
    print("   da tu sua phan lon thien lech -> policy khong phai thu pham.")
    ta, tb = A_open[-5 * per:], A[-5 * per:]
    print(f"   {'':14s} {'|tb action| lon nhat':>22s} {'|tb action| trung binh':>24s}")
    print(f"   {'A giu chat':14s} {np.abs(ta.mean(axis=0)).max():22.4f} "
          f"{np.abs(ta.mean(axis=0)).mean():24.4f}")
    print(f"   {'B vong kin':14s} {np.abs(tb.mean(axis=0)).max():22.4f} "
          f"{np.abs(tb.mean(axis=0)).mean():24.4f}")


main()
