"""Phat lai ban ghi rt/lowstate qua leg odometry + WBC, KHONG can robot.

Day la thu thay the duoc mo phong o dung cho mo phong yeu nhat: mo phong cho
dq hoan hao, quaternion hoan hao, tau_est hoan hao. Robot that thi khong.
Nhieu 6 mm/s tren van toc than - thu lam rung mo-men 1.3 Nm - khong ton tai
trong MuJoCo. Phat lai thi no con nguyen.

Doi lai, phat lai KHONG co phan hoi: robot trong ban ghi lam gi la lam roi,
mo-men ta tinh ra khong lam no dong y. Nen dung phat lai de tra loi:
    - QP co giai duoc o moi tu the that khong, het bao lau?
    - Doi he so thi mo-men doi bao nhieu, rung bao nhieu?
    - Uoc luong van toc/luc tiep xuc co hop ly khong?
Va KHONG dung de tra loi "co giu duoc thang bang khong" - cho nay van phai
MuJoCo (co phan hoi) hoac robot that.

    .venv-real/bin/python G1/src/g1_wbc/g1_wbc/replay.py BANGHI.npz
    ... --sweep kd_com=0,15,30        # quet mot he so, in bang so sanh
    ... --csv ra.csv                  # xuat tung khung
"""
import argparse
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
for p in (os.path.join(_REPO, "G1", "src", "g1_wbc"),
          os.path.join(_REPO, "G1", "src", "g1_leg_odometry")):
    if p not in sys.path:
        sys.path.insert(0, p)

from g1_leg_odometry.leg_odometry import LegOdometry          # noqa: E402
from g1_wbc.recording import Recording, N_MOTOR               # noqa: E402
from g1_wbc.wbc import BalanceWBC                             # noqa: E402

URDF = os.path.join(_REPO, "G1/src/g1_description/urdf/g1_29dof.urdf")
PAYLOAD = os.path.join(_REPO, "G1/src/g1_leg_odometry/config/payload.yaml")

# Mac dinh = dung config/wbc.yaml dang chay tren robot.
DEFAULTS = dict(kp_com=60.0, kd_com=15.0, kp_ori=250.0, kd_ang=40.0,
                kp_q=100.0, kd_q=20.0, vel_filter_hz=10.0, dq_filter_hz=10.0,
                rate_divider=2, settle_secs=1.0, w_tau=1.0, q_ref_tau=0.0)


def run(rec, cfg, urdf=URDF, payload=PAYLOAD, collect=False, progress=None):
    """Chay lai toan bo ban ghi. Tra ve dict thong ke (va cac khung neu collect)."""
    odom = LegOdometry(urdf)
    wbc = BalanceWBC(urdf, payload_yaml=payload,
                     kp_com=cfg["kp_com"], kd_com=cfg["kd_com"],
                     kp_ori=cfg["kp_ori"], kd_ang=cfg["kd_ang"],
                     kp_q=cfg["kp_q"], kd_q=cfg["kd_q"],
                     w_tau=cfg.get("w_tau", 0.0))
    div = int(cfg["rate_divider"])
    v_fc, dq_fc = cfg["vel_filter_hz"], cfg["dq_filter_hz"]
    dq_filt = np.zeros(N_MOTOR)
    v_filt = np.zeros(3)
    v_init = False
    t_prev = None

    ref_done = False
    t_ref = None
    t0 = float(rec.t[0])
    solve_ms, taus, tau_ests, com_errs, fails, frames = [], [], [], [], {}, []
    v_hist = []

    for i in range(len(rec)):
        if i % div:
            continue
        fr = rec.frame(i)
        t = fr["t"]
        if dq_fc > 0:
            dt = (t - t_prev) if t_prev is not None else 0.002
            t_prev = t
            a = dt / (1.0 / (2 * np.pi * dq_fc) + dt)
            dq_filt += a * (fr["dq"] - dq_filt)
            dq_ctrl = dq_filt.copy()
        else:
            dq_ctrl = fr["dq"]

        try:
            out = odom.update(fr["q"], fr["dq"], fr["gyro"], fr["tau_est"], fr["quat"])
        except Exception as e:
            fails[f"odom: {str(e)[:50]}"] = fails.get(f"odom: {str(e)[:50]}", 0) + 1
            continue

        if not ref_done:
            if t - t0 < cfg["settle_secs"]:
                continue
            wbc.capture_reference(fr["q"], fr["quat"])
            ref_done = True
            continue

        if cfg.get("q_ref_tau", 0.0) > 0.0:
            wbc.update_reference(fr["q"], (t - t_ref) if t_ref is not None else 0.002,
                                 cfg["q_ref_tau"])
        t_ref = t

        v_body = out["v_body"]
        v_hist.append(v_body)
        if v_fc > 0:
            if not v_init:
                v_filt[:] = v_body
                v_init = True
            # 0.002 ghi cung - GIONG HET wbc_shadow_node de phat lai trung khop.
            dtv = 0.002
            av = dtv / (1.0 / (2 * np.pi * v_fc) + dtv)
            v_filt += av * (v_body - v_filt)
            v_ctrl = v_filt.copy()
        else:
            v_ctrl = v_body
        contact = (out["contact"]["left"], out["contact"]["right"])

        try:
            tic = time.perf_counter()
            tau, info = wbc.solve(fr["q"], dq_ctrl, fr["quat"], fr["gyro"], v_ctrl, contact)
            solve_ms.append((time.perf_counter() - tic) * 1e3)
        except Exception as e:
            k = str(e)[:50]
            fails[k] = fails.get(k, 0) + 1
            continue

        taus.append(tau)
        tau_ests.append(fr["tau_est"])
        com_errs.append(info["com_err"])
        if collect:
            # Luu ca dau vao/ra cua LEG ODOMETRY: khong co chung thi ban C++ cua
            # leg odometry khong ai kiem, ma v_body sai thi WBC sai theo im lang.
            # v_per_foot va fz la ham THUAN cua mot khung (khong co trang thai),
            # nen doi chieu duoc tung khung; con contact thi co tre nguong phu
            # thuoc lich su, khong doi chieu kieu nay duoc.
            frames.append(dict(i=i, t=t, q=fr["q"], dq=dq_ctrl, quat=fr["quat"],
                               gyro=fr["gyro"], v_body=v_ctrl, contact=contact,
                               tau=tau, info=info, tau_est=fr["tau_est"],
                               dq_raw=fr["dq"],
                               v_foot=np.concatenate([out["v_per_foot"]["left"],
                                                      out["v_per_foot"]["right"]]),
                               fz=np.array([out["fz"]["left"], out["fz"]["right"]])))
        if progress and len(taus) % progress == 0:
            print(f"  ... {t - t0:5.1f}s", flush=True)

    tau = np.array(taus) if taus else np.zeros((0, N_MOTOR))
    te = np.array(tau_ests) if tau_ests else np.zeros((0, N_MOTOR))
    ms = np.array(solve_ms) if solve_ms else np.zeros(0)
    st = dict(
        n_ok=len(tau), n_fail=sum(fails.values()), fails=fails,
        solve_mean=float(ms.mean()) if ms.size else float("nan"),
        solve_p99=float(np.percentile(ms, 99)) if ms.size else float("nan"),
        solve_max=float(ms.max()) if ms.size else float("nan"),
        # "rung" = do lech chuan theo thoi gian cua tung khop chan, lay trung binh.
        # Day la con so da dung de do chatter tren robot that (goc 1.289 Nm).
        chatter=float(tau[:, :12].std(axis=0).mean()) if len(tau) > 50 else float("nan"),
        chatter_unitree=float(te[:, :12].std(axis=0).mean()) if len(te) > 50 else float("nan"),
        tau_max=float(np.abs(tau).max()) if len(tau) else float("nan"),
        tau_est_max=float(np.abs(te).max()) if len(te) else float("nan"),
        com_err_max=float(np.abs(np.array(com_errs)).max() * 1e3) if com_errs else float("nan"),
        sign_match=float(100 * np.mean(np.sign(tau[:, :12]) == np.sign(te[:, :12])))
        if len(tau) > 50 else float("nan"),
        vel_std=np.array(v_hist).std(axis=0) * 1e3 if len(v_hist) > 50 else np.zeros(3),
        wbc=wbc, frames=frames,
        tau_hist=tau, tau_est_hist=te,
    )
    return st


def _print(st):
    print(f"  chu ky giai duoc {st['n_ok']} | that bai {st['n_fail']}")
    for k, v in st["fails"].items():
        print(f"    [FAIL x{v}] {k}")
    print(f"  thoi gian giai: TB {st['solve_mean']:.3f} ms | p99 {st['solve_p99']:.3f} | "
          f"max {st['solve_max']:.3f}   (ngan sach 500 Hz = 2.0 ms)")
    print(f"  rung mo-men chan: ta {st['chatter']:.3f} Nm | "
          f"Unitree {st['chatter_unitree']:.3f} Nm")
    print(f"  |tau| max: ta {st['tau_max']:.1f} | Unitree {st['tau_est_max']:.1f} Nm | "
          f"cung dau {st['sign_match']:.0f}%")
    print(f"  CoM lech max {st['com_err_max']:.1f} mm | "
          f"nhieu van toc than {np.array2string(st['vel_std'], precision=2)} mm/s")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--urdf", default=URDF)
    ap.add_argument("--payload", default=PAYLOAD)
    ap.add_argument("--no-payload", action="store_true")
    ap.add_argument("--sweep", default="", metavar="TEN=v1,v2,...",
                    help="quet mot he so, vd --sweep kd_com=0,15,30")
    ap.add_argument("--csv", default="")
    for k, v in DEFAULTS.items():
        ap.add_argument(f"--{k.replace('_', '-')}", type=type(v), default=v)
    a = ap.parse_args(argv)

    rec = Recording(a.recording)
    print(rec.summary())
    cfg = {k: getattr(a, k) for k in DEFAULTS}
    pay = None if a.no_payload else a.payload

    if a.sweep:
        name, vals = a.sweep.split("=")
        name = name.strip()
        if name not in DEFAULTS:
            ap.error(f"khong quet duoc '{name}', chon trong: {', '.join(DEFAULTS)}")
        rows = []
        for v in vals.split(","):
            c = dict(cfg)
            c[name] = type(DEFAULTS[name])(v)
            print(f"\n--- {name}={c[name]} ---")
            st = run(rec, c, a.urdf, pay)
            _print(st)
            rows.append((c[name], st))
        print(f"\n{'='*64}\nBANG SO SANH ({name})")
        print(f"{name:>10} | {'rung Nm':>8} | {'|tau|max':>8} | {'CoM mm':>7} | "
              f"{'giai ms':>7} | {'fail':>5}")
        for v, st in rows:
            print(f"{str(v):>10} | {st['chatter']:8.3f} | {st['tau_max']:8.1f} | "
                  f"{st['com_err_max']:7.1f} | {st['solve_mean']:7.3f} | {st['n_fail']:5d}")
        print("Unitree (tham chieu) rung "
              f"{rows[0][1]['chatter_unitree']:.3f} Nm, |tau|max "
              f"{rows[0][1]['tau_est_max']:.1f} Nm")
        return 0

    print(f"\n--- phat lai (payload {'co' if pay else 'khong'}) ---")
    st = run(rec, cfg, a.urdf, pay, collect=bool(a.csv))
    _print(st)
    if a.csv:
        rows = [np.concatenate([[f["t"]], f["tau"], f["info"]["com_err"],
                                [f["info"]["fz_left"], f["info"]["fz_right"]],
                                f["v_body"]]) for f in st["frames"]]
        hdr = "t," + ",".join(f"tau{i}" for i in range(N_MOTOR)) + \
              ",cex,cey,cez,fzl,fzr,vx,vy,vz"
        np.savetxt(a.csv, np.array(rows), delimiter=",", header=hdr, comments="")
        print(f"  CSV: {a.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
