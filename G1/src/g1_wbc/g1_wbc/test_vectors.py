"""Sinh va doi chieu TEST VECTOR cho ban C++ cua BalanceWBC.

Y tuong: chot lai mot so tu the THAT cua robot cung voi dau ra ma ban Python
tinh ra o dung tu the do. Ban C++ doc cung dau vao, ghi dau ra ra file, roi
doi chieu. Khop den 1e-9 thi ban dich dung; lech thi bang so sanh chi thang
vao NHOM KHOP nao lech - tu do lan ra so hang sai, khong phai doan mo.

Vi sao phai la du lieu THAT chu khong phai tu the tu nghi ra: tu the tu nghi
chi phu duoc cac truong hop minh NGHI RA. Ban ghi that chua nhung thu minh
khong nghi ra - giai doan mot chan cham dat khi xoay, fz am phi vat ly luc
IMU truot, tu the chuyen tiep luc buoc. Do moi la cho nhanh re code it duoc
di qua nhat, tuc la cho loi dich am lau nhat.

    # sinh (tu ban ghi robot that)
    python test_vectors.py make BANGHI.npz -o G1/data/test_vectors/v1 -n 40
    # doi chieu (khi da co ban C++)
    python test_vectors.py check G1/data/test_vectors/v1.txt ra_cpp.txt

DINH DANG .txt co y de doc bang C++ chi voi ifstream >> - khong can thu vien
JSON. File .json kem theo la de NGUOI doc, khong dung de doi chieu.
"""
import argparse
import json
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
for p in (os.path.join(_REPO, "G1", "src", "g1_wbc"),
          os.path.join(_REPO, "G1", "src", "g1_leg_odometry")):
    if p not in sys.path:
        sys.path.insert(0, p)

from g1_wbc.recording import N_MOTOR                      # noqa: E402

FORMAT = 4   # 4 = GAINS co them w_tau o cuoi
# 2 = them IN_TAUEST / OUT_VFOOT / OUT_FZ cho leg odometry
# 3 = them IN_DQRAW. Vi sao phai co rieng: WBC nhan dq DA LOC, con leg odometry
#     chay tren dq THO. Phien ban 2 chi ghi mot cai IN_DQ (ban da loc) nen ban
#     C++ bi cho an nham dau vao -> OUT_VFOOT lech toi 0.985 m/s trong khi
#     OUT_FZ khop den 1e-13 (fz khong dung dq). Loi nam o DINH DANG, khong phai
#     o ban dich - nhung neu khong ghi ca hai thi khong the phan biet duoc.
# Ten nhom khop de bao cao lech tro thang vao noi can xem.
GROUPS = [("chan trai", 0, 6), ("chan phai", 6, 12), ("eo", 12, 15),
          ("tay trai", 15, 22), ("tay phai", 22, 29)]
# Nguong 1e-4 Nm. KHONG phai noi ngu*ng cho test chay qua - day la ly do:
#
# Do tren 55 vector THAT (2026-10-03): tu the HAI CHAN lech 1.3e-09, tu the MOT
# CHAN lech toi 1.7e-06. Siet eps_abs cua ProxSuite tu 1e-7 xuong 1e-11 thi lech
# CHUNG LAI o 1.717e-06, khong giam them. Kiem tra ca hai nghiem tren chinh bai
# toan QP: ca hai deu KHA THI (vi pham rang buoc ~1e-12) va ham muc tieu chenh
# 2.4e-03 tren gia tri 6.7e+06 - tuc khac nhau o chu so co nghia thu MUOI.
#
# Nguyen nhan la THANG cua bai toan: w_com=60 nhan voi luc co 400 N binh phuong
# cho ham muc tieu co 1e7, nen sai so tuong doi cua double (1e-16) da la 1e-9
# tuyet doi tren ham muc tieu; o cac huong gan bang phang trong giai doan mot
# chan, no no ra thanh 1e-6 tren bien. Khong the lam tot hon bang double.
#
# Nguong nay van bat duoc moi loi dich THAT: hai loi tiem co y trong
# test_record_replay_pipeline.py lech 8.4 va 54.8 Nm - cao hon nguong 1e5 lan.
# Va 1e-4 Nm thi nho hon do phan giai mo-men cua dong co nhieu bac.
TOL = 1e-4
# Dai luong KHONG di qua QP (leg odometry: chi FK, Jacobian, giai he 6x6) thi
# khong co van de thang/bang phang noi tren - phai khop den muc may. De rong
# 1e-9 cho khac biet thu tu phep cong, van chat hon TOL mot trieu lan.
TOL_LIN = 1e-9
TOLS = {"OUT_TAU": TOL, "OUT_QACC": TOL, "OUT_F": TOL,
        "OUT_VFOOT": TOL_LIN, "OUT_FZ": TOL_LIN}


# --------------------------------------------------------------- ghi
def _w(f, key, arr):
    a = np.atleast_1d(np.asarray(arr, dtype=float)).ravel()
    f.write(key + " " + " ".join(f"{x:.17g}" for x in a) + "\n")


def write_vectors(path_base, wbc, ref, vecs):
    """ref: dict(q_motor, quat) da dung cho capture_reference. vecs: list dict."""
    txt = path_base + ".txt"
    os.makedirs(os.path.dirname(os.path.abspath(txt)) or ".", exist_ok=True)
    with open(txt, "w") as f:
        f.write(f"# g1_wbc test vectors, format {FORMAT}\n")
        f.write("# moi dong: TEN roi cac so cach nhau bang dau cach.\n")
        f.write(f"FORMAT {FORMAT}\n")
        f.write(f"NVEC {len(vecs)}\n")
        f.write(f"NMOTOR {N_MOTOR}\nNV {wbc.nv}\nNC {wbc.nc}\n")
        _w(f, "MASS", wbc.mass)
        _w(f, "GAINS", [wbc.mu, wbc.fz_min, wbc.kd_contact,
                        wbc.kp_com, wbc.kd_com, wbc.kp_ori, wbc.kd_ang,
                        wbc.kp_q, wbc.kd_q,
                        wbc.w_com, wbc.w_ang, wbc.w_post, wbc.w_f, wbc.w_qacc,
                        wbc.w_tau])
        # Dau vao cua capture_reference: de ban C++ tu chay lai buoc chot moc
        # va doi chieu duoc ca buoc do, chu khong chi doi chieu solve().
        _w(f, "REF_IN_Q", ref["q_motor"])
        _w(f, "REF_IN_QUAT", ref["quat"])
        _w(f, "REF_QREF", wbc.q_ref)
        _w(f, "REF_COMDES", wbc.com_des)
        _w(f, "REF_QUATDES", [wbc.quat_des.w, wbc.quat_des.x,
                              wbc.quat_des.y, wbc.quat_des.z])
        _w(f, "REF_ANCHOR", wbc.anchor_ref)
        for k, v in enumerate(vecs):
            f.write(f"VEC {k}\n")
            _w(f, "IN_Q", v["q"])
            _w(f, "IN_DQ", v["dq"])
            _w(f, "IN_QUAT", v["quat"])
            _w(f, "IN_GYRO", v["gyro"])
            _w(f, "IN_VBODY", v["v_body"])
            _w(f, "IN_CONTACT", [1.0 if c else 0.0 for c in v["contact"]])
            if "tau_est" in v:
                _w(f, "IN_TAUEST", v["tau_est"])
                _w(f, "IN_DQRAW", v["dq_raw"])
            _w(f, "OUT_TAU", v["tau"])
            _w(f, "OUT_QACC", v["info"]["qacc"])
            _w(f, "OUT_F", v["info"]["f"])
            if "v_foot" in v:
                _w(f, "OUT_VFOOT", v["v_foot"])
                _w(f, "OUT_FZ", v["fz"])
        f.write("END\n")

    js = path_base + ".json"
    with open(js, "w") as f:
        json.dump(dict(format=FORMAT, n_motor=N_MOTOR, nv=wbc.nv, nc=wbc.nc,
                       mass=wbc.mass,
                       reference=dict(in_q=list(map(float, ref["q_motor"])),
                                      in_quat=list(map(float, ref["quat"])),
                                      com_des=list(map(float, wbc.com_des)),
                                      anchor=list(map(float, wbc.anchor_ref))),
                       vectors=[dict(t=v.get("t"), frame=v.get("i"),
                                     contact=[bool(c) for c in v["contact"]],
                                     tau=list(map(float, v["tau"])))
                                for v in vecs]), f, indent=1)
    return txt, js


# --------------------------------------------------------------- doc
def read_vectors(path):
    """Doc file .txt ve dict. Bo qua dong trong va dong bat dau bang '#'."""
    head, vecs, cur = {}, [], None
    with open(path) as f:
        for ln in f:
            ln = ln.strip()
            if not ln or ln.startswith("#"):
                continue
            parts = ln.split()
            key, vals = parts[0], parts[1:]
            if key == "END":
                break
            if key == "VEC":
                cur = {}
                vecs.append(cur)
                continue
            arr = np.array([float(x) for x in vals])
            tgt = head if cur is None else cur
            tgt[key] = arr[0] if len(arr) == 1 else arr
    return dict(head=head, vecs=vecs)


# --------------------------------------------------------------- doi chieu
def compare(gold_path, cand_path, tol=None, keys=None):
    g, c = read_vectors(gold_path), read_vectors(cand_path)
    print(f"chuan : {gold_path}  ({len(g['vecs'])} vector)")
    print(f"doi chieu: {cand_path}  ({len(c['vecs'])} vector)")
    if len(g["vecs"]) != len(c["vecs"]):
        print(f"\nSAI SO LUONG VECTOR: {len(g['vecs'])} vs {len(c['vecs'])}")
        return False

    tols = dict(TOLS) if tol is None else {k: tol for k in TOLS}
    keys = keys or tuple(TOLS)
    head_tol = tol if tol is not None else TOL

    ok = True
    for k in ("MASS", "REF_COMDES", "REF_ANCHOR", "REF_QREF"):
        if k in c["head"]:
            d = np.abs(np.atleast_1d(g["head"][k]) - np.atleast_1d(c["head"][k])).max()
            flag = "OK" if d <= head_tol else "LECH"
            print(f"  {k:12s} lech {d:.3e}  {flag}")
            ok &= d <= head_tol
        else:
            print(f"  {k:12s} (ban doi chieu khong ghi - bo qua)")

    worst = {}
    for k in keys:
        if k not in c["vecs"][0] or k not in g["vecs"][0]:
            print(f"\n{k}: khong co trong mot trong hai file - bo qua")
            continue
        t = tols[k]
        d = np.array([np.abs(gv[k] - cv[k]) for gv, cv in zip(g["vecs"], c["vecs"])])
        worst[k] = d
        m = d.max()
        print(f"\n{k}: lech lon nhat {m:.3e}  (nguong {t:.0e})  "
              f"{'OK' if m <= t else '*** LECH ***'}")
        ok &= m <= t
        if k == "OUT_TAU" and m > t:
            print("  theo nhom khop (lech lon nhat tren moi vector, moi khop):")
            for nm, a, b in GROUPS:
                gm = d[:, a:b].max()
                print(f"    khop {a:2d}-{b-1:2d} ({nm:9s}): {gm:.3e}  "
                      f"{'OK' if gm <= t else 'LECH'}")
            per = d.max(axis=0)
            bad = np.argsort(per)[::-1][:5]
            print("  5 khop lech nhat:", ", ".join(f"{i}({per[i]:.2e})" for i in bad))
            vbad = np.argsort(d.max(axis=1))[::-1][:3]
            print("  3 vector lech nhat:", ", ".join(f"#{i}({d[i].max():.2e})" for i in vbad))

    print("\n" + ("KHOP - ban dich dung." if ok else
                  "LECH - xem nhom khop o tren de khoanh vung so hang sai."))
    return ok


# --------------------------------------------------------------- chay lai
def rerun(gold_path, out_path, urdf=None, payload=None, wbc=None):
    """Chay BalanceWBC tren DUNG dau vao cua file chuan, ghi ra file doi chieu.

    Day chinh la cong viec ma chuong trinh C++ se phai lam: doc file, chot moc
    tham chieu, giai tung vector, ghi OUT_*. Co ban Python cua no de (a) chung
    minh dinh dang doc/ghi lai duoc, (b) dung lam khuon cho ban C++, (c) tiem
    loi co y vao ma kiem tra xem bo doi chieu co bat duoc khong.
    """
    from g1_wbc.wbc import BalanceWBC
    g = read_vectors(gold_path)
    h = g["head"]
    if wbc is None:
        from g1_wbc.replay import PAYLOAD, URDF
        gn = g["head"].get("GAINS")
        wbc = BalanceWBC(urdf or URDF, payload_yaml=payload or PAYLOAD,
                         w_tau=float(gn[14]) if gn is not None and len(gn) > 14 else 0.0)
    wbc.capture_reference(h["REF_IN_Q"], h["REF_IN_QUAT"])
    out = []
    odom = None
    for v in g["vecs"]:
        tau, info = wbc.solve(v["IN_Q"], v["IN_DQ"], v["IN_QUAT"], v["IN_GYRO"],
                              v["IN_VBODY"], tuple(bool(c) for c in v["IN_CONTACT"]))
        o = dict(q=v["IN_Q"], dq=v["IN_DQ"], quat=v["IN_QUAT"],
                 gyro=v["IN_GYRO"], v_body=v["IN_VBODY"],
                 contact=[bool(c) for c in v["IN_CONTACT"]],
                 tau=tau, info=info)
        if "IN_TAUEST" in v:
            if odom is None:
                from g1_leg_odometry.leg_odometry import LegOdometry
                from g1_wbc.replay import URDF
                odom = LegOdometry(urdf or URDF)
            d = odom.update(v["IN_Q"], v.get("IN_DQRAW", v["IN_DQ"]), v["IN_GYRO"],
                            v["IN_TAUEST"], v["IN_QUAT"])
            o["tau_est"] = v["IN_TAUEST"]
            o["dq_raw"] = v.get("IN_DQRAW", v["IN_DQ"])
            o["v_foot"] = np.concatenate([d["v_per_foot"]["left"], d["v_per_foot"]["right"]])
            o["fz"] = np.array([d["fz"]["left"], d["fz"]["right"]])
        out.append(o)
    base = out_path[:-4] if out_path.endswith(".txt") else out_path
    return write_vectors(base, wbc, dict(q_motor=h["REF_IN_Q"], quat=h["REF_IN_QUAT"]), out)


# --------------------------------------------------------------- chon khung
def pick_frames(feats, n):
    """Chon n khung CACH XA NHAU nhat (farthest point sampling).

    Lay n khung lien tiep thi duoc n ban sao gan giong nhau - vo dung, vi ban
    C++ chi can dung o mot tu the la dung ca chum. Can cac tu the KHAC NHAU.
    """
    x = np.asarray(feats, dtype=float)
    x = (x - x.mean(0)) / (x.std(0) + 1e-9)
    n = min(n, len(x))
    idx = [0]
    d = np.linalg.norm(x - x[0], axis=1)
    for _ in range(n - 1):
        j = int(np.argmax(d))
        idx.append(j)
        d = np.minimum(d, np.linalg.norm(x - x[j], axis=1))
    return sorted(set(idx))


def make(args):
    from g1_wbc.replay import DEFAULTS, PAYLOAD, URDF, run
    from g1_wbc.recording import Recording

    rec = Recording(args.recording)
    print(rec.summary())
    cfg = dict(DEFAULTS)
    print("\nchay lai toan bo ban ghi de lay ung vien...")
    st = run(rec, cfg, args.urdf, None if args.no_payload else args.payload, collect=True)
    frames = st["frames"]
    if not frames:
        print("KHONG co khung nao giai duoc - khong sinh duoc vector.")
        return 1
    print(f"  {len(frames)} khung giai duoc, {st['n_fail']} that bai")

    feats = [np.concatenate([f["q"][:12], f["quat"], f["v_body"],
                             np.array(f["contact"], float) * 3.0]) for f in frames]
    idx = pick_frames(feats, args.n)
    # Giai doan mot chan la truong hop it gap nhung de lo loi nhat -> ep lay them.
    single = [i for i, f in enumerate(frames) if sum(f["contact"]) == 1]
    for j in single[:: max(1, len(single) // 5)][:5]:
        if j not in idx:
            idx.append(j)
    idx = sorted(set(idx))
    chosen = [frames[i] for i in idx]
    ns = sum(1 for f in chosen if sum(f["contact"]) == 1)
    print(f"  chon {len(chosen)} vector (trong do {ns} o giai doan mot chan)")

    wbc = st["wbc"]
    i0 = frames[0]["i"]
    # capture_reference da chay trong run() tai khung dau tien sau settle_secs;
    # tim lai dung khung do de ghi dau vao cua no.
    ref_i = max(0, i0 - int(cfg["rate_divider"]))
    fr = rec.frame(ref_i)
    txt, js = write_vectors(args.out, wbc, dict(q_motor=fr["q"], quat=fr["quat"]), chosen)
    print(f"\nDA GHI:\n  {txt}   <- dung file nay de doi chieu\n  {js}   <- de nguoi doc")
    print("\nTu kiem tra (doi chieu chinh no, phai lech 0):")
    print(f"  python {os.path.basename(__file__)} check {txt} {txt}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    m = sub.add_parser("make", help="sinh test vector tu mot ban ghi")
    m.add_argument("recording")
    m.add_argument("-o", "--out", required=True, help="duong dan goc, khong duoi")
    m.add_argument("-n", type=int, default=40)
    m.add_argument("--urdf", default="")
    m.add_argument("--payload", default="")
    m.add_argument("--no-payload", action="store_true")

    r = sub.add_parser("rerun", help="chay lai ban Python tren dau vao cua file chuan")
    r.add_argument("golden")
    r.add_argument("-o", "--out", required=True)

    c = sub.add_parser("check", help="doi chieu ban doi chieu voi ban chuan")
    c.add_argument("golden")
    c.add_argument("candidate")
    # Mac dinh None = dung nguong RIENG cho tung dai luong (TOLS): dai luong qua
    # QP thi 1e-4, dai luong chi la dai so tuyen tinh thi 1e-9.
    c.add_argument("--tol", type=float, default=None)

    a = ap.parse_args(argv)
    if a.cmd == "make":
        from g1_wbc.replay import PAYLOAD, URDF
        a.urdf = a.urdf or URDF
        a.payload = a.payload or PAYLOAD
        return make(a)
    if a.cmd == "rerun":
        txt, _ = rerun(a.golden, a.out)
        print(f"da ghi {txt}")
        return 0
    return 0 if compare(a.golden, a.candidate, a.tol) else 1


if __name__ == "__main__":
    sys.exit(main())
