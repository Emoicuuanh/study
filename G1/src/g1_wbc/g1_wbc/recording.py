"""Doc/ghi ban ghi rt/lowstate THO.

Vi sao co file nay: lan truoc minh ghi log vao /tmp (mat sach sau khi khoi dong
lai may) va chi ghi DAU RA da xu ly (tau, v_body, fz) chu khong ghi DAU VAO
(q, dq, quat, gyro). Hai loi do cong lai = khong the phat lai duoc gi ca.

Nguyen tac o day:
  1. Ghi DAU VAO THO, dung nguyen don vi va thu tu cua rt/lowstate. Moi dai
     luong dan xuat (van toc than, luc tiep xuc, mo-men WBC) deu tinh lai duoc
     tu day. Nguoc lai thi khong.
  2. Ghi VAO REPO (G1/data/recordings/), khong phai /tmp.
  3. Mot schema duy nhat, khai bao o day, dung chung cho bo ghi va bo phat lai -
     de hai ben khong the lech nhau.

Ghi ca 35 dong co du G1 29 DOF chi dung 29: ghi tho la ghi tho, cat bot la
mot quyet dinh dien giai, de dan ra khi doc.
"""
import json
import os
import time

import numpy as np

SCHEMA = 2   # 2 = them wireless_remote (tin hieu tay cam)
N_MOTOR_RAW = 35          # do dai mang motor_state trong IDL unitree_hg
N_MOTOR = 29              # so khop that su cua G1 EDU

# ten truong -> (so cot, kieu). so cot 0 = vo huong.
FIELDS = {
    "t":            (0, np.float64),   # giay, dong ho may tinh (monotonic) tu luc bat dau
    "tick":         (0, np.uint32),    # dong ho robot, ms
    "mode_pr":      (0, np.uint8),
    "mode_machine": (0, np.uint8),
    "crc":          (0, np.uint32),
    "imu_temp":     (0, np.int16),
    "quat":         (4, np.float32),   # w, x, y, z
    "gyro":         (3, np.float32),   # rad/s, he than
    "accel":        (3, np.float32),   # m/s^2, he than
    "rpy":          (3, np.float32),
    "q":            (N_MOTOR_RAW, np.float32),
    "dq":           (N_MOTOR_RAW, np.float32),
    "ddq":          (N_MOTOR_RAW, np.float32),
    "tau_est":      (N_MOTOR_RAW, np.float32),
    "motor_mode":   (N_MOTOR_RAW, np.uint8),
    # Tin hieu tay cam. Can de biet NGUOI VAN HANH bam nut luc nao - khong co
    # no thi khong doi chieu duoc "bam nut" voi "robot doi trang thai".
    "wireless_remote": (40, np.uint8),
}


class Buffer:
    """Bo dem cap phat truoc. Khong noi mang trong callback 1 kHz."""

    def __init__(self, capacity):
        self.capacity = int(capacity)
        self.n = 0
        self.full = False
        self.buf = {k: (np.zeros(self.capacity, dtype=dt) if c == 0
                        else np.zeros((self.capacity, c), dtype=dt))
                    for k, (c, dt) in FIELDS.items()}
        self.mark_t = []
        self.mark_txt = []

    def append(self, **kw):
        i = self.n
        if i >= self.capacity:
            self.full = True
            return False
        for k, v in kw.items():
            self.buf[k][i] = v
        self.n = i + 1
        return True

    def mark(self, t, text):
        self.mark_t.append(float(t))
        self.mark_txt.append(str(text))

    def save(self, path, meta=None):
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        m = dict(meta or {})
        m.update(schema=SCHEMA, n_frames=self.n, saved_at=time.strftime("%Y-%m-%d %H:%M:%S"))
        out = {k: v[:self.n] for k, v in self.buf.items()}
        out["meta"] = np.array(json.dumps(m, ensure_ascii=False))
        out["mark_t"] = np.array(self.mark_t, dtype=np.float64)
        out["mark_txt"] = np.array(self.mark_txt, dtype=object) if self.mark_txt \
            else np.array([], dtype=object)
        np.savez_compressed(path, **out)
        return path


class Recording:
    """Ban ghi da doc len bo nho. Chi so khop = thu tu G1JointIndex."""

    def __init__(self, path):
        z = np.load(path, allow_pickle=True)
        self.path = path
        self.meta = json.loads(str(z["meta"]))
        # Ban ghi cu thieu truong moi: de rong thay vi tu choi doc. Cac ban ghi
        # cu la nguon sinh test vector nen khong duoc lam hong chung.
        self.missing = []
        for k in FIELDS:
            if k in z:
                setattr(self, k, z[k])
            else:
                setattr(self, k, np.zeros(0))
                self.missing.append(k)
        self.mark_t = z["mark_t"]
        self.mark_txt = list(z["mark_txt"])
        got = self.meta.get("schema")
        if got is None or got > SCHEMA:
            raise ValueError(f"schema ban ghi = {got}, ma nay doc toi schema {SCHEMA}")
        self.schema = got

    def __len__(self):
        return len(self.t)

    @property
    def duration(self):
        return float(self.t[-1] - self.t[0]) if len(self) > 1 else 0.0

    @property
    def rate(self):
        return (len(self) - 1) / self.duration if self.duration > 0 else 0.0

    def frame(self, i):
        """Dau vao cho LegOdometry/BalanceWBC tai khung i, da cat ve 29 khop."""
        return dict(
            t=float(self.t[i]),
            q=self.q[i, :N_MOTOR].astype(np.float64),
            dq=self.dq[i, :N_MOTOR].astype(np.float64),
            tau_est=self.tau_est[i, :N_MOTOR].astype(np.float64),
            quat=self.quat[i].astype(np.float64),
            gyro=self.gyro[i].astype(np.float64),
            accel=self.accel[i].astype(np.float64),
        )

    def summary(self):
        s = [f"ban ghi: {self.path}",
             f"  {len(self)} khung | {self.duration:.1f} s | {self.rate:.0f} Hz"]
        for k, v in self.meta.items():
            if k not in ("schema", "n_frames"):
                s.append(f"  {k}: {v}")
        if self.missing:
            s.append(f"  (ban ghi schema {self.schema}, thieu: {', '.join(self.missing)})")
        if len(self.mark_t):
            s.append("  moc su kien:")
            t0 = self.t[0] if len(self) else 0.0
            for tm, tx in zip(self.mark_t, self.mark_txt):
                s.append(f"    t={tm - t0:6.2f}s  {tx}")
        # kiem tra suc khoe du lieu: khe thoi gian bat thuong = mat goi
        if len(self) > 2:
            dt = np.diff(self.t)
            bad = int((dt > 5 * np.median(dt)).sum())
            s.append(f"  chu ky: TB {np.median(dt)*1e3:.2f} ms | "
                     f"max {dt.max()*1e3:.1f} ms | so lan gian doan {bad}")
        return "\n".join(s)


def default_dir():
    """Thu muc ghi mac dinh = G1/data/recordings TRONG REPO.

    KHONG duoc tinh bang "len bon cap tu __file__": khi chay qua ros2 run thi
    __file__ nam trong G1/install/g1_wbc/lib/python3.12/site-packages/, len bon
    cap ra G1/install/g1_wbc/ - va ban ghi se roi vao cay install, nghia la mat
    sach o lan colcon build sau. Da gap that o lan chay dau tien.

    Nen tim NGUOC LEN tu ca __file__ lan thu muc hien tai cho den khi thay dau
    hieu cua repo.
    """
    env = os.environ.get("G1_DATA_DIR")
    if env:
        return env
    for start in (os.path.dirname(os.path.abspath(__file__)), os.getcwd()):
        d = start
        while True:
            if os.path.isdir(os.path.join(d, "G1", "src", "g1_wbc")):
                return os.path.join(d, "G1", "data", "recordings")
            parent = os.path.dirname(d)
            if parent == d:
                break
            d = parent
    raise RuntimeError(
        "khong tim duoc repo de ghi - chay tu trong repo, hoac dat tham so "
        "out_dir, hoac bien moi truong G1_DATA_DIR")


def find_robot_interface(subnet="192.168.123."):
    """Tim card mang dang o CUNG SUBNET voi robot.

    Vi sao can: may nay co hai card wifi va chung DOI CHO MANG cho nhau giua cac
    lan ket noi - co luc wlp0s20f3 o mang robot, co luc card USB moi o mang
    robot. Ghi cung ten card vao lenh thi hong im lang: node chay, khong bao loi,
    chi la khong bao gio nhan duoc goi nao. Da mat mot phien ghi 70 giay vi vay.

    Tra ve (ten_card, dia_chi) hoac (None, None).
    """
    import glob
    import socket
    import struct
    try:
        import fcntl
    except ImportError:
        return None, None
    sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for path in sorted(glob.glob("/sys/class/net/*")):
        name = os.path.basename(path)
        if name == "lo":
            continue
        try:
            if open(os.path.join(path, "carrier")).read().strip() != "1":
                continue
            addr = socket.inet_ntoa(fcntl.ioctl(
                sk.fileno(), 0x8915,  # SIOCGIFADDR
                struct.pack("256s", name[:15].encode()))[20:24])
        except OSError:
            continue
        if addr.startswith(subnet):
            return name, addr
    return None, None
