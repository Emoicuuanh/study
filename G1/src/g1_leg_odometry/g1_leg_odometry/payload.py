"""Ap khoi luong gan them len mo hinh Pinocchio.

g1_29dof.urdf goc khong co: tay Inspire (thay tay cao su), lidar Mid360,
thiet bi tren lung. Tong chenh ~3.5 kg tren 35.1 kg = 10%. Voi WBC thi 10%
khoi luong sai la 34 N trong luc khong duoc bu, va CoM lech - ca hai deu
vao thang bai toan QP.

Giu URDF goc nguyen ban, ap tai trong tu file YAML luc nap.
"""
import numpy as np
import pinocchio as pin
import yaml


def apply_payload(model, yaml_path, verbose=False):
    """Cong khoi luong tu file YAML vao model Pinocchio (sua tai cho).

    Tra ve (tong_khoi_luong_truoc, tong_khoi_luong_sau).
    """
    before = sum(I.mass for I in model.inertias)
    with open(yaml_path) as f:
        items = (yaml.safe_load(f) or {}).get("payload", []) or []

    for it in items:
        fid = model.getFrameId(it["frame"])
        if fid >= model.nframes:
            raise ValueError(f"payload '{it['name']}': khong co frame '{it['frame']}'")
        fr = model.frames[fid]
        m = float(it["mass"])
        lever = np.array(it.get("xyz", [0.0, 0.0, 0.0]), dtype=float)
        # Coi la khoi diem, nhung cho mot quan tinh cau nho khac 0 de ma tran
        # quan tinh khong suy bien ve mat so.
        Ic = pin.Inertia.FromSphere(m, 0.05).inertia
        inertia_in_frame = pin.Inertia(m, lever, Ic)
        # dua ve he cua khop cha
        model.appendBodyToJoint(fr.parentJoint, inertia_in_frame, fr.placement)
        if verbose:
            print(f"  + {it['name']:28s} {m:5.3f} kg @ {it['frame']} ({it.get('source','?')})")

    after = sum(I.mass for I in model.inertias)
    if verbose:
        print(f"  tong: {before:.3f} -> {after:.3f} kg  (+{after-before:.3f})")
    return before, after
