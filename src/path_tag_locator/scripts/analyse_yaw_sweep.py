#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyse_yaw_sweep.py
====================
Decide whether the chain's rotation error lives in the HAND-EYE or in
FRONT_CAM, from a yaw-sweep session. OFFLINE — no ROS, no robot.

The question it answers
-----------------------
Expressing each calibrated tag's normal error as a vector and asking which
frame holds it constant gives hand-cam 79.6 % and mobile-base 82.3 % — a
tie. A calibration session only ever spins the camera about its own optical
axis, and that is exactly the motion a vertical-tilt signature cannot
separate. Both candidate fixes are expensive (`handeye_calib` is 15–30
captures; re-fitting `T_mb2fc` invalidates a 108-hop navigation
verification), so guessing is not an option.

How the sweep resolves it
-------------------------
One path tag, one ref tag, N camera yaws, base never moves. Then
``T_ab2mb @ T_mb2fc @ T_fc2B`` is identical in every entry and CANCELS —
front_cam simply cannot appear in the variation. A hand-eye error is fixed
in the EE frame, and sweeping the camera yaw rotates the EE about the
vertical, so the computed tag position traces a **circle**:

    RADIUS  = the arm-side (hand-eye) error magnitude — needs NO ground
              truth, which is what makes it usable here
    CENTRE  = everything that does not rotate with the yaw

Forward-model sensitivity through the real chain:

    front_cam rotation  +2 deg   ->  spread   0.00 mm
    hand-eye rotation   +2 deg   ->  spread  22.20 mm
    hand-eye translation 20 mm   ->  spread  21.79 mm

so a large radius means HAND-EYE. A radius near zero, combined with a large
absolute error in the normal session, means FRONT_CAM.

⚠️ Reads the per-attempt ``entries/``, not ``map_world.yaml`` — that file
upserts per tag and every entry here is the SAME tag, so it would keep only
the last one and destroy the entire measurement.

Usage::

    python3 analyse_yaw_sweep.py <session_dir>
    python3 analyse_yaw_sweep.py --self-test
"""
import argparse
import glob
import math
import os
import sys

import numpy as np

try:
    import yaml
except ImportError:
    print("FAIL: PyYAML required")
    sys.exit(1)

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)

# Decision thresholds, set from the sensitivity above and the measured
# session-to-session repeatability (sd_xy 2.9 mm, so ~4 mm of scatter is
# expected between two entries even with a perfect chain).
R_HANDEYE_MM = 6.0        # above this the arm side is clearly implicated
R_CLEAN_MM = 3.0          # below this the arm side is quiet
MM_PER_DEG_HANDEYE = 11.1     # 22.20 mm per 2 deg, from the forward model


def load_entries(session_dir):
    """[(yaw_deg, x, y, z, tag, ref)] from the per-attempt records."""
    files = sorted(glob.glob(os.path.join(session_dir, "entries", "*_ok.yaml")))
    if not files:
        raise SystemExit(f"no entries/*_ok.yaml under {session_dir}")
    out = []
    for f in files:
        d = yaml.safe_load(open(f))
        res = d.get("result_world") or {}
        pos = res.get("position_m")
        if pos is None:
            continue
        # The yaw marker is a plan key the orchestrator drops, so recover
        # the executed value from the TCP that was actually used.
        tcp = d.get("view_tcp_mm_deg") or d.get("tcp_pose_mm_deg")
        yaw = float(tcp[5]) if tcp else float("nan")
        out.append((yaw, pos[0], pos[1], pos[2],
                    int(d["path_tag_id"]), int(d["ref_tag_id"])))
    return out


def fit_circle(P):
    """Algebraic circle fit (Kasa). P is (N,2) in metres.
    Returns (cx, cy, r, residual_rms) in metres."""
    x, y = P[:, 0], P[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0], sol[1]
    r = math.sqrt(max(0.0, sol[2] + cx ** 2 + cy ** 2))
    resid = np.hypot(x - cx, y - cy) - r
    return cx, cy, r, float(np.sqrt(np.mean(resid ** 2)))


def report(rows, design_xy=None):
    yaws = np.array([r[0] for r in rows])
    P = np.array([[r[1], r[2]] for r in rows])
    Z = np.array([r[3] for r in rows])
    tag = rows[0][4]
    ref = rows[0][5]

    print(f"tag {tag}, ref {ref}, {len(rows)} entries")
    if len({r[4] for r in rows}) > 1 or len({r[5] for r in rows}) > 1:
        print("  ⚠️ entries do NOT all share one tag/ref — this is not a yaw "
              "sweep, the method does not apply")
        return 1
    span = float(np.nanmax(yaws) - np.nanmin(yaws))
    print(f"  camera yaw span {span:.0f}°  ({', '.join('%.0f' % y for y in yaws)})")
    if span < 120:
        print("  ⚠️ span under 120° — a circle cannot be resolved; the radius "
              "below is not trustworthy")

    spread = float(np.max(np.linalg.norm(P - P.mean(axis=0), axis=1)) * 1e3)
    cx, cy, r, res = fit_circle(P)
    print(f"\n{'yaw':>6} {'x (m)':>10} {'y (m)':>10} {'z (mm)':>9}"
          f" {'from centre':>12}")
    for (yw, x, y, z, _, _) in rows:
        print(f"{yw:>6.0f} {x:>10.4f} {y:>10.4f} {z*1e3:>9.1f}"
              f" {math.hypot(x-cx, y-cy)*1e3:>10.2f}mm")

    print(f"\n  raw spread about the mean : {spread:7.2f} mm")
    print(f"  CIRCLE  radius            : {r*1e3:7.2f} mm   <- arm-side error")
    print(f"          centre            : ({cx:.4f}, {cy:.4f})")
    print(f"          fit residual rms  : {res*1e3:7.2f} mm")
    print(f"  z spread                  : {(Z.max()-Z.min())*1e3:7.2f} mm"
          f"   (mean {Z.mean()*1e3:+.1f})")
    if design_xy is not None:
        d = math.hypot(cx - design_xy[0], cy - design_xy[1]) * 1e3
        print(f"  centre vs map.yaml design : {d:7.2f} mm")

    print("\n" + "=" * 62)
    if res * 1e3 > max(3.0, 0.5 * r * 1e3) and r * 1e3 > R_CLEAN_MM:
        print("  ⚠️ the points do not lie on a circle (residual is a large")
        print("     fraction of the radius). Something varied besides the")
        print("     camera yaw — did the base move? Treat the radius as an")
        print("     upper bound only.")
    if r * 1e3 >= R_HANDEYE_MM:
        print(f"  VERDICT: HAND-EYE. Radius {r*1e3:.1f} mm is arm-side error,")
        print(f"           roughly equivalent to {r*1e3/MM_PER_DEG_HANDEYE:.2f}° of")
        print(f"           hand-eye rotation (or a comparable translation).")
        print("           -> run handeye_calib; do NOT touch T_mb2fc yet.")
    elif r * 1e3 <= R_CLEAN_MM:
        print(f"  VERDICT: NOT the hand-eye. Radius {r*1e3:.1f} mm is within the")
        print("           ~3 mm this rig scatters anyway. If the normal")
        print("           session still shows a large absolute error and a")
        print("           tag-normal tilt, the remaining suspect is FRONT_CAM")
        print("           (T_mb2fc) -> re-fit it, and re-run the 108-hop")
        print("           navigation check afterwards.")
    else:
        print(f"  VERDICT: INCONCLUSIVE. Radius {r*1e3:.1f} mm sits between the")
        print(f"           {R_CLEAN_MM:.0f} mm noise floor and the {R_HANDEYE_MM:.0f} mm")
        print("           decision line — both terms are probably present.")
        print("           Re-run with more yaws, or accept that both need work.")
    print("=" * 62)
    return 0


# ---------------------------------------------------------------- self-test
def self_test():
    """Drive the REAL chain forward with a known error and check the verdict.

    Not a synthetic circle: the observations are generated from true
    geometry through compute_view_tcp + compute_T_A2B, so the test exercises
    the same maths the robot will.
    """
    sys.path.insert(0, os.path.join(PKG, "src"))
    from path_tag_locator.chain import compute_T_A2B
    from path_tag_locator.constants import load_extrinsics_full
    from path_tag_locator.calibration.view_pose import compute_view_tcp
    from path_tag_locator.calibration.plan_io import load_reference_tags
    from path_tag_locator.geometry import rpy_deg_to_R, invert_T, \
        pose_fr5_to_matrix_m

    cfg = os.path.join(PKG, "config")
    _ext = load_extrinsics_full(os.path.join(cfg, "extrinsics.yaml"))
    T_ab2mb, T_mb2fc_T = _ext.T_ab2mb, _ext.T_mb2fc_chain   # detections' frame
    T_hc2ee_T = np.load(os.path.join(cfg, "hand_eye", "T_hc2ee.npz"))["arr_0"]
    REFS = {k: v.T_world for k, v in load_reference_tags(
        os.path.join(cfg, "reference_tags.yaml")).items()}

    def pert(R3, t=(0, 0, 0)):
        T = np.eye(4)
        T[:3, :3] = R3
        T[:3, 3] = t
        return T

    def face_up(x, y, z):
        T = np.eye(4)
        T[:3, :3] = rpy_deg_to_R(180.0, 0.0, 0.0)
        T[:3, 3] = [x, y, z]
        return T

    TAGB = face_up(-1.71, -0.65, -0.080)
    T_w2mb = np.eye(4)
    T_w2mb[:3, :3] = rpy_deg_to_R(0, 0, 90.0)
    T_w2mb[:3, 3] = [-1.71, -0.65 - 0.55, -0.080]
    REF = 0

    def sweep(he_err, fc_err, yaws, noise_mm=0.0, seed=0):
        rng = np.random.RandomState(seed)
        rows = []
        for yw in yaws:
            he = he_err @ T_hc2ee_T
            tcp = compute_view_tcp(T_A_world=REFS[REF], T_world2mb=T_w2mb,
                                   T_ab2mb=T_ab2mb, T_hc2ee=he,
                                   view_distance_m=0.50, yaw_deg=float(yw))
            T_ab2ee = pose_fr5_to_matrix_m(tcp)
            T_hc_w = T_w2mb @ invert_T(T_ab2mb) @ T_ab2ee @ invert_T(T_hc2ee_T)
            T_hc2A = invert_T(T_hc_w) @ REFS[REF]
            T_hc2A[:3, 3] += rng.normal(0, noise_mm / 1e3, 3)
            T_fc_w = T_w2mb @ (T_mb2fc_T @ fc_err)
            T_fc2B = invert_T(T_fc_w) @ TAGB
            r = compute_T_A2B(T_hc2A=T_hc2A, T_fc2B=T_fc2B,
                              tcp_pose_mm_deg=tcp, T_hc2ee=he,
                              T_ab2mb=T_ab2mb, T_mb2fc=T_mb2fc_T,
                              lift_height_m=0.0)
            p = (REFS[REF] @ r["T_A2B"])[:3, 3]
            rows.append((float(tcp[5]), p[0], p[1], p[2], 103, REF))
        return rows

    YAWS = [0, 60, 120, 180, 240, 300]
    bad = 0
    for label, he, fc, want in (
            ("clean", np.eye(4), np.eye(4), "NOT the hand-eye"),
            ("front_cam rot +2 deg", np.eye(4),
             pert(rpy_deg_to_R(2, 0, 0)), "NOT the hand-eye"),
            ("hand-eye rot +2 deg", pert(rpy_deg_to_R(2, 0, 0)),
             np.eye(4), "HAND-EYE"),
            ("hand-eye trans +20 mm", pert(np.eye(3), (0.02, 0, 0)),
             np.eye(4), "HAND-EYE")):
        rows = sweep(he, fc, YAWS, noise_mm=1.0, seed=7)
        P = np.array([[r[1], r[2]] for r in rows])
        _, _, r_, res = fit_circle(P)
        got = ("HAND-EYE" if r_ * 1e3 >= R_HANDEYE_MM
               else "NOT the hand-eye" if r_ * 1e3 <= R_CLEAN_MM
               else "INCONCLUSIVE")
        ok = got == want
        bad += (not ok)
        print(f"  [{'ok ' if ok else 'FAIL'}] {label:<22} radius "
              f"{r_*1e3:6.2f} mm  fit-rms {res*1e3:5.2f}  -> {got}")

    print("\n  the two that matter: front_cam is INVISIBLE to the sweep "
          "(that is the design),\n  hand-eye shows up in both its rotation "
          "and its translation.")
    print(f"\n{4 - bad}/4 cases classified correctly")
    return 1 if bad else 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", nargs="?", help="yaw-sweep session directory")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args()
    if a.self_test:
        return self_test()
    if not a.session:
        ap.error("give a session directory, or --self-test")
    if not os.path.isdir(a.session):
        print(f"FAIL: no such directory {a.session}")
        return 1
    rows = load_entries(a.session)
    if len(rows) < 4:
        print(f"FAIL: only {len(rows)} successful entries; a circle needs 4+")
        return 1
    return report(rows)


if __name__ == "__main__":
    sys.exit(main())
