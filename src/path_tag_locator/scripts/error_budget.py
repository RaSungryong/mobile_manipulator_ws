#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
error_budget.py — everything expressed as PATH-TAG world-position error.

Monte-Carlo through the REAL chain (chain.compute_T_A2B) for one plan
entry's geometry: perturb each input with a representative noise, read
the resulting B-position scatter. Random sources are reported as
per-source sigma / P95; the constant transforms (hand-eye, extrinsics)
are reported as SENSITIVITIES (they bias, not scatter).

Noise model (all overridable):
  --px      corner/pixel noise sigma, both cameras     [px, def 0.5]
  --tilt    hand/front cam out-of-plane rotation sigma [deg, def 0.5]
  --yaw     in-plane rotation sigma                    [deg, def 0.2]
  --tcp-mm  TCP position sigma (Fairino repeatability) [mm, def 0.05]
  --tcp-deg TCP rotation sigma                         [deg, def 0.02]
Pixel noise maps to camera-frame translation as
  lateral sigma = z * px / f,   depth sigma = 2 * z^2 * px / (f * s)
(s = tag size), the standard planar-target scaling.

Usage (no ROS needed):
    python3 error_budget.py [--entry 105] [--view-m 0.8] [-n 3000]
"""
import argparse
import math
from pathlib import Path

import numpy as np
import yaml

PKG = Path(__file__).resolve().parent.parent
WS = PKG.parent.parent
import sys
sys.path.insert(0, str(PKG / "src"))
sys.path.insert(0, str(WS / "src" / "apriltag_nav" / "src"))

from path_tag_locator.chain import compute_T_A2B, compute_T_B_world
from path_tag_locator.constants import load_extrinsics
from path_tag_locator.geometry import invert_T, pose_fr5_to_matrix_m, \
    matrix_m_to_pose_fr5
from path_tag_locator.hand_eye import load_T_hc2ee

FLOOR_Z = -0.080
ZONE_YAW = {"A": 0.0, "DOCK": 0.0, "B": 90.0, "D": 90.0,
            "C": -90.0, "E": -90.0}
FX = 910.0


def rz(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def small_rot(rng, tilt_sigma_deg, yaw_sigma_deg):
    """Random small rotation: tilt about a random in-plane axis + yaw."""
    ang = math.radians(rng.normal(0, tilt_sigma_deg))
    phi = rng.uniform(0, 2 * math.pi)
    axis = np.array([math.cos(phi), math.sin(phi), 0.0])
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    R_tilt = (np.eye(3) + math.sin(ang) * K
              + (1 - math.cos(ang)) * (K @ K))
    return R_tilt @ rz(rng.normal(0, yaw_sigma_deg))


def face_up(x, y, z, yaw_deg=0.0):
    T = np.eye(4)
    T[:3, :3] = rz(yaw_deg) @ np.diag([1.0, -1.0, -1.0])
    T[:3, 3] = [x, y, z]
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--entry", type=int, default=105)
    ap.add_argument("--view-m", type=float, default=0.8)
    ap.add_argument("-n", type=int, default=3000)
    ap.add_argument("--px", type=float, default=0.5)
    ap.add_argument("--tilt", type=float, default=0.5)
    ap.add_argument("--yaw", type=float, default=0.2)
    ap.add_argument("--tcp-mm", type=float, default=0.05)
    ap.add_argument("--tcp-deg", type=float, default=0.02)
    a = ap.parse_args()
    rng = np.random.RandomState(20260903)

    # ---- exact geometry for the chosen entry --------------------------
    tags = yaml.safe_load(open(
        WS / "src/apriltag_nav/config/map.yaml"))["tags"]
    info = tags[a.entry]
    zone = info["zone"]
    plate = 1 if zone in ("B", "C") else 2
    plan = yaml.safe_load(open(
        PKG / ("config/calibration_plan_plate%d.yaml" % plate)))
    entry = next(e for e in plan["plan"] if e["path_tag_id"] == a.entry)
    reffile = ("config/reference_tags.yaml" if plate == 1
               else "config/reference_tags_plate2.yaml")
    refs = {r["id"]: r for r in yaml.safe_load(
        open(PKG / reffile))["reference_tags"]}
    ref = refs[entry["ref_tag_id"]]

    T_ab2mb, T_mb2fc = load_extrinsics(str(PKG / "config/extrinsics.yaml"))
    T_hc2ee = load_T_hc2ee(str(PKG / "config/hand_eye/T_hc2ee.npz"))

    heading = ZONE_YAW[zone]
    hr = math.radians(heading)
    T_wmb = np.eye(4)
    T_wmb[:3, :3] = rz(heading)
    T_wmb[:3, 3] = [info["x"] - 0.55 * math.cos(hr),
                    info["y"] - 0.55 * math.sin(hr), FLOOR_Z]

    T_wB = face_up(info["x"], info["y"], FLOOR_Z, heading)
    T_wA = face_up(*ref["position_m"], yaw_deg=ref["rpy_deg"][2])

    T_A2hc = np.eye(4)
    T_A2hc[2, 3] = -a.view_m
    T_whc = T_wA @ T_A2hc
    T_hc2A0 = invert_T(T_A2hc)
    T_wfc = T_wmb @ T_mb2fc
    T_fc2B0 = invert_T(T_wfc) @ T_wB
    T_wab = T_wmb @ invert_T(T_ab2mb)
    tcp0 = matrix_m_to_pose_fr5(invert_T(T_wab) @ T_whc @ T_hc2ee)

    truth = np.array([info["x"], info["y"], FLOOR_Z, heading])

    def run(T_hc2A, T_fc2B, tcp, ab2mb=T_ab2mb, mb2fc=T_mb2fc,
            hc2ee=T_hc2ee):
        """Returns [x, y, z, laid_yaw_deg] — BOTH deliverables. The
        laid yaw is recovered from the face-up convention
        R = Rz(yaw) @ diag(1,-1,-1): yaw = atan2(R10, R00)."""
        out = compute_T_A2B(T_hc2A=T_hc2A, T_fc2B=T_fc2B,
                            tcp_pose_mm_deg=tcp, T_hc2ee=hc2ee,
                            T_ab2mb=ab2mb, T_mb2fc=mb2fc)
        T = compute_T_B_world(np.array(yaml_ref_T(ref)), out["T_A2B"])
        yaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
        return np.array([T[0, 3], T[1, 3], T[2, 3], yaw])

    def yaml_ref_T(r):
        return face_up(*r["position_m"], yaw_deg=r["rpy_deg"][2])

    base = run(T_hc2A0, T_fc2B0, tcp0)
    assert np.linalg.norm(base - truth) < 1e-8, base - truth
    print("entry %d (zone %s, ref %d): geometry closes exactly; "
          "A->B lever %.3f m; hand-cam view %.2f m\n"
          % (a.entry, zone, entry["ref_tag_id"],
             np.linalg.norm((invert_T(T_wA) @ T_wB)[:3, 3]), a.view_m))

    # ---- noise scalings ----------------------------------------------
    def cam_noise(z, s_tag, n):
        lat = z * a.px / FX
        dep = 2.0 * z * z * a.px / (FX * s_tag)
        t = np.zeros((n, 3))
        t[:, 0] = rng.normal(0, lat, n)
        t[:, 1] = rng.normal(0, lat, n)
        t[:, 2] = rng.normal(0, dep, n)
        return t

    def perturb_obs(T0, dt, dR):
        T = T0.copy()
        T[:3, :3] = T0[:3, :3] @ dR
        T[:3, 3] = T0[:3, 3] + dt
        return T

    sources = {}
    N = a.n

    # 1. hand-cam observation (translation + rotation separately)
    errs_t, errs_r = [], []
    tA = cam_noise(a.view_m, 0.09, N)
    for i in range(N):
        errs_t.append(run(perturb_obs(T_hc2A0, tA[i], np.eye(3)),
                          T_fc2B0, tcp0))
        errs_r.append(run(perturb_obs(T_hc2A0, np.zeros(3),
                                      small_rot(rng, a.tilt, a.yaw)),
                          T_fc2B0, tcp0))
    sources["hand-cam translation (%.1f px)" % a.px] = np.array(errs_t)
    sources["hand-cam ROTATION (tilt %.1f deg)" % a.tilt] = np.array(errs_r)

    # 2. front-cam observation
    errs_t, errs_r = [], []
    tB = cam_noise(0.30, 0.09, N)
    for i in range(N):
        errs_t.append(run(T_hc2A0,
                          perturb_obs(T_fc2B0, tB[i], np.eye(3)), tcp0))
        errs_r.append(run(T_hc2A0,
                          perturb_obs(T_fc2B0, np.zeros(3),
                                      small_rot(rng, a.tilt, a.yaw)),
                          tcp0))
    sources["front-cam translation (%.1f px)" % a.px] = np.array(errs_t)
    sources["front-cam rotation (tilt %.1f deg)" % a.tilt] = np.array(errs_r)

    # 3. TCP pose
    errs = []
    for i in range(N):
        tcp = list(tcp0)
        for k in range(3):
            tcp[k] += rng.normal(0, a.tcp_mm)
        for k in range(3, 6):
            tcp[k] += rng.normal(0, a.tcp_deg)
        errs.append(run(T_hc2A0, T_fc2B0, tcp))
    sources["TCP pose (%.2f mm / %.2f deg)" % (a.tcp_mm, a.tcp_deg)] = \
        np.array(errs)

    # ---- report random sources ---------------------------------------
    print("%-38s %9s %9s %8s %9s %9s"
          % ("random source", "sig_xy mm", "P95_xy mm", "sig_z mm",
             "sig_yaw d", "P95_yaw d"))
    combined = np.zeros((N, 4))
    for name, e in sources.items():
        d = e - truth
        d[:, 3] = (d[:, 3] + 180) % 360 - 180
        d[:, :3] *= 1000.0                # mm; yaw stays deg
        dxy = np.linalg.norm(d[:, :2], axis=1)
        print("%-38s %9.2f %9.2f %8.2f %9.3f %9.3f"
              % (name, np.sqrt(d[:, 0].var() + d[:, 1].var()),
                 np.percentile(dxy, 95), d[:, 2].std(),
                 d[:, 3].std(), np.percentile(np.abs(d[:, 3]), 95)))
        combined += d
    d = combined
    dxy = np.linalg.norm(d[:, :2], axis=1)
    print("%-38s %9.2f %9.2f %8.2f %9.3f %9.3f"
          % ("ALL COMBINED (rss)",
             np.sqrt(d[:, 0].var() + d[:, 1].var()),
             np.percentile(dxy, 95), d[:, 2].std(),
             d[:, 3].std(), np.percentile(np.abs(d[:, 3]), 95)))

    # ---- systematic sensitivities ------------------------------------
    print("\nsystematic sensitivities (bias, not scatter):")

    def sens(label, **kw):
        b = run(T_hc2A0, T_fc2B0, tcp0, **kw) - truth
        print("  %-42s -> B shift (%+.2f, %+.2f, %+.2f) mm, yaw %+.3f deg"
              % (label, b[0] * 1000, b[1] * 1000, b[2] * 1000,
                 (b[3] + 180) % 360 - 180))

    for axis, v in (("x", [0.001, 0, 0]), ("y", [0, 0.001, 0]),
                    ("z", [0, 0, 0.001])):
        M = T_ab2mb.copy()
        M[:3, 3] += v
        sens("extrinsics T_ab2mb +1 mm %s" % axis, ab2mb=M)
    for axis, v in (("x", [0.001, 0, 0]), ("y", [0, 0.001, 0]),
                    ("z", [0, 0, 0.001])):
        M = T_mb2fc.copy()
        M[:3, 3] += v
        sens("extrinsics T_mb2fc +1 mm %s" % axis, mb2fc=M)
    M = T_hc2ee.copy()
    M[:3, 3] += [0.001, 0, 0]
    sens("hand-eye +1 mm x", hc2ee=M)
    M = T_hc2ee.copy()
    M[:3, :3] = M[:3, :3] @ rz(0.1)
    sens("hand-eye +0.1 deg yaw", hc2ee=M)
    Aerr = yaml_ref_T(ref)
    # ref tag yaw error: rotates result about A
    T_wA_e = face_up(*ref["position_m"], yaw_deg=ref["rpy_deg"][2] + 0.1)
    out = compute_T_A2B(T_hc2A=T_hc2A0, T_fc2B=T_fc2B0,
                        tcp_pose_mm_deg=tcp0, T_hc2ee=T_hc2ee,
                        T_ab2mb=T_ab2mb, T_mb2fc=T_mb2fc)
    Tb = compute_T_B_world(T_wA_e, out["T_A2B"])
    yawb = math.degrees(math.atan2(Tb[1, 0], Tb[0, 0]))
    b = np.array([Tb[0, 3], Tb[1, 3], Tb[2, 3], yawb]) - truth
    print("  %-42s -> B shift (%+.2f, %+.2f, %+.2f) mm, yaw %+.3f deg"
          % ("reference_tags.yaml yaw +0.1 deg", b[0] * 1000,
             b[1] * 1000, b[2] * 1000, (b[3] + 180) % 360 - 180))


if __name__ == "__main__":
    main()
