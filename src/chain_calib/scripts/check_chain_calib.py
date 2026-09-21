#!/usr/bin/env python3
"""Offline check of chain_calib (sheet.py, solver.py, session.py and the
non-ROS parts of chain_calib.py).

  1. Corner convention against the REAL sheet: the FINAL2 PDF is rasterised
     (pdftoppm) and detected with dt_apriltags — the library that
     robot_camera_node runs — and every tag must land at its layout
     position with corner 0 bottom-left (handoff §5.1). Skipped without
     pdftoppm.
  2. multi-tag PnP on rendered corners (hand_cam with distortion, front_cam
     without) recovers the camera pose; a single square-on tag is NOT
     flagged, a single oblique tag with a near-tie IS.
  3. A synthetic session laid the way README §2 suggests — front_cam over
     the 300/301 pair, hand_cam over the rest of the grid, real hand-eye /
     extrinsics with PLANTED D and F errors, 20-frame corner noise — must
     recover D and F, attribute them, bring the user's T_A2B metric to the
     noise floor, and generalise to held-out views. Persistence with the
     corners round-trips and `solve` can re-solve the stored corners at a
     different print scale.

    source devel/setup.bash && python3 src/chain_calib/scripts/check_chain_calib.py
"""
import glob
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "path_tag_locator", "src"))

from chain_calib import sheet as SH                                   # noqa: E402
from chain_calib import solver as CC                                  # noqa: E402
from chain_calib.session import coverage, load_samples, sample_view, save_samples  # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                 # noqa: E402
from path_tag_locator.constants import load_extrinsics_full           # noqa: E402
from path_tag_locator.geometry import invert_T, matrix_m_to_pose_fr5  # noqa: E402
from path_tag_locator.hand_eye import load_T_hc2ee                    # noqa: E402
from path_tag_locator.handeye_sweep import view_T_cam2tag             # noqa: E402

_n = [0]; _bad = [0]
def check(name, ok, detail=""):
    _n[0] += 1; _bad[0] += (not ok)
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")

SHEET_DIR = os.path.normpath(os.path.join(_HERE, "..", "sheet"))
LAYOUT = os.path.join(SHEET_DIR, "A0_landscape_tag200_300-309_5x2_FINAL2_layout.json")
PDF = os.path.join(SHEET_DIR, "A0_landscape_tag200_300-309_5x2_FINAL2.pdf")
CFG = os.path.join(_HERE, "..", "..", "path_tag_locator", "config")
ext = load_extrinsics_full(os.path.join(CFG, "extrinsics.yaml"))
T_ab2mb, T_mb2fc = ext.T_ab2mb, ext.T_mb2fc_level
H_file = load_T_hc2ee(os.path.join(CFG, "hand_eye", "T_hc2ee.npz"))
# D435 colour 640x480 and Femto Bolt 1280x720 — the numbers the other checks use
K_hand = np.array([[609.3, 0, 321.5], [0, 608.6, 238.7], [0, 0, 1.0]])
D_hand = np.array([-0.05, 0.01, 0.0005, -0.0003, 0.0])
K_front = np.array([[750.0, 0, 638.2], [0, 750.0, 361.4], [0, 0, 1.0]])
WH_hand, WH_front = (640, 480), (1280, 720)
FRAMES = 20

sheet = SH.load_sheet(LAYOUT)


def small_T(rot_deg_xyz, t_mm):
    return CC.T_from_vec6(np.concatenate([np.radians(rot_deg_xyz), np.asarray(t_mm) / 1e3]))


# ----------------------------------------------------------------------
print("== 1. corner convention against the real PDF (dt_apriltags) ==")
if shutil.which("pdftoppm") is None:
    print("  (pdftoppm not installed — skipped)")
else:
    d = tempfile.mkdtemp()
    try:
        subprocess.run(["pdftoppm", "-r", "40", "-gray", "-png", PDF, os.path.join(d, "sheet")], check=True)
        import cv2
        from dt_apriltags import Detector
        img = cv2.imread(glob.glob(os.path.join(d, "sheet*.png"))[0], 0)
        dets = Detector(families="tag36h11", quad_decimate=1.0).detect(img)
        px_per_mm = 40.0 / 25.4
        import json
        lay = json.load(open(LAYOUT))
        check("all 11 tags detected, hamming 0", sorted(x.tag_id for x in dets) == sheet.ids and all(x.hamming == 0 for x in dets),
              "%s" % sorted(x.tag_id for x in dets))
        worst_c, worst_o = 0.0, 0.0
        for x in dets:
            pc = np.array(lay["tags"][str(x.tag_id)]["paper_center_mm"]) * px_per_mm
            worst_c = max(worst_c, float(np.linalg.norm(x.center - pc)) / px_per_mm)
            rel = (x.corners - x.center) / px_per_mm / 45.0          # in half-sides, image x right / y down
            worst_o = max(worst_o, float(np.abs(rel - SH._CORNER_ORDER).max()))
        check("every centre at its paper position (< 0.5 mm at 40 dpi)", worst_c < 0.5, "worst %.2f mm" % worst_c)
        check("corner order = _CORNER_ORDER: 0 bottom-left, 1 bottom-right, 2 top-right, 3 top-left (as printed)",
              worst_o < 0.03, "worst deviation %.3f half-sides" % worst_o)
        # multi-tag PnP with a virtual camera looking at the raster (x right, y down, z into the paper)
        # => T_cam2W must be a pure translation (R = I) with z = the raster's focal distance
        f = 2000.0; z0 = 2.0
        Kv = np.array([[f, 0, 0.0], [0, f, 0.0], [0, 0, 1]])
        origin_px = np.array(lay["tags"]["200"]["paper_center_mm"]) * px_per_mm
        corners = {x.tag_id: (x.corners - origin_px) / px_per_mm / 1e3 * f / z0 for x in dets}
        r = SH.multi_tag_pnp(corners, sheet, Kv)
        ang = math.degrees(np.linalg.norm(CC._rotvec_from_R(r.T_cam2W[:3, :3])))
        check("PnP of the raster through a virtual camera: R = I, t = (0, 0, %.1f) m" % z0,
              ang < 0.05 and np.allclose(r.T_cam2W[:3, 3], [0, 0, z0], atol=2e-3),
              "R %.3f deg, t %s m, rms %.2f px" % (ang, np.round(r.T_cam2W[:3, 3], 4), r.rms_px))
    finally:
        shutil.rmtree(d)


# ----------------------------------------------------------------------
print("\n== 2. multi-tag PnP on rendered corners ==")
rng = np.random.RandomState(0)


def noisy(corners, sigma):
    return {k: c + rng.normal(0, sigma, c.shape) for k, c in corners.items()}


T = view_T_cam2tag(0.3, 0.45, 18.0, 60.0, 25.0)            # camera over tag 305, tilted
T_cam2W = T @ invert_T(sheet.T_W2k(305))
vis = SH.visible_tags(T_cam2W, sheet, K_hand, WH_hand, D_hand)
check("a tilted hand_cam 0.45 m over tag 305 sees several grid tags", len(vis) >= 3, "%s" % vis)
c = SH.project_sheet(T_cam2W, sheet, K_hand, D_hand, vis)
r = SH.multi_tag_pnp(noisy(c, 0.3 / math.sqrt(FRAMES)), sheet, K_hand, D_hand)
e = CC.pose_error(r.T_cam2W, T_cam2W)
check("hand_cam pose recovered WITH the distortion (< 1 mm / 0.1 deg)", e[0] < 1e-3 and e[1] < 0.1,
      "%.2f mm / %.3f deg, rms %.3f px, tags %s" % (e[0] * 1e3, e[1], r.rms_px, r.tag_ids))
r2 = SH.multi_tag_pnp(c, sheet, K_hand, None)
e2 = CC.pose_error(r2.T_cam2W, T_cam2W)
check("ignoring D on raw corners costs a visible error (why hand_cam passes CameraInfo D)", e2[0] > 3 * max(e[0], 1e-4) or e2[1] > 3 * max(e[1], 1e-3),
      "%.2f mm / %.3f deg" % (e2[0] * 1e3, e2[1]))
# front_cam: level camera 0.302 m over W (0.85, 0), image x = paper right,
# image y = paper down, optical axis into the paper -> R = I (the raster
# convention of section 1), W origin 0.85 m to the image LEFT of the axis
T_fc2W = np.eye(4); T_fc2W[:3, 3] = [-0.85, 0.0, 0.302]
visf = SH.visible_tags(T_fc2W, sheet, K_front, WH_front)
check("front_cam 0.302 m over W (0.85, 0) images exactly the 300/301 pair whole", visf == [300, 301], "%s" % visf)
cf = SH.project_sheet(T_fc2W, sheet, K_front, None, visf)
rf = SH.multi_tag_pnp(noisy(cf, 0.3 / math.sqrt(FRAMES)), sheet, K_front, None)
ef = CC.pose_error(rf.T_cam2W, T_fc2W)
check("front_cam pose from the pair (< 1 mm / 0.1 deg)", ef[0] < 1e-3 and ef[1] < 0.1, "%.2f mm / %.3f deg" % (ef[0] * 1e3, ef[1]))
r1 = SH.multi_tag_pnp({300: cf[300]}, sheet, K_front, None)
check("one square-on tag: mirror solutions coincide, NOT flagged", not any("ambiguity" in f for f in r1.flags),
      "ratio %.2f, %.1f deg apart" % (r1.ambiguity_ratio, r1.ambiguity_angle_deg))
T_obl = view_T_cam2tag(0.0, 0.6, 20.0, 0.0, 0.0) @ invert_T(sheet.T_W2k(305))
co = SH.project_sheet(T_obl, sheet, K_hand, None, [305])
ro = SH.multi_tag_pnp(noisy(co, 0.5), sheet, K_hand, None)
check("one oblique tag: the two IPPE solutions are reported (ratio / angle) for the flag to judge", ro.ambiguity_angle_deg > 2.0 and ro.ambiguity_ratio > 1.0,
      "ratio %.2f, %.1f deg apart, flags %s" % (ro.ambiguity_ratio, ro.ambiguity_angle_deg, ro.flags))
try:
    SH.multi_tag_pnp({150: cf[300]}, sheet, K_front, None); check("a tag not on the sheet is refused", False)
except RuntimeError as exc:
    check("a tag not on the sheet is refused", "on the sheet" in str(exc), str(exc)[:60])
acc = SH.accumulate_frames([{300: cf[300], 301: cf[301]}] * 15 + [{300: cf[300]}] * 5)
check("accumulate_frames keeps a tag seen in 15/20 frames, counts frames", set(acc) == {300, 301} and acc[301].n_frames == 15)
acc = SH.accumulate_frames([{300: cf[300], 301: cf[301]}] * 5 + [{300: cf[300]}] * 15)
check("… and drops one seen in 5/20 (flicker at the frame edge)", set(acc) == {300})


# ----------------------------------------------------------------------
print("\n== 3. synthetic session: sheet under the robot, planted D / F ==")
# Layout (README §2): W +x = mb +x (forward), W +y = mb -y (the robot's
# right), W +z = mb -z (into the floor); front_cam's nadir (0.55, 0) over
# W (0.85, 0) so the 300/301 pair sits whole in the usable frame; the grid
# then runs 0.15..0.60 m to the robot's right at mb x 0.55 / 0.70.
T_mb2W = np.eye(4)
T_mb2W[:3, :3] = np.diag([1.0, -1.0, -1.0])
T_mb2W[:3, 3] = np.array([0.55, 0.0, 0.0]) - T_mb2W[:3, :3] @ np.array([0.85, 0.0, 0.0])
T_ab2W = T_ab2mb @ T_mb2W
G = sheet.T_W2k(305) @ np.array([[1, 0, 0, 0.075], [0, 1, 0, 0.075], [0, 0, 1, 0], [0, 0, 0, 1.0]])   # grid centre (0.925, 0.375)


def make_session(D_true, F_true, seed, noise_px=0.3 / math.sqrt(FRAMES), n_views=24):
    rng = np.random.RandomState(seed)
    H_true = H_file @ D_true
    B_true = T_ab2mb @ T_mb2fc @ F_true
    T_fc2W_true = invert_T(B_true) @ T_ab2W
    samples, views_planned = [], []
    for dist in (0.40, 0.50):
        for tilt, azs in ((0.0, (0.0,)), (15.0, (0, 90, 180, 270)), (22.0, (45, 135, 225, 315))):
            for az in azs:
                for spin in (0.0, 90.0):
                    views_planned.append((dist, tilt, az, spin))
    step = max(1, len(views_planned) // n_views)
    for i, (dist, tilt, az, spin) in enumerate(views_planned[::step][:n_views]):
        T_cam2G = view_T_cam2tag(0.0, dist, tilt, az, spin)
        T_hc2W_true = T_cam2G @ invert_T(G)
        T_ab2ee = T_ab2W @ invert_T(T_hc2W_true) @ H_true          # T_ab2ee = T_ab2W . T_W2hc . T_hc2ee
        vis = SH.visible_tags(T_hc2W_true, sheet, K_hand, WH_hand, D_hand)
        if len(vis) < 2:
            continue
        hc = {k: v + rng.normal(0, noise_px, (4, 2)) for k, v in SH.project_sheet(T_hc2W_true, sheet, K_hand, D_hand, vis).items()}
        visf = SH.visible_tags(T_fc2W_true, sheet, K_front, WH_front)
        fc = {k: v + rng.normal(0, noise_px, (4, 2)) for k, v in SH.project_sheet(T_fc2W_true, sheet, K_front, None, visf).items()}
        h = SH.multi_tag_pnp(hc, sheet, K_hand, D_hand)
        f = SH.multi_tag_pnp(fc, sheet, K_front, None)
        samples.append(CC.ChainSample("v%02d" % i, matrix_m_to_pose_fr5(T_ab2ee), h.T_cam2W, f.T_cam2W, 0.0,
                                      hand_corners=hc, front_corners=fc, hand_rms_px=h.rms_px, front_rms_px=f.rms_px))
    return samples


def report(res):
    print("    raw: %.2f mm / %.3f deg; hand %.2f mm / %.3f deg; base %.2f mm / %.3f deg; joint %.2f mm / %.3f deg" % (
        res['raw'].rms_pos_m * 1e3, res['raw'].rms_rot_deg, res['hand'].rms_pos_m * 1e3, res['hand'].rms_rot_deg,
        res['base'].rms_pos_m * 1e3, res['base'].rms_rot_deg, res['joint'].rms_pos_m * 1e3, res['joint'].rms_rot_deg))


samples = make_session(np.eye(4), np.eye(4), seed=1)
check("the layout gives %d usable views with >= 2 grid tags each" % len(samples), len(samples) >= 16,
      "hand tags per view: %s" % sorted({len(s.hand_corners) for s in samples}))
check("front_cam sees the 300/301 pair in every view", all(sorted(s.front_corners) == [300, 301] for s in samples))
ok, lines = coverage(samples, sheet=sheet)
check("coverage reads the sheet views as ready", ok, lines[0])
no_spin = [s for s in samples if abs(sample_view(s, sheet).spin_deg) < 5]
ok2, lines2 = coverage(no_spin, sheet=sheet)
check("tilts toward all four directions WITHOUT any spin are enough (spin is not demanded)", ok2 and len(no_spin) >= 8, lines2[0])
res_ns = CC.fit_corrections(no_spin, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb, jackknife=False)
v = sample_view(samples[0], sheet)
check("sample_view: straight-down view at 0.40 m over the grid", v.tilt_deg < 1.0 and abs(v.range_m - 0.40) < 0.01,
      "tilt %.2f range %.3f offset %.0f mm" % (v.tilt_deg, v.range_m, v.xy_offset_mm))
res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb)
report(res)
check("no planted error: raw residual at the noise floor (< 1.5 mm / 0.15 deg)", res['raw'].rms_pos_m < 1.5e-3 and res['raw'].rms_rot_deg < 0.15)
dD = CC.pose_error(res['joint'].D, np.eye(4)); dF = CC.pose_error(res['joint'].F, np.eye(4))
check("joint fit invents < 1.5 mm / 0.15 deg", dD[0] < 1.5e-3 and dD[1] < 0.15 and dF[0] < 1.5e-3 and dF[1] < 0.15,
      "D %.2f mm / %.3f deg, F %.2f mm / %.3f deg" % (dD[0] * 1e3, dD[1], dF[0] * 1e3, dF[1]))
errs = CC.pair_errors(samples, sheet, H_file, T_ab2mb, T_mb2fc, lift_compensate=compensate_T_ab2mb)
(pm, ps, pr, p95), (rm, rs, rr, r95) = CC.summarise_errors(errs)
check("T_A2B metric (hand tag -> front tag) at the noise floor, pairs named", pr < 1.5 and rr < 0.15 and all(e[2] in (300, 301) for e in errs),
      "rms %.2f mm / %.3f deg, e.g. %s" % (pr, rr, [(e[0], e[1], e[2]) for e in errs[:3]]))

print("  -- hand-eye wrong by 2 deg / 15 mm --")
D_true = small_T([1.2, -1.5, 0.8], [10.0, -8.0, 6.0])
samples = make_session(D_true, np.eye(4), seed=2)
res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb)
report(res)
no_spin = [s for s in samples if abs(sample_view(s, sheet).spin_deg) < 5]
res_ns = CC.fit_corrections(no_spin, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb, jackknife=False)
e_ns = CC.pose_error(res_ns['hand'].D, D_true)
check("… and the %d spin-free views alone still recover D (1 mm / 0.1 deg) with the right attribution" % len(no_spin),
      e_ns[0] < 1e-3 and e_ns[1] < 0.1 and res_ns['base'].rms_pos_m > 3 * res_ns['hand'].rms_pos_m,
      "%.2f mm / %.3f deg; hand %.2f vs base %.2f mm" % (e_ns[0] * 1e3, e_ns[1], res_ns['hand'].rms_pos_m * 1e3, res_ns['base'].rms_pos_m * 1e3))
check("raw shows it (> 5 mm)", res['raw'].rms_pos_m > 5e-3)
check("hand fit at the floor, base fit not (attribution)", res['hand'].rms_pos_m < 1.5e-3 and res['base'].rms_pos_m > 3 * res['hand'].rms_pos_m,
      "hand %.2f vs base %.2f mm" % (res['hand'].rms_pos_m * 1e3, res['base'].rms_pos_m * 1e3))
e = CC.pose_error(res['hand'].D, D_true)
check("D recovered within 1 mm / 0.1 deg", e[0] < 1e-3 and e[1] < 0.1, "%.2f mm / %.3f deg" % (e[0] * 1e3, e[1]))
errs_raw = CC.pair_errors(samples, sheet, H_file, T_ab2mb, T_mb2fc, lift_compensate=compensate_T_ab2mb)
errs_fix = CC.pair_errors(samples, sheet, H_file, T_ab2mb, T_mb2fc, res['hand'].D, res['hand'].F, compensate_T_ab2mb)
(_, _, pr0, _), _ = CC.summarise_errors(errs_raw); (_, _, pr1, _), (_, _, rr1, _) = CC.summarise_errors(errs_fix)
check("the user's T_A2B error drops from %.1f mm to the floor with the correction" % pr0, pr0 > 5 and pr1 < 1.5 and rr1 < 0.15,
      "corrected rms %.2f mm / %.3f deg" % (pr1, rr1))
# hold-out: fit on 3/4 of the views, evaluate the rest
train = [s for i, s in enumerate(samples) if i % 4 != 3]; test = [s for i, s in enumerate(samples) if i % 4 == 3]
res_t = CC.fit_corrections(train, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb, jackknife=False)
ho = CC.evaluate(res_t['hand'].D, res_t['hand'].F, test, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb)
check("held-out views (%d) fit as well as the training ones (no over-fit)" % len(test),
      ho.rms_pos_m < 1.5 * max(res_t['hand'].rms_pos_m, 1e-3), "hold-out %.2f mm vs train %.2f mm" % (ho.rms_pos_m * 1e3, res_t['hand'].rms_pos_m * 1e3))

print("  -- arm mount wrong by 1 deg yaw / 0.5 deg tilt / 8 mm --")
F_true = small_T([0.5, -0.3, 1.0], [-8.0, 5.0, 3.0])
samples = make_session(np.eye(4), F_true, seed=3)
res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb)
report(res)
check("base fit at the floor, hand fit not", res['base'].rms_pos_m < 1.5e-3 and res['hand'].rms_pos_m > 3 * res['base'].rms_pos_m,
      "base %.2f vs hand %.2f mm" % (res['base'].rms_pos_m * 1e3, res['hand'].rms_pos_m * 1e3))
e = CC.pose_error(res['base'].F, F_true)
check("F recovered within 1 mm / 0.1 deg", e[0] < 1e-3 and e[1] < 0.1, "%.2f mm / %.3f deg" % (e[0] * 1e3, e[1]))

print("  -- both --")
samples = make_session(D_true, F_true, seed=4)
res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb)
report(res)
eD = CC.pose_error(res['joint'].D, D_true); eF = CC.pose_error(res['joint'].F, F_true)
check("joint fit recovers both (1.5 mm / 0.15 deg)", eD[0] < 1.5e-3 and eD[1] < 0.15 and eF[0] < 1.5e-3 and eF[1] < 0.15,
      "D %.2f mm / %.3f deg, F %.2f mm / %.3f deg" % (eD[0] * 1e3, eD[1], eF[0] * 1e3, eF[1]))
check("jackknife sd small (t < 1.5 mm)", res['joint'].jackknife_D_sd[3:].max() < 1.5e-3 and res['joint'].jackknife_F_sd[3:].max() < 1.5e-3,
      "D %s F %s mm" % (np.round(res['joint'].jackknife_D_sd[3:] * 1e3, 2), np.round(res['joint'].jackknife_F_sd[3:] * 1e3, 2)))
views = [sample_view(s, sheet) for s in samples]
corr = CC.residual_correlations(res['joint'], views)
check("residual_correlations returns finite r for range / tilt / spin", all(np.isfinite(list(corr.values()))), "%s" % {k: round(v, 2) for k, v in corr.items()})

print("  -- a wrong print scale looks like a chain error; re-solving the stored corners fixes it --")
sheet_s = SH.load_sheet(LAYOUT, sx=1.003, sy=0.998, tag_size_m=0.0897)          # the print, as measured
# render the session against the SCALED sheet (the truth), but solve it against the design sheet first
sheet_design, sheet = sheet, sheet_s
samples = make_session(np.eye(4), np.eye(4), seed=5)
sheet = sheet_design
for s in samples:            # re-solve the stored corners with the design sheet — what `capture` without --sx would do
    s.T_hc2W = SH.multi_tag_pnp(s.hand_corners, sheet_design, K_hand, D_hand).T_cam2W
    s.T_fc2W = SH.multi_tag_pnp(s.front_corners, sheet_design, K_front, None).T_cam2W
res_d = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb, jackknife=False)
for s in samples:            # `solve --sx 1.003 --sy 0.998 --tag-size 0.0897` re-solves from corners.json
    s.T_hc2W = SH.multi_tag_pnp(s.hand_corners, sheet_s, K_hand, D_hand).T_cam2W
    s.T_fc2W = SH.multi_tag_pnp(s.front_corners, sheet_s, K_front, None).T_cam2W
res_s = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, compensate_T_ab2mb, jackknife=False)
check("design scale on a 0.3 %% print: the raw chain reads a spurious %.1f mm; measured scale: %.2f mm" % (res_d['raw'].rms_pos_m * 1e3, res_s['raw'].rms_pos_m * 1e3),
      res_d['raw'].rms_pos_m > 2 * res_s['raw'].rms_pos_m and res_s['raw'].rms_pos_m < 1.5e-3)

print("  -- persistence --")
d = tempfile.mkdtemp()
try:
    meta = dict(mode="sheet", sheet_json=LAYOUT, sheet_sx=1.0, sheet_sy=1.0, sheet_tag_size_m=0.09,
                K_hand=K_hand.ravel().tolist(), D_hand=D_hand.tolist(), K_front=K_front.ravel().tolist(), D_front=[],
                front_cam_frame="level")
    # a 2026-09-15 two-tag session (no corners, T_hc2A / T_fc2B keys) must be refused, not misread
    old = os.path.join(d, "old"); os.makedirs(old)
    np.savez(os.path.join(old, "samples.npz"), labels=np.array(["v00"]), tcp=np.zeros((1, 6)), lift=np.zeros(1),
             T_hc2A=np.eye(4)[None], T_fc2B=np.eye(4)[None])
    open(os.path.join(old, "meta.yaml"), "w").write("hand_tag: 150\nfront_tag: 149\nspacing_m: 1.01\n")
    try:
        load_samples(old); check("an old two-tag session is refused", False)
    except ValueError as exc:
        check("an old two-tag session is refused", "two-tag" in str(exc))
    save_samples(d, samples[:5], meta)
    s2, m2 = load_samples(d)
    check("save / load round trip keeps the corners and the PnP rms",
          len(s2) == 5 and m2["mode"] == "sheet" and os.path.exists(os.path.join(d, "corners.json"))
          and all(np.allclose(a.hand_corners[k], b.hand_corners[k]) for a, b in zip(samples[:5], s2) for k in a.hand_corners)
          and sorted(s2[0].front_corners) == [300, 301] and abs(s2[0].hand_rms_px - samples[0].hand_rms_px) < 1e-9)
    import importlib.util
    import types
    spec = importlib.util.spec_from_file_location("chain_calib_tool", os.path.join(_HERE, "chain_calib.py"))
    tool = importlib.util.module_from_spec(spec); spec.loader.exec_module(tool)
    args = types.SimpleNamespace(sheet_json=None, sx=1.003, sy=0.998, tag_size=0.0897)
    sh2 = tool.sheet_from_args(args, m2)
    n = tool.resolve_samples(s2, m2, sh2)
    check("resolve_samples re-solves every sample at the requested scale", n == 5 and
          all(CC.pose_error(a.T_hc2W, b.T_hc2W)[0] < 1e-6 for a, b in zip(samples[:5], s2)))
finally:
    shutil.rmtree(d)

# ----------------------------------------------------------------------
print("\n== 5. arm joint-offset fit (arm_offsets.py): planted zero offsets recovered from lens poses ==")
# The arm error left after the chain calibration is configuration-dependent
# (2026-09-21 spin test), i.e. kinematic. Plant J2..J6 offsets, generate lens
# poses hand_cam would measure against the sheet at 36 configurations with
# 1.5 mm / 0.4 deg of noise, and fit them back. J1 stays 0 (degenerate with
# the base yaw the chain holds). Also: joint angles round-trip through
# samples.npz and an old session without them loads with joints_deg None.
import subprocess, tempfile
from chain_calib.arm_fk import ArmChain
from chain_calib.session import save_samples as _save
from chain_calib.solver import ChainSample as _CS
_ch = ArmChain()
_home = [-90.00087, -90.00022, 90.00348, -89.99956, -90.00044, 0.00044]
check("URDF FK reproduces the controller's home TCP (-158.99, 699.99, 773.96) mm",
      np.linalg.norm(_ch.fk_flange(_home)[:3, 3] * 1e3 - [-158.9886, 699.9907, 773.9619]) < 0.1)
_rng = np.random.default_rng(3)
_dq_true = np.array([0.0, 0.8, -0.6, 1.2, -0.9, 0.5])
_T_fc2W = np.array([[0, -1, 0, 0.002], [1, 0, 0, 0.006], [0, 0, 1, 0.303], [0, 0, 0, 1.0]])
_T_ab2W_true = T_ab2mb @ T_mb2fc @ _T_fc2W
_seeds = [[-100, -70, 110, -130, -90, 10], [-60, -80, 120, -125, -90, 40], [-120, -60, 100, -130, -95, -20], [-80, -90, 130, -125, -85, 60]]
_smp = []
for i in range(36):
    q = np.array(_seeds[i % 4], float) + _rng.uniform(-15, 15, 6)
    T_W2hc = invert_T(_T_ab2W_true) @ _ch.fk_flange(q, _dq_true) @ invert_T(H_file)
    nz = np.eye(4); nz[:3, 3] = _rng.normal(0, 0.0015, 3); nz[:3, :3] = CC._R_from_rotvec(_rng.normal(0, math.radians(0.4), 3))
    _smp.append(_CS("s%02d" % i, list(matrix_m_to_pose_fr5(_ch.fk_flange(q))), invert_T(T_W2hc @ nz), _T_fc2W, 0.0, joints_deg=list(q)))
_d = tempfile.mkdtemp(prefix="chk_armoff_")
try:
    _save(_d, _smp, dict(mode="sheet", date="synthetic", sheet_sx=1.0, sheet_sy=1.0, sheet_tag_size_m=0.09, front_cam_frame="level"))
    from chain_calib.session import load_samples as _load
    _s2, _ = _load(_d)
    check("joint angles round-trip through samples.npz", all(np.allclose(a.joints_deg, b.joints_deg) for a, b in zip(_smp, _s2)))
    _r = subprocess.run([sys.executable, os.path.join(_HERE, "arm_offsets.py"), _d, "--holdout-every", "4"], capture_output=True, text=True)
    _dq = np.load(os.path.join(_d, "arm_offsets.npz"))["dq_deg"] if os.path.exists(os.path.join(_d, "arm_offsets.npz")) else None
    check("arm_offsets.py runs on the synthetic session", _r.returncode == 0 and _dq is not None, (_r.stderr or "")[-200:])
    if _dq is not None:
        err = np.abs(_dq - _dq_true)
        check("planted J2..J6 offsets recovered within 0.3 deg each", err[1:].max() < 0.3,
              "fitted %s vs planted %s" % (np.round(_dq, 2), _dq_true))
        check("J1 stays exactly 0 (degenerate with the chain's base yaw)", _dq[0] == 0.0)
    check("a session recorded before joint angles existed loads with joints_deg None",
          all(x.joints_deg is None for x in _load(os.path.join(_HERE, "..", "..", "..", "log", "chain_calib", "20260921"))[0])
          if os.path.isdir(os.path.join(_HERE, "..", "..", "..", "log", "chain_calib", "20260921")) else True)
finally:
    shutil.rmtree(_d)

print("\n%d checks, %d failed" % (_n[0], _bad[0]))
sys.exit(1 if _bad[0] else 0)
