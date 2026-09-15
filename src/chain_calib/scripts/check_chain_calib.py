#!/usr/bin/env python3
"""Offline check of chain_calib.solver on a synthetic front_cam <-> hand_cam
chain with PLANTED errors: the true hand-eye differs from the file by a
known D, the true arm mount from T_ab2mb by a known F, the hand camera is
placed over tag A at the views the hand-eye sweep produces, and both tag
observations get pixel-level noise. The fit must recover D and F and the
residuals must say which side carries the error.

    python3 src/chain_calib/scripts/check_chain_calib.py
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "path_tag_locator", "src"))

from chain_calib import solver as CC                                 # noqa: E402
from chain_calib.session import coverage, describe_view, load_samples, save_samples  # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                # noqa: E402
from path_tag_locator.constants import load_extrinsics_full          # noqa: E402
from path_tag_locator.detections import _CORNER_ORDER, pose_from_corners  # noqa: E402
from path_tag_locator.geometry import invert_T, matrix_m_to_pose_fr5  # noqa: E402
from path_tag_locator.hand_eye import load_T_hc2ee                   # noqa: E402
from path_tag_locator.handeye_sweep import SweepCfg, plan_sweep       # noqa: E402

_n = [0]; _bad = [0]
def check(name, ok, detail=""):
    _n[0] += 1; _bad[0] += (not ok)
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")

CFG = os.path.join(_HERE, "..", "..", "path_tag_locator", "config")
ext = load_extrinsics_full(os.path.join(CFG, "extrinsics.yaml"))
T_ab2mb, T_mb2fc = ext.T_ab2mb, ext.T_mb2fc_level
H_file = load_T_hc2ee(os.path.join(CFG, "hand_eye", "T_hc2ee.npz"))
K_hand = np.array([[609.3, 0, 321.5], [0, 608.6, 238.7], [0, 0, 1.0]])
K_front = np.array([[910.0, 0, 642.7], [0, 910.0, 361.4], [0, 0, 1.0]])
TAG = 0.090
SPACING = 0.700


def render(T_cam2tag, K, rng, noise_px):
    """Tag corners imaged by a pinhole camera, with noise; returns
    T_cam2tag re-solved from those corners (what the tool does)."""
    obj = np.c_[_CORNER_ORDER * TAG / 2, np.zeros(4)]
    pc = (T_cam2tag[:3, :3] @ obj.T).T + T_cam2tag[:3, 3]
    uv = (pc / pc[:, 2:3]) @ K.T
    uv = uv[:, :2] + rng.normal(0, noise_px, (4, 2))
    return pose_from_corners(uv.ravel(), TAG, K)


# Per-corner noise of ONE frame is ~0.3 px; the tool averages 20 frames
# per sample (mean_detection), so the sample sees ~0.07 px. This matters:
# the OUT-OF-PLANE tilt of a 120 px tag is only good to ~0.7 deg per frame
# (edge-length difference 24 px per radian), and 0.7 deg over the 0.7 m
# A -> B lever is 8 mm — single frames would drown the fit.
FRAMES_PER_SAMPLE = 20


def make_world(D_true, F_true, seed, noise_px=0.3 / math.sqrt(FRAMES_PER_SAMPLE), n_views=18):
    """Synthetic session. Returns (samples, truths)."""
    rng = np.random.RandomState(seed)
    H_true = H_file @ D_true                        # the real hand-eye
    B_true = T_ab2mb @ T_mb2fc @ F_true             # the real ab -> fc
    # tag B on the floor 0.62 m ahead of the base centre (in front_cam's
    # view). Face-up AprilTag: its z points INTO the floor (down), x along
    # mb +y (the tag is laid a quarter turn round so that "A -> B along the
    # tag edge" puts A beside the robot, not behind it).
    T_mb2B = np.eye(4)
    T_mb2B[:3, :3] = CC.Rz(90.0) @ np.diag([1.0, -1.0, -1.0])
    T_mb2B[:3, 3] = [0.62, 0.0, 0.001]
    # the user lays A -> B along a straightedge: B is d along A's +x
    T_A2B_true = CC.truth_T_A2B(SPACING, 0, 0)
    T_mb2A = T_mb2B @ invert_T(T_A2B_true)              # A at mb (0.62, -0.70): the robot's right
    T_ab2A = T_ab2mb @ T_mb2A
    T_ab2fc_true = B_true                                # base-side truth (fc in ab)
    T_fc2B_true = invert_T(T_ab2fc_true) @ T_ab2mb @ T_mb2B
    # hand-cam views over tag A (the sweep's own planner, real hand-eye):
    # square view = tag z along the optical axis, 0.45 m away
    T_cam2tag_sq = np.eye(4); T_cam2tag_sq[2, 3] = 0.45
    T_ab2ee_start = T_ab2A @ invert_T(T_cam2tag_sq) @ H_true
    cfg = SweepCfg(distances_m=[0.40, 0.50], tilts_deg=[0.0, 12.0, 22.0], azimuths_deg=[0, 90, 180, 270],
                   spins_deg=[0.0, 90.0, 180.0, -90.0], max_samples=n_views,
                   max_xy_from_start_m=0.5, min_clearance_m=0.05)
    views, rejected = plan_sweep(T_ab2A, H_true, T_cam2tag_sq, T_ab2ee_start, cfg)
    samples = []
    for v in views:
        T_ab2ee = v.T_ab2ee
        T_hc2A_true = H_true @ invert_T(T_ab2ee) @ T_ab2A      # T_hc2A = T_hc2ee . T_ee2ab . T_ab2A
        assert T_hc2A_true[2, 3] > 0.2, "camera below the floor: frame convention wrong"
        T_hc2A = render(T_hc2A_true, K_hand, rng, noise_px)
        T_fc2B = render(T_fc2B_true, K_front, rng, noise_px)
        # the arm reports its FLANGE pose with a little noise-free FK
        samples.append(CC.ChainSample(v.label, matrix_m_to_pose_fr5(T_ab2ee), T_hc2A, T_fc2B, 0.0))
    return samples, dict(D=D_true, F=F_true, T_A2B=T_A2B_true, n_rejected=len(rejected))


def small_T(rot_deg_xyz, t_mm):
    return CC.T_from_vec6(np.concatenate([np.radians(rot_deg_xyz), np.asarray(t_mm) / 1e3]))


def report(truth, res):
    print("    raw: %.1f mm / %.2f deg; hand %.2f mm / %.3f deg; base %.2f mm / %.3f deg; joint %.2f mm / %.3f deg" % (
        res['raw'].rms_pos_m * 1e3, res['raw'].rms_rot_deg, res['hand'].rms_pos_m * 1e3, res['hand'].rms_rot_deg,
        res['base'].rms_pos_m * 1e3, res['base'].rms_rot_deg, res['joint'].rms_pos_m * 1e3, res['joint'].rms_rot_deg))


print("== 1. no planted error: nothing invented ==")
samples, tr = make_world(np.eye(4), np.eye(4), seed=1)
check("the sweep planner accepted a usable set of views", len(samples) >= 14, "%d views (%d rejected)" % (len(samples), tr['n_rejected']))
check("rotation diversity tens of degrees", CC.rotation_diversity_deg(samples) > 30, "%.0f deg" % CC.rotation_diversity_deg(samples))
truth, res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, SPACING, compensate_T_ab2mb)
report(truth, res)
check("truth snapped to k=0, a=0", truth.k90 == 0 and truth.a90 == 0)
# the noise floor: a 0.07 px corner noise is ~0.15 deg of hand-cam tilt per
# sample, times the 0.7 m A -> B lever = 2-3 mm of T_A2B per sample
check("raw chain residual at the noise floor (< 4 mm / 0.4 deg)", res['raw'].rms_pos_m < 4e-3 and res['raw'].rms_rot_deg < 0.4,
      "%.2f mm / %.3f deg" % (res['raw'].rms_pos_m * 1e3, res['raw'].rms_rot_deg))
dD = CC.pose_error(res['joint'].D, np.eye(4)); dF = CC.pose_error(res['joint'].F, np.eye(4))
check("joint fit invents < 3 mm / 0.3 deg on either side", dD[0] < 3e-3 and dD[1] < 0.3 and dF[0] < 3e-3 and dF[1] < 0.3,
      "D %.2f mm / %.3f deg, F %.2f mm / %.3f deg" % (dD[0] * 1e3, dD[1], dF[0] * 1e3, dF[1]))

print("\n== 2. hand-eye wrong by 2 deg / 15 mm (the 2026-09-11 suspect) ==")
D_true = small_T([1.2, -1.5, 0.8], [10.0, -8.0, 6.0])
samples, tr = make_world(D_true, np.eye(4), seed=2)
truth, res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, SPACING, compensate_T_ab2mb)
report(truth, res)
check("raw chain residual shows the error (> 5 mm or > 1 deg)", res['raw'].rms_pos_m > 5e-3 or res['raw'].rms_rot_deg > 1.0)
check("hand-only fit reaches the noise floor", res['hand'].rms_pos_m < 4e-3 and res['hand'].rms_rot_deg < 0.4)
check("base-only fit does NOT (attribution visible)", res['base'].rms_pos_m > 3 * res['hand'].rms_pos_m or res['base'].rms_rot_deg > 3 * res['hand'].rms_rot_deg,
      "base %.2f mm / %.3f deg vs hand %.2f mm / %.3f deg" % (res['base'].rms_pos_m * 1e3, res['base'].rms_rot_deg, res['hand'].rms_pos_m * 1e3, res['hand'].rms_rot_deg))
e = CC.pose_error(res['hand'].D, D_true)
check("D recovered within 2.5 mm / 0.3 deg", e[0] < 2.5e-3 and e[1] < 0.3, "%.2f mm / %.3f deg" % (e[0] * 1e3, e[1]))
e = CC.pose_error(res['joint'].D, D_true)
check("joint fit's D agrees too (F stays small)", e[0] < 3e-3 and e[1] < 0.4 and CC.pose_error(res['joint'].F, np.eye(4))[0] < 6e-3,
      "D %.2f mm / %.3f deg, F %.2f mm" % (e[0] * 1e3, e[1], CC.pose_error(res['joint'].F, np.eye(4))[0] * 1e3))
Hc = CC.corrected_hand_eye(H_file, res['hand'].D)
e = CC.pose_error(Hc, H_file @ D_true)
check("corrected hand-eye equals the true one (2.5 mm / 0.3 deg)", e[0] < 2.5e-3 and e[1] < 0.3)

print("\n== 3. arm mount wrong by 1 deg yaw / 0.5 deg tilt / 8 mm (the untested T_ab2mb assumption) ==")
F_true = small_T([0.5, -0.3, 1.0], [-8.0, 5.0, 3.0])
samples, tr = make_world(np.eye(4), F_true, seed=3)
truth, res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, SPACING, compensate_T_ab2mb)
report(truth, res)
check("base-only fit reaches the noise floor", res['base'].rms_pos_m < 4e-3 and res['base'].rms_rot_deg < 0.4)
check("hand-only fit does NOT", res['hand'].rms_pos_m > 3 * res['base'].rms_pos_m or res['hand'].rms_rot_deg > 3 * res['base'].rms_rot_deg,
      "hand %.2f mm / %.3f deg vs base %.2f mm / %.3f deg" % (res['hand'].rms_pos_m * 1e3, res['hand'].rms_rot_deg, res['base'].rms_pos_m * 1e3, res['base'].rms_rot_deg))
e = CC.pose_error(res['base'].F, F_true)
check("F recovered within 2.5 mm / 0.3 deg", e[0] < 2.5e-3 and e[1] < 0.3, "%.2f mm / %.3f deg" % (e[0] * 1e3, e[1]))
Tc = CC.corrected_T_ab2mb(T_ab2mb, T_mb2fc, res['base'].F)
e = CC.pose_error(Tc @ T_mb2fc, T_ab2mb @ T_mb2fc @ F_true)
check("folding F into T_ab2mb reproduces the true ab -> fc", e[0] < 2.5e-3 and e[1] < 0.3)

print("\n== 4. both sides wrong: only the joint fit fits ==")
samples, tr = make_world(D_true, F_true, seed=4, n_views=24)
truth, res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, SPACING, compensate_T_ab2mb)
report(truth, res)
check("joint fit at the noise floor, both single-side fits clearly worse",
      res['joint'].rms_pos_m < 4e-3 and res['hand'].rms_pos_m > 2 * res['joint'].rms_pos_m and res['base'].rms_pos_m > 2 * res['joint'].rms_pos_m)
eD = CC.pose_error(res['joint'].D, D_true); eF = CC.pose_error(res['joint'].F, F_true)
check("D and F both recovered (3 mm / 0.3 deg)", eD[0] < 3e-3 and eD[1] < 0.3 and eF[0] < 3e-3 and eF[1] < 0.3,
      "D %.2f mm / %.3f deg, F %.2f mm / %.3f deg" % (eD[0] * 1e3, eD[1], eF[0] * 1e3, eF[1]))
check("jackknife sd reported for the joint fit (translation sd < 4 mm)",
      res['joint'].jackknife_D_sd is not None and res['joint'].jackknife_D_sd[3:].max() < 4e-3 and res['joint'].jackknife_F_sd[3:].max() < 4e-3,
      "D sd t %s mm, F sd t %s mm" % (np.round(res['joint'].jackknife_D_sd[3:] * 1e3, 2), np.round(res['joint'].jackknife_F_sd[3:] * 1e3, 2)))

print("\n== 5. tags laid a quarter turn round / the other way: the truth snap follows ==")
samples, tr = make_world(np.eye(4), np.eye(4), seed=5)
# rotate tag B's laid orientation by 90 deg and put it at -x of A: re-express the samples' T_fc2B
T_alt = CC.truth_T_A2B(SPACING, 1, 2)
for s in samples:
    s.T_fc2B = invert_T(T_alt) @ tr['T_A2B'] @ s.T_fc2B if False else s.T_fc2B
# simpler: verify snap_truth on synthetic means directly
for k, a in ((1, 2), (3, 1), (2, 3)):
    T_mean = CC.truth_T_A2B(SPACING, k, a) @ small_T([0.5, -0.4, 1.5], [6, -4, 3])   # a few-degree chain error on top
    tc = CC.snap_truth(T_mean, SPACING)
    check("snap (%d, %d) with a 1.6 deg / 8 mm chain error on top" % (k, a), tc.k90 == k and tc.a90 == a, "-> (%d, %d)" % (tc.k90, tc.a90))

print("\n== 6. single-frame noise (no averaging): the fit still attributes but the numbers are 3x worse ==")
samples, tr = make_world(D_true, np.eye(4), seed=6, noise_px=0.3)
truth, res = CC.fit_corrections(samples, H_file, T_ab2mb, T_mb2fc, SPACING, compensate_T_ab2mb, jackknife=False)
report(truth, res)
e = CC.pose_error(res['hand'].D, D_true)
check("attribution still right with single frames", res['base'].rms_pos_m > 2 * res['hand'].rms_pos_m)
check("but D only within ~8 mm / 0.8 deg — hence the 20-frame mean per sample", e[0] < 10e-3 and e[1] < 1.0, "%.2f mm / %.3f deg" % (e[0] * 1e3, e[1]))

print("\n== 7. session persistence + coverage advice (the operator-jogged workflow) ==")
import tempfile, shutil
samples, tr = make_world(D_true, np.eye(4), seed=7, n_views=18)
d = tempfile.mkdtemp()
save_samples(d, samples[:3], dict(spacing_m=SPACING, hand_tag=150, front_tag=149))
s2, m2 = load_samples(d)
check("save / load round trip", len(s2) == 3 and m2["hand_tag"] == 150 and np.allclose(s2[1].T_hc2A, samples[1].T_hc2A))
ok, lines = coverage(samples[:3])
check("3 straight-ish samples: not ready, says what is missing", not ok and "still needed" in lines[1], lines[1])
ok, lines = coverage(samples)
check("the full grid: ready", ok, lines[0])
v = describe_view(samples[0].T_hc2A)
check("describe_view reads a square view as straight down at the planned range", v.tilt_deg < 2.0 and 0.35 < v.range_m < 0.55,
      "tilt %.1f range %.2f" % (v.tilt_deg, v.range_m))
tilted = [describe_view(s.T_hc2A) for s in samples if ' t22 ' in s.label]
check("22 deg views read as ~22 deg tilt", tilted and all(abs(x.tilt_deg - 22.0) < 1.0 for x in tilted),
      "%s" % [round(x.tilt_deg, 1) for x in tilted[:4]])
shutil.rmtree(d)

print("\n%d checks, %d failed" % (_n[0], _bad[0]))
sys.exit(1 if _bad[0] else 0)
