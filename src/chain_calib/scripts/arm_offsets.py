#!/usr/bin/env python3
"""
arm_offsets.py — the ARM's own error from the sheet: fit joint zero offsets
(and optionally link-length scales) of the FR10v6 to the lens poses hand_cam
measured against the A0 sheet.

    rosrun chain_calib arm_offsets.py log/chain_calib/<session>                 # offsets J2..J6
    rosrun chain_calib arm_offsets.py log/chain_calib/<session> --links         # + link scales
    rosrun chain_calib arm_offsets.py log/chain_calib/<session> --holdout-every 4

Why (2026-09-21): with the chain calibrated (constant error ~1 mm, invariant
to re-parking and wrist spin), the remaining +-15-20 mm is the ARM's — the
lens lands at different places for the same commanded point depending on
the arm configuration (sheet_path spin test), so it is not a rigid mount
tilt but kinematic: joint zeros / link geometry. The sheet gives the lens
pose per view against a rigid ground truth; the FK gives the model's; the
difference over many configurations pins the offsets.

Model, per sample i (joints q_i recorded with the capture):

    T_W2hc_meas_i = inv(T_hc2W_i)                                   hand_cam multi-tag PnP
    T_W2hc_pred_i = inv(T_ab2W) . FK(q_i + dq) . inv(T_hc2ee)        URDF FK, hand-eye as calibrated
    residual_i    = pose_error(pred, meas)                           translation [m] + w_rot * rotation [rad]

Unknowns: dq (J2..J6; J1 is fixed at 0 — a J1 offset IS a yaw of the base
frame and the chain calibration already absorbed it) and the sheet pose in
the arm base T_ab2W (6, initialised from the calibrated chain and front_cam's
view of tag 200, left free so the fit does not inherit the chain's residual).
Hand-eye stays fixed. ``--links`` also frees the j2 / j3 origin lengths.

What it reports: the URDF FK vs the controller's own TCP per sample (must
agree to ~1 mm or the URDF is not the controller's model), the residual
with dq = 0 vs fitted, per-sample residuals, the residual's correlation
with the flange x (the gradient that motivated this), jackknife sd of dq,
hold-out, and the fitted T_ab2W against the chain's. It APPLIES nothing —
the offsets go into how targets are commanded (see the record) and that is
a separate step.

Needs samples with joint angles (captures from 2026-09-21 on). Samples
without them are skipped with a count.
"""
import argparse
import math
import os
import sys

import numpy as np
from scipy.optimize import least_squares

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))     # NOT _HERE: scripts/chain_calib.py would shadow the package

from chain_calib import solver as CC                                   # noqa: E402
from chain_calib.arm_fk import ArmChain                                # noqa: E402
from chain_calib.session import load_samples                           # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                  # noqa: E402
from path_tag_locator.geometry import invert_T, pose_fr5_to_matrix_m, rot2rpy_deg  # noqa: E402

import importlib.util                                                  # noqa: E402
_spec = importlib.util.spec_from_file_location("chain_calib_tool", os.path.join(_HERE, "chain_calib.py"))
tool = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(tool)   # platform(), sheet_from_args, resolve_samples


def rotvec(Rm):
    ang = math.acos(max(-1.0, min(1.0, (np.trace(Rm) - 1.0) / 2.0)))
    if ang < 1e-12:
        return np.zeros(3)
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]]) / (2.0 * math.sin(ang))
    return v * ang


def pose_err(Ta, Tb):
    """(translation error m, rotation error deg) between two poses."""
    E = invert_T(Ta) @ Tb
    return float(np.linalg.norm(E[:3, 3])), math.degrees(np.linalg.norm(rotvec(E[:3, :3])))


class Problem:
    def __init__(self, chain, samples, H, T_ab2W0, fit_links, w_rot):
        self.chain, self.H, self.w_rot, self.fit_links = chain, H, w_rot, fit_links
        self.Hinv = invert_T(H)
        self.q = np.array([s.joints_deg for s in samples], float)
        self.meas = [invert_T(np.asarray(s.T_hc2W, float)) for s in samples]     # T_W2hc measured
        self.T_ab2W0 = T_ab2W0

    # parameter vector: [dq2..dq6 (deg)] [+ scale2, scale3] [+ base pose 6: rotvec(3), t(3)]
    def unpack(self, p):
        dq = np.zeros(6); dq[1:6] = p[0:5]
        k = 5
        scale = np.ones(6)
        if self.fit_links:
            scale[1], scale[2] = 1.0 + p[k], 1.0 + p[k + 1]; k += 2
        rv, t = p[k:k + 3], p[k + 3:k + 6]
        dT = np.eye(4); dT[:3, :3] = CC._R_from_rotvec(rv); dT[:3, 3] = t
        return dq, scale, self.T_ab2W0 @ dT

    def n_params(self):
        return 5 + (2 if self.fit_links else 0) + 6

    def residuals(self, p, idx=None):
        dq, scale, T_ab2W = self.unpack(p)
        T_W2ab = invert_T(T_ab2W)
        out = []
        for i in (range(len(self.q)) if idx is None else idx):
            pred = T_W2ab @ self.chain.fk_flange(self.q[i], dq, scale if self.fit_links else None) @ self.Hinv
            E = invert_T(pred) @ self.meas[i]
            out.append(np.concatenate([E[:3, 3], self.w_rot * rotvec(E[:3, :3])]))
        return np.concatenate(out)

    def solve(self, idx=None, fix_dq=False):
        p0 = np.zeros(self.n_params())
        if fix_dq:
            n_free = self.n_params() - 6
            f = lambda pb: self.residuals(np.concatenate([np.zeros(n_free), pb]), idx)
            r = least_squares(f, np.zeros(6), method="lm")
            return np.concatenate([np.zeros(n_free), r.x])
        r = least_squares(lambda pp: self.residuals(pp, idx), p0, method="lm")
        return r.x

    def per_sample(self, p, idx=None):
        dq, scale, T_ab2W = self.unpack(p)
        T_W2ab = invert_T(T_ab2W)
        out = []
        for i in (range(len(self.q)) if idx is None else idx):
            pred = T_W2ab @ self.chain.fk_flange(self.q[i], dq, scale if self.fit_links else None) @ self.Hinv
            out.append(pose_err(pred, self.meas[i]))
        return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", help="chain_calib session directory (samples must carry joint angles)")
    ap.add_argument("--links", action="store_true", help="also fit the j2 / j3 link-length scales")
    ap.add_argument("--w-rot", type=float, default=0.1,
                    help="metres per radian of rotation residual (default 0.1: hand_cam's per-view orientation is the "
                         "noisiest measurement, so weight it lightly and let the lens POSITION drive the fit)")
    ap.add_argument("--holdout-every", type=int, default=0, help="hold every k-th sample out of the fit, evaluate after")
    ap.add_argument("--exclude", nargs="*", default=None)
    ap.add_argument("--urdf", default=None)
    ap.add_argument("--hand-eye", default=None)
    ap.add_argument("--sheet-json", default=None); ap.add_argument("--sx", type=float, default=None)
    ap.add_argument("--sy", type=float, default=None); ap.add_argument("--tag-size", type=float, default=None)
    ap.add_argument("--front-rotation", choices=["level", "measured"], default="level")
    args = ap.parse_args()

    samples, meta = load_samples(args.dir)
    if not samples:
        sys.exit("no samples in %s" % args.dir)
    cfg, ext, H = tool.platform(hand_eye=args.hand_eye or meta.get("hand_eye_npz"))
    sheet = tool.sheet_from_args(args, meta)
    if all(s.hand_corners for s in samples) and meta.get("K_hand"):
        tool.resolve_samples(samples, meta, sheet, args.front_rotation)      # PnP at the sheet scale given
    else:
        print("(no stored corners — using the T_hc2W / T_fc2W as saved)")
    if args.exclude:
        samples = [s for s in samples if s.label not in set(args.exclude)]
    n_all = len(samples)
    samples = [s for s in samples if s.joints_deg is not None]
    print("%d samples, %d with joint angles%s" % (n_all, len(samples),
          "" if len(samples) == n_all else " (the rest predate 2026-09-21 and are skipped)"))
    if len(samples) < 12:
        sys.exit("need at least 12 samples with joint angles (have %d)" % len(samples))
    chain = ArmChain(args.urdf)
    print("URDF: %s\nhand-eye: %s" % (chain.path, tool._resolve(args.hand_eye or meta.get("hand_eye_npz"))))

    # 0. the URDF is the controller's model?  FK(q_i) vs the TCP the controller reported
    dfk = np.array([pose_err(chain.fk_flange(s.joints_deg), pose_fr5_to_matrix_m(s.tcp_pose_mm_deg)) for s in samples])
    print("\nURDF FK(q) vs the controller's TCP over the samples: %.2f mm / %.3f deg rms, max %.2f mm / %.3f deg"
          % (np.sqrt((dfk[:, 0] ** 2).mean()) * 1e3, np.sqrt((dfk[:, 1] ** 2).mean()), dfk[:, 0].max() * 1e3, dfk[:, 1].max()))
    if dfk[:, 0].max() > 0.005:
        print("  ! the URDF does not reproduce the controller's own FK — the offsets below would be fitted to the wrong model")

    # 1. sheet pose in the arm base from the chain (front_cam's view, first sample's lift), as the starting point
    s0 = samples[0]
    T_ab2W0 = compensate_T_ab2mb(ext.T_ab2mb, s0.lift_height_m) @ ext.T_mb2fc_chain @ np.asarray(s0.T_fc2W, float)

    prob = Problem(chain, samples, H, T_ab2W0, args.links, args.w_rot)
    idx_all = list(range(len(samples)))
    test = [i for i in idx_all if args.holdout_every and (i % args.holdout_every == args.holdout_every - 1)]
    train = [i for i in idx_all if i not in test]

    p_base = prob.solve(train, fix_dq=True)      # base pose only: what a rigid-chain model can do
    p_fit = prob.solve(train)                    # base pose + joint offsets (+ links)

    def summ(p, idx, name):
        e = prob.per_sample(p, idx)
        return "%-34s rms %5.2f mm / %.3f deg   max %5.2f mm / %.3f deg" % (
            name, np.sqrt((e[:, 0] ** 2).mean()) * 1e3, np.sqrt((e[:, 1] ** 2).mean()), e[:, 0].max() * 1e3, e[:, 1].max())
    print("\n== lens pose residual (hand_cam PnP vs FK through the chain) ==")
    print(summ(p_base, train, "base pose free, offsets 0 (rigid)"))
    print(summ(p_fit, train, "base pose + joint offsets%s" % (" + links" if args.links else "")))
    if test:
        print(summ(p_base, test, "  hold-out, rigid"))
        print(summ(p_fit, test, "  hold-out, offsets"))

    dq, scale, T_ab2W = prob.unpack(p_fit)
    print("\n== joint zero offsets (deg, ADDED to the reading = physical angle) ==")
    # jackknife
    jk = []
    for k in train:
        jk.append(prob.unpack(prob.solve([i for i in train if i != k]))[0])
    jk = np.array(jk); n = len(train)
    sd = np.sqrt((n - 1) / n * ((jk - jk.mean(0)) ** 2).sum(0))
    for j in range(6):
        tag = "  (fixed: degenerate with the base yaw the chain already holds)" if j == 0 else "  jackknife sd %.3f" % sd[j]
        print("  J%d  %+7.3f%s" % (j + 1, dq[j], tag))
    if args.links:
        print("  link scales j2 %.5f  j3 %.5f  (x 0.700 / 0.586 m -> %+.1f / %+.1f mm)"
              % (scale[1], scale[2], (scale[1] - 1) * 700, (scale[2] - 1) * 586))
    print("  wrist offsets act through the %.0f mm hand-eye lever: 1 deg there = %.1f mm at the lens"
          % (np.linalg.norm(H[:3, 3]) * 1e3, np.linalg.norm(H[:3, 3]) * 1e3 * math.radians(1)))

    # the gradient that motivated this: residual vs flange x, before / after
    fx = np.array([pose_fr5_to_matrix_m(samples[i].tcp_pose_mm_deg)[0, 3] for i in train])
    for p, nm in ((p_base, "rigid"), (p_fit, "offsets")):
        dq_, sc_, Tw = prob.unpack(p); T_W2ab = invert_T(Tw)
        ex = []
        for i in train:
            pred = T_W2ab @ chain.fk_flange(prob.q[i], dq_, sc_ if args.links else None) @ prob.Hinv
            ex.append((prob.meas[i][:3, 3] - pred[:3, 3]) * 1e3)          # lens position error in W (mm)
        ex = np.array(ex)
        c = np.polyfit(fx, ex[:, 0], 1)
        print("  %-8s lens W-x error vs flange x: %+.1f mm per 600 mm (r = %+.2f)"
              % (nm, c[0] * 0.6, np.corrcoef(fx, ex[:, 0])[0, 1]))

    print("\n== the sheet in the arm base ==")
    print("  " + CC.describe_T(T_ab2W0, "from the chain (front_cam view)"))
    print("  " + CC.describe_T(T_ab2W, "fitted with the offsets       "))
    d, a = pose_err(T_ab2W0, T_ab2W)
    print("  difference %.1f mm / %.2f deg — how much the chain's placement of the sheet disagrees with the arm once "
          "its offsets are removed" % (d * 1e3, a))

    print("\nper-sample lens residual (mm / deg), rigid -> offsets:")
    e0 = prob.per_sample(p_base, idx_all); e1 = prob.per_sample(p_fit, idx_all)
    for i, s in enumerate(samples):
        flag = " (hold-out)" if i in test else ""
        print("  %-5s %5.1f / %.2f  ->  %5.1f / %.2f%s" % (s.label, e0[i, 0] * 1e3, e0[i, 1], e1[i, 0] * 1e3, e1[i, 1], flag))
    out = os.path.join(args.dir, "arm_offsets.npz")
    np.savez(out, dq_deg=dq, link_scale=scale, T_ab2W_fit=T_ab2W, T_ab2W_chain=T_ab2W0,
             labels=np.array([s.label for s in samples]), holdout=np.array([samples[i].label for i in test]))
    print("\nsaved -> %s   (NOT applied anywhere — see the record for how offsets enter a command)" % out)


if __name__ == "__main__":
    main()
