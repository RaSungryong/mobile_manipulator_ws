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

    T_W2hc_pred_i = inv(T_ab2W) . FK(q_i + dq) . inv(T_hc2ee)        URDF FK, hand-eye as calibrated
    residual_i    = project(K_hand, D_hand, inv(T_W2hc_pred_i) . corners_W) - corners_px   [px]  (default)
                  | pose_error(T_W2hc_pred_i, inv(T_hc2W_i))  translation [m] + w_rot * rot [rad]  (--residual pose)

The REPROJECTION residual is the default since the first real session
(20260921_arm, 65 views): most views hold only two sheet tags, and a
two-tag PnP has a near-planar ambiguity — a false 3-7 deg tilt with a
matching 30-110 mm depth error — so a fit on the PnP POSES was fitting
that noise (rigid 21 mm rms, offsets no better on hold-out). Fitting the
corner pixels instead lets a two-tag view constrain exactly what its
corners constrain (position and spin well, tilt weakly) and the six-tag
views carry the tilt. 1 px at 0.45 m with fx 609 is ~0.75 mm.

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

import cv2
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
    def __init__(self, chain, samples, H, T_ab2W0, fit_links, w_rot, sheet=None, K=None, D=None, residual="reproj"):
        self.chain, self.H, self.w_rot, self.fit_links = chain, H, w_rot, fit_links
        self.Hinv = invert_T(H)
        self.q = np.array([s.joints_deg for s in samples], float)
        self.meas = [invert_T(np.asarray(s.T_hc2W, float)) for s in samples]     # T_W2hc measured (PnP)
        self.T_ab2W0 = T_ab2W0
        self.residual_kind = residual
        if residual == "reproj":
            if sheet is None or K is None or not all(s.hand_corners for s in samples):
                raise ValueError("reprojection residual needs the sheet, K_hand and stored corners (use --residual pose)")
            self.K = np.asarray(K, float).reshape(3, 3)
            self.D = np.zeros(5) if D is None else np.asarray(D, float).ravel()
            self.obj, self.img = [], []
            for s in samples:
                ids = sorted(s.hand_corners)
                self.obj.append(np.vstack([sheet.corners_W(k) for k in ids]))                 # (4n, 3) in W
                self.img.append(np.vstack([np.asarray(s.hand_corners[k], float).reshape(4, 2) for k in ids]))

    fit_hand_eye = False        # --hand-eye-free: also correct T_hc2ee (6), see unpack

    # parameter vector: [dq2..dq6 (deg)] [+ scale2, scale3] [+ hand-eye 6] [+ base pose 6: rotvec(3), t(3)]
    def unpack(self, p):
        dq = np.zeros(6); dq[1:6] = p[0:5]
        if self.fit_hand_eye:
            dq[5] = 0.0             # a J6 offset IS a hand-eye rotation about the flange z: fixed when the hand-eye is free
        k = 5
        scale = np.ones(6)
        if self.fit_links:
            # the two arm links are the ORIGINS of j3 (0.700 m, upper arm) and j4 (0.586 m, forearm);
            # j2's origin (0.18 m along J1's axis) is a pure base shift and must not be freed
            scale[2], scale[3] = 1.0 + p[k], 1.0 + p[k + 1]; k += 2
        if self.fit_hand_eye:
            dH = np.eye(4); dH[:3, :3] = CC._R_from_rotvec(p[k:k + 3]); dH[:3, 3] = p[k + 3:k + 6]; k += 6
            self.Hinv = invert_T(self.H @ dH)          # corrected hand-eye for this evaluation
        rv, t = p[k:k + 3], p[k + 3:k + 6]
        dT = np.eye(4); dT[:3, :3] = CC._R_from_rotvec(rv); dT[:3, 3] = t
        return dq, scale, self.T_ab2W0 @ dT

    def hand_eye_of(self, p):
        self.unpack(p)
        return invert_T(self.Hinv)

    def n_params(self):
        return 5 + (2 if self.fit_links else 0) + (6 if self.fit_hand_eye else 0) + 6

    def _pred(self, i, dq, scale, T_W2ab):
        """T_W2hc predicted: the lens pose in W through FK + hand-eye."""
        return T_W2ab @ self.chain.fk_flange(self.q[i], dq, scale if self.fit_links else None) @ self.Hinv

    def reproj_px(self, i, T_W2hc):
        """Predicted pixels of sample i's sheet corners for a camera at T_W2hc, minus the observed ones."""
        T_hc2W = invert_T(T_W2hc)
        rvec, _ = cv2.Rodrigues(T_hc2W[:3, :3])
        uv, _ = cv2.projectPoints(self.obj[i].reshape(-1, 1, 3), rvec, T_hc2W[:3, 3].reshape(3, 1), self.K, self.D)
        return (uv.reshape(-1, 2) - self.img[i]).ravel()

    def residuals(self, p, idx=None):
        dq, scale, T_ab2W = self.unpack(p)
        T_W2ab = invert_T(T_ab2W)
        out = []
        for i in (range(len(self.q)) if idx is None else idx):
            pred = self._pred(i, dq, scale, T_W2ab)
            if self.residual_kind == "reproj":
                out.append(self.reproj_px(i, pred))
            else:
                E = invert_T(pred) @ self.meas[i]
                out.append(np.concatenate([E[:3, 3], self.w_rot * rotvec(E[:3, :3])]))
        return np.concatenate(out)

    def per_sample_px(self, p, idx=None):
        """rms reprojection error (px) per sample — the quantity the default fit minimises."""
        dq, scale, T_ab2W = self.unpack(p)
        T_W2ab = invert_T(T_ab2W)
        return np.array([np.sqrt((self.reproj_px(i, self._pred(i, dq, scale, T_W2ab)) ** 2).mean())
                         for i in (range(len(self.q)) if idx is None else idx)])

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
            out.append(pose_err(self._pred(i, dq, scale, T_W2ab), self.meas[i]))
        return np.array(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", help="chain_calib session directory (samples must carry joint angles)")
    ap.add_argument("--links", action="store_true", help="also fit the j2 / j3 link-length scales")
    ap.add_argument("--residual", choices=["reproj", "pose"], default="reproj",
                    help="reproj (default): corner pixels through the model; pose: hand_cam PnP pose (the 2026-09-21 first version)")
    ap.add_argument("--min-tags", type=int, default=0, help="drop samples whose hand_cam saw fewer sheet tags")
    ap.add_argument("--hand-eye-free", action="store_true",
                    help="also fit a 6-DOF correction of T_hc2ee (needs spins AND tilts in the data; reported, not applied)")
    ap.add_argument("--quick", action="store_true", help="skip the jackknife (65 refits)")
    ap.add_argument("--write-hand-eye", default=None, metavar="NPZ",
                    help="with --hand-eye-free: save the fitted T_hc2ee as a load_T_hc2ee npz (NOT installed as the config file)")
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
    if args.min_tags:
        n0 = len(samples)
        samples = [s for s in samples if len(s.hand_corners or []) >= args.min_tags]
        print("view filter: hand_cam >= %d tags -> %d of %d samples" % (args.min_tags, len(samples), n0))
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

    K_hand = np.asarray(meta["K_hand"], float).reshape(3, 3) if meta.get("K_hand") else None
    D_hand = np.asarray(meta.get("D_hand", [0.0] * 5), float)
    if args.residual == "reproj" and (K_hand is None or not all(s.hand_corners for s in samples)):
        print("! no stored corners / K_hand in this session — falling back to --residual pose")
        args.residual = "pose"
    prob = Problem(chain, samples, H, T_ab2W0, args.links, args.w_rot, sheet, K_hand, D_hand, args.residual)
    prob.fit_hand_eye = args.hand_eye_free
    print("residual: %s" % ("corner reprojection [px]" if args.residual == "reproj"
                             else "hand_cam PnP pose [m + %.2f * rad]" % args.w_rot))
    idx_all = list(range(len(samples)))
    test = [i for i in idx_all if args.holdout_every and (i % args.holdout_every == args.holdout_every - 1)]
    train = [i for i in idx_all if i not in test]

    p_base = prob.solve(train, fix_dq=True)      # base pose only: what a rigid-chain model can do
    p_fit = prob.solve(train)                    # base pose + joint offsets (+ links)

    def summ(p, idx, name):
        e = prob.per_sample(p, idx)
        px = ""
        if args.residual == "reproj":
            r = prob.per_sample_px(p, idx)
            px = "   reproj rms %5.2f px (max %5.2f)" % (np.sqrt((r ** 2).mean()), r.max())
        return "%-34s lens pose rms %5.2f mm / %.3f deg   max %5.2f mm / %.3f deg%s" % (
            name, np.sqrt((e[:, 0] ** 2).mean()) * 1e3, np.sqrt((e[:, 1] ** 2).mean()), e[:, 0].max() * 1e3, e[:, 1].max(), px)
    print("\n== residual: FK through the chain vs hand_cam (lens pose = against the per-view PnP, which is itself "
          "noisy on 2-tag views; the px column is what the fit minimises) ==")
    print(summ(p_base, train, "base pose free, offsets 0 (rigid)"))
    print(summ(p_fit, train, "base pose + joint offsets%s%s" % (" + links" if args.links else "",
                                                                   " + hand-eye" if args.hand_eye_free else "")))
    if test:
        print(summ(p_base, test, "  hold-out, rigid"))
        print(summ(p_fit, test, "  hold-out, offsets"))

    dq, scale, T_ab2W = prob.unpack(p_fit)
    print("\n== joint zero offsets (deg, ADDED to the reading = physical angle) ==")
    # jackknife
    sd = np.full(6, float("nan"))
    if not args.quick:
        jk = []
        for k in train:
            jk.append(prob.unpack(prob.solve([i for i in train if i != k]))[0])
        jk = np.array(jk); n = len(train)
        sd = np.sqrt((n - 1) / n * ((jk - jk.mean(0)) ** 2).sum(0))
    for j in range(6):
        tag = "  (fixed: degenerate with the base yaw the chain already holds)" if j == 0 else (
            "  (fixed: degenerate with the free hand-eye's spin)" if (j == 5 and args.hand_eye_free) else (
            "" if args.quick else "  jackknife sd %.3f" % sd[j]))
        print("  J%d  %+7.3f%s" % (j + 1, dq[j], tag))
    if args.hand_eye_free:
        Hf = prob.hand_eye_of(p_fit)
        dp, da = pose_err(H, Hf)
        print("  hand-eye correction: %.1f mm / %.2f deg from the calibrated file  -> " % (dp * 1e3, da)
              + CC.describe_T(Hf, "fitted T_hc2ee"))
        print("    delta t (hc frame) %s mm" % np.array2string((Hf[:3, 3] - H[:3, 3]) * 1e3, precision=1))
        if args.write_hand_eye:
            np.savez(args.write_hand_eye, Hf)
            print("    fitted hand-eye saved -> %s (a candidate; the chain's T_ab2mb was fitted with the OLD hand-eye "
                  "and must be re-solved before either is applied)" % args.write_hand_eye)
    if args.links:
        print("  link scales upper arm %.5f  forearm %.5f  (x 0.700 / 0.586 m -> %+.1f / %+.1f mm)"
              % (scale[2], scale[3], (scale[2] - 1) * 700, (scale[3] - 1) * 586))
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

    print("\nper-sample residual, rigid -> offsets  (lens pose mm / deg%s; hand tags):"
          % ("; reproj px" if args.residual == "reproj" else ""))
    e0 = prob.per_sample(p_base, idx_all); e1 = prob.per_sample(p_fit, idx_all)
    r0 = prob.per_sample_px(p_base, idx_all) if args.residual == "reproj" else None
    r1 = prob.per_sample_px(p_fit, idx_all) if args.residual == "reproj" else None
    for i, s in enumerate(samples):
        flag = " (hold-out)" if i in test else ""
        px = ("  %5.2f -> %5.2f px" % (r0[i], r1[i])) if r0 is not None else ""
        print("  %-5s %5.1f / %.2f  ->  %5.1f / %.2f%s  %d tags%s" % (
            s.label, e0[i, 0] * 1e3, e0[i, 1], e1[i, 0] * 1e3, e1[i, 1], px, len(s.hand_corners or []), flag))
    out = os.path.join(args.dir, "arm_offsets.npz")
    np.savez(out, dq_deg=dq, link_scale=scale, T_ab2W_fit=T_ab2W, T_ab2W_chain=T_ab2W0,
             labels=np.array([s.label for s in samples]), holdout=np.array([samples[i].label for i in test]))
    print("\nsaved -> %s   (NOT applied anywhere — see the record for how offsets enter a command)" % out)


if __name__ == "__main__":
    main()
