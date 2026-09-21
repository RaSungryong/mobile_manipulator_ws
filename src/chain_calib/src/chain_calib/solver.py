"""
solver.py
=========
Measure the error of the front_cam <-> hand_cam transform chain against
the printed A0 tag sheet (``sheet.py``) and fit a constant correction
(2026-09-15 user request: "T_fc2hc 사이의 tf들은 블랙박스 — 이상값과
실측값의 오차를 보정값으로"; 2026-09-18: the sheet replaced two hand-laid
tags as the ground truth, and the two-tag mode was removed).

The chain the locator uses (chain.py, T_X2Y = pose of Y in X):

    T_A2B = inv(T_hc2A) · T_hc2ee · T_ee2ab · T_ab2mb · T_mb2fc · T_fc2B
                          \\_____________ T_hc2fc(model) ______________/

Every tag of the sheet is a pure translation of the sheet frame W, so
each camera observes W directly (multi-tag PnP) and every arm pose i
gives a MEASURED T_hc2fc with no other truth needed:

    S_i = T_hc2W_i · inv(T_fc2W_i)

against the model M_i = H · A_i · B with H = T_hc2ee, A_i = T_ee2ab_i
(arm FK), B = T_ab2mb(lift) · T_mb2fc. The base does not move, so B and
T_fc2W are the same for every sample and the arm poses are the only
excitation — exactly the AX = YB (robot-world / hand-eye) problem:

    inv(H) · S_i = D · (A_i · B) · F

with D a constant correction on the HAND side (T_hc2ee' = H · D, i.e. the
hand-eye is wrong) and F a constant correction on the BASE side
(B' = B · F: the arm mount T_ab2mb, T_mb2fc, or front_cam's observation —
everything after the arm, indistinguishable from one base pose). A
single arm pose cannot tell D from F; a set of poses with rotation
diversity (tilts about two axes + spins) can, because D sits BEFORE the
varying A_i and F after it.

``fit_corrections`` fits three models and reports each one's residual:
hand only (F = I), base only (D = I), joint. Read the attribution off the
residuals: if the hand-only fit already reaches the noise floor, the
error is the hand-eye; if only the joint fit does, both sides carry
error; if the base-only fit is as good as the hand-only one the poses
did not have enough rotation diversity and the split is undetermined.
(The handoff document's `X_fixed` / `Y_fixed` / `XY` are hand / base /
joint.) ``evaluate`` applies a fitted D, F to held-out samples;
``pair_errors`` is the user's metric — a tag pair's T_A2B through the
chain vs the sheet's truth — before and after a correction.

Pure numpy / scipy — no ROS. ``scripts/chain_calib.py`` collects the
samples (operator-jogged views, one ``capture`` each);
``scripts/check_chain_calib.py`` verifies this module on a synthetic
session with planted errors.
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as _Rot

from path_tag_locator.geometry import invert_T, pose_fr5_to_matrix_m, rot2rpy_deg


# ----------------------------------------------------------------------
# small helpers
# ----------------------------------------------------------------------
def _R_from_rotvec(v):
    r = _Rot.from_rotvec(np.asarray(v, dtype=float))
    return r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()


def _rotvec_from_R(R):
    r = _Rot.from_matrix(R) if hasattr(_Rot, 'from_matrix') else _Rot.from_dcm(R)
    return r.as_rotvec()


def T_from_vec6(v):
    """[rx, ry, rz (rad, rotvec), tx, ty, tz (m)] -> 4x4."""
    T = np.eye(4)
    T[:3, :3] = _R_from_rotvec(v[:3])
    T[:3, 3] = np.asarray(v[3:6], dtype=float)
    return T


def vec6_from_T(T):
    return np.concatenate([_rotvec_from_R(T[:3, :3]), T[:3, 3]])


def pose_error(T_est, T_true):
    """(translation error m, rotation error deg) of T_est vs T_true."""
    E = invert_T(T_true) @ T_est
    ang = math.degrees(np.linalg.norm(_rotvec_from_R(E[:3, :3])))
    return float(np.linalg.norm(E[:3, 3])), float(ang)


# ----------------------------------------------------------------------
# data
# ----------------------------------------------------------------------
@dataclass
class ChainSample:
    label: str
    tcp_pose_mm_deg: List[float]             # the arm's FLANGE pose (/arm/state), Fairino mm / deg
    T_hc2W: np.ndarray                       # sheet in hand_cam, multi-tag PnP
    T_fc2W: np.ndarray                       # sheet in front_cam, multi-tag PnP
    lift_height_m: float = 0.0
    # the corner means the two PnPs were solved from, {tag id: (4,2) px} —
    # kept so `solve` can re-run the PnP with the print's measured scale
    hand_corners: Optional[dict] = None
    front_corners: Optional[dict] = None
    hand_rms_px: float = float("nan")
    front_rms_px: float = float("nan")
    # J1..J6 (deg) from /arm/state at capture — the input of the ARM joint-offset
    # calibration (arm_offsets.py, 2026-09-21): the chain fit needs only the TCP,
    # the arm's own error needs the configuration it was in. NaN for sessions
    # recorded before this field existed.
    joints_deg: Optional[List[float]] = None


# ----------------------------------------------------------------------
# the fit
# ----------------------------------------------------------------------
@dataclass
class FitResult:
    name: str
    D: np.ndarray                      # hand-side correction (ee frame): T_hc2ee' = H @ D
    F: np.ndarray                      # base-side correction (fc frame): B' = B @ F
    rms_pos_m: float
    rms_rot_deg: float
    max_pos_m: float
    max_rot_deg: float
    per_sample: List[tuple] = field(default_factory=list)   # (label, pos_m, rot_deg)
    jackknife_D_sd: Optional[np.ndarray] = None            # sd of vec6(D) leaving one sample out
    jackknife_F_sd: Optional[np.ndarray] = None


def _residuals(params, mode, H, As, B, Ss, w_rot):
    """Stacked residuals: for each sample the log of inv(model)·measured
    (rotation in rad × w_rot, translation in m)."""
    if mode == 'hand':
        D, F = T_from_vec6(params[:6]), np.eye(4)
    elif mode == 'base':
        D, F = np.eye(4), T_from_vec6(params[:6])
    elif mode == 'joint':
        D, F = T_from_vec6(params[:6]), T_from_vec6(params[6:12])
    else:                                      # 'raw'
        D, F = np.eye(4), np.eye(4)
    out = []
    for A, S in zip(As, Ss):
        M = H @ D @ A @ B @ F
        E = invert_T(M) @ S
        out.append(np.concatenate([_rotvec_from_R(E[:3, :3]) * w_rot, E[:3, 3]]))
    return np.concatenate(out)


def _summarise(name, D, F, H, As, B, Ss, labels):
    per = []
    for A, S, lab in zip(As, Ss, labels):
        M = H @ D @ A @ B @ F
        dp, dr = pose_error(S, M)
        per.append((lab, dp, dr))
    p = np.array([x[1] for x in per]); r = np.array([x[2] for x in per])
    return FitResult(name=name, D=D, F=F,
                     rms_pos_m=float(np.sqrt(np.mean(p ** 2))), rms_rot_deg=float(np.sqrt(np.mean(r ** 2))),
                     max_pos_m=float(p.max()), max_rot_deg=float(r.max()), per_sample=per)


def _solve(mode, H, As, B, Ss, w_rot):
    n = 12 if mode == 'joint' else 6
    x0 = np.zeros(n)
    if mode == 'joint':
        # seed the joint fit from the hand-only fit
        xh = least_squares(_residuals, np.zeros(6), args=('hand', H, As, B, Ss, w_rot), method='lm').x
        x0[:6] = xh
    f = least_squares(_residuals, x0, args=(mode, H, As, B, Ss, w_rot), method='lm', xtol=1e-12, ftol=1e-12)
    x = f.x
    if mode == 'hand':
        return T_from_vec6(x[:6]), np.eye(4)
    if mode == 'base':
        return np.eye(4), T_from_vec6(x[:6])
    return T_from_vec6(x[:6]), T_from_vec6(x[6:12])


def level_front_observation(T_fc2W):
    """front_cam's T_fc2W with its ROTATION replaced by the level-floor prior:
    z = the level frame's z (vertical), only the yaw about it kept.

    Why (2026-09-21): front_cam usually sees ONE 90 mm tag (200) at 0.30 m,
    and a single tag's out-of-plane tilt is not the sheet's orientation but
    the LOCAL slope of the paper under that tag — tag 200 sits 100 mm from
    the A0 corner, where paper lifts. The chain model assumes every tag is
    coplanar, so that local slope has nowhere to go but the base-side F:
    the first (discarded) session's F carried 1.16 deg of tilt that was
    1.27 deg of paper under tag 200 (equal and opposite), and re-laying the
    sheet moved it by 1.7 deg. Forcing the prior changed the fit rms by 0.05 mm and dropped
    F's tilt to 0.17 deg. The frame is level by construction (the
    ground-plane correction was fitted so the floor is z = const in it) and
    the sheet lies on that floor, so the prior costs only the floor-slope
    difference between the two spots (~0.2-0.4 deg here). Translation and
    yaw of a single-tag PnP are fine and are kept."""
    T = np.asarray(T_fc2W, dtype=float)
    yaw = math.atan2(T[1, 0], T[0, 0])
    c, s_ = math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    out[:3, 3] = T[:3, 3]
    return out


def level_front_samples(samples: List[ChainSample]):
    """Apply :func:`level_front_observation` in place; returns the samples."""
    for s in samples:
        s.T_fc2W = level_front_observation(s.T_fc2W)
    return samples


def build_inputs(samples: List[ChainSample], T_hc2ee, T_ab2mb, T_mb2fc, lift_compensate=None):
    """(H, [A_i], B, [S_i], labels) with S_i = T_hc2W · inv(T_fc2W). B is
    taken at the FIRST sample's lift height (all samples must share it;
    the tool keeps the lift still)."""
    H = np.asarray(T_hc2ee, dtype=float)
    As, Ss, labels = [], [], []
    for s in samples:
        As.append(invert_T(pose_fr5_to_matrix_m(s.tcp_pose_mm_deg)))
        Ss.append(np.asarray(s.T_hc2W) @ invert_T(np.asarray(s.T_fc2W)))
        labels.append(s.label)
    T_ab2mb_c = np.asarray(T_ab2mb, dtype=float)
    if lift_compensate is not None:
        T_ab2mb_c = lift_compensate(T_ab2mb_c, samples[0].lift_height_m)
    B = T_ab2mb_c @ np.asarray(T_mb2fc, dtype=float)
    return H, As, B, Ss, labels


def fit_corrections(samples: List[ChainSample], T_hc2ee, T_ab2mb, T_mb2fc,
                    lift_compensate=None, w_rot=0.5, jackknife=True):
    """raw residual (= the chain's T_hc2fc error per view, handoff §6.3)
    + hand / base / joint fits (+ leave-one-out sd). ``w_rot`` weights a
    radian of rotation residual against a metre of translation
    (0.5 m/rad ≈ the hc -> fc lever: 1 deg ~ 9 mm)."""
    H, As, B, Ss, labels = build_inputs(samples, T_hc2ee, T_ab2mb, T_mb2fc, lift_compensate)
    results = {'raw': _summarise('raw', np.eye(4), np.eye(4), H, As, B, Ss, labels)}
    for mode in ('hand', 'base', 'joint'):
        D, F = _solve(mode, H, As, B, Ss, w_rot)
        results[mode] = _summarise(mode, D, F, H, As, B, Ss, labels)
    n = len(As)
    if jackknife and n >= 4:
        for mode in ('hand', 'base', 'joint'):
            Ds, Fs = [], []
            for k in range(n):
                idx = [i for i in range(n) if i != k]
                D, F = _solve(mode, H, [As[i] for i in idx], B, [Ss[i] for i in idx], w_rot)
                Ds.append(vec6_from_T(D)); Fs.append(vec6_from_T(F))
            Ds, Fs = np.array(Ds), np.array(Fs)
            results[mode].jackknife_D_sd = np.sqrt((n - 1) / n * ((Ds - Ds.mean(0)) ** 2).sum(0))
            results[mode].jackknife_F_sd = np.sqrt((n - 1) / n * ((Fs - Fs.mean(0)) ** 2).sum(0))
    return results


def evaluate(D, F, samples: List[ChainSample], T_hc2ee, T_ab2mb, T_mb2fc, lift_compensate=None,
             name='holdout') -> FitResult:
    """Residual of a GIVEN correction on samples that did not take part in
    the fit (view hold-out, handoff §9)."""
    H, As, B, Ss, labels = build_inputs(samples, T_hc2ee, T_ab2mb, T_mb2fc, lift_compensate)
    return _summarise(name, np.asarray(D), np.asarray(F), H, As, B, Ss, labels)


def pair_errors(samples: List[ChainSample], sheet, T_hc2ee, T_ab2mb, T_mb2fc, D=None, F=None,
                lift_compensate=None, pairs=None):
    """The user's metric: for each sample the pose of tag B (front_cam
    side) in tag A (hand_cam side) THROUGH THE CHAIN,

        T_A2B_est = inv(T_hc2A) · (H · D · A_i · B · F) · T_fc2B,
        T_hc2A = T_hc2W · T_W2A,  T_fc2B = T_fc2W · T_W2B,

    against the sheet's T_A2B. ``pairs`` = [(A, B)] per sample; default =
    the tag nearest each camera's optical axis (``sheet.axis_tag``).
    Returns [(label, A, B, pos_err_m, rot_err_deg, t_err_in_B_mm(3))]."""
    from .sheet import axis_tag
    D = np.eye(4) if D is None else np.asarray(D)
    F = np.eye(4) if F is None else np.asarray(F)
    H = np.asarray(T_hc2ee)
    out = []
    for i, s in enumerate(samples):
        T_ab2mb_c = np.asarray(T_ab2mb, dtype=float)
        if lift_compensate is not None:
            T_ab2mb_c = lift_compensate(T_ab2mb_c, s.lift_height_m)
        A_i = invert_T(pose_fr5_to_matrix_m(s.tcp_pose_mm_deg))
        M = H @ D @ A_i @ T_ab2mb_c @ np.asarray(T_mb2fc) @ F
        if pairs is not None:
            a, b = pairs[i]
        else:
            a = axis_tag(s.T_hc2W, sheet, (s.hand_corners or {}).keys() or None)
            b = axis_tag(s.T_fc2W, sheet, (s.front_corners or {}).keys() or None)
        T_hc2A = np.asarray(s.T_hc2W) @ sheet.T_W2k(a)
        T_fc2B = np.asarray(s.T_fc2W) @ sheet.T_W2k(b)
        est = invert_T(T_hc2A) @ M @ T_fc2B
        gt = sheet.T_A2B(a, b)
        dp, dr = pose_error(est, gt)
        E = invert_T(gt) @ est
        out.append((s.label, int(a), int(b), dp, dr, E[:3, 3] * 1e3))
    return out


def all_pair_errors(samples: List[ChainSample], sheet, T_hc2ee, T_ab2mb, T_mb2fc, D=None, F=None,
                    lift_compensate=None):
    """``pair_errors`` for EVERY (A seen by hand_cam, B seen by front_cam)
    combination of every sample — e.g. 305->300 and 305->301 from one view.
    Only pairs split across the two cameras measure the chain; two tags
    in the SAME camera differ by a PnP consistency only (the chain
    cancels), so those are not listed."""
    out = []
    for s in samples:
        for a in sorted(s.hand_corners or []):
            for b in sorted(s.front_corners or []):
                if a in sheet.t_W_m and b in sheet.t_W_m:
                    out.extend(pair_errors([s], sheet, T_hc2ee, T_ab2mb, T_mb2fc, D, F, lift_compensate, pairs=[(a, b)]))
    return out


def summarise_errors(errs):
    """(mean, sd, rms, p95) of position [mm] and rotation [deg] over a
    list of (…, pos_m, rot_deg, …) tuples — bias vs random, handoff §1.3."""
    p = np.array([e[3] for e in errs]) * 1e3
    r = np.array([e[4] for e in errs])
    f = lambda v: (float(v.mean()), float(v.std()), float(np.sqrt((v ** 2).mean())), float(np.percentile(v, 95)))
    return f(p), f(r)


def residual_correlations(res: FitResult, views):
    """Pearson r of the per-sample position residual against view range,
    tilt and spin — the diagnosis table of handoff §9.2 (a residual that
    tracks range/tilt points at detection / intrinsics / tag size, one
    that tracks the arm pose at FK, none at a constant extrinsic)."""
    p = np.array([dp for _, dp, _ in res.per_sample])
    out = {}
    for name, vals in (('range_m', [v.range_m for v in views]), ('tilt_deg', [v.tilt_deg for v in views]),
                       ('spin_deg', [v.spin_deg for v in views])):
        x = np.asarray(vals, dtype=float)
        if len(x) < 3 or x.std() < 1e-9 or p.std() < 1e-12:
            out[name] = float("nan")
        else:
            out[name] = float(np.corrcoef(x, p)[0, 1])
    return out


# ----------------------------------------------------------------------
# what a correction means for the config
# ----------------------------------------------------------------------
def corrected_hand_eye(T_hc2ee, D):
    return np.asarray(T_hc2ee) @ np.asarray(D)


def corrected_T_ab2mb(T_ab2mb, T_mb2fc, F):
    """Fold a base-side correction F (fc frame) into T_ab2mb, leaving the
    GENERATED T_mb2fc untouched: T_ab2mb' = T_ab2mb · T_mb2fc · F · inv(T_mb2fc)."""
    T_mb2fc = np.asarray(T_mb2fc)
    return np.asarray(T_ab2mb) @ T_mb2fc @ np.asarray(F) @ invert_T(T_mb2fc)


def describe_T(T, name):
    p = T[:3, 3] * 1e3
    rx, ry, rz = rot2rpy_deg(T[:3, :3])
    return "%s: t (%+.2f, %+.2f, %+.2f) mm  rpy (%+.3f, %+.3f, %+.3f) deg" % (name, p[0], p[1], p[2], rx, ry, rz)


def rotation_diversity_deg(samples: List[ChainSample]):
    """Largest angle between any two samples' hand-cam orientations — the
    excitation the hand/base split relies on (needs tens of degrees)."""
    Rs = [pose_fr5_to_matrix_m(s.tcp_pose_mm_deg)[:3, :3] for s in samples]
    worst = 0.0
    for i in range(len(Rs)):
        for j in range(i + 1, len(Rs)):
            worst = max(worst, math.degrees(np.linalg.norm(_rotvec_from_R(Rs[i].T @ Rs[j]))))
    return worst
