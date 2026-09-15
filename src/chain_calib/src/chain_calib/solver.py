"""
solver.py
=========
Measure the error of the front_cam <-> hand_cam transform chain with two
floor tags laid a known distance apart, and fit a constant correction
(2026-09-15, user request: "T_fc2hc 사이의 tf들은 블랙박스 — 이상값과
실측값의 오차를 보정값으로").

The chain the locator uses (chain.py, T_X2Y = pose of Y in X):

    T_A2B = inv(T_hc2A) · T_hc2ee · T_ee2ab · T_ab2mb · T_mb2fc · T_fc2B
                          \\_____________ T_hc2fc(model) ______________/

With tag A under hand_cam and tag B under front_cam laid on ONE floor
with their edges collinear (a straightedge) and the centre distance d
measured, the TRUE T_A2B is known: R = Rz(k·90°), t = d·(cos a, sin a, 0)
with a a multiple of 90° (which multiples: whichever the raw chain is
closest to — its error is degrees, not a quarter turn). Every arm pose i
then gives a MEASURED T_hc2fc:

    S_i = T_hc2A_i · T_A2B_true · inv(T_fc2B_i)

against the model M_i = H · A_i · B with H = T_hc2ee, A_i = T_ee2ab_i
(arm FK), B = T_ab2mb(lift) · T_mb2fc. The base does not move, so B and
T_fc2B are the same for every sample and the arm poses are the only
excitation — exactly the AX = YB (robot-world / hand-eye) problem:

    inv(H) · S_i = D · (A_i · B) · F

with D a constant correction on the HAND side (T_hc2ee' = H · D, i.e. the
hand-eye is wrong) and F a constant correction on the BASE side
(B' = B · F: the arm mount T_ab2mb, T_mb2fc, or the front tag's
observation — everything after the arm, indistinguishable from one base
pose). A single arm pose cannot tell D from F; a set of poses with
rotation diversity (tilts about two axes + spins, as the hand-eye sweep
produces) can, because D sits BEFORE the varying A_i and F after it.

``fit_corrections`` fits three models and reports each one's residual:
hand only (F = I), base only (D = I), joint. Read the attribution off the
residuals: if the hand-only fit already reaches the noise floor, the
error is the hand-eye; if only the joint fit does, both sides carry
error; if the base-only fit is as good as the hand-only one the poses
did not have enough rotation diversity and the split is undetermined.

Pure numpy / scipy — no ROS. ``scripts/chain_calib.py`` collects the
samples (operator-jogged views, one ``capture`` each);
``scripts/check_chain_calib.py`` verifies this module on a synthetic chain
with planted errors.
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


def Rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


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
    tcp_pose_mm_deg: List[float]
    T_hc2A: np.ndarray
    T_fc2B: np.ndarray
    lift_height_m: float = 0.0


@dataclass
class TruthChoice:
    k90: int            # tag B's in-plane rotation vs A, in quarter turns
    a90: int            # direction of A -> B in A's frame, in quarter turns
    T_A2B: np.ndarray
    raw_pos_err_m: float
    raw_rot_err_deg: float


def truth_T_A2B(spacing_m, k90, a90):
    T = np.eye(4)
    T[:3, :3] = Rz(90.0 * k90)
    T[0, 3] = spacing_m * math.cos(math.radians(90.0 * a90))
    T[1, 3] = spacing_m * math.sin(math.radians(90.0 * a90))
    return T


def snap_truth(T_A2B_model_mean, spacing_m) -> TruthChoice:
    """The laid geometry, up to the quarter-turn ambiguities of two tags
    with collinear edges: pick the (k, a) the raw chain is closest to."""
    best = None
    for k in range(4):
        for a in range(4):
            T = truth_T_A2B(spacing_m, k, a)
            dp, dr = pose_error(T_A2B_model_mean, T)
            score = dp / max(spacing_m, 1e-3) + math.radians(dr)
            if best is None or score < best[0]:
                best = (score, k, a, T, dp, dr)
    _, k, a, T, dp, dr = best
    return TruthChoice(k90=k, a90=a, T_A2B=T, raw_pos_err_m=dp, raw_rot_err_deg=dr)


def mean_T(Ts):
    """Chordal mean of a few close transforms."""
    Ts = [np.asarray(T, dtype=float) for T in Ts]
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    M = np.mean([T[:3, :3] for T in Ts], axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


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
        # seed the joint fit from the two single-side fits
        xh = least_squares(_residuals, np.zeros(6), args=('hand', H, As, B, Ss, w_rot), method='lm').x
        x0[:6] = xh
    f = least_squares(_residuals, x0, args=(mode, H, As, B, Ss, w_rot), method='lm', xtol=1e-12, ftol=1e-12)
    x = f.x
    if mode == 'hand':
        return T_from_vec6(x[:6]), np.eye(4)
    if mode == 'base':
        return np.eye(4), T_from_vec6(x[:6])
    return T_from_vec6(x[:6]), T_from_vec6(x[6:12])


def build_inputs(samples: List[ChainSample], T_hc2ee, T_ab2mb, T_mb2fc, truth_T_A2B, lift_compensate=None):
    """(H, [A_i], B, [S_i], labels). B is taken at the FIRST sample's lift
    height (all samples must share the lift height; the tool keeps the
    lift still)."""
    H = np.asarray(T_hc2ee, dtype=float)
    As, Ss, labels = [], [], []
    for s in samples:
        As.append(invert_T(pose_fr5_to_matrix_m(s.tcp_pose_mm_deg)))
        Ss.append(np.asarray(s.T_hc2A) @ truth_T_A2B @ invert_T(np.asarray(s.T_fc2B)))
        labels.append(s.label)
    T_ab2mb_c = np.asarray(T_ab2mb, dtype=float)
    if lift_compensate is not None:
        T_ab2mb_c = lift_compensate(T_ab2mb_c, samples[0].lift_height_m)
    B = T_ab2mb_c @ np.asarray(T_mb2fc, dtype=float)
    return H, As, B, Ss, labels


def raw_chain_T_A2B(samples: List[ChainSample], T_hc2ee, T_ab2mb, T_mb2fc, lift_compensate=None):
    """The production chain's T_A2B per sample (before any correction)."""
    out = []
    for s in samples:
        T_ab2mb_c = np.asarray(T_ab2mb, dtype=float)
        if lift_compensate is not None:
            T_ab2mb_c = lift_compensate(T_ab2mb_c, s.lift_height_m)
        A = invert_T(pose_fr5_to_matrix_m(s.tcp_pose_mm_deg))
        out.append(invert_T(np.asarray(s.T_hc2A)) @ np.asarray(T_hc2ee) @ A @ T_ab2mb_c @ np.asarray(T_mb2fc) @ np.asarray(s.T_fc2B))
    return out


def fit_corrections(samples: List[ChainSample], T_hc2ee, T_ab2mb, T_mb2fc, spacing_m,
                    lift_compensate=None, w_rot=0.5, jackknife=True):
    """Everything: truth snap, raw residual, hand / base / joint fits.
    ``w_rot`` weights a radian of rotation residual against a metre of
    translation (0.5 m/rad ≈ the hc -> fc lever: 1 deg ~ 9 mm)."""
    raw = raw_chain_T_A2B(samples, T_hc2ee, T_ab2mb, T_mb2fc, lift_compensate)
    truth = snap_truth(mean_T(raw), spacing_m)
    H, As, B, Ss, labels = build_inputs(samples, T_hc2ee, T_ab2mb, T_mb2fc, truth.T_A2B, lift_compensate)
    results = {'raw': _summarise('raw', np.eye(4), np.eye(4), H, As, B, Ss, labels)}
    for mode in ('hand', 'base', 'joint'):
        D, F = _solve(mode, H, As, B, Ss, w_rot)
        results[mode] = _summarise(mode, D, F, H, As, B, Ss, labels)
    if jackknife and len(samples) >= 4:
        for mode in ('hand', 'base', 'joint'):
            Ds, Fs = [], []
            for k in range(len(samples)):
                idx = [i for i in range(len(samples)) if i != k]
                D, F = _solve(mode, H, [As[i] for i in idx], B, [Ss[i] for i in idx], w_rot)
                Ds.append(vec6_from_T(D)); Fs.append(vec6_from_T(F))
            n = len(samples)
            Ds, Fs = np.array(Ds), np.array(Fs)
            results[mode].jackknife_D_sd = np.sqrt((n - 1) / n * ((Ds - Ds.mean(0)) ** 2).sum(0))
            results[mode].jackknife_F_sd = np.sqrt((n - 1) / n * ((Fs - Fs.mean(0)) ** 2).sum(0))
    return truth, results


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
