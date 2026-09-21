"""
basler_tip.py
=============
Where the wrist Basler looks, in the flange frame — the ``vision_tip``
(``apriltag_nav/config/tf/tf_chain.yaml`` ``T_ee2tip``, the planner URDF's
``vision_tip_joint``, tool 1 of ``set_tool_tcp.py``) measured instead of
designed, from the same printed tag sheet the hand camera sees.

Why not a hand-eye for the Basler: at its 16.5 mm working distance
(keyence sensor zero, 2026-09-18) the lens is macro, the depth of field
a few mm and the projection nearly orthographic, so a tag's out-of-plane
tilt is not measurable and there is no CameraInfo. What the scan needs
is also less than 6-DOF: the point on the focus plane that the frame
CENTRE images (3 numbers in the flange frame) and the image's roll
about the flange z (1 number). Model:

    tip frame = flange frame rotated by psi about z, moved by p_tip;
    a sheet point imaged at pixel (u, v) at the focus standoff is, in
    the tip frame, ((u - cx) / s, (v - cy) / s, 0)   (s = px/mm)

Inputs, all with the base still and the sheet flat on the plate:

* hand samples: the arm at 0.25-0.35 m over the sheet, hand_cam corners
  of every tag it sees -> ``multi_tag_pnp`` -> ``T_hc2W`` -> with the
  hand-eye D and the flange pose A: ``T_ab2W = A · inv(D) · T_hc2W``.
  Several poses, averaged (their scatter = the hand_cam chain's own
  accuracy, the floor of everything below).
* basler samples: the arm jogged so the Basler is at its standoff over
  one tag (the Keyence standoff loop puts it there), one lamp-on frame,
  the tag corners at full resolution -> a 2-D similarity px -> W over
  all corners seen -> the W point under the frame centre ``o_j`` and the
  W angle of the image x axis ``theta_j``; plus the flange pose ``A_j``.

Solve (least squares over p_tip, psi): for every basler sample
    T_ab2W · [o_j, 0]  ==  A_j · [p_tip]                (3 residuals, mm)
    angle_W(R_W^T · R_Aj · Rz(psi) · x)  ==  theta_j     (1 residual, deg)
The z of p_tip is the standoff geometry as the hand_cam chain sees it —
i.e. the CSV z the planner must use to land the Basler at focus.

Pure numpy / cv2 / scipy, no ROS; ``scripts/basler_tip_calib.py`` is
the ROS side and ``scripts/check_basler_tip.py`` the offline check.
"""
import json
import math
import os
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from path_tag_locator.geometry import invert_T, pose_fr5_to_matrix_m

from .sheet import Sheet, multi_tag_pnp


# ----------------------------------------------------------------------
# samples
# ----------------------------------------------------------------------
@dataclass
class HandSample:
    label: str
    tcp_pose_mm_deg: List[float]
    corners: Dict[int, np.ndarray]            # hand_cam, (4,2) means
    rms_px: float = float('nan')
    T_ab2W: Optional[np.ndarray] = None       # filled by resolve_hand()


@dataclass
class BaslerSample:
    label: str
    tcp_pose_mm_deg: List[float]
    corners: Dict[int, np.ndarray]            # Basler, (4,2) full-res px
    image_wh: Tuple[int, int]
    standoff_mm: float = float('nan')         # the Keyence reading at capture, when known
    image_path: str = ''
    # derived by fit_similarity()
    o_W_m: Optional[np.ndarray] = None        # sheet point under the frame centre (x, y in W, m)
    theta_W_deg: float = float('nan')         # W angle of the image x axis
    px_per_mm: float = float('nan')
    sim_rms_px: float = float('nan')


def _np(x):
    return None if x is None else np.asarray(x, dtype=float).tolist()


def save_session(dir_, hand: List[HandSample], basler: List[BaslerSample], meta: dict):
    os.makedirs(dir_, exist_ok=True)
    d = {
        'meta': meta,
        'hand': [{'label': s.label, 'tcp_pose_mm_deg': list(map(float, s.tcp_pose_mm_deg)),
                  'corners': {str(k): _np(v) for k, v in s.corners.items()}, 'rms_px': float(s.rms_px)} for s in hand],
        'basler': [{'label': s.label, 'tcp_pose_mm_deg': list(map(float, s.tcp_pose_mm_deg)),
                    'corners': {str(k): _np(v) for k, v in s.corners.items()}, 'image_wh': list(s.image_wh),
                    'standoff_mm': float(s.standoff_mm), 'image_path': s.image_path} for s in basler],
    }
    tmp = os.path.join(dir_, 'session.json.tmp')
    with open(tmp, 'w') as fh:
        json.dump(d, fh, indent=1)
    os.replace(tmp, os.path.join(dir_, 'session.json'))


def load_session(dir_):
    p = os.path.join(dir_, 'session.json')
    if not os.path.exists(p):
        return [], [], {}
    with open(p) as fh:
        d = json.load(fh)
    hand = [HandSample(s['label'], s['tcp_pose_mm_deg'], {int(k): np.asarray(v, float) for k, v in s['corners'].items()},
                       float(s.get('rms_px', 'nan'))) for s in d.get('hand', [])]
    bas = [BaslerSample(s['label'], s['tcp_pose_mm_deg'], {int(k): np.asarray(v, float) for k, v in s['corners'].items()},
                        tuple(s['image_wh']), float(s.get('standoff_mm', 'nan')), s.get('image_path', ''))
           for s in d.get('basler', [])]
    return hand, bas, d.get('meta', {})


# ----------------------------------------------------------------------
# hand_cam side: the sheet in the arm frame
# ----------------------------------------------------------------------
def resolve_hand(hand: List[HandSample], sheet: Sheet, K, D, T_hc2ee):
    """T_ab2W per hand sample = A · inv(T_hc2ee) · T_hc2W."""
    T_ee2hc = invert_T(np.asarray(T_hc2ee, float))
    for s in hand:
        r = multi_tag_pnp(s.corners, sheet, K, D)
        s.rms_px = r.rms_px
        if not np.isfinite(r.T_cam2W).all():      # IPPE/LM can return NaN on a degenerate exact set
            s.T_ab2W = None
            continue
        A = pose_fr5_to_matrix_m(s.tcp_pose_mm_deg)
        s.T_ab2W = A @ T_ee2hc @ r.T_cam2W
    return hand


def _mean_rotation(Rs):
    M = np.mean(Rs, axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def sheet_pose(hand: List[HandSample]):
    """Mean T_ab2W over the resolved hand samples and their scatter:
    (T_ab2W, rms position mm, max position mm, rms normal deg)."""
    Ts = [s.T_ab2W for s in hand if s.T_ab2W is not None]
    if not Ts:
        raise RuntimeError('no resolved hand samples')
    R = _mean_rotation([T[:3, :3] for T in Ts])
    t = np.mean([T[:3, 3] for T in Ts], axis=0)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    d = np.array([np.linalg.norm(Ti[:3, 3] - t) for Ti in Ts]) * 1e3
    ang = np.array([math.degrees(math.acos(np.clip(float(Ti[:3, 2] @ R[:, 2]), -1, 1))) for Ti in Ts])
    return T, float(np.sqrt((d ** 2).mean())), float(d.max()), float(np.sqrt((ang ** 2).mean()))


# ----------------------------------------------------------------------
# Basler side: corners -> the sheet point under the frame centre
# ----------------------------------------------------------------------
def detect_basler_corners(gray, family='tag36h11', decimate=4, refine=True) -> Dict[int, np.ndarray]:
    """Tag corners in a full-resolution Basler frame. The 5472 x 3648 frame
    is detected at 1/``decimate`` (a 20 mm tag at the 16.5 mm standoff
    is ~1000+ px, so 250 px decimated is plenty), the corners scaled
    back and refined at full resolution with cornerSubPix."""
    if cv2 is None:
        raise RuntimeError('detect_basler_corners needs cv2')
    from path_tag_locator.detect import _get_detector
    if gray.ndim == 3:
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (gray.shape[1] // decimate, gray.shape[0] // decimate), interpolation=cv2.INTER_AREA) \
        if decimate > 1 else gray
    dets = _get_detector(family).detect(small, estimate_tag_pose=False)
    out = {}
    for d in dets:
        c = np.asarray(d.corners, dtype=np.float32).reshape(4, 2) * float(decimate)
        if refine:
            win = max(5, int(round(0.03 * np.linalg.norm(c[1] - c[0]))))
            c = cv2.cornerSubPix(gray, c.reshape(-1, 1, 2).astype(np.float32), (win, win), (-1, -1),
                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01)).reshape(4, 2)
        out[int(d.tag_id)] = np.asarray(c, dtype=float)
    return out


def fit_similarity(corners: Dict[int, np.ndarray], sheet: Sheet, image_wh, centre_px=None):
    """2-D similarity  W_xy = s · R(theta) · px + t  over every corner of
    every sheet tag seen (least squares, Umeyama). Returns
    (o_W_m under the frame centre, theta_W_deg of the image x axis,
    px_per_mm, rms_px, tag ids used)."""
    ids = sorted(k for k in corners if k in sheet.t_W_m)
    if not ids:
        raise RuntimeError('no sheet tag in the Basler frame (seen %s)' % sorted(corners))
    P = np.vstack([np.asarray(corners[k], float).reshape(4, 2) for k in ids])
    Q = np.vstack([sheet.corners_W(k)[:, :2] for k in ids])
    mp, mq = P.mean(0), Q.mean(0)
    Pc, Qc = P - mp, Q - mq
    H = Pc.T @ Qc
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                       # a mirrored fit means the corner order is wrong
        Vt[-1] *= -1
        R = Vt.T @ U.T
    s = float(S.sum() / (Pc ** 2).sum())           # m per px
    t = mq - s * R @ mp
    fit = (s * (R @ Pc.T)).T + mq
    rms = float(np.sqrt(((fit - Q) ** 2).sum(1).mean())) / s    # px
    w, h = image_wh
    c = np.array(centre_px if centre_px is not None else [(w - 1) / 2.0, (h - 1) / 2.0], float)
    o = s * R @ c + t
    theta = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return o, theta, 1.0 / (s * 1e3), rms, ids


def resolve_basler(bas: List[BaslerSample], sheet: Sheet, centre_px=None):
    for b in bas:
        o, th, ppm, rms, _ = fit_similarity(b.corners, sheet, b.image_wh, centre_px)
        b.o_W_m, b.theta_W_deg, b.px_per_mm, b.sim_rms_px = o, th, ppm, rms
    return bas


# ----------------------------------------------------------------------
# the solve
# ----------------------------------------------------------------------
@dataclass
class TipResult:
    p_tip_mm: np.ndarray                     # flange frame
    psi_deg: float                           # image x axis vs flange x, about flange z
    n_hand: int
    n_basler: int
    sheet_scatter_mm: float                  # hand samples' T_ab2W position rms
    sheet_scatter_deg: float
    resid_mm: np.ndarray                     # per basler sample, |position residual|
    resid_deg: np.ndarray                    # per basler sample, angle residual
    rms_mm: float
    max_mm: float
    rms_deg: float
    jackknife_sd_mm: Optional[np.ndarray] = None
    labels: List[str] = field(default_factory=list)


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _angle_residual(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def solve_tip(T_ab2W, bas: List[BaslerSample], p0_mm=(0.0, -253.0, 225.2), w_deg_per_mm=0.2):
    """Least squares over (p_tip [mm], psi [deg]). ``w_deg_per_mm``:
    weight of the angle residuals relative to the position ones (1 deg
    counts as 5 mm by default; the two are decoupled anyway — psi only
    enters the angle rows)."""
    from scipy.optimize import least_squares
    T_ab2W = np.asarray(T_ab2W, float)
    R_W, t_W = T_ab2W[:3, :3], T_ab2W[:3, 3]
    As = [pose_fr5_to_matrix_m(b.tcp_pose_mm_deg) for b in bas]
    o_ab = [R_W @ np.array([b.o_W_m[0], b.o_W_m[1], 0.0]) + t_W for b in bas]
    x_ab = [R_W @ np.array([math.cos(math.radians(b.theta_W_deg)), math.sin(math.radians(b.theta_W_deg)), 0.0]) for b in bas]

    def resid(x):
        p = x[:3] / 1e3
        psi = math.radians(x[3])
        r = []
        for A, o, xa in zip(As, o_ab, x_ab):
            pa = A[:3, :3] @ p + A[:3, 3]
            r.extend(((pa - o) * 1e3).tolist())
            # image x axis in the arm frame, then its angle in the sheet plane
            xi = A[:3, :3] @ (_Rz(psi) @ np.array([1.0, 0.0, 0.0]))
            xi_W = R_W.T @ xi
            ang = math.degrees(math.atan2(xi_W[1], xi_W[0]))
            th = math.degrees(math.atan2((R_W.T @ xa)[1], (R_W.T @ xa)[0]))
            r.append(_angle_residual(ang, th) / w_deg_per_mm)
        return np.array(r)

    x0 = np.array(list(p0_mm) + [0.0], float)
    # psi start: from the first sample's angle directly
    A = As[0]
    xi_W = R_W.T @ (A[:3, :3] @ np.array([1.0, 0.0, 0.0]))
    x0[3] = _angle_residual(bas[0].theta_W_deg, math.degrees(math.atan2(xi_W[1], xi_W[0])))
    sol = least_squares(resid, x0, xtol=1e-12, ftol=1e-12)
    r = resid(sol.x).reshape(len(bas), 4)
    pos = np.linalg.norm(r[:, :3], axis=1)
    ang = r[:, 3] * w_deg_per_mm
    return sol.x[:3].copy(), float(sol.x[3]), pos, ang


def fit_tip(hand: List[HandSample], bas: List[BaslerSample], p0_mm=(0.0, -253.0, 225.2), jackknife=True) -> TipResult:
    T_ab2W, sc_mm, sc_max, sc_deg = sheet_pose(hand)
    p, psi, pos, ang = solve_tip(T_ab2W, bas, p0_mm)
    res = TipResult(p_tip_mm=p, psi_deg=psi, n_hand=len(hand), n_basler=len(bas),
                    sheet_scatter_mm=sc_mm, sheet_scatter_deg=sc_deg, resid_mm=pos, resid_deg=ang,
                    rms_mm=float(np.sqrt((pos ** 2).mean())), max_mm=float(pos.max()),
                    rms_deg=float(np.sqrt((ang ** 2).mean())), labels=[b.label for b in bas])
    if jackknife and len(bas) >= 3:
        ps = []
        for j in range(len(bas)):
            sub = [b for i, b in enumerate(bas) if i != j]
            ps.append(solve_tip(T_ab2W, sub, p0_mm)[0])
        ps = np.array(ps)
        res.jackknife_sd_mm = ps.std(0) * math.sqrt(len(bas) - 1)
    return res


def T_ee2tip(res: TipResult):
    T = np.eye(4)
    T[:3, :3] = _Rz(math.radians(res.psi_deg))
    T[:3, 3] = res.p_tip_mm / 1e3
    return T


def summarize(res: TipResult, T_hc2ee=None, design_mm=(0.0, -253.0, 225.2)) -> str:
    p = res.p_tip_mm
    d = np.asarray(design_mm, float)
    lines = [
        'sheet pose from %d hand samples: scatter %.1f mm rms / %.2f deg (the floor of everything below)'
        % (res.n_hand, res.sheet_scatter_mm, res.sheet_scatter_deg),
        'vision tip (flange frame): x %+.1f  y %+.1f  z %+.1f mm   (design %+.1f %+.1f %+.1f -> delta %+.1f %+.1f %+.1f)'
        % (p[0], p[1], p[2], d[0], d[1], d[2], p[0] - d[0], p[1] - d[1], p[2] - d[2]),
        'image x axis vs flange x: psi %+.2f deg' % res.psi_deg,
        'fit over %d basler samples: %.1f mm rms / %.1f max, angle %.2f deg rms'
        % (res.n_basler, res.rms_mm, res.max_mm, res.rms_deg),
    ]
    if res.jackknife_sd_mm is not None:
        lines.append('jackknife sd: %.1f / %.1f / %.1f mm' % tuple(res.jackknife_sd_mm))
    for lab, r, a in zip(res.labels, res.resid_mm, res.resid_deg):
        lines.append('  %-6s %5.1f mm  %+5.2f deg' % (lab, r, a))
    if T_hc2ee is not None:
        T = np.asarray(T_hc2ee, float) @ T_ee2tip(res)
        t = T[:3, 3] * 1e3
        lines.append('T_hc2tip (tip in the hand_cam frame): t = (%+.1f, %+.1f, %+.1f) mm' % (t[0], t[1], t[2]))
    return '\n'.join(lines)
