"""
sheet.py
========
The printed A0 calibration sheet as a rigid ground truth (2026-09-18,
`sheet/A0_landscape_tag200_300-309_5x2_FINAL2_layout.json`): eleven
tag36h11 tags — 200 alone at one end, 300..309 as a 5 x 2 grid at
150 mm pitch — all in ONE plane with the SAME orientation, so every tag
frame k is a pure translation of the sheet frame W (= tag 200).

Frame convention (verified against the PDF with dt_apriltags, not read
off the doc): W origin = tag 200 centre, +x = paper right, +y = paper
down as printed, +z INTO the paper (AprilTag 3). dt_apriltags reports
its corners bottom-left, bottom-right, top-right, top-left of the
printed tag, i.e. `path_tag_locator.detections._CORNER_ORDER`
((-h,+h), (+h,+h), (+h,-h), (-h,-h)) — the same table the locator's
single-tag re-solve uses, so a multi-tag PnP over the sheet lands in
exactly the frame the rest of the chain expects for one tag.

What this buys over two loose tags (solver.py's original mode):

* each camera solves ``T_cam2W`` from EVERY tag it sees — 8 to 16 corner
  points instead of 4, over a 150-600 mm baseline instead of 90 mm, so
  the out-of-plane tilt (the term the error budget says dominates) is
  measured, not extrapolated from one tag's edge-length difference;
* the truth needs no quarter-turn snap — the JSON is the truth, and the
  two cameras may look at ANY tags of the sheet;
* the print's scale is a parameter (`sx`, `sy`, `tag_size_m`) applied to
  the design coordinates, per the handoff doc §4.4 — measure the print.

Pure numpy / cv2, no ROS. ``scripts/chain_calib.py --sheet`` feeds it
corner means collected from ``/<cam>/tag_detections``.
"""
import json
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import cv2
except ImportError:                     # the layout still loads without it
    cv2 = None

from path_tag_locator.detections import _CORNER_ORDER
from path_tag_locator.geometry import invert_T


# ----------------------------------------------------------------------
# layout
# ----------------------------------------------------------------------
@dataclass
class Sheet:
    path: str
    world_tag: int
    tag_size_m: float                       # black edge, measured on the print
    sx: float                               # print scale along W x (1.0 = design)
    sy: float                               # print scale along W y
    t_W_m: Dict[int, np.ndarray]            # tag id -> centre in W (m), design x scale
    family: str = "tag36h11"
    design_tag_size_m: float = 0.090

    @property
    def ids(self) -> List[int]:
        return sorted(self.t_W_m)

    def T_W2k(self, tag_id: int) -> np.ndarray:
        """Tag k in W: pure translation (same plane, same orientation)."""
        T = np.eye(4)
        T[:3, 3] = self.t_W_m[int(tag_id)]
        return T

    def T_A2B(self, tag_a: int, tag_b: int) -> np.ndarray:
        """GROUND TRUTH pose of tag B in tag A's frame."""
        return invert_T(self.T_W2k(tag_a)) @ self.T_W2k(tag_b)

    def corners_W(self, tag_id: int) -> np.ndarray:
        """(4, 3) corner points of tag k in W (m), in dt_apriltags order."""
        p = np.zeros((4, 3))
        p[:, :2] = _CORNER_ORDER * (self.tag_size_m / 2.0)
        return p + self.t_W_m[int(tag_id)]

    def nearest_tag(self, xy_W) -> int:
        xy = np.asarray(xy_W, dtype=float)[:2]
        return min(self.t_W_m, key=lambda k: np.hypot(*(self.t_W_m[k][:2] - xy)))

    def describe(self) -> str:
        return ("sheet %s: %d tags %s, tag %.2f mm, scale sx %.5f sy %.5f (W = tag %d)"
                % (self.path, len(self.t_W_m), self.ids, self.tag_size_m * 1e3, self.sx, self.sy, self.world_tag))


def load_sheet(json_path: str, sx: float = 1.0, sy: float = 1.0, tag_size_m: Optional[float] = None) -> Sheet:
    """Load the layout JSON (mm) and apply the print's measured scale:
    t_k = (sx * x_k, sy * y_k, 0), tag size = the measured black edge.
    Defaults are the DESIGN values — a plotter is off by 0.1-0.3 % along
    its feed, i.e. 1-2 mm over the grid, which the fit cannot tell from a
    chain error; measure (README §2)."""
    with open(json_path) as fh:
        d = json.load(fh)
    design = float(d.get("tag_size_black_edge_mm", 90.0)) / 1e3
    world = d.get("world", "tag 200")
    world_tag = int(str(world).split()[-1])
    t = {}
    for k, v in d["tags"].items():
        rpy = v.get("rpy_deg", [0, 0, 0])
        if any(abs(float(a)) > 1e-9 for a in rpy):
            raise ValueError("sheet.py assumes every tag is laid in W's orientation; tag %s has rpy %s" % (k, rpy))
        x, y, z = [float(c) / 1e3 for c in v["t_W_mm"]]
        if abs(z) > 1e-9:
            raise ValueError("sheet.py assumes a planar sheet; tag %s has z %.3f mm" % (k, z * 1e3))
        t[int(k)] = np.array([sx * x, sy * y, 0.0])
    if world_tag not in t or np.linalg.norm(t[world_tag]) > 1e-12:
        raise ValueError("world tag %d must be in the layout at the origin" % world_tag)
    return Sheet(path=json_path, world_tag=world_tag, tag_size_m=float(tag_size_m or design), sx=float(sx),
                 sy=float(sy), t_W_m=t, family=str(d.get("tag_family", "tag36h11")), design_tag_size_m=design)


# ----------------------------------------------------------------------
# frames -> one corner set per tag
# ----------------------------------------------------------------------
@dataclass
class TagCorners:
    tag_id: int
    corners_px: np.ndarray                  # (4, 2) mean over the frames it was seen in
    n_frames: int
    std_px: float                           # rms corner scatter over those frames


def accumulate_frames(frames: List[Dict[int, np.ndarray]], min_fraction: float = 0.5) -> Dict[int, TagCorners]:
    """``frames``: one dict {tag id: (4,2) corners} per detection frame.
    A tag counts if it was seen in at least ``min_fraction`` of the frames
    (a tag flickering at the frame edge would otherwise enter with a
    biased mean); its corners are the per-frame mean, its scatter the rms
    over frames — > 0.3 px says the arm was still moving (handoff §7.3)."""
    n = len(frames)
    if n == 0:
        return {}
    seen: Dict[int, list] = {}
    for fr in frames:
        for k, c in fr.items():
            seen.setdefault(int(k), []).append(np.asarray(c, dtype=float).reshape(4, 2))
    out = {}
    for k, cs in seen.items():
        if len(cs) < max(1, int(math.ceil(min_fraction * n))):
            continue
        arr = np.array(cs)
        mean = arr.mean(axis=0)
        std = float(np.sqrt(((arr - mean) ** 2).sum(axis=2).mean())) if len(cs) > 1 else 0.0
        out[k] = TagCorners(tag_id=k, corners_px=mean, n_frames=len(cs), std_px=std)
    return out


# ----------------------------------------------------------------------
# multi-tag PnP
# ----------------------------------------------------------------------
@dataclass
class PnPResult:
    T_cam2W: np.ndarray
    tag_ids: List[int]
    rms_px: float
    per_tag_rms_px: Dict[int, float]
    n_points: int
    ambiguity_ratio: float        # IPPE 2nd / 1st solution reprojection error (inf when only one)
    ambiguity_angle_deg: float    # rotation between the two IPPE solutions (0 when only one)
    flags: List[str] = field(default_factory=list)


def multi_tag_pnp(corners_by_id: Dict[int, np.ndarray], sheet: Sheet, K, dist=None,
                  ambiguity_min_ratio: float = 1.2) -> PnPResult:
    """``T_cam2W`` from all detected sheet tags at once.

    Planar target -> ``SOLVEPNP_IPPE`` over every corner (initialisation,
    both mirror solutions), keep the one with the smaller reprojection
    error, refine with LM. With a single tag the two IPPE solutions can
    be close (the planar flip ambiguity); a ratio under
    ``ambiguity_min_ratio`` is flagged so the caller can reject the sample
    (handoff §5.3). ``dist`` = CameraInfo D for RAW corners (hand_cam);
    None for front_cam's ground-plane-corrected corners, which are a
    distortion-free virtual camera's pixels at the same K.
    """
    if cv2 is None:
        raise RuntimeError("multi_tag_pnp needs cv2")
    ids = sorted(int(k) for k in corners_by_id if int(k) in sheet.t_W_m)
    unknown = sorted(int(k) for k in corners_by_id if int(k) not in sheet.t_W_m)
    if not ids:
        raise RuntimeError("none of the detected tags %s is on the sheet %s" % (sorted(corners_by_id), sheet.ids))
    obj = np.vstack([sheet.corners_W(k) for k in ids])
    img = np.vstack([np.asarray(corners_by_id[k], dtype=float).reshape(4, 2) for k in ids])
    K = np.asarray(K, dtype=np.float64)
    D = np.zeros(5) if dist is None or len(dist) == 0 else np.asarray(dist, dtype=np.float64).ravel()
    n_sol, rvecs, tvecs, err = cv2.solvePnPGeneric(
        obj.reshape(-1, 1, 3), np.ascontiguousarray(img).reshape(-1, 1, 2), K, D,
        flags=cv2.SOLVEPNP_IPPE, reprojectionError=np.zeros((2, 1)))
    if n_sol < 1:
        raise RuntimeError("IPPE found no solution")
    err = np.asarray(err, dtype=float).ravel()[:n_sol]
    order = np.argsort(err)
    ratio, amb_deg = float("inf"), 0.0
    if n_sol > 1:
        ratio = float(err[order[1]] / max(err[order[0]], 1e-9))
        R0, R1 = cv2.Rodrigues(rvecs[order[0]])[0], cv2.Rodrigues(rvecs[order[1]])[0]
        amb_deg = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R0.T @ R1) - 1.0) / 2.0))))
    rvec, tvec = rvecs[order[0]], tvecs[order[0]]
    rvec, tvec = cv2.solvePnPRefineLM(obj, np.ascontiguousarray(img), K, D, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    res = np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = np.asarray(tvec, dtype=float).ravel()
    per = {k: float(np.sqrt((res[4 * i:4 * i + 4] ** 2).mean())) for i, k in enumerate(ids)}
    flags = []
    if len(ids) == 1 and ratio < ambiguity_min_ratio and amb_deg > 2.0:
        flags.append("single tag, IPPE flip ambiguity (ratio %.2f < %.2f, solutions %.1f deg apart)"
                     % (ratio, ambiguity_min_ratio, amb_deg))
    if T[2, 3] <= 0:
        flags.append("sheet behind the camera (z %.3f) — corner convention or ids wrong" % T[2, 3])
    if unknown:
        flags.append("ignored tags not on the sheet: %s" % unknown)
    return PnPResult(T_cam2W=T, tag_ids=ids, rms_px=float(np.sqrt((res ** 2).mean())), per_tag_rms_px=per,
                     n_points=len(ids) * 4, ambiguity_ratio=ratio, ambiguity_angle_deg=amb_deg, flags=flags)


def project_sheet(T_cam2W: np.ndarray, sheet: Sheet, K, dist=None, tag_ids=None) -> Dict[int, np.ndarray]:
    """Render the sheet's corners through a camera (test / synthetic aid)."""
    if cv2 is None:
        raise RuntimeError("project_sheet needs cv2")
    out = {}
    rvec = cv2.Rodrigues(np.ascontiguousarray(T_cam2W[:3, :3]))[0]
    D = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64).ravel()
    for k in (tag_ids if tag_ids is not None else sheet.ids):
        uv, _ = cv2.projectPoints(sheet.corners_W(k), rvec, T_cam2W[:3, 3].copy(), np.asarray(K, float), D)
        out[int(k)] = uv.reshape(4, 2)
    return out


def visible_tags(T_cam2W: np.ndarray, sheet: Sheet, K, image_wh: Tuple[int, int], dist=None, margin_px: float = 10.0):
    """Which sheet tags a camera at T_cam2W would image whole (all four
    corners inside the frame by ``margin_px`` and in front of the lens)."""
    w, h = image_wh
    out = []
    for k in sheet.ids:
        pc = (T_cam2W[:3, :3] @ sheet.corners_W(k).T).T + T_cam2W[:3, 3]
        if (pc[:, 2] <= 0.05).any():
            continue
        uv = project_sheet(T_cam2W, sheet, K, dist, [k])[k]
        if (uv[:, 0] >= margin_px).all() and (uv[:, 0] <= w - margin_px).all() and \
           (uv[:, 1] >= margin_px).all() and (uv[:, 1] <= h - margin_px).all():
            out.append(k)
    return out


def axis_tag(T_cam2W: np.ndarray, sheet: Sheet, tag_ids=None) -> int:
    """The sheet tag the camera's optical axis passes nearest — the
    natural 'tag A' / 'tag B' of a view for the T_A2B report."""
    Ti = invert_T(T_cam2W)
    c, a = Ti[:3, 3], Ti[:3, 2]
    if abs(a[2]) < 1e-9:
        foot = c
    else:
        foot = c + a * (-c[2] / a[2])
    cands = [int(k) for k in (tag_ids if tag_ids else sheet.ids)]
    return min(cands, key=lambda k: np.hypot(*(sheet.t_W_m[k][:2] - foot[:2])))
