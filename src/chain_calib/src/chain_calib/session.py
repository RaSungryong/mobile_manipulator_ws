"""
session.py
==========
Sample persistence and the per-sample diagnostics the operator sees
while jogging the arm from view to view. No ROS here; ``scripts/
chain_calib.py`` feeds it.

A session directory holds
    samples.npz    labels, flange poses, lift heights, T_hc2W, T_fc2W
    corners.json   per sample the corner means each camera's PnP used
                   ({tag id: 4x[u, v]}) and the PnP rms — so `solve` can
                   re-solve with the print's measured scale
    meta.yaml      sheet file + scale, hand-eye file, front_cam frame,
                   K / D of both cameras, date
    corrections.npz (after solve)

View coverage (``coverage``): the hand / base split needs the hand
camera to look at the sheet from DIFFERENT DIRECTIONS — tilted about two
axes, plus spun about its own axis. The advice printed after every
capture counts what has been collected so far and names what is still
missing, so the operator knows when to stop.
"""
import json
import math
import os
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np
import yaml

from path_tag_locator.geometry import invert_T, pose_fr5_to_matrix_m

from .solver import ChainSample, rotation_diversity_deg


# ----------------------------------------------------------------------
# persistence
# ----------------------------------------------------------------------
def _corners_to_json(c):
    return None if c is None else {str(k): np.asarray(v, dtype=float).reshape(4, 2).tolist() for k, v in c.items()}


def _corners_from_json(c):
    return None if c is None else {int(k): np.asarray(v, dtype=float).reshape(4, 2) for k, v in c.items()}


def save_samples(dir_, samples: List[ChainSample], meta: dict):
    os.makedirs(dir_, exist_ok=True)
    np.savez(os.path.join(dir_, "samples.npz"),
             labels=np.array([s.label for s in samples]),
             tcp=np.array([s.tcp_pose_mm_deg for s in samples], dtype=float).reshape(-1, 6),
             lift=np.array([s.lift_height_m for s in samples], dtype=float),
             T_hc2W=np.array([s.T_hc2W for s in samples], dtype=float).reshape(-1, 4, 4),
             T_fc2W=np.array([s.T_fc2W for s in samples], dtype=float).reshape(-1, 4, 4),
             joints=np.array([(s.joints_deg if s.joints_deg is not None else [float("nan")] * 6) for s in samples],
                             dtype=float).reshape(-1, 6))
    with open(os.path.join(dir_, "meta.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)
    with open(os.path.join(dir_, "corners.json"), "w") as fh:
        json.dump({s.label: dict(hand=_corners_to_json(s.hand_corners), front=_corners_to_json(s.front_corners),
                                 hand_rms_px=float(s.hand_rms_px), front_rms_px=float(s.front_rms_px))
                   for s in samples}, fh, indent=1)


def load_samples(dir_):
    """(samples, meta); an empty / missing session gives ([], {})."""
    npz = os.path.join(dir_, "samples.npz")
    if not os.path.exists(npz):
        return [], {}
    d = np.load(npz)
    meta = yaml.safe_load(open(os.path.join(dir_, "meta.yaml"))) or {}
    if "T_hc2W" not in d or meta.get("mode") != "sheet":
        raise ValueError("%s is a session of the removed two-tag mode (2026-09-15..18); the sheet is the "
                         "only truth now — start a new session directory" % dir_)
    samples = [ChainSample(str(lab), list(map(float, tcp)), T1, T2, float(lift))
               for lab, tcp, lift, T1, T2 in zip(d["labels"], d["tcp"], d["lift"], d["T_hc2W"], d["T_fc2W"])]
    if "joints" in d:                                   # sessions before 2026-09-21 have none
        for s, q in zip(samples, d["joints"]):
            s.joints_deg = None if np.any(np.isnan(q)) else [float(v) for v in q]
    extra = json.load(open(os.path.join(dir_, "corners.json")))
    for s in samples:
        e = extra.get(s.label)
        if e:
            s.hand_corners = _corners_from_json(e.get("hand"))
            s.front_corners = _corners_from_json(e.get("front"))
            s.hand_rms_px = float(e.get("hand_rms_px", float("nan")))
            s.front_rms_px = float(e.get("front_rms_px", float("nan")))
    return samples, meta


def next_label(samples):
    n = 0
    for s in samples:
        try:
            n = max(n, int(str(s.label).lstrip("v")) + 1)
        except ValueError:
            pass
    return "v%02d" % n


# ----------------------------------------------------------------------
# per-sample view description
# ----------------------------------------------------------------------
@dataclass
class ViewInfo:
    range_m: float          # hand_cam -> the tag plane along the optical axis
    tilt_deg: float         # optical axis vs the tag normal
    azimuth_deg: float      # direction the camera is displaced toward, in the TAG frame (0 = +x, 90 = +y)
    spin_deg: float         # camera rotation about its optical axis vs the tag's x axis
    xy_offset_mm: float     # nearest tag centre off the optical axis (in the plane)


def describe_view(T_cam2W, tag_xy=None) -> ViewInfo:
    """View geometry from T_cam2W, W's z = 0 plane holding the tags.
    ``tag_xy``: the centres of the tags this camera saw (the offset is to
    the nearest of them); default W's origin."""
    T = np.asarray(T_cam2W, dtype=float)
    Ti = invert_T(T)
    axis, cam = Ti[:3, 2], Ti[:3, 3]           # optical axis and camera position, target frame
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(axis[2])))))
    if abs(axis[2]) > 1e-6:
        rng = -cam[2] / axis[2]                # along the axis to the plane
        foot = cam + axis * rng
    else:
        rng, foot = float(T[2, 3]), cam
    pts = np.zeros((1, 2)) if tag_xy is None else np.asarray(list(tag_xy), dtype=float).reshape(-1, 2)
    off = float(np.min(np.hypot(pts[:, 0] - foot[0], pts[:, 1] - foot[1]))) * 1e3
    d = cam[:2] - foot[:2]
    az = math.degrees(math.atan2(d[1], d[0])) if tilt > 2.0 else float("nan")
    spin = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    return ViewInfo(range_m=float(abs(rng)), tilt_deg=float(tilt), azimuth_deg=float(az),
                    spin_deg=float(spin), xy_offset_mm=off)


def sample_view(s: ChainSample, sheet=None) -> ViewInfo:
    """describe_view of the hand camera for one sample."""
    if sheet is None or s.hand_corners is None:
        return describe_view(s.T_hc2W)
    return describe_view(s.T_hc2W, [sheet.t_W_m[k][:2] for k in s.hand_corners if k in sheet.t_W_m])


def coverage(samples: List[ChainSample], min_tilt_deg=12.0, sheet=None):
    """What the set of views covers, and what is still missing for a
    determined hand / base split. Returns (ok: bool, lines: [str]).

    Identifiability needs rotations about two non-parallel axes: tilts
    toward the sheet's +-x AND +-y are enough. A spin about the optical
    axis adds a third axis and conditions the fit better, but it is only
    REQUIRED when the tilts cover fewer than three directions — a 90 deg
    wrist spin is a collision risk with the arm close to the body
    (user, 2026-09-18), so it is advised in small steps (20-30 deg) and
    never demanded when the tilts already span the plane."""
    views = [sample_view(s, sheet) for s in samples]
    n = len(views)
    tilted = [v for v in views if v.tilt_deg >= min_tilt_deg]
    quads = set()
    for v in tilted:
        if not math.isnan(v.azimuth_deg):
            quads.add(int(((v.azimuth_deg + 45.0) % 360.0) // 90.0))     # 0:+x 1:+y 2:-x 3:-y
    spins = sorted({int(round(v.spin_deg / 15.0)) * 15 for v in views})
    spin_spread = (max(spins) - min(spins)) if spins else 0
    div = rotation_diversity_deg(samples) if n >= 2 else 0.0
    lines = ["%d sample(s); %d tilted >= %.0f deg from %d of 4 directions; spins %s; rotation diversity %.0f deg"
             % (n, len(tilted), min_tilt_deg, len(quads), spins, div)]
    need = []
    if n < 8:
        need.append("at least 8 samples (have %d)" % n)
    if len(tilted) < 4:
        need.append("at least 4 tilted views (have %d) — tilt the camera 15-25 deg" % len(tilted))
    missing = [name for k, name in enumerate(["sheet +x", "sheet +y", "sheet -x", "sheet -y"]) if k not in quads]
    both_axes = ({0, 2} & quads) and ({1, 3} & quads)
    if len(quads) < 3 and missing:
        need.append("tilts toward more directions (missing: %s)" % ", ".join(missing))
    if spin_spread < 30 and not (both_axes and len(quads) >= 3 and len(tilted) >= 6):
        need.append("a spin: rotate the camera 20-30 deg about its own axis for a few views (or tilt toward all four directions instead)")
    ok = not need
    lines.append("READY to solve" if ok else "still needed: " + "; ".join(need))
    return ok, lines


def azimuth_word(v: ViewInfo):
    if math.isnan(v.azimuth_deg):
        return "straight down"
    return "tilted %.0f deg, camera displaced toward sheet %s" % (
        v.tilt_deg, ["+x", "+y", "-x", "-y"][int(((v.azimuth_deg + 45.0) % 360.0) // 90.0)])


def new_meta(sheet, frames, hand_eye_npz, extrinsics_note, front_cam_frame, K_front, D_front, K_hand, D_hand):
    """Session header. The print scale lives here (sx, sy, tag_size_m) and
    is what `solve` re-solves the stored corners with."""
    return dict(date=time.strftime("%Y-%m-%d %H:%M:%S"), mode="sheet",
                sheet_json=str(os.path.abspath(sheet.path)), sheet_world_tag=int(sheet.world_tag),
                sheet_sx=float(sheet.sx), sheet_sy=float(sheet.sy), sheet_tag_size_m=float(sheet.tag_size_m),
                frames_per_sample=int(frames), hand_eye_npz=str(hand_eye_npz),
                extrinsics_note=str(extrinsics_note), front_cam_frame=str(front_cam_frame),
                K_front=[float(v) for v in np.asarray(K_front).ravel()],
                D_front=[float(v) for v in np.asarray(D_front if D_front is not None else []).ravel()],
                K_hand=[float(v) for v in np.asarray(K_hand).ravel()],
                D_hand=[float(v) for v in np.asarray(D_hand if D_hand is not None else []).ravel()])
