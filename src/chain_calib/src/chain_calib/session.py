"""
session.py
==========
Sample persistence and the per-sample diagnostics the operator sees
while jogging the arm from view to view. No ROS here; ``scripts/
chain_calib.py`` feeds it.

A session directory holds
    samples.npz    labels, TCP poses, lift heights, T_hc2A, T_fc2B
    meta.yaml      tags, spacing, hand-eye file, K, date
    corrections.npz (after solve)

View coverage (``coverage``): the hand / base split needs the hand
camera to look at tag A from DIFFERENT DIRECTIONS — tilted about two
axes, plus spun about its own axis. The advice printed after every
capture counts what has been collected so far and names what is still
missing, so the operator knows when to stop.
"""
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
def save_samples(dir_, samples: List[ChainSample], meta: dict):
    os.makedirs(dir_, exist_ok=True)
    np.savez(os.path.join(dir_, "samples.npz"),
             labels=np.array([s.label for s in samples]),
             tcp=np.array([s.tcp_pose_mm_deg for s in samples], dtype=float).reshape(-1, 6),
             lift=np.array([s.lift_height_m for s in samples], dtype=float),
             T_hc2A=np.array([s.T_hc2A for s in samples], dtype=float).reshape(-1, 4, 4),
             T_fc2B=np.array([s.T_fc2B for s in samples], dtype=float).reshape(-1, 4, 4))
    with open(os.path.join(dir_, "meta.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)


def load_samples(dir_):
    """(samples, meta); an empty / missing session gives ([], {})."""
    npz = os.path.join(dir_, "samples.npz")
    if not os.path.exists(npz):
        return [], {}
    d = np.load(npz)
    meta = yaml.safe_load(open(os.path.join(dir_, "meta.yaml"))) or {}
    samples = [ChainSample(str(lab), list(map(float, tcp)), T1, T2, float(lift))
               for lab, tcp, lift, T1, T2 in zip(d["labels"], d["tcp"], d["lift"], d["T_hc2A"], d["T_fc2B"])]
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
    range_m: float          # hand_cam -> tag A along the optical axis
    tilt_deg: float         # optical axis vs the tag normal
    azimuth_deg: float      # direction the camera is displaced toward, in the TAG frame (0 = +x, 90 = +y)
    spin_deg: float         # camera rotation about its optical axis vs the tag's x axis
    xy_offset_mm: float     # tag centre off the optical axis


def describe_view(T_hc2A) -> ViewInfo:
    T = np.asarray(T_hc2A, dtype=float)
    z_cam_in_tag = invert_T(T)[:3, 2]          # camera optical axis, tag frame
    cam_pos_in_tag = invert_T(T)[:3, 3]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(z_cam_in_tag[2])))))
    az = math.degrees(math.atan2(cam_pos_in_tag[1], cam_pos_in_tag[0])) if tilt > 2.0 else float("nan")
    spin = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    return ViewInfo(range_m=float(T[2, 3]), tilt_deg=float(tilt), azimuth_deg=float(az),
                    spin_deg=float(spin), xy_offset_mm=float(np.hypot(T[0, 3], T[1, 3]) * 1e3))


def coverage(samples: List[ChainSample], min_tilt_deg=12.0):
    """What the set of views covers, and what is still missing for a
    determined hand / base split. Returns (ok: bool, lines: [str])."""
    views = [describe_view(s.T_hc2A) for s in samples]
    n = len(views)
    tilted = [v for v in views if v.tilt_deg >= min_tilt_deg]
    quads = set()
    for v in tilted:
        if not math.isnan(v.azimuth_deg):
            quads.add(int(((v.azimuth_deg + 45.0) % 360.0) // 90.0))     # 0:+x 1:+y 2:-x 3:-y
    spins = sorted({int(round(v.spin_deg / 45.0)) * 45 for v in views})
    spin_spread = (max(spins) - min(spins)) if spins else 0
    div = rotation_diversity_deg(samples) if n >= 2 else 0.0
    lines = ["%d sample(s); %d tilted >= %.0f deg from %d of 4 directions; spins %s; rotation diversity %.0f deg"
             % (n, len(tilted), min_tilt_deg, len(quads), spins, div)]
    need = []
    if n < 8:
        need.append("at least 8 samples (have %d)" % n)
    if len(tilted) < 4:
        need.append("at least 4 tilted views (have %d) — tilt the camera 15-25 deg" % len(tilted))
    missing = [name for k, name in enumerate(["tag +x", "tag +y", "tag -x", "tag -y"]) if k not in quads]
    if len(quads) < 3 and missing:
        need.append("tilts toward more directions (missing: %s)" % ", ".join(missing))
    if spin_spread < 60:
        need.append("a spin: rotate the camera ~90 deg about its own axis for a few views")
    ok = not need
    lines.append("READY to solve" if ok else "still needed: " + "; ".join(need))
    return ok, lines


def azimuth_word(v: ViewInfo):
    if math.isnan(v.azimuth_deg):
        return "straight down"
    return "tilted %.0f deg, camera displaced toward tag %s" % (
        v.tilt_deg, ["+x", "+y", "-x", "-y"][int(((v.azimuth_deg + 45.0) % 360.0) // 90.0)])


def new_meta(hand_tag, front_tag, spacing_m, frames, hand_eye_npz, extrinsics_note, K_front, K_hand):
    return dict(date=time.strftime("%Y-%m-%d %H:%M:%S"), hand_tag=int(hand_tag), front_tag=int(front_tag),
                spacing_m=float(spacing_m), frames_per_sample=int(frames), hand_eye_npz=str(hand_eye_npz),
                extrinsics_note=str(extrinsics_note),
                K_front=[float(v) for v in np.asarray(K_front).ravel()],
                K_hand=[float(v) for v in np.asarray(K_hand).ravel()], mode="operator-jogged capture")
