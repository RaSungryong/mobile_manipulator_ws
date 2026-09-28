#!/usr/bin/env python3
"""
sheet_sweep.py — AUTOMATIC chain-calibration collection over the A0 sheet:
plan hand_cam views over a whitelist of grid tags, drive the arm through
them (bounded rotations, safety rules, chunked MoveL), capture a sample at
each, save as it goes. The result is an ordinary chain_calib session that
`chain_calib.py solve` and `arm_offsets.py` read unchanged.

    rosrun chain_calib sheet_sweep.py --sx .. --sy .. --tag-size .. --dry-run     # plan only, nothing moves
    rosrun chain_calib sheet_sweep.py --sx .. --sy .. --tag-size ..               # plan, confirm once, run
    rosrun chain_calib sheet_sweep.py ... --tags 301,303,305,307,309 --heights 0.45,0.55 --tilt 15 --spins 0,40,-40

Default session directory: log/chain_calib/<YYYYMMDD>_sheet_auto (samples.npz,
corners.json, meta.yaml — plus this script's sweep_plan.csv and sweep_log.csv;
`solve` adds corrections.npz there).

What the user asked for (2026-09-28): hand_cam uses ONLY tags 301 303 305
307 309 (the whitelist goes into the session's meta as `hand_tags`, so
`solve` ignores the other column too), the collection is automatic, and
the rotations stay small — every commanded orientation is within
--max-rot (60 deg) of the START orientation, tilts are --tilt (15 deg),
spins are relative to the CURRENT camera spin (0 / +40 / -40 by default).

Views per tag (7): straight down at the first height with three spins
across the tags, four tilts (sheet +x / -x / +y / -y) at that height, and
straight down + one tilt at the second height. 5 tags x 7 = 35 views,
~6-8 min. That covers what `solve` needs (tilts toward four directions,
spin spread) and what `arm_offsets.py` needs (30-40 different arm
configurations over the tags, two heights, spins).

Safety, evaluated for EVERY target and EVERY MoveL chunk between targets
(a view is skipped, never adjusted, when a rule fails):
  * the flange and the vision tip stay >= --min-clearance (0.12 m) above
    the sheet plane (tf_chain T_ee2tip);
  * verify_chain's body-clearance line: flange below the arm-base plane
    needs >= --min-reach-low (0.65 m) of horizontal reach (a 0.40 m one
    COLLIDED on 2026-09-21) — a view refused at the first height is
    retried 5 / 10 cm higher;
  * flange reach 0.25..1.25 m, flange xy within --max-xy (0.70 m) of the
    start;
  * orientation within --max-rot of the start orientation; a tilt that
    would displace the camera TOWARD the arm base (the flange folds in
    toward the body) is reduced to --body-tilt (8 deg);
  * hand_cam is PREDICTED to image at least --min-tags (2) whitelisted
    tags whole (a two-tag PnP is the minimum that has no flip ambiguity);
  * moves are straight MoveL chunks of <= 0.20 m / 30 deg, both ends
    checked; when the straight line between two views fails a rule the
    route goes up 0.10 m first, across, and down (each leg checked);
    a failed move skips the view; Ctrl-C stops after the current
    move and LEAVES the arm where it is (no return move).
The first target must be within --max-first-move (0.50 m) of the current
flange: park hand_cam over the MIDDLE tag of the whitelist (305) at
~0.45 m, looking straight down, before starting — the flange-xy window
(--max-xy 0.70 m) is measured from there, and the column is 0.60 m long. The base must not move during the run
(front_cam's view of tag 200 is re-read at every view and a drift > 3 mm
is flagged). path_tag_locator.launch must be DOWN (second arm commander).
Hand on the e-stop.
"""
import argparse
import csv
import math
import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))     # NOT _HERE: scripts/chain_calib.py would shadow the package

from chain_calib import sheet as SH                                    # noqa: E402
from chain_calib import solver as CC                                   # noqa: E402
from chain_calib.session import coverage, load_samples, new_meta, next_label, save_samples  # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                  # noqa: E402
from path_tag_locator.geometry import invert_T, matrix_m_to_pose_fr5, pose_fr5_to_matrix_m  # noqa: E402
from apriltag_nav import tf_chain as TC                                # noqa: E402

import importlib.util                                                  # noqa: E402
_spec = importlib.util.spec_from_file_location("chain_calib_tool", os.path.join(_HERE, "chain_calib.py"))
tool = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(tool)   # platform(), Session, sheet_from_args
_vspec = importlib.util.spec_from_file_location("verify_chain_tool", os.path.join(_HERE, "verify_chain.py"))
vc = importlib.util.module_from_spec(_vspec); _vspec.loader.exec_module(vc)     # body_clearance_ok(), Rz()

DEFAULT_TAGS = [301, 303, 305, 307, 309]


# ----------------------------------------------------------------------
# configuration and the view list
# ----------------------------------------------------------------------
@dataclass
class SweepCfg:
    tags: List[int] = field(default_factory=lambda: list(DEFAULT_TAGS))
    heights_m: List[float] = field(default_factory=lambda: [0.45, 0.55])
    tilt_deg: float = 15.0
    spins_rel_deg: List[float] = field(default_factory=lambda: [0.0, 40.0, -40.0])
    max_rot_from_start_deg: float = 60.0      # the user's bound (2026-09-28): rx/ry/rz never far from the start
    max_tilt_deg: float = 20.0
    body_tilt_deg: float = 8.0                # tilt used when the camera would be displaced TOWARD the arm base
    max_spin_deg: float = 60.0
    min_clearance_m: float = 0.12             # flange + vision tip above the sheet plane
    min_tags: int = 2                         # whitelisted tags predicted whole in the image
    max_flange_reach_m: float = 1.25
    min_flange_reach_m: float = 0.25
    max_xy_from_start_m: float = 0.70
    max_step_m: float = 0.20                  # MoveL chunk clamps
    max_step_deg: float = 30.0
    via_lift_m: float = 0.10                  # detour height when the straight path between views fails a rule
    min_reach_low_m: float = vc.MIN_REACH_LOW_M
    low_z_margin_m: float = 0.02              # the body-clearance line applies up to this far ABOVE the base plane too
    height_fallback_m: List[float] = field(default_factory=lambda: [0.0, 0.05, 0.10, 0.15])
    image_wh: tuple = (640, 480)
    margin_px: float = 12.0


@dataclass
class View:
    label: str
    tag: int
    height_m: float
    tilt_deg: float
    azimuth_deg: float                        # in the SHEET frame: 0 = +x, 90 = +y (coverage's quadrants)
    spin_rel_deg: float
    spin_deg: float = 0.0                     # absolute camera spin vs W x
    T_ab2ee: Optional[np.ndarray] = None
    pose: Optional[list] = None               # flange [x y z rx ry rz] mm / deg
    reach_m: float = float("nan")
    rot_from_start_deg: float = float("nan")
    predicted_tags: List[int] = field(default_factory=list)
    reason: Optional[str] = None              # set when rejected


def view_list(cfg: SweepCfg) -> List[View]:
    """Seven views per tag, spins and the second-height tilt direction
    rotating with the tag index so consecutive tags differ."""
    out = []
    sp = list(cfg.spins_rel_deg) or [0.0]
    h1 = cfg.heights_m[0]
    h2 = cfg.heights_m[1] if len(cfg.heights_m) > 1 else None
    azs = [0.0, 180.0, 90.0, 270.0]
    for i, tag in enumerate(cfg.tags):
        s = lambda j: sp[(i + j) % len(sp)]
        out.append(View("t%d_h%.0f_down_s%+.0f" % (tag, h1 * 100, s(0)), tag, h1, 0.0, 0.0, s(0)))
        for j, az in enumerate(azs):
            out.append(View("t%d_h%.0f_a%.0f_s%+.0f" % (tag, h1 * 100, az, s(j + 1)), tag, h1, cfg.tilt_deg, az, s(j + 1)))
        if h2 is not None:
            out.append(View("t%d_h%.0f_down_s%+.0f" % (tag, h2 * 100, s(2)), tag, h2, 0.0, 0.0, s(2)))
            az = azs[i % 4]
            out.append(View("t%d_h%.0f_a%.0f_s%+.0f" % (tag, h2 * 100, az, s(3)), tag, h2, cfg.tilt_deg, az, s(3)))
    return out


# ----------------------------------------------------------------------
# geometry
# ----------------------------------------------------------------------
def _R_axis(axis, a_rad):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12 or abs(a_rad) < 1e-12:
        return np.eye(3)
    x, y, z = axis / n
    c, s = math.cos(a_rad), math.sin(a_rad)
    C = 1.0 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def camera_pose_W(tag_xy, h, tilt_deg, az_deg, spin_deg):
    """T_W2hc: hand_cam ``h`` above the sheet point ``tag_xy`` (W, m) with the
    tag on its optical axis, displaced toward sheet azimuth ``az_deg`` by
    ``tilt_deg``, spun ``spin_deg`` about its own axis. ``spin`` is the
    session's convention (session.describe_view: atan2 of T_hc2W's first
    column, i.e. the SHEET's rotation in the camera), so the camera in W
    carries Rz(-spin). W z points INTO the paper: the square view is
    R = Rz(-spin), t = (x, y, -h)."""
    az = math.radians(az_deg)
    axis = (math.sin(az), -math.cos(az), 0.0)           # rotating (0,0,-h) about it moves the camera toward (cos az, sin az)
    R_tilt = _R_axis(axis, math.radians(tilt_deg))
    T = np.eye(4)
    T[:3, :3] = R_tilt @ vc.Rz(-spin_deg)
    T[:3, 3] = np.array([tag_xy[0], tag_xy[1], 0.0]) + R_tilt @ np.array([0.0, 0.0, -float(h)])
    return T


def rot_angle_deg(R_a, R_b):
    E = np.asarray(R_a).T @ np.asarray(R_b)
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0))))


def tool_points_m():
    """Flange origin + the vision tip (tf_chain T_ee2tip) — the lowest parts of the tool."""
    return [np.zeros(3), np.asarray(TC.tip_offset_mm(), dtype=float) / 1e3]


@dataclass
class Rules:
    """Everything a target / chunk endpoint is checked against."""
    up_ab: np.ndarray                          # unit normal of the sheet plane, pointing at the camera side (ab frame)
    origin_ab: np.ndarray                      # a point of the plane (W origin in ab)
    tool_pts: list
    R_start: np.ndarray
    xy_start: np.ndarray
    cfg: SweepCfg


def make_rules(T_ab2W, T_ab2ee_start, cfg: SweepCfg) -> Rules:
    up = -np.asarray(T_ab2W)[:3, 2]                          # W z into the paper -> up is -z
    return Rules(up / np.linalg.norm(up), np.asarray(T_ab2W)[:3, 3].copy(), tool_points_m(),
                 np.asarray(T_ab2ee_start)[:3, :3].copy(), np.asarray(T_ab2ee_start)[:2, 3].copy(), cfg)


def check_T(T_ab2ee, rules: Rules, what="target"):
    """None when the flange pose passes every rule, else the reason."""
    cfg = rules.cfg
    for i, p in enumerate(rules.tool_pts):
        P = T_ab2ee[:3, :3] @ p + T_ab2ee[:3, 3]
        h = float(np.dot(rules.up_ab, P - rules.origin_ab))
        if h < cfg.min_clearance_m:
            return "%s: %s only %.0f mm above the sheet (< %.0f)" % (what, "flange" if i == 0 else "vision tip", h * 1e3, cfg.min_clearance_m * 1e3)
    pose = matrix_m_to_pose_fr5(T_ab2ee)
    ok_b, why_b = vc.body_clearance_ok(pose, cfg.min_reach_low_m)
    if not ok_b:
        return "%s: body clearance — %s" % (what, why_b)
    r = float(np.linalg.norm(T_ab2ee[:3, 3]))
    reach_xy = float(np.hypot(T_ab2ee[0, 3], T_ab2ee[1, 3]))
    if T_ab2ee[2, 3] < cfg.low_z_margin_m and reach_xy < cfg.min_reach_low_m:
        # verify_chain's line is a step at z = 0; a pose a few mm above it at short reach has
        # no margin against it, so the sweep applies the same rule up to low_z_margin_m
        return "%s: flange only %.0f mm above the arm-base plane at %.2f m reach (line + %.0f mm margin)" % (
            what, T_ab2ee[2, 3] * 1e3, reach_xy, cfg.low_z_margin_m * 1e3)
    if r > cfg.max_flange_reach_m:
        return "%s: reach %.2f m > %.2f" % (what, r, cfg.max_flange_reach_m)
    if r < cfg.min_flange_reach_m:
        return "%s: reach %.2f m < %.2f" % (what, r, cfg.min_flange_reach_m)
    dxy = float(np.linalg.norm(T_ab2ee[:2, 3] - rules.xy_start))
    if dxy > cfg.max_xy_from_start_m:
        return "%s: flange xy %.2f m from the start (> %.2f)" % (what, dxy, cfg.max_xy_from_start_m)
    a = rot_angle_deg(rules.R_start, T_ab2ee[:3, :3])
    if a > cfg.max_rot_from_start_deg + 1e-6:
        return "%s: orientation %.1f deg from the start (> %.0f)" % (what, a, cfg.max_rot_from_start_deg)
    return None


def plan(T_ab2W, Hc, T_ab2ee_start, sheet, K_hand, D_hand, cfg: SweepCfg, spin_cur_deg):
    """Flange targets for the view list, rules applied, nearest-first.
    Returns (accepted, rejected); every View carries pose or reason."""
    rules = make_rules(T_ab2W, T_ab2ee_start, cfg)
    if abs(cfg.tilt_deg) > cfg.max_tilt_deg:
        raise SystemExit("--tilt %.0f exceeds the %.0f deg limit" % (cfg.tilt_deg, cfg.max_tilt_deg))
    if any(abs(s) > cfg.max_spin_deg for s in cfg.spins_rel_deg):
        raise SystemExit("a --spins value exceeds the %.0f deg limit" % cfg.max_spin_deg)
    base_W = invert_T(np.asarray(T_ab2W))[:2, 3]                     # the arm base origin, xy in W
    accepted, rejected = [], []
    for v in view_list(cfg):
        if v.tag not in sheet.t_W_m:
            v.reason = "tag %d is not on the sheet" % v.tag; rejected.append(v); continue
        xy = sheet.t_W_m[v.tag][:2]
        v.spin_deg = spin_cur_deg + v.spin_rel_deg
        if v.tilt_deg > 0:
            # a tilt displaces the camera toward the azimuth; toward the ARM BASE that folds the
            # flange in toward the body (README §3-2: 5-10 deg only on that side) — use body_tilt_deg
            d = np.array([math.cos(math.radians(v.azimuth_deg)), math.sin(math.radians(v.azimuth_deg))])
            to_base = base_W - xy
            if np.dot(d, to_base / max(np.linalg.norm(to_base), 1e-9)) > 0.5:
                v.tilt_deg = min(v.tilt_deg, cfg.body_tilt_deg)
                v.label += "_body"
        last = None
        for dh in cfg.height_fallback_m:
            h = v.height_m + dh
            T_W2hc = camera_pose_W(xy, h, v.tilt_deg, v.azimuth_deg, v.spin_deg)
            T_ab2ee = np.asarray(T_ab2W) @ T_W2hc @ np.asarray(Hc)
            why = check_T(T_ab2ee, rules)
            if why is None:
                vis = SH.visible_tags(invert_T(T_W2hc), sheet, K_hand, cfg.image_wh, D_hand, cfg.margin_px)
                allowed = [k for k in vis if k in set(cfg.tags)]
                if len(allowed) < cfg.min_tags:
                    why = "predicted %d whitelisted tag(s) in view %s (need %d)" % (len(allowed), allowed, cfg.min_tags)
                else:
                    v.predicted_tags = allowed
            if why is None:
                v.T_ab2ee, v.pose = T_ab2ee, matrix_m_to_pose_fr5(T_ab2ee)
                v.reach_m = float(np.linalg.norm(T_ab2ee[:3, 3]))
                v.rot_from_start_deg = rot_angle_deg(rules.R_start, T_ab2ee[:3, :3])
                if dh:
                    v.height_m = h
                    v.label += "_up%.0f" % (dh * 100)
                break
            last = why
        if v.T_ab2ee is None:
            v.reason = last; rejected.append(v)
        else:
            accepted.append(v)
    # nearest-first from the start (translation + 0.5 m per rad of rotation)
    ordered, T_cur, pool = [], np.asarray(T_ab2ee_start), list(accepted)
    while pool:
        i = min(range(len(pool)), key=lambda j: float(np.linalg.norm(pool[j].T_ab2ee[:3, 3] - T_cur[:3, 3]))
                + 0.5 * math.radians(rot_angle_deg(T_cur[:3, :3], pool[j].T_ab2ee[:3, :3])))
        v = pool.pop(i); ordered.append(v); T_cur = v.T_ab2ee
    return ordered, rejected, rules


def chunks(T_from, T_to, cfg: SweepCfg):
    """Intermediate + final poses of a straight move split into MoveL
    chunks of <= max_step_m / max_step_deg (translation linear, rotation
    by a scaled rotation vector)."""
    T_from, T_to = np.asarray(T_from, float), np.asarray(T_to, float)
    d = float(np.linalg.norm(T_to[:3, 3] - T_from[:3, 3]))
    E = T_from[:3, :3].T @ T_to[:3, :3]
    rv = CC._rotvec_from_R(E)
    a = math.degrees(np.linalg.norm(rv))
    n = max(1, int(math.ceil(d / cfg.max_step_m - 1e-9)), int(math.ceil(a / cfg.max_step_deg - 1e-9)))
    out = []
    for i in range(1, n + 1):
        f = i / float(n)
        T = np.eye(4)
        T[:3, :3] = T_from[:3, :3] @ _R_axis(rv, np.linalg.norm(rv) * f) if a > 1e-9 else T_to[:3, :3].copy()
        T[:3, 3] = T_from[:3, 3] + f * (T_to[:3, 3] - T_from[:3, 3])
        out.append(T)
    out[-1] = T_to.copy()
    return out


def _checked(path, rules):
    for k, T in enumerate(path):
        why = check_T(T, rules, "chunk %d/%d" % (k + 1, len(path)))
        if why:
            return why
    return None


def safe_path(T_from, T_to, rules: Rules, cfg: SweepCfg):
    """MoveL chunks from T_from to T_to with every endpoint checked. The
    straight line first; when a chunk of it fails a rule (typically the
    body-clearance line, crossed by a straight move between two low
    views), the route goes UP ``via_lift_m`` at the current xy, across at
    that height, and down — each leg chunked and checked. Returns
    (path, None), (["via", path], None), or (None, reason)."""
    direct = chunks(T_from, T_to, cfg)
    why = _checked(direct, rules)
    if why is None:
        return direct, None
    up_a, up_b = np.asarray(T_from, float).copy(), np.asarray(T_to, float).copy()
    up_a[2, 3] += cfg.via_lift_m; up_b[2, 3] += cfg.via_lift_m
    via = chunks(T_from, up_a, cfg) + chunks(up_a, up_b, cfg) + chunks(up_b, T_to, cfg)
    why2 = _checked(via, rules)
    if why2 is None:
        return ["via", via], None
    return None, "%s; via +%.2f m also fails: %s" % (why, cfg.via_lift_m, why2)


# ----------------------------------------------------------------------
# execution — every ROS / arm / camera access is a callable, so the
# offline check drives the same loop against a fake
# ----------------------------------------------------------------------
class Stop(Exception):
    pass


def execute(targets, rules: Rules, cfg: SweepCfg, *, get_T, move, capture, on_sample, settle, log,
            return_to=None, front_ref_t=None, max_recapture=1):
    """Drive the accepted views. ``get_T()`` -> current flange 4x4 (m);
    ``move(pose_mm_deg)`` blocks, raises on failure; ``capture(label)`` ->
    (ChainSample, info) or raises; ``on_sample(sample, info, view)`` stores
    it; ``settle()`` waits after a move. Returns a summary dict. Ctrl-C
    stops after the current move / capture and leaves the arm."""
    n_ok = n_move_fail = n_cap_fail = 0
    stopped = False
    try:
        for i, v in enumerate(targets):
            log("-> %d/%d %s  tag %d, h %.2f, tilt %.0f toward %s, spin %+.0f (%.0f deg from the start), flange %s"
                % (i + 1, len(targets), v.label, v.tag, v.height_m, v.tilt_deg,
                   "-" if v.tilt_deg == 0 else ["+x", "+y", "-x", "-y"][int(((v.azimuth_deg + 45) % 360) // 90)],
                   v.spin_rel_deg, v.rot_from_start_deg, ["%.1f" % p for p in v.pose]))
            T_cur = get_T()
            path, bad = safe_path(T_cur, v.T_ab2ee, rules, cfg)
            if bad:
                log("   SKIPPED (path): %s" % bad); v.reason = bad; n_move_fail += 1; continue
            if path[0] == "via":
                log("   (straight path fails a rule — going up %.2f m first)" % cfg.via_lift_m); path = path[1]
            try:
                for T in path:
                    move(matrix_m_to_pose_fr5(T))
            except KeyboardInterrupt:
                raise
            except Exception as e:
                log("   move failed: %s — skipping this view" % e); v.reason = "move failed: %s" % e; n_move_fail += 1; continue
            settle()
            sample = info = None
            for attempt in range(max_recapture + 1):
                try:
                    sample, info = capture(v.label)
                except KeyboardInterrupt:
                    raise
                except Exception as e:
                    log("   capture failed: %s" % e); sample = None
                    if attempt < max_recapture:
                        settle(); continue
                    break
                scatter = max([t.std_px for t in info["hand_tags"].values()] + [t.std_px for t in info["front_tags"].values()])
                n_allowed = len(info["hand_tags"])
                if n_allowed < cfg.min_tags:
                    log("   hand_cam sees only %s of the whitelist" % sorted(info["hand_tags"]))
                    sample = None; break
                if scatter > 0.3 and attempt < max_recapture:
                    log("   corner scatter %.2f px — arm still settling, capturing again" % scatter); settle(); continue
                break
            if sample is None:
                v.reason = "capture failed"; n_cap_fail += 1; continue
            if front_ref_t is not None:
                drift = float(np.linalg.norm(info["front"].T_cam2W[:3, 3] - front_ref_t)) * 1e3
                if drift > 3.0:
                    log("   ! front_cam's tag 200 moved %.1f mm since the start — did the base move? (recorded anyway)" % drift)
            on_sample(sample, info, v)
            n_ok += 1
        if return_to is not None:
            log("-> returning to the start pose")
            path, bad = safe_path(get_T(), return_to, rules, cfg)
            if bad:
                log("   return path fails a rule (%s) — the arm stays where it is" % bad)
            else:
                for T in (path[1] if path[0] == "via" else path):
                    move(matrix_m_to_pose_fr5(T))
    except KeyboardInterrupt:
        stopped = True
        log("\n(interrupted — stopped after the current step; the arm stays where it is)")
    return dict(ok=n_ok, move_failed=n_move_fail, capture_failed=n_cap_fail, stopped=stopped)


def float_list(text):
    return [float(v) for v in str(text).replace(";", ",").split(",") if v.strip()]


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=None,
                    help="session directory (default log/chain_calib/<YYYYMMDD>_sheet_auto; an existing one is continued)")
    ap.add_argument("--tags", type=tool.int_list, default=list(DEFAULT_TAGS), metavar="ID,ID,..",
                    help="hand_cam tag whitelist AND the tags viewed (comma-separated; default 301,303,305,307,309)")
    ap.add_argument("--heights", type=float_list, default=[0.45, 0.55], metavar="M,M", help="hand_cam heights above the sheet (m; default 0.45,0.55)")
    ap.add_argument("--tilt", type=float, default=15.0, help="tilt of the tilted views (deg, <= 20)")
    ap.add_argument("--body-tilt", type=float, default=8.0, help="tilt when the camera would be displaced toward the arm base (deg)")
    ap.add_argument("--spins", type=float_list, default=[0.0, 40.0, -40.0], metavar="DEG,DEG,..",
                    help="camera spins RELATIVE to the current one (deg, |s| <= 60; default 0,40,-40)")
    ap.add_argument("--max-rot", type=float, default=60.0, help="max orientation change from the start pose (deg)")
    ap.add_argument("--min-clearance", type=float, default=0.12)
    ap.add_argument("--min-tags", type=int, default=2)
    ap.add_argument("--max-xy", type=float, default=0.70, help="flange xy excursion from the start (m)")
    ap.add_argument("--min-reach-low", type=float, default=vc.MIN_REACH_LOW_M)
    ap.add_argument("--max-first-move", type=float, default=0.50)
    ap.add_argument("--settle", type=float, default=0.8, help="s at rest before each capture")
    ap.add_argument("--vel", type=float, default=20.0, help="MoveL velocity (percent)")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--dry-run", action="store_true", help="plan and print; move nothing")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation")
    ap.add_argument("--no-return", action="store_true", help="do not move back to the start pose at the end")
    ap.add_argument("--force", action="store_true", help="run even with the calibration nodes registered")
    ap.add_argument("--hand-eye", default=None)
    ap.add_argument("--front-rotation", choices=["level", "measured"], default="level")
    ap.add_argument("--sheet-json", default=None); ap.add_argument("--sx", type=float, default=None)
    ap.add_argument("--sy", type=float, default=None); ap.add_argument("--tag-size", type=float, default=None)
    args = ap.parse_args()
    args.hand_tags = args.tags

    import rospy
    rospy.init_node("sheet_sweep", anonymous=True)
    if not args.force:
        try:
            import rosnode
            others = [n for n in rosnode.get_node_names() if n in ("/map_calibrator", "/path_tag_locator", "/handeye_calib")]
            if others:
                sys.exit("%s registered — a second arm commander; stop path_tag_locator.launch first (or --force)" % others)
        except SystemExit:
            raise
        except Exception:
            pass
    out_dir = args.dir or os.path.join(tool.DEFAULT_OUT, time.strftime("%Y%m%d") + "_sheet_auto")
    samples, meta = load_samples(out_dir)
    cfg_l, ext, H = tool.platform(hand_eye=args.hand_eye)
    sheet = tool.sheet_from_args(args, meta)
    if meta:
        if os.path.abspath(sheet.path) != meta.get("sheet_json"):
            sys.exit("%s: this session was started with sheet %s" % (out_dir, meta.get("sheet_json")))
        if meta.get("front_cam_frame") != ext.front_cam_frame:
            sys.exit("%s: this session's front_cam detections were %s, now %s" % (out_dir, meta.get("front_cam_frame"), ext.front_cam_frame))
    tags = tool.hand_tags_for(args, meta)
    cfg = SweepCfg(tags=list(tags), heights_m=list(args.heights), tilt_deg=args.tilt, spins_rel_deg=list(args.spins),
                   max_rot_from_start_deg=args.max_rot, min_clearance_m=args.min_clearance, min_tags=args.min_tags,
                   body_tilt_deg=args.body_tilt,
                   max_xy_from_start_m=args.max_xy, min_reach_low_m=args.min_reach_low)
    print("session: %s (%d sample(s) so far)" % (out_dir, len(samples)))
    print("extrinsics: %s\n%s" % (ext.note, sheet.describe()))
    print("hand_cam tags: %s; heights %s m; tilt %.0f deg; spins %s deg (relative); max rotation from start %.0f deg"
          % (cfg.tags, cfg.heights_m, cfg.tilt_deg, cfg.spins_rel_deg, cfg.max_rot_from_start_deg))

    S = tool.Session(cfg_l, sheet, ext.front_cam_frame, args.frames, hand_tags=cfg.tags); S.ext = ext
    S.arm.default_vel = float(args.vel)
    if not meta:
        meta = new_meta(sheet, args.frames, tool._resolve(cfg_l.hand_eye_npz), ext.note, ext.front_cam_frame,
                        S.K_front, S.D_front, S.K_hand, S.D_hand)
        meta["hand_cam_intrinsics"] = S.hand_intrinsics_source
    meta.setdefault("hand_tags", list(cfg.tags))
    meta["sweep"] = dict(script="sheet_sweep.py", date=time.strftime("%Y-%m-%d %H:%M:%S"), tags=list(cfg.tags),
                         heights_m=list(cfg.heights_m), tilt_deg=cfg.tilt_deg, spins_rel_deg=list(cfg.spins_rel_deg),
                         max_rot_from_start_deg=cfg.max_rot_from_start_deg)

    # the sheet in the arm frame, from front_cam's LIVE view of tag 200
    f, ftags, nf = S.camera_T(S.cfg.topics.front_cam_detections, S.K_front, None if S.front_frame == "level" else S.D_front, "front_cam")
    slope = math.degrees(math.acos(max(-1.0, min(1.0, abs(f.T_cam2W[2, 2])))))
    T_fc2W = f.T_cam2W if args.front_rotation == "measured" else CC.level_front_observation(f.T_cam2W)
    lift = S.lift.height_m() or 0.0
    cur = S.arm.get_tcp_pose()
    T_ab2ee_start = pose_fr5_to_matrix_m(cur)
    T_ab2W = compensate_T_ab2mb(ext.T_ab2mb, lift) @ ext.T_mb2fc_chain @ T_fc2W
    T_hc2W_cur = invert_T(T_ab2ee_start @ invert_T(H)) @ T_ab2W
    spin_cur = math.degrees(math.atan2(T_hc2W_cur[1, 0], T_hc2W_cur[0, 0]))
    print("front_cam sees %s (PnP rms %.2f px, paper slope %.2f deg%s); lift %.3f m; current camera spin %.0f deg; flange %s"
          % (f.tag_ids, f.rms_px, slope, " — over 1 deg, tape it flat" if slope > 1.0 else "", lift, spin_cur, ["%.1f" % v for v in cur]))
    K_hand = S.K_hand
    targets, rejected, rules = plan(T_ab2W, H, T_ab2ee_start, sheet, K_hand, S.D_hand, cfg, spin_cur)

    print("\n%d view(s) planned, %d rejected:" % (len(targets), len(rejected)))
    print("  #  label                        flange x      y      z      rx      ry      rz   reach  rot   tags in view")
    for i, v in enumerate(targets):
        print("%3d  %-28s %8.1f %6.1f %6.1f %7.2f %7.2f %7.2f   %.3f  %4.1f  %s"
              % (i + 1, v.label, *v.pose, v.reach_m, v.rot_from_start_deg, v.predicted_tags))
    for v in rejected:
        print("  rejected %-28s %s" % (v.label, v.reason))
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "sweep_plan.csv"), "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["order", "label", "tag", "height_m", "tilt_deg", "azimuth_deg", "spin_rel_deg", "x_mm", "y_mm", "z_mm",
                                        "rx_deg", "ry_deg", "rz_deg", "reach_m", "rot_from_start_deg", "predicted_tags", "reason"])
        for i, v in enumerate(targets + rejected):
            w.writerow([i + 1 if v.pose else "", v.label, v.tag, "%.3f" % v.height_m, v.tilt_deg, v.azimuth_deg, v.spin_rel_deg]
                       + (["%.3f" % p for p in v.pose] if v.pose else [""] * 6)
                       + ["%.4f" % v.reach_m if v.pose else "", "%.2f" % v.rot_from_start_deg if v.pose else "",
                          " ".join(map(str, v.predicted_tags)), v.reason or ""])
    if not targets:
        sys.exit("nothing to do")
    d0 = float(np.linalg.norm(targets[0].T_ab2ee[:3, 3] - T_ab2ee_start[:3, 3]))
    print("first target is %.2f m from the current flange; plan -> %s/sweep_plan.csv" % (d0, out_dir))
    if args.dry_run:
        print("dry run — nothing moved"); return
    if d0 > args.max_first_move:
        sys.exit("first target is %.2f m away — park hand_cam over the %s column at ~%.2f m first (or --max-first-move)"
                 % (d0, cfg.tags, cfg.heights_m[0]))
    if not args.yes:
        if input("\nRun %d views automatically (MoveL, %.0f%% speed, %.0f deg max rotation)? Hand on the e-stop. [y/N] "
                 % (len(targets), args.vel, cfg.max_rot_from_start_deg)).strip().lower() != "y":
            print("aborted"); return

    log_path = os.path.join(out_dir, "sweep_log.csv")
    new_log = not os.path.exists(log_path)
    lfh = open(log_path, "a", newline=""); lw = csv.writer(lfh)
    if new_log:
        lw.writerow(["time", "label", "sample", "tag", "height_m", "tilt_deg", "azimuth_deg", "spin_rel_deg", "hand_tags", "hand_rms_px",
                     "scatter_px", "front_tags", "raw_err_mm", "raw_err_deg", "tcp", "joints", "note"]); lfh.flush()
    print("results -> %s (samples.npz / corners.json / meta.yaml appended per view, sweep_log.csv)\n" % out_dir)

    def on_sample(sample, info, v):
        label = next_label(samples)
        sample.label = label
        samples.append(sample)
        save_samples(out_dir, samples, meta)
        tool.print_sample(sample, info, H, ext, sheet, args.front_rotation)
        ok, lines = coverage(samples, sheet=sheet)
        print("   coverage: %s\n             %s" % (lines[0], lines[1]))
        e = CC.pair_errors([sample], sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain, lift_compensate=compensate_T_ab2mb)[0]
        scatter = max(t.std_px for t in info["hand_tags"].values())
        lw.writerow([time.strftime("%H:%M:%S"), v.label, label, v.tag, "%.3f" % v.height_m, v.tilt_deg, v.azimuth_deg, v.spin_rel_deg,
                     " ".join(map(str, sorted(info["hand_tags"]))), "%.2f" % info["hand"].rms_px, "%.2f" % scatter,
                     " ".join(map(str, sorted(info["front_tags"]))), "%.1f" % (e[3] * 1e3), "%.2f" % e[4],
                     " ".join("%.1f" % x for x in sample.tcp_pose_mm_deg), " ".join("%.2f" % x for x in (sample.joints_deg or [])), "ok"])
        lfh.flush()

    def _log(msg):
        print(msg)
        if msg.startswith("   "):
            lw.writerow([time.strftime("%H:%M:%S"), "", "", "", "", "", "", "", "", "", "", "", "", "", "", "", msg.strip()]); lfh.flush()

    res = execute(targets, rules, cfg,
                  get_T=lambda: pose_fr5_to_matrix_m(S.arm.get_tcp_pose()),
                  move=lambda pose: S.arm.move_j_to_pose(pose, linear=True, physical=True),   # absolute view targets
                  capture=lambda label: S.one_sample(label),
                  on_sample=on_sample, settle=lambda: rospy.sleep(args.settle), log=_log,
                  return_to=None if args.no_return else T_ab2ee_start, front_ref_t=T_fc2W[:3, 3].copy())
    lfh.close()
    save_samples(out_dir, samples, meta)
    ok, lines = coverage(samples, sheet=sheet)
    print("\ndone: %d captured, %d move failures, %d capture failures%s; %d samples in %s"
          % (res["ok"], res["move_failed"], res["capture_failed"], " (interrupted)" if res["stopped"] else "", len(samples), out_dir))
    print("coverage: %s\n          %s" % (lines[0], lines[1]))
    sc = "--sx %s --sy %s --tag-size %s" % (sheet.sx, sheet.sy, sheet.tag_size_m)
    print("\nnext:\n  rosrun chain_calib chain_calib.py %s solve %s --holdout-every 4\n  rosrun chain_calib arm_offsets.py %s %s --holdout-every 4"
          % (sc, out_dir, out_dir, sc))


if __name__ == "__main__":
    main()
