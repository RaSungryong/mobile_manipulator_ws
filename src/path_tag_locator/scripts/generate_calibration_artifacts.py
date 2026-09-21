#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate every design-value calibration artifact in one pass:

  1. config/calibration_plan_plate1.yaml   (zones B+C, 정반 1 cross tags)
  2. config/calibration_plan_plate2.yaml   (zones D+E, 정반 2 cross tags)
     — each WORK tag paired with a cross tag by the EXPLICIT id ranges in
       ``REF_RANGES`` (user assignment, 2026-09-08; it replaced the older
       nearest-by-y rule, which split every column differently: 4/3/6
       tags per ref instead of 5/3/5), WITH a per-entry
       ``arm_view_tcp_mm_deg`` computed from design values (see below).
  3. <ws>/docs/all_tags_position.csv — all 78 tags with position,
     orientation, and for WORK tags: ref pairing, robot stop pose, the
     arm view TCP and the ab-frame reach distance.

How the arm view TCP is computed (all design values, no robot needed):
  - robot stop pose: centre = tag − 0.55 m · heading  (map.yaml rule),
    heading from zone (B/D +90°, C/E −90°); mb origin on the FLOOR,
    i.e. z = −0.080 in the world frame (world z=0 = plate top).
  - T_ab2mb from apriltag_nav/config/tf/tf_chain.yaml, shifted by --lift-mm
    (chain.compensate_T_ab2mb; default 0 = lift at origin — REGENERATE
    with the session height if map_calibrator.yaml lift_height_mm set).
  - T_hc2ee from the hand-eye npz.
  - view_pose.compute_view_tcp puts hand-cam --view-m (default:
    locator.yaml auto_view_distance_m) squarely above the ref tag.

The per-entry TCP makes a session deterministic (entry override beats the
auto_view_pose bootstrap). run_auto_align still refines each pose, so
cm-level design/tape mismatches are absorbed as before.

Usage (workspace sourced):
    rosrun path_tag_locator generate_calibration_artifacts.py \
        [--lift-mm 0] [--view-m 0.8]
"""
import argparse
import csv
import itertools
import math
from pathlib import Path

import numpy as np
import yaml

from path_tag_locator.calibration.view_pose import compute_view_tcp
from path_tag_locator.chain import compensate_T_ab2mb
from path_tag_locator.constants import load_extrinsics
from path_tag_locator.geometry import rpy_deg_to_R
from path_tag_locator.hand_eye import load_T_hc2ee

PKG = Path(__file__).resolve().parent.parent          # src/path_tag_locator
WS = PKG.parent.parent
MAP = WS / "src" / "apriltag_nav" / "config" / "map.yaml"
CFG = PKG / "config"

CAMERA_OFFSET_M = 0.55
FLOOR_Z_M = -0.080          # floor (= mb origin height) in the world frame
# vision_tip TCP offset from the FLANGE, tool frame mm (set_tool_tcp.py /
# fr10v6_visionDF_addtip.urdf). Reach is a FLANGE constraint: the added
# link extends the TCP ~339 mm beyond it, and the camera yaw about the
# tag normal is free, so the overhang can be swung toward the arm base.
TOOL_OFFSET_MM = np.array([-1.8, -245.6, 209.6])   # vision tip, measured 2026-09-21 (robot.yaml)
FLANGE_REACH_M = 1.40       # FR10 nominal reach (to wrist/flange)
REACH_MARGIN_M = 0.05       # keep diagnostic poses this far inside it


def flange_reach_m(tcp_mm_deg):
    """Distance arm-base -> FLANGE for a given vision_tip TCP pose."""
    R = rpy_deg_to_R(tcp_mm_deg[3], tcp_mm_deg[4], tcp_mm_deg[5])
    flange = np.array(tcp_mm_deg[:3]) - R @ TOOL_OFFSET_MM
    return float(np.linalg.norm(flange) / 1000.0)
ZONE_HEADING_DEG = {"A": 0.0, "DOCK": 0.0, "B": 90.0, "D": 90.0,
                    "C": -90.0, "E": -90.0}
WEST = {0: -1.2, 1: 0.0, 2: 1.2}      # cross-tag column ids by y
EAST = {5: -1.2, 4: 0.0, 3: 1.2}
PLATE_OF_ZONE = {"B": 1, "C": 1, "D": 2, "E": 2}
COLUMN_OF_ZONE = {"B": WEST, "C": EAST, "D": WEST, "E": EAST}
# WORK tag -> ref (cross) tag, by inclusive id range. User assignment
# 2026-09-08 ("각 주행태그의 참고 태그"): the three y-extreme tags of a
# corridor end share the corner ref tag, the middle three the centre one
# — 5 / 3 / 5 per column on both plates (zone B/D ids ascend northward,
# C/E descend, so the two corridors of a plate mirror each other).
# Checked 2026-09-08: this table equals "nearest cross tag to the robot's
# STOP pose" (tag y − 0.55 m along the heading) for every tag, which is
# why every re-paired entry got a SHORTER flange reach (0.93 → 0.74 m)
# than under the old rule. The table stays the source of truth.
# The generator refuses a ref that is not on the column facing the
# tag's corridor, so a typo here cannot send the camera across the plate.
REF_RANGES = (
    # plate 1 — zone B (west column 0/1/2), zone C (east column 3/4/5)
    (100, 104, 0), (105, 107, 1), (108, 112, 2),
    (113, 117, 3), (118, 120, 4), (121, 125, 5),
    # plate 2 — zone D (west), zone E (east); same ids 0-5 on 정반 2
    (126, 130, 0), (131, 133, 1), (134, 137, 2),
    (138, 142, 3), (143, 145, 4), (146, 150, 5),
)
REF_OF_TAG = {}
_PLATE_TAG_IDS = {}         # plate -> WORK tag ids seen (filled in main)
for _lo, _hi, _ref in REF_RANGES:
    for _t in range(_lo, _hi + 1):
        assert _t not in REF_OF_TAG, "tag %d in two REF_RANGES" % _t
        REF_OF_TAG[_t] = _ref


def ref_ranges_text(plate):
    """'100-104→0, 105-107→1, …' for the plan header of one plate."""
    return ", ".join("%d-%d→%d" % (lo, hi, ref) for lo, hi, ref in REF_RANGES
                     if lo in _PLATE_TAG_IDS[plate])


def rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def stop_pose(info):
    """Robot-centre stop (x, y, heading_deg) for a floor tag."""
    heading = ZONE_HEADING_DEG[info["zone"]]
    hx, hy = math.cos(math.radians(heading)), math.sin(math.radians(heading))
    return (info["x"] - CAMERA_OFFSET_M * hx,
            info["y"] - CAMERA_OFFSET_M * hy, heading)


def t_world2mb(stop_x, stop_y, heading_deg):
    T = np.eye(4)
    T[:3, :3] = rz(heading_deg)
    T[:3, 3] = [stop_x, stop_y, FLOOR_Z_M]
    return T


def load_refs(path):
    d = yaml.safe_load(open(path))["reference_tags"]
    out = {}
    for r in d:
        T = np.eye(4)
        # face-up: rpy [180, 0, yaw] -> Rz(yaw)·Rx(180); yaw is 0 today
        rx, ry_, yaw = r["rpy_deg"]
        assert (rx, ry_) == (180.0, 0.0), "generator assumes face-up tags"
        T[:3, :3] = rz(yaw) @ np.diag([1.0, -1.0, -1.0])
        T[:3, 3] = r["position_m"]
        out[r["id"]] = T
    return out


def corridor(info):
    zone, name = info.get("zone", ""), info.get("name", "") or ""
    if zone in ("B", "C", "D", "E"):
        return zone
    if name.startswith("Entry_"):
        return name[-1]
    return "A"


# ===========================================================================
# YAW-SWEEP entries — separating hand-eye from front_cam (B0 §4, 2026-09-11)
# ===========================================================================
# ONE path tag, ONE ref tag, N different CAMERA YAWS. The base never moves
# and the front_cam observation of the path tag is identical every time, so
# T_ab2mb @ T_mb2fc @ T_fc2B is a common factor and CANCELS. What is left
# varying is the arm side: hand-eye and the hand-cam observation.
#
# A hand-eye error is fixed in the EE frame, and sweeping the camera yaw
# rotates the EE about the vertical — so the computed tag position traces a
# CIRCLE. Its RADIUS is the arm-side error magnitude and needs no ground
# truth; its CENTRE is everything that does not rotate. That is the whole
# measurement (analyse_yaw_sweep.py fits the circle).
#
# Forward-model sensitivity, from the real chain:
#     front_cam rotation +2 deg  ->  spread  0.00 mm   (invisible, by design)
#     hand-eye rotation  +2 deg  ->  spread 22.20 mm
#     hand-eye translation 20 mm ->  spread 21.79 mm
# so a large radius means HAND-EYE, and a small radius with a large absolute
# error (from the normal session) means FRONT_CAM. That decides which of the
# two expensive fixes to run.
#
# ⚠️ A DUAL-ANCHOR design (same tag from two ref tags at a PINNED camera
# yaw) was built first and REMOVED on 2026-09-11. It was right for the
# premise it was designed under — ref tags suspect, so pin the yaw to cancel
# the chain and expose the anchors — and exactly backwards once the cross
# tags turned out to be machined into the plate and the CHAIN became the
# target: the pinned yaw cancels the very thing being measured. Measured on
# this same forward model it moves 0.47 mm for a 2 deg hand-eye error and
# 0.00 mm for hand-eye translation. It is in git history if the cross tags
# ever turn out to be printed inserts — print registration would give
# ±0.13 deg of ref yaw, which dual-anchor CAN see and this sweep cannot.


def emit_yaw_sweep_plans(args, views, tags, refs, T_ab2mb, T_hc2ee,
                         view_m, ref_yaml_name):
    """One short plan per plate: a single tag observed at N camera yaws."""
    for plate, zones in ((1, ("B", "C")), (2, ("D", "E"))):
        # pick the tag with the WIDEST reachable yaw span — the sweep's
        # resolving power is the angular coverage, not the entry count
        best_tag = None
        for tid, v in sorted(views.items()):
            if v["plate"] != plate:
                continue
            sx, sy, heading = v["stop"]
            T_w2mb = t_world2mb(sx, sy, heading)
            feasible = []
            for yaw in range(0, 360, 5):
                tcp = compute_view_tcp(
                    T_A_world=refs[plate][v["ref"]], T_world2mb=T_w2mb,
                    T_ab2mb=T_ab2mb, T_hc2ee=T_hc2ee,
                    view_distance_m=view_m, yaw_deg=float(yaw))
                if flange_reach_m(tcp) <= FLANGE_REACH_M - REACH_MARGIN_M:
                    feasible.append((yaw, tcp))
            if best_tag is None or len(feasible) > len(best_tag[1]):
                best_tag = (tid, feasible, v)
        tid, feasible, v = best_tag
        if len(feasible) < 4:
            print("calibration_plan_plate%d_yawsweep.yaml: only %d reachable "
                  "yaws — sweep would not resolve a circle" % (plate, len(feasible)))
            continue

        n = max(4, int(args.yaw_sweep_n))
        pick = [feasible[int(round(i * (len(feasible) - 1) / (n - 1)))]
                for i in range(n)]
        # drop duplicates that rounding may produce, keep order
        seen, chosen = set(), []
        for yaw, tcp in pick:
            if yaw not in seen:
                seen.add(yaw)
                chosen.append((yaw, tcp))

        info = tags[tid]
        span = chosen[-1][0] - chosen[0][0]
        lines = [
            "# YAW SWEEP — 정반 %d. GENERATED by\n"
            "# scripts/generate_calibration_artifacts.py "
            "(lift %.0f mm, view %.2f m)\n"
            "# — do not hand-edit; re-run the generator.\n"
            "#\n"
            "# THIS IS NOT A CALIBRATION. One path tag (%d), one ref tag\n"
            "# (%d), %d different CAMERA YAWS spanning %d°. The base never\n"
            "# moves, so front_cam's view of the tag is identical every time\n"
            "# and cancels; only the ARM side varies.\n"
            "#\n"
            "# Read it with: rosrun path_tag_locator analyse_yaw_sweep.py <session_dir>\n"
            "#   large circle radius          -> HAND-EYE is wrong\n"
            "#   radius ~0 but the normal session shows a large absolute\n"
            "#   error                        -> FRONT_CAM (T_mb2fc) is wrong\n"
            "#\n"
            "# ⚠️ Park the base ON tag %d and leave it there. Do NOT reorder\n"
            "# the entries and do NOT let a TASK/GOTO move the base midway —\n"
            "# a moved base breaks the common factor the whole method rests\n"
            "# on. Run with ref_tags_path = config/%s.\n"
            "\ndefaults:\n  retry_count: 1\n  align_required: true\n"
            "\nplan:\n"
            % (plate, args.lift_mm, view_m, tid, v["ref"], len(chosen), span,
               tid, ref_yaml_name[plate])]
        for k, (yaw, tcp) in enumerate(chosen):
            lines.append(
                "  - path_tag_id: %d    # %s y %+.3f | YAW SWEEP %d/%d | "
                "cam yaw %d° | flange %.3f m\n"
                "    ref_tag_id: %d\n"
                "    yaw_sweep_deg: %d   "
                "# analysis marker; the orchestrator ignores it\n"
                "    arm_view_tcp_mm_deg: [%s]\n"
                % (tid, info["zone"], info["y"], k + 1, len(chosen), yaw,
                   flange_reach_m(tcp), v["ref"], yaw,
                   ", ".join("%.1f" % round(x, 1) for x in tcp)))
            if k == 0:
                lines.append("    # START HERE — park the base on this tag "
                             "and do not move it\n")
        out = CFG / ("calibration_plan_plate%d_yawsweep.yaml" % plate)
        out.write_text("".join(lines))
        print("%s: tag %d / ref %d, %d yaws spanning %d° (%d reachable)"
              % (out.name, tid, v["ref"], len(chosen), span, len(feasible)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lift-mm", type=float, default=0.0,
                    help="session lift height (map_calibrator.yaml "
                         "lift_height_mm); TCP z shifts with it")
    ap.add_argument("--view-m", type=float, default=None,
                    help="hand-cam height above the ref tag (default: "
                         "locator.yaml auto_view_distance_m)")
    ap.add_argument("--yaw-sweep-n", type=int, default=6, metavar="N",
                    help="camera yaws in the yaw-sweep plans (min 4; the "
                         "analysis fits a circle, so angular COVERAGE "
                         "matters more than the count)")
    ap.add_argument("--diagnostics-only", action="store_true",
                    help="write ONLY the yaw-sweep plans; leave the main "
                         "calibration plans and the CSV alone. ⚠️ USE THIS "
                         "unless you mean to regenerate everything — a full "
                         "run RESETS the main plans' per-entry seeds to "
                         "design values, discarding any that "
                         "update_plan_seeds_from_session.py measured")
    args = ap.parse_args()

    locator = yaml.safe_load(open(CFG / "locator.yaml"))["path_tag_locator"]
    view_m = (args.view_m if args.view_m is not None
              else float(locator["align"]["auto_view_distance_m"]))
    from apriltag_nav.tf_chain import TF_CHAIN_PATH, npz_path as _tf_npz   # config/tf (2026-09-21)
    T_ab2mb, _ = load_extrinsics(TF_CHAIN_PATH)
    T_ab2mb = compensate_T_ab2mb(T_ab2mb, args.lift_mm / 1000.0)
    T_hc2ee = load_T_hc2ee(_tf_npz("T_hc2ee"))

    tags = yaml.safe_load(open(MAP))["tags"]
    refs = {1: load_refs(CFG / "reference_tags.yaml"),
            2: load_refs(CFG / "reference_tags_plate2.yaml")}
    ref_yaml_name = {1: "reference_tags.yaml",
                     2: "reference_tags_plate2.yaml"}

    # ---- per-WORK-tag design values --------------------------------
    views = {}
    for tid in sorted(tags):
        info = tags[tid]
        if info.get("type") != "WORK":
            continue
        zone = info["zone"]
        plate = PLATE_OF_ZONE[zone]
        column = COLUMN_OF_ZONE[zone]
        if tid not in REF_OF_TAG:
            raise SystemExit("WORK tag %d has no entry in REF_RANGES" % tid)
        ref_id = REF_OF_TAG[tid]
        if ref_id not in column:
            raise SystemExit(
                "tag %d (zone %s) -> ref %d, which is not on the column "
                "facing that corridor (%s)" % (tid, zone, ref_id,
                                               sorted(column)))
        _PLATE_TAG_IDS.setdefault(plate, set()).add(tid)
        sx, sy, heading = stop_pose(info)
        # Camera yaw about the tag normal is free: sweep it and keep the
        # pose whose FLANGE sits closest to the arm base.
        best = None
        for yaw in range(0, 360, 5):
            tcp = compute_view_tcp(
                T_A_world=refs[plate][ref_id],
                T_world2mb=t_world2mb(sx, sy, heading),
                T_ab2mb=T_ab2mb,
                T_hc2ee=T_hc2ee,
                view_distance_m=view_m,
                yaw_deg=float(yaw))
            fr = flange_reach_m(tcp)
            if best is None or fr < best[0]:
                best = (fr, yaw, tcp)
        fr, yaw, tcp = best
        views[tid] = dict(plate=plate, ref=ref_id, stop=(sx, sy, heading),
                          tcp=[round(v, 1) for v in tcp],
                          cam_yaw=yaw,
                          reach_tcp=round(
                              float(np.linalg.norm(np.array(tcp[:3]))
                                    / 1000.0), 3),
                          reach=round(fr, 3))

    # ---- plans ------------------------------------------------------
    for plate, zones in (() if args.diagnostics_only
                         else ((1, ("B", "C")), (2, ("D", "E")))):
        lines = [
            "# Calibration plan — 정반 %d (zones %s). GENERATED by\n"
            "# scripts/generate_calibration_artifacts.py (lift %.0f mm, "
            "view %.2f m)\n"
            "# — do not hand-edit values; re-run the generator instead.\n"
            "# Pairing: EXPLICIT id ranges (REF_RANGES in the generator, "
            "user assignment 2026-09-08):\n"
            "#   %s\n"
            "# Run with ref_tags_path = config/%s (SWAP TOGETHER with the "
            "plan).\n"
            "# Per-entry arm_view_tcp_mm_deg are DESIGN seeds (map.yaml "
            "stop pose\n"
            "# + extrinsics + hand-eye); run_auto_align refines each one.\n"
            "# HAND CAMERA IS THE ANCHOR: every pose puts the camera on the\n"
            "# ref tag's normal at the view height, optical axis through the\n"
            "# tag; the noted cam yaw spins about that axis only (free for\n"
            "# alignment), chosen so the FLANGE sits closest to the arm base.\n"
            "\ndefaults:\n  retry_count: 1\n  align_required: true\n"
            "\nplan:\n"
            % (plate, "+".join(zones), args.lift_mm, view_m,
               ref_ranges_text(plate), ref_yaml_name[plate])]
        first = True
        for tid in sorted(views):
            v = views[tid]
            if v["plate"] != plate:
                continue
            info = tags[tid]
            over = "  ⚠ OVER FLANGE REACH" if v["reach"] > FLANGE_REACH_M else ""
            lines.append(
                "  - path_tag_id: %d    # %s, y %+.3f, flange reach %.3f m"
                " (cam yaw %d°)%s\n"
                "    ref_tag_id: %d\n"
                "    arm_view_tcp_mm_deg: [%s]\n"
                % (tid, info["zone"], info["y"], v["reach"], v["cam_yaw"],
                   over, v["ref"],
                   ", ".join("%.1f" % x for x in v["tcp"])))
            if first:
                # No nav_start_id: the session starts from wherever the
                # base is parked, which must be ON this first tag (front_cam
                # sees it). Earlier plans drove to DOCK 500 first, which
                # for plate 2 meant a 9 m detour before the first entry.
                lines.append("    # START HERE — park the base on this tag before "
                             "calling run_calibration\n")
                first = False
        out = CFG / ("calibration_plan_plate%d.yaml" % plate)
        out.write_text("".join(lines))
        n = sum(1 for v in views.values() if v["plate"] == plate)
        print("%s: %d entries" % (out.name, n))

    # ---- yaw-sweep diagnostic plans (docs/chain_error_diagnosis.md) ---
    emit_yaw_sweep_plans(args, views, tags, refs, T_ab2mb, T_hc2ee,
                         view_m, ref_yaml_name)

    # ---- csv --------------------------------------------------------
    if args.diagnostics_only:
        return
    header = ["tag_id", "zone", "corridor", "type", "name",
              "tag_x_mm", "tag_y_mm", "tag_z_mm",
              "roll_deg", "pitch_deg", "yaw_deg",
              "ref_tag_id", "stop_x_mm", "stop_y_mm", "robot_heading_deg",
              "arm_x_mm", "arm_y_mm", "arm_z_mm",
              "arm_rx_deg", "arm_ry_deg", "arm_rz_deg",
              "reach_tcp_m", "reach_flange_m"]
    rows = []
    for plate in (1, 2):
        src = yaml.safe_load(open(CFG / ref_yaml_name[plate]))
        for r in sorted(src["reference_tags"], key=lambda r: r["id"]):
            if plate == 2:
                continue          # same ids; list the 정반 1 set only,
                                  # 정반 2 = +3900 mm x (plate geometric
                                  # centre; see yaml header)
            x, y, z = r["position_m"]
            roll, pitch, yaw = r["rpy_deg"]
            rows.append([r["id"], "plate", "", "Calibration", "",
                         round(x * 1000, 1), round(y * 1000, 1),
                         round(z * 1000, 1), roll, pitch, yaw]
                        + [""] * 12)
    for tid in sorted(tags):
        info = tags[tid]
        yaw = ZONE_HEADING_DEG.get(info.get("zone", "A"), 0.0)
        row = [tid, info.get("zone", ""), corridor(info),
               info.get("type", ""), info.get("name", "") or "",
               round(info["x"] * 1000, 1), round(info["y"] * 1000, 1),
               -80.0, 180.0, 0.0, yaw]
        v = views.get(tid)
        if v is None:
            row += [""] * 12
        else:
            sx, sy, heading = v["stop"]
            row += [v["ref"], round(sx * 1000, 1), round(sy * 1000, 1),
                    heading] + v["tcp"] + [v["reach_tcp"], v["reach"]]
        rows.append(row)

    out_csv = WS / "docs" / "all_tags_position.csv"
    with open(out_csv, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)
    reaches = [v["reach"] for v in views.values()]
    over = sum(1 for r in reaches if r > FLANGE_REACH_M)
    print("%s: %d rows; FLANGE reach %.3f .. %.3f m, %d/%d over %.2f"
          % (out_csv, len(rows), min(reaches), max(reaches),
             over, len(reaches), FLANGE_REACH_M))


if __name__ == "__main__":
    main()
