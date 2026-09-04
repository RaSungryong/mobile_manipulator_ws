#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate every design-value calibration artifact in one pass:

  1. config/calibration_plan_plate1.yaml   (zones B+C, 정반 1 cross tags)
  2. config/calibration_plan_plate2.yaml   (zones D+E, 정반 2 cross tags)
     — each WORK tag paired with the nearest-by-y cross tag of the column
       facing its corridor, WITH a per-entry ``arm_view_tcp_mm_deg``
       computed from design values (see below).
  3. <ws>/docs/all_tags_position.csv — all 78 tags with position,
     orientation, and for WORK tags: ref pairing, robot stop pose, the
     arm view TCP and the ab-frame reach distance.

How the arm view TCP is computed (all design values, no robot needed):
  - robot stop pose: centre = tag − 0.55 m · heading  (map.yaml rule),
    heading from zone (B/D +90°, C/E −90°); mb origin on the FLOOR,
    i.e. z = −0.080 in the world frame (world z=0 = plate top).
  - T_ab2mb from extrinsics.yaml, shifted by --lift-mm
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
TOOL_OFFSET_MM = np.array([0.0, -253.0, 225.2])
FLANGE_REACH_M = 1.40       # FR10 nominal reach (to wrist/flange)


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lift-mm", type=float, default=0.0,
                    help="session lift height (map_calibrator.yaml "
                         "lift_height_mm); TCP z shifts with it")
    ap.add_argument("--view-m", type=float, default=None,
                    help="hand-cam height above the ref tag (default: "
                         "locator.yaml auto_view_distance_m)")
    args = ap.parse_args()

    locator = yaml.safe_load(open(CFG / "locator.yaml"))["path_tag_locator"]
    view_m = (args.view_m if args.view_m is not None
              else float(locator["align"]["auto_view_distance_m"]))
    T_ab2mb, _ = load_extrinsics(str(CFG / "extrinsics.yaml"))
    T_ab2mb = compensate_T_ab2mb(T_ab2mb, args.lift_mm / 1000.0)
    T_hc2ee = load_T_hc2ee(str(CFG / "hand_eye" / "T_hc2ee.npz"))

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
        ref_id = min(column, key=lambda k: abs(column[k] - info["y"]))
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
    for plate, zones in ((1, ("B", "C")), (2, ("D", "E"))):
        lines = [
            "# Calibration plan — 정반 %d (zones %s). GENERATED by\n"
            "# scripts/generate_calibration_artifacts.py (lift %.0f mm, "
            "view %.2f m)\n"
            "# — do not hand-edit values; re-run the generator instead.\n"
            "# Pairing: nearest-by-y cross tag of the column facing the "
            "corridor.\n"
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
               ref_yaml_name[plate])]
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

    # ---- csv --------------------------------------------------------
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
