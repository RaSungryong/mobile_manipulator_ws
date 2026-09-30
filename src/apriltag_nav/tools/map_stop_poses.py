#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Where the mobile base CENTRE is, in the world (map.yaml) frame, when
front_cam has aligned on a drive tag and stopped — one row per map tag.

    python3 tools/map_stop_poses.py [--out docs/robot_base_stop_poses.csv]

Geometry (the same arithmetic `mobile_controller.calculate_robot_pose`
uses at an at-rest arrival, with fore = lateral = 0 and align residual 0):

    heading  = zone axis (A/DOCK 0, B/D +90, C/E -90 deg)
             + delta, the tag's calibrated laying angle from map.yaml
               `yaw` (deviation from the nearest 90-deg axis; tags without
               a `yaw` have delta 0)
    base     = tag - camera_offset * (cos heading, sin heading)

Why T_mb2fc's tilt / yaw do NOT appear as separate terms, although the
physical camera is 1.44 deg off vertical and -0.38 deg spun vs the travel
axis: `robot_camera_node` re-images front_cam's detections through a LEVEL
virtual camera (robot.yaml ground_plane.front_cam), whose frame is
T_mb2fc_physical @ T_tilted_to_level(roll, pitch, yaw) — a rotation of
exactly diag(1, -1, -1) about the same lens centre (asserted below). So
"tag on the crosshair" means the tag centre is under the lens NADIR
(camera_offset, camera_lateral) in the body frame, and "edge angle 0"
means the body's travel axis is parallel to the tag edge. The tilt moves
nothing (the nadir, not the optical-axis floor point, is the reference)
and the -0.38 deg yaw is what makes "aligned" mean body-parallel; both
are already inside the pose above. Its residual uncertainty is the fit's
(+-0.22 deg of yaw -> +-2 mm of lateral at the base centre) plus the
align band (0.2 deg -> 1.9 mm).

Output columns: tag_id, zone, type, name, tag_x_m, tag_y_m, tag_yaw_deg
(map.yaml, blank when absent), lane_heading_deg, laying_delta_deg,
base_x_m, base_y_m, base_yaw_deg (world heading of the body's forward
axis, CCW +, deg).  Pivot tags: 501-504 are read facing EAST (before the
pivot), 505-508 facing INTO the corridor (after it) — the map's zone.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WS = os.path.dirname(os.path.dirname(PKG))          # <ws>/src/apriltag_nav -> <ws>
sys.path.insert(0, os.path.join(PKG, 'src'))

from apriltag_nav.ground_plane import T_tilted_to_level  # noqa: E402

ZONE_HEADING = {'A': 0.0, 'DOCK': 0.0, 'B': 90.0, 'D': 90.0, 'C': -90.0, 'E': -90.0}


def laying_delta_deg(tag, use_tag_yaw):
    """map.yaml `yaw` -> deviation from the nearest 90-deg axis (deg, CCW +);
    mirrors MobileController._tag_yaw_error_deg."""
    if not use_tag_yaw or tag.get('yaw') is None:
        return 0.0
    err = (float(tag['yaw']) + 45.0) % 90.0 - 45.0
    return 0.0 if abs(err) > 10.0 else err


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def check_level_frame(tf_yaml, gp):
    """Assert T_mb2fc_physical @ T_tilted_to_level == [diag(1,-1,-1) | lens]."""
    T = np.array(tf_yaml['T_mb2fc']['matrix'], dtype=float).reshape(4, 4)
    L = T @ T_tilted_to_level(math.radians(gp['roll_deg']),
                              math.radians(gp['pitch_deg']),
                              math.radians(gp.get('yaw_deg', 0.0)))
    R_err = np.abs(L[:3, :3] - np.diag([1.0, -1.0, -1.0])).max()
    z_axis = T[:3, 2]                       # physical optical axis in mb
    lens = T[:3, 3]
    floor_pt = lens - z_axis * (lens[2] / z_axis[2])   # where the axis meets the floor
    return R_err, lens, floor_pt


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--map', default=os.path.join(PKG, 'config', 'map.yaml'))
    ap.add_argument('--robot', default=os.path.join(PKG, 'config', 'robot.yaml'))
    ap.add_argument('--tf', default=os.path.join(PKG, 'config', 'tf', 'tf_chain.yaml'))
    ap.add_argument('--out', default=os.path.join(WS, 'docs', 'robot_base_stop_poses.csv'))
    ap.add_argument('--ids', default='100-199',
                    help="tag id ranges to include, e.g. '100-199' (default: the WORK "
                         "tags only, user 2026-09-28) or '100-199,500-508'; 'all' for every tag")
    args = ap.parse_args()

    def wanted(tid):
        if args.ids.strip().lower() == 'all':
            return True
        for part in args.ids.split(','):
            lo, _, hi = part.strip().partition('-')
            lo = int(lo)
            hi = int(hi) if hi else lo
            if lo <= int(tid) <= hi:
                return True
        return False

    with open(args.map) as f:
        tags = yaml.safe_load(f)['tags']
    with open(args.robot) as f:
        robot = yaml.safe_load(f)
    with open(args.tf) as f:
        tf_yaml = yaml.safe_load(f)

    rcfg = robot['robot']
    cam_offset = float(rcfg.get('camera_offset', 0.55))
    cam_lateral = float(rcfg.get('camera_lateral', 0.0))
    use_tag_yaw = bool(rcfg.get('robot_pose_use_tag_yaw', True))
    gp = robot['robot_camera']['ground_plane']['front_cam']

    R_err, lens, floor_pt = check_level_frame(tf_yaml, gp)
    if R_err > 1e-6:
        sys.exit(f"T_mb2fc @ T_tilted_to_level is not diag(1,-1,-1) (max err {R_err:.2e}); "
                 "regenerate tf_chain.yaml with tf_chain_tool.py front-cam --apply")
    if abs(lens[0] - cam_offset) > 1e-6 or abs(lens[1] - cam_lateral) > 1e-6:
        sys.exit("tf_chain.yaml T_mb2fc lens does not match robot.yaml camera_offset / camera_lateral")
    print(f"T_mb2fc: level-frame rotation == diag(1,-1,-1) to {R_err:.1e}; "
          f"lens (nadir) at ({lens[0]:.4f}, {lens[1]:.4f}) m in the body, "
          f"physical optical axis meets the floor at ({floor_pt[0]:.4f}, {floor_pt[1]:.4f}) "
          f"-> {1e3*math.hypot(floor_pt[0]-lens[0], floor_pt[1]-lens[1]):.1f} mm from the nadir "
          "(not used: the crosshair is the nadir)")
    print(f"camera_offset {cam_offset}, camera_lateral {cam_lateral}, "
          f"tag yaw {'used' if use_tag_yaw else 'IGNORED'}, ground_plane yaw_deg {gp.get('yaw_deg', 0.0)}")

    rows = []
    for tid, tag in tags.items():
        if not wanted(tid):
            continue
        zone = str(tag.get('zone', 'A'))
        if zone not in ZONE_HEADING:
            print(f"  tag {tid}: unknown zone {zone!r}, skipped")
            continue
        delta = laying_delta_deg(tag, use_tag_yaw)
        heading = ZONE_HEADING[zone] + delta
        h = math.radians(heading)
        ux, uy = math.cos(h), math.sin(h)          # body forward, world
        rx, ry = math.sin(h), -math.cos(h)         # body right, world
        tx, ty = float(tag['x']), float(tag['y'])
        # lens nadir is over the tag; the base centre is cam_offset behind it
        # (and cam_lateral to the left of the lens, if ever non-zero)
        bx = tx - cam_offset * ux + cam_lateral * rx
        by = ty - cam_offset * uy + cam_lateral * ry
        rows.append({
            'tag_id': int(tid), 'zone': zone, 'type': tag.get('type', ''),
            'name': tag.get('name', ''),
            'tag_x_m': f"{tx:.4f}", 'tag_y_m': f"{ty:.4f}",
            'tag_yaw_deg': '' if tag.get('yaw') is None else f"{float(tag['yaw']):.2f}",
            'lane_heading_deg': f"{ZONE_HEADING[zone]:.0f}",
            'laying_delta_deg': f"{delta:.2f}",
            'base_x_m': f"{bx:.4f}", 'base_y_m': f"{by:.4f}",
            'base_yaw_deg': f"{wrap180(heading):.2f}",
        })

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"{len(rows)} tags -> {args.out}")


if __name__ == '__main__':
    main()
