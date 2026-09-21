#!/usr/bin/env python3
"""Fit front_cam's ground-plane extrinsics from tag-pair snapshots.

Method (2026-09-08): two tags are laid on the floor a precisely known
CENTRE distance apart. The base is parked so both are in the frame and the
operator saves the raw detections at rest at several positions (small
manual moves and small in-place pivots — the commanded motion is NOT used,
only what the tags show). Every snapshot then constrains the camera with 8
corner points of known relative geometry at an unknown robot pose. The fit
solves
    roll, pitch, lens height, printed tag size,
    each tag's in-plane rotation vs the centre line (2026-09-15: the tags
    no longer have to be laid edge-parallel — only the centre spacing is
    the ruler; a tag laid a quarter turn round just fits with a 90 deg
    angle, so the dt_apriltags corner order needs no special handling)
    (+ 3 pose params / snapshot)
against CameraInfo K / D, and reports the residual, the numbers for
robot.yaml `robot_camera.ground_plane.front_cam`, the edge-angle error the
UNCORRECTED pipeline would read vs fore-aft position, and — from any
consecutive pivot snapshots — the lens-to-pivot lever (`camera_offset`).

The fitted height is the lens above the TAG-TOP plane (the corners' plane);
every tag is a 1 mm plate (robot.yaml robot.tag_thickness), which is why
tf_chain.yaml T_mb2fc's tz is height_m + tag_thickness.

Snapshots: `rostopic echo -n1 /front_cam/tag_detections > dir/scan_<t>.txt`
(the node must be publishing RAW detections, i.e. ground_plane disabled, or
the fit sees already-corrected data). CameraInfo:
`rostopic echo -n1 /front_cam/color/camera_info > dir/camera_info.txt`.
The full session (moves, pivots, drives, solve, apply) is
tools/calib_front_cam_pose.py; this script is the fit it reuses, and
reproduces the 2026-09-08 record as is:

    rosrun apriltag_nav fit_front_cam_ground.py log/apriltag_nav/calib_pair --spacing 0.150 --tags 15 16 --size 0.060
"""
import argparse
import glob
import math
import os
import re
import sys

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from apriltag_nav.ground_plane import GroundPlane, rot_xyz  # noqa: E402


def parse_snapshot(fn):
    txt = open(fn).read()
    out = {}
    for tid, cx, cy, cs in re.findall(
            r"id: (\d+).*?center_x: ([-\d.e]+).*?center_y: ([-\d.e]+).*?corners: \[([^\]]+)\]", txt, re.S):
        out[int(tid)] = dict(cx=float(cx), cy=float(cy),
                             c=np.array([float(v) for v in cs.split(',')]).reshape(4, 2))
    return out


def parse_camera_info(fn):
    txt = open(fn).read()
    K = [float(v) for v in re.search(r"K: \[([^\]]+)\]", txt).group(1).split(',')]
    m = re.search(r"D: \[([^\]]+)\]", txt)
    D = [float(v) for v in m.group(1).split(',')] if m else []
    return K[0], K[4], K[2], K[5], D


def pair_corners(size, spacing, deltas=(0.0, 0.0)):
    """Corners of both tags in the PAIR frame (x along the centre line, y
    right), in dt_apriltags order as seen on this camera: c0 (x-, y-),
    c1 (x-, y+), c2 (x+, y+), c3 (x+, y-) for a tag laid edge-parallel;
    `deltas` rotates each tag's square about its own centre (rad, the
    tag's in-plane laying angle vs the centre line, fitted)."""
    h = size / 2.0
    base = np.array([[-h, -h], [-h, h], [h, h], [h, -h]])
    out = []
    for k, d in enumerate(deltas):
        c, s = math.cos(d), math.sin(d)
        R = np.array([[c, -s], [s, c]])
        out.append(base @ R.T + [k * spacing, 0.0])
    return np.vstack(out)


def _edge_angle(corners):
    d = corners[1] - corners[0]
    return math.atan2(d[1], d[0])


def initial_tag_rotations(d, tags):
    """Per-tag in-plane angle vs the centre line from one raw snapshot
    (image geometry; the fit refines it). For an edge-parallel tag the
    c0 -> c1 edge images at the pair direction + 90 deg."""
    pair = math.atan2(d[tags[1]]['cy'] - d[tags[0]]['cy'], d[tags[1]]['cx'] - d[tags[0]]['cx'])
    out = []
    for t in tags:
        a = _edge_angle(d[t]['c']) - (pair + math.pi / 2)
        out.append(math.atan2(math.sin(a), math.cos(a)))
    return out


def load_snapshots(dir_, tags, prefixes=None, min_tags=2):
    """[(basename, parsed)] for every snapshot in `dir_` that shows at
    least `min_tags` of the pair (2 = both, the fit's requirement; 1 =
    either, for the rotation-centre fit which can place the pair from one
    tag). Sorted by name; `prefixes` restricts to e.g. ('scan', 'piv')."""
    snaps = []
    for f in sorted(glob.glob(os.path.join(dir_, '*.txt'))):
        base = os.path.basename(f)
        if base.startswith('camera_info'):
            continue
        if prefixes and not base.startswith(tuple(prefixes)):
            continue
        d = parse_snapshot(f)
        if len([tg for tg in tags if tg in d]) >= min_tags:
            snaps.append((base, d))
    return snaps


N_GLOBAL = 6     # roll, pitch, h, size, delta_tag0, delta_tag1


def fit_ground(snaps, tags, fx, fy, cx, cy, D, spacing, size0=0.060, h0=0.30):
    """The tag-pair fit. Returns dict(roll, pitch, h, size, tag_rot, rms_px,
    max_px, rms_level_px, gp) — angles in radians (tag_rot = each tag's
    in-plane angle vs the centre line), h the lens height above the tag-top
    plane, gp a GroundPlane with yaw 0 (its x axis is the CAMERA x, not yet
    the travel axis)."""
    t0, t1 = tags
    obs = [np.vstack([d[t0]['c'], d[t1]['c']]) for _, d in snaps]

    def residuals(p):
        roll, pitch, h, s, d0, d1 = p[:N_GLOBAL]
        gp = GroundPlane(fx, fy, cx, cy, D, roll, pitch, h)
        pc = pair_corners(s, spacing, (d0, d1))
        res = []
        for k in range(len(snaps)):
            X, Y, psi = p[N_GLOBAL + 3 * k: N_GLOBAL + 3 + 3 * k]
            c, sn = math.cos(psi), math.sin(psi)
            g = np.column_stack([X + c * pc[:, 0] - sn * pc[:, 1], Y + sn * pc[:, 0] + c * pc[:, 1]])
            res.append((gp.project(g) - obs[k]).ravel())
        return np.concatenate(res)

    p0 = [0.0, 0.0, h0, size0] + initial_tag_rotations(snaps[0][1], tags)
    for _, d in snaps:
        p0 += [(d[t0]['cx'] - cx) * h0 / fx, (d[t0]['cy'] - cy) * h0 / fy,
               math.atan2(d[t1]['cy'] - d[t0]['cy'], d[t1]['cx'] - d[t0]['cx'])]
    fit = least_squares(residuals, p0, method='lm', xtol=1e-12, ftol=1e-12)
    roll, pitch, h, s, d0, d1 = fit.x[:N_GLOBAL]
    r = residuals(fit.x)
    lvl = least_squares(lambda q: residuals(np.concatenate([[0.0, 0.0], q])), p0[2:], method='lm')
    r0 = residuals(np.concatenate([[0.0, 0.0], lvl.x]))
    return dict(roll=roll, pitch=pitch, h=h, size=s, tag_rot=(float(d0), float(d1)),
                rms_px=math.sqrt(np.mean(r ** 2)), max_px=float(np.abs(r).max()),
                rms_level_px=math.sqrt(np.mean(r0 ** 2)),
                gp=GroundPlane(fx, fy, cx, cy, D, roll, pitch, h))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('dir')
    ap.add_argument('--spacing', type=float, default=0.150, help='tag centre spacing (m)')
    ap.add_argument('--tags', type=int, nargs=2, default=[15, 16], help='tag ids, first -> second along +x')
    ap.add_argument('--size', type=float, default=0.060, help='nominal printed tag size (m), refined by the fit')
    ap.add_argument('--height', type=float, default=0.30, help='initial lens height (m)')
    ap.add_argument('--camera-info', default=None, help='camera_info dump (default <dir>/camera_info.txt)')
    a = ap.parse_args()

    fx, fy, cx, cy, D = parse_camera_info(a.camera_info or os.path.join(a.dir, 'camera_info.txt'))
    snaps = load_snapshots(a.dir, a.tags)
    if len(snaps) < 3:
        sys.exit(f"need at least 3 snapshots with both tags, found {len(snaps)}")
    t0, t1 = a.tags
    F = fit_ground(snaps, a.tags, fx, fy, cx, cy, D, a.spacing, a.size, a.height)
    roll, pitch, h, s = F['roll'], F['pitch'], F['h'], F['size']
    gp = GroundPlane(fx, fy, cx, cy, D, roll, pitch, h)
    ax = gp.axis_offset_m()

    print(f"snapshots {len(snaps)}, corner points {8 * len(snaps)}, rms {F['rms_px']:.3f} px "
          f"(max {F['max_px']:.2f}); level camera would give {F['rms_level_px']:.3f} px")
    print(f"roll {math.degrees(roll):+.3f} deg, pitch {math.degrees(pitch):+.3f} deg, height {h * 1000:.1f} mm "
          f"(lens above the tag-top plane), tag size {s * 1000:.2f} mm; optical axis meets the floor "
          f"{ax[0] * 1000:+.1f} / {ax[1] * 1000:+.1f} mm from the nadir")
    print(f"tags' in-plane angle vs the centre line: {t0} {math.degrees(F['tag_rot'][0]):+.2f} deg, "
          f"{t1} {math.degrees(F['tag_rot'][1]):+.2f} deg (multiples of 90 = laid rotated; the rest = laying skew)")
    print("\nrobot.yaml:\n  robot_camera:\n    ground_plane:\n      front_cam:\n        enabled: true\n"
          f"        roll_deg: {math.degrees(roll):.3f}\n        pitch_deg: {math.degrees(pitch):.3f}\n"
          f"        height_m: {h:.3f}\n        yaw_deg: <keep the current value — not observable from tags; "
          "re-measure with a straight-drive test (see robot.yaml)>")

    print("\nedge angle the UNCORRECTED pipeline reads for a square-laid 90 mm tag, vs fore-aft position:")
    for X in (-0.03, 0.0, 0.05, 0.10, 0.155, 0.20):
        sq = np.array([[X - 0.045, -0.045], [X - 0.045, 0.045], [X + 0.045, 0.045], [X + 0.045, -0.045]])
        c = gp.project(sq)
        ang = (math.degrees(math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0])) - 90.0 + 90.0) % 180.0 - 90.0
        print(f"   X {X:+.3f} m: {ang:+.3f} deg")

    print("\nground-projected pair per snapshot (spacing should be the laid value; edges = c0->c1 angle - 90):")
    for (name, d) in snaps:
        g0 = gp.to_ground(d[t0]['c']); g1 = gp.to_ground(d[t1]['c'])
        sp = np.linalg.norm(g1.mean(0) - g0.mean(0)) * 1000
        e0 = (math.degrees(math.atan2(g0[1][1] - g0[0][1], g0[1][0] - g0[0][0])) - 90 + 180) % 360 - 180
        e1 = (math.degrees(math.atan2(g1[1][1] - g1[0][1], g1[1][0] - g1[0][0])) - 90 + 180) % 360 - 180
        print(f"   {name:18s} spacing {sp:7.2f} mm  edges {e0:+6.3f} / {e1:+6.3f} deg  centre {t0} at ({g0.mean(0)[0]:+.3f}, {g0.mean(0)[1]:+.3f}) m")

    piv = [(n, d) for n, d in snaps if n.startswith('piv')]
    if len(piv) >= 2:
        print("\nlens-to-pivot lever from consecutive pivot snapshots (needs >= 2 deg of rotation):")
        Ls = []
        for (n0, d0), (n1, d1) in zip(piv, piv[1:]):
            g0 = np.vstack([gp.to_ground(d0[t0]['c']), gp.to_ground(d0[t1]['c'])])
            g1 = np.vstack([gp.to_ground(d1[t0]['c']), gp.to_ground(d1[t1]['c'])])
            m0, m1 = g0.mean(0), g1.mean(0)
            a0 = math.atan2(*(g0[4:].mean(0) - g0[:4].mean(0))[::-1])
            a1 = math.atan2(*(g1[4:].mean(0) - g1[:4].mean(0))[::-1])
            dpsi = a1 - a0
            if abs(math.degrees(dpsi)) < 2.0:
                continue
            L = (m1[1] - math.cos(dpsi) * m0[1]) / math.sin(dpsi) - m0[0]
            Ls.append(L)
            print(f"   {n0} -> {n1}: rotated {math.degrees(dpsi):+.2f} deg -> lever {L:.3f} m")
        if Ls:
            print(f"   lever mean {np.mean(Ls):.3f} m, sd {np.std(Ls):.3f}  (robot.yaml camera_offset)")


if __name__ == '__main__':
    main()
