#!/usr/bin/env python3
"""Self-check of apriltag_nav.ground_plane.GroundPlane (no ROS needed).

    python3 src/apriltag_nav/tools/check_ground_plane.py [snapshot_dir]

Uses front_cam's fitted numbers (robot.yaml robot_camera.ground_plane) and
CameraInfo of 2026-09-08. With `snapshot_dir` (e.g. ~/calib_pair, raw
tag-pair snapshots `scan_*.txt` with tags 15/16 laid 0.150 m apart) it
also checks the correction against real data. Exit status 1 on failure.
"""
import glob
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from apriltag_nav.ground_plane import GroundPlane  # noqa: E402

FX, FY, CX, CY = 750.2402, 749.7717, 638.1971, 352.9817
D = [0.08307457715272903, -0.1101505309343338, 5.2812414651270956e-05, -0.0003344604920130223, 0.045477673411369324]
ROLL, PITCH, H, YAW = math.radians(1.228), math.radians(-0.504), 0.302, math.radians(-0.38)

checks = []


def check(name, cond, detail=''):
    checks.append(bool(cond))
    print(('PASS ' if cond else 'FAIL ') + name + (f'  [{detail}]' if detail else ''))


def edge(c):
    a = math.degrees(math.atan2(c[1][1] - c[0][1], c[1][0] - c[0][0])) - 90.0
    return (a + 90.0) % 180.0 - 90.0


def square(X, Y, s=0.09):
    h = s / 2
    return np.array([[X - h, Y - h], [X - h, Y + h], [X + h, Y + h], [X + h, Y - h]])


def main():
    # 1. level camera, no distortion: correct() is the identity
    lvl = GroundPlane(FX, FY, CX, CY, None, 0.0, 0.0, 0.30)
    px = lvl.project(square(0.2, 0.03))
    vc, vcen, g = lvl.correct(px, lvl.project([[0.2, 0.03]])[0])
    check('level camera: corrected corners == raw corners', np.abs(vc - px).max() < 1e-6)
    check('level camera: ground centre == (0.2, 0.03)', np.allclose(g, [0.2, 0.03], atol=1e-9))

    # 2. tilt only: a square-laid tag reads edge 0 and its true position after correction, anywhere
    gp = GroundPlane(FX, FY, CX, CY, D, ROLL, PITCH, H)
    worst_e = worst_p = 0.0
    raw_edges = {}
    for X in (-0.03, 0.0, 0.1, 0.155, 0.2):
        for Y in (-0.06, 0.0, 0.05):
            sq = square(X, Y)
            px = gp.project(sq)
            vc, vcen, g = gp.correct(px, gp.project([[X, Y]])[0])
            raw_edges[(X, Y)] = edge(px)
            worst_e = max(worst_e, abs(edge(vc)))
            worst_p = max(worst_p, np.abs(g - [X, Y]).max())
            ok = abs(vcen[0] - (CX + FX * X / H)) < 1e-6 and abs(vcen[1] - (CY + FY * Y / H)) < 1e-6
            if not ok:
                check(f'virtual centre px consistent with z/fx at X {X} Y {Y}', False)
    check('tilt: corrected edge of a square-laid tag is 0 everywhere', worst_e < 1e-3, f'worst {worst_e:.5f} deg')
    check('tilt: corrected centre is the true floor position', worst_p < 1e-6, f'worst {worst_p:.2e} m')
    check('tilt: uncorrected edge reads ~+0.67 deg at X=0.2, ~-0.21 at X=0 (the measured pattern)',
          abs(raw_edges[(0.2, 0.0)] - 0.674) < 0.02 and abs(raw_edges[(0.0, 0.0)] + 0.213) < 0.02,
          f'{raw_edges[(0.2, 0.0)]:+.3f} / {raw_edges[(0.0, 0.0)]:+.3f}')
    check('tilt: optical axis meets the floor ~7 mm from the nadir',
          abs(np.linalg.norm(gp.axis_offset_m()) - 0.0070) < 0.001, f'{np.linalg.norm(gp.axis_offset_m()) * 1000:.1f} mm')

    # 3. yaw: convention and effect
    cam = GroundPlane(FX, FY, CX, CY, D, ROLL, PITCH, H, yaw=YAW)     # the real camera, forward model
    for X in (0.0, 0.146):
        raw_e = edge(cam.project(square(X, 0.0)))
        tilt_b = edge(gp.project(square(X, 0.0)))
        check(f'yaw: raw edge of a body-square tag at X {X:.3f} = tilt bias {tilt_b:+.2f} + yaw {math.degrees(YAW):+.2f}',
              abs(raw_e - (tilt_b + math.degrees(YAW))) < 0.02, f'{raw_e:+.3f}')
    raw = cam.project(square(0.0, 0.0))
    cen = cam.project([[0.0, 0.0]])[0]
    check('yaw: tilt-only correction leaves the yaw (-0.38)', abs(edge(gp.correct(raw, cen)[0]) - math.degrees(YAW)) < 0.02)
    vc, _, g = cam.correct(raw, cen)
    check('yaw: full correction reads 0.000 at the body-frame origin', abs(edge(vc)) < 1e-3 and np.abs(g).max() < 1e-6)
    drift = []
    for X in (0.25, 0.05):
        raw = cam.project(square(X, 0.0))
        drift.append(cam.correct(raw, cam.project([[X, 0.0]])[0])[2][1])
    check('yaw: a tag on the travel line keeps a constant lateral over a 0.2 m drive', abs(drift[0] - drift[1]) < 1e-6)

    # 4. robustness: no D, D of length 8 (as published), short D
    for dd in (None, D + [0, 0, 0], D[:4]):
        g2 = GroundPlane(FX, FY, CX, CY, dd, ROLL, PITCH, H)
        px = g2.project(square(0.15, 0.0))
        vc, _, g = g2.correct(px, g2.project([[0.15, 0.0]])[0])
        check(f'D={None if dd is None else len(dd)}: round trip exact', abs(edge(vc)) < 1e-3 and np.abs(g - [0.15, 0]).max() < 1e-6)

    # 5. rectification maps: the virtual image of a floor point is where the level camera puts it
    mx, my = gp.rectify_maps(1280, 720)
    u, v = gp.to_virtual_px([[0.155, -0.02]])[0]
    raw_pt = gp.project([[0.155, -0.02]])[0]
    check('rectify maps: virtual pixel of a floor point samples its raw pixel',
          abs(mx[int(round(v)), int(round(u))] - raw_pt[0]) < 1.5 and abs(my[int(round(v)), int(round(u))] - raw_pt[1]) < 1.5)

    # 6. real snapshots (optional): tag 16's corrected edge vs the pair's centre line
    sdir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/calib_pair')
    files = sorted(glob.glob(os.path.join(sdir, 'scan_*.txt')))
    if files:
        def parse(fn):
            out = {}
            for tid, cs in re.findall(r"id: (\d+).*?corners: \[([^\]]+)\]", open(fn).read(), re.S):
                out[int(tid)] = np.array([float(v) for v in cs.split(',')]).reshape(4, 2)
            return out

        def line_angle(c15, c16):
            return math.degrees(math.atan2(c16[1] - c15[1], c16[0] - c15[0]))
        sp, dif, raw_dif = [], [], []
        for fn in files:
            d = parse(fn)
            if 15 not in d or 16 not in d:
                continue
            g15, g16 = gp.to_ground(d[15]), gp.to_ground(d[16])
            sp.append(np.linalg.norm(g16.mean(0) - g15.mean(0)) * 1000)
            dif.append(edge(g16) - line_angle(g15.mean(0), g16.mean(0)))
            raw_dif.append(edge(d[16]) - line_angle(d[15].mean(0), d[16].mean(0)))
        if sp:
            check(f'real scans ({len(sp)}): corrected spacing 150.0 +- 0.5 mm', max(abs(s - 150.0) for s in sp) < 0.5, f'{min(sp):.2f}..{max(sp):.2f}')
            check('real scans: tag 16 corrected edge within 0.15 deg of the pair line at every position',
                  max(abs(x) for x in dif) < 0.15, f'corrected {min(dif):+.3f}..{max(dif):+.3f}; raw {min(raw_dif):+.3f}..{max(raw_dif):+.3f}')
    else:
        print(f'(no snapshots in {sdir}; real-data checks skipped)')

    n = sum(1 for c in checks if not c)
    print(f'\n{len(checks) - n}/{len(checks)} checks passed')
    return 1 if n else 0


if __name__ == '__main__':
    sys.exit(main())
