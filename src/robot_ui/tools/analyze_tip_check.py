#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Where did the vision TIP really land? — offline analysis of a tip_check
session's Basler frames (2026-09-28).

    python3 analyze_tip_check.py log/apriltag_nav/tip_check/<ts>_<name>/

Every point of a touch session (tip_touch_ref_tag / tip_touch_cross_tags)
ends with a Basler frame taken at the Keyence standoff with the tip on the
target. A 20 mm tag36h11 marker lies on the target (the cross tag's
centre), so the frame shows the tip's real landing error: the image centre
IS the tip (that is how the tip is defined — the surface point the Basler
centre looks at at the case's zero standoff), the marker centre is the
target. This tool measures that offset and rotates it into WORLD axes:

    d_image  = (image centre − marker centre) / px_per_mm   (image x right, y down)
    d_flange = Rz(psi) · d_image          psi = the calibrated image roll about
                                          flange z (basler tip result, −179.2°)
    d_arm    = R_tcp · d_flange           R_tcp from the recorded TCP rpy
                                          (after the standoff correction)
    d_world  = R_AWᵀ · d_arm              R_AW from the point's /robot_pose + lift,
                                          the same transform the target used

So err_w = where the tip is − where it was sent, in map.yaml axes, as
measured by the camera — NOT the chain's own belief (the yaml's
tip_error_world_mm, which is ~0 by construction).

px_per_mm comes from the marker's own side (20 mm, --marker-mm). A frame in
which the marker is not fully visible (errors > ~15 mm push it off the
62 × 41 mm field) is registered against another frame of the SAME cross tag
in which it was found — the etched cross-tag cells make the scene rigid;
the method reproduced the detected centre to ≤ 0.06 mm on 2026-09-22's
frames. The marker is assumed laid on the cross-tag centre (tag 230 on
cross tag 0 measured 0.3 mm off from the etched grid, 2026-09-28).

The marker id is the point's target_info.marker_id (tip_touch_cross_tags);
for older sessions it is inferred from the nearest cross tag with
--markers (default = the current layout 0:219 … 5:224; the 2026-09-22
touch session had tag 230 on cross tag 0 — analyse it with --markers 0:230).

Output: <record_dir>/tip_image_error.csv and a per-stop / per-cross-tag
summary. Drive tags sharing a cross tag see the same physical point, so
their differences are the stop-pose error; the mean over all stops is the
chain's constant.
"""

import argparse
import csv
import glob
import math
import os
import sys

import cv2
import numpy as np
import yaml

import rospy
# transform_world_to_arm reads arm_node's private overrides through
# rospy.get_param; offline there is no node (and maybe no master), so every
# lookup takes its default — the tf_chain.yaml value, as arm_node does.
rospy.get_param = lambda name, default=None: default

from robot_ui import paths                                     # noqa: E402
from robot_ui.tip_check import _Pose, _rot, world_to_arm_R      # noqa: E402

DEFAULT_MARKERS = {0: 219, 1: 220, 2: 221, 3: 222, 4: 223, 5: 224}
TIP_RESULT = os.path.join(paths.WS_DIR, 'log', 'chain_calib', 'basler_tip_20260921',
                          'result_K20260922_HE20260922.yaml')
REF_TAGS_YAML = os.path.join(paths.WS_DIR, 'src', 'path_tag_locator', 'config',
                             'reference_tags.yaml')
DETECT_SCALE = 0.25


# ---------------------------------------------------------------- inputs
def load_psi(override):
    if override is not None:
        return float(override), 'command line'
    try:
        with open(TIP_RESULT) as f:
            return float(yaml.safe_load(f)['image_roll_deg']), os.path.basename(TIP_RESULT)
    except Exception as e:                                      # noqa: BLE001
        sys.exit(f'cannot read the image roll from {TIP_RESULT} ({e}); pass --psi-deg')


def cross_tags():
    with open(REF_TAGS_YAML) as f:
        doc = yaml.safe_load(f)
    return {int(r['id']): np.asarray(r['position_m'][:2], float) for r in doc['reference_tags']}


def parse_markers(text):
    out = {}
    for item in text.split(','):
        k, v = item.split(':')
        out[int(k)] = int(v)
    return out


def load_points(record_dir):
    pts = []
    for path in sorted(glob.glob(os.path.join(record_dir, 'r*_tag*.yaml'))):
        with open(path) as f:
            d = yaml.safe_load(f)
        if not isinstance(d, dict) or 'robot_pose' not in d:
            continue
        files = (d.get('capture') or {}).get('files') or []
        if not files:
            continue
        snap = d.get('after_correction') or d.get('before')
        if not snap or 'tcp_mm_deg' not in snap:
            continue
        d['_yaml'] = path
        d['_image'] = files[0]
        d['_tcp'] = snap['tcp_mm_deg']
        d['_stage'] = 'after_correction' if d.get('after_correction') else 'before'
        pts.append(d)
    return pts


# ---------------------------------------------------------------- image side
def detect_marker(detector, gray, marker_id):
    """(centre px, corners 4x2 px) of `marker_id` at full resolution, or None."""
    small = cv2.resize(gray, None, fx=DETECT_SCALE, fy=DETECT_SCALE, interpolation=cv2.INTER_AREA)
    for det in detector.detect(small):
        if int(det.tag_id) != int(marker_id):
            continue
        c = (np.asarray(det.corners, float) / DETECT_SCALE).astype(np.float32).reshape(-1, 1, 2)
        cv2.cornerSubPix(gray, c, (15, 15), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.01))
        c = c.reshape(4, 2).astype(float)
        return c.mean(axis=0), c
    return None


def texture(gray, ds=4):
    im = cv2.resize(gray.astype(np.float32), None, fx=1.0 / ds, fy=1.0 / ds,
                    interpolation=cv2.INTER_AREA)
    mu = cv2.blur(im, (9, 9))
    sd = np.sqrt(np.maximum(cv2.blur(im * im, (9, 9)) - mu * mu, 0))
    return np.clip(sd / 30.0, 0, 1).astype(np.float32)


def register_shift(ref_gray, gray, ds=4):
    """Shift s (full-res px) with content(gray) = content(ref) + s, and the
    best normalised correlation. Several template windows are tried because
    the overlap depends on which way the view moved."""
    ref, img = texture(ref_gray, ds), texture(gray, ds)
    h, w = img.shape
    wins = [(0.02, 0.62, 0.15, 0.85), (0.38, 0.98, 0.15, 0.85), (0.20, 0.80, 0.15, 0.85),
            (0.15, 0.85, 0.02, 0.60), (0.15, 0.85, 0.40, 0.98)]
    best = (-1.0, None)
    for y0, y1, x0, x1 in wins:
        Y0, Y1, X0, X1 = int(h * y0), int(h * y1), int(w * x0), int(w * x1)
        r = cv2.matchTemplate(ref, img[Y0:Y1, X0:X1], cv2.TM_CCOEFF_NORMED)
        _, mx, _, loc = cv2.minMaxLoc(r)
        if mx > best[0]:
            best = (mx, np.array([X0 - loc[0], Y0 - loc[1]], float) * ds)
    return best[1], best[0]


# ---------------------------------------------------------------- geometry
def image_to_world(d_img_mm, psi_deg, tcp_rpy_deg, pose, lift_m):
    """Image-plane vector (mm, x right / y down) -> world xy (mm)."""
    c, s = math.cos(math.radians(psi_deg)), math.sin(math.radians(psi_deg))
    d_flange = np.array([c * d_img_mm[0] - s * d_img_mm[1],
                         s * d_img_mm[0] + c * d_img_mm[1], 0.0])
    d_arm = _rot(tcp_rpy_deg) @ d_flange
    R_AW = world_to_arm_R(pose, lift_m)
    return (R_AW.T @ d_arm)[:2]


def axis_dev_deg(v_world):
    a = math.degrees(math.atan2(v_world[1], v_world[0]))
    return (a + 45.0) % 90.0 - 45.0


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('record_dir')
    ap.add_argument('--marker-mm', type=float, default=20.0, help='marker black-edge size')
    ap.add_argument('--psi-deg', type=float, default=None, help='image roll about flange z')
    ap.add_argument('--markers', default=','.join(f'{k}:{v}' for k, v in DEFAULT_MARKERS.items()),
                    help='cross tag -> marker id for sessions without target_info')
    ap.add_argument('--min-score', type=float, default=0.6, help='registration acceptance')
    args = ap.parse_args()

    psi, psi_src = load_psi(args.psi_deg)
    markers = parse_markers(args.markers)
    refs = cross_tags()
    pts = load_points(args.record_dir)
    if not pts:
        sys.exit(f'no captured points in {args.record_dir}')
    print(f'{len(pts)} captured point(s); image roll psi {psi:+.3f} deg ({psi_src}); '
          f'marker {args.marker_mm:g} mm')

    from dt_apriltags import Detector
    detector = Detector(families='tag36h11', quad_decimate=1.0)

    # pass 1: detect
    for d in pts:
        info = d.get('target_info') or {}
        tgt = np.asarray(d.get('target_world_m_plate_top', [0, 0, 0])[:2], float)
        ref = info.get('ref_tag')
        if ref is None:
            ref = min(refs, key=lambda k: np.linalg.norm(refs[k] - tgt))
        d['_ref'] = int(ref)
        d['_marker'] = int(info.get('marker_id', markers.get(int(ref), -1)))
        gray = cv2.imread(d['_image'], cv2.IMREAD_GRAYSCALE)
        if gray is None:
            d['_status'] = 'image missing'
            continue
        d['_gray'] = gray
        d['_wh'] = (gray.shape[1], gray.shape[0])
        found = detect_marker(detector, gray, d['_marker'])
        if found is not None:
            ctr, corners = found
            side = np.mean([np.linalg.norm(corners[(i + 1) % 4] - corners[i]) for i in range(4)])
            d.update(_ctr=ctr, _corners=corners, _ppm=side / args.marker_mm, _method='detected')

    ppm_all = [d['_ppm'] for d in pts if '_ppm' in d]
    ppm_med = float(np.median(ppm_all)) if ppm_all else None

    # pass 2: register the frames where the marker was not fully visible
    for d in pts:
        if '_ctr' in d or '_gray' not in d:
            continue
        donors = [e for e in pts if e.get('_ref') == d['_ref'] and '_ctr' in e]
        best = None
        for e in donors:
            shift, score = register_shift(e['_gray'], d['_gray'])
            if shift is not None and (best is None or score > best[0]):
                best = (score, e['_ctr'] + shift, os.path.basename(e['_image']))
        if best and best[0] >= args.min_score and ppm_med:
            d.update(_ctr=best[1], _ppm=ppm_med, _method=f'registered to {best[2]} ({best[0]:.2f})')
        else:
            d['_status'] = ('marker not found, no frame of the same cross tag to register to'
                            if not best else f'marker not found, registration score {best[0]:.2f}')

    # results
    rows = []
    for d in pts:
        rp = d['robot_pose']
        base = {'round': d.get('round'), 'tag': d.get('tag'), 'ref_tag': d['_ref'],
                'marker_id': d['_marker'], 'robot_x': rp['x'], 'robot_y': rp['y'],
                'theta_deg': rp['theta_deg'],
                'correction_dz_mm': (d.get('summary') or {}).get('correction_dz_mm', ''),
                'image': os.path.basename(d['_image'])}
        if '_ctr' not in d:
            rows.append(dict(base, method=d.get('_status', 'failed')))
            continue
        pose = _Pose(rp['x'], rp['y'], rp['theta_deg'])
        lift_m = float(rp.get('lift_mm') or 0.0) / 1000.0
        w, h = d['_wh']
        centre = np.array([(w - 1) / 2.0, (h - 1) / 2.0])
        d_img = (centre - d['_ctr']) / d['_ppm']
        e_w = image_to_world(d_img, psi, d['_tcp'][3:6], pose, lift_m)
        row = dict(base, method=d['_method'], px_per_mm=round(d['_ppm'], 2),
                   err_wx_mm=round(float(e_w[0]), 2), err_wy_mm=round(float(e_w[1]), 2),
                   err_xy_mm=round(float(np.hypot(*e_w)), 2))
        if '_corners' in d:
            edge = d['_corners'][1] - d['_corners'][0]
            v = image_to_world(edge, psi, d['_tcp'][3:6], pose, lift_m)
            row['marker_axis_dev_deg'] = round(axis_dev_deg(v), 2)
        rows.append(row)

    cols = ['round', 'tag', 'ref_tag', 'marker_id', 'method', 'px_per_mm', 'err_wx_mm',
            'err_wy_mm', 'err_xy_mm', 'marker_axis_dev_deg', 'correction_dz_mm',
            'robot_x', 'robot_y', 'theta_deg', 'image']
    out = os.path.join(args.record_dir, 'tip_image_error.csv')
    with open(out, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=cols)
        wr.writeheader()
        for r in rows:
            wr.writerow({k: r.get(k, '') for k in cols})

    print(f'\n{"pt":>9} {"ref":>3} {"err_wx":>7} {"err_wy":>7} {"|xy|":>6} {"dz":>5} '
          f'{"axis":>6}  method')
    for r in rows:
        if 'err_wx_mm' in r:
            print(f'r{r["round"]}_{r["tag"]:<6} {r["ref_tag"]:>3} {r["err_wx_mm"]:+7.2f} '
                  f'{r["err_wy_mm"]:+7.2f} {r["err_xy_mm"]:6.2f} '
                  f'{r["correction_dz_mm"] if r["correction_dz_mm"] != "" else "":>5} '
                  f'{r.get("marker_axis_dev_deg", ""):>6}  {r["method"]}')
        else:
            print(f'r{r["round"]}_{r["tag"]:<6} {r["ref_tag"]:>3}  —  {r["method"]}')

    ok = [r for r in rows if 'err_wx_mm' in r]
    if ok:
        E = np.array([[r['err_wx_mm'], r['err_wy_mm']] for r in ok])
        print(f'\nall {len(ok)}: mean ({E[:, 0].mean():+.2f}, {E[:, 1].mean():+.2f}) mm  '
              f'= the chain constant; |xy| max {np.hypot(E[:, 0], E[:, 1]).max():.2f}')
        for key, label in (('tag', 'stop'), ('ref_tag', 'cross tag')):
            print(f'\nper {label}: mean err (x, y), minus the overall mean, sd, n')
            for k in sorted({r[key] for r in ok}):
                G = np.array([[r['err_wx_mm'], r['err_wy_mm']] for r in ok if r[key] == k])
                m = G.mean(axis=0)
                sd = G.std(axis=0, ddof=1) if len(G) > 1 else np.array([np.nan, np.nan])
                print(f'  {label} {k:>4}: ({m[0]:+6.2f}, {m[1]:+6.2f})  '
                      f'rel ({m[0] - E[:, 0].mean():+6.2f}, {m[1] - E[:, 1].mean():+6.2f})  '
                      f'sd ({sd[0]:.2f}, {sd[1]:.2f})  n {len(G)}')
    bad = [r for r in rows if 'err_wx_mm' not in r]
    if bad:
        print(f'\n{len(bad)} point(s) without a result: ' +
              '; '.join(f'r{r["round"]}_tag{r["tag"]}: {r["method"]}' for r in bad))
    print(f'\nwritten {out}')


if __name__ == '__main__':
    main()
