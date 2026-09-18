#!/usr/bin/env python3
"""
check_basler_tip.py — offline checks for chain_calib.basler_tip (2026-09-18).
No ROS master, no robot.

    python3 src/chain_calib/scripts/check_basler_tip.py

Pins, on a synthetic plant (the A4 20 mm sheet in the arm frame, a
hand-eye, a true vision tip 8 mm / 2 deg off the design value):
  * fit_similarity recovers the sheet point under the frame centre and
    the image roll from tag corners (one tag or several), to 0.01 mm at
    zero noise; a rendered 5472 x 3648 Basler frame (cv2.aruco tag36h11,
    1300 px per 20 mm tag) goes through detect_basler_corners at 1/4 and
    lands the centre within 0.05 mm;
  * fit_tip recovers p_tip / psi exactly at zero noise; with 0.3 px
    hand_cam corner noise, 2 px Basler corner noise and 0.02 deg arm
    repeatability it is within 1.5 mm / 0.3 deg over 8 basler views, the
    jackknife tracks the truth error, and a hand-eye error of 5 mm shows
    up as a 5 mm shift of the tip (the floor is the hand_cam chain);
  * a mirrored corner order is rejected (det < 0 fixed, not silently
    accepted) — the tip lands on the wrong side otherwise.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(HERE)), 'path_tag_locator', 'src'))

import cv2                                                                       # noqa: E402
from chain_calib import basler_tip as BT                                         # noqa: E402
from chain_calib.sheet import load_sheet, project_sheet                          # noqa: E402
from path_tag_locator.geometry import invert_T, matrix_m_to_pose_fr5, rpy_deg_to_R  # noqa: E402

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


def T_from(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


SHEET = load_sheet(os.path.join(os.path.dirname(HERE), 'sheet', 'A4_tag20_201-230_5x6_layout.json'))
# the sheet flat on the plate, 0.57 m below the arm base, +z into the plate
T_ab2W_true = T_from(rpy_deg_to_R(180.0, 0.0, 25.0), [-0.45, 0.95, -0.572])
T_hc2ee = T_from(rpy_deg_to_R(0.5, -0.4, -179.3), [0.036, -0.335, -0.152])       # today's hand-eye
p_tip_true = np.array([4.0, -258.0, 231.0])                                         # mm, 8 mm off the design
psi_true = 2.0
BASLER_WH = (5472, 3648)
PX_PER_MM = 65.0                                                                   # ~1300 px per 20 mm tag
K_hand = np.array([[640.0, 0, 640.0], [0, 640.0, 360.0], [0, 0, 1.0]])


def tip_T():
    T = np.eye(4); T[:3, :3] = BT._Rz(math.radians(psi_true)); T[:3, 3] = p_tip_true / 1e3; return T


def basler_view(o_W_mm, theta_deg):
    """Flange pose that puts the frame centre over sheet point o_W with
    the image x axis at theta_deg in W (tip z along the sheet normal)."""
    T_W2tip = T_from(rpy_deg_to_R(0.0, 0.0, theta_deg), [o_W_mm[0] / 1e3, o_W_mm[1] / 1e3, 0.0])
    T_ab2tip = T_ab2W_true @ T_W2tip
    return T_ab2tip @ invert_T(tip_T())


def basler_corners(A, noise_px, rng):
    """Corners of the sheet tags as the Basler at flange pose A images them."""
    T_ab2tip = A @ tip_T()
    T_tip2W = invert_T(T_ab2tip) @ T_ab2W_true
    out = {}
    w, h = BASLER_WH
    for k in SHEET.ids:
        pts = (T_tip2W[:3, :3] @ SHEET.corners_W(k).T).T + T_tip2W[:3, 3]      # tip frame, m
        px = pts[:, :2] * 1e3 * PX_PER_MM + np.array([(w - 1) / 2.0, (h - 1) / 2.0])
        if (px[:, 0] > 0).all() and (px[:, 0] < w).all() and (px[:, 1] > 0).all() and (px[:, 1] < h).all():
            out[k] = px + rng.normal(0, noise_px, size=(4, 2))
    return out


def hand_samples(n, noise_px, rng, arm_noise_deg=0.0):
    out = []
    for i in range(n):
        # camera 0.30 m over the sheet centre, spun a bit each time
        T_W2hc = T_from(rpy_deg_to_R(rng.normal(0, 2), rng.normal(0, 2), 20 * i), [0.08, 0.10, -0.30])
        T_ab2hc = T_ab2W_true @ T_W2hc
        A = T_ab2hc @ T_hc2ee
        T_hc2W = invert_T(T_ab2hc) @ T_ab2W_true
        corners = project_sheet(T_hc2W, SHEET, K_hand)
        corners = {k: c + rng.normal(0, noise_px, size=(4, 2)) for k, c in corners.items()}
        An = A.copy()
        if arm_noise_deg:
            An[:3, :3] = rpy_deg_to_R(*rng.normal(0, arm_noise_deg, 3)) @ A[:3, :3]
        out.append(BT.HandSample(f'h{i}', matrix_m_to_pose_fr5(An), corners))
    return out


def basler_samples(views, noise_px, rng, arm_noise_deg=0.0):
    out = []
    for i, (o, th) in enumerate(views):
        A = basler_view(o, th)
        c = basler_corners(A, noise_px, rng)
        An = A.copy()
        if arm_noise_deg:
            An[:3, :3] = rpy_deg_to_R(*rng.normal(0, arm_noise_deg, 3)) @ A[:3, :3]
        out.append(BT.BaslerSample(f'b{i}', matrix_m_to_pose_fr5(An), c, BASLER_WH))
    return out


VIEWS = [((40.0, 40.0), 0.0), ((80.0, 40.0), 30.0), ((120.0, 80.0), -30.0), ((40.0, 120.0), 60.0),
         ((160.0, 160.0), -60.0), ((80.0, 200.0), 90.0), ((0.0, 0.0), 15.0), ((160.0, 0.0), -45.0)]

print('== fit_similarity')
rng = np.random.RandomState(0)
b = basler_samples([((45.0, 37.0), 12.0)], 0.0, rng)[0]
o, th, ppm, rms, ids = BT.fit_similarity(b.corners, SHEET, b.image_wh)
check(np.linalg.norm(o * 1e3 - [45.0, 37.0]) < 0.01 and abs(th - 12.0) < 0.01 and abs(ppm - PX_PER_MM) < 0.01,
      f'centre ({o[0] * 1e3:.2f}, {o[1] * 1e3:.2f}) mm, roll {th:.2f} deg, {ppm:.2f} px/mm from tags {ids} (rms {rms:.3f} px)')
mirrored = {k: c[[1, 0, 3, 2]] for k, c in b.corners.items()}
o2, th2, ppm2, rms2, _ = BT.fit_similarity(mirrored, SHEET, b.image_wh)
check(rms2 > 50, f'mirrored corner order does not fit (rms {rms2:.0f} px) — it is not silently accepted')

print('== a rendered Basler frame through detect_basler_corners')
frame = np.full((BASLER_WH[1], BASLER_WH[0]), 200, np.uint8)
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
A = basler_view((40.0, 40.0), 10.0)
truth = basler_corners(A, 0.0, rng)
for k, c in truth.items():
    tag = cv2.aruco.generateImageMarker(dic, k, 800) if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dic, k, 800)
    tag = np.rot90(tag, 2)            # aruco's APRILTAG_36h11 bitmap is upside down vs dt_apriltags' canonical corner order
    # the rendered marker's outer edge is the black edge; map its corners TL,TR,BR,BL onto ours (BL,BR,TR,TL)
    src = np.array([[0, 0], [799, 0], [799, 799], [0, 799]], np.float32)
    dst = np.array([c[3], c[2], c[1], c[0]], np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(tag, M, (BASLER_WH[0], BASLER_WH[1]), flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_TRANSPARENT)
    mask = cv2.warpPerspective(np.full((800, 800), 255, np.uint8), M, (BASLER_WH[0], BASLER_WH[1]))
    frame[mask > 0] = warped[mask > 0]
frame = cv2.GaussianBlur(frame, (5, 5), 1.0)
det = BT.detect_basler_corners(frame)
check(set(det) == set(truth), f'detected {sorted(det)} (rendered {sorted(truth)})')
err = max(np.abs(det[k] - truth[k]).max() for k in truth) if set(det) == set(truth) else 1e9
check(err < 2.0, f'corners within {err:.2f} px of the rendering at full resolution')
o, th, ppm, rms, ids = BT.fit_similarity(det, SHEET, BASLER_WH)
check(np.linalg.norm(o * 1e3 - [40.0, 40.0]) < 0.05 and abs(th - 10.0) < 0.05,
      f'frame centre at ({o[0] * 1e3:.3f}, {o[1] * 1e3:.3f}) mm, roll {th:.3f} deg from the rendered frame')

print('== fit_tip, zero noise')
rng = np.random.RandomState(1)
hand = BT.resolve_hand(hand_samples(5, 0.001, rng), SHEET, K_hand, None, T_hc2ee)   # 0.001 px: exact planar sets can NaN in LM
bas = BT.resolve_basler(basler_samples(VIEWS, 0.0, rng), SHEET)
res = BT.fit_tip(hand, bas)
print(BT.summarize(res, T_hc2ee))
check(np.linalg.norm(res.p_tip_mm - p_tip_true) < 0.01 and abs(res.psi_deg - psi_true) < 0.01,
      f'tip {np.round(res.p_tip_mm, 2)} / psi {res.psi_deg:.3f} == truth')
check(res.sheet_scatter_mm < 0.05, f'hand samples agree on the sheet pose ({res.sheet_scatter_mm:.3f} mm)')

print('== fit_tip under noise (0.3 px hand_cam, 2 px Basler, 0.02 deg arm)')
errs = []
for seed in range(8):
    rng = np.random.RandomState(10 + seed)
    hand = BT.resolve_hand(hand_samples(6, 0.3, rng, 0.02), SHEET, K_hand, None, T_hc2ee)
    bas = BT.resolve_basler(basler_samples(VIEWS, 2.0, rng, 0.02), SHEET)
    res = BT.fit_tip(hand, bas)
    errs.append((np.linalg.norm(res.p_tip_mm - p_tip_true), abs(res.psi_deg - psi_true),
                 res.sheet_scatter_mm, res.rms_mm, float(np.linalg.norm(res.jackknife_sd_mm))))
e = np.array(errs)
print(f'   tip error mean {e[:, 0].mean():.2f} max {e[:, 0].max():.2f} mm; psi {e[:, 1].max():.3f} deg; '
      f'sheet scatter {e[:, 2].mean():.2f} mm; fit rms {e[:, 3].mean():.2f} mm; jackknife {e[:, 4].mean():.2f} mm')
check(e[:, 0].max() < 1.5 and e[:, 1].max() < 0.3, 'tip within 1.5 mm / 0.3 deg of the truth on every seed')
check(0.3 * e[:, 4].mean() < e[:, 0].mean() < 3 * e[:, 4].mean(), 'jackknife sd is the right order for the truth error')

print('== a hand-eye error moves the tip 1:1 (the hand_cam chain is the floor)')
rng = np.random.RandomState(3)
H_bad = T_hc2ee.copy(); H_bad[:3, 3] += [0.005, 0.0, 0.0]
hand = BT.resolve_hand(hand_samples(5, 0.001, rng), SHEET, K_hand, None, H_bad)
bas = BT.resolve_basler(basler_samples(VIEWS, 0.0, rng), SHEET)
res = BT.fit_tip(hand, bas)
d = np.linalg.norm(res.p_tip_mm - p_tip_true)
check(1.0 < d < 7.0 and res.sheet_scatter_mm > 1.0,
      f'5 mm hand-eye translation error -> tip {d:.1f} mm off, AND the spun hand samples disagree on the sheet '
      f'pose by {res.sheet_scatter_mm:.2f} mm rms (the error rotates with the spin: the scatter line is the tell)')

print('== session round trip')
import tempfile
with tempfile.TemporaryDirectory() as td:
    BT.save_session(td, hand, bas, {'note': 'x'})
    h2, b2, m2 = BT.load_session(td)
    check(len(h2) == len(hand) and len(b2) == len(bas) and m2['note'] == 'x'
          and np.allclose(b2[0].corners[list(b2[0].corners)[0]], bas[0].corners[list(bas[0].corners)[0]]),
          'save_session / load_session round trip')

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
