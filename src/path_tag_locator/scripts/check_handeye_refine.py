#!/usr/bin/env python3
"""
check_handeye_refine.py — offline checks for the tag-scatter refinement in
path_tag_locator.handeye_calib (2026-09-18). No ROS master, no images:
synthetic (T_gripper2base, T_target2cam) pairs through the same functions
calibrate() calls after detection.

    python3 src/path_tag_locator/scripts/check_handeye_refine.py

Pins: tag_scatter is 0 for the true hand-eye and grows with an error;
refine_hand_eye recovers a 30 mm / 2 deg perturbed start to < 1 mm /
0.05 deg at zero noise; under 0.15-0.7 deg / 1-3 mm per-frame noise it
never raises the fit scatter, lowers the located-tag bias and per-view
rms on HELD-OUT views (the chain's deliverable) and costs at most
~1 mm / 0.1 deg of hand-eye truth; a 2026-09-18-shaped set (18 sweep
views + 14 small +/-10 deg rotations, 7 of them at 1.2 m with 4x the
noise) is no better than the 18 sweep views alone — why the bootstrap
samples are dropped.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

import cv2                                                                    # noqa: E402
from path_tag_locator.handeye_calib import refine_hand_eye, tag_scatter       # noqa: E402
from path_tag_locator.handeye_sweep import (SweepCfg, generate_views, view_T_cam2tag,  # noqa: E402
                                             BOOTSTRAP_AXES, _R_axis)
from path_tag_locator.geometry import invert_T, rpy_deg_to_R                  # noqa: E402

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


def T_from(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


def he_err(T, T_true):
    d = invert_T(T_true) @ T
    return (float(np.linalg.norm(d[:3, 3]) * 1000.0),
            math.degrees(math.acos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1))))


def noisy(T, rng, sigma_deg, sigma_m):
    ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
    Tn = T.copy()
    Tn[:3, :3] = _R_axis(ax, math.radians(rng.normal(0, sigma_deg))) @ T[:3, :3]
    Tn[:3, 3] = T[:3, 3] + rng.normal(0, sigma_m, size=3)
    return Tn


def opencv_best(T_g2b, T_t2c):
    best = None
    for m in (cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD,
              cv2.CALIB_HAND_EYE_ANDREFF, cv2.CALIB_HAND_EYE_DANIILIDIS):
        try:
            R, t = cv2.calibrateHandEye([T[:3, :3] for T in T_g2b], [T[:3, 3] for T in T_g2b],
                                        [T[:3, :3] for T in T_t2c], [T[:3, 3] for T in T_t2c], method=m)
        except Exception:
            continue
        T_c2g = T_from(R, np.asarray(t).flatten())
        resid = 0.0
        for i in range(len(T_g2b) - 1):
            A = invert_T(T_g2b[i + 1]) @ T_g2b[i]
            B = T_t2c[i + 1] @ invert_T(T_t2c[i])
            resid += float(np.linalg.norm(A @ T_c2g - T_c2g @ B))
        if best is None or resid < best[0]:
            best = (resid, invert_T(T_c2g))
    return best[1]


# the 2026-09-18 mount, roughly: camera 0.37 m from the flange, 180 deg spun
T_hc2ee_true = T_from(rpy_deg_to_R(1.2, -0.7, -178.0), [0.035, -0.337, -0.128])
T_ab2tag = T_from(rpy_deg_to_R(180.0, 0.0, 30.0), [-0.40, 1.01, -0.57])


def sweep_pairs(rng, sigma_deg, sigma_m, n_views=18):
    """(T_g2b, T_t2c) for the sweep's views, exact geometry + per-frame noise."""
    T_g2b, T_t2c = [], []
    for v in generate_views(SweepCfg(max_samples=n_views)):
        T_ct = view_T_cam2tag(0.3, v.distance_m, v.tilt_deg, v.azimuth_deg, v.spin_deg)
        T_ee = T_ab2tag @ invert_T(T_ct) @ T_hc2ee_true
        T_g2b.append(T_ee); T_t2c.append(noisy(T_ct, rng, sigma_deg, sigma_m))
    return T_g2b, T_t2c


def bootstrap_pairs(rng, depth, sigma_deg, sigma_m, angle_deg=10.0):
    T_ct0 = view_T_cam2tag(0.3, depth, 0, 0, 0)
    T_ee0 = T_ab2tag @ invert_T(T_ct0) @ T_hc2ee_true
    T_ee0[:3, 3] += np.array([0.2, 0.0, 0.0])          # tag off-centre, as in the session
    T_g2b, T_t2c = [], []
    for label, axis, sign in [('start', (1, 0, 0), 0.0)] + BOOTSTRAP_AXES:
        T_ee = T_ee0.copy(); T_ee[:3, :3] = T_ee0[:3, :3] @ _R_axis(axis, sign * math.radians(angle_deg))
        T_ct = invert_T(T_ee @ invert_T(T_hc2ee_true)) @ T_ab2tag
        T_g2b.append(T_ee); T_t2c.append(noisy(T_ct, rng, sigma_deg, sigma_m))
    return T_g2b, T_t2c


print('== tag_scatter')
rng = np.random.RandomState(0)
G, C = sweep_pairs(rng, 0.0, 0.0)
s0 = tag_scatter(T_hc2ee_true, G, C)
check(s0[0] < 1e-9 and s0[2] < 1e-6, f'true hand-eye, no noise: scatter {s0[0] * 1e3:.2e} mm')
T_bad = T_hc2ee_true.copy(); T_bad[:3, 3] += [0.03, 0.0, 0.0]
T_bad[:3, :3] = rpy_deg_to_R(0, 0, 2.0) @ T_bad[:3, :3]
s1 = tag_scatter(T_bad, G, C)
check(s1[0] > 0.005, f'30 mm / 2 deg off: scatter {s1[0] * 1e3:.1f} mm rms, normal {s1[2]:.2f} deg')

print('== refine_hand_eye, no noise')
T_ref = refine_hand_eye(T_bad, G, C)
mm, deg = he_err(T_ref, T_hc2ee_true)
check(mm < 1.0 and deg < 0.05, f'recovers the truth from the perturbed start: {mm:.3f} mm / {deg:.4f} deg')

def chain_err(T, G, C):
    """What the locator chain delivers with this hand-eye on held-out
    noisy views: (bias of the mean located tag position, per-view rms) mm."""
    P = np.array([(Tg @ invert_T(T) @ Tc)[:3, 3] for Tg, Tc in zip(G, C)])
    return (float(np.linalg.norm(P.mean(0) - T_ab2tag[:3, 3]) * 1000),
            float(np.sqrt((np.linalg.norm(P - T_ab2tag[:3, 3], axis=1) ** 2).mean()) * 1000))


print('== sweep views under per-frame noise: refinement vs the closed form, on held-out views')
# The refinement minimises the fixed tag's scatter, which is the chain's
# own metric; it trades ~1 mm / 0.1 deg of hand-eye truth for a smaller
# located-tag bias and per-view rms. It is not allowed to raise the scatter.
for sd, sm in ((0.7, 0.003), (0.3, 0.0015), (0.15, 0.001)):
    rows = []; worse = 0
    for seed in range(12):
        rng = np.random.RandomState(100 + seed)
        G, C = sweep_pairs(rng, sd, sm)
        Gh, Ch = sweep_pairs(rng, sd, sm)                 # held-out, fresh noise
        T_o = opencv_best(G, C); T_r = refine_hand_eye(T_o, G, C)
        if tag_scatter(T_r, G, C)[0] > tag_scatter(T_o, G, C)[0] + 1e-12:
            worse += 1
        rows.append(he_err(T_o, T_hc2ee_true) + he_err(T_r, T_hc2ee_true) + chain_err(T_o, Gh, Ch) + chain_err(T_r, Gh, Ch))
    m = np.array(rows).mean(0)
    print(f'   {sd} deg / {sm * 1000:.1f} mm: hand-eye ocv {m[0]:.1f} mm/{m[1]:.2f} deg, refined {m[2]:.1f} mm/{m[3]:.2f} deg; '
          f'located tag bias ocv {m[4]:.1f} -> ref {m[6]:.1f} mm, per-view rms ocv {m[5]:.1f} -> ref {m[7]:.1f} mm')
    check(worse == 0, f'  {sd} deg: the refinement never raises the fit scatter')
    check(m[6] <= m[4] and m[7] <= m[5], f'  {sd} deg: located-tag bias and per-view rms not worse than the closed form')
    check(m[2] < m[0] + 2.0 and m[3] < m[1] + 0.2, f'  {sd} deg: hand-eye truth error within 2 mm / 0.2 deg of the closed form')

print('== the 2026-09-18 set shape: 18 sweep views + 14 bootstrap rotations, 7 at 1.2 m with 4x noise')
mixed = []; sweep_only = []
for seed in range(8):
    rng = np.random.RandomState(seed)
    G, C = sweep_pairs(rng, 0.7, 0.003)
    Gh, Ch = sweep_pairs(rng, 0.7, 0.003)
    Gb1, Cb1 = bootstrap_pairs(rng, 1.2, 2.8, 0.012)       # small tag: 4x the noise
    Gb2, Cb2 = bootstrap_pairs(rng, 0.62, 0.7, 0.003)
    Ga, Ca = Gb1 + G[:5] + Gb2 + G[5:], Cb1 + C[:5] + Cb2 + C[5:]
    T_all = refine_hand_eye(opencv_best(Ga, Ca), Ga, Ca)
    T_sw = refine_hand_eye(opencv_best(G, C), G, C)
    mixed.append(chain_err(T_all, Gh, Ch)); sweep_only.append(chain_err(T_sw, Gh, Ch))
mb = np.mean([e[0] for e in mixed]); sb = np.mean([e[0] for e in sweep_only])
print(f'   located tag bias, all 32: {mb:.1f} mm; sweep-only 18: {sb:.1f} mm')
check(sb <= mb, 'dropping the bootstrap samples (sweep only) is at least as good — why they are dropped')

print('== refinement skipped when it cannot lower the scatter')
rng = np.random.RandomState(3)
G, C = sweep_pairs(rng, 0.0, 0.0)
T_ref = refine_hand_eye(T_hc2ee_true, G, C)
check(tag_scatter(T_ref, G, C)[0] <= 1e-9, 'starting at the truth stays at the truth')

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
