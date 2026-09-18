#!/usr/bin/env python3
"""
check_handeye_sweep.py — offline checks for path_tag_locator.handeye_sweep
(2026-09-14). No ROS master needed.

Run from the workspace root (any sourced shell):
    python3 src/path_tag_locator/scripts/check_handeye_sweep.py

What it pins:
  * view_T_cam2tag: the tag stays on the optical axis at the requested
    range, the camera sits at the requested polar angle from the tag
    normal, and the spin only rotates about the optical axis.
  * plan_sweep: every accepted flange target re-images the tag centred
    with the intended tilt (round trip through the same hand-eye); every
    tool point clears the plate; views that would put the vision tip
    under the clearance, outside the xy window or beyond reach are
    rejected with the reason; nearest-first ordering keeps every hop
    inside a few clamp chunks.
  * SweepRunner against a fake arm + fake detector with a limited field
    of view: squares up first, captures the square view, moves in
    MoveL chunks that never exceed the clamps, skips views where the tag
    is not seen, survives a move failure, honours cancel, returns to the
    start pose, and the progress stream has the phases the UI reads.
  * A wrong (perturbed) aiming hand-eye only mis-centres the tag: views
    are skipped, never captured off-tag, and nothing dips below the
    plate — and it does NOT trip the divergence guard.
  * (2026-09-18) An aiming hand-eye of a DIFFERENT mount (the camera
    was moved) diverges or stalls the square-up: the guard retreats to
    the best pose seen before the tag is lost, `bootstrap: never` fails
    with the manual procedure named, `auto` captures the start view +
    six flange rotations, solves a provisional hand-eye with the real
    cv2.calibrateHandEye under detection noise, squares up with it and
    completes the sweep with >= 8 usable samples at both 0.55 m and
    1.2 m starting heights; `always` / no file bootstrap without trying
    the file; a camera too close to the tag is refused; the 2026-09-18
    session's numbers (xy 289 -> 373 mm, tilt 3 -> 7.7 deg) trip the
    guard while near-convergence noise does not.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

from path_tag_locator.handeye_sweep import (SweepCfg, SweepRunner, generate_views,  # noqa: E402
                                             plan_sweep, view_T_cam2tag, _align_error_mm,
                                             BOOTSTRAP_AXES, MANUAL_PROCEDURE)
from path_tag_locator.align import alignment_metrics                       # noqa: E402
from path_tag_locator.geometry import (invert_T, matrix_m_to_pose_fr5,      # noqa: E402
                                        pose_fr5_to_matrix_m, rpy_deg_to_R)

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


def T_from(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t; return T


# A plausible hand-eye: camera 80 mm ahead of the flange along its z,
# 120 mm off to the side, looking along flange +z with a 180 deg spin.
T_ee2hc_true = T_from(rpy_deg_to_R(0.0, 0.0, 180.0), [0.12, -0.05, 0.08])
T_hc2ee_true = invert_T(T_ee2hc_true)
# Tag on the plate: 0.60 m below the arm base, 0.85 m out in +y, face up,
# with +z INTO the plate (so a camera above it sees tilt 0 at identity).
T_ab2tag = T_from(rpy_deg_to_R(180.0, 0.0, 30.0), [0.05, 0.85, -0.60])

# ================================================================ 1. views
print('== view geometry')
for d, tilt, az, spin in [(0.4, 0.0, 0.0, 0.0), (0.5, 20.0, 90.0, 0.0), (0.35, 12.0, 270.0, 30.0)]:
    T = view_T_cam2tag(0.3, d, tilt, az, spin)
    m = alignment_metrics(T)
    cam_in_tag = invert_T(T)[:3, 3]
    polar = math.degrees(math.acos(abs(cam_in_tag[2]) / np.linalg.norm(cam_in_tag)))
    check(m.xy_offset_m < 1e-9 and abs(m.z_distance_m - d) < 1e-9,
          f'd={d} tilt={tilt} az={az}: tag on the optical axis at range {d}')
    check(abs(m.tilt_deg - tilt) < 1e-6, f'  tilt metric {m.tilt_deg:.3f} == {tilt}')
    check(abs(polar - tilt) < 1e-6 and abs(np.linalg.norm(cam_in_tag) - d) < 1e-9,
          f'  camera at polar {polar:.2f} deg, radius {np.linalg.norm(cam_in_tag):.3f} m from the tag')
T0 = view_T_cam2tag(0.3, 0.4, 12.0, 45.0, 0.0)
T1 = view_T_cam2tag(0.3, 0.4, 12.0, 45.0, 25.0)
check(np.allclose(invert_T(T0)[:3, 3], invert_T(T1)[:3, 3]), 'spin leaves the camera position unchanged')

views = generate_views(SweepCfg())
check(len(views) == 24, f'default grid subsampled to {len(views)} views')
check(len({(v.distance_m) for v in views}) == 3 and len({v.tilt_deg for v in views}) == 3
      and len({v.spin_deg for v in views}) == 3, 'all distances, tilts and spins represented')
allv = generate_views(SweepCfg(max_samples=0))
check(len(allv) == 27, f'full grid is 3 x (1 + 2*4) = {len(allv)} views (spins round-robin)')

# ================================================================ 2. plan
print('== plan_sweep: round trip + safety rules')
cfg = SweepCfg()
T_cam2tag_sq = view_T_cam2tag(0.3, cfg.distances_m[0], 0, 0, 0)
T_start = T_ab2tag @ invert_T(T_cam2tag_sq) @ T_hc2ee_true
ordered, rejected = plan_sweep(T_ab2tag, T_hc2ee_true, T_cam2tag_sq, T_start, cfg)
print(f'  planned {len(ordered)}, rejected {len(rejected)}: '
      f'{sorted(set(v.reject_reason.split(" ")[0] for v in rejected))}')
check(len(ordered) >= 15, f'{len(ordered)} of 24 views survive the safety rules')
ok_rt = True
for v in ordered:
    T_seen = invert_T(T_ab2tag) @ v.T_ab2ee @ invert_T(T_hc2ee_true)   # T_tag2cam
    T_seen = invert_T(T_seen)                                          # T_cam2tag
    m = alignment_metrics(T_seen)
    if m.xy_offset_m > 1e-6 or abs(m.tilt_deg - v.tilt_deg) > 1e-4 or abs(m.z_distance_m - v.distance_m) > 1e-6:
        ok_rt = False
check(ok_rt, 'every accepted flange target re-images the tag centred, at its tilt and range')
n = -T_ab2tag[:3, 2]     # up
tag_o = T_ab2tag[:3, 3]
low = []
for v in ordered:
    for p in cfg.tool_points_mm:
        P = v.T_ab2ee[:3, :3] @ (np.array(p) / 1000.0) + v.T_ab2ee[:3, 3]
        low.append(float(np.dot(n, P - tag_o)))
check(min(low) >= cfg.min_clearance_m - 1e-9, f'lowest tool point {min(low) * 1000:.0f} mm above the plate (min {cfg.min_clearance_m * 1000:.0f})')
# tighten the rules -> more rejections, with reasons
_, rej2 = plan_sweep(T_ab2tag, T_hc2ee_true, T_cam2tag_sq, T_start, SweepCfg(min_clearance_m=0.40))
check(len(rej2) > len(rejected) and all('clearance' in v.reject_reason for v in rej2 if 'clearance' in v.reject_reason)
      and any('clearance' in v.reject_reason for v in rej2), f'clearance 0.40 m rejects more views ({len(rej2)}), reason named')
_, rej3 = plan_sweep(T_ab2tag, T_hc2ee_true, T_cam2tag_sq, T_start, SweepCfg(max_xy_from_start_m=0.05))
check(any('xy excursion' in v.reject_reason for v in rej3), 'xy window rejects the far-tilted views')
_, rej4 = plan_sweep(T_ab2tag, T_hc2ee_true, T_cam2tag_sq, T_start, SweepCfg(max_flange_reach_m=0.9))
check(any('reach' in v.reject_reason for v in rej4), 'reach cap rejects, reason named')
hops = []
T_prev = T_start
for v in ordered:
    d = invert_T(T_prev) @ v.T_ab2ee
    hops.append(float(np.linalg.norm(d[:3, 3])))
    T_prev = v.T_ab2ee
check(max(hops) < 0.45, f'nearest-first: longest hop {max(hops):.2f} m (<= ~2 chunks of {cfg.max_step_m})')


# ================================================================ 3. runner
class FakeArm:
    def __init__(self, T):
        self.T = T.copy(); self.moves = []; self.fail_on = set(); self.n = 0
    def get_tcp(self):
        return matrix_m_to_pose_fr5(self.T)
    def move(self, pose):
        self.n += 1
        if self.n in self.fail_on:
            raise RuntimeError('MoveL error 112')
        T_new = pose_fr5_to_matrix_m(pose)
        d = invert_T(self.T) @ T_new
        self.moves.append((float(np.linalg.norm(d[:3, 3])),
                           math.degrees(math.acos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1)))))
        self.T = T_new


class FakeCam:
    """Sees the tag when it is inside a cone (half-FOV) and not too oblique."""
    def __init__(self, arm, T_hc2ee_actual, half_fov_deg=28.0, max_tilt_deg=40.0):
        self.arm = arm; self.T_hc2ee = T_hc2ee_actual
        self.half_fov = half_fov_deg; self.max_tilt = max_tilt_deg; self.seen = []
    def detect(self):
        T_cam2tag = invert_T(self.arm.T @ invert_T(self.T_hc2ee)) @ T_ab2tag
        t = T_cam2tag[:3, 3]
        if t[2] <= 0.05:
            return None
        ang = math.degrees(math.atan2(math.hypot(t[0], t[1]), t[2]))
        m = alignment_metrics(T_cam2tag)
        if ang > self.half_fov or m.tilt_deg > self.max_tilt:
            return None
        self.seen.append((m.xy_offset_m, m.tilt_deg))
        self.last_T = T_cam2tag
        return T_cam2tag


def _noisy(T, rng, sigma_deg, sigma_m):
    """A detection with per-frame noise: a small random rotation and a
    translation jitter (single raw frame, as the node's capture is)."""
    ax = rng.normal(size=3); ax /= np.linalg.norm(ax)
    a = math.radians(rng.normal(0.0, sigma_deg))
    from path_tag_locator.handeye_sweep import _R_axis
    Tn = T.copy()
    Tn[:3, :3] = _R_axis(ax, a) @ T[:3, :3]
    Tn[:3, 3] = T[:3, 3] + rng.normal(0.0, sigma_m, size=3)
    return Tn


def make_solve(caps, rng, sigma_deg=0.7, sigma_m=0.003):
    """The node's _solve_provisional against the fake: cv2.calibrateHandEye
    over (T_ab2ee, T_cam2tag) pairs captured from `since` on, the tag
    observation noised per frame. Same conventions as handeye_calib.calibrate."""
    import cv2

    def solve(since):
        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for T_ee, _, T_ct in caps[since:]:
            if T_ct is None:
                continue
            Tn = _noisy(T_ct, rng, sigma_deg, sigma_m)
            R_g2b.append(T_ee[:3, :3]); t_g2b.append(T_ee[:3, 3].copy())
            R_t2c.append(Tn[:3, :3]); t_t2c.append(Tn[:3, 3].copy())
        best = None
        for m in (cv2.CALIB_HAND_EYE_TSAI, cv2.CALIB_HAND_EYE_PARK, cv2.CALIB_HAND_EYE_HORAUD,
                  cv2.CALIB_HAND_EYE_ANDREFF, cv2.CALIB_HAND_EYE_DANIILIDIS):
            try:
                R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
            except Exception:
                continue
            T_c2g = T_from(R_c2g, np.asarray(t_c2g).flatten())
            if abs(np.linalg.det(R_c2g) - 1) > 1e-3:
                continue
            # AX = XB residual, as handeye_calib._axxb_residual
            resid = 0.0
            for i in range(len(R_g2b) - 1):
                A = invert_T(T_from(R_g2b[i + 1], t_g2b[i + 1])) @ T_from(R_g2b[i], t_g2b[i])
                B = T_from(R_t2c[i + 1], t_t2c[i + 1]) @ invert_T(T_from(R_t2c[i], t_t2c[i]))
                resid += float(np.linalg.norm(A @ T_c2g - T_c2g @ B))
            if best is None or resid < best[0]:
                best = (resid, T_c2g)
        return None if best is None else invert_T(best[1])
    return solve


def he_error(T_est, T_true=None):
    """(mm, deg) between two hand-eye estimates."""
    T_true = T_hc2ee_true if T_true is None else T_true
    d = invert_T(T_true) @ T_est
    ang = math.degrees(math.acos(np.clip((np.trace(d[:3, :3]) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(d[:3, 3]) * 1000.0), float(ang)


def run_case(T_hc2ee_aim, start_offset=(0.06, -0.04, 0.10), cancel_after=None, fail_on=(), cfg=None,
             seed=0, with_solve=False, start_depth=None):
    cfg = cfg or SweepCfg()
    # operator parks the camera roughly over the tag: square view + an offset
    T_sq = view_T_cam2tag(0.3, start_depth or 0.45, 0, 0, 0)
    T_ee0 = T_ab2tag @ invert_T(T_sq) @ T_hc2ee_true
    T_ee0[:3, 3] += np.array(start_offset)
    arm = FakeArm(T_ee0); arm.fail_on = set(fail_on)
    cam = FakeCam(arm, T_hc2ee_true)
    caps = []; events = []
    state = {'cancel': False}
    rng = np.random.RandomState(seed)

    def capture():
        caps.append((arm.T.copy(), cam.seen[-1] if cam.seen else None,
                     cam.last_T.copy() if cam.seen else None))
        return True, f'sample {len(caps)}'

    def progress(d):
        events.append(d)
        if cancel_after is not None and d.get('phase') == 'sample' and d.get('index') == cancel_after:
            state['cancel'] = True

    extra = {}
    if with_solve:
        def discard(since):
            del caps[since:]
        extra = dict(n_samples=lambda: len(caps), solve=make_solve(caps, rng), discard=discard)
    r = SweepRunner(cfg, get_tcp=arm.get_tcp, move=arm.move, detect=cam.detect, capture=capture,
                    cancelled=lambda: state['cancel'], log_info=lambda s: None, log_warn=lambda s: None,
                    progress=progress, **extra)
    res = r.run(T_hc2ee_aim)
    return res, arm, cam, caps, events


print('== SweepRunner with the true hand-eye')
res, arm, cam, caps, events = run_case(T_hc2ee_true)
print('  ', res.summary())
check(res.error is None and not res.cancelled, 'runs to completion')
check(1 <= res.align_iterations <= 5, f'squared up in {res.align_iterations} iteration(s) (12 cm start offset, 10 cm clamp)')
check(events[0]['phase'] == 'align' and any(e['phase'] == 'start' for e in events)
      and events[-1]['phase'] == 'finished', 'progress phases: align … start … finished')
sq = [e for e in events if e['phase'] == 'sample' and e['label'] == 'square']
check(len(sq) == 1 and sq[0]['ok'] and caps and caps[0][1][0] < 0.004 and caps[0][1][1] < 1.0,
      'sample 0 is the square view (tag centred, tilt < 1 deg)')
check(res.n_captured == res.n_planned + 1 and res.n_skipped == 0,
      f'every planned view captured ({res.n_captured} = {res.n_planned} + square)')
check(max(m[0] for m in arm.moves) <= SweepCfg().max_step_m + 1e-9
      and max(m[1] for m in arm.moves) <= SweepCfg().max_step_deg + 1e-6,
      f'no MoveL chunk exceeds the clamps (max {max(m[0] for m in arm.moves):.3f} m / {max(m[1] for m in arm.moves):.1f} deg)')
lows = []
for T_c, _, _ in caps:
    for p in SweepCfg().tool_points_mm:
        P = T_c[:3, :3] @ (np.array(p) / 1000.0) + T_c[:3, 3]
        lows.append(float(np.dot(n, P - tag_o)))
check(min(lows) >= SweepCfg().min_clearance_m - 1e-6, f'every captured pose keeps the tool {min(lows) * 1000:.0f} mm above the plate')
tilts = sorted({round(c[1][1]) for c in caps})
check(len(tilts) >= 3 and max(tilts) >= 20, f'captured tilts {tilts} deg — rotation diversity')
check(np.allclose(arm.T[:3, 3], pose_fr5_to_matrix_m(res.start_tcp)[:3, 3], atol=1e-6),
      'arm returned to the start pose')

print('== move failure, cancel')
res, arm, cam, caps, events = run_case(T_hc2ee_true, fail_on=(6,))
check(res.error is None and res.n_move_failed == 1 and res.n_captured >= res.n_planned - 1,
      f'one failed move is skipped, the rest continue ({res.summary()})')
res, arm, cam, caps, events = run_case(T_hc2ee_true, cancel_after=3)
check(res.cancelled and res.n_captured <= 5 and events[-1]['phase'] == 'finished' and events[-1]['cancelled'],
      f'cancel after view 3 stops the sweep ({res.n_captured} captured) and still reports finished')

print('== wrong aiming hand-eye (interim file): skips, never captures off-tag, never dips')
T_bad = T_hc2ee_true.copy(); T_bad[:3, 3] += np.array([0.10, 0.0, 0.03])   # 10 cm off, like May's translation
T_bad[:3, :3] = rpy_deg_to_R(3.0, -2.0, 4.0) @ T_bad[:3, :3]
res, arm, cam, caps, events = run_case(T_bad)
print('  ', res.summary())
check(res.error is None, 'runs (align tolerates the model error by re-measuring)')
check(all(c[1] is not None for c in caps), 'a capture only ever happens with the tag detected')
lows = []
for T_c, _, _ in caps:
    for p in SweepCfg().tool_points_mm:
        P = T_c[:3, :3] @ (np.array(p) / 1000.0) + T_c[:3, 3]
        lows.append(float(np.dot(n, P - tag_o)))
check(min(lows) >= 0.08, f'tool never below {min(lows) * 1000:.0f} mm even with the 10 cm aiming error')
check(res.n_captured >= 8, f'still {res.n_captured} usable samples')

check(res.aim_source == 'file' and not res.diverged,
      '10 cm / 4 deg aiming error does not trip the divergence guard (still aimed by the file)')

# ================================================================ 4. remounted camera
print('== divergence guard: the 2026-09-18 numbers trip it, near-convergence noise does not')
from path_tag_locator.align import AlignMetrics   # noqa: E402
r0 = SweepRunner(SweepCfg(), get_tcp=None, move=None, detect=None, capture=None)
e1 = _align_error_mm(AlignMetrics(xy_offset_m=0.289, z_distance_m=1.203, tilt_deg=2.98))
e2 = _align_error_mm(AlignMetrics(xy_offset_m=0.3735, z_distance_m=1.098, tilt_deg=7.73))
check(r0._diverged(e1, e2), f'2026-09-18 align 1 -> 2 ({e1:.0f} -> {e2:.0f} mm-eq) is a divergence')
e3 = _align_error_mm(AlignMetrics(xy_offset_m=0.003, z_distance_m=0.45, tilt_deg=0.7))
e4 = _align_error_mm(AlignMetrics(xy_offset_m=0.006, z_distance_m=0.45, tilt_deg=1.4))
check(not r0._diverged(e3, e4), f'noise near convergence ({e3:.0f} -> {e4:.0f}, ratio 2 but +10 mm) is not')
e5 = _align_error_mm(AlignMetrics(xy_offset_m=0.066, z_distance_m=0.8, tilt_deg=5.1))
e6 = _align_error_mm(AlignMetrics(xy_offset_m=0.131, z_distance_m=0.8, tilt_deg=8.5))
check(r0._diverged(e5, e6), f'2026-09-02 (180 deg-spun file) align 1 -> 2 ({e5:.0f} -> {e6:.0f}) is a divergence')

# A hand-eye of a DIFFERENT mount: the camera turned a quarter turn and moved 15 cm
T_moved = T_hc2ee_true.copy()
T_moved[:3, :3] = rpy_deg_to_R(0.0, 0.0, 90.0) @ T_moved[:3, :3]
T_moved[:3, 3] += np.array([-0.12, 0.09, 0.04])
T_tilted = T_hc2ee_true.copy()
T_tilted[:3, :3] = rpy_deg_to_R(45.0, 0.0, 0.0) @ T_tilted[:3, :3]

print('== bootstrap: never -> retreat + manual procedure named')
for label, T_aim, depth, kind in (('quarter-turn, 0.55 m', T_moved, 0.45, 'wrong way'),
                                  ('quarter-turn, 1.20 m (stall)', T_moved, 1.10, 'no progress'),
                                  ('45 deg tilt, 1.20 m', T_tilted, 1.10, 'wrong way')):
    res, arm, cam, caps, events = run_case(T_aim, cfg=SweepCfg(bootstrap='never'), start_depth=depth)
    check(res.error is not None and MANUAL_PROCEDURE in res.error and res.diverged and kind in res.error,
          f'{label}: fails as "{kind}" with the manual procedure named ({res.align_iterations} iterations)')
    T_ee0 = T_ab2tag @ invert_T(view_T_cam2tag(0.3, depth, 0, 0, 0)) @ T_hc2ee_true
    T_ee0[:3, 3] += np.array((0.06, -0.04, 0.10))
    check(np.allclose(arm.T, T_ee0, atol=1e-9) and cam.detect() is not None,
          '  arm is back at the best (= start) pose with the tag in view')
    check(len(caps) == 0 and not any(e['phase'] == 'bootstrap' for e in events),
          '  nothing captured, no bootstrap attempted')

print('== bootstrap: auto with the moved-camera file, real cv2.calibrateHandEye under noise')
for depth, off in ((0.45, (0.06, -0.04, 0.10)), (1.10, (0.06, -0.04, 0.10))):
    flags = []; n_caps = []; iters = []; nb = []; caps_all = []; n_caps_all = []
    for seed in range(6):
        res, arm, cam, caps, events = run_case(T_moved, cfg=SweepCfg(), seed=seed, with_solve=True,
                                               start_depth=depth, start_offset=off)
        caps_all.append(caps); n_caps_all.append(res.n_captured)
        if seed == 0:
            print('  ', f'start depth {depth + off[2]:.2f} m:', res.summary())
        if res.error:
            print('   ERROR', res.error)
        flags.append((res.diverged, res.bootstrapped, res.error is None))
        n_caps.append(res.n_captured); iters.append(res.align_iterations); nb.append(res.n_bootstrap)
    check(all(all(f) for f in flags),
          f'depth {depth + off[2]:.2f} m: every seed diverged on the file, bootstrapped, completed')
    check(min(nb) >= 5, f'  bootstrap captured {sorted(set(nb))} of 7 views')
    check(min(n_caps) >= 8, f'  every seed ends with >= 8 samples ({sorted(n_caps)})')
    check(all(len(c) == r for c, r in zip(caps_all, n_caps_all)),
          '  the 7 bootstrap samples were dropped from the set (sweep samples only remain)')
    check(max(iters) <= SweepCfg().align_max_iterations,
          f'  square-up with the provisional hand-eye ran {sorted(set(iters))} iterations')

# quality of the provisional estimate itself, and the sweep's safety under it
errs = []
for seed in range(6):
    res, arm, cam, caps, events = run_case(T_moved, cfg=SweepCfg(bootstrap_keep_samples=True), seed=seed, with_solve=True)
    T_prov = make_solve(caps[:res.n_bootstrap], np.random.RandomState(seed))(0)
    errs.append(he_error(T_prov))
mm = max(e[0] for e in errs); deg = max(e[1] for e in errs)
check(mm < 60 and deg < 4.0,
      f'provisional hand-eye from the 7 bootstrap views: worst of 6 seeds {mm:.0f} mm / {deg:.2f} deg off (aim-grade)')
mm_f, deg_f = he_error(T_moved)
check(mm_f > 100 and deg_f > 60, f'  (the file it replaced was {mm_f:.0f} mm / {deg_f:.1f} deg off)')
res, arm, cam, caps, events = run_case(T_moved, cfg=SweepCfg(bootstrap_keep_samples=True), seed=1, with_solve=True)
check(all(c[1] is not None for c in caps), 'every capture (bootstrap and sweep) had the tag detected')
check(len(caps) == res.n_captured + res.n_bootstrap, 'bootstrap_keep_samples: true keeps the 7 in the set')
lows = []
for T_c, _, _ in caps:
    for p in SweepCfg().tool_points_mm:
        P = T_c[:3, :3] @ (np.array(p) / 1000.0) + T_c[:3, 3]
        lows.append(float(np.dot(n, P - tag_o)))
check(min(lows) >= 0.10, f'tool never below {min(lows) * 1000:.0f} mm with the provisional aim (+50 mm margin planned)')
check(np.allclose(arm.T[:3, 3], pose_fr5_to_matrix_m(res.start_tcp)[:3, 3], atol=1e-6),
      'arm returned to the (squared-up) start pose')
# after the align steps and the retreat: 6 rotations + the return, all pure rotations
runs = []
for i, (d, a) in enumerate(arm.moves):
    if d < 1e-6 and 0 < a <= 2 * SweepCfg().bootstrap_angle_deg + 1e-6:
        runs.append(i)
consec = 1; best = 1
for a, b in zip(runs, runs[1:]):
    consec = consec + 1 if b == a + 1 else 1
    best = max(best, consec)
check(best == len(BOOTSTRAP_AXES) + 1,
      f'bootstrap moves are {len(BOOTSTRAP_AXES) + 1} consecutive pure flange rotations '
      f'(0 mm, <= {2 * SweepCfg().bootstrap_angle_deg:.0f} deg between views)')
ph = [e['phase'] for e in events]
check(ph.index('diverged') < ph.index('bootstrap') < ph.index('start') < ph.index('finished'),
      'progress: align … diverged … bootstrap … start … finished')
st = [e for e in events if e['phase'] == 'start'][0]
check(st.get('aim_source') == 'bootstrap', "the start event says aim_source: bootstrap")

print('== bootstrap: always, and no file at all')
res, arm, cam, caps, events = run_case(T_hc2ee_true, cfg=SweepCfg(bootstrap='always'), seed=2, with_solve=True)
check(res.error is None and res.bootstrapped and not res.diverged and res.aim_source == 'bootstrap',
      f'always: bootstraps without trying the file ({res.summary()})')
res, arm, cam, caps, events = run_case(None, cfg=SweepCfg(), seed=3, with_solve=True)
check(res.error is None and res.bootstrapped and res.n_captured >= 8,
      f'no file: bootstraps and completes ({res.n_captured} samples)')
res, arm, cam, caps, events = run_case(None, cfg=SweepCfg(bootstrap='never'))
check(res.error and MANUAL_PROCEDURE in res.error and not arm.moves, 'no file + never: refuses without moving')

print('== bootstrap refused when the camera is too close to the tag')
res, arm, cam, caps, events = run_case(T_moved, cfg=SweepCfg(), seed=4, with_solve=True,
                                       start_depth=0.30, start_offset=(0.02, -0.02, 0.0))
check(res.error and 'raise it first' in res.error and res.diverged,
      f'0.30 m start: {res.error}')
check(cam.detect() is not None, 'tag still in view (retreated, no bootstrap moves)')

print('== bootstrap without the solve callable')
res, arm, cam, caps, events = run_case(T_moved, cfg=SweepCfg())
check(res.error and 'solve' in res.error, 'auto without solve: fails cleanly, names the missing callable')

print('== tag not visible at the start')
arm = FakeArm(T_from(np.eye(3), [0.3, 0.3, 0.3]))
cam = FakeCam(arm, T_hc2ee_true)
r = SweepRunner(SweepCfg(), get_tcp=arm.get_tcp, move=arm.move, detect=cam.detect,
                capture=lambda: (True, ''), log_info=lambda s: None, log_warn=lambda s: None)
res = r.run(T_hc2ee_true)
check(res.error and 'not visible' in res.error and not arm.moves, 'refuses without moving when the tag is not in view')

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
