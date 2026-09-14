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
    plate.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))

from path_tag_locator.handeye_sweep import (SweepCfg, SweepRunner, generate_views,  # noqa: E402
                                             plan_sweep, view_T_cam2tag)
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
        return T_cam2tag


def run_case(T_hc2ee_aim, start_offset=(0.06, -0.04, 0.10), cancel_after=None, fail_on=(), cfg=None):
    cfg = cfg or SweepCfg()
    # operator parks the camera roughly over the tag: square view + an offset
    T_sq = view_T_cam2tag(0.3, 0.45, 0, 0, 0)
    T_ee0 = T_ab2tag @ invert_T(T_sq) @ T_hc2ee_true
    T_ee0[:3, 3] += np.array(start_offset)
    arm = FakeArm(T_ee0); arm.fail_on = set(fail_on)
    cam = FakeCam(arm, T_hc2ee_true)
    caps = []; events = []
    state = {'cancel': False}

    def capture():
        caps.append((arm.T.copy(), cam.seen[-1] if cam.seen else None)); return True, f'sample {len(caps)}'

    def progress(d):
        events.append(d)
        if cancel_after is not None and d.get('phase') == 'sample' and d.get('index') == cancel_after:
            state['cancel'] = True

    r = SweepRunner(cfg, get_tcp=arm.get_tcp, move=arm.move, detect=cam.detect, capture=capture,
                    cancelled=lambda: state['cancel'], log_info=lambda s: None, log_warn=lambda s: None,
                    progress=progress)
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
for T_c, _ in caps:
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
for T_c, _ in caps:
    for p in SweepCfg().tool_points_mm:
        P = T_c[:3, :3] @ (np.array(p) / 1000.0) + T_c[:3, 3]
        lows.append(float(np.dot(n, P - tag_o)))
check(min(lows) >= 0.08, f'tool never below {min(lows) * 1000:.0f} mm even with the 10 cm aiming error')
check(res.n_captured >= 8, f'still {res.n_captured} usable samples')

print('== tag not visible at the start')
arm = FakeArm(T_from(np.eye(3), [0.3, 0.3, 0.3]))
cam = FakeCam(arm, T_hc2ee_true)
r = SweepRunner(SweepCfg(), get_tcp=arm.get_tcp, move=arm.move, detect=cam.detect,
                capture=lambda: (True, ''), log_info=lambda s: None, log_warn=lambda s: None)
res = r.run(T_hc2ee_true)
check(res.error and 'not visible' in res.error and not arm.moves, 'refuses without moving when the tag is not in view')

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
