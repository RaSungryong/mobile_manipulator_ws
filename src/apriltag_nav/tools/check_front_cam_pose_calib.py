#!/usr/bin/env python3
"""Offline check of tools/calib_front_cam_pose.py's solver on a synthetic
plant — run before trusting a real session, and after touching the tool.

    python3 src/apriltag_nav/tools/check_front_cam_pose_calib.py

The plant: a base whose ROTATION CENTRE is the body origin, a lens (tx, ty)
from it (body y LEFT positive), a camera tilted (roll, pitch) at height h
and yawed `yaw` about its optical axis vs the travel axis, a laid tag pair.
Snapshots (moves, pivots) and drive tracks are rendered through the real
GroundPlane forward model with pixel noise, written in the tool's file
format, and solved with the tool's own functions. The base under-executes
commands and yaws on its own while driving, as the real one does.
"""
import math
import os
import shutil
import sys
import tempfile

import numpy as np

_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_TOOLS, '..', 'src'))
sys.path.insert(0, _TOOLS)
from apriltag_nav.ground_plane import GroundPlane  # noqa: E402
import calib_front_cam_pose as C  # noqa: E402

_n = [0]
_bad = [0]


def check(name, ok, detail=""):
    _n[0] += 1
    if not ok:
        _bad[0] += 1
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")


FX, FY, CX, CY = 910.0, 910.0, 642.7, 361.4
D = [0.0830745, -0.1101505, 5.28e-05, -0.000334, 0.0454777]   # the real front_cam D
W, H = 1280, 720
TAGS = (15, 16)
SPACING, SIZE = 0.150, 0.060


def rz(a):
    return C.R2(a)


class Plant:
    def __init__(self, roll, pitch, h, yaw, tx, ty, noise_px=0.3, seed=1):
        self.gp = GroundPlane(FX, FY, CX, CY, D, roll, pitch, h, yaw=yaw)
        self.tx, self.ty = tx, ty
        self.rng = np.random.RandomState(seed)
        self.noise = noise_px
        # base pose in world: rotation centre position + heading
        self.p = np.array([0.0, 0.0])
        self.th = 0.0
        # pair laid along world x at (0.552 + something) ahead-ish so it sits
        # under the lens: put the pair centre where the nadir starts
        self.pair = [np.array([0.0, 0.0]), np.array([SPACING, 0.0])]
        nadir = self.p + rz(self.th) @ np.array([self.tx, self.ty])
        self.pair = [nadir - [SPACING / 2, 0.0], nadir + [SPACING / 2, 0.0]]

    def nadir(self):
        return self.p + rz(self.th) @ np.array([self.tx, self.ty])

    def corners_world(self, k):
        h = SIZE / 2
        base = np.array([[-h, -h], [-h, h], [h, h], [h, -h]])   # pair frame, y RIGHT(down)
        # world frame is right-handed z-UP: y left. The pair frame's "y right"
        # is world -y. The pair's x is world +x.
        return self.pair[k] + base * [1, -1]

    def frame(self):
        """{id: {'c': raw corners}} for the current pose, or None if out of frame."""
        n = self.nadir()
        out = {}
        for k, tid in enumerate(TAGS):
            cw = self.corners_world(k)
            rel = (rz(-self.th) @ (cw - n).T).T           # body frame, y LEFT
            g = rel * [1, -1]                               # -> x fwd, y RIGHT (ground frame of the fit)
            px = self.gp.project(g) + self.rng.normal(0, self.noise, (4, 2))
            if (px < 5).any() or (px[:, 0] > W - 5).any() or (px[:, 1] > H - 5).any():
                return None
            out[tid] = dict(cx=px[:, 0].mean(), cy=px[:, 1].mean(), c=px)
        return out

    def move(self, dist):
        # under-executed, with a yaw wander proportional to the distance
        d = dist * self.rng.uniform(0.5, 0.7)
        dth = math.radians(self.rng.uniform(-0.6, 0.6)) * abs(d) / 0.1
        self.p = self.p + rz(self.th) @ np.array([d, 0.0])
        self.th += dth

    def pivot(self, deg):
        self.th += math.radians(deg * self.rng.uniform(0.7, 0.85))


def averaged_frame(P, n=15):
    fs = [P.frame() for _ in range(n)]
    if any(f is None for f in fs):
        return None
    out = {}
    for t in TAGS:
        c = np.mean([f[t]['c'] for f in fs], axis=0)
        out[t] = dict(cx=c[:, 0].mean(), cy=c[:, 1].mean(), c=c)
    return out


def write_snapshot(fn, det):
    lines = ["header:", "detections: "]
    for tid, d in det.items():
        lines += ["  - ", "    id: %d" % tid, "    center_x: %r" % d['cx'], "    center_y: %r" % d['cy'],
                  "    pose_x: 0.0", "    pose_y: 0.0", "    pose_z: 0.45", "    roll: 0", "    pitch: 0", "    yaw: 0",
                  "    corners: [%s]" % ", ".join("%r" % float(v) for v in d['c'].ravel()),
                  "    tilt_from_normal: 0"]
    open(fn, 'a').write("\n".join(lines) + "\n---\n")


def run_case(roll_deg, pitch_deg, h, yaw_deg, tx, ty, seed):
    P = Plant(math.radians(roll_deg), math.radians(pitch_deg), h, math.radians(yaw_deg), tx, ty, seed=seed)
    d = tempfile.mkdtemp(prefix='fcpose_')
    open(os.path.join(d, 'camera_info.txt'), 'w').write(
        "D: [%s]\nK: [%s]\n" % (", ".join(map(str, D)), ", ".join(map(str, [FX, 0, CX, 0, FY, CY, 0, 0, 1]))))
    k = 0
    for dist in [0.0, +0.02, +0.02, +0.02, -0.02, -0.02, -0.04, -0.02, +0.04]:
        if dist:
            P.move(dist)
        f = averaged_frame(P)
        assert f is not None, "pair left the frame during scans"
        write_snapshot(os.path.join(d, 'scan_%02d.txt' % k), f); k += 1
    for ang in [0.0, +4, -4, +4, -4, +4, -4]:
        if ang:
            P.pivot(ang)
        f = averaged_frame(P)
        assert f is not None, "pair left the frame during pivots"
        write_snapshot(os.path.join(d, 'piv_%02d.txt' % k), f); k += 1
    for j, dist in enumerate([+0.12, -0.12] * 3):
        fn = os.path.join(d, 'drive_%02d.txt' % j)
        for _ in range(100):
            P.move(dist / 100 / 0.6)        # ~1 mm per frame (0.03 m/s at 30 Hz) after under-execution
            f = P.frame()
            if f is not None:
                write_snapshot(fn, f)
    r = C.solve_dir(d, TAGS, SPACING, SIZE, 0.30)
    shutil.rmtree(d)
    return r


print("== nominal: the 2026-09-08 numbers, lens 4 mm right of the centre, yaw -0.4 deg ==")
r = run_case(1.228, -0.504, 0.302, -0.40, 0.552, -0.004, seed=1)
F = r['fit']
check("roll/pitch recovered within 0.02 deg",
      abs(math.degrees(F['roll']) - 1.228) < 0.02 and abs(math.degrees(F['pitch']) + 0.504) < 0.02,
      "%.3f / %.3f" % (math.degrees(F['roll']), math.degrees(F['pitch'])))
check("lens height within 0.5 mm", abs(F['h'] - 0.302) < 5e-4, "%.1f mm" % (F['h'] * 1e3))
check("lever within 1 mm", abs(r['lever'] - math.hypot(0.552, 0.004)) < 1e-3, "%.4f m" % r['lever'])
check("tx within 1 mm", abs(r['tx'] - 0.552) < 1e-3, "%.4f m" % r['tx'])
check("ty within 1 mm AND the right sign (lens right of centre -> ty negative)",
      abs(r['ty'] + 0.004) < 1e-3, "%.4f m" % r['ty'])
check("yaw within 0.08 deg (the 0.3 px / 0.12 m limit)", abs(math.degrees(r['yaw']) + 0.40) < 0.08,
      "%.3f deg (rms %.2f mm, %d pairs)" % (math.degrees(r['yaw']), r['yaw_rms_m'] * 1e3, r['yaw_pairs']))
check("per-track yaws within 0.25 deg of truth (read their spread as the uncertainty)",
      max(abs(math.degrees(v) + 0.40) for v in r['yaw_per_track']) < 0.25,
      ", ".join("%.3f" % math.degrees(v) for v in r['yaw_per_track']))

print("\n== opposite signs: yaw +0.7, lens 8 mm LEFT of the centre, roll -0.8, pitch +0.6 ==")
r = run_case(-0.8, 0.6, 0.298, +0.70, 0.548, +0.008, seed=2)
F = r['fit']
check("roll/pitch/height recovered",
      abs(math.degrees(F['roll']) + 0.8) < 0.02 and abs(math.degrees(F['pitch']) - 0.6) < 0.02 and abs(F['h'] - 0.298) < 5e-4,
      "%.3f / %.3f / %.1f mm" % (math.degrees(F['roll']), math.degrees(F['pitch']), F['h'] * 1e3))
check("tx within 1 mm", abs(r['tx'] - 0.548) < 1e-3, "%.4f" % r['tx'])
check("ty +8 mm recovered with sign (±1.5 mm)", abs(r['ty'] - 0.008) < 1.5e-3, "%.4f" % r['ty'])
check("yaw +0.70 recovered within 0.08 deg", abs(math.degrees(r['yaw']) - 0.70) < 0.08, "%.3f" % math.degrees(r['yaw']))

print("\n== level camera, zero offsets: nothing invented ==")
r = run_case(0.0, 0.0, 0.300, 0.0, 0.550, 0.0, seed=3)
check("tx 0.550 / ty 0 / yaw 0 within 1 mm / 0.08 deg",
      abs(r['tx'] - 0.55) < 1e-3 and abs(r['ty']) < 1.5e-3 and abs(math.degrees(r['yaw'])) < 0.08,
      "%.4f / %.4f / %.3f" % (r['tx'], r['ty'], math.degrees(r['yaw'])))

print("\n== apply: robot.yaml edit round-trips through the generator ==")
import yaml
from apriltag_nav.paths import CONFIG_PATH
tmpd = tempfile.mkdtemp(prefix='fcpose_apply_')
ry = os.path.join(tmpd, 'robot.yaml')
shutil.copy(CONFIG_PATH, ry)
ex = os.path.join(tmpd, 'extrinsics.yaml')
shutil.copy(os.path.join(_TOOLS, '..', '..', 'path_tag_locator', 'config', 'extrinsics.yaml'), ex)
r = run_case(1.0, -0.3, 0.301, -0.25, 0.5535, -0.006, seed=4)
import subprocess
# apply to the COPY, then regenerate the COPY of extrinsics from it
old_check_call = subprocess.check_call
subprocess.check_call = lambda cmd, *a, **k: old_check_call(cmd + ['--robot-yaml', ry, '--extrinsics', ex], *a, **k)
try:
    C.apply_to_robot_yaml(r, math.degrees(r['yaw']), path=ry)
finally:
    subprocess.check_call = old_check_call
cfg = yaml.safe_load(open(ry))
gp = cfg['robot_camera']['ground_plane']['front_cam']
check("camera_offset / camera_lateral written", cfg['robot']['camera_offset'] == round(r['tx'], 3)
      and cfg['robot']['camera_lateral'] == round(r['ty'], 3))
check("ground_plane numbers written", abs(gp['roll_deg'] - math.degrees(r['fit']['roll'])) < 1e-3
      and abs(gp['yaw_deg'] - math.degrees(r['yaw'])) < 1e-3 and gp['height_m'] == round(r['fit']['h'], 3))
check("nothing else in robot.yaml changed",
      {k: v for k, v in yaml.safe_load(open(CONFIG_PATH)).items() if k not in ('robot', 'robot_camera')}
      == {k: v for k, v in cfg.items() if k not in ('robot', 'robot_camera')})
sys.path.insert(0, os.path.join(_TOOLS, '..', '..', 'path_tag_locator', 'src'))
from path_tag_locator.constants import load_extrinsics_full
e = load_extrinsics_full(ex, robot_yaml_path=ry)
check("regenerated extrinsics load: level t == (camera_offset, camera_lateral, height_m)",
      np.allclose(e.T_mb2fc_level[:3, 3], [cfg['robot']['camera_offset'], cfg['robot']['camera_lateral'], gp['height_m']]),
      str(np.round(e.T_mb2fc_level[:3, 3], 4)))
shutil.rmtree(tmpd)

print("\n%d checks, %d failed" % (_n[0], _bad[0]))
sys.exit(1 if _bad[0] else 0)
