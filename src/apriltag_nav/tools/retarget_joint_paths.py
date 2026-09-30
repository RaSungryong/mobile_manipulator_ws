#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Retarget the planner's joint paths (rrt_final_path_*.csv) to the robot as it is.

    python3 tools/retarget_joint_paths.py [task_dir]            # dry run, writes nothing
    python3 tools/retarget_joint_paths.py [task_dir] --apply    # back up, then rewrite q1..q6

Why
---
A `rrt_final_path_<key>.csv` row is an absolute joint configuration fed straight
to MoveJ (`arm_controller._exec_joint`); no transform reads it, so it puts the
tool where the planner meant only if the arm base is where the planner assumed.
The planner's model, settled by FK against its own `assigned_workpoints_*`
twins (2026-09-29: 0.010 mm over all 2828 work points, one combination only):

    base stop pose   tag of map_idle.yaml - 0.55 m, heading exactly +-90 deg
                     (docs/robot_base_stop_poses_plate1_idle.csv)
    arm mount        the DESIGN T_ab2mb   (tf_chain.yaml `T_ab2mb.design`)
    vision tip       the DESIGN T_ee2tip  (tf_chain.yaml `T_ee2tip.design`)

The robot stops on the CALIBRATED tags (map.yaml ->
docs/robot_base_stop_poses_plate1_0928.csv) and carries the calibrated mount
and the measured tip (tf_chain.yaml applied values) — the model pose mode
(`_exec_pose`) already uses at run time. Replayed as planned, the tip lands
9 mm (zone B) / 17 mm (zone C) beside the planned point and 10 mm above it.

What it does, per row (W = the base stop pose of the row's group tag, M = mb ->
arm base, TIP = flange -> vision tip; _p the planner's, _r the robot's):

    P      = W_p @ M_p @ FK(q) @ TIP_p            the planned tip pose, world
    target = inv(W_r @ M_r) @ P @ inv(TIP_r)      the flange that puts the real
                                                  tip there, real arm frame
    q_new  = IK(target), seeded with q            same branch as the plan

    task        re-solved exactly
    transition  between two work points: re-solved exactly (the tool follows
                the planned world path); next to a `home` row: the work-point
                end's correction faded to 0 at home along the joint-space
                path length, so the run still starts / ends ON the home pose
    home        unchanged (the arm's home joints are not a world target)

Only the q1..q6 cells change; BOM, CRLF and every other cell stay byte-identical.
`assigned_workpoints_*` is NOT touched: its x y z rx ry rz are WORLD
coordinates, and `_exec_pose` applies the live /robot_pose to them already.

Safe to re-run: every file is classified first from its work points —
ORIGINAL (FK under the planner's model is on the paired pose rows), RETARGETED
(FK under the robot's model is) or UNKNOWN — and only ORIGINAL files are
rewritten. After map.yaml or tf_chain.yaml changes again, restore the planner
originals from the backup directory, regenerate the actual stop poses
(tools/map_stop_poses.py) and run this again.

What it does NOT do: apply the joint zero offsets (joint mode never has; pose
mode applies their xy part), or see where the base really stopped on a given
arrival (+-2 mm / +-0.2 deg) — the reference is the nominal stop pose.
"""
import argparse
import csv
import datetime
import hashlib
import math
import os
import shutil
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))
sys.path.insert(0, HERE)

from apriltag_nav import paths, tf_chain                      # noqa: E402
from apriltag_nav.arm_fk import ArmChain                      # noqa: E402
from map_stop_poses import ZONE_HEADING, laying_delta_deg, wrap180   # noqa: E402

JOINT_PREFIX = 'rrt_final_path_'
POSE_PREFIX = 'assigned_workpoints_'
Q_COLS = ['q%d' % i for i in range(1, 7)]
KINDS = ('home', 'task', 'transition')
BOM = b'\xef\xbb\xbf'

DOCS = os.path.join(paths.WS_DIR, 'docs')
PLANNED_DEFAULT = os.path.join(DOCS, 'robot_base_stop_poses_plate1_idle.csv')
ACTUAL_DEFAULT = os.path.join(DOCS, 'robot_base_stop_poses_plate1_0928.csv')
DIFF_DEFAULT = os.path.join(DOCS, 'robot_base_stop_poses_plate1_diff_idle_vs_0928.csv')

# IK: accepted when the flange is on the target to these.
IK_TOL_M = 1e-8
IK_TOL_RAD = 1e-8
IK_MAX_ITERS = 60
IK_MAX_STEP_RAD = 0.2


# ---------------------------------------------------------------- text-level CSV
def load_text_csv(path):
    """(header, rows) of cells as written — the planner's dialect: UTF-8 BOM,
    CRLF, no quoting. Refuses anything else rather than normalising it."""
    raw = open(path, 'rb').read()
    if raw[:3] != BOM:
        raise ValueError('%s: no UTF-8 BOM — not the planner dialect' % path)
    text = raw[3:].decode('utf-8')
    if not text.endswith('\r\n') or '"' in text:
        raise ValueError('%s: expected CRLF line ends and no quoting' % path)
    rows = [line.split(',') for line in text[:-2].split('\r\n')]
    hdr = rows[0]
    if any(len(r) != len(hdr) for r in rows):
        raise ValueError('%s: a row does not have %d cells' % (path, len(hdr)))
    return hdr, rows[1:]


def save_text_csv(path, hdr, rows):
    text = '\r\n'.join(','.join(r) for r in [hdr] + rows) + '\r\n'
    with open(path, 'wb') as f:
        f.write(BOM + text.encode('utf-8'))


def fmt_q(v):
    """Radians, 1e-9 resolution (1.4e-6 mm at full reach), no exponent."""
    s = ('%.9f' % v).rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def sha256(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


# ---------------------------------------------------------------- geometry
def T_world_base(x, y, yaw_deg):
    """World pose of the mobile base centre: Rz(heading) at (x, y, floor)."""
    a = math.radians(yaw_deg)
    M = np.eye(4)
    M[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    M[0, 3], M[1, 3] = x, y
    return M


def T_translate(t_m):
    M = np.eye(4)
    M[:3, 3] = t_m
    return M


def rotvec(Rm):
    """Rotation vector of a rotation matrix; exact for the small angles an IK
    residual has (atan2, not acos, which loses half its digits near 0)."""
    v = np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])
    s = np.linalg.norm(v)
    if s < 1e-15:
        return np.zeros(3)
    return v / s * math.atan2(0.5 * s, 0.5 * (np.trace(Rm) - 1.0))


def pose_error(T, T_target):
    """6-vector taking T onto T_target: translation, rotation vector (base frame)."""
    return np.concatenate([T_target[:3, 3] - T[:3, 3],
                           rotvec(T_target[:3, :3] @ T[:3, :3].T)])


def euler_matrix(spec, angles):
    from scipy.spatial.transform import Rotation as R
    rot = R.from_euler(spec, angles)
    return rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()


class Kinematics:
    """FK / IK of the flange in the arm base frame, joints in RADIANS (the CSV
    unit). The chain is the planner URDF's, i.e. the controller's own model."""

    def __init__(self, chain):
        self.chain = chain

    def fk(self, q_rad):
        return self.chain.fk_flange(np.degrees(q_rad))

    def ik(self, T_target, seed_rad):
        """Damped Gauss-Newton from the seed. Returns (q, converged, iters).
        Stays on the seed's branch: the step is bounded and the target is
        millimetres / a fraction of a degree from FK(seed)."""
        q = np.array(seed_rad, dtype=float)
        h = 1e-6
        for it in range(IK_MAX_ITERS):
            T = self.fk(q)
            e = pose_error(T, T_target)
            if np.linalg.norm(e[:3]) < IK_TOL_M and np.linalg.norm(e[3:]) < IK_TOL_RAD:
                return q, True, it
            J = np.zeros((6, 6))
            for k in range(6):
                dq = np.zeros(6)
                dq[k] = h
                J[:, k] = pose_error(T, self.fk(q + dq)) / h
            step = np.linalg.solve(J.T @ J + 1e-6 * np.eye(6), J.T @ e)
            n = np.linalg.norm(step)
            if n > IK_MAX_STEP_RAD:
                step *= IK_MAX_STEP_RAD / n
            q = q + step
        e = pose_error(self.fk(q), T_target)
        ok = np.linalg.norm(e[:3]) < IK_TOL_M and np.linalg.norm(e[3:]) < IK_TOL_RAD
        return q, bool(ok), IK_MAX_ITERS


def urdf_joint_limits(urdf_path):
    import xml.etree.ElementTree as ET
    joints = {j.get('name'): j for j in ET.parse(urdf_path).getroot().findall('joint')}
    lim = []
    for i in range(1, 7):
        el = joints['j%d' % i].find('limit')
        lim.append((float(el.get('lower')), float(el.get('upper'))))
    return np.array(lim)


# ---------------------------------------------------------------- models
class Model:
    """One consistent set: base stop poses + mount + tip."""

    def __init__(self, name, stop_poses, T_ab2mb, tip_m):
        self.name = name
        self.stop = stop_poses                       # {tag: (x, y, yaw_deg)}
        self.M = tf_chain.invert_T(T_ab2mb)          # mb -> arm base
        self.tip = np.asarray(tip_m, dtype=float)
        self.TIP = T_translate(self.tip)

    def T_world_arm(self, tag, lift_m=0.0):
        """World pose of the arm base with the lift `lift_m` above its origin
        (the CSV's `lift_mm`): the lift raises the arm base vertically in the
        mb frame, exactly as `transform_world_to_arm` adds it to arm_base_z."""
        return T_world_base(*self.stop[tag]) @ T_translate((0.0, 0.0, lift_m)) @ self.M

    def tip_world(self, tag, T_flange, lift_m=0.0):
        return self.T_world_arm(tag, lift_m) @ T_flange @ self.TIP


def read_stop_poses(path):
    out = {}
    with open(path, encoding='utf-8-sig', newline='') as f:
        for r in csv.DictReader(f):
            if not (r.get('tag_id') or '').strip():
                continue
            out[int(r['tag_id'])] = (float(r['base_x_m']), float(r['base_y_m']),
                                     float(r['base_yaw_deg']), r.get('zone', ''))
    if not out:
        raise ValueError('%s: no stop poses' % path)
    return out


def stop_poses_from_map(map_path, robot_cfg):
    """The same arithmetic as tools/map_stop_poses.py, for the staleness check."""
    rcfg = robot_cfg['robot']
    off = float(rcfg.get('camera_offset', 0.55))
    lat = float(rcfg.get('camera_lateral', 0.0))
    use_yaw = bool(rcfg.get('robot_pose_use_tag_yaw', True))
    with open(map_path) as f:
        tags = yaml.safe_load(f)['tags']
    out = {}
    for tid, tag in tags.items():
        zone = str(tag.get('zone', 'A'))
        if zone not in ZONE_HEADING:
            continue
        heading = ZONE_HEADING[zone] + laying_delta_deg(tag, use_yaw)
        h = math.radians(heading)
        out[int(tid)] = (float(tag['x']) - off * math.cos(h) + lat * math.sin(h),
                         float(tag['y']) - off * math.sin(h) - lat * math.cos(h),
                         wrap180(heading))
    return out


def write_diff_csv(path, planned, actual, used_tags):
    cols = ['tag_id', 'zone', 'idle_base_x_m', 'idle_base_y_m', 'idle_base_yaw_deg',
            'actual_base_x_m', 'actual_base_y_m', 'actual_base_yaw_deg',
            'd_world_x_mm', 'd_world_y_mm', 'd_yaw_deg',
            'd_body_fwd_mm', 'd_body_left_mm', 'used_by_task_csv']
    rows = []
    for tag in planned:
        if tag not in actual:
            continue
        px, py, pyaw, zone = planned[tag]
        ax, ay, ayaw, _ = actual[tag]
        dx, dy = ax - px, ay - py
        h = math.radians(pyaw)
        rows.append([tag, zone, '%.4f' % px, '%.4f' % py, '%.2f' % pyaw,
                     '%.4f' % ax, '%.4f' % ay, '%.2f' % ayaw,
                     '%+.1f' % (dx * 1e3), '%+.1f' % (dy * 1e3), '%+.2f' % wrap180(ayaw - pyaw),
                     '%+.1f' % ((dx * math.cos(h) + dy * math.sin(h)) * 1e3),
                     '%+.1f' % ((-dx * math.sin(h) + dy * math.cos(h)) * 1e3),
                     'yes' if tag in used_tags else ''])
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(cols)
        w.writerows(rows)
    return len(rows)


# ---------------------------------------------------------------- one file
class JointFile:
    def __init__(self, path, pose_path):
        self.path = path
        self.pose_path = pose_path
        self.name = os.path.basename(path)
        self.hdr, self.rows = load_text_csv(path)
        need = Q_COLS + ['group_id', 'source_point_id', 'waypoint_kind']
        missing = [c for c in need if c not in self.hdr]
        if missing:
            raise ValueError('%s: missing column(s) %s' % (self.name, ', '.join(missing)))
        self.iq = [self.hdr.index(c) for c in Q_COLS]
        ig, isrc, ik = (self.hdr.index(c) for c in ('group_id', 'source_point_id', 'waypoint_kind'))
        self.kind = [r[ik] for r in self.rows]
        bad = sorted(set(self.kind) - set(KINDS))
        if bad:
            raise ValueError('%s: unknown waypoint_kind %s' % (self.name, bad))
        self.tag = [int(float(r[ig])) for r in self.rows]
        self.src = [int(float(r[isrc])) if r[isrc].strip() else None for r in self.rows]
        self.q = np.array([[float(r[i]) for i in self.iq] for r in self.rows])
        self.n = len(self.rows)
        # the base height the joint rows were solved at: `lift_mm`, ONE value
        # per file (task_manager refuses a file that disagrees with itself)
        self.lift_m = 0.0
        if 'lift_mm' in self.hdr:
            il = self.hdr.index('lift_mm')
            vals = sorted(set(round(float(r[il]), 6) for r in self.rows if r[il].strip()))
            if len(vals) != 1:
                raise ValueError('%s: lift_mm is not one value: %s' % (self.name, vals))
            self.lift_m = vals[0] / 1e3
        self.task_idx = [i for i in range(self.n) if self.kind[i] == 'task']

        # the paired work points: world tip position + flange orientation
        with open(pose_path, encoding='utf-8-sig', newline='') as f:
            self.pose = {(int(float(r['group_id'])), int(float(r['point_id']))):
                         [float(r[c]) for c in ('x', 'y', 'z', 'rx', 'ry', 'rz')]
                         for r in csv.DictReader(f)}
        unpaired = [i for i in self.task_idx if (self.tag[i], self.src[i]) not in self.pose]
        if unpaired:
            raise ValueError('%s: %d work point(s) have no row in %s'
                             % (self.name, len(unpaired), os.path.basename(pose_path)))

    def pair_error(self, kin, model, euler, q=None):
        """Worst (mm, deg) between FK under `model` and the paired pose rows."""
        q = self.q if q is None else q
        e_mm = e_deg = 0.0
        for i in self.task_idx:
            p = self.pose[(self.tag[i], self.src[i])]
            T = model.tip_world(self.tag[i], kin.fk(q[i]), self.lift_m)
            e_mm = max(e_mm, 1e3 * float(np.linalg.norm(T[:3, 3] - p[:3])))
            Rc = euler_matrix(euler, p[3:])
            e_deg = max(e_deg, math.degrees(float(np.linalg.norm(rotvec(Rc.T @ T[:3, :3])))))
        return e_mm, e_deg

    def runs(self):
        """(a, i, j, b): transition rows i..j-1 between anchors a = i-1, b = j."""
        out, i = [], 0
        while i < self.n:
            if self.kind[i] != 'transition':
                i += 1
                continue
            j = i
            while j < self.n and self.kind[j] == 'transition' and self.tag[j] == self.tag[i]:
                j += 1
            a, b = i - 1, j
            if a < 0 or b >= self.n or self.tag[a] != self.tag[i] or self.tag[b] != self.tag[i]:
                raise ValueError('%s: the transition run at row %d has no anchor row in its '
                                 'group on both sides' % (self.name, i + 2))
            out.append((a, i, j, b))
            i = j
        return out

    def retarget(self, kin, planner, robot):
        """(q_new, info). q_new is what would be written, before rounding."""
        exact = self.q.copy()
        fails, iters = [], 0
        B = planner.TIP @ tf_chain.invert_T(robot.TIP)
        A = {t: tf_chain.invert_T(robot.T_world_arm(t, self.lift_m)) @ planner.T_world_arm(t, self.lift_m)
             for t in sorted(set(self.tag))}
        targets = {}
        for i in range(self.n):
            if self.kind[i] == 'home':
                continue
            target = A[self.tag[i]] @ kin.fk(self.q[i]) @ B
            exact[i], ok, it = kin.ik(target, self.q[i])
            iters = max(iters, it)
            if not ok:
                fails.append(i)
                targets[i] = target
        # A TRANSITION row the robot's model cannot reach (the planner routes
        # through the fully straight arm, ~1.5 m of flange reach, and the
        # target sits a few mm beyond it) is a route, not a target: it keeps
        # the planned row plus the correction interpolated from the nearest
        # solved rows of its group, and the deviation is reported. A WORK
        # point that fails stays a failure.
        fallback = []
        failset = set(fails)
        for i in [k for k in fails if self.kind[k] == 'transition']:
            lo, hi = i - 1, i + 1
            while lo >= 0 and self.tag[lo] == self.tag[i] and lo in failset:
                lo -= 1
            while hi < self.n and self.tag[hi] == self.tag[i] and hi in failset:
                hi += 1
            lo_ok = lo >= 0 and self.tag[lo] == self.tag[i]
            hi_ok = hi < self.n and self.tag[hi] == self.tag[i]
            if not (lo_ok or hi_ok):
                continue
            d_lo = exact[lo] - self.q[lo] if lo_ok else None
            d_hi = exact[hi] - self.q[hi] if hi_ok else None
            if lo_ok and hi_ok:
                seg = self.q[lo:hi + 1]
                s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
                f = s[i - lo] / s[-1] if s[-1] > 0 else 0.5
                corr = (1.0 - f) * d_lo + f * d_hi
            else:
                corr = d_lo if lo_ok else d_hi
            exact[i] = self.q[i] + corr
            dev = 1e3 * float(np.linalg.norm(pose_error(kin.fk(exact[i]), targets[i])[:3]))
            reach = 1e3 * float(np.linalg.norm(kin.fk(self.q[i])[:3, 3]))
            fallback.append((i, reach, dev))
        fails = [k for k in fails if k not in {f[0] for f in fallback}]
        q_new = exact.copy()
        n_blend = 0
        for a, i, j, b in self.runs():
            if self.kind[a] != 'home' and self.kind[b] != 'home':
                continue                                  # re-solved exactly
            seg = self.q[a:b + 1]
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
            s = s / s[-1] if s[-1] > 0 else np.linspace(0.0, 1.0, len(s))
            da, db = exact[a] - self.q[a], exact[b] - self.q[b]
            for k in range(i, j):
                f = s[k - a]
                q_new[k] = self.q[k] + (1.0 - f) * da + f * db
                n_blend += 1
        return q_new, dict(ik_failures=fails, ik_max_iters=iters, n_blended=n_blend,
                           transition_fallback=fallback)


def max_step_deg(q, tag):
    """Largest single-joint change between consecutive rows of one group."""
    d = np.degrees(np.abs(np.diff(q, axis=0))).max(axis=1)
    same = np.array([tag[k] == tag[k + 1] for k in range(len(tag) - 1)])
    return float(d[same].max()) if same.any() else 0.0


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('task_dir', nargs='?', default=paths.TASK_DIR)
    ap.add_argument('--planned', default=PLANNED_DEFAULT,
                    help='stop poses the planner assumed (default: %(default)s)')
    ap.add_argument('--actual', default=ACTUAL_DEFAULT,
                    help='stop poses of the calibrated map (default: %(default)s)')
    ap.add_argument('--map', default=paths.MAP_PATH,
                    help='the map --actual must agree with (default: %(default)s)')
    ap.add_argument('--apply', action='store_true', help='back up and rewrite; default is a dry run')
    ap.add_argument('--backup-dir', default=None,
                    help='default: <ws>/log/apriltag_nav/task_csv_backup/<today>_before_stop_pose_retarget')
    ap.add_argument('--diff-out', default=DIFF_DEFAULT,
                    help='per-tag stop pose difference, written with --apply (default: %(default)s)')
    ap.add_argument('--pair-tol-mm', type=float, default=0.05)
    ap.add_argument('--pair-tol-deg', type=float, default=0.01)
    ap.add_argument('--max-dq-deg', type=float, default=20.0,
                    help='refuse when a joint would move further than this from the planned row')
    ap.add_argument('--limit-margin-deg', type=float, default=5.0,
                    help='refuse when a joint would come closer than this to its URDF limit')
    ap.add_argument('--max-step-growth-deg', type=float, default=0.5,
                    help='refuse when the largest row-to-row joint step grows by more than this')
    ap.add_argument('--max-transition-dev-mm', type=float, default=15.0,
                    help='refuse when an unreachable TRANSITION row (kept on the planned row + the '
                         'neighbours\' correction) deviates further than this from its target')
    args = ap.parse_args()

    n_ok = n_fail = 0

    def check(cond, text):
        nonlocal n_ok, n_fail
        if cond:
            n_ok += 1
        else:
            n_fail += 1
        print('  %s %s' % ('ok  ' if cond else 'FAIL', text))
        return bool(cond)

    # ---- the two models
    robot_cfg = paths.load_config()
    euler = str((robot_cfg.get('arm_calibration') or {}).get('csv_euler', 'ZYX'))
    planned, actual = read_stop_poses(args.planned), read_stop_poses(args.actual)
    d_ab, d_tip = tf_chain.load_entry('T_ab2mb')['design'], tf_chain.load_entry('T_ee2tip')['design']
    planner = Model('planner',
                    {t: v[:3] for t, v in planned.items()},
                    tf_chain.matrix_from_pose(np.array(d_ab['t_mm']) / 1e3, d_ab['rpy_deg_xyz']),
                    np.array(d_tip['t_mm']) / 1e3)
    robot = Model('robot',
                  {t: v[:3] for t, v in actual.items()},
                  tf_chain.load_transform('T_ab2mb'),
                  tf_chain.tip_offset_mm() / 1e3)
    chain = ArmChain()
    kin = Kinematics(chain)
    limits = urdf_joint_limits(chain.path)

    print('planner: stop poses %s' % args.planned)
    print('         %s, tip (%.1f, %.1f, %.1f) mm'
          % (tf_chain.describe(tf_chain.invert_T(planner.M), 'T_ab2mb (design)'), *(planner.tip * 1e3)))
    print('robot:   stop poses %s' % args.actual)
    print('         %s, tip (%.1f, %.1f, %.1f) mm'
          % (tf_chain.describe(tf_chain.invert_T(robot.M), 'T_ab2mb (applied)'), *(robot.tip * 1e3)))
    print('URDF %s, CSV euler %s' % (os.path.relpath(chain.path, paths.WS_DIR), euler))

    # the actual stop poses must be the current map.yaml's, or the result is
    # for a map the robot no longer drives on
    from_map = stop_poses_from_map(args.map, robot_cfg)
    stale = [t for t, v in actual.items() if t in from_map
             and (math.hypot(v[0] - from_map[t][0], v[1] - from_map[t][1]) > 1.5e-4
                  or abs(wrap180(v[2] - from_map[t][2])) > 0.011)]
    print('\n== stop poses')
    check(not stale, '--actual agrees with %s on every tag it lists' % os.path.basename(args.map)
          + ('' if not stale else ' — DIFFERENT for tags %s: regenerate it with '
             'tools/map_stop_poses.py' % stale))

    # ---- the files
    names = sorted(f for f in os.listdir(args.task_dir)
                   if f.startswith(JOINT_PREFIX) and f.lower().endswith('.csv'))
    if not names:
        sys.exit('no %s*.csv in %s' % (JOINT_PREFIX, args.task_dir))

    todo, used_tags, record = [], set(), {}
    for name in names:
        path = os.path.join(args.task_dir, name)
        pose_path = os.path.join(args.task_dir, POSE_PREFIX + name[len(JOINT_PREFIX):])
        print('\n== %s' % name)
        if not os.path.exists(pose_path):
            check(False, 'paired %s exists (needed to tell what the file is)' % os.path.basename(pose_path))
            continue
        jf = JointFile(path, pose_path)
        tags = sorted(set(jf.tag))
        used_tags.update(tags)
        if jf.lift_m:
            print('  lift_mm %.1f: both models place the arm base that much higher' % (jf.lift_m * 1e3))
        # a tag the --actual file does not list (e.g. the 정반 2 lanes of a
        # *_plate2 pair, tools/make_plate2_paths.py) takes the map's stop pose
        filled = [t for t in tags if t not in robot.stop and t in from_map]
        for t in filled:
            robot.stop[t] = from_map[t]
        if filled:
            print('  tags %s are not in --actual: stop poses taken from %s' % (filled, os.path.basename(args.map)))
        if not check(all(t in robot.stop for t in tags),
                     'group tags %s have a stop pose in --actual or the map' % tags):
            continue
        in_planned = all(t in planner.stop for t in tags)
        if not in_planned:
            print('  tags %s are not in --planned: the file cannot be a planner original for 정반 1'
                  % [t for t in tags if t not in planner.stop])

        ep = jf.pair_error(kin, planner, euler) if in_planned else (float('inf'), float('inf'))
        er = jf.pair_error(kin, robot, euler)
        is_orig = ep[0] <= args.pair_tol_mm and ep[1] <= args.pair_tol_deg
        is_done = er[0] <= args.pair_tol_mm and er[1] <= args.pair_tol_deg
        print('  FK of the %d work points vs the paired pose rows: planner model %.3f mm / %.4f deg, '
              'robot model %.3f mm / %.4f deg' % (len(jf.task_idx), ep[0], ep[1], er[0], er[1]))
        if is_done and not is_orig:
            print('  RETARGETED already — nothing to do')
            record[name] = dict(state='retargeted', sha256=sha256(path))
            continue
        if not check(is_orig, 'the file is the planner ORIGINAL (or it is neither: restore it from '
                              'the backup before retargeting)'):
            continue

        q_new, info = jf.retarget(kin, planner, robot)
        q_out = np.array([[float(fmt_q(v)) for v in row] for row in q_new])   # as written
        changed = [i for i in range(jf.n) if jf.kind[i] != 'home']
        dq = np.degrees(q_out - jf.q)
        task = np.array([k == 'task' for k in jf.kind])
        per_row = np.abs(dq).max(axis=1)

        fb = info['transition_fallback']
        check(not info['ik_failures'], 'IK converged on all %d rows (at most %d iterations)%s'
              % (len(changed) - len(fb), info['ik_max_iters'],
                 '' if not info['ik_failures'] else ' — FAILED on csv lines %s'
                 % [i + 2 for i in info['ik_failures'][:10]]))
        if fb:
            check(max(f[2] for f in fb) <= args.max_transition_dev_mm,
                  '%d transition row(s) unreachable under the robot model (csv lines %s; planned at '
                  '%.0f..%.0f mm of flange reach, the straight arm) keep the planned row + the '
                  'neighbours\' correction: path deviation up to %.1f mm <= %.0f'
                  % (len(fb), [f[0] + 2 for f in fb][:10], min(f[1] for f in fb), max(f[1] for f in fb),
                     max(f[2] for f in fb), args.max_transition_dev_mm))
        check(per_row.max() <= args.max_dq_deg,
              'largest joint change %.2f deg <= %.0f (per joint: %s; median row %.2f deg; '
              '%d work points move a joint > 5 deg)'
              % (per_row.max(), args.max_dq_deg, ' '.join('%.2f' % v for v in np.abs(dq).max(axis=0)),
                 float(np.median(per_row[task])), int((per_row[task] > 5.0).sum())))
        same_cfg = all(np.sign(q_out[i, k]) == np.sign(jf.q[i, k]) or abs(jf.q[i, k]) < 1e-9
                       for i in changed for k in (2, 4))
        check(same_cfg, 'elbow (J3) and wrist (J5) keep their sign on every row — same arm '
                        'configuration as planned (J3 closest to straight: %.2f -> %.2f deg)'
              % (np.degrees(np.abs(jf.q[:, 2]).min()), np.degrees(np.abs(q_out[:, 2]).min())))
        margin = np.degrees(np.minimum(q_out - limits[:, 0], limits[:, 1] - q_out).min(axis=0))
        check(margin.min() >= args.limit_margin_deg,
              'every joint stays >= %.0f deg inside its URDF limit (closest per joint: %s)'
              % (args.limit_margin_deg, ' '.join('%.1f' % v for v in margin)))
        s0, s1 = max_step_deg(jf.q, jf.tag), max_step_deg(q_out, jf.tag)
        check(s1 <= s0 + args.max_step_growth_deg,
              'largest row-to-row joint step %.2f -> %.2f deg' % (s0, s1))
        home = [i for i in range(jf.n) if jf.kind[i] == 'home']
        check(all(np.array_equal(q_out[i], jf.q[i]) for i in home),
              '%d home rows unchanged; %d work points and %d transition rows re-solved, '
              '%d transition rows next to home blended'
              % (len(home), len(jf.task_idx),
                 jf.kind.count('transition') - info['n_blended'], info['n_blended']))
        ea = jf.pair_error(kin, robot, euler, q_out)
        check(ea[0] <= args.pair_tol_mm and ea[1] <= args.pair_tol_deg,
              'as written, FK under the ROBOT model is on the paired pose rows: %.3f mm / %.4f deg'
              % ea)
        # where the plan as it is would have put the real tip, per lane
        for zone in sorted(set(planned[t][3] for t in tags)):
            e = np.array([robot.tip_world(jf.tag[i], kin.fk(jf.q[i]), jf.lift_m)[:3, 3]
                          - np.array(jf.pose[(jf.tag[i], jf.src[i])][:3])
                          for i in jf.task_idx if planned[jf.tag[i]][3] == zone]) * 1e3
            print('       zone %s, replayed as planned: tip off by mean (%+.1f, %+.1f, %+.1f) mm world, '
                  '|xy| mean %.1f / max %.1f mm  ->  0 after'
                  % (zone, *e.mean(axis=0), np.linalg.norm(e[:, :2], axis=1).mean(),
                     np.linalg.norm(e[:, :2], axis=1).max()))
        dz = [1e3 * ((robot.T_world_arm(jf.tag[i], jf.lift_m) @ kin.fk(q_out[i]))[2, 3]
                     - (planner.T_world_arm(jf.tag[i], jf.lift_m) @ kin.fk(jf.q[i]))[2, 3]) for i in jf.task_idx]
        print('       flange height at the work points vs the plan: %+.1f .. %+.1f mm '
              '(the measured tip is shorter than the design one)' % (min(dz), max(dz)))

        todo.append((jf, q_new))
        record[name] = dict(
            state='original -> retargeted', rows=jf.n, work_points=len(jf.task_idx),
            transitions_resolved=jf.kind.count('transition') - info['n_blended'],
            transitions_blended=info['n_blended'], home_rows=len(home),
            transitions_unreachable=[dict(csv_line=f[0] + 2, planned_reach_mm=round(f[1]), deviation_mm=round(f[2], 2))
                                     for f in fb],
            max_joint_change_deg=round(float(per_row.max()), 3),
            max_joint_change_per_joint_deg=[round(float(v), 3) for v in np.abs(dq).max(axis=0)],
            pair_error_before_mm=round(er[0], 3), pair_error_after_mm=round(ea[0], 4),
            sha256_before=sha256(path))

    print('\n%d ok, %d failed' % (n_ok, n_fail))
    if n_fail:
        sys.exit('nothing written: %d check(s) failed' % n_fail)
    if not args.apply:
        print('dry run — %d file(s) would be rewritten; --apply to do it' % len(todo))
        return
    if not todo:
        print('nothing to rewrite')
        return

    # ---- back up (never over a different backup), write, read back
    backup = args.backup_dir or os.path.join(
        paths.LOG_DIR, 'apriltag_nav', 'task_csv_backup',
        datetime.date.today().strftime('%Y%m%d') + '_before_stop_pose_retarget')
    os.makedirs(backup, exist_ok=True)
    for jf, _ in todo:
        for src in (jf.path, jf.pose_path):
            dst = os.path.join(backup, os.path.basename(src))
            if os.path.exists(dst):
                if sha256(dst) != sha256(src):
                    sys.exit('nothing written: %s exists and differs from the file in %s'
                             % (dst, args.task_dir))
            else:
                shutil.copy2(src, dst)
    print('\noriginals in %s' % backup)

    for jf, q_new in todo:
        for i in range(jf.n):
            if jf.kind[i] == 'home':
                continue
            for k, col in enumerate(jf.iq):
                jf.rows[i][col] = fmt_q(q_new[i, k])
        save_text_csv(jf.path, jf.hdr, jf.rows)

        hdr0, rows0 = load_text_csv(os.path.join(backup, jf.name))
        hdr1, rows1 = load_text_csv(jf.path)
        other = sum(1 for r0, r1 in zip(rows0, rows1) for c in range(len(hdr0))
                    if r0[c] != r1[c] and c not in jf.iq)
        homes = sum(1 for i, (r0, r1) in enumerate(zip(rows0, rows1))
                    if jf.kind[i] == 'home' and r0 != r1)
        back = JointFile(jf.path, jf.pose_path)
        e = back.pair_error(kin, robot, euler)
        if hdr0 != hdr1 or len(rows0) != len(rows1) or other or homes \
                or e[0] > args.pair_tol_mm or e[1] > args.pair_tol_deg:
            sys.exit('%s: read-back FAILED (other cells changed %d, home rows changed %d, '
                     'pair error %.3f mm) — restore it from %s' % (jf.name, other, homes, e[0], backup))
        record[jf.name]['sha256_after'] = sha256(jf.path)
        print('written %s — read back: only q1..q6 of the %d non-home rows differ from the backup, '
              'pair error %.3f mm' % (jf.name, jf.n - jf.kind.count('home'), e[0]))

    n = write_diff_csv(args.diff_out, planned, actual, used_tags)
    print('stop pose difference of %d tags -> %s' % (n, args.diff_out))
    rec_path = os.path.join(backup, 'retarget_record.yaml')
    with open(rec_path, 'w') as f:
        yaml.safe_dump(dict(
            date=datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            tool='tools/retarget_joint_paths.py',
            planner=dict(stop_poses=os.path.relpath(args.planned, paths.WS_DIR),
                         stop_poses_sha256=sha256(args.planned),
                         T_ab2mb_t_mm=[float(v) for v in d_ab['t_mm']],
                         T_ab2mb_rpy_deg_xyz=[float(v) for v in d_ab['rpy_deg_xyz']],
                         tip_mm=[float(v) for v in d_tip['t_mm']]),
            robot=dict(stop_poses=os.path.relpath(args.actual, paths.WS_DIR),
                       stop_poses_sha256=sha256(args.actual),
                       T_ab2mb_t_mm=[round(v, 3) for v in tf_chain.t_mm(tf_chain.invert_T(robot.M))],
                       T_ab2mb_rpy_deg_xyz=[round(v, 4) for v in
                                            tf_chain.rpy_deg_xyz(tf_chain.invert_T(robot.M))],
                       tip_mm=[round(float(v) * 1e3, 3) for v in robot.tip]),
            files=record), f, default_flow_style=False, sort_keys=False)
    print('record -> %s' % rec_path)
    print('RELOAD_TASKS (or restart task_executor) to pick the files up')


if __name__ == '__main__':
    main()
