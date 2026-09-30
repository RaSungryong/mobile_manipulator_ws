#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_retarget_joint_paths.py
=============================
OFFLINE self-check for tools/retarget_joint_paths.py (2026-09-29) — no ROS
master, no robot::

    python3 src/apriltag_nav/tools/check_retarget_joint_paths.py

What it guards
--------------
A joint-mode row is an absolute joint configuration: it puts the tool where
the planner meant only if base stop pose, arm mount and tip are the planner's.
The tool re-solves the rows for the robot's. Two things can go wrong silently,
and both leave a file that loads and runs:

  * the retarget is applied the wrong way round, twice, or to a file that is
    not the planner's original (the correction then DOUBLES the error);
  * the tool's own model composition disagrees with what pose mode does at
    run time, so "retargeted" joint rows and their pose-mode twins land in
    two places.

Part 1 runs the real tool end to end on a SYNTHETIC pair in a scratch
directory (real planner joint rows embedded below, so it does not depend on
task/csv, which the user replaces). Part 2 reads the REAL task/csv and holds
every joint file to pose mode's own arithmetic — the production
`transform_world_to_arm` + the tip -> flange conversion — at the nominal stop
pose of each group tag.
"""
import csv
import math
import os
import shutil
import subprocess
import sys
import tempfile
import types

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))
sys.path.insert(0, HERE)

# arm_transform asks rospy for private params and logs one line per call. The
# stub is forced (also where rospy is installed): the check must not read a
# live param server.
_r = types.ModuleType('rospy')
_r.get_param = lambda name, default=None: default
for _f in ('loginfo', 'logwarn', 'logerr', 'logerr_throttle', 'logwarn_throttle'):
    setattr(_r, _f, lambda *a, **k: None)
sys.modules['rospy'] = _r

from apriltag_nav import paths, tf_chain                         # noqa: E402
from apriltag_nav.arm_fk import ArmChain                         # noqa: E402
from apriltag_nav import arm_transform                           # noqa: E402
from apriltag_nav.arm_transform import transform_world_to_arm    # noqa: E402
import retarget_joint_paths as rj                                # noqa: E402

# transform_world_to_arm re-reads robot.yaml and tf_chain.yaml on every call
# (0.2 s each); thousands of work points need the files once. Same values,
# same arithmetic.
_load_transform, _load_block, _memo = tf_chain.load_transform, arm_transform.load_yaml_block, {}


def _cached(fn):
    def wrapper(*a):
        key = (fn.__name__,) + a
        if key not in _memo:
            _memo[key] = fn(*a)
        return _memo[key]
    return wrapper


tf_chain.load_transform = _cached(_load_transform)
arm_transform.load_yaml_block = _cached(_load_block)

TOOL = os.path.join(HERE, 'retarget_joint_paths.py')
STOP_TOOL = os.path.join(HERE, 'map_stop_poses.py')

_n = [0]
_bad = [0]


def check(name, ok, detail=''):
    _n[0] += 1
    if not ok:
        _bad[0] += 1
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")
    return bool(ok)


class Msg(object):
    """/robot_pose at a base pose given in the world frame."""
    def __init__(self, x_w, y_w, yaw_deg):
        self.x, self.y, self.theta = -y_w, -x_w, yaw_deg


def rot_xyz_deg(rpy_deg):
    from scipy.spatial.transform import Rotation as R
    r = R.from_euler('xyz', rpy_deg, degrees=True)
    return r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()


def pose_mode_flange(pose_row, stop, tip_mm, lift_m=0.0):
    """The flange target `_exec_pose` sends to IK for one work point with the
    base on `stop` (lift at `lift_m`, design floor): arm frame, mm + matrix."""
    g = dict(zip(('x', 'y', 'z', 'rx', 'ry', 'rz'), pose_row))
    pos_tip, rpy = transform_world_to_arm(g, Msg(*stop), lift_m)
    Rm = rot_xyz_deg(rpy)
    return np.asarray(pos_tip, dtype=float) - Rm @ tip_mm, Rm


def run_tool(*args):
    p = subprocess.run([sys.executable, TOOL] + list(args), stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, universal_newlines=True)
    return p.returncode, p.stdout


# ---------------------------------------------------------------- synthetic pair
# Real planner rows (standoff 010, 2026-09-29 export): four work points of a
# zone B group and four of a zone C group, the last of each the most stretched
# one of its group — the rows the correction moves furthest.
HOME = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
WORK = {
    105: [[-1.859292128, -0.7903474002, 1.438616139, -2.187707151, -1.50892738, -0.2890452318],
          [-1.460353201, -0.9772095918, 1.812106137, -2.426790988, -1.563866811, 0.1104195846],
          [-1.162584106, -0.8888974924, 1.646760276, -2.355434701, -1.598407883, 0.4081004114],
          [-1.1918476, -0.6154540395, 1.038934607, -1.954451626, -1.582372135, 0.3787583709]],
    119: [[-1.539814768, -0.1882359682, 0.2293210666, -1.692589179, -1.579525913, 0.03087787822],
          [-1.682816343, -0.5547095995, 1.050338421, -2.162590254, -1.564189005, -0.1115045989],
          [-2.12344823, -0.3538229887, 0.6254084716, -1.891102212, -1.488287219, -0.5525388129],
          [-1.825981672, -0.144918183, 0.1281514114, -1.618702513, -1.531627972, -0.2547040323]],
}
J_HDR = ('target_line,group_id,point_id,q1,q2,q3,q4,q5,q6,is_discontinuous,is_task_waypoint,'
         'source_point_id,waypoint_kind,speed,base_normal_error_mm,base_x_actual_mm,standoff_mm,'
         'base_height_mm,lift_mm').split(',')
P_HDR = ('target_line,group_id,point_id,x,y,z,rx,ry,rz,speed,is_discontinuous,coordinate_frame,'
         'base_normal_error_mm,base_x_actual_mm,standoff_mm,base_height_mm,lift_mm').split(',')
SYN_MAP_IDLE = {105: dict(x=-1.71, y=0.15, zone='B'), 119: dict(x=1.71, y=-0.55, zone='C')}
SYN_MAP_REAL = {105: dict(x=-1.7015, y=0.1489, yaw=-0.14, zone='B'),
                119: dict(x=1.6929, y=-0.5496, yaw=-179.93, zone='C')}
KEY = 'synthetic_standoff_010mm_height_652mm'


def lerp_rows(a, b, n):
    a, b = np.asarray(a), np.asarray(b)
    return [list(a + (b - a) * k / (n + 1.0)) for k in range(1, n + 1)]


def build_pair(task_dir, planner, kin):
    """home, 3 transitions, work, 2 transitions, work, work, transition, work,
    3 transitions, home — per group."""
    from scipy.spatial.transform import Rotation as R
    jrows, prows = [], []
    for line, tag in ((1, 105), (2, 119)):
        w = WORK[tag]
        seq = ([('home', HOME, None)]
               + [('transition', q, None) for q in lerp_rows(HOME, w[0], 3)]
               + [('task', w[0], 1)]
               + [('transition', q, None) for q in lerp_rows(w[0], w[1], 2)]
               + [('task', w[1], 2), ('task', w[2], 3)]
               + [('transition', q, None) for q in lerp_rows(w[2], w[3], 1)]
               + [('task', w[3], 4)]
               + [('transition', q, None) for q in lerp_rows(w[3], HOME, 3)]
               + [('home', HOME, None)])
        for pid, (kind, q, src) in enumerate(seq, 1):
            qs = ['%.4f' % v if kind == 'home' else '%.10g' % v for v in q]
            qs = [s.rstrip('0').rstrip('.') if kind == 'home' and '.' in s else s for s in qs]
            jrows.append([str(line), str(tag), str(pid)] + qs
                         + ['0', '1' if kind == 'task' else '0', '' if src is None else str(src),
                            kind, '30', '0', '-1610' if line == 1 else '1610', '10', '652', '0'])
            if kind != 'task':
                continue
            T = planner.tip_world(tag, kin.fk(np.array([float(s) for s in qs])))
            rot = (R.from_matrix(T[:3, :3]) if hasattr(R, 'from_matrix') else R.from_dcm(T[:3, :3]))
            e = rot.as_euler('ZYX')
            prows.append([str(line), str(tag), str(src)] + ['%.17g' % v for v in T[:3, 3]]
                         + ['%.17g' % v for v in e]
                         + ['30', '0.000000000000000000e+00', 'zone_bc_work_m_v1', '0',
                            '-1610' if line == 1 else '1610', '10', '652', '0'])
    jp = os.path.join(task_dir, rj.JOINT_PREFIX + KEY + '.csv')
    pp = os.path.join(task_dir, rj.POSE_PREFIX + KEY + '.csv')
    rj.save_text_csv(jp, J_HDR, jrows)
    rj.save_text_csv(pp, P_HDR, prows)
    return jp, pp


def write_map(path, tags):
    with open(path, 'w') as f:
        yaml.safe_dump({'tags': {t: dict(v, type='WORK') for t, v in tags.items()}}, f)


def models():
    d_ab = tf_chain.load_entry('T_ab2mb')['design']
    d_tip = tf_chain.load_entry('T_ee2tip')['design']
    T_design = tf_chain.matrix_from_pose(np.array(d_ab['t_mm']) / 1e3, d_ab['rpy_deg_xyz'])
    return T_design, np.array(d_tip['t_mm']) / 1e3


def part1(kin):
    print('== part 1: the tool end to end on a synthetic pair')
    tmp = tempfile.mkdtemp(prefix='check_retarget_')
    try:
        task_dir = os.path.join(tmp, 'csv')
        os.makedirs(task_dir)
        idle_map, real_map = os.path.join(tmp, 'map_idle.yaml'), os.path.join(tmp, 'map.yaml')
        write_map(idle_map, SYN_MAP_IDLE)
        write_map(real_map, SYN_MAP_REAL)
        planned, actual = os.path.join(tmp, 'idle.csv'), os.path.join(tmp, 'actual.csv')
        for m, out in ((idle_map, planned), (real_map, actual)):
            subprocess.run([sys.executable, STOP_TOOL, '--map', m, '--ids', 'all', '--out', out],
                           check=True, stdout=subprocess.DEVNULL)
        stop_p, stop_a = rj.read_stop_poses(planned), rj.read_stop_poses(actual)
        check('stop poses: idle = tag - 0.55 m at exactly +-90 deg; calibrated carries the tag yaw',
              stop_p[105][:3] == (-1.71, -0.4, 90.0) and stop_p[119][:3] == (1.71, 0.0, -90.0)
              and abs(stop_a[105][2] - 89.86) < 1e-9 and abs(stop_a[119][2] + 89.93) < 1e-9,
              f'{stop_p[105][:3]} {stop_a[105][:3]}')

        T_design, tip_design = models()
        planner = rj.Model('planner', {t: v[:3] for t, v in stop_p.items()}, T_design, tip_design)
        robot = rj.Model('robot', {t: v[:3] for t, v in stop_a.items()},
                         tf_chain.load_transform('T_ab2mb'), tf_chain.tip_offset_mm() / 1e3)
        jp, pp = build_pair(task_dir, planner, kin)
        raw0, pose0 = open(jp, 'rb').read(), open(pp, 'rb').read()
        backup, diff = os.path.join(tmp, 'backup'), os.path.join(tmp, 'diff.csv')
        common = [task_dir, '--planned', planned, '--actual', actual, '--map', real_map,
                  '--backup-dir', backup, '--diff-out', diff]

        rc, out = run_tool(*common)
        check('dry run: exit 0, nothing written, no backup directory',
              rc == 0 and open(jp, 'rb').read() == raw0 and not os.path.exists(backup)
              and 'dry run' in out, out.strip().splitlines()[-1])

        rc, out = run_tool(*(common + ['--apply']))
        raw1 = open(jp, 'rb').read()
        check('--apply: exit 0 and the joint file changed', rc == 0 and raw1 != raw0,
              out.strip().splitlines()[-1])
        check('the pose file is untouched', open(pp, 'rb').read() == pose0)
        check('the backup holds both originals byte for byte, plus the record',
              open(os.path.join(backup, os.path.basename(jp)), 'rb').read() == raw0
              and open(os.path.join(backup, os.path.basename(pp)), 'rb').read() == pose0
              and os.path.exists(os.path.join(backup, 'retarget_record.yaml')))
        check('BOM and CRLF kept', raw1[:3] == rj.BOM and raw1.endswith(b'\r\n')
              and raw1.count(b'\n') == raw1.count(b'\r\n') == raw0.count(b'\r\n'))

        hdr, rows0 = rj.load_text_csv(os.path.join(backup, os.path.basename(jp)))
        _, rows1 = rj.load_text_csv(jp)
        iq = [hdr.index(c) for c in rj.Q_COLS]
        ik = hdr.index('waypoint_kind')
        other = [(i, c) for i, (a, b) in enumerate(zip(rows0, rows1)) for c in range(len(hdr))
                 if a[c] != b[c] and c not in iq]
        check('only q1..q6 cells differ', not other and len(rows0) == len(rows1), str(other[:3]))
        check('home rows are unchanged, every other row moved',
              all((a == b) == (a[ik] == 'home') for a, b in zip(rows0, rows1)))
        check('no exponent notation in the written cells',
              not any('e' in r[c].lower() for r in rows1 for c in iq))

        jf0 = rj.JointFile(os.path.join(backup, os.path.basename(jp)),
                           os.path.join(backup, os.path.basename(pp)))
        jf1 = rj.JointFile(jp, pp)
        e0, e1 = jf0.pair_error(kin, robot, 'ZYX'), jf1.pair_error(kin, robot, 'ZYX')
        check('work points: the real tip was off the planned point, and is on it now',
              e0[0] > 5.0 and e1[0] < 0.01 and e1[1] < 1e-3,
              f'{e0[0]:.1f} mm / {e0[1]:.2f} deg -> {e1[0]:.4f} mm / {e1[1]:.5f} deg')

        # the same through pose mode's own code, not the tool's model
        tip_mm = tf_chain.tip_offset_mm()
        worst = worst_r = 0.0
        for i in jf1.task_idx:
            want, Rm = pose_mode_flange(jf1.pose[(jf1.tag[i], jf1.src[i])],
                                        stop_a[jf1.tag[i]][:3], tip_mm)
            T = kin.fk(jf1.q[i])
            worst = max(worst, float(np.linalg.norm(T[:3, 3] * 1e3 - want)))
            worst_r = max(worst_r, math.degrees(float(np.linalg.norm(rj.rotvec(Rm.T @ T[:3, :3])))))
        check('work points: FK of the written row == the flange target pose mode computes',
              worst < 0.01 and worst_r < 1e-3, f'{worst:.4f} mm / {worst_r:.5f} deg')

        # transitions between work points follow the planned world path
        worst = 0.0
        n_mid = 0
        for a, i, j, b in jf0.runs():
            if 'home' in (jf0.kind[a], jf0.kind[b]):
                continue
            for k in range(i, j):
                n_mid += 1
                P0 = planner.tip_world(jf0.tag[k], kin.fk(jf0.q[k]))
                P1 = robot.tip_world(jf1.tag[k], kin.fk(jf1.q[k]))
                worst = max(worst, 1e3 * float(np.linalg.norm(P1[:3, 3] - P0[:3, 3])),
                            1e3 * float(np.linalg.norm(rj.rotvec(P0[:3, :3].T @ P1[:3, :3]))))
        check(f'{n_mid} transition rows between work points: the real tip is on the planned world path',
              n_mid == 6 and worst < 0.01, f'{worst:.4f} mm')

        # runs next to home: correction fades from the work point's to 0 at home
        ok, detail = True, ''
        n_home = 0
        for a, i, j, b in jf0.runs():
            if 'home' not in (jf0.kind[a], jf0.kind[b]):
                continue
            w = b if jf0.kind[a] == 'home' else a           # the work-point end
            dw = jf1.q[w] - jf0.q[w]
            for k in range(i, j):
                n_home += 1
                f = (k - a) / float(b - a) if w == b else (b - k) / float(b - a)
                d = jf1.q[k] - jf0.q[k]
                if np.abs(d - f * dw).max() > 2e-9:          # equal joint steps -> linear in the row index
                    ok, detail = False, f'row {k}: {d} vs {f * dw}'
        check(f'{n_home} transition rows next to home: the work point\'s correction x (0..1), '
              'so the run still ends on the home joints', ok and n_home == 12, detail)

        rc, out = run_tool(*(common + ['--apply']))
        check('second --apply: recognised as RETARGETED, file untouched',
              rc == 0 and 'RETARGETED already' in out and open(jp, 'rb').read() == raw1,
              out.strip().splitlines()[-1])

        # a file that is neither the original nor the retargeted one
        rows_bad = [list(r) for r in rows0]
        t = next(i for i, r in enumerate(rows_bad) if r[ik] == 'task')
        rows_bad[t][iq[1]] = rj.fmt_q(float(rows_bad[t][iq[1]]) + math.radians(1.0))
        rj.save_text_csv(jp, hdr, rows_bad)
        raw_bad = open(jp, 'rb').read()
        rc, out = run_tool(*(common + ['--apply']))
        check('a file that is neither (one work point 1 deg off its pose row) is refused, untouched',
              rc != 0 and open(jp, 'rb').read() == raw_bad and 'nothing written' in out,
              out.strip().splitlines()[-1])

        # the original again, but the actual stop poses no longer the map's
        rj.save_text_csv(jp, hdr, rows0)
        stale_map = os.path.join(tmp, 'map_moved.yaml')
        write_map(stale_map, {105: dict(SYN_MAP_REAL[105], x=-1.6965), 119: SYN_MAP_REAL[119]})
        args = [task_dir, '--planned', planned, '--actual', actual, '--map', stale_map,
                '--backup-dir', os.path.join(tmp, 'backup2'), '--diff-out', diff, '--apply']
        rc, out = run_tool(*args)
        check('stop poses that disagree with the map (tag 105 moved 5 mm) are refused, untouched',
              rc != 0 and open(jp, 'rb').read() == raw0 and 'DIFFERENT for tags [105]' in out,
              out.strip().splitlines()[-1])

        # a backup directory holding a DIFFERENT file of the same name is never overwritten
        b3 = os.path.join(tmp, 'backup3')
        os.makedirs(b3)
        with open(os.path.join(b3, os.path.basename(jp)), 'wb') as f:
            f.write(raw1)
        rc, out = run_tool(*(common[:-4] + ['--backup-dir', b3, '--diff-out', diff, '--apply']))
        check('an existing, different backup is not overwritten and nothing is written',
              rc != 0 and open(jp, 'rb').read() == raw0
              and open(os.path.join(b3, os.path.basename(jp)), 'rb').read() == raw1,
              out.strip().splitlines()[-1])

        with open(diff, newline='') as f:
            d = {int(r['tag_id']): r for r in csv.DictReader(f)}
        check('difference table: base at tag 105 moved (+7.2, -1.1) mm world = 1.1 back, 7.2 right in the body',
              d[105]['d_world_x_mm'] == '+7.2' and d[105]['d_world_y_mm'] == '-1.1'
              and d[105]['d_body_fwd_mm'] == '-1.1' and d[105]['d_body_left_mm'] == '-7.2'
              and d[105]['d_yaw_deg'] == '-0.14' and d[105]['used_by_task_csv'] == 'yes',
              str({k: v for k, v in d[105].items() if k.startswith('d_')}))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def part2(kin):
    print('== part 2: the joint files in %s' % os.path.relpath(paths.TASK_DIR, paths.WS_DIR))
    names = sorted(f for f in os.listdir(paths.TASK_DIR)
                   if f.startswith(rj.JOINT_PREFIX) and f.lower().endswith('.csv'))
    if not names:
        print('  (no joint files)')
        return
    actual = rj.read_stop_poses(rj.ACTUAL_DEFAULT)
    from_map = rj.stop_poses_from_map(paths.MAP_PATH, paths.load_config())
    stale = [t for t, v in actual.items() if t in from_map
             and (math.hypot(v[0] - from_map[t][0], v[1] - from_map[t][1]) > 1.5e-4
                  or abs(rj.wrap180(v[2] - from_map[t][2])) > 0.011)]
    check('%s is the current map.yaml\'s stop poses' % os.path.basename(rj.ACTUAL_DEFAULT),
          not stale, 'differs for tags %s' % stale if stale else '')
    for t, v in from_map.items():          # tags the csv does not list (정반 2 lanes of *_plate2 pairs)
        actual.setdefault(t, v)
    tip_mm = tf_chain.tip_offset_mm()
    for name in names:
        pose_path = os.path.join(paths.TASK_DIR, rj.POSE_PREFIX + name[len(rj.JOINT_PREFIX):])
        if not os.path.exists(pose_path):
            check(name, False, 'no paired pose file')
            continue
        jf = rj.JointFile(os.path.join(paths.TASK_DIR, name), pose_path)
        worst = worst_r = 0.0
        n_off = 0
        for i in jf.task_idx:
            want, Rm = pose_mode_flange(jf.pose[(jf.tag[i], jf.src[i])], actual[jf.tag[i]][:3], tip_mm, jf.lift_m)
            T = kin.fk(jf.q[i])
            e = float(np.linalg.norm(T[:3, 3] * 1e3 - want))
            n_off += e >= 0.05
            worst = max(worst, e)
            worst_r = max(worst_r, math.degrees(float(np.linalg.norm(rj.rotvec(Rm.T @ T[:3, :3])))))
        # a handful of points off = rows the arm cannot reach, kept on the
        # planned row by make_plate2_paths.py (listed in its record yaml);
        # (nearly) all points off = the file was never retargeted
        check(name.replace(rj.JOINT_PREFIX, ''), worst < 0.05 and worst_r < 0.01,
              f'{len(jf.task_idx)} work points, lift {jf.lift_m * 1e3:.0f} mm, joint row vs pose mode\'s flange target: '
              f'{worst:.3f} mm / {worst_r:.4f} deg'
              + ('' if worst < 0.05 else
                 f' — {n_off} of {len(jf.task_idx)} work points off by >= 0.05 mm: '
                 + ('unreachable rows kept on the planned row (see the make_plate2 record)'
                    if n_off <= 10 else 'NOT retargeted for this map: tools/retarget_joint_paths.py --apply')))


def main():
    kin = rj.Kinematics(ArmChain())
    part1(kin)
    part2(kin)
    print(f'\n{_n[0] - _bad[0]} ok, {_bad[0]} failed')
    sys.exit(1 if _bad[0] else 0)


if __name__ == '__main__':
    main()
