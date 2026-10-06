#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-express a planner export made for 정반 1 as a task pair for 정반 2, with
every point pushed a fixed distance further along the vision tip's axis.

    python3 tools/make_plate2_paths.py SRC_DIR                 # dry run, writes nothing
    python3 tools/make_plate2_paths.py SRC_DIR --apply         # write into task/csv

SRC_DIR holds planner ORIGINALS (pose_<key>.csv + joint_<key>.csv pairs
under TaskManager's prefixes — rename an export first; the 2026-09-29 하형 set is
generated in the 정반 1 frame although the workpiece sits on 정반 2). For each
pair the tool writes pose_<key><suffix>.csv and
joint_<key><suffix>.csv (suffix `_plate2`), which `TaskManager`
discovers as scan_pose_<key>_plate2 / scan_joint_<key>_plate2.

What changes
------------
1. World frame -> 정반 2: x += --dx (3.900 m = the measured plate spacing;
   reference_tags_plate2.yaml puts 정반 2's centre there). The D / E floor
   lanes in map.yaml sit at +3.89 (never re-laid), so the base stands 10 mm
   further from the workpiece than on 정반 1 — the joint re-solve below
   absorbs it, pose mode absorbs it through /robot_pose at run time.
2. Group tags -> their 정반 2 twins: zone B -> D and C -> E, matched on the
   tag's design y (105 -> 131, 106 -> 132, 107 -> 133, 118 -> 143,
   119 -> 144, 120 -> 145). Refused when a tag has no twin.
3. Every point --tip-down-mm (20) further along the TOOL z axis (the vision
   tip's approach direction; the tool is up to 28 deg off vertical in these
   files, so this is not the same as world -z): pose rows p += d * R[:, 2]
   with R the ZYX flange orientation of the row; joint rows the same on the
   planned tip pose before the IK.

The joint rows are RE-SOLVED, not copied: a joint row is an absolute
configuration, and the planner's model (idle 정반 1 stop pose, DESIGN mount,
DESIGN tip — see tools/retarget_joint_paths.py) is replaced by the robot's
(map.yaml stop pose of the 정반 2 tag, calibrated T_ab2mb, measured tip),
exactly as the retarget tool does for 정반 1:

    P      = W_p(tag1) @ M_p @ FK(q) @ TIP_p          planned tip pose, world
    P'     = P shifted by (dx, 0, 0) + d * P[:, 2]    정반 2, tip pushed down
    target = inv(W_r(tag2) @ M_r) @ P' @ inv(TIP_r)   flange, real arm frame
    q_new  = IK(target) seeded with q                 same branch as planned

Work points and transitions between work points exactly; transitions next to
a `home` row blend the correction to 0 at home; `home` rows untouched. Both
files then get the standing speed-10 rule (tools/slow_task_entry.py: both
rows of every transition -> task boundary).

The result classifies as RETARGETED in tools/retarget_joint_paths.py (FK under
the robot model is on the paired pose rows), so a later run of that tool
leaves it alone; tools/check_retarget_joint_paths.py holds it to pose mode's
own arithmetic like every other joint file.

Not covered: the joint zero offsets (joint mode never applies them), the
per-arrival stop error, and whether the 정반 2 tags 126-150 are where map.yaml
says (they are DESIGN values — plate 2 has never been map-calibrated).
"""
import argparse
import datetime
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))
sys.path.insert(0, HERE)

from apriltag_nav import paths, tf_chain                      # noqa: E402
from apriltag_nav.arm_fk import ArmChain                      # noqa: E402
import retarget_joint_paths as rj                             # noqa: E402

TWIN_ZONE = {'B': 'D', 'C': 'E'}
SLOW_TOOL = os.path.join(HERE, 'slow_task_entry.py')


def fmt_m(v):
    """Metres as the planner writes them: repr (shortest round trip)."""
    return repr(float(v))


def load_map_tags(map_path):
    with open(map_path) as f:
        return {int(k): v for k, v in yaml.safe_load(f)['tags'].items()}


def twin_tags(tags, map_tags, tol_m=0.05):
    """{plate-1 tag: plate-2 tag} — the zone D / E tag on the same 0.4 m grid
    row (nearest y, within tol_m), zone B -> D / C -> E. The 정반 1 y is the
    CALIBRATED one (up to ~6 mm off the design row), the 정반 2 lanes are
    design values; the grid pitch is 400 mm, so 50 mm is unambiguous."""
    out = {}
    for t in tags:
        zone = str(map_tags[t].get('zone', ''))
        if zone not in TWIN_ZONE:
            raise ValueError('tag %d is in zone %r, not a 정반 1 work lane (B / C)' % (t, zone))
        want = TWIN_ZONE[zone]
        y = float(map_tags[t]['y'])
        cand = sorted((abs(float(v['y']) - y), u) for u, v in map_tags.items()
                      if str(v.get('zone', '')) == want and abs(float(v['y']) - y) <= tol_m)
        if not cand:
            raise ValueError('tag %d (zone %s, y %.3f): no twin within %.0f mm in zone %s'
                             % (t, zone, y, tol_m * 1e3, want))
        out[t] = cand[0][1]
    return out


def load_pose_rows(path):
    hdr, rows = rj.load_text_csv(path)
    need = ['group_id', 'point_id', 'x', 'y', 'z', 'rx', 'ry', 'rz']
    missing = [c for c in need if c not in hdr]
    if missing:
        raise ValueError('%s: missing column(s) %s' % (os.path.basename(path), missing))
    return hdr, rows


def convert_pose(hdr, rows, twins, dx, d, euler):
    """New rows: x += dx, p += d * tool z, group_id -> twin. Others byte-identical."""
    ig, ix, iy, iz = (hdr.index(c) for c in ('group_id', 'x', 'y', 'z'))
    irpy = [hdr.index(c) for c in ('rx', 'ry', 'rz')]
    out, shift = [], []
    for r in rows:
        r = list(r)
        p = np.array([float(r[ix]), float(r[iy]), float(r[iz])])
        Rm = rj.euler_matrix(euler, [float(r[i]) for i in irpy])
        p2 = p + np.array([dx, 0.0, 0.0]) + d * Rm[:, 2]
        shift.append(p2 - p - np.array([dx, 0.0, 0.0]))
        r[ix], r[iy], r[iz] = fmt_m(p2[0]), fmt_m(p2[1]), fmt_m(p2[2])
        r[ig] = str(twins[int(float(r[ig]))])
        out.append(r)
    return out, np.array(shift)


def convert_joint(jf, kin, planner, robot, twins, dx, d):
    """(q_new, info) for the plate-2 twin of a planner-original joint file."""
    exact = jf.q.copy()
    fails, iters, shift = [], 0, np.zeros((jf.n, 3))
    inv_tip_r = tf_chain.invert_T(robot.TIP)
    inv_arm_r = {t: tf_chain.invert_T(robot.T_world_arm(twins[t], jf.lift_m)) for t in sorted(set(jf.tag))}
    targets = {}
    for i in range(jf.n):
        if jf.kind[i] == 'home':
            continue
        P = planner.tip_world(jf.tag[i], kin.fk(jf.q[i]), jf.lift_m)
        P2 = P.copy()
        P2[:3, 3] += np.array([dx, 0.0, 0.0]) + d * P[:3, 2]
        shift[i] = P2[:3, 3] - P[:3, 3] - np.array([dx, 0.0, 0.0])
        target = inv_arm_r[jf.tag[i]] @ P2 @ inv_tip_r
        exact[i], ok, it = kin.ik(target, jf.q[i])
        iters = max(iters, it)
        if not ok:
            fails.append(i)
            targets[i] = target
    # A row the robot cannot reach on 정반 2 (the base stands 10 mm further
    # from the plate than on 정반 1, the measured tip is shorter, and the
    # tip-down adds reach on a tilted tool — a point the planner put at the
    # straight-arm limit ends a few mm beyond it): keep the planned row +
    # the correction interpolated from the nearest solved rows of its group,
    # so the arm configuration stays smooth, and report how far the tip
    # misses. Work points too — pose mode refuses the same point with an IK
    # error at run time, so the miss is not hidden there either.
    unreachable = []
    failset = set(fails)
    for i in fails:
        lo, hi = i - 1, i + 1
        while lo >= 0 and jf.tag[lo] == jf.tag[i] and lo in failset:
            lo -= 1
        while hi < jf.n and jf.tag[hi] == jf.tag[i] and hi in failset:
            hi += 1
        lo_ok = lo >= 0 and jf.tag[lo] == jf.tag[i]
        hi_ok = hi < jf.n and jf.tag[hi] == jf.tag[i]
        if not (lo_ok or hi_ok):
            continue
        d_lo = exact[lo] - jf.q[lo] if lo_ok else None
        d_hi = exact[hi] - jf.q[hi] if hi_ok else None
        if lo_ok and hi_ok:
            seg = jf.q[lo:hi + 1]
            s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
            f = s[i - lo] / s[-1] if s[-1] > 0 else 0.5
            corr = (1.0 - f) * d_lo + f * d_hi
        else:
            corr = d_lo if lo_ok else d_hi
        exact[i] = jf.q[i] + corr
        dev = 1e3 * float(np.linalg.norm(rj.pose_error(kin.fk(exact[i]), targets[i])[:3]))
        reach = 1e3 * float(np.linalg.norm(targets[i][:3, 3]))
        unreachable.append((i, jf.kind[i], jf.src[i], reach, dev))
    fails = [k for k in fails if k not in {u[0] for u in unreachable}]
    q_new = exact.copy()
    n_blend = 0
    for a, i, j, b in jf.runs():
        if jf.kind[a] != 'home' and jf.kind[b] != 'home':
            continue                                  # re-solved exactly
        seg = jf.q[a:b + 1]
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(seg, axis=0), axis=1))])
        s = s / s[-1] if s[-1] > 0 else np.linspace(0.0, 1.0, len(s))
        da, db = exact[a] - jf.q[a], exact[b] - jf.q[b]
        for k in range(i, j):
            f = s[k - a]
            q_new[k] = jf.q[k] + (1.0 - f) * da + f * db
            n_blend += 1
    return q_new, dict(ik_failures=fails, ik_max_iters=iters, n_blended=n_blend, shift=shift,
                       unreachable=unreachable)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('src_dir', help='directory of planner-original pairs (정반 1 frame)')
    ap.add_argument('--out', default=paths.TASK_DIR, help='default: %(default)s')
    ap.add_argument('--suffix', default='_plate2')
    ap.add_argument('--dx', type=float, default=3.900,
                    help='world x of 정반 2\'s centre, m (default %(default)s)')
    ap.add_argument('--tip-down-mm', type=float, default=20.0,
                    help='push every point this far along the tool z axis (default %(default)s)')
    ap.add_argument('--planned', default=rj.PLANNED_DEFAULT,
                    help='stop poses the planner assumed on 정반 1 (default: %(default)s)')
    ap.add_argument('--map', default=paths.MAP_PATH)
    ap.add_argument('--apply', action='store_true', help='write the files; default is a dry run')
    ap.add_argument('--force', action='store_true', help='overwrite existing output files')
    ap.add_argument('--record-dir', default=None,
                    help='default: <ws>/log/apriltag_nav/task_csv_backup/<today>_plate2_from_<src>')
    ap.add_argument('--pair-tol-mm', type=float, default=0.05)
    ap.add_argument('--pair-tol-deg', type=float, default=0.01)
    ap.add_argument('--max-dq-deg', type=float, default=20.0)
    ap.add_argument('--limit-margin-deg', type=float, default=5.0)
    ap.add_argument('--max-step-growth-deg', type=float, default=0.5)
    ap.add_argument('--max-unreachable', type=int, default=5,
                    help='refuse when more rows than this are beyond the reach (default %(default)s)')
    ap.add_argument('--max-unreachable-mm', type=float, default=10.0,
                    help='refuse when an unreachable row misses its target by more than this (default %(default)s)')
    args = ap.parse_args()
    d = args.tip_down_mm / 1e3

    n_ok = n_fail = 0

    def check(cond, text):
        nonlocal n_ok, n_fail
        if cond:
            n_ok += 1
        else:
            n_fail += 1
        print('  %s %s' % ('ok  ' if cond else 'FAIL', text))
        return bool(cond)

    robot_cfg = paths.load_config()
    euler = str((robot_cfg.get('arm_calibration') or {}).get('csv_euler', 'ZYX'))
    map_tags = load_map_tags(args.map)
    planned = rj.read_stop_poses(args.planned)
    d_ab, d_tip = tf_chain.load_entry('T_ab2mb')['design'], tf_chain.load_entry('T_ee2tip')['design']
    planner = rj.Model('planner', {t: v[:3] for t, v in planned.items()},
                       tf_chain.matrix_from_pose(np.array(d_ab['t_mm']) / 1e3, d_ab['rpy_deg_xyz']),
                       np.array(d_tip['t_mm']) / 1e3)
    robot = rj.Model('robot', rj.stop_poses_from_map(args.map, robot_cfg),
                     tf_chain.load_transform('T_ab2mb'), tf_chain.tip_offset_mm() / 1e3)
    chain = ArmChain()
    kin = rj.Kinematics(chain)
    limits = rj.urdf_joint_limits(chain.path)

    print('source   %s' % args.src_dir)
    print('planner: stop poses %s' % args.planned)
    print('         %s, tip (%.1f, %.1f, %.1f) mm'
          % (tf_chain.describe(tf_chain.invert_T(planner.M), 'T_ab2mb (design)'), *(planner.tip * 1e3)))
    print('robot:   stop poses from %s (정반 2 tags are design values)' % os.path.basename(args.map))
    print('         %s, tip (%.1f, %.1f, %.1f) mm'
          % (tf_chain.describe(tf_chain.invert_T(robot.M), 'T_ab2mb (applied)'), *(robot.tip * 1e3)))
    print('정반 2 at world x %+.3f m; every point %.1f mm further along the tool z; CSV euler %s'
          % (args.dx, args.tip_down_mm, euler))

    names = sorted(f for f in os.listdir(args.src_dir)
                   if f.startswith(rj.JOINT_PREFIX) and f.lower().endswith('.csv'))
    if not names:
        sys.exit('no %s*.csv in %s' % (rj.JOINT_PREFIX, args.src_dir))

    tmp = tempfile.mkdtemp(prefix='plate2_')
    outputs, record = [], {}
    for name in names:
        key = name[len(rj.JOINT_PREFIX):-4]
        jpath = os.path.join(args.src_dir, name)
        ppath = os.path.join(args.src_dir, rj.POSE_PREFIX + key + '.csv')
        print('\n== %s' % key)
        if not check(os.path.exists(ppath), 'paired %s exists' % os.path.basename(ppath)):
            continue
        jf = rj.JointFile(jpath, ppath)
        tags = sorted(set(jf.tag))
        try:
            twins = twin_tags(tags, map_tags)
        except ValueError as e:
            check(False, str(e))
            continue
        print('  tags %s -> %s' % (tags, [twins[t] for t in tags]))
        if not check(all(t in planner.stop for t in tags), 'every group tag has a planned stop pose'):
            continue
        ep = jf.pair_error(kin, planner, euler)
        if not check(ep[0] <= args.pair_tol_mm and ep[1] <= args.pair_tol_deg,
                     'the pair is a planner ORIGINAL: FK under the planner model on the pose rows '
                     '%.3f mm / %.4f deg' % ep):
            continue

        # ---- pose file
        phdr, prows = load_pose_rows(ppath)
        prows2, pshift = convert_pose(phdr, prows, twins, args.dx, d, euler)
        pn = np.linalg.norm(pshift, axis=1) * 1e3
        check(abs(pn.min() - args.tip_down_mm) < 1e-6 and abs(pn.max() - args.tip_down_mm) < 1e-6,
              'pose rows: every point moved exactly %.1f mm along its tool z (world dz %+.1f .. %+.1f mm, '
              '|dxy| up to %.1f mm)' % (args.tip_down_mm, pshift[:, 2].min() * 1e3, pshift[:, 2].max() * 1e3,
                                        np.linalg.norm(pshift[:, :2], axis=1).max() * 1e3))

        # ---- joint file
        q_new, info = convert_joint(jf, kin, planner, robot, twins, args.dx, d)
        q_out = np.array([[float(rj.fmt_q(v)) for v in row] for row in q_new])
        changed = [i for i in range(jf.n) if jf.kind[i] != 'home']
        home = [i for i in range(jf.n) if jf.kind[i] == 'home']
        dq = np.degrees(q_out - jf.q)
        task = np.array([k == 'task' for k in jf.kind])
        per_row = np.abs(dq).max(axis=1)
        unr = info['unreachable']
        unr_rows = {u[0] for u in unr}
        check(not info['ik_failures'], 'IK converged on all %d rows (at most %d iterations)%s'
              % (len(changed) - len(unr), info['ik_max_iters'],
                 '' if not info['ik_failures'] else ' — FAILED on csv lines %s'
                 % [i + 2 for i in info['ik_failures'][:10]]))
        if unr:
            n_work = sum(1 for u in unr if u[1] == 'task')
            check(len(unr) <= args.max_unreachable and max(u[4] for u in unr) <= args.max_unreachable_mm,
                  '%d row(s) beyond the arm\'s reach on 정반 2 (%d work point(s), flange target %.0f..%.0f mm): '
                  'kept on the planned row + the neighbours\' correction, tip misses by up to %.1f mm '
                  '(limits %d rows / %.0f mm): %s'
                  % (len(unr), n_work, min(u[3] for u in unr), max(u[3] for u in unr), max(u[4] for u in unr),
                     args.max_unreachable, args.max_unreachable_mm,
                     ', '.join('line %d %s%s %.1f mm' % (u[0] + 2, u[1], ' pt %s' % u[2] if u[2] is not None else '', u[4])
                               for u in unr)))
        check(per_row.max() <= args.max_dq_deg,
              'largest joint change %.2f deg <= %.0f (per joint: %s; median work-point row %.2f deg)'
              % (per_row.max(), args.max_dq_deg, ' '.join('%.2f' % v for v in np.abs(dq).max(axis=0)),
                 float(np.median(per_row[task]))))
        same_cfg = all(np.sign(q_out[i, k]) == np.sign(jf.q[i, k]) or abs(jf.q[i, k]) < 1e-9
                       for i in changed for k in (2, 4))
        check(same_cfg, 'elbow (J3) and wrist (J5) keep their sign on every row (J3 closest to straight: '
                        '%.2f -> %.2f deg)' % (np.degrees(np.abs(jf.q[:, 2]).min()), np.degrees(np.abs(q_out[:, 2]).min())))
        margin = np.degrees(np.minimum(q_out - limits[:, 0], limits[:, 1] - q_out).min(axis=0))
        check(margin.min() >= args.limit_margin_deg,
              'every joint stays >= %.0f deg inside its URDF limit (closest per joint: %s)'
              % (args.limit_margin_deg, ' '.join('%.1f' % v for v in margin)))
        s0, s1 = rj.max_step_deg(jf.q, jf.tag), rj.max_step_deg(q_out, jf.tag)
        check(s1 <= s0 + args.max_step_growth_deg, 'largest row-to-row joint step %.2f -> %.2f deg' % (s0, s1))
        check(all(np.array_equal(q_out[i], jf.q[i]) for i in home),
              '%d home rows unchanged; %d work points and %d transition rows re-solved, %d blended'
              % (len(home), len(jf.task_idx), jf.kind.count('transition') - info['n_blended'], info['n_blended']))

        # as written: FK under the robot model at the TWIN tag vs the NEW pose rows
        ig, ipid = phdr.index('group_id'), phdr.index('point_id')
        ixyz = [phdr.index(c) for c in ('x', 'y', 'z', 'rx', 'ry', 'rz')]
        pose2 = {(int(float(r[ig])), int(float(r[ipid]))): [float(r[i]) for i in ixyz] for r in prows2}
        e_mm = e_deg = 0.0
        for i in jf.task_idx:
            if i in unr_rows:
                continue
            p = pose2[(twins[jf.tag[i]], jf.src[i])]
            T = robot.tip_world(twins[jf.tag[i]], kin.fk(q_out[i]), jf.lift_m)
            e_mm = max(e_mm, 1e3 * float(np.linalg.norm(T[:3, 3] - p[:3])))
            Rc = rj.euler_matrix(euler, p[3:])
            e_deg = max(e_deg, math.degrees(float(np.linalg.norm(rj.rotvec(Rc.T @ T[:3, :3])))))
        check(e_mm <= args.pair_tol_mm and e_deg <= args.pair_tol_deg,
              'as written, the real tip at the 정반 2 stop pose is on the new pose rows: %.3f mm / %.4f deg%s'
              % (e_mm, e_deg, '' if not unr else ' (%d unreachable work point(s) excluded)'
                 % sum(1 for u in unr if u[1] == 'task')))
        jn = np.linalg.norm(info['shift'][jf.task_idx], axis=1) * 1e3
        check(abs(jn.min() - args.tip_down_mm) < 1e-6 and abs(jn.max() - args.tip_down_mm) < 1e-6,
              'joint work points: planned tip moved exactly %.1f mm along its tool z before the IK'
              % args.tip_down_mm)
        dz = [1e3 * ((robot.T_world_arm(twins[jf.tag[i]], jf.lift_m) @ kin.fk(q_out[i]))[2, 3]
                     - (planner.T_world_arm(jf.tag[i], jf.lift_m) @ kin.fk(jf.q[i]))[2, 3]) for i in jf.task_idx]
        print('       flange height at the work points vs the 정반 1 plan: %+.1f .. %+.1f mm '
              '(-%.0f tip-down, the measured tip is shorter than the design one)' % (min(dz), max(dz), args.tip_down_mm))
        reach = [1e3 * float(np.linalg.norm(kin.fk(q_out[i])[:3, 3])) for i in jf.task_idx]
        print('       flange reach at the work points %.0f .. %.0f mm' % (min(reach), max(reach)))

        # ---- write the pair into the scratch dir
        jhdr, jrows = jf.hdr, [list(r) for r in jf.rows]
        ig_j = jhdr.index('group_id')
        for i, r in enumerate(jrows):
            for c, col in enumerate(jf.iq):
                r[col] = rj.fmt_q(q_new[i][c])
            r[ig_j] = str(twins[jf.tag[i]])
        out_j = os.path.join(tmp, rj.JOINT_PREFIX + key + args.suffix + '.csv')
        out_p = os.path.join(tmp, rj.POSE_PREFIX + key + args.suffix + '.csv')
        rj.save_text_csv(out_j, jhdr, jrows)
        rj.save_text_csv(out_p, phdr, prows2)
        outputs += [out_j, out_p]
        record[key] = dict(
            source_joint=dict(file=name, sha256=rj.sha256(jpath)),
            source_pose=dict(file=os.path.basename(ppath), sha256=rj.sha256(ppath)),
            tags={int(t): int(twins[t]) for t in tags}, rows=jf.n, work_points=len(jf.task_idx),
            transitions_resolved=jf.kind.count('transition') - info['n_blended'],
            transitions_blended=info['n_blended'], home_rows=len(home),
            max_joint_change_deg=round(float(per_row.max()), 3),
            max_joint_change_per_joint_deg=[round(float(v), 3) for v in np.abs(dq).max(axis=0)],
            unreachable=[dict(csv_line=u[0] + 2, kind=u[1], point=u[2], flange_target_mm=round(u[3]),
                              tip_miss_mm=round(u[4], 2)) for u in unr],
            pair_error_after_mm=round(float(e_mm), 4),
            flange_dz_mm=[round(float(min(dz)), 1), round(float(max(dz)), 1)],
            flange_reach_mm=[round(float(min(reach))), round(float(max(reach)))])

    # ---- the standing speed-10 rule on the new pair(s)
    if outputs:
        print('\n== speed-10 rule (tools/slow_task_entry.py)')
        p = subprocess.run([sys.executable, SLOW_TOOL, tmp, '--apply'], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, universal_newlines=True)
        print('\n'.join('  ' + l for l in p.stdout.rstrip().split('\n')))
        check(p.returncode == 0, 'speed rule applied')
        p = subprocess.run([sys.executable, SLOW_TOOL, tmp], stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, universal_newlines=True)
        check(p.returncode == 0 and 'changed 0, task rows changed 0' in p.stdout
              and 'mismatch' not in p.stdout.replace('0 mismatch', ''),
              'second pass changes nothing')

    print('\n%d ok, %d failed' % (n_ok, n_fail))
    if n_fail or not outputs:
        shutil.rmtree(tmp, ignore_errors=True)
        sys.exit('refusing: %d check(s) failed, nothing written' % n_fail if n_fail else 'nothing to write')

    targets = [os.path.join(args.out, os.path.basename(f)) for f in outputs]
    exists = [t for t in targets if os.path.exists(t)]
    if exists and not args.force:
        shutil.rmtree(tmp, ignore_errors=True)
        sys.exit('refusing: output exists (--force to overwrite): %s' % [os.path.basename(t) for t in exists])
    if not args.apply:
        print('\nDRY RUN — would write %s into %s' % ([os.path.basename(t) for t in targets], args.out))
        shutil.rmtree(tmp, ignore_errors=True)
        return

    rec_dir = args.record_dir or os.path.join(
        paths.WS_DIR, 'log', 'apriltag_nav', 'task_csv_backup',
        '%s_plate2_from_%s' % (datetime.date.today().strftime('%Y%m%d'),
                               os.path.basename(os.path.normpath(args.src_dir)) or 'src'))
    os.makedirs(rec_dir, exist_ok=True)
    for key in record:
        for k in ('source_joint', 'source_pose'):
            shutil.copy2(os.path.join(args.src_dir, record[key][k]['file']), rec_dir)
    os.makedirs(args.out, exist_ok=True)
    for src, dst in zip(outputs, targets):
        shutil.copy2(src, dst)
        record.setdefault('outputs', []).append(dict(file=os.path.basename(dst), sha256=rj.sha256(dst)))
        print('wrote %s' % dst)
    record['settings'] = dict(dx_m=args.dx, tip_down_mm=args.tip_down_mm, suffix=args.suffix,
                              csv_euler=euler, planned=args.planned, map=args.map,
                              T_ab2mb_applied_t_mm=[round(float(v), 3) for v in tf_chain.load_transform('T_ab2mb')[:3, 3] * 1e3],
                              tip_mm=[round(float(v), 3) for v in tf_chain.tip_offset_mm()],
                              date=datetime.datetime.now().isoformat(timespec='seconds'))
    with open(os.path.join(rec_dir, 'make_plate2_record.yaml'), 'w') as f:
        yaml.safe_dump(record, f, sort_keys=False, allow_unicode=True)
    print('sources + record in %s' % rec_dir)
    shutil.rmtree(tmp, ignore_errors=True)
    print('RELOAD_TASKS (or restart task_executor) to pick the files up')


if __name__ == '__main__':
    main()
