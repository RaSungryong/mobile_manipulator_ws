#!/usr/bin/env python3
"""Check the straight approach `TASK <task> start=<group>:<point>` would drive.

    check_direct_start.py <task_name> <group>:<source_point_id> [--surface POSE_CSV ...]

Builds the plan exactly as task_executor does (scan_resume.start_done_points
-> plan_resume -> direct_start) from task/csv, then follows the vision tip
(URDF FK + T_ee2tip) and the case bottom (17 mm above the tip along the tool
axis) along the joint-linear approach home -> start point, and prints their
height above the work surface at fixed distances before the target plus the
lowest tip clearance more than 20 mm before it. The surface is every pose
point of the task's pose twin, or of --surface files (give the planner's
ORIGINAL pose file so trimmed edges count). arm frame -> world is fitted on
the group's own work points. Only the tip / case is checked — no link model.
Exit 1 when the tip comes lower than 15 mm above the surface more than
20 mm before the target.
"""
import argparse
import math
import os
import sys
import tempfile

import numpy as np

from apriltag_nav import scan_resume as SR
from apriltag_nav.arm_fk import ArmChain
from apriltag_nav.paths import TASK_DIR
from apriltag_nav.task_manager import TaskManager
from apriltag_nav.tf_chain import tip_offset_mm


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('task')
    ap.add_argument('start')
    ap.add_argument('--surface', nargs='*', default=None)
    ap.add_argument('--min-clearance-mm', type=float, default=15.0)
    a = ap.parse_args()
    _, start, err = SR.parse_start_arg(['start=' + a.start])
    if err:
        sys.exit(err)
    tm = TaskManager(TASK_DIR, result_dir=tempfile.mkdtemp())
    steps = tm.get_task(a.task)
    if not steps:
        sys.exit('unknown task %s' % a.task)
    points = tm.scan_points[a.task]
    done, why = SR.start_done_points(steps, points, start)
    if why:
        sys.exit(why)
    plan = SR.plan_resume('x_ra_map_x', a.task, steps, points, done, '/dev/null')
    g0 = int(start[0])
    pts, dropped, added = SR.direct_start(plan.points_by_tag[g0], start[1])
    print('%s start=%d:%d: %d rows before the start point not driven, %d approach rows; group %d sends %d rows'
          % (a.task, g0, start[1], dropped, added, g0, len(pts)))
    if not dropped:
        print('nothing to check (no straight approach)')
        return
    arm = ArmChain()
    tip = np.append(tip_offset_mm() / 1000.0, 1.0)

    def fk(q):
        return arm.fk_flange(np.degrees(np.asarray(q, float)))
    work = [p for p in points[g0] if p.get('scan', True) and 'x' in p]
    A_ = np.array([(fk(p['joints']) @ tip)[:3] for p in work])
    W_ = np.array([[p['x'], p['y'], p['z']] for p in work])
    ca, cw = A_.mean(0), W_.mean(0)
    U, S, Vt = np.linalg.svd((A_ - ca).T @ (W_ - cw))
    R = (U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt).T
    res = np.linalg.norm((A_ - ca) @ R.T + cw - W_, axis=1)
    print('arm->world fit on %d work points: max %.2f mm' % (len(work), res.max() * 1000))
    if a.surface:
        surf = []
        for f in a.surface:
            import pandas as pd
            d = pd.read_csv(f, encoding='utf-8-sig')
            surf.append(d[['x', 'y', 'z']].values)
        surf = np.vstack(surf)
    else:
        surf = np.array([[p['x'], p['y'], p['z']] for tag in tm.scan_points[a.task]
                         for p in tm.scan_points[a.task][tag] if 'x' in p])
    print('surface: %d points' % len(surf))

    q0 = np.array(pts[0]['joints'], float)
    q1 = np.array(pts[added + 1]['joints'], float)
    ts = np.linspace(0, 1, 800)
    tips, cases = [], []
    for t in ts:
        M = fk(q0 + (q1 - q0) * t)
        p = (M @ tip)[:3]
        tips.append((p - ca) @ R.T + cw)
        cases.append(((p - 0.017 * M[:3, 2]) - ca) @ R.T + cw)
    tips, cases = np.array(tips), np.array(cases)
    d_end = np.r_[np.cumsum(np.linalg.norm(np.diff(tips[::-1], axis=0), axis=1))[::-1], 0.0]

    def cl(p):
        m = np.hypot(surf[:, 0] - p[0], surf[:, 1] - p[1]) < 0.015
        return p[2] - surf[m, 2].max() if m.any() else float('nan')
    print('approach %.0f mm long; height above the surface (nan = nothing below):' % (d_end[0] * 1000))
    for D in (5, 10, 20, 50, 100, 200, 400):
        i = int(np.argmin(abs(d_end - D / 1000.0)))
        print('  %4d mm before the target: tip %6.1f mm, case bottom %6.1f mm' % (D, cl(tips[i]) * 1000, cl(cases[i]) * 1000))
    far = [cl(p) for p, d in zip(tips, d_end) if d > 0.02]
    far = [v for v in far if not math.isnan(v)]
    lo = min(far) * 1000 if far else float('inf')
    print('lowest tip clearance more than 20 mm before the target: %s' % ('%.1f mm' % lo if far else 'never over the surface'))
    if lo < a.min_clearance_mm:
        print('NOT OK: below %.0f mm' % a.min_clearance_mm)
        sys.exit(1)
    print('OK')


if __name__ == '__main__':
    main()
