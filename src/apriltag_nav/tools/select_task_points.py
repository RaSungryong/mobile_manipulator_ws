#!/usr/bin/env python3
"""Make a NEW task that scans only a hand-picked list of work points of a
joint + pose pair.

    select_task_points.py JOINT_CSV POSE_CSV SUFFIX
                          --points 122:207,210,213 106:761,764 ...
                          [--surface POSE_CSV ...] [--keep-order] [--speed 10]
                          [--max-dip-mm 1.0] [--min-approach-mm 10] [--apply]

Writes  joint_<key>_<SUFFIX>.csv / pose_<key>_<SUFFIX>.csv  next to the inputs
(the inputs are only read), so TaskManager discovers a separate task
scan_joint_<key>_<SUFFIX> / scan_pose_<key>_<SUFFIX>. Only the groups that
have a selected point are written; the others are not part of the new task.

The path, per group (nothing of the input's route between the selected points
is driven):

    home row (verbatim)
    -> straight joint-linear approach to the first selected point
    -> the selected points, each reached by ONE joint-linear move from the
       previous one
    -> straight joint-linear retreat -> home row (verbatim)

Every move whose largest joint change exceeds 0.1 rad is cut into joint-linear
transition rows (not scanned) — the same geometry a single MoveJ drives, the
planner's step size, and the shape `TASK ... start=` already gives an approach.

Order: the selected points are taken in the input's path order and grouped
into raster lines (constant world y); each line is oriented to start next to
where the previous one ended (serpentine), so a line change is a short move.
--keep-order keeps the input's path order as it is.

Numbers: `source_point_id` (joint) / `point_id` (pose) are KEPT from the
input, so the UI, the frame names and the result CSVs show the numbers the
points were picked by. Only the joint file's path index `point_id` is
renumbered 1..N per group. Speed: --speed on every row.

Checks (exit 1, nothing written, when one fails) — the vision tip (URDF FK +
T_ee2tip) and the case bottom (17 mm above the tip along the tool axis) are
followed along every new move and compared with the work SURFACE = every pose
point of --surface (default: the input pose file; give the planner's ORIGINAL
pose file so thinned / trimmed points count), a plane fitted to the surface
points within 25 mm:
  * point -> point: the tip may not go more than --max-dip-mm BELOW the
    surface (a straight move over a convex surface cuts the corner);
  * approach / retreat: more than 20 mm away from the work point the tip must
    be at least --min-approach-mm above the surface.
Only the tip / case is checked — there is no link model, and nothing is known
about the die outside the surface points.
Dry run by default; --apply writes the two files (refuses to overwrite).
"""
import argparse
import collections
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from thin_scan_points import load, save  # noqa: E402

STEP_RAD = 0.1          # planner's transition resample step (5.73 deg)
CASE_M = 0.017          # case bottom above the vision tip, along the tool axis
SURF_R_M = 0.025        # surface plane fitted to the points within this radius
NEAR_M = 0.020          # approach / retreat: the last 20 mm are the landing


def fmt(v):
    return '%.10g' % v


def parse_points(tokens):
    """['122:207,210', '106:761'] -> {122: [207, 210], 106: [761]} (order kept)."""
    sel = collections.OrderedDict()
    for tok in tokens:
        if ':' not in tok:
            sys.exit('bad --points token %r (want GROUP:id,id,...)' % tok)
        g, ids = tok.split(':', 1)
        lst = sel.setdefault(int(g), [])
        for part in ids.replace(';', ',').split(','):
            part = part.strip()
            if not part:
                continue
            if '-' in part:
                a, b = part.split('-', 1)
                lst.extend(range(int(a), int(b) + 1))
            else:
                lst.append(int(part))
    for g, lst in sel.items():
        dup = [s for s, c in collections.Counter(lst).items() if c > 1]
        if dup:
            sys.exit('group %d: point(s) listed twice: %s' % (g, dup))
    return sel


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('joint')
    ap.add_argument('pose')
    ap.add_argument('suffix')
    ap.add_argument('--points', nargs='+', required=True, metavar='GROUP:id,id,...')
    ap.add_argument('--surface', nargs='*', default=None,
                    help='pose file(s) whose points are the work surface (default: POSE_CSV)')
    ap.add_argument('--keep-order', action='store_true',
                    help="keep the input's path order (no serpentine)")
    ap.add_argument('--speed', type=int, default=10)
    ap.add_argument('--max-dip-mm', type=float, default=1.0)
    ap.add_argument('--min-approach-mm', type=float, default=10.0)
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    sel = parse_points(a.points)

    jdir, jbase = os.path.split(os.path.abspath(a.joint))
    pdir, pbase = os.path.split(os.path.abspath(a.pose))
    assert jbase.startswith('joint_') and pbase.startswith('pose_')
    key = jbase[len('joint_'):-4]
    assert pbase == 'pose_%s.csv' % key, 'pose file must have the same key'
    out_j = os.path.join(jdir, 'joint_%s_%s.csv' % (key, a.suffix))
    out_p = os.path.join(pdir, 'pose_%s_%s.csv' % (key, a.suffix))

    J, P = load(a.joint), load(a.pose)
    jh, ph = J['hdr'], P['hdr']
    C = {c: jh.index(c) for c in jh}
    D = {c: ph.index(c) for c in ph}
    rows = J['rows']
    pose_row = {(int(float(r[D['group_id']])), int(float(r[D['point_id']]))): r for r in P['rows']}

    def qof(r):
        return np.array([float(r[C['q%d' % k]]) for k in range(1, 7)])

    by_group = collections.OrderedDict()
    for i, r in enumerate(rows):
        by_group.setdefault(int(float(r[C['group_id']])), []).append(i)
    for g in sel:
        if g not in by_group:
            sys.exit('group %d is not in %s (its groups: %s)' % (g, jbase, list(by_group)))

    from apriltag_nav.arm_fk import ArmChain
    from apriltag_nav.tf_chain import tip_offset_mm
    arm = ArmChain()
    tip = np.append(tip_offset_mm() / 1000.0, 1.0)

    def fk(qr):
        return arm.fk_flange(np.degrees(qr))

    surf_files = a.surface if a.surface else [a.pose]
    surf = []
    for f in surf_files:
        S = load(f)
        sc = {c: S['hdr'].index(c) for c in S['hdr']}
        surf += [[float(r[sc['x']]), float(r[sc['y']]), float(r[sc['z']])] for r in S['rows']]
    surf = np.array(surf)

    def clearance(p):
        """Height of world point p above the surface (m); nan = no surface below."""
        m = np.hypot(surf[:, 0] - p[0], surf[:, 1] - p[1]) < SURF_R_M
        if m.sum() < 3:
            return float('nan')
        A = np.c_[surf[m, 0] - p[0], surf[m, 1] - p[1], np.ones(int(m.sum()))]
        return p[2] - np.linalg.lstsq(A, surf[m, 2], rcond=None)[0][2]

    print('%s\n  -> %s\n  -> %s' % (a.joint, out_j, out_p))
    print('  surface: %d points from %s' % (len(surf), ', '.join(os.path.basename(f) for f in surf_files)))

    out, new_pose, failed = [], [], []
    for g, idx in by_group.items():           # the input's group order
        if g not in sel:
            continue
        task_i = {int(float(rows[i][C['source_point_id']])): i for i in idx
                  if rows[i][C['waypoint_kind']] == 'task'}
        missing = [s for s in sel[g] if s not in task_i]
        if missing:
            sys.exit('group %d has no work point %s (its source_point_id run %d..%d)'
                     % (g, missing, min(task_i), max(task_i)))
        nopose = [s for s in sel[g] if (g, s) not in pose_row]
        if nopose:
            sys.exit('group %d: no pose row for %s' % (g, nopose))
        h0, h1 = idx[0], idx[-1]
        if rows[h0][C['waypoint_kind']] != 'home' or rows[h1][C['waypoint_kind']] != 'home':
            sys.exit('group %d does not start and end with a home row' % g)

        def xy(s):
            pr = pose_row[(g, s)]
            return float(pr[D['x']]) * 1000.0, float(pr[D['y']]) * 1000.0

        # path order, raster lines, serpentine
        order = sorted(sel[g], key=lambda s: task_i[s])
        lines = []
        for s in order:
            yk = round(xy(s)[1] / 10.0)
            if lines and lines[-1][0] == yk:
                lines[-1][1].append(s)
            else:
                lines.append((yk, [s]))
        prev = None
        for _, ln in lines:
            if (not a.keep_order and prev is not None
                    and math.dist(xy(prev), xy(ln[-1])) < math.dist(xy(prev), xy(ln[0]))):
                ln.reverse()
            prev = ln[-1]
        seq = [s for _, ln in lines for s in ln]

        # arm frame -> world, fitted on every work point of the group
        ws = sorted(task_i)
        A_ = np.array([(fk(qof(rows[task_i[s]])) @ tip)[:3] for s in ws])
        W_ = np.array([[float(pose_row[(g, s)][D[c]]) for c in 'xyz'] for s in ws])
        ca, cw = A_.mean(0), W_.mean(0)
        U, _, Vt = np.linalg.svd((A_ - ca).T @ (W_ - cw))
        R = (U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt).T
        fit = np.linalg.norm((A_ - ca) @ R.T + cw - W_, axis=1).max() * 1000

        def follow(qa, qb, n=400):
            tips, cases = [], []
            for t in np.linspace(0, 1, n):
                M = fk(qa + (qb - qa) * t)
                p = (M @ tip)[:3]
                tips.append((p - ca) @ R.T + cw)
                cases.append(((p - CASE_M * M[:3, 2]) - ca) @ R.T + cw)
            return np.array(tips), np.array(cases)

        print('  group %d: %d of %d work points, %d line(s); arm->world fit max %.2f mm'
              % (g, len(seq), len(task_i), len(lines), fit))
        print('    order: %s' % ' '.join(str(s) for s in seq))

        # --- emit ---
        g_out = []

        def emit_move(qa, tmpl, qb):
            """Transition rows of the joint-linear move qa -> qb (ends excluded)."""
            d = qb - qa
            nstep = int(math.ceil(np.abs(d).max() / STEP_RAD - 1e-9))
            for s in range(1, nstep):
                r = list(tmpl)
                qq = qa + d * s / nstep
                for k in range(6):
                    r[C['q%d' % (k + 1)]] = fmt(qq[k])
                r[C['is_task_waypoint']] = '0'
                r[C['source_point_id']] = ''
                r[C['waypoint_kind']] = 'transition'
                r[C['speed']] = str(a.speed)
                r[C['is_discontinuous']] = '0'
                g_out.append(r)
            return max(0, nstep - 1)

        q_home0, q_home1 = qof(rows[h0]), qof(rows[h1])
        g_out.append(list(rows[h0]))
        q_prev, name_prev = q_home0, 'home'
        worst_dip, n_tr = 0.0, 0
        for s in seq:
            src = rows[task_i[s]]
            q_s = qof(src)
            n_tr += emit_move(q_prev, src, q_s)
            r = list(src)
            r[C['is_task_waypoint']] = '1'
            r[C['speed']] = str(a.speed)
            g_out.append(r)
            pr = list(pose_row[(g, s)])
            pr[D['speed']] = str(a.speed)
            new_pose.append(pr)
            # check the move that reaches this point
            tips, cases = follow(q_prev, q_s)
            cl_t = np.array([clearance(p) for p in tips]) * 1000
            cl_c = np.array([clearance(p) for p in cases]) * 1000
            length = np.linalg.norm(np.diff(tips, axis=0), axis=1).sum() * 1000
            step = np.degrees(np.abs(q_s - q_prev).max())
            if name_prev == 'home':
                d_end = np.r_[np.cumsum(np.linalg.norm(np.diff(tips[::-1], axis=0), axis=1))[::-1], 0.0]
                far = cl_t[(d_end > NEAR_M) & ~np.isnan(cl_t)]
                lo = far.min() if len(far) else float('inf')
                ok = lo >= a.min_approach_mm
                print('    approach home -> %d: %.0f mm, %.1f deg; tip above the surface beyond the last 20 mm: '
                      'lowest %.1f mm; case bottom lowest %.1f mm%s'
                      % (s, length, step, lo, np.nanmin(cl_c), '' if ok else '   <-- TOO LOW'))
                if not ok:
                    failed.append('group %d approach to %d' % (g, s))
            else:
                dip = -np.nanmin(cl_t)
                worst_dip = max(worst_dip, dip)
                ok = dip <= a.max_dip_mm
                print('    %5s -> %-5d %6.1f mm, %5.2f deg; tip lowest %+.2f mm, case bottom lowest %.2f mm%s'
                      % (name_prev, s, length, step, -dip, np.nanmin(cl_c), '' if ok else '   <-- DIPS'))
                if not ok:
                    failed.append('group %d move %s -> %d' % (g, name_prev, s))
            q_prev, name_prev = q_s, str(s)
        n_tr += emit_move(q_prev, rows[task_i[seq[-1]]], q_home1)
        g_out.append(list(rows[h1]))
        tips, cases = follow(q_prev, q_home1)
        cl_t = np.array([clearance(p) for p in tips]) * 1000
        cl_c = np.array([clearance(p) for p in cases]) * 1000
        d_sta = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(tips, axis=0), axis=1))]
        far = cl_t[(d_sta > NEAR_M) & ~np.isnan(cl_t)]
        lo = far.min() if len(far) else float('inf')
        ok = lo >= a.min_approach_mm
        print('    retreat %d -> home: %.0f mm, %.1f deg; tip above the surface beyond the first 20 mm: '
              'lowest %.1f mm; case bottom lowest %.1f mm%s'
              % (seq[-1], d_sta[-1] * 1000, np.degrees(np.abs(q_home1 - q_prev).max()), lo,
                 np.nanmin(cl_c), '' if ok else '   <-- TOO LOW'))
        if not ok:
            failed.append('group %d retreat from %d' % (g, seq[-1]))

        for n, r in enumerate(g_out, 1):
            r[C['point_id']] = str(n)
        qn = np.array([qof(r) for r in g_out])
        print('    rows %d (%d scanned, %d transition, 2 home); largest joint step %.2f deg; '
              'largest dip between points %.2f mm'
              % (len(g_out), len(seq), n_tr, np.degrees(np.abs(np.diff(qn, axis=0)).max()), worst_dip))
        out += g_out

    assert len(new_pose) == sum(len(v) for v in sel.values())
    assert sum(1 for r in out if r[C['waypoint_kind']] == 'task') == len(new_pose)
    if failed:
        sys.exit('  REFUSED, nothing written: ' + '; '.join(failed))
    if not a.apply:
        print('  dry run (--apply to write)')
        return
    for f in (out_j, out_p):
        if os.path.exists(f):
            sys.exit('  %s exists — refusing to overwrite' % f)
    save(out_j, dict(J, rows=out))
    save(out_p, dict(P, rows=new_pose))
    print('  written')


if __name__ == '__main__':
    main()
