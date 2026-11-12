#!/usr/bin/env python3
"""Make a NEW, smaller task from a joint + pose pair: thin the middle block
to a 20 mm grid and trim one outer ring off the side patches, then renumber.

    grid_task_points.py JOINT_CSV POSE_CSV SUFFIX [--mid-half-mm 650]
                        [--grid-mm 20] [--side-trim 1] [--mid-trim 0] [--apply]

Writes  joint_<key>_<SUFFIX>.csv / pose_<key>_<SUFFIX>.csv  next to the inputs
(the inputs are only read), so TaskManager discovers a separate task
scan_joint_<key>_<SUFFIX> / scan_pose_<key>_<SUFFIX>.

Work points sit on raster lines of constant world y, 10 mm apart, ~10 mm pitch
along a line (world x y from the pose twin).

* MIDDLE block (|world x| < --mid-half-mm): only every second y line is kept
  (the line parity is chosen globally, so the groups of the block share one
  grid), and on each kept line every second point, the phase chosen so the
  kept x sit nearest the 20 mm multiples -> a 20 x 20 mm grid, ~1/4 of the
  points. The dropped points are REMOVED from the path (not driven through):
  the kept lines are re-chained, each one oriented (forward or reversed) to
  start next to where the previous one ended (serpentine), so a line change is
  a ~20 mm move.
  --mid-trim N first removes N rings (10 mm each) around the outline of the
  whole middle block (its groups together, inner group borders untouched);
  --mid-trim 2 = the outer 2 cm.
* SIDE patches: --side-trim rings of boundary points are removed (a point
  is boundary when it is the first / last of its line or has no point on the
  line above / below within 8 mm of x); the path order is otherwise kept.

Rows kept verbatim: home rows and the transition runs that are NOT a straight
joint interpolation between two work points of the same region (approach /
retreat, the region-to-region routes). Transition runs that ARE straight
(a single MoveJ cut into 0.1 rad pieces by the planner) are dropped and every
new connection whose largest joint change exceeds 0.1 rad is cut again into
joint-linear transition rows (speed 10) — the same geometry a single MoveJ
drives, the planner's step size.

Renumbering, per group: joint point_id = 1..N in path order, source_point_id
= 1..M over the work points in path order; pose point_id = that new
source_point_id (pose rows copied, re-ordered to the new path order).
Speeds: the first work point of every line / after a transition 10, the rest
of a line 30 (the planner's own rule); pose speed = the joint row's.
is_discontinuous (pose) moves to the first kept point of the stretch whose
original first point carried it.

Checks printed: counts per group / region, nearest-neighbour spacing, the
largest joint step, and for every NEW connection the vision-tip path of the
joint-linear move (URDF FK + T_ee2tip) against the straight chord — the
largest excursion along the tool z (toward the surface) must stay small.
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
LINEAR_DEV_RAD = 1e-4   # a transition run within this of the chord is "straight"
BIG_JUMP_DEG = 20.0     # an intra-region jump this large starts a new stretch


def fmt(v):
    return '%.10g' % v


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('joint')
    ap.add_argument('pose')
    ap.add_argument('suffix')
    ap.add_argument('--mid-half-mm', type=float, default=650.0)
    ap.add_argument('--grid-mm', type=float, default=20.0)
    ap.add_argument('--side-trim', type=int, default=1)
    ap.add_argument('--mid-trim', type=int, default=0,
                    help='rings (10 mm each) removed around the whole middle block before the grid')
    ap.add_argument('--fk-limit-mm', type=float, default=1.0,
                    help='refuse when a new connection dips more than this toward the surface')
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    assert abs(a.grid_mm - 20.0) < 1e-9, 'only the 20 mm grid (every second of a 10 mm raster) is implemented'

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

    n = len(rows)
    kind = [r[C['waypoint_kind']] for r in rows]
    gid = [int(float(r[C['group_id']])) for r in rows]
    q = np.array([[float(r[C['q%d' % k]]) for k in range(1, 7)] for r in rows])
    xy = [None] * n
    sp_old = [None] * n
    for i, r in enumerate(rows):
        if kind[i] == 'task':
            s = int(float(r[C['source_point_id']]))
            pr = pose_row[(gid[i], s)]
            sp_old[i] = s
            xy[i] = (float(pr[D['x']]) * 1000.0, float(pr[D['y']]) * 1000.0)
    region = [None if xy[i] is None else ('mid' if abs(xy[i][0]) < a.mid_half_mm else 'side')
              for i in range(n)]

    # ---- 1. structure: stretches of work rows, verbatim rows in between ----
    # items: ('row', i) kept verbatim, or ('stretch', [task row indices])
    items, i = [], 0
    while i < n:
        if kind[i] == 'task':
            st = [i]
            i += 1
            while i < n:
                if kind[i] == 'task' and gid[i] == gid[st[-1]] and region[i] == region[st[-1]]:
                    jump = np.degrees(np.abs(q[i] - q[st[-1]]).max())
                    if jump > BIG_JUMP_DEG:
                        break
                    st.append(i)
                    i += 1
                    continue
                if kind[i] == 'transition':
                    e = i
                    while e < n and kind[e] == 'transition':
                        e += 1
                    if e < n and kind[e] == 'task' and gid[e] == gid[st[-1]] and region[e] == region[st[-1]]:
                        A, B = q[st[-1]], q[e]
                        d = B - A
                        L2 = float(d @ d)
                        dev = max(np.linalg.norm((q[k] - A) - ((q[k] - A) @ d) / L2 * d)
                                  for k in range(i, e)) if L2 > 0 else 1.0
                        if dev < LINEAR_DEV_RAD and np.degrees(np.abs(d).max()) <= BIG_JUMP_DEG:
                            i = e            # straight intra-stretch transition: dropped
                            continue
                break
            items.append(('stretch', st))
        else:
            items.append(('row', i))
            i += 1

    # ---- 2. selection ----
    def lines_of(st):
        segs = []
        for k in st:
            yk = round(xy[k][1] / 10.0)
            if segs and segs[-1][0] == yk:
                segs[-1][1].append(k)
            else:
                segs.append((yk, [k]))
        return segs

    mid_rows = [k for t, st in items if t == 'stretch' for k in st if region[k] == 'mid']
    ypar = max((0, 1), key=lambda p: sum(1 for k in mid_rows if round(xy[k][1] / 10.0) % 2 == p))

    # x selection decided per y LINE over every middle point on it (all groups,
    # all pieces of the line), so pieces and group borders share one lattice:
    # lattice index = round((x - line offset) / 10), keep one parity — the one
    # whose points sit nearest the 20 mm multiples.
    xkeep = {}
    byline = collections.defaultdict(list)
    for k in mid_rows:
        byline[round(xy[k][1] / 10.0)].append(k)
    for yk, ks in byline.items():
        ang = np.array([xy[k][0] for k in ks]) * (2 * math.pi / 10.0)
        off = (math.atan2(np.sin(ang).mean(), np.cos(ang).mean()) * 10.0 / (2 * math.pi)) % 10.0
        idx = {k: int(round((xy[k][0] - off) / 10.0)) for k in ks}

        def cost(p):
            sel = [k for k in ks if idx[k] % 2 == p]
            return (sum(abs(xy[k][0] - 20.0 * round(xy[k][0] / 20.0)) for k in sel) / max(1, len(sel)),
                    -len(sel))
        par = min((0, 1), key=cost)
        for k in ks:
            xkeep[k] = idx[k] % 2 == par
        # the planner's path repeats a few edge points 0-6 mm apart (left
        # edge of the block): of kept points closer than 8 mm keep the first
        # one in path order
        kept = []
        for k in sorted((k for k in ks if xkeep[k])):
            if any(abs(xy[k][0] - xy[m][0]) < 8.0 for m in kept):
                xkeep[k] = False
            else:
                kept.append(k)

    def erode(st, rings):
        """st minus `rings` rings of boundary points of the set (10 mm raster)."""
        alive = set(st)
        for _ in range(rings):
            byline = collections.defaultdict(list)
            for k in alive:
                byline[round(xy[k][1] / 10.0)].append(k)
            edge = set()
            for yk, ks in byline.items():
                ks.sort(key=lambda k: xy[k][0])
                # line segments separated by gaps > 15 mm
                seg = [ks[0]]
                for u, v in zip(ks, ks[1:]):
                    if xy[v][0] - xy[u][0] > 15.0:
                        edge.update((seg[0], seg[-1]))
                        seg = []
                    seg.append(v)
                edge.update((seg[0], seg[-1]))
                for k in ks:
                    for dy in (-1, 1):
                        nb = byline.get(yk + dy, [])
                        if not any(abs(xy[m][0] - xy[k][0]) <= 8.0 for m in nb):
                            edge.add(k)
            alive -= edge
        return [k for k in st if k in alive]

    # middle block: the outer --mid-trim rings of the WHOLE block (all its
    # groups together, so the borders between groups are not trimmed)
    mid_alive = set(erode(mid_rows, a.mid_trim))

    # new sequence of ('row', i) / ('task', i, seg_start, stretch_first) / ('jump',)
    seq = []
    disc_old = {k for k, r in pose_row.items() if float(r[D['is_discontinuous']]) != 0.0}
    disc_stretch_first = set()
    stats = collections.Counter()
    kept_xy = collections.defaultdict(list)
    for t, st in items:
        if t == 'row':
            seq.append(('row', st))
            continue
        reg = region[st[0]]
        stats[(gid[st[0]], reg, 'before')] += len(st)
        flagged = (gid[st[0]], sp_old[st[0]]) in disc_old
        if reg == 'side':
            keep = erode(st, a.side_trim)
            segs = [ks for _, ks in lines_of(keep)]
        else:
            segs = []
            for yk, ks in lines_of(st):
                if yk % 2 != ypar:
                    continue
                sel = [k for k in ks if xkeep[k] and k in mid_alive]
                if sel:
                    segs.append(sel)
            # serpentine: orient each line to start next to the previous end
            prev = None
            for s_i, sel in enumerate(segs):
                if prev is not None:
                    d0 = math.dist(xy[prev], xy[sel[0]])
                    d1 = math.dist(xy[prev], xy[sel[-1]])
                    if d1 < d0:
                        sel.reverse()
                prev = sel[-1]
        first = True
        for sel in segs:
            for m, k in enumerate(sel):
                seq.append(('task', k, m == 0, first))
                if first and flagged:
                    disc_stretch_first.add(k)
                first = False
                stats[(gid[k], reg, 'after')] += 1
                kept_xy[(gid[k], reg)].append(xy[k])

    # ---- 3. emit rows: transitions where a new connection is long, ids, speeds ----
    out, src_of_new = [], []        # src_of_new: original row index (or None for inserted)
    prev_i = None
    for it in seq:
        i = it[1]
        if prev_i is not None and gid[i] == gid[prev_i] and i != prev_i + 1:
            d = q[i] - q[prev_i]
            nstep = int(math.ceil(np.abs(d).max() / STEP_RAD - 1e-9))
            # a verbatim row follows / precedes on the planner's own path:
            # only a NEW pairing is cut, an original neighbour pair is not
            for s in range(1, nstep):
                qq = q[prev_i] + d * s / nstep
                r = list(rows[prev_i])
                for k in range(6):
                    r[C['q%d' % (k + 1)]] = fmt(qq[k])
                r[C['is_task_waypoint']] = '0'
                r[C['source_point_id']] = ''
                r[C['waypoint_kind']] = 'transition'
                r[C['speed']] = '10'
                r[C['is_discontinuous']] = '0'
                out.append(r)
                src_of_new.append(None)
        r = list(rows[i])
        if it[0] == 'task':
            starts = it[2] or (out and out[-1][C['waypoint_kind']] != 'task')
            r[C['speed']] = '10' if starts else '30'
            r[C['is_task_waypoint']] = '1'   # a kept point is scanned, even from a thinned input
        out.append(r)
        src_of_new.append(i)
        prev_i = i

    # renumber
    pid_n = collections.Counter()
    sp_n = collections.Counter()
    new_pose = []
    old_to_new_sp = {}
    for r, src in zip(out, src_of_new):
        g = int(float(r[C['group_id']]))
        pid_n[g] += 1
        r[C['point_id']] = str(pid_n[g])
        if r[C['waypoint_kind']] == 'task':
            sp_n[g] += 1
            old_to_new_sp[(g, sp_old[src])] = sp_n[g]
            r[C['source_point_id']] = str(sp_n[g])
            pr = list(pose_row[(g, sp_old[src])])
            pr[D['point_id']] = str(sp_n[g])
            pr[D['speed']] = r[C['speed']]
            one = '1.000000000000000000e+00'
            zero = '0.000000000000000000e+00'
            pr[D['is_discontinuous']] = one if src in disc_stretch_first else zero
            new_pose.append(pr)

    # ---- 4. checks ----
    print('%s\n  -> %s\n  -> %s' % (a.joint, out_j, out_p))
    print('  middle y-line parity kept: y = %s (mod 20 mm)' % ('10' if ypar else '0'))
    tb = ta = 0
    for (g, reg) in sorted({(k[0], k[1]) for k in stats}):
        b, af = stats[(g, reg, 'before')], stats[(g, reg, 'after')]
        tb, ta = tb + b, ta + af
        print('    group %d %-4s %5d -> %5d  (%.2f)' % (g, reg, b, af, af / b if b else 0))
    mb = sum(stats[k] for k in stats if k[1] == 'mid' and k[2] == 'before')
    ma = sum(stats[k] for k in stats if k[1] == 'mid' and k[2] == 'after')
    sb = sum(stats[k] for k in stats if k[1] == 'side' and k[2] == 'before')
    sa = sum(stats[k] for k in stats if k[1] == 'side' and k[2] == 'after')
    print('  middle %d -> %d (%.3f), sides %d -> %d, total work points %d -> %d'
          % (mb, ma, ma / mb, sb, sa, tb, ta))
    print('  rows %d -> %d' % (n, len(out)))
    for reg in ('mid', 'side'):
        pts = [p for (g, rr), ps in kept_xy.items() if rr == reg for p in ps]
        nn = []
        for p in pts:
            dd = [math.dist(p, o) for o in pts if o is not p and abs(o[0] - p[0]) < 60 and abs(o[1] - p[1]) < 60]
            if dd:
                nn.append(min(dd))
        nn.sort()
        print('  %s nearest neighbour: median %.1f mm, min %.1f, max %.1f'
              % (reg, nn[len(nn) // 2], nn[0], nn[-1]))

    qn = np.array([[float(r[C['q%d' % k]]) for k in range(1, 7)] for r in out])
    steps = np.degrees(np.abs(np.diff(qn, axis=0)).max(axis=1))
    orig_pairs = set(zip(range(n - 1), range(1, n)))
    new_conn = [m for m in range(len(out) - 1)
                if (src_of_new[m], src_of_new[m + 1]) not in orig_pairs
                and int(float(out[m][C['group_id']])) == int(float(out[m + 1][C['group_id']]))]
    print('  largest joint step: %.2f deg overall, %.2f deg on new connections (%d)'
          % (steps.max(), max(steps[m] for m in new_conn), len(new_conn)))

    from apriltag_nav.arm_fk import ArmChain
    from apriltag_nav.tf_chain import tip_offset_mm
    arm = ArmChain()
    tip = np.append(tip_offset_mm() / 1000.0, 1.0)

    def tip_and_z(qr):
        T = arm.fk_flange(np.degrees(qr))
        return (T @ tip)[:3], T[:3, 2]
    worst_dip, worst_any, worst_at = 0.0, 0.0, None
    for m in new_conn:
        A, B = qn[m], qn[m + 1]
        pa, za = tip_and_z(A)
        pb, _ = tip_and_z(B)
        for s in np.linspace(0, 1, 11)[1:-1]:
            p, _ = tip_and_z(A + (B - A) * s)
            chord = pa + (pb - pa) * s
            dev = p - chord
            dip = float(dev @ za)
            worst_any = max(worst_any, float(np.linalg.norm(dev)))
            if dip > worst_dip:
                worst_dip, worst_at = dip, m
    print('  new connections, tip vs straight chord: largest deviation %.3f mm, '
          'largest toward the surface (tool +z) %.3f mm' % (worst_any * 1000, worst_dip * 1000))
    if worst_dip * 1000 > a.fk_limit_mm:
        r = out[worst_at]
        sys.exit('  REFUSED: connection after group %s row %s dips %.2f mm toward the surface'
                 % (r[C['group_id']], r[C['point_id']], worst_dip * 1000))

    # every task row has its pose twin, numbering consistent
    assert len(new_pose) == ta
    assert sum(1 for r in out if r[C['waypoint_kind']] == 'task') == ta

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
