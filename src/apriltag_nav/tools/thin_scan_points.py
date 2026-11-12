#!/usr/bin/env python3
"""Photograph only part of a joint path's work points; drive through the rest.

    thin_scan_points.py JOINT_CSV POSE_CSV [--keep 2] [--apply]

The arm still visits EVERY row of the joint file (the planned path is
unchanged), but a dropped work point gets `is_task_waypoint` 0, so the loader
marks it `scan: False`: MoveJ through it, no standoff loop, no capture, no Ra
map row (task_manager._is_scan_row). `waypoint_kind` stays `task` and
`source_point_id` stays, so the row is still recognisable as a surface point
(and `slow_task_entry.py`'s transition -> task rule does not fire on it);
speeds are untouched.

Which points: the work points lie on raster lines of constant world y
(10 mm apart, 10 mm pitch along the line — read from the pose twin). With
`--keep 2` every line keeps every second point, and the phase of each line
segment (one group's run of points on one y line) is chosen so its kept points
fall BETWEEN the previous line's kept points — a staggered (checkerboard)
pattern, nearest kept neighbour ~14 mm instead of 10, no 20 mm stripes.
`--keep N` keeps every N-th point with the same stagger rule.

Text-level edit: only the is_task_waypoint cell of the dropped rows changes;
BOM, line endings and every other cell stay byte-identical. Dry run by
default; `--apply` rewrites JOINT_CSV in place (back it up first). The pose
file is only read.
"""
import argparse
import collections
import sys


def load(path):
    raw = open(path, 'rb').read()
    bom = raw[:3] == b'\xef\xbb\xbf'
    text = raw[3:].decode('utf-8') if bom else raw.decode('utf-8')
    eol = '\r\n' if '\r\n' in text else '\n'
    assert '"' not in text, '%s: quoted cells are not handled' % path
    tail = text.endswith(eol)
    lines = (text[:-len(eol)] if tail else text).split(eol)
    rows = [l.split(',') for l in lines]
    assert all(len(r) == len(rows[0]) for r in rows), path
    return dict(bom=bom, eol=eol, tail=tail, hdr=rows[0], rows=rows[1:])


def save(path, f):
    text = f['eol'].join(','.join(r) for r in [f['hdr']] + f['rows'])
    if f['tail']:
        text += f['eol']
    with open(path, 'wb') as fp:
        fp.write((b'\xef\xbb\xbf' if f['bom'] else b'') + text.encode('utf-8'))


def is_scan(cell):
    return cell.strip() not in ('0', '0.0', 'False', 'false')


def choose(points, keep):
    """points: list of (row_index, gid, x_mm, y_mm). Returns the kept row set."""
    lines = collections.defaultdict(list)
    for p in points:
        lines[round(p[3], 1)].append(p)
    kept = set()
    prev_x = []                      # kept x of the previous y line
    prev_y = None
    for y in sorted(lines):
        line = sorted(lines[y], key=lambda p: p[2])
        # segments: one group's contiguous run on this line
        segs, cur = [], [line[0]]
        for a, b in zip(line, line[1:]):
            if b[1] != a[1] or b[2] - a[2] > 25.0:
                segs.append(cur); cur = []
            cur.append(b)
        segs.append(cur)
        near = prev_y is not None and abs(y - prev_y) < 15.0
        this_x = []
        for seg in segs:
            best, best_score = 0, None
            for phase in range(keep):
                sel = [p for i, p in enumerate(seg) if i % keep == phase]
                if not sel:
                    continue
                if near and prev_x:
                    score = sum(min(abs(p[2] - q) for q in prev_x) for p in sel) / len(sel)
                else:
                    score = 0.0
                # larger distance to the previous line's kept points wins;
                # ties -> more points kept, then the lower phase
                key = (round(score, 3), len(sel), -phase)
                if best_score is None or key > best_score:
                    best, best_score = phase, key
            for i, p in enumerate(seg):
                if i % keep == best:
                    kept.add(p[0])
                    this_x.append(p[2])
        prev_x, prev_y = this_x, y
    return kept


def nn_stats(xy):
    import math
    out = []
    for i, a in enumerate(xy):
        best = min((math.dist(a, b) for j, b in enumerate(xy)
                    if j != i and abs(a[0] - b[0]) < 40 and abs(a[1] - b[1]) < 40),
                   default=None)
        if best is not None:
            out.append(best)
    out.sort()
    return out[len(out) // 2], out[int(len(out) * 0.95)], out[-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('joint')
    ap.add_argument('pose')
    ap.add_argument('--keep', type=int, default=2, help='keep every N-th work point (2 = half)')
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    assert a.keep >= 2

    J, P = load(a.joint), load(a.pose)
    jh, ph = J['hdr'], P['hdr']
    ig, isrc, itw, ik = (jh.index(c) for c in
                         ('group_id', 'source_point_id', 'is_task_waypoint', 'waypoint_kind'))
    pg, pp, px, py = (ph.index(c) for c in ('group_id', 'point_id', 'x', 'y'))
    pose = {(int(float(r[pg])), int(float(r[pp]))): (float(r[px]) * 1000, float(r[py]) * 1000)
            for r in P['rows']}

    points = []
    for i, r in enumerate(J['rows']):
        if not is_scan(r[itw]):
            continue
        assert r[ik] == 'task', 'row %d: scanned but waypoint_kind %s' % (i + 2, r[ik])
        key = (int(float(r[ig])), int(float(r[isrc])))
        if key not in pose:
            sys.exit('row %d: work point %s has no pose twin' % (i + 2, key))
        points.append((i, key[0]) + pose[key])
    kept = choose(points, a.keep)

    by_g = collections.Counter(p[1] for p in points)
    kept_g = collections.Counter(p[1] for p in points if p[0] in kept)
    print('%s\n  work points %d -> %d scanned (%d driven through)'
          % (a.joint, len(points), len(kept), len(points) - len(kept)))
    for g in sorted(by_g):
        print('    group %d: %d -> %d' % (g, by_g[g], kept_g[g]))
    for name, sel in (('before', points), ('after', [p for p in points if p[0] in kept])):
        med, p95, mx = nn_stats([p[2:] for p in sel])
        print('  nearest scanned neighbour %-6s median %.1f mm, p95 %.1f, max %.1f'
              % (name, med, p95, mx))

    for p in points:
        if p[0] not in kept:
            J['rows'][p[0]][itw] = '0'
    if a.apply:
        save(a.joint, J)
        print('  written')
    else:
        print('  dry run (--apply to write)')


if __name__ == '__main__':
    main()
