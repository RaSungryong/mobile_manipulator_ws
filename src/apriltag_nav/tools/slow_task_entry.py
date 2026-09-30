#!/usr/bin/env python3
"""Set speed 10 on both rows of every transition -> task boundary.

rrt_final_path_<key>.csv : the last `transition` row and the `task` row that
                           follows it both get speed 10.
assigned_workpoints_<key>.csv : has no waypoint_kind, so the work point paired
                           with that task row (group_id, source_point_id ==
                           point_id) gets the same speed.
Text-level edit: only the speed cell changes; BOM, CRLF and every other cell
stay byte-identical.
"""
import glob, os, sys

SLOW = '10'
d = sys.argv[1]
apply = '--apply' in sys.argv


def load(path):
    raw = open(path, 'rb').read()
    assert raw[:3] == b'\xef\xbb\xbf', path
    text = raw[3:].decode('utf-8')
    assert text.endswith('\r\n') and '"' not in text, path
    rows = [l.split(',') for l in text[:-2].split('\r\n')]
    hdr = rows[0]
    assert all(len(r) == len(hdr) for r in rows), path
    return hdr, rows[1:]


def save(path, hdr, rows):
    text = '\r\n'.join(','.join(r) for r in [hdr] + rows) + '\r\n'
    with open(path, 'wb') as f:
        f.write(b'\xef\xbb\xbf' + text.encode('utf-8'))


for rrt in sorted(glob.glob(os.path.join(d, 'rrt_final_path_*.csv'))):
    key = os.path.basename(rrt)[len('rrt_final_path_'):]
    hdr, rows = load(rrt)
    ik, isp, ig, isrc = (hdr.index(c) for c in
                         ('waypoint_kind', 'speed', 'group_id', 'source_point_id'))
    n_bound = n_tr = n_task = 0
    slow_points = set()
    for i in range(1, len(rows)):
        if rows[i - 1][ik] == 'transition' and rows[i][ik] == 'task':
            assert rows[i - 1][ig] == rows[i][ig], (rrt, i)
            n_bound += 1
            if rows[i - 1][isp] != SLOW:
                rows[i - 1][isp] = SLOW; n_tr += 1
            if rows[i][isp] != SLOW:
                rows[i][isp] = SLOW; n_task += 1
            slow_points.add((rows[i][ig], str(int(float(rows[i][isrc])))))
    print('%s\n  boundaries %d: transition rows changed %d, task rows changed %d'
          % (os.path.basename(rrt), n_bound, n_tr, n_task))
    if apply:
        save(rrt, hdr, rows)

    pose = os.path.join(d, 'assigned_workpoints_' + key)
    if not os.path.exists(pose):
        print('  (no paired assigned_workpoints file)')
        continue
    ph, prows = load(pose)
    pg, pp, ps = (ph.index(c) for c in ('group_id', 'point_id', 'speed'))
    n_pose = 0; seen = set()
    for r in prows:
        k = (r[pg], r[pp])
        if k in slow_points:
            seen.add(k)
            if r[ps] != SLOW:
                r[ps] = SLOW; n_pose += 1
    assert seen == slow_points, (pose, len(slow_points - seen))
    print('%s\n  paired work points %d: changed %d'
          % (os.path.basename(pose), len(slow_points), n_pose))
    if apply:
        save(pose, ph, prows)
