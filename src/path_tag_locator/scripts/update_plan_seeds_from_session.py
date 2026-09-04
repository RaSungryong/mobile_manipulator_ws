#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
update_plan_seeds_from_session.py — rewrite a calibration plan's
``arm_view_tcp_mm_deg`` seeds from a recorded session.

    rosrun path_tag_locator update_plan_seeds_from_session.py \
        ~/.ros/path_tag_locator/calibrate/20260904_153218 \
        $(find path_tag_locator)/config/calibration_plan_plate1.yaml [--apply]

For every plan entry:
  * an entry that SUCCEEDED in the session gets the x, y the align loop
    actually converged to (``tcp_pose_mm_deg`` of its last ok attempt) —
    the best possible seed for the next session, since the base stops on
    the same tag within a centimetre;
  * an entry that FAILED (or was not run) keeps its design seed shifted by
    the MEDIAN (aligned - seed) x, y of the successful entries that share
    its ref tag (any ref tag as fallback) — i.e. its initial pose is
    estimated from the tags that were measured.

z and the orientation are KEPT from the plan on purpose: z is the design
view height (0.5 m above the tag), and the recorded z is not — the align
loop keeps whatever depth the initial move ended at (``target_distance_m
0``), so an entry whose initial move was clamped short converged at
~0.63 m, 80 mm high. Descending along the tag normal does not move the
camera off the tag, so measured x, y at design z is the right seed.

Only the ``arm_view_tcp_mm_deg:`` lines are touched; every comment and
the rest of the file survive (text-level edit, not a YAML re-dump), and a
header line records which session the seeds came from. Re-running
``generate_calibration_artifacts.py`` resets them to design seeds.

Default is a dry run that prints the table; ``--apply`` writes the file
(``--out`` to write elsewhere).
"""

import argparse
import glob
import os
import re
import sys
from datetime import datetime

import numpy as np
import yaml


def load_session(session_dir):
    """{path_tag_id: {'status', 'seed', 'final', 'ref', 'attempt'}} from the
    LAST attempt of each entry in the session's entries/ directory."""
    out = {}
    for f in sorted(glob.glob(os.path.join(session_dir, 'entries', '*.yaml'))):
        with open(f) as fh:
            e = yaml.safe_load(fh)
        tid = int(e['path_tag_id'])
        prev = out.get(tid)
        if prev is not None and prev['attempt'] > int(e.get('attempt', 1)):
            continue
        final = e.get('tcp_pose_mm_deg') or (e.get('auto_align') or {}).get('final_tcp')
        seed = (e.get('plan_entry') or {}).get('arm_view_tcp_mm_deg') or e.get('view_tcp_mm_deg')
        out[tid] = {
            'status': e.get('status'),
            'attempt': int(e.get('attempt', 1)),
            'ref': int(e['ref_tag_id']),
            'seed': [float(v) for v in seed] if seed else None,
            'final': [float(v) for v in final] if final else None,
            'source': e.get('view_tcp_source'),
        }
    return out


def corrections_by_ref(session):
    """{ref: [delta_xyz_mm, ...]} over successful entries that started from
    the plan seed."""
    corr = {}
    for tid, r in session.items():
        if r['status'] != 'ok' or not r['final'] or not r['seed']:
            continue
        if r['source'] not in (None, 'entry-override'):
            continue      # a raised / corrected retry would double-count
        d = np.array(r['final'][:2]) - np.array(r['seed'][:2])
        corr.setdefault(r['ref'], []).append(d)
    return corr


def median_correction(corr, ref):
    same = corr.get(ref) or []
    pool = same if same else [d for ds in corr.values() for d in ds]
    if not pool:
        return None, 0, False
    return np.median(np.array(pool), axis=0), len(pool), bool(same)


def fmt(v):
    return '[' + ', '.join(f'{x:.1f}' for x in v) + ']'


def rewrite_plan(text, plan_entries, session, corr, session_name):
    """Replace each entry's arm_view_tcp_mm_deg line. Returns (text, rows)."""
    rows = []
    lines = text.split('\n')
    cur_tag = None
    for i, line in enumerate(lines):
        m = re.match(r'^(\s*)-\s*path_tag_id:\s*(\d+)', line)
        if m:
            cur_tag = int(m.group(2))
            continue
        m = re.match(r'^(\s*)arm_view_tcp_mm_deg:\s*\[([^\]]*)\](.*)$', line)
        if not m or cur_tag is None:
            continue
        indent = m.group(1)
        old = [float(v) for v in m.group(2).split(',')]
        r = session.get(cur_tag)
        if r and r['status'] == 'ok' and r['final']:
            new = list(old)
            new[0], new[1] = float(r['final'][0]), float(r['final'][1])
            how = f'measured x,y {session_name} (z/orientation: design)'
        else:
            ref = r['ref'] if r else next(
                (e['ref_tag_id'] for e in plan_entries if e['path_tag_id'] == cur_tag), None)
            c, n, same = median_correction(corr, ref)
            if c is None:
                rows.append((cur_tag, 'unchanged (no data)', old, old))
                continue
            new = list(old)
            for k in range(2):
                new[k] = old[k] + float(c[k])
            how = (f'design + median Δxy{fmt(c)} of ref {ref} '
                   f'({"same" if same else "any"} ref, n={n}) {session_name}')
        lines[i] = f'{indent}arm_view_tcp_mm_deg: {fmt(new)}    # seed: {how}'
        rows.append((cur_tag, how, old, new))
    return '\n'.join(lines), rows


def add_header(text, session_name):
    note = (f'# Seeds UPDATED from session {session_name} on '
            f'{datetime.now():%Y-%m-%d %H:%M} by '
            'scripts/update_plan_seeds_from_session.py (per-line "# seed:" '
            'comments say how). Re-running the generator resets them.')
    lines = text.split('\n')
    # replace an earlier update note, else insert before the first blank line
    for i, l in enumerate(lines):
        if l.startswith('# Seeds UPDATED from session'):
            lines[i] = note
            return '\n'.join(lines)
    for i, l in enumerate(lines):
        if not l.strip():
            lines.insert(i, note)
            break
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session_dir')
    ap.add_argument('plan_yaml')
    ap.add_argument('--apply', action='store_true', help='write the plan')
    ap.add_argument('--out', default=None, help='write here instead of in place')
    args = ap.parse_args()

    session = load_session(os.path.expanduser(args.session_dir))
    if not session:
        sys.exit(f'no entries under {args.session_dir}/entries')
    with open(args.plan_yaml) as fh:
        text = fh.read()
    plan_entries = yaml.safe_load(text)['plan']
    corr = corrections_by_ref(session)
    name = os.path.basename(os.path.normpath(args.session_dir))
    new_text, rows = rewrite_plan(text, plan_entries, session, corr, name)
    new_text = add_header(new_text, name)

    print(f'session {name}: {sum(1 for r in session.values() if r["status"]=="ok")} ok, '
          f'{sum(1 for r in session.values() if r["status"]!="ok")} failed')
    for ref, ds in sorted(corr.items()):
        med = np.median(np.array(ds), axis=0)
        print(f'  ref {ref}: n={len(ds)} median Δxy(aligned-seed) = {fmt(med)} mm')
    print(f'{"tag":>4}  {"old seed (x,y)":>18}  {"new seed (x,y)":>18}  {"Δ":>6}  how')
    for tag, how, old, new in rows:
        d = float(np.linalg.norm(np.array(new[:2]) - np.array(old[:2])))
        print(f'{tag:>4}  {fmt(old[:2]):>18}  {fmt(new[:2]):>18}  {d:5.1f}  {how}')
    if not args.apply:
        print('\n(dry run — add --apply to write)')
        return
    out = args.out or args.plan_yaml
    yaml.safe_load(new_text)          # must still parse
    with open(out, 'w') as fh:
        fh.write(new_text)
    print(f'\nwritten: {out}')


if __name__ == '__main__':
    main()
