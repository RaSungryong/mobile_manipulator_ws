#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Join one scan run's three outputs into a training-set table (2026-10-06).

A TASK scan in Ra COLLECT mode (robot.yaml `collect:`, robot_ui Task tab)
leaves, all keyed by the run stem `<task>_ra_map_<ts>`:

  log/apriltag_nav/ra_maps/<run>.csv                 the Ra map: per work point
                                                      x y z, the MODEL's Ra,
                                                      the standoff outcome
  results/scan_images/<run>/<run>_ra_measured.csv    the HAND-MEASURED Ra the
                                                      operator typed per point
                                                      (log/apriltag_nav/ra_measured/
                                                      before 2026-10-06 evening)
  results/scan_images/<run>/g<g>_p<p>_i<i>_s<n>.png   the frames

This writes results/ra_dataset/<run>_dataset.csv with ONE ROW PER FRAME:
run, group_id, point_id, index, sample, image (absolute path), ra_measured,
ra_readings, note, model_ra_mean, standoff_ok, standoff, x, y, z,
measured_at — and prints what is missing (frames with no measurement,
measurements with no frame, points whose standoff did not converge).

  python3 tools/merge_ra_dataset.py                      # every run that has a measured CSV
  python3 tools/merge_ra_dataset.py <run stem or path>   # one run
  python3 tools/merge_ra_dataset.py --all-frames         # keep unmeasured frames too (blank ra_measured)
  python3 tools/merge_ra_dataset.py --out dataset.csv    # one combined table

Legacy frames (`point_<id>_sample_<n>_ra_<x>.png`, before 2026-10-06) carry
no group, so they are paired by point_id only and flagged `ambiguous` when
that id occurs in more than one group of the run.

Method B (mark pass): the measurements live in a FILLED-IN
`<mark run>_mark_template.csv` (same columns, `mark_no` first), not in a
`<scan run>_ra_measured.csv` — pass it with `--measured <file>`; without
it, a scan run that has no measured CSV of its own takes the newest
template of the SAME TASK (`<task>_ra_map_*_mark_template.csv`) that has
at least one `ra_measured` filled in, and says which.
"""
import argparse
import glob
import os
import re
import sys

import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))
from apriltag_nav import paths as _paths   # noqa: E402

NEW_RE = re.compile(r'^g(?P<g>-?\d+)_p(?P<p>\d+)_i(?P<i>\d+)_s(?P<s>\d+)\.png$')
OLD_RE = re.compile(r'^point_(?P<p>\d+)_sample_(?P<s>\d+)(?:_ra_[-\d.]+)?\.png$')

COLUMNS = ['run', 'group_id', 'point_id', 'index', 'sample', 'image',
           'ra_measured', 'ra_readings', 'note', 'model_ra_mean',
           'standoff_ok', 'standoff', 'x', 'y', 'z', 'measured_at', 'flags']


def _truthy(v):
    """CSV cell -> bool: 'True' / 'true' / '1' are True; blank / NaN / 'False' not."""
    if v is None or (isinstance(v, float) and pd.isna(v)):
        return False
    return str(v).strip().lower() in ('true', '1', 'yes')


def run_stem(arg):
    base = os.path.basename(arg.rstrip('/'))
    for suf in ('_ra_measured.csv', '_dataset.csv', '.csv'):
        if base.endswith(suf):
            return base[:-len(suf)]
    return base


def list_frames(image_dir):
    """[{group_id|None, point_id, index|None, sample, image}] for the run."""
    out = []
    if not os.path.isdir(image_dir):
        return out
    for name in sorted(os.listdir(image_dir)):
        m = NEW_RE.match(name)
        if m:
            out.append({'group_id': int(m['g']), 'point_id': int(m['p']),
                        'index': int(m['i']), 'sample': int(m['s']),
                        'image': os.path.join(image_dir, name)})
            continue
        m = OLD_RE.match(name)
        if m:
            out.append({'group_id': None, 'point_id': int(m['p']), 'index': None,
                        'sample': int(m['s']), 'image': os.path.join(image_dir, name)})
    return out


def task_of(stem):
    """`<task>` of a `<task>_ra_map_<ts>` stem (the stem itself otherwise)."""
    m = re.match(r'^(?P<task>.+)_ra_map_\d{8}_\d{6}$', stem)
    return m['task'] if m else stem


def find_measured(stem, measured_dir, override=None):
    """The measured CSV for a scan run: the override, else the run's own
    `_ra_measured.csv`, else the newest filled-in mark template of the same
    task. Returns (path or None, how)."""
    if override:
        return override, 'given'
    own = _paths.ra_measured_path(stem, root=measured_dir)
    if os.path.exists(own):
        return own, 'own'
    task = task_of(stem)
    cands = sorted(glob.glob(os.path.join(measured_dir, f'{task}_ra_map_*',
                                          f'{task}_ra_map_*_mark_template.csv')),
                   reverse=True)
    for c in cands:
        try:
            t = pd.read_csv(c)
        except Exception:
            continue
        if 'ra_measured' in t.columns and t['ra_measured'].notna().any():
            return c, 'mark template'
    return None, 'none'


def merge_run(stem, all_frames=False, ra_map_dir=None, measured_dir=None,
              image_root=None, measured_override=None):
    ra_map_dir = ra_map_dir or _paths.RA_MAP_DIR
    measured_dir = measured_dir or _paths.RA_MEASURED_DIR
    image_root = image_root or _paths.SCAN_IMAGE_DIR
    ra_map_path = os.path.join(ra_map_dir, stem + '.csv')
    measured_path, measured_how = find_measured(stem, measured_dir, measured_override)
    image_dir = os.path.join(image_root, stem)

    ra_map = pd.read_csv(ra_map_path) if os.path.exists(ra_map_path) else pd.DataFrame()
    measured = (pd.read_csv(measured_path, dtype={'note': str, 'ra_readings': str})
                if measured_path and os.path.exists(measured_path) else pd.DataFrame())
    frames = list_frames(image_dir)
    report = {'run': stem, 'ra_map': os.path.exists(ra_map_path),
              'measured_csv': bool(measured_path and os.path.exists(measured_path)),
              'measured_how': measured_how,
              'measured_path': measured_path or '', 'frames': len(frames),
              'measured_rows': len(measured), 'rows': 0, 'labelled': 0,
              'frames_unmeasured': 0, 'measured_without_frame': 0,
              'standoff_not_ok': 0, 'ambiguous': 0}

    # Ra map by (group, point); groups per point for the legacy pairing.
    ra_rows = {}
    groups_of = {}
    for _, r in ra_map.iterrows():
        try:
            key = (int(r['group_id']), int(r['point_id']))
        except (TypeError, ValueError):
            continue
        ra_rows[key] = r
        groups_of.setdefault(key[1], set()).add(key[0])

    # Measured rows by (group, point) — the LAST row wins when a point was
    # recorded twice (a re-run of the same point in one run).
    meas_rows = {}
    for _, r in measured.iterrows():
        try:
            key = (int(r['group_id']), int(r['point_id']))
        except (TypeError, ValueError):
            continue
        meas_rows[key] = r

    seen_meas = set()
    rows = []
    for f in frames:
        gid, pid = f['group_id'], f['point_id']
        flags = []
        if gid is None:                       # legacy name: pair by point id
            cands = sorted(groups_of.get(pid, set()) | {g for g, p in meas_rows if p == pid})
            if len(cands) == 1:
                gid = cands[0]
            elif len(cands) > 1:
                flags.append('ambiguous')
                report['ambiguous'] += 1
                gid = cands[0]
        key = (gid, pid)
        m = meas_rows.get(key)
        r = ra_rows.get(key)
        ra_measured = None
        if m is not None and not _truthy(m.get('skipped')) and pd.notna(m.get('ra_measured')):
            try:
                ra_measured = float(m['ra_measured'])
            except (TypeError, ValueError):
                ra_measured = None
        if m is not None:
            seen_meas.add(key)
        if ra_measured is None:
            report['frames_unmeasured'] += 1
            if not all_frames:
                continue
            flags.append('unmeasured')
        standoff = str(r['execution_message']) if r is not None and pd.notna(r.get('execution_message')) else ''
        standoff_ok = ('standoff ok' in standoff) if standoff else None
        if standoff and not standoff_ok:
            report['standoff_not_ok'] += 1
            flags.append('standoff_not_ok')
        rows.append({
            'run': stem, 'group_id': gid, 'point_id': pid,
            'index': f['index'] if f['index'] is not None else
                     (int(m['index']) if m is not None and pd.notna(m.get('index')) else None),
            'sample': f['sample'], 'image': f['image'],
            'ra_measured': ra_measured,
            'ra_readings': (m['ra_readings'] if m is not None and pd.notna(m.get('ra_readings')) else ''),
            'note': (m['note'] if m is not None and pd.notna(m.get('note')) else ''),
            'model_ra_mean': (float(r['ra_mean']) if r is not None and pd.notna(r.get('ra_mean')) else None),
            'standoff_ok': standoff_ok, 'standoff': standoff,
            'x': r['x'] if r is not None else None,
            'y': r['y'] if r is not None else None,
            'z': r['z'] if r is not None else None,
            'measured_at': (m['measured_at'] if m is not None else ''),
            'flags': ' '.join(flags),
        })
        if ra_measured is not None:
            report['labelled'] += 1
    report['measured_without_frame'] = sum(
        1 for k, m in meas_rows.items()
        if k not in seen_meas and not _truthy(m.get('skipped'))
        and pd.notna(m.get('ra_measured')))
    report['rows'] = len(rows)
    return pd.DataFrame(rows, columns=COLUMNS), report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('runs', nargs='*', help='run stem(s) / paths; default: every run with a measured CSV')
    ap.add_argument('--all-frames', action='store_true', help='keep frames without a measurement (blank ra_measured)')
    ap.add_argument('--measured', help='the measured CSV to use for the given run (e.g. a filled-in mark template)')
    ap.add_argument('--out', help='one combined CSV instead of one per run')
    ap.add_argument('--out-dir', default=_paths.RA_DATASET_DIR)
    ap.add_argument('--ra-map-dir', default=_paths.RA_MAP_DIR)
    ap.add_argument('--measured-dir', default=_paths.RA_MEASURED_DIR,
                    help='root holding <run>/<run>_ra_measured.csv (default: the frame root)')
    ap.add_argument('--image-root', default=_paths.SCAN_IMAGE_DIR)
    args = ap.parse_args(argv)

    stems = [run_stem(r) for r in args.runs]
    if not stems:
        stems = sorted(run_stem(p) for p in glob.glob(os.path.join(args.measured_dir, '*', '*_ra_measured.csv')))
        if not stems:
            print(f'no <run>/<run>_ra_measured.csv under {args.measured_dir}')
            return 1
    tables = []
    for stem in stems:
        if args.measured and len(stems) != 1:
            print('--measured goes with exactly one run'); return 2
        df, rep = merge_run(stem, args.all_frames, args.ra_map_dir, args.measured_dir,
                            args.image_root, args.measured)
        print(f"{stem}: {rep['rows']} rows ({rep['labelled']} labelled) from {rep['frames']} frames, "
              f"{rep['measured_rows']} measured rows"
              + (f" ({rep['measured_how']}: {os.path.basename(rep['measured_path'])})"
                 if rep['measured_how'] in ('mark template', 'given') else '')
              + ('' if rep['ra_map'] else ' — NO Ra map')
              + ('' if rep['measured_csv'] else ' — NO measured CSV')
              + (f"; {rep['frames_unmeasured']} frames unmeasured" if rep['frames_unmeasured'] else '')
              + (f"; {rep['measured_without_frame']} measurements without a frame" if rep['measured_without_frame'] else '')
              + (f"; {rep['standoff_not_ok']} rows standoff NOT ok" if rep['standoff_not_ok'] else '')
              + (f"; {rep['ambiguous']} legacy frames ambiguous" if rep['ambiguous'] else ''))
        if not args.out:
            os.makedirs(args.out_dir, exist_ok=True)
            out = os.path.join(args.out_dir, stem + '_dataset.csv')
            df.to_csv(out, index=False)
            print(f'  -> {out}')
        tables.append(df)
    if args.out:
        df = pd.concat(tables, ignore_index=True) if tables else pd.DataFrame(columns=COLUMNS)
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or '.', exist_ok=True)
        df.to_csv(args.out, index=False)
        print(f'-> {args.out}: {len(df)} rows, {int(df["ra_measured"].notna().sum())} labelled')
    return 0


if __name__ == '__main__':
    sys.exit(main())
