#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Write hand-measured Ra from an operator's spreadsheet into a run's
`<run>_ra_measured.csv` (2026-10-14).

The operator scans in COLLECT mode (method B), releases every group entry
stop with Skip all (so each captured point has a row with `skipped`
True or a PENDING row), measures the marked spots with the hand-held
tester afterwards and types the values into a spreadsheet:

    측정구역 | 번호 | 조도값          one row per captured point
    1        | 1    | 0.281           측정구역 = the group's ordinal (--zones maps
    1        | 2    | 0.382           it to the group_id), 번호 = source_point_id
    2        | 274  |                 (the planner's work-point number, = the
    2        | 275  | x               `_sp` token of the frame name), 조도값 = Ra;
                                      blank or `x` = not usable -> skipped

This tool rewrites the run's measured CSV in place (backup beside it):
a numeric 조도값 becomes a RECORDED row (`ra_measured` / `ra_readings`
to 4 decimals, `skipped` False, `measured_at` now, `note` names the
sheet); blank / `x` / no sheet row at all becomes (stays) a SKIPPED row
with the reason in `note`. Rows already carrying a Ra are left alone
unless --overwrite. Nothing else in the row changes. Dry run by default.

  python3 tools/import_ra_xlsx.py "docs/금형 데이터(2026.10.13).xlsx" \\
      scan_joint_HOOD-OTR_LWR-DIE_plate1_offset50mm_roughing_ra_map_20261013-1014 \\
      --zones 1:105,2:106,3:107,4:118,5:119 [--set 119:450=0.863] [--apply]

`--set <group>:<sp>=<ra>` overrides one sheet cell (a typo the operator
confirmed), recorded in the row's note. A sheet row whose (group, sp) has
no CSV row is reported and refuses the import (--ignore-extra to drop
them). The xlsx is read with the standard library only (no openpyxl on
the robot PC): sheet 1, header row looked up by name (측정구역 / 번호 /
조도값), falling back to columns A / B / C.

After it: `tools/merge_ra_dataset.py <run>` builds the per-frame dataset.
"""
import argparse
import csv
import os
import re
import shutil
import sys
import time
import zipfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))
from apriltag_nav import paths as _paths   # noqa: E402

NS = '{http://schemas.openxmlformats.org/spreadsheetml/2006/main}'
HEADERS = {'zone': '측정구역', 'number': '번호', 'ra': '조도값'}
SKIP_MARKS = ('x', 'X', '×')


def _col_letters(ref):
    return re.match(r'[A-Z]+', ref).group(0)


def read_xlsx_rows(path):
    """Sheet 1 of an xlsx as a list of {column letter: cell text}, 1-based
    row numbers kept as '_row'. Shared strings resolved; numbers as the
    stored text (e.g. '0.28100000000000003')."""
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        sst = []
        if 'xl/sharedStrings.xml' in names:
            root = ET.fromstring(z.read('xl/sharedStrings.xml'))
            for si in root.findall(NS + 'si'):
                sst.append(''.join(t.text or '' for t in si.iter(NS + 't')))
        sheet = 'xl/worksheets/sheet1.xml'
        if sheet not in names:
            cands = sorted(n for n in names if n.startswith('xl/worksheets/sheet'))
            if not cands:
                raise SystemExit(f'{path}: no worksheet found')
            sheet = cands[0]
        root = ET.fromstring(z.read(sheet))
    rows = []
    for row in root.iter(NS + 'row'):
        d = {'_row': int(row.get('r'))}
        for c in row.findall(NS + 'c'):
            v = c.find(NS + 'v')
            if v is None:
                t = c.find(NS + 'is')
                d[_col_letters(c.get('r'))] = ''.join(x.text or '' for x in t.iter(NS + 't')) if t is not None else ''
                continue
            if c.get('t') == 's':
                d[_col_letters(c.get('r'))] = sst[int(v.text)]
            else:
                d[_col_letters(c.get('r'))] = (v.text or '').strip()
        rows.append(d)
    return rows


def parse_sheet(rows):
    """-> ({(zone:int, number:int): ra_text}, header_info). ra_text is the
    cell text: '' (blank), 'x', or a number string."""
    if not rows:
        raise SystemExit('empty sheet')
    cols = {'zone': 'A', 'number': 'B', 'ra': 'C'}
    head = rows[0]
    found = {}
    for col, text in head.items():
        if col == '_row':
            continue
        for key, name in HEADERS.items():
            if str(text).strip() == name:
                found[key] = col
    if len(found) == 3:
        cols = found
        body = rows[1:]
    elif found:
        raise SystemExit(f'header row has {found} but not all of {list(HEADERS.values())}')
    else:
        body = rows   # no header: A / B / C
    cells = {}
    dup = []
    for r in body:
        z = str(r.get(cols['zone'], '')).strip()
        n = str(r.get(cols['number'], '')).strip()
        ra = str(r.get(cols['ra'], '')).strip()
        if z == '' and n == '' and ra == '':
            continue
        if not re.match(r'^\d+(\.0+)?$', z) or not re.match(r'^\d+(\.0+)?$', n):
            print(f'  sheet row {r["_row"]}: zone {z!r} / number {n!r} not numeric -> ignored'
                  f' (ra {ra!r})')
            continue
        key = (int(float(z)), int(float(n)))
        if key in cells:
            dup.append((r['_row'], key))
        cells[key] = ra
    if dup:
        raise SystemExit(f'duplicate (zone, number) in the sheet: {dup[:10]}')
    return cells, cols


def parse_zones(text):
    zones = {}
    for tok in re.split(r'[,; ]+', text.strip()):
        if not tok:
            continue
        m = re.match(r'^(\d+):(\d+)$', tok)
        if not m:
            raise SystemExit(f'--zones token {tok!r}: want <zone>:<group_id>')
        zones[int(m.group(1))] = int(m.group(2))
    if not zones:
        raise SystemExit('--zones is empty')
    if len(set(zones.values())) != len(zones):
        raise SystemExit('--zones maps two zones to one group')
    return zones


def parse_sets(items):
    out = {}
    for s in items or []:
        m = re.match(r'^(\d+):(\d+)=([-+]?\d*\.?\d+)$', s)
        if not m:
            raise SystemExit(f'--set {s!r}: want <group>:<sp>=<ra>')
        out[(int(m.group(1)), int(m.group(2)))] = float(m.group(3))
    return out


def _truthy(v):
    return str(v or '').strip().lower() in ('1', 'true', 'yes', 'y')


def plan_rows(csv_rows, cells, zones, sets, sheet_label, overwrite):
    """Decide the new value of every CSV row. Returns (new_rows, stats,
    problems)."""
    by_group = {g: z for z, g in zones.items()}
    stamp = time.strftime('%Y-%m-%d %H:%M:%S')
    stats = {'recorded': 0, 'skip_x': 0, 'skip_blank': 0, 'skip_no_entry': 0,
             'kept_recorded': 0, 'overwritten': 0, 'no_sp': 0, 'group_not_mapped': 0,
             'set_applied': 0}
    problems = []
    seen = set()
    new_rows = []
    for r in csv_rows:
        r = dict(r)
        try:
            g = int(float(r.get('group_id', '')))
        except ValueError:
            problems.append(f'CSV row with a bad group_id {r.get("group_id")!r}')
            new_rows.append(r)
            continue
        sp_txt = str(r.get('source_point_id', '') or '').strip()
        if sp_txt == '':
            stats['no_sp'] += 1
            new_rows.append(r)
            continue
        sp = int(float(sp_txt))
        if g not in by_group:
            stats['group_not_mapped'] += 1
            new_rows.append(r)
            continue
        zone = by_group[g]
        key = (zone, sp)
        seen.add(key)
        had_ra = str(r.get('ra_measured', '') or '').strip() != ''
        if had_ra and not overwrite:
            stats['kept_recorded'] += 1
            new_rows.append(r)
            continue
        if (g, sp) in sets:
            ra_val = sets[(g, sp)]
            note = (f'{sheet_label} zone {zone} #{sp}: sheet says '
                    f'{cells.get(key, "")!r}, set to {ra_val:g} on the operator\'s word')
            stats['set_applied'] += 1
            kind = 'recorded'
        elif key not in cells:
            ra_val, note, kind = None, f'{sheet_label}: no entry for zone {zone} #{sp}', 'skip_no_entry'
        else:
            cell = cells[key]
            if cell == '':
                ra_val, note, kind = None, f'{sheet_label} zone {zone} #{sp}: blank (not usable)', 'skip_blank'
            elif cell in SKIP_MARKS:
                ra_val, note, kind = None, f'{sheet_label} zone {zone} #{sp}: x (not usable)', 'skip_x'
            else:
                try:
                    ra_val = float(cell)
                except ValueError:
                    problems.append(f'zone {zone} #{sp}: 조도값 {cell!r} is neither a number nor x')
                    new_rows.append(r)
                    continue
                note, kind = f'{sheet_label} zone {zone} #{sp}', 'recorded'
        if had_ra:
            stats['overwritten'] += 1
        stats[kind] += 1
        if ra_val is None:
            r['ra_measured'] = ''
            r['ra_readings'] = ''
            r['skipped'] = 'True'
        else:
            r['ra_measured'] = f'{ra_val:.4f}'
            r['ra_readings'] = f'{ra_val:.4f}'
            r['skipped'] = 'False'
        r['note'] = note
        r['measured_at'] = stamp
        new_rows.append(r)
    extra = sorted(k for k in cells if k not in seen)
    return new_rows, stats, problems, extra


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('xlsx')
    ap.add_argument('run', help='run stem (or a path to its _ra_measured.csv)')
    ap.add_argument('--zones', required=True, help='<zone>:<group_id>,… e.g. 1:105,2:106')
    ap.add_argument('--set', action='append', default=[], metavar='G:SP=RA',
                    help='override one cell (confirmed typo); recorded in the note')
    ap.add_argument('--overwrite', action='store_true', help='replace rows that already carry a Ra')
    ap.add_argument('--ignore-extra', action='store_true',
                    help='drop sheet rows whose (group, sp) has no CSV row instead of refusing')
    ap.add_argument('--measured-dir', default=_paths.RA_MEASURED_DIR)
    ap.add_argument('--apply', action='store_true', help='write (default: dry run)')
    args = ap.parse_args(argv)

    zones = parse_zones(args.zones)
    sets = parse_sets(args.set)
    for (g, sp) in sets:
        if g not in zones.values():
            raise SystemExit(f'--set {g}:{sp}: group {g} is not in --zones')

    if args.run.endswith('.csv'):
        csv_path = args.run
    else:
        csv_path = _paths.ra_measured_path(os.path.basename(args.run.rstrip('/')), root=args.measured_dir)
    if not os.path.isfile(csv_path):
        raise SystemExit(f'no measured CSV: {csv_path}')

    rows = read_xlsx_rows(args.xlsx)
    cells, cols = parse_sheet(rows)
    sheet_label = f'xlsx {os.path.basename(args.xlsx)}'
    per_zone = {}
    for (z, n), ra in cells.items():
        d = per_zone.setdefault(z, {'n': 0, 'num': 0, 'blank': 0, 'x': 0, 'other': 0, 'min': None, 'max': None})
        d['n'] += 1
        if ra == '':
            d['blank'] += 1
        elif ra in SKIP_MARKS:
            d['x'] += 1
        else:
            try:
                v = float(ra)
                d['num'] += 1
                d['min'] = v if d['min'] is None else min(d['min'], v)
                d['max'] = v if d['max'] is None else max(d['max'], v)
            except ValueError:
                d['other'] += 1
    print(f'{args.xlsx}: columns {cols}, {len(cells)} data rows')
    for z in sorted(per_zone):
        d = per_zone[z]
        nums = sorted(n for (zz, n) in cells if zz == z)
        print(f'  zone {z} -> group {zones.get(z, "UNMAPPED")}: {d["n"]} rows, numbers {nums[0]}..{nums[-1]}'
              f' ({"no gaps" if nums[-1] - nums[0] + 1 == len(nums) else "GAPS"}),'
              f' {d["num"]} Ra ({d["min"]}..{d["max"]}), {d["blank"]} blank, {d["x"]} x'
              + (f', {d["other"]} OTHER' if d['other'] else ''))
    unmapped = sorted(z for z in per_zone if z not in zones)
    if unmapped:
        raise SystemExit(f'sheet zones {unmapped} are not in --zones')

    with open(csv_path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        fieldnames = list(reader.fieldnames or [])
        csv_rows = list(reader)
    need = {'group_id', 'source_point_id', 'ra_measured', 'ra_readings', 'note', 'skipped', 'measured_at'}
    if not need <= set(fieldnames):
        raise SystemExit(f'{csv_path}: missing columns {sorted(need - set(fieldnames))}')
    before_pending = sum(1 for r in csv_rows if str(r.get('ra_measured', '')).strip() == '' and not _truthy(r.get('skipped')))
    print(f'{csv_path}: {len(csv_rows)} rows, {before_pending} pending, '
          f'{sum(1 for r in csv_rows if str(r.get("ra_measured", "")).strip() != "")} with a Ra')

    new_rows, stats, problems, extra = plan_rows(csv_rows, cells, zones, sets, sheet_label, args.overwrite)
    print('plan:', ', '.join(f'{k} {v}' for k, v in stats.items() if v))
    for g in sorted(set(int(float(r['group_id'])) for r in new_rows)):
        rs = [r for r in new_rows if int(float(r['group_id'])) == g]
        rec = sum(1 for r in rs if str(r['ra_measured']).strip() != '')
        pend = sum(1 for r in rs if str(r['ra_measured']).strip() == '' and not _truthy(r['skipped']))
        print(f'  group {g}: {len(rs)} rows -> {rec} recorded, {len(rs) - rec - pend} skipped, {pend} pending')
    if extra:
        print(f'  {len(extra)} sheet rows have no CSV row (zone, number): {extra[:20]}')
        if not args.ignore_extra:
            problems.append(f'{len(extra)} sheet rows without a CSV row (--ignore-extra to drop them)')
    if problems:
        print('REFUSED:')
        for p in problems:
            print('  -', p)
        return 2
    if not args.apply:
        print('dry run — nothing written (--apply)')
        return 0

    bak = csv_path[:-4] + f'.before_xlsx_{time.strftime("%Y%m%d_%H%M%S")}.csv.bak'
    shutil.copy2(csv_path, bak)
    tmp = csv_path + '.tmp'
    with open(tmp, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        w.writeheader()
        for r in new_rows:
            w.writerow({c: r.get(c, '') for c in fieldnames})
    os.replace(tmp, csv_path)
    print(f'written {csv_path} (backup {bak})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
