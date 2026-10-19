#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tf_chain_tool.py — look at, check and update config/tf/tf_chain.yaml.

    python3 tools/tf_chain_tool.py show                 every transform, t / rpy, the
                                                        arm_calibration six numbers, the
                                                        URDF lines, vs design
    python3 tools/tf_chain_tool.py check                yaml vs npz, T_mb2fc vs robot.yaml,
                                                        the planner URDF, rigidity  (exit 1 on fail)
    python3 tools/tf_chain_tool.py front-cam [--apply]  regenerate T_mb2fc from robot.yaml
                                                        (after a front_cam re-fit / remount)
    python3 tools/tf_chain_tool.py set T_hc2ee --npz F --source "..."
    python3 tools/tf_chain_tool.py set T_ee2tip --t-mm -2.0 -245.2 214.4 --rpy-deg 0 0 0 --source "..."
                                                        write one block + its npz  [--kind calibrated|design]
    python3 tools/tf_chain_tool.py set T_mb2sc --new --kind calibrated --t-mm ... --source "..."
                                                        ADD a block (any T_x2y name; the loader picks
                                                        it up, NAMES stays the required core)
    python3 tools/tf_chain_tool.py use T_ee2tip measured_2026_09_22
                                                        re-apply one of a block's sub-block poses
                                                        (design / measured_<date> / ...) as the value
    python3 tools/tf_chain_tool.py export-npz           rewrite every npz from the yaml
    python3 tools/tf_chain_tool.py urdf [--apply]       the mobile_to_base / vision_tip_joint
                                                        lines the planner URDF must hold; --apply
                                                        rewrites them IN PLACE (everything else
                                                        byte-identical), then copy the URDF to
                                                        the planner PC yourself
    python3 tools/tf_chain_tool.py joint-offsets        the arm joint zero offsets in use
    python3 tools/tf_chain_tool.py joint-offsets --apply <session>/arm_offsets.npz --source "..."
                                                        write config/tf/arm_joint_offsets.yaml (+ npz)
                                                        from an arm_offsets.py result
    python3 tools/tf_chain_tool.py joint-offsets --disable | --enable

`set`, `use` and `front-cam --apply` rewrite the block's source / t_mm /
rpy / matrix (and `kind`) lines in place; comments and every sub-block
(`design`, `measured_<date>`, ...) stay. `source` is ONE line — what, when,
by which tool; the long story goes to CLAUDE.md's Work Log (the pre-2026-10-13
texts are in config/tf/HISTORY.md). Restart
arm_node (T_ab2mb, T_ee2tip) and the calibration nodes (all four) after a
change — they read the file at start.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src'))

from apriltag_nav import tf_chain as TC                     # noqa: E402
from apriltag_nav.paths import CONFIG_PATH, SRC_SPACE      # noqa: E402

URDF = os.path.join(SRC_SPACE, 'frcobot_ros', 'frcobot_description', 'urdf',
                    'fr10v6_mobile_visionsystem_final.urdf')


def _fmt(v, n=3):
    return '(' + ', '.join(('%%.%df' % n) % x for x in v) + ')'


def _design_T(entry):
    d = (entry or {}).get('design') or {}
    if d.get('t_mm') is None or d.get('rpy_deg_xyz') is None:
        return None
    return TC.matrix_from_pose(np.asarray(d['t_mm'], float) / 1000.0, d['rpy_deg_xyz'])


def _ang_deg(A, B):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(A[:3, :3].T @ B[:3, :3]) - 1) / 2))))


def _urdf_joint(name, urdf=None):
    return TC.read_urdf_joint(urdf or URDF, name)


def cmd_show(args):
    chain = TC.load_tf_chain(args.tf_yaml)
    raw = yaml.safe_load(open(args.tf_yaml))
    print('%s\n' % args.tf_yaml)
    for name in TC.block_names(args.tf_yaml):
        T = chain[name]
        print('%-9s %-11s %s' % (name, '[%s]' % raw[name].get('kind', '?'), TC.describe(T)))
        print('          source: %s' % raw[name].get('source', ''))
        alts = TC.alternatives(raw[name])
        if not alts:
            print('          design: none (%s)' % ((raw[name].get('design') or {}).get('note', '')))
        for key, (t, r) in alts.items():
            A = TC.matrix_from_pose(np.asarray(t) / 1000.0, r)
            dt = (T[:3, 3] - A[:3, 3]) * 1e3
            same = np.abs(dt).max() < 1e-6 and _ang_deg(T, A) < 1e-6
            print('          %-22s %s mm / rpy %s deg%s' % (key + ':', _fmt(t, 2), _fmt(r, 2),
                  '  = applied' if same else '  (applied - this: dt %s mm, %.3f deg)' % (_fmt(dt, 2), _ang_deg(T, A))))
    c = TC.arm_calibration_from_T_ab2mb(chain['T_ab2mb'])
    print('\narm_transform parametrisation of T_ab2mb (inv, body frame):')
    for k in ('arm_body_offset_x', 'arm_body_offset_y', 'arm_base_z'):
        print('  %-18s %+.6f m' % (k, c[k]))
    for k in ('arm_mount_yaw', 'arm_tilt_x', 'arm_tilt_y'):
        print('  %-18s %+.9f rad = %+.3f deg' % (k, c[k], math.degrees(c[k])))
    _print_urdf(chain, args.urdf)
    jo = os.path.join(os.path.dirname(args.tf_yaml), 'arm_joint_offsets.yaml')
    if os.path.exists(jo):
        e = TC.load_joint_offsets_entry(jo)
        print('\narm joint zero offsets (%s): J1..J6 = %s deg  (%s)'
              % ('enabled' if e.get('enabled', True) else 'DISABLED',
                 ', '.join('%+.3f' % v for v in TC.load_joint_offsets(jo, require_enabled=False)), e.get('source', '')))
    else:
        print('\narm joint zero offsets: none (dq = 0)')


def _print_urdf(chain, urdf=None):
    print('\nplanner URDF (%s) must hold:' % (urdf or URDF))
    for joint, line in TC.urdf_joint_lines(chain['T_ab2mb'], chain['T_ee2tip']).items():
        print('  <joint name="%s"> %s%s' % (joint, line, '  (+ vision_fixed at 0)' if joint == 'vision_tip_joint' else ''))


def cmd_urdf(args):
    chain = TC.load_tf_chain(args.tf_yaml)
    _print_urdf(chain, args.urdf)
    if not os.path.isfile(args.urdf):
        sys.exit('planner URDF not found: %s' % args.urdf)
    rows = TC.write_urdf_joints(args.urdf, chain['T_ab2mb'], chain['T_ee2tip'], dry_run=not args.apply)
    changed = [r for r in rows if r[3]]
    if not changed:
        print('-- %s already holds them' % args.urdf)
        return 0
    for joint, old, new, _ in changed:
        print('-- %s %s:\n     was %s\n     now %s' % ('rewrote' if args.apply else 'would rewrite', joint, old, new))
    if not args.apply:
        print('   run with --apply to rewrite them in place (nothing else in the file changes)')
    else:
        print('-- wrote %s — copy it to the planner PC and re-export the paths' % args.urdf)
    return 0


def cmd_check(args):
    bad = 0

    def check(ok, msg):
        nonlocal bad
        bad += (not ok)
        print('  [%s] %s' % ('ok ' if ok else 'FAIL', msg))

    chain = TC.load_tf_chain(args.tf_yaml)
    names = TC.block_names(args.tf_yaml)
    check(all(n in names for n in TC.NAMES), 'required blocks present: %s' % ', '.join(TC.NAMES))
    for name in names:
        try:
            TC.assert_rigid(chain[name], name)
            check(True, '%s rigid' % name)
        except ValueError as e:
            check(False, str(e))
    for name, ok, d in TC.check_npz_agree(args.tf_yaml):
        check(ok, '%s.npz == yaml block (%s)' % (name, ('%.1e' % d) if isinstance(d, float) else d))
    for name, ok, d in TC.check_readouts(args.tf_yaml):
        check(ok, '%s t_mm / rpy_deg_xyz readouts == matrix (%s)' % (name, d))
    raw0 = yaml.safe_load(open(args.tf_yaml))
    for name in names:
        k = raw0[name].get('kind')
        check(k in TC.KINDS, '%s kind: %s' % (name, k if k in TC.KINDS else '%r (must be %s)' % (k, ' | '.join(TC.KINDS))))
        src = str(raw0[name].get('source', ''))
        if len(src) > 240:          # the generated T_mb2fc line (front_cam_source_line) is ~225
            print('  [warn] %s source is %d chars — one line please, the story goes to the Work Log' % (name, len(src)))
    ok, d, params = TC.check_front_cam(args.tf_yaml, args.robot_yaml)
    check(ok, 'T_mb2fc == generator(robot.yaml) (max diff %.1e) — else `front-cam --apply`' % d)
    raw = yaml.safe_load(open(args.tf_yaml))
    D = _design_T(raw['T_ab2mb'])
    check(_ang_deg(chain['T_ab2mb'], D) < 3.0 and np.linalg.norm(chain['T_ab2mb'][:3, 3] - D[:3, 3]) < 0.05,
          'T_ab2mb within 3 deg / 50 mm of the design (%.2f deg, %.1f mm)'
          % (_ang_deg(chain['T_ab2mb'], D), 1e3 * np.linalg.norm(chain['T_ab2mb'][:3, 3] - D[:3, 3])))
    if os.path.isfile(args.urdf):
        xyz, rpy = TC.urdf_mobile_to_base(chain['T_ab2mb'])
        uxyz, urpy = _urdf_joint('mobile_to_base', args.urdf)
        check(np.abs(np.array(uxyz) - xyz).max() < 1e-5 and np.abs(np.array(urpy) - rpy).max() < 1e-6,
              'planner URDF mobile_to_base == T_ab2mb (urdf %s / %s)' % (_fmt(uxyz, 6), _fmt(urpy, 9)))
        txyz, _ = _urdf_joint('vision_tip_joint', args.urdf)
        fxyz, _ = _urdf_joint('vision_fixed', args.urdf)
        check(np.abs(np.array(txyz) + np.array(fxyz) - chain['T_ee2tip'][:3, 3]).max() < 5e-5,
              'planner URDF vision_fixed + vision_tip_joint == T_ee2tip (urdf %s m)' % _fmt(np.array(txyz) + np.array(fxyz), 4))
    else:
        print('  [skip] planner URDF not found: %s' % args.urdf)
    ok, why = TC.check_joint_offsets(os.path.join(os.path.dirname(args.tf_yaml), 'arm_joint_offsets.yaml'))
    check(ok, 'arm_joint_offsets.yaml: %s' % why)
    print('%d failed' % bad)
    return 1 if bad else 0


def cmd_joint_offsets(args):
    path = os.path.join(os.path.dirname(args.tf_yaml), 'arm_joint_offsets.yaml')
    if args.apply:
        d = np.load(args.apply)
        dq = np.asarray(d['dq_deg'], dtype=float).ravel()
        he = str(d['hand_eye']) if 'hand_eye' in d.files else ''
        urdf = str(d['urdf']) if 'urdf' in d.files else URDF
        TC.write_joint_offsets(dq, args.source, he, urdf, session=os.path.dirname(os.path.abspath(args.apply)),
                               enabled=not args.disable, note=args.note, path=path)
        print('-- wrote %s (+ npz)' % path)
    elif args.disable or args.enable:
        e = TC.load_joint_offsets_entry(path)
        TC.write_joint_offsets(e['dq_deg'], e.get('source', ''), e.get('hand_eye', ''), e.get('urdf', ''),
                               session=e.get('session', ''), enabled=bool(args.enable), note=e.get('note', ''), path=path)
        print('-- %s %s' % ('enabled' if args.enable else 'DISABLED', path))
    if not os.path.exists(path):
        print('no %s — dq = 0 (the controller\'s own FK)' % path)
        return 0
    e = TC.load_joint_offsets_entry(path)
    dq = TC.load_joint_offsets(path, require_enabled=False)
    print('arm joint zero offsets (%s): J1..J6 = %s deg' % ('enabled' if e.get('enabled', True) else 'DISABLED',
                                                          ', '.join('%+.3f' % v for v in dq)))
    for k in ('source', 'hand_eye', 'urdf', 'session', 'note'):
        if e.get(k):
            print('  %-9s %s' % (k, e[k]))
    return 0


def cmd_front_cam(args):
    cfg = yaml.safe_load(open(args.robot_yaml))
    T, params = TC.physical_T_mb2fc(cfg)
    src = TC.front_cam_source_line(params)
    print('T_mb2fc from %s:\n  %s\n  %s' % (args.robot_yaml, src, TC.describe(T)))
    stored = TC.load_transform('T_mb2fc', args.tf_yaml)
    d = float(np.abs(stored - T).max())
    if d <= 1e-8:
        print('-- %s already up to date (max diff %.1e)' % (args.tf_yaml, d))
        return 0
    print('-- differs from the stored block by %.1e' % d)
    if not args.apply:
        print('   run with --apply to rewrite it')
        return 0
    for p in TC.write_transform('T_mb2fc', T, src, path=args.tf_yaml):
        print('-- wrote %s' % p)
    # round trip through the consumer that checks it against robot.yaml
    sys.path.insert(0, os.path.join(SRC_SPACE, 'path_tag_locator', 'src'))
    try:
        from path_tag_locator.constants import load_extrinsics_full
        ext = load_extrinsics_full(args.tf_yaml, robot_yaml_path=args.robot_yaml)
        print('-- loader: %s' % ext.note)
    except ImportError:
        pass
    return 0


def cmd_set(args):
    if args.npz:
        T = TC.load_npz(args.npz)
    elif args.matrix:
        T = np.asarray(args.matrix, float).reshape(4, 4)
    elif args.t_mm is not None:
        T = TC.matrix_from_pose(np.asarray(args.t_mm, float) / 1000.0, args.rpy_deg or [0.0, 0.0, 0.0])
    else:
        sys.exit('give --npz, --matrix (16 numbers) or --t-mm x y z [--rpy-deg r p y]')
    TC.assert_rigid(T, args.name)
    if args.new:
        if args.name in TC.block_names(args.tf_yaml):
            sys.exit('%s already exists — drop --new to update it' % args.name)
        if not args.source:
            sys.exit('--source is required')
        print('%s: NEW block, kind %s\n  %s' % (args.name, args.kind or 'calibrated', TC.describe(T)))
        for p in TC.write_transform(args.name, T, args.source, path=args.tf_yaml, kind=args.kind, create=True):
            print('-- wrote %s' % p)
        return 0
    if args.name not in TC.block_names(args.tf_yaml):
        sys.exit('no block %s in %s (add it with --new)' % (args.name, args.tf_yaml))
    old = TC.load_transform(args.name, args.tf_yaml)
    print('%s\n  old: %s\n  new: %s\n  change: dt %s mm, %.3f deg'
          % (args.name, TC.describe(old), TC.describe(T), _fmt((T[:3, 3] - old[:3, 3]) * 1e3, 2), _ang_deg(T, old)))
    if not args.source:
        sys.exit('--source is required: say how this value was measured (it is written into the yaml)')
    for p in TC.write_transform(args.name, T, args.source, path=args.tf_yaml, kind=args.kind, create=args.new):
        print('-- wrote %s' % p)
    return 0


def cmd_use(args):
    """Re-apply a block's sub-block pose (design / measured_<date> / ...)."""
    import time
    raw = yaml.safe_load(open(args.tf_yaml))
    if args.name not in raw:
        sys.exit('no block %s in %s' % (args.name, args.tf_yaml))
    alts = TC.alternatives(raw[args.name])
    if args.key not in alts:
        sys.exit('%s has no sub-block %r with t_mm / rpy_deg_xyz (have: %s)'
                 % (args.name, args.key, ', '.join(alts) or 'none'))
    t, r = alts[args.key]
    T = TC.matrix_from_pose(np.asarray(t) / 1000.0, r)
    old = TC.load_transform(args.name, args.tf_yaml)
    kind = args.kind or ('design' if args.key == 'design' else 'calibrated')
    note = (raw[args.name][args.key] or {}).get('note', '')
    src = args.source or ('%s %s applied %s by tf_chain_tool.py use%s'
                          % (kind.upper(), args.key, time.strftime('%Y-%m-%d'), (' — ' + note) if note else ''))
    print('%s <- %s\n  old: %s\n  new: %s\n  change: dt %s mm, %.3f deg\n  kind: %s'
          % (args.name, args.key, TC.describe(old), TC.describe(T), _fmt((T[:3, 3] - old[:3, 3]) * 1e3, 2), _ang_deg(T, old), kind))
    for p in TC.write_transform(args.name, T, src, path=args.tf_yaml, kind=kind):
        print('-- wrote %s' % p)
    return 0


def cmd_export(args):
    for p in TC.export_npz(args.tf_yaml):
        print('-- wrote %s' % p)
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tf-yaml', default=TC.TF_CHAIN_PATH)
    ap.add_argument('--robot-yaml', default=CONFIG_PATH)
    ap.add_argument('--urdf', default=URDF, help='the planner URDF (check / urdf / show)')
    sp = ap.add_subparsers(dest='cmd')
    sp.add_parser('show')
    sp.add_parser('check')
    p = sp.add_parser('urdf'); p.add_argument('--apply', action='store_true', help='rewrite the two joint lines in place')
    sp.add_parser('export-npz')
    p = sp.add_parser('front-cam'); p.add_argument('--apply', action='store_true')
    p = sp.add_parser('joint-offsets')
    p.add_argument('--apply', metavar='ARM_OFFSETS_NPZ', help='an arm_offsets.py result (dq_deg, hand_eye, urdf)')
    p.add_argument('--source', default='')
    p.add_argument('--note', default='')
    p.add_argument('--disable', action='store_true'); p.add_argument('--enable', action='store_true')
    p = sp.add_parser('set')
    p.add_argument('name', help='a block of the yaml (%s, or any T_x2y with --new)' % ', '.join(TC.NAMES))
    p.add_argument('--npz')
    p.add_argument('--matrix', nargs=16, type=float)
    p.add_argument('--t-mm', nargs=3, type=float)
    p.add_argument('--rpy-deg', nargs=3, type=float, help="scipy extrinsic 'xyz' euler, degrees")
    p.add_argument('--source', default='', help='ONE line: what, when, by which tool')
    p.add_argument('--kind', choices=TC.KINDS, default=None, help='rewrite the kind: line too')
    p.add_argument('--new', action='store_true', help='append a new block (the loader picks it up)')
    p = sp.add_parser('use')
    p.add_argument('name')
    p.add_argument('key', help='sub-block to apply: design, measured_<date>, ...')
    p.add_argument('--kind', choices=TC.KINDS, default=None, help='default: design for `design`, else calibrated')
    p.add_argument('--source', default='')
    args = ap.parse_args()
    if not args.cmd:
        ap.print_help()
        return 2
    return {'show': cmd_show, 'check': cmd_check, 'urdf': cmd_urdf, 'front-cam': cmd_front_cam,
            'set': cmd_set, 'use': cmd_use, 'export-npz': cmd_export, 'joint-offsets': cmd_joint_offsets}[args.cmd](args) or 0


if __name__ == '__main__':
    sys.exit(main())
