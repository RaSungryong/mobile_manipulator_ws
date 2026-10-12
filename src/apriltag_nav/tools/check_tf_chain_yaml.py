#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_tf_chain_yaml.py
======================
OFFLINE self-check for the tf_chain.yaml tooling of 2026-10-13 (config
stage 3) — no ROS, no robot, and the REAL file is never written: everything
runs on a copy in a temp dir::

    python3 src/apriltag_nav/tools/check_tf_chain_yaml.py

What it guards
--------------
  * the committed file follows the block convention: every block has a
    `kind:` from tf_chain.KINDS, a one-line `source:`, readouts that agree
    with `matrix`, and HISTORY.md holds the pre-10-13 texts;
  * `tf_chain_tool.py use NAME KEY` applies a sub-block's pose exactly, sets
    `kind`, keeps every other line, and `use ... design` brings the file back;
  * `set --new` appends a block the loader and `check` pick up without any
    code change, `set` on a missing name without --new refuses, `set --new`
    on an existing name refuses;
  * `set` on an existing block keeps its kind line and sub-blocks;
  * the loader still requires NAMES and is unaffected by extra blocks.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
from apriltag_nav import tf_chain as TC                                 # noqa: E402

TOOL = os.path.join(HERE, 'tf_chain_tool.py')
REAL = TC.TF_CHAIN_PATH

_n = [0]
_bad = [0]


def check(name, ok, detail=''):
    _n[0] += 1
    if not ok:
        _bad[0] += 1
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")


def tool(tf_yaml, *args):
    r = subprocess.run([sys.executable, TOOL, '--tf-yaml', tf_yaml] + list(args),
                       capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


def value_free_lines(text):
    """The file minus the lines `set` is allowed to rewrite (kind/source/t_mm/rpy/matrix rows)."""
    out = []
    for ln in text.splitlines():
        if re.match(r'^  (kind|source|t_mm|rpy_deg_xyz|matrix):', ln):
            continue
        if re.match(r'^    \[?\s*-?\d', ln):          # matrix rows
            continue
        out.append(ln)
    return out


print('== the committed file')
raw = yaml.safe_load(open(REAL))
names = TC.block_names(REAL)
check('blocks with a matrix == NAMES (no extra block committed)', names == list(TC.NAMES), str(names))
for n in names:
    k = raw[n].get('kind')
    check(f'{n} kind in KINDS', k in TC.KINDS, str(k))
    src = str(raw[n].get('source', ''))
    check(f'{n} source is one line of <= 240 chars', '\n' not in src and len(src) <= 240, f'{len(src)} chars')
check('readouts == matrix for every block', all(ok for _, ok, _ in TC.check_readouts(REAL)))
hist = os.path.join(os.path.dirname(REAL), 'HISTORY.md')
check('HISTORY.md next to the yaml names every block', os.path.isfile(hist)
      and all(('## ' + n) in open(hist).read() for n in TC.NAMES))
alts = TC.alternatives(raw['T_ee2tip'])
check('T_ee2tip alternatives: design + measured_2026_09_22', set(alts) == {'design', 'measured_2026_09_22'}, str(sorted(alts)))
check('T_hc2ee has no usable alternative (design is null)', TC.alternatives(raw['T_hc2ee']) == {})

tmp = tempfile.mkdtemp(prefix='check_tf_chain_yaml_')
tfd = os.path.join(tmp, 'tf')
shutil.copytree(os.path.dirname(REAL), tfd)
Y = os.path.join(tfd, 'tf_chain.yaml')
orig_text = open(Y).read()
orig = TC.load_tf_chain(Y)

print('== use NAME KEY (on the copy)')
rc, out = tool(Y, 'use', 'T_ee2tip', 'measured_2026_09_22')
check('use T_ee2tip measured_2026_09_22 exits 0', rc == 0, out.strip().splitlines()[-1] if out.strip() else '')
T = TC.load_transform('T_ee2tip', Y)
t, r = alts['measured_2026_09_22']
check('applied matrix == the measured sub-block pose', np.allclose(T, TC.matrix_from_pose(np.asarray(t) / 1e3, r), atol=1e-9))
d = yaml.safe_load(open(Y))
check('kind became calibrated', d['T_ee2tip'].get('kind') == 'calibrated', str(d['T_ee2tip'].get('kind')))
check('sub-blocks survive', set(TC.alternatives(d['T_ee2tip'])) == {'design', 'measured_2026_09_22'})
check('npz follows', all(ok for n_, ok, _ in TC.check_npz_agree(Y) if n_ == 'T_ee2tip'))
check('readouts follow', all(ok for n_, ok, _ in TC.check_readouts(Y) if n_ == 'T_ee2tip'))
check('other blocks untouched', all(np.allclose(TC.load_transform(n, Y), orig[n], atol=1e-12) for n in TC.NAMES if n != 'T_ee2tip'))
rc, out = tool(Y, 'use', 'T_ee2tip', 'design')
check('use T_ee2tip design exits 0', rc == 0)
check('... and restores the committed matrix', np.allclose(TC.load_transform('T_ee2tip', Y), orig['T_ee2tip'], atol=1e-9))
check('... with kind design', yaml.safe_load(open(Y))['T_ee2tip'].get('kind') == 'design')
check('... every value-free line byte-identical to the original', value_free_lines(open(Y).read()) == value_free_lines(orig_text))
rc, out = tool(Y, 'use', 'T_hc2ee', 'design')
check('use of a null sub-block is refused', rc != 0 and 'no sub-block' in out, out.strip().splitlines()[-1] if out.strip() else '')
rc, out = tool(Y, 'use', 'T_ee2tip', 'nope')
check('use of an unknown key is refused', rc != 0)

print('== set --new (on the copy)')
rc, out = tool(Y, 'set', 'T_mb2sc', '--new', '--kind', 'calibrated', '--t-mm', '100', '200', '300',
               '--rpy-deg', '0', '0', '90', '--source', 'check_tf_chain_yaml test block')
check('set T_mb2sc --new exits 0', rc == 0, out.strip().splitlines()[-1] if out.strip() else '')
check('block_names lists it last', TC.block_names(Y) == list(TC.NAMES) + ['T_mb2sc'])
chain = TC.load_tf_chain(Y)
check('loader (require=NAMES) loads 5 blocks, the new one with its pose',
      len(chain) == 5 and np.allclose(chain['T_mb2sc'], TC.matrix_from_pose([0.1, 0.2, 0.3], [0, 0, 90]), atol=1e-9))
check('its npz exists and agrees', all(ok for _, ok, _ in TC.check_npz_agree(Y)))
check('its kind / readouts pass', all(ok for _, ok, _ in TC.check_readouts(Y)) and yaml.safe_load(open(Y))['T_mb2sc']['kind'] == 'calibrated')
rc, out = tool(Y, 'set', 'T_mb2sc', '--t-mm', '101', '200', '300', '--rpy-deg', '0', '0', '90', '--source', 'updated')
check('set on the new block (no --new) updates it', rc == 0 and abs(TC.load_transform('T_mb2sc', Y)[0, 3] - 0.101) < 1e-9)
rc, out = tool(Y, 'set', 'T_mb2sc', '--new', '--t-mm', '1', '2', '3', '--source', 'x')
check('set --new on an existing block is refused', rc != 0 and 'already exists' in out)
rc, out = tool(Y, 'set', 'T_nope', '--t-mm', '1', '2', '3', '--source', 'x')
check('set on a missing block without --new is refused', rc != 0 and 'add it with --new' in out)
rc, out = tool(Y, 'set', 'bad_name', '--new', '--t-mm', '1', '2', '3', '--source', 'x')
check('set --new with a name that is not T_x2y is refused', rc != 0)
rc, out = tool(Y, 'check')
check('check exits 0 with the extra block', rc == 0 and '0 failed' in out, out.strip().splitlines()[-1] if out.strip() else '')
rc, out = tool(Y, 'show')
check('show exits 0 and lists the extra block with its kind', rc == 0 and 'T_mb2sc' in out and '[calibrated]' in out)

print('== set on an existing block keeps kind and sub-blocks')
rc, out = tool(Y, 'set', 'T_ee2tip', '--t-mm', '0.12', '-251.96', '222.0', '--rpy-deg', '0', '0', '0', '--source', 'same value again')
d = yaml.safe_load(open(Y))
check('set exits 0, kind still design, sub-blocks intact', rc == 0 and d['T_ee2tip']['kind'] == 'design'
      and set(TC.alternatives(d['T_ee2tip'])) == {'design', 'measured_2026_09_22'})
rc, out = tool(Y, 'set', 'T_ee2tip', '--kind', 'calibrated', '--t-mm', '0.12', '-251.96', '222.0', '--rpy-deg', '0', '0', '0', '--source', 'kind switch')
check('set --kind rewrites the kind line', rc == 0 and yaml.safe_load(open(Y))['T_ee2tip']['kind'] == 'calibrated')

print('== the real file was not touched')
check('tf_chain.yaml unchanged', open(REAL).read() == orig_text)
shutil.rmtree(tmp, ignore_errors=True)

print(f'\n{_n[0] - _bad[0]} ok, {_bad[0]} failed')
sys.exit(1 if _bad[0] else 0)
