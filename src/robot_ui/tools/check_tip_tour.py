#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Offline checks for the per-stop tip tour (2026-09-28): robot_ui.tip_check
Settings.targets / target_info / skip_refused, design_stop_pose for zone C,
the tip_touch_cross_tags plugin's pairing, and analyze_tip_check's
image -> world rotation. Runs run_sequence against a fake bridge; records
go to a temp dir, nothing touches ROS.

    python3 src/robot_ui/tools/check_tip_tour.py
"""

import csv
import math
import os
import sys
import tempfile
import time

import numpy as np
import yaml

import rospy
rospy.get_param = lambda name, default=None: default

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'plugins'))
sys.path.insert(0, HERE)

from robot_ui import tip_check as TC                       # noqa: E402
import tip_touch_cross_tags as PLUGIN                      # noqa: E402
import analyze_tip_check as AN                             # noqa: E402

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1
        print(f'  ok   {what}')
    else:
        N_FAIL += 1
        print(f'  FAIL {what}')


HOME_DEG = [-90.0, -90.0, 90.0, -90.0, -90.0, 0.0]
HOME_TCP = [-159.0, 700.0, 774.0, 180.0, 0.0, 0.0]


class FakeBridge:
    """Just enough of RosBridge for run_sequence."""

    def __init__(self):
        self.calls = []
        self.tcp = list(HOME_TCP)
        self.joints = list(HOME_DEG)
        self.ts = {'state': 'IDLE', 'task': None, 'stamp': time.time() - 10}
        self._pending = None
        self.pose = None

    def cached_states(self):
        if self._pending:
            name, n = self._pending
            if n == 0:
                self.ts = {'state': 'MOVING', 'task': name, 'stamp': time.time()}
                self._pending = (name, 1)
            else:
                self.ts = {'state': 'IDLE', 'task': None, 'stamp': time.time()}
                self._pending = None
        return {'task_state': dict(self.ts), 'lift_state': {'height_mm': 0.0},
                'standoff_state': {'standoff_mm': 16.5}}

    def send_task_command(self, cmd):
        self.calls.append(('task', cmd))
        if cmd.startswith('GOTO'):
            tag = int(cmd.split()[1])
            self._pending = ('goto_%d' % tag, 0)
            self.tag = tag
            self.tcp, self.joints = list(HOME_TCP), list(HOME_DEG)
            p = TC.design_stop_pose(tag)
            self.pose = {'id': tag, 'x': p.x, 'y': p.y, 'theta': p.theta,
                         'stamp': time.time() + 0.5}

    def mobile_snapshot(self):
        return {'result': {'ok': True, 'tag': self.tag, 'message': 'arrived'},
                'last_known_tag': self.tag}

    def robot_pose_snapshot(self):
        return dict(self.pose) if self.pose else None

    def arm_snapshot(self):
        return {'pose_valid': True, 'tcp_pose': list(self.tcp), 'joints': list(self.joints)}

    def arm_move_cart(self, pose, vel=30.0, acc=50.0, timeout=60.0, linear=True,
                      physical=False):
        self.calls.append(('move', [round(v, 1) for v in pose], linear))
        self.tcp = list(pose)
        self.joints = [0.0, -10.0, 90.0, -170.0, -90.0, 15.0]
        return True, 'move_cart ok'

    def arm_standoff(self, target_mm=None, timeout=120.0):
        self.calls.append(('standoff', target_mm))
        self.tcp[2] -= 4.8
        return True, 'standoff ok'

    def capture(self, num_samples=1, use_vision_led=True):
        self.calls.append(('capture', use_vision_led))
        return True, 'captured', [np.zeros((8, 8), np.uint8)]

    def arm_home(self, timeout=60.0):
        self.calls.append(('home',))
        return True, 'home'


class Ctx:
    def __init__(self, bridge):
        self.bridge = bridge
        self.lines = []

    def log(self, m):
        self.lines.append(m)

    def cancelled(self):
        return False

    def sleep(self, t):
        return True


def settings(stops, skip):
    targets, info = PLUGIN.build_targets([t for t in stops])
    return TC.Settings(name='t', target_world_m=(9.0, 9.0, 0.0), stop_tags=stops, rounds=1,
                       dwell_s=0.0, standoff=True, capture=True, pre_standoff_wait_s=0.0,
                       targets=targets, target_info=info, skip_refused=skip)


def main():
    tmp = tempfile.mkdtemp(prefix='tip_tour_')
    TC.RECORD_ROOT = os.path.join(tmp, 'log')
    TC.IMAGE_ROOT = os.path.join(tmp, 'img')
    PLUGIN_ALL = PLUGIN.STOP_TAGS

    print('== Settings')
    s = TC.Settings(name='x', target_world_m=(1.0, 2.0, 0.0), targets={5: (3, 4, 0.1)},
                    target_info={5: {'ref_tag': 2}})
    check(s.for_tag(5).target_world_m == (3.0, 4.0, 0.1) and s.for_tag(6).target_world_m == (1.0, 2.0, 0.0),
          'for_tag: own target, else the default')
    check(s.target_world_m == (1.0, 2.0, 0.0) and s.info(5) == {'ref_tag': 2} and s.info(6) == {},
          'for_tag does not mutate the base settings; info per tag')

    print('== design stop poses')
    pB, pC = TC.design_stop_pose(104), TC.design_stop_pose(117)
    check(pB.theta == 90.0 and pC.theta == -90.0, 'zone B heads +90, zone C -90')
    with open(TC.paths.MAP_PATH) as f:
        m = yaml.safe_load(f)
    tags = m.get('tags', m)
    aB = np.asarray(TC.transform_world_to_arm(dict(x=tags[104]['x'], y=tags[104]['y'], z=0, rx=0, ry=0, rz=0), pB, 0.0)[0])
    aC = np.asarray(TC.transform_world_to_arm(dict(x=tags[117]['x'], y=tags[117]['y'], z=0, rx=0, ry=0, rz=0), pC, 0.0)[0])
    check(np.allclose(aB, aC, atol=0.5), f'each zone\'s own tag lands at the same arm point {aB.round(1)} / {aC.round(1)}')

    print('== plugin pairing')
    tg, info = PLUGIN.build_targets(PLUGIN_ALL)
    check(len(PLUGIN_ALL) == 22 and not {100, 112, 113, 125} & set(PLUGIN_ALL),
          '22 stops, the four unreachable lane-end tags left out')
    check(info[101] == {'ref_tag': 0, 'marker_id': 219} and info[107]['ref_tag'] == 1
          and info[111] == {'ref_tag': 2, 'marker_id': 221} and info[114]['marker_id'] == 222
          and info[120]['ref_tag'] == 4 and info[124] == {'ref_tag': 5, 'marker_id': 224},
          'ref pairing and marker ids as specified (0->219 ... 5->224)')
    check(tg[101] == (-0.6, -1.2, 0.002) and tg[124] == (0.6, -1.2, 0.002),
          'targets = cross-tag position, 1 mm above its top face')
    for t in PLUGIN_ALL:
        st = settings([t], False).for_tag(t)
        _, fl, _, _, _ = TC.compute_targets(st, TC.design_stop_pose(t), 0.0, True, TC.tf_chain.tip_offset_mm())
        if not TC.check_bounds(fl)[0]:
            check(False, f'tag {t} within bounds')
            break
    else:
        check(True, 'all 22 stops within the flange bounds')

    print('== run_sequence, per-stop targets, skip_refused')
    br = FakeBridge()
    ctx = Ctx(br)
    targets, info = PLUGIN.build_targets([102, 115])
    targets[112], info[112] = (-0.6, 1.2, 0.002), {'ref_tag': 2, 'marker_id': 221}
    s = TC.Settings(name='t', target_world_m=(9.0, 9.0, 0.0), stop_tags=[102, 112, 115], rounds=1,
                    dwell_s=0.0, standoff=True, capture=True, pre_standoff_wait_s=0.0,
                    targets=targets, target_info=info, skip_refused=True)
    TC.run_sequence(ctx, s)
    gotos = [c[1] for c in br.calls if c[0] == 'task']
    moves = [c for c in br.calls if c[0] == 'move']
    check(gotos == ['GOTO 102', 'GOTO 112', 'GOTO 115'], f'three GOTOs in order: {gotos}')
    check(len(moves) == 6, f'no arm motion at the refused stop (moves {len(moves)}, expected 2 x 3)')
    # expected descend targets from the LIVE transform (tf_chain changes move them)
    tip_off = TC.tf_chain.tip_offset_mm()
    exp = [np.round(TC.compute_targets(s.for_tag(t), TC.design_stop_pose(t), 0.0, True, tip_off)[2], 1).tolist()
           for t in (102, 115)]
    check(np.allclose(moves[1][1][:3], exp[0], atol=0.15) and np.allclose(moves[4][1][:3], exp[1], atol=0.15)
          and abs(exp[0][1] - exp[1][1]) > 5.0,
          f'each stop descends to its OWN cross tag: {moves[1][1][:3]} / {moves[4][1][:3]}')
    check(any('SKIPPED' in l for l in ctx.lines) and br.calls[-1] == ('home',),
          'refused stop logged SKIPPED, sequence continued to the end and homed')
    rec = [d for d in os.listdir(TC.RECORD_ROOT)][0]
    rdir = os.path.join(TC.RECORD_ROOT, rec)
    with open(os.path.join(rdir, 'summary.csv')) as f:
        rows = list(csv.DictReader(f))
    by = {r['tag']: r for r in rows}
    check(set(by) == {'102', '112', '115'}, 'three summary rows, the skipped one included')
    check(by['102']['ref_tag'] == '0' and by['102']['marker_id'] == '219'
          and by['115']['marker_id'] == '222' and by['112']['message'].startswith('SKIPPED'),
          'ref_tag / marker_id / SKIPPED in summary.csv')
    with open(os.path.join(rdir, 'r1_tag115.yaml')) as f:
        y115 = yaml.safe_load(f)
    check(y115['target_world_m_plate_top'] == [0.6, 1.2, 0.002]
          and y115['target_info'] == {'ref_tag': 3, 'marker_id': 222}
          and abs(y115['before']['tip_error_world_mm'][0]) < 0.5,
          'per-point yaml: own target, target_info, tip error vs its own target')
    with open(os.path.join(rdir, 'session.yaml')) as f:
        sess = yaml.safe_load(f)
    check(sess['targets_per_stop'][115] == [0.6, 1.2, 0.002], 'session.yaml carries the per-stop targets')

    br2 = FakeBridge()
    ctx2 = Ctx(br2)
    s.skip_refused = False
    TC.run_sequence(ctx2, s)
    check([c[1] for c in br2.calls if c[0] == 'task'] == ['GOTO 102', 'GOTO 112'],
          'skip_refused False: the sequence ends at the refused stop')

    print('== analyze_tip_check image -> world')
    tcp = (180.0, 0.0, 0.0)
    vB = AN.image_to_world(np.array([1.0, 0.0]), -179.212, tcp, TC.design_stop_pose(102), 0.0)
    vC = AN.image_to_world(np.array([1.0, 0.0]), -179.212, tcp, TC.design_stop_pose(115), 0.0)
    check(abs(np.linalg.norm(vB) - 1.0) < 1e-3, 'rotation keeps length')
    check(vB[1] > 0.99 and np.allclose(vB, -vC, atol=0.03),
          f'zone B: image x ~ world +y; zone C (heading flipped) the opposite: {vB.round(3)} / {vC.round(3)}')
    check(abs(AN.axis_dev_deg(np.array([1.0, 0.02])) - 1.146) < 0.01
          and abs(AN.axis_dev_deg(np.array([-0.02, 1.0])) - 1.146) < 0.01, 'axis deviation wraps to +-45')

    print(f'\n{N_OK} ok, {N_FAIL} failed')
    sys.exit(1 if N_FAIL else 0)


if __name__ == '__main__':
    main()
