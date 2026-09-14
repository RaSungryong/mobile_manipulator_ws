#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline checks of TaskManager's file-driven task discovery (2026-09-14).

Stubs rospy, loads the REAL TaskManager against the REAL task/csv directory
and against scratch directories, and asserts:

  * every assigned_workpoints_<key>.csv registers scan_pose_<key> (pose mode,
    IK seed from the paired rrt file on every work point);
  * every rrt_final_path_<key>.csv registers scan_joint_<key> (joint mode,
    transition/home waypoints flagged scan=False, world x y z from the paired
    pose file on every work point);
  * result files next to the inputs are never mistaken for path data;
  * an unpaired file still registers on its own; explicit TASK_DEFS still
    work (groups filter); describe_tasks() is JSON-safe and ordered.

Run with a sourced workspace:  python3 tools/check_task_discovery.py
"""
import csv
import json
import os
import shutil
import sys
import tempfile
import types

# ---------------------------------------------------------------- rospy stub
LOG = {'info': [], 'warn': [], 'err': []}
rospy = types.ModuleType('rospy')
rospy.loginfo = lambda m, *a: LOG['info'].append(str(m))
rospy.logwarn = lambda m, *a: LOG['warn'].append(str(m))
rospy.logerr = lambda m, *a: LOG['err'].append(str(m))
rospy.logdebug = lambda m, *a: None
sys.modules['rospy'] = rospy

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))
from apriltag_nav.task_manager import TaskManager  # noqa: E402

REAL_TASK_DIR = os.path.join(PKG, 'task', 'csv')

N_OK = 0
N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1
        print(f'  ok   {what}')
    else:
        N_FAIL += 1
        print(f'  FAIL {what}')


def fresh_log():
    for k in LOG:
        LOG[k].clear()


# ================================================================ real dir
print('== real task dir:', REAL_TASK_DIR)
fresh_log()
tm = TaskManager(REAL_TASK_DIR)
names = tm.get_all_task_names()
pose_tasks = [n for n in names if n.startswith(TaskManager.POSE_TASK_PREFIX)]
joint_tasks = [n for n in names if n.startswith(TaskManager.JOINT_TASK_PREFIX)]
print('  registered:', names)
check(not LOG['err'], f'no logerr during load ({LOG["err"][:2]})')
check(len(pose_tasks) == 3 and len(joint_tasks) == 3,
      f'3 pose + 3 joint tasks discovered ({len(pose_tasks)}/{len(joint_tasks)})')
check('go_home' in names and names[-1] == 'go_home', 'go_home is last (system)')
keys = sorted(n[len(TaskManager.POSE_TASK_PREFIX):] for n in pose_tasks)
check(keys == sorted(n[len(TaskManager.JOINT_TASK_PREFIX):] for n in joint_tasks),
      'pose and joint tasks share the same run keys')
check(all('standoff_010mm' in k or 'standoff_030mm' in k or 'standoff_050mm' in k
          for k in keys), f'keys are the three standoffs: {keys}')

k010 = [k for k in keys if 'standoff_010mm' in k][0]
pose010 = TaskManager.POSE_TASK_PREFIX + k010
joint010 = TaskManager.JOINT_TASK_PREFIX + k010

# Expected numbers come from the FILES, read independently with pandas, so
# the check survives the user swapping in a new export (errorY -> errorX on
# 2026-09-14 changed every tag list and count).
import pandas as pd


def expect(key):
    pose_df = pd.read_csv(os.path.join(REAL_TASK_DIR, TaskManager.POSE_FILE_PREFIX + key + '.csv'))
    joint_df = pd.read_csv(os.path.join(REAL_TASK_DIR, TaskManager.JOINT_FILE_PREFIX + key + '.csv'))
    tags = sorted(int(g) for g in pose_df['group_id'].unique())
    n_work = len(joint_df[joint_df['is_task_waypoint'].astype(float) == 1])
    return {'tags': tags, 'n_pose': len(pose_df), 'n_rows': len(joint_df),
            'n_work': n_work, 'n_trav': len(joint_df) - n_work,
            'n_first_group_pose': int((pose_df['group_id'] == tags[0]).sum()),
            'speeds': {int(v) for v in joint_df['speed'].unique()}}


E010 = expect(k010)

# ---- pose task
steps = tm.get_task(pose010)
tags = [s['tag'] for s in steps]
check(tags == E010['tags'], f'{pose010} tags {tags} == the file\'s group_ids in ascending order')
pts = [p for t in tags for p in tm.get_scan_points(pose010, t)]
check(len(pts) == E010['n_pose'], f'pose task has every file row as a work point ({len(pts)} of {E010["n_pose"]})')
check(all(p['mode'] == 'pose' for p in pts), 'all pose points are mode=pose')
check(all('q0' in p and len(p['q0']) == 6 for p in pts),
      'every pose point carries a 6-value IK seed from the paired rrt file')
check(all(k in p for p in pts for k in ('x', 'y', 'z', 'rx', 'ry', 'rz')),
      'pose points carry x y z rx ry rz')
check(all(p.get('scan', True) for p in pts), 'pose points are all scan points')
check(tm.get_lift_height(pose010) == 0.0, 'pose task lift height 0.0 (lift_mm)')
info = tm.task_info[pose010]
check(info['scan_mode'] == 'pose' and info['source'] == 'discovered'
      and info['points'] == E010['n_pose'] and info['ik_seeded_points'] == E010['n_pose']
      and info['paired_file'].startswith(TaskManager.JOINT_FILE_PREFIX),
      f'pose task_info: {json.dumps({k: info[k] for k in ("scan_mode", "points", "ik_seeded_points", "paired_file")})}')
check(info['result_name'] == pose010 + TaskManager.RESULT_SUFFIX
      and not info['result_name'].startswith(TaskManager.POSE_FILE_PREFIX),
      f'result name {info["result_name"]} cannot be re-discovered as input')

# ---- joint task
steps = tm.get_task(joint010)
tags_j = [s['tag'] for s in steps]
check(tags_j == tags, f'{joint010} tags match the pose task {tags_j}')
jpts = [p for t in tags_j for p in tm.get_scan_points(joint010, t)]
scan_pts = [p for p in jpts if p['scan']]
trav_pts = [p for p in jpts if not p['scan']]
check(len(jpts) == E010['n_rows'] and len(scan_pts) == E010['n_work'] and len(trav_pts) == E010['n_trav'],
      f'joint path: {len(jpts)} rows = {len(scan_pts)} work + {len(trav_pts)} traverse '
      f'(file: {E010["n_rows"]} = {E010["n_work"]} + {E010["n_trav"]})')
check(all(p['mode'] == 'joint' and len(p['joints']) == 6 for p in jpts),
      'all joint points are mode=joint with 6 angles')
check(all(abs(q) < 7 for p in jpts for q in p['joints']),
      'joint angles are radians (|q| < 7)')
check(all(all(k in p for k in ('x', 'y', 'z')) for p in scan_pts),
      'every joint WORK point carries world x y z from the paired pose file')
check(all(not any(k in p for k in ('x', 'y', 'z')) for p in trav_pts),
      'traverse points carry no world coords (nothing to map)')
first = tm.get_scan_points(joint010, tags[0])[0]
check(first['scan'] is False and first['point_id'] == 1,
      f'group {tags[0]} starts with its home waypoint (scan False)')
check({int(p['speed']) for p in jpts} == E010['speeds'] == {10, 30},
      f'per-row speed passes through and is the halved 10 / 30 set ({E010["speeds"]})')
info = tm.task_info[joint010]
check(info['scan_mode'] == 'joint' and info['points'] == E010['n_work']
      and info['traverse_points'] == E010['n_trav'] and info['points_with_world_xyz'] == E010['n_work']
      and info['paired_file'].startswith(TaskManager.POSE_FILE_PREFIX),
      'joint task_info counts and pairing')
check(any(joint010 in m and 'JOINT path replay' in m for m in LOG['warn']),
      'joint task registration warns about the bare-MoveJ replay')

# ---- the other standoffs differ in assignment
k050 = [k for k in keys if 'standoff_050mm' in k][0]
t050 = [s['tag'] for s in tm.get_task(TaskManager.POSE_TASK_PREFIX + k050)]
check(t050 == expect(k050)['tags'] and t050 != tags,
      f'standoff 050 tags {t050} come from its own file and differ from 010')

# ---- describe_tasks
desc = tm.describe_tasks()
try:
    json.dumps(desc)
    js_ok = True
except Exception as e:
    js_ok = False
    print('   json error:', e)
check(js_ok, 'describe_tasks() is JSON-serialisable')
check([d['name'] for d in desc] == names, 'describe_tasks order == registration order')
check(desc[-1]['kind'] == 'system' and desc[-1]['tags'] == [TaskManager.START_TAG],
      'go_home described as system task to START_TAG')

# ================================================================ scratch dirs
def write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


tmp = tempfile.mkdtemp(prefix='tm_check_')
try:
    real_pose = os.path.join(REAL_TASK_DIR, TaskManager.POSE_FILE_PREFIX + k010 + '.csv')
    real_joint = os.path.join(REAL_TASK_DIR, TaskManager.JOINT_FILE_PREFIX + k010 + '.csv')

    # -- 1. result files next to the inputs are ignored
    d1 = os.path.join(tmp, 'd1'); os.makedirs(d1)
    shutil.copy(real_pose, d1); shutil.copy(real_joint, d1)
    shutil.copy(real_pose, os.path.join(d1, TaskManager.POSE_FILE_PREFIX + k010 + '_result_20260914_120000.csv'))
    shutil.copy(real_pose, os.path.join(d1, pose010 + '_ra_map_20260914_120000.csv'))
    shutil.copy(real_pose, os.path.join(d1, 'optimized_joints_line1_result_20260814_175030.csv'))
    open(os.path.join(d1, TaskManager.POSE_FILE_PREFIX + 'notes.txt'), 'w').write('x')
    print('== scratch d1 (pair + result files + txt)')
    fresh_log()
    tm1 = TaskManager(d1)
    check(sorted(tm1.get_all_task_names()) == sorted([pose010, joint010, 'go_home']),
          f'only the pair registers: {tm1.get_all_task_names()}')
    check(not LOG['err'], 'no logerr')

    # -- 2. unpaired pose file alone
    d2 = os.path.join(tmp, 'd2'); os.makedirs(d2)
    shutil.copy(real_pose, d2)
    print('== scratch d2 (pose file only)')
    fresh_log()
    tm2 = TaskManager(d2)
    check(tm2.get_all_task_names() == [pose010, 'go_home'], 'pose task registers alone')
    p2 = [p for t in [s['tag'] for s in tm2.get_task(pose010)] for p in tm2.get_scan_points(pose010, t)]
    check(len(p2) == E010['n_pose'] and not any('q0' in p for p in p2),
          'unpaired pose task has no IK seed (GetInverseKin path)')
    check(tm2.task_info[pose010]['paired_file'] is None
          and tm2.task_info[pose010]['ik_seeded_points'] == 0, 'task_info says unpaired')

    # -- 3. unpaired joint file alone
    d3 = os.path.join(tmp, 'd3'); os.makedirs(d3)
    shutil.copy(real_joint, d3)
    print('== scratch d3 (joint file only)')
    fresh_log()
    tm3 = TaskManager(d3)
    check(tm3.get_all_task_names() == [joint010, 'go_home'], 'joint task registers alone')
    p3 = [p for t in [s['tag'] for s in tm3.get_task(joint010)] for p in tm3.get_scan_points(joint010, t)]
    check(len(p3) == E010['n_rows'] and not any('x' in p for p in p3),
          'unpaired joint task has no world coords')
    check(tm3.task_info[joint010]['points_with_world_xyz'] == 0, 'task_info says 0 xyz')

    # -- 4. explicit TASK_DEFS override with a groups filter
    print('== scratch d1 + explicit TASK_DEFS (groups filter)')
    saved = TaskManager.TASK_DEFS
    TaskManager.TASK_DEFS = {
        'scan_g104_standoff010': {
            'file': os.path.basename(real_pose),
            'joint_file': os.path.basename(real_joint),
            'groups': [tags[0]], 'type': 'scan', 'scan_mode': 'pose',
        },
        joint010: {   # override the discovered joint task: groups filter
            'file': os.path.basename(real_joint),
            'pose_file': os.path.basename(real_pose),
            'groups': tags[:2], 'type': 'scan', 'scan_mode': 'joint',
        },
    }
    try:
        fresh_log()
        tm4 = TaskManager(d1)
        n4 = tm4.get_all_task_names()
        check('scan_g104_standoff010' in n4, f'explicit extra registered: {n4}')
        check([s['tag'] for s in tm4.get_task('scan_g104_standoff010')] == [tags[0]]
              and len(tm4.get_scan_points('scan_g104_standoff010', tags[0])) == E010['n_first_group_pose'],
              f'groups filter [{tags[0]}] -> {E010["n_first_group_pose"]} pose points')
        check([s['tag'] for s in tm4.get_task(joint010)] == tags[:2],
              f'explicit override of a discovered name wins (groups {tags[:2]})')
        check(any('overrides the discovered task' in m for m in LOG['warn']),
              'override is announced')
        check(tm4.task_info['scan_g104_standoff010']['source'] == 'explicit'
              and tm4.task_info['scan_g104_standoff010']['groups_filter'] == [tags[0]],
              'task_info records source=explicit and the groups filter')
        check(tm4.task_info['scan_g104_standoff010']['result_name'] is None
              or not tm4.task_info['scan_g104_standoff010']['result_name'].startswith(
                  TaskManager.POSE_FILE_PREFIX), 'explicit result name safe')
    finally:
        TaskManager.TASK_DEFS = saved

    # -- 5. a file whose lift_mm disagrees is refused, the rest still loads
    d5 = os.path.join(tmp, 'd5'); os.makedirs(d5)
    shutil.copy(real_joint, d5)
    with open(real_pose, newline='', encoding='utf-8-sig') as f:
        rows = list(csv.reader(f))
    hdr, body = rows[0], rows[1:]
    li = hdr.index('lift_mm')
    body[5][li] = '150'
    write_csv(os.path.join(d5, os.path.basename(real_pose)), hdr, body)
    print('== scratch d5 (pose file with a disagreeing lift_mm)')
    fresh_log()
    tm5 = TaskManager(d5)
    check(pose010 not in tm5.get_all_task_names() and joint010 in tm5.get_all_task_names(),
          'bad pose file refused, joint task still registers')
    check(any('lift_height disagrees' in m for m in LOG['err']), 'refusal is logged')

    # -- 6. discover_task_defs is pure and stable
    d6 = os.path.join(tmp, 'd6'); os.makedirs(d6)
    for k in ('b_run', 'a_run'):
        open(os.path.join(d6, TaskManager.POSE_FILE_PREFIX + k + '.csv'), 'w').write('')
        open(os.path.join(d6, TaskManager.JOINT_FILE_PREFIX + k + '.csv'), 'w').write('')
    defs = TaskManager.discover_task_defs(d6)
    check(list(defs) == ['scan_joint_a_run', 'scan_joint_b_run',
                         'scan_pose_a_run', 'scan_pose_b_run'],
          f'discover_task_defs sorted by name: {list(defs)}')
    check(defs['scan_pose_a_run']['joint_file'] == 'rrt_final_path_a_run.csv'
          and defs['scan_joint_a_run']['pose_file'] == 'assigned_workpoints_a_run.csv',
          'pairing by key in both directions')
    check(TaskManager.discover_task_defs(os.path.join(tmp, 'nope')) == {},
          'missing dir -> no defs')
    print('== scratch d6 (empty CSVs): loader must not crash')
    fresh_log()
    tm6 = TaskManager(d6)
    check(tm6.get_all_task_names() == ['go_home'], 'empty CSVs register nothing')
finally:
    shutil.rmtree(tmp, ignore_errors=True)

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
