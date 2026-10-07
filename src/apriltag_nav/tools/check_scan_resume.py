#!/usr/bin/env python3
"""Offline check of RESUME — continuing an interrupted scan run in its own
result files (2026-10-06).

    python3 src/apriltag_nav/tools/check_scan_resume.py

Part 1 drives `apriltag_nav.scan_resume` on a scratch task set (the REAL
TaskManager over synthetic joint / pose files) with hand-written Ra maps
and measured CSVs. Part 2 sends RESUME / TASK through the real
task_executor with the charging check's fakes and asserts what the base
and the arm are told. Part 3 plans the real 2026-10-06 17:56 run when its
files are present (read-only). Exit status 1 on failure.
"""
import csv
import os
import shutil
import sys
import tempfile
import types

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))

import check_charging_manager as H          # noqa: E402  (rospy stubs + task_executor + fakes)
from apriltag_nav import scan_resume as SR  # noqa: E402
from apriltag_nav.task_manager import TaskManager  # noqa: E402

N = {'ok': 0, 'fail': 0}


def check(cond, what):
    N['ok' if cond else 'fail'] += 1
    print(('  ok   ' if cond else '  FAIL ') + what)


# ---------------------------------------------------------------- scratch task set
JOINT_COLS = ['target_line', 'group_id', 'point_id', 'q1', 'q2', 'q3', 'q4', 'q5', 'q6',
              'is_discontinuous', 'is_task_waypoint', 'source_point_id', 'waypoint_kind',
              'speed', 'lift_mm']
POSE_COLS = ['group_id', 'point_id', 'x', 'y', 'z', 'rx', 'ry', 'rz', 'speed', 'lift_mm']


def joint_rows(groups):
    """Per group: home, then (transition, work point) x n, then home."""
    rows = []
    for gid, n in groups:
        pid = 1
        rows.append([1, gid, pid, 0, 0, 0, 0, 0, 0, 0, 0, '', 'home', 10, 0]); pid += 1
        for k in range(n):
            rows.append([1, gid, pid, 0.1 * k, 0, 0, 0, 0, 0, 0, 0, '', 'transition', 10, 0]); pid += 1
            rows.append([1, gid, pid, 0.1 * k, 0.1, 0, 0, 0, 0, 0, 1, k + 1, 'task', 10, 0]); pid += 1
        rows.append([1, gid, pid, 0, 0, 0, 0, 0, 0, 0, 0, '', 'home', 10, 0])
    return rows


def pose_rows(groups):
    return [[gid, k + 1, 0.1 * k, -0.5, 0.6, 3.14, 0, 0, 10, 0] for gid, n in groups for k in range(n)]


def write_csv(path, cols, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(cols); w.writerows(rows)


def write_ra_map(path, done_keys, seeded_keys=(), failed_keys=()):
    cols = ['group_id', 'point_id', 'x', 'y', 'z', 'ra_mean', 'ra_std', 'ra_min', 'ra_max',
            'num_samples', 'success', 'execution_message', 'validated_at']
    rows = []
    for g, p in done_keys:
        rows.append([g, p, '', '', '', 1.23, 0.0, 1.23, 1.23, 1, 'True', 'Success (standoff ok)', '2026-10-06 18:00:00'])
    for g, p in failed_keys:
        rows.append([g, p, '', '', '', '', '', '', '', 0, 'False', 'MoveJ failed', '2026-10-06 18:00:01'])
    for g, p in seeded_keys:
        rows.append([g, p, '', '', '', '', '', '', '', 0, '', '', ''])
    write_csv(path, cols, rows)


def write_measured(path, keys, skipped=(), pending=()):
    """`pending` keys get a method-B mark row: mark_no, frames, Ra blank,
    not skipped, no stamp — captured and numbered, Ra entry still owed."""
    cols = ['run', 'mark_no', 'index', 'group_id', 'point_id', 'images', 'ra_measured', 'ra_readings',
            'note', 'skipped', 'standoff', 'x', 'y', 'z', 'image_dir', 'measured_at']
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rows = []
    for n, (g, p) in enumerate(keys, 1):
        if (g, p) in pending:
            rows.append(['r', n, p, g, p, f'g{g}_p{p}_i{p:04d}_s1.png', '', '',
                         'pending: Ra at the group entry stop', False, 'ok', '', '', '', '', ''])
        else:
            rows.append(['r', '', p, g, p, f'g{g}_p{p}_i{p:04d}_s1.png', '' if (g, p) in skipped else 1.5,
                         '' if (g, p) in skipped else 1.5, '', (g, p) in skipped, 'ok', '', '', '', '', ''])
    write_csv(path, cols, rows)


def scan_keys(points):
    return [(p['group_id'], p['point_id']) for p in points if p.get('scan', True)]


def main():
    tmp = tempfile.mkdtemp(prefix='scan_resume_')
    task_dir = os.path.join(tmp, 'csv'); ra_dir = os.path.join(tmp, 'ra_maps'); meas_root = os.path.join(tmp, 'scan_images')
    os.makedirs(task_dir); os.makedirs(ra_dir)
    groups = [(105, 4), (106, 3), (107, 2)]
    write_csv(os.path.join(task_dir, 'joint_t.csv'), JOINT_COLS, joint_rows(groups))
    write_csv(os.path.join(task_dir, 'pose_t.csv'), POSE_COLS, pose_rows(groups))
    tm = TaskManager(task_dir, result_dir=ra_dir)
    jt, pt = 'scan_joint_t', 'scan_pose_t'
    check(set(tm.get_all_task_names()) >= {jt, pt}, f'scratch tasks registered: {tm.get_all_task_names()}')
    j105 = tm.get_scan_points(jt, 105)
    j_keys = {g: scan_keys(tm.get_scan_points(jt, g)) for g, _ in groups}
    check([k[1] for k in j_keys[105]] == [3, 5, 7, 9] and len(j105) == 10
          and [p['source_point_id'] for p in j105 if p['scan']] == [1, 2, 3, 4]
          and all(p['source_point_id'] is None for p in j105 if not p['scan']),
          f'joint group 105: 4 work points among 10 path rows, point_id = path row, source_point_id = work point {[k[1] for k in j_keys[105]]}')

    print('== 1. done_points')
    stem = f'{jt}_ra_map_20261006_175616'
    ra_map = os.path.join(ra_dir, stem + '.csv')
    meas = os.path.join(meas_root, stem, stem + '_ra_measured.csv')
    # group 105 finished and answered (one skipped), 106's first point captured
    # (Ra map row) but the collect pause was cancelled (no measured row), the
    # rest of 106 seeded empty, one failed point in 106
    write_ra_map(ra_map, j_keys[105] + [j_keys[106][0]], seeded_keys=j_keys[106][2:], failed_keys=[j_keys[106][1]])
    write_measured(meas, j_keys[105], skipped=[j_keys[105][1]])
    done = SR.done_points(ra_map, meas)
    check(done == set(j_keys[105]), f'done = the 4 answered points of 105 (skip counts as answered): {sorted(done)}')
    check(j_keys[106][0] not in done, 'captured-but-unanswered point of 106 is NOT done (operator gets its stop back)')
    check(j_keys[106][1] not in done, 'a failed point is not done')
    done_nomeas = SR.done_points(ra_map, os.path.join(tmp, 'nope.csv'))
    check(done_nomeas == set(j_keys[105]) | {j_keys[106][0]}, 'without a measured CSV the Ra map row alone makes a point done')
    check(SR.done_points(os.path.join(tmp, 'missing.csv')) == set(), 'missing Ra map -> nothing done')
    check(SR.task_of(stem) == jt and SR.task_of('go_home_result_1') is None, 'task_of parses the run stem only')

    print('== 2. plan_resume, joint mode')
    plan = SR.plan_resume(stem, jt, tm.get_task(jt), tm.scan_points[jt], done, ra_map)
    check([s['tag'] for s in plan.steps] == [106, 107], f'finished group 105 dropped from the steps: {[s["tag"] for s in plan.steps]}')
    check(plan.groups_done == [105] and plan.groups_kept == [106, 107], 'groups done / kept reported')
    check(plan.n_total == 9 and plan.n_done == 4 and plan.n_remaining == 5, f'counts {plan.n_total}/{plan.n_done}/{plan.n_remaining}')
    p106 = plan.points_by_tag[106]
    check(len(p106) == len(tm.get_scan_points(jt, 106)), 'joint path rows all kept (route intact, index unchanged)')
    check([p['scan'] for p in p106] == [p['scan'] for p in tm.get_scan_points(jt, 106)],
          'nothing of 106 is done -> scan flags unchanged')
    check(all(p['csv_path'] == ra_map for p in p106) and all(p['csv_path'] == ra_map for p in plan.points_by_tag[107]),
          "every planned point carries the run's own csv_path")
    check(all(p.get('csv_path', '') != ra_map for p in tm.get_scan_points(jt, 106)), "TaskManager's own points untouched (copied)")
    check(105 not in plan.points_by_tag and not plan.nothing_left, 'no points planned for the dropped group')

    # partial group: the first two work points of 105 done, the rest open
    done2 = set(j_keys[105][:2])
    plan2 = SR.plan_resume(stem, jt, tm.get_task(jt), tm.scan_points[jt], done2, ra_map)
    p105 = plan2.points_by_tag[105]
    check([s['tag'] for s in plan2.steps] == [105, 106, 107], 'partly finished group keeps its step')
    flags = [(p['point_id'], p['scan']) for p in p105]
    orig = [(p['point_id'], p['scan']) for p in tm.get_scan_points(jt, 105)]
    check(len(p105) == 10 and [f for f in flags if f[0] in (3, 5)] == [(3, False), (5, False)]
          and [f for f in flags if f[0] in (7, 9)] == [(7, True), (9, True)]
          and [f for f in flags if f[0] not in (3, 5, 7, 9)] == [f for f in orig if f[0] not in (3, 5, 7, 9)],
          f'done work points become drive-through (scan False), open ones stay, transitions untouched : {flags}')
    check(plan2.n_done == 2 and plan2.n_remaining == 7, 'partial counts')

    # nothing left
    plan3 = SR.plan_resume(stem, jt, tm.get_task(jt), tm.scan_points[jt], set(sum(j_keys.values(), [])), ra_map)
    check(plan3.nothing_left and plan3.steps == [] and plan3.groups_done == [105, 106, 107], 'all done -> nothing left, no steps')
    # a move-only step survives
    plan4 = SR.plan_resume(stem, jt, [{'tag': 500, 'scan': False}] + tm.get_task(jt), tm.scan_points[jt], done, ra_map)
    check([s['tag'] for s in plan4.steps] == [500, 106, 107], 'a move-only step is kept as is')

    print('== 2b. group selection (2026-10-07)')
    rest, g, err = SR.parse_groups_arg('RESUME x groups=106,107'.split())
    check(rest == ['RESUME', 'x'] and g == {106, 107} and err is None, f'parse groups=106,107: {rest} {g}')
    rest, g, err = SR.parse_groups_arg('TASK x GROUPS=105;106+107'.split())
    check(rest == ['TASK', 'x'] and g == {105, 106, 107} and err is None, 'key case-insensitive, ; and + separators')
    rest, g, err = SR.parse_groups_arg('TASK groups=105 x'.split())
    check(rest == ['TASK', 'x'] and g == {105}, 'the token may sit anywhere')
    check(SR.parse_groups_arg(['TASK', 'x'])[1] is None, 'no token -> no filter')
    check(SR.parse_groups_arg(['TASK', 'x', 'groups=1a'])[2] and SR.parse_groups_arg(['TASK', 'x', 'groups='])[2],
          'bad id / empty list -> error, not "no filter"')
    check(SR.task_groups(tm.get_task(jt)) == [105, 106, 107], 'task_groups = the scan steps\' tags')
    check(SR.check_groups(tm.get_task(jt), {106}) is None and 'not in this task' in SR.check_groups(tm.get_task(jt), {106, 999}),
          'check_groups accepts the task\'s groups, names an unknown one')
    fs = SR.filter_steps([{'tag': 500, 'scan': False}] + tm.get_task(jt), {106})
    check([st['tag'] for st in fs] == [500, 106], f'filter_steps keeps move-only steps and the selected group: {[st["tag"] for st in fs]}')
    # resume of the 17:56-shaped run, groups 107 only: 105 done (dropped), 106 unfinished but skipped
    plang = SR.plan_resume(stem, jt, tm.get_task(jt), tm.scan_points[jt], done, ra_map, {107})
    check([st['tag'] for st in plang.steps] == [107] and plang.groups_kept == [107] and plang.groups_skipped == [106]
          and plang.groups_done == [105] and plang.groups_filter == [107],
          f'groups=107: only 107 planned, 106 skipped (not done), 105 done: {plang.summary()}')
    check(plang.n_remaining == 2 and plang.n_done == 4 and plang.n_total == 9, 'n_remaining counts the selected group only')
    check(106 not in plang.points_by_tag and all(p['csv_path'] == ra_map for p in plang.points_by_tag[107]),
          'no points for the skipped group; the planned ones keep the run csv_path')
    plang2 = SR.plan_resume(stem, jt, tm.get_task(jt), tm.scan_points[jt], done, ra_map, {105})
    check(plang2.nothing_left and plang2.groups_done == [105] and plang2.groups_skipped == [106, 107],
          'selecting a finished group only -> nothing left')
    check('selected [107], skipped [106]' in plang.summary() and 'selected' not in plan.summary(),
          'summary names the selection only when there is one')

    print('== 2c. method B pending rows (2026-10-07): captured + numbered points are kept, their Ra entry is owed')
    bstem = f'{jt}_ra_map_20261006_100000'   # older than the 17:56 fixture, so section 4 still picks that one
    b_map = os.path.join(ra_dir, bstem + '.csv')
    b_meas = os.path.join(meas_root, bstem, bstem + '_ra_measured.csv')
    # 105 answered; 106: all three captured and numbered, the e-stop hit at the group's entry stop
    write_ra_map(b_map, j_keys[105] + j_keys[106])
    write_measured(b_meas, j_keys[105] + j_keys[106], pending=j_keys[106])
    bdone = SR.done_points(b_map, b_meas)
    bpend = SR.pending_points(b_meas)
    check(bdone == set(j_keys[105]) | set(j_keys[106]), f'a pending mark row counts as done (no re-shoot): {sorted(bdone)}')
    check(bpend == set(j_keys[106]), f'pending_points = the three numbered rows of 106: {sorted(bpend)}')
    check(SR.pending_points(os.path.join(tmp, 'nope.csv')) == set(), 'no measured CSV -> nothing pending')
    bplan = SR.plan_resume(bstem, jt, tm.get_task(jt), tm.scan_points[jt], bdone, b_map, pending=bpend)
    check([s['tag'] for s in bplan.steps] == [106, 107] and bplan.groups_done == [105] and bplan.groups_kept == [106, 107],
          f'106 is KEPT although every point is done — its Ra entries are owed: {bplan.summary()}')
    b106 = bplan.points_by_tag[106]
    check(len(b106) == len(tm.get_scan_points(jt, 106)) and not any(p['scan'] for p in b106),
          'joint: the whole path of 106 is driven through, nothing scanned')
    check(bplan.n_pending == 3 and bplan.n_remaining == 2 and not bplan.nothing_left
          and '3 captured point(s) await their Ra entry' in bplan.summary(), f'counts: {bplan.summary()}')
    # the same with 107 done too: only the entry stop of 106 is left, and that is NOT "nothing left"
    bdone2 = bdone | set(j_keys[107])
    bplan2 = SR.plan_resume(bstem, jt, tm.get_task(jt), tm.scan_points[jt], bdone2, b_map, pending=bpend)
    check([s['tag'] for s in bplan2.steps] == [106] and bplan2.n_remaining == 0 and bplan2.n_pending == 3
          and not bplan2.nothing_left, f'only the entry stop of 106 left -> still resumable: {bplan2.summary()}')
    bplan3 = SR.plan_resume(bstem, jt, tm.get_task(jt), tm.scan_points[jt], bdone2, b_map, pending=set())
    check(bplan3.nothing_left, 'without pending rows the same done set is complete')
    # half of 106 numbered, the rest open: pending ones scan False, open ones scanned
    bdone4 = set(j_keys[105]) | set(j_keys[106][:2])
    bplan4 = SR.plan_resume(bstem, jt, tm.get_task(jt), tm.scan_points[jt], bdone4, b_map, pending=set(j_keys[106][:2]))
    fl = [(p['point_id'], p['scan']) for p in bplan4.points_by_tag[106] if p['point_id'] in [k[1] for k in j_keys[106]]]
    check(fl == [(j_keys[106][0][1], False), (j_keys[106][1][1], False), (j_keys[106][2][1], True)]
          and bplan4.n_pending == 2 and bplan4.n_remaining == 3, f'mixed group: numbered points through, the open one scanned: {fl}')
    # pose mode: everything of a group captured -> the last done point stays as the one (unscanned) move
    pb_keys = {g: scan_keys(tm.get_scan_points(pt, g)) for g, _ in groups}
    pbplan = SR.plan_resume(f'{pt}_ra_map_20261007_100000', pt, tm.get_task(pt), tm.scan_points[pt],
                            set(pb_keys[105]) | set(pb_keys[106]), '/x/p.csv', pending=set(pb_keys[106]))
    pb106 = pbplan.points_by_tag[106]
    check([s['tag'] for s in pbplan.steps] == [106, 107] and len(pb106) == 1 and pb106[0]['scan'] is False
          and pb106[0]['point_id'] == pb_keys[106][-1][1] and pb106[0]['csv_path'] == '/x/p.csv',
          f'pose: one unscanned move to the last done point carries the run csv_path: {[(p["point_id"], p["scan"]) for p in pb106]}')
    pbplan2 = SR.plan_resume(f'{pt}_ra_map_20261007_100000', pt, tm.get_task(pt), tm.scan_points[pt],
                             set(pb_keys[105]) | set(pb_keys[106][:1]), '/x/p.csv', pending=set(pb_keys[106][:1]))
    check([p['point_id'] for p in pbplan2.points_by_tag[106]] == [k[1] for k in pb_keys[106][1:]],
          'pose, partly done: the pending point is dropped as before (an open point carries the stem)')

    print('== 3. plan_resume, pose mode')
    pstem = f'{pt}_ra_map_20261006_120000'
    p_keys = {g: scan_keys(tm.get_scan_points(pt, g)) for g, _ in groups}
    pdone = set(p_keys[105]) | set(p_keys[106][:1])
    pplan = SR.plan_resume(pstem, pt, tm.get_task(pt), tm.scan_points[pt], pdone, os.path.join(ra_dir, pstem + '.csv'))
    check([s['tag'] for s in pplan.steps] == [106, 107], 'pose: finished group dropped')
    check([p['point_id'] for p in pplan.points_by_tag[106]] == [2, 3], f'pose: done point REMOVED, the rest kept: {[p["point_id"] for p in pplan.points_by_tag[106]]}')
    check(all(p['mode'] == 'pose' and 'x' in p and p['csv_path'].endswith(pstem + '.csv') for p in pplan.points_by_tag[106]),
          'pose points keep their target and get the run csv_path')
    check(pplan.n_done == 5 and pplan.n_remaining == 4, 'pose counts')

    print('== 4. find_resumable')
    newest = f'{jt}_ra_map_20261006_190000'
    write_ra_map(os.path.join(ra_dir, newest + '.csv'), sum(j_keys.values(), []))           # complete
    write_csv(os.path.join(ra_dir, 'notes.csv'), ['a'], [[1]])                              # not a run
    write_ra_map(os.path.join(ra_dir, 'scan_joint_unknown_ra_map_20261006_200000.csv'), [(1, 1)])

    def make_plan(s):
        t = SR.task_of(s)
        if t not in tm.tasks:
            return None
        rp = os.path.join(ra_dir, s + '.csv')
        d = SR.done_points(rp, os.path.join(meas_root, s, s + '_ra_measured.csv'))
        return SR.plan_resume(s, t, tm.get_task(t), tm.scan_points[t], d, rp)
    check(SR.list_runs(ra_dir)[0].endswith('200000') and 'notes' not in ' '.join(SR.list_runs(ra_dir)), 'runs listed newest first, non-run names ignored')
    found = SR.find_resumable(ra_dir, make_plan)
    check(found is not None and found.run_stem == stem,
          f'newest run with points left wins (the complete 19:00 run and the unknown task are passed over): {found and found.run_stem}')
    os.remove(os.path.join(ra_dir, newest + '.csv'))

    print('== 5. task_executor RESUME')
    H.te.RA_MAP_DIR = ra_dir
    H.te.ra_measured_path = lambda s, suffix='_ra_measured.csv', root=None: os.path.join(meas_root, s, s + suffix)
    states = []

    class Arm(H.FakeArm):
        def __init__(self, ex): super().__init__(); self.ex = ex; self.points = []
        def execute_scan_points(self, pts):
            self.points.append(list(pts)); self.calls.append(('scan', len(pts))); self.ex._scan_done_event.set()

    def fresh():
        ex, b = H.make(60.0, docked=False); ex._charge_phase = 'full'
        ex._charge_cfg['return_after_task'] = False
        ex.task_mgr = tm; ex.arm = Arm(ex)
        ex._task_state_pub = types.SimpleNamespace(publish=lambda m: states.append(__import__('json').loads(m.data)))
        ex._pending_resume = None; ex._current_resume = None
        ex._pending_groups = None; ex._current_groups = None
        states.clear(); H.LOG.clear()
        return ex

    ex = fresh()
    ex._command_cb(H._Msg('RESUME'))
    check(ex._pending_task_name == jt and ex._pending_resume and ex._pending_resume['stem'] == stem and ex._stop_requested,
          f'RESUME (no arg) queues the newest interrupted run: {ex._pending_task_name} {ex._pending_resume and ex._pending_resume["stem"]}')
    check(('preempt',) in ex.mobile.calls and 'cancel' in ex.arm.calls, 'RESUME preempts like TASK')
    H.ticks(ex, 4)
    gotos = [c for c in ex.mobile.calls if c[0] == 'goto']
    check(gotos == [('goto', 106), ('goto', 107)], f'base drives to 106 and 107 only, never to the finished 105: {gotos}')
    check(len(ex.arm.points) == 2 and scan_keys(ex.arm.points[0]) == j_keys[106] and scan_keys(ex.arm.points[1]) == j_keys[107],
          'arm gets the remaining groups\' planned rows')
    check(all(p['csv_path'] == ra_map for pts in ex.arm.points for p in pts), "every point sent carries the run's csv_path (no new timestamp)")
    check(any(s.get('note') == f'resumed {stem}' and s.get('resume_run') == stem for s in states), '/task_state says which run is resumed')
    check(ex._current_resume is None and ex._pending_resume is None and ex._task_completed, 'resume cleared after the run; task completed normally')
    check(any('RESUME' in l and '4 of 9 points done' in l for l in H.LOG), 'log line carries the summary')

    ex = fresh()
    ex._command_cb(H._Msg(f'RESUME {stem}'))
    check(ex._pending_resume and ex._pending_resume['stem'] == stem, 'RESUME <stem> picks that run')
    ex = fresh()
    ex._command_cb(H._Msg(f'RESUME {stem}.csv'))
    check(ex._pending_resume and ex._pending_resume['stem'] == stem, 'RESUME <stem>.csv accepted too')
    ex = fresh()
    ex._command_cb(H._Msg('RESUME scan_joint_t_ra_map_20261006_000000'))
    check(ex._pending_task is None and any('no Ra map' in l for l in H.LOG), 'unknown run refused with a log line')
    ex = fresh()
    ex._command_cb(H._Msg('RESUME scan_joint_unknown_ra_map_20261006_200000'))
    check(ex._pending_task is None and any('no registered task' in l for l in H.LOG), 'run of an unregistered task refused')
    ex = fresh()
    write_ra_map(os.path.join(ra_dir, f'{jt}_ra_map_20261006_210000.csv'), sum(j_keys.values(), []))
    ex._command_cb(H._Msg(f'RESUME {jt}_ra_map_20261006_210000'))
    check(ex._pending_task is None and any('nothing left' in l for l in H.LOG), 'a complete run refuses to resume')
    os.remove(os.path.join(ra_dir, f'{jt}_ra_map_20261006_210000.csv'))
    ex = fresh()
    ex._command_cb(H._Msg('RESUME a b'))
    check(ex._pending_task is None and any('Usage' in l for l in H.LOG), 'bad arity refused')

    print('== 5b. task_executor with groups= (2026-10-07)')
    ex = fresh()
    ex._command_cb(H._Msg('RESUME groups=107'))
    check(ex._pending_task_name == jt and ex._pending_resume and ex._pending_resume['stem'] == stem
          and ex._pending_groups == [107], f'RESUME groups=107 queues the newest run with the selection: {ex._pending_groups}')
    H.ticks(ex, 4)
    gotos = [c for c in ex.mobile.calls if c[0] == 'goto']
    check(gotos == [('goto', 107)], f'base drives to 107 only — 105 done, 106 skipped: {gotos}')
    check(len(ex.arm.points) == 1 and scan_keys(ex.arm.points[0]) == j_keys[107]
          and all(p['csv_path'] == ra_map for p in ex.arm.points[0]), "arm gets group 107's rows with the run csv_path")
    check(any(s.get('groups_filter') == [107] and s.get('note', '').endswith('(groups 107)') and s.get('resume_run') == stem for s in states),
          '/task_state carries groups_filter and the note names the groups')
    check(ex._current_groups is None and ex._pending_groups is None, 'selection cleared after the run')
    check(any('skipped, not done' in l for l in H.LOG), 'log says the other groups are skipped, not done')
    # the skipped group is still open afterwards: a plain RESUME plans 106 and 107 again
    done_after = SR.done_points(ra_map, meas)
    check(done_after == set(j_keys[105]), 'nothing new is marked done by the fake run (writer not involved)')
    ex = fresh()
    ex._command_cb(H._Msg(f'RESUME {stem} groups=106'))
    check(ex._pending_resume and ex._pending_groups == [106], 'RESUME <stem> groups=106 accepted')
    ex = fresh()
    ex._command_cb(H._Msg(f'RESUME {stem} groups=999'))
    check(ex._pending_task is None and any('not in this task' in l for l in H.LOG), 'a group the task does not have is refused')
    ex = fresh()
    ex._command_cb(H._Msg(f'RESUME {stem} groups=105'))
    check(ex._pending_task is None and any('nothing left' in l for l in H.LOG), 'selecting only the finished group is refused')
    ex = fresh()
    ex._command_cb(H._Msg('RESUME groups=105'))
    check(ex._pending_task is None and any('nothing left in the selected groups' in l for l in H.LOG),
          'no-arg RESUME with a finished-group selection is refused, not silently re-targeted')
    ex = fresh()
    ex._command_cb(H._Msg(f'RESUME {stem} groups=x'))
    check(ex._pending_task is None and any('bad group id' in l for l in H.LOG), 'malformed groups= refused')

    ex = fresh()
    ex._command_cb(H._Msg(f'TASK {jt} groups=106,107'))
    check(ex._pending_task_name == jt and ex._pending_resume is None and ex._pending_groups == [106, 107]
          and [st['tag'] for st in ex._pending_task] == [106, 107], f'TASK groups=106,107: steps filtered: {[st["tag"] for st in ex._pending_task]}')
    H.ticks(ex, 4)
    gotos = [c for c in ex.mobile.calls if c[0] == 'goto']
    check(gotos == [('goto', 106), ('goto', 107)], f'fresh TASK drives to the selected groups only: {gotos}')
    check(len(ex.arm.points) == 2 and all(p['csv_path'] != ra_map and '_ra_map_' in p['csv_path'] for pts in ex.arm.points for p in pts)
          and all(p['scan'] for pts in ex.arm.points for p in pts if p['point_id'] in (3, 5, 7, 9)),
          'fresh TASK: new csv_path stamp, every work point of the selected groups scanned')
    check(any(s.get('groups_filter') == [106, 107] and s.get('note') == 'task started (groups 106,107)' for s in states),
          '/task_state: groups_filter + note on a fresh TASK')
    check(tm.get_task(jt) and [st['tag'] for st in tm.get_task(jt)] == [105, 106, 107], "TaskManager's own steps untouched")
    ex = fresh()
    ex._command_cb(H._Msg(f'TASK {jt} groups=104'))
    check(ex._pending_task is None and any('not in this task' in l for l in H.LOG), 'TASK with an unknown group refused')
    ex = fresh()
    ex._command_cb(H._Msg(f'TASK {jt} groups=106,107')); ex._command_cb(H._Msg('GOTO 105'))
    check(ex._pending_groups is None and ex._pending_task_name == 'goto_105', 'GOTO after a grouped TASK drops the selection')
    ex = fresh()
    ex._command_cb(H._Msg(f'TASK {jt}'))
    check(ex._pending_groups is None and [st['tag'] for st in ex._pending_task] == [105, 106, 107], 'plain TASK: no selection, all steps')

    # a plain TASK after a pending RESUME drops the resume and runs the whole task on a NEW csv_path
    ex = fresh()
    ex._command_cb(H._Msg('RESUME')); ex._command_cb(H._Msg(f'TASK {jt}'))
    check(ex._pending_resume is None and ex._pending_task_name == jt, 'TASK after RESUME: resume dropped')
    H.ticks(ex, 4)
    gotos = [c for c in ex.mobile.calls if c[0] == 'goto']
    sent = ex.arm.points
    check(gotos == [('goto', 105), ('goto', 106), ('goto', 107)] and len(sent) == 3, 'plain TASK drives every group')
    check(all(p['csv_path'] != ra_map and '_ra_map_' in p['csv_path'] for pts in sent for p in pts)
          and all(p['scan'] for pts in sent for p in pts if p['point_id'] in (3, 5, 7, 9)),
          'plain TASK: fresh csv_path stamp, every work point scanned')
    check(not any(s.get('resume_run') for s in states), 'resume_run None on a plain task')

    print('== 5c. lift per group (2026-10-07): differing groups load, one group disagreeing refuses')
    lift_dir = os.path.join(tmp, 'lift_csv'); os.makedirs(lift_dir)
    rows_l = joint_rows(groups)
    for r in rows_l:
        r[-1] = 0 if r[1] == 107 else 10          # 105 / 106 at 10 mm, 107 at 0
    write_csv(os.path.join(lift_dir, 'joint_l.csv'), JOINT_COLS, rows_l)
    rows_bad = joint_rows(groups); rows_bad[3][-1] = 5
    write_csv(os.path.join(lift_dir, 'joint_bad.csv'), JOINT_COLS, rows_bad)
    H.LOG.clear()
    tml = TaskManager(lift_dir, result_dir=ra_dir)
    check('scan_joint_l' in tml.get_all_task_names() and 'scan_joint_bad' not in tml.get_all_task_names(),
          f'groups at 10 / 10 / 0 register; a group disagreeing with itself is refused: {tml.get_all_task_names()}')
    check(any('across the rows of one group' in l for l in H.LOG), 'the refusal names the group rule')
    check(tml.get_lift_height('scan_joint_l') == 10.0 and tml.lift_heights_by_group['scan_joint_l'] == {105: 10.0, 106: 10.0, 107: 0.0},
          f'task value = the highest, by_group kept: {tml.lift_heights_by_group["scan_joint_l"]}')
    check(tml.get_group_lift_height('scan_joint_l', 107) == 0.0 and tml.get_group_lift_height('scan_joint_l', '106') == 10.0
          and tml.get_group_lift_height('scan_joint_l', 999) == 10.0, 'get_group_lift_height per tag, task value as the fallback')
    info_l = tml.task_info['scan_joint_l']
    check(info_l['lift_height_mm'] == 10.0 and info_l['lift_by_group_mm'] == {'105': 10.0, '106': 10.0, '107': 0.0},
          f"/task_list carries lift_by_group_mm only when they differ: {info_l['lift_by_group_mm']}")
    check(tm.task_info[jt]['lift_by_group_mm'] is None, 'a task whose groups agree has no lift_by_group_mm')
    ex = fresh(); ex.task_mgr = tml
    ex._command_cb(H._Msg('TASK scan_joint_l'))
    H.ticks(ex, 4)
    lifts = ex.lift.calls
    check(lifts == [('goto', 10.0), 'home', 'home'],
          f'lift: 10 mm at the first group, held through 106, origin homing for the 0 mm group (descent), homing at the end: {lifts}')
    check([c for c in ex.mobile.calls if c[0] == 'goto'] == [('goto', 105), ('goto', 106), ('goto', 107)] and ex._task_completed,
          'the task ran all three groups and completed')
    # 0 -> 10 -> 0 : climb is a goto, the descent homes
    rows_l2 = joint_rows(groups)
    for r in rows_l2:
        r[-1] = 10 if r[1] == 106 else 0
    write_csv(os.path.join(lift_dir, 'joint_l.csv'), JOINT_COLS, rows_l2)
    os.remove(os.path.join(lift_dir, 'joint_bad.csv'))
    tml = TaskManager(lift_dir, result_dir=ra_dir)
    ex = fresh(); ex.task_mgr = tml
    ex._command_cb(H._Msg('TASK scan_joint_l'))
    H.ticks(ex, 4)
    check(ex.lift.calls == [('goto', 0.0), ('goto', 10.0), 'home', 'home'],
          f'0 / 10 / 0: goto 0 (origin already), climb to 10, descent = homing only, end homing: {ex.lift.calls}')
    # a grouped TASK on a per-group file sets only the selected group's height
    ex = fresh(); ex.task_mgr = tml
    ex._command_cb(H._Msg('TASK scan_joint_l groups=106'))
    H.ticks(ex, 4)
    check(ex.lift.calls == [('goto', 10.0), 'home'], f'groups=106 alone: lift 10 at 106, homing at the end: {ex.lift.calls}')

    print('== 6. the real 2026-10-06 17:56 run (read-only, if present)')
    real_dir = os.path.join(PKG, 'task', 'csv')
    real_stem = 'scan_joint_hoodouter_lower_plate1_offset50mm_ra_map_20261006_175616'
    ws = os.path.dirname(os.path.dirname(PKG))
    real_map = os.path.join(ws, 'log', 'apriltag_nav', 'ra_maps', real_stem + '.csv')
    real_meas = os.path.join(ws, 'results', 'scan_images', real_stem, real_stem + '_ra_measured.csv')
    if os.path.isfile(real_map) and os.path.isfile(os.path.join(real_dir, 'joint_hoodouter_lower_plate1_offset50mm.csv')):
        rtm = TaskManager(real_dir, result_dir=os.path.dirname(real_map))
        t = SR.task_of(real_stem)
        d = SR.done_points(real_map, real_meas)
        rplan = SR.plan_resume(real_stem, t, rtm.get_task(t), rtm.scan_points[t], d, real_map)
        print('     ' + rplan.summary())
        # the run is being RESUMED on the robot (2026-10-07 morning), so the
        # numbers move: assert the invariants, not the day's counts
        check(105 in rplan.groups_done and sorted(rplan.groups_done + rplan.groups_kept) == [105, 106, 107, 118, 119, 120],
              f'real run: 105 done (first group of the e-stopped run), every group done or kept: {rplan.groups_done} / {rplan.groups_kept}')
        check(rplan.n_done == len(d) and rplan.n_remaining == rplan.n_total - rplan.n_done and rplan.n_total == 1271,
              f'real run: {rplan.n_done} done of 1271, {rplan.n_remaining} left')
        if 106 in rplan.points_by_tag:
            p49 = [p for p in rplan.points_by_tag[106] if p['point_id'] == 49]
            check(p49 and p49[0]['source_point_id'] == 1 and p49[0]['scan'] is ((106, 49) not in d),
                  f"real run: g106 p49 (work point 1) scan flag follows the done set ({(106, 49) in d})")
    else:
        print('     (files not present — skipped)')

    shutil.rmtree(tmp, ignore_errors=True)
    print(f'\n{N["ok"]} ok, {N["fail"]} failed')
    return 1 if N['fail'] else 0


if __name__ == '__main__':
    sys.exit(main())
