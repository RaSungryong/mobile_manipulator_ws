#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offscreen check of the Task tab's /task_list-driven combo (2026-09-14).

Builds the REAL MainWindow against a fake bridge (Qt offscreen, no ROS),
feeds it a /task_list payload shaped like task_executor's, and asserts the
combo, tooltips, detail line, typed-name preservation and the two buttons
(Send TASK / Reload tasks) do what the operator expects.

Run:  QT_QPA_PLATFORM=offscreen python3 tools/check_task_list_ui.py
"""
import os
import sys
import tempfile

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))

from PyQt5.QtCore import QObject, Qt, pyqtSignal      # noqa: E402
from PyQt5.QtTest import QTest                         # noqa: E402
from PyQt5.QtWidgets import QApplication, QPushButton  # noqa: E402

from robot_ui.main_window import MainWindow            # noqa: E402


class FakeBridge(QObject):
    """Signals the window connects to, plus a recorder for every call."""
    image_received = pyqtSignal(str, object)
    arm_state = pyqtSignal(dict)
    task_state = pyqtSignal(dict)
    task_list = pyqtSignal(dict)
    lift_state = pyqtSignal(dict)
    mobile_state = pyqtSignal(dict)
    battery_state = pyqtSignal(dict)
    estop_state = pyqtSignal(bool)
    camera_state = pyqtSignal(str)
    calib_progress = pyqtSignal(dict)
    handeye_progress = pyqtSignal(dict)
    scan_progress = pyqtSignal(dict)
    tag_ids = pyqtSignal(str, object)
    log = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.calls = []

    def __getattr__(self, name):
        if name.startswith('__'):
            raise AttributeError(name)

        def stub(*a, **k):
            self.calls.append((name,) + a)
            return None
        return stub


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


def click(tab, text):
    for b in tab.findChildren(QPushButton):
        if b.text() == text:
            b.click()
            return True
    return False


PAYLOAD = {
    'task_dir': '/x/task/csv',
    'tasks': [
        {'name': 'scan_joint_errorY_p000mm_standoff_010mm_height_652mm',
         'kind': 'scan', 'scan_mode': 'joint', 'source': 'discovered',
         'tags': [104, 105, 106, 107, 118, 119], 'points': 1035,
         'traverse_points': 422, 'lift_height_mm': 0.0,
         'files': ['rrt_final_path_errorY_p000mm_standoff_010mm_height_652mm.csv'],
         'paired_file': 'assigned_workpoints_errorY_p000mm_standoff_010mm_height_652mm.csv',
         'result_name': 'scan_joint_errorY_p000mm_standoff_010mm_height_652mm_ra_map.csv',
         'groups_filter': None, 'points_with_world_xyz': 1035},
        {'name': 'scan_pose_errorY_p000mm_standoff_010mm_height_652mm',
         'kind': 'scan', 'scan_mode': 'pose', 'source': 'discovered',
         'tags': [104, 105, 106, 107, 118, 119], 'points': 1035,
         'traverse_points': 0, 'lift_height_mm': 0.0,
         'files': ['assigned_workpoints_errorY_p000mm_standoff_010mm_height_652mm.csv'],
         'paired_file': 'rrt_final_path_errorY_p000mm_standoff_010mm_height_652mm.csv',
         'result_name': 'scan_pose_errorY_p000mm_standoff_010mm_height_652mm_ra_map.csv',
         'groups_filter': None, 'ik_seeded_points': 1035},
        {'name': 'go_home', 'kind': 'system', 'scan_mode': None,
         'source': 'system', 'tags': [500], 'points': 0, 'traverse_points': 0,
         'lift_height_mm': None, 'files': [], 'paired_file': None,
         'result_name': None, 'groups_filter': None},
    ],
    'stamp': 1.0,
}


def main():
    app = QApplication.instance() or QApplication(sys.argv)
    bridge = FakeBridge()
    plugin_dir = tempfile.mkdtemp(prefix='ui_plugins_')
    save_dir = tempfile.mkdtemp(prefix='ui_saves_')
    win = MainWindow(bridge, plugin_dir=plugin_dir, save_dir=save_dir)
    app.processEvents()
    combo = win.combo_task
    tab = combo.parentWidget()
    while tab is not None and not click.__code__ and False:
        pass
    # the Task tab is the widget holding the combo's group box
    tab = combo.parentWidget().parentWidget()

    print('== before /task_list')
    check([combo.itemText(i) for i in range(combo.count())] == ['go_home'],
          'combo offers only go_home before /task_list arrives')
    check('not received yet' in win.lbl_task_detail.text(),
          f'detail says the list is pending: {win.lbl_task_detail.text()!r}')
    check(any(c[0] == 'replay' for c in bridge.calls),
          'window asked the bridge to replay latched state')

    print('== /task_list arrives')
    logged_before = win.txt_log.toPlainText() if hasattr(win, 'txt_log') else ''
    bridge.task_list.emit(PAYLOAD)
    app.processEvents()
    names = [t['name'] for t in PAYLOAD['tasks']]
    check([combo.itemText(i) for i in range(combo.count())] == names,
          f'combo lists the 3 reported tasks in order')
    tip = combo.itemData(0, Qt.ToolTipRole)
    check(tip == MainWindow.task_summary(PAYLOAD['tasks'][0]),
          f'item tooltip is the task summary: {tip}')
    check('joint' in tip and '1035 pts (+422 traverse)' in tip
          and 'lift 0 mm' in tip and 'paired assigned_workpoints_' in tip,
          'joint summary shows mode, work + traverse counts, lift, pairing')
    check(combo.currentIndex() == 0 and win.lbl_task_detail.text().startswith(
        names[0] + ': joint · tags 104,105,106,107,118,119'),
          f'detail line follows the selection: {win.lbl_task_detail.text()[:60]}…')
    tip2 = MainWindow.task_summary(PAYLOAD['tasks'][2])
    check(tip2 == 'system · tags 500 · lift —', f'go_home summary: {tip2}')

    print('== selection / typed names')
    combo.setCurrentIndex(1)
    app.processEvents()
    check(win.lbl_task_detail.text().startswith(names[1] + ': pose'),
          'selecting the pose task updates the detail line')
    bridge.task_list.emit(PAYLOAD)
    app.processEvents()
    check(combo.currentText() == names[1], 're-publish keeps the current selection')
    combo.setEditText('scan_pose_something_typed')
    app.processEvents()
    check('not in /task_list' in win.lbl_task_detail.text(),
          'a typed unknown name is flagged, not rejected')
    bridge.task_list.emit(PAYLOAD)
    app.processEvents()
    check(combo.currentText() == 'scan_pose_something_typed',
          're-publish preserves a hand-typed name')

    print('== buttons')
    combo.setCurrentIndex(0)
    bridge.calls.clear()
    check(click(tab, 'Send TASK'), 'Send TASK button found')
    check(('send_task_command', f'TASK {names[0]}') in bridge.calls,
          f'Send TASK publishes TASK <selected>: {bridge.calls}')
    bridge.calls.clear()
    check(click(tab, 'Reload tasks'), 'Reload tasks button found')
    check(('send_task_command', 'RELOAD_TASKS') in bridge.calls,
          'Reload tasks publishes RELOAD_TASKS')

    print('== dock & charge buttons + CHARGE chip')
    check(click(tab, 'Dock && charge (tag 500)') and ('send_task_command', 'CHARGE') in bridge.calls,
          'Dock & charge publishes CHARGE')
    check(click(tab, 'Undock / stop charging') and ('send_task_command', 'UNDOCK') in bridge.calls,
          'Undock publishes UNDOCK')
    bridge.task_state.emit({'state': 'IDLE', 'task': None, 'group_index': 0, 'group_total': 0,
                            'charge_phase': 'charging', 'charging': True, 'battery_pct': 57.2})
    app.processEvents()
    check(win.lbl_charge.text() == 'CHARGE charging 57%', f'chip while charging: {win.lbl_charge.text()!r}')
    bridge.task_state.emit({'state': 'IDLE', 'task': None, 'group_index': 0, 'group_total': 0,
                            'charge_phase': 'dock_failed', 'charging': False, 'battery_pct': 30.0})
    app.processEvents()
    check(win.lbl_charge.text() == 'CHARGE DOCK FAILED 30%' and 'BMS charging: no' in win.lbl_charge_detail.text(),
          f'chip on a failed dock: {win.lbl_charge.text()!r}')
    bridge.task_state.emit({'state': 'IDLE', 'task': None, 'group_index': 0, 'group_total': 0})
    app.processEvents()
    check(win.lbl_charge.text() == 'CHARGE DOCK FAILED 30%', 'an old executor without the fields leaves the chip alone')

    print('== /arm/scan_progress -> SCAN chip + log lines')
    ev = [
        {'phase': 'start', 'index': 0, 'total': 468, 'scan_points': 468},
        {'phase': 'move', 'index': 1, 'total': 468, 'point_id': 1, 'group_id': 106, 'scan': True},
        {'phase': 'failed', 'index': 1, 'total': 468, 'point_id': 1, 'group_id': 106, 'scan': True,
         'message': 'IK failed (code 112): target (-1714, 1896, -267) mm is 2.56 m from the arm base (FR10 reach 1.40 m)',
         'n_ok': 0, 'n_fail': 1},
        {'phase': 'move', 'index': 2, 'total': 468, 'point_id': 2, 'group_id': 106, 'scan': True},
        {'phase': 'done', 'index': 2, 'total': 468, 'point_id': 2, 'group_id': 106, 'scan': True,
         'message': 'Success (standoff ok)', 'n_ok': 1, 'n_fail': 1},
        {'phase': 'done', 'index': 3, 'total': 468, 'point_id': 3, 'group_id': 106, 'scan': False,
         'message': 'traverse', 'n_ok': 2, 'n_fail': 1},
        {'phase': 'result', 'index': 2, 'total': 468, 'point_id': 2, 'group_id': 106, 'scan': True,
         'message': 'Success (standoff ok)', 'ra_mean': 0.4123, 'ra_std': 0.0, 'num_samples': 1},
        {'phase': 'result', 'index': 4, 'total': 468, 'point_id': 4, 'group_id': 106, 'scan': True,
         'message': 'Success (no Ra)', 'ra_mean': None, 'ra_std': None, 'num_samples': 0},
        {'phase': 'finished', 'index': 3, 'total': 468, 'n_ok': 2, 'n_fail': 1, 'cancelled': True},
    ]
    texts = []
    for e in ev:
        bridge.scan_progress.emit(e)
        app.processEvents()
        texts.append(win.lbl_scan.text())
    log = win.log_view.toPlainText()
    check(texts[0] == 'SCAN 0/468', f'start -> chip {texts[0]!r}')
    check(texts[1] == 'SCAN 1/468 pt 1', f'move -> chip {texts[1]!r}')
    check(texts[2] == 'SCAN 1/468 ok 0 fail 1', f'failed -> chip {texts[2]!r}')
    check('[scan] 1/468 pt 1 g106: FAIL — IK failed (code 112)' in log
          and '2.56 m from the arm base' in log, 'the failure reason is in the UI log')
    check('[scan] 2/468 pt 2 g106: OK  Success (standoff ok)' in log,
          'a captured point logs OK as soon as its frames are in hand')
    check('[scan] 2/468 pt 2 g106: ra=0.4123' in log, 'its Ra is logged when the result event arrives')
    check('[scan] 4/468 pt 4 g106: no Ra  Success (no Ra)' in log, 'a point whose inference failed says so')
    check(texts[6] == texts[5] == 'SCAN 3/468 ok 2 fail 1', 'result events do not touch the chip')
    check('pt 3 g106' not in log, 'a traverse row does not spam the log')
    check(texts[-1] == 'SCAN done 2 ok / 1 fail (cancelled)', f'finished -> chip {texts[-1]!r}')
    check('[scan] finished (cancelled): 2 ok, 1 failed of 468' in log, 'finished summary logged')

    print('== hand-eye group (Calibration tab)')
    ctab = win.lbl_handeye_nodes.parentWidget()      # the hand-eye QGroupBox
    bridge.calls.clear()
    bridge.handeye_online = lambda: True
    check(click(ctab, 'Auto-sample (sweep)'), 'Auto-sample button found')
    app.processEvents(); QTest.qWait(150); app.processEvents()
    check(('handeye_auto_sample',) in bridge.calls, f'Auto-sample calls handeye_auto_sample {bridge.calls}')
    check(click(ctab, 'Cancel sweep') and (QTest.qWait(150) or True) and ('handeye_cancel',) in bridge.calls,
          'Cancel sweep calls handeye_cancel')
    check(click(ctab, 'Capture here') and (QTest.qWait(150) or True) and ('handeye_capture',) in bridge.calls,
          'Capture here calls handeye_capture')
    check(click(ctab, 'Compute && save T_hc2ee') and (QTest.qWait(150) or True) and ('handeye_compute',) in bridge.calls,
          'Compute calls handeye_compute')
    for e in [{'phase': 'align', 'iteration': 2, 'xy_mm': 4.2, 'tilt_deg': 0.8, 'n_samples': 0},
              {'phase': 'start', 'n_planned': 23, 'n_rejected': 1, 'rejected': {'clearance': 1}, 'n_samples': 0},
              {'phase': 'sample', 'index': 3, 'total': 23, 'label': 'd450 t12 a90 s+30', 'ok': False,
               'reason': 'tag not seen', 'n_captured': 3, 'n_skipped': 1, 'n_samples': 3},
              {'phase': 'finished', 'summary': 'sweep: 20 captured, 3 skipped (tag not seen), 0 move failures of 23 planned', 'n_samples': 20}]:
        bridge.handeye_progress.emit(e)
        app.processEvents()
    log = win.log_view.toPlainText()
    check(win.lbl_handeye_state.text() == 'samples: 20', f'sample count follows progress: {win.lbl_handeye_state.text()!r}')
    check('[handeye] sweep: 23 views planned, 1 rejected' in log and
          '[handeye] view 3/23 d450 t12 a90 s+30: tag not seen' in log and
          '[handeye] sweep: 20 captured' in log, 'sweep start / skipped view / summary reach the log')
    bridge.handeye_online = lambda: False
    bridge.calls.clear()
    click(ctab, 'Auto-sample (sweep)'); app.processEvents()
    check(('handeye_auto_sample',) not in bridge.calls and 'use_handeye_calib:=true' in win.log_view.toPlainText(),
          'with the node offline, Auto-sample refuses and names the launch flag')

    print('== empty list')
    bridge.task_list.emit({'task_dir': '/x', 'tasks': []})
    app.processEvents()
    check(combo.count() == 0, 'an empty /task_list empties the combo')
    check(win.lbl_task_detail.text() == '', 'detail line cleared')

    win.close()
    print(f'\n{N_OK} ok, {N_FAIL} failed')
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
