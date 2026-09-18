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
    lamp_state = pyqtSignal(bool)
    calib_progress = pyqtSignal(dict)
    handeye_progress = pyqtSignal(dict)
    scan_progress = pyqtSignal(dict)
    standoff_state = pyqtSignal(dict)
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
        {'name': 'weird&<name>', 'kind': 'scan', 'scan_mode': 'pose',
         'source': 'explicit', 'tags': [104], 'points': 1,
         'traverse_points': 0, 'lift_height_mm': 0.0,
         'files': ['a&b<c>.csv'], 'paired_file': None,
         'result_name': None, 'groups_filter': [104]},
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
          f'combo lists the {len(names)} reported tasks in order')
    tip = combo.itemData(0, Qt.ToolTipRole)
    check(tip == MainWindow.task_summary(PAYLOAD['tasks'][0]),
          f'item tooltip is the task summary: {tip}')
    check('joint' in tip and '1035 pts (+422 traverse)' in tip
          and 'lift 0 mm' in tip and 'paired assigned_workpoints_' in tip,
          'joint summary shows mode, work + traverse counts, lift, pairing')
    tip2 = MainWindow.task_summary(PAYLOAD['tasks'][2])
    check(tip2 == 'system · tags 500 · lift —', f'go_home summary: {tip2}')

    print('== task detail: field-by-field (2026-09-15), not the old one-liner')
    check(win.lbl_task_detail.textFormat() == Qt.RichText,
          'lbl_task_detail is explicitly RichText, not relying on HTML auto-detection')
    detail = win.lbl_task_detail.text()
    check(combo.currentIndex() == 0 and f'<b>{names[0]}</b>' in detail,
          f'joint task selected first; name shown bold: {detail[:60]}…')
    check('joint mode' in detail and 'MoveJ' in detail,
          'joint task: plain-language mode line, not the raw "joint" enum')
    check('104, 105, 106, 107, 118, 119' in detail and 'drives to each stop' in detail,
          'joint task: tags row spells out the drive order')
    check('1035 scan point' in detail and '+ 422 traverse' in detail
          and 'not scanned' in detail, 'joint task: points row separates scan from traverse')
    check('world x y z for the Ra map' in detail,
          "joint task: paired file names WHY it's paired")
    check('world xyz' in detail and '1035 / 1035 points' in detail,
          'joint task: world-xyz-points row')
    check('⚠️ JOINT PATH REPLAY' in detail and 'reach nor collision' in detail,
          'joint task: reach/collision warning shown next to the task, not only at registration')

    print('== selection / typed names')
    combo.setCurrentIndex(1)
    app.processEvents()
    detail = win.lbl_task_detail.text()
    check(f'<b>{names[1]}</b>' in detail, 'selecting the pose task updates the detail line')
    check('pose mode' in detail and 'IK solved per point' in detail,
          'pose task: plain-language mode line')
    check('IK seed' in detail and 'IK seeded' in detail and '1035 / 1035 points' in detail,
          "pose task: paired file says IK seed, IK-seeded-points row present")
    check('+ 0 traverse' not in detail, 'pose task (0 traverse points): no traverse note')
    check('JOINT PATH REPLAY' not in detail, 'pose task shows no joint warning')

    print('== a task name / file containing < and & is escaped, not injected as markup')
    combo.setCurrentIndex(3)
    app.processEvents()
    detail = win.lbl_task_detail.text()
    check('<script>' not in detail.lower() and '<b>weird&amp;&lt;name&gt;</b>' in detail,
          f'task name escaped: {detail[:80]}…')
    check('a&amp;b&lt;c&gt;.csv' in detail, 'source file name escaped')
    check('104' in detail and 'explicit subset' in detail, 'groups_filter row renders')
    combo.setCurrentIndex(1)
    app.processEvents()
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
    # Basler vision tip group (2026-09-18)
    bridge.basler_tip = lambda cmd, d, so, ex: (bridge.calls.append(('basler_tip', cmd, d, so, tuple(ex))) or
                                                (True, 'vision tip (flange frame): x +3.1  y -257.4  z +230.9 mm\nfit 1.2 mm',
                                                 {'dir': d, 'n_hand': 4, 'n_basler': 6, 'p_tip_mm': [3.1, -257.4, 230.9],
                                                  'psi_deg': 1.75, 'rms_mm': 1.2} if cmd == 'solve' else
                                                 {'dir': d, 'n_hand': 2, 'n_basler': 0}))
    win.txt_bt_dir.setText('/tmp/bt')
    bt = win.txt_bt_dir.parentWidget()      # the Basler-tip QGroupBox
    check(click(bt, 'Capture hand') and (QTest.qWait(300) or True) and ('basler_tip', 'capture_hand', '/tmp/bt', None, ()) in bridge.calls,
          'Capture hand → bridge.basler_tip(capture_hand, dir)')
    app.processEvents()
    check(win.lbl_bt_state.text() == 'hand 2 · basler 0', f'counts follow the reply: {win.lbl_bt_state.text()!r}')
    win.spin_bt_standoff.setValue(17.0)
    check(click(bt, 'Capture Basler') and (QTest.qWait(300) or True) and ('basler_tip', 'capture_basler', '/tmp/bt', 17.0, ()) in bridge.calls,
          'Capture Basler passes the standoff spinbox value')
    win.chk_bt_standoff.setChecked(False)
    check(click(bt, 'Capture Basler') and (QTest.qWait(300) or True) and ('basler_tip', 'capture_basler', '/tmp/bt', None, ()) in bridge.calls,
          'standoff box unticked → no standoff')
    win.txt_bt_exclude.setText('b3 h2')
    check(click(bt, 'Solve') and (QTest.qWait(300) or True) and ('basler_tip', 'solve', '/tmp/bt', None, ('b3', 'h2')) in bridge.calls,
          'Solve passes the excludes')
    app.processEvents()
    check(win.lbl_bt_last.text().startswith('solve: tip (+3.1, -257.4, +230.9) mm, roll +1.75°')
          and '[basler_tip] fit 1.2 mm' in win.log_view.toPlainText() and win.btn_bt_solve.isEnabled(),
          f'solve line, report lines in the log, buttons re-enabled: {win.lbl_bt_last.text()!r}')
    check(click(ctab, 'Compute && save T_hc2ee') and (QTest.qWait(150) or True) and ('handeye_compute',) in bridge.calls,
          'Compute calls handeye_compute')
    for e in [{'phase': 'align', 'iteration': 2, 'xy_mm': 4.2, 'tilt_deg': 0.8, 'n_samples': 0},
              {'phase': 'diverged', 'reason': 'the aiming hand-eye moves the tag the wrong way', 'n_samples': 0},
              {'phase': 'bootstrap', 'index': 0, 'total': 7, 'label': 'start', 'ok': None, 'z_m': 1.2, 'n_samples': 0},
              {'phase': 'bootstrap', 'index': 3, 'total': 7, 'label': 'ry+', 'ok': False, 'reason': 'tag not seen', 'n_samples': 3},
              {'phase': 'bootstrap', 'index': 7, 'total': 7, 'label': 'solved', 'ok': True, 'n_bootstrap': 6,
               't_mm': [26.0, 165.0, -157.0], 'n_samples': 6},
              {'phase': 'start', 'n_planned': 23, 'n_rejected': 1, 'rejected': {'clearance': 1}, 'n_samples': 0,
               'aim_source': 'bootstrap'},
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
    check('moves the tag the wrong way — bootstrapping' in log and
          '[handeye] bootstrap view 3/7 ry+: tag not seen' in log and
          '[handeye] bootstrap: provisional hand-eye from 6 samples, t = (26, 165, -157) mm' in log and
          '(aimed by the bootstrap hand-eye)' in log,
          'divergence / bootstrap skipped view / provisional solve / bootstrap-aimed start reach the log')
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

    print('== distance-sensor assist (Arm tab, 2026-09-15)')
    import time as _time
    arm_tab = win.btn_standoff.parentWidget().parentWidget()
    check('no /arm/standoff_state yet' in win.lbl_standoff.text(),
          'standoff label says no reading before the first state')
    bridge.standoff_state.emit({'raw_mm': -3.0, 'target_mm': 10.0,
                                'sensor_zero_mm': 10.0, 'valid': True,
                                'side': None, 'perp_mm': -2.209,
                                'standoff_mm': 12.209, 'err_mm': -2.209})
    app.processEvents()
    t = win.lbl_standoff.text()
    check(t.startswith('standoff: 12.21 mm') and 'too far' in t and 'raw -3.00' in t,
          f'valid reading rendered as standoff + error side + raw: {t!r}')
    bridge.standoff_state.emit({'raw_mm': 0.1, 'target_mm': 10.0,
                                'sensor_zero_mm': 10.0, 'valid': True,
                                'side': None, 'perp_mm': 0.07,
                                'standoff_mm': 9.93, 'err_mm': 0.07})
    app.processEvents()
    check('ON TARGET' in win.lbl_standoff.text(),
          'inside 0.2 mm reads ON TARGET')
    bridge.standoff_state.emit({'raw_mm': -99999.0, 'target_mm': 10.0,
                                'sensor_zero_mm': 10.0, 'valid': False,
                                'side': 'far', 'perp_mm': None,
                                'standoff_mm': None, 'err_mm': None})
    app.processEvents()
    t = win.lbl_standoff.text()
    check('OUT OF RANGE (too far)' in t and 'jog Z' in t,
          f'sentinel rendered as out of range with the side: {t!r}')
    win._standoff_seen_at = _time.monotonic() - 5.0
    win._age_tag_labels()
    check('no reading for >1.5 s' in win.lbl_standoff.text(),
          'label ages out when the state stops arriving')

    win.spin_standoff_target.setValue(12.5)
    bridge.calls.clear()
    check(win.btn_standoff.isEnabled(), 'Auto standoff enabled at rest')
    check(click(arm_tab, 'Auto standoff'), 'Auto standoff button found on the Arm tab')
    check(not win.btn_standoff.isEnabled(), 'button disabled while the call is in flight')
    t0 = _time.monotonic()
    while _time.monotonic() - t0 < 3.0 and not any(c[0] == 'arm_standoff' for c in bridge.calls):
        app.processEvents(); _time.sleep(0.02)
    t0 = _time.monotonic()
    while _time.monotonic() - t0 < 3.0 and not win.btn_standoff.isEnabled():
        app.processEvents(); _time.sleep(0.02)
    calls = [c for c in bridge.calls if c[0] == 'arm_standoff']
    check(calls == [('arm_standoff', 12.5)],
          f'bridge.arm_standoff called once with the spinbox target: {calls}')
    check(win.btn_standoff.isEnabled(), 'button re-enabled after completion')
    check('standoff -> 12.5 mm' in win.log_view.toPlainText(),
          'log line names the standoff call')
    bridge.calls.clear()
    check(click(arm_tab, 'Cancel'), 'Cancel button found next to Auto standoff')
    check(('arm_cancel',) in bridge.calls, 'Cancel publishes the arm cancel')

    print('== VISION lamp hold (Collect tab, 2026-09-15)')
    bridge.calls.clear()
    check(not win.chk_lamp.isChecked(), 'lamp box starts unticked')
    win.chk_lamp.setChecked(True)
    app.processEvents()
    check(('set_vision_lamp', True) in bridge.calls, 'ticking the box asks the node to hold the lamp')
    bridge.calls.clear()
    bridge.lamp_state.emit(False)          # node refused / device closed
    app.processEvents()
    check(not win.chk_lamp.isChecked(), 'lamp_state false from the node unticks the box')
    check(not any(c[0] == 'set_vision_lamp' for c in bridge.calls),
          'reflecting the node state sends no command back')
    bridge.lamp_state.emit(True)
    app.processEvents()
    check(win.chk_lamp.isChecked(), 'lamp_state true ticks it')
    bridge.calls.clear()
    win._handeye_online = True; win._calib_online = True
    click(win.chk_lamp.parentWidget().parentWidget(), 'STOP ALL') or win._on_stop_all()
    t0 = _time.monotonic()
    while _time.monotonic() - t0 < 2.0 and not all(c in bridge.calls for c in
                                                   [('set_vision_lamp', False), ('handeye_cancel',), ('cancel_map_calibration',)]):
        app.processEvents(); _time.sleep(0.02)
    check(('set_vision_lamp', False) in bridge.calls, 'STOP ALL releases the lamp hold')
    check(('handeye_cancel',) in bridge.calls and ('cancel_map_calibration',) in bridge.calls,
          'STOP ALL cancels a hand-eye sweep and a calibration session (nodes online)')
    win._handeye_online = False; win._calib_online = False

    print('== CAPTURE while the live preview runs (2026-09-15)')
    import numpy as _np
    import threading as _thr
    gate = _thr.Event()
    def slow_capture(n=1, delay=-1.0, use_led=False, timeout=30.0):
        bridge.calls.append(('capture', n, use_led))
        if not use_led:
            gate.wait(2.0)           # a preview grab that is still outstanding
        return True, 'captured 1/1', [_np.zeros((8, 8), _np.uint8)]
    bridge.capture = slow_capture
    win.chk_save.setChecked(False); win.chk_infer.setChecked(False)
    # let the STOP ALL section's pooled calls (mobile_stop, lift_stop) finish
    t0 = _time.monotonic()
    while _time.monotonic() - t0 < 3.0 and win._busy_calls:
        app.processEvents(); _time.sleep(0.02)
    check(win._busy_calls == 0, 'no call in flight before the preview starts')
    bridge.calls.clear()
    win.chk_preview.setChecked(True)
    app.processEvents()
    win._preview_tick()              # one preview grab now in flight (blocked on gate)
    app.processEvents()
    check(win._preview_calls == 1 and win._busy_calls == 1,
          f'a preview grab is in flight (busy {win._busy_calls}, preview {win._preview_calls}, live {len(win._live_workers)})')
    check(win.btn_capture.isEnabled(), 'CAPTURE stays enabled while only a preview grab is in flight')
    check(win._preview_timer.isActive(), 'preview timer running')
    click(win.chk_lamp.parentWidget().parentWidget(), 'CAPTURE')
    check(not win._preview_timer.isActive(), 'capture PAUSES the preview timer (device stays held)')
    check(win.chk_preview.isChecked() and win._preview_on, 'preview is not switched off')
    check(not win.btn_capture.isEnabled(), 'CAPTURE disabled while the real capture runs')
    gate.set()
    t0 = _time.monotonic()
    while _time.monotonic() - t0 < 3.0 and not win._preview_timer.isActive():
        app.processEvents(); _time.sleep(0.02)
    check(win._preview_timer.isActive(), 'preview timer resumes after the capture')
    check(('capture', 1, True) in bridge.calls, 'the real capture went out with the lamp')
    check(not any(c[0] == 'set_camera_active' and c[1] is False for c in bridge.calls),
          'the device was NOT released around the capture')
    win.chk_preview.setChecked(False)
    app.processEvents()

    win.close()
    print(f'\n{N_OK} ok, {N_FAIL} failed')
    return 1 if N_FAIL else 0


if __name__ == '__main__':
    sys.exit(main())
