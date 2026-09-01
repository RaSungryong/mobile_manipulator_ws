#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lift_calib_ui.py — measure the lift's COUNT -> MILLIMETRE relation by hand.

A calibration bench, not an operating tool: drive the lift to a count, put a
height gauge / tape on the arm base, type the number you read into the window,
repeat. It fits a straight line through the samples and reports the
`lifter.mm_per_count` that line implies, next to the one `robot.yaml` currently
carries (0.04976077 = 343.2 mm / 6897 counts, measured 2026-08-14 at the top of
travel — commanded to 6900, settled at 6897, gauge read 343.2 mm).

    python3 tools/lift_calib_ui.py                 # live
    python3 tools/lift_calib_ui.py --csv old.csv   # live, reload old samples
    python3 tools/lift_calib_ui.py --offline old.csv   # review a CSV, no ROS

It runs on the NAVIFRA DRIVER ALONE
-----------------------------------
The only thing that has to be up is `lift_driver` (the `navifra-robot` systemd
unit). `mobile_manipulator.launch` is not needed and **must not be running**:
this tool commands the raw `/lift/*` topics through `NavifraDevices`, and
`lifter_node` is normally the sole writer of those. Two writers on one axis is
exactly the failure `tools/vw_drive.py` guards against on `/cmd_vel`, so this
follows the same rule — it checks `rosnode` for `lifter_node` at startup and
refuses, with `--force` for the case where the name is stale.

⚠️ **That means the guards live here now.** `lifter_node` is what normally
holds the soft travel clamp, and the upper end of the stroke has NO limit
switch — driving past it is a stall, and a stall is what corrupts the count.
So this tool clamps every target and every jog to `lifter.soft_min_counts` ..
`soft_max_counts` (0..6900) itself, and says so on screen. `NavifraDevices`
has no clamp of its own; it will publish whatever it is handed.

The un-homed rule still comes from the driver, which silently ignores absolute
position commands until the lift has homed to its lower limit switch. So `Go`
stays disabled until `homed` is true, and the relative jog — which does work
un-homed — is the only way to move before that.

Why it commands COUNTS and not millimetres
------------------------------------------
There is no millimetre command at the driver level at all: `/lift/position_cmd`
is `Int32` counts. `lifter_node`'s `/lifter/height_cmd` is the only mm entry
point in the system, and it converts with `mm_per_count` — the very number
under test. Commanding mm would therefore measure the scale against itself.
Counts are both the honest axis and the only one available here.

Backlash — why "approach from below" is back, for this tool only
----------------------------------------------------------------
The drive has play, so a count reached by descending is not the same physical
height as that count reached by climbing. `lifter_node` used to route every
descending move through the origin and that rule was deleted on 2026-08-10,
because the task flow cannot produce a descent — every task ends with lift
origin homing and nothing lowers the lift mid-task.

**A calibration sweep is exactly the case that rule guarded.** Stepping back
down to re-measure a point is the most natural thing to do at a bench, and it
would fold the backlash straight into the fitted slope. So the checkbox is on
by default: a target below the current position homes first and then climbs,
and each sample records which way it was approached. The fit uses the ascending
samples only; descending ones are kept, listed and used to report the backlash
instead of being averaged into the answer.

The sample taken at the origin right after homing is the one exception worth
knowing about — the drive is resting *down* against the lower limit switch
there. It is the reference the whole count system is defined by, so it is
included by default, and there is a checkbox to drop it if you would rather
fit only the climb.

What the numbers mean
---------------------
`measured = slope * count + intercept`

  * **slope** is `mm_per_count`.
  * **intercept** depends on what you chose to measure: with *height above
    ground* it is the arm base height at the lift origin (`arm_base_z`,
    0.652 m today); with *travel from the origin* it should come out near 0,
    and a large value means the origin sample is off.

Residuals are the thing to look at before trusting the slope: a clean line and
a 0.3 mm scatter says the scale is real, while a systematic curve says the
gauge is measuring something that is not the stroke.
"""

import argparse
import csv
import os
import queue
import sys
import threading

# --- ROS ------------------------------------------------------------------
# Imported permissively: --offline must work on a machine with no master, and
# a missing rospy should not stop someone reviewing yesterday's CSV.
try:
    import rosnode
    import rospy
    _ROS_IMPORT_ERROR = None
except Exception as _e:                                    # pragma: no cover
    rosnode = None
    rospy = None
    _ROS_IMPORT_ERROR = _e

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QFont
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QMessageBox, QPlainTextEdit,
    QPushButton, QSpinBox, QTableWidget, QTableWidgetItem, QVBoxLayout,
    QWidget)

# The tool lives in tools/, which is not on the path when run directly.
sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))), 'src'))

from apriltag_nav.paths import PKG_DIR, load_yaml_block   # noqa: E402

try:
    from apriltag_nav.navifra_devices import NavifraDevices   # noqa: E402
except Exception:                                          # pragma: no cover
    NavifraDevices = None


# Same figures as lifter_node.DEFAULTS / robot.yaml `lifter:`. Read from the
# config at runtime; these only cover the file being unreadable.
# 343.2 mm at count 6897 is the 2026-08-14 top-of-travel measurement; 6900 is
# the soft ceiling that landing came from.
FALLBACK_SOFT_MAX = 6900
FALLBACK_MM_PER_COUNT = 343.2 / 6897

# The driver settles within navifra.lift_settle_tol (20 counts) of a target.
# 25 leaves a little over that so a normal landing is not called a miss.
ARRIVE_TOL_COUNTS = 25

# The node that owns /lift/* in the running stack. If it is registered, this
# tool would be a second writer on the same axis.
OWNER_NODE = 'lifter_node'

CSV_COLUMNS = ['stamp', 'planned_counts', 'commanded_counts', 'actual_counts',
               'config_mm', 'approach', 'measured_mm', 'measure_mode', 'note']

MODE_ABSOLUTE = 'height_above_ground'
MODE_TRAVEL = 'travel_from_origin'


def owner_node_running(timeout_s=3.0):
    """(state, name) — is lifter_node registered right now?

    `state` is True / False / **None for "could not tell"**, and the third
    case is not theoretical: `rosnode.get_node_names()` is an XML-RPC round
    trip to the master, and a stale registration left behind by a killed node
    is enough to make it hang indefinitely (seen on this master 2026-08-14,
    the `rosnode list` CLI hanging with it). A guard that can hang is worse
    than the collision it guards against, so the query runs on a thread with a
    deadline and an unanswered master downgrades to a warning.
    """
    if rosnode is None:
        return False, None

    out = []

    def query():
        try:
            out.append(rosnode.get_node_names())
        except Exception:
            out.append(None)     # no master yet; init_node will report it

    t = threading.Thread(target=query, daemon=True)
    t.start()
    t.join(float(timeout_s))
    if not out:
        return None, None        # master did not answer in time
    if out[0] is None:
        return False, None
    for name in out[0]:
        if name.strip('/').split('/')[-1] == OWNER_NODE:
            return True, name
    return False, None


# ==============================================================
# ROS side — the navifra driver, and nothing else
# ==============================================================
class LiftBench:
    """The lift command surface this tool needs, over the raw driver.

    Thin on purpose: `NavifraDevices` already owns the topics, the e-stop
    gate and the `_lift_cancel` flag that lets `stop()` release a blocking
    `lift_home()` / `lift_goto()` from another thread. What it does NOT own,
    and what this class adds, is the **soft travel clamp** — that lives in
    `lifter_node`, which is deliberately not running here.
    """

    def __init__(self, navifra_cfg, soft_min, soft_max, mm_per_count):
        self.soft_min = int(soft_min)
        self.soft_max = int(soft_max)
        self.mm_per_count = float(mm_per_count)
        # Before NavifraDevices: it subscribes to /safety/estop in its own
        # constructor and treats the first message as an edge, so _on_estop
        # can fire while that constructor is still running. Observed live
        # 2026-08-14 as "on_estop handler raised: no attribute '_busy'".
        self._busy = None                 # name of the motion in flight
        self.devices = NavifraDevices(navifra_cfg, on_estop=self._on_estop)

    # ---------- state ----------
    def _on_estop(self):
        """Subscriber thread, on every false->true edge. Mirrors lifter_node:
        the PILZ circuit already cut motor power, this makes the software side
        agree so a blocked wait returns instead of sitting out its timeout.

        ⚠️ Gated on a motion actually being in flight, unlike lifter_node's.
        `NavifraDevices` treats the FIRST `/safety/estop` message as an edge
        when it is already true (prev None -> True), so an unconditional
        handler publishes a `stop` to `/lift/command` merely because the tool
        was started while the e-stop was latched — observed 2026-08-14. A
        bench tool that has not been asked to move anything should not write
        to the driver at all.
        """
        if self._busy is None:
            return
        try:
            self.devices.lift_stop()
        except Exception:
            pass

    @property
    def busy(self):
        return self._busy

    def state(self):
        d = self.devices
        pos = d.lift_position
        return {
            'position': pos,
            'config_mm': None if pos is None else pos * self.mm_per_count,
            'homed': d.lift_homed,
            'error': d.lift_error,
            'alarm': d.lift_status,
            'estop': d.estop_active,
            'safety_ok': d.safety_link_ok(),
            'busy': self._busy,
        }

    def wait_for_driver(self, timeout_s=10.0):
        """Block until /lift/position has arrived at least once."""
        deadline = rospy.get_time() + float(timeout_s)
        while rospy.get_time() < deadline and not rospy.is_shutdown():
            if self.devices.lift_position is not None:
                return True, 'lift_driver up'
            rospy.sleep(0.1)
        return False, 'no /lift/position — is the navifra driver running?'

    # ---------- clamp ----------
    def clamp(self, counts):
        """(clamped, was_clamped). The guard lifter_node would normally hold."""
        c = int(counts)
        cl = max(self.soft_min, min(self.soft_max, c))
        return cl, (cl != c)

    # ---------- motion (blocking; call from a worker thread) ----------
    def home(self):
        self._busy = 'lift origin homing'
        try:
            return self.devices.lift_home()
        finally:
            self._busy = None

    def goto(self, counts):
        target, clamped = self.clamp(counts)
        if not self.devices.lift_homed:
            return False, ('lift not homed — the driver silently ignores '
                           'absolute position commands until it has. Home '
                           'first, or use the relative jog.')
        self._busy = f'goto {target}'
        try:
            ok, msg = self.devices.lift_goto(target)
        finally:
            self._busy = None
        if clamped:
            msg += f' (clamped from {int(counts)} to the soft limit)'
        return ok, msg

    def jog(self, delta):
        pos = self.devices.lift_position
        if pos is None:
            return False, 'lift position unknown (lift_driver running?)'
        target, clamped = self.clamp(pos + int(delta))
        applied = target - pos
        if applied == 0:
            return True, f'already at the travel limit ({pos} counts)'
        self._busy = f'jog {applied:+d}'
        try:
            ok, msg = self.devices.lift_jog(applied)
        finally:
            self._busy = None
        if clamped:
            msg += f' (clamped {int(delta):+d} to {applied:+d})'
        return ok, msg

    def stop(self):
        """Halt, and release whatever wait is blocked. Never raises."""
        try:
            self.devices.lift_stop()
        except Exception:
            pass


# ==============================================================
# Fit
# ==============================================================
def linear_fit(xs, ys):
    """Least squares y = a*x + b. Returns (a, b, r2, rms, max_abs_resid)."""
    n = len(xs)
    if n < 2:
        return None
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0:                       # every sample at the same count
        return None
    a = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    b = my - a * mx
    resid = [y - (a * x + b) for x, y in zip(xs, ys)]
    ss_res = sum(r * r for r in resid)
    ss_tot = sum((y - my) ** 2 for y in ys)
    r2 = (1.0 - ss_res / ss_tot) if ss_tot > 0 else float('nan')
    rms = (ss_res / n) ** 0.5
    return a, b, r2, rms, max(abs(r) for r in resid)


# ==============================================================
# UI
# ==============================================================
class LiftCalibWindow(QWidget):

    def __init__(self, offline=False, csv_path=None):
        super().__init__()
        self.setWindowTitle('Lift count <-> millimetre calibration '
                            '(navifra driver only)')
        self.resize(1180, 780)

        cfg = load_yaml_block('lifter')
        self.soft_max = int(cfg.get('soft_max_counts', FALLBACK_SOFT_MAX))
        self.soft_min = int(cfg.get('soft_min_counts', 0))
        self.cfg_mm_per_count = float(cfg.get('mm_per_count',
                                              FALLBACK_MM_PER_COUNT))

        self.offline = offline
        self.lift = None
        self.samples = []          # list of dicts, CSV_COLUMNS keys
        self.plan = []             # planned sweep targets, counts
        self.plan_idx = 0
        self._last_approach = 'unknown'
        self._last_planned = ''
        self._last_commanded = ''
        self._op_result = queue.Queue()
        self._op_thread = None
        self._csv_path = csv_path

        self._build_ui()

        if not offline:
            self._connect_ros()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(200)

        if csv_path and os.path.exists(csv_path):
            self._load_csv(csv_path)
        self._refresh_table()

    # ----------------------------------------------------------
    # construction
    # ----------------------------------------------------------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.addWidget(self._build_state_bar())

        mid = QHBoxLayout()
        mid.addWidget(self._build_move_box(), 4)
        mid.addWidget(self._build_measure_box(), 6)
        root.addLayout(mid, 3)

        root.addWidget(self._build_fit_box(), 2)

    def _chip(self, text):
        lbl = QLabel(text)
        lbl.setStyleSheet('padding:4px 8px; border:1px solid #666; '
                          'border-radius:4px;')
        return lbl

    def _build_state_bar(self):
        box = QGroupBox('navifra lift_driver — /lift/*')
        row = QHBoxLayout(box)
        self.lbl_conn = self._chip('connecting')
        self.lbl_pos = self._chip('count —')
        self.lbl_mm = self._chip('config mm —')
        self.lbl_homed = self._chip('homed —')
        self.lbl_busy = self._chip('idle')
        self.lbl_alarm = self._chip('alarm —')
        for c in (self.lbl_conn, self.lbl_pos, self.lbl_mm, self.lbl_homed,
                  self.lbl_busy, self.lbl_alarm):
            row.addWidget(c)
        row.addStretch(1)
        return box

    def _build_move_box(self):
        box = QGroupBox('Move (commands go out in COUNTS)')
        lay = QVBoxLayout(box)

        note = QLabel(
            'The driver has no millimetre command — /lift/position_cmd is '
            'Int32 counts. Targets and jogs are clamped here to %d..%d, '
            'because lifter_node normally holds that clamp and is not '
            'running: the upper end has no limit switch.'
            % (self.soft_min, self.soft_max))
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        lay.addWidget(note)

        grid = QGridLayout()
        grid.addWidget(QLabel('Target'), 0, 0)
        self.spin_target = QSpinBox()
        self.spin_target.setRange(self.soft_min, self.soft_max)
        self.spin_target.setSingleStep(100)
        self.spin_target.setSuffix(' counts')
        self.spin_target.valueChanged.connect(self._update_target_hint)
        grid.addWidget(self.spin_target, 0, 1)
        self.lbl_target_mm = QLabel('')
        self.lbl_target_mm.setStyleSheet('color:#888;')
        grid.addWidget(self.lbl_target_mm, 0, 2)
        lay.addLayout(grid)

        self.chk_from_below = QCheckBox(
            'Approach every target from below (lift origin homing first when '
            'the target is under the current position)')
        self.chk_from_below.setChecked(True)
        self.chk_from_below.setToolTip(
            'The drive has backlash, so a count reached on the way down is a '
            'different physical height from the same count reached on the way '
            'up. lifter_node dropped this rule because a task never descends; '
            'a calibration sweep does.')
        lay.addWidget(self.chk_from_below)

        row = QHBoxLayout()
        self.btn_go = QPushButton('Go')
        self.btn_go.clicked.connect(self._on_go)
        self.btn_home = QPushButton('Lift origin homing')
        self.btn_home.clicked.connect(self._on_home)
        self.btn_stop = QPushButton('STOP')
        self.btn_stop.setStyleSheet('font-weight:bold; color:#b00;')
        self.btn_stop.clicked.connect(self._on_stop)
        for b in (self.btn_go, self.btn_home, self.btn_stop):
            row.addWidget(b)
        lay.addLayout(row)

        jog = QHBoxLayout()
        jog.addWidget(QLabel('Fine jog'))
        for d in (-50, -10, +10, +50):
            b = QPushButton(f'{d:+d}')
            b.clicked.connect(lambda _c, dd=d: self._on_jog(dd))
            jog.addWidget(b)
        jog.addStretch(1)
        lay.addLayout(jog)
        jog_note = QLabel(
            'Jog is relative (/lift/inc_position_cmd), so it works before '
            'homing — and a downward one leaves the drive loaded the other '
            'way, which the sample is tagged with.')
        jog_note.setWordWrap(True)
        jog_note.setStyleSheet('color:#888;')
        lay.addWidget(jog_note)

        sweep = QGroupBox('Sweep')
        sl = QGridLayout(sweep)
        sl.addWidget(QLabel('Points'), 0, 0)
        self.spin_points = QSpinBox()
        self.spin_points.setRange(2, 30)
        self.spin_points.setValue(8)
        sl.addWidget(self.spin_points, 0, 1)
        btn_plan = QPushButton('Plan 0 .. %d' % self.soft_max)
        btn_plan.clicked.connect(self._on_plan)
        sl.addWidget(btn_plan, 0, 2)
        self.btn_next = QPushButton('Go to next planned point')
        self.btn_next.clicked.connect(self._on_next_point)
        sl.addWidget(self.btn_next, 1, 0, 1, 3)
        self.lbl_plan = QLabel('no sweep planned')
        self.lbl_plan.setWordWrap(True)
        self.lbl_plan.setStyleSheet('color:#888;')
        sl.addWidget(self.lbl_plan, 2, 0, 1, 3)
        lay.addWidget(sweep)

        lay.addStretch(1)
        self._update_target_hint()
        return box

    def _build_measure_box(self):
        box = QGroupBox('Measurements')
        lay = QVBoxLayout(box)

        row = QHBoxLayout()
        row.addWidget(QLabel('Measured'))
        self.spin_measured = QDoubleSpinBox()
        self.spin_measured.setRange(0.0, 3000.0)
        self.spin_measured.setDecimals(2)
        self.spin_measured.setSingleStep(1.0)
        self.spin_measured.setSuffix(' mm')
        self.spin_measured.setMinimumWidth(140)
        row.addWidget(self.spin_measured)
        self.combo_mode = QComboBox()
        self.combo_mode.addItem('height above ground', MODE_ABSOLUTE)
        self.combo_mode.addItem('travel from the lift origin', MODE_TRAVEL)
        row.addWidget(self.combo_mode)
        self.btn_record = QPushButton('Record (Enter)')
        self.btn_record.setDefault(True)
        self.btn_record.clicked.connect(self._on_record)
        row.addWidget(self.btn_record)
        row.addStretch(1)
        lay.addLayout(row)

        self.lbl_record_hint = QLabel('')
        self.lbl_record_hint.setWordWrap(True)
        self.lbl_record_hint.setStyleSheet('color:#888;')
        lay.addWidget(self.lbl_record_hint)

        self.table = QTableWidget(0, 8)
        self.table.setHorizontalHeaderLabels(
            ['planned', 'commanded', 'count', 'config mm', 'dir',
             'measured mm', 'fit resid', 'note'])
        self.table.horizontalHeader().setStretchLastSection(True)
        lay.addWidget(self.table, 1)

        row2 = QHBoxLayout()
        for text, slot in (('Delete selected', self._on_delete),
                           ('Clear all', self._on_clear),
                           ('Save CSV', self._on_save),
                           ('Load CSV', self._on_load)):
            b = QPushButton(text)
            b.clicked.connect(slot)
            row2.addWidget(b)
        row2.addStretch(1)
        lay.addLayout(row2)
        return box

    def _build_fit_box(self):
        box = QGroupBox('Fit')
        lay = QVBoxLayout(box)
        opts = QHBoxLayout()
        self.chk_use_origin = QCheckBox(
            'Include the post-homing origin sample (it rests DOWN on the '
            'limit switch)')
        self.chk_use_origin.setChecked(True)
        self.chk_use_origin.stateChanged.connect(lambda _s: self._refresh_fit())
        opts.addWidget(self.chk_use_origin)
        opts.addStretch(1)
        lay.addLayout(opts)

        self.txt_fit = QPlainTextEdit()
        self.txt_fit.setReadOnly(True)
        f = QFont('monospace')
        f.setStyleHint(QFont.Monospace)
        self.txt_fit.setFont(f)
        lay.addWidget(self.txt_fit)
        return box

    # ----------------------------------------------------------
    # ROS
    # ----------------------------------------------------------
    def _connect_ros(self):
        if rospy is None or NavifraDevices is None:
            QMessageBox.critical(
                self, 'ROS imports failed',
                f'Could not import rospy / NavifraDevices: '
                f'{_ROS_IMPORT_ERROR}\n\nSource the workspace, or use '
                '--offline to review a CSV.')
            self.offline = True
            return
        rospy.init_node('lift_calib_ui', anonymous=True, disable_signals=True)
        self.lift = LiftBench(load_yaml_block('navifra'),
                              self.soft_min, self.soft_max,
                              self.cfg_mm_per_count)

    def _state(self):
        return None if self.lift is None else self.lift.state()

    # ----------------------------------------------------------
    # periodic refresh
    # ----------------------------------------------------------
    def _tick(self):
        st = self._state()
        busy_op = self._op_thread is not None and self._op_thread.is_alive()

        while True:
            try:
                ok, msg = self._op_result.get_nowait()
            except queue.Empty:
                break
            self.lbl_conn.setText(('ok: ' if ok else 'FAILED: ') + msg)
            self.lbl_conn.setStyleSheet(
                'padding:4px 8px; border:1px solid #666; border-radius:4px;'
                + ('' if ok else 'color:#b00;'))

        if self.offline:
            self.lbl_conn.setText('offline — CSV review only')
            for b in (self.btn_go, self.btn_home, self.btn_stop,
                      self.btn_next, self.btn_record):
                b.setEnabled(False)
            return

        pos = st.get('position')
        if pos is None:
            self.lbl_conn.setText(
                'no /lift/position — is the navifra driver running?')
            self.lbl_conn.setStyleSheet(
                'padding:4px 8px; border:1px solid #b00; border-radius:4px; '
                'color:#b00;')
            for b in (self.btn_go, self.btn_home, self.btn_next,
                      self.btn_record):
                b.setEnabled(False)
            return

        mm = st.get('config_mm')
        homed = st.get('homed')
        busy = st.get('busy')
        estop = bool(st.get('estop'))
        alarm = st.get('alarm')
        error = bool(st.get('error'))

        self.lbl_pos.setText('count %s' % pos)
        self.lbl_mm.setText('config mm %s' % ('—' if mm is None else f'{mm:.1f}'))
        self.lbl_homed.setText('homed ' + ('yes' if homed else 'NO'))
        self.lbl_homed.setStyleSheet(
            'padding:4px 8px; border-radius:4px; border:1px solid '
            + ('#666;' if homed else '#b00; color:#b00;'))
        self.lbl_busy.setText(str(busy) if busy else 'idle')
        self.lbl_alarm.setText(
            'E-STOP' if estop else
            ('ALARM: %s' % alarm) if error else
            ('ok' if st.get('safety_ok') else 'ok (safety link unconfirmed)'))
        self.lbl_alarm.setStyleSheet(
            'padding:4px 8px; border-radius:4px; border:1px solid '
            + ('#b00; color:#b00;' if (estop or error) else '#666;'))

        movable = (not busy_op) and (not busy) and (not estop) and (not error)
        self.btn_go.setEnabled(movable and bool(homed))
        self.btn_next.setEnabled(movable and bool(homed) and bool(self.plan))
        self.btn_home.setEnabled(movable)
        self.btn_record.setEnabled(not busy and not busy_op)

        self._update_record_hint(pos)

    def _update_record_hint(self, pos):
        bits = []
        if self._last_commanded != '' and pos is not None:
            try:
                d = int(pos) - int(self._last_commanded)
                if abs(d) > ARRIVE_TOL_COUNTS:
                    bits.append(f'⚠ {d:+d} counts off the last commanded '
                                f'target ({self._last_commanded})')
            except (TypeError, ValueError):
                pass
        bits.append(f'will record count {pos}, approached "{self._last_approach}"')
        self.lbl_record_hint.setText(' · '.join(bits))

    def _update_target_hint(self):
        c = self.spin_target.value()
        self.lbl_target_mm.setText(
            f'= {c * self.cfg_mm_per_count:.1f} mm at the current scale '
            f'({self.cfg_mm_per_count:.7g} mm/count)')

    # ----------------------------------------------------------
    # motion
    # ----------------------------------------------------------
    def _run_op(self, fn, *args):
        if self._op_thread is not None and self._op_thread.is_alive():
            return
        self.lbl_conn.setText('working…')

        def worker():
            try:
                self._op_result.put(fn(*args))
            except Exception as e:
                self._op_result.put((False, f'{type(e).__name__}: {e}'))

        self._op_thread = threading.Thread(target=worker, daemon=True)
        self._op_thread.start()

    def _goto(self, target, planned=''):
        """Home-then-climb when asked, then the absolute move. Worker thread."""
        pos = self.lift.devices.lift_position
        approach = 'up'
        if (self.chk_from_below.isChecked() and pos is not None
                and target < pos - ARRIVE_TOL_COUNTS):
            ok, msg = self.lift.home()
            if not ok:
                return False, f'origin homing before the climb failed: {msg}'
        elif pos is not None and target < pos - ARRIVE_TOL_COUNTS:
            approach = 'down'
        ok, msg = self.lift.goto(target)
        if ok:
            self._last_approach = approach
            self._last_commanded = target
            self._last_planned = planned
        return ok, msg

    def _on_go(self):
        self._run_op(self._goto, self.spin_target.value(), '')

    def _on_home(self):
        def home_op():
            ok, msg = self.lift.home()
            if ok:
                # The origin is reached by descending onto the limit switch.
                self._last_approach = 'home'
                self._last_commanded = 0
                self._last_planned = ''
            return ok, msg
        self._run_op(home_op)

    def _on_stop(self):
        if self.lift is not None:
            self.lift.stop()

    def _on_jog(self, delta):
        if self.lift is None:
            return

        def jog_op():
            ok, msg = self.lift.jog(delta)
            if ok:
                self._last_approach = 'up' if delta > 0 else 'down'
                self._last_commanded = ''
            return ok, msg
        self._run_op(jog_op)

    def _on_plan(self):
        n = self.spin_points.value()
        step = (self.soft_max - self.soft_min) / float(n - 1)
        self.plan = [int(round(self.soft_min + i * step)) for i in range(n)]
        self.plan_idx = 0
        self._show_plan()

    def _show_plan(self):
        if not self.plan:
            self.lbl_plan.setText('no sweep planned')
            return
        parts = []
        for i, c in enumerate(self.plan):
            parts.append(f'[{c}]' if i == self.plan_idx else str(c))
        self.lbl_plan.setText('next in brackets:  ' + '  '.join(parts))

    def _on_next_point(self):
        if not self.plan or self.plan_idx >= len(self.plan):
            return
        target = self.plan[self.plan_idx]
        self._run_op(self._goto, target, str(target))
        self.plan_idx += 1
        self._show_plan()

    # ----------------------------------------------------------
    # samples
    # ----------------------------------------------------------
    def _on_record(self):
        st = self._state()
        if st is None:
            return
        pos = st.get('position')
        if pos is None:
            QMessageBox.warning(self, 'No position',
                                'The lift position is unknown — is the '
                                'navifra lift_driver running?')
            return
        measured = self.spin_measured.value()
        if measured <= 0.0:
            QMessageBox.warning(self, 'No measurement',
                                'Type the value you read off the gauge first.')
            return
        mode = self.combo_mode.currentData()
        if self.samples and self.samples[-1]['measure_mode'] != mode:
            r = QMessageBox.question(
                self, 'Mixed measurement modes',
                'The earlier samples were recorded as "%s" and this one is '
                '"%s". A fit over both is meaningless. Record anyway?'
                % (self.samples[-1]['measure_mode'], mode))
            if r != QMessageBox.Yes:
                return

        self.samples.append({
            'stamp': self._now(),
            'planned_counts': self._last_planned,
            'commanded_counts': self._last_commanded,
            'actual_counts': int(pos),
            'config_mm': (None if st.get('config_mm') is None
                          else round(st['config_mm'], 1)),
            'approach': self._last_approach,
            'measured_mm': measured,
            'measure_mode': mode,
            'note': '',
        })
        self._refresh_table()

    def _now(self):
        if rospy is not None and not self.offline:
            try:
                return f'{rospy.get_time():.1f}'
            except Exception:
                pass
        return ''

    def _on_delete(self):
        rows = sorted({i.row() for i in self.table.selectedIndexes()},
                      reverse=True)
        for r in rows:
            if 0 <= r < len(self.samples):
                del self.samples[r]
        self._refresh_table()

    def _on_clear(self):
        if not self.samples:
            return
        if QMessageBox.question(self, 'Clear all',
                                'Drop all %d samples?' % len(self.samples)
                                ) == QMessageBox.Yes:
            self.samples = []
            self._refresh_table()

    def _on_save(self):
        default = self._csv_path or os.path.join(
            PKG_DIR, 'task', 'csv', 'lift_calibration.csv')
        path, _ = QFileDialog.getSaveFileName(self, 'Save samples', default,
                                              'CSV (*.csv)')
        if not path:
            return
        with open(path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
            w.writeheader()
            for s in self.samples:
                w.writerow(s)
        self._csv_path = path
        self.lbl_conn.setText('saved %d samples to %s'
                              % (len(self.samples), path))

    def _on_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self, 'Load samples',
            self._csv_path or os.path.join(PKG_DIR, 'task', 'csv'),
            'CSV (*.csv)')
        if path:
            self._load_csv(path)

    def _load_csv(self, path):
        try:
            with open(path) as f:
                rows = list(csv.DictReader(f))
        except Exception as e:
            QMessageBox.critical(self, 'Load failed', str(e))
            return
        loaded = []
        for r in rows:
            try:
                # 'node_mm' is what the first version of this tool wrote, when
                # it went through lifter_node. Same number, older name.
                mm = r.get('config_mm', r.get('node_mm'))
                loaded.append({
                    'stamp': r.get('stamp', ''),
                    'planned_counts': r.get('planned_counts', ''),
                    'commanded_counts': r.get('commanded_counts', ''),
                    'actual_counts': int(float(r['actual_counts'])),
                    'config_mm': (float(mm)
                                  if mm not in (None, '', 'None') else None),
                    'approach': r.get('approach', 'unknown'),
                    'measured_mm': float(r['measured_mm']),
                    'measure_mode': r.get('measure_mode', MODE_ABSOLUTE),
                    'note': r.get('note', ''),
                })
            except (KeyError, TypeError, ValueError):
                continue
        self.samples.extend(loaded)
        self._csv_path = path
        self._refresh_table()
        self.lbl_conn.setText('loaded %d samples from %s' % (len(loaded), path))

    # ----------------------------------------------------------
    # table + fit
    # ----------------------------------------------------------
    def _fit_rows(self):
        """The samples the fit uses: ascending only, origin optional."""
        keep = []
        for i, s in enumerate(self.samples):
            if s['approach'] == 'down':
                continue
            if s['approach'] == 'home' and not self.chk_use_origin.isChecked():
                continue
            keep.append(i)
        return keep

    def _refresh_table(self):
        self.table.setRowCount(len(self.samples))
        for r, s in enumerate(self.samples):
            cells = [
                str(s['planned_counts']),
                str(s['commanded_counts']),
                str(s['actual_counts']),
                '—' if s['config_mm'] is None else f"{s['config_mm']:.1f}",
                s['approach'],
                f"{s['measured_mm']:.2f}",
                '',
                s['note'],
            ]
            for c, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if s['approach'] == 'down':
                    item.setForeground(QBrush(QColor(150, 150, 150)))
                self.table.setItem(r, c, item)
        self.table.resizeColumnsToContents()
        self._refresh_fit()

    def _refresh_fit(self):
        idx = self._fit_rows()
        xs = [self.samples[i]['actual_counts'] for i in idx]
        ys = [self.samples[i]['measured_mm'] for i in idx]
        fit = linear_fit(xs, ys)

        # Residual column, blanked for anything outside the fit.
        for r in range(self.table.rowCount()):
            item = self.table.item(r, 6)
            if item is None:
                continue
            if fit is None or r not in idx:
                item.setText('')
            else:
                a, b, _r2, _rms, _mx = fit
                s = self.samples[r]
                item.setText('%+.2f' % (s['measured_mm']
                                        - (a * s['actual_counts'] + b)))

        self.txt_fit.setPlainText(self._fit_report(fit, xs, ys, idx))

    def _fit_report(self, fit, xs, ys, idx):
        n_down = sum(1 for s in self.samples if s['approach'] == 'down')
        n_unknown = sum(1 for s in self.samples if s['approach'] == 'unknown')
        L = []
        L.append('samples: %d in the fit, %d descending (excluded), '
                 '%d of unknown approach'
                 % (len(idx), n_down, n_unknown))
        modes = {s['measure_mode'] for s in self.samples}
        mode = list(modes)[0] if len(modes) == 1 else None
        if len(modes) > 1:
            L.append('⚠ MIXED measurement modes (%s) — the fit below is '
                     'meaningless until they agree.' % ', '.join(sorted(modes)))

        if fit is None:
            L.append('')
            L.append('Need at least two ascending samples at different counts.')
            L.append('Suggested run: origin homing, measure at count 0, then '
                     'climb through the planned sweep measuring at each stop.')
            return '\n'.join(L)

        a, b, r2, rms, mx = fit
        span = max(xs) - min(xs)
        L.append('')
        L.append('  measured_mm = %.7g * count  +  %.2f' % (a, b))
        L.append('')
        L.append('  mm_per_count   fitted %.7g   config %.7g   (%+.2f %%)'
                 % (a, self.cfg_mm_per_count,
                    100.0 * (a - self.cfg_mm_per_count) / self.cfg_mm_per_count))
        if mode == MODE_TRAVEL:
            L.append('  intercept      %+.2f mm at count 0 — should be ~0 for '
                     'travel-from-origin; a large value means the origin '
                     'sample is off' % b)
        else:
            L.append('  intercept      %.2f mm = the measured height at the '
                     'lift origin (arm_base_z is 652 mm today)' % b)
        L.append('  quality        R^2 %.6f   rms %.2f mm   worst %.2f mm '
                 'over a %d-count span' % (r2, rms, mx, span))
        L.append('  full stroke    0 -> %d counts = %.1f mm  (config says '
                 '%.1f mm)' % (self.soft_max, a * self.soft_max,
                               self.cfg_mm_per_count * self.soft_max))
        drift = (a - self.cfg_mm_per_count) * self.soft_max
        L.append('  at the top of the stroke the config scale is off by '
                 '%+.1f mm' % (-drift))

        if len(xs) >= 2:
            i0, i1 = xs.index(min(xs)), xs.index(max(xs))
            if xs[i1] != xs[i0]:
                L.append('  two-point      %.7g mm/count (lowest vs highest '
                         'sample only, as a cross-check)'
                         % ((ys[i1] - ys[i0]) / float(xs[i1] - xs[i0])))

        L.append('')
        L.append(self._backlash_report(a, b))
        L.append('')
        L.append('robot.yaml:')
        L.append('  lifter:')
        L.append('    mm_per_count: %.7g' % a)
        L.append('    mm_calibrated: true')
        if mode == MODE_ABSOLUTE:
            L.append('  arm_calibration:')
            L.append('    arm_base_z: %.4f    # only if the measured value is '
                     'the arm base height' % (b / 1000.0))
            L.append('  # and path_tag_locator/config/extrinsics.yaml T_ab2mb '
                     'tz must become %.4f' % (-b / 1000.0))
        L.append('# lifter_node.py STROKE_MM = %.1f (STROKE_COUNTS %d)'
                 % (a * self.soft_max, self.soft_max))
        return '\n'.join(L)

    def _backlash_report(self, a, b):
        """Descending samples measure backlash rather than the scale."""
        downs = [s for s in self.samples if s['approach'] == 'down']
        if not downs:
            return ('backlash: no descending samples. To measure it, untick '
                    '"approach from below", drive DOWN onto a count already '
                    'measured on the way up, and record it.')
        deltas = [s['measured_mm'] - (a * s['actual_counts'] + b)
                  for s in downs]
        mean = sum(deltas) / len(deltas)
        return ('backlash: %d descending sample(s) sit %+.2f mm from the '
                'ascending line on average (range %+.2f .. %+.2f). That gap '
                'is the play in the drive — it is why a scan height is '
                'reached by homing and climbing.'
                % (len(downs), mean, min(deltas), max(deltas)))

    # ----------------------------------------------------------
    def keyPressEvent(self, ev):
        if ev.key() in (Qt.Key_Return, Qt.Key_Enter):
            if self.btn_record.isEnabled():
                self._on_record()
            return
        super().keyPressEvent(ev)

    def closeEvent(self, ev):
        st = self._state()
        if st is not None and st.get('busy'):
            r = QMessageBox.question(
                self, 'Lift is moving',
                'A lift motion is in flight (%s).\n\nYes: stop it and close.\n'
                'No: close and leave it running.' % st.get('busy'),
                QMessageBox.Yes | QMessageBox.No | QMessageBox.Cancel)
            if r == QMessageBox.Cancel:
                ev.ignore()
                return
            if r == QMessageBox.Yes and self.lift is not None:
                self.lift.stop()
        # Deliberately NOT devices.shutdown(): that darkens the STATUS lamp and
        # drops the charge relay, neither of which this tool owns or touched.
        if rospy is not None and not self.offline:
            try:
                rospy.signal_shutdown('lift_calib_ui closed')
            except Exception:
                pass
        ev.accept()


def main():
    ap = argparse.ArgumentParser(
        description='Bench UI for the lift count <-> millimetre scale. Runs '
                    'against the navifra driver alone; the apriltag_nav stack '
                    'must NOT be up.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--csv', help='load these samples at startup')
    ap.add_argument('--offline', metavar='CSV', nargs='?', const=True,
                    help='review a CSV without connecting to ROS')
    ap.add_argument('--force', action='store_true',
                    help='start even though %s is registered. Only when you '
                         'know the stack is down and the name is stale — two '
                         'writers on /lift/* fight with no error anywhere.'
                         % OWNER_NODE)
    args = ap.parse_args()

    offline = bool(args.offline)
    csv_path = args.csv or (args.offline if isinstance(args.offline, str)
                            else None)

    if not offline:
        if NavifraDevices is None:
            print('navifra_devices could not be imported — source the '
                  'workspace, or pass --offline <csv> to review samples.',
                  file=sys.stderr)
            return 1
        running, name = owner_node_running()
        if running is None:
            print('WARNING: the master did not answer a node list in time, so '
                  f'whether {OWNER_NODE} is running could not be checked. '
                  'Starting anyway — make sure the apriltag_nav stack is '
                  'down, or this is a second writer on /lift/*.',
                  file=sys.stderr)
        elif running and not args.force:
            print(f'{name} is running, so /lift/* already has an owner.\n'
                  'This tool writes those topics directly and would be a '
                  'second writer on one axis — the driver obeys whichever '
                  'message arrived last, with no error anywhere.\n'
                  'Shut the apriltag_nav stack down, or command the lift '
                  'through /lifter/* instead. --force overrides.',
                  file=sys.stderr)
            return 2
        if running:
            print(f'WARNING: --force with {name} registered. Two writers on '
                  '/lift/*.', file=sys.stderr)

    app = QApplication(sys.argv)
    win = LiftCalibWindow(offline=offline, csv_path=csv_path)
    win.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
