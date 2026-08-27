#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MainWindow — operator UI for data collection, manual teaching and task control.

Owns no device. Every button here turns into a topic or a service call on the
node that owns the hardware, via RosBridge. This module does not import rospy,
and it must not start doing so: the moment it can open a camera or an RPC
connection, the second-owner problem this package exists to remove is back.

Layout is built in code with real Qt layout managers, not loaded from a Qt
Designer .ui file. The UI this replaces used absolute widget geometry and then
carried a scale_widgets() routine that repositioned and re-fonted every widget
on each resize — several hundred lines of machinery to reimplement, badly, what
QVBoxLayout does. Layouts also make the window usable on the robot's actual
screen, which is not the one the .ui was drawn on.

Threading
---------
Service calls block for seconds. Every one goes through CallWorker onto a
QThreadPool; button handlers only ever start work and return. Results come back
as Qt signals on the GUI thread.
"""

import os
from datetime import datetime

import cv2
import numpy as np
from PyQt5.QtCore import (QObject, QRunnable, Qt, QThreadPool, QTimer,
                          pyqtSignal, pyqtSlot)
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
                             QFormLayout, QGridLayout, QGroupBox, QHBoxLayout,
                             QLabel, QLineEdit, QMainWindow, QMessageBox,
                             QPlainTextEdit, QPushButton, QSpinBox, QSplitter,
                             QTabWidget, QVBoxLayout, QWidget)

from robot_ui import paths
from robot_ui.image_view import ImageView
from robot_ui.plugin_runner import PluginRunner
from robot_ui.ros_bridge import ARM_AXES, STREAM_CAMERAS


class _WorkerSignals(QObject):
    finished = pyqtSignal(object)
    failed = pyqtSignal(str)


class CallWorker(QRunnable):
    """Runs one blocking bridge call off the GUI thread."""

    def __init__(self, fn, *args, **kwargs):
        super().__init__()
        self._fn = fn
        self._args = args
        self._kwargs = kwargs
        self.signals = _WorkerSignals()

    @pyqtSlot()
    def run(self):
        try:
            self.signals.finished.emit(self._fn(*self._args, **self._kwargs))
        except Exception as e:
            self.signals.failed.emit(f'{type(e).__name__}: {e}')


class MainWindow(QMainWindow):

    # Every log line goes through here so append_log can be called from the
    # plugin worker thread and rospy callback threads. See append_log.
    _log_line = pyqtSignal(str)

    def __init__(self, bridge, plugin_dir=None, save_dir=None):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle('Mobile Manipulator — Data Collection')
        self.resize(1600, 950)

        self._pool = QThreadPool.globalInstance()
        self._views = {}
        self._last_capture = None       # newest captured BGR frame
        self._arm_pose = [0.0] * 6
        self._preview_on = False
        self._busy_calls = 0

        self.plugins = PluginRunner(
            plugin_dir or paths.PLUGIN_DIR, bridge, log=self.append_log)

        self._build_ui(save_dir or paths.DEFAULT_SAVE_DIR)
        self._connect_bridge()

        # Basler preview: the device is normally CLOSED, so a "live" view is a
        # deliberate poll of the capture service rather than a stream. Off at
        # startup — opening the window must not warm up the sensor.
        self._preview_timer = QTimer(self)
        self._preview_timer.timeout.connect(self._preview_tick)

        self.append_log('[UI] ready — this window owns no device; every action '
                        'goes through the node that does.')

    # ==========================================================
    # LAYOUT
    # ==========================================================
    def _build_ui(self, save_dir):
        self._control_panel = self._build_control_panel(save_dir)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._build_view_panel())
        splitter.addWidget(self._control_panel)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        # Explicit initial split, because stretch factors alone let the control
        # column claim width from its widest child (the long explanatory
        # labels) and squeeze the camera pane. The operator can still drag it.
        splitter.setSizes([1080, 520])
        self._splitter = splitter

        central = QWidget()
        outer = QVBoxLayout(central)
        outer.addWidget(self._build_status_bar())
        outer.addWidget(splitter, 1)
        self.setCentralWidget(central)

    def _build_status_bar(self):
        box = QGroupBox('System')
        row = QHBoxLayout(box)
        self.lbl_estop = self._status_chip('E-STOP —')
        self.lbl_battery = self._status_chip('BAT —')
        self.lbl_arm = self._status_chip('ARM —')
        self.lbl_lift = self._status_chip('LIFT —')
        self.lbl_task = self._status_chip('TASK —')
        self.lbl_camera = self._status_chip('CAM —')
        for chip in (self.lbl_estop, self.lbl_battery, self.lbl_arm,
                     self.lbl_lift, self.lbl_task, self.lbl_camera):
            row.addWidget(chip)
        row.addStretch(1)

        # One button that stops everything that can move. Kept at top level and
        # not inside a tab: a stop the operator has to go looking for is not a
        # stop. It is a SOFT stop — the hardware e-stop is a PILZ relay that
        # cuts motor power independently of ROS and is not reachable from here.
        self.btn_stop_all = QPushButton('STOP ALL (soft)')
        self.btn_stop_all.setStyleSheet(
            'background:#b02020; color:white; font-weight:bold; padding:8px;')
        self.btn_stop_all.clicked.connect(self._on_stop_all)
        row.addWidget(self.btn_stop_all)
        return box

    @staticmethod
    def _status_chip(text):
        label = QLabel(text)
        label.setFont(QFont('monospace', 10))
        label.setStyleSheet('padding:4px 10px; border:1px solid #555;')
        return label

    def _build_view_panel(self):
        """Camera area: one big view on top, thumbnails underneath.

        The original grid gave the Basler a near-square cell (~640x600) while
        its frames are 5472x3648, i.e. 3:2. KeepAspectRatio then letterboxed
        29% of the pane away and drew the image at 640x426 — small, and oddly
        placed. The image was never distorted; the CELL was the wrong shape.

        So the main area is a tab stack that spans the full panel width, which
        lands near 3:2 at ordinary window sizes and fills >90% of it. Live
        aiming and reviewing a capture happen at different moments, so they
        share that space rather than splitting it: the capture tab is raised
        automatically when a frame arrives.
        """
        # Basler gets the ROI tool: it is the frame that goes to inference, so
        # it is the only one where a region selection changes a result.
        self._views['basler'] = ImageView(
            'basler (wrist)', roi_enabled=True,
            roi_config=os.path.join(paths.PKG_DIR, 'roi_config.json'))
        # 900 px is what inference_node centre-crops to. Drawing the real crop
        # rather than the reference UI's arbitrary 500 px box means what the
        # operator frames is what the model actually sees.
        self._views['basler'].set_centre_box(900)

        self.view_shot = ImageView('captured')

        shot_page = QWidget()
        shot_layout = QVBoxLayout(shot_page)
        shot_layout.setContentsMargins(0, 0, 0, 0)
        shot_layout.addWidget(self.view_shot, 1)
        self.lbl_ra = QLabel('Ra —')
        self.lbl_ra.setFont(QFont('monospace', 13, QFont.Bold))
        self.lbl_ra.setAlignment(Qt.AlignCenter)
        shot_layout.addWidget(self.lbl_ra)

        self.main_views = QTabWidget()
        self.main_views.addTab(self._views['basler'], 'Basler (live)')
        self.main_views.addTab(shot_page, 'Last capture')

        # Tag cameras: a thumbnail strip. They are for confirming a camera is
        # alive and roughly aimed, not for judging surface texture — double
        # click one to inspect it properly.
        self.thumb_strip = QWidget()
        strip = QHBoxLayout(self.thumb_strip)
        strip.setContentsMargins(0, 0, 0, 0)
        for name in STREAM_CAMERAS:
            self._views[name] = ImageView(name)
            strip.addWidget(self._views[name])

        for view in list(self._views.values()) + [self.view_shot]:
            view.double_clicked.connect(self._on_view_double_clicked)

        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        # 7:2, not 5:1. Measured at 1920x1080 with real frame geometry:
        #
        #   main:strip   Basler          one thumbnail
        #   5:1          1152x768        284x160
        #   7:2          1071x714        380x214     <- here
        #   3:1          1030x687        398x223
        #   5:2           979x653        398x223     <- no gain, pure loss
        #
        # The thumbnails saturate at 398x223 because three of them share the
        # width; past 3:1 the strip only gets taller and 16:9 frames cannot use
        # it. 7:2 buys 95% of that ceiling for 14% of the Basler's area, which
        # is the efficient point. Going further costs the main view for nothing.
        layout.addWidget(self.main_views, 7)
        layout.addWidget(self.thumb_strip, 2)
        return panel

    def _on_view_double_clicked(self, title):
        """Give the image the whole window, or put the controls back.

        Hides the CONTROL PANEL as well as the thumbnail strip, and the reason
        is worth stating because the first attempt hid only the strip and made
        things worse. A Basler frame is 3:2 and the pane is already close to
        that, so it is WIDTH-limited: adding height alone just adds letterbox.
        Measured — hiding only the strip took the fill ratio from 97% down to
        79% while the image stayed exactly 954x636.

        What actually matters when aiming is absolute image size, not fill
        ratio. Taking the control column's ~520 px is what delivers it: at
        1920x1080 the drawn image goes from 1152x768 to roughly 1515x1010, a
        1.7x area gain.
        """
        maximising = self._control_panel.isVisible()
        self.thumb_strip.setVisible(not maximising)
        self._control_panel.setVisible(not maximising)
        if not maximising:
            # Restore the split; a hidden pane collapses to zero and does not
            # come back on its own.
            self._splitter.setSizes([self.width() - 520, 520])
        self.append_log(
            f'[UI] {"maximised" if maximising else "restored"} view ({title})'
            ' — double-click again to toggle')

    def _build_control_panel(self, save_dir):
        # The log widget is built BEFORE the tabs, not after. Tab construction
        # already logs — _build_plugin_tab scans the plugin directory and
        # reports what it found — so a log created afterwards means those first
        # messages hit an attribute that does not exist yet.
        log_box = QGroupBox('Log')
        log_layout = QVBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        self.log_view.setFont(QFont('monospace', 9))
        log_layout.addWidget(self.log_view)
        btn_clear = QPushButton('Clear log')
        btn_clear.clicked.connect(self.log_view.clear)
        log_layout.addWidget(btn_clear)
        # Connected before any tab is built, because tab construction logs.
        self._log_line.connect(self._append_log_gui)

        tabs = QTabWidget()
        tabs.addTab(self._build_collect_tab(save_dir), 'Collect')
        tabs.addTab(self._build_arm_tab(), 'Arm')
        tabs.addTab(self._build_task_tab(), 'Task')
        tabs.addTab(self._build_plugin_tab(), 'Scripts')

        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.addWidget(tabs, 3)
        layout.addWidget(log_box, 2)
        return panel

    # ---------- Collect tab ----------
    def _build_collect_tab(self, save_dir):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        preview_box = QGroupBox('Basler preview')
        preview_layout = QVBoxLayout(preview_box)
        note = QLabel(
            'The Basler is kept closed between captures (heat, sensor life, '
            'and the VISION lamp must only be lit while the shutter is open). '
            'Preview asks basler_camera_node to hold it open and polls the '
            'capture service with the lamp OFF — use it to aim, then turn it '
            'off. The UI never opens the device itself.')
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        preview_layout.addWidget(note)

        row = QHBoxLayout()
        self.chk_preview = QCheckBox('Live preview')
        self.chk_preview.toggled.connect(self._on_preview_toggled)
        row.addWidget(self.chk_preview)
        row.addWidget(QLabel('rate'))
        self.spin_preview_hz = QDoubleSpinBox()
        # 5 Hz is the sensor's own ceiling — an acA5472-5gm is a 5 fps part, so
        # asking for more only queues requests it cannot serve. The timer skips
        # a tick whenever the previous grab is still outstanding, so setting the
        # maximum is safe: it self-limits to whatever the link sustains.
        self.spin_preview_hz.setRange(0.2, 5.0)
        self.spin_preview_hz.setValue(5.0)
        self.spin_preview_hz.setSuffix(' Hz')
        self.spin_preview_hz.setToolTip(
            'Camera maximum is 5 fps (acA5472-5gm). The achieved rate is '
            'lower — each frame is a full-resolution round trip through the '
            'capture service.')
        row.addWidget(self.spin_preview_hz)
        self.spin_preview_hz.valueChanged.connect(self._on_preview_rate)
        row.addStretch(1)
        preview_layout.addLayout(row)
        layout.addWidget(preview_box)

        cap_box = QGroupBox('Capture')
        form = QFormLayout(cap_box)
        self.edit_save_dir = QLineEdit(save_dir)
        browse = QPushButton('Browse…')
        browse.clicked.connect(self._on_browse)
        dir_row = QHBoxLayout()
        dir_row.addWidget(self.edit_save_dir)
        dir_row.addWidget(browse)
        dir_widget = QWidget()
        dir_widget.setLayout(dir_row)
        form.addRow('Save folder', dir_widget)

        self.edit_prefix = QLineEdit('capture')
        form.addRow('File prefix', self.edit_prefix)

        self.spin_samples = QSpinBox()
        self.spin_samples.setRange(1, 20)
        self.spin_samples.setValue(1)
        form.addRow('Frames per shot', self.spin_samples)

        self.chk_led = QCheckBox('VISION lamp on during capture')
        self.chk_led.setChecked(True)
        self.chk_led.setToolTip(
            'Ra inference expects the lamp on. Turn it off only for '
            'diagnostics where ambient light is intended.')
        form.addRow('', self.chk_led)

        self.chk_save = QCheckBox('Save frames to disk')
        self.chk_save.setChecked(True)
        form.addRow('', self.chk_save)

        self.chk_infer = QCheckBox('Predict Ra after capture')
        self.chk_infer.setChecked(True)
        form.addRow('', self.chk_infer)

        self.chk_roi = QCheckBox('Send ROI crop to inference (else full frame)')
        self.chk_roi.setToolTip(
            'Off by default: inference centre-crops 900x900 itself, and a '
            'hand-drawn ROI smaller than that gets zero-padded, which is not '
            'what the model was trained on.')
        form.addRow('', self.chk_roi)

        self.btn_capture = QPushButton('CAPTURE')
        self.btn_capture.setStyleSheet(
            'font-weight:bold; padding:10px; background:#205020; color:white;')
        self.btn_capture.clicked.connect(self._on_capture)
        form.addRow('', self.btn_capture)
        layout.addWidget(cap_box)

        cam_box = QGroupBox('Tag cameras')
        cam_layout = QHBoxLayout(cam_box)
        for name in STREAM_CAMERAS:
            chk = QCheckBox(name)
            chk.setChecked(True)
            chk.toggled.connect(
                lambda on, n=name: self._run(
                    self.bridge.set_stream_camera_enabled, n, on))
            cam_layout.addWidget(chk)
        layout.addWidget(cam_box)

        layout.addStretch(1)
        return tab

    # ---------- Arm tab ----------
    def _build_arm_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        pose_box = QGroupBox('Current TCP pose  (tool = vision_tip)')
        pose_grid = QGridLayout(pose_box)
        self.lbl_pose = {}
        for i, axis in enumerate(ARM_AXES):
            unit = 'mm' if i < 3 else 'deg'
            pose_grid.addWidget(QLabel(f'{axis} [{unit}]'), 0, i)
            value = QLabel('—')
            value.setFont(QFont('monospace', 12))
            value.setAlignment(Qt.AlignCenter)
            value.setStyleSheet('border:1px solid #555; padding:4px;')
            pose_grid.addWidget(value, 1, i)
            self.lbl_pose[axis] = value
        layout.addWidget(pose_box)

        jog_box = QGroupBox('Jog')
        jog_layout = QVBoxLayout(jog_box)
        warn = QLabel('Each press is ONE move of the step size below — it is '
                      'not continuous. Refused while the arm is scanning.')
        warn.setWordWrap(True)
        warn.setStyleSheet('color:#888;')
        jog_layout.addWidget(warn)

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel('step'))
        self.spin_step = QDoubleSpinBox()
        self.spin_step.setRange(0.1, 50.0)
        self.spin_step.setValue(1.0)
        self.spin_step.setDecimals(2)
        step_row.addWidget(self.spin_step)
        step_row.addWidget(QLabel('speed'))
        self.spin_vel = QDoubleSpinBox()
        self.spin_vel.setRange(1.0, 100.0)
        self.spin_vel.setValue(20.0)
        step_row.addWidget(self.spin_vel)
        step_row.addStretch(1)
        jog_layout.addLayout(step_row)

        grid = QGridLayout()
        for col, axis in enumerate(ARM_AXES):
            grid.addWidget(QLabel(axis.upper()), 0, col, Qt.AlignCenter)
            plus = QPushButton('+')
            minus = QPushButton('−')
            plus.clicked.connect(lambda _, a=axis: self._on_jog(a, +1))
            minus.clicked.connect(lambda _, a=axis: self._on_jog(a, -1))
            grid.addWidget(plus, 1, col)
            grid.addWidget(minus, 2, col)
        jog_layout.addLayout(grid)
        layout.addWidget(jog_box)

        move_box = QGroupBox('Absolute move')
        move_layout = QVBoxLayout(move_box)
        target_grid = QGridLayout()
        self.edit_target = {}
        for i, axis in enumerate(ARM_AXES):
            target_grid.addWidget(QLabel(axis), 0, i, Qt.AlignCenter)
            field = QLineEdit()
            field.setPlaceholderText('—')
            target_grid.addWidget(field, 1, i)
            self.edit_target[axis] = field
        move_layout.addLayout(target_grid)

        btn_row = QHBoxLayout()
        btn_fill = QPushButton('Fill from current')
        btn_fill.clicked.connect(self._on_fill_target)
        btn_move = QPushButton('MOVE')
        btn_move.clicked.connect(self._on_move_cart)
        btn_home = QPushButton('Arm home pose')
        btn_home.setToolTip(
            'MoveJ to the stored home joint configuration. This is NOT lift '
            'origin homing — that is a different device (see the Task tab).')
        btn_home.clicked.connect(
            lambda: self._run(self.bridge.arm_home, label='arm home'))
        btn_cancel = QPushButton('Cancel arm motion')
        btn_cancel.clicked.connect(lambda: self.bridge.arm_cancel())
        for b in (btn_fill, btn_move, btn_home, btn_cancel):
            btn_row.addWidget(b)
        move_layout.addLayout(btn_row)
        layout.addWidget(move_box)

        layout.addStretch(1)
        return tab

    # ---------- Task tab ----------
    def _build_task_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        task_box = QGroupBox('Task command')
        task_layout = QVBoxLayout(task_box)
        note = QLabel(
            'A task never returns to the start tag on its own — "scan then '
            'come back" is two commands. Send TASK go_home separately.')
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        task_layout.addWidget(note)

        row = QHBoxLayout()
        self.combo_task = QComboBox()
        self.combo_task.setEditable(True)
        # Names come from task_manager.TASK_DEFS. Editable so an operator can
        # type one this list has not been updated for, rather than being stuck.
        self.combo_task.addItems([
            'scan_joints_line1', 'scan_joints_line2',
            'scan_grid_line1', 'scan_grid_line2',
            'scan_full_joints', 'scan_full_pose',
            'go_home',
        ])
        row.addWidget(self.combo_task, 1)
        btn_task = QPushButton('Send TASK')
        btn_task.clicked.connect(
            lambda: self.bridge.send_task_command(
                f'TASK {self.combo_task.currentText().strip()}'))
        row.addWidget(btn_task)
        task_layout.addLayout(row)

        goto_row = QHBoxLayout()
        goto_row.addWidget(QLabel('GOTO tag'))
        self.spin_tag = QSpinBox()
        self.spin_tag.setRange(0, 999)
        self.spin_tag.setValue(105)
        goto_row.addWidget(self.spin_tag)
        btn_goto = QPushButton('Send GOTO')
        btn_goto.clicked.connect(
            lambda: self.bridge.send_task_command(f'GOTO {self.spin_tag.value()}'))
        goto_row.addWidget(btn_goto)
        goto_row.addStretch(1)
        task_layout.addLayout(goto_row)

        raw_row = QHBoxLayout()
        self.edit_raw_cmd = QLineEdit()
        self.edit_raw_cmd.setPlaceholderText('raw /task_command line, e.g. STATE')
        self.edit_raw_cmd.returnPressed.connect(self._on_send_raw)
        raw_row.addWidget(self.edit_raw_cmd, 1)
        btn_raw = QPushButton('Send')
        btn_raw.clicked.connect(self._on_send_raw)
        raw_row.addWidget(btn_raw)
        task_layout.addLayout(raw_row)
        layout.addWidget(task_box)

        lift_box = QGroupBox('Lift')
        lift_layout = QVBoxLayout(lift_box)
        lift_note = QLabel(
            'Reach a scan height by lift origin homing and then climbing — the '
            'drive has backlash, so a height reached by descending is not the '
            'same physical position. Absolute moves are refused before origin '
            'homing.')
        lift_note.setWordWrap(True)
        lift_note.setStyleSheet('color:#888;')
        lift_layout.addWidget(lift_note)

        lift_row = QHBoxLayout()
        self.spin_lift_mm = QDoubleSpinBox()
        self.spin_lift_mm.setRange(0.0, 341.0)   # soft_max 7000 * 0.0487143
        self.spin_lift_mm.setSuffix(' mm')
        self.spin_lift_mm.setValue(150.0)
        lift_row.addWidget(self.spin_lift_mm)
        btn_lift_go = QPushButton('Go')
        btn_lift_go.clicked.connect(
            lambda: self.bridge.lift_goto_mm(self.spin_lift_mm.value()))
        btn_lift_home = QPushButton('Lift origin homing')
        btn_lift_home.clicked.connect(
            lambda: self._run(self.bridge.lift_home, label='lift origin homing'))
        btn_lift_stop = QPushButton('Stop lift')
        btn_lift_stop.clicked.connect(
            lambda: self._run(self.bridge.lift_stop, label='lift stop'))
        for b in (btn_lift_go, btn_lift_home, btn_lift_stop):
            lift_row.addWidget(b)
        lift_row.addStretch(1)
        lift_layout.addLayout(lift_row)
        layout.addWidget(lift_box)

        layout.addStretch(1)
        return tab

    # ---------- Scripts tab ----------
    def _build_plugin_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        note = QLabel(
            'Scripts are reloaded from disk on every run, so a collection '
            'routine can be edited and re-run without restarting. A script '
            'gets a context object exposing the ROS bridge — it cannot reach '
            'a device handle directly, by design.')
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        layout.addWidget(note)

        row = QHBoxLayout()
        self.combo_plugin = QComboBox()
        row.addWidget(self.combo_plugin, 1)
        btn_refresh = QPushButton('Refresh')
        btn_refresh.clicked.connect(self.refresh_plugins)
        row.addWidget(btn_refresh)
        layout.addLayout(row)

        run_row = QHBoxLayout()
        self.btn_plugin_run = QPushButton('RUN SCRIPT')
        self.btn_plugin_run.clicked.connect(self._on_plugin_run)
        run_row.addWidget(self.btn_plugin_run)
        btn_plugin_stop = QPushButton('Stop script')
        btn_plugin_stop.clicked.connect(self.plugins.cancel)
        run_row.addWidget(btn_plugin_stop)
        layout.addLayout(run_row)

        self.lbl_plugin_dir = QLabel(self.plugins.plugin_dir)
        self.lbl_plugin_dir.setStyleSheet('color:#888;')
        self.lbl_plugin_dir.setWordWrap(True)
        layout.addWidget(self.lbl_plugin_dir)

        layout.addStretch(1)
        self.refresh_plugins()
        return tab

    # ==========================================================
    # BRIDGE WIRING
    # ==========================================================
    def _connect_bridge(self):
        self.bridge.image_received.connect(self._on_image)
        self.bridge.arm_state.connect(self._on_arm_state)
        self.bridge.task_state.connect(self._on_task_state)
        self.bridge.lift_state.connect(self._on_lift_state)
        self.bridge.battery_state.connect(self._on_battery)
        self.bridge.estop_state.connect(self._on_estop)
        self.bridge.camera_state.connect(self._on_camera_state)
        self.bridge.log.connect(self.append_log)

        # Ask for whatever already arrived. The bridge subscribes in its own
        # constructor, which runs before these connects, so a LATCHED topic's
        # retained message was emitted into nothing — latched means "delivered
        # once on connect", not "redelivered later". /task_state and
        # /camera/state are both latched one-shots and were simply missing from
        # the window until something changed them. Also populates every other
        # field immediately rather than after the next periodic update.
        self.bridge.replay()

    def _on_image(self, name, bgr):
        view = self._views.get(name)
        if view is not None:
            view.set_frame(bgr)

    def _on_arm_state(self, state):
        if state['pose_valid']:
            self._arm_pose = state['tcp_pose']
        for i, axis in enumerate(ARM_AXES):
            text = f'{state["tcp_pose"][i]:.2f}' if state['pose_valid'] else '—'
            self.lbl_pose[axis].setText(text)
        flag = 'BUSY' if state['busy'] else state['state'].upper()
        self.lbl_arm.setText(f'ARM {flag}')
        self._tint(self.lbl_arm, '#553311' if state['busy'] else '#1b3a1b')

    def _on_task_state(self, state):
        name = state.get('task') or '—'
        idx, total = state.get('group_index', 0), state.get('group_total', 0)
        progress = f' {idx}/{total}' if total else ''
        self.lbl_task.setText(f'TASK {state.get("state", "?")} {name}{progress}')
        note = state.get('note')
        if note:
            self.append_log(f'[task] {state.get("state")} — {note}')

    def _on_lift_state(self, state):
        height = state.get('height_mm')
        homed = state.get('homed')
        text = 'LIFT '
        text += '—' if height is None else f'{float(height):.0f}mm'
        if homed is False:
            text += ' UNHOMED'
        self.lbl_lift.setText(text)
        self._tint(self.lbl_lift, '#553311' if homed is False else '')

    def _on_battery(self, state):
        pct = state['percentage']
        self.lbl_battery.setText(f'BAT {pct:.0f}% {state["voltage"]:.1f}V')
        # 20% is navifra.low_battery_pct in robot.yaml. Kept in step by hand;
        # the UI has no reader for that file and should not grow one.
        self._tint(self.lbl_battery, '#553311' if pct < 20.0 else '')

    def _on_estop(self, active):
        self.lbl_estop.setText('E-STOP ACTIVE' if active else 'E-STOP clear')
        self._tint(self.lbl_estop, '#b02020' if active else '#1b3a1b')

    def _on_camera_state(self, state):
        self.lbl_camera.setText(f'CAM {state}')

    @staticmethod
    def _tint(label, colour):
        base = 'padding:4px 10px; border:1px solid #555;'
        label.setStyleSheet(base + (f' background:{colour}; color:white;'
                                    if colour else ''))

    # ==========================================================
    # ACTIONS
    # ==========================================================
    def _run(self, fn, *args, label=None, on_done=None, **kwargs):
        """Run a blocking bridge call on the pool and log its result."""
        worker = CallWorker(fn, *args, **kwargs)
        self._busy_calls += 1
        self._update_busy()

        def _finished(result):
            self._busy_calls -= 1
            self._update_busy()
            if label:
                if isinstance(result, tuple) and len(result) >= 2:
                    ok, message = result[0], result[1]
                    self.append_log(f'[{label}] {"ok" if ok else "FAILED"}: '
                                    f'{message}')
                else:
                    self.append_log(f'[{label}] done')
            if on_done is not None:
                on_done(result)

        def _failed(message):
            self._busy_calls -= 1
            self._update_busy()
            self.append_log(f'[{label or "call"}] ERROR: {message}')

        worker.signals.finished.connect(_finished)
        worker.signals.failed.connect(_failed)
        self._pool.start(worker)

    def _update_busy(self):
        self.btn_capture.setEnabled(self._busy_calls == 0)

    def _on_jog(self, axis, sign):
        step = self.spin_step.value() * sign
        self._run(self.bridge.arm_jog, axis, step, self.spin_vel.value(),
                  label=f'jog {axis} {step:+g}')

    def _on_fill_target(self):
        for i, axis in enumerate(ARM_AXES):
            self.edit_target[axis].setText(f'{self._arm_pose[i]:.2f}')

    def _on_move_cart(self):
        pose = []
        for i, axis in enumerate(ARM_AXES):
            text = self.edit_target[axis].text().strip()
            if not text:
                # An empty field means "leave this axis alone", taken from the
                # live pose. Refusing the whole move would make single-axis
                # repositioning need all six typed in every time.
                pose.append(self._arm_pose[i])
                continue
            try:
                pose.append(float(text))
            except ValueError:
                QMessageBox.warning(self, 'Bad target',
                                    f'{axis} is not a number: "{text}"')
                return
        self._run(self.bridge.arm_move_cart, pose, self.spin_vel.value(),
                  label='move_cart')

    def _on_send_raw(self):
        text = self.edit_raw_cmd.text().strip()
        if text and self.bridge.send_task_command(text):
            self.edit_raw_cmd.clear()

    def _on_stop_all(self):
        """Soft-stop every device that can move, plus any running script."""
        self.append_log('[UI] STOP ALL pressed')
        self.plugins.cancel()
        self.bridge.arm_cancel()
        self.bridge.send_task_command('STOP')
        self._run(self.bridge.mobile_stop, label='mobile stop')
        self._run(self.bridge.lift_stop, label='lift stop')
        if self.chk_preview.isChecked():
            self.chk_preview.setChecked(False)

    def _on_browse(self):
        directory = QFileDialog.getExistingDirectory(
            self, 'Save folder', self.edit_save_dir.text())
        if directory:
            self.edit_save_dir.setText(directory)

    # ---------- preview ----------
    def _on_preview_toggled(self, on):
        self._preview_on = on
        if on:
            self.bridge.set_camera_active(True)
            self._preview_timer.start(int(1000 / self.spin_preview_hz.value()))
            self.append_log('[UI] preview on — basler held open, lamp off')
        else:
            self._preview_timer.stop()
            self.bridge.set_camera_active(False)
            self.append_log('[UI] preview off — basler released')

    def _on_preview_rate(self, hz):
        """Apply a rate change without needing the preview toggled off and on."""
        if self._preview_timer.isActive():
            self._preview_timer.start(int(1000 / max(0.2, hz)))

    def _preview_tick(self):
        # Skip rather than queue if the previous grab has not returned: at
        # 2 Hz against a camera that sometimes takes longer, queuing would
        # build an unbounded backlog of stale frames.
        if self._busy_calls > 0:
            return
        self._run(self.bridge.capture, 1, -1.0, False,
                  on_done=self._on_preview_frame)

    def _on_preview_frame(self, result):
        ok, _message, frames = result
        if ok and frames:
            self._views['basler'].set_frame(frames[-1])

    # ---------- capture ----------
    def _on_capture(self):
        if self._preview_on:
            # The lamp state and the open/close cycle both belong to the
            # capture service; letting a preview grab interleave with a real
            # capture would mix lamp-off and lamp-on frames in one shot.
            self.chk_preview.setChecked(False)
        self.append_log('[UI] capture requested')
        self._run(self.bridge.capture, self.spin_samples.value(), -1.0,
                  self.chk_led.isChecked(), on_done=self._on_captured)

    def _on_captured(self, result):
        ok, message, frames = result
        if not ok or not frames:
            self.append_log(f'[capture] FAILED: {message}')
            return
        self.append_log(f'[capture] {len(frames)} frame(s): {message}')
        self._last_capture = frames[-1]
        self._views['basler'].set_frame(frames[-1])
        self.view_shot.set_frame(frames[-1])
        # Raise the capture tab: the operator's attention moves from aiming to
        # judging the shot, and the two views share one area.
        self.main_views.setCurrentIndex(1)

        saved = []
        if self.chk_save.isChecked():
            saved = self._save_frames(frames)

        if self.chk_infer.isChecked():
            image = (self._views['basler'].cropped_roi()
                     if self.chk_roi.isChecked() else frames[-1])
            tag = os.path.basename(saved[-1]) if saved else ''
            self.lbl_ra.setText('Ra … predicting')
            self._run(self.bridge.predict_ra, image, tag,
                      on_done=self._on_ra)

    def _save_frames(self, frames):
        directory = self.edit_save_dir.text().strip() or paths.DEFAULT_SAVE_DIR
        prefix = self.edit_prefix.text().strip() or 'capture'
        try:
            os.makedirs(directory, exist_ok=True)
        except Exception as e:
            self.append_log(f'[save] cannot create {directory}: {e}')
            return []

        # Timestamped names, not a scan-for-the-next-free-index. The reference
        # UI probed capture_1.png, capture_2.png … which is O(n) per shot and
        # silently reuses a number if an old file is deleted mid-session.
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        written = []
        for i, frame in enumerate(frames):
            suffix = f'_{i}' if len(frames) > 1 else ''
            path = os.path.join(directory, f'{prefix}_{stamp}{suffix}.png')
            try:
                if cv2.imwrite(path, frame):
                    written.append(path)
                else:
                    self.append_log(f'[save] imwrite refused {path}')
            except Exception as e:
                self.append_log(f'[save] {path}: {e}')
        if written:
            self.append_log(f'[save] wrote {len(written)} file(s) to {directory}')
        return written

    def _on_ra(self, result):
        if not result['success']:
            self.lbl_ra.setText('Ra — failed')
            self.append_log(f'[inference] FAILED: {result["message"]}')
            return
        parts = ' | '.join(
            f'{name}={value:.4f}' if not np.isnan(value) else f'{name}=NaN'
            for name, value in result['models'])
        self.lbl_ra.setText(f'Ra {result["ra"]:.4f}    ({parts})')
        self.append_log(f'[inference] {parts}  in {result["elapsed_s"]:.2f}s'
                        + (f'  tag={result["tag"]}' if result['tag'] else ''))

    # ---------- plugins ----------
    def refresh_plugins(self):
        names = self.plugins.discover()
        current = self.combo_plugin.currentText()
        self.combo_plugin.clear()
        self.combo_plugin.addItems(names)
        if current in names:
            self.combo_plugin.setCurrentText(current)
        self.append_log(f'[plugin] {len(names)} script(s) available')

    def _on_plugin_run(self):
        name = self.combo_plugin.currentText().strip()
        if not name:
            return
        self.plugins.start(name, on_finished=lambda ok: None)

    # ==========================================================
    # LOG / SHUTDOWN
    # ==========================================================
    def append_log(self, message):
        """Thread-safe log. Callable from any thread.

        It has to be: PluginRunner takes this as a plain callable and calls it
        from the plugin worker thread, and CallWorker error paths can reach it
        off the GUI thread too. QPlainTextEdit.appendPlainText is NOT thread
        safe — touching it cross-thread produces
        "QObject::connect: Cannot queue arguments of type 'QTextCursor'" and
        can corrupt the document. Emitting a signal instead makes Qt queue the
        call onto the GUI thread; a same-thread emit still runs directly, so
        this costs nothing on the common path.
        """
        stamp = datetime.now().strftime('%H:%M:%S')
        self._log_line.emit(f'{stamp}  {message}')

    @pyqtSlot(str)
    def _append_log_gui(self, line):
        self.log_view.appendPlainText(line)

    def closeEvent(self, event):
        self.append_log('[UI] closing')
        self._preview_timer.stop()
        self.plugins.cancel()
        try:
            self.bridge.shutdown()
        except Exception:
            pass
        # Motion is deliberately NOT cancelled: closing a window is not a stop
        # request, and silently aborting a running task would surprise more
        # than it protects. Use STOP ALL for that.
        event.accept()
