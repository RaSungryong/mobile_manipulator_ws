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
import time
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
    # (handler, args): every bridge signal is delivered to its handler
    # through this, so the handler runs on the GUI thread. See _connect_bridge.
    _dispatch = pyqtSignal(object, object)

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
        self._preview_calls = 0
        # Strong refs to in-flight CallWorkers. QThreadPool auto-deletes the
        # C++ QRunnable after run(); without a Python reference the wrapper
        # and its _WorkerSignals could be collected while run() is still
        # emitting ('wrapped C/C++ object ... has been deleted'), the
        # finished slot never fires, and _busy_calls leaks — which greys
        # out CAPTURE for the rest of the session (seen in the offscreen
        # check's startup call, 2026-09-15).
        self._live_workers = set()
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
        self.lbl_charge = self._status_chip('CHARGE —')
        self.lbl_arm = self._status_chip('ARM —')
        self.lbl_lift = self._status_chip('LIFT —')
        self.lbl_mobile = self._status_chip('BASE —')
        self.lbl_task = self._status_chip('TASK —')
        self.lbl_scan = self._status_chip('SCAN —')
        self.lbl_camera = self._status_chip('CAM —')
        for chip in (self.lbl_estop, self.lbl_battery, self.lbl_charge, self.lbl_arm,
                     self.lbl_lift, self.lbl_mobile, self.lbl_task,
                     self.lbl_scan, self.lbl_camera):
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

        # Tag cameras: each pane shows robot_camera_node's tag overlay (IDs,
        # crosshair offset, rpy) by default; the "tags" box swaps it for the
        # raw stream, the "on" box turns the camera + detector off entirely
        # (robot_camera/<name>/set_enabled). A label under each pane lists
        # the IDs in the latest detection frame.
        self.chk_cam_on = {}
        self.chk_cam_tags = {}
        self.lbl_cam_tags = {}
        self._tag_seen_at = {}
        for name in STREAM_CAMERAS:
            self._views[name] = ImageView(name)

        # One MAIN slot and a thumbnail strip; every camera lives in a "cell"
        # (view + its control row) and a single click on any thumbnail moves
        # that cell into the main slot and the previous main cell back down.
        # Cells are moved between layouts rather than views re-created, so a
        # camera keeps its last frame, ROI and checkbox state across swaps.
        self._cells = {'basler': self._make_cell('basler')}
        for name in STREAM_CAMERAS:
            self._cells[name] = self._make_cell(name)

        self._main_slot = QWidget()
        self._main_layout = QVBoxLayout(self._main_slot)
        self._main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_views = QTabWidget()
        self.main_views.addTab(self._main_slot, 'Live')
        self.main_views.addTab(shot_page, 'Last capture')

        self.thumb_strip = QWidget()
        self._strip_layout = QHBoxLayout(self.thumb_strip)
        self._strip_layout.setContentsMargins(0, 0, 0, 0)
        self._main_name = None
        self._select_main('basler')

        for name, view in self._views.items():
            view.clicked.connect(lambda _t, n=name: self._select_main(n))
        for view in list(self._views.values()) + [self.view_shot]:
            view.double_clicked.connect(self._on_view_double_clicked)

        # Age out the per-camera tag list when a camera stops reporting.
        self._tag_age_timer = QTimer(self)
        self._tag_age_timer.timeout.connect(self._age_tag_labels)
        self._tag_age_timer.start(1000)

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

    def _make_cell(self, name):
        """A camera view plus its one-line control row, as one widget."""
        cell = QWidget()
        lay = QVBoxLayout(cell)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(2)
        lay.addWidget(self._views[name], 1)
        row = QHBoxLayout()
        row.setContentsMargins(2, 0, 2, 0)
        if name in STREAM_CAMERAS:
            on = QCheckBox('on')
            on.setChecked(True)
            on.setToolTip(f'/robot_camera/{name}/set_enabled — stops the '
                          'detector AND the vendor stream')
            on.toggled.connect(
                lambda v, n=name: self._run(
                    self.bridge.set_stream_camera_enabled, n, v,
                    label=f'{n} {"on" if v else "off"}'))
            tags = QCheckBox('tags')
            tags.setChecked(True)
            tags.setToolTip('Show the detector overlay (tag ID, offset from '
                            'the optical axis, rpy) instead of the raw image')
            tags.toggled.connect(
                lambda v, n=name: self._run(
                    self.bridge.set_stream_source, n, v,
                    label=f'{n} {"overlay" if v else "raw"}'))
            lbl = QLabel('tags: —')
            lbl.setStyleSheet('color:#aaa; font-family:monospace;')
            row.addWidget(on)
            row.addWidget(tags)
            row.addWidget(lbl, 1)
            self.chk_cam_on[name] = on
            self.chk_cam_tags[name] = tags
            self.lbl_cam_tags[name] = lbl
        else:
            hint = QLabel('wrist camera — drag an ROI, right-click clears')
            hint.setStyleSheet('color:#888;')
            row.addWidget(hint, 1)
        lay.addLayout(row)
        return cell

    def _select_main(self, name):
        """Put camera `name` in the main slot; everything else in the strip."""
        if name == self._main_name or name not in self._cells:
            return
        for cell in self._cells.values():
            for lay in (self._main_layout, self._strip_layout):
                lay.removeWidget(cell)
        self._main_layout.addWidget(self._cells[name], 1)
        for other in self._cells:
            if other != name:
                self._strip_layout.addWidget(self._cells[other], 1)
                self._cells[other].show()
        self._cells[name].show()
        self._main_name = name
        self.main_views.setTabText(0, f'{name} (live)')
        if self.main_views.currentIndex() != 0:
            self.main_views.setCurrentIndex(0)
        self.append_log(f'[UI] main view: {name}')

    def _on_tag_ids(self, cam, ids):
        lbl = self.lbl_cam_tags.get(cam)
        if lbl is None:
            return
        self._tag_seen_at[cam] = time.monotonic()
        lbl.setText('tags: ' + (', '.join(str(i) for i in ids) if ids else '—'))

    def _age_tag_labels(self):
        now = time.monotonic()
        for cam, lbl in self.lbl_cam_tags.items():
            seen = self._tag_seen_at.get(cam)
            if seen is not None and now - seen > 1.5:
                lbl.setText('tags: — (no detections topic)')
                self._tag_seen_at[cam] = None
        seen = getattr(self, '_standoff_seen_at', None)
        if seen is not None and now - seen > 1.5:
            self.lbl_standoff.setText(
                'standoff: —  (no reading for >1.5 s: keyence node / arm_node?)')
            self._tint(self.lbl_standoff, '#553311')
            self._standoff_seen_at = None

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
        tabs.addTab(self._build_mobile_tab(), 'Mobile')
        tabs.addTab(self._build_calibration_tab(), 'Calibration')
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
            'capture service; "VISION lamp" holds the lamp on for aiming and '
            'is released automatically when the camera closes. The UI never '
            'opens the device or drives the lamp itself.')
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        preview_layout.addWidget(note)

        row = QHBoxLayout()
        self.chk_preview = QCheckBox('Live preview')
        self.chk_preview.toggled.connect(self._on_preview_toggled)
        row.addWidget(self.chk_preview)
        # Lamp HOLD for aiming (2026-09-15). The box shows /camera/lamp_state,
        # not its own click: a hold the node refused, or dropped when the
        # device closed, unticks it.
        self.chk_lamp = QCheckBox('VISION lamp')
        self.chk_lamp.setToolTip(
            'Hold the VISION lamp on while aiming (the preview is otherwise '
            'lamp-off). basler_camera_node drives the relay and turns it off '
            'again whenever the camera closes — idle timeout, preview off, '
            'STOP ALL. Captures taken while held use the lamp as it is.')
        self.chk_lamp.toggled.connect(self._on_lamp_toggled)
        row.addWidget(self.chk_lamp)
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

        # Tag-camera on/off moved to the camera panes themselves (2026-09-04),
        # next to the overlay toggle and the detected-ID readout.
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

        # ---- Distance-sensor assist (2026-09-15) ----
        # The Keyence DL-EN1 on the tool reads 0 at its 10 mm zero, negative
        # when too far, positive when too close; arm_node projects the oblique
        # beam and publishes the standoff on /arm/standoff_state. "Auto
        # standoff" runs the scan's own closed loop from wherever the arm is
        # (fresh median readings, capped approach steps, 25 mm travel budget)
        # — the operator jogs roughly into range, then lets the sensor finish.
        so_box = QGroupBox('Distance sensor assist  (Keyence standoff)')
        so_layout = QVBoxLayout(so_box)
        self.lbl_standoff = QLabel('standoff: —  (no /arm/standoff_state yet)')
        self.lbl_standoff.setFont(QFont('monospace', 11))
        self.lbl_standoff.setStyleSheet('border:1px solid #555; padding:4px;')
        so_layout.addWidget(self.lbl_standoff)
        so_row = QHBoxLayout()
        so_row.addWidget(QLabel('target [mm]'))
        self.spin_standoff_target = QDoubleSpinBox()
        self.spin_standoff_target.setRange(1.0, 30.0)
        self.spin_standoff_target.setDecimals(1)
        self.spin_standoff_target.setSingleStep(0.5)
        self.spin_standoff_target.setValue(10.0)
        self.spin_standoff_target.setToolTip(
            'Perpendicular standoff the loop drives to. 10 mm is the sensor '
            'zero and where the Basler focus / Ra model were set; the sensor '
            'measures roughly ±20 mm around it. A value typed here applies '
            'to this button only — TASK scans keep robot.yaml\'s target.')
        so_row.addWidget(self.spin_standoff_target)
        self.btn_standoff = QPushButton('Auto standoff')
        self.btn_standoff.setToolTip(
            'Move the tool along its own Z axis until the Keyence reads the '
            'target standoff. Needs the surface inside the sensor range '
            '(no "out of range" above). Refused while the arm is scanning.')
        self.btn_standoff.clicked.connect(self._on_standoff)
        so_row.addWidget(self.btn_standoff)
        btn_so_cancel = QPushButton('Cancel')
        btn_so_cancel.clicked.connect(lambda: self.bridge.arm_cancel())
        so_row.addWidget(btn_so_cancel)
        so_row.addStretch(1)
        so_layout.addLayout(so_row)
        layout.addWidget(so_box)
        self._standoff_seen_at = None
        self._standoff_inflight = False

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
            'Tasks are the path-data files in task/csv, as task_executor '
            'reports them on /task_list: assigned_workpoints_* (end-effector '
            'poses) → scan_pose_*, rrt_final_path_* (joint angles) → '
            'scan_joint_*. A task never returns to the start tag on its own — '
            '"scan then come back" is two commands. Send TASK go_home '
            'separately.')
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        task_layout.addWidget(note)

        row = QHBoxLayout()
        self.combo_task = QComboBox()
        self.combo_task.setEditable(True)
        # Filled from /task_list (_on_task_list). Until that arrives only the
        # one task that needs no CSV is offered. Editable so an operator can
        # still type a name the executor knows but this list does not show.
        self._task_infos = {}
        self._task_list_seen = False
        self.combo_task.addItem('go_home')
        self.combo_task.lineEdit().setPlaceholderText(
            'waiting for /task_list from task_executor …')
        self.combo_task.currentTextChanged.connect(self._update_task_detail)
        row.addWidget(self.combo_task, 1)
        btn_task = QPushButton('Send TASK')
        btn_task.clicked.connect(
            lambda: self.bridge.send_task_command(
                f'TASK {self.combo_task.currentText().strip()}'))
        row.addWidget(btn_task)
        btn_reload = QPushButton('Reload tasks')
        btn_reload.setToolTip(
            'RELOAD_TASKS: task_executor re-scans task/csv (refused while a '
            'task is running).')
        btn_reload.clicked.connect(
            lambda: self.bridge.send_task_command('RELOAD_TASKS'))
        row.addWidget(btn_reload)
        task_layout.addLayout(row)

        # What the selected task is: mode, tags in order, point counts, lift
        # height, source file(s). Read from /task_list, never guessed.
        self.lbl_task_detail = QLabel('')
        self.lbl_task_detail.setWordWrap(True)
        self.lbl_task_detail.setStyleSheet('color:#aaa; font-family: monospace;')
        task_layout.addWidget(self.lbl_task_detail)
        self._update_task_detail(self.combo_task.currentText())

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

        # Dock + charge (2026-09-14). CHARGE = task_executor's battery_return:
        # lift origin home -> drive to the dock tag (500) -> /crevis/charging
        # true -> wait for BMS current (the charger only starts on that
        # explicit true). UNDOCK = /crevis/charging false -> 0.10 m forward.
        charge_row = QHBoxLayout()
        btn_charge = QPushButton('Dock && charge (tag 500)')
        btn_charge.setToolTip(
            'CHARGE: lift origin home → drive to the dock tag → '
            '/crevis/charging true → wait for the BMS to report current. '
            'Preempts a running task.')
        btn_charge.clicked.connect(
            lambda: self.bridge.send_task_command('CHARGE'))
        charge_row.addWidget(btn_charge)
        btn_undock = QPushButton('Undock / stop charging')
        btn_undock.setToolTip(
            'UNDOCK: /crevis/charging false → drive forward off the dock. '
            'No automatic return until the next task.')
        btn_undock.clicked.connect(
            lambda: self.bridge.send_task_command('UNDOCK'))
        charge_row.addWidget(btn_undock)
        self.lbl_charge_detail = QLabel('')
        self.lbl_charge_detail.setStyleSheet('color:#aaa;')
        charge_row.addWidget(self.lbl_charge_detail, 1)
        task_layout.addLayout(charge_row)

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

    # ---------- Mobile tab ----------
    def _build_mobile_tab(self):
        tab = QWidget()
        layout = QVBoxLayout(tab)

        box = QGroupBox('Manual move  (odometry-closed, no tag)')
        box_layout = QVBoxLayout(box)
        note = QLabel(
            'Drives the base a fixed distance or pivots a fixed angle through '
            'mobile_node (/mobile/move_cmd) — trapezoid profile, closed on '
            '/odom, aborts if odom stalls. Refused while a GOTO/TASK or a '
            'calibration session is driving. After STOP ALL the base is '
            'emergency-latched: press "Clear stop latch" before the next move.')
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        box_layout.addWidget(note)

        # --- straight line ---
        line = QHBoxLayout()
        line.addWidget(QLabel('Distance'))
        self.spin_mob_dist = QDoubleSpinBox()
        self.spin_mob_dist.setRange(0.005, 5.0)
        self.spin_mob_dist.setDecimals(3)
        self.spin_mob_dist.setSingleStep(0.05)
        self.spin_mob_dist.setSuffix(' m')
        self.spin_mob_dist.setValue(0.300)
        line.addWidget(self.spin_mob_dist)
        line.addWidget(QLabel('speed'))
        self.spin_mob_v = QDoubleSpinBox()
        self.spin_mob_v.setRange(0.005, 0.5)
        self.spin_mob_v.setDecimals(3)
        self.spin_mob_v.setSingleStep(0.005)
        self.spin_mob_v.setSuffix(' m/s')
        self.spin_mob_v.setValue(0.05)
        self.spin_mob_v.setToolTip('Capped by robot.yaml max_linear_speed')
        line.addWidget(self.spin_mob_v)
        self.btn_mob_fwd = QPushButton('▲ Forward')
        self.btn_mob_fwd.clicked.connect(lambda: self._on_mobile_drive(+1))
        self.btn_mob_rev = QPushButton('▼ Reverse')
        self.btn_mob_rev.clicked.connect(lambda: self._on_mobile_drive(-1))
        line.addWidget(self.btn_mob_fwd)
        line.addWidget(self.btn_mob_rev)
        line.addStretch(1)
        box_layout.addLayout(line)

        # --- pivot ---
        turn = QHBoxLayout()
        turn.addWidget(QLabel('Angle'))
        self.spin_mob_angle = QDoubleSpinBox()
        self.spin_mob_angle.setRange(0.5, 360.0)
        self.spin_mob_angle.setDecimals(1)
        self.spin_mob_angle.setSingleStep(5.0)
        self.spin_mob_angle.setSuffix(' °')
        self.spin_mob_angle.setValue(90.0)
        turn.addWidget(self.spin_mob_angle)
        turn.addWidget(QLabel('speed'))
        self.spin_mob_w = QDoubleSpinBox()
        self.spin_mob_w.setRange(0.01, 1.0)
        self.spin_mob_w.setDecimals(3)
        self.spin_mob_w.setSingleStep(0.01)
        self.spin_mob_w.setSuffix(' rad/s')
        self.spin_mob_w.setValue(0.20)
        self.spin_mob_w.setToolTip('Capped by robot.yaml max_angular_speed')
        turn.addWidget(self.spin_mob_w)
        self.btn_mob_ccw = QPushButton('↺ Turn left (CCW)')
        self.btn_mob_ccw.clicked.connect(lambda: self._on_mobile_pivot(+1))
        self.btn_mob_cw = QPushButton('↻ Turn right (CW)')
        self.btn_mob_cw.clicked.connect(lambda: self._on_mobile_pivot(-1))
        turn.addWidget(self.btn_mob_ccw)
        turn.addWidget(self.btn_mob_cw)
        turn.addStretch(1)
        box_layout.addLayout(turn)

        # --- stop / latch ---
        ctl = QHBoxLayout()
        btn_cancel = QPushButton('Stop base')
        btn_cancel.setStyleSheet('background:#8a4a00; color:white; '
                                 'font-weight:bold;')
        btn_cancel.clicked.connect(
            lambda: self._run(self.bridge.mobile_cancel, label='base stop'))
        btn_clear = QPushButton('Clear stop latch')
        btn_clear.clicked.connect(
            lambda: self._run(self.bridge.mobile_clear_stop,
                              label='base clear_stop'))
        ctl.addWidget(btn_cancel)
        ctl.addWidget(btn_clear)
        ctl.addStretch(1)
        box_layout.addLayout(ctl)

        self.lbl_mob_status = QLabel('base: —')
        self.lbl_mob_status.setStyleSheet('color:#888;')
        self.lbl_mob_status.setWordWrap(True)
        box_layout.addWidget(self.lbl_mob_status)
        layout.addWidget(box)
        layout.addStretch(1)

        self._mobile_buttons = (self.btn_mob_fwd, self.btn_mob_rev,
                                self.btn_mob_ccw, self.btn_mob_cw)
        self._mobile_move_inflight = False
        return tab

    # ---------- Scripts tab ----------
    def _build_calibration_tab(self):
        """Tag-map calibration panel — drives path_tag_locator's nodes.

        Needs `roslaunch path_tag_locator path_tag_locator.launch`
        running alongside the main stack; without it every button lands
        in a clean wait_for_service timeout. Plan + reference files are
        switched TOGETHER by the plate selector (same ids on both
        plates — a mixed pair shifts every result by metres).
        """
        tab = QWidget()
        layout = QVBoxLayout(tab)

        box = QGroupBox('Map calibration session')
        form = QVBoxLayout(box)
        note = QLabel(
            'Session drives the base via /mobile/goto_tag — send no '
            'TASK/GOTO while it runs. Always dry-run a new plan first.')
        note.setWordWrap(True)
        note.setStyleSheet('color:#888;')
        form.addWidget(note)

        row = QHBoxLayout()
        row.addWidget(QLabel('Plan:'))
        self.combo_calib_plate = QComboBox()
        self.combo_calib_plate.addItems(
            ['정반 1  (zones B+C, 26 tags)',
             '정반 2  (zones D+E, 25 tags)',
             '정반 1  YAW SWEEP  (hand-eye vs front_cam, 6 entries)',
             '정반 2  YAW SWEEP  (hand-eye vs front_cam, 6 entries)'])
        self.combo_calib_plate.currentIndexChanged.connect(
            self._refresh_calib_plan_note)
        row.addWidget(self.combo_calib_plate, 1)
        self.chk_calib_dry = QCheckBox('dry run')
        self.chk_calib_dry.setChecked(True)
        row.addWidget(self.chk_calib_dry)
        form.addLayout(row)

        # The diagnostic plans are NOT calibration runs — same service,
        # same buttons, completely different meaning — so say so where the
        # operator is looking rather than only in the doc.
        self.lbl_calib_plan_note = QLabel('')
        self.lbl_calib_plan_note.setWordWrap(True)
        form.addWidget(self.lbl_calib_plan_note)

        self.lbl_calib_nodes = QLabel('nodes: checking…')
        form.addWidget(self.lbl_calib_nodes)

        row = QHBoxLayout()
        self.btn_calib_start = QPushButton('START SESSION')
        self.btn_calib_start.clicked.connect(self._on_calib_start)
        row.addWidget(self.btn_calib_start)
        btn_cancel = QPushButton('Cancel (after current tag)')
        btn_cancel.clicked.connect(
            lambda: self._run(self.bridge.cancel_map_calibration,
                              label='calib-cancel'))
        row.addWidget(btn_cancel)
        form.addLayout(row)

        self.lbl_calib_state = QLabel('idle')
        self.lbl_calib_state.setStyleSheet('font-weight:bold;')
        form.addWidget(self.lbl_calib_state)
        self.lbl_calib_counts = QLabel('ok 0   fail 0   degraded 0')
        form.addWidget(self.lbl_calib_counts)
        self.lbl_calib_last = QLabel('—')
        self.lbl_calib_last.setWordWrap(True)
        self.lbl_calib_last.setStyleSheet('color:#888;')
        form.addWidget(self.lbl_calib_last)
        layout.addWidget(box)

        # Hand-eye calibration (2026-09-14): hand_cam <-> FLANGE (what
        # /arm/state reports). Needs the launch flag use_handeye_calib:=true.
        he_box = QGroupBox('Hand-eye calibration (hand_cam ↔ flange, tag 0)')
        he_layout = QVBoxLayout(he_box)
        he_note = QLabel(
            'Base on tag 102/103, arm driven so hand_cam sees cross tag 0 '
            '(Arm tab). Auto-sample squares up on the tag and orbits it '
            '(tilts, spins, 3 distances) capturing at every view, then '
            'returns; or capture by hand pose by pose. Compute writes '
            'config/hand_eye/T_hc2ee.npz — restart the calibration nodes '
            'afterwards and regenerate the plans.')
        he_note.setWordWrap(True)
        he_note.setStyleSheet('color:#888;')
        he_layout.addWidget(he_note)
        self.lbl_handeye_nodes = QLabel('hand-eye node: checking…')
        he_layout.addWidget(self.lbl_handeye_nodes)
        row = QHBoxLayout()
        btn_he_auto = QPushButton('Auto-sample (sweep)')
        btn_he_auto.setToolTip('/handeye_calib/auto_sample — moves the arm')
        btn_he_auto.clicked.connect(self._on_handeye_auto)
        row.addWidget(btn_he_auto)
        btn_he_cancel = QPushButton('Cancel sweep')
        btn_he_cancel.clicked.connect(
            lambda: self._run(self.bridge.handeye_cancel, label='handeye-cancel'))
        row.addWidget(btn_he_cancel)
        btn_he_cap = QPushButton('Capture here')
        btn_he_cap.setToolTip('/handeye_calib/capture — one sample at the current pose')
        btn_he_cap.clicked.connect(
            lambda: self._run(self.bridge.handeye_capture, label='handeye-capture',
                              on_done=lambda r: self._refresh_handeye_status()))
        row.addWidget(btn_he_cap)
        he_layout.addLayout(row)
        row = QHBoxLayout()
        btn_he_compute = QPushButton('Compute && save T_hc2ee')
        btn_he_compute.clicked.connect(self._on_handeye_compute)
        row.addWidget(btn_he_compute)
        btn_he_load = QPushButton('Load latest run')
        btn_he_load.setToolTip('/handeye_calib/load_latest — re-feed the previous run\'s samples')
        btn_he_load.clicked.connect(
            lambda: self._run(self.bridge.handeye_load_latest, label='handeye-load',
                              on_done=lambda r: self._refresh_handeye_status()))
        row.addWidget(btn_he_load)
        btn_he_reset = QPushButton('Reset samples')
        btn_he_reset.clicked.connect(
            lambda: self._run(self.bridge.handeye_reset, label='handeye-reset',
                              on_done=lambda r: self._refresh_handeye_status()))
        row.addWidget(btn_he_reset)
        btn_he_status = QPushButton('Status')
        btn_he_status.clicked.connect(self._refresh_handeye_status)
        row.addWidget(btn_he_status)
        he_layout.addLayout(row)
        self.lbl_handeye_state = QLabel('samples: —')
        self.lbl_handeye_state.setStyleSheet('font-weight:bold;')
        he_layout.addWidget(self.lbl_handeye_state)
        self.lbl_handeye_last = QLabel('—')
        self.lbl_handeye_last.setWordWrap(True)
        self.lbl_handeye_last.setStyleSheet('color:#888;')
        he_layout.addWidget(self.lbl_handeye_last)
        layout.addWidget(he_box)
        self._handeye_online = None

        loc_box = QGroupBox('Single tag locate (debug)')
        loc_layout = QVBoxLayout(loc_box)
        lnote = QLabel(
            'Base parked on the target floor tag, hand cam already '
            'viewing a cross tag (drive the arm from the Arm tab or a '
            'plan seed first).')
        lnote.setWordWrap(True)
        lnote.setStyleSheet('color:#888;')
        loc_layout.addWidget(lnote)
        row = QHBoxLayout()
        row.addWidget(QLabel('tag id:'))
        self.spin_locate_tag = QSpinBox()
        self.spin_locate_tag.setRange(100, 150)
        self.spin_locate_tag.setValue(105)
        row.addWidget(self.spin_locate_tag)
        btn_loc = QPushButton('Locate')
        btn_loc.clicked.connect(self._on_calib_locate)
        row.addWidget(btn_loc)
        row.addStretch(1)
        loc_layout.addLayout(row)
        layout.addWidget(loc_box)
        layout.addStretch(1)

        self._calib_running = False
        self._calib_counts = {'ok': 0, 'fail': 0, 'degraded': 0}
        self._calib_online = False
        # Periodic master-registry poll (cheap XML-RPC lookup, no service
        # call) so the operator sees "nodes offline" BEFORE pressing
        # START instead of a generic timeout 6 s after.
        self._calib_node_timer = QTimer(self)
        self._calib_node_timer.timeout.connect(self._poll_calib_nodes)
        self._calib_node_timer.start(3000)
        self._poll_calib_nodes()
        self._refresh_calib_plan_note()
        return tab

    def _on_handeye_auto(self):
        if not self.bridge.handeye_online():
            self.append_log(
                '[handeye] hand-eye node is NOT running — start it with:  '
                'roslaunch path_tag_locator path_tag_locator.launch '
                'use_handeye_calib:=true')
            return
        self.lbl_handeye_last.setText('sweep requested…')
        self._run(self.bridge.handeye_auto_sample, label='handeye-auto')

    def _on_handeye_compute(self):
        def _done(result):
            if not (isinstance(result, tuple) and len(result) >= 2):
                return
            ok, message = result[0], str(result[1])
            self.lbl_handeye_last.setText(
                ('saved: ' if ok else 'compute FAILED: ') + message.split('\n')[0])
            self._refresh_handeye_status()
        self.lbl_handeye_last.setText('computing…')
        self._run(self.bridge.handeye_compute, label='handeye-compute', on_done=_done)

    def _refresh_handeye_status(self):
        def _done(result):
            if isinstance(result, tuple) and len(result) >= 2 and result[0]:
                self.lbl_handeye_state.setText(str(result[1]).split('\n')[0])
        self._run(self.bridge.handeye_status, label=None, on_done=_done)

    def _on_handeye_progress(self, ev):
        """Sweep events from /handeye_calib/progress."""
        phase = ev.get('phase')
        n = ev.get('n_samples')
        if n is not None:
            self.lbl_handeye_state.setText(f'samples: {n}')
        if phase == 'align':
            self.lbl_handeye_last.setText(
                f"squaring up: iteration {ev.get('iteration')}, "
                f"xy {ev.get('xy_mm', 0):.1f} mm, tilt {ev.get('tilt_deg', 0):.2f}°")
        elif phase == 'start':
            self.append_log(
                f"[handeye] sweep: {ev.get('n_planned')} views planned, "
                f"{ev.get('n_rejected')} rejected {ev.get('rejected') or ''}")
            self.lbl_handeye_last.setText(
                f"sweep: 0/{ev.get('n_planned')} views")
        elif phase == 'sample':
            idx, total = ev.get('index', 0), ev.get('total', 0)
            ok = ev.get('ok')
            self.lbl_handeye_last.setText(
                f"sweep: {idx}/{total} {ev.get('label', '')} — "
                f"{'captured' if ok else 'skipped: ' + str(ev.get('reason', ''))}"
                f"  (captured {ev.get('n_captured', 0)}, skipped {ev.get('n_skipped', 0)})")
            if not ok:
                self.append_log(f"[handeye] view {idx}/{total} {ev.get('label', '')}: "
                                f"{ev.get('reason', '')}")
        elif phase == 'finished':
            self.append_log(f"[handeye] {ev.get('summary', 'sweep finished')}")
            self.lbl_handeye_last.setText(ev.get('summary', 'sweep finished'))

    def _poll_calib_nodes(self):
        he = self.bridge.handeye_online()
        if he != self._handeye_online:
            self._handeye_online = he
            if he:
                self.lbl_handeye_nodes.setText('hand-eye node: ONLINE')
                self.lbl_handeye_nodes.setStyleSheet('color:#2e7d32;')
            else:
                self.lbl_handeye_nodes.setText(
                    'hand-eye node: OFFLINE — run:  roslaunch path_tag_locator '
                    'path_tag_locator.launch use_handeye_calib:=true')
                self.lbl_handeye_nodes.setStyleSheet('color:#b02020;')
        online = self.bridge.calib_nodes_online()
        if online == self._calib_online and 'checking' not in \
                self.lbl_calib_nodes.text():
            return
        self._calib_online = online
        if online:
            self.lbl_calib_nodes.setText('nodes: ONLINE')
            self.lbl_calib_nodes.setStyleSheet('color:#2e7d32;')
        else:
            self.lbl_calib_nodes.setText(
                'nodes: OFFLINE — run:  roslaunch path_tag_locator '
                'path_tag_locator.launch')
            self.lbl_calib_nodes.setStyleSheet(
                'color:#b02020; font-weight:bold;')

    # (plan file, ref file, kind). The ref file ALWAYS
    # travels with the plan: both plates carry cross tags with the same ids
    # 0-5, so a mixed pair shifts every result by 3.89 m.
    # (plan file, ref file, kind). The ref file ALWAYS travels with the
    # plan: both plates carry cross tags with the same ids 0-5, so a mixed
    # pair shifts every result by 3.89 m.
    _CALIB_PLANS = (
        ('calibration_plan_plate1.yaml', 'reference_tags.yaml', ''),
        ('calibration_plan_plate2.yaml', 'reference_tags_plate2.yaml', ''),
        ('calibration_plan_plate1_yawsweep.yaml',
         'reference_tags.yaml', 'sweep'),
        ('calibration_plan_plate2_yawsweep.yaml',
         'reference_tags_plate2.yaml', 'sweep'),
    )

    def _calib_paths(self):
        plan, ref, _ = self._CALIB_PLANS[self.combo_calib_plate.currentIndex()]
        base = '$(find path_tag_locator)/config/'
        return base + plan, base + ref

    def _calib_kind(self):
        """'' = real calibration, 'sweep' = a diagnostic whose map_world
        output is NOT a result."""
        return self._CALIB_PLANS[self.combo_calib_plate.currentIndex()][2]

    _CALIB_NOTES = {
        'sweep': (
            '⚠️ NOT a calibration — a DIAGNOSTIC. One tag, one cross tag, 6 '
            'camera yaws; the base must not move. front_cam cancels by '
            'construction, so the spread is arm-side only: large ⇒ HAND-EYE '
            'is wrong, ~0 ⇒ front_cam is. Analyse with:  rosrun '
            'path_tag_locator analyse_yaw_sweep.py <session_dir>'),
    }

    def _refresh_calib_plan_note(self):
        note = self._CALIB_NOTES.get(self._calib_kind(), '')
        self.lbl_calib_plan_note.setText(note)
        self.lbl_calib_plan_note.setStyleSheet(
            'color:#b06000; font-weight:bold;' if note else '')

    def _on_calib_start(self):
        if self._calib_running:
            self.append_log('[calib] a session is already running')
            return
        if not self.bridge.calib_nodes_online():
            self.append_log(
                '[calib] calibration nodes are NOT running — start them '
                'first:  roslaunch path_tag_locator path_tag_locator.launch')
            self._poll_calib_nodes()
            return
        plan_path, ref_path = self._calib_paths()
        dry = self.chk_calib_dry.isChecked()
        kind = self._calib_kind()
        self._calib_running = True
        self._calib_counts = {'ok': 0, 'fail': 0, 'degraded': 0}
        self._refresh_calib_labels()
        self.btn_calib_start.setEnabled(False)
        self.lbl_calib_state.setText(
            ('DRY RUN' if dry else 'RUNNING') + '  ' +
            self.combo_calib_plate.currentText())
        self.append_log('[calib] session start (%s, dry=%s)'
                        % (self.combo_calib_plate.currentText(), dry))

        def _done(result):
            self._calib_running = False
            self.btn_calib_start.setEnabled(True)
            ok, message, report = result
            self.lbl_calib_state.setText(
                ('finished OK' if ok else 'finished with FAILURES')
                + '  —  ' + message.split(';')[0])
            if report.get('output_yaml_path'):
                self.append_log('[calib] output: %s'
                                % report['output_yaml_path'])
            if kind and not dry:
                # The per-attempt records are what the analysers read —
                # map_world upserts per tag, and in a sweep EVERY entry is
                # the same tag, so it would keep only the last one and
                # destroy the whole measurement. Point at the session dir.
                self.append_log(
                    '[calib] %s session recorded under '
                    '<ws>/log/path_tag_locator/calibrate/<newest>/  — analyse '
                    'with:  rosrun path_tag_locator analyse_yaw_sweep.py '
                    '<that dir>' % kind)

        self._run(self.bridge.run_map_calibration,
                  dry_run=dry, plan_path=plan_path, ref_tags_path=ref_path,
                  label='calibration', on_done=_done)

    def _on_calib_locate(self):
        tag = int(self.spin_locate_tag.value())

        def _done(result):
            if result.get('success'):
                x, y, z = result['position_m']
                self.append_log('[locate] tag %d: (%.4f, %.4f, %.4f) m'
                                % (tag, x, y, z))
            else:
                self.append_log('[locate] tag %d FAILED: %s'
                                % (tag, result.get('message')))

        self._run(self.bridge.locate_path_tag, tag_b_id=tag,
                  label='locate', on_done=_done)

    def _refresh_calib_labels(self):
        c = self._calib_counts
        self.lbl_calib_counts.setText(
            'ok %d   fail %d   degraded %d'
            % (c['ok'], c['fail'], c['degraded']))

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
        # RosBridge's outputs are plain `Signal`s since 2026-09-15 (the
        # bridge is shared with the web server and no longer a QObject), so
        # they fire SYNCHRONOUSLY on rospy's callback threads. Every handler
        # below touches widgets, which Qt allows only from the GUI thread —
        # so each connection is routed through ONE pyqtSignal, which Qt
        # queues across the thread boundary (the same mechanism append_log
        # already relies on). A signal emitted on the GUI thread itself
        # (the offscreen checks' fake bridge) is delivered directly.
        self._dispatch.connect(self._dispatch_gui)

        def link(signal, handler):
            signal.connect(
                lambda *args, h=handler: self._dispatch.emit(h, args))

        link(self.bridge.image_received, self._on_image)
        link(self.bridge.arm_state, self._on_arm_state)
        link(self.bridge.standoff_state, self._on_standoff_state)
        link(self.bridge.task_state, self._on_task_state)
        link(self.bridge.task_list, self._on_task_list)
        link(self.bridge.lift_state, self._on_lift_state)
        link(self.bridge.mobile_state, self._on_mobile_state)
        link(self.bridge.battery_state, self._on_battery)
        link(self.bridge.estop_state, self._on_estop)
        link(self.bridge.camera_state, self._on_camera_state)
        link(self.bridge.lamp_state, self._on_lamp_state)
        link(self.bridge.calib_progress, self._on_calib_progress)
        link(self.bridge.scan_progress, self._on_scan_progress)
        link(self.bridge.handeye_progress, self._on_handeye_progress)
        link(self.bridge.tag_ids, self._on_tag_ids)
        self.bridge.log.connect(self.append_log)     # already thread-safe

        # Ask for whatever already arrived. The bridge subscribes in its own
        # constructor, which runs before these connects, so a LATCHED topic's
        # retained message was emitted into nothing — latched means "delivered
        # once on connect", not "redelivered later". /task_state and
        # /camera/state are both latched one-shots and were simply missing from
        # the window until something changed them. Also populates every other
        # field immediately rather than after the next periodic update.
        self.bridge.replay()

    @pyqtSlot(object, object)
    def _dispatch_gui(self, handler, args):
        handler(*args)

    def _on_image(self, name, bgr):
        view = self._views.get(name)
        if view is not None:
            view.set_frame(bgr)

    def _on_scan_progress(self, ev):
        """Per-point events from a running scan (/arm/scan_progress).

        One log line per finished point — a FAILED one carries the arm's
        reason (e.g. "IK failed (code 112): target ... is 2.55 m from the
        arm base") — plus a SCAN chip with the running count. Before this
        the operator saw nothing in the UI while every point of a scan
        failed; the only trace was arm_node's rosout.
        """
        phase = ev.get('phase')
        idx, total = ev.get('index', 0), ev.get('total', 0)
        n_ok, n_fail = ev.get('n_ok', 0), ev.get('n_fail', 0)
        if phase == 'start':
            self.append_log(f"[scan] start: {total} points "
                            f"({ev.get('scan_points', total)} to scan)")
            self.lbl_scan.setText(f'SCAN 0/{total}')
            self._tint(self.lbl_scan, '#553311')
        elif phase == 'move':
            kind = 'pt' if ev.get('scan', True) else 'via'
            self.lbl_scan.setText(
                f"SCAN {idx}/{total} {kind} {ev.get('point_id', '?')}")
        elif phase == 'done':
            # `done` = the frames are captured and the arm is free to move
            # on; the Ra arrives later in a `result` event from arm_node's
            # inference worker (2026-09-14). An older arm_node still puts
            # ra_mean on `done`, which is logged here too.
            if ev.get('scan', True):
                ra = ev.get('ra_mean')
                ra_s = f" ra={float(ra):.4f}" if ra is not None else ''
                self.append_log(
                    f"[scan] {idx}/{total} pt {ev.get('point_id', '?')} "
                    f"g{ev.get('group_id', '?')}: OK{ra_s}  {ev.get('message', '')}")
            self.lbl_scan.setText(f'SCAN {idx}/{total} ok {n_ok} fail {n_fail}')
            self._tint(self.lbl_scan, '#553311' if not n_fail else '#663300')
        elif phase == 'result':
            ra = ev.get('ra_mean')
            ra_s = (f"ra={float(ra):.4f}" if ra is not None
                    else f"no Ra  {ev.get('message', '')}")
            self.append_log(
                f"[scan] {idx}/{total} pt {ev.get('point_id', '?')} "
                f"g{ev.get('group_id', '?')}: {ra_s}")
        elif phase == 'failed':
            self.append_log(
                f"[scan] {idx}/{total} pt {ev.get('point_id', '?')} "
                f"g{ev.get('group_id', '?')}: FAIL — {ev.get('message', '')}")
            self.lbl_scan.setText(f'SCAN {idx}/{total} ok {n_ok} fail {n_fail}')
            self._tint(self.lbl_scan, '#7a1f1f')
        elif phase == 'finished':
            tail = ' (cancelled)' if ev.get('cancelled') else ''
            self.append_log(
                f"[scan] finished{tail}: {n_ok} ok, {n_fail} failed of {total}")
            self.lbl_scan.setText(
                f'SCAN done {n_ok} ok / {n_fail} fail{tail}')
            self._tint(self.lbl_scan, '#7a1f1f' if n_fail else '#1b3a1b')

    def _on_calib_progress(self, entry):
        """Per-tag status from a running map-calibration session."""
        status = entry.get('status', '?')
        tag = entry.get('tag', '?')
        if status == 'ok':
            degraded = bool(entry.get('degraded'))
            note = ' (DEGRADED)' if degraded else ''
            self.append_log(
                f"[calib] tag {tag}: OK{note}  x={entry.get('x', 0):.3f} "
                f"y={entry.get('y', 0):.3f}")
            self._calib_counts['ok'] += 1
            if degraded:
                self._calib_counts['degraded'] += 1
            self.lbl_calib_last.setText(
                'tag %s: OK%s  (%.3f, %.3f, %.3f)'
                % (tag, note, entry.get('x', 0), entry.get('y', 0),
                   entry.get('z', 0)))
        elif status == 'fail':
            self.append_log(
                f"[calib] tag {tag}: FAIL — {entry.get('error', '')}")
            self._calib_counts['fail'] += 1
            self.lbl_calib_last.setText(
                'tag %s: FAIL — %s' % (tag, entry.get('error', '')[:120]))
        else:
            self.append_log(f'[calib] tag {tag}: {status}')
            self.lbl_calib_last.setText('tag %s: %s' % (tag, status))
        self._refresh_calib_labels()

    def _on_arm_state(self, state):
        if state['pose_valid']:
            self._arm_pose = state['tcp_pose']
        for i, axis in enumerate(ARM_AXES):
            text = f'{state["tcp_pose"][i]:.2f}' if state['pose_valid'] else '—'
            self.lbl_pose[axis].setText(text)
        flag = 'BUSY' if state['busy'] else state['state'].upper()
        self.lbl_arm.setText(f'ARM {flag}')
        self._tint(self.lbl_arm, '#553311' if state['busy'] else '#1b3a1b')

    # ---------- /task_list -> Task tab combo ----------
    @staticmethod
    def task_summary(info):
        """One line describing a /task_list entry (also the combo tooltip)."""
        mode = info.get('scan_mode') or info.get('kind') or '?'
        tags = info.get('tags') or []
        parts = [str(mode), 'tags ' + (
            ','.join(str(t) for t in tags) if tags else '—')]
        pts = info.get('points')
        if pts:
            s = f'{pts} pts'
            trav = info.get('traverse_points')
            if trav:
                s += f' (+{trav} traverse)'
            parts.append(s)
        lift = info.get('lift_height_mm')
        parts.append('lift —' if lift is None else f'lift {float(lift):g} mm')
        files = info.get('files') or []
        if files:
            parts.append(files[0] if len(files) == 1 else f'{len(files)} files')
        paired = info.get('paired_file')
        if paired:
            parts.append(f'paired {paired}')
        return ' · '.join(parts)

    def _on_task_list(self, payload):
        """Rebuild the task combo from task_executor's /task_list."""
        tasks = payload.get('tasks') or []
        first_time = not self._task_list_seen
        self._task_list_seen = True
        # A name the operator typed by hand (not one of the previously
        # listed tasks) survives a republish; a listed task that has gone
        # away does not — the executor no longer knows it.
        prev_names = set(self._task_infos)
        self._task_infos = {t.get('name'): t for t in tasks if t.get('name')}
        current = self.combo_task.currentText().strip()
        typed = bool(current) and current not in prev_names and not first_time
        names = list(self._task_infos.keys())
        self.combo_task.blockSignals(True)
        try:
            self.combo_task.clear()
            self.combo_task.setEditText('')
            for i, name in enumerate(names):
                self.combo_task.addItem(name)
                self.combo_task.setItemData(
                    i, self.task_summary(self._task_infos[name]), Qt.ToolTipRole)
            if first_time and current == 'go_home':
                # The placeholder item, not an operator's choice: offer the
                # first real task instead.
                current = names[0] if names else ''
            if current in names:
                self.combo_task.setCurrentIndex(names.index(current))
            elif typed:
                # Keep the hand-typed name in the edit field rather than
                # silently replacing it; the detail line flags it.
                self.combo_task.setEditText(current)
        finally:
            self.combo_task.blockSignals(False)
        self._update_task_detail(self.combo_task.currentText())
        self.append_log(f'[task] /task_list: {len(names)} task(s) from '
                        f'{payload.get("task_dir", "?")}')

    def _update_task_detail(self, text):
        name = (text or '').strip()
        info = self._task_infos.get(name)
        if info is None:
            self.lbl_task_detail.setText(
                '' if not name else
                (f'{name}: not in /task_list' if self._task_list_seen
                 else f'{name}: task list not received yet'))
            return
        self.lbl_task_detail.setText(f'{name}: {self.task_summary(info)}')

    def _on_task_state(self, state):
        name = state.get('task') or '—'
        idx, total = state.get('group_index', 0), state.get('group_total', 0)
        progress = f' {idx}/{total}' if total else ''
        self.lbl_task.setText(f'TASK {state.get("state", "?")} {name}{progress}')
        note = state.get('note')
        if note:
            self.append_log(f'[task] {state.get("state")} — {note}')
        self._update_charge_chip(state)

    def _update_charge_chip(self, state):
        """CHARGE chip + Task-tab line from /task_state's charging fields
        (task_executor since 2026-09-14; an older executor sends none, and
        the chip stays at '—')."""
        if 'charge_phase' not in state:
            return
        phase = state.get('charge_phase') or '?'
        charging = state.get('charging')
        pct = state.get('battery_pct')
        pct_s = f' {float(pct):.0f}%' if pct is not None else ''
        if charging:
            text, tint = f'CHARGE charging{pct_s}', '#4a1a4a'      # magenta, like the lamp
        elif phase == 'full':
            text, tint = f'CHARGE full{pct_s}', '#3a3a3a'          # white on the lamp
        elif phase == 'returning':
            text, tint = f'CHARGE returning{pct_s}', '#553311'
        elif phase == 'dock_failed':
            text, tint = f'CHARGE DOCK FAILED{pct_s}', '#7a1f1f'
        else:
            text, tint = f'CHARGE {phase}{pct_s}', ''
        self.lbl_charge.setText(text)
        self._tint(self.lbl_charge, tint)
        self.lbl_charge_detail.setText(
            f'phase: {phase}, BMS charging: {"yes" if charging else "no"}{pct_s}')

    def _on_lift_state(self, state):
        height = state.get('height_mm')
        homed = state.get('homed')
        text = 'LIFT '
        text += '—' if height is None else f'{float(height):.0f}mm'
        if homed is False:
            text += ' UNHOMED'
        self.lbl_lift.setText(text)
        self._tint(self.lbl_lift, '#553311' if homed is False else '')

    def _on_mobile_state(self, state):
        busy = state.get('busy')
        if state.get('emergency_stop'):
            text, colour = 'BASE E-LATCHED', '#552222'
        elif busy:
            text, colour = f'BASE {busy}', '#553311'
        elif state.get('stop_requested'):
            text, colour = 'BASE STOPPED', '#553311'
        else:
            text, colour = 'BASE idle', '#1b3a1b'
        self.lbl_mobile.setText(text)
        self._tint(self.lbl_mobile, colour)
        tags = state.get('visible_tags') or []
        last = state.get('last_known_tag')
        res = state.get('result') or {}
        self.lbl_mob_status.setText(
            f'base: {busy or "idle"} | visible tags {tags} | last tag {last}'
            + (f' | last result: {res.get("message")}' if res else ''))

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

    def _on_lamp_toggled(self, on):
        if getattr(self, '_lamp_from_node', False):
            return          # reflecting the node's state, not a click
        self.bridge.set_vision_lamp(on)

    def _on_lamp_state(self, on):
        """/camera/lamp_state is the truth; the checkbox follows it."""
        self._lamp_from_node = True
        try:
            self.chk_lamp.setChecked(bool(on))
        finally:
            self._lamp_from_node = False
        self._tint(self.lbl_camera, '#665500' if on else None)

    @staticmethod
    def _tint(label, colour):
        base = 'padding:4px 10px; border:1px solid #555;'
        label.setStyleSheet(base + (f' background:{colour}; color:white;'
                                    if colour else ''))

    # ==========================================================
    # ACTIONS
    # ==========================================================
    def _run(self, fn, *args, label=None, on_done=None, on_error=None,
             preview=False, **kwargs):
        """Run a blocking bridge call on the pool and log its result.

        `on_done(result)` runs only when the call returned; `on_error(msg)`
        only when it raised — kept separate because every existing on_done
        unpacks its result and would not survive a None. `preview=True`
        marks a live-preview grab: it is counted separately so the CAPTURE
        button is not greyed out by the 5 Hz preview (2026-09-15 — with the
        preview on the button was disabled almost continuously)."""
        worker = CallWorker(fn, *args, **kwargs)
        self._live_workers.add(worker)
        self._busy_calls += 1
        if preview:
            self._preview_calls += 1
        self._update_busy()

        def _finished(result):
            self._live_workers.discard(worker)
            self._busy_calls -= 1
            if preview:
                self._preview_calls -= 1
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
            self._live_workers.discard(worker)
            self._busy_calls -= 1
            if preview:
                self._preview_calls -= 1
            self._update_busy()
            self.append_log(f'[{label or "call"}] ERROR: {message}')
            if on_error is not None:
                on_error(message)

        worker.signals.finished.connect(_finished)
        worker.signals.failed.connect(_failed)
        self._pool.start(worker)

    def _update_busy(self):
        # Preview grabs do not block CAPTURE; a capture (or any other call)
        # in flight does.
        self.btn_capture.setEnabled(
            self._busy_calls - self._preview_calls == 0)

    # ---------- Keyence standoff assist ----------
    @staticmethod
    def standoff_text(st):
        """One line for /arm/standoff_state: standoff, error vs target, raw."""
        raw = st.get('raw_mm')
        tgt = st.get('target_mm')
        if st.get('valid') and st.get('standoff_mm') is not None:
            err = st.get('err_mm') or 0.0
            hint = ('ON TARGET' if abs(err) <= 0.2 else
                    f'{abs(err):.2f} mm too {"close" if err > 0 else "far"}')
            return (f'standoff: {st["standoff_mm"]:.2f} mm   '
                    f'(target {tgt:g}: {hint})   raw {raw:+.2f}')
        side = st.get('side')
        which = ({'far': 'too far', 'close': 'too close'}.get(side)
                 or 'unknown side')
        return (f'standoff: OUT OF RANGE ({which})   raw {raw:+.0f}   '
                f'— jog Z toward the surface until a value appears')

    def _on_standoff_state(self, st):
        self._standoff_seen_at = time.monotonic()
        self.lbl_standoff.setText(self.standoff_text(st))
        self._tint(self.lbl_standoff,
                   '#1b3a1b' if st.get('valid') else '#553311')

    def _set_standoff_inflight(self, inflight):
        self._standoff_inflight = inflight
        self.btn_standoff.setEnabled(not inflight)

    def _on_standoff(self):
        target = self.spin_standoff_target.value()
        self._set_standoff_inflight(True)
        self._run(self.bridge.arm_standoff, target,
                  label=f'standoff -> {target:g} mm',
                  on_done=lambda _r: self._set_standoff_inflight(False),
                  on_error=lambda _m: self._set_standoff_inflight(False))

    def _on_jog(self, axis, sign):
        step = self.spin_step.value() * sign
        self._run(self.bridge.arm_jog, axis, step, self.spin_vel.value(),
                  label=f'jog {axis} {step:+g}')

    def _set_mobile_inflight(self, inflight):
        self._mobile_move_inflight = inflight
        for b in self._mobile_buttons:
            b.setEnabled(not inflight)

    def _on_mobile_drive(self, sign):
        dist = self.spin_mob_dist.value() * sign
        self._set_mobile_inflight(True)
        self._run(self.bridge.mobile_drive, dist, self.spin_mob_v.value(),
                  label=f'base drive {dist:+.3f} m',
                  on_done=lambda _r: self._set_mobile_inflight(False),
                  on_error=lambda _m: self._set_mobile_inflight(False))

    def _on_mobile_pivot(self, sign):
        angle = self.spin_mob_angle.value() * sign
        self._set_mobile_inflight(True)
        self._run(self.bridge.mobile_pivot, angle, self.spin_mob_w.value(),
                  label=f'base pivot {angle:+.1f} deg',
                  on_done=lambda _r: self._set_mobile_inflight(False),
                  on_error=lambda _m: self._set_mobile_inflight(False))

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
        if self.chk_lamp.isChecked():
            self.chk_lamp.setChecked(False)
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
            self.append_log('[UI] preview on — basler held open'
                            + (' (lamp held on)' if self.chk_lamp.isChecked()
                               else ', lamp off'))
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
                  on_done=self._on_preview_frame, preview=True)

    def _on_preview_frame(self, result):
        ok, _message, frames = result
        if ok and frames:
            self._views['basler'].set_frame(frames[-1])

    # ---------- capture ----------
    def _on_capture(self):
        if self._preview_on:
            # PAUSE the preview for the shot instead of switching it off:
            # the device stays held open (no close + reopen in the middle
            # of the capture) and no preview grab is queued behind the real
            # one, so lamp-off and lamp-on frames cannot interleave in one
            # shot. The timer resumes in _on_captured.
            self._preview_timer.stop()
        self.append_log('[UI] capture requested'
                        + (' (preview paused)' if self._preview_on else ''))
        self._run(self.bridge.capture, self.spin_samples.value(), -1.0,
                  self.chk_led.isChecked(), on_done=self._on_captured,
                  on_error=lambda _m: self._resume_preview())

    def _resume_preview(self):
        if self._preview_on and not self._preview_timer.isActive():
            self._preview_timer.start(
                int(1000 / max(0.2, self.spin_preview_hz.value())))

    def _on_captured(self, result):
        self._resume_preview()
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
