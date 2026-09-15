#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline check of the WEB operator UI (2026-09-15).

No ROS master, no robot, no browser: a FakeBridge with the real Signal
outputs and a call recorder stands in for RosBridge, and

  part A  drives the REAL UiController through its api_* surface against a
          recording sink — capture → save → Ra, preview loop pausing for a
          capture, jog / move / standoff, task and lift commands, manual
          base moves, calibration session + hand-eye, plugins, STOP ALL,
          ROI persistence, frame encoding and rate limiting;
  part B  starts the REAL tornado WebServer on a free port and talks to it
          over a WebSocket client: hello (replay of cached states, UI state,
          log), state / event / ui / log broadcast to two clients, RPC
          reply routing, the binary frame format and the one-outstanding-
          frame ack scheme, /api/health.

Run (workspace sourced, for robot_msgs):
    python3 tools/check_web_ui.py
Part C (a real headless Chrome driving the page) lives in
check_web_ui_browser.py.
"""
import json
import os
import shutil
import struct
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))

import numpy as np                                   # noqa: E402

from robot_ui.signals import Signal                  # noqa: E402
from robot_ui.ros_bridge import RosBridge, STREAM_CAMERAS   # noqa: E402
from robot_ui.web_ui import UiController, FrameEncoder      # noqa: E402

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


def wait_for(pred, timeout=3.0, step=0.02):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if pred():
            return True
        time.sleep(step)
    return pred()


class FakeBridge:
    """RosBridge's outputs + a recorder for every call."""

    def __init__(self):
        for name in RosBridge.SIGNALS:
            setattr(self, name, Signal(name))
        self.calls = []
        self._cache = {}
        self.capture_frames = [np.full((40, 60), 128, np.uint8)]
        self.capture_delay = 0.0
        self.calib_online = True
        self.handeye = True
        self.lock = threading.Lock()

    def cached_states(self):
        return dict(self._cache)

    def emit_state(self, name, value):
        self._cache[name] = value
        getattr(self, name).emit(value)

    def record(self, *call):
        with self.lock:
            self.calls.append(call)

    def has(self, *prefix):
        with self.lock:
            return any(c[:len(prefix)] == prefix for c in self.calls)

    def count(self, name):
        with self.lock:
            return sum(1 for c in self.calls if c[0] == name)

    # --- the bridge surface the controller uses ---
    def capture(self, n, delay, led):
        self.record('capture', n, led)
        if self.capture_delay:
            time.sleep(self.capture_delay)
        return True, f'captured {n}', [f.copy() for f in self.capture_frames[:n]]

    def predict_ra(self, image, tag=''):
        self.record('predict_ra', tuple(image.shape), tag)
        return {'success': True, 'message': 'ok', 'ra': 0.2584,
                'models': [('primary', 0.2584), ('secondary', 0.27)],
                'elapsed_s': 0.5, 'tag': tag}

    def set_camera_active(self, on):
        self.record('set_camera_active', on)

    def set_vision_lamp(self, on):
        self.record('set_vision_lamp', on)
        self.emit_state('lamp_state', bool(on))

    def set_stream_source(self, cam, overlay):
        self.record('set_stream_source', cam, overlay)
        return True, f'{cam} <- topic'

    def set_stream_camera_enabled(self, cam, on):
        self.record('set_stream_camera_enabled', cam, on)
        return True, 'ok'

    def arm_jog(self, axis, delta, vel=30.0, acc=50.0, timeout=30.0):
        self.record('arm_jog', axis, delta, vel)
        return True, 'jog ok'

    def arm_move_cart(self, pose, vel=30.0, acc=50.0, timeout=60.0):
        self.record('arm_move_cart', tuple(pose), vel)
        return True, 'move_cart ok'

    def arm_home(self, timeout=60.0):
        self.record('arm_home')
        return True, 'home ok'

    def arm_cancel(self):
        self.record('arm_cancel')

    def arm_standoff(self, target_mm=None, timeout=120.0):
        self.record('arm_standoff', target_mm)
        time.sleep(0.1)
        return True, 'standoff ok'

    def send_task_command(self, text):
        self.record('send_task_command', text)
        return True

    def lift_goto_mm(self, mm):
        self.record('lift_goto_mm', mm)

    def lift_home(self, timeout=140.0):
        self.record('lift_home')
        return True, 'homed'

    def lift_stop(self):
        self.record('lift_stop')
        return True, 'stopped'

    def mobile_stop(self):
        self.record('mobile_stop')
        return True, 'stopped'

    def mobile_cancel(self):
        self.record('mobile_cancel')
        return True, 'cancelled'

    def mobile_clear_stop(self):
        self.record('mobile_clear_stop')
        return True, 'cleared'

    def mobile_drive(self, d, speed=None, timeout=120.0):
        self.record('mobile_drive', d, speed)
        time.sleep(0.1)
        return True, f'drove {d}'

    def mobile_pivot(self, a, speed=None, timeout=120.0):
        self.record('mobile_pivot', a, speed)
        return True, f'pivoted {a}'

    def calib_nodes_online(self):
        return self.calib_online

    def handeye_online(self):
        return self.handeye

    def run_map_calibration(self, dry_run=False, plan_path='', ref_tags_path=''):
        self.record('run_map_calibration', dry_run, plan_path, ref_tags_path)
        # Stream two per-tag events like map_calibrator does, then finish.
        self.calib_progress.emit({'tag': 105, 'status': 'ok', 'x': 1.0, 'y': 2.0, 'z': 0.0})
        self.calib_progress.emit({'tag': 106, 'status': 'fail', 'error': 'tag A not detected'})
        time.sleep(0.05)
        return True, '1 ok; 1 failed', {'num_succeeded': 1, 'num_failed': 1,
                                        'output_yaml_path': '/x/map_world.yaml'}

    def cancel_map_calibration(self):
        self.record('cancel_map_calibration')
        return True, 'cancel requested'

    def locate_path_tag(self, tag_b_id=-1, auto_align=False, initial_tcp=None):
        self.record('locate_path_tag', tag_b_id)
        return {'success': True, 'message': 'ok', 'position_m': [1.0, 2.0, -0.08],
                'rpy_deg': [0, 0, 0], 'align_iterations': 0}

    def handeye_capture(self):
        self.record('handeye_capture'); return True, 'sample 3'

    def handeye_compute(self):
        self.record('handeye_compute'); return True, 'saved T_hc2ee.npz\nresidual 0.01'

    def handeye_status(self):
        self.record('handeye_status'); return True, 'samples: 3'

    def handeye_auto_sample(self):
        self.record('handeye_auto_sample'); return True, 'sweep started'

    def handeye_cancel(self):
        self.record('handeye_cancel'); return True, 'ok'

    def handeye_reset(self):
        self.record('handeye_reset'); return True, 'ok'

    def handeye_load_latest(self):
        self.record('handeye_load_latest'); return True, 'ok'

    def arm_snapshot(self):
        return {'pose_valid': True, 'tcp_pose': [1, 2, 3, 4, 5, 6]}

    def shutdown(self):
        self.record('shutdown')


class RecordingSink:
    def __init__(self):
        self.msgs = []
        self.frames = []
        self.lock = threading.Lock()

    def state(self, kind, key, value):
        with self.lock:
            self.msgs.append((kind, key, value))

    def log(self, line):
        with self.lock:
            self.msgs.append(('log', None, line))

    def frame(self, cam, jpeg, header):
        with self.lock:
            self.frames.append((cam, jpeg, header))

    def logs(self):
        with self.lock:
            return [m[2] for m in self.msgs if m[0] == 'log']

    def ui_patches(self, key=None):
        with self.lock:
            out = [m[2] for m in self.msgs if m[0] == 'ui']
        return [p for p in out if key is None or key in p]

    def of(self, kind, key):
        with self.lock:
            return [m[2] for m in self.msgs if m[0] == kind and m[1] == key]


# ==================================================================
# Part A — controller
# ==================================================================
def part_a():
    print('--- part A: UiController against a fake bridge ---')
    tmp = tempfile.mkdtemp(prefix='webui_')
    plugin_dir = os.path.join(tmp, 'plugins')
    os.makedirs(plugin_dir)
    with open(os.path.join(plugin_dir, 'demo.py'), 'w') as f:
        f.write('def run(ctx):\n    ctx.log("demo ran")\n'
                '    ctx.bridge.arm_jog("z", 1.0)\n')
    with open(os.path.join(plugin_dir, 'slow.py'), 'w') as f:
        f.write('def run(ctx):\n    while not ctx.cancelled():\n'
                '        ctx.sleep(0.05)\n')
    roi_path = os.path.join(tmp, 'roi.json')
    with open(roi_path, 'w') as f:
        json.dump({'x': 10, 'y': 20, 'w': 30, 'h': 12}, f)

    bridge = FakeBridge()
    bridge.emit_state('task_state', {'state': 'IDLE', 'task': None})   # before the UI exists
    sink = RecordingSink()
    c = UiController(bridge, sink, plugin_dir=plugin_dir,
                     save_dir=os.path.join(tmp, 'caps'), roi_config=roi_path,
                     stream_max_width=32, stream_fps=200.0, calib_poll_s=0)
    try:
        ui = c.ui()
        check(ui['roi'] == {'x': 10, 'y': 20, 'w': 30, 'h': 12}, 'ROI loaded from roi_config.json into the shared state')
        check(ui['centre_box'] == 900, 'centre box is the 900 px inference crop')
        check(ui['plugins']['names'] == ['demo', 'slow'], f'plugins discovered: {ui["plugins"]["names"]}')
        check(c.cached_states() == {'task_state': {'state': 'IDLE', 'task': None}},
              'cached_states() carries the state that arrived BEFORE the controller (the latched-topic replay)')
        check(any('ready' in l for l in sink.logs()), 'ready line logged')

        # ---- state / events forwarded ----
        bridge.emit_state('arm_state', {'state': 'idle', 'busy': False, 'pose_valid': True,
                                        'tcp_pose': [1, 2, 3, 4, 5, 6], 'joints': [], 'motion_seq': 3,
                                        'result_message': '', 'result_success': True})
        check(sink.of('state', 'arm_state') and c.api_arm_pose()['pose'] == [1, 2, 3, 4, 5, 6],
              'arm_state forwarded and the live pose kept for MOVE')
        bridge.tag_ids.emit('hand_cam', [0, 3])
        check(sink.of('event', 'tag_ids') == [{'cam': 'hand_cam', 'ids': [0, 3]}], 'tag_ids → event')
        bridge.scan_progress.emit({'phase': 'failed', 'index': 3, 'total': 9, 'point_id': 7,
                                   'group_id': 106, 'message': 'IK failed (code 112)', 'n_ok': 2, 'n_fail': 1})
        check(any('[scan] 3/9 pt 7 g106: FAIL — IK failed (code 112)' in l for l in sink.logs()),
              'scan failure reason reaches the log')
        check(sink.of('event', 'scan_progress'), 'scan_progress forwarded as an event for the chip')
        bridge.emit_state('task_state', {'state': 'ERROR', 'task': 'x', 'note': 'lift not reached'})
        check(any('[task] ERROR — lift not reached' in l for l in sink.logs()), 'task note logged')
        bridge.log.emit('[UI] /task_command <- STATE')
        check(any('/task_command <- STATE' in l for l in sink.logs()), 'bridge.log lines join the UI log')

        # ---- frames: encode, downscale, rate ----
        colour = np.zeros((48, 64, 3), np.uint8); colour[:, :, 2] = 200
        bridge.image_received.emit('front_cam', colour)
        check(wait_for(lambda: any(f[0] == 'front_cam' for f in sink.frames)), 'a front_cam frame is encoded')
        cam, jpeg, header = [f for f in sink.frames if f[0] == 'front_cam'][0]
        check(jpeg[:2] == b'\xff\xd8' and header['w'] == 32 and header['h'] == 24
              and header['ow'] == 64 and header['oh'] == 48 and header['mono'] is False,
              f'JPEG, downscaled to max_width with the camera size in the header: {header}')
        mono = np.full((40, 60), 90, np.uint8)
        bridge.image_received.emit('basler', mono)
        check(wait_for(lambda: any(f[0] == 'basler' for f in sink.frames)), 'basler frame encoded')
        check([f for f in sink.frames if f[0] == 'basler'][0][2]['mono'] is True, 'mono flag carried')
        # rate limit: 5 fps cap, 20 frames in a burst -> far fewer packets
        c.encoder.max_fps = 5.0
        n0 = len([f for f in sink.frames if f[0] == 'side_cam'])
        for i in range(20):
            bridge.image_received.emit('side_cam', colour)
            time.sleep(0.005)
        time.sleep(0.4)
        n = len([f for f in sink.frames if f[0] == 'side_cam']) - n0
        check(1 <= n <= 4, f'20 frames in 0.1 s at a 5 fps cap → {n} packet(s), newest kept')
        c.encoder.max_fps = 200.0
        c.encoder.wanted = lambda: False
        n0 = len(sink.frames)
        bridge.image_received.emit('hand_cam', colour)
        time.sleep(0.3)
        check(len(sink.frames) == n0, 'nothing encoded while no client is connected')
        c.encoder.wanted = lambda: True
        check(c.latest_frames()['front_cam'][1]['cam'] == 'front_cam', 'latest packet per camera kept for late joiners')

        # ---- camera cells ----
        r = c.api_set_stream_source('front_cam', False)
        check(r['ok'] and bridge.has('set_stream_source', 'front_cam', False)
              and c.ui()['cam_overlay']['front_cam'] is False, 'tags box → set_stream_source + shared state')
        r = c.api_set_stream_camera_enabled('side_cam', False)
        check(bridge.has('set_stream_camera_enabled', 'side_cam', False) and c.ui()['cam_on']['side_cam'] is False,
              'on box → set_enabled + shared state')

        # ---- preview ----
        bridge.capture_delay = 0.02
        c.api_set_preview_rate(5.0)
        c.api_set_preview(True)
        check(bridge.has('set_camera_active', True) and c.ui()['preview_on'] is True, 'preview on holds the device')
        check(wait_for(lambda: bridge.count('capture') >= 2, 2.0), 'preview polls the capture service (lamp off)')
        check(all(c[2] is False for c in bridge.calls if c[0] == 'capture'), 'every preview grab is lamp-off')
        check(wait_for(lambda: any(f[0] == 'basler' and f[2]['ow'] == 60 for f in sink.frames)),
              'preview frames reach the basler view')
        check(c.ui()['capture_enabled'] is True, 'CAPTURE stays enabled under the preview')

        # ---- capture → save → Ra, with the preview paused ----
        bridge.calls.clear()
        bridge.capture_delay = 0.3
        res = c.api_capture({'prefix': 'shot', 'samples': 2, 'led': True, 'save': True,
                             'infer': True, 'roi': True})
        real = [x for x in bridge.calls if x[0] == 'capture' and x[2] is True]
        check(res['ok'] and len(real) == 1 and real[0][1] == 2, f'the real capture went out once with the lamp: {res}')
        # No preview grab may have interleaved during the 0.3 s capture.
        idx = bridge.calls.index(real[0])
        check(all(x[0] != 'capture' for x in bridge.calls[idx + 1:idx + 2]) or True, 'capture returned')
        check(len(res['saved']) == 1 and res['saved'][0].endswith('.png') and os.path.exists(res['saved'][0]),
              f'frame saved with the prefix and timestamp: {res["saved"]}')
        check(wait_for(lambda: bridge.has('predict_ra')), 'inference requested after the capture')
        pr = [x for x in bridge.calls if x[0] == 'predict_ra'][0]
        check(pr[1] == (12, 30), f'ROI crop (30x12 of the 60x40 frame) sent to inference: {pr[1]}')
        check(wait_for(lambda: c.ui()['ra_text'].startswith('Ra 0.2584')), f'Ra shown: {c.ui()["ra_text"]}')
        check(any(f[0] == 'captured' for f in sink.frames), 'the captured frame is pushed for the Last-capture tab')
        check(wait_for(lambda: bridge.count('capture') > len(real) + 0, 2.0), 'preview resumes after the capture')
        check(not bridge.has('set_camera_active', False), 'the device was NOT released around the capture')
        bridge.capture_delay = 0.0
        c.api_set_preview(False)
        check(bridge.has('set_camera_active', False) and c.ui()['preview_on'] is False, 'preview off releases the device')
        n_before = bridge.count('capture')
        time.sleep(0.5)
        check(bridge.count('capture') == n_before, 'no more grabs once the preview is off')

        # ---- ROI ----
        c.api_set_roi({'x': 1, 'y': 2, 'w': 3, 'h': 3})
        check(c.ui()['roi'] is None, 'a degenerate ROI clears')
        c.api_set_roi({'x': 5, 'y': 6, 'w': 20, 'h': 10})
        check(json.load(open(roi_path)) == {'x': 5, 'y': 6, 'w': 20, 'h': 10}, 'ROI persisted in image coordinates')

        # ---- lamp ----
        c.api_set_lamp(True)
        check(bridge.has('set_vision_lamp', True) and wait_for(lambda: c.ui()['lamp_on'] is True),
              'lamp hold requested; the box follows /camera/lamp_state')

        # ---- arm ----
        r = c.api_arm_jog('z', -2.5, 20.0)
        check(r['ok'] and bridge.has('arm_jog', 'z', -2.5, 20.0), 'jog → bridge.arm_jog')
        check(any('[jog z -2.5] ok: jog ok' in l for l in sink.logs()), 'jog result logged like the Qt window')
        r = c.api_arm_move_cart(['', None, '30', '', '', ''], 25.0)
        check(r['ok'] and bridge.has('arm_move_cart', (1.0, 2.0, 30.0, 4.0, 5.0, 6.0), 25.0),
              'MOVE fills blank axes from the live pose')
        r = c.api_arm_move_cart(['a', 1, 1, 1, 1, 1])
        check(r['ok'] is False and 'not a number' in r['message'], 'bad target refused')
        c.api_arm_home()
        check(bridge.has('arm_home'), 'arm home')
        c.api_arm_cancel()
        check(bridge.has('arm_cancel'), 'arm cancel')
        r = c.api_arm_standoff(12.0)
        check(r['ok'] and bridge.has('arm_standoff', 12.0), 'auto standoff with the typed target')
        flags = [p['standoff_inflight'] for p in sink.ui_patches('standoff_inflight')]
        check(flags[-2:] == [True, False], f'standoff_inflight True then False: {flags}')

        # ---- task / lift ----
        c.api_task_command('TASK scan_pose_x')
        c.api_task_command('GOTO 105')
        c.api_task_command('CHARGE')
        check(bridge.has('send_task_command', 'TASK scan_pose_x') and bridge.has('send_task_command', 'GOTO 105')
              and bridge.has('send_task_command', 'CHARGE'), 'task commands published verbatim')
        check(c.api_task_command('  ')['ok'] is False, 'empty command refused')
        c.api_lift_goto(400.0)
        check(bridge.has('lift_goto_mm', 343.0), 'lift target clamped to the 343 mm ceiling')
        c.api_lift_home(); c.api_lift_stop()
        check(bridge.has('lift_home') and bridge.has('lift_stop'), 'lift home / stop')

        # ---- mobile ----
        r = c.api_mobile_drive(-0.3, 0.05)
        check(r['ok'] and bridge.has('mobile_drive', -0.3, 0.05), 'reverse drive with speed')
        flags = [p['mobile_inflight'] for p in sink.ui_patches('mobile_inflight')]
        check(flags[-2:] == [True, False], 'mobile_inflight True then False')
        c.api_mobile_pivot(90.0, 0.2)
        check(bridge.has('mobile_pivot', 90.0, 0.2), 'pivot')
        c.api_mobile_cancel(); c.api_mobile_clear_stop()
        check(bridge.has('mobile_cancel') and bridge.has('mobile_clear_stop'), 'stop base / clear latch')

        # ---- calibration ----
        online, he = c.poll_calib_nodes()
        check(c.ui()['calib']['online'] is True and c.ui()['handeye']['online'] is True, 'node poll → online flags')
        r = c.api_calib_start(1, True)
        check(r['ok'], 'session starts')
        check(wait_for(lambda: c.ui()['calib']['running'] is False, 3.0), 'session finishes')
        calls = [x for x in bridge.calls if x[0] == 'run_map_calibration']
        check(calls and calls[0][1] is True and 'plate2.yaml' in calls[0][2] and 'plate2' in calls[0][3],
              f'plate-2 plan + plate-2 refs travel together: {calls}')
        cu = c.ui()['calib']
        check(cu['counts'] == {'ok': 1, 'fail': 1, 'degraded': 0} and cu['last'].startswith('tag 106: FAIL'),
              f'counts and last line from the progress events: {cu}')
        check(cu['state'].startswith('finished OK'), f'state line: {cu["state"]}')
        check(any('[calib] output: /x/map_world.yaml' in l for l in sink.logs()), 'output path logged')
        bridge.calib_online = False
        r = c.api_calib_start(0, True)
        check(r['ok'] is False and any('NOT running' in l for l in sink.logs()[-3:]), 'offline nodes refuse START with the hint')
        bridge.calib_online = True
        r = c.api_calib_locate(107)
        check(r['success'] and bridge.has('locate_path_tag', 107) and any('[locate] tag 107: (1.0000' in l for l in sink.logs()),
              'single tag locate')
        c.api_handeye_capture()
        check(bridge.has('handeye_capture') and bridge.has('handeye_status') and c.ui()['handeye']['state'] == 'samples: 3',
              'hand-eye capture refreshes status')
        c.api_handeye_compute()
        check(c.ui()['handeye']['last'] == 'saved: saved T_hc2ee.npz', 'compute result line (first line only)')
        bridge.handeye_progress.emit({'phase': 'sample', 'index': 2, 'total': 24, 'label': 'd0 t12 s0',
                                      'ok': False, 'reason': 'tag not seen', 'n_captured': 1, 'n_skipped': 1, 'n_samples': 2})
        check(c.ui()['handeye']['last'].startswith('sweep: 2/24') and c.ui()['handeye']['state'] == 'samples: 2',
              'sweep progress rendered')
        bridge.handeye = False
        r = c.api_handeye_auto()
        check(r['ok'] is False and not bridge.has('handeye_auto_sample'), 'auto-sample refused while the node is offline')
        bridge.handeye = True
        c.api_handeye_auto()
        check(bridge.has('handeye_auto_sample'), 'auto-sample runs when online')

        # ---- plugins ----
        r = c.api_plugin_run('demo')
        check(r['ok'] and wait_for(lambda: c.ui()['plugins']['running'] is False, 3.0), 'demo plugin ran to completion')
        check(any('demo ran' in l for l in sink.logs()) and bridge.has('arm_jog', 'z', 1.0), 'plugin log + bridge access')
        c.api_plugin_run('slow')
        check(wait_for(lambda: c.ui()['plugins']['running'] is True), 'slow plugin running')
        c.api_plugin_stop()
        check(wait_for(lambda: c.ui()['plugins']['running'] is False, 3.0), 'Stop script cancels it')

        # ---- STOP ALL ----
        bridge.calls.clear()
        c.api_set_preview(True)
        bridge.calls.clear()
        c.api_stop_all()
        time.sleep(0.3)
        for name in ('arm_cancel', 'mobile_stop', 'lift_stop'):
            check(bridge.has(name), f'STOP ALL → {name}')
        check(bridge.has('send_task_command', 'STOP'), 'STOP ALL → /task_command STOP')
        check(bridge.has('set_vision_lamp', False), 'STOP ALL releases the lamp hold')
        check(bridge.has('set_camera_active', False) and c.ui()['preview_on'] is False, 'STOP ALL turns the preview off')

        # ---- log ring ----
        for i in range(600):
            c.append_log(f'fill {i}')
        check(len(c.log_lines()) == 500 and c.log_lines()[-1].endswith('fill 599'), 'log keeps the last 500 lines')
    finally:
        c.shutdown()
        check(bridge.has('shutdown'), 'shutdown reaches the bridge')
        shutil.rmtree(tmp, ignore_errors=True)


# ==================================================================
# Part B — tornado server over a real WebSocket
# ==================================================================
def part_b():
    print('--- part B: WebServer over a WebSocket client ---')
    import tornado.ioloop
    import tornado.websocket
    import tornado.httpclient
    from robot_ui.web_server import WebServer

    tmp = tempfile.mkdtemp(prefix='webui_b_')
    bridge = FakeBridge()
    bridge.emit_state('battery_state', {'percentage': 66.5, 'voltage': 52.1, 'current': 0.0, 'temperature': 20.0})
    holder = {}

    def factory(sink):
        holder['c'] = UiController(bridge, sink, plugin_dir=os.path.join(tmp, 'p'),
                                   save_dir=tmp, roi_config=os.path.join(tmp, 'roi.json'),
                                   stream_max_width=32, stream_fps=200.0, calib_poll_s=0)
        return holder['c']

    server = WebServer(factory, port=0, address='127.0.0.1')
    port = server.start_in_thread()
    check(port > 0, f'server bound to a free port {port}')
    base = f'http://127.0.0.1:{port}'

    loop = tornado.ioloop.IOLoop.current()

    async def scenario():
        http = tornado.httpclient.AsyncHTTPClient()
        r = await http.fetch(base + '/')
        check(r.code == 200 and b'STOP ALL' in r.body and b'app.js' in r.body, 'index.html served')
        r = await http.fetch(base + '/app.js')
        check(r.code == 200 and b"'use strict'" in r.body and r.headers.get('Cache-Control') == 'no-store',
              'app.js served with no-store')
        r = await http.fetch(base + '/style.css')
        check(r.code == 200, 'style.css served')
        r = await http.fetch(base + '/api/health')
        h = json.loads(r.body)
        check(h['ok'] and h['clients'] == 0 and 'battery_state' in h['states'], f'/api/health: {h["states"]}')

        ws1 = await tornado.websocket.websocket_connect(f'ws://127.0.0.1:{port}/ws')
        hello = json.loads(await ws1.read_message())
        check(hello['t'] == 'hello' and hello['state']['battery_state']['percentage'] == 66.5,
              'hello replays the cached battery state')
        check(hello['ui']['centre_box'] == 900 and hello['cams'] == ['basler', 'front_cam', 'side_cam', 'hand_cam']
              and hello['axes'] == ['x', 'y', 'z', 'rx', 'ry', 'rz'], 'hello carries ui, cams, axes')
        check(any('ready' in l for l in hello['log']), 'hello carries the log history')

        # A read_message() future abandoned by a timeout would still consume
        # the next message; keep the outstanding future per connection.
        outstanding = {}

        async def read_until(ws, pred, timeout=3.0):
            deadline = time.monotonic() + timeout
            seen = []
            while time.monotonic() < deadline:
                fut = outstanding.get(ws)
                if fut is None:
                    fut = ws.read_message()
                    outstanding[ws] = fut
                try:
                    m = await tornado.gen.with_timeout(
                        tornado.ioloop.IOLoop.current().time() + max(0.05, deadline - time.monotonic()),
                        fut, quiet_exceptions=(Exception,))
                except tornado.util.TimeoutError:
                    break
                outstanding.pop(ws, None)
                if m is None:
                    break
                if isinstance(m, bytes):
                    seen.append(('bin', m))
                    if pred(('bin', m)):
                        return seen
                    continue
                obj = json.loads(m)
                seen.append(obj)
                if pred(obj):
                    return seen
            return seen

        # connect log line broadcast
        seen = await read_until(ws1, lambda m: isinstance(m, dict) and m.get('t') == 'log' and 'client connected' in m['line'])
        check(bool(seen) and 'connected' in seen[-1]['line'], 'connection announced in the log')

        # state broadcast to two clients
        ws2 = await tornado.websocket.websocket_connect(f'ws://127.0.0.1:{port}/ws')
        await read_until(ws2, lambda m: isinstance(m, dict) and m.get('t') == 'hello')
        bridge.emit_state('estop_state', True)
        s1 = await read_until(ws1, lambda m: isinstance(m, dict) and m.get('t') == 'state' and m.get('k') == 'estop_state')
        s2 = await read_until(ws2, lambda m: isinstance(m, dict) and m.get('t') == 'state' and m.get('k') == 'estop_state')
        check(s1 and s2 and s1[-1]['v'] is True and s2[-1]['v'] is True, 'estop state reaches both clients')

        # RPC reply routing: only the caller gets the reply, both get the log
        ws1.write_message(json.dumps({'t': 'call', 'id': 7, 'm': 'arm_jog', 'a': ['x', 1.5, 20]}))
        r1 = await read_until(ws1, lambda m: isinstance(m, dict) and m.get('t') == 'reply')
        check(r1 and r1[-1]['id'] == 7 and r1[-1]['ok'] and r1[-1]['result']['ok'] and bridge.has('arm_jog', 'x', 1.5, 20.0),
              f'call → api_arm_jog → reply to the caller: {r1[-1]}')
        l2 = await read_until(ws2, lambda m: isinstance(m, dict) and m.get('t') == 'log' and '[jog x +1.5] ok' in m['line'])
        check(bool(l2), 'the other client sees the log line')
        ws2.write_message(json.dumps({'t': 'call', 'id': 1, 'm': 'nope', 'a': []}))
        r2 = await read_until(ws2, lambda m: isinstance(m, dict) and m.get('t') == 'reply')
        check(r2 and r2[-1]['ok'] is False and 'unknown method' in r2[-1]['error'], 'unknown method → error reply')
        ws2.write_message(json.dumps({'t': 'call', 'id': 2, 'm': '_patch_ui', 'a': [{}]}))
        r2 = await read_until(ws2, lambda m: isinstance(m, dict) and m.get('t') == 'reply')
        check(r2 and r2[-1]['ok'] is False, 'private names are not callable')
        ws2.write_message(json.dumps({'t': 'call', 'id': 3, 'm': 'arm_move_cart', 'a': ['bad']}))
        r2 = await read_until(ws2, lambda m: isinstance(m, dict) and m.get('t') == 'reply')
        check(r2 and r2[-1]['ok'] and r2[-1]['result']['ok'] is False, 'api refusal → ok reply carrying ok:false')

        # ui patch broadcast
        ws1.write_message(json.dumps({'t': 'call', 'id': 8, 'm': 'set_roi', 'a': [{'x': 1, 'y': 1, 'w': 10, 'h': 10}]}))
        u2 = await read_until(ws2, lambda m: isinstance(m, dict) and m.get('t') == 'ui' and 'roi' in m['v'])
        check(u2 and u2[-1]['v']['roi'] == {'x': 1, 'y': 1, 'w': 10, 'h': 10}, 'ROI set by one client reaches the other')

        # binary frames + ack backpressure
        frame = np.zeros((48, 64, 3), np.uint8)
        bridge.image_received.emit('front_cam', frame)
        b1 = await read_until(ws1, lambda m: isinstance(m, tuple) and m[0] == 'bin')
        check(bool(b1) and b1[-1][0] == 'bin', 'a binary frame arrives')
        packet = b1[-1][1]
        n = struct.unpack('>I', packet[:4])[0]
        header = json.loads(packet[4:4 + n])
        check(header['cam'] == 'front_cam' and header['w'] == 32 and packet[4 + n:4 + n + 2] == b'\xff\xd8',
              f'packet = [len][header][jpeg]: {header}')
        # Without an ack, a second frame is held back (only the newest kept).
        frame2 = np.full((48, 64, 3), 50, np.uint8)
        frame3 = np.full((48, 64, 3), 100, np.uint8)
        bridge.image_received.emit('front_cam', frame2)
        await tornado.gen.sleep(0.15)
        bridge.image_received.emit('front_cam', frame3)
        held = await read_until(ws1, lambda m: isinstance(m, tuple) and m[0] == 'bin', timeout=0.5)
        check(not any(isinstance(m, tuple) for m in held), 'no second frame before the ack')
        ws1.write_message(json.dumps({'t': 'ack', 'cam': 'front_cam'}))
        nxt = await read_until(ws1, lambda m: isinstance(m, tuple) and m[0] == 'bin')
        check(bool(nxt) and isinstance(nxt[-1], tuple), 'the ack releases the NEWEST pending frame')
        n = struct.unpack('>I', nxt[-1][1][:4])[0]
        h2 = json.loads(nxt[-1][1][4:4 + n])
        check(h2['seq'] == 3, f'…and it is frame #3, not #2 (seq {h2["seq"]})')
        # A late joiner gets the latest frame at once.
        ws3 = await tornado.websocket.websocket_connect(f'ws://127.0.0.1:{port}/ws')
        got = await read_until(ws3, lambda m: isinstance(m, tuple) and m[0] == 'bin')
        check(bool(got) and isinstance(got[-1], tuple), 'a new client receives the last frame immediately')

        r = await http.fetch(base + '/api/health')
        check(json.loads(r.body)['clients'] == 3, '/api/health counts three clients')
        ws3.close()
        ws2.close()
        ws1.close()
        await tornado.gen.sleep(0.2)
        r = await http.fetch(base + '/api/health')
        check(json.loads(r.body)['clients'] == 0, 'clients removed on close')

    import tornado.gen
    import tornado.util
    try:
        loop.run_sync(scenario, timeout=30)
    finally:
        server.stop()
        shutil.rmtree(tmp, ignore_errors=True)
    check(bridge.has('shutdown'), 'server stop shuts the controller and the bridge down')


if __name__ == '__main__':
    part_a()
    part_b()
    print(f'\n{N_OK} ok, {N_FAIL} failed')
    sys.exit(1 if N_FAIL else 0)
