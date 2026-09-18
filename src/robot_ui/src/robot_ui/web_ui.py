#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UiController — the operator UI's behaviour, with no toolkit in it.

This is MainWindow's logic (2026-09-15) lifted out of the Qt widgets so it
can drive a web page instead: capture → save → Ra, the Basler preview loop,
jog / move / standoff, task and lift commands, manual base moves, the
map-calibration and hand-eye panels, hot-reloadable scripts, STOP ALL, the
log. Every browser that connects sees ONE shared state (this object), and
any of them may act — the robot has one preview, one lamp hold, one
calibration session, whoever pressed the button.

It owns no device (RosBridge is the only way to a node, as before) and no
socket: it talks to the outside through a `sink` with three methods,

    sink.state(kind, key, value)    kind 'state' | 'event' | 'ui'
    sink.log(line)                  one timestamped log line
    sink.frame(cam, jpeg, header)   an encoded image (bytes + dict)

which web_server implements over WebSocket and the offline check records.
Calls into the bridge that BLOCK (a capture is seconds, a calibration
session minutes) run on a thread pool, exactly as CallWorker did; results
come back through `_run`'s on_done and are logged the same way
(`[label] ok: message` / `[label] FAILED: message`).

Shared UI state (`ui()`) is a plain dict, patched with `_patch_ui`; every
patch is broadcast, and a new client gets the whole dict in its hello.
Per-client things — which camera is in the main slot, the form values —
stay in the browser.
"""

import json
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

import cv2
import numpy as np

from robot_ui import paths
from robot_ui.plugin_runner import PluginRunner
from robot_ui.ros_bridge import ARM_AXES, STREAM_CAMERAS

# 900 px is what inference_node centre-crops to; drawn on the Basler view
# so what the operator frames is what the model sees.
CENTRE_BOX_PX = 900
# Lift spinbox ceiling in the Qt window (soft_max 6900 * 0.04976077 = 343.35).
LIFT_MAX_MM = 343.0
LOG_KEEP = 500

# (plan file, ref file, kind). The ref file ALWAYS travels with the plan:
# both plates carry cross tags with the same ids 0-5, so a mixed pair
# shifts every result by 3.89 m.
CALIB_PLANS = (
    ('정반 1  (zones B+C, 26 tags)',
     'calibration_plan_plate1.yaml', 'reference_tags.yaml', ''),
    ('정반 2  (zones D+E, 25 tags)',
     'calibration_plan_plate2.yaml', 'reference_tags_plate2.yaml', ''),
    ('정반 1  YAW SWEEP  (hand-eye vs front_cam, 6 entries)',
     'calibration_plan_plate1_yawsweep.yaml', 'reference_tags.yaml', 'sweep'),
    ('정반 2  YAW SWEEP  (hand-eye vs front_cam, 6 entries)',
     'calibration_plan_plate2_yawsweep.yaml', 'reference_tags_plate2.yaml',
     'sweep'),
)
CALIB_NOTES = {
    'sweep': (
        '⚠️ NOT a calibration — a DIAGNOSTIC. One tag, one cross tag, 6 '
        'camera yaws; the base must not move. front_cam cancels by '
        'construction, so the spread is arm-side only: large ⇒ HAND-EYE '
        'is wrong, ~0 ⇒ front_cam is. Analyse with:  rosrun '
        'path_tag_locator analyse_yaw_sweep.py <session_dir>'),
}
CALIB_HINT = ('roslaunch path_tag_locator path_tag_locator.launch')
HANDEYE_HINT = ('roslaunch path_tag_locator path_tag_locator.launch '
                'use_handeye_calib:=true')


class FrameEncoder:
    """Turns the bridge's numpy frames into rate-limited JPEGs.

    One encoder thread for all cameras: a frame is encoded ONCE and handed
    to the sink, which fans it out to every client (encoding is independent
    of the client count). Only the newest frame of a camera is ever
    encoded — the queue depth is one, like the Qt views' set_frame — and a
    camera is skipped while (a) nobody is connected or (b) fewer than
    1/max_fps seconds have passed since its last packet. Frames are
    downscaled to `max_width` for the wire; the CONTROLLER keeps the
    full-resolution Basler frame for the ROI crop and the save.
    """

    def __init__(self, sink, max_width=1400, max_fps=10.0, quality=80):
        self._sink = sink
        self.max_width = int(max_width)
        self.max_fps = float(max_fps)
        self.quality = int(quality)
        self._lock = threading.Lock()
        self._pending = {}          # cam -> frame (newest only)
        self._last_sent = {}        # cam -> monotonic
        self._seq = {}
        self._latest_packet = {}    # cam -> (jpeg, header) for late joiners
        self._wake = threading.Event()
        self._stop = threading.Event()
        self.wanted = lambda: True  # replaced by the server: any client?
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name='ui-frame-encoder')
        self._thread.start()

    def push(self, cam, frame):
        with self._lock:
            self._pending[cam] = frame
        self._wake.set()

    def latest_packets(self):
        with self._lock:
            return dict(self._latest_packet)

    def stop(self):
        self._stop.set()
        self._wake.set()

    @staticmethod
    def encode(frame, max_width, quality):
        """(jpeg bytes, header dict). Mono stays single-channel."""
        h, w = frame.shape[:2]
        out = frame
        if max_width and w > max_width:
            scale = max_width / float(w)
            out = cv2.resize(frame, (max_width, max(1, int(round(h * scale)))),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode('.jpg', out,
                               [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            raise RuntimeError('imencode failed')
        sh, sw = out.shape[:2]
        return bytes(buf), {'w': sw, 'h': sh, 'ow': w, 'oh': h,
                            'mono': frame.ndim == 2}

    def _loop(self):
        while not self._stop.is_set():
            self._wake.wait(0.2)
            self._wake.clear()
            with self._lock:
                items = list(self._pending.items())
                self._pending.clear()
            if not items:
                continue
            if not self.wanted():
                continue
            now = time.monotonic()
            min_dt = 1.0 / self.max_fps if self.max_fps > 0 else 0.0
            for cam, frame in items:
                last = self._last_sent.get(cam, 0.0)
                if now - last < min_dt and cam != 'captured':
                    # Too soon: keep it as pending so the next wake sends
                    # the newest frame rather than dropping the camera.
                    with self._lock:
                        self._pending.setdefault(cam, frame)
                    self._wake.set()
                    continue
                try:
                    jpeg, header = self.encode(frame, self.max_width,
                                               self.quality)
                except Exception:
                    continue
                self._seq[cam] = self._seq.get(cam, 0) + 1
                header.update({'cam': cam, 'seq': self._seq[cam]})
                self._last_sent[cam] = time.monotonic()
                with self._lock:
                    self._latest_packet[cam] = (jpeg, header)
                self._sink.frame(cam, jpeg, header)
            # A rate-limited camera set the wake flag; pace the loop.
            if self._wake.is_set() and min_dt > 0:
                time.sleep(min(min_dt, 0.05))


class UiController:
    """MainWindow without the window. See the module docstring."""

    def __init__(self, bridge, sink, plugin_dir=None, save_dir=None,
                 roi_config=None, stream_max_width=1400, stream_fps=10.0,
                 jpeg_quality=80, calib_poll_s=3.0):
        self.bridge = bridge
        self.sink = sink
        self._pool = ThreadPoolExecutor(max_workers=8,
                                        thread_name_prefix='ui-call')
        self._lock = threading.RLock()
        self._alive = True

        self.save_dir = save_dir or paths.DEFAULT_SAVE_DIR
        self._roi_config = roi_config or os.path.join(paths.PKG_DIR,
                                                      'roi_config.json')
        self._log_lines = []
        self._frames = {}           # cam -> newest full-res frame
        self._last_capture = None
        self._arm_pose = [0.0] * 6
        self._busy_calls = 0
        self._preview_calls = 0
        self._preview_hz = 5.0
        self._preview_on = False
        self._preview_paused = False
        self._preview_wake = threading.Event()
        self._preview_thread = None
        self._calib_running = False
        self._calib_counts = {'ok': 0, 'fail': 0, 'degraded': 0}
        self._calib_online = None
        self._handeye_online = None

        self.encoder = FrameEncoder(sink, max_width=stream_max_width,
                                    max_fps=stream_fps, quality=jpeg_quality)

        self.plugins = PluginRunner(plugin_dir or paths.PLUGIN_DIR, bridge,
                                    log=self.append_log)

        # ---- the shared UI state every client renders ----
        self._ui = {
            'preview_on': False,
            'preview_hz': 5.0,
            'lamp_on': False,
            'capture_enabled': True,
            'roi': self._load_roi(),
            'centre_box': CENTRE_BOX_PX,
            'save_dir': self.save_dir,
            'ra_text': 'Ra —',
            'cam_on': {name: True for name in STREAM_CAMERAS},
            'cam_overlay': {name: True for name in STREAM_CAMERAS},
            'standoff_inflight': False,
            'mobile_inflight': False,
            'calib': {
                'plans': [p[0] for p in CALIB_PLANS],
                'kinds': [p[3] for p in CALIB_PLANS],
                'notes': CALIB_NOTES,
                'online': None,
                'running': False,
                'state': 'idle',
                'counts': dict(self._calib_counts),
                'last': '—',
            },
            'handeye': {'online': None, 'state': 'samples: —', 'last': '—'},
            'plugins': {'dir': self.plugins.plugin_dir, 'names': [],
                        'running': False},
            'lift_max_mm': LIFT_MAX_MM,
        }

        self._connect_bridge()
        self.append_log('[UI] ready — this page owns no device; every action '
                        'goes through the node that does.')
        self.refresh_plugins()

        # Periodic master-registry poll (cheap XML-RPC lookup, no service
        # call) so the operator sees "nodes offline" BEFORE pressing START.
        self._calib_poll_s = float(calib_poll_s)
        self._poll_thread = None
        if self._calib_poll_s > 0:
            self._poll_thread = threading.Thread(
                target=self._poll_loop, daemon=True, name='ui-calib-poll')
            self._poll_thread.start()

    # ==========================================================
    # STATE ACCESS (for the server's hello)
    # ==========================================================
    def ui(self):
        with self._lock:
            return json.loads(json.dumps(self._ui))

    def log_lines(self):
        with self._lock:
            return list(self._log_lines)

    def cached_states(self):
        get = getattr(self.bridge, 'cached_states', None)
        return dict(get()) if get else {}

    def latest_frames(self):
        return self.encoder.latest_packets()

    @staticmethod
    def cameras():
        return ['basler'] + list(STREAM_CAMERAS)

    # ==========================================================
    # BRIDGE WIRING — every bridge signal becomes a sink message
    # ==========================================================
    def _connect_bridge(self):
        b = self.bridge
        b.image_received.connect(self._on_image)
        b.arm_state.connect(self._on_arm_state)
        b.lamp_state.connect(self._on_lamp_state)
        b.task_state.connect(self._on_task_state)
        b.calib_progress.connect(self._on_calib_progress)
        b.scan_progress.connect(self._on_scan_progress)
        b.handeye_progress.connect(self._on_handeye_progress)
        b.tag_ids.connect(self._on_tag_ids)
        b.log.connect(self.append_log)
        # Plain state: forwarded as-is; the browser renders the chips.
        for name in ('task_list', 'lift_state', 'mobile_state',
                     'battery_state', 'estop_state', 'camera_state',
                     'standoff_state'):
            getattr(b, name).connect(
                lambda value, n=name: self.sink.state('state', n, value))

    def _on_image(self, name, bgr):
        if not self._alive:
            return
        with self._lock:
            self._frames[name] = bgr
        self.encoder.push(name, bgr)

    def _on_arm_state(self, state):
        if state.get('pose_valid'):
            with self._lock:
                self._arm_pose = list(state['tcp_pose'])
        self.sink.state('state', 'arm_state', state)

    def _on_lamp_state(self, on):
        """/camera/lamp_state is the truth; the checkbox follows it."""
        self._patch_ui({'lamp_on': bool(on)})
        self.sink.state('state', 'lamp_state', bool(on))

    def _on_task_state(self, state):
        note = state.get('note')
        if note:
            self.append_log(f'[task] {state.get("state")} — {note}')
        self.sink.state('state', 'task_state', state)

    def _on_tag_ids(self, cam, ids):
        self.sink.state('event', 'tag_ids', {'cam': cam, 'ids': list(ids)})

    def _on_scan_progress(self, ev):
        """Per-point events from a running scan: one log line per finished
        point (a FAILED one carries the arm's reason); the SCAN chip is
        rendered by the browser from the same event."""
        phase = ev.get('phase')
        idx, total = ev.get('index', 0), ev.get('total', 0)
        n_ok, n_fail = ev.get('n_ok', 0), ev.get('n_fail', 0)
        if phase == 'start':
            self.append_log(f"[scan] start: {total} points "
                            f"({ev.get('scan_points', total)} to scan)")
        elif phase == 'done':
            if ev.get('scan', True):
                ra = ev.get('ra_mean')
                ra_s = f" ra={float(ra):.4f}" if ra is not None else ''
                self.append_log(
                    f"[scan] {idx}/{total} pt {ev.get('point_id', '?')} "
                    f"g{ev.get('group_id', '?')}: OK{ra_s}  "
                    f"{ev.get('message', '')}")
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
        elif phase == 'finished':
            tail = ' (cancelled)' if ev.get('cancelled') else ''
            self.append_log(
                f"[scan] finished{tail}: {n_ok} ok, {n_fail} failed of {total}")
        self.sink.state('event', 'scan_progress', ev)

    def _on_calib_progress(self, entry):
        """Per-tag status from a running map-calibration session."""
        status = entry.get('status', '?')
        tag = entry.get('tag', '?')
        with self._lock:
            c = self._calib_counts
            if status == 'ok':
                degraded = bool(entry.get('degraded'))
                note = ' (DEGRADED)' if degraded else ''
                c['ok'] += 1
                if degraded:
                    c['degraded'] += 1
                last = ('tag %s: OK%s  (%.3f, %.3f, %.3f)'
                        % (tag, note, entry.get('x', 0), entry.get('y', 0),
                           entry.get('z', 0)))
                line = (f"[calib] tag {tag}: OK{note}  x={entry.get('x', 0):.3f} "
                        f"y={entry.get('y', 0):.3f}")
            elif status == 'fail':
                c['fail'] += 1
                last = 'tag %s: FAIL — %s' % (tag, entry.get('error', '')[:120])
                line = f"[calib] tag {tag}: FAIL — {entry.get('error', '')}"
            else:
                last = 'tag %s: %s' % (tag, status)
                line = f'[calib] tag {tag}: {status}'
            counts = dict(c)
        self.append_log(line)
        self._patch_ui({'calib': {'counts': counts, 'last': last}})
        self.sink.state('event', 'calib_progress', entry)

    def _on_handeye_progress(self, ev):
        """Sweep events from /handeye_calib/progress."""
        phase = ev.get('phase')
        patch = {}
        n = ev.get('n_samples')
        if n is not None:
            patch['state'] = f'samples: {n}'
        if phase == 'align':
            patch['last'] = (
                f"squaring up: iteration {ev.get('iteration')}, "
                f"xy {ev.get('xy_mm', 0):.1f} mm, tilt {ev.get('tilt_deg', 0):.2f}°")
        elif phase == 'diverged':
            self.append_log(f"[handeye] {ev.get('reason', 'square-up diverged')} — "
                            "bootstrapping a provisional hand-eye")
            patch['last'] = 'square-up diverged (camera remounted?) — bootstrapping'
        elif phase == 'bootstrap':
            idx, total = ev.get('index', 0), ev.get('total', 0)
            ok = ev.get('ok')
            if ev.get('label') == 'solved':
                t = ev.get('t_mm') or [0, 0, 0]
                self.append_log(f"[handeye] bootstrap: provisional hand-eye from "
                                f"{ev.get('n_bootstrap')} samples, t = ({t[0]:.0f}, {t[1]:.0f}, {t[2]:.0f}) mm")
                patch['last'] = f"bootstrap: solved from {ev.get('n_bootstrap')} samples — squaring up"
            elif ok is None:
                patch['last'] = f"bootstrap: {total} views about the flange axes (tag at {ev.get('z_m', 0):.2f} m)"
            else:
                patch['last'] = (f"bootstrap: {idx}/{total} {ev.get('label', '')} — "
                                 f"{'captured' if ok else 'skipped: ' + str(ev.get('reason', ''))}")
                if not ok:
                    self.append_log(f"[handeye] bootstrap view {idx}/{total} "
                                    f"{ev.get('label', '')}: {ev.get('reason', '')}")
        elif phase == 'start':
            self.append_log(
                f"[handeye] sweep: {ev.get('n_planned')} views planned, "
                f"{ev.get('n_rejected')} rejected {ev.get('rejected') or ''}"
                + (" (aimed by the bootstrap hand-eye)" if ev.get('aim_source') == 'bootstrap' else ''))
            patch['last'] = f"sweep: 0/{ev.get('n_planned')} views"
        elif phase == 'sample':
            idx, total = ev.get('index', 0), ev.get('total', 0)
            ok = ev.get('ok')
            patch['last'] = (
                f"sweep: {idx}/{total} {ev.get('label', '')} — "
                f"{'captured' if ok else 'skipped: ' + str(ev.get('reason', ''))}"
                f"  (captured {ev.get('n_captured', 0)}, skipped {ev.get('n_skipped', 0)})")
            if not ok:
                self.append_log(f"[handeye] view {idx}/{total} {ev.get('label', '')}: "
                                f"{ev.get('reason', '')}")
        elif phase == 'finished':
            self.append_log(f"[handeye] {ev.get('summary', 'sweep finished')}")
            patch['last'] = ev.get('summary', 'sweep finished')
        if patch:
            self._patch_ui({'handeye': patch})
        self.sink.state('event', 'handeye_progress', ev)

    # ==========================================================
    # SHARED UI STATE
    # ==========================================================
    def _patch_ui(self, patch):
        """Deep-merge one level of nested dicts, then broadcast the patch."""
        with self._lock:
            for key, value in patch.items():
                if isinstance(value, dict) and isinstance(self._ui.get(key), dict):
                    self._ui[key].update(value)
                else:
                    self._ui[key] = value
        self.sink.state('ui', None, patch)

    def _update_busy(self):
        with self._lock:
            enabled = self._busy_calls - self._preview_calls == 0
            changed = enabled != self._ui.get('capture_enabled')
        if changed:
            self._patch_ui({'capture_enabled': enabled})

    # ==========================================================
    # WORK OFF THE CALLER'S THREAD
    # ==========================================================
    def _run(self, fn, *args, label=None, on_done=None, on_error=None,
             preview=False, **kwargs):
        """Run a blocking bridge call on the pool and log its result — the
        CallWorker of the Qt window. Returns the Future, whose result is the
        call's return value (or its exception)."""
        with self._lock:
            self._busy_calls += 1
            if preview:
                self._preview_calls += 1
        self._update_busy()

        def _job():
            try:
                result = fn(*args, **kwargs)
            except Exception as e:      # noqa: BLE001 — reported, not fatal
                message = f'{type(e).__name__}: {e}'
                self._release(preview)
                self.append_log(f'[{label or "call"}] ERROR: {message}')
                if on_error is not None:
                    on_error(message)
                raise
            self._release(preview)
            if label:
                if isinstance(result, tuple) and len(result) >= 2:
                    ok, message = result[0], result[1]
                    self.append_log(f'[{label}] {"ok" if ok else "FAILED"}: '
                                    f'{message}')
                else:
                    self.append_log(f'[{label}] done')
            if on_done is not None:
                on_done(result)
            return result

        return self._pool.submit(_job)

    def _release(self, preview):
        with self._lock:
            self._busy_calls -= 1
            if preview:
                self._preview_calls -= 1
        self._update_busy()

    @staticmethod
    def _result(res):
        """Normalise a bridge (ok, message[, extra]) into a reply dict."""
        if isinstance(res, tuple) and len(res) >= 2:
            return {'ok': bool(res[0]), 'message': str(res[1])}
        if isinstance(res, dict):
            return res
        return {'ok': True, 'message': '' if res is None else str(res)}

    # ==========================================================
    # RPC SURFACE — every `api_*` is callable from a browser
    # ==========================================================
    # Each returns something JSON-serialisable; the server runs it on the
    # pool and replies to the caller. Long motions return when the motion
    # is done (like the Qt buttons re-enabling), so a browser awaiting the
    # reply sees the real completion.

    # ---------- camera panes ----------
    def api_set_stream_source(self, cam, overlay):
        res = self.bridge.set_stream_source(cam, bool(overlay))
        self.append_log(f'[{cam} {"overlay" if overlay else "raw"}] '
                        f'{"ok" if res[0] else "FAILED"}: {res[1]}')
        if res[0]:
            self._patch_ui({'cam_overlay': {cam: bool(overlay)}})
        return self._result(res)

    def api_set_stream_camera_enabled(self, cam, on):
        res = self.bridge.set_stream_camera_enabled(cam, bool(on))
        self.append_log(f'[{cam} {"on" if on else "off"}] '
                        f'{"ok" if res[0] else "FAILED"}: {res[1]}')
        self._patch_ui({'cam_on': {cam: bool(on)}})
        return self._result(res)

    # ---------- Collect ----------
    def api_set_preview(self, on):
        on = bool(on)
        with self._lock:
            if on == self._preview_on:
                return {'ok': True, 'message': 'unchanged'}
            self._preview_on = on
        if on:
            self.bridge.set_camera_active(True)
            self._start_preview_thread()
            self.append_log('[UI] preview on — basler held open'
                            + (' (lamp held on)' if self._ui.get('lamp_on')
                               else ', lamp off'))
        else:
            self._preview_wake.set()
            self.bridge.set_camera_active(False)
            self.append_log('[UI] preview off — basler released')
        self._patch_ui({'preview_on': on})
        return {'ok': True, 'message': 'preview on' if on else 'preview off'}

    def api_set_preview_rate(self, hz):
        hz = max(0.2, min(5.0, float(hz)))
        with self._lock:
            self._preview_hz = hz
        self._preview_wake.set()
        self._patch_ui({'preview_hz': hz})
        return {'ok': True, 'message': f'{hz:g} Hz'}

    def _start_preview_thread(self):
        if self._preview_thread is not None and self._preview_thread.is_alive():
            self._preview_wake.set()
            return
        self._preview_thread = threading.Thread(
            target=self._preview_loop, daemon=True, name='ui-preview')
        self._preview_thread.start()

    def _preview_loop(self):
        """The Basler is normally CLOSED, so a "live" view is a deliberate
        poll of the capture service (lamp OFF) at preview_hz. A tick is
        skipped while any other call is in flight — queuing would build an
        unbounded backlog of stale frames — and the loop is PAUSED, not
        stopped, for a capture (see api_capture)."""
        while self._alive:
            with self._lock:
                on = self._preview_on
                hz = self._preview_hz
                paused = self._preview_paused
                busy = self._busy_calls > 0
            if not on:
                return
            period = 1.0 / max(0.2, hz)
            if paused or busy:
                self._preview_wake.wait(0.05)
                self._preview_wake.clear()
                continue
            t0 = time.monotonic()
            with self._lock:
                self._busy_calls += 1
                self._preview_calls += 1
            try:
                ok, _message, frames = self.bridge.capture(1, -1.0, False)
            except Exception as e:      # noqa: BLE001
                ok, frames = False, []
                self.append_log(f'[preview] ERROR: {type(e).__name__}: {e}')
            finally:
                self._release(preview=True)
            if ok and frames:
                self._on_image('basler', frames[-1])
            wait = period - (time.monotonic() - t0)
            if wait > 0:
                self._preview_wake.wait(wait)
                self._preview_wake.clear()

    def api_set_lamp(self, on):
        """Ask the node to HOLD the VISION lamp; the checkbox follows
        /camera/lamp_state, not the click."""
        self.bridge.set_vision_lamp(bool(on))
        return {'ok': True, 'message': 'requested'}

    def api_set_roi(self, rect):
        """rect = {x, y, w, h} in FULL-RES image pixels, or null to clear."""
        roi = None
        if rect:
            try:
                roi = {k: int(rect[k]) for k in ('x', 'y', 'w', 'h')}
            except Exception:
                return {'ok': False, 'message': 'roi needs x, y, w, h'}
            if roi['w'] <= 4 or roi['h'] <= 4:
                roi = None
        self._save_roi(roi)
        self._patch_ui({'roi': roi})
        return {'ok': True, 'message': 'roi set' if roi else 'roi cleared'}

    def api_set_save_dir(self, directory):
        directory = str(directory).strip() or paths.DEFAULT_SAVE_DIR
        with self._lock:
            self.save_dir = directory
        self._patch_ui({'save_dir': directory})
        return {'ok': True, 'message': directory}

    def api_capture(self, opts=None):
        """CAPTURE: grab through the owner node, then save and/or score.

        opts: prefix, samples, led, save, infer, roi (use the ROI crop for
        inference), save_dir. Blocks until the capture returned (the Ra
        arrives later on ui.ra_text). Returns {ok, message, frames, saved}.
        """
        opts = dict(opts or {})
        samples = int(opts.get('samples', 1))
        led = bool(opts.get('led', True))
        with self._lock:
            preview_on = self._preview_on
            if preview_on:
                # PAUSE the preview for the shot instead of switching it
                # off: the device stays held open and no preview grab is
                # queued behind the real one, so lamp-off and lamp-on frames
                # cannot interleave in one shot.
                self._preview_paused = True
        self.append_log('[UI] capture requested'
                        + (' (preview paused)' if preview_on else ''))
        with self._lock:
            self._busy_calls += 1
        self._update_busy()
        try:
            ok, message, frames = self.bridge.capture(samples, -1.0, led)
        except Exception as e:      # noqa: BLE001
            ok, message, frames = False, f'{type(e).__name__}: {e}', []
        finally:
            self._release(preview=False)
            with self._lock:
                self._preview_paused = False
            self._preview_wake.set()
        if not ok or not frames:
            self.append_log(f'[capture] FAILED: {message}')
            return {'ok': False, 'message': message, 'frames': 0, 'saved': []}
        self.append_log(f'[capture] {len(frames)} frame(s): {message}')
        with self._lock:
            self._last_capture = frames[-1]
            self._frames['basler'] = frames[-1]
        self.encoder.push('basler', frames[-1])
        self.encoder.push('captured', frames[-1])

        saved = []
        if opts.get('save', True):
            saved = self._save_frames(frames, opts.get('save_dir'),
                                      opts.get('prefix'))
        if opts.get('infer', True):
            image = (self._cropped_roi(frames[-1]) if opts.get('roi')
                     else frames[-1])
            tag = os.path.basename(saved[-1]) if saved else ''
            self._patch_ui({'ra_text': 'Ra … predicting'})
            self._run(self.bridge.predict_ra, image, tag, on_done=self._on_ra)
        return {'ok': True, 'message': message, 'frames': len(frames),
                'saved': saved}

    def _cropped_roi(self, frame):
        roi = self._ui.get('roi')
        if frame is None or not roi:
            return frame
        h, w = frame.shape[:2]
        x0 = max(0, min(w - 1, roi['x']))
        y0 = max(0, min(h - 1, roi['y']))
        x1 = max(x0 + 1, min(w, roi['x'] + roi['w']))
        y1 = max(y0 + 1, min(h, roi['y'] + roi['h']))
        return frame[y0:y1, x0:x1]

    def _save_frames(self, frames, directory=None, prefix=None):
        directory = (directory or '').strip() or self.save_dir
        prefix = (prefix or '').strip() or 'capture'
        try:
            os.makedirs(directory, exist_ok=True)
        except Exception as e:
            self.append_log(f'[save] cannot create {directory}: {e}')
            return []
        # Timestamped names, not scan-for-the-next-free-index (O(n) per
        # shot and silently reuses a number if an old file is deleted).
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
        if not result.get('success'):
            self._patch_ui({'ra_text': 'Ra — failed'})
            self.append_log(f'[inference] FAILED: {result.get("message")}')
            return
        parts = ' | '.join(
            f'{name}={value:.4f}' if not np.isnan(value) else f'{name}=NaN'
            for name, value in result['models'])
        self._patch_ui({'ra_text': f'Ra {result["ra"]:.4f}    ({parts})'})
        self.append_log(f'[inference] {parts}  in {result["elapsed_s"]:.2f}s'
                        + (f'  tag={result["tag"]}' if result.get('tag') else ''))

    # ---------- Arm ----------
    def api_arm_jog(self, axis, delta, vel=20.0):
        if axis not in ARM_AXES:
            return {'ok': False, 'message': f'unknown axis {axis}'}
        delta = float(delta)
        res = self._run(self.bridge.arm_jog, axis, delta, float(vel),
                        label=f'jog {axis} {delta:+g}').result()
        return self._result(res)

    def api_arm_move_cart(self, fields, vel=20.0):
        """fields: six entries, each a number or null/'' = keep the live
        pose on that axis (single-axis repositioning without typing all
        six)."""
        if not isinstance(fields, (list, tuple)) or len(fields) != 6:
            return {'ok': False, 'message': 'need 6 target fields'}
        with self._lock:
            live = list(self._arm_pose)
        pose = []
        for i, axis in enumerate(ARM_AXES):
            v = fields[i]
            if v is None or (isinstance(v, str) and not v.strip()):
                pose.append(live[i])
                continue
            try:
                pose.append(float(v))
            except (TypeError, ValueError):
                return {'ok': False,
                        'message': f'{axis} is not a number: "{v}"'}
        res = self._run(self.bridge.arm_move_cart, pose, float(vel),
                        label='move_cart').result()
        return self._result(res)

    def api_arm_pose(self):
        with self._lock:
            return {'ok': True, 'pose': list(self._arm_pose)}

    def api_arm_home(self):
        return self._result(self._run(self.bridge.arm_home,
                                      label='arm home').result())

    def api_arm_cancel(self):
        self.bridge.arm_cancel()
        return {'ok': True, 'message': 'cancel published'}

    def api_arm_standoff(self, target_mm=10.0):
        target = float(target_mm)
        self._patch_ui({'standoff_inflight': True})
        try:
            res = self._run(self.bridge.arm_standoff, target,
                            label=f'standoff -> {target:g} mm').result()
        except Exception as e:      # noqa: BLE001
            res = (False, f'{type(e).__name__}: {e}')
        finally:
            self._patch_ui({'standoff_inflight': False})
        return self._result(res)

    # ---------- Task / lift ----------
    def api_task_command(self, text):
        """One /task_command line: TASK <name>, GOTO <n>, CHARGE, UNDOCK,
        RELOAD_TASKS, STOP, STATE, or anything typed raw."""
        text = str(text).strip()
        if not text:
            return {'ok': False, 'message': 'empty command'}
        ok = self.bridge.send_task_command(text)
        return {'ok': bool(ok), 'message': text}

    def api_lift_goto(self, mm):
        mm = max(0.0, min(LIFT_MAX_MM, float(mm)))
        self.bridge.lift_goto_mm(mm)
        self.append_log(f'[lift] /lifter/height_cmd <- {mm:.1f} mm')
        return {'ok': True, 'message': f'{mm:.1f} mm requested'}

    def api_lift_home(self):
        return self._result(self._run(self.bridge.lift_home,
                                      label='lift origin homing').result())

    def api_lift_stop(self):
        return self._result(self._run(self.bridge.lift_stop,
                                      label='lift stop').result())

    # ---------- Mobile ----------
    def api_mobile_drive(self, distance_m, speed=None):
        dist = float(distance_m)
        self._patch_ui({'mobile_inflight': True})
        try:
            res = self._run(self.bridge.mobile_drive, dist,
                            float(speed) if speed else None,
                            label=f'base drive {dist:+.3f} m').result()
        except Exception as e:      # noqa: BLE001
            res = (False, f'{type(e).__name__}: {e}')
        finally:
            self._patch_ui({'mobile_inflight': False})
        return self._result(res)

    def api_mobile_pivot(self, angle_deg, speed=None):
        angle = float(angle_deg)
        self._patch_ui({'mobile_inflight': True})
        try:
            res = self._run(self.bridge.mobile_pivot, angle,
                            float(speed) if speed else None,
                            label=f'base pivot {angle:+.1f} deg').result()
        except Exception as e:      # noqa: BLE001
            res = (False, f'{type(e).__name__}: {e}')
        finally:
            self._patch_ui({'mobile_inflight': False})
        return self._result(res)

    def api_mobile_cancel(self):
        return self._result(self._run(self.bridge.mobile_cancel,
                                      label='base stop').result())

    def api_mobile_clear_stop(self):
        return self._result(self._run(self.bridge.mobile_clear_stop,
                                      label='base clear_stop').result())

    # ---------- Calibration ----------
    def _poll_loop(self):
        while self._alive:
            try:
                self.poll_calib_nodes()
            except Exception:
                pass
            for _ in range(int(self._calib_poll_s * 10)):
                if not self._alive:
                    return
                time.sleep(0.1)

    def poll_calib_nodes(self):
        he = bool(self.bridge.handeye_online())
        online = bool(self.bridge.calib_nodes_online())
        patch = {}
        if he != self._handeye_online:
            self._handeye_online = he
            patch['handeye'] = {'online': he}
        if online != self._calib_online:
            self._calib_online = online
            patch['calib'] = {'online': online}
        if patch:
            self._patch_ui(patch)
        return online, he

    @staticmethod
    def calib_paths(index):
        label, plan, ref, kind = CALIB_PLANS[int(index)]
        base = '$(find path_tag_locator)/config/'
        return base + plan, base + ref, kind, label

    def api_calib_start(self, plan_index=0, dry_run=True):
        with self._lock:
            if self._calib_running:
                self.append_log('[calib] a session is already running')
                return {'ok': False, 'message': 'a session is already running'}
        if not self.bridge.calib_nodes_online():
            self.append_log(
                '[calib] calibration nodes are NOT running — start them '
                f'first:  {CALIB_HINT}')
            self.poll_calib_nodes()
            return {'ok': False, 'message': 'calibration nodes offline'}
        try:
            plan_path, ref_path, kind, label = self.calib_paths(plan_index)
        except (IndexError, ValueError):
            return {'ok': False, 'message': f'no plan {plan_index}'}
        dry = bool(dry_run)
        with self._lock:
            self._calib_running = True
            self._calib_counts = {'ok': 0, 'fail': 0, 'degraded': 0}
            counts = dict(self._calib_counts)
        self._patch_ui({'calib': {
            'running': True, 'counts': counts, 'last': '—',
            'state': ('DRY RUN' if dry else 'RUNNING') + '  ' + label}})
        self.append_log('[calib] session start (%s, dry=%s)' % (label, dry))

        def _done(result):
            ok, message, report = result
            state = (('finished OK' if ok else 'finished with FAILURES')
                     + '  —  ' + message.split(';')[0])
            if report.get('output_yaml_path'):
                self.append_log('[calib] output: %s' % report['output_yaml_path'])
            if kind and not dry:
                self.append_log(
                    '[calib] %s session recorded under '
                    '<ws>/log/path_tag_locator/calibrate/<newest>/  — analyse '
                    'with:  rosrun path_tag_locator analyse_yaw_sweep.py '
                    '<that dir>' % kind)
            self._finish_calib(state)

        self._run(self.bridge.run_map_calibration,
                  dry_run=dry, plan_path=plan_path, ref_tags_path=ref_path,
                  label='calibration', on_done=_done,
                  on_error=lambda m: self._finish_calib(f'ERROR — {m}'))
        return {'ok': True, 'message': 'session started'}

    def _finish_calib(self, state):
        with self._lock:
            self._calib_running = False
        self._patch_ui({'calib': {'running': False, 'state': state}})

    def api_calib_cancel(self):
        return self._result(self._run(self.bridge.cancel_map_calibration,
                                      label='calib-cancel').result())

    def api_calib_locate(self, tag_id):
        tag = int(tag_id)
        result = self._run(self.bridge.locate_path_tag, tag_b_id=tag,
                           label='locate').result()
        if result.get('success'):
            x, y, z = result['position_m']
            self.append_log('[locate] tag %d: (%.4f, %.4f, %.4f) m'
                            % (tag, x, y, z))
        else:
            self.append_log('[locate] tag %d FAILED: %s'
                            % (tag, result.get('message')))
        return result

    def api_handeye_auto(self):
        if not self.bridge.handeye_online():
            self.append_log(
                f'[handeye] hand-eye node is NOT running — start it with:  '
                f'{HANDEYE_HINT}')
            return {'ok': False, 'message': 'hand-eye node offline'}
        self._patch_ui({'handeye': {'last': 'sweep requested…'}})
        return self._result(self._run(self.bridge.handeye_auto_sample,
                                      label='handeye-auto').result())

    def api_handeye_cancel(self):
        return self._result(self._run(self.bridge.handeye_cancel,
                                      label='handeye-cancel').result())

    def api_handeye_capture(self):
        res = self._run(self.bridge.handeye_capture,
                        label='handeye-capture').result()
        self.api_handeye_status()
        return self._result(res)

    def api_handeye_compute(self):
        self._patch_ui({'handeye': {'last': 'computing…'}})
        res = self._run(self.bridge.handeye_compute,
                        label='handeye-compute').result()
        if isinstance(res, tuple) and len(res) >= 2:
            ok, message = res[0], str(res[1])
            self._patch_ui({'handeye': {'last': (
                ('saved: ' if ok else 'compute FAILED: ')
                + message.split('\n')[0])}})
        self.api_handeye_status()
        return self._result(res)

    def api_handeye_load_latest(self):
        res = self._run(self.bridge.handeye_load_latest,
                        label='handeye-load').result()
        self.api_handeye_status()
        return self._result(res)

    def api_handeye_reset(self):
        res = self._run(self.bridge.handeye_reset,
                        label='handeye-reset').result()
        self.api_handeye_status()
        return self._result(res)

    def api_handeye_status(self):
        res = self._run(self.bridge.handeye_status, label=None).result()
        if isinstance(res, tuple) and len(res) >= 2 and res[0]:
            self._patch_ui({'handeye': {'state': str(res[1]).split('\n')[0]}})
        return self._result(res)

    # ---------- Scripts ----------
    def refresh_plugins(self):
        names = self.plugins.discover()
        self._patch_ui({'plugins': {'names': names,
                                    'running': self.plugins.is_running()}})
        self.append_log(f'[plugin] {len(names)} script(s) available')
        return names

    def api_plugin_refresh(self):
        return {'ok': True, 'names': self.refresh_plugins()}

    def api_plugin_run(self, name):
        name = str(name).strip()
        if not name:
            return {'ok': False, 'message': 'no script selected'}
        started = self.plugins.start(
            name, on_finished=lambda ok: self._patch_ui(
                {'plugins': {'running': False}}))
        if started:
            self._patch_ui({'plugins': {'running': True}})
        return {'ok': bool(started),
                'message': 'started' if started else 'not started'}

    def api_plugin_stop(self):
        self.plugins.cancel()
        return {'ok': True, 'message': 'cancel requested'}

    # ---------- STOP ALL ----------
    def api_stop_all(self):
        """Soft-stop every device that can move, plus any running script.
        The hardware e-stop is a PILZ relay independent of ROS; this is
        not it."""
        self.append_log('[UI] STOP ALL pressed')
        self.plugins.cancel()
        self.bridge.arm_cancel()
        self.bridge.send_task_command('STOP')
        self._run(self.bridge.mobile_stop, label='mobile stop')
        self._run(self.bridge.lift_stop, label='lift stop')
        if self._ui.get('lamp_on'):
            self.bridge.set_vision_lamp(False)
        if self._preview_on:
            self.api_set_preview(False)
        return {'ok': True, 'message': 'stop all sent'}

    # ---------- misc ----------
    def api_ping(self):
        return {'ok': True, 'message': 'pong', 'time': time.time()}

    # ==========================================================
    # ROI persistence (image coordinates, like ImageView)
    # ==========================================================
    def _load_roi(self):
        try:
            if not self._roi_config or not os.path.exists(self._roi_config):
                return None
            with open(self._roi_config, 'r') as f:
                data = json.load(f)
            if all(k in data for k in ('x', 'y', 'w', 'h')):
                return {k: int(data[k]) for k in ('x', 'y', 'w', 'h')}
        except Exception:
            pass                    # a corrupt ROI file must not stop the UI
        return None

    def _save_roi(self, roi):
        if not self._roi_config:
            return
        try:
            os.makedirs(os.path.dirname(self._roi_config), exist_ok=True)
            with open(self._roi_config, 'w') as f:
                json.dump(roi or {}, f, indent=2)
        except Exception:
            pass

    # ==========================================================
    # LOG / SHUTDOWN
    # ==========================================================
    def append_log(self, message):
        """Thread-safe: called from rospy threads, the pool, plugins."""
        stamp = datetime.now().strftime('%H:%M:%S')
        line = f'{stamp}  {message}'
        with self._lock:
            self._log_lines.append(line)
            if len(self._log_lines) > LOG_KEEP:
                del self._log_lines[:len(self._log_lines) - LOG_KEEP]
        self.sink.log(line)

    def shutdown(self):
        self.append_log('[UI] closing')
        self._alive = False
        with self._lock:
            self._preview_on = False
        self._preview_wake.set()
        self.plugins.cancel()
        self.encoder.stop()
        try:
            self.bridge.shutdown()
        except Exception:
            pass
        # Motion is deliberately NOT cancelled: closing the server is not a
        # stop request. Use STOP ALL for that.
        self._pool.shutdown(wait=False)
