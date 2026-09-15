#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Part C of the web UI check: a REAL headless Chrome drives the page.

Starts the real WebServer + UiController on a FakeBridge (from
check_web_ui.py), opens http://127.0.0.1:<port>/ in `google-chrome
--headless` and talks to the tab over the Chrome DevTools Protocol
(plain WebSocket, no node/puppeteer): every JS exception is a failure,
the chips must render the states the bridge emits, buttons must produce
the bridge calls, a pushed camera frame must be drawn on the canvas and
acknowledged, an ROI drag must reach the server in image coordinates,
a thumbnail click must swap the main view. Saves a screenshot next to
this file (web_ui_screenshot.png) so a human can look at the layout.

Run (workspace sourced, google-chrome or chromium on PATH):
    python3 tools/check_web_ui_browser.py
"""
import base64
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))
sys.path.insert(0, HERE)

import numpy as np                                    # noqa: E402
import tornado.gen                                    # noqa: E402
import tornado.ioloop                                 # noqa: E402
import tornado.websocket                              # noqa: E402

import check_web_ui as base                           # noqa: E402
from robot_ui.web_server import WebServer             # noqa: E402
from robot_ui.web_ui import UiController              # noqa: E402

check = base.check


def find_chrome():
    for name in ('google-chrome', 'google-chrome-stable', 'chromium',
                 'chromium-browser'):
        path = shutil.which(name)
        if path:
            return path
    return None


class Cdp:
    """Minimal DevTools client: one page target, sequential commands."""

    def __init__(self, ws_url):
        self.ws_url = ws_url
        self.ws = None
        self._id = 0
        self.exceptions = []
        self.console_errors = []
        self._pending = {}

    async def connect(self):
        self.ws = await tornado.websocket.websocket_connect(
            self.ws_url, max_message_size=64 << 20)
        tornado.ioloop.IOLoop.current().spawn_callback(self._pump)
        await self.cmd('Runtime.enable')
        await self.cmd('Page.enable')
        await self.cmd('Log.enable')

    async def _pump(self):
        while True:
            m = await self.ws.read_message()
            if m is None:
                return
            obj = json.loads(m)
            if 'id' in obj:
                fut = self._pending.pop(obj['id'], None)
                if fut is not None:
                    fut.set_result(obj)
            else:
                method = obj.get('method')
                if method == 'Runtime.exceptionThrown':
                    d = obj['params']['exceptionDetails']
                    self.exceptions.append(d.get('text', '') + ' ' + str(
                        (d.get('exception') or {}).get('description', '')))
                elif method == 'Runtime.consoleAPICalled' and \
                        obj['params'].get('type') == 'error':
                    self.console_errors.append(str(obj['params'].get('args')))
                elif method == 'Log.entryAdded' and \
                        obj['params']['entry'].get('level') == 'error':
                    self.console_errors.append(obj['params']['entry'].get('text'))

    async def cmd(self, method, **params):
        self._id += 1
        fut = tornado.concurrent.Future()
        self._pending[self._id] = fut
        self.ws.write_message(json.dumps({'id': self._id, 'method': method,
                                          'params': params}))
        resp = await tornado.gen.with_timeout(
            tornado.ioloop.IOLoop.current().time() + 15.0, fut)
        if 'error' in resp:
            raise RuntimeError(f'{method}: {resp["error"]}')
        return resp.get('result', {})

    async def js(self, expression):
        r = await self.cmd('Runtime.evaluate', expression=expression,
                           returnByValue=True, awaitPromise=True)
        if 'exceptionDetails' in r:
            raise RuntimeError(r['exceptionDetails'].get('text'))
        return r.get('result', {}).get('value')


async def wait_js(cdp, expression, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        v = await cdp.js(expression)
        if v:
            return v
        await tornado.gen.sleep(0.05)
    return await cdp.js(expression)


async def scenario(cdp, url, bridge, holder):
    await cdp.cmd('Page.navigate', url=url)
    ok = await wait_js(cdp, "document.readyState === 'complete' && !!document.getElementById('chip-conn')")
    check(ok, 'page loaded')
    ok = await wait_js(cdp, "document.getElementById('chip-conn').classList.contains('ok')")
    check(ok, 'WebSocket connected (conn chip green)')

    # ---- hello → chips / forms ----
    check(await wait_js(cdp, "document.getElementById('chip-battery').textContent === 'BAT 67% 52.1V'"),
          'battery chip rendered from the replayed state')
    check(await cdp.js("document.getElementById('log').textContent.includes('ready')"), 'log history shown')
    check(await cdp.js("document.getElementById('sel-calib-plan').options.length === 4"), 'four calibration plans listed')
    check(await cdp.js("document.getElementById('lbl-plugin-dir').textContent.length > 0"), 'plugin dir shown')
    check(await cdp.js("document.getElementById('tab-live').textContent === 'basler (live)' && "
                       "document.querySelector('#main-slot canvas').dataset.cam === undefined && "
                       "document.querySelectorAll('#thumb-strip .cell').length === 3"),
          'Basler in the main slot, three thumbnails')

    # ---- live states ----
    bridge.emit_state('estop_state', False)
    bridge.emit_state('arm_state', {'state': 'idle', 'busy': True, 'pose_valid': True,
                                    'tcp_pose': [1.234, -2.5, 3, 4, 5, 6.789], 'joints': [],
                                    'motion_seq': 1, 'result_message': '', 'result_success': True})
    bridge.emit_state('lift_state', {'height_mm': 149.9, 'homed': False})
    bridge.emit_state('mobile_state', {'busy': None, 'emergency_stop': True, 'visible_tags': [105],
                                       'last_known_tag': 105, 'seq': 2, 'result': {'ok': True, 'message': 'arrived'}})
    bridge.emit_state('task_state', {'state': 'SCANNING', 'task': 'scan_pose_x', 'group_index': 2,
                                     'group_total': 6, 'charge_phase': 'charging', 'charging': True,
                                     'battery_pct': 57.0})
    bridge.emit_state('standoff_state', {'raw_mm': -3.0, 'target_mm': 10.0, 'valid': True,
                                         'standoff_mm': 12.21, 'err_mm': -2.21, 'side': None})
    check(await wait_js(cdp, "document.getElementById('chip-estop').textContent === 'E-STOP clear'"), 'E-STOP chip')
    check(await wait_js(cdp, "document.getElementById('chip-arm').textContent === 'ARM BUSY' && "
                            "document.getElementById('pose-x').textContent === '1.23' && "
                            "document.getElementById('pose-rz').textContent === '6.79'"), 'ARM chip + pose fields')
    check(await wait_js(cdp, "document.getElementById('chip-lift').textContent === 'LIFT 150mm UNHOMED'"), 'LIFT chip')
    check(await wait_js(cdp, "document.getElementById('chip-mobile').textContent === 'BASE E-LATCHED' && "
                            "document.getElementById('lbl-mob-status').textContent.includes('last result: arrived')"),
          'BASE chip + status line')
    check(await wait_js(cdp, "document.getElementById('chip-task').textContent === 'TASK SCANNING scan_pose_x 2/6' && "
                            "document.getElementById('chip-charge').textContent === 'CHARGE charging 57%'"),
          'TASK + CHARGE chips')
    check(await wait_js(cdp, "document.getElementById('lbl-standoff').textContent.startsWith('standoff: 12.21 mm   (target 10: 2.21 mm too far)')"),
          'standoff line')
    bridge.scan_progress.emit({'phase': 'failed', 'index': 3, 'total': 9, 'point_id': 7, 'group_id': 106,
                               'message': 'IK failed', 'n_ok': 2, 'n_fail': 1})
    check(await wait_js(cdp, "document.getElementById('chip-scan').textContent === 'SCAN 3/9 ok 2 fail 1'"), 'SCAN chip from the event')
    bridge.tag_ids.emit('hand_cam', [0, 3])
    check(await wait_js(cdp, "[...document.querySelectorAll('.cell .tags')].some(e => e.textContent === 'tags: 0, 3')"),
          'tag ids under the hand_cam pane')
    bridge.emit_state('task_list', {'task_dir': '/ws/task/csv', 'tasks': [
        {'name': 'scan_pose_a', 'scan_mode': 'pose', 'tags': [104, 105], 'points': 12, 'traverse_points': 0,
         'lift_height_mm': 0, 'files': ['assigned_workpoints_a.csv'], 'paired_file': 'rrt_final_path_a.csv'},
        {'name': 'scan_joint_a', 'scan_mode': 'joint', 'tags': [104, 105], 'points': 12, 'traverse_points': 5,
         'lift_height_mm': 0, 'files': ['rrt_final_path_a.csv']}]})
    check(await wait_js(cdp, "document.getElementById('task-list').options.length === 2 && "
                            "document.getElementById('txt-task').value === 'scan_pose_a'"),
          'task list fills the datalist and picks the first task')
    check(await cdp.js("document.getElementById('lbl-task-detail').textContent === "
                       "'scan_pose_a: pose · tags 104,105 · 12 pts · lift 0 mm · assigned_workpoints_a.csv · paired rrt_final_path_a.csv'"),
          'task detail line')

    # ---- buttons → bridge ----
    bridge.calls.clear()
    await cdp.js("document.getElementById('btn-task').click()")
    check(base.wait_for(lambda: bridge.has('send_task_command', 'TASK scan_pose_a')), 'Send TASK → /task_command')
    await cdp.js("document.getElementById('num-goto').value = '112'; document.getElementById('btn-goto').click()")
    check(base.wait_for(lambda: bridge.has('send_task_command', 'GOTO 112')), 'Send GOTO')
    await cdp.js("document.getElementById('btn-charge').click(); document.getElementById('btn-undock').click()")
    check(base.wait_for(lambda: bridge.has('send_task_command', 'CHARGE') and bridge.has('send_task_command', 'UNDOCK')),
          'Dock & charge / Undock')
    await cdp.js("document.getElementById('txt-raw').value = 'STATE'; document.getElementById('btn-raw').click()")
    check(base.wait_for(lambda: bridge.has('send_task_command', 'STATE')), 'raw command')
    check(await wait_js(cdp, "document.getElementById('txt-raw').value === ''"), 'raw field cleared after send')
    await cdp.js("document.getElementById('num-step').value = '2.5'; "
                 "[...document.querySelectorAll('#jog-grid button')].find(b => b.dataset.axis === 'z' && b.dataset.sign === '-1').click()")
    check(base.wait_for(lambda: bridge.has('arm_jog', 'z', -2.5, 20.0)), 'jog Z − with the step')
    await cdp.js("document.getElementById('btn-fill').click(); document.getElementById('target-z').value = '99'; "
                 "document.getElementById('btn-move').click()")
    # 'Fill from current' rounds to 2 decimals, like the Qt window's fields.
    check(base.wait_for(lambda: bridge.has('arm_move_cart', (1.23, -2.5, 99.0, 4.0, 5.0, 6.79), 20.0)),
          'Fill from current + MOVE with one axis edited')
    await cdp.js("document.getElementById('btn-arm-home').click(); document.getElementById('btn-arm-cancel').click()")
    check(base.wait_for(lambda: bridge.has('arm_home') and bridge.has('arm_cancel')), 'arm home / cancel')
    await cdp.js("document.getElementById('num-standoff').value = '12'; document.getElementById('btn-standoff').click()")
    check(base.wait_for(lambda: bridge.has('arm_standoff', 12.0)), 'Auto standoff with the typed target')
    check(await wait_js(cdp, "!document.getElementById('btn-standoff').disabled", 3.0), 'standoff button re-enabled after')
    await cdp.js("document.getElementById('num-lift').value = '200'; document.getElementById('btn-lift-go').click(); "
                 "document.getElementById('btn-lift-home').click(); document.getElementById('btn-lift-stop').click()")
    check(base.wait_for(lambda: bridge.has('lift_goto_mm', 200.0) and bridge.has('lift_home') and bridge.has('lift_stop')),
          'lift Go / origin homing / stop')
    await cdp.js("document.getElementById('num-mob-dist').value = '0.25'; document.getElementById('btn-mob-rev').click()")
    check(base.wait_for(lambda: bridge.has('mobile_drive', -0.25, 0.05)), 'Reverse → mobile_drive −0.25')
    check(await wait_js(cdp, "!document.getElementById('btn-mob-fwd').disabled", 3.0), 'mobile buttons re-enabled')
    await cdp.js("document.getElementById('btn-mob-cw').click()")
    check(base.wait_for(lambda: bridge.has('mobile_pivot', -90.0, 0.2)), 'Turn right → pivot −90')
    await cdp.js("document.getElementById('btn-mob-stop').click(); document.getElementById('btn-mob-clear').click()")
    check(base.wait_for(lambda: bridge.has('mobile_cancel') and bridge.has('mobile_clear_stop')), 'Stop base / clear latch')

    # calibration
    await cdp.js("document.getElementById('sel-calib-plan').value = '2'; "
                 "document.getElementById('sel-calib-plan').dispatchEvent(new Event('change'))")
    check(await wait_js(cdp, "document.getElementById('lbl-calib-plan-note').textContent.startsWith('⚠️ NOT a calibration')"),
          'yaw-sweep plan shows the diagnostic warning')
    check(await wait_js(cdp, "document.getElementById('lbl-calib-nodes').textContent === 'nodes: ONLINE'", 4.0), 'nodes ONLINE line')
    await cdp.js("document.getElementById('btn-calib-start').click()")
    check(base.wait_for(lambda: bridge.has('run_map_calibration', True), 3.0), 'START SESSION (dry run) → run_map_calibration')
    check(await wait_js(cdp, "document.getElementById('lbl-calib-counts').textContent === 'ok 1   fail 1   degraded 0' && "
                            "document.getElementById('lbl-calib-state').textContent.startsWith('finished OK')", 5.0),
          'counts + state from the session')
    check(await cdp.js("document.getElementById('log').textContent.includes('[calib] tag 106: FAIL')"), 'calib failure in the log')
    await cdp.js("document.getElementById('btn-he-capture').click()")
    check(await wait_js(cdp, "document.getElementById('lbl-handeye-state').textContent === 'samples: 3'", 3.0),
          'hand-eye capture → status line')
    await cdp.js("document.getElementById('num-locate').value = '108'; document.getElementById('btn-locate').click()")
    check(base.wait_for(lambda: bridge.has('locate_path_tag', 108)), 'Locate')

    # scripts
    check(await cdp.js("document.getElementById('sel-plugin').options.length === 2"), 'scripts listed')
    await cdp.js("document.getElementById('sel-plugin').value = 'demo'; document.getElementById('btn-plugin-run').click()")
    check(await wait_js(cdp, "document.getElementById('log').textContent.includes('demo ran')", 3.0), 'RUN SCRIPT ran on the server')

    # camera cells
    await cdp.js("[...document.querySelectorAll('#thumb-strip .cell')][0].querySelector('input[type=checkbox]').click()")
    check(base.wait_for(lambda: bridge.has('set_stream_camera_enabled', 'front_cam', False)), 'on box → set_enabled front_cam off')
    await cdp.js("document.getElementById('chk-preview').click()")
    check(base.wait_for(lambda: bridge.has('set_camera_active', True)), 'Live preview → device held')
    check(await wait_js(cdp, "document.getElementById('chk-preview').checked"), 'preview box stays ticked (shared state)')
    await cdp.js("document.getElementById('chk-lamp').click()")
    check(base.wait_for(lambda: bridge.has('set_vision_lamp', True)), 'VISION lamp box → hold')
    check(await wait_js(cdp, "document.getElementById('chk-lamp').checked"), 'lamp box follows /camera/lamp_state')

    # ---- frames on the canvas ----
    bridge.calls.clear()
    frame = np.zeros((360, 640, 3), np.uint8)
    frame[:, :, 1] = 180
    bridge.image_received.emit('side_cam', frame)
    lit = await wait_js(cdp, """(() => {
        const c = [...document.querySelectorAll('canvas')].find(x => x.width > 0 && [...document.querySelectorAll('#thumb-strip .cell')][1].contains(x));
        if (!c) return false;
        const d = c.getContext('2d').getImageData(Math.floor(c.width/2), Math.floor(c.height/2), 1, 1).data;
        return d[1] > 100 && d[0] < 60;
    })()""", 5.0)
    check(lit, 'a pushed side_cam frame is drawn (green pixel at the canvas centre)')
    ws_client = list(holder['server'].sink.clients)
    check(len(ws_client) == 1 and ws_client[0]._credit.get('side_cam') is True,
          'the browser acknowledged the frame (credit restored)')

    # ---- capture → Last capture tab ----
    await cdp.js("document.getElementById('btn-capture').click()")
    check(base.wait_for(lambda: bridge.has('capture', 1, True), 3.0), 'CAPTURE → capture(1, lamp)')
    check(await wait_js(cdp, "!document.getElementById('shot-page').hidden", 4.0), 'Last-capture tab raised')
    check(await wait_js(cdp, "document.getElementById('ra-label').textContent.startsWith('Ra 0.2584')", 4.0), 'Ra label')

    # ---- ROI drag on the Basler (main slot) in image coordinates ----
    # Preview off first: the fake capture's 60x40 frames would otherwise
    # keep replacing the full-size frame the drag is measured against.
    await cdp.js("document.getElementById('chk-preview').click()")
    check(base.wait_for(lambda: bridge.has('set_camera_active', False), 3.0), 'preview off releases the device')
    await tornado.gen.sleep(0.5)
    await cdp.js("document.getElementById('tab-live').click()")
    frame_b = np.full((3648, 5472), 120, np.uint8)
    bridge.image_received.emit('basler', frame_b)
    check(await wait_js(cdp, "views.basler.ow === 5472 && document.querySelector('#main-slot canvas').width > 100", 8.0),
          'full-size basler frame drawn (5472 px wide, sent downscaled)')
    bridge.calls.clear()
    roi = await wait_js(cdp, """(async () => {
        await new Promise(r => setTimeout(r, 300));
        const c = document.querySelector('#main-slot canvas');
        const r = c.getBoundingClientRect();
        // Drag INSIDE the drawn (letterboxed) image, not in the black margin.
        const d = views.basler.drawn;
        const ev = (type, x, y, button) => new MouseEvent(type, {clientX: r.left + d.x + x, clientY: r.top + d.y + y, button: button || 0, bubbles: true});
        c.dispatchEvent(ev('mousedown', 40, 40));
        c.dispatchEvent(ev('mousemove', 120, 100));
        c.dispatchEvent(ev('mouseup', 120, 100));
        await new Promise(r => setTimeout(r, 300));
        return JSON.stringify(ui.roi);
    })()""", 5.0)
    roi = json.loads(roi) if roi else None
    check(roi is not None and roi['w'] > 100 and roi['h'] > 100 and roi['x'] >= 0,
          f'ROI drag → server in image pixels: {roi}')
    check(bool(roi) and holder['c'].ui()['roi'] == roi, 'server holds the same ROI')

    # ---- thumbnail click swaps the main view; double-click maximises ----
    await cdp.js("""(() => {
        const c = [...document.querySelectorAll('#thumb-strip .cell canvas')][2];
        const r = c.getBoundingClientRect();
        const ev = (type) => new MouseEvent(type, {clientX: r.left + 10, clientY: r.top + 10, button: 0, bubbles: true});
        c.dispatchEvent(ev('mousedown')); c.dispatchEvent(ev('mouseup'));
    })()""")
    check(await wait_js(cdp, "document.getElementById('tab-live').textContent === 'hand_cam (live)' && "
                            "document.querySelectorAll('#thumb-strip .cell').length === 3"),
          'clicking the hand_cam thumbnail makes it the main view')
    await cdp.js("document.querySelector('#main-slot canvas').dispatchEvent(new MouseEvent('dblclick', {button: 0, bubbles: true}))")
    check(await wait_js(cdp, "document.getElementById('controlpanel').classList.contains('hidden')"), 'double-click maximises (control panel hidden)')
    await cdp.js("document.querySelector('#main-slot canvas').dispatchEvent(new MouseEvent('dblclick', {button: 0, bubbles: true}))")
    check(await wait_js(cdp, "!document.getElementById('controlpanel').classList.contains('hidden')"), 'double-click again restores')
    await cdp.js("""(() => {
        const c = [...document.querySelectorAll('#thumb-strip .cell canvas')].find(x => x.parentElement.querySelector('.hint'));
        const r = c.getBoundingClientRect();
        const ev = (type) => new MouseEvent(type, {clientX: r.left + 10, clientY: r.top + 10, button: 0, bubbles: true});
        c.dispatchEvent(ev('mousedown')); c.dispatchEvent(ev('mouseup'));
    })()""")
    check(await wait_js(cdp, "document.getElementById('tab-live').textContent === 'basler (live)'"), 'Basler back in the main slot')
    check(holder['c'].ui()['roi'] == roi, 'selecting the Basler thumbnail did not clear the ROI')

    # ---- STOP ALL ----
    bridge.calls.clear()
    await cdp.js("document.getElementById('btn-stop-all').click()")
    check(base.wait_for(lambda: bridge.has('arm_cancel') and bridge.has('send_task_command', 'STOP')
                        and bridge.has('mobile_stop') and bridge.has('lift_stop'), 3.0), 'STOP ALL reaches arm, task, base, lift')
    check(await wait_js(cdp, "!document.getElementById('chk-preview').checked", 3.0), 'STOP ALL unticks the preview')

    # ---- screenshot for a human ----
    await cdp.js("document.querySelector('#ctl-tabs .tab[data-tab=arm]').click()")
    await tornado.gen.sleep(0.3)
    shot = await cdp.cmd('Page.captureScreenshot', format='png')
    path = os.path.join(HERE, 'web_ui_screenshot.png')
    with open(path, 'wb') as f:
        f.write(base64.b64decode(shot['data']))
    print(f'  screenshot: {path}')

    check(not cdp.exceptions, f'no JS exceptions: {cdp.exceptions[:3]}')
    check(not cdp.console_errors, f'no console errors: {cdp.console_errors[:3]}')


def main():
    chrome = find_chrome()
    if not chrome:
        print('no chrome/chromium on PATH — skipping part C')
        return 0
    tmp = tempfile.mkdtemp(prefix='webui_c_')
    bridge = base.FakeBridge()
    bridge.emit_state('battery_state', {'percentage': 66.5, 'voltage': 52.1, 'current': 0.0, 'temperature': 20.0})
    plugin_dir = os.path.join(tmp, 'plugins')
    os.makedirs(plugin_dir)
    with open(os.path.join(plugin_dir, 'demo.py'), 'w') as f:
        f.write('def run(ctx):\n    ctx.log("demo ran")\n')
    with open(os.path.join(plugin_dir, 'slow.py'), 'w') as f:
        f.write('def run(ctx):\n    while not ctx.cancelled():\n        ctx.sleep(0.05)\n')
    holder = {}

    def factory(sink):
        holder['c'] = UiController(bridge, sink, plugin_dir=plugin_dir, save_dir=os.path.join(tmp, 'caps'),
                                   roi_config=os.path.join(tmp, 'roi.json'), stream_max_width=1400,
                                   stream_fps=30.0, calib_poll_s=1.0)
        return holder['c']

    server = WebServer(factory, port=0, address='127.0.0.1')
    port = server.start_in_thread()
    holder['server'] = server
    url = f'http://127.0.0.1:{port}/'

    devtools_port = port + 1
    proc = subprocess.Popen(
        [chrome, '--headless=new', '--no-sandbox', '--disable-gpu', '--hide-scrollbars',
         f'--remote-debugging-port={devtools_port}', f'--user-data-dir={os.path.join(tmp, "chrome")}',
         '--window-size=1600,950', '--remote-allow-origins=*', 'about:blank'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        ws_url = None
        for _ in range(100):
            try:
                with urllib.request.urlopen(f'http://127.0.0.1:{devtools_port}/json', timeout=1) as r:
                    targets = json.loads(r.read())
                pages = [t for t in targets if t.get('type') == 'page']
                if pages:
                    ws_url = pages[0]['webSocketDebuggerUrl']
                    break
            except Exception:
                pass
            time.sleep(0.1)
        check(ws_url is not None, 'headless Chrome started with DevTools')
        if ws_url:
            cdp = Cdp(ws_url)

            async def run():
                await cdp.connect()
                await scenario(cdp, url, bridge, holder)

            tornado.ioloop.IOLoop.current().run_sync(run, timeout=120)
    finally:
        proc.terminate()
        try:
            proc.wait(5)
        except Exception:
            proc.kill()
        server.stop()
        shutil.rmtree(tmp, ignore_errors=True)
    print(f'\n{base.N_OK} ok, {base.N_FAIL} failed')
    return 1 if base.N_FAIL else 0


if __name__ == '__main__':
    import tornado.concurrent  # noqa: F401
    sys.exit(main())
