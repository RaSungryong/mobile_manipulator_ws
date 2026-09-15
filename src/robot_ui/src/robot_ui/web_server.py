#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Web front for UiController: tornado HTTP + WebSocket, one process, one port.

    http://<robot>:8080/          the page (web/index.html + app.js + style.css)
    ws://<robot>:8080/ws          state, events, log, RPC, camera frames
    http://<robot>:8080/api/health  JSON liveness for a script / curl

Tornado because it is already on the robot PC (rosbridge_server depends on
it) — no new pip package, and the site LAN has no internet, so the page
loads no CDN either: everything is served from web/.

Protocol (JSON text frames unless stated)
-----------------------------------------
server → client
  {"t":"hello", "state":{name: value}, "ui":{...}, "log":[lines],
   "cams":[...], "axes":[...]}           once, on connect (the replay())
  {"t":"state", "k":<signal>, "v":...}   a bridge STATE signal
  {"t":"event", "k":<signal>, "v":...}   scan/calib/handeye progress, tag ids
  {"t":"ui",    "v":{patch}}             shared UI-state patch (deep-merge one level)
  {"t":"log",   "line":"HH:MM:SS  ..."}
  {"t":"reply", "id":n, "ok":bool, "result":..., "error":...}
  binary: [u32 BE header length][header JSON][JPEG]
          header {"cam","seq","w","h","ow","oh","mono"} — w,h are the
          wire size, ow,oh the camera's; the ROI is in ow×oh pixels.
client → server
  {"t":"call", "id":n, "m":"<api name without api_>", "a":[args]}
  {"t":"ack",  "cam":name}               "I have drawn that frame, send the next"

Frames are pushed with ONE outstanding frame per camera per client (the
ack scheme): a slow browser or a slow Wi-Fi link gets fewer frames, never
a growing backlog, and a fast one gets every encoded frame. New clients
get the last encoded frame of every camera immediately.

There is no authentication — same rule as rosbridge on 9090: keep the
port inside the site LAN. `check_origin` accepts any origin so the page
works through the Phoenix bridge's port forward (the browser's Origin is
then 192.168.0.20, not the robot's own address).
"""

import json
import os
import struct
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import tornado.ioloop
import tornado.web
import tornado.websocket

from robot_ui import paths
from robot_ui.ros_bridge import ARM_AXES

WEB_DIR = os.path.join(paths.PKG_DIR, 'web')


class WebSink:
    """UiController's sink, marshalled onto the tornado IO loop.

    Everything the controller emits arrives on some other thread (rospy
    callbacks, the pool, the encoder); tornado handlers may only be
    touched from the loop, so every method here is one add_callback.
    """

    def __init__(self):
        self.loop = None
        self.clients = set()
        self.controller = None

    def bind(self, loop, controller):
        self.loop = loop
        self.controller = controller

    def _post(self, fn, *args):
        if self.loop is None:
            return
        self.loop.add_callback(fn, *args)

    # ---- sink interface ----
    def state(self, kind, key, value):
        if kind == 'ui':
            self._post(self._broadcast_text, {'t': 'ui', 'v': value})
        else:
            self._post(self._broadcast_text, {'t': kind, 'k': key, 'v': value})

    def log(self, line):
        self._post(self._broadcast_text, {'t': 'log', 'line': line})

    def frame(self, cam, jpeg, header):
        self._post(self._broadcast_frame, cam, pack_frame(header, jpeg))

    # ---- loop-side ----
    def any_client(self):
        return bool(self.clients)

    def _broadcast_text(self, obj):
        text = json.dumps(obj, default=_json_default)
        for client in list(self.clients):
            client.send_text(text)

    def _broadcast_frame(self, cam, packet):
        for client in list(self.clients):
            client.offer_frame(cam, packet)


def pack_frame(header, jpeg):
    head = json.dumps(header).encode('utf-8')
    return struct.pack('>I', len(head)) + head + jpeg


def _json_default(obj):
    try:
        import numpy as np
        if isinstance(obj, np.generic):
            return obj.item()
        if isinstance(obj, np.ndarray):
            return obj.tolist()
    except Exception:
        pass
    return str(obj)


class UiSocket(tornado.websocket.WebSocketHandler):
    """One browser tab."""

    def initialize(self, sink, controller, executor):
        self.sink = sink
        self.controller = controller
        self.executor = executor
        self._credit = {}
        self._pending = {}
        self.remote = '?'

    def check_origin(self, origin):
        return True

    def open(self):
        self.set_nodelay(True)
        self.remote = self.request.remote_ip
        self.sink.clients.add(self)
        c = self.controller
        self.send_text(json.dumps({
            't': 'hello',
            'state': c.cached_states(),
            'ui': c.ui(),
            'log': c.log_lines(),
            'cams': c.cameras(),
            'axes': list(ARM_AXES),
            'server_time': time.time(),
        }, default=_json_default))
        for cam, (jpeg, header) in c.latest_frames().items():
            self.offer_frame(cam, pack_frame(header, jpeg))
        c.append_log(f'[web] client connected: {self.remote} '
                     f'({len(self.sink.clients)} online)')

    def on_close(self):
        self.sink.clients.discard(self)
        self.controller.append_log(
            f'[web] client left: {self.remote} '
            f'({len(self.sink.clients)} online)')

    # ---- outbound ----
    def send_text(self, text):
        try:
            self.write_message(text)
        except tornado.websocket.WebSocketClosedError:
            self.sink.clients.discard(self)
        except Exception:
            pass

    def offer_frame(self, cam, packet):
        """Deliver now if the client has acknowledged the previous frame
        of this camera, else keep only the newest as pending."""
        if self._credit.get(cam, True):
            self._credit[cam] = False
            self._pending.pop(cam, None)
            self._write_binary(packet)
        else:
            self._pending[cam] = packet

    def _write_binary(self, packet):
        try:
            self.write_message(packet, binary=True)
        except tornado.websocket.WebSocketClosedError:
            self.sink.clients.discard(self)
        except Exception:
            pass

    # ---- inbound ----
    def on_message(self, message):
        try:
            msg = json.loads(message)
        except Exception:
            return
        kind = msg.get('t')
        if kind == 'ack':
            cam = msg.get('cam')
            pending = self._pending.pop(cam, None)
            if pending is not None:
                self._write_binary(pending)      # credit stays consumed
            else:
                self._credit[cam] = True
        elif kind == 'call':
            self._dispatch(msg)

    def _dispatch(self, msg):
        call_id = msg.get('id')
        name = str(msg.get('m', ''))
        args = msg.get('a') or []
        fn = getattr(self.controller, 'api_' + name, None) \
            if name and not name.startswith('_') else None
        if fn is None:
            self._reply(call_id, False, error=f'unknown method {name!r}')
            return
        loop = tornado.ioloop.IOLoop.current()

        def _job():
            try:
                result = fn(*args)
                loop.add_callback(self._reply, call_id, True, result)
            except Exception as e:      # noqa: BLE001 — report to the caller
                loop.add_callback(self._reply, call_id, False,
                                  error=f'{type(e).__name__}: {e}')

        self.executor.submit(_job)

    def _reply(self, call_id, ok, result=None, error=None):
        obj = {'t': 'reply', 'id': call_id, 'ok': ok}
        if result is not None:
            obj['result'] = result
        if error is not None:
            obj['error'] = error
        self.send_text(json.dumps(obj, default=_json_default))


class HealthHandler(tornado.web.RequestHandler):
    def initialize(self, sink, controller):
        self.sink = sink
        self.controller = controller

    def get(self):
        self.set_header('Content-Type', 'application/json')
        self.write(json.dumps({
            'ok': True,
            'clients': len(self.sink.clients),
            'states': sorted(self.controller.cached_states().keys()),
            'ui': self.controller.ui(),
        }, default=_json_default))


class IndexHandler(tornado.web.RequestHandler):
    def initialize(self, web_dir):
        self.web_dir = web_dir

    def get(self):
        self.set_header('Cache-Control', 'no-store')
        with open(os.path.join(self.web_dir, 'index.html'), 'rb') as f:
            self.write(f.read())


class NoCacheStatic(tornado.web.StaticFileHandler):
    """The page is edited in place on the robot; a browser must not keep a
    stale app.js after a restart."""

    def set_extra_headers(self, path):
        self.set_header('Cache-Control', 'no-store')


class WebServer:
    """Owns the IO loop thread, the tornado app and the controller's sink."""

    def __init__(self, controller_factory, port=8080, address='0.0.0.0',
                 web_dir=None, workers=8):
        self.port = int(port)
        self.address = address
        self.web_dir = web_dir or WEB_DIR
        self.sink = WebSink()
        self._controller_factory = controller_factory
        self.controller = None
        self.executor = ThreadPoolExecutor(max_workers=workers,
                                           thread_name_prefix='ui-rpc')
        self.loop = None
        self.http = None
        self._thread = None
        self._ready = threading.Event()
        self.bound_port = None

    def _make_app(self):
        return tornado.web.Application([
            (r'/', IndexHandler, {'web_dir': self.web_dir}),
            (r'/ws', UiSocket, {'sink': self.sink,
                                'controller': self.controller,
                                'executor': self.executor}),
            (r'/api/health', HealthHandler, {'sink': self.sink,
                                             'controller': self.controller}),
            (r'/(.*)', NoCacheStatic, {'path': self.web_dir}),
        ], websocket_ping_interval=10, websocket_max_message_size=64 << 20)

    def start_in_thread(self):
        """Run the loop on a background thread (the ROS node keeps the main
        thread for rospy.spin). Returns once the port is bound."""
        self._thread = threading.Thread(target=self._serve, daemon=True,
                                        name='ui-web')
        self._thread.start()
        if not self._ready.wait(10.0):
            raise RuntimeError('web server did not start')
        return self.bound_port

    def _serve(self):
        import asyncio
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.loop = tornado.ioloop.IOLoop.current()
        self.controller = self._controller_factory(self.sink)
        self.sink.bind(self.loop, self.controller)
        self.controller.encoder.wanted = self.sink.any_client
        app = self._make_app()
        self.http = app.listen(self.port, address=self.address)
        self.bound_port = self.port
        if self.port == 0:
            for sock in self.http._sockets.values():
                self.bound_port = sock.getsockname()[1]
                break
        self._ready.set()
        self.loop.start()

    def stop(self):
        if self.loop is None:
            return

        def _shutdown():
            try:
                for client in list(self.sink.clients):
                    client.close()
                if self.http is not None:
                    self.http.stop()
            finally:
                self.loop.stop()

        self.loop.add_callback(_shutdown)
        if self._thread is not None:
            self._thread.join(5.0)
        if self.controller is not None:
            self.controller.shutdown()
        self.executor.shutdown(wait=False)
