#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Signal — a minimal, thread-agnostic connect/emit primitive.

RosBridge used to be a QObject whose outputs were pyqtSignals. That tied
the ONE module allowed to talk to ROS to a GUI toolkit, so a UI that is not
a Qt window (the web UI, 2026-09-15) could not share it without a running
Qt event loop. This class is what replaces pyqtSignal on the bridge:

  * `connect(fn)` / `disconnect(fn)` / `emit(*args)` — same shape, so the
    call sites did not change.
  * emit runs every handler SYNCHRONOUSLY on the emitting thread — for the
    bridge that is a rospy callback thread. A consumer that has a thread
    rule of its own (Qt: widgets only from the GUI thread) marshals in its
    own handler; MainWindow does that through one pyqtSignal, the web
    server through tornado's `add_callback`.
  * A handler that raises does not stop the others and does not propagate
    into the emitter; the failure is reported through `on_error` (default:
    print to stderr) so a bad slot cannot kill a rospy callback.
"""

import sys
import threading
import traceback


class Signal:
    """connect / disconnect / emit, safe to call from any thread."""

    def __init__(self, name='signal', on_error=None):
        self.name = name
        self._handlers = []
        self._lock = threading.Lock()
        self._on_error = on_error

    def connect(self, handler):
        with self._lock:
            if handler not in self._handlers:
                self._handlers.append(handler)
        return handler

    def disconnect(self, handler=None):
        with self._lock:
            if handler is None:
                self._handlers = []
            elif handler in self._handlers:
                self._handlers.remove(handler)

    def receivers(self):
        with self._lock:
            return len(self._handlers)

    def emit(self, *args):
        with self._lock:
            handlers = list(self._handlers)
        for handler in handlers:
            try:
                handler(*args)
            except Exception as e:          # noqa: BLE001 — isolate slots
                if self._on_error is not None:
                    try:
                        self._on_error(self.name, e)
                        continue
                    except Exception:
                        pass
                sys.stderr.write('[signal %s] handler %r raised %s: %s\n%s'
                                 % (self.name, handler, type(e).__name__, e,
                                    traceback.format_exc()))
