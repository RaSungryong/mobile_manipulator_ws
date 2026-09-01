#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PluginRunner — hot-reloadable operator scripts.

Keeps the reference UI's most useful property: a script under plugins/ can be
edited and re-run without restarting the application, so a collection routine
can be iterated against the real robot in seconds.

What changed is what a script is handed. The reference passed the whole main
window, so `run(window)` could reach `window.robot_thread.robot` — a live
Fairino RPC handle — and drive the arm directly, bypassing arm_node entirely.
Two processes commanding one arm with neither aware of the other is exactly the
failure this rewrite exists to remove, and a plugin is the easiest place for it
to creep back in.

So a plugin receives a PluginContext instead: the ROS bridge, a logger, and a
cancellation flag. Everything reachable through it is a topic or a service on
the node that owns the device. There is no handle to bypass.

    def run(ctx):
        ctx.log('starting')
        ok, msg = ctx.bridge.arm_jog('z', -5.0)
        ok, msg, frames = ctx.bridge.capture(num_samples=3)
        result = ctx.bridge.predict_ra(frames[0], tag='p1')
        ctx.log(f"Ra = {result['ra']:.4f}")

Plugins run in a worker thread, so ctx.cancelled() must be checked inside any
loop — that is the only way the Stop button can interrupt one.

⚠️ A plugin is arbitrary Python executed in this process. The directory is
trusted the same way the launch file is; do not point ~plugin_dir at anything
you would not run by hand.
"""

import importlib
import importlib.util
import os
import sys
import threading
import traceback


class PluginContext:
    """What a plugin is allowed to touch."""

    def __init__(self, bridge, log, cancel_event, params=None):
        self.bridge = bridge
        self._log = log
        self._cancel = cancel_event
        self.params = dict(params or {})

    def log(self, message):
        self._log(str(message))

    def cancelled(self):
        """True once the operator asked to stop. Check it inside every loop."""
        return self._cancel.is_set()

    def sleep(self, seconds, step=0.1):
        """Interruptible sleep. Returns False if cancelled partway through."""
        remaining = float(seconds)
        while remaining > 0:
            if self.cancelled():
                return False
            self._cancel.wait(min(step, remaining))
            remaining -= step
        return not self.cancelled()


class PluginRunner:
    """Discovers, reloads and executes plugin modules."""

    def __init__(self, plugin_dir, bridge, log=print):
        self.plugin_dir = plugin_dir
        self.bridge = bridge
        self.log = log
        self._cancel = threading.Event()
        self._thread = None

    # ---------- discovery ----------
    def discover(self):
        """Module names in plugin_dir, sorted. Excludes dunder files."""
        if not os.path.isdir(self.plugin_dir):
            self.log(f'[plugin] directory not found: {self.plugin_dir}')
            return []
        names = [f[:-3] for f in sorted(os.listdir(self.plugin_dir))
                 if f.endswith('.py') and not f.startswith('__')]
        return names

    def _load_module(self, name):
        """Import (or re-import) a plugin by file path.

        Loading by path rather than by package name on purpose: the reference
        used `importlib.import_module('scripts.' + name)`, which needs the
        directory to be an importable package and silently picks up a
        same-named module from elsewhere on sys.path if one exists. A path
        loader has neither problem, and makes ~plugin_dir work anywhere.
        """
        path = os.path.join(self.plugin_dir, f'{name}.py')
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        mod_key = f'robot_ui_plugin_{name}'
        spec = importlib.util.spec_from_file_location(mod_key, path)
        module = importlib.util.module_from_spec(spec)
        # Registered before exec so a plugin containing a dataclass or an
        # enum — both of which look themselves up in sys.modules — imports.
        sys.modules[mod_key] = module
        spec.loader.exec_module(module)
        return module

    # ---------- execution ----------
    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def start(self, name, params=None, on_finished=None):
        """Run plugin `name` in a worker thread. Returns True if it started."""
        if self.is_running():
            self.log('[plugin] one is already running — stop it first')
            return False

        self._cancel.clear()
        ctx = PluginContext(self.bridge, self.log, self._cancel, params)

        def _target():
            ok = False
            try:
                module = self._load_module(name)
                if not hasattr(module, 'run'):
                    self.log(f"[plugin] '{name}' has no run(ctx) function")
                else:
                    self.log(f"[plugin] '{name}' started")
                    module.run(ctx)
                    ok = not self._cancel.is_set()
                    self.log(f"[plugin] '{name}' "
                             f"{'finished' if ok else 'cancelled'}")
            except Exception as e:
                # Full traceback: a plugin is being edited live, so the line
                # number is the whole point of the message.
                self.log(f"[plugin] '{name}' raised {type(e).__name__}: {e}\n"
                         f'{traceback.format_exc()}')
            finally:
                if on_finished is not None:
                    try:
                        on_finished(ok)
                    except Exception:
                        pass

        self._thread = threading.Thread(target=_target, daemon=True)
        self._thread.start()
        return True

    def cancel(self):
        """Ask the running plugin to stop.

        Only sets the flag — a plugin that never checks ctx.cancelled() keeps
        going, and there is no safe way to kill a Python thread mid-service-call.
        The arm/base/lift stops are separate and immediate; this is not the
        emergency path.
        """
        if self.is_running():
            self.log('[plugin] cancel requested')
        self._cancel.set()
