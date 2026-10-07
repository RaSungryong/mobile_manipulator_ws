# -*- coding: utf-8 -*-
"""Arm Ethernet link and Fairino RPC health (2026-10-06).

Why this exists. On 2026-10-06 the scan `scan_joint_offset10mm_h662` froze
13.5 s at point 65 and 14.7 s at traverse 89 with NOTHING in any ROS log.
The kernel journal had it: `enp2s0: NIC Link is Down` — the PC's link to
the Fairino controller (192.168.58.2) — each time, back up ~12 s later at
100 Mbps. The scan loop was sitting inside a blocking SDK call
(GetActualTCPPose after the capture, SetSpeed before the next MoveJ) until
the link returned and TCP recovered. The journal shows that link flapping
5–23 times a day since 2026-09-02. Two things make it visible from inside
the stack:

* `TimedRPC` wraps the SDK object. Every call is timed; one that takes
  longer than `warn_s` is logged with its name and duration and counted,
  and a call that is STILL in flight past `warn_s` is reported live by
  `check()` (arm_node's state timer calls it), so the freeze shows up
  while it happens, not only after. Nothing about the call itself changes:
  same arguments, same return value, same exceptions.
* `LinkMonitor` reads the NIC carrier from sysfs for the interface the
  route to the arm goes through (`ip route get <ip>`, resolved once).
  A carrier drop is logged at once with the sysfs down-count, the
  recovery with its duration. Reading a sysfs file at 10 Hz costs nothing.

Both are published in `/arm/state` (`link_*`, `rpc_*`) and shown on the
UI's ARM chip. Neither changes what the arm does — a stalled RPC is still
waited for; the controller has the command and finishes it. What they
buy is the diagnosis: `ARM LINK DOWN` on the chip and a `[Arm REAL]
Fairino RPC GetActualTCPPose blocked 13.5 s` line instead of a silent
15 s pause and a journal search afterwards.
"""
import os
import subprocess
import threading
import time

import rospy


class TimedRPC:
    """Proxy around the Fairino `Robot.RPC` object that times every call.

    Attribute access is forwarded; callables come back wrapped. Thread-safe
    for the two callers the stack has (arm_node's state timer and the
    executor worker): in-flight calls are tracked per thread and `inflight()`
    reports the oldest one.
    """

    def __init__(self, target, warn_s=2.0, name='Fairino RPC', clock=None):
        # Set through __dict__ so __getattr__ never recurses on them.
        self.__dict__['_target'] = target
        self.__dict__['_warn_s'] = float(warn_s)
        self.__dict__['_name'] = str(name)
        self.__dict__['_clock'] = clock or time.time
        self.__dict__['_lock'] = threading.Lock()
        self.__dict__['_inflight'] = {}        # thread ident -> (method, t0)
        self.__dict__['_live_warned'] = set()  # thread idents already reported live
        self.__dict__['stall_count'] = 0
        self.__dict__['last_stall'] = None     # (method, seconds, wall time at return)

    # ---- forwarding -------------------------------------------------
    def __getattr__(self, attr):
        value = getattr(self._target, attr)
        if not callable(value):
            return value

        def timed(*args, **kwargs):
            tid = threading.get_ident()
            t0 = self._clock()
            with self._lock:
                self._inflight[tid] = (attr, t0)
            try:
                return value(*args, **kwargs)
            finally:
                t1 = self._clock()
                dt = t1 - t0
                with self._lock:
                    self._inflight.pop(tid, None)
                    live = tid in self._live_warned
                    self._live_warned.discard(tid)
                    if dt >= self._warn_s:
                        self.__dict__['stall_count'] += 1
                        self.__dict__['last_stall'] = (attr, dt, t1)
                if dt >= self._warn_s:
                    rospy.logwarn(
                        f"[Arm REAL] {self._name} {attr} blocked {dt:.1f} s"
                        f"{' (reported live above)' if live else ''}"
                        " — arm Ethernet link? see /arm/state link_* and "
                        "`journalctl -k | grep 'NIC Link'`")
        timed.__name__ = attr
        return timed

    def __setattr__(self, attr, value):
        setattr(self._target, attr, value)

    # ---- health -----------------------------------------------------
    def inflight(self, now=None):
        """(method, age_s) of the oldest call still running, or None."""
        now = self._clock() if now is None else now
        with self._lock:
            if not self._inflight:
                return None
            method, t0 = min(self._inflight.values(), key=lambda v: v[1])
        return method, now - t0

    def check(self, now=None):
        """Call from a timer: logs ONCE per stuck call while it is stuck.
        Returns True while some call has been in flight longer than warn_s."""
        now = self._clock() if now is None else now
        with self._lock:
            stuck = [(tid, m, now - t0) for tid, (m, t0) in self._inflight.items()
                     if now - t0 >= self._warn_s]
            fresh = [(tid, m, age) for tid, m, age in stuck
                     if tid not in self._live_warned]
            for tid, _, _ in fresh:
                self._live_warned.add(tid)
        for _, method, age in fresh:
            rospy.logwarn(
                f"[Arm REAL] {self._name} {method} has not returned for "
                f"{age:.1f} s — the arm is NOT being commanded; waiting "
                "(arm Ethernet link down?)")
        return bool(stuck)

    def health(self, now=None):
        now = self._clock() if now is None else now
        stalled = self.check(now)
        last = self.last_stall
        text = ''
        if last is not None:
            method, dt, t1 = last
            text = f"{method} {dt:.1f} s at {time.strftime('%H:%M:%S', time.localtime(t1))}"
        return {'rpc_stalled': stalled,
                'rpc_stall_count': int(self.stall_count),
                'rpc_last_stall': text}


class LinkMonitor:
    """Carrier state of the NIC that carries the route to `host_ip`."""

    def __init__(self, host_ip, iface=None, sysfs='/sys/class/net', clock=None):
        self.host_ip = str(host_ip)
        self.sysfs = sysfs
        self._clock = clock or time.time
        self.iface = iface or self.resolve_iface(self.host_ip)
        self.up = None              # None = never read
        self.down_count = 0         # drops seen by THIS monitor
        self.sysfs_down_count = None  # the kernel's count since boot
        self._down_since = None
        self._read_failed = False
        if self.iface:
            rospy.loginfo(f"[Arm REAL] arm link monitor: {self.host_ip} via {self.iface}")
        else:
            rospy.logwarn(f"[Arm REAL] arm link monitor OFF: no route interface "
                          f"found for {self.host_ip}")

    @staticmethod
    def resolve_iface(host_ip):
        """`ip -o route get <ip>` → the `dev` token, or '' when it fails."""
        try:
            out = subprocess.run(['ip', '-o', 'route', 'get', host_ip],
                                 capture_output=True, text=True, timeout=2.0).stdout
        except Exception:
            return ''
        toks = out.split()
        for i, t in enumerate(toks):
            if t == 'dev' and i + 1 < len(toks):
                return toks[i + 1]
        return ''

    def _read(self, name):
        with open(os.path.join(self.sysfs, self.iface, name)) as f:
            return f.read().strip()

    def poll(self, now=None):
        """Read the carrier once; log transitions. Returns the state dict."""
        now = self._clock() if now is None else now
        if self.iface:
            try:
                up = self._read('carrier') == '1'
                try:
                    self.sysfs_down_count = int(self._read('carrier_down_count'))
                except Exception:
                    pass
                self._read_failed = False
            except Exception as e:
                if not self._read_failed:
                    rospy.logwarn(f"[Arm REAL] arm link monitor: cannot read "
                                  f"{self.iface} carrier: {e}")
                self._read_failed = True
                up = None
            if up is not None and up != self.up:
                if self.up is not None or not up:
                    if not up:
                        self.down_count += 1
                        self._down_since = now
                        rospy.logwarn(
                            f"[Arm REAL] ARM LINK DOWN: {self.iface} carrier lost "
                            f"(drop {self.down_count} since start"
                            f"{', ' + str(self.sysfs_down_count) + ' since boot' if self.sysfs_down_count is not None else ''}"
                            ") — every Fairino RPC blocks until it returns")
                    else:
                        dur = (now - self._down_since) if self._down_since else None
                        rospy.logwarn(
                            f"[Arm REAL] arm link {self.iface} back up"
                            f"{' after %.1f s' % dur if dur is not None else ''}")
                        self._down_since = None
                self.up = up
        return self.state(now)

    def state(self, now=None):
        now = self._clock() if now is None else now
        return {'link_iface': self.iface or '',
                'link_up': bool(self.up) if self.up is not None else True,
                'link_down_count': int(self.down_count),
                'link_down_s': (now - self._down_since) if self._down_since else 0.0}
