#!/usr/bin/env python3
"""Offline check of apriltag_nav.camera_interface against a FAKE pypylon.

Why this exists (2026-10-13): pylon 3.0.1 leaks 4 file descriptors per
TlFactory.CreateDevice + InstantCamera and 1 per Open(); the Basler node used
to create the device at EVERY scan point and ran into the 1024 soft limit
after ~180 points (`WaitObject duplicate failed: Too many open files`), and
the collection scan walked 27 points without frames. The interface now keeps
ONE device object across close() and raises the soft RLIMIT_NOFILE at start.
Pinned here: one CreateDevice over many open/close cycles, a stale device is
recreated once when Open() fails, the limit is raised, the descriptor
warning fires near the limit. No camera, no ROS.
"""
import os
import resource
import sys
import types

# ---- stubs ---------------------------------------------------------------
LOG = {'info': [], 'warn': [], 'err': []}
rospy = types.ModuleType('rospy')
rospy.loginfo = lambda m: LOG['info'].append(str(m))
rospy.logwarn = lambda m: LOG['warn'].append(str(m))
rospy.logerr = lambda m: LOG['err'].append(str(m))
sys.modules['rospy'] = rospy


class _Node:
    def SetValue(self, *_a): pass
    def Execute(self): pass
    def GetValue(self): return 'Mono8'


class FakeCam:
    created = 0

    def __init__(self, fail_open_once=False):
        FakeCam.created += 1
        self.opened = False; self.grabbing = False
        self.opens = 0; self.closes = 0; self.destroyed = 0
        self.fail_open_once = fail_open_once

    def Open(self):
        if self.fail_open_once:
            self.fail_open_once = False
            raise RuntimeError('device unreachable')
        self.opened = True; self.opens += 1
    def Close(self): self.opened = False; self.closes += 1
    def IsOpen(self): return self.opened
    def IsGrabbing(self): return self.grabbing
    def StartGrabbing(self, *_a): self.grabbing = True
    def StopGrabbing(self): self.grabbing = False
    def DestroyDevice(self): self.destroyed += 1
    def GetDeviceInfo(self): return types.SimpleNamespace(GetModelName=lambda: 'acA5472-5gm')
    def __getattr__(self, name):          # UserSetSelector, GainRaw, PixelFormat, …
        return _Node()


class FakeTl:
    def __init__(self): self.enumerations = 0; self.creates = 0; self.next_fail_open = False
    def EnumerateDevices(self): self.enumerations += 1; return ['dev0']
    def CreateDevice(self, _d): self.creates += 1; return ('dev', self.next_fail_open)


TL = FakeTl()
pylon = types.ModuleType('pylon')
pylon.TlFactory = types.SimpleNamespace(GetInstance=lambda: TL)
pylon.InstantCamera = lambda dev: FakeCam(fail_open_once=dev[1])
pylon.ImageFormatConverter = lambda: types.SimpleNamespace(OutputPixelFormat=None, OutputBitAlignment=None)
pylon.PixelType_BGR8packed = 1; pylon.PixelType_Mono8 = 2; pylon.OutputBitAlignment_MsbAligned = 3
pylon.GrabStrategy_LatestImageOnly = 4; pylon.TimeoutHandling_ThrowException = 5
pypylon = types.ModuleType('pypylon'); pypylon.pylon = pylon
sys.modules['pypylon'] = pypylon; sys.modules['pypylon.pylon'] = pylon

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))
from apriltag_nav import camera_interface as CI          # noqa: E402

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


# ---- 1. the open-files limit is raised at construction -------------------
soft0, hard0 = resource.getrlimit(resource.RLIMIT_NOFILE)
low = min(soft0, 1024)
resource.setrlimit(resource.RLIMIT_NOFILE, (low, hard0))
ci = CI.CameraInterface()
soft1, _ = resource.getrlimit(resource.RLIMIT_NOFILE)
check(soft1 == hard0 if hard0 != resource.RLIM_INFINITY else soft1 >= 65536,
      f'constructing the interface raises the soft open-files limit {low} -> {soft1} (hard {hard0})')
check(any('open-files limit' in m for m in LOG['info']), 'the limit is logged once')
check(CI.fd_count() > 0, 'fd_count reads /proc/self/fd')

# ---- 2. one device object across many open/close cycles -------------------
for i in range(5):
    check(ci.initialize() and ci.start_grabbing(), f'cycle {i + 1}: initialize + start_grabbing ok') if i == 0 else (ci.initialize(), ci.start_grabbing())
    ci.stop_grabbing(); ci.close()
check(TL.creates == 1 and TL.enumerations == 1 and FakeCam.created == 1,
      f'5 open/close cycles create the pylon device ONCE (creates {TL.creates}, enumerations {TL.enumerations})')
check(ci._device_cam.opens == 5 and ci._device_cam.closes == 5 and ci.camera is None,
      '… and Open()/Close() it five times; `camera` is None while closed')
check(ci._device_cam.destroyed == 0, 'the cached device is never destroyed on close')

# ---- 3. a stale device (Open raises) is dropped and recreated once -------
cam_old = ci._device_cam
cam_old.fail_open_once = True
ok = ci.initialize()
check(ok and ci._device_cam is not cam_old and cam_old.destroyed == 1 and TL.creates == 2,
      f'Open() failure on the cached device -> DestroyDevice + one new CreateDevice, init succeeds (creates {TL.creates})')
check(any('recreating' in m for m in LOG['warn']), 'the recreate is logged')
ci.close()
ci.initialize(); ci.close()
check(TL.creates == 2, 'after the recreate the new device is cached again')

# ---- 4. no camera at all --------------------------------------------------
TL.EnumerateDevices = lambda: []
ci2 = CI.CameraInterface()
check(ci2.initialize() is False and ci2.camera is None and ci2._device_cam is None, 'no device enumerated -> False, nothing cached')
TL.EnumerateDevices = lambda: ['dev0']

# ---- 5. the descriptor warning near the limit ------------------------------
ci3 = CI.CameraInterface()
LOG['err'].clear()
real = CI.fd_count
CI.fd_count = lambda: int(0.95 * resource.getrlimit(resource.RLIMIT_NOFILE)[0]) + 1
ci3.initialize(); ci3.close(); ci3.initialize(); ci3.close()
CI.fd_count = real
check(len([m for m in LOG['err'] if 'file descriptors in use' in m]) == 1,
      'above 90 % of the soft limit the warning fires once, not per open')

resource.setrlimit(resource.RLIMIT_NOFILE, (soft0, hard0))
print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
