#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline checks of basler_camera_node's lamp bracket (2026-09-15):
the pre-lamp frame flush, the dark-frame re-grab and the lamp HOLD.

Stubs rospy / messages / cv_bridge / the camera and the Crevis devices, then
drives the REAL BaslerCameraNode handlers against a camera model with the
acA5472's one-frame delivery lag: RetrieveResult hands back the frame
exposed one period EARLIER, so the first grab after lamp-on is black.
Run:  python3 tools/check_basler_lamp.py
"""
import os
import sys
import threading
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))

# ---------------------------------------------------------------- stubs
LOG = {'info': [], 'warn': [], 'err': []}
rospy = types.ModuleType('rospy')
rospy.loginfo = lambda m, *a: LOG['info'].append(str(m))
rospy.logwarn = lambda m, *a: LOG['warn'].append(str(m))
rospy.logerr = lambda m, *a: LOG['err'].append(str(m))
rospy.sleep = lambda s: None
rospy.get_param = lambda k, d=None: d
rospy.init_node = lambda *a, **k: None
rospy.Duration = lambda s: s


class _Pub:
    def __init__(self, topic, *a, **k):
        self.topic = topic
        self.sent = []

    def publish(self, msg):
        self.sent.append(getattr(msg, 'data', msg))


class _Time:
    @staticmethod
    def now():
        return 0.0


rospy.Publisher = _Pub
rospy.Subscriber = lambda *a, **k: None
rospy.Service = lambda *a, **k: None
rospy.Time = _Time
rospy.Timer = lambda *a, **k: types.SimpleNamespace(shutdown=lambda: None)
sys.modules['rospy'] = rospy


def _msg(name):
    class M:
        def __init__(self, data=None):
            self.data = data
    M.__name__ = name
    return M


std_msgs = types.ModuleType('std_msgs'); std_msgs.msg = types.ModuleType('std_msgs.msg')
std_msgs.msg.Bool = _msg('Bool'); std_msgs.msg.String = _msg('String')
sys.modules['std_msgs'] = std_msgs; sys.modules['std_msgs.msg'] = std_msgs.msg
sensor_msgs = types.ModuleType('sensor_msgs'); sensor_msgs.msg = types.ModuleType('sensor_msgs.msg')
sensor_msgs.msg.Image = _msg('Image')
sys.modules['sensor_msgs'] = sensor_msgs; sys.modules['sensor_msgs.msg'] = sensor_msgs.msg
cv_bridge = types.ModuleType('cv_bridge')


class _Bridge:
    def cv2_to_imgmsg(self, frame, encoding='mono8'):
        m = types.SimpleNamespace(frame=frame, encoding=encoding,
                                  header=types.SimpleNamespace(stamp=None))
        return m


cv_bridge.CvBridge = _Bridge
sys.modules['cv_bridge'] = cv_bridge
robot_msgs = types.ModuleType('robot_msgs'); robot_msgs.srv = types.ModuleType('robot_msgs.srv')


class _Resp:
    def __init__(self):
        self.images = []; self.success = False; self.message = ''


robot_msgs.srv.CaptureImages = object; robot_msgs.srv.CaptureImagesResponse = _Resp
sys.modules['robot_msgs'] = robot_msgs; sys.modules['robot_msgs.srv'] = robot_msgs.srv
for name in ('apriltag_nav.camera_interface', 'apriltag_nav.navifra_devices'):
    mod = types.ModuleType(name)
    sys.modules[name] = mod
sys.modules['apriltag_nav.camera_interface'].CameraInterface = object
sys.modules['apriltag_nav.navifra_devices'].NavifraDevices = object
paths = types.ModuleType('apriltag_nav.paths'); paths.load_yaml_block = lambda *a: {}
sys.modules['apriltag_nav.paths'] = paths
pkg = types.ModuleType('apriltag_nav'); pkg.__path__ = []
sys.modules.setdefault('apriltag_nav', pkg)

import importlib.util                                           # noqa: E402
spec = importlib.util.spec_from_file_location(
    'basler_camera_node', os.path.join(PKG, 'scripts', 'basler_camera_node.py'))
bcn = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bcn)

# ---------------------------------------------------------------- fakes


class FakeDevices:
    """The Crevis VISION relay, with an optional switch-on latency in frames."""
    def __init__(self, on_latency_frames=0):
        self.lit = False
        self.calls = []
        self._pending = 0
        self.on_latency_frames = on_latency_frames

    def vision_led(self, on):
        self.calls.append(bool(on))
        if on and self.on_latency_frames:
            self._pending = self.on_latency_frames     # lights after N frames
        else:
            self.lit = bool(on)
            self._pending = 0

    def tick(self):
        if self._pending:
            self._pending -= 1
            if self._pending == 0:
                self.lit = True


class FakeCamera:
    """Free-running 5 fps ROLLING-shutter camera (IMX183): every RetrieveResult
    returns the frame exposed over the PREVIOUS period (readout + GigE
    transfer ~ the whole period), its top half under the lamp as it was at
    the start of that period and its bottom half as it was at the end. A
    lamp switch inside the window therefore yields a half-lit frame — the
    "part of the image black" the operator saw."""
    encoding = 'mono8'

    def __init__(self, devices):
        self.devices = devices
        self.grabs = 0
        self._states = [devices.lit, devices.lit]   # lamp at the last two ticks

    def initialize(self):
        return True

    def start_grabbing(self):
        return True

    def stop_grabbing(self):
        pass

    def close(self):
        pass

    def grab_frame(self, timeout=0):
        self.grabs += 1
        top, bottom = self._states[-2], self._states[-1]
        frame = np.empty((64, 64), dtype=np.uint8)
        frame[:32] = 180 if top else 1
        frame[32:] = 180 if bottom else 1
        # the next period starts now, under the lamp as it is now
        self.devices.tick()
        self._states = [self._states[-1], self.devices.lit]
        return frame


def make_node(devices, flush=1, dark_mean=4.0, retries=2):
    n = bcn.BaslerCameraNode.__new__(bcn.BaslerCameraNode)
    # idle_close_sec 0 would close (and drop a hold) right after every
    # capture — the real default is 5 s; rospy.Timer is a no-op stub here.
    n._num_samples = 1; n._delay_between_s = 0.0; n._idle_close_sec = 5.0
    n._grab_timeout_ms = 10; n._warmup_s = 0.0; n._publish_last = False
    n._service_name = '/camera/capture'
    n._lamp_flush_frames = flush; n._dark_frame_mean = dark_mean
    n._dark_frame_retries = retries
    n._bridge = _Bridge(); n._camera = FakeCamera(devices)
    n._lock = threading.RLock(); n._is_open = False; n._idle_timer = None
    n._devices = devices; n._lamp_hold = False; n._lamp_on = False
    n._state_pub = _Pub('/camera/state'); n._lamp_pub = _Pub('/camera/lamp_state')
    n._image_pub = _Pub('/basler/image_raw')
    return n


def req(n=1, led=True):
    return types.SimpleNamespace(num_samples=n, delay_between_s=0.0, use_vision_led=led)


def means(resp):
    return [int(m.frame.mean()) for m in resp.images]


N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


print('== the defect: no flush, no dark check -> first lamp-on frame black, second half-lit')
dev = FakeDevices(); node = make_node(dev, flush=0, dark_mean=0.0)
r = node._handle_capture(req(3))
m = means(r)
check(r.success and m[0] < 4 and 60 < m[1] < 120 and m[2] > 170,
      f'burst of 3, old behaviour: means {m} — black, then a half-lit (rolling shutter) frame, then lit')
check(dev.calls == [True, False], 'lamp bracketed on/off around the burst')

print('== flush alone is not enough: the frame after the flushed one straddles the lamp switch')
dev = FakeDevices(); node = make_node(dev, flush=1, dark_mean=0.0)
r = node._handle_capture(req(2))
m = means(r)
check(60 < m[0] < 120 and m[1] > 170,
      f'flush 1 + whole-frame mean disabled: first kept frame is half lit: {m}')

print('== fix: flush + band check -> every frame fully lit')
dev = FakeDevices(); node = make_node(dev, flush=1, dark_mean=4.0, retries=2)
r = node._handle_capture(req(3))
m = means(r)
check(r.success and all(v > 170 for v in m), f'all three frames fully lit: {m}')
check('flushed 1 pre-lamp frame' in r.message and 'dark frame re-grabbed' in r.message,
      f'message says so: {r.message!r}')
check(node._camera.grabs == 5, '5 grabs for 3 frames (one flushed, one half-lit re-grabbed)')
half = np.empty((64, 64), dtype=np.uint8); half[:32] = 1; half[32:] = 180
check(node._is_dark(half) and not node._is_dark(np.full((64, 64), 180, np.uint8))
      and node._is_dark(np.full((64, 64), 1, np.uint8)),
      'band check flags half-lit and black frames, passes a lit one')

print('== fix 2: dark re-grab covers a slow relay (lights 2 frames after the command)')
dev = FakeDevices(on_latency_frames=2); node = make_node(dev, flush=1, dark_mean=4.0, retries=3)
r = node._handle_capture(req(2))
check(r.success and all(m > 170 for m in means(r)), f'both frames fully lit despite the late relay: {means(r)}')
check('dark frame' in r.message and 're-grabbed' in r.message, f'message reports the re-grab: {r.message!r}')
check(any('exposed unlit' in w for w in LOG['warn']), 'a warning names the unlit frame')
dev = FakeDevices(on_latency_frames=8); node = make_node(dev, flush=1, dark_mean=4.0, retries=2)
r = node._handle_capture(req(1))
check(r.success and means(r)[0] < 4, 'retries are bounded: a relay that never lights in time still returns a frame')

print('== lamp-off preview frames are never re-grabbed')
dev = FakeDevices(); node = make_node(dev)
node._cb_set_active(types.SimpleNamespace(data=True))
r = node._handle_capture(req(1, led=False))
check(r.success and means(r)[0] < 4 and node._camera.grabs == 1 and dev.calls == [],
      'lamp-off capture: one grab, dark frame accepted, relay untouched')

print('== lamp HOLD')
dev = FakeDevices(); node = make_node(dev)
node._cb_set_lamp(types.SimpleNamespace(data=True))
check(node._is_open and node._lamp_hold and dev.lit and node._lamp_pub.sent[-1] is True,
      'set_lamp true opens the device, lights the lamp, publishes lamp_state true')
# the operator aims with the preview for a while: the frame in flight is lit
node._handle_capture(req(1, led=False))
g0 = node._camera.grabs
r = node._handle_capture(req(2, led=True))
check(r.success and all(m > 170 for m in means(r)) and node._camera.grabs - g0 == 2,
      f'capture under a hold: no flush, no toggling, frames lit: {means(r)}')
check(dev.calls == [True] and dev.lit, 'the relay was switched once (the hold) and is still on')
check('lamp held' in r.message, f'message notes the hold: {r.message!r}')
r = node._handle_capture(req(1, led=False))
check(r.success and means(r)[0] > 170 and dev.lit, 'a lamp-off request under a hold leaves the lamp on')
node._cb_set_lamp(types.SimpleNamespace(data=False))
check(not node._lamp_hold and not dev.lit and node._lamp_pub.sent[-1] is False,
      'set_lamp false releases the hold and publishes false')
node._cb_set_lamp(types.SimpleNamespace(data=True))
node._close_locked()
check(not node._lamp_hold and not dev.lit and node._lamp_pub.sent[-1] is False,
      'closing the device (idle / set_active false / shutdown) drops the hold and the lamp')
node._cb_set_lamp(types.SimpleNamespace(data=True))
node.shutdown()
check(not dev.lit and dev.calls[-1] is False, 'shutdown ends with the lamp off')

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
