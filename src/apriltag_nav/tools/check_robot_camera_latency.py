#!/usr/bin/env python3
"""Offline check of robot_camera_node's latency levers (2026-09-15):
the overlay renders on a timer thread instead of the detection callback,
detect_max_hz skips frames, and the render happens once per new frame
only while subscribed. rospy is stubbed; the detector is a fake.

    python3 src/apriltag_nav/tools/check_robot_camera_latency.py
"""
import os
import sys
import time
import types

import numpy as np

_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_TOOLS, '..', 'src'))
sys.path.insert(0, os.path.join(_TOOLS, '..', 'scripts'))

# ---- stub rospy -------------------------------------------------------
timers = []
class _Timer:
    def __init__(self, period, cb):
        self.period, self.cb, self.alive = period, cb, True
        timers.append(self)
    def shutdown(self):
        self.alive = False
class _Duration:
    def __init__(self, s): self.s = s
class _Stamp:
    def __init__(self, s): self._s = s
    def to_sec(self): return self._s
    def __eq__(self, o): return isinstance(o, _Stamp) and o._s == self._s
    def __ne__(self, o): return not self.__eq__(o)
class _Pub:
    def __init__(self, topic, *a, **k):
        self.topic, self.msgs, self.conns = topic, [], 0
    def publish(self, m): self.msgs.append(m)
    def get_num_connections(self): return self.conns
rospy = types.ModuleType('rospy')
rospy.Publisher = _Pub
rospy.Subscriber = lambda *a, **k: types.SimpleNamespace(unregister=lambda: None)
rospy.Service = lambda *a, **k: None
rospy.Timer = _Timer
rospy.Duration = _Duration
for lv in ('loginfo', 'logwarn', 'logerr', 'logdebug', 'logwarn_throttle', 'logerr_throttle'):
    setattr(rospy, lv, lambda *a, **k: None)
rospy.get_param = lambda k, d=None: d
rospy.init_node = lambda *a, **k: None
sys.modules['rospy'] = rospy
# sensor_msgs / std_srvs / cv_bridge / dt_apriltags stubs
sm = types.ModuleType('sensor_msgs'); smm = types.ModuleType('sensor_msgs.msg')
class Image:
    def __init__(self): self.header = types.SimpleNamespace(stamp=_Stamp(0.0), frame_id='')
class CameraInfo: pass
smm.Image, smm.CameraInfo = Image, CameraInfo
sys.modules['sensor_msgs'] = sm; sys.modules['sensor_msgs.msg'] = smm
ss = types.ModuleType('std_srvs'); sss = types.ModuleType('std_srvs.srv')
sss.SetBool = object; sss.SetBoolResponse = object
sys.modules['std_srvs'] = ss; sys.modules['std_srvs.srv'] = sss
cvb = types.ModuleType('cv_bridge')
class CvBridge:
    def imgmsg_to_cv2(self, m, enc): return m.img
    def cv2_to_imgmsg(self, img, enc):
        o = Image(); o.img = img; return o
cvb.CvBridge = CvBridge
sys.modules['cv_bridge'] = cvb
dta = types.ModuleType('dt_apriltags')
class _Det:
    def __init__(self, tid):
        self.tag_id = tid; self.center = (640.0, 360.0)
        self.corners = np.array([[600, 320], [680, 320], [680, 400], [600, 400]], float)
        self.pose_t = np.array([[0.0], [0.0], [0.3]]); self.pose_R = np.eye(3)
class Detector:
    made = []
    def __init__(self, families=None, quad_decimate=1.0, nthreads=1):
        Detector.made.append(dict(nthreads=nthreads)); self.calls = 0
    def detect(self, gray, estimate_tag_pose=False, camera_params=None, tag_size=None):
        self.calls += 1
        time.sleep(0.005)
        return [_Det(105)]
dta.Detector = Detector
sys.modules['dt_apriltags'] = dta
rm = types.ModuleType('robot_msgs'); rmm = types.ModuleType('robot_msgs.msg')
class AprilTagDetection:
    def __init__(self):
        self.id = 0; self.center_x = self.center_y = 0.0; self.pose_x = self.pose_y = self.pose_z = 0.0
        self.roll = self.pitch = self.yaw = self.tilt_from_normal = 0.0; self.corners = []
class AprilTagDetectionArray:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id=''); self.camera_name = ''
        self.image_height = self.image_width = 0; self.detections = []
rmm.AprilTagDetection, rmm.AprilTagDetectionArray = AprilTagDetection, AprilTagDetectionArray
sys.modules['robot_msgs'] = rm; sys.modules['robot_msgs.msg'] = rmm
paths = types.ModuleType('apriltag_nav.paths'); paths.load_yaml_block = lambda k: {}
sys.modules['apriltag_nav.paths'] = paths

import robot_camera_node as R  # noqa: E402

_n = [0]; _bad = [0]
def check(name, ok, detail=""):
    _n[0] += 1; _bad[0] += (not ok)
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")

def make_worker(**kw):
    w = R._CameraTagWorker('front_cam', '/i', '/ci', '/d', 'tag36h11', 0.09, CvBridge(), None, True,
                           quad_decimate=2.0, stop_columns={'FWD': 0.0, 'REV': 400.0}, ground_plane=None, **kw)
    w.camera_params = (910.0, 910.0, 640.0, 360.0)
    return w

def frame(t):
    m = Image(); m.header.stamp = _Stamp(t); m.img = np.zeros((720, 1280, 3), np.uint8); return m

# render cost instrumentation: count draw_overlay calls
draws = [0]
_orig_draw = R.draw_overlay
def _draw(*a, **k):
    draws[0] += 1; return _orig_draw(*a, **k)
R.draw_overlay = _draw

print("== overlay off the detection callback ==")
w = make_worker(overlay_hz=10.0)
w.overlay_pub.conns = 2
d0 = draws[0]
for k in range(5):
    w._image_cb(frame(k / 30.0))
check("5 frames detected", w.detector.calls == 5, str(w.detector.calls))
check("no overlay drawn inside the callback", draws[0] == d0, str(draws[0] - d0))
check("detections published for every frame", len(w.pub.msgs) == 5)
check("an overlay timer exists at 10 Hz", timers and abs(timers[-1].period.s - 0.1) < 1e-9)
timers[-1].cb()
check("one timer tick renders the newest frame once", draws[0] == d0 + 1 and len(w.overlay_pub.msgs) == 1
      and w.overlay_pub.msgs[-1].header.stamp == _Stamp(4 / 30.0))
timers[-1].cb()
check("a second tick without a new frame renders nothing", draws[0] == d0 + 1)
w._image_cb(frame(5 / 30.0)); timers[-1].cb()
check("a new frame is rendered on the next tick", draws[0] == d0 + 2)
w.overlay_pub.conns = 0
w._image_cb(frame(6 / 30.0)); timers[-1].cb()
check("nothing rendered while nobody subscribes", draws[0] == d0 + 2)
w.overlay_pub.conns = 1
w._image_cb(frame(7 / 30.0)); timers[-1].cb()
check("rendering resumes with a subscriber (one draw regardless of subscriber count)", draws[0] == d0 + 3)

print("\n== detect_max_hz ==")
w = make_worker(detect_max_hz=10.0)
for k in range(30):
    w._image_cb(frame(k / 30.0))
check("30 frames at 30 Hz -> 10 detections at 10 Hz", w.detector.calls == 10, str(w.detector.calls))
w = make_worker(detect_max_hz=None)
for k in range(30):
    w._image_cb(frame(k / 30.0))
check("null -> every frame", w.detector.calls == 30)

print("\n== detector threads ==")
Detector.made = []
make_worker(detector_threads=2)
check("nthreads handed to dt_apriltags", Detector.made and Detector.made[-1]['nthreads'] == 2)

print("\n== disable tears the overlay timer down ==")
w = make_worker(overlay_hz=10.0)
tm = timers[-1]
w._apply(False)
check("timer shut down and source cleared", not tm.alive and w._overlay_src is None)

print("\n%d checks, %d failed" % (_n[0], _bad[0]))
sys.exit(1 if _bad[0] else 0)
