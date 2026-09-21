#!/usr/bin/env python3
"""Offline checks for the per-camera intrinsics override (2026-09-21):
apriltag_nav/camera_intrinsics.py + robot_camera_node's use of it.

    python3 src/apriltag_nav/tools/check_camera_intrinsics_override.py
"""
import os, sys, types, tempfile
import numpy as np
import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
from apriltag_nav.camera_intrinsics import (IntrinsicsOverride, Rectifier, intrinsics_override,
                                            effective_intrinsics, rectify_raw_frame)
from apriltag_nav.paths import CONFIG_PATH

fails = 0
N_CHECKS = [0]
def check(cond, msg):
    global fails
    N_CHECKS[0] += 1
    print(("  [ok ] " if cond else "  [FAIL] ") + msg)
    if not cond: fails += 1

# --- 1. the robot.yaml entry ------------------------------------------------
o = intrinsics_override('hand_cam')
check(o is not None, "robot.yaml carries robot_camera.intrinsics_override.hand_cam")
check(intrinsics_override('front_cam') is None and intrinsics_override('side_cam') is None,
      "front_cam / side_cam have no override (driver CameraInfo stays in use)")
fx, fy, cx, cy = o.camera_params
check(590 < fx < 615 and 590 < fy < 615 and 300 < cx < 340 and 220 < cy < 260,
      "hand_cam K is a 640x480 D435 colour K (fx %.1f fy %.1f cx %.1f cy %.1f)" % (fx, fy, cx, cy))
check(o.has_distortion and 0.10 < o.D[0] < 0.20 and -0.45 < o.D[1] < -0.20,
      "hand_cam D is the fitted k1 / k2 (%.4f / %.4f)" % (o.D[0], o.D[1]))
check(o.image_size == (640, 480), "image_size 640x480")
K_drv = [609.30, 0, 321.47, 0, 608.62, 238.67, 0, 0, 1]
K, D, src = effective_intrinsics('hand_cam', K_drv, [0, 0, 0, 0, 0])
check(np.allclose(K, o.K) and not np.any(D) and 'override' in src,
      "effective_intrinsics(hand_cam) = (K_override, D = 0): detections are rectified")
K, D, src = effective_intrinsics('front_cam', K_drv, [0.08, -0.11, 0, 0, 0.05])
check(np.allclose(K, np.reshape(K_drv, (3, 3))) and abs(D[0] - 0.08) < 1e-9 and src == 'driver',
      "effective_intrinsics(front_cam) = the driver's K and D unchanged")
check(IntrinsicsOverride.from_dict('x', {'enabled': False, 'K': K_drv}) is None, "enabled: false = no override")
with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
    f.write("robot_camera:\n  tag_size: {hand_cam: 0.09}\n"); tmp = f.name
check(intrinsics_override('hand_cam', tmp) is None, "a config without the block = no override")
os.unlink(tmp)

# --- 2. the rectifier: distorted pixels -> ideal pixels ------------------------
Kf = o.K; Df = o.D
rng = np.random.default_rng(0)
pts3 = np.column_stack([rng.uniform(-0.25, 0.25, 200), rng.uniform(-0.18, 0.18, 200), rng.uniform(0.35, 0.6, 200)])
uv_dist, _ = cv2.projectPoints(pts3, np.zeros(3), np.zeros(3), Kf, Df)
uv_ideal, _ = cv2.projectPoints(pts3, np.zeros(3), np.zeros(3), Kf, np.zeros(5))
uv_dist = uv_dist.reshape(-1, 2); uv_ideal = uv_ideal.reshape(-1, 2)
inside = (uv_dist[:, 0] > 5) & (uv_dist[:, 0] < 635) & (uv_dist[:, 1] > 5) & (uv_dist[:, 1] < 475)
shift = np.linalg.norm(uv_dist - uv_ideal, axis=1)[inside]
check(shift.max() > 3.0, "the fitted distortion moves points by up to %.1f px in frame (it is not negligible)" % shift.max())
r = Rectifier(Kf, Df, (640, 480))
back = r.undistort_points(uv_dist[inside])
check(np.abs(back - uv_ideal[inside]).max() < 1e-3,
      "undistort_points maps distorted pixels onto the ideal projection (max %.2e px)" % np.abs(back - uv_ideal[inside]).max())
# an image with a dot at each DISTORTED position rectifies to dots at the IDEAL positions
# (sub-pixel circles, dots >= 24 px apart so each centroid window holds one dot)
img = np.zeros((480, 640), np.uint8)
sel = []
for i in np.where(inside)[0]:
    if all(np.hypot(*(uv_ideal[i] - uv_ideal[j])) > 24 for j in sel):
        sel.append(i)
    if len(sel) >= 40: break
for i in sel:
    cv2.circle(img, (int(round(uv_dist[i, 0] * 16)), int(round(uv_dist[i, 1] * 16))), 3 * 16, 255, -1, cv2.LINE_AA, shift=4)
rect = r.rectify(img)
errs = []
for i in sel:
    x, y = uv_ideal[i]
    y0, y1, x0, x1 = max(0, int(y) - 8), int(y) + 9, max(0, int(x) - 8), int(x) + 9
    win = rect[y0:y1, x0:x1].astype(float)
    if win.sum() == 0: errs.append(99); continue
    ys, xs = np.mgrid[y0:y1, x0:x1]
    errs.append(np.hypot((xs * win).sum() / win.sum() - x, (ys * win).sum() / win.sum() - y))
check(max(errs) < 0.5, "a remapped frame puts each dot within 0.5 px of its ideal position (max %.2f px over %d dots)" % (max(errs), len(sel)))
try:
    r.rectify(np.zeros((720, 1280), np.uint8)); check(False, "size mismatch refused")
except ValueError:
    check(True, "a frame of another size is refused instead of silently remapped")
img2, K2, src2 = rectify_raw_frame('hand_cam', np.dstack([img] * 3), np.reshape(K_drv, (3, 3)))
check(np.allclose(K2, Kf) and 'override' in src2 and np.array_equal(img2[:, :, 0], rect),
      "rectify_raw_frame: raw-frame consumers get the remapped image + K_override")
img3, K3, src3 = rectify_raw_frame('front_cam', img, np.reshape(K_drv, (3, 3)))
check(img3 is img and src3 == 'driver', "rectify_raw_frame without an override returns the frame untouched")

# --- 3. robot_camera_node honours it (rospy stubbed as in check_robot_camera_latency) ------
class _Pub:
    def __init__(self, *a, **k): self.msgs = []
    def publish(self, m): self.msgs.append(m)
    def get_num_connections(self): return 0
rospy = types.ModuleType('rospy'); rospy.Publisher = _Pub
rospy.Subscriber = lambda *a, **k: types.SimpleNamespace(unregister=lambda: None)
rospy.Service = lambda *a, **k: None; rospy.Timer = lambda *a, **k: None; rospy.Duration = lambda s: s
for lv in ('loginfo', 'logwarn', 'logerr', 'logdebug', 'logwarn_throttle', 'loginfo_throttle'):
    setattr(rospy, lv, lambda *a, **k: None)
rospy.get_param = lambda k, d=None: d; rospy.init_node = lambda *a, **k: None
sys.modules['rospy'] = rospy
for name, attrs in (('sensor_msgs.msg', {'Image': type('Image', (), {}), 'CameraInfo': type('CameraInfo', (), {})}),
                    ('std_srvs.srv', {'SetBool': object, 'SetBoolResponse': object}),
                    ('cv_bridge', {'CvBridge': type('CvBridge', (), {'imgmsg_to_cv2': lambda s, m, e: m.img})}),
                    ('robot_msgs.msg', {'AprilTagDetection': type('D', (), {}), 'AprilTagDetectionArray': type('A', (), {})})):
    mod = types.ModuleType(name)
    for k, v in attrs.items(): setattr(mod, k, v)
    sys.modules[name] = mod
    sys.modules.setdefault(name.split('.')[0], types.ModuleType(name.split('.')[0]))
dta = types.ModuleType('dt_apriltags')
class _Detector:
    def __init__(self, *a, **k): pass
    def detect(self, *a, **k): return []
dta.Detector = _Detector; sys.modules['dt_apriltags'] = dta
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))
import importlib.util
spec = importlib.util.spec_from_file_location('rcn', os.path.join(HERE, '..', 'scripts', 'robot_camera_node.py'))
R = importlib.util.module_from_spec(spec); spec.loader.exec_module(R)
def worker(override):
    return R._CameraTagWorker('hand_cam', '/i', '/ci', '/d', 'tag36h11', 0.09, sys.modules['cv_bridge'].CvBridge(),
                              None, True, intrinsics_override=override)
info = types.SimpleNamespace(K=K_drv, D=[0.0] * 5, width=640, height=480)
w = worker(o); w._info_cb(info)
check(np.allclose(w.camera_params, o.camera_params) and w.rectifier is not None and w.camera_dist == [],
      "worker with the override: camera_params = K_override, frames remapped, D = 0 for the ground plane")
w0 = worker(None); w0._info_cb(info)
check(np.allclose(w0.camera_params, [609.30, 608.62, 321.47, 238.67]) and w0.rectifier is None,
      "worker without an override: the driver's K, no remap (pre-change behaviour)")
w1 = worker(o); w1._info_cb(types.SimpleNamespace(K=K_drv, D=[], width=1280, height=720))
check(w1.rectifier is None and np.allclose(w1.camera_params, [609.30, 608.62, 321.47, 238.67]),
      "a stream at another size than the override was fitted at: override ignored, driver K kept")
oD0 = IntrinsicsOverride('hand_cam', o.K, np.zeros(5), (640, 480))
w2 = worker(oD0); w2._info_cb(info)
check(w2.rectifier is None and np.allclose(w2.camera_params, o.camera_params),
      "an override with D = 0 changes camera_params only, no remap cost")
# the frame handed to the detector is the remapped one
seen = {}
class _Det2(_Detector):
    def detect(self, gray, **k): seen['gray'] = gray; return []
w.detector = _Det2(); w.pub = _Pub(); w.overlay_pub = _Pub()
msg = types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda: 1.0), frame_id=''),
                            img=np.dstack([img] * 3))
w.detect_min_period = 0.0; w._image_cb(msg)
check('gray' in seen and np.array_equal(seen['gray'], rect), "the detector sees the remapped frame, not the raw one")

print("\n%d checks, %d failed" % (N_CHECKS[0], fails))
sys.exit(1 if fails else 0)
