#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_repose_from_corners.py
============================
OFFLINE self-check for ``detections.pose_from_corners`` — no ROS master,
no robot, no cameras. Run it after touching detections.py, ground_plane.py
or the front_cam extrinsics::

    python3 src/path_tag_locator/scripts/check_repose_from_corners.py

Why this exists
---------------
With ``robot_camera.ground_plane.front_cam`` enabled, robot_camera_node
publishes a NAVIGATION-shaped detection: pose_x/pose_y are ground
coordinates relative to the lens nadir, pose_z is the CONFIGURED lens
height, and the orientation fields are dt_apriltags' RAW uncorrected
values. Good for mobile_controller, wrong for the locator chain, which
needs a real 6-DOF T_fc2B. The corrected CORNERS are published though, so
the chain re-solves the pose from them.

That re-solve hinges on one convention — the object-point ordering that
matches dt_apriltags' corner order. Get it wrong and every calibrated tag
position is silently corrupted (the wrong ordering is 42-180 deg out, not
subtly off). These checks pin it down, against the real library where it
is available.

Checks
------
1-3  round trip: project a known pose -> corners -> solve -> same pose
4    the WRONG corner ordering is caught (a guard against silent drift)
5    ground-plane corrected corners recover the TRUE pose, while the
     published pose/rpy fields do not — the whole point of the change
6    tag size scales translation linearly and leaves rotation alone
7    (only with dt_apriltags + cv2.aruco) the ordering reproduces the
     library's own pose from the library's own corners
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WS = os.path.dirname(os.path.dirname(PKG))
sys.path.insert(0, os.path.join(PKG, "src"))
sys.path.insert(0, os.path.join(WS, "src", "apriltag_nav", "src"))

try:
    import cv2
except ImportError:
    print("FAIL: cv2 is required")
    sys.exit(1)

# detections.py imports rospy + robot_msgs at module level for the
# subscriber helpers. The geometry under test needs neither, so stub them
# and the check runs on a dev box with no ROS install.
try:
    import rospy  # noqa: F401
except ImportError:
    import types
    _r = types.ModuleType("rospy")
    for _f in ("loginfo", "logwarn", "logerr", "logwarn_once", "sleep"):
        setattr(_r, _f, lambda *a, **k: None)
    _r.is_shutdown = lambda: False
    sys.modules["rospy"] = _r
    _m = types.ModuleType("robot_msgs")
    _mm = types.ModuleType("robot_msgs.msg")
    _mm.AprilTagDetectionArray = object
    _m.msg = _mm
    sys.modules["robot_msgs"] = _m
    sys.modules["robot_msgs.msg"] = _mm

from path_tag_locator.detections import pose_from_corners, _CORNER_ORDER
from apriltag_nav.ground_plane import GroundPlane

FX = FY = 910.0
CX, CY = 640.0, 360.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], float)
TAG = 0.090

_n = [0]
_bad = [0]


def check(name, ok, detail=""):
    _n[0] += 1
    if not ok:
        _bad[0] += 1
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")


def rot(rx, ry, rz):
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy, sy = math.cos(rz), math.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def obj_pts(order, size=TAG):
    p = np.zeros((4, 3))
    p[:, :2] = order * (size / 2.0)
    return p


def project(R, t, order=_CORNER_ORDER, size=TAG, K_=None):
    """Ideal pinhole projection of the four tag corners."""
    K_ = K if K_ is None else K_
    pts = (obj_pts(order, size) @ R.T) + t
    uv = (pts / pts[:, 2:3]) @ K_.T
    return uv[:, :2]


def angdiff(A, B):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(A.T @ B) - 1) / 2))))


print(__doc__.split("Checks")[0].strip().splitlines()[0])
print("\n== round trip: pose -> corners -> pose ==")
POSES = []
rng = np.random.RandomState(3)
for _ in range(30):
    R = rot(*np.radians(rng.uniform(-30, 30, 3)))
    t = np.array([rng.uniform(-.08, .08), rng.uniform(-.08, .08),
                  rng.uniform(0.25, 0.60)])
    POSES.append((R, t))

# thresholds are set by SOLVEPNP_ITERATIVE's convergence tolerance, not by
# the geometry: 1e-4 deg / 1e-4 mm is ~4 orders below anything that matters
dR = dT = 0.0
for R, t in POSES:
    T = pose_from_corners(project(R, t).ravel(), TAG, K)
    dR = max(dR, angdiff(R, T[:3, :3]))
    dT = max(dT, np.linalg.norm(T[:3, 3] - t) * 1e3)
check("30 random poses recovered", dR < 1e-4 and dT < 1e-4,
      f"worst {dR:.2e} deg / {dT:.2e} mm")

R, t = rot(0, 0, 0), np.array([0.0, 0.0, 0.302])
T = pose_from_corners(project(R, t).ravel(), TAG, K)
check("square-on tag at the nadir", np.linalg.norm(T[:3, 3] - t) * 1e3 < 1e-4
      and angdiff(R, T[:3, :3]) < 1e-4)

flat = np.asarray(project(*POSES[0])).ravel().tolist()
check("accepts the flat 8-vector the message carries",
      np.allclose(pose_from_corners(flat, TAG, K),
                  pose_from_corners(np.asarray(flat).reshape(4, 2), TAG, K)))

print("\n== the wrong corner ordering must NOT look right ==")
WRONG = np.array([[-1.0, -1.0], [+1.0, -1.0], [+1.0, +1.0], [-1.0, +1.0]])
R, t = POSES[1]
T = pose_from_corners(project(R, t, order=WRONG).ravel(), TAG, K)
check("mirrored ordering is grossly wrong, not subtly",
      angdiff(R, T[:3, :3]) > 5.0, f"{angdiff(R, T[:3,:3]):.1f} deg off")

print("\n== ground-plane corrected corners recover the true pose ==")
# The real front_cam: tilted, at 302 mm, with the fitted numbers.
gp = GroundPlane(FX, FY, CX, CY, None, math.radians(1.228),
                 math.radians(-0.504), 0.302, math.radians(-0.38))
# A tag lying FLAT on the floor, 0.18 m ahead and 0.04 m right of the nadir,
# spun 12 deg in its own plane. In the ground frame its corners are at z=0.
spin = math.radians(12.0)
c, s = math.cos(spin), math.sin(spin)
Rz = np.array([[c, -s], [s, c]])
ground_corners = (obj_pts(_CORNER_ORDER)[:, :2] @ Rz.T) + np.array([0.18, 0.04])
raw_px = gp.project(ground_corners)              # what the tilted camera sees
vc = gp.to_virtual_px(gp.to_ground(raw_px))      # what robot_camera_node publishes

T_corr = pose_from_corners(vc.ravel(), TAG, K)
# truth in the LEVEL virtual camera: the tag is flat at depth h.
R_true = np.eye(3)
R_true[:2, :2] = Rz
t_true = np.array([0.18, 0.04, gp.h])
check("corrected corners -> true position",
      np.linalg.norm(T_corr[:3, 3] - t_true) * 1e3 < 0.05,
      f"{np.linalg.norm(T_corr[:3,3]-t_true)*1e3:.4f} mm")
check("corrected corners -> true orientation (flat tag reads flat)",
      angdiff(R_true, T_corr[:3, :3]) < 0.05,
      f"{angdiff(R_true, T_corr[:3,:3]):.4f} deg")

# and the raw corners, which is what the pose/rpy fields describe, do not:
T_raw = pose_from_corners(raw_px.ravel(), TAG, K)
tilt_raw = angdiff(R_true, T_raw[:3, :3])
check("uncorrected corners carry the camera tilt (so the published "
      "pose/rpy cannot be used)", tilt_raw > 0.5,
      f"{tilt_raw:.3f} deg of false tag tilt")

print("\n== tag size ==")
R, t = POSES[2]
T1 = pose_from_corners(project(R, t, size=TAG).ravel(), TAG, K)
T2 = pose_from_corners(project(R, t, size=TAG).ravel(), TAG * 2, K)
check("translation scales linearly with the declared size",
      np.allclose(T2[:3, 3], T1[:3, 3] * 2, rtol=1e-9))
check("rotation is size-independent", angdiff(T1[:3, :3], T2[:3, :3]) < 1e-9)

print("\n== against dt_apriltags itself ==")
try:
    from dt_apriltags import Detector
    marker = cv2.aruco.drawMarker(
        cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36H11), 0, 400)
    marker = cv2.copyMakeBorder(marker, 50, 50, 50, 50,
                                cv2.BORDER_CONSTANT, value=255)
    det = Detector(families="tag36h11", nthreads=4, quad_decimate=1.0)
    worst_R = worst_T = 0.0
    n = 0
    # The warp must map the marker image's own TL,TR,BR,BL to the tag's
    # corners in the SAME visual order, else the code comes out mirrored and
    # is undetectable. That ordering is a property of the drawn bitmap and
    # is deliberately independent of _CORNER_ORDER — this check compares the
    # re-solve against the LIBRARY's pose, so it never needs to know how the
    # renderer and the detector label their corners.
    RENDER_ORDER = np.array([[-1.0, -1.0], [+1.0, -1.0],
                             [+1.0, +1.0], [-1.0, +1.0]])
    for R, t in POSES[:10]:
        uv = project(R, t, order=RENDER_ORDER).astype(np.float32)
        # the marker image's BLACK BORDER is the tag extent, not the quiet zone
        src = np.array([[50, 50], [450, 50], [450, 450], [50, 450]], np.float32)
        Hm = cv2.getPerspectiveTransform(src, uv)
        img = np.minimum(
            np.full((720, 1280), 255, np.uint8),
            cv2.warpPerspective(marker, Hm, (1280, 720),
                                borderMode=cv2.BORDER_CONSTANT, borderValue=255))
        ds = [d for d in det.detect(img, estimate_tag_pose=True,
              camera_params=(FX, FY, CX, CY), tag_size=TAG) if d.tag_id == 0]
        if not ds:
            continue
        T = pose_from_corners(ds[0].corners, TAG, K)
        worst_R = max(worst_R, angdiff(ds[0].pose_R, T[:3, :3]))
        worst_T = max(worst_T, np.linalg.norm(ds[0].pose_t.ravel() - T[:3, 3]) * 1e3)
        n += 1
    check(f"reproduces dt_apriltags' own pose from its own corners ({n} tags)",
          n >= 8 and worst_R < 1.5 and worst_T < 1.0,
          f"worst {worst_R:.3f} deg / {worst_T:.3f} mm")
except ImportError as e:
    print(f"  [skip] dt_apriltags / cv2.aruco unavailable ({e}) — "
          f"the convention is still pinned by check 4")

print(f"\n{_n[0] - _bad[0]}/{_n[0]} checks passed")
sys.exit(1 if _bad[0] else 0)
