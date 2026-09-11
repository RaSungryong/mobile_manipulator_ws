"""
detections.py
=============
Tag observation through the main stack's shared detector — replaces the
in-package ``dt_apriltags`` runs over raw camera frames.

``robot_camera_node`` is the only tag detector in the stack (one detector,
two consumers); it publishes ``robot_msgs/AprilTagDetectionArray`` per
camera. This module turns one of those detections back into the 4x4
``T_cam2tag`` the chain math consumes.

Size rescaling
--------------
dt_apriltags scales ``pose_t`` linearly with the ``tag_size`` it was given.
The shared detector runs one size per camera (robot.yaml
``robot_camera.tag_size``), but reference tags may differ per tag
(reference_tags.yaml ``size_m``). Since translation is linear in size,
``t_actual = t_detected * (actual_size / detector_size)`` recovers the
true translation without a re-detection; rotation is size-independent.

⚠️ front_cam's pose fields are NOT a 6-DOF measurement
------------------------------------------------------
With the ground-plane correction on (robot.yaml
``robot_camera.ground_plane.front_cam``, 2026-09-08) ``robot_camera_node``
publishes a NAVIGATION-shaped detection: ``pose_x/pose_y`` are ground
coordinates relative to the lens nadir, ``pose_z`` is the CONFIGURED lens
height (a constant — the measured depth is gone), and the orientation
fields keep dt_apriltags' RAW, uncorrected values. That is correct for
`mobile_controller`, which works in pixels with ``z / fx`` scaling, but it
is not a pose: the chain would get corrected position + uncorrected
rotation + an asserted depth.

This is structural, not a coding slip — ``GroundPlane.to_ground()``
intersects every ray with the floor plane, so depth is an INPUT
assumption and no 3D rotation is ever formed.

The corrected CORNERS, however, *are* published, and they are the pixels
of a level, distortion-free virtual camera with the SAME intrinsics. So
the honest 6-DOF pose is recovered here, by the consumer that needs it,
with :func:`pose_from_corners` — leaving robot_camera_node and the
verified navigation behaviour untouched.
"""
import threading

import numpy as np
import rospy
from scipy.spatial.transform import Rotation as _Rot

from robot_msgs.msg import AprilTagDetectionArray

try:
    import cv2
except ImportError:            # the euler path still works without it
    cv2 = None


def wait_for_tag_detection(topic: str, tag_id: int, timeout: float = 3.0):
    """Block until ``tag_id`` appears on ``topic``; return the detection.

    Frames without the tag are skipped (the array arrives per processed
    frame whether or not any tag is visible). Raises ``RuntimeError`` on
    timeout — same contract the old detect-over-image path had.

    ONE subscriber lives for the whole wait. The earlier
    ``wait_for_message``-per-second loop created and tore down a
    subscriber up to 3x per call; each cycle has connect latency during
    which frames are missed, which with marginal tag visibility turned
    into spurious timeouts.
    """
    found = {}
    event = threading.Event()

    def _cb(arr):
        for det in arr.detections:
            if int(det.id) == int(tag_id):
                found['det'] = det
                event.set()
                return

    sub = rospy.Subscriber(topic, AprilTagDetectionArray, _cb,
                           queue_size=1)
    try:
        deadline = rospy.Time.now() + rospy.Duration(float(timeout))
        while not rospy.is_shutdown():
            if event.wait(0.05):
                return found['det']
            if rospy.Time.now() >= deadline:
                raise RuntimeError(
                    f'tag {tag_id} not detected on {topic} within '
                    f'{timeout:.1f}s')
        raise RuntimeError('wait_for_tag_detection: rospy shutdown')
    finally:
        sub.unregister()


def wait_for_tag_detections(topic: str, tag_id: int, n: int,
                            timeout: float = 3.0):
    """Collect up to ``n`` consecutive detections of ``tag_id`` (one per
    frame) within ``timeout``; returns the list (>= 1 entry, else raises
    like :func:`wait_for_tag_detection`). Lets the align loop take a
    median instead of trusting one frame — the tilt of a 70 px tag is
    noisy (std 0.5 deg, spikes of 3-4 deg measured 2026-09-02)."""
    n = max(1, int(n))
    found = []
    lock = threading.Lock()
    event = threading.Event()

    def _cb(arr):
        for det in arr.detections:
            if int(det.id) == int(tag_id):
                with lock:
                    found.append(det)
                    if len(found) >= n:
                        event.set()
                return

    sub = rospy.Subscriber(topic, AprilTagDetectionArray, _cb,
                           queue_size=1)
    try:
        deadline = rospy.Time.now() + rospy.Duration(float(timeout))
        while not rospy.is_shutdown():
            if event.wait(0.05):
                break
            if rospy.Time.now() >= deadline:
                break
        with lock:
            if not found:
                raise RuntimeError(
                    f'tag {tag_id} not detected on {topic} within '
                    f'{timeout:.1f}s')
            return list(found)
    finally:
        sub.unregister()


def median_tilt_detection(dets):
    """The detection whose ``tilt_from_normal`` is the median of the
    batch — a robust pick that keeps a REAL frame (no averaging of
    rotations) while discarding the tilt spikes."""
    dets = sorted(dets, key=lambda d: float(d.tilt_from_normal))
    return dets[len(dets) // 2]


def mean_detection(dets):
    """Average a batch of detections of the SAME tag into one synthetic
    detection: translation and pixel fields arithmetically, angles via
    their sin/cos means (roll sits near ±180 for face-up tags, where an
    arithmetic mean wraps catastrophically).

    Averaging divides every noise component — including the in-plane
    yaw, which the error budget shows dominates the path-tag position
    error through the A->B lever — by ~sqrt(n). Median-by-tilt (above)
    only rejects spikes; use THIS for the final chain observation, the
    median for align-loop convergence checks.
    """
    if len(dets) == 1:
        return dets[0]
    out = type(dets[0])()
    out.id = dets[0].id
    for f in ('center_x', 'center_y', 'pose_x', 'pose_y', 'pose_z',
              'tilt_from_normal'):
        setattr(out, f, float(np.mean([getattr(d, f) for d in dets])))
    for f in ('roll', 'pitch', 'yaw'):
        ang = np.radians([float(getattr(d, f)) for d in dets])
        setattr(out, f, float(np.degrees(
            np.arctan2(np.mean(np.sin(ang)), np.mean(np.cos(ang))))))
    out.corners = [float(v) for v in
                   np.mean([np.asarray(d.corners) for d in dets], axis=0)]
    return out


# AprilTag's tag frame, in the corner order dt_apriltags reports:
# corner0 = (-h, +h), then +x, then -y — established EMPIRICALLY against
# the library (scratch t_convention.py: this ordering reproduces
# dt_apriltags' own pose_R/pose_t from its own corners to 0.53 deg /
# 0.15 mm, while the other candidate ordering is 42-180 deg out). Against
# a rendered ground truth both agree to 0.23 deg / 0.11 mm, i.e. the
# re-solve is as accurate as the library's own estimate.
_CORNER_ORDER = np.array([[-1.0, +1.0], [+1.0, +1.0],
                          [+1.0, -1.0], [-1.0, -1.0]])


def pose_from_corners(corners, tag_size_m: float, K, dist=None) -> np.ndarray:
    """T_cam2tag (4x4, metres) solved from the four image corners.

    ``corners`` is the flat 8-vector (or (4,2)) carried by
    AprilTagDetection, ``K`` the 3x3 intrinsics. With the ground-plane
    correction on, the corners are the level virtual camera's pixels and
    that camera has the SAME K and ZERO distortion, so ``dist`` must stay
    None; pass the real D only when re-solving RAW corners.

    Unlike the euler path this needs no size rescale — the tag's actual
    size goes straight into the object points.
    """
    if cv2 is None:
        raise RuntimeError("pose_from_corners needs cv2")
    img = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    obj = np.zeros((4, 3), dtype=np.float64)
    obj[:, :2] = _CORNER_ORDER * (float(tag_size_m) / 2.0)
    ok, rvec, tvec = cv2.solvePnP(obj, img, np.asarray(K, dtype=np.float64),
                                  None if dist is None else np.asarray(dist, float),
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("solvePnP failed on the tag corners")
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).ravel()
    return T


def detection_to_T_cam2tag(det,
                           actual_size_m: float,
                           detector_size_m: float,
                           camera_K=None) -> np.ndarray:
    """Reconstruct T_cam2tag (4x4, metres) from an AprilTagDetection.

    ``camera_K`` given -> the pose is SOLVED FROM THE CORNERS
    (:func:`pose_from_corners`), which is the only correct route for
    front_cam once the ground-plane correction is on (see the module
    docstring). ``camera_K`` None -> the legacy euler path below, still
    right for any camera whose pose fields are a real measurement.

    robot_camera_node encodes ``pose_R`` as
    ``as_euler('zyx', degrees=True)[::-1]`` (scipy LOWERCASE 'zyx', i.e.
    extrinsic — despite the message comment calling it "ZYX-intrinsic").
    The exact inverse is ``from_euler('zyx', [yaw, pitch, roll])`` —
    undoing the ``[::-1]`` and feeding the same sequence string back.
    ⚠️ Do NOT use ``geometry.rpy_deg_to_R`` here: that helper is the
    Rz·Ry·Rx intrinsic convention (Fairino TCP poses), which only agrees
    with this encoding for small angles.

    Translation is rescaled from the detector's tag size to the tag's
    actual size (see module docstring).
    """
    if camera_K is not None:
        return pose_from_corners(det.corners, actual_size_m, camera_K)
    scale = float(actual_size_m) / float(detector_size_m)
    rot = _Rot.from_euler(
        'zyx', [float(det.yaw), float(det.pitch), float(det.roll)],
        degrees=True)
    # scipy >= 1.4 spells it as_matrix(); 1.3 only has as_dcm().
    R = rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.array([det.pose_x, det.pose_y, det.pose_z],
                        dtype=np.float64) * scale
    return T
