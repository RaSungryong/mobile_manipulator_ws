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
"""
import threading

import numpy as np
import rospy
from scipy.spatial.transform import Rotation as _Rot

from robot_msgs.msg import AprilTagDetectionArray


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


def detection_to_T_cam2tag(det,
                           actual_size_m: float,
                           detector_size_m: float) -> np.ndarray:
    """Reconstruct T_cam2tag (4x4, metres) from an AprilTagDetection.

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
