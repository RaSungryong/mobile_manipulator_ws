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
