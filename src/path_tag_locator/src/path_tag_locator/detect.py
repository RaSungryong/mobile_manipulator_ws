"""
detect.py
=========
AprilTag detection wrapper. Returns 4x4 pose of tag in camera frame.

Uses ``dt_apriltags`` (Duckietown). The Detection objects expose ``tag_id``,
``pose_R`` (3x3), and ``pose_t`` (3x1) when ``estimate_tag_pose=True``.
"""
import numpy as np
import cv2

from .geometry import apriltag_to_matrix


_DETECTORS = {}


def _get_detector(family: str):
    det = _DETECTORS.get(family)
    if det is None:
        from dt_apriltags import Detector  # lazy import
        det = Detector(families=family)
        _DETECTORS[family] = det
    return det


def detect_apriltag(image_bgr: np.ndarray,
                    K: np.ndarray,
                    tag_size_m: float,
                    tag_id: int,
                    family: str = "tag36h11"):
    """Detect ``tag_id`` in ``image_bgr`` and return 4x4 T_cam2tag (m),
    or ``None`` if not present.
    """
    if image_bgr is None:
        return None
    if image_bgr.ndim == 3:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    else:
        gray = image_bgr

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])

    detections = _get_detector(family).detect(
        gray,
        estimate_tag_pose=True,
        camera_params=(fx, fy, cx, cy),
        tag_size=tag_size_m,
    )
    for d in detections:
        if d.tag_id == tag_id:
            return apriltag_to_matrix(d.pose_R, d.pose_t)
    return None
