"""Per-camera intrinsics OVERRIDE (robot.yaml ``robot_camera.intrinsics_override``).

Why this exists (2026-09-21): the RealSense D435 hand_cam driver publishes
K = (609.3, 608.6, 321.5, 238.7) and D = 0 on ``/hand_cam/color/camera_info``.
``cv2.calibrateCamera`` over the stored A0-sheet corners of the two
2026-09-21 chain_calib sessions (60 + 52 views, k1 k2 model, both halves of
each session agreeing to 1 px of fx and 0.01 of k1) gives fx 601.7 / fy
603.9 / cx 322.0 / cy 238.5 and k1 +0.160, k2 -0.322: the lens IS distorted
(4.5 px at r = 250 px) and fx / fy are 1.2 % / 0.8 % lower than reported.
The corner-fit rms drops 0.8 -> 0.27 px (the corner floor), the arm
session's sheet-position scatter through the chain 31.8 -> 16.1 mm, and
the fy/fx ratio change (+0.46 %) is the "y-only 0.35 % shrink" the chain
session had seen in the print scale. Nothing in the drivers can be told a
different K, so every consumer applies this file's value instead.

Contract — the ONE rule every consumer follows:

* ``robot_camera_node`` REMAPS the frame with (K, D) -> a distortion-free
  image whose intrinsics are K (same K as ``newCameraMatrix``), detects on
  it, and publishes ``/<cam>/tag_detections`` in that RECTIFIED frame.
* therefore a consumer of the DETECTIONS (chain_calib capture,
  basler_tip_ros, the locator chain, the hand-eye sweep) uses
  ``effective_intrinsics(cam, ...)`` = (K_override, D = 0).
* a consumer of RAW frames (handeye_calib_node's archived samples) rectifies
  the image itself with ``Rectifier`` and stores K_override with it.
* a session captured BEFORE the override (raw corners, driver K in its
  meta) is re-solved with the override's (K, D) on its raw corners —
  ``arm_offsets.py / chain_calib.py solve --hand-intrinsics config``.

No override entry (or ``enabled: false``) = the driver's CameraInfo,
bit-for-bit the pre-2026-09-21 behaviour.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class IntrinsicsOverride:
    camera: str
    K: np.ndarray                 # (3, 3)
    D: np.ndarray                 # (5,) plumb_bob k1 k2 p1 p2 k3
    image_size: Optional[Tuple[int, int]] = None   # (width, height) the K was fitted at
    note: str = ""

    @property
    def camera_params(self):
        """(fx, fy, cx, cy) — the dt_apriltags argument."""
        return [float(self.K[0, 0]), float(self.K[1, 1]), float(self.K[0, 2]), float(self.K[1, 2])]

    @property
    def has_distortion(self) -> bool:
        return bool(np.any(np.abs(self.D) > 0))

    @staticmethod
    def from_dict(camera: str, d) -> Optional["IntrinsicsOverride"]:
        if not d or not d.get("enabled", True):
            return None
        K = np.asarray(d["K"], dtype=float).reshape(3, 3)
        D = np.zeros(5)
        dd = np.asarray(d.get("D") or [], dtype=float).ravel()
        D[:min(5, dd.size)] = dd[:5]
        size = d.get("image_size")
        size = (int(size[0]), int(size[1])) if size else None
        return IntrinsicsOverride(camera, K, D, size, str(d.get("note", "")))


def load_config_block(config_path=None) -> dict:
    import yaml
    if config_path is None:
        from apriltag_nav.paths import CONFIG_PATH      # lazy: test harnesses stub apriltag_nav
        config_path = CONFIG_PATH
    with open(config_path) as f:
        cfg = yaml.safe_load(f) or {}
    return (cfg.get("robot_camera") or {}).get("intrinsics_override") or {}


def intrinsics_override(camera: str, config_path=None) -> Optional[IntrinsicsOverride]:
    """The override for ``camera`` ('hand_cam', ...) or None."""
    return IntrinsicsOverride.from_dict(camera, load_config_block(config_path).get(camera))


def effective_intrinsics(camera: str, K_driver, D_driver=None, config_path=None):
    """(K, D, source) for a consumer of robot_camera_node's DETECTIONS.

    With an override the node publishes rectified-frame detections, so the
    intrinsics to use are (K_override, D = 0). Without one, the driver's."""
    o = intrinsics_override(camera, config_path)
    if o is None:
        D = np.asarray(D_driver, dtype=float).ravel() if D_driver is not None and len(D_driver) else np.zeros(5)
        return np.asarray(K_driver, dtype=float).reshape(3, 3), D, "driver"
    return o.K.copy(), np.zeros(5), "override (rectified detections)"


class Rectifier:
    """Undistort frames with (K, D) so the result has intrinsics K and no
    distortion. Maps are built once per image size (~1 ms per 640x480
    remap afterwards)."""

    def __init__(self, K, D, image_size: Tuple[int, int]):
        import cv2
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.D = np.asarray(D, dtype=float).ravel()
        self.size = (int(image_size[0]), int(image_size[1]))
        self._m1, self._m2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, self.K, self.size, cv2.CV_16SC2)
        self._cv2 = cv2

    def rectify(self, img):
        if img.shape[1] != self.size[0] or img.shape[0] != self.size[1]:
            raise ValueError("Rectifier built for %dx%d, frame is %dx%d"
                             % (self.size[0], self.size[1], img.shape[1], img.shape[0]))
        return self._cv2.remap(img, self._m1, self._m2, self._cv2.INTER_LINEAR)

    def undistort_points(self, pts):
        """(N, 2) raw pixels -> rectified pixels (same K)."""
        p = np.asarray(pts, dtype=float).reshape(-1, 1, 2)
        return self._cv2.undistortPoints(p, self.K, self.D, P=self.K).reshape(-1, 2)


def rectify_raw_frame(camera: str, img, K_driver, config_path=None):
    """For RAW-frame consumers: (image, K) to store. With an override the
    frame is remapped and K is the override's; otherwise both unchanged."""
    o = intrinsics_override(camera, config_path)
    if o is None:
        return img, np.asarray(K_driver, dtype=float).reshape(3, 3), "driver"
    r = Rectifier(o.K, o.D, (img.shape[1], img.shape[0]))
    return r.rectify(img), o.K.copy(), "override (frame rectified)"
