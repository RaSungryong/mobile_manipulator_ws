"""
align.py
========
Pure-numpy logic for aligning the hand camera squarely onto an AprilTag.

Goal (per iteration):
    Given current T_ab2ee, T_hc2ee (calibration), and the latest detection
    T_cam2tag, compute a new T_ab2ee_target such that the tag appears at
    image center and the camera's optical axis is perpendicular to the tag
    plane. Clamp the step displacement for safety, then return both the
    raw target and the clamped target (the latter is what should be sent
    to ``MoveJ``).
"""
import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .geometry import invert_T, rot2rpy_deg


# What the recorded camera-frame numbers mean. persistence.py writes it
# ONCE at the top of result.yaml (``camera_frame_note``) so the file
# explains itself without repeating it per entry.
CAM_FRAME_NOTE = (
    "camera optical frame: +x = image right, +y = image down, +z = optical "
    "axis into the scene. position_m = tag centre relative to the camera "
    "centre (x, y are the lateral error from the optical axis, z the "
    "range); rpy_deg = tag orientation in that frame, ZYX intrinsic. "
    "Square-on tag: position [0, 0, z], rpy [0, 0, spin]."
)


@dataclass
class AlignMetrics:
    xy_offset_m: float        # sqrt(tx^2 + ty^2) of tag in cam frame
    z_distance_m: float       # tz of tag in cam frame
    tilt_deg: float           # angle between cam +z and tag +z
    # Full camera-frame observation the scalars above are derived from.
    t_cam_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)      # (tx, ty, tz)
    rpy_cam_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # tag in cam, ZYX

    def as_report(self) -> dict:
        """Camera-frame error as a plain dict for yaml / json (see
        CAM_FRAME_NOTE for the axes). Rounded to 1 um / 1e-6 deg."""
        return {
            "position_m": [round(float(v), 6) for v in self.t_cam_m],
            "rpy_deg": [round(float(v), 6) for v in self.rpy_cam_deg],
            "xy_offset_m": round(float(self.xy_offset_m), 6),
            "z_distance_m": round(float(self.z_distance_m), 6),
            "tilt_deg": round(float(self.tilt_deg), 6),
        }


@dataclass
class AlignStep:
    T_ab2ee_target: np.ndarray    # raw target before clamping
    T_ab2ee_step: np.ndarray      # target after step-size clamping (the actual MoveL target)
    clamped: bool                 # True if either translation or rotation was clamped
    delta_t_norm_m: float         # cartesian distance of the *clamped* step
    delta_rot_deg: float          # rotation magnitude of the *clamped* step


# ----------------------------------------------------------------------
def alignment_metrics(T_cam2tag: np.ndarray) -> AlignMetrics:
    """Compute alignment error metrics from a fresh tag detection."""
    t = T_cam2tag[:3, 3]
    tag_z_in_cam = T_cam2tag[:3, 2]
    # tilt = angle between (0,0,1) and tag_z_in_cam
    c = float(np.clip(tag_z_in_cam[2] / max(np.linalg.norm(tag_z_in_cam), 1e-12), -1.0, 1.0))
    tilt_deg = math.degrees(math.acos(c))
    rx, ry, rz = rot2rpy_deg(T_cam2tag[:3, :3])
    return AlignMetrics(
        xy_offset_m=float(math.hypot(float(t[0]), float(t[1]))),
        z_distance_m=float(t[2]),
        tilt_deg=float(tilt_deg),
        t_cam_m=(float(t[0]), float(t[1]), float(t[2])),
        rpy_cam_deg=(float(rx), float(ry), float(rz)),
    )


def tag_in_cam_report(T_cam2tag: np.ndarray) -> dict:
    """Camera-frame error of one tag observation, ready for result.yaml."""
    return alignment_metrics(T_cam2tag).as_report()


def _target_T_cam2tag(T_cam2tag_now: np.ndarray,
                      target_distance_m: float) -> np.ndarray:
    """Desired pose of the tag in the camera frame after alignment.

    Rotation: PRESERVE the current spin about the tag normal (z), zero
    only the tilt. Yaw is invisible to the metrics and ``is_converged``
    — it is a free parameter the calibration planner deliberately picks
    to keep the arm FLANGE inside reach (view_pose._target_T_A2hc).
    Targeting identity here (as this helper originally did) made every
    correction step servo that yaw back toward zero, swinging the
    vision_tip overhang out again and defeating the reach optimization.

    Translation = (0, 0, d) where d = ``target_distance_m`` if > 0, else
    the current z-component (preserve depth).
    """
    if target_distance_m and target_distance_m > 0.0:
        d = float(target_distance_m)
    else:
        d = float(T_cam2tag_now[2, 3])
    yaw = math.atan2(float(T_cam2tag_now[1, 0]), float(T_cam2tag_now[0, 0]))
    c, s = math.cos(yaw), math.sin(yaw)
    T = np.eye(4, dtype=np.float64)
    T[0, 0], T[0, 1] = c, -s
    T[1, 0], T[1, 1] = s, c
    T[2, 3] = d
    return T


def compute_target_ee_pose(T_ab2ee_now: np.ndarray,
                           T_hc2ee: np.ndarray,
                           T_cam2tag_now: np.ndarray,
                           target_distance_m: float = 0.0) -> np.ndarray:
    """Compute T_ab2ee_target that places the camera squarely on the tag.

    Derivation (user notation T_X2Y = pose of Y in X):
        T_ab2tag = T_ab2ee_now · inv(T_hc2ee) · T_cam2tag_now      (fixed)
        T_ab2hc_target = T_ab2tag · inv(T_target_cam2tag)
        T_ab2ee_target = T_ab2hc_target · T_hc2ee
    """
    T_target = _target_T_cam2tag(T_cam2tag_now, target_distance_m)
    T_ee2hc = invert_T(T_hc2ee)
    T_ab2tag = T_ab2ee_now @ T_ee2hc @ T_cam2tag_now
    T_ab2hc_target = T_ab2tag @ invert_T(T_target)
    return T_ab2hc_target @ T_hc2ee


# ----------------------------------------------------------------------
def _R_to_axis_angle(R: np.ndarray) -> Tuple[np.ndarray, float]:
    """3x3 rotation -> (unit axis, angle_rad). Angle is in [0, pi]."""
    cos_a = (np.trace(R) - 1.0) * 0.5
    cos_a = float(np.clip(cos_a, -1.0, 1.0))
    angle = math.acos(cos_a)
    if angle < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    if math.isclose(angle, math.pi, abs_tol=1e-6):
        # near pi: extract axis from diagonal
        M = (R + np.eye(3)) * 0.5
        axis = np.array([math.sqrt(max(M[0, 0], 0.0)),
                         math.sqrt(max(M[1, 1], 0.0)),
                         math.sqrt(max(M[2, 2], 0.0))])
        # signs from off-diagonal
        if M[0, 1] < 0: axis[1] = -axis[1]
        if M[0, 2] < 0: axis[2] = -axis[2]
        n = np.linalg.norm(axis)
        return axis / max(n, 1e-12), angle
    rx = R[2, 1] - R[1, 2]
    ry = R[0, 2] - R[2, 0]
    rz = R[1, 0] - R[0, 1]
    axis = np.array([rx, ry, rz]) / (2.0 * math.sin(angle))
    return axis, angle


def _axis_angle_to_R(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues formula. ``axis`` need not be unit length."""
    n = float(np.linalg.norm(axis))
    if n < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3, dtype=np.float64)
    k = axis / n
    K = np.array([[0.0, -k[2], k[1]],
                  [k[2], 0.0, -k[0]],
                  [-k[1], k[0], 0.0]], dtype=np.float64)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def clamp_step(T_ab2ee_now: np.ndarray,
               T_ab2ee_target: np.ndarray,
               max_step_m: float,
               max_step_deg: float) -> AlignStep:
    """Clamp the proposed motion so neither translation nor rotation
    exceeds the configured per-step thresholds.

    Returns the *step* target (what we actually send to MoveL). When the
    raw target lies inside both thresholds, ``step == target``.
    """
    delta = invert_T(T_ab2ee_now) @ T_ab2ee_target
    R_delta = delta[:3, :3]
    t_delta = delta[:3, 3].copy()

    clamped = False

    t_norm = float(np.linalg.norm(t_delta))
    if t_norm > max_step_m > 0.0:
        t_delta *= (max_step_m / t_norm)
        clamped = True
        t_norm = max_step_m

    axis, angle = _R_to_axis_angle(R_delta)
    max_rad = math.radians(max_step_deg) if max_step_deg > 0.0 else float("inf")
    if angle > max_rad:
        R_delta = _axis_angle_to_R(axis, max_rad)
        angle = max_rad
        clamped = True

    step = np.eye(4, dtype=np.float64)
    step[:3, :3] = R_delta
    step[:3, 3] = t_delta
    T_step = T_ab2ee_now @ step
    return AlignStep(
        T_ab2ee_target=T_ab2ee_target,
        T_ab2ee_step=T_step,
        clamped=clamped,
        delta_t_norm_m=float(t_norm),
        delta_rot_deg=float(math.degrees(angle)),
    )


def is_converged(metrics: AlignMetrics,
                 position_tol_m: float,
                 angle_tol_deg: float) -> bool:
    return (metrics.xy_offset_m <= position_tol_m
            and metrics.tilt_deg <= angle_tol_deg)
