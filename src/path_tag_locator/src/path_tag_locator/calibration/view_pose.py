"""
view_pose.py
============
Bootstrap helpers for auto-computing ``arm_view_tcp_mm_deg``.

Idea:
- After each successful chain, we can solve for the base's world-frame
  pose at that moment: ``T_world2mb = T_B_world · inv(T_mb2fc · T_fc2B)``.
- For the NEXT entry the base will park near a different path tag.
  Assuming map.yaml is approximately rigid (its relative geometry is
  cm-level accurate even if its absolute frame is offset/rotated from
  the user world frame), the inter-entry base translation in world is
  approximately the inter-entry path-tag translation in map.yaml.
- The base's yaw change between entries is observable directly via
  /odom (``RobotController.current_theta``).
- Putting these together, we can estimate ``T_world2mb`` for the upcoming
  entry and from that derive a sensible ``arm_view_tcp_mm_deg`` that
  puts the hand camera squarely above the chosen ref tag, ready for
  ``run_auto_align`` to refine.

The estimate is intentionally rough (cm-to-decimeter accuracy). The
align step has cm-level convergence tolerance and a >50 cm initial-step
clamp, so this is plenty for a seed pose.
"""
import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from ..geometry import (
    invert_T,
    matrix_m_to_pose_fr5,
    rpy_deg_to_R,
)


@dataclass
class BaseAnchor:
    """Snapshot of base in world frame, taken at the moment a chain
    completed. Used to propagate to the next entry."""
    T_world2mb: np.ndarray            # 4x4: pose of mb in world at this anchor
    odom_theta: float                  # /odom yaw at this anchor (radians)
    path_tag_id: int                   # which path tag's location this anchor was taken near
    path_map_xy: Sequence[float]       # the path tag's (x, y) in map.yaml at this anchor


# ----------------------------------------------------------------------
def compute_T_world2mb_from_chain(T_B_world: np.ndarray,
                                  T_fc2B: np.ndarray,
                                  T_mb2fc: np.ndarray) -> np.ndarray:
    """Reverse-engineer mb's world pose using the chain outputs.

    Derivation (T_X2Y = pose of Y in X):
        T_B_world = T_world2mb · T_mb2fc · T_fc2B
        =>  T_world2mb = T_B_world · inv(T_mb2fc · T_fc2B)
    """
    bridge = T_mb2fc @ T_fc2B
    return T_B_world @ invert_T(bridge)


def propagate_world_mb(anchor: BaseAnchor,
                       path_xy_map_now: Sequence[float],
                       odom_theta_now: float) -> np.ndarray:
    """Estimate ``T_world2mb`` for the current entry from the previous
    anchor + Δmap (path-tag relative position in map.yaml) + Δodom_theta.

    Assumes the map↔world transform is approximately a pure horizontal
    translation. The residual yaw mismatch is absorbed by run_auto_align.
    """
    # Δmap is in 2D (xy). We treat z as constant (floor).
    dp = np.array([
        float(path_xy_map_now[0]) - float(anchor.path_map_xy[0]),
        float(path_xy_map_now[1]) - float(anchor.path_map_xy[1]),
        0.0,
    ], dtype=np.float64)

    dtheta = float(odom_theta_now) - float(anchor.odom_theta)
    # Wrap to [-pi, pi] so a 359° → 1° crossing doesn't blow up.
    dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))

    R_delta = np.array([
        [math.cos(dtheta), -math.sin(dtheta), 0.0],
        [math.sin(dtheta),  math.cos(dtheta), 0.0],
        [0.0,               0.0,              1.0],
    ], dtype=np.float64)

    T_new = np.eye(4, dtype=np.float64)
    # World-frame rotation delta is pre-multiplied: R_new = Rz(Δθ) · R_old.
    T_new[:3, :3] = R_delta @ anchor.T_world2mb[:3, :3]
    T_new[:3, 3] = anchor.T_world2mb[:3, 3] + dp
    return T_new


# ----------------------------------------------------------------------
def _target_T_A2hc(view_distance_m: float) -> np.ndarray:
    """Desired pose of hand-cam in ref-tag-A's frame: squarely above the
    tag at the given height.

    Convention (matches detect.py / align.py): when squarely aligned,
    ``T_cam2tag = [[I | (0, 0, d)]; [0 0 0 1]]``. The inverse is the cam
    pose in the tag frame, ``[[I | (0, 0, -d)]; [0 0 0 1]]``, i.e. cam
    sits at -d along the tag's z-axis. For a floor tag (z up), that
    places the cam at +d above the floor — the intended geometry.
    """
    T = np.eye(4, dtype=np.float64)
    T[2, 3] = -float(view_distance_m)
    return T


def compute_view_tcp(T_A_world: np.ndarray,
                     T_world2mb: np.ndarray,
                     T_ab2mb: np.ndarray,
                     T_hc2ee: np.ndarray,
                     view_distance_m: float) -> list:
    """Compute the FR5 TCP pose ([x_mm, y_mm, z_mm, rx, ry, rz] deg, ZYX
    intrinsic) that places hand-cam ``view_distance_m`` above ref tag A,
    squarely facing it, assuming the base is at ``T_world2mb``.

    Chain (T_X2Y = pose of Y in X):
        T_world2hc_desired = T_A_world · T_A2hc_desired
        T_world2ab         = T_world2mb · inv(T_ab2mb)
        T_ab2hc            = inv(T_world2ab) · T_world2hc_desired
        T_ab2ee            = T_ab2hc · T_hc2ee
        view_tcp           = matrix_m_to_pose_fr5(T_ab2ee)
    """
    T_A2hc_desired = _target_T_A2hc(view_distance_m)
    T_world2hc_desired = T_A_world @ T_A2hc_desired
    T_world2ab = T_world2mb @ invert_T(T_ab2mb)
    T_ab2hc = invert_T(T_world2ab) @ T_world2hc_desired
    T_ab2ee = T_ab2hc @ T_hc2ee
    return matrix_m_to_pose_fr5(T_ab2ee)
