"""
chain.py
========
Pure-numpy transform chain to localize a path AprilTag (B) in the reference
tag (A) frame, then in the world frame.

Chain (user notation T_X2Y = pose of Y in X):

    T_A2B = T_A2hc * T_hc2ee * T_ee2ab * T_ab2mb * T_mb2fc * T_fc2B

where
    - T_A2hc    : from hand-cam image (detect tag A) -> invert
    - T_hc2ee   : hand-eye calibration (loaded npz)
    - T_ee2ab   : invert(pose_fr5_to_matrix_m(tcp_pose))
    - T_ab2mb   : platform constant (arm_base -> mobile_base)
    - T_mb2fc   : platform constant (mobile_base -> front_cam)
    - T_fc2B    : from front-cam image (detect tag B)

With a user-supplied accurate T_A_world,
    T_B_world = T_A_world @ T_A2B
"""
import numpy as np

from .detect import detect_apriltag
from .geometry import (
    invert_T,
    pose_fr5_to_matrix_m,
    rot2rpy_deg,
)


def compute_T_A2B(*,
                  image_hc: np.ndarray,
                  image_fc: np.ndarray,
                  tcp_pose_mm_deg,
                  T_hc2ee: np.ndarray,
                  K_hc: np.ndarray,
                  K_fc: np.ndarray,
                  T_ab2mb: np.ndarray,
                  T_mb2fc: np.ndarray,
                  tag_a_id: int,
                  tag_b_id: int,
                  tag_a_size_m: float,
                  tag_b_size_m: float,
                  family: str = "tag36h11") -> dict:
    """Compute T_A2B and return intermediates."""
    T_hc2A = detect_apriltag(image_hc, K_hc, tag_a_size_m, tag_a_id, family=family)
    if T_hc2A is None:
        raise RuntimeError(f"Tag A (id={tag_a_id}) not detected in hand-cam image")
    T_A2hc = invert_T(T_hc2A)

    T_ab2ee = pose_fr5_to_matrix_m(tcp_pose_mm_deg)
    T_ee2ab = invert_T(T_ab2ee)

    T_fc2B = detect_apriltag(image_fc, K_fc, tag_b_size_m, tag_b_id, family=family)
    if T_fc2B is None:
        raise RuntimeError(f"Tag B (id={tag_b_id}) not detected in front-cam image")

    T_A2B = T_A2hc @ T_hc2ee @ T_ee2ab @ T_ab2mb @ T_mb2fc @ T_fc2B

    pos = T_A2B[:3, 3]
    rpy = rot2rpy_deg(T_A2B[:3, :3])

    return {
        "T_A2B": T_A2B,
        "position_m": (float(pos[0]), float(pos[1]), float(pos[2])),
        "rpy_deg": rpy,
        "intermediates": {
            "T_A2hc": T_A2hc,
            "T_hc2ee": T_hc2ee,
            "T_ee2ab": T_ee2ab,
            "T_ab2mb": T_ab2mb,
            "T_mb2fc": T_mb2fc,
            "T_fc2B": T_fc2B,
        },
    }


def compute_T_B_world(T_A_world: np.ndarray, T_A2B: np.ndarray) -> np.ndarray:
    """T_B_world = T_A_world @ T_A2B."""
    return T_A_world @ T_A2B
