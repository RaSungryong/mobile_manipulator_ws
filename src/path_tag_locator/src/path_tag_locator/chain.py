"""
chain.py
========
Pure-numpy transform chain to localize a path AprilTag (B) in the reference
tag (A) frame, then in the world frame.

Chain (user notation T_X2Y = pose of Y in X):

    T_A2B = T_A2hc * T_hc2ee * T_ee2ab * T_ab2mb * T_mb2fc * T_fc2B

where
    - T_A2hc    : invert(T_hc2A), hand-cam observation of tag A
    - T_hc2ee   : hand-eye calibration (loaded npz)
    - T_ee2ab   : invert(pose_fr5_to_matrix_m(tcp_pose))
    - T_ab2mb   : platform constant (arm_base -> mobile_base)
    - T_mb2fc   : platform constant (mobile_base -> front_cam)
    - T_fc2B    : front-cam observation of tag B

With a user-supplied accurate T_A_world,
    T_B_world = T_A_world @ T_A2B

The tag observations arrive as ready-made 4x4 ``T_cam2tag`` matrices
(from the shared detector via ``detections.detection_to_T_cam2tag``);
this module no longer runs a detector of its own.
"""
import numpy as np

from .geometry import (
    invert_T,
    pose_fr5_to_matrix_m,
    rot2rpy_deg,
)


def compensate_T_ab2mb(T_ab2mb: np.ndarray,
                       lift_height_m: float) -> np.ndarray:
    """Shift T_ab2mb for the live lift extension.

    extrinsics.yaml measures T_ab2mb with the lift AT ORIGIN. Raising
    the lift by h lifts the arm base h above the mobile-base origin, so
    mb sits h FURTHER below ab along ab.z: t_z goes -0.652 -> -(0.652+h).
    (ab.z == mb.z == up; R is exactly Rz(180°), so only t_z moves.)
    """
    if not lift_height_m:
        return T_ab2mb
    T = T_ab2mb.copy()
    T[2, 3] -= float(lift_height_m)
    return T


def compute_T_A2B(*,
                  T_hc2A: np.ndarray,
                  T_fc2B: np.ndarray,
                  tcp_pose_mm_deg,
                  T_hc2ee: np.ndarray,
                  T_ab2mb: np.ndarray,
                  T_mb2fc: np.ndarray,
                  lift_height_m: float = 0.0) -> dict:
    """Compute T_A2B from the two camera observations and return
    intermediates. ``lift_height_m`` (live, from /lifter/height) shifts
    T_ab2mb per ``compensate_T_ab2mb``; 0.0 keeps the lift-at-origin
    matrix unchanged."""
    if T_hc2A is None:
        raise RuntimeError("T_hc2A is None (tag A not observed)")
    if T_fc2B is None:
        raise RuntimeError("T_fc2B is None (tag B not observed)")

    T_A2hc = invert_T(T_hc2A)

    T_ab2ee = pose_fr5_to_matrix_m(tcp_pose_mm_deg)
    T_ee2ab = invert_T(T_ab2ee)

    T_ab2mb = compensate_T_ab2mb(T_ab2mb, lift_height_m)

    T_A2B = T_A2hc @ T_hc2ee @ T_ee2ab @ T_ab2mb @ T_mb2fc @ T_fc2B

    pos = T_A2B[:3, 3]
    rpy = rot2rpy_deg(T_A2B[:3, :3])

    return {
        "T_A2B": T_A2B,
        "position_m": (float(pos[0]), float(pos[1]), float(pos[2])),
        "rpy_deg": rpy,
        "lift_height_m": float(lift_height_m),
        "intermediates": {
            "T_A2hc": T_A2hc,
            "T_hc2ee": T_hc2ee,
            "T_ee2ab": T_ee2ab,
            "T_ab2mb": T_ab2mb,   # lift-compensated, as used in the product
            "T_mb2fc": T_mb2fc,
            "T_fc2B": T_fc2B,
        },
    }


def compute_T_B_world(T_A_world: np.ndarray, T_A2B: np.ndarray) -> np.ndarray:
    """T_B_world = T_A_world @ T_A2B."""
    return T_A_world @ T_A2B
