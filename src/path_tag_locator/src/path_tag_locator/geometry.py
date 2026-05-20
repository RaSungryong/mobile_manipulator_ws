"""
geometry.py
===========
Rigid-body transform utilities.

Convention: T_X2Y = pose of frame Y as seen from frame X.
Lengths in meters, angles in radians (degrees only at the boundary).
"""
import math
import numpy as np


def invert_T(T: np.ndarray) -> np.ndarray:
    """Inverse of a 4x4 rigid transform: [R^T | -R^T t]."""
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=np.float64)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def apriltag_to_matrix(pose_R: np.ndarray, pose_t: np.ndarray) -> np.ndarray:
    """AprilTag detection (R 3x3, t 3x1) -> 4x4 = pose of tag in camera frame."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(pose_R, dtype=np.float64)
    T[:3, 3] = np.asarray(pose_t, dtype=np.float64).flatten()
    return T


def pose_fr5_to_matrix_m(pose_mm_deg) -> np.ndarray:
    """FR5 TCP pose [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] (ZYX intrinsic)
    -> 4x4 homogeneous transform in meters (T_ab2ee)."""
    x, y, z = [v / 1000.0 for v in pose_mm_deg[:3]]
    rx, ry, rz = [math.radians(v) for v in pose_mm_deg[3:]]

    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy_, sy = math.cos(rz), math.sin(rz)

    R = np.array([
        [cy_ * cp, cy_ * sp * sr - sy * cr, cy_ * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy_ * cr, sy * sp * cr - cy_ * sr],
        [-sp,      cp * sr,                 cp * cr],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def rot2rpy_deg(R: np.ndarray):
    """3x3 rotation -> (rx_deg, ry_deg, rz_deg), ZYX intrinsic order."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        rx = math.degrees(math.atan2(R[2, 1], R[2, 2]))
        ry = math.degrees(math.atan2(-R[2, 0], sy))
        rz = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    else:
        rx = math.degrees(math.atan2(-R[1, 2], R[1, 1]))
        ry = math.degrees(math.atan2(-R[2, 0], sy))
        rz = 0.0
    return rx, ry, rz


def rpy_deg_to_R(rx_deg: float, ry_deg: float, rz_deg: float) -> np.ndarray:
    """(rx_deg, ry_deg, rz_deg) ZYX intrinsic -> 3x3 rotation."""
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cr, sr = math.cos(rx), math.sin(rx)
    cp, sp = math.cos(ry), math.sin(ry)
    cy_, sy = math.cos(rz), math.sin(rz)
    return np.array([
        [cy_ * cp, cy_ * sp * sr - sy * cr, cy_ * sp * cr + sy * sr],
        [sy * cp,  sy * sp * sr + cy_ * cr, sy * sp * cr - cy_ * sr],
        [-sp,      cp * sr,                 cp * cr],
    ], dtype=np.float64)


def matrix_m_to_pose_fr5(T: np.ndarray) -> list:
    """4x4 (m) -> [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] (ZYX)."""
    x_mm = float(T[0, 3]) * 1000.0
    y_mm = float(T[1, 3]) * 1000.0
    z_mm = float(T[2, 3]) * 1000.0
    rx_deg, ry_deg, rz_deg = rot2rpy_deg(T[:3, :3])
    return [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]


def assert_rigid(T: np.ndarray, atol: float = 1e-6, name: str = "T") -> None:
    """Validate that T is a proper 4x4 rigid transform."""
    if T.shape != (4, 4):
        raise ValueError(f"{name}: expected shape (4,4), got {T.shape}")
    R = T[:3, :3]
    if not np.allclose(R @ R.T, np.eye(3), atol=atol):
        raise ValueError(f"{name}: R is not orthogonal (R R^T != I)")
    if not math.isclose(float(np.linalg.det(R)), 1.0, abs_tol=1e-4):
        raise ValueError(f"{name}: det(R) = {np.linalg.det(R):.6f} (expected +1)")
    if not np.allclose(T[3, :], [0, 0, 0, 1], atol=atol):
        raise ValueError(f"{name}: last row != [0,0,0,1]")


def R_to_quat_xyzw(R: np.ndarray):
    """3x3 rotation -> quaternion (x, y, z, w). Numerically stable variant."""
    R = np.asarray(R, dtype=np.float64)
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s
    return (qx, qy, qz, qw)


def quat_xyzw_to_R(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion (x, y, z, w) -> 3x3 rotation."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ], dtype=np.float64)


def pose_to_matrix(position_xyz, quat_xyzw) -> np.ndarray:
    """(position [m], quaternion xyzw) -> 4x4 m."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_xyzw_to_R(*quat_xyzw)
    T[:3, 3] = np.asarray(position_xyz, dtype=np.float64).flatten()
    return T


def matrix_to_pose(T: np.ndarray):
    """4x4 m -> (position [m] tuple, quaternion xyzw tuple)."""
    pos = (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]))
    quat = R_to_quat_xyzw(T[:3, :3])
    return pos, quat
