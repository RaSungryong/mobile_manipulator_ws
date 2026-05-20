"""
handeye_calib.py
================
Pure-numpy / OpenCV core for hand-eye calibration. No ROS imports — keep
this module unit-testable without a running master.

Given a list of samples, each holding
    - image_bgr           : HxWx3 ndarray (hand-cam capture),
    - K                   : 3x3 intrinsic matrix used at capture time,
    - tcp_pose_mm_deg     : FR5 TCP pose [x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg]
                            (= pose of EE in arm base frame),
we
    1. detect ``tag_id`` in each image (drop samples where it is missing),
    2. for each kept sample, build:
        - R_gripper2base, t_gripper2base from the TCP pose,
        - R_target2cam,   t_target2cam   from the AprilTag detection,
    3. run ``cv2.calibrateHandEye`` with all five methods and pick the
       result with the smallest AX = XB Frobenius residual,
    4. return the best 4x4 ``T_hc2ee`` (= pose of EE in hand-cam frame,
       = OpenCV's ``T_gripper2cam``).

The user-notation ``T_hc2ee`` is what the locator node loads.
"""
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from .detect import detect_apriltag
from .geometry import (
    assert_rigid,
    invert_T,
    pose_fr5_to_matrix_m,
    rot2rpy_deg,
)


_METHODS = {
    cv2.CALIB_HAND_EYE_TSAI:       "TSAI",
    cv2.CALIB_HAND_EYE_PARK:       "PARK",
    cv2.CALIB_HAND_EYE_HORAUD:     "HORAUD",
    cv2.CALIB_HAND_EYE_ANDREFF:    "ANDREFF",
    cv2.CALIB_HAND_EYE_DANIILIDIS: "DANIILIDIS",
}


@dataclass
class CalibSample:
    image_bgr: np.ndarray
    K: np.ndarray
    tcp_pose_mm_deg: list


@dataclass
class CalibResult:
    method: str
    residual: float
    T_hc2ee: np.ndarray         # 4x4 (= T_gripper2cam in OpenCV notation)
    T_ee2hc: np.ndarray         # 4x4 (= T_cam2gripper, inverse of above)
    num_samples_used: int
    num_samples_total: int


def _detect_target_in_cam(samples, tag_id, tag_size_m, family):
    """For each sample, detect tag; return aligned lists of poses and the
    indices kept."""
    R_t2c, t_t2c, kept = [], [], []
    for i, s in enumerate(samples):
        T_cam2tag = detect_apriltag(s.image_bgr, s.K, tag_size_m, tag_id, family=family)
        if T_cam2tag is None:
            continue
        R_t2c.append(T_cam2tag[:3, :3].astype(np.float64))
        t_t2c.append(T_cam2tag[:3, 3].reshape(3, 1).astype(np.float64))
        kept.append(i)
    return R_t2c, t_t2c, kept


def _gripper2base_from_pose(tcp_pose_mm_deg):
    """T_base2ee from FR5 TCP pose -> (R_gripper2base, t_gripper2base).

    The OpenCV convention's ``gripper2base`` is the *pose of the gripper in
    the base frame*, which is exactly ``pose_fr5_to_matrix_m`` (= T_ab2ee
    in user notation).
    """
    T_ab2ee = pose_fr5_to_matrix_m(tcp_pose_mm_deg)
    return T_ab2ee[:3, :3].astype(np.float64), T_ab2ee[:3, 3].reshape(3, 1).astype(np.float64)


def _axxb_residual(R_c2g, t_c2g, R_g2b, t_g2b, R_t2c, t_t2c) -> float:
    """Average Frobenius residual of AX = XB over all sample pairs."""
    n = len(R_g2b)
    X = np.eye(4)
    X[:3, :3] = R_c2g
    X[:3, 3] = np.asarray(t_c2g).flatten()
    res = []
    for i in range(n):
        Ti_g2b = np.eye(4); Ti_g2b[:3, :3] = R_g2b[i]; Ti_g2b[:3, 3] = t_g2b[i].flatten()
        Ti_t2c = np.eye(4); Ti_t2c[:3, :3] = R_t2c[i]; Ti_t2c[:3, 3] = t_t2c[i].flatten()
        for j in range(i + 1, n):
            Tj_g2b = np.eye(4); Tj_g2b[:3, :3] = R_g2b[j]; Tj_g2b[:3, 3] = t_g2b[j].flatten()
            Tj_t2c = np.eye(4); Tj_t2c[:3, :3] = R_t2c[j]; Tj_t2c[:3, 3] = t_t2c[j].flatten()
            A = invert_T(Ti_g2b) @ Tj_g2b
            B = Ti_t2c @ invert_T(Tj_t2c)
            res.append(np.linalg.norm(A @ X - X @ B, ord="fro"))
    return float(np.mean(res)) if res else float("inf")


def calibrate(samples: List[CalibSample],
              tag_id: int,
              tag_size_m: float,
              family: str = "tag36h11",
              min_samples: int = 8) -> CalibResult:
    """Run all five OpenCV methods and return the best-residual result.

    Raises ``RuntimeError`` if the tag is detected in fewer than
    ``min_samples`` images, or if every method fails.
    """
    if len(samples) < min_samples:
        raise RuntimeError(
            f"need at least {min_samples} samples, got {len(samples)}")

    R_t2c, t_t2c, kept = _detect_target_in_cam(samples, tag_id, tag_size_m, family)
    if len(kept) < min_samples:
        raise RuntimeError(
            f"tag id={tag_id} detected in only {len(kept)}/{len(samples)} "
            f"samples (need >= {min_samples})")

    R_g2b, t_g2b = [], []
    for i in kept:
        R, t = _gripper2base_from_pose(samples[i].tcp_pose_mm_deg)
        R_g2b.append(R)
        t_g2b.append(t)

    best: Optional[Tuple[float, str, np.ndarray]] = None
    for m, name in _METHODS.items():
        try:
            R_c2g, t_c2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
        except Exception:
            continue
        T_c2g = np.eye(4, dtype=np.float64)
        T_c2g[:3, :3] = R_c2g
        T_c2g[:3, 3] = np.asarray(t_c2g).flatten()
        try:
            assert_rigid(T_c2g, name=f"T_cam2EE[{name}]")
        except ValueError:
            continue
        residual = _axxb_residual(R_c2g, t_c2g, R_g2b, t_g2b, R_t2c, t_t2c)
        if best is None or residual < best[0]:
            best = (residual, name, T_c2g)

    if best is None:
        raise RuntimeError("all OpenCV hand-eye methods failed")

    residual, method_name, T_cam2EE = best
    T_EE2cam = invert_T(T_cam2EE)
    # In user notation: T_hc2ee = pose of EE in cam = T_EE2cam (OpenCV name).
    return CalibResult(
        method=method_name,
        residual=residual,
        T_hc2ee=T_EE2cam,
        T_ee2hc=T_cam2EE,
        num_samples_used=len(kept),
        num_samples_total=len(samples),
    )


def save_result(result: CalibResult, out_path) -> Path:
    """Save ``result.T_hc2ee`` to a .npz file (single key) and return path."""
    p = Path(out_path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(p), result.T_hc2ee)
    return p


def summarize(result: CalibResult) -> str:
    t = result.T_hc2ee[:3, 3]
    rpy = rot2rpy_deg(result.T_hc2ee[:3, :3])
    return (
        f"BEST method  : {result.method}\n"
        f"residual     : {result.residual:.6f}\n"
        f"samples used : {result.num_samples_used}/{result.num_samples_total}\n"
        f"T_hc2ee t [m]: x={t[0]:+.4f} y={t[1]:+.4f} z={t[2]:+.4f}\n"
        f"T_hc2ee rpy  : rx={rpy[0]:+.3f} ry={rpy[1]:+.3f} rz={rpy[2]:+.3f} deg"
    )
