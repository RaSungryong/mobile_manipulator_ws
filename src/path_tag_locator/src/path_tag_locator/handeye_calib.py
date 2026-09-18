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
    4. (2026-09-18) REFINE it: the tag is one fixed object, so the pose
       ``T_ab2ee_i · inv(T_hc2ee) · T_cam2tag_i`` must be the same for
       every sample — minimise that scatter (position + normal) over the
       six parameters of T_hc2ee with the OpenCV result as the start,
       and keep the refinement only if it lowers the scatter. This is
       the number the locator chain actually depends on, and the
       closed-form methods do not minimise it: on the 2026-09-18 session
       (32 samples, 14 of them small +/-10 deg bootstrap rotations, 7 at
       1.2 m) ANDREFF's answer scattered the tag 11.1 mm rms while the
       refined one sits 5.3 mm rms over the same 32 and 2.3 mm over the
       18 sweep samples — 27 mm / 1.4 deg apart. Requires a FIXED tag
       (the base still) for the whole sample set.
    5. return the 4x4 ``T_hc2ee`` (= pose of EE in hand-cam frame,
       = OpenCV's ``T_gripper2cam``), with the tag scatter before / after.

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
    # tag-scatter refinement (2026-09-18): rms / max distance of the
    # re-projected tag centre from its mean over the samples, and the
    # rms angle of its normal, BEFORE (OpenCV) and AFTER the refinement
    scatter_rms_m: float = float('nan')
    scatter_max_m: float = float('nan')
    scatter_normal_rms_deg: float = float('nan')
    opencv_scatter_rms_m: float = float('nan')
    opencv_scatter_max_m: float = float('nan')
    refined: bool = False
    T_hc2ee_opencv: Optional[np.ndarray] = None


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


def tag_scatter(T_hc2ee, T_g2b_list, T_t2c_list):
    """How far the fixed tag's re-projected pose scatters over the
    samples with this hand-eye: (rms_m, max_m, normal_rms_deg)."""
    T_ee2hc = invert_T(np.asarray(T_hc2ee, dtype=np.float64))
    P, N = [], []
    for T_g2b, T_t2c in zip(T_g2b_list, T_t2c_list):
        T = T_g2b @ T_ee2hc @ T_t2c
        P.append(T[:3, 3]); N.append(T[:3, 2])
    P = np.array(P); N = np.array(N)
    d = np.linalg.norm(P - P.mean(axis=0), axis=1)
    nm = N.mean(axis=0); nm /= max(np.linalg.norm(nm), 1e-12)
    ang = np.degrees(np.arccos(np.clip(N @ nm, -1.0, 1.0)))
    return (float(np.sqrt(np.mean(d ** 2))), float(d.max()),
            float(np.sqrt(np.mean(ang ** 2))))


def refine_hand_eye(T_hc2ee0, T_g2b_list, T_t2c_list, normal_weight_m_per_rad=0.05):
    """Minimise the tag scatter over the 6 parameters of T_hc2ee
    (rotation vector left-multiplied onto the start rotation, plus a
    translation delta). The normal term is weighted as a lever of
    ``normal_weight_m_per_rad`` (0.05 m per radian). Returns the refined
    4x4; scipy's least_squares (Levenberg-Marquardt-like trust region)."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation as Rot
    T0 = np.asarray(T_hc2ee0, dtype=np.float64)
    R0, t0 = T0[:3, :3], T0[:3, 3]

    def unpack(p):
        rot = Rot.from_rotvec(p[:3])
        Rd = rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()
        T = np.eye(4); T[:3, :3] = Rd @ R0; T[:3, 3] = t0 + p[3:]
        return T

    def resid(p):
        T_ee2hc = invert_T(unpack(p))
        P, N = [], []
        for T_g2b, T_t2c in zip(T_g2b_list, T_t2c_list):
            T = T_g2b @ T_ee2hc @ T_t2c
            P.append(T[:3, 3]); N.append(T[:3, 2])
        P = np.array(P); N = np.array(N)
        return np.concatenate([(P - P.mean(axis=0)).ravel(),
                               normal_weight_m_per_rad * (N - N.mean(axis=0)).ravel()])

    r = least_squares(resid, np.zeros(6), xtol=1e-12, ftol=1e-12, max_nfev=2000)
    return unpack(r.x)


def calibrate(samples: List[CalibSample],
              tag_id: int,
              tag_size_m: float,
              family: str = "tag36h11",
              min_samples: int = 8,
              refine: bool = True) -> CalibResult:
    """Run all five OpenCV methods, take the best-residual result, and
    (``refine``) minimise the fixed tag's re-projection scatter from it.

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
    T_g2b_list = []
    for R, t in zip(R_g2b, t_g2b):
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t).flatten(); T_g2b_list.append(T)
    T_t2c_list = []
    for R, t in zip(R_t2c, t_t2c):
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(t).flatten(); T_t2c_list.append(T)
    sc0 = tag_scatter(T_EE2cam, T_g2b_list, T_t2c_list)
    result = CalibResult(
        method=method_name,
        residual=residual,
        T_hc2ee=T_EE2cam,
        T_ee2hc=T_cam2EE,
        num_samples_used=len(kept),
        num_samples_total=len(samples),
        scatter_rms_m=sc0[0], scatter_max_m=sc0[1], scatter_normal_rms_deg=sc0[2],
        opencv_scatter_rms_m=sc0[0], opencv_scatter_max_m=sc0[1],
        T_hc2ee_opencv=T_EE2cam.copy(),
    )
    if refine:
        try:
            T_ref = refine_hand_eye(T_EE2cam, T_g2b_list, T_t2c_list)
            assert_rigid(T_ref, name="T_hc2ee[refined]")
            sc1 = tag_scatter(T_ref, T_g2b_list, T_t2c_list)
        except Exception:
            sc1 = None
        if sc1 is not None and sc1[0] < sc0[0]:
            result.T_hc2ee = T_ref
            result.T_ee2hc = invert_T(T_ref)
            result.scatter_rms_m, result.scatter_max_m, result.scatter_normal_rms_deg = sc1
            result.refined = True
            result.method = f"{method_name}+refined"
    return result


def save_result(result: CalibResult, out_path) -> Path:
    """Save ``result.T_hc2ee`` to a .npz file (single key) and return path."""
    p = Path(out_path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    np.savez(str(p), result.T_hc2ee)
    return p


def summarize(result: CalibResult) -> str:
    t = result.T_hc2ee[:3, 3]
    rpy = rot2rpy_deg(result.T_hc2ee[:3, :3])
    lines = [
        f"BEST method  : {result.method}",
        f"residual     : {result.residual:.6f}",
        f"samples used : {result.num_samples_used}/{result.num_samples_total}",
        f"T_hc2ee t [m]: x={t[0]:+.4f} y={t[1]:+.4f} z={t[2]:+.4f}",
        f"T_hc2ee rpy  : rx={rpy[0]:+.3f} ry={rpy[1]:+.3f} rz={rpy[2]:+.3f} deg",
    ]
    if np.isfinite(result.scatter_rms_m):
        lines.append(f"tag scatter  : {result.scatter_rms_m * 1000:.1f} mm rms / "
                     f"{result.scatter_max_m * 1000:.1f} mm max, normal "
                     f"{result.scatter_normal_rms_deg:.2f} deg rms"
                     + (f" (OpenCV {result.opencv_scatter_rms_m * 1000:.1f} / "
                        f"{result.opencv_scatter_max_m * 1000:.1f} mm before refinement)"
                        if result.refined else " (refinement did not improve it)"))
    return "\n".join(lines)
