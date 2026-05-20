"""
persistence.py
==============
On-disk layout for path_tag_locator and handeye_calib results.

Locate run layout:

    <root>/locate/<YYYYMMDD>/run_<YYYYMMDD_HHMMSS>_tag<id>/
        hand_cam.png            BGR image fed to detector (or empty if failed)
        front_cam.png           BGR image fed to detector
        K_hc.npz                3x3 intrinsics used at this call
        K_fc.npz
        result.npz              T_B_world, T_A2B, T_A_world, T_hc2ee,
                                T_ab2mb, T_mb2fc, tcp_pose_mm_deg,
                                K_hc, K_fc, plus tag IDs and sizes
        result.yaml             human-readable summary
        request.yaml            full service request echo

    <root>/locate/locate_log.csv     append-only run index

Hand-eye calibration layout:

    <root>/handeye_calib/run_<YYYYMMDD_HHMMSS>/
        samples/<NNNN>_image.png
        samples/<NNNN>_pose.npz   {tcp_pose_mm_deg, K, T_ab2ee}
        samples_index.csv
        result.npz                T_hc2ee, T_ee2hc, method, residual, num_used
        result.yaml
"""
import csv
import datetime as _dt
import os
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml


# ----------------------------------------------------------------------
def _now_str():
    return _dt.datetime.now().strftime("%Y%m%d_%H%M%S")


def _expand(p):
    return Path(os.path.expandvars(os.path.expanduser(str(p))))


# ----------------------------------------------------------------------
def save_locate_run(*,
                    root_dir,
                    tag_b_id,
                    image_hc,
                    image_fc,
                    K_hc,
                    K_fc,
                    T_A_world,
                    T_hc2ee,
                    T_ab2mb,
                    T_mb2fc,
                    tcp_pose_mm_deg,
                    T_A2B,
                    T_B_world,
                    position_m,
                    rpy_deg,
                    tag_a_id,
                    tag_a_size_m,
                    tag_b_size_m,
                    family,
                    request_echo: Optional[dict] = None,
                    success: bool = True,
                    message: str = "ok",
                    auto_align_report: Optional[dict] = None) -> Path:
    """Persist a single locate call. Always called; never throws on disk
    error (caller logs)."""
    root = _expand(root_dir) / "locate"
    day = _dt.datetime.now().strftime("%Y%m%d")
    ts = _now_str()
    run_dir = root / day / f"run_{ts}_tag{tag_b_id}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # Images
    if image_hc is not None:
        cv2.imwrite(str(run_dir / "hand_cam.png"), image_hc)
    if image_fc is not None:
        cv2.imwrite(str(run_dir / "front_cam.png"), image_fc)
    if K_hc is not None:
        np.savez(str(run_dir / "K_hc.npz"), K_hc=np.asarray(K_hc, dtype=np.float64))
    if K_fc is not None:
        np.savez(str(run_dir / "K_fc.npz"), K_fc=np.asarray(K_fc, dtype=np.float64))

    # Result npz (everything needed to recompute T_A2B offline)
    np.savez(
        str(run_dir / "result.npz"),
        T_B_world=np.asarray(T_B_world, dtype=np.float64),
        T_A2B=np.asarray(T_A2B, dtype=np.float64),
        T_A_world=np.asarray(T_A_world, dtype=np.float64),
        T_hc2ee=np.asarray(T_hc2ee, dtype=np.float64),
        T_ab2mb=np.asarray(T_ab2mb, dtype=np.float64),
        T_mb2fc=np.asarray(T_mb2fc, dtype=np.float64),
        tcp_pose_mm_deg=np.asarray(tcp_pose_mm_deg, dtype=np.float64),
        K_hc=np.asarray(K_hc, dtype=np.float64),
        K_fc=np.asarray(K_fc, dtype=np.float64),
        position_m=np.asarray(position_m, dtype=np.float64),
        rpy_deg=np.asarray(rpy_deg, dtype=np.float64),
    )

    # YAML summary
    summary = {
        "timestamp": ts,
        "success": bool(success),
        "message": message,
        "tag_a_id": int(tag_a_id),
        "tag_a_size_m": float(tag_a_size_m),
        "tag_b_id": int(tag_b_id),
        "tag_b_size_m": float(tag_b_size_m),
        "apriltag_family": family,
        "tcp_pose_mm_deg": [float(v) for v in tcp_pose_mm_deg],
        "T_B_world_position_m": [float(v) for v in position_m],
        "T_B_world_rpy_deg": [float(v) for v in rpy_deg],
    }
    if auto_align_report is not None:
        summary["auto_align"] = {k: (float(v) if isinstance(v, (int, float))
                                     else v)
                                 for k, v in auto_align_report.items()}
    with open(run_dir / "result.yaml", "w") as fh:
        yaml.safe_dump(summary, fh, default_flow_style=False, sort_keys=False)

    if request_echo is not None:
        with open(run_dir / "request.yaml", "w") as fh:
            yaml.safe_dump(request_echo, fh, default_flow_style=False, sort_keys=False)

    # CSV append
    _append_locate_log(root, ts, run_dir, success, tag_b_id,
                       position_m, rpy_deg, message)
    return run_dir


def save_locate_failure(*, root_dir, tag_b_id, message,
                        request_echo: Optional[dict] = None) -> Path:
    """Persist a failed locate call (no result, just the message + request)."""
    root = _expand(root_dir) / "locate"
    day = _dt.datetime.now().strftime("%Y%m%d")
    ts = _now_str()
    run_dir = root / day / f"run_{ts}_tag{tag_b_id}_FAILED"
    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / "result.yaml", "w") as fh:
        yaml.safe_dump({
            "timestamp": ts,
            "success": False,
            "message": message,
            "tag_b_id": int(tag_b_id),
        }, fh, default_flow_style=False, sort_keys=False)
    if request_echo is not None:
        with open(run_dir / "request.yaml", "w") as fh:
            yaml.safe_dump(request_echo, fh, default_flow_style=False,
                           sort_keys=False)
    _append_locate_log(root, ts, run_dir, False, tag_b_id,
                       (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), message)
    return run_dir


def _append_locate_log(root, ts, run_dir, success, tag_b_id,
                       position_m, rpy_deg, message):
    log_path = root / "locate_log.csv"
    new = not log_path.exists()
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with open(log_path, "a", newline="") as fh:
        w = csv.writer(fh)
        if new:
            w.writerow([
                "timestamp", "success", "tag_b_id",
                "x_m", "y_m", "z_m", "rx_deg", "ry_deg", "rz_deg",
                "message", "run_dir",
            ])
        w.writerow([
            ts, int(bool(success)), int(tag_b_id),
            f"{float(position_m[0]):.6f}",
            f"{float(position_m[1]):.6f}",
            f"{float(position_m[2]):.6f}",
            f"{float(rpy_deg[0]):.4f}",
            f"{float(rpy_deg[1]):.4f}",
            f"{float(rpy_deg[2]):.4f}",
            (message or "").replace("\n", " ")[:240],
            str(run_dir),
        ])


# ----------------------------------------------------------------------
class HandeyeRunRecorder:
    """Incrementally writes calibration samples; finalize with save_result()."""

    def __init__(self, root_dir):
        root = _expand(root_dir) / "handeye_calib"
        self.run_dir = root / f"run_{_now_str()}"
        self.samples_dir = self.run_dir / "samples"
        self.samples_dir.mkdir(parents=True, exist_ok=True)
        self.index_path = self.run_dir / "samples_index.csv"
        self._n = 0
        with open(self.index_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["index", "timestamp", "tcp_x_mm", "tcp_y_mm",
                        "tcp_z_mm", "tcp_rx_deg", "tcp_ry_deg",
                        "tcp_rz_deg", "image", "pose"])

    def add_sample(self, image_bgr, K, tcp_pose_mm_deg):
        idx = self._n
        self._n += 1
        img_name = f"{idx:04d}_image.png"
        pose_name = f"{idx:04d}_pose.npz"
        cv2.imwrite(str(self.samples_dir / img_name), image_bgr)
        np.savez(str(self.samples_dir / pose_name),
                 tcp_pose_mm_deg=np.asarray(tcp_pose_mm_deg, dtype=np.float64),
                 K=np.asarray(K, dtype=np.float64))
        with open(self.index_path, "a", newline="") as fh:
            w = csv.writer(fh)
            w.writerow([
                idx, _now_str(),
                f"{tcp_pose_mm_deg[0]:.3f}",
                f"{tcp_pose_mm_deg[1]:.3f}",
                f"{tcp_pose_mm_deg[2]:.3f}",
                f"{tcp_pose_mm_deg[3]:.4f}",
                f"{tcp_pose_mm_deg[4]:.4f}",
                f"{tcp_pose_mm_deg[5]:.4f}",
                f"samples/{img_name}",
                f"samples/{pose_name}",
            ])
        return idx

    def reset(self):
        # Drop the existing samples and reopen the directory fresh.
        import shutil
        if self.run_dir.exists():
            shutil.rmtree(self.run_dir)
        self.samples_dir.mkdir(parents=True, exist_ok=True)
        self._n = 0
        with open(self.index_path, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["index", "timestamp", "tcp_x_mm", "tcp_y_mm",
                        "tcp_z_mm", "tcp_rx_deg", "tcp_ry_deg",
                        "tcp_rz_deg", "image", "pose"])

    def save_result(self, result, tag_id, tag_size_m, family) -> Path:
        np.savez(
            str(self.run_dir / "result.npz"),
            T_hc2ee=np.asarray(result.T_hc2ee, dtype=np.float64),
            T_ee2hc=np.asarray(result.T_ee2hc, dtype=np.float64),
        )
        summary = {
            "timestamp": _now_str(),
            "method": result.method,
            "residual": float(result.residual),
            "num_samples_used": int(result.num_samples_used),
            "num_samples_total": int(result.num_samples_total),
            "tag_id": int(tag_id),
            "tag_size_m": float(tag_size_m),
            "apriltag_family": family,
        }
        with open(self.run_dir / "result.yaml", "w") as fh:
            yaml.safe_dump(summary, fh, default_flow_style=False,
                           sort_keys=False)
        return self.run_dir
