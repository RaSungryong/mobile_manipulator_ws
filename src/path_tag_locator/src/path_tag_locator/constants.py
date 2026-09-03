"""
constants.py
============
Configuration loaders.

Reads ROS-style nested yaml/params and produces typed Python objects. Pure
helpers — no ROS imports here, so the loaders work in unit-test scripts too.
"""
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np
import yaml

from .geometry import assert_rigid, rpy_deg_to_R


@dataclass
class TopicsCfg:
    # Shared-detector outputs (robot_camera_node) — the tag observations.
    hand_cam_detections: str
    front_cam_detections: str
    # Raw image/info topics. Images are used ONLY for best-effort record
    # snapshots in the persistence layer (empty string disables); the
    # info topics serve the standalone verification / hand-eye tools,
    # which re-detect over raw frames on purpose.
    hand_cam_image: str = ""
    front_cam_image: str = ""
    hand_cam_info: str = ""
    front_cam_info: str = ""


@dataclass
class TagCfg:
    family: str
    tag_a_id: int
    tag_b_id: int
    tag_a_size_m: float
    tag_b_size_m: float


@dataclass
class DetectorCfg:
    """Tag sizes the SHARED detector was configured with (robot.yaml
    ``robot_camera.tag_size``). pose_t scales linearly with tag size, so
    observations are rescaled from these to the actual tag sizes above."""
    hand_cam_tag_size_m: float
    front_cam_tag_size_m: float


@dataclass
class ArmCfg:
    """arm_node proxy settings (replaces the old direct-SDK robot: block)."""
    state_topic: str = "/arm/state"
    move_cart_topic: str = "/arm/move_cart"
    home_service: str = "/arm/move_home"
    motion_timeout_s: float = 60.0
    # Home the arm BEFORE every base move in a calibration session, so
    # the base never drives/pivots with the arm extended at a view pose.
    home_before_nav: bool = True


@dataclass
class IOCfg:
    detection_wait_timeout: float
    default_save_dir: str


@dataclass
class AlignCfg:
    target_distance_m: float
    max_iterations: int
    position_tol_m: float
    angle_tol_deg: float
    max_step_m: float
    max_step_deg: float
    max_initial_step_m: float
    max_initial_step_deg: float
    move_vel: float
    move_acc: float
    move_ovl: float
    move_settle_s: float
    # Auto-view-pose bootstrap (used by map_calibrator only, ignored by
    # the single-tag locator). When ``auto_view_pose`` is true and the
    # orchestrator has at least one previous successful entry, the next
    # entry's ``arm_view_tcp_mm_deg`` is auto-computed from the previous
    # anchor + map.yaml relative offsets. Explicit per-entry override in
    # calibration_plan.yaml always wins.
    auto_view_pose: bool = True
    auto_view_distance_m: float = 0.20
    # When an align arm move fails (IK/reach/timeout) but the ref tag is
    # still observable, keep the last reachable pose and let the chain
    # run (result marked "degraded") instead of failing the entry.
    continue_on_move_failure: bool = True
    # Detections collected per align iteration; the median-tilt frame is
    # used. 1 = trust a single frame (pre-2026-09-02 behaviour).
    samples_per_iteration: int = 5


@dataclass
class LocatorCfg:
    topics: TopicsCfg
    tag: TagCfg
    detector: DetectorCfg
    arm: ArmCfg
    hand_eye_npz: str
    extrinsics_yaml: str
    reference_tag_yaml: str
    io: IOCfg
    align: AlignCfg


def _expand(p: str) -> str:
    return os.path.expandvars(os.path.expanduser(p))


def load_locator_cfg_from_dict(d: dict) -> LocatorCfg:
    """Parse a dict with the same shape as ``config/locator.yaml`` (under
    the ``path_tag_locator`` key) into a :class:`LocatorCfg`.

    Accepts both the wrapped form (``{path_tag_locator: {...}}``) and the
    inner form ``{...}``.
    """
    root = d.get("path_tag_locator", d)
    topics = TopicsCfg(**root["topics"])
    tag = TagCfg(**root["tag"])
    detector = DetectorCfg(**root["detector"])
    arm = ArmCfg(**root.get("arm", {}))
    io = IOCfg(**root["io"])
    align_defaults = dict(
        target_distance_m=0.0,
        max_iterations=5,
        position_tol_m=0.005,
        angle_tol_deg=1.0,
        max_step_m=0.10,
        max_step_deg=15.0,
        max_initial_step_m=0.80,
        max_initial_step_deg=180.0,
        move_vel=20.0,
        move_acc=20.0,
        move_ovl=100.0,
        move_settle_s=0.3,
        auto_view_pose=True,
        auto_view_distance_m=0.20,
        continue_on_move_failure=True,
        samples_per_iteration=5,
    )
    align_defaults.update(root.get("align", {}))
    align = AlignCfg(**align_defaults)
    return LocatorCfg(
        topics=topics,
        tag=tag,
        detector=detector,
        arm=arm,
        hand_eye_npz=_expand(root["hand_eye"]["npz_path"]),
        extrinsics_yaml=_expand(root["extrinsics_yaml"]),
        reference_tag_yaml=_expand(root["reference_tag_yaml"]),
        io=io,
        align=align,
    )


def load_locator_cfg(yaml_path) -> LocatorCfg:
    with open(yaml_path, "r") as fh:
        d = yaml.safe_load(fh)
    return load_locator_cfg_from_dict(d)


def load_extrinsics(yaml_path):
    """Load T_AB2MB and T_MB2FC (both 4x4) from yaml row-major lists."""
    with open(yaml_path, "r") as fh:
        d = yaml.safe_load(fh)
    T_ab2mb = np.asarray(d["T_ab2mb_row_major"], dtype=np.float64).reshape(4, 4)
    T_mb2fc = np.asarray(d["T_mb2fc_row_major"], dtype=np.float64).reshape(4, 4)
    assert_rigid(T_ab2mb, name="T_ab2mb")
    assert_rigid(T_mb2fc, name="T_mb2fc")
    return T_ab2mb, T_mb2fc


def load_reference_tag(yaml_path) -> np.ndarray:
    """Parse reference_tag.yaml and return T_A_world (4x4)."""
    with open(yaml_path, "r") as fh:
        d = yaml.safe_load(fh)
    ref = d["reference_tag"]
    fmt = ref.get("format", "pose")
    if fmt == "matrix":
        T = np.asarray(ref["matrix_4x4"], dtype=np.float64).reshape(4, 4)
    elif fmt == "pose":
        pos = np.asarray(ref["position_m"], dtype=np.float64)
        rpy = ref["rpy_deg"]
        R = rpy_deg_to_R(float(rpy[0]), float(rpy[1]), float(rpy[2]))
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = pos
    else:
        raise ValueError(f"unknown reference_tag.format: {fmt!r}")
    assert_rigid(T, name="T_A_world")
    return T
