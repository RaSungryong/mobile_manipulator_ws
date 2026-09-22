"""
constants.py
============
Configuration loaders.

Reads ROS-style nested yaml/params and produces typed Python objects. Pure
helpers — no ROS imports here, so the loaders work in unit-test scripts too.
"""
import math
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
    # Solve front_cam's tag pose from the published CORNERS instead of its
    # pose_x/y/z + rpy fields. Required whenever robot.yaml
    # robot_camera.ground_plane.front_cam is enabled: those fields are then
    # ground coordinates + a CONSTANT depth + an UNCORRECTED rotation, which
    # is right for navigation and is not a 6-DOF measurement. See
    # detections.py's module docstring. Harmless when the correction is off
    # (the re-solve matches dt_apriltags' own estimate to ~0.2 deg).
    front_cam_repose_from_corners: bool = True
    # Which front_cam frame the published detections are expressed in,
    # i.e. which T_mb2fc the chain must use (2026-09-15, tf_chain.yaml
    # now carries the PHYSICAL, tilted camera):
    #   "auto"     — read robot.yaml robot_camera.ground_plane.front_cam
    #                .enabled (the same file robot_camera_node reads):
    #                enabled -> "level", off -> "physical"
    #   "level"    — detections are the level virtual camera's (correction
    #                on): chain uses T_mb2fc_level = T_mb2fc @ tilt
    #   "physical" — raw detections (correction off, or a consumer that
    #                re-detects raw frames): chain uses T_mb2fc as stored
    # Getting this wrong applies the 1.3 deg tilt twice or not at all —
    # ~7 mm of tag position at the 0.3 m lens height, 1.3 deg of yaw.
    front_cam_frame: str = "auto"


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
    # The initial view move is chunked: up to this many clamped steps of
    # max_initial_step_m each, so a seed farther than one step from the
    # start pose is still REACHED instead of stopped short. 1 = the old
    # single-clamped-step behaviour (which, with the arm homing before
    # every base move since 2026-09-03, left every seed > 0.8 m from the
    # home TCP out of the camera's view — 6/26 entries on 2026-09-04).
    max_initial_steps: int = 4
    # Retry seed fallback (map_calibrator): when the first attempt failed
    # because the ref tag was not seen from the seed, a retry may raise
    # the camera by this much (m) to widen the field of view. Used only
    # when no session correction / anchor estimate is available.
    retry_raise_m: float = 0.25
    # Orientation policy of the align loop (2026-09-22, user rule for map
    # calibration): 'correct' = square the optical axis to the tag every
    # step (tilt -> 0, spin kept; the pre-09-22 behaviour, still what the
    # hand-eye sweep's square-up does through its own cfg); 'fixed' = the
    # seed orientation (the plan's design view TCP: camera parallel to
    # the tag through the chain, rz free) is never commanded — the loop
    # moves in translation only, x/y in the image plane and z along the
    # optical axis to target_distance_m. Convergence is then xy (+ depth
    # within depth_tol_m); the measured tilt is recorded and, above
    # tilt_warn_deg, warned about, never corrected.
    orientation: str = "correct"
    depth_tol_m: float = 0.005
    tilt_warn_deg: float = 3.0
    # 'fixed' only: rx / ry the align STEPS command (rz = the seed's,
    # free). None = keep the seed's own rx / ry. User rule 2026-09-22:
    # -180 / 0 = tool straight down the arm z. The approach to the seed
    # is not held to it.
    fixed_rx_deg: float = None
    fixed_ry_deg: float = None

    def __post_init__(self):
        self.orientation = str(self.orientation).lower()
        if self.orientation not in ("correct", "fixed"):
            raise ValueError(
                "align.orientation must be 'correct' or 'fixed', got %r"
                % (self.orientation,))
        if (self.fixed_rx_deg is None) != (self.fixed_ry_deg is None):
            raise ValueError(
                "align.fixed_rx_deg / fixed_ry_deg must be set together "
                "(both, or both null)")


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
        max_initial_steps=4,
        retry_raise_m=0.25,
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


def _tf_block_matrix(d, name, yaml_path):
    """A 4x4 from a tf_chain.yaml block ``name: {matrix: [16 numbers]}``."""
    entry = d.get(name)
    if not isinstance(entry, dict) or "matrix" not in entry:
        raise ValueError("%s: no `%s: {matrix: ...}` block (this must be "
                         "apriltag_nav/config/tf/tf_chain.yaml since 2026-09-21)"
                         % (yaml_path, name))
    m = np.asarray(entry["matrix"], dtype=np.float64)
    if m.size != 16:
        raise ValueError("%s: %s.matrix must hold 16 numbers" % (yaml_path, name))
    return m.reshape(4, 4)


def load_extrinsics(yaml_path=None):
    """Load T_AB2MB and T_MB2FC (both 4x4) from tf_chain.yaml
    (apriltag_nav/config/tf; default = that file).

    T_MB2FC is the PHYSICAL front_cam optical frame as stored — since
    2026-09-15 that includes the measured 1.3 deg tilt. A consumer of
    robot_camera_node's ground-plane-corrected detections must NOT use it
    directly: see :func:`load_extrinsics_full` and ``Extrinsics.T_mb2fc_chain``.
    """
    if yaml_path is None:
        from apriltag_nav.tf_chain import TF_CHAIN_PATH  # exec_depend
        yaml_path = TF_CHAIN_PATH
    with open(yaml_path, "r") as fh:
        d = yaml.safe_load(fh) or {}
    T_ab2mb = _tf_block_matrix(d, "T_ab2mb", yaml_path)
    T_mb2fc = _tf_block_matrix(d, "T_mb2fc", yaml_path)
    assert_rigid(T_ab2mb, name="T_ab2mb")
    assert_rigid(T_mb2fc, name="T_mb2fc")
    return T_ab2mb, T_mb2fc


# Rotation of the level virtual front_cam in the mobile-base frame: x = image
# right = forward, y = image down = robot right, z = optical axis = down.
# This is what T_mb2fc's rotation WAS before the tilt went into it, and what
# T_mb2fc_level's rotation must come out as.
R_MB2FC_LEVEL = np.diag([1.0, -1.0, -1.0])

# How far T_mb2fc @ tilt may deviate from R_MB2FC_LEVEL before the stored
# matrix and robot.yaml's ground-plane fit are declared inconsistent.
_LEVEL_TOL_RAD = math.radians(0.01)


def load_front_cam_ground_plane(robot_yaml_path=None):
    """robot.yaml ``robot_camera.ground_plane.front_cam`` as a dict, or None
    when the block is absent. Defaults to the same file robot_camera_node
    reads (apriltag_nav.paths.CONFIG_PATH), so "what frame are the
    detections in" is answered by the publisher's own config. The dict
    also carries ``tag_thickness_m`` (robot.tag_thickness, 0 if absent):
    ``height_m`` is the lens height above the TAG-TOP plane, and the lens
    sits height_m + tag_thickness above the floor / mb origin."""
    if robot_yaml_path is None:
        from apriltag_nav.paths import CONFIG_PATH  # exec_depend
        robot_yaml_path = CONFIG_PATH
    with open(robot_yaml_path, "r") as fh:
        d = yaml.safe_load(fh) or {}
    gp = ((d.get("robot_camera") or {}).get("ground_plane") or {}).get("front_cam")
    if not gp:
        return None
    out = dict(gp)
    out["tag_thickness_m"] = float((d.get("robot") or {}).get("tag_thickness", 0.0) or 0.0)
    return out


@dataclass
class Extrinsics:
    """Platform extrinsics plus the two front_cam frames (2026-09-15).

    T_mb2fc        the PHYSICAL front_cam optical frame (tilted), as stored
                   in tf_chain.yaml — right for anything that re-detects
                   RAW frames (verify_arm_pointing, hand-eye tools).
    T_mb2fc_level  the LEVEL virtual camera robot_camera_node re-images
                   detections into (= T_mb2fc @ T_tilted_to_level(fit)) —
                   right for a chain fed from /front_cam/tag_detections
                   while the ground-plane correction is on. Rotation is
                   exactly R_MB2FC_LEVEL; translation is the same lens
                   centre.
    T_mb2fc_chain  whichever of the two matches the detections the chain
                   consumes (``front_cam_frame`` after "auto" resolution).
    """
    T_ab2mb: np.ndarray
    T_mb2fc: np.ndarray
    T_mb2fc_level: np.ndarray
    T_mb2fc_chain: np.ndarray
    front_cam_frame: str          # "level" | "physical" (resolved)
    ground_plane: Optional[dict]  # robot.yaml block used for the derivation
    note: str = ""

    def as_tuple(self):
        return self.T_ab2mb, self.T_mb2fc_chain


def load_extrinsics_full(yaml_path=None, front_cam_frame="auto",
                         ground_plane="auto", robot_yaml_path=None):
    """Load tf_chain.yaml (T_ab2mb, T_mb2fc) and derive the level front_cam frame.

    ``ground_plane``: "auto" reads robot.yaml (see
    :func:`load_front_cam_ground_plane`), a dict is used as given, None
    means "no correction exists" (level == physical is then only true if
    the stored matrix is level — checked).
    ``front_cam_frame``: "auto" | "level" | "physical" (DetectorCfg).

    Consistency is enforced, not assumed: the stored T_mb2fc must equal
    R_MB2FC_LEVEL @ inv(tilt) to 0.01 deg and its tz must equal the fit's
    ``height_m`` + ``tag_thickness_m`` — otherwise the yaml was hand-edited out of step with
    robot.yaml and the caller gets a ValueError naming
    apriltag_nav/tools/tf_chain_tool.py front-cam.
    """
    from apriltag_nav.ground_plane import T_tilted_to_level  # exec_depend

    T_ab2mb, T_mb2fc = load_extrinsics(yaml_path)
    if isinstance(ground_plane, str) and ground_plane == "auto":
        ground_plane = load_front_cam_ground_plane(robot_yaml_path)

    if ground_plane:
        tilt = T_tilted_to_level(math.radians(float(ground_plane.get("roll_deg", 0.0))),
                                 math.radians(float(ground_plane.get("pitch_deg", 0.0))),
                                 math.radians(float(ground_plane.get("yaw_deg", 0.0))))
        enabled = bool(ground_plane.get("enabled", False))
        h = ground_plane.get("height_m")
        if h is not None:
            h = float(h) + float(ground_plane.get("tag_thickness_m", 0.0) or 0.0)
    else:
        tilt = np.eye(4)
        enabled = False
        h = None
    T_level = T_mb2fc @ tilt

    # --- the stored matrix must agree with the fit it claims to embed.
    R_err = T_level[:3, :3].T @ R_MB2FC_LEVEL
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0)))
    if ang > _LEVEL_TOL_RAD:
        raise ValueError(
            "tf_chain.yaml T_mb2fc does not embed robot.yaml's front_cam "
            "ground-plane fit: T_mb2fc @ tilt is %.3f deg from the level "
            "camera. Regenerate it with "
            "apriltag_nav/tools/tf_chain_tool.py front-cam --apply "
            "(never hand-edit the rotation)." % math.degrees(ang))
    if h is not None and abs(float(T_mb2fc[2, 3]) - float(h)) > 1e-6:
        raise ValueError(
            "tf_chain.yaml T_mb2fc tz %.4f != robot.yaml ground_plane."
            "front_cam.height_m + robot.tag_thickness %.4f — the lens height "
            "above the floor; regenerate with tf_chain_tool.py front-cam --apply."
            % (float(T_mb2fc[2, 3]), float(h)))
    T_level[:3, :3] = R_MB2FC_LEVEL  # exact, the check above bounds the residual

    frame = (front_cam_frame or "auto").lower()
    if frame == "auto":
        frame = "level" if enabled else "physical"
    if frame not in ("level", "physical"):
        raise ValueError("front_cam_frame must be auto|level|physical, got %r"
                         % front_cam_frame)
    chain = T_level if frame == "level" else T_mb2fc
    note = ("front_cam detections taken as %s (ground_plane %s, roll %+.3f "
            "pitch %+.3f yaw %+.3f deg, lens %.3f m)"
            % (frame, "ON" if enabled else "OFF",
               float((ground_plane or {}).get("roll_deg", 0.0)),
               float((ground_plane or {}).get("pitch_deg", 0.0)),
               float((ground_plane or {}).get("yaw_deg", 0.0)),
               float(T_mb2fc[2, 3])))
    return Extrinsics(T_ab2mb=T_ab2mb, T_mb2fc=T_mb2fc, T_mb2fc_level=T_level,
                      T_mb2fc_chain=chain, front_cam_frame=frame,
                      ground_plane=ground_plane, note=note)


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
