# -*- coding: utf-8 -*-
"""
tf_chain_frames.py — the fixed-transform chain of config/tf/tf_chain.yaml
as ROS TF frames (pure numpy; the node is scripts/tf_chain_publisher.py).

Frame names are the short names of tf_chain.yaml / docs/FRAMES_kr.md::

    world ─(/robot_pose)─▶ mb ─┬─ fc, fc_level       static: T_mb2fc, + the ground-plane tilt
                               ├─ mobile_base_urdf   static: Rz(180 deg) — the planner URDF's cart frame
                               └─(lift)─▶ ab ─(/arm/state FK)─▶ ee ─┬─ tip   static: T_ee2tip
                                                                    └─ hc    static: inv(T_hc2ee)
    ab ─▶ ee_ctrl   the controller's own TCP pose (no joint offsets), published
                    beside `ee` while the offsets are applied, so dq is visible.

Nothing here is new geometry. Every matrix comes from the loaders the
runtime already uses — tf_chain.load_tf_chain, physical_T_mb2fc,
arm_flange_T, the /robot_pose axis rule of arm_transform — so what rviz
shows IS what the code believes. Read-only: this module writes no file,
and nothing in the stack subscribes to the frames it produces.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation as R

from apriltag_nav import tf_chain
from apriltag_nav.ground_plane import T_tilted_to_level
from apriltag_nav.paths import load_config

WORLD = 'world'
MB = 'mb'
AB = 'ab'
EE = 'ee'
EE_CTRL = 'ee_ctrl'
TIP = 'tip'
HC = 'hc'
FC = 'fc'
FC_LEVEL = 'fc_level'
MB_URDF = 'mobile_base_urdf'


def _T_rz(deg, t=(0.0, 0.0, 0.0)):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    T[:3, 3] = t
    return T


def compensate_T_ab2mb(T_ab2mb, lift_m):
    """T_ab2mb for the live lift extension — the same rule as
    path_tag_locator.chain.compensate_T_ab2mb (that package depends on this
    one, not the other way round, hence the copy): tf_chain.yaml measures
    T_ab2mb with the lift AT ORIGIN; raising the lift by h lifts the arm base
    h above mb, so mb sits h further below ab along ab.z (R is Rz(180 deg) up
    to the calibrated yaw, so only t_z moves)."""
    T = np.array(T_ab2mb, dtype=float, copy=True)
    T[2, 3] -= float(lift_m or 0.0)
    return T


def T_mb2ab(T_ab2mb, lift_m=0.0):
    """Pose of the arm base in mb at the given lift extension."""
    return tf_chain.invert_T(compensate_T_ab2mb(T_ab2mb, lift_m))


def T_world2mb(x, y, theta_deg, floor_z_m=0.0):
    """Pose of mb in world from a /robot_pose message (x, y [m], theta [deg]).

    The axis rule is arm_transform.transform_world_to_arm's, verbatim:
    world_x = -msg.y, world_y = -msg.x, theta is the body heading about
    world z. mb's origin is on the floor, `floor_z_m` below the CSV z datum
    (robot.yaml arm_calibration.world_floor_z_m, -0.080 by design)."""
    return _T_rz(float(theta_deg), (-float(y), -float(x), float(floor_z_m)))


def static_frames(tf_yaml=None, robot_yaml=None):
    """[(parent, child, T)] for every FIXED frame: mb->fc, mb->fc_level,
    mb->mobile_base_urdf, ee->tip, ee->hc. `fc` is the stored T_mb2fc (what
    the code reads); `fc_level` = T_mb2fc @ T_tilted_to_level(fit) — the
    LEVEL virtual camera robot_camera_node publishes detections in, derived
    exactly as path_tag_locator.constants.load_extrinsics_full does."""
    chain = tf_chain.load_tf_chain(tf_yaml)
    _, p = tf_chain.physical_T_mb2fc(load_config(robot_yaml))
    tilt = T_tilted_to_level(math.radians(p['roll_deg']), math.radians(p['pitch_deg']),
                             math.radians(p['yaw_deg']))
    return [
        (MB, FC, chain['T_mb2fc']),
        (MB, FC_LEVEL, chain['T_mb2fc'] @ tilt),
        (MB, MB_URDF, _T_rz(180.0)),
        (EE, TIP, chain['T_ee2tip']),
        (EE, HC, tf_chain.invert_T(chain['T_hc2ee'])),
    ]


def matrix_to_tq(T):
    """(translation xyz [m], quaternion xyzw) of a 4x4 — for TransformStamped."""
    T = np.asarray(T, dtype=float)
    rot = R.from_matrix(T[:3, :3]) if hasattr(R, 'from_matrix') else R.from_dcm(T[:3, :3])
    return [float(v) for v in T[:3, 3]], [float(v) for v in rot.as_quat()]


class ChainState(object):
    """Latest live inputs -> the DYNAMIC frames.

    world->mb needs a /robot_pose; mb->ab is always there (lift 0 until
    /lifter/state says otherwise); ab->ee needs a valid /arm/state. A frame
    whose input has not arrived is simply not in `dynamic_frames()`.
    `warn` (callable or None) receives arm_flange_T's guard messages."""

    def __init__(self, tf_yaml=None, robot_yaml=None, warn=None):
        chain = tf_chain.load_tf_chain(tf_yaml)
        self.T_ab2mb = chain['T_ab2mb']
        cfg = load_config(robot_yaml)
        self.floor_z_m = float((cfg.get('arm_calibration') or {}).get('world_floor_z_m', 0.0) or 0.0)
        try:                                   # zeros when absent / disabled (load_joint_offsets)
            self.dq_deg = tf_chain.load_joint_offsets(require_enabled=True)
        except Exception:                      # unreadable: the controller's own FK
            self.dq_deg = None
        self.warn = warn
        self.pose = None          # (x, y, theta_deg) of the last /robot_pose
        self.lift_m = 0.0
        self.lift_known = False
        self.arm = None           # (tcp_pose_mm_deg, joints_deg)

    def offsets_enabled(self):
        return self.dq_deg is not None and bool(np.any(self.dq_deg))

    # --- inputs ------------------------------------------------------------
    def set_robot_pose(self, x, y, theta_deg):
        self.pose = (float(x), float(y), float(theta_deg))

    def set_lift_mm(self, height_mm):
        if height_mm is None:
            return False
        self.lift_m = float(height_mm) / 1000.0
        self.lift_known = True
        return True

    def set_arm_state(self, tcp_pose_mm_deg, joints_deg, pose_valid=True):
        if not pose_valid or tcp_pose_mm_deg is None or len(tcp_pose_mm_deg) < 6:
            self.arm = None
            return False
        self.arm = ([float(v) for v in tcp_pose_mm_deg[:6]],
                    None if joints_deg is None or len(joints_deg) < 6 else [float(v) for v in joints_deg[:6]])
        return True

    # --- outputs -----------------------------------------------------------
    def dynamic_frames(self):
        out = []
        if self.pose is not None:
            x, y, th = self.pose
            out.append((WORLD, MB, T_world2mb(x, y, th, self.floor_z_m)))
        out.append((MB, AB, T_mb2ab(self.T_ab2mb, self.lift_m)))
        if self.arm is not None:
            tcp, joints = self.arm
            T_ee, applied = tf_chain.arm_flange_T(tcp, joints, self.dq_deg, warn=self.warn)
            out.append((AB, EE, T_ee))
            if applied:
                T_ctrl, _ = tf_chain.arm_flange_T(tcp, None)
                out.append((AB, EE_CTRL, T_ctrl))
        return out
