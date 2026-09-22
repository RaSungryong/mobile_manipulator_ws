# -*- coding: utf-8 -*-
"""
Forward kinematics of the FR10v6 from the planner URDF — for the ARM
joint-offset calibration (chain_calib/arm_offsets.py, 2026-09-21) and,
since 2026-09-22, for APPLYING those offsets in the locator chain
(`apriltag_nav.tf_chain.arm_flange_T`: T_ab2ee = FK(q + dq) instead of the
controller's own TCP, which is FK(q) with dq = 0). Lives in apriltag_nav so
that path_tag_locator and chain_calib share ONE kinematic model;
`chain_calib.arm_fk` re-exports it.

The chain is read from frcobot_description/urdf/fr10v6_mobile_vision_0317_test.urdf
(joints j1..j6 + the fixed `tool` joint = the FLANGE the controller reports
at tool offset 0). tools/check_pose_vs_joint.py showed this FK reproduces
the controller's own TCP at the home pose to 0.0 mm (2026-09-14), so it is
the controller's kinematic model as far as we can tell — and the point of
the offset calibration is that the model's JOINT ZEROS (and possibly link
lengths) do not match the physical arm.

Convention: q in DEGREES, the Fairino reading; the URDF joint axes are all
+z of their own frame after the fixed origin rotation, as the file has them.
No ROS imports; pure numpy.
"""
import math
import os
import xml.etree.ElementTree as ET

import numpy as np


def _rot_xyz(rpy):
    """URDF rpy = fixed-axis (extrinsic) x, y, z -> R = Rz(y) Ry(p) Rx(r)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def T_of(xyz, rpy=(0.0, 0.0, 0.0)):
    M = np.eye(4)
    M[:3, :3] = _rot_xyz(rpy)
    M[:3, 3] = xyz
    return M


def default_urdf_path():
    """The planner's URDF (user, 2026-09-14: the one in use), next to this
    package in the source space."""
    from apriltag_nav.paths import SRC_SPACE
    return os.path.join(SRC_SPACE, "frcobot_ros", "frcobot_description", "urdf",
                        "fr10v6_mobile_vision_0317_test.urdf")


class ArmChain:
    """j1..j6 origins (xyz, rpy) + flange transform, from the URDF."""

    def __init__(self, urdf_path=None):
        path = urdf_path or default_urdf_path()
        joints = {j.get("name"): j for j in ET.parse(path).getroot().findall("joint")}

        def xyz_rpy(j):
            o = j.find("origin")
            xyz = tuple(map(float, (o.get("xyz") if o is not None and o.get("xyz") else "0 0 0").split()))
            rpy = tuple(map(float, (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()))
            return xyz, rpy
        self.path = path
        self.origins = [xyz_rpy(joints["j%d" % i]) for i in range(1, 7)]
        self.axes = []
        for i in range(1, 7):
            a = joints["j%d" % i].find("axis")
            self.axes.append(np.array([float(v) for v in (a.get("xyz") if a is not None else "0 0 1").split()]))
        self.flange = T_of(*xyz_rpy(joints["tool"]))

    def fk_flange(self, q_deg, dq_deg=None, link_scale=None):
        """T_ab2ee for joint angles q (deg). ``dq_deg`` = zero offsets ADDED to
        the reading (the physical angle is q + dq); ``link_scale`` = per-joint
        multipliers on the origin translation (link-length errors), default 1."""
        M = np.eye(4)
        for k in range(6):
            xyz, rpy = self.origins[k]
            xyz = np.asarray(xyz, float) * (1.0 if link_scale is None else float(link_scale[k]))
            M = M @ T_of(xyz, rpy)
            ang = math.radians(float(q_deg[k]) + (0.0 if dq_deg is None else float(dq_deg[k])))
            ax = self.axes[k] / np.linalg.norm(self.axes[k])
            M = M @ _axis_angle_T(ax, ang)
        return M @ self.flange


def _axis_angle_T(axis, ang):
    x, y, z = axis
    c, s, C = math.cos(ang), math.sin(ang), 1.0 - math.cos(ang)
    R = np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                  [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                  [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])
    M = np.eye(4)
    M[:3, :3] = R
    return M
