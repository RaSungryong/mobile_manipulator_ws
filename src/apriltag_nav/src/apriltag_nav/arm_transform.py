#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
World-frame → arm-base-frame pose transform (4-DOF physical model).

Split out of arm_controller.py: this is pure geometry, not motion control,
and it is exactly the code the lift integration will have to touch —
`arm_base_z` is a constant here while the new base can move the arm
vertically. See docs/lift_arm_base_z_analysis.md before changing that.

Calibration source of truth: path_tag_locator/config/extrinsics.yaml
T_ab2mb — R = Rz(180°) exactly (mount_yaw = π, NO tilt), t = (0, -0.100, -0.652)
(arm base 652 mm above the mobile-base origin, lift at its origin; measured on
the replacement base 2026-08-13, the previous base read 1.025). The no-tilt
claim is independently confirmed by the 655-point real-robot fit
(task/csv/calib_data_params.yaml, tilt ≈ 0). Earlier USD-derived defaults
(base_z 1.0076, tilts ±1.5°) are superseded — those tilts do not exist on
the real platform. Lookup chain per value:
    private ROS param  >  robot.yaml `arm_calibration`  >  hardcoded default
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy

from apriltag_nav.paths import load_yaml_block


def transform_world_to_arm(g, msg):
    """
    World frame (CSV pose frame) → arm base_link.

    g   : dict with x, y, z [m] and rx, ry, rz (ZYX intrinsic, rad)
    msg : /robot_pose message (x, y [m] manipulator frame; theta [deg])

    Returns (pos_mm, rpy_deg) in Fairino SDK units (mm, degrees).
    """
    _calib = load_yaml_block('arm_calibration')
    body_off_x = rospy.get_param('~arm_body_offset_x', _calib.get('arm_body_offset_x', 0.0))
    body_off_y = rospy.get_param('~arm_body_offset_y', _calib.get('arm_body_offset_y', -0.100))
    body_off_z = rospy.get_param('~arm_base_z',        _calib.get('arm_base_z',        0.652))
    mount_yaw  = rospy.get_param('~arm_mount_yaw',     _calib.get('arm_mount_yaw',     np.pi))
    tilt_x     = rospy.get_param('~arm_tilt_x',        _calib.get('arm_tilt_x',        0.0))
    tilt_y     = rospy.get_param('~arm_tilt_y',        _calib.get('arm_tilt_y',        0.0))

    x_base = -msg.y
    y_base = -msg.x
    theta  = np.radians(msg.theta)

    c, s = np.cos(theta), np.sin(theta)
    p_A_W = np.array([
        x_base + c * body_off_x - s * body_off_y,
        y_base + s * body_off_x + c * body_off_y,
        body_off_z
    ])

    alpha = theta + mount_yaw
    R_WA = (R.from_euler('z', alpha)
            * R.from_euler('y', tilt_y)
            * R.from_euler('x', tilt_x)).as_matrix()
    R_AW = R_WA.T
    R_AW_rot = R.from_matrix(R_AW)

    # Position: world → arm base_link, then meters → mm for Fairino
    p_W = np.array([g["x"], g["y"], g["z"]])
    p_arm = R_AW @ (p_W - p_A_W)
    pos_mm = p_arm * 1000.0

    # Orientation: CSV ZYX intrinsic → arm frame → degrees for Fairino
    r_csv_W = R.from_euler('zyx', [g["rx"], g["ry"], g["rz"]])
    r_arm = R_AW_rot * r_csv_W
    rpy_deg = r_arm.as_euler("xyz", degrees=True)

    rospy.loginfo(
        f"[Arm REAL] Transform: world=({g['x']:.3f}, {g['y']:.3f}, {g['z']:.3f}) "
        f"-> arm=({pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}) mm"
    )

    return pos_mm, rpy_deg
