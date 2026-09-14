#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
World-frame → arm-base-frame pose transform (4-DOF physical model).

Split out of arm_controller.py: this is pure geometry, not motion control.

Lift compensation (2026-09-11)
------------------------------
`arm_base_z` is measured with the lift AT ORIGIN. The lift raises the arm
base by up to ~343 mm on top of that, so `lift_m` must carry the live
extension (`/lifter/height` via `lift_height.LiftHeightListener`) or the
result is wrong by exactly that much.

Direction, since it is not guessable: `R_AW` is a pure z-rotation (the
mount has no tilt), so it does not mix z, and `p_arm[2] = z_world -
arm_base_z`. Leaving the lift out makes that term `lift_m` too LARGE, and
the arm — whose base really is `lift_m` higher — puts the TCP `lift_m`
ABOVE the target. So an uncompensated raised lift scans too HIGH, away
from the plate: a wrong measurement rather than a collision. It is not
self-correcting either — the Keyence standoff loop is clamped to
`keyence_max_step_mm` (1.0 mm) per step and cannot close a 150-300 mm gap.

Joint-mode targets are unaffected: those CSVs are absolute joint angles
fed straight to MoveJ and no transform reads them.

Calibration source of truth: path_tag_locator/config/extrinsics.yaml
T_ab2mb — R = Rz(180°) exactly (mount_yaw = π, NO tilt), t = (0, -0.100, -0.652)
(arm base 652 mm above the mobile-base origin, lift at its origin; measured
on the replacement base 2026-08-13 — figures from before that swap describe
different hardware). The no-tilt claim was independently confirmed by a
655-point real-robot fit which found tilt ≈ 0.0001/0.0007 rad; that fit's
data (`task/csv/calib_data*`) was old-base and has been deleted, so the
number survives only as this note. Earlier USD-derived defaults (tilts
±1.5°) are superseded — those tilts do not exist on the real platform.
Lookup chain per value:
    private ROS param  >  robot.yaml `arm_calibration`  >  hardcoded default
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy

from apriltag_nav.paths import load_yaml_block


def transform_world_to_arm(g, msg, lift_m=0.0, euler=None):
    """
    World frame (CSV pose frame) → arm base_link.

    g      : dict with x, y, z [m] and rx, ry, rz (radians)
    msg    : /robot_pose message (x, y [m] manipulator frame; theta [deg])
    lift_m : live lift extension above origin [m]. 0.0 reproduces the
             pre-2026-09-11 behaviour exactly; see the module docstring for
             why leaving it at 0 while the lift is raised puts the TCP that
             far ABOVE the target.

    ⚠️ `g`'s rx/ry/rz convention is PER GENERATOR — `euler` (default:
    robot.yaml `arm_calibration.csv_euler`) is the scipy from_euler spec
    applied to [rx, ry, rz]:
      * "ZYX" — the 2026-09 RRT planner (`assigned_workpoints_*`): checked
        2026-09-14 against the FK of its own paired joint rows, 0.00 deg
        over 943 points. "zyx" read those files 180 deg off (tool UP).
      * "zyx" — the deleted pre-cell-swap `grid_path_line*.csv`: on those
        it put the tool z-axis at (-0.089, -0.037, -0.995), down at the
        plate, and "ZYX" gave horizontal (the 2026-09-11 finding).
    Both findings are real; they are about different files. Change the
    key, not this line, and verify a new generator the same way
    (tools/check_pose_vs_joint.py).

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
        body_off_z + float(lift_m)      # lift raises the arm base, 2026-09-11
    ])

    alpha = theta + mount_yaw
    _rot_WA = (R.from_euler('z', alpha)
               * R.from_euler('y', tilt_y)
               * R.from_euler('x', tilt_x))
    # scipy compat: >=1.4 as_matrix()/from_matrix(), 1.3 as_dcm()/from_dcm()
    R_WA = (_rot_WA.as_matrix() if hasattr(_rot_WA, 'as_matrix')
            else _rot_WA.as_dcm())
    R_AW = R_WA.T
    R_AW_rot = (R.from_matrix(R_AW) if hasattr(R, 'from_matrix')
                else R.from_dcm(R_AW))

    # Position: world → arm base_link, then meters → mm for Fairino
    p_W = np.array([g["x"], g["y"], g["z"]])
    p_arm = R_AW @ (p_W - p_A_W)
    pos_mm = p_arm * 1000.0

    # Orientation: CSV ZYX intrinsic → arm frame → degrees for Fairino
    if euler is None:
        euler = str(_calib.get('csv_euler', 'ZYX'))
    r_csv_W = R.from_euler(euler, [g["rx"], g["ry"], g["rz"]])
    r_arm = R_AW_rot * r_csv_W
    rpy_deg = r_arm.as_euler("xyz", degrees=True)

    rospy.loginfo(
        f"[Arm REAL] Transform: world=({g['x']:.3f}, {g['y']:.3f}, {g['z']:.3f}) "
        f"-> arm=({pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}) mm"
        f"  [lift {float(lift_m) * 1000.0:.1f} mm]"
    )

    return pos_mm, rpy_deg
