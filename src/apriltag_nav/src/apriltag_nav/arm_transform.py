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

Calibration source of truth: apriltag_nav/config/tf/tf_chain.yaml `T_ab2mb`
(CALIBRATED 2026-09-21 by chain_calib against the printed A0 tag sheet;
design R = Rz(180°), t = (0, -0.100, 0.652)). This module reads that matrix
through `apriltag_nav.tf_chain` and derives its own parametrisation from it
(`arm_calibration_from_T_ab2mb`): T_mb2ab = inv(T_ab2mb) =
Rz(mount_yaw)·Ry(tilt_y)·Rx(tilt_x) | (offset_x, offset_y, base_z), body
frame x forward / y left / z up — so the arm goes where the locator chain
says a tag is, from ONE stored value. Calibrated: (-0.013, -0.120, 0.629),
yaw 181.43°, tilts 0.34 / 0.78° — the tilts are as much the chassis'
attitude at that parking as the mount (±1°), kept for consistency with the
locator chain. Note the docstring above about the lift: with a non-zero
tilt a lift move is no longer purely along arm z (0.78° × 0.34 m = ~5 mm of
x/y at full stroke) — physically right, since the lift is vertical in the
WORLD.
Lookup chain per value:
    private ROS param (~arm_body_offset_x, ... — a debug override)
    >  tf_chain.yaml T_ab2mb (the only stored copy; a missing file raises)
robot.yaml `arm_calibration` keeps only `csv_euler` (a CSV-reading
convention, not a transform).
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy

from apriltag_nav.paths import load_yaml_block
from apriltag_nav import tf_chain


def tag_floor_z_m(tag_info, calib=None, tag_thickness_m=0.001):
    """Height of the FLOOR under a calibrated floor tag above the CSV /
    world z datum, in metres — the per-tag term `transform_world_to_arm`
    adds to the arm base height (2026-09-22, user: use every calibrated
    value, not only x / y).

    map.yaml carries the calibrated tag `z` in the calibration world frame
    (reference_tags.yaml: z = 0 at the plate top, the design floor at
    `arm_calibration.world_floor_z_m` = -0.080). The tag is a 1 mm plate
    (`robot.tag_thickness`), so the floor under it is `z - thickness`, and
    its offset from the design floor — which is the datum the planner's
    CSV z and `arm_base_z` are measured from — is
        floor_z = (z - thickness) - world_floor_z_m.
    A tag without `z`, or `arm_calibration.use_tag_z: false`, gives 0.0:
    the pre-2026-09-22 behaviour bit for bit. |floor_z| above
    `tag_z_max_offset_m` (0.05) is refused (raises) — the calibrated z is
    the chain's weakest axis (~10 mm sd) and a gross value would send the
    tool INTO the workpiece, not merely off it.
    """
    calib = load_yaml_block('arm_calibration') if calib is None else calib
    if not calib.get('use_tag_z', True) or not tag_info or tag_info.get('z') is None:
        return 0.0
    floor_design = float(calib.get('world_floor_z_m', -0.080))
    floor_z = float(tag_info['z']) - float(tag_thickness_m) - floor_design
    limit = float(calib.get('tag_z_max_offset_m', 0.05))
    if abs(floor_z) > limit:
        raise ValueError(
            f"calibrated tag z {float(tag_info['z']):.4f} m puts the floor "
            f"{floor_z * 1000:+.1f} mm from the design floor "
            f"({floor_design:.3f} m) — beyond tag_z_max_offset_m "
            f"{limit:.3f} m; refusing (check the calibration, or raise the limit)")
    return floor_z


def transform_world_to_arm(g, msg, lift_m=0.0, euler=None, floor_z_m=0.0):
    """
    World frame (CSV pose frame) → arm base_link.

    g      : dict with x, y, z [m] and rx, ry, rz (radians)
    msg    : /robot_pose message (x, y [m] manipulator frame; theta [deg])
    lift_m : live lift extension above origin [m]. 0.0 reproduces the
             pre-2026-09-11 behaviour exactly; see the module docstring for
             why leaving it at 0 while the lift is raised puts the TCP that
             far ABOVE the target.
    floor_z_m : height of the floor under the robot above the CSV z datum
             [m] (`tag_floor_z_m`, from the calibrated tag z; 2026-09-22).
             The arm base rides on that floor, so it enters exactly like
             the lift: base height = arm_base_z + lift + floor. 0.0 = the
             design floor, the pre-2026-09-22 result bit for bit.

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
    _calib = load_yaml_block('arm_calibration')          # csv_euler only
    _mount = tf_chain.arm_calibration_from_T_ab2mb(tf_chain.load_transform('T_ab2mb'))
    body_off_x = rospy.get_param('~arm_body_offset_x', _mount['arm_body_offset_x'])
    body_off_y = rospy.get_param('~arm_body_offset_y', _mount['arm_body_offset_y'])
    body_off_z = rospy.get_param('~arm_base_z',        _mount['arm_base_z'])
    mount_yaw  = rospy.get_param('~arm_mount_yaw',     _mount['arm_mount_yaw'])
    tilt_x     = rospy.get_param('~arm_tilt_x',        _mount['arm_tilt_x'])
    tilt_y     = rospy.get_param('~arm_tilt_y',        _mount['arm_tilt_y'])

    x_base = -msg.y
    y_base = -msg.x
    theta  = np.radians(msg.theta)

    c, s = np.cos(theta), np.sin(theta)
    p_A_W = np.array([
        x_base + c * body_off_x - s * body_off_y,
        y_base + s * body_off_x + c * body_off_y,
        # lift raises the arm base (2026-09-11); the calibrated floor height
        # under the tag does the same (2026-09-22, tag_floor_z_m)
        body_off_z + float(lift_m) + float(floor_z_m)
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
        f"  [lift {float(lift_m) * 1000.0:.1f} mm, floor {float(floor_z_m) * 1000.0:+.1f} mm]"
    )

    return pos_mm, rpy_deg
