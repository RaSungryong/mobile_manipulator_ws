#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_lift_compensation.py
==========================
OFFLINE self-check for the pose-mode lift compensation (2026-09-11) — no
ROS master, no robot::

    python3 src/apriltag_nav/tools/check_lift_compensation.py

What it guards
--------------
`arm_calibration.arm_base_z` is measured with the lift AT ORIGIN and the
lift adds up to ~343 mm on top. Before this change
`transform_world_to_arm` used the constant, so a pose-mode scan run with
the lift raised put the TCP exactly that far ABOVE its world target — a
silently wrong measurement, and one the Keyence standoff loop cannot undo
(it is clamped to 1 mm per step).

The checks below pin the three things that could go wrong:
  * the SIGN (compensating the wrong way doubles the error instead of
    removing it),
  * that the lift moves the target along the WORLD vertical — with the
    2026-09-21 calibrated tilt that is not purely arm z, and must not leak
    into x/y or into the orientation,
  * that `lift_m=0` still reproduces the old result BIT-FOR-BIT, so this
    is a no-op for every task that runs at the origin.
"""
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, "src"))

# arm_transform imports rospy for its param lookups and one loginfo; stub it
# so this runs on a dev box. get_param must return the DEFAULT it is given,
# which is how the real lookup behaves with no param server entry.
if "rospy" not in sys.modules:
    try:
        import rospy  # noqa: F401
    except ImportError:
        _r = types.ModuleType("rospy")
        _r.get_param = lambda name, default=None: default
        _r.Subscriber = lambda *a, **k: None
        for _f in ("loginfo", "logwarn", "logerr", "logerr_throttle",
                   "logwarn_throttle"):
            setattr(_r, _f, lambda *a, **k: None)
        sys.modules["rospy"] = _r
        _s, _sm = types.ModuleType("std_msgs"), types.ModuleType("std_msgs.msg")
        _sm.Float32 = type("Float32", (), {})
        _s.msg = _sm
        sys.modules["std_msgs"], sys.modules["std_msgs.msg"] = _s, _sm

from apriltag_nav.arm_transform import transform_world_to_arm

_n = [0]
_bad = [0]


def check(name, ok, detail=""):
    _n[0] += 1
    if not ok:
        _bad[0] += 1
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")


class Msg(object):
    """Stand-in for /robot_pose."""
    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta


def g(x, y, z, rx=1.5708, ry=-0.05, rz=3.1416):
    return dict(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)


# The real robot.yaml value, so the check fails if the config drifts from
# the assumption this test is written against.
from apriltag_nav.paths import load_yaml_block
CAL = load_yaml_block('arm_calibration')
BASE_Z = float(CAL.get('arm_base_z', 0.652))
STROKE_MM = 343.35          # soft_max_counts 6900 x mm_per_count 0.04976077

print(__doc__.strip().splitlines()[0])
print(f"\narm_base_z = {BASE_Z} m (from robot.yaml), full stroke ~{STROKE_MM:.1f} mm")

MSGS = [Msg(0.0, 0.0, 0.0), Msg(-1.71, 0.15, 90.0),
        Msg(1.71, -0.15, -90.0), Msg(2.18, -0.65, 90.0)]
PTS = [g(0.0, 0.0, 0.0), g(0.35, -0.20, -0.08),
       g(-0.42, 0.66, 0.05), g(1.10, 0.02, -0.08)]

print("\n== lift_m = 0 reproduces the pre-change result exactly ==")
worst = 0.0
for m in MSGS:
    for p in PTS:
        a, _ = transform_world_to_arm(p, m)              # default arg
        b, _ = transform_world_to_arm(p, m, 0.0)         # explicit
        worst = max(worst, float(np.abs(np.asarray(a) - np.asarray(b)).max()))
check("default argument == explicit 0.0", worst == 0.0, f"max diff {worst:g}")

print("\n== the lift moves the target by exactly -h along the WORLD vertical (arm frame) ==")
# Since 2026-09-21 the mount carries a small calibrated tilt (robot.yaml
# arm_tilt_x/y), so the lift — vertical in the WORLD — is no longer purely
# along arm z: it leaks h*sin(tilt) into arm x/y (0.78 deg x 343 mm ~ 5 mm),
# which is physically right. The test is therefore against the full model:
#   p_arm(h) - p_arm(0) == -R_AW . (0, 0, h)         (exact)
#   p_A_W(h) + R_WA . p_arm(h) == the requested world point   (exact)
from scipy.spatial.transform import Rotation as _R
_TX, _TY, _YAW = float(CAL.get('arm_tilt_x', 0.0)), float(CAL.get('arm_tilt_y', 0.0)), float(CAL.get('arm_mount_yaw', np.pi))
_OX, _OY = float(CAL.get('arm_body_offset_x', 0.0)), float(CAL.get('arm_body_offset_y', -0.1))

def _R_WA(theta_deg):
    return (_R.from_euler('z', np.radians(theta_deg) + _YAW) * _R.from_euler('y', _TY) * _R.from_euler('x', _TX)).as_matrix()

def _p_A_W(m, h):
    th = np.radians(m.theta); c, sn = np.cos(th), np.sin(th)
    return np.array([-m.y + c * _OX - sn * _OY, -m.x + sn * _OX + c * _OY, BASE_Z + h])

bad_dir = 0.0
bad_rpy = 0.0
for m in MSGS:
    for p in PTS:
        for h_mm in (0.0, 1.0, 150.0, 300.0, STROKE_MM):
            p0, r0 = transform_world_to_arm(p, m, 0.0)
            p1, r1 = transform_world_to_arm(p, m, h_mm / 1000.0)
            expect = -_R_WA(m.theta).T @ np.array([0.0, 0.0, h_mm])
            bad_dir = max(bad_dir, float(np.abs((np.asarray(p1) - np.asarray(p0)) - expect).max()))
            bad_rpy = max(bad_rpy, float(np.abs(np.asarray(r1) - np.asarray(r0)).max()))
check("the lift shifts the arm-frame target by exactly -R_AW.(0,0,h)", bad_dir < 1e-6, f"max {bad_dir:.3e} mm")
check("orientation is untouched", bad_rpy < 1e-9, f"max {bad_rpy:.3e} deg")
_leak = STROKE_MM * np.sin(np.hypot(_TX, _TY))
print(f"   (tilt {np.degrees(np.hypot(_TX, _TY)):.3f} deg -> the full stroke leaks {_leak:.1f} mm into arm x/y — expected, the lift is vertical in the world)")

print("\n== SIGN: compensation must CANCEL the error, not double it ==")
# Physical model: the arm base really sits at p_A_W(h). Commanding the arm
# to put its TCP at p_arm lands the TCP at p_A_W(h) + R_WA . p_arm; correct
# behaviour: that equals the requested world point for every h.
err_comp, err_uncomp = 0.0, 0.0
for h_mm in (0.0, 150.0, 300.0, STROKE_MM):
    h = h_mm / 1000.0
    for m in MSGS:
        for p in PTS:
            pc, _ = transform_world_to_arm(p, m, h)       # compensated
            pu, _ = transform_world_to_arm(p, m, 0.0)     # as before
            RWA = _R_WA(m.theta)
            landed_c = _p_A_W(m, h) + RWA @ (np.asarray(pc) / 1000.0)
            landed_u = _p_A_W(m, h) + RWA @ (np.asarray(pu) / 1000.0)
            err_comp = max(err_comp, float(np.linalg.norm(landed_c - [p["x"], p["y"], p["z"]])))
            err_uncomp = max(err_uncomp, abs(landed_u[2] - p["z"]))
check("compensated TCP lands on the requested world point", err_comp < 1e-9,
      f"worst {err_comp*1e3:.3e} mm")
check("uncompensated lands HIGH by the full stroke (the bug being fixed)",
      abs(err_uncomp * 1e3 - STROKE_MM) < 1e-6,
      f"worst {err_uncomp*1e3:.2f} mm = the stroke")

# direction, stated explicitly: a wrong sign would put it 2h off (vertically)
h = 0.300
p, m = PTS[1], MSGS[1]
pw, _ = transform_world_to_arm(p, m, -h)      # deliberately wrong sign
landed_wrong = _p_A_W(m, h) + _R_WA(m.theta) @ (np.asarray(pw) / 1000.0)
check("a flipped sign would be 2x the lift off (so the sign is testable)",
      abs((landed_wrong[2] - p["z"]) - 2 * h) < 1e-9,
      f"{(landed_wrong[2] - p['z'])*1e3:.1f} mm vs 2h = {2*h*1e3:.1f} mm")

print("\n== magnitude at the heights the tasks actually use ==")
for h_mm, what in ((150.0, "optimized_joints_line2"),
                   (300.0, "optimized_joints_line1"),
                   (STROKE_MM, "top of stroke")):
    print(f"  lift {h_mm:6.2f} mm ({what:24s}) -> uncompensated pose-mode "
          f"error {h_mm:6.2f} mm in world z")
print("  (pose-mode CSVs carry no lift_height today, so this is 0 in the")
print("   current task set — the compensation is what makes a raised-lift")
print("   pose task possible at all.)")

print("\n== LiftHeightListener: unknown must not masquerade as origin ==")
from apriltag_nav.lift_height import LiftHeightListener
_r = sys.modules["rospy"]
_r.Subscriber = lambda *a, **k: None
lis = LiftHeightListener.__new__(LiftHeightListener)
import threading
lis._lock, lis._mm, lis._topic = threading.Lock(), None, "/lifter/height"
check("height_m() is None before any message (not 0.0)",
      lis.height_m() is None)
lis._mm = 150.0
check("height_m() converts mm -> m", abs(lis.height_m() - 0.150) < 1e-12)
lis._mm = 0.0
check("a real 0.0 mm reading is 0.0, not None", lis.height_m() == 0.0)

print("\n== ArmController's unknown-height POLICY ==")
# The one decision in this change: what happens when lifter_node has never
# published. Import the real class under stubs (the Fairino SDK and the ROS
# message packages are not needed by _pose_lift_m) and drive both branches.
for _name, _attrs in (("robot_msgs.msg", ("Pose2DWithFlag",)),
                      ("robot_msgs.srv", ("CaptureImages",)),
                      ("cv_bridge", ("CvBridge",)),
                      ("sensor_msgs.msg", ("Image",)),
                      ("std_msgs.msg", ("Bool", "String", "Float32")),
                      ("fairino", ("Robot",))):
    # top up an existing stub rather than skipping it — std_msgs.msg was
    # created above with only Float32 and arm_controller also wants Bool
    _mod = sys.modules.get(_name) or types.ModuleType(_name)
    for _a in _attrs:
        if not hasattr(_mod, _a):
            setattr(_mod, _a, type(_a, (), {}))
    sys.modules[_name] = _mod
    _p = _name.split(".")
    if len(_p) > 1:
        _pk = sys.modules.setdefault(_p[0], types.ModuleType(_p[0]))
        setattr(_pk, _p[1], _mod)

try:
    from apriltag_nav.arm_controller import ArmController

    class Stub(object):
        pass

    def policy(height_mm, require):
        s = Stub()
        s.lift_listener = LiftHeightListener.__new__(LiftHeightListener)
        s.lift_listener._lock = threading.Lock()
        s.lift_listener._mm = height_mm
        s.lift_listener._topic = "/lifter/height"
        s.require_lift_height = require
        return ArmController._pose_lift_m(s)

    check("known height is used as metres", abs(policy(150.0, False) - 0.150) < 1e-12)
    check("known height is used even when require=True",
          abs(policy(300.0, True) - 0.300) < 1e-12)
    check("unknown + require=False assumes the origin (old behaviour)",
          policy(None, False) == 0.0)
    raised = False
    try:
        policy(None, True)
    except RuntimeError as e:
        raised = "lifter_node" in str(e)
    check("unknown + require=True refuses the move, naming lifter_node", raised)
except Exception as e:                                  # pragma: no cover
    print(f"  [skip] could not import ArmController under stubs ({e})")

print(f"\n{_n[0] - _bad[0]}/{_n[0]} checks passed")
sys.exit(1 if _bad[0] else 0)
