#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_tf_chain_publisher.py
===========================
OFFLINE self-check for apriltag_nav.tf_chain_frames (the maths behind
scripts/tf_chain_publisher.py) — no ROS master, no robot::

    python3 src/apriltag_nav/tools/check_tf_chain_publisher.py

What it guards
--------------
The TF publisher is a VIEWER: it must show exactly what the runtime code
believes, never a second opinion. So every frame it builds is compared
with the loader / function the stack itself uses:

  * the static frames against tf_chain.yaml (fc, tip, hc) and against the
    ground-plane rule (fc_level is LEVEL and sits height_m + tag_thickness
    above the floor, as physical_T_mb2fc / load_extrinsics_full define it);
  * mb->ab against inv(T_ab2mb), and the lift's SIGN and AXIS (the arm base
    rises along mb z by exactly the lift);
  * world->mb->ab against arm_transform.transform_world_to_arm — the
    function pose-mode IK uses — for several /robot_pose headings, lift
    heights and floor offsets: a world point must land at the same arm-frame
    millimetres through both;
  * ab->ee against tf_chain.arm_flange_T: FK(q + dq) when the joints are
    consistent with the pose and the offsets are enabled (and the
    controller's pose beside it as ee_ctrl), the controller's pose otherwise;
  * the quaternion the node sends reproduces the rotation.
"""
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
sys.path.insert(0, os.path.join(HERE, '..', '..', 'path_tag_locator', 'src'))

# arm_transform imports rospy at module level — fake it on a dev box, as
# check_lift_compensation.py does. get_param returns the default it is given.
if 'rospy' not in sys.modules:
    try:
        __import__('rospy')
    except ImportError:
        _r = types.ModuleType('rospy')
        _r.get_param = lambda name, default=None: default
        _r.Subscriber = lambda *a, **k: None
        for _f in ('loginfo', 'logwarn', 'logerr', 'logerr_throttle', 'logwarn_throttle'):
            setattr(_r, _f, lambda *a, **k: None)
        sys.modules['rospy'] = _r
        _s, _sm = types.ModuleType('std_msgs'), types.ModuleType('std_msgs.msg')
        _sm.Float32 = type('Float32', (), {})
        _s.msg = _sm
        sys.modules['std_msgs'], sys.modules['std_msgs.msg'] = _s, _sm

from scipy.spatial.transform import Rotation as R                      # noqa: E402

from apriltag_nav import tf_chain                                       # noqa: E402
from apriltag_nav import tf_chain_frames as F                           # noqa: E402
from apriltag_nav.arm_transform import transform_world_to_arm           # noqa: E402
from apriltag_nav.tf_chain import R_MB2FC_LEVEL                         # noqa: E402
from apriltag_nav.paths import load_config                              # noqa: E402

_n = [0]
_bad = [0]


def check(name, ok, detail=''):
    _n[0] += 1
    if not ok:
        _bad[0] += 1
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")


def close(A, B, atol=1e-9):
    return np.allclose(np.asarray(A, float), np.asarray(B, float), atol=atol)


def rot_deg(A, B):
    c = (np.trace(np.asarray(A)[:3, :3].T @ np.asarray(B)[:3, :3]) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


class Msg(object):
    """Stand-in for /robot_pose."""
    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = x, y, theta


chain = tf_chain.load_tf_chain()
cfg = load_config()
statics = {(p, c): T for p, c, T in F.static_frames()}

print('== static frames vs tf_chain.yaml')
check('mb->fc is the stored T_mb2fc', close(statics[(F.MB, F.FC)], chain['T_mb2fc']))
check('ee->tip is the stored T_ee2tip', close(statics[(F.EE, F.TIP)], chain['T_ee2tip']))
check('ee->hc is inv(T_hc2ee)', close(statics[(F.EE, F.HC)] @ chain['T_hc2ee'], np.eye(4), 1e-9))
T_lv = statics[(F.MB, F.FC_LEVEL)]
_, p = tf_chain.physical_T_mb2fc(cfg)
check('mb->fc_level is LEVEL (R == diag(1,-1,-1))', close(T_lv[:3, :3], R_MB2FC_LEVEL, 1e-9),
      f'{rot_deg(T_lv, np.diag([1.0, -1.0, -1.0, 1.0])):.4f} deg off')
check('mb->fc_level lens at (camera_offset, camera_lateral, height_m + tag_thickness)',
      close(T_lv[:3, 3], [p['camera_offset'], p['camera_lateral'], p['height_m'] + p['tag_thickness']], 1e-9),
      f'{np.round(T_lv[:3, 3] * 1e3, 2).tolist()} mm')
check('fc and fc_level share the lens position (tilt is a rotation about the lens)',
      close(T_lv[:3, 3], statics[(F.MB, F.FC)][:3, 3], 1e-9))
T_u = statics[(F.MB, F.MB_URDF)]
check('mb->mobile_base_urdf is Rz(180) at the origin',
      close(T_u[:3, :3], np.diag([-1.0, -1.0, 1.0]), 1e-12) and close(T_u[:3, 3], 0, 1e-12))

print('== mb->ab and the lift')
T_ab2mb = chain['T_ab2mb']
T0 = F.T_mb2ab(T_ab2mb, 0.0)
check('lift 0: mb->ab == inv(T_ab2mb)', close(T0 @ T_ab2mb, np.eye(4), 1e-9))
T1 = F.T_mb2ab(T_ab2mb, 0.150)
check('lift +150 mm raises the arm base by +150 mm along mb z, x/y untouched',
      close(T1[:3, 3] - T0[:3, 3], [0.0, 0.0, 0.150], 1e-6),
      f'd = {np.round((T1[:3, 3] - T0[:3, 3]) * 1e3, 3).tolist()} mm')
check('lift leaves the rotation alone', rot_deg(T0, T1) < 1e-9)
check('lift None / 0 are the same', close(F.T_mb2ab(T_ab2mb, None), T0))

print('== world->mb->ab vs arm_transform.transform_world_to_arm (pose-mode IK path)')
pW = np.array([0.30, -0.20, 0.10])            # a world point
g = dict(x=pW[0], y=pW[1], z=pW[2], rx=1.5708, ry=-0.05, rz=3.1416)
floor = float(cfg['arm_calibration'].get('world_floor_z_m', 0.0))
worst = 0.0
for (x, y, th) in [(0.0, 0.0, 0.0), (1.2, -0.4, 90.0), (-0.7, 2.1, -90.0), (0.3, 0.3, 37.5)]:
    for lift in (0.0, 0.123):
        for fz in (0.0, floor, -0.0421):
            pos_mm, _ = transform_world_to_arm(g, Msg(x, y, th), lift_m=lift, euler='ZYX', floor_z_m=fz)
            T_w2ab = F.T_world2mb(x, y, th, fz) @ F.T_mb2ab(T_ab2mb, lift)
            ours = (tf_chain.invert_T(T_w2ab) @ np.r_[pW, 1.0])[:3] * 1e3
            worst = max(worst, float(np.linalg.norm(ours - np.asarray(pos_mm, float))))
check('same arm-frame mm through both paths (4 poses x 2 lifts x 3 floors)', worst < 1e-6,
      f'worst {worst:.2e} mm')

print('== ab->ee vs tf_chain.arm_flange_T')
from apriltag_nav.arm_fk import ArmChain                                 # noqa: E402
arm = ArmChain()
q = [-90.0, -90.0, 90.0, -90.0, -90.0, 10.0]
T_fk = arm.fk_flange(q)
tcp = tf_chain.t_mm(T_fk) + tf_chain.rpy_deg_xyz(T_fk)                  # the pose the controller would report
st = F.ChainState()
st.set_robot_pose(0.0, 0.0, 0.0)
st.set_arm_state(tcp, q, True)
fr = {(p_, c_): T for p_, c_, T in st.dynamic_frames()}
check('dynamic frames: world->mb, mb->ab, ab->ee present',
      all(k in fr for k in [(F.WORLD, F.MB), (F.MB, F.AB), (F.AB, F.EE)]), str(sorted(fr)))
T_ctrl, _ = tf_chain.arm_flange_T(tcp, None)
check('the controller pose round-trips (rpy convention of ArmState == pose_fr5_to_matrix_m)',
      close(T_ctrl, T_fk, 1e-6), f'{rot_deg(T_ctrl, T_fk):.2e} deg, {np.linalg.norm(T_ctrl[:3, 3] - T_fk[:3, 3]) * 1e3:.2e} mm')
if st.dq_deg is not None and np.any(st.dq_deg):
    T_exp = arm.fk_flange(q, dq_deg=st.dq_deg)
    check('joint offsets enabled: ab->ee == FK(q + dq)', close(fr[(F.AB, F.EE)], T_exp, 1e-9))
    check('... and ab->ee_ctrl == the controller pose, beside it',
          (F.AB, F.EE_CTRL) in fr and close(fr[(F.AB, F.EE_CTRL)], T_ctrl, 1e-9))
    d = np.linalg.norm(fr[(F.AB, F.EE)][:3, 3] - T_ctrl[:3, 3]) * 1e3
    check('ee and ee_ctrl differ (dq is visible in rviz)', d > 0.01, f'{d:.2f} mm apart')
else:
    check('joint offsets disabled: ab->ee == the controller pose, no ee_ctrl',
          close(fr[(F.AB, F.EE)], T_ctrl, 1e-9) and (F.AB, F.EE_CTRL) not in fr)
st2 = F.ChainState()
st2.dq_deg = None
st2.set_arm_state(tcp, q, True)
fr2 = {(p_, c_): T for p_, c_, T in st2.dynamic_frames()}
check('dq None: ab->ee is the controller pose and nothing else', close(fr2[(F.AB, F.EE)], T_ctrl, 1e-9)
      and (F.AB, F.EE_CTRL) not in fr2 and (F.WORLD, F.MB) not in fr2)
st2.set_arm_state(tcp, q, False)
check('pose_valid False: no arm frame', (F.AB, F.EE) not in {(p_, c_) for p_, c_, _ in st2.dynamic_frames()})
st2.set_arm_state(tcp, None, True)
fr3 = {(p_, c_): T for p_, c_, T in st2.dynamic_frames()}
check('no joints: the controller pose, no ee_ctrl', close(fr3[(F.AB, F.EE)], T_ctrl, 1e-9) and (F.AB, F.EE_CTRL) not in fr3)
inconsistent = [v + 5.0 for v in q]            # joints that do not match the pose
st3 = F.ChainState()
st3.set_arm_state(tcp, inconsistent, True)
fr4 = {(p_, c_): T for p_, c_, T in st3.dynamic_frames()}
check('joints inconsistent with the pose: guard falls back to the controller pose',
      close(fr4[(F.AB, F.EE)], T_ctrl, 1e-9) and (F.AB, F.EE_CTRL) not in fr4)

print('== lift input')
st4 = F.ChainState()
check('height_mm None is rejected and keeps lift 0', st4.set_lift_mm(None) is False and st4.lift_m == 0.0 and not st4.lift_known)
check('height_mm 123.4 -> 0.1234 m', st4.set_lift_mm(123.4) and abs(st4.lift_m - 0.1234) < 1e-12 and st4.lift_known)

print('== quaternion the node sends')
for T in [chain['T_mb2fc'], chain['T_hc2ee'], chain['T_ab2mb'], T_fk]:
    t, qq = F.matrix_to_tq(T)
    Rq = R.from_quat(qq)
    Rm = Rq.as_matrix() if hasattr(Rq, 'as_matrix') else Rq.as_dcm()
    check('matrix -> (t, q xyzw) -> matrix round-trips', close(Rm, T[:3, :3], 1e-9) and close(t, T[:3, 3], 1e-12))

print(f'\n{_n[0] - _bad[0]} ok, {_bad[0]} failed')
sys.exit(1 if _bad[0] else 0)
