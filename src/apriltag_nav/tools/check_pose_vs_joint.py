#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Do pose mode and joint mode reach the SAME place? (2026-09-14)

The planner exports each work point twice: joint angles (rrt_final_path_*,
replayed by MoveJ — physically right, per the user) and vision-tip world
coordinates (assigned_workpoints_*, solved by IK). This check drives the
REAL transform_world_to_arm + ArmController._exec_pose with one paired row
per lane against a fake Fairino that records the IK target, and compares
that target with the URDF forward kinematics of the joint row:

  * the CSV orientation must be read as `csv_euler` "ZYX" — 0.00 deg;
    "zyx" (the pre-2026-09-14 code) is 180 deg off;
  * when the controller's active tool is the FLANGE (what /arm/state showed
    on 2026-09-14: home TCP == URDF flange to 0.0 mm), the IK target must be
    the flange pose, i.e. tip - R @ (0, -253, 225.2) mm — else the flange
    is sent to the tip's coordinates and the tip lands 338.7 mm away;
  * _probe_tool_frame's three verdicts on GetTCPOffset.

Rows are embedded verbatim (standoff_050 errorX export of 13:42, groups
106 and 119) so the check does not depend on task/csv, which the user
replaces freely. Run with a sourced workspace:
    python3 tools/check_pose_vs_joint.py
"""
import math
import os
import sys
import threading
import time
import types

import numpy as np
from scipy.spatial.transform import Rotation as R

# ---------------------------------------------------------------- stubs
LOG = {'info': [], 'warn': [], 'err': []}
rospy = types.ModuleType('rospy')
rospy.loginfo = lambda m, *a: LOG['info'].append(str(m))
rospy.logwarn = lambda m, *a: LOG['warn'].append(str(m))
rospy.logerr = lambda m, *a: LOG['err'].append(str(m))
rospy.logdebug = lambda m, *a: None
rospy.logwarn_throttle = lambda t, m, *a: LOG['warn'].append(str(m))
rospy.logerr_throttle = lambda t, m, *a: LOG['err'].append(str(m))
rospy.get_param = lambda name, default=None: default
rospy.get_time = lambda: time.time()
rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda m: None)
rospy.Subscriber = lambda *a, **k: None
sys.modules['rospy'] = rospy


class _Msg:
    def __init__(self, data=None):
        self.data = data


for pkg, names in (('std_msgs', ('Bool', 'Float32', 'String')), ('robot_msgs', ('Pose2DWithFlag',))):
    m = types.ModuleType(pkg); mm = types.ModuleType(pkg + '.msg')
    for n in names:
        setattr(mm, n, type(n, (_Msg,), {}))
    sys.modules[pkg] = m; sys.modules[pkg + '.msg'] = mm
fairino = types.ModuleType('fairino'); fairino.Robot = types.SimpleNamespace(RPC=object)
sys.modules['fairino'] = fairino
sp = types.ModuleType('apriltag_nav.scan_pipeline'); sp.RaScanPipeline = object
sys.modules['apriltag_nav.scan_pipeline'] = sp

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))
from apriltag_nav import arm_transform                        # noqa: E402
from apriltag_nav.arm_transform import transform_world_to_arm  # noqa: E402
from apriltag_nav.arm_controller import ArmController          # noqa: E402

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


# ---------------------------------------------------------------- URDF FK
# The planner's URDF (user, 2026-09-14: "이것이 현재 사용하는 urdf"):
# frcobot_description/urdf/fr10v6_mobile_vision_0317_test.urdf. Chain
# base_link -> j1..j6 -> tool_Link (the flange the controller reports at tool
# offset 0) -> vision -> vision_tip (= tool 1's offset). Read from the file
# so the check follows it; the embedded copy is the fallback.
URDF = os.path.join(os.path.dirname(PKG), 'frcobot_ros',
                    'frcobot_description', 'urdf', 'fr10v6_mobile_vision_0317_test.urdf')


def _T(xyz, rpy=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R.from_euler('xyz', rpy).as_matrix(); M[:3, 3] = xyz; return M


CHAIN = [((0, 0, 0), (0, 0, 0)), ((0, 0, 0.18), (1.5708, 0, 0)), ((-0.7, 0, 0), (0, 0, 0)),
         ((-0.586, 0, 0), (0, 0, 0)), ((0, 0, 0.159), (1.5708, 0, 0)), ((0, 0, 0.114), (-1.5708, 0, 0))]
FLANGE = _T((0, 0, 0.106))
URDF_TIP_MM = None
URDF_MOUNT = None
if os.path.isfile(URDF):
    import xml.etree.ElementTree as ET
    _joints = {j.get('name'): j for j in ET.parse(URDF).getroot().findall('joint')}

    def _xyz_rpy(j):
        o = j.find('origin')
        xyz = tuple(map(float, (o.get('xyz') if o is not None and o.get('xyz') else '0 0 0').split()))
        rpy = tuple(map(float, (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0').split()))
        return xyz, rpy
    CHAIN = [_xyz_rpy(_joints[f'j{i}']) for i in range(1, 7)]
    FLANGE = _T(*_xyz_rpy(_joints['tool']))
    _tip = np.zeros(3)
    for name in ('vision_fixed', 'vision_tip_joint'):   # both fixed, rpy 0
        _tip = _tip + np.array(_xyz_rpy(_joints[name])[0])
    URDF_TIP_MM = _tip * 1000.0
    URDF_MOUNT = _xyz_rpy(_joints['mobile_to_base'])


def fk_flange(q_rad):
    M = np.eye(4)
    for (xyz, rpy), q in zip(CHAIN, q_rad):
        M = M @ _T(xyz, rpy) @ _T((0, 0, 0), (0, 0, q))
    return M @ FLANGE


TIP = np.array([0.0, -253.0, 225.2])     # mm, flange frame (set_tool_tcp.py)

print('== the planner URDF')
check(URDF_TIP_MM is not None, f'URDF found: {URDF}')
if URDF_TIP_MM is not None:
    check(np.linalg.norm(URDF_TIP_MM - TIP) < 0.05,
          f'URDF vision_tip offset {np.round(URDF_TIP_MM, 2).tolist()} mm == tool 1 / robot.yaml vision_tip_offset_mm {TIP.tolist()}')
    mx, mr = URDF_MOUNT
    check(abs(mx[2] - 0.652) < 1e-6 and abs(mx[1] - 0.1) < 1e-6 and abs(mx[0]) < 1e-6 and max(map(abs, mr)) < 1e-6,
          f'URDF mobile_to_base {mx} rpy {mr}: arm base 0.652 up and 0.1 to +y of mobile_base with no yaw '
          '(== T_ab2mb (0, -0.100, -0.652) with the code\'s Rz(180 deg) body frame)')

print('== FK model vs the robot (2026-09-14, /arm/state at home)')
home = np.radians([-90.00087, -90.00022, 90.00348, -89.99956, -90.00044, 0.00044])
Mh = fk_flange(home)
real_home = np.array([-158.9886, 699.9907, 773.9619])
check(np.linalg.norm(Mh[:3, 3] * 1000 - real_home) < 1.0,
      f'URDF flange at home {np.round(Mh[:3, 3] * 1000, 1).tolist()} == reported TCP {real_home.tolist()} '
      '-> the active tool offset on the controller was ZERO')
check(np.linalg.norm(Mh[:3, 3] * 1000 + Mh[:3, :3] @ TIP - real_home) > 300,
      'the vision tip would be >300 mm from where the controller said the TCP was')

# ---------------------------------------------------------------- paired rows
ROWS = {
    106: dict(pose={'x': 0.003897, 'y': -0.284696, 'z': 0.385059, 'rx': -1.570665, 'ry': -0.029218, 'rz': 3.13708},
              joints=[-1.884632, -0.044301, 0.296601, -1.827833, -1.599976, -0.313975],
              msg=types.SimpleNamespace(x=0.0, y=1.71, theta=90.0)),        # zone B lane
    119: dict(pose={'x': 0.100377, 'y': 0.010087, 'z': 0.389159, 'rx': 1.57162, 'ry': -0.075189, 'rz': 3.130634},
              joints=[-1.685551, -0.338261, 0.961167, -2.191476, -1.646742, -0.115085],
              msg=types.SimpleNamespace(x=0.0, y=-1.71, theta=-90.0)),      # zone C lane
}


def ang_deg(Ra, Rb):
    return math.degrees(math.acos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


print('== transform_world_to_arm orientation convention vs FK of the paired joint row')
for gid, d in ROWS.items():
    Mq = fk_flange(d['joints'])
    for euler in ('ZYX', 'zyx'):
        pos, rpy = transform_world_to_arm(d['pose'], d['msg'], 0.0, euler=euler)
        Rc = R.from_euler('xyz', rpy, degrees=True).as_matrix()
        a = ang_deg(Rc, Mq[:3, :3])
        if euler == 'ZYX':
            check(a < 0.05, f'group {gid}: csv_euler ZYX reproduces the joint row orientation ({a:.3f} deg)')
            tip_fk = (Mq[:3, 3] + Mq[:3, :3] @ (TIP / 1000)) * 1000
            check(np.linalg.norm(np.asarray(pos) - tip_fk) < 1.0,
                  f'group {gid}: transform position == FK vision tip ({np.linalg.norm(np.asarray(pos) - tip_fk):.2f} mm)')
        else:
            check(a > 170, f'group {gid}: the old zyx reading is {a:.1f} deg off (tool pointing up)')
_blk = arm_transform.load_yaml_block('arm_calibration')
check(str(_blk.get('csv_euler')) == 'ZYX', f"robot.yaml csv_euler is ZYX ({_blk.get('csv_euler')!r})")
check(list(map(float, _blk.get('vision_tip_offset_mm', []))) == TIP.tolist(),
      'robot.yaml vision_tip_offset_mm matches set_tool_tcp.py')


# ---------------------------------------------------------------- _exec_pose end to end
class FakeRobot:
    def __init__(self, tcp_offset):
        self.tcp_offset = tcp_offset
        self.targets = []
        self.joints = [0.0] * 6
        self.pose = [0.0] * 6

    def GetTCPOffset(self, flag=1):
        return self.tcp_offset

    def GetInverseKinRef(self, t, target, q0):
        self.targets.append(list(target)); return 0, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    def GetInverseKin(self, t, target, config=-1):
        self.targets.append(list(target)); return 0, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    def MoveJ(self, joints, tool, user):
        return 0

    def GetActualTCPPose(self):
        return 0, list(self.pose)

    def GetActualJointPosDegree(self):
        return 0, list(self.joints)


def make(robot, msg):
    ac = ArmController.__new__(ArmController)
    ac.robot = robot
    ac.require_lift_height = False
    ac.lift_listener = types.SimpleNamespace(height_m=lambda: 0.0, topic='/lifter/height')
    ac.current_pose_msg = msg
    ac._live_lock = threading.Lock(); ac._live_pose = None; ac._live_joints = None; ac._live_stamp = 0.0
    ac._tip_offset_mm = TIP.copy()
    ac._pose_tip_to_flange = ac._probe_tool_frame()
    return ac


print('== _probe_tool_frame verdicts')
check(make(FakeRobot((0, [0, 0, 0, 0, 0, 0])), ROWS[106]['msg'])._pose_tip_to_flange is True,
      'offset 0 -> convert tip to flange (warned)')
check(any('FLANGE' in m for m in LOG['warn']), 'the flange verdict is logged as a warning')
check(make(FakeRobot((0, [0, -253.0, 225.2, 0, 0, 0])), ROWS[106]['msg'])._pose_tip_to_flange is False,
      'offset == vision tip -> send as is')
check(make(FakeRobot((0, [0, -100.0, 0, 0, 0, 0])), ROWS[106]['msg'])._pose_tip_to_flange is None,
      'some other offset -> None (pose mode refused)')
check(make(FakeRobot(112), ROWS[106]['msg'])._pose_tip_to_flange is None, 'bare int error -> None')
check(make(FakeRobot((0, [0, 0, 0, 30, 0, 0])), ROWS[106]['msg'])._pose_tip_to_flange is None,
      'zero translation but a rotated tool -> None')

print('== _exec_pose sends the FLANGE pose of the paired joint row when the active tool is the flange')
for gid, d in ROWS.items():
    robot = FakeRobot((0, [0, 0, 0, 0, 0, 0]))
    ac = make(robot, d['msg'])
    p = dict(d['pose']); p['q0'] = d['joints']
    ac._exec_pose(p)
    tgt = np.array(robot.targets[-1])
    Mq = fk_flange(d['joints'])
    dpos = np.linalg.norm(tgt[:3] - Mq[:3, 3] * 1000)
    dang = ang_deg(R.from_euler('xyz', tgt[3:], degrees=True).as_matrix(), Mq[:3, :3])
    check(dpos < 1.0 and dang < 0.05,
          f'group {gid}: IK target == URDF flange of the joint row ({dpos:.2f} mm, {dang:.3f} deg)')
    robot2 = FakeRobot((0, [0, -253.0, 225.2, 0, 0, 0]))
    ac2 = make(robot2, d['msg'])
    ac2._exec_pose(dict(p))
    tgt2 = np.array(robot2.targets[-1])
    check(np.linalg.norm(tgt2[:3] - tgt[:3] - Mq[:3, :3] @ TIP) < 1.0,
          f'group {gid}: with the tip active the target is the tip, exactly R @ (0,-253,225.2) further')
robot3 = FakeRobot((0, [0, -100.0, 0, 0, 0, 0]))
ac3 = make(robot3, ROWS[106]['msg'])
try:
    ac3._exec_pose(dict(ROWS[106]['pose']))
    check(False, 'unknown tool frame refuses pose mode')
except RuntimeError as e:
    check('refused' in str(e) and not robot3.targets, f'unknown tool frame refuses pose mode: {e}')

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
