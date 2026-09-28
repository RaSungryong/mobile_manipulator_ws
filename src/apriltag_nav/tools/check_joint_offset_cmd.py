#!/usr/bin/env python3
"""
check_joint_offset_cmd.py — offline check of the command-side joint-offset
correction (apriltag_nav.joint_offset_cmd, 2026-09-28).

Against the REAL planner URDF FK and the APPLIED offsets, with a numerical
IK standing in for the controller's: the pose the corrector commands must
put FK(IK(cmd) + dq) on the requested physical target; then the two
ArmController hooks against a fake Fairino.

    python3 src/apriltag_nav/tools/check_joint_offset_cmd.py
"""
import math
import os
import sys
import types

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, '..', 'src'))
sys.path.insert(0, os.path.join(_HERE, '..', '..', 'path_tag_locator', 'src'))

# rospy stub so ArmController's methods can run without a master
rospy = types.ModuleType('rospy')
LOG = dict(info=[], warn=[], err=[])
rospy.loginfo = lambda m, *a: LOG['info'].append(m % a if a else m)
rospy.logwarn = lambda m, *a: LOG['warn'].append(m % a if a else m)
rospy.logerr = lambda m, *a: LOG['err'].append(m % a if a else m)
rospy.logwarn_throttle = lambda t, m, *a: LOG['warn'].append(m)
rospy.logerr_throttle = lambda t, m, *a: LOG['err'].append(m)
rospy.get_param = lambda k, d=None: d
rospy.sleep = lambda s: None
rospy.Time = types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_sec=lambda: 0.0))
sys.modules['rospy'] = rospy

from apriltag_nav import joint_offset_cmd as JC                     # noqa: E402
from apriltag_nav import tf_chain as TC                              # noqa: E402
from apriltag_nav.arm_fk import ArmChain                             # noqa: E402
from path_tag_locator.geometry import pose_fr5_to_matrix_m, matrix_m_to_pose_fr5  # noqa: E402

N_OK = N_FAIL = 0


def check(cond, what, detail=''):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print('  ok   %s%s' % (what, ('  [%s]' % detail) if detail else ''))
    else:
        N_FAIL += 1; print('  FAIL %s%s' % (what, ('  [%s]' % detail) if detail else ''))


chain = ArmChain()
dq = TC.load_joint_offsets()
print('applied offsets:', np.round(dq, 3).tolist())


def numeric_ik(fk, pose, seed, iters=40):
    """Damped least squares on the URDF FK — the controller's IK stand-in
    (its FK is this FK to 0.01 mm, check_pose_vs_joint 2026-09-14)."""
    T_t = JC.pose_to_T(pose)
    q = np.asarray(seed, float).copy()
    for _ in range(iters):
        T = fk(q)
        E = JC._inv(T) @ T_t
        rv = _rotvec(E[:3, :3])
        err = np.concatenate([T[:3, :3] @ E[:3, 3], T[:3, :3] @ rv])   # in the base frame
        if np.linalg.norm(err[:3]) < 1e-7 and np.linalg.norm(err[3:]) < 1e-7:
            break
        J = np.zeros((6, 6)); h = 1e-5
        for k in range(6):
            qk = q.copy(); qk[k] += math.degrees(h)
            Tk = fk(qk); Ek = JC._inv(T) @ Tk
            J[:, k] = np.concatenate([T[:3, :3] @ Ek[:3, 3], T[:3, :3] @ _rotvec(Ek[:3, :3])]) / h
        dqs = np.linalg.solve(J.T @ J + 1e-9 * np.eye(6), J.T @ err)
        q += np.degrees(dqs)
    return q


def _rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2)); a = math.acos(c)
    if a < 1e-12:
        return np.zeros(3)
    return a / (2 * math.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


print('\n== 1. pose conversions agree with path_tag_locator.geometry ==')
rng = np.random.RandomState(3)
worst = 0.0
for _ in range(200):
    p = list(rng.uniform(-800, 800, 3)) + list(rng.uniform(-170, 170, 3))
    worst = max(worst, float(np.abs(JC.pose_to_T(p) - pose_fr5_to_matrix_m(p)).max()))
    back = JC.T_to_pose(JC.pose_to_T(p)); ref = matrix_m_to_pose_fr5(pose_fr5_to_matrix_m(p))
    worst = max(worst, float(np.abs(np.array(back) - np.array(ref)).max()))
check(worst < 1e-9, 'pose_to_T / T_to_pose == geometry.pose_fr5_to_matrix_m / matrix_m_to_pose_fr5', '%.1e' % worst)

print('\n== 2. the corrected command lands the PHYSICAL flange on the target (real URDF, applied dq) ==')
corr = JC.CommandCorrector(dq, chain.fk_flange)
check(corr.enabled, 'corrector enabled with the applied offsets')
# configurations: the 2026-09-28 sheet views if the session is on disk, else the home pose
sess = os.path.join(TC.WS_DIR if hasattr(TC, 'WS_DIR') else os.path.join(_HERE, '..', '..', '..'),
                    'log', 'chain_calib', '20260928_sheet_auto', 'samples.npz')
if os.path.exists(sess):
    Q = np.load(sess)['joints'][:12]
else:
    Q = np.array([[-90, -90, 90, -90, -90, 0.0]])
res_mm, res_deg, corr_mm, corr_deg = [], [], [], []
for q_true in Q:
    q_true = np.asarray(q_true, float)
    target = JC.T_to_pose(chain.fk_flange(q_true, dq))          # where the physical flange must go
    ik = lambda pose, s=q_true: numeric_ik(chain.fk_flange, pose, s)
    cmd, info = corr.pose_for_physical(target, ik)
    q_cmd = ik(cmd)                                              # what the controller would do
    T_phys = chain.fk_flange(q_cmd, dq)                          # where the physical flange then is
    E = JC._inv(JC.pose_to_T(target)) @ T_phys
    res_mm.append(np.linalg.norm(E[:3, 3]) * 1e3); res_deg.append(math.degrees(np.linalg.norm(_rotvec(E[:3, :3]))))
    corr_mm.append(info['corr_mm']); corr_deg.append(info['corr_deg'])
check(max(res_mm) < 0.05 and max(res_deg) < 0.005,
      'FK(IK(cmd) + dq) == target over %d configurations' % len(Q), 'worst %.4f mm / %.5f deg' % (max(res_mm), max(res_deg)))
check(8.0 < np.mean(corr_mm) < 16.0 and 0.9 < np.mean(corr_deg) < 1.2,
      'the correction is the 9-12 mm / ~1.06 deg the Work Log measured at these views',
      'mean %.1f mm / %.2f deg' % (np.mean(corr_mm), np.mean(corr_deg)))
check(all(i == 2 for i in [info['iters']]), 'two IK evaluations per target')
# uncorrected, the physical flange misses by that much — the defect being fixed
miss = []
for q_true in Q:
    target = JC.T_to_pose(chain.fk_flange(np.asarray(q_true, float), dq))
    q_naive = numeric_ik(chain.fk_flange, target, q_true)
    E = JC._inv(JC.pose_to_T(target)) @ chain.fk_flange(q_naive, dq)
    miss.append(np.linalg.norm(E[:3, 3]) * 1e3)
check(min(miss) > 5.0, 'without the correction the physical flange misses by >= 5 mm', 'min %.1f max %.1f mm' % (min(miss), max(miss)))

print('\n== 3. edge cases ==')
z = JC.CommandCorrector(np.zeros(6), chain.fk_flange)
cmd, info = z.pose_for_physical([100, 200, 300, 180, 0, 90], lambda p: [0] * 6)
check(not z.enabled and cmd == [100, 200, 300, 180, 0, 90] and not info['applied'], 'dq = 0: the target is sent as is')
check(JC.CommandCorrector(dq, None).enabled is False, 'no FK: disabled')
try:
    corr.pose_for_physical([100, 200, 300, 180, 0, 90], lambda p: None); check(False, 'IK failure raises')
except ValueError as e:
    check('IK failed' in str(e), 'IK failure raises ValueError', str(e)[:50])
big = JC.CommandCorrector([0, 4.0, 4.0, 0, 0, 0], chain.fk_flange)
try:
    big.pose_for_physical(JC.T_to_pose(chain.fk_flange(Q[0])), lambda p: numeric_ik(chain.fk_flange, p, Q[0]))
    check(False, 'implausible correction refused')
except ValueError as e:
    check('plausibility' in str(e), 'an implausibly large correction (4 deg offsets) is refused', str(e)[:60])
q = [-90, -90, 90, -90, -90, 0]
check(np.allclose(corr.joints_for_physical(q), np.array(q) - dq), 'joints_for_physical = q - dq')
check(z.joints_for_physical(q) == q, '… and q itself with dq = 0')
loaded = JC.load_default(warn=print)
check(loaded.enabled and np.allclose(loaded.dq, dq), 'load_default reads config/tf/arm_joint_offsets.yaml + the URDF')

print('\n== 4. ArmController hooks against a fake Fairino ==')
from apriltag_nav.arm_controller import ArmController                  # noqa: E402
import apriltag_nav.arm_controller as _acmod                            # noqa: E402
_acmod.rospy = rospy


class FakeRobot:
    def __init__(self, q_ik):
        self.q_ik = list(q_ik); self.calls = []; self.ik_fail = False
    def GetInverseKin(self, t, target, config=-1):
        self.calls.append(('IK', [round(v, 3) for v in target]))
        return 112 if self.ik_fail else (0, list(self.q_ik))
    def MoveL(self, target, tool, user, **kw):
        self.calls.append(('MoveL', [round(v, 3) for v in target])); return 0
    def MoveCart(self, target, tool, user, *a, **kw):
        self.calls.append(('MoveCart', [round(v, 3) for v in target])); return 0


q_view = list(Q[0])
robot = FakeRobot(q_view)
ac = ArmController.__new__(ArmController)
ac.robot = robot; ac.busy = False; ac.cancel_requested = False; ac._cmd_corr = corr
target = JC.T_to_pose(chain.fk_flange(np.asarray(q_view), dq))
ok, msg = ac.move_cart(target, physical=True)
sent = [c for c in robot.calls if c[0] == 'MoveL'][-1][1]
expect = corr.pose_for_physical(target, lambda p: q_view)[0]
check(ok and msg == 'move_cart ok' and np.allclose(sent, expect, atol=1e-3), 'move_cart(physical=True) sends the corrected pose',
      'shift %.1f mm' % (np.linalg.norm(np.array(sent[:3]) - np.array(target[:3]))))
check(np.linalg.norm(np.array(sent[:3]) - np.array(target[:3])) > 5.0, '… which differs from the target by the offsets\' effect')
check(sum(1 for c in robot.calls if c[0] == 'IK') == 2, '… after two IK calls')
robot.calls.clear()
ok, msg = ac.move_cart(target, physical=False)
check(ok and [c for c in robot.calls if c[0] == 'MoveL'][-1][1] == [round(v, 3) for v in target]
      and not any(c[0] == 'IK' for c in robot.calls), 'move_cart(physical=False) sends the target unchanged, no IK')
robot.calls.clear()
ok, msg = ac.move_cart(target, physical=True, linear=False)
check(ok and robot.calls[-1][0] == 'MoveCart' and np.allclose(robot.calls[-1][1], expect, atol=1e-3), 'MoveCart path corrected too')
robot.ik_fail = True; robot.calls.clear()
ok, msg = ac.move_cart(target, physical=True)
check(not ok and 'move_cart failed' in msg and 'joint-offset' in msg and not any(c[0] in ('MoveL', 'MoveCart') for c in robot.calls),
      'IK failure refuses the move (no motion), result string keeps "move_cart failed"', msg[:60])
check(ac.busy is False, 'busy released after the refusal')
ac._cmd_corr = None; robot.ik_fail = False; robot.calls.clear()
ok, msg = ac.move_cart(target, physical=True)
check(ok and robot.calls[-1][1] == [round(v, 3) for v in target] and any('not loaded' in w for w in LOG['warn']),
      'no corrector loaded: physical target sent uncorrected with a warning')

print('\n== 5. skip_z: xy (+ rotation) corrected, the commanded arm-frame z kept (the mode in use) ==')
corr_xy = JC.CommandCorrector(dq, chain.fk_flange, skip_z=True)
check('xy only' in corr_xy.describe() and 'full 6-DOF' in corr.describe(), 'describe() names the mode')
xy_res, z_miss, dz_sk, rot_res, xy_corr = [], [], [], [], []
for q_true in Q:
    q_true = np.asarray(q_true, float)
    target = JC.T_to_pose(chain.fk_flange(q_true, dq))
    ik = lambda pose, s=q_true: numeric_ik(chain.fk_flange, pose, s)
    cmd, info = corr_xy.pose_for_physical(target, ik)
    check_z = (cmd[2] == target[2])
    q_cmd = ik(cmd)
    T_phys = chain.fk_flange(q_cmd, dq); T_tgt = JC.pose_to_T(target)
    d = (T_phys[:3, 3] - T_tgt[:3, 3]) * 1e3                       # arm base frame, mm
    xy_res.append(float(np.hypot(d[0], d[1]))); z_miss.append(float(d[2])); dz_sk.append(info['dz_skipped_mm'])
    rot_res.append(math.degrees(np.linalg.norm(_rotvec(T_tgt[:3, :3].T @ T_phys[:3, :3]))))
    xy_corr.append(info['corr_mm'])
    if not check_z:
        break
check(check_z, 'the commanded z equals the target z exactly (bit for bit)')
check(max(xy_res) < 0.05, 'FK(IK(cmd) + dq) lands on the target xy over %d configurations' % len(Q), 'worst %.4f mm' % max(xy_res))
check(max(rot_res) < 0.005, '… with the target orientation (the rotation IS corrected)', 'worst %.5f deg' % max(rot_res))
check(all(2.0 < abs(v) < 16.0 for v in dz_sk), 'the dropped z term is the offsets\' z (2-16 mm at these views)', 'dz %.1f .. %.1f mm' % (min(dz_sk), max(dz_sk)))
check(all(abs(m + s) < 0.1 for m, s in zip(z_miss, dz_sk)), 'the physical z then misses by exactly the dropped term',
      'worst %.3f mm' % max(abs(m + s) for m, s in zip(z_miss, dz_sk)))
check(all(info.get('skip_z') for info in [info]) and 3.0 < np.mean(xy_corr) < 16.0,
      'info carries skip_z / dz_skipped_mm; corr_mm is the xy part applied', 'mean xy corr %.1f mm' % np.mean(xy_corr))
# joint-target form
worst_j, worst_same = 0.0, 0.0
for q_true in Q[:6]:
    q_true = np.asarray(q_true, float)
    target = JC.T_to_pose(chain.fk_flange(q_true, dq))
    ik = lambda pose, s=q_true: numeric_ik(chain.fk_flange, pose, s)
    qj, info = corr_xy.joints_for_physical_pose(target, ik)
    d = (chain.fk_flange(np.asarray(qj), dq)[:3, 3] - JC.pose_to_T(target)[:3, 3]) * 1e3
    worst_j = max(worst_j, float(np.hypot(d[0], d[1])))
    qf, _ = corr.joints_for_physical_pose(target, ik)              # full 6-DOF, pose form
    worst_same = max(worst_same, float(np.abs(np.asarray(qf) - np.asarray(corr.joints_for_physical(ik(target)))).max()))
check(worst_j < 0.05, 'joints_for_physical_pose (skip_z): FK(q + dq) xy on target', 'worst %.4f mm' % worst_j)
check(worst_same < 0.01, 'joints_for_physical_pose without skip_z == q - dq (to the IK\'s precision)', 'worst %.4f deg' % worst_same)
check(JC.load_default(skip_z=True).skip_z is True and JC.load_default().skip_z is False, 'load_default(skip_z=) passes the flag')
try:
    corr_xy.joints_for_physical_pose([100, 200, 300, 180, 0, 90], lambda p: None); check(False, 'IK failure raises')
except ValueError as e:
    check('IK failed' in str(e), 'joints_for_physical_pose: IK failure raises ValueError', str(e)[:50])

print('\n== 6. ArmController with the skip_z corrector ==')
robot = FakeRobot(q_view); ac.robot = robot; ac._cmd_corr = corr_xy
target = JC.T_to_pose(chain.fk_flange(np.asarray(q_view), dq))
ok, msg = ac.move_cart(target, physical=True)
sent = [c for c in robot.calls if c[0] == 'MoveL'][-1][1]
check(ok and sent[2] == round(target[2], 3) and np.hypot(sent[0] - target[0], sent[1] - target[1]) > 3.0,
      'move_cart(physical=True): z sent as the target\'s, xy shifted', 'xy shift %.1f mm' % np.hypot(sent[0] - target[0], sent[1] - target[1]))
check(any('z kept' in m for m in LOG['info']), 'the log line says z was kept')


class FakeRobotIK(FakeRobot):
    """IK = the numerical IK on the URDF seeded with the caller's q0, so
    _exec_pose's MoveJ can be checked against the FK."""
    def GetInverseKinRef(self, t, target, q0):
        self.calls.append(('IKref', [round(v, 3) for v in target]))
        return 0, [float(v) for v in numeric_ik(chain.fk_flange, list(target), q0)]
    def MoveJ(self, joints, tool, user, **kw):
        self.calls.append(('MoveJ', [float(v) for v in joints])); return 0


robot = FakeRobotIK(q_view); ac.robot = robot
ac.lift_listener = types.SimpleNamespace(height_m=lambda: 0.0, topic='/lifter/height')
ac.require_lift_height = False; ac._map_tags = {}
ac.current_pose_msg = types.SimpleNamespace(x=0.0003, y=1.7003, theta=89.81, id=106)
ac._pose_tip_to_flange = False; ac._tip_offset_mm = np.array([-2.0, -245.2, 214.4])
ac._refresh_live_pose = lambda: None
LOG['info'].clear()
ac._exec_pose({'mode': 'pose', 'x': -0.6, 'y': 0.0, 'z': 0.082 + 0.3, 'rx': 3.1416, 'ry': 0.0, 'rz': 0.0,
               'q0': [math.radians(v) for v in q_view], 'point_id': 1, 'group_id': 106})
ik_target = next(m for m in LOG['info'] if m.startswith('[Arm REAL] IK target:'))
tgt = [float(v) for v in ik_target.split('[')[-1].rstrip(']').split(',')]
mj = [c for c in robot.calls if c[0] == 'MoveJ'][-1][1]
d = (chain.fk_flange(np.asarray(mj), dq)[:3, 3] - JC.pose_to_T(tgt)[:3, 3]) * 1e3
d_naive = (chain.fk_flange(np.asarray(mj))[:3, 3] - JC.pose_to_T(tgt)[:3, 3]) * 1e3
check(np.hypot(d[0], d[1]) < 0.05, '_exec_pose: the MoveJ joints put the PHYSICAL flange (FK(q+dq)) on the IK target xy',
      '%.4f mm; the controller\'s own FK(q) is %.1f mm off it' % (np.hypot(d[0], d[1]), np.hypot(d_naive[0], d_naive[1])))
check(2.0 < abs(d[2]) < 16.0 and any('xy only, z kept' in m for m in LOG['info']),
      '… while its z is left uncorrected by the offsets\' z term', 'dz %.1f mm' % d[2])
check(sum(1 for c in robot.calls if c[0] == 'IKref') == 4, 'four IK evaluations: seed, two correction iterations, final')

print('\n%d ok, %d failed' % (N_OK, N_FAIL))
sys.exit(1 if N_FAIL else 0)
