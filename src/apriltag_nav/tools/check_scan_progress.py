#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline checks of ArmController's scan loop failure handling and the
/arm/scan_progress events (2026-09-14).

Stubs rospy, the message packages, the Fairino SDK import and the capture
pipeline; builds the REAL ArmController without its __init__ (no robot, no
model) and runs execute_scan_points against a fake Fairino whose IK returns
a bare int error code — the real SDK's behaviour when there is no solution,
which used to surface as "cannot unpack non-iterable int object".

Run with a sourced workspace:  python3 tools/check_scan_progress.py
"""
import json
import os
import sys
import threading
import time
import types

import numpy as np

# ---------------------------------------------------------------- stubs
LOG = {'info': [], 'warn': [], 'err': []}
PUBLISHED = []          # (topic, data)


class _Pub:
    def __init__(self, topic, *a, **k):
        self.topic = topic

    def publish(self, msg):
        PUBLISHED.append((self.topic, getattr(msg, 'data', msg)))


rospy = types.ModuleType('rospy')
rospy.loginfo = lambda m, *a: LOG['info'].append(str(m))
rospy.logwarn = lambda m, *a: LOG['warn'].append(str(m))
rospy.logerr = lambda m, *a: LOG['err'].append(str(m))
rospy.logdebug = lambda m, *a: None
rospy.logwarn_throttle = lambda t, m, *a: LOG['warn'].append(str(m))
rospy.logerr_throttle = lambda t, m, *a: LOG['err'].append(str(m))
rospy.loginfo_throttle = lambda t, m, *a: None
rospy.get_param = lambda name, default=None: default
rospy.get_time = lambda: time.time()
rospy.Publisher = _Pub
rospy.Subscriber = lambda *a, **k: None
rospy.is_shutdown = lambda: False
sys.modules['rospy'] = rospy


class _Msg:
    def __init__(self, data=None):
        self.data = data


std_msgs = types.ModuleType('std_msgs'); std_msgs_msg = types.ModuleType('std_msgs.msg')
for n in ('Bool', 'Float32', 'String'):
    setattr(std_msgs_msg, n, type(n, (_Msg,), {}))
sys.modules['std_msgs'] = std_msgs; sys.modules['std_msgs.msg'] = std_msgs_msg
robot_msgs = types.ModuleType('robot_msgs'); robot_msgs_msg = types.ModuleType('robot_msgs.msg')
robot_msgs_msg.Pose2DWithFlag = type('Pose2DWithFlag', (), {})
sys.modules['robot_msgs'] = robot_msgs; sys.modules['robot_msgs.msg'] = robot_msgs_msg
fairino = types.ModuleType('fairino'); fairino.Robot = types.SimpleNamespace(RPC=object)
sys.modules['fairino'] = fairino


class FakePipeline:
    def __init__(self):
        self.captured = []
        self.processed_on = []
        self.preopened = 0
        self.released = 0

    def preopen(self):
        self.preopened += 1

    def release(self):
        self.released += 1

    def capture(self, point_id, cancelled):
        self.captured.append(point_id)
        return [np.zeros((4, 4), dtype=np.uint8)]

    def process(self, point_id, frames, cancelled=None, name_prefix=None):
        # Runs on the controller's inference worker thread.
        self.processed_on.append(threading.current_thread().name)
        self.prefixes = getattr(self, 'prefixes', []) + [name_prefix]
        return {'ra_mean': 0.4, 'ra_std': 0.0, 'ra_min': 0.4, 'ra_max': 0.4,
                'num_samples': 1}


sp = types.ModuleType('apriltag_nav.scan_pipeline'); sp.RaScanPipeline = FakePipeline
sys.modules['apriltag_nav.scan_pipeline'] = sp

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))
from apriltag_nav.arm_controller import ArmController, TOOL_ID   # noqa: E402
from apriltag_nav.scan_results import ScanResultWriter            # noqa: E402

N_OK = N_FAIL = 0
import tempfile                                                      # noqa: E402
COLLECT_DIR = tempfile.mkdtemp(prefix='ra_measured_')


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


class FakeRobot:
    """IK: bare int 112 when the target is beyond `reach_mm`, else (0, joints).
    MoveJ: 0, or `movej_error` when set."""
    def __init__(self, reach_mm=1400.0):
        self.reach_mm = reach_mm
        self.movej_error = 0
        self.calls = []
        self.pose = [100.0, 200.0, 300.0, 180.0, 0.0, 90.0]
        self.joints = [-90.0, -90.0, 90.0, -90.0, -90.0, 0.0]

    def SetSpeed(self, v):
        self.calls.append(('SetSpeed', v)); return 0

    def _ik(self, target):
        r = (target[0] ** 2 + target[1] ** 2) ** 0.5
        if r > self.reach_mm:
            return 112                       # <- the SDK's bare int
        return 0, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    def GetInverseKinRef(self, t, target, q0):
        self.calls.append(('IKref', tuple(round(v, 1) for v in target))); return self._ik(target)

    def GetInverseKin(self, t, target, config=-1):
        self.calls.append(('IK', tuple(round(v, 1) for v in target))); return self._ik(target)

    def MoveJ(self, joints, tool, user, **kw):
        self.calls.append(('MoveJ', tool, tuple(round(v, 1) for v in joints)))
        self.movej_kw = dict(kw)
        if self.movej_error:
            return self.movej_error
        self.joints = list(joints)
        self.pose = [self.pose[0] + 1.0] + self.pose[1:]
        return 0

    def MoveL(self, pose, tool, user, **kw):
        self.calls.append(('MoveL', tool, tuple(round(v, 2) for v in pose)))
        self.pose = [float(v) for v in pose]
        return 0

    def GetActualTCPPose(self):
        return 0, list(self.pose)

    def GetActualJointPosDegree(self):
        return 0, list(self.joints)


def make_controller(robot):
    ac = ArmController.__new__(ArmController)
    ac.robot = robot
    ac.busy = False
    ac.cancel_requested = False
    ac.results_writer = ScanResultWriter()
    ac.pipeline = FakePipeline()
    ac.stabilization_time = 0.0
    ac.keyence_require_converged = False
    ac.require_lift_height = False
    ac.lift_listener = types.SimpleNamespace(height_m=lambda: 0.0, topic='/lifter/height')
    ac.current_pose_msg = types.SimpleNamespace(x=-0.000, y=1.711, theta=90.0)
    ac._tip_offset_mm = np.array([0.0, -253.0, 225.2])
    ac._pose_tip_to_flange = False      # targets go to IK as tip coordinates here
    ac.progress_pub = _Pub('/arm/scan_progress')
    ac.done_pub = _Pub('/scan_finished')
    ac._live_lock = threading.Lock()
    ac._live_pose = None; ac._live_joints = None; ac._live_stamp = 0.0
    ac._adjust_distance_to_surface = lambda: None
    # Ra collect mode (2026-10-06): off unless a scenario turns it on.
    ac.collect_enabled = False
    ac.collect_retreat_mm = 80.0
    ac.collect_retreat_speed = 10
    ac.collect_wait_timeout_s = 0.0
    ac.collect_record_dir = COLLECT_DIR
    ac.collect_mode = 'pause'
    ac.collect_batch_size = 1
    ac.collect_mark_retreat_mm = 30.0
    ac.collect_mark_dwell_s = 0.0
    ac._collect_batch = []
    ac._collect_mark_no = 0
    ac._collect_run_stem = ''
    ac.keyence_dir = -1.0
    ac.collect_state_pub = _Pub('/arm/collect_state')
    ac._collect_event = threading.Event()
    ac._collect_payload = None
    ac._collect_lock = threading.Lock()
    ac._collect = {'waiting': False, 'n_recorded': 0, 'n_skipped': 0, 'record_csv': ''}
    ac.homed = []
    ac.move_to_home = lambda: ac.homed.append(1)
    ac.last_results = None
    def _remember(csv_path, results, _ac=ac):
        _ac.last_results = results
    ac._save_results = _remember
    return ac


def events(with_result=False):
    """Progress events in publish order. `result` events come from the
    inference worker and can interleave with the next point's `move`, so
    the sequence checks drop them and check them separately."""
    ev = [json.loads(d) for t, d in PUBLISHED if t == '/arm/scan_progress']
    return ev if with_result else [e for e in ev if e['phase'] != 'result']


def pose_pt(pid, x, y, z=0.3851, q0=True):
    p = {'mode': 'pose', 'x': x, 'y': y, 'z': z, 'rx': 0.0001, 'ry': -0.0292,
         'rz': 3.1371, 'speed': 30, 'point_id': pid, 'group_id': 106,
         'csv_path': '', 'scan': True}
    if q0:
        p['q0'] = [-1.9, -0.1, 0.5, -1.9, -1.55, -0.2]
    return p


def joint_pt(pid, scan=True, n=6):
    return {'mode': 'joint', 'joints': [-1.5708, -1.4, 1.5, -1.6, -1.56, -0.03][:n],
            'speed': 10, 'point_id': pid, 'group_id': 106, 'csv_path': '',
            'scan': scan}


# ================================================================ scenario 1
print('== the robot run of 2026-09-14 13:20: errorX group 106 at tag 106')
PUBLISHED.clear(); LOG['err'].clear()
robot = FakeRobot()
ac = make_controller(robot)
far = pose_pt(1, 0.2847, 1.7139)           # the CSV's first row, verbatim
near = pose_pt(2, -1.30, 0.20)             # 0.37 m from the arm base
ac.execute_scan_points([far, near])
ev = events()
failed = [e for e in ev if e['phase'] == 'failed']
done = [e for e in ev if e['phase'] == 'done']
check(len(failed) == 1 and failed[0]['point_id'] == 1, 'the far point fails, once')
msg = failed[0]['message']
check(msg.startswith('IK failed (code 112): tip target'), f'reason is the IK code, not a TypeError: {msg[:40]}')
check('unpack' not in msg, 'no "cannot unpack" any more')
# The arm-frame numbers depend on robot.yaml arm_calibration (CALIBRATED since
# 2026-09-21, so no longer the design-mount (-1714, 1896, -267) / 2.56 m): derive
# them the way _exec_pose does and check the message quotes THOSE.
import re as _re
_m = _re.search(r'\((-?\d+), (-?\d+), (-?\d+)\) mm', msg); _d = _re.search(r'(\d+\.\d+) m from the arm base', msg)
_ok = False
if _m and _d:
    _t = np.array([float(v) for v in _m.groups()])
    _ok = 2.3 < float(_d.group(1)) < 2.8 and abs(np.linalg.norm(_t) / 1000.0 - float(_d.group(1))) < 0.02 \
        and np.linalg.norm(_t - np.array([-1714.0, 1896.0, -267.0])) < 80.0
check(_ok, f'message names the arm-frame target and its distance from the base ({_m.group(0) if _m else "?"}, '
           f'{_d.group(0) if _d else "?"}; design-mount value was (-1714, 1896, -267) / 2.56 m, '
           f'the calibrated mount moves it a few cm)')
check('world (0.285, 1.714, 0.385)' in msg and 'robot pose (-0.000, 1.711, 90.0 deg)' in msg,
      'message names the CSV world point and the robot pose it was transformed at')
check(not any(c[0] == 'MoveJ' and c[1] == TOOL_ID and c[2][0] == 1.0 for c in robot.calls[:3]),
      'no MoveJ was sent for the unreachable point')
check(ac.pipeline.captured == [2], 'capture ran only for the reachable point')
check(len(done) == 1 and done[0]['point_id'] == 2 and 'ra_mean' not in done[0]
      and done[0]['message'] == 'Success', 'the reachable point is done as soon as the frames are in hand')
res = [e for e in events(True) if e['phase'] == 'result']
check(len(res) == 1 and res[0]['point_id'] == 2 and abs(res[0]['ra_mean'] - 0.4) < 1e-9
      and res[0]['index'] == 2, 'its Ra arrives in a result event from the worker')
allev = events(True)
check(allev.index(res[0]) > allev.index(done[0]) and allev[-1]['phase'] == 'finished',
      'result comes after done and before finished (worker drained before the end)')
check(ac.pipeline.processed_on == ['scan-infer'], 'inference ran on the scan-infer thread, not the arm thread')
phases = [e['phase'] for e in ev]
check(phases == ['start', 'move', 'failed', 'move', 'done', 'finished'],
      f'event sequence {phases}')
fin = ev[-1]
check(fin['n_ok'] == 1 and fin['n_fail'] == 1 and fin['total'] == 2 and fin['cancelled'] is False,
      'finished carries the counts')
check(ev[0]['total'] == 2 and ev[0]['scan_points'] == 2, 'start carries the totals')
check(all('tcp_pose' in e for e in ev[4:]) and 'tcp_pose' not in ev[2],
      'events after the first successful move carry the live pose; the failed one has none yet')
lp = ac.live_pose()
check(lp is not None and lp[2] < 5.0 and lp[0][0] == 101.0, 'live_pose() is the worker snapshot after MoveJ')
check(any(t == '/scan_finished' for t, _ in PUBLISHED) and ac.homed == [1],
      '/scan_finished published and the arm homed at the end')
check(ac.last_results[1]['ra_mean'] == 0.4 and ac.last_results[1]['num_samples'] == 1
      and ac.last_results[0]['ra_mean'] is None,
      'the result row was filled in by the worker before execute_scan_points returned')
check(any('IK failed (code 112)' in m for m in LOG['err']), 'rosout error carries the same reason')

# ================================================================ scenario 2
print('== joint points: MoveJ failure, traverse rows, bad length')
PUBLISHED.clear()
robot = FakeRobot(); ac = make_controller(robot)
robot.movej_error = 3
ac.execute_scan_points([joint_pt(5)])
f = [e for e in events() if e['phase'] == 'failed']
check(len(f) == 1 and f[0]['message'] == 'MoveJ failed (code 3)', 'a MoveJ error fails the point with its code')
check(ac.pipeline.captured == [], 'no capture after a failed move (used to capture anyway)')
PUBLISHED.clear()
robot = FakeRobot(); ac = make_controller(robot)
ac.execute_scan_points([joint_pt(1, scan=False), joint_pt(2, scan=True), joint_pt(3, n=5)])
ev = events()
check([e['phase'] for e in ev] == ['start', 'move', 'done', 'move', 'done', 'move', 'failed', 'finished'],
      f'traverse -> done(traverse), work -> done, bad row -> failed: {[e["phase"] for e in ev]}')
check(ev[2]['scan'] is False and ev[2]['message'] == 'traverse' and 'ra_mean' not in ev[2],
      'traverse row reports done without a measurement')
check(ac.pipeline.captured == [2], 'only the work point was captured')
check(ev[6]['message'] == 'joint goal must have 6 values', 'a 5-value joint row fails loudly')
check(ev[-1]['n_ok'] == 2 and ev[-1]['n_fail'] == 1, 'counts: traverse counts as ok')

# ================================================================ scenario 2b
print('== the post-standoff settle only when the standoff loop moved')
import apriltag_nav.arm_controller as _acmod
from apriltag_nav.keyence_standoff import StandoffResult
slept = []
_real_sleep = _acmod.time.sleep
_acmod.time.sleep = lambda s: slept.append(s)
try:
    robot = FakeRobot(); ac = make_controller(robot)
    ac._adjust_distance_to_surface = lambda: StandoffResult(False, 'out of range', steps=0, travel_mm=0.0)
    ac.execute_scan_points([joint_pt(1)])
    check(0.5 not in slept, f'no 0.5 s settle when the standoff loop took no step: {slept}')
    slept.clear()
    robot = FakeRobot(); ac = make_controller(robot)
    ac._adjust_distance_to_surface = lambda: StandoffResult(True, 'ok', steps=2, travel_mm=1.4)
    ac.execute_scan_points([joint_pt(1)])
    check(0.5 in slept, f'0.5 s settle kept when the tool was moved: {slept}')
finally:
    _acmod.time.sleep = _real_sleep

# ================================================================ scenario 2c
print('== a failing inference is recorded, not fatal; the next point still gets its Ra')
PUBLISHED.clear()
robot = FakeRobot(); ac = make_controller(robot)
calls = []
def _flaky(point_id, frames, cancelled=None, name_prefix=None):
    calls.append(point_id)
    if point_id == 1:
        raise RuntimeError('onnx exploded')
    return {'ra_mean': 0.7, 'ra_std': 0.0, 'ra_min': 0.7, 'ra_max': 0.7, 'num_samples': 1}
ac.pipeline.process = _flaky
ac.execute_scan_points([joint_pt(1), joint_pt(2)])
res = [e for e in events(True) if e['phase'] == 'result']
check(calls == [1, 2] and len(res) == 2, 'worker survived the raise and processed both points in order')
check(res[0]['ra_mean'] is None and res[0]['message'].endswith('(no Ra)')
      and res[1]['ra_mean'] == 0.7, 'the failed one carries no Ra and says so; the next one has its value')
check(events()[-1]['phase'] == 'finished' and events()[-1]['n_ok'] == 2,
      'the point itself still counts as measured (the capture succeeded)')

# ================================================================ scenario 3
print('== _ik_result normalisation')
check(ArmController._ik_result(112) == (112, None), 'bare int -> (code, None)')
check(ArmController._ik_result((0, [1, 2, 3, 4, 5, 6])) == (0, [1, 2, 3, 4, 5, 6]), 'tuple passes through')
check(ArmController._ik_result([0, [1] * 6]) == (0, [1] * 6), 'list passes through')
check(ArmController._ik_result(None) == (-1, None), 'None -> (-1, None)')

# ================================================================ scenario 4
print('== unseeded pose point uses GetInverseKin and still fails cleanly')
PUBLISHED.clear()
robot = FakeRobot(); ac = make_controller(robot)
ac.execute_scan_points([pose_pt(7, 0.2847, 1.7139, q0=False)])
check(any(c[0] == 'IK' for c in robot.calls) and not any(c[0] == 'IKref' for c in robot.calls),
      'GetInverseKin path taken without a seed')

# 2026-09-28: pose-mode targets are world points, so the joint zero offsets
# are applied on the command side — MoveJ receives IK joints minus dq.
from apriltag_nav.joint_offset_cmd import CommandCorrector
robot = FakeRobot(); ac = make_controller(robot)
ac._cmd_corr = CommandCorrector([0.0, -0.341, -0.529, -0.051, -0.134, -0.496], fk=None)
ac.execute_scan_points([pose_pt(8, -1.30, 0.20)])
check(robot.calls[-1] == ('MoveJ', TOOL_ID, (1.0, 2.3, 3.5, 4.1, 5.1, 6.5)),
      f'pose mode commands IK joints minus the joint offsets: {robot.calls[-1]}')
robot = FakeRobot(); ac = make_controller(robot)
ac._cmd_corr = CommandCorrector([0.0] * 6, fk=None)
ac.execute_scan_points([pose_pt(9, -1.30, 0.20)])
check(robot.calls[-1] == ('MoveJ', TOOL_ID, (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)), 'zero offsets: IK joints sent as is')
check(events()[2]['phase'] == 'failed' and events()[2]['message'].startswith('IK failed (code 112)'),
      'same failure report on that path')

# ---------------------------------------------------------------- standoff assist (2026-09-15)
print('== distance-sensor assist: standoff_state / keyence_cb / adjust_standoff')
robot = FakeRobot()
ac = make_controller(robot)
ac._keyence_cos = float(np.cos(np.radians(42.6)))
ac.keyence_invalid_abs_mm = 90.0
ac.keyence_sensor_zero_mm = 10.0
ac.keyence_target_distance_mm = 10.0
ac.keyence_setpoint_mm = 0.0
ac._keyence_lock = threading.Lock()
ac.current_keyence_val = None
ac._keyence_seq = 0
ac.standoff_pub = _Pub('/arm/standoff_state')
st = ac.standoff_state(-3.0)
check(st['valid'] and abs(st['perp_mm'] - (-3.0 * ac._keyence_cos)) < 1e-9,
      'raw -3.0 projects by cos(42.6 deg) to the perpendicular')
check(abs(st['standoff_mm'] - (10.0 + 3.0 * ac._keyence_cos)) < 1e-9
      and st['err_mm'] < 0,
      f'standoff = zero - perp = {st["standoff_mm"]:.3f} mm, error negative = too far')
st = ac.standoff_state(+2.0)
check(st['standoff_mm'] < 10.0 and st['err_mm'] > 0,
      'positive raw = closer than the zero, error positive (approach-positive)')
for raw, side in ((-99999.0, 'far'), (99999.0, 'close')):
    st = ac.standoff_state(raw)
    check(not st['valid'] and st['standoff_mm'] is None and st['side'] == side,
          f'sentinel {raw:+.0f} -> invalid, side {side}')
PUBLISHED.clear()
ac.keyence_cb(types.SimpleNamespace(data=-1.5))
pub = [json.loads(d) for t, d in PUBLISHED if t == '/arm/standoff_state']
check(len(pub) == 1 and pub[0]['valid'] and abs(pub[0]['raw_mm'] + 1.5) < 1e-9
      and ac._keyence_seq == 1 and ac.current_keyence_val == -1.5,
      'keyence_cb caches the reading, bumps the sequence and publishes the state')

seen = {}
def _fake_adjust(_ac=ac):
    seen['setpoint'] = _ac.keyence_setpoint_mm
    seen['target'] = _ac.keyence_target_distance_mm
    seen['cancel'] = _ac.cancel_requested
    return StandoffResult(converged=True, reason='ok', steps=2, travel_mm=1.5,
                          final_err_mm=0.05)
ac._adjust_distance_to_surface = _fake_adjust
ac.cancel_requested = True
ok, message, res = ac.adjust_standoff(12.0)
check(ok and res.converged and message.startswith('standoff (target 12 mm): standoff ok'),
      f'adjust_standoff reports the loop result: {message!r}')
check(abs(seen['setpoint'] - (-2.0)) < 1e-9 and seen['target'] == 12.0,
      'the one-off target 12 mm is a setpoint of -2 mm during the call')
check(ac.keyence_setpoint_mm == 0.0 and ac.keyence_target_distance_mm == 10.0,
      'the configured setpoint / target are restored afterwards')
check(seen['cancel'] is False, 'a stale cancel flag is cleared before the loop runs')
ok, message, res = ac.adjust_standoff(None)
check(ok and seen['setpoint'] == 0.0 and 'target 10 mm' in message,
      'no target -> the configured 10 mm')
def _fail_adjust():
    return StandoffResult(converged=False, reason='out of range on the far side',
                          steps=0, travel_mm=0.0)
ac._adjust_distance_to_surface = _fail_adjust
ok, message, _ = ac.adjust_standoff(None)
check(not ok and 'NOT corrected: out of range' in message,
      f'a non-converged loop reports failure with its reason: {message!r}')

# ================================================================ scenario 5
# Joint control for robot_ui (2026-09-21): move_joint = one MoveJ to six
# angles, jog_joint = read the live joints, add delta to ONE of them, MoveJ.
robot = FakeRobot()
ac = make_controller(robot)
ac.busy = False
robot.calls.clear()
ok, msg = ac.move_joint([-90, -80, 100, -95, -90, 12.5], vel=25.0, acc=40.0)
check(ok and msg == 'move_joint ok', f'move_joint ok: {msg!r}')
check(robot.calls[-1] == ('MoveJ', TOOL_ID, (-90.0, -80.0, 100.0, -95.0, -90.0, 12.5))
      and robot.movej_kw == {'vel': 25.0, 'acc': 40.0},
      f'one MoveJ with the six angles, tool {TOOL_ID}, vel/acc passed: {robot.calls[-1]} {robot.movej_kw}')
check(robot.joints == [-90, -80, 100, -95, -90, 12.5], 'the fake arm is at the target')
ok, msg = ac.move_joint([1, 2, 3])
check(not ok and 'needs 6' in msg, 'five-or-fewer values refused before any RPC')
robot.movej_error = 7
ok, msg = ac.move_joint([0] * 6)
check(not ok and msg == 'move_joint failed: MoveJ error 7' and ac.busy is False,
      f'a MoveJ error code is reported and busy released: {msg!r}')
robot.movej_error = 0
robot.calls.clear()
ok, msg = ac.jog_joint('j3', -2.5, vel=20.0)
check(ok and msg == 'jog j3 -2.5 deg ok', f'jog_joint j3 -2.5: {msg!r}')
check(robot.calls[-1] == ('MoveJ', TOOL_ID, (-90.0, -80.0, 97.5, -95.0, -90.0, 12.5)),
      f'only J3 moved, by -2.5 deg from the LIVE joints: {robot.calls[-1]}')
ok, msg = ac.jog_joint(6, 1.0)
check(ok and robot.joints[5] == 13.5 and msg == 'jog j6 +1 deg ok', 'a 1..6 integer names the joint too')
ok, msg = ac.jog_joint('j7', 1.0)
check(not ok and 'unknown joint' in msg, 'unknown joint refused')
n = len(robot.calls)
ok, msg = ac.jog_joint('j1', 80.0, max_step=50.0)
check(not ok and 'exceeds max_step' in msg and len(robot.calls) == n, 'a jog over max_step never reaches the RPC')
ok, msg = ac.jog_joint('j1', 'x')
check(not ok and 'not a number' in msg, 'a non-numeric delta refused')
ac.busy = True
ok, msg = ac.move_joint([0] * 6)
check(not ok and msg == 'refused: arm busy', 'refused while busy')
ok, msg = ac.jog_joint('j1', 1.0)
check(not ok and msg == 'refused: arm busy', 'jog refused while busy')
ac.busy = False

# Error recovery (2026-09-28): reset_error = ResetAllError -> RobotEnable(1)
# -> Mode(0), no motion; ok judged from GetRobotErrorCode afterwards.
print('\n--- reset_error ---')


class FakeFaultRobot(FakeRobot):
    """A latched controller error: ResetAllError clears it unless `sticky`
    (a joint still beyond its soft limit re-trips at once)."""
    def __init__(self):
        super().__init__()
        self.err = [5, 3]
        self.sticky = False
        self.reset_ret = 0
        self.error_shape = 'tuple'

    def ResetAllError(self):
        self.calls.append(('ResetAllError',))
        if self.reset_ret == 0 and not self.sticky:
            self.err = [0, 0]
        return self.reset_ret

    def RobotEnable(self, s):
        self.calls.append(('RobotEnable', s)); return 0

    def Mode(self, m):
        self.calls.append(('Mode', m)); return 0

    def GetRobotErrorCode(self):
        if self.error_shape == 'raise':
            raise RuntimeError('socket busy')
        return 0, list(self.err)


import apriltag_nav.arm_controller as _acmod
_real_sleep = _acmod.time.sleep
_acmod.time.sleep = lambda _s: None
try:
    frobot = FakeFaultRobot()
    fac = make_controller(frobot)
    ok, msg = fac.reset_error()
    check(ok and msg == 'error cleared (code (0, [5, 3]) -> (0, [0, 0]))',
          f'latched error cleared: {msg!r}')
    check([c[0] for c in frobot.calls] == ['ResetAllError', 'RobotEnable', 'Mode']
          and ('RobotEnable', 1) in frobot.calls and ('Mode', 0) in frobot.calls,
          f'ResetAllError -> RobotEnable(1) -> Mode(0), in that order: {frobot.calls}')
    check(not any(c[0] == 'MoveJ' for c in frobot.calls), 'no motion commanded')
    check(fac.busy is False, 'busy released')

    frobot = FakeFaultRobot(); frobot.sticky = True
    fac = make_controller(frobot)
    ok, msg = fac.reset_error()
    check(not ok and 'still set after reset' in msg and 'soft limit' in msg,
          f'an error that re-trips is reported, with the joint-limit hint: {msg!r}')

    frobot = FakeFaultRobot(); frobot.reset_ret = 4
    fac = make_controller(frobot)
    ok, msg = fac.reset_error()
    check(not ok and msg.startswith('ResetAllError refused (code 4)'),
          f'a refused ResetAllError is a failure: {msg!r}')

    frobot = FakeFaultRobot(); frobot.error_shape = 'raise'
    fac = make_controller(frobot)
    ok, msg = fac.reset_error()
    check(ok and 'unconfirmed' in msg,
          f'unreadable error code: reset sent, reported as unconfirmed: {msg!r}')

    frobot = FakeFaultRobot()
    fac = make_controller(frobot)
    fac.busy = True
    ok, msg = fac.reset_error()
    check(not ok and msg == 'refused: arm busy' and frobot.calls == [],
          'refused while busy, nothing sent')
    fac.busy = False

    ec = ArmController.error_is_clear
    check(ec((0, [0, 0])) is True and ec((0, [5, 3])) is False
          and ec(0) is True and ec(7) is False and ec((-1, [0, 0])) is None
          and ec(None) is None and ec('x') is None,
          'error_is_clear: tuple / int / failed read / unknown shapes')
finally:
    _acmod.time.sleep = _real_sleep

# ================================================================ scenario: Ra collect mode (2026-10-06)
print('== Ra collect mode: pause after every scanned point, retreat, record, resume')
import csv as _csv


def collect_states():
    return [json.loads(d) for t, d in PUBLISHED if t == '/arm/collect_state']


def run_collect(points, release, timeout=0.0, csv_path='/x/scan_pose_demo_ra_map_20261006_120000.csv',
                mode='pause', batch=1, mark_dwell=0.0):
    """Run execute_scan_points with collect mode ON; `release(ac, state)` is
    called on a helper thread for every `waiting` state it sees."""
    PUBLISHED.clear(); LOG['err'].clear(); LOG['warn'].clear()
    robot = FakeRobot(); ac = make_controller(robot)
    ac.pipeline.output_dir = '/img/scan_pose_demo_ra_map_20261006_120000'
    ac.collect_enabled = True
    ac.collect_wait_timeout_s = timeout
    ac.collect_mode = mode
    ac.collect_batch_size = batch
    ac.collect_mark_dwell_s = mark_dwell
    for p in points:
        p['csv_path'] = csv_path
    seen = []
    stop = threading.Event()

    def helper():
        last = None
        while not stop.is_set():
            st = [x for x in collect_states() if x.get('waiting')]
            if st and (last is None or st[-1]['index'] != last):
                last = st[-1]['index']
                seen.append(st[-1])
                release(ac, st[-1])
            time.sleep(0.01)
    th = threading.Thread(target=helper, daemon=True); th.start()
    ac.execute_scan_points(points)
    stop.set(); th.join(1.0)
    return robot, ac, seen


def read_csv(path):
    with open(path) as f:
        return list(_csv.DictReader(f))


# (a) two scanned points + a traverse: each scanned point waits once
pts = [pose_pt(1, -1.30, 0.20), joint_pt(99, scan=False), pose_pt(1, -1.31, 0.21)]
pts[2]['group_id'] = 107           # same point_id in another group
robot, ac, seen = run_collect(
    pts, lambda a, st: a.collect_continue({'readings': [0.40, 0.42], 'note': 'ok'}))
ev = events()
check([e['phase'] for e in ev if e['phase'] in ('wait', 'resume')] == ['wait', 'resume', 'wait', 'resume'],
      f"wait/resume once per SCANNED point, none for the traverse: {[e['phase'] for e in ev]}")
check(len(seen) == 2 and seen[0]['images'] == ['g106_p1_i0001_s1.png'] and seen[1]['images'] == ['g107_p1_i0003_s1.png'],
      f"the waiting state names the frames by group / point / run index: {[s['images'] for s in seen]}")
check(seen[0]['image_dir'].endswith('scan_pose_demo_ra_map_20261006_120000') and seen[0]['total'] == 3,
      'waiting state carries the image dir and the totals')
movel = [c for c in robot.calls if c[0] == 'MoveL']
check(len(movel) == 4, f'retreat + return MoveL per scanned point: {len(movel)}')
# retreat: 80 mm along tool z, the sign that moves AWAY from the surface
# (keyence_dir -1 => dz = -80 along tool z; tool rpy (180, 0, 90) => tool z = -base z)
z0 = 300.0
check(abs(movel[0][2][2] - (z0 + 80.0)) < 1e-6 and movel[0][2][:2] == movel[1][2][:2]
      and movel[0][2][3:] == movel[1][2][3:],
      f'retreat is +80 mm base z for a tool pointing down (keyence_dir -1), xy / rpy kept: {movel[0][2]}')
check(abs(movel[1][2][2] - z0) < 1e-6, f'return goes back to the captured pose: {movel[1][2]}')
speeds = [c[1] for c in robot.calls if c[0] == 'SetSpeed']
check(10 in speeds, 'retreat / return use collect_retreat_speed')
rec = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_demo_ra_map_20261006_120000_ra_measured.csv'))
check(len(rec) == 2 and rec[0]['ra_measured'] == '0.4100' and rec[0]['ra_readings'] == '0.4000 0.4200'
      and rec[0]['note'] == 'ok' and rec[0]['skipped'] == 'False',
      f'measured CSV: mean of the readings, the readings, the note: {rec[0] if rec else None}')
check(rec[0]['images'] == 'g106_p1_i0001_s1.png' and rec[1]['images'] == 'g107_p1_i0003_s1.png'
      and rec[0]['group_id'] == '106' and rec[1]['group_id'] == '107',
      'rows keyed by group / point with the frame names')
check(rec[0]['run'] == 'scan_pose_demo_ra_map_20261006_120000' and rec[0]['x'] == '-1.3',
      f"row carries the run stem and the world x y z: run={rec[0]['run']} x={rec[0]['x']}")
check(ac.pipeline.prefixes == ['g106_p1_i0001', 'g107_p1_i0003'],
      f'the inference worker saves the frames under the same prefix: {ac.pipeline.prefixes}')
st = collect_states()[-1]
check(st['waiting'] is False and st['n_recorded'] == 2 and st['n_skipped'] == 0 and st['enabled'],
      f'final collect state: {st}')
check(ac.homed == [1] and [e['phase'] for e in ev][-1] == 'finished', 'scan ends normally (home, finished)')
resume = [e for e in ev if e['phase'] == 'resume']
check(abs(resume[0]['ra_measured'] - 0.41) < 1e-9 and resume[0]['skipped'] is False
      and resume[0]['n_ra'] == 1, 'resume event carries the Ra')

# (b) skip, and a refused release while nothing waits
ok, msg = ac.collect_continue({'ra': 0.5})
check(not ok and 'no point is waiting' in msg, f'release refused while nothing waits: {msg}')
robot, ac, seen = run_collect([pose_pt(3, -1.30, 0.20)], lambda a, st: a.collect_continue({'skip': True, 'note': 'no access'}),
                              csv_path='/x/scan_pose_skip_ra_map_20261006_120100.csv')
rec = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_skip_ra_map_20261006_120100_ra_measured.csv'))
check(len(rec) == 1 and rec[0]['skipped'] == 'True' and rec[0]['ra_measured'] == '' and rec[0]['note'] == 'no access',
      f'skip recorded with a blank Ra: {rec[0]}')
check(collect_states()[-1]['n_skipped'] == 1, 'skip counted')
check(len([c for c in robot.calls if c[0] == 'MoveL']) == 2, 'skip still returns the tool to the captured pose')

# (c) a release with no value is refused and the point keeps waiting
def _bad_then_good(a, st):
    ok, msg = a.collect_continue({'note': 'oops'})
    _bad_then_good.refused = (not ok, msg)
    a.collect_continue({'ra': 0.33})
robot, ac, seen = run_collect([pose_pt(4, -1.30, 0.20)], _bad_then_good,
                              csv_path='/x/scan_pose_bad_ra_map_20261006_120200.csv')
check(_bad_then_good.refused[0] and 'no Ra value' in _bad_then_good.refused[1],
      f'a release without ra / readings / skip is refused: {_bad_then_good.refused[1]}')
rec = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_bad_ra_map_20261006_120200_ra_measured.csv'))
check(len(rec) == 1 and rec[0]['ra_measured'] == '0.3300', 'the good release is the one recorded')

# (d) cancel during the wait: the scan ends, the tool stays retreated, nothing recorded
robot, ac, seen = run_collect([pose_pt(5, -1.30, 0.20), pose_pt(6, -1.30, 0.21)],
                              lambda a, st: setattr(a, 'cancel_requested', True),
                              csv_path='/x/scan_pose_cancel_ra_map_20261006_120300.csv')
check(len([c for c in robot.calls if c[0] == 'MoveL']) == 1, 'cancel while waiting: retreat only, no return move')
check(not os.path.exists(os.path.join(COLLECT_DIR, 'scan_pose_cancel_ra_map_20261006_120300_ra_measured.csv')),
      'nothing recorded for the cancelled point')
check(ac.homed == [] and events()[-1]['cancelled'] is True and len(ac.pipeline.captured) == 1,
      'scan ends cancelled at the next loop top, the second point never captured')
check(collect_states()[-1]['waiting'] is False, 'waiting cleared after the cancel')

# (e) wait timeout: recorded as skipped with the reason, scan goes on
robot, ac, seen = run_collect([pose_pt(7, -1.30, 0.20)], lambda a, st: None, timeout=0.3,
                              csv_path='/x/scan_pose_to_ra_map_20261006_120400.csv')
rec = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_to_ra_map_20261006_120400_ra_measured.csv'))
check(len(rec) == 1 and rec[0]['skipped'] == 'True' and 'timeout' in rec[0]['note'] and ac.homed == [1],
      f'timeout -> skipped with the reason, scan finished: {rec[0]["note"]}')

# (f) collect mode OFF: no wait, no MoveL, no record, prefix still passed
PUBLISHED.clear()
robot = FakeRobot(); ac = make_controller(robot)
ac.execute_scan_points([pose_pt(8, -1.30, 0.20, )])
check(not [e for e in events() if e['phase'] in ('wait', 'resume')] and not [c for c in robot.calls if c[0] == 'MoveL'],
      'collect mode off: no pause, no retreat')
check(ac.pipeline.prefixes == ['g106_p8_i0001'], 'frames are still named g<group>_p<point>_i<index> with the mode off')
ok, msg = ac.set_collect_mode(True)
check(ok and ac.collect_enabled and collect_states()[-1]['enabled'] is True, 'set_collect_mode publishes the state')

# (g) a point whose move failed or captured nothing does not pause
ac.pipeline.capture = lambda point_id, cancelled: []
PUBLISHED.clear(); robot.calls.clear()
ac.execute_scan_points([pose_pt(9, -1.30, 0.20)])
check(not [e for e in events() if e['phase'] == 'wait'] and not [c for c in robot.calls if c[0] == 'MoveL'],
      'no frames captured -> no pause')

# (h) the image naming helper itself
from apriltag_nav.arm_controller import image_file_name as _ifn, image_name_prefix as _inp
check(_inp(106, 14, 37) == 'g106_p14_i0037' and _ifn(14, 2, 0.5, 'g106_p14_i0037') == 'g106_p14_i0037_s2.png',
      'image name helpers')


# ================================================================ method A: batches (2026-10-06 evening)
print('== collect mode, batch_size 2: one stop per two scanned points, offsets from the tip')
pts = [pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225), joint_pt(99, scan=False), pose_pt(3, -1.325, 0.225)]
got = []
def _release_batch(a, st):
    got.append([(p['group_id'], p['point_id'], p['is_tip'], p['dx_world_mm'], p['dy_world_mm'],
                 p['dx_arm_mm'], p['dy_arm_mm']) for p in st['points']])
    a.collect_continue({'points': [{'group_id': p['group_id'], 'point_id': p['point_id'],
                                    'readings': f"{0.4 + 0.01 * p['point_id']:.2f}"}
                                   for p in st['points']]})
robot, ac, seen = run_collect(pts, _release_batch, batch=2,
                              csv_path='/x/scan_pose_b2_ra_map_20261006_130000.csv')
ev = events()
check([e['phase'] for e in ev if e['phase'] in ('wait', 'resume')] == ['wait', 'resume', 'wait', 'resume'],
      f"3 scanned points, batch 2: stop after point 2 and after the LAST point: {[e['phase'] for e in ev]}")
check(len(got) == 2 and [g[:3] for g in got[0]] == [(106, 1, False), (106, 2, True)] and [g[:3] for g in got[1]] == [(106, 3, True)],
      f'first stop lists points 1 + 2 with point 2 under the tip, the last stop point 3 alone: {got}')
check(got[0][0][3] == 0.0 and got[0][0][4] == -25.0, f'world offset of point 1 from the tip: dx 0, dy -25 mm: {got[0][0][3:5]}')
check(got[0][0][5] is not None and abs(abs(got[0][0][5]) + abs(got[0][0][6]) - 25.0) < 0.5,
      f'arm-frame offset through transform_world_to_arm, 25 mm long: {got[0][0][5:7]}')
movel = [c for c in robot.calls if c[0] == 'MoveL']
check(len(movel) == 4, f'retreat + return per STOP, not per point: {len(movel)} MoveL')
rec = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_b2_ra_map_20261006_130000_ra_measured.csv'))
check([(r['point_id'], r['ra_measured']) for r in rec] == [('1', '0.4100'), ('2', '0.4200'), ('3', '0.4300')],
      f'one row per point with its own Ra: {[(r["point_id"], r["ra_measured"]) for r in rec]}')
check(rec[0]['images'] == 'g106_p1_i0001_s1.png' and rec[1]['images'] == 'g106_p2_i0002_s1.png' and rec[2]['images'] == 'g106_p3_i0004_s1.png',
      'frame names per row (run index skips the traverse row)')
check(collect_states()[-1]['n_recorded'] == 3, 'three recorded')

print('== batch release rules')
def _partial(a, st):
    ok, msg = a.collect_continue({'ra': 0.5})
    _partial.flat = (ok, msg)
    ok, msg = a.collect_continue({'points': [{'group_id': 999, 'point_id': 1, 'ra': 0.5}]})
    _partial.wrong = (ok, msg)
    ok, msg = a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'note': 'x'}]})
    _partial.empty = (ok, msg)
    a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.5}]})   # point 2 not entered
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225)], _partial, batch=2,
                              csv_path='/x/scan_pose_b2r_ra_map_20261006_130100.csv')
check(not _partial.flat[0] and '2 points are waiting' in _partial.flat[1], f'the flat form is refused for a 2-point batch: {_partial.flat[1]}')
check(not _partial.wrong[0] and 'not in the waiting batch' in _partial.wrong[1], 'a point outside the batch is refused')
check(not _partial.empty[0] and 'no Ra value' in _partial.empty[1], 'a listed point without a value is refused')
rec = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_b2r_ra_map_20261006_130100_ra_measured.csv'))
check([(r['point_id'], r['ra_measured'], r['skipped'], r['note']) for r in rec] == [('1', '0.5000', 'False', ''), ('2', '', 'True', 'not entered')],
      f'a batch point left out of the release is recorded as skipped "not entered": {rec}')

print('== set_collect_config')
ok, msg = ac.set_collect_config({'mode': 'mark', 'batch_size': 3, 'mark_dwell_s': 2})
check(ok and ac.collect_mode == 'mark' and ac.collect_batch_size == 3 and ac.collect_mark_dwell_s == 2.0, msg)
st = collect_states()[-1]
check(st['mode'] == 'mark' and st['batch_size'] == 3 and st['mark_dwell_s'] == 2.0, 'config published in the state')
ok, msg = ac.set_collect_config({'mode': 'x'})
check(not ok and 'unknown collect mode' in msg, 'bad mode refused')
ok, msg = ac.set_collect_config({'batch_size': 0})
check(not ok and 'out of range' in msg, 'batch 0 refused')

# ================================================================ method B: mark pass
print('== mark pass: no capture, stop at every scanned point, template CSV with running numbers')
pts = [pose_pt(1, -1.30, 0.20), joint_pt(99, scan=False), pose_pt(2, -1.30, 0.225)]
marks = []
def _mark(a, st):
    marks.append((st['kind'], st['mark_no'], st['group_id'], st['point_id']))
    a.collect_continue({})                      # anything releases a mark stop
robot, ac, seen = run_collect(pts, _mark, mode='mark', csv_path='/x/scan_pose_mk_ra_map_20261006_140000.csv')
check(ac.pipeline.captured == [] and ac.pipeline.preopened == 0, 'mark pass captures nothing and never opens the camera')
check(marks == [('mark', 1, 106, 1), ('mark', 2, 106, 2)], f'running numbers 1, 2 over the scanned points only: {marks}')
movel = [c for c in robot.calls if c[0] == 'MoveL']
check(len(movel) == 4 and abs(movel[0][2][2] - 330.0) < 1e-6, f'30 mm retreat + return per stop: {len(movel)}, z {movel[0][2][2]}')
check(ac.last_results is None or not os.path.exists('/x'), 'no Ra map written for a mark pass')
tpl = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_mk_ra_map_20261006_140000_mark_template.csv'))
check([(r['mark_no'], r['group_id'], r['point_id'], r['ra_measured'], r['images']) for r in tpl]
      == [('1', '106', '1', '', ''), ('2', '106', '2', '', '')], f'template: number -> group / point, blank Ra, no images: {tpl}')
check(tpl[0]['x'] == '-1.3' and tpl[1]['y'] == '0.225', 'template carries the world x y z')
ev = events()
check([e for e in ev if e['phase'] == 'wait'][0]['kind'] == 'mark' and [e for e in ev if e['phase'] == 'wait'][0]['mark_no'] == 1,
      'wait event says mark #1')
check([e['message'] for e in ev if e['phase'] == 'done'] == ['mark', 'traverse', 'mark'], 'done events say mark')
check(ac.homed == [1], 'mark pass ends with the home move')

print('== mark pass with a dwell: continues by itself')
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20)], lambda a, st: None, mode='mark', mark_dwell=0.2,
                              csv_path='/x/scan_pose_mkd_ra_map_20261006_140100.csv')
tpl = read_csv(os.path.join(COLLECT_DIR, 'scan_pose_mkd_ra_map_20261006_140100_mark_template.csv'))
check(len(tpl) == 1 and ac.homed == [1], 'dwell 0.2 s: one template row, scan finished without a release')
ok, msg = ac.collect_continue({'ra': 0.5})
check(not ok, 'release refused when nothing waits (mark mode too)')

import shutil; shutil.rmtree(COLLECT_DIR, ignore_errors=True)

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
