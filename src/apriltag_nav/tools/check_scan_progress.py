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


def cpath(stem, suffix):
    """Where the controller puts a run's measured CSV:
    <record_dir>/<run>/<run><suffix>, i.e. inside the run's frame folder."""
    return os.path.join(COLLECT_DIR, stem, stem + suffix)


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
    ac.collect_premark = False
    ac.collect_mark_retreat_mm = 0.0
    ac.collect_mark_dwell_s = 0.0
    ac._collect_batch = []
    ac._collect_mark_no = 0
    ac._collect_mark_stem = None
    ac._collect_run_stem = ''
    ac.keyence_dir = -1.0
    ac.collect_state_pub = _Pub('/arm/collect_state')
    ac._collect_event = threading.Event()
    ac._collect_payload = None
    ac._collect_lock = threading.Lock()
    ac._collect_io_lock = threading.Lock()
    ac._collect = {'waiting': False, 'n_recorded': 0, 'n_skipped': 0, 'record_csv': '', 'saved': {}}
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
                mode='pause', batch=1, mark_dwell=0.0, premark=False):
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
    ac.collect_premark = premark
    for p in points:
        p['csv_path'] = csv_path
    seen = []
    stop = threading.Event()

    def helper():
        last = None
        while not stop.is_set():
            st = [x for x in collect_states() if x.get('waiting')]
            if st and (last is None or (st[-1]['index'], st[-1].get('kind')) != last):
                last = (st[-1]['index'], st[-1].get('kind'))
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
rec = read_csv(cpath('scan_pose_demo_ra_map_20261006_120000', '_ra_measured.csv'))
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
rec = read_csv(cpath('scan_pose_skip_ra_map_20261006_120100', '_ra_measured.csv'))
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
rec = read_csv(cpath('scan_pose_bad_ra_map_20261006_120200', '_ra_measured.csv'))
check(len(rec) == 1 and rec[0]['ra_measured'] == '0.3300', 'the good release is the one recorded')

# (d) cancel during the wait: the scan ends, the tool stays retreated, nothing recorded
robot, ac, seen = run_collect([pose_pt(5, -1.30, 0.20), pose_pt(6, -1.30, 0.21)],
                              lambda a, st: setattr(a, 'cancel_requested', True),
                              csv_path='/x/scan_pose_cancel_ra_map_20261006_120300.csv')
check(len([c for c in robot.calls if c[0] == 'MoveL']) == 1, 'cancel while waiting: retreat only, no return move')
check(not os.path.exists(cpath('scan_pose_cancel_ra_map_20261006_120300', '_ra_measured.csv')),
      'nothing recorded for the cancelled point')
check(ac.homed == [] and events()[-1]['cancelled'] is True and len(ac.pipeline.captured) == 1,
      'scan ends cancelled at the next loop top, the second point never captured')
check(collect_states()[-1]['waiting'] is False, 'waiting cleared after the cancel')

# (e) wait timeout: recorded as skipped with the reason, scan goes on
robot, ac, seen = run_collect([pose_pt(7, -1.30, 0.20)], lambda a, st: None, timeout=0.3,
                              csv_path='/x/scan_pose_to_ra_map_20261006_120400.csv')
rec = read_csv(cpath('scan_pose_to_ra_map_20261006_120400', '_ra_measured.csv'))
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
rec = read_csv(cpath('scan_pose_b2_ra_map_20261006_130000', '_ra_measured.csv'))
check([(r['point_id'], r['ra_measured']) for r in rec] == [('1', '0.4100'), ('2', '0.4200'), ('3', '0.4300')],
      f'one row per point with its own Ra: {[(r["point_id"], r["ra_measured"]) for r in rec]}')
check(rec[0]['images'] == 'g106_p1_i0001_s1.png' and rec[1]['images'] == 'g106_p2_i0002_s1.png' and rec[2]['images'] == 'g106_p3_i0004_s1.png',
      'frame names per row (run index skips the traverse row)')
check(collect_states()[-1]['n_recorded'] == 3, 'three recorded')

print('== premark: a marking stop at the standoff at every scanned point, before the batch stop')
kinds = []
def _premark(a, st):
    kinds.append((st['kind'], st['point_id'], st['retreated'], len([c for c in robot_ref[0].calls if c[0] == 'MoveL'])))
    if st['kind'] == 'premark':
        ok, msg = a.collect_continue({})
        _premark.ok = (ok, msg)
    else:
        a.collect_continue({'points': [{'group_id': p['group_id'], 'point_id': p['point_id'], 'ra': 0.5} for p in st['points']]})
robot_ref = [None]
_orig_make = make_controller
def make_controller(robot, _m=_orig_make):      # noqa: F811
    robot_ref[0] = robot
    return _m(robot)
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225)], _premark, batch=2, premark=True,
                              csv_path='/x/scan_pose_pm_ra_map_20261006_130200.csv')
make_controller = _orig_make
check([k[:3] for k in kinds] == [('premark', 1, False), ('premark', 2, False), ('pause', 2, True)],
      f'premark at points 1 and 2 (not retreated), then the batch stop (retreated): {kinds}')
check(kinds[0][3] == 0 and kinds[1][3] == 0 and kinds[2][3] == 1,
      f'no MoveL before either premark stop, the retreat only at the batch stop: {[k[3] for k in kinds]}')
check(_premark.ok[0] and 'marked' in _premark.ok[1], f'a bare continue releases a premark stop: {_premark.ok}')
ev = events()
check([(e['phase'], e.get('kind')) for e in ev if e['phase'] in ('wait', 'resume')]
      == [('wait', 'premark'), ('resume', 'premark'), ('wait', 'premark'), ('resume', 'premark'), ('wait', 'pause'), ('resume', 'pause')],
      'wait / resume events carry the kind')
rec = read_csv(cpath('scan_pose_pm_ra_map_20261006_130200', '_ra_measured.csv'))
check([(r['point_id'], r['ra_measured']) for r in rec] == [('1', '0.5000'), ('2', '0.5000')], 'rows only from the batch stop')
ok, msg = ac.set_collect_config({'premark': True})
check(ok and collect_states()[-1]['premark'] is True, 'premark in the config / state')

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
rec = read_csv(cpath('scan_pose_b2r_ra_map_20261006_130100', '_ra_measured.csv'))
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
print('== mark pass (2026-10-07): capture, numbered marking stop at every point, ONE entry stop per group')
pts = [pose_pt(1, -1.30, 0.20), joint_pt(99, scan=False), pose_pt(2, -1.30, 0.225)]
marks = []
def _mark(a, st):
    if st['kind'] == 'mark':
        marks.append(('mark', st['mark_no'], st['group_id'], st['point_id'], st['retreated'], list(st['images'])))
        a.collect_continue({})                  # anything releases a marking stop
    else:
        marks.append(('pause', [(p['mark_no'], p['group_id'], p['point_id']) for p in st['points']], st['retreated']))
        a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.31},
                                       {'group_id': 106, 'point_id': 2, 'readings': [0.5, 0.52], 'note': 'n2'}]})
robot, ac, seen = run_collect(pts, _mark, mode='mark', csv_path='/x/scan_pose_mk_ra_map_20261006_140000.csv')
check(ac.pipeline.captured == [1, 2] and ac.pipeline.preopened >= 1, f'mark pass CAPTURES every scanned point: {ac.pipeline.captured}')
check(marks[:2] == [('mark', 1, 106, 1, False, ['g106_p1_i0001_s1.png']), ('mark', 2, 106, 2, False, ['g106_p2_i0003_s1.png'])],
      f'numbered marking stops #1, #2 at the standoff (not retreated), the frames named: {marks[:2]}')
check(len(marks) == 3 and marks[2] == ('pause', [(1, 106, 1), (2, 106, 2)], True),
      f'ONE entry stop after the last point, listing the whole group with its numbers, retreated: {marks[2:]}')
movel = [c for c in robot.calls if c[0] == 'MoveL']
check(len(movel) == 2 and abs(movel[0][2][2] - (300.0 + 80.0)) < 1e-6 and abs(movel[1][2][2] - 300.0) < 1e-6,
      f'no MoveL at the marking stops (retreat 0); the entry stop retreats 80 mm and returns: {len(movel)}')
rows = read_csv(cpath('scan_pose_mk_ra_map_20261006_140000', '_ra_measured.csv'))
check([(r['mark_no'], r['group_id'], r['point_id'], r['ra_measured'], r['images'], r['note']) for r in rows]
      == [('1', '106', '1', '0.3100', 'g106_p1_i0001_s1.png', ''), ('2', '106', '2', '0.5100', 'g106_p2_i0003_s1.png', 'n2')],
      f'rows in <run>_ra_measured.csv with mark_no, Ra and the frame: {rows}')
check(not os.path.exists(cpath('scan_pose_mk_ra_map_20261006_140000', '_mark_template.csv')), 'no template file any more')
_by_pid = {r['point_id']: r for r in (ac.last_results or [])}
check(_by_pid.get(1, {}).get('ra_mean') == 0.4 and _by_pid.get(2, {}).get('ra_mean') == 0.4,
      f'the Ra map rows are written (model Ra) like a normal scan: {sorted(_by_pid)}')
ev = events()
w = [e for e in ev if e['phase'] == 'wait']
check([(e['kind'], e.get('mark_no')) for e in w] == [('mark', 1), ('mark', 2), ('pause', None)]
      and w[2]['points'] == [[106, 1], [106, 2]], f'wait events: mark #1, mark #2, then the group entry: {[(e["kind"], e.get("mark_no")) for e in w]}')
check([e['message'] for e in ev if e['phase'] == 'done'] == ['Success', 'traverse', 'Success'], 'done events are the normal ones')
check(ac.homed == [1], 'mark pass ends with the home move')
st = collect_states()[-1]
check(st['n_recorded'] == 2 and st['last']['n_ra'] == 2, 'state counts the recorded group')

print('== mark numbering continues across the groups of a run, restarts on a new run, resumes from the CSV')
pts2 = [pose_pt(3, -1.30, 0.25)]
pts2[0]['group_id'] = 107
PUBLISHED.clear()
ac.collect_enabled = True
for p_ in pts2: p_['csv_path'] = '/x/scan_pose_mk_ra_map_20261006_140000.csv'
marks2 = []
def _mark2(a, st):
    if st['kind'] == 'mark':
        marks2.append(st['mark_no']); a.collect_continue({})
    else:
        a.collect_continue({'points': [{'group_id': 107, 'point_id': 3, 'ra': 0.7}]})
seen2 = []
stop2 = threading.Event()
def helper2():
    last = None
    while not stop2.is_set():
        st_ = [x for x in collect_states() if x.get('waiting')]
        if st_ and (last is None or (st_[-1]['index'], st_[-1].get('kind')) != last):
            last = (st_[-1]['index'], st_[-1].get('kind')); _mark2(ac, st_[-1])
        time.sleep(0.01)
th2 = threading.Thread(target=helper2, daemon=True); th2.start()
ac.execute_scan_points(pts2)
stop2.set(); th2.join(1.0)
rows = read_csv(cpath('scan_pose_mk_ra_map_20261006_140000', '_ra_measured.csv'))
check(marks2 == [3] and [r['mark_no'] for r in rows] == ['1', '2', '3'],
      f'the next group of the SAME run (same controller) continues at #3: {marks2} / {[r["mark_no"] for r in rows]}')
# a fresh controller on the same run (arm_node restarted / RESUME): starts after the CSV's highest number
robot, ac, seen = run_collect([pose_pt(4, -1.30, 0.275)], lambda a, st: (a.collect_continue({}) if st['kind'] == 'mark'
                              else a.collect_continue({'points': [{'group_id': 106, 'point_id': 4, 'ra': 0.2}]})),
                              mode='mark', csv_path='/x/scan_pose_mk_ra_map_20261006_140000.csv')
check([s_['mark_no'] for s_ in seen if s_['kind'] == 'mark'] == [4], f'a fresh controller on the same run resumes at #4: {[s_.get("mark_no") for s_ in seen]}')
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20)], lambda a, st: (a.collect_continue({}) if st['kind'] == 'mark'
                              else a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.2}]})),
                              mode='mark', csv_path='/x/scan_pose_mk2_ra_map_20261006_150000.csv')
check([s_['mark_no'] for s_ in seen if s_['kind'] == 'mark'] == [1], 'a new run starts at #1')

print('== mark pass with a retreat and a dwell: the marking stop continues by itself, the entry stop waits')
def _dwell(a, st):
    if st['kind'] == 'pause':
        a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.33}]})
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20)], _dwell, mode='mark', mark_dwell=0.2,
                              csv_path='/x/scan_pose_mkd_ra_map_20261006_140100.csv')
PUBLISHED_SNAP = list(PUBLISHED)
check([s_['kind'] for s_ in seen] == ['mark', 'pause'] and ac.homed == [1],
      f'dwell 0.2 s: the marking stop released itself, the entry stop answered: {[s_["kind"] for s_ in seen]}')
rows = read_csv(cpath('scan_pose_mkd_ra_map_20261006_140100', '_ra_measured.csv'))
check(len(rows) == 1 and rows[0]['mark_no'] == '1' and rows[0]['ra_measured'] == '0.3300', 'one row, #1, Ra 0.33')
ac.collect_mark_retreat_mm = 30.0
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20)], lambda a, st: (a.collect_continue({}) if st['kind'] == 'mark'
                              else a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'skip': True}]})),
                              mode='mark', csv_path='/x/scan_pose_mkr_ra_map_20261006_140200.csv')
check(seen[0]['kind'] == 'mark' and seen[0]['retreated'] is False, 'default fixture: marking stop not retreated')
ok, msg = ac.collect_continue({})
check(not ok, 'release refused when nothing waits (mark mode too)')

# ================================================================ method B: an interrupted group keeps its photos (2026-10-07)
print('== mark pass: the number is in the CSV at the marking stop (pending row), the entry stop fills it in')
pend_seen = []
def _mark_pend(a, st):
    if st['kind'] == 'mark':
        pend_seen.append([(r['mark_no'], r['point_id'], r['ra_measured'], r['note'], r['measured_at'], r['skipped'])
                          for r in read_csv(cpath('scan_pose_mkp_ra_map_20261007_100000', '_ra_measured.csv'))])
        a.collect_continue({})
    else:
        a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.41},
                                       {'group_id': 106, 'point_id': 2, 'ra': 0.42}]})
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225)], _mark_pend, mode='mark',
                              csv_path='/x/scan_pose_mkp_ra_map_20261007_100000.csv')
check(pend_seen[0] == [('1', '1', '', 'pending: Ra at the group entry stop', '', 'False')],
      f'at marking stop #1 the CSV already holds the pending row (Ra blank, not skipped): {pend_seen[0]}')
check(len(pend_seen) == 2 and [r[0] for r in pend_seen[1]] == ['1', '2'] and pend_seen[1][1][2] == '',
      f'at marking stop #2 both pending rows are there: {pend_seen[1]}')
rows = read_csv(cpath('scan_pose_mkp_ra_map_20261007_100000', '_ra_measured.csv'))
check([(r['mark_no'], r['point_id'], r['ra_measured'], r['note']) for r in rows] == [('1', '1', '0.4100', ''), ('2', '2', '0.4200', '')]
      and all(r['measured_at'] for r in rows),
      f'the entry stop REPLACES the pending rows (no duplicates, Ra filled, stamped): {rows}')

print('== mark pass interrupted at the entry stop: a RESUME drives the captured points through and lists them')
def _mark_cancel(a, st):
    if st['kind'] == 'mark':
        a.collect_continue({})
    else:
        a.cancel_requested = True               # the e-stop at the group's entry stop
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225)], _mark_cancel, mode='mark',
                              csv_path='/x/scan_pose_mki_ra_map_20261007_110000.csv')
rows = read_csv(cpath('scan_pose_mki_ra_map_20261007_110000', '_ra_measured.csv'))
check([(r['mark_no'], r['point_id'], r['ra_measured']) for r in rows] == [('1', '1', ''), ('2', '2', '')] and ac.homed == [],
      f'cancelled at the entry stop: both rows stay PENDING, no home move: {[(r["mark_no"], r["ra_measured"]) for r in rows]}')
# the resumed plan (scan_resume): points 1, 2 driven through (scan False), point 3 still to scan
res_marks = []
def _mark_resume(a, st):
    if st['kind'] == 'mark':
        res_marks.append((st['mark_no'], st['point_id'])); a.collect_continue({})
    else:
        res_marks.append(('pause', [(p['mark_no'], p['point_id'], p.get('is_tip')) for p in st['points']], st['retreated']))
        a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.11},
                                       {'group_id': 106, 'point_id': 2, 'skip': True, 'note': 'smudged'},
                                       {'group_id': 106, 'point_id': 3, 'ra': 0.33}]})
pts_r = [pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225), pose_pt(3, -1.30, 0.25)]
pts_r[0]['scan'] = False; pts_r[1]['scan'] = False
robot, ac, seen = run_collect(pts_r, _mark_resume, mode='mark', csv_path='/x/scan_pose_mki_ra_map_20261007_110000.csv')
check(ac.pipeline.captured == [3], f'resume: the captured points are NOT re-shot, only point 3: {ac.pipeline.captured}')
check(res_marks[0] == (3, 3), f'point 3 gets the next number #3 (counter from the pending rows): {res_marks[:1]}')
check(len(res_marks) == 2 and res_marks[1] == ('pause', [(1, 1, False), (2, 2, False), (3, 3, True)], True),
      f'the entry stop lists the two pending points in front of the new one, tip = the new one, retreated: {res_marks[1:]}')
rows = read_csv(cpath('scan_pose_mki_ra_map_20261007_110000', '_ra_measured.csv'))
check([(r['mark_no'], r['point_id'], r['ra_measured'], r['skipped'], r['note']) for r in rows]
      == [('1', '1', '0.1100', 'False', ''), ('2', '2', '', 'True', 'smudged'), ('3', '3', '0.3300', 'False', '')],
      f'three rows, the pending two replaced in place (one Ra, one skipped), the new one appended: {rows}')
check(ac.homed == [1], 'resumed group ends with the home move')

print('== mark pass resumed with EVERY point already captured: no capture, one entry stop at the end, no retreat')
all_marks = []
def _mark_all(a, st):
    all_marks.append((st['kind'], [(p['mark_no'], p['point_id']) for p in st['points']], st['retreated']))
    a.collect_continue({'points': [{'group_id': 106, 'point_id': 7, 'ra': 0.77}]})
robot0, ac0, seen0 = run_collect([pose_pt(7, -1.30, 0.20)], _mark_cancel, mode='mark',
                                 csv_path='/x/scan_pose_mka_ra_map_20261007_120000.csv')
pts_a = [pose_pt(7, -1.30, 0.20)]; pts_a[0]['scan'] = False
robot, ac, seen = run_collect(pts_a, _mark_all, mode='mark', csv_path='/x/scan_pose_mka_ra_map_20261007_120000.csv')
check(ac.pipeline.captured == [] and all_marks == [('pause', [(1, 7)], False)],
      f'nothing captured, one entry stop listing #1 without a retreat: {all_marks}')
check([c for c in robot.calls if c[0] == 'MoveL'] == [], 'no MoveL at all (no retreat, no return)')
rows = read_csv(cpath('scan_pose_mka_ra_map_20261007_120000', '_ra_measured.csv'))
check([(r['mark_no'], r['ra_measured']) for r in rows] == [('1', '0.7700')] and ac.homed == [1],
      f'the pending row got its Ra, then home: {rows}')
w = [json.loads(d) for t, d in PUBLISHED if t == '/arm/scan_progress']
check([e['phase'] for e in w if e['phase'] in ('wait', 'resume')] == ['wait', 'resume'], 'one wait / resume pair')

print('== Save during the entry stop (2026-10-07 evening): rows on disk before the release, pre-filled, release wins')
save_log = []
def _save_then_release(a, st):
    if st['kind'] == 'mark':
        a.collect_continue({}); return
    csvp = cpath('scan_pose_sav_ra_map_20261007_150000', '_ra_measured.csv')
    # (1) a save with one Ra, one blank (still typing), one skip
    ok, msg = a.collect_save({'points': [{'group_id': 106, 'point_id': 1, 'readings': '0.40 0.42', 'note': 'first'},
                                         {'group_id': 106, 'point_id': 2, 'readings': ''},
                                         {'group_id': 106, 'point_id': 3, 'skip': True, 'note': 'smudged'}]})
    save_log.append(('save1', ok, msg, [(r['mark_no'], r['point_id'], r['ra_measured'], r['skipped'], r['note'], bool(r['measured_at'])) for r in read_csv(csvp)]))
    st1 = collect_states()[-1]
    save_log.append(('state1', st1.get('waiting'), st1.get('n_saved'), dict(st1.get('saved') or {})))
    # (2) refusals: unknown point, bad number, nothing typed, no points
    save_log.append(('bad', a.collect_save({'points': [{'group_id': 999, 'point_id': 1, 'ra': 0.5}]}),
                     a.collect_save({'points': [{'group_id': 106, 'point_id': 2, 'readings': 'abc'}]}),
                     a.collect_save({'points': [{'group_id': 106, 'point_id': 2, 'readings': ''}]}),
                     a.collect_save({})))
    # (3) a second save overwrites point 1 and adds point 2
    ok, msg = a.collect_save({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.45},
                                         {'group_id': 106, 'point_id': 2, 'ra': 0.22}]})
    save_log.append(('save2', ok, msg, [(r['point_id'], r['ra_measured'], r['skipped']) for r in read_csv(csvp)]))
    # (4) the release names ONLY point 3 with a value — 1 and 2 come from the save
    a.collect_continue({'points': [{'group_id': 106, 'point_id': 3, 'ra': 0.33}]})
pts_s = [pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225), pose_pt(3, -1.30, 0.25)]
robot, ac, seen = run_collect(pts_s, _save_then_release, mode='mark', csv_path='/x/scan_pose_sav_ra_map_20261007_150000.csv')
s1 = [x for x in save_log if x[0] == 'save1'][0]
check(s1[1] and 'saved 2 point(s) (1 Ra, 1 skip); 2 of 3 on disk' in s1[2], f'save: two of three entries taken, the blank one left out: {s1[2]}')
check(s1[3] == [('1', '1', '0.4100', 'False', 'first', True), ('2', '2', '', 'False', 'pending: Ra at the group entry stop', False),
                ('3', '3', '', 'True', 'smudged', True)],
      f'CSV right after the save: p1 Ra (mean of the readings) stamped, p2 still PENDING, p3 skipped — numbers kept: {s1[3]}')
st1 = [x for x in save_log if x[0] == 'state1'][0]
check(st1[1] is True and st1[2] == 2 and set(st1[3]) == {'106,1', '106,3'} and st1[3]['106,1']['ra'] == 0.41 and st1[3]['106,3']['skip'] is True,
      f'the stop stays open; the state carries saved per point and n_saved: {st1[1:]}')
bad = [x for x in save_log if x[0] == 'bad'][0]
check(all(not r[0] for r in bad[1:]) and 'not in the waiting batch' in bad[1][1] and 'not a number' in bad[2][1]
      and 'nothing to save yet' in bad[3][1] and 'send `points`' in bad[4][1],
      f'save refusals: unknown point / bad number / nothing typed / no points: {[r[1] for r in bad[1:]]}')
s2 = [x for x in save_log if x[0] == 'save2'][0]
check(s2[1] and s2[3] == [('1', '0.4500', 'False'), ('2', '0.2200', 'False'), ('3', '', 'True')] and '3 of 3 on disk' in s2[2],
      f'a second save overwrites p1 in place and fills p2: {s2[2]} {s2[3]}')
rows = read_csv(cpath('scan_pose_sav_ra_map_20261007_150000', '_ra_measured.csv'))
check([(r['mark_no'], r['point_id'], r['ra_measured'], r['skipped'], r['note']) for r in rows]
      == [('1', '1', '0.4500', 'False', ''), ('2', '2', '0.2200', 'False', ''), ('3', '3', '0.3300', 'False', '')],
      f'release naming only p3: p1 / p2 take their SAVED values, p3 the release (which wins over its saved skip), no duplicates: {rows}')
stf = collect_states()[-1]
check(stf.get('waiting') is False and stf.get('saved') == {} and stf.get('n_saved') == 0
      and stf['n_recorded'] == 3 and stf['n_skipped'] == 0, f'after the release the saved map is cleared, counts from the final rows: {stf}')
ok, msg = ac.collect_save({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.5}]})
check(not ok and 'nothing to save' in msg, 'save refused when nothing waits')

print('== Save, then the e-stop: the saved point is done, the rest pending; the RESUME lists only the rest')
def _save_cancel(a, st):
    if st['kind'] == 'mark':
        a.collect_continue({}); return
    ok, msg = a.collect_save({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.61}]})
    save_log.append(('sc', ok, msg))
    a.cancel_requested = True
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225)], _save_cancel, mode='mark',
                              csv_path='/x/scan_pose_svc_ra_map_20261007_151000.csv')
rows = read_csv(cpath('scan_pose_svc_ra_map_20261007_151000', '_ra_measured.csv'))
check([(r['mark_no'], r['point_id'], r['ra_measured']) for r in rows] == [('1', '1', '0.6100'), ('2', '2', '')],
      f'cancelled after a save: p1 keeps its Ra on disk, p2 stays pending: {rows}')
check(any('1 saved point(s) are on disk' in w for w in LOG['warn']), 'the cancel warning counts the saved points')
pend = ac._collect_pending_rows([106])
check([b['point_id'] for b in pend] == [2], f'only p2 is pending: {[b["point_id"] for b in pend]}')
from apriltag_nav import scan_resume as _sr
check(_sr.pending_points(cpath('scan_pose_svc_ra_map_20261007_151000', '_ra_measured.csv')) == {(106, 2)},
      'scan_resume sees p2 pending and p1 answered')
res_list = []
def _after_save_resume(a, st):
    if st['kind'] == 'mark':
        a.collect_continue({}); return
    res_list.append([(p['mark_no'], p['point_id']) for p in st['points']])
    a.collect_continue({'points': [{'group_id': 106, 'point_id': 2, 'ra': 0.62}]})
pts_r2 = [pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225)]; pts_r2[0]['scan'] = False; pts_r2[1]['scan'] = False
robot, ac, seen = run_collect(pts_r2, _after_save_resume, mode='mark', csv_path='/x/scan_pose_svc_ra_map_20261007_151000.csv')
check(res_list == [[(2, 2)]], f'the resumed entry stop lists p2 alone (p1 was saved): {res_list}')
rows = read_csv(cpath('scan_pose_svc_ra_map_20261007_151000', '_ra_measured.csv'))
check([(r['point_id'], r['ra_measured']) for r in rows] == [('1', '0.6100'), ('2', '0.6200')], f'both rows filled, two rows only: {rows}')

print('== save at a marking stop is refused; method A (pause) batch takes a save too')
def _mark_save(a, st):
    if st['kind'] == 'mark':
        save_log.append(('ms', a.collect_save({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.5}]})))
        a.collect_continue({})
    else:
        a.collect_continue({'points': [{'group_id': 106, 'point_id': 1, 'ra': 0.5}]})
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20)], _mark_save, mode='mark', csv_path='/x/scan_pose_msv_ra_map_20261007_152000.csv')
ms = [x for x in save_log if x[0] == 'ms'][0][1]
check(not ms[0] and 'marking stop' in ms[1], f'save at a marking stop refused: {ms}')
def _pause_save(a, st):
    a.collect_save({'points': [{'group_id': 106, 'point_id': 1, 'readings': [0.3, 0.5]}]})
    a.collect_continue({'points': [{'group_id': 106, 'point_id': 2, 'ra': 0.7}]})
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20), pose_pt(2, -1.30, 0.225)], _pause_save, batch=2,
                              csv_path='/x/scan_pose_psv_ra_map_20261007_153000.csv')
rows = read_csv(cpath('scan_pose_psv_ra_map_20261007_153000', '_ra_measured.csv'))
check([(r['mark_no'], r['point_id'], r['ra_measured'], r['ra_readings']) for r in rows] == [('', '1', '0.4000', '0.3000 0.5000'), ('', '2', '0.7000', '')],
      f'method A batch of 2: p1 from the save (no mark_no), p2 from the release: {rows}')

print('== source_point_id (2026-10-07): in the frame name, the collect CSV and the Ra map')
pts_sp = [pose_pt(11, -1.30, 0.20), joint_pt(99, scan=False), pose_pt(12, -1.30, 0.225)]
pts_sp[0]['source_point_id'] = 1; pts_sp[2]['source_point_id'] = 2      # path rows 11 / 12 = work points 1 / 2
sp_seen = []
def _sp(a, st):
    if st['kind'] == 'mark':
        sp_seen.append((st['mark_no'], st['point_id'], list(st['images']))); a.collect_continue({})
    else:
        sp_seen.append(('pause', [(p['point_id'], p.get('source_point_id'), p['mark_no']) for p in st['points']]))
        a.collect_continue({'points': [{'group_id': 106, 'point_id': 11, 'ra': 0.1}, {'group_id': 106, 'point_id': 12, 'ra': 0.2}]})
robot, ac, seen = run_collect(pts_sp, _sp, mode='mark', csv_path='/x/scan_joint_sp_ra_map_20261007_140000.csv')
check(ac.pipeline.prefixes == ['g106_p11_sp1_i0001', 'g106_p12_sp2_i0003'],
      f'frame prefix carries _sp<source_point_id> between the path row and the index: {ac.pipeline.prefixes}')
check(sp_seen[0] == (1, 11, ['g106_p11_sp1_i0001_s1.png']) and sp_seen[2] == ('pause', [(11, 1, 1), (12, 2, 2)]),
      f'marking stop names the sp frame; the entry table carries source_point_id per point: {sp_seen}')
rows = read_csv(cpath('scan_joint_sp_ra_map_20261007_140000', '_ra_measured.csv'))
check(list(rows[0].keys())[:7] == ['run', 'mark_no', 'index', 'group_id', 'point_id', 'source_point_id', 'images']
      and [(r['point_id'], r['source_point_id'], r['images']) for r in rows] == [('11', '1', 'g106_p11_sp1_i0001_s1.png'), ('12', '2', 'g106_p12_sp2_i0003_s1.png')],
      f'collect CSV: source_point_id column after point_id, filled: {[(r["point_id"], r["source_point_id"]) for r in rows]}')
# a pending row read back keeps it (RESUME lists it with the number)
pend = ac._collect_pending_rows([106])
check(pend == [], 'nothing pending after the entry stop')
robot, ac, seen = run_collect([dict(pts_sp[0])], lambda a, st: (a.collect_continue({}) if st['kind'] == 'mark' else setattr(a, 'cancel_requested', True)),
                              mode='mark', csv_path='/x/scan_joint_sp2_ra_map_20261007_140100.csv')
pend = ac._collect_pending_rows([106])
check(len(pend) == 1 and pend[0]['source_point_id'] == 1 and pend[0]['images'] == ['g106_p11_sp1_i0001_s1.png'],
      f'a pending row read back carries source_point_id and the sp frame name: {pend}')
# a point without a source id (no column in its file): the old name, blank cell
robot, ac, seen = run_collect([pose_pt(5, -1.30, 0.20)], lambda a, st: a.collect_continue({'ra': 0.5}),
                              csv_path='/x/scan_pose_nosp_ra_map_20261007_140200.csv')
rows = read_csv(cpath('scan_pose_nosp_ra_map_20261007_140200', '_ra_measured.csv'))
check(ac.pipeline.prefixes == ['g106_p5_i0001'] and rows[0]['source_point_id'] == '',
      f'no source id -> no _sp token, blank cell: {ac.pipeline.prefixes} {rows[0]["source_point_id"]}')
# the Ra map writer (real class, pandas): column after point_id, integers, filled into an older file
from apriltag_nav.scan_results import ScanResultWriter, COLUMNS as RA_COLS
import tempfile
check(RA_COLS[:4] == ['group_id', 'point_id', 'source_point_id', 'x'], f'Ra map columns: {RA_COLS[:4]}')
td = tempfile.mkdtemp(); ramap = os.path.join(td, 't_ra_map_x.csv')
wtr = ScanResultWriter(); wtr.begin(pts_sp)
wtr.save(ramap, [{'group_id': 106, 'point_id': 11, 'source_point_id': 1, 'success': True, 'execution_message': 'Success',
                  'ra_mean': 0.5, 'ra_std': 0.0, 'ra_min': 0.5, 'ra_max': 0.5, 'num_samples': 1}])
rr = read_csv(ramap)
check([(r['group_id'], r['point_id'], r['source_point_id'], r['success']) for r in rr] == [('106', '11', '1', 'True'), ('106', '12', '2', '')],
      f'Ra map rows carry source_point_id as an integer, the traverse row absent: {[(r["point_id"], r["source_point_id"]) for r in rr]}')
with open(ramap, 'w') as f:                      # an older file without the column, then a resumed save
    f.write('group_id,point_id,x,y,z,ra_mean,ra_std,ra_min,ra_max,num_samples,success,execution_message,validated_at\n106,11,,,,0.5,0,0.5,0.5,1,True,Success,2026-10-07 10:00:00\n')
wtr.save(ramap, [])
rr = read_csv(ramap)
check([(r['point_id'], r['source_point_id']) for r in rr] == [('11', '1'), ('12', '2')] and rr[0]['validated_at'] == '2026-10-07 10:00:00',
      f'an older Ra map gains the column, existing rows filled from the registered points, the rest untouched: {rr}')

print('== method A is untouched by the pending machinery (no pending rows ever exist)')
robot, ac, seen = run_collect([pose_pt(1, -1.30, 0.20)], lambda a, st: a.collect_continue({'ra': 0.5}),
                              csv_path='/x/scan_pose_mkb_ra_map_20261007_130000.csv')
rows = read_csv(cpath('scan_pose_mkb_ra_map_20261007_130000', '_ra_measured.csv'))
check(len(rows) == 1 and rows[0]['ra_measured'] == '0.5000' and rows[0]['mark_no'] == '', f'pause mode: one row, no mark_no: {rows}')

import shutil; shutil.rmtree(COLLECT_DIR, ignore_errors=True)

# ------------------------------------------------------------------
# Arm link / RPC health (2026-10-06): the 13.5 s freeze at point 65 was a
# blocked SDK call during an arm-NIC carrier drop, with no log line.
# ------------------------------------------------------------------
print('== TimedRPC: a blocked SDK call is timed, reported live and counted')
from apriltag_nav.arm_link_watch import TimedRPC, LinkMonitor


class SlowSDK:
    attr = 7

    def __init__(self):
        self.gate = threading.Event()
        self.calls = []

    def GetActualTCPPose(self):
        self.calls.append('GetActualTCPPose')
        self.gate.wait(2.0)
        return 0, [1.0] * 6

    def SetSpeed(self, v):
        self.calls.append(('SetSpeed', v))
        return 0


sdk = SlowSDK()
rpc = TimedRPC(sdk, warn_s=0.15)
check(rpc.attr == 7, 'non-callable attributes pass through')
LOG['warn'].clear()
check(rpc.SetSpeed(30) == 0 and sdk.calls[-1] == ('SetSpeed', 30), 'a call is forwarded with its arguments and result')
check(rpc.health()['rpc_stall_count'] == 0 and not rpc.check() and not LOG['warn'], 'a fast call records nothing')
got = {}
th = threading.Thread(target=lambda: got.setdefault('r', rpc.GetActualTCPPose()))
th.start(); time.sleep(0.05)
infl = rpc.inflight()
check(infl is not None and infl[0] == 'GetActualTCPPose' and infl[1] < 0.15, 'the in-flight call is visible before warn_s, not yet a stall')
check(not rpc.check() and not rpc.health()['rpc_stalled'], 'not stalled before warn_s')
time.sleep(0.2)
check(rpc.check() and rpc.health()['rpc_stalled'], 'stalled once the call is older than warn_s')
live = [m for m in LOG['warn'] if 'has not returned for' in m]
rpc.check(); rpc.health()
check(len([m for m in LOG['warn'] if 'has not returned for' in m]) == 1 and 'GetActualTCPPose' in live[0],
      'the live warning names the method and is logged once per stuck call')
sdk.gate.set(); th.join(2.0)
check(got.get('r') == (0, [1.0] * 6), 'the blocked call returns its real result')
h = rpc.health()
check(not h['rpc_stalled'] and h['rpc_stall_count'] == 1 and h['rpc_last_stall'].startswith('GetActualTCPPose 0.'),
      f"after the return: not stalled, count 1, last stall recorded ({h['rpc_last_stall']})")
done = [m for m in LOG['warn'] if 'blocked' in m]
check(len(done) == 1 and 'GetActualTCPPose' in done[0] and 'reported live above' in done[0],
      'the completion warning names the method and the duration')
check(rpc.inflight() is None, 'nothing in flight afterwards')
sdk.gate.clear(); got.clear()
th = threading.Thread(target=lambda: got.setdefault('r', rpc.GetActualTCPPose())); th.start()
time.sleep(0.05); sdk.gate.set(); th.join(2.0)
check(rpc.health()['rpc_stall_count'] == 1, 'a short call after a stall does not count')

print('== LinkMonitor: carrier transitions from sysfs')
import tempfile
root = tempfile.mkdtemp()
os.makedirs(os.path.join(root, 'armif'))
def set_carrier(v, down=3):
    open(os.path.join(root, 'armif', 'carrier'), 'w').write(f'{v}\n')
    open(os.path.join(root, 'armif', 'carrier_down_count'), 'w').write(f'{down}\n')
set_carrier(1)
clock = [1000.0]
LOG['warn'].clear(); LOG['info'].clear()
lm = LinkMonitor('192.168.58.2', iface='armif', sysfs=root, clock=lambda: clock[0])
st = lm.poll()
check(st == {'link_iface': 'armif', 'link_up': True, 'link_down_count': 0, 'link_down_s': 0.0}, 'up at start, no drop counted')
check(not LOG['warn'], 'no warning while up')
set_carrier(0, 4); clock[0] += 1.0
st = lm.poll()
check(st['link_up'] is False and st['link_down_count'] == 1, 'carrier 0 -> link_up False, drop counted')
check(len(LOG['warn']) == 1 and 'ARM LINK DOWN' in LOG['warn'][0] and '4 since boot' in LOG['warn'][0],
      'one warning naming the drop and the kernel count')
clock[0] += 5.0; lm.poll()
check(len(LOG['warn']) == 1 and lm.state()['link_down_s'] == 5.0, 'still down: no repeat warning, down time counted')
set_carrier(1, 4); clock[0] += 7.0
st = lm.poll()
check(st['link_up'] is True and st['link_down_count'] == 1 and st['link_down_s'] == 0.0, 'carrier back: up, count kept')
check(len(LOG['warn']) == 2 and 'back up after 12.0 s' in LOG['warn'][1], 'recovery logged with the outage duration')
LOG['warn'].clear()
lm2 = LinkMonitor('192.168.58.2', iface='', sysfs=root, clock=lambda: clock[0])
lm2.iface = ''  # resolve_iface may or may not find a route on this machine; force the off case
st = lm2.poll()
check(st['link_iface'] == '' and st['link_up'] is True and st['link_down_count'] == 0, 'no interface: monitor off, link reported up')
lm3 = LinkMonitor('192.168.58.2', iface='gone', sysfs=root, clock=lambda: clock[0])
LOG['warn'].clear()
st = lm3.poll(); lm3.poll()
check(st['link_up'] is True and len(LOG['warn']) == 1 and 'cannot read' in LOG['warn'][0],
      'unreadable carrier: warned once, link not flagged down')
check(LinkMonitor.resolve_iface('256.1.1.1') == '', 'an unroutable address resolves to no interface')
shutil.rmtree(root, ignore_errors=True)

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
