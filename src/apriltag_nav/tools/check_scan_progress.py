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

    def process(self, point_id, frames, cancelled=None):
        # Runs on the controller's inference worker thread.
        self.processed_on.append(threading.current_thread().name)
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

    def MoveJ(self, joints, tool, user):
        self.calls.append(('MoveJ', tool, tuple(round(v, 1) for v in joints)))
        if self.movej_error:
            return self.movej_error
        self.joints = list(joints)
        self.pose = [self.pose[0] + 1.0] + self.pose[1:]
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
check('2.56 m from the arm base' in msg and '(-1714, 1896, -267) mm' in msg,
      'message names the arm-frame target and its distance from the base')
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
def _flaky(point_id, frames, cancelled=None):
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

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
