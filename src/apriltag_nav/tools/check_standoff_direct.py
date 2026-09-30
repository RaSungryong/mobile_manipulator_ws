#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline check of the standoff loop's DIRECT move (2026-09-29): the whole
measured gap in one computed move, verified at rest, with its safeguards.

Part A — keyence_standoff.StandoffController against a surface plant
  (sensitivity = reading change per executed mm, sample noise, frozen
  sensor, guard-stop / executed-distance reports).
Part B — the real ArmController methods (no __init__) against a fake
  Fairino whose MoveL moves in small timed increments and ends a constant
  offset from its target, with a feeder thread publishing /keyence/value
  from the fake's live pose: command chain, offset learning and carry,
  live guard.

Run:  python3 tools/check_standoff_direct.py
"""
import os
import random
import sys
import threading
import time
import types

import numpy as np

# ---------------------------------------------------------------- stubs
LOG = {'info': [], 'warn': [], 'err': []}
rospy = types.ModuleType('rospy')
rospy.loginfo = lambda m, *a: LOG['info'].append(str(m))
rospy.logwarn = lambda m, *a: LOG['warn'].append(str(m))
rospy.logerr = lambda m, *a: LOG['err'].append(str(m))
rospy.logdebug = lambda m, *a: None
rospy.logwarn_throttle = lambda t, m, *a: None
rospy.logerr_throttle = lambda t, m, *a: None
rospy.loginfo_throttle = lambda t, m, *a: None
rospy.get_param = lambda name, default=None: default
rospy.get_time = lambda: time.time()
rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda m: None)
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
sp = types.ModuleType('apriltag_nav.scan_pipeline'); sp.RaScanPipeline = object
sys.modules['apriltag_nav.scan_pipeline'] = sp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'src'))
from apriltag_nav.keyence_standoff import (StandoffController, StandoffConfig,  # noqa: E402
                                            MoveReport)
from apriltag_nav.arm_controller import ArmController                           # noqa: E402

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1; print(f'  ok   {what}')
    else:
        N_FAIL += 1; print(f'  FAIL {what}')


ZERO, NEAR, FAR = 16.5, 6.5, 27.0
SENT = 100000.0


# ================================================================ part A
class Plant:
    """Tool case at standoff s over a surface. The reading changes by
    `k` mm per executed mm (k != 1 = the oblique beam's spot walk)."""
    def __init__(self, s0, k=1.0, noise=0.0, frozen=False, exec_gain=1.0,
                 report=True, guard_at=None, k_seq=None, seed=1):
        self.s = float(s0)
        self.read_perp = ZERO - self.s          # what the sensor shows
        self.k = k; self.k_seq = list(k_seq or [])
        self.noise = noise; self.frozen = frozen
        self.exec_gain = exec_gain; self.report = report
        self.guard_at = guard_at                # perp reading the guard stops at
        self.moves = []; self.min_s = self.s
        self.max_read = self.read_perp          # furthest PAST the target, as read
        self.rng = random.Random(seed)

    def read(self, n, timeout):
        if self.s > FAR:
            return [-SENT] * n
        if self.s < NEAR:
            return [SENT] * n
        return [self.read_perp + self.rng.uniform(-self.noise, self.noise)
                for _ in range(n)]

    def move(self, approach):
        k = self.k_seq.pop(0) if self.k_seq else self.k
        done = approach * self.exec_gain
        stopped = False
        if self.guard_at is not None and approach > 0 and not self.frozen:
            # The guard stops the move where the reading passes guard_at.
            reach = (self.guard_at - self.read_perp) / k
            if 0 <= reach < done:
                done = reach + 0.3                  # reaction distance
                stopped = True
        self.s -= done
        self.min_s = min(self.min_s, self.s)
        if not self.frozen:
            self.read_perp += k * done
        self.max_read = max(self.max_read, self.read_perp)
        self.moves.append(approach)
        if not self.report:
            return True
        return MoveReport(True, executed_mm=done, guard_stop=stopped,
                          note='guard' if stopped else '')


def cfg(**kw):
    base = dict(tolerance_mm=0.2, kp=0.8, max_steps=15, max_step_mm=1.0,
                approach_fraction=0.5, retreat_step_mm=3.0,
                activate_threshold_mm=45.0, max_travel_mm=25.0,
                invalid_abs_mm=90.0, samples=5, move_mode='direct')
    base.update(kw)
    return StandoffConfig(**base)


def run(plant, c):
    return StandoffController(c, plant.read, plant.move).run()


print('== A1. direct: the measured gap in one move')
p = Plant(ZERO + 5.27); r = run(p, cfg())
check(r.converged and len(p.moves) == 1 and abs(p.moves[0] - 5.27) < 1e-9,
      f'5.27 mm too far, sensitivity 1.0: ONE approach of {p.moves[0]:.2f} mm, converged')
p = Plant(ZERO + 5.27); r = run(p, cfg(move_mode='stepped'))
check(r.converged and len(p.moves) >= 4 and abs(p.moves[0] - 2.635) < 1e-3,
      f'stepped law on the same start: {len(p.moves)} moves, first {p.moves[0]:.3f} mm '
      '(the 2026-09-29 log: 2.634)')
check(StandoffConfig().move_mode == 'stepped',
      'the dataclass default stays stepped (robot.yaml selects direct)')
p = Plant(ZERO - 5.0); r = run(p, cfg())
check(r.converged and p.moves == [-5.0],
      'too close by 5 mm: ONE retreat of 5 mm (the stepped cap was 3)')
p = Plant(ZERO + 0.1); r = run(p, cfg())
check(r.converged and p.moves == [], 'inside the tolerance: no move')

print('== A2. measured sensitivities (0.83 .. 1.04): main move + one trim')
for k in (0.83, 0.93, 1.0, 1.04):
    p = Plant(ZERO + 5.3, k=k); r = run(p, cfg())
    # The READING is the reference (the Basler focus is at reading 0), so
    # "how far past the target" is judged on it, not on the plant's s.
    check(r.converged and len(p.moves) <= 2 and p.max_read < 0.5,
          f'k {k}: {len(p.moves)} move(s) {[round(m, 2) for m in p.moves]}, '
          f'furthest past the target {max(p.max_read, 0.0):.2f} mm')
n_d = n_s = 0; worst = 0; past = -99.0; N = 300
rng = random.Random(7)
for i in range(N):
    gap = rng.uniform(2.0, 8.0)
    ks = [rng.uniform(0.83, 1.04) for _ in range(12)]
    p = Plant(ZERO + gap, k_seq=list(ks), noise=0.01, seed=i); r = run(p, cfg())
    q = Plant(ZERO + gap, k_seq=list(ks), noise=0.01, seed=i); r2 = run(q, cfg(move_mode='stepped'))
    if not (r.converged and r2.converged):
        worst = 99
    n_d += len(p.moves); n_s += len(q.moves)
    worst = max(worst, len(p.moves)); past = max(past, p.max_read)
check(worst <= 3 and n_d / N < 2.0,
      f'{N} random starts 2-8 mm, sensitivity 0.83-1.04 per move: direct '
      f'{n_d / N:.2f} moves/point (worst {worst}), stepped {n_s / N:.2f}')
check(past < 0.5, f'furthest past the target over all of them: {past:.2f} mm')

print('== A3. safeguards before the move')
p = Plant(ZERO + 5.0, noise=0.4); r = run(p, cfg())
check(abs(p.moves[0]) <= 2.6,
      f'noisy decision (spread > 0.3 mm): first move is stepped, {p.moves[0]:.2f} mm')
p = Plant(ZERO + 10.0); r = run(p, cfg(direct_max_approach_mm=6.0))
check(abs(p.moves[0] - 6.0) < 1e-9 and r.converged and len(p.moves) == 2,
      f'one approach move is capped: {[round(m, 2) for m in p.moves]}')
p = Plant(ZERO + 9.0); r = run(p, cfg(max_travel_mm=5.0))
check(not r.converged and p.moves == [] and 'travel budget' in r.reason,
      'a move beyond the travel budget is not started')
p = Plant(ZERO + 5.0, k=1.6); r = run(p, cfg())
check(r.converged and p.moves[1] < 0 and abs(p.moves[1]) <= 3.0,
      f'surface reads 1.6x: the overshoot is retreated ({[round(m, 2) for m in p.moves]})')
p = Plant(ZERO + 5.0, k=1.6, k_seq=[0.6, 1.6, 1.6, 1.6]); r = run(p, cfg())
check(all(m <= 5.0 + 1e-9 for m in p.moves) and p.moves[1] <= (5.0 - 0.6 * 5.0) + 1e-9,
      'an approach is never amplified by a low measured sensitivity '
      f'({[round(m, 2) for m in p.moves]})')

print('== A4. safeguards after the move')
p = Plant(ZERO + 5.0, frozen=True); r = run(p, cfg())
check(not r.converged and len(p.moves) == 1 and 'does not follow' in r.reason,
      f'frozen sensor: ONE move, then stop ({r.reason[:48]}...)')
p = Plant(ZERO + 5.0, exec_gain=0.02); r = run(p, cfg())
check(not r.converged and len(p.moves) == 1 and 'arm executed' in r.reason,
      f'arm did not execute the move: stop ({r.reason})')
p = Plant(ZERO + 5.0, report=False); r = run(p, cfg())
check(r.converged and len(p.moves) == 1, 'a move() that returns a bare True still works')
p = Plant(ZERO + 0.6, k_seq=[0.1, 1.0, 1.0, 1.0]); r = run(p, cfg())
check(r.converged and len(p.moves) >= 2,
      'a SHORT trim the reading did not follow: stepped from there, converges')

print('== A5. live guard reports')
p = Plant(ZERO + 6.0, k=1.5, guard_at=1.0); r = run(p, cfg())
hist = [h for h in r.history if h.get('guard_stop')]
check(len(hist) == 1 and r.converged and p.moves[1] < 0,
      f'guard stop mid-move: re-measured, retreated, converged '
      f'({[round(m, 2) for m in p.moves]})')
check(all(h.get('mode') == 'stepped' for h in r.history[1:] if 'mode' in h),
      'after a guard stop the rest of the adjustment is stepped')
check(p.min_s > ZERO - 2.0, f'closest {p.min_s:.2f} mm (target {ZERO}, guard margin 1.0 + reaction)')


class TwoStops(Plant):
    def move(self, approach):
        rep = super().move(approach)
        if approach > 0:
            rep.guard_stop = True
        return rep


p = TwoStops(ZERO + 6.0); p.read_perp = -6.0
# the reading never improves, so every approach is stopped again
p.frozen = True
r = StandoffController(cfg(min_response_ratio=-9.0), p.read, p.move).run()
check(not r.converged and 'live guard' in r.reason and len(p.moves) == 2,
      f'second guard stop ends the adjustment ({r.reason[:46]}...)')

# ================================================================ part B
print('== B. ArmController: command chain, offset, live guard')

DELTA = np.array([0.2, -0.3, -0.4, -0.06, -0.05, 0.0])   # MoveL ends here vs target


class FakeRobot:
    """Tool straight down (rx 180): tool Z = base -Z. z_surface is where
    the standoff would be 0."""
    def __init__(self, z0, delta=DELTA, speed_mm_s=200.0, fail=0):
        self.pose = [100.0, 700.0, z0, 180.0, 0.0, 0.0]
        self.delta = np.array(delta, dtype=float)
        self.speed = speed_mm_s
        self.targets = []
        self.stops = 0
        self._stop = threading.Event()
        self.fail = fail
        self.moving = False

    def SetSpeed(self, v):
        return 0

    def GetActualTCPPose(self):
        return 0, list(self.pose)

    def GetActualJointPosDegree(self):
        return 0, [0.0] * 6

    def StopMotion(self):
        self.stops += 1
        if self.stops % 2 == 1 and self.moving:
            raise RuntimeError('Request-sent')      # the socket is held
        self._stop.set()
        return 0

    def MoveL(self, target, tool=0, user=0, **kw):
        if self.fail:
            return self.fail
        self.targets.append(list(target))
        self._stop.clear()
        self.moving = True
        start = np.array(self.pose, dtype=float)
        end = np.array(target, dtype=float) + self.delta
        dist = float(np.linalg.norm(end[:3] - start[:3]))
        n = max(1, int(dist / self.speed / 0.002))
        for i in range(1, n + 1):
            if self._stop.is_set():
                self.moving = False
                return 0
            self.pose = list(start + (end - start) * i / n)
            time.sleep(0.002)
        self.moving = False
        return 0


class Surface:
    """Standoff of the case = (z - z_surface); an optional step `bump_mm`
    high that comes under the beam once the tool is below `bump_below_z`."""
    def __init__(self, robot, z_surface, cosb, k=1.0, bump_mm=0.0, bump_below_z=None):
        self.robot = robot; self.z_surface = z_surface; self.cosb = cosb
        self.k = k; self.bump_mm = bump_mm; self.bump_below_z = bump_below_z
        self.min_standoff = 1e9
        self.z_ref = robot.pose[2]

    def raw(self):
        z = self.robot.pose[2]
        standoff = z - self.z_surface
        if self.bump_below_z is not None and z < self.bump_below_z:
            standoff -= self.bump_mm
        self.min_standoff = min(self.min_standoff, standoff)
        # sensitivity k about the pose the adjustment started from
        s_ref = self.z_ref - self.z_surface
        s_eff = s_ref + self.k * (standoff - s_ref)
        if s_eff > FAR:
            return -SENT
        if s_eff < NEAR:
            return SENT
        return (ZERO - s_eff) / self.cosb


def make(robot, **kw):
    ac = ArmController.__new__(ArmController)
    ac.robot = robot
    ac.cancel_requested = False
    ac.cancel_attempts = 10; ac.cancel_retry_s = 0.002
    ac.keyence_tol = 0.2; ac.keyence_dir = -1.0; ac.keyence_kp = 0.8
    ac.keyence_max_steps = 15; ac.keyence_max_step_mm = 1.0
    ac.keyence_activate_threshold = 45.0; ac.keyence_approach_fraction = 0.5
    ac.keyence_retreat_step_mm = 3.0; ac.keyence_max_travel_mm = 25.0
    ac.keyence_invalid_abs_mm = 90.0; ac.keyence_samples = 5
    ac.keyence_read_timeout_s = 1.0; ac.keyence_settle_s = 0.01
    ac.keyence_adaptive_gain = True; ac.keyence_gain_ratio_max = 4.0
    ac.keyence_min_response_ratio = 0.25
    ac.keyence_seek_enabled = False; ac.keyence_seek_step_mm = 5.0
    ac.keyence_seek_max_mm = 40.0
    ac.keyence_sensor_zero_mm = ZERO; ac.keyence_target_distance_mm = ZERO
    ac.keyence_setpoint_mm = 0.0
    ac._keyence_cos = float(np.cos(np.radians(37.1)))
    ac._keyence_lock = threading.Lock()
    ac.current_keyence_val = None; ac._keyence_seq = 0
    ac.keyence_move_mode = 'direct'; ac.keyence_direct_gain = 1.0
    ac.keyence_direct_max_approach_mm = 12.0; ac.keyence_direct_max_retreat_mm = 15.0
    ac.keyence_direct_max_spread_mm = 0.3; ac.keyence_direct_max_moves = 5
    ac.keyence_exec_mismatch_mm = 1.0
    ac.keyence_guard_enabled = True; ac.keyence_guard_overshoot_mm = 1.0
    ac.keyence_guard_frames = 2; ac.keyence_max_guard_stops = 1
    ac.keyence_cmd_bias_enabled = True; ac.keyence_cmd_bias_max_mm = 1.5
    ac.keyence_cmd_bias_max_deg = 0.3; ac.keyence_cmd_bias_carry_mm = 100.0
    ac.keyence_cmd_bias_carry_s = 120.0
    ac._standoff_guard = None; ac._standoff_chain = None; ac._standoff_bias = None
    ac._live_lock = threading.Lock()
    ac._live_pose = None; ac._live_joints = None; ac._live_stamp = 0.0
    for k, v in kw.items():
        setattr(ac, k, v)
    return ac


class Feeder:
    """/keyence/value at ~500 Hz from the fake's live pose."""
    def __init__(self, ac, surface):
        self.ac = ac; self.surface = surface; self.run = True
        self.t = threading.Thread(target=self._loop, daemon=True); self.t.start()

    def _loop(self):
        while self.run:
            self.ac.keyence_cb(_Msg(self.surface.raw()))
            time.sleep(0.002)

    def stop(self):
        self.run = False; self.t.join(timeout=1.0)


Z_SURF = -600.0

# ---- B1: chain + offset
rb = FakeRobot(Z_SURF + ZERO + 5.0)
ac = make(rb)
sf = Surface(rb, Z_SURF, ac._keyence_cos)
fd = Feeder(ac, sf)
res = ac._adjust_distance_to_surface()
final = rb.pose[2] - Z_SURF
check(res.converged and abs(final - ZERO) <= 0.25,
      f'MoveL ends {np.linalg.norm(DELTA[:3]):.2f} mm off its target: '
      f'converged in {res.steps} move(s) at {final:.2f} mm')
check(res.steps <= 3, f'{res.steps} moves (the readback-chained loop never settled here)')
t = rb.targets
if len(t) >= 2:
    d = np.array(t[1]) - np.array(t[0])
    check(abs(d[0]) < 1e-9 and abs(d[1]) < 1e-9 and np.allclose(t[1][3:], t[0][3:]),
          f'move 2 = COMMAND 1 + dz along tool Z, same orientation (dz {d[2]:+.3f} mm)')
    h = [x for x in res.history if 'cmd_mm' in x]
    check(abs(h[1]['executed_mm'] - h[1]['cmd_mm']) < 0.02,
          f"the trim executes what it was asked: cmd {h[1]['cmd_mm']:+.3f} "
          f"-> {h[1]['executed_mm']:+.3f} mm")
    check(abs(h[0]['executed_mm'] - (h[0]['cmd_mm'] + 0.4)) < 0.02,
          f"first move carries the offset once: cmd {h[0]['cmd_mm']:+.3f} "
          f"-> {h[0]['executed_mm']:+.3f} mm")
else:
    check(False, 'expected two moves'); check(False, '-'); check(False, '-')
b = ac._standoff_bias
check(b is not None and np.allclose(b['delta'], DELTA, atol=1e-6),
      f"offset learned from the readback: {np.round(b['delta'], 3).tolist() if b else None}")
check(ac._standoff_chain is None and ac._standoff_guard is None,
      'chain and guard cleared when the adjustment ends')

# ---- B2: next point nearby: the carried offset makes the first move exact
rb.pose = [130.0, 700.0, Z_SURF + ZERO + 4.0, 180.0, 0.0, 0.0]
sf.z_ref = rb.pose[2]; rb.targets.clear()
time.sleep(0.05)
p_before = np.array(rb.pose[:3])
res = ac._adjust_distance_to_surface()
final = rb.pose[2] - Z_SURF
check(res.converged and res.steps == 1 and abs(final - ZERO) <= 0.05,
      f'next point 30 mm away: ONE move, lands at {final:.3f} mm (offset pre-compensated)')
check(abs(rb.pose[0] - p_before[0]) < 0.01 and abs(rb.pose[1] - p_before[1]) < 0.01
      and abs((rb.pose[3] - 180.0 + 180.0) % 360.0 - 180.0) < 1e-3
      and abs(rb.pose[4]) < 1e-3,
      'and the tool did not drift sideways or in orientation')
t1 = np.array(rb.targets[0][:3])
check(np.allclose(t1, p_before - DELTA[:3] + np.array([0, 0, -4.0]), atol=0.05),
      'first command = readback - offset + dz')

# ---- B3: carry refused when far / stale, offset too large not learned
rb.pose = [400.0, 700.0, Z_SURF + ZERO + 4.0, 180.0, 0.0, 0.0]
check(np.allclose(ac._standoff_carried_bias(rb.pose[:3]), 0.0),
      'an offset learned 270 mm away is not applied')
ac._standoff_bias['stamp'] -= 500.0
check(np.allclose(ac._standoff_carried_bias(ac._standoff_bias['pos']), 0.0),
      'a stale offset is not applied')
fd.stop()
rb = FakeRobot(Z_SURF + ZERO + 3.0, delta=[0.0, 0.0, -2.5, 0.0, 0.0, 0.0])
ac = make(rb); sf = Surface(rb, Z_SURF, ac._keyence_cos); fd = Feeder(ac, sf)
LOG['warn'].clear()
rep = ac._keyence_move_approach(1.0)
check(rep.ok and ac._standoff_bias is None
      and any('offset not learned' in w for w in LOG['warn']),
      'a 2.5 mm miss is not a settle offset: not learned, warned')
fd.stop()

# ---- B4: live guard — a raised edge comes under the beam mid-move
def guard_run(enabled):
    rb = FakeRobot(Z_SURF + ZERO + 8.0, delta=np.zeros(6), speed_mm_s=40.0)
    ac = make(rb, keyence_guard_enabled=enabled)
    # 5 mm step that appears after 3 mm of descent: the first reading
    # (8 mm too far) is wrong by 5 mm for the rest of the move.
    sf = Surface(rb, Z_SURF, ac._keyence_cos, bump_mm=5.0,
                 bump_below_z=Z_SURF + ZERO + 5.0)
    fd = Feeder(ac, sf)
    LOG['warn'].clear()
    res = ac._adjust_distance_to_surface()
    fd.stop()
    return rb, ac, sf, res


rb, ac, sf, res = guard_run(True)
g_min = sf.min_standoff
check(rb.stops >= 1 and any(h.get('guard_stop') for h in res.history),
      f'guard stopped the move in flight (StopMotion x{rb.stops}, first rejected, retried)')
check(res.converged and abs((rb.pose[2] - Z_SURF - 5.0) - ZERO) <= 0.25,
      f'then re-measured and converged on the raised surface in {res.steps} moves')
check(g_min > ZERO - 2.5, f'closest with the guard: {g_min:.2f} mm (target {ZERO})')
check(not ac.cancel_requested, 'the guard stop is not a cancel: the scan goes on')
rb2, ac2, sf2, res2 = guard_run(False)
check(sf2.min_standoff < g_min - 1.5,
      f'without the guard the same move reaches {sf2.min_standoff:.2f} mm')

# ---- B5: guard scope
rb = FakeRobot(Z_SURF + ZERO - 4.0, delta=np.zeros(6))
ac = make(rb); sf = Surface(rb, Z_SURF, ac._keyence_cos); fd = Feeder(ac, sf)
res = ac._adjust_distance_to_surface()
check(res.converged and res.steps == 1 and rb.stops == 0,
      'a retreat is not guarded (4 mm too close: one move, no stop)')
ac.keyence_cb(_Msg(SENT)); ac.keyence_cb(_Msg(SENT)); ac.keyence_cb(_Msg(5.0 / ac._keyence_cos))
check(rb.stops == 0, 'no guard outside a standoff move: readings never stop the arm')
fd.stop()
ac._standoff_guard = {'limit': 1.0, 'frames': 2, 'count': 0, 'tripped': False,
                      'why': '', 'stopped': None}
ac.keyence_cb(_Msg(-SENT)); ac.keyence_cb(_Msg(-SENT)); ac.keyence_cb(_Msg(-SENT))
check(rb.stops == 0, 'the FAR sentinel does not trip the guard')
ac.keyence_cb(_Msg(2.0 / ac._keyence_cos)); ac.keyence_cb(_Msg(0.5 / ac._keyence_cos))
ac.keyence_cb(_Msg(2.0 / ac._keyence_cos))
check(rb.stops == 0, 'one reading beyond the limit is not enough (frames 2)')
ac.keyence_cb(_Msg(SENT)); 
check(rb.stops >= 1 and ac._standoff_guard['tripped'],
      'two in a row (reading, then the too-close sentinel) stop the arm')
n = rb.stops
ac.keyence_cb(_Msg(SENT)); ac.keyence_cb(_Msg(SENT))
check(rb.stops == n, 'a tripped guard stops once')
ac._standoff_guard = None

# ---- B6: target override moves the guard limit; MoveL failure
rb = FakeRobot(Z_SURF + 20.0 + 3.0, delta=np.zeros(6))
ac = make(rb); sf = Surface(rb, Z_SURF, ac._keyence_cos); fd = Feeder(ac, sf)
seen = {}
orig = rb.MoveL
def spy(target, **kw):
    seen['limit'] = ac._standoff_guard['limit'] if ac._standoff_guard else None
    return orig(target, **kw)
rb.MoveL = spy
ok, msg, res = ac.adjust_standoff(20.0)
check(ok and abs(seen['limit'] - (ZERO - 20.0 + 1.0)) < 1e-9
      and abs(rb.pose[2] - Z_SURF - 20.0) <= 0.2,
      f"Auto standoff target 20 mm: guard limit {seen['limit']:+.2f} mm perpendicular, {msg[:40]}")
fd.stop()
rb = FakeRobot(Z_SURF + ZERO + 3.0, fail=112)
ac = make(rb); sf = Surface(rb, Z_SURF, ac._keyence_cos); fd = Feeder(ac, sf)
res = ac._adjust_distance_to_surface()
check(not res.converged and 'move failed' in res.reason and 'MoveL 112' in res.reason,
      f'MoveL error ends the adjustment: {res.reason}')
fd.stop()

print(f'\n{N_OK} ok, {N_FAIL} failed')
sys.exit(1 if N_FAIL else 0)
