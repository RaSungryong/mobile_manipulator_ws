#!/usr/bin/env python3
"""Offline checks of MobileController's hop sequencing and the re-seat /
aim / align behaviour on a small 2-D plant (no ROS master needed).

    python3 src/apriltag_nav/tools/check_nav_sequencing.py

Stubs rospy and the message packages, loads the real MobileController with
the real robot.yaml, and drives it against a unicycle plant with the
robot's 0.55 s command delay and a simulated front_cam (lens 0.55 m ahead
of the base centre, image right = forward, image down = right, 0.1 s
image latency, whole-tag visibility, bumper occlusion behind the lens).
Exit status 1 on failure. The scratch suites of 2026-09-08 (t_pivot.py,
t_plan.py) were lost with their session; this is the repo-resident subset.
"""
import collections
import copy
import math
import os
import sys
import types

import numpy as np

WS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(WS, 'src'))

# ---------------------------------------------------------------- stubs
class _Clock:
    t = 1000.0


CLK = _Clock()


class Duration:
    def __init__(self, s): self.s = float(s)
    def to_sec(self): return self.s


class Time:
    def __init__(self, s): self.s = float(s)
    @staticmethod
    def now(): return Time(CLK.t)
    def to_sec(self): return self.s
    def __sub__(self, o): return Duration(self.s - o.s)
    def __add__(self, d): return Time(self.s + d.s)
    def __lt__(self, o): return self.s < o.s
    def __le__(self, o): return self.s <= o.s
    def __gt__(self, o): return self.s > o.s
    def __ge__(self, o): return self.s >= o.s


STEP_HOOK = [None]
LOG = []


class Rate:
    def __init__(self, hz): self.dt = 1.0 / hz
    def sleep(self):
        CLK.t += self.dt
        if STEP_HOOK[0]:
            STEP_HOOK[0](self.dt)


def _log(*a, **k):
    try:
        msg = a[0] % a[1:] if len(a) > 1 else a[0]
    except Exception:
        msg = ' '.join(str(x) for x in a)
    LOG.append(str(msg))


rospy = types.ModuleType('rospy')
rospy.Time, rospy.Duration, rospy.Rate = Time, Duration, Rate
rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda m: None)
rospy.Subscriber = lambda *a, **k: None
rospy.is_shutdown = lambda: False
for n in ('loginfo', 'logwarn', 'logerr', 'logdebug'):
    setattr(rospy, n, _log)
    setattr(rospy, n + '_throttle', lambda period, *a, **k: _log(*a))
rospy.get_param = lambda k, d=None: d
sys.modules['rospy'] = rospy
tf = types.ModuleType('tf'); tft = types.ModuleType('tf.transformations')
tft.euler_from_quaternion = lambda q: (0, 0, 0); tf.transformations = tft
sys.modules['tf'] = tf; sys.modules['tf.transformations'] = tft


class _Msg:
    def __init__(self, *a, **k):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.header = types.SimpleNamespace(stamp=None, frame_id='')


def _msgmod(name, *classes):
    m = types.ModuleType(name)
    for c in classes:
        setattr(m, c, type(c, (_Msg,), {}))
    sys.modules[name] = m


_msgmod('geometry_msgs'); _msgmod('geometry_msgs.msg', 'Twist')
_msgmod('sensor_msgs'); _msgmod('sensor_msgs.msg', 'CameraInfo')
_msgmod('nav_msgs'); _msgmod('nav_msgs.msg', 'Odometry')
_msgmod('std_msgs'); _msgmod('std_msgs.msg', 'Bool')
_msgmod('robot_msgs'); _msgmod('robot_msgs.msg', 'Pose2DWithFlag', 'AprilTagDetectionArray')

import yaml  # noqa: E402
from apriltag_nav.mobile_controller import MobileController  # noqa: E402

CFG0 = yaml.safe_load(open(os.path.join(WS, 'config', 'robot.yaml')))
CFG0['robot']['predictive_centering']['record_alignment_result'] = False
CFG0['robot']['predictive_centering']['enabled'] = False
FX, FY, CX, CY, Z = 750.0, 750.0, 638.2, 353.0, 0.302
CAM = 0.55


def wrap(a): return math.atan2(math.sin(a), math.cos(a))


class Plant:
    """Base centre (x, y, psi) in a world whose lane is the x axis. Commands
    execute `delay` s late. front_cam over the lens at base + 0.55 along
    the heading. `tags` maps id -> (X, Y) or (X, Y, heading_rad): the tag
    is laid with its edge along `heading` (0 = the x lane; pi/2 for a tag
    on a perpendicular lane, e.g. a pivot's exit tag), so the edge angle the
    camera reads is psi - heading. A tag is visible when the whole tag is in
    the 1280x720 frame and not hidden by the bumper (tx > -0.03)."""

    def __init__(self, ctrl, x, y, psi, tags, delay=0.55, latency=0.10):
        self.c = ctrl; self.x, self.y, self.psi = x, y, psi
        self.tags = tags; self.delay = delay; self.latency = latency
        self.q = collections.deque(); self.last = (0.0, 0.0)
        self.hist = collections.deque()
        self.odom_x = self.odom_y = self.odom_psi = 0.0
        real_pub = ctrl._publish_vel

        def pub(lin, ang):
            self.q.append((CLK.t, float(lin), float(ang))); real_pub(lin, ang)
        ctrl._publish_vel = pub
        ctrl.camera_params = [FX, FY, CX, CY]
        self.push_odom(); self.push_cam()

    def cmd(self):
        while self.q and self.q[0][0] <= CLK.t - self.delay:
            _, v, w = self.q.popleft(); self.last = (v, w)
        return self.last

    def step(self, dt):
        v, w = self.cmd()
        self.psi = wrap(self.psi + w * dt)
        self.x += v * math.cos(self.psi) * dt; self.y += v * math.sin(self.psi) * dt
        self.odom_psi = wrap(self.odom_psi + w * dt)
        self.odom_x += v * math.cos(self.odom_psi) * dt; self.odom_y += v * math.sin(self.odom_psi) * dt
        self.push_odom(); self.push_cam()

    def push_odom(self):
        self.c.odom_x, self.c.odom_y, self.c.current_theta = self.odom_x, self.odom_y, self.odom_psi
        self.c.odom_stamp = Time(CLK.t); self.c.odom_v = self.last[0]

    def lens(self):
        return self.x + CAM * math.cos(self.psi), self.y + CAM * math.sin(self.psi)

    def view(self, x, y, psi):
        lx, ly = x + CAM * math.cos(psi), y + CAM * math.sin(psi)
        dets = {}
        for tid, spec in self.tags.items():
            X, Y = spec[0], spec[1]
            hdg = spec[2] if len(spec) > 2 else 0.0
            dx, dy = X - lx, Y - ly
            tx = dx * math.cos(psi) + dy * math.sin(psi)
            ty = dx * math.sin(psi) - dy * math.cos(psi)
            cx = CX + tx * FX / Z; cy = CY + ty * FY / Z
            half = 0.045 * FX / Z + 15
            if abs(cx - CX) > 640 - half or abs(cy - CY) > 360 - half or tx < -0.03:
                continue
            a = math.radians(math.degrees(wrap(psi - hdg)) + 90.0)
            c = np.array([[0, 0], [math.cos(a), math.sin(a)], [math.cos(a) - math.sin(a), math.sin(a) + math.cos(a)],
                          [-math.sin(a), math.cos(a)]]) * (0.09 * FX / Z) + np.array([cx, cy])
            dets[tid] = {'x': tx, 'y': ty, 'z': Z, 'corners': c, 'center_x': cx, 'center_y': cy}
        return dets

    def push_cam(self):
        self.hist.append((CLK.t, self.x, self.y, self.psi))
        while len(self.hist) > 1 and self.hist[0][0] < CLK.t - self.latency - 0.05:
            self.hist.popleft()
        t_img, x, y, psi = self.hist[0]
        self.c._store_detections(self.view(x, y, psi), t_img)


class FakeMap:
    def __init__(self): self.edges = {}
    def get_edge(self, a, b): return self.edges.get((a, b))
    def get_tag_info(self, t): return {'x': 0.0, 'y': 0.0, 'zone': 'A'}
    def find_path(self, a, b): return self.path


def make(x, y, psi, tags, cfg=None):
    CLK.t = 1000.0; LOG.clear()
    c = MobileController(copy.deepcopy(cfg or CFG0), FakeMap())
    c.stop_requested = False
    c.publish_robot_pose = lambda tid: None
    p = Plant(c, x, y, psi, tags)
    STEP_HOOK[0] = p.step
    c._last_tag_depth_m = Z
    return c, p


checks = []


def check(name, cond, detail=''):
    checks.append(bool(cond))
    print(('PASS ' if cond else 'FAIL ') + name + (f'  [{detail}]' if detail else ''))


def spy(c, calls):
    real_align, real_pivot, real_pp = c.align_to_tag, c.execute_pivot, c.execute_pure_pursuit
    c.align_to_tag = lambda tid: (calls.append(('align', tid)) or real_align(tid))
    c.execute_pivot = lambda d, exit_tag_id=None: (calls.append(('pivot', d, exit_tag_id)) or real_pivot(d, exit_tag_id=exit_tag_id))

    def pp(**kw):
        calls.append(('pp', kw['target_id'], kw['direction'], round(kw['total_distance'], 3)))
        return real_pp(**kw)
    c.execute_pure_pursuit = pp


def main():
    # ---- A. re-seat: reverse arrival (tag on the REV column) then a forward hop
    # Lens rests 0.16 m short of tag 106 (reverse column); base 0.55 behind the lens; next hop forward to 107 (0.40 m).
    c, p = make(0.40 - 0.16 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 107: (0.80, 0.0)})
    c.map_mgr.edges[(106, 107)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.80, 'y': 0.0, 'zone': 'A'}
    calls = []; spy(c, calls)
    fore0 = c.detected_tags[106]['x']
    ok = c.go_to_next_tag(107, known_start_id=106)
    for _ in range(20): Rate(20).sleep()
    lens_x = p.lens()[0]
    check('A1 reverse-column start seen 0.16 m ahead', abs(fore0 - 0.16) < 0.01, f'{fore0:.3f}')
    check('A2 forward hop from a reverse arrival: reseat pp(106 fwd, ~0.16) -> align(106) -> pp(107 fwd 0.40) -> align(107)',
          ok and len(calls) == 4 and calls[0][:3] == ('pp', 106, 'forward') and abs(calls[0][3] - fore0) < 0.01
          and calls[1] == ('align', 106) and calls[2][:3] == ('pp', 107, 'forward') and abs(calls[2][3] - 0.40) < 0.02 and calls[3] == ('align', 107),
          str(calls))
    aims = [l for l in LOG if l.startswith('[Aim] at rest')]
    check('A2b the reseat used the same arrival algorithm (an aim at rest for tag 106, then one for 107)',
          len(aims) == 2 and 'tag (0.1' in aims[0], str([a[:60] for a in aims]))
    check('A3 hop ends with the lens over tag 107 within 5 mm', abs(lens_x - 0.80) < 0.005 and abs(p.lens()[1]) < 0.005,
          f'lens ({lens_x:.4f}, {p.lens()[1]:+.4f})')
    reseat_arr = [l for l in LOG if l.startswith('[Reseat]')]
    check('A4 reseat logged once', len(reseat_arr) == 1)

    # ---- B. no re-seat when the start tag is on the crosshair (forward arrival)
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 107: (0.80, 0.0)})
    c.map_mgr.edges[(106, 107)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.80, 'y': 0.0, 'zone': 'A'}
    calls = []; spy(c, calls)
    ok = c.go_to_next_tag(107, known_start_id=106)
    check('B1 forward hop from a forward arrival: no reseat, pp(107) -> align(107)',
          ok and [x[:2] for x in calls] == [('pp', 107), ('align', 107)], str(calls))

    # ---- C. no re-seat before a REVERSE hop even with the tag ahead of the lens
    c, p = make(0.40 - 0.16 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 105: (0.0, 0.0)})
    c.map_mgr.edges[(106, 105)] = {'type': 'move', 'direction': 'backward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.0, 'y': 0.0, 'zone': 'A'}
    calls = []; spy(c, calls)
    ok = c.go_to_next_tag(105, known_start_id=106)
    check('C1 reverse hop: no reseat', ok and [x[:2] for x in calls] == [('pp', 105), ('align', 105)], str(calls))

    # ---- D. config off -> no reseat
    cfg = copy.deepcopy(CFG0); cfg['robot']['reseat_forward_from_reverse_column'] = False
    c, p = make(0.40 - 0.16 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 107: (0.80, 0.0)}, cfg)
    c.map_mgr.edges[(106, 107)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.80, 'y': 0.0, 'zone': 'A'}
    calls = []; spy(c, calls)
    ok = c.go_to_next_tag(107, known_start_id=106)
    check('D1 reseat disabled: forward hop plans 0.40 + 0.16 as before', ok and calls[0][:2] == ('pp', 107) and abs(calls[0][3] - 0.56) < 0.02, str(calls))

    # ---- E. first-hop align rules still hold (with a reseat in the middle)
    c, p = make(0.40 - 0.16 - CAM, 0.0, math.radians(1.2), {106: (0.40, 0.0), 107: (0.80, 0.0)})
    c.map_mgr.edges[(106, 107)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.80, 'y': 0.0, 'zone': 'A'}
    calls = []; spy(c, calls)
    ok = c.go_to_next_tag(107, known_start_id=106, first_hop=True)
    check('E1 first hop: align(106) -> reseat -> align(106) -> pp(107) -> align(107)',
          ok and [x[:2] for x in calls] == [('align', 106), ('pp', 106), ('align', 106), ('pp', 107), ('align', 107)], str(calls))

    # ---- G. a command ENDING on a reverse arrival re-seats before returning (arm work at the FWD column)
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 105: (0.0, 0.0)})   # lens over 106, reverse to 105
    c.map_mgr.edges[(106, 105)] = {'type': 'move', 'direction': 'backward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.0, 'y': 0.0, 'zone': 'A'}
    c.map_mgr.path = [106, 105]; c.last_known_tag = 106
    calls = []; spy(c, calls)
    ok = c.move_to_tag(105)
    for _ in range(20): Rate(20).sleep()
    fore_after = c.detected_tags[105]['x'] if 105 in c.detected_tags else float('nan')
    check('G1 reverse-ending command: align(106) -> pp(105 bwd) -> align -> reseat pp(105 fwd ~0.16) -> align, returns True',
          ok and [x[:3] for x in calls] == [('align', 106), ('pp', 105, 'backward'), ('align', 105), ('pp', 105, 'forward'), ('align', 105)]
          and abs(calls[3][3] - 0.16) < 0.02, str(calls))
    check('G2 ... and the tag rests on the FWD column (crosshair) within 5 mm, lens over the tag',
          abs(fore_after) < 0.005 and abs(p.lens()[0] - 0.0) < 0.005, f'fore {fore_after:+.4f} lens x {p.lens()[0]:+.4f}')
    calls.clear()
    c.map_mgr.edges[(105, 106)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.path = [105, 106]
    ok = c.move_to_tag(106)
    check('G3 the following forward command needs no launch re-seat', ok and [x[:2] for x in calls] == [('align', 105), ('pp', 106), ('align', 106)], str(calls))
    # config off: the old behaviour (tag left on the REV column)
    cfg = copy.deepcopy(CFG0); cfg['robot']['reseat_at_command_end'] = False
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 105: (0.0, 0.0)}, cfg)
    c.map_mgr.edges[(106, 105)] = {'type': 'move', 'direction': 'backward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.0, 'y': 0.0, 'zone': 'A'}
    c.map_mgr.path = [106, 105]; c.last_known_tag = 106
    calls = []; spy(c, calls)
    ok = c.move_to_tag(105)
    check('G4 reseat_at_command_end off: the command ends on the REV column', ok and [x[:2] for x in calls] == [('align', 106), ('pp', 105), ('align', 105)]
          and abs(c.detected_tags[105]['x'] - 0.16) < 0.02, str(calls))

    # ---- F. pivot sequencing (from the lost t_pivot suite)
    c, p = make(0.0, 0.0, 0.0, {501: (CAM, 0.0)})
    p.tags[505] = (0.0, CAM, math.pi / 2)   # exit tag under the lens after a +90 turn, laid along the perpendicular lane
    c.map_mgr.edges[(501, 505)] = {'type': 'pivot', 'direction': 'ccw'}
    calls = []; spy(c, calls)
    ok = c.go_to_next_tag(505, known_start_id=501, first_hop=True)
    for _ in range(20): Rate(20).sleep()
    check('F1 first-hop pivot: align(501) -> pivot(ccw, 505) -> align(505), ends within 0.2 deg of the exit tag',
          ok and [x[:2] for x in calls] == [('align', 501), ('pivot', 'ccw'), ('align', 505)] and abs(math.degrees(p.psi) - 90.0) <= 0.2,
          f'{calls} psi {math.degrees(p.psi):+.2f}')

    n = sum(1 for x in checks if not x)
    print(f'\n{len(checks) - n}/{len(checks)} checks passed')
    return 1 if n else 0


if __name__ == '__main__':
    sys.exit(main())
