#!/usr/bin/env python3
"""Offline checks of the live /robot_pose stream (2026-09-28, user:
"실시간으로 로봇 현재 위치 계속 보내고 싶어").

    python3 src/apriltag_nav/tools/check_robot_pose_live.py

Reuses check_nav_sequencing's harness (stubbed rospy with a simulated
clock, the real MobileController + robot.yaml, the 2-D plant with a
simulated front_cam whose lane is the world x axis = zone A). Checks that
the live pose equals calculate_robot_pose while a map tag is in view, is
carried forward on odom (translation AND heading, in the world frame)
once the tag is gone, agrees with the plant to millimetres on a real
driven hop, is never published before an anchor exists or from a
non-map tag, that the arrival message stays flag True while the stream
is flag False, and that arm_controller's pose_cb keeps only flag True.
Exit status 1 on failure.
"""
import importlib.util
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'check_nav_sequencing', os.path.join(HERE, 'check_nav_sequencing.py'))
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)          # installs the rospy / msg stubs

CLK, Rate, LOG, check, make, CAM = H.CLK, H.Rate, H.LOG, H.check, H.make, H.CAM


def capture(c):
    """Record every message pose_pub sends: list of (flag, id, x, y, theta)."""
    out = []
    c.pose_pub = types.SimpleNamespace(
        publish=lambda m: out.append((bool(m.flag), int(m.id), float(m.x), float(m.y), float(m.theta))))
    return out


def world_of(msg_x, msg_y):
    """Manipulator-frame message -> world (x, y) of the base centre."""
    return -msg_y, -msg_x


def zone_a_map(tags):
    return lambda t: ({'x': tags[t][0], 'y': tags[t][1], 'zone': 'A'} if t in tags else None)


def main():
    tags = {106: (0.40, 0.0), 107: (0.80, 0.0)}

    # ---- A. tag in view: live pose == calculate_robot_pose, flag False, anchored
    c, p = make(0.40 - CAM, 0.0, 0.0, tags)          # lens over 106, base 0.55 behind
    c.map_mgr.get_tag_info = zone_a_map(tags)
    c.publish_robot_pose = H.MobileController.publish_robot_pose.__get__(c)   # the real one (make() stubs it)
    out = capture(c)
    check('A1 config read (enabled, 10 Hz, odom on, 1.0 s)',
          c.live_pose_enabled and c.live_pose_rate_hz == 10.0 and c.live_pose_odom_extrapolate
          and c.live_pose_odom_max_age_s == 1.0)
    src = c.publish_live_robot_pose()
    ref = c.calculate_robot_pose(106)
    check('A2 published from the tag', src == 'tag' and len(out) == 1 and out[0][0] is False and out[0][1] == 106, str(out))
    check('A3 equals calculate_robot_pose(106) exactly',
          abs(out[0][2] - ref[0]) < 1e-12 and abs(out[0][3] - ref[1]) < 1e-12 and abs(out[0][4] - ref[2]) < 1e-12)
    wx, wy = world_of(out[0][2], out[0][3])
    check('A4 ... and that is the plant: base at world (-0.15, 0), heading 0',
          abs(wx - p.x) < 1e-6 and abs(wy - p.y) < 1e-6 and abs(out[0][4]) < 1e-6, f'{wx:.4f} {wy:.4f} {out[0][4]:.3f}')
    check('A5 anchor set on tag 106 at this odom', c._live_anchor and c._live_anchor['tag'] == 106
          and c._live_anchor['odom_x'] == p.odom_x)

    # ---- B. tag gone: odom carries the pose (translation rotated into the world, heading follows)
    p.tags = {}                                       # camera sees nothing from now on
    p.push_cam()
    check('B1 no map tag in view', c._live_map_tag_id() is None)
    # the plant moves 0.20 m along the lane and yaws +10 deg; odom frame is rotated 30 deg vs the world
    p.odom_psi = math.radians(30.0); p.push_odom()
    c._set_live_anchor(106, ref)                      # re-anchor with the rotated odom frame
    d = 0.20; a0 = math.radians(30.0)
    p.odom_x += d * math.cos(a0); p.odom_y += d * math.sin(a0); p.odom_psi = a0 + math.radians(10.0)
    p.x += d; p.psi = math.radians(10.0); p.push_odom()
    out.clear(); src = c.publish_live_robot_pose()
    wx, wy = world_of(out[0][2], out[0][3])
    check('B2 published from odom, same anchoring tag', src == 'odom' and out[0][0] is False and out[0][1] == 106, str(out))
    check('B3 translation rotated from the odom frame into the world: +0.20 m along x, 0 in y',
          abs(wx - p.x) < 1e-9 and abs(wy - p.y) < 1e-9, f'{wx:.4f} {wy:.4f} vs plant {p.x:.4f} {p.y:.4f}')
    check('B4 heading follows the odom yaw delta (+10 deg)', abs(out[0][4] - 10.0) < 1e-9, f'{out[0][4]:.3f}')
    # stale odom -> nothing
    CLK.t += 1.5
    out.clear(); src = c.publish_live_robot_pose()
    check('B5 /odom older than odom_max_age_s: nothing published', src is None and out == [])
    p.push_odom()
    check('B6 fresh odom again: published', c.publish_live_robot_pose() == 'odom')

    # ---- C. a tag comes back: it re-anchors and the odom drift is discarded
    p.tags = {107: (0.80, 0.0)}
    p.x = 0.80 - CAM + 0.02; p.psi = 0.0   # lens 2 cm past tag 107 (3 cm is the plant's bumper limit)
    p.hist.clear(); p.push_cam()    # drop the 0.1 s image-latency history so the frame is of THIS pose
    out.clear(); src = c.publish_live_robot_pose()
    wx, wy = world_of(out[0][2], out[0][3])
    check('C1 tag 107 in view: tag-based again, id 107', src == 'tag' and out[0][1] == 107, str(out))
    check('C2 ... base at world x 0.27 within 1 mm (lens 2 cm past the tag)', abs(wx - p.x) < 1e-3, f'{wx:.4f}')
    check('C3 anchor moved to 107', c._live_anchor['tag'] == 107)

    # ---- D. a visible tag the map does not know produces NO pose, and no anchor before any tag
    c, p = make(0.40 - CAM, 0.0, 0.0, {15: (0.40, 0.0)})
    c.map_mgr.get_tag_info = lambda t: None
    out = capture(c)
    check('D1 non-map tag in view: nothing published, no anchor',
          c.publish_live_robot_pose() is None and out == [] and c._live_anchor is None)
    p.tags = {}; p.push_cam(); p.x += 0.1; p.push_odom()
    check('D2 no anchor yet, no tag: nothing published (no blind guess)', c.publish_live_robot_pose() is None)

    # ---- E. arrival publish (flag True) re-anchors the stream; the two kinds are distinguishable
    c, p = make(0.40 - CAM, 0.0, 0.0, tags)
    c.map_mgr.get_tag_info = zone_a_map(tags)
    c.publish_robot_pose = H.MobileController.publish_robot_pose.__get__(c)
    out = capture(c)
    c.publish_robot_pose(106)
    check('E1 arrival message flag True, id 106', out and out[-1][0] is True and out[-1][1] == 106, str(out))
    check('E2 arrival set the live anchor', c._live_anchor and c._live_anchor['tag'] == 106)
    c.publish_live_robot_pose()
    check('E3 live message flag False with the same x, y, theta at rest',
          out[-1][0] is False and all(abs(out[-1][i] - out[-2][i]) < 1e-12 for i in (2, 3, 4)))

    # ---- F. disabled: the stream publishes nothing, the arrival message is unchanged
    cfg = H.copy.deepcopy(H.CFG0); cfg['robot']['robot_pose_live'] = {'enabled': False}
    c, p = make(0.40 - CAM, 0.0, 0.0, tags, cfg=cfg)
    c.map_mgr.get_tag_info = zone_a_map(tags)
    c.publish_robot_pose = H.MobileController.publish_robot_pose.__get__(c)
    out = capture(c)
    check('F1 disabled: publish_live_robot_pose is a no-op', c.publish_live_robot_pose() is None and out == [])
    c.publish_robot_pose(106)
    check('F2 disabled: the arrival message still goes out flag True', len(out) == 1 and out[0][0] is True)

    # ---- G. a real driven hop: the stream on every tick tracks the plant to millimetres
    c, p = make(0.40 - CAM, 0.0, 0.0, tags)
    c.map_mgr.get_tag_info = zone_a_map(tags)
    c.map_mgr.edges[(106, 107)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.path = [106, 107]
    out = capture(c)
    errs, sources = [], []
    real_step = p.step

    def step(dt):
        real_step(dt)
        n = len(out)
        src = c.publish_live_robot_pose()
        if src is not None:
            sources.append(src)
            wx, wy = world_of(out[-1][2], out[-1][3])
            errs.append(math.hypot(wx - p.x, wy - p.y))
    H.STEP_HOOK[0] = step
    ok = c.move_to_tag(107)
    check('G1 hop 106 -> 107 arrived', ok)
    check('G2 stream published on every tick of the drive', len(sources) > 100, str(len(sources)))
    check('G3 both sources used (tag while in view, odom in the blind part)',
          'tag' in sources and 'odom' in sources, str(sorted(set(sources))))
    # While moving the tag-based pose lags by the 0.1 s image latency x speed
    # (5 mm at 0.05 m/s) — the camera's, not the stream's; at rest it is exact.
    check('G4 live position within 8 mm of the plant on every tick (image latency x speed)',
          max(errs) < 0.008, f'max {max(errs)*1000:.1f} mm')
    check('G5 ... and within 1 mm at rest after the arrival', errs[-1] < 0.001, f'{errs[-1]*1000:.2f} mm')
    check('G6 all stream messages flag False', all(f is False for f, *_ in out))

    # ---- H. arm_controller.pose_cb keeps the arrival pose only
    sys.path.insert(0, os.path.join(HERE, '..', 'src'))
    for n in ('Float32', 'String', 'Bool'):
        setattr(sys.modules['std_msgs.msg'], n, type(n, (H._Msg,), {}))
    ac_path = os.path.join(HERE, '..', 'src', 'apriltag_nav', 'arm_controller.py')
    src_txt = open(ac_path, encoding='utf-8').read()
    i = src_txt.index('    def pose_cb(self, msg):'); j = src_txt.index('\n    def ', i + 10)
    ns = {}
    exec('class AC:\n' + src_txt[i:j], ns)
    ac = ns['AC'](); ac.current_pose_msg = None
    live = types.SimpleNamespace(flag=False, id=106, x=1.0, y=2.0, theta=3.0)
    arrival = types.SimpleNamespace(flag=True, id=106, x=0.1, y=0.2, theta=0.3)
    ac.pose_cb(live)
    check('H1 pose_cb ignores a live (flag False) message', ac.current_pose_msg is None)
    ac.pose_cb(arrival)
    check('H2 pose_cb keeps an arrival (flag True) message', ac.current_pose_msg is arrival)
    ac.pose_cb(live)
    check('H3 a later live message does not replace the arrival pose', ac.current_pose_msg is arrival)

    n_ok = sum(H.checks); n = len(H.checks)
    print(f'\n{n_ok}/{n} checks passed')
    sys.exit(0 if n_ok == n else 1)


if __name__ == '__main__':
    main()
