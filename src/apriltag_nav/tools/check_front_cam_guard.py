#!/usr/bin/env python3
"""Offline checks of the front_cam liveness gate (2026-09-22, user rule:
if the front camera is not working, refuse every command that drives by
tag).

    python3 src/apriltag_nav/tools/check_front_cam_guard.py

Reuses check_nav_sequencing's harness (stubbed rospy with a simulated
clock, the real MobileController with the real robot.yaml, the 2-D plant
with a simulated front_cam). Exit status 1 on failure.
"""
import importlib.util
import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'check_nav_sequencing', os.path.join(HERE, 'check_nav_sequencing.py'))
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)          # installs the rospy / msg stubs

CLK, Rate, LOG, check, make = H.CLK, H.Rate, H.LOG, H.check, H.make
CAM = H.CAM


def freeze_camera(p):
    """The plant stops feeding detections (camera dead) but keeps stepping."""
    p.push_cam = lambda: None


def main():
    # ---- A. status / property on a live camera
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 107: (0.80, 0.0)})
    ok, reason, age = c.front_cam_status()
    check('A1 live camera: status ok, age 0', ok and age == 0.0, f'{ok} {reason} {age}')
    check('A2 live camera: tag 106 visible through detected_tags', 106 in c.detected_tags)
    check('A3 timeout read from robot.yaml (1.0 s)', abs(c.front_cam_alive_timeout_s - 1.0) < 1e-9,
          str(c.front_cam_alive_timeout_s))

    # ---- B. camera dies: property goes empty after the timeout, not before
    freeze_camera(p)
    CLK.t += 0.9
    check('B1 0.9 s without a frame: still inside the timeout, tag still visible',
          c.front_cam_status()[0] and 106 in c.detected_tags)
    CLK.t += 0.2
    ok, reason, age = c.front_cam_status()
    check('B2 1.1 s without a frame: status false with the age in the reason',
          not ok and 'front_cam not working' in reason and '1.1 s ago' in reason, reason)
    check('B3 detected_tags reads {} (the stale frame is not served)', c.detected_tags == {})
    check('B4 get_current_tag_id() sees no tag', c.get_current_tag_id() is None)
    dead = [l for l in LOG if 'front_cam detections stopped' in l]
    check('B5 logged once', len(dead) == 1, str(len(dead)))
    _ = c.detected_tags
    check('B6 ... and only once on repeated reads', len([l for l in LOG if 'detections stopped' in l]) == 1)

    # ---- C. camera resumes: tags come back, resume logged
    p.push_cam = type(p).push_cam.__get__(p)
    p.push_cam()
    check('C1 a new frame restores visibility', 106 in c.detected_tags and c.front_cam_status()[0])
    check('C2 resume logged', any('detections resumed' in l for l in LOG))

    # ---- D. move_to_tag refused at the command start, nothing moves
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 107: (0.80, 0.0)})
    c.map_mgr.edges[(106, 107)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.path = [106, 107]
    calls = []; H.spy(c, calls)
    freeze_camera(p); CLK.t += 1.5
    x0 = p.x
    ok = c.move_to_tag(107)
    check('D1 refused (False) with the reason kept', not ok and c.last_refusal_reason
          and 'front_cam not working' in c.last_refusal_reason, str(c.last_refusal_reason))
    check('D2 no hop attempted, base did not move', calls == [] and p.x == x0, str(calls))
    check('D3 REFUSED line in the log', any('REFUSED' in l for l in LOG))
    check('D4 last_known_tag fallback NOT used (refusal precedes it)',
          not any('fallback to last_known_tag' in l for l in LOG))

    # ---- E. before the first frame ever (node just started): refused with the "no detections yet" reason
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0)})
    c._front_cam_last_msg_s = None; c._detected_tags_raw = {}
    ok, reason, age = c.front_cam_status()
    check('E1 no message yet: refused, age None', not ok and age is None and 'no detections received yet' in reason, reason)

    # ---- F. camera dies mid-route: the next hop is not started, base stopped
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 107: (0.80, 0.0), 108: (1.20, 0.0)})
    for a, b in ((106, 107), (107, 108)):
        c.map_mgr.edges[(a, b)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': {106: 0.40, 107: 0.80, 108: 1.20}[t], 'y': 0.0, 'zone': 'A'}
    c.map_mgr.path = [106, 107, 108]
    calls = []; H.spy(c, calls)
    real_hop = c.go_to_next_tag

    def hop(**kw):
        ok = real_hop(**kw)
        if kw['target_id'] == 107:
            freeze_camera(p); CLK.t += 1.5      # camera dies right after hop 1
        return ok
    c.go_to_next_tag = hop
    stops = []
    real_stop = c.stop
    c.stop = lambda: (stops.append(CLK.t) or real_stop())
    ok = c.move_to_tag(108)
    hops = [x for x in calls if x[0] == 'pp']
    check('F1 first hop ran, route failed before the second', not ok and [h[1] for h in hops] == [107], str(hops))
    check('F2 aborted-before-hop reason kept and logged',
          c.last_refusal_reason and 'front_cam not working' in c.last_refusal_reason
          and any('aborted before hop 107' in l for l in LOG), str(c.last_refusal_reason))
    check('F3 stop() called on the abort', len(stops) >= 1)

    # ---- G. a live camera still drives the route (regression of the gate itself)
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0), 107: (0.80, 0.0)})
    c.map_mgr.edges[(106, 107)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40 if t == 106 else 0.80, 'y': 0.0, 'zone': 'A'}
    c.map_mgr.path = [106, 107]
    ok = c.move_to_tag(107)
    for _ in range(20): Rate(20).sleep()
    check('G1 live camera: move_to_tag(107) arrives within 5 mm', ok and abs(p.lens()[0] - 0.80) < 0.005,
          f'{ok} lens {p.lens()[0]:.4f}')
    check('G2 no refusal reason', c.last_refusal_reason is None)

    # ---- H. gate disabled (<= 0): a dead camera is not refused (offline plants)
    cfg = H.copy.deepcopy(H.CFG0); cfg['robot']['front_cam_alive_timeout_s'] = 0
    c, p = make(0.40 - CAM, 0.0, 0.0, {106: (0.40, 0.0)}, cfg=cfg)
    freeze_camera(p); CLK.t += 5.0
    check('H1 timeout 0 disables: status ok, stale tag still served',
          c.front_cam_status()[0] and 106 in c.detected_tags)

    # ---- I. mobile_node: refusal at the command boundary + state fields
    rospy = sys.modules['rospy']
    subs = {}
    rospy.init_node = lambda *a, **k: None
    rospy.Subscriber = lambda topic, typ, cb, **k: subs.__setitem__(topic, cb)
    rospy.Service = lambda *a, **k: None
    rospy.Timer = lambda *a, **k: None
    rospy.on_shutdown = lambda f: None
    published = []
    rospy.Publisher = lambda topic, *a, **k: types.SimpleNamespace(
        publish=lambda m: published.append((topic, getattr(m, 'data', m))))
    std = sys.modules['std_msgs.msg']
    for n in ('String', 'Int32'):
        setattr(std, n, type(n, (), {'__init__': lambda self, data=None: setattr(self, 'data', data)}))
    srv = types.ModuleType('std_srvs.srv')
    srv.Trigger = object; srv.TriggerResponse = lambda **k: types.SimpleNamespace(**k)
    sys.modules['std_srvs'] = types.ModuleType('std_srvs'); sys.modules['std_srvs.srv'] = srv
    an = types.ModuleType('apriltag_nav.utils'); an.load_config = lambda p: H.copy.deepcopy(H.CFG0)
    sys.modules['apriltag_nav.utils'] = an
    mm = types.ModuleType('apriltag_nav.map_manager'); mm.MapManager = lambda p: H.FakeMap()
    sys.modules['apriltag_nav.map_manager'] = mm
    nd = types.ModuleType('apriltag_nav.navifra_devices')
    nd.NavifraDevices = lambda cfg, on_estop=None: types.SimpleNamespace(estop_active=False)
    sys.modules['apriltag_nav.navifra_devices'] = nd
    pa = types.ModuleType('apriltag_nav.paths'); pa.CONFIG_PATH = pa.MAP_PATH = ''
    sys.modules['apriltag_nav.paths'] = pa
    import threading
    real_thread = threading.Thread
    threading.Thread = lambda target=None, args=(), daemon=None: types.SimpleNamespace(start=lambda: target(*args))
    try:
        nspec = importlib.util.spec_from_file_location(
            'mobile_node', os.path.join(HERE, '..', 'scripts', 'mobile_node.py'))
        MN = importlib.util.module_from_spec(nspec); nspec.loader.exec_module(MN)
        node = MN.MobileNode()
    finally:
        threading.Thread = real_thread
    c = node.mobile
    c.stop_requested = False
    c.publish_robot_pose = lambda tid: None
    c.map_mgr.path = [106, 107]

    def last_state():
        return json.loads([m for t, m in published if t == '/mobile/state'][-1])

    st = last_state()
    check('I1 state before any frame: front_cam_ok false, age null, reason set',
          st['front_cam_ok'] is False and st['front_cam_age_s'] is None
          and 'no detections received yet' in st['front_cam_reason'], str(st))
    moved = []
    c.move_to_tag = lambda tid: (moved.append(tid) or True)
    subs['/mobile/goto_tag'](std.Int32(107))
    st = last_state()
    check('I2 goto refused at the node: seq advanced, ok false, reason in the message, move_to_tag not called',
          st['seq'] == 1 and st['result']['ok'] is False and st['result']['tag'] == 107
          and 'front_cam not working' in st['result']['message'] and moved == [], str(st['result']))
    check('I3 not busy after the refusal', st['busy'] is None)
    # a frame arrives -> goto goes through
    c._store_detections({106: {'x': 0.0, 'y': 0.0, 'z': 0.302, 'corners': H.np.zeros((4, 2)),
                               'center_x': 638.2, 'center_y': 353.0}}, CLK.t)
    node._publish_state()
    st = last_state()
    check('I4 state with a fresh frame: front_cam_ok true, age 0.0, no reason',
          st['front_cam_ok'] is True and st['front_cam_age_s'] == 0.0 and st['front_cam_reason'] is None, str(st))
    subs['/mobile/goto_tag'](std.Int32(107))
    st = last_state()
    check('I5 goto accepted with a live camera', moved == [107] and st['result']['ok'] is True, str(st['result']))
    # the controller's own refusal reason is surfaced in the result message
    c.move_to_tag = lambda tid: (setattr(c, 'last_refusal_reason', 'front_cam not working: TEST') or False)
    subs['/mobile/goto_tag'](std.Int32(107))
    st = last_state()
    check('I6 controller-level refusal surfaces in result.message',
          'refused: front_cam not working: TEST' in st['result']['message'], st['result']['message'])
    # manual (odom-only) move is NOT gated
    CLK.t += 5.0
    check('I7 camera stale again', node.mobile.front_cam_status()[0] is False)
    drove = []
    c.drive_distance = lambda d, s: (drove.append(d) or (True, 'ok'))
    subs['/mobile/move_cmd'](std.String(json.dumps({'type': 'move', 'distance_m': 0.1})))
    st = last_state()
    check('I8 manual move_cmd still runs with the camera dead', drove == [0.1] and st['result']['ok'] is True,
          str(st['result']))
    # a goto that raises stops the base
    stops = []
    c.stop = lambda: stops.append(1)
    c.move_to_tag = lambda tid: (_ for _ in ()).throw(RuntimeError('boom'))
    c._store_detections({}, CLK.t)
    subs['/mobile/goto_tag'](std.Int32(107))
    st = last_state()
    check('I9 a goto that raises calls stop() and reports the exception',
          stops == [1] and 'exception: boom' in st['result']['message'], str(st['result']))

    print(f"\n{sum(H.checks)}/{len(H.checks)} checks passed")
    sys.exit(0 if all(H.checks) else 1)


if __name__ == '__main__':
    main()
