#!/usr/bin/env python3
"""
check_robot_pose_heading.py — offline check of two 2026-09-28 (night)
changes in MobileController, found on the 22-stop tip tour:

  1. calculate_robot_pose projects the camera -> base-centre lever
     (camera_offset) and the tag's fore / lateral offsets along the BODY
     HEADING (zone + align residual + laying angle), not the bare zone
     axis (robot.robot_pose_offset_along_heading). A body yawed delta off
     the lane put the base centre 0.55 sin(delta) beside the old estimate
     (9.6 mm/deg): tag 116 at delta -0.77 deg landed the tip +9.7 mm.
  2. The align's settled measurement is the MEDIAN of align_record_frames
     frames at rest; an out-of-passes residual is re-settled and re-measured
     before it is accepted; publish_robot_pose publishes THAT view (fresh,
     same tag) instead of the newest single frame (tag 104: the two
     disagreed by 1.44 deg = 27 mm of arm target).

    python3 src/apriltag_nav/tools/check_robot_pose_heading.py
"""
import copy
import importlib.util
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location(
    'check_nav_sequencing', os.path.join(HERE, 'check_nav_sequencing.py'))
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)          # installs the rospy / msg stubs

CLK, LOG, check, CAM = H.CLK, H.LOG, H.check, H.CAM
MC = H.MobileController


def ctrl(x, y, psi, tags, delay=0.55):
    CLK.t = 1000.0; LOG.clear()
    c = MC(copy.deepcopy(H.CFG0), H.FakeMap())
    c.stop_requested = False
    p = H.Plant(c, x, y, psi, tags, delay=delay)
    H.STEP_HOOK[0] = p.step
    c._last_tag_depth_m = H.Z
    records = []
    c._record_tag_offset = lambda tid, tag, stage, yaw_error_deg=None, extra=None: \
        records.append(dict(tag=tid, stage=stage, yaw=yaw_error_deg, view=dict(tag), extra=dict(extra or {})))
    c.records = records
    out = []
    c.pose_pub = types.SimpleNamespace(
        publish=lambda m: out.append((bool(m.flag), int(m.id), float(m.x), float(m.y), float(m.theta))))
    c.published = out
    return c, p


def world_of(msg):
    return -msg[3], -msg[2]           # manipulator (x, y) -> world (x, y)


def tag_dict(fore, lateral, edge_deg):
    return {'x': fore, 'y': lateral, 'z': 0.302, 'edge_deg': edge_deg,
            'center_x': 640.0, 'center_y': 360.0, 'corners': np.zeros((4, 2))}


def main():
    print('== 1. heading projection vs the zone-axis projection ==')
    c, _ = ctrl(0.0, 0.0, 0.0, {})
    check('1a config: robot_pose_offset_along_heading on, 3 record frames, 10 s view age',
          c.robot_pose_offset_along_heading and c.align_record_frames == 3 and c.align_view_max_age_s == 10.0)
    zones = {'A': ({'x': 0.5, 'y': 3.02, 'zone': 'A'}, 0.0),
             'B': ({'x': -1.71, 'y': 0.55, 'zone': 'B'}, 90.0),
             'C': ({'x': 1.71, 'y': 0.55, 'zone': 'C'}, -90.0),
             'DOCK': ({'x': -1.9123, 'y': 3.02, 'zone': 'DOCK'}, 0.0)}
    worst = 0.0
    for z, (info, hd) in zones.items():
        c.map_mgr.get_tag_info = lambda t, info=info: info
        for fore, lat in ((0.0, 0.0), (0.16, 0.004), (-0.01, -0.02)):
            t = tag_dict(fore, lat, 0.0)
            c.robot_pose_offset_along_heading = True; new = c.calculate_robot_pose(5, tag=t)
            c.robot_pose_offset_along_heading = False; old = c.calculate_robot_pose(5, tag=t)
            worst = max(worst, max(abs(a - b) for a, b in zip(new, old)))
    c.robot_pose_offset_along_heading = True
    check('1b body exactly on the zone axis: new == old in zones A/B/C/DOCK, fore/lateral included',
          worst < 1e-12, 'worst %.1e' % worst)

    # tag 116 (zone C, calibrated yaw 179.23 -> delta -0.77) with the run-2 align residual +0.16
    info = {'x': 1.6866, 'y': 0.6458, 'zone': 'C', 'yaw': 179.23}
    c.map_mgr.get_tag_info = lambda t: info
    t = tag_dict(0.0, -0.0034, 0.161)
    c.robot_pose_offset_along_heading = True; new = c.calculate_robot_pose(116, tag=t)
    c.robot_pose_offset_along_heading = False; old = c.calculate_robot_pose(116, tag=t)
    c.robot_pose_offset_along_heading = True
    dwx, dwy = (-new[1]) - (-old[1]), (-new[0]) - (-old[0])
    exp = -CAM * (math.cos(math.radians(new[2])) - math.cos(math.radians(-90.0))) * 1000
    check('1c tag 116: heading -90.61 -> the base moves +%.1f mm in world x vs the zone axis (expected %.1f), y ~0' % (dwx * 1000, exp),
          abs(dwx * 1000 - exp) < 0.3 and abs(dwy) < 0.0003, 'theta %.3f' % new[2])
    check('1d … and the sign says the tip used to land +x of the target in zone C (the measured +9.7 mm)', dwx > 0)
    # tag 104 (zone B, delta +0.43): base = cam - 0.55 u(90.43 deg) -> +0.55 sin(0.43 deg) in
    # world x. Same sign as 116 because delta has the opposite sign in the opposite zone;
    # zone B's usual delta -0.3 gives -2.9 mm, the -3..-4 mm measured at 101-103.
    info = {'x': -1.7066, 'y': -0.2531, 'zone': 'B', 'yaw': 0.43}
    t = tag_dict(0.0, 0.0, 0.0)
    c.robot_pose_offset_along_heading = True; new = c.calculate_robot_pose(104, tag=t)
    c.robot_pose_offset_along_heading = False; old = c.calculate_robot_pose(104, tag=t)
    dwx = (-new[1]) - (-old[1])
    exp = CAM * math.sin(math.radians(0.43)) * 1000
    check('1e tag 104: zone B, delta +0.43 -> base moves %+.1f mm in world x (expected %+.1f; run-1 residual there was +7.2)' % (dwx * 1000, exp),
          abs(dwx * 1000 - exp) < 0.05)
    new104 = new
    info = {'x': -1.7033, 'y': -1.4501, 'zone': 'B', 'yaw': -0.35}
    c.robot_pose_offset_along_heading = True; new = c.calculate_robot_pose(101, tag=t)
    c.robot_pose_offset_along_heading = False; old = c.calculate_robot_pose(101, tag=t)
    c.robot_pose_offset_along_heading = True
    dwx = (-new[1]) - (-old[1])
    check('1e2 tag 101: zone B, delta -0.35 -> %+.1f mm (measured tip error there -3.7)' % (dwx * 1000),
          abs(dwx * 1000 + CAM * math.sin(math.radians(0.35)) * 1000) < 0.05)
    check('1f edge_deg in the dict is preferred over the corners', abs(new104[2] - (90.0 + 0.43)) < 1e-9)

    print('\n== 2. closed loop: tag laid 0.8 deg off the lane, align, publish -> the base centre is right ==')
    hdg = math.radians(0.8)
    tags = {106: (0.40, 0.0, hdg)}
    c, p = ctrl(0.40 - CAM, 0.0, 0.0, tags)               # lens over the tag, body along the lane (0.8 deg off the tag)
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40, 'y': 0.0, 'zone': 'A', 'yaw': 0.8} if t == 106 else None
    for _ in range(3):
        p.step(0.1)
    ok = c.align_to_tag(106)
    rec = [r for r in c.records if r['stage'] == 'aligned']
    check('2a align converges, body now along the TAG (psi %.2f deg)' % math.degrees(p.psi),
          ok and abs(math.degrees(p.psi) - 0.8) < 0.2, 'yaw error %.3f' % rec[-1]['yaw'])
    check('2b the settled measurement is a median of 3 frames and was remembered',
          rec[-1]['extra'].get('align_median_frames') == 3 and c._aligned_view is not None
          and c._aligned_view['tag'] == 106, str(rec[-1]['extra']))
    c.publish_robot_pose(106)
    msg = c.published[-1]; wx, wy = world_of(msg)
    err = math.hypot(wx - p.x, wy - p.y) * 1000
    check('2c published base centre within 1 mm of the plant (%.2f mm), theta %.2f ~ 0.8' % (err, msg[4]),
          err < 1.0 and abs(msg[4] - 0.8) < 0.15)
    check('2d … from the align\'s at-rest median (log says so)',
          any("from the align's at-rest median (3 frames)" in m for m in LOG))
    view = c._aligned_view['view']
    c.robot_pose_offset_along_heading = False
    old = c.calculate_robot_pose(106, tag=view)
    c.robot_pose_offset_along_heading = True
    err_old = math.hypot(-old[1] - p.x, -old[0] - p.y) * 1000
    check('2e the zone-axis projection would have been %.1f mm off (0.55 sin 0.8 deg = %.1f)' % (err_old, CAM * math.sin(hdg) * 1000),
          abs(err_old - CAM * math.sin(hdg) * 1000) < 1.0)
    # stale view -> newest frame
    c._aligned_view['t'] -= 60.0
    LOG.clear(); c.publish_robot_pose(106)
    check('2f a stale aligned view (60 s) is not used: newest frame', any('from the newest frame' in m for m in LOG))
    c._aligned_view = None
    LOG.clear(); c.publish_robot_pose(106)
    check('2g no aligned view: newest frame, pose still published', any('from the newest frame' in m for m in LOG) and len(c.published) == 3)

    print('\n== 3. out of passes: re-settle, re-measure, then accept (the tag-104 case) ==')
    c, p = ctrl(0.40 - CAM, 0.0, math.radians(3.0), {106: (0.40, 0.0)}, delay=2.0)   # base executes 2.0 s late vs the 0.55 s model
    c.map_mgr.get_tag_info = lambda t: {'x': 0.40, 'y': 0.0, 'zone': 'A'} if t == 106 else None
    for _ in range(3):
        p.step(0.1)
    t0 = CLK.t
    ok = c.align_to_tag(106)
    rec = [r for r in c.records if r['stage'] == 'aligned']
    ex = rec[-1]['extra'] if rec else {}
    check('3a the align gives up after %d passes and ACCEPTS a residual' % ex.get('align_passes', 0),
          ok and ex.get('align_residual_accepted') is True and ex.get('align_passes') == 3, str(ex))
    check('3b … after a re-settle and a fresh median measurement (align_resettled, 3 frames)',
          ex.get('align_resettled') is True and ex.get('align_median_frames') == 3)
    check('3c the log names the re-settle before accepting',
          any('re-settling' in m for m in LOG) and any('at rest after the re-settle' in m for m in LOG))
    # the recorded angle is the base's TRUE yaw at rest (plant), i.e. measured after the motion stopped
    check('3d the recorded residual equals the plant\'s yaw at rest (%.2f vs %.2f deg)' % (rec[-1]['yaw'], math.degrees(p.psi)),
          abs(rec[-1]['yaw'] - math.degrees(p.psi)) < 0.05)
    c.publish_robot_pose(106)
    msg = c.published[-1]
    check('3e the published theta is that same at-rest angle, from the aligned view',
          abs(msg[4] - rec[-1]['yaw']) < 1e-6 and any("from the align's at-rest median" in m for m in LOG))
    check('3f the stop settled long enough: last command to record >= 2 settles (%.2f s)' % (CLK.t - t0),
          CLK.t - t0 > 2 * (c.stop_latency_s + 0.3))

    n_ok = sum(H.checks); n = len(H.checks)
    print('\n%d/%d checks passed' % (n_ok, n))
    sys.exit(0 if n_ok == n else 1)


if __name__ == '__main__':
    main()
