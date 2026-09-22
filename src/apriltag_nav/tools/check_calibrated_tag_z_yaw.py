#!/usr/bin/env python3
"""Offline check: the calibrated tag `z` and `yaw` of map.yaml are USED
(2026-09-22, user: "x, y 말고 모든 값 사용").

  * `arm_transform.tag_floor_z_m`: the floor under a calibrated tag above
    the CSV z datum — (z - tag_thickness) - world_floor_z_m; 0 without a
    `z` or with use_tag_z false; refused beyond tag_z_max_offset_m.
  * `transform_world_to_arm(..., floor_z_m)`: enters exactly like the lift
    — only the arm base height moves (x/y follow the mount tilt by < 1 mm),
    and floor 0 is the pre-change result bit for bit.
  * `MobileController.calculate_robot_pose`: theta gains the tag's laying
    angle — map.yaml `yaw` minus the nearest 90-deg axis — in every zone,
    with the sign of the world CCW convention; tags without `yaw`, the
    config switch off, and a nonsense yaw all give the old theta.
  * The real map.yaml: every calibrated tag has a z inside the refusal
    limit and a yaw within 2 deg of an axis; uncalibrated tags have neither.

Runs with no ROS master (rospy / msgs stubbed by the nav harness).
"""
import copy
import importlib.util
import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location(
    'check_nav_sequencing', os.path.join(HERE, 'check_nav_sequencing.py'))
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)          # installs the rospy / msg stubs
check, make, CFG0 = H.check, H.make, H.CFG0

from apriltag_nav.arm_transform import tag_floor_z_m, transform_world_to_arm  # noqa: E402
from apriltag_nav import paths  # noqa: E402


class Msg:
    def __init__(self, x, y, theta, id=0):
        self.x, self.y, self.theta, self.id = x, y, theta, id


def main():
    calib = {'use_tag_z': True, 'world_floor_z_m': -0.080,
             'tag_z_max_offset_m': 0.05}

    # ---- tag_floor_z_m ----
    check('no z -> 0.0', tag_floor_z_m({'x': 1.0}, calib) == 0.0)
    check('None tag -> 0.0', tag_floor_z_m(None, calib) == 0.0)
    fz = tag_floor_z_m({'z': -0.0585}, calib)
    check('z -0.0585 -> floor +20.5 mm', abs(fz - 0.0205) < 1e-9, f'{fz*1e3:+.2f} mm')
    check('design tag top -0.079 -> floor 0', abs(tag_floor_z_m({'z': -0.079}, calib)) < 1e-9)
    check('use_tag_z false -> 0.0',
          tag_floor_z_m({'z': -0.0585}, dict(calib, use_tag_z=False)) == 0.0)
    try:
        tag_floor_z_m({'z': -0.020}, calib)
        check('z beyond the limit refused', False)
    except ValueError as e:
        check('z beyond the limit refused', 'refusing' in str(e), str(e)[:60])
    check('limit is configurable',
          abs(tag_floor_z_m({'z': -0.020}, dict(calib, tag_z_max_offset_m=0.1)) - 0.059) < 1e-9)

    # ---- transform_world_to_arm with floor_z ----
    g = {'x': 0.3, 'y': -1.0, 'z': 0.35, 'rx': -1.5707, 'ry': -0.03, 'rz': 3.137}
    msg = Msg(-0.0, 1.711, 90.0, id=106)
    p0, r0 = transform_world_to_arm(g, msg, 0.0)
    p1, r1 = transform_world_to_arm(g, msg, 0.0, floor_z_m=0.0)
    check('floor 0 is bit-identical to the old call',
          np.array_equal(p0, p1) and np.array_equal(r0, r1))
    p2, r2 = transform_world_to_arm(g, msg, 0.0, floor_z_m=0.0205)
    d = p2 - p0
    check('floor +20.5 mm lowers the arm-frame target ~20.5 mm',
          abs(d[2] + 20.5) < 0.2, f'dz {d[2]:+.2f} mm')
    check('x/y move only by the mount tilt (< 1 mm)',
          abs(d[0]) < 1.0 and abs(d[1]) < 1.0, f'dx {d[0]:+.2f} dy {d[1]:+.2f}')
    check('orientation unchanged', np.allclose(r0, r2))
    p3, _ = transform_world_to_arm(g, msg, 0.150, floor_z_m=0.0205)
    p4, _ = transform_world_to_arm(g, msg, 0.1705, floor_z_m=0.0)
    check('floor enters exactly like the lift', np.allclose(p3, p4, atol=1e-9))

    # ---- calculate_robot_pose with the calibrated yaw ----
    class Map:
        def __init__(self, info): self.info = info
        def get_tag_info(self, t): return self.info
        def get_edge(self, a, b): return None
    for zone, base, x, y in (('B', 90.0, -1.71, 1.05), ('C', -90.0, 1.71, -1.05),
                             ('A', 0.0, 0.0, 3.02)):
        for yaw, expect in ((None, 0.0), (0.58, 0.58), (-0.41, -0.41),
                            (179.69, -0.31), (-179.73, 0.27), (90.4, 0.4)):
            c, p = make(0, 0, 0, {5: (0.55, 0.0, 0.0)})
            info = {'x': x, 'y': y, 'zone': zone}
            if yaw is not None:
                info['yaw'] = yaw
            c.map_mgr = Map(info)
            p.push_cam()                     # square tag on the crosshair
            pose = c.calculate_robot_pose(5)
            ok = pose is not None and abs(pose[2] - (base + expect)) < 1e-6
            check(f'zone {zone} yaw {yaw} -> theta {base + expect:+.2f}', ok,
                  f'got {pose[2] if pose else None}')
    c, p = make(0, 0, 0, {5: (0.55, 0.0, 0.0)})
    c.map_mgr = Map({'x': -1.71, 'y': 1.05, 'zone': 'B', 'yaw': 0.58})
    c.robot_pose_use_tag_yaw = False
    p.push_cam()
    check('switch off -> zone heading only', abs(c.calculate_robot_pose(5)[2] - 90.0) < 1e-9)
    c.robot_pose_use_tag_yaw = True
    c.map_mgr = Map({'x': -1.71, 'y': 1.05, 'zone': 'B', 'yaw': 30.0})
    check('a yaw 30 deg off every axis is ignored', abs(c.calculate_robot_pose(5)[2] - 90.0) < 1e-9)
    # sign: a body squared to a tag laid +0.58 deg CCW reads align 0 and
    # heading 90.58 — the tag's rotation carried 1:1 into theta
    c.map_mgr = Map({'x': -1.71, 'y': 1.05, 'zone': 'B', 'yaw': 0.58})
    check('CCW-laid tag -> larger theta (CCW +)', c.calculate_robot_pose(5)[2] > 90.0)

    # ---- the real map.yaml ----
    tags = yaml.safe_load(open(paths.MAP_PATH))['tags']
    calib_real = (yaml.safe_load(open(paths.CONFIG_PATH)).get('arm_calibration') or {})
    have = [t for t, v in tags.items() if 'z' in v]
    check('26 tags carry z', len(have) == 26 and sorted(have) == list(range(100, 126)), str(len(have)))
    check('every z inside the refusal limit',
          all(abs(tag_floor_z_m(tags[t], calib_real)) <= 0.05 for t in have))
    check('every yaw within 2 deg of an axis',
          all(abs((tags[t]['yaw'] + 45) % 90 - 45) < 2.0 for t in have))
    check('zone B tags laid along +x, zone C along -x',
          all(abs(tags[t]['yaw']) < 2 for t in range(100, 113)) and
          all(abs(abs(tags[t]['yaw']) - 180) < 2 for t in range(113, 126)))
    check('uncalibrated tags have neither key',
          all('z' not in v and 'yaw' not in v for t, v in tags.items() if t not in have))
    # user's decision (2026-09-22 evening): yaw IS used, z is recorded only
    check('robot.yaml: yaw on', bool((yaml.safe_load(open(paths.CONFIG_PATH))['robot']).get('robot_pose_use_tag_yaw')))
    check('robot.yaml: z OFF (user decision)', calib_real.get('use_tag_z') is False)
    check('with the real config every tag gives floor 0',
          all(tag_floor_z_m(tags[t], calib_real) == 0.0 for t in have))

    bad = sum(1 for ok in H.checks if not ok) if hasattr(H, 'checks') else 0
    return bad


if __name__ == '__main__':
    bad = main()
    n = len(H.checks)
    print(f'{n - bad}/{n} checks passed')
    sys.exit(1 if bad else 0)
