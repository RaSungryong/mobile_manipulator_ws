#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E2E navigation test against the sim. Needs sim.launch running.

    rosrun robot_sim test_nav.py [tag_id ...]

Default route: 501 (single hop + align), then 105 (pivot + reverse
corridor entry + 8 hops). Prints arrival error vs the design stop pose
(tag - 0.55 m along the corridor heading). Pass/fail at 5 mm / 0.5 deg.
"""
import json
import math
import sys

import rospy
import yaml
from std_msgs.msg import String

from apriltag_nav.mobile_client import MobileClient
from apriltag_nav.paths import MAP_PATH

ZONE_HEADING = {"A": 0.0, "DOCK": 0.0, "B": 90.0, "D": 90.0,
                "C": -90.0, "E": -90.0}
CAMERA_OFFSET = 0.55
TOL_MM, TOL_DEG = 5.0, 0.5


def stop_pose(info):
    hd = ZONE_HEADING[info.get("zone", "A")]
    r = math.radians(hd)
    return (info["x"] - CAMERA_OFFSET * math.cos(r),
            info["y"] - CAMERA_OFFSET * math.sin(r), hd)


def main():
    targets = [int(a) for a in sys.argv[1:] if a.isdigit()] or [501, 105]
    tags = yaml.safe_load(open(MAP_PATH))["tags"]

    rospy.init_node("sim_nav_test", anonymous=True)
    truth = {}
    rospy.Subscriber("/robot_sim/ground_truth", String,
                     lambda m: truth.update(json.loads(m.data)))
    mc = MobileClient(move_timeout_s=600)
    ok, why = mc.wait_for_node(10)
    if not ok:
        print("FAIL: mobile_node not up:", why)
        sys.exit(1)

    failures = 0
    for tag in targets:
        ex, ey, eth = stop_pose(tags[tag])
        t0 = rospy.Time.now()
        ok = mc.move_to_tag(tag)
        dt = (rospy.Time.now() - t0).to_sec()
        rospy.sleep(1.0)
        dx = (truth["x"] - ex) * 1000
        dy = (truth["y"] - ey) * 1000
        dth = (truth["theta_deg"] - eth + 180) % 360 - 180
        good = (ok and abs(dx) < TOL_MM and abs(dy) < TOL_MM
                and abs(dth) < TOL_DEG)
        failures += 0 if good else 1
        print("[%s] goto %d: nav_ok=%s %.0fs  err dx=%+.1fmm dy=%+.1fmm "
              "dth=%+.2fdeg" % ("PASS" if good else "FAIL", tag, ok, dt,
                                dx, dy, dth))
    sys.exit(1 if failures else 0)


if __name__ == "__main__":
    main()
