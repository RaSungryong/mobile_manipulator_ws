#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""E2E calibration test against the sim. Needs sim.launch AND
path_tag_locator.launch running.

    rosrun robot_sim test_calibration.py [path_tag_id]

Drives the base to the tag (real mobile_node), then runs the REAL
map_calibrator on a generated single-entry plan (ref pairing + view TCP
taken from the plate plan), and compares the computed world pose against
the design ground truth. Pass at 2 mm.
"""
import json
import sys
import tempfile

import rospy
import yaml
from std_msgs.msg import String

import rospkg
from apriltag_nav.mobile_client import MobileClient
from apriltag_nav.paths import MAP_PATH
from path_tag_locator.srv import RunMapCalibration

FLOOR_Z = -0.080
TOL_MM = 2.0


def main():
    tag_id = int(sys.argv[1]) if len(sys.argv) > 1 else 105
    tags = yaml.safe_load(open(MAP_PATH))["tags"]
    info = tags[tag_id]
    plate = 1 if info["zone"] in ("B", "C") else 2

    ptl = rospkg.RosPack().get_path("path_tag_locator")
    plan = yaml.safe_load(open(
        ptl + "/config/calibration_plan_plate%d.yaml" % plate))
    entry = next(e for e in plan["plan"] if e["path_tag_id"] == tag_id)
    ref_file = (ptl + "/config/reference_tags.yaml" if plate == 1
                else ptl + "/config/reference_tags_plate2.yaml")

    rospy.init_node("sim_calib_test", anonymous=True)
    mc = MobileClient(move_timeout_s=600)
    ok, why = mc.wait_for_node(10)
    assert ok, why
    print("driving to tag %d ..." % tag_id)
    if not mc.move_to_tag(tag_id):
        print("FAIL: navigation to %d failed" % tag_id)
        sys.exit(1)

    with tempfile.NamedTemporaryFile("w", suffix=".yaml",
                                     delete=False) as fh:
        yaml.safe_dump({
            "defaults": {"retry_count": 0, "align_required": True},
            "plan": [{"path_tag_id": tag_id,
                      "ref_tag_id": entry["ref_tag_id"],
                      "arm_view_tcp_mm_deg":
                          entry["arm_view_tcp_mm_deg"]}],
        }, fh)
        plan_path = fh.name

    prog = []
    rospy.Subscriber("/map_calibrator/progress", String,
                     lambda m: prog.append(json.loads(m.data)))
    rospy.wait_for_service("/map_calibrator/run_calibration", timeout=10)
    proxy = rospy.ServiceProxy("/map_calibrator/run_calibration",
                               RunMapCalibration)
    req = RunMapCalibration._request_class()
    req.plan_path = plan_path
    req.ref_tags_path = ref_file
    resp = proxy(req)
    print("session:", resp.success, "|", resp.message)
    done = [p for p in prog if p.get("status") == "ok"]
    if not (resp.success and done):
        print("FAIL:", prog)
        sys.exit(1)
    p = done[-1]
    dx = (p["x"] - info["x"]) * 1000
    dy = (p["y"] - info["y"]) * 1000
    dz = (p.get("z", 0) - FLOOR_Z) * 1000
    good = max(abs(dx), abs(dy), abs(dz)) < TOL_MM
    print("[%s] tag %d world error: dx=%+.2fmm dy=%+.2fmm dz=%+.2fmm"
          % ("PASS" if good else "FAIL", tag_id, dx, dy, dz))
    sys.exit(0 if good else 1)


if __name__ == "__main__":
    main()
