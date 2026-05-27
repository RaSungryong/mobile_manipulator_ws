#!/usr/bin/env python3
"""
test_repeatability.py
=====================
Run the map calibration twice in a row (same plan, same defaults) and
diff the two ``map_world*.yaml`` outputs. The per-tag |Δxy| gives a
direct measurement of the **actual** precision of the chain
(detection + alignment + extrinsics jitter combined).

Typical good numbers (after hand-eye calibration is solid):
  - |Δxy| mean < 3 mm
  - |Δxy| max  < 1 cm

Anything systematically > 1 cm points to either a noisy detection
(check hand_cam.png archives) or unstable hand-eye calibration.

Usage::

    # Requires path_tag_locator + map_calibrator running, plus the
    # base parked in front of the plan's first nav_start_id.
    rosrun path_tag_locator test_repeatability.py

    # Or with explicit output paths:
    rosrun path_tag_locator test_repeatability.py \
        --out1 /tmp/run_a.yaml --out2 /tmp/run_b.yaml
"""
import argparse
import math
import os
import sys
import time

import rospy
import yaml


def call_run_calibration(out_path: str):
    from path_tag_locator.srv import (RunMapCalibration,
                                       RunMapCalibrationRequest)
    rospy.wait_for_service("/map_calibrator/run_calibration", timeout=10.0)
    proxy = rospy.ServiceProxy(
        "/map_calibrator/run_calibration", RunMapCalibration)
    req = RunMapCalibrationRequest(
        plan_path="", ref_tags_path="", map_in_path="",
        map_out_path=out_path, dry_run=False,
    )
    rospy.loginfo("calling run_calibration -> %s", out_path)
    resp = proxy(req)
    rospy.loginfo("  success=%s succeeded=%d failed=%d out=%s",
                  resp.success, resp.num_succeeded, resp.num_failed,
                  resp.output_yaml_path)
    if not resp.success:
        rospy.logwarn("  message: %s", resp.message)
    return resp.output_yaml_path


def diff_two(p1: str, p2: str) -> int:
    with open(p1, "r") as fh: a = yaml.safe_load(fh) or {}
    with open(p2, "r") as fh: b = yaml.safe_load(fh) or {}
    ta = a.get("tags", {}) or {}
    tb = b.get("tags", {}) or {}
    common = sorted(set(ta.keys()) & set(tb.keys()))

    only_a = sorted(set(ta.keys()) - set(tb.keys()))
    only_b = sorted(set(tb.keys()) - set(ta.keys()))
    if only_a or only_b:
        print(f"\n⚠ tag sets differ between runs:")
        if only_a: print(f"  only in run A: {only_a}")
        if only_b: print(f"  only in run B: {only_b}")

    if not common:
        print("\nno common tags to diff", file=sys.stderr)
        return 1

    print(f"\n=== Per-tag diff ({len(common)} tags) ===")
    print(f"{'tag':>5}  {'Δx mm':>+9}  {'Δy mm':>+9}  {'Δz mm':>+9}  "
          f"{'|Δxy| mm':>11}")
    print("-" * 56)
    deltas_xy = []
    for tid in common:
        pa = ta[tid]["position_m"]
        pb = tb[tid]["position_m"]
        dx_mm = (pb[0] - pa[0]) * 1000.0
        dy_mm = (pb[1] - pa[1]) * 1000.0
        dz_mm = (pb[2] - pa[2]) * 1000.0
        dxy = math.hypot(dx_mm, dy_mm)
        deltas_xy.append(dxy)
        flag = "  !!" if dxy > 10.0 else ""
        print(f"{tid:>5}  {dx_mm:>+9.2f}  {dy_mm:>+9.2f}  {dz_mm:>+9.2f}  "
              f"{dxy:>11.2f}{flag}")

    mean = sum(deltas_xy) / len(deltas_xy)
    print(f"\n|Δxy| stats: mean={mean:.2f} mm, "
          f"median={sorted(deltas_xy)[len(deltas_xy)//2]:.2f} mm, "
          f"max={max(deltas_xy):.2f} mm")
    if mean < 3.0:
        print("✓ excellent repeatability (mean < 3 mm)")
    elif mean < 10.0:
        print("✓ acceptable repeatability (mean < 1 cm)")
    else:
        print("⚠ high inter-run drift — inspect hand_cam.png / extrinsics / "
              "hand-eye residual")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--out1", default="/tmp/path_tag_locator_run_a.yaml")
    ap.add_argument("--out2", default="/tmp/path_tag_locator_run_b.yaml")
    ap.add_argument("--pause-s", type=float, default=2.0,
                    help="pause between runs (s)")
    args = ap.parse_args()

    rospy.init_node("test_repeatability", anonymous=True)
    rospy.loginfo("=== run A ===")
    out1 = call_run_calibration(args.out1)
    rospy.loginfo("pausing %.1fs before run B (drive base back if needed)...",
                  args.pause_s)
    time.sleep(args.pause_s)
    rospy.loginfo("=== run B ===")
    out2 = call_run_calibration(args.out2)

    if not (os.path.exists(out1) and os.path.exists(out2)):
        rospy.logfatal("one of the outputs is missing; cannot diff")
        sys.exit(2)
    sys.exit(diff_two(out1, out2))


if __name__ == "__main__":
    main()
