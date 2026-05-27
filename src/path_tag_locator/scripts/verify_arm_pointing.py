#!/usr/bin/env python3
"""
verify_arm_pointing.py
======================
Physical closed-loop verification of map_world.yaml. For each calibrated
path tag visible to the front camera, the arm is moved so the hand
camera **should** look straight at that tag (using the tag's calibrated
world pose to compute the view pose). We then capture a hand-cam image,
re-detect the same tag, and measure how far it lands from image center.

Small residual (< 2 cm) means the calibration is self-consistent.
Large residual on one tag means that tag's `map_world.yaml` entry is
inaccurate (or the base position estimate via front-cam happens to be
inaccurate at that moment).

Prerequisites:
  - path_tag_locator is launched (so /path_tag_locator params + npz
    + extrinsics are loaded into the node and on the param server).
  - The base is parked near one of the calibrated tags so the front
    cam sees it. For multi-tag testing, drive the base between tags
    yourself and re-run with --tag-id N.

Usage::

    # Test one tag (base already parked in front of it):
    rosrun path_tag_locator verify_arm_pointing.py --tag-id 101

    # Bulk: iterate through every tag in the latest map_world.yaml
    # that front-cam currently sees. (Practical only for tags within
    # current base view.)
    rosrun path_tag_locator verify_arm_pointing.py --all
"""
import argparse
import math
import os
import re
import sys
import time
from pathlib import Path

import numpy as np
import rospy
import rospkg
import yaml

from path_tag_locator.calibration.view_pose import (
    compute_T_world2mb_from_chain,
    compute_view_tcp,
)
from path_tag_locator.constants import (
    load_extrinsics,
    load_locator_cfg_from_dict,
)
from path_tag_locator.detect import detect_apriltag
from path_tag_locator.geometry import assert_rigid, rpy_deg_to_R
from path_tag_locator.hand_eye import load_T_hc2ee
from path_tag_locator.ros_image import grab_image_and_K
from path_tag_locator.tcp_pose import FairinoTCPClient


_FIND_RE = re.compile(r"\$\(find\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")


def _resolve_ros_path(p):
    if not p:
        return p
    rp = rospkg.RosPack()
    return os.path.expandvars(os.path.expanduser(
        _FIND_RE.sub(lambda m: rp.get_path(m.group(1)), str(p))))


def _latest_map_world():
    root = Path("~/.ros/path_tag_locator").expanduser()
    files = sorted(root.glob("map_world_*.yaml"), reverse=True)
    return str(files[0]) if files else None


def _entry_to_T(entry):
    pos = entry["position_m"]
    rpy = entry.get("rpy_deg", [0.0, 0.0, 0.0])
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rpy_deg_to_R(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    T[:3, 3] = pos
    return T


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--map-world", default=None)
    ap.add_argument("--tag-id", type=int, default=None,
                    help="test only this tag (must currently be in front-cam view)")
    ap.add_argument("--all", action="store_true",
                    help="iterate every calibrated tag — only tags front-cam "
                         "sees right now will actually be tested")
    ap.add_argument("--view-distance-m", type=float, default=0.20)
    ap.add_argument("--ok-threshold-m", type=float, default=0.02,
                    help="|xy| in hand-cam considered OK (default 2 cm)")
    args = ap.parse_args()

    rospy.init_node("verify_arm_pointing", anonymous=True)

    # Pull locator config off the global param server (set up by
    # path_tag_locator.launch — load it first if not running).
    loc_params = rospy.get_param("/path_tag_locator", None)
    if loc_params is None:
        rospy.logfatal("/path_tag_locator/* params not on the server. "
                       "Launch path_tag_locator first.")
        sys.exit(2)
    cfg = load_locator_cfg_from_dict(loc_params)

    T_hc2ee = load_T_hc2ee(_resolve_ros_path(cfg.hand_eye_npz))
    assert_rigid(T_hc2ee, name="T_hc2ee")
    T_ab2mb, T_mb2fc = load_extrinsics(_resolve_ros_path(cfg.extrinsics_yaml))

    if not cfg.robot.use_sdk:
        rospy.logfatal("robot.use_sdk=false: this test needs the arm")
        sys.exit(2)
    client = FairinoTCPClient(
        robot_ip=cfg.robot.robot_ip,
        sdk_path=cfg.robot.fairino_sdk_path,
        tcp_index=cfg.robot.tcp_index,
        default_vel=cfg.align.move_vel,
        default_acc=cfg.align.move_acc,
        default_ovl=cfg.align.move_ovl,
    )

    mw_path = args.map_world or _latest_map_world()
    if not mw_path:
        rospy.logfatal("no map_world*.yaml found; supply --map-world")
        sys.exit(2)
    rospy.loginfo("using %s", mw_path)
    mw = yaml.safe_load(open(mw_path)) or {}
    all_tags = mw.get("tags", {}) or {}

    if args.tag_id is not None:
        candidate_ids = [int(args.tag_id)]
    elif args.all:
        candidate_ids = sorted(int(k) for k in all_tags.keys())
    else:
        rospy.logfatal("supply --tag-id N or --all")
        sys.exit(2)

    results = []
    for tid in candidate_ids:
        if tid not in all_tags:
            rospy.logwarn("tag %d not in map_world; skipping", tid)
            continue
        rospy.loginfo("---- tag %d ----", tid)
        entry = all_tags[tid]
        T_tag_world = _entry_to_T(entry)

        # 1. Front cam: capture + detect the same tag id.
        try:
            fc_img, K_fc = grab_image_and_K(
                cfg.topics.front_cam_image, cfg.topics.front_cam_info,
                timeout=cfg.io.image_wait_timeout)
        except Exception as e:
            rospy.logwarn("front-cam capture failed: %s", e)
            results.append((tid, None, None, "fc_capture_failed"))
            continue
        T_fc2tag = detect_apriltag(
            fc_img, K_fc, float(cfg.tag.tag_b_size_m),
            tid, family=cfg.tag.family)
        if T_fc2tag is None:
            rospy.logwarn("tag %d NOT in front-cam view — skip "
                          "(park base near it)", tid)
            results.append((tid, None, None, "fc_no_detect"))
            continue

        # 2. Derive current T_world2mb from the just-observed tag pose.
        T_world2mb = compute_T_world2mb_from_chain(T_tag_world, T_fc2tag, T_mb2fc)
        rospy.loginfo("  T_world2mb (from front-cam) = (%.3f, %.3f, %.3f)",
                      *T_world2mb[:3, 3])

        # 3. Compute view_tcp targeting this tag.
        view_tcp = compute_view_tcp(
            T_A_world=T_tag_world,
            T_world2mb=T_world2mb,
            T_ab2mb=T_ab2mb,
            T_hc2ee=T_hc2ee,
            view_distance_m=args.view_distance_m,
        )
        rospy.loginfo("  view_tcp = %s",
                      ["%.2f" % v for v in view_tcp])

        # 4. Move arm.
        try:
            client.move_j_to_pose(view_tcp,
                                  settle_s=max(cfg.align.move_settle_s, 0.5))
        except Exception as e:
            rospy.logwarn("  MoveJ failed: %s", e)
            results.append((tid, None, None, f"movej_failed: {e}"))
            continue

        # 5. Hand-cam: capture + redetect the same tag, measure offset.
        try:
            hc_img, K_hc = grab_image_and_K(
                cfg.topics.hand_cam_image, cfg.topics.hand_cam_info,
                timeout=cfg.io.image_wait_timeout)
        except Exception as e:
            rospy.logwarn("  hand-cam capture failed: %s", e)
            results.append((tid, None, None, f"hc_capture_failed: {e}"))
            continue
        T_hc2tag = detect_apriltag(
            hc_img, K_hc, float(cfg.tag.tag_b_size_m),
            tid, family=cfg.tag.family)
        if T_hc2tag is None:
            rospy.logwarn("  hand-cam does NOT see tag %d ❌", tid)
            results.append((tid, None, None, "hc_no_detect"))
            continue

        xy = math.hypot(T_hc2tag[0, 3], T_hc2tag[1, 3])
        z = float(T_hc2tag[2, 3])
        ok = xy <= args.ok_threshold_m
        rospy.loginfo("  hand-cam sees tag %d: xy_offset=%.1f mm, z=%.1f mm  %s",
                      tid, xy * 1000, z * 1000,
                      "OK" if ok else f"❌ (> {args.ok_threshold_m*1000:.0f} mm)")
        results.append((tid, xy, z, "ok" if ok else "high_xy"))

    # Summary
    print("\n=== Summary ===")
    valid = [(t, xy, z, s) for t, xy, z, s in results if xy is not None]
    print(f"tested: {len(results)}, detected: {len(valid)}, "
          f"NOT detected: {len(results) - len(valid)}")
    if valid:
        offsets = [xy for _, xy, _, _ in valid]
        print(f"xy_offset: mean={sum(offsets)/len(offsets)*1000:.1f} mm, "
              f"median={sorted(offsets)[len(offsets)//2]*1000:.1f} mm, "
              f"max={max(offsets)*1000:.1f} mm")
        bad = [t for t, xy, _, _ in valid if xy > args.ok_threshold_m]
        if bad:
            print(f"⚠ likely-miscalibrated tags (xy > {args.ok_threshold_m*1000:.0f} mm): {bad}")
        else:
            print("✓ every tested tag within threshold")


if __name__ == "__main__":
    main()
