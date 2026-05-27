#!/usr/bin/env python3
"""
visualize_map_world.py
======================
Publish RViz markers for a calibrated ``map_world.yaml`` (and,
optionally, the user's ``reference_tags.yaml``). Useful for spotting
flyaway / mirrored / mis-yawed tags at a glance.

Topics:
  ~markers   visualization_msgs/MarkerArray   (latched)

Marker layout:
  - ref tags        : red squares  (size 80 mm) + red text labels "R<id>"
  - calibrated tags : green squares (size 60 mm) + green text labels "<id>"
  - world origin    : RGB axes (x=red, y=green, z=blue, length 0.25 m)

frame_id of every marker = "world".

Usage::

    # Defaults (latest map_world_*.yaml + package's reference_tags.yaml)
    rosrun path_tag_locator visualize_map_world.py

    # Then in rviz:
    #   Fixed Frame = world
    #   Displays -> Add -> MarkerArray  -> topic = /visualize_map_world/markers
"""
import argparse
import os
import re
import sys
from pathlib import Path

import numpy as np
import rospy
import rospkg
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from path_tag_locator.calibration.plan_io import load_reference_tags
from path_tag_locator.geometry import R_to_quat_xyzw, rpy_deg_to_R


_FIND_RE = re.compile(r"\$\(find\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")


def _resolve_ros_path(p):
    if not p:
        return p
    try:
        rp = rospkg.RosPack()
        s = _FIND_RE.sub(lambda m: rp.get_path(m.group(1)), str(p))
    except Exception:
        s = str(p)
    return os.path.expandvars(os.path.expanduser(s))


def _latest_map_world():
    root = Path("~/.ros/path_tag_locator").expanduser()
    if not root.exists():
        return None
    files = sorted(root.glob("map_world_*.yaml"), reverse=True)
    return str(files[0]) if files else None


def _default_ref_tags():
    try:
        return str(Path(rospkg.RosPack().get_path("path_tag_locator")) /
                   "config" / "reference_tags.yaml")
    except Exception:
        return None


def _quat_from_rpy_deg(rpy):
    R = rpy_deg_to_R(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    qx, qy, qz, qw = R_to_quat_xyzw(R)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def _make_cube(ns, mid, pos, color, scale=0.06, rpy=None, frame="world"):
    m = Marker()
    m.header = Header(stamp=rospy.Time.now(), frame_id=frame)
    m.ns = ns
    m.id = mid
    m.action = Marker.ADD
    m.type = Marker.CUBE
    m.scale = Vector3(scale, scale, 0.005)
    m.color = ColorRGBA(*color)
    m.pose.position = Point(x=float(pos[0]), y=float(pos[1]),
                             z=float(pos[2]) if len(pos) > 2 else 0.0)
    m.pose.orientation = (_quat_from_rpy_deg(rpy) if rpy is not None
                          else Quaternion(x=0, y=0, z=0, w=1))
    return m


def _make_text(ns, mid, pos, text, color, size=0.05, frame="world"):
    m = Marker()
    m.header = Header(stamp=rospy.Time.now(), frame_id=frame)
    m.ns = ns
    m.id = mid
    m.action = Marker.ADD
    m.type = Marker.TEXT_VIEW_FACING
    m.scale = Vector3(0, 0, size)
    m.color = ColorRGBA(*color)
    m.pose.position = Point(x=float(pos[0]), y=float(pos[1]),
                             z=float(pos[2]) if len(pos) > 2 else 0.0)
    m.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)
    m.text = text
    return m


def _make_axis(ns, mid, color, axis_vec, length=0.25, frame="world"):
    m = Marker()
    m.header = Header(stamp=rospy.Time.now(), frame_id=frame)
    m.ns = ns
    m.id = mid
    m.action = Marker.ADD
    m.type = Marker.ARROW
    m.scale = Vector3(0.01, 0.02, 0.0)  # shaft, head diameter, head len
    m.color = ColorRGBA(*color)
    m.points = [
        Point(x=0.0, y=0.0, z=0.0),
        Point(x=length * axis_vec[0],
              y=length * axis_vec[1],
              z=length * axis_vec[2]),
    ]
    return m


def build(mw, refs, args):
    markers = MarkerArray()
    next_id = [0]
    def push(m):
        markers.markers.append(m)
        next_id[0] += 1

    # 1. World origin axes.
    for color, vec in [
        ((1, 0, 0, 0.9), (1, 0, 0)),
        ((0, 1, 0, 0.9), (0, 1, 0)),
        ((0, 0, 1, 0.9), (0, 0, 1)),
    ]:
        push(_make_axis("origin", next_id[0], color, vec,
                        length=args.axis_length_m))
    push(_make_text("origin", next_id[0],
                    (0.05, 0.05, 0.05), "world", (1, 1, 1, 1)))

    # 2. Ref tags.
    for tid, ref in (refs or {}).items():
        pos = ref.T_world[:3, 3].tolist()
        rpy_deg = None
        # Recover rpy from the rotation matrix:
        from path_tag_locator.geometry import rot2rpy_deg
        rx, ry, rz = rot2rpy_deg(ref.T_world[:3, :3])
        rpy_deg = (rx, ry, rz)
        push(_make_cube("ref_tags", next_id[0], pos, (1, 0, 0, 0.9),
                        scale=0.08, rpy=rpy_deg))
        push(_make_text("ref_labels", next_id[0],
                        (pos[0], pos[1], pos[2] + 0.05),
                        f"R{tid}", (1, 0.3, 0.3, 1)))

    # 3. Calibrated path tags.
    for tid, e in (mw.get("tags") or {}).items():
        pos = e["position_m"]
        rpy = e.get("rpy_deg", [0, 0, 0])
        push(_make_cube("path_tags", next_id[0], pos, (0, 1, 0, 0.85),
                        scale=0.06, rpy=rpy))
        push(_make_text("path_labels", next_id[0],
                        (pos[0], pos[1], pos[2] + 0.04),
                        f"{tid}", (0.3, 1, 0.3, 1), size=0.04))

    return markers


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--map-world", default=None)
    ap.add_argument("--ref-tags", default=None)
    ap.add_argument("--axis-length-m", type=float, default=0.25)
    args = ap.parse_args()

    rospy.init_node("visualize_map_world", anonymous=True)
    pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)

    mw_path = _resolve_ros_path(args.map_world) or _latest_map_world()
    if not mw_path:
        rospy.logfatal("no map_world*.yaml found; supply --map-world")
        sys.exit(2)
    rospy.loginfo("map_world: %s", mw_path)
    mw = yaml.safe_load(open(mw_path)) or {}

    refs = {}
    ref_path = _resolve_ros_path(args.ref_tags) or _default_ref_tags()
    if ref_path and os.path.exists(ref_path):
        rospy.loginfo("ref_tags : %s", ref_path)
        try:
            refs = load_reference_tags(ref_path)
        except Exception as e:
            rospy.logwarn("could not load ref tags: %s", e)

    markers = build(mw, refs, args)
    pub.publish(markers)
    rospy.loginfo("published %d markers on %s (frame=world). "
                  "Add MarkerArray display in RViz subscribed to that topic.",
                  len(markers.markers), rospy.resolve_name("~markers"))
    rospy.spin()


if __name__ == "__main__":
    main()
