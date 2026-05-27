#!/usr/bin/env python3
"""
verify_map_world.py
===================
OFFLINE sanity check for a calibrated ``map_world_<ts>.yaml`` — no
robot / ROS master needed. Runs two independent checks:

1. **Per-tag summary**: position (world) + rpy + ref_tag used + zone +
   original (x, y) from map.yaml for each calibrated path tag.
2. **Relative-geometry consistency**: for every ``move`` edge in
   ``map.yaml``, compare the inter-tag distance in the map frame to the
   distance in the world frame (from map_world.yaml). Large differences
   (default > 5 cm) are flagged — those entries probably failed the
   chain or were standing on a bad ref-tag measurement.

Usage::

    rosrun path_tag_locator verify_map_world.py
        # picks the most recent map_world_*.yaml under ~/.ros/path_tag_locator/locate/
        # and the package's config/map.yaml

    rosrun path_tag_locator verify_map_world.py \
        --map-world /path/to/map_world_xxx.yaml \
        --map-yaml  /path/to/map.yaml \
        --threshold-m 0.10
"""
import argparse
import math
import os
import sys
from pathlib import Path

import yaml


def _latest_map_world():
    root = Path("~/.ros/path_tag_locator").expanduser()
    if not root.exists():
        return None
    files = sorted(root.glob("map_world_*.yaml"), reverse=True)
    return str(files[0]) if files else None


def _default_map_yaml():
    try:
        import rospkg
        return str(Path(rospkg.RosPack().get_path("path_tag_locator")) /
                   "config" / "map.yaml")
    except Exception:
        here = Path(__file__).resolve().parent.parent
        return str(here / "config" / "map.yaml")


def per_tag_summary(mw: dict) -> None:
    tags = mw.get("tags", {})
    print(f"=== Per-tag summary ({len(tags)} calibrated tags) ===")
    print(f"frame: {mw.get('frame')}  origin_tag_id: {mw.get('origin_tag_id')}")
    print(f"{'tag':>5}  {'x':>8} {'y':>8} {'z':>8}  {'rz_deg':>8}  "
          f"{'ref':>4}  {'zone':>5}  map.yaml (x, y)")
    print("-" * 78)
    for tid, e in sorted(tags.items()):
        p = e.get("position_m", [0, 0, 0])
        r = e.get("rpy_deg", [0, 0, 0])
        ref = e.get("ref_tag_id", "?")
        zone = e.get("zone", "?")
        mxy = e.get("map_xy")
        mxy_str = (f"({mxy[0]:>+7.4f}, {mxy[1]:>+7.4f})"
                   if mxy else "(    n/a    )")
        print(f"{tid:>5}  {p[0]:>+8.4f} {p[1]:>+8.4f} {p[2]:>+8.4f}  "
              f"{r[2]:>+8.2f}  {str(ref):>4}  {zone:>5}  {mxy_str}")


def relative_geometry_check(mw: dict, mm: dict, threshold_m: float) -> int:
    """Compare distances along map.yaml's 'move' edges in both frames.
    Returns the number of flagged edges (|diff| > threshold)."""
    print(f"\n=== Relative geometry vs map.yaml ===")
    print(f"(flagging |Δdistance| > {threshold_m*1000:.0f} mm)")
    print(f"{'from':>5} {'to':>5}  {'map_d (m)':>10}  {'world_d (m)':>12}  "
          f"{'diff (m)':>10}")
    print("-" * 60)

    edges = mm.get("edges", []) or []
    flagged = 0
    pairs = 0
    for edge in edges:
        if edge.get("type") != "move":
            continue
        f, t = edge["from"], edge["to"]
        if f not in mw["tags"] or t not in mw["tags"]:
            continue
        m1 = (mm["tags"][f]["x"], mm["tags"][f]["y"])
        m2 = (mm["tags"][t]["x"], mm["tags"][t]["y"])
        w1 = mw["tags"][f]["position_m"][:2]
        w2 = mw["tags"][t]["position_m"][:2]
        dm = math.hypot(m1[0] - m2[0], m1[1] - m2[1])
        dw = math.hypot(w1[0] - w2[0], w1[1] - w2[1])
        diff = dw - dm
        flag = "  !!" if abs(diff) > threshold_m else ""
        pairs += 1
        if flag:
            flagged += 1
        print(f"{f:>5} {t:>5}  {dm:>10.4f}  {dw:>12.4f}  {diff:>+8.4f}{flag}")
    if pairs == 0:
        print("(no map-edges have both endpoints in map_world.yaml)")
    else:
        print(f"\nchecked {pairs} edges; {flagged} flagged "
              f"(|Δdistance| > {threshold_m*1000:.0f} mm)")
    return flagged


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--map-world", default=None,
                    help="path to map_world_*.yaml "
                         "(default: latest in ~/.ros/path_tag_locator/)")
    ap.add_argument("--map-yaml", default=None,
                    help="path to apriltag_nav-style map.yaml "
                         "(default: package's config/map.yaml)")
    ap.add_argument("--threshold-m", type=float, default=0.05,
                    help="|Δdistance| threshold for flagging (m, default 0.05)")
    args = ap.parse_args()

    mw_path = args.map_world or _latest_map_world()
    if not mw_path or not Path(mw_path).exists():
        print("error: no map_world*.yaml found; "
              "supply --map-world", file=sys.stderr)
        sys.exit(2)
    mm_path = args.map_yaml or _default_map_yaml()
    if not Path(mm_path).exists():
        print(f"error: map.yaml not found at {mm_path}; "
              "supply --map-yaml", file=sys.stderr)
        sys.exit(2)

    print(f"map_world: {mw_path}")
    print(f"map.yaml : {mm_path}")
    print()

    with open(mw_path, "r") as fh:
        mw = yaml.safe_load(fh) or {}
    with open(mm_path, "r") as fh:
        mm = yaml.safe_load(fh) or {}

    if "tags" not in mw or not mw["tags"]:
        print("error: map_world.yaml has no calibrated tags", file=sys.stderr)
        sys.exit(2)

    per_tag_summary(mw)
    n_flagged = relative_geometry_check(mw, mm, args.threshold_m)
    print()
    if n_flagged == 0:
        print("✓ all relative distances within threshold")
    else:
        print(f"⚠ {n_flagged} edge(s) outside threshold — inspect those "
              f"tag pairs (rerun the calibration, or check ref_tags.yaml "
              f"accuracy / hand_cam.png in the run archive)")


if __name__ == "__main__":
    main()
