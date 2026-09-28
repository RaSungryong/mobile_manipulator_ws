#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Operator script: tip on the paired CROSS TAG from every plate-1 drive tag
(user request, 2026-09-28) — the tip-error tour.

Per drive tag, in STOP_TAGS order (same sequence as tip_touch_ref_tag.py,
robot_ui.tip_check): GOTO <tag> → world -> arm transform from THAT stop's
/robot_pose → MoveCart 0.20 m above the cross tag this drive tag is paired
with → MoveL down to 1 mm above its top face → record → Keyence standoff →
record → Basler frame with the VISION lamp → MoveL up → next tag.

Pairing = the map-calibration plan's (REF_RANGES in
path_tag_locator/scripts/generate_calibration_artifacts.py). On each cross
tag a 20 mm tag36h11 marker sits on the centre (MARKER_ID): the Basler
frame shows where the tip really landed, which
src/robot_ui/tools/analyze_tip_check.py turns into a world-axes error per
stop. Drive tags sharing a cross tag see the SAME physical point, so their
differences are purely the stop pose (/robot_pose) error.

Unreachable pairs (the lane-end tags 100 / 112 / 113 / 125 are 1.2 m along
the lane from their cross tag: flange 1.31–1.42 m > the 1.25 m bound) are
left out of STOP_TAGS; SKIP_REFUSED records any other out-of-bounds stop
and continues instead of ending the tour.

Records: log/apriltag_nav/tip_check/<ts>_cross_tags/, images
results/tip_check/<ts>_cross_tags/r<n>_tag<id>.png.
"""

import importlib
import os

import yaml

from robot_ui import paths

DRY_RUN = False                 # True: compute + print only, no motion

# drive tag ranges -> cross tag (plate 1), as in the calibration plan
PAIRING = [(100, 104, 0), (105, 107, 1), (108, 112, 2),
           (113, 117, 3), (118, 120, 4), (121, 125, 5)]
# small 20 mm marker laid on each cross tag's centre (user, 2026-09-28)
# (2026-09-28: first 230..225, replaced the same day by 219..224)
MARKER_ID = {0: 219, 1: 220, 2: 221, 3: 222, 4: 223, 5: 224}

# 22 drive tags; 100 / 112 / 113 / 125 cannot reach their cross tag
STOP_TAGS = [101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
             114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124]
ROUNDS = 1
SKIP_REFUSED = True             # out-of-bounds stop: record + continue

TIP_ABOVE_TAG_M = 0.001         # tip target 1 mm above the cross tag's top face
REF_TAGS_YAML = os.path.join(paths.WS_DIR, 'src', 'path_tag_locator', 'config',
                             'reference_tags.yaml')

TOOL_RPY_DEG = (-180.0, 0.0, 0.0)   # tool straight down, rz 0 (arm frame)
APPROACH_ABOVE_M = 0.20
DWELL_S = 1.0                       # after the capture, before ascending
MOVE_VEL = 30.0
LINE_VEL = 20.0
UNDOCK_IF_CHARGING = True

PRE_STANDOFF_WAIT_S = 1.0
STANDOFF_TARGET_MM = None           # None = robot.yaml keyence.target_distance_mm (16.5)
STANDOFF_TIMEOUT_S = 120.0

CAPTURE_SAMPLES = 1
CAPTURE_USE_LED = True


def ref_for(tag):
    for lo, hi, ref in PAIRING:
        if lo <= tag <= hi:
            return ref
    raise ValueError(f'drive tag {tag} is not paired with a cross tag')


def cross_tag_positions():
    """{cross tag id: (x, y, z)} from reference_tags.yaml (plate-top frame)."""
    with open(REF_TAGS_YAML) as f:
        doc = yaml.safe_load(f)
    return {int(r['id']): tuple(float(v) for v in r['position_m'])
            for r in doc['reference_tags']}


def build_targets(stop_tags):
    refs = cross_tag_positions()
    targets, info = {}, {}
    for tag in stop_tags:
        ref = ref_for(tag)
        x, y, z = refs[ref]
        targets[tag] = (x, y, z + TIP_ABOVE_TAG_M)
        info[tag] = {'ref_tag': ref, 'marker_id': MARKER_ID[ref]}
    return targets, info


def run(ctx):
    # The plugin runner reloads THIS file, not robot_ui.tip_check; a web node
    # that already ran a tip plugin would keep the pre-2026-09-28 module
    # (no per-stop targets). Reload it so no node restart is needed.
    tc = importlib.reload(importlib.import_module('robot_ui.tip_check'))
    targets, info = build_targets(STOP_TAGS)
    tc.run_sequence(ctx, tc.Settings(
        name='cross_tags', target_world_m=targets[STOP_TAGS[0]], stop_tags=STOP_TAGS,
        rounds=ROUNDS, tool_rpy_deg=TOOL_RPY_DEG, approach_above_m=APPROACH_ABOVE_M,
        dwell_s=DWELL_S, move_vel=MOVE_VEL, line_vel=LINE_VEL,
        undock_if_charging=UNDOCK_IF_CHARGING,
        standoff=True, standoff_target_mm=STANDOFF_TARGET_MM,
        pre_standoff_wait_s=PRE_STANDOFF_WAIT_S, standoff_timeout_s=STANDOFF_TIMEOUT_S,
        capture=True, capture_samples=CAPTURE_SAMPLES, capture_use_led=CAPTURE_USE_LED,
        dry_run=DRY_RUN, targets=targets, target_info=info, skip_refused=SKIP_REFUSED))
