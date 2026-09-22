#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Operator script: HOVER the vision tip 30 mm above cross tag 0 from the
tag-102 / 103 / 104 base stops, three rounds, 3 s at each point (user
request, 2026-09-22) — a pose-mode accuracy check.

Per point: GOTO <tag> (arm homed, base aligned) -> /robot_pose -> world ->
arm frame (transform_world_to_arm, lift compensated) -> tip -> flange
(T_ee2tip) -> MoveCart to 0.20 m above -> MoveL down -> dwell -> MoveL up.
The sequence itself lives in robot_ui.tip_check (shared with
tip_touch_ref_tag.py); this file is the settings.

TARGET_WORLD_M is in the map.yaml world frame with z from the PLATE TOP
(reference_tags.yaml's frame: cross tag 0 = (-0.600, -1.200, 0.001)); the
module adds the 0.080 m plate height before the floor-datum transform.

Edit the constants and press RUN again — the file reloads every run.
DRY_RUN = True prints the arm-frame targets for the design stop poses and
moves nothing. Stop button: honoured between steps. Record:
log/apriltag_nav/tip_check/<ts>_hover30/.
"""

from robot_ui.tip_check import Settings, run_sequence

DRY_RUN = False                 # True: compute + print only, no motion

STOP_TAGS = [102, 103, 104]     # base stops, in order (zone B, front_cam)
ROUNDS = 3                      # 102 -> 103 -> 104, repeated this many times

# Vision TIP target, map.yaml world frame, z from the PLATE TOP: 30 mm
# above the plate = 29 mm above cross tag 0's top face.
TARGET_WORLD_M = (-0.600, -1.200, 0.030)

# Tool orientation in the ARM frame: tool z straight down, rz 0 at every
# stop (user's choice). (180, 0, 0) is also the flange's orientation at the
# home pose, so the approach move is a pure translation.
TOOL_RPY_DEG = (-180.0, 0.0, 0.0)

APPROACH_ABOVE_M = 0.20         # MoveCart to here first, then MoveL down
DWELL_S = 3.0                   # hold at the target
MOVE_VEL = 30.0                 # MoveCart (approach) speed %
LINE_VEL = 20.0                 # MoveL (descend / ascend) speed %
UNDOCK_IF_CHARGING = True       # send UNDOCK first when the BMS shows current


def run(ctx):
    run_sequence(ctx, Settings(
        name='hover30', target_world_m=TARGET_WORLD_M, stop_tags=STOP_TAGS,
        rounds=ROUNDS, tool_rpy_deg=TOOL_RPY_DEG, approach_above_m=APPROACH_ABOVE_M,
        dwell_s=DWELL_S, move_vel=MOVE_VEL, line_vel=LINE_VEL,
        undock_if_charging=UNDOCK_IF_CHARGING,
        standoff=False, capture=False, dry_run=DRY_RUN))
