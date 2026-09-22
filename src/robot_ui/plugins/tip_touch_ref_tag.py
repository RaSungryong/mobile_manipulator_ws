#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Operator script: put the vision TIP ON cross tag 0 (1 mm above its top
face), let the Keyence set the standoff, photograph it with the VISION
lamp — from the tag-102 / 103 / 104 base stops, three rounds (user
request, 2026-09-22).

Per point, after GOTO <tag> and the world -> arm transform (see
tip_over_ref_tag.py / robot_ui.tip_check):
    MoveCart to 0.20 m above, MoveL down to the target
    record "before": TCP, joints, tip position in world axes
    wait PRE_STANDOFF_WAIT_S, /arm/standoff (the scan's own Keyence loop,
        target STANDOFF_TARGET_MM; None = robot.yaml keyence.target_distance_mm)
    record "after_correction": TCP, joints, tip in world, standoff result
    /camera/capture with the VISION lamp -> results/tip_check/<session>/r<n>_tag<id>.png
    dwell DWELL_S, MoveL up; next tag
A standoff that does not converge is recorded (reason in
after_correction.standoff) and the capture still happens at that height.

z = 0.002 from the PLATE TOP: the tip is the surface point the Basler
centre looks at with the case at its 16.5 mm zero standoff, so the case
bottom arrives ~16.5 mm above the tag and the Keyence loop trims it.
Records: log/apriltag_nav/tip_check/<ts>_touch/ (summary.csv, one yaml
per point with before / after_correction blocks).
"""

from robot_ui.tip_check import Settings, run_sequence

DRY_RUN = False                 # True: compute + print only, no motion

STOP_TAGS = [102, 103, 104]
ROUNDS = 3

# Vision TIP target, map.yaml world frame, z from the PLATE TOP:
# 1 mm above cross tag 0's top face (0.001).
TARGET_WORLD_M = (-0.600, -1.200, 0.002)

TOOL_RPY_DEG = (-180.0, 0.0, 0.0)   # tool straight down, rz 0 (arm frame)
APPROACH_ABOVE_M = 0.20
DWELL_S = 3.0                       # after the capture, before ascending
MOVE_VEL = 30.0
LINE_VEL = 20.0
UNDOCK_IF_CHARGING = True

PRE_STANDOFF_WAIT_S = 1.0           # at the target, before the Keyence loop
STANDOFF_TARGET_MM = None           # None = robot.yaml keyence.target_distance_mm (16.5)
STANDOFF_TIMEOUT_S = 120.0

CAPTURE_SAMPLES = 1                 # Basler frames per point, VISION lamp on
CAPTURE_USE_LED = True


def run(ctx):
    run_sequence(ctx, Settings(
        name='touch', target_world_m=TARGET_WORLD_M, stop_tags=STOP_TAGS,
        rounds=ROUNDS, tool_rpy_deg=TOOL_RPY_DEG, approach_above_m=APPROACH_ABOVE_M,
        dwell_s=DWELL_S, move_vel=MOVE_VEL, line_vel=LINE_VEL,
        undock_if_charging=UNDOCK_IF_CHARGING,
        standoff=True, standoff_target_mm=STANDOFF_TARGET_MM,
        pre_standoff_wait_s=PRE_STANDOFF_WAIT_S, standoff_timeout_s=STANDOFF_TIMEOUT_S,
        capture=True, capture_samples=CAPTURE_SAMPLES, capture_use_led=CAPTURE_USE_LED,
        dry_run=DRY_RUN))
