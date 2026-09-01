#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Operator script: run a tag-map calibration session through map_calibrator.

Prerequisites (this script checks none of them for you):
  1. `roslaunch path_tag_locator path_tag_locator.launch` is running
     alongside the main stack (the calibration nodes are NOT part of
     mobile_manipulator.launch).
  2. reference_tags.yaml holds the six REAL measured tag poses (not the
     example values) and calibration_plan.yaml lists the entries to run.
  3. Nobody sends TASK/GOTO while this runs — the session drives the base
     through /mobile/goto_tag, the same topic task_executor commands.

Edit DRY_RUN below and press RUN again — the file reloads every run.

⚠️ Stop button caveat: run_map_calibration is ONE blocking service call;
ctx.cancelled() cannot interrupt it mid-session. To abort a session
cleanly call the calibrator's cancel:
    rosservice call /map_calibrator/cancel_calibration
(or ctx.bridge.cancel_map_calibration() from another script) — the
session stops after the current entry, keeping partial output. STOP ALL
remains the immediate motion stop; note an EMERGENCY stop latch now
makes every following nav entry refuse to drive until cleared
deliberately.
"""

# True: parse plan + ref tags + map without moving anything. ALWAYS run a
# dry pass first on a new plan.
DRY_RUN = True

# 1 = 정반 1 session (zones B+C, cross tags on plate 1)
# 2 = 정반 2 session (zones D+E, cross tags on plate 2)
# Both plates carry cross tags with the SAME ids 0-5, so the plan and the
# reference file are switched TOGETHER — never mix them.
PLATE = 1

_FILES = {
    1: ('$(find path_tag_locator)/config/calibration_plan_plate1.yaml',
        '$(find path_tag_locator)/config/reference_tags.yaml'),
    2: ('$(find path_tag_locator)/config/calibration_plan_plate2.yaml',
        '$(find path_tag_locator)/config/reference_tags_plate2.yaml'),
}


def run(ctx):
    plan_path, ref_path = _FILES[PLATE]
    mode = 'DRY RUN' if DRY_RUN else 'LIVE'
    ctx.log(f'map calibration 정반 {PLATE} ({mode}) — per-tag progress '
            f'appears as [calib] lines')
    ctx.log(f'plan: {plan_path.split("/")[-1]}  '
            f'refs: {ref_path.split("/")[-1]}')
    if not DRY_RUN:
        ctx.log('LIVE session: base will drive, arm will move, lift may '
                'reposition (map_calibrator.yaml lift_height_mm). '
                'No TASK/GOTO until it finishes.')

    ok, message, report = ctx.bridge.run_map_calibration(
        dry_run=DRY_RUN, plan_path=plan_path, ref_tags_path=ref_path)

    ctx.log(f'session finished: ok={ok} — {message}')
    if report:
        ctx.log(f"succeeded={report['num_succeeded']} "
                f"failed={report['num_failed']}")
        if report.get('output_yaml_path'):
            ctx.log(f"world map written to {report['output_yaml_path']}")
            ctx.log('verify with: rosrun path_tag_locator '
                    'verify_map_world.py')
