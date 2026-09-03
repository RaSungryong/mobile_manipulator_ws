#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Operator script: SURVEY where the cross tags physically are.

Use when calibration reports "tag not detected" en masse (정반 2 case):
park the base at a work-tag stop of the failing corridor, move the arm
to that entry's plan seed pose (or anywhere near where a cross tag
SHOULD be), then RUN. The arm sweeps a line of world-x offsets around
the current pose, reporting which tag ids the hand camera sees at each
offset. The offset where a tag appears IS the config-vs-physical
placement error.

Interpretation for 정반 2:
  found near offset 0        -> tags ARE at +3.890 (geometric centre);
                                look for another cause
  found near offset -0.47 m  -> tags were laid by the "2번정반 중심"
                                CSV origin (+3.420): fix
                                reference_tags_plate2.yaml (-0.470 in x)
                                and re-run the generator
  found nowhere              -> tags likely not installed / covered

Edit the constants and RUN again — the file reloads every run.
"""

# Sweep offsets along the arm-base AXIS closest to world x, in mm.
# The seed poses face the plate, so arm-base y maps to world x for
# B/D-zone stops and -x for C/E — the report prints raw offsets; read
# the sign with the zone in mind.
SWEEP_AXIS = 'y'
SWEEP_MM = [0, -150, 150, -300, 300, -470, 470, -600, 600]
SETTLE_S = 0.8


def run(ctx):
    start = ctx.bridge.arm_snapshot()
    if start is None or not start['pose_valid']:
        ctx.log('no valid arm pose — is arm_node running?')
        return
    start_pose = list(start['tcp_pose'])
    ctx.log('sweeping %s by %s mm from %s'
            % (SWEEP_AXIS, SWEEP_MM,
               ['%.0f' % v for v in start_pose[:3]]))

    axis_i = {'x': 0, 'y': 1, 'z': 2}[SWEEP_AXIS]
    found = {}
    for off in SWEEP_MM:
        if ctx.cancelled():
            break
        target = list(start_pose)
        target[axis_i] += off
        ok, msg = ctx.bridge.arm_move_cart(target, vel=30.0)
        if not ok:
            ctx.log('offset %+d mm: move failed (%s) — skipping'
                    % (off, msg))
            continue
        if not ctx.sleep(SETTLE_S):
            break
        dets = ctx.bridge.latest_detections('hand_cam')
        if dets:
            for d in dets:
                ctx.log('offset %+d mm: SEE tag %d  (cam frame '
                        'x=%+.3f y=%+.3f z=%.3f, yaw %+.1f)'
                        % (off, d['id'], d['pose_x'], d['pose_y'],
                           d['pose_z'], d['yaw']))
                found.setdefault(d['id'], off)
        else:
            ctx.log('offset %+d mm: nothing' % off)

    # home is safer than returning to an arbitrary sweep endpoint
    ctx.bridge.arm_home()
    if found:
        ctx.log('SUMMARY: first sighting per tag (offset mm): %s'
                % found)
        ctx.log('config-vs-physical error ~= -(offset where the tag '
                'is CENTERED); ~470 means the CSV-origin layout.')
    else:
        ctx.log('SUMMARY: no cross tag anywhere in the sweep — check '
                'installation/lighting, or widen SWEEP_MM.')
