#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Example operator script: capture a grid of points around the current pose.

Reloaded from disk every run, so edit and press RUN again — no restart.

Shows the three rules every plugin follows:
  1. Reach hardware only through ctx.bridge. There is no device handle here,
     and there must not be: arm_node owns the arm and basler_camera_node owns
     the camera, and a second commander of either is the bug this whole
     package was restructured to remove.
  2. Check ctx.cancelled() inside every loop. Stop cannot interrupt a thread
     that never looks.
  3. Check the (ok, message) every motion returns. A jog refused because the
     arm is busy comes back False; carrying on regardless would capture a grid
     of images that are all the same point.

Run it with the arm already roughly in place — it moves RELATIVE to wherever
the tool currently is, and does not home first.
"""

import os
from datetime import datetime

# Grid geometry, in tool-frame mm. Small on purpose: this is an example, and a
# plugin that sweeps far by default is one careless RUN away from a collision.
STEP_MM = 5.0
COLS = 3
ROWS = 2

SAVE_DIR = '/tmp/robot_ui_captures/grid'


def run(ctx):
    ctx.log(f'grid capture: {COLS}x{ROWS} at {STEP_MM} mm spacing')

    start = ctx.bridge.arm_snapshot()
    if start is None or not start['pose_valid']:
        ctx.log('no valid arm pose yet — is arm_node running?')
        return
    ctx.log(f'starting from {[round(v, 2) for v in start["tcp_pose"]]}')

    os.makedirs(SAVE_DIR, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    results = []

    # Serpentine order: reverse alternate rows so the tool never makes a long
    # traverse back to the start of the next one.
    for row in range(ROWS):
        if ctx.cancelled():
            break
        columns = range(COLS) if row % 2 == 0 else range(COLS - 1, -1, -1)
        for col in columns:
            if ctx.cancelled():
                break

            # Absolute targets computed from the pose recorded before the run,
            # not accumulated jogs. Accumulating drifts: any step that gets
            # clamped or refused silently shifts every later point.
            target = list(start['tcp_pose'])
            target[0] += col * STEP_MM
            target[1] += row * STEP_MM

            point = f'r{row}c{col}'
            ok, message = ctx.bridge.arm_move_cart(target, vel=20.0)
            if not ok:
                ctx.log(f'{point}: move failed ({message}) — stopping')
                return
            if not ctx.sleep(0.4):        # let the arm settle before the shot
                break

            ok, message, frames = ctx.bridge.capture(
                num_samples=1, use_vision_led=True)
            if not ok or not frames:
                ctx.log(f'{point}: capture failed ({message})')
                continue

            path = os.path.join(SAVE_DIR, f'{stamp}_{point}.png')
            try:
                import cv2
                cv2.imwrite(path, frames[-1])
            except Exception as e:
                ctx.log(f'{point}: save failed ({e})')

            ra = ctx.bridge.predict_ra(frames[-1], tag=point)
            if ra['success']:
                ctx.log(f'{point}: Ra = {ra["ra"]:.4f}  ({ra["elapsed_s"]:.2f}s)')
                results.append((point, ra['ra']))
            else:
                ctx.log(f'{point}: inference failed ({ra["message"]})')

    ctx.log(f'grid capture done — {len(results)} point(s) scored')
    if results:
        values = [v for _, v in results]
        ctx.log(f'Ra min={min(values):.4f} max={max(values):.4f} '
                f'mean={sum(values) / len(values):.4f}')
    # Deliberately does NOT return the arm anywhere. Same rule as tasks: a
    # script that appends its own motion is not composable with the next one.
