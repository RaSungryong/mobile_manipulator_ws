#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Operator script: locate ONE path tag in the world frame.

Needs `roslaunch path_tag_locator path_tag_locator.launch` running
alongside the main stack. Park the base where front_cam sees the target
tag and hand_cam can see the reference tag, then RUN.

Edit the constants and press RUN again — the file reloads every run.
"""

# -1 = locator.yaml's default tag_b_id. Set to the tag you parked at.
TAG_B_ID = -1

# True: MoveJ to INITIAL_TCP first, then iteratively centre hand_cam on
# the reference tag before computing. False: compute from wherever the
# arm currently is (arm must already see the ref tag).
AUTO_ALIGN = False
INITIAL_TCP = [350.0, 0.0, 250.0, 180.0, 0.0, 0.0]   # mm / deg ZYX


def run(ctx):
    ctx.log(f'locate tag {TAG_B_ID if TAG_B_ID >= 0 else "(default)"} '
            f'auto_align={AUTO_ALIGN}')
    if AUTO_ALIGN:
        ctx.log(f'initial view pose: {INITIAL_TCP}')

    result = ctx.bridge.locate_path_tag(
        tag_b_id=TAG_B_ID,
        auto_align=AUTO_ALIGN,
        initial_tcp=INITIAL_TCP if AUTO_ALIGN else None,
    )

    if not result['success']:
        ctx.log(f"locate FAILED: {result['message']}")
        return
    x, y, z = result['position_m']
    ctx.log(f'world position: x={x:.4f} y={y:.4f} z={z:.4f} m')
    ctx.log(f"rpy: {['%.2f' % v for v in result['rpy_deg']]} deg")
    if result.get('align_iterations'):
        ctx.log(f"auto-align converged in {result['align_iterations']} "
                f'iteration(s)')
    ctx.log('full record saved under ~/.ros/path_tag_locator/locate/')
