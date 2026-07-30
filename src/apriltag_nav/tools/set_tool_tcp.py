#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
set_tool_tcp.py
===============
Set Fairino robot TCP (Tool Center Point) to vision_tip position.

Offset derived from fr10v6_visionDF_addtip.urdf:
    flange (tool_Link) → vision      : xyz = 0, 0, 0         (no offset)
    vision             → vision_tip  : xyz = 0, -0.25299, 0.2252  (m)

    Total from flange to vision_tip:
        x =   0.0   mm
        y = -253.0   mm
        z =  225.2   mm
        rx = ry = rz = 0°

Usage:
    python3 set_tool_tcp.py [--tool_id 1] [--robot_ip 192.168.58.2] [--dry_run]

After running:
    Change MoveJ / GetInverseKin calls in arm_controller.py:
        tool=0  →  tool=1   (or whichever tool_id was used)
"""

import sys
import time
import argparse

BASE_DIR = __import__('os').path.dirname(__import__('os').path.abspath(__file__))
sys.path.append(BASE_DIR + '/../../fairino_sdk/fairino-python-sdk/Linux')
from fairino import Robot

# ---------------------------------------------------------------
# TCP offset: flange → vision_tip  (unit: mm, degrees)
# ---------------------------------------------------------------
VISION_TIP_X  =    0.0    # mm
VISION_TIP_Y  = -253.0    # mm   (0 -0.25299 m)
VISION_TIP_Z  =  225.2    # mm   (0.2252 m)
VISION_TIP_RX =    0.0    # deg
VISION_TIP_RY =    0.0    # deg
VISION_TIP_RZ =    0.0    # deg

TCP_COORD = [VISION_TIP_X, VISION_TIP_Y, VISION_TIP_Z,
             VISION_TIP_RX, VISION_TIP_RY, VISION_TIP_RZ]

TOOL_TYPE   = 0   # 0 = tool relative to flange
TOOL_INSTALL = 0  # 0 = normal (upward) installation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--tool_id',  type=int, default=1,
                        help='Tool ID to write (0-14, default 1)')
    parser.add_argument('--robot_ip', default='192.168.58.2')
    parser.add_argument('--dry_run',  action='store_true',
                        help='Print values only, do not write to robot')
    args = parser.parse_args()

    print('='*55)
    print('vision_tip TCP offset (from flange):')
    print(f'  x  = {VISION_TIP_X:>8.2f} mm')
    print(f'  y  = {VISION_TIP_Y:>8.2f} mm')
    print(f'  z  = {VISION_TIP_Z:>8.2f} mm')
    print(f'  rx = {VISION_TIP_RX:>8.2f} deg')
    print(f'  ry = {VISION_TIP_RY:>8.2f} deg')
    print(f'  rz = {VISION_TIP_RZ:>8.2f} deg')
    print(f'  → Tool ID: {args.tool_id}')
    print('='*55)

    if args.dry_run:
        print('[DRY RUN] No changes written to robot.')
        return

    print(f'Connecting to robot {args.robot_ip} ...')
    robot = Robot.RPC(args.robot_ip)
    time.sleep(0.5)

    # ---- read current TCP offset (tool=0 baseline) ----
    ret, current = robot.GetTCPOffset()
    if ret == 0:
        print(f'Current TCP offset (flag=1): {[round(v,3) for v in current]}')
    else:
        print(f'GetTCPOffset failed: {ret}')

    # ---- set tool list (save to robot memory) ----
    ret = robot.SetToolList(args.tool_id, TCP_COORD, TOOL_TYPE, TOOL_INSTALL)
    if ret == 0:
        print(f'SetToolList(tool_id={args.tool_id}) → OK')
    else:
        print(f'SetToolList failed: {ret}')
        return

    time.sleep(0.3)

    # ---- activate tool (apply immediately) ----
    ret = robot.SetToolCoord(args.tool_id, TCP_COORD, TOOL_TYPE, TOOL_INSTALL)
    if ret == 0:
        print(f'SetToolCoord(tool_id={args.tool_id}) → OK (activated)')
    else:
        print(f'SetToolCoord failed: {ret}')
        return

    time.sleep(0.3)

    # ---- verify ----
    ret, result = robot.GetTCPOffset()
    if ret == 0:
        print(f'Verified TCP offset: {[round(v, 3) for v in result]}')
    else:
        print(f'Verification GetTCPOffset failed: {ret}')

    print()
    print('Done. Now update arm_controller.py:')
    print(f'  MoveJ(joints_deg, tool={args.tool_id}, user=0)')
    print(f'  GetInverseKin(0, target, config=-1)   → tool is implicit')
    print(f'  GetInverseKinRef(0, target, q0_deg)   → tool is implicit')


if __name__ == '__main__':
    main()
