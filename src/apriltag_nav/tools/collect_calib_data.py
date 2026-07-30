#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
collect_calib_data.py
=====================
Executes joint positions from a CSV, records actual TCP pose from the
Fairino robot and robot base pose from /robot_pose, then saves a
calibration data CSV for use with calibrate_transform.py.

Usage:
    rosrun apriltag_nav collect_calib_data.py \
        --joint_csv task/csv/optimized_joints_line1.csv \
        --pose_csv  task/csv/grid_path_line1.csv \
        --out       task/csv/calib_data_line1.csv \
        --speed     20 \
        --settle    1.5

Output columns:
    group_id, point_id,
    robot_x, robot_y, robot_theta,
    tcp_x_mm, tcp_y_mm, tcp_z_mm, tcp_rx_deg, tcp_ry_deg, tcp_rz_deg,
    world_x, world_y, world_z, world_rx, world_ry, world_rz
"""

import os
import sys
import csv
import time
import argparse
import numpy as np

import rospy
from std_msgs.msg import Float32
from robot_msgs.msg import Pose2DWithFlag

# ---- Fairino SDK ----
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FAIRINO_PATH = os.path.join(BASE_DIR, "../../fairino_sdk/fairino-python-sdk/Linux")
sys.path.append(FAIRINO_PATH)
from fairino import Robot

ROBOT_IP = "192.168.58.2"


# ==========================================================
# Helpers
# ==========================================================

def read_csv(path):
    rows = []
    with open(path, newline='', encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def build_pose_lookup(pose_rows):
    lut = {}
    for r in pose_rows:
        key = (int(r['group_id']), int(r['point_id']))
        lut[key] = {
            'world_x':  float(r['x']),
            'world_y':  float(r['y']),
            'world_z':  float(r['z']),
            'world_rx': float(r['rx']),
            'world_ry': float(r['ry']),
            'world_rz': float(r['rz']),
        }
    return lut


def write_row(writer, row):
    writer.writerow(row)


# ==========================================================
# Main
# ==========================================================

COLS = [
    'group_id', 'point_id',
    'robot_x', 'robot_y', 'robot_theta',
    'tcp_x_mm', 'tcp_y_mm', 'tcp_z_mm',
    'tcp_rx_deg', 'tcp_ry_deg', 'tcp_rz_deg',
    'world_x', 'world_y', 'world_z',
    'world_rx', 'world_ry', 'world_rz',
]

HOME_DEG = [-90.0, -90.0, 90.0, -90.0, -90.0, 0.0]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--joint_csv', required=True, help='Path to optimized_joints_*.csv')
    parser.add_argument('--pose_csv',  required=True, help='Path to grid_path_*.csv (world coords)')
    parser.add_argument('--out',       required=True, help='Output calibration CSV path')
    parser.add_argument('--speed',  type=int,   default=20,  help='MoveJ speed (%)')
    parser.add_argument('--settle', type=float, default=1.5, help='Settle time after move (s)')
    parser.add_argument('--robot_ip', default=ROBOT_IP)
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node('collect_calib_data', anonymous=False)

    # ---- /robot_pose subscriber ----
    current_pose = {'msg': None}

    def pose_cb(msg):
        current_pose['msg'] = msg

    rospy.Subscriber('/robot_pose', Pose2DWithFlag, pose_cb, queue_size=1)
    rospy.loginfo('[Calib] Waiting for /robot_pose ...')
    timeout = rospy.Time.now() + rospy.Duration(10)
    while current_pose['msg'] is None and not rospy.is_shutdown():
        if rospy.Time.now() > timeout:
            rospy.logwarn('[Calib] /robot_pose not received — robot_x/y/theta will be 0')
            break
        rospy.sleep(0.1)

    # ---- Fairino connection ----
    rospy.loginfo(f'[Calib] Connecting to robot {args.robot_ip} ...')
    robot = Robot.RPC(args.robot_ip)
    time.sleep(0.5)
    robot.RobotEnable(1)
    time.sleep(1.0)
    robot.ResetAllError()
    time.sleep(0.3)
    robot.Mode(0)
    time.sleep(0.5)

    # ---- Load data ----
    joint_rows = read_csv(args.joint_csv)
    pose_lut   = build_pose_lookup(read_csv(args.pose_csv))

    joint_rows.sort(key=lambda r: (int(r['group_id']), int(r['point_id'])))

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    out_file = open(args.out, 'w', newline='', encoding='utf-8')
    writer = csv.DictWriter(out_file, fieldnames=COLS)
    writer.writeheader()

    rospy.loginfo(f'[Calib] Moving to home ...')
    robot.SetSpeed(args.speed)
    robot.MoveJ(HOME_DEG, tool=0, user=0)
    time.sleep(2.0)

    collected = 0
    skipped   = 0

    rospy.loginfo(f'[Calib] Starting collection ({len(joint_rows)} points) ...')

    for r in joint_rows:
        if rospy.is_shutdown():
            break

        gid = int(r['group_id'])
        pid = int(r['point_id'])

        if str(r.get('collision_detected', 'FALSE')).strip().upper() == 'TRUE':
            rospy.logwarn(f'  [Skip] ({gid},{pid}) collision_detected=TRUE')
            skipped += 1
            continue

        key = (gid, pid)
        if key not in pose_lut:
            rospy.logwarn(f'  [Skip] ({gid},{pid}) no matching world pose')
            skipped += 1
            continue

        joints_rad = [float(r[f'q{i}']) for i in range(1, 7)]
        joints_deg = [np.degrees(q) for q in joints_rad]

        rospy.loginfo(f'  -> ({gid},{pid})')
        robot.SetSpeed(args.speed)
        ret = robot.MoveJ(joints_deg, tool=0, user=0)
        if ret != 0:
            rospy.logerr(f'  MoveJ failed: {ret} — skip')
            skipped += 1
            continue

        time.sleep(args.settle)

        # ---- Read actual TCP pose ----
        ret_tcp, tcp = robot.GetActualTCPPose()
        if ret_tcp != 0:
            rospy.logerr(f'  GetActualTCPPose failed: {ret_tcp} — skip')
            skipped += 1
            continue
        tx, ty, tz, trx, try_, trz = tcp

        # ---- Read robot base pose ----
        pose_msg = current_pose['msg']
        if pose_msg is not None:
            rx = pose_msg.x
            ry = pose_msg.y
            rth = pose_msg.theta
        else:
            rx, ry, rth = 0.0, 0.0, 0.0

        world = pose_lut[key]
        row = {
            'group_id':    gid,
            'point_id':    pid,
            'robot_x':     round(rx,  6),
            'robot_y':     round(ry,  6),
            'robot_theta': round(rth, 6),
            'tcp_x_mm':    round(tx,  4),
            'tcp_y_mm':    round(ty,  4),
            'tcp_z_mm':    round(tz,  4),
            'tcp_rx_deg':  round(trx, 4),
            'tcp_ry_deg':  round(try_, 4),
            'tcp_rz_deg':  round(trz, 4),
            'world_x':     world['world_x'],
            'world_y':     world['world_y'],
            'world_z':     world['world_z'],
            'world_rx':    world['world_rx'],
            'world_ry':    world['world_ry'],
            'world_rz':    world['world_rz'],
        }
        writer.writerow(row)
        out_file.flush()
        collected += 1

        rospy.loginfo(
            f'    TCP=({tx:.1f},{ty:.1f},{tz:.1f})mm  '
            f'robot=({rx:.3f},{ry:.3f},{rth:.1f}°)'
        )

    # ---- Return home ----
    robot.SetSpeed(20)
    robot.MoveJ(HOME_DEG, tool=0, user=0)

    out_file.close()
    rospy.loginfo(
        f'[Calib] Done. Collected={collected}, Skipped={skipped}. '
        f'Saved → {args.out}'
    )


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
