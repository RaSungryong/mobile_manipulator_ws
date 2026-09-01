#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_transform.py
======================
Optimizes the 6 transform parameters in _transform_pose using
calibration data collected by collect_calib_data.py.

Usage:
    python3 calibrate_transform.py --calib_csv task/csv/calib_data_line1.csv

Input CSV columns (from collect_calib_data.py):
    group_id, point_id,
    robot_x, robot_y, robot_theta,          ← robot base pose (msg.x, msg.y, theta_deg)
    tcp_x_mm, tcp_y_mm, tcp_z_mm,           ← measured TCP in arm base frame (mm)
    tcp_rx_deg, tcp_ry_deg, tcp_rz_deg,     ← measured TCP orientation (deg)
    world_x, world_y, world_z               ← world target position (m)

Output:
    Optimized parameters printed to stdout + saved to {calib_csv}_params.yaml
"""

import csv
import argparse
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize, differential_evolution


# ==========================================================
# Transform (mirrors _transform_pose in arm_controller.py)
# ==========================================================

def transform_world_to_arm_mm(world_xyz_m, robot_msg_x, robot_msg_y,
                               robot_theta_deg, params):
    """
    World (m) → arm base_link (mm), same math as _transform_pose in v2.

    params: [body_off_x, body_off_y, arm_base_z, mount_yaw, tilt_x, tilt_y]
            (meters / radians)

    robot_msg_x, robot_msg_y: Pose2DWithFlag.x, .y  (world frame, meters)
    robot_theta_deg: Pose2DWithFlag.theta            (degrees)
    """
    body_off_x, body_off_y, arm_base_z, mount_yaw, tilt_x, tilt_y = params

    theta = np.radians(robot_theta_deg)
    x_base = -robot_msg_y
    y_base = -robot_msg_x

    c, s = np.cos(theta), np.sin(theta)
    p_A_W = np.array([
        x_base + c * body_off_x - s * body_off_y,
        y_base + s * body_off_x + c * body_off_y,
        arm_base_z
    ])

    alpha = theta + mount_yaw
    R_WA = (R.from_euler('z', alpha)
            * R.from_euler('y', tilt_y)
            * R.from_euler('x', tilt_x)).as_matrix()
    R_AW = R_WA.T

    p_W   = np.array(world_xyz_m)
    p_arm = R_AW @ (p_W - p_A_W)
    pos_mm = p_arm * 1000.0

    return pos_mm


# ==========================================================
# Data loading
# ==========================================================

def load_calib_data(csv_path):
    rows = []
    with open(csv_path, newline='', encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            rows.append({
                'gid':          int(r['group_id']),
                'pid':          int(r['point_id']),
                'robot_x':      float(r['robot_x']),
                'robot_y':      float(r['robot_y']),
                'robot_theta':  float(r['robot_theta']),
                'tcp_mm':       np.array([float(r['tcp_x_mm']),
                                          float(r['tcp_y_mm']),
                                          float(r['tcp_z_mm'])]),
                'world_xyz':    np.array([float(r['world_x']),
                                          float(r['world_y']),
                                          float(r['world_z'])]),
            })
    return rows


# ==========================================================
# Objective
# ==========================================================

def objective(params, data):
    """Mean squared position residual (mm²)."""
    total = 0.0
    for d in data:
        pred = transform_world_to_arm_mm(
            d['world_xyz'],
            d['robot_x'], d['robot_y'], d['robot_theta'],
            params
        )
        diff = d['tcp_mm'] - pred
        total += float(np.dot(diff, diff))
    return total / len(data)


# ==========================================================
# Per-point residuals
# ==========================================================

def compute_residuals(params, data):
    residuals = []
    for d in data:
        pred = transform_world_to_arm_mm(
            d['world_xyz'],
            d['robot_x'], d['robot_y'], d['robot_theta'],
            params
        )
        err_mm = float(np.linalg.norm(d['tcp_mm'] - pred))
        residuals.append((d['gid'], d['pid'], err_mm, d['tcp_mm'], pred))
    return residuals


# ==========================================================
# Main
# ==========================================================

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calib_csv', required=False, default='/home/ku/mobile_manipulator_ws/src/apriltag_nav/task/csv/calib_data.csv',
                        help='Calibration data CSV from collect_calib_data.py')
    parser.add_argument('--method', default='both',
                        choices=['nelder', 'de', 'both'],
                        help='Optimizer: nelder-mead, differential_evolution, or both')
    parser.add_argument('--no_tilt', action='store_true',
                        help='Fix tilt_x=tilt_y=0 (optimize only 4 params)')
    args = parser.parse_args()

    data = load_calib_data(args.calib_csv)
    print(f'Loaded {len(data)} calibration points')

    # ---- Initial guess (current defaults from CLAUDE.md) ----
    # [body_off_x, body_off_y, arm_base_z, mount_yaw, tilt_x, tilt_y]
    x0 = np.array([0.0, 0.0, 1.0076, np.pi, -0.02248, 0.02639])

    # ---- Initial residual ----
    r0 = compute_residuals(x0, data)
    errs0 = [r[2] for r in r0]
    print(f'\nInitial residuals (mm):  mean={np.mean(errs0):.2f}  '
          f'max={np.max(errs0):.2f}  std={np.std(errs0):.2f}')

    best_params = x0.copy()
    best_val    = objective(x0, data)

    # ---- Bounds ----
    bounds = [
        (-0.10,  0.10),   # body_off_x  (m)
        (-0.10,  0.10),   # body_off_y  (m)
        # Spans the replacement base (0.652 at the lift origin, 0.995 at the
        # top of the 343 mm stroke) as well as the retired one (1.025). The old
        # 0.80 floor predates the base swap and would have clamped a new-base
        # fit to a wrong answer instead of failing visibly.
        ( 0.55,  1.20),   # arm_base_z  (m)
        ( 2.80,  3.48),   # mount_yaw   (rad, ≈π ± 20°)
        (-0.15,  0.15),   # tilt_x      (rad, ≈±8.6°)
        (-0.15,  0.15),   # tilt_y      (rad, ≈±8.6°)
    ]

    if args.no_tilt:
        # Fix tilt to zero — only optimize 4 params
        def obj4(p4):
            return objective([p4[0], p4[1], p4[2], p4[3], 0.0, 0.0], data)
        bounds4 = bounds[:4]
    else:
        obj4    = None
        bounds4 = None

    # ---- Stage 1: Differential Evolution (global) ----
    if args.method in ('de', 'both'):
        print('\n[Stage 1] Differential Evolution (global) ...')
        obj_fn = obj4 if args.no_tilt else lambda p: objective(p, data)
        b      = bounds4 if args.no_tilt else bounds
        de_res = differential_evolution(
            obj_fn, b,
            seed=42, maxiter=2000, tol=1e-8,
            workers=1, polish=True
        )
        p_de = de_res.x if not args.no_tilt else [*de_res.x, 0.0, 0.0]
        print(f'  DE result: obj={de_res.fun:.4f}  converged={de_res.success}')
        if de_res.fun < best_val:
            best_val    = de_res.fun
            best_params = np.array(p_de)

    # ---- Stage 2: Nelder-Mead (local polish) ----
    if args.method in ('nelder', 'both'):
        print('\n[Stage 2] Nelder-Mead (local polish) ...')
        x_init = best_params if args.method == 'both' else x0
        if args.no_tilt:
            x_init4 = x_init[:4]
            nm_res  = minimize(obj4, x_init4, method='Nelder-Mead',
                               options={'maxiter': 100000, 'xatol': 1e-7, 'fatol': 1e-7})
            p_nm = [*nm_res.x, 0.0, 0.0]
        else:
            nm_res = minimize(lambda p: objective(p, data), x_init,
                              method='Nelder-Mead',
                              options={'maxiter': 100000, 'xatol': 1e-7, 'fatol': 1e-7})
            p_nm = nm_res.x
        print(f'  NM result: obj={nm_res.fun:.4f}  converged={nm_res.success}')
        if nm_res.fun < best_val:
            best_val    = nm_res.fun
            best_params = np.array(p_nm)

    # ---- Final residuals ----
    residuals = compute_residuals(best_params, data)
    errs = np.array([r[2] for r in residuals])

    print(f'\n{"="*60}')
    print(f'OPTIMIZED PARAMETERS')
    print(f'{"="*60}')
    print(f'  arm_body_offset_x: {best_params[0]:.6f}  m')
    print(f'  arm_body_offset_y: {best_params[1]:.6f}  m')
    print(f'  arm_base_z:        {best_params[2]:.6f}  m')
    print(f'  arm_mount_yaw:     {best_params[3]:.6f}  rad  '
          f'({np.degrees(best_params[3]):.3f}°)')
    print(f'  arm_tilt_x:        {best_params[4]:.6f}  rad  '
          f'({np.degrees(best_params[4]):.4f}°)')
    print(f'  arm_tilt_y:        {best_params[5]:.6f}  rad  '
          f'({np.degrees(best_params[5]):.4f}°)')
    print(f'\nResiduals (mm):  mean={errs.mean():.2f}  '
          f'max={errs.max():.2f}  std={errs.std():.2f}')
    print(f'Improvement: {np.mean(errs0):.2f} → {errs.mean():.2f} mm')

    # ---- Per-point worst residuals ----
    residuals_sorted = sorted(residuals, key=lambda x: -x[2])
    print(f'\nTop 5 worst residuals:')
    for gid, pid, err, tcp, pred in residuals_sorted[:5]:
        print(f'  ({gid},{pid}): {err:.2f} mm  '
              f'tcp=({tcp[0]:.1f},{tcp[1]:.1f},{tcp[2]:.1f})  '
              f'pred=({pred[0]:.1f},{pred[1]:.1f},{pred[2]:.1f})')

    # ---- Save params YAML ----
    out_yaml = args.calib_csv.replace('.csv', '_params.yaml')
    param_dict = {
        'arm_body_offset_x': float(best_params[0]),
        'arm_body_offset_y': float(best_params[1]),
        'arm_base_z':        float(best_params[2]),
        'arm_mount_yaw':     float(best_params[3]),
        'arm_tilt_x':        float(best_params[4]),
        'arm_tilt_y':        float(best_params[5]),
        '_residual_mean_mm': float(errs.mean()),
        '_residual_max_mm':  float(errs.max()),
        '_n_points':         len(data),
    }
    with open(out_yaml, 'w') as f:
        yaml.dump(param_dict, f, default_flow_style=False)
    print(f'\nParams saved → {out_yaml}')
    print(f'\nTo apply, update config/robot.yaml or pass as ROS params:')
    print(f'  _arm_body_offset_x:={best_params[0]:.6f}')
    print(f'  _arm_body_offset_y:={best_params[1]:.6f}')
    print(f'  _arm_base_z:={best_params[2]:.6f}')
    print(f'  _arm_mount_yaw:={best_params[3]:.6f}')
    print(f'  _arm_tilt_x:={best_params[4]:.6f}')
    print(f'  _arm_tilt_y:={best_params[5]:.6f}')


if __name__ == '__main__':
    main()
