#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tip-over-reference-tag sequences (robot_ui operator scripts, 2026-09-22).

The two plugins `tip_over_ref_tag.py` (hover 30 mm above cross tag 0) and
`tip_touch_ref_tag.py` (tip on the tag, Keyence standoff, LED capture)
share this module: per round, per base stop tag

    GOTO <tag>              task_executor homes the arm, drives, aligns
    /robot_pose             the stop pose mobile_node computed at rest
    world -> arm frame      transform_world_to_arm (pose-mode scan maths,
                            live lift); tip -> flange with T_ee2tip when
                            the controller's active tool is the flange
                            (read off /arm/state at the home joints)
    MoveCart to APPROACH above, MoveL straight down
    [record "before": TCP, joints, tip in world]
    [wait, Keyence standoff loop (/arm/standoff), record "_after_correction"]
    [LED capture through /camera/capture, PNG saved]
    dwell, MoveL straight up
The next GOTO's arm home takes the arm back; after the last point the
sequence homes it itself. UNDOCK first when the BMS shows current.

World frame: map.yaml (origin = centre of 정반 1, +x east, +y north), z
from the PLATE TOP — the frame reference_tags.yaml uses (cross tag 0 =
(-0.600, -1.200, 0.001)). ⚠️ transform_world_to_arm's world z is the FLOOR
datum (arm_base_z is measured from the floor), 0.080 m lower, so
PLATE_TOP_ABOVE_FLOOR_M is added before the transform; feeding a plate-top
z in directly puts the tip 80 mm too LOW.

Records: log/apriltag_nav/tip_check/<ts>_<name>/summary.csv + one yaml per
point (commanded target, robot pose, targets in the arm frame, the before /
after_correction TCP + joints + tip world position, standoff result, image
file); images in results/tip_check/<ts>_<name>/.
"""

import csv
import math
import os
import time

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

from apriltag_nav import paths, tf_chain
from apriltag_nav.arm_transform import transform_world_to_arm

PLATE_TOP_ABOVE_FLOOR_M = 0.080     # world z=0 (plate top) is 80 mm above the floor

# What the controller reports at the home joints: the URDF FLANGE
# (2026-09-14; tool 1 is not active). Read live from /arm/state at home to
# decide whether a tip target must be converted to a flange target.
HOME_FLANGE_MM = (-159.0, 700.0, 774.0)
TOOL_FRAME_TOL_MM = 15.0
HOME_JOINT_TOL_DEG = 3.0

# Safety bounds on the PHYSICAL flange (arm frame): FR10 reach, and the
# empirical body-clearance line from verify_chain (2026-09-21 collision):
# a flange below the arm-base plane needs >= 0.65 m of horizontal reach.
MAX_FLANGE_REACH_M = 1.25
MIN_REACH_BELOW_BASE_M = 0.65
MIN_FLANGE_Z_M = -0.50

GOTO_TIMEOUT_S = 600.0
GOTO_ACK_S = 20.0
POSE_MAX_AGE_S = 300.0          # /robot_pose must post-date the GOTO command

RECORD_ROOT = os.path.join(paths.LOG_DIR, 'apriltag_nav', 'tip_check')
IMAGE_ROOT = os.path.join(paths.RESULTS_DIR, 'tip_check')


class Settings:
    """One sequence's parameters (a plugin fills these from its constants)."""

    def __init__(self, name, target_world_m, stop_tags=(102, 103, 104), rounds=3,
                 tool_rpy_deg=(-180.0, 0.0, 0.0), approach_above_m=0.20,
                 dwell_s=3.0, move_vel=30.0, line_vel=20.0,
                 undock_if_charging=True,
                 standoff=False, standoff_target_mm=None, pre_standoff_wait_s=1.0,
                 standoff_timeout_s=120.0,
                 capture=False, capture_samples=1, capture_use_led=True,
                 dry_run=False):
        self.name = str(name)
        self.target_world_m = tuple(float(v) for v in target_world_m)
        self.stop_tags = [int(t) for t in stop_tags]
        self.rounds = int(rounds)
        self.tool_rpy_deg = tuple(float(v) for v in tool_rpy_deg)
        self.approach_above_m = float(approach_above_m)
        self.dwell_s = float(dwell_s)
        self.move_vel = float(move_vel)
        self.line_vel = float(line_vel)
        self.undock_if_charging = bool(undock_if_charging)
        self.standoff = bool(standoff)
        self.standoff_target_mm = None if standoff_target_mm is None else float(standoff_target_mm)
        self.pre_standoff_wait_s = float(pre_standoff_wait_s)
        self.standoff_timeout_s = float(standoff_timeout_s)
        self.capture = bool(capture)
        self.capture_samples = int(capture_samples)
        self.capture_use_led = bool(capture_use_led)
        self.dry_run = bool(dry_run)


class _Pose:
    """Duck-typed /robot_pose for transform_world_to_arm (x, y, theta)."""

    def __init__(self, x, y, theta):
        self.x, self.y, self.theta = float(x), float(y), float(theta)


# ---------------------------------------------------------------- geometry
def _rot(rpy_deg):
    r = R.from_euler('xyz', rpy_deg, degrees=True)
    return r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()


def world_to_arm_R(pose, lift_m):
    """Arm-frame axes of the world x / y / z unit vectors, from the same
    transform the targets go through (so tip positions reported back in
    world axes use exactly the rotation the target used). R_AW: world -> arm."""
    g0 = dict(x=0.0, y=0.0, z=0.0, rx=0, ry=0, rz=0)
    p0, _ = transform_world_to_arm(g0, pose, lift_m)
    cols = []
    for axis in ('x', 'y', 'z'):
        g = dict(g0)
        g[axis] = 1.0
        p, _ = transform_world_to_arm(g, pose, lift_m)
        cols.append((np.asarray(p) - np.asarray(p0)) / 1000.0)
    return np.stack(cols, axis=1)


def compute_targets(s, pose, lift_m, tool_is_flange, tip_offset_mm):
    """Arm-frame tip target, the PHYSICAL flange position for it, the pose
    to COMMAND (flange or tip, whichever frame the controller is in) and
    the approach pose above it (mm / deg)."""
    g = dict(x=s.target_world_m[0], y=s.target_world_m[1],
             z=s.target_world_m[2] + PLATE_TOP_ABOVE_FLOOR_M,
             rx=0.0, ry=0.0, rz=0.0)
    tip_arm, _ = transform_world_to_arm(g, pose, lift_m)
    tip_arm = np.asarray(tip_arm, dtype=float)
    rpy = np.asarray(s.tool_rpy_deg, dtype=float)
    flange = tip_arm - _rot(rpy) @ np.asarray(tip_offset_mm, dtype=float)
    cmd = flange if tool_is_flange else tip_arm
    approach = cmd + np.array([0.0, 0.0, s.approach_above_m * 1000.0])
    return tip_arm, flange, cmd, approach, rpy


def check_bounds(flange_mm):
    reach = math.hypot(flange_mm[0], flange_mm[1]) / 1000.0
    z = flange_mm[2] / 1000.0
    if reach > MAX_FLANGE_REACH_M:
        return False, f'flange {reach:.3f} m from the base > {MAX_FLANGE_REACH_M} m'
    if z < MIN_FLANGE_Z_M:
        return False, f'flange z {z:.3f} m below {MIN_FLANGE_Z_M} m'
    if z < 0.0 and reach < MIN_REACH_BELOW_BASE_M:
        return False, (f'flange below the base plane at {reach:.3f} m reach '
                       f'< {MIN_REACH_BELOW_BASE_M} m (body clearance rule)')
    return True, f'flange reach {reach:.3f} m, z {z:.3f} m'


def tip_from_tcp(tcp_pose, tool_is_flange, tip_offset_mm):
    """Arm-frame tip position (mm) from a reported TCP pose."""
    p = np.asarray(tcp_pose[:3], dtype=float)
    if tool_is_flange:
        return p + _rot(tcp_pose[3:6]) @ np.asarray(tip_offset_mm, dtype=float)
    return p


def tip_world_m(s, tip_arm_mm, tip_target_arm_mm, R_AW):
    """Tip position in the world frame (plate-top z), from its arm-frame
    position: the transform is affine, so target_W + R_AW^T (p - p_target)."""
    d = (np.asarray(tip_arm_mm) - np.asarray(tip_target_arm_mm)) / 1000.0
    return np.asarray(s.target_world_m) + R_AW.T @ d


def design_stop_pose(tag):
    """Zone-B design stop for a dry run: 0.55 m south of the tag, heading +90."""
    with open(paths.MAP_PATH) as f:
        m = yaml.safe_load(f)
    tags = m.get('tags', m)
    t = tags[int(tag)]
    sx, sy = float(t['x']), float(t['y']) - 0.55
    return _Pose(-sy, -sx, 90.0)


# ---------------------------------------------------------------- ROS-side helpers
def _task_state(bridge):
    return (bridge.cached_states() or {}).get('task_state')


def _wait_task(ctx, name, timeout_s, ack_s=GOTO_ACK_S):
    """Wait for task_executor to pick `name` up and return to IDLE.
    Returns (ok, message); ok=False on cancel / timeout / ERROR seen."""
    t0 = time.time()
    seen = False
    while time.time() - t0 < ack_s:
        ts = _task_state(ctx.bridge)
        if ts and ts.get('task') == name and ts.get('stamp', 0) >= t0 - 1.0:
            seen = True
            break
        if not ctx.sleep(0.2):
            return False, 'cancelled'
    if not seen:
        return False, f'task_executor did not start {name} within {ack_s:.0f}s'
    error_seen = False
    while time.time() - t0 < timeout_s:
        ts = _task_state(ctx.bridge)
        if ts and ts.get('state') == 'ERROR':
            error_seen = True
        if ts and ts.get('state') == 'IDLE' and ts.get('task') is None \
                and ts.get('stamp', 0) >= t0:
            note = ts.get('note') or ''
            if error_seen:
                return False, f'{name} ended after an ERROR state ({note})'
            return True, note
        if not ctx.sleep(0.2):
            return False, 'cancelled'
    return False, f'{name} still running after {timeout_s:.0f}s'


def _goto(ctx, tag):
    """GOTO <tag> through /task_command; verified on mobile_node's result."""
    bridge = ctx.bridge
    name = f'goto_{tag}'
    t_cmd = time.time()
    bridge.send_task_command(f'GOTO {tag}')
    ok, msg = _wait_task(ctx, name, GOTO_TIMEOUT_S)
    if not ok:
        return False, msg, t_cmd
    st = bridge.mobile_snapshot() or {}
    res = st.get('result') or {}
    if not (res.get('ok') and int(res.get('tag') or -1) == int(tag)):
        return False, (f'base result does not confirm tag {tag}: '
                       f'{res.get("message", res)}'), t_cmd
    if st.get('last_known_tag') != tag:
        return False, f'mobile_node last_known_tag is {st.get("last_known_tag")}, not {tag}', t_cmd
    return True, res.get('message', ''), t_cmd


def _robot_pose(bridge, tag, not_before):
    p = bridge.robot_pose_snapshot()
    if p is None:
        return None, 'no /robot_pose received since the UI started'
    if int(p['id']) != int(tag):
        return None, f'/robot_pose is for tag {p["id"]}, not {tag}'
    if p['stamp'] < not_before:
        return None, (f'/robot_pose for tag {tag} predates this GOTO '
                      f'({not_before - p["stamp"]:.0f}s older)')
    if time.time() - p['stamp'] > POSE_MAX_AGE_S:
        return None, f'/robot_pose is {time.time() - p["stamp"]:.0f}s old'
    return _Pose(p['x'], p['y'], p['theta']), 'ok'


def _lift_m(bridge):
    ls = (bridge.cached_states() or {}).get('lift_state') or {}
    h = ls.get('height_mm')
    if h is None:
        return None, 'lift height unknown (/lifter/state has no height_mm — lifter_node down or unhomed?)'
    return float(h) / 1000.0, f'lift {float(h):.1f} mm'


def _arm_pose(bridge):
    st = bridge.arm_snapshot()
    if not st or not st.get('pose_valid'):
        return None, None
    return (np.asarray(st['tcp_pose'][:6], dtype=float),
            np.asarray(st['joints'][:6], dtype=float))


def _standoff_state(bridge):
    st = (bridge.cached_states() or {}).get('standoff_state')
    return dict(st) if isinstance(st, dict) else None


def _tool_frame_at_home(bridge, tip_offset_mm):
    """'flange' / 'tip' from the pose the controller reports at the home
    joints; (None, reason) if the arm is not at home or reports neither."""
    pose, joints = _arm_pose(bridge)
    if pose is None:
        return None, '/arm/state has no valid pose'
    home = paths.load_yaml_block('arm_home').get('joints_rad') \
        or [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
    home_deg = np.degrees(np.asarray(home, dtype=float))
    dj = np.abs(joints - home_deg)
    if np.max(dj) > HOME_JOINT_TOL_DEG:
        return None, f'arm is not at the home joints (max {np.max(dj):.1f} deg off)'
    p = pose[:3]
    if np.linalg.norm(p - np.asarray(HOME_FLANGE_MM)) < TOOL_FRAME_TOL_MM:
        return 'flange', f'controller reports the FLANGE at home {p.round(1).tolist()} mm'
    tip_home = np.asarray(HOME_FLANGE_MM) + _rot(pose[3:6]) @ np.asarray(tip_offset_mm)
    if np.linalg.norm(p - tip_home) < TOOL_FRAME_TOL_MM:
        return 'tip', f'controller reports the VISION TIP at home {p.round(1).tolist()} mm'
    return None, (f'home TCP {p.round(1).tolist()} is neither the flange '
                  f'{list(HOME_FLANGE_MM)} nor the tip {tip_home.round(1).tolist()}')


# ---------------------------------------------------------------- record
_SUMMARY_COLS = [
    'round', 'tag', 'robot_x', 'robot_y', 'theta_deg', 'lift_mm', 'tool_frame',
    'target_wx', 'target_wy', 'target_wz',
    'before_tip_wx', 'before_tip_wy', 'before_tip_wz',
    'before_j1', 'before_j2', 'before_j3', 'before_j4', 'before_j5', 'before_j6',
    'standoff_converged', 'standoff_message',
    'after_tip_wx', 'after_tip_wy', 'after_tip_wz',
    'after_j1', 'after_j2', 'after_j3', 'after_j4', 'after_j5', 'after_j6',
    'correction_dz_mm', 'image', 'ok', 'message',
]


class _Record:
    def __init__(self, s):
        self.session = time.strftime('%Y%m%d_%H%M%S') + '_' + s.name
        self.dir = os.path.join(RECORD_ROOT, self.session)
        self.image_dir = os.path.join(IMAGE_ROOT, self.session)
        os.makedirs(self.dir, exist_ok=True)
        self.csv_path = os.path.join(self.dir, 'summary.csv')
        self._f = open(self.csv_path, 'w', newline='')
        self._w = csv.DictWriter(self._f, fieldnames=_SUMMARY_COLS)
        self._w.writeheader()
        self._f.flush()
        with open(os.path.join(self.dir, 'session.yaml'), 'w') as f:
            yaml.safe_dump({
                'name': s.name, 'started': self.session[:15],
                'target_world_m_plate_top': list(s.target_world_m),
                'plate_top_above_floor_m': PLATE_TOP_ABOVE_FLOOR_M,
                'stop_tags': s.stop_tags, 'rounds': s.rounds,
                'tool_rpy_deg_arm_frame': list(s.tool_rpy_deg),
                'approach_above_m': s.approach_above_m, 'dwell_s': s.dwell_s,
                'standoff': s.standoff, 'standoff_target_mm': s.standoff_target_mm,
                'pre_standoff_wait_s': s.pre_standoff_wait_s,
                'capture': s.capture, 'capture_samples': s.capture_samples,
                'tf_chain': tf_chain.TF_CHAIN_PATH,
            }, f, sort_keys=False)

    def point(self, rnd, tag, data):
        path = os.path.join(self.dir, f'r{rnd}_tag{tag}.yaml')
        with open(path, 'w') as f:
            yaml.safe_dump(_plain(data), f, sort_keys=False)
        row = {k: '' for k in _SUMMARY_COLS}
        row.update({k: v for k, v in _plain(data.get('summary', {})).items() if k in row})
        row['round'], row['tag'] = rnd, tag
        self._w.writerow(row)
        self._f.flush()
        return path

    def close(self):
        self._f.close()


def _plain(v):
    """numpy -> python for yaml/csv."""
    if isinstance(v, dict):
        return {str(k): _plain(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_plain(x) for x in v]
    if isinstance(v, np.ndarray):
        return [_plain(x) for x in v.tolist()]
    if isinstance(v, (np.floating,)):
        return round(float(v), 4)
    if isinstance(v, float):
        return round(v, 4)
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, np.bool_):
        return bool(v)
    return v


def _snapshot(s, bridge, tool_is_flange, tip_off, tip_target_arm, R_AW):
    """TCP, joints, tip (arm + world) and the Keyence line, right now."""
    tcp, joints = _arm_pose(bridge)
    if tcp is None:
        return {'error': '/arm/state has no valid pose'}
    tip_arm = tip_from_tcp(tcp, tool_is_flange, tip_off)
    tip_w = tip_world_m(s, tip_arm, tip_target_arm, R_AW)
    return {
        'tcp_mm_deg': tcp.round(3).tolist(),
        'joints_deg': joints.round(4).tolist(),
        'tip_arm_mm': tip_arm.round(2).tolist(),
        'tip_world_m': tip_w.round(5).tolist(),
        'tip_error_world_mm': ((tip_w - np.asarray(s.target_world_m)) * 1000.0).round(2).tolist(),
        'standoff_state': _standoff_state(bridge),
        'stamp': time.time(),
    }


# ---------------------------------------------------------------- main
def run_sequence(ctx, s):
    bridge = ctx.bridge
    tip_off = tf_chain.tip_offset_mm()
    ctx.log(f'{s.name}: tip -> world {s.target_world_m} (plate-top z; transform z '
            f'{s.target_world_m[2] + PLATE_TOP_ABOVE_FLOOR_M:.3f} floor datum); stops '
            f'{s.stop_tags} x {s.rounds}; tool rpy {s.tool_rpy_deg}; approach '
            f'{s.approach_above_m:.2f} m; standoff {s.standoff}'
            f'{"" if s.standoff_target_mm is None else f" (target {s.standoff_target_mm} mm)"}; '
            f'capture {s.capture}; tip offset {np.round(tip_off, 1).tolist()} mm')

    if s.dry_run:
        ctx.log('DRY RUN — design stop poses from map.yaml, nothing moves')
        for tag in s.stop_tags:
            pose = design_stop_pose(tag)
            tip_arm, flange, cmd, approach, rpy = compute_targets(s, pose, 0.0, True, tip_off)
            ok, why = check_bounds(flange)
            ctx.log(f'  tag {tag}: robot_pose ({pose.x:.3f}, {pose.y:.3f}, {pose.theta:.1f}) '
                    f'tip arm {tip_arm.round(1).tolist()} mm -> flange '
                    f'{flange.round(1).tolist()} rpy {rpy.tolist()} | approach z '
                    f'{approach[2]:.1f} | {"OK" if ok else "REFUSED"}: {why}')
        return

    # ---- preflight ----
    lift_m, msg = _lift_m(bridge)
    if lift_m is None:
        ctx.log(f'REFUSED: {msg}')
        return
    ctx.log(msg)
    if abs(lift_m) > 0.002:
        ctx.log(f'note: lift is at {lift_m * 1000:.1f} mm — compensated in the transform')
    ts = _task_state(bridge)
    if ts is None:
        ctx.log('REFUSED: no /task_state — is task_executor running?')
        return
    if ts.get('state') != 'IDLE' or ts.get('task'):
        ctx.log(f'REFUSED: task_executor is busy ({ts.get("state")} {ts.get("task")})')
        return
    if ts.get('charging') or ts.get('charge_phase') == 'charging':
        if not s.undock_if_charging:
            ctx.log('REFUSED: the robot is charging — send UNDOCK first (or set UNDOCK_IF_CHARGING)')
            return
        ctx.log('robot is charging: sending UNDOCK first (relay off, 0.10 m forward)')
        bridge.send_task_command('UNDOCK')
        ok, msg = _wait_task(ctx, 'battery_undock', 120.0)
        if not ok:
            ctx.log(f'UNDOCK failed: {msg} — stopping')
            return
        ctx.log('undocked')

    rec = _Record(s)
    ctx.log(f'record: {rec.dir}')
    n_ok = 0
    tool_frame = None

    def fail(rnd, tag, data, why):
        data.setdefault('summary', {}).update(ok=False, message=why)
        rec.point(rnd, tag, data)
        ctx.log(f'round {rnd} tag {tag}: {why}')

    try:
        for rnd in range(1, s.rounds + 1):
            for tag in s.stop_tags:
                if ctx.cancelled():
                    ctx.log('cancelled')
                    return
                label = f'round {rnd}/{s.rounds} tag {tag}'
                data = {'round': rnd, 'tag': tag,
                        'target_world_m_plate_top': list(s.target_world_m),
                        'summary': {'target_wx': s.target_world_m[0],
                                    'target_wy': s.target_world_m[1],
                                    'target_wz': s.target_world_m[2]}}
                ctx.log(f'--- {label}: GOTO {tag}')
                ok, msg, t_cmd = _goto(ctx, tag)
                if not ok:
                    fail(rnd, tag, data, f'GOTO failed — {msg}. Stopping the sequence.')
                    return
                ctx.log(f'{label}: arrived ({msg})')

                # The GOTO homed the arm: the reported pose at the home joints
                # says which tool frame the controller is in.
                frame, why = _tool_frame_at_home(bridge, tip_off)
                if frame is None:
                    fail(rnd, tag, data, f'REFUSED — {why}')
                    return
                if frame != tool_frame:
                    ctx.log(f'{why} -> tip targets '
                            f'{"converted to flange" if frame == "flange" else "sent as is"}')
                    tool_frame = frame
                is_flange = tool_frame == 'flange'

                pose, why = _robot_pose(bridge, tag, t_cmd)
                if pose is None:
                    fail(rnd, tag, data, f'REFUSED — {why}')
                    return
                lift_m, _ = _lift_m(bridge)
                lift_m = lift_m or 0.0
                tip_arm, flange, cmd, approach, rpy = compute_targets(
                    s, pose, lift_m, is_flange, tip_off)
                R_AW = world_to_arm_R(pose, lift_m)
                ok, why = check_bounds(flange)      # always the PHYSICAL flange
                data.update({
                    'robot_pose': {'x': pose.x, 'y': pose.y, 'theta_deg': pose.theta,
                                   'tag': tag, 'lift_mm': lift_m * 1000.0},
                    'tool_frame': tool_frame,
                    'tip_target_arm_mm': tip_arm.round(2).tolist(),
                    'flange_target_arm_mm': flange.round(2).tolist(),
                    'commanded_pose_mm_deg': list(cmd.round(2)) + list(rpy),
                    'approach_pose_mm_deg': list(approach.round(2)) + list(rpy),
                    'bounds': why,
                })
                data['summary'].update(robot_x=pose.x, robot_y=pose.y, theta_deg=pose.theta,
                                       lift_mm=lift_m * 1000.0, tool_frame=tool_frame)
                ctx.log(f'{label}: robot_pose ({pose.x:.3f}, {pose.y:.3f}, {pose.theta:.2f} deg), '
                        f'lift {lift_m * 1000:.1f} mm -> tip {tip_arm.round(1).tolist()} mm, '
                        f'flange {flange.round(1).tolist()} rpy {rpy.tolist()} ({why})')
                if not ok:
                    fail(rnd, tag, data, f'REFUSED — {why}')
                    return

                # approach: MoveCart (joint-interpolated) to APPROACH above
                target_hi = list(approach) + list(rpy)
                ok, msg = bridge.arm_move_cart(target_hi, vel=s.move_vel, linear=False, timeout=90.0)
                if not ok:
                    fail(rnd, tag, data, f'approach move failed — {msg}. Stopping.')
                    return
                if ctx.cancelled():
                    ctx.log('cancelled above the target (arm left there)')
                    return
                # descend: MoveL straight down
                target = list(cmd) + list(rpy)
                ok, msg = bridge.arm_move_cart(target, vel=s.line_vel, linear=True, timeout=90.0)
                if not ok:
                    fail(rnd, tag, data, f'descend failed — {msg}. Stopping (arm left in place).')
                    return

                before = _snapshot(s, bridge, is_flange, tip_off, tip_arm, R_AW)
                data['before'] = before
                if 'tip_world_m' in before:
                    tw = before['tip_world_m']
                    data['summary'].update(before_tip_wx=tw[0], before_tip_wy=tw[1], before_tip_wz=tw[2],
                                           **{f'before_j{i + 1}': j for i, j in enumerate(before['joints_deg'])})
                    ctx.log(f'{label}: AT TARGET — tip world {np.round(tw, 4).tolist()} m '
                            f'(error {before["tip_error_world_mm"]} mm), joints '
                            f'{np.round(before["joints_deg"], 2).tolist()}')
                else:
                    ctx.log(f'{label}: AT TARGET ({before.get("error")})')

                if s.standoff:
                    if not ctx.sleep(s.pre_standoff_wait_s):
                        ctx.log('cancelled at the target (arm left there)')
                        return
                    ks = _standoff_state(bridge)
                    ctx.log(f'{label}: Keyence before correction: {ks}')
                    conv, msg = bridge.arm_standoff(target_mm=s.standoff_target_mm,
                                                    timeout=s.standoff_timeout_s)
                    after = _snapshot(s, bridge, is_flange, tip_off, tip_arm, R_AW)
                    after['standoff'] = {'converged': bool(conv), 'message': msg,
                                         'target_mm': s.standoff_target_mm}
                    data['after_correction'] = after
                    data['summary'].update(standoff_converged=bool(conv), standoff_message=msg)
                    if 'tip_world_m' in after and 'tip_world_m' in before:
                        tw = after['tip_world_m']
                        dz = (after['tip_arm_mm'][2] - before['tip_arm_mm'][2])
                        data['after_correction']['correction_arm_mm'] = (
                            np.asarray(after['tip_arm_mm']) - np.asarray(before['tip_arm_mm'])).round(3).tolist()
                        data['summary'].update(after_tip_wx=tw[0], after_tip_wy=tw[1], after_tip_wz=tw[2],
                                               correction_dz_mm=dz,
                                               **{f'after_j{i + 1}': j for i, j in enumerate(after['joints_deg'])})
                        ctx.log(f'{label}: standoff {"ok" if conv else "NOT converged"} — {msg}; '
                                f'tip world {np.round(tw, 4).tolist()} m, arm dz {dz:+.2f} mm, '
                                f'joints {np.round(after["joints_deg"], 2).tolist()}')
                    else:
                        ctx.log(f'{label}: standoff {"ok" if conv else "NOT converged"} — {msg}')
                    if ctx.cancelled():
                        ctx.log('cancelled after the standoff (arm left there)')
                        return

                if s.capture:
                    ok, msg, frames = bridge.capture(num_samples=s.capture_samples,
                                                     use_vision_led=s.capture_use_led)
                    cap = {'ok': bool(ok), 'message': msg, 'frames': len(frames), 'files': []}
                    if ok and frames:
                        import cv2
                        os.makedirs(rec.image_dir, exist_ok=True)
                        for i, fr in enumerate(frames):
                            suffix = '' if len(frames) == 1 else f'_{i + 1}'
                            path = os.path.join(rec.image_dir, f'r{rnd}_tag{tag}{suffix}.png')
                            if cv2.imwrite(path, fr):
                                cap['files'].append(path)
                        data['summary']['image'] = cap['files'][0] if cap['files'] else ''
                        ctx.log(f'{label}: captured {len(frames)} frame(s) ({msg}) -> '
                                f'{cap["files"][0] if cap["files"] else "NOT saved"}')
                    else:
                        ctx.log(f'{label}: capture FAILED — {msg}')
                    data['capture'] = cap

                n_ok += 1
                data['summary'].update(ok=True, message='ok')
                path = rec.point(rnd, tag, data)
                ctx.log(f'{label}: saved {os.path.basename(path)}; dwell {s.dwell_s:.0f} s')
                if not ctx.sleep(s.dwell_s):
                    ctx.log('cancelled at the target (arm left there)')
                    return
                # ascend: MoveL straight up to the approach pose
                ok, msg = bridge.arm_move_cart(target_hi, vel=s.line_vel, linear=True, timeout=90.0)
                if not ok:
                    ctx.log(f'{label}: ascend failed — {msg}. Stopping.')
                    return
                ctx.log(f'{label}: back above the target')

        ctx.log('sequence complete: arm home')
        ok, msg = bridge.arm_home()
        ctx.log(f'arm home: {msg}')
    finally:
        rec.close()
        ctx.log(f'{n_ok} point(s) done, record {rec.dir}')
