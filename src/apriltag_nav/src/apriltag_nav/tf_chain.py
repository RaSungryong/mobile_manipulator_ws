#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tf_chain.py — the robot's FIXED transforms, in ONE place (2026-09-21).

    <apriltag_nav>/config/tf/tf_chain.yaml    every transform, with its design
                                              value and how the applied value
                                              was measured (the source of truth)
    <apriltag_nav>/config/tf/<name>.npz       the same 4x4 per transform, for
                                              tools that take an npz path
                                              (hand-eye node, chain_calib, ...)

Before this the applied values were spread over six files that had to be
changed together and were checked against each other by hand:
path_tag_locator/config/extrinsics.yaml (T_ab2mb, T_mb2fc), its hand_eye/
T_hc2ee.npz, robot.yaml `arm_calibration` (inv(T_ab2mb) as six numbers +
the vision tip), the planner URDF and a record yaml under chain_calib/docs.
Now: pose-mode IK (`arm_transform`), the tip conversion (`arm_controller`),
the locator / calibration chain (`path_tag_locator`, `chain_calib`,
`robot_sim`) and `set_tool_tcp.py` all read HERE. The planner URDF is the
one copy that stays outside (it is the planner's input file);
`tools/check_pose_vs_joint.py` asserts it agrees.

Convention (path_tag_locator.geometry): T_X2Y = pose of frame Y expressed
in frame X — a Y-frame point p maps to X as T_X2Y @ p. Row-major 4x4,
metres. `rpy_deg_xyz` in the yaml is scipy's extrinsic 'xyz' euler.

Frames:
    mb   mobile base body: x forward, y left, z up, origin on the floor
         under the chassis centre (what /robot_pose is anchored to)
    ab   arm base_link (Fairino base)
    ee   arm FLANGE — what /arm/state reports (the controller's active tool
         is the flange, CLAUDE.md 2026-09-14)
    tip  the Basler vision tip: where the Basler's frame centre looks at the
         Keyence standoff; orientation = the flange's
    hc   hand_cam optical frame (RealSense D435 colour): x image right,
         y image down, z optical axis
    fc   front_cam optical frame — the PHYSICAL camera, 1.44 deg tilted.
         robot_camera_node publishes front_cam detections in a LEVEL virtual
         camera while robot.yaml ground_plane is on; the level frame is
         DERIVED (path_tag_locator.constants.load_extrinsics_full), never
         stored, so it cannot drift from the fit.

Writing: `write_transform()` rewrites ONE block's `source` / `t_mm` /
`rpy_deg_xyz` / `matrix` lines in place (comments and the `design` block
survive) and the matching npz. `handeye_calib_node`'s compute and
`tools/tf_chain_tool.py set|front-cam` are the writers; nothing else should
edit the yaml's numbers by hand. T_mb2fc is GENERATED from robot.yaml
(`physical_T_mb2fc`) — `tf_chain_tool.py front-cam --apply` after a
front_cam re-fit.

Pure numpy; no ROS imports, so every offline check can use it.
"""
import math
import os
import re

import numpy as np

from apriltag_nav.paths import CONFIG_DIR, CONFIG_PATH

TF_DIR = os.path.join(CONFIG_DIR, 'tf')
TF_CHAIN_PATH = os.path.join(TF_DIR, 'tf_chain.yaml')

# Every transform the yaml must carry. Loading refuses a file missing one.
NAMES = ('T_ab2mb', 'T_mb2fc', 'T_hc2ee', 'T_ee2tip')

# The arm's joint zero offsets (2026-09-22): not a transform, but part of the
# same calibrated set — see `load_joint_offsets` / `arm_flange_T`.
JOINT_OFFSETS_PATH = os.path.join(TF_DIR, 'arm_joint_offsets.yaml')
JOINT_OFFSETS_NPZ = os.path.join(TF_DIR, 'arm_joint_offsets.npz')

# Rotation of the LEVEL virtual front_cam in the mobile-base frame: x = image
# right = forward, y = image down = robot right, z = optical axis = down.
R_MB2FC_LEVEL = np.diag([1.0, -1.0, -1.0])

_RIGID_ATOL = 1e-6


# ---------------------------------------------------------------- basics
def npz_path(name, tf_dir=None):
    """<tf_dir>/<name>.npz — the npz twin of a yaml block."""
    if name not in NAMES:
        raise KeyError('unknown transform %r (known: %s)' % (name, ', '.join(NAMES)))
    return os.path.join(tf_dir or TF_DIR, name + '.npz')


def invert_T(T):
    T = np.asarray(T, dtype=np.float64)
    Rm = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = Rm.T
    out[:3, 3] = -Rm.T @ T[:3, 3]
    return out


def assert_rigid(T, name='T', atol=_RIGID_ATOL):
    T = np.asarray(T, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError('%s: expected a 4x4, got %s' % (name, T.shape))
    Rm = T[:3, :3]
    if np.abs(Rm @ Rm.T - np.eye(3)).max() > atol or abs(np.linalg.det(Rm) - 1.0) > atol:
        raise ValueError('%s: rotation is not orthonormal / det +1' % name)
    if np.abs(T[3] - [0, 0, 0, 1]).max() > atol:
        raise ValueError('%s: last row is not [0 0 0 1]' % name)


def _as_rotation(Rm):
    from scipy.spatial.transform import Rotation as R
    # scipy compat: >=1.4 from_matrix(), 1.3 from_dcm()
    return R.from_matrix(Rm) if hasattr(R, 'from_matrix') else R.from_dcm(Rm)


def rpy_deg_xyz(T):
    """scipy extrinsic 'xyz' euler of T's rotation, degrees (as in the yaml)."""
    return [float(v) for v in _as_rotation(np.asarray(T)[:3, :3]).as_euler('xyz', degrees=True)]


def t_mm(T):
    return [float(v) * 1000.0 for v in np.asarray(T)[:3, 3]]


def describe(T, name=''):
    t = t_mm(T)
    r = rpy_deg_xyz(T)
    return ('%s t (%.2f, %.2f, %.2f) mm  rpy_xyz (%.3f, %.3f, %.3f) deg'
            % (name, t[0], t[1], t[2], r[0], r[1], r[2])).strip()


def matrix_from_pose(t_m, rpy_deg):
    """4x4 from a translation (m) and scipy extrinsic 'xyz' euler (deg)."""
    from scipy.spatial.transform import Rotation as R
    rot = R.from_euler('xyz', rpy_deg, degrees=True)
    T = np.eye(4)
    T[:3, :3] = rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()
    T[:3, 3] = t_m
    return T


# ---------------------------------------------------------------- reading
def _matrix_from_entry(name, entry):
    if not isinstance(entry, dict) or 'matrix' not in entry:
        raise ValueError('%s: block has no `matrix`' % name)
    m = np.asarray(entry['matrix'], dtype=np.float64)
    if m.size != 16:
        raise ValueError('%s: matrix must hold 16 numbers, got %d' % (name, m.size))
    T = m.reshape(4, 4)
    assert_rigid(T, name)
    return T


def load_tf_chain(path=None, require=NAMES):
    """{name: 4x4} for every transform block in tf_chain.yaml.

    `require`: names that must be present (default: all of NAMES). Extra
    blocks with a `matrix` are loaded too."""
    import yaml
    path = path or TF_CHAIN_PATH
    with open(path, 'r') as fh:
        d = yaml.safe_load(fh) or {}
    out = {}
    for name, entry in d.items():
        if isinstance(entry, dict) and 'matrix' in entry:
            out[name] = _matrix_from_entry(name, entry)
    missing = [n for n in (require or ()) if n not in out]
    if missing:
        raise ValueError('%s: missing transform block(s) %s' % (path, ', '.join(missing)))
    return out


def load_transform(name, path=None):
    return load_tf_chain(path, require=(name,))[name]


def load_entry(name, path=None):
    """The raw yaml dict of one block (source, design, ... as written)."""
    import yaml
    with open(path or TF_CHAIN_PATH, 'r') as fh:
        d = yaml.safe_load(fh) or {}
    if name not in d:
        raise KeyError('%s: no block %s' % (path or TF_CHAIN_PATH, name))
    return d[name]


def load_npz(path):
    """A 4x4 from an npz (first key) — the same reader path_tag_locator uses."""
    if not os.path.exists(path):
        raise FileNotFoundError('transform npz not found: %s' % path)
    data = np.load(path)
    if len(data.files) == 0:
        raise ValueError('%s: empty npz' % path)
    T = np.asarray(data[data.files[0]], dtype=np.float64)
    assert_rigid(T, os.path.basename(path))
    return T


# ---------------------------------------------------------------- joint offsets
def load_joint_offsets(path=None, require_enabled=True):
    """dq_deg (6,) — the FR10v6 joint ZERO offsets, ADDED to the controller's
    reading to get the physical angle (chain_calib/arm_offsets.py's
    convention). Zeros when the file is absent or `enabled: false`
    (`require_enabled=False` returns the stored values regardless).

    What they are for: the controller reports its TCP as FK(q) with the
    URDF's nominal zeros; the sheet sessions showed the lens landing
    +-15 mm from where that FK puts it depending on the configuration.
    `arm_flange_T` recomputes T_ab2ee = FK(q + dq) from the joints, so the
    locator chain (path_tag_locator, chain_calib) sees the corrected
    flange. They were fitted with the applied T_hc2ee FIXED (`hand_eye`
    below names it), so the pair (T_hc2ee, dq) is one set — refitting
    either means refitting the other and T_ab2mb after it."""
    import yaml
    path = path or JOINT_OFFSETS_PATH
    if not os.path.exists(path):
        return np.zeros(6)
    with open(path, 'r') as fh:
        d = yaml.safe_load(fh) or {}
    dq = np.asarray(d.get('dq_deg') or [0.0] * 6, dtype=np.float64).ravel()
    if dq.size != 6:
        raise ValueError('%s: dq_deg must hold 6 values, got %d' % (path, dq.size))
    if np.abs(dq).max() > 5.0:
        raise ValueError('%s: an offset over 5 deg (%s) is not a zero offset — refused' % (path, dq.round(3).tolist()))
    if require_enabled and not bool(d.get('enabled', True)):
        return np.zeros(6)
    return dq


def load_joint_offsets_entry(path=None):
    import yaml
    with open(path or JOINT_OFFSETS_PATH, 'r') as fh:
        return yaml.safe_load(fh) or {}


def write_joint_offsets(dq_deg, source, hand_eye, urdf, session='', enabled=True, note='',
                        path=None, npz_path_=None):
    """Write arm_joint_offsets.yaml (+ its npz twin). Whole-file rewrite —
    unlike the transform blocks this file is small and has no design
    block to preserve; the provenance goes into the fields."""
    dq = np.asarray(dq_deg, dtype=np.float64).ravel()
    if dq.size != 6:
        raise ValueError('dq_deg must hold 6 values')
    path = path or JOINT_OFFSETS_PATH
    npz_path_ = npz_path_ or os.path.join(os.path.dirname(path), 'arm_joint_offsets.npz')
    lines = [
        '# FR10v6 joint ZERO offsets (deg), ADDED to the controller\'s joint reading =',
        '# the physical angle. Consumed by apriltag_nav.tf_chain.arm_flange_T: the',
        '# locator chain\'s T_ab2ee is FK_urdf(q + dq_deg) instead of the TCP the',
        '# controller reports (= FK_urdf(q), agreement 0.00 mm). J1 is 0 by',
        '# construction (a J1 offset is a base yaw, which T_ab2mb holds).',
        '# Fitted with the hand-eye below FIXED, so (T_hc2ee, dq_deg, T_ab2mb) are one',
        '# set — a new hand-eye or a new K means refitting this and T_ab2mb after it.',
        '# `enabled: false` makes every consumer use dq = 0 (the controller\'s pose).',
        '# Writer: tools/tf_chain_tool.py joint-offsets --apply <session arm_offsets.npz>.',
        'enabled: %s' % ('true' if enabled else 'false'),
        'dq_deg: [%s]' % ', '.join('%.6f' % v for v in dq),
        'source: %s' % _yaml_str(source),
        'hand_eye: %s' % _yaml_str(hand_eye),
        'urdf: %s' % _yaml_str(urdf),
        'session: %s' % _yaml_str(session),
        'note: %s' % _yaml_str(note),
        '',
    ]
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines))
    np.savez(npz_path_, dq_deg=dq)
    back = load_joint_offsets(path, require_enabled=False)
    if np.abs(back - dq).max() > 1e-5:
        raise RuntimeError('%s: read-back differs from what was written' % path)
    return path, npz_path_


def check_joint_offsets(path=None, atol=1e-6):
    """(ok, reason): yaml == npz, 6 values, all under 5 deg. ok with 'absent'
    when there is no file (offsets are optional)."""
    path = path or JOINT_OFFSETS_PATH
    if not os.path.exists(path):
        return True, 'absent (dq = 0)'
    try:
        dq = load_joint_offsets(path, require_enabled=False)
        npz = os.path.join(os.path.dirname(path), 'arm_joint_offsets.npz')
        if not os.path.exists(npz):
            return False, 'npz twin missing: %s' % npz
        d = float(np.abs(np.load(npz)['dq_deg'] - dq).max())
        if d > atol:
            return False, 'npz != yaml (%.1e deg)' % d
        en = bool(load_joint_offsets_entry(path).get('enabled', True))
        return True, '%s, dq %s deg' % ('enabled' if en else 'DISABLED', dq.round(3).tolist())
    except Exception as e:                            # noqa: BLE001
        return False, str(e)


_ARM_CHAIN = None


def arm_flange_T(tcp_pose_mm_deg, joints_deg=None, dq_deg=None, tol_mm=2.0, tol_deg=0.1, warn=None):
    """T_ab2ee (4x4, m) for the locator chain.

    With joints and non-zero offsets: FK_urdf(q + dq). Guard: FK_urdf(q)
    must reproduce the controller's TCP within `tol_mm` / `tol_deg` —
    otherwise the joints and the pose are not from the same state (or the
    URDF is not the controller's model) and the controller's pose is used
    unchanged, with `warn(msg)` called if given. Without joints, or with
    dq = 0, it is exactly pose_fr5_to_matrix_m(tcp_pose): bit-for-bit the
    pre-2026-09-22 chain.

    Returns (T_ab2ee, applied: bool)."""
    global _ARM_CHAIN
    from path_tag_locator.geometry import pose_fr5_to_matrix_m
    T_ctrl = pose_fr5_to_matrix_m([float(v) for v in tcp_pose_mm_deg])
    dq = None if dq_deg is None else np.asarray(dq_deg, dtype=np.float64).ravel()
    if joints_deg is None or dq is None or not np.any(dq):
        return T_ctrl, False
    q = np.asarray(joints_deg, dtype=np.float64).ravel()
    if q.size != 6:
        if warn:
            warn('arm_flange_T: %d joint value(s), need 6 — joint offsets NOT applied' % q.size)
        return T_ctrl, False
    if _ARM_CHAIN is None:
        from apriltag_nav.arm_fk import ArmChain
        _ARM_CHAIN = ArmChain()
    T_fk = _ARM_CHAIN.fk_flange(q)
    dt = 1e3 * float(np.linalg.norm(T_fk[:3, 3] - T_ctrl[:3, 3]))
    c = (np.trace(T_fk[:3, :3].T @ T_ctrl[:3, :3]) - 1.0) / 2.0
    dr = math.degrees(math.acos(max(-1.0, min(1.0, c))))
    if dt > tol_mm or dr > tol_deg:
        if warn:
            warn('arm_flange_T: URDF FK(q) is %.2f mm / %.3f deg from the controller TCP (joints and pose '
                 'from different states, or the URDF is not the controller model) — joint offsets NOT applied'
                 % (dt, dr))
        return T_ctrl, False
    return _ARM_CHAIN.fk_flange(q, dq_deg=dq), True


# ---------------------------------------------------------------- derived forms
def arm_calibration_from_T_ab2mb(T_ab2mb):
    """T_ab2mb -> the six numbers arm_transform.transform_world_to_arm uses.

    T_mb2ab = inv(T_ab2mb) = [Rz(mount_yaw) Ry(tilt_y) Rx(tilt_x) |
    (offset_x, offset_y, base_z)] in the body frame (x forward, y left,
    z up). Yaw is wrapped to [0, 2pi) so the design value reads pi, not -pi.
    base_z is the arm base height with the LIFT AT ITS ORIGIN."""
    M = invert_T(T_ab2mb)
    yaw, ty, tx = _as_rotation(M[:3, :3]).as_euler('ZYX')   # intrinsic Z, Y, X
    yaw = float(yaw) % (2.0 * math.pi)
    return dict(arm_body_offset_x=float(M[0, 3]),
                arm_body_offset_y=float(M[1, 3]),
                arm_base_z=float(M[2, 3]),
                arm_mount_yaw=yaw,
                arm_tilt_x=float(tx),
                arm_tilt_y=float(ty))


def T_ab2mb_from_arm_calibration(c):
    """Inverse of arm_calibration_from_T_ab2mb (for checks and the design value)."""
    from scipy.spatial.transform import Rotation as R
    rot = (R.from_euler('z', float(c['arm_mount_yaw']))
           * R.from_euler('y', float(c['arm_tilt_y']))
           * R.from_euler('x', float(c['arm_tilt_x'])))
    M = np.eye(4)
    M[:3, :3] = rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()
    M[:3, 3] = [float(c['arm_body_offset_x']), float(c['arm_body_offset_y']), float(c['arm_base_z'])]
    return invert_T(M)


def tip_offset_mm(path=None):
    """vision tip in the flange frame, mm (T_ee2tip translation)."""
    return np.asarray(t_mm(load_transform('T_ee2tip', path)), dtype=float)


def urdf_mobile_to_base(T_ab2mb):
    """(xyz m, rpy rad) of the planner URDF's `mobile_to_base` joint for this
    T_ab2mb: the same transform seen from the URDF's `mobile_base` frame,
    which is yawed 180 deg from mb (its +x points backwards)."""
    from scipy.spatial.transform import Rotation as R
    Rz180 = np.diag([-1.0, -1.0, 1.0, 1.0])
    M = Rz180 @ invert_T(T_ab2mb)          # mobile_base -> base_link
    rot = _as_rotation(M[:3, :3])
    return M[:3, 3].tolist(), [float(v) for v in rot.as_euler('xyz')]


# ---------------------------------------------------------------- front_cam generator
def physical_T_mb2fc(robot_cfg):
    """(T_mb2fc_physical, params) from a parsed robot.yaml — the ONLY way the
    T_mb2fc block should ever change.

        T_mb2fc_level    = [R_MB2FC_LEVEL | (camera_offset, camera_lateral,
                                             height_m + tag_thickness)]
        T_mb2fc_physical = T_mb2fc_level @ inv(T_tilted_to_level(roll, pitch, yaw))

    tz is the lens height above the FLOOR (= mb origin): the tag-pair fit
    measures the lens above the TAG-TOP plane (height_m) and every laid tag
    is a `tag_thickness` plate."""
    from apriltag_nav.ground_plane import T_tilted_to_level
    cam_off = float(robot_cfg['robot']['camera_offset'])
    cam_lat = float(robot_cfg['robot'].get('camera_lateral', 0.0) or 0.0)
    gp = ((robot_cfg.get('robot_camera') or {}).get('ground_plane') or {}).get('front_cam') or {}
    roll = float(gp.get('roll_deg', 0.0))
    pitch = float(gp.get('pitch_deg', 0.0))
    yaw = float(gp.get('yaw_deg', 0.0))
    height = float(gp.get('height_m', 0.30))
    thick = float(robot_cfg['robot'].get('tag_thickness', 0.0) or 0.0)
    T_level = np.eye(4)
    T_level[:3, :3] = R_MB2FC_LEVEL
    T_level[:3, 3] = [cam_off, cam_lat, height + thick]
    tilt = T_tilted_to_level(math.radians(roll), math.radians(pitch), math.radians(yaw))
    T_phys = T_level @ invert_T(tilt)
    return T_phys, dict(camera_offset=cam_off, camera_lateral=cam_lat, roll_deg=roll,
                        pitch_deg=pitch, yaw_deg=yaw, height_m=height, tag_thickness=thick)


def front_cam_source_line(params):
    return ('GENERATED by tools/tf_chain_tool.py front-cam from robot.yaml: camera_offset %.3f, '
            'camera_lateral %.3f, ground_plane.front_cam roll %+.3f pitch %+.3f yaw %+.3f deg, '
            'height_m %.3f + tag_thickness %.3f - do not hand-edit'
            % (params['camera_offset'], params['camera_lateral'], params['roll_deg'],
               params['pitch_deg'], params['yaw_deg'], params['height_m'], params['tag_thickness']))


# ---------------------------------------------------------------- writing
def format_matrix_lines(T, indent='    '):
    rows = []
    for r in range(4):
        cells = ['%12.9f' % T[r, c] for c in range(4)]
        rows.append(indent + ' ' + ', '.join(cells) + (',' if r < 3 else ' ]'))
    rows[0] = indent + '[' + rows[0][len(indent) + 1:]
    return '\n'.join(rows) + '\n'


def _yaml_str(s):
    return '"' + str(s).replace('\\', '\\\\').replace('"', '\\"').replace('\n', ' ') + '"'


_KEY_LINE = {
    'source': re.compile(r'^  source:[^\n]*\n', re.MULTILINE),
    't_mm': re.compile(r'^  t_mm:[^\n]*\n', re.MULTILINE),
    'rpy_deg_xyz': re.compile(r'^  rpy_deg_xyz:[^\n]*\n', re.MULTILINE),
    # `  matrix:` then indented lines up to and including the one holding `]`
    'matrix': re.compile(r'^  matrix:[^\n]*\n(?:[ \t]+[^\n]*\n)*?[ \t]+[^\n]*\][^\n]*\n', re.MULTILINE),
}


def _block_span(text, name):
    m = re.search(r'^%s:[^\n]*\n' % re.escape(name), text, re.MULTILINE)
    if not m:
        raise KeyError('block %s not found' % name)
    start = m.start()
    body = m.end()
    nxt = re.search(r'^\S', text[body:], re.MULTILINE)
    end = body + nxt.start() if nxt else len(text)
    return start, body, end


def render_block_values(T, source):
    t = t_mm(T)
    r = rpy_deg_xyz(T)
    return {
        'source': '  source: %s\n' % _yaml_str(source),
        't_mm': '  t_mm: [%.3f, %.3f, %.3f]\n' % tuple(t),
        'rpy_deg_xyz': '  rpy_deg_xyz: [%.4f, %.4f, %.4f]\n' % tuple(r),
        'matrix': '  matrix:   # row-major 4x4, metres\n' + format_matrix_lines(T),
    }


def write_transform(name, T, source, path=None, tf_dir=None, write_npz=True):
    """Replace one block's source / t_mm / rpy_deg_xyz / matrix in tf_chain.yaml
    (text-level: comments and the design block stay) and rewrite <name>.npz.
    Returns the paths written."""
    T = np.asarray(T, dtype=np.float64)
    assert_rigid(T, name)
    path = path or TF_CHAIN_PATH
    tf_dir = tf_dir or os.path.dirname(path)
    with open(path, 'r') as fh:
        text = fh.read()
    start, body, end = _block_span(text, name)
    block = text[body:end]
    vals = render_block_values(T, source)
    for key in ('source', 't_mm', 'rpy_deg_xyz', 'matrix'):
        rx = _KEY_LINE[key]
        m = rx.search(block)
        if not m:
            raise ValueError('%s: block %s has no `%s` line to replace' % (path, name, key))
        block = block[:m.start()] + vals[key] + block[m.end():]
    new_text = text[:body] + block + text[end:]
    with open(path, 'w') as fh:
        fh.write(new_text)
    written = [path]
    if write_npz:
        p = os.path.join(tf_dir, name + '.npz')
        os.makedirs(tf_dir, exist_ok=True)
        np.savez(p, T)
        written.append(p)
    # round trip: the loader must accept what was written
    T_back = load_transform(name, path)
    if np.abs(T_back - T).max() > 1e-8:
        raise RuntimeError('%s: written block does not read back (max diff %.1e)'
                           % (name, np.abs(T_back - T).max()))
    return written


def export_npz(path=None, tf_dir=None):
    """Rewrite every <name>.npz from the yaml (yaml is the source of truth)."""
    path = path or TF_CHAIN_PATH
    tf_dir = tf_dir or os.path.dirname(path)
    chain = load_tf_chain(path)
    out = []
    for name in NAMES:
        p = os.path.join(tf_dir, name + '.npz')
        np.savez(p, chain[name])
        out.append(p)
    return out


# ---------------------------------------------------------------- checks
def check_npz_agree(path=None, tf_dir=None, atol=1e-8):
    """[(name, ok, max_diff_or_reason)] — each npz vs its yaml block."""
    path = path or TF_CHAIN_PATH
    tf_dir = tf_dir or os.path.dirname(path)
    chain = load_tf_chain(path)
    out = []
    for name in NAMES:
        p = os.path.join(tf_dir, name + '.npz')
        try:
            d = float(np.abs(load_npz(p) - chain[name]).max())
            out.append((name, d <= atol, d))
        except Exception as e:                        # noqa: BLE001
            out.append((name, False, str(e)))
    return out


def check_front_cam(path=None, robot_yaml=None, atol=1e-8):
    """(ok, max_diff, params): the stored T_mb2fc == the generator's output from
    robot.yaml. False means robot.yaml's ground-plane fit or camera_offset
    changed and `tf_chain_tool.py front-cam --apply` was not run."""
    import yaml
    with open(robot_yaml or CONFIG_PATH, 'r') as fh:
        cfg = yaml.safe_load(fh)
    T_gen, params = physical_T_mb2fc(cfg)
    T_stored = load_transform('T_mb2fc', path)
    d = float(np.abs(T_gen - T_stored).max())
    return d <= atol, d, params
