#!/usr/bin/env python3
"""front_cam pose calibration: T_mb2fc (all six DOF) from a laid tag pair.

What is measured, and by which motion (2026-09-15; the 2026-09-08 tag-pair
session made into a repeatable procedure):

    roll, pitch, lens height    tag-pair fit over snapshots AT REST
                                (fit_front_cam_ground.fit_ground)
    tx, ty                      the centre of the arc the lens draws during
                                in-place PIVOTS (one linear least squares
                                over every pivot snapshot) — i.e. the lens position
                                relative to the base's ROTATION CENTRE, which
                                is the point /robot_pose, the align lever and
                                aim-and-drive all refer to
    yaw (about the optical axis, vs the travel axis)
                                a straight DRIVE while the tag pair is in
                                view: the lens's lateral motion in the body
                                frame must equal tx * (body yaw change), the
                                camera yaw is what makes that hold

Everything is read off the tags AT REST or frame by frame — commanded
distances/angles are never used (the base under-executes small moves by
~45 % and pivots by ~25 %, and yaws on its own while driving).

The mobile-base frame this produces has its ORIGIN AT THE ROTATION CENTRE.
Whether that is the chassis' geometric centre is a mechanical question the
camera cannot answer — see docs/FRONT_CAM_POSE_CALIBRATION_kr.md for the
floor-mark test.

Preconditions
  * mobile_manipulator.launch up (mobile_node drives, robot_camera_node
    detects); path_tag_locator.launch NOT running (second base commander);
    a hand on the e-stop for `collect`.
  * robot_camera_node publishing RAW detections: robot.yaml
    robot_camera.ground_plane.front_cam.enabled: false, node restarted
    (`rosnode kill /robot_camera_node && rosrun apriltag_nav
    robot_camera_node.py`). `snap`/`collect` refuse corrected detections
    (pose_z pinned to height_m gives them away).
  * two tags of ONE printed size laid flat with their CENTRES a precisely
    known distance apart, both in the frame with the pair roughly centred
    and the first -> second direction along the robot's FORWARD axis
    (image right). Since 2026-09-15 the tags need not be edge-parallel:
    the fit solves each tag's in-plane angle, only the centre spacing is
    the ruler. Defaults are the 90 mm tags 149 -> 150 (the 60 mm pair
    15 / 16 of 2026-09-08 is gone); measure the spacing as
    (outer extent + inner gap) / 2 so the printed size drops out, and pass
    it to `solve --spacing`. The fit's lens height is above the TAG-TOP
    plane; tf_chain.yaml T_mb2fc's tz adds robot.yaml robot.tag_thickness (1 mm).
  * frame room: the body / front bumper hides the LEFT THIRD of the
    image (`--left-edge-px`, 430), and 90 mm tags fill ~225 px each at
    0.30 m, so with 0.12 m of centre spacing the pair spans 0.21 m of the
    0.34 m usable width. `check` prints the room; `collect` caps every
    scan move to the room that keeps BOTH tags in view and every drive
    track to the room that keeps ONE tag in view — the yaw fit places a
    frame from a single tag (its square orientation + fitted laying
    angle), so a track can run ~0.15 m instead of ~0.05.

Commands
  check               preconditions only (nothing moves)
  snap DIR NAME       one at-rest snapshot (any name; 'scan_'/'piv_' prefixes
                      decide what solve uses it for)
  record DIR NAME     record every frame for --seconds while YOU drive
                      (robot_ui Mobile tab, ≤ 0.1 m, straight) -> drive_NAME
  collect DIR         the whole session through mobile_node: 8 small moves
                      with snapshots, 8 pivots of ±2 deg (cumulative ±4)
                      with snapshots,
                      then --drive-repeat x forward/back drives of --drive m
                      (0.10, capped to the frame room) recorded as tracks.
                      Asks before the first motion.
  solve DIR           fit + rotation centre + yaw; prints the robot.yaml
                      numbers; --apply writes them and regenerates
                      config/tf/tf_chain.yaml T_mb2fc. --spacing is REQUIRED.

    rosrun apriltag_nav calib_front_cam_pose.py check
    rosrun apriltag_nav calib_front_cam_pose.py collect log/apriltag_nav/calib_pair_<date>
    rosrun apriltag_nav calib_front_cam_pose.py --spacing 0.120 solve log/apriltag_nav/calib_pair_<date> [--apply]
"""
import argparse
import glob
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import yaml

_TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_TOOLS, '..', 'src'))
sys.path.insert(0, _TOOLS)
from apriltag_nav.ground_plane import GroundPlane  # noqa: E402
from apriltag_nav.paths import CONFIG_PATH, WS_DIR  # noqa: E402
from fit_front_cam_ground import (fit_ground, load_snapshots, parse_camera_info,  # noqa: E402
                                  parse_snapshot)

DEFAULT_TAGS = (149, 150)      # 2026-09-15: the 90 mm tags on hand (the 60 mm 15/16 pair is gone); user's pick
DEFAULT_SIZE = 0.090
DEFAULT_SPACING = None         # no default on purpose — the measured centre distance is the ruler
ROOM_MARGIN_M = 0.015          # keep every tag this far inside the usable frame
DEFAULT_LEFT_EDGE_PX = 430     # the body / front bumper hides the left third of the
                               # image (user, 2026-09-15: "화면 좌측의 1/3까지는 가려져")


def frame_room_m(det, tags, fx, fy, h, width, height, x_min_px=DEFAULT_LEFT_EDGE_PX, keep='both'):
    """How far the base may drive before a tag corner leaves the USABLE
    frame (x from x_min_px — the bumper edge — to width), from one raw
    detection dict {id: {'c': corners(4,2)}}: dict(fwd, rev, left, right)
    in metres at lens height h. Driving FORWARD the floor moves toward
    image LEFT (image right = forward), so the forward room is the corner
    nearest the bumper edge; reverse is the room to the right edge; left /
    right are the lateral rooms (image up / down = robot left / right).
    keep='both': every tag must stay in view (snapshots); keep='one': at
    least one tag must (drive tracks, which use single-tag frames)."""
    per = []
    for tg in tags:
        if tg not in det:
            continue
        c = np.asarray(det[tg]['c'], float)
        per.append((c[:, 0].min() - x_min_px, width - c[:, 0].max(), c[:, 1].min(), height - c[:, 1].max()))
    if not per:
        raise ValueError("no calibration tag in the frame")
    if keep != 'one' and len(per) < len(tags):
        return dict(fwd=0.0, rev=0.0, left=0.0, right=0.0)     # a tag is already out of view
    agg = max if keep == 'one' else min
    return dict(fwd=float(agg(p[0] for p in per) * h / fx),
                rev=float(agg(p[1] for p in per) * h / fx),
                left=float(min(p[2] for p in per) * h / fy),
                right=float(min(p[3] for p in per) * h / fy))


def recentre_m(det, tags, fx, h, width, x_min_px=DEFAULT_LEFT_EDGE_PX):
    """Signed body-x drive that puts the pair's centre in the middle of
    the USABLE image (+ = the pair is ahead of that middle, drive
    forward). Needs both tags."""
    xs = [np.asarray(det[tg]['c'], float)[:, 0].mean() for tg in tags]
    return float((np.mean(xs) - 0.5 * (x_min_px + width)) * h / fx)


PIVOT_MARGIN_M = 0.012         # quiet zone: at 8.7 deg on 2026-09-15 a tag 6 mm from the edge was still detected


def max_pivot_from_corners(det, tags, camera_offset, fx, fy, cxK, h, height, margin=PIVOT_MARGIN_M,
                           keep='both', direction=0):
    """Largest body rotation (deg) from the pose of `det` that keeps both
    tags (keep='both') or at least one (keep='one') inside the image's
    vertical extent: a tag centre is camera_offset + (its distance ahead
    of the principal point) from the rotation centre and swings sideways
    (image y) by that times sin(angle); the room is its distance to the
    edge it moves toward minus `margin`. `direction` is the sign of the
    rotation in IMAGE-ANGLE terms (+ = the tags' edge angle grows = the
    tags move DOWN the image, toward the bottom edge; − = up; 0 = the
    smaller of the two, for a sweep either way). None when no tag is
    present. This is the physical limit of the pivot sweep — with the
    0.24 m tall view at 0.30 m and a ~0.6 m lever it is ~7 deg."""
    caps = []
    for tg in tags:
        if tg not in det:
            continue
        c = np.asarray(det[tg]['c'], float)
        top, bottom = c[:, 1].min(), height - c[:, 1].max()
        room_px = bottom if direction > 0 else top if direction < 0 else min(top, bottom)
        lat = room_px * h / fy - margin
        lever = abs(camera_offset) + max(0.0, (c[:, 0].mean() - cxK) * h / fx)
        caps.append(math.degrees(math.asin(min(1.0, max(0.0, lat) / max(lever, 0.3)))))
    if not caps:
        return None
    return max(caps) if keep == 'one' else min(caps)


def cap_distance(dist, room, margin=ROOM_MARGIN_M):
    """Clip a signed body-x distance to the frame room in its direction
    (room dict from frame_room_m). Returns (capped, was_capped)."""
    avail = max(0.0, (room['fwd'] if dist > 0 else room['rev']) - margin)
    if abs(dist) <= avail:
        return dist, False
    return math.copysign(avail, dist), True


# =====================================================================
# pure maths (no ROS) — exercised by tools/check_front_cam_pose_calib.py
# =====================================================================
def R2(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def pair_pose_in_cam(gp, det, tags):
    """(pair centre m (2,), pair x-axis angle phi) in the LEVEL camera frame
    C (gp with yaw 0: x = image right, y = image down), from one detection
    dict {id: {'c': corners(4,2)}}. Uses both tags' corner centroids."""
    g0 = gp.to_ground(det[tags[0]]['c'])
    g1 = gp.to_ground(det[tags[1]]['c'])
    m = np.vstack([g0, g1]).mean(0)
    d = g1.mean(0) - g0.mean(0)
    return m, math.atan2(d[1], d[0])


def square_angle(g):
    """Orientation of a ground-projected square (4,2) in corner order:
    the mean direction of its four edges, each brought back to the c0->c1
    direction (edge k is turned k*90 deg the OTHER way round the square:
    c0->c1 at +90, c1->c2 at 0, c2->c3 at -90 ...), i.e. the c0->c1 edge
    direction with the noise of all four edges."""
    vx = vy = 0.0
    for k in range(4):
        d = g[(k + 1) % 4] - g[k]
        a = math.atan2(d[1], d[0]) + k * math.pi / 2
        vx += math.cos(a)
        vy += math.sin(a)
    return math.atan2(vy, vx)


def pair_pose_any(gp, det, tags, spacing, tag_rot):
    """pair_pose_in_cam when both tags are in `det`; from ONE tag when only
    one is (2026-09-15 — the bumper hides the left third of front_cam's
    view, so a drive track loses a tag long before the frame edge). A
    single tag's square orientation is the pair direction + 90 deg + its
    fitted laying angle (fit_ground's tag_rot), and the pair centre is
    half a spacing from its centroid along that direction. Returns
    (m, phi) or None when neither tag is present."""
    have = [tg for tg in tags if tg in det]
    if len(have) == 2:
        return pair_pose_in_cam(gp, det, tags)
    if not have:
        return None
    k = tags.index(have[0])
    g = gp.to_ground(det[have[0]]['c'])
    phi = square_angle(g) - math.pi / 2 - tag_rot[k]
    u = np.array([math.cos(phi), math.sin(phi)])
    m = g.mean(0) + (0.5 if k == 0 else -0.5) * spacing * u
    return m, phi


def lens_in_pair_frame(m, phi):
    """Lens nadir position in the pair frame T (origin pair centre, x along
    15 -> 16), from the pair's pose in C."""
    return -R2(phi).T @ m


def rotation_centres(gp, piv_snaps, tags, min_deg=1.0, spacing=None, tag_rot=(0.0, 0.0)):
    """For consecutive pivot snapshots: the centre of the arc the lens drew,
    relative to the lens nadir, in the LEVEL camera frame C of the first
    snapshot. Returns [(name0, name1, dtheta_rad, centre_C (2,))]. With
    `spacing` given a snapshot showing ONE tag is placed from it."""
    out = []
    if spacing:
        poses = [(n, pair_pose_any(gp, d, tags, spacing, tag_rot)) for n, d in piv_snaps]
        poses = [(n, p) for n, p in poses if p is not None]
    else:
        poses = [(n, pair_pose_in_cam(gp, d, tags)) for n, d in piv_snaps]
    for (n0, (m0, p0)), (n1, (m1, p1)) in zip(poses, poses[1:]):
        dphi = p1 - p0
        dtheta = -dphi                      # body yaw in T = -(pair angle in C) + const
        if abs(math.degrees(dtheta)) < min_deg:
            continue
        l0, l1 = lens_in_pair_frame(m0, p0), lens_in_pair_frame(m1, p1)
        Rd = R2(dtheta)
        c_T = np.linalg.solve(np.eye(2) - Rd, l1 - Rd @ l0)   # l1 - c = Rd (l0 - c)
        c_C = R2(p0) @ (c_T - l0)
        out.append((n0, n1, dtheta, c_C))
    return out


def fit_rotation_centre(gp, piv_snaps, tags, min_spread_deg=3.0, spacing=None, tag_rot=(0.0, 0.0)):
    """The rotation centre from ALL pivot snapshots jointly (2026-09-15):
    the lens nadir l_i and pair angle phi_i of snapshot i (pair frame T /
    level camera frame C) satisfy  l_i + R(-phi_i) centre_C = c_T  with
    the centre's position in T (c_T) and the lens->centre vector in the
    camera frame (centre_C) both constant — linear in the four unknowns,
    so one least squares over every snapshot instead of pairing them.
    Returns (centre_C (2,), rms_m, spread_deg) or None when the snapshots
    span less than `min_spread_deg` of body rotation."""
    if spacing:
        poses = [pair_pose_any(gp, d, tags, spacing, tag_rot) for _, d in piv_snaps]
        poses = [p for p in poses if p is not None]
    else:
        poses = [pair_pose_in_cam(gp, d, tags) for _, d in piv_snaps]
    if len(poses) < 2:
        return None
    phis = np.unwrap([p for _, p in poses])
    spread = math.degrees(phis.max() - phis.min())
    if spread < min_spread_deg:
        return None
    A, b = [], []
    for (m, _), phi in zip(poses, phis):
        l = lens_in_pair_frame(m, phi)
        Rm = R2(-phi)
        # l + Rm @ centre_C - c_T = 0  ->  [Rm | -I] [centre_C; c_T] = -l
        A.append(np.hstack([Rm, -np.eye(2)]))
        b.append(-l)
    A, b = np.vstack(A), np.concatenate(b)
    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    rms = math.sqrt(np.mean((A @ x - b) ** 2))
    return x[:2], rms, spread


def fit_yaw_from_tracks(gp, tracks, tags, centre_C, min_step_m=0.0005,
                        phi_smooth_frames=21, spacing=None, tag_rot=(0.0, 0.0)):
    """Camera yaw (rad, robot.yaml sign: the angle of the BODY x axis in the
    level camera frame C) from drive tracks — lists of detection dicts
    recorded frame by frame while the base drove roughly straight.

    The base's ROTATION CENTRE moves only along the body x axis (a
    differential / skid base cannot crab). Per frame the centre's position
    in the pair frame T is c_i = l_i + R(-phi_i) centre_C (lens position
    plus the lens->centre vector, which is fixed in the camera frame), and
    the body heading is theta_i = -phi_i + yaw. So the LATERAL component of
    every centre step, taken in the body frame, must be zero — and yaw is
    the one unknown that rotates all of them. The residual is the
    CUMULATIVE lateral drift along each track rather than per-step
    differences: the step noise is then a random walk while the yaw signal
    grows with distance, which is what makes 0.3 px of corner noise usable
    at 1 mm per frame. Returns (yaw, rms_m, n_frames, per_track_yaw).

    ⚠️ Lateral slip of the rotation centre is indistinguishable from
    camera yaw here (1 mm per 0.1 m = 0.57 deg) — drive slowly, several
    tracks both ways, and read the per-track spread as the uncertainty."""
    from scipy.optimize import least_squares
    tr_steps = []
    n_single = 0
    for tr in tracks:
        if spacing:
            poses = [pair_pose_any(gp, d, tags, spacing, tag_rot) for d in tr]
            n_single += sum(1 for d in tr if len([tg for tg in tags if tg in d]) == 1)
        else:
            poses = [pair_pose_in_cam(gp, d, tags) for d in tr if tags[0] in d and tags[1] in d]
        poses = [p for p in poses if p is not None]
        if len(poses) < 12:
            continue
        # The pair angle carries ~0.04 deg of per-frame noise, which the
        # 0.55 m lens->centre lever turns into 0.4 mm of centre position —
        # more than the yaw signal over a whole track. The body heading
        # changes slowly, so smooth phi along the track (window ~ 21 frames
        # ~ 2 cm) before placing the centre; the lens position itself is
        # left per frame.
        raw_phi = np.unwrap([phi for _, phi in poses])
        w = max(1, min(phi_smooth_frames, len(raw_phi)))
        if w > 1:
            pad = np.pad(raw_phi, (w // 2, w - 1 - w // 2), mode='edge')
            sm_phi = np.convolve(pad, np.ones(w) / w, mode='valid')
        else:
            sm_phi = raw_phi
        cs, phis = [], []
        for (m, phi), phs in zip(poses, sm_phi):
            cs.append(lens_in_pair_frame(m, phi) + R2(-phs) @ centre_C)
            phis.append(phs)
        steps = []
        for k in range(len(cs) - 1):
            dc = cs[k + 1] - cs[k]
            if np.linalg.norm(dc) < min_step_m:
                continue
            steps.append((dc, 0.5 * (phis[k] + phis[k + 1])))
        if len(steps) >= 10:
            tr_steps.append(steps)
    if not tr_steps:
        raise ValueError("no drive track with >= 10 moving frames")

    def resid(x, groups):
        yaw = x[0]
        r = []
        for steps in groups:
            acc = 0.0
            for dc, phi in steps:
                acc += (R2(-yaw) @ (R2(phi) @ dc))[1]
                r.append(acc)
        return np.asarray(r)

    f = least_squares(resid, [0.0], args=(tr_steps,), method='lm')
    yaw = float(f.x[0])
    rms = math.sqrt(np.mean(resid(f.x, tr_steps) ** 2))
    per = [float(least_squares(resid, [0.0], args=([S],), method='lm').x[0]) for S in tr_steps]
    fit_yaw_from_tracks.n_single = n_single      # frames placed from one tag (reported by solve)
    return yaw, rms, sum(len(S) for S in tr_steps), per


def solve_dir(dir_, tags, spacing, size0=DEFAULT_SIZE, h0=0.30):
    """Everything from a session directory. Returns a dict of results."""
    if not spacing or spacing <= 0:
        raise ValueError("spacing (measured centre-to-centre distance, m) is required")
    fx, fy, cx, cy, D = parse_camera_info(os.path.join(dir_, 'camera_info.txt'))
    snaps = load_snapshots(dir_, tags, prefixes=('scan', 'piv'))
    if len(snaps) < 3:
        raise ValueError("need >= 3 scan_/piv_ snapshots with both tags, found %d" % len(snaps))
    F = fit_ground(snaps, tags, fx, fy, cx, cy, D, spacing, size0, h0)
    gp = F['gp']
    res = dict(fit=F, n_snaps=len(snaps))

    # pivot snapshots: the pair is placed from ONE tag when the other has
    # swung out of the (bumper-limited) view, so the pivots can span more
    # than the both-in-view ±6 deg (2026-09-15)
    piv = load_snapshots(dir_, tags, prefixes=('piv',), min_tags=1)
    res['n_piv'] = len(piv)
    res['n_piv_single'] = sum(1 for _, d in piv if len([tg for tg in tags if tg in d]) == 1)
    centres = rotation_centres(gp, piv, tags, spacing=spacing, tag_rot=F['tag_rot'])
    res['centres'] = centres
    joint = fit_rotation_centre(gp, piv, tags, spacing=spacing, tag_rot=F['tag_rot'])
    if centres and joint is not None:
        C = np.array([c for _, _, _, c in centres])
        res['centre_C'] = joint[0]           # the joint solution; the pairwise list is for the sd
        res['centre_C_sd'] = C.std(0)
        res['centre_rms_m'] = joint[1]
        res['centre_spread_deg'] = joint[2]
        res['lever'] = float(np.linalg.norm(joint[0]))
    else:
        centres = res['centres'] = []
    tracks = []
    for f in sorted(glob.glob(os.path.join(dir_, 'drive_*.txt'))):
        tracks.append(parse_track(f))
    res['n_tracks'] = len(tracks)
    if tracks and centres:
        yaw, rms, n, per = fit_yaw_from_tracks(gp, tracks, tags, res['centre_C'],
                                               spacing=spacing, tag_rot=F['tag_rot'])
        res.update(yaw=yaw, yaw_rms_m=rms, yaw_pairs=n, yaw_per_track=per,
                   yaw_single_tag_frames=getattr(fit_yaw_from_tracks, 'n_single', 0))
        res['tx'], res['ty'] = centre_to_lens_body(res['centre_C'], yaw)
    elif centres:
        res['yaw'] = None
        res['tx'], res['ty'] = centre_to_lens_body(res['centre_C'], 0.0)
    return res


def centre_to_lens_body(centre_C, yaw):
    """(tx, ty): the lens relative to the rotation centre in the BODY frame
    (x forward, y LEFT) from the centre's position relative to the lens in
    the level camera frame C (x forward, y RIGHT = image down), given the
    camera yaw (angle of the body x axis in C). Rotate C onto the body x
    axis, flip y to left-handed-positive, negate (centre->lens)."""
    c = R2(-yaw) @ np.asarray(centre_C, dtype=float)
    return float(-c[0]), float(+c[1])


def parse_track(fn):
    """A drive_*.txt file: frames separated by '---' lines."""
    txt = open(fn).read()
    frames = []
    for block in txt.split('\n---'):
        if 'corners' not in block:
            continue
        d = parse_snapshot_text(block)
        if d:
            frames.append(d)
    return frames


def parse_snapshot_text(txt):
    out = {}
    for tid, cx_, cy_, cs in re.findall(
            r"id: (\d+).*?center_x: ([-\d.e]+).*?center_y: ([-\d.e]+).*?corners: \[([^\]]+)\]", txt, re.S):
        out[int(tid)] = dict(cx=float(cx_), cy=float(cy_),
                             c=np.array([float(v) for v in cs.split(',')]).reshape(4, 2))
    return out


def format_detections(msg):
    """AprilTagDetectionArray -> the rostopic-echo-like text fit_front_cam_ground parses."""
    lines = ["header:", "  stamp: %d.%09d" % (msg.header.stamp.secs, msg.header.stamp.nsecs),
             "camera_name: \"%s\"" % msg.camera_name,
             "image_width: %d" % msg.image_width, "image_height: %d" % msg.image_height,
             "detections: "]
    for d in msg.detections:
        lines += ["  - ", "    id: %d" % d.id,
                  "    center_x: %r" % float(d.center_x), "    center_y: %r" % float(d.center_y),
                  "    pose_x: %r" % float(d.pose_x), "    pose_y: %r" % float(d.pose_y),
                  "    pose_z: %r" % float(d.pose_z),
                  "    roll: %r" % float(d.roll), "    pitch: %r" % float(d.pitch), "    yaw: %r" % float(d.yaw),
                  "    corners: [%s]" % ", ".join("%r" % float(v) for v in d.corners),
                  "    tilt_from_normal: %r" % float(d.tilt_from_normal)]
    return "\n".join(lines) + "\n"


def average_detections(frames, tags):
    """A copy of the last frame whose tag corners / centres are the mean
    over all frames (only the calibration tags are averaged)."""
    import copy
    out = copy.deepcopy(frames[-1])
    for d in out.detections:
        if d.id not in tags:
            continue
        cs = [np.asarray([x for x in f.detections if x.id == d.id][0].corners, float)
              for f in frames if any(x.id == d.id for x in f.detections)]
        mean = np.mean(cs, axis=0)
        d.corners = [float(v) for v in mean]
        d.center_x = float(np.mean(mean[0::2]))
        d.center_y = float(np.mean(mean[1::2]))
    return out


def format_camera_info(ci):
    return ("width: %d\nheight: %d\ndistortion_model: %s\nD: [%s]\nK: [%s]\n"
            % (ci.width, ci.height, ci.distortion_model,
               ", ".join("%r" % float(v) for v in ci.D), ", ".join("%r" % float(v) for v in ci.K)))


# =====================================================================
# ROS side
# =====================================================================
class Session:
    def __init__(self, tags, settle_s=1.5, left_edge_px=DEFAULT_LEFT_EDGE_PX):
        import rospy
        from robot_msgs.msg import AprilTagDetectionArray
        from sensor_msgs.msg import CameraInfo
        self.rospy = rospy
        self.tags = tuple(tags)
        self.settle_s = settle_s
        self.left_edge_px = int(left_edge_px)
        self._last = None
        self._recording = None
        self._ci = None
        cfg = yaml.safe_load(open(CONFIG_PATH))
        gp = ((cfg.get('robot_camera') or {}).get('ground_plane') or {}).get('front_cam') or {}
        self.gp_enabled_cfg = bool(gp.get('enabled', False))
        self.gp_height_cfg = float(gp.get('height_m', 0.30))
        rospy.Subscriber('/front_cam/tag_detections', AprilTagDetectionArray, self._cb, queue_size=1)
        rospy.Subscriber('/front_cam/color/camera_info', CameraInfo, self._ci_cb, queue_size=1)

    def _cb(self, msg):
        self._last = msg
        if self._recording is not None:
            self._recording.append(msg)

    def _ci_cb(self, msg):
        self._ci = msg

    def is_corrected(self, msg):
        """robot_camera_node with the ground-plane correction ON pins every
        pose_z to the configured lens height — raw detections carry the
        detector's own depth, which varies tag to tag and frame to frame."""
        zs = [float(d.pose_z) for d in msg.detections]
        return bool(zs) and all(abs(z - self.gp_height_cfg) < 1e-6 for z in zs)

    def wait_pair(self, timeout=5.0):
        return self.wait_tags(len(self.tags), timeout)

    def wait_tags(self, min_tags, timeout=5.0):
        """A fresh detection message showing at least `min_tags` of the
        calibration tags (2 = the pair, 1 = either, for drive tracks)."""
        t0 = time.time()
        while time.time() - t0 < timeout and not self.rospy.is_shutdown():
            m = self._last
            if m is not None and (time.time() - m.header.stamp.to_sec()) < 0.5:
                ids = {d.id for d in m.detections}
                if len(set(self.tags) & ids) >= min_tags:
                    return m
            self.rospy.sleep(0.05)
        return None

    def check(self):
        print("robot.yaml ground_plane.front_cam.enabled: %s" % self.gp_enabled_cfg)
        m = self.wait_pair()
        if m is None:
            print("FAIL: no detection with both tags %s within 5 s" % (self.tags,))
            return False
        if self.is_corrected(m):
            print("FAIL: detections are ground-plane CORRECTED (pose_z pinned to %.3f). "
                  "Set robot.yaml robot_camera.ground_plane.front_cam.enabled: false and "
                  "restart robot_camera_node (rosnode kill /robot_camera_node && rosrun "
                  "apriltag_nav robot_camera_node.py)." % self.gp_height_cfg)
            return False
        if self._ci is None:
            print("FAIL: no CameraInfo on /front_cam/color/camera_info")
            return False
        for d in m.detections:
            if d.id in self.tags:
                print("  tag %d at (%.0f, %.0f) px, corners span %.0f px" % (
                    d.id, d.center_x, d.center_y,
                    max(d.corners[0::2]) - min(d.corners[0::2])))
        room = self.room(m)
        room1 = self.room(m, keep='one')
        xs = [d.center_x for d in m.detections if d.id in self.tags]
        usable_mid = 0.5 * (self.left_edge_px + m.image_width)
        print("  usable image x %d..%d px (bumper hides the left %d px); pair centre %.0f px, "
              "%+.0f mm from the usable middle (+ = ahead; drive FORWARD to centre)"
              % (self.left_edge_px, m.image_width, self.left_edge_px, np.mean(xs),
                 (np.mean(xs) - usable_mid) * self.gp_height_cfg / (float(self._ci.K[0]) if self._ci else 910.0) * 1e3))
        print("  room with BOTH tags in view (snapshots): forward %.3f m, reverse %.3f m, left %.3f m, right %.3f m"
              % (room['fwd'], room['rev'], room['left'], room['right']))
        print("  room with ONE tag in view (drive tracks): forward %.3f m, reverse %.3f m" % (room1['fwd'], room1['rev']))
        d_ok = min(room1['fwd'], room1['rev']) - ROOM_MARGIN_M
        print("  -> collect can drive %.3f m tracks; scans need ~0.04 each way with both tags, "
              "the pivots swing the lens ~0.05 m sideways" % max(0.0, d_ok))
        if min(room['fwd'], room['rev']) < 0.03 or min(room['left'], room['right']) < 0.05:
            print("FAIL: too little room — re-park with the pair in the middle of the USABLE image "
                  "(the offset printed above near 0), or lay the tags closer together")
            return False
        print("OK: raw detections, both tags, CameraInfo present")
        return True

    def room(self, msg, keep='both'):
        """frame_room_m for a live detection message (lens height from the
        config, K from CameraInfo, the bumper edge from --left-edge-px)."""
        det = {d.id: dict(c=np.asarray(d.corners, float).reshape(4, 2)) for d in msg.detections}
        fx = float(self._ci.K[0]) if self._ci is not None else 910.0
        fy = float(self._ci.K[4]) if self._ci is not None else fx
        return frame_room_m(det, self.tags, fx, fy, self.gp_height_cfg, msg.image_width, msg.image_height,
                            x_min_px=self.left_edge_px, keep=keep)

    def tag_angles_deg(self, timeout=2.0):
        """{id: square orientation in the RAW image (deg)} for every
        calibration tag in view, each a 30-frame mean of the four-edge
        angle (square_angle). The body's rotation between two at-rest
        readings is the change per tag (image sign; the pivot loop learns
        the sign with the gain), so the feedback survives one tag
        swinging out of view. {} if none is seen."""
        acc = {tg: [] for tg in self.tags}
        t0, last, n = time.time(), None, 0
        while n < 30 and time.time() - t0 < 4.0 and not self.rospy.is_shutdown():
            m = self.wait_tags(1, timeout=1.0)
            if m is None:
                break
            if m.header.stamp != last:
                last = m.header.stamp
                n += 1
                for d in m.detections:
                    if d.id in acc:
                        acc[d.id].append(square_angle(np.asarray(d.corners, float).reshape(4, 2)))
            self.rospy.sleep(0.01)
        out = {}
        for tg, vals in acc.items():
            if len(vals) >= 5:
                out[tg] = math.degrees(math.atan2(np.mean(np.sin(vals)), np.mean(np.cos(vals))))
        return out

    def rotation_since(self, ref):
        """Body rotation (deg, image sign) since the reference reading
        `ref` (a tag_angles_deg dict), averaged over the tags visible in
        both; None when no tag is common."""
        now = self.tag_angles_deg()
        ds = [(now[tg] - ref[tg] + 180.0) % 360.0 - 180.0 for tg in ref if tg in now]
        if not ds:
            return None
        return float(np.mean(ds))

    def max_pivot_deg(self, camera_offset, spacing, keep='both', direction=0):
        """The largest body rotation (deg) from the CURRENT pose that keeps
        both tags (keep='both') or at least one tag (keep='one') inside
        the lateral room: a tag is camera_offset + (its distance ahead of
        the nadir) from the rotation centre and swings sideways by that
        times the angle. None when no tag is in view."""
        m = self.wait_tags(1 if keep == 'one' else len(self.tags), timeout=2.0)
        if m is None:
            return None
        fx = float(self._ci.K[0]) if self._ci is not None else 910.0
        fy = float(self._ci.K[4]) if self._ci is not None else fx
        cxK = float(self._ci.K[2]) if self._ci is not None else 0.5 * m.image_width
        det = {d.id: dict(c=np.asarray(d.corners, float).reshape(4, 2)) for d in m.detections}
        return max_pivot_from_corners(det, self.tags, camera_offset, fx, fy, cxK, self.gp_height_cfg,
                                      m.image_height, keep=keep, direction=direction)

    def lateral_offset_deg(self, camera_offset):
        """Body rotation (deg, image-angle sign) that would bring the
        visible tags' mean row onto the principal row: + moves the tags
        DOWN the image. None when no tag is seen."""
        m = self.wait_tags(1, timeout=2.0)
        if m is None:
            return None
        fx = float(self._ci.K[0]) if self._ci is not None else 910.0
        fy = float(self._ci.K[4]) if self._ci is not None else fx
        cxK = float(self._ci.K[2]) if self._ci is not None else 0.5 * m.image_width
        cyK = float(self._ci.K[5]) if self._ci is not None else 0.5 * m.image_height
        rows, levers = [], []
        for d in m.detections:
            if d.id in self.tags:
                rows.append(d.center_y)
                levers.append(abs(camera_offset) + max(0.0, (d.center_x - cxK) * self.gp_height_cfg / fx))
        dy = (cyK - np.mean(rows)) * self.gp_height_cfg / fy
        return math.degrees(math.asin(max(-1.0, min(1.0, dy / np.mean(levers)))))

    def recentre(self):
        """Drive distance that recentres the pair in the usable image, or
        None when the pair is not in view."""
        m = self.wait_pair(timeout=2.0)
        if m is None:
            return None
        det = {d.id: dict(c=np.asarray(d.corners, float).reshape(4, 2)) for d in m.detections}
        fx = float(self._ci.K[0]) if self._ci is not None else 910.0
        return recentre_m(det, self.tags, fx, self.gp_height_cfg, m.image_width, self.left_edge_px)

    def capped(self, dist, keep='both'):
        """`dist` clipped to the room the CURRENT frame leaves (None when
        no usable frame arrives). keep='one' for drive tracks: a single
        tag in view is enough to plan the next track."""
        m = self.wait_tags(1 if keep == 'one' else len(self.tags), timeout=2.0)
        if m is None:
            return None, False
        return cap_distance(dist, self.room(m, keep=keep))

    def snap(self, dir_, name, n_frames=30, min_tags=None):
        """One at-rest snapshot = the corner-wise MEAN of n_frames frames
        (0.3 px single-frame noise cost the 2026-09-08 single-frame fit
        ~0.07 deg of roll / 2 mm of lever in the synthetic check; 30
        frames — one second — bring tx under 0.6 mm worst case over eight
        noise seeds, where 15 frames left 1.5 mm). min_tags 1 accepts a
        single-tag snapshot (pivots)."""
        frames = []
        t0 = time.time()
        last_stamp = None
        need = len(self.tags) if min_tags is None else min_tags
        while len(frames) < n_frames and time.time() - t0 < 5.0 and not self.rospy.is_shutdown():
            m = self.wait_tags(need, timeout=1.0)
            if m is None:
                break
            if m.header.stamp != last_stamp:
                last_stamp = m.header.stamp
                frames.append(m)
            self.rospy.sleep(0.01)
        if not frames:
            raise RuntimeError("snapshot %s: %s in view" % (name, "no tag" if need == 1 else "both tags not"))
        # keep only the frames showing the same tag set as the last one
        ids_last = {d.id for d in frames[-1].detections} & set(self.tags)
        frames = [f for f in frames if ({d.id for d in f.detections} & set(self.tags)) == ids_last]
        m = frames[-1]
        if self.is_corrected(m):
            raise RuntimeError("detections are ground-plane corrected — refuse (see check)")
        m = average_detections(frames, self.tags)
        os.makedirs(dir_, exist_ok=True)
        ci_path = os.path.join(dir_, 'camera_info.txt')
        if not os.path.exists(ci_path) and self._ci is not None:
            open(ci_path, 'w').write(format_camera_info(self._ci))
        fn = os.path.join(dir_, name if name.endswith('.txt') else name + '.txt')
        open(fn, 'w').write(format_detections(m))
        return fn

    def record(self, dir_, name, seconds=None, until=None):
        """Record every frame for `seconds`, or while until() is False."""
        os.makedirs(dir_, exist_ok=True)
        self._recording = []
        t0 = time.time()
        while not self.rospy.is_shutdown():
            if seconds is not None and time.time() - t0 >= seconds:
                break
            if until is not None and until():
                break
            self.rospy.sleep(0.02)
        frames, self._recording = self._recording, None
        fn = os.path.join(dir_, name if name.endswith('.txt') else name + '.txt')
        with open(fn, 'w') as fh:
            for m in frames:
                fh.write(format_detections(m) + "---\n")
        good = sum(1 for m in frames if set(self.tags) <= {d.id for d in m.detections})
        return fn, len(frames), good


def cmd_collect(args):
    import rospy
    import threading
    from apriltag_nav.mobile_client import MobileClient
    rospy.init_node('calib_front_cam_pose', anonymous=True)
    S = Session(args.tags, left_edge_px=args.left_edge_px)
    rospy.sleep(1.0)
    if not S.check():
        sys.exit(2)
    d = args.dir
    only = set(args.only.split(',')) if args.only else {'scan', 'piv', 'drive'}
    if not only <= {'scan', 'piv', 'drive'}:
        sys.exit("--only takes a comma list of scan, piv, drive")
    existing = glob.glob(os.path.join(d, '*.txt'))
    if existing and not args.only:
        sys.exit("refusing to write into a non-empty session directory: %s "
                 "(use --only piv|scan|drive to redo one phase in place)" % d)
    scans = [+0.02, +0.02, +0.02, -0.02, -0.02, -0.04, -0.02, +0.04]
    # EXECUTED body-rotation targets (deg, measured on the tags, not
    # commanded): 3 deg steps summing to +6 / 0 / -6 / 0, nine snapshots
    # spanning ~12 deg for the joint centre fit. The base executes a
    # commanded 2 deg pivot as 0.3-0.8 deg (2026-09-15 session) and 5 deg
    # as ~3.7, so each step is a closed loop: command, read the pair
    # angle at rest, learn the gain, repeat until inside pivot_tol. Each
    # command is capped by the lateral frame room of the moment.
    # CUMULATIVE executed targets vs the first pivot snapshot, in units of
    # the largest rotation the frame allows with ONE tag still in view —
    # decided at run time from the room (`pivot_full`, ≤ --pivot-max deg).
    pivs_unit = [+0.5, +1.0, +0.5, 0.0, -0.5, -1.0, -0.5, 0.0]
    pivot_tol = 0.5
    drives = []
    for _ in range(args.drive_repeat):
        drives += [+args.drive, -args.drive]
    print("\nPlan (every move through mobile_node, odometry-closed, ≤ %.2f m / %.0f deg; "
          "each move is capped to the frame room of the moment):" %
          (max(abs(x) for x in scans + [args.drive]), args.pivot_max))
    if 'scan' in only:
        print("  1. snapshot at rest, then %d moves of %s m with a snapshot after each" % (len(scans), scans))
    if 'piv' in only:
        print("  2. %d closed-loop pivots to the cumulative EXECUTED angles %s x F deg (±%.1f) with a snapshot after "
              "each, F = the largest rotation that keeps %s in view (≤ %.0f deg, read from the frame), at %.2f rad/s"
              % (len(pivs_unit), pivs_unit, pivot_tol, "ONE tag" if args.pivot_one_tag else "BOTH tags",
                 args.pivot_max, args.pivot_speed))
    if 'drive' in only:
        print("  3. %d drives of %s m, detections recorded throughout" % (len(drives), drives))
    print("  output: %s%s" % (d, "  (only: %s; that phase's old files are renamed old_*)" % ",".join(sorted(only)) if args.only else ""))
    if args.dry_run:
        return
    if not args.yes:
        ans = input("Hand on the e-stop? The base will move now. Type 'go' to start: ")
        if ans.strip().lower() != 'go':
            sys.exit("aborted")
    mc = MobileClient()
    ok, why = mc.wait_for_node(10.0)
    if not ok:
        sys.exit("mobile_node: %s" % why)

    def snap(name, min_tags=None):
        rospy.sleep(S.settle_s)
        fn = S.snap(d, name, min_tags=min_tags)
        print("  saved %s" % os.path.basename(fn))

    def capped_or_exit(dist, what, keep='both'):
        c, was = S.capped(dist, keep=keep)
        if c is None:
            sys.exit("%s: %s in view — stop here, solve what exists"
                     % (what, "no calibration tag" if keep == 'one' else "pair not"))
        if was:
            print("  %s: %+.3f m capped to %+.3f m by the frame room" % (what, dist, c))
        return c

    if args.only:
        for ph in only:
            for f in glob.glob(os.path.join(d, ph + '_*.txt')):
                os.rename(f, os.path.join(d, 'old_' + os.path.basename(f)))
    cfg = yaml.safe_load(open(CONFIG_PATH))
    cam_off_prior = float(cfg['robot'].get('camera_offset', 0.55))

    if 'scan' in only:
        snap('scan_%s' % time.strftime('%H%M%S'))
        for k, dist in enumerate(scans):
            dist = capped_or_exit(dist, "move %d" % k)
            if abs(dist) < 0.008:
                print("  move %d skipped (no room)" % k)
                continue
            if not mc.drive_distance(dist, speed=args.speed):
                sys.exit("move %d (%+.3f m) failed — stop here, solve what exists" % (k, dist))
            snap('scan_%s' % time.strftime('%H%M%S'))

    def recentre(what):
        # the scans / drives leave the pair wherever the caps allowed; the
        # pivots and each drive phase start with it in the middle of the
        # usable image so the swing / the track has room both ways
        r = S.recentre()
        if r is None:
            sys.exit("%s: pair not in view — stop here, solve what exists" % what)
        if abs(r) >= 0.008:
            print("  %s: recentring the pair, %+.3f m" % (what, r))
            if not mc.drive_distance(r, speed=args.speed):
                sys.exit("%s: recentre move failed — stop here, solve what exists" % what)
            rospy.sleep(S.settle_s)

    keep = 'both'

    def pivot_to(target, gain, ref):
        """Pivot until the tags' square angles are `target` deg away from
        the reference reading `ref` (image sign convention; cumulative
        targets so per-step errors do not add up; per-tag reference, so
        one tag out of view is fine). Returns (reached, gain) — gain =
        commanded / executed, learned from every attempt with a measurable
        result."""
        total = S.rotation_since(ref)
        if total is None:
            sys.exit("pivot: no calibration tag in view — stop here, solve what exists")
        stalled = 0
        for attempt in range(6):
            rem = target - total
            if abs(rem) <= pivot_tol:
                break
            # the cap keeps ONE tag in view in the direction this rotation
            # moves the tags (rem is in image-angle terms: + = down the image)
            cap = S.max_pivot_deg(cam_off_prior, args.spacing or 0.12, keep=keep,
                                  direction=1 if rem > 0 else -1)
            if cap is None:
                sys.exit("pivot: no calibration tag in view — stop here, solve what exists")
            cmd = rem * gain
            if abs(rem) > cap:
                print("    remaining %+.2f deg exceeds the lateral room (%.2f deg) — stopping this step short" % (rem, cap))
                cmd = math.copysign(cap * abs(gain), cmd)
            cmd = max(-args.pivot_max * 2.5, min(args.pivot_max * 2.5, cmd))
            if abs(cmd) < 0.3:
                break
            # mobile_node verifies the pivot against ODOM and reports a move
            # that ended more than shortfall_frac short as a failure — the
            # base under-executes small pivots, so that is the normal case
            # here. What matters is what the TAGS say: a short move is
            # progress, only "nothing moved twice" stops the session.
            ok = mc.pivot_angle(cmd, speed=args.pivot_speed)
            rospy.sleep(S.settle_s)
            ex = S.rotation_since(ref)
            if ex is None:
                sys.exit("pivot: tags lost after a command — stop here, solve what exists")
            step = (ex - total + 180.0) % 360.0 - 180.0
            total = ex
            if abs(step) > 0.15:
                gain = max(-6.0, min(6.0, cmd / step))
                stalled = 0
            else:
                gain *= 1.5              # nothing moved: push harder
                stalled += 1
                if stalled >= 2:
                    sys.exit("pivot: two commands in a row moved the tags by < 0.15 deg%s — stop here, "
                             "solve what exists" % ("" if ok else " (mobile_node reported the command failed)"))
            if not ok:
                print("    (mobile_node reported the pivot short of its odom target — using the tag reading)")
            print("    attempt %d: commanded %+.2f -> executed %+.2f deg (total %+.2f / %+.2f, gain %.2f)"
                  % (attempt + 1, cmd, step, total, target, gain))
            if abs(rem) > cap:
                break
        return total, gain

    def level_pivot():
        """Pivot until the tags sit on the principal row (a previous
        sweep may have left them at an edge, one even out of view)."""
        gain = 1.8
        for attempt in range(5):
            need = S.lateral_offset_deg(cam_off_prior)
            if need is None:
                sys.exit("pivot: no calibration tag in view — re-park with the pair in view")
            if abs(need) < 0.4:
                return
            lim = 3.0 if attempt == 0 else args.pivot_max * 2.5   # first: small, until the sign is learned
            cmd = max(-lim, min(lim, need * gain))
            print("  levelling the pair on the principal row: %+.2f deg needed, commanding %+.2f" % (need, cmd))
            before = S.tag_angles_deg()
            mc.pivot_angle(cmd, speed=args.pivot_speed)
            rospy.sleep(S.settle_s)
            ex = S.rotation_since(before)
            if ex is not None and abs(ex) > 0.15:
                gain = max(-6.0, min(6.0, cmd / ex))

    if 'piv' in only:
        level_pivot()
        recentre("before the pivots")
        snap('piv_00_%s' % time.strftime('%H%M%S'))
        ref = S.tag_angles_deg()
        if len(ref) < 2:
            sys.exit("pivot: pair not in view — stop here, solve what exists")
        keep = 'one' if args.pivot_one_tag else 'both'
        full = S.max_pivot_deg(cam_off_prior, args.spacing or 0.12, keep=keep)
        full = min(args.pivot_max, (full or 0.0) - 0.7)      # the loop lands within ±0.5 and overshoots a little
        if full < 3.0:
            sys.exit("pivot: the lateral room allows only %.1f deg of rotation — re-park with the pair "
                     "vertically centred (overlay y offsets near 0)" % full)
        pivs = [u * full for u in pivs_unit]
        print("  pivot targets (cumulative, executed): %s deg" % ", ".join("%+.1f" % v for v in pivs))
        gain = 1.8                                   # the base executes ~40-75 % of a 3-6 deg command
        for k, ang in enumerate(pivs):
            reached, gain = pivot_to(ang, gain, ref)
            print("  pivot %d: cumulative target %+.1f, at %+.2f deg" % (k + 1, ang, reached))
            snap('piv_%02d_%s' % (k + 1, time.strftime('%H%M%S')), min_tags=1 if args.pivot_one_tag else None)
    if 'drive' not in only:
        print("collect done: %s" % d)
        return
    recentre("before the drives")
    for k, dist in enumerate(drives):
        dist = capped_or_exit(dist, "drive %d" % k, keep='one')
        if abs(dist) < 0.03:
            print("  drive %d skipped (%.3f m of room is too short for a track)" % (k, abs(dist)))
            continue
        done = {'v': False}

        def runner():
            done['ok'] = mc.drive_distance(dist, speed=args.speed)
            done['v'] = True
        th = threading.Thread(target=runner, daemon=True)
        th.start()
        fn, n, good = S.record(d, 'drive_%s' % time.strftime('%H%M%S'), until=lambda: done['v'])
        th.join()
        print("  drive %+.3f m: %s, %d frames (%d with both tags) -> %s" %
              (dist, 'ok' if done.get('ok') else 'FAILED', n, good, os.path.basename(fn)))
        if not done.get('ok'):
            sys.exit("drive failed — stop here, solve what exists")
        rospy.sleep(S.settle_s)
    print("collect done: %s" % d)


def cmd_snap(args):
    import rospy
    rospy.init_node('calib_front_cam_pose', anonymous=True)
    S = Session(args.tags)
    rospy.sleep(1.0)
    print("saved", S.snap(args.dir, args.name))


def cmd_record(args):
    import rospy
    rospy.init_node('calib_front_cam_pose', anonymous=True)
    S = Session(args.tags, left_edge_px=args.left_edge_px)
    rospy.sleep(1.0)
    if not S.check():
        sys.exit(2)
    name = args.name if args.name.startswith('drive_') else 'drive_' + args.name
    print("recording %.1f s — drive now (straight, ≤ 0.1 m)" % args.seconds)
    fn, n, good = S.record(args.dir, name, seconds=args.seconds)
    print("saved %s: %d frames, %d with both tags" % (fn, n, good))


def cmd_check(args):
    import rospy
    rospy.init_node('calib_front_cam_pose', anonymous=True)
    S = Session(args.tags, left_edge_px=args.left_edge_px)
    rospy.sleep(1.0)
    sys.exit(0 if S.check() else 2)


def cmd_solve(args):
    if not args.spacing:
        sys.exit("solve needs --spacing <measured centre-to-centre distance in m> "
                 "(= (outer extent + inner gap) / 2 of the laid pair); it is the scale reference")
    r = solve_dir(args.dir, tuple(args.tags), args.spacing, args.size)
    F = r['fit']
    print("tag-pair fit: %d snapshots, rms %.3f px (max %.2f; level camera %.3f px)"
          % (r['n_snaps'], F['rms_px'], F['max_px'], F['rms_level_px']))
    print("  roll %+.3f deg  pitch %+.3f deg  lens height %.1f mm above the tag top  tag size %.2f mm"
          % (math.degrees(F['roll']), math.degrees(F['pitch']), F['h'] * 1e3, F['size'] * 1e3))
    print("  tags' in-plane angle vs the centre line: %d %+.2f deg, %d %+.2f deg"
          % (args.tags[0], math.degrees(F['tag_rot'][0]), args.tags[1], math.degrees(F['tag_rot'][1])))
    if abs(F['size'] - args.size) > 0.004:
        print("  ⚠️ fitted tag size is %.1f mm off the nominal %.0f mm — check --spacing / --size" %
              ((F['size'] - args.size) * 1e3, args.size * 1e3))
    if not r['centres']:
        print("pivot snapshots span too little body rotation for the centre fit (need >= 3 deg; "
              "redo them: collect DIR --only piv) — tx/ty not determined")
        return
    print("rotation centre relative to the lens nadir, level camera frame (x fwd, y right), per pivot pair "
          "(%d pivot snapshots, %d of them from a single tag):" % (r['n_piv'], r['n_piv_single']))
    for n0, n1, dth, c in r['centres']:
        print("  %s -> %s: %+5.2f deg  (%+7.1f, %+6.1f) mm" % (n0, n1, math.degrees(dth), c[0] * 1e3, c[1] * 1e3))
    cC, sd = r['centre_C'], r['centre_C_sd']
    print("  joint fit over all pivot snapshots (%.1f deg of spread, rms %.2f mm): (%+.1f, %+.1f) mm, "
          "pairwise sd (%.1f, %.1f) mm, lever %.4f m"
          % (r['centre_spread_deg'], r['centre_rms_m'] * 1e3, cC[0] * 1e3, cC[1] * 1e3,
             sd[0] * 1e3, sd[1] * 1e3, r['lever']))
    if r.get('yaw') is None:
        print("no drive_*.txt tracks — yaw not determined (tx/ty below assume yaw 0)")
    else:
        print("camera yaw vs the travel axis: %+.3f deg (cumulative-lateral rms %.2f mm over %d steps, "
              "%d frames placed from a single tag; per track: %s)"
              % (math.degrees(r['yaw']), r['yaw_rms_m'] * 1e3, r['yaw_pairs'], r.get('yaw_single_tag_frames', 0),
                 ", ".join("%+.3f" % math.degrees(v) for v in r['yaw_per_track']) or "-"))
    print("\nlens relative to the ROTATION CENTRE, body frame: tx %+.4f m  ty %+.4f m" % (r['tx'], r['ty']))
    yaw_deg = math.degrees(r['yaw']) if r.get('yaw') is not None else None
    print("\nrobot.yaml:")
    print("  robot:\n    camera_offset: %.3f\n    camera_lateral: %.3f" % (r['tx'], r['ty']))
    print("  robot_camera:\n    ground_plane:\n      front_cam:\n        roll_deg: %.3f\n        pitch_deg: %.3f\n        height_m: %.3f\n        yaw_deg: %s"
          % (math.degrees(F['roll']), math.degrees(F['pitch']), F['h'],
             ("%.3f" % yaw_deg) if yaw_deg is not None else "<unchanged: no drive tracks>"))
    if args.apply:
        apply_to_robot_yaml(r, yaw_deg)


def apply_to_robot_yaml(r, yaw_deg, path=CONFIG_PATH):
    F = r['fit']
    txt = open(path).read()

    def sub_line(txt, indent, key, value, block=None):
        pat = re.compile(r"^(%s%s:)[ \t]*[-+\d.e]+" % (indent, key), re.M)
        if block is not None:
            i = txt.index(block)
            m = pat.search(txt, i)
            if m is None or m.start() - i > 800:
                raise ValueError("%s not found inside %r" % (key, block))
        else:
            m = pat.search(txt)
            if m is None:
                raise ValueError("%s not found" % key)
        return txt[:m.start()] + m.group(1) + " " + value + txt[m.end():]

    txt = sub_line(txt, "  ", "camera_offset", "%.3f" % r['tx'])
    txt = sub_line(txt, "  ", "camera_lateral", "%.3f" % r['ty'])
    blk = "  ground_plane:\n    front_cam:\n"
    txt = sub_line(txt, "      ", "roll_deg", "%.3f" % math.degrees(F['roll']), blk)
    txt = sub_line(txt, "      ", "pitch_deg", "%.3f" % math.degrees(F['pitch']), blk)
    txt = sub_line(txt, "      ", "height_m", "%.3f" % F['h'], blk)
    if yaw_deg is not None:
        txt = sub_line(txt, "      ", "yaw_deg", "%.3f" % yaw_deg, blk)
    open(path, 'w').write(txt)
    cfg = yaml.safe_load(open(path))
    gp = cfg['robot_camera']['ground_plane']['front_cam']
    print("\nrobot.yaml written: camera_offset %.3f camera_lateral %.3f roll %.3f pitch %.3f height %.3f yaw %.3f"
          % (cfg['robot']['camera_offset'], cfg['robot']['camera_lateral'],
             gp['roll_deg'], gp['pitch_deg'], gp['height_m'], gp['yaw_deg']))
    gen = os.path.join(WS_DIR, 'src', 'apriltag_nav', 'tools', 'tf_chain_tool.py')
    subprocess.check_call([sys.executable, gen, 'front-cam', '--apply'])
    print("\nNext: set ground_plane.front_cam.enabled back to true, restart robot_camera_node, "
          "mobile_node (camera_offset) and the calibration nodes (tf_chain.yaml T_mb2fc). T_mb2fc tz = height_m + "
          "robot.tag_thickness (%.3f)." % float(cfg['robot'].get('tag_thickness', 0.0)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tags', type=int, nargs=2, default=list(DEFAULT_TAGS),
                    help='tag ids, first -> second along the robot FORWARD axis (default 149 150)')
    ap.add_argument('--spacing', type=float, default=DEFAULT_SPACING,
                    help='measured centre-to-centre distance (m); required by solve')
    ap.add_argument('--size', type=float, default=DEFAULT_SIZE, help='nominal printed tag size (m), refined by the fit')
    ap.add_argument('--left-edge-px', type=int, default=DEFAULT_LEFT_EDGE_PX,
                    help='first usable image column: the body / bumper hides everything left of it (default 430)')
    sp = ap.add_subparsers(dest='cmd', required=True)
    sp.add_parser('check').set_defaults(fn=cmd_check)
    p = sp.add_parser('snap'); p.add_argument('dir'); p.add_argument('name'); p.set_defaults(fn=cmd_snap)
    p = sp.add_parser('record'); p.add_argument('dir'); p.add_argument('name')
    p.add_argument('--seconds', type=float, default=15.0); p.set_defaults(fn=cmd_record)
    p = sp.add_parser('collect'); p.add_argument('dir')
    p.add_argument('--drive', type=float, default=0.15,
                   help='straight drive length per track (m), capped to the room in which ONE tag stays in view')
    p.add_argument('--drive-repeat', type=int, default=4, help='forward+back pairs (8 tracks)')
    p.add_argument('--speed', type=float, default=0.03)
    p.add_argument('--dry-run', action='store_true'); p.add_argument('--yes', action='store_true')
    p.add_argument('--pivot-max', type=float, default=12.0,
                   help='largest cumulative executed pivot (deg); the frame room may allow less')
    p.add_argument('--pivot-one-tag', action='store_true',
                   help='size the sweep so only ONE tag has to stay in view (~+25 %% of spread; the far tag '
                        'leaves at the extremes and those snapshots are single-tag)')
    p.add_argument('--pivot-speed', type=float, default=0.08,
                   help='angular speed of the pivot commands (rad/s; mobile_node default 0.2 was too fast, user 2026-09-15)')
    p.add_argument('--only', default=None,
                   help='redo only these phases (comma list of scan,piv,drive) INTO an existing session dir; '
                        'that phase\'s previous files are renamed old_*')
    p.set_defaults(fn=cmd_collect)
    p = sp.add_parser('solve'); p.add_argument('dir'); p.add_argument('--apply', action='store_true')
    p.set_defaults(fn=cmd_solve)
    a = ap.parse_args()
    a.fn(a)


if __name__ == '__main__':
    main()
