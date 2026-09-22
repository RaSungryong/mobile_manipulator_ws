#!/usr/bin/env python3
"""
check_align_fixed_orientation.py — offline checks for the align loop's
orientation policy (2026-09-22). No ROS master needed.

Run from the workspace root (any sourced shell):
    python3 src/path_tag_locator/scripts/check_align_fixed_orientation.py

What it pins:
  * compute_target_ee_pose(fix_orientation=True): the target's rotation
    is the CURRENT rotation bit for bit; after the move the tag sits on
    the optical axis at the target range with the camera-frame rotation
    unchanged (the residual tilt is untouched); with a square camera a
    pure range error moves the tool along the tag normal only.
  * clamp_step of such a target commands exactly zero rotation.
  * is_converged: check_tilt=False ignores the tilt; the depth gate.
  * run_auto_align (real driver, fake arm + fake detector, real
    T_hc2ee) with align.orientation 'fixed': from a seed 1.5 deg off the
    tag normal, 40 mm off-centre and 0.63 m up, the arm ends with the
    seed's rx/ry/rz unchanged (< 1e-9 deg), the tag centred within
    position_tol_m, the range at target_distance_m within depth_tol_m,
    every commanded move a pure translation, the tilt recorded at
    ~1.5 deg in every history entry and reported, and the report says
    'fixed'. A 40 deg spin (rz) seed is kept. A 4 deg tilt raises the
    warning and still converges.
  * fixed_rx_deg -180 / fixed_ry_deg 0 (the user's rule, later the same
    day), fixed_rpy_frame 'camera' (default; hand-eye applied): from a
    plan-like seed (rx -179.5, ry -0.3, rz 25) the first step turns the
    CAMERA optical frame onto Rz(spin)·Rx(-180) — axis straight down
    arm z, its spin kept — with the flange following through T_hc2ee
    (flange reads -179.86 / +0.45 on the applied file), every later
    step is a pure translation, the tag is centred at 0.50 m, and the
    recorded tilt is exactly the tag's slope vs the arm vertical (flat
    -> 0.000, 0.7 planted -> 0.700), constant over the iterations.
    'flange': the flange itself reads |rx| 180 / ry 0 / rz 25 to 1e-9
    deg and a flat tag then shows the hand-eye's 0.47 deg axis offset.
    The approach (initial move) is NOT held to it.
  * The same seed with 'correct' still squares the camera (tilt < tol,
    rotation changed) — the hand-eye sweep's behaviour is intact.
  * AlignCfg refuses an unknown orientation; the real locator.yaml
    loads with 'fixed' / 0.50 m and the design view distance agrees.
"""
import math
import os
import sys
import types

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, 'src'))

try:
    import rospy  # noqa: F401
except ImportError:
    _r = types.ModuleType("rospy")
    for _f in ("loginfo", "logwarn", "logerr", "logwarn_once", "sleep"):
        setattr(_r, _f, lambda *a, **k: None)
    _r.is_shutdown = lambda: False
    sys.modules["rospy"] = _r
    _m = types.ModuleType("robot_msgs")
    _mm = types.ModuleType("robot_msgs.msg")
    _mm.AprilTagDetectionArray = object
    _m.msg = _mm
    sys.modules["robot_msgs"] = _m
    sys.modules["robot_msgs.msg"] = _mm

import rospy  # noqa: E402
from path_tag_locator import align_runner                                  # noqa: E402
from path_tag_locator.align import (alignment_metrics, clamp_step,          # noqa: E402
                                    compute_target_ee_pose, is_converged,
                                    fixed_orientation_R)
from path_tag_locator.constants import AlignCfg, load_locator_cfg_from_dict  # noqa: E402
from path_tag_locator.geometry import (invert_T, matrix_m_to_pose_fr5,      # noqa: E402
                                        pose_fr5_to_matrix_m, rpy_deg_to_R)

N_OK = N_FAIL = 0


def check(cond, what):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1
        print("  ok   " + what)
    else:
        N_FAIL += 1
        print("  FAIL " + what)


def T_of(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rot_angle_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) * 0.5
    return math.degrees(math.acos(float(np.clip(c, -1.0, 1.0))))


# Real hand-eye (the applied file) so the lever is the robot's.
_npz = np.load(os.path.join(PKG, '..', 'apriltag_nav', 'config', 'tf', 'T_hc2ee.npz'))
T_hc2ee = np.asarray(_npz[_npz.files[0]], dtype=np.float64).reshape(4, 4)
T_ee2hc = invert_T(T_hc2ee)

# A floor tag in the arm frame. Module convention (align.CAM_FRAME_NOTE,
# view_pose._target_T_A2hc): square-on = T_cam2tag rotation IDENTITY, i.e.
# the tag's +z points AWAY from the camera, into the floor -> tag z = -arm z.
T_ab2tag = T_of(np.diag([1.0, -1.0, -1.0]), [0.30, 0.90, -0.57])


def seed_pose(tilt_deg, spin_deg, xy_off, z_range):
    """T_ab2ee whose camera looks DOWN at the tag (square = identity),
    rotated ``tilt_deg`` about cam x off the normal, spun ``spin_deg``
    about its axis, with the tag ``xy_off`` (m, cam x/y) off the axis at
    range ``z_range``."""
    R_spin = rpy_deg_to_R(0.0, 0.0, spin_deg)
    R_tilt = rpy_deg_to_R(tilt_deg, 0.0, 0.0)
    R_cam2tag = R_spin @ R_tilt                       # rotation of tag seen from cam
    t_cam2tag = np.array([xy_off[0], xy_off[1], z_range])
    T_cam2tag = T_of(R_cam2tag, t_cam2tag)
    T_ab2hc = T_ab2tag @ invert_T(T_cam2tag)
    return T_ab2hc @ T_hc2ee


def observe(T_ab2ee):
    return invert_T(T_ab2ee @ T_ee2hc) @ T_ab2tag     # T_cam2tag


# ------------------------------------------------------------------ §1 pure
print("§1 compute_target_ee_pose(fix_orientation=True)")
T0 = seed_pose(1.5, 25.0, (0.04, -0.03), 0.63)
obs0 = observe(T0)
m0 = alignment_metrics(obs0)
check(abs(m0.tilt_deg - 1.5) < 1e-6 and abs(m0.xy_offset_m - 0.05) < 1e-9,
      "seed observes tilt 1.50 deg, xy 50 mm (plant sane)")
Tt = compute_target_ee_pose(T0, T_hc2ee, obs0, target_distance_m=0.50, fix_orientation=True)
check(np.array_equal(Tt[:3, :3], T0[:3, :3]), "target rotation is the current rotation bit for bit")
obs1 = observe(Tt)
check(np.allclose(obs1[:3, 3], [0, 0, 0.50], atol=1e-12),
      "after the move the tag is on the axis at 0.50 m")
check(np.allclose(obs1[:3, :3], obs0[:3, :3], atol=1e-12),
      "camera-frame rotation of the tag unchanged (tilt left at 1.5 deg)")
check(abs(alignment_metrics(obs1).tilt_deg - 1.5) < 1e-9, "tilt still 1.50 deg")
# keep-depth variant
Tk = compute_target_ee_pose(T0, T_hc2ee, obs0, target_distance_m=0.0, fix_orientation=True)
check(abs(observe(Tk)[2, 3] - 0.63) < 1e-12 and np.allclose(observe(Tk)[:2, 3], 0, atol=1e-12),
      "target_distance 0 keeps the current range 0.63 m")
# square camera, pure range error -> motion along the tag normal only
Ts = seed_pose(0.0, 0.0, (0.0, 0.0), 0.63)
Tts = compute_target_ee_pose(Ts, T_hc2ee, observe(Ts), target_distance_m=0.50, fix_orientation=True)
d = Tts[:3, 3] - Ts[:3, 3]
check(abs(d[0]) < 1e-12 and abs(d[1]) < 1e-12 and abs(d[2] + 0.13) < 1e-12,
      "square camera: a 130 mm range error moves the tool 130 mm straight DOWN (arm z) only")
# 'correct' still squares
Tc = compute_target_ee_pose(T0, T_hc2ee, obs0, target_distance_m=0.50, fix_orientation=False)
check(alignment_metrics(observe(Tc)).tilt_deg < 1e-6, "'correct' squares the camera (tilt -> 0)")
check(rot_angle_deg(Tc[:3, :3], T0[:3, :3]) > 1.0, "'correct' changes the rotation")

print("§2 clamp_step / is_converged")
st = clamp_step(T0, Tt, max_step_m=0.10, max_step_deg=15.0)
check(st.delta_rot_deg == 0.0 and np.array_equal(st.T_ab2ee_step[:3, :3], T0[:3, :3]),
      "clamped step of a fixed-orientation target has exactly zero rotation")
check(st.clamped and abs(st.delta_t_norm_m - 0.10) < 1e-12, "translation clamp still applies (0.10 m)")
check(not is_converged(m0, 0.06, 0.5), "correct: 1.5 deg tilt blocks convergence")
check(is_converged(m0, 0.06, 0.5, check_tilt=False), "fixed: tilt ignored")
check(not is_converged(m0, 0.06, 0.5, check_tilt=False, target_distance_m=0.50, depth_tol_m=0.005),
      "fixed: depth gate blocks at 0.63 vs 0.50")
check(is_converged(alignment_metrics(obs1), 0.001, 0.5, check_tilt=False,
                   target_distance_m=0.50, depth_tol_m=0.005), "fixed: converged after the move")


# ------------------------------------------------------------------ §3 runner
class FakeArm:
    def __init__(self, T):
        self.T = T.copy()
        self.moves = []

    def get_tcp_pose(self):
        return matrix_m_to_pose_fr5(self.T)

    def move_j_to_pose(self, pose, settle_s=0.0, linear=True):
        self.moves.append((list(pose), linear))
        self.T = pose_fr5_to_matrix_m(pose)


class Cfg:
    def __init__(self, **kw):
        base = dict(target_distance_m=0.50, max_iterations=10, position_tol_m=0.001,
                    angle_tol_deg=0.5, max_step_m=0.10, max_step_deg=15.0,
                    max_initial_step_m=0.80, max_initial_step_deg=180.0,
                    move_settle_s=0.0, samples_per_iteration=1, max_initial_steps=4,
                    continue_on_move_failure=True, orientation="fixed",
                    depth_tol_m=0.005, tilt_warn_deg=3.0)
        base.update(kw)
        self.__dict__.update(base)


warns = []
rospy.logwarn = lambda fmt, *a: warns.append(fmt % a if a else fmt)
rospy.loginfo = lambda *a, **k: None
ARM = [None]
align_runner.wait_for_tag_detections = lambda topic, tid, n, timeout=0: [observe(ARM[0].T)]
align_runner.median_tilt_detection = lambda dets: dets[0]
align_runner.detection_to_T_cam2tag = lambda det, a, b: det


def run(cfg, T_seed):
    ARM[0] = FakeArm(T_seed)
    rep = align_runner.run_auto_align(
        align_cfg=cfg, tcp_client=ARM[0], T_hc2ee=T_hc2ee, tag_a_id=0,
        tag_a_size_m=0.09, hand_cam_detections_topic="/x",
        hand_cam_detector_size_m=0.09, detection_wait_timeout_s=1.0,
        initial_tcp_mm_deg=matrix_m_to_pose_fr5(T_seed), skip_initial_move=True)
    return rep, ARM[0]


print("§3 run_auto_align, orientation fixed")
warns.clear()
rep, arm = run(Cfg(), T0)
seed_deg = matrix_m_to_pose_fr5(T0)[3:]
fin_deg = rep["final_tcp"][3:]
check(rep["orientation"] == "fixed", "report says fixed")
check(max(abs(a - b) for a, b in zip(seed_deg, fin_deg)) < 1e-9,
      "final rx/ry/rz == seed (%.1e deg)" % max(abs(a - b) for a, b in zip(seed_deg, fin_deg)))
check(all(max(abs(a - b) for a, b in zip(seed_deg, p[3:])) < 1e-9 for p, _ in arm.moves),
      "every commanded move carried the seed orientation (%d moves)" % len(arm.moves))
check(all(lin for _, lin in arm.moves), "correction moves are MoveL")
mf = alignment_metrics(observe(arm.T))
check(mf.xy_offset_m <= 0.001, "tag centred (xy %.2f mm)" % (mf.xy_offset_m * 1e3))
check(abs(mf.z_distance_m - 0.50) <= 0.005, "range at target (%.4f m)" % mf.z_distance_m)
check(abs(mf.tilt_deg - 1.5) < 1e-6, "tilt still 1.50 deg — not corrected")
check(all(abs(h["tilt_deg"] - 1.5) < 1e-3 for h in rep["history"]) and abs(rep["tilt_deg"] - 1.5) < 1e-3,
      "tilt recorded in every history entry and the report")
check(2 <= rep["iterations"] <= 4, "converged in %d iterations (0.13 m at 0.10 m clamp)" % rep["iterations"])
check(not [w for w in warns if "tilt" in w], "no tilt warning at 1.5 deg")

# spin free: 40 deg rz seed kept
T40 = seed_pose(0.8, 40.0, (0.02, 0.02), 0.55)
rep40, arm40 = run(Cfg(), T40)
check(max(abs(a - b) for a, b in zip(matrix_m_to_pose_fr5(T40)[3:], rep40["final_tcp"][3:])) < 1e-9
      and alignment_metrics(observe(arm40.T)).xy_offset_m <= 0.001,
      "40 deg spin seed kept, tag centred")

# tilt warning
warns.clear()
T4 = seed_pose(4.0, 0.0, (0.01, 0.0), 0.52)
rep4, arm4 = run(Cfg(), T4)
check(any("tilt 4.00 deg > 3.0" in w for w in warns), "4 deg tilt warned about")
check(alignment_metrics(observe(arm4.T)).xy_offset_m <= 0.001 and abs(rep4["tilt_deg"] - 4.0) < 1e-3,
      "and the loop still converges with the tilt left at 4 deg")

print("§3b fixed rx -180 / ry 0, rz kept")


def seed_from_rpy(rx, ry, rz, T_tag, z_range, xy_off):
    """EE pose with the given FR5 rx ry rz whose camera sees ``T_tag``
    ``xy_off`` off-axis at ``z_range``."""
    R_ee = rpy_deg_to_R(rx, ry, rz)
    R_hc = R_ee @ T_ee2hc[:3, :3]
    p_hc = T_tag[:3, 3] - z_range * R_hc[:, 2] + R_hc @ np.array([xy_off[0], xy_off[1], 0.0])
    return T_of(R_hc, p_hc) @ T_hc2ee


def observe_tag(T_ab2ee, T_tag):
    return invert_T(T_ab2ee @ T_ee2hc) @ T_tag


def cam_R(T_ab2ee):
    return T_ab2ee[:3, :3] @ T_ee2hc[:3, :3]


# The hand-eye's optical axis is not exactly the flange z: a FLANGE at
# rx -180 / ry 0 reads a flat floor tag at this constant tilt.
R_vert = rpy_deg_to_R(-180.0, 0.0, 25.0)
z_cam_vert = (R_vert @ T_ee2hc[:3, :3])[:, 2]
he_axis_deg = math.degrees(math.acos(float(np.clip(-z_cam_vert[2], -1, 1))))
print("  (hand-eye optical axis vs flange z: %.3f deg)" % he_axis_deg)
check(0.1 < he_axis_deg < 1.0, "the applied hand-eye's axis is %.3f deg off the flange z (what 'camera' absorbs)" % he_axis_deg)

R_seed = rpy_deg_to_R(-179.5, -0.3, 25.0)
Rf = fixed_orientation_R(R_seed, (-180.0, 0.0))
check(rot_angle_deg(Rf, R_vert) < 1e-9, "fixed_orientation_R flange: rx/ry replaced, rz 25 kept")
check(np.array_equal(fixed_orientation_R(R_vert, None), R_vert), "fixed_orientation_R(None) is the input itself")
Rc = fixed_orientation_R(R_seed, (-180.0, 0.0), R_hc2ee=T_hc2ee[:3, :3])
Rc_cam = Rc @ T_ee2hc[:3, :3]
from path_tag_locator.geometry import rot2rpy_deg  # noqa: E402
rz_cam_seed = float(rot2rpy_deg(R_seed @ T_ee2hc[:3, :3])[2])
check(np.abs(Rc_cam - rpy_deg_to_R(-180.0, 0.0, rz_cam_seed)).max() < 1e-12,
      "fixed_orientation_R camera: the CAMERA frame is at rx -180 / ry 0 with its spin (%.2f) kept" % rz_cam_seed)
check(abs(Rc_cam[2, 2] + 1.0) < 1e-12, "camera optical axis exactly along -arm z")
check(rot_angle_deg(Rc, R_vert) > 0.3, "the flange then differs from -180 / 0 by the hand-eye axis offset")

for frame in ("camera", "flange"):
    for slope_deg in (0.0, 0.7):
        T_tag_s = T_of(rpy_deg_to_R(slope_deg, 0.0, 0.0) @ np.diag([1.0, -1.0, -1.0]), [0.30, 0.90, -0.57])
        align_runner.wait_for_tag_detections = lambda topic, tid, n, timeout=0, _T=T_tag_s: [observe_tag(ARM[0].T, _T)]
        T_seed = seed_from_rpy(-179.5, -0.3, 25.0, T_tag_s, 0.63, (0.04, -0.03))
        warns.clear()
        rep, arm = run(Cfg(fixed_rx_deg=-180.0, fixed_ry_deg=0.0, fixed_rpy_frame=frame), T_seed)
        fin = rep["final_tcp"]
        tag = "%s slope %.1f" % (frame, slope_deg)
        check(rep["fixed_rx_ry_deg"] == [-180.0, 0.0] and rep["fixed_rpy_frame"] == frame,
              "%s: report carries the fixed rx/ry and frame" % tag)
        if frame == "flange":
            check(abs(abs(fin[3]) - 180.0) < 1e-9 and abs(fin[4]) < 1e-9 and abs(fin[5] - 25.0) < 1e-9,
                  "%s: final flange rx %.6f ry %.6f rz %.6f (|rx| 180, ry 0, rz kept)" % (tag, fin[3], fin[4], fin[5]))
            expect_tilt = None   # slope + the hand-eye axis offset, direction-dependent
        else:
            Rfin_cam = cam_R(arm.T)
            check(np.abs(Rfin_cam - rpy_deg_to_R(-180.0, 0.0, rz_cam_seed)).max() < 1e-12
                  and abs(Rfin_cam[2, 2] + 1.0) < 1e-12,
                  "%s: final CAMERA frame at rx -180 / ry 0, spin kept; flange reads %.3f / %.3f / %.3f"
                  % (tag, fin[3], fin[4], fin[5]))
            expect_tilt = slope_deg
        first_rot = rot_angle_deg(pose_fr5_to_matrix_m(arm.moves[0][0])[:3, :3], T_seed[:3, :3])
        check(0.1 < first_rot < 1.5, "%s: first step turned %.3f deg onto the fixed orientation" % (tag, first_rot))
        later = [rot_angle_deg(pose_fr5_to_matrix_m(p)[:3, :3], arm.T[:3, :3]) for p, _ in arm.moves[1:]]
        check(all(r < 1e-9 for r in later), "%s: later steps are pure translations (%d)" % (tag, len(later)))
        mf = alignment_metrics(observe_tag(arm.T, T_tag_s))
        check(mf.xy_offset_m <= 0.001 and abs(mf.z_distance_m - 0.50) <= 0.005,
              "%s: tag centred (%.2f mm) at %.4f m" % (tag, mf.xy_offset_m * 1e3, mf.z_distance_m))
        if expect_tilt is None:
            check(abs(mf.tilt_deg - slope_deg) < he_axis_deg + 0.02 and mf.tilt_deg > 0.1,
                  "%s: recorded tilt %.3f deg = slope + the hand-eye axis offset" % (tag, mf.tilt_deg))
        else:
            check(abs(mf.tilt_deg - expect_tilt) < 1e-6,
                  "%s: recorded tilt %.3f deg = exactly the tag's slope vs the arm vertical" % (tag, mf.tilt_deg))
        tilts = [h["tilt_deg"] for h in rep["history"][1:]]
        check(max(tilts) - min(tilts) < 1e-6, "%s: tilt constant over the iterations after the first step" % tag)
        check(not [w for w in warns if "tilt" in w], "%s: no tilt warning" % tag)
# restore the default plant
align_runner.wait_for_tag_detections = lambda topic, tid, n, timeout=0: [observe(ARM[0].T)]

print("§4 run_auto_align, orientation correct (sweep behaviour intact)")
repc, armc = run(Cfg(orientation="correct"), T0)
mc = alignment_metrics(observe(armc.T))
check(repc["orientation"] == "correct" and mc.tilt_deg <= 0.5 and mc.xy_offset_m <= 0.001,
      "correct: tilt %.3f deg, xy %.2f mm" % (mc.tilt_deg, mc.xy_offset_m * 1e3))
check(rot_angle_deg(armc.T[:3, :3], T0[:3, :3]) > 1.0, "correct: rotation was commanded")

print("§5 config")
try:
    Cfg2 = dict(target_distance_m=0, max_iterations=1, position_tol_m=0, angle_tol_deg=0,
                max_step_m=0, max_step_deg=0, max_initial_step_m=0, max_initial_step_deg=0,
                move_vel=0, move_acc=0, move_ovl=0, move_settle_s=0, orientation="sideways")
    AlignCfg(**Cfg2)
    check(False, "unknown orientation refused")
except ValueError:
    check(True, "unknown orientation refused")
cfg = load_locator_cfg_from_dict(yaml.safe_load(open(os.path.join(PKG, "config", "locator.yaml"))))
check(cfg.align.orientation == "fixed", "locator.yaml: orientation fixed")
check(abs(cfg.align.target_distance_m - 0.50) < 1e-9
      and abs(cfg.align.target_distance_m - cfg.align.auto_view_distance_m) < 1e-9,
      "locator.yaml: target_distance_m 0.50 == auto_view_distance_m")
check(cfg.align.depth_tol_m > 0 and cfg.align.tilt_warn_deg > 0, "depth_tol_m / tilt_warn_deg set")
check(cfg.align.fixed_rx_deg == -180.0 and cfg.align.fixed_ry_deg == 0.0, "locator.yaml: fixed rx -180 / ry 0")
check(cfg.align.fixed_rpy_frame == "camera", "locator.yaml: fixed_rpy_frame camera (hand-eye applied)")
try:
    Cfg2["orientation"] = "fixed"; Cfg2["fixed_rx_deg"] = -180.0; Cfg2["fixed_ry_deg"] = 0.0
    Cfg2["fixed_rpy_frame"] = "world"
    AlignCfg(**Cfg2)
    check(False, "unknown fixed_rpy_frame refused")
except ValueError:
    check(True, "unknown fixed_rpy_frame refused")
try:
    Cfg2["orientation"] = "fixed"; Cfg2["fixed_rx_deg"] = -180.0
    AlignCfg(**Cfg2)
    check(False, "fixed_rx without fixed_ry refused")
except ValueError:
    check(True, "fixed_rx without fixed_ry refused")

print("\n%d ok, %d fail" % (N_OK, N_FAIL))
sys.exit(1 if N_FAIL else 0)
