#!/usr/bin/env python3
"""
check_front_cam_extrinsics.py
=============================
Offline check that tf_chain.yaml's T_mb2fc (apriltag_nav/config/tf — the
PHYSICAL, tilted front_cam since 2026-09-15) and robot.yaml's ground-plane
fit agree, and that the
chain lands a floor tag on its true world position whichever frame the
detections come in — no robot, no cameras.

    python3 src/path_tag_locator/scripts/check_front_cam_extrinsics.py

What is proven, in order:
  1. the stored matrix is exactly what tf_chain.physical_T_mb2fc
     (tf_chain_tool.py front-cam) produces from robot.yaml;
  2. load_extrinsics_full derives a level frame with rotation
     diag(1,-1,-1) and the same lens centre, follows ground_plane.enabled
     for "auto", honours the overrides, and REFUSES a hand-edited
     rotation or a tz that drifted from height_m;
  3. end to end on a synthetic scene rendered by a plain pinhole through
     the physical camera: (a) RAW corners + physical T_mb2fc, (b) the
     same corners through the real GroundPlane.correct() (what
     robot_camera_node publishes) + T_mb2fc_level — both put the tag on
     its true world pose to < 0.1 mm / 0.001 deg; (c) the cross pairing
     is off by the full tilt (~1.33 deg, mm of position), i.e. the
     selection matters and the sign is right. Also that the pinhole
     render equals GroundPlane.project(), so the two conventions agree.
"""
import copy
import math
import os
import sys
import tempfile

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.realpath(__file__))
_PKG = os.path.abspath(os.path.join(_HERE, ".."))
sys.path.insert(0, os.path.join(_PKG, "src"))
sys.path.insert(0, os.path.join(_PKG, "..", "apriltag_nav", "src"))

try:
    import rospy  # noqa: F401
except ImportError:
    import types
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

from apriltag_nav.ground_plane import GroundPlane, T_tilted_to_level  # noqa: E402
from apriltag_nav.paths import CONFIG_PATH                             # noqa: E402
from path_tag_locator.constants import (                               # noqa: E402
    R_MB2FC_LEVEL, load_extrinsics, load_extrinsics_full)
from path_tag_locator.detections import pose_from_corners, _CORNER_ORDER  # noqa: E402
from path_tag_locator.geometry import invert_T                         # noqa: E402

from apriltag_nav.tf_chain import TF_CHAIN_PATH, physical_T_mb2fc    # noqa: E402

EXTRINSICS = TF_CHAIN_PATH

_n = [0]
_bad = [0]


def check(name, ok, detail=""):
    _n[0] += 1
    if not ok:
        _bad[0] += 1
    print(f"  [{'ok ' if ok else 'FAIL'}] {name}{(' — ' + detail) if detail else ''}")


def angdiff(A, B):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(A.T @ B) - 1) / 2))))


def rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


robot_cfg = yaml.safe_load(open(CONFIG_PATH))
gp = robot_cfg["robot_camera"]["ground_plane"]["front_cam"]
cam_off = float(robot_cfg["robot"]["camera_offset"])
roll, pitch, yaw = (math.radians(float(gp[k])) for k in ("roll_deg", "pitch_deg", "yaw_deg"))
H = float(gp["height_m"])
THICK = float(robot_cfg["robot"].get("tag_thickness", 0.0))

print("== 1. stored matrix == generator(robot.yaml) ==")
T_ab2mb, T_stored = load_extrinsics(EXTRINSICS)
T_gen, params = physical_T_mb2fc(robot_cfg)
check("T_mb2fc equals the generator's output", np.abs(T_stored - T_gen).max() < 1e-8,
      "max |diff| %.1e" % np.abs(T_stored - T_gen).max())
check("tx == robot.camera_offset", abs(T_stored[0, 3] - cam_off) < 1e-9, "%.3f" % T_stored[0, 3])
check("ty == 0", abs(T_stored[1, 3]) < 1e-9)
check("tz == ground_plane.front_cam.height_m + robot.tag_thickness (lens above the FLOOR)",
      abs(T_stored[2, 3] - (H + THICK)) < 1e-9, "%.3f = %.3f + %.3f" % (T_stored[2, 3], H, THICK))
ax = math.degrees(math.acos(-T_stored[2, 2]))
check("optical axis tilted off vertical by the fit's magnitude", 1.2 < ax < 1.5,
      "%.3f deg" % ax)
# T_ab2mb is the 2026-09-21 chain_calib value (A0 sheet): a rigid transform a
# degree or two and a few cm from the design Rz(180) / (0, -0.100, -0.652). Pin that neighbourhood, not the design numbers — a hand-edit that
# breaks orthonormality or drifts far from the mount geometry is what should
# fail here, not a re-calibration.
R_ab, t_ab = T_ab2mb[:3, :3], T_ab2mb[:3, 3]
ang_ab = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(np.diag([-1, -1, 1.0]).T @ R_ab) - 1) / 2))))
check("T_ab2mb rotation is orthonormal, det +1",
      np.abs(R_ab @ R_ab.T - np.eye(3)).max() < 1e-6 and abs(np.linalg.det(R_ab) - 1) < 1e-6)
check("T_ab2mb within 3 deg of Rz(180) and 50 mm of the design (0, -0.100, -0.652) (calibrated 2026-09-21: 1.7 deg, 34 mm)",
      ang_ab < 3.0 and np.linalg.norm(t_ab - [0, -0.100, -0.652]) < 0.050,
      "%.2f deg, %.1f mm" % (ang_ab, 1e3 * np.linalg.norm(t_ab - [0, -0.100, -0.652])))

print("\n== 2. loader ==")
ext = load_extrinsics_full(EXTRINSICS)
check("level rotation is exactly diag(1,-1,-1)",
      np.array_equal(ext.T_mb2fc_level[:3, :3], R_MB2FC_LEVEL))
check("level translation == physical translation (same lens centre)",
      np.allclose(ext.T_mb2fc_level[:3, 3], ext.T_mb2fc[:3, 3]))
check("level == physical @ T_tilted_to_level to 1e-9",
      np.abs(ext.T_mb2fc @ T_tilted_to_level(roll, pitch, yaw) - ext.T_mb2fc_level).max() < 1e-9)
check("auto with ground_plane enabled -> level", ext.front_cam_frame == "level"
      and ext.T_mb2fc_chain is ext.T_mb2fc_level, ext.note)
gp_off = dict(gp, enabled=False, tag_thickness_m=THICK)
e2 = load_extrinsics_full(EXTRINSICS, ground_plane=gp_off)
check("auto with ground_plane disabled -> physical", e2.front_cam_frame == "physical"
      and e2.T_mb2fc_chain is e2.T_mb2fc)
e3 = load_extrinsics_full(EXTRINSICS, front_cam_frame="physical")
check("override 'physical' wins over an enabled correction",
      e3.T_mb2fc_chain is e3.T_mb2fc)
e4 = load_extrinsics_full(EXTRINSICS, front_cam_frame="level", ground_plane=gp_off)
check("override 'level' wins over a disabled correction",
      e4.T_mb2fc_chain is e4.T_mb2fc_level)
try:
    load_extrinsics_full(EXTRINSICS, front_cam_frame="sideways")
    check("bad front_cam_frame refused", False)
except ValueError:
    check("bad front_cam_frame refused", True)


def _write_tmp(T):
    d = yaml.safe_load(open(EXTRINSICS))
    d["T_mb2fc"]["matrix"] = [float(v) for v in T.ravel()]
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(d, f)
    f.close()
    return f.name


# a level matrix (the pre-2026-09-15 file) no longer matches the fit
T_old = np.eye(4)
T_old[:3, :3] = R_MB2FC_LEVEL
T_old[:3, 3] = [cam_off, 0, H + THICK]
try:
    load_extrinsics_full(_write_tmp(T_old))
    check("a LEVEL matrix in the yaml is refused while the fit is non-zero", False)
except ValueError as e:
    check("a LEVEL matrix in the yaml is refused while the fit is non-zero", True,
          str(e)[:60] + "…")
T_bad = T_stored.copy()
T_bad[2, 3] = H          # the pre-thickness value: lens above the TAG plane, not the floor
try:
    load_extrinsics_full(_write_tmp(T_bad))
    check("tz == height_m alone (tag thickness forgotten) is refused", False)
except ValueError:
    check("tz == height_m alone (tag thickness forgotten) is refused", True)
# with NO ground-plane block the stored (tilted) matrix must also be refused —
# it is not level, and nothing would un-tilt the detections
try:
    load_extrinsics_full(EXTRINSICS, ground_plane=None)
    check("tilted matrix with no ground_plane block is refused", False)
except ValueError:
    check("tilted matrix with no ground_plane block is refused", True)
e5 = load_extrinsics_full(_write_tmp(T_old), ground_plane=None)
check("level matrix with no ground_plane block loads as physical == level",
      e5.front_cam_frame == "physical" and np.allclose(e5.T_mb2fc, e5.T_mb2fc_level))

print("\n== 3. end to end: floor tag through the chain ==")
FX = FY = 910.0
CX, CY = 640.0, 360.0
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], float)
TAG = 0.090
gpl = GroundPlane(FX, FY, CX, CY, None, roll, pitch, H, yaw=yaw)
worst = dict(raw_t=0.0, raw_R=0.0, lvl_t=0.0, lvl_R=0.0, x_t=0.0, x_R=0.0, px=0.0, lvl_z=0.0)
rng = np.random.RandomState(7)
for i in range(40):
    # base somewhere on the floor, heading anything; tag laid flat ahead
    th = rng.uniform(-180, 180)
    T_wmb = np.eye(4)
    T_wmb[:3, :3] = rz(th)
    T_wmb[:3, 3] = [rng.uniform(-2, 2), rng.uniform(-2, 2), 0.0]
    # tag on the floor, near the lens nadir (within the frame at 0.3 m)
    tag_yaw = rng.uniform(-180, 180)
    d_fwd, d_right = rng.uniform(0.35, 0.75), rng.uniform(-0.09, 0.09)
    T_mb2B = np.eye(4)
    T_mb2B[:3, :3] = rz(tag_yaw)
    T_mb2B[:3, 3] = [d_fwd, -d_right, THICK]   # mb y is LEFT; the tag's top face is a plate-thickness up
    T_wB = T_wmb @ T_mb2B
    # a face-UP tag's +z points up; the camera looks down -> z_tag = +mb.z
    T_wfc_phys = T_wmb @ ext.T_mb2fc
    T_wfc_lvl = T_wmb @ ext.T_mb2fc_level
    corners_w = (T_wB[:3, :3] @ (np.c_[_CORNER_ORDER * TAG / 2, np.zeros(4)]).T).T + T_wB[:3, 3]
    # render RAW pixels through the PHYSICAL camera (plain pinhole)
    pc = (invert_T(T_wfc_phys)[:3, :3] @ corners_w.T).T + invert_T(T_wfc_phys)[:3, 3]
    uv_raw = (pc / pc[:, 2:3]) @ K.T
    uv_raw = uv_raw[:, :2]
    if (uv_raw < 0).any() or (uv_raw[:, 0] > 1280).any() or (uv_raw[:, 1] > 720).any():
        continue
    # consistency with GroundPlane's own forward model: ground coords are
    # the LEVEL camera's (X fwd, Y right) relative to the nadir
    pl = (invert_T(T_wfc_lvl)[:3, :3] @ corners_w.T).T + invert_T(T_wfc_lvl)[:3, 3]
    uv_gp = gpl.project(pl[:, :2])
    worst["px"] = max(worst["px"], np.abs(uv_gp - uv_raw).max())
    # (a) raw corners -> pose -> physical chain
    T_fc2B = pose_from_corners(uv_raw.ravel(), TAG, K)
    T_est = T_wfc_phys @ T_fc2B
    worst["raw_t"] = max(worst["raw_t"], np.linalg.norm(T_est[:3, 3] - T_wB[:3, 3]) * 1e3)
    worst["raw_R"] = max(worst["raw_R"], angdiff(T_est[:3, :3], T_wB[:3, :3]))
    # (b) what robot_camera_node publishes: corrected corners -> level chain
    c_lvl, _, _ = gpl.correct(uv_raw, uv_raw.mean(axis=0))
    T_fc2B_l = pose_from_corners(np.asarray(c_lvl).ravel(), TAG, K)
    T_est_l = T_wfc_lvl @ T_fc2B_l
    worst["lvl_t"] = max(worst["lvl_t"], np.linalg.norm(T_est_l[:3, 3] - T_wB[:3, 3]) * 1e3)
    worst["lvl_R"] = max(worst["lvl_R"], angdiff(T_est_l[:3, :3], T_wB[:3, :3]))
    worst["lvl_z"] = float((invert_T(T_wmb) @ T_est_l)[2, 3])
    # (c) the wrong pairing: level corners with the physical matrix
    T_est_x = T_wfc_phys @ T_fc2B_l
    worst["x_t"] = max(worst["x_t"], np.linalg.norm(T_est_x[:3, 3] - T_wB[:3, 3]) * 1e3)
    worst["x_R"] = max(worst["x_R"], angdiff(T_est_x[:3, :3], T_wB[:3, :3]))

check("pinhole render through T_mb2fc == GroundPlane.project (same convention)",
      worst["px"] < 1e-6, "worst %.1e px" % worst["px"])
check("RAW corners + physical T_mb2fc land on the true tag pose",
      worst["raw_t"] < 0.1 and worst["raw_R"] < 1e-2,      # solvePnP's own numerical floor (~0.002 deg)
      "worst %.4f mm / %.5f deg" % (worst["raw_t"], worst["raw_R"]))
check("CORRECTED corners + T_mb2fc_level land on the true tag pose",
      worst["lvl_t"] < 0.1 and worst["lvl_R"] < 1e-2,
      "worst %.4f mm / %.5f deg" % (worst["lvl_t"], worst["lvl_R"]))
check("cross pairing is off by the full tilt (selection matters)",
      worst["x_R"] > 1.2 and worst["x_t"] > 3.0,
      "worst %.2f mm / %.3f deg" % (worst["x_t"], worst["x_R"]))
check("the located floor tag sits at z = tag_thickness in mb, not on the floor",
      abs(worst["lvl_z"] - THICK) < 1e-4, "z %.4f m (tag_thickness %.3f)" % (worst["lvl_z"], THICK))

print("\n%d checks, %d failed" % (_n[0], _bad[0]))
sys.exit(1 if _bad[0] else 0)
