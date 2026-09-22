#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_joint_offsets.py
======================
OFFLINE self-check for the arm joint zero offsets (2026-09-22) — no ROS
master, no robot::

    python3 src/apriltag_nav/tools/check_joint_offsets.py

What it guards
--------------
config/tf/arm_joint_offsets.yaml holds dq (deg), ADDED to the controller's
joint reading; `tf_chain.arm_flange_T` turns the locator chain's flange
into FK_urdf(q + dq) instead of the TCP the controller reports, and
`path_tag_locator.chain.compute_T_A2B` / `chain_calib.py solve` go through
it. The checks pin:
  * the URDF FK reproduces the controller's TCP with dq = 0 (the two
    poses the robot reported on 2026-09-22 — home and a scan-height pose),
  * dq = 0, no joints, a disabled file and a joints/pose mismatch all
    give the controller's pose BIT-FOR-BIT (nothing changes where the
    offsets are off),
  * the SIGN and the lever: a +1 deg J6 offset spins the flange about its
    own z by +1 deg; a J2 offset moves the flange by the link lever,
  * the yaml <-> npz round trip and the refusals (6 values, < 5 deg),
  * compute_T_A2B: with offsets the flange used is FK(q + dq) and the
    result says `joint_offsets_applied`; without, the old product exactly.
"""
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PKG, "src"))
sys.path.insert(0, os.path.join(os.path.dirname(PKG), "path_tag_locator", "src"))

from apriltag_nav import tf_chain as TC                          # noqa: E402
from apriltag_nav.arm_fk import ArmChain                         # noqa: E402
from path_tag_locator.geometry import pose_fr5_to_matrix_m, invert_T   # noqa: E402
from path_tag_locator.chain import compute_T_A2B                 # noqa: E402

N_OK = N_FAIL = 0


def check(cond, msg):
    global N_OK, N_FAIL
    if cond:
        N_OK += 1
        print("  [ok ] " + msg)
    else:
        N_FAIL += 1
        print("  [FAIL] " + msg)


def ang_deg(A, B):
    c = (np.trace(A[:3, :3].T @ B[:3, :3]) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# two states the robot reported on 2026-09-22 (/arm/state)
HOME_Q = [-90.0013, -55.0659 + 0.0, 141.6320, -176.6158, -89.9987, 0.0002]
HOME_TCP = [-158.9878, 550.0385, 63.0180, -179.9506, 0.0013, -0.0017]
Q2 = [-90.0, -55.0659, 141.6320, -176.6158, -89.9987, 0.0]

print("== 1. URDF FK == controller TCP (dq = 0) ==")
chain = ArmChain()
T = chain.fk_flange(HOME_Q)
Tc = pose_fr5_to_matrix_m(HOME_TCP)
d = 1e3 * np.linalg.norm(T[:3, 3] - Tc[:3, 3])
check(d < 0.1 and ang_deg(T, Tc) < 0.01, "FK(q) reproduces the reported TCP: %.3f mm / %.4f deg" % (d, ang_deg(T, Tc)))

print("== 2. arm_flange_T is a no-op wherever offsets do not apply ==")
DQ = [0.0, -0.239, -0.652, 0.011, -0.141, -0.473]
T0, a0 = TC.arm_flange_T(HOME_TCP, HOME_Q, np.zeros(6))
check(not a0 and np.array_equal(T0, Tc), "dq = 0 -> controller pose bit-for-bit, applied False")
T1, a1 = TC.arm_flange_T(HOME_TCP, None, DQ)
check(not a1 and np.array_equal(T1, Tc), "no joints -> controller pose, applied False")
T2, a2 = TC.arm_flange_T(HOME_TCP, HOME_Q[:5], DQ, warn=lambda m: None)
check(not a2 and np.array_equal(T2, Tc), "5 joint values -> controller pose, applied False")
msgs = []
T3, a3 = TC.arm_flange_T([0, 0, 0, 0, 0, 0], HOME_Q, DQ, warn=msgs.append)
check(not a3 and msgs and "NOT applied" in msgs[0], "joints/pose from different states -> refused with a warning")

print("== 3. sign and lever ==")
T4, a4 = TC.arm_flange_T(HOME_TCP, HOME_Q, [0, 0, 0, 0, 0, 1.0])
E = invert_T(chain.fk_flange(HOME_Q)) @ T4
check(a4 and np.abs(E[:3, 3]).max() < 1e-9 and abs(math.degrees(math.atan2(E[1, 0], E[0, 0])) - 1.0) < 1e-6,
      "+1 deg on J6 spins the flange +1 deg about its own z, no translation")
T5, a5 = TC.arm_flange_T(HOME_TCP, HOME_Q, [0, 1.0, 0, 0, 0, 0])
# the J2 axis is horizontal: a 1 deg offset swings the flange by ~ lever * 1 deg
moved = 1e3 * np.linalg.norm(T5[:3, 3] - Tc[:3, 3])
check(a5 and 5.0 < moved < 25.0, "+1 deg on J2 moves the flange by a link lever: %.1f mm" % moved)
T6, a6 = TC.arm_flange_T(HOME_TCP, HOME_Q, DQ)
check(a6 and np.allclose(T6, chain.fk_flange(HOME_Q, dq_deg=DQ)), "the applied pose is FK(q + dq)")
check(1e3 * np.linalg.norm(T6[:3, 3] - Tc[:3, 3]) < 15.0, "today's dq moves the home flange by %.1f mm (< 15)" % (1e3 * np.linalg.norm(T6[:3, 3] - Tc[:3, 3])))

print("== 4. yaml / npz round trip, refusals ==")
with tempfile.TemporaryDirectory() as td:
    y = os.path.join(td, "arm_joint_offsets.yaml")
    TC.write_joint_offsets(DQ, "test", "he.npz", "urdf", session="s", path=y)
    check(np.allclose(TC.load_joint_offsets(y), DQ), "write -> load reproduces dq")
    ok, why = TC.check_joint_offsets(y)
    check(ok and "enabled" in why, "check_joint_offsets: %s" % why)
    TC.write_joint_offsets(DQ, "test", "he.npz", "urdf", enabled=False, path=y)
    check(np.array_equal(TC.load_joint_offsets(y), np.zeros(6)), "enabled: false -> zeros")
    check(np.allclose(TC.load_joint_offsets(y, require_enabled=False), DQ), "  ... but the stored values remain readable")
    ok, why = TC.check_joint_offsets(y)
    check(ok and "DISABLED" in why, "check reports DISABLED")
    check(np.array_equal(TC.load_joint_offsets(os.path.join(td, "nope.yaml")), np.zeros(6)), "absent file -> zeros")
    open(y, "w").write("enabled: true\ndq_deg: [0, 1, 2]\n")
    try:
        TC.load_joint_offsets(y); check(False, "3 values refused")
    except ValueError:
        check(True, "3 values refused")
    open(y, "w").write("enabled: true\ndq_deg: [0, 7, 0, 0, 0, 0]\n")
    try:
        TC.load_joint_offsets(y); check(False, "7 deg refused")
    except ValueError:
        check(True, "an offset over 5 deg refused")
    ok, why = TC.check_joint_offsets(y)
    check(not ok, "check fails on it: %s" % why)

print("== 5. compute_T_A2B goes through it ==")
tf = TC.load_tf_chain()
T_hc2A = np.eye(4); T_hc2A[:3, 3] = [0.01, -0.02, 0.5]
T_fc2B = np.eye(4); T_fc2B[:3, 3] = [0.03, -0.05, 0.302]
base = compute_T_A2B(T_hc2A=T_hc2A, T_fc2B=T_fc2B, tcp_pose_mm_deg=HOME_TCP, T_hc2ee=tf["T_hc2ee"],
                     T_ab2mb=tf["T_ab2mb"], T_mb2fc=tf["T_mb2fc"])
old = invert_T(T_hc2A) @ tf["T_hc2ee"] @ invert_T(Tc) @ tf["T_ab2mb"] @ tf["T_mb2fc"] @ T_fc2B
check(not base["joint_offsets_applied"] and np.array_equal(base["T_A2B"], old), "no joints: the old product exactly, applied False")
with_ = compute_T_A2B(T_hc2A=T_hc2A, T_fc2B=T_fc2B, tcp_pose_mm_deg=HOME_TCP, joints_deg=HOME_Q, joint_offsets_deg=DQ,
                      T_hc2ee=tf["T_hc2ee"], T_ab2mb=tf["T_ab2mb"], T_mb2fc=tf["T_mb2fc"])
exp = invert_T(T_hc2A) @ tf["T_hc2ee"] @ invert_T(chain.fk_flange(HOME_Q, dq_deg=DQ)) @ tf["T_ab2mb"] @ tf["T_mb2fc"] @ T_fc2B
check(with_["joint_offsets_applied"] and np.allclose(with_["T_A2B"], exp), "joints + dq: FK(q + dq) in the product, applied True")
shift = 1e3 * np.linalg.norm(with_["T_A2B"][:3, 3] - base["T_A2B"][:3, 3])
check(0.5 < shift < 20.0, "the offsets move the located tag by %.1f mm at this pose" % shift)
zero = compute_T_A2B(T_hc2A=T_hc2A, T_fc2B=T_fc2B, tcp_pose_mm_deg=HOME_TCP, joints_deg=HOME_Q, joint_offsets_deg=np.zeros(6),
                     T_hc2ee=tf["T_hc2ee"], T_ab2mb=tf["T_ab2mb"], T_mb2fc=tf["T_mb2fc"])
check(not zero["joint_offsets_applied"] and np.array_equal(zero["T_A2B"], old), "joints + dq = 0: the old product exactly")

print("\n%d ok, %d failed" % (N_OK, N_FAIL))
sys.exit(1 if N_FAIL else 0)
