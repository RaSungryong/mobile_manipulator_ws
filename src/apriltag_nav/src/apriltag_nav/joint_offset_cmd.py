# -*- coding: utf-8 -*-
"""
joint_offset_cmd.py — apply the arm's joint ZERO offsets on the COMMAND side
(2026-09-28).

`config/tf/arm_joint_offsets.yaml` holds dq (deg) with the convention
    physical angle = controller reading + dq
(chain_calib/arm_offsets.py, applied 2026-09-22). Until now they acted on
the MEASUREMENT chain only (`tf_chain.arm_flange_T`: T_ab2ee = FK(q + dq)),
so the locator knew where the flange really was — but every command still
went to the controller unchanged, and the controller places FK(q) at the
target with its own zeros, i.e. the PHYSICAL flange lands at FK(q + dq):
~10 mm and ~1 deg off at the 2026-09-28 sheet views (CLAUDE.md Work Log).

Two forms, one for each way the controller is commanded:

* joint targets (pose-mode IK -> MoveJ): the IK's q puts the flange on the
  target under the nominal model, so the PHYSICAL joints must be q, and
  the reading to command is q - dq  (`joints_for_physical`).
* Cartesian targets (MoveL / MoveCart, the controller does the IK): the
  controller can only be asked for a pose; ask for the pose whose
  nominal solution q satisfies FK(q + dq) = target. Fixed point in two
  iterations (dq is < 1 deg):  q = IK(cmd);  D = inv(FK(q)) . FK(q + dq)
  (the offsets' effect in the FLANGE frame);  cmd = target . inv(D)
  (`pose_for_physical`). Needs an IK callable (the SDK's GetInverseKin)
  and the URDF FK (`arm_fk.ArmChain`, which reproduces the controller's
  TCP to 0.01 mm).

Which targets get corrected is the caller's decision, and the rule is:
**an ABSOLUTE physical pose (a world point, the sheet, a plan seed) — yes;
a pose built as "current READING + a delta" (jog, the align's camera-frame
correction steps, the standoff loop) — no**, because for those the
offsets' effect at the start and at the end cancels: reading + delta
commanded lands at physical + delta. `ArmController.move_cart(...,
physical=True)` / the JSON key `"physical": true` on /arm/move_cart /
`ArmInterface.move_j_to_pose(physical=True)` opt in; `_exec_pose` always
corrects (its targets are world points) — all of it only while arm_node's
`~apply_joint_offsets_cmd` is true.

xy only, z left alone (`skip_z`, 2026-09-28 evening — the mode in use).
The applied set (offsets, planar T_ab2mb, hand-eye z) is consistent in the
arm's in-plane position but NOT in its absolute z: at the touch poses the
offsets predict the tip +9..14 mm above the reading while the Keyence
measured it +4 mm (cross tags 0/1/5) or −4 mm (cross tag 2). Correcting z
would send the case 5..9 mm INTO the plate on the approach. So with
`skip_z` the full 6-DOF correction is computed (translation AND the ~1 deg
rotation — the rotation is part of the in-plane fix: the vision tip sits
0.32 m from the flange, so 1 deg is 5.6 mm of tip xy and 0.05 mm of tip z)
and then the commanded arm-frame z is put back to the target's z. Arm z is
the vertical with the planar T_ab2mb (roll = pitch = 0), so the standoff
loop sees exactly the z it saw before and finds the surface as before.
`joints_for_physical_pose` is the joint-target form of the same thing
(IK of the xy-corrected pose), used by `_exec_pose` instead of q − dq.

Why this is the fix, measured (Work Log 2026-09-28 night): the map
calibration runs with these offsets IN its chain (`joint_offsets_applied:
true`), the tip command ran without them, and the 22-stop
tip_touch_cross_tags run landed the tip a constant 8.8 mm toward body −y in
BOTH lane zones (+9 mm world x in zone B, −8 in zone C) plus an along-lane
pattern; FK(q + dq) − FK(q) at every recorded touch configuration
reproduces the measured error (mean (+1.9, −0.3) vs (+1.9, −0.1) mm, same
sign flip, same trend, rms 10.8 → residual 5.3 mm). The default in
arm_controller is `~apply_joint_offsets_cmd` false / `~joint_offsets_cmd_
skip_z` true; the launch turns the correction ON.

Pure numpy; the pose <-> matrix conversions are the FR5 ZYX-intrinsic ones
of path_tag_locator.geometry, copied so arm_node needs no import from
that package (the check asserts they agree).
"""
import math

import numpy as np


def pose_to_T(pose_mm_deg):
    """[x y z mm, rx ry rz deg] (ZYX intrinsic) -> 4x4 (m)."""
    x, y, z = [float(v) / 1000.0 for v in pose_mm_deg[:3]]
    rx, ry, rz = [math.radians(float(v)) for v in pose_mm_deg[3:6]]
    cr, sr, cp, sp, cy, sy = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = [x, y, z]
    return T


def T_to_pose(T):
    """4x4 (m) -> [x y z mm, rx ry rz deg] (ZYX intrinsic)."""
    R = T[:3, :3]
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    if sy > 1e-6:
        rx, ry, rz = math.atan2(R[2, 1], R[2, 2]), math.atan2(-R[2, 0], sy), math.atan2(R[1, 0], R[0, 0])
    else:
        rx, ry, rz = math.atan2(-R[1, 2], R[1, 1]), math.atan2(-R[2, 0], sy), 0.0
    return [float(T[0, 3]) * 1e3, float(T[1, 3]) * 1e3, float(T[2, 3]) * 1e3,
            math.degrees(rx), math.degrees(ry), math.degrees(rz)]


def _inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


def _rot_deg(R):
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))))


class CommandCorrector:
    """Turns PHYSICAL targets into what to send to a controller whose joint
    zeros are off by ``dq_deg`` (physical = reading + dq)."""

    def __init__(self, dq_deg, fk=None, max_corr_mm=30.0, max_corr_deg=3.0, iters=2, skip_z=False):
        self.dq = np.asarray(dq_deg if dq_deg is not None else np.zeros(6), dtype=np.float64).ravel()
        if self.dq.size != 6:
            raise ValueError("dq_deg must hold 6 values")
        self.fk = fk                       # callable(q_deg (6,), dq_deg or None) -> 4x4 m; ArmChain.fk_flange
        self.max_corr_mm = float(max_corr_mm)
        self.max_corr_deg = float(max_corr_deg)
        self.iters = int(iters)
        # xy only: the commanded ARM-FRAME z stays the target's z (the
        # offsets' z term is not trusted — module docstring).
        self.skip_z = bool(skip_z)

    @property
    def enabled(self):
        return bool(np.any(self.dq)) and self.fk is not None

    # ---- joint targets --------------------------------------------------
    def joints_for_physical(self, q_deg):
        """The reading to command so the PHYSICAL joints end at q_deg."""
        q = np.asarray(q_deg, dtype=np.float64).ravel()
        return (q - self.dq).tolist() if np.any(self.dq) else q.tolist()

    # ---- Cartesian targets ----------------------------------------------
    def offset_effect(self, q_deg):
        """D = inv(FK(q)) . FK(q + dq): where the physical flange sits relative
        to the nominal one, in the flange frame, at configuration q."""
        q = np.asarray(q_deg, dtype=np.float64).ravel()
        return _inv(self.fk(q)) @ self.fk(q, self.dq)

    def pose_for_physical(self, target_pose_mm_deg, ik):
        """(cmd_pose, info) — the pose to command so the PHYSICAL flange lands
        on ``target``. ``ik(pose_mm_deg)`` -> joints (deg, 6) or None.
        Raises ValueError when the IK fails or the correction is
        implausibly large (a wrong URDF / dq would send the arm elsewhere)."""
        target = [float(v) for v in target_pose_mm_deg]
        if not self.enabled:
            return list(target), dict(applied=False, corr_mm=0.0, corr_deg=0.0, iters=0)
        T_tgt = pose_to_T(target)
        cmd = T_tgt.copy()
        D = np.eye(4)
        for k in range(max(1, self.iters)):
            q = ik(T_to_pose(cmd))
            if q is None:
                raise ValueError("IK failed for the offset-corrected target (iteration %d)" % (k + 1))
            D = self.offset_effect(q)
            cmd = T_tgt @ _inv(D)
            dz_skipped_mm = float(cmd[2, 3] - T_tgt[2, 3]) * 1e3
            if self.skip_z:
                # keep the target's arm-frame z; D is then re-evaluated at the
                # configuration that is actually commanded (next iteration)
                cmd[2, 3] = T_tgt[2, 3]
        # plausibility is judged on the FULL correction, the dropped z included
        full = np.array([cmd[0, 3] - T_tgt[0, 3], cmd[1, 3] - T_tgt[1, 3],
                         (cmd[2, 3] - T_tgt[2, 3]) if not self.skip_z else dz_skipped_mm * 1e-3])
        corr_full_mm = float(np.linalg.norm(full)) * 1e3
        corr_deg = _rot_deg(T_tgt[:3, :3].T @ cmd[:3, :3])
        if corr_full_mm > self.max_corr_mm or corr_deg > self.max_corr_deg:
            raise ValueError("offset correction %.1f mm / %.2f deg exceeds the %.0f mm / %.1f deg plausibility bound — "
                             "wrong joint offsets or URDF? target not sent" % (corr_full_mm, corr_deg, self.max_corr_mm, self.max_corr_deg))
        corr_mm = float(np.linalg.norm(cmd[:3, 3] - T_tgt[:3, 3])) * 1e3   # what is actually applied
        if not self.skip_z:
            dz_skipped_mm = 0.0
        return T_to_pose(cmd), dict(applied=True, corr_mm=corr_mm, corr_deg=corr_deg, iters=k + 1,
                                    skip_z=self.skip_z, dz_skipped_mm=dz_skipped_mm,
                                    q_deg=[float(v) for v in q])

    def joints_for_physical_pose(self, target_pose_mm_deg, ik):
        """(joints, info) — the JOINT reading to command so the physical
        flange lands on ``target`` (a pose), honouring ``skip_z``: the
        xy-corrected pose of `pose_for_physical`, solved once more by the
        same IK. With skip_z False this equals `joints_for_physical(IK(
        target))` to the IK's precision; with skip_z True it is the only
        form there is, since a z-free correction has no joint-space
        shortcut. Raises ValueError like `pose_for_physical`."""
        cmd, info = self.pose_for_physical(target_pose_mm_deg, ik)
        if not info['applied']:
            q = ik(list(cmd))
            if q is None:
                raise ValueError("IK failed for the target")
            return [float(v) for v in q], info
        q = ik(list(cmd))
        if q is None:
            raise ValueError("IK failed for the offset-corrected target (final solve)")
        info = dict(info, q_cmd_deg=[float(v) for v in q])
        return [float(v) for v in q], info

    def describe(self):
        return "joint offsets on the command side: dq %s deg (physical = reading + dq)%s%s" % (
            np.round(self.dq, 3).tolist(),
            ", xy only (arm-frame z left as commanded)" if self.skip_z else ", full 6-DOF",
            "" if self.enabled else " — DISABLED (all zero or no FK)")


def load_default(warn=None, skip_z=False):
    """CommandCorrector from config/tf/arm_joint_offsets.yaml + the planner
    URDF; a disabled corrector (dq = 0) when either is unavailable.
    ``skip_z``: correct xy (and rotation) only — see the module docstring."""
    from apriltag_nav import tf_chain as TC
    try:
        dq = TC.load_joint_offsets()
    except Exception as e:                       # malformed file: refuse loudly, but do not kill the node
        if warn:
            warn("joint offsets not loaded (%s) — command-side correction OFF" % e)
        return CommandCorrector(np.zeros(6), None, skip_z=skip_z)
    if not np.any(dq):
        return CommandCorrector(dq, None, skip_z=skip_z)
    try:
        from apriltag_nav.arm_fk import ArmChain
        fk = ArmChain().fk_flange
    except Exception as e:
        if warn:
            warn("planner URDF not readable (%s) — command-side joint offsets OFF" % e)
        return CommandCorrector(dq, None, skip_z=skip_z)
    return CommandCorrector(dq, fk, skip_z=skip_z)
