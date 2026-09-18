"""
handeye_sweep.py
================
Automatic hand-eye sample collection: orbit the hand camera around the
calibration tag and capture at every view, without the operator jogging
the arm between captures.

Pure numpy; no ROS imports. ``SweepRunner`` takes the arm / camera /
capture as callables so the whole procedure runs offline against fakes
(``scripts/check_handeye_sweep.py``). ``handeye_calib_node`` wires it to
``ArmInterface``, the shared detector's ``/hand_cam/tag_detections`` and
its own raw-frame capture.

The procedure
-------------
1. The tag must already be in view (the operator drives the camera over
   it — Arm tab jog, or a calibration-plan seed).
2. Square up: the align loop (``align.compute_target_ee_pose``) puts the
   tag on the optical axis at ``distances_m[0]`` with zero tilt. That
   pose is sample 0 and the reference the sweep is planned from:
   ``T_ab2tag = T_ab2ee · inv(T_hc2ee) · T_cam2tag``.
3. Views: for every distance × tilt × azimuth the camera is placed on a
   sphere around the tag centre, optical axis through the tag centre, and
   additionally spun about its own axis (round-robin over ``spins_deg``).
   A hand-eye solve is conditioned by ROTATION diversity — pure
   translations give it nothing — so tilt and spin are what the grid is
   built around; distance mostly varies the tag's pixel size.
4. Safety, in the tag plane's frame (the plate IS the tag plane):
   every ``tool_points_mm`` point (flange origin and the vision-tip /
   Keyence bracket, the lowest part of the tool) must stay
   ``min_clearance_m`` above the plate at every view; the flange must
   stay within ``max_xy_from_start_m`` of the start pose horizontally
   (the sweep never leaves the plate area the operator chose) and inside
   ``max_flange_reach_m`` of the arm base. Views failing any rule are
   dropped, with the reason counted. Between views the flange moves in
   straight ``MoveL`` chunks of at most ``max_step_m`` / ``max_step_deg``
   (the arm interface's ``linear=True``): both ends of every chunk are
   inside the safe region, and the region is convex in translation, so
   the straight flange path is too. Views are ordered nearest-first so
   the moves stay short.
5. At each view: settle, re-detect (median of several frames); no tag
   -> skipped, not captured; tag -> capture (image + K + TCP). A failed
   move skips that view and plans the next one from wherever the arm
   actually is. Cancel is checked before every move. Finally the arm
   returns to the start pose.

⚠️ The hand-eye used to AIM the sweep is the current estimate: an
error there only mis-centres the tag in the image, which the per-view
re-detection catches (skip), and the clearance rule is evaluated with
the same estimate, hence the generous margin. The captured samples
themselves do not depend on it.

Bootstrap (2026-09-18) — when the current hand-eye describes a DIFFERENT
mount (the camera was moved: the 2026-09-18 session, xy 289 -> 373 mm,
tilt 3 -> 7.7 deg, tag lost on the second step; the 2026-09-02 session
with the 180 deg-spun May file was the same signature), the square-up
does not converge, it DIVERGES, and it used to run until the tag left
the frame. Now every align step is checked against the best one so far: an
error that grows by ``align_diverge_ratio`` (and by more than
``align_diverge_min_growth_mm`` in mm-equivalent, so detection noise
near convergence cannot trip it), a tag lost right after a step, or
the iterations running out with less than ``align_stall_min_improvement``
of the initial error removed (a spun hand-eye moves the camera
SIDEWAYS to the error, and the number never changes) retreats to the
best pose seen and raises ``AimDiverged``. With
``bootstrap: auto`` the runner then makes its own aiming estimate at
that pose: it captures the current view and, about the FLANGE axes
(x, y, z, +/- ``bootstrap_angle_deg``), six more — pure flange
rotations, so the moves are small and need no hand-eye to plan — and
``solve`` (cv2.calibrateHandEye over just those samples) returns a
provisional T_hc2ee. It is only cm / degree accurate, which is all the
aim needs: the square-up re-measures every step, the views re-detect,
and the clearance rule gets ``bootstrap_clearance_extra_m`` on top.
Those samples are dropped from the set once solved
(``bootstrap_keep_samples`` false): they are small rotations at whatever
height the operator started from — the 2026-09-18 ones at 1.2 m
scattered the tag 17–22 mm each and pulled the final solve 27 mm off —
and the sweep's views are the calibration. ``bootstrap: always`` skips the file entirely; ``never`` fails with the
manual procedure named (jog + ~capture at >= 8 poses with tilt and
spin, ~compute, then the sweep). Precondition: the camera at least
``bootstrap_min_depth_m`` from the tag, so a 10 deg flange rotation
(the tool tip moves a few cm) has room.
"""
import math
from dataclasses import dataclass, field, replace
from typing import Callable, List, Optional

import numpy as np

from .align import (alignment_metrics, clamp_step, compute_target_ee_pose,
                    is_converged)
from .geometry import invert_T, matrix_m_to_pose_fr5, pose_fr5_to_matrix_m


@dataclass
class SweepCfg:
    distances_m: List[float] = field(default_factory=lambda: [0.35, 0.45, 0.55])
    tilts_deg: List[float] = field(default_factory=lambda: [0.0, 12.0, 22.0])
    azimuths_deg: List[float] = field(default_factory=lambda: [0.0, 90.0, 180.0, 270.0])
    spins_deg: List[float] = field(default_factory=lambda: [-30.0, 0.0, 30.0])
    max_samples: int = 24
    # Flange-frame points (mm) that must clear the plate: the flange
    # itself and the vision tip (robot.yaml arm_calibration.vision_tip_offset_mm).
    tool_points_mm: List[List[float]] = field(
        default_factory=lambda: [[0.0, 0.0, 0.0], [0.0, -253.0, 225.2]])
    min_clearance_m: float = 0.12
    max_xy_from_start_m: float = 0.30
    max_flange_reach_m: float = 1.25
    min_flange_reach_m: float = 0.25
    max_step_m: float = 0.20
    max_step_deg: float = 30.0
    max_chunks_per_move: int = 8
    return_to_start: bool = True
    # square-up loop before the sweep
    align_max_iterations: int = 6
    align_position_tol_m: float = 0.003
    align_angle_tol_deg: float = 1.0
    align_max_step_m: float = 0.10
    align_max_step_deg: float = 15.0
    # divergence guard: mm-equivalent error = xy_mm + 10 * tilt_deg; a step
    # after which it grew by the ratio AND by the minimum growth retreats
    align_diverge_ratio: float = 1.25
    align_diverge_min_growth_mm: float = 30.0
    # ... and the loop ending unconverged with less than this fraction of
    # the initial error removed is a stall (same treatment)
    align_stall_min_improvement: float = 0.25
    # provisional aiming hand-eye from small flange rotations when the
    # file's one diverges ('auto'), always ('always'), or never ('never')
    bootstrap: str = 'auto'
    bootstrap_angle_deg: float = 10.0
    bootstrap_min_depth_m: float = 0.5
    bootstrap_min_samples: int = 5
    bootstrap_clearance_extra_m: float = 0.05
    # keep the bootstrap's samples in the set? Default no: they are small
    # rotations at whatever height the operator started (the 2026-09-18
    # ones at 1.2 m scattered the tag 17-22 mm each and pulled the solve
    # 27 mm off); the sweep's views are the calibration.
    bootstrap_keep_samples: bool = False


@dataclass
class SweepView:
    label: str
    distance_m: float
    tilt_deg: float
    azimuth_deg: float
    spin_deg: float
    T_ab2ee: Optional[np.ndarray] = None
    reject_reason: Optional[str] = None


@dataclass
class SweepResult:
    n_planned: int = 0
    n_rejected: int = 0
    rejected_reasons: dict = field(default_factory=dict)
    n_captured: int = 0
    n_skipped: int = 0
    n_move_failed: int = 0
    cancelled: bool = False
    align_iterations: int = 0
    start_tcp: Optional[list] = None
    error: Optional[str] = None
    aim_source: str = 'file'          # 'file' | 'bootstrap'
    bootstrapped: bool = False
    n_bootstrap: int = 0              # samples the bootstrap captured
    diverged: bool = False            # the file's hand-eye diverged the square-up

    def summary(self) -> str:
        s = (f"sweep: {self.n_captured} captured, {self.n_skipped} skipped "
             f"(tag not seen), {self.n_move_failed} move failures of "
             f"{self.n_planned} planned ({self.n_rejected} views rejected by "
             f"the safety rules)")
        if self.bootstrapped:
            s += f"; aimed by a bootstrap hand-eye ({self.n_bootstrap} samples)"
        if self.cancelled:
            s += " — CANCELLED"
        if self.error:
            s += f" — ERROR: {self.error}"
        return s


# ----------------------------------------------------------------------
# geometry helpers
# ----------------------------------------------------------------------
def _Rz(a_rad):
    c, s = math.cos(a_rad), math.sin(a_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _R_axis(axis, a_rad):
    axis = np.asarray(axis, dtype=np.float64)
    n = np.linalg.norm(axis)
    if n < 1e-12 or abs(a_rad) < 1e-12:
        return np.eye(3)
    x, y, z = axis / n
    c, s = math.cos(a_rad), math.sin(a_rad)
    C = 1.0 - c
    return np.array([
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def view_T_cam2tag(yaw_rad, distance_m, tilt_deg, azimuth_deg, spin_deg):
    """Desired pose of the TAG in the CAMERA frame for one view.

    The tag centre sits on the optical axis at ``distance_m`` (image
    centre). The camera is on the sphere of that radius around the tag
    centre, at polar angle ``tilt_deg`` from the tag normal in direction
    ``azimuth_deg``, and rotated ``spin_deg`` about its own optical axis.
    ``yaw_rad`` is the spin the square view had (align preserves it — it
    is what keeps the flange inside reach), the sweep spins relative to
    it.
    """
    t = math.radians(tilt_deg)
    az = math.radians(azimuth_deg)
    axis = (math.cos(az), math.sin(az), 0.0)          # in the camera frame
    R = _Rz(math.radians(spin_deg)) @ _R_axis(axis, t) @ _Rz(yaw_rad)
    T = np.eye(4)
    T[:3, :3] = R
    T[2, 3] = float(distance_m)
    return T


def generate_views(cfg: SweepCfg) -> List[SweepView]:
    """The view grid, spins round-robin, evenly subsampled to max_samples."""
    views = []
    k = 0
    spins = list(cfg.spins_deg) or [0.0]
    for d in cfg.distances_m:
        combos = [(0.0, 0.0)]
        for tilt in cfg.tilts_deg:
            if abs(tilt) < 1e-9:
                continue
            for az in cfg.azimuths_deg:
                combos.append((float(tilt), float(az)))
        for tilt, az in combos:
            spin = float(spins[k % len(spins)])
            k += 1
            views.append(SweepView(
                label=f"d{d * 1000:.0f} t{tilt:.0f} a{az:.0f} s{spin:+.0f}",
                distance_m=float(d), tilt_deg=tilt, azimuth_deg=az,
                spin_deg=spin))
    n_max = int(cfg.max_samples)
    if 0 < n_max < len(views):
        idx = sorted({int(round(i * (len(views) - 1) / (n_max - 1)))
                      for i in range(n_max)})
        views = [views[i] for i in idx]
    return views


def plan_sweep(T_ab2tag, T_hc2ee, T_cam2tag_square, T_ab2ee_start, cfg: SweepCfg):
    """Turn the view grid into flange targets and apply the safety rules.

    Returns (accepted views ordered nearest-first, rejected views). Every
    view carries either ``T_ab2ee`` or ``reject_reason``.
    """
    T_ab2tag = np.asarray(T_ab2tag, dtype=np.float64)
    T_hc2ee = np.asarray(T_hc2ee, dtype=np.float64)
    tag_o = T_ab2tag[:3, 3]
    # "Up" = the side of the tag plane the camera is on at the start.
    cam_start = (T_ab2ee_start @ invert_T(T_hc2ee))[:3, 3]
    n = T_ab2tag[:3, 2].copy()
    if float(np.dot(n, cam_start - tag_o)) < 0.0:
        n = -n
    n /= max(np.linalg.norm(n), 1e-12)
    start_xy = T_ab2ee_start[:2, 3]
    yaw = math.atan2(float(T_cam2tag_square[1, 0]), float(T_cam2tag_square[0, 0]))
    tool_pts = [np.asarray(p, dtype=np.float64) / 1000.0 for p in cfg.tool_points_mm]

    accepted, rejected = [], []
    for v in generate_views(cfg):
        T_target = view_T_cam2tag(yaw, v.distance_m, v.tilt_deg, v.azimuth_deg, v.spin_deg)
        T_ab2ee = T_ab2tag @ invert_T(T_target) @ T_hc2ee
        reason = None
        # 1. every tool point above the plate
        for p in tool_pts:
            P = T_ab2ee[:3, :3] @ p + T_ab2ee[:3, 3]
            h = float(np.dot(n, P - tag_o))
            if h < cfg.min_clearance_m:
                reason = f"clearance {h * 1000:.0f} mm < {cfg.min_clearance_m * 1000:.0f} mm"
                break
        # 2. flange stays near where the operator put it (over the plate)
        if reason is None:
            dxy = float(np.linalg.norm(T_ab2ee[:2, 3] - start_xy))
            if dxy > cfg.max_xy_from_start_m:
                reason = f"xy excursion {dxy:.2f} m > {cfg.max_xy_from_start_m:.2f} m"
        # 3. reach
        if reason is None:
            r = float(np.linalg.norm(T_ab2ee[:3, 3]))
            if r > cfg.max_flange_reach_m:
                reason = f"reach {r:.2f} m > {cfg.max_flange_reach_m:.2f} m"
            elif r < cfg.min_flange_reach_m:
                reason = f"reach {r:.2f} m < {cfg.min_flange_reach_m:.2f} m"
        if reason is None:
            v.T_ab2ee = T_ab2ee
            accepted.append(v)
        else:
            v.reject_reason = reason
            rejected.append(v)

    # nearest-first ordering (translation + a rotation term), from the start
    ordered = []
    T_cur = np.asarray(T_ab2ee_start, dtype=np.float64)
    pool = list(accepted)
    while pool:
        best_i, best_c = 0, float("inf")
        for i, v in enumerate(pool):
            c = _move_cost(T_cur, v.T_ab2ee)
            if c < best_c:
                best_i, best_c = i, c
        v = pool.pop(best_i)
        ordered.append(v)
        T_cur = v.T_ab2ee
    return ordered, rejected


def _move_cost(T_a, T_b):
    d = invert_T(T_a) @ T_b
    t = float(np.linalg.norm(d[:3, 3]))
    c = float(np.clip((np.trace(d[:3, :3]) - 1.0) / 2.0, -1.0, 1.0))
    ang = math.acos(c)
    return t + 0.25 * ang        # 0.25 m per radian


MANUAL_PROCEDURE = ("capture by hand instead: jog the camera over the tag, "
                    "~capture at >= 8 poses that differ in tilt AND spin, "
                    "~compute, then run the sweep again")

# The bootstrap's flange-frame rotations: label, axis. Six pure rotations
# about the flange origin; a hand-eye solve needs rotation diversity and
# nothing else, and rotations about the flange move the camera by only
# lever x angle (a few cm), so no hand-eye is needed to keep them safe.
BOOTSTRAP_AXES = [('rx+', (1.0, 0.0, 0.0), +1.0), ('rx-', (1.0, 0.0, 0.0), -1.0),
                  ('ry+', (0.0, 1.0, 0.0), +1.0), ('ry-', (0.0, 1.0, 0.0), -1.0),
                  ('rz+', (0.0, 0.0, 1.0), +1.0), ('rz-', (0.0, 0.0, 1.0), -1.0)]


class AimDiverged(RuntimeError):
    """The square-up step made the error larger (or lost the tag): the
    aiming hand-eye does not describe this camera mount."""


def _align_error_mm(m) -> float:
    """One scalar for the divergence guard: xy in mm plus tilt at ~10 mm
    per degree (a 1 deg tilt at the 0.35-0.55 m working distance)."""
    return float(m.xy_offset_m * 1000.0 + 10.0 * m.tilt_deg)


# ----------------------------------------------------------------------
class SweepRunner:
    """Drive the procedure through callables (all synchronous):

    get_tcp()            -> [x_mm, y_mm, z_mm, rx, ry, rz] (flange, arm frame)
    move(pose_mm_deg)    -> None; raises on failure; blocks incl. settle
    detect()             -> T_cam2tag (4x4 m) or None when the tag is not seen
    capture()            -> (ok: bool, message: str)  — one hand-eye sample
    cancelled()          -> bool
    progress(dict)       -> None
    n_samples()          -> int, samples held so far (bootstrap bookkeeping)
    solve(since)         -> T_hc2ee (4x4 m) from the samples captured from
                            index ``since`` on, or None — the bootstrap's
                            provisional hand-eye (cv2.calibrateHandEye)
    discard(since)       -> drop the samples from index ``since`` on (the
                            bootstrap's, once it has been solved), optional
    """

    def __init__(self, cfg: SweepCfg, *, get_tcp, move, detect, capture,
                 cancelled=None, log_info=print, log_warn=print, progress=None,
                 n_samples=None, solve=None, discard=None):
        self.cfg = cfg
        self.get_tcp = get_tcp
        self.move = move
        self.detect = detect
        self.capture = capture
        self.cancelled = cancelled or (lambda: False)
        self.log_info = log_info
        self.log_warn = log_warn
        self.progress = progress or (lambda d: None)
        self.n_samples = n_samples
        self.solve = solve
        self.discard = discard

    # ------------------------------------------------------------------
    def _move_chunked(self, T_target) -> bool:
        """Straight MoveL chunks of at most max_step_m / max_step_deg."""
        cfg = self.cfg
        for _ in range(max(1, int(cfg.max_chunks_per_move))):
            if self.cancelled():
                return False
            T_cur = pose_fr5_to_matrix_m(self.get_tcp())
            step = clamp_step(T_cur, T_target, max_step_m=cfg.max_step_m,
                              max_step_deg=cfg.max_step_deg)
            self.move(matrix_m_to_pose_fr5(step.T_ab2ee_step))
            if not step.clamped:
                return True
        self.log_warn("sweep: target still beyond the step clamp after "
                      f"{cfg.max_chunks_per_move} chunks")
        return True

    def _square_up(self, T_hc2ee, res: SweepResult):
        """Align loop: tag centred, zero tilt, at distances_m[0].

        Divergence guard (2026-09-18, remounted camera): the error after
        every step is judged against the BEST one seen so far — a step
        that made it worse by ``align_diverge_ratio`` and by more than
        ``align_diverge_min_growth_mm`` (mm-equivalent) is a divergence,
        and so is a tag lost right after a step, and so is the loop
        running out of iterations with less than
        ``align_stall_min_improvement`` of the initial error removed (a
        90 deg-spun hand-eye moves the camera sideways to the error and
        the number never changes). All three retreat to the pose where
        the best error was measured — the tag was in view there — and
        raise ``AimDiverged``, instead of walking the tag out of the
        frame or handing a wrong hand-eye to the planner.
        """
        cfg = self.cfg
        T_cam2tag = self.detect()
        if T_cam2tag is None:
            raise RuntimeError("calibration tag not visible from the current "
                               "pose — drive the hand camera over it first")
        first_err = best_err = None
        T_best = best_m = None
        converged = False
        for it in range(1, int(cfg.align_max_iterations) + 1):
            res.align_iterations = it
            m = alignment_metrics(T_cam2tag)
            err = _align_error_mm(m)
            T_cur = pose_fr5_to_matrix_m(self.get_tcp())
            self.progress({'phase': 'align', 'iteration': it,
                           'xy_mm': m.xy_offset_m * 1000.0,
                           'tilt_deg': m.tilt_deg, 'z_m': m.z_distance_m})
            self.log_info(f"sweep: align {it}: xy {m.xy_offset_m * 1000:.1f} mm, "
                          f"tilt {m.tilt_deg:.2f} deg, z {m.z_distance_m:.3f} m")
            if best_err is not None and self._diverged(best_err, err):
                self.log_warn(f"sweep: align {it}: the steps made the error WORSE "
                              f"({best_err:.0f} -> {err:.0f} mm-equivalent) — "
                              "retreating to the best pose seen")
                self.move(matrix_m_to_pose_fr5(T_best))
                res.diverged = True
                raise AimDiverged(
                    f"the aiming hand-eye moves the tag the wrong way (align -> {it}: "
                    f"xy {best_m.xy_offset_m * 1000:.0f} -> {m.xy_offset_m * 1000:.0f} mm, "
                    f"tilt {best_m.tilt_deg:.1f} -> {m.tilt_deg:.1f} deg); "
                    "it describes a different camera mount")
            if first_err is None:
                first_err = err
            if best_err is None or err < best_err:
                best_err, T_best, best_m = err, T_cur, m
            depth_ok = abs(m.z_distance_m - cfg.distances_m[0]) < 0.02
            if is_converged(m, cfg.align_position_tol_m, cfg.align_angle_tol_deg) and depth_ok:
                converged = True
                break
            if self.cancelled():
                raise RuntimeError("cancelled during square-up")
            T_tgt = compute_target_ee_pose(T_cur, T_hc2ee, T_cam2tag,
                                           target_distance_m=cfg.distances_m[0])
            step = clamp_step(T_cur, T_tgt, max_step_m=cfg.align_max_step_m,
                              max_step_deg=cfg.align_max_step_deg)
            self.move(matrix_m_to_pose_fr5(step.T_ab2ee_step))
            T_cam2tag = self.detect()
            if T_cam2tag is None:
                self.log_warn(f"sweep: align {it}: tag lost after the step — "
                              "retreating to the best pose seen")
                self.move(matrix_m_to_pose_fr5(T_best))
                res.diverged = True
                raise AimDiverged(
                    f"tag lost during square-up (iteration {it}, from xy "
                    f"{m.xy_offset_m * 1000:.0f} mm / tilt {m.tilt_deg:.1f} deg): "
                    "the aiming hand-eye describes a different camera mount")
        if not converged and first_err is not None:
            m = alignment_metrics(T_cam2tag)
            err = _align_error_mm(m)
            need = first_err * (1.0 - float(cfg.align_stall_min_improvement))
            if err > need and first_err > 0:
                self.log_warn(f"sweep: align made no progress in {res.align_iterations} "
                              f"iterations ({first_err:.0f} -> {err:.0f} mm-equivalent) — "
                              "retreating to the best pose seen")
                self.move(matrix_m_to_pose_fr5(T_best))
                res.diverged = True
                raise AimDiverged(
                    f"the aiming hand-eye makes no progress ({res.align_iterations} steps: "
                    f"xy {best_m.xy_offset_m * 1000:.0f} mm / tilt {best_m.tilt_deg:.1f} deg "
                    f"at best, now {m.xy_offset_m * 1000:.0f} mm / {m.tilt_deg:.1f} deg); "
                    "it describes a different camera mount")
        return T_cam2tag

    def _diverged(self, prev_err, err) -> bool:
        cfg = self.cfg
        return (err > prev_err * float(cfg.align_diverge_ratio)
                and err - prev_err > float(cfg.align_diverge_min_growth_mm))

    def _bootstrap(self, res: SweepResult):
        """Provisional aiming hand-eye from small flange rotations at the
        current pose. Captures the view here and six flange-frame
        rotations (BOOTSTRAP_AXES x bootstrap_angle_deg), returns to the
        start, and asks ``solve`` for T_hc2ee over just those samples."""
        cfg = self.cfg
        if self.solve is None or self.n_samples is None:
            raise RuntimeError("bootstrap needs the solve / n_samples callables")
        T_cam2tag = self.detect()
        if T_cam2tag is None:
            raise RuntimeError("bootstrap: calibration tag not visible from the "
                               "current pose")
        m = alignment_metrics(T_cam2tag)
        if m.z_distance_m < float(cfg.bootstrap_min_depth_m):
            raise RuntimeError(
                f"bootstrap: the camera is {m.z_distance_m:.2f} m from the tag; "
                f"it needs >= {cfg.bootstrap_min_depth_m:.2f} m of room for "
                f"{cfg.bootstrap_angle_deg:.0f} deg flange rotations — raise it first")
        T0 = pose_fr5_to_matrix_m(self.get_tcp())
        since = int(self.n_samples())
        n_views = len(BOOTSTRAP_AXES) + 1
        self.log_info(f"bootstrap: provisional hand-eye from {n_views} views "
                      f"(+/-{cfg.bootstrap_angle_deg:.0f} deg about the flange axes), "
                      f"tag at {m.z_distance_m:.2f} m")
        self.progress({'phase': 'bootstrap', 'index': 0, 'total': n_views,
                       'label': 'start', 'ok': None, 'z_m': m.z_distance_m})
        ok, msg = self.capture()
        self.progress({'phase': 'bootstrap', 'index': 0, 'total': n_views,
                       'label': 'start', 'ok': bool(ok), 'reason': msg})
        a = math.radians(float(cfg.bootstrap_angle_deg))
        try:
            for i, (label, axis, sign) in enumerate(BOOTSTRAP_AXES, start=1):
                if self.cancelled():
                    raise RuntimeError("cancelled during bootstrap")
                T_v = T0.copy()
                T_v[:3, :3] = T0[:3, :3] @ _R_axis(axis, sign * a)
                try:
                    self.move(matrix_m_to_pose_fr5(T_v))
                except Exception as e:
                    self.log_warn(f"bootstrap: view {i}/{n_views} {label}: move failed: {e}")
                    self.progress({'phase': 'bootstrap', 'index': i, 'total': n_views,
                                   'label': label, 'ok': False, 'reason': f'move failed: {e}'})
                    continue
                seen = self.detect()
                if seen is None:
                    self.log_warn(f"bootstrap: view {i}/{n_views} {label}: tag not seen, skipped")
                    self.progress({'phase': 'bootstrap', 'index': i, 'total': n_views,
                                   'label': label, 'ok': False, 'reason': 'tag not seen'})
                    continue
                ms = alignment_metrics(seen)
                ok, msg = self.capture()
                self.log_info(f"bootstrap: view {i}/{n_views} {label}: "
                              f"{'captured' if ok else 'capture failed: ' + msg} "
                              f"(tag xy {ms.xy_offset_m * 1000:.0f} mm, tilt {ms.tilt_deg:.1f} deg)")
                self.progress({'phase': 'bootstrap', 'index': i, 'total': n_views,
                               'label': label, 'ok': bool(ok), 'reason': msg,
                               'xy_mm': ms.xy_offset_m * 1000.0, 'tilt_deg': ms.tilt_deg})
        finally:
            # back where the operator left it, whatever happened
            self.move(matrix_m_to_pose_fr5(T0))
        n_new = int(self.n_samples()) - since
        res.n_bootstrap = n_new
        if n_new < int(cfg.bootstrap_min_samples):
            raise RuntimeError(
                f"bootstrap: only {n_new} of {n_views} views captured the tag "
                f"(need {cfg.bootstrap_min_samples}) — centre the tag in the image "
                f"and raise the camera, or {MANUAL_PROCEDURE}")
        T_prov = self.solve(since)
        if T_prov is None:
            raise RuntimeError(f"bootstrap: the provisional hand-eye solve failed "
                               f"over {n_new} samples — {MANUAL_PROCEDURE}")
        T_prov = np.asarray(T_prov, dtype=np.float64)
        res.bootstrapped = True
        res.aim_source = 'bootstrap'
        t = T_prov[:3, 3] * 1000.0
        kept = ''
        if not cfg.bootstrap_keep_samples and self.discard is not None:
            self.discard(since)
            kept = f"; its {n_new} samples dropped from the set (aiming only)"
        self.log_info(f"bootstrap: provisional T_hc2ee from {n_new} samples: "
                      f"t = ({t[0]:.0f}, {t[1]:.0f}, {t[2]:.0f}) mm — aiming with it "
                      f"(clearance margin +{cfg.bootstrap_clearance_extra_m * 1000:.0f} mm){kept}")
        self.progress({'phase': 'bootstrap', 'index': n_views, 'total': n_views,
                       'label': 'solved', 'ok': True, 'n_bootstrap': n_new,
                       't_mm': [float(v) for v in t]})
        return T_prov

    def _aim(self, T_hc2ee, res: SweepResult):
        """Square up with the given hand-eye, falling back to a bootstrap
        one when it diverges (bootstrap 'auto') — returns
        (T_hc2ee_used, T_cam2tag_square)."""
        cfg = self.cfg
        mode = str(cfg.bootstrap or 'auto').lower()
        if mode not in ('auto', 'always', 'never'):
            raise RuntimeError(f"bootstrap must be auto / always / never, not {cfg.bootstrap!r}")
        if T_hc2ee is None and mode == 'never':
            raise RuntimeError("no aiming hand-eye and bootstrap is 'never' — "
                               + MANUAL_PROCEDURE)
        if mode == 'always' or T_hc2ee is None:
            if T_hc2ee is None:
                self.log_warn("sweep: no aiming hand-eye — bootstrapping one")
            T_hc2ee = self._bootstrap(res)
            return T_hc2ee, self._square_up(T_hc2ee, res)
        try:
            return T_hc2ee, self._square_up(T_hc2ee, res)
        except AimDiverged as e:
            if mode == 'never':
                raise RuntimeError(f"{e} — {MANUAL_PROCEDURE}")
            self.log_warn(f"sweep: {e} — bootstrapping a provisional hand-eye")
            self.progress({'phase': 'diverged', 'reason': str(e)})
            T_hc2ee = self._bootstrap(res)
            try:
                return T_hc2ee, self._square_up(T_hc2ee, res)
            except AimDiverged as e2:
                raise RuntimeError(f"bootstrap hand-eye diverges too ({e2}) — {MANUAL_PROCEDURE}")

    # ------------------------------------------------------------------
    def run(self, T_hc2ee=None) -> SweepResult:
        """``T_hc2ee`` is the aiming estimate (the file); None = none on
        hand, bootstrap one (unless ``bootstrap: never``)."""
        cfg = self.cfg
        res = SweepResult()
        if T_hc2ee is not None:
            T_hc2ee = np.asarray(T_hc2ee, dtype=np.float64)
        else:
            res.aim_source = 'none'
        try:
            T_hc2ee, T_cam2tag_sq = self._aim(T_hc2ee, res)
            start_tcp = self.get_tcp()
            res.start_tcp = list(start_tcp)
            T_start = pose_fr5_to_matrix_m(start_tcp)
            T_ab2tag = T_start @ invert_T(T_hc2ee) @ T_cam2tag_sq

            plan_cfg = cfg
            if res.bootstrapped and cfg.bootstrap_clearance_extra_m > 0:
                plan_cfg = replace(cfg, min_clearance_m=float(cfg.min_clearance_m)
                                   + float(cfg.bootstrap_clearance_extra_m))
            views, rejected = plan_sweep(T_ab2tag, T_hc2ee, T_cam2tag_sq, T_start, plan_cfg)
            res.n_planned = len(views)
            res.n_rejected = len(rejected)
            for v in rejected:
                key = v.reject_reason.split(' ')[0]
                res.rejected_reasons[key] = res.rejected_reasons.get(key, 0) + 1
            self.log_info(f"sweep: {len(views)} views planned, {len(rejected)} "
                          f"rejected {res.rejected_reasons}")
            self.progress({'phase': 'start', 'n_planned': len(views),
                           'n_rejected': len(rejected),
                           'rejected': dict(res.rejected_reasons),
                           'start_tcp': res.start_tcp,
                           'aim_source': res.aim_source})

            # sample 0: the square view itself
            ok, msg = self.capture()
            if ok:
                res.n_captured += 1
            else:
                self.log_warn(f"sweep: capture at the square view failed: {msg}")
            self.progress({'phase': 'sample', 'index': 0, 'total': len(views),
                           'label': 'square', 'ok': bool(ok), 'reason': msg,
                           'n_captured': res.n_captured, 'n_skipped': res.n_skipped})

            for i, v in enumerate(views, start=1):
                if self.cancelled():
                    res.cancelled = True
                    break
                try:
                    reached = self._move_chunked(v.T_ab2ee)
                except Exception as e:
                    res.n_move_failed += 1
                    self.log_warn(f"sweep: view {i}/{len(views)} {v.label}: move failed: {e}")
                    self.progress({'phase': 'sample', 'index': i, 'total': len(views),
                                   'label': v.label, 'ok': False,
                                   'reason': f'move failed: {e}',
                                   'n_captured': res.n_captured, 'n_skipped': res.n_skipped})
                    continue
                if not reached:
                    res.cancelled = True
                    break
                T_seen = self.detect()
                if T_seen is None:
                    res.n_skipped += 1
                    self.log_warn(f"sweep: view {i}/{len(views)} {v.label}: tag not seen, skipped")
                    self.progress({'phase': 'sample', 'index': i, 'total': len(views),
                                   'label': v.label, 'ok': False, 'reason': 'tag not seen',
                                   'n_captured': res.n_captured, 'n_skipped': res.n_skipped})
                    continue
                m = alignment_metrics(T_seen)
                ok, msg = self.capture()
                if ok:
                    res.n_captured += 1
                else:
                    res.n_skipped += 1
                self.log_info(f"sweep: view {i}/{len(views)} {v.label}: "
                              f"{'captured' if ok else 'capture failed: ' + msg} "
                              f"(tag xy {m.xy_offset_m * 1000:.0f} mm, tilt {m.tilt_deg:.1f} deg)")
                self.progress({'phase': 'sample', 'index': i, 'total': len(views),
                               'label': v.label, 'ok': bool(ok), 'reason': msg,
                               'tilt_deg': m.tilt_deg, 'xy_mm': m.xy_offset_m * 1000.0,
                               'n_captured': res.n_captured, 'n_skipped': res.n_skipped})

            if cfg.return_to_start and not self.cancelled():
                try:
                    self._move_chunked(T_start)
                except Exception as e:
                    self.log_warn(f"sweep: return to the start pose failed: {e}")
        except Exception as e:
            res.error = str(e)
            self.log_warn(f"sweep: {e}")
        self.progress({'phase': 'finished', 'n_captured': res.n_captured,
                       'n_skipped': res.n_skipped, 'n_move_failed': res.n_move_failed,
                       'n_planned': res.n_planned, 'cancelled': res.cancelled,
                       'error': res.error, 'summary': res.summary()})
        return res
