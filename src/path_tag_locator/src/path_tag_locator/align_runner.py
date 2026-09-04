"""
align_runner.py
===============
Shared iterative-alignment driver. Originally lived inside
``scripts/path_tag_locator_node.PathTagLocatorNode._auto_align``; lifted
into a module so the map-calibration orchestrator can reuse the exact
same logic.

Logic (per iteration):
    1. Initial move to ``initial_tcp_mm_deg`` (clamped by initial-step
       thresholds). Skipped if ``skip_initial_move=True``.
    2. Wait for a hand-cam detection of tag A on the shared detector's
       detections topic; not detected within timeout -> RuntimeError.
    3. Compute alignment metrics (xy_offset, tilt).
    4. If both metrics ≤ tolerances -> converged.
    5. Else compute target EE pose via ``compute_target_ee_pose``,
       clamp by per-step thresholds, move to the clamped target,
       settle, and loop.

``tcp_client`` is duck-typed: anything with ``get_tcp_pose()`` and
``move_j_to_pose(pose, settle_s=...)`` works — since the refactor that
is ``arm_interface.ArmInterface`` (arm_node proxy), not an SDK client.

The function uses rospy logging (loginfo / logwarn) but never calls
init_node or rospy.spin — callers run it from whatever node they own.
"""
import rospy

from .align import (
    alignment_metrics,
    clamp_step,
    compute_target_ee_pose,
    is_converged,
)
from .detections import (detection_to_T_cam2tag, wait_for_tag_detection,
                         wait_for_tag_detections, median_tilt_detection)
from .geometry import matrix_m_to_pose_fr5, pose_fr5_to_matrix_m


def _fmt_pose(p):
    return ("[x={:.2f}mm y={:.2f}mm z={:.2f}mm "
            "rx={:.2f}deg ry={:.2f}deg rz={:.2f}deg]").format(*p)


class AutoAlignError(RuntimeError):
    """run_auto_align failed part-way. ``report`` carries what the loop
    had measured up to the failure (same keys as the success dict, with
    ``error`` added) so the session record keeps the iteration history —
    the numbers that diagnose a diverging loop."""

    def __init__(self, message, report):
        super().__init__(message)
        self.report = report


def approach_pose(tcp_client, target_tcp_mm_deg, align_cfg, what="initial"):
    """Move the arm to ``target_tcp_mm_deg`` in up to
    ``align_cfg.max_initial_steps`` clamped MoveJ steps of
    ``max_initial_step_m`` / ``max_initial_step_deg`` each.

    Returns ``(steps_taken, reached)``. ``reached`` is False when the
    target was still beyond the clamp after the last allowed step — the
    arm is then short of the seed and the caller should expect the tag
    to be out of view. Move exceptions propagate to the caller.

    Why chunked: the clamp exists so a bad seed cannot become one
    unbounded jump. It used to be applied ONCE, which silently turned
    "seed 1.15 m from the home pose" into "stop 0.35 m short of it" —
    the 2026-09-04 plate-1 session lost 6 of 26 entries exactly that way
    (log: "initial move clamped (Δt=0.800 m)" then "tag A not detected").
    """
    max_steps = max(1, int(getattr(align_cfg, "max_initial_steps", 1)))
    T_tgt = pose_fr5_to_matrix_m([float(v) for v in target_tcp_mm_deg])
    for k in range(1, max_steps + 1):
        T_cur = pose_fr5_to_matrix_m(tcp_client.get_tcp_pose())
        step = clamp_step(T_cur, T_tgt,
                          max_step_m=align_cfg.max_initial_step_m,
                          max_step_deg=align_cfg.max_initial_step_deg)
        step_pose = matrix_m_to_pose_fr5(step.T_ab2ee_step)
        if step.clamped:
            rospy.logwarn(
                "auto_align: %s move clamped (Δt=%.3f m, Δrot=%.2f deg) — "
                "step %d/%d", what, step.delta_t_norm_m, step.delta_rot_deg,
                k, max_steps)
        rospy.loginfo("auto_align: %s MoveJ %d/%d -> %s", what, k, max_steps,
                      _fmt_pose(step_pose))
        # Big repositioning move: joint-interpolated. A straight-line MoveL
        # here crawled (22-34 s, two 60 s timeouts) on 2026-09-02.
        tcp_client.move_j_to_pose(step_pose,
                                  settle_s=align_cfg.move_settle_s,
                                  linear=False)
        if not step.clamped:
            return k, True
    rospy.logwarn(
        "auto_align: %s move still clamped after %d steps — arm is short of "
        "the target; raise align.max_initial_steps or max_initial_step_m",
        what, max_steps)
    return max_steps, False


def run_auto_align(*,
                   align_cfg,
                   tcp_client,
                   T_hc2ee,
                   tag_a_id: int,
                   tag_a_size_m: float,
                   hand_cam_detections_topic: str,
                   hand_cam_detector_size_m: float,
                   detection_wait_timeout_s: float,
                   initial_tcp_mm_deg,
                   skip_initial_move: bool = False) -> dict:
    """Drive the hand camera squarely onto tag A. Returns a report dict::

        {"iterations": int, "xy_offset_m": float, "tilt_deg": float,
         "final_tcp": list[float],
         "tag_in_cam": dict,          # final observation, CAMERA frame
         "history": list[dict]}       # one tag_in_cam dict per iteration

    ``tag_in_cam`` / ``history`` carry the full camera-frame error
    (position_m x/y/z + rpy_deg of the tag in the hand-cam optical frame,
    see ``align.CAM_FRAME_NOTE``); ``xy_offset_m`` / ``tilt_deg`` are the
    scalars derived from it.

    Raises ``RuntimeError`` if the arm interface is missing, the initial
    pose argument is invalid, or tag A is not detected during the
    refinement loop.
    """
    if tcp_client is None:
        raise RuntimeError("auto_align requires an arm interface")

    initial = [float(v) for v in initial_tcp_mm_deg]
    if len(initial) != 6 or all(v == 0.0 for v in initial):
        raise RuntimeError(
            "auto_align: initial_tcp_mm_deg must be 6 floats; "
            "got all zeros (set explicit pose)")

    # Set when an arm move fails but the tag is still observable: the
    # loop stops refining and the caller computes the chain from the
    # LAST REACHABLE pose (accuracy caveat rides in the report).
    degraded = None

    if not skip_initial_move:
        try:
            approach_pose(tcp_client, initial, align_cfg, what="initial")
        except Exception as e:
            if not getattr(align_cfg, 'continue_on_move_failure', True):
                raise
            # Reach-marginal entries (planner comments flag them) can fail
            # IK on the ideal view pose. The chain does NOT require a
            # square view — T_hc2A is a full 6-DOF observation — so if
            # the tag turns out to be visible from wherever the arm
            # actually is, degrade instead of failing the entry. The
            # detection attempt below is the arbiter: no tag -> _fail.
            degraded = f"initial move failed, using current pose: {e}"
            rospy.logwarn("auto_align: %s", degraded)

    last_metrics = None
    history = []
    iters = 0

    def _fail(msg):
        report = {
            "iterations": iters,
            "xy_offset_m": last_metrics.xy_offset_m if last_metrics else 0.0,
            "tilt_deg": last_metrics.tilt_deg if last_metrics else 0.0,
            "final_tcp": None,
            "tag_in_cam": last_metrics.as_report() if last_metrics else None,
            "history": history,
            "degraded": degraded,
            "error": msg,
        }
        return AutoAlignError(msg, report)

    for i in range(int(align_cfg.max_iterations)):
        iters = i + 1
        try:
            n_samples = int(getattr(align_cfg, 'samples_per_iteration', 1))
            dets = wait_for_tag_detections(
                hand_cam_detections_topic, int(tag_a_id), n_samples,
                timeout=detection_wait_timeout_s)
            det = median_tilt_detection(dets)
        except RuntimeError as e:
            raise _fail(
                f"auto_align: tag A (id={tag_a_id}) not detected at "
                f"iteration {iters} — adjust initial pose ({e})")
        T_cam2tag = detection_to_T_cam2tag(
            det, float(tag_a_size_m), float(hand_cam_detector_size_m))
        metrics = alignment_metrics(T_cam2tag)
        last_metrics = metrics
        history.append(dict(iteration=iters, **metrics.as_report()))
        rospy.loginfo(
            "auto_align iter %d/%d: xy=%.4f m, tilt=%.3f deg, z=%.3f m",
            iters, align_cfg.max_iterations,
            metrics.xy_offset_m, metrics.tilt_deg, metrics.z_distance_m)
        if is_converged(metrics, align_cfg.position_tol_m,
                        align_cfg.angle_tol_deg):
            rospy.loginfo("auto_align: converged at iteration %d", iters)
            break

        cur_tcp = tcp_client.get_tcp_pose()
        T_cur = pose_fr5_to_matrix_m(cur_tcp)
        T_target = compute_target_ee_pose(
            T_cur, T_hc2ee, T_cam2tag,
            target_distance_m=align_cfg.target_distance_m)
        step = clamp_step(T_cur, T_target,
                          max_step_m=align_cfg.max_step_m,
                          max_step_deg=align_cfg.max_step_deg)
        if step.clamped:
            rospy.logwarn(
                "auto_align: step %d clamped (Δt=%.3f m, Δrot=%.2f deg)",
                iters, step.delta_t_norm_m, step.delta_rot_deg)
        step_pose = matrix_m_to_pose_fr5(step.T_ab2ee_step)
        try:
            # Small camera-frame correction: keep the TCP path straight.
            tcp_client.move_j_to_pose(step_pose, settle_s=align_cfg.move_settle_s,
                                      linear=True)
        except Exception as e:
            if not getattr(align_cfg, 'continue_on_move_failure', True):
                raise _fail(f"auto_align: step {iters} move failed: {e}")
            # The tag IS visible (we just measured it) — keep the last
            # reachable pose and let the chain run from here instead of
            # failing the whole entry. Typical trigger: reach-marginal
            # entries whose ideal square-view pose the IK cannot serve.
            degraded = (f"step {iters} move failed, continuing with "
                        f"last reachable pose: {e}")
            rospy.logwarn("auto_align: %s (xy=%.4f m, tilt=%.2f deg)",
                          degraded, metrics.xy_offset_m, metrics.tilt_deg)
            break

    final_tcp = tcp_client.get_tcp_pose()
    return {
        "iterations": iters,
        "xy_offset_m": last_metrics.xy_offset_m if last_metrics else 0.0,
        "tilt_deg": last_metrics.tilt_deg if last_metrics else 0.0,
        "final_tcp": final_tcp,
        "tag_in_cam": last_metrics.as_report() if last_metrics else None,
        "history": history,
        "degraded": degraded,
    }
