"""
align_runner.py
===============
Shared iterative-alignment driver. Originally lived inside
``scripts/path_tag_locator_node.PathTagLocatorNode._auto_align``; lifted
into a module so the map-calibration orchestrator can reuse the exact
same logic.

Logic (per iteration):
    1. Initial MoveJ to ``initial_tcp_mm_deg`` (clamped by initial-step
       thresholds). Skipped if ``skip_initial_move=True``.
    2. Capture hand-cam Image + CameraInfo via the configured topics.
    3. Detect tag A; if not detected -> RuntimeError.
    4. Compute alignment metrics (xy_offset, tilt).
    5. If both metrics ≤ tolerances -> converged.
    6. Else compute target EE pose via ``compute_target_ee_pose``,
       clamp by per-step thresholds, MoveJ to the clamped target,
       settle, and loop.

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
from .detect import detect_apriltag
from .geometry import matrix_m_to_pose_fr5, pose_fr5_to_matrix_m
from .ros_image import grab_image_and_K


def _fmt_pose(p):
    return ("[x={:.2f}mm y={:.2f}mm z={:.2f}mm "
            "rx={:.2f}deg ry={:.2f}deg rz={:.2f}deg]").format(*p)


def run_auto_align(*,
                   align_cfg,
                   tcp_client,
                   T_hc2ee,
                   tag_a_id: int,
                   tag_a_size_m: float,
                   tag_family: str,
                   hand_cam_image_topic: str,
                   hand_cam_info_topic: str,
                   image_wait_timeout_s: float,
                   initial_tcp_mm_deg,
                   skip_initial_move: bool = False) -> dict:
    """Drive the hand camera squarely onto tag A. Returns a report dict::

        {"iterations": int, "xy_offset_m": float, "tilt_deg": float,
         "final_tcp": list[float]}

    Raises ``RuntimeError`` if the TCP client is missing, the initial
    pose argument is invalid, or tag A is not detected during the
    refinement loop.
    """
    if tcp_client is None:
        raise RuntimeError("auto_align requires robot.use_sdk=true")

    initial = [float(v) for v in initial_tcp_mm_deg]
    if len(initial) != 6 or all(v == 0.0 for v in initial):
        raise RuntimeError(
            "auto_align: initial_tcp_mm_deg must be 6 floats; "
            "got all zeros (set explicit pose)")

    if not skip_initial_move:
        cur_tcp = tcp_client.get_tcp_pose()
        T_cur = pose_fr5_to_matrix_m(cur_tcp)
        T_init = pose_fr5_to_matrix_m(initial)
        step = clamp_step(T_cur, T_init,
                          max_step_m=align_cfg.max_initial_step_m,
                          max_step_deg=align_cfg.max_initial_step_deg)
        if step.clamped:
            rospy.logwarn(
                "auto_align: initial move clamped (Δt=%.3f m, Δrot=%.2f deg)",
                step.delta_t_norm_m, step.delta_rot_deg)
        step_pose = matrix_m_to_pose_fr5(step.T_ab2ee_step)
        rospy.loginfo("auto_align: initial MoveJ -> %s", _fmt_pose(step_pose))
        tcp_client.move_j_to_pose(step_pose, settle_s=align_cfg.move_settle_s)

    last_metrics = None
    iters = 0
    for i in range(int(align_cfg.max_iterations)):
        iters = i + 1
        img, K = grab_image_and_K(
            hand_cam_image_topic, hand_cam_info_topic,
            timeout=image_wait_timeout_s)
        T_cam2tag = detect_apriltag(
            img, K, float(tag_a_size_m), int(tag_a_id), family=tag_family)
        if T_cam2tag is None:
            raise RuntimeError(
                f"auto_align: tag A (id={tag_a_id}) not detected at "
                f"iteration {iters} — adjust initial pose")
        metrics = alignment_metrics(T_cam2tag)
        last_metrics = metrics
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
        tcp_client.move_j_to_pose(step_pose, settle_s=align_cfg.move_settle_s)

    final_tcp = tcp_client.get_tcp_pose()
    return {
        "iterations": iters,
        "xy_offset_m": last_metrics.xy_offset_m if last_metrics else 0.0,
        "tilt_deg": last_metrics.tilt_deg if last_metrics else 0.0,
        "final_tcp": final_tcp,
    }
