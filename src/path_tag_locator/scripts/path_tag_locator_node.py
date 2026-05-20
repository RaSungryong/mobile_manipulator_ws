#!/usr/bin/env python3
"""
path_tag_locator_node
=====================
Single ROS node that:
  - Loads platform extrinsics, hand-eye calibration, and reference tag.
  - Provides ``locate_path_tag`` service computing T_B_world for one path tag.
  - Latch-publishes the most recent result on ``tag_world_pose``.
  - Optionally persists results to disk (npz + yaml).

The node owns no internal state besides cached calibration matrices; each
service call grabs fresh images, fresh camera_info, and the live TCP pose.
"""
import datetime as _dt
import os
import re
import sys
import time
from pathlib import Path

import numpy as np
import rospy
import rospkg
import yaml
from geometry_msgs.msg import Pose, PoseStamped

from path_tag_locator.align import (
    alignment_metrics,
    clamp_step,
    compute_target_ee_pose,
    is_converged,
)
from path_tag_locator.chain import compute_T_A2B, compute_T_B_world
from path_tag_locator.constants import (
    load_extrinsics,
    load_locator_cfg_from_dict,
    load_reference_tag,
)
from path_tag_locator.detect import detect_apriltag
from path_tag_locator.geometry import (
    R_to_quat_xyzw,
    assert_rigid,
    matrix_m_to_pose_fr5,
    matrix_to_pose,
    pose_fr5_to_matrix_m,
    pose_to_matrix,
)
from path_tag_locator.hand_eye import load_T_hc2ee
from path_tag_locator.ros_image import grab_image_and_K
from path_tag_locator.tcp_pose import FairinoTCPClient
from path_tag_locator.srv import (
    LocatePathTag,
    LocatePathTagResponse,
)


_FIND_RE = re.compile(r"\$\(find\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")


def _fmt_pose(p):
    return ("[x={:.2f}mm y={:.2f}mm z={:.2f}mm "
            "rx={:.2f}deg ry={:.2f}deg rz={:.2f}deg]").format(*p)


def _resolve_ros_path(p: str) -> str:
    """Expand ``$(find pkg)`` and ``~`` / ``$VAR`` in a path string."""
    if p is None:
        return p
    rp = rospkg.RosPack()

    def _sub(m):
        pkg = m.group(1)
        return rp.get_path(pkg)

    expanded = _FIND_RE.sub(_sub, p)
    return os.path.expandvars(os.path.expanduser(expanded))


class PathTagLocatorNode:

    def __init__(self):
        rospy.init_node("path_tag_locator")

        params = rospy.get_param("~", {})
        if not params:
            rospy.logfatal("path_tag_locator: no parameters loaded under '~'. "
                           "Did you forget to <rosparam command=\"load\" ...>?")
            sys.exit(1)

        # constants module accepts either wrapped or inner dict
        cfg = load_locator_cfg_from_dict(params)
        self.cfg = cfg

        hand_eye_path = _resolve_ros_path(cfg.hand_eye_npz)
        extrinsics_path = _resolve_ros_path(cfg.extrinsics_yaml)
        ref_tag_path = _resolve_ros_path(cfg.reference_tag_yaml)

        rospy.loginfo("path_tag_locator: hand_eye_npz   = %s", hand_eye_path)
        rospy.loginfo("path_tag_locator: extrinsics_yaml = %s", extrinsics_path)
        rospy.loginfo("path_tag_locator: reference_tag_yaml = %s", ref_tag_path)

        self.T_hc2ee = load_T_hc2ee(hand_eye_path)
        assert_rigid(self.T_hc2ee, name="T_hc2ee")

        self.T_ab2mb, self.T_mb2fc = load_extrinsics(extrinsics_path)
        self.T_A_world_default = load_reference_tag(ref_tag_path)

        self.tcp_client = None
        if cfg.robot.use_sdk:
            self.tcp_client = FairinoTCPClient(
                robot_ip=cfg.robot.robot_ip,
                sdk_path=cfg.robot.fairino_sdk_path,
                tcp_index=cfg.robot.tcp_index,
                default_vel=cfg.align.move_vel,
                default_acc=cfg.align.move_acc,
                default_ovl=cfg.align.move_ovl,
            )

        # Publisher (latched) and service
        self.pub = rospy.Publisher("~tag_world_pose", PoseStamped,
                                   queue_size=1, latch=True)
        self.srv = rospy.Service("~locate_path_tag", LocatePathTag,
                                 self._handle_locate)

        rospy.loginfo("path_tag_locator: ready. service=%s topic=%s",
                      rospy.resolve_name("~locate_path_tag"),
                      rospy.resolve_name("~tag_world_pose"))

    # ------------------------------------------------------------------
    def _handle_locate(self, req):
        resp = LocatePathTagResponse()
        try:
            tag_b_id = (int(req.tag_b_id)
                        if int(req.tag_b_id) >= 0
                        else int(self.cfg.tag.tag_b_id))

            if req.override_ref:
                T_A_world = pose_to_matrix(
                    (req.ref_pose.position.x,
                     req.ref_pose.position.y,
                     req.ref_pose.position.z),
                    (req.ref_pose.orientation.x,
                     req.ref_pose.orientation.y,
                     req.ref_pose.orientation.z,
                     req.ref_pose.orientation.w),
                )
                try:
                    assert_rigid(T_A_world, name="T_A_world(override)")
                except ValueError as e:
                    raise RuntimeError(f"invalid ref_pose: {e}")
            else:
                T_A_world = self.T_A_world_default

            timeout = float(self.cfg.io.image_wait_timeout)
            hc_img, K_hc = grab_image_and_K(
                self.cfg.topics.hand_cam_image,
                self.cfg.topics.hand_cam_info,
                timeout=timeout,
            )
            fc_img, K_fc = grab_image_and_K(
                self.cfg.topics.front_cam_image,
                self.cfg.topics.front_cam_info,
                timeout=timeout,
            )

            if self.tcp_client is None:
                raise RuntimeError("robot.use_sdk=false: no TCP pose source")

            align_iters = 0
            align_xy = 0.0
            align_tilt = 0.0
            align_final_tcp = [0.0] * 6
            if bool(req.auto_align):
                align_result = self._auto_align(req.align_initial_tcp_mm_deg)
                align_iters = align_result["iterations"]
                align_xy = align_result["xy_offset_m"]
                align_tilt = align_result["tilt_deg"]
                align_final_tcp = align_result["final_tcp"]

            tcp_pose = self.tcp_client.get_tcp_pose()

            out = compute_T_A2B(
                image_hc=hc_img,
                image_fc=fc_img,
                tcp_pose_mm_deg=tcp_pose,
                T_hc2ee=self.T_hc2ee,
                K_hc=K_hc,
                K_fc=K_fc,
                T_ab2mb=self.T_ab2mb,
                T_mb2fc=self.T_mb2fc,
                tag_a_id=int(self.cfg.tag.tag_a_id),
                tag_b_id=tag_b_id,
                tag_a_size_m=float(self.cfg.tag.tag_a_size_m),
                tag_b_size_m=float(self.cfg.tag.tag_b_size_m),
                family=self.cfg.tag.family,
            )
            T_A2B = out["T_A2B"]
            T_B_world = compute_T_B_world(T_A_world, T_A2B)
            pos, quat = matrix_to_pose(T_B_world)

            resp.success = True
            resp.message = "ok"
            resp.tag_b_world_pose = Pose()
            resp.tag_b_world_pose.position.x = pos[0]
            resp.tag_b_world_pose.position.y = pos[1]
            resp.tag_b_world_pose.position.z = pos[2]
            resp.tag_b_world_pose.orientation.x = quat[0]
            resp.tag_b_world_pose.orientation.y = quat[1]
            resp.tag_b_world_pose.orientation.z = quat[2]
            resp.tag_b_world_pose.orientation.w = quat[3]

            from path_tag_locator.geometry import rot2rpy_deg
            rpy = rot2rpy_deg(T_B_world[:3, :3])
            resp.position_m = [float(pos[0]), float(pos[1]), float(pos[2])]
            resp.rpy_deg = [float(rpy[0]), float(rpy[1]), float(rpy[2])]
            resp.t_a2b_row_major = [float(v) for v in T_A2B.flatten().tolist()]
            resp.align_iterations_used = int(align_iters)
            resp.align_final_xy_offset_m = float(align_xy)
            resp.align_final_tilt_deg = float(align_tilt)
            resp.align_final_tcp_mm_deg = [float(v) for v in align_final_tcp]

            # Latched publish
            stamped = PoseStamped()
            stamped.header.stamp = rospy.Time.now()
            stamped.header.frame_id = "world"
            stamped.pose = resp.tag_b_world_pose
            self.pub.publish(stamped)

            if bool(req.save_result):
                save_dir = (req.save_dir or self.cfg.io.default_save_dir)
                self._save(save_dir, tag_b_id, T_A_world, T_A2B, T_B_world,
                           tcp_pose, resp.position_m, resp.rpy_deg)

            rospy.loginfo("path_tag_locator: tag_b_id=%d pos=%s rpy=%s",
                          tag_b_id, resp.position_m, resp.rpy_deg)
        except Exception as e:
            rospy.logwarn("path_tag_locator: %s", e)
            resp.success = False
            resp.message = str(e)
            resp.tag_b_world_pose = Pose()
            resp.position_m = [0.0, 0.0, 0.0]
            resp.rpy_deg = [0.0, 0.0, 0.0]
            resp.t_a2b_row_major = [0.0] * 16
            resp.align_iterations_used = 0
            resp.align_final_xy_offset_m = 0.0
            resp.align_final_tilt_deg = 0.0
            resp.align_final_tcp_mm_deg = [0.0] * 6
        return resp

    # ------------------------------------------------------------------
    def _auto_align(self, initial_tcp_mm_deg):
        """Run the iterative align procedure. Returns dict with keys
        iterations, xy_offset_m, tilt_deg, final_tcp.
        Raises ``RuntimeError`` on detection / motion failure.
        """
        cfg = self.cfg.align
        if self.tcp_client is None:
            raise RuntimeError("auto_align requires robot.use_sdk=true")

        initial = [float(v) for v in initial_tcp_mm_deg]
        if len(initial) != 6 or all(v == 0.0 for v in initial):
            raise RuntimeError(
                "auto_align: align_initial_tcp_mm_deg must be 6 floats; "
                "got all zeros (set explicit pose in the service request)")

        # 1. Initial move (clamped with initial-step limits)
        cur_tcp = self.tcp_client.get_tcp_pose()
        T_cur = pose_fr5_to_matrix_m(cur_tcp)
        T_init = pose_fr5_to_matrix_m(initial)
        step = clamp_step(T_cur, T_init,
                          max_step_m=cfg.max_initial_step_m,
                          max_step_deg=cfg.max_initial_step_deg)
        if step.clamped:
            rospy.logwarn(
                "auto_align: initial move clamped (Δt=%.3f m, Δrot=%.2f deg)",
                step.delta_t_norm_m, step.delta_rot_deg)
        step_pose = matrix_m_to_pose_fr5(step.T_ab2ee_step)
        rospy.loginfo("auto_align: initial MoveJ -> %s", _fmt_pose(step_pose))
        self.tcp_client.move_j_to_pose(step_pose, settle_s=cfg.move_settle_s)

        # 2. Iterative refinement
        last_metrics = None
        iters = 0
        timeout = float(self.cfg.io.image_wait_timeout)
        for i in range(int(cfg.max_iterations)):
            iters = i + 1
            img, K = grab_image_and_K(
                self.cfg.topics.hand_cam_image,
                self.cfg.topics.hand_cam_info,
                timeout=timeout)
            T_cam2tag = detect_apriltag(
                img, K, float(self.cfg.tag.tag_a_size_m),
                int(self.cfg.tag.tag_a_id), family=self.cfg.tag.family)
            if T_cam2tag is None:
                raise RuntimeError(
                    f"auto_align: tag A (id={self.cfg.tag.tag_a_id}) not "
                    f"detected at iteration {iters} — adjust initial pose")
            metrics = alignment_metrics(T_cam2tag)
            last_metrics = metrics
            rospy.loginfo(
                "auto_align iter %d/%d: xy=%.4f m, tilt=%.3f deg, z=%.3f m",
                iters, cfg.max_iterations,
                metrics.xy_offset_m, metrics.tilt_deg, metrics.z_distance_m)
            if is_converged(metrics, cfg.position_tol_m, cfg.angle_tol_deg):
                rospy.loginfo("auto_align: converged at iteration %d", iters)
                break

            cur_tcp = self.tcp_client.get_tcp_pose()
            T_cur = pose_fr5_to_matrix_m(cur_tcp)
            T_target = compute_target_ee_pose(
                T_cur, self.T_hc2ee, T_cam2tag,
                target_distance_m=cfg.target_distance_m)
            step = clamp_step(T_cur, T_target,
                              max_step_m=cfg.max_step_m,
                              max_step_deg=cfg.max_step_deg)
            if step.clamped:
                rospy.logwarn(
                    "auto_align: step %d clamped (Δt=%.3f m, Δrot=%.2f deg)",
                    iters, step.delta_t_norm_m, step.delta_rot_deg)
            step_pose = matrix_m_to_pose_fr5(step.T_ab2ee_step)
            self.tcp_client.move_j_to_pose(step_pose, settle_s=cfg.move_settle_s)

        final_tcp = self.tcp_client.get_tcp_pose()
        return {
            "iterations": iters,
            "xy_offset_m": last_metrics.xy_offset_m if last_metrics else 0.0,
            "tilt_deg": last_metrics.tilt_deg if last_metrics else 0.0,
            "final_tcp": final_tcp,
        }

    # ------------------------------------------------------------------
    def _save(self, save_dir, tag_b_id, T_A_world, T_A2B, T_B_world,
              tcp_pose, position_m, rpy_deg):
        d = Path(os.path.expanduser(save_dir))
        d.mkdir(parents=True, exist_ok=True)
        ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = d / f"path_tag_{tag_b_id}_{ts}"
        np.savez(str(base) + ".npz",
                 T_B_world=T_B_world,
                 T_A2B=T_A2B,
                 T_A_world=T_A_world,
                 tcp_pose_mm_deg=np.asarray(tcp_pose, dtype=np.float64))
        with open(str(base) + ".yaml", "w") as fh:
            yaml.safe_dump({
                "tag_b_id": int(tag_b_id),
                "position_m": [float(v) for v in position_m],
                "rpy_deg": [float(v) for v in rpy_deg],
                "tcp_pose_mm_deg": [float(v) for v in tcp_pose],
            }, fh, default_flow_style=False)
        rospy.loginfo("path_tag_locator: saved %s.{npz,yaml}", base)

    def spin(self):
        rospy.spin()


def main():
    node = PathTagLocatorNode()
    node.spin()


if __name__ == "__main__":
    main()
