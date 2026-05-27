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
import os
import re
import sys

import numpy as np
import rospy
import rospkg
from geometry_msgs.msg import Pose, PoseStamped

from path_tag_locator.align_runner import run_auto_align
from path_tag_locator.chain import compute_T_A2B, compute_T_B_world
from path_tag_locator.constants import (
    load_extrinsics,
    load_locator_cfg_from_dict,
    load_reference_tag,
)
from path_tag_locator.geometry import (
    assert_rigid,
    matrix_to_pose,
    pose_to_matrix,
    rot2rpy_deg,
)
from path_tag_locator.hand_eye import load_T_hc2ee
from path_tag_locator.persistence import save_locate_failure, save_locate_run
from path_tag_locator.ros_image import grab_image_and_K
from path_tag_locator.tcp_pose import FairinoTCPClient
from path_tag_locator.srv import (
    LocatePathTag,
    LocatePathTagResponse,
)


_FIND_RE = re.compile(r"\$\(find\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")


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
        if np.allclose(self.T_A_world_default, np.eye(4), atol=1e-9):
            rospy.logwarn(
                "path_tag_locator: reference_tag.yaml is identity (no real "
                "world calibration). Results equal T_A2B until you edit "
                "%s or pass override_ref=true in the service request.",
                ref_tag_path)

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
        # Build a request echo eagerly (always written, even on failure).
        request_echo = self._echo_request(req)
        tag_b_id = (int(req.tag_b_id)
                    if int(req.tag_b_id) >= 0
                    else int(self.cfg.tag.tag_b_id))
        save_dir_req = (str(req.save_dir) if req.save_dir else
                        self.cfg.io.default_save_dir)
        try:
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

            # Always persist (raw images, K, all transforms, CSV append).
            try:
                run_dir = save_locate_run(
                    root_dir=save_dir_req,
                    tag_b_id=tag_b_id,
                    image_hc=hc_img,
                    image_fc=fc_img,
                    K_hc=K_hc,
                    K_fc=K_fc,
                    T_A_world=T_A_world,
                    T_hc2ee=self.T_hc2ee,
                    T_ab2mb=self.T_ab2mb,
                    T_mb2fc=self.T_mb2fc,
                    tcp_pose_mm_deg=tcp_pose,
                    T_A2B=T_A2B,
                    T_B_world=T_B_world,
                    position_m=resp.position_m,
                    rpy_deg=resp.rpy_deg,
                    tag_a_id=int(self.cfg.tag.tag_a_id),
                    tag_a_size_m=float(self.cfg.tag.tag_a_size_m),
                    tag_b_size_m=float(self.cfg.tag.tag_b_size_m),
                    family=self.cfg.tag.family,
                    request_echo=request_echo,
                    success=True,
                    message="ok",
                    auto_align_report={
                        "iterations": int(align_iters),
                        "xy_offset_m": float(align_xy),
                        "tilt_deg": float(align_tilt),
                        "final_tcp_mm_deg": [float(v) for v in align_final_tcp],
                    } if bool(req.auto_align) else None,
                )
                rospy.loginfo("path_tag_locator: saved run to %s", run_dir)
            except Exception as save_err:
                rospy.logwarn("path_tag_locator: failed to persist run: %s",
                              save_err)

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
            try:
                save_locate_failure(
                    root_dir=save_dir_req,
                    tag_b_id=tag_b_id,
                    message=str(e),
                    request_echo=request_echo,
                )
            except Exception as save_err:
                rospy.logwarn("path_tag_locator: failed to persist failure: %s",
                              save_err)
        return resp

    # ------------------------------------------------------------------
    @staticmethod
    def _echo_request(req):
        return {
            "tag_b_id": int(req.tag_b_id),
            "override_ref": bool(req.override_ref),
            "ref_pose": {
                "position": {
                    "x": float(req.ref_pose.position.x),
                    "y": float(req.ref_pose.position.y),
                    "z": float(req.ref_pose.position.z),
                },
                "orientation": {
                    "x": float(req.ref_pose.orientation.x),
                    "y": float(req.ref_pose.orientation.y),
                    "z": float(req.ref_pose.orientation.z),
                    "w": float(req.ref_pose.orientation.w),
                },
            },
            "save_result": bool(req.save_result),
            "save_dir": str(req.save_dir),
            "auto_align": bool(req.auto_align),
            "align_initial_tcp_mm_deg": [float(v)
                                         for v in req.align_initial_tcp_mm_deg],
        }

    # ------------------------------------------------------------------
    def _auto_align(self, initial_tcp_mm_deg):
        """Delegate to the shared align_runner module."""
        return run_auto_align(
            align_cfg=self.cfg.align,
            tcp_client=self.tcp_client,
            T_hc2ee=self.T_hc2ee,
            tag_a_id=int(self.cfg.tag.tag_a_id),
            tag_a_size_m=float(self.cfg.tag.tag_a_size_m),
            tag_family=self.cfg.tag.family,
            hand_cam_image_topic=self.cfg.topics.hand_cam_image,
            hand_cam_info_topic=self.cfg.topics.hand_cam_info,
            image_wait_timeout_s=float(self.cfg.io.image_wait_timeout),
            initial_tcp_mm_deg=initial_tcp_mm_deg,
        )

    def spin(self):
        rospy.spin()


def main():
    node = PathTagLocatorNode()
    node.spin()


if __name__ == "__main__":
    main()
