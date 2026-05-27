#!/usr/bin/env python3
"""
map_calibrator_node
===================
Orchestrator ROS node for the map-calibration workflow.

Service:
  ~run_calibration (path_tag_locator/RunMapCalibration) — kicks off the
                   full session over the configured plan.

Topics:
  ~progress           (std_msgs/String, JSON-encoded per-tag status)
  ~current_target_tag (std_msgs/Int32, the path_tag currently being
                       processed)

All heavy lifting is in ``path_tag_locator.calibration.orchestrator``.
This script is just a thin ROS shell.
"""
import datetime as _dt
import os
import re
import sys
from pathlib import Path

import rospy
import rospkg
from std_msgs.msg import Int32, String

from path_tag_locator.calibration.orchestrator import (
    CalibrationOrchestrator,
    OrchestratorCfg,
)
from path_tag_locator.constants import (
    load_extrinsics,
    load_locator_cfg_from_dict,
)
from path_tag_locator.geometry import assert_rigid
from path_tag_locator.hand_eye import load_T_hc2ee
from path_tag_locator.tcp_pose import FairinoTCPClient
from path_tag_locator.srv import (
    RunMapCalibration,
    RunMapCalibrationResponse,
)


_FIND_RE = re.compile(r"\$\(find\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")


def _resolve_ros_path(p: str) -> str:
    if not p:
        return p
    rp = rospkg.RosPack()
    expanded = _FIND_RE.sub(lambda m: rp.get_path(m.group(1)), p)
    return os.path.expandvars(os.path.expanduser(expanded))


def _default_map_out_path() -> str:
    ts = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return str(Path("~/.ros/path_tag_locator").expanduser() /
               f"map_world_{ts}.yaml")


class MapCalibratorNode:

    def __init__(self):
        rospy.init_node("map_calibrator")

        # --- locator params: reuse the existing parser (same yaml layout).
        loc_params = rospy.get_param("/path_tag_locator", None) or \
                     rospy.get_param("~path_tag_locator", None)
        if loc_params is None:
            rospy.logfatal(
                "map_calibrator: cannot find 'path_tag_locator' params. "
                "Did you <rosparam command=\"load\" file=\"locator.yaml\"> "
                "in your launch?")
            sys.exit(1)
        self.locator_cfg = load_locator_cfg_from_dict(loc_params)

        # --- map_calibrator-specific params (under ~ namespace).
        mc = rospy.get_param("~", {})
        root = mc.get("map_calibrator", mc)

        self.ref_tags_yaml_default = _resolve_ros_path(
            root.get("ref_tags_yaml", ""))
        self.plan_yaml_default = _resolve_ros_path(
            root.get("plan_yaml", ""))
        self.map_in_default = _resolve_ros_path(
            root.get("map_in_path", ""))
        self.map_out_default = _resolve_ros_path(
            root.get("map_out_path", ""))
        self.robot_nav_yaml = _resolve_ros_path(
            root.get("robot_nav_yaml", ""))
        self.map_yaml_for_nav = _resolve_ros_path(
            root.get("map_yaml_for_nav", root.get("map_in_path", "")))

        # --- one-time loads: hand-eye + extrinsics + Fairino client.
        self.T_hc2ee = load_T_hc2ee(_resolve_ros_path(self.locator_cfg.hand_eye_npz))
        assert_rigid(self.T_hc2ee, name="T_hc2ee")
        self.T_ab2mb, self.T_mb2fc = load_extrinsics(
            _resolve_ros_path(self.locator_cfg.extrinsics_yaml))

        self.tcp_client = None
        if self.locator_cfg.robot.use_sdk:
            self.tcp_client = FairinoTCPClient(
                robot_ip=self.locator_cfg.robot.robot_ip,
                sdk_path=self.locator_cfg.robot.fairino_sdk_path,
                tcp_index=self.locator_cfg.robot.tcp_index,
                default_vel=self.locator_cfg.align.move_vel,
                default_acc=self.locator_cfg.align.move_acc,
                default_ovl=self.locator_cfg.align.move_ovl,
            )

        # --- publishers + service.
        self.progress_pub = rospy.Publisher(
            "~progress", String, queue_size=64)
        self.target_pub = rospy.Publisher(
            "~current_target_tag", Int32, queue_size=1, latch=True)
        rospy.Service("~run_calibration", RunMapCalibration,
                      self._on_run_calibration)

        rospy.loginfo(
            "[map_calibrator] ready. service=%s topic=%s default plan=%s",
            rospy.resolve_name("~run_calibration"),
            rospy.resolve_name("~progress"),
            self.plan_yaml_default)

    # ------------------------------------------------------------------
    def _on_run_calibration(self, req):
        resp = RunMapCalibrationResponse()
        try:
            ref_tags_yaml = _resolve_ros_path(req.ref_tags_path) or self.ref_tags_yaml_default
            plan_yaml = _resolve_ros_path(req.plan_path) or self.plan_yaml_default
            map_in_path = _resolve_ros_path(req.map_in_path) or self.map_in_default
            map_out_path = _resolve_ros_path(req.map_out_path) or self.map_out_default
            if not map_out_path:
                map_out_path = _default_map_out_path()

            for name, val in [("ref_tags_yaml", ref_tags_yaml),
                              ("plan_yaml", plan_yaml),
                              ("map_in_path", map_in_path)]:
                if not val:
                    raise RuntimeError(
                        f"{name} is empty and no yaml default configured")

            orch_cfg = OrchestratorCfg(
                ref_tags_yaml=ref_tags_yaml,
                plan_yaml=plan_yaml,
                map_in_path=map_in_path,
                map_out_path=map_out_path,
                save_dir=self.locator_cfg.io.default_save_dir,
                hand_cam_image_topic=self.locator_cfg.topics.hand_cam_image,
                hand_cam_info_topic=self.locator_cfg.topics.hand_cam_info,
                front_cam_image_topic=self.locator_cfg.topics.front_cam_image,
                front_cam_info_topic=self.locator_cfg.topics.front_cam_info,
                image_wait_timeout_s=float(self.locator_cfg.io.image_wait_timeout),
                tag_family=self.locator_cfg.tag.family,
                tag_a_size_m_default=float(self.locator_cfg.tag.tag_a_size_m),
                tag_b_size_m=float(self.locator_cfg.tag.tag_b_size_m),
                robot_nav_yaml=self.robot_nav_yaml,
                map_yaml_for_nav=self.map_yaml_for_nav or map_in_path,
                align_cfg=self.locator_cfg.align,
                dry_run=bool(req.dry_run),
            )

            orch = CalibrationOrchestrator(
                cfg=orch_cfg,
                T_hc2ee=self.T_hc2ee,
                T_ab2mb=self.T_ab2mb,
                T_mb2fc=self.T_mb2fc,
                tcp_client=self.tcp_client,
                progress_pub=self.progress_pub,
                target_pub=self.target_pub,
            )
            report = orch.run()

            resp.success = (report.num_failed == 0)
            resp.message = (
                f"succeeded={report.num_succeeded}, failed={report.num_failed}; "
                f"out={report.output_yaml_path}")
            resp.num_succeeded = report.num_succeeded
            resp.num_failed = report.num_failed
            resp.output_yaml_path = report.output_yaml_path
        except Exception as e:
            rospy.logerr("[map_calibrator] %s", e)
            resp.success = False
            resp.message = str(e)
            resp.num_succeeded = 0
            resp.num_failed = 0
            resp.output_yaml_path = ""
        return resp

    def spin(self):
        rospy.spin()


def main():
    MapCalibratorNode().spin()


if __name__ == "__main__":
    main()
