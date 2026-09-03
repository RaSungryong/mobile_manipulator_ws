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
import threading
from pathlib import Path

import rospy
import rospkg
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger, TriggerResponse

from path_tag_locator.arm_interface import ArmInterface
from path_tag_locator.base_interface import BaseInterface
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
from path_tag_locator.lift_listener import LiftHeightListener
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
        # Own private namespace FIRST: the launch loads locator.yaml into
        # this node's <node> block, and that copy must be authoritative —
        # the global fallback actually resolves to the path_tag_locator
        # NODE's private namespace (same wrapped dict), which would make a
        # calibrator-only override silently lose.
        loc_params = rospy.get_param("~path_tag_locator", None) or \
                     rospy.get_param("/path_tag_locator", None)
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
        # Negative or absent = leave the lift alone. (YAML null cannot go
        # through the parameter server, so -1 is the config's "unset".)
        lh = root.get("lift_height_mm", None)
        self.lift_height_mm = None if lh is None or float(lh) < 0 else float(lh)
        self.lift_home_first = bool(root.get("lift_home_first", True))

        # --- one-time loads: hand-eye + extrinsics + device proxies.
        self.T_hc2ee = load_T_hc2ee(_resolve_ros_path(self.locator_cfg.hand_eye_npz))
        assert_rigid(self.T_hc2ee, name="T_hc2ee")
        self.T_ab2mb, self.T_mb2fc = load_extrinsics(
            _resolve_ros_path(self.locator_cfg.extrinsics_yaml))

        # Arm through arm_node, base through mobile_node — this package
        # owns no hardware. ⚠️ Second commander of /mobile/goto_tag after
        # task_executor: do not issue TASK/GOTO during a calibration
        # session (mobile_node refuses overlapping moves; the session
        # would fail its nav step).
        self.tcp_client = ArmInterface(
            state_topic=self.locator_cfg.arm.state_topic,
            move_cart_topic=self.locator_cfg.arm.move_cart_topic,
            home_service=self.locator_cfg.arm.home_service,
            default_vel=self.locator_cfg.align.move_vel,
            default_acc=self.locator_cfg.align.move_acc,
            default_ovl=self.locator_cfg.align.move_ovl,
            motion_timeout_s=self.locator_cfg.arm.motion_timeout_s,
        )
        self.base = BaseInterface()
        # T_ab2mb is lift-at-origin; the chain compensates by live height.
        self.lift = LiftHeightListener()
        # Lift COMMANDING (session-start positioning) goes through the
        # main stack's LiftClient; lazy so an unset (-1) lift_height_mm config
        # never touches the lift at all.
        self._lift_client = None

        # --- publishers + service.
        self.progress_pub = rospy.Publisher(
            "~progress", String, queue_size=64)
        self.target_pub = rospy.Publisher(
            "~current_target_tag", Int32, queue_size=1, latch=True)
        # One session at a time: rospy runs each service call in its own
        # thread, and two concurrent orchestrators would interleave
        # /mobile/goto_tag + /arm/move_cart commands (both owner nodes
        # refuse overlaps, so BOTH sessions would fail entries randomly).
        self._session_lock = threading.Lock()
        self._cancel = threading.Event()
        rospy.Service("~run_calibration", RunMapCalibration,
                      self._on_run_calibration)
        # Cooperative abort: checked at the top of every plan entry, so
        # the current entry finishes/fails normally and the session stops
        # cleanly with the partial output kept. (For an immediate motion
        # stop use the arm/base STOP paths — this is not the e-stop.)
        rospy.Service("~cancel_calibration", Trigger, self._on_cancel)

        rospy.loginfo(
            "[map_calibrator] ready. service=%s topic=%s default plan=%s",
            rospy.resolve_name("~run_calibration"),
            rospy.resolve_name("~progress"),
            self.plan_yaml_default)

    # ------------------------------------------------------------------
    def _position_lift_for_session(self):
        """Move the lift to the configured fixed height (home first by
        default — counts drift after up-then-down strokes; a single
        home+up stroke reaches a known height reliably). No-op when
        lift_height_mm is unset (negative / absent)."""
        if self.lift_height_mm is None:
            return
        from apriltag_nav.lift_client import LiftClient
        if self._lift_client is None:
            self._lift_client = LiftClient()
        target = float(self.lift_height_mm)
        if self.lift_home_first:
            rospy.loginfo("[map_calibrator] homing lift before session "
                          "(lift_home_first=true)")
            ok, msg = self._lift_client.home()
            if not ok:
                raise RuntimeError(f"lift home failed: {msg}")
        rospy.loginfo("[map_calibrator] moving lift to session height "
                      "%.1f mm", target)
        ok, msg = self._lift_client.goto_mm(target)
        if not ok:
            raise RuntimeError(
                f"lift goto {target:.1f} mm failed: {msg}")

    # ------------------------------------------------------------------
    def _on_cancel(self, _req):
        self._cancel.set()
        return TriggerResponse(success=True,
                               message="cancel requested — session stops "
                                       "after the current entry")

    def _on_run_calibration(self, req):
        resp = RunMapCalibrationResponse()
        if not self._session_lock.acquire(blocking=False):
            resp.success = False
            resp.message = ("a calibration session is already running — "
                            "one at a time (cancel with "
                            "~cancel_calibration)")
            resp.num_succeeded = 0
            resp.num_failed = 0
            resp.output_yaml_path = ""
            return resp
        self._cancel.clear()
        try:
            return self._run_session(req, resp)
        finally:
            self._session_lock.release()

    def _run_session(self, req, resp):
        try:
            if not bool(req.dry_run):
                self._position_lift_for_session()
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
                hand_cam_detections_topic=(
                    self.locator_cfg.topics.hand_cam_detections),
                front_cam_detections_topic=(
                    self.locator_cfg.topics.front_cam_detections),
                hand_cam_detector_size_m=float(
                    self.locator_cfg.detector.hand_cam_tag_size_m),
                front_cam_detector_size_m=float(
                    self.locator_cfg.detector.front_cam_tag_size_m),
                detection_wait_timeout_s=float(
                    self.locator_cfg.io.detection_wait_timeout),
                tag_a_size_m_default=float(self.locator_cfg.tag.tag_a_size_m),
                tag_b_size_m=float(self.locator_cfg.tag.tag_b_size_m),
                align_cfg=self.locator_cfg.align,
                hand_cam_image_topic=self.locator_cfg.topics.hand_cam_image,
                front_cam_image_topic=self.locator_cfg.topics.front_cam_image,
                tag_family=self.locator_cfg.tag.family,
                dry_run=bool(req.dry_run),
                home_before_nav=bool(
                    self.locator_cfg.arm.home_before_nav),
            )

            orch = CalibrationOrchestrator(
                cfg=orch_cfg,
                T_hc2ee=self.T_hc2ee,
                T_ab2mb=self.T_ab2mb,
                T_mb2fc=self.T_mb2fc,
                tcp_client=self.tcp_client,
                base=self.base,
                lift=self.lift,
                cancel_check=self._cancel.is_set,
                progress_pub=self.progress_pub,
                target_pub=self.target_pub,
            )
            report = orch.run()

            resp.success = (report.num_failed == 0)
            resp.message = (
                f"succeeded={report.num_succeeded}, failed={report.num_failed}; "
                f"out={report.output_yaml_path}"
                + (f"; session={report.session_dir}" if report.session_dir else ""))
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
