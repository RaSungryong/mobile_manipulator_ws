#!/usr/bin/env python3
"""
handeye_calib_node
==================
Interactive hand-eye calibration via ROS services. The user moves the arm
to N distinct poses (such that the calibration tag is visible in the hand
camera) and calls ``~capture`` at each pose. Once enough samples are
collected, ``~compute`` runs the calibration and saves ``T_hc2ee.npz``.

Services:
  ~capture (std_srvs/Trigger) : grab one (image, K, TCP pose) triplet,
                                 detect the calibration tag, store sample.
  ~compute (std_srvs/Trigger) : run cv2.calibrateHandEye (5 methods),
                                 pick smallest AX=XB residual, save npz.
  ~reset   (std_srvs/Trigger) : clear all collected samples.
  ~status  (std_srvs/Trigger) : print sample count and last result info.

Parameters (under ``~`` namespace, see ``config/handeye_calib.yaml``):
  topics.hand_cam_image, topics.hand_cam_info,
  tag.id, tag.size_m, tag.family,
  robot.use_sdk, robot.robot_ip, robot.fairino_sdk_path, robot.tcp_index,
  io.image_wait_timeout, io.output_path, io.min_samples
"""
import os
import re
import sys
from pathlib import Path

import rospkg
import rospy
from std_srvs.srv import Trigger, TriggerResponse

from path_tag_locator.handeye_calib import (
    CalibSample,
    calibrate,
    save_result,
    summarize,
)
from path_tag_locator.persistence import HandeyeRunRecorder
from path_tag_locator.ros_image import grab_image_and_K
from path_tag_locator.tcp_pose import FairinoTCPClient


_FIND_RE = re.compile(r"\$\(find\s+([A-Za-z_][A-Za-z0-9_]*)\s*\)")


def _resolve_ros_path(p):
    if not p:
        return p
    rp = rospkg.RosPack()
    expanded = _FIND_RE.sub(lambda m: rp.get_path(m.group(1)), p)
    return os.path.expandvars(os.path.expanduser(expanded))


class HandeyeCalibNode:

    def __init__(self):
        rospy.init_node("handeye_calib")
        params = rospy.get_param("~", {})
        if not params:
            rospy.logfatal("handeye_calib: no parameters loaded under '~'.")
            sys.exit(1)

        root = params.get("handeye_calib", params)
        self.topic_image = root["topics"]["hand_cam_image"]
        self.topic_info = root["topics"]["hand_cam_info"]
        self.tag_id = int(root["tag"]["id"])
        self.tag_size_m = float(root["tag"]["size_m"])
        self.tag_family = str(root["tag"].get("family", "tag36h11"))
        self.image_wait_timeout = float(root["io"]["image_wait_timeout"])
        self.output_path = _resolve_ros_path(root["io"]["output_path"])
        self.min_samples = int(root["io"].get("min_samples", 8))
        self.run_root = _resolve_ros_path(
            root["io"].get("run_root", "~/.ros/path_tag_locator"))

        robot = root["robot"]
        self.tcp_client = None
        if bool(robot.get("use_sdk", True)):
            self.tcp_client = FairinoTCPClient(
                robot_ip=str(robot["robot_ip"]),
                sdk_path=robot.get("fairino_sdk_path"),
                tcp_index=int(robot.get("tcp_index", 1)),
            )

        self.samples = []           # list[CalibSample]
        self.last_result = None     # CalibResult | None
        self.recorder = HandeyeRunRecorder(self.run_root)
        rospy.loginfo("handeye_calib: persistence run_dir=%s",
                      self.recorder.run_dir)

        rospy.Service("~capture", Trigger, self._on_capture)
        rospy.Service("~compute", Trigger, self._on_compute)
        rospy.Service("~reset",   Trigger, self._on_reset)
        rospy.Service("~status",  Trigger, self._on_status)

        rospy.loginfo("handeye_calib: ready. output_path=%s tag_id=%d "
                      "tag_size_m=%.4f min_samples=%d",
                      self.output_path, self.tag_id, self.tag_size_m,
                      self.min_samples)
        rospy.loginfo("handeye_calib: services -> %s, %s, %s, %s",
                      rospy.resolve_name("~capture"),
                      rospy.resolve_name("~compute"),
                      rospy.resolve_name("~reset"),
                      rospy.resolve_name("~status"))

    # ------------------------------------------------------------------
    def _on_capture(self, _req):
        try:
            if self.tcp_client is None:
                raise RuntimeError("robot.use_sdk=false: no TCP pose source")
            img, K = grab_image_and_K(self.topic_image, self.topic_info,
                                      timeout=self.image_wait_timeout)
            tcp = self.tcp_client.get_tcp_pose()
            self.samples.append(CalibSample(image_bgr=img, K=K,
                                            tcp_pose_mm_deg=tcp))
            try:
                self.recorder.add_sample(img, K, tcp)
            except Exception as save_err:
                rospy.logwarn("handeye_calib: failed to persist sample: %s",
                              save_err)
            msg = (f"sample {len(self.samples)} captured "
                   f"(tcp_mm_deg={['%.2f' % v for v in tcp]})")
            rospy.loginfo("handeye_calib: %s", msg)
            return TriggerResponse(success=True, message=msg)
        except Exception as e:
            rospy.logwarn("handeye_calib.capture: %s", e)
            return TriggerResponse(success=False, message=str(e))

    def _on_compute(self, _req):
        try:
            result = calibrate(
                self.samples,
                tag_id=self.tag_id,
                tag_size_m=self.tag_size_m,
                family=self.tag_family,
                min_samples=self.min_samples,
            )
            path = save_result(result, self.output_path)
            self.last_result = result
            try:
                run_dir = self.recorder.save_result(
                    result, self.tag_id, self.tag_size_m, self.tag_family)
                rospy.loginfo("handeye_calib: archived run to %s", run_dir)
            except Exception as save_err:
                rospy.logwarn("handeye_calib: failed to archive run: %s",
                              save_err)
            text = summarize(result)
            rospy.loginfo("handeye_calib.compute:\n%s\nsaved: %s", text, path)
            return TriggerResponse(success=True, message=f"saved {path}\n{text}")
        except Exception as e:
            rospy.logwarn("handeye_calib.compute: %s", e)
            return TriggerResponse(success=False, message=str(e))

    def _on_reset(self, _req):
        n = len(self.samples)
        self.samples = []
        self.last_result = None
        try:
            self.recorder = HandeyeRunRecorder(self.run_root)
            rospy.loginfo("handeye_calib.reset: new run_dir=%s",
                          self.recorder.run_dir)
        except Exception as save_err:
            rospy.logwarn("handeye_calib.reset: failed to start new run: %s",
                          save_err)
        rospy.loginfo("handeye_calib.reset: cleared %d samples", n)
        return TriggerResponse(success=True, message=f"cleared {n} samples")

    def _on_status(self, _req):
        lines = [f"samples: {len(self.samples)} (min required: {self.min_samples})"]
        if self.last_result is not None:
            lines.append(summarize(self.last_result))
        msg = "\n".join(lines)
        return TriggerResponse(success=True, message=msg)

    def spin(self):
        rospy.spin()


def main():
    HandeyeCalibNode().spin()


if __name__ == "__main__":
    main()
