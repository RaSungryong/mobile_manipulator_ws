#!/usr/bin/env python3
"""
handeye_calib_node
==================
Interactive hand-eye calibration via ROS services. The user moves the arm
to N distinct poses (such that the calibration tag is visible in the hand
camera) and calls ``~capture`` at each pose. Once enough samples are
collected, ``~compute`` runs the calibration and saves ``T_hc2ee.npz``.

Services:
  ~capture     (Trigger) : grab one (image, K, TCP pose) triplet,
                           detect the calibration tag, store sample.
  ~compute     (Trigger) : run cv2.calibrateHandEye (5 methods),
                           pick smallest AX=XB residual, save npz.
  ~reset       (Trigger) : clear all collected samples.
  ~status      (Trigger) : print sample count and last result info.
  ~load_latest (Trigger) : append samples from the most recent prior
                           run_*/ directory under run_root.

Parameters (under ``~`` namespace, see ``config/handeye_calib.yaml``):
  topics.hand_cam_image, topics.hand_cam_info,
  tag.id, tag.size_m, tag.family,
  arm.state_topic (TCP pose comes from arm_node's /arm/state — this node
                   opens no SDK connection; move the arm with the teach
                   pendant or robot_ui jog between captures),
  io.image_wait_timeout, io.output_path, io.min_samples,
  io.load_samples_dirs (optional: list of prior run/samples dirs to
                        preload at node start; also accepts a single
                        string).

Note this node deliberately keeps grabbing RAW hand-cam frames (not the
shared detector's output): cv2.calibrateHandEye needs per-sample
T_cam2target re-detections over archived images, and samples must stay
reloadable from disk across runs.
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
from path_tag_locator.arm_interface import ArmInterface
from path_tag_locator.persistence import HandeyeRunRecorder, load_handeye_samples
from path_tag_locator.ros_image import grab_image_and_K


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

        arm = root.get("arm", {})
        # Read-only TCP pose from arm_node; this node never moves the arm.
        self.tcp_client = ArmInterface(
            state_topic=str(arm.get("state_topic", "/arm/state")),
            move_cart_topic=str(arm.get("move_cart_topic", "/arm/move_cart")),
        )

        self.samples = []           # list[CalibSample]
        self.last_result = None     # CalibResult | None
        self.recorder = HandeyeRunRecorder(self.run_root)
        rospy.loginfo("handeye_calib: persistence run_dir=%s",
                      self.recorder.run_dir)

        # Optional: load previously-captured sample directories at startup.
        # ~load_samples_dirs accepts either a single string or a list of
        # strings. Each entry can be a run directory (containing samples/)
        # or the samples/ directory directly. `$(find pkg)` and `~` are
        # expanded.
        load_param = rospy.get_param("~load_samples_dirs",
                                     root.get("io", {}).get("load_samples_dirs", []))
        load_dirs = ([load_param] if isinstance(load_param, str) else
                     list(load_param or []))
        for entry in load_dirs:
            self._load_samples_from(entry)

        rospy.Service("~capture", Trigger, self._on_capture)
        rospy.Service("~compute", Trigger, self._on_compute)
        rospy.Service("~reset",   Trigger, self._on_reset)
        rospy.Service("~status",  Trigger, self._on_status)
        rospy.Service("~load_latest", Trigger, self._on_load_latest)

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
    def _load_samples_from(self, dir_entry):
        """Append samples from a previously-saved run directory into
        ``self.samples``. Errors are logged but do not raise."""
        path = _resolve_ros_path(str(dir_entry))
        try:
            loaded = load_handeye_samples(path)
        except Exception as e:
            rospy.logwarn("handeye_calib: cannot load from %s: %s", path, e)
            return 0
        for s in loaded:
            self.samples.append(CalibSample(
                image_bgr=s["image_bgr"],
                K=s["K"],
                tcp_pose_mm_deg=s["tcp_pose_mm_deg"],
            ))
        rospy.loginfo("handeye_calib: loaded %d sample(s) from %s "
                      "(total in memory: %d)",
                      len(loaded), path, len(self.samples))
        return len(loaded)

    def _on_load_latest(self, _req):
        """Locate and load the most recent run_<ts>/ directory under
        ``run_root`` that is NOT the recorder's own (in-progress) dir."""
        try:
            root = Path(self.run_root) / "handeye_calib"
            runs = sorted([d for d in root.glob("run_*")
                           if d.is_dir() and d != self.recorder.run_dir],
                          key=lambda d: d.name)
            if not runs:
                return TriggerResponse(success=False,
                                       message=f"no prior runs under {root}")
            latest = runs[-1]
            n = self._load_samples_from(str(latest))
            return TriggerResponse(
                success=n > 0,
                message=f"loaded {n} sample(s) from {latest} "
                        f"(total in memory: {len(self.samples)})")
        except Exception as e:
            rospy.logwarn("handeye_calib.load_latest: %s", e)
            return TriggerResponse(success=False, message=str(e))

    # ------------------------------------------------------------------
    def _on_capture(self, _req):
        try:
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
