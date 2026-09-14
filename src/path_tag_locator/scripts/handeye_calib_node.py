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
  ~auto_sample (Trigger) : (2026-09-14) sweep the camera around the tag
                           and capture at every view — see
                           path_tag_locator.handeye_sweep. Starts a
                           thread and returns; progress streams on
                           ~progress (String JSON: align / start /
                           sample / finished); ~cancel stops it after
                           the current move. Needs the tag in view
                           from the current pose, robot_camera_node's
                           hand_cam detector, and the current T_hc2ee
                           (aims the sweep only).
  ~cancel      (Trigger) : stop a running auto_sample.

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
from path_tag_locator.handeye_sweep import SweepCfg, SweepRunner
from path_tag_locator.detections import (detection_to_T_cam2tag,
                                         median_tilt_detection,
                                         wait_for_tag_detections)
from path_tag_locator.hand_eye import load_T_hc2ee
from std_msgs.msg import String
import json
import threading


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
        # TCP pose from arm_node's /arm/state. The arm is moved ONLY by
        # ~auto_sample (through /arm/move_cart, like the calibrator);
        # ~capture never moves it.
        self.tcp_client = ArmInterface(
            state_topic=str(arm.get("state_topic", "/arm/state")),
            move_cart_topic=str(arm.get("move_cart_topic", "/arm/move_cart")),
        )

        # --- automatic sweep (handeye_calib.yaml `auto:`)
        auto = dict(root.get("auto", {}) or {})
        self.auto_detections_topic = str(auto.pop("detections_topic", "/hand_cam/tag_detections"))
        self.auto_detector_size_m = float(auto.pop("detector_tag_size_m", self.tag_size_m))
        self.auto_hand_eye_npz = _resolve_ros_path(
            auto.pop("hand_eye_npz", "$(find path_tag_locator)/config/hand_eye/T_hc2ee.npz"))
        self.auto_move_vel = float(auto.pop("move_vel", 15.0))
        self.auto_move_acc = float(auto.pop("move_acc", 20.0))
        self.auto_settle_s = float(auto.pop("settle_s", 0.5))
        self.auto_detect_frames = int(auto.pop("detect_frames", 5))
        self.auto_detect_timeout_s = float(auto.pop("detect_timeout_s", 2.0))
        known = set(SweepCfg.__dataclass_fields__)
        unknown = [k for k in auto if k not in known]
        if unknown:
            rospy.logwarn("handeye_calib: unknown auto: keys ignored: %s", unknown)
        self.sweep_cfg = SweepCfg(**{k: v for k, v in auto.items() if k in known})
        self._sweep_thread = None
        self._sweep_cancel = threading.Event()
        self._sample_lock = threading.Lock()
        self.progress_pub = rospy.Publisher("~progress", String, queue_size=50)

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
        rospy.Service("~auto_sample", Trigger, self._on_auto_sample)
        rospy.Service("~cancel", Trigger, self._on_cancel)

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
    def _capture_sample(self):
        """One (image, K, TCP) sample at the current pose. Returns (ok, msg)."""
        try:
            img, K = grab_image_and_K(self.topic_image, self.topic_info,
                                      timeout=self.image_wait_timeout)
            tcp = self.tcp_client.get_tcp_pose()
            with self._sample_lock:
                self.samples.append(CalibSample(image_bgr=img, K=K,
                                                tcp_pose_mm_deg=tcp))
                n = len(self.samples)
            try:
                self.recorder.add_sample(img, K, tcp)
            except Exception as save_err:
                rospy.logwarn("handeye_calib: failed to persist sample: %s",
                              save_err)
            msg = (f"sample {n} captured "
                   f"(tcp_mm_deg={['%.2f' % v for v in tcp]})")
            rospy.loginfo("handeye_calib: %s", msg)
            return True, msg
        except Exception as e:
            rospy.logwarn("handeye_calib.capture: %s", e)
            return False, str(e)

    def _on_capture(self, _req):
        ok, msg = self._capture_sample()
        return TriggerResponse(success=ok, message=msg)

    # ------------------------------------------------------------------
    # automatic sweep
    # ------------------------------------------------------------------
    def _sweep_running(self):
        t = self._sweep_thread
        return t is not None and t.is_alive()

    def _publish_progress(self, d):
        try:
            d = dict(d)
            d['stamp'] = rospy.get_time()
            d['n_samples'] = len(self.samples)
            self.progress_pub.publish(String(data=json.dumps(d, default=str)))
        except Exception as e:
            rospy.logwarn("handeye_calib: progress publish failed: %s", e)

    def _detect_T_cam2tag(self):
        """Median-tilt detection of the calibration tag on the shared
        hand_cam detector, as T_cam2tag (m); None when not seen."""
        try:
            dets = wait_for_tag_detections(self.auto_detections_topic, self.tag_id,
                                           self.auto_detect_frames,
                                           timeout=self.auto_detect_timeout_s)
        except RuntimeError:
            return None
        det = median_tilt_detection(dets)
        return detection_to_T_cam2tag(det, self.tag_size_m, self.auto_detector_size_m)

    def _on_auto_sample(self, _req):
        if self._sweep_running():
            return TriggerResponse(success=False, message="a sweep is already running")
        try:
            T_hc2ee = load_T_hc2ee(self.auto_hand_eye_npz)
        except Exception as e:
            return TriggerResponse(success=False,
                                   message=f"cannot load the current hand-eye ({e}); "
                                           "the sweep needs it to aim")
        ok, why = self.tcp_client.wait_for_node(timeout_s=3.0)
        if not ok:
            return TriggerResponse(success=False, message=why)
        self._sweep_cancel.clear()
        runner = SweepRunner(
            self.sweep_cfg,
            get_tcp=self.tcp_client.get_tcp_pose,
            move=lambda pose: self.tcp_client.move_j_to_pose(
                pose, vel=self.auto_move_vel, acc=self.auto_move_acc,
                settle_s=self.auto_settle_s, linear=True),
            detect=self._detect_T_cam2tag,
            capture=self._capture_sample,
            cancelled=self._sweep_cancel.is_set,
            log_info=lambda m: rospy.loginfo("handeye_calib: %s", m),
            log_warn=lambda m: rospy.logwarn("handeye_calib: %s", m),
            progress=self._publish_progress,
        )

        def _worker():
            res = runner.run(T_hc2ee)
            rospy.loginfo("handeye_calib: %s (samples in memory: %d)",
                          res.summary(), len(self.samples))

        self._sweep_thread = threading.Thread(target=_worker, name="handeye-sweep",
                                              daemon=True)
        self._sweep_thread.start()
        cfg = self.sweep_cfg
        return TriggerResponse(
            success=True,
            message=(f"sweep started: up to {cfg.max_samples} views, distances "
                     f"{cfg.distances_m} m, tilts {cfg.tilts_deg} deg, spins "
                     f"{cfg.spins_deg} deg; progress on "
                     f"{rospy.resolve_name('~progress')}"))

    def _on_cancel(self, _req):
        if not self._sweep_running():
            return TriggerResponse(success=False, message="no sweep running")
        self._sweep_cancel.set()
        return TriggerResponse(success=True,
                               message="cancel requested — stops after the current move")

    def _on_compute(self, _req):
        if self._sweep_running():
            return TriggerResponse(success=False,
                                   message="a sweep is running — wait or ~cancel first")
        try:
            with self._sample_lock:
                samples = list(self.samples)
            result = calibrate(
                samples,
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
        if self._sweep_running():
            return TriggerResponse(success=False,
                                   message="a sweep is running — ~cancel first")
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
        if self._sweep_running():
            lines.append("auto_sample: RUNNING")
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
