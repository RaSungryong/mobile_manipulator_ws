#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ra scan pipeline: capture frames from the camera node, run ONNX inference,
publish scan topics, optionally save images.

Split out of arm_controller.py — none of this moves the arm. The controller
positions the tool and then calls scan_point(); everything camera/inference
related lives here.

Owns the ROS publishers for /scan/ra_value, /scan/point_result and
/scan/image. Frames come from basler_camera_node's /camera/capture service
(the camera node owns the device and the VISION lamp — see its docstring);
this class never touches PyPylon.
"""

import os
import json
import time

import numpy as np
import cv2

import rospy
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robot_msgs.srv import CaptureImages
from apriltag_nav.inference_interface import InferenceInterface


class RaScanPipeline:
    """Capture → infer → publish/persist for one scan point at a time."""

    def __init__(self,
                 capture_service='/camera/capture',
                 capture_timeout_s=20.0,
                 use_vision_led=True,
                 num_samples=1,
                 delay_between_samples=0.2,
                 save_images=False,
                 output_dir='/tmp/scan_results',
                 model_path=None):
        self.capture_service_name = capture_service
        self.capture_timeout_s = float(capture_timeout_s)
        self.use_vision_led = bool(use_vision_led)
        self.num_samples = int(num_samples)
        self.delay_between_samples = float(delay_between_samples)
        self.save_images = bool(save_images)
        self.output_dir = output_dir

        self._capture_srv = None
        self.bridge = CvBridge()

        self.inference = InferenceInterface()
        if model_path and not self.inference.load_model(model_path):
            rospy.logerr("[Scan] Model loading failed")

        self.ra_pub     = rospy.Publisher("/scan/ra_value",     Float32, queue_size=10)
        self.result_pub = rospy.Publisher("/scan/point_result", String,  queue_size=10)
        self.image_pub  = rospy.Publisher("/scan/image",        Image,   queue_size=1)
        # Manual open/close override on the camera node (see preopen/release).
        self._active_pub = rospy.Publisher("/camera/set_active", Bool, queue_size=1)

    # --------------------------------------------------
    # DEVICE PRE-OPEN (latency hiding)
    # --------------------------------------------------
    def preopen(self):
        """Ask the camera node to open the device NOW.

        Called at the start of each scan point, before the arm starts moving,
        so the device-open latency overlaps the motion + Keyence adjustment
        instead of adding to it at capture time. Without this the camera's
        idle timer (~5 s) closes the device between points, because
        move+stabilize+adjust usually takes longer than that.

        The VISION lamp is NOT touched here — it stays bracketed with the
        shutter inside the capture service. Best-effort: if the message is
        lost, the capture service opens the device itself as before.
        """
        try:
            self._active_pub.publish(Bool(True))
        except Exception as e:
            rospy.logwarn(f"[Scan] camera preopen failed: {e}")

    def release(self):
        """Let the camera close immediately (end of scan / cancel).

        Counterpart of preopen(): without it the device would stay warm for
        idle_close_sec after the last point for no benefit.
        """
        try:
            self._active_pub.publish(Bool(False))
        except Exception as e:
            rospy.logwarn(f"[Scan] camera release failed: {e}")

    # --------------------------------------------------
    # CAPTURE (via basler_camera_node service)
    # --------------------------------------------------
    def _capture_frames(self):
        """Request a burst of frames from basler_camera_node.

        Returns a list of BGR ndarrays (possibly empty). The camera node keeps
        the device closed between calls and brackets the grab with the VISION
        lamp, so there is nothing to switch on or off here.
        """
        if self._capture_srv is None:
            try:
                rospy.wait_for_service(self.capture_service_name,
                                       timeout=self.capture_timeout_s)
                self._capture_srv = rospy.ServiceProxy(
                    self.capture_service_name, CaptureImages)
            except rospy.ROSException:
                rospy.logerr(
                    f"[Scan] Camera service {self.capture_service_name} "
                    "unavailable — is basler_camera_node running?")
                return []

        try:
            resp = self._capture_srv(
                num_samples=int(self.num_samples),
                delay_between_s=float(self.delay_between_samples),
                use_vision_led=bool(self.use_vision_led),
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"[Scan] Camera capture call failed: {e}")
            # Force a re-resolve in case the node restarted.
            self._capture_srv = None
            return []

        if not resp.success:
            rospy.logwarn(f"[Scan] Camera capture unsuccessful: {resp.message}")
            return []

        frames = []
        for img in resp.images:
            try:
                frames.append(self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8'))
            except Exception as e:
                rospy.logwarn(f"[Scan] Image decode failed: {e}")
        return frames

    # --------------------------------------------------
    # SCAN ONE POINT
    # --------------------------------------------------
    def scan_point(self, point_id, cancelled=None):
        """Capture + infer at the current tool position.

        cancelled: optional zero-arg callable checked between steps, so the
        controller's cancel flag aborts the pipeline without coupling.
        Returns the per-point stats dict, or None if no valid sample.
        """
        cancelled = cancelled or (lambda: False)
        rospy.loginfo(f"[Scan] Point {point_id} — {self.num_samples} sample(s)")

        results = []
        images  = []

        if cancelled():
            return None

        # One service call covers the whole burst: the camera node opens the
        # device, lights the lamp, grabs every sample, then darkens and releases.
        frames = self._capture_frames()
        if not frames:
            rospy.logwarn(f"  [Scan] Point {point_id}: no frames captured")

        for s, frame in enumerate(frames):
            if cancelled():
                break

            t0 = time.time()
            ra_value = self.inference.infer(frame)
            infer_ms = (time.time() - t0) * 1000

            if ra_value is not None:
                results.append({
                    'point_id':  point_id,
                    'sample_id': s + 1,
                    'ra_value':  ra_value,
                    'infer_ms':  infer_ms,
                })
                images.append(frame)

                self.ra_pub.publish(Float32(ra_value))
                rospy.loginfo(
                    f"  Sample {s+1}/{len(frames)}: "
                    f"Ra={ra_value:.4f}  ({infer_ms:.1f}ms)"
                )

                try:
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    )
                except Exception as e:
                    rospy.logwarn(f"  [Scan] Image publish failed: {e}")

        if not results:
            rospy.logerr(f"[Scan] No valid samples at point {point_id}")
            return None

        ra_values = [r['ra_value'] for r in results]
        scan_result = {
            'point_id':    point_id,
            'num_samples': len(results),
            'ra_mean':     float(np.mean(ra_values)),
            'ra_std':      float(np.std(ra_values)),
            'ra_min':      float(np.min(ra_values)),
            'ra_max':      float(np.max(ra_values)),
            'ra_values':   ra_values,
        }
        self.result_pub.publish(json.dumps(scan_result))

        if self.save_images and images:
            os.makedirs(self.output_dir, exist_ok=True)
            for idx, img in enumerate(images):
                fname = (
                    f"point_{point_id}_sample_{idx+1}"
                    f"_ra_{ra_values[idx]:.4f}.png"
                )
                cv2.imwrite(os.path.join(self.output_dir, fname), img)

        return scan_result

    def shutdown(self):
        """Drop the service proxy; the camera node owns the actual device."""
        self._capture_srv = None
