#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Basler camera node — on-demand capture server for the wrist camera.

Sole owner of two pieces of hardware:
  * the Basler camera (PyPylon holds the device exclusively)
  * the VISION lamp (/crevis/led/vision on the Navifra Crevis IO)

No other node may open the camera or drive the VISION lamp. They are owned
together on purpose: the lamp must be lit only while the shutter is open, and
splitting ownership across two nodes guarantees they eventually desynchronise.

Why this is not a streaming publisher
-------------------------------------
The camera must NOT stay on: heat, sensor lifetime and power. So the device is
kept **Closed** between captures — not merely "not grabbing" — and the lamp is
dark. A caller asks for frames via the /camera/capture service and gets them in
the reply, which also removes any chance of pairing a scan point with a stale
frame from a topic.

Consecutive scan points would thrash Open/Close, so the device lingers open for
~idle_close_sec after a capture and closes itself once scanning stops.

Interface
---------
Service:
  /camera/capture      robot_msgs/CaptureImages
Topics:
  /camera/set_active   std_msgs/Bool   in   — force open (true) / close (false)
  /camera/state        std_msgs/String out  — "closed" | "open" | "capturing"
  /basler/image_raw    sensor_msgs/Image out — last captured frame, for debug
                                               only; not a continuous stream

ROS parameters (defaults come from robot.yaml `camera:` when present):
  ~num_samples       (int)   frames per capture when the request says <= 0
  ~delay_between_s   (float) delay between frames in a burst
  ~idle_close_sec    (float) close the device this long after the last capture
                             (0 = close immediately after every capture)
  ~grab_timeout_ms   (int)   per-frame RetrieveResult timeout
  ~warmup_s          (float) settle time after lamp on / grabbing start
  ~publish_last      (bool)  also publish each frame on /basler/image_raw
"""

import os
import threading

import rospy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robot_msgs.srv import CaptureImages, CaptureImagesResponse

from apriltag_nav.camera_interface import CameraInterface
from apriltag_nav.navifra_devices import NavifraDevices
from apriltag_nav.paths import load_yaml_block as _load_yaml_block


class BaslerCameraNode:
    """On-demand capture server. Holds the camera closed unless capturing."""

    def __init__(self):
        rospy.init_node('basler_camera_node', anonymous=False)

        cam_cfg = _load_yaml_block('camera')
        nav_cfg = _load_yaml_block('navifra')

        def p(name, default):
            return rospy.get_param('~' + name, cam_cfg.get(name, default))

        self._num_samples     = int(p('num_samples', 1))
        self._delay_between_s = float(p('delay_between_s', 0.2))
        self._idle_close_sec  = float(p('idle_close_sec', 5.0))
        self._grab_timeout_ms = int(p('grab_timeout_ms', 5000))
        self._warmup_s        = float(p('warmup_s', 0.15))
        self._publish_last    = bool(p('publish_last', True))
        self._service_name    = p('service', '/camera/capture')

        self._bridge = CvBridge()
        self._camera = CameraInterface()

        # The camera is a single exclusive device: serialise every access.
        self._lock = threading.RLock()
        self._is_open = False
        self._idle_timer = None

        # VISION lamp lives here, with the shutter it belongs to.
        self._devices = NavifraDevices(nav_cfg)

        self._state_pub = rospy.Publisher('/camera/state', String,
                                          queue_size=1, latch=True)
        self._image_pub = rospy.Publisher('/basler/image_raw', Image,
                                          queue_size=1)
        rospy.Subscriber('/camera/set_active', Bool, self._cb_set_active,
                         queue_size=1)

        self._srv = rospy.Service(self._service_name, CaptureImages,
                                  self._handle_capture)

        self._publish_state('closed')
        rospy.loginfo(
            f"[BaslerCamera] Ready — service {self._service_name}, "
            f"device closed until requested (idle_close={self._idle_close_sec}s)")

    # ==========================================================
    # STATE
    # ==========================================================
    def _publish_state(self, state):
        try:
            self._state_pub.publish(String(state))
        except Exception:
            pass

    # ==========================================================
    # DEVICE LIFECYCLE
    # ==========================================================
    def _open_locked(self):
        """Open + start grabbing. Caller must hold the lock."""
        if self._is_open:
            return True
        if not self._camera.initialize():
            rospy.logerr("[BaslerCamera] Camera open failed")
            return False
        if not self._camera.start_grabbing():
            rospy.logerr("[BaslerCamera] StartGrabbing failed")
            self._camera.close()
            return False
        self._is_open = True
        rospy.loginfo("[BaslerCamera] Device opened")
        self._publish_state('open')
        return True

    def _close_locked(self):
        """Stop grabbing + Close the device. Caller must hold the lock.

        close() is what actually powers the sensor down — stop_grabbing() alone
        would leave it warm, which is the whole thing we are avoiding.
        """
        if not self._is_open:
            return
        try:
            self._camera.stop_grabbing()
            self._camera.close()
        finally:
            self._is_open = False
            rospy.loginfo("[BaslerCamera] Device closed")
            self._publish_state('closed')

    def _cancel_idle_timer(self):
        if self._idle_timer is not None:
            self._idle_timer.shutdown()
            self._idle_timer = None

    def _arm_idle_timer(self):
        """Close the device once captures stop arriving."""
        self._cancel_idle_timer()
        if self._idle_close_sec <= 0:
            with self._lock:
                self._close_locked()
            return
        self._idle_timer = rospy.Timer(
            rospy.Duration(self._idle_close_sec), self._on_idle, oneshot=True)

    def _on_idle(self, _evt):
        with self._lock:
            self._idle_timer = None
            rospy.loginfo(
                f"[BaslerCamera] Idle {self._idle_close_sec}s — closing device")
            self._close_locked()

    # ==========================================================
    # MANUAL OVERRIDE
    # ==========================================================
    def _cb_set_active(self, msg):
        want = bool(msg.data)
        with self._lock:
            if want:
                self._cancel_idle_timer()
                self._open_locked()
            else:
                self._cancel_idle_timer()
                self._close_locked()

    # ==========================================================
    # CAPTURE SERVICE
    # ==========================================================
    def _handle_capture(self, req):
        n = int(req.num_samples) if req.num_samples > 0 else self._num_samples
        delay = (float(req.delay_between_s) if req.delay_between_s >= 0
                 else self._delay_between_s)
        use_led = bool(req.use_vision_led)

        resp = CaptureImagesResponse()
        resp.images = []

        with self._lock:
            self._cancel_idle_timer()

            if not self._open_locked():
                resp.success = False
                resp.message = "camera open failed"
                return resp

            self._publish_state('capturing')
            led_on = False
            try:
                if use_led:
                    self._devices.vision_led(True)
                    led_on = True
                if self._warmup_s > 0:
                    rospy.sleep(self._warmup_s)

                frames = self._grab_burst(n, delay)
            except Exception as e:
                rospy.logerr(f"[BaslerCamera] Capture failed: {e}")
                resp.success = False
                resp.message = f"capture error: {e}"
                return resp
            finally:
                # Lamp off before anything else can fail. The topic is latched,
                # so a leaked "on" would stay lit indefinitely.
                if led_on:
                    try:
                        self._devices.vision_led(False)
                    except Exception as e:
                        rospy.logerr(f"[BaslerCamera] VISION lamp off failed: {e}")
                self._publish_state('open' if self._is_open else 'closed')
                self._arm_idle_timer()

            if not frames:
                resp.success = False
                resp.message = f"no frames captured (requested {n})"
                return resp

            for f in frames:
                resp.images.append(f)
            resp.success = True
            resp.message = f"captured {len(frames)}/{n}"
            return resp

    def _grab_burst(self, n, delay):
        """Grab n frames as ROS Images. Skips failed grabs rather than aborting."""
        out = []
        for i in range(n):
            frame = self._camera.grab_frame(timeout=self._grab_timeout_ms)
            if frame is None:
                rospy.logwarn(f"[BaslerCamera] Sample {i + 1}/{n} grab failed")
            else:
                msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = rospy.Time.now()
                out.append(msg)
                if self._publish_last:
                    try:
                        self._image_pub.publish(msg)
                    except Exception as e:
                        rospy.logwarn(f"[BaslerCamera] debug publish failed: {e}")
            if i < n - 1 and delay > 0:
                rospy.sleep(delay)
        return out

    # ==========================================================
    # SHUTDOWN
    # ==========================================================
    def shutdown(self):
        with self._lock:
            self._cancel_idle_timer()
            self._close_locked()
        try:
            self._devices.vision_led(False)
        except Exception:
            pass


if __name__ == '__main__':
    node = BaslerCameraNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
