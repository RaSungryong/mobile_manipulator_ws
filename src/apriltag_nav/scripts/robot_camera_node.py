#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot camera node — AprilTag detection for front_cam + side_cam + hand_cam.

Sole owner of AprilTag detection on the three driver-fed cameras:
  * front_cam — Orbbec Femto Bolt (orbbec_camera driver)
  * side_cam  — Intel RealSense D405 (realsense2_camera driver)
  * hand_cam  — Intel RealSense D435 (realsense2_camera driver)

The wrist Basler is NOT one of these — it stays owned by basler_camera_node,
which keeps it closed between captures (see CLAUDE.md); these three are
free-running driver streams that anyone may subscribe to.

This node is detection-only. It NEVER touches /cmd_vel, never calls a stop
method, and never talks to the Navifra base directly. It only publishes one
robot_msgs/AprilTagDetectionArray per processed frame, per camera. Whatever a
detection should trigger (e.g. stopping the mobile base when front_cam sees a
configured tag centered in view) is decided by the SUBSCRIBER
(mobile_controller.py's vision_stop_callback) — this mirrors basler_camera_node
handing raw frames to task_executor rather than deciding anything itself.

Each camera gets its own dt_apriltags Detector instance (not shared): rospy
delivers different topics' callbacks on different threads, and a Detector
wraps a stateful C library instance that is not safe to call concurrently
from two threads. CvBridge has no internal state, so one instance is shared.

Interface
---------
Subscribes (per enabled camera, topic names from robot.yaml `topics:`):
  <NAME_image>   sensor_msgs/Image
  <NAME_info>    sensor_msgs/CameraInfo
Publishes:
  <NAME_detections>  robot_msgs/AprilTagDetectionArray
  /<NAME>/tag_overlay  sensor_msgs/Image — the frame annotated with a centre
                     crosshair, each tag's ID, its offset from the crosshair
                     in px and degrees, its roll/pitch/yaw and how far off
                     square it sits. Rendered only while subscribed to.
Serves (one per camera, always advertised — even while that camera is off):
  /robot_camera/<NAME>/set_enabled   std_srvs/SetBool

Config (robot.yaml `robot_camera:` block; falls back to `robot:` block when
null so every camera shares the existing navigation tag settings by default):
  tag_family
  tag_size:      {front_cam, side_cam, hand_cam}  -- metres, PER CAMERA. The
                 cell has 90 mm floor tags and 30 mm tags on the 정반 step, and
                 dt_apriltags scales pose_t by this, so one global value would
                 corrupt one camera's pose. A null entry falls back to
                 robot.tag_size.
  enabled:       {front_cam, side_cam, hand_cam}  -- startup on/off
  driver_toggle: {front_cam, side_cam, hand_cam}  -- vendor stream service

Each camera is independent: a disabled one runs no Detector and holds no
subscribers, and a missing camera never blocks the others, since a worker
only starts detecting once its own CameraInfo arrives.

Switching at runtime -- no restart needed:
  rosservice call /robot_camera/hand_cam/set_enabled "data: false"
That stops the detector AND asks the vendor driver to stop its stream
(driver_toggle above: orbbec's /<cam>/toggle_color, realsense's /<cam>/enable
-- both std_srvs/SetBool). The driver call is best effort: a camera whose
driver was never launched still toggles its detector, with a warning.

Startup state is `~driver_<name>` AND (`~enable_<name>` > robot.yaml > on):
mobile_manipulator.launch writes ~driver_<name> on every run to say which
drivers it started, and a camera with no driver never gets a detector.
~enable_<name> is the manual override (a standalone rosrun), normally absent,
so robot.yaml decides among the cameras the launch did start.
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R

# Fallback for apriltags (same pattern as mobile_controller.py / basler_camera_node.py)
try:
    from dt_apriltags import Detector
except ImportError:
    import subprocess
    subprocess.check_call(['pip', 'install', 'dt-apriltags'])
    from dt_apriltags import Detector

from robot_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from apriltag_nav.paths import load_yaml_block as _load_yaml_block


_FONT = cv2.FONT_HERSHEY_SIMPLEX
_CROSSHAIR = (80, 220, 255)   # BGR, amber — the optical axis reference
_MARK = (0, 255, 0)           # tag marker + leader line
_TEXT = (0, 255, 255)         # BGR yellow — all overlay text


def _rot_to_matrix(rot):
    """scipy compat: >=1.4 spells it as_matrix(), 1.3 only has as_dcm().
    This machine runs 1.3.3 (the merged stack assumed >=1.4 and crashed
    at runtime on every call)."""
    return rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()


def _rot_from_matrix(m):
    return (R.from_matrix(m) if hasattr(R, 'from_matrix')
            else R.from_dcm(m))


def _orientation(pose_R):
    """(roll, pitch, yaw, tilt_from_normal) in degrees from dt_apriltags pose_R.

    ZYX-intrinsic euler to match the scan CSV convention. tilt_from_normal is
    the angle between the tag's own Z (its surface normal, third column of R)
    and the camera's optical axis — the one number that says "how far from
    square-on", which the euler triple does not show at a glance.
    """
    if pose_R is None:
        return 0.0, 0.0, 0.0, 0.0
    Rm = np.asarray(pose_R, dtype=float).reshape(3, 3)
    roll, pitch, yaw = _rot_from_matrix(Rm).as_euler('zyx',
                                                     degrees=True)[::-1]
    # Camera looks along +Z; the tag faces the camera when its normal is -Z,
    # so a dead-square tag gives Rm[2, 2] = -1 and tilt 0.
    tilt = np.degrees(np.arccos(np.clip(abs(Rm[2, 2]), -1.0, 1.0)))
    return float(roll), float(pitch), float(yaw), float(tilt)


class _CameraTagWorker:
    """Owns one camera's Detector + subscribers; publishes detections for it.

    Independent per camera on purpose (see module docstring) -- no state is
    shared with any other worker.
    """

    def __init__(self, name, image_topic, info_topic, detections_topic,
                 tag_family, tag_size, bridge, driver_service, enabled):
        self.name = name
        self.image_topic = image_topic
        self.info_topic = info_topic
        self.tag_family = tag_family
        self.tag_size = tag_size
        self.bridge = bridge
        self.driver_service = driver_service

        self.detector = None       # built on first enable, kept afterwards
        self.camera_params = None  # (fx, fy, cx, cy), set from CameraInfo
        self._subs = []
        self.enabled = False

        # Advertised even while disabled — the whole point of the service is
        # to be reachable for a camera that is currently off.
        self.pub = rospy.Publisher(detections_topic, AprilTagDetectionArray,
                                    queue_size=10)
        # Debug view for RViz/rqt. Only rendered while subscribed to, so it
        # costs nothing on a headless run.
        self.overlay_pub = rospy.Publisher(f'/{name}/tag_overlay', Image,
                                           queue_size=1)
        self._srv = rospy.Service(f'/robot_camera/{name}/set_enabled',
                                  SetBool, self._srv_set_enabled)

        rospy.loginfo(
            f"[RobotCamera] {self.name}: {image_topic} -> {detections_topic}")
        # Startup never touches the driver: mobile_manipulator.launch already
        # brought it up in the state its use_<name>_cam arg asked for, and
        # re-asserting that state is not a no-op — realsense answers a second
        # "enable" with `open(...) failed. UVC device is streaming!` and can
        # drop the stream it was already serving.
        self._apply(enabled)

    # ---------- runtime switch ----------
    def _srv_set_enabled(self, req):
        return SetBoolResponse(success=True, message=self.set_enabled(req.data))

    def set_enabled(self, enable):
        """Turn this camera's stream + detection on or off, live."""
        if enable == self.enabled:
            return f"{self.name} already {'enabled' if enable else 'disabled'}"

        # Order matters: bring the stream up before subscribing so no frame is
        # missed, and tear the detector down before the stream so no callback
        # runs against a dying device.
        note = self._toggle_driver(True) if enable else ""
        self._apply(enable)
        if not enable:
            note = self._toggle_driver(False)

        return f"{self.name} {'enabled' if enable else 'disabled'}{note}"

    def _apply(self, enable):
        """Attach or detach this camera's subscribers. Never touches the driver."""
        if enable:
            if self.detector is None:
                self.detector = Detector(families=self.tag_family)
            # Re-read the intrinsics: a driver restarted at a different
            # resolution publishes a different K, and reusing the stale one
            # would silently skew every tag pose.
            self.camera_params = None
            self._subs = [
                rospy.Subscriber(self.info_topic, CameraInfo, self._info_cb,
                                 queue_size=1),
                rospy.Subscriber(self.image_topic, Image, self._image_cb,
                                 queue_size=1),
            ]
        else:
            for sub in self._subs:
                sub.unregister()
            self._subs = []

        self.enabled = enable
        rospy.loginfo(f"[RobotCamera] {self.name}: "
                      f"{'enabled' if enable else 'disabled'}")

    def _toggle_driver(self, enable):
        """Start/stop the vendor driver's stream. Best effort on purpose.

        A camera whose driver was never launched (use_<name>_cam:=false) has
        no such service, and that must not stop the detector switch from
        working — so a failure here is reported, not raised.
        """
        if not self.driver_service:
            return ""
        try:
            rospy.wait_for_service(self.driver_service, timeout=2.0)
            rospy.ServiceProxy(self.driver_service, SetBool)(enable)
            return f", driver stream {'on' if enable else 'off'}"
        except Exception as e:
            rospy.logwarn(f"[RobotCamera] {self.name}: "
                          f"driver toggle {self.driver_service} failed: {e}")
            return ", driver toggle unavailable (detector only)"

    # ---------- detection ----------
    def _info_cb(self, msg):
        if self.camera_params is None:
            K = msg.K
            self.camera_params = [K[0], K[4], K[2], K[5]]  # fx, fy, cx, cy

    def _image_cb(self, msg):
        if self.camera_params is None or not self.enabled:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(
                gray, estimate_tag_pose=True,
                camera_params=self.camera_params, tag_size=self.tag_size)

            out = AprilTagDetectionArray()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = self.name
            out.camera_name = self.name
            out.image_height, out.image_width = cv_img.shape[:2]

            for det in detections:
                d = AprilTagDetection()
                d.id = int(det.tag_id)
                d.center_x = float(det.center[0])
                d.center_y = float(det.center[1])
                d.pose_x = float(det.pose_t[0][0])
                d.pose_y = float(det.pose_t[1][0])
                d.pose_z = float(det.pose_t[2][0])
                d.roll, d.pitch, d.yaw, d.tilt_from_normal = _orientation(det.pose_R)
                d.corners = np.asarray(det.corners, dtype=float).ravel().tolist()
                out.detections.append(d)

            self.pub.publish(out)

            # Drawing a 1920x1080 overlay at 30 Hz is not free, so it happens
            # only while something is actually looking (RViz, rqt, a rosbag).
            if self.overlay_pub.get_num_connections() > 0:
                self._publish_overlay(msg, cv_img, out.detections)

        except Exception as e:
            rospy.logerr(f"[RobotCamera] {self.name} processing error: {e}")

    def _publish_overlay(self, src_msg, cv_img, detections):
        out_msg = self.bridge.cv2_to_imgmsg(
            draw_overlay(cv_img, detections, self.name, self.camera_params),
            "bgr8")
        out_msg.header = src_msg.header
        self.overlay_pub.publish(out_msg)


def draw_overlay(cv_img, detections, name, camera_params):
    """Annotated copy of the frame: crosshair, tag IDs, offsets, yaw.

    Kept free of ROS so it can be rendered and checked without a camera.
    """
    img = cv_img.copy()
    h, w = img.shape[:2]
    cx_img, cy_img = w // 2, h // 2
    fx, fy = camera_params[0], camera_params[1]

    # Crosshair = where the optical axis lands. Everything below is measured
    # against it, so it is drawn even with no tag in view.
    cv2.line(img, (cx_img, 0), (cx_img, h), _CROSSHAIR, 1)
    cv2.line(img, (0, cy_img), (w, cy_img), _CROSSHAIR, 1)

    cv2.putText(img, f"{name}  {w}x{h}", (10, 26), _FONT, 0.7, _TEXT, 2)

    for i, d in enumerate(detections):
        tx, ty = int(d.center_x), int(d.center_y)
        # Offset in pixels, and the same offset as a bearing angle — px alone
        # means nothing across cameras with different focal lengths.
        dx, dy = d.center_x - cx_img, d.center_y - cy_img
        bear_x = np.degrees(np.arctan2(dx, fx))
        bear_y = np.degrees(np.arctan2(dy, fy))

        cv2.line(img, (cx_img, cy_img), (tx, ty), _MARK, 1)
        cv2.circle(img, (tx, ty), 6, _MARK, -1)

        # ID is what you look for first, so it gets the big type; the rest is
        # detail and stays small underneath it.
        cv2.putText(img, f"ID {d.id}", (tx + 14, ty - 24), _FONT, 1.4, _TEXT, 3)
        for j, text in enumerate((
                f"yaw {d.yaw:+.1f}deg",
                f"off {dx:+.0f},{dy:+.0f}px ({bear_x:+.1f},{bear_y:+.1f}deg)",
                f"dist {d.pose_z:.3f}m")):
            cv2.putText(img, text, (tx + 14, ty + 4 + j * 20),
                        _FONT, 0.55, _TEXT, 1)

        # Same numbers stacked top-left, readable when tags sit at the frame
        # edge and their per-tag labels run off screen.
        cv2.putText(img,
                    f"[{d.id}] yaw {d.yaw:+6.1f}deg  "
                    f"bearing {bear_x:+.1f},{bear_y:+.1f}deg  "
                    f"{d.pose_z:.2f}m",
                    (10, 52 + i * 20), _FONT, 0.5, _TEXT, 1)

    return img


# Every camera this node can run a detector for. Adding one is a matter of
# adding its name here plus the three <name>_* entries in robot.yaml topics:.
CAMERA_NAMES = ('front_cam', 'side_cam', 'hand_cam')


class RobotCameraNode:
    def __init__(self):
        rospy.init_node('robot_camera_node', anonymous=False)

        topics = _load_yaml_block('topics')
        robot_cfg = _load_yaml_block('robot')
        cam_cfg = _load_yaml_block('robot_camera')

        tag_family = cam_cfg.get('tag_family') or robot_cfg['tag_family']
        tag_size_cfg = cam_cfg.get('tag_size') or {}
        enabled_cfg = cam_cfg.get('enabled') or {}
        driver_cfg = cam_cfg.get('driver_toggle') or {}

        bridge = CvBridge()

        # A worker is built for every camera, enabled or not: a disabled one
        # costs only its (unused) publisher and service, and that service is
        # what lets the camera be switched back on later without a restart.
        self.workers = {}
        for name in CAMERA_NAMES:
            self.workers[name] = _CameraTagWorker(
                name,
                topics[f'{name}_image'], topics[f'{name}_info'],
                topics[f'{name}_detections'],
                tag_family,
                tag_size_cfg.get(name) or robot_cfg['tag_size'],
                bridge,
                driver_cfg.get(name),
                # A detector needs both a running driver and a config that
                # wants it. ~driver_<name> is set by the launch on every run;
                # ~enable_<name> is the manual override for a standalone
                # rosrun and is normally absent.
                rospy.get_param(f'~driver_{name}', True)
                and rospy.get_param(f'~enable_{name}',
                                    enabled_cfg.get(name, True)))

        active = [n for n, w in self.workers.items() if w.enabled]
        if not active:
            rospy.logwarn("[RobotCamera] all cameras disabled — call "
                          "/robot_camera/<name>/set_enabled to start one")

        rospy.loginfo(
            f"[RobotCamera] Ready ({', '.join(active) or 'none active'}) "
            f"— publish-only, no /cmd_vel access")


if __name__ == '__main__':
    node = RobotCameraNode()
    rospy.spin()
