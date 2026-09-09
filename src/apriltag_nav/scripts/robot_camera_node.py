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
                     square it sits. front_cam's overlay also draws the
                     navigation stop columns — FWD at cx +
                     robot.center_x_stop_offset and REV at cx +
                     robot.center_x_stop_offset_reverse (robot.yaml), the
                     columns mobile_controller stops the tag on — so the
                     operator sees the target the drive is aiming at.
                     Rendered only while subscribed to.
Serves (one per camera, always advertised — even while that camera is off):
  /robot_camera/<NAME>/set_enabled   std_srvs/SetBool

Config (robot.yaml `robot_camera:` block; falls back to `robot:` block when
null so every camera shares the existing navigation tag settings by default):
  tag_family
  quad_decimate: {front_cam, side_cam, hand_cam}  -- dt_apriltags detector
                 decimation, PER CAMERA (missing = 1.0 = full resolution).
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
import math
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
_STOP_FWD = (255, 200, 0)     # BGR cyan-blue — forward stop column
_STOP_REV = (255, 0, 255)     # BGR magenta — reverse stop column


def _rot_to_matrix(rot):
    """scipy compat: >=1.4 spells it as_matrix(), 1.3 only has as_dcm().
    This machine runs 1.3.3 (the merged stack assumed >=1.4 and crashed
    at runtime on every call)."""
    return rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()


def _rot_from_matrix(m):
    return (R.from_matrix(m) if hasattr(R, 'from_matrix')
            else R.from_dcm(m))


from apriltag_nav.ground_plane import GroundPlane


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
                 tag_family, tag_size, bridge, driver_service, enabled,
                 quad_decimate=1.0, stop_columns=None, ground_plane=None):
        self.name = name
        # Ground-plane correction (2026-09-08): {enabled, roll_deg,
        # pitch_deg, height_m} from robot.yaml robot_camera.ground_plane.
        # Built once CameraInfo (K + D) has arrived. See
        # apriltag_nav/ground_plane.py for what it does and why.
        self.ground_cfg = dict(ground_plane) if ground_plane else None
        self.ground = None
        self.camera_dist = None    # CameraInfo D (plumb_bob)
        # {'FWD': offset_px, 'REV': offset_px} from the calibrated cx, drawn
        # on the overlay. Only the navigation camera gets them; None = draw
        # nothing beyond the crosshair.
        self.stop_columns = dict(stop_columns) if stop_columns else None
        self.image_topic = image_topic
        self.info_topic = info_topic
        self.tag_family = tag_family
        self.tag_size = tag_size
        # dt_apriltags' default is 2.0 (detect on a half-resolution image).
        # On a 640x480 camera that turns a 90 mm tag at 0.8 m (69 px) into
        # ~35 px: no detection, or a decoded tag with corrupted corners and a
        # 20-30 deg bogus tilt (measured 2026-09-02 on hand_cam — that fed
        # the calibration align loop). Per camera from robot.yaml.
        self.quad_decimate = float(quad_decimate)
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
                self.detector = Detector(families=self.tag_family,
                                         quad_decimate=self.quad_decimate)
                rospy.loginfo(f"[RobotCamera] {self.name}: detector "
                              f"quad_decimate={self.quad_decimate}")
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
            self.camera_dist = list(msg.D) if msg.D else []
            if self.ground_cfg and self.ground_cfg.get('enabled', False):
                try:
                    self.ground = GroundPlane(
                        K[0], K[4], K[2], K[5], self.camera_dist,
                        math.radians(float(self.ground_cfg.get('roll_deg', 0.0))),
                        math.radians(float(self.ground_cfg.get('pitch_deg', 0.0))),
                        float(self.ground_cfg.get('height_m', 0.30)))
                    ax = self.ground.axis_offset_m()
                    rospy.loginfo(
                        "[RobotCamera] %s: ground-plane correction ON — roll %+.3f "
                        "pitch %+.3f deg, height %.1f mm, D %s; optical axis meets "
                        "the floor %+.1f / %+.1f mm from the nadir",
                        self.name, float(self.ground_cfg.get('roll_deg', 0.0)),
                        float(self.ground_cfg.get('pitch_deg', 0.0)),
                        self.ground.h * 1000.0,
                        'applied' if self.ground.D is not None else 'none',
                        ax[0] * 1000.0, ax[1] * 1000.0)
                except Exception as e:
                    rospy.logerr("[RobotCamera] %s: ground-plane correction "
                                 "disabled: %s", self.name, e)
                    self.ground = None

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
                if self.ground is not None:
                    # Re-image through a level, distortion-free virtual
                    # camera: corners / centre in its pixels, pose = the
                    # floor position relative to the lens nadir, pose_z =
                    # the calibrated lens height (so consumers' z/fx
                    # scaling stays consistent). Orientation fields keep
                    # dt_apriltags' values.
                    vc, vcen, g = self.ground.correct(det.corners, det.center)
                    c = AprilTagDetection()
                    c.id, c.roll, c.pitch, c.yaw, c.tilt_from_normal = (
                        d.id, d.roll, d.pitch, d.yaw, d.tilt_from_normal)
                    c.center_x, c.center_y = float(vcen[0]), float(vcen[1])
                    c.pose_x, c.pose_y, c.pose_z = float(g[0]), float(g[1]), float(self.ground.h)
                    c.corners = np.asarray(vc, dtype=float).ravel().tolist()
                    out.detections.append(c)
                else:
                    out.detections.append(d)

            self.pub.publish(out)

            # Drawing a 1920x1080 overlay at 30 Hz is not free, so it happens
            # only while something is actually looking (RViz, rqt, a rosbag).
            if self.overlay_pub.get_num_connections() > 0:
                self._publish_overlay(msg, cv_img, out.detections)

        except Exception as e:
            rospy.logerr(f"[RobotCamera] {self.name} processing error: {e}")

    def _publish_overlay(self, src_msg, cv_img, detections):
        # With the ground-plane correction on, the overlay is drawn on the
        # frame RECTIFIED to the level virtual camera, so the crosshair
        # (= the lens nadir), the stop columns, the boxes and the mm
        # numbers all refer to the same view — the one the detections and
        # mobile_controller use. Without it, the raw frame as before.
        frame = cv_img if self.ground is None else self.ground.rectify(cv_img)
        out_msg = self.bridge.cv2_to_imgmsg(
            draw_overlay(frame, detections, self.name, self.camera_params,
                         self.stop_columns,
                         ground_height=(self.ground.h if self.ground is not None else None)),
            "bgr8")
        out_msg.header = src_msg.header
        self.overlay_pub.publish(out_msg)


def _edge_angle_deg(corners):
    """In-plane angle of the corner0->corner1 edge vs the horizontal
    centre line, 0 = square to the lane — the same number
    mobile_controller aligns on (tag_edge_angle_deg)."""
    c = np.asarray(corners, dtype=float).reshape(4, 2)
    a = np.degrees(np.arctan2(c[1][1] - c[0][1], c[1][0] - c[0][0])) - 90.0
    return (a + 90.0) % 180.0 - 90.0


def draw_overlay(cv_img, detections, name, camera_params, stop_columns=None,
                 ground_height=None):
    """Annotated copy of the frame (2026-09-09 layout, user request):

    * crosshair on the calibrated principal point — with the ground-plane
      correction this is the lens NADIR of the rectified frame the caller
      passes in;
    * the FWD / REV stop columns (navigation camera only), dashed,
      labelled in mm from the crosshair — `offset_px * z / fx` with z =
      `ground_height` (the calibrated lens height) or 0.30 m;
    * per tag: marker, line from the crosshair, and the ID only;
    * top-left, one block per tag, one line per kind:
          ID 118
          offset: (+5.3 mm, -8.2 mm)   x = along the horizontal centre
                                        line (+ = image right = forward),
                                        y = down (+ = robot right), from the
                                        crosshair — pose_x / pose_y in mm
          degree: +0.16                 corner0->corner1 edge vs the
                                        horizontal centre line, 0 = square
    No pixel numbers anywhere. `stop_columns` is {'FWD': px, 'REV': px,
    'REV_skip': 'lo-hi, ...'} from the calibrated cx, exactly as
    mobile_controller computes `target_x = cx + offset`.

    Kept free of ROS so it can be rendered and checked without a camera.
    """
    img = cv_img.copy()
    h, w = img.shape[:2]
    fx, fy = camera_params[0], camera_params[1]
    cx_cal = (camera_params[2] if len(camera_params) > 2
              and camera_params[2] else w / 2.0)
    cy_cal = (camera_params[3] if len(camera_params) > 3
              and camera_params[3] else h / 2.0)
    cx_img, cy_img = int(round(cx_cal)), int(round(cy_cal))
    z_ref = float(ground_height) if ground_height else 0.30

    # Crosshair = the calibrated optical axis / nadir. Everything below is
    # measured against it, so it is drawn even with no tag in view.
    cv2.line(img, (cx_img, 0), (cx_img, h), _CROSSHAIR, 1)
    cv2.line(img, (0, cy_img), (w, cy_img), _CROSSHAIR, 1)

    if stop_columns:
        by_col = {}
        skip_note = stop_columns.get('REV_skip') or ''
        for label, off in stop_columns.items():
            if off is None or label == 'REV_skip':
                continue
            col = int(round(cx_cal + float(off)))
            by_col.setdefault(col, []).append((label, float(off)))
        # One label row per column, stacked from the bottom, so a long REV
        # label that has to sit LEFT of its column cannot overprint FWD's.
        for row, (col, items) in enumerate(sorted(by_col.items())):
            colour = _STOP_REV if any(l == 'REV' for l, _ in items) else _STOP_FWD
            for y0 in range(0, h, 24):
                cv2.line(img, (col, y0), (col, min(h, y0 + 12)), colour, 2)
            text = " / ".join(f"{l} stop {o * z_ref / fx * 1000.0:+.0f} mm"
                              for l, o in items)
            if skip_note and any(l == 'REV' for l, _ in items):
                text += f" (tags {skip_note}: FWD)"
            (tw, _), _ = cv2.getTextSize(text, _FONT, 0.6, 2)
            tx = col + 8 if col + 8 + tw <= w - 4 else max(4, col - 8 - tw)
            cv2.putText(img, text, (tx, h - 14 - row * 24), _FONT, 0.6,
                        colour, 2)

    y_text = 34
    for d in detections:
        tx, ty = int(round(d.center_x)), int(round(d.center_y))
        cv2.line(img, (cx_img, cy_img), (tx, ty), _MARK, 1)
        cv2.circle(img, (tx, ty), 6, _MARK, -1)
        cv2.putText(img, f"ID {d.id}", (tx + 14, ty - 12), _FONT, 1.2, _TEXT, 3)

        # mm from the crosshair: the detection's floor position (pose_x
        # fore / pose_y right, already nadir-relative with the correction;
        # otherwise pixel offset x depth / f).
        if ground_height:
            off_x_mm, off_y_mm = d.pose_x * 1000.0, d.pose_y * 1000.0
        else:
            z = d.pose_z if d.pose_z else z_ref
            off_x_mm = (d.center_x - cx_cal) * z / fx * 1000.0
            off_y_mm = (d.center_y - cy_cal) * z / fy * 1000.0
        deg = _edge_angle_deg(d.corners) if len(d.corners) == 8 else float('nan')
        if abs(deg) < 0.005:
            deg = 0.0          # no '-0.00'
        for line in (f"ID {d.id}",
                     f"offset: ({off_x_mm:+.1f} mm, {off_y_mm:+.1f} mm)",
                     f"degree: {deg:+.2f}"):
            cv2.putText(img, line, (10, y_text), _FONT, 0.8, (0, 0, 0), 4)
            cv2.putText(img, line, (10, y_text), _FONT, 0.8, _TEXT, 2)
            y_text += 30
        y_text += 12

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
        decimate_cfg = cam_cfg.get('quad_decimate') or {}
        enabled_cfg = cam_cfg.get('enabled') or {}
        driver_cfg = cam_cfg.get('driver_toggle') or {}
        ground_cfg = cam_cfg.get('ground_plane') or {}
        # The columns mobile_controller stops the tag on, read from the SAME
        # keys it reads (robot.yaml `robot:`), drawn on front_cam's overlay
        # only — it is the navigation camera. Same fallback rule as the
        # controller: no reverse key -> one shared column.
        fwd_off = float(robot_cfg.get('center_x_stop_offset', 0.0) or 0.0)
        rev_raw = robot_cfg.get('center_x_stop_offset_reverse', None)
        rev_off = float(rev_raw) if rev_raw is not None else fwd_off
        nav_stop_columns = {'FWD': fwd_off, 'REV': rev_off}
        # Target tags that keep the forward column in reverse — shown in the
        # REV label so the operator knows which column applies to a hop.
        skip = robot_cfg.get(
            'stop_offset_skip_tag_ranges',
            robot_cfg.get('center_x_stop_offset_reverse_skip_tag_ranges', [])) or []
        nav_stop_columns['REV_skip'] = ", ".join(
            f"{int(r[0])}-{int(r[1])}" for r in skip
            if isinstance(r, (list, tuple)) and len(r) >= 2)

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
                                    enabled_cfg.get(name, True)),
                quad_decimate=decimate_cfg.get(name, 1.0),
                stop_columns=(nav_stop_columns if name == 'front_cam'
                              else None),
                ground_plane=ground_cfg.get(name))

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
