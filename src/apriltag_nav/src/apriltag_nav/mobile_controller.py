#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import glob
import os
import tempfile
import numpy as np
import tf.transformations as tft
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# No image handling here any more: robot_camera_node owns the cameras and
# publishes AprilTagDetectionArray, this class only consumes it. cv2 /
# CvBridge / dt_apriltags went with the detector it used to run.
from robot_msgs.msg import Pose2DWithFlag  # Custom message
from robot_msgs.msg import AprilTagDetectionArray


def tag_edge_angle_deg(corners):
    """In-plane angle of the tag's corner0->corner1 edge, normalised to [-90, 90].

    The -90 removes front_cam's 2026-08-13 rotation about its optical axis: a
    floor direction along +body X imaged at -90 deg before it and 0 deg after.
    """
    dx = corners[1][0] - corners[0][0]
    dy = corners[1][1] - corners[0][1]
    angle = math.degrees(math.atan2(dy, dx)) - 90.0
    return (angle + 90.0) % 180.0 - 90.0


class MobileController:
    """
    Low-level control of the robot.
    Handles:
    - Camera/Tag Detection
    - Movement (Pure Pursuit, Pivot, Alignment)
    - Communication with Manipulator (Scan logic)
    """

    def __init__(self, config, map_manager):
        self.cfg = config
        self.map_mgr = map_manager
        
        # ROS Setup
        self.cmd_pub = rospy.Publisher(self.cfg['topics']['cmd_vel'], Twist, queue_size=10)
        self.pose_pub = rospy.Publisher(self.cfg['topics']['robot_pose'], Pose2DWithFlag, queue_size=10)

        # Navigation reads front_cam's detections instead of detecting for
        # itself. This node used to run its own dt_apriltags Detector over
        # topics.camera_rgb (/rgb) -- a topic nothing publishes, so
        # detected_tags stayed empty and /robot_pose never went out. Pointing
        # it at the camera robot_camera_node already processes both fixes that
        # and avoids running the detector twice over the same frames.
        self.detections_sub = rospy.Subscriber(
            self.cfg['topics']['front_cam_detections'],
            AprilTagDetectionArray, self.detections_callback)
        # Still needed for cx alone (the stop condition compares the tag's
        # centre column against the image centre); cx/cy are not simply w/2, h/2.
        self.info_sub = rospy.Subscriber(self.cfg['topics']['front_cam_info'], CameraInfo, self.info_callback)
        self.odom_sub = rospy.Subscriber(self.cfg['topics']['odom'], Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber(self.cfg['topics']['scan_signal'], Bool, self.scan_callback)

        # Vision-triggered soft stop (see robot.yaml `vision_stop:`). front_cam
        # detections only -- robot_camera_node publishes, we decide. Driven
        # from detections_callback rather than its own subscription, so the
        # stop decision always sees exactly the frame navigation is acting on.
        vision_cfg = self.cfg.get('vision_stop', {}) or {}
        self.vision_stop_enabled = bool(vision_cfg.get('enabled', False))
        self.vision_stop_tag_ids = set(vision_cfg.get('stop_tag_ids', []) or [])
        self.vision_stop_tolerance_px = float(vision_cfg.get('center_tolerance_px', 40.0))

        # Internal State
        self.detected_tags = {}  # {id: {x, y, z, yaw, ...}}
        self.current_theta = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.camera_params = None
        self.image_center_x_fallback = 0.0
        self.image_center_y_fallback = 0.0
        self.scan_finished_signal = False
        self.last_known_tag = None  # Tracks the last visited tag
        self.stop_sleep_duration = self.cfg['robot'].get('stop_sleep_duration', 0.0)
        temp_cfg = self.cfg['robot'].get('temporary_missing_tags', {}) or {}
        self.temp_missing_tags_enabled = bool(temp_cfg.get('enabled', False))
        self.temp_missing_tag_ids = set(
            int(v) for v in (temp_cfg.get('tag_ids', []) or []))
        self.temp_missing_arrival_ratio = float(
            temp_cfg.get('odom_arrival_ratio', 0.98))

        # Velocity ramping state (for smooth acceleration/deceleration)
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_vel_time = None

        # Cache frequently used config values
        self.max_linear = self.cfg['robot']['max_linear_speed']
        self.max_angular = self.cfg['robot']['max_angular_speed']
        self.min_linear = self.max_linear * self.cfg['robot'].get('min_linear_factor', 0.3)
        self.min_angular = self.max_angular * self.cfg['robot'].get('min_angular_factor', 0.25)
        self.linear_accel = self.cfg['robot'].get('linear_accel', 0.05)
        self.angular_accel = self.cfg['robot'].get('angular_accel', 0.3)
        self.ramp_enabled = self.cfg['robot'].get('ramp_enabled', True)

        # ⚠️ There is deliberately NO absolute speed floor — see robot.yaml.
        # The value below applies only inside the final approach; nothing else
        # is clamped to it.
        self.final_approach_speed = float(
            self.cfg['robot'].get('final_approach_speed', 0.015))
        # Final approach: distance to the stop point at which we drop to
        # final_approach_speed and latch there until the stop condition fires.
        self.final_approach_dist = float(
            self.cfg['robot'].get('final_approach_dist', 0.06))
        # Latch state, all set by _latch_final_approach.
        self._final_approach = False
        self._fa_v0 = 0.0       # speed on entering the zone
        self._fa_d0 = 0.0       # distance to the stop point on entering
        self._fa_s0 = 0.0       # odom travel on entering
        self._fa_accel = self.linear_accel

        # Blind-segment steering: before the next tag is visible, use the
        # calibrated/map tag coordinates plus odometry to keep the front camera
        # on the line that should put the next floor tag through image centre.
        pred_cfg = self.cfg['robot'].get('predictive_centering', {}) or {}
        self.pred_centering_enabled = bool(pred_cfg.get('enabled', True))
        self.pred_use_map_world = bool(pred_cfg.get('use_map_world', True))
        self.pred_fallback_to_map_yaml = bool(
            pred_cfg.get('fallback_to_map_yaml', False))
        self.pred_map_world_path = pred_cfg.get('map_world_path', 'latest')
        self.pred_lookahead = float(
            pred_cfg.get('lookahead_m', self.cfg['robot']['look_ahead_base']))
        self.pred_gain_forward = float(
            pred_cfg.get('gain_forward', self.cfg['robot']['pp_gain_forward']))
        self.pred_gain_backward = float(
            pred_cfg.get('gain_backward', self.cfg['robot']['pp_gain_backward']))
        self.pred_heading_gain = float(pred_cfg.get('heading_gain', 0.4))
        self.approach_heading_gain = float(
            pred_cfg.get('approach_heading_gain',
                         self.cfg['robot'].get('align_gain', 0.8)))
        self.approach_center_y_tolerance_px = float(
            pred_cfg.get('approach_center_y_tolerance_px', 20.0))
        self.approach_yaw_tolerance_deg = float(
            pred_cfg.get('approach_yaw_tolerance_deg',
                         self.cfg['robot'].get('align_threshold_deg', 0.5)))
        self.approach_slow_window_px = float(
            pred_cfg.get('approach_slow_window_px', 180.0))
        self.approach_error_slow_speed = float(
            pred_cfg.get('approach_error_slow_speed',
                         self.final_approach_speed))
        self.post_move_align_enabled = bool(
            pred_cfg.get('post_move_align_enabled', True))
        self.post_move_align_y_tolerance_px = float(
            pred_cfg.get('post_move_align_y_tolerance_px',
                         self.approach_center_y_tolerance_px))
        self.record_alignment_result = bool(
            pred_cfg.get('record_alignment_result', True))
        self.alignment_result_path = pred_cfg.get(
            'alignment_result_path',
            '~/.ros/path_tag_locator/tag_alignment_results.yaml')
        self._last_prediction_segment_active = False
        self.pred_calibrated_xy = {}
        self.pred_loaded_map_world_path = None
        if self.pred_centering_enabled and self.pred_use_map_world:
            self._load_calibrated_prediction_map(self.pred_map_world_path)

        self.stop_requested = False      # Any interruption
        self.emergency_stop = False      # Only true STOP command

    def _is_temp_missing_tag(self, tag_id):
        return (self.temp_missing_tags_enabled and
                int(tag_id) in self.temp_missing_tag_ids)

    def _expand_path(self, path):
        return os.path.expandvars(os.path.expanduser(str(path)))

    def _latest_map_world_path(self):
        files = sorted(
            glob.glob(self._expand_path("~/.ros/path_tag_locator/map_world_*.yaml")),
            reverse=True)
        return files[0] if files else None

    def _resolve_map_world_path(self, path):
        return (self._latest_map_world_path()
                if not path or path == 'latest'
                else self._expand_path(path))

    def _refresh_calibrated_prediction_map(self):
        if not self.pred_centering_enabled or not self.pred_use_map_world:
            return
        yaml_path = self._resolve_map_world_path(self.pred_map_world_path)
        if yaml_path and yaml_path != self.pred_loaded_map_world_path:
            self._load_calibrated_prediction_map(self.pred_map_world_path)

    def _load_calibrated_prediction_map(self, path):
        """Load path_tag_locator's map_world_*.yaml for relative tag geometry."""
        try:
            yaml_path = self._resolve_map_world_path(path)
            if not yaml_path:
                rospy.logwarn(
                    "[PredictiveCentering] no map_world_*.yaml found; blind steering unchanged")
                return
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            tags = data.get('tags', {}) or {}
            loaded = {}
            for raw_id, entry in tags.items():
                pos = entry.get('position_m')
                if pos is None or len(pos) < 2:
                    continue
                loaded[int(raw_id)] = (float(pos[0]), float(pos[1]))
            self.pred_calibrated_xy = loaded
            rospy.loginfo(
                "[PredictiveCentering] loaded %d calibrated tag positions from %s",
                len(loaded), yaml_path)
            self.pred_loaded_map_world_path = yaml_path
        except Exception as e:
            self.pred_calibrated_xy = {}
            self.pred_loaded_map_world_path = None
            rospy.logwarn(
                "[PredictiveCentering] failed to load map_world '%s': %s; blind steering unchanged",
                path, e)

    def _prediction_xy(self, tag_id):
        """Return ((x, y), source) for blind steering."""
        if int(tag_id) in self.pred_calibrated_xy:
            return self.pred_calibrated_xy[int(tag_id)], 'map_world'
        if self.pred_use_map_world and not self.pred_fallback_to_map_yaml:
            return None, None
        xy = self._map_yaml_xy(tag_id)
        return (xy, 'map.yaml') if xy is not None else (None, None)

    def _map_yaml_xy(self, tag_id):
        info = self.map_mgr.get_tag_info(tag_id)
        if info is None or 'x' not in info or 'y' not in info:
            return None
        return (float(info['x']), float(info['y']))

    def _make_prediction_segment(self, start_id, target_id, direction,
                                 start_odom_x, start_odom_y,
                                 start_theta, fallback_distance):
        if not self.pred_centering_enabled or start_id is None:
            return None
        self._refresh_calibrated_prediction_map()
        start_xy, start_src = self._prediction_xy(start_id)
        target_xy, target_src = self._prediction_xy(target_id)
        if start_xy is None or target_xy is None:
            return None

        source = start_src
        if start_src != target_src:
            if self.pred_use_map_world and not self.pred_fallback_to_map_yaml:
                return None
            start_xy = self._map_yaml_xy(start_id)
            target_xy = self._map_yaml_xy(target_id)
            if start_xy is None or target_xy is None:
                return None
            source = 'map.yaml'

        dx = target_xy[0] - start_xy[0]
        dy = target_xy[1] - start_xy[1]
        seg_len = math.hypot(dx, dy)
        if seg_len < 1e-6:
            return None

        motion_bearing = math.atan2(dy, dx)
        body_bearing = motion_bearing
        if 'backward' in direction:
            body_bearing = math.atan2(
                math.sin(motion_bearing + math.pi),
                math.cos(motion_bearing + math.pi))

        if source != 'map_world':
            seg_len = float(fallback_distance)

        return {
            'start_xy': start_xy,
            'target_xy': target_xy,
            'seg_len': seg_len,
            'motion_bearing': motion_bearing,
            'body_bearing': body_bearing,
            'start_odom_x': float(start_odom_x),
            'start_odom_y': float(start_odom_y),
            'start_theta': float(start_theta),
            'source': source,
        }

    def _predictive_centering_omega(self, segment, speed):
        """Preview-control omega for the blind part of a tag-to-tag segment."""
        if segment is None or abs(speed) < 1e-6:
            return 0.0

        dx_o = self.odom_x - segment['start_odom_x']
        dy_o = self.odom_y - segment['start_odom_y']
        map_from_odom = segment['body_bearing'] - segment['start_theta']
        c = math.cos(map_from_odom)
        s = math.sin(map_from_odom)
        dx_m = c * dx_o - s * dy_o
        dy_m = s * dx_o + c * dy_o

        sx, sy = segment['start_xy']
        rx = sx + dx_m
        ry = sy + dy_m

        ux = math.cos(segment['motion_bearing'])
        uy = math.sin(segment['motion_bearing'])
        rel_x = rx - sx
        rel_y = ry - sy
        progress = rel_x * ux + rel_y * uy

        lookahead = max(0.05, self.pred_lookahead)
        preview_s = min(segment['seg_len'], max(0.0, progress + lookahead))
        tx = sx + ux * preview_s
        ty = sy + uy * preview_s

        yaw_in_map = segment['body_bearing'] + (
            self.current_theta - segment['start_theta'])
        yaw_in_map = math.atan2(math.sin(yaw_in_map), math.cos(yaw_in_map))
        fx = math.cos(yaw_in_map)
        fy = math.sin(yaw_in_map)
        right_x = math.cos(yaw_in_map - math.pi / 2.0)
        right_y = math.sin(yaw_in_map - math.pi / 2.0)

        to_target_x = tx - rx
        to_target_y = ty - ry
        forward = to_target_x * fx + to_target_y * fy
        right = to_target_x * right_x + to_target_y * right_y

        L = max(abs(forward), lookahead, self.cfg['robot']['look_ahead_base'])
        alpha = math.atan2(-right, L)
        curvature = (2.0 * math.sin(alpha)) / L
        gain = self.pred_gain_backward if speed < 0 else self.pred_gain_forward

        desired_heading = segment['body_bearing']
        heading_error = desired_heading - yaw_in_map
        heading_error = math.atan2(math.sin(heading_error),
                                   math.cos(heading_error))

        omega = abs(speed) * curvature * gain + self.pred_heading_gain * heading_error
        return float(np.clip(omega, -self.max_angular, self.max_angular))

    def _atomic_write_yaml(self, data, path):
        p = self._expand_path(path)
        parent = os.path.dirname(p)
        if parent:
            if not os.path.isdir(parent):
                os.makedirs(parent)
        fd, tmp_path = tempfile.mkstemp(
            prefix=os.path.basename(p) + ".",
            suffix=".tmp",
            dir=parent or None)
        try:
            with os.fdopen(fd, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False,
                               sort_keys=False, allow_unicode=True)
            os.replace(tmp_path, p)
        except Exception:
            try:
                os.remove(tmp_path)
            except OSError:
                pass
            raise

    def _load_yaml_if_exists(self, path):
        p = self._expand_path(path)
        if not os.path.exists(p):
            return {}
        with open(p, 'r') as f:
            return yaml.safe_load(f) or {}

    def _tag_entry_from_yaml(self, data, tag_id):
        tags = data.get('tags', {}) or {}
        if tag_id in tags:
            return tags[tag_id]
        return tags.get(str(tag_id))

    def _record_alignment_result(self, tag_id, tag, yaw_error_deg):
        """Write final alignment residual to a separate result YAML."""
        if not self.record_alignment_result:
            return

        map_world_path = self._resolve_map_world_path(self.pred_map_world_path)
        if not map_world_path or not os.path.exists(map_world_path):
            rospy.logwarn(
                "[AlignmentRecord] no map_world_*.yaml found; result not recorded")
            return

        try:
            map_data = self._load_yaml_if_exists(map_world_path)
            map_entry = self._tag_entry_from_yaml(map_data, tag_id)
            if map_entry is None:
                rospy.logwarn(
                    "[AlignmentRecord] tag %s is not in %s; result not recorded",
                    tag_id, map_world_path)
                return

            calibrated_pos = map_entry.get('position_m')
            if calibrated_pos is None or len(calibrated_pos) < 3:
                rospy.logwarn(
                    "[AlignmentRecord] tag %s has no position_m in %s; result not recorded",
                    tag_id, map_world_path)
                return
            calibrated_rpy = map_entry.get('rpy_deg', [0.0, 0.0, 0.0])

            cx = (self.camera_params[2]
                  if self.camera_params is not None
                  else self.image_center_x_fallback)
            cy = (self.camera_params[3]
                  if self.camera_params is not None
                  else self.image_center_y_fallback)
            fx = self.camera_params[0] if self.camera_params is not None else 0.0
            fy = self.camera_params[1] if self.camera_params is not None else 0.0

            dx_px = float(tag['center_x'] - cx)
            dy_px = float(tag['center_y'] - cy)
            depth_m = float(tag.get('z', 0.0))
            tag_fore_aft_m = float(tag.get('x', 0.0))
            tag_lateral_m = float(tag.get('y', 0.0))
            camera_from_tag_m = [-tag_fore_aft_m, -tag_lateral_m]
            camera_from_tag_mm = [
                camera_from_tag_m[0] * 1000.0,
                camera_from_tag_m[1] * 1000.0,
            ]
            camera_from_tag_norm_mm = math.hypot(
                camera_from_tag_mm[0], camera_from_tag_mm[1])
            yaw_rad = 0.0
            if calibrated_rpy is not None and len(calibrated_rpy) >= 3:
                yaw_rad = math.radians(float(calibrated_rpy[2]))
            cyaw = math.cos(yaw_rad)
            syaw = math.sin(yaw_rad)
            world_offset_m = [
                cyaw * camera_from_tag_m[0] - syaw * camera_from_tag_m[1],
                syaw * camera_from_tag_m[0] + cyaw * camera_from_tag_m[1],
                0.0,
            ]
            world_offset_mm = [v * 1000.0 for v in world_offset_m]
            estimated_camera_center_m = [
                float(calibrated_pos[0]) + world_offset_m[0],
                float(calibrated_pos[1]) + world_offset_m[1],
                float(calibrated_pos[2]) + world_offset_m[2],
            ]

            dx_mm = None
            dy_mm = None
            norm_mm = None
            if fx > 0.0 and fy > 0.0 and depth_m > 0.0:
                dx_mm = dx_px * depth_m / fx * 1000.0
                dy_mm = dy_px * depth_m / fy * 1000.0
                norm_mm = math.hypot(dx_mm, dy_mm)

            result = {
                'updated_at_ros_time': float(rospy.Time.now().to_sec()),
                'source_map_world_path': map_world_path,
                'yaw_error_deg': float(yaw_error_deg),
                'calibrated_tag_position_m': [
                    float(calibrated_pos[0]),
                    float(calibrated_pos[1]),
                    float(calibrated_pos[2]),
                ],
                'calibrated_tag_rpy_deg': [float(v) for v in calibrated_rpy],
                'estimated_camera_center_position_m': estimated_camera_center_m,
                'world_offset_from_calibrated_tag_m': world_offset_m,
                'world_offset_from_calibrated_tag_mm': world_offset_mm,
                'image_center_px': [float(cx), float(cy)],
                'tag_center_px': [float(tag['center_x']), float(tag['center_y'])],
                'image_offset_px': [dx_px, dy_px],
                'camera_center_offset_from_tag_m': camera_from_tag_m,
                'camera_center_offset_from_tag_mm': camera_from_tag_mm,
                'camera_center_offset_from_tag_norm_mm': camera_from_tag_norm_mm,
                'tag_pose_m': [
                    tag_fore_aft_m,
                    tag_lateral_m,
                    depth_m,
                ],
            }
            if dx_mm is not None:
                result['image_center_offset_mm'] = [float(dx_mm), float(dy_mm)]
                result['image_center_offset_norm_mm'] = float(norm_mm)

            result_path = self._expand_path(self.alignment_result_path)
            result_data = self._load_yaml_if_exists(result_path)
            if not result_data:
                result_data = {
                    'frame': 'front_camera_alignment_result',
                    'note': (
                        'Separate runtime alignment measurements. '
                        'Calibration map_world_*.yaml is read as the reference '
                        'and is not modified by this file.'
                    ),
                    'tags': {},
                }
            result_data.setdefault('tags', {})[int(tag_id)] = result
            self._atomic_write_yaml(result_data, result_path)
            rospy.loginfo(
                "[AlignmentRecord] tag %s result recorded to %s: %.2f mm from reference",
                tag_id, result_path, camera_from_tag_norm_mm)
        except Exception as e:
            rospy.logwarn(
                "[AlignmentRecord] failed to record tag %s alignment result: %s",
                tag_id, e)

    def info_callback(self, msg):
        if self.camera_params is None:
            K = msg.K
            self.camera_params = [K[0], K[4], K[2], K[5]] # fx, fy, cx, cy
            rospy.loginfo("Camera params received.")

    def odom_callback(self, msg):
        # Extract position
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        # Extract orientation
        q = msg.pose.pose.orientation
        euler = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_theta = euler[2]

    def detections_callback(self, msg):
        """front_cam detections from robot_camera_node -> navigation state."""
        try:
            # Atomic swap: build a fresh dict locally, then replace the
            # attribute in one assignment. This avoids the race where
            # consumers iterating `self.detected_tags` from the main
            # control thread would see a transiently empty/partial dict
            # while the callback rebuilds it in-place.
            new_tags = {}
            for det in msg.detections:
                # front_cam was rotated -90 deg about its optical axis on
                # 2026-08-13, so in the frame the detector reports in:
                #   pose_x / center_x = FORE-AFT (+ = ahead of the lens)
                #   pose_y / center_y = LATERAL  (+ = the robot's right, wall)
                #   pose_z            = the ~0.30 m camera height, not a range
                # corners arrive flattened; reshaping to 4x2 keeps the
                # corners[1][0] - corners[0][0] maths in tag_edge_angle_deg.
                new_tags[det.id] = {
                    'x': det.pose_x,
                    'y': det.pose_y,
                    'z': det.pose_z,
                    'corners': np.asarray(det.corners, dtype=float).reshape(4, 2),
                    'center_x': det.center_x,
                    'center_y': det.center_y
                }
            self.detected_tags = new_tags
            self.image_center_x_fallback = msg.image_width / 2.0
            self.image_center_y_fallback = msg.image_height / 2.0

            if self.detected_tags:
                rospy.loginfo_throttle(2.0, f"[Vision] Tags detected: {list(self.detected_tags.keys())}")
            else:
                rospy.logdebug_throttle(2.0, "[Vision] No tags detected.")

            self.vision_stop_callback(msg)

        except Exception as e:
            rospy.logerr(f"Detection processing error: {e}")


    def vision_stop_callback(self, msg):
        """front_cam only: soft-stop when a configured tag is centered.

        robot_camera_node only publishes detections; the decision and the
        /cmd_vel write happen here. Guarded on self.stop_requested so this
        does not re-issue stop() every frame once already stopped -- the
        robot stays stopped until move_to_tag() clears the flag for the next
        TASK/GOTO, exactly like the existing preempt_stop_robot() callers.
        """
        if self.stop_requested or not self.vision_stop_tag_ids:
            return
        cx, cy = msg.image_width / 2.0, msg.image_height / 2.0
        for det in msg.detections:
            if det.id not in self.vision_stop_tag_ids:
                continue
            dist_px = math.hypot(det.center_x - cx, det.center_y - cy)
            if dist_px <= self.vision_stop_tolerance_px:
                rospy.logwarn(
                    f"[Vision] Stop tag {det.id} centered (dist={dist_px:.1f}px) "
                    f"on front_cam -> preempt_stop_robot()")
                self.preempt_stop_robot()
                return

    def scan_callback(self, msg):
        """External node says scan is done."""
        if msg.data:
            self.scan_finished_signal = True

    # =========================================================
    # MOVEMENT PRIMITIVES
    # =========================================================


    def emergency_stop_robot(self):
        rospy.logerr("[Robot] EMERGENCY STOP (external command)")
        self.stop_requested = True
        self.emergency_stop = True
        self.stop()

    def preempt_stop_robot(self):
        rospy.logwarn("[Robot] Preempt stop (task switch)")
        self.stop_requested = True
        self.emergency_stop = False
        self.stop()


    def stop(self):
        """Immediate stop - resets ramping state."""
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_vel_time = None
        self._publish_vel(0, 0)

    def clear_stop_flag(self):
        self.stop_requested = False
        self.emergency_stop = False
        # Clearing the stop flags is what precedes the next move, so the final
        # approach must not still be latched from the previous one.
        self._final_approach = False


    def _publish_vel(self, linear, angular):
        """Direct velocity publish without ramping."""
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def send_vel(self, linear, angular, linear_accel=None):
        """
        Send velocity command with optional ramping for smooth acceleration.
        Uses configured acceleration limits to prevent jerky motion.

        `linear_accel` overrides the configured limit for this call only. The
        final approach uses it: honouring `final_approach_dist` from an
        arbitrary entry speed can need more deceleration than the general ramp
        allows, and its own envelope is already smooth — the general limiter
        would only stop it following that curve.
        """
        if not self.ramp_enabled:
            self._publish_vel(linear, angular)
            return

        now = rospy.Time.now()
        if self.last_vel_time is None:
            dt = 0.05  # Default dt for first call
        else:
            dt = (now - self.last_vel_time).to_sec()
            dt = max(0.001, min(dt, 0.2))  # Clamp dt to reasonable range
        self.last_vel_time = now

        # Ramp linear velocity
        linear_diff = linear - self.current_linear_vel
        max_linear_change = (self.linear_accel if linear_accel is None
                             else linear_accel) * dt
        if abs(linear_diff) > max_linear_change:
            self.current_linear_vel += max_linear_change if linear_diff > 0 else -max_linear_change
        else:
            self.current_linear_vel = linear

        # Ramp angular velocity
        angular_diff = angular - self.current_angular_vel
        max_angular_change = self.angular_accel * dt
        if abs(angular_diff) > max_angular_change:
            self.current_angular_vel += max_angular_change if angular_diff > 0 else -max_angular_change
        else:
            self.current_angular_vel = angular

        self._publish_vel(self.current_linear_vel, self.current_angular_vel)
        rospy.loginfo_throttle(1.0, f"[Velocity] target:{linear:.3f} -> actual:{self.current_linear_vel:.3f}")

    def get_current_tag_id(self):
        """
        Returns the ID of the closest/most central detected tag.
        Returns None if no tags visible.
        """
        if not self.detected_tags:
            return None
        # Simple heuristic: return the one with smallest lateral offset (closest to center)
        # or closest distance. Let's use distance (calculated from x and y for top-down view)
        best_id = min(self.detected_tags, key=lambda k: math.hypot(self.detected_tags[k]['x'], self.detected_tags[k]['y']))
        return best_id

    def align_to_tag(self, tag_id):
        rospy.loginfo(f"Aligning to tag {tag_id}...")
        rate = rospy.Rate(10)

        align_gain = self.cfg['robot'].get('align_gain', 0.8)
        align_threshold = self.cfg['robot'].get('align_threshold_deg', 0.5)

        while not rospy.is_shutdown():

            # ★ PREEMPT / STOP CHECK
            if self.stop_requested:
                rospy.logwarn("[Robot] align_to_tag interrupted")
                self.stop()
                return False

            if tag_id not in self.detected_tags:
                self.stop()
                rate.sleep()
                continue

            angle_deg = tag_edge_angle_deg(self.detected_tags[tag_id]['corners'])

            if abs(angle_deg) < align_threshold:
                tag = dict(self.detected_tags[tag_id])
                self.stop()
                rospy.loginfo(f"Alignment Complete. Final Angle: {angle_deg:.2f}")
                self._record_alignment_result(tag_id, tag, angle_deg)
                return True

            angular_vel = -math.radians(angle_deg) * align_gain
            angular_vel = np.clip(angular_vel, -self.max_angular, self.max_angular)

            self.send_vel(0, angular_vel)
            rate.sleep()


    def calculate_robot_pose(self, tag_id):
        """
        Calculates the robot center's pose relative to the tag, 
        mirroring the original V9 logic.
        """
        if tag_id not in self.detected_tags:
            return None
            
        tag = self.detected_tags[tag_id]
        # pose_y is the lateral offset since front_cam's 2026-08-13 rotation:
        # image row + (down) = the robot's right, so the sign is unchanged.
        lateral = tag['y']

        # Original V9 uses corners to get a precise alignment angle
        corners = tag.get('corners')
        align_angle_deg = 0.0
        if corners is not None:
            align_angle_deg = tag_edge_angle_deg(corners)

        tag_info = self.map_mgr.get_tag_info(tag_id)
        zone = tag_info.get('zone', 'A') if tag_info else 'A'
        
        # 1. Calculate World Coordinates (Camera Position)
        # Mirroring get_robot_pose_from_tag logic
        tag_x = tag_info['x'] if tag_info else 0.0
        tag_y = tag_info['y'] if tag_info else 0.0
        
        if zone == 'A' or zone == 'DOCK':
            robot_x, robot_y = tag_x, tag_y + lateral
            heading = align_angle_deg
        elif zone in ['B', 'D']:
            robot_x, robot_y = tag_x - lateral, tag_y
            heading = 90 + align_angle_deg
        elif zone in ['C', 'E']:
            robot_x, robot_y = tag_x + lateral, tag_y
            heading = -90 + align_angle_deg
        else:
            robot_x, robot_y, heading = tag_x, tag_y, 0.0

        # 2. Convert to Manipulator Coordinates (world_to_manipulator)
        manip_cam_x = -robot_y
        manip_cam_y = -robot_x
        
        # 3. Apply Camera-to-Robot-Center Offset (apply_robot_center_offset)
        cam_offset = self.cfg['robot'].get('camera_offset', 0.45)
        
        if zone == 'A' or zone == 'DOCK':
            final_x, final_y = manip_cam_x, manip_cam_y + cam_offset
        elif zone in ['B', 'D']:
            final_x, final_y = manip_cam_x + cam_offset, manip_cam_y
        elif zone in ['C', 'E']:
            final_x, final_y = manip_cam_x - cam_offset, manip_cam_y
        else:
            final_x, final_y = manip_cam_x, manip_cam_y

        return final_x, final_y, heading

    def go_to_next_tag(self, target_id, known_start_id=None):

        # ★ NEW: early abort
        if self.stop_requested:
            rospy.logwarn("[Robot] go_to_next_tag aborted")
            return False

        current_id = self.get_current_tag_id()

        if (known_start_id is not None and
                self._is_temp_missing_tag(known_start_id) and
                current_id != known_start_id and
                self.map_mgr.get_edge(known_start_id, target_id)):
            rospy.logwarn(
                "[TemporaryMissingTag] using virtual start tag %s instead of visible tag %s",
                known_start_id, current_id)
            current_id = known_start_id

        if current_id is None:
            if known_start_id is not None:
                rospy.logwarn(f"No tag visible. Using known start ID: {known_start_id}")
                current_id = known_start_id
            elif self.last_known_tag is not None:
                rospy.logwarn(f"No tag visible. Using last known tag: {self.last_known_tag}")
                current_id = self.last_known_tag
            else:
                rospy.logwarn("No tag visible and no known location. Cannot start move.")
                return False

        edge = self.map_mgr.get_edge(current_id, target_id)
        if not edge:
            rospy.logerr(f"No edge from {current_id} to {target_id}")
            return False

        action_type = edge['type']
        direction = edge['direction']

        rospy.loginfo(
            f"Going to {target_id} from {current_id} ({action_type}, {direction})"
        )

        if action_type == 'pivot':
            return self.execute_pivot(direction)

        elif action_type == 'move':
            start_info = self.map_mgr.get_tag_info(current_id)
            target_info = self.map_mgr.get_tag_info(target_id)

            if not start_info or not target_info:
                rospy.logerr("Missing tag info for move")
                return False

            dx = target_info['x'] - start_info['x']
            dy = target_info['y'] - start_info['y']
            total_distance = math.hypot(dx, dy)

            ok = self.execute_pure_pursuit(
                target_id=target_id,
                direction=direction,
                total_distance=total_distance,
                start_id=current_id
            )

            if not ok:
                return False

            if self._is_temp_missing_tag(target_id):
                rospy.logwarn(
                    "[TemporaryMissingTag] skip align_to_tag(%s); tag is temporarily virtual",
                    target_id)
                return True

            # Preserve the old behaviour unless the calibrated predictive
            # approach was actually active for this segment. Without a
            # map_world entry, the same ROS1/Ubuntu 20.04 deployment should
            # behave like the pre-change controller.
            if (self._last_prediction_segment_active and
                    not self.post_move_align_enabled):
                return True

            if self._last_prediction_segment_active and target_id in self.detected_tags:
                tag = self.detected_tags[target_id]
                image_center_y = (self.camera_params[3]
                                  if self.camera_params is not None
                                  else self.image_center_y_fallback)
                center_y_diff = tag['center_y'] - image_center_y
                if abs(center_y_diff) > self.post_move_align_y_tolerance_px:
                    rospy.logwarn(
                        "[Navigation] skip post-align: y_err=%.1fpx would be worsened by in-place yaw",
                        center_y_diff)
                    return True

            return self.align_to_tag(target_id)

        else:
            rospy.logerr(f"Unknown edge type: {action_type}")
            return False

        
    def move_to_tag(self, target_id):
        """
        Move robot to target tag using MapManager.find_path(),
        executing each step via go_to_next_tag().
        """

        rospy.loginfo(f"[Robot] move_to_tag (path-based) → {target_id}")

        # ★ NEW: clear stop flag for new command
        self.clear_stop_flag()

        start_id = self.get_current_tag_id()

        if start_id is None:
            start_id = self.last_known_tag
            if start_id is not None:
                rospy.logwarn(
                    f"[Robot] No tag visible, fallback to last_known_tag={start_id}"
                )

        if start_id is None:
            rospy.logerr("[Robot] move_to_tag failed: cannot determine start tag")
            return False

        if start_id == target_id:
            rospy.loginfo(f"[Robot] Already at target tag {target_id}")
            self.last_known_tag = target_id
            self.publish_robot_pose(target_id)
            return True

        path = self.map_mgr.find_path(start_id, target_id)

        if not path or len(path) < 2:
            rospy.logerr(
                f"[Robot] No valid path from {start_id} to {target_id}"
            )
            return False

        rospy.loginfo(f"[Robot] Path: {path}")

        current_id = start_id

        for next_id in path[1:]:

            # ★ NEW: interrupt check
            if self.stop_requested:
                rospy.logwarn("[Robot] move_to_tag interrupted")
                return False

            rospy.loginfo(f"[Robot] Step {current_id} → {next_id}")

            ok = self.go_to_next_tag(
                target_id=next_id,
                known_start_id=current_id
            )

            if not ok:
                rospy.logerr(
                    f"[Robot] Failed at step {current_id} → {next_id}"
                )
                return False

            current_id = next_id
            self.last_known_tag = next_id

        rospy.loginfo(f"[Robot] move_to_tag success → {target_id}")
        self.publish_robot_pose(target_id)
        return True


    def execute_pivot(self, direction):
        target_angle = self.current_theta + (
            math.pi/2 if 'ccw' in direction else -math.pi/2
        )
        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            # ★ PREEMPT / STOP CHECK
            if self.stop_requested:
                rospy.logwarn("[Robot] pivot interrupted")
                self.stop()
                return False

            diff = target_angle - self.current_theta
            diff = math.atan2(math.sin(diff), math.cos(diff))

            if abs(diff) < math.radians(1.0):
                self.stop()
                rospy.loginfo(
                    f"Pivot Complete. Final error: {math.degrees(diff):.2f} deg"
                )
                return True

            vel = diff * self.cfg['robot'].get('pivot_gain', 1.5)
            vel = np.clip(vel, -self.max_angular, self.max_angular)

            self.send_vel(0, vel)
            rate.sleep()


    def _smooth_speed_factor(self, ratio, phase='accel'):
        """
        Converts linear ratio [0,1] to smooth speed factor.
        For acceleration: fast ramp up (sqrt curve)
        For deceleration: smooth ramp down (quadratic curve)
        """
        ratio = max(0.0, min(1.0, ratio))  # Clamp to [0, 1]

        if phase == 'accel':
            # Square root curve: ramps up quickly at start, then slows
            # ratio=0 -> 0, ratio=0.25 -> 0.5, ratio=1 -> 1
            return math.sqrt(ratio)
        else:
            # Quadratic curve for smooth deceleration
            # ratio=0 -> 0, ratio=1 -> 1, smooth end
            return ratio * ratio

    def _latch_final_approach(self, remaining_m, traveled_dist):
        """Enter the final approach and solve for the deceleration it needs.

        `final_approach_dist` is honoured whatever the entry speed, so the
        deceleration is derived rather than configured:

            a = (v_entry**2 - v_final**2) / (2 * remaining)

        which is the constant rate that passes through the current speed here
        and reaches `final_approach_speed` exactly at the stop point. It is
        allowed to exceed `linear_accel` — at the full 0.1 m/s over 6 cm it is
        0.0825, 1.65x the normal ramp. The alternative would be to trigger
        earlier at high speed, i.e. to not honour the configured distance.
        """
        self._final_approach = True
        self._fa_v0 = abs(self.current_linear_vel)
        self._fa_d0 = max(remaining_m, 1e-3)
        self._fa_s0 = traveled_dist

        vf = self.final_approach_speed
        # Exactly the solved rate — NOT max(linear_accel, needed). Using the
        # solved value makes the envelope pass through v_entry at this very
        # point, so the reference is continuous with the speed already being
        # commanded and there is no step for a limiter to smooth. Clamping it
        # up to linear_accel instead starts the curve ABOVE v_entry, which
        # forces a hold-then-brake with only millimetres of margin, and the
        # brake then needs exactly linear_accel — equal to the limiter, so
        # discretisation makes it lag and it lands short (measured: 0.015 m/s
        # at the stop instead of 0.010).
        needed = (self._fa_v0 ** 2 - vf ** 2) / (2.0 * self._fa_d0)
        self._fa_accel = max(0.0, needed)

        rospy.loginfo(
            f"[Navigation] Final approach: {remaining_m*100:.1f} cm to the stop "
            f"point at {self._fa_v0:.3f} m/s -> {vf:.3f} m/s, "
            f"decelerating at {self._fa_accel:.4f} m/s2"
            + (" (above linear_accel)" if needed > self.linear_accel else ""))

    def _final_approach_speed_at(self, traveled_dist):
        """Braking envelope for the final approach: sqrt(vf^2 + 2*a*remaining).

        Remaining distance is tracked by ODOM from the latch point, not
        re-measured from the tag every frame. Odom is continuous, so the curve
        does not jitter with detection noise and keeps working through a
        dropout; the robot still STOPS on the tag, so odom drift over 6 cm
        changes the speed profile slightly and the stop position not at all.
        """
        rem = max(0.0, self._fa_d0 - (traveled_dist - self._fa_s0))
        vf = self.final_approach_speed
        target = math.sqrt(vf * vf + 2.0 * self._fa_accel * rem)
        # Never exceed the speed we came in at — if the robot was already
        # slower than this curve, keep its speed rather than accelerating into
        # the tag.
        return min(target, self._fa_v0) if self._fa_v0 > 0 else target

    def execute_pure_pursuit(self, target_id, direction, total_distance=1.0,
                             start_id=None):
        """
        Moves towards target_id using Pure Pursuit logic.
        Speed is controlled purely by distance (odom-based), not tag detection.
        Tag detection is only used for steering and final stop condition.
        """

        
        rate = rospy.Rate(20)
        move_dir_sign = -1 if 'backward' in direction else 1

        # Load velocity profile parameters from config
        accel_dist = total_distance * self.cfg['robot'].get('s_curve_accel_ratio', 0.15)
        decel_dist = total_distance * self.cfg['robot'].get('s_curve_decel_ratio', 0.20)
        min_speed = self.max_linear * self.cfg['robot'].get('s_curve_min_speed_factor', 0.5)

        # Safety timeout
        timeout_limit = self.cfg['robot'].get('navigation_timeout', 8.0)
        # Also calculate based on distance at minimum speed
        worst_case_time = total_distance / min_speed if min_speed > 0 else 30.0
        timeout_limit = max(timeout_limit, worst_case_time * 1.5)
        start_time = rospy.Time.now()

        # Record starting odometry position
        start_odom_x = self.odom_x
        start_odom_y = self.odom_y
        start_theta = self.current_theta

        prediction_segment = self._make_prediction_segment(
            start_id=start_id,
            target_id=target_id,
            direction=direction,
            start_odom_x=start_odom_x,
            start_odom_y=start_odom_y,
            start_theta=start_theta,
            fallback_distance=total_distance)
        self._last_prediction_segment_active = prediction_segment is not None
        if prediction_segment is not None:
            rospy.loginfo(
                "[PredictiveCentering] %s -> %s using %s geometry, len=%.3fm",
                start_id, target_id, prediction_segment['source'],
                prediction_segment['seg_len'])

        # A new move must never inherit the previous one's latch. It is cleared
        # when the stop condition fires too; this covers the paths that do not
        # get there — a preempt, a timeout, or a move that ends blind.
        self._final_approach = False

        rospy.loginfo(f"[Navigation] Moving {total_distance:.2f}m to tag {target_id}, timeout: {timeout_limit:.1f}s")

        while not rospy.is_shutdown():
            # Check timeout
            if self.stop_requested:
                rospy.logwarn("[Robot] pure_pursuit interrupted")
                self.stop()
                return False
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout_limit:
                self.stop()
                rospy.logerr(f"[Watchdog] TIMEOUT ({elapsed:.1f}s). Target {target_id} not reached.")
                return False

            # Calculate distance traveled using odometry
            traveled_dist = math.hypot(self.odom_x - start_odom_x, self.odom_y - start_odom_y)
            remaining_dist = max(0.0, total_distance - traveled_dist)

            # ===== VELOCITY PROFILE (distance-based) =====
            # Acceleration phase: ramp up in first accel_dist
            if traveled_dist < accel_dist and accel_dist > 0:
                accel_factor = traveled_dist / accel_dist
                target_speed = min_speed + (self.max_linear - min_speed) * accel_factor
            # Deceleration phase: ramp down in last decel_dist
            elif remaining_dist < decel_dist and decel_dist > 0:
                decel_factor = remaining_dist / decel_dist
                target_speed = min_speed + (self.max_linear - min_speed) * decel_factor
            # Cruise phase: full speed
            else:
                target_speed = self.max_linear

            # Apply direction
            speed = target_speed * move_dir_sign

            # ===== STEERING (tag-based if visible) =====
            omega = 0.0
            tag_visible = target_id in self.detected_tags

            if tag_visible:
                tag = self.detected_tags[target_id]
                # image row + (down) = the robot's right, so `lateral` keeps the
                # same sign it had before front_cam was rotated on 2026-08-13.
                lateral = tag['y']
                heading_error_deg = tag_edge_angle_deg(tag['corners'])
                # Always the ~0.30 m camera height, never a range — the optical
                # axis pointed down before the rotation too, so the lookahead
                # below has always been pinned at look_ahead_base. Pre-existing;
                # tag['x'] is the true fore/aft distance if this is ever retuned.
                dist_to_tag = tag['z']

                # Check stop condition (tag center crosses image center).
                # The fore/aft axis is the image COLUMN since the rotation, and
                # it DECREASES on a forward approach (image right = forward), so
                # both comparisons below are inverted from the center_y version.
                center_x = tag['center_x']
                # Detections can arrive before CameraInfo does — they come
                # from different publishers now — so fall back to the frame's
                # geometric centre rather than indexing None. Only a stand-in:
                # the calibrated cx is usually a few px off centre.
                image_center_x = (self.camera_params[2]
                                  if self.camera_params is not None
                                  else self.image_center_x_fallback)
                image_center_y = (self.camera_params[3]
                                  if self.camera_params is not None
                                  else self.image_center_y_fallback)
                stop_offset = self.cfg['robot'].get('center_x_stop_offset', 0.0)
                stop_tolerance = self.cfg['robot'].get('center_x_stop_tolerance', 10.0)
                target_x = image_center_x + stop_offset
                diff = center_x - target_x
                center_y_diff = tag['center_y'] - image_center_y
                remaining_px_to_stop = diff * move_dir_sign - stop_tolerance

                if (prediction_segment is not None and
                        remaining_px_to_stop <= self.approach_slow_window_px and
                        (abs(center_y_diff) > self.approach_center_y_tolerance_px or
                         abs(heading_error_deg) > self.approach_yaw_tolerance_deg)):
                    slow_speed = max(0.0, self.approach_error_slow_speed)
                    if slow_speed > 0.0:
                        speed = math.copysign(min(abs(speed), slow_speed),
                                              speed)
                    rospy.loginfo_throttle(
                        0.5,
                        "[ApproachBalance] x_rem=%.1fpx y_err=%.1fpx yaw_err=%.2fdeg spd=%.3f",
                        remaining_px_to_stop, center_y_diff,
                        heading_error_deg, speed)

                should_stop = False
                if move_dir_sign > 0 and diff <= stop_tolerance:
                    should_stop = True
                elif move_dir_sign < 0 and diff >= -stop_tolerance:
                    should_stop = True

                if should_stop:
                    self._final_approach = False   # re-arm for the next move
                    self.stop()
                    rospy.loginfo(f"Arrived at {target_id} (traveled:{traveled_dist:.3f}m)")
                    return True

                # ===== FINAL APPROACH LATCH =====
                # `diff` is already the signed pixel distance still to go, so
                # converting it is exact against the stop test rather than a
                # second, independently-drifting estimate of where the tag is.
                # metres-per-pixel on the tag's plane is depth / fx; tag['z'] is
                # that depth (the ~0.30 m camera height, since the optical axis
                # points down) and camera_params[0] is fx.
                if not self._final_approach and self.camera_params is not None:
                    fx = self.camera_params[0]
                    if fx > 0:
                        # Measure to where the stop ACTUALLY fires, not to the
                        # nominal target_x: should_stop triggers a whole
                        # stop_tolerance early (10 px is ~3 mm at this depth).
                        # Aiming the envelope at target_x instead left the
                        # robot doing 0.024 m/s at the real stop point rather
                        # than final_approach_speed.
                        remaining_px = diff * move_dir_sign - stop_tolerance
                        remaining_m = max(0.0, remaining_px) * tag['z'] / fx
                        if remaining_m <= self.final_approach_dist:
                            self._latch_final_approach(remaining_m,
                                                       traveled_dist)
                elif self.camera_params is None:
                    # Startup transient only — CameraInfo and detections come
                    # from different publishers. Say so rather than silently
                    # driving the last 6 cm at profile speed.
                    rospy.logwarn_throttle(
                        2.0, "[Navigation] No CameraInfo yet — final approach "
                             "deceleration is inactive")

                # Pure Pursuit steering
                L = max(dist_to_tag, self.cfg['robot']['look_ahead_base'])
                alpha = math.atan2(-lateral, L)
                curvature = (2.0 * math.sin(alpha)) / L
                gain = self.cfg['robot']['pp_gain_backward'] if move_dir_sign < 0 else self.cfg['robot']['pp_gain_forward']
                omega = abs(speed) * curvature * gain
                # Only the calibrated approach changes visible steering. When
                # no map_world segment is available, leave the old Pure
                # Pursuit + post-align behaviour untouched.
                if prediction_segment is not None:
                    omega += -math.radians(heading_error_deg) * self.approach_heading_gain
                omega = np.clip(omega, -self.max_angular, self.max_angular)
            elif prediction_segment is not None:
                omega = self._predictive_centering_omega(
                    prediction_segment, speed)

            # Also check if we've traveled the expected distance (backup stop)
            if traveled_dist >= total_distance * 0.95:
                if self._is_temp_missing_tag(target_id):
                    arrival_ratio = max(0.0, min(1.0,
                                                 self.temp_missing_arrival_ratio))
                    if traveled_dist >= total_distance * arrival_ratio:
                        self._final_approach = False
                        self.stop()
                        rospy.logwarn(
                            "[TemporaryMissingTag] arrived at virtual tag %s by odom "
                            "(traveled:%.3fm / target:%.3fm)",
                            target_id, traveled_dist, total_distance)
                        return True
                # We've traveled most of the distance, slow down and look for tag
                if not tag_visible:
                    speed = min_speed * move_dir_sign
                    rospy.logwarn_throttle(0.5, f"Near target but tag not visible, slowing down")

            # ===== FINAL APPROACH — APPLIED LAST =====
            # Order matters and is the whole point of doing it here: both the
            # profile and the backup branch above write `speed` from
            # `min_speed` (= max_linear * s_curve_min_speed_factor = 0.02),
            # so a final-approach value set earlier would be silently
            # overwritten by a slower-but-larger number.
            #
            # The latch is deliberately NOT conditioned on tag_visible: a
            # dropped detection in the last few centimetres must not release
            # the robot back to profile speed.
            #
            # ⚠️ No general speed floor follows this. One was added and removed
            # on 2026-08-14 — see robot.yaml. Anything slower than this that
            # the profile produces is commanded as-is.
            accel_override = None
            if self._final_approach:
                speed = self._final_approach_speed_at(traveled_dist) * move_dir_sign
                # The envelope is already a constant-deceleration curve that
                # starts at the speed being commanded, so it is rate-bounded by
                # construction and there is nothing for the limiter to protect
                # against. Handing it the reference unfiltered is what makes the
                # deceleration exactly constant; a finite limit here can only
                # lag the curve, never improve it.
                accel_override = float('inf')

            # Performance logging
            tag_str = "TAG" if tag_visible else "BLIND"
            if self._final_approach:
                tag_str = "FINAL"
            rospy.loginfo_throttle(0.5, f"[{tag_str}] traveled:{traveled_dist:.3f} remain:{remaining_dist:.3f} spd:{speed:+.3f}")

            self.send_vel(speed, omega, linear_accel=accel_override)
            rate.sleep()

        return False
    
    def publish_robot_pose(self, tag_id):
        """
        Publish robot pose for manipulator coordinate transform.
        This replaces perform_scan_procedure's pose publishing role.
        """
        pose = self.calculate_robot_pose(tag_id)

        msg = Pose2DWithFlag()
        msg.header.stamp = rospy.Time.now()
        msg.flag = True
        msg.id = tag_id

        if pose is not None:
            msg.x, msg.y, msg.theta = pose
            rospy.loginfo(
                f"[RobotPose] tag {tag_id} "
                f"x={msg.x:.3f}, y={msg.y:.3f}, theta={msg.theta:.2f}"
            )
        else:
            rospy.logwarn(
                f"[RobotPose] Failed to compute pose for tag {tag_id}, using zeros"
            )
            msg.x = 0.0
            msg.y = 0.0
            msg.theta = 0.0

        self.pose_pub.publish(msg)
