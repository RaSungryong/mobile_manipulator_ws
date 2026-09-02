#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import glob
import os
import re
import tempfile
import time
import datetime as _dt
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
        self.move_max_angular = float(
            self.cfg['robot'].get('move_max_angular_speed',
                                  self.max_angular))
        self.align_max_angular = float(
            self.cfg['robot'].get('align_max_angular_speed',
                                  self.move_max_angular))
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
        # The base keeps rolling for ~stop_latency_s after the stop command
        # (measured 2026-09-02: 5.8 mm at 0.011 m/s forward, 12.6 mm at
        # 0.022 m/s reverse — proportional to speed). The stop is therefore
        # triggered `current speed * stop_latency_s` early, in the direction
        # of travel, so the base ENDS on the target column. 0 disables.
        self.stop_latency_s = max(0.0, float(
            self.cfg['robot'].get('stop_latency_s', 0.0)))
        self._vel_hist = []      # (t, |linear cmd|) for _pending_roll_m
        # Odom-based creep before the stop, independent of tag visibility.
        # In REVERSE the target tag is hidden behind the front bumper until
        # it is ~3 cm from the lens (measured 2026-09-02), so the tag-based
        # final approach has no room to slow down; odom remaining distance
        # was within 3 mm on the same hops, so it carries the pre-slowdown.
        self.blind_approach_dist = float(
            self.cfg['robot'].get('blind_approach_dist', 0.0))
        self.blind_approach_speed = float(
            self.cfg['robot'].get('blind_approach_speed', 0.015))
        # Profile speed multiplier while the target tag is visible (Pure
        # Pursuit phase). 1.0 = no change. See robot.yaml.
        self.tag_visible_speed_factor = max(0.0, min(1.0, float(
            self.cfg['robot'].get('tag_visible_speed_factor', 1.0))))
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
        # fallback_to_map_yaml used to gate map.yaml prediction when a
        # map_world file existed; since 2026-09-02 map.yaml is ALWAYS the
        # fallback, so the key no longer changes anything.
        self.pred_fallback_to_map_yaml = True
        self.pred_map_world_path = pred_cfg.get('map_world_path', 'latest')
        # Hops touching these tags run WITHOUT prediction (user request
        # 2026-09-02, for the zone A transit lane 400-499): tag visible ->
        # plain Pure Pursuit, tag not visible -> straight-line command
        # (omega 0), stop line -> align_to_tag. No map/odom steering, no
        # approach-heading blend, no ApproachBalance slowdown. A hop is
        # excluded when EITHER its start or its target tag matches.
        self.pred_disabled_ranges = []
        for rng in (pred_cfg.get('disabled_tag_ranges', []) or []):
            try:
                lo, hi = int(rng[0]), int(rng[1])
            except (TypeError, ValueError, IndexError):
                rospy.logwarn("[PredictiveCentering] bad disabled_tag_ranges entry %r", rng)
                continue
            self.pred_disabled_ranges.append((min(lo, hi), max(lo, hi)))
        self.pred_disabled_tag_ids = set(
            int(v) for v in (pred_cfg.get('disabled_tag_ids', []) or []))
        # Hops whose start AND target are both in these ranges end WITHOUT
        # the in-place align (user request 2026-09-02: 400->400 lane hops
        # are Pure Pursuit only; any hop with a 100- or 500-series endpoint
        # still aligns). Ranges only — a pivot never qualifies.
        self.align_skip_ranges = []
        for rng in (self.cfg['robot'].get('align_skip_tag_ranges', []) or []):
            try:
                lo, hi = int(rng[0]), int(rng[1])
            except (TypeError, ValueError, IndexError):
                rospy.logwarn("[Align] bad align_skip_tag_ranges entry %r", rng)
                continue
            self.align_skip_ranges.append((min(lo, hi), max(lo, hi)))
        self.pred_lookahead = float(
            pred_cfg.get('lookahead_m', self.cfg['robot']['look_ahead_base']))
        self.pred_min_lookahead = float(
            pred_cfg.get('min_lookahead_m',
                         min(0.20, self.cfg['robot']['look_ahead_base'])))
        self.pred_lookahead_segment_ratio = float(
            pred_cfg.get('lookahead_segment_ratio', 0.40))
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
        self.record_alignment_result = bool(
            pred_cfg.get('record_alignment_result', True))
        # One yaml PER TASK COMMAND under this directory (see
        # begin_nav_session); the old single alignment_result_path file,
        # which overwrote per tag, is gone.
        self.nav_log_dir = pred_cfg.get(
            'alignment_result_dir', '~/.ros/apriltag_nav/nav_log')
        self._nav_session = None        # dict being appended to
        self._nav_session_path = None
        self._nav_seq = 0
        self._last_prediction_segment_active = False
        self.pred_calibrated_xy = {}
        self.pred_loaded_map_world_path = None
        self.pred_map_world_available = False
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
        if not yaml_path:
            if self.pred_map_world_available:
                rospy.logwarn(
                    "[PredictiveCentering] map_world_*.yaml disappeared; using map.yaml geometry")
            self.pred_calibrated_xy = {}
            self.pred_loaded_map_world_path = None
            self.pred_map_world_available = False
            return
        if yaml_path != self.pred_loaded_map_world_path:
            self._load_calibrated_prediction_map(self.pred_map_world_path)

    def _load_calibrated_prediction_map(self, path):
        """Load path_tag_locator's map_world_*.yaml for relative tag geometry."""
        try:
            yaml_path = self._resolve_map_world_path(path)
            if not yaml_path:
                self.pred_map_world_available = False
                rospy.logwarn(
                    "[PredictiveCentering] no map_world_*.yaml found; using map.yaml geometry")
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
            self.pred_map_world_available = True
        except Exception as e:
            self.pred_calibrated_xy = {}
            self.pred_loaded_map_world_path = None
            self.pred_map_world_available = False
            rospy.logwarn(
                "[PredictiveCentering] failed to load map_world '%s': %s; using map.yaml geometry",
                path, e)

    def _prediction_xy(self, tag_id):
        """Return ((x, y), source) for blind steering.

        Calibrated map_world position when there is one, else map.yaml.
        A tag missing from both is the only way to get (None, None) — the
        blind segment must always have geometry to follow (2026-09-02).
        """
        if int(tag_id) in self.pred_calibrated_xy:
            return self.pred_calibrated_xy[int(tag_id)], 'map_world'
        xy = self._map_yaml_xy(tag_id)
        return (xy, 'map.yaml') if xy is not None else (None, None)

    def _map_yaml_xy(self, tag_id):
        info = self.map_mgr.get_tag_info(tag_id)
        if info is None or 'x' not in info or 'y' not in info:
            return None
        return (float(info['x']), float(info['y']))

    def _prediction_disabled_for(self, tag_id):
        """True when this tag is in predictive_centering.disabled_tag_ranges /
        disabled_tag_ids (the zone A 400-series lane by default)."""
        if tag_id is None:
            return False
        t = int(tag_id)
        if t in self.pred_disabled_tag_ids:
            return True
        return any(lo <= t <= hi for lo, hi in self.pred_disabled_ranges)

    def _make_prediction_segment(self, start_id, target_id, direction,
                                 start_odom_x, start_odom_y,
                                 start_theta, fallback_distance):
        if not self.pred_centering_enabled or start_id is None:
            return None
        if (self._prediction_disabled_for(start_id) or
                self._prediction_disabled_for(target_id)):
            rospy.loginfo(
                "[PredictiveCentering] OFF for %s -> %s (excluded tag range): "
                "tag visible -> Pure Pursuit, blind -> straight, then align",
                start_id, target_id)
            return None
        self._refresh_calibrated_prediction_map()
        start_xy, start_src = self._prediction_xy(start_id)
        target_xy, target_src = self._prediction_xy(target_id)
        if start_xy is None or target_xy is None:
            return None

        source = start_src
        if start_src != target_src:
            # Mixed sources would put the two endpoints in two different
            # frames; use map.yaml for both rather than steering blind.
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

        # Keep the blind controller tuned from the existing Pure Pursuit
        # values, but shorten preview distance on short tag-to-tag segments.
        # A 0.62 m pivot-exit segment should not use the full 0.4 m preview.
        dynamic_lookahead = segment['seg_len'] * self.pred_lookahead_segment_ratio
        lookahead = min(self.pred_lookahead, dynamic_lookahead)
        lookahead = max(self.pred_min_lookahead, lookahead, 0.05)
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
        return float(np.clip(omega, -self.move_max_angular,
                             self.move_max_angular))

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

    # ------------------------------------------------------------------
    # Tag-offset record: one yaml per TASK / GOTO command, appended to,
    # never overwritten.
    # ------------------------------------------------------------------
    NAV_CAM_FRAME_NOTE = (
        "front_cam optical frame after the 2026-08-13 -90 deg rotation: "
        "tag_pose_m = [fore_aft (+ ahead of the lens), lateral (+ robot's "
        "right / wall side), depth (~camera height)] of the tag centre "
        "relative to the camera centre; image_offset_px = tag centre minus "
        "image centre [column, row] (+col = ahead, +row = right); "
        "camera_center_offset_from_tag = where the camera centre sits "
        "relative to the tag, i.e. the negated tag_pose xy. Stage "
        "'arrival' = the pure-pursuit stop fired, 'aligned' = after "
        "align_to_tag trimmed the yaw."
    )

    @staticmethod
    def _sanitize_label(label):
        label = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(label).strip())
        return label.strip('_') or 'cmd'

    def begin_nav_session(self, label):
        """Open a NEW record file for one task command (TASK ... / GOTO n).

        Every tag-offset record taken until the next call lands in this
        file. Files never collide: <dir>/<YYYYMMDD>/<YYYYMMDD_HHMMSS>_<label>.yaml
        with a numeric suffix if the second is already taken.
        """
        if not self.record_alignment_result:
            return None
        try:
            now = _dt.datetime.now()
            day_dir = os.path.join(self._expand_path(self.nav_log_dir),
                                   now.strftime('%Y%m%d'))
            base = f"{now.strftime('%Y%m%d_%H%M%S')}_{self._sanitize_label(label)}"
            path = os.path.join(day_dir, base + '.yaml')
            n = 1
            while os.path.exists(path):
                n += 1
                path = os.path.join(day_dir, f"{base}_{n}.yaml")
            self._nav_session = {
                'command': str(label),
                'started_at': now.isoformat(timespec='milliseconds'),
                'camera_frame_note': self.NAV_CAM_FRAME_NOTE,
                'records': [],
            }
            self._nav_session_path = path
            self._nav_seq = 0
            self._atomic_write_yaml(self._nav_session, path)
            rospy.loginfo("[AlignmentRecord] new record file for '%s': %s",
                          label, path)
            return path
        except Exception as e:
            rospy.logwarn("[AlignmentRecord] could not open record file: %s", e)
            self._nav_session = None
            self._nav_session_path = None
            return None

    def _calibrated_reference(self, tag_id, tag_pose_xy_m):
        """World-frame estimate of the camera centre against the calibrated
        tag position, when a map_world_*.yaml exists. None otherwise —
        the camera-frame numbers do not depend on it."""
        map_world_path = self._resolve_map_world_path(self.pred_map_world_path)
        if not map_world_path or not os.path.exists(map_world_path):
            return None
        map_data = self._load_yaml_if_exists(map_world_path)
        map_entry = self._tag_entry_from_yaml(map_data, tag_id)
        if map_entry is None:
            return None
        calibrated_pos = map_entry.get('position_m')
        if calibrated_pos is None or len(calibrated_pos) < 3:
            return None
        calibrated_rpy = map_entry.get('rpy_deg', [0.0, 0.0, 0.0])
        camera_from_tag_m = [-tag_pose_xy_m[0], -tag_pose_xy_m[1]]
        yaw_rad = 0.0
        if calibrated_rpy is not None and len(calibrated_rpy) >= 3:
            yaw_rad = math.radians(float(calibrated_rpy[2]))
        cyaw, syaw = math.cos(yaw_rad), math.sin(yaw_rad)
        world_offset_m = [
            cyaw * camera_from_tag_m[0] - syaw * camera_from_tag_m[1],
            syaw * camera_from_tag_m[0] + cyaw * camera_from_tag_m[1],
            0.0,
        ]
        return {
            'source_map_world_path': map_world_path,
            'calibrated_tag_position_m': [float(v) for v in calibrated_pos[:3]],
            'calibrated_tag_rpy_deg': [float(v) for v in calibrated_rpy],
            'world_offset_from_calibrated_tag_m': world_offset_m,
            'world_offset_from_calibrated_tag_mm': [v * 1000.0 for v in world_offset_m],
            'estimated_camera_center_position_m': [
                float(calibrated_pos[i]) + world_offset_m[i] for i in range(3)],
        }

    def _record_tag_offset(self, tag_id, tag, stage, yaw_error_deg=None,
                           traveled_m=None, extra=None):
        """Append one camera-centre-vs-tag record to the current command's
        file. Always records the camera-frame numbers; the calibrated
        world-frame reference is added only when a map_world exists.
        Opens a 'goto_<tag>' file by itself if no command opened one."""
        if not self.record_alignment_result:
            return
        try:
            if self._nav_session is None:
                self.begin_nav_session(f"goto_{tag_id}")
                if self._nav_session is None:
                    return
            cx = (self.camera_params[2] if self.camera_params is not None
                  else self.image_center_x_fallback)
            cy = (self.camera_params[3] if self.camera_params is not None
                  else self.image_center_y_fallback)
            fx = self.camera_params[0] if self.camera_params is not None else 0.0
            fy = self.camera_params[1] if self.camera_params is not None else 0.0

            dx_px = float(tag['center_x'] - cx)
            dy_px = float(tag['center_y'] - cy)
            depth_m = float(tag.get('z', 0.0))
            fore_aft_m = float(tag.get('x', 0.0))
            lateral_m = float(tag.get('y', 0.0))
            cam_from_tag_m = [-fore_aft_m, -lateral_m]

            self._nav_seq += 1
            rec = {
                'seq': self._nav_seq,
                'stamp': float(time.time()),
                'ros_time': (float(rospy.Time.now().to_sec())
                             if rospy.core.is_initialized() else None),
                'stage': str(stage),
                'tag_id': int(tag_id),
                'image_center_px': [float(cx), float(cy)],
                'tag_center_px': [float(tag['center_x']), float(tag['center_y'])],
                'image_offset_px': [dx_px, dy_px],
                'tag_pose_m': [fore_aft_m, lateral_m, depth_m],
                'camera_center_offset_from_tag_m': cam_from_tag_m,
                'camera_center_offset_from_tag_mm': [v * 1000.0 for v in cam_from_tag_m],
                'camera_center_offset_from_tag_norm_mm': math.hypot(
                    cam_from_tag_m[0], cam_from_tag_m[1]) * 1000.0,
                'tag_edge_yaw_error_deg': (float(yaw_error_deg)
                                           if yaw_error_deg is not None else None),
                'traveled_m': (float(traveled_m) if traveled_m is not None else None),
                # Per-stage extras (e.g. the arrival's commanded speed and
                # stop lead) — used to fit stop_latency_s from the roll.
                **(dict(extra) if extra else {}),
            }
            if fx > 0.0 and fy > 0.0 and depth_m > 0.0:
                dx_mm = dx_px * depth_m / fx * 1000.0
                dy_mm = dy_px * depth_m / fy * 1000.0
                rec['image_center_offset_mm'] = [round(dx_mm, 3), round(dy_mm, 3)]
                rec['image_center_offset_norm_mm'] = round(math.hypot(dx_mm, dy_mm), 3)
            try:
                ref = self._calibrated_reference(tag_id, [fore_aft_m, lateral_m])
            except Exception as e:
                rospy.logwarn_throttle(
                    10.0, "[AlignmentRecord] calibrated reference skipped: %s", e)
                ref = None
            rec['calibrated_reference'] = ref

            self._nav_session['records'].append(rec)
            self._atomic_write_yaml(self._nav_session, self._nav_session_path)
            rospy.loginfo(
                "[AlignmentRecord] #%d %s tag %s: camera centre %.1f mm from tag "
                "(fore_aft %.1f, lateral %.1f) -> %s",
                self._nav_seq, stage, tag_id,
                rec['camera_center_offset_from_tag_norm_mm'],
                cam_from_tag_m[0] * 1000.0, cam_from_tag_m[1] * 1000.0,
                os.path.basename(self._nav_session_path))
        except Exception as e:
            rospy.logwarn(
                "[AlignmentRecord] failed to record tag %s (%s): %s",
                tag_id, stage, e)

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
        # Command history for the stop-roll lead: the base executes what was
        # commanded ~stop_latency_s ago, so the distance it will STILL travel
        # after a stop is the integral of the commands sent in that window.
        now = rospy.Time.now().to_sec()
        self._vel_hist.append((now, abs(float(linear))))
        cutoff = now - max(self.stop_latency_s, 0.0) - 1.0
        while self._vel_hist and self._vel_hist[0][0] < cutoff:
            self._vel_hist.pop(0)

    def _pending_roll_m(self):
        """Distance the base will still travel after a stop command NOW,
        under a pure transport-delay model: the commands of the last
        `stop_latency_s` seconds have not been executed yet, so integrate
        them. Falls back to |current cmd| x latency with no history."""
        lat = self.stop_latency_s
        if lat <= 0:
            return 0.0
        now = rospy.Time.now().to_sec()
        hist = [(t, v) for t, v in self._vel_hist if t >= now - lat]
        if len(hist) < 2:
            return abs(self.current_linear_vel) * lat
        dist = 0.0
        # Each sample holds until the next one; the last holds to `now`.
        for (t0, v0), (t1, _) in zip(hist, hist[1:] + [(now, 0.0)]):
            dist += v0 * max(0.0, t1 - t0)
        # The window may start before the oldest sample; extend the first.
        if hist[0][0] > now - lat:
            dist += hist[0][1] * (hist[0][0] - (now - lat))
        return dist

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
        """Rotate in place until the tag's edge reads square (yaw only).

        Runs after EVERY hop — forward, backward and pivot — once the base
        has stopped (2026-09-02). Bounded by `align_timeout_s`: the tag can
        be out of frame after a pivot (odometry-only turn) or an arrival at
        the frame edge, and before this the loop waited forever with the
        600 s MobileClient timeout as the only exit. <= 0 disables the bound.
        """
        rospy.loginfo(f"Aligning to tag {tag_id}...")
        rate = rospy.Rate(10)

        align_gain = self.cfg['robot'].get('align_gain', 0.8)
        align_threshold = self.cfg['robot'].get('align_threshold_deg', 0.5)
        align_timeout = float(self.cfg['robot'].get('align_timeout_s', 20.0))
        # Floor on |angular| so the P term cannot fall below a speed the base
        # executes (0.8 * 0.2 deg = 0.0028 rad/s would stall). Never above
        # the align clamp. See robot.yaml align_min_angular_speed.
        align_min_angular = min(
            float(self.cfg['robot'].get('align_min_angular_speed', 0.0)),
            self.align_max_angular)
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():

            # ★ PREEMPT / STOP CHECK
            if self.stop_requested:
                rospy.logwarn("[Robot] align_to_tag interrupted")
                self.stop()
                return False

            if align_timeout > 0:
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > align_timeout:
                    self.stop()
                    visible = tag_id in self.detected_tags
                    rospy.logerr(
                        f"[Robot] align_to_tag({tag_id}) TIMEOUT after "
                        f"{elapsed:.1f}s ("
                        + ("tag visible but not converging"
                           if visible else "tag not visible") + ")")
                    return False

            if tag_id not in self.detected_tags:
                self.stop()
                rospy.logwarn_throttle(
                    2.0, f"[Robot] align_to_tag: tag {tag_id} not visible, waiting")
                rate.sleep()
                continue

            angle_deg = tag_edge_angle_deg(self.detected_tags[tag_id]['corners'])

            if abs(angle_deg) < align_threshold:
                tag = dict(self.detected_tags[tag_id])
                self.stop()
                rospy.loginfo(f"Alignment Complete. Final Angle: {angle_deg:.2f}")
                self._record_tag_offset(tag_id, tag, 'aligned',
                                        yaw_error_deg=angle_deg)
                return True

            angular_vel = -math.radians(angle_deg) * align_gain
            if align_min_angular > 0 and abs(angular_vel) < align_min_angular:
                angular_vel = math.copysign(align_min_angular, angular_vel)
            angular_vel = np.clip(angular_vel, -self.align_max_angular,
                                  self.align_max_angular)

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
            # Odometry-only quarter turn, then square up to the EXIT tag the
            # front_cam should now be over (505 after 501 etc.) — the same
            # stop -> align_to_tag tail every move hop has (2026-09-02).
            if not self.execute_pivot(direction):
                return False
            return self._align_after_arrival(target_id, current_id)

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

            # The stop has already fired inside execute_pure_pursuit (the tag
            # crossed the stop line). ALWAYS trim the yaw to the tag now —
            # every hop ends squared up to its tag, whatever steered it there
            # (2026-09-02; the two conditional skips that lived here are gone).
            return self._align_after_arrival(target_id, current_id)

        else:
            rospy.logerr(f"Unknown edge type: {action_type}")
            return False

    def _in_align_skip_range(self, tag_id):
        if tag_id is None:
            return False
        t = int(tag_id)
        return any(lo <= t <= hi for lo, hi in self.align_skip_ranges)

    def _align_after_arrival(self, target_id, start_id=None):
        """The one tail every hop ends with: base already stopped, now
        rotate in place until the target tag reads square. Forward, backward
        and pivot hops all come through here (2026-09-02). Two skips: a
        temporarily-missing VIRTUAL tag (nothing to align to), and a hop
        whose BOTH endpoints are in `align_skip_tag_ranges` (the 400-series
        lane: Pure Pursuit only, no in-place align between lane tags; the
        moment a 100-/500-series tag is either endpoint, align runs)."""
        if self._is_temp_missing_tag(target_id):
            rospy.logwarn(
                "[TemporaryMissingTag] skip align_to_tag(%s); tag is temporarily virtual",
                target_id)
            return True
        if (self._in_align_skip_range(start_id) and
                self._in_align_skip_range(target_id)):
            rospy.loginfo(
                "[Align] %s -> %s: both in align_skip_tag_ranges, no in-place align",
                start_id, target_id)
            return True
        return self.align_to_tag(target_id)

        
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

            # ===== STEERING (tag-based if visible) =====
            omega = 0.0
            tag_visible = target_id in self.detected_tags

            # Once the target tag is in view the profile speed is scaled down
            # (user request 2026-09-02: "너무 빨라" — 1/3 of the driving speed
            # while Pure Pursuit is steering on the tag). Applied to the
            # profile only; the final-approach envelope below still wins in
            # the last few cm and starts from whatever speed this leaves.
            if tag_visible:
                target_speed *= self.tag_visible_speed_factor

            # Apply direction
            speed = target_speed * move_dir_sign

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
                # ONE stop column for both directions (user decision
                # 2026-09-02: "후진에서 기존에 만들어놓았던 태그 목표 중심에서
                # 정지해야돼"). The tag is brought to the same target column
                # whether the robot arrives driving forward or in reverse, so
                # the physical stop spot over the tag is the same either way;
                # only the approach side differs. A direction-mirrored
                # variant (cx ± offset) was tried and backed out the same day.
                target_x = image_center_x + stop_offset
                diff = center_x - target_x
                center_y_diff = tag['center_y'] - image_center_y
                # Roll compensation: fire the stop early by the distance the
                # base will still travel after the command (speed x latency),
                # converted to px on the tag plane (depth / fx). Uses the
                # ramped commanded speed, which is what the base is doing.
                # Direction-aware by construction — it is a lead along the
                # direction of travel, the target column itself is unchanged.
                lead_px = 0.0
                lead_m = 0.0
                if (self.stop_latency_s > 0 and self.camera_params is not None
                        and self.camera_params[0] > 0):
                    lead_m = self._pending_roll_m()
                    lead_px = lead_m * self.camera_params[0] / max(tag['z'], 1e-3)
                # The tolerance is a FLOOR on how early the stop fires; when
                # the roll lead is larger it replaces it, so the base ends on
                # the target column rather than `tolerance` short of it.
                fire_px = max(stop_tolerance, lead_px)
                remaining_px_to_stop = diff * move_dir_sign - fire_px

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

                should_stop = diff * move_dir_sign <= fire_px

                if should_stop:
                    self._final_approach = False   # re-arm for the next move
                    v_at_stop = abs(self.current_linear_vel)
                    self.stop()
                    rospy.loginfo(
                        f"Arrived at {target_id} (traveled:{traveled_dist:.3f}m, "
                        f"cmd {v_at_stop:.4f} m/s, lead {lead_px:.1f}px)")
                    self._record_tag_offset(
                        target_id, dict(tag), 'arrival',
                        yaw_error_deg=heading_error_deg,
                        traveled_m=traveled_dist,
                        extra={'commanded_speed_at_stop_mps': round(v_at_stop, 5),
                               'stop_lead_px': round(lead_px, 2),
                               'stop_lead_mm': round(lead_m * 1000.0, 2),
                               'stop_latency_s': self.stop_latency_s})
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
                        remaining_px = diff * move_dir_sign - fire_px
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
                omega = np.clip(omega, -self.move_max_angular,
                                self.move_max_angular)
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
            # ===== ODOM CREEP ZONE (before the final approach) =====
            # Cap the speed inside the last `blind_approach_dist` metres of
            # the ODOM distance, whether or not the tag is visible. This is
            # what lets a reverse hop arrive at final_approach_speed: the tag
            # only appears ~3 cm out there (bumper occlusion), far too late
            # for the tag-based envelope alone. Placed after the backup
            # branch above, which writes min_speed, and before the final
            # approach, which still wins.
            if (self.blind_approach_dist > 0 and
                    remaining_dist <= self.blind_approach_dist and
                    abs(speed) > self.blind_approach_speed):
                speed = math.copysign(self.blind_approach_speed, speed)

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
