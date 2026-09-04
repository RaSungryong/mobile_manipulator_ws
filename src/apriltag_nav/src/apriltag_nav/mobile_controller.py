#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
import collections
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
        self.odom_stamp = None          # rospy.Time of the last /odom
        self.odom_v = None              # signed forward speed from /odom twist
        self._tag_hist = {}             # tag id -> deque of recent detections
        self.stop_measure_frames = max(1, int(
            self.cfg['robot'].get('stop_measure_frames', 3)))
        self.camera_latency_compensation = bool(
            self.cfg['robot'].get('camera_latency_compensation', True))
        # Steering law while the target tag is in view (2026-09-04):
        # 'state_feedback' = omega = -(v * k_y) * e_y - k_theta * e_theta on
        # the BASE-referenced lateral and the PREDICTED heading (current +
        # rotation still pending from the command delay); 'pure_pursuit' =
        # the old point tracker (omega ~ speed * curvature, blind to yaw).
        self.steer_mode = str(self.cfg['robot'].get('steer_mode', 'state_feedback')).lower()
        self.sf_lateral_gain = float(self.cfg['robot'].get('sf_lateral_gain', 32.0))
        self.sf_heading_gain = float(self.cfg['robot'].get('sf_heading_gain', 0.6))
        self.sf_min_speed_for_gain = float(
            self.cfg['robot'].get('sf_min_speed_for_gain', 0.02))
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
        self._vel_hist = []      # (t, |linear cmd|, angular cmd) for _pending_roll_m / _pending_yaw_rad
        # Odom-based creep before the stop, independent of tag visibility.
        # In REVERSE the target tag is hidden behind the front bumper until
        # it is ~3 cm from the lens (measured 2026-09-02), so the tag-based
        # final approach has no room to slow down; odom remaining distance
        # was within 3 mm on the same hops, so it carries the pre-slowdown.
        self.blind_approach_dist = float(
            self.cfg['robot'].get('blind_approach_dist', 0.0))
        self.blind_approach_speed = float(
            self.cfg['robot'].get('blind_approach_speed', 0.015))
        # Stop target column, px from the calibrated cx. Forward hops stop
        # the tag on `center_x_stop_offset` (0 = the optical axis). Reverse
        # hops stop it on `center_x_stop_offset_reverse` when that key is
        # present (user request 2026-09-04: in reverse the tag must end as
        # far RIGHT in the image as possible = as far AHEAD of the lens as
        # the FOV allows); absent/null falls back to the forward column, the
        # 2026-09-02 one-column behaviour. robot_camera_node draws both
        # columns on /front_cam/tag_overlay from the same keys.
        self.center_x_stop_offset = float(
            self.cfg['robot'].get('center_x_stop_offset', 0.0) or 0.0)
        rev = self.cfg['robot'].get('center_x_stop_offset_reverse', None)
        self.center_x_stop_offset_reverse = (
            float(rev) if rev is not None else self.center_x_stop_offset)
        # Target tags for which a REVERSE hop still stops on the FORWARD
        # column (user rule 2026-09-04: "500번대 태그가 목표면 기존 목표
        # 위치에 정지"). Same [[lo, hi], ...] shape as align_skip_tag_ranges.
        self.reverse_stop_fwd_column_ranges = []
        # 2026-09-04: the exception now applies to BOTH directions (forward
        # got a far column too), so the key is `stop_offset_skip_tag_ranges`;
        # the old reverse-only name is read as an alias.
        for rng in (self.cfg['robot'].get('stop_offset_skip_tag_ranges',
                    self.cfg['robot'].get(
                        'center_x_stop_offset_reverse_skip_tag_ranges', [])) or []):
            try:
                lo, hi = int(rng[0]), int(rng[1])
            except (TypeError, ValueError, IndexError):
                rospy.logwarn("[Navigation] bad "
                              "center_x_stop_offset_reverse_skip_tag_ranges "
                              "entry %r", rng)
                continue
            self.reverse_stop_fwd_column_ranges.append((min(lo, hi), max(lo, hi)))
        # Where the lens came to rest relative to the LAST tag it stopped on:
        # fore-aft metres, + = tag ahead of the lens (tag['x']). With a
        # reverse column of +400 px the lens rests ~0.16 m short of the tag
        # after a reverse hop, and the map edge length — measured stop pose
        # to stop pose — is then wrong by that much for the NEXT hop's odom
        # profile (see _odom_distance_for_hop). Updated at every arrival /
        # align; reset by the manual moves, which leave it unknown.
        self._arrival_fore_m = 0.0
        self._last_tag_depth_m = None
        # Profile speed multiplier while the target tag is visible (Pure
        # Pursuit phase). 1.0 = no change. See robot.yaml.
        self.tag_visible_speed_factor = max(0.0, min(1.0, float(
            self.cfg['robot'].get('tag_visible_speed_factor', 1.0))))
        # Manual relative moves (drive_distance / pivot_angle) — odometry-
        # closed, no tag involved. See robot.yaml `manual_move:`.
        mm_cfg = self.cfg['robot'].get('manual_move', {}) or {}
        self.manual_arrive_tol_m = float(mm_cfg.get('arrive_tol_m', 0.003))
        self.manual_pivot_tol_rad = math.radians(float(
            mm_cfg.get('pivot_tol_deg',
                       self.cfg['robot'].get('pivot_threshold_deg', 1.0))))
        self.manual_stall_grace_s = float(mm_cfg.get('stall_grace_s', 2.5))
        self.manual_stall_eps_m = float(mm_cfg.get('stall_eps_m', 0.005))
        self.manual_stall_eps_rad = math.radians(float(
            mm_cfg.get('stall_eps_deg', 0.5)))
        self.manual_odom_stale_s = float(mm_cfg.get('odom_stale_s', 1.0))
        self.manual_ramp_out_max_s = float(mm_cfg.get('ramp_out_max_s', 5.0))
        self.manual_shortfall_frac = float(mm_cfg.get('shortfall_frac', 0.05))
        self.manual_max_distance_m = float(mm_cfg.get('max_distance_m', 5.0))
        self.manual_max_angle_deg = float(mm_cfg.get('max_angle_deg', 360.0))
        self.manual_default_linear = float(
            mm_cfg.get('default_linear_speed', 0.0)) or self.max_linear
        self.manual_default_angular = float(
            mm_cfg.get('default_angular_speed', 0.0)) or self.max_angular

        # Final approach: distance to the stop point at which we drop to
        # final_approach_speed and latch there until the stop condition fires.
        self.final_approach_dist = float(
            self.cfg['robot'].get('final_approach_dist', 0.06))
        # Launch yaw hold (2026-09-04, user: after the in-place align the
        # base sets off with a slight yaw — one wheel starts late). Pure
        # Pursuit's omega is |speed| x curvature, i.e. ~0 at launch and
        # blind to yaw, so a launch yaw error is only corrected once it has
        # become a lateral error. Inside the first `launch_yaw_hold_dist` m
        # of a hop the heading is held: to the tag's edge angle while the
        # tag is in view, else to the odom yaw at the hop start (the
        # encoders see a late wheel as a differential). 0 disables.
        self.launch_yaw_hold_dist = float(
            self.cfg['robot'].get('launch_yaw_hold_dist', 0.0))
        self.launch_yaw_gain = float(
            self.cfg['robot'].get('launch_yaw_gain', 1.0))
        # Backlash feed-forward (2026-09-04, user: "얼라인 때 후진을 줬던
        # 바퀴가 출발 때 약간 반응이 늦다"). An in-place align turns one
        # wheel backward; on the next hop that wheel has to REVERSE, so it
        # sits idle through its gear play while the other wheel already
        # rolls — the base twists toward the late wheel, by play / track
        # (0.7 deg per 8 mm of play on the 0.65 m track), and a motor-side
        # encoder never sees it. The twist direction is known from the
        # align's last turn (CCW align -> left wheel was backing -> twist
        # is CCW on a forward hop, and ALSO CCW on a reverse hop, where the
        # right wheel is the one reversing), so the launch pre-rotates the
        # other way: omega_ff = -sign(last align turn) x ff_omega for ff_s.
        self.launch_backlash_ff_omega = float(
            self.cfg['robot'].get('launch_backlash_ff_omega', 0.0))
        self.launch_backlash_ff_s = float(
            self.cfg['robot'].get('launch_backlash_ff_s', 0.3))
        self._last_align_turn_sign = 0       # +1 CCW, -1 CW, 0 unknown
        self.camera_offset = float(self.cfg['robot'].get('camera_offset', 0.55))
        self.robot_pose_use_fore_aft = bool(
            self.cfg['robot'].get('robot_pose_use_fore_aft', True))
        self.pp_lateral_reference = str(
            self.cfg['robot'].get('pp_lateral_reference', 'base')).lower()
        self._align_pulse_gain = 1.0         # achieved / commanded rotation, learned
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
        # ⚠️ There is deliberately NO per-tag align skip any more (user rule
        # 2026-09-04: "어떤 상황이든 정지해서 얼라인은 필수"). The
        # `align_skip_tag_ranges` key that let 400→400 lane hops skip the
        # in-place align (2026-09-02) is ignored if present.
        if self.cfg['robot'].get('align_skip_tag_ranges'):
            rospy.logwarn("[Align] robot.align_skip_tag_ranges is no longer "
                          "honoured — every hop ends with align_to_tag")
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

        # SIGNED speed (2026-09-04): this steers the BASE toward the lane
        # line, and the base's lateral motion is v*sin(yaw) — reversing, the
        # same yaw rate moves it the other way (a car backing up), so the
        # curvature term flips with the direction of travel. |speed| here
        # made the reverse blind lateral loop positive feedback; the heading
        # term hid it (a few mm of offset persisted instead of growing).
        omega = speed * curvature * gain + self.pred_heading_gain * heading_error
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
        """Start a NEW record for one task command (TASK ... / GOTO n).

        Every tag-offset record taken until the next call lands in ONE
        file: <dir>/<YYYYMMDD>/<YYYYMMDD_HHMMSS>_<label>.yaml (numeric
        suffix if that second is taken). The file is created on the FIRST
        record, not here (2026-09-04, user: one yaml per command): a
        command that is preempted, refused or fails before any arrival
        leaves no empty file behind — before this, every re-issued GOTO
        left a `records: []` twin next to the real one.
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
            rospy.loginfo("[AlignmentRecord] record for '%s' -> %s "
                          "(written at the first arrival)", label, path)
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
            if len(self._nav_session['records']) == 1:
                # First record creates the file. Another process cannot
                # have taken the name (one mobile_node), but a session that
                # was begun long ago could collide with a later one of the
                # same second — re-check, same suffix rule as begin.
                path = self._nav_session_path
                n = 1
                while os.path.exists(path):
                    n += 1
                    path = self._nav_session_path[:-5] + f"_{n}.yaml"
                self._nav_session_path = path
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
        self.odom_stamp = rospy.Time.now()
        try:
            self.odom_v = float(msg.twist.twist.linear.x)
        except Exception:
            self.odom_v = None
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
            try:
                stamp = float(msg.header.stamp.to_sec()) or float(rospy.Time.now().to_sec())
            except Exception:
                stamp = float(rospy.Time.now().to_sec())
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
            self._store_detections(new_tags, stamp)
            self.image_center_x_fallback = msg.image_width / 2.0
            self.image_center_y_fallback = msg.image_height / 2.0

            if self.detected_tags:
                rospy.loginfo_throttle(2.0, f"[Vision] Tags detected: {list(self.detected_tags.keys())}")
            else:
                rospy.logdebug_throttle(2.0, "[Vision] No tags detected.")

            self.vision_stop_callback(msg)

        except Exception as e:
            rospy.logerr(f"Detection processing error: {e}")


    def _store_detections(self, new_tags, stamp):
        """Publish `new_tags` as the current detections and append each
        to a short per-tag history (for the median filter). `stamp` is the
        source IMAGE time, kept per tag for latency compensation."""
        for tid, t in new_tags.items():
            t['stamp'] = float(stamp)
            t['edge_deg'] = tag_edge_angle_deg(t['corners'])
            hist = self._tag_hist.get(tid)
            if hist is None or hist.maxlen != self.stop_measure_frames:
                hist = collections.deque(maxlen=self.stop_measure_frames)
                self._tag_hist[tid] = hist
            hist.append(t)
        self.detected_tags = new_tags

    def _executed_linear_vel(self):
        """Signed forward speed the base is executing NOW: odom twist when
        fresh, else the command sent stop_latency_s ago (transport-delay
        model), else the current ramped command."""
        now = rospy.Time.now().to_sec()
        if (self.odom_v is not None and self.odom_stamp is not None
                and now - self.odom_stamp.to_sec() < 0.3):
            return float(self.odom_v)
        if self._vel_hist:
            t_exec = now - self.stop_latency_s
            v = None
            for h in self._vel_hist:
                if h[0] <= t_exec:
                    v = h[3]
                else:
                    break
            if v is not None:
                return float(v)
        return float(self.current_linear_vel)

    def _tag_view(self, tag_id):
        """The measurement the control loop should act on for `tag_id`:
        the MEDIAN of the last stop_measure_frames detections (one bad
        frame cannot fire the stop or the align), extrapolated to NOW along
        the fore-aft axis by the executed speed x the frame's age
        (camera_latency_compensation — the detector's frame is ~0.1 s old,
        i.e. 3-4 mm at 0.033 m/s). Returns None if the tag is not in the
        current frame. Keys as detected_tags plus 'age_s' and 'comp_px'."""
        cur = self.detected_tags.get(tag_id)
        if cur is None:
            return None
        hist = self._tag_hist.get(tag_id)
        now = rospy.Time.now().to_sec()
        if hist and len(hist) >= 3:
            recent = [h for h in hist if now - h['stamp'] <= 0.35]
            # A median needs an odd count of at least 3 to reject one bad
            # sample; with two, numpy's median is their MEAN and a spike
            # leaks in at half strength — so below 3 the newest frame is
            # used unfiltered.
            if len(recent) >= 3:
                view = dict(cur)
                for k in ('center_x', 'center_y', 'x', 'y', 'z', 'edge_deg'):
                    view[k] = float(np.median([h[k] for h in recent if k in h]))
                view['corners'] = cur['corners']
            else:
                view = dict(cur)
        else:
            view = dict(cur)
        view['age_s'] = 0.0
        view['comp_px'] = 0.0
        if (self.camera_latency_compensation and self.camera_params is not None
                and view.get('stamp') is not None):
            age = min(max(now - view['stamp'], 0.0), 0.3)
            fx = self.camera_params[0]
            depth = view.get('z') or 0.30
            if age > 0.0 and fx > 0 and depth > 0:
                d_fore = -self._executed_linear_vel() * age   # robot moved on
                view['x'] = view['x'] + d_fore
                view['comp_px'] = d_fore * fx / depth
                view['center_x'] = view['center_x'] + view['comp_px']
                view['age_s'] = age
        return view

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
        self._vel_hist.append((now, abs(float(linear)), float(angular), float(linear)))
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
        return self._integrate_pending(1, abs(self.current_linear_vel))

    def _pending_yaw_rad(self):
        """Signed rotation (rad, + = CCW) the base will still make after a
        stop command NOW — the angular twin of _pending_roll_m, same
        transport-delay model. Skid steer makes this matter more than for
        the drive: an in-place turn is executed with the same ~0.55 s lag,
        so a P loop that stops the moment the error is inside the band ends
        up `pending` further round — measured as the align 'twisting' past
        the tag (user, 2026-09-04)."""
        return self._integrate_pending(2, self.current_angular_vel)

    def _integrate_pending(self, col, fallback_rate):
        """Integral of column `col` of the command history over the last
        stop_latency_s. Exact under the delay model: each sample holds
        until the next one; the value in force at the START of the window
        is the last sample published before it (not the first one inside
        it — that guess inflated the estimate at the start of a motion to
        a whole `rate * latency` after ONE tick, which made align stop
        after a single 0.057 deg step, 2026-09-04). No history at all ->
        `fallback_rate * latency`."""
        lat = self.stop_latency_s
        if lat <= 0:
            return 0.0
        now = rospy.Time.now().to_sec()
        t_win = now - lat
        if not self._vel_hist:
            return fallback_rate * lat
        prev = None
        hist = []
        for h in self._vel_hist:
            if h[0] < t_win:
                prev = (h[0], h[col])
            else:
                hist.append((h[0], h[col]))
        total = 0.0
        if prev is not None:
            total += prev[1] * ((hist[0][0] if hist else now) - t_win)
        for (t0, v0), (t1, _) in zip(hist, hist[1:] + [(now, 0.0)]):
            total += v0 * max(0.0, t1 - t0)
        return total

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
        """Square up to the tag (yaw only). Dispatches on robot.align_mode:
        'continuous' (default: the P loop with a delay lead, settle and
        re-measure below) or 'pulse' — see _align_to_tag_pulse."""
        mode = str(self.cfg['robot'].get('align_mode', 'continuous')).lower()
        if mode == 'pulse':
            return self._align_to_tag_pulse(tag_id)
        return self._align_to_tag_continuous(tag_id)

    def _measure_tag_angle_at_rest(self, tag_id, frames, rate):
        """Median edge angle over `frames` consecutive detections (rejects a
        single bad frame). Returns (angle_deg, tag_dict) or (None, None)
        if the tag is not seen for the whole window."""
        vals = []
        last = None
        misses = 0
        while len(vals) < frames and not rospy.is_shutdown():
            if self.stop_requested:
                return None, None
            if tag_id in self.detected_tags:
                last = dict(self.detected_tags[tag_id])
                vals.append(tag_edge_angle_deg(last['corners']))
                misses = 0
            else:
                misses += 1
                if misses > frames * 3:
                    return None, None
            rate.sleep()
        return float(np.median(vals)), last

    def _align_to_tag_pulse(self, tag_id):
        """Pulse alignment for a skid-steer base with a command delay,
        stiction and a release lurch (2026-09-04, user: the continuous
        loop still over-rotated on the robot).

        Every decision is taken with the base AT REST on a multi-frame
        median of the tag's edge angle:
          1. measure error e (deg); |e| < band -> done.
          2. command ONE pulse: omega = align_pulse_omega (well above
             stiction) for t = |rot| / omega, where rot = e *
             align_pulse_fraction / gain_est. The rotation is set by the
             pulse DURATION, so a transport delay only shifts it in time
             — it cannot add to it, unlike the continuous loop where the
             last 0.55 s of commands keep turning the base.
          3. stop, wait stop_latency_s + align_settle_s, re-measure.
          4. gain_est := achieved / commanded (smoothed, clamped) — a base
             that lurches 1.5x the command is corrected on the next pulse
             and remembered for the next hop (self._align_pulse_gain).
        The pulse aims at align_pulse_fraction (0.8) of the error on
        purpose: a small undershoot costs one more pulse in the SAME
        direction; an overshoot costs a reversal, which unloads a wheel
        (backlash) and is what the launch feed-forward then has to fight.
        """
        rospy.loginfo(f"Aligning to tag {tag_id} (pulse mode)...")
        rate = rospy.Rate(10)
        cfg = self.cfg['robot']
        band = float(cfg.get('align_threshold_deg', 0.5))
        timeout = float(cfg.get('align_timeout_s', 20.0))
        settle_s = self.stop_latency_s + float(cfg.get('align_settle_s', 0.3))
        max_pulses = max(1, int(cfg.get('align_max_pulses', 5)))
        frac = float(cfg.get('align_pulse_fraction', 0.8))
        omega_max = min(float(cfg.get('align_pulse_omega', 0.05)),
                        self.align_max_angular)
        omega_min = min(float(cfg.get('align_min_angular_speed', 0.01)), omega_max)
        min_pulse_s = float(cfg.get('align_pulse_min_s', 0.3))
        frames = max(1, int(cfg.get('align_measure_frames', 5)))
        gain = float(np.clip(self._align_pulse_gain, 0.3, 3.0))
        start = rospy.Time.now()
        pulses = 0
        history = []

        def timed_out():
            return timeout > 0 and (rospy.Time.now() - start).to_sec() > timeout

        while not rospy.is_shutdown():
            if self.stop_requested:
                rospy.logwarn("[Robot] align_to_tag interrupted")
                self.stop()
                return False
            if timed_out():
                self.stop()
                rospy.logerr(f"[Robot] align_to_tag({tag_id}) TIMEOUT after "
                             f"{timeout:.0f}s ({pulses} pulses, "
                             + ("tag visible" if tag_id in self.detected_tags
                                else "tag not visible") + ")")
                return False
            err, tag = self._measure_tag_angle_at_rest(tag_id, frames, rate)
            if err is None:
                if self.stop_requested:
                    self.stop()
                    return False
                rospy.logwarn_throttle(
                    2.0, f"[Robot] align_to_tag: tag {tag_id} not visible, waiting")
                continue
            if abs(err) < band or pulses >= max_pulses:
                self._arrival_fore_m = float(tag.get('x', 0.0))
                self._last_tag_depth_m = float(tag.get('z', 0.0)) or None
                self._align_pulse_gain = gain
                extra = {'align_passes': pulses, 'align_mode': 'pulse',
                         'align_pulse_gain': round(gain, 3),
                         'align_pulse_history': history}
                if abs(err) < band:
                    rospy.loginfo(f"Alignment Complete. Final Angle: {err:+.2f} "
                                  f"(at rest, {pulses} pulses, gain {gain:.2f})")
                else:
                    extra['align_residual_accepted'] = True
                    rospy.logwarn(f"[Robot] align_to_tag({tag_id}): {err:+.2f} deg "
                                  f"left after {pulses} pulses — accepting")
                self._record_tag_offset(tag_id, tag, 'aligned',
                                        yaw_error_deg=err, extra=extra)
                return True
            # ---- one pulse ----
            # The loop's sign convention: a positive edge angle is removed by
            # turning CW (negative omega); the edge angle moves with the yaw.
            rot_deg = -err * frac / gain          # rotation to command
            dur = abs(math.radians(rot_deg)) / omega_max
            omega = omega_max
            if dur < min_pulse_s:
                # too short to execute as a burst at full omega: stretch it
                # to the minimum duration at a lower omega, but never below
                # the speed the base is known to move at
                omega = max(omega_min, abs(math.radians(rot_deg)) / min_pulse_s)
                dur = abs(math.radians(rot_deg)) / omega
            omega = math.copysign(omega, rot_deg)
            self._last_align_turn_sign = 1 if omega > 0 else -1
            rospy.loginfo(f"[Robot] align pulse {pulses + 1}: error {err:+.2f} deg, "
                          f"rotate {rot_deg:+.2f} deg = {omega:+.3f} rad/s x {dur:.2f}s "
                          f"(gain {gain:.2f})")
            t_end = rospy.Time.now() + rospy.Duration(dur)
            while rospy.Time.now() < t_end and not rospy.is_shutdown():
                if self.stop_requested or timed_out():
                    self.stop()
                    return False
                self._publish_vel(0.0, omega)   # unramped: the burst IS the plan
                rate.sleep()
            self.stop()
            pulses += 1
            t_settle = rospy.Time.now() + rospy.Duration(settle_s)
            while rospy.Time.now() < t_settle and not rospy.is_shutdown():
                if self.stop_requested:
                    self.stop()
                    return False
                rate.sleep()
            err2, _ = self._measure_tag_angle_at_rest(tag_id, frames, rate)
            if err2 is not None:
                achieved = err2 - err          # edge angle moves with yaw
                history.append({'error_before': round(err, 3),
                                'commanded_deg': round(rot_deg, 3),
                                'achieved_deg': round(achieved, 3),
                                'omega': round(omega, 4), 'duration_s': round(dur, 3)})
                if abs(rot_deg) >= 0.15:
                    ratio = achieved / rot_deg
                    if 0.2 < ratio < 5.0:
                        gain = float(np.clip(0.5 * gain + 0.5 * gain * ratio, 0.3, 3.0))
                    else:
                        rospy.logwarn(f"[Robot] align pulse: commanded {rot_deg:+.2f} "
                                      f"deg, tag moved {achieved:+.2f} — ignored for gain")
                rospy.loginfo(f"[Robot] align pulse {pulses}: tag moved {achieved:+.2f} deg "
                              f"for {rot_deg:+.2f} commanded -> gain {gain:.2f}, "
                              f"error now {err2:+.2f}")
        self.stop()
        return False

    def _align_to_tag_continuous(self, tag_id):
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
        # Skid-steer overshoot handling (2026-09-04). The base keeps turning
        # for ~stop_latency_s after the command drops (same transport delay
        # as the linear stop), so (1) the stop fires when the PREDICTED
        # settled angle — current error minus the rotation still pending —
        # is inside the band, not when the raw error is; (2) the loop then
        # waits stop_latency_s + align_settle_s with zero command and
        # re-measures the tag at rest; (3) if the settled error is still
        # outside the band it runs another pass, up to align_max_passes.
        # Only a measurement taken AT REST counts as aligned.
        settle_s = self.stop_latency_s + float(
            self.cfg['robot'].get('align_settle_s', 0.3))
        max_passes = max(1, int(self.cfg['robot'].get('align_max_passes', 3)))
        lead_target = align_threshold * float(
            self.cfg['robot'].get('align_lead_target_ratio', 0.25))
        passes = 0
        settle_until = None      # rospy.Time while settling, else None
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

            if settle_until is not None:
                # Command is zero; let the delayed rotation play out before
                # judging the result.
                if rospy.Time.now() < settle_until:
                    rate.sleep()
                    continue
                settle_until = None
                if abs(angle_deg) < align_threshold:
                    tag = dict(self.detected_tags[tag_id])
                    # Base at rest now — the best estimate of where the
                    # lens sits relative to this tag for the next hop.
                    self._arrival_fore_m = float(tag.get('x', 0.0))
                    self._last_tag_depth_m = float(tag.get('z', 0.0)) or None
                    rospy.loginfo(f"Alignment Complete. Final Angle: "
                                  f"{angle_deg:.2f} (pass {passes})")
                    self._record_tag_offset(tag_id, tag, 'aligned',
                                            yaw_error_deg=angle_deg,
                                            extra={'align_passes': passes})
                    return True
                if passes >= max_passes:
                    tag = dict(self.detected_tags[tag_id])
                    self._arrival_fore_m = float(tag.get('x', 0.0))
                    self._last_tag_depth_m = float(tag.get('z', 0.0)) or None
                    rospy.logwarn(
                        f"[Robot] align_to_tag({tag_id}): {angle_deg:+.2f} deg "
                        f"left after {passes} passes (band {align_threshold}) "
                        "— accepting; check stop_latency_s / align_settle_s")
                    self._record_tag_offset(tag_id, tag, 'aligned',
                                            yaw_error_deg=angle_deg,
                                            extra={'align_passes': passes,
                                                   'align_residual_accepted': True})
                    return True
                rospy.loginfo(f"[Robot] align pass {passes} settled at "
                              f"{angle_deg:+.2f} deg, correcting again")
                # fall through: start the next pass from this measurement

            # Rotation still to come from the commands of the last
            # stop_latency_s. The loop below commands omega = -angle*gain,
            # i.e. a positive edge angle is removed by turning CW (negative
            # omega): the edge angle moves WITH the base's yaw, so the angle
            # the base will SETTLE at if we stop now is angle + pending.
            pending_deg = math.degrees(self._pending_yaw_rad())
            predicted_deg = angle_deg + pending_deg
            crossing = (angle_deg != 0.0 and
                        math.copysign(1.0, predicted_deg) != math.copysign(1.0, angle_deg))
            # Aim the settled angle at ZERO, not at the band edge: stopping
            # as soon as the prediction is inside the band leaves the base
            # parked at ~threshold on the near side. At the 0.01 rad/s floor
            # one 10 Hz tick is 0.057 deg of pending rotation, so a quarter
            # of the 0.2 deg band (0.05) is as fine as the loop can aim.
            if abs(predicted_deg) <= lead_target or crossing:
                self.stop()
                passes += 1
                settle_until = rospy.Time.now() + rospy.Duration(settle_s)
                rospy.loginfo(
                    f"[Robot] align stop: error {angle_deg:+.2f} deg, "
                    f"pending {pending_deg:+.2f} -> predicted {predicted_deg:+.2f}; "
                    f"settling {settle_s:.2f}s")
                rate.sleep()
                continue

            angular_vel = -math.radians(angle_deg) * align_gain
            if align_min_angular > 0 and abs(angular_vel) < align_min_angular:
                angular_vel = math.copysign(align_min_angular, angular_vel)
            angular_vel = np.clip(angular_vel, -self.align_max_angular,
                                  self.align_max_angular)

            if angular_vel != 0.0:
                self._last_align_turn_sign = 1 if angular_vel > 0 else -1
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
        # Fore-aft term (2026-09-04): the lens is no longer assumed to sit
        # over the tag — with the far stop columns it rests 12-16 cm short
        # of it. tag['x'] is the tag's distance AHEAD of the lens, so the
        # camera is that far behind the tag along the zone heading.
        fore = float(tag.get('x', 0.0)) if self.robot_pose_use_fore_aft else 0.0

        if zone == 'A' or zone == 'DOCK':
            robot_x, robot_y = tag_x - fore, tag_y + lateral
            heading = align_angle_deg
        elif zone in ['B', 'D']:
            robot_x, robot_y = tag_x - lateral, tag_y - fore
            heading = 90 + align_angle_deg
        elif zone in ['C', 'E']:
            robot_x, robot_y = tag_x + lateral, tag_y + fore
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

    def _stop_offset_px(self, direction, target_id):
        """The stop column (px from the calibrated cx) for this hop.
        Forward: center_x_stop_offset (+300 since 2026-09-04 — the tag is
        left AHEAD of the lens so the next launch still sees it, the
        condition that made reverse work well). Reverse:
        center_x_stop_offset_reverse (+400). A target in
        stop_offset_skip_tag_ranges (the 500-series dock / pivot tags)
        stops on the crosshair (offset 0) in BOTH directions: a pivot
        turns about the base centre and needs the base on the designed
        stop pose."""
        try:
            t = int(target_id)
        except (TypeError, ValueError):
            t = None
        if t is not None and any(lo <= t <= hi
                                 for lo, hi in self.reverse_stop_fwd_column_ranges):
            return 0.0
        if 'backward' not in direction:
            return self.center_x_stop_offset
        return self.center_x_stop_offset_reverse

    def _stop_target_fore_m(self, direction, target_id=None):
        """Fore-aft distance (m, + = tag ahead of the lens) the tag sits
        at when the stop column for this hop is reached: offset px on the
        tag plane = px * depth / fx. 0 when fx or a depth is not known yet."""
        off = self._stop_offset_px(direction, target_id)
        if off == 0.0 or self.camera_params is None or self.camera_params[0] <= 0:
            return 0.0
        depth = self._last_tag_depth_m if self._last_tag_depth_m else 0.30
        return off * depth / self.camera_params[0]

    def _odom_distance_for_hop(self, edge_dist, direction, start_id=None,
                               target_id=None):
        """Odom distance the profile should plan for on this hop.

        The map edge length is stop pose to stop pose (lens over the tag).
        The lens is actually `fore_prev` short of / past the start tag, and
        the stop column puts it `fore_target` short of the target tag, so
        the real travel is  edge + dir * (fore_prev - fore_target)  with
        dir = +1 forward / -1 reverse. With one shared column at 0 both
        terms vanish and this returns `edge_dist` unchanged. Without it a
        reverse hop to a +400 px column ran the last 0.16 m on the
        end-of-profile crawl and tripped the watchdog (offline plant,
        2026-09-04). `fore_prev` is read LIVE from the start tag when it is
        in view (it usually is — that is what the far-right column buys),
        else from the last arrival."""
        sign = -1.0 if 'backward' in direction else 1.0
        if start_id is not None and start_id in self.detected_tags:
            fore_prev = float(self.detected_tags[start_id].get('x', 0.0))
            src = 'live'
        else:
            fore_prev = float(self._arrival_fore_m)
            src = 'remembered'
        fore_target = self._stop_target_fore_m(direction, target_id)
        corr = sign * (fore_prev - fore_target)
        dist = edge_dist + corr
        if abs(corr) > 0.005:
            rospy.loginfo(
                "[Navigation] odom distance %.3f m = edge %.3f %+.3f "
                "(start tag %s fore %+.3f %s, stop column fore %+.3f)",
                dist, edge_dist, corr, start_id, fore_prev, src, fore_target)
        if dist < 0.05:
            rospy.logwarn(
                "[Navigation] corrected odom distance %.3f m is below 0.05 m "
                "(edge %.3f, corr %+.3f) — clamping", dist, edge_dist, corr)
            dist = 0.05
        return dist

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
            total_distance = self._odom_distance_for_hop(
                math.hypot(dx, dy), direction, current_id, target_id)

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

    def _align_after_arrival(self, target_id, start_id=None):
        """The one tail EVERY hop ends with: base already stopped, now
        rotate in place until the target tag reads square. Forward,
        backward and pivot hops all come through here, with no per-tag
        exception (user rule 2026-09-04: stop-then-align is mandatory on
        every drive; the 400-lane skip from 2026-09-02 is gone). A hop
        whose tag is not in view at rest fails here on `align_timeout_s`
        rather than being waved through. The ONLY remaining skip is a
        temporarily-missing VIRTUAL tag (`temporary_missing_tags`, off by
        default): there is physically nothing to align to."""
        if self._is_temp_missing_tag(target_id):
            rospy.logwarn(
                "[TemporaryMissingTag] skip align_to_tag(%s); tag is temporarily virtual",
                target_id)
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


    # ==========================================================
    # MANUAL RELATIVE MOVES — odometry-closed, no tag involved
    # ==========================================================
    # Ported from tools/vw_drive.py (2026-08-14) so the UI can drive the base
    # a given distance / angle THROUGH mobile_node instead of running a second
    # /cmd_vel publisher. The profile is the trapezoid that tool settled on
    # after the arrival judder: v = min(v_top, sqrt(2*a*remaining)) — a
    # braking curve that is self-consistent with a constant-accel ramp-out,
    # so the loop hands off when the COMMANDED speed's stopping distance
    # equals what is left and _manual_ramp_to_zero coasts the rest. No phase
    # ratios, no speed floor. Both methods return (ok, message).

    @staticmethod
    def _manual_envelope(remaining, top, accel):
        """Fastest speed from which a constant-`accel` ramp still stops on
        target: sqrt(2*a*remaining), clipped to `top`. The min produces the
        whole accel / cruise / brake trapezoid, and a triangle on a move too
        short to reach `top`."""
        return min(top, math.sqrt(max(0.0, 2.0 * accel * remaining)))

    @staticmethod
    def _manual_timeout_s(target, top, accel):
        """Generous bound on a trapezoid's duration. Bounds a hang, not a spec."""
        return 3.0 * (target / max(top, 1e-6) + top / max(accel, 1e-6)) + 5.0

    def _manual_blocked(self):
        """Reason the base must not be commanded right now, or None."""
        if self.stop_requested:
            return "stop requested"
        if self.odom_stamp is None:
            return "no /odom received yet"
        age = (rospy.Time.now() - self.odom_stamp).to_sec()
        if age > self.manual_odom_stale_s:
            return f"/odom is stale by {age:.1f}s — is the navifra driver up?"
        return None

    def _manual_begin(self, what):
        """Common entry: refuse under an EMERGENCY latch (a deliberate
        clear_stop is required, same rule as the calibration BaseInterface),
        clear a leftover preempt latch, and check odom is alive."""
        if self.emergency_stop:
            return (f"{what} refused — emergency stop is latched; "
                    "clear_stop first")
        # A preempt latch left by a previous cancel/task switch is not a
        # reason to refuse a fresh, explicit manual command.
        self.stop_requested = False
        self._final_approach = False
        # A manual move ends wherever it ends — the lens-vs-last-tag rest
        # position is no longer known.
        self._arrival_fore_m = 0.0
        reason = self._manual_blocked()
        if reason:
            return f"{what} refused — {reason}"
        return None

    def _manual_ramp_to_zero(self):
        """Bring the commanded velocity to zero at the accel limit, then
        stop(). NOT stop() alone — that is a step from the current command
        to zero, and the step is exactly the jolt the profile avoids."""
        rate = rospy.Rate(20)
        for _ in range(int(20 * self.manual_ramp_out_max_s)):
            if rospy.is_shutdown() or self.stop_requested:
                break
            if (abs(self.current_linear_vel) < 1e-3
                    and abs(self.current_angular_vel) < 1e-3):
                break
            self.send_vel(0.0, 0.0)
            rate.sleep()
        self.stop()

    def _manual_run_loop(self, progress, target, tol, eps, timeout_s,
                         command, braking, what):
        """Drive until braking now would land on target, then ramp out.

        `progress()` is the odom-measured amount done so far (m or rad),
        `command(remaining)` gives (v, w), `braking()` the distance the
        CURRENTLY COMMANDED velocity needs to reach zero at the accel limit
        PLUS the transport-delay lead (`stop_latency_s`, the same measured
        delay the tag stop compensates): under a pure delay the base still
        executes the last `stop_latency_s` of commands after the hand-off,
        so `|cmd| * latency` is added to the braking distance. Without it
        the base overshoots by exactly that (sim: +21 mm at 0.05 m/s,
        +48 mm at 0.1 m/s, +4 deg on a 90 deg pivot, at 0.55 s).
        Returns (ok, reason).
        """
        rate = rospy.Rate(20)
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        stall_ref = progress()
        stall_since = rospy.Time.now()

        while not rospy.is_shutdown():
            reason = self._manual_blocked()
            if reason:
                rospy.logerr(f"[Manual] {what} aborted — {reason}")
                self.stop()
                return False, f"{what} aborted — {reason}"

            done = progress()
            remaining = target - done
            # Leave when braking now lands on target, or on the tolerance —
            # never by waiting for the envelope to reach zero, which would
            # creep at a speed the drive may not execute and trip the stall
            # check.
            if remaining <= max(tol, braking()):
                break

            now = rospy.Time.now()
            if now > deadline:
                rospy.logerr(f"[Manual] {what} timed out at {done:.3f} of "
                             f"{target:.3f}")
                self.stop()
                return False, f"{what} timed out at {done:.3f} of {target:.3f}"

            # Stall check: commanding a base that is not moving is the
            # 2026-08-12 failure mode (frozen /odom, MOTOR_FEEDBACK_TIMEOUT).
            if abs(done - stall_ref) > eps:
                stall_ref = done
                stall_since = now
            elif (now - stall_since).to_sec() > self.manual_stall_grace_s:
                msg = (f"{what} aborted — /odom has not advanced in "
                       f"{self.manual_stall_grace_s:.1f}s while commanding "
                       "motion (check /motor/error, /motor/alarm)")
                rospy.logerr(f"[Manual] {msg}")
                self.stop()
                return False, msg

            v, w = command(remaining)
            self.send_vel(v, w)
            rate.sleep()

        self._manual_ramp_to_zero()
        # The base executes the last `stop_latency_s` of commands AFTER the
        # ramp-out has been published, so it is still coasting when this
        # returns unless we wait it out. Waiting also makes the arrival
        # verification below honest and means mobile_node reports
        # completion only once the base is physically at rest.
        if self.stop_latency_s > 0:
            rospy.sleep(rospy.Duration(self.stop_latency_s + 0.1))
        if self.stop_requested:
            return False, f"{what} aborted — stop requested during ramp-out"
        return True, "ok"

    def _manual_arrived(self, shortfall, target, floor_tol):
        """The stall check alone is not enough: on a base that is not moving
        the commanded speed keeps climbing until its stopping distance covers
        the whole move, which on a short move happens before the stall
        grace. Caught live on vw_drive: `forward 0.05` reported success
        having travelled 0.000 m."""
        return shortfall <= max(self.manual_shortfall_frac * target, floor_tol)

    def drive_distance(self, distance_m, speed=None):
        """Drive `distance_m` along body X (negative = reverse), closed on
        odometry. Returns (ok, message)."""
        distance_m = float(distance_m)
        target = abs(distance_m)
        what = f"drive {distance_m:+.3f} m"
        if target > self.manual_max_distance_m:
            return False, (f"{what} refused — over manual_move.max_distance_m "
                           f"({self.manual_max_distance_m:.2f} m)")
        why = self._manual_begin(what)
        if why:
            rospy.logwarn(f"[Manual] {why}")
            return False, why
        if target < self.manual_arrive_tol_m:
            return True, f"{what}: below arrival tolerance, nothing to do"

        sign = 1.0 if distance_m > 0 else -1.0
        v_top = min(abs(float(speed)) if speed else self.manual_default_linear,
                    self.max_linear)
        accel = self.linear_accel
        start = (self.odom_x, self.odom_y)

        def travelled():
            return math.hypot(self.odom_x - start[0], self.odom_y - start[1])

        rospy.loginfo(f"[Manual] {what} at up to {v_top:.3f} m/s")
        ok, reason = self._manual_run_loop(
            progress=travelled, target=target, tol=self.manual_arrive_tol_m,
            eps=self.manual_stall_eps_m,
            timeout_s=self._manual_timeout_s(target, v_top, accel),
            command=lambda rem: (
                sign * self._manual_envelope(rem, v_top, accel), 0.0),
            braking=lambda: (self.current_linear_vel ** 2 / (2.0 * accel)
                             + abs(self.current_linear_vel) * self.stop_latency_s),
            what=what)
        if not ok:
            return False, reason
        short = target - travelled()
        if not self._manual_arrived(short, target, 3 * self.manual_arrive_tol_m):
            msg = (f"{what} ended {short * 1000:.0f} mm short — the base did "
                   "not follow the command (check /motor/error, /motor/alarm, "
                   "and whether /odom is advancing)")
            rospy.logerr(f"[Manual] {msg}")
            return False, msg
        msg = f"{what} done: travelled {travelled():.3f} m"
        rospy.loginfo(f"[Manual] {msg}")
        return True, msg

    def pivot_angle(self, angle_deg, speed=None):
        """Pivot in place by `angle_deg` (positive = CCW / left), closed on
        odometry yaw. Returns (ok, message)."""
        angle_deg = float(angle_deg)
        target = abs(math.radians(angle_deg))
        what = f"pivot {angle_deg:+.1f} deg"
        if abs(angle_deg) > self.manual_max_angle_deg:
            return False, (f"{what} refused — over manual_move.max_angle_deg "
                           f"({self.manual_max_angle_deg:.0f} deg)")
        why = self._manual_begin(what)
        if why:
            rospy.logwarn(f"[Manual] {why}")
            return False, why
        if target < self.manual_pivot_tol_rad:
            return True, f"{what}: below pivot tolerance, nothing to do"

        sign = 1.0 if angle_deg > 0 else -1.0
        w_top = min(abs(float(speed)) if speed else self.manual_default_angular,
                    self.max_angular)
        accel = self.angular_accel
        turned = [0.0]
        last_yaw = [self.current_theta]

        def progress():
            # Integrate wrapped deltas rather than comparing against the
            # start: a pivot through +-180 would otherwise read as a jump
            # backwards.
            d = self.current_theta - last_yaw[0]
            turned[0] += abs(math.atan2(math.sin(d), math.cos(d)))
            last_yaw[0] = self.current_theta
            return turned[0]

        rospy.loginfo(f"[Manual] {what} at up to {w_top:.3f} rad/s")
        ok, reason = self._manual_run_loop(
            progress=progress, target=target, tol=self.manual_pivot_tol_rad,
            eps=self.manual_stall_eps_rad,
            timeout_s=self._manual_timeout_s(target, w_top, accel),
            command=lambda rem: (
                0.0, sign * self._manual_envelope(rem, w_top, accel)),
            braking=lambda: (self.current_angular_vel ** 2 / (2.0 * accel)
                             + abs(self.current_angular_vel) * self.stop_latency_s),
            what=what)
        if not ok:
            return False, reason
        # One more sample AFTER the ramp-out: the accumulator only advances
        # when called, and the ramp-out is real rotation the loop never saw.
        short = target - progress()
        if not self._manual_arrived(short, target, 3 * self.manual_pivot_tol_rad):
            msg = (f"{what} ended {math.degrees(short):.1f} deg short — the "
                   "base did not follow the command (check /motor/error, "
                   "/motor/alarm, and whether /odom is advancing)")
            rospy.logerr(f"[Manual] {msg}")
            return False, msg
        msg = (f"{what} done: turned {math.degrees(turned[0]):.2f} deg "
               f"(error {math.degrees(short):+.2f} deg)")
        rospy.loginfo(f"[Manual] {msg}")
        return True, msg

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
        launch_theta_ref = start_theta       # re-anchored to a tag when seen
        launch_peak_yaw_err = 0.0            # for the arrival record
        launch_ff_applied = False

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
            sf_heading_applied = False
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
                # Median over the last frames + latency extrapolation —
                # every decision below (steer, stop, final approach, the
                # arrival record) sees the same filtered measurement.
                tag = self._tag_view(target_id) or self.detected_tags[target_id]
                # image row + (down) = the robot's right, so `lateral` keeps the
                # same sign it had before front_cam was rotated on 2026-08-13.
                lateral = tag['y']
                heading_error_deg = tag.get('edge_deg',
                                            tag_edge_angle_deg(tag['corners']))
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
                stop_tolerance = self.cfg['robot'].get('center_x_stop_tolerance', 10.0)
                # The stop column depends on the direction of travel
                # (2026-09-04). Forward: cx + center_x_stop_offset, the tag
                # arriving from image-right. Reverse: cx +
                # center_x_stop_offset_reverse, the tag arriving from
                # image-left and carried on to the far right of the frame
                # (= ahead of the lens), so the physical stop spot in reverse
                # is that many px * depth / fx further along the reverse
                # direction than the forward spot. Both columns are drawn on
                # /front_cam/tag_overlay. (History: one shared column from
                # 2026-09-02, after a cx - offset mirror was backed out —
                # that variant stopped the tag on the LEFT; this one is the
                # opposite side and was asked for explicitly.) A reverse hop
                # whose TARGET is a 500-series tag keeps the forward column
                # (center_x_stop_offset_reverse_skip_tag_ranges).
                stop_offset = self._stop_offset_px(direction, target_id)
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
                    self._arrival_fore_m = float(tag.get('x', 0.0))
                    self._last_tag_depth_m = float(tag.get('z', 0.0)) or None
                    rospy.loginfo(
                        f"Arrived at {target_id} (traveled:{traveled_dist:.3f}m, "
                        f"cmd {v_at_stop:.4f} m/s, lead {lead_px:.1f}px)")
                    self._record_tag_offset(
                        target_id, dict(tag), 'arrival',
                        yaw_error_deg=heading_error_deg,
                        traveled_m=traveled_dist,
                        extra={'commanded_speed_at_stop_mps': round(v_at_stop, 5),
                               'direction': str(direction),
                               'stop_offset_px': round(float(stop_offset), 2),
                               'stop_target_x_px': round(float(target_x), 2),
                               'tag_offset_from_target_px': round(
                                   float(diff), 2),
                               'stop_lead_px': round(lead_px, 2),
                               'stop_lead_mm': round(lead_m * 1000.0, 2),
                               'stop_latency_s': self.stop_latency_s,
                               'launch_peak_yaw_err_deg': round(launch_peak_yaw_err, 3),
                               'tag_age_s': round(float(tag.get('age_s', 0.0)), 3),
                               'latency_comp_px': round(float(tag.get('comp_px', 0.0)), 2),
                               'steer_mode': self.steer_mode,
                               'launch_ff_applied': launch_ff_applied,
                               'last_align_turn_sign': self._last_align_turn_sign})
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

                # Pure Pursuit steering — on the BASE CENTRE's lateral offset,
                # not the lens's (2026-09-04). The lens is camera_offset
                # (0.55 m) ahead of the centre the align rotates about, so
                # steering the lens onto the tag leaves the base off the
                # line by 0.55*sin(yaw) and the align then swings the lens
                # that far back out (robot records: arrival lateral +1.5 mm,
                # yaw -1.66 deg -> after align +19.5 mm; predicted 17.4).
                # Aiming the base at the line means the align lands the lens
                # ON the tag. Sign checked against those records.
                if self.pp_lateral_reference == 'base':
                    lateral_pp = lateral - self.camera_offset * math.sin(
                        math.radians(heading_error_deg))
                    # The base-referenced error obeys e_y' = v sin(theta), so
                    # the lateral steering SIGN flips with the direction of
                    # travel — reversing, the base centre moves the other
                    # way for the same yaw (a car backing up). The
                    # lens-referenced error did not need this: the lens is
                    # 0.55 m ahead of the pivot, and that lever moved it
                    # toward the tag under a CW turn whichever way the base
                    # rolled. Missing this made the reverse lateral loop
                    # positive feedback (found 2026-09-04, sim-confirmed).
                    lat_dir = float(move_dir_sign)
                else:
                    lateral_pp = lateral
                    lat_dir = 1.0
                # Predicted heading (2026-09-04): the base still executes the
                # last stop_latency_s of angular commands, so steer on where
                # the heading WILL be, not where the delayed frame says it
                # is — the same lead the stop and the align use. The edge
                # angle moves with the yaw, so predicted = current + pending.
                pending_yaw_deg = math.degrees(self._pending_yaw_rad())
                heading_pred_deg = heading_error_deg + pending_yaw_deg
                sf_heading_applied = False
                if self.steer_mode == 'state_feedback':
                    # omega = -(v k_y) e_y - k_theta e_theta: the lateral term
                    # scales with speed like Pure Pursuit (kinematics: e_y' =
                    # v e_theta), the heading term does not — that is what
                    # gives it authority at the 0.01-0.03 m/s the last 25 cm
                    # are driven at. Closed-loop: wn = v sqrt(k_y),
                    # zeta = k_theta / (2 v sqrt(k_y)) -> 1.6 at 0.033 m/s,
                    # 0.53 at 0.1 m/s with the defaults 32 / 0.6.
                    v_gain = max(abs(speed), self.sf_min_speed_for_gain) * lat_dir
                    omega = (-(v_gain * self.sf_lateral_gain) * lateral_pp
                             - self.sf_heading_gain * math.radians(heading_pred_deg))
                    sf_heading_applied = True
                else:
                    L = max(dist_to_tag, self.cfg['robot']['look_ahead_base'])
                    alpha = math.atan2(-lateral_pp, L)
                    curvature = (2.0 * math.sin(alpha)) / L
                    gain = self.cfg['robot']['pp_gain_backward'] if move_dir_sign < 0 else self.cfg['robot']['pp_gain_forward']
                    omega = abs(speed) * curvature * gain * lat_dir
                    # Only the calibrated approach changes visible steering.
                    if prediction_segment is not None:
                        omega += -math.radians(heading_pred_deg) * self.approach_heading_gain
                        sf_heading_applied = True
                omega = np.clip(omega, -self.move_max_angular,
                                self.move_max_angular)
            elif prediction_segment is not None:
                omega = self._predictive_centering_omega(
                    prediction_segment, speed)

            # ===== LAUNCH YAW HOLD =====
            # Hold the heading the align left us with through the first few
            # cm, where one wheel starting late twists the base and Pure
            # Pursuit (omega ~ speed x lateral) cannot see it yet. The edge
            # angle moves WITH the base yaw, and a positive angle is removed
            # by turning CW (same convention as align_to_tag), so the odom
            # fallback uses the same sign: yaw error = theta - theta_start.
            # Direction-independent — a yaw error is undone by the same
            # in-place turn whether the base is moving forward or back.
            if (self.launch_yaw_hold_dist > 0 and
                    traveled_dist < self.launch_yaw_hold_dist):
                # Heading reference, best source first: the START tag we
                # just aligned to (in view for the first cm), else the
                # target tag, else odom yaw against a reference that is
                # re-anchored to the tag whenever a tag IS in view — so a
                # twist the encoders cannot see (gear play on the reversing
                # wheel) is still held once the tag has left the frame.
                ref_tag = None
                if start_id is not None and start_id in self.detected_tags:
                    ref_tag = start_id
                elif tag_visible:
                    ref_tag = target_id
                already = False
                if ref_tag is not None:
                    rv = self._tag_view(ref_tag) or self.detected_tags[ref_tag]
                    yaw_err_deg = rv.get('edge_deg', tag_edge_angle_deg(rv['corners']))
                    launch_theta_ref = self.current_theta - math.radians(yaw_err_deg)
                    src = f'tag {ref_tag}'
                    # the steering law above already holds the heading on
                    # the target tag
                    already = (ref_tag == target_id and tag_visible
                               and sf_heading_applied)
                else:
                    yaw_err_deg = math.degrees(
                        (self.current_theta - launch_theta_ref + math.pi)
                        % (2.0 * math.pi) - math.pi)
                    src = 'odom'
                # act on the heading the base will settle at, not the one it
                # had when the frame was taken (command delay)
                yaw_err_deg += math.degrees(self._pending_yaw_rad())
                if abs(yaw_err_deg) > abs(launch_peak_yaw_err):
                    launch_peak_yaw_err = yaw_err_deg      # signed, largest |.|
                if not already:
                    omega += -math.radians(yaw_err_deg) * self.launch_yaw_gain
                # Backlash feed-forward against the wheel that has to
                # reverse after the align (see __init__), first ff_s only.
                ff = 0.0
                if (self.launch_backlash_ff_omega > 0 and
                        self._last_align_turn_sign != 0 and
                        (rospy.Time.now() - start_time).to_sec() < self.launch_backlash_ff_s):
                    ff = -self._last_align_turn_sign * self.launch_backlash_ff_omega
                    omega += ff
                    launch_ff_applied = True
                omega = float(np.clip(omega, -self.move_max_angular,
                                      self.move_max_angular))
                rospy.loginfo_throttle(
                    0.5, "[LaunchYawHold] traveled %.3f m yaw_err %+.2f deg "
                         "(%s) ff %+.3f omega %+.4f", traveled_dist, yaw_err_deg,
                    src, ff, omega)

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
