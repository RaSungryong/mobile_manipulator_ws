#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArmController — Fairino FR10v6 motion control + scan orchestration.

After the split this module contains ONLY what moves the arm:
  * Fairino SDK connection, homing, joint/pose motion (IK + q0 seed, TOOL_ID)
  * Keyence closed-loop standoff adjustment (sensor-guided MoveL)
  * the scan-point orchestration loop (move → settle → adjust → delegate)

Everything that is not motion control was extracted, same package:
  * arm_transform.py — world → arm-base 4-DOF pose transform (pure geometry;
    also the file the lift/arm_base_z fix will touch)
  * scan_pipeline.py — camera capture (via /camera/capture) + ONNX Ra
    inference + /scan/* publishing
  * scan_results.py  — incremental 13-column result CSV persistence

The public surface is unchanged: execute_scan_points / move_to_home / cancel /
current_pose_msg / publish_done / is_busy / shutdown. arm_node.py
wraps this class; nothing else instantiates it.
"""

import os
import threading
import queue
import json
import time
import numpy as np

import rospy
from std_msgs.msg import Bool, Float32, String
from robot_msgs.msg import Pose2DWithFlag
from scipy.spatial.transform import Rotation as R

from apriltag_nav import paths
from apriltag_nav.paths import load_yaml_block
try:
    from apriltag_nav.scan_pipeline import image_name_prefix, image_file_name
except ImportError:                     # offline harness with a stub module
    def image_name_prefix(group_id, point_id, index):
        return f"g{int(group_id)}_p{int(point_id)}_i{int(index):04d}"

    def image_file_name(point_id, sample_no, ra_value=None, name_prefix=None):
        return f"{name_prefix}_s{int(sample_no)}.png"
from apriltag_nav.arm_transform import transform_world_to_arm, tag_floor_z_m
from apriltag_nav.lift_height import LiftHeightListener
from apriltag_nav.scan_pipeline import RaScanPipeline
from apriltag_nav.scan_results import ScanResultWriter
from apriltag_nav import paths as _paths
from apriltag_nav.keyence_standoff import (StandoffConfig, StandoffController,
                                            MoveReport)

# ================= Fairino SDK =================
if not paths.add_fairino_sdk_to_path():
    rospy.logwarn(f"[Arm REAL] Fairino SDK not found at {paths.FAIRINO_SDK_PATH}")
from fairino import Robot

# Tool ID registered via set_tool_tcp.py  (vision_tip TCP offset)
# tool=0 → flange (identity), tool=1 → vision_tip
TOOL_ID = 1


class ArmController:
    """
    ArmController (REAL ROBOT)
    =========================
    - Fairino SDK arm control (MoveJ / MoveL / IK)
    - Joint scan and pose scan orchestration
    - Keyence closed-loop distance adjustment
    - STOP-safe cancel
    Capture/inference and CSV persistence are delegated (see module docstring).
    """

    def __init__(self, robot_ip="192.168.58.2", model_path=None):

        # ---------- state ----------
        self.busy = False
        self.cancel_requested = False
        self.current_pose_msg = None

        # StopMotion retry budget — see cancel() for why this is not one shot.
        # 10 x 20 ms = 200 ms worst case; observed successes land on attempt 2,
        # ~14 ms in. Raising the count costs nothing when the stop is accepted
        # early, since the loop returns immediately.
        self.cancel_attempts = int(rospy.get_param('~cancel_attempts', 10))
        self.cancel_retry_s = float(rospy.get_param('~cancel_retry_s', 0.02))

        # ---------- lift compensation for POSE-mode IK ----------
        # arm_base_z is measured at the lift origin; the lift adds up to
        # ~343 mm. Read-only listener — arm_node must not be able to COMMAND
        # the lift (lifter_node is the sole writer; reading is open).
        # `require_lift_height` is the policy for "lifter_node has never
        # published": false (default) keeps the pre-2026-09-11 behaviour of
        # assuming the origin, which is right in practice because every task
        # ends with lift origin homing and pose-mode CSVs carry no
        # lift_height — but it is an ASSUMPTION, so it is logged at error
        # level each time it is used. true refuses the move instead; set it
        # for unattended runs, where a silently-wrong scan is worse than a
        # failed task.
        self.lift_listener = LiftHeightListener(
            rospy.get_param('~lift_height_topic', '/lifter/height'))
        self.require_lift_height = bool(
            rospy.get_param('~require_lift_height', False))
        # Calibrated floor height per tag (2026-09-22): map.yaml's tag `z`
        # (map calibration) → the arm base rides that much above the design
        # floor. Read on first use (_pose_floor_z_m); the tag comes from
        # /robot_pose.id.
        self._map_tags = None

        # ---------- Fairino ----------
        rospy.loginfo("[Arm REAL] Connecting to Fairino robot...")
        self.robot = Robot.RPC(robot_ip)
        time.sleep(0.5)
        ret = self.robot.RobotEnable(1)
        rospy.loginfo(f"[Arm REAL] RobotEnable(1) → {ret}")
        time.sleep(1.0)

        # ---------- which tool frame is the controller actually using? ----------
        # Pose-mode CSV rows are VISION-TIP coordinates (tf_chain.yaml
        # T_ee2tip, the same numbers tools/set_tool_tcp.py writes as tool 1).
        # GetInverseKin has no tool argument: it solves for the controller's
        # ACTIVE tool frame. On 2026-09-14 that frame was the FLANGE (offset
        # 0), so every tip target put the flange there — 338.7 mm from where
        # the paired joint row put the tip. See _probe_tool_frame / _exec_pose.
        from apriltag_nav.tf_chain import tip_offset_mm
        self._tip_offset_mm = tip_offset_mm()
        self._pose_tip_to_flange = self._probe_tool_frame()

        # ---------- joint zero offsets on the COMMAND side (2026-09-28) ----------
        # config/tf/arm_joint_offsets.yaml (physical = reading + dq) acts on
        # the MEASUREMENT chain (tf_chain.arm_flange_T); the controller places
        # FK(q) on every target, so the physical flange lands at FK(q + dq).
        # With ~apply_joint_offsets_cmd true, _exec_pose commands q - dq and
        # move_cart corrects ABSOLUTE targets when asked (physical=True) —
        # see joint_offset_cmd.py for which targets must NOT be corrected.
        # ~joint_offsets_cmd_skip_z (default TRUE): correct xy + rotation and
        # leave the commanded arm-frame z as it is. The offsets' z term is
        # NOT trusted: at the touch poses they predict the tip +9..14 mm
        # above the reading while the Keyence measured +4 / -4 mm, so a
        # full correction would run the case 5..9 mm into the plate on the
        # approach. Their xy IS the measured error: the 2026-09-28
        # tip_touch_cross_tags run (22 stops) landed the tip a constant
        # 8.8 mm toward body -y in both zones, and FK(q + dq) - FK(q) at the
        # recorded touch configurations reproduces it (rms 10.8 -> 5.3 mm
        # residual). The map calibration has these offsets in its chain
        # (joint_offsets_applied: true), so the command side must too.
        # The launch sets ~apply_joint_offsets_cmd true since that evening.
        from apriltag_nav.joint_offset_cmd import load_default as _load_corr
        self._cmd_corr = None
        if bool(rospy.get_param('~apply_joint_offsets_cmd', False)):
            skip_z = bool(rospy.get_param('~joint_offsets_cmd_skip_z', True))
            self._cmd_corr = _load_corr(warn=lambda m: rospy.logwarn("[Arm REAL] " + m), skip_z=skip_z)
            rospy.logwarn("[Arm REAL] " + self._cmd_corr.describe() + " — ON: absolute targets are pre-corrected")
        else:
            rospy.loginfo("[Arm REAL] command-side joint offsets OFF (~apply_joint_offsets_cmd) — "
                          "targets go to the controller as given")

        # ---------- Home ----------
        # Source of truth is config/robot.yaml `arm_home.joints_rad`.
        # Hardcoded list remains as fallback for standalone use.
        _home_cfg = load_yaml_block('arm_home')
        _home_default = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        self.home_joint_positions = list(
            _home_cfg.get('joints_rad', _home_default)
        )
        # Pre-compute degrees once to avoid repeated conversion in move_to_home
        self._home_joints_deg = [np.degrees(j) for j in self.home_joint_positions]

        # ---------- Scan pipeline (camera + inference + /scan topics) ----------
        # The controller does NOT open the camera and does NOT drive the VISION
        # lamp — basler_camera_node owns both; the pipeline calls its service.
        _camera_cfg = load_yaml_block('camera')
        self.stabilization_time = rospy.get_param('~stabilization_time', 0.5)
        # Bound on frames waiting for inference (each ~20 MB). A full queue
        # blocks the scan loop, i.e. a slow model throttles the arm rather
        # than growing memory without limit.
        self.infer_queue_max = int(rospy.get_param('~infer_queue_max', 4))
        self._results_lock = threading.Lock()
        self.pipeline = RaScanPipeline(
            capture_service=rospy.get_param(
                '~capture_service', _camera_cfg.get('service', '/camera/capture')),
            capture_timeout_s=rospy.get_param(
                '~capture_timeout_s', _camera_cfg.get('capture_timeout_s', 20.0)),
            use_vision_led=rospy.get_param(
                '~use_vision_led', _camera_cfg.get('use_vision_led', True)),
            num_samples=rospy.get_param('~num_samples', 1),
            delay_between_samples=rospy.get_param('~delay_between_samples', 0.2),
            save_images=rospy.get_param('~save_images', False),
            output_dir=rospy.get_param('~output_dir', _paths.SCAN_IMAGE_DIR),
            model_path=model_path,
        )

        # ---------- Result persistence ----------
        self.results_writer = ScanResultWriter()

        # ---------- Keyence distance alignment parameters ----------
        # Lookup chain: private ROS param > robot.yaml keyence block > hardcoded default.
        # The loop itself lives in keyence_standoff.StandoffController (pure
        # logic, offline-testable); this block only gathers its config and
        # the two robot-side conversions (beam projection, tool-Z sign).
        _keyence_cfg = load_yaml_block('keyence')
        self.current_keyence_val = None
        self._keyence_seq = 0            # bumps on every /keyence/value message
        self._keyence_lock = threading.Lock()

        def _kp(name, key, default):
            return rospy.get_param('~keyence_' + name, _keyence_cfg.get(key, default))

        self.keyence_tol          = float(_kp('tol', 'tolerance_mm', 0.2))      # perpendicular mm
        self.keyence_dir          = float(rospy.get_param('~keyence_dir', 1.0))  # -sign(k); launch sets -1.0
        self.keyence_kp           = float(_kp('kp', 'kp', 0.8))
        self.keyence_max_steps    = int(_kp('max_steps', 'max_steps', 15))
        self.keyence_max_step_mm  = float(_kp('max_step_mm', 'max_step_mm', 1.0))   # approach fine step
        self.keyence_activate_threshold = float(_kp('activate_threshold', 'activate_threshold', 20.0))
        self.keyence_approach_fraction  = float(_kp('approach_fraction', 'approach_fraction', 0.5))
        self.keyence_retreat_step_mm    = float(_kp('retreat_step_mm', 'retreat_step_mm', 3.0))
        self.keyence_max_travel_mm      = float(_kp('max_travel_mm', 'max_travel_mm', 25.0))
        self.keyence_invalid_abs_mm     = float(_kp('invalid_abs_mm', 'invalid_abs_mm', 90.0))
        self.keyence_samples            = int(_kp('samples', 'samples', 5))
        self.keyence_read_timeout_s     = float(_kp('read_timeout_s', 'read_timeout_s', 1.0))
        self.keyence_settle_s           = float(_kp('settle_s', 'settle_s', 0.3))
        self.keyence_adaptive_gain      = bool(_kp('adaptive_gain', 'adaptive_gain', True))
        self.keyence_gain_ratio_max     = float(_kp('gain_ratio_max', 'gain_ratio_max', 4.0))
        self.keyence_min_response_ratio = float(_kp('min_response_ratio', 'min_response_ratio', 0.25))
        self.keyence_seek_enabled       = bool(_kp('seek_enabled', 'seek_enabled', False))
        self.keyence_seek_step_mm       = float(_kp('seek_step_mm', 'seek_step_mm', 2.0))
        self.keyence_seek_max_mm        = float(_kp('seek_max_mm', 'seek_max_mm', 10.0))
        self.keyence_require_converged  = bool(_kp('require_converged', 'require_converged', False))
        # Direct move (2026-09-29): the whole measured gap in ONE move, then
        # verify — keyence_standoff docstring 10. 'stepped' = the old law.
        self.keyence_move_mode              = str(_kp('move_mode', 'move_mode', 'direct'))
        self.keyence_direct_gain            = float(_kp('direct_gain', 'direct_gain', 1.0))
        self.keyence_direct_max_approach_mm = float(_kp('direct_max_approach_mm', 'direct_max_approach_mm', 12.0))
        self.keyence_direct_max_retreat_mm  = float(_kp('direct_max_retreat_mm', 'direct_max_retreat_mm', 15.0))
        self.keyence_direct_max_spread_mm   = float(_kp('direct_max_spread_mm', 'direct_max_spread_mm', 0.3))
        self.keyence_direct_max_moves       = int(_kp('direct_max_moves', 'direct_max_moves', 5))
        self.keyence_exec_mismatch_mm       = float(_kp('exec_mismatch_mm', 'exec_mismatch_mm', 1.0))
        # Live guard: while an APPROACH move of the standoff loop is in
        # flight, keyence_cb watches every reading and stops the arm when
        # the reading has passed the target by guard_overshoot_mm (or the
        # sensor reports "too close") on guard_frames consecutive messages.
        self.keyence_guard_enabled      = bool(_kp('guard_enabled', 'guard_enabled', True))
        self.keyence_guard_overshoot_mm = float(_kp('guard_overshoot_mm', 'guard_overshoot_mm', 1.0))
        self.keyence_guard_frames       = int(_kp('guard_frames', 'guard_frames', 2))
        self.keyence_max_guard_stops    = int(_kp('max_guard_stops', 'max_guard_stops', 1))
        # Command -> actual offset of a MoveL (see _keyence_move_approach).
        self.keyence_cmd_bias_enabled   = bool(_kp('cmd_bias_enabled', 'cmd_bias_enabled', True))
        self.keyence_cmd_bias_max_mm    = float(_kp('cmd_bias_max_mm', 'cmd_bias_max_mm', 1.5))
        self.keyence_cmd_bias_max_deg   = float(_kp('cmd_bias_max_deg', 'cmd_bias_max_deg', 0.3))
        self.keyence_cmd_bias_carry_mm  = float(_kp('cmd_bias_carry_mm', 'cmd_bias_carry_mm', 100.0))
        self.keyence_cmd_bias_carry_s   = float(_kp('cmd_bias_carry_s', 'cmd_bias_carry_s', 120.0))
        self._standoff_guard = None      # armed only while an approach MoveL runs
        self._standoff_chain = None      # per-adjustment command chain
        self._standoff_bias = None       # last learned command->actual offset
        # Target standoff. The DL-EN1 reads 0 at sensor_zero_mm; the loop holds
        # the reading at (sensor_zero - target), so target == zero (the
        # default, 10 mm) reproduces the old "drive the reading to 0".
        self.keyence_sensor_zero_mm     = float(_kp('sensor_zero_mm', 'sensor_zero_mm', 10.0))
        self.keyence_target_distance_mm = float(_kp('target_distance_mm', 'target_distance_mm',
                                                    self.keyence_sensor_zero_mm))
        self.keyence_setpoint_mm = self.keyence_sensor_zero_mm - self.keyence_target_distance_mm
        if self.keyence_setpoint_mm != 0.0:
            rospy.logwarn(
                f"[Arm REAL] Keyence target standoff {self.keyence_target_distance_mm} mm "
                f"!= sensor zero {self.keyence_sensor_zero_mm} mm: the loop holds the "
                f"reading at {self.keyence_setpoint_mm:+.2f} mm perpendicular. The "
                "Basler focus / Ra model were established at the sensor zero.")
        if self.keyence_seek_enabled:
            rospy.logwarn(
                "[Arm REAL] keyence seek is ENABLED: an out-of-range first "
                f"reading steps {self.keyence_seek_step_mm} mm per attempt "
                f"(<= {self.keyence_seek_max_mm} mm) on the sentinel's sign. "
                "Only safe if the sentinel sign has been confirmed on this sensor.")
        # Angle between the DL-EN1 laser beam and the tool Z axis. The sensor
        # measures along its BEAM, but the correction below moves along tool Z,
        # so the reading must be projected: a perpendicular error e shows up as
        # a reading e/cos(angle), and the matching step is reading*cos(angle).
        # 0.0 = normal incidence = no projection (the historical behaviour).
        self.keyence_beam_angle_deg = rospy.get_param(
            '~keyence_beam_angle_deg', _keyence_cfg.get('beam_angle_deg', 0.0))
        self._keyence_cos = float(np.cos(np.radians(self.keyence_beam_angle_deg)))
        if self._keyence_cos <= 1e-3:
            rospy.logerr(
                f"[Arm REAL] keyence_beam_angle_deg="
                f"{self.keyence_beam_angle_deg} is at/over 90 deg — the beam "
                "never reaches the surface. Falling back to no projection.")
            self._keyence_cos = 1.0
        # Stability. With the projection applied the reading's 1/cos factor is
        # cancelled, so a perpendicular error e leaves e*(1 - kp) after a step:
        # the loop gain is kp ALONE, independent of the beam angle. kp = 1 would
        # be deadbeat; below 1 is under-damped and safe; 2 diverges.
        if not 0.0 < self.keyence_kp < 2.0:
            rospy.logerr(
                f"[Arm REAL] keyence_kp={self.keyence_kp} is outside (0, 2) — "
                "the distance loop does not converge. Expected ~0.8.")
        elif self.keyence_kp > 1.5:
            rospy.logwarn(
                f"[Arm REAL] keyence_kp={self.keyence_kp} > 1.5 — the distance "
                "loop will overshoot and oscillate before settling.")

        # An unset angle is the dangerous case: the reading keeps its 1/cos
        # factor, so the TRUE gain becomes kp/cos(real angle) and the loop
        # diverges once that reaches 2 — silently, since nothing else notices.
        if self.keyence_beam_angle_deg == 0.0:
            rospy.logwarn(
                "[Arm REAL] keyence_beam_angle_deg is 0 (no projection). If the "
                "laser is actually mounted oblique, every correction overshoots "
                f"by 1/cos and the loop diverges past "
                f"{np.degrees(np.arccos(min(1.0, self.keyence_kp / 2.0))):.1f} deg. "
                "Measure it with tools/measure_keyence_angle.py.")

        # ---------- ROS ----------
        rospy.Subscriber("/robot_pose", Pose2DWithFlag, self.pose_cb, queue_size=1)
        rospy.Subscriber("keyence/value", Float32, self.keyence_cb, queue_size=1)
        self.done_pub = rospy.Publisher("/scan_finished", Bool, queue_size=1)
        # Live standoff derived from every /keyence/value message (2026-09-15,
        # for robot_ui's distance-sensor assist): the raw reading, its
        # perpendicular projection, the standoff in mm and the error against
        # the target. Published here rather than computed in the UI so the
        # beam angle, sensor zero and target live in ONE place.
        self.standoff_pub = rospy.Publisher("/arm/standoff_state", String,
                                            queue_size=1)
        # Per-point scan progress for the operator UI (2026-09-14). JSON
        # events, one per phase: start / move / done / failed / finished.
        # Before this the only trace of a failed point was arm_node's rosout
        # and the result CSV — robot_ui showed nothing while 110 IK failures
        # went by.
        self.progress_pub = rospy.Publisher("/arm/scan_progress", String,
                                            queue_size=50)
        # ---------- Ra data collection mode (2026-10-06) ----------
        # With it on, every SCANNED point pauses after the capture: the tool
        # retreats along its own Z (away from the surface) so the operator
        # can put a roughness tester on the spot the Basler just imaged,
        # and the scan resumes on /arm/scan_continue carrying the measured
        # Ra, which is appended to <record_dir>/<run>_ra_measured.csv next
        # to the frame names. Switched live by /arm/collect_mode; nothing
        # changes while it is off. State on /arm/collect_state (latched).
        _collect_cfg = load_yaml_block('collect')

        def _cp(name, key, default):
            return rospy.get_param('~collect_' + name, _collect_cfg.get(key, default))
        self.collect_enabled = bool(_cp('enabled', 'enabled', False))
        # 'pause': N scanned points, then one stop for the Ra entry (method
        # A, two testers measure in parallel); 'mark': no capture at all —
        # every scanned point stops for the operator to write its number
        # beside the spot, and a template CSV is written for the later hand
        # measurement (method B). 2026-10-06 (evening).
        self.collect_mode = str(_cp('mode', 'mode', 'pause'))
        self.collect_batch_size = max(1, int(_cp('batch_size', 'batch_size', 1)))
        self.collect_mark_retreat_mm = float(_cp('mark_retreat_mm', 'mark_retreat_mm', 30.0))
        self.collect_mark_dwell_s = float(_cp('mark_dwell_s', 'mark_dwell_s', 0.0))
        self.collect_retreat_mm = float(_cp('retreat_mm', 'retreat_mm', 80.0))
        self.collect_retreat_speed = int(_cp('retreat_speed', 'retreat_speed', 10))
        self.collect_wait_timeout_s = float(_cp('wait_timeout_s', 'wait_timeout_s', 0.0))
        self.collect_record_dir = _paths.expand_path(
            _cp('record_dir', 'record_dir', _paths.RA_MEASURED_DIR))
        self.collect_state_pub = rospy.Publisher("/arm/collect_state", String,
                                                 queue_size=1, latch=True)
        self._collect_event = threading.Event()
        self._collect_payload = None
        self._collect_lock = threading.Lock()
        self._collect = {'waiting': False, 'n_recorded': 0, 'n_skipped': 0,
                         'record_csv': ''}
        self._collect_batch = []
        self._collect_mark_no = 0
        self._collect_run_stem = ''
        if self.collect_enabled:
            rospy.logwarn("[Arm REAL] Ra COLLECT mode is ON: every scanned point "
                          "pauses for a hand measurement (/arm/scan_continue)")
        self._publish_collect_state()
        # Pose snapshot taken by the WORKER thread between its own RPC calls,
        # so arm_node can keep /arm/state live during a scan (its timer must
        # not touch the single RPC socket while a motion holds it).
        self._live_lock = threading.Lock()
        self._live_pose = None
        self._live_joints = None
        self._live_stamp = 0.0

        # Clear faults and enter automatic mode, but do NOT move. Bringing the
        # stack up must never command arm motion: whatever pose the arm powered
        # up in may be inside a fixture or against the workpiece, and an
        # unattended MoveJ out of it is a collision risk with nobody expecting
        # the arm to move. Homing is explicit only — task_executor homes before
        # every task, and /arm/move_home triggers it manually.
        self.robot.ResetAllError()
        time.sleep(0.3)
        self.robot.Mode(0)
        time.sleep(0.5)

        rospy.loginfo("[ArmController REAL] Ready — arm left in place "
                      "(call /arm/move_home to home it)")

    # --------------------------------------------------
    # ROS CALLBACKS
    # --------------------------------------------------
    def pose_cb(self, msg):
        # Since 2026-09-28 mobile_node also streams a LIVE estimate on
        # /robot_pose (flag False, ~10 Hz: tag-based or odom-carried).
        # Pose-mode IK must keep using the at-rest ARRIVAL pose only
        # (flag True), exactly as before the stream existed.
        if not getattr(msg, 'flag', True):
            return
        self.current_pose_msg = msg

    def keyence_cb(self, msg):
        """Cache the latest Keyence sensor reading and count it, so the
        standoff loop can insist on readings that arrived AFTER its last move
        (a cached value that stopped updating must not drive the arm)."""
        with self._keyence_lock:
            self.current_keyence_val = msg.data
            self._keyence_seq += 1
        self._standoff_guard_check(msg.data)
        pub = getattr(self, 'standoff_pub', None)
        if pub is not None:
            try:
                pub.publish(String(json.dumps(self.standoff_state(msg.data))))
            except Exception:
                pass

    def _standoff_guard_check(self, raw):
        """Live guard of the standoff loop's approach moves (runs on the
        /keyence/value callback thread, so it sees the sensor WHILE MoveL
        blocks the worker). Armed by _keyence_move_approach with the
        perpendicular reading that must not be passed; `frames` consecutive
        readings beyond it — or the positive "too close" sentinel — stop the
        arm. The far sentinel is not a violation: approaching from out of
        range is the seek's business and the surface is then further away,
        not nearer. Does not touch cancel_requested: the scan goes on, the
        loop re-measures at rest and decides."""
        g = getattr(self, '_standoff_guard', None)
        if g is None or g.get('tripped'):
            return
        try:
            raw = float(raw)
        except (TypeError, ValueError):
            return
        if abs(raw) >= self.keyence_invalid_abs_mm:
            bad = raw > 0
            shown = 'too-close sentinel'
        else:
            perp = raw * self._keyence_cos
            bad = perp > g['limit']
            shown = f"reading {perp:+.2f} mm > limit {g['limit']:+.2f} mm"
        g['count'] = g['count'] + 1 if bad else 0
        if g['count'] < g['frames']:
            return
        g['tripped'] = True
        g['why'] = shown
        rospy.logwarn(f"[Arm REAL] standoff live guard: {shown} on "
                      f"{g['frames']} readings in a row — stopping the move")
        g['stopped'] = self._stop_motion_retry()

    def _stop_motion_retry(self):
        """StopMotion with cancel()'s retry (the RPC socket is held by the
        blocking move, so the first attempt is often rejected) but WITHOUT
        the cancel flag. True once one attempt was accepted."""
        attempts = int(getattr(self, 'cancel_attempts', 10))
        wait_s = float(getattr(self, 'cancel_retry_s', 0.02))
        last_error = None
        for attempt in range(1, attempts + 1):
            try:
                ret = self.robot.StopMotion()
            except Exception as e:
                last_error = e
            else:
                if not isinstance(ret, int) or ret == 0:
                    return True
                last_error = f"error code {ret}"
            if attempt < attempts:
                time.sleep(wait_s)
        rospy.logerr(f"[Arm REAL] standoff live guard: StopMotion rejected "
                     f"{attempts} times (last: {last_error})")
        return False

    def standoff_state(self, raw):
        """Interpret one raw Keyence reading in scan terms.

        raw (mm along the beam; the DL-EN1 reads 0 at sensor_zero_mm, NEGATIVE
        when too far, POSITIVE when too close) -> perpendicular deviation
        perp = raw * cos(beam), standoff = sensor_zero - perp, and the error
        against the live target (positive = closer than the target, the
        loop's approach-positive convention). |raw| >= invalid_abs_mm is the
        +/-99999 out-of-range sentinel: no standoff, only the side its sign
        names (negative = too far).
        """
        raw = float(raw)
        state = {'raw_mm': raw, 'target_mm': float(self.keyence_target_distance_mm),
                 'sensor_zero_mm': float(self.keyence_sensor_zero_mm),
                 'valid': False, 'side': None,
                 'perp_mm': None, 'standoff_mm': None, 'err_mm': None}
        if abs(raw) >= self.keyence_invalid_abs_mm:
            state['side'] = 'far' if raw < 0 else 'close'
            return state
        perp = raw * self._keyence_cos
        standoff = self.keyence_sensor_zero_mm - perp
        state.update(valid=True, perp_mm=perp, standoff_mm=standoff,
                     err_mm=self.keyence_target_distance_mm - standoff)
        return state

    def adjust_standoff(self, target_mm=None):
        """On-demand standoff correction from the CURRENT pose (robot_ui's
        "Auto standoff", 2026-09-15): the same closed loop the scan runs
        before every capture, optionally toward a one-off target in mm
        (None = keyence.target_distance_mm). Blocks; honours cancel().
        Returns (converged, message, StandoffResult). The target override
        is scoped to this call so a UI experiment cannot change what the
        next TASK scans at."""
        self.cancel_requested = False
        saved = (self.keyence_setpoint_mm, self.keyence_target_distance_mm)
        if target_mm is not None:
            self.keyence_target_distance_mm = float(target_mm)
            self.keyence_setpoint_mm = (self.keyence_sensor_zero_mm
                                        - float(target_mm))
        try:
            result = self._adjust_distance_to_surface()
        finally:
            self.keyence_setpoint_mm, self.keyence_target_distance_mm = saved
        if result is None:
            return False, 'standoff: no result', None
        tgt = (float(target_mm) if target_mm is not None
               else float(self.keyence_target_distance_mm))
        return (bool(result.converged),
                f"standoff (target {tgt:g} mm): {result.summary()}", result)

    # --------------------------------------------------
    # HOME
    # --------------------------------------------------
    def move_to_home(self):
        rospy.loginfo("[Arm REAL] Move to Home position")
        try:
            self.robot.Mode(0)
            time.sleep(0.3)
            self.robot.SetSpeed(30)
            ret = self.robot.MoveJ(self._home_joints_deg, tool=TOOL_ID, user=0)
            if ret == 14:
                rospy.logwarn("[Arm REAL] Joint cmd point error (14), resetting and retrying...")
                self.robot.ResetAllError()
                time.sleep(0.5)
                ret = self.robot.MoveJ(self._home_joints_deg, tool=TOOL_ID, user=0)
            if ret != 0:
                rospy.logerr(f"[Arm REAL] MoveJ(Home) failed: {ret}")
        except Exception as e:
            rospy.logerr(f"[Arm REAL] Home move exception: {e}")

    # --------------------------------------------------
    # EMERGENCY CANCEL
    # --------------------------------------------------
    def cancel(self):
        """Halt the arm now. Returns True once StopMotion has been accepted.

        ⚠️ RETRIES ARE LOAD-BEARING — a single StopMotion loses a race it hits
        often. The Fairino Robot.RPC is one socket, and MoveCart / MoveJ hold it
        while they block, so a StopMotion issued from another thread (which is
        the only way a stop can arrive) is frequently rejected by the SDK with
        `Request-sent`: a request is already outstanding.

        Measured on the robot 2026-08-12. Both times the arm was genuinely
        moving, the first StopMotion was rejected and a second one ~14 ms later
        went through, with MoveCart returning 5 ms after that — the arm
        stopping. Those second attempts only existed because STOP ALL happens to
        publish /arm/cancel twice (once directly, once via task_executor's STOP).
        The UI's own "Cancel arm motion" button publishes once, and the user
        confirmed it did nothing: same collision, nothing behind it.

        So the retry is what makes a stop reliable rather than lucky. Do not
        "simplify" this back to a single call. A second dedicated RPC connection
        was considered and is unnecessary — the socket frees within tens of ms.

        This runs on a rospy callback thread and can block for up to
        attempts * interval (default 200 ms). That is deliberate: a stop is
        worth blocking one callback thread for, and it returns as soon as one
        attempt is accepted, which is the normal case on the first or second.
        """
        rospy.logwarn("[Arm REAL] CANCEL requested")
        # Set before the first attempt, never after: the flag is what the
        # blocking movers poll, so it must already be true if StopMotion
        # succeeds immediately.
        self.cancel_requested = True

        last_error = None
        for attempt in range(1, self.cancel_attempts + 1):
            try:
                ret = self.robot.StopMotion()
            except Exception as e:
                last_error = e
            else:
                # The SDK returns an error code on some builds and raises on
                # others; treat a non-int return as success.
                if not isinstance(ret, int) or ret == 0:
                    if attempt > 1:
                        rospy.logwarn(
                            f"[Arm REAL] Stop accepted on attempt {attempt} "
                            f"(earlier: {last_error})")
                    return True
                last_error = f"error code {ret}"
            if attempt < self.cancel_attempts:
                time.sleep(self.cancel_retry_s)

        rospy.logerr(
            f"[Arm REAL] Stop REJECTED after {self.cancel_attempts} attempts "
            f"over {self.cancel_attempts * self.cancel_retry_s:.2f}s "
            f"(last: {last_error}). The arm may still be moving — use the "
            "hardware e-stop.")
        return False

    # --------------------------------------------------
    # ERROR RECOVERY
    # --------------------------------------------------
    def read_error_code(self):
        """The controller's error as GetRobotErrorCode returns it (the SDK
        gives (ret, [main, sub]) on current builds), or None if unreadable."""
        fn = getattr(self.robot, 'GetRobotErrorCode', None)
        if fn is None:
            return None
        try:
            return fn()
        except Exception as e:
            rospy.logwarn(f"[Arm REAL] GetRobotErrorCode failed: {e}")
            return None

    @staticmethod
    def error_is_clear(res):
        """True / False from a GetRobotErrorCode result, None if the shape is
        not recognised (then nothing can be concluded from it)."""
        if res is None:
            return None
        codes = res
        if isinstance(res, (tuple, list)) and len(res) == 2 \
                and isinstance(res[0], int):
            if res[0] != 0:
                return None             # the read itself failed
            codes = res[1]
        if isinstance(codes, int):
            return codes == 0
        if isinstance(codes, (tuple, list)) and codes \
                and all(isinstance(c, (int, float)) for c in codes):
            return all(c == 0 for c in codes)
        return None

    def reset_error(self):
        """Clear the controller's latched error WITHOUT moving the arm.

        What a joint soft-limit trip, a collision stop or a command-point
        error leaves behind: the controller refuses every later motion until
        the error is reset. Before 2026-09-28 the only reset was the one in
        __init__ (i.e. restart arm_node) plus move_to_home's single retry on
        error 14. Same sequence as __init__: ResetAllError → RobotEnable(1) →
        Mode(0). No motion is commanded here — getting the joint back inside
        its limit is the operator's next step (jog_joint, then home).

        Returns (ok, message). ok is False when ResetAllError is refused or
        the error code still reads non-zero afterwards — typically because the
        joint is still beyond its soft limit and the controller re-trips; then
        jog that joint back inward or use the controller's teach pendant.
        """
        if self.busy:
            return False, "refused: arm busy"
        self.busy = True
        try:
            before = self.read_error_code()
            try:
                ret = self.robot.ResetAllError()
                time.sleep(0.3)
                en = self.robot.RobotEnable(1)
                time.sleep(0.5)
                self.robot.Mode(0)
                time.sleep(0.3)
            except Exception as e:
                rospy.logerr(f"[Arm REAL] reset_error exception: {e}")
                return False, f"reset_error exception: {e}"
            after = self.read_error_code()
            clear = self.error_is_clear(after)
            rospy.logwarn(f"[Arm REAL] reset_error: ResetAllError → {ret}, "
                          f"RobotEnable(1) → {en}; error {before} → {after}")
            detail = f"code {before} -> {after}"
            if isinstance(ret, int) and ret != 0:
                return False, f"ResetAllError refused (code {ret}); {detail}"
            if clear is False:
                return False, (f"error still set after reset ({detail}) — a "
                               "joint may still be beyond its soft limit: jog "
                               "it back inward, or use the teach pendant")
            if clear is None:
                return True, f"reset sent, error state unconfirmed ({detail})"
            return True, f"error cleared ({detail})"
        finally:
            self.busy = False

    # --------------------------------------------------
    # MAIN ENTRY
    # --------------------------------------------------
    def execute_scan_points(self, scan_points):

        if self.busy:
            rospy.logwarn("[Arm REAL] Busy, ignore scan request")
            return

        if not scan_points:
            rospy.logwarn("[Arm REAL] Empty scan_points")
            self.publish_done()
            return

        if any(p["mode"] == "pose" for p in scan_points):
            if self.current_pose_msg is None:
                rospy.logerr("[Arm REAL] pose scan requires /robot_pose")
                self.publish_done()
                return

        # Register queued points so the CSV is seeded on first write
        self.results_writer.begin(scan_points)

        # Saved frames go into ONE SUBDIRECTORY PER RUN, named after the
        # run's Ra map CSV (<task>_ra_map_<ts>), under the configured
        # output_dir (2026-09-14). Before this every run dumped flat
        # point_<id>_… files into one folder, so 1273 frames of three runs
        # in a day could not be told apart. csv_path is the same for every
        # point of a scan; a scan without one keeps the flat folder.
        base_dir = getattr(self, '_image_dir_base', None)
        if base_dir is None:
            base_dir = self._image_dir_base = getattr(
                self.pipeline, 'output_dir', None)
        run_csv = next((p.get("csv_path") for p in scan_points
                        if p.get("csv_path")), "")
        self._collect_run_stem = (os.path.splitext(os.path.basename(run_csv))[0]
                                  if run_csv else time.strftime('collect_%Y%m%d_%H%M%S'))
        self._collect_batch = []
        mark_pass = bool(self.collect_enabled and self.collect_mode == 'mark')
        if mark_pass:
            self._collect_mark_no = 0
            rospy.logwarn("[Arm REAL] MARK pass: no capture, no Ra map — every scanned "
                          "point stops for its number to be written beside the spot")
        if base_dir:
            self.pipeline.output_dir = (
                os.path.join(base_dir,
                             os.path.splitext(os.path.basename(run_csv))[0])
                if run_csv else base_dir)

        self.busy = True
        self.cancel_requested = False
        results = []
        current_csv_path = None
        n_total = len(scan_points)
        n_ok = n_fail = 0
        self._progress('start', index=0, total=n_total,
                       scan_points=sum(1 for p in scan_points if p.get("scan", True)))

        # Inference + PNG save run OFF the arm's thread (2026-09-14). The arm
        # only has to be still for the capture (~0.4 s); the ~2.6 s of ONNX
        # + disk work that used to follow it now overlaps the move to the
        # next point. One worker, FIFO, so results land in order and the
        # CPU is never asked to run two inferences at once; the queue is
        # bounded so a slow inference throttles the arm instead of piling
        # up 20 MB frames. `results` entries are filled in by the worker
        # and the CSV is rewritten under _results_lock from both threads.
        infer_q = queue.Queue(maxsize=getattr(self, 'infer_queue_max', 4))
        self._results_lock = getattr(self, '_results_lock', None) or threading.Lock()
        worker = threading.Thread(target=self._infer_worker,
                                  args=(infer_q, results, n_total),
                                  name='scan-infer', daemon=True)
        worker.start()

        try:
            for i, p in enumerate(scan_points):

                if self.cancel_requested:
                    rospy.logwarn("[Arm REAL] Scan cancelled")
                    break

                pid = int(p.get("point_id", i))
                gid = int(p.get("group_id", -1))
                current_csv_path = p.get("csv_path", "")

                # TRAVERSE-ONLY waypoints (2026-09-11). An RRT-planned CSV
                # interleaves the collision-free path between work points with
                # the work points themselves; those transitions must be DRIVEN
                # THROUGH but not scanned. Skipping them instead of executing
                # them would send the arm straight between work points and
                # throw away the planning. `scan` defaults True, so a CSV
                # without the column behaves exactly as before.
                do_scan = bool(p.get("scan", True))

                rospy.loginfo(
                    f"[Arm REAL] {'Execute scan point' if do_scan else 'Traverse'}"
                    f" {i+1}/{len(scan_points)}")
                self._progress('move', index=i + 1, total=n_total, point_id=pid,
                               group_id=gid, scan=do_scan, mode=p.get("mode"))

                # Pre-open the camera before the arm starts moving, so the
                # device-open latency runs in parallel with motion + Keyence
                # adjustment instead of serially at capture time. Pointless on
                # a traverse point — nothing is captured there.
                if do_scan and not mark_pass:
                    self.pipeline.preopen()

                entry = {
                    "point_id":          pid,
                    "group_id":          gid,
                    "success":           False,
                    "execution_message": "Not executed",
                    "ra_mean":           None,
                    "ra_std":            None,
                    "ra_min":            None,
                    "ra_max":            None,
                    "num_samples":       0,
                }

                speed = int(p.get("speed", 30))
                self.robot.SetSpeed(speed)

                try:
                    if p["mode"] == "joint":
                        self._exec_joint(p["joints"])
                    elif p["mode"] == "pose":
                        self._exec_pose(p)
                    entry["success"] = True
                    entry["execution_message"] = "Success"
                except Exception as move_err:
                    rospy.logerr(f"[Arm REAL] Move failed at point {pid}: {move_err}")
                    entry["execution_message"] = str(move_err)
                    n_fail += 1
                    self._progress('failed', index=i + 1, total=n_total,
                                   point_id=pid, group_id=gid, scan=do_scan,
                                   message=str(move_err), n_ok=n_ok, n_fail=n_fail)
                    with self._results_lock:
                        results.append(entry)
                        self._save_results(current_csv_path, results)
                    continue

                # A traverse point is DONE once the move lands: no settle, no
                # Keyence (it is nowhere near the surface, and the standoff
                # loop would chase a reading that means nothing there), no
                # capture, and no result row — it is not a measurement.
                if not do_scan:
                    n_ok += 1          # executed as planned; nothing to measure
                    self._progress('done', index=i + 1, total=n_total,
                                   point_id=pid, group_id=gid, scan=False,
                                   message='traverse', n_ok=n_ok, n_fail=n_fail)
                    continue

                # Wait for arm to stabilize at target point
                rospy.loginfo(f"[Arm REAL] Stabilizing {self.stabilization_time}s ...")
                time.sleep(self.stabilization_time)

                # Keyence closed-loop distance adjustment before scan. The
                # outcome goes into the result row: a scan taken at the wrong
                # standoff used to be indistinguishable from a good one.
                standoff = None
                if not self.cancel_requested:
                    standoff = self._adjust_distance_to_surface()
                    # Settle only if the standoff loop actually moved the
                    # tool (each of its MoveLs already rests keyence_settle_s;
                    # this is the extra margin before the shutter). When it
                    # took no step there is nothing to settle from.
                    if standoff is not None and standoff.travel_mm > 0.0:
                        time.sleep(0.5)
                if standoff is not None:
                    if not standoff.converged and self.keyence_require_converged:
                        entry["success"] = False
                        entry["execution_message"] = (
                            f"Standoff not corrected: {standoff.reason}")
                        rospy.logerr(
                            f"[Arm REAL] Point {pid}: {standoff.summary()} — "
                            "capture skipped (keyence_require_converged)")
                        n_fail += 1
                        self._progress('failed', index=i + 1, total=n_total,
                                       point_id=pid, group_id=gid, scan=True,
                                       message=entry["execution_message"],
                                       n_ok=n_ok, n_fail=n_fail)
                        with self._results_lock:
                            results.append(entry)
                            self._save_results(current_csv_path, results)
                        continue
                    entry["execution_message"] = f"Success ({standoff.summary()})"

                # MARK pass (collect mode 'mark'): the tool is at the
                # standoff over the spot; stop for the operator to write the
                # point's number beside it, no capture, no Ra-map row.
                if mark_pass:
                    entry["execution_message"] = "Mark pass (no capture)"
                    n_ok += 1
                    self._refresh_live_pose()
                    self._progress('done', index=i + 1, total=n_total, point_id=pid,
                                   group_id=gid, scan=True, message='mark',
                                   n_ok=n_ok, n_fail=n_fail)
                    self._collect_mark_point(i + 1, n_total, p, pid, gid, entry)
                    continue

                # Capture while the arm is at rest; inference and the PNG
                # save go to the worker. `done` is published as soon as the
                # frames are in hand (the point is measured); the Ra follows
                # in a separate `result` event when the worker finishes it.
                frames = []
                if not self.cancel_requested:
                    frames = self.pipeline.capture(
                        point_id=pid,
                        cancelled=lambda: self.cancel_requested,
                    )
                if not frames:
                    entry["execution_message"] += " (no frames captured)"

                n_ok += 1
                self._refresh_live_pose()
                self._progress('done', index=i + 1, total=n_total, point_id=pid,
                               group_id=gid, scan=True,
                               message=entry["execution_message"],
                               n_ok=n_ok, n_fail=n_fail)
                with self._results_lock:
                    results.append(entry)
                    # Incremental CSV save (preserves results even if cancelled mid-scan)
                    self._save_results(current_csv_path, results)
                image_prefix = image_name_prefix(gid, pid, i + 1)
                if frames:
                    infer_q.put((i + 1, pid, gid, entry, frames, current_csv_path,
                                 image_prefix))
                # Ra data collection (mode 'pause'): after every batch_size
                # scanned points — or after the LAST one, while the tool is
                # still over it — retreat and wait until the operator has
                # measured the spots and sent /arm/scan_continue. A cancel
                # during the wait ends the scan at the loop top with the
                # tool left retreated.
                if frames and self.collect_enabled and not mark_pass:
                    self._collect_batch.append({
                        'index': i + 1, 'group_id': gid, 'point_id': pid,
                        'images': [image_file_name(pid, k + 1, None, image_prefix)
                                   for k in range(len(frames))],
                        'standoff': entry["execution_message"],
                        'x': p.get('x'), 'y': p.get('y'), 'z': p.get('z'),
                        'csv_path': current_csv_path})
                    more = any(q.get("scan", True) for q in scan_points[i + 1:])
                    if len(self._collect_batch) >= self.collect_batch_size or not more:
                        batch, self._collect_batch = self._collect_batch, []
                        self._collect_pause(batch, n_total)

            if not self.cancel_requested:
                rospy.loginfo("[Arm REAL] Scan finished → Home")
                self.move_to_home()

        except Exception as e:
            rospy.logerr(f"[Arm REAL] Scan exception: {e}")

        finally:
            # Scan over (finished or cancelled) — let the camera close now
            # rather than idling warm for idle_close_sec.
            self.pipeline.release()
            # Every captured frame still gets its Ra: the worker drains the
            # queue (a cancel only stops the ARM; frames already taken are
            # cheap to finish and the CSV must not end with blank rows for
            # points that were measured). The home move above overlapped
            # the last inference, so this join is usually short.
            infer_q.put(None)
            worker.join()
            self._progress('finished', index=len(results), total=n_total,
                           n_ok=n_ok, n_fail=n_fail,
                           cancelled=bool(self.cancel_requested))
            # Always publish done so task_executor never hangs
            self.publish_done()
            self.busy = False

    # --------------------------------------------------
    # BACKGROUND INFERENCE (one worker per scan)
    # --------------------------------------------------
    def _infer_worker(self, infer_q, results, n_total):
        """Consume (index, pid, gid, entry, frames, csv_path, image_prefix)
        until None.

        Fills the entry's ra_* fields in place, rewrites the CSV under
        _results_lock and publishes a `result` progress event. Never lets
        an exception escape — a failed inference is recorded on its row
        and the next frame is processed.
        """
        while True:
            item = infer_q.get()
            if item is None:
                return
            index, pid, gid, entry, frames, csv_path, image_prefix = item
            try:
                scan_result = self.pipeline.process(pid, frames,
                                                    name_prefix=image_prefix)
            except Exception as e:
                rospy.logerr(f"[Arm REAL] Inference failed at point {pid}: {e}")
                scan_result = None
            with self._results_lock:
                if scan_result:
                    entry["ra_mean"]     = scan_result["ra_mean"]
                    entry["ra_std"]      = scan_result["ra_std"]
                    entry["ra_min"]      = scan_result["ra_min"]
                    entry["ra_max"]      = scan_result["ra_max"]
                    entry["num_samples"] = scan_result["num_samples"]
                else:
                    entry["execution_message"] += " (no Ra)"
                self._save_results(csv_path, results)
            self._progress('result', index=index, total=n_total, point_id=pid,
                           group_id=gid, scan=True,
                           ra_mean=entry["ra_mean"], ra_std=entry["ra_std"],
                           num_samples=entry["num_samples"],
                           message=entry["execution_message"])

    def _save_results(self, csv_path, results):
        """Rewrite the Ra map. Caller holds _results_lock."""
        if not csv_path:
            return
        try:
            self.results_writer.save(csv_path, results)
        except Exception as csv_err:
            rospy.logerr(f"[Arm REAL] Failed to save results: {csv_err}")

    # --------------------------------------------------
    # RA DATA COLLECTION MODE (2026-10-06)
    # --------------------------------------------------
    COLLECT_MODES = ('pause', 'mark')

    def set_collect_mode(self, on):
        """Switch the collection behaviour on / off. Takes effect at the next
        scanned point; a point already waiting is not released by this."""
        self.collect_enabled = bool(on)
        rospy.loginfo(f"[Arm REAL] Ra collect mode {'ON' if on else 'OFF'} "
                      f"({self.collect_mode}, batch {self.collect_batch_size})")
        self._publish_collect_state()
        return True, f"collect mode {'on' if on else 'off'}"

    def set_collect_config(self, cfg=None):
        """mode ('pause' | 'mark'), batch_size (>= 1), retreat_mm,
        mark_retreat_mm, mark_dwell_s — any subset. Applies from the next
        scanned point (the mode from the next SCAN: a running pass keeps
        the one it started with)."""
        cfg = dict(cfg or {})
        changed = []
        if 'mode' in cfg:
            mode = str(cfg['mode'])
            if mode not in self.COLLECT_MODES:
                return False, f"unknown collect mode {mode!r} (pause | mark)"
            self.collect_mode = mode; changed.append(f"mode {mode}")
        if 'batch_size' in cfg:
            n = int(cfg['batch_size'])
            if n < 1 or n > 50:
                return False, f"batch_size {n} out of range 1..50"
            self.collect_batch_size = n; changed.append(f"batch {n}")
        for key, lo, hi in (('retreat_mm', 0.0, 300.0), ('mark_retreat_mm', 0.0, 300.0),
                            ('mark_dwell_s', 0.0, 600.0)):
            if key in cfg:
                v = float(cfg[key])
                if not lo <= v <= hi:
                    return False, f"{key} {v} out of range {lo}..{hi}"
                setattr(self, 'collect_' + key, v); changed.append(f"{key} {v:g}")
        rospy.loginfo(f"[Arm REAL] collect config: {', '.join(changed) or 'unchanged'}")
        self._publish_collect_state()
        return True, "collect config: " + (', '.join(changed) or 'unchanged')

    def collect_continue(self, payload=None):
        """Release the stop the scan is waiting on. Called from the ROS
        callback thread, like cancel().

        pause mode — payload (dict): `points`: list of {group_id, point_id,
        ra | readings (list, ra = mean), note, skip}; a batch point not in
        the list is recorded as skipped ("not entered"). The legacy
        top-level `ra` / `readings` / `skip` / `note` form is accepted when
        the batch holds ONE point.
        mark mode — any payload (the operator has written the number)."""
        payload = dict(payload or {})
        with self._collect_lock:
            waiting = bool(self._collect.get('waiting'))
            kind = self._collect.get('kind')
            batch = [dict(b) for b in self._collect.get('points') or []]
        if not waiting:
            return False, "no point is waiting for a measurement"
        if kind == 'mark':
            self._collect_payload = {'mark': True, 'note': str(payload.get('note', '') or '')}
            self._collect_event.set()
            return True, "marked — continuing"

        entries = payload.get('points')
        if entries is None:
            if len(batch) != 1:
                return False, (f"{len(batch)} points are waiting — send `points` "
                               "with one entry per point")
            entries = [{'group_id': batch[0]['group_id'], 'point_id': batch[0]['point_id'],
                        'ra': payload.get('ra'), 'readings': payload.get('readings'),
                        'note': payload.get('note', ''), 'skip': payload.get('skip', False)}]
        parsed = {}
        for e in entries:
            try:
                key = (int(e['group_id']), int(e['point_id']))
            except (KeyError, TypeError, ValueError):
                return False, f"point entry without group_id / point_id: {e}"
            if key not in {(b['group_id'], b['point_id']) for b in batch}:
                return False, f"point g{key[0]} p{key[1]} is not in the waiting batch"
            readings = e.get('readings')
            if isinstance(readings, str):
                readings = readings.replace(',', ' ').split()
            try:
                readings = [float(v) for v in (readings or [])]
                ra = e.get('ra')
                ra = None if ra is None else float(ra)
            except (TypeError, ValueError):
                return False, f"point g{key[0]} p{key[1]}: not a number"
            if ra is None and readings:
                ra = float(np.mean(readings))
            skip = bool(e.get('skip', False))
            if ra is None and not skip:
                return False, f"point g{key[0]} p{key[1]}: no Ra value — give ra / readings, or skip"
            parsed[key] = {'ra': ra, 'readings': readings,
                           'note': str(e.get('note', '') or ''), 'skip': skip}
        self._collect_payload = {'points': parsed}
        self._collect_event.set()
        n_ra = sum(1 for v in parsed.values() if not v['skip'])
        return True, f"{n_ra} Ra recorded, {len(batch) - n_ra} skipped"

    def _publish_collect_state(self, **extra):
        try:
            with self._collect_lock:
                st = dict(self._collect)
            st.update(extra)
            st['enabled'] = bool(self.collect_enabled)
            st['mode'] = self.collect_mode
            st['batch_size'] = int(self.collect_batch_size)
            st['retreat_mm'] = float(self.collect_retreat_mm)
            st['mark_retreat_mm'] = float(self.collect_mark_retreat_mm)
            st['mark_dwell_s'] = float(self.collect_mark_dwell_s)
            st['stamp'] = rospy.get_time()
            self.collect_state_pub.publish(String(json.dumps(st, default=str)))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[Arm REAL] collect state publish failed: {e}")

    def _collect_csv_path(self, suffix='_ra_measured.csv'):
        """<record_dir>/<ra_map stem><suffix> — the stem the run's Ra map and
        image folder carry, so the three pair by name."""
        return os.path.join(self.collect_record_dir, self._collect_run_stem + suffix)

    COLLECT_COLUMNS = ['run', 'mark_no', 'index', 'group_id', 'point_id', 'images',
                       'ra_measured', 'ra_readings', 'note', 'skipped',
                       'standoff', 'x', 'y', 'z', 'image_dir', 'measured_at']

    def _collect_record(self, path, row):
        """Append one row (dict over COLLECT_COLUMNS) to a measured /
        template CSV; the header is written when the file is new."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        new = not os.path.exists(path)
        import csv
        with open(path, 'a', newline='') as f:
            w = csv.DictWriter(f, fieldnames=self.COLLECT_COLUMNS,
                               extrasaction='ignore')
            if new:
                w.writeheader()
            w.writerow(row)

    def _collect_retreat(self, retreat_mm):
        """MoveL the tool retreat_mm along its own Z AWAY from the surface
        (the standoff loop's sign convention: approach = -keyence_dir
        along tool Z, so retreat = +keyence_dir). Returns the pose read
        before the move — the pose to return to — or None if it failed.
        retreat_mm 0 reads the pose and moves nothing."""
        ret, pose = self.robot.GetActualTCPPose()
        if ret != 0:
            rospy.logerr(f"[Arm REAL] collect: GetActualTCPPose failed: {ret}")
            return None
        if float(retreat_mm) <= 0.0:
            return [float(v) for v in pose]
        p0 = np.array(pose[:3], dtype=float)
        rpy = [float(v) for v in pose[3:6]]
        r = R.from_euler('xyz', rpy, degrees=True)
        r_mat = r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()
        z_vec = np.array(r_mat[:, 2], dtype=float)
        dz = float(retreat_mm) * float(self.keyence_dir)
        target = [float(v) for v in (p0 + z_vec * dz)] + rpy
        self.robot.SetSpeed(int(self.collect_retreat_speed))
        ret = self.robot.MoveL(target, TOOL_ID, 0)
        if ret != 0:
            rospy.logerr(f"[Arm REAL] collect: retreat MoveL failed: {ret}")
            return None
        self._refresh_live_pose()
        return [float(v) for v in pose]

    def _collect_return(self, pose, moved):
        if not moved:
            return True
        self.robot.SetSpeed(int(self.collect_retreat_speed))
        ret = self.robot.MoveL(list(pose), TOOL_ID, 0)
        if ret != 0:
            rospy.logerr(f"[Arm REAL] collect: return MoveL failed: {ret}")
            return False
        self._refresh_live_pose()
        return True

    def _collect_arm_xy(self, point):
        """Arm-frame (x, y) mm of a world point through pose mode's own
        transform (position only), or None without x y z / a robot pose."""
        try:
            if point.get('x') is None or point.get('y') is None or point.get('z') is None:
                return None
            msg = getattr(self, 'current_pose_msg', None)
            if msg is None:
                return None
            g = {'x': float(point['x']), 'y': float(point['y']), 'z': float(point['z']),
                 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
            lift_m = 0.0
            try:
                lift_m = float(self.lift_listener.height_m() or 0.0)
            except Exception:
                pass
            pos, _ = transform_world_to_arm(g, msg, lift_m)
            return (float(pos[0]), float(pos[1]))
        except Exception:
            return None

    def _collect_batch_points(self, batch):
        """The batch as published in the waiting state: every point with its
        offset from the LAST one (the one under the tip) in world mm and in
        arm-base mm, so the operators can find the earlier spots."""
        last = batch[-1]
        last_arm = self._collect_arm_xy(last)
        out = []
        for b in batch:
            pt = {k: b[k] for k in ('index', 'group_id', 'point_id', 'images', 'standoff')}
            pt['is_tip'] = b is last
            pt['x'], pt['y'], pt['z'] = b.get('x'), b.get('y'), b.get('z')
            pt['dx_world_mm'] = pt['dy_world_mm'] = None
            pt['dx_arm_mm'] = pt['dy_arm_mm'] = None
            if None not in (b.get('x'), b.get('y'), last.get('x'), last.get('y')):
                pt['dx_world_mm'] = round((float(b['x']) - float(last['x'])) * 1000.0, 1)
                pt['dy_world_mm'] = round((float(b['y']) - float(last['y'])) * 1000.0, 1)
            arm = self._collect_arm_xy(b)
            if arm is not None and last_arm is not None:
                pt['dx_arm_mm'] = round(arm[0] - last_arm[0], 1)
                pt['dy_arm_mm'] = round(arm[1] - last_arm[1], 1)
            out.append(pt)
        return out

    def _collect_wait(self):
        """Block until collect_continue() or cancel (or the wait timeout).
        Returns (payload or None, timed_out)."""
        t0 = time.time()
        while not self.cancel_requested:
            if self._collect_event.wait(0.1):
                return self._collect_payload, False
            if (self.collect_wait_timeout_s > 0
                    and time.time() - t0 > self.collect_wait_timeout_s):
                return self._collect_payload, True
        return self._collect_payload, False

    def _collect_row(self, b, measured, mark_no=''):
        measured = measured or {}
        ra = measured.get('ra')
        return {'run': self._collect_run_stem, 'mark_no': mark_no,
                'index': b['index'], 'group_id': b['group_id'], 'point_id': b['point_id'],
                'images': ';'.join(b.get('images') or []),
                'ra_measured': '' if ra is None else f"{float(ra):.4f}",
                'ra_readings': ' '.join(f"{v:.4f}" for v in measured.get('readings', [])),
                'note': measured.get('note', ''),
                'skipped': bool(measured.get('skip', False)),
                'standoff': b.get('standoff', ''),
                'x': '' if b.get('x') is None else b['x'],
                'y': '' if b.get('y') is None else b['y'],
                'z': '' if b.get('z') is None else b['z'],
                'image_dir': getattr(self.pipeline, 'output_dir', '') or '',
                'measured_at': time.strftime('%Y-%m-%d %H:%M:%S')}

    def _collect_pause(self, batch, total):
        """Method A, one stop: the tool is over batch[-1]. Retreat, publish
        `wait` with every point of the batch and its offset from the tip,
        block until collect_continue() / cancel / timeout, record one row
        per point, return to the captured pose. Never raises into the scan
        loop."""
        record_csv = self._collect_csv_path('_ra_measured.csv')
        last = batch[-1]
        moved = False
        try:
            back_pose = self._collect_retreat(self.collect_retreat_mm)
            moved = back_pose is not None and self.collect_retreat_mm > 0
            points = self._collect_batch_points(batch)
            with self._collect_lock:
                self._collect.update({
                    'waiting': True, 'kind': 'pause', 'index': last['index'],
                    'total': total, 'group_id': last['group_id'],
                    'point_id': last['point_id'], 'images': last['images'],
                    'image_dir': getattr(self.pipeline, 'output_dir', '') or '',
                    'standoff': last['standoff'], 'points': points,
                    'retreated': back_pose is not None, 'record_csv': record_csv})
            self._collect_event.clear()
            self._collect_payload = None
            self._publish_collect_state()
            self._progress('wait', index=last['index'], total=total,
                           point_id=last['point_id'], group_id=last['group_id'],
                           scan=True, kind='pause', images=last['images'],
                           points=[(p['group_id'], p['point_id']) for p in points],
                           message=f"waiting for the hand measurement of {len(batch)} point(s)")
            rospy.loginfo(f"[Arm REAL] collect: {len(batch)} point(s) waiting for Ra "
                          f"(tip over g{last['group_id']} p{last['point_id']}, "
                          f"{last['index']}/{total})")

            payload, timed_out = self._collect_wait()
            if self.cancel_requested and not payload:
                rospy.logwarn("[Arm REAL] collect: cancelled while waiting; tool left retreated")
                return
            measured = (payload or {}).get('points', {})
            n_ra = n_skip = 0
            for b in batch:
                key = (b['group_id'], b['point_id'])
                m = measured.get(key)
                if m is None:
                    m = {'ra': None, 'readings': [], 'skip': True,
                         'note': (f'timeout after {self.collect_wait_timeout_s:.0f} s'
                                  if timed_out else 'not entered')}
                row = self._collect_row(b, m)
                try:
                    self._collect_record(record_csv, row)
                except Exception as e:
                    rospy.logerr(f"[Arm REAL] collect: record failed: {e}")
                if row['skipped']:
                    n_skip += 1
                else:
                    n_ra += 1
                rospy.loginfo(f"[Arm REAL] collect: g{key[0]} p{key[1]} -> "
                              f"{'skipped' if row['skipped'] else 'Ra ' + row['ra_measured']}")
            with self._collect_lock:
                self._collect['n_recorded'] += n_ra
                self._collect['n_skipped'] += n_skip
                self._collect['last'] = {'group_id': last['group_id'],
                                         'point_id': last['point_id'],
                                         'n_ra': n_ra, 'n_skip': n_skip}
            self._progress('resume', index=last['index'], total=total,
                           point_id=last['point_id'], group_id=last['group_id'],
                           scan=True, kind='pause', n_ra=n_ra, n_skip=n_skip,
                           ra_measured=(measured.get((last['group_id'], last['point_id'])) or {}).get('ra'),
                           skipped=bool((measured.get((last['group_id'], last['point_id'])) or {'skip': True}).get('skip')))
            if back_pose is not None and not self.cancel_requested:
                self._collect_return(back_pose, moved)
        except Exception as e:
            rospy.logerr(f"[Arm REAL] collect: pause failed: {e}")
        finally:
            with self._collect_lock:
                self._collect['waiting'] = False
                self._collect['points'] = []
            self._publish_collect_state()

    def _collect_mark_point(self, index, total, point, pid, gid, entry):
        """Method B, one stop: the tool is at the standoff over the spot.
        Retreat mark_retreat_mm, publish `wait` (kind mark) with the running
        number, block until collect_continue() / cancel (or mark_dwell_s),
        append the template row, return. Never raises into the scan loop."""
        template = self._collect_csv_path('_mark_template.csv')
        self._collect_mark_no += 1
        mark_no = self._collect_mark_no
        b = {'index': index, 'group_id': gid, 'point_id': pid, 'images': [],
             'standoff': entry.get("execution_message", ""),
             'x': point.get('x'), 'y': point.get('y'), 'z': point.get('z')}
        moved = False
        try:
            back_pose = self._collect_retreat(self.collect_mark_retreat_mm)
            moved = back_pose is not None and self.collect_mark_retreat_mm > 0
            with self._collect_lock:
                self._collect.update({
                    'waiting': True, 'kind': 'mark', 'mark_no': mark_no,
                    'index': index, 'total': total, 'group_id': gid,
                    'point_id': pid, 'images': [], 'image_dir': '',
                    'standoff': b['standoff'], 'points': [],
                    'retreated': back_pose is not None, 'record_csv': template})
            self._collect_event.clear()
            self._collect_payload = None
            self._publish_collect_state()
            self._progress('wait', index=index, total=total, point_id=pid,
                           group_id=gid, scan=True, kind='mark', mark_no=mark_no,
                           message=f"write #{mark_no} beside the spot")
            rospy.loginfo(f"[Arm REAL] mark: #{mark_no} = g{gid} p{pid} ({index}/{total}) "
                          f"— waiting for the operator")
            if self.collect_mark_dwell_s > 0:
                self._collect_event.wait(self.collect_mark_dwell_s)
                payload = self._collect_payload or {'mark': True}
            else:
                payload, _ = self._collect_wait()
            if self.cancel_requested and not payload:
                rospy.logwarn("[Arm REAL] mark: cancelled while waiting; tool left retreated")
                return
            row = self._collect_row(b, {'ra': None, 'readings': [], 'skip': False,
                                        'note': (payload or {}).get('note', '')}, mark_no)
            try:
                self._collect_record(template, row)
            except Exception as e:
                rospy.logerr(f"[Arm REAL] mark: template write failed: {e}")
            with self._collect_lock:
                self._collect['n_recorded'] += 1
                self._collect['last'] = {'group_id': gid, 'point_id': pid, 'mark_no': mark_no}
            self._progress('resume', index=index, total=total, point_id=pid,
                           group_id=gid, scan=True, kind='mark', mark_no=mark_no)
            if back_pose is not None and not self.cancel_requested:
                self._collect_return(back_pose, moved)
        except Exception as e:
            rospy.logerr(f"[Arm REAL] mark: stop failed at point {pid}: {e}")
        finally:
            with self._collect_lock:
                self._collect['waiting'] = False
            self._publish_collect_state()

    # --------------------------------------------------
    # PROGRESS EVENTS + LIVE POSE (for the operator UI)
    # --------------------------------------------------
    def _progress(self, phase, **fields):
        """Publish one /arm/scan_progress event. Never raises."""
        try:
            payload = {'phase': phase, 'stamp': rospy.get_time()}
            payload.update(fields)
            pose = self.live_pose()
            if pose is not None:
                payload['tcp_pose'] = [round(float(v), 2) for v in pose[0]]
            self.progress_pub.publish(String(json.dumps(payload, default=str)))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[Arm REAL] progress publish failed: {e}")

    def _refresh_live_pose(self):
        """Snapshot pose + joints from the worker thread (between its own RPC
        calls, so it cannot collide with a held motion). arm_node's state
        timer serves it while it cannot take the executor lock."""
        pose = self.get_tcp_pose()
        joints = self.get_joints_deg()
        if pose is None and joints is None:
            return
        with self._live_lock:
            if pose is not None:
                self._live_pose = pose
            if joints is not None:
                self._live_joints = joints
            self._live_stamp = time.time()

    def live_pose(self):
        """(tcp_pose, joints, age_s) of the last worker-thread snapshot, or
        None when there has never been one."""
        with self._live_lock:
            if self._live_pose is None and self._live_joints is None:
                return None
            return (list(self._live_pose or [0.0] * 6),
                    list(self._live_joints or [0.0] * 6),
                    time.time() - self._live_stamp)

    # --------------------------------------------------
    # KEYENCE DISTANCE ADJUSTMENT
    # --------------------------------------------------
    def _keyence_read_fresh(self, n, timeout_s):
        """Return up to n perpendicular readings that arrive from now on.
        Each /keyence/value message is taken once (by sequence), so a sensor
        that stopped publishing yields [] after the timeout instead of its
        last value repeated n times."""
        out = []
        with self._keyence_lock:
            last_seq = self._keyence_seq
        deadline = time.time() + max(0.05, float(timeout_s))
        while len(out) < n and time.time() < deadline:
            if self.cancel_requested:
                break
            with self._keyence_lock:
                seq, val = self._keyence_seq, self.current_keyence_val
            if seq != last_seq and val is not None:
                last_seq = seq
                v = float(val)
                # The +/-99999 out-of-range sentinel must reach the loop
                # unprojected, or cos() would shrink it under invalid_abs_mm
                # and it would be taken for a (huge) real reading.
                out.append(v if abs(v) >= self.keyence_invalid_abs_mm
                           else v * self._keyence_cos)
                continue
            time.sleep(0.002)
        return out

    @staticmethod
    def _wrap_deg(a):
        return (float(a) + 180.0) % 360.0 - 180.0

    def _standoff_carried_bias(self, pos):
        """The command->actual offset learned by the last standoff move, if
        it was learned near here and recently; else zeros. [x y z rx ry rz]."""
        b = getattr(self, '_standoff_bias', None)
        if not self.keyence_cmd_bias_enabled or b is None:
            return np.zeros(6)
        if time.time() - b['stamp'] > self.keyence_cmd_bias_carry_s:
            return np.zeros(6)
        if np.linalg.norm(np.asarray(pos) - b['pos']) > self.keyence_cmd_bias_carry_mm:
            return np.zeros(6)
        return b['delta'].copy()

    def _keyence_move_approach(self, approach_mm):
        """Translate the tool along its own Z axis by approach_mm of
        PERPENDICULAR standoff (positive = toward the surface), orientation
        unchanged, then settle so the next reading is taken at rest.
        keyence_dir carries the tool-Z sign: it is -sign(k) with
        k = d(reading)/d(toolZ), so approach = -dir along tool Z.

        Returns a keyence_standoff.MoveReport with the distance the arm
        EXECUTED (pose readback before / after, along the tool Z of the
        adjustment's first pose).

        COMMAND CHAIN (2026-09-29). A MoveL does not end on its target:
        the readback afterwards is a constant small offset from it (2026-09-18:
        (0, -0.09, -0.135) mm; tip tour 2026-09-28 at 1.1 m reach:
        ~(0.2, -0.3, -0.4) mm and 0.06 deg per move). The loop used to build
        every target as "readback + dz", so EVERY move carried that offset
        once more: a 0.3 mm retreat moved the tool 0.2 mm CLOSER and 47 of
        64 adjustments of the tip tours ended "reading does not follow the
        motion". Now the first move of an adjustment starts from the
        readback (minus the offset the last adjustment measured, when that
        was nearby and recent — keyence_cmd_bias_*), and every later move
        is the PREVIOUS COMMAND + dz with the orientation of the first
        pose, so the offset is common to both ends and cancels.

        LIVE GUARD: an approach move arms _standoff_guard_check with the
        reading that must not be passed (target + keyence_guard_overshoot_mm);
        a stop in flight is reported as guard_stop, not as a failure."""
        direction = -float(self.keyence_dir)
        dz = float(approach_mm) * direction

        ret, pose = self.robot.GetActualTCPPose()
        if ret != 0:
            rospy.logerr(f"[Arm REAL] GetActualTCPPose failed: {ret}")
            return MoveReport(False, note=f'GetActualTCPPose {ret}')
        p0 = np.array(pose[:3], dtype=float)

        chain = getattr(self, '_standoff_chain', None)
        if chain is None:
            rpy = [float(v) for v in pose[3:6]]
            # Tool Z-axis direction in the robot base frame (Fairino: degrees).
            r = R.from_euler('xyz', rpy, degrees=True)
            # scipy compat: >=1.4 as_matrix(), 1.3 as_dcm()
            r_mat = r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()
            chain = {'z': np.array(r_mat[:, 2], dtype=float), 'rpy': rpy,
                     'cmd': None, 'cmd_rpy': None}
            self._standoff_chain = chain
        z_vec = chain['z']

        if chain['cmd'] is None:
            # First move (or the move after a guard stop): from where the
            # arm IS, pre-compensated with the carried offset.
            bias = self._standoff_carried_bias(p0)
            cmd_pos = p0 - bias[:3] + z_vec * dz
            cmd_rpy = [self._wrap_deg(a - b) for a, b in zip(chain['rpy'], bias[3:])]
        else:
            cmd_pos = chain['cmd'] + z_vec * dz
            cmd_rpy = list(chain['cmd_rpy'])
        new_pose = [float(cmd_pos[0]), float(cmd_pos[1]), float(cmd_pos[2])] + cmd_rpy

        guard = None
        if approach_mm > 0 and self.keyence_guard_enabled:
            guard = {'limit': float(self.keyence_setpoint_mm)
                              + float(self.keyence_guard_overshoot_mm),
                     'frames': max(1, int(self.keyence_guard_frames)),
                     'count': 0, 'tripped': False, 'why': '', 'stopped': None}
        # Low speed for the correction; MoveL keeps the orientation and
        # blocks until the motion is done.
        self.robot.SetSpeed(5)
        self._standoff_guard = guard
        try:
            ret = self.robot.MoveL(new_pose, tool=TOOL_ID, user=0)
        finally:
            self._standoff_guard = None
        tripped = bool(guard and guard['tripped'])
        if ret != 0 and not tripped:
            rospy.logerr(f"[Arm REAL] MoveL failed during standoff adjustment: {ret}")
            chain['cmd'] = None
            return MoveReport(False, note=f'MoveL {ret}')
        self._refresh_live_pose()
        time.sleep(self.keyence_settle_s)

        executed = None
        ret1, pose1 = self.robot.GetActualTCPPose()
        if ret1 == 0:
            p1 = np.array(pose1[:3], dtype=float)
            executed = float(np.dot(p1 - p0, z_vec)) / direction
        if tripped:
            # Interrupted: the chain restarts from the readback.
            chain['cmd'] = None
            return MoveReport(True, executed_mm=executed, guard_stop=True,
                              note=guard['why'] + ('' if guard['stopped']
                                                   else '; StopMotion REJECTED'))
        chain['cmd'] = np.array(cmd_pos, dtype=float)
        chain['cmd_rpy'] = cmd_rpy
        if ret1 == 0 and self.keyence_cmd_bias_enabled:
            d_pos = p1 - chain['cmd']
            d_rpy = np.array([self._wrap_deg(a - b)
                              for a, b in zip(pose1[3:6], cmd_rpy)])
            if (np.linalg.norm(d_pos) <= self.keyence_cmd_bias_max_mm
                    and np.max(np.abs(d_rpy)) <= self.keyence_cmd_bias_max_deg):
                self._standoff_bias = {
                    'delta': np.concatenate([d_pos, d_rpy]),
                    'pos': p1, 'stamp': time.time()}
            else:
                # Not a settle offset any more — do not learn it, and do
                # not keep applying an older one here.
                self._standoff_bias = None
                rospy.logwarn(
                    "[Arm REAL] standoff MoveL ended "
                    f"{np.linalg.norm(d_pos):.2f} mm / "
                    f"{np.max(np.abs(d_rpy)):.2f} deg from its target — "
                    "offset not learned")
        return MoveReport(True, executed_mm=executed)

    def _adjust_distance_to_surface(self):
        """
        Close the standoff loop before a capture: read the Keyence, move the
        tool along its Z axis by the distance the reading asks for (one
        computed move in keyence.move_mode direct), verify at rest, repeat
        until the perpendicular error is inside keyence_tol. Returns a keyence_standoff.StandoffResult; the algorithm
        and every parameter are documented in that module.
        """
        rospy.loginfo("[Arm REAL] Adjusting tool distance using Keyence sensor...")
        cfg = StandoffConfig(
            tolerance_mm=self.keyence_tol,
            kp=self.keyence_kp,
            max_steps=self.keyence_max_steps,
            max_step_mm=self.keyence_max_step_mm,
            approach_fraction=self.keyence_approach_fraction,
            retreat_step_mm=self.keyence_retreat_step_mm,
            activate_threshold_mm=self.keyence_activate_threshold,
            max_travel_mm=self.keyence_max_travel_mm,
            invalid_abs_mm=self.keyence_invalid_abs_mm,
            samples=self.keyence_samples,
            read_timeout_s=self.keyence_read_timeout_s,
            adaptive_gain=self.keyence_adaptive_gain,
            gain_ratio_max=self.keyence_gain_ratio_max,
            min_response_ratio=self.keyence_min_response_ratio,
            setpoint_mm=self.keyence_setpoint_mm,
            seek_enabled=self.keyence_seek_enabled,
            seek_step_mm=self.keyence_seek_step_mm,
            seek_max_mm=self.keyence_seek_max_mm,
            move_mode=self.keyence_move_mode,
            direct_gain=self.keyence_direct_gain,
            direct_max_approach_mm=self.keyence_direct_max_approach_mm,
            direct_max_retreat_mm=self.keyence_direct_max_retreat_mm,
            direct_max_spread_mm=self.keyence_direct_max_spread_mm,
            direct_max_moves=self.keyence_direct_max_moves,
            max_guard_stops=self.keyence_max_guard_stops,
            exec_mismatch_mm=self.keyence_exec_mismatch_mm,
        )
        self._standoff_chain = None      # a new adjustment, a new command chain
        ctl = StandoffController(
            cfg,
            read=self._keyence_read_fresh,
            move=self._keyence_move_approach,
            cancelled=lambda: self.cancel_requested,
            log_info=lambda s: rospy.loginfo(f"[Arm REAL] {s}"),
            log_warn=lambda s: rospy.logwarn(f"[Arm REAL] {s}"),
        )
        try:
            result = ctl.run()
        finally:
            self._standoff_chain = None
            self._standoff_guard = None
        if result.converged:
            rospy.loginfo(f"[Arm REAL] {result.summary()}")
        else:
            rospy.logwarn(f"[Arm REAL] {result.summary()}")
        return result

    # --------------------------------------------------
    # JOINT MOTION
    # --------------------------------------------------
    # ⚠️ _exec_joint / _exec_pose RAISE on any failure (2026-09-14). They
    # used to log and return, and execute_scan_points then marked the point
    # "Success" and went on to the Keyence loop and the capture at whatever
    # pose the arm was left in. On the robot the only reason 110 IK failures
    # were recorded as failures at all was an unrelated TypeError (the SDK
    # returns a bare int when IK has no solution, and `ret, joints = ...`
    # blew up on it). The caller's per-point `except` is the contract: a
    # raise here fails THAT point with the message in the CSV and on
    # /arm/scan_progress, and the scan continues.
    def _exec_joint(self, joints_rad):
        if len(joints_rad) != 6:
            raise ValueError("joint goal must have 6 values")
        joints_deg = [np.degrees(j) for j in joints_rad]
        rospy.loginfo(f"[Arm REAL] MoveJ (deg) → {joints_deg}")
        ret = self.robot.MoveJ(joints_deg, tool=TOOL_ID, user=0)
        if ret != 0:
            raise RuntimeError(f"MoveJ failed (code {ret})")
        self._refresh_live_pose()

    def _probe_tool_frame(self):
        """Read the controller's active TCP offset and decide how a pose-mode
        target (vision-tip coordinates) reaches IK.

        Returns True  — active tool is the FLANGE: convert tip -> flange;
                False — active tool IS the vision tip: send as is;
                None  — unknown / something else: pose mode is REFUSED
                        (a scan at the wrong tool frame is a wrong map, not
                        an error anyone would notice).
        Never raises; the decision is logged at startup.
        """
        try:
            ret, off = self._ik_result(self.robot.GetTCPOffset())
        except Exception as e:
            ret, off = -1, None
            rospy.logerr(f"[Arm REAL] GetTCPOffset raised: {e}")
        if ret != 0 or off is None or len(off) < 6:
            rospy.logerr(
                f"[Arm REAL] Active tool offset unknown (GetTCPOffset -> {ret}, "
                f"{off}). POSE mode is refused until it can be read.")
            return None
        t = np.array(off[:3], dtype=float)
        rot = np.array(off[3:6], dtype=float)
        tip = self._tip_offset_mm
        if np.linalg.norm(t) < 1.0 and np.max(np.abs(rot)) < 0.5:
            rospy.logwarn(
                f"[Arm REAL] Active tool frame is the FLANGE (offset {t.round(1).tolist()} mm). "
                f"Pose-mode tip targets will be converted to flange targets "
                f"(tip offset {tip.round(1).tolist()} mm in the flange frame) before IK.")
            return True
        if np.linalg.norm(t - tip) < 1.0 and np.max(np.abs(rot)) < 0.5:
            rospy.loginfo(
                f"[Arm REAL] Active tool frame is the vision tip {t.round(1).tolist()} mm; "
                "pose-mode targets go to IK as they are.")
            return False
        rospy.logerr(
            f"[Arm REAL] Active tool offset {np.round(off, 2).tolist()} is neither the "
            f"flange nor the vision tip {tip.round(1).tolist()}. POSE mode is refused — "
            "run tools/set_tool_tcp.py or fix config/tf/tf_chain.yaml T_ee2tip.")
        return None

    def _tip_to_flange(self, pos_tip_mm, rpy_deg):
        """Flange position for a vision-tip target: the tip is a pure
        translation in the flange frame, so flange = tip - R(rpy) @ offset."""
        r = R.from_euler('xyz', rpy_deg, degrees=True)
        rm = r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()
        return np.asarray(pos_tip_mm, dtype=float) - rm @ self._tip_offset_mm

    @staticmethod
    def _ik_result(res):
        """Normalise a Fairino IK return: (code, joints) on success, a bare
        int error code when there is no solution."""
        if isinstance(res, (tuple, list)) and len(res) == 2:
            return int(res[0]), res[1]
        try:
            return int(res), None
        except (TypeError, ValueError):
            return -1, None

    # --------------------------------------------------
    # POSE MOTION (IK → MoveJ)
    # --------------------------------------------------
    def _pose_lift_m(self):
        """Live lift extension for pose-mode IK, in metres.

        Raises when the height is unknown and `require_lift_height` is set.
        Otherwise returns 0.0 — the pre-2026-09-11 assumption — and says so
        at error level, because that assumption silently puts the TCP the
        lift's height ABOVE the target when it is wrong.
        """
        lift_m = self.lift_listener.height_m()
        if lift_m is None:
            msg = (f"no {self.lift_listener.topic} yet — lift extension "
                   f"unknown. Pose-mode IK needs it: arm_base_z is measured "
                   f"at the lift origin, so a raised lift puts the TCP that "
                   f"far above the target. Is lifter_node running?")
            if self.require_lift_height:
                raise RuntimeError(msg)
            rospy.logerr_throttle(10.0, "[Arm REAL] " + msg +
                                  " Assuming the lift is at its origin.")
            return 0.0
        return lift_m

    @staticmethod
    def _load_map_tags():
        """map.yaml `tags` (id → dict) for the calibrated floor height; {}
        when the file cannot be read — pose mode then uses the design
        floor, as before 2026-09-22, and says so per point."""
        try:
            import yaml
            with open(paths.MAP_PATH) as f:
                tags = (yaml.safe_load(f) or {}).get('tags') or {}
            return {int(k): v for k, v in tags.items()}
        except Exception as e:
            rospy.logwarn(f"[Arm REAL] map.yaml not readable ({e}); pose-mode "
                          "IK will assume the design floor under every tag")
            return {}

    def _pose_floor_z_m(self):
        """Floor height under the current tag above the CSV z datum (m),
        from the calibrated tag z in map.yaml; 0.0 for a tag without one."""
        if getattr(self, '_map_tags', None) is None:
            self._map_tags = self._load_map_tags()
        tag_id = int(getattr(self.current_pose_msg, 'id', 0) or 0)
        info = self._map_tags.get(tag_id)
        thickness = float((load_yaml_block('robot') or {}).get('tag_thickness', 0.001))
        floor_z = tag_floor_z_m(info, tag_thickness_m=thickness)
        if info is None or info.get('z') is None:
            rospy.logwarn_throttle(
                30.0, f"[Arm REAL] tag {tag_id} has no calibrated z in map.yaml — "
                      "assuming the design floor under the arm")
        return floor_z

    def _exec_pose(self, p):
        lift_m = self._pose_lift_m()
        floor_z_m = self._pose_floor_z_m()
        pos_tip, rpy = transform_world_to_arm(p, self.current_pose_msg, lift_m,
                                              floor_z_m=floor_z_m)
        if self._pose_tip_to_flange is None:
            raise RuntimeError(
                "pose mode refused: the controller's active tool frame is not "
                "the flange or the vision tip (see the startup log)")
        if self._pose_tip_to_flange:
            pos = self._tip_to_flange(pos_tip, rpy)
            rospy.loginfo(
                f"[Arm REAL] tip target {np.round(pos_tip, 1).tolist()} -> flange "
                f"target {np.round(pos, 1).tolist()} (active tool = flange)")
        else:
            pos = np.asarray(pos_tip, dtype=float)
        target = [pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]]
        rospy.loginfo(f"[Arm REAL] IK target: {target}")

        q0 = p.get("q0")
        if q0 is not None:
            # Use GetInverseKinRef with paired joint CSV as reference
            q0_deg = [np.degrees(j) for j in q0]
            rospy.loginfo(f"[Arm REAL] IK with q0 ref: {[round(d,1) for d in q0_deg]}")
            ret, joints = self._ik_result(
                self.robot.GetInverseKinRef(0, target, q0_deg))
        else:
            ret, joints = self._ik_result(
                self.robot.GetInverseKin(0, target, config=-1))

        if ret != 0 or joints is None:
            reach_m = float(np.hypot(pos[0], pos[1])) / 1000.0
            frame = 'flange' if self._pose_tip_to_flange else 'tip'
            raise RuntimeError(
                f"IK failed (code {ret}): {frame} target ({pos[0]:.0f}, {pos[1]:.0f}, "
                f"{pos[2]:.0f}) mm is {reach_m:.2f} m from the arm base "
                f"(FR10 reach 1.40 m); world ({p.get('x', 0):.3f}, "
                f"{p.get('y', 0):.3f}, {p.get('z', 0):.3f}) at robot pose "
                f"({self.current_pose_msg.x:.3f}, {self.current_pose_msg.y:.3f}, "
                f"{self.current_pose_msg.theta:.1f} deg)")
        rospy.loginfo(f"[Arm REAL] IK → joints: {joints}")
        # The IK's q puts the flange on the target under the controller's
        # nominal zeros; the PHYSICAL joints must be q, so the reading to
        # command is q - dq (joint_offset_cmd.py). A world point is always an
        # absolute target, so this is unconditional when offsets are loaded.
        # With skip_z the correction has no joint-space shortcut: the xy-
        # corrected POSE is solved again (seeded with the IK's own q).
        corr = getattr(self, '_cmd_corr', None)
        if corr is not None and np.any(corr.dq):
            if getattr(corr, 'skip_z', False) and corr.enabled:
                def _ik_ref(pose_cmd, _seed=[float(v) for v in joints]):
                    r, q = self._ik_result(self.robot.GetInverseKinRef(0, list(pose_cmd), _seed))
                    return None if (r != 0 or q is None) else [float(v) for v in q]
                try:
                    joints, info = corr.joints_for_physical_pose(target, _ik_ref)
                except ValueError as e:
                    raise RuntimeError(f"joint-offset correction failed: {e}")
                rospy.loginfo(f"[Arm REAL] joint offsets applied (xy only, z kept; {info['corr_mm']:.1f} mm / "
                              f"{info['corr_deg']:.2f} deg, dz {info['dz_skipped_mm']:+.1f} mm skipped): "
                              f"commanding {[round(v, 3) for v in joints]}")
            else:
                joints = corr.joints_for_physical(joints)
                rospy.loginfo(f"[Arm REAL] joint offsets applied: commanding {[round(v, 3) for v in joints]}")
        ret = self.robot.MoveJ(joints, tool=TOOL_ID, user=0)
        if ret != 0:
            raise RuntimeError(f"MoveJ failed (code {ret})")
        self._refresh_live_pose()

    # --------------------------------------------------
    # MANUAL TEACHING — read pose, absolute move, incremental jog
    # --------------------------------------------------
    # These exist for the operator UI (finding a data-collection pose by hand).
    # They are NOT used by the scan loop, which goes through _exec_joint /
    # _exec_pose with the q0 IK seed. Three rules hold for all three:
    #
    #   * TOOL_ID, not tool 0. A pose read or commanded against the flange is a
    #     different point in space than the same numbers against vision_tip, and
    #     mixing the two is how the deleted scripts_ros/ tree got its TCP wrong.
    #   * self.busy is refused, not queued. A jog arriving mid-scan would move
    #     the arm out from under the scan point that is being captured.
    #   * MoveL, not MoveJ (and not MoveCart — switched 2026-09-02). The
    #     operator is watching Cartesian axes, and the calibration align loop
    #     applies small camera-frame corrections: a straight-line TCP path is
    #     what both expect. MoveCart interpolates in JOINT space to the same
    #     endpoint, so the TCP can arc away from the straight line, and it lets
    #     the controller pick a configuration (config=-1); MoveL keeps the
    #     path linear. Trade-off: a straight path through a singularity or a
    #     joint limit is refused where MoveCart would have gone round it.

    def get_tcp_pose(self):
        """Current TCP pose [x,y,z mm, rx,ry,rz deg], or None if the read fails.

        Returning None rather than raising: this is polled ~10 Hz for a live
        display, and a single dropped RPC read must not take the node down.
        """
        try:
            ret, pose = self.robot.GetActualTCPPose()
            if ret != 0:
                return None
            return [float(v) for v in pose]
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[Arm REAL] TCP pose read failed: {e}")
            return None

    def get_joints_deg(self):
        """Current joint angles J1..J6 [deg], or None if the read fails."""
        try:
            ret, joints = self.robot.GetActualJointPosDegree()
            if ret != 0:
                return None
            return [float(v) for v in joints]
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[Arm REAL] Joint read failed: {e}")
            return None

    def move_cart(self, pose, vel=30.0, acc=50.0, linear=True, physical=False):
        """Absolute Cartesian move to [x,y,z mm, rx,ry,rz deg].

        Returns (ok, message). Blocks until the SDK call returns.

        ``physical=True`` (2026-09-28): ``pose`` is where the PHYSICAL flange
        must land, and the joint zero offsets are applied — the controller
        is sent the pose whose nominal IK solution q gives FK(q + dq) =
        pose (joint_offset_cmd.CommandCorrector). Use it for absolute
        targets (a world point, the sheet, a plan seed). Leave it False
        for targets built as "current reading + delta" (jog, the align's
        correction steps, the standoff loop): there the offsets' effect
        cancels between the reading and the target, and correcting would
        ADD the error. The UI's MOVE button sends readings, so it stays
        False.

        ``linear=True`` -> MoveL (straight TCP path), the default since
        2026-09-02. ``linear=False`` -> MoveCart (joint-interpolated to the
        same endpoint). Measured the same day on the robot: MoveL crawls
        when the path reorients the wrist — a 300 mm / 20 deg-yaw view move
        took 5-6 s in one arm configuration and 22-34 s in another, and two
        800 mm / 40-90 deg moves hit the SDK's 60 s RPC timeout — where
        MoveCart to the same pose takes a few seconds. So big repositioning
        moves (the calibration view pose) ask for linear=False and only the
        small camera-frame correction steps stay linear.

        The result strings deliberately keep the "move_cart" wording because
        ArmInterface attributes completions by that substring in
        /arm/state.result_message.
        """
        if len(pose) != 6:
            return False, "move_cart needs 6 values [x y z rx ry rz]"
        if self.busy:
            return False, "refused: arm busy"

        self.busy = True
        self.cancel_requested = False
        kind = "MoveL" if linear else "MoveCart"
        try:
            target = [float(v) for v in pose]
            corr = getattr(self, '_cmd_corr', None)
            if physical and corr is not None and corr.enabled:
                def _ik(p):
                    ret, q = self._ik_result(self.robot.GetInverseKin(0, list(p), config=-1))
                    return None if (ret != 0 or q is None) else [float(v) for v in q]
                try:
                    cmd, info = corr.pose_for_physical(target, _ik)
                except ValueError as e:
                    rospy.logerr(f"[Arm REAL] move_cart refused: {e}")
                    return False, f"move_cart failed: joint-offset correction — {e}"
                rospy.loginfo(f"[Arm REAL] physical target {[round(v, 2) for v in target]} -> commanded "
                              f"{[round(v, 2) for v in cmd]} (joint offsets: {info['corr_mm']:.1f} mm / "
                              f"{info['corr_deg']:.2f} deg, {info['iters']} IK"
                              + (f", z kept — dz {info['dz_skipped_mm']:+.1f} mm skipped" if info.get('skip_z') else '')
                              + ")")
                target = cmd
            elif physical:
                rospy.logwarn_throttle(30.0, "[Arm REAL] physical target requested but joint offsets are "
                                             "not loaded — sent uncorrected")
            rospy.loginfo(f"[Arm REAL] {kind} → {[round(v, 2) for v in target]} "
                          f"(vel={vel}, acc={acc})")
            if linear:
                ret = self.robot.MoveL(target, TOOL_ID, 0,
                                       vel=float(vel), acc=float(acc))
            else:
                ret = self.robot.MoveCart(target, TOOL_ID, 0,
                                          float(vel), float(acc))
            if ret != 0:
                rospy.logerr(f"[Arm REAL] {kind} failed: {ret}")
                return False, f"move_cart failed: {kind} error {ret}"
            if self.cancel_requested:
                return False, "cancelled"
            return True, "move_cart ok"
        except Exception as e:
            rospy.logerr(f"[Arm REAL] {kind} exception: {e}")
            if 'timed out' in str(e).lower():
                # The RPC gave up but the CONTROLLER is still executing the
                # move. Returning now lets the next command be sent into a
                # moving arm (seen 2026-09-02: a 0 mm "move" then took 14 s
                # because it queued behind the unfinished one). Hold the
                # busy flag until the controller reports the motion done.
                self._wait_motion_done(120.0)
            return False, f"move_cart exception: {e}"
        finally:
            self.busy = False

    def _wait_motion_done(self, timeout_s):
        """Poll GetRobotMotionDone until the controller is idle (or timeout).
        Used after an RPC timeout, when the SDK has lost track of a move
        that is still running."""
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < timeout_s:
            if self.cancel_requested:
                return
            try:
                ret, done = self.robot.GetRobotMotionDone()
                if ret == 0 and int(done) == 1:
                    rospy.logwarn("[Arm REAL] controller reports motion done "
                                  f"after {(rospy.Time.now() - t0).to_sec():.1f}s")
                    return
            except Exception:
                pass
            rospy.sleep(0.2)
        rospy.logerr(f"[Arm REAL] motion still not done after {timeout_s:.0f}s")

    # Axis order matches the Fairino TCP pose vector, which is also the order the
    # UI's six fields are laid out in. Keep them in step.
    JOG_AXES = ('x', 'y', 'z', 'rx', 'ry', 'rz')

    def jog(self, axis, delta, vel=30.0, acc=50.0, max_step=50.0):
        """Move one Cartesian axis by `delta` (mm for x/y/z, deg for rx/ry/rz).

        Reads the CURRENT pose first rather than accumulating onto a cached one:
        a cached target drifts away from reality after any refused or clamped
        step, and the operator holding the button would not see it happen.
        """
        axis = str(axis).lower()
        if axis not in self.JOG_AXES:
            return False, f"unknown axis '{axis}' (expected one of {self.JOG_AXES})"
        try:
            delta = float(delta)
        except (TypeError, ValueError):
            return False, f"jog delta '{delta}' is not a number"

        # Bring-up guard. A typo'd step (a stray zero) is the realistic way to
        # drive the tool into the workpiece with an operator's hand on the
        # button; there is no soft limit below this in the SDK path.
        if abs(delta) > max_step:
            return False, (f"jog {delta} exceeds max_step {max_step} — "
                           "raise ~jog_max_step deliberately if this is intended")
        if self.busy:
            return False, "refused: arm busy"

        current = self.get_tcp_pose()
        if current is None:
            return False, "refused: current TCP pose unreadable"

        target = list(current)
        target[self.JOG_AXES.index(axis)] += delta
        ok, msg = self.move_cart(target, vel=vel, acc=acc)
        if not ok:
            return ok, msg
        return True, f"jog {axis} {delta:+g} ok"

    # Joint names as the UI and the JSON commands spell them, in the order
    # the Fairino joint vector uses. j1 is the base.
    JOINT_NAMES = ('j1', 'j2', 'j3', 'j4', 'j5', 'j6')

    def move_joint(self, joints_deg, vel=30.0, acc=50.0):
        """Absolute joint move: MoveJ to [J1..J6] in degrees.

        Returns (ok, message). Blocks until the SDK call returns. The
        operator's joint-space twin of move_cart (2026-09-21, robot_ui's
        joint control): no IK, no tool frame involved — the target IS the
        joint configuration, so this is the same call a scan_joint_* task
        makes per row and the home pose uses. No reach or collision check
        below this (none exists for MoveJ anywhere in the stack); the
        controller refuses a joint limit with its own error code.
        """
        if len(joints_deg) != 6:
            return False, "move_joint needs 6 values [j1 .. j6]"
        if self.busy:
            return False, "refused: arm busy"

        self.busy = True
        self.cancel_requested = False
        try:
            target = [float(v) for v in joints_deg]
            rospy.loginfo(f"[Arm REAL] MoveJ (joint) → {[round(v, 2) for v in target]} "
                          f"(vel={vel}, acc={acc})")
            ret = self.robot.MoveJ(target, TOOL_ID, 0,
                                   vel=float(vel), acc=float(acc))
            if ret != 0:
                rospy.logerr(f"[Arm REAL] MoveJ (joint) failed: {ret}")
                return False, f"move_joint failed: MoveJ error {ret}"
            if self.cancel_requested:
                return False, "cancelled"
            return True, "move_joint ok"
        except Exception as e:
            rospy.logerr(f"[Arm REAL] MoveJ (joint) exception: {e}")
            if 'timed out' in str(e).lower():
                self._wait_motion_done(120.0)      # same reason as move_cart
            return False, f"move_joint exception: {e}"
        finally:
            self.busy = False

    def jog_joint(self, joint, delta_deg, vel=30.0, acc=50.0, max_step=50.0):
        """Move ONE joint by `delta_deg`; the other five keep their current
        angle. `joint` is 'j1'..'j6' or 1..6. Same rules as the Cartesian
        jog: the CURRENT joints are read first (no accumulated target), the
        step is bounded by max_step, refused while busy.
        """
        name = str(joint).lower()
        if name.isdigit():
            name = 'j' + name
        if name not in self.JOINT_NAMES:
            return False, (f"unknown joint '{joint}' "
                           f"(expected one of {self.JOINT_NAMES} or 1..6)")
        try:
            delta = float(delta_deg)
        except (TypeError, ValueError):
            return False, f"jog delta '{delta_deg}' is not a number"
        if abs(delta) > max_step:
            return False, (f"jog {delta} deg exceeds max_step {max_step} — "
                           "raise ~jog_max_step deliberately if this is intended")
        if self.busy:
            return False, "refused: arm busy"

        current = self.get_joints_deg()
        if current is None:
            return False, "refused: current joint angles unreadable"
        target = list(current)
        target[self.JOINT_NAMES.index(name)] += delta
        ok, msg = self.move_joint(target, vel=vel, acc=acc)
        if not ok:
            return ok, msg
        return True, f"jog {name} {delta:+g} deg ok"

    # --------------------------------------------------
    # PUBLISH / STATUS
    # --------------------------------------------------
    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        rospy.loginfo("[Arm REAL] scan_finished published")

    def is_busy(self):
        return self.busy

    def shutdown(self):
        rospy.loginfo("[Arm REAL] Shutting down...")
        # The camera and the VISION lamp belong to basler_camera_node — it
        # closes the device and darkens the lamp on its own shutdown.
        self.pipeline.shutdown()


if __name__ == "__main__":
    rospy.init_node("arm_controller", anonymous=False)
    model_path = rospy.get_param('~model_path', None)
    controller = ArmController(model_path=model_path)
    rospy.on_shutdown(controller.shutdown)
    rospy.spin()
