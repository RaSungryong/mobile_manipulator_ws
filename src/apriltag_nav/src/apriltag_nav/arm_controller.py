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
from apriltag_nav.arm_transform import transform_world_to_arm
from apriltag_nav.lift_height import LiftHeightListener
from apriltag_nav.scan_pipeline import RaScanPipeline
from apriltag_nav.scan_results import ScanResultWriter
from apriltag_nav.keyence_standoff import StandoffConfig, StandoffController

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

        # ---------- Fairino ----------
        rospy.loginfo("[Arm REAL] Connecting to Fairino robot...")
        self.robot = Robot.RPC(robot_ip)
        time.sleep(0.5)
        ret = self.robot.RobotEnable(1)
        rospy.loginfo(f"[Arm REAL] RobotEnable(1) → {ret}")
        time.sleep(1.0)

        # ---------- which tool frame is the controller actually using? ----------
        # Pose-mode CSV rows are VISION-TIP coordinates (robot.yaml
        # arm_calibration.vision_tip_offset_mm, the same numbers
        # tools/set_tool_tcp.py writes as tool 1). GetInverseKin has no tool
        # argument: it solves for the controller's ACTIVE tool frame. On
        # 2026-09-14 that frame was the FLANGE (offset 0), so every tip
        # target put the flange there — 338.7 mm from where the paired joint
        # row put the tip. See _probe_tool_frame / _exec_pose.
        _ac = load_yaml_block('arm_calibration')
        self._tip_offset_mm = np.array(
            _ac.get('vision_tip_offset_mm', [0.0, -253.0, 225.2]), dtype=float)
        self._pose_tip_to_flange = self._probe_tool_frame()

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
            output_dir=rospy.get_param('~output_dir', '/tmp/scan_results'),
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
        # Per-point scan progress for the operator UI (2026-09-14). JSON
        # events, one per phase: start / move / done / failed / finished.
        # Before this the only trace of a failed point was arm_node's rosout
        # and the result CSV — robot_ui showed nothing while 110 IK failures
        # went by.
        self.progress_pub = rospy.Publisher("/arm/scan_progress", String,
                                            queue_size=50)
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
        self.current_pose_msg = msg

    def keyence_cb(self, msg):
        """Cache the latest Keyence sensor reading and count it, so the
        standoff loop can insist on readings that arrived AFTER its last move
        (a cached value that stopped updating must not drive the arm)."""
        with self._keyence_lock:
            self.current_keyence_val = msg.data
            self._keyence_seq += 1

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
                if do_scan:
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
                if frames:
                    infer_q.put((i + 1, pid, gid, entry, frames, current_csv_path))

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
        """Consume (index, pid, gid, entry, frames, csv_path) until None.

        Fills the entry's ra_* fields in place, rewrites the CSV under
        _results_lock and publishes a `result` progress event. Never lets
        an exception escape — a failed inference is recorded on its row
        and the next frame is processed.
        """
        while True:
            item = infer_q.get()
            if item is None:
                return
            index, pid, gid, entry, frames, csv_path = item
            try:
                scan_result = self.pipeline.process(pid, frames)
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

    def _keyence_move_approach(self, approach_mm):
        """Translate the tool along its own Z axis by approach_mm of
        PERPENDICULAR standoff (positive = toward the surface), orientation
        unchanged, then settle so the next reading is taken at rest.
        keyence_dir carries the tool-Z sign: it is -sign(k) with
        k = d(reading)/d(toolZ), so approach = -dir along tool Z."""
        dz = float(approach_mm) * (-self.keyence_dir)

        ret, pose = self.robot.GetActualTCPPose()
        if ret != 0:
            rospy.logerr(f"[Arm REAL] GetActualTCPPose failed: {ret}")
            return False
        x, y, z, rx, ry, rz = pose

        # Tool Z-axis direction in the robot base frame (Fairino: degrees).
        r = R.from_euler('xyz', [rx, ry, rz], degrees=True)
        # scipy compat: >=1.4 as_matrix(), 1.3 as_dcm()
        r_mat = r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()
        z_vec = r_mat[:, 2]
        new_pose = [x + z_vec[0] * dz, y + z_vec[1] * dz, z + z_vec[2] * dz,
                    rx, ry, rz]

        # Low speed for the fine correction; MoveL keeps the orientation and
        # blocks until the motion is done.
        self.robot.SetSpeed(5)
        ret = self.robot.MoveL(new_pose, tool=TOOL_ID, user=0)
        if ret != 0:
            rospy.logerr(f"[Arm REAL] MoveL failed during standoff adjustment: {ret}")
            return False
        self._refresh_live_pose()
        time.sleep(self.keyence_settle_s)
        return True

    def _adjust_distance_to_surface(self):
        """
        Close the standoff loop before a capture: read the Keyence, move the
        tool along its Z axis, repeat until the perpendicular error is inside
        keyence_tol. Returns a keyence_standoff.StandoffResult; the algorithm
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
        )
        ctl = StandoffController(
            cfg,
            read=self._keyence_read_fresh,
            move=self._keyence_move_approach,
            cancelled=lambda: self.cancel_requested,
            log_info=lambda s: rospy.loginfo(f"[Arm REAL] {s}"),
            log_warn=lambda s: rospy.logwarn(f"[Arm REAL] {s}"),
        )
        result = ctl.run()
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
            "run tools/set_tool_tcp.py or fix arm_calibration.vision_tip_offset_mm.")
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

    def _exec_pose(self, p):
        lift_m = self._pose_lift_m()
        pos_tip, rpy = transform_world_to_arm(p, self.current_pose_msg, lift_m)
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

    def move_cart(self, pose, vel=30.0, acc=50.0, linear=True):
        """Absolute Cartesian move to [x,y,z mm, rx,ry,rz deg].

        Returns (ok, message). Blocks until the SDK call returns.

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
