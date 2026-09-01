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

import time
import numpy as np

import rospy
from std_msgs.msg import Bool, Float32
from robot_msgs.msg import Pose2DWithFlag
from scipy.spatial.transform import Rotation as R

from apriltag_nav import paths
from apriltag_nav.paths import load_yaml_block
from apriltag_nav.arm_transform import transform_world_to_arm
from apriltag_nav.scan_pipeline import RaScanPipeline
from apriltag_nav.scan_results import ScanResultWriter

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

        # ---------- Fairino ----------
        rospy.loginfo("[Arm REAL] Connecting to Fairino robot...")
        self.robot = Robot.RPC(robot_ip)
        time.sleep(0.5)
        ret = self.robot.RobotEnable(1)
        rospy.loginfo(f"[Arm REAL] RobotEnable(1) → {ret}")
        time.sleep(1.0)

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
        _keyence_cfg = load_yaml_block('keyence')
        self.current_keyence_val = None
        self.keyence_tol          = rospy.get_param('~keyence_tol', _keyence_cfg.get('tolerance_mm', 0.2))       # tolerance (mm)
        self.keyence_dir          = rospy.get_param('~keyence_dir', 1.0)                                         # direction sign (1.0 or -1.0)
        self.keyence_kp           = rospy.get_param('~keyence_kp',  _keyence_cfg.get('kp', 0.8))                 # proportional gain
        self.keyence_max_steps    = rospy.get_param('~keyence_max_steps', _keyence_cfg.get('max_steps', 10))     # max adjustment iterations
        self.keyence_max_step_mm  = rospy.get_param('~keyence_max_step_mm', _keyence_cfg.get('max_step_mm', 1.0))                     # single-step move limit (mm)
        self.keyence_activate_threshold = rospy.get_param('~keyence_activate_threshold', _keyence_cfg.get('activate_threshold', 5.0)) # only adjust when |perp| < this (mm)
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
        """Cache the latest Keyence sensor reading."""
        self.current_keyence_val = msg.data

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

        try:
            for i, p in enumerate(scan_points):

                if self.cancel_requested:
                    rospy.logwarn("[Arm REAL] Scan cancelled")
                    break

                pid = int(p.get("point_id", i))
                gid = int(p.get("group_id", -1))
                current_csv_path = p.get("csv_path", "")

                rospy.loginfo(f"[Arm REAL] Execute scan point {i+1}/{len(scan_points)}")

                # Pre-open the camera before the arm starts moving, so the
                # device-open latency runs in parallel with motion + Keyence
                # adjustment instead of serially at capture time.
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
                    results.append(entry)
                    if current_csv_path:
                        self.results_writer.save(current_csv_path, results)
                    continue

                # Wait for arm to stabilize at target point
                rospy.loginfo(f"[Arm REAL] Stabilizing {self.stabilization_time}s ...")
                time.sleep(self.stabilization_time)

                # Keyence closed-loop distance adjustment before scan
                if not self.cancel_requested:
                    self._adjust_distance_to_surface()
                    time.sleep(0.5)

                # Capture and infer (delegated — nothing here moves the arm)
                if not self.cancel_requested:
                    scan_result = self.pipeline.scan_point(
                        point_id=pid,
                        cancelled=lambda: self.cancel_requested,
                    )
                    if scan_result:
                        entry["ra_mean"]     = scan_result["ra_mean"]
                        entry["ra_std"]      = scan_result["ra_std"]
                        entry["ra_min"]      = scan_result["ra_min"]
                        entry["ra_max"]      = scan_result["ra_max"]
                        entry["num_samples"] = scan_result["num_samples"]

                results.append(entry)

                # Incremental CSV save (preserves results even if cancelled mid-scan)
                if current_csv_path:
                    try:
                        self.results_writer.save(current_csv_path, results)
                    except Exception as csv_err:
                        rospy.logerr(f"[Arm REAL] Failed to save results: {csv_err}")

            if not self.cancel_requested:
                rospy.loginfo("[Arm REAL] Scan finished → Home")
                self.move_to_home()

        except Exception as e:
            rospy.logerr(f"[Arm REAL] Scan exception: {e}")

        finally:
            # Scan over (finished or cancelled) — let the camera close now
            # rather than idling warm for idle_close_sec.
            self.pipeline.release()
            # Always publish done so task_executor never hangs
            self.publish_done()
            self.busy = False

    # --------------------------------------------------
    # KEYENCE DISTANCE ADJUSTMENT
    # --------------------------------------------------
    def _adjust_distance_to_surface(self):
        """
        Translate the tool along its Z-axis using Keyence feedback
        until the reading reaches 0 within tolerance. Orientation is preserved.
        """
        rospy.loginfo("[Arm REAL] Adjusting tool distance using Keyence sensor...")

        for step in range(self.keyence_max_steps):
            if self.cancel_requested:
                rospy.logwarn("[Arm REAL] Adjustment cancelled.")
                break

            if self.current_keyence_val is None:
                rospy.logwarn("[Arm REAL] Keyence value NOT available, skipping adjustment.")
                break

            val = self.current_keyence_val

            # Project the beam reading onto the surface normal FIRST, then use
            # that everywhere. keyence_tol / keyence_activate_threshold /
            # keyence_max_step_mm are all PERPENDICULAR standoff millimetres —
            # the physical quantity the scan cares about — so they must not be
            # compared against the raw along-the-beam reading, which is larger
            # by 1/cos (at 42.6 deg, 36% larger).
            perp = val * self._keyence_cos

            if abs(perp) >= self.keyence_activate_threshold:
                rospy.logwarn(
                    f"[Arm REAL] Keyence {val:.3f} mm (beam) = {perp:.3f} mm "
                    f"perpendicular >= {self.keyence_activate_threshold} mm, "
                    "skipping adjustment."
                )
                break

            if abs(perp) <= self.keyence_tol:
                rospy.loginfo(
                    f"[Arm REAL] Distance reached target. Current: {val:.3f} mm "
                    f"(beam) = {perp:.3f} mm perpendicular")
                break

            # Displacement along tool Z-axis (mm). `perp` is already the
            # perpendicular error, so this is a plain proportional step in the
            # same units, clamped by keyence_max_step_mm (also perpendicular mm).
            # keyence_dir carries the sensor's polarity: it must be -sign(k),
            # where k = d(reading)/d(toolZ) — see tools/measure_keyence_angle.py.
            dz = np.clip(
                perp * self.keyence_dir * self.keyence_kp,
                -self.keyence_max_step_mm,
                self.keyence_max_step_mm
            )

            ret, pose = self.robot.GetActualTCPPose()
            if ret != 0:
                rospy.logerr(f"[Arm REAL] GetActualTCPPose failed: {ret}")
                break

            x, y, z, rx, ry, rz = pose

            # Compute tool Z-axis direction in robot base frame
            # Fairino uses degrees for Euler angles
            r = R.from_euler('xyz', [rx, ry, rz], degrees=True)
            # scipy compat: >=1.4 as_matrix(), 1.3 as_dcm()
            r_mat = r.as_matrix() if hasattr(r, 'as_matrix') else r.as_dcm()
            z_vec = r_mat[:, 2]  # third column of rotation matrix

            # Apply offset while keeping orientation (rx, ry, rz) unchanged
            new_pose = [
                x + z_vec[0] * dz,
                y + z_vec[1] * dz,
                z + z_vec[2] * dz,
                rx, ry, rz
            ]

            rospy.loginfo(
                f"  -> [Adjust {step+1}/{self.keyence_max_steps}] "
                f"Sensor: {val:.3f} mm (beam) -> {perp:.3f} mm perpendicular. "
                f"Shifting tool Z by {dz:.3f} mm "
                f"(Kp={self.keyence_kp}, dir={self.keyence_dir}, "
                f"beam={self.keyence_beam_angle_deg} deg)"
            )

            # Use low speed for fine distance adjustment
            self.robot.SetSpeed(5)
            # MoveL preserves orientation during linear motion
            ret = self.robot.MoveL(new_pose, tool=TOOL_ID, user=0)
            if ret != 0:
                rospy.logerr(f"[Arm REAL] MoveL failed during adjustment: {ret}")
                break

            # Wait for motion to complete and sensor to refresh
            time.sleep(1.0)
        else:
            rospy.logwarn(
                f"[Arm REAL] Failed to reach 0 within {self.keyence_max_steps} steps. "
                f"Last val: {self.current_keyence_val:.3f} mm."
            )

    # --------------------------------------------------
    # JOINT MOTION
    # --------------------------------------------------
    def _exec_joint(self, joints_rad):
        if len(joints_rad) != 6:
            rospy.logerr("[Arm REAL] Joint goal must have 6 values")
            return
        joints_deg = [np.degrees(j) for j in joints_rad]
        rospy.loginfo(f"[Arm REAL] MoveJ (deg) → {joints_deg}")
        ret = self.robot.MoveJ(joints_deg, tool=TOOL_ID, user=0)
        if ret != 0:
            rospy.logerr(f"[Arm REAL] MoveJ failed: {ret}")

    # --------------------------------------------------
    # POSE MOTION (IK → MoveJ)
    # --------------------------------------------------
    def _exec_pose(self, p):
        pos, rpy = transform_world_to_arm(p, self.current_pose_msg)
        target = [pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]]
        rospy.loginfo(f"[Arm REAL] IK target: {target}")

        q0 = p.get("q0")
        if q0 is not None:
            # Use GetInverseKinRef with paired joint CSV as reference
            q0_deg = [np.degrees(j) for j in q0]
            rospy.loginfo(f"[Arm REAL] IK with q0 ref: {[round(d,1) for d in q0_deg]}")
            ret, joints = self.robot.GetInverseKinRef(0, target, q0_deg)
        else:
            ret, joints = self.robot.GetInverseKin(0, target, config=-1)

        if ret != 0:
            rospy.logerr(f"[Arm REAL] IK failed: {ret}")
            return
        rospy.loginfo(f"[Arm REAL] IK → joints: {joints}")
        ret = self.robot.MoveJ(joints, tool=TOOL_ID, user=0)
        if ret != 0:
            rospy.logerr(f"[Arm REAL] MoveJ failed: {ret}")

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
    #   * MoveCart, not MoveJ. The operator is watching Cartesian axes; solving
    #     IK and driving joints can swing the elbow through a different arc for
    #     the same endpoint.

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

    def move_cart(self, pose, vel=30.0, acc=50.0):
        """Absolute Cartesian move to [x,y,z mm, rx,ry,rz deg].

        Returns (ok, message). Blocks until the SDK call returns.
        """
        if len(pose) != 6:
            return False, "move_cart needs 6 values [x y z rx ry rz]"
        if self.busy:
            return False, "refused: arm busy"

        self.busy = True
        self.cancel_requested = False
        try:
            target = [float(v) for v in pose]
            rospy.loginfo(f"[Arm REAL] MoveCart → {[round(v, 2) for v in target]} "
                          f"(vel={vel}, acc={acc})")
            ret = self.robot.MoveCart(target, TOOL_ID, 0, float(vel), float(acc))
            if ret != 0:
                rospy.logerr(f"[Arm REAL] MoveCart failed: {ret}")
                return False, f"MoveCart error {ret}"
            if self.cancel_requested:
                return False, "cancelled"
            return True, "move_cart ok"
        except Exception as e:
            rospy.logerr(f"[Arm REAL] MoveCart exception: {e}")
            return False, f"exception: {e}"
        finally:
            self.busy = False

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
