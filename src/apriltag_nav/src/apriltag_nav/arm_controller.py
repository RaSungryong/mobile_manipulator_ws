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
current_pose_msg / publish_done / is_busy / shutdown. arm_controller_node.py
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
        self.keyence_max_step_mm  = rospy.get_param('~keyence_max_step_mm', 5.0)                                 # single-step move limit (mm)
        self.keyence_activate_threshold = rospy.get_param('~keyence_activate_threshold', 5.0)                    # only adjust when |val| < this (mm)

        # ---------- ROS ----------
        rospy.Subscriber("/robot_pose", Pose2DWithFlag, self.pose_cb, queue_size=1)
        rospy.Subscriber("keyence/value", Float32, self.keyence_cb, queue_size=1)
        self.done_pub = rospy.Publisher("/scan_finished", Bool, queue_size=1)

        rospy.loginfo("[Arm REAL] Move to Home (init)")
        self.robot.ResetAllError()
        time.sleep(0.3)
        self.robot.Mode(0)
        time.sleep(0.5)
        self.move_to_home()

        rospy.loginfo("[ArmController REAL] Ready")

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
        rospy.logwarn("[Arm REAL] CANCEL requested")
        self.cancel_requested = True
        try:
            self.robot.StopMotion()
        except Exception as e:
            rospy.logerr(f"[Arm REAL] Stop failed: {e}")

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

            if abs(val) >= self.keyence_activate_threshold:
                rospy.logwarn(
                    f"[Arm REAL] Keyence reading {val:.3f} mm >= "
                    f"{self.keyence_activate_threshold} mm, skipping adjustment."
                )
                break

            if abs(val) <= self.keyence_tol:
                rospy.loginfo(f"[Arm REAL] Distance reached target. Current: {val:.3f} mm")
                break

            # Displacement along tool Z-axis (mm):
            # val > 0 → too far → move +Z; val < 0 → too close → move -Z
            # Clamped to sensor measurement range (±keyence_max_step_mm)
            dz = np.clip(
                val * self.keyence_dir * self.keyence_kp,
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
            z_vec = r.as_dcm()[:, 2]  # third column of rotation matrix

            # Apply offset while keeping orientation (rx, ry, rz) unchanged
            new_pose = [
                x + z_vec[0] * dz,
                y + z_vec[1] * dz,
                z + z_vec[2] * dz,
                rx, ry, rz
            ]

            rospy.loginfo(
                f"  -> [Adjust {step+1}/{self.keyence_max_steps}] "
                f"Sensor: {val:.3f} mm. Shifting tool Z by {dz:.3f} mm (Kp={self.keyence_kp})"
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
