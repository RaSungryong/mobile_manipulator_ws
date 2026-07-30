#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import time
import numpy as np
import cv2
import pandas as pd
from datetime import datetime
from scipy.spatial.transform import Rotation as R

import rospy
from std_msgs.msg import Bool, Float32, String
from robot_msgs.msg import Pose2DWithFlag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from robot_msgs.srv import CaptureImages
from apriltag_nav.inference_interface import InferenceInterface

# ================= Fairino SDK =================
# Resolved via paths.py — this module moved into the python package, so the old
# __file__-relative "../../fairino_sdk" walk no longer lands in the src space.
from apriltag_nav import paths
if not paths.add_fairino_sdk_to_path():
    rospy.logwarn(f"[Arm REAL] Fairino SDK not found at {paths.FAIRINO_SDK_PATH}")
from fairino import Robot

# robot.yaml path for calibration defaults
_CONFIG_PATH = paths.CONFIG_PATH


def _load_yaml_block(block_name):
    """
    Load a top-level block from robot.yaml. Returns {} if the file or
    block is missing so callers can fall back to hardcoded defaults.
    """
    try:
        import yaml
        with open(_CONFIG_PATH, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        return cfg.get(block_name, {}) or {}
    except Exception:
        return {}

# Tool ID registered via set_tool_tcp.py  (vision_tip TCP offset)
# tool=0 → flange (identity), tool=1 → vision_tip
TOOL_ID = 1


class ArmController:
    """
    ArmController (REAL ROBOT)
    =========================
    - Fairino SDK arm control (MoveJ / MoveL / IK)
    - Joint scan and pose scan
    - Keyence closed-loop distance adjustment
    - Basler camera capture + ONNX Ra inference
    - STOP-safe cancel
    """

    def __init__(self, robot_ip="192.168.58.2", model_path=None):

        # ---------- state ----------
        self.busy = False
        self.cancel_requested = False
        self.current_pose_msg = None
        self._scan_meta = {}          # (group_id, point_id) → {x, y, z, csv_path}

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
        _home_cfg = _load_yaml_block('arm_home')
        _home_default = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        self.home_joint_positions = list(
            _home_cfg.get('joints_rad', _home_default)
        )
        # Pre-compute degrees once to avoid repeated conversion in move_to_home
        self._home_joints_deg = [np.degrees(j) for j in self.home_joint_positions]

        # ---------- Camera (via basler_camera_node service) ----------
        # This controller does NOT open the camera and does NOT drive the VISION
        # lamp. basler_camera_node owns both, so the device can stay closed
        # between captures (heat/lifetime/power) and the lamp is lit only while
        # the shutter is open. See that node's docstring.
        _camera_cfg = _load_yaml_block('camera')
        self.capture_service_name = rospy.get_param(
            '~capture_service', _camera_cfg.get('service', '/camera/capture'))
        self.capture_timeout_s = float(rospy.get_param(
            '~capture_timeout_s', _camera_cfg.get('capture_timeout_s', 20.0)))
        self.use_vision_led = bool(rospy.get_param(
            '~use_vision_led', _camera_cfg.get('use_vision_led', True)))
        self._capture_srv = None

        # ---------- Inference ----------
        self.inference = InferenceInterface()

        # No Navifra device handle here: the VISION lamp belongs to
        # basler_camera_node, and the STATUS lamp / lift / battery / e-stop
        # belong to task_executor. This controller only moves the arm.

        # ---------- Scan parameters ----------
        self.num_samples           = rospy.get_param('~num_samples', 1)
        self.delay_between_samples = rospy.get_param('~delay_between_samples', 0.2)
        self.stabilization_time    = rospy.get_param('~stabilization_time', 0.5)
        self.save_images           = rospy.get_param('~save_images', False)
        self.output_dir            = rospy.get_param('~output_dir', '/tmp/scan_results')

        # ---------- Keyence distance alignment parameters ----------
        # Lookup chain: private ROS param > robot.yaml keyence block > hardcoded default.
        _keyence_cfg = _load_yaml_block('keyence')
        self.current_keyence_val = None
        self.keyence_tol          = rospy.get_param('~keyence_tol', _keyence_cfg.get('tolerance_mm', 0.2))       # tolerance (mm)
        self.keyence_dir          = rospy.get_param('~keyence_dir', 1.0)                                         # direction sign (1.0 or -1.0)
        self.keyence_kp           = rospy.get_param('~keyence_kp',  _keyence_cfg.get('kp', 0.8))                 # proportional gain
        self.keyence_max_steps    = rospy.get_param('~keyence_max_steps', _keyence_cfg.get('max_steps', 10))     # max adjustment iterations
        self.keyence_max_step_mm  = rospy.get_param('~keyence_max_step_mm', 5.0)                                 # single-step move limit (mm)
        self.keyence_activate_threshold = rospy.get_param('~keyence_activate_threshold', 5.0)                    # only adjust when |val| < this (mm)

        # ---------- Pose transform (calibrated 9-DOF, params loaded in _transform_pose) ----------

        # ---------- Model ----------
        if model_path and not self.inference.load_model(model_path):
            rospy.logerr("[Arm REAL] Model loading failed")

        # ---------- ROS ----------
        rospy.Subscriber("/robot_pose", Pose2DWithFlag, self.pose_cb, queue_size=1)
        rospy.Subscriber("keyence/value", Float32, self.keyence_cb, queue_size=1)

        self.done_pub   = rospy.Publisher("/scan_finished",     Bool,    queue_size=1)
        self.ra_pub     = rospy.Publisher("/scan/ra_value",     Float32, queue_size=10)
        self.result_pub = rospy.Publisher("/scan/point_result", String,  queue_size=10)
        self.image_pub  = rospy.Publisher("/scan/image",        Image,   queue_size=1)

        self.bridge = CvBridge()

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

        # Build per-point metadata (group_id, point_id) → {x, y, z, csv_path}
        # Used to seed the CSV on first write
        self._scan_meta = {}
        for i, p in enumerate(scan_points):
            key = (int(p.get("group_id", -1)), int(p.get("point_id", i)))
            self._scan_meta[key] = {
                "x":        float(p["x"]) if "x" in p else None,
                "y":        float(p["y"]) if "y" in p else None,
                "z":        float(p["z"]) if "z" in p else None,
                "csv_path": p.get("csv_path", ""),
            }

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
                        self._save_results_to_csv(current_csv_path, results)
                    continue

                # Wait for arm to stabilize at target point
                rospy.loginfo(f"[Arm REAL] Stabilizing {self.stabilization_time}s ...")
                time.sleep(self.stabilization_time)

                # Keyence closed-loop distance adjustment before scan
                if not self.cancel_requested:
                    self._adjust_distance_to_surface()
                    time.sleep(0.5)

                # Capture and infer
                if not self.cancel_requested:
                    scan_result = self._scan_at_current_position(point_id=pid)
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
                        self._save_results_to_csv(current_csv_path, results)
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
    # SCAN AT CURRENT POSITION
    # --------------------------------------------------
    def _capture_frames(self):
        """Request a burst of frames from basler_camera_node.

        Returns a list of BGR ndarrays (possibly empty). The camera node keeps
        the device closed between calls and brackets the grab with the VISION
        lamp, so there is nothing to switch on or off here.
        """
        if self._capture_srv is None:
            try:
                rospy.wait_for_service(self.capture_service_name,
                                       timeout=self.capture_timeout_s)
                self._capture_srv = rospy.ServiceProxy(
                    self.capture_service_name, CaptureImages)
            except rospy.ROSException:
                rospy.logerr(
                    f"[Scan] Camera service {self.capture_service_name} "
                    "unavailable — is basler_camera_node running?")
                return []

        try:
            resp = self._capture_srv(
                num_samples=int(self.num_samples),
                delay_between_s=float(self.delay_between_samples),
                use_vision_led=bool(self.use_vision_led),
            )
        except rospy.ServiceException as e:
            rospy.logerr(f"[Scan] Camera capture call failed: {e}")
            # Force a re-resolve in case the node restarted.
            self._capture_srv = None
            return []

        if not resp.success:
            rospy.logwarn(f"[Scan] Camera capture unsuccessful: {resp.message}")
            return []

        frames = []
        for img in resp.images:
            try:
                frames.append(self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8'))
            except Exception as e:
                rospy.logwarn(f"[Scan] Image decode failed: {e}")
        return frames

    def _scan_at_current_position(self, point_id):
        rospy.loginfo(f"[Scan] Point {point_id} — {self.num_samples} sample(s)")

        results = []
        images  = []

        if self.cancel_requested:
            return None

        # One service call covers the whole burst: the camera node opens the
        # device, lights the lamp, grabs every sample, then darkens and releases.
        frames = self._capture_frames()
        if not frames:
            rospy.logwarn(f"  [Scan] Point {point_id}: no frames captured")

        for s, frame in enumerate(frames):
            if self.cancel_requested:
                break

            t0 = time.time()
            ra_value = self.inference.infer(frame)
            infer_ms = (time.time() - t0) * 1000

            if ra_value is not None:
                results.append({
                    'point_id':  point_id,
                    'sample_id': s + 1,
                    'ra_value':  ra_value,
                    'infer_ms':  infer_ms,
                })
                images.append(frame)

                self.ra_pub.publish(Float32(ra_value))
                rospy.loginfo(
                    f"  Sample {s+1}/{len(frames)}: "
                    f"Ra={ra_value:.4f}  ({infer_ms:.1f}ms)"
                )

                try:
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    )
                except Exception as e:
                    rospy.logwarn(f"  [Scan] Image publish failed: {e}")

        if results:
            ra_values = [r['ra_value'] for r in results]
            scan_result = {
                'point_id':    point_id,
                'num_samples': len(results),
                'ra_mean':     float(np.mean(ra_values)),
                'ra_std':      float(np.std(ra_values)),
                'ra_min':      float(np.min(ra_values)),
                'ra_max':      float(np.max(ra_values)),
                'ra_values':   ra_values,
            }
            self.result_pub.publish(json.dumps(scan_result))

            if self.save_images and images:
                os.makedirs(self.output_dir, exist_ok=True)
                for idx, img in enumerate(images):
                    fname = (
                        f"point_{point_id}_sample_{idx+1}"
                        f"_ra_{ra_values[idx]:.4f}.png"
                    )
                    cv2.imwrite(os.path.join(self.output_dir, fname), img)

            return scan_result
        else:
            rospy.logerr(f"[Scan] No valid samples at point {point_id}")
            return None

    # --------------------------------------------------
    # INCREMENTAL CSV SAVE
    # --------------------------------------------------
    def _save_results_to_csv(self, csv_path, results):
        """
        Persist per-point results to csv_path incrementally.

        On first call the file is seeded from self._scan_meta so every queued
        scan point gets a row (with x/y/z populated). Subsequent calls update
        rows in place.  13-column schema (see CLAUDE.md "Task Commands"):
            group_id, point_id, x, y, z,
            ra_mean, ra_std, ra_min, ra_max, num_samples,
            success, execution_message, validated_at
        """
        if not csv_path:
            return

        cols = ['group_id', 'point_id', 'x', 'y', 'z',
                'ra_mean', 'ra_std', 'ra_min', 'ra_max', 'num_samples',
                'success', 'execution_message', 'validated_at']

        if os.path.exists(csv_path):
            df = pd.read_csv(csv_path)
            for c in cols:
                if c not in df.columns:
                    df[c] = None
            # Add rows for new scan points (current group) not yet in the CSV
            existing_keys = set()
            for _, row in df[['group_id', 'point_id']].iterrows():
                try:
                    existing_keys.add((int(row['group_id']), int(row['point_id'])))
                except (TypeError, ValueError):
                    pass
            new_rows = [{
                'group_id': gid, 'point_id': pid,
                'x': m['x'], 'y': m['y'], 'z': m['z'],
                'ra_mean': None, 'ra_std': None, 'ra_min': None,
                'ra_max': None, 'num_samples': 0,
                'success': None, 'execution_message': None,
                'validated_at': None,
            } for (gid, pid), m in self._scan_meta.items()
              if (gid, pid) not in existing_keys]
            if new_rows:
                df = pd.concat([df, pd.DataFrame(new_rows, columns=cols)], ignore_index=True)
        else:
            # Seed with all queued points so partial results are visible
            rows = [{
                'group_id': gid, 'point_id': pid,
                'x': m['x'], 'y': m['y'], 'z': m['z'],
                'ra_mean': None, 'ra_std': None, 'ra_min': None,
                'ra_max': None, 'num_samples': 0,
                'success': None, 'execution_message': None,
                'validated_at': None,
            } for (gid, pid), m in self._scan_meta.items()]
            df = pd.DataFrame(rows, columns=cols)

        result_dict = {(r['group_id'], r['point_id']): r for r in results}
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        def _update(row):
            try:
                key = (int(row['group_id']), int(row['point_id']))
            except (TypeError, ValueError):
                return row
            if key in result_dict:
                r = result_dict[key]
                row['success']           = r['success']
                row['execution_message'] = r['execution_message']
                row['ra_mean']           = r['ra_mean']
                row['ra_std']            = r['ra_std']
                row['ra_min']            = r['ra_min']
                row['ra_max']            = r['ra_max']
                row['num_samples']       = r['num_samples']
                if r['success'] is not None and pd.isna(row.get('validated_at')):
                    row['validated_at'] = now
            return row

        df = df.apply(_update, axis=1)
        df.to_csv(csv_path, index=False)
        rospy.loginfo(f"[Arm REAL] Ra map saved → {csv_path}")

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
        pos, rpy = self._transform_pose(p, self.current_pose_msg)
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
    # POSE TRANSFORM (world → arm base frame, 4-DOF physical model)
    # --------------------------------------------------
    def _transform_pose(self, g, msg):
        """
        World frame (CSV pose frame) → arm base_link (4-DOF physical model).
        Returns position in mm and orientation in degrees (for Fairino SDK).

        Derivation: see CLAUDE.md "Coordinate Frames". Defaults validated
        against 328 paired joint/pose rows: mean residual 12 mm, max 27 mm.
        """
        # Calibration defaults match CLAUDE.md / robot.yaml.
        # Mean residual 12 mm, max 27 mm against 328 paired joint/pose rows.
        # Lookup chain: private ROS param > robot.yaml arm_calibration > hardcoded default.
        _calib = _load_yaml_block('arm_calibration')
        body_off_x = rospy.get_param('~arm_body_offset_x', _calib.get('arm_body_offset_x', 0.0))
        body_off_y = rospy.get_param('~arm_body_offset_y', _calib.get('arm_body_offset_y', 0.0))
        body_off_z = rospy.get_param('~arm_base_z',        _calib.get('arm_base_z',        1.0076))
        mount_yaw  = rospy.get_param('~arm_mount_yaw',     _calib.get('arm_mount_yaw',     np.pi))
        tilt_x     = rospy.get_param('~arm_tilt_x',        _calib.get('arm_tilt_x',       -0.02248))
        tilt_y     = rospy.get_param('~arm_tilt_y',        _calib.get('arm_tilt_y',        0.02639))



        x_base = -msg.y
        y_base = -msg.x
        theta  = np.radians(msg.theta)

        c, s = np.cos(theta), np.sin(theta)
        p_A_W = np.array([
            x_base + c * body_off_x - s * body_off_y,
            y_base + s * body_off_x + c * body_off_y,
            body_off_z
        ])

        alpha = theta + mount_yaw
        R_WA = (R.from_euler('z', alpha)
                * R.from_euler('y', tilt_y)
                * R.from_euler('x', tilt_x)).as_dcm()
        R_AW = R_WA.T
        R_AW_rot = R.from_dcm(R_AW)

        # Position: world → arm base_link, then meters → mm for Fairino
        p_W = np.array([g["x"], g["y"], g["z"]])
        p_arm = R_AW @ (p_W - p_A_W)
        pos_mm = p_arm * 1000.0

        # Orientation: CSV ZYX intrinsic → arm frame → degrees for Fairino
        r_csv_W = R.from_euler('zyx', [g["rx"], g["ry"], g["rz"]])
        r_arm = R_AW_rot * r_csv_W
        rpy_deg = r_arm.as_euler("xyz", degrees=True)

        rospy.loginfo(
            f"[Arm REAL] Transform: world=({g['x']:.3f}, {g['y']:.3f}, {g['z']:.3f}) "
            f"-> arm=({pos_mm[0]:.1f}, {pos_mm[1]:.1f}, {pos_mm[2]:.1f}) mm"
        )

        return pos_mm, rpy_deg

    def _T(self, Rm, t):
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

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
        self._capture_srv = None


if __name__ == "__main__":
    rospy.init_node("arm_controller", anonymous=False)
    model_path = rospy.get_param('~model_path', None)
    controller = ArmController(model_path=model_path)
    rospy.on_shutdown(controller.shutdown)
    rospy.spin()
