#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import time
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

import rospy
from std_msgs.msg import Bool, Float32, String
from robot_msgs.msg import Pose2DWithFlag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from camera_interface import CameraInterface
from inference_interface import InferenceInterface

# ================= Fairino SDK =================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FAIRINO_PATH = os.path.join(
    BASE_DIR,
    "../../fairino_sdk/fairino-python-sdk/Linux"
)
sys.path.append(FAIRINO_PATH)
from fairino import Robot


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

        # ---------- Fairino ----------
        rospy.loginfo("[Arm REAL] Connecting to Fairino robot...")
        self.robot = Robot.RPC(robot_ip)
        time.sleep(0.5)
        ret = self.robot.RobotEnable(1)
        rospy.loginfo(f"[Arm REAL] RobotEnable(1) → {ret}")
        time.sleep(1.0)

        # ---------- Home ----------
        self.home_joint_positions = [
            -1.5708, -1.5708, 1.5708,
            -1.5708, -1.5708, 0.0
        ]
        # Pre-compute degrees once to avoid repeated conversion in move_to_home
        self._home_joints_deg = [np.degrees(j) for j in self.home_joint_positions]

        # ---------- Camera ----------
        self.camera = CameraInterface()

        # ---------- Inference ----------
        self.inference = InferenceInterface()

        # ---------- Scan parameters ----------
        self.num_samples           = rospy.get_param('~num_samples', 1)
        self.delay_between_samples = rospy.get_param('~delay_between_samples', 0.2)
        self.stabilization_time    = rospy.get_param('~stabilization_time', 0.5)
        self.save_images           = rospy.get_param('~save_images', False)
        self.output_dir            = rospy.get_param('~output_dir', '/tmp/scan_results')

        # ---------- Keyence distance alignment parameters ----------
        self.current_keyence_val = None
        self.keyence_tol          = rospy.get_param('~keyence_tol', 0.2)       # tolerance (mm)
        self.keyence_dir          = rospy.get_param('~keyence_dir', 1.0)       # direction sign (1.0 or -1.0)
        self.keyence_kp           = rospy.get_param('~keyence_kp', 0.8)        # proportional gain
        self.keyence_max_steps    = rospy.get_param('~keyence_max_steps', 10)  # max adjustment iterations
        self.keyence_max_step_mm  = rospy.get_param('~keyence_max_step_mm', 5.0)  # single-step move limit (mm)
        self.keyence_activate_threshold = rospy.get_param('~keyence_activate_threshold', 5.0)  # only adjust when |val| < this (mm)

        # ---------- Pose transform (calibrated 9-DOF, params loaded in _transform_pose) ----------

        # ---------- Initialize camera & model ----------
        if not self.camera.initialize():
            rospy.logerr("[Arm REAL] Camera initialization failed")
        else:
            self.camera.start_grabbing()

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
        self.robot.Mode(1)
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
            ret = self.robot.MoveJ(self._home_joints_deg, tool=0, user=0)
            if ret == 14:
                rospy.logwarn("[Arm REAL] Joint cmd point error (14), resetting and retrying...")
                self.robot.ResetAllError()
                time.sleep(0.5)
                ret = self.robot.MoveJ(self._home_joints_deg, tool=0, user=0)
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

        self.busy = True
        self.cancel_requested = False

        try:
            for i, p in enumerate(scan_points):

                if self.cancel_requested:
                    rospy.logwarn("[Arm REAL] Scan cancelled")
                    break

                rospy.loginfo(f"[Arm REAL] Execute scan point {i+1}/{len(scan_points)}")

                speed = int(p.get("speed", 60))
                self.robot.SetSpeed(speed)

                if p["mode"] == "joint":
                    self._exec_joint(p["joints"])
                elif p["mode"] == "pose":
                    self._exec_pose(p)

                # Wait for arm to stabilize at target point
                rospy.loginfo(f"[Arm REAL] Stabilizing {self.stabilization_time}s ...")
                time.sleep(self.stabilization_time)

                # Keyence closed-loop distance adjustment before scan
                if not self.cancel_requested:
                    self._adjust_distance_to_surface()
                    time.sleep(0.5)

                # Capture and infer
                if not self.cancel_requested:
                    self._scan_at_current_position(point_id=i)

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
            z_vec = r.as_matrix()[:, 2]  # third column of rotation matrix

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
            ret = self.robot.MoveL(new_pose, tool=0, user=0)
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
    def _scan_at_current_position(self, point_id):
        rospy.loginfo(f"[Scan] Point {point_id} — {self.num_samples} sample(s)")

        results = []
        images  = []

        for s in range(self.num_samples):
            if self.cancel_requested:
                break

            frame = self.camera.grab_frame()
            if frame is None:
                rospy.logwarn(f"  [Scan] Sample {s+1} capture failed")
                continue

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
                    f"  Sample {s+1}/{self.num_samples}: "
                    f"Ra={ra_value:.4f}  ({infer_ms:.1f}ms)"
                )

                try:
                    self.image_pub.publish(
                        self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    )
                except Exception as e:
                    rospy.logwarn(f"  [Scan] Image publish failed: {e}")

            if s < self.num_samples - 1:
                time.sleep(self.delay_between_samples)

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
        else:
            rospy.logerr(f"[Scan] No valid samples at point {point_id}")

    # --------------------------------------------------
    # JOINT MOTION
    # --------------------------------------------------
    def _exec_joint(self, joints_rad):
        if len(joints_rad) != 6:
            rospy.logerr("[Arm REAL] Joint goal must have 6 values")
            return
        joints_deg = [np.degrees(j) for j in joints_rad]
        rospy.loginfo(f"[Arm REAL] MoveJ (deg) → {joints_deg}")
        ret = self.robot.MoveJ(joints_deg, tool=0, user=0)
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
        ret = self.robot.MoveJ(joints, tool=0, user=0)
        if ret != 0:
            rospy.logerr(f"[Arm REAL] MoveJ failed: {ret}")

    # --------------------------------------------------
    # POSE TRANSFORM (Isaac Sim world → arm base frame, calibrated 9-DOF)
    # --------------------------------------------------
    def _transform_pose(self, g, msg):
        """
        Transform goal pose from Isaac Sim world frame to arm base_link frame.
        Returns position in mm and orientation in degrees (for Fairino SDK).

        Calibrated from paired joint/pose CSV data (655 points, 4 tag groups).
        Position accuracy ~16 mm RMS, orientation ~5 deg mean.
        """
        # --- Calibrated parameters (9-DOF) ---
        body_off_x = rospy.get_param('~arm_body_offset_x', -0.166715)
        body_off_y = rospy.get_param('~arm_body_offset_y', -0.254772)
        arm_base_z = rospy.get_param('~arm_base_z', 0.974167)
        tilt_x     = rospy.get_param('~arm_tilt_x', 0.054898)
        tilt_y     = rospy.get_param('~arm_tilt_y', 0.017894)
        h_bias     = rospy.get_param('~arm_heading_bias', 0.014368)
        ori_corr_x = rospy.get_param('~arm_ori_corr_x', -0.095520)
        ori_corr_y = rospy.get_param('~arm_ori_corr_y', -0.052944)
        ori_corr_z = rospy.get_param('~arm_ori_corr_z', -0.008688)

        R_corr = R.from_euler('xyz', [ori_corr_x, ori_corr_y, ori_corr_z]).as_matrix()

        # robot_pose: manipulator frame → Isaac world frame
        isaac_x = -msg.y
        isaac_y = -msg.x
        theta_rad = np.radians(msg.theta) + h_bias

        # Rotate body-frame arm offset into Isaac world frame
        c, s = np.cos(theta_rad), np.sin(theta_rad)
        world_off_x = c * body_off_x - s * body_off_y
        world_off_y = s * body_off_x + c * body_off_y

        # Arm base origin in Isaac world
        arm_world = np.array([isaac_x + world_off_x,
                              isaac_y + world_off_y,
                              arm_base_z])

        # R_aw: Isaac world → arm base_link = Rz(theta) · Ry(tilt_y) · Rx(tilt_x)
        R_aw = (R.from_euler('z', theta_rad)
                * R.from_euler('y', tilt_y)
                * R.from_euler('x', tilt_x)).as_matrix()

        t_aw = -R_aw @ arm_world
        T_aw = self._T(R_aw, t_aw)
        R_aw_rot = R.from_matrix(R_aw)

        # Position: world → arm base_link, then meters → mm for Fairino
        p_world = np.array([g["x"], g["y"], g["z"], 1.0])
        p_arm = (T_aw @ p_world)[:3]
        pos_mm = p_arm * 1000.0

        # Orientation: CSV ZYX euler (rad) → R_corr → R_aw → degrees
        r_csv = R.from_euler('zyx', [g["rx"], g["ry"], g["rz"]])
        r_arm = R_aw_rot * R.from_matrix(R_corr @ r_csv.as_matrix())
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
        self.camera.stop_grabbing()
        self.camera.close()


if __name__ == "__main__":
    rospy.init_node("arm_controller", anonymous=False)
    model_path = rospy.get_param('~model_path', None)
    controller = ArmController(model_path=model_path)
    rospy.on_shutdown(controller.shutdown)
    rospy.spin()
