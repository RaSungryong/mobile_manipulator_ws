#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plan B — Arm Controller ROS Node (Isaac Sim / MoveIt)
======================================================
For simulation testing only. Uses MoveIt Commander for motion control.
Sensor data (camera, Keyence) comes from Isaac Sim via ROS topics.
Interface is identical to arm_controller_ros.py (real robot).

Subscribers:
  /basler/image_raw      (sensor_msgs/Image)
  keyence/value          (std_msgs/Float32)
  /robot_pose            (robot_msgs/Pose2DWithFlag)
  /arm/scan_command      (std_msgs/String, JSON)
  /arm/cancel            (std_msgs/Bool)

Publishers:
  /scan_finished         (std_msgs/Bool)
  /scan/ra_value         (std_msgs/Float32)
  /scan/point_result     (std_msgs/String, JSON)
  /scan/image            (sensor_msgs/Image)

Prerequisite: MoveIt must be running before this node starts.
  roslaunch fr10v6_vision_251219_moveit_config fr10v6_vision_isaac_execution.launch
"""

import os
import sys
import json
import time
import threading

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
import moveit_commander
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from robot_msgs.msg import Pose2DWithFlag
from cv_bridge import CvBridge

from inference_interface import InferenceInterface

_MOVEIT_GROUP = 'arm'
_REFERENCE_FRAME = 'world'


class ArmControllerRosSim:

    def __init__(self):
        moveit_commander.roscpp_initialize([])
        rospy.init_node('arm_controller_ros', anonymous=False)

        model_path        = rospy.get_param('~model_path',    None)
        # mock_sensors=true: use synthetic frame + skip Keyence when real
        # sensor topics are not publishing (e.g. Isaac Sim sensor not configured)
        self._mock_sensors = rospy.get_param('~mock_sensors', False)

        self._num_samples    = rospy.get_param('~num_samples',           1)
        self._delay_samples  = rospy.get_param('~delay_between_samples', 0.2)
        self._stabilize_time = rospy.get_param('~stabilization_time',    0.5)
        self._save_images    = rospy.get_param('~save_images',           False)
        self._output_dir     = rospy.get_param('~output_dir',            '/tmp/scan_results')

        self._k_tol      = rospy.get_param('~keyence_tol',          0.2)
        self._k_dir      = rospy.get_param('~keyence_dir',          1.0)
        self._k_kp       = rospy.get_param('~keyence_kp',           0.8)
        self._k_steps    = rospy.get_param('~keyence_max_steps',    10)
        self._k_max_step = rospy.get_param('~keyence_max_step_mm',  5.0)

        self._pose_base_z = rospy.get_param('~pose_base_z', -0.835)
        self._pose_mb_z   = rospy.get_param('~pose_mb_z',    0.18)
        self._pose_arm_z  = rospy.get_param('~pose_arm_z',  -1.02)

        self._latest_frame = None
        self._keyence_val  = None
        self._robot_pose   = None
        self._bridge       = CvBridge()

        self._busy   = False
        self._cancel = False

        self._home_rad = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]

        # MoveIt
        self._arm = moveit_commander.MoveGroupCommander(_MOVEIT_GROUP)
        self._arm.set_goal_joint_tolerance(0.01)
        self._arm.set_goal_position_tolerance(0.001)
        self._arm.set_goal_orientation_tolerance(0.05)
        self._arm.allow_replanning(True)
        self._arm.set_planning_time(5.0)
        rospy.loginfo("[ArmRosSim] MoveIt Commander initialized")

        self._inference = InferenceInterface()
        if model_path and not self._inference.load_model(model_path):
            rospy.logerr("[ArmRosSim] ONNX model loading failed")

        rospy.Subscriber('/basler/image_raw', Image,          self._image_cb,   queue_size=1)
        rospy.Subscriber('keyence/value',     Float32,        self._keyence_cb, queue_size=1)
        rospy.Subscriber('/robot_pose',       Pose2DWithFlag, self._pose_cb,    queue_size=1)
        rospy.Subscriber('/arm/scan_command', String,         self._scan_cb,    queue_size=1)
        rospy.Subscriber('/arm/cancel',       Bool,           self._cancel_cb,  queue_size=1)

        self._done_pub   = rospy.Publisher('/scan_finished',     Bool,    queue_size=1)
        self._ra_pub     = rospy.Publisher('/scan/ra_value',     Float32, queue_size=10)
        self._result_pub = rospy.Publisher('/scan/point_result', String,  queue_size=10)
        self._image_pub  = rospy.Publisher('/scan/image',        Image,   queue_size=1)

        self._move_to_home()
        rospy.loginfo("[ArmRosSim] Ready — waiting on /arm/scan_command")

    # --------------------------------------------------
    def _image_cb(self, msg):
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[ArmRosSim] Image decode error: {e}")

    def _keyence_cb(self, msg):
        self._keyence_val = msg.data

    def _pose_cb(self, msg):
        self._robot_pose = msg

    def _cancel_cb(self, msg):
        if msg.data:
            rospy.logwarn("[ArmRosSim] Cancel received")
            self._cancel = True
            try:
                self._arm.stop()
                self._arm.clear_pose_targets()
            except Exception as e:
                rospy.logerr(f"[ArmRosSim] Stop error: {e}")

    def _scan_cb(self, msg):
        if self._busy:
            rospy.logwarn("[ArmRosSim] Busy — scan_command ignored")
            return
        try:
            scan_points = json.loads(msg.data)
        except Exception as e:
            rospy.logerr(f"[ArmRosSim] Invalid scan_command JSON: {e}")
            self._publish_done()
            return
        threading.Thread(target=self._execute_scan, args=(scan_points,), daemon=True).start()

    # --------------------------------------------------
    def _execute_scan(self, scan_points):
        if not scan_points:
            rospy.logwarn("[ArmRosSim] Empty scan_points")
            self._publish_done()
            return

        if any(p['mode'] == 'pose' for p in scan_points):
            if self._robot_pose is None:
                rospy.logerr("[ArmRosSim] pose scan requires /robot_pose — not received yet")
                self._publish_done()
                return

        self._busy   = True
        self._cancel = False

        try:
            for i, p in enumerate(scan_points):
                if self._cancel:
                    rospy.logwarn("[ArmRosSim] Scan cancelled")
                    break

                rospy.loginfo(f"[ArmRosSim] Point {i+1}/{len(scan_points)}")
                speed = p.get('speed', 60)

                if p['mode'] == 'joint':
                    self._exec_joint(p['joints'], speed)
                elif p['mode'] == 'pose':
                    self._exec_pose(p, speed)

                time.sleep(self._stabilize_time)

                if not self._cancel:
                    self._adjust_keyence()
                    time.sleep(0.5)

                if not self._cancel:
                    self._scan_at_point(i)

            if not self._cancel:
                self._move_to_home()

        except Exception as e:
            rospy.logerr(f"[ArmRosSim] Scan exception: {e}")
        finally:
            self._publish_done()
            self._busy = False

    # --------------------------------------------------
    def _adjust_keyence(self):
        if self._mock_sensors:
            rospy.loginfo("[ArmRosSim] Keyence adjustment skipped (mock_sensors=true)")
            return
        rospy.loginfo("[ArmRosSim] Keyence distance adjustment...")

        for step in range(self._k_steps):
            if self._cancel:
                rospy.logwarn("[ArmRosSim] Adjustment cancelled")
                break

            if self._keyence_val is None:
                rospy.logwarn("[ArmRosSim] Keyence value not available — skipping")
                break

            val = self._keyence_val
            if abs(val) <= self._k_tol:
                rospy.loginfo(f"[ArmRosSim] Distance OK: {val:.3f} mm")
                break

            dz_mm = np.clip(val * self._k_dir * self._k_kp,
                            -self._k_max_step, self._k_max_step)
            dz_m = dz_mm / 1000.0   # mm → meters for MoveIt

            current = self._arm.get_current_pose().pose
            quat = [current.orientation.x, current.orientation.y,
                    current.orientation.z, current.orientation.w]
            z_vec = R.from_quat(quat).as_matrix()[:, 2]

            target = PoseStamped()
            target.header.frame_id = _REFERENCE_FRAME
            target.header.stamp = rospy.Time.now()
            target.pose.position.x = current.position.x + z_vec[0] * dz_m
            target.pose.position.y = current.position.y + z_vec[1] * dz_m
            target.pose.position.z = current.position.z + z_vec[2] * dz_m
            target.pose.orientation = current.orientation

            rospy.loginfo(f"  [Adj {step+1}/{self._k_steps}] val={val:.3f} dz={dz_mm:.3f} mm")
            self._arm.set_pose_target(target)
            self._arm.go(wait=True)
            self._arm.clear_pose_targets()
            rospy.sleep(1.0)

        else:
            rospy.logwarn(f"[ArmRosSim] Keyence not converged in {self._k_steps} steps. "
                          f"Last: {self._keyence_val:.3f} mm")

    # --------------------------------------------------
    def _scan_at_point(self, point_id):
        rospy.loginfo(f"[ArmRosSim] Scanning point {point_id} ({self._num_samples} samples)")
        results = []
        images  = []

        for s in range(self._num_samples):
            if self._cancel:
                break

            frame = self._latest_frame
            if frame is None:
                if self._mock_sensors:
                    # Synthetic 224x224 gray frame for pipeline testing
                    frame = np.full((224, 224, 3), 128, dtype=np.uint8)
                    rospy.loginfo(f"  [ArmRosSim] Sample {s+1}: using mock frame")
                else:
                    rospy.logwarn(f"  [ArmRosSim] Sample {s+1}: no frame yet")
                    continue

            frame = frame.copy()
            t0 = time.time()
            ra = self._inference.infer(frame)
            elapsed_ms = (time.time() - t0) * 1000

            if ra is not None:
                results.append({'point_id': point_id, 'sample_id': s+1,
                                 'ra_value': ra, 'infer_ms': elapsed_ms})
                images.append(frame)
                self._ra_pub.publish(Float32(data=ra))
                rospy.loginfo(f"  Sample {s+1}: Ra={ra:.4f}  ({elapsed_ms:.1f} ms)")
                try:
                    self._image_pub.publish(self._bridge.cv2_to_imgmsg(frame, 'bgr8'))
                except Exception as e:
                    rospy.logwarn(f"  [ArmRosSim] Image publish failed: {e}")

            if s < self._num_samples - 1:
                time.sleep(self._delay_samples)

        if results:
            ra_vals = [r['ra_value'] for r in results]
            summary = {'point_id': point_id, 'num_samples': len(results),
                       'ra_mean': float(np.mean(ra_vals)), 'ra_std': float(np.std(ra_vals)),
                       'ra_min': float(np.min(ra_vals)),   'ra_max': float(np.max(ra_vals)),
                       'ra_values': ra_vals}
            self._result_pub.publish(String(data=json.dumps(summary)))

            if self._save_images and images:
                os.makedirs(self._output_dir, exist_ok=True)
                for idx, img in enumerate(images):
                    fname = f"point_{point_id}_sample_{idx+1}_ra_{ra_vals[idx]:.4f}.png"
                    cv2.imwrite(os.path.join(self._output_dir, fname), img)
        else:
            rospy.logerr(f"[ArmRosSim] No valid samples at point {point_id}")

    # --------------------------------------------------
    def _move_to_home(self):
        rospy.loginfo("[ArmRosSim] Moving to home...")
        try:
            self._arm.set_joint_value_target(self._home_rad)
            self._arm.go(wait=True)
            self._arm.clear_pose_targets()
        except Exception as e:
            rospy.logerr(f"[ArmRosSim] Home exception: {e}")

    def _exec_joint(self, joints_rad, speed=60):
        if len(joints_rad) != 6:
            rospy.logerr("[ArmRosSim] Joint goal must have 6 values")
            return
        rospy.loginfo(f"[ArmRosSim] MoveIt joint → {[f'{np.degrees(j):.2f}' for j in joints_rad]}")
        self._arm.set_max_velocity_scaling_factor(min(speed / 100.0, 1.0))
        self._arm.set_joint_value_target(joints_rad)
        self._arm.go(wait=True)
        self._arm.clear_pose_targets()

    def _exec_pose(self, p, speed=60):
        pos, rpy_deg = self._transform_pose(p, self._robot_pose)
        quat = R.from_euler('xyz', rpy_deg, degrees=True).as_quat()   # [x,y,z,w]

        target = PoseStamped()
        target.header.frame_id = _REFERENCE_FRAME
        target.header.stamp = rospy.Time.now()
        target.pose.position.x    = float(pos[0])
        target.pose.position.y    = float(pos[1])
        target.pose.position.z    = float(pos[2])
        target.pose.orientation.x = quat[0]
        target.pose.orientation.y = quat[1]
        target.pose.orientation.z = quat[2]
        target.pose.orientation.w = quat[3]

        rospy.loginfo(f"[ArmRosSim] MoveIt pose → {[f'{v:.3f}' for v in pos]}")
        self._arm.set_max_velocity_scaling_factor(min(speed / 100.0, 1.0))
        self._arm.set_pose_target(target)
        self._arm.go(wait=True)
        self._arm.clear_pose_targets()

    def _transform_pose(self, g, msg):
        base_x, base_y = msg.x, msg.y
        dx = g['x'] - base_x
        dy = g['y'] - base_y

        R_base = R.from_euler('y', np.pi)
        R_yaw  = R.from_euler('z', msg.theta)
        R_mb   = (R_base * R_yaw).as_matrix()

        T_mb = self._T(R_mb, [base_x, base_y, self._pose_mb_z])
        T_ba = self._T(R.from_euler('z', np.pi).as_matrix(), [0, 0, self._pose_arm_z])
        T    = T_mb @ T_ba

        r_tag  = R.from_euler('xyz', [g['rx'], g['ry'], g['rz']])
        r_goal = R.from_matrix(T[:3, :3]).inv() * r_tag
        rpy_deg = r_goal.as_euler('xyz', degrees=True)

        dz = g['z'] - self._pose_base_z
        return np.array([dx, -dy, -dz]), rpy_deg

    @staticmethod
    def _T(Rm, t):
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3]  = t
        return T

    # --------------------------------------------------
    def _publish_done(self):
        self._done_pub.publish(Bool(data=True))
        rospy.loginfo("[ArmRosSim] /scan_finished published")

    def shutdown(self):
        rospy.loginfo("[ArmRosSim] Shutting down...")
        try:
            self._arm.stop()
            self._arm.clear_pose_targets()
        except Exception:
            pass


if __name__ == '__main__':
    node = ArmControllerRosSim()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
