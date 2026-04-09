#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plan B — Arm Controller ROS Node (Real Robot)
==============================================
Uses Fairino SDK for motion control.
All sensor I/O via ROS topics.

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
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image
from robot_msgs.msg import Pose2DWithFlag
from cv_bridge import CvBridge

from inference_interface import InferenceInterface

_FAIRINO_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'fairino_sdk', 'fairino-python-sdk', 'Linux'
)
if _FAIRINO_PATH not in sys.path:
    sys.path.append(_FAIRINO_PATH)
from fairino import Robot  # type: ignore  (resolved at runtime via sys.path)


class ArmControllerRos:

    def __init__(self):
        rospy.init_node('arm_controller_ros', anonymous=False)

        robot_ip   = rospy.get_param('~robot_ip',   '192.168.58.2')
        model_path = rospy.get_param('~model_path', None)

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

        _home_rad = [-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
        self._home_deg = [np.degrees(j) for j in _home_rad]

        rospy.loginfo("[ArmRos] Connecting to Fairino robot...")
        self._robot = Robot.RPC(robot_ip)
        time.sleep(0.5)
        ret = self._robot.RobotEnable(1)
        rospy.loginfo(f"[ArmRos] RobotEnable(1) → {ret}")
        time.sleep(1.0)
        self._robot.ResetAllError()
        time.sleep(0.3)
        self._robot.Mode(1)
        time.sleep(0.5)

        self._inference = InferenceInterface()
        if model_path and not self._inference.load_model(model_path):
            rospy.logerr("[ArmRos] ONNX model loading failed")

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
        rospy.loginfo("[ArmRos] Ready — waiting on /arm/scan_command")

    # --------------------------------------------------
    def _image_cb(self, msg):
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[ArmRos] Image decode error: {e}")

    def _keyence_cb(self, msg):
        self._keyence_val = msg.data

    def _pose_cb(self, msg):
        self._robot_pose = msg

    def _cancel_cb(self, msg):
        if msg.data:
            rospy.logwarn("[ArmRos] Cancel received")
            self._cancel = True
            try:
                self._robot.StopMotion()
            except Exception as e:
                rospy.logerr(f"[ArmRos] StopMotion error: {e}")

    def _scan_cb(self, msg):
        if self._busy:
            rospy.logwarn("[ArmRos] Busy — scan_command ignored")
            return
        try:
            scan_points = json.loads(msg.data)
        except Exception as e:
            rospy.logerr(f"[ArmRos] Invalid scan_command JSON: {e}")
            self._publish_done()
            return
        threading.Thread(target=self._execute_scan, args=(scan_points,), daemon=True).start()

    # --------------------------------------------------
    def _execute_scan(self, scan_points):
        if not scan_points:
            rospy.logwarn("[ArmRos] Empty scan_points")
            self._publish_done()
            return

        if any(p['mode'] == 'pose' for p in scan_points):
            if self._robot_pose is None:
                rospy.logerr("[ArmRos] pose scan requires /robot_pose — not received yet")
                self._publish_done()
                return

        self._busy   = True
        self._cancel = False

        try:
            for i, p in enumerate(scan_points):
                if self._cancel:
                    rospy.logwarn("[ArmRos] Scan cancelled")
                    break

                rospy.loginfo(f"[ArmRos] Point {i+1}/{len(scan_points)}")
                self._robot.SetSpeed(int(p.get('speed', 60)))

                if p['mode'] == 'joint':
                    self._exec_joint(p['joints'])
                elif p['mode'] == 'pose':
                    self._exec_pose(p)

                time.sleep(self._stabilize_time)

                if not self._cancel:
                    self._adjust_keyence()
                    time.sleep(0.5)

                if not self._cancel:
                    self._scan_at_point(i)

            if not self._cancel:
                self._move_to_home()

        except Exception as e:
            rospy.logerr(f"[ArmRos] Scan exception: {e}")
        finally:
            self._publish_done()
            self._busy = False

    # --------------------------------------------------
    def _adjust_keyence(self):
        rospy.loginfo("[ArmRos] Keyence distance adjustment...")

        for step in range(self._k_steps):
            if self._cancel:
                rospy.logwarn("[ArmRos] Adjustment cancelled")
                break

            if self._keyence_val is None:
                rospy.logwarn("[ArmRos] Keyence value not available — skipping")
                break

            val = self._keyence_val
            if abs(val) <= self._k_tol:
                rospy.loginfo(f"[ArmRos] Distance OK: {val:.3f} mm")
                break

            dz = np.clip(val * self._k_dir * self._k_kp,
                         -self._k_max_step, self._k_max_step)

            ret, pose = self._robot.GetActualTCPPose()
            if ret != 0:
                rospy.logerr(f"[ArmRos] GetActualTCPPose failed: {ret}")
                break

            x, y, z, rx, ry, rz = pose
            z_vec = R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()[:, 2]
            new_pose = [x + z_vec[0]*dz, y + z_vec[1]*dz, z + z_vec[2]*dz, rx, ry, rz]

            rospy.loginfo(f"  [Adj {step+1}/{self._k_steps}] val={val:.3f} dz={dz:.3f} mm")
            ret = self._robot.MoveL(new_pose, tool=0, user=0)
            if ret != 0:
                rospy.logerr(f"[ArmRos] MoveL failed: {ret}")
                break
            time.sleep(1.0)

        else:
            rospy.logwarn(f"[ArmRos] Keyence not converged in {self._k_steps} steps. "
                          f"Last: {self._keyence_val:.3f} mm")

    # --------------------------------------------------
    def _scan_at_point(self, point_id):
        rospy.loginfo(f"[ArmRos] Scanning point {point_id} ({self._num_samples} samples)")
        results = []
        images  = []

        for s in range(self._num_samples):
            if self._cancel:
                break

            frame = self._latest_frame
            if frame is None:
                rospy.logwarn(f"  [ArmRos] Sample {s+1}: no frame yet")
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
                    rospy.logwarn(f"  [ArmRos] Image publish failed: {e}")

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
            rospy.logerr(f"[ArmRos] No valid samples at point {point_id}")

    # --------------------------------------------------
    def _move_to_home(self):
        rospy.loginfo("[ArmRos] Moving to home...")
        try:
            self._robot.Mode(0)
            time.sleep(0.3)
            self._robot.SetSpeed(30)
            ret = self._robot.MoveJ(self._home_deg, tool=0, user=0)
            if ret == 14:
                self._robot.ResetAllError()
                time.sleep(0.5)
                ret = self._robot.MoveJ(self._home_deg, tool=0, user=0)
            if ret != 0:
                rospy.logerr(f"[ArmRos] MoveJ(Home) failed: {ret}")
        except Exception as e:
            rospy.logerr(f"[ArmRos] Home exception: {e}")

    def _exec_joint(self, joints_rad):
        if len(joints_rad) != 6:
            rospy.logerr("[ArmRos] Joint goal must have 6 values")
            return
        joints_deg = [np.degrees(j) for j in joints_rad]
        rospy.loginfo(f"[ArmRos] MoveJ → {[f'{d:.2f}' for d in joints_deg]}")
        ret = self._robot.MoveJ(joints_deg, tool=0, user=0)
        if ret != 0:
            rospy.logerr(f"[ArmRos] MoveJ failed: {ret}")

    def _exec_pose(self, p):
        pos, rpy = self._transform_pose(p, self._robot_pose)
        target = [pos[0], pos[1], pos[2], rpy[0], rpy[1], rpy[2]]
        rospy.loginfo(f"[ArmRos] IK target: {[f'{v:.3f}' for v in target]}")
        ret, joints = self._robot.GetInverseKin(0, target, config=-1)
        if ret != 0:
            rospy.logerr(f"[ArmRos] IK failed: {ret}")
            return
        ret = self._robot.MoveJ(joints, tool=0, user=0)
        if ret != 0:
            rospy.logerr(f"[ArmRos] MoveJ failed: {ret}")

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
        rospy.loginfo("[ArmRos] /scan_finished published")

    def shutdown(self):
        rospy.loginfo("[ArmRos] Shutting down...")
        try:
            self._robot.StopMotion()
        except Exception:
            pass


if __name__ == '__main__':
    node = ArmControllerRos()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
