#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

from std_msgs.msg import Bool
from robot_msgs.msg import Pose2DWithFlag

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
    - Fairino SDK only
    - pose scan  (IK → MoveJ)
    - joint scan (MoveJ)
    - STOP-safe (cancel)
    - Home position management
    """

    def __init__(self, robot_ip="192.168.58.2", model_path=None):

        # ---------- state ----------
        self.busy = False
        self.cancel_requested = False
        self.current_pose_msg = None

        # ---------- Fairino ----------
        rospy.loginfo("[Arm REAL] Connecting to Fairino robot...")
        self.robot = Robot.RPC(robot_ip)

        time.sleep(1)

        # ---------- Home ----------
        self.home_joint_positions = [
            -1.5708, -1.5708, 1.5708,
            -1.5708, -1.5708, 0.0
        ]

        # ---------- ROS ----------
        rospy.Subscriber(
            "/robot_pose",
            Pose2DWithFlag,
            self.pose_cb,
            queue_size=1
        )
        self.done_pub = rospy.Publisher(
            "/scan_finished",
            Bool,
            queue_size=1
        )

        # No homing on startup — see the same note in arm_controller.py.
        # Bringing the stack up must never command arm motion; homing is
        # explicit only (task start, or the /arm/move_home service).
        rospy.loginfo("[ArmController REAL] Ready — arm left in place "
                      "(call /arm/move_home to home it)")

    # --------------------------------------------------
    # robot_pose cache
    # --------------------------------------------------
    def pose_cb(self, msg):
        self.current_pose_msg = msg

    # --------------------------------------------------
    # HOME POSITION
    # --------------------------------------------------
    def move_to_home(self):
        """
        Move robot arm to predefined home joint position.
        (Used only for init & normal scan finish)
        """
        rospy.loginfo("[Arm REAL] Move to Home position")
        try:
            self.robot.SetSpeed(30)
            ret = self.robot.MoveJ(
                self.home_joint_positions,
                tool=0,
                user=0
            )
            if ret != 0:
                rospy.logerr(f"[Arm REAL] MoveJ(Home) failed: {ret}")
        except Exception as e:
            rospy.logerr(f"[Arm REAL] Home move exception: {e}")

    # --------------------------------------------------
    # EMERGENCY CANCEL (STOP)
    # --------------------------------------------------
    def cancel(self):
        """Halt the arm now. Returns True once StopMotion has been accepted.

        ⚠️ The retry is load-bearing, not defensive padding. One RPC socket,
        held by a blocking MoveCart/MoveJ, means a StopMotion from the calling
        thread is often rejected with `Request-sent`. Measured on the robot
        2026-08-12: a single-shot stop visibly did nothing while the arm moved;
        a second attempt ~14 ms later went through. Full reasoning is in
        arm_controller.ArmController.cancel — this variant is kept in step with
        it deliberately, since arm_node can be pointed at either.
        """
        rospy.logwarn("[Arm REAL] CANCEL requested")
        self.cancel_requested = True

        attempts = int(rospy.get_param('~cancel_attempts', 10))
        interval = float(rospy.get_param('~cancel_retry_s', 0.02))

        last_error = None
        for attempt in range(1, attempts + 1):
            try:
                ret = self.robot.StopMotion()
            except Exception as e:
                last_error = e
            else:
                if not isinstance(ret, int) or ret == 0:
                    if attempt > 1:
                        rospy.logwarn(
                            f"[Arm REAL] Stop accepted on attempt {attempt} "
                            f"(earlier: {last_error})")
                    return True
                last_error = f"error code {ret}"
            if attempt < attempts:
                time.sleep(interval)

        rospy.logerr(
            f"[Arm REAL] Stop REJECTED after {attempts} attempts "
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

        # pose scan requires robot_pose
        if any(p["mode"] == "pose" for p in scan_points):
            if self.current_pose_msg is None:
                rospy.logerr("[Arm REAL] pose scan requires /robot_pose")
                return

        self.busy = True
        self.cancel_requested = False

        try:
            for i, p in enumerate(scan_points):

                if self.cancel_requested:
                    rospy.logwarn("[Arm REAL] Scan cancelled")
                    break

                rospy.loginfo(
                    f"[Arm REAL] Execute scan point "
                    f"{i+1}/{len(scan_points)}"
                )

                speed = int(p.get("speed", 60))
                self.robot.SetSpeed(speed)

                if p["mode"] == "joint":
                    self._exec_joint(p["joints"])

                elif p["mode"] == "pose":
                    self._exec_pose(p)

                time.sleep(0.3)

            # Return home only on normal (non-cancelled) completion
            if not self.cancel_requested:
                rospy.loginfo("[Arm REAL] Scan finished → Home")
                self.move_to_home()

            self.publish_done()

        except Exception as e:
            rospy.logerr(f"[Arm REAL] Scan exception: {e}")

        finally:
            self.busy = False

    # --------------------------------------------------
    # JOINT
    # --------------------------------------------------
    def _exec_joint(self, joints_rad):
        if len(joints_rad) != 6:
            rospy.logerr("[Arm REAL] Joint goal must have 6 values")
            return

        # ---- rad -> degree ----
        joints_deg = [np.degrees(j) for j in joints_rad]

        rospy.loginfo(f"[Arm REAL] MoveJ (deg) → {joints_deg}")
        ret = self.robot.MoveJ(joints_deg, tool=0, user=0)

        if ret != 0:
            rospy.logerr(f"[Arm REAL] MoveJ failed: {ret}")


    # --------------------------------------------------
    # POSE (IK → MoveJ)
    # --------------------------------------------------
    def _exec_pose(self, p):

        pos, rpy = self._transform_pose(p, self.current_pose_msg)

        target = [
            pos[0], pos[1], pos[2],
            rpy[0], rpy[1], rpy[2]
        ]

        rospy.loginfo(f"[Arm REAL] IK target: {target}")

        ret, joints = self.robot.GetInverseKin(
            0, target, config=-1
        )

        if ret != 0:
            rospy.logerr(f"[Arm REAL] IK failed: {ret}")
            return

        rospy.loginfo(f"[Arm REAL] IK → joints: {joints}")
        ret = self.robot.MoveJ(joints, tool=0, user=0)

        if ret != 0:
            rospy.logerr(f"[Arm REAL] MoveJ failed: {ret}")

    # --------------------------------------------------
    # POSE TRANSFORM
    # --------------------------------------------------
    def _transform_pose(self, g, msg):
        """
        Transform a scan pose defined in the task CSV into the robot base frame.

        Conventions:
        - CSV pose angles (rx, ry, rz): radians
        - Internal geometric computation: radians
        - Output pose (rpy): degrees (required by Fairino SDK)
        """

        # --------------------------------------------------
        # Base position from mobile robot pose
        # --------------------------------------------------
        base_x, base_y = msg.x, msg.y
        base_z = -0.835

        # Relative translation (scan point -> robot base)
        dx = g["x"] - base_x
        dy = g["y"] - base_y
        dz = g["z"] - base_z

        # Coordinate system adjustment
        dx, dy, dz = dx, -dy, -dz

        # --------------------------------------------------
        # Rotation: all computations in radians
        # --------------------------------------------------
        # Base frame flip (camera / robot convention)
        R_base = R.from_euler('y', np.pi)

        # Yaw from mobile robot pose (already in radians)
        R_yaw = R.from_euler('z', msg.theta)

        # Mobile base rotation
        R_mb = (R_base * R_yaw).as_matrix()

        # Homogeneous transforms
        T_mb = self._T(R_mb, [base_x, base_y, 0.18])
        T_ba = self._T(
            R.from_euler('z', np.pi).as_matrix(),
            [0, 0, -1.02]
        )

        # Final transform: mobile base -> arm base
        T = T_mb @ T_ba
        R_ab = R.from_matrix(T[:3, :3])

        # --------------------------------------------------
        # Orientation from CSV (rx, ry, rz in radians)
        # --------------------------------------------------
        r_tag = R.from_euler(
            'xyz',
            [g["rx"], g["ry"], g["rz"]]   # radians
        )

        # Transform orientation into arm base frame
        r_goal = R_ab.inv() * r_tag

        # --------------------------------------------------
        # Output orientation in degrees (required by SDK)
        # --------------------------------------------------
        rpy_deg = r_goal.as_euler("xyz", degrees=True)

        # Final position (meters) and orientation (degrees)
        pos = np.array([dx, dy, dz])
        return pos, rpy_deg



    def _T(self, Rm, t):
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

    # --------------------------------------------------
    # DONE
    # --------------------------------------------------
    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        rospy.loginfo("[Arm REAL] scan_finished published")

    def is_busy(self):
        return self.busy


if __name__ == "__main__":
    controller = ArmController()
    rospy.spin()
