#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R

import roboticstoolbox as rtb
from roboticstoolbox import jtraj
from spatialmath import SE3, UnitQuaternion

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from robot_msgs.msg import Pose2DWithFlag


# URDF path (resolved relative to this script)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_URDF_PATH = os.path.join(
    _SCRIPT_DIR, '..', '..', 'frcobot_ros', 'frcobot_description',
    'urdf', 'fr10v6_vision.urdf'
)

JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

# Trajectory interpolation rate (Hz)
TRAJ_RATE = 50
# Default number of waypoints for jtraj
DEFAULT_STEPS = 100


class ArmController:
    """
    ArmController (roboticstoolbox version)
    ========================================
    - Uses roboticstoolbox for FK/IK instead of MoveIt
    - Publishes joint trajectories to /joint_states
    - Subscribe /robot_pose (for pose scan only)
    - Execute scan_points from TaskExecutor
    - Support:
        - pose control (numerical IK via ikine_LM)
        - joint control (direct joint interpolation)
    """

    def __init__(self, model_path=None):
        self.busy = False
        self.cancel_requested = False
        self.current_pose_msg = None
        self.goals_queue = []

        # ---------- ROS node ----------
        if not rospy.core.is_initialized():
            rospy.init_node('arm_controller', anonymous=True)

        # ---------- roboticstoolbox ----------
        urdf_path = os.path.normpath(_URDF_PATH)
        rospy.loginfo(f"[Arm] Loading URDF: {urdf_path}")
        self.robot = rtb.ERobot.URDF(urdf_path)
        rospy.loginfo(f"[Arm] Robot loaded, DOF={self.robot.n}")

        # ---------- Home ----------
        self.home_joint_positions = np.array([
            -1.5708, -1.5708, 1.5708,
            -1.5708, -1.5708, 0.0
        ])
        self.current_q = np.copy(self.home_joint_positions)

        # ---------- ROS publishers / subscribers ----------
        self.joint_pub = rospy.Publisher('/joint_command', JointState, queue_size=1)
        self.done_pub = rospy.Publisher('/scan_finished', Bool, queue_size=1)
        rospy.Subscriber('/robot_pose', Pose2DWithFlag, self.pose_cb, queue_size=1)

        rospy.loginfo("[Arm] Move to home")
        self._publish_joint_state(self.home_joint_positions)
        self.current_q = np.copy(self.home_joint_positions)

        rospy.loginfo("[ArmController] Ready (roboticstoolbox)")

    # --------------------------------------------------
    # robot_pose cache (pose scan only)
    # --------------------------------------------------
    def pose_cb(self, msg):
        self.current_pose_msg = msg

    # --------------------------------------------------
    # Publish a single JointState message
    # --------------------------------------------------
    def _publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_NAMES
        msg.position = q.tolist()
        self.joint_pub.publish(msg)

    # --------------------------------------------------
    # Execute a trajectory (list of joint waypoints)
    # --------------------------------------------------
    def _execute_trajectory(self, traj_q, rate_hz=TRAJ_RATE):
        rate = rospy.Rate(rate_hz)
        for q in traj_q:
            if self.cancel_requested or rospy.is_shutdown():
                break
            self._publish_joint_state(q)
            self.current_q = np.copy(q)
            rate.sleep()

    # --------------------------------------------------
    # MAIN ENTRY
    # --------------------------------------------------
    def execute_scan_points(self, scan_points):

        if self.busy:
            rospy.logwarn("[Arm] Busy, ignore scan request")
            return

        if not scan_points:
            rospy.logwarn("[Arm] Empty scan_points")
            self.publish_done()
            return

        if any(p["mode"] == "pose" for p in scan_points):
            if self.current_pose_msg is None:
                rospy.logerr("[Arm] pose scan requires /robot_pose")
                return
            msg = self.current_pose_msg
            rospy.loginfo(
                f"[Arm] robot_pose: x={msg.x:.3f}, y={msg.y:.3f}, "
                f"theta={msg.theta:.3f}, id={msg.id}"
            )
        self.cancel_requested = False
        self.busy = True

        try:
            self.goals_queue = self._build_goals(scan_points)

            if not self.goals_queue:
                rospy.logwarn("[Arm] No valid goals")
                self.publish_done()
                return

            self._execute_goals()

        except Exception as e:
            rospy.logerr(f"[Arm] Scan exception: {e}")

        finally:
            rospy.loginfo("[Arm] Scan finished → home")
            self.move_to_home()
            self.goals_queue = []
            self.busy = False

    # --------------------------------------------------
    # BUILD GOALS
    # --------------------------------------------------
    def _build_goals(self, scan_points):
        goals = []

        pose_points = []
        for p in scan_points:
            if p["mode"] == "pose":
                pose_points.append(p)
            elif p["mode"] == "joint":
                goals.append(("joint", p["joints"], p.get("speed", 80)))

        if pose_points:
            raw = []
            for i, p in enumerate(pose_points):
                raw.append({
                    "point_id": i,
                    "x": p["x"],
                    "y": p["y"],
                    "z": p["z"],
                    "rx": p["rx"],
                    "ry": p["ry"],
                    "rz": p["rz"],
                    "speed": p.get("speed", 80)
                })

            transformed = self.process_transforms(raw, self.current_pose_msg)
            for idx, (pos, quat, speed) in enumerate(transformed):
                q0 = pose_points[idx].get("q0")  # IK seed from joint CSV
                goals.append(("pose", pos, quat, speed, q0))

        return goals

    # --------------------------------------------------
    # EXECUTE
    # --------------------------------------------------
    def _execute_goals(self):
        rospy.loginfo(f"[Arm] Execute {len(self.goals_queue)} goals")

        for i, goal in enumerate(self.goals_queue):

            if self.cancel_requested:
                rospy.logwarn("[Arm] Scan cancelled")
                break

            if goal[0] == "pose":
                _, pos, quat, speed, q0 = goal
                self._execute_pose_goal(pos, quat, speed, q0)

            elif goal[0] == "joint":
                _, joints, speed = goal
                self._execute_joint_goal(joints, speed)

            rospy.sleep(0.3)

        self.publish_done()

    # --------------------------------------------------
    # POSE GOAL (IK + trajectory interpolation)
    # --------------------------------------------------
    def _execute_pose_goal(self, pos, quat, speed, q0=None):
        # Build SE3 target from position + quaternion (xyzw)
        rotation = UnitQuaternion(s=quat[3], v=quat[:3])
        T_target = SE3.Rt(rotation.R, pos)

        # IK seed: prefer paired joint data, fallback to current joints
        if q0 is not None:
            ik_seed = np.array(q0, dtype=float)
        else:
            ik_seed = self.current_q

        # Numerical IK
        dist = np.linalg.norm(pos)
        rospy.loginfo(f"[Arm] IK target in base_link: pos={np.round(pos, 4)}, dist={dist:.3f}m")
        sol = self.robot.ikine_LM(T_target, q0=ik_seed)
        if not sol.success:
            rospy.logerr(f"[Arm] IK failed for pos={np.round(pos, 4)}, dist={dist:.3f}m, skipping")
            return

        q_target = sol.q
        steps = max(20, int(DEFAULT_STEPS * (100 - speed) / 100) + 20)
        traj = jtraj(self.current_q, q_target, steps)

        rospy.loginfo(f"[Arm] Pose goal: {np.round(pos, 3)}, steps={steps}")
        self._execute_trajectory(traj.q)

    # --------------------------------------------------
    # JOINT GOAL (direct interpolation)
    # --------------------------------------------------
    def _execute_joint_goal(self, joints, speed):
        if len(joints) != 6:
            rospy.logerr("[Arm] joint goal must have 6 values")
            return

        q_target = np.array(joints, dtype=float)
        steps = max(20, int(DEFAULT_STEPS * (100 - speed) / 100) + 20)
        traj = jtraj(self.current_q, q_target, steps)

        rospy.loginfo(f"[Arm] Joint goal: {np.round(q_target, 3)}, steps={steps}")
        self._execute_trajectory(traj.q)

    # --------------------------------------------------
    # HOME
    # --------------------------------------------------
    def move_to_home(self):
        traj = jtraj(self.current_q, self.home_joint_positions, 80)
        self._execute_trajectory(traj.q)

    # --------------------------------------------------
    # TRANSFORM (unchanged)
    # --------------------------------------------------
    def create_homogeneous_transform(self, Rm, t):
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

    def process_transforms(self, goals, msg):
        """
        Transform goal poses from Isaac Sim world frame to arm base_link frame.

        Coordinate frames
        -----------------
        - CSV poses are in Isaac Sim world frame.
        - robot_pose (msg) is in **manipulator** frame:
              manip_x = -isaac_y,  manip_y = -isaac_x
          msg.theta is the heading in **Isaac world degrees**.
        - The arm base_link has a fixed offset from the mobile-base center
          expressed in the **body** frame.  It is rotated into the Isaac world
          frame by the heading before being added.
        - R_aw includes small tilt corrections (Rx, Ry) and a heading bias.
        - CSV orientations use **ZYX** intrinsic euler convention (radians),
          with an additional fixed rotation correction (R_corr).

        Calibrated from paired joint / pose CSV data across zone B and C
        (4 tag groups, 655 points).
        Position: mean ~16 mm, max ~39 mm.
        Orientation: mean ~5 deg, max ~9 deg.
        IK success: 100%.
        """
        transformed = []

        # --- Calibrated parameters (9-DOF) ---
        # Arm mount offset in robot body frame
        body_off_x = rospy.get_param('~arm_body_offset_x', -0.166715)
        body_off_y = rospy.get_param('~arm_body_offset_y', -0.254772)
        arm_base_z = rospy.get_param('~arm_base_z', 0.974167)
        # R_aw tilt corrections (radians)
        tilt_x     = rospy.get_param('~arm_tilt_x', 0.054898)
        tilt_y     = rospy.get_param('~arm_tilt_y', 0.017894)
        # Heading bias (radians)
        h_bias     = rospy.get_param('~arm_heading_bias', 0.014368)
        # Orientation correction (radians, applied before R_aw)
        ori_corr_x = rospy.get_param('~arm_ori_corr_x', -0.095520)
        ori_corr_y = rospy.get_param('~arm_ori_corr_y', -0.052944)
        ori_corr_z = rospy.get_param('~arm_ori_corr_z', -0.008688)

        R_corr = R.from_euler('xyz', [ori_corr_x, ori_corr_y, ori_corr_z]).as_dcm()

        # --- robot_pose: manipulator frame → Isaac world frame ---
        isaac_x = -msg.y
        isaac_y = -msg.x
        theta_rad = np.radians(msg.theta) + h_bias

        # Rotate body-frame offset into Isaac world frame
        c, s = np.cos(theta_rad), np.sin(theta_rad)
        world_off_x = c * body_off_x - s * body_off_y
        world_off_y = s * body_off_x + c * body_off_y

        # Arm base origin in Isaac world
        arm_world = np.array([isaac_x + world_off_x,
                              isaac_y + world_off_y,
                              arm_base_z])

        # R_aw: Isaac world → arm base_link  =  Rz(theta) · Ry(tilt_y) · Rx(tilt_x)
        R_aw = (R.from_euler('z', theta_rad)
                * R.from_euler('y', tilt_y)
                * R.from_euler('x', tilt_x)).as_dcm()

        # Build 4×4 T_aw
        t_aw = -R_aw @ arm_world
        T_aw = self.create_homogeneous_transform(R_aw, t_aw)
        R_aw_rot = R.from_dcm(R_aw)

        for g in goals:
            # Position: Isaac world → arm base_link
            p_world = np.array([g['x'], g['y'], g['z'], 1.0])
            p_arm = (T_aw @ p_world)[:3]

            # Orientation: CSV ZYX euler (rad) → apply R_corr → rotate by R_aw
            r_csv   = R.from_euler('zyx', [g['rx'], g['ry'], g['rz']])
            r_arm   = R_aw_rot * R.from_dcm(R_corr @ r_csv.as_dcm())

            rospy.loginfo(
                f"[Arm] Transform: world={np.round(p_world[:3], 3)} "
                f"-> arm={np.round(p_arm, 4)}, dist={np.linalg.norm(p_arm):.3f}m"
            )

            transformed.append(
                (p_arm, r_arm.as_quat(), g['speed'])
            )

        return transformed

    # --------------------------------------------------
    # DONE
    # --------------------------------------------------
    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        rospy.loginfo("[Arm] scan_finished published")

    def is_busy(self):
        return self.busy

    def cancel(self):
        """
        scan STOP
        """
        rospy.logwarn("[Arm] CANCEL requested")
        self.cancel_requested = True


if __name__ == '__main__':
    controller = ArmController()
    rospy.spin()
