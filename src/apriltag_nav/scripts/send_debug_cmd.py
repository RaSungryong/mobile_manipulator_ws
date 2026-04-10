#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Debug command sender.

Two formats for cmd_str:

  1. Direct topic publish (starts with '/'):
       "/topic  msg_type  data"
     e.g.
       "/arm/scan_command  std_msgs/String  [...]"
       "/arm/cancel        std_msgs/Bool    true"
       "/mobile/goto_tag   std_msgs/Int32   108"

  2. /task_command string (Plan A / Plan B orchestrator):
       "TASK scan_joints_line1"
       "GOTO 108"
       "STOP"
       "STATE"
       "EVAL self.state.name"
       "EXEC self.arm.move_to_home()"

Supported msg types for format 1:
  std_msgs/String  std_msgs/Bool  std_msgs/Int32  std_msgs/Float32
"""

import rospy
from std_msgs.msg import String, Bool, Int32, Float32

# ---- message type registry ----
_MSG_TYPES = {
    'std_msgs/String':  (String,  lambda d: String(data=d)),
    'std_msgs/Bool':    (Bool,    lambda d: Bool(data=d.strip().lower() in ('true', '1'))),
    'std_msgs/Int32':   (Int32,   lambda d: Int32(data=int(d.strip()))),
    'std_msgs/Float32': (Float32, lambda d: Float32(data=float(d.strip()))),
}


def _parse_direct(cmd_str):
    """Parse '/topic  msg_type  data' → (topic, MsgClass, msg_instance)."""
    parts = cmd_str.split(None, 2)   # split on whitespace, max 3 parts
    if len(parts) < 3:
        raise ValueError(f"Format: '/topic msg_type data', got: {cmd_str!r}")
    topic, msg_type_str, data = parts
    if msg_type_str not in _MSG_TYPES:
        raise ValueError(f"Unsupported msg type '{msg_type_str}'. "
                         f"Supported: {list(_MSG_TYPES)}")
    MsgClass, builder = _MSG_TYPES[msg_type_str]
    return topic, MsgClass, builder(data)


def main():
    rospy.init_node("debug_command_sender_once", anonymous=True)
    rospy.sleep(0.5)   # wait for ROS Master registration

    # ==========================================
    # Set cmd_str to the command you want to send
    # ==========================================

    # ------------------------------------------
    # Format 1 — direct topic publish (Plan B)
    # ------------------------------------------

    # Arm scan — joint mode (single point)
    # cmd_str = '/arm/scan_command std_msgs/String [{"mode": "joint", "joints": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], "speed": 50}]'

    # Arm scan — joint mode (multiple points)
    # cmd_str = '/arm/scan_command std_msgs/String [{"mode": "joint", "joints": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], "speed": 50}, {"mode": "joint", "joints": [0.3, -1.57, 1.57, -1.57, -1.57, 0.0], "speed": 50}]'

    # Arm scan — pose mode
    # cmd_str = '/arm/scan_command std_msgs/String [{"mode": "pose", "x": 0.5, "y": 0.0, "z": -0.5, "rx": 0.0, "ry": 3.14, "rz": 0.0, "speed": 50}]'

    # Cancel arm
    # cmd_str = '/arm/cancel std_msgs/Bool true'

    # Navigate to tag (Plan B robot_controller_ros)
    # cmd_str = '/mobile/goto_tag std_msgs/Int32 108'

    # Stop mobile base
    # cmd_str = '/mobile/stop std_msgs/Bool true'

    # ------------------------------------------
    # Format 2 — /task_command (Plan A & Plan B)
    # ------------------------------------------
    # cmd_str = 'TASK scan_joints_line1'
    cmd_str = 'TASK scan_grid_line1'
    # cmd_str =  'TASK scan_joints_line1_new'
    # cmd_str = 'GOTO 108'
    # cmd_str = 'STOP'

    # Plan A — pose test: TEST_POSE x y z [rx ry rz]
    # cmd_str = 'TEST_POSE 0.737 2.54 0.704'
    # cmd_str = 'STATE'

    # Plan A only (self.arm / self.mobile exist)
    # cmd_str = 'EVAL self.state.name'
    # cmd_str = 'EXEC self.arm.move_to_home()'
    # cmd_str = 'EXEC self.arm._execute_joint_goal([0.0, -1.57, 1.57, -1.57, -1.57, 0.0], 50)'
    # cmd_str = 'EXEC self.arm._execute_joint_goal([-1.58081596,-0.084689954,0.608351288,-2.097363859,-1.665561886,-0.010076786],50)'
    # cmd_str = 'EXEC self.arm.execute_scan_points([{"mode": "joint", "joints": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0], "speed": 50}])'
    # cmd_str = 'EXEC self.mobile.move_to_tag(108)'

    # Plan A — pose mode test (inject fake robot_pose + execute pose scan)
    # Uses world coord (0.737, 2.14, 0.704), keeps home orientation
    # cmd_str = 'EXEC from robot_msgs.msg import Pose2DWithFlag; p = Pose2DWithFlag(); p.x = self.mobile.robot_x if hasattr(self.mobile, "robot_x") else 0.7; p.y = self.mobile.robot_y if hasattr(self.mobile, "robot_y") else 2.1; p.theta = self.mobile.robot_theta if hasattr(self.mobile, "robot_theta") else 0.0; p.flag = True; p.id = 0; self.arm.current_pose_msg = p; self.arm.execute_scan_points([{"mode": "pose", "x": 0.737, "y": 2.14, "z": 0.704, "rx": 1.5708, "ry": 0.0, "rz": 3.1416, "speed": 50}])'

    # Plan A — pose test (simpler: set robot_pose then scan)
    # Step 1: Set robot_pose (run this first, adjust x/y/theta to actual base position)
    # cmd_str = 'EXEC from robot_msgs.msg import Pose2DWithFlag; p = Pose2DWithFlag(); p.x = 0.7; p.y = 2.1; p.theta = 0.0; p.flag = True; p.id = 106; self.arm.current_pose_msg = p; rospy.loginfo(f"[DEBUG] robot_pose set: x={p.x}, y={p.y}, theta={p.theta}")'
    # Step 2: Execute pose scan at (0.737, 2.14, 0.704)
    # cmd_str = 'EXEC self.arm.execute_scan_points([{"mode": "pose", "x": 0.737, "y": 2.14, "z": 0.704, "rx": 1.5708, "ry": 0.0, "rz": 3.1416, "speed": 50}])'

    # ==========================================
    # Dispatch and publish
    # ==========================================
    cmd_str = cmd_str.strip()

    if cmd_str.startswith('/'):
        # Format 1: direct topic
        topic, MsgClass, msg = _parse_direct(cmd_str)
        pub = rospy.Publisher(topic, MsgClass, queue_size=1)
        rospy.sleep(0.3)
        rospy.loginfo(f"[Direct] {topic} → {msg}")
        pub.publish(msg)
    else:
        # Format 2: /task_command
        pub = rospy.Publisher('/task_command', String, queue_size=1)
        rospy.sleep(0.3)
        rospy.loginfo(f"[TaskCmd] {cmd_str}")
        pub.publish(String(data=cmd_str))

    rospy.sleep(0.5)   # wait for network flush
    rospy.loginfo("Command sent.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
