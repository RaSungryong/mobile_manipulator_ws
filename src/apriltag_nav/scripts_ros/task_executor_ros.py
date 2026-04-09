#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plan B — Task Executor ROS Node
================================
Same state machine as task_executor.py (Plan A), but arm communication
uses ROS topics instead of direct Python method calls:

  Plan A:  self.arm.execute_scan_points(scan_points)   [Python call]
  Plan B:  self._scan_cmd_pub.publish(JSON)             [ROS topic]

Arm cancel uses /arm/cancel (Bool) topic instead of self.arm.cancel().

All navigation and task logic is identical to Plan A.
"""

import os
import json

import rospy
from std_msgs.msg import Bool, String
from enum import Enum, auto
from typing import List, Optional

from map_manager import MapManager
from task_manager import TaskManager
from robot_controller import RobotController
import utils


# ============================================================
# STATE ENUM
# ============================================================
class MobileManipulatorState(Enum):
    IDLE      = auto()
    MOVING    = auto()
    ARRIVED   = auto()
    SCANNING  = auto()
    SCAN_DONE = auto()
    ERROR     = auto()


# ============================================================
# PATHS
# ============================================================
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR    = os.path.abspath(os.path.join(_SCRIPT_DIR, '..'))

CONFIG_PATH = os.path.join(_PKG_DIR, 'config', 'robot.yaml')
MAP_PATH    = os.path.join(_PKG_DIR, 'config', 'map.yaml')
TASK_DIR    = os.path.join(_PKG_DIR, 'task', 'csv')


# ============================================================
# TASK EXECUTOR (Plan B)
# ============================================================
class MobileManipulatorTaskExecutorRos:
    """
    Plan B task executor.
    Communicates with arm_controller_ros via /arm/scan_command and /arm/cancel topics.
    No Python import of ArmController.
    """

    def __init__(self):
        rospy.init_node('mobile_manipulator_system')

        # ---------- Config ----------
        robot_config = utils.load_config(CONFIG_PATH)

        # ---------- Managers ----------
        self.map_mgr  = MapManager(MAP_PATH)
        self.task_mgr = TaskManager(TASK_DIR)

        # ---------- Navigation ----------
        self.mobile = RobotController(
            config=robot_config,
            map_manager=self.map_mgr
        )

        # ---------- State ----------
        self.state     = MobileManipulatorState.IDLE
        self.scan_done = False

        self._current_task_name: Optional[str]       = None
        self._current_task:      Optional[List[dict]] = None
        self._pending_task_name: Optional[str]       = None
        self._pending_task:      Optional[List[dict]] = None

        self._stop_requested = False
        self._task_running   = False

        # ---------- ROS: arm interface (Plan B) ----------
        # Publish scan command as JSON string → arm_controller_ros
        self._scan_cmd_pub = rospy.Publisher(
            '/arm/scan_command', String, queue_size=1
        )
        # Publish cancel signal → arm_controller_ros
        self._cancel_pub = rospy.Publisher(
            '/arm/cancel', Bool, queue_size=1
        )

        # ---------- ROS subscribers ----------
        rospy.Subscriber('/scan_finished', Bool,   self._scan_done_cb, queue_size=1)
        rospy.Subscriber('/task_command',  String, self._command_cb,   queue_size=10)

        rospy.loginfo("[ExecutorRos] System ready — Plan B (ROS-only arm interface)")

    # ==========================================================
    # COMMAND CALLBACK
    # ==========================================================
    def _command_cb(self, msg: String):
        cmd = msg.data.strip()
        rospy.logwarn(f"[TASK CMD] {cmd}")

        if cmd.upper().startswith("EXEC "):
            try:
                exec(cmd[5:].strip(), globals(), locals())
                rospy.loginfo("[EXEC] OK")
            except Exception as e:
                rospy.logerr(f"[EXEC] {e}")
            return

        if cmd.upper().startswith("EVAL "):
            try:
                rospy.loginfo(f"[EVAL] => {eval(cmd[5:].strip(), globals(), locals())}")
            except Exception as e:
                rospy.logerr(f"[EVAL] {e}")
            return

        # ---------- STOP ----------
        if cmd.upper() == "STOP":
            rospy.logerr("[TASK] EMERGENCY STOP")
            self._stop_requested    = True
            self._pending_task      = None
            self._pending_task_name = None
            try:
                self.mobile.emergency_stop_robot()
            except Exception:
                pass
            self._cancel_pub.publish(Bool(data=True))   # Plan B: cancel arm via topic
            self._current_task      = None
            self._current_task_name = None
            self.state = MobileManipulatorState.IDLE
            return

        # ---------- TASK ----------
        if cmd.upper().startswith("TASK"):
            parts = cmd.split()
            if len(parts) != 2:
                rospy.logerr("[TASK] Usage: TASK <task_name>")
                return
            task_name = parts[1]
            task = self.task_mgr.get_task(task_name)
            if not task:
                rospy.logerr(f"[TASK] Unknown task '{task_name}'")
                return
            rospy.logwarn(f"[TASK] Preempt → pending '{task_name}'")
            self._pending_task_name = task_name
            self._pending_task      = task
            self._stop_requested    = True
            try:
                self.mobile.preempt_stop_robot()
            except Exception:
                pass
            self._cancel_pub.publish(Bool(data=True))
            return

        # ---------- GOTO ----------
        if cmd.upper().startswith("GOTO"):
            parts = cmd.split()
            if len(parts) != 2:
                rospy.logerr("[TASK] Usage: GOTO <tag_id>")
                return
            tag_id = int(parts[1])
            rospy.logwarn(f"[TASK] Preempt → pending GOTO {tag_id}")
            self._pending_task_name = f"goto_{tag_id}"
            self._pending_task      = self.task_mgr.build_goto_task(tag_id)
            self._stop_requested    = True
            try:
                self.mobile.preempt_stop_robot()
            except Exception:
                pass
            return

        # ---------- STATE ----------
        if cmd.upper() == "STATE":
            rospy.loginfo(f"[STATE] {self.state.name}")

    # ==========================================================
    # SCAN DONE CALLBACK
    # ==========================================================
    def _scan_done_cb(self, msg: Bool):
        if msg.data:
            self.scan_done = True

    # ==========================================================
    # MAIN LOOP
    # ==========================================================
    def run(self):
        rate = rospy.Rate(5)
        rospy.loginfo("[ExecutorRos] Main loop started")

        while not rospy.is_shutdown():
            if self._task_running:
                rate.sleep()
                continue

            if self._current_task is None and self._pending_task is not None:
                rospy.loginfo(f"[TASK] Activate pending '{self._pending_task_name}'")
                self._current_task_name = self._pending_task_name
                self._current_task      = self._pending_task
                self._pending_task      = None
                self._pending_task_name = None

            if self._current_task is None:
                rate.sleep()
                continue

            self._task_running   = True
            self._stop_requested = False
            self.mobile.clear_stop_flag()

            self._run_task(self._current_task_name, self._current_task)

            self._current_task      = None
            self._current_task_name = None
            self._task_running      = False
            self.state = MobileManipulatorState.IDLE

            rate.sleep()

    # ==========================================================
    # TASK EXECUTION
    # ==========================================================
    def _run_task(self, task_name: str, task_items: List[dict]):
        rospy.loginfo(f"[TASK] Start task '{task_name}'")

        for item in task_items:
            if self._stop_requested:
                rospy.logwarn("[TASK] Task preempted safely")
                return

            tag_id  = item['tag']
            do_scan = item.get('scan', False)

            # ---- MOVE ----
            self.state = MobileManipulatorState.MOVING
            rospy.loginfo(f"[TASK] Moving to tag {tag_id}")

            ok = self.mobile.move_to_tag(tag_id)
            if not ok:
                rospy.logwarn(f"[TASK] Navigation failed at tag {tag_id}")
                self.state = MobileManipulatorState.ERROR
                return

            self.state = MobileManipulatorState.ARRIVED
            rospy.loginfo(f"[TASK] Arrived at tag {tag_id}")

            # ---- SCAN ----
            if do_scan:
                self.state = MobileManipulatorState.SCANNING
                rospy.loginfo(f"[TASK] Starting scan at tag {tag_id}")

                scan_points = self.task_mgr.get_scan_points(task_name, tag_id)

                # Plan B: trigger arm via ROS topic (not Python method call)
                self.scan_done = False
                self._scan_cmd_pub.publish(String(data=json.dumps(scan_points)))

                # Wait for /scan_finished from arm_controller_ros
                while not self.scan_done and not self._stop_requested:
                    rospy.sleep(0.05)

                if self._stop_requested:
                    rospy.logwarn("[TASK] Scan interrupted by stop request")
                    return

                self.state = MobileManipulatorState.SCAN_DONE
                rospy.loginfo(f"[TASK] Scan finished at tag {tag_id}")

        rospy.loginfo("[TASK] Task finished normally")


# ============================================================
# MAIN
# ============================================================
if __name__ == '__main__':
    executor = MobileManipulatorTaskExecutorRos()
    executor.run()
