#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from std_msgs.msg import Bool, String
from enum import Enum, auto
from typing import List, Optional

from map_manager import MapManager
from task_manager import TaskManager
from robot_controller import RobotController
# from arm_controller_sdk import ArmController
from arm_controller import ArmController
import utils


# ============================================================
# STATE ENUM
# ============================================================
class MobileManipulatorState(Enum):
    IDLE = auto()
    MOVING = auto()
    ARRIVED = auto()
    SCANNING = auto()
    SCAN_DONE = auto()
    ERROR = auto()


# ============================================================
# PATHS
# ============================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)

CONFIG_PATH = os.path.join(PKG_DIR, 'config', 'robot.yaml')
MAP_PATH = os.path.join(PKG_DIR, 'config', 'map.yaml')
TASK_DIR = os.path.join(PKG_DIR, 'task', 'csv')
MODEL_PATH = os.path.join(PKG_DIR, 'model/exported', 'resnet3D.onnx')

# ============================================================
# TASK EXECUTOR
# ============================================================
class MobileManipulatorTaskExecutor:

    def __init__(self):

        rospy.init_node("mobile_manipulator_system")

        # ---------- Config ----------
        robot_config = utils.load_config(CONFIG_PATH)

        # ---------- Managers ----------
        self.map_mgr = MapManager(MAP_PATH)
        self.task_mgr = TaskManager(TASK_DIR)

        # ---------- Controllers ----------
        self.mobile = RobotController(
            config=robot_config,
            map_manager=self.map_mgr
        )
        self.arm = ArmController(model_path=MODEL_PATH)

        # ---------- State ----------
        self.state = MobileManipulatorState.IDLE
        self.scan_done = False

        # ---------- Task state ----------
        self._current_task_name: Optional[str] = None
        self._current_task: Optional[List[dict]] = None

        self._pending_task_name: Optional[str] = None
        self._pending_task: Optional[List[dict]] = None

        self._stop_requested = False
        self._task_running = False

        # ---------- ROS ----------
        rospy.Subscriber("/scan_finished", Bool, self._scan_done_cb, queue_size=1)
        rospy.Subscriber("/task_command", String, self._command_cb, queue_size=10)

        rospy.loginfo("[Executor] System ready (IDLE)")


    # ==========================================================
    # COMMAND CALLBACK
    # ==========================================================
    def _command_cb(self, msg: String):
        cmd = msg.data.strip()
        rospy.logwarn(f"[TASK COMMAND] {cmd}")

        # ==========================================
        # [新增] 外部调试专用指令：动态执行代码
        # ==========================================
        # 用法示例: EXEC self.mobile.move_to_tag(3)
        if cmd.upper().startswith("EXEC "):
            code_to_run = cmd[5:].strip()
            rospy.logwarn(f"[DEBUG EXEC] 正在执行: {code_to_run}")
            try:
                # 传入 globals() 和 locals() 确保能拿到 self
                exec(code_to_run, globals(), locals()) 
                rospy.loginfo("[DEBUG EXEC] 执行成功")
            except Exception as e:
                rospy.logerr(f"[DEBUG EXEC] 执行失败: {e}")
            return

        # 用法示例: EVAL self.state.name
        if cmd.upper().startswith("EVAL "):
            code_to_eval = cmd[5:].strip()
            try:
                res = eval(code_to_eval, globals(), locals())
                rospy.loginfo(f"[DEBUG EVAL] 结果 => {res}")
            except Exception as e:
                rospy.logerr(f"[DEBUG EVAL] 计算失败: {e}")
            return
        # ==========================================


        # ---------- TEST_POSE x y z [rx ry rz] ----------
        # Usage: TEST_POSE 0.737 2.14 0.704
        #        TEST_POSE 0.737 2.14 0.704 1.5708 0.0 3.1416
        # Uses current robot_pose from navigation; if unavailable,
        # injects a pose from the mobile controller's latest known position.
        if cmd.upper().startswith("TEST_POSE"):
            parts = cmd.split()
            if len(parts) < 4:
                rospy.logerr("[TEST_POSE] Usage: TEST_POSE x y z [rx ry rz]")
                return
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            # Default orientation: roughly pointing down (rx=pi/2, ry=0, rz=pi)
            rx = float(parts[4]) if len(parts) > 4 else 1.5708
            ry = float(parts[5]) if len(parts) > 5 else 0.0
            rz = float(parts[6]) if len(parts) > 6 else 3.1416

            # Ensure robot_pose is available
            if self.arm.current_pose_msg is None:
                from robot_msgs.msg import Pose2DWithFlag
                p = Pose2DWithFlag()
                p.x = getattr(self.mobile, 'robot_x', x)
                p.y = getattr(self.mobile, 'robot_y', y)
                p.theta = getattr(self.mobile, 'robot_theta', 0.0)
                p.flag = True
                p.id = 0
                self.arm.current_pose_msg = p
                rospy.logwarn(f"[TEST_POSE] Injected robot_pose: "
                             f"x={p.x:.3f}, y={p.y:.3f}, theta={p.theta:.3f}")

            scan_point = [{
                "mode": "pose",
                "x": x, "y": y, "z": z,
                "rx": rx, "ry": ry, "rz": rz,
                "speed": 50
            }]
            rospy.loginfo(f"[TEST_POSE] Target: ({x}, {y}, {z}), "
                         f"ori: ({rx}, {ry}, {rz})")
            self.arm.execute_scan_points(scan_point)
            return

        # ---------- STOP ----------
        if cmd.upper() == "STOP":
            rospy.logerr("[TASK] EMERGENCY STOP")

            self._stop_requested = True
            self._pending_task = None
            self._pending_task_name = None

            try:
                self.mobile.emergency_stop_robot()
            except Exception:
                pass

            try:
                self.arm.cancel()
            except Exception:
                pass

            self._current_task = None
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
            self._pending_task = task

            self._stop_requested = True

            try:
                self.mobile.preempt_stop_robot()
            except Exception:
                pass

            try:
                self.arm.cancel()
            except Exception:
                pass

            return

        # ---------- GOTO ----------
        if cmd.upper().startswith("GOTO"):
            parts = cmd.split()
            if len(parts) != 2:
                rospy.logerr("[TASK] Usage: GOTO <tag_id>")
                return

            tag_id = int(parts[1])
            task = self.task_mgr.build_goto_task(tag_id)

            rospy.logwarn(f"[TASK] Preempt → pending GOTO {tag_id}")

            self._pending_task_name = f"goto_{tag_id}"
            self._pending_task = task
            self._stop_requested = True

            try:
                self.mobile.preempt_stop_robot()
            except Exception:
                pass

            return

        # ---------- STATE ----------
        if cmd.upper() == "STATE":
            rospy.loginfo(f"[STATE] {self.state.name}")


    # ==========================================================
    # SCAN FINISHED CALLBACK
    # ==========================================================
    def _scan_done_cb(self, msg: Bool):
        if msg.data:
            self.scan_done = True


    # ==========================================================
    # MAIN LOOP
    # ==========================================================
    def run(self):
        rate = rospy.Rate(5)
        rospy.loginfo("[Executor] Main loop started")

        while not rospy.is_shutdown():

            # 正在执行任务
            if self._task_running:
                rate.sleep()
                continue

            # ★ 激活 pending task
            if self._current_task is None and self._pending_task is not None:
                rospy.loginfo(
                    f"[TASK] Activate pending task '{self._pending_task_name}'"
                )
                self._current_task = self._pending_task
                self._current_task_name = self._pending_task_name
                self._pending_task = None
                self._pending_task_name = None

            if self._current_task is None:
                rate.sleep()
                continue

            self._task_running = True
            self._stop_requested = False
            self.mobile.clear_stop_flag()

            self._run_task(
                self._current_task_name,
                self._current_task
            )

            self._current_task = None
            self._current_task_name = None
            self._task_running = False
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

            tag_id = item["tag"]
            do_scan = item.get("scan", False)

            # ---------- MOVE ----------
            self.state = MobileManipulatorState.MOVING
            rospy.loginfo(f"[TASK] Moving to tag {tag_id}")

            ok = self.mobile.move_to_tag(tag_id)
            if not ok:
                rospy.logwarn(
                    f"[TASK] Navigation failed at tag {tag_id}, task aborted safely"
                )
                self.state = MobileManipulatorState.ERROR
                return

            self.state = MobileManipulatorState.ARRIVED
            rospy.loginfo(f"[TASK] Arrived at tag {tag_id}")

            # ---------- SCAN ----------
            if do_scan:
                self.state = MobileManipulatorState.SCANNING
                rospy.loginfo(f"[TASK] Start scan at tag {tag_id}")

                scan_points = self.task_mgr.get_scan_points(
                    task_name, tag_id
                )

                self.scan_done = False
                self.arm.execute_scan_points(scan_points)

                while not self.scan_done and not self._stop_requested:
                    rospy.sleep(0.05)

                if self._stop_requested:
                    rospy.logwarn("[TASK] Scan interrupted")
                    return

                self.state = MobileManipulatorState.SCAN_DONE
                rospy.loginfo(f"[TASK] Scan finished at tag {tag_id}")

        rospy.loginfo("[TASK] Task finished normally")


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    executor = MobileManipulatorTaskExecutor()
    executor.run()
