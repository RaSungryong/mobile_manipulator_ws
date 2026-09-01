#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import re
import threading
import rospy
import numpy as np
from datetime import datetime
from std_msgs.msg import Bool, String
from enum import Enum, auto
from typing import List, Optional

from apriltag_nav.task_manager import TaskManager
from apriltag_nav.navifra_devices import NavifraDevices


# Every device this node commands lives in its own node, reached through a
# client that mirrors the call surface the in-process controller had — so the
# call sites below never changed as each one was split out. To swap an
# implementation, change the import inside that device's node, not here.
from apriltag_nav.arm_client import ArmClient          # arm_node
from apriltag_nav.lift_client import LiftClient        # lifter_node
from apriltag_nav.mobile_client import MobileClient    # mobile_node
from apriltag_nav import utils
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
# PATHS — resolved centrally, see apriltag_nav/paths.py
# ============================================================
from apriltag_nav.paths import CONFIG_PATH, TASK_DIR
# The Ra model is loaded by arm_node (~model_path), not here.

# ============================================================
# TASK EXECUTOR
# ============================================================
class MobileManipulatorTaskExecutor:

    def __init__(self):

        rospy.init_node("mobile_manipulator_system")

        # ---------- Config ----------
        robot_config = utils.load_config(CONFIG_PATH)

        # ---------- Managers ----------
        # The tag map is not here any more: path finding belongs to whoever
        # drives, and that is mobile_node.
        self.task_mgr = TaskManager(TASK_DIR)

        # ---------- State ----------
        # Declared before the controllers: NavifraDevices' on_estop callback
        # fires from the ROS subscriber thread and touches these attributes.
        self._state = MobileManipulatorState.IDLE
        self.scan_done = False

        # ---------- Task state ----------
        self._current_task_name: Optional[str] = None
        self._current_task: Optional[List[dict]] = None

        self._pending_task_name: Optional[str] = None
        self._pending_task: Optional[List[dict]] = None

        self._stop_requested = False
        self._task_running = False

        # ---------- Navifra driver peripherals ----------
        # Safety e-stop feedback, VISION/STATUS lighting, battery, lift.
        # Constructed even when the driver is absent: every accessor degrades to
        # None/"unknown" and safe_to_move() only warns unless require_safety_link.
        #
        # on_estop fires in the subscriber thread, which is what lets an e-stop
        # interrupt a blocking navigation/scan wait — run() does not tick while
        # _run_task is executing. It is registered before self.mobile/self.arm
        # exist, so _abort_for_estop guards those calls.
        self._navifra_cfg = robot_config.get('navifra', {}) or {}
        self._status_colors = self._navifra_cfg.get('status_colors', {}) or {}
        self._battery_warned = False
        self.devices = NavifraDevices(
            self._navifra_cfg,
            on_estop=self._abort_for_estop,
        )

        # ---------- Device clients ----------
        # None of these is a controller. Each is a proxy onto the node that
        # owns the device; this process publishes to no device topic at all.
        #
        # Drive lives in mobile_node — it is the only publisher of /cmd_vel.
        self.mobile = MobileClient()

        # Arm lives in arm_node. The model path is that node's ~model_path.
        self.arm = ArmClient()

        # Lift lives in lifter_node. Only used when a task CSV carries a
        # lift_height column; tasks without one never touch the lift.
        self.lift = LiftClient()
        self._task_flow = robot_config.get('task_flow', {}) or {}
        self._lift_home_on_finish = bool(
            self._task_flow.get('lift_home_on_finish', True))

        # ---------- Debug mode (gates EXEC/EVAL RCE channel) ----------
        # The /task_command topic is unauthenticated; any node publishing to it
        # can otherwise execute arbitrary Python in this process. Default OFF.
        # Enable with: <param name="debug_mode" value="true"/> in launch.
        self._debug_mode = bool(rospy.get_param('~debug_mode', False))
        if self._debug_mode:
            rospy.logwarn(
                "[Executor] DEBUG MODE ENABLED: EXEC/EVAL commands accepted "
                "on /task_command. Do not run in production."
            )

        # ---------- Thread sync for scan completion ----------
        # _scan_done_cb fires in ROS callback thread; run-loop waits in another.
        self._scan_done_event = threading.Event()

        # ---------- ROS ----------
        rospy.Subscriber("/scan_finished", Bool, self._scan_done_cb, queue_size=1)
        rospy.Subscriber("/task_command", String, self._command_cb, queue_size=10)

        # ---------- Outward state ----------
        # The state machine used to be visible only through rospy.loginfo and
        # the three-colour STATUS lamp, so nothing outside this process could
        # tell IDLE from SCANNING — a GUI could send a TASK and then had no way
        # to know when it finished, or which group it was on. Latched, so a
        # viewer that connects mid-task sees the current state immediately
        # instead of waiting for the next transition.
        self._task_state_pub = rospy.Publisher("/task_state", String,
                                               queue_size=1, latch=True)
        # Progress within the running task. Not derivable from the state enum:
        # a task visits several tags and SCANNING is entered once per group.
        self._progress_index = 0
        self._progress_total = 0
        self._progress_tag = -1
        self._progress_note = ''

        # Reflect the initial state on the STATUS lamp.
        self._publish_status_color()
        self._publish_task_state()

        rospy.loginfo("[Executor] System ready (IDLE)")


    # ==========================================================
    # STATE  (property so every assignment drives the STATUS lamp)
    # ==========================================================
    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        changed = new_state is not self._state
        self._state = new_state
        if changed:
            self._publish_status_color()
            self._publish_task_state()

    def _publish_task_state(self, note=None):
        """Publish /task_state. Never raises — a blind GUI is worse than a log.

        JSON rather than a custom .msg on purpose: this is the one place whose
        shape is still moving (the block-coding GUI will want fields nobody has
        named yet), and a JSON string can gain a key without a rebuild of every
        consumer. Freeze it into robot_msgs once the GUI stops changing it.
        """
        if note is not None:
            self._progress_note = note
        try:
            payload = {
                'state': self._state.name,
                # IDLE alone does not mean "finished": it is also the state
                # before anything has ever run. `task` is None in that case.
                'task': self._current_task_name,
                'group_index': self._progress_index,
                'group_total': self._progress_total,
                'tag_id': self._progress_tag,
                'note': self._progress_note,
                'stop_requested': bool(self._stop_requested),
                'stamp': rospy.get_time(),
            }
            self._task_state_pub.publish(String(json.dumps(payload)))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[Executor] task_state publish failed: {e}")

    def _publish_status_color(self):
        """Map the task state onto the RGB STATUS lamp. Never raises."""
        name = self._state.name.lower()
        # SCAN_DONE / ARRIVED have no dedicated colour — fall back to idle.
        if name in ('scanning',):
            key = 'scanning'
        elif name in ('moving', 'arrived'):
            key = 'moving'
        elif name == 'error':
            key = 'error'
        else:
            key = 'idle'
        color = self._status_colors.get(key)
        if not color:
            return
        try:
            self.devices.set_status_color(color)
        except Exception as e:
            rospy.logwarn(f"[Executor] STATUS lamp update failed: {e}")

    # ==========================================================
    # SAFETY / BATTERY GATES
    # ==========================================================
    def _check_safety_gate(self, what):
        """(bool, reason) — refuse to start motion/scan when unsafe."""
        try:
            ok, why = self.devices.safe_to_move()
        except Exception as e:
            rospy.logwarn(f"[Executor] safety gate check failed: {e}")
            return True, "safety check unavailable"
        if not ok:
            rospy.logerr(f"[Executor] {what} refused — {why}")
        return ok, why

    def _check_estop_abort(self):
        """True if a hardware e-stop fired; drives the same path as STOP."""
        try:
            if not self.devices.take_estop_edge():
                return False
        except Exception:
            return False
        rospy.logerr("[Executor] Hardware e-stop detected — aborting task")
        self._abort_for_estop()
        return True

    def _abort_for_estop(self):
        """Hardware already cut motor power; bring ROS-side state in line.

        Consumes the latched edge so the _check_estop_abort() backstop in run()
        does not abort a second time for the same event.
        """
        try:
            self.devices.take_estop_edge()
        except Exception:
            pass
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
        self.state = MobileManipulatorState.ERROR

    def _warn_if_battery_low(self):
        try:
            if self.devices.battery_low():
                if not self._battery_warned:
                    pct = self.devices.battery_percent
                    rospy.logwarn(
                        f"[Executor] Battery low: {pct:.1f}% "
                        f"(< {self._navifra_cfg.get('low_battery_pct', 20.0)}%)"
                    )
                    self._battery_warned = True
            else:
                self._battery_warned = False
        except Exception:
            pass

    def _set_task_lift_height(self, mm):
        """Raise/lower the lift to the height the task CSV asks for. (ok, why).

        Called once, at the first scan step, and then left alone for the rest
        of the task: the joint angles were solved at this base height, so it
        has to stay put between groups. Nothing else in the task loop touches
        the lift, so "hold" needs no enforcement beyond not commanding it.

        Deliberately after arriving at the first tag rather than before driving
        — a raised lift puts the arm's mass high while the base is moving.
        """
        rospy.loginfo(f"[TASK] Setting lift to {mm:.1f} mm before scanning")
        ok, why = self.lift.goto_mm(mm)
        if not ok:
            rospy.logerr(f"[TASK] Lift height {mm:.1f} mm not reached — {why}")
            return False, why
        rospy.loginfo(f"[TASK] {why}; holding for the rest of the task")

        # One-shot conflict report. The per-group guard is skipped for this
        # task (see _run_task), so this is the only place it gets said.
        target = self._navifra_cfg.get('scan_height_counts')
        guard = str(self._navifra_cfg.get('scan_height_guard', 'warn')).lower()
        if target is not None and guard != 'off':
            rospy.logwarn(
                f"[TASK] Task CSV asks for {mm:.1f} mm but "
                f"navifra.scan_height_counts is {target} counts — two sources "
                "of truth for the lift height. Pose-mode IK uses the constant "
                "arm_calibration.arm_base_z and will be off by the difference; "
                "joint-mode scans are unaffected. See "
                "docs/lift_arm_base_z_analysis.md."
            )
        return True, why

    def _finish_task(self, task_name):
        """End-of-task sequence: arm home pose, then lift origin homing.

        Only on normal completion. A preempt is a request to do something else
        immediately, and a navigation/scan failure leaves the robot where a
        human needs to look at it — neither should trigger unattended motion.

        The robot deliberately stays where it finished. Driving back is the
        separate `go_home` task, so a caller can queue "scan, then go home" or
        "scan, then scan again" from the same building blocks.
        """
        rospy.loginfo("[TASK] Task finished normally")
        # Say so before the lift tail runs. A GUI queueing the next block reads
        # "completed" here; the state enum alone cannot distinguish a normal
        # finish from a preempt, since both end up back at IDLE.
        self._publish_task_state(note=f"task '{task_name}' completed")

        if not (self._lift_home_on_finish and
                self.task_mgr.get_lift_height(task_name) is not None):
            return

        # The lift is about to move underneath the arm. The arm already homes
        # itself after each scan group, so this guards a cancelled or scan-less
        # path having left it extended, rather than being routine.
        try:
            self.arm.move_to_home()
        except Exception as e:
            rospy.logwarn(f"[TASK] Arm home before finish failed: {e}")

        rospy.loginfo("[TASK] Lift origin homing (end of task)")
        ok, why = self.lift.home()
        if not ok:
            rospy.logerr(f"[TASK] Lift origin homing failed — {why}")
            self.state = MobileManipulatorState.ERROR

    def _check_lift_scan_height(self):
        """Guard the constant-arm_base_z assumption against lift movement.

        arm_calibration.arm_base_z is a CONSTANT, so a lift at a different
        height silently invalidates every pose IK result. See
        docs/lift_arm_base_z_analysis.md. Returns (ok, reason).
        """
        guard = str(self._navifra_cfg.get('scan_height_guard', 'warn')).lower()
        target = self._navifra_cfg.get('scan_height_counts')
        if guard == 'off' or target is None:
            return True, "guard disabled"
        try:
            pos = self.devices.lift_position
            if pos is None:
                msg = "lift position unknown (lift_driver running?)"
            elif self.devices.lift_at(target):
                return True, "at scan height"
            else:
                msg = (f"lift at {pos} counts, calibrated scan height is "
                       f"{target} — pose IK will be off by the difference")
        except Exception as e:
            msg = f"lift height check failed: {e}"
        if guard == 'refuse':
            rospy.logerr(f"[Executor] Scan refused — {msg}")
            return False, msg
        rospy.logwarn(f"[Executor] {msg}")
        return True, msg

    # ==========================================================
    # COMMAND CALLBACK
    # ==========================================================
    def _command_cb(self, msg: String):
        cmd = msg.data.strip()
        rospy.logwarn(f"[TASK COMMAND] {cmd}")

        # ==========================================================
        # Debug-only commands: EXEC / EVAL
        # Disabled by default. Enable via ROS param ~debug_mode:=true
        # Example: EXEC self.mobile.move_to_tag(3)
        # Example: EVAL self.state.name
        # ==========================================================
        if cmd.upper().startswith("EXEC ") or cmd.upper().startswith("EVAL "):
            if not self._debug_mode:
                rospy.logerr(
                    "[Executor] EXEC/EVAL command rejected. "
                    "Set ROS param ~debug_mode:=true to enable. "
                    f"Rejected command: {cmd[:80]}"
                )
                return

            if cmd.upper().startswith("EXEC "):
                code_to_run = cmd[5:].strip()
                rospy.logwarn(f"[DEBUG EXEC] Running: {code_to_run}")
                try:
                    exec(code_to_run, globals(), locals())
                    rospy.loginfo("[DEBUG EXEC] Success")
                except Exception as e:
                    rospy.logerr(f"[DEBUG EXEC] Failed: {e}")
                return

            # EVAL branch
            code_to_eval = cmd[5:].strip()
            try:
                res = eval(code_to_eval, globals(), locals())
                rospy.loginfo(f"[DEBUG EVAL] result => {res}")
            except Exception as e:
                rospy.logerr(f"[DEBUG EVAL] Failed: {e}")
            return


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

            try:
                self.arm.cancel()
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
            self._scan_done_event.set()


    # ==========================================================
    # MAIN LOOP
    # ==========================================================
    def run(self):
        rate = rospy.Rate(5)
        rospy.loginfo("[Executor] Main loop started")

        while not rospy.is_shutdown():

            # Hardware e-stop aborts whatever is in flight, then falls through
            # to the idle path below. Checked every tick, task running or not.
            self._check_estop_abort()
            self._warn_if_battery_low()

            # A task is currently executing — skip
            if self._task_running:
                rate.sleep()
                continue

            # Activate pending task
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

            # Safety gate — refuse to start a task under an active e-stop.
            # (Once running, aborts come from _check_estop_abort above.)
            ok, _why = self._check_safety_gate(
                f"task '{self._current_task_name}'")
            if not ok:
                self._current_task = None
                self._current_task_name = None
                self.state = MobileManipulatorState.ERROR
                rate.sleep()
                continue

            self._task_running = True
            self._stop_requested = False
            self.mobile.clear_stop_flag()

            # Ensure arm is at home before starting a new task
            # (move_to_home is idempotent — safe to call even if already home)
            rospy.loginfo("[TASK] Ensuring arm is at home before task start")
            self.arm.move_to_home()

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

        # Stamp a fresh timestamp into all scan points' csv_path so each task
        # execution produces a uniquely named result file
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        for tag_points in self.task_mgr.scan_points.get(task_name, {}).values():
            for sp in tag_points:
                orig = sp.get("csv_path", "")
                if orig:
                    base, ext = os.path.splitext(orig)
                    # Strip any previously injected timestamp suffix of the
                    # form _YYYYMMDD_HHMMSS (15 digits separated by '_').
                    base = re.sub(r'_\d{8}_\d{6}$', '', base)
                    sp["csv_path"] = f"{base}_{ts}{ext}"

        # Height from the CSV's lift_height column; None means this task does
        # not command the lift at all.
        lift_target = self.task_mgr.get_lift_height(task_name)
        lift_set = False

        self._progress_index = 0
        self._progress_total = len(task_items)
        self._progress_tag = -1
        self._publish_task_state(note='task started')

        for idx, item in enumerate(task_items, start=1):

            if self._stop_requested:
                rospy.logwarn("[TASK] Task preempted safely")
                return

            tag_id = item["tag"]
            do_scan = item.get("scan", False)

            self._progress_index = idx
            self._progress_tag = tag_id

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
                if lift_target is not None:
                    # The CSV names the height, so it is the authority here and
                    # the scan_height_counts guard below would only second-guess
                    # it once per group. Set once, then held.
                    if not lift_set:
                        ok, _why = self._set_task_lift_height(lift_target)
                        if not ok:
                            self.state = MobileManipulatorState.ERROR
                            return
                        lift_set = True
                else:
                    # Lift must sit at the height the arm transform was
                    # calibrated at, otherwise every pose IK result is offset.
                    # Warn or refuse per navifra.scan_height_guard.
                    lift_ok, _lift_why = self._check_lift_scan_height()
                    if not lift_ok:
                        self.state = MobileManipulatorState.ERROR
                        return

                self.state = MobileManipulatorState.SCANNING
                rospy.loginfo(f"[TASK] Start scan at tag {tag_id}")

                scan_points = self.task_mgr.get_scan_points(
                    task_name, tag_id
                )

                self.scan_done = False
                self._scan_done_event.clear()
                self.arm.execute_scan_points(scan_points)

                # Wait on Event (signaled by _scan_done_cb) with poll for stop.
                while not self._stop_requested:
                    if self._scan_done_event.wait(timeout=0.05):
                        break

                if self._stop_requested:
                    rospy.logwarn("[TASK] Scan interrupted")
                    return

                self.state = MobileManipulatorState.SCAN_DONE
                rospy.loginfo(f"[TASK] Scan finished at tag {tag_id}")

        self._finish_task(task_name)


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    executor = MobileManipulatorTaskExecutor()
    # LED publishers are latched, so without this the STATUS/VISION lamps stay
    # lit after the node exits.
    rospy.on_shutdown(executor.devices.shutdown)
    executor.run()