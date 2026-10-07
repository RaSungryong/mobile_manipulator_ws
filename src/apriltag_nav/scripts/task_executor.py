#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import os
import re
import threading
import rospy
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
from apriltag_nav.paths import CONFIG_PATH, TASK_DIR, RA_MAP_DIR, ra_measured_path
from apriltag_nav import scan_resume

# Tasks the charging manager queues for itself (2026-09-09). They run through
# the ordinary task machinery (preempt, safety gate, /task_state, lamp) but
# are not user tasks: they never trigger return_after_task, and a user
# TASK/GOTO preempts them like anything else.
INTERNAL_TASKS = ('battery_return', 'battery_undock')
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
        self.task_mgr = TaskManager(TASK_DIR, result_dir=RA_MAP_DIR)

        # ---------- State ----------
        # Declared before the controllers: NavifraDevices' on_estop callback
        # fires from the ROS subscriber thread and touches these attributes.
        self._state = MobileManipulatorState.IDLE
        self.scan_done = False

        # ---------- Task state ----------
        self._current_task_name: Optional[str] = None
        self._current_task: Optional[List[dict]] = None
        # RESUME (2026-10-06): a pending / running task may be the REST of an
        # interrupted run — {'stem', 'csv_path', 'points': {tag: [...]}}.
        # _run_task then keeps that run's csv_path (no new timestamp) and
        # scans the planned points instead of the task's full list.
        self._pending_resume: Optional[dict] = None
        self._current_resume: Optional[dict] = None
        # group selection of the pending / running command (sorted list or None)
        self._pending_groups: Optional[list] = None
        self._current_groups: Optional[list] = None

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
        # ---------- Charging manager (2026-09-09, see _charge_tick) ----------
        self._charge_cfg = self._navifra_cfg.get('charging', {}) or {}
        self._charge_enabled = bool(self._charge_cfg.get('enabled', False))
        # unknown | working | charging | full | returning | stopped | dock_failed
        self._charge_phase = 'unknown'
        self._last_state_pct = None       # battery_pct last put on /task_state
        self._charge_next_eval = 0.0
        self._lamp_key = None
        self._task_completed = False
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

        # The registered tasks, so a UI never hard-codes a task name. Latched:
        # the set only changes on RELOAD_TASKS or a restart, and a viewer that
        # connects later must still get it.
        self._task_list_pub = rospy.Publisher("/task_list", String,
                                              queue_size=1, latch=True)

        # Reflect the initial state on the STATUS lamp.
        self._publish_status_color()
        self._publish_task_state()
        self._publish_task_list()

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
        if getattr(self, '_task_state_pub', None) is None:
            return      # constructor not finished (see _publish_status_color)
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
                # the run being RESUMED (its <task>_ra_map_<ts> stem), else None
                'resume_run': (self._current_resume or {}).get('stem')
                              if getattr(self, '_current_resume', None) else None,
                # the `groups=` selection of the running command, else None
                'groups_filter': getattr(self, '_current_groups', None),
                # Charging manager (2026-09-14, for robot_ui's CHARGE chip):
                # phase is the manager's own word, `charging` is the BMS's.
                'charge_phase': self._charge_phase,
                'charging': self._is_charging(),
                'battery_pct': self._charge_pct(),
                'stamp': rospy.get_time(),
            }
            self._task_state_pub.publish(String(json.dumps(payload)))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[Executor] task_state publish failed: {e}")

    def _publish_task_list(self):
        """Publish /task_list: every registered task with mode, tags, point
        counts and source files (TaskManager.describe_tasks). JSON for the
        same reason as /task_state."""
        try:
            payload = {
                'task_dir': TASK_DIR,
                'tasks': self.task_mgr.describe_tasks(),
                'stamp': rospy.get_time(),
            }
            self._task_list_pub.publish(String(json.dumps(payload)))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[Executor] task_list publish failed: {e}")

    def _reload_tasks(self):
        """Re-scan the task directory and re-register every task.

        Refused while a task is running or pending: the running task holds
        references into the old TaskManager's scan points, and swapping the
        manager under it would let the next group read from the new one.
        """
        if self._task_running or self._pending_task is not None:
            rospy.logerr("[Executor] RELOAD_TASKS refused: a task is running "
                         "or pending. STOP it first.")
            return False
        try:
            new_mgr = TaskManager(TASK_DIR, result_dir=RA_MAP_DIR)
        except Exception as e:
            rospy.logerr(f"[Executor] RELOAD_TASKS failed, keeping the old "
                         f"task set: {e}")
            return False
        self.task_mgr = new_mgr
        rospy.loginfo(f"[Executor] Tasks reloaded from {TASK_DIR}: "
                      f"{self.task_mgr.get_all_task_names()}")
        self._publish_task_list()
        return True

    def _lamp_key_for_state(self):
        name = self._state.name.lower()
        # SCAN_DONE / ARRIVED have no dedicated colour — fall back to idle.
        if name in ('scanning',):
            return 'scanning'
        if name in ('moving', 'arrived'):
            return 'moving'
        if name == 'error':
            return 'error'
        # IDLE: the charging manager owns the colour (user rule 2026-09-09:
        # red while charging, green once charged). 'charged' falls back to
        # idle's colour when not configured.
        if self._charge_enabled:
            if self._is_charging():
                return 'charging'
            if self._charge_phase == 'full':
                return 'charged' if self._status_colors.get('charged') else 'idle'
        return 'idle'

    def _publish_status_color(self):
        """Map the task state (and the charging phase) onto the RGB STATUS
        lamp. Never raises."""
        key = self._lamp_key_for_state()
        self._lamp_key = key
        color = self._status_colors.get(key)
        if not color:
            return
        # The e-stop callback can fire inside NavifraDevices' constructor,
        # before self.devices is assigned; the main loop re-drives the lamp
        # from the latched edge a moment later, so nothing is lost here.
        if getattr(self, 'devices', None) is None:
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
        self._pending_resume = None
        self._pending_groups = None
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

    def _check_estop_recovery(self):
        """E-stop RELEASED (true -> false on /safety/estop: the button turned
        out, or the PLC's RESET pressed after a power-up) with nothing running:
        ERROR -> IDLE, so the STATUS lamp leaves red without an operator
        command (user rule 2026-10-07). Until then ERROR was only left by the
        next TASK / GOTO / CHARGE / STOP, so every boot sat red from the
        executor's start until the first command even though the PLC had been
        reset minutes earlier. A task still in flight is not touched (the
        abort path ends it; a pending task is activated by this same tick).
        Returns True when the state was changed."""
        try:
            if not self.devices.take_estop_cleared_edge():
                return False
            if self.devices.estop_active:
                return False        # a newer message says active again
        except Exception:
            return False
        if self._state is not MobileManipulatorState.ERROR:
            rospy.loginfo(f"[Executor] E-stop cleared (state {self._state.name}, unchanged)")
            return False
        if self._task_running or self._current_task is not None:
            rospy.logwarn("[Executor] E-stop cleared while a task is still winding "
                          "down — state stays ERROR until it has")
            return False
        rospy.logwarn("[Executor] E-stop cleared, nothing running — ERROR -> IDLE")
        self._stop_requested = False      # the abort's flag; that stop is over
        self._progress_note = 'e-stop cleared'
        self.state = MobileManipulatorState.IDLE
        return True

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

    # ==========================================================
    # CHARGING MANAGER (2026-09-09)
    # ==========================================================
    # User rules: charge until `full_pct` (85 %), then stop charging
    # (/crevis/charging false) and come forward a little; at `return_pct`
    # (30 %) stop whatever is running and go back to the charger; after a
    # user task completes normally, go back to the charger too
    # (`return_after_task`); /crevis/charging true must be sent explicitly
    # after docking. Lamp: red while charging, green once charged.
    #
    # Everything here is decided in _charge_tick (run() loop) and DONE by
    # two internal tasks that go through the ordinary task machinery, so a
    # user TASK/GOTO can preempt them and the safety gate / lamp /
    # /task_state apply as usual:
    #   battery_return: lift origin home -> move_to_tag(dock) [-> reverse
    #                   dock_reverse_m] -> /crevis/charging true, confirmed
    #                   by BMS current within charge_confirm_s
    #   battery_undock: /crevis/charging false (wait for the current to
    #                   drop); the base STAYS on the dock (user rule
    #                   2026-10-06) unless undock_forward_m > 0
    # "Charging" is judged from the BMS (current into the pack or status
    # CHARGING), never from the relay feedback topic alone.
    def _is_charging(self):
        try:
            v = self.devices.charging_by_bms(
                float(self._charge_cfg.get('charge_current_min_a', 0.5)))
        except Exception:
            v = None
        return bool(v)

    def _charge_pct(self):
        try:
            return self.devices.battery_percent
        except Exception:
            return None

    def _queue_internal_task(self, name, items):
        """Make `name` the pending task, preempting whatever runs — the same
        path a TASK command takes."""
        self._pending_task_name = name
        self._pending_task = items
        self._pending_resume = None
        self._pending_groups = None
        if self._task_running:
            rospy.logwarn(f"[Charge] preempting '{self._current_task_name}' for '{name}'")
            self._stop_requested = True
            try:
                self.mobile.preempt_stop_robot()
            except Exception:
                pass
            try:
                self.arm.cancel()
            except Exception:
                pass

    def _battery_return_task(self):
        items = [{'lift_home': True},
                 {'tag': int(self._charge_cfg.get('dock_tag', self.task_mgr.START_TAG)), 'scan': False}]
        back = float(self._charge_cfg.get('dock_reverse_m', 0.0) or 0.0)
        if back > 0:
            items.append({'drive_m': -back})
        items.append({'charge_on': True})
        return items

    def _battery_undock_task(self):
        # Relay off only: the base stays where it is (user, 2026-10-06:
        # "undock 할때도 위치는 이동하지 말고 충전명령을 false 바꾼다"). The
        # forward move exists only while undock_forward_m is > 0 — with it
        # every UNDOCK -> CHARGE cycle was a 0.10 m forward + a 0.10 m
        # reverse hop, and CHARGE from the dock then always reversed.
        items = [{'charge_off': True}]
        fwd = float(self._charge_cfg.get('undock_forward_m', 0.0) or 0.0)
        if fwd > 0:
            items.append({'drive_m': fwd})
        return items

    def _charge_tick(self):
        """One evaluation of the charging rules; cheap, called every loop tick."""
        if not self._charge_enabled:
            return
        # The lamp follows the charging state even while IDLE (no state change)
        if self._lamp_key != self._lamp_key_for_state():
            self._publish_status_color()
            self._publish_task_state()      # charge_phase / charging changed
        now = rospy.get_time()
        if now < self._charge_next_eval:
            return
        self._charge_next_eval = now + float(self._charge_cfg.get('poll_s', 2.0))

        pct = self._charge_pct()
        if pct is None:
            return
        # Keep /task_state's battery_pct live: it is latched and otherwise
        # republished only on a state / lamp change, so a UI reading the
        # CHARGE chip from it drifted away from the BMS (user, 2026-09-15).
        last_pct = getattr(self, '_last_state_pct', None)
        if last_pct is None or abs(pct - last_pct) >= 0.5:
            self._last_state_pct = pct
            self._publish_task_state()
        full_pct = float(self._charge_cfg.get('full_pct', 85.0))
        return_pct = float(self._charge_cfg.get('return_pct', 30.0))
        # The automatic low-battery return (<= return_pct) is OFF unless
        # low_battery_return is true (user, 2026-09-15: "20 % 이하 자동
        # 복귀 발동 안 되게"). CHARGE on /task_command, the 85 % undock and
        # return_after_task are unaffected.
        low_return = bool(self._charge_cfg.get('low_battery_return', False))
        charging = self._is_charging()
        phase = self._charge_phase

        if self._task_running:
            # Only a USER task is abandoned for the battery; the return task
            # itself, or the undock, is left to finish.
            if (low_return and self._current_task_name not in INTERNAL_TASKS
                    and pct <= return_pct and phase != 'returning'):
                rospy.logwarn(f"[Charge] battery {pct:.1f}% <= {return_pct:.0f}%: "
                              "stopping work and returning to the charger")
                self._queue_internal_task('battery_return', self._battery_return_task())
                self._charge_phase = 'returning'
            return

        if self._pending_task is not None:
            return          # something is about to run; judge it once it runs

        if charging:
            if phase not in ('charging', 'full'):
                rospy.loginfo(f"[Charge] charging ({pct:.1f}%)")
                self._charge_phase = 'charging'
                phase = 'charging'
            if phase == 'charging' and pct >= full_pct:
                rospy.loginfo(f"[Charge] battery {pct:.1f}% >= {full_pct:.0f}%: "
                              "stopping the charge and coming forward")
                self._charge_phase = 'full'
                self._queue_internal_task('battery_undock', self._battery_undock_task())
            return

        # idle and NOT charging
        if phase in ('full', 'stopped', 'dock_failed', 'returning'):
            # full: undocked, waiting for work. stopped: the operator pressed
            # STOP — no unattended motion until a task runs again. dock_failed:
            # the last docking did not produce current; do not loop.
            # returning: the return task ended without current (failed /
            # preempted) — same as dock_failed until something else happens.
            return
        if low_return and pct <= return_pct:
            rospy.logwarn(f"[Charge] battery {pct:.1f}% <= {return_pct:.0f}%: returning to the charger")
            self._queue_internal_task('battery_return', self._battery_return_task())
            self._charge_phase = 'returning'

    def _run_service_item(self, item):
        """Non-navigation task items (charging manager). Returns bool."""
        if item.get('lift_home'):
            rospy.loginfo("[Charge] lift origin homing before the drive")
            ok, why = self.lift.home()
            if not ok:
                rospy.logerr(f"[Charge] lift origin homing failed — {why}")
            return bool(ok)
        if 'drive_m' in item:
            d = float(item['drive_m'])
            rospy.loginfo(f"[Charge] straight move {d:+.3f} m")
            self.state = MobileManipulatorState.MOVING
            ok = self.mobile.drive_distance(d, speed=self._charge_cfg.get('undock_speed'))
            if not ok:
                rospy.logerr("[Charge] straight move failed")
            return bool(ok)
        if item.get('charge_on'):
            rospy.loginfo("[Charge] docked — /crevis/charging true")
            self.devices.set_charging(True)
            confirm_s = float(self._charge_cfg.get('charge_confirm_s', 15.0))
            deadline = rospy.get_time() + confirm_s
            while rospy.get_time() < deadline and not rospy.is_shutdown():
                if self._is_charging():
                    self._charge_phase = 'charging'
                    rospy.loginfo(f"[Charge] charging confirmed by the BMS ({self._charge_pct()}%)")
                    self._publish_status_color()
                    return True
                if self._stop_requested:
                    return False
                rospy.sleep(0.2)
            rospy.logerr(f"[Charge] no charging current within {confirm_s:.0f}s after docking — "
                         "check the contacts; not retrying automatically")
            self._charge_phase = 'dock_failed'
            return False
        if item.get('charge_off'):
            rospy.loginfo("[Charge] /crevis/charging false")
            self.devices.set_charging(False)
            deadline = rospy.get_time() + 5.0
            while rospy.get_time() < deadline and not rospy.is_shutdown():
                if not self._is_charging():
                    break
                rospy.sleep(0.2)
            else:
                rospy.logwarn("[Charge] BMS still reports charging current 5 s after the stop command")
            self._publish_status_color()
            return True
        rospy.logerr(f"[TASK] unknown task item {item}")
        return False

    def _set_task_lift_height(self, mm, prev=None):
        """Raise/lower the lift to the height the task CSV asks for. (ok, why).

        Called at the first scan step and again at every group whose height
        differs from the one set before (`prev`, 2026-10-07 — one value per
        task until then): the joint angles of a group were solved at its base
        height. Between the groups of one height nothing touches the lift, so
        "hold" needs no enforcement beyond not commanding it.

        A DESCENT goes through lift origin homing first (then climbs to the
        target when it is not 0): the drive has backlash, so a count reached
        by descending is not the height the joint rows were solved at —
        "reach a scan height by homing and climbing" (CLAUDE.md, lift).

        Deliberately after arriving at the tag rather than before driving —
        a raised lift puts the arm's mass high while the base is moving.
        """
        if prev is not None and mm < prev:
            rospy.loginfo(f"[TASK] Lift {prev:.1f} -> {mm:.1f} mm: descending, so "
                          "lift origin homing first (backlash)")
            ok, why = self.lift.home()
            if not ok:
                rospy.logerr(f"[TASK] Lift origin homing before {mm:.1f} mm failed — {why}")
                return False, why
            if mm <= 0.0:
                rospy.loginfo(f"[TASK] {why}; lift at the origin for this group")
                return True, why
        rospy.loginfo(f"[TASK] Setting lift to {mm:.1f} mm before scanning")
        ok, why = self.lift.goto_mm(mm)
        if not ok:
            rospy.logerr(f"[TASK] Lift height {mm:.1f} mm not reached — {why}")
            return False, why
        rospy.loginfo(f"[TASK] {why}; holding until a group asks for another height")

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


        # ---------- RELOAD_TASKS ----------
        # Re-scan task/csv (new path-data files dropped in) without a restart.
        if cmd.upper() == "RELOAD_TASKS":
            self._reload_tasks()
            return

        # ---------- CHARGE / UNDOCK (operator, 2026-09-14) ----------
        # The charging manager's two internal tasks, on demand. CHARGE =
        # lift origin home -> drive to the dock tag (500) -> /crevis/charging
        # true -> wait for BMS current (the charger only starts on that
        # explicit true, user-confirmed). UNDOCK = /crevis/charging false,
        # base left on the dock (undock_forward_m 0 since 2026-10-06). Both
        # preempt whatever runs, like TASK. CHARGE while the base is already
        # on the dock drives nothing: move_to_tag sees tag 500 as the
        # current tag and only the relay is switched.
        # They do not need charging.enabled — that key gates only the
        # automatic rules — but the manager's phase is kept in step so its
        # rules read the situation correctly afterwards.
        if cmd.upper() == "CHARGE":
            rospy.logwarn("[Charge] operator CHARGE -> dock at tag "
                          f"{int(self._charge_cfg.get('dock_tag', self.task_mgr.START_TAG))} and charge")
            self._queue_internal_task('battery_return', self._battery_return_task())
            self._charge_phase = 'returning'
            return
        if cmd.upper() == "UNDOCK":
            fwd = float(self._charge_cfg.get('undock_forward_m', 0.0) or 0.0)
            rospy.logwarn("[Charge] operator UNDOCK -> /crevis/charging false"
                          + (f", come forward {fwd:.2f} m" if fwd > 0 else ", base stays on the dock"))
            self._queue_internal_task('battery_undock', self._battery_undock_task())
            # 'stopped': no unattended return until the next user task, and
            # a plain idle lamp — 'full' would light the "charged" colour
            # for a pack the operator merely unplugged.
            self._charge_phase = 'stopped'
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
            # No unattended charge return after an operator STOP until a task
            # runs again (or charging is seen).
            self._charge_phase = 'stopped'

            self._stop_requested = True
            self._pending_task = None
            self._pending_task_name = None
            self._pending_resume = None
            self._pending_groups = None

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
        # ---------- RESUME [<run>] (2026-10-06) ----------
        # Continue an interrupted run in ITS OWN result files: the finished
        # points (Ra map row + hand-measured row when the run collects) are
        # left out, the rest is scanned with the run's csv_path, so the Ra
        # map, the frame folder and the collect CSV grow instead of a new
        # run starting from the first point. No argument = the newest Ra
        # map with points left. Preempts like TASK.
        # `groups=104,105` (2026-10-07) on either command keeps only those
        # groups: the other unfinished groups are skipped (not driven to),
        # not marked done, so a later RESUME offers them again.
        if cmd.upper().split()[:1] == ["RESUME"]:
            parts, groups, err = scan_resume.parse_groups_arg(cmd.split())
            if err:
                rospy.logerr(f"[RESUME] {err}")
                return
            if len(parts) > 2:
                rospy.logerr("[RESUME] Usage: RESUME [<run stem>] [groups=104,105]")
                return
            plan = self._plan_resume(parts[1] if len(parts) == 2 else None, groups)
            if plan is None:
                return
            rospy.logwarn(f"[RESUME] Preempt → pending '{plan.task_name}' "
                          f"({plan.summary()})")
            self._pending_task_name = plan.task_name
            self._pending_task = plan.steps
            self._pending_resume = {'stem': plan.run_stem, 'csv_path': plan.csv_path,
                                    'points': plan.points_by_tag,
                                    'summary': plan.summary(),
                                    'groups': plan.groups_filter}
            self._pending_groups = plan.groups_filter
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

        if cmd.upper().startswith("TASK"):
            parts, groups, err = scan_resume.parse_groups_arg(cmd.split())
            if err:
                rospy.logerr(f"[TASK] {err}")
                return
            if len(parts) != 2:
                rospy.logerr("[TASK] Usage: TASK <task_name> [groups=104,105]")
                return

            task_name = parts[1]
            task = self.task_mgr.get_task(task_name)
            if not task:
                rospy.logerr(f"[TASK] Unknown task '{task_name}'")
                return
            why = scan_resume.check_groups(task, groups)
            if why:
                rospy.logerr(f"[TASK] '{task_name}': {why}")
                return
            if groups:
                task = scan_resume.filter_steps(task, groups)
                rospy.logwarn(f"[TASK] '{task_name}': groups {sorted(groups)} only — "
                              f"{len(task)} of {len(self.task_mgr.get_task(task_name))} steps")

            rospy.logwarn(f"[TASK] Preempt → pending '{task_name}'")

            self._pending_task_name = task_name
            self._pending_task = task
            self._pending_resume = None
            self._pending_groups = sorted(groups) if groups else None

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
            self._pending_resume = None
            self._pending_groups = None
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
    # RESUME (2026-10-06)
    # ==========================================================
    def _resume_plan_for(self, stem, groups=None):
        """ResumePlan for one run stem, or None when its task is not
        registered or a selected group is not the task's (logged). Pure
        bookkeeping — nothing moves."""
        task_name = scan_resume.task_of(stem)
        steps = self.task_mgr.get_task(task_name) if task_name else []
        if not steps:
            rospy.logerr(f"[RESUME] run '{stem}' belongs to no registered task "
                         f"('{task_name}') — RELOAD_TASKS, or check task/csv")
            return None
        why = scan_resume.check_groups(steps, groups)
        if why:
            rospy.logerr(f"[RESUME] run '{stem}' ({task_name}): {why}")
            return None
        csv_path = os.path.join(RA_MAP_DIR, stem + '.csv')
        done = scan_resume.done_points(csv_path, ra_measured_path(stem))
        pending = scan_resume.pending_points(ra_measured_path(stem))
        return scan_resume.plan_resume(
            stem, task_name, steps,
            self.task_mgr.scan_points.get(task_name, {}), done, csv_path, groups,
            pending=pending)

    def _plan_resume(self, stem=None, groups=None):
        """The plan for `stem`, or for the newest run with points left.
        With `groups`, the newest run is still chosen by ALL its points
        (a run whose selected groups are done is refused, not skipped —
        the operator named a selection that does not fit)."""
        if stem:
            stem = scan_resume.run_stem_of(stem)
            if not os.path.isfile(os.path.join(RA_MAP_DIR, stem + '.csv')):
                rospy.logerr(f"[RESUME] no Ra map {stem}.csv under {RA_MAP_DIR}")
                return None
            plan = self._resume_plan_for(stem, groups)
            if plan is not None and plan.nothing_left:
                rospy.logerr(f"[RESUME] nothing left to scan — {plan.summary()}")
                return None
            return plan
        plan = scan_resume.find_resumable(RA_MAP_DIR, self._resume_plan_for)
        if plan is None:
            rospy.logerr("[RESUME] no interrupted run found among the newest "
                         f"Ra maps under {RA_MAP_DIR}")
            return None
        if groups:
            plan = self._resume_plan_for(plan.run_stem, groups)
            if plan is not None and plan.nothing_left:
                rospy.logerr(f"[RESUME] nothing left in the selected groups — {plan.summary()}")
                return None
        return plan

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
            self._tick()
            rate.sleep()

    def _tick(self):
        """One pass of the main loop (split out so it can be driven offline)."""
        # Hardware e-stop aborts whatever is in flight, then falls through
        # to the idle path below. Checked every tick, task running or not.
        # Its release with nothing running takes ERROR back to IDLE.
        self._check_estop_abort()
        self._check_estop_recovery()
        self._warn_if_battery_low()
        self._charge_tick()

        # A task is currently executing — skip
        if self._task_running:
            return

        # Activate pending task
        if self._current_task is None and self._pending_task is not None:
            rospy.loginfo(
                f"[TASK] Activate pending task '{self._pending_task_name}'"
            )
            self._current_task = self._pending_task
            self._current_task_name = self._pending_task_name
            self._current_resume = getattr(self, '_pending_resume', None)
            self._current_groups = getattr(self, '_pending_groups', None)
            self._pending_task = None
            self._pending_task_name = None
            self._pending_resume = None
            self._pending_groups = None

        if self._current_task is None:
            return

        # Safety gate — refuse to start a task under an active e-stop.
        # (Once running, aborts come from _check_estop_abort above.)
        ok, _why = self._check_safety_gate(
            f"task '{self._current_task_name}'")
        if not ok:
            self._current_task = None
            self._current_task_name = None
            self.state = MobileManipulatorState.ERROR
            return

        task_name = self._current_task_name
        is_user_task = task_name not in INTERNAL_TASKS
        if is_user_task and self._charge_enabled:
            self._charge_phase = 'working'

        self._task_running = True
        self._stop_requested = False
        self._task_completed = False
        self.mobile.clear_stop_flag()

        # Ensure arm is at home before starting a new task
        # (move_to_home is idempotent — safe to call even if already home)
        rospy.loginfo("[TASK] Ensuring arm is at home before task start")
        self.arm.move_to_home()

        self._run_task(task_name, self._current_task)

        self._current_task = None
        self._current_task_name = None
        self._current_resume = None
        self._current_groups = None
        self._task_running = False
        self.state = MobileManipulatorState.IDLE

        if self._charge_enabled:
            if task_name == 'battery_return' and not self._task_completed:
                # navigation failed or the return was preempted: no loop —
                # the operator / the next command decides
                if self._charge_phase == 'returning':
                    self._charge_phase = 'dock_failed' if not self._stop_requested else 'stopped'
            elif (is_user_task and self._task_completed and not self._stop_requested
                  and self._pending_task is None
                  and bool(self._charge_cfg.get('return_after_task', True))
                  # a GOTO is an operator positioning command, not work: it
                  # never triggers the return (2026-09-09: 'GOTO 100' was
                  # followed by an unasked-for drive back to the dock)
                  and (not task_name.startswith('goto_')
                       or bool(self._charge_cfg.get('return_after_goto', False)))
                  and not self._is_charging()):
                rospy.loginfo(f"[Charge] task '{task_name}' completed — returning to the charger")
                self._queue_internal_task('battery_return', self._battery_return_task())
                self._charge_phase = 'returning'


    # ==========================================================
    # TASK EXECUTION
    # ==========================================================
    def _run_task(self, task_name: str, task_items: List[dict]):

        resume = getattr(self, '_current_resume', None)
        if resume:
            rospy.logwarn(f"[TASK] RESUME '{task_name}' — {resume.get('summary')}")
        else:
            rospy.loginfo(f"[TASK] Start task '{task_name}'")

        # Stamp a fresh timestamp into all scan points' csv_path so each task
        # execution produces a uniquely named result file. A RESUME keeps the
        # interrupted run's own csv_path (already set on its planned points)
        # so every writer appends to that run's files.
        if not resume:
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
        # not command the lift at all. Per GROUP since 2026-10-07: the lift is
        # (re-)set at a scan step whose group names a height different from
        # the one set before (`lift_set` = mm set so far, None = not yet).
        lift_target = self.task_mgr.get_lift_height(task_name)
        group_lift = getattr(self.task_mgr, 'get_group_lift_height', None)
        lift_set = None

        self._progress_index = 0
        self._progress_total = len(task_items)
        self._progress_tag = -1
        groups = getattr(self, '_current_groups', None)
        note = f"resumed {resume['stem']}" if resume else 'task started'
        if groups:
            note += f" (groups {','.join(str(g) for g in groups)})"
            rospy.logwarn(f"[TASK] groups {groups} only — the other groups are skipped, not done")
        self._publish_task_state(note=note)

        for idx, item in enumerate(task_items, start=1):

            if self._stop_requested:
                rospy.logwarn("[TASK] Task preempted safely")
                return

            if 'tag' not in item:
                # charging-manager items (lift home, straight move, relay)
                self._progress_index = idx
                if not self._run_service_item(item):
                    self.state = MobileManipulatorState.ERROR
                    return
                continue

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
                    # it once per group. Set at the first group and whenever a
                    # group's height differs from the one set before.
                    want = lift_target
                    if group_lift is not None:
                        g = group_lift(task_name, tag_id)
                        if g is not None:
                            want = float(g)
                    if lift_set is None or abs(want - lift_set) > 1e-6:
                        ok, _why = self._set_task_lift_height(want, prev=lift_set)
                        if not ok:
                            self.state = MobileManipulatorState.ERROR
                            return
                        lift_set = want
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

                if resume:
                    # the planned remainder: finished work points are driven
                    # through (joint) or left out (pose), csv_path = the run's
                    scan_points = resume['points'].get(tag_id, [])
                else:
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

        self._task_completed = True
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