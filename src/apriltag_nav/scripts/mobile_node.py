#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mobile_node — sole owner of the mobile base.

The last device to get its own node. Before 2026-08-11 `task_executor` held a
`MobileController` in-process and published `/cmd_vel` itself, which made the
drive the one device without an owner. That had three concrete costs:

  1. Two stop flags. `task_executor._stop_requested` (task loop) and
     `MobileController.stop_requested` (drive loop) are different variables, and
     every stop site had to set both by hand.
  2. `move_to_tag()` blocks, so the orchestrator's main loop sat inside it for
     the whole drive and could not tick.
  3. Retuning navigation meant restarting the whole stack.

This node wraps `mobile_controller.MobileController` **unchanged** — same rule
as arm_node/ArmController. The controller keeps Pure Pursuit, the S-curve ramp,
tag-relative pose estimation and the vision-stop decision; nothing is
reimplemented here.

⚠️ This node is the only publisher of `/cmd_vel` and `/robot_pose` in
apriltag_nav. `/cmd_vel` in particular has no arbitration below it — navifra's
`base_controller` obeys the last message it received, so a second publisher
would fight this one at 50 Hz with no error anywhere.

Interface
---------
  /mobile/goto_tag  (Int32)   start driving to a tag. Returns immediately;
                              watch /mobile/state for the outcome.
  /mobile/stop      (Trigger) EMERGENCY stop — zero velocity, latch
                              emergency_stop. Mirrors STOP.
  /mobile/cancel    (Trigger) preempt stop — zero velocity, no emergency latch.
                              Mirrors a task switch.
  /mobile/clear_stop(Trigger) clear both flags so the next move may start.
  /mobile/state     (String)  JSON, published at ~`rate_hz`.

Why goto is a topic and not a service
-------------------------------------
Navigation is the longest motion in the system — minutes across several tags —
and a rospy service call has no in-flight timeout, so if this node wedged, the
caller would block forever with no way out. The topic plus `/mobile/state`
polling gives `MobileClient` its own deadline. Same trade as the lift; see
`lift_client.py` and the CLAUDE.md section on it.

⚠️ The honest answer is that **actionlib is the right primitive** for this and
was not used because the other two clients do not use it either. If a third
long motion ever needs a client, convert all three rather than writing this
pattern a fourth time.

Stop services deliberately do NOT take the motion lock. A stop that waited for
the move it is trying to abort would never run.
"""

import json
import threading

import rospy
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Trigger, TriggerResponse

from apriltag_nav import utils
from apriltag_nav.paths import CONFIG_PATH, MAP_PATH
from apriltag_nav.map_manager import MapManager
from apriltag_nav.mobile_controller import MobileController
from apriltag_nav.navifra_devices import NavifraDevices


class MobileNode:

    def __init__(self):
        rospy.init_node('mobile_node', anonymous=False)

        robot_config = utils.load_config(CONFIG_PATH)
        self.map_mgr = MapManager(MAP_PATH)
        self.mobile = MobileController(config=robot_config,
                                       map_manager=self.map_mgr)

        # Own e-stop subscription rather than relying on task_executor to
        # forward one. The base is the most dangerous device here, and a stop
        # that only works while the orchestrator is healthy is not a stop.
        # Read-only use of NavifraDevices — this node writes no /lift/*.
        self.devices = NavifraDevices(
            robot_config.get('navifra', {}) or {},
            on_estop=self._on_estop,
        )

        rate_hz = float(rospy.get_param('~state_rate_hz', 5.0))

        self._motion_lock = threading.Lock()
        self._busy_what = None
        # Sequence number of the last COMPLETED move. MobileClient waits for
        # this to advance rather than checking a position, because navigation
        # has no measurable end state the way an absolute lift height does.
        self._seq = 0
        self._result = None

        self.pub_state = rospy.Publisher('/mobile/state', String,
                                         queue_size=1, latch=True)
        self.pub_busy = rospy.Publisher('/mobile/busy', Bool,
                                        queue_size=1, latch=True)

        rospy.Service('/mobile/stop', Trigger, self._srv_stop)
        rospy.Service('/mobile/cancel', Trigger, self._srv_cancel)
        rospy.Service('/mobile/clear_stop', Trigger, self._srv_clear_stop)

        rospy.Subscriber('/mobile/goto_tag', Int32, self._cb_goto_tag,
                         queue_size=1)

        rospy.on_shutdown(self._on_shutdown)

        rospy.loginfo("[Mobile] Ready — sole publisher of /cmd_vel")
        self._publish_state()
        self._timer = rospy.Timer(rospy.Duration(1.0 / max(rate_hz, 0.1)),
                                  self._publish_state)

    # ==========================================================
    # HELPERS
    # ==========================================================
    def _on_estop(self):
        """Fires in the subscriber thread on every false->true e-stop edge.

        The PILZ circuit already cut motor power; this makes the software side
        agree so a blocked move_to_tag() returns instead of polling out its
        timeout against a base that cannot move.
        """
        rospy.logerr("[Mobile] E-stop — cancelling navigation")
        try:
            self.mobile.emergency_stop_robot()
        except Exception as e:
            rospy.logerr(f"[Mobile] stop-on-estop failed: {e}")

    def _begin(self, what):
        if not self._motion_lock.acquire(blocking=False):
            return False
        self._busy_what = what
        self._publish_state()
        return True

    def _end(self):
        self._busy_what = None
        self._motion_lock.release()
        self._publish_state()

    def _finish(self, tag_id, ok, message):
        self._seq += 1
        self._result = {'seq': self._seq, 'tag': int(tag_id),
                        'ok': bool(ok), 'message': str(message)}

    # ==========================================================
    # COMMANDS
    # ==========================================================
    def _cb_goto_tag(self, msg):
        tag_id = int(msg.data)
        if self._busy_what is not None:
            rospy.logwarn(f"[Mobile] goto {tag_id} refused — {self._busy_what}")
            self._finish(tag_id, False, f"busy: {self._busy_what}")
            self._publish_state()
            return
        threading.Thread(target=self._do_goto, args=(tag_id,),
                         daemon=True).start()

    def _do_goto(self, tag_id):
        if not self._begin(f'goto tag {tag_id}'):
            self._finish(tag_id, False, "busy")
            self._publish_state()
            return
        try:
            ok = self.mobile.move_to_tag(tag_id)
            msg = (f"arrived at tag {tag_id}" if ok
                   else f"navigation to tag {tag_id} failed")
            if not ok and self.mobile.stop_requested:
                msg = f"navigation to tag {tag_id} cancelled"
            self._finish(tag_id, ok, msg)
        except Exception as e:
            rospy.logerr(f"[Mobile] goto {tag_id} raised: {e}")
            self._finish(tag_id, False, f"exception: {e}")
        finally:
            self._end()

    # ==========================================================
    # SERVICES — must not take the motion lock, see the module docstring
    # ==========================================================
    def _srv_stop(self, _req):
        rospy.logerr("[Mobile] EMERGENCY STOP")
        try:
            self.mobile.emergency_stop_robot()
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))
        self._publish_state()
        return TriggerResponse(success=True, message="emergency stop")

    def _srv_cancel(self, _req):
        rospy.logwarn("[Mobile] Preempt stop")
        try:
            self.mobile.preempt_stop_robot()
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))
        self._publish_state()
        return TriggerResponse(success=True, message="preempt stop")

    def _srv_clear_stop(self, _req):
        try:
            self.mobile.clear_stop_flag()
        except Exception as e:
            return TriggerResponse(success=False, message=str(e))
        self._publish_state()
        return TriggerResponse(success=True, message="stop flags cleared")

    # ==========================================================
    # STATE PUBLISHING
    # ==========================================================
    def _publish_state(self, _event=None):
        m = self.mobile
        state = {
            'busy':           self._busy_what,
            'seq':            self._seq,
            'result':         self._result,
            'last_known_tag': m.last_known_tag,
            'visible_tags':   sorted(m.detected_tags.keys()),
            'stop_requested': bool(m.stop_requested),
            'emergency_stop': bool(m.emergency_stop),
            'estop':          self.devices.estop_active,
        }
        try:
            self.pub_state.publish(String(json.dumps(state)))
            self.pub_busy.publish(Bool(self._busy_what is not None))
        except Exception:
            pass

    # ==========================================================
    # SHUTDOWN
    # ==========================================================
    def _on_shutdown(self):
        """Never leave the base rolling."""
        try:
            self.mobile.stop()
        except Exception:
            pass


if __name__ == '__main__':
    node = MobileNode()
    rospy.spin()
