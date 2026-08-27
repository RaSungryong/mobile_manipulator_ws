#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MobileClient — talk to mobile_node from another node.

Third of the three device proxies, after ArmClient and LiftClient. It exposes
exactly the four methods `task_executor` used to call on an in-process
`MobileController` — `move_to_tag`, `emergency_stop_robot`,
`preempt_stop_robot`, `clear_stop_flag` — so no call site changed when the
drive moved into its own node.

How completion is detected
--------------------------
`move_to_tag()` blocks, but it cannot verify arrival the way `LiftClient` does.
The lift has a measurable end state (a count within tolerance of the target);
navigation does not — "did we arrive" is a judgement `MobileController` makes
from tag detections, and it is only ever expressed as the bool it returns.

So mobile_node stamps every completed move with an incrementing `seq` and
stores the outcome in `state['result']`. This client records `seq` before
publishing and waits for it to advance. That makes the handshake robust against
the two races the lift version had to reason about:

  * a state message that predates the command (the `seq` has not moved yet)
  * a move that finishes so fast it is never observed as busy (the `seq` has
    moved, so no busy latch is needed to confirm it ran)

⚠️ Like the lift, this assumes a **single commanding caller**. Two nodes both
publishing `/mobile/goto_tag` would each see the other's `seq` advance and
could report each other's results. task_executor is the only caller by design;
mobile_node also refuses a second move while one is in flight.
"""

import json
import threading

import rospy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger


class MobileClient:
    def __init__(self,
                 goto_topic='/mobile/goto_tag',
                 state_topic='/mobile/state',
                 stop_service='/mobile/stop',
                 cancel_service='/mobile/cancel',
                 clear_stop_service='/mobile/clear_stop',
                 move_timeout_s=600.0,
                 ack_timeout_s=5.0,
                 connect_timeout_s=15.0):
        # A task can cross several tags and each leg is a full pivot + pure
        # pursuit run, so the default is generous on purpose: a premature
        # timeout here aborts a task that was progressing fine.
        self.move_timeout_s = float(move_timeout_s)
        self.ack_timeout_s = float(ack_timeout_s)
        self._connect_timeout_s = float(connect_timeout_s)

        self._services = {
            'stop': stop_service,
            'cancel': cancel_service,
            'clear_stop': clear_stop_service,
        }
        self._proxies = {}

        self._lock = threading.Lock()
        self._state = None

        self._pub_goto = rospy.Publisher(goto_topic, Int32, queue_size=1)
        rospy.Subscriber(state_topic, String, self._cb_state, queue_size=1)

    # ==========================================================
    # STATE
    # ==========================================================
    def _cb_state(self, msg):
        try:
            state = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._state = state

    @property
    def state(self):
        with self._lock:
            return dict(self._state) if self._state is not None else None

    @property
    def last_known_tag(self):
        st = self.state
        return None if st is None else st.get('last_known_tag')

    def wait_for_node(self, timeout_s=None):
        """Block until mobile_node has published a state. (ok, reason)."""
        timeout_s = self._connect_timeout_s if timeout_s is None else timeout_s
        deadline = rospy.get_time() + float(timeout_s)
        while rospy.get_time() < deadline and not rospy.is_shutdown():
            if self.state is not None:
                return True, "mobile node up"
            rospy.sleep(0.1)
        return False, "no /mobile/state — is mobile_node running?"

    # ==========================================================
    # COMMANDS
    # ==========================================================
    def move_to_tag(self, tag_id, timeout_s=None):
        """Drive to an AprilTag. Blocks. Returns bool.

        Returns a bare bool, not (ok, msg), because that is what the in-process
        MobileController returned and task_executor branches on it directly.
        The reason for a failure is logged here instead.

        mobile_node owns path finding, Pure Pursuit and the arrival decision;
        none of that is duplicated here.
        """
        timeout_s = self.move_timeout_s if timeout_s is None else float(timeout_s)
        tag_id = int(tag_id)

        ok, why = self.wait_for_node()
        if not ok:
            rospy.logerr(f"[MobileClient] {why}")
            return False

        st = self.state
        seq_before = st.get('seq', 0) if st else 0

        try:
            self._pub_goto.publish(Int32(tag_id))
        except Exception as e:
            rospy.logerr(f"[MobileClient] could not publish goto: {e}")
            return False

        rospy.loginfo(f"[MobileClient] Requested tag {tag_id} — waiting")

        # Phase 1: has the node picked it up? Either it went busy, or it
        # already finished (seq advanced). Both mean the command landed.
        ack_deadline = rospy.get_time() + self.ack_timeout_s
        acked = False
        while rospy.get_time() < ack_deadline and not rospy.is_shutdown():
            st = self.state
            if st is not None and (st.get('busy') or st.get('seq', 0) > seq_before):
                acked = True
                break
            rospy.sleep(0.05)

        if not acked:
            rospy.logerr(f"[MobileClient] mobile_node never acknowledged tag "
                         f"{tag_id} within {self.ack_timeout_s:.0f}s")
            return False

        # Phase 2: wait for the seq to advance past what we saw. Keyed on seq
        # rather than on `busy` going false, so a move that completed inside
        # phase 1 is already satisfied here.
        deadline = rospy.get_time() + timeout_s
        while not rospy.is_shutdown():
            st = self.state
            if st is not None and st.get('seq', 0) > seq_before:
                break
            if rospy.get_time() >= deadline:
                rospy.logerr(f"[MobileClient] move to tag {tag_id} timed out "
                             f"after {timeout_s:.0f}s")
                return False
            rospy.sleep(0.05)

        result = (self.state or {}).get('result') or {}
        if result.get('tag') != tag_id:
            rospy.logwarn(f"[MobileClient] result is for tag "
                          f"{result.get('tag')}, expected {tag_id} — treating "
                          "as failure")
            return False
        if not result.get('ok'):
            rospy.logwarn(f"[MobileClient] {result.get('message')}")
            return False
        rospy.loginfo(f"[MobileClient] {result.get('message')}")
        return True

    # ==========================================================
    # STOPS — best effort, never raise. Call sites wrap them in try/except
    # already, but a stop that throws would be worse than one that logs.
    # ==========================================================
    def _call(self, key):
        name = self._services[key]
        try:
            if self._proxies.get(key) is None:
                rospy.wait_for_service(name, timeout=2.0)
                self._proxies[key] = rospy.ServiceProxy(name, Trigger)
            resp = self._proxies[key]()
            return bool(resp.success), str(resp.message)
        except Exception as e:
            self._proxies[key] = None      # force re-resolve if it restarted
            rospy.logwarn(f"[MobileClient] {name} failed: {e}")
            return False, str(e)

    def emergency_stop_robot(self):
        return self._call('stop')

    def preempt_stop_robot(self):
        return self._call('cancel')

    def clear_stop_flag(self):
        return self._call('clear_stop')
