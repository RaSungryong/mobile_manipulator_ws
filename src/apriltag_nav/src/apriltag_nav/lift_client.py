#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiftClient — talk to lifter_node from another node.

Same role ArmClient plays for the arm: task_executor must not touch `/lift/*`
or NavifraDevices' lift commands directly, because lifter_node is the sole
writer (CLAUDE.md). Everything here goes through `/lifter/*`.

⚠️ Terminology: `home()` here is **LIFT ORIGIN HOMING** — the lift descends to
its lower limit switch and the encoder origin is reset. It is unrelated to the
ARM HOME POSE (`ArmClient.move_to_home()`). See the comparison table in
CLAUDE.md before using either word on its own.

Why goto_mm() polls instead of calling a service
------------------------------------------------
lifter_node's synchronous surface is `std_srvs/Trigger` only, which carries no
argument, so an arbitrary height can only be requested over the
`/lifter/height_cmd` TOPIC — and that returns immediately. This class
reconstructs the synchronous call by watching `/lifter/state`.

A custom srv *was* always possible — `robot_msgs` has `message_generation` and
`add_service_files` and already ships a float-carrying `CaptureImages.srv`. The
reason not to use one is narrower: a rospy service call has no in-flight
timeout, so a wedged node blocks the caller forever, whereas this loop owns its
`move_timeout_s`. It is not a strong reason. `actionlib` is the right primitive
for a long, cancellable motion and remains the eventual answer for all three
device clients.

The subtle part is deciding when the move has *started*. Sampling `busy` right
after publishing can catch the pre-command state and report success while the
lift is still about to move — and it can genuinely be about to move a long way,
since an absolute move to a target at or below the current position takes a full
origin pass first. So `_saw_busy` is a **latch** set from the state callback,
not a snapshot: once any state message reports busy, the completion wait starts
from a message that is known to post-date the command.

A move that turns out to be a no-op never raises `busy` at all. That is what
`ack_timeout_s` bounds — after it elapses with no busy, the position is checked
directly rather than waiting out the full move timeout.
"""

import json
import threading

import rospy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class LiftClient:
    def __init__(self,
                 height_topic='/lifter/height_cmd',
                 state_topic='/lifter/state',
                 home_service='/lifter/home',
                 stop_service='/lifter/stop',
                 move_timeout_s=180.0,
                 home_timeout_s=180.0,
                 ack_timeout_s=3.0,
                 tol_mm=1.5,
                 connect_timeout_s=15.0):
        # A full stroke is ~28 s and an absolute move may take a whole origin
        # pass (down then up), so the default move timeout allows both plus
        # margin. Not tuned below that: a premature timeout here aborts a task.
        self.move_timeout_s = float(move_timeout_s)
        self.home_timeout_s = float(home_timeout_s)
        self.ack_timeout_s = float(ack_timeout_s)
        self.tol_mm = float(tol_mm)
        self._connect_timeout_s = float(connect_timeout_s)

        self._home_service = home_service
        self._stop_service = stop_service
        self._home_srv = None
        self._stop_srv = None

        self._lock = threading.Lock()
        self._state = None
        self._saw_busy = False

        self._pub_height = rospy.Publisher(height_topic, Float32, queue_size=1)
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
            # `busy` is the name of the motion in flight, or null when idle.
            if state.get('busy'):
                self._saw_busy = True

    @property
    def state(self):
        with self._lock:
            return dict(self._state) if self._state is not None else None

    @property
    def height_mm(self):
        st = self.state
        return None if st is None else st.get('height_mm')

    def wait_for_node(self, timeout_s=None):
        """Block until lifter_node has published a state. (ok, reason)."""
        timeout_s = self._connect_timeout_s if timeout_s is None else timeout_s
        deadline = rospy.get_time() + float(timeout_s)
        while rospy.get_time() < deadline and not rospy.is_shutdown():
            if self.state is not None:
                return True, "lift node up"
            rospy.sleep(0.1)
        return False, ("no /lifter/state — is lifter_node running?")

    # ==========================================================
    # COMMANDS
    # ==========================================================
    def goto_mm(self, mm, timeout_s=None):
        """Absolute height in mm above the origin. Blocks. Returns (ok, msg).

        lifter_node owns the anti-backlash rule, the soft travel clamp and
        the mm->count conversion; none of that is duplicated here.
        """
        timeout_s = self.move_timeout_s if timeout_s is None else float(timeout_s)
        target = float(mm)

        ok, why = self.wait_for_node()
        if not ok:
            return False, why

        with self._lock:
            self._saw_busy = False
        try:
            self._pub_height.publish(Float32(target))
        except Exception as e:
            return False, f"could not publish height command: {e}"

        rospy.loginfo(f"[LiftClient] Requested {target:.1f} mm — waiting")

        # Phase 1: has the node picked it up? See the class docstring.
        ack_deadline = rospy.get_time() + self.ack_timeout_s
        while rospy.get_time() < ack_deadline and not rospy.is_shutdown():
            with self._lock:
                if self._saw_busy:
                    break
            rospy.sleep(0.05)

        # Phase 2: wait for it to go idle again. Skipped when phase 1 saw no
        # motion at all, in which case the position check below is the answer.
        with self._lock:
            started = self._saw_busy
        if started:
            deadline = rospy.get_time() + timeout_s
            while not rospy.is_shutdown():
                st = self.state
                if st is not None and not st.get('busy'):
                    break
                if rospy.get_time() >= deadline:
                    return False, (f"lift move to {target:.1f} mm timed out "
                                   f"after {timeout_s:.0f}s (at "
                                   f"{self.height_mm} mm)")
                rospy.sleep(0.05)

        return self._verify(target, moved=started)

    def _verify(self, target_mm, moved):
        st = self.state
        if st is None:
            return False, "lift state lost"
        if st.get('estop'):
            return False, "e-stop active"
        if st.get('error'):
            return False, f"lift alarm: {st.get('alarm')}"
        pos_mm = st.get('height_mm')
        if pos_mm is None:
            return False, "lift position unknown (lift_driver running?)"
        if abs(pos_mm - target_mm) > self.tol_mm:
            return False, (f"lift ended at {pos_mm:.1f} mm, wanted "
                           f"{target_mm:.1f} mm")
        return True, (f"lift at {pos_mm:.1f} mm"
                      + ("" if moved else " (already there)"))

    def home(self, timeout_s=None):
        """LIFT ORIGIN HOMING — descend to the lower limit switch, zero the
        encoder. Synchronous; the node's service blocks until it finishes.

        This is the lift, NOT the arm's home pose.
        """
        timeout_s = self.home_timeout_s if timeout_s is None else float(timeout_s)
        if self._home_srv is None:
            try:
                rospy.wait_for_service(self._home_service,
                                       timeout=self._connect_timeout_s)
                self._home_srv = rospy.ServiceProxy(self._home_service, Trigger)
            except rospy.ROSException:
                return False, (f"{self._home_service} unavailable — is "
                               "lifter_node running?")
        try:
            resp = self._home_srv()
            return bool(resp.success), str(resp.message)
        except Exception as e:
            self._home_srv = None
            return False, f"lift origin homing call failed: {e}"

    def stop(self):
        """Halt the lift. Best effort — never raises."""
        try:
            if self._stop_srv is None:
                rospy.wait_for_service(self._stop_service, timeout=2.0)
                self._stop_srv = rospy.ServiceProxy(self._stop_service, Trigger)
            self._stop_srv()
        except Exception as e:
            rospy.logwarn(f"[LiftClient] stop failed: {e}")
