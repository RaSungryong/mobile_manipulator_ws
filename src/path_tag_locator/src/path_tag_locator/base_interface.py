"""
base_interface.py
=================
Base navigation through the main stack's ``mobile_node`` — replaces the
deleted in-package ``nav/`` tree (MapManager + RobotController +
Navigator), which duplicated apriltag_nav's navigation and published
``/cmd_vel`` itself.

``mobile_node`` is the only ``/cmd_vel`` publisher; we command it through
``MobileClient`` (the same proxy task_executor uses) and read the base
heading from ``/odom`` for the auto-view-pose bootstrap.

⚠️ Single-commander assumption (MobileClient docstring): do not issue
TASK/GOTO through task_executor while a calibration session is driving
the base — mobile_node refuses overlapping moves, so the session would
fail its nav step.
"""
import threading

import rospy
import tf.transformations as tft
from nav_msgs.msg import Odometry

from apriltag_nav.mobile_client import MobileClient


class BaseInterface:
    """Facade with the old ``Navigator`` call surface (goto / current
    tag-relative state), backed by mobile_node."""

    def __init__(self,
                 goto_topic='/mobile/goto_tag',
                 state_topic='/mobile/state',
                 stop_service='/mobile/stop',
                 cancel_service='/mobile/cancel',
                 clear_stop_service='/mobile/clear_stop',
                 odom_topic='/odom',
                 move_timeout_s=600.0):
        self.client = MobileClient(
            goto_topic=goto_topic,
            state_topic=state_topic,
            stop_service=stop_service,
            cancel_service=cancel_service,
            clear_stop_service=clear_stop_service,
            move_timeout_s=move_timeout_s,
        )
        self._theta_lock = threading.Lock()
        self._theta = 0.0
        rospy.Subscriber(odom_topic, Odometry, self._cb_odom, queue_size=1)

    # ------------------------------------------------------------------
    def _cb_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self._theta_lock:
            self._theta = float(yaw)

    @property
    def current_theta(self):
        """Base heading from /odom (rad) — feeds the auto-view-pose
        bootstrap, same signal the old Navigator exposed."""
        with self._theta_lock:
            return self._theta

    # ------------------------------------------------------------------
    def wait_for_node(self, timeout_s=15.0):
        return self.client.wait_for_node(timeout_s)

    def goto(self, target_id: int) -> bool:
        """Drive the base to ``target_id``. Returns True on success.

        ⚠️ NEVER auto-clears a latched EMERGENCY stop. An earlier version
        called ``clear_stop_flag()`` unconditionally here — which wipes
        the emergency latch mobile_node sets on STOP / a hardware e-stop
        edge, so a session that failed one entry on an e-stop would
        silently drive the base again on the next entry. Now: refuse
        while an emergency/e-stop is latched (the operator clears it
        deliberately, e.g. via task_executor's safety-gated path); only
        a leftover PREEMPT latch (task-switch cancel) is cleared, since
        that is the non-emergency flag this pre-move clear existed for.
        """
        st = self.client.state or {}
        if st.get('emergency_stop') or st.get('estop'):
            rospy.logerr('[BaseInterface] emergency stop latched — '
                         'refusing to drive; clear it deliberately '
                         'before resuming the session')
            return False
        if st.get('stop_requested'):
            self.client.clear_stop_flag()
        return bool(self.client.move_to_tag(int(target_id)))

    def current_tag_id(self):
        state = self.client.state or {}
        visible = state.get('visible_tags') or []
        return visible[0] if visible else state.get('last_known_tag')

    def stop(self):
        self.client.preempt_stop_robot()

    def shutdown(self):
        try:
            self.client.preempt_stop_robot()
        except Exception:
            pass
