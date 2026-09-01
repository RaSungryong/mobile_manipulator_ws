#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ArmClient — task_executor's handle on arm_node.

Deliberately exposes exactly the surface task_executor used when it constructed
ArmController in-process:

    current_pose_msg      (read + write)
    execute_scan_points(scan_points)
    cancel()
    move_to_home()

Keeping the surface identical is the point: the arm moved out of process without
task_executor's call sites changing, which is what keeps the migration
reviewable. Do not widen this class — if task_executor needs something new,
add it to the node's ROS interface first.

Asynchrony note
---------------
execute_scan_points() now returns as soon as the command is published, whereas
the in-process version blocked for the whole scan. task_executor already waited
on /scan_finished afterwards, so its logic is unaffected — but anything new that
assumes the call blocks would be wrong.
"""

import json

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_msgs.msg import Pose2DWithFlag


def _json_safe(obj):
    """Coerce numpy scalars/arrays so json.dumps cannot fail on scan points.

    Scan points come from pandas/numpy via task_manager, so values are often
    numpy.float64 / numpy.int64 rather than plain Python numbers. json.dumps
    raises TypeError on those, which would break every scan with an opaque
    error, so normalise here rather than trusting the producer.
    """
    if isinstance(obj, dict):
        return {str(k): _json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_json_safe(v) for v in obj]
    if isinstance(obj, (str, bool)) or obj is None:
        return obj
    if isinstance(obj, int):
        return int(obj)
    if isinstance(obj, float):
        return float(obj)
    # numpy scalar / anything else exposing .item()
    item = getattr(obj, 'item', None)
    if callable(item):
        try:
            return _json_safe(item())
        except Exception:
            pass
    # numpy array / iterable
    tolist = getattr(obj, 'tolist', None)
    if callable(tolist):
        try:
            return _json_safe(tolist())
        except Exception:
            pass
    return str(obj)


class ArmClient:
    def __init__(self,
                 scan_topic='/arm/scan_command',
                 cancel_topic='/arm/cancel',
                 home_service='/arm/move_home',
                 pose_topic='/robot_pose',
                 home_timeout_s=60.0):
        self._home_service = home_service
        self._home_timeout_s = float(home_timeout_s)

        self._scan_pub = rospy.Publisher(scan_topic, String, queue_size=1)
        self._cancel_pub = rospy.Publisher(cancel_topic, Bool, queue_size=1)

        # Only used by the TEST_POSE debug path — see the setter below.
        self._pose_pub = rospy.Publisher(pose_topic, Pose2DWithFlag,
                                         queue_size=1, latch=True)

        self._current_pose_msg = None
        rospy.Subscriber(pose_topic, Pose2DWithFlag, self._pose_cb,
                         queue_size=1)

        self._home_srv = None

    # ==========================================================
    # ROBOT POSE
    # ==========================================================
    def _pose_cb(self, msg):
        self._current_pose_msg = msg

    @property
    def current_pose_msg(self):
        """Latest /robot_pose, or None if nothing has published one.

        The client subscribes the same topic the arm node does, so this view
        matches what the arm will use for a pose-mode scan.
        """
        return self._current_pose_msg

    @current_pose_msg.setter
    def current_pose_msg(self, msg):
        """TEST_POSE injection.

        In-process this used to poke the controller's attribute directly. Across
        the node boundary the equivalent is to PUBLISH the synthetic pose, so
        the arm node receives it through its normal /robot_pose subscription.
        Latched, so it lands even if the arm node subscribes late.

        Safe because task_executor only injects when current_pose_msg is None —
        i.e. when nothing (including mobile_controller) is publishing poses, so
        there is no publisher to fight with.
        """
        self._current_pose_msg = msg
        if msg is None:
            return
        try:
            self._pose_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"[ArmClient] robot_pose injection failed: {e}")

    # ==========================================================
    # COMMANDS
    # ==========================================================
    def execute_scan_points(self, scan_points):
        """Send scan points to the arm node. Returns immediately.

        Completion arrives on /scan_finished, which task_executor already waits
        on via its _scan_done_event.
        """
        if not scan_points:
            rospy.logwarn("[ArmClient] Empty scan_points — nothing sent")
            return
        try:
            payload = json.dumps(_json_safe(scan_points))
        except Exception as e:
            rospy.logerr(f"[ArmClient] Could not encode scan points: {e}")
            return
        self._scan_pub.publish(String(payload))
        rospy.loginfo(f"[ArmClient] Sent {len(scan_points)} scan point(s)")

    def cancel(self):
        try:
            self._cancel_pub.publish(Bool(True))
        except Exception as e:
            rospy.logerr(f"[ArmClient] Cancel publish failed: {e}")

    def move_to_home(self):
        """Synchronous — blocks until the arm node reports home.

        Matches the in-process behaviour task_executor relies on before starting
        a task. Returns (bool, str); the old in-process method returned None and
        callers ignored it, so returning a value stays compatible.
        """
        if self._home_srv is None:
            try:
                rospy.wait_for_service(self._home_service,
                                       timeout=self._home_timeout_s)
                self._home_srv = rospy.ServiceProxy(self._home_service, Trigger)
            except rospy.ROSException:
                rospy.logerr(
                    f"[ArmClient] {self._home_service} unavailable — "
                    "is arm_node running?")
                return False, "arm node unavailable"
        try:
            resp = self._home_srv()
        except rospy.ServiceException as e:
            rospy.logerr(f"[ArmClient] move_to_home failed: {e}")
            self._home_srv = None      # force re-resolve if the node restarted
            return False, str(e)
        if not resp.success:
            rospy.logerr(f"[ArmClient] move_to_home refused: {resp.message}")
        return resp.success, resp.message
