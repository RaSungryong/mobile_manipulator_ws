"""
lift_listener.py
================
Live lift height for the transform chain.

``T_ab2mb`` in extrinsics.yaml is measured with the lift AT ORIGIN; the
lift adds up to ~343 mm on top and is NOT reflected there. Any locate /
calibration performed with the lift raised must therefore shift
``T_ab2mb``'s z by the live height — see ``chain.compensate_T_ab2mb``.

This listener reads ``/lifter/height`` (std_msgs/Float32, mm above
origin, latched by lifter_node). If lifter_node is not running, height
is unknown: we WARN and fall back to 0.0 (the pre-compensation
behaviour, correct only if the lift really is at origin).
"""
import threading

import rospy
from std_msgs.msg import Float32


class LiftHeightListener:
    def __init__(self, height_topic: str = "/lifter/height"):
        self._lock = threading.Lock()
        self._height_mm = None
        self._topic = height_topic
        rospy.Subscriber(height_topic, Float32, self._cb, queue_size=1)

    def _cb(self, msg):
        with self._lock:
            self._height_mm = float(msg.data)

    def height_m(self) -> float:
        """Lift extension above origin in metres; 0.0 (with a warning) if
        lifter_node has not published yet."""
        with self._lock:
            mm = self._height_mm
        if mm is None:
            rospy.logwarn_throttle(
                10.0,
                "[LiftHeightListener] no %s yet — assuming lift at origin "
                "(0 mm). If the lift is actually raised, every chain "
                "result is offset by that height.", self._topic)
            return 0.0
        return mm / 1000.0
