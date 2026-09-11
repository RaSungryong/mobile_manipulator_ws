#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lift_height.py
==============
READ-ONLY live lift extension, for consumers that must not be able to
command the lift.

`arm_calibration.arm_base_z` is measured with the lift AT ORIGIN, and the
lift adds up to ~343 mm on top of it. Pose-mode IK
(`arm_transform.transform_world_to_arm`) therefore needs the live height
or every world-frame target is off by exactly the lift extension — see
`docs/lift_arm_base_z_analysis.md`.

⚠️ Why not `LiftClient`: that class already exposes `height_mm`, but it is
the COMMAND proxy — it publishes `/lifter/height_cmd` and holds
home/stop service proxies. `arm_node` has no business being able to move
the lift, and CLAUDE.md's rule is that `lifter_node` is the sole writer
while *reading* stays open to anyone. This listener is the read-only half.

Source is `/lifter/height` (std_msgs/Float32, mm above origin, published
LATCHED by lifter_node), so a subscriber created at any time gets the
current value immediately rather than waiting for the next 2 Hz tick.
"""
import threading

import rospy
from std_msgs.msg import Float32


class LiftHeightListener(object):
    """Live lift extension in metres. `height_m()` returns None when
    lifter_node has never published — the caller decides what that means,
    because "unknown" and "at origin" are genuinely different states and
    conflating them is how a raised lift becomes a silent offset."""

    def __init__(self, topic='/lifter/height'):
        self._lock = threading.Lock()
        self._mm = None
        self._topic = topic
        self._sub = rospy.Subscriber(topic, Float32, self._cb, queue_size=1)

    def _cb(self, msg):
        with self._lock:
            self._mm = float(msg.data)

    @property
    def topic(self):
        return self._topic

    def height_mm(self):
        with self._lock:
            return self._mm

    def height_m(self):
        """Lift extension above origin in metres, or None if unknown."""
        mm = self.height_mm()
        return None if mm is None else mm / 1000.0

    def wait(self, timeout_s=2.0):
        """Block briefly for the first (latched) message. True if it came."""
        deadline = rospy.get_time() + float(timeout_s)
        while rospy.get_time() < deadline and not rospy.is_shutdown():
            if self.height_mm() is not None:
                return True
            rospy.sleep(0.05)
        return self.height_mm() is not None
