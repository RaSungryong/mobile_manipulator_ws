#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plan B — Basler Camera Publisher Node
======================================
Publishes Basler camera frames as a ROS Image topic so that other
nodes (e.g. arm_controller_ros) can subscribe without direct PyPylon access.

Topics published:
  /basler/image_raw  (sensor_msgs/Image, bgr8)

ROS parameters:
  ~fps    (float, default 30)  — publishing rate
  ~topic  (str)                — output topic name override
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from camera_interface import CameraInterface


class BaslerCameraNode:
    """Wraps CameraInterface and publishes frames as ROS Image messages."""

    def __init__(self):
        rospy.init_node('basler_camera_node', anonymous=False)

        self._fps   = float(rospy.get_param('~fps',   30.0))
        self._topic = rospy.get_param('~topic', '/basler/image_raw')

        self._bridge = CvBridge()
        self._pub    = rospy.Publisher(self._topic, Image, queue_size=1)

        self._camera = CameraInterface()
        if not self._camera.initialize():
            rospy.logerr("[BaslerCamera] Initialization failed — check camera connection")
            return

        self._camera.start_grabbing()
        rospy.loginfo(f"[BaslerCamera] Publishing to '{self._topic}' at {self._fps} Hz")

    def run(self):
        rate = rospy.Rate(self._fps)
        while not rospy.is_shutdown():
            frame = self._camera.grab_frame()
            if frame is not None:
                msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = rospy.Time.now()
                self._pub.publish(msg)
            rate.sleep()

    def shutdown(self):
        rospy.loginfo("[BaslerCamera] Shutting down...")
        self._camera.stop_grabbing()
        self._camera.close()


if __name__ == '__main__':
    node = BaslerCameraNode()
    rospy.on_shutdown(node.shutdown)
    node.run()
