#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tf_chain_publisher.py — the fixed transforms of config/tf/tf_chain.yaml,
plus the live lift, /robot_pose and the arm FK, as ROS TF — so rviz shows
the chain the code believes (2026-10-13, config stage 2).

OFF by default: `roslaunch apriltag_nav mobile_manipulator.launch
use_tf_publish:=true`, or `rosrun apriltag_nav tf_chain_publisher.py` next
to a running stack. Owns no device, publishes only /tf and /tf_static, and
NOTHING in the stack subscribes to it — it is a viewer, not a dependency.

Frames (docs/FRAMES_kr.md; the maths is apriltag_nav.tf_chain_frames):
    world -> mb              /robot_pose (both kinds; the live stream wins)
    mb    -> ab              inv(T_ab2mb) shifted by /lifter/state height_mm
    ab    -> ee              FK(q + dq) from /arm/state (joints), guarded as the
                             locator chain does; falls back to the controller pose
    ab    -> ee_ctrl         the controller's pose, only while dq is applied
    mb    -> fc, fc_level    static  (T_mb2fc; + ground-plane tilt = level camera)
    mb    -> mobile_base_urdf static (Rz 180: the planner URDF's cart frame)
    ee    -> tip, hc         static  (T_ee2tip, inv T_hc2ee)

Params
    ~rate_hz          10.0   dynamic frames are re-sent at this rate with the
                             latest inputs (TF needs a steady stream)
    ~publish_ee_ctrl  true   also publish ab->ee_ctrl while dq is applied

rviz: `rviz -d $(rospack find apriltag_nav)/config/tf_chain.rviz`.
"""
import json

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from robot_msgs.msg import ArmState, Pose2DWithFlag

from apriltag_nav.tf_chain_frames import ChainState, EE_CTRL, matrix_to_tq, static_frames


def _msg(parent, child, T, stamp):
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id = parent
    m.child_frame_id = child
    t, q = matrix_to_tq(T)
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = t
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
    return m


class TfChainPublisher(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param('~rate_hz', 10.0))
        self.publish_ee_ctrl = bool(rospy.get_param('~publish_ee_ctrl', True))
        self.state = ChainState(warn=lambda s: rospy.logwarn_throttle(30.0, '[tf_chain] ' + s))
        self.static_bc = tf2_ros.StaticTransformBroadcaster()
        self.dyn_bc = tf2_ros.TransformBroadcaster()

        now = rospy.Time.now()
        statics = static_frames()
        self.static_bc.sendTransform([_msg(p, c, T, now) for p, c, T in statics])
        for p, c, T in statics:
            rospy.loginfo('[tf_chain] static %s -> %s  t %s mm', p, c,
                          [round(float(v) * 1e3, 1) for v in T[:3, 3]])
        rospy.loginfo('[tf_chain] joint offsets %s; mb floor z %.3f m; dynamic frames at %.1f Hz',
                      'APPLIED (ee = FK(q + dq), ee_ctrl beside it)' if self.state.offsets_enabled()
                      else 'off (ee = the controller pose)', self.state.floor_z_m, self.rate_hz)

        rospy.Subscriber('/robot_pose', Pose2DWithFlag, self._on_pose, queue_size=5)
        rospy.Subscriber('/lifter/state', String, self._on_lift, queue_size=5)
        rospy.Subscriber('/arm/state', ArmState, self._on_arm, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 0.1)), self._tick)

    # --- inputs: never raise, a dead callback means a blind viewer ------------
    def _on_pose(self, msg):
        try:
            self.state.set_robot_pose(msg.x, msg.y, msg.theta)
        except Exception as e:
            rospy.logwarn_throttle(30.0, '[tf_chain] /robot_pose: %s', e)

    def _on_lift(self, msg):
        try:
            d = json.loads(msg.data)
            if not self.state.set_lift_mm(d.get('height_mm')):
                rospy.logwarn_throttle(60.0, '[tf_chain] /lifter/state has no height_mm yet — ab at lift 0')
        except Exception as e:
            rospy.logwarn_throttle(30.0, '[tf_chain] /lifter/state: %s', e)

    def _on_arm(self, msg):
        try:
            self.state.set_arm_state(list(msg.tcp_pose), list(msg.joints), bool(msg.pose_valid))
        except Exception as e:
            rospy.logwarn_throttle(30.0, '[tf_chain] /arm/state: %s', e)

    # --- output ---------------------------------------------------------------
    def _tick(self, _evt):
        try:
            now = rospy.Time.now()
            frames = self.state.dynamic_frames()
            if not self.publish_ee_ctrl:
                frames = [f for f in frames if f[1] != EE_CTRL]
            self.dyn_bc.sendTransform([_msg(p, c, T, now) for p, c, T in frames])
        except Exception as e:
            rospy.logerr_throttle(10.0, '[tf_chain] publish failed: %s', e)


def main():
    rospy.init_node('tf_chain_publisher')
    TfChainPublisher()
    rospy.spin()


if __name__ == '__main__':
    main()
