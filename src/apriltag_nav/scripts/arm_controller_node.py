#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arm controller node — ROS front end for the Fairino FR10v6.

Sole owner of the arm. task_executor no longer instantiates ArmController; it
talks to this node through ArmClient (scripts/arm_client.py), which mirrors the
old in-process call surface.

Design note — this node WRAPS `arm_controllerrealwithscan_v2.ArmController`
without touching its logic. That controller carries the parts that matter and
are easy to lose in a rewrite:
  * TOOL_ID = 1 (vision_tip TCP) on every move — not the flange
  * q0 IK seed via GetInverseKinRef
  * the validated 4-DOF pose transform
  * 13-column incremental result CSV
  * Keyence closed-loop standoff with the activate threshold
A previous node-per-device attempt (the deleted scripts_ros/ tree) reimplemented
the controller instead of wrapping it and silently lost several of these — most
damagingly it used tool=0 (flange) rather than TOOL_ID=1, so every move had the
wrong TCP offset. Do not "simplify" this node by inlining the controller.

Interface
---------
Subscribes:
  /arm/scan_command   std_msgs/String   JSON list of scan points
  /arm/cancel         std_msgs/Bool     true = abort current motion/scan
Services:
  /arm/move_home      std_srvs/Trigger  synchronous; refused while busy
Publishes:
  /arm/status         std_msgs/String   "idle" | "busy" (latched)

The controller itself additionally subscribes /robot_pose and keyence/value and
publishes /scan_finished, /scan/ra_value, /scan/point_result, /scan/image.

ROS parameters
--------------
All of ArmController's ~params now resolve in THIS node's private namespace
(num_samples, delay_between_samples, stabilization_time, save_images,
output_dir, keyence_*, capture_service, ...). They used to sit on
mobile_manipulator_system — see mobile_manipulator.launch.
"""

import json
import os
import threading

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from apriltag_nav.arm_controllerrealwithscan_v2 import ArmController
from apriltag_nav.paths import MODEL_PATH as DEFAULT_MODEL_PATH


class ArmControllerNode:
    def __init__(self):
        rospy.init_node('arm_controller_node', anonymous=False)

        model_path = rospy.get_param('~model_path', DEFAULT_MODEL_PATH)
        robot_ip = rospy.get_param('~robot_ip', '192.168.58.2')

        self._status_pub = rospy.Publisher('/arm/status', String,
                                           queue_size=1, latch=True)
        self._publish_status('starting')

        self.arm = ArmController(robot_ip=robot_ip, model_path=model_path)

        # Serialises scan execution: one scan at a time, in a worker thread so
        # /arm/cancel is still delivered while a scan is running.
        self._exec_lock = threading.Lock()
        self._worker = None

        rospy.Subscriber('/arm/scan_command', String, self._cb_scan,
                         queue_size=1)
        rospy.Subscriber('/arm/cancel', Bool, self._cb_cancel, queue_size=1)
        rospy.Service('/arm/move_home', Trigger, self._srv_move_home)

        self._publish_status('idle')
        rospy.loginfo("[ArmNode] Ready — /arm/scan_command, /arm/cancel, "
                      "/arm/move_home")

    # ==========================================================
    # STATUS
    # ==========================================================
    def _publish_status(self, state):
        try:
            self._status_pub.publish(String(state))
        except Exception:
            pass

    def _busy(self):
        return self._worker is not None and self._worker.is_alive()

    # ==========================================================
    # SCAN
    # ==========================================================
    def _cb_scan(self, msg):
        try:
            scan_points = json.loads(msg.data)
        except Exception as e:
            rospy.logerr(f"[ArmNode] Bad /arm/scan_command JSON: {e}")
            # Unblock the caller, which is waiting on /scan_finished.
            self.arm.publish_done()
            return

        if not isinstance(scan_points, list):
            rospy.logerr("[ArmNode] /arm/scan_command must be a JSON list")
            self.arm.publish_done()
            return

        if self._busy():
            rospy.logwarn("[ArmNode] Busy — ignoring scan request")
            return

        self._worker = threading.Thread(
            target=self._run_scan, args=(scan_points,), daemon=True)
        self._worker.start()

    def _run_scan(self, scan_points):
        with self._exec_lock:
            self._publish_status('busy')
            try:
                self.arm.execute_scan_points(scan_points)
            except Exception as e:
                rospy.logerr(f"[ArmNode] Scan failed: {e}")
                # execute_scan_points normally publishes /scan_finished itself;
                # on an unexpected raise it may not have, and the caller would
                # wait forever.
                try:
                    self.arm.publish_done()
                except Exception:
                    pass
            finally:
                self._publish_status('idle')

    # ==========================================================
    # CANCEL
    # ==========================================================
    def _cb_cancel(self, msg):
        if not msg.data:
            return
        try:
            self.arm.cancel()
        except Exception as e:
            rospy.logerr(f"[ArmNode] Cancel failed: {e}")

    # ==========================================================
    # HOME
    # ==========================================================
    def _srv_move_home(self, _req):
        if self._busy():
            return TriggerResponse(
                success=False,
                message="arm busy with a scan — cancel it first")
        with self._exec_lock:
            self._publish_status('busy')
            try:
                self.arm.move_to_home()
                return TriggerResponse(success=True, message="home")
            except Exception as e:
                rospy.logerr(f"[ArmNode] move_to_home failed: {e}")
                return TriggerResponse(success=False, message=str(e))
            finally:
                self._publish_status('idle')

    # ==========================================================
    # SHUTDOWN
    # ==========================================================
    def shutdown(self):
        try:
            self.arm.cancel()
        except Exception:
            pass
        try:
            self.arm.shutdown()
        except Exception:
            pass
        self._publish_status('shutdown')


if __name__ == '__main__':
    node = ArmControllerNode()
    rospy.on_shutdown(node.shutdown)
    rospy.spin()
