#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Arm controller node — ROS front end for the Fairino FR10v6.

Sole owner of the arm. task_executor no longer instantiates ArmController; it
talks to this node through ArmClient (src/apriltag_nav/arm_client.py), which
mirrors the old in-process call surface. The operator UI reaches the arm the
same way and must never open its own Fairino RPC connection.

Design note — this node WRAPS `arm_controller.ArmController`
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
  /arm/move_cart      std_msgs/String   JSON {"pose":[x,y,z,rx,ry,rz],
                                              "vel":30, "acc":50}
  /arm/jog_cmd        std_msgs/String   JSON {"axis":"z", "delta":1.0,
                                              "vel":30, "acc":50}
Services:
  /arm/move_home      std_srvs/Trigger  synchronous; refused while busy
Publishes:
  /arm/state          robot_msgs/ArmState  ~10 Hz — live pose, joints,
                                           busy, motion_seq + result
  /arm/status         std_msgs/String   "idle" | "busy" (latched, LEGACY)

The controller itself additionally subscribes /robot_pose and keyence/value and
publishes /scan_finished, /scan/ra_value, /scan/point_result, /scan/image.

Why manual teaching lives here
------------------------------
move_cart / jog exist so an operator UI can hunt for a data-collection pose by
hand. They are deliberately NOT services: a Cartesian move takes seconds and is
cancellable, which by this workspace's own rule (duration / cancellability /
is there a result) argues for actionlib — but actionlib is not used anywhere in
this stack yet, and introducing it for one command would give the arm two
different completion protocols alongside /arm/scan_command + /scan_finished.
So they follow mobile_node's motion_seq model instead, which CLAUDE.md already
records as the better of the two shapes in use: the caller reads motion_seq,
publishes, and waits for it to advance. A stale pre-command state cannot satisfy
that wait. If actionlib is ever adopted, these three are the natural first move.

Both commands are refused — not queued — while a scan is running. Queueing a jog
behind a scan would move the arm some seconds after the operator let go of the
button, at a moment nobody is expecting motion.

ROS parameters
--------------
All of ArmController's ~params resolve in THIS node's private namespace
(num_samples, delay_between_samples, stabilization_time, save_images,
output_dir, keyence_*, capture_service, ...). They used to sit on
mobile_manipulator_system — see mobile_manipulator.launch.

This node's own:
  ~state_rate_hz   10.0   /arm/state publish rate
  ~default_vel     30.0   velocity for move_cart / jog when the JSON omits it
  ~default_acc     50.0   acceleration, likewise
  ~jog_max_step    50.0   largest single jog [mm or deg]; a typo'd step is the
                          realistic way to drive the tool into the workpiece
"""

import json
import threading

import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
from robot_msgs.msg import ArmState

from apriltag_nav.arm_controller import ArmController
from apriltag_nav.paths import MODEL_PATH as DEFAULT_MODEL_PATH


class ArmControllerNode:
    def __init__(self):
        rospy.init_node('arm_node', anonymous=False)

        model_path = rospy.get_param('~model_path', DEFAULT_MODEL_PATH)
        robot_ip = rospy.get_param('~robot_ip', '192.168.58.2')

        self._state_rate = float(rospy.get_param('~state_rate_hz', 10.0))
        self._default_vel = float(rospy.get_param('~default_vel', 30.0))
        self._default_acc = float(rospy.get_param('~default_acc', 50.0))
        self._jog_max_step = float(rospy.get_param('~jog_max_step', 50.0))

        # /arm/status is kept because docs, tools/test_all_devices.py and the
        # Korean guide all name it. New consumers should read /arm/state.
        self._status_pub = rospy.Publisher('/arm/status', String,
                                           queue_size=1, latch=True)
        self._state_pub = rospy.Publisher('/arm/state', ArmState,
                                          queue_size=1, latch=True)
        self._publish_status('starting')

        self.arm = ArmController(robot_ip=robot_ip, model_path=model_path)

        # Serialises execution: one motion at a time, in a worker thread so
        # /arm/cancel is still delivered while a scan or move is running.
        self._exec_lock = threading.Lock()
        self._worker = None

        # Completion bookkeeping. Guarded because the worker thread writes it
        # and the state timer reads it.
        self._seq_lock = threading.Lock()
        self._motion_seq = 0
        self._result_message = ''
        self._result_success = False
        self._state_word = 'starting'

        # Last successfully read pose, so a transient RPC hiccup shows up as
        # pose_valid=False for one cycle instead of a jump to zeros.
        self._last_pose = [0.0] * 6
        self._last_joints = [0.0] * 6
        self._pose_valid = False

        rospy.Subscriber('/arm/scan_command', String, self._cb_scan,
                         queue_size=1)
        rospy.Subscriber('/arm/cancel', Bool, self._cb_cancel, queue_size=1)
        rospy.Subscriber('/arm/move_cart', String, self._cb_move_cart,
                         queue_size=1)
        rospy.Subscriber('/arm/jog_cmd', String, self._cb_jog, queue_size=1)
        rospy.Service('/arm/move_home', Trigger, self._srv_move_home)

        self._publish_status('idle')
        rospy.Timer(rospy.Duration(1.0 / max(1.0, self._state_rate)),
                    self._tick_state)

        rospy.loginfo("[ArmNode] Ready — /arm/scan_command, /arm/cancel, "
                      "/arm/move_cart, /arm/jog_cmd, /arm/move_home; "
                      "state on /arm/state")

    # ==========================================================
    # STATUS / STATE
    # ==========================================================
    def _publish_status(self, state):
        self._state_word = state
        try:
            self._status_pub.publish(String(state))
        except Exception:
            pass

    def _busy(self):
        return self._worker is not None and self._worker.is_alive()

    def _bump(self, success, message):
        """Record one COMPLETED motion. Clients wait on motion_seq advancing."""
        with self._seq_lock:
            self._motion_seq += 1
            self._result_success = bool(success)
            self._result_message = str(message)
            seq = self._motion_seq
        if success:
            rospy.loginfo(f"[ArmNode] motion {seq}: {message}")
        else:
            rospy.logwarn(f"[ArmNode] motion {seq}: {message}")

    def _tick_state(self, _event):
        """Publish /arm/state. Never raises — a dead timer means a blind UI."""
        try:
            # Read the pose only when no motion holds the executor. The Fairino
            # RPC is one connection; issuing a read from this timer thread while
            # a worker thread is mid-MoveCart interleaves two request/response
            # pairs on it. Skipping the read costs a stale pose for the duration
            # of the move, which the UI shows as pose_valid=False rather than as
            # a wrong number.
            acquired = self._exec_lock.acquire(blocking=False)
            if acquired:
                try:
                    pose = self.arm.get_tcp_pose()
                    joints = self.arm.get_joints_deg()
                finally:
                    self._exec_lock.release()
                if pose is not None:
                    self._last_pose = pose
                if joints is not None:
                    self._last_joints = joints
                self._pose_valid = pose is not None
            else:
                self._pose_valid = False

            msg = ArmState()
            msg.header.stamp = rospy.Time.now()
            msg.state = self._state_word
            msg.busy = self._busy()
            msg.tcp_pose = self._last_pose
            msg.pose_valid = self._pose_valid
            msg.joints = self._last_joints
            with self._seq_lock:
                msg.motion_seq = self._motion_seq
                msg.result_message = self._result_message
                msg.result_success = self._result_success
            self._state_pub.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[ArmNode] state publish failed: {e}")

    def _start_worker(self, target, args, what):
        """Run `target` in the single executor thread, or refuse if one is up."""
        if self._busy():
            self._bump(False, f"refused: busy, dropped {what}")
            return False
        self._worker = threading.Thread(target=target, args=args, daemon=True)
        self._worker.start()
        return True

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

        self._start_worker(self._run_scan, (scan_points,), 'scan')

    def _run_scan(self, scan_points):
        with self._exec_lock:
            self._publish_status('busy')
            try:
                self.arm.execute_scan_points(scan_points)
                self._bump(True, f"scan {len(scan_points)} pts ok")
            except Exception as e:
                rospy.logerr(f"[ArmNode] Scan failed: {e}")
                self._bump(False, f"scan failed: {e}")
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
    # MANUAL TEACHING — absolute move / incremental jog
    # ==========================================================
    def _cb_move_cart(self, msg):
        try:
            req = json.loads(msg.data)
            pose = req['pose']
        except Exception as e:
            self._bump(False, f"bad /arm/move_cart JSON: {e}")
            return
        vel = float(req.get('vel', self._default_vel))
        acc = float(req.get('acc', self._default_acc))
        self._start_worker(self._run_move_cart, (pose, vel, acc), 'move_cart')

    def _run_move_cart(self, pose, vel, acc):
        with self._exec_lock:
            self._publish_status('busy')
            try:
                ok, message = self.arm.move_cart(pose, vel=vel, acc=acc)
                self._bump(ok, message)
            except Exception as e:
                rospy.logerr(f"[ArmNode] move_cart failed: {e}")
                self._bump(False, f"move_cart exception: {e}")
            finally:
                self._publish_status('idle')

    def _cb_jog(self, msg):
        try:
            req = json.loads(msg.data)
            axis = req['axis']
            delta = req['delta']
        except Exception as e:
            self._bump(False, f"bad /arm/jog_cmd JSON: {e}")
            return
        vel = float(req.get('vel', self._default_vel))
        acc = float(req.get('acc', self._default_acc))
        self._start_worker(self._run_jog, (axis, delta, vel, acc), 'jog')

    def _run_jog(self, axis, delta, vel, acc):
        with self._exec_lock:
            self._publish_status('busy')
            try:
                ok, message = self.arm.jog(axis, delta, vel=vel, acc=acc,
                                           max_step=self._jog_max_step)
                self._bump(ok, message)
            except Exception as e:
                rospy.logerr(f"[ArmNode] jog failed: {e}")
                self._bump(False, f"jog exception: {e}")
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
                self._bump(True, 'move_home ok')
                return TriggerResponse(success=True, message="home")
            except Exception as e:
                rospy.logerr(f"[ArmNode] move_to_home failed: {e}")
                self._bump(False, f"move_home failed: {e}")
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
