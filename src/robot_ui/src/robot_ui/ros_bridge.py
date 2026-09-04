#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RosBridge — the ONLY place in robot_ui that talks to ROS.

This file is the architectural boundary, not a convenience wrapper. The UI this
package replaces held a pypylon camera, a pyrealsense2 pipeline, a Fairino RPC
connection and a pyserial handle in the GUI process, which made it a second
owner of four devices that already have owners here. Running both at once meant
fighting for the arm and the camera with nothing in either process aware of it.

So: no other module in robot_ui may import rospy, and this module may not
import PyQt widgets. If a new capability needs a device, it gets a topic or a
service on the owner node — it does not get a direct handle here.

Threading
---------
rospy callbacks arrive on rospy's own threads; Qt widgets may only be touched
from the GUI thread. Every inbound message is therefore re-emitted as a Qt
signal, which Qt queues across the thread boundary for us. Nothing in this file
touches a widget.

Outbound service calls BLOCK. Call them from a worker thread (see
main_window.CallWorker), never straight from a button handler, or the window
freezes for the duration — a Basler capture is seconds and a MoveCart longer.

Motion completion
-----------------
/arm/move_cart and /arm/jog_cmd are fire-and-forget topics; completion arrives
as ArmState.motion_seq advancing. `wait_for_motion()` implements that wait the
way MobileClient does: read the seq BEFORE publishing, then wait for something
strictly newer. A stale pre-command state cannot satisfy it.
"""

import json
import threading

import numpy as np
import rospy
from cv_bridge import CvBridge
from PyQt5.QtCore import QObject, pyqtSignal

from sensor_msgs.msg import BatteryState, Image
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger, SetBool

from robot_msgs.msg import AprilTagDetectionArray, ArmState
from robot_msgs.srv import CaptureImages, PredictRa


# Live-stream cameras. The Basler is deliberately NOT here — basler_camera_node
# keeps it Close()d between captures (heat, sensor lifetime, and the VISION lamp
# must only be lit while the shutter is open), so /basler/image_raw carries the
# last captured frame rather than a stream. See preview_* below.
STREAM_CAMERAS = {
    'front_cam': '/front_cam/color/image_raw',
    'side_cam': '/side_cam/color/image_raw',
    'hand_cam': '/hand_cam/color/image_raw',
}
# robot_camera_node's annotated frame per camera: amber crosshair on the
# optical axis, and per tag its ID, px + bearing offset from the crosshair,
# rpy / tilt / range. Rendered by the node ONLY while something subscribes,
# so showing it costs nothing when the box is unticked. The UI subscribes to
# either this or the raw stream per camera — never both (see
# set_stream_source), so a camera pane is one subscription either way.
OVERLAY_TOPICS = {name: f'/{name}/tag_overlay' for name in STREAM_CAMERAS}

ARM_AXES = ('x', 'y', 'z', 'rx', 'ry', 'rz')


class RosBridge(QObject):
    """Qt-facing view of the robot. Owns no device; calls the nodes that do."""

    # name, bgr image
    image_received = pyqtSignal(str, object)
    arm_state = pyqtSignal(dict)
    task_state = pyqtSignal(dict)
    lift_state = pyqtSignal(dict)
    mobile_state = pyqtSignal(dict)
    battery_state = pyqtSignal(dict)
    estop_state = pyqtSignal(bool)
    camera_state = pyqtSignal(str)
    calib_progress = pyqtSignal(dict)
    tag_ids = pyqtSignal(str, object)   # (camera, [tag ids in latest frame])
    log = pyqtSignal(str)

    def __init__(self, node_name='robot_ui', init_node=True):
        super().__init__()
        if init_node:
            # disable_signals: Qt owns SIGINT here, otherwise rospy's handler
            # and Qt's race on Ctrl-C and the window survives a "shutdown".
            rospy.init_node(node_name, anonymous=False, disable_signals=True)

        self._bridge = CvBridge()

        # Cleared first thing in shutdown(). rospy delivers callbacks on its own
        # threads and keeps doing so after Qt has begun tearing the window down,
        # at which point emitting a signal from a half-destroyed QObject raises
        # "wrapped C/C++ object of type RosBridge has been deleted" — once per
        # frame, from three cameras. Unregistering is not enough on its own:
        # messages already handed to a callback thread still arrive.
        self._alive = True

        # Every subscriber, so shutdown() can unregister all of them. Keeping
        # only the image ones (as an earlier version did) left the state and
        # battery subscriptions firing into the same destroyed object.
        self._subs = []

        # Last value emitted on each state signal, so replay() can re-send it.
        #
        # ⚠️ THIS IS WHAT MAKES LATCHED TOPICS WORK. Subscriptions are created
        # here in __init__, but a consumer (MainWindow) connects its slots
        # afterwards — so a latched topic's retained message is emitted into
        # nothing and is gone: latched means "delivered once on connect", not
        # "redelivered later". /task_state and /camera/state are both latched
        # one-shots, and both were silently missing from the window until it
        # happened to change. A periodic topic like /lifter/state hides the
        # same bug by arriving again 500 ms later.
        self._cache = {}

        # Latest arm state, plus an Event so wait_for_motion() can block on a
        # change instead of polling.
        self._arm_lock = threading.Lock()
        self._arm_latest = None
        self._arm_event = threading.Event()
        # Latest /mobile/state dict, for wait_for_mobile(). Kept in its own
        # slot rather than looked up in _cache: PyQt returns a NEW bound
        # signal object on every `self.mobile_state` access, so a dict keyed
        # by it cannot be read back.
        self._mobile_lock = threading.Lock()
        self._mobile_latest = None

        # ---------- publishers ----------
        self._pub_task_cmd = rospy.Publisher('/task_command', String,
                                             queue_size=10)
        self._pub_arm_move = rospy.Publisher('/arm/move_cart', String,
                                             queue_size=1)
        self._pub_arm_jog = rospy.Publisher('/arm/jog_cmd', String,
                                            queue_size=1)
        self._pub_arm_cancel = rospy.Publisher('/arm/cancel', Bool,
                                               queue_size=1)
        self._pub_cam_active = rospy.Publisher('/camera/set_active', Bool,
                                               queue_size=1)
        self._pub_lift_mm = rospy.Publisher('/lifter/height_cmd', Float32,
                                            queue_size=1)
        self._pub_mobile_move = rospy.Publisher('/mobile/move_cmd', String,
                                                queue_size=1)

        # ---------- subscribers ----------
        # queue_size=1 + a large buff_size on the image topics on purpose: for
        # a live view the newest frame is the only one that matters, and the
        # default 64 kB buffer fragments 720p frames badly enough to look like
        # packet loss.
        # One subscription per tag camera, raw OR overlay; the overlay is the
        # default because the operator is usually checking what the detector
        # sees. set_stream_source() swaps it live.
        self._stream_subs = {}
        self._stream_overlay = {}
        for name in STREAM_CAMERAS:
            self._subscribe_stream(name, overlay=True)
        self._sub('/basler/image_raw', Image, self._cb_image,
                  callback_args='basler', queue_size=1, buff_size=2 ** 24)

        self._sub('/arm/state', ArmState, self._cb_arm, queue_size=1)
        self._sub('/task_state', String, self._cb_json,
                  callback_args=self.task_state, queue_size=5)
        self._sub('/lifter/state', String, self._cb_json,
                  callback_args=self.lift_state, queue_size=1)
        self._sub('/mobile/state', String, self._cb_mobile, queue_size=1)
        self._sub('/camera/state', String, self._cb_camera_state, queue_size=1)
        self._sub('/bms/state', BatteryState, self._cb_battery, queue_size=1)
        self._sub('/safety/estop', Bool, self._cb_estop, queue_size=1)
        # Per-tag status lines from a running calibration session. Larger
        # queue than the periodic states: entries can complete in bursts and
        # every one matters in the log. Deliberately NOT routed through
        # _cb_json/_emit: the cache+replay machinery exists for latched
        # STATE topics, and replaying the last per-tag event to a late
        # consumer would look like a live session.
        self._sub('/map_calibrator/progress', String,
                  self._cb_calib_progress, queue_size=64)
        # Latest tag detections per camera, for scripts that need to ask
        # "what does the hand cam see right now" (e.g. the cross-tag
        # survey). Snapshot store only — no signal, poll via
        # latest_detections().
        self._det_lock = threading.Lock()
        self._det_latest = {}
        for cam in STREAM_CAMERAS:
            self._sub(f'/{cam}/tag_detections', AprilTagDetectionArray,
                      self._cb_detections, callback_args=cam, queue_size=1)

    def _sub(self, topic, msg_type, callback, **kwargs):
        sub = rospy.Subscriber(topic, msg_type, callback, **kwargs)
        self._subs.append(sub)
        return sub

    def _subscribe_stream(self, name, overlay):
        old = self._stream_subs.pop(name, None)
        if old is not None:
            try:
                old.unregister()
            except Exception:
                pass
            if old in self._subs:
                self._subs.remove(old)
        topic = OVERLAY_TOPICS[name] if overlay else STREAM_CAMERAS[name]
        # queue_size=1 + a large buff_size on purpose: for a live view the
        # newest frame is the only one that matters, and the default 64 kB
        # buffer fragments 720p frames badly enough to look like packet loss.
        self._stream_subs[name] = self._sub(
            topic, Image, self._cb_image, callback_args=name,
            queue_size=1, buff_size=2 ** 24)
        self._stream_overlay[name] = bool(overlay)
        return topic

    def set_stream_source(self, name, overlay):
        """Show one tag camera as the detector's annotated overlay (True) or
        the raw colour stream (False). (ok, message)."""
        if name not in STREAM_CAMERAS:
            return False, f'unknown camera {name}'
        if self._stream_overlay.get(name) == bool(overlay):
            return True, f'{name} already on {"overlay" if overlay else "raw"}'
        topic = self._subscribe_stream(name, overlay)
        return True, f'{name} <- {topic}'

    def stream_is_overlay(self, name):
        return bool(self._stream_overlay.get(name, False))

    def replay(self):
        """Re-emit the latest value of every cached state signal.

        Call this once after connecting slots. It is what recovers the retained
        message of a latched topic, and it also means a window populates the
        moment it is built instead of waiting for the next periodic update.

        Images are deliberately not cached or replayed: they are large, and a
        live camera sends another one within ~33 ms anyway.
        """
        for signal, value in list(self._cache.items()):
            try:
                signal.emit(value)
            except Exception:
                pass

    def _emit(self, signal, value):
        """Emit and remember, so replay() can re-send it to a later consumer."""
        self._cache[signal] = value
        signal.emit(value)

    # ==========================================================
    # INBOUND
    # ==========================================================
    @staticmethod
    def _decode_hint(msg):
        """Keep mono frames mono; normalise everything else to bgr8.

        Asking cv_bridge for bgr8 unconditionally would widen the wrist
        camera's mono8 back to three identical channels in this process —
        undoing the 3x saving the node just made, 40 MB per frame, on the GUI
        thread's data path. 'passthrough' is only safe because it is applied
        exclusively to mono8; for colour topics it would hand back whatever
        channel order the publisher chose and the reds and blues would swap.
        """
        return 'passthrough' if msg.encoding == 'mono8' else 'bgr8'

    def _cb_image(self, msg, name):
        if not self._alive:
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding=self._decode_hint(msg))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f'[UI] {name} image convert failed: {e}')
            return
        # .copy() because cv_bridge can hand back a view onto the message
        # buffer, which rospy is free to reuse once this callback returns.
        self.image_received.emit(name, bgr.copy())

    def _cb_arm(self, msg):
        if not self._alive:
            return
        state = {
            'state': msg.state,
            'busy': msg.busy,
            'tcp_pose': list(msg.tcp_pose),
            'pose_valid': msg.pose_valid,
            'joints': list(msg.joints),
            'motion_seq': int(msg.motion_seq),
            'result_message': msg.result_message,
            'result_success': msg.result_success,
        }
        with self._arm_lock:
            self._arm_latest = state
        self._arm_event.set()
        self._emit(self.arm_state, state)

    def _cb_json(self, msg, signal):
        if not self._alive:
            return
        try:
            self._emit(signal, json.loads(msg.data))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f'[UI] bad JSON state: {e}')

    def _cb_mobile(self, msg):
        if not self._alive:
            return
        try:
            state = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn_throttle(10.0, f'[UI] bad /mobile/state JSON: {e}')
            return
        with self._mobile_lock:
            self._mobile_latest = state
        self._emit(self.mobile_state, state)

    def _cb_camera_state(self, msg):
        if not self._alive:
            return
        self._emit(self.camera_state, msg.data)

    def _cb_calib_progress(self, msg):
        """Event stream, not state: emit without caching (see subscriber)."""
        if not self._alive:
            return
        try:
            self.calib_progress.emit(json.loads(msg.data))
        except Exception as e:
            rospy.logwarn_throttle(10.0, f'[UI] bad calib progress: {e}')

    def _cb_battery(self, msg):
        if not self._alive:
            return
        self._emit(self.battery_state, {
            'percentage': float(msg.percentage) * 100.0,
            'voltage': float(msg.voltage),
            'current': float(msg.current),
            'temperature': float(msg.temperature),
        })

    def _cb_estop(self, msg):
        if not self._alive:
            return
        self._emit(self.estop_state, bool(msg.data))

    # ==========================================================
    # ARM — manual teaching
    # ==========================================================
    def arm_seq(self):
        with self._arm_lock:
            return self._arm_latest['motion_seq'] if self._arm_latest else 0

    def arm_snapshot(self):
        with self._arm_lock:
            return dict(self._arm_latest) if self._arm_latest else None

    def wait_for_motion(self, since_seq, timeout=60.0):
        """Block until arm motion_seq passes `since_seq`. Returns (ok, message).

        Read `since_seq` with arm_seq() BEFORE publishing the command. Waiting
        for "seq != what I last saw" instead would be satisfied by a state
        message already in flight when the command went out.
        """
        deadline = rospy.get_time() + timeout
        while not rospy.is_shutdown():
            with self._arm_lock:
                latest = self._arm_latest
            if latest and latest['motion_seq'] > since_seq:
                return latest['result_success'], latest['result_message']
            if rospy.get_time() > deadline:
                return False, f'timed out after {timeout:.0f}s waiting for arm'
            self._arm_event.clear()
            # Wake on either a new state or the deadline, whichever is first.
            self._arm_event.wait(timeout=0.1)
        return False, 'shutting down'

    def arm_move_cart(self, pose, vel=30.0, acc=50.0, timeout=60.0):
        """Absolute Cartesian move. Blocks until arm_node reports completion."""
        if len(pose) != 6:
            return False, 'pose needs 6 values [x y z rx ry rz]'
        seq = self.arm_seq()
        self._pub_arm_move.publish(String(json.dumps(
            {'pose': [float(v) for v in pose], 'vel': vel, 'acc': acc})))
        return self.wait_for_motion(seq, timeout)

    def arm_jog(self, axis, delta, vel=30.0, acc=50.0, timeout=30.0):
        """Incremental move on one Cartesian axis. Blocks until complete."""
        if axis not in ARM_AXES:
            return False, f'unknown axis {axis}'
        seq = self.arm_seq()
        self._pub_arm_jog.publish(String(json.dumps(
            {'axis': axis, 'delta': float(delta), 'vel': vel, 'acc': acc})))
        return self.wait_for_motion(seq, timeout)

    def arm_home(self, timeout=60.0):
        return self._call_trigger('/arm/move_home', timeout)

    def arm_cancel(self):
        """Abort whatever the arm is doing. Deliberately not a service call.

        A stop must not queue behind the motion it is aborting, and must not
        block if arm_node is wedged — a rospy service call has no in-flight
        timeout and would hang the GUI thread forever.
        """
        self._pub_arm_cancel.publish(Bool(True))
        self.log.emit('[UI] arm cancel published')

    # ==========================================================
    # CAMERA + INFERENCE
    # ==========================================================
    def set_camera_active(self, active):
        """Ask basler_camera_node to hold the device open (or release it).

        This is how the UI gets a Basler preview without becoming a second
        owner: the node still opens, closes and lamps the camera, it is merely
        told not to close between the frames the preview is grabbing.
        """
        self._pub_cam_active.publish(Bool(bool(active)))

    def capture(self, num_samples=1, delay_between_s=-1.0, use_vision_led=True,
                timeout=30.0):
        """Grab frames through the owner node. Returns (ok, message, [bgr]).

        BLOCKS — call from a worker thread. The reply carries the frames, so a
        captured frame can never be paired with a stale preview frame.
        """
        try:
            rospy.wait_for_service('/camera/capture', timeout=5.0)
            proxy = rospy.ServiceProxy('/camera/capture', CaptureImages)
            resp = proxy(int(num_samples), float(delay_between_s),
                         bool(use_vision_led))
        except Exception as e:
            return False, f'capture failed: {e}', []

        frames = []
        for img in resp.images:
            try:
                frames.append(self._bridge.imgmsg_to_cv2(
                    img, desired_encoding=self._decode_hint(img)).copy())
            except Exception as e:
                rospy.logwarn(f'[UI] capture frame convert failed: {e}')
        return resp.success, resp.message, frames

    def predict_ra(self, bgr, tag='', timeout=30.0):
        """Score one frame through inference_node. Returns a result dict.

        BLOCKS — call from a worker thread. The UI must not load a model of its
        own; inference_node holds the only resident copy.
        """
        try:
            rospy.wait_for_service('/inference/predict', timeout=5.0)
            proxy = rospy.ServiceProxy('/inference/predict', PredictRa)
            # Ship whatever we hold. A mono frame stays 1 channel on the wire;
            # inference_node asks cv_bridge for bgr8 and RaPredictor's model
            # input is bit-identical either way (verified, max |diff| = 0.0).
            encoding = 'mono8' if bgr.ndim == 2 else 'bgr8'
            msg = self._bridge.cv2_to_imgmsg(bgr, encoding=encoding)
            resp = proxy(msg, str(tag))
        except Exception as e:
            return {'success': False, 'message': f'inference failed: {e}',
                    'ra': float('nan'), 'models': [], 'elapsed_s': 0.0,
                    'tag': tag}
        return {
            'success': resp.success,
            'message': resp.message,
            'ra': float(resp.ra),
            'models': list(zip(resp.model_names, [float(v) for v in resp.ra_values])),
            'elapsed_s': float(resp.elapsed_s),
            'tag': resp.tag,
        }

    def set_stream_camera_enabled(self, name, enabled):
        """Turn one tag camera's detector and vendor stream on or off."""
        return self._call_setbool(f'/robot_camera/{name}/set_enabled', enabled)

    # ==========================================================
    # LIFT / BASE / TASKS
    # ==========================================================
    def lift_goto_mm(self, mm):
        """Fire-and-forget. Watch /lifter/state for arrival.

        ⚠️ This is /lifter/*, the guarded interface. Never publish to /lift/*,
        which is the raw navifra driver with no soft limit and no origin check
        — the two topic names differ by one character.
        """
        self._pub_lift_mm.publish(Float32(float(mm)))

    def lift_home(self, timeout=140.0):
        return self._call_trigger('/lifter/home', timeout)

    def lift_stop(self):
        return self._call_trigger('/lifter/stop', 5.0)

    def mobile_stop(self):
        """EMERGENCY latch on mobile_node — needs mobile_clear_stop() after."""
        return self._call_trigger('/mobile/stop', 5.0)

    def mobile_cancel(self):
        """Preempt stop: zero velocity, no emergency latch."""
        return self._call_trigger('/mobile/cancel', 5.0)

    def mobile_clear_stop(self):
        return self._call_trigger('/mobile/clear_stop', 5.0)

    # ---- manual relative moves (odometry-closed, no tag) ----
    # Same completion model as move_to_tag in MobileClient: mobile_node
    # stamps every finished move with an incrementing `seq`; read it BEFORE
    # publishing and wait for it to advance. /mobile/state is a periodic
    # 5 Hz topic; _cb_mobile keeps the latest dict for the poll below.

    def mobile_snapshot(self):
        with self._mobile_lock:
            return dict(self._mobile_latest) if self._mobile_latest else None

    def mobile_seq(self):
        st = self.mobile_snapshot()
        return int(st.get('seq', 0)) if st else 0

    def wait_for_mobile(self, since_seq, timeout=120.0, ack_timeout=5.0):
        """Block until mobile_node's seq passes `since_seq`. (ok, message)."""
        # Phase 1: did the node pick it up? Either it went busy, or it
        # already finished (seq advanced).
        deadline = rospy.get_time() + ack_timeout
        while not rospy.is_shutdown() and self._alive:
            st = self.mobile_snapshot()
            if st and (st.get('busy') or st.get('seq', 0) > since_seq):
                break
            if rospy.get_time() > deadline:
                return False, (f'mobile_node never acknowledged within '
                               f'{ack_timeout:.0f}s — is it running?')
            rospy.sleep(0.05)
        # Phase 2: completion.
        deadline = rospy.get_time() + timeout
        while not rospy.is_shutdown() and self._alive:
            st = self.mobile_snapshot()
            if st and st.get('seq', 0) > since_seq:
                res = st.get('result') or {}
                return bool(res.get('ok')), str(res.get('message', ''))
            if rospy.get_time() > deadline:
                return False, f'timed out after {timeout:.0f}s waiting for base'
            rospy.sleep(0.05)
        return False, 'shutting down'

    def mobile_drive(self, distance_m, speed=None, timeout=120.0):
        """Drive the base `distance_m` along body X (negative = reverse).
        Blocks until mobile_node reports completion. (ok, message)."""
        if self.mobile_snapshot() is None:
            return False, 'no /mobile/state — is mobile_node running?'
        seq = self.mobile_seq()
        req = {'type': 'move', 'distance_m': float(distance_m)}
        if speed:
            req['speed'] = float(speed)
        self._pub_mobile_move.publish(String(json.dumps(req)))
        self.log.emit(f'[UI] /mobile/move_cmd <- {json.dumps(req)}')
        return self.wait_for_mobile(seq, timeout)

    def mobile_pivot(self, angle_deg, speed=None, timeout=120.0):
        """Pivot the base in place by `angle_deg` (positive = CCW / left).
        Blocks until mobile_node reports completion. (ok, message)."""
        if self.mobile_snapshot() is None:
            return False, 'no /mobile/state — is mobile_node running?'
        seq = self.mobile_seq()
        req = {'type': 'pivot', 'angle_deg': float(angle_deg)}
        if speed:
            req['speed'] = float(speed)
        self._pub_mobile_move.publish(String(json.dumps(req)))
        self.log.emit(f'[UI] /mobile/move_cmd <- {json.dumps(req)}')
        return self.wait_for_mobile(seq, timeout)

    def send_task_command(self, command):
        """Publish a line to /task_command (TASK / GOTO / STOP / STATE)."""
        text = str(command).strip()
        if not text:
            return False
        self._pub_task_cmd.publish(String(text))
        self.log.emit(f'[UI] /task_command <- {text}')
        return True

    # ==========================================================
    # TAG-MAP CALIBRATION (path_tag_locator nodes)
    # ==========================================================
    # These nodes are NOT part of mobile_manipulator.launch — run
    # `roslaunch path_tag_locator path_tag_locator.launch` alongside the
    # stack first, or every call below fails with a wait_for_service
    # timeout. Progress arrives on the calib_progress signal.
    # ⚠️ A calibration session drives the base through /mobile/goto_tag —
    # do not send TASK/GOTO while one is running.

    def _cb_detections(self, msg, cam):
        if not self._alive:
            return
        with self._det_lock:
            self._det_latest[cam] = msg
        # Small list at frame rate — cheap enough to hand to the GUI thread
        # directly; the window shows it under the camera pane.
        self.tag_ids.emit(cam, [int(d.id) for d in msg.detections])

    def latest_detections(self, cam='hand_cam', max_age_s=1.0):
        """Fresh detections of one camera as a list of dicts (id,
        pose_x/y/z, center, yaw); [] if none/stale."""
        with self._det_lock:
            msg = self._det_latest.get(cam)
        if msg is None:
            return []
        if (rospy.Time.now() - msg.header.stamp).to_sec() > max_age_s:
            return []
        return [{'id': d.id, 'pose_x': d.pose_x, 'pose_y': d.pose_y,
                 'pose_z': d.pose_z, 'center_x': d.center_x,
                 'center_y': d.center_y, 'yaw': d.yaw}
                for d in msg.detections]

    def calib_nodes_online(self):
        """Fast master-registry check (no service call): are the
        path_tag_locator nodes up? They are NOT part of the main launch —
        `roslaunch path_tag_locator path_tag_locator.launch` runs them.
        Cheap enough for a periodic UI poll."""
        try:
            import rosgraph
            master = rosgraph.Master('/robot_ui')
            master.lookupService('/map_calibrator/run_calibration')
            return True
        except Exception:
            return False

    _CALIB_HINT = (' — is `roslaunch path_tag_locator '
                   'path_tag_locator.launch` running? (not part of the '
                   'main launch)')

    def locate_path_tag(self, tag_b_id=-1, auto_align=False,
                        initial_tcp=None):
        """One LocatePathTag call. BLOCKS (unbounded, like every ROS1
        service proxy here) — call from a worker thread.

        Returns a dict: success/message plus position_m + rpy_deg on
        success. tag_b_id -1 uses locator.yaml's default tag.
        """
        try:
            from path_tag_locator.srv import LocatePathTag
        except ImportError as e:
            return {'success': False,
                    'message': f'path_tag_locator not built: {e}'}
        try:
            rospy.wait_for_service('/path_tag_locator/locate_path_tag',
                                   timeout=5.0)
            proxy = rospy.ServiceProxy('/path_tag_locator/locate_path_tag',
                                       LocatePathTag)
            req = LocatePathTag._request_class()
            req.tag_b_id = int(tag_b_id)
            req.auto_align = bool(auto_align)
            if initial_tcp is not None:
                req.align_initial_tcp_mm_deg = [float(v)
                                                for v in initial_tcp]
            resp = proxy(req)
        except Exception as e:
            return {'success': False,
                    'message': f'locate failed: {e}{self._CALIB_HINT}'}
        return {
            'success': bool(resp.success),
            'message': resp.message,
            'position_m': list(resp.position_m),
            'rpy_deg': list(resp.rpy_deg),
            'align_iterations': int(resp.align_iterations_used),
        }

    def run_map_calibration(self, dry_run=False, plan_path='',
                            ref_tags_path=''):
        """Full RunMapCalibration session.

        Empty paths use map_calibrator.yaml's defaults (the 정반 1
        session). Sessions run PER PLATE — both plates carry cross tags
        with the same ids, so plan_path and ref_tags_path must be
        swapped TOGETHER for 정반 2.

        BLOCKS for the whole session (minutes) — call from a worker
        thread. Per-tag results stream on calib_progress meanwhile.
        Returns (ok, message, report dict).
        """
        try:
            from path_tag_locator.srv import RunMapCalibration
        except ImportError as e:
            return False, f'path_tag_locator not built: {e}', {}
        try:
            rospy.wait_for_service('/map_calibrator/run_calibration',
                                   timeout=5.0)
            proxy = rospy.ServiceProxy('/map_calibrator/run_calibration',
                                       RunMapCalibration)
            req = RunMapCalibration._request_class()
            req.dry_run = bool(dry_run)
            req.plan_path = str(plan_path)
            req.ref_tags_path = str(ref_tags_path)
            resp = proxy(req)
        except Exception as e:
            return (False,
                    f'run_calibration failed: {e}{self._CALIB_HINT}', {})
        return bool(resp.success), resp.message, {
            'num_succeeded': int(resp.num_succeeded),
            'num_failed': int(resp.num_failed),
            'output_yaml_path': resp.output_yaml_path,
        }

    def cancel_map_calibration(self):
        """Cooperative session abort: the calibrator stops after the
        current entry, keeping the partial output. Not an e-stop."""
        return self._call_trigger('/map_calibrator/cancel_calibration',
                                  10.0)

    def handeye_capture(self):
        """One hand-eye sample (needs use_handeye_calib:=true launch)."""
        return self._call_trigger('/handeye_calib/capture', 30.0)

    def handeye_compute(self):
        return self._call_trigger('/handeye_calib/compute', 120.0)

    def handeye_status(self):
        return self._call_trigger('/handeye_calib/status', 10.0)

    # ==========================================================
    # SERVICE HELPERS
    # ==========================================================
    def _call_trigger(self, name, timeout):
        try:
            rospy.wait_for_service(name, timeout=min(5.0, timeout))
            resp = rospy.ServiceProxy(name, Trigger)()
            return bool(resp.success), resp.message
        except Exception as e:
            return False, f'{name} failed: {e}'

    def _call_setbool(self, name, value):
        try:
            rospy.wait_for_service(name, timeout=5.0)
            resp = rospy.ServiceProxy(name, SetBool)(bool(value))
            return bool(resp.success), resp.message
        except Exception as e:
            return False, f'{name} failed: {e}'

    # ==========================================================
    # SHUTDOWN
    # ==========================================================
    def shutdown(self):
        """Release what the UI asked other nodes to hold, then go quiet.

        Order matters. Publish the camera release FIRST, while publishers still
        work: if the window closes with preview running, basler_camera_node
        would otherwise keep the sensor open and warm with nobody looking.

        Then stop delivering. Clearing _alive before unregistering closes the
        window where a message already handed to a rospy callback thread lands
        on a QObject Qt has started destroying — which raises "wrapped C/C++
        object of type RosBridge has been deleted", once per frame, from every
        camera at once.

        Motion is deliberately NOT cancelled here. Closing a window is not a
        stop request, and silently aborting a running task would surprise more
        than it protects; the UI has an explicit STOP ALL for that.
        """
        try:
            self.set_camera_active(False)
        except Exception:
            pass

        self._alive = False
        for sub in self._subs:
            try:
                sub.unregister()
            except Exception:
                pass
        self._subs = []
        # Release anything blocked in wait_for_motion so a worker thread does
        # not sit out its full timeout while the process is trying to exit.
        self._arm_event.set()
