"""
arm_interface.py
================
Arm access through the main stack's ``arm_node`` — replaces the old
``tcp_pose.FairinoTCPClient`` which opened a second Fairino RPC connection.

The main stack owns the arm (one owner node per device); this module is a
duck-type replacement exposing the same three methods the align/chain code
consumes:

  - ``get_tcp_pose()``       -> [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
                                 read from ``/arm/state`` (ArmState.tcp_pose,
                                 TOOL_ID=1 vision_tip — same TCP the old SDK
                                 read returned when the pendant tool matched)
  - ``move_j_to_pose(pose)`` -> JSON on ``/arm/move_cart``; completion is
                                 detected the same way the other device
                                 proxies do it: ``motion_seq`` advances in
                                 ``/arm/state`` and ``result_success`` says
                                 whether the motion landed.
  - ``enable()``             -> no-op (arm_node enables the arm at startup)

⚠️ Single-commander assumption, same as MobileClient: two nodes publishing
``/arm/move_cart`` concurrently would each see the other's ``motion_seq``
advance. arm_node also refuses move_cart while a scan is running, so a
locate/calibration call issued mid-scan fails fast here with a clear error
instead of queueing behind the scan.
"""
import json
import threading

import rospy
from std_msgs.msg import String

from robot_msgs.msg import ArmState


class ArmInterface:
    """Proxy for arm_node with the FairinoTCPClient call surface."""

    def __init__(self,
                 state_topic='/arm/state',
                 move_cart_topic='/arm/move_cart',
                 default_vel=20.0,
                 default_acc=20.0,
                 default_ovl=100.0,
                 motion_timeout_s=60.0,
                 state_timeout_s=5.0,
                 connect_timeout_s=15.0):
        self.default_vel = float(default_vel)
        self.default_acc = float(default_acc)
        self.default_ovl = float(default_ovl)  # accepted for signature
        self.motion_timeout_s = float(motion_timeout_s)
        self.state_timeout_s = float(state_timeout_s)
        self._connect_timeout_s = float(connect_timeout_s)

        self._lock = threading.Lock()
        self._state = None

        self._pub_move = rospy.Publisher(move_cart_topic, String,
                                         queue_size=1)
        rospy.Subscriber(state_topic, ArmState, self._cb_state,
                         queue_size=1)

    # ------------------------------------------------------------------
    def _cb_state(self, msg):
        with self._lock:
            self._state = msg

    @property
    def state(self):
        with self._lock:
            return self._state

    def wait_for_node(self, timeout_s=None):
        """Block until the first /arm/state arrives. Returns (ok, reason)."""
        timeout_s = (self._connect_timeout_s if timeout_s is None
                     else float(timeout_s))
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.state is not None:
                return True, 'ok'
            if rospy.Time.now() >= deadline:
                return False, (f'no /arm/state within {timeout_s:.1f}s — '
                               f'is arm_node running?')
            rate.sleep()
        return False, 'shutdown'

    # ------------------------------------------------------------------
    def enable(self):
        """No-op: arm_node enables the arm; kept for call-site parity."""

    def _fresh(self, st, max_age_s=1.0):
        """A state message counts only if it is RECENT. Without this, a
        dead arm_node leaves the last state cached forever and a stale
        TCP pose silently enters the precision chain."""
        if st is None:
            return False
        age = (rospy.Time.now() - st.header.stamp).to_sec()
        return 0.0 <= age < max_age_s or age < 0.0  # tolerate clock skew back

    def _wait_state(self, what):
        deadline = rospy.Time.now() + rospy.Duration(self.state_timeout_s)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            st = self.state
            if self._fresh(st) and st.pose_valid:
                return st
            if rospy.Time.now() >= deadline:
                raise RuntimeError(
                    f'{what}: no fresh pose_valid /arm/state within '
                    f'{self.state_timeout_s:.1f}s — is arm_node running '
                    'and the arm reachable?')
            rate.sleep()
        raise RuntimeError(f'{what}: rospy shutdown')

    def get_tcp_pose(self):
        """Return [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg] from a FRESH
        /arm/state (the RPC read inside arm_node can fail transiently and
        arm_node itself can die — both are surfaced, never papered over)."""
        return [float(v) for v in self._wait_state('get_tcp_pose').tcp_pose]

    def get_joints(self):
        """Return current joint angles (deg) as a list."""
        return [float(v) for v in self._wait_state('get_joints').joints]

    # ------------------------------------------------------------------
    def move_j_to_pose(self, target_pose_mm_deg,
                       vel=None, acc=None, ovl=None,
                       settle_s: float = 0.2):
        """Command an absolute Cartesian TCP move through arm_node and
        block until it completes.

        Same semantics as the old SDK path (IK + MoveJ to a mm/deg ZYX
        descriptor pose); ``ovl`` is accepted for signature compatibility
        and ignored, exactly as before.
        """
        ok, reason = self.wait_for_node()
        if not ok:
            raise RuntimeError(f'move_j_to_pose: {reason}')

        # The publisher must actually be CONNECTED before the one-shot
        # command goes out: rospy drops messages published before the TCP
        # link to arm_node's subscriber is up, and wait_for_node() only
        # proves the /arm/state direction. Without this, the first move
        # after node start vanishes and burns the full motion timeout.
        conn_deadline = rospy.Time.now() + rospy.Duration(5.0)
        while self._pub_move.get_num_connections() == 0:
            if rospy.Time.now() >= conn_deadline:
                raise RuntimeError(
                    'move_j_to_pose: no subscriber on the move_cart '
                    'topic — is arm_node running?')
            rospy.sleep(0.05)

        seq0 = self.state.motion_seq
        payload = {
            'pose': [float(v) for v in target_pose_mm_deg],
            'vel': float(self.default_vel if vel is None else vel),
            'acc': float(self.default_acc if acc is None else acc),
        }
        rospy.loginfo('[ArmInterface] move_cart -> %s',
                      ['%.2f' % v for v in payload['pose']])
        self._pub_move.publish(String(data=json.dumps(payload)))

        deadline = rospy.Time.now() + rospy.Duration(self.motion_timeout_s)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            st = self.state
            if st is not None and st.motion_seq < seq0:
                # motion_seq never decreases within one arm_node life.
                raise RuntimeError(
                    'move_cart: arm_node restarted mid-move '
                    f'(motion_seq {st.motion_seq} < {seq0}) — arm state '
                    'unknown, re-issue the command deliberately')
            if st is not None and st.motion_seq > seq0:
                # arm_node bumps motion_seq for EVERY completed motion
                # (scans, jogs, refused commands). Attribute by result
                # message: a foreign bump (e.g. a scan finishing right
                # after we published) advances the baseline and keeps
                # waiting for OUR result instead of mis-reporting the
                # scan's success as our completion.
                if 'move_cart' not in st.result_message:
                    seq0 = st.motion_seq
                    continue
                if not st.result_success:
                    raise RuntimeError(
                        f'move_cart failed: {st.result_message} '
                        f'(target={payload["pose"]})')
                break
            if rospy.Time.now() >= deadline:
                raise RuntimeError(
                    f'move_cart: no completion within '
                    f'{self.motion_timeout_s:.1f}s (arm busy with a scan, '
                    f'or arm_node down?); target={payload["pose"]}')
            rate.sleep()
        if settle_s > 0:
            rospy.sleep(settle_s)
