#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_node — kinematic simulator of the hardware-owner boundary.

Everything the REAL nodes consume is synthesized from one ground-truth
state; everything they command moves that state. The nodes under test
(mobile_node/mobile_controller, path_tag_locator's locator + calibrator)
run UNMODIFIED — this node impersonates the topics/services otherwise
provided by the navifra driver, robot_camera_node, arm_node and
lifter_node. Do not run it on the robot PC while the real stack is up:
it publishes the same topic names on purpose.

Ground truth
------------
- base (x, y, theta) in the map/world frame; mb origin ON THE FLOOR
  (world z = floor_z, default -0.080; plate top = 0).
- arm TCP pose [mm, deg ZYX] in the arm-base frame (vision_tip, same
  convention as arm_node); moves toward move_cart/jog targets at a
  constant rate, then bumps motion_seq with "move_cart ok" — the exact
  attribution string ArmInterface filters on.
- lift height (mm); /lifter/home descends to 0 and re-zeros.

Cameras
-------
A pinhole camera per T_mb2fc (front) and base∘TCP∘inv(hand-eye) (hand).
Floor tags come from apriltag_nav map.yaml (z = floor_z, yaw from the
zone convention A/DOCK=0, B/D=+90, C/E=-90); cross tags from the two
reference_tags yamls (z = 0). All tags face-up (tag +z pointing DOWN,
AprilTag body convention). Euler encoding matches robot_camera_node:
``as_euler('zyx', degrees=True)[::-1]`` with the as_dcm scipy-1.3
fallback. pose_t is scaled by detector_size/actual_size exactly like
dt_apriltags would when the configured size differs from the printed
one. Corners are projected so that the corner0->corner1 edge lies along
the tag's +x axis, which reproduces tag_edge_angle_deg() == 0 for a
robot aligned with its corridor.
"""
import json
import math
import threading

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from robot_msgs.msg import AprilTagDetection, AprilTagDetectionArray, ArmState

from apriltag_nav.paths import CONFIG_PATH, MAP_PATH
from path_tag_locator.hand_eye import load_T_hc2ee
from path_tag_locator.geometry import (
    invert_T,
    pose_fr5_to_matrix_m,
)

try:
    from scipy.spatial.transform import Rotation as _Rot
except ImportError:
    _Rot = None

FLOOR_Z = -0.080
ZONE_YAW_DEG = {"A": 0.0, "DOCK": 0.0, "B": 90.0, "D": 90.0,
                "C": -90.0, "E": -90.0}


def rz(rad):
    c, s = math.cos(rad), math.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_to_euler_zyx_deg(Rm):
    """robot_camera_node's exact encoding: as_euler('zyx')[::-1]."""
    rot = (_Rot.from_matrix(Rm) if hasattr(_Rot, 'from_matrix')
           else _Rot.from_dcm(Rm))
    return rot.as_euler('zyx', degrees=True)[::-1]


class Tag:
    def __init__(self, tag_id, x, y, z, yaw_deg, size_m, kind):
        self.id = int(tag_id)
        self.size_m = float(size_m)
        self.kind = kind                       # 'floor' | 'cross'
        # Face-up: tag +z points DOWN (into the tag, away from the
        # printed face) -> R = Rz(yaw) @ diag(1,-1,-1).
        self.T_w = np.eye(4)
        self.T_w[:3, :3] = rz(math.radians(yaw_deg)) @ np.diag(
            [1.0, -1.0, -1.0])
        self.T_w[:3, 3] = [x, y, z]
        # Corners in the tag frame, corner0->corner1 along tag +y.
        # Hardware-verified convention (mobile_controller work log): an
        # ALIGNED robot images that edge at raw +90 deg, which
        # tag_edge_angle_deg's -90 turns into 0. Tag +y = world -y for a
        # face-up yaw-0 tag = the robot's right = image +v. The first
        # corner order tried (+x) read 0 raw when aligned and the 9/2
        # heading-correction term slewed the robot ~6 deg off toward the
        # wrong zero -- a faithful reminder that the corner order IS the
        # convention.
        h = self.size_m / 2.0
        self.corners_t = np.array([[h, -h, 0, 1], [h, h, 0, 1],
                                   [-h, h, 0, 1], [-h, -h, 0, 1]],
                                  dtype=float).T


class SimNode:
    def __init__(self):
        rospy.init_node('robot_sim')
        p = rospy.get_param

        # ---------- ground truth ----------
        self.lock = threading.Lock()
        self.x = float(p('~start_x', -1.9123))
        self.y = float(p('~start_y', 3.02))
        self.theta = math.radians(float(p('~start_theta_deg', 0.0)))
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        # Actual velocities lag the commands (first-order, tau chosen so
        # the post-stop roll integral matches the ~0.55 s the real base
        # showed on 2026-09-02 — this is what robot.yaml stop_latency_s
        # compensates, so the sim must reproduce it or the stop line
        # lands systematically early).
        self.v_act = 0.0
        self.w_act = 0.0
        self.v_tau = float(p('~vel_tau_s', 0.55))
        self.w_tau = float(p('~ang_tau_s', 0.08))
        self.last_cmd_t = rospy.Time(0)

        self.tcp = [float(v) for v in p('~start_tcp_mm_deg',
                                        [-500.0, 200.0, 400.0,
                                         178.7, 0.3, 0.0])]
        self.tcp_target = None
        self.arm_busy = False
        self.motion_seq = 0
        self.result_message = ''
        self.result_success = True
        self.arm_lin_mm_s = float(p('~arm_lin_mm_s', 400.0))
        # crude reachability: reject targets beyond this TCP radius,
        # mimicking GetInverseKin failure on over-reach view poses
        self.arm_reach_mm = float(p('~arm_reach_mm', 1700.0))
        self.arm_ang_deg_s = float(p('~arm_ang_deg_s', 90.0))
        self.arm_home = [float(v) for v in p('~arm_home_tcp_mm_deg',
                                             [-500.0, 200.0, 400.0,
                                              178.7, 0.3, 0.0])]

        self.lift_mm = float(p('~start_lift_mm', 0.0))
        self.lift_target = None
        self.lift_mm_s = float(p('~lift_mm_s', 40.0))

        # detection noise (std devs; 0 = perfect)
        self.noise_t_m = float(p('~noise_t_m', 0.0))
        self.noise_px = float(p('~noise_px', 0.0))
        self.odom_noise = float(p('~odom_noise', 0.0))

        # ---------- static geometry ----------
        cfg = yaml.safe_load(open(CONFIG_PATH))
        topics = cfg['topics']
        ex_path = p('~extrinsics_yaml',
                    rospy.get_param('~ex', '') or None)
        if not ex_path:
            import rospkg
            ex_path = (rospkg.RosPack().get_path('path_tag_locator')
                       + '/config/extrinsics.yaml')
        ex = yaml.safe_load(open(ex_path))
        self.T_mb2fc = np.asarray(ex['T_mb2fc_row_major'],
                                  dtype=float).reshape(4, 4)
        self.T_ab2mb = np.asarray(ex['T_ab2mb_row_major'],
                                  dtype=float).reshape(4, 4)
        import rospkg
        ptl = rospkg.RosPack().get_path('path_tag_locator')
        self.T_hc2ee = load_T_hc2ee(
            p('~hand_eye_npz', ptl + '/config/hand_eye/T_hc2ee.npz'))

        # cameras: intrinsics + configured detector tag size
        cam_cfg = cfg.get('robot_camera', {})
        size_cfg = cam_cfg.get('tag_size') or {}
        base_size = float(cfg['robot'].get('tag_size', 0.09))
        self.det_size = {
            'front_cam': float(size_cfg.get('front_cam') or base_size),
            'hand_cam': float(size_cfg.get('hand_cam') or base_size),
        }
        self.W = int(p('~image_width', 1280))
        self.H = int(p('~image_height', 720))
        self.K = [float(p('~fx', 910.0)), float(p('~fy', 910.0)),
                  float(p('~cx', 642.7)), float(p('~cy', 361.4))]

        # tags
        self.tags = []
        m = yaml.safe_load(open(MAP_PATH))['tags']
        floor_size = float(cfg['robot'].get('tag_size', 0.09))
        for tid, info in m.items():
            yaw = ZONE_YAW_DEG.get(info.get('zone', 'A'), 0.0)
            self.tags.append(Tag(tid, info['x'], info['y'], FLOOR_Z,
                                 yaw, floor_size, 'floor'))
        for fname in ('reference_tags.yaml', 'reference_tags_plate2.yaml'):
            try:
                refs = yaml.safe_load(open(ptl + '/config/' + fname))
                for r in refs['reference_tags']:
                    x, y, z = r['position_m']
                    yaw = float(r['rpy_deg'][2])
                    self.tags.append(Tag(r['id'], x, y, z, yaw,
                                         float(r.get('size_m', 0.09)),
                                         'cross'))
            except Exception as e:
                rospy.logwarn('[sim] cannot load %s: %s', fname, e)
        # Cross tags share ids across plates; keep both — visibility
        # naturally separates them (a camera only ever sees one).

        # ---------- ROS I/O ----------
        self.pub_odom = rospy.Publisher(topics.get('odom', '/odom'),
                                        Odometry, queue_size=1)
        self.pub_fc_det = rospy.Publisher(
            topics.get('front_cam_detections', '/front_cam/tag_detections'),
            AprilTagDetectionArray, queue_size=1)
        self.pub_hc_det = rospy.Publisher(
            topics.get('hand_cam_detections', '/hand_cam/tag_detections'),
            AprilTagDetectionArray, queue_size=1)
        self.pub_fc_info = rospy.Publisher(
            topics.get('front_cam_info', '/front_cam/color/camera_info'),
            CameraInfo, queue_size=1, latch=True)
        self.pub_arm = rospy.Publisher('/arm/state', ArmState, queue_size=1)
        self.pub_lift_state = rospy.Publisher('/lifter/state', String,
                                              queue_size=1, latch=True)
        self.pub_lift_h = rospy.Publisher('/lifter/height', Float32,
                                          queue_size=1, latch=True)
        self.pub_estop = rospy.Publisher('/safety/estop', Bool,
                                         queue_size=1, latch=True)
        self.pub_truth = rospy.Publisher('~ground_truth', String,
                                         queue_size=1)
        self.pub_markers = rospy.Publisher('~markers', MarkerArray,
                                           queue_size=1, latch=True)

        rospy.Subscriber(topics.get('cmd_vel', '/cmd_vel'), Twist,
                         self._cb_cmd, queue_size=1)
        rospy.Subscriber('/arm/move_cart', String, self._cb_move_cart,
                         queue_size=4)
        rospy.Subscriber('/arm/jog_cmd', String, self._cb_jog,
                         queue_size=4)
        rospy.Subscriber('/lifter/height_cmd', Float32,
                         self._cb_lift_height, queue_size=1)
        rospy.Subscriber('/lifter/command', String, self._cb_lift_cmd,
                         queue_size=1)
        rospy.Service('/arm/move_home', Trigger, self._srv_arm_home)
        rospy.Service('/lifter/home', Trigger, self._srv_lift_home)
        rospy.Service('/lifter/stop', Trigger, self._srv_lift_stop)
        rospy.Service('/lifter/reset', Trigger,
                      lambda _r: TriggerResponse(success=True, message='ok'))

        self.pub_estop.publish(Bool(False))
        self._publish_camera_info()
        self._publish_markers()

        self.dt = 0.02
        rospy.Timer(rospy.Duration(self.dt), self._physics)
        rospy.Timer(rospy.Duration(1.0 / 15.0), self._cameras)
        rospy.Timer(rospy.Duration(0.05), self._arm_state)
        rospy.Timer(rospy.Duration(0.2), self._lift_state)
        rospy.loginfo('[sim] up: %d tags, base (%.3f, %.3f, %.1f deg)',
                      len(self.tags), self.x, self.y,
                      math.degrees(self.theta))

    # ================= commands =================
    def _cb_cmd(self, msg):
        with self.lock:
            self.v_cmd = float(msg.linear.x)
            self.w_cmd = float(msg.angular.z)
            self.last_cmd_t = rospy.Time.now()

    def _cb_move_cart(self, msg):
        try:
            req = json.loads(msg.data)
            pose = [float(v) for v in req['pose']]
            assert len(pose) == 6
        except Exception as e:
            self._bump(False, f'bad /arm/move_cart JSON: {e}')
            return
        # decide under the lock, bump AFTER releasing it — _bump takes
        # the same non-reentrant lock (calling it inside deadlocked the
        # whole node the first time a rejection path actually fired)
        reject = None
        with self.lock:
            if self.arm_busy:
                reject = 'refused: arm busy'
            else:
                r = math.sqrt(pose[0]**2 + pose[1]**2 + pose[2]**2)
                if r > self.arm_reach_mm:
                    reject = ('move_cart failed: IK error 112 (target '
                              '%.0f mm > reach %.0f mm)'
                              % (r, self.arm_reach_mm))
                else:
                    self.tcp_target = pose
                    self.arm_busy = True
        if reject:
            self._bump(False, reject)

    def _cb_jog(self, msg):
        axes = ('x', 'y', 'z', 'rx', 'ry', 'rz')
        try:
            req = json.loads(msg.data)
            i = axes.index(req['axis'])
            delta = float(req['delta'])
        except Exception as e:
            self._bump(False, f'bad /arm/jog_cmd JSON: {e}')
            return
        reject = None
        with self.lock:
            if self.arm_busy:
                reject = 'refused: arm busy'
            else:
                tgt = list(self.tcp)
                tgt[i] += delta
                self.tcp_target = tgt
                self.arm_busy = True
        if reject:
            self._bump(False, reject)

    def _srv_arm_home(self, _req):
        with self.lock:
            if self.arm_busy:
                return TriggerResponse(success=False,
                                       message='refused: busy')
            self.tcp_target = list(self.arm_home)
            self.arm_busy = True
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                if not self.arm_busy:
                    return TriggerResponse(success=True, message='home ok')
            r.sleep()
        return TriggerResponse(success=False, message='shutdown')

    def _cb_lift_height(self, msg):
        with self.lock:
            self.lift_target = max(0.0, float(msg.data))

    def _cb_lift_cmd(self, msg):
        cmd = msg.data.strip().split()
        with self.lock:
            if cmd and cmd[0] == 'stop':
                self.lift_target = None
            elif cmd and cmd[0] == 'home':
                self.lift_target = 0.0
            elif len(cmd) == 2 and cmd[0] == 'mm':
                self.lift_target = max(0.0, float(cmd[1]))

    def _srv_lift_home(self, _req):
        with self.lock:
            self.lift_target = 0.0
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            with self.lock:
                if self.lift_target is None or abs(self.lift_mm) < 0.5:
                    self.lift_mm = 0.0
                    self.lift_target = None
                    return TriggerResponse(success=True,
                                           message='homed (sim)')
            r.sleep()
        return TriggerResponse(success=False, message='shutdown')

    def _srv_lift_stop(self, _req):
        with self.lock:
            self.lift_target = None
        return TriggerResponse(success=True, message='stopped')

    def _bump(self, ok, message):
        with self.lock:
            self.motion_seq += 1
            self.result_success = bool(ok)
            self.result_message = str(message)

    # ================= physics =================
    def _physics(self, _evt):
        with self.lock:
            # base: unicycle; commands decay after 0.3 s of silence,
            # actual velocity follows with a first-order lag (see init)
            if (rospy.Time.now() - self.last_cmd_t).to_sec() > 0.3:
                self.v_cmd = 0.0
                self.w_cmd = 0.0
            self.v_act += (self.v_cmd - self.v_act) * self.dt / self.v_tau
            self.w_act += (self.w_cmd - self.w_act) * self.dt / self.w_tau
            self.x += self.v_act * math.cos(self.theta) * self.dt
            self.y += self.v_act * math.sin(self.theta) * self.dt
            self.theta += self.w_act * self.dt
            self.theta = math.atan2(math.sin(self.theta),
                                    math.cos(self.theta))

            # arm: straight-line interpolation in pose space
            if self.tcp_target is not None:
                done = True
                step_l = self.arm_lin_mm_s * self.dt
                step_a = self.arm_ang_deg_s * self.dt
                for i in range(6):
                    step = step_l if i < 3 else step_a
                    d = self.tcp_target[i] - self.tcp[i]
                    if i >= 3:
                        d = (d + 180.0) % 360.0 - 180.0
                    if abs(d) > step:
                        self.tcp[i] += math.copysign(step, d)
                        done = False
                    else:
                        self.tcp[i] = self.tcp_target[i]
                if done:
                    self.tcp_target = None
                    self.arm_busy = False
                    self._bump_locked(True, 'move_cart ok')

            # lift
            if self.lift_target is not None:
                d = self.lift_target - self.lift_mm
                step = self.lift_mm_s * self.dt
                if abs(d) > step:
                    self.lift_mm += math.copysign(step, d)
                else:
                    self.lift_mm = self.lift_target
                    self.lift_target = None

        self._publish_odom()

    def _bump_locked(self, ok, message):
        self.motion_seq += 1
        self.result_success = bool(ok)
        self.result_message = str(message)

    # ================= outputs =================
    def _publish_odom(self):
        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = 'odom'
        n = (np.random.normal(0, self.odom_noise, 3)
             if self.odom_noise > 0 else (0.0, 0.0, 0.0))
        od.pose.pose.position.x = self.x + n[0]
        od.pose.pose.position.y = self.y + n[1]
        th = self.theta + n[2]
        od.pose.pose.orientation.z = math.sin(th / 2.0)
        od.pose.pose.orientation.w = math.cos(th / 2.0)
        od.twist.twist.linear.x = self.v_act
        od.twist.twist.angular.z = self.w_act
        self.pub_odom.publish(od)
        # camera world positions ride along so a viewer needs no
        # transform knowledge of its own
        with self.lock:
            T_wmb = self._T_world2mb()
            T_wfc = T_wmb @ self.T_mb2fc
            T_ab2mb_live = self.T_ab2mb.copy()
            T_ab2mb_live[2, 3] -= self.lift_mm / 1000.0
            T_whc = (T_wmb @ invert_T(T_ab2mb_live)
                     @ pose_fr5_to_matrix_m(self.tcp)
                     @ invert_T(self.T_hc2ee))
        self.pub_truth.publish(String(json.dumps({
            'x': self.x, 'y': self.y,
            'theta_deg': math.degrees(self.theta),
            'tcp': list(self.tcp), 'lift_mm': self.lift_mm,
            'v': self.v_act, 'w': self.w_act,
            'fc': [float(T_wfc[0, 3]), float(T_wfc[1, 3])],
            'hc': [float(T_whc[0, 3]), float(T_whc[1, 3]),
                   float(T_whc[2, 3])]})))

    def _publish_camera_info(self):
        ci = CameraInfo()
        ci.width, ci.height = self.W, self.H
        fx, fy, cx, cy = self.K
        ci.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.pub_fc_info.publish(ci)

    def _T_world2mb(self):
        T = np.eye(4)
        T[:3, :3] = rz(self.theta)
        T[:3, 3] = [self.x, self.y, FLOOR_Z]
        return T

    def _cameras(self, _evt):
        with self.lock:
            T_wmb = self._T_world2mb()
            tcp = list(self.tcp)
            lift_m = self.lift_mm / 1000.0
        stamp = rospy.Time.now()

        # front camera: fixed on the body
        T_wfc = T_wmb @ self.T_mb2fc
        self.pub_fc_det.publish(
            self._detect(T_wfc, 'front_cam', stamp))

        # hand camera: world -> ab -> ee(TCP) -> hc.
        # T_ab2mb is lift-at-origin; the LIVE arm base sits lift_m higher,
        # so mb is lift_m further below ab (same shift chain.compensate
        # applies on the consumer side).
        T_ab2mb_live = self.T_ab2mb.copy()
        T_ab2mb_live[2, 3] -= lift_m
        T_wab = T_wmb @ invert_T(T_ab2mb_live)
        T_ab2ee = pose_fr5_to_matrix_m(tcp)
        T_whc = T_wab @ T_ab2ee @ invert_T(self.T_hc2ee)
        self.pub_hc_det.publish(
            self._detect(T_whc, 'hand_cam', stamp))

    def _detect(self, T_wc, cam_name, stamp):
        arr = AprilTagDetectionArray()
        arr.header.stamp = stamp
        arr.header.frame_id = cam_name
        arr.camera_name = cam_name
        arr.image_width = self.W
        arr.image_height = self.H
        fx, fy, cx, cy = self.K
        T_cw = invert_T(T_wc)
        for tag in self.tags:
            T_ct = T_cw @ tag.T_w
            t = T_ct[:3, 3]
            if t[2] < 0.05:            # behind / at the camera
                continue
            u = fx * t[0] / t[2] + cx
            v = fy * t[1] / t[2] + cy
            if not (0 <= u < self.W and 0 <= v < self.H):
                continue
            # corners must all project into the frame too
            pc = T_ct @ tag.corners_t
            if np.any(pc[2] < 0.02):
                continue
            us = fx * pc[0] / pc[2] + cx
            vs = fy * pc[1] / pc[2] + cy
            if (us.min() < 0 or us.max() >= self.W
                    or vs.min() < 0 or vs.max() >= self.H):
                continue
            if self.noise_px > 0:
                u += np.random.normal(0, self.noise_px)
                v += np.random.normal(0, self.noise_px)
                us = us + np.random.normal(0, self.noise_px, 4)
                vs = vs + np.random.normal(0, self.noise_px, 4)

            # dt_apriltags reports t scaled by ITS configured size
            scale = self.det_size.get(cam_name, tag.size_m) / tag.size_m
            t_rep = t * scale
            if self.noise_t_m > 0:
                t_rep = t_rep + np.random.normal(0, self.noise_t_m, 3)

            d = AprilTagDetection()
            d.id = tag.id
            d.center_x, d.center_y = float(u), float(v)
            d.pose_x, d.pose_y, d.pose_z = [float(x) for x in t_rep]
            roll, pitch, yaw = rot_to_euler_zyx_deg(T_ct[:3, :3])
            d.roll, d.pitch, d.yaw = float(roll), float(pitch), float(yaw)
            d.corners = [float(x) for pair in zip(us, vs) for x in pair]
            d.tilt_from_normal = float(math.degrees(math.acos(
                min(1.0, abs(float(T_ct[2, 2]))))))
            arr.detections.append(d)
        return arr

    def _arm_state(self, _evt):
        with self.lock:
            msg = ArmState()
            msg.header.stamp = rospy.Time.now()
            msg.state = 'busy' if self.arm_busy else 'idle'
            msg.busy = self.arm_busy
            msg.tcp_pose = list(self.tcp)
            msg.pose_valid = True
            msg.joints = [0.0] * 6
            msg.motion_seq = self.motion_seq
            msg.result_message = self.result_message
            msg.result_success = self.result_success
        self.pub_arm.publish(msg)

    def _lift_state(self, _evt):
        with self.lock:
            state = {
                'position': int(self.lift_mm / 0.04976077),
                'height_mm': self.lift_mm,
                'mm_calibrated': True,
                'homed': True,
                'error': False, 'alarm': False,
                'busy': self.lift_target is not None,
                'estop': False,
                'soft_min': 0, 'soft_max': 6900,
                'scan_height': None,
            }
            h = self.lift_mm
        self.pub_lift_state.publish(String(json.dumps(state)))
        self.pub_lift_h.publish(Float32(h))

    def _publish_markers(self):
        ma = MarkerArray()
        for i, tag in enumerate(self.tags):
            mk = Marker()
            mk.header.frame_id = 'odom'
            mk.ns = tag.kind
            mk.id = i
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = tag.T_w[0, 3]
            mk.pose.position.y = tag.T_w[1, 3]
            mk.pose.position.z = tag.T_w[2, 3]
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = tag.size_m
            mk.scale.z = 0.002
            if tag.kind == 'cross':
                mk.color.r, mk.color.a = 1.0, 1.0
            else:
                mk.color.g, mk.color.a = 1.0, 1.0
            ma.markers.append(mk)
        self.pub_markers.publish(ma)


if __name__ == '__main__':
    SimNode()
    rospy.spin()
