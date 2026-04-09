#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import math
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from cv_bridge import CvBridge

# Fallback for apriltags
try:
    from dt_apriltags import Detector
except ImportError:
    import subprocess
    subprocess.check_call(['pip', 'install', 'dt-apriltags'])
    from dt_apriltags import Detector

from robot_msgs.msg import Pose2DWithFlag  # Custom message

class RobotController:
    """
    Low-level control of the robot.
    Handles:
    - Camera/Tag Detection
    - Movement (Pure Pursuit, Pivot, Alignment)
    - Communication with Manipulator (Scan logic)
    """

    def __init__(self, config, map_manager):
        self.cfg = config
        self.map_mgr = map_manager
        
        # Initialize Detector (Before subscribers to avoid callback errors)
        self.detector = Detector(families=self.cfg['robot']['tag_family'])
        
        # ROS Setup
        self.bridge = CvBridge()
        self.cmd_pub = rospy.Publisher(self.cfg['topics']['cmd_vel'], Twist, queue_size=10)
        self.pose_pub = rospy.Publisher(self.cfg['topics']['robot_pose'], Pose2DWithFlag, queue_size=10)
        
        self.image_sub = rospy.Subscriber(self.cfg['topics']['camera_rgb'], Image, self.image_callback)
        self.info_sub = rospy.Subscriber(self.cfg['topics']['camera_info'], CameraInfo, self.info_callback)
        self.odom_sub = rospy.Subscriber(self.cfg['topics']['odom'], Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber(self.cfg['topics']['scan_signal'], Bool, self.scan_callback)

        # Internal State
        self.detected_tags = {}  # {id: {x, y, z, yaw, ...}}
        self.current_theta = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.camera_params = None
        self.scan_finished_signal = False
        self.last_known_tag = None  # Tracks the last visited tag
        self.stop_sleep_duration = self.cfg['robot'].get('stop_sleep_duration', 0.0)

        # Velocity ramping state (for smooth acceleration/deceleration)
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_vel_time = None

        # Cache frequently used config values
        self.max_linear = self.cfg['robot']['max_linear_speed']
        self.max_angular = self.cfg['robot']['max_angular_speed']
        self.min_linear = self.max_linear * self.cfg['robot'].get('min_linear_factor', 0.3)
        self.min_angular = self.max_angular * self.cfg['robot'].get('min_angular_factor', 0.25)
        self.linear_accel = self.cfg['robot'].get('linear_accel', 0.05)
        self.angular_accel = self.cfg['robot'].get('angular_accel', 0.3)
        self.ramp_enabled = self.cfg['robot'].get('ramp_enabled', True)

        self.stop_requested = False      # Any interruption
        self.emergency_stop = False      # Only true STOP command

    def info_callback(self, msg):
        if self.camera_params is None:
            K = msg.K
            self.camera_params = [K[0], K[4], K[2], K[5]] # fx, fy, cx, cy
            rospy.loginfo("Camera params received.")

    def odom_callback(self, msg):
        # Extract position
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        # Extract orientation
        q = msg.pose.pose.orientation
        euler = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_theta = euler[2]

    def image_callback(self, msg):
        if self.camera_params is None: return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray, estimate_tag_pose=True, 
                                            camera_params=self.camera_params, 
                                            tag_size=self.cfg['robot']['tag_size'])
            
            self.detected_tags.clear()
            for det in detections:
                # Store relevant info
                # pose_t[0] = lateral (x), pose_t[2] = distance (z)
                self.detected_tags[det.tag_id] = {
                    'x': det.pose_t[0][0],
                    'y': det.pose_t[1][0],
                    'z': det.pose_t[2][0],
                    'corners': det.corners,
                    'center_y': det.center[1]
                }
            
            if self.detected_tags:
                rospy.loginfo_throttle(2.0, f"[Vision] Tags detected: {list(self.detected_tags.keys())}")
            else:
                rospy.logdebug_throttle(2.0, "[Vision] No tags detected.")
                
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")
            pass
            
    def scan_callback(self, msg):
        """External node says scan is done."""
        if msg.data:
            self.scan_finished_signal = True

    # =========================================================
    # MOVEMENT PRIMITIVES
    # =========================================================


    def emergency_stop_robot(self):
        rospy.logerr("[Robot] EMERGENCY STOP (external command)")
        self.stop_requested = True
        self.emergency_stop = True
        self.stop()

    def preempt_stop_robot(self):
        rospy.logwarn("[Robot] Preempt stop (task switch)")
        self.stop_requested = True
        self.emergency_stop = False
        self.stop()


    def stop(self):
        """Immediate stop - resets ramping state."""
        self.current_linear_vel = 0.0
        self.current_angular_vel = 0.0
        self.last_vel_time = None
        self._publish_vel(0, 0)

    def clear_stop_flag(self):
        self.stop_requested = False
        self.emergency_stop = False


    def _publish_vel(self, linear, angular):
        """Direct velocity publish without ramping."""
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def send_vel(self, linear, angular):
        """
        Send velocity command with optional ramping for smooth acceleration.
        Uses configured acceleration limits to prevent jerky motion.
        """
        if not self.ramp_enabled:
            self._publish_vel(linear, angular)
            return

        now = rospy.Time.now()
        if self.last_vel_time is None:
            dt = 0.05  # Default dt for first call
        else:
            dt = (now - self.last_vel_time).to_sec()
            dt = max(0.001, min(dt, 0.2))  # Clamp dt to reasonable range
        self.last_vel_time = now

        # Ramp linear velocity
        linear_diff = linear - self.current_linear_vel
        max_linear_change = self.linear_accel * dt
        if abs(linear_diff) > max_linear_change:
            self.current_linear_vel += max_linear_change if linear_diff > 0 else -max_linear_change
        else:
            self.current_linear_vel = linear

        # Ramp angular velocity
        angular_diff = angular - self.current_angular_vel
        max_angular_change = self.angular_accel * dt
        if abs(angular_diff) > max_angular_change:
            self.current_angular_vel += max_angular_change if angular_diff > 0 else -max_angular_change
        else:
            self.current_angular_vel = angular

        self._publish_vel(self.current_linear_vel, self.current_angular_vel)
        rospy.loginfo_throttle(1.0, f"[Velocity] target:{linear:.3f} -> actual:{self.current_linear_vel:.3f}")

    def get_current_tag_id(self):
        """
        Returns the ID of the closest/most central detected tag.
        Returns None if no tags visible.
        """
        if not self.detected_tags:
            return None
        # Simple heuristic: return the one with smallest lateral offset (closest to center)
        # or closest distance. Let's use distance (calculated from x and y for top-down view)
        best_id = min(self.detected_tags, key=lambda k: math.hypot(self.detected_tags[k]['x'], self.detected_tags[k]['y']))
        return best_id

    def align_to_tag(self, tag_id):
        rospy.loginfo(f"Aligning to tag {tag_id}...")
        rate = rospy.Rate(10)

        align_gain = self.cfg['robot'].get('align_gain', 0.8)
        align_threshold = self.cfg['robot'].get('align_threshold_deg', 0.5)

        while not rospy.is_shutdown():

            # ★ PREEMPT / STOP CHECK
            if self.stop_requested:
                rospy.logwarn("[Robot] align_to_tag interrupted")
                self.stop()
                return False

            if tag_id not in self.detected_tags:
                self.stop()
                rate.sleep()
                continue

            corners = self.detected_tags[tag_id]['corners']
            dx = corners[1][0] - corners[0][0]
            dy = corners[1][1] - corners[0][1]
            angle_deg = math.degrees(math.atan2(dy, dx))

            if angle_deg > 90:
                angle_deg -= 180
            elif angle_deg < -90:
                angle_deg += 180

            if abs(angle_deg) < align_threshold:
                self.stop()
                rospy.loginfo(f"Alignment Complete. Final Angle: {angle_deg:.2f}")
                return True

            angular_vel = -math.radians(angle_deg) * align_gain
            angular_vel = np.clip(angular_vel, -self.max_angular, self.max_angular)

            self.send_vel(0, angular_vel)
            rate.sleep()


    def calculate_robot_pose(self, tag_id):
        """
        Calculates the robot center's pose relative to the tag, 
        mirroring the original V9 logic.
        """
        if tag_id not in self.detected_tags:
            return None
            
        tag = self.detected_tags[tag_id]
        lateral = tag['x'] # + = tag is right of camera
        
        # Original V9 uses corners to get a precise alignment angle
        corners = tag.get('corners')
        align_angle_deg = 0.0
        if corners is not None:
            dx = corners[1][0] - corners[0][0]
            dy = corners[1][1] - corners[0][1]
            align_angle_deg = math.degrees(math.atan2(dy, dx))
            # Normalize to [-90, 90]
            if align_angle_deg > 90: align_angle_deg -= 180
            elif align_angle_deg < -90: align_angle_deg += 180

        tag_info = self.map_mgr.get_tag_info(tag_id)
        zone = tag_info.get('zone', 'A') if tag_info else 'A'
        
        # 1. Calculate Isaac Sim Coordinates (Camera Position)
        # Mirroring get_robot_pose_from_tag logic
        tag_x = tag_info['x'] if tag_info else 0.0
        tag_y = tag_info['y'] if tag_info else 0.0
        
        if zone == 'A' or zone == 'DOCK':
            robot_x, robot_y = tag_x, tag_y + lateral
            heading = align_angle_deg
        elif zone in ['B', 'D']:
            robot_x, robot_y = tag_x - lateral, tag_y
            heading = 90 + align_angle_deg
        elif zone in ['C', 'E']:
            robot_x, robot_y = tag_x + lateral, tag_y
            heading = -90 + align_angle_deg
        else:
            robot_x, robot_y, heading = tag_x, tag_y, 0.0

        # 2. Convert to Manipulator Coordinates (isaac_to_manipulator)
        manip_cam_x = -robot_y
        manip_cam_y = -robot_x
        
        # 3. Apply Camera-to-Robot-Center Offset (apply_robot_center_offset)
        cam_offset = self.cfg['robot'].get('camera_offset', 0.45)
        
        if zone == 'A' or zone == 'DOCK':
            final_x, final_y = manip_cam_x, manip_cam_y + cam_offset
        elif zone in ['B', 'D']:
            final_x, final_y = manip_cam_x + cam_offset, manip_cam_y
        elif zone in ['C', 'E']:
            final_x, final_y = manip_cam_x - cam_offset, manip_cam_y
        else:
            final_x, final_y = manip_cam_x, manip_cam_y

        return final_x, final_y, heading

    def go_to_next_tag(self, target_id, known_start_id=None):

        # ★ NEW: early abort
        if self.stop_requested:
            rospy.logwarn("[Robot] go_to_next_tag aborted")
            return False

        current_id = self.get_current_tag_id()

        if current_id is None:
            if known_start_id is not None:
                rospy.logwarn(f"No tag visible. Using known start ID: {known_start_id}")
                current_id = known_start_id
            elif self.last_known_tag is not None:
                rospy.logwarn(f"No tag visible. Using last known tag: {self.last_known_tag}")
                current_id = self.last_known_tag
            else:
                rospy.logwarn("No tag visible and no known location. Cannot start move.")
                return False

        edge = self.map_mgr.get_edge(current_id, target_id)
        if not edge:
            rospy.logerr(f"No edge from {current_id} to {target_id}")
            return False

        action_type = edge['type']
        direction = edge['direction']

        rospy.loginfo(
            f"Going to {target_id} from {current_id} ({action_type}, {direction})"
        )

        if action_type == 'pivot':
            return self.execute_pivot(direction)

        elif action_type == 'move':
            start_info = self.map_mgr.get_tag_info(current_id)
            target_info = self.map_mgr.get_tag_info(target_id)

            if not start_info or not target_info:
                rospy.logerr("Missing tag info for move")
                return False

            dx = target_info['x'] - start_info['x']
            dy = target_info['y'] - start_info['y']
            total_distance = math.hypot(dx, dy)

            ok = self.execute_pure_pursuit(
                target_id=target_id,
                direction=direction,
                total_distance=total_distance
            )

            if not ok:
                return False

            return self.align_to_tag(target_id)

        else:
            rospy.logerr(f"Unknown edge type: {action_type}")
            return False

        
    def move_to_tag(self, target_id):
        """
        Move robot to target tag using MapManager.find_path(),
        executing each step via go_to_next_tag().
        """

        rospy.loginfo(f"[Robot] move_to_tag (path-based) → {target_id}")

        # ★ NEW: clear stop flag for new command
        self.clear_stop_flag()

        start_id = self.get_current_tag_id()

        if start_id is None:
            start_id = self.last_known_tag
            if start_id is not None:
                rospy.logwarn(
                    f"[Robot] No tag visible, fallback to last_known_tag={start_id}"
                )

        if start_id is None:
            rospy.logerr("[Robot] move_to_tag failed: cannot determine start tag")
            return False

        if start_id == target_id:
            rospy.loginfo(f"[Robot] Already at target tag {target_id}")
            self.last_known_tag = target_id
            self.publish_robot_pose(target_id)
            return True

        path = self.map_mgr.find_path(start_id, target_id)

        if not path or len(path) < 2:
            rospy.logerr(
                f"[Robot] No valid path from {start_id} to {target_id}"
            )
            return False

        rospy.loginfo(f"[Robot] Path: {path}")

        current_id = start_id

        for next_id in path[1:]:

            # ★ NEW: interrupt check
            if self.stop_requested:
                rospy.logwarn("[Robot] move_to_tag interrupted")
                return False

            rospy.loginfo(f"[Robot] Step {current_id} → {next_id}")

            ok = self.go_to_next_tag(
                target_id=next_id,
                known_start_id=current_id
            )

            if not ok:
                rospy.logerr(
                    f"[Robot] Failed at step {current_id} → {next_id}"
                )
                return False

            current_id = next_id
            self.last_known_tag = next_id

        rospy.loginfo(f"[Robot] move_to_tag success → {target_id}")
        self.publish_robot_pose(target_id)
        return True


    def execute_pivot(self, direction):
        target_angle = self.current_theta + (
            math.pi/2 if 'ccw' in direction else -math.pi/2
        )
        target_angle = math.atan2(math.sin(target_angle), math.cos(target_angle))

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            # ★ PREEMPT / STOP CHECK
            if self.stop_requested:
                rospy.logwarn("[Robot] pivot interrupted")
                self.stop()
                return False

            diff = target_angle - self.current_theta
            diff = math.atan2(math.sin(diff), math.cos(diff))

            if abs(diff) < math.radians(1.0):
                self.stop()
                rospy.loginfo(
                    f"Pivot Complete. Final error: {math.degrees(diff):.2f} deg"
                )
                return True

            vel = diff * self.cfg['robot'].get('pivot_gain', 1.5)
            vel = np.clip(vel, -self.max_angular, self.max_angular)

            self.send_vel(0, vel)
            rate.sleep()


    def _smooth_speed_factor(self, ratio, phase='accel'):
        """
        Converts linear ratio [0,1] to smooth speed factor.
        For acceleration: fast ramp up (sqrt curve)
        For deceleration: smooth ramp down (quadratic curve)
        """
        ratio = max(0.0, min(1.0, ratio))  # Clamp to [0, 1]

        if phase == 'accel':
            # Square root curve: ramps up quickly at start, then slows
            # ratio=0 -> 0, ratio=0.25 -> 0.5, ratio=1 -> 1
            return math.sqrt(ratio)
        else:
            # Quadratic curve for smooth deceleration
            # ratio=0 -> 0, ratio=1 -> 1, smooth end
            return ratio * ratio

    def execute_pure_pursuit(self, target_id, direction, total_distance=1.0):
        """
        Moves towards target_id using Pure Pursuit logic.
        Speed is controlled purely by distance (odom-based), not tag detection.
        Tag detection is only used for steering and final stop condition.
        """

        
        rate = rospy.Rate(20)
        move_dir_sign = -1 if 'backward' in direction else 1

        # Load velocity profile parameters from config
        accel_dist = total_distance * self.cfg['robot'].get('s_curve_accel_ratio', 0.15)
        decel_dist = total_distance * self.cfg['robot'].get('s_curve_decel_ratio', 0.20)
        min_speed = self.max_linear * self.cfg['robot'].get('s_curve_min_speed_factor', 0.5)

        # Safety timeout
        timeout_limit = self.cfg['robot'].get('navigation_timeout', 8.0)
        # Also calculate based on distance at minimum speed
        worst_case_time = total_distance / min_speed if min_speed > 0 else 30.0
        timeout_limit = max(timeout_limit, worst_case_time * 1.5)
        start_time = rospy.Time.now()

        # Record starting odometry position
        start_odom_x = self.odom_x
        start_odom_y = self.odom_y

        rospy.loginfo(f"[Navigation] Moving {total_distance:.2f}m to tag {target_id}, timeout: {timeout_limit:.1f}s")

        while not rospy.is_shutdown():
            # Check timeout
            if self.stop_requested:
                rospy.logwarn("[Robot] pure_pursuit interrupted")
                self.stop()
                return False
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout_limit:
                self.stop()
                rospy.logerr(f"[Watchdog] TIMEOUT ({elapsed:.1f}s). Target {target_id} not reached.")
                return False

            # Calculate distance traveled using odometry
            traveled_dist = math.hypot(self.odom_x - start_odom_x, self.odom_y - start_odom_y)
            remaining_dist = max(0.0, total_distance - traveled_dist)

            # ===== VELOCITY PROFILE (distance-based) =====
            # Acceleration phase: ramp up in first accel_dist
            if traveled_dist < accel_dist and accel_dist > 0:
                accel_factor = traveled_dist / accel_dist
                target_speed = min_speed + (self.max_linear - min_speed) * accel_factor
            # Deceleration phase: ramp down in last decel_dist
            elif remaining_dist < decel_dist and decel_dist > 0:
                decel_factor = remaining_dist / decel_dist
                target_speed = min_speed + (self.max_linear - min_speed) * decel_factor
            # Cruise phase: full speed
            else:
                target_speed = self.max_linear

            # Apply direction
            speed = target_speed * move_dir_sign

            # ===== STEERING (tag-based if visible) =====
            omega = 0.0
            tag_visible = target_id in self.detected_tags

            if tag_visible:
                tag = self.detected_tags[target_id]
                lateral = tag['x']
                dist_to_tag = tag['z']

                # Check stop condition (tag center crosses image center)
                center_y = tag['center_y']
                image_center_y = self.camera_params[3]
                stop_offset = self.cfg['robot'].get('center_y_stop_offset', 0.0)
                stop_tolerance = self.cfg['robot'].get('center_y_stop_tolerance', 10.0)
                target_y = image_center_y + stop_offset
                diff = center_y - target_y

                should_stop = False
                if move_dir_sign > 0 and diff >= -stop_tolerance:
                    should_stop = True
                elif move_dir_sign < 0 and diff <= stop_tolerance:
                    should_stop = True

                if should_stop:
                    self.stop()
                    rospy.loginfo(f"Arrived at {target_id} (traveled:{traveled_dist:.3f}m)")
                    return True

                # Pure Pursuit steering
                L = max(dist_to_tag, self.cfg['robot']['look_ahead_base'])
                alpha = math.atan2(-lateral, L)
                curvature = (2.0 * math.sin(alpha)) / L
                gain = self.cfg['robot']['pp_gain_backward'] if move_dir_sign < 0 else self.cfg['robot']['pp_gain_forward']
                omega = abs(speed) * curvature * gain
                omega = np.clip(omega, -self.max_angular, self.max_angular)

            # Also check if we've traveled the expected distance (backup stop)
            if traveled_dist >= total_distance * 0.95:
                # We've traveled most of the distance, slow down and look for tag
                if not tag_visible:
                    speed = min_speed * move_dir_sign
                    rospy.logwarn_throttle(0.5, f"Near target but tag not visible, slowing down")

            # Performance logging
            tag_str = "TAG" if tag_visible else "BLIND"
            rospy.loginfo_throttle(0.5, f"[{tag_str}] traveled:{traveled_dist:.3f} remain:{remaining_dist:.3f} spd:{speed:+.3f}")

            self.send_vel(speed, omega)
            rate.sleep()

        return False
    
    def publish_robot_pose(self, tag_id):
        """
        Publish robot pose for manipulator coordinate transform.
        This replaces perform_scan_procedure's pose publishing role.
        """
        pose = self.calculate_robot_pose(tag_id)

        msg = Pose2DWithFlag()
        msg.header.stamp = rospy.Time.now()
        msg.flag = True
        msg.id = tag_id

        if pose is not None:
            msg.x, msg.y, msg.theta = pose
            rospy.loginfo(
                f"[RobotPose] tag {tag_id} "
                f"x={msg.x:.3f}, y={msg.y:.3f}, theta={msg.theta:.2f}"
            )
        else:
            rospy.logwarn(
                f"[RobotPose] Failed to compute pose for tag {tag_id}, using zeros"
            )
            msg.x = 0.0
            msg.y = 0.0
            msg.theta = 0.0

        self.pose_pub.publish(msg)
