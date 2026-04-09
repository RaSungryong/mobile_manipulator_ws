#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Extended ArmController - Integrates camera capture and Ra inference
Adds image capture and inference functionality during scanning on top of the original robotic arm control
"""

import os
import rospy
import sys
import time
import numpy as np
import cv2
import torch
from scipy.spatial.transform import Rotation as R
from typing import Optional
from std_msgs.msg import Bool, Float32, String
from robot_msgs.msg import Pose2DWithFlag
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

import roboticstoolbox as rtb
from roboticstoolbox import jtraj
from spatialmath import SE3, UnitQuaternion

from pypylon import pylon
import torchvision.transforms as transforms
from PIL import Image as PILImage
import json
import onnxruntime as ort


# URDF path (resolved relative to this script)
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_URDF_PATH = os.path.join(
    _SCRIPT_DIR, '..', '..', 'frcobot_ros', 'frcobot_description',
    'urdf', 'fr10v6_vision.urdf'
)

JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

# Trajectory interpolation rate (Hz)
TRAJ_RATE = 50
# Default number of waypoints for jtraj
DEFAULT_STEPS = 100


class CameraInterface:
    """
    General camera interface - supports Basler and Webcam
    Prioritizes Basler, automatically falls back to Webcam on failure
    """
    
    def __init__(self, use_webcam=False):
        self.camera = None
        self.camera_type = None  # 'basler' or 'webcam'
        self.use_webcam = use_webcam
        
        # Basler related
        self.converter = None
        if not use_webcam:
            try:
                self.converter = pylon.ImageFormatConverter()
                self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
                self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
            except:
                pass
        
        self.bridge = CvBridge()
        
    def initialize(self):
        """Initialize camera - prioritize Basler, fall back to Webcam on failure"""
        
        # If forced to use webcam
        if self.use_webcam:
            rospy.logwarn("[Camera] Forced to use webcam mode")
            return self._initialize_webcam()
        
        # Try Basler first
        if self._initialize_basler():
            return True
        
        # Basler failed, try Webcam
        rospy.logwarn("[Camera] Basler not available, trying webcam...")
        return self._initialize_webcam()
    
    def _initialize_basler(self):
        """Initialize Basler camera"""
        try:
            tlFactory = pylon.TlFactory.GetInstance()
            devices = tlFactory.EnumerateDevices()
            
            if len(devices) == 0:
                rospy.logwarn("[Camera] No Basler camera found")
                return False
            
            self.camera = pylon.InstantCamera(tlFactory.CreateDevice(devices[0]))
            self.camera.Open()
            
            self.camera.PixelFormat.SetValue("BayerRG8")
            self.camera.ExposureTime.SetValue(50000)
            self.camera.Gain.SetValue(0)
            self.camera.AcquisitionMode.SetValue("Continuous")
            
            self.camera_type = 'basler'
            rospy.loginfo(f"[Camera] Basler initialized: {self.camera.GetDeviceInfo().GetModelName()}")
            return True
            
        except Exception as e:
            rospy.logwarn(f"[Camera] Basler initialization failed: {e}")
            if self.camera:
                try:
                    self.camera.Close()
                except:
                    pass
                self.camera = None
            return False
    
    def _initialize_webcam(self):
        """Initialize Webcam"""
        try:
            # Try multiple device indices
            for device_id in range(4):
                rospy.loginfo(f"[Camera] Trying webcam device {device_id}...")
                
                cap = cv2.VideoCapture(device_id)
                
                if not cap.isOpened():
                    continue
                
                # Try reading one frame for testing
                ret, frame = cap.read()
                if ret and frame is not None and frame.size > 0:
                    # Set resolution
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    
                    self.camera = cap
                    self.camera_type = 'webcam'
                    
                    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    
                    rospy.loginfo(f"[Camera] Webcam initialized: device {device_id}, "
                                f"resolution {actual_width}x{actual_height}")
                    return True
                else:
                    cap.release()
            
            rospy.logerr("[Camera] No working webcam found")
            return False
            
        except Exception as e:
            rospy.logerr(f"[Camera] Webcam initialization failed: {e}")
            return False
    
    def start_grabbing(self):
        """Start grabbing"""
        if self.camera is None:
            return False
        
        if self.camera_type == 'basler':
            if self.camera.IsOpen():
                self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
                return True
        elif self.camera_type == 'webcam':
            # Webcam does not need an explicit start
            return True
        
        return False
    
    def grab_frame(self, timeout=5000):
        """Grab a frame"""
        if self.camera is None:
            return None
        
        try:
            if self.camera_type == 'basler':
                return self._grab_basler_frame(timeout)
            elif self.camera_type == 'webcam':
                return self._grab_webcam_frame()
        except Exception as e:
            rospy.logerr(f"[Camera] Error grabbing frame: {e}")
            return None
        
        return None
    
    def _grab_basler_frame(self, timeout):
        """Grab Basler image"""
        if not self.camera.IsGrabbing():
            return None
        
        grabResult = self.camera.RetrieveResult(timeout, pylon.TimeoutHandling_ThrowException)
        
        if grabResult.GrabSucceeded():
            image = self.converter.Convert(grabResult)
            img_array = image.GetArray()
            grabResult.Release()
            return img_array
        else:
            grabResult.Release()
            return None
    
    def _grab_webcam_frame(self):
        """Grab Webcam image"""
        ret, frame = self.camera.read()
        
        if ret and frame is not None:
            return frame
        return None
    
    def stop_grabbing(self):
        """Stop grabbing"""
        if self.camera is None:
            return
        
        try:
            if self.camera_type == 'basler':
                if self.camera.IsGrabbing():
                    self.camera.StopGrabbing()
            # Webcam does not need an explicit stop
        except Exception as e:
            rospy.logwarn(f"[Camera] Error stopping: {e}")
    
    def close(self):
        """Close camera"""
        if self.camera is None:
            return
        
        try:
            if self.camera_type == 'basler':
                if self.camera.IsGrabbing():
                    self.camera.StopGrabbing()
                if self.camera.IsOpen():
                    self.camera.Close()
            elif self.camera_type == 'webcam':
                self.camera.release()
            
            rospy.loginfo(f"[Camera] {self.camera_type.capitalize()} closed")
        except Exception as e:
            rospy.logwarn(f"[Camera] Error closing: {e}")
        
        self.camera = None
        self.camera_type = None



class InferenceInterface:
    """Inference Interface -- ONNX Runtime (CPU)"""

    def __init__(self, device='cpu', use_fp16=False):
        # device / use_fp16 kept for parameter compatibility, not used for ONNX CPU inference
        self.device = device
        self.use_fp16 = use_fp16
        self.session = None
        self.input_name = None

        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((900, 900)),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
        ])

    def load_model(self, model_path: str) -> bool:
        """Load ONNX model"""
        try:
            rospy.loginfo("[Inference] Loading ONNX model...")

            sess_options = ort.SessionOptions()
            sess_options.intra_op_num_threads = max(1, os.cpu_count() // 2)
            sess_options.graph_optimization_level = (
                ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            )

            self.session = ort.InferenceSession(
                model_path,
                sess_options=sess_options,
                providers=["CPUExecutionProvider"],
            )
            self.input_name = self.session.get_inputs()[0].name

            # Warm-up
            dummy = np.random.randn(1, 3, 900, 900).astype(np.float32)
            for _ in range(3):
                self.session.run(None, {self.input_name: dummy})

            rospy.loginfo("[Inference] ONNX model loaded successfully")
            return True

        except Exception as e:
            rospy.logerr(f"[Inference] Failed to load ONNX model: {e}")
            return False

    def infer(self, image) -> Optional[float]:
        """Inference, inputs BGR numpy image, returns float"""
        if self.session is None:
            return None

        try:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(image_rgb)
            # transform outputs torch.Tensor, convert to numpy
            input_np = (
                self.transform(pil_image)
                .unsqueeze(0)          # [1, 3, 900, 900]
                .numpy()
                .astype(np.float32)
            )

            output = self.session.run(None, {self.input_name: input_np})[0]
            return float(output.squeeze())

        except Exception as e:
            rospy.logerr(f"[Inference] Error: {e}")
            return None


class ArmController:
    """
    Extended ArmController - Integrates scanning and inference
    
    At each scan point:
    1. Move arm to position
    2. Wait for stabilization
    3. Capture multiple images
    4. Infer Ra value
    5. Calculate statistics
    6. Publish results
    """

    def __init__(self, model_path=None):
        self.busy = False
        self.cancel_requested = False
        self.current_pose_msg = None
        self.model_path = model_path
        self.goals_queue = []

        # ---------- ROS node ----------
        if not rospy.core.is_initialized():
            rospy.init_node('arm_controller', anonymous=True)

        # ---------- roboticstoolbox ----------
        urdf_path = os.path.normpath(_URDF_PATH)
        rospy.loginfo(f"[Arm] Loading URDF: {urdf_path}")
        self.robot = rtb.ERobot.URDF(urdf_path)
        rospy.loginfo(f"[Arm] Robot loaded, DOF={self.robot.n}")

        # ---------- Home ----------
        self.home_joint_positions = np.array([
            -1.5708, -1.5708, 1.5708,
            -1.5708, -1.5708, 0.0
        ])
        self.current_q = np.copy(self.home_joint_positions)

        # ---------- Camera & Inference ----------
        # Support forcing webcam via ROS parameter for testing
        use_webcam = rospy.get_param('~use_webcam', False)
        self.camera = CameraInterface(use_webcam=use_webcam)
        self.inference = InferenceInterface(device='cuda:0', use_fp16=True)
        
        # Scan parameters (configurable via ROS parameters)
        self.num_samples = rospy.get_param('~num_samples', 1)
        self.delay_between_samples = rospy.get_param('~delay_between_samples', 0.2)
        self.stabilization_time = rospy.get_param('~stabilization_time', 0.5)
        self.save_images = rospy.get_param('~save_images', False)
        self.output_dir = rospy.get_param('~output_dir', '/tmp/scan_results')
        
        # ---------- Initialize camera ----------
        if not self.camera.initialize():
            rospy.logerr("[Arm] Camera initialization failed")
        else:
            self.camera.start_grabbing()
        
        if not self.inference.load_model(self.model_path):
            rospy.logerr("[Arm] Model loading failed")

        rospy.loginfo("[Arm] Move to home")
        self._publish_joint_state(self.home_joint_positions)
        self.current_q = np.copy(self.home_joint_positions)

        # ---------- ROS ----------
        rospy.Subscriber('/robot_pose', Pose2DWithFlag, self.pose_cb, queue_size=1)

        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.done_pub = rospy.Publisher('/scan_finished', Bool, queue_size=1)
        self.ra_pub = rospy.Publisher('/scan/ra_value', Float32, queue_size=10)
        self.result_pub = rospy.Publisher('/scan/point_result', String, queue_size=10)
        self.image_pub = rospy.Publisher('/scan/image', Image, queue_size=1)

        self.bridge = CvBridge()

        rospy.loginfo("[ArmController] Ready with camera and inference (roboticstoolbox)")

    # --------------------------------------------------
    # robot_pose cache
    # --------------------------------------------------
    def pose_cb(self, msg):
        self.current_pose_msg = msg

    # --------------------------------------------------
    # MAIN ENTRY - Scan execution
    # --------------------------------------------------
    def execute_scan_points(self, scan_points):
        """
        Execute sequence of scan points
        Each point will: Move -> Stabilize -> Sample -> Infer -> Stat
        """
        if self.busy:
            rospy.logwarn("[Arm] Busy, ignore scan request")
            return

        if not scan_points:
            rospy.logwarn("[Arm] Empty scan_points")
            self.publish_done()
            return

        # pose scan requires robot_pose
        if any(p["mode"] == "pose" for p in scan_points):
            if self.current_pose_msg is None:
                rospy.logerr("[Arm] pose scan requires /robot_pose")
                return

        self.busy = True
        self.cancel_requested = False 
        try:
            self.goals_queue = self._build_goals(scan_points)

            if not self.goals_queue:
                rospy.logwarn("[Arm] No valid goals")
                self.publish_done()
                return

            self._execute_goals_with_scan()

        except Exception as e:
            rospy.logerr(f"[Arm] Scan exception: {e}")
            import traceback
            traceback.print_exc()

        finally:
            rospy.loginfo("[Arm] Scan finished → home")
            self.move_to_home()
            self.goals_queue = []
            self.busy = False

    # --------------------------------------------------
    # BUILD GOALS
    # --------------------------------------------------
    def _build_goals(self, scan_points):
        goals = []

        pose_points = []
        for p in scan_points:
            if p["mode"] == "pose":
                pose_points.append(p)
            elif p["mode"] == "joint":
                goals.append(("joint", p["joints"], p.get("speed", 80)))

        # process pose points
        if pose_points:
            raw = []
            for i, p in enumerate(pose_points):
                raw.append({
                    "point_id": i,
                    "x": p["x"],
                    "y": p["y"],
                    "z": p["z"],
                    "rx": p["rx"],
                    "ry": p["ry"],
                    "rz": p["rz"],
                    "speed": p.get("speed", 80)
                })

            transformed = self.process_transforms(raw, self.current_pose_msg)
            for idx, (pos, quat, speed) in enumerate(transformed):
                q0 = pose_points[idx].get("q0")  # IK seed from joint CSV
                goals.append(("pose", pos, quat, speed, q0))

        return goals

    # --------------------------------------------------
    # EXECUTE WITH SCAN - Core functionality
    # --------------------------------------------------
    def _execute_goals_with_scan(self):
        """Execute goal sequence, sample and infer at each point"""
        rospy.loginfo(f"[Arm] Execute {len(self.goals_queue)} goals with scan")

        for i, goal in enumerate(self.goals_queue):

            if self.cancel_requested:
                rospy.logwarn("[Arm] Scan cancelled")
                break

            rospy.loginfo(f"[Arm] === Point {i+1}/{len(self.goals_queue)} ===")

            # 1. Move to target position
            if goal[0] == "pose":
                _, pos, quat, speed, q0 = goal
                self._execute_pose_goal(pos, quat, speed, q0)
            elif goal[0] == "joint":
                _, joints, speed = goal
                self._execute_joint_goal(joints, speed)

            # 2. Wait for stabilization
            rospy.loginfo(f"[Arm] Stabilizing for {self.stabilization_time}s...")
            rospy.sleep(self.stabilization_time)

            # 3. Execute sampling and inference
            if not self.cancel_requested:
                self._scan_at_current_position(point_id=i)

        self.publish_done()

    def _scan_at_current_position(self, point_id):
        """
        Scan at the current position
        Capture multiple images, infer, calculate statistics
        """
        rospy.loginfo(f"[Scan] Starting scan at point {point_id} ({self.num_samples} samples)")

        results = []
        images = []

        for sample_id in range(self.num_samples):
            
            if self.cancel_requested:
                rospy.logwarn("[Scan] Scan cancelled")
                break

            # Capture image
            frame = self.camera.grab_frame()

            if frame is None:
                rospy.logwarn(f"  [Scan] Failed to capture sample {sample_id+1}/{self.num_samples}")
                continue

            # Inference
            start_time = rospy.Time.now()
            ra_value = self.inference.infer(frame)
            inference_time = (rospy.Time.now() - start_time).to_sec()

            if ra_value is not None:
                result = {
                    'point_id': point_id,
                    'sample_id': sample_id + 1,
                    'ra_value': ra_value,
                    'inference_time': inference_time,
                    'timestamp': rospy.Time.now().to_sec()
                }
                results.append(result)
                images.append(frame)

                # Publish real-time Ra value
                self.ra_pub.publish(Float32(ra_value))

                rospy.loginfo(f"  Sample {sample_id+1}/{self.num_samples}: "
                             f"Ra={ra_value:.4f}, Time={inference_time*1000:.2f}ms")
                
                # Publish image
                try:
                    img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.image_pub.publish(img_msg)
                except Exception as e:
                    rospy.logwarn(f"[Scan] Failed to publish image: {e}")

            # Wait for next sampling
            if sample_id < self.num_samples - 1:
                rospy.sleep(self.delay_between_samples)

        # Calculate statistics
        if len(results) > 0:
            ra_values = [r['ra_value'] for r in results]
            
            scan_result = {
                'point_id': point_id,
                'num_samples': len(results),
                'ra_mean': float(np.mean(ra_values)),
                'ra_std': float(np.std(ra_values)),
                'ra_min': float(np.min(ra_values)),
                'ra_max': float(np.max(ra_values)),
                'ra_values': ra_values,
                'timestamp': rospy.Time.now().to_sec()
            }

            # Publish results
            self.result_pub.publish(json.dumps(scan_result))

            rospy.loginfo("-"*60)
            rospy.loginfo(f"[Scan] Point {point_id} Results:")
            rospy.loginfo(f"  Samples: {scan_result['num_samples']}")
            rospy.loginfo(f"  Ra Mean: {scan_result['ra_mean']:.4f}")
            rospy.loginfo(f"  Ra Std:  {scan_result['ra_std']:.4f}")
            rospy.loginfo(f"  Ra Range: [{scan_result['ra_min']:.4f}, {scan_result['ra_max']:.4f}]")
            rospy.loginfo("-"*60)

            # Save images (optional)
            if self.save_images and len(images) > 0:
                import os
                os.makedirs(self.output_dir, exist_ok=True)
                timestamp = int(rospy.Time.now().to_sec())
                
                for i, img in enumerate(images):
                    filename = f"point_{point_id}_sample_{i+1}_ra_{ra_values[i]:.4f}.png"
                    filepath = os.path.join(self.output_dir, filename)
                    cv2.imwrite(filepath, img)
                
                rospy.loginfo(f"  Saved {len(images)} images to {self.output_dir}")

        else:
            rospy.logerr(f"[Scan] No valid samples at point {point_id}")

    # --------------------------------------------------
    # Publish a single JointState message
    # --------------------------------------------------
    def _publish_joint_state(self, q):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = JOINT_NAMES
        msg.position = q.tolist() if isinstance(q, np.ndarray) else list(q)
        self.joint_pub.publish(msg)

    # --------------------------------------------------
    # Execute a trajectory (list of joint waypoints)
    # --------------------------------------------------
    def _execute_trajectory(self, traj_q, rate_hz=TRAJ_RATE):
        rate = rospy.Rate(rate_hz)
        for q in traj_q:
            if self.cancel_requested or rospy.is_shutdown():
                break
            self._publish_joint_state(q)
            self.current_q = np.copy(q)
            rate.sleep()

    # --------------------------------------------------
    # POSE GOAL (IK + trajectory interpolation)
    # --------------------------------------------------
    def _execute_pose_goal(self, pos, quat, speed, q0=None):
        # Build SE3 target from position + quaternion (xyzw)
        rotation = UnitQuaternion(s=quat[3], v=quat[:3])
        T_target = SE3.Rt(rotation.R, pos)

        # IK seed: prefer paired joint data, fallback to current joints
        if q0 is not None:
            ik_seed = np.array(q0, dtype=float)
        else:
            ik_seed = self.current_q

        # Numerical IK
        dist = np.linalg.norm(pos)
        rospy.loginfo(f"[Arm] IK target in base_link: pos={np.round(pos, 4)}, dist={dist:.3f}m")
        sol = self.robot.ikine_LM(T_target, q0=ik_seed)
        if not sol.success:
            rospy.logerr(f"[Arm] IK failed for pos={np.round(pos, 4)}, dist={dist:.3f}m, skipping")
            return

        q_target = sol.q
        steps = max(20, int(DEFAULT_STEPS * (100 - speed) / 100) + 20)
        traj = jtraj(self.current_q, q_target, steps)

        rospy.loginfo(f"[Arm] Pose goal: {np.round(pos, 3)}, steps={steps}")
        self._execute_trajectory(traj.q)

    # --------------------------------------------------
    # JOINT GOAL (direct interpolation)
    # --------------------------------------------------
    def _execute_joint_goal(self, joints, speed):
        if len(joints) != 6:
            rospy.logerr("[Arm] joint goal must have 6 values")
            return

        q_target = np.array(joints, dtype=float)
        steps = max(20, int(DEFAULT_STEPS * (100 - speed) / 100) + 20)
        traj = jtraj(self.current_q, q_target, steps)

        rospy.loginfo(f"[Arm] Joint goal: {np.round(q_target, 3)}, steps={steps}")
        self._execute_trajectory(traj.q)

    # --------------------------------------------------
    # HOME
    # --------------------------------------------------
    def move_to_home(self):
        traj = jtraj(self.current_q, self.home_joint_positions, 80)
        self._execute_trajectory(traj.q)

    # --------------------------------------------------
    # TRANSFORM
    # --------------------------------------------------
    def create_homogeneous_transform(self, Rm, t):
        T = np.eye(4)
        T[:3, :3] = Rm
        T[:3, 3] = t
        return T

    def process_transforms(self, goals, msg):
        """
        Transform goal poses from Isaac Sim world frame to arm base_link frame.
        Calibrated 9-DOF model. See arm_controller.py for full documentation.
        """
        transformed = []

        # --- Calibrated parameters (9-DOF) ---
        body_off_x = rospy.get_param('~arm_body_offset_x', -0.166715)
        body_off_y = rospy.get_param('~arm_body_offset_y', -0.254772)
        arm_base_z = rospy.get_param('~arm_base_z', 0.974167)
        tilt_x     = rospy.get_param('~arm_tilt_x', 0.054898)
        tilt_y     = rospy.get_param('~arm_tilt_y', 0.017894)
        h_bias     = rospy.get_param('~arm_heading_bias', 0.014368)
        ori_corr_x = rospy.get_param('~arm_ori_corr_x', -0.095520)
        ori_corr_y = rospy.get_param('~arm_ori_corr_y', -0.052944)
        ori_corr_z = rospy.get_param('~arm_ori_corr_z', -0.008688)

        R_corr = R.from_euler('xyz', [ori_corr_x, ori_corr_y, ori_corr_z]).as_dcm()

        # robot_pose: manipulator frame -> Isaac world frame
        isaac_x = -msg.y
        isaac_y = -msg.x
        theta_rad = np.radians(msg.theta) + h_bias

        c, s = np.cos(theta_rad), np.sin(theta_rad)
        world_off_x = c * body_off_x - s * body_off_y
        world_off_y = s * body_off_x + c * body_off_y

        arm_world = np.array([isaac_x + world_off_x,
                              isaac_y + world_off_y,
                              arm_base_z])

        R_aw = (R.from_euler('z', theta_rad)
                * R.from_euler('y', tilt_y)
                * R.from_euler('x', tilt_x)).as_dcm()

        t_aw = -R_aw @ arm_world
        T_aw = self.create_homogeneous_transform(R_aw, t_aw)
        R_aw_rot = R.from_dcm(R_aw)

        for g in goals:
            p_world = np.array([g['x'], g['y'], g['z'], 1.0])
            p_arm = (T_aw @ p_world)[:3]

            r_csv = R.from_euler('zyx', [g['rx'], g['ry'], g['rz']])
            r_arm = R_aw_rot * R.from_dcm(R_corr @ r_csv.as_dcm())

            rospy.loginfo(
                f"[Arm] Transform: world={np.round(p_world[:3], 3)} "
                f"-> arm={np.round(p_arm, 4)}, dist={np.linalg.norm(p_arm):.3f}m"
            )

            transformed.append(
                (p_arm, r_arm.as_quat(), g['speed'])
            )

        return transformed

    # --------------------------------------------------
    # DONE
    # --------------------------------------------------
    def publish_done(self):
        msg = Bool()
        msg.data = True
        self.done_pub.publish(msg)
        rospy.loginfo("[Arm] scan_finished published")

    def is_busy(self):
        return self.busy
    
    def cancel(self):
        """scan STOP"""
        rospy.logwarn("[Arm] CANCEL requested")
        self.cancel_requested = True
    
    def shutdown(self):
        """Shut down system"""
        rospy.loginfo("[Arm] Shutting down...")
        self.camera.stop_grabbing()
        self.camera.close()


if __name__ == '__main__':
    controller = ArmControllerWithScan()
    
    # Register shutdown callback
    rospy.on_shutdown(controller.shutdown)
    
    rospy.spin()