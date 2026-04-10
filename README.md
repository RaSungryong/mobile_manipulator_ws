# Mobile Manipulator Workspace — Detailed Documentation

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Directory Structure](#2-directory-structure)
3. [Core Modules](#3-core-modules)
4. [Arm Controller Architecture](#4-arm-controller-architecture)
5. [Coordinate Transform System](#5-coordinate-transform-system)
6. [Task System](#6-task-system)
7. [ROS Interface](#7-ros-interface)
8. [Custom Messages](#8-custom-messages)
9. [System Architecture](#9-system-architecture)
10. [Hardware Interface](#10-hardware-interface)
11. [Build & Run](#11-build--run)
12. [Configuration Reference](#12-configuration-reference)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. Project Overview

A mobile manipulator system combining a mobile base with a Fairino FR10v6 6-DOF collaborative robot arm. The system provides:

- **Autonomous navigation** based on AprilTag visual markers (Pure Pursuit algorithm)
- **Arm scanning tasks** at designated positions
- **Surface roughness (Ra) prediction** using ResNet3D + ONNX Runtime
- **Dual mode support**: real robot (Fairino SDK) and simulation (Isaac Sim + roboticstoolbox)

### Technology Stack

| Category | Technology |
|----------|-----------|
| ROS | Noetic (Ubuntu 20.04) |
| Languages | Python 3.8, C++17 (GCC 9.4) |
| Build | Catkin (CMake 3.16) |
| Arm Control (Sim) | roboticstoolbox-python (URDF FK/IK) |
| Arm Control (Real) | Fairino SDK (XML-RPC/TCP) |
| Hardware Interface | ros_control, frrobot_hw_interface |
| Visual Localization | dt_apriltags (tag36h11, 60mm) |
| Camera | Basler (PyPylon) / USB (OpenCV) |
| Distance Sensor | Keyence DL-EN1 (TCP, port 64000) |
| ML Inference | ONNX Runtime (CPU), ResNet3D |
| Transforms | scipy, numpy, spatialmath-python |

---

## 2. Directory Structure

```
mobile_manipulator_ws/
├── CLAUDE.md                       # Quick reference for Claude Code
├── docs.md                         # This file (detailed documentation)
├── modification_report_kr.md       # Modification report (Korean)
├── readme.txt                      # Isaac Sim setup instructions
├── polishing_env/                  # Isaac Sim USD scene files
├── src/
│   ├── apriltag_nav/               # Core navigation & task execution package
│   │   ├── scripts/                # Python nodes (main business logic)
│   │   │   ├── task_executor.py        # System entry point, state machine
│   │   │   ├── robot_controller.py     # Mobile base navigation (Pure Pursuit)
│   │   │   ├── arm_controller.py       # Arm control - sim, rtb (no scan)
│   │   │   ├── arm_controllerwithscan.py       # Arm control - sim, rtb + scan
│   │   │   ├── arm_controllerrealwithscan.py   # Arm control - real, Fairino (original)
│   │   │   ├── arm_controllerrealwithscan_v2.py # Arm control - real, Fairino (v2, calibrated)
│   │   │   ├── arm_controller_sdk.py   # Arm control - real, Fairino (basic, no scan)
│   │   │   ├── map_manager.py          # AprilTag topological map + BFS
│   │   │   ├── task_manager.py         # CSV task loader + q0 seed support
│   │   │   ├── keyence_dlen1_node.py   # Keyence distance sensor node
│   │   │   ├── camera_interface.py     # Basler/Webcam camera abstraction
│   │   │   ├── inference_interface.py  # ONNX Runtime inference wrapper
│   │   │   ├── send_debug_cmd.py       # Debug command sender
│   │   │   └── utils.py               # Config loading utilities
│   │   ├── config/
│   │   │   ├── robot.yaml              # Robot parameters
│   │   │   └── map.yaml               # AprilTag topological map
│   │   ├── task/csv/                   # Scan task CSV definitions
│   │   │   ├── grid_path_line1.csv         # Pose mode scan (line 1, groups 105/106)
│   │   │   ├── grid_path_line2.csv         # Pose mode scan (line 2, groups 117/118)
│   │   │   ├── optimized_joints_line1.csv  # Joint mode scan (line 1, paired with grid_path)
│   │   │   └── optimized_joints_line2.csv  # Joint mode scan (line 2, paired with grid_path)
│   │   ├── model/                  # ML model weights (.onnx/.pt, gitignored)
│   │   └── launch/                 # ROS launch files
│   ├── robot_msgs/                 # Custom ROS message definitions
│   ├── fairino_sdk/                # Fairino robot SDK wrapper
│   └── frcobot_ros/                # Arm-related package collection
│       ├── frcobot_description/    # URDF/Xacro robot models
│       │   └── urdf/
│       │       ├── fr10v6.urdf             # Standard FR10v6
│       │       ├── fr10v6_vision.urdf      # FR10v6 + vision link (used by rtb)
│       │       └── ...
│       ├── frcobot_hw/             # C++ hardware status node
│       ├── fr10v6_moveit_config/   # MoveIt config (standard)
│       ├── fr10v6_vision_moveit_config/        # MoveIt config (vision)
│       ├── fr10v6_vision_251219_moveit_config/ # MoveIt config (latest vision)
│       └── ros_control_boilerplate/frrobot_control/ # ros_control HW interface (C++)
├── build/                          # Build artifacts (gitignored)
└── devel/                          # Catkin devel space (gitignored)
```

---

## 3. Core Modules

### 3.1 task_executor.py — Task Coordinator

**Path:** `src/apriltag_nav/scripts/task_executor.py`

System entry point. Maintains a state machine (IDLE / MOVING / ARRIVED / SCANNING / SCAN_DONE / ERROR) and coordinates sequential execution of mobile base navigation and arm scanning.

**Supported commands** (via `/task_command` topic):

| Command | Description |
|---------|-------------|
| `TASK <name>` | Execute predefined task (e.g., `scan_joints_line1`) |
| `GOTO <tag_id>` | Navigate to specific AprilTag |
| `TEST_POSE x y z [rx ry rz]` | Test pose control at given world coordinates |
| `STOP` | Emergency stop |
| `STATE` | Query current state |
| `EXEC <python>` | Debug: execute arbitrary Python code |
| `EVAL <expr>` | Debug: evaluate Python expression |

### 3.2 robot_controller.py — Mobile Base Navigation

**Path:** `src/apriltag_nav/scripts/robot_controller.py`

AprilTag-based visual servoing with Pure Pursuit algorithm and S-curve velocity profiles.

**Key parameters** (`config/robot.yaml`):
- Max linear speed: 0.05 m/s
- Max angular speed: 0.25 rad/s
- Navigation timeout: 8 seconds

**`/robot_pose` output:** Published once after arriving at each tag. Contains:
- `x, y` — mobile base center in **manipulator frame** (meters)
- `theta` — heading in **Isaac world frame** (degrees)
- `id` — tag ID

**Coordinate conversion in `calculate_robot_pose()`:**
1. Isaac Sim camera position from detected tag
2. `isaac_to_manipulator`: `manip_x = -isaac_y`, `manip_y = -isaac_x`
3. Camera-to-robot-center offset applied per zone

### 3.3 map_manager.py — Map & Path Planning

**Path:** `src/apriltag_nav/scripts/map_manager.py`

Manages the AprilTag topological map (`config/map.yaml`) with BFS shortest path planning.

- 151 AprilTags (IDs: 100–147, 400–415, 500–508)
- Tag types: DOCK, PIVOT, MOVE, WORK
- Work zones: A (entrance corridor), B/D (left side), C/E (right side)

### 3.4 task_manager.py — Task Loader

**Path:** `src/apriltag_nav/scripts/task_manager.py`

Loads CSV scan tasks from `task/csv/` directory. Supports runtime GOTO task generation.

**Scan point modes:**
- `joint` mode: direct joint angles (radians)
- `pose` mode: Cartesian pose (x, y, z, rx, ry, rz) in Isaac Sim world frame

**Joint CSV q0 seed:** For pose tasks with a `joint_file` defined, the corresponding joint values are loaded and attached to each scan point as `q0` for IK initialization.

**Task definitions:**

**Task definitions:**

| Task Name | CSV File | Mode | Joint File |
|-----------|----------|------|------------|
| `scan_joints_line1` | `optimized_joints_line1.csv` | joint | — |
| `scan_joints_line2` | `optimized_joints_line2.csv` | joint | — |
| `scan_joints_line1_new` | `optimized_joints_line1_new.csv` | joint | — |
| `scan_grid_line1` | `grid_path_line1.csv` | pose | `optimized_joints_line1.csv` |
| `scan_grid_line2` | `grid_path_line2.csv` | pose | `optimized_joints_line2.csv` |

**Scan point metadata fields** (all modes):
- `point_id` — point index from CSV
- `group_id` — tag group from CSV
- `csv_path` — source CSV file path (for result saving)
- `is_discontinuous` — if 1, arm returns to Home before moving to this point

### 3.5 keyence_dlen1_node.py — Distance Sensor

**Path:** `src/apriltag_nav/scripts/keyence_dlen1_node.py`

TCP polling of Keyence DL-EN1 sensor at 60 Hz. Default address: 192.168.1.5:64000.

---

## 4. Arm Controller Architecture

### 4.1 Simulation Controllers (roboticstoolbox)

**`arm_controller.py`** — Basic sim controller (no scanning):
- Loads URDF: `fr10v6_vision.urdf` via `rtb.ERobot.URDF()`
- FK/IK: `robot.fkine()`, `robot.ikine_LM()`
- Trajectory: `jtraj()` 5th-order polynomial interpolation
- Publishes `/joint_states` at 50 Hz
- Pose goals use calibrated 9-DOF `process_transforms`
- Supports `q0` IK seed from paired joint CSV
- Joint/velocity limit validation (`JOINT_LIMITS`, `VELOCITY_LIMITS`)
- `is_discontinuous` support (Home before discontinuous points)
- Returns `(bool, str)` from motion methods for success/failure tracking

**`arm_controllerwithscan.py`** — Sim controller with camera + inference:
- Same rtb-based control as above
- Adds: Basler/Webcam camera capture, ONNX Ra inference at each scan point
- Publishes `/scan/ra_value`, `/scan/point_result`, `/scan/image`
- Incremental result saving to `{csv_name}_result.csv` via pandas
- Isaac Sim collision detection flag

### 4.2 Real Robot Controllers (Fairino SDK)

**`arm_controllerrealwithscan.py`** — Original real robot controller:
- Fairino SDK: `Robot.RPC(ip)`, `MoveJ()`, `GetInverseKin()`
- Old `_transform_pose`: simple delta with sign flips (broken for Isaac Sim coords)
- Keyence closed-loop distance adjustment
- Camera capture + ONNX inference

**`arm_controllerrealwithscan_v2.py`** — Updated real robot controller:
- **`_transform_pose`**: calibrated 9-DOF transform (position output in mm for Fairino)
- **`_exec_pose`**: uses `GetInverseKinRef(0, target, q0_deg)` when q0 available
- All other functionality unchanged (Keyence, camera, inference)

### 4.3 Fairino SDK API Reference (Key Methods)

| Method | Parameters | Description |
|--------|-----------|-------------|
| `GetInverseKin(type, desc_pos, config=-1)` | type: 0=abs, desc_pos: [x,y,z,rx,ry,rz] mm/deg | IK, references current joint pos |
| `GetInverseKinRef(type, desc_pos, joint_pos_ref)` | joint_pos_ref: [j1..j6] deg | IK with specified joint reference |
| `GetInverseKinHasSolution(type, desc_pos, joint_pos_ref)` | same as above | Check if IK has solution |
| `MoveJ(joints, tool=0, user=0)` | joints: [j1..j6] deg | Joint space motion |
| `MoveL(pose, tool=0, user=0)` | pose: [x,y,z,rx,ry,rz] mm/deg | Cartesian linear motion |
| `GetActualTCPPose()` | — | Get current TCP pose |
| `SetSpeed(speed)` | speed: int | Set motion speed |
| `StopMotion()` | — | Emergency stop |

### 4.4 Safety & Validation

**Joint Limits (FR10v6):**

| Joint | Min (rad) | Max (rad) |
|-------|-----------|-----------|
| J1 | -3.0543 | 3.0543 |
| J2 | -4.6251 | 1.4835 |
| J3 | -2.8274 | 2.8274 |
| J4 | -4.6251 | 1.4835 |
| J5 | -3.0543 | 3.0543 |
| J6 | -3.0543 | 3.0543 |

**Velocity Limits:** `[3.15, 3.15, 3.15, 3.2, 3.2, 3.2]` rad/s

**Validation methods:**
- `validate_joint_values(joints)` — checks all joints within limits before execution
- `validate_velocities(current, target, time_delta)` — checks velocity constraints

**Discontinuity handling:**
When `is_discontinuous == 1` for a scan point, the arm returns to Home position before moving to that point. This prevents unsafe direct paths between distant configurations.

**Motion return values:**
`_execute_pose_goal()` and `_execute_joint_goal()` return `(bool, str)`:
- `(True, "Success")` — motion completed
- `(False, "IK failed")` — inverse kinematics failed
- `(False, "Joint limit violation: ...")` — target exceeds limits
- `(False, "Isaac Sim Collision Detected")` — collision during execution

**Result tracking (withscan only):**
Execution results are saved incrementally to `{original_csv}_result.csv` with columns:
- `success` — True/False
- `execution_message` — status description
- `validated_at` — timestamp

### 4.5 Public API (Common Interface)

All arm controller variants expose the same API consumed by `task_executor.py`:

```python
class ArmController:
    def __init__(self, model_path=None)
    def execute_scan_points(self, scan_points: list)
    def move_to_home(self)
    def is_busy(self) -> bool
    def cancel(self)
    # Publishes: /scan_finished (Bool)
```

---

## 5. Coordinate Transform System

### 5.1 Frame Definitions

| Frame | Used By | Units | Notes |
|-------|---------|-------|-------|
| Isaac Sim World | CSV pose files | m, rad | Origin at sim world origin |
| Manipulator | `/robot_pose` msg | m, deg | Rotated: `manip_x = -isaac_y`, `manip_y = -isaac_x` |
| Arm base_link (rtb) | IK input (sim) | m, rad | URDF root frame |
| Arm base_link (Fairino) | IK input (real) | mm, deg | Same frame, different units |

### 5.2 Transform Pipeline (process_transforms)

```
CSV World Pose (x, y, z, rx, ry, rz)     [meters, radians, ZYX euler]
         │
         │  1. robot_pose: manip → Isaac world
         │     isaac_x = -msg.y, isaac_y = -msg.x
         │
         │  2. Arm base in world:
         │     body_offset rotated by Rz(theta + heading_bias)
         │     arm_world = isaac_pos + rotated_offset + [0, 0, arm_base_z]
         │
         │  3. R_aw = Rz(theta) · Ry(tilt_y) · Rx(tilt_x)
         │     T_aw = [R_aw | -R_aw @ arm_world]
         │
         │  4. Position: p_arm = T_aw @ p_world
         │     Orientation: r_arm = R_aw · R_corr · R_csv_zyx
         ▼
Arm base_link Pose (p_arm, r_arm)         [meters for rtb, mm for Fairino]
```

### 5.3 Calibration

Calibrated from 655 paired data points across 4 tag groups:
- **Zone B** (tags 105, 106): heading ≈ +90°
- **Zone C** (tags 117, 118): heading ≈ -90°

**9-DOF Parameters:**

| Parameter | ROS Param | Value | Description |
|-----------|-----------|-------|-------------|
| Body offset X | `~arm_body_offset_x` | -0.166715 m | Arm mount offset in body frame |
| Body offset Y | `~arm_body_offset_y` | -0.254772 m | Arm mount offset in body frame |
| Base Z | `~arm_base_z` | 0.974167 m | Arm base height in world |
| Tilt X | `~arm_tilt_x` | 0.054898 rad | R_aw X-axis tilt (3.15°) |
| Tilt Y | `~arm_tilt_y` | 0.017894 rad | R_aw Y-axis tilt (1.03°) |
| Heading bias | `~arm_heading_bias` | 0.014368 rad | Heading offset (0.82°) |
| Ori corr X | `~arm_ori_corr_x` | -0.095520 rad | Orientation correction (-5.47°) |
| Ori corr Y | `~arm_ori_corr_y` | -0.052944 rad | Orientation correction (-3.03°) |
| Ori corr Z | `~arm_ori_corr_z` | -0.008688 rad | Orientation correction (-0.50°) |

**Accuracy:**

| Group | Zone | Pos Mean | Pos Max | Ori Mean | IK |
|-------|------|----------|---------|----------|-----|
| 105 | B | 20.9 mm | 39.1 mm | 4.62° | 100% |
| 106 | B | 17.5 mm | 27.6 mm | 5.10° | 100% |
| 117 | C | 8.6 mm | 14.2 mm | 5.63° | 100% |
| 118 | C | 14.2 mm | 21.5 mm | 4.10° | 100% |

---

## 6. Task System

### 6.1 CSV Format — Joint Mode

```csv
group_id,point_id,q1,q2,q3,q4,q5,q6,collision_detected
106,1,-1.5808,-0.0847,0.6084,-2.0974,-1.6656,-0.0101,FALSE
```

Fields: 6 joint angles in **radians**.

### 6.2 CSV Format — Pose Mode

```csv
group_id,point_id,x,y,z,rx,ry,rz,speed,comment
106,1,1.1766,2.5154,0.5712,1.5741,-0.0891,3.1049,80,
```

Fields: position in **meters**, orientation as **ZYX intrinsic euler in radians**.

### 6.3 Task Execution Flow

```
1. TASK scan_grid_line1
2. task_manager loads grid_path_line1.csv (pose) + optimized_joints_line1.csv (q0)
3. For each group_id (= tag_id):
   a. Navigate mobile base to tag (robot_controller)
   b. robot_controller publishes /robot_pose
   c. arm_controller builds goals via process_transforms
   d. For each scan point:
      - IK solve (with q0 seed if available)
      - Move arm to target
      - [If scan enabled] Capture image → ONNX inference → publish Ra
4. Return arm to home position
5. Publish /scan_finished
```

---

## 7. ROS Interface

### 7.1 Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/rgb` | sensor_msgs/Image | Camera → Nav | AprilTag image input |
| `/camera_info` | sensor_msgs/CameraInfo | Camera → Nav | Camera intrinsics |
| `/odom` | nav_msgs/Odometry | Base → Nav | Odometry feedback |
| `/cmd_vel` | geometry_msgs/Twist | Nav → Base | Base velocity command |
| `/robot_pose` | robot_msgs/Pose2DWithFlag | Nav → Arm | Mobile base pose |
| `/task_command` | std_msgs/String | External → Executor | Task commands |
| `/scan_finished` | std_msgs/Bool | Arm → Executor | Scan completion signal |
| `/joint_states` | sensor_msgs/JointState | Arm → RViz | Joint positions (sim) |
| `keyence/value` | std_msgs/Float32 | Sensor → Arm | Distance measurement |
| `keyence/raw` | std_msgs/Int32 | Sensor → Debug | Raw sensor value |
| `/scan/ra_value` | std_msgs/Float32 | Arm → External | Ra roughness value |
| `/scan/point_result` | std_msgs/String | Arm → External | JSON scan result |
| `/scan/image` | sensor_msgs/Image | Arm → External | Captured image |

---

## 8. Custom Messages

### robot_msgs/Pose2DWithFlag

```
std_msgs/Header header
float64 x           # position X (manipulator frame, meters)
float64 y           # position Y (manipulator frame, meters)
float64 theta       # heading (Isaac world frame, degrees)
float64 theta_web   # heading for web display
bool flag           # validity flag
int32 id            # tag ID
```

### robot_msgs/NavDebugStatus

Navigation debug status (zone, offset, progress, etc.).

### frcobot_hw/status

Robot hardware status (joint positions, torques, IO, etc.).

---

## 9. System Architecture

### Plan A (Current) — Single-Process Architecture

`task_executor.py` imports `ArmController` directly. All control logic runs in one ROS node process.

```
[mobile_manipulator_system process]
├── RobotController   → subscribes /rgb, publishes /cmd_vel, /robot_pose
├── ArmController     → rtb IK (sim) or Fairino SDK (real)
│                     → subscribes keyence/value, /robot_pose
│                     → publishes /scan_finished, /scan/ra_value
├── TaskManager       → loads CSV tasks
└── MapManager        → BFS path planning
```

### Plan B (Planned) — Distributed ROS Nodes

Separate hardware access into independent nodes communicating only via ROS topics/services. See CLAUDE.md history for architecture details.

---

## 10. Hardware Interface

### frrobot_hw_interface.cpp

**Path:** `src/frcobot_ros/ros_control_boilerplate/frrobot_control/src/frrobot_hw_interface.cpp`

ROS Control hardware interface communicating via TCP (192.168.58.2:8080).

- **Write:** sends `ServoJ` joint commands (radians → degrees)
- **Read:** reads `GetActualJointPosRadian` current positions
- **Rate:** 125 Hz

---

## 11. Build & Run

### Build

```bash
cd /home/lcl/mobile_manipulator_ws
catkin_make
source devel/setup.bash

# Build specific package
catkin_make --only-pkg-with-deps apriltag_nav
```

### Real Robot

```bash
# Terminal 1
roscore

# Terminal 2
source devel/setup.bash
rosrun apriltag_nav task_executor.py

# Terminal 3 (optional)
rosrun apriltag_nav keyence_dlen1_node.py
```

### Isaac Sim

```bash
# After starting Isaac Sim (see readme.txt)
roslaunch fr10v6_vision_251219_moveit_config fr10v6_vision_isaac_execution.launch
rosrun apriltag_nav task_executor.py
```

### Commands

```bash
# Predefined task
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"

# Navigation
rostopic pub -1 /task_command std_msgs/String "GOTO 108"

# Pose test
rostopic pub -1 /task_command std_msgs/String "TEST_POSE 0.737 2.14 0.704"

# Debug
rosrun apriltag_nav send_debug_cmd.py
```

---

## 12. Configuration Reference

### config/robot.yaml

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_speed` | 0.05 m/s | Max base linear velocity |
| `max_angular_speed` | 0.25 rad/s | Max base angular velocity |
| `camera_offset` | 0.45 m | Camera to robot center distance |
| `tag_size` | 0.06 m | AprilTag physical size |
| `nav_timeout` | 8 s | Navigation timeout |

### Arm Transform Params (ROS `~` private)

See [Section 5.3 Calibration](#53-calibration) for the full list.

### Network Addresses

| Device | IP | Port |
|--------|-----|------|
| Fairino arm controller | 192.168.58.2 | 8080 |
| Keyence DL-EN1 sensor | 192.168.1.5 | 64000 |

---

## 13. Troubleshooting

### Robot IP connection failure
Check `robot.yaml` IP addresses and network connectivity.

### Camera fails / PyPylon error
Node auto-falls back to USB camera. Force USB with `use_webcam:=true`.

### ONNX model inference error
Confirm model files exist (gitignored, manage separately):
```
src/apriltag_nav/model/exported/resnet3D.onnx
src/apriltag_nav/model/exported/resnet3D_gray.onnx
```

### AprilTag detection unstable
Verify `tag_size` matches physical markers. Check `/camera_info` topic.

### Navigation timeout (8s)
Increase `nav_timeout` in `robot.yaml`. System enters ERROR state on timeout.

### catkin_make can't find robot_msgs
Build dependencies first:
```bash
catkin_make --only-pkg-with-deps robot_msgs
catkin_make
```

### IK failure in pose mode
Check log output for `dist=` value — if > 1.5m, the transform is producing out-of-workspace targets. Verify `/robot_pose` values and calibration parameters.

### scipy as_matrix() error
This codebase uses `as_dcm()` / `from_dcm()` for scipy < 1.4 compatibility. Do not use `as_matrix()` / `from_matrix()`.
