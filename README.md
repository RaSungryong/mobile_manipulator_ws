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
- **Real robot control** via Fairino SDK (simulation support removed)

### Technology Stack

| Category | Technology |
|----------|-----------|
| ROS | Noetic (Ubuntu 20.04) |
| Languages | Python 3.8, C++17 (GCC 9.4) |
| Build | Catkin (CMake 3.16) |
| Arm Control | Fairino SDK (XML-RPC/TCP) |
| Offline Validation | roboticstoolbox-python (URDF FK, dev tools only) |
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
├── readme.txt                      # Run instructions (Korean)
├── src/
│   ├── apriltag_nav/               # Core navigation & task execution package
│   │   ├── scripts/                # ROS node entry points (one per device)
│   │   │   ├── task_executor.py        # Orchestrator: state machine, STATUS lamp, e-stop
│   │   │   ├── arm_controller_node.py  # Fairino arm node (wraps the v2 controller)
│   │   │   ├── basler_camera_node.py   # Wrist Basler + VISION lamp (capture service)
│   │   │   └── keyence_dlen1_node.py   # Keyence distance sensor node
│   │   ├── src/apriltag_nav/       # Importable python package (library code)
│   │   │   ├── paths.py                # Single source of truth for on-disk paths
│   │   │   ├── robot_controller.py     # Mobile base navigation (Pure Pursuit)
│   │   │   ├── arm_controller.py # Arm controller (default, 4-DOF calibrated)
│   │   │   ├── arm_client.py           # task_executor's ROS client for the arm node
│   │   │   ├── navifra_devices.py      # Navifra base peripherals (lift/LED/BMS/safety)
│   │   │   ├── map_manager.py          # AprilTag topological map + BFS
│   │   │   ├── task_manager.py         # CSV task loader + q0 seed support
│   │   │   ├── camera_interface.py     # Basler (PyPylon) device wrapper
│   │   │   ├── inference_interface.py  # ONNX Runtime inference wrapper
│   │   │   └── utils.py               # Config loading utilities
│   │   ├── tools/                  # Standalone one-off scripts (not installed)
│   │   │   ├── calibrate_transform.py / collect_calib_data.py  # transform calibration
│   │   │   ├── validate_transform.py / validate_compare.py     # offline FK validation
│   │   │   ├── ra_map_plotter.py / set_tool_tcp.py / test_hardware.py
│   │   │   └── navigate.py / send_debug_cmd.py / FrCmd.py / arm_controller_sdk.py
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
│       │       ├── fr10v6_vision.urdf      # FR10v6 + vision link (offline FK validation)
│       │       └── ...
│       ├── frcobot_hw/             # C++ hardware status node
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
- `theta` — heading in **world frame** (degrees)
- `id` — tag ID

**Coordinate conversion in `calculate_robot_pose()`:**
1. World-frame camera position from detected tag
2. `world_to_manipulator`: `manip_x = -world_y`, `manip_y = -world_x`
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
- `pose` mode: Cartesian pose (x, y, z, rx, ry, rz) in the world frame

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

### 4.1 Real Robot Controllers (Fairino SDK)

**`arm_controller.py`** — the real robot controller (sole variant;
the original `arm_controllerrealwithscan.py` was removed — it kept the broken
sign-flip transform and opened the Basler directly, violating the camera policy):
- **`_transform_pose`**: 4-DOF physical transform (position output in mm for Fairino)
- **`_exec_pose`**: uses `GetInverseKinRef(0, target, q0_deg)` when q0 available
- Keyence closed-loop distance adjustment; frames via /camera/capture service

### 4.2 Fairino SDK API Reference (Key Methods)

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

### 4.3 Safety & Validation

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

**Result tracking (withscan only):**
Execution results are saved incrementally to `{original_csv}_result.csv` with columns:
- `success` — True/False
- `execution_message` — status description
- `validated_at` — timestamp

### 4.4 Public API (Common Interface)

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
| World | CSV pose files | m, rad | Origin at the polishing cell |
| Manipulator | `/robot_pose` msg | m, deg | Rotated: `manip_x = -world_y`, `manip_y = -world_x` |
| Arm base_link (Fairino) | IK input | mm, deg | URDF root frame, Fairino units |

### 5.2 Transform Pipeline (process_transforms)

```
CSV World Pose (x, y, z, rx, ry, rz)     [meters, radians, ZYX intrinsic]
         │
         │  1. robot_pose (manip) → world:
         │     x_base = -msg.y,  y_base = -msg.x,  θ = radians(msg.theta)
         │
         │  2. Arm base origin in world:
         │     p_A_W = (x_base + Rz(θ)·body_off).x
         │             (x_base + Rz(θ)·body_off).y
         │             body_off_z
         │
         │  3. R_WA = Rz(θ + mount_yaw) · Ry(tilt_y) · Rx(tilt_x)
         │     R_AW = R_WAᵀ                 # world → arm frame
         │
         │  4. Position:    p_A = R_AW · (p_W − p_A_W)
         │     Orientation: r_A = R_AW · R_zyx(rx, ry, rz)         (CSV value used directly)
         ▼
Arm base_link Pose (p_A, r_A)            [mm for Fairino]
```

### 5.3 Physical Model (4-DOF)

Parameters derived from the USD scene geometry (`polishing_env_10255.usd`)
and refined by SVD Procrustes fit on 328 paired joint/pose CSV rows. The
**old 9-DOF calibration was overfit** — it absorbed a missing
`mount_yaw = π` into `body_off_x/y`, `tilt_*`, and `ori_corr_*`, yielding
355 mm mean residual on re-evaluation. The physical 4-DOF model below
scores 12 mm mean residual with fewer parameters and direct physical
interpretation.

**Parameters** (all overridable via ROS `~` private params):

| Parameter | ROS Param | Value | Source | Meaning |
|---|---|---|---|---|
| Body offset X | `~arm_body_offset_x` | 0.0 m | USD | Arm mount x in body frame |
| Body offset Y | `~arm_body_offset_y` | 0.0 m | USD | Arm mount y in body frame |
| Base Z | `~arm_base_z` | 1.0076 m | fit | Arm base height above ground |
| Mount yaw | `~arm_mount_yaw` | π | USD | Arm base yaw vs body (180°) |
| Tilt X | `~arm_tilt_x` | -0.02248 rad | fit | Small mount roll (-1.29°) |
| Tilt Y | `~arm_tilt_y` | 0.02639 rad | fit | Small mount pitch (+1.51°) |

**Accuracy** (4-DOF model, 328 paired rows across groups 105/106):

| Metric | Value |
|---|---|
| Position mean residual | **12 mm** |
| Position max residual | 27 mm |
| IK success | 100% |
| Parameters | 4 (was 9) |

Sub-mm per-group fit is achievable (Procrustes per-group gives 0.7 mm RMS,
4.5 mm max) but requires group-specific rotation — the single-param model
trades that for physical interpretability.

### 5.4 Pose Control Orientation

`_execute_pose_goal` uses **CSV-derived orientation** (via
`process_transforms`) as the IK target, not `FK(q0).R`. Reason: end-effector
orientation barely changes across a scan pattern, so the CSV value is the
cleanest source. `q0` (paired joint CSV) is used only as the IK seed.

Real robot (`arm_controller.py`) also feeds `q0` into
`GetInverseKinRef` — there the joint CSV is a **reference only** since
real-world joint→pose mapping can diverge from the URDF FK.

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
106,1,1.1766,2.5154,0.5712,1.5741,-0.0891,3.1049,30,
```

Fields: position in **meters**, orientation as **ZYX intrinsic euler in radians**.
`speed` is scan-motion speed factor (0–100; current datasets use 30).

### 6.3 Registered Tasks

`task_manager.TASK_DEFS` — `file` (single) / `files` (list) forms are both accepted:

| Task | Mode | Inputs | Output |
|---|---|---|---|
| `scan_joints_line1` | joint | `optimized_joints_line1.csv` | `..._result.csv` |
| `scan_joints_line2` | joint | `optimized_joints_line2.csv` | `..._result.csv` |
| `scan_grid_line1` | pose | `grid_path_line1.csv` + paired joint CSV (q0) | `..._result.csv` |
| `scan_grid_line2` | pose | `grid_path_line2.csv` + paired joint CSV (q0) | `..._result.csv` |
| `scan_joints_line1_new` | joint | `optimized_joints_line1_new.csv` | `..._result.csv` |
| **`scan_full_joints`** | joint | line1+line2 joint CSVs (+pose CSVs for xyz) | **`scan_full_joints_ra_map.csv`** |
| **`scan_full_pose`** | pose | line1+line2 pose CSVs (+joint CSVs for q0) | **`scan_full_pose_ra_map.csv`** |
| `go_home` | move | — | — |

### 6.4 Ra Map Output (merged tasks)

Merged scan tasks persist Ra statistics to a single 13-column CSV:

```
group_id, point_id, x, y, z,
ra_mean, ra_std, ra_min, ra_max, num_samples,
success, execution_message, validated_at
```

- Written incrementally (one append per scan point) — partial results
  survive `STOP` / cancel.
- Joint-mode `x, y, z` come from the paired pose CSV via
  `task_manager`'s `pose_files` key.
- Visualize with `scripts/ra_map_plotter.py <csv> [--interpolate] [--metric ra_std]`
  → PNG alongside the CSV.

### 6.5 Task Execution Flow

```
1. TASK scan_full_pose
2. task_manager concatenates grid_path_line1.csv + grid_path_line2.csv
   (q0 IK seeds loaded from optimized_joints_line{1,2}.csv)
3. For each group_id in sorted order (105 → 106 → 117 → 118):
   a. Navigate mobile base to tag (robot_controller)
   b. robot_controller publishes /robot_pose
   c. arm_controller builds goals via process_transforms (4-DOF)
   d. For each scan point:
      - IK solve (seeded with paired q0)
      - Move arm to target (orientation from CSV quat)
      - Capture images (num_samples) → ONNX ResNet3D → Ra scalar
      - Append row to scan_full_pose_ra_map.csv
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
float64 theta       # heading (world frame, degrees)
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
├── ArmController     → Fairino SDK IK
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

### Commands

```bash
# Per-line tasks
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_grid_line1"

# Merged full-line scan → unified Ra map CSV
rostopic pub -1 /task_command std_msgs/String "TASK scan_full_pose"
rostopic pub -1 /task_command std_msgs/String "TASK scan_full_joints"

# Navigation / pose test / debug
rostopic pub -1 /task_command std_msgs/String "GOTO 108"
rostopic pub -1 /task_command std_msgs/String "TEST_POSE 0.737 2.14 0.704"
rosrun apriltag_nav send_debug_cmd.py

# Render Ra map heatmap from a completed scan
python3 src/apriltag_nav/scripts/ra_map_plotter.py \
        src/apriltag_nav/task/csv/scan_full_pose_ra_map.csv --interpolate
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

See [Section 5.3 Physical Model (4-DOF)](#53-physical-model-4-dof) for the full list
(`~arm_body_offset_x/y`, `~arm_base_z`, `~arm_mount_yaw`, `~arm_tilt_x/y`).

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
