# Mobile Manipulator Workspace

## Overview

Mobile manipulator system: Fairino FR10v6 6-DOF arm on a mobile base.
AprilTag visual navigation, arm scanning tasks, surface roughness (Ra) prediction.
Supports real robot and Isaac Sim simulation.

## Tech Stack

- **ROS Noetic** / Ubuntu 20.04 / Python 3.8 / C++17
- **Arm control (sim):** roboticstoolbox-python (URDF FK/IK + jtraj)
- **Arm control (real):** Fairino SDK (GetInverseKin / GetInverseKinRef / MoveJ)
- **Navigation:** dt_apriltags, Pure Pursuit, S-curve velocity
- **Camera:** Basler (PyPylon) → USB fallback (OpenCV)
- **Sensor:** Keyence DL-EN1 (TCP)
- **Inference:** ONNX Runtime (CPU), ResNet3D
- **Transforms:** scipy, numpy, spatialmath-python

## Build & Run

```bash
cd /home/lcl/mobile_manipulator_ws
catkin_make && source devel/setup.bash
rosrun apriltag_nav task_executor.py
```

## Task Commands

```
TASK <name>                # scan_joints_line1, scan_joints_line1_new, scan_grid_line1, etc.
GOTO <tag_id>              # Navigate to AprilTag
TEST_POSE x y z [rx ry rz] # Test pose control (debug)
STOP / STATE               # Emergency stop / query state
EXEC <code> / EVAL <expr>  # Debug execution
```

## Arm Controller Variants

| File | Mode | IK Engine | Scan |
|------|------|-----------|------|
| `arm_controller.py` | Sim | rtb ikine_LM | No |
| `arm_controllerwithscan.py` | Sim | rtb ikine_LM | Yes (camera+ONNX) |
| `arm_controllerrealwithscan.py` | Real | Fairino SDK (original) | Yes |
| `arm_controllerrealwithscan_v2.py` | Real | Fairino SDK + q0 ref | Yes |

Switch in `task_executor.py` line ~14: `from arm_controller import ArmController`

## Coordinate Frames

```
Isaac Sim World (CSV poses)
  │  isaac_x = -manip_y, isaac_y = -manip_x
  ▼
Manipulator Frame (/robot_pose msg, theta in degrees)
  │  T_aw = calibrated 9-DOF transform
  ▼
Arm base_link (IK input: meters+rad for rtb, mm+deg for Fairino)
```

CSV orientation uses **ZYX** intrinsic euler (radians).

## Calibrated Transform Parameters

9 ROS params (`~` prefix), calibrated from 655 paired data points:

```yaml
arm_body_offset_x: -0.166715   # body-frame arm mount offset X (m)
arm_body_offset_y: -0.254772   # body-frame arm mount offset Y (m)
arm_base_z:         0.974167   # arm base height in world (m)
arm_tilt_x:         0.054898   # R_aw tilt correction X (rad)
arm_tilt_y:         0.017894   # R_aw tilt correction Y (rad)
arm_heading_bias:   0.014368   # heading bias correction (rad)
arm_ori_corr_x:    -0.095520   # orientation correction X (rad)
arm_ori_corr_y:    -0.052944   # orientation correction Y (rad)
arm_ori_corr_z:    -0.008688   # orientation correction Z (rad)
```

## Coding Conventions

- **Python:** PascalCase classes, snake_case methods, UPPER constants
- **Comments in English**
- **ROS topics:** lowercase with `/` separator, scan results under `/scan/`
- **Config:** all tunable params in `config/robot.yaml`, not hardcoded
- **scipy compat:** use `as_dcm()` / `from_dcm()` (not `as_matrix()` / `from_matrix()`) for scipy < 1.4
