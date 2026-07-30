# Mobile Manipulator Workspace

## Overview

Mobile manipulator system: Fairino FR10v6 6-DOF arm on a Navifra mobile base.
AprilTag visual navigation, arm scanning tasks, surface roughness (Ra) prediction.
Real robot only — simulation support has been removed.

Base driver: **Navifra KU Polishing Robot Driver v0.16** (separate ROS1 install
at `~/navifra`, systemd unit `navifra-robot`). It owns `/cmd_vel` + `/odom` plus
the lift / lighting / battery / Safety-PLC peripherals. Field tuning lives in
`~/navifra/param.yaml`, not in this workspace.

## Tech Stack

- **ROS Noetic** / Ubuntu 20.04 / Python 3.8 / C++17
- **Arm control:** Fairino SDK (GetInverseKin / GetInverseKinRef / MoveJ)
- **Navigation:** dt_apriltags, Pure Pursuit, S-curve velocity
- **Base peripherals:** `navifra_devices.py` → lift, VISION/STATUS LEDs,
  BMS battery, Safety PLC e-stop feedback, charge relay
- **Camera:** Basler only (PyPylon) — `camera_interface.py` has no webcam fallback,
  and imports `pypylon` unguarded, so it is a hard runtime dependency
- **Sensor:** Keyence DL-EN1 (TCP)
- **Inference:** ONNX Runtime (CPU), ResNet3D; preprocessing uses
  `torchvision.transforms` + PIL (also hard runtime deps)
- **Transforms:** scipy, numpy
- **Offline validation only:** `validate_transform.py` / `validate_compare.py`
  use roboticstoolbox-python for URDF FK; not a runtime dependency

## Build & Run

```bash
cd /home/lcl/mobile_manipulator_ws
catkin_make && source devel/setup.bash
rosrun apriltag_nav task_executor.py
```

## Task Commands

```
TASK <name>                # per-line: scan_joints_line{1,2}, scan_grid_line{1,2}
                           # merged: scan_full_joints, scan_full_pose  → *_ra_map.csv
                           # other: scan_joints_line1_new, go_home
GOTO <tag_id>              # Navigate to AprilTag
TEST_POSE x y z [rx ry rz] # Test pose control (debug)
STOP / STATE               # Emergency stop / query state
EXEC <code> / EVAL <expr>  # Debug execution
```

Merged-scan output is a 13-column CSV: `group_id, point_id, x, y, z,
ra_mean, ra_std, ra_min, ra_max, num_samples, success, execution_message,
validated_at`. Render with `tools/ra_map_plotter.py <csv> [--interpolate]`.

## Arm Controller Variants

| File | IK Engine | Scan |
|------|-----------|------|
| `src/apriltag_nav/arm_controller.py` | Fairino SDK + q0 ref | Yes (default) |
| `tools/arm_controller_sdk.py` | Fairino SDK (basic) | No |

Switch via the import in `scripts/arm_controller_node.py`:
`from apriltag_nav.arm_controller import ArmController`

## Architecture

Package layout (inside `src/apriltag_nav/`):

```
scripts/            ROS node entry points only — 4 files, one per device
src/apriltag_nav/   importable python package (installed by catkin_python_setup)
                    import as `from apriltag_nav.map_manager import MapManager`
tools/              standalone one-off scripts (calibration, validation, debug)
                    — not installed, run directly with a sourced workspace
```

**Never compute paths from `__file__`** — use `apriltag_nav.paths`
(`PKG_DIR`, `CONFIG_PATH`, `MAP_PATH`, `TASK_DIR`, `MODEL_PATH`,
`add_fairino_sdk_to_path()`). The old per-file `__file__` walks assumed
"this file lives in `<pkg>/scripts/`" and broke when files moved.

Hardware is split into ROS nodes following the Navifra driver's pattern — one
owner node per device, other nodes reach it over topics/services.

| Node | Owns | Interface |
|------|------|-----------|
| `task_executor.py` | orchestration, STATUS lamp, e-stop, battery | `/task_command` |
| `arm_controller_node.py` | Fairino FR10v6 arm | `/arm/scan_command`, `/arm/cancel`, `/arm/move_home` (srv) |
| `basler_camera_node.py` | wrist Basler **+ VISION lamp** | `/camera/capture` (srv) |
| `keyence_dlen1_node.py` | Keyence DL-EN1 | `keyence/value` |

`mobile_manipulator.launch` starts all four; each is required.

### Arm split

`arm_controller_node.py` **wraps** `arm_controller.ArmController`
unchanged rather than reimplementing it — that controller holds `TOOL_ID=1`
(vision_tip TCP), the q0 IK seed, the 4-DOF transform, the 13-column CSV and the
Keyence loop. A previous node-per-device attempt (the deleted `scripts_ros/`
tree) reimplemented the controller instead of wrapping it and silently lost
several of those — including using `tool=0` (flange) instead of `TOOL_ID=1`, so
its TCP offset was wrong. Wrap, don't rewrite. To switch controller
implementation, change the import in `arm_controller_node.py`.

`task_executor` talks to it through `ArmClient` (`src/apriltag_nav/arm_client.py`), which
mirrors the old in-process surface exactly (`current_pose_msg`,
`execute_scan_points`, `cancel`, `move_to_home`) so the call sites did not
change. Two things to know:

- `execute_scan_points()` is now **async** — it returns once published.
  Completion still arrives on `/scan_finished`, which `task_executor` already
  waits on. `move_to_home()` stays synchronous via the service.
- Scan points are JSON over `/arm/scan_command`. `ArmClient` coerces numpy
  scalars first — task points come from pandas, and raw `json.dumps` fails on
  `numpy.float64`.

⚠️ **All of ArmController's `~params` now live in `arm_controller_node`'s private
namespace** (`num_samples`, `save_images`, `output_dir`, `keyence_*`, ...). They
used to sit on `mobile_manipulator_system`; left there they silently fall back
to defaults.

### Camera lifecycle — the Basler must not stay on

Heat / sensor lifetime / power, and the VISION lamp must be lit only while the
shutter is open. So:

- The device is kept **`Close()`d** between captures, not merely "not grabbing".
- `basler_camera_node` owns the camera **and** the VISION lamp together, so they
  cannot desynchronise. Nothing else may open the device or publish
  `/crevis/led/vision`.
- Capture is a **service**, not a stream: the reply carries the frames, so a
  scan point can never be paired with a stale topic frame.
- `camera.idle_close_sec` keeps the device open briefly between adjacent scan
  points to avoid Open/Close thrash, then closes it automatically.

LED ownership is split by channel: `STATUS_{red,green,blue}` → `task_executor`,
`VISION` → `basler_camera_node`. Use `devices.shutdown(leds='status')` from any
node that does not own VISION.

## Navifra Base Driver Interface

The driver runs as separate nodes (not in this workspace). We consume:

| Topic | Type | Dir | Used by |
|-------|------|-----|---------|
| `/cmd_vel` | geometry_msgs/Twist | out | `robot_controller.py` |
| `/odom` | nav_msgs/Odometry | in | `robot_controller.py` |
| `/safety/estop` | std_msgs/Bool | in | `task_executor.py` (abort) |
| `/crevis/led/vision` | std_msgs/Bool | out | scan illumination |
| `/crevis/led/status_{red,green,blue}` | std_msgs/Bool | out | task-state lamp |
| `/bms/state` | sensor_msgs/BatteryState | in | low-battery warning |
| `/lift/*` | Bool/String/Int32/Int16 | both | `navifra_devices.py` |

All wrapped by `NavifraDevices` (`src/apriltag_nav/navifra_devices.py`) — nothing else in
`apriltag_nav` should touch raw driver topics. Config: `robot.yaml` `navifra:`.

**E-stop is hardware.** A PILZ PNOZmulti 2 cuts motor power independently of
ROS; `/safety/estop` is read-only feedback, never the stopping mechanism. The
software `/estop` topic was removed in driver v0.11 — don't look for it.
`/safety/estop` is fail-safe (true at startup and on PLC comms loss), so
`estop_active` reports true only after an actual message, and `safety_link_ok()`
covers the "driver not running" case separately.

⚠️ **The lift breaks the constant `arm_base_z`.** Pose-mode IK is silently
offset by the lift travel. Joint-mode tasks are unaffected. Transform code is
unchanged so far — see `docs/lift_arm_base_z_analysis.md` and the
`scan_height_guard` in `robot.yaml`.

## Network Map (per driver guide §1)

| Robot PC port | Robot PC IP | Device | Device IP |
|---|---|---|---|
| LAN 1 | 192.168.100.100 | Front / Rear LiDAR | .101 / .102 |
| | | Safety PLC (PNOZ) | .103 |
| | | Crevis GN-9289 IO | .104 |
| | | **Keyence DL-EN1** | **.105** |
| LAN 2 | 192.168.200.100 | Manipulator camera | .106 |
| LAN 3 | 192.168.58.100 | Fairino FR10v6 arm | 192.168.58.2 |
| LAN 4 | 192.168.1.100 | Wireless AP | 192.168.1.10 |

192.168.1.x is reserved for the AP; every LAN-hub device belongs on
192.168.100.x. Keyence moved 192.168.1.5 → 192.168.100.105 accordingly.

## Coordinate Frames

```
World Frame (CSV poses; origin at the polishing cell)
  │  world_x = -msg.y, world_y = -msg.x
  ▼
Manipulator Frame (/robot_pose msg: x, y, theta°)
  │  p_A_W = (x_base, y_base, body_off_z)                  # 2-DOF translation
  │  R_AW  = Rz(-(θ+mount_yaw)) · Ry(-tilt_y) · Rx(-tilt_x) # 4-DOF rotation (π yaw + small tilt)
  ▼
Arm base_link (IK input: mm+deg for Fairino SDK)
```

CSV orientation: **ZYX** intrinsic euler (radians). Target orientation for
IK comes directly from CSV (through `process_transforms`) — EE orientation
barely changes across a scan, so the CSV quat is reliable.

## Transform Parameters (4-DOF physical model)

Derived from the cell CAD geometry and validated against
328 paired joint/pose CSV rows (mean residual 12 mm, max 27 mm; replaced
the old 9-DOF calibration whose residual was >300 mm mean because a missing
mount_yaw=π was absorbed into other params).

```yaml
arm_body_offset_x:  0.0       # arm mount in body frame X (m, CAD: 0)
arm_body_offset_y:  0.0       # arm mount in body frame Y (m, CAD: 0)
arm_base_z:         1.0076    # arm base height in world (m)
arm_mount_yaw:      π         # arm base yaw vs body (rad, CAD: 180°)
arm_tilt_x:        -0.02248   # mount tilt X (rad, ≈-1.3°)
arm_tilt_y:         0.02639   # mount tilt Y (rad, ≈+1.5°)
```

All parameters overridable via ROS `~` private params.

## Coding Conventions

- **Python:** PascalCase classes, snake_case methods, UPPER constants
- **Comments in English**
- **ROS topics:** lowercase with `/` separator, scan results under `/scan/`
- **Config:** all tunable params in `config/robot.yaml`, not hardcoded
- **scipy compat:** use `as_dcm()` / `from_dcm()` (not `as_matrix()` / `from_matrix()`) for scipy < 1.4
