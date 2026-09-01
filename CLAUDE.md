# Mobile Manipulator Workspace

## Overview

> Picking this up on a different machine or in a fresh session? Read the
> **[Work Log](#work-log)** at the bottom of this file first — it records what
> changed, why, and what was verified, newest first.

Mobile manipulator system: Fairino FR10v6 6-DOF arm on a Navifra mobile base.
AprilTag visual navigation, arm scanning tasks, surface roughness (Ra) prediction.
Real robot only — simulation support has been removed.

Base driver: **Navifra KU Polishing Robot Driver v0.16** (separate ROS1 install
at `~/navifra`, systemd unit `navifra-robot`). It owns `/cmd_vel` + `/odom` plus
the lift / lighting / battery / Safety-PLC peripherals. Field tuning lives in
`~/navifra/param.yaml`, not in this workspace.

**Read `docs/HANDOVER.md` before substantive work** — it holds the base-swap
transition state: what is verified vs still open (motor-2 repair, lift
integration, arm_base_z re-validation, base dimensions) and the interim
operating rules.

## Tech Stack

- **ROS Noetic** / Ubuntu 20.04 / Python 3.8 / C++17
- **Arm control:** Fairino SDK (GetInverseKin / GetInverseKinRef / MoveJ)
- **Navigation:** dt_apriltags, Pure Pursuit, S-curve velocity
- **Base peripherals:** `navifra_devices.py` → lift, VISION/STATUS LEDs,
  BMS battery, Safety PLC e-stop feedback, charge relay
- **Camera:** Basler only (PyPylon) — `camera_interface.py` has no webcam fallback,
  and imports `pypylon` unguarded, so it is a hard runtime dependency
- **Tag cameras:** front_cam (Orbbec Femto Bolt, `orbbec_camera` driver,
  vendored under `src/orbbec_camera`) + side_cam (RealSense D405) + hand_cam
  (RealSense D435), both RealSense served by `realsense2_camera` **built from
  source** under `src/realsense-ros`. Detection-only — no capture-service or
  lamp semantics like the Basler; see `robot_camera_node.py` below.

  ⚠️ **Never `apt install ros-noetic-realsense2-camera`.** That package pins
  librealsense 2.50, whose hardcoded supported-device table predates the D405
  and rejects it with `Unsupported device! Product ID: 0x0B5B`. It is a version
  problem, not a cable or udev problem. Installing it also shadows the
  source-built driver, so a working D405 stops working.

  ⚠️ **`ros-noetic-ddynamic-reconfigure` is a genuine runtime dependency of the
  source-built `realsense2_camera` (it dynamically links
  `libddynamic_reconfigure.so`), but apt has no idea that dependency exists.**
  It was pulled in as an "automatic" package back on 2026-08-05, as a side
  effect of a one-time `apt install ros-noetic-realsense2-camera` (the full
  apt package — never installed on purpose, see above, but the run pulled its
  deps down first). Nothing in the devel-space build records that
  `realsense2_camera`'s `.so` needs it, so a later `sudo apt autoremove` sees
  no reverse-dependency and deletes it — which is exactly what happened
  2026-08-19, taking a pile of unrelated vlc packages with it. Symptom:
  `side_cam`/`hand_cam` nodelets die at launch with `Could not load library
  ... Poco exception = libddynamic_reconfigure.so: cannot open shared object
  file`; front_cam (Orbbec) is unaffected since it doesn't link it. Fix:
  `sudo apt install ros-noetic-ddynamic-reconfigure`, then
  `sudo apt-mark manual ros-noetic-ddynamic-reconfigure` so the next
  `autoremove` leaves it alone — reinstalling alone does not fix that, it goes
  back to "automatic" every time.
- **Sensor:** Keyence DL-EN1 (TCP)
- **Inference:** ONNX Runtime (CPU), ResNet3D; preprocessing uses
  `torchvision.transforms` + PIL (also hard runtime deps)
- **Transforms:** scipy, numpy
- **Offline validation only:** `validate_transform.py` / `validate_compare.py`
  use roboticstoolbox-python for URDF FK; not a runtime dependency

## Build & Run

```bash
catkin_make && source devel/setup.bash
roslaunch apriltag_nav mobile_manipulator.launch   # starts all eight nodes
```

The Navifra systemd service already runs `roscore` — do not start one.
`rosrun apriltag_nav task_executor.py` starts *only* the orchestrator and is a
debug path, not the way to bring the stack up.

## Task Commands

```
TASK <name>                # per-line: scan_joints_line{1,2}, scan_grid_line{1,2}
                           # merged: scan_full_pose  → *_ra_map.csv
                           # other: go_home
                           # (scan_joints_line1_new / move_route_A are commented
                           #  out in task_manager.py TASK_DEFS — not available)
                           # ⚠️ scan_full_joints does NOT register since
                           #    2026-08-24 — line1 is at lift 300 mm and line2
                           #    at 150, and it concatenates the two. See the
                           #    `lift_height` section.
GOTO <tag_id>              # Navigate to AprilTag
TEST_POSE x y z [rx ry rz] # Test pose control (debug)
STOP / STATE               # Emergency stop / query state
EXEC <code> / EVAL <expr>  # Debug execution
```

`scan_joints_line1` is the **105 / 106 scan** — `optimized_joints_line1.csv`
carries exactly those two `group_id`s (169 + 159 points), so it registers as
two steps and the arm scans at tag 105 then tag 106. Rows are sorted by
`(group_id, point_id)`, so 105 always runs first even though the CSV lists 106
first; from `START_TAG` 500 that means the robot **drives past 106 to reach
105 and then backtracks one tag**. Costs one corridor hop, nothing else. (Still
true in the new cell: zone B is entered at its north end and the IDs descend, so
`500→105` is `[500, 501, 505, 112 … 106, 105]`. Zone C ascends, so
`scan_joints_line2` has no backtrack.)

The two joint-mode scans no longer set the same lift height — **line1 is
300 mm, line2 is 150 mm** (see the `lift_height` column below), which is why
`scan_full_joints` is currently unregistered. The pose-mode `scan_grid_*` /
`scan_full_pose` deliberately set none — `arm_base_z` is a constant, so a
raised lift would offset every pose IK result.

### 🛑 EVERY scan CSV is invalid — the cell was replaced 2026-08-21

**Do not trust the output of any scan task.** `map.yaml` was replaced with the
new cell's map (see the design record in the parent directory's CLAUDE.md).
This supersedes the older "joint-mode scans are 100 mm off" warning: the error
is no longer 100 mm, it is *the wrong end of a different room*.

**Every `group_id` still resolves, which is exactly what makes it dangerous.**
The zone letter is preserved for all of them, so the robot faces the right way
and nothing errors — it just drives somewhere else:

| group_id | old cell | new cell |
|---|---|---|
| 105 / 106 (line1, zone B) | (−0.40, 2.45) / (−0.40, 2.85) | (−1.71, 0.15) / (−1.71, 0.55) |
| 117 / 118 (line2, zone C) | (2.80, 2.00) / (2.80, 1.59) | (1.71, 0.25) / (1.71, −0.15) |
| 129 / 130 (line3, zone D) | (3.47, 2.45) / (3.47, 2.85) | (2.18, −0.65) / (2.18, −0.25) |

On top of the move, the **joint angles themselves were solved for the old
cell's arm-over-plate geometry**, so joint mode cannot be rescued by any
transform — those CSVs have to be re-solved. Pose mode is recoverable in
principle (`arm_transform` follows `/robot_pose`) but its grid CSVs still
describe the old workpiece position.

🛑 **And the lift cannot make up the difference — the arithmetic forecloses
it.** The angles were solved with `arm_base_z` = **1.025** (the retired base).
The new base is **0.652**, a 373 mm drop, while the whole lift stroke is only
**343.35 mm**. So even at `soft_max_counts` the arm base reaches 0.995 m and is
still **30 mm short** of the height those angles assume; at line1's 300 mm it
is 73 mm short, and at line2's 150 mm, 223 mm short. No `lift_height` value
exists that makes an old joint CSV correct. Stop looking for one.

⚠️ **`_exec_joint` (`arm_controller.py:494`) is a bare `MoveJ(joints_deg)`** —
no reachability check, no collision check. The Keyence standoff loop runs
*after* the move completes, so it cannot intervene. A joint config whose TCP
was tuned to sit just above the old plate will be driven straight down toward
the new one. **This is a collision path, not a bad-data path** — do not run
`scan_joints_line*` on the new cell to "see what happens".

⚠️ `grid_path_line1_-5.csv` / `grid_path_line2_-5.csv` use `group_id` **4 and
5**, which were never valid in *either* map. Pre-existing, not caused by the
swap.

Two facts from the previous warning that are still load-bearing:

- `/robot_pose` is derived **only** from the map tag coordinate plus
  `lateral` plus `camera_offset` (`mobile_controller.calculate_robot_pose`).
  There is **no body-width term anywhere in the pipeline**, so changing the
  chassis does not move where the robot stops. `wall_dist_*` is documentation.
- Joint mode reads no transform at all: the CSV rows are absolute joint angles
  fed straight to `MoveJ`.

Merged-scan output is a 13-column CSV: `group_id, point_id, x, y, z,
ra_mean, ra_std, ra_min, ra_max, num_samples, success, execution_message,
validated_at`. Render with `tools/ra_map_plotter.py <csv> [--interpolate]`.

## Arm Controller Variants

| File | IK Engine | Scan |
|------|-----------|------|
| `src/apriltag_nav/arm_controller.py` | Fairino SDK + q0 ref | Yes (default) |
| `tools/arm_controller_sdk.py` | Fairino SDK (basic) | No |

Switch via the import in `scripts/arm_node.py`:
`from apriltag_nav.arm_controller import ArmController`

## Architecture

Package layout (inside `src/apriltag_nav/`):

```
scripts/            ROS node entry points only — 8 files, one per device
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
| `task_executor.py` | orchestration, STATUS lamp, e-stop, battery. **Owns no device** | `/task_command` |
| `mobile_node.py` | mobile base (**sole publisher** of `/cmd_vel` and `/robot_pose`) | `/mobile/goto_tag`, `/mobile/{stop,cancel,clear_stop}` (srv), `/mobile/state` |
| `arm_node.py` | Fairino FR10v6 arm | `/arm/scan_command`, `/arm/cancel`, `/arm/move_home` (srv) |
| `basler_camera_node.py` | wrist Basler **+ VISION lamp** | `/camera/capture` (srv) |
| `keyence_dlen1_node.py` | Keyence DL-EN1 | `keyence/value` |
| `robot_camera_node.py` | front_cam (Orbbec Femto Bolt) + side_cam (RealSense D405) + hand_cam (RealSense D435) AprilTag detection | `/<cam>/tag_detections`, `/<cam>/tag_overlay` (publish-only) |
| `lifter_node.py` | manipulator base lift (**sole writer** of `/lift/*`) | `/lifter/height_cmd`, `/lifter/{home,stop,reset,goto_scan_height}` (srv), `/lifter/state` |
| `camera_viewer_node.py` | RViz debug window (owns no device) | `/camera_viewer/set_enabled` (srv) |

`mobile_manipulator.launch` starts all eight. Seven are required;
`camera_viewer_node` is the only optional one (a debug aid that owns no device).

The calibration nodes (`path_tag_locator` + `map_calibrator`, plus
`handeye_calib` behind `use_handeye_calib:=false`) live in a SEPARATE
`path_tag_locator/launch/path_tag_locator.launch`, run alongside this
stack for a calibration session — the main launch must already be up.
Since the 2026-09-01 refactor they own **no** hardware: arm via
`/arm/move_cart`, base via `MobileClient`, observations via
`/<cam>/tag_detections`. ⚠️ `map_calibrator` is a second commander of
`/mobile/goto_tag`: no `TASK`/`GOTO` during a calibration session.

⚠️ **`/lifter/*` (this workspace) and `/lift/*` (the navifra driver) differ by
one character.** `/lift/*` is the raw driver with none of the guards below —
no soft travel clamp, no origin check. Read the topic name twice before
`rostopic pub`.

`robot_camera_node` **is** required despite vision-stop being inert: navigation
consumes its detections, so without it `detected_tags` stays empty, `/robot_pose`
never publishes and `GOTO` cannot work. What the empty `vision_stop.stop_tag_ids`
placeholder disables is only the *stop* decision, not the node.

### The three motion devices are the same shape

Drive, lift and arm each have **pure logic class → owner node → client proxy**:

| Device | Logic | Owner node | Proxy |
|---|---|---|---|
| drive | `mobile_controller.MobileController` | `mobile_node` | `MobileClient` |
| lift | `navifra_devices` lift command side | `lifter_node` | `LiftClient` |
| arm | `arm_controller.ArmController` | `arm_node` | `ArmClient` |

The proxy exists so `task_executor`'s call sites do not know a process boundary
is there. That is not decoration — when the drive was split out on 2026-08-11,
`self.mobile.move_to_tag(tag_id)`, `emergency_stop_robot()`,
`preempt_stop_robot()` and `clear_stop_flag()` were left **byte-for-byte
unchanged**; only the `import` moved. To swap an implementation, change the
import inside that device's node, never here.

### Mobile base split

`mobile_node.py` wraps `MobileController` unchanged — same rule as
`arm_node`/`ArmController`. Pure Pursuit, the S-curve ramp, tag-relative pose
estimation and the vision-stop decision all stay in the controller.

Before this split `task_executor` held a `MobileController` in-process, which
made the drive the one device without an owner. Three concrete costs:

1. **Two stop flags.** `task_executor._stop_requested` and
   `MobileController.stop_requested` are different variables, and every stop
   site had to set both by hand.
2. `move_to_tag()` blocks, so the orchestrator's main loop sat inside it for
   the whole drive and could not tick.
3. Retuning navigation meant restarting the whole stack.

⚠️ **`mobile_node` is the only publisher of `/cmd_vel`.** There is no
arbitration below it — navifra's `base_controller` obeys the last message it
received, so a second publisher fights this one at 50 Hz with no error
anywhere. `tools/navigate.py` holds its own `MobileController` and is exactly
such a second publisher; it is a standalone bring-up tool and must not be run
while the stack is up. `tools/vw_drive.py` (manual v/w driving) is the other
one — it at least *refuses to start* when it sees `mobile_node` registered,
which `navigate.py` does not.

`mobile_node` subscribes to `/safety/estop` itself rather than having
`task_executor` forward one. The base is the most dangerous device here, and a
stop that only works while the orchestrator is healthy is not a stop.

**Completion is `seq`-based, not position-based.** The lift has a measurable
end state (an absolute height); navigation does not — arrival is decided inside
`MobileController`. So `mobile_node` stamps each finished move with an
incrementing `seq` plus a `result` dict, and `MobileClient` waits for `seq` to
advance. This also removes the two races `LiftClient._saw_busy` exists to close.

`MapManager` moved into `mobile_node` with it: path finding belongs to whoever
drives. `task_executor` no longer imports it.

Stop services deliberately do **not** take the motion lock. A stop that waited
for the move it is trying to abort would never run. Same rule in `lifter_node`.

#### Two open navigation defects, both hit on the first real drive (2026-08-12)

Navigation itself works — a 56→57 move drove, steered and stopped on the tag
correctly (see the Work Log; those two test tags were deleted 2026-08-13, so
don't go looking for them in `map.yaml`). These are the two ways it fails
*silently*. Both were observed on hardware, not reasoned about.

🗓️ **Both are scheduled to be fixed on 2026-08-14**, on the user's decision
(2026-08-13: "둘 다 내일 고치자"). They had been left open since 2026-08-12
pending exactly that call, so this is the go-ahead, not a new request. Do them
together with that day's on-robot verification of the front_cam rotation port —
the first defect is the one that will show up *during* that verification, since
every drive ends with the tag at or past the frame edge.

⚠️ **`align_to_tag()` has no timeout.** When the target tag is not in
`detected_tags` it calls `stop()`, sleeps and `continue`s — forever. The only
exit is `stop_requested`. It runs immediately after `execute_pure_pursuit()`
returns, and the pure-pursuit stop condition is *the tag crossing the image
centre*, so arrival routinely leaves the tag at the frame edge or just outside
it. That is not an edge case, it is the normal end of every move.
`MobileClient.move_timeout_s` is 600 s, so the symptom is ten minutes of
nothing. **`center_x_stop_offset` (currently +50 px)** is what decides how much
tag is left in frame; **more positive** stops earlier and leaves more margin.
(It was `center_y_stop_offset: -50` until the 2026-08-13 camera rotation — same
physical stopping point, opposite sign, because the fore/aft image axis flipped
from the row axis to the column axis. Measured: the stop lands at body
x = 0.567 m either way.)

⚠️ **A frozen `/odom` deadlocks `execute_pure_pursuit()` at minimum speed.**
`traveled_dist` comes from odom, and the S-curve reads it: `traveled == 0`
means `accel_factor == 0` means `target = min_speed`. If the base is not
actually moving, traveled never grows, so the command never rises above
**0.01 m/s** and the loop keeps writing it to a dead drive for the full
60 s timeout with no error. Nothing checks that odom is advancing. This is how
a `MOTOR_FEEDBACK_TIMEOUT` presented as "the robot just sits there quietly".

#### Speed: what the config actually produces

Measured against the real loop and confirmed on hardware (5.1 s to reach top
speed, predicted 4.9 s). `max_linear_speed: 0.05` is the ceiling and **every
move reaches it** — `s_curve_accel_ratio + decel_ratio = 0.7 < 1`, so a cruise
window always exists, and the 0.05 m/s² ramp needs only 1.0 s / 2.5 cm.

| | value |
|---|---|
| top linear | 0.05 m/s (motor ≈ 52 rpm) |
| start/end floor | 0.01 m/s (motor ≈ 10 rpm) |
| top angular | 0.25 rad/s = 14.3 °/s |
| 0.40 m move | ~13.8 s, timeout 60 s |

Pure-pursuit steering never approaches the angular limit (6.4 °/s at 20 cm
lateral); only `align_to_tag` past ~20° of error and `execute_pivot` clip.
The navifra `base_controller` limits (2.0 m/s / 20 rad/s) are 40x away and
never bind — `robot.yaml` is the only constraint.

⚠️ **Four `robot:` keys in `robot.yaml` are read by nothing.** Tuning them does
nothing at all:

- `min_linear_factor` / `min_angular_factor` — `self.min_linear` /
  `self.min_angular` are computed in `MobileController.__init__` and never
  read. The real floor is `s_curve_min_speed_factor` (0.2 → 0.01 m/s). Note
  the comment at `min_linear_factor` claims it prevents motor stiction; the
  speed actually commanded is *lower* than the 0.015 m/s it intends.
- `slow_factor` — no reference anywhere.
- `navigation_timeout` (8.0) — `timeout_limit = max(8.0, D/min_speed * 1.5)`,
  and the second term wins for any D above 5.3 cm. It never applies.

### Arm split

`arm_node.py` **wraps** `arm_controller.ArmController`
unchanged rather than reimplementing it — that controller holds `TOOL_ID=1`
(vision_tip TCP), the q0 IK seed, the 4-DOF transform, the 13-column CSV and the
Keyence loop. A previous node-per-device attempt (the deleted `scripts_ros/`
tree) reimplemented the controller instead of wrapping it and silently lost
several of those — including using `tool=0` (flange) instead of `TOOL_ID=1`, so
its TCP offset was wrong. Wrap, don't rewrite. To switch controller
implementation, change the import in `arm_node.py`.

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

⚠️ **All of ArmController's `~params` now live in `arm_node`'s private
namespace** (`num_samples`, `save_images`, `output_dir`, `keyence_*`, ...). They
used to sit on `mobile_manipulator_system`; left there they silently fall back
to defaults.

### Camera switching — three independent cameras, live

`robot_camera_node` runs a detector per camera (front_cam / side_cam /
hand_cam), each with its own `dt_apriltags` Detector and subscribers, so any
one can be switched without touching the others. Three ways in, by lifetime:

| Scope | How |
|-------|-----|
| Live, no restart | `rosservice call /robot_camera/<name>/set_enabled "data: false"` |
| One run | `roslaunch … use_<name>_cam:=false` (also skips the driver) |
| Persistent default | `robot.yaml` `robot_camera.enabled.<name>` |

The service stops the detector **and** the vendor's stream, via the
`robot_camera.driver_toggle` service names (orbbec `/<cam>/toggle_color`,
realsense `/<cam>/enable` — both `std_srvs/SetBool`). Two rules that came from
getting it wrong:

- **Never toggle the driver at startup.** The launch already brought it up in
  the requested state, and re-asserting it is not a no-op — realsense answers
  a second `enable` with `open(...) failed. UVC device is streaming!` and can
  drop the stream it was already serving. Only real transitions touch it.
- **The launch writes `~driver_<name>` on every run**, not just when
  disabling. rosmaster outlives roslaunch here (the Navifra driver starts it),
  so a param written only in the false case survives as a stale `false` and
  silently overrides `robot.yaml` on every later launch.

### One detector, two consumers

`robot_camera_node` is the only thing that runs a tag detector. Navigation
consumes its output instead of detecting again:

```
front_cam ─▶ robot_camera_node ─▶ /front_cam/tag_detections ─┬─▶ detected_tags (nav)
                                                             └─▶ vision-stop check
```

`mobile_controller` used to run its own `dt_apriltags` Detector over
`topics.camera_rgb` (`/rgb`) — **a topic no launch in this workspace ever
published**. `rostopic info /rgb` showed `Publishers: None`, so
`image_callback` never fired, `detected_tags` stayed permanently empty and
`/robot_pose` never went out: `GOTO` could not have worked. Both `camera_rgb`
and `camera_info` are gone from `robot.yaml`; navigation reads
`front_cam_detections` and `front_cam_info`.

Consequences worth remembering:

- **`corners` rides in the message** (`float64[8]`, flattened). The alignment
  angle comes from the corner0→corner1 edge, and that maths is tuned — the
  raw corners travel rather than being recomputed or approximated from yaw.
- **One subscription, not two.** `detections_callback` updates `detected_tags`
  and then calls `vision_stop_callback`, so the stop decision always sees the
  same frame navigation is steering on.
- **`camera_params` can be None when detections arrive** — they come from
  different publishers now. The stop condition falls back to
  `image_height / 2`; the calibrated `cy` is usually a few px off that.

### Reading tag ID and orientation off the image

`robot_camera_node` publishes `/<cam>/tag_overlay` — the frame with an amber
crosshair on the optical axis, and per tag: its ID, offset from the crosshair
in px **and degrees of bearing**, roll/pitch/yaw, tilt and range. Pixels alone
are not comparable between cameras of different focal length, hence both.

Rendered only while something subscribes, so it costs nothing when no viewer
is open. The prepared RViz layout shows front_cam's overlay first.

`AprilTagDetection` carries the same numbers for consumers. Orientation comes
from `pose_R`, which the node previously discarded. The overlay shows only
yaw; roll/pitch/tilt stay in the message for anything that needs them.

**front_cam runs at 1280x720, not the driver's 1920x1080 default.** Measured
on the real robot, 1080p cost 140 ms of latency inside the driver alone (MJPG
decode) and held detection to 17 Hz against a 30 Hz stream; 720p publishes
RGB888, keeps detection at a full 30 Hz and cuts end-to-end latency to 109 ms.
1280x720 is also the *lowest* colour profile the Femto Bolt offers — 640x480
is rejected with "No matched video stream profile found" (the driver then
blames USB 2.0, which is misleading). Tune with `front_cam_width/height/fps`.

⚠️ **A square-on tag does not read rpy 0,0,0** — one angle sits at ±180,
because the tag's +Z points back at the camera. Which angle carries the flip
depends on the euler split (observed as roll in one case, yaw in another), so
never test squareness on a single angle. **`tilt_from_normal`** — the angle
between the tag's surface normal and the optical axis — is the number for
that: 0 = dead square, and it is invariant to spinning the tag in its own
plane (verified: 45° in-plane rotation still gives tilt 0).

### Looking at the cameras — `camera_viewer_node`

Debug aid, separate from `robot_camera_node` so the detection path has no GUI
in it. It owns no device and publishes nothing; it only starts and stops RViz
against `config/robot_cameras.rviz` (Image displays for all three cameras).

```bash
rosservice call /camera_viewer/set_enabled "data: true"    # open
rosservice call /camera_viewer/set_enabled "data: false"   # close
```

It starts **closed** — bringing the stack up must not throw a window on
screen, and a headless boot may have no display at all. `~auto_start` true
overrides that for a debug session.

It is a node in `mobile_manipulator.launch` specifically to bound its
lifetime: roslaunch shuts it down with everything else, and its
`rospy.on_shutdown` kills the RViz child. RViz is spawned with
`start_new_session=True` and signalled by process **group** — roslaunch
signals its own nodes, not their grandchildren, and RViz's helpers would
otherwise survive holding the X window.

Per-camera on/off belongs to `robot_camera_node`, not here; this node only
decides whether a window exists. A camera switched off there shows no image.

### Startup must not move the arm

`ArmController.__init__` clears faults and sets `Mode(0)`, but deliberately
does **not** move the arm to its home pose. Launching the stack is not a
request to move: the arm may have powered up inside a fixture or against the
workpiece, and an unattended `MoveJ` out of that pose is a collision risk at a
moment when nobody expects motion. Going to the home pose is explicit only —

- `task_executor` calls `move_to_home()` before every task (idempotent), and
- `/arm/move_home` (`std_srvs/Trigger`) does it on demand.

(This is the **arm home pose**, not lift origin homing — see the terminology
table two sections down. `lifter_node` follows the same startup rule for
its own, unrelated reason.)

Both controller variants (`arm_controller.py`, `tools/arm_controller_sdk.py`)
follow this. Don't reintroduce an init-time home to "get to a known state".

### ⚠️ "Homing" means two unrelated things — always qualify it

Two devices in this workspace have something called homing. They share no
code, no node and no semantics, and one of them moves the base underneath the
other. **Never write plain "homing" in code, comments or docs here.**

| | **Lift origin homing** (리프트 원점복귀) | **Arm home pose** (매니퓰레이터 홈 자세) |
|---|---|---|
| Device | base lift | Fairino FR10v6 arm |
| Node | `lifter_node` | `arm_node` |
| Call | `/lifter/home`, driver `/lift/home` | `/arm/move_home`, `ArmController.move_to_home()` |
| What it does | descends to the **lower limit switch**, resets the encoder origin to count 0 | `MoveJ` to a stored **home joint configuration** |
| Direction | always **down** | whatever the joints require |
| Establishes a reference? | **yes** — this is the whole point | no, nothing is zeroed |
| Needed how often | once per power cycle (count is lost on power-off) | idempotent, before every task |

`lifter.auto_home_on_start` is lift origin homing only; it never touches
the arm. The Navifra driver's own `auto_home_on_start` (in
`~/navifra/param.yaml`, currently **true**) is a third, separate setting that
does lift origin homing at driver start — before `lifter_node` exists.

### The base lift — `lifter_node`

Sole owner of the vertical lift under the arm (MDROBOT DC drive on RS485,
served by `lift_driver` in `~/navifra`). Stroke is 0..6900 hole counts =
**~343 mm** of arm-base extension (re-measured 2026-08-14: 343.2 mm at the
count 6897 the drive settles at), **~27.8 s end to end**. Four hardware facts make raw
`rostopic pub /lift/...` genuinely unsafe, and the node exists to hold them in
one place:

- **The upper end has no limit switch.** Only the lower one is wired. Driving
  up runs the mechanism into a hard stop, which the drive can only read as a
  stall; `auto_release_on_stop` then drops the velocity command to dodge a
  `CTRL_FAIL` alarm. `lifter.soft_max_counts` is **6900** as of 2026-08-14
  (7000 before that, 6700 before that) — the top of travel, and the count the
  mm scale was measured at. It still does real work beyond capping absolute
  targets: the same clamp bounds `jog` and the open-ended `up`/`down`, which
  run until stopped on the driver, so the node's jog is bounded by both the
  soft limit and `jog_timeout_s` and sends `stop` on every exit path. Do not
  raise it above 6900. ⚠️ The launch file sets it as a `~param`, so **both
  `robot.yaml` and `mobile_manipulator.launch` have to be changed together** —
  a `robot.yaml`-only edit does nothing.
- **The count is incremental and drifts.** It is lost on power cycle, and a
  descent started while pressed against the upper stop counts *backwards* for
  a few seconds (~+500), accumulating 1000–1800 counts of error per full
  up-down cycle. Lift origin homing to the physical lower limit switch is the
  only trustworthy origin.
- **Absolute position commands are silently ignored before origin homing.**
  That is what `/lifter/jog_cmd` (relative) is for — it works unhomed.
  ⚠️ **And `/lift/homed` reading True does NOT mean an absolute move will be
  accepted.** Observed on hardware 2026-08-14: `homed=1`, a completed homing in
  the journal 53 minutes earlier, `status=OK`, no alarm — and a `position
  command -> target=3014 counts` that the driver logged sending and the drive
  ignored for a full 60 s. Homing again fixed it, and the re-home itself
  **moved zero counts in 0.1 s**, so nothing physical changed. Recognise the
  signature — command logged, position frozen, no alarm — and re-home before
  debugging anything above the drive.
- **The drive has backlash**, so one count is two physical heights depending
  on the direction of travel. Hence the rule below. One origin pass takes it
  up; it does not need repeating (confirmed on the robot 2026-08-10).

⚠️ **The lift is capped at 1000 rpm by a hardware setting on the MDROBOT
controller**, so `up_speed_rpm` (currently 2000) and `post_home_speed_scale`
(2.0) in `~/navifra/param.yaml` are both already clipped and changing them does
nothing. The cap is raisable to 16000 but is deliberately left at 1000. That
is why every timing figure here is ~28.2 s per 7000 counts (so ~27.8 s over the
6900-count clamp) and why it does not vary between origin homing and a
post-home move. Earlier docs said "28 s at 2000 rpm" —
wrong; the 28.2 s measurement was always at 1000 rpm. If the cap is ever
lowered, `lifter.jog_timeout_s` (35 s, ~7.2 s of margin over a worst-case
0→6900 jog of ~27.8 s) has to be raised with it.

**Backlash: reach a scan height by homing and climbing.** The drive has play,
so a count reached by descending is not the same physical height as that count
reached by climbing. There is **no automatic defence** against this — an
`approach_from_below` rule that routed every descending move via the origin was
removed 2026-08-10 because it cost up to 56 s and guarded a case the task flow
cannot produce (every task ends with lift origin homing, nothing lowers the
lift mid-task, so absolute moves only ever climb from 0).

| Situation | What happens |
|---|---|
| target **above** current | drive straight up — the normal case |
| target **below** current | drive straight down, with a warning. The stop is loaded the other way; don't scan here |
| within `lift_settle_tol` of the target | no motion |
| origin unknown / position unreported | **refused** — the driver silently ignores absolute commands before homing, so success would be a lie. Use `/lifter/home`, or `jog_cmd` for a relative move |

⚠️ `jog` / `up` / `down` have no absolute target at all, so they too can leave
the drive loaded either way. Don't reach a scan height with them.

Startup does not move it, same rule as the arm's home pose. See the
terminology table above before touching anything called `auto_home_on_start` —
there are three of them and only one is in this workspace.

#### Setting the lift from a task — the `lift_height` column

A scan CSV may carry a `lift_height` column in **mm**. `task_executor` then
raises the lift to it once and holds it for the whole task.

**`optimized_joints_line1.csv` carries 300 mm** (6029 counts, ~24.2 s — set
2026-08-24) and **`optimized_joints_line2.csv` carries 150 mm** (3014 counts,
~12.1 s), on the measured 0.04976077 mm/count scale. So the arm base scans at
652 + 300 = **952 mm** for line1 and 652 + 150 = **802 mm** for line2. The grid
CSVs do not carry the column, so every pose-mode task is unchanged.

⚠️ **The ceiling is 343.35 mm** (`soft_max_counts` 6900 × `mm_per_count`), and
overshooting it does not clamp-and-continue — it **fails the task**.
`lifter_node._clamp` caps the target, then `LiftClient._verify` compares the
height actually reached against the one requested with `tol_mm` = **1.5 mm**,
so a request of 350 stops at 343.35, misses by 6.65 and aborts at the lift step
with `state = ERROR` before any scanning. 350 was tried on 2026-08-24 and
backed out to 300 for exactly this. Treat **343** as the practical maximum.

⚠️ **The 2026-08-14 rescale moved that stop by 3.2 mm.** The same
`lift_height: 150` used to convert to 3079 counts, which is 153.2 mm of real
travel on the corrected scale — so joint-mode scans now sit **3.2 mm lower**
than every run before 2026-08-14. That is the scale being fixed, not a
regression, but the joint angles in those CSVs were solved at one base height:
if a scan starts fouling or missing standoff, this is the 3.2 mm to remember.

⚠️ **Both line CSVs have to agree, and as of 2026-08-24 they do not.**
`scan_full_joints` concatenates them and `_extract_lift_height` refuses both a
partly-filled column and one whose values disagree, so it **unregisters the
task** rather than degrading gracefully. With line1 at 300 and line2 at 150 it
logs `lift_height disagrees across rows ([150.0, 300.0]) ... Refusing to load`
at startup and `TASK scan_full_joints` is simply not a command. Accepted for
now — every joint CSV has to be re-solved anyway (see the blocker above).
Setting line2 to 300 as well is what restores it.

```
TASK → arm home pose → drive to first tag → SET LIFT → scan group
     → drive to next tag → scan group (lift untouched)   … repeat …
     → arm home pose → LIFT ORIGIN HOMING → stop, wherever it is
```

- **One value per task, enforced.** The joint angles in the CSV were solved at
  one base height, so a file whose rows disagree is **refused at load time**
  rather than resolved by picking a winner — `task_manager._extract_lift_height`
  also rejects a partly-filled column, since a blank cell is not 0 mm. A CSV
  with no such column is unchanged: the lift is never commanded.
- **The lift is set after arriving at the first tag**, not before driving. A
  raised lift puts the arm's mass high while the base is moving.
- **`LiftClient` (`src/apriltag_nav/lift_client.py`) is the only way in**, same
  role `ArmClient` plays for the arm. It reconstructs a synchronous call by
  watching `/lifter/state` rather than calling a service — see
  *Topic-and-poll vs. a service* below for why, and for what that trade is
  really about. The tricky part is `_saw_busy`
  being a **latch** rather than a snapshot: sampling `busy` right after
  publishing can catch the pre-command state and report success while the lift
  is still about to move, with the arm already scanning.
- **`lift_height` and `navifra.scan_height_counts` are two sources of truth.**
  When a task names a height, the CSV wins and the per-group scan-height guard
  is skipped; the conflict is reported once instead of once per group. Joint
  mode is unaffected by either, but **pose mode is not** — `arm_base_z` is a
  constant, so raising the lift offsets every pose IK result.

#### Topic-and-poll vs. a service — and what the real constraint is

`LiftClient.goto_mm()` publishes `Float32` to `/lifter/height_cmd` and
polls `/lifter/state`. `home()` and `stop()` are ordinary `std_srvs/Trigger`
calls. Both models live in one class, split on a single question: does the call
need to carry an argument?

⚠️ **Not because services cannot carry arguments — they obviously can.** A
`.srv` is just two `.msg` blocks separated by `---`, run through the same
generator. `robot_msgs/CaptureImages.srv` in this very workspace takes an
`int32`, a `float32` and a `bool`. The narrower true statement is that
**`std_srvs` ships only `Empty` / `Trigger` / `SetBool`**, none of which has a
float request, while `std_msgs` does ship `Float32`. That asymmetry is an
accident of what the standard packages contain, nothing deeper.

And it would not have blocked a service anyway: `apriltag_nav` has no
`message_generation`, but **`robot_msgs` does**, apriltag_nav already depends on
it, and a `SetLiftHeight.srv` could go there in a few lines. (An earlier note
here claimed a custom srv was impossible. It was wrong; corrected 2026-08-10.)

⚠️ **Topics do not make the motion cancellable.** An earlier version of this
section implied they did. Stopping the lift means `/lifter/stop` →
`devices.lift_stop()` no matter how the command arrived, and it always takes a
second thread to call it. `_srv_stop` deliberately does not take the motion
lock and trips the cancel flag the blocking mover polls, so a service call
would have been released by that same stop. Both models need the identical
stop path.

The one difference that survives is narrow: if `lifter_node` goes
**unresponsive**, a rospy service call blocks forever — an in-flight call takes
no timeout, and `wait_for_service(timeout=)` only covers connecting — whereas
the poll loop owns its own deadline and returns after `move_timeout_s`.

And it is paid for. `_saw_busy`, the two-phase wait and the race they exist to
close are artifacts of *reconstructing* completion from an async state stream.
A service has none of them, because `_do_goto` already holds the lock, blocks
to completion and returns `(ok, msg)` — correct server-side logic that already
exists.

| | topic + poll (current) | service |
|---|---|---|
| node hangs | caller frees itself after `move_timeout_s` | caller blocked forever; needs a watchdog |
| race | must hand-build the latch | none |
| build | no change | `.srv` in `robot_msgs` + `catkin_make` |

So this is a real trade, not a clear win, and the current code does not even
use its own advantage — the loop watches `is_shutdown()` and its deadline but
ignores `task_executor._stop_requested`.

⚠️ **The ROS1 primitive designed for this is `actionlib`**: goal arguments,
cancellation in the protocol, a feedback stream, client-side timeouts. A
long-running, cancellable, progress-reporting motion is exactly its use case,
and `robot_msgs` could host the `.action`. Neither `LiftClient` nor `ArmClient`
uses it — that is inherited habit, not a technical finding. Weigh it first if
either client is ever reworked.

By contrast `CaptureImages` is *correctly* a service: sub-second, nothing to
cancel, and it has a real result to return (the frames). Decide the next device
command on duration / cancellability / whether there is a result — not by
copying whichever shape the last one used.

`task_flow.lift_home_on_finish` in `robot.yaml` gates the tail. It does not run
on a preempted or failed task, and it is what lets the next task assume the
lift starts at the origin.

#### Tasks are composable blocks — none of them drives home

**A task never returns to `START_TAG` on its own.** Returning is its own task
(`TASK go_home`), so "scan then come back" is two commands and "scan then scan
somewhere else" is two different ones, built from the same pieces. This is
deliberate and is the shape the planned block-coding GUI needs: each block is
one self-contained task, they run in the order they are placed, and the system
waits at IDLE between them. Anything that auto-appends motion to the end of a
task breaks that composition — a brief `return_home_on_finish` flag was tried
on 2026-08-10 and removed the same day for exactly this reason.

`lifter.mm_per_count` is 343.2 mm / 6897 counts = **0.04976077**, re-measured
2026-08-14 at the top of travel: commanded to the new `soft_max_counts` of
6900, the drive settled at count **6897** (inside `lift_settle_tol`) and the
gauge read **343.2 mm** of extension. **The scale is derived from the count
actually reached, not the commanded one** — using 6900 would have baked the
3-count settle error into every conversion. So the manipulator base sits
**652 mm** above the ground at the lift origin and **~995 mm** at the top.
`mm_calibrated` stays **true** and the startup warning does not fire.
(Superseded: 341 mm / 7000 = 0.0487143 from 2026-08-13, which ran 7.2 mm low
over the stroke; and 350 mm / 7031 = 0.0497795 before that, where the 350 mm
was a catalogue figure that had never met a height gauge.)

Re-measuring it is a bench job, and `tools/lift_calib_ui.py` is the bench: it
drives the lift in **counts**, takes the gauge reading you type in, and fits
the line. Counts and not mm on purpose — the only mm entry point in the system
is `/lifter/height_cmd`, which converts with the `mm_per_count` under test,
i.e. measures the scale against itself.

⚠️ **That tool runs on the navifra driver alone and writes `/lift/*`
directly**, so it is a second writer whenever the stack is up — the same rule
as `tools/navigate.py` / `tools/vw_drive.py` on `/cmd_vel`, and like
`vw_drive.py` it enforces it by refusing to start when `lifter_node` is
registered. It therefore carries its own soft travel clamp, because that guard
normally lives in `lifter_node`. It also re-enables an approach-from-below rule
that `lifter_node` deliberately does not have, since a calibration sweep is the
one case that rule guarded; see the Work Log entry.

⚠️ What this does *not* fix is `arm_base_z` tracking the lift. That value is
still a constant fitted at the lift origin, so pose-mode IK is still offset by
whatever the lift has travelled — see below. The measured scale is the
*prerequisite* for a dynamic `arm_base_z`, not the thing itself.

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
- The scan loop **pre-opens** the device (`/camera/set_active` true) at the
  start of each scan point, so the open latency overlaps arm motion + Keyence
  adjustment instead of adding to capture time, and releases it when the scan
  ends or is cancelled. The lamp is untouched by pre-open — it stays bracketed
  with the shutter inside the capture service.

LED ownership is split by channel: `STATUS_{red,green,blue}` → `task_executor`,
`VISION` → `basler_camera_node`. Use `devices.shutdown(leds='status')` from any
node that does not own VISION.

## Navifra Base Driver Interface

The driver runs as separate nodes (not in this workspace). We consume:

| Topic | Type | Dir | Used by |
|-------|------|-----|---------|
| `/cmd_vel` | geometry_msgs/Twist | out | `mobile_node.py` (**sole publisher**) |
| `/odom` | nav_msgs/Odometry | in | `mobile_node.py` |
| `/safety/estop` | std_msgs/Bool | in | `task_executor.py` (abort), `mobile_node.py` + `lifter_node.py` (each cancels its own motion, independent of the orchestrator) |
| `/crevis/led/vision` | std_msgs/Bool | out | scan illumination |
| `/crevis/led/status_{red,green,blue}` | std_msgs/Bool | out | task-state lamp |
| `/bms/state` | sensor_msgs/BatteryState | in | low-battery warning |
| `/lift/*` | Bool/String/Int32/Int16 | both | `lifter_node.py` (writes), `task_executor.py` (reads) |

All wrapped by `NavifraDevices` (`src/apriltag_nav/navifra_devices.py`) — nothing else in
`apriltag_nav` should touch raw driver topics. Config: `robot.yaml` `navifra:`.

### Drivetrain geometry lives in `~/navifra/param.yaml`

`base_controller` there owns **`wheel_radius: 0.0825`** (0.165 m diameter) and
**`wheel_separation: 0.65`**. The driver does all odometry from them; nothing
in `apriltag_nav` reads either one. They are mirrored into `robot.yaml`
`robot:` for visibility only — **if the two ever disagree, `param.yaml` wins**
and the workspace copy is the stale one.

`robot.length` / `robot.width` are **0.90 / 0.70** as of 2026-08-13 (the old
base was 0.80 / 0.50, a width narrower than the 0.65 m wheel track and
therefore impossible). Nothing reads either key.

`wall_dist_work_zone` was raised **0.35 → 0.45** on 2026-08-13 to go with it.
These are centre-to-wall distances and they track the body half-width:
`0.35 − 0.25` (half of the old 0.50) `= 0.10 = min_wall_clearance`, and
`0.45 − 0.35 = 0.10` preserves it. The new cell happens to agree exactly —
every corridor lane sits 450 from the 정반 face — so 0.45 survived the
2026-08-21 map swap unchanged. **`wall_dist_zone_a` did not: 0.6275 → 0.52**,
because the new zone A lane (y = 3.02) is 520 from 정반 1's top edge and was
chosen to balance the pivot between the north wall and the plate corner, not to
hold a clearance figure. Nothing reads these three keys
today — the robot's real stop position comes from centring the tag under
front_cam, not from here.

**The 100 mm the wider body costs was paid back by moving the arm, not the
base.** The arm mount was physically shifted 100 mm toward the wall
(`arm_body_offset_y: -0.100`), so its absolute position over the workpiece is
unchanged even though the chassis centre sits 100 mm further out. That is the
whole point of the pairing — see *Transform Parameters* and the blocker below.

`NavifraDevices` is the *wrapper*, not the *owner*: for the lift, the owner is
`lifter_node`, which is the only node allowed to call the command side
(`lift_home/goto/jog/up/down/stop/velocity`). Reading (`lift_position`,
`lift_at()`) stays open to anyone — `task_executor`'s scan-height guard does
exactly that. Everything else should command the lift over `/lifter/*`.

**E-stop is hardware.** A PILZ PNOZmulti 2 cuts motor power independently of
ROS; `/safety/estop` is read-only feedback, never the stopping mechanism. The
software `/estop` topic was removed in driver v0.11 — don't look for it.
`/safety/estop` is fail-safe (true at startup and on PLC comms loss), so
`estop_active` reports true only after an actual message, and `safety_link_ok()`
covers the "driver not running" case separately.

## Keyence Distance Loop

`_adjust_distance_to_surface()` in `arm_controller.py` nudges the tool along
tool Z before each capture. Three things about it are not guessable from the
code — full record in `docs/keyence_scan_chain.md`:

- **The laser is mounted oblique, 42.6° off tool Z** (measured, not documented
  anywhere in the URDF or TCP). The reading is a distance along the *beam*, so
  it is projected by `cos(beam_angle_deg)` before anything else. After that
  projection `keyence_tol`, `keyence_max_step_mm` and
  `keyence_activate_threshold` are all **perpendicular standoff mm** — never
  compare them against the raw reading.
- **`keyence_dir` must be `-sign(k)`, currently −1.0.** The sensor reads 0 at a
  10 mm standoff, negative when too far, positive when too close, so a positive
  reading must *retreat*. It sat at +1.0 for a long time, which amplifies the
  error by (1 + kp) per step and drives the tool into the workpiece; it was
  never noticed because `keyence_dlen1_node` was commented out of the launch
  file, so the loop had literally never run. Re-derive with
  `tools/measure_keyence_angle.py` if the sensor is remounted.
- **`keyence_max_step_mm` is 1.0 as a bring-up guard and is doing real work.**
  The oblique beam walks the laser spot `0.919*dz` sideways per correction, so
  on a sloped surface the effective sensitivity is much larger than the
  calibrated 1.358 — one observed step hit 7.52, past the divergence limit of
  3.40. The clamp is what kept that from running away. Don't restore 5.0 without
  reading the open-issues section of the doc.

⚠️ **The lift breaks the constant `arm_base_z`.** Pose-mode IK is silently
offset by the lift travel. Joint-mode tasks are unaffected. Transform code is
unchanged so far — see `docs/lift_arm_base_z_analysis.md` and the
`scan_height_guard` in `robot.yaml`.

## Vision-Triggered Soft Stop (front_cam)

`robot_camera_node` publishes `AprilTagDetectionArray` (per-frame, per-camera)
to `/front_cam/tag_detections` and `/side_cam/tag_detections`. It never acts
on a detection itself — it is the same "device owner publishes, consumer
decides" split as `basler_camera_node` / `task_executor`.

`mobile_controller.py`'s `vision_stop_callback` (subscribed only to the
front_cam topic; side_cam is published but has no consumer yet) is the
consumer: when a tag ID listed in `robot.yaml` `vision_stop.stop_tag_ids` is
detected within `center_tolerance_px` of the image center, it calls
`preempt_stop_robot()` — the same soft-stop used when a new `TASK`/`GOTO`
preempts the current motion, not the hardware-e-stop path.

No separate "stay stopped until resumed" logic exists or was added: it falls
out for free from the existing latch. `stop_requested` is only cleared inside
`move_to_tag()` via `clear_stop_flag()`, and every movement loop
(`align_to_tag`, `execute_pure_pursuit`, `execute_pivot`) bails out
immediately while it is set. So once vision-stop trips, the robot stays put —
even after the tag leaves view — until the next explicit `TASK`/`GOTO`.

⚠️ **`vision_stop.stop_tag_ids` is an unfilled placeholder (`[]`).** The
feature is inert until a real robot deployment fills in the actual tag ID(s)
to stop on; do not test on hardware with a guessed ID.

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

Source of truth: **`path_tag_locator/config/extrinsics.yaml` `T_ab2mb`**
(platform-measured; that file explicitly deprecates earlier tunings). The
mount has **no tilt** — R is exactly Rz(180°) — which the 655-point real-robot
fit (`task/csv/calib_data_params.yaml`, tilt ≈ 0.0001/0.0007 rad) confirms
independently. The USD-derived values used before (base_z 1.0076, tilts
-1.3°/+1.5°) are superseded; those tilts do not exist on the real platform.

```yaml
arm_body_offset_x:  0.0       # arm mount in body frame X (m)
arm_body_offset_y: -0.100     # arm mount in body frame Y (m) — moved 2026-08-13
arm_base_z:         0.652     # arm base height above ground (m), lift at origin
arm_mount_yaw:      π         # arm base yaw vs body (rad, exact)
arm_tilt_x:         0.0       # no mount tilt
arm_tilt_y:         0.0       # no mount tilt
```

`T_ab2mb` t is therefore **(0, -0.100, -0.652)**. Both signs come out negative
because Rz(180°) flips y and the inverse flips it back; the arm did move toward
the wall, i.e. body **-Y**.

⚠️ **`arm_body_offset_y` corrects POSE mode only.** `arm_transform.py` reads it
into `p_A_W`, and `transform_world_to_arm` is called from exactly one place —
`arm_controller._exec_pose`. Joint-mode CSVs are absolute joint configurations
fed straight to `MoveJ`; **no transform touches them**, so moving the arm on the
chassis moves where every joint-mode point lands, by the same 100 mm. See the
blocker below.

All parameters overridable via ROS `~` private params.

⚠️ **`arm_base_z` is 0.652 as of 2026-08-23 — the mobile base was replaced.**
It was 1.025 on the old base, so every `arm_base_z` figure in Work Log entries
before 2026-08-13 is against different hardware and is not a discrepancy to
chase. (It read **0.651** between 2026-08-13 and 2026-08-23, when it was
corrected by 1 mm to match the cell design record's 652 — see §3 of the parent
directory's CLAUDE.md. Work Log entries citing 651 predate that correction.) In particular the old "655-point fit says 0.9541, extrinsics says 1.025,
71 mm apart" open item is **closed by obsolescence**: both numbers describe the
retired base. The 12 mm mean-residual figure is gone with them.

⚠️ **The value is measured with the lift at its origin**, and it is a constant
that does not track the lift. At the top of the stroke the arm base is at
0.995 m — a 343 mm error if a pose-mode scan runs there. Joint-mode tasks are
unaffected. See `docs/lift_arm_base_z_analysis.md`.

`T_mb2fc` front_cam translation is **(0.55, 0, 0.300)** as of 2026-08-21 (was
`(0.547, 0, 0.300)` from 2026-08-13, and `(0.45, 0, 0.293)` before that). The
height moved only 7 mm even though the deck dropped 374 mm, so the camera is
mounted off the chassis, not off the deck. `tx` must stay equal to `robot.yaml`
`camera_offset`, which is the only key in that block any code reads.

⚠️ **The last 3 mm is a design figure overriding a measurement, and it is not
reconciled.** 0.547 was measured on the base 2026-08-13. 0.55 is what the new
cell's `map.yaml` was *generated* from — every tag sits at
`stop pose + 0.55 × heading` — and it is the only value that reproduces the
design record's documented dock stop of x = −1.9123 (0.547 gives −1.9093).
Since the map and the code have to agree with each other before either agrees
with a tape measure, the map's number won. **If 547 is the true lens position,
the tags are the thing to move, not this key** — changing it alone silently
shifts every derived stop pose off the tag grid.

### front_cam rotated −90° — navigation adapted 2026-08-13

The camera was physically turned **−90° about its own (downward) optical axis**
on 2026-08-13. `fc` *is* the optical frame the detector reports in, so this
re-labels every `pose_x` / `pose_y` / `center_x` / `center_y` the navigation
code consumes:

| | before | now |
|---|---|---|
| `fc.x` — image column + (right) | `-mb.y` (robot's right) | **`+mb.x`** (forward) |
| `fc.y` — image row + (down) | `-mb.x` (aft) | **`-mb.y`** (right, wall side) |
| `fc.z` — optical axis | `-mb.z` (down) | `-mb.z` (down, unchanged) |

**The lens is 100 mm AHEAD of the front bumper** — the body is 0.90 m long so
its front face is 0.45 m out, and `camera_offset` is 0.55 m. That is why the
two things the operator sees are not contradictory: the **bumper appears on the
LEFT** of the image (it is *behind* the camera) while an **approaching tag
enters from the RIGHT** (it is ahead). Both follow from `fc.x = +mb.x`, and
both were observed on the robot — that is what fixed the sign.

`extrinsics.yaml` and `mobile_controller.py` are **both updated**. It broke in
three independent places, all now fixed:

| Site | What it assumed | What it now does |
|---|---|---|
| `lateral = tag['x']` (`calculate_robot_pose`, `execute_pure_pursuit`) | image X runs left/right, + = right | **`tag['y']`**, sign unchanged — image down = robot right, so the steering sign never moved |
| `center_y` stop condition | image Y runs fore/aft, and grows as you approach | **`center_x`** vs `camera_params[2]`, falling back to `image_width/2`. It **decreases** on forward approach, so **both comparisons are inverted** and the config key became `center_x_stop_offset: +50.0` |
| `align_to_tag` / `corner0→corner1` angle | edge angle reads 0 when square | the shared `tag_edge_angle_deg()` helper **subtracts 90°** before the wrap. Measured, not assumed: a floor direction along `+mb.x` imaged at −90° before and 0° after |

`path_tag_locator` carried a second, unfixed copy of this logic in
`nav/robot_controller.py` + `config/robot_nav.yaml`. **Resolved 2026-09-01 by
deletion**: the copy was first ported (verified offline, 16 checks) and then
the same-day standalone-mode refactor deleted the whole `nav/` tree — base
navigation now goes through `mobile_node` via `MobileClient`, so there is
exactly one implementation of this logic in the workspace.

`get_current_tag_id` is safe — `hypot(x, y)` is invariant to in-plane rotation.
`dist_to_tag = tag['z']` is also unaffected, but note it was **already** just
the ~0.30 m camera height and never the distance to the tag, so the pure-pursuit
lookahead `L = max(dist_to_tag, 0.4)` has always been pinned at 0.4. Pre-existing,
not caused by the rotation — but the true fore/aft distance is now available as
`pose_x` if that is ever worth using.

Everything needed already shipped in `AprilTagDetection` (`center_x`) and
`AprilTagDetectionArray` (`image_width`); `camera_params[2]` is `cx`. No message
or node change was required.

**Verified offline, not on hardware** (38 checks, `/tmp/t_nav_rot.py`). The
strongest of them reconstructs the *pre-rotation* camera and the *old*
`center_y` logic with its old −50 offset and compares stop distances on a 1 mm
grid: forward **0.567 m** and reverse **0.561 m**, identical old-vs-new to
1e-9. So this is a faithful mirror of the tuning that already worked on
hardware, not a re-tune. The rest: tag ahead images right / bumper images left /
robot's right images down (all derived from `T_mb2fc`, not from the code),
`center_x` monotonically decreasing on approach, the stop being monotone and
landing with the tag still ahead of the lens, the steering sign unchanged in
both directions, and `tag_edge_angle_deg` reading 0 for a square tag.

## Coding Conventions

- **Python:** PascalCase classes, snake_case methods, UPPER constants
- **Comments in English**
- **ROS topics:** lowercase with `/` separator, scan results under `/scan/`
- **Config:** all tunable params in `config/robot.yaml`, not hardcoded
- **scipy:** write COMPAT calls —
  `rot.as_matrix() if hasattr(rot, 'as_matrix') else rot.as_dcm()` (same
  for `from_matrix`/`from_dcm`). The earlier ">= 1.4 only, old scipy
  cannot even import" claim is empirically FALSE on this robot PC: it
  runs scipy **1.3.3** with numpy 1.23.5 and imports fine (the
  `np.typeDict` failure does not hit `scipy.spatial.transform`), so bare
  `as_matrix()` crashes at runtime here — it silently emptied every
  `/…/tag_detections` array until 2026-09-01 (found in review; three
  files patched: `robot_camera_node.py`, `arm_transform.py`,
  `arm_controller.py`).

---

## Deferred — Korean guide (`docs/GUIDE_kr.md` + the PDF)

**Do not touch these files without asking the user first.** Regenerating the
PDF is expensive and the user has asked for the backlog to be cleared in one
batch rather than per session. This list is that backlog — append to it instead
of editing the guide.

| Where | What is now wrong |
|---|---|
| §2.1 node table, lines 63 / 66 | says 6개 노드, "6개 중 5개 필수". It is **7개 중 6개 필수** since `lifter_node` (2026-08-07). |
| Troubleshooting, line 295 | "launch가 띄우는 6개 노드" — same count error. |
| line 653 | "약 7000 카운트, 전 구간 약 28초(2000 rpm)" — both halves are now wrong. The clamp is **6900 counts ≈ 343 mm** (re-measured 2026-08-14, 343.2 mm at count 6897), and the speed is **~27.8 s at 1000 rpm**, which is a hardware cap. |
| Wherever the lift scale appears | `mm_per_count` is **0.04976077** (343.2 mm / 6897 counts) since 2026-08-14, not 0.0487143 / 0.05. The arm base tops out at **~995 mm**, and `lift_height: 150` is **3014 counts**. |
| line 704 Appendix / §7 | `arm_base_z` is **0.652 m**, not 1.025 — the mobile base was replaced 2026-08-13, and the value was corrected 0.651 → 0.652 on 2026-08-23. Joint-mode scans therefore sit at 652 + 150 = **802 mm**. |
| line 1070 open-issues table | drop the "`arm_base_z` 1.025 vs 0.9541 (71 mm)" row entirely; both figures belong to the retired base. |
| Appendix A | missing the `lifter:` and `task_flow:` blocks of `robot.yaml`. |
| Appendix B checklist | predates `lifter_node`. (The "`mm_calibrated` is false" caveat it was missing is now moot — measured 2026-08-13, it is true.) |
| Throughout | "homing" is still used for both senses. The workspace now separates **리프트 원점복귀** (lift origin homing) from **매니퓰레이터 홈 자세** (arm home pose). |
| Missing entirely | the `lift_height` CSV column and the task flow it drives; that a task ends with lift origin homing and then **stays put** (`go_home` is a separate task); that absolute lift moves are refused before origin homing. |
| Everywhere | **node names renamed 2026-08-11**: `arm_controller_node` → `arm_node`, `base_lifter_node` → `lifter_node`. Also `robot_controller.py` → `mobile_controller.py` and `RobotController` → `MobileController`. Affects §2.1, §2.4 (line 268 sample output), §5, §7 topic/service tables and the troubleshooting table. |
| Everywhere | **`/base_lifter/*` → `/lifter/*`** in the same pass, and `robot.yaml`'s `base_lifter:` block key is now `lifter:`. Appendix A must follow. |
| §2.1 node table + line 295 | node count is now **8개 중 7개 필수** — `mobile_node` was added 2026-08-11 and is required. |
| §7 topic tables | `/cmd_vel` and `/robot_pose` are published by **`mobile_node`**, not `mobile_manipulator_system`. New: `/mobile/goto_tag`, `/mobile/state`, `/mobile/busy` and the `/mobile/{stop,cancel,clear_stop}` services. |
| Missing entirely | that `task_executor` now owns **no device at all** — drive, lift and arm are each reached through a client proxy. Worth a short section; it is the main structural change since the guide was written. |
| Task list / §5 | `scan_joints_line1_lift` no longer exists (retired 2026-08-13). Both `optimized_joints_line*.csv` now carry `lift_height: 150`, so **every joint-mode scan raises the lift 150 mm** and pose-mode scans still do not. |
| Appendix / §7 | robot footprint is **0.90 x 0.70 m** (was 0.80 x 0.50), `wheel_radius` 0.0825 / `wheel_separation` 0.65, and `T_mb2fc` is **(0.55, 0, 0.300)** (was `(0.45, 0, 0.293)`). |
| Wherever wall clearance appears | `wall_dist_work_zone` is **0.45** (was 0.35). `wall_dist_zone_a` is **0.52** (was 0.6275) since the 2026-08-21 cell change. |
| §7 / Appendix, transform block | `arm_body_offset_y` is **−0.100 m** — the arm mount was moved 100 mm toward the wall 2026-08-13, so `T_ab2mb` t is `(0, −0.100, −0.652)`. Explain that this corrects **pose mode only**. |
| Wherever `camera_offset` appears | it is **0.55 m** (was 0.45, briefly 0.547) and it is the only key in the `robot:` block that any code reads. |
| Wherever the cell layout / tag map appears | **the whole cell was replaced 2026-08-21.** Coordinate origin is now the **centre of 정반 1**, and `map.yaml` holds **72 tags / 142 edges** in four corridors (zones B/C/D/E) plus a zone A transit lane. Every tag ID in the old guide is wrong. |
| Wherever the home / dock tag appears | `TaskManager.START_TAG` is **500**, not 508. In the new map 508 is a zone-E pivot tag, so the old value would drive to the far end of the cell. |
| Wherever `tag_size` appears | there are now **two physical tag sizes**: 90 mm floor tags (front_cam, straight down) and 30 mm tags on the 정반 step (side_cam, horizontal). Neither is the old 60 mm. |
| Appendix A, `robot_camera:` block | `tag_size` is no longer a scalar — it is a **per-camera dict** (`front_cam: 0.09`, `side_cam: 0.03`, `hand_cam: null`), with `null` falling back to `robot.tag_size`. |
| Navigation / troubleshooting | front_cam was **rotated −90° about its optical axis** 2026-08-13: image right = robot forward, image down = robot right. `mobile_controller.py` **was adapted the same day** — no longer a blocker, but the axis meanings need updating wherever the guide explains what the camera sees. |
| Wherever the stop offset appears | the key is now **`center_x_stop_offset: +50.0`** (was `center_y_stop_offset: -50.0`) and **more positive** stops earlier. Same physical stop point; the fore/aft image axis moved from rows to columns. |

## Work Log

Newest first. **Append an entry for every session that changes this workspace.**
Record the *reasoning* and what was *verified*, not a file diff — the diff is in
git, the reasoning is not. Keep entries short; promote anything that becomes a
standing rule up into the sections above instead of leaving it buried here.

### 2026-09-01 — full review of the day's work: 12 findings fixed, 2 of them serious

Two parallel review passes over the whole staged set (49 files) before
commit. Everything below is fixed and re-verified (33-check suite, build
clean); nothing ran on hardware.

**The two serious ones:**
- **The shared detector never published a detection on this machine.**
  `robot_camera_node._orientation` calls `R.from_matrix` — absent in this
  PC's scipy 1.3.3 — and the per-frame `except` swallowed the
  AttributeError, so `/…/tag_detections` carried empty arrays for every
  frame WITH a tag. The entire refactored calibration path (and
  mobile_node's vision stop) depended on it. Compat fallbacks applied in
  three files; convention rewritten (see Coding Conventions — the ">=1.4
  or it won't even import" premise was empirically false).
- **`BaseInterface.goto()` auto-cleared the EMERGENCY stop latch** before
  every move — a mid-session e-stop would fail one entry and the next
  entry would drive the base again. Now: refuses to drive while
  emergency/estop is latched (deliberate clear required); only a
  leftover preempt latch is cleared.

**The rest:** move_cart completion could be claimed by a scan's
motion_seq bump (now attributed via result_message, with arm_node-restart
detection); stale-state guard on `/arm/state` (1 s freshness, previously
a dead arm_node served its last pose forever); pub-connection wait before
the first `move_cart` (rospy drops pre-connection publishes);
`align._target_T_cam2tag` servoed the camera yaw back to zero, undoing
the planner's flange-reach optimization — it now preserves the current
spin and corrects only tilt (regression-tested); `retry_count` was
parsed-but-ignored (now a real retry loop); `run_calibration` gained a
one-session lock + `~cancel_calibration` (cooperative, stops after the
current entry); `align_required: false` moves now go through the same
initial-step clamp as the align path; `wait_for_tag_detection` uses one
persistent subscriber per call instead of churning `wait_for_message`;
dry runs publish per-tag progress; map_calibrator reads its own private
locator params before the global copy; README/USAGE_kr purged of
pre-refactor instructions (the worst told operators to `rosnode kill
/task_executor` before a session).

### 2026-09-01 — path_tag_locator refactored into the main stack; standalone mode removed

Two passes in one day; the first is kept here because its verification is
what makes the second safe.

**Pass 1 — un-quarantine (rotation port + new map, together, as the 🛑
headers demanded).** The `nav/robot_controller.py` copy got the three
already-verified `mobile_controller` fixes (`lateral = tag['y']`,
`center_x_stop_offset: +50.0` with inverted comparisons, the
`tag_edge_angle_deg()` −90° helper); `map.yaml` became a verbatim copy of the
new 72-tag cell map. Verified offline (16 checks, `verify_ptl_port.py`):
helper parity over 200 random rotations, `calculate_robot_pose` parity vs
`mobile_controller` over all 72 tags, dock 500 reproducing the design
record's **−1.9123**, stop condition monotone. Also fixed while there:
dock 508 → 500 in `calibration_plan.yaml`/README/USAGE_kr, `tag_b_size_m`
0.090.

**Pass 2 — full refactor: the package now runs inside
`mobile_manipulator.launch` and owns no hardware.** Deleted: the whole
`nav/` tree (+`robot_nav.yaml`) — base nav goes through `mobile_node` via
`MobileClient` (`base_interface.py`); `tcp_pose.py` (the second Fairino RPC
connection) — arm moves go through `arm_node`'s `/arm/move_cart` with
`motion_seq` completion (`arm_interface.py`, duck-type identical, so
`align_runner` only changed its detection source); the in-package `map.yaml`
copy — `map_calibrator.yaml` now points at `$(find apriltag_nav)`'s map. The
three old launch files became ONE `path_tag_locator.launch` (handeye node
behind `use_handeye_calib`, default false), kept separate from
`mobile_manipulator.launch` by user decision — run it alongside the main
stack, which must already be up.
Tag observations now come from `robot_camera_node`'s detections topics (one
detector, many consumers) with per-camera size rescaling — pose_t is linear
in tag size, so odd-sized ref tags can be recovered from the detector's
configured size by scaling t; `robot.yaml` `robot_camera.tag_size.hand_cam`
was first set to 0.06, then **same day the user switched the (six) reference
tags to 90 mm** — hand_cam, `tag_a_size_m` and the handeye tag are all 0.09
now, scale factor 1; `reference_tags.yaml` still holds example poses and
needs the six real measurements. `handeye_calib_node`
deliberately keeps grabbing RAW hand-cam frames (calibrateHandEye re-detects
over archived samples) but reads TCP pose from `/arm/state`.

**Two traps found on the way:**
- `robot_camera_node` encodes euler as `as_euler('zyx')[::-1]` — scipy
  LOWERCASE 'zyx' (extrinsic), despite its own docstring saying
  "ZYX-intrinsic". Reconstructing with `geometry.rpy_deg_to_R` (Rz·Ry·Rx
  intrinsic, the Fairino TCP convention) is wrong at large angles; the exact
  inverse is `from_euler('zyx', [yaw, pitch, roll])`. `detections.py`
  documents this; a 300-random-pose round-trip test pins it.
- This machine runs **scipy 1.3.3** (no `as_matrix`), while the merged stack
  assumes ≥ 1.4 — `robot_camera_node`/`arm_transform`/`arm_controller` would
  AttributeError at runtime here. Flagged as a separate task; `detections.py`
  uses an `as_dcm` fallback so this package works on both.

**Lift compensation (same day, user request):** `T_ab2mb` is measured with
the lift at origin, so a raised lift used to shift every chain result by the
lift height silently — the locate/calibrate twin of the main stack's
"lift breaks arm_base_z" problem. `chain.compensate_T_ab2mb()` now subtracts
the live `/lifter/height` (via `lift_listener.py`) from `T_ab2mb`'s t_z in
both nodes AND in the auto-view-pose bootstrap; the height is logged and
persisted with each run. If lifter_node is down the listener warns and
assumes origin (the old behaviour). The main stack's pose-mode IK remains
deliberately uncompensated — this fixes only path_tag_locator's chain.
extrinsics.yaml must STAY lift-at-origin (comment added there).

The intended workflow is a FIXED height per session:
`map_calibrator.yaml lift_height_mm` (null = leave the lift alone) makes
`run_calibration` position the lift there first — home-then-single-up-stroke
by default (`lift_home_first: true`), because the driver guide documents
~1000–1800 counts of encoder error after an up-then-down stroke, so home+up
is the only reliable route to a known height. The orchestrator then guards
the session: >5 mm drift from the session-start height logs a loud warning
(compensation follows the live value, so drift implies the ENCODER is lying
— re-home and re-run affected entries).

**Per-plate sessions (user requirement):** both plates carry cross tags
with the SAME ids 0-5, so map calibration runs as TWO sessions —
`calibration_plan_plate1.yaml` (all 26 B+C work tags) with
`reference_tags.yaml`, and `calibration_plan_plate2.yaml` (all 25 D+E)
with `reference_tags_plate2.yaml` (= plate 1 poses + 3.420 m x, the
row-by-row-verified offset between the two design CSVs). Plan and ref
file swap TOGETHER — the wrong pairing shifts every result by 3.420 m.
map_calibrator.yaml defaults to the plate-1 pair; plate 2 goes through
the service request's `plan_path`/`ref_tags_path` or robot_ui's
`map_calibration` script (`PLATE = 2`).

All design artifacts come from ONE generator,
`path_tag_locator/scripts/generate_calibration_artifacts.py` (takes
`--lift-mm` / `--view-m`; re-run it, don't hand-edit): both plans — each
entry paired nearest-cross-tag-by-y AND carrying a design
`arm_view_tcp_mm_deg` (map.yaml stop pose + extrinsics + hand-eye, so
sessions are deterministic; the entry override beats the bootstrap, and
run_auto_align still refines) — plus `docs/all_tags_position.csv`
(78 tags: positions, orientations, ref pairing, stop pose, view TCP,
ab-frame reach).

⚠️ Two-frame trap, caught by the user: the "2번정반 중심" design CSV's
origin is NOT 정반 2's geometric centre. D(−1.24)/E(+2.18) are asymmetric
about that origin; with the 0.45 m lane-to-face rule and the 2.52 m plate
width, the plate spans world 2.63..5.15 with its centre at **+3.890** —
0.47 m east of the CSV origin (+3.420). The user confirmed the plate-2
cross tags are laid about the GEOMETRIC centre, so
`reference_tags_plate2.yaml` = 정반 1 poses **+3.890** x (an earlier
+3.420 version briefly existed — never calibrate against it).

Reach: RESOLVED — all 51 pairs fit. Two corrections got there. (1) The
plate-2 frame fix above (D/E mirror B/C, 1.06–1.76 m TCP distance).
(2) User-pointed: reach is a FLANGE constraint, and the vision_tip tool
link (set_tool_tcp.py: flange→tip (0, −253, +225.2) mm, ≈339 mm) extends
the TCP well beyond it — AND the camera yaw about the tag normal is a
FREE parameter (align converges on xy+tilt, never yaw), so the planner
sweeps it (5° steps) and keeps the pose whose flange sits closest to the
arm base; both the hand-eye lever and the tool overhang swing toward the
base. Result: flange reach 0.86–1.315 m, 0/51 over the 1.40 m FR10
nominal, worst margin 85 mm. Caveat: Euclidean flange distance is not
full IK feasibility (wrist config, joint limits) — entries near the
margin can still fail IK at run time and are skipped per-entry, not
fatally. Plans carry the chosen cam yaw per entry; the CSV now has
`reach_tcp_m` + `reach_flange_m`.

**robot_ui integration (same day):** the calibration workflow is drivable
from the UI. `RosBridge` gained `locate_path_tag` / `run_map_calibration` /
`handeye_{capture,compute,status}` (blocking — worker-thread rule as ever;
`path_tag_locator.srv` is lazy-imported so the UI still starts without the
package built) plus a `calib_progress` signal fed by
`/map_calibrator/progress`, which MainWindow renders as `[calib] tag N: OK
x= y=` log lines. Two hot-reloadable operator scripts under `plugins/`:
`map_calibration.py` (DRY_RUN constant, defaults true) and `locate_tag.py`
(TAG_B_ID / AUTO_ALIGN / INITIAL_TCP constants). Caveat written into the
script header: `run_map_calibration` is ONE blocking service call, so the
Stop button cannot interrupt a live session — STOP ALL fails the current
entry instead. The calibration nodes stay out of the UI's launch; they must
be running from `path_tag_locator.launch`, else the bridge methods return a
clean wait_for_service timeout.

**Verified offline** (33 checks, `verify_ptl_refactor.py`): all imports,
deleted modules unimportable, config loaders, detector-size invariants vs
robot.yaml, euler round-trip, chain math vs hand-built ground truth,
image-less persistence, launch XML with the three nodes, orchestrator
dry-run construction. `catkin_make` clean. **Nothing ran on hardware** — the
first live calibration session still needs: arm_node's `move_cart` behaviour
under the align loop, detections latency vs the old direct grabs, and the
MobileClient single-commander discipline (no TASK/GOTO mid-session).

### 2026-08-24 — Zone B is drivable; line1 lift → 300 mm; the scan still is not

The user has physically installed the first **16 tags of the new cell** —
**100–112, 500, 501, 505**, all as 90 mm — and asked whether
`scan_joints_line1` could be run. **Navigation yes, scan no**, and the two
halves of that answer are worth keeping separate.

**Zone B routes end to end on the installed tags alone.** Checked with the real
`MapManager`, not by eye: `500→105` is `[500, 501, 505, 112, 111, 110, 109,
108, 107, 106, 105]` and `500→106` the same minus the last hop, with the return
leg symmetric — **no leg touches an uninstalled tag**. The `501→400` edge
exists but BFS never takes it. Config agrees: `START_TAG` 500, `camera_offset`
0.55, front_cam `tag_size` 0.09. So `GOTO 501 / 505 / 112 / 105 / 500` exercises
the new map, the 90 mm tags, the 501↔505 pivot pair and zone B's **reverse**
entry with the arm parked — which is exactly what these 16 tags unlock.

**The scan is a collision path and the reason is arithmetic, not calibration.**
Promoted into the scan-CSV blocker section rather than left here: the old joint
angles assume `arm_base_z` 1.025, the new base is 0.652, and the entire lift
stroke (343.35 mm) is **less than the 373 mm drop**. There is no `lift_height`
that fixes an old joint CSV. `_exec_joint` is a bare `MoveJ` with no
reachability or collision check and the Keyence loop only runs after the move
lands, so nothing downstream catches it.

**`lift_height` on line1: 150 → 350 → 300, on the user's instruction, line2
deliberately untouched.** 350 was asked for first and backed out after checking
what it does: it is past `soft_max_counts` (6900 = 343.35 mm), and the overshoot
does **not** clamp-and-continue — `LiftClient._verify` compares reached against
requested at `tol_mm` 1.5, so 350 lands at 343.35, misses by 6.65 and aborts the
task at the lift step with `state = ERROR`. 300 mm = 6029 counts, well inside
the clamp; arm base 652 + 300 = **952 mm**. The 343 ceiling is now documented at
the `lift_height` section, since nothing in the config hints that exceeding it
is fatal rather than saturating.

⚠️ **`scan_full_joints` is unregistered as a result**, accepted by the user. It
concatenates both line CSVs and `_extract_lift_height` refuses disagreeing
values (`[150.0, 300.0]`). Confirmed by reloading the real `TaskManager`: the
other five tasks register normally, `scan_joints_line1` at 300.0 and
`scan_joints_line2` at 150.0. Setting line2 to 300 restores it — but every
joint CSV is being re-solved anyway, so it was not worth touching a file the
user asked to leave alone.

**Verified offline; nothing ran on hardware.** Real `MapManager` for the route,
real `TaskManager` for registration, and the clamp/tolerance arithmetic read
out of `lifter_node._clamp` and `LiftClient._verify` rather than assumed. The
workspace was being moved to the robot PC at the end of this session and
`catkin_make` had not been run — no message definitions changed, so only the
unbuilt `devel/` needs it.

**Also this session, outside the workspace:** the cell design record (the parent
directory's CLAUDE.md) had its **Z datum stated three inconsistent ways** — §1
read as floor-datum, §2 defines 정반-top as FL ±0, §8 quoted plate-face tags at
"z 25…55" from the floor. The two conventions are exactly 80 mm apart. §1 now
declares the cell datum and names the two deliberate exceptions, §3's mount
table is labelled robot-frame with the −80 conversion, and §8 carries both
values. Two derived CSVs were generated at the repo root
(`tags_plate1_frame.csv` / `tags_plate2_frame.csv`, 72 tags each, z = −80,
plate2 = plate1 − (3890, 0)) with `make_plate_frame_csvs.py` to regenerate them
after calibration.

### 2026-08-23 — `arm_base_z` 0.651 → 0.652, and the derived top of stroke

One millimetre, on the user's instruction ("원점에서 arm base 높이는 652 mm 로
바꿔줘"), to make the workspace agree with the **cell design record** — the
parent directory's CLAUDE.md §3 mounts table lists the manipulator base at
z = **652** in the robot frame. Nothing was re-measured; this is the workspace
adopting the drawing's number, and the 1 mm against the 2026-08-13 tape
measurement is not reconciled. It is recorded that way at the config key rather
than presented as a new measurement.

**Three sites have to agree and were changed together**, which is the only part
of this that is not bookkeeping: `robot.yaml` `arm_calibration.arm_base_z`, the
hardcoded fallback in `arm_transform.py` (used when neither the private ROS
param nor the YAML block resolves), and `T_ab2mb` tz in
`path_tag_locator/config/extrinsics.yaml`, which is the file both of the others
name as the source of truth. The invariant `extrinsics tz == −arm_base_z` is
what ties them, and it is asserted in the verification below.

**The top of the stroke follows and is now ~995 mm**, not 994: 652 + 6900 ×
0.04976077 = **995.35 mm**. The old 994 came from 651 + 343.2, i.e. the gauge
reading at count 6897 rather than the clamp — both are defensible to the
nearest millimetre, so the arithmetic is now stated explicitly wherever the
figure appears instead of the figure alone. The joint-mode scan height moved
with it: 652 + 150 = **802 mm**.

**Scope of the actual behaviour change is one line of geometry.**
`arm_base_z` enters only `p_A_W[2]` in `transform_world_to_arm`, called from
exactly one place (`arm_controller._exec_pose`), so every **pose-mode** target
shifts 1 mm in the arm frame and **joint mode is untouched** — those CSVs are
absolute joint angles that no transform reads. Well inside any residual this
model has ever been validated to; the reason to do it at all is that three
files disagreeing with the drawing is how the *next* discrepancy gets blamed on
the wrong number.

**Four stale figures were fixed in the same pass, all pre-existing.** This
workspace copy never received the 2026-08-14 documentation sweep, so
`docs/lift_arm_base_z_analysis.md`, `docs/architecture_slides_kr.md` and
`README.md` were still quoting the *retired* 341 mm / 7000 count / 0.0487143
scale and a 992 mm top. They now carry 343.2 mm at count 6897 / 6900 clamp /
0.04976077, and `lift_height: 150` reads 3014 counts (was 3079). Also corrected
while in the files: `extrinsics.yaml` said the lift adds "341 mm" (→ 343) and
"the 97 mm is the new mount" (→ 100, stale from before `camera_offset` went to
0.55), and `robot.yaml` wrote `T_ab2mb` t as `(0, 0, -0.651)` — **missing its
y term entirely** since the 2026-08-13 arm relocation. Now `(0, -0.100, -0.652)`.

Work Log entries below citing 651 / 994 / 341 are left alone; they are a record
of what was true then. `docs/GUIDE_kr.md` untouched per the standing rule — the
two affected Deferred rows were updated instead.

**`task/csv/joints_line1_lift.csv` deleted**, on the user's call. The
2026-08-13 entry below says it was already gone; it survived in this checkout
and nothing loaded it — the only remaining references were that entry, one
comment in `task_manager.py` and a Deferred row. Confirmed dead before
deleting, and confirmed it held no unique data: parsed against
`optimized_joints_line1.csv` it is **328 rows, all 10 columns, zero value
differences**, `lift_height` 150.0 on both. That is the same row-by-row check
the 2026-08-13 entry describes, re-run rather than trusted.

**Verified offline; nothing ran on hardware.** 14 checks: all three YAMLs and
the launch XML parse; `arm_base_z` == 0.652 and `extrinsics T_ab2mb` tz ==
−0.652 exactly; `T_ab2mb` ty == `arm_body_offset_y` and its R is exactly
diag(−1,−1,1) with det +1; `arm_transform.py`'s two hardcoded fallbacks match
the YAML; `T_mb2fc` tx still == `camera_offset` (0.55, untouched); the top of
stroke rounds to 995 and the scan height to 802; `mm_per_count` == 343.2/6897
and `soft_max_counts` == 6900 in **both** `robot.yaml` and the launch file;
`calibrate_transform.py`'s `arm_base_z` bound (0.55–1.20) still brackets 0.652,
so a future fit cannot be silently clamped. The four touched Python files
byte-compile.

### 2026-08-21 — The cell was replaced: new `map.yaml`, new origin, 90 mm tags

The user replaced the polishing cell and asked for the new `map.yaml` (72 tags,
142 edges, origin at **the centre of 정반 1**) to be swapped in, then for
everything *outside* the map that the swap breaks to be found and fixed. The
design record for the new cell lives in the parent directory's CLAUDE.md; this
entry records only what changed in the workspace and why.

Old map kept as `config/map.yaml.bak` — it was uncommitted when overwritten.

**The swap itself is the least interesting part. What breaks is everything that
held a tag ID or a tag size as a constant**, and all of it still *parses*, which
is what made a careful sweep necessary rather than a compile.

**`TaskManager.START_TAG` 508 → 500 is the one that would have caused motion.**
508 exists in the new map — it is a zone-E pivot tag at the far end of the cell
— so `TASK go_home` would have routed successfully and driven ~7 m the wrong
way with nothing to report. `tools/navigate.py` held the same literal in three
places; it now has a single `DOCK_TAG = 500` constant with a comment naming
`START_TAG` and the map's `DOCK` entry as the things it must agree with.

**`tag_size` had to become per-camera, and that is a real code change.**
`dt_apriltags` scales `pose_t` linearly by this number, so a wrong value
corrupts `tag['z']` (which the 6 cm final-approach trigger multiplies by) and
`tag['y']` (`lateral`, which feeds `/robot_pose`). The new cell uses **90 mm**
floor tags and **30 mm** tags on the 정반 step, and one global value cannot
serve both. `robot_camera.tag_size` is now a dict keyed by camera; a `null`
entry falls back to `robot.tag_size` (0.09). `robot_camera_node.py` is the sole
reader — confirmed by grep across the workspace before changing it — so the
change is contained to one file.

⚠️ **`camera_offset` 0.547 → 0.55, and the 3 mm is NOT reconciled.** 0.547 was
measured on the base 2026-08-13. The new map was *generated* from 0.550 — every
tag sits at `stop pose + 0.55 × heading`. They cannot both be right, and the
map won for a checkable reason rather than a preference: running the real
`calculate_robot_pose` over all 72 tags reproduces the design record's
documented dock stop of **x = −1.9123** at 0.55 and gives **−1.9093** at 0.547.
The map and the code have to agree with each other before either agrees with a
tape measure. Flagged at all three sites (`robot.yaml`, `extrinsics.yaml`, the
Transform Parameters section above) with the same warning: **if 547 is the true
lens position, move the tags, not this key** — changing it alone shifts every
derived stop pose off the tag grid.

`extrinsics.yaml` `T_mb2fc` tx followed to 0.55, because its own comment
declares the invariant `tx == robot.yaml camera_offset` and `robot.yaml` points
at that file as the source of truth. That is the only edit made inside
`path_tag_locator` — see below.

**`wall_dist_zone_a` 0.6275 → 0.52**; the new zone A lane (y = 3.02) is 520 mm
from 정반 1's top edge. `wall_dist_work_zone` stayed **0.45** — the new cell's
corridor lanes happen to sit exactly 450 from the plate face, so the number
survived the swap by coincidence, not by being maintained. Nothing reads either
key.

**`mobile_controller.py` needed no change**, which was worth checking rather
than assuming. Its `zone` → heading transform (`A`/`DOCK` → 0, `B`/`D` → +90,
`C`/`E` → −90) already matches the convention the new map was written to.

**Verified offline; nothing ran on hardware.**

- **The whole map, through the real code.** `calculate_robot_pose` over all 72
  tags gives `stop pose == tag − 0.55 × heading` with **zero mismatches**. Dock
  at (−1.9123, 3.02) facing east; lanes B/C/D/E at x −1.71 / 1.71 / 2.18 /
  5.60; every scan station on the 400 mm grid with the mandatory 0 / ±1.2 stops
  present in all four zones; zone A transit all at y 3.02; zone D holding 12
  tags topping out at y 2.00, which is the documented design compromise and not
  a generation error.
- **The task layer.** `START_TAG` 500 resolves to the `DOCK` entry, and all
  seven tasks route: `go_home` `[500]`, `scan_full_joints` `[105, 106, 117,
  118]` at lift 150 mm, the three pose tasks at lift `None`.
- **Parsing / compiling.** Both `map.yaml`s, `robot.yaml` and `extrinsics.yaml`
  parse; the launch XML parses; every touched Python file compiles; the
  `extrinsics tx == camera_offset` invariant holds at 0.55.

⚠️ **`path_tag_locator` was deliberately NOT ported, and its `map.yaml` is now
actively dangerous.** It carries a full copy of the old cell whose IDs all still
resolve — 105 exists there and in the real map, ~2.3 m apart. A 🛑 header was
added saying so. The package was already quarantined for an unrelated reason
(its `nav/robot_controller.py` was never updated for the 2026-08-13 front_cam
−90° rotation), and porting the map alone would make it *look* current while
still steering on the wrong image axes. Both have to happen together, and that
is a user decision, not a sweep.

**Four things could not be fixed here** — all recorded above in the scan-CSV
blocker section:

1. **Every scan CSV's joint angles were solved for the old cell's geometry.**
   No transform reads them, so they cannot be corrected — they have to be
   re-solved. Pose mode is recoverable in principle but its grid CSVs still
   describe the old workpiece position.
2. `grid_path_line{1,2}_-5.csv` use `group_id` **4 and 5**, valid in neither
   map. Pre-existing, not caused by the swap.
3. `path_tag_locator`, above.
4. The 3 mm `camera_offset`, above.

### 2026-08-19 — `apt autoremove` deleted a real realsense runtime dep

Docs-only; no workspace files changed except this one and a system package
reinstall. First bring-up attempt of the day: `roslaunch apriltag_nav
mobile_manipulator.launch` came up with `arm_node` and everything else fine,
but `side_cam/realsense2_camera` and `hand_cam/realsense2_camera` both died at
launch — `Could not load library
/home/abc/mobile_manipulator_ws_20260814/mobile_manipulator_ws/devel/lib//librealsense2_camera.so
... Poco exception = libddynamic_reconfigure.so: cannot open shared object
file`.

**Traced to the user's own `sudo apt autoremove` run minutes earlier.**
`apt-cache policy ros-noetic-ddynamic-reconfigure` showed it not installed but
available, and `/var/log/apt/history.log` had both ends of the story: it was
pulled in "automatic" on 2026-08-05 as a dependency of a one-time
`apt install ros-noetic-realsense2-camera` (the full apt package this
workspace's Tech Stack section says never to install on purpose — but that
install still ran once and dragged its deps down with it), and never
re-flagged manual afterward. `autoremove` on 2026-08-19 correctly-by-its-own-
logic saw no reverse dependency (the devel-space `.so` link is invisible to
apt) and deleted it, along with an unrelated pile of vlc packages in the same
run. front_cam (Orbbec) doesn't link this library and came up fine, which is
what made this a two-camera failure and not a three-camera one — worth
knowing as a diagnostic signature if it recurs.

**Fix:** `sudo apt install ros-noetic-ddynamic-reconfigure`, confirmed with
`ldconfig -p | grep ddynamic`. Promoted to a standing ⚠️ under Tech Stack /
RealSense, including the `apt-mark manual` step to stop it from re-marking
itself automatic and repeating this on the next `autoremove`.

Also asked this session: what the repeated `Error, more than one new minimum
found.` lines in the launch log are. Traced by `strings` on
`~/.local/lib/python3.8/site-packages/dt_apriltags/libapriltag.so` — it's an
internal diagnostic from the AprilTag C library's quad-fitting step (gradient
histogram peak-finding when splitting a candidate quad's boundary into 4
lines), not from any code in this repo. Benign: it means one candidate quad
was ambiguous on one frame, not that detection has stopped working. Not worth
suppressing; `rostopic hz /front_cam/tag_detections` is the actual check for
whether detection is healthy.

### 2026-08-14 — On the robot: final approach works; the lift needed a re-home

First hardware run of everything above, `TASK scan_joints_line1` on the real
machine. **Navigation and the final approach worked on the first try. The lift
did not, and the reason is a trap worth knowing.**

#### The final approach, measured on hardware

Three drives to tag 105, from `mobile_node`'s log:

```
17:36:36  Final approach: 5.9 cm to the stop point at 0.020 m/s -> 0.010 m/s, decelerating at 0.0025 m/s2
17:50:35  Final approach: 5.8 cm to the stop point at 0.049 m/s -> 0.010 m/s, decelerating at 0.0194 m/s2
17:59:41  Final approach: 5.9 cm to the stop point at 0.048 m/s -> 0.010 m/s, decelerating at 0.0187 m/s2
```

**5.8–5.9 cm against the configured 6 cm**, three times. That number is what
retires the open `fx` question: the trigger is `diff * tag_z / fx`, so a real
`fx` far from the sim's 1000 px stand-in would have moved it. Entry speeds
0.020–0.049 m/s and solved decelerations 0.0025–0.019 m/s2 — all well under
`linear_accel`, i.e. the gentle end of the range, because the odom profile has
already slowed the robot by the time the zone is entered. The high-decel case
(0.0825 at a full 0.1 m/s entry) has still never happened on hardware.

Arrivals: `traveled:0.556m` on the first drive and `0.400m` / `0.403m` on the
next two, from the same tag pair — the first started further back.

#### 🛑 `/lift/homed` can read True while the drive still needs re-homing

The lift did nothing for 60 s:

```
17:36:42  [TASK] Setting lift to 150.0 mm before scanning
17:36:42  lift_driver: position command -> target=3014 counts   <- driver's OWN log
          (60 s of nothing: no alarm, no error, status=OK, pos stuck at 0)
17:37:42  [BaseLifter] lift move timeout after 60s (pos=0, target=3014)
```

Everything above the drive was correct — the command traversed
`task_executor → LiftClient → /lifter/height_cmd → lifter_node →
/lift/position_cmd → lift_driver` and the driver logged receiving it. And
`/lift/homed` read **True**, with a completed homing in the driver's journal
53 minutes earlier (a real 103.6 s descent from 6897 to 0).

**The fix was to home it again**, and the tell is that the re-home was a
no-op: `homing: lower limit switch reached after 0.1s, start=0 end=0
(moved +0)`. Nothing moved. Yet the identical command then worked:

```
17:49:58  homing started (from pos=0) -> lower limit reached after 0.1s, moved +0
17:50:39  position command -> target=3014 counts
17:50:51  reached target (target=3014, pos=3002)      <- 11.6 s
```

So `homed=1` is not sufficient to conclude the drive will accept an absolute
move; re-issuing homing re-establishes something inside the drive that the flag
does not track. **The failure signature to recognise:** the driver logs
`position command -> target=N counts`, the position never changes, and there is
no alarm and no error — `status=OK` throughout. That combination means home it
again, not chase the ROS layer. A zero-motion re-home costs 0.1 s.

Once past it the whole chain ran: arrived at 105 → `lift at 149.9 mm; holding
for the rest of the task` → `[ArmClient] Sent 169 scan point(s)`. The user
stopped it there rather than running all 169 points.

Three smaller things from the same journal:

- **`task_executor` handled the failure correctly** — `Lift height 150.0 mm not
  reached — lift ended at 0.0 mm` and it aborted before scanning. The arm not
  moving was the designed behaviour, not a second fault.
- **Origin drift is real and small.** A later homing from 3009 ended at
  `pos=-15` — ~0.7 mm of drift over one up-down cycle, consistent with the
  incremental-count warning in the lift section.
- **This also closes item 1 of the 2026-08-14 plan** — the front_cam rotation
  port is now exercised on hardware. Three drives arrived on the tag, so the
  steering sign, the `center_x` stop test and `tag_edge_angle_deg` are all
  right way round; they had only ever been checked offline.

⚠️ **Item 2 of that plan is still open**: neither the `align_to_tag` timeout nor
the odom-stall check has been written. `align_to_tag` did **not** hang on any of
the three drives — it took ~2.2 s between `Arrived at 105` and the client's
`arrived at tag 105` — but the loop still has no timeout, so this is evidence
that the case is survivable in practice, not that the defect is gone.

### 2026-08-14 — Lift re-measured: `soft_max` 7000 → 6900, scale → 0.04976077

User's numbers off the robot: cap the top of travel at **6900 counts**, and the
gauge read **343.2 mm at count 6897**. Config + node + bench UI updated to
match. No code logic changed — every consumer already read these from
`robot.yaml`.

**The scale is derived from 6897, not 6900, and that is the one real decision
here.** 6897 is where the drive settled when commanded to 6900 — 3 counts
inside `lift_settle_tol` (20) — so it is the count the 343.2 mm was actually
measured at. Dividing by the commanded 6900 instead would bake the settle error
into every conversion the system ever does. `lifter_node` now carries the
measured pair (`CALIB_COUNTS` / `CALIB_MM`) and derives `MM_PER_COUNT` from it,
with `STROKE_COUNTS = 6900` kept separate as the clamp ceiling — they are two
different facts and pairing them in one constant is what let the old
341 mm/7000 figure drift.

**0.0487143 → 0.04976077 is +2.1%, i.e. 7.2 mm over the full stroke.** Not a
rounding change. Consequence worth watching: `lift_height: 150` now converts to
**3014 counts** where it used to be 3079, so **joint-mode scans sit 3.2 mm
lower than every run before today** — the old commanded 150 mm was really
153.2 mm of travel. The scale is the thing that was wrong, but those CSVs hold
absolute joint angles solved at one base height, so it is flagged in the
`lift_height` section above rather than left to be rediscovered.

`soft_max_counts` had to change in **both** `robot.yaml` and
`mobile_manipulator.launch` — the launch sets it as a `~param`, which wins, so
a config-only edit would have been silently inert. Same trap as the 6700 → 7000
change on 2026-08-13; now warned about at the config key.

`tools/lift_calib_ui.py` needed only its fallbacks and docstring: it reads
`soft_max_counts` / `mm_per_count` from `robot.yaml` at startup and drives the
target spinbox, the sweep planner and the fit report off them. Verified
offscreen — the window comes up with a 0..6900 target range and reports
"= 343.3 mm at the current scale (0.04976077 mm/count)" at the top.

Derived figures corrected in the same pass: arm base tops out at **~994 mm**
(was 992), full stroke **~343 mm / ~27.8 s** (was 341 / 28.2), `jog_timeout_s`
margin **~7.2 s**, the `robot_ui` lift spinbox range **343.3 mm**, the
`arm_base_z` fit bound comment in `calibrate_transform.py`, `extrinsics.yaml`,
`README.md`, `docs/lift_arm_base_z_analysis.md` §Magnitude / §4.2 / §6 and
`docs/architecture_slides_kr.md`. `docs/GUIDE_kr.md` was left alone per the
standing rule and two rows were added to the Deferred table instead.

**Verified offline only** — `robot.yaml` / `extrinsics.yaml` parse, the launch
XML parses, the four touched Python files compile, and the node's derived
`MM_PER_COUNT` equals `robot.yaml`'s `mm_per_count` while `STROKE_COUNTS`
equals both `soft_max_counts` values. **Nothing ran on hardware** — the lift
has not been moved through the new clamp, and 343.2 mm is taken on the user's
measurement.

### 2026-08-14 — Navigation final approach: latched creep at the motor's floor

The user's design, specified by them and implemented on their instruction
("지금바로 적용해줘") — so this DOES change `execute_pure_pursuit`, which the
entry below had deferred. Two new `robot.yaml` keys, both read by
`mobile_controller`.

**`min_drive_speed: 0.015` — an ABSOLUTE floor, not a ratio.** The user's
reason is a hardware one worth recording: 0.015 m/s is the speed matching the
drive's minimum usable rpm (~15 rpm at the motor, through 9:1 on 0.0825 m
wheels), and **a command below it may not turn the motor at all**. A command
the motor ignores does not read as "slow", it reads as a stall. This is also
why it is absolute: `s_curve_min_speed_factor` gives 0.02 m/s at the current
max 0.1 but only 0.01 m/s if `max_linear_speed` ever goes back to 0.05, i.e. a
ratio silently walks the floor under the hardware limit whenever the top speed
is retuned.

**`final_approach_dist: 0.06` / `final_approach_speed: 0.010` — one
deceleration stage, latched, and the distance holds at ANY top speed.** The
user's requirement: start decelerating 6 cm before the tag reaches the stop
point and be at 0.010 m/s there, "regardless of whether max_linear_speed is
0.05 or 0.1". An earlier two-stage proposal (0.03 m/s on first tag visibility,
then a second stage) was dropped — the visibility trigger is redundant once the
distance trigger exists.

Honouring a fixed distance from an arbitrary entry speed means the
deceleration cannot be configured, it has to be **solved at the latch**:

    a = (v_entry**2 - v_final**2) / (2 * remaining)
    v(remaining) = sqrt(v_final**2 + 2 * a * remaining)

Three things about this are load-bearing, and each was wrong first:

- **Use the solved `a`, not `max(linear_accel, a)`.** The solved value makes
  the curve pass through `v_entry` at the latch, so the reference is continuous
  with what is already being commanded. Clamping it up to `linear_accel`
  instead starts the curve above the current speed, forcing a hold-then-brake
  whose brake needs exactly `linear_accel` — equal to the limiter, so
  discretisation makes it lag forever. Measured: arrived at 0.015 m/s instead
  of 0.010.
- **`a` is allowed to exceed `linear_accel`**, because the distance is the
  fixed quantity. Entering at 0.1 m/s over 6 cm needs 0.0825 m/s2, 1.65x the
  normal ramp. `send_vel` grew a per-call `linear_accel` override for this.
- **The approach bypasses the slew limiter entirely** (`float('inf')`). Its
  reference is already a constant-deceleration curve starting at the current
  speed, so it is rate-bounded by construction; a finite limiter can only lag
  it.

⚠️ **The envelope must aim at where the stop ACTUALLY fires**, which is
`center_x_stop_tolerance` (10 px ≈ 3 mm) *before* `target_x`, not at `target_x`.
Aiming at the nominal point left the robot doing 0.024 m/s when the stop
tripped. `remaining_px = diff * move_dir_sign - stop_tolerance`.

Three implementation details that are the difference between working and not:

- **The trigger is derived from the SAME pixel quantity the stop test uses.**
  `diff = center_x - target_x`, converted with `tag['z'] / fx`. Using
  `pose_x` instead would be a second, independently-drifting estimate of where
  the tag is, and the two could disagree about the stop point.
  ⚠️ `dist_to_tag = tag['z']` is the ~0.30 m camera height, not a range — that
  is exactly why it is the right scale factor here and the wrong lookahead
  there.
- **The latch is not conditioned on `tag_visible`.** One dropped detection in
  the last 6 cm would otherwise release the robot back to profile speed.
  Cleared when the stop fires, at loop entry, and in `clear_stop_flag()`.
- **The cap is applied LAST, after the backup-stop branch.** Both the profile
  and `mobile_controller.py:654-657` write `speed` from `min_speed` (0.02), so
  a cap set earlier is silently overwritten by a slower-but-larger number.

**Verified offline** (`t_nav_approach.py`, 29 checks): the real
`execute_pure_pursuit` against a stubbed rospy, a unicycle plant and a
simulated front_cam whose `center_x` is inverted from the true remaining
distance, so the stop test and the trigger are exercised against one ground
truth. The distance requirement is checked by sweeping the entry speed:

| top speed | entered at | solved a | starts | arrives |
|---|---|---|---|---|
| 0.05 | 0.032 m/s | 0.008 | 5.9 cm | 0.0100 |
| 0.10 | 0.065 m/s | 0.034 | 5.9 cm | 0.0116 |
| 0.20 | 0.128 m/s | 0.139 | 5.6 cm | 0.0174 |

The residual above 0.010 is one loop tick of the solved deceleration
(`a * 0.05 s`) — the last command before the stop — and the test asserts that
bound rather than a round number. Deceleration is constant to within 5% (the
rest is discrete integration of `remaining`) and matches the solved `a`. Also
covered: the stop lands at 0.397 m of the 0.400 m target, a 0.15 s detection
dropout inside the zone changes nothing while the unlatched control arrives at
0.037 instead of 0.010, a stale latch is cleared at loop entry, and a missing
`CameraInfo` warns rather than silently skipping.

⚠️ **Documented limit, recorded as a test rather than hidden: this degrades
above ~0.15 m/s entry**, and the cause is the 20 Hz loop, not the maths. At
0.182 m/s one tick is 9 mm, so the latch cannot resolve 6 cm better than that
(it fired at 5.3 cm) and the final tick sits `a*dt` = 28 mm/s above target (it
arrived at 0.030). It still slows to a quarter of the entry speed and still
stops on the tag. Nothing in the current config gets near this — the profile
enters the zone at 0.065 m/s at `max_linear_speed` 0.1 — but raising the top
speed past ~0.2 would need a faster control loop, not a tuning change.

**Verified on hardware the same day** — see the run-log entry above. The
trigger fired at 5.8–5.9 cm against the configured 6 cm on three separate
drives, which is also what retires the `fx` caveat this entry used to carry:
the sim's 1000 px stand-in was close enough to the real calibration that the
trigger distance lands where it should.

#### The absolute speed floor was added and removed the same day

Worth keeping as a decision, because the reasoning generalises.

It went in as `min_drive_speed: 0.015`, clamping every commanded speed up to at
least the drive's supposed minimum usable rpm. Two problems surfaced within the
hour:

1. **`vw_drive.py` had its own, different floor** — a hardcoded
   `MIN_MOVE_MPS = 0.008`, under the number navigation was enforcing. Two
   hardcoded copies of one hardware limit, already disagreeing.
2. **The tool's ANGULAR floor was not an independent number and had been
   treated as one** — 1.5 deg/s, picked by eye. Pivoting in place runs each
   wheel at `w * wheel_separation/2`, so a linear floor maps to exactly one
   angular floor: `0.015 / 0.325 = 0.046 rad/s = 2.64 deg/s`, 76% higher than
   the guess. **Any per-axis speed limit should be checked by asking what it
   makes the WHEELS do.**

Then the premise itself failed. **No minimum rpm and no deadband is documented
anywhere in the navifra driver** — `param.yaml`, `base_controller.yaml` and
`motor_driver.yaml` were all read, and `base_controller.yaml` calls its physical
values "로봇 실측 확정값" while listing no such limit. And the 2026-08-12
hardware drive ran a 0.01 m/s floor (≈10.4 rpm at the motor) with the base
moving normally. So stiction here is a soft, unreliable region rather than a
cutoff, and 0.015 was an estimate.

**The user's call, on being shown that: remove it** ("최저속도의 제한이 없다면
그냥 네비게이션 절대 하한은 없애자"). Right call — clamping every command up to
an unmeasured number silently changes commanded motion to defend a limit nobody
has established, and it would have masked the very evidence needed to measure
the real one.

What survives is `final_approach_speed: 0.015`, **renamed from
`min_drive_speed`** because it is no longer a floor: it is the speed the final
approach holds, and nothing else is clamped to it. Leaving the old name on a key
that had stopped being a minimum was the more dangerous option. `vw_drive.py`
lost its floor entirely, which changes almost nothing — the braking envelope
only falls under 0.015 m/s inside the last 2.25 mm (`0.015**2 / 2a`), and
`ARRIVE_TOL_M` is 3 mm, so the loop has already handed off to the ramp-out.

If a stiction floor is ever **measured**, `robot.yaml` says where it goes and
why it belongs there as an absolute m/s value rather than a ratio.

### 2026-08-14 — The arrival judder: an S-curve that is quadratic in DISTANCE

Same session as the entry below, after the user drove the tool on the real
robot: it worked, but the base "울컥울컥" — juddered — into a stop instead of
decelerating smoothly, and they correctly guessed the profile rather than the
motors. `tools/vw_drive.py` is fixed. **The same defect is live in
`mobile_controller.execute_pure_pursuit` and is NOT fixed** — see the warning
at the end.

**The bug is a units mismatch between two control laws.** The profile
`_smooth_speed_factor` is a function of DISTANCE; the ramp limiter in
`send_vel` is a function of TIME. Copying the first onto the second gives two
failures that only show up at the end of a move:

1. **The decel curve never converges.** `v ∝ remaining**2` with
   `d(remaining)/dt = -v` integrates to `remaining(t) = 1/(kt+C)` — hyperbolic.
   It flattens onto the minimum speed and creeps there, and the move ends with
   `stop()` STEPPING the command from that speed to zero. `stop()` assigns
   `_cur_v = 0` and publishes, bypassing the very limiter that exists to
   prevent that step.
2. **The curve's steepest demand scales with the SQUARE of the top speed**
   (~`2*v_top**2/span`), so raising `max_linear_speed` 0.05 → 0.1 quadrupled
   it. Measured against the real code: 0.032 m/s2 at 0.05 (0.6x of the
   `linear_accel` limit — never bound, which is why nobody had seen this) and
   0.125 m/s2 at 0.1 (**2.5x over** — the limiter saturates, so the real speed
   runs above the plan for the whole braking phase and the plan answers by
   demanding an even steeper drop). The floor speed doubled with it, making
   the terminal step twice as big.

**The fix is the braking curve itself:** `v = min(v_top, sqrt(2*a*remaining))`.
Constant deceleration, i.e. velocity linear in time. It needs no phase ratios
— cruise and brake fall out of the `min`, and a move too short to reach
`v_top` becomes a triangle without a special case — so `s_curve_accel_ratio` /
`s_curve_decel_ratio` / `s_curve_min_speed_factor` are no longer read here.

Two details that are the difference between "smoother" and "correct":

- **Hand off to the ramp-out on BRAKING DISTANCE, not on remaining distance.**
  A speed on the envelope satisfies `v**2/(2a) == remaining`, so leaving when
  `remaining <= _cur_v**2/(2a)` lands on target. Leaving at a fixed tolerance
  instead hands off at whatever speed the limiter happened to be at, which
  overshot by 2.5 mm when it was lagging above the envelope. Measured: +2.48 mm
  → **+0.00 mm**.
- **`_ramp_to_zero`, not `stop()`.** `stop()` remains right for an abort — an
  e-stop is no occasion for a graceful ramp — but ending a normal move with it
  is the jolt. Measured hand-off speed: **0.0201 m/s → 0.0000 m/s**.

⚠️ **A live dry-run then caught a hole the fix opened.** On a move shorter than
the braking distance (0.10 m at `v_top` 0.1), a frozen base lets the commanded
speed climb until its stopping distance covers the whole move, so the loop
hands off BEFORE `STALL_GRACE_S` and the stall check never fires: `forward
0.05` reported success having travelled 0.000 m. `_arrived` now verifies the
move actually happened. Note it cannot verify a move smaller than its own floor
tolerance — a 3 deg pivot against a 3 deg tolerance is unverifiable, by
construction.

**Verified offline** (`t_vw.py`, now 106 checks): for 0.3 m / 1.0 m at both top
speeds and a 90 deg pivot — deceleration is constant to 1e-6 and equals the
accel limit exactly, the value handed to `stop()` is < 2e-3, there is no
re-acceleration after the peak, and the floor-speed creep is gone. Plus the
four new stalled-base cases. `trace_profile.py` reconstructs the OLD profile
and prints both traces side by side, which is what the numbers above come from.
**Nothing on hardware** — the user drove the previous version; this one has not
been driven.

⚠️ **Correction, same session:** the first version of this entry said
`execute_pure_pursuit` has "the identical profile". It does not, and the error
is worth keeping because it is easy to repeat. **`_smooth_speed_factor`
(`mobile_controller.py:524`) is defined and called by nothing** — dead code.
`execute_pure_pursuit` uses a plain LINEAR-in-distance ramp
(`mobile_controller.py:586-597`), which is milder than the tool's quadratic
one. Grep for the call site before assuming a helper is the code that runs.

The real navigation numbers, simulated against that linear ramp plus the actual
slew limiter (`nav_decel.py`). Peak demand is
`v_top * (v_top - v_floor) / (0.4 * D)`, so doubling the top speed quadruples
it:

| move | at 0.05 | at 0.1 |
|---|---|---|
| 1.00 m | 0.005 (0.10x) | 0.020 (0.40x) |
| 0.40 m | 0.013 (0.25x) | **0.050 (1.00x)** |
| 0.30 m | 0.017 (0.33x) | **0.066 (1.31x)** |
| 0.20 m | 0.025 (0.50x) | **0.098 (1.95x)**, `stop()` at 0.045 m/s |

The saturation threshold moved from "moves under 0.1 m" to **"moves under
0.4 m"** — and 0.40 m is the length of the only real drive on record.

### 2026-08-14 — `tools/lift_calib_ui.py`: bench UI for the count↔mm scale

User asked for a lift test UI where a **measured** value can be typed in to
check it against the count, then specified that it must run on the **navifra
driver alone** — `lifter_node` and the rest of the apriltag_nav stack stay
down. New standalone Qt tool in `tools/`; nothing else changed except this
file.

**So it writes `/lift/*` directly, through `NavifraDevices`, and that makes it
a second writer whenever the stack IS up.** Same hazard as `tools/navigate.py`
on `/cmd_vel`, so it takes `vw_drive.py`'s remedy: `rosnode` is checked for
`lifter_node` at startup and the tool refuses, `--force` overriding. The
guard query runs on a thread with a deadline, because a wedged master makes
`rosnode.get_node_names()` hang outright (seen the same day, below) and a
guard that can hang is worse than the collision it prevents; an unanswered
master downgrades to a warning rather than a false all-clear.

⚠️ **The soft travel clamp had to come with it.** `NavifraDevices` has none —
it publishes whatever it is handed — and the clamp normally lives in
`lifter_node`, which is the thing not running. The upper end has no limit
switch, so the tool clamps every target and every jog to
`lifter.soft_min_counts`..`soft_max_counts` itself and says so on screen. The
un-homed refusal still comes from the driver, which ignores absolute position
commands until homed, so `Go` stays disabled until `homed` and the relative
jog is the only pre-homing move.

**It moves in COUNTS, not mm, and that is the whole point.** The driver has no
millimetre command at all (`/lift/position_cmd` is `Int32`); the only one in
the system is `lifter_node`'s `/lifter/height_cmd`, which converts with
`lifter.mm_per_count` — the number being measured. A mm-commanded sweep would
test the scale against itself and hand back the constant it was given.

**Approach-from-below is back — for this tool only, default on.** The node's
rule was deleted on 2026-08-10 because the task flow cannot produce a descent.
A calibration sweep produces one the moment you step back to re-measure a
point, and it would fold the backlash straight into the fitted slope. So a
target below the current position homes first and climbs, every sample records
its approach direction, and the fit uses the ascending ones. Descending samples
are kept and **reported as the backlash** instead of being averaged in. The
post-homing origin sample is included by default with a checkbox to drop it —
it is the reference the count system is defined by, but the drive is resting
*down* on the limit switch there.

Output is `slope = mm_per_count`, intercept (the arm base height at the origin
when the gauge reads height above ground — 651 mm today), R²/rms/worst
residual, the implied full stroke against the configured 341.0 mm, a two-point
cross-check, and a paste-ready `robot.yaml` block. Samples save to CSV.

**Verified offline** (`t_lift_calib.py`, 83 checks, offscreen Qt, a fake
`NavifraDevices` and a synthetic sweep with a known slope): the fit recovers
0.0488 mm/count and a 651.0 mm intercept from ±0.1 mm reading noise and
reports the planted −1.35 mm backlash to 0.01 mm; over-travel clamps to 7000
and under-travel to 0 with the clamp named in the message; an un-homed
absolute never reaches the driver while an un-homed jog does; a jog at the
limit is a no-op; e-stop, alarm and a missing `/lift/position` each disable
the right buttons while an unconfirmed safety link only annotates; a
descending target homes first and is recorded as an ascent, unticking that
descends directly and tags the sample `down`; a hanging master leaves the
startup guard in 0.5 s reporting "unknown". Worth knowing for the real run:
**±0.1 mm of gauge noise over the full 7000-count span is already ~1.4e-5
mm/count of slope uncertainty** (~0.1 mm over the stroke), so a
tighter-looking answer than that is noise.

**Verified live against the real driver, read-only** — with `/lift/command`
under `rostopic echo` for the whole run to prove it: **zero messages**. The
driver's state parses (`position 0`, `homed false`, the raw
`NO_DATA mode=MANUAL …` alarm string), the startup guard clears in well under
a second with `lifter_node` absent, and the clamp answers 9999 -> 7000 and
-50 -> 0. Both gate states were exercised on the real machine: with the e-stop
latched and `homed=false` the window disables `Go` and homing while leaving
`STOP` and `Record` live, and once the PLC was reset and the bumper cleared
(`estop=false, homed=true, position 0`) it enables all four — with
`/lift/command` and `/lift/position_cmd` both still recording zero messages.
**No motion has ever been commanded through this tool.**

Two defects only the live run could produce, both fixed:

1. **It published a `stop` at startup.** `NavifraDevices` counts the FIRST
   `/safety/estop` message as an edge (prev `None` -> `True`), so with the
   e-stop already latched the handler fired before anything had been asked to
   move. The handler is now gated on a motion being in flight — `lifter_node`
   is unconditional and should stay that way, but a bench tool that has not
   been told to move must not write to the driver at all.
2. **`_on_estop` raised `no attribute '_busy'`.** That callback can fire
   *inside* `NavifraDevices.__init__`, which runs before the rest of
   `LiftBench.__init__`. `_busy` is now initialised before the devices object.
   Ordinary init-order, invisible to every offline test because the fake never
   called back during construction — the regression test now does.

⚠️ **rosmaster hit its 1024-fd limit after ~3.5 h and stopped answering
anything** — `rosnode list`, `rostopic echo` and `getPid` all hung, and the
master log went silent. Cause: **VS Code Remote auto-forwards port 11311**, and
its server process was holding **1042 connections** to it (1007 of the master's
1024 fds were those sockets; rosmaster had 1019 threads). Not a ROS fault and
not caused by any node here. Already-established TCPROS traffic between driver
nodes is unaffected — only new registrations and lookups block, which is why
the robot kept running while every tool hung. Two remedies: drop the 11311
forward in the VS Code PORTS panel (frees the fds with no restart and no
hardware motion), or restart `navifra-robot`. The user restarted, which cleared
it. Consider `LimitNOFILE` on the unit, and excluding 11311 from VS Code's
auto-forwarding, if it recurs.

**The restart did not origin-home the lift at first, and reading why took two
steps** — worth keeping, because `NO_DATA` plus `homed=0` looks like a driver
or RS485 fault and was neither. The driver's `auto_home_on_start` is true, but
`/safety/estop` came up latched, so motor power was cut: the lift sat at
`homed=false, position 0`, status `NO_DATA mode=MANUAL`. The PLC named two
independent causes, and they had to be cleared in order:

1. `traction_motor_power_on = 0` with STO 0/0 — the PLC latch. Pressing the
   RESET switch restored STO 1/1 and power, and the lift then origin-homed by
   itself (`connected mode=MANUAL homed=1 pos=0`).
2. `/safety/estop` nonetheless stayed **true**, because `safety_bumper_rear`
   still read 1 — pressed, for an a-contact bumper. It is an e-stop source in
   its own right, so reset alone does not clear it. It went to 0 when the
   bumper was physically released, and `/safety/estop` followed to false.

Both emergency buttons were idle (NC, `=1`) throughout and the front bumper
clear, so neither was ever the cause. A momentary bumper clears its own input
on release but the PLC latch does not follow it — that is why the order is
reset first, then the bumper, and why a single check of `/safety/estop` does
not tell you which of the two is still holding.

### 2026-08-14 — `tools/vw_drive.py`: manual (v, w) driving for bring-up

User asked for a debug script to drive the base by velocity — forward/back,
pivot turn, and relative moves given as a distance ("go 0.3 m ahead"). New
standalone tool in `tools/`; no existing file changed except this one.

**It is a second `/cmd_vel` publisher**, same as `tools/navigate.py`, so it
carries the same never-run-while-the-stack-is-up rule. Unlike `navigate.py` it
*enforces* it: `rosnode.get_node_names()` is checked for `mobile_node` at
startup and the tool refuses with an explanation. `--force` overrides for the
case where the node name is stale; `--dry-run` publishes nothing at all.

**Three behaviours came straight out of the 2026-08-12 hardware session**, and
they are the reason this exists instead of a `rostopic pub` one-liner:

- **20 Hz publishing.** `base_controller`'s `cmd_vel_timeout` is 0.5 s, so a
  single message is a twitch. (Read off the live master, not assumed.)
- **A stalled `/odom` aborts the move.** This is exactly the defect recorded
  below — with odom frozen the S-curve reads `traveled == 0`, pins the command
  at the floor speed and writes it to a dead drive for the whole timeout with
  no error. Here 2.5 s of commanded motion with no odom change stops and names
  `/motor/error` / `/motor/alarm`. A debug tool that reproduces the silent
  failure mode it is meant to diagnose would be worse than useless.
- **`/safety/estop` aborts**, both at entry and mid-move.

Speeds default to `robot.yaml`. The profile started as a copy of
`MobileController`'s S-curve and **was replaced the same day** — see the
judder entry below.

**Changing the speed is deliberately asymmetric.** A trailing argument on any
command goes slower (`forward 0.3 0.02`); going *faster* needs `--vmax` /
`--wmax`, which raise the ceiling and log a warning saying so. Those are
bounded by `base_controller`'s `max_linear_vel` / `max_angular_vel` read off
the param server (2.0 / 20) rather than by a limit invented in the tool —
past the driver's own clamp the tool would be reporting a velocity it never
sent. That is the only reason those 40x numbers appear here at all; they are
not a safety margin and nothing in this system has driven at them.

⚠️ **The one open-loop command is `vw <v> <w> <sec>`, and that is on purpose** —
it answers "does the base respond to a twist at all", which is the question you
have precisely when odom is what you suspect. Everything else closes on odom.
Pivot integrates wrapped yaw deltas rather than differencing against the start
angle, so `pivot 270` works instead of reading as −90.

**Verified offline** (`t_vw.py`, stubbed rospy + a unicycle plant, 63 checks):
0.30 / 1.0 / −0.30 / 0.05 m all land within 5 mm with the right sign and end
stopped; ±90 / 45 / 180 / 270 deg pivots land within 1.5 deg and translate
zero; peak speed reaches exactly 0.05 and a requested 99 m/s clamps to it;
e-stop refuses all three motions and mid-move e-stop stops short; a frozen odom
aborts in ~2.8 s rather than at the timeout; stale odom refuses; and the
command parser maps `back`/`right` to the correct signs.

**NOT verified on hardware — no motion has ever been commanded through this
tool.** Both live runs were `--dry-run`. The first one is worth recording
because it exercised a guard for real: `/safety/estop` was **latched ACTIVE**
with the emergency buttons reading idle (`safety_emergency_1b/2b = 1`) and
`traction_motor_power_on = 0`, which forces e-stop true per `safety_io.yaml`,
and the tool refused with that reason rather than commanding a base that could
not move. The circuit was reset partway through the session (STO 1/1,
traction power on, `hardware_estop=0`), so the base *can* drive now.

⚠️ **Battery fell 19.3% → 14.0% during the session**, well under
`low_battery_pct: 20.0`. Charge before the first real drive.

### 2026-08-13 — New base, part 3: arm moved 100 mm, front_cam rotated 90°

Same session as the two entries below. The user answered the wall-clearance
concern part 2 raised, and in answering it disclosed two more platform changes.
**Read this before running anything tomorrow** — one of them leaves navigation
in a known-broken state.

**The wall-clearance question is resolved, and the answer was not "retune".**
Part 2 flagged that a 0.70 m body needs `wall_dist_work_zone >= 0.45`, which
pushes every work-zone approach 100 mm off the wall. The user confirmed that is
correct *and* that they had already compensated on the other side: **the arm
mount was physically moved 100 mm toward the wall** (body −Y, the side_cam
side). So `wall_dist_work_zone` is now 0.45, `arm_body_offset_y` is −0.100, and
`T_ab2mb` t is `(0, −0.100, −0.651)`. Both signs are negative because Rz(180°)
flips y and the inverse flips it back — worth re-deriving rather than trusting,
which is why it was checked numerically (det +1, tz still `−arm_base_z`).

🛑 **That does NOT make joint-mode scans correct — it makes them 100 mm wrong.**
Promoted to its own section above, because it is the single most consequential
thing in this session. The short version: `/robot_pose` is built from the map
tag coordinate + `lateral` + `camera_offset` and **contains no body-width term
at all**, so a wider chassis does not change where the robot stops. Pose mode
follows the arm automatically (`arm_body_offset_y` → `p_A_W`); joint mode
cannot, because those CSVs are absolute joint angles that no transform reads.
The fix is the one the user already plans — **move the tags 100 mm off the wall
and re-measure `map.yaml`** — and it is explicitly deferred ("나중에 할 거야").
Until then: pose-mode scans fine, joint-mode scans off by 100 mm.

**front_cam: `camera_offset` 0.45 → 0.547, and the camera was rotated 90°.**
The lens is on the centreline before and after, so it sits *on* the rotation
axis and the 97 mm is the new mount, not the rotation. `T_mb2fc` is now
`(0.547, 0, 0.300)` and `robot.yaml` `camera_offset` matches it (checked — it
is the only key in that block anything reads).

⚠️ **The rotation broke `mobile_controller.py` in three places — all three are
now fixed.** `fc` *is* the optical frame `dt_apriltags` reports in, so rotating
it re-labels every `pose_x`/`pose_y`/`center_x`/`center_y`: `lateral = tag['x']`
started reading the fore/aft distance, the `center_y` arrival test started
running left/right, and the `corner0→corner1` angle `align_to_tag` drives to
zero shifted by 90° — that last one would have tried to spin the base a quarter
turn. Table of all three in the section above. Everything needed already shipped
(`center_x`, `image_width`, `camera_params[2]`); no message or node change.

**The rotation SENSE was initially ambiguous and the user resolved it the same
day: it is −90°**, giving `fc.x = +mb.x`, `fc.y = -mb.y`. "반시계방향 90°"
about a downward axis reads one way by the right-hand rule and the opposite way
to an observer standing above the robot, and the two candidates give **opposite
steering signs** — the wrong one drives into the wall — so the frame was written
with the alternative beside it and a one-tag test, rather than guessed. The
answer came from the FOV instead: **the front bumper appears at image-left and
an approaching tag enters from image-right.** Those look contradictory until you
notice the lens sits **97 mm ahead of the bumper** (`camera_offset` 0.547 vs a
0.45 m front face), so the bumper is genuinely *behind* the camera. Both
observations then agree, and both match `R_old · Rz(-90°)` exactly. Three
independent confirmations, no measurement needed.

**Lesson worth keeping:** the disambiguating fact was a *mounting* detail
(lens ahead of bumper) that neither party stated — it fell out of comparing
`camera_offset` against `length/2`. When two operator observations seem to
conflict, check the geometry before asking again.

**Verified offline** (config only, nothing on hardware): both extrinsics
matrices are orthonormal with det +1; `camera_offset == T_mb2fc` tx = 0.547;
front_cam height 0.300; `T_ab2mb` tz `== −arm_base_z` and ty
`== arm_body_offset_y` = −0.100; `wall_dist_work_zone − width/2 ==
min_wall_clearance` = 0.10; `mobile_controller.py` byte-compiles. The final
`T_mb2fc` reproduces all three observations: a tag 0.8 m ahead images at
`pose_x +0.800` (right), the bumper at `pose_x −0.097` (left), a tag 0.3 m to
the robot's right at `pose_y +0.300` (bottom).

#### Then the navigation code was ported, same session

The first draft of this entry stopped at comments, on the grounds that the
rotation sense was unconfirmed. The user confirmed it — "로봇이 전진하면 앞범퍼는
FOV 왼쪽, 태그는 오른쪽에서 나타난다" — and asked for the fix now rather than
tomorrow, so `mobile_controller.py` was ported and the config key renamed.

**The corner-angle correction became a shared helper.** `align_to_tag` and
`calculate_robot_pose` each carried their own copy of the
`atan2(dy, dx)` + `[-90, 90]` wrap; the 90° correction would have had to land in
both. They now call one module-level `tag_edge_angle_deg()`, which also replaces
the two-branch wrap with `(angle + 90) % 180 - 90`. Note this makes the range
half-open — a raw edge at exactly 180° reads −90 where the old code read +90 —
but ±90 describe the same physical line, the old code had the identical
discontinuity, and a tag a quarter-turn off square is not a state a drive-to-tag
ends in.

⚠️ **`center_y_stop_offset: -50.0` became `center_x_stop_offset: +50.0`** — the
user's call between renaming and keeping the old name. Both the key *and* the
sign had to move, and the sign is the trap: the fore/aft image axis moved from
rows to columns **and** reversed direction (image right = forward, so `center_x`
*decreases* on approach where `center_y` used to *increase*). So both
comparisons in the stop test are inverted too. Anyone reading only the rename
would get a robot that never stops.

**`lateral`'s sign did not change**, which is worth stating because it looks
like it should have. Before, `lateral = tag['x']` with `fc.x = -mb.y`; now
`lateral = tag['y']` with `fc.y = -mb.y`. Different axis, same physical meaning
(+ = the robot's right), so pure-pursuit steering is untouched.

**The verification that actually settles it** is not a sign check — it is a
regression test against the old behaviour. `/tmp/t_nav_rot.py` rebuilds the
*pre-rotation* camera matrix and the *old* `center_y` logic with its old −50
offset, then sweeps a 1 mm grid for the stop point: **forward 0.567 m, reverse
0.561 m, identical old-vs-new to 1e-9.** That matters because the old numbers
were tuned on hardware and observed working on the 2026-08-12 drive; reproducing
them exactly means this is a coordinate change, not a re-tune. 38 checks total,
all passing — including that the frame predictions come from `T_mb2fc` rather
than from the code under test. **Still nothing on hardware.**

⚠️ **`path_tag_locator` was deliberately NOT ported.** Its
`nav/robot_controller.py` + `robot_nav.yaml` are a second copy of this logic and
still hold the old axes. Confirmed it is not started by
`mobile_manipulator.launch` (it has its own three launch files), so it cannot
affect the main stack — but it drives the same base off the same rotated camera,
so its launches must not be run until it is ported. Flagged rather than fixed:
touching a package outside the requested scope, with its own separate config,
was not what was asked for.

**Agreed plan for 2026-08-14 (on the robot PC, with the user):**

1. **Verify the front_cam port on hardware.** Everything above is offline only.
   The drive direction, the stop point and the alignment angle have never been
   commanded through the rotated camera. Watch `/front_cam/tag_overlay` and
   confirm an approaching tag enters from image-RIGHT before trusting a move.
2. **Fix the two 2026-08-12 defects** — the `align_to_tag` timeout and the
   odom-stall check. The user gave the go-ahead ("둘 다 내일 고치자"). Expect
   the first one to fire during step 1: pure pursuit stops when the tag crosses
   the image centre, so arrival routinely leaves the tag at or past the frame
   edge, and `align_to_tag` then waits forever.
3. **Not on this list, still deferred:** moving the tags 100 mm off the wall and
   re-measuring `map.yaml`. Until that happens joint-mode scans are 100 mm off —
   see the blocker section near the top. Pose-mode scans are fine.

### 2026-08-13 — New base, part 2: footprint, front_cam height, and joint scans now raise the lift

Follow-up to the entry below, same session, same replacement base. The two
measurements it left open are now filled in, and the scan flow the user
actually wants is wired up.

**front_cam is 0.300 m above the mb origin, not 0.293.** The previous entry
predicted this was "almost certainly stale" because the deck dropped 374 mm.
It moved **7 mm**. Worth remembering *why* the prediction was wrong: the camera
is mounted off the chassis, not off the deck, so deck height does not carry it.
Guessing a proportional correction would have been much worse than the 7 mm
error of leaving it alone.

**Footprint 0.80 x 0.50 → 0.90 x 0.70 m.** That also retires the flag that a
0.50 m width was impossible next to a 0.65 m wheel track.

⚠️ **`wall_dist_work_zone` was NOT retuned and is now wrong.** The old three
values were self-consistent — `0.35 − 0.25` (half of width 0.50) `= 0.10 =
min_wall_clearance` — and at the new half-width of 0.35 that arithmetic gives
**zero clearance**. Holding the margin needs `>= 0.45`, which moves every
work-zone approach 100 mm off the wall. That changes where the robot drives, so
it is the user's call, not a silent edit. Left visible at the config key and in
the Drivetrain section. Nothing reads these keys today, which is the only
reason it is a documentation defect rather than a collision path.

**`lift_height: 150` added to `optimized_joints_line1.csv` — and to line2.**
The user asked only for line1. Line2 was included because `scan_full_joints`
concatenates the two and `_extract_lift_height` **refuses a partly-filled
column**, so a line1-only edit would have silently unregistered that task
instead of degrading. The physical argument agrees: the new deck sits 374 mm
lower, both lines scan the same workpiece, so if line1 needs the rise line2
does too. 150 mm = 3079 counts on the measured 0.0487143 scale, a ~12.4 s rise,
and the arm base scans at 651 + 150 = **801 mm**.

The requested tail — arm home, then lift origin homing — needed **no code**:
`_finish_task` already does exactly that, gated on
`task_flow.lift_home_on_finish` (true) *and* the task having a lift height.
Before this change `scan_joints_line1` had none, so the tail was inert for it;
adding the column is what turns it on.

**`scan_joints_line1_lift` and `joints_line1_lift.csv` are deleted.** With the
column on `optimized_joints_line1.csv`, the two CSVs were content-identical
(verified row by row before deleting) and the two tasks did the same thing.
Two identical entries in a task list is a trap, not redundancy. The Work Log
entries below that mention them are left as-is — they are a record of what was
true then.

Pose-mode tasks are deliberately untouched: the grid CSVs have no
`lift_height`, and `arm_base_z` is still a constant, so a raised lift would
offset every pose IK result by the travel.

**Verified offline** (rospy stubbed, real `TaskManager`): all six `TASK_DEFS`
plus `go_home` register with no `logerr`; `scan_joints_line{1,2}` and
`scan_full_joints` report `lift_height == 150.0` while all three pose tasks and
`go_home` report `None`; `scan_joints_line1` is still tags `[105, 106]` at
169 + 159 points and `scan_full_joints` is `[105, 106, 117, 118]` at 655; the
retired task and CSV are both gone. Configs: footprint reads 0.90/0.70 and is
now wider than the wheel track, `T_mb2fc` z is 0.300, extrinsics tz still
equals `−arm_base_z`, and 150 mm lands at 3079 counts inside the 0..7000 soft
range. **Nothing ran on hardware.**

### 2026-08-13 — Mobile base replaced: new lift scale, new `arm_base_z`, test scaffolding removed

The mobile base was swapped for a different unit. Every number below came from
the user measuring the new hardware, not from a fit or a catalogue.

**`arm_base_z` 1.025 → 0.651 m.** The manipulator base now sits 651 mm above
the ground with the lift at its origin. Changed in three places that have to
agree: `robot.yaml` `arm_calibration`, the hardcoded fallback in
`arm_transform.py`, and `T_ab2mb` in `path_tag_locator/config/extrinsics.yaml`
(tz −1.025 → −0.651), which is the declared source of truth. A check that the
extrinsics tz is exactly `−arm_base_z` is part of the verification below.

The useful consequence is that **the long-standing "1.025 vs 0.9541, 71 mm
apart" open item is closed by obsolescence** rather than resolved — both
numbers described the retired base. Any pre-2026-08-13 Work Log entry citing an
`arm_base_z` is about different hardware; don't chase those deltas.

**Lift scale is now fully measured: 341 mm / 7000 counts = 0.0487143.** The old
0.0497795 was 350 mm / 7031 where the 350 was a *catalogue* stroke that had
never met a height gauge — that was the whole reason `mm_calibrated` was false
and the node warned at startup. Both sides are real measurements now
(651 mm at the origin, 992 mm at the top), so `mm_calibrated` is **true** and
the warning is gone. `STROKE_COUNTS`/`STROKE_MM` in `lifter_node.py` follow.

⚠️ This does **not** make `arm_base_z` dynamic. It was the stated prerequisite
for that work, and it is now met, but the value is still a constant fitted at
the lift origin — a pose-mode scan at the top of the stroke is still 341 mm
off. `docs/lift_arm_base_z_analysis.md` §4.2 can be closed; the rest cannot.

**`soft_max_counts` 6700 → 7000, at the user's call.** 7000 is the full
measured stroke, so the clamp no longer holds back a margin. It is still not
decoration — the same clamp bounds `jog` and the open-ended `up`/`down`, and
the upper end still has no limit switch. The user's first phrasing was to
remove the cap outright; that was queried rather than implemented, because
deleting `_clamp` would let an `up` command push the hard stop for the full
35 s `jog_timeout_s`, which is exactly the stall that corrupts the count.
Note the launch file sets this as a `~param` and **overrides `robot.yaml`**, so
it had to change in both — a `robot.yaml`-only edit would have done nothing.

Side effect worth remembering: a worst-case 0→7000 jog now consumes ~28.2 s of
the 35 s timeout, leaving ~6.8 s of margin instead of ~8.

**`wheel_radius: 0.0825` / `wheel_separation: 0.65` mirrored into
`robot.yaml`.** `~/navifra/param.yaml` `base_controller` owns these and nothing
in `apriltag_nav` reads either — the copy is for visibility, and param.yaml
wins on any disagreement. They also make `robot.width: 0.50` provably wrong
(the wheel track alone is 0.65), which is now flagged where the clearance
margins are defined; length/width still need a tape measure.

**The 56/57 bring-up scaffolding is gone** — the `test_move_57` TASK_DEFS
entry, `task/csv/test_move_57.csv`, and the tag block plus edge pair in
`map.yaml`. The replacement scenario needed no new code: `scan_joints_line1`
already runs `optimized_joints_line1.csv`, whose `group_id`s are exactly the
requested 105 and 106.

⚠️ It is not a like-for-like swap, though. The deleted task was **one** joint
point (a 5° wrist roll); `scan_joints_line1` is **328** points and runs the
full Keyence → Basler → ONNX chain at every one of them. Treat the first run as
a long operation, not a twitch test.

**Verified offline** (rospy stubbed, real `MapManager` / `TaskManager`): all
three YAML files parse; `arm_base_z` and the extrinsics tz agree; 651 + 341
sums to 992; tags 56/57 and their edges are gone and `find_path(56,57)` now
fails; `508→105` (10 hops) and `508→106` (9 hops) still route; `test_move_57`
is unregistered while all seven real tasks still load; and `scan_joints_line1`
registers as steps `[105, 106]`, 169 + 159 joint-mode points, no lift height.
Touched Python compiles and the launch XML parses with `soft_max_counts=7000`.

**Not done, needs the robot:** `T_mb2fc` in `extrinsics.yaml` still claims
front_cam is 0.293 m above the mb origin. That was measured on the old base and
the deck moved 374 mm, so it is almost certainly stale — but guessing it would
be worse than leaving it visibly untouched. `robot.length`/`width` likewise.

### 2026-08-12 — First real drive of `mobile_controller`; two silent-failure defects

Bring-up session on hardware. User asked to test 56 → 57 and then a small arm
move, to see whether `mobile_controller.py` actually works. **It does** — and
the two things that went wrong are both worth keeping.

**Temporary test scaffolding — DELETE when the bring-up is over.** Three
places: the `56`/`57` tag block and their edge pair in `config/map.yaml`, the
`test_move_57` entry in `TaskManager.TASK_DEFS`, and
`task/csv/test_move_57.csv`. All three carry a `TEMPORARY`/`DELETE` comment.
The tags are deliberately an **island** — their only edges are 56↔57 — so BFS
cannot route a real task through them and `GOTO 57` from anywhere else fails
with "No valid path" instead of driving somewhere unexpected. The CSV is one
`point_id` whose q1..q5 are byte-identical to the arm home pose with q6 rotated
+5°, i.e. a wrist roll and nothing else. `group_id` is the tag id, so it
registers as the single step `{tag: 57, scan: True}`.

Verified offline before running (rospy stubbed, real `MapManager` /
`TaskManager`): 19 checks — `find_path(56,57) == [56,57]`, 57 unreachable from
every real tag, `508→106` still routes, the task registering as one step with
no lift height, q1..q5 diff 0.0 rad, and all five pre-existing tasks still
loading.

**What the hardware run showed.**

1. **Navigation works.** 18:15:10 — path `[56, 57]`, `move, forward`, 0.40 m,
   reached 0.05 m/s at 5.1 s (the offline model said 4.9 s), stopped on the
   tag-centre condition at `traveled:0.275m`. Map, edges, BFS, pure pursuit,
   S-curve and the odom-driven profile are all sound.
2. **It then hung in `align_to_tag()` for 30 s until the user hit Ctrl-C.**
   The tag left the frame at the moment of arrival — `[Vision] Tags detected`
   goes silent for the whole 31 s — and that loop has no timeout. Promoted to
   a standing warning under *Mobile base split*.
3. **From 18:16:43 the motor CAN feedback died** (`MOTOR_FEEDBACK_TIMEOUT` on
   node 1, later 2 as well; `/safety/estop` false, can0 ERROR-ACTIVE with
   berr 0/0 and no bus-off, so the bus is fine and the drives are not). Every
   later `GOTO` was commanding a dead drive. Drive enable had been flaky all
   session — repeated "failed to reach OPERATION_ENABLED" / "servo OFF after
   STO/E-stop?" from 18:05, one successful fault reset at 18:13:04, then the
   good run, then the feedback loss.
4. **And that is how defect two surfaced:** with odom frozen, `traveled_dist`
   stays 0.000, so the S-curve sits at `min_speed` and the loop writes
   0.01 m/s to a dead drive for 60 s with no error. 14 s of byte-identical log
   lines. Also promoted to a standing warning.

**Diagnosis method worth repeating:** the answer came from `~/.ros/log/<run>/`
(`mobile_node-13.log` has the per-tick `[BLIND]/[TAG] traveled:… spd:…` trace)
cross-referenced with `journalctl -u navifra-robot`, plus reading
`/motor/error`, `/motor/alarm` and two `/odom` samples off the still-running
driver. The apriltag_nav stack was already down; the navifra service was not,
which is what made the live reads possible.

**Speed audit** (asked for before the run, confirmed by it) and **four dead
`robot:` config keys** are both recorded under *Mobile base split* rather than
here, since they constrain future tuning.

**Not fixed at the time — needed the user's call, and got it on 2026-08-13:
both are scheduled for 2026-08-14.** The `align_to_tag`
timeout, a more negative `center_y_stop_offset` (**that key is now
`center_x_stop_offset` and the direction reversed — more *positive* stops
earlier; see the front_cam rotation section**), and an odom-stall check in
`execute_pure_pursuit`. Nothing should be re-tested until `/motor/error` reads
false and `/odom` advances when the base is pushed.

### 2026-08-12 — `robot_ui`: an operator UI that owns no device

The user has a working PyQt data-collection system (`pyqt_systeam.zip`) and
wants the same capability here, with one architectural change stated up front:
**everything that touches a device stays in the workspaces; the UI only
integrates.** That is the right call, and the reason is concrete — that UI holds
a pypylon camera, a pyrealsense2 pipeline, a Fairino RPC connection and a
pyserial handle *in the GUI process*, so it is a second owner of four devices
that already have owners here. Running it alongside this stack means two
processes commanding one arm with neither aware of the other.

**The boundary is structural, not a convention.** `robot_ui/ros_bridge.py` is
the only module allowed to import rospy; no module in the package may import a
device SDK. Verified by grep, not by intent: the only `import rospy` lines in
the package are in `ros_bridge.py` and the node entry point, and there are zero
imports of pypylon / pyrealsense2 / fairino / serial.

**Four gaps had to be closed in `apriltag_nav` first** — the UI could not have
been written against the interfaces that existed:

1. **`/arm/state` (new `robot_msgs/ArmState`).** `/arm/status` was a bare String
   with a state word. Nothing published the live TCP pose, so a UI wanting the
   six pose fields had no source but its own RPC connection — the exact
   double-ownership being removed. Now carries pose, joints, busy and a
   `motion_seq`. `/arm/status` stays for existing consumers.
2. **Manual teaching: `/arm/move_cart`, `/arm/jog_cmd`.** `ArmController`
   exposed only `move_to_home` / `cancel` / `execute_scan_points`, so hunting
   for a collection pose by hand had *no* ROS path at all. Added `get_tcp_pose`,
   `get_joints_deg`, `move_cart`, `jog`.
3. **`/task_state`.** `task_executor` had a full state machine
   (IDLE/MOVING/ARRIVED/SCANNING/SCAN_DONE/ERROR) visible only through
   `loginfo` and the STATUS lamp — so "IDLE is a detectable end-of-task", listed
   on the 2026-08-11 deck as a GUI precondition, **was not actually true**.
   Latched JSON, with group progress the enum cannot express.
4. **`inference_node` + `robot_msgs/PredictRa`.** On-demand Ra for one frame.
   The UI must not load its own model: two resident copies is ~250 MB duplicated
   and two answers that drift apart when the paths do.

**Decisions the user made when asked:** manual jog *and* absolute move both
wanted; inference on the ws side as a service; plugin hot-reload kept but
scripts get ROS access only; the serial light bar is *not* the Crevis VISION
lamp, so the UI shows lamp state and never drives it.

**Why topics and not actionlib for move_cart/jog.** Weighed, per the standing
note. actionlib is still the better primitive for a long cancellable motion, but
adopting it for one command would give the arm two completion protocols
alongside `/arm/scan_command` + `/scan_finished`. They use `mobile_node`'s
`motion_seq` model instead, which this file already records as the better of the
two shapes in use. If actionlib is ever adopted, these are the natural first
move.

**Two things found by reading, not guessing:**

- **There is no Basler live preview in this architecture** and there cannot
  casually be one — the device is kept `Close()`d between captures and
  `/basler/image_raw` is "last captured frame, not a stream". The reference UI's
  whole aiming workflow depends on a live view. Resolved without breaking the
  rule: preview asks `/camera/set_active` to hold the device open and polls the
  capture service with the lamp **off**. The UI still never opens the device,
  and the lamp stays bracketed with the shutter.
- **`inference_interface.py`'s `Resize((900,900))` is very probably wrong.** The
  exported graphs cut their input into a 3x3 grid of native 300x300 tiles fed to
  Conv3d — confirmed by reading `model/exported/resnet3D_gray.onnx`'s operators,
  not from the reference code. Resizing 5472x3648 to 900x900 makes each tile
  cover ~6x the surface at ~1/6 the detail, destroying the texture scale a
  roughness model reads, while still returning a confident number.
  `inference_node` centre-crops. **`inference_interface.py` was deliberately NOT
  changed** — every Ra value in every existing result CSV came out of the resize
  path, and silently moving them would make old and new scans incomparable with
  nothing in the data to show why. Open decision, see below.

**ONNX only, on the user's instruction.** A first pass imported the reference
system's two `.pt` checkpoints (they pickle whole `nn.Module` objects, needing a
`sys.modules['model']` alias to unpickle). The user ruled out both `.pt` and the
reference weights, so all of it — 266 MB of checkpoints, the copied network
definitions, the export tool — was removed, and `RaPredictor` now *refuses* a
non-`.onnx` path rather than quietly loading torch. Slots are configurable and
partial: a missing secondary costs the cross-check, not the primary prediction.

**Verified offline, nothing on hardware:**

- `t_preproc.py` — the numpy `CenterCrop` reproduces torchvision's
  `ToTensor → CenterCrop(900) → Normalize(0.5,0.5)` **bit-exactly** (max |diff|
  = 0.0) on full-frame, 720p, straddling and undersized inputs. A silent
  mismatch here would be equivalent to swapping the model.
- `t_ra_onnx.py` — both workspace graphs load and score; a `.pt` is refused; a
  missing secondary leaves the primary working.
- `t_ui.py` — 26 checks against the real `MainWindow` with a fake bridge under
  `QT_QPA_PLATFORM=offscreen`: state → label plumbing, jog arguments, blank
  target fields taking the live pose, capture → save → inference ordering, Ra
  display, preview holding and releasing the camera, capture forcing preview
  off, STOP ALL reaching arm + task + mobile + lift, plugin discovery and a
  6-point grid run, and close releasing the camera.
- `t_bridge_live.py` — the real bridge against the running master, **read-only
  by construction**: it registers and inspects, never publishes, and
  `shutdown()` is not called. All 15 subscriptions and 6 publishers resolve.

Two real defects were caught this way and fixed: `append_log` was touching
`QPlainTextEdit` from the plugin worker thread (Qt printed
`Cannot queue arguments of type 'QTextCursor'`; the comment claiming it was
safe was wrong), and `shutdown()` unregistered only the image subscribers, so
the rest kept firing into a destroyed QObject.

**NOT verified:** no hardware. `catkin_make` passes and the whole tree
byte-compiles, but no new node has been *run* — in particular `move_cart` /
`jog` have never commanded the real arm, and `~jog_max_step` (50) is an
untested bring-up guard. The stack that was running during this session
predates these edits, so it is still serving the old `arm_node`; the changes
need a restart. The user asked for read-only work on the live robot and no
motion commands were published at any point.

**Follow-up the same day, on the user's decision: both paths now CenterCrop.**

The open question above was answered — make the scan pipeline crop too. Rather
than editing the one `Resize` line and leaving two copies of the transform to
drift again, `inference_interface.py` became a thin adapter over `RaPredictor`.
Its public surface (`load_model(path) -> bool`, `infer(image) -> Optional[float]`)
is unchanged, so `scan_pipeline.py` was not touched; model loading, warm-up,
preprocessing and NaN rejection now exist once. The scan path also inherits the
ONNX-only rule: it refuses a `.pt` instead of quietly pulling in torch.

⚠️ **Ra values recorded before 2026-08-12 are not comparable with later ones.**
Measured, not asserted — the old resize transform was reconstructed and run
against the same graph:

| frame | old (resize) | new (crop) | shift |
|---|---|---|---|
| 5472x3648 noise | 0.527 | 0.261 | −0.267 |
| 5472x3648 structured | 0.298 | 0.106 | −0.192 |
| 1280x720 noise | 0.617 | 0.274 | −0.343 |

Roughly 50–64% lower. That is a different measurement, not a rounding
difference. These are synthetic frames so the absolute numbers mean nothing;
what they establish is the magnitude. **The shift on real workpiece surfaces is
unknown** — worth one paired re-scan of a surface with an existing CSV before
trusting new numbers against old targets. Old CSVs are not wrong as records;
they measure a different transform. Do not mix them in one analysis.

Verified: `t_unified.py` — the scan path and the on-demand path return
**bit-identical** values (worst |diff| = 0.0) on three frame shapes; a `.pt` is
refused by both; a refused model returns None rather than raising.
`t_preproc.py` and `t_ui.py` re-run unchanged. Still nothing on hardware.

**Follow-up: first run against the real robot.** The user restarted the stack,
so the new interfaces went live and were checked read-only — no motion command
was published at any point, and `ArmState.motion_seq` stayed 0 throughout,
which is the machine-checkable proof of that.

Working on hardware: `/arm/state` publishes the real TCP pose and joints at
10 Hz (`pose_valid: true`), `/task_state` reports IDLE, `/inference/predict`
returns `0.260715` for a synthetic frame — **bit-identical to the offline
result**, so the cv_bridge round-trip changes nothing.

Two defects the live run found that no offline test could have:

1. **`inference_node.py` was not executable**, so roslaunch skipped it silently.
   Created with an editor rather than `cp`, and `install(PROGRAMS)` only affects
   the install space — roslaunch runs the source file from the devel space, so
   the mode bit on the source is what matters. `chmod +x` applied to it and to
   `robot_ui_node.py`, which had the same problem waiting.
2. **`RosBridge` silently lost every LATCHED topic.** Subscriptions are created
   in its constructor, but a consumer connects its slots afterwards — so the
   retained message was emitted with nothing attached and never came again;
   latched means "delivered once on connect", not "redelivered later".
   `/task_state` and `/camera/state` are both latched one-shots and were simply
   absent from the window. `/lifter/state` hid it by re-arriving at 2 Hz.
   Fixed with a per-signal cache plus `replay()`, which `MainWindow` calls after
   connecting; it also populates the whole window immediately instead of after
   the next periodic update.

**Follow-up: the UI was driven on the real robot for 11 minutes**, which closed
out every remaining unexercised path and turned up one defect that only an
operator pressing buttons could have found.

Recorded by `arm_node`: 33 completed motions — 12 × `jog z +1`, 8 × `jog z +10`,
3 × `jog z -10`, 7 × `move_cart`, all ok; the busy guard refused one command
(`refused: busy, dropped move_cart`) and two motions came back `cancelled`.
Three real captures went through capture → save → inference:

    capture_20260812_154210.png  primary=0.4097  secondary=0.4468  2.58 s
    capture_20260812_154218.png  primary=0.3992  secondary=0.4439  2.44 s
    capture_20260812_154617.png  primary=0.3825  secondary=0.4306  2.66 s

The two slots track each other to within ~0.05, so the cross-check is doing
something useful rather than agreeing trivially.

### `StopMotion` collides with an in-flight move — but the stop still lands

`ArmController.cancel()` calls `self.robot.StopMotion()`. The Fairino
`Robot.RPC` is **one socket**, and `MoveCart` / `MoveJ` hold it while they
block, so a `StopMotion` issued from another thread can be rejected by the SDK
with `Request-sent` — a request is already outstanding.

**STOP ALL nonetheless stops a moving arm.** Observed by the user on the robot,
and the log agrees. Both times the arm was actually in motion, the pattern was
identical:

    15:47:42,833  CANCEL requested            <- 1st
    15:47:42,836  Stop failed: Request-sent   <- 1st StopMotion collides
    15:47:42,847  CANCEL requested            <- 2nd, 14 ms later
                                              <- no error: 2nd StopMotion took
    15:47:42,852  motion 30: cancelled        <- MoveCart returns 5 ms later

`MoveCart` returning 5 ms after a successful `StopMotion` is the arm halting;
a move left to finish does not return on that timescale.

The reason there is a second attempt is that **STOP ALL sends two cancels**:
`MainWindow._on_stop_all` calls `bridge.arm_cancel()` and also publishes
`STOP` to `/task_command`, and `task_executor` cancels the arm too. First one
collides, second gets through.

⚠️ **That made a working stop an accident of the double-send, not a designed
property** — and the single-send path was confirmed broken on the robot the
same day: the user reported "STOP ALL 没有问题，但是 cancel arm motion 没有用".
The Arm tab's own button publishes `/arm/cancel` exactly once, and so does the
scan-cancel path; a lone `StopMotion` that loses the race had nothing behind it.

**Fixed: `cancel()` now retries.** Up to `~cancel_attempts` (10) tries at
`~cancel_retry_s` (0.02 s), returning the moment one is accepted — so the normal
case costs one call, and the observed collision costs two and ~20 ms. It returns
True/False rather than nothing, and on total failure says so loudly and names
the hardware e-stop. `tools/arm_controller_sdk.py` was kept in step, since
`arm_node` can be pointed at either.

A second dedicated `Robot.RPC` connection was considered and is not needed: the
socket frees within tens of milliseconds, so a retry is enough and adds no new
connection to the controller.

Verified offline (`t_cancel.py`, 12 checks against a fake robot that rejects the
first N attempts): accepted-first-time costs exactly one call; one rejection
recovers on attempt 2 in 20 ms; five rejections still recover; SDK builds that
return an error code instead of raising are handled; the budget is not exceeded;
`cancel_requested` is set **before** the first StopMotion, so a mover polling it
cannot miss the flag if the stop is accepted instantly.

**Confirmed on the robot**: "Cancel arm motion" alone now halts a moving arm.

### Camera pane: the image was never distorted, the CELL was the wrong shape

Reported as "太小了而且比例还不对". Measured before assuming: the drawn image's
aspect matched the source to **0.00%** at every widget size, so nothing was
being stretched. What was wrong is that the original grid gave the Basler a
near-square ~640x600 cell for a 3:2 (5472x3648) sensor, so `KeepAspectRatio`
letterboxed 29% of the pane away and drew the image at just 640x426.

Rebuilt as a full-width tab stack (Basler live / Last capture) over a thumbnail
strip for the three tag cameras, with an explicit initial splitter size — stretch
factors alone let the control column claim width from its long explanatory
labels. The capture tab raises itself when a frame arrives, since aiming and
judging a shot happen at different moments and can share one large area.
Double-click any view to give it the whole window.

Main-view : strip stretch is **7:2**, set from measurement after the user asked
for more room for the three tag cameras. At 1920x1080:

| main:strip | Basler | one thumbnail |
|---|---|---|
| 5:1 | 1152x768 | 284x160 |
| **7:2** | **1071x714** | **380x214** |
| 3:1 | 1030x687 | 398x223 |
| 5:2 | 979x653 | 398x223 ← no gain, pure loss |

The thumbnails **saturate at 398x223** because three share the width; past 3:1
the strip only grows taller and 16:9 frames cannot use it. 7:2 buys 95% of that
ceiling for 14% of the Basler's area. Don't push it further "to make the
thumbnails bigger" — beyond 3:1 they do not get bigger.

Net against the original layout: Basler 1071x714, **2.8x the old area**;
maximised 1401x934, 4.8x.

### The wrist camera is MONO — do not widen it to BGR

Reported as "Basler 实时达不到相机的正常帧数 5fps". It was reaching 2.18 fps.
Two costs, both measured on the robot rather than guessed:

| stage | per frame | rate | bytes/frame |
|---|---|---|---|
| as found | 459 ms | 2.18 fps | 59.9 MB |
| after warmup skip | 309 ms | 3.23 fps | 59.9 MB |
| **after mono8** | **183 ms** | **5.19 fps** | **20.0 MB** |

**1. `camera.warmup_s` (150 ms) was applied to every capture**, including
preview frames taken with the lamp off — and the warmup exists to let the VISION
lamp reach brightness. `_handle_capture` now skips it when the lamp is off AND
the device was already open, which is exactly the preview case; scan captures
use the lamp and still warm up. Worth exactly 150 ms, confirmed by timing
lamp-off (309 ms) against lamp-on (459 ms).

**2. `camera_interface.py` forced `PixelType_BGR8packed` on a mono sensor.**
The part is an **acA5472-5gm** — the `m` is mono — and this was settled from the
pixels, not the model number: on a real frame `B == G == R` exactly, every
pixel. So each 20 MB frame was inflated to 59.9 MB of three identical channels
and carried twice (service reply plus the `/basler/image_raw` echo), on a 5 fps
part whose entire budget is ~190 ms. The converter now follows
`camera.PixelFormat` — mono8 for a mono sensor, BGR8packed for a colour one.

Nothing downstream needed changing, which is why this was safe:

- `RaPredictor.preprocess` already accepts a 2-D frame and does GRAY2RGB, and
  the **model input tensor is bit-identical** either way (verified, max |diff| =
  0.0). Ra values do not move.
- cv_bridge widens mono8 for any consumer that asks for bgr8.
- The UI keeps it mono end to end — `QImage.Format_Grayscale8` straight to the
  screen, no channel expansion on the GUI thread — and saved PNGs drop from
  59.8 MB to 20.0 MB, losslessly.

⚠️ **5.19 fps is the sensor, not the software.** Verified that these are real
new frames and not `GrabStrategy_LatestImageOnly` handing back the same buffer:
16 consecutive grabs, 16 distinct MD5s, zero repeats. So there is nothing left
to win here — `publish_last: false` would still halve the echoed bytes but
cannot raise the rate, because the bottleneck moved from the wire to the
exposure. Do not chase it.

⚠️ **Fill ratio is the wrong thing to optimise, and chasing it caused a wrong
first attempt.** It is fixed entirely by pane aspect vs image aspect — a 3:2
frame in a 16:9 pane cannot exceed 84% no matter what. The first maximise hid
only the thumbnail strip, which adds HEIGHT to a width-limited image: fill fell
97% -> 79% while the image stayed exactly 954x636, i.e. strictly worse. Hiding
the control panel is what actually enlarges it. `t_layout.py` therefore asserts
undistorted aspect and absolute drawn size, and only reports fill.

(An earlier version of this entry claimed `/arm/cancel` cannot stop a moving arm
at all. That was wrong — it counted the four `Stop failed` lines without
checking whether the immediately following attempt succeeded, which it did. The
hardware e-stop, a PILZ PNOZmulti 2 cutting motor power independently of ROS,
remains the authority regardless.)

Also seen once: `TCP pose read failed: Idle`, same collision — the state timer's
try-lock keeps it off the socket during a *held* motion, but `cancel()`
deliberately does not take that lock (a stop that waited for the motion it
aborts would never run), so a stop and a pose read can still overlap.

**Open, for the user:** `pip3 install --user onnx` was run during the abandoned
`.pt` export and pulled protobuf 5.29.6. rospy, onnxruntime, cv_bridge and
robot_msgs were checked and all still import; `onnx` itself is now unused and
can be removed.

### 2026-08-11 — `mobile_node` + `MobileClient`; the drive gets an owner

Closes blemish #1 from the deck written earlier the same day: the drive was the
only device `task_executor` held in-process. All three motion devices now have
the identical shape — pure logic → owner node → client proxy — and
`task_executor` owns **no device at all**.

**Renames, per the user:** `arm_controller_node` → `arm_node`,
`base_lifter_node` → `lifter_node`, `robot_controller.py` →
`mobile_controller.py` (`RobotController` → `MobileController`). Two design
questions were put to the user and answered:

- **MobileClient ↔ mobile_node: topic + state polling**, mirroring `LiftClient`
  rather than introducing a service or actionlib. Consistency with the client
  that already exists won over the better primitive; the actionlib note above
  still stands for whenever any of the three is reworked.
- **`/base_lifter/*` → `/lifter/*`**, matching the node's new name.

⚠️ **That rename put `/lifter/*` (ours) one character from `/lift/*` (the
navifra driver's raw interface).** `/lift/*` has none of `lifter_node`'s
guards. Flagged to the user, and warned about in the Architecture section, in
README §9 and on slide 8 of the deck — but the topic names themselves are the
hazard and no amount of documentation removes it.

**`seq`-based completion, not position-based.** The lift can be polled for an
absolute height, so `LiftClient` can ask "are you there yet". Navigation has no
such measurable end state — arrival is a judgement made inside
`MobileController`. So `mobile_node` stamps every finished move with an
incrementing `seq` plus a `result` dict, and `MobileClient` waits for `seq` to
pass the value it read *before* publishing. That is strictly better than
`LiftClient._saw_busy`: it needs no latch, and a stale pre-command state message
cannot satisfy the wait because its `seq` is by definition not newer.

**MapManager moved into `mobile_node`** — path finding belongs to whoever
drives. Stop services do not take the motion lock, same rule as `lifter_node`;
a stop that waited for the move it aborts would never run. `mobile_node`
subscribes to `/safety/estop` itself instead of having the orchestrator forward
one, because a stop that only works while the orchestrator is healthy is not a
stop.

**The dependent-value audit the user asked for** found three things past the
obvious renames:

1. **`tools/navigate.py` holds its own `MobileController`**, so it is a second
   `/cmd_vel` publisher. There is no arbitration below it — navifra's
   `base_controller` obeys whichever message arrived last, so it would fight
   `mobile_node` at 50 Hz with no error anywhere. Documented with a "never run
   this while the stack is up" warning rather than left as a trap.
2. **`tools/send_debug_cmd.py` documented `/mobile/stop` as a `std_msgs/Bool`
   topic.** It is a `std_srvs/Trigger` service; corrected to `rosservice call`.
3. **`tools/test_all_devices.py`** predated the split — new node, new services,
   new `/mobile/state` topic row. The state row matters: it is what `MobileClient`
   polls, so a silent one means every `GOTO` times out instead of failing visibly.

`CMakeLists.txt`'s `install(PROGRAMS)` was also missing several scripts
(pre-existing, not caused by the rename); it now lists all eight.

**Verified offline** — `/tmp/rosfake.py` (in-process fake ROS: real threads,
real time, latched topics, services), plus two suites:

- `/tmp/t_mobile.py`, 16 checks on `MobileClient` against a fake
  `mobile_node`: happy path; completion so fast `busy` is never observed;
  navigation reporting failure; the node ignoring the command (bounded by
  `ack_timeout`, not `move_timeout`); a result carrying someone else's tag;
  the node absent entirely (bounded by `connect_timeout`); **a stale
  pre-command state not satisfying the wait**; two moves in one session;
  stop/cancel/clear_stop reaching the right services in order; and the stops
  not raising when the node is absent.
- `/tmp/t_flow.py`, 18 checks driving the real `task_executor`, `TaskManager`
  and all three real clients against faked owner nodes. `TASK
  scan_joints_line1_lift` reproduces as `mobile_cancel → arm_cancel →
  mobile_clear_stop → arm_home → move 105 → lift_goto 150.0 → scan 169 pts →
  move 106 → scan 159 pts → arm_home → lift_home`. Also confirms
  `hasattr(ex, 'map_mgr') is False`, that `GOTO` makes exactly one trip and
  touches no lift, and that `STOP` reaches `/mobile/stop`.

The first three events in every task are not noise — a `TASK`/`GOTO` preempts
whatever is running, so `_command_cb` only *queues*; `run()` is what clears the
stop latch and starts it. A harness that calls `_command_cb` without running
`run()` sees the preempt and nothing else.

**NOT verified:** no hardware, no ROS master, no `catkin_make` — the user is
checking on the robot PC. The harnesses live in `/tmp` and will not survive a
reboot.

### 2026-08-11 — Architecture deck + an honest modularity audit

Docs-only; no code touched. User asked for PPT material covering both
workspaces, and prefaced it with "역할 분담이 확실하고 전체 모듈화가 잘 되어
있는거지?". That is a verification question, not a rhetorical one — and after
two of my confident claims were proven wrong the day before, agreeing without
checking was not an option. So I audited before drawing.

New file: `docs/architecture_slides_kr.md`, 11 slides. Kept out of
`docs/GUIDE_kr.md` deliberately — the guide is the operating manual and is
under a standing "ask before touching" rule; this is presentation material with
a different audience and a much shorter half-life.

**What the audit confirmed (by grep, not by memory):**
- No file in `apriltag_nav` publishes to `/lift/*` except through
  `navifra_devices.py`, and only `lifter_node` instantiates the writer
  path. The one-owner rule is actually held, not merely intended.
- `arm_controller.py` is the only importer of the Fairino SDK.
- `scripts/` (ROS wiring) vs `src/apriltag_nav/` (pure logic) separation is
  real, which is exactly why the offline stub harnesses work at all.

**What the audit found wrong — recorded on the deck rather than hidden:**
1. Drive is the one device with **no owner node**. `task_executor` publishes
   `/cmd_vel` directly via `MobileController`, asymmetric with arm and lift.
   Works today; it is a consistency debt, not a bug.
2. `arm_transform.py` holds `arm_base_z` as a **constant** (0.974167 in the
   calibration tools). Raising the lift does not move it, so any task using the
   `lift_height` column silently scans with a stale world→arm transform. This
   is the highest-value item on the list. See `docs/lift_arm_base_z_analysis.md`.
3. `mm_calibrated: false` — mm↔count is still unverified.
4. `LiftClient.goto_mm()` still ignores `task_executor._stop_requested`, i.e.
   the poll loop does not use the one advantage that justified polling.
5. `navifra-robot.service` hardcodes `User=abc` / `HOME=/home/abc`, which does
   not match this machine's `seonghyeok`. A deployment assumption baked into a
   unit file — flagged for the merge work.

**Navifra facts gathered this session (first proper look):** 6 packages ↔ 6
physical devices; `param.yaml` is loaded *after* the sub-launch includes in
`robot.launch`, which is *why* field overrides win — worth knowing before
anyone "cleans up" that ordering. BMS is CAN **250K**, separate bus from the
motor drive's 500K. Crevis and PILZ are Modbus/TCP.

**Merge plan put on the deck** (4 reversible stages: startup unification →
freeze interfaces in `robot_msgs` → absorb source if obtainable → single
bringup launch). Stage 3 is optional because navifra ships as a binary install
space with no `src/`; stages 1/2/4 deliver most of the value without it.

**GUI section** records the three preconditions the current design already
satisfies (self-contained tasks, single `/task_command` entry point, IDLE as a
detectable end-of-task), and the recommendation that the sequence queue live in
`task_executor` rather than the GUI so a GUI crash cannot strand a running job.
Same conclusion as the topic-vs-service discussion: `actionlib` is the right
primitive and keeps being the answer.

Not verified: nothing ran against hardware or a master; this session read code
and config only.

### 2026-08-10 — Correction: a custom srv was never impossible

Docs-only. Two places in this file claimed `apriltag_nav` cannot define a
service that carries a height, because it has no `message_generation`. The
premise is true and the conclusion does not follow: **`robot_msgs` has
`message_generation` and `add_service_files`**, apriltag_nav already depends on
it in both `package.xml` and `CMakeLists.txt`, and `robot_msgs/CaptureImages.srv`
is an existing service in this workspace taking `int32` + `float32` + `bool`.
A `SetLiftHeight.srv` was always a few lines away.

The narrower true statement — the one the docs should have made — is that
`std_srvs` ships only `Empty` / `Trigger` / `SetBool`, none with a float
request, while `std_msgs` ships `Float32`. That is an accident of what the
standard packages contain, not a structural property of services. A `.srv` is
two `.msg` blocks split by `---`, through the same generator.

`LiftClient` is unchanged, but the justification for it was overstated twice
in one session and both corrections are in the section above.

The first draft said a custom srv was impossible. The second said a service
call cannot be cancelled, so a service would have needed a second thread to
call `/lifter/stop` — the user immediately pointed out that a **topic
needs exactly the same thread**, which is right: the stop path is identical
either way, and `_srv_stop` would have released a blocked service call just as
readily. What actually survives is much smaller — a rospy service call takes no
in-flight timeout, so an unresponsive node blocks the caller forever, while the
poll loop owns its deadline. Against that sits the cost the polling approach
imposes: `_saw_busy`, the two-phase wait and the race they close would not
exist with a service, since `_do_goto` already blocks correctly server-side.

Lesson worth keeping: the decision was made first and the reasons assembled
after, which is how two wrong ones got written down. `actionlib` — the ROS1
primitive built for long, cancellable, progress-reporting motions — was never
considered at all, and it is the obvious candidate if either client is reworked.

### 2026-08-10 — Origin pass deleted; tasks stay put when they finish

Two simplifications, both from the user, both removing something added earlier
the same day.

**The `approach_from_below` origin pass is gone.** The rule routed every
descending absolute move down to the limit switch and back up, so the last
motion before stopping was always an ascent and backlash was always taken up
the same way. The user's argument for dropping it: every task now ends with
lift origin homing and nothing lowers the lift mid-task, so an absolute move
only ever climbs from 0 — the descending case the rule defended cannot occur.
Checked that against the code and it holds. The rule cost up to 56 s when it
tripped, and roughly a third of `lifter_node` existed to service it
(`_needs_origin_pass`, `_already_parked`, `_approached_from_below` and the six
places that invalidated it).

Two behaviours had to be chosen to replace it, and the user picked both:

- **A descending target is now carried out as asked**, not refused. It logs a
  warning that the stop is loaded the opposite way, because the count no longer
  means the same physical height. Backlash is still real — what changed is that
  the node no longer spends a minute defending against it.
- **An absolute move while un-homed is refused**, restoring the pre-2026-08-07
  behaviour. This is the one case that must not be silent: hardware fact 3 is
  that the driver *ignores* absolute commands before homing, so auto-homing
  would turn one line of config into an unannounced 28 s descent, and returning
  success without homing would be a lie. `jog_cmd` remains the un-homed path.

**Tasks no longer drive back to `START_TAG`.** `return_home_on_finish` was
added a few hours earlier and is now removed entirely, along with the `scanned`
flag that gated it. The user is building a block-coding GUI where tasks are
dropped into slots and run in sequence — `scan_joints_line1 → go_home` is two
blocks — so a task that appends its own motion is not composable. `go_home` is
back to being an ordinary task you send yourself. `lift_home_on_finish` stays:
it is not extra motion, it is what re-establishes the origin the next task's
climb depends on. This is promoted to a standing rule in the section above,
not left in the log, because it constrains future task design.

**Changed:** `lifter_node.py` (docstring, `_do_goto` rewritten, the
bookkeeping removed); `robot.yaml` (`approach_from_below` and
`return_home_on_finish` keys deleted, with the reasoning left at both sites);
`mobile_manipulator.launch` (param removed); `task_executor.py`
(`_finish_task` no longer takes `scanned`); CLAUDE.md.

**Verified offline** (`/tmp/t_goto.py`, new): a 0→3013 climb is one direct move
with no origin pass; 3013→1000 descends directly, warns, and calls no `home()`;
un-homed absolute is refused with nothing moved; a same-target and a
within-tolerance target produce no motion; 9999 clamps to 6700; two
home-then-climb cycles produce exactly four moves. The task-flow suite was
updated and still passes — a lift task now ends at `arm_home → lift_home` with
no trip to 508. **Not verified:** no hardware, no ROS master, no `catkin_make`.

⚠️ If a future task ever *lowers* the lift mid-run, or if one is started
without homing in between, the descending-approach warning above becomes a real
position error. The assumption is documented at `base_lifter.soft_max_counts`
in `robot.yaml` and in the node docstring — check both before adding one.

### 2026-08-10 — Tasks can set the lift: `lift_height` column + `LiftClient`

The user described the operating scenario they want end to end and asked what
was missing. Steps 1–7 and 10–12 already worked. The gap was the middle: a task
could not set the lift *at all*. `task_executor.py` had zero references to
`base_lifter` outside one comment, and `lifter_node`'s only services are
argument-less `Trigger`s, so the only argument-carrying entry point was the
`/lifter/height_cmd` `Float32` topic, which is fire-and-forget.

*(This entry originally justified that with "apriltag_nav has no
`message_generation`, so a height-carrying service cannot even be defined
here". That was wrong — `robot_msgs` exists for exactly this and apriltag_nav
already depends on it. See the corrected reasoning in the section above; the
choice stands on cancellability and duration, not on the srv being impossible.)*

**`LiftClient` (new, `src/apriltag_nav/lift_client.py`) mirrors `ArmClient`.**
It publishes to the topic and then *polls* `/lifter/state` to turn that
into a blocking call, because there is nothing to call synchronously. The
subtle part is `_saw_busy`: it is a **latch set in the state callback**, not a
snapshot read after publishing. A snapshot is a real race — `state` is
published at 2 Hz, so a sample taken just after the publish can still hold the
*pre-command* state, read `busy == null`, see the lift already near target and
report success while the node is in fact about to run a full anti-backlash
origin pass: down ~28 s, up ~28 s, with the arm mid-scan. The latch plus a
two-phase wait (ack, then idle) makes a late pickup impossible to miss. Note
`busy` in the state JSON is a *string or null*, not a bool.

**One height per task, enforced at load time.** `TaskManager._extract_lift_height`
refuses to register a task whose `lift_height` column disagrees across rows,
is only partly filled, is all blank, or is non-numeric. No winner is picked:
the joint angles in a scan CSV were solved at one base height, so running them
at another sends the arm somewhere else entirely, and there is no safe way to
guess which rows are wrong. A blank cell is not 0 mm; `0.0` is a real height.
A CSV with no such column returns `None` and the lift is never commanded —
that is how every pre-existing task keeps working.

**Sequencing decisions.** The lift is raised *after* arriving at the first tag,
not before driving, so the arm's mass is not high while the base moves. It is
set once and held; the arm homes between groups but the lift does not.
`task_flow.lift_home_on_finish` in `robot.yaml` gates the tail, and it does not
run after a preempt or a failure: a preempt means something else was asked for,
and a failure should leave the robot where a human can look at it.
*(A `return_home_on_finish` flag also landed here and was removed the same day
— see the entry above.)*

**Two sources of truth, resolved.** `lift_height` and
`navifra.scan_height_counts` both name a scan height. The CSV wins, the
per-group `_check_lift_scan_height` guard is skipped for such tasks, and the
conflict is warned about exactly once per task instead of once per group.

**Changed:** new `lift_client.py`; `task_manager.py` (loader, `lift_heights`,
`get_lift_height()`, `scan_joints_line1_lift` task def); `task_executor.py`
(`LiftClient`, `_set_task_lift_height`, `_finish_task`, scan block);
`robot.yaml` new `task_flow:` block; new `task/csv/joints_line1_lift.csv`
(328 rows, `lift_height` = 150.0 mm = 3013 counts on every row).

**Verified offline only**, with `rospy` / `std_msgs` / `std_srvs` stubbed and
every `apriltag_nav` module except the real `task_manager` / `lift_client` /
`task_executor` replaced. The full sequence reproduces as
`arm_home → move 105 → lift_goto 150.0 → scan 105 (169 pts) → move 106 →
scan 106 (159 pts) → arm_home → lift_home`; a lift failure aborts before
scanning; a column-less task never touches the lift; `go_home` and `GOTO 107`
make one trip each. The race above is covered by a
test that fails on a snapshot implementation. **Not verified:** nothing ran
against hardware or a ROS master, and `catkin_make` was not run (`devel/` is
unbuilt) — the user is checking on the robot PC.

⚠️ The user's scenario named tags **106 → 107**; `scan_joints_line1` actually
uses **105 → 106** (line2 uses 117 → 118). Coded to the CSV, not the message.

### 2026-08-10 — Lift scale, stroke and speed corrected from the robot

User supplied three facts off the actual machine, two of which contradicted
what was written here.

**`mm_per_count` 0.05 → 0.0497795.** Was 350 mm / 7000; now 350 mm / **7031**,
the re-measured stroke. 0.44% smaller, ~1.5 mm over full travel. `STROKE_COUNTS`
in `lifter_node.py` went 7000 → 7031 and `STROKE_MM = 350.0` was added so
the default is computed rather than a copied literal that can drift from the
comment next to it. **`mm_calibrated` stays false** — asked, and 350 mm is the
catalogue figure, never put against a height gauge. So the ratio is *half*
measured, and the startup warning now says which half instead of the blanket
"never measured", which would have been the wrong thing to chase.

**The lift is hardware-capped at 1000 rpm, not 2000.** `~/navifra/param.yaml`
has `up_speed_rpm: 2000` and `post_home_speed_scale: 2.0`, and this file
repeated "~28 s at 2000 rpm" from it. Both are clipped by a setting on the
MDROBOT controller itself: the real ceiling is 1000 rpm (raisable to 16000,
deliberately not). The one honest number in that file was the `home_timeout_sec`
comment, "실측 1000rpm 전행정=28.2s". Consequences: tuning those two params
changes nothing, timing does not differ between origin homing and a post-home
move, and `jog_timeout_s: 35.0` has ~8 s of real margin over a worst-case
0→6700 jog (~26.9 s) rather than the comfortable one implied by a 2000 rpm
assumption. Documented at the config key, because that is where someone about
to "just raise the speed" will be looking.

**One origin pass is sufficient** — confirmed on the robot, so the
`approach_from_below` rule needs no repeat pass or dwell. That closes the
open question left by the 2026-08-07 entry below.

**Changed:** `lifter_node.py`, `robot.yaml` `navifra:`/`base_lifter:`,
`mobile_manipulator.launch`, `docs/lift_arm_base_z_analysis.md` §1 Magnitude,
and this file. **Not verified:** documentation and a constant — nothing was run
against hardware or a ROS master, and the 1000 rpm cap is taken on the user's
word, not read back off the drive.

⚠️ `docs/GUIDE_kr.md` line 653 still says "약 7000 카운트, 전 구간 약 28초
(2000 rpm)" and is now wrong on both counts. Left alone on purpose — the guide
is batched for a single later pass — see the Deferred section above the Work Log.

### 2026-08-07 — `lifter_node` added (7th node, owns the base lift)

The lift was reachable only as raw `/lift/*` topics through `NavifraDevices`,
with no node owning it — the one device in the system without an owner, which
broke the one-owner-per-device rule the rest of the stack follows.

**Ownership decision.** CLAUDE.md said "nothing else in `apriltag_nav` should
touch raw driver topics", and `task_executor` already *reads* `lift_position`
for its scan-height guard, so a strict reading would have made the new node
illegal or forced a rewrite of that guard. Resolved by splitting the rule
rather than bending it: `NavifraDevices` stays the wrapper, `lifter_node`
becomes the sole **writer**, reads stay open to anyone. `task_executor` was
therefore not touched at all.

**Why a node and not just wrapper methods.** Three measured hardware facts
(driver guide §3.5/§6, re-measured 2026-07-28) each need enforcement that has
no home in a stateless wrapper — no upper limit switch, an incremental count
that drifts 1000–1800 per up-down cycle, and absolute commands being silently
ignored before homing. Details are in the Architecture section above.

**Changed:** new `scripts/lifter_node.py`; `navifra_devices.py` gained
`lift_jog()` (relative, works unhomed), `lift_velocity()`, and a `_lift_cancel`
flag so `lift_stop()` actually breaks a blocked `lift_home()`/`lift_goto()`
instead of letting it sit out its 60 s timeout — that was a real latent bug,
a stop request halted the drive but the waiter kept polling. `robot.yaml`
`base_lifter:` block; launch node; `test_all_devices.py` node/service/topic
lists (6 → 7 nodes, 5 → 6 required).

**Verified** offline with a stubbed rospy and two fake lifts: absolute move
refused unhomed, relative jog allowed unhomed, clamping at both ends, busy
lock rejecting a concurrent mover, `goto_scan_height` failing cleanly while
`scan_height_counts` is null, and — the safety-critical one — an up-jog
against a lift that *keeps moving* stops exactly at 6700 and leaves 300 counts
of headroom before the 7000 hard stop, with `stop` sent on the way out.

**NOT verified:** nothing ran against the real lift, or against a ROS master
at all. `catkin_make` was not run (`devel/` is unbuilt in this checkout).
`mm_per_count` is still the nominal 0.05 — *superseded 2026-08-10, see the
entry above.* The `arm_base_z` coupling is unchanged: the node now *warns*
when it moves off `scan_height_counts`, which is still `null`, so that warning
is inert until someone fills it in.

**Follow-up the same day, on user feedback:**

- *Terminology.* "Homing" was being used for both the lift's origin recovery
  and the arm's home pose, which is genuinely ambiguous in a workspace where
  one device sits on top of the other. Split into **lift origin homing** vs
  **arm home pose** everywhere, with a comparison table in CLAUDE.md. There
  are now known to be *three* separate `auto_home_on_start` settings (this
  node's, the driver's in `~/navifra/param.yaml` — which is `true`, so the
  lift does origin-home on robot boot — and the driver package default); the
  table exists so nobody conflates them again.
- *Backlash.* User flagged position error from drive backlash. Added
  `approach_from_below` (default true): the final motion of every absolute
  move is an ascent, so a target at or below the current position takes a lift
  origin homing pass first, then climbs. Consequence worth knowing: an
  absolute move while un-homed now **homes first** rather than being refused —
  the old refusal only survives with the rule disabled.
  `_approached_from_below` tracks the last target reached on an ascent so a
  repeat move is a no-op instead of a needless full cycle, and jog / manual
  up / manual down / stop / e-stop all clear it.

  Verified offline: the user's own two examples (50→100 mm direct, 50→25 mm
  via the origin), target-is-the-origin, sub-tolerance descent becoming a
  no-op, each manual command invalidating the state, and a randomised 500-move
  sweep in which **zero** moves ended on a descent (252 via origin, 243 direct
  ascents, 5 no-ops) while staying inside the soft range. Still nothing on
  real hardware. (Whether *one* origin pass suffices was open here; the user
  confirmed on 2026-08-10 that it does.)

### 2026-08-07 — `robot_camera_node` is required, not optional

Corrected a claim that had propagated into CLAUDE.md: `robot_camera_node` was
described as "the one exception to each-is-required — inert whenever
`vision_stop.stop_tag_ids` is the empty placeholder."

That conflates two things. The empty placeholder disables only the **stop
decision**. The node is still navigation's only tag source — `mobile_controller`
consumes `/front_cam/tag_detections` to populate `detected_tags`, which is what
`/robot_pose` is derived from. Without the node there are no detections, no
pose, and `GOTO` cannot work. Five of the six launch nodes are required;
`camera_viewer_node` is the only genuinely optional one.

Propagated to: CLAUDE.md Architecture, `GUIDE_kr.md` §2.1 and Appendix B item 7,
and `tools/test_all_devices.py` `EXPECTED_NODES` (`robot_camera_node` promoted
to required in the same pass as the entry below). Guide PDF regenerated at the
user's request — 34pp.

**Watch for the inverse error too:** side_cam and hand_cam detections genuinely
have no consumer. "This camera's output is unused" and "this node is optional"
are different statements; the node publishes for all three cameras.

### 2026-08-07 — Documentation truth pass (README / CLAUDE.md / guide)

Audited the docs against the actual code and removed contradictions. The docs
had drifted far enough that several statements were the exact inverse of the
code.

**What was wrong and how it was confirmed:**

- **README described a single-process architecture as "Plan A (Current)"** and
  the real multi-node design as "Plan B (Planned)". `task_executor` actually
  imports `ArmClient` and talks over ROS (`arm_client.py:100`), so the labels
  were backwards. Rewrote as the six-node layout.
- **README documented a USB/OpenCV camera fallback** (`use_webcam:=true`). No
  such thing exists — `camera_interface.py` imports `pypylon` unguarded, so it
  is a hard dependency. Grep for `use_webcam` across the tree: 0 hits.
- **README's scipy guidance was inverted** — it told developers to use
  `as_dcm()`/`from_dcm()` and avoid `as_matrix()`. Verified with grep:
  0 occurrences of the `dcm` spelling in `src/`, 15 of `as_matrix|from_matrix`
  across 8 files. See the scipy note under Coding Conventions for why the old
  spelling cannot work here.
- **`/rgb` and `/camera_info` were still documented as live topics.** They were
  removed from `robot.yaml`; navigation consumes `front_cam_detections` now.
- **"launch starts all five"** — it starts six. `camera_viewer_node` was missing
  from the node table and from the `scripts/` file count.
- **side_cam was documented as the `realsense2_camera` apt package.** It is a
  source build under `src/realsense-ros`; see the D405 note in Tech Stack.
  Confirmed: `dpkg -l | grep realsense` is empty, `src/realsense-ros/` exists.
- **`docs/WORKLOG_2026-08-05.md` was referenced but never existed** — replaced
  that dangling pointer with this section.

**Code change:** `tools/test_all_devices.py` `EXPECTED_NODES` was stale — it
listed `/keyence_dlen1_node` as "commented out in the launch file" (it is at
`mobile_manipulator.launch:40`) and omitted `camera_viewer_node` entirely, so
both reported `SKIP` regardless of reality. Now all six are checked, five
required. Also added the four `set_enabled` services the script predated.
Syntax-checked and `--help` verified; **not exercised against live hardware.**

**Deleted as superseded:** `docs/USAGE_kr.md` (documented only 4 nodes),
`readme.txt` (told users to start `roscore`, which the Navifra service owns).

**Doc precedence when sources disagree:** `CLAUDE.md` + `config/robot.yaml` +
`docs/GUIDE_kr.md` win over `README.md`.

**Still open / not verified:** everything in the guide's Appendix B checklist —
notably the empty `vision_stop.stop_tag_ids` placeholder, the `arm_base_z`
1.025-vs-0.9541 discrepancy, and `keyence_max_step_mm` still at the 1.0
bring-up guard. None of this session's work touched runtime behavior on the
robot; only one Python file changed and it is a read-only diagnostic.
