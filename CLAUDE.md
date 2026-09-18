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

**Read `docs/HANDOVER.md` before substantive work** — it is the short
current-state summary (rewritten 2026-09-02): what is verified on hardware,
what is still open, and the interim operating rules. This file holds the
detail and the reasoning; HANDOVER.md holds the checklist.

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
roslaunch apriltag_nav mobile_manipulator.launch   # starts all nine nodes
                                                   # + rosbridge on :9090 (use_rosbridge:=false to skip)
                                                   # + the WEB operator UI on :8080 (use_web_ui:=false)
```

The Navifra systemd service already runs `roscore` — do not start one.
`rosrun apriltag_nav task_executor.py` starts *only* the orchestrator and is a
debug path, not the way to bring the stack up.

## Where run output lives — inside the workspace (2026-09-14)

**User rule: every result and every record a run produces is stored in
this workspace, never in `~/.ros`, `/tmp` or `$HOME`.** The pre-existing
output (2026-09-02..14) was moved in the same session; what was not worth
keeping was deleted (see the Work Log entry).

```
<ws>/log/apriltag_nav/ra_maps/<task>_ra_map_<ts>.csv task_manager result_dir  (versioned)
<ws>/results/scan_images/<task>_ra_map_<ts>/*.png  arm_node output_dir, ONE FOLDER PER RUN (ignored)
<ws>/results/captures/                              robot_ui Collect tab       (ignored)
<ws>/log/apriltag_nav/nav_log/<day>/<ts>_<cmd>.yaml mobile_controller alignment_result_dir
<ws>/log/apriltag_nav/calib_pair/                   the 2026-09-08 front_cam tilt-fit snapshots
<ws>/log/path_tag_locator/{calibrate,locate,handeye_calib}/, map_world_*.yaml
                                                    locator default_save_dir / handeye run_root / map_out
<ws>/log/ros/<run_id>/                              roslaunch + node logs, ROS_LOG_DIR (ignored)
```

⚠️ **`ra_maps/` moved from `<ws>/results/` to `<ws>/log/apriltag_nav/` on
2026-09-15** (user request) — same `<task>_ra_map_<ts>.csv` file, still
`paths.RA_MAP_DIR`, still versioned; it now sits with the other per-run
RECORDS (`nav_log`, `path_tag_locator`) instead of next to the large,
unversioned `scan_images` bulk in `results/`. See the Work Log entry.

How the root is found, three ways that must agree: **`MM_WS`** is exported
by the catkin env hook `apriltag_nav/env-hooks/50.apriltag_nav.sh.in`
(sourced by `devel/setup.bash`, together with `ROS_LOG_DIR=$MM_WS/log/ros`
— the ONE thing only the environment can decide, since roslaunch reads it
at start); `apriltag_nav.paths.WS_DIR` / `path_tag_locator.WS_DIR` /
`robot_ui.paths.WS_DIR` take `MM_WS` or derive the parent of the source
space and `setdefault` it into the process environment, so `${MM_WS}` in
`robot.yaml` / `locator.yaml` / `handeye_calib.yaml` resolves in an
unsourced `python3 tool.py` too; the launch files use
`$(eval optenv('MM_WS', dirname() + '/../../..'))`. ⚠️ Not
`$(find apriltag_nav)/../..` — that substitution also matches
`<ws>/devel/share/apriltag_nav` and resolved to `devel/` when tried.
Re-run `catkin_make` after touching the env hook; a shell sourced before
that has no `MM_WS` and its roslaunch still logs to `~/.ros/log`.

The navifra driver's systemd service does not source this workspace and
keeps writing `~/.ros/log/<run_id>/` — leave that directory alone; only
this workspace's runs moved. `.gitignore`: `log/apriltag_nav/ra_maps` and
every yaml / csv / npz record under `log/` are versioned, plus the hand-eye
`samples/*.png` (the input `calibrate()` re-detects over — `!log/path_tag_
locator/handeye_calib/**/*.png`); frames, captures, other png, video and
`log/ros` are not.

## Task Commands

```
TASK <name>                # Names are DERIVED FROM THE FILES in task/csv
                           # (since 2026-09-14; `rostopic echo /task_list`
                           # or robot_ui's Task tab lists them):
                           #   assigned_workpoints_<key>.csv → scan_pose_<key>
                           #       end-effector poses x y z rx ry rz — IK per
                           #       point (seeded from the paired rrt file)
                           #   rrt_final_path_<key>.csv      → scan_joint_<key>
                           #       joint-angle path q1..q6 — MoveJ replay,
                           #       transition/home rows driven through, not
                           #       scanned; world x y z from the paired file
                           # other: go_home            → <task>_ra_map_<ts>.csv
                           #
                           # Today's keys: errorY_p000mm_standoff_{010,030,
                           # 050}mm_height_652mm — SEPARATE tasks because the
                           # standoff changes which tag each work point is
                           # assigned to, not just the offset.
                           # 🛑 target_line 2 (groups 118/119/120) is not
                           #    trustworthy yet — see the scan-CSV section.
                           # ⚠️ scan_joint_* replays a planned trajectory with
                           #    no reach/collision check; only safe if the base
                           #    is at the planned stop of every group tag.
RELOAD_TASKS               # Re-scan task/csv without a restart (refused while
                           # a task runs); republishes /task_list (latched)
CHARGE                     # Operator dock + charge (2026-09-14): lift origin
                           # home → drive to the dock tag 500 → /crevis/charging
                           # true → wait for BMS current. = the charging
                           # manager's battery_return, on demand; works with
                           # charging.enabled false too. robot_ui: Task tab
                           # "Dock & charge", CHARGE chip from /task_state
UNDOCK                     # /crevis/charging false → 0.10 m forward; no
                           # automatic return until the next task
GOTO <tag_id>              # Navigate to AprilTag
                           # Every TASK / GOTO gets exactly ONE camera-centre-vs-tag
                           # record: log/apriltag_nav/nav_log/<day>/<ts>_<cmd>.yaml
                           # (Work Log 2026-09-02/04) — never overwritten, created
                           # at the first arrival (no file if nothing arrived)
TEST_POSE x y z [rx ry rz] # Test pose control (debug)
STOP / STATE               # Emergency stop / query state
EXEC <code> / EVAL <expr>  # Debug execution
```

Each task registers one step per `group_id` in ascending order, and the robot
drives to that tag before scanning its points. The `standoff_010mm` pair is
`[104, 105, 106, 107, 118, 119]`; 030 and 050 differ (`…107, 119, 120` and
`…106, 119, 120`) because the standoff changes the assignment. All of them
route from `START_TAG` 500.

`TaskManager.discover_task_defs()` builds the definitions from the directory
listing (prefix → mode, the rest of the stem is the pairing key); the loader
then validates the rows exactly as before. `TASK_DEFS` still exists for
explicit extras or overrides (e.g. a `groups: [104]` bring-up subset) and is
normally empty. Result CSVs are `<task>_ra_map_<timestamp>.csv`, which
matches neither prefix, so nothing written into task/csv is ever re-read as
path data. `task_executor` publishes `/task_list` (latched JSON: name, mode,
tags, work/traverse point counts, lift height, files) and robot_ui fills its
Task combo from it — no task name is hard-coded anywhere.

`lift_mm` is 0 in every one of these files, so the lift is commanded to its
origin — which is where `arm_base_z` is measured, and what pose-mode IK
assumes unless the live height says otherwise.

### The RRT dialect: a path, not a point list

The `rrt_final_path_*` files are **planned paths**. About 30 % of their rows
are `transition` / `home` waypoints: the arm **drives through them** — that is
the collision-free route, and skipping them would send it straight between
work points instead — but does not settle, run the Keyence standoff loop,
capture, or write a result row there. (Keyence especially: a transition pose
is nowhere near the surface, so the loop would chase a meaningless reading.)
`is_task_waypoint` marks the difference; `task_manager` turns it into a `scan`
flag on the point, `arm_controller` acts on it, and `ScanResultWriter.begin`
skips it so the Ra map has no permanently-empty rows.

⚠️ **The stop-start motion between those rows is INTENDED (user, 2026-09-14:
"끊기며 움직이는 게 정상이었어").** Each row goes out as its own blocking
`MoveJ`, so a resampled straight transition (e.g. group 104's home → point
14: 12 rows of 5.72°, 0.88 s each) is driven as 12 accelerate-decelerate
hops. That is the planner's path executed row for row, not a defect. A
loader-side fold of collinear traverse runs into one `MoveJ` was built and
backed out the same day on the user's instruction — do not re-add it.

Three columns differ from the original dialect, all handled in
`task_manager`'s module-level helpers:

| | RRT dialect | original |
|---|---|---|
| work-point id | `source_point_id` (`point_id` is the path index) | `point_id` |
| lift height | `lift_mm` | `lift_height` |
| integer cells | may be `0.000000000000000000e+00` | plain ints |

⚠️ **The pairing key is `source_point_id`.** Each joint file pairs with the
`assigned_workpoints_*` file of the SAME standoff — for world (x, y, z) in
joint mode, for the IK seed in pose mode. Pairing on `point_id` instead
matches only 797 of 1035 rows and silently drops the rest. Note the pose file
has no `source_point_id` at all: there, `point_id` IS the work-point id.

### 🛑 The group → tag assignment does not survive checking

Measured from the robot's **STOP pose** (tag − 0.55 m `camera_offset`, which
is what `/robot_pose` reports — not the tag position), every group's work
points are nearest to a DIFFERENT tag than the one it is assigned to, and
all of them cluster near tags 102–104:

| group | assigned, max dist | nearest tag, max dist |
|---|---|---|
| 104 | 104, 1.00 m | 104, 1.00 m ✓ |
| 105 | 105, 1.29 m | 103, 0.92 m |
| 106 | 106, 1.48 m | 103, 0.72 m |
| 107 | 107, 1.72 m | 103, 0.16 m |
| 118 | 118, 4.80 m | 102, 0.94 m |
| 119 | 119, 4.55 m | 103, 0.94 m |

The work points span only **~0.9 × 0.8 m in total** with heavily overlapping
per-group bounding boxes — one area, spread across tags up to 6.4 m apart.
**Only group 104 is reachable.** Needs the generator's author.

⚠️ **Both kinds register since 2026-09-14 (user instruction: run from the
files in task/csv), joint tasks included.** The hazard stands: joint angles
are relative to the arm base, so replaying an RRT trajectory from a base that
is not where the planner assumed leaves the arm's shape unchanged but its
absolute position offset — "collision-free" does not transfer, and
`_exec_joint` is a bare `MoveJ` with no checks. `TaskManager` logs a warning
at registration for every `scan_joint_*` task saying so. Pose mode solves IK
per point, so a mis-assigned group fails the move and says so — run
`scan_pose_*` on a new pair first; that failure is the diagnostic, not a
hazard. (Between 2026-09-11 and 09-14 the joint tasks were commented out in
`TASK_DEFS` for this reason.)

⚠️ **The pre-cell-swap CSVs were deleted 2026-09-11** —
`optimized_joints_line{1,2,3}*`, `grid_path_line{1,2}*` and the
`scan_joints_line*` / `scan_grid_line*` / `scan_full_*` tasks that used them.
They were solved for the retired base (an arm base 373 mm higher than today's
0.652, a drop the whole 343.35 mm lift stroke cannot cover) and for the old
cell's geometry, and `_exec_joint` is a bare `MoveJ` with no reachability or
collision check — running them was a collision path, not merely a bad-data
path. git has them.

Two facts from the previous warning that are still load-bearing:

- `/robot_pose` is derived from the map tag coordinate plus `lateral`
  plus `camera_offset` **plus, since 2026-09-04, the tag's fore-aft
  distance ahead of the lens** (`mobile_controller.calculate_robot_pose`,
  `robot_pose_use_fore_aft`) — needed because the base now stops with the
  tag 12–16 cm ahead of the lens. There is still **no body-width term
  anywhere in the pipeline**, so changing the chassis does not move where
  the robot stops. `wall_dist_*` is documentation.
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
scripts/            ROS node entry points only — 9 files, one per device
                    (plus inference_node, which owns a model, not a device)
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
| `mobile_node.py` | mobile base (**sole publisher** of `/cmd_vel` and `/robot_pose`) | `/mobile/goto_tag`, `/mobile/move_cmd` (manual distance / angle, JSON), `/mobile/{stop,cancel,clear_stop}` (srv), `/mobile/state` |
| `arm_node.py` | Fairino FR10v6 arm | `/arm/scan_command`, `/arm/cancel`, `/arm/move_home` (srv); `/arm/state` (10 Hz, pose kept live through a scan), `/arm/scan_progress` (JSON per point: start / move / done / result / failed / finished — `done` when the frames are captured, `result` when the background inference has the Ra) ; **`/arm/standoff`** (JSON `{target_mm}`, optional — run the Keyence standoff loop from the current pose, completion via `motion_seq`) and **`/arm/standoff_state`** (per Keyence reading: raw, perpendicular, standoff mm, error vs target, out-of-range side) — robot_ui's distance-sensor assist, 2026-09-15 |
| `basler_camera_node.py` | wrist Basler **+ VISION lamp** | `/camera/capture` (srv) |
| `keyence_dlen1_node.py` | Keyence DL-EN1 | `keyence/value` |
| `robot_camera_node.py` | front_cam (Orbbec Femto Bolt) + side_cam (RealSense D405) + hand_cam (RealSense D435) AprilTag detection | `/<cam>/tag_detections`, `/<cam>/tag_overlay` (publish-only) |
| `lifter_node.py` | manipulator base lift (**sole writer** of `/lift/*`) | `/lifter/height_cmd`, `/lifter/{home,stop,reset,goto_scan_height}` (srv), `/lifter/state` |
| `camera_viewer_node.py` | RViz debug window (owns no device). **Not in the launch since 2026-09-04** — `rosrun` it for a UI-less debug session | `/camera_viewer/set_enabled` (srv) |
| `inference_node.py` | the resident ONNX Ra model(s) — on-demand prediction for one frame (owns no device) | `/inference/predict` (srv, `robot_msgs/PredictRa`) |

`mobile_manipulator.launch` starts eight of them (the RViz
`camera_viewer_node` was dropped from it 2026-09-04 — robot_ui shows the
cameras now). Seven are required; the optional one owns no device:
`inference_node` (the TASK scan path scores frames in-process through
`inference_interface.py`, so only `robot_ui`'s on-demand Ra needs the node —
both paths share `RaPredictor` and return bit-identical values).

**Since 2026-09-14 the launch also starts `rosbridge_websocket` (+ `rosapi`)
on port 9090** (`use_rosbridge`, `rosbridge_port`): the external interface
for computers WITHOUT ROS — the operator's Windows PC reaches it as
`ws://192.168.0.20:9090`, the Phoenix Contact AP bridge's WLAN address,
which the bridge port-forwards to this PC's `192.168.1.100:9090` (the
bridge is a NAT router between 192.168.1.x and the site's 192.168.0.x, so
native ROS networking cannot cross it). Clients speak the rosbridge JSON
protocol (`roslibpy`, `roslib.js`, or a bare websocket): publish
`/task_command`, subscribe `/task_state` / `/arm/state` /
`/arm/scan_progress` / `/mobile/state` / `/lifter/state` / `/bms/state`,
call the `/arm/*` / `/mobile/*` / `/lifter/*` services. Nothing in the
stack depends on it. Rules for external clients are the local ones:
**never publish `/cmd_vel` or `/lift/*`** (sole owners, no arbitration),
**no raw image topics through it** (a Basler frame is ~27 MB of base64;
use `web_video_server` / `/compressed`), and it has **no authentication**
— keep 9090 inside the site LAN. A hand-started
`roslaunch rosbridge_server rosbridge_websocket.launch` must be stopped
before the stack launch or the second one fails to bind the port.
Connection check from any machine (no ROS, no stack needed):
`{"op":"call_service","service":"/rosapi/topics"}` over the websocket
returns the topic list.
**Operator tooling: `tools/rosbridge/robot_cmd.py`** (runs on Windows with
only `websocket-client`; task commands, `/task_state` watch, and generic
`topics / type / fields / sub / pub / call / param`) and the Korean
operator guide **`docs/ROSBRIDGE_kr.md`** (reboot procedure for both PCs,
usage, topic table, rules, troubleshooting).

**Since 2026-09-15 the launch also starts the WEB operator UI,
`robot_ui_web_node` on port 8080** (`use_web_ui`, `web_ui_port`): the
whole `robot_ui` operator UI (camera panes with the tag overlays, Collect /
Arm / Task / Mobile / Calibration / Scripts tabs, STOP ALL, log) served as
a page — `http://192.168.1.100:8080` on the LAN, `http://192.168.0.20:8080`
from the Windows PC once the Phoenix bridge forwards TCP 8080 like 9090.
Every browser tab sees ONE shared state (preview, lamp hold, calibration
session, plugin run, log) and any tab may act; frames go out as JPEG on
the same WebSocket (10 fps / 1400 px per camera, one frame in flight per
client), so the "no raw images through rosbridge" rule is not touched.
It owns no device — same `RosBridge` as the PyQt window — needs no X
display, and is a client of the stack: `roslaunch robot_ui
robot_ui_web.launch` restarts it alone. No authentication, same LAN-only
rule as 9090. Operator guide **`docs/ROBOT_UI_WEB_kr.md`**; design in
`src/robot_ui/src/robot_ui/web_server.py`'s docstring.

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

✅ **`align_to_tag()` is bounded since 2026-09-02** — `align_timeout_s`
(20 s, `<= 0` disables) fails the hop with a logged reason (tag not visible /
not converging). **Since 2026-09-04 it leads the stop by the rotation still pending
from the base's 0.55 s command delay, settles, and re-measures at rest
(up to `align_max_passes`)** — the skid-steer "turns a bit further than
the error" twist. **Also since 2026-09-04 it runs after EVERY hop with no
per-tag exception** (user rule: "어떤 상황이든 정지해서 얼라인은 필수") —
the 2026-09-02 `align_skip_tag_ranges` (400→400 lane hops) is retired and
ignored with a warning; the only remaining skip is a temporarily-missing
VIRTUAL tag (`temporary_missing_tags`, off), which has nothing to align to. Before that, a target tag absent from `detected_tags` made it
`stop()`, sleep and `continue` forever, with `MobileClient.move_timeout_s`
(600 s) as the only exit. It now runs after **every** hop — forward, backward
**and pivot** (`_align_after_arrival`; the pivot squares up to its EXIT tag,
e.g. 505 after 501→505). **Since 2026-09-08 the FIRST hop of every command starts
with an align on the tag the base is standing on — forward, reverse,
pivot or dock 500, no exception** (`go_to_next_tag(first_hop=True)`, set
by `move_to_tag` for `path[1]`): a mid-route hop starts where the
previous hop's mandatory arrival align left the base, so nothing is
repeated there, but a command may begin on a tag the base was pushed
onto by hand or be resuming after an e-stop with nothing squared up.
Consequence: the start tag must be in view at rest — a base parked off
its tag fails the command on `align_timeout_s` instead of driving blind
from `last_known_tag`. **Forward / reverse arrivals use `steer_mode:
aim_and_drive` since 2026-09-08 (evening)**: at the first sight of the
target tag the base stops, measures the tag at rest, pivots so its
CENTRE points at the stop pose on the tag's line (capped by the plate
wall, `aim_wall_*`, and by tag visibility, `aim_max_deg`), drives that
straight line, stops on the yaw-corrected column, and the stop align
then lands the lens ON the tag instead of swinging it off by
0.55·sin(yaw) — see the 2026-09-08 aim-and-drive Work Log entry;
`state_feedback` and the smooth-path `tag_line_plan` stay selectable.
**Since 2026-09-09 a forward hop that starts from a REVERSE arrival
re-seats first** (`reseat_forward_from_reverse_column`): the start tag
rests on the REV column ~0.16 m ahead of the lens, so before the hop
the base drives that tag onto the FWD column with the very same arrival
algorithm (aim, straight drive, column stop, align) and only then plans
the hop — every forward hop launches from the standard pose and
`_odom_distance_for_hop` reads the live fore ≈ 0. Skipped when the tag
is already within `reseat_min_fore_m` (0.08) of the crosshair, before
reverse hops, and for virtual tags. **And a command whose LAST hop
arrived in reverse re-seats before `move_to_tag` returns**
(`reseat_at_command_end`, user rule the same day: "후진 기준점 도착 →
전진 전환 → 전진 기준점 도착 → 로봇팔"), so a TASK scan or a
calibration entry does its arm work only at the FWD-column pose; the
following forward command then finds nothing to re-seat. **A pivot then finishes ON its exit tag**: `execute_pivot`
turns on odom only until the EXIT tag is in view, steers on the tag's
edge angle from then on, drops to `pivot_tag_slow_max_angular` (0.05
rad/s) once the predicted settled error is inside `pivot_tag_slow_deg`
(5°), and stops on the delay-led prediction; the exit-tag align then
measures at rest and certifies the 0.2° band (both aligns 0.2°). **`center_x_stop_offset` (currently +50 px)** is what
decides how much tag is left in frame; **more positive** stops earlier and
leaves more margin. **It is 0 since 2026-09-02** (user: the tag centre must
reach the target centre), so the target column is the optical axis itself.
**The target column depends on the direction since 2026-09-04**: forward
hops stop the tag on `cx + center_x_stop_offset` (0 = the crosshair);
reverse hops stop it on `cx + center_x_stop_offset_reverse` (**+400 px**,
the far RIGHT of the frame = the tag ~162 mm AHEAD of the lens — user
request, "후진할 때만 화면 기준 최대한 오른쪽") — **unless the TARGET tag is
in `center_x_stop_offset_reverse_skip_tag_ranges` (`[[500, 599]]`)**: a
reverse hop into a dock / pivot tag stops on the forward column, i.e. the
designed stop pose, because a pivot turns about the base centre and needs
the base ON that pose (user rule, same day). Removing that key restores
the 2026-09-02 one-column rule (reverse stops on the forward column from
the other side; a cx − offset mirror to the LEFT was tried and backed out
that day — the opposite side of this one). `robot_camera_node` draws both
columns on `/front_cam/tag_overlay` (cyan `FWD stop`, magenta `REV stop`,
from the same two keys). Because a reverse arrival leaves the lens 162 mm
short of the tag and `/robot_pose` has no fore-aft term, `go_to_next_tag`
corrects the next hop's odom distance (`_odom_distance_for_hop`: edge ±
(where the lens rests vs the start tag − where the stop column puts it),
read live from the start tag when visible) — without it a reverse hop ran
its last 0.16 m on the end-of-profile crawl and tripped the 30 s watchdog.
Expect the measured 6–8 mm post-trigger roll to leave the tag ~15–20 px past
either column; raising the offset by ≈ +18 would pre-compensate it. (It was `center_y_stop_offset: -50` until the 2026-08-13 camera
rotation — same physical stopping point, opposite sign, because the fore/aft
image axis flipped from the row axis to the column axis. Measured: the stop
lands at body x = 0.567 m either way.)

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

### front_cam ground-plane correction — the camera is tilted 1.3° and it matters

**Since 2026-09-08 `robot_camera_node` re-images front_cam's detections
through a level virtual camera** (`robot_camera.ground_plane.front_cam` in
`robot.yaml`, module `apriltag_nav/ground_plane.py`): the raw corners are
undistorted with CameraInfo `D`, cast through the calibrated tilt (roll
+1.406°, pitch −0.323° since 2026-09-15 — +1.228 / −0.504 on 09-08, the
0.18° difference being floor-flatness sized; lens 302 mm above the tag
top — the tags are 1 mm plates, `robot.tag_thickness`) onto the floor, and re-projected with
the same K at that height. `/front_cam/tag_detections` therefore carries
flat-view pixels, `pose_x/pose_y` = floor position relative to the lens
NADIR (m, robot frame), `pose_z` = 0.302; `mobile_controller` is
unchanged and its `z / fx` scaling stays consistent. The overlay still
draws the raw frame with raw boxes.

Why: a tilt rotates a tag's imaged edge in proportion to the tag's
fore-aft position — a square-laid tag read **+0.67° at 0.2 m ahead** (where
`aim_and_drive` measures), +0.50° at the reverse column, −0.21° on the
crosshair — and every consumer multiplies that by a 0.55–0.75 m lever.
That was the whole +8.8 mm forward / −11.6 mm reverse lateral bias of the
first aim-and-drive run, AND it means every "aligned" stop actually left
the body 0.2–0.5° yawed. Lens distortion (k1 0.083) moves positions
~4 mm at the frame edge but barely touches angles (0.05°).

How it was measured (`tools/fit_front_cam_ground.py`): two 60 mm tags laid
0.150 m apart (user, 0.1 mm), 8 snapshots of small manual moves + 7 of
±4° in-place pivots, all read AT REST — the commanded motion is never
used, only what the tags show (the base under-executes small moves by
~45 % and pivots by ~25 %). 120 corner points, rms 0.33 px (0.74 with a
level camera). The same session put the **lens-to-pivot lever at
0.552 ± 0.001 m** (`camera_offset` 0.55 stands) and the camera yaw vs the
travel axis at −0.38 ± 0.22° (from a straight-drive test; not corrected,
1.3 mm per 0.2 m). Re-fit after any camera remount: snapshots must be
taken with `ground_plane.enabled: false` (raw detections). One of the two
tags (15) had corners skewed ~0.4° (print or glare) — use the pair's
centre line, not a single tag's edge, as the angular reference.

Consequence to remember: "lens over the tag" is now the nadir, 7 mm from
where the optical axis meets the floor, so `/robot_pose`'s lateral and
every stop moved by that constant relative to pre-2026-09-08 records.

**`yaw_deg: -0.38` (2026-09-09)** adds the camera's rotation about its
optical axis vs the TRAVEL axis, which the tag pair cannot see: it is the
edge angle a tag laid parallel to the lane reads on the crosshair when
the body is parallel to it, from the 2026-09-08 straight-drive test
(−0.38 ± 0.22°). With it the virtual camera's x axis is the travel axis,
so "aligned" means body-parallel and the aim's base offset is estimated
in the body frame. Caveat: that test used the RAW pipeline's pose `yaw`
as the edge, so up to ~0.1–0.2° of tilt bias may be folded in; the
forward hops of 2026-09-09 (residual +1.9 mm) suggest ~−0.45°. Re-measure
with the same straight-drive test with the correction ON — the residual
drift is then the error of this number.

### Reading tag ID and orientation off the image

`robot_camera_node` publishes `/<cam>/tag_overlay`. **Layout since
2026-09-09 (user request): no pixel numbers anywhere.** An amber crosshair
on the calibrated principal point; on front_cam the dashed FWD / REV stop
columns labelled in **mm** from the crosshair; per tag a marker, a leader
line and the ID; and top-left, one block per tag, one line per kind —
`ID n` / `offset: (x mm, y mm)` (x along the horizontal centre line,
+ = image right = forward; y down, + = robot right; these are
`pose_x / pose_y`) / `degree: ±d.dd` (the corner0→corner1 edge vs the
horizontal centre line, 0 = square — the number the align drives to 0).
**With the ground-plane correction on, the overlay frame is the raw image
RECTIFIED to the level virtual camera** (`GroundPlane.rectify`, ~7 ms,
only while subscribed), so the crosshair is the lens nadir and the
boxes, columns and mm numbers all refer to the same view the controller
uses; a tag resting on the REV line reads `offset: (+161 mm, …)`.

Rendered only while something subscribes, so it costs nothing when no viewer
is open. The prepared RViz layout shows front_cam's overlay first.

`AprilTagDetection` carries the same numbers for consumers. Orientation comes
from `pose_R`, which the node previously discarded. The overlay shows only
yaw; roll/pitch/tilt stay in the message for anything that needs them.

**Latency levers (2026-09-15, `robot_camera.detect_max_hz` /
`detector_threads` / `overlay_hz`):** the `tag_overlay` is rendered on a
timer thread at 10 Hz from the newest frame, never inside the detection
callback (rectify 16 ms + drawing 15 ms used to sit in front of every
following frame); side_cam / hand_cam detect at 10 Hz (they only serve
the calibration session — its 5-frame medians now take 0.5 s instead of
0.2); front_cam's detector runs 2 threads. Measured before: front_cam
detections 0.33 s after the image stamp (image 0.09) with the node at
350 % CPU; after the restart 0.12 s (0.10–0.27) at 30.2 Hz, side / hand
10.0 Hz, node 244 %. `tools/check_robot_camera_latency.py` (13) pins the
behaviour offline.

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

### Looking at the cameras — robot_ui (web or PyQt), or `camera_viewer_node`

**Since 2026-09-04 the operator UI is the camera viewer, and since
2026-09-15 that UI is a web page** (`robot_ui_web_node`, port 8080, in
the main launch — any browser on the LAN; the PyQt window
`robot_ui.launch` still exists with the same panes and shares every line
of behaviour through `robot_ui.web_ui.UiController` / `MainWindow` on
one Qt-free `RosBridge`). Its left pane
shows the Basler plus all three tag cameras; each tag camera defaults to
`robot_camera_node`'s `/<cam>/tag_overlay` (crosshair, stop columns in mm,
per-tag ID / offset (mm) / degree) with a `tags` box to fall back to the raw stream, an `on` box
that calls `/robot_camera/<cam>/set_enabled`, and a `tags: 105, 106` readout
from `/<cam>/tag_detections`. A single click on any thumbnail makes it the
main (large) view; double-click still maximises. The launch therefore no
longer opens RViz.

`camera_viewer_node` remains as a UI-less fallback
(`rosrun apriltag_nav camera_viewer_node.py _auto_start:=true`). Debug aid,
separate from `robot_camera_node` so the detection path has no GUI in it. It owns no device and publishes nothing; it only starts and stops RViz
against `config/robot_cameras.rviz` (Image displays for all three cameras).

```bash
rosservice call /camera_viewer/set_enabled "data: true"    # open
rosservice call /camera_viewer/set_enabled "data: false"   # close
```

It starts **closed** — bringing the stack up must not throw a window on
screen, and a headless boot may have no display at all. `~auto_start` true
overrides that for a debug session.

When it was in `mobile_manipulator.launch` (until 2026-09-04) that was
specifically to bound its lifetime: roslaunch shut it down with everything
else, and its `rospy.on_shutdown` kills the RViz child (still true under
`rosrun` + Ctrl-C). RViz is spawned with
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

**Every current scan CSV carries 0 mm** (the 2026-09-11 RRT set spells the
column `lift_mm`; `task_manager` aliases it), so the lift is commanded to its
origin — which is where `arm_base_z` is measured. Scale is 0.04976077 mm/count,
so 300 mm would be 6029 counts (~24.2 s) and 150 mm 3014 counts (~12.1 s), the
values the retired line1 / line2 CSVs used.

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

⚠️ **A task built from several CSVs needs them all to agree.**
`_extract_lift_height` refuses both a partly-filled column and one whose
values disagree, and **unregisters the task** rather than picking a winner —
the joint angles were solved at one base height, so guessing which rows are
wrong is not safe. This is what unregistered the old `scan_full_joints` when
its two halves were set to 300 and 150. The current tasks are one CSV each at
0 mm, so it does not bite today; it will the moment two are concatenated.

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

#### Charging manager — `task_executor` (2026-09-09)

**Operator entry points since 2026-09-14: `CHARGE` and `UNDOCK` on
`/task_command`** (robot_ui Task tab buttons) — the same two internal
tasks the rules below queue, on demand, and independent of `enabled`.
`/task_state` carries `charge_phase` / `charging` / `battery_pct` for the
UI's CHARGE chip.

User rules, in `robot.yaml` `navifra.charging:`: charge until **85 %**
(`full_pct`), then `/crevis/charging false` and come forward
`undock_forward_m` (0.10); at **20 %** (`return_pct`; 30 → 20 on the
user's instruction the same day) abandon whatever is running and go back
to the charger; **after a user TASK completes
normally, go back to the charger too** (`return_after_task`) — but never
after a `GOTO`, which is positioning, not work (`return_after_goto`
false; found on the robot 2026-09-09 when `GOTO 100` was followed by an
unasked-for drive back to the dock); after
docking `/crevis/charging true` MUST be sent — the charger only starts
on that explicit command (user-confirmed) — and the BMS must show
current within `charge_confirm_s`. Lamp: `status_colors.charging`
(**magenta**) while the BMS reports current into the pack, `charged`
(**white**) once full, undocked and waiting for a command; plain idle
stays green and error red, so the four rest states are distinguishable
(user's colour choice, 2026-09-09). Task states keep their colours.

**Since 2026-09-15 the automatic low-battery return is OFF**
(`low_battery_return: false`, user: "20 % 이하 자동 복귀 발동 안 되게"):
neither the mid-task abandon nor the idle return at `return_pct` fires;
`CHARGE` / `UNDOCK`, the 85 % undock and `return_after_task` are
unchanged, `true` re-arms it. Same day: `/task_state` is republished
whenever the battery moves 0.5 % (it is latched and was otherwise only
resent on a state change, so robot_ui's CHARGE chip showed a stale
percentage next to the live BAT chip), and both UIs now print the LIVE
`/bms/state` figure on the CHARGE chip, `/task_state`'s as the fallback.

Mechanics: `_charge_tick()` runs every main-loop tick (`_tick()`, the
old `run()` body) and only QUEUES two internal tasks — `battery_return`
(lift origin home → `move_to_tag(dock_tag)` → optional `dock_reverse_m`
→ `charge_on`) and `battery_undock` (`charge_off` → `drive_m` forward
via the new `MobileClient.drive_distance`, mobile_node `/mobile/move_cmd`)
— which go through the ordinary task machinery: a user TASK/GOTO
preempts them, the safety gate applies, `/task_state` and the lamp follow.
"Charging" is judged by `NavifraDevices.charging_by_bms()` (current >
`charge_current_min_a` or status CHARGING), never by the relay feedback
topic alone. Phases: working → returning → charging → full → (working…);
**STOP** parks the manager (`stopped`) until a task runs again, and a
docking that produces no current ends in `dock_failed` and is NOT retried
(the arrival records + `[Charge]` log lines say why). `enabled: false`
turns all of it off.

⚠️ This deliberately overrides the 2026-08-10 "tasks never drive home"
rule for the charge return only, on the user's decision, and only when
nothing else is queued — a GUI that queues "scan, scan, scan" still runs
all three before the robot returns. ⚠️ Assumed, not verified: the
designed stop pose on dock 500 (reverse arrival, crosshair column) makes
the charger contacts; if it needs an extra push, `dock_reverse_m` is the
knob, and if 10 cm forward does not break contact raise
`undock_forward_m`. Offline checks: `tools/check_charging_manager.py`
(13: charge → 85 % undock → lamp; 29 % mid-task preempt → return → true
→ confirmed; return after a completed task; return_after_task off;
STOP disarms; dock without current → dock_failed, no retry; disabled).

#### Tasks are composable blocks — none of them drives home

**A task never returns to `START_TAG` on its own.** Returning is its own task
(`TASK go_home`), so "scan then come back" is two commands and "scan then scan
somewhere else" is two different ones, built from the same pieces. This is
deliberate and is the shape the planned block-coding GUI needs: each block is
one self-contained task, they run in the order they are placed, and the system
waits at IDLE between them. Anything that auto-appends motion to the end of a
task breaks that composition — a brief `return_home_on_finish` flag was tried
on 2026-08-10 and removed the same day for exactly this reason. (The
one exception, since 2026-09-09, is the charging manager's
`return_after_task`, which appends a charge return only when the queue
is empty — see the section above.)

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
- **Operator lamp HOLD (2026-09-15):** `/camera/set_lamp` (Bool) makes the
  node hold the lamp on for aiming with the live preview — robot_ui's
  "VISION lamp" box on the Collect tab, which follows the latched
  `/camera/lamp_state` rather than its own click. The hold opens the device
  and is dropped by `_close_locked` whoever closes it (idle timer, preview
  off, STOP ALL, shutdown), so the lamp still cannot outlive the shutter; a
  capture under a hold neither toggles nor flushes.
- ⚠️ **Black frames after lamp-on — a timing defect, fixed 2026-09-15.** The
  camera free-runs at 5 fps under `GrabStrategy_LatestImageOnly`, and the
  acA5472's readout + GigE transfer of a 20 MB frame takes about the whole
  200 ms period, so the frame `RetrieveResult` returns right after
  `vision_led(True)` + the 150 ms warmup was EXPOSED (4 ms, gain 0) before
  the lamp: completely black. The operator saw it in bursts; a survey of
  240 of the 1273 scan frames of 2026-09-14 found **7.1 % black (mean
  < 5/255)**, every one scored **Ra ≈ 0.082** — the model's answer to a
  black image, so those Ra map rows are wrong. Fix in `_handle_capture`:
  `lamp_flush_frames` (1) frames discarded after this call switched the
  lamp on, and `_grab_burst` re-grabs any lamp-on frame with mean <
  `dark_frame_mean` (4.0) up to `dark_frame_retries` (2) times (a slow
  relay); lamp-off preview frames are never re-grabbed. The response
  message says what happened (`flushed 1 pre-lamp frame`, `1 dark frame
  re-grabbed`, `lamp held`). Cost: one extra frame (~200 ms) per lamp-on
  capture. `tools/check_basler_lamp.py` reproduces the defect against a
  one-period-delayed camera model and pins the fix.
  **The dark check is per horizontal BAND (8), not the whole-frame mean**
  (same day, second report: "part of the image black" on fast repeated
  captures): the acA5472's IMX183 is a rolling-shutter sensor, its rows
  are exposed one after another across the ~200 ms readout, so a lamp
  switch inside that window leaves a frame lit on one side and black on
  the other — mean ~90, passes a whole-frame threshold. With flush 1 that
  straddling frame is the NEXT one about a quarter of the time; the band
  check re-grabs it.
- **CAPTURE works during the live preview (2026-09-15).** It used to be
  greyed out whenever any pooled call was in flight, which at 5 Hz is
  almost always; preview grabs are now counted separately
  (`_run(..., preview=True)`), and a capture PAUSES the preview timer for
  the shot instead of switching the preview off (the device stays held,
  no close/reopen mid-capture, no interleaved lamp-off frame) and resumes
  it afterwards. `_live_workers` keeps a strong reference to every
  in-flight `CallWorker`: the pool auto-deletes the C++ runnable and the
  Python wrapper with its `_WorkerSignals` could be collected mid-run
  (`wrapped C/C++ object has been deleted` — seen in the offscreen check),
  after which `finished` never fired and `_busy_calls` leaked, greying
  CAPTURE for the rest of the session.

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
tool Z before each capture. **Since 2026-09-08 the loop itself is
`src/apriltag_nav/keyence_standoff.py` (`StandoffController`, pure logic,
offline-tested)**; the controller only projects the reading by
`cos(beam_angle)`, converts approach mm to tool Z with `keyence_dir`, and
does the MoveL. What the rewrite changed: it engages over the whole sensor
range (`activate_threshold` 20 mm, was 5), an approach step is at most half
the MEASURED gap while far and `max_step_mm` (1.0) near the target while a
retreat may be 3 mm, a `max_travel_mm` budget bounds the whole adjustment,
every decision is the median of 5 readings that arrived AFTER the last move
(a cached value cannot drive the arm; a silent sensor means no motion), a
reading that does not follow the motion aborts, the gain adapts DOWN to the
measured sensitivity on sloped material, `keyence.target_distance_mm` is
live (default 10 = the sensor zero), and the outcome is written into the
CSV row's `execution_message` (`require_converged` makes a failed standoff
fail the point). `seek_enabled` (step toward an out-of-range side) is OFF
until the sentinel's sign is confirmed on the real sensor. Not yet run on
the robot. Three things about it are not guessable from the code — full
record in `docs/keyence_scan_chain.md`:

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
- **`keyence_max_step_mm` is 1.0 and is doing real work.** The oblique beam
  walks the laser spot `0.919*dz` sideways per correction, so on a sloped
  surface the effective sensitivity is much larger than the calibrated 1.358
  — one observed step hit 7.52, past the divergence limit of 3.40. The clamp
  is what kept that from running away; since 2026-09-08 it is the approach
  FINE step and the adaptive gain backs it up, but don't raise it without
  reading the open-issues section of the doc.

### ✅ The lift no longer breaks `arm_base_z` (2026-09-11)

`arm_base_z` is still measured at the lift origin, but pose-mode IK now
subtracts the live extension: `transform_world_to_arm(g, msg, lift_m)`,
fed by `_exec_pose` from `lift_height.LiftHeightListener` on the latched
`/lifter/height`. Before this, a pose-mode scan at a raised lift was
silently offset by the full travel.

**Direction, because it is not guessable and it changes how alarming this
was:** the mount has no tilt, so `R_AW` is a pure z-rotation and
`p_arm[2] = z_world − arm_base_z`. Omitting the lift makes that term
`lift_m` too LARGE, and the arm — whose base really is that much higher —
puts the TCP `lift_m` **ABOVE** the target. So the old behaviour scanned
too high, *away* from the plate: a wrong measurement, not a collision. It
is not self-correcting either, because the Keyence standoff loop is
clamped to `keyence_max_step_mm` (1.0 mm) per step and cannot close a
150–300 mm gap.

- **Reading only.** `LiftHeightListener` is a separate, read-only class
  rather than the existing `LiftClient`, which also exposes `height_mm`:
  `LiftClient` is the COMMAND proxy (it publishes `/lifter/height_cmd` and
  holds home/stop proxies) and `arm_node` has no business being able to
  move the lift. `lifter_node` stays the sole writer; reading is open.
- **Unknown ≠ origin.** `height_m()` returns None before the first
  message instead of 0.0, and `arm_node`'s `~require_lift_height` decides:
  false (default) assumes the origin — today's convention, since every
  task ends with lift origin homing and pose-mode CSVs carry no
  `lift_height` — but logs it at **error** level each time, because it is
  an assumption. true fails the move instead; set it for unattended runs.
- **`lift_m = 0` is bit-for-bit the old result**, so nothing that runs at
  the origin changed.
- Joint mode is untouched: those CSVs are absolute joint angles fed to
  `MoveJ` and no transform reads them.
- ⚠️ `tools/arm_controller_sdk.py` holds a SECOND copy of this geometry
  (`_transform_pose`) that did **not** get the fix — flagged in its
  docstring, alongside the scipy-compat work it already needed.

`tools/check_lift_compensation.py` (14 checks) pins the sign, that only z
moves, and the unknown-height policy. See also
`docs/lift_arm_base_z_analysis.md` and the `scan_height_guard` in
`robot.yaml` — the *guard* is still the thing that stops a joint-mode task
from scanning at an unexpected height.

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

⚠️ **CSV orientation is PER GENERATOR — `arm_calibration.csv_euler`
(2026-09-14).** It is the scipy `from_euler` spec applied to
`[rx, ry, rz]`. The 2026-09 RRT planner's `assigned_workpoints_*` need
**`"ZYX"`** (intrinsic, R = Rz(rx)·Ry(ry)·Rx(rz) — its "rx" column is the
yaw): checked against the FK of its own paired joint rows, **0.00° over
943 points**, while the `"zyx"` the code had hard-coded read them **180°
off, tool pointing up**. The deleted pre-cell-swap `grid_path_line*.csv`
were the opposite (`"zyx"` gave the tool z-axis (−0.089, −0.037, −0.995),
down at the plate; ZYX gave horizontal — the 2026-09-11 finding). Both
findings are real; they are about different files. Verify a new generator
with `tools/check_pose_vs_joint.py` and change the key, not the code.

⚠️ **The CSV x y z are the VISION TIP, but the controller's active tool
frame was the FLANGE (2026-09-14).** `GetInverseKin` has no tool argument
— it solves for the controller's ACTIVE tool — and `/arm/state` at the
home joints read (−159, 700, 774) mm, the URDF **flange** to 0.0 mm, so
tool 1's (0, −253, 225.2) offset from `set_tool_tcp.py` was not in effect.
A tip target sent as-is therefore put the flange there: the tip landed
**338.7 mm** from where the paired joint row put it ("joint 값은 정확한데
pose는 다른 곳"). `ArmController._probe_tool_frame` now reads
`GetTCPOffset` at start: flange → `_exec_pose` converts tip → flange
(`flange = tip − R·offset`) before IK; tip → sends as is; anything else →
pose mode refused. Deliberately NOT fixed by activating tool 1 on the
controller: `/arm/state`, `move_cart`, the calibration seeds and the
hand-eye `T_hc2ee` are all expressed against what the controller reports
today (the flange), and activating the tip would shift every one of them
by that 339 mm. If tool 1 is ever activated, the probe sees it and the
conversion switches itself off — but the calibration chain has to be
re-expressed first.

Target orientation for IK comes directly from CSV (through
`process_transforms`) — EE orientation barely changes across a scan, so the
CSV quat is reliable.

⚠️ **`transform_world_to_arm` takes `lift_m` since 2026-09-11** and
`_exec_pose` feeds it the live `/lifter/height`. Omitting it (or leaving it
0 while the lift is raised) puts the TCP exactly the lift extension **ABOVE**
the world target — see the lift section below.

## Transform Parameters (4-DOF physical model)

Source of truth: **`path_tag_locator/config/extrinsics.yaml` `T_ab2mb`**
(platform-measured; that file explicitly deprecates earlier tunings). The
mount has **no tilt** — R is exactly Rz(180°) — which a 655-point real-robot
fit (tilt ≈ 0.0001/0.0007 rad) confirmed independently. ⚠️ That fit's data
(`task/csv/calib_data*`) was old-base and was **deleted 2026-09-11**, so the
number survives only as this note; git has the files. The USD-derived values
used before (base_z 1.0076, tilts −1.3°/+1.5°) are superseded; those tilts do
not exist on the real platform.

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

**`arm_base_z` is 0.652.** It read **0.651** between 2026-08-13 and
2026-08-23, when it was corrected by 1 mm to match the cell design record's
652 — see §3 of the parent directory's CLAUDE.md. Work Log entries citing
651 predate that correction.

⚠️ Anything older than 2026-08-13 describes the **retired** mobile base and
is not a discrepancy to chase — the base was replaced, not re-measured.

**The value is measured with the lift at its origin** and it is still a
constant — but since 2026-09-11 pose-mode IK **adds the live lift extension
on top of it** (`transform_world_to_arm`'s `lift_m`, from
`/lifter/height`), so the arm base is tracked to its real height of up to
0.995 m at the top of the stroke. The old 343 mm pose-mode error is closed;
see *The lift no longer breaks `arm_base_z`* above for the direction and
the unknown-height policy. Joint-mode tasks were never affected.
`docs/lift_arm_base_z_analysis.md` still holds the background (its §4.2 is
now obsolete).

**`T_mb2fc` is the PHYSICAL front_cam since 2026-09-15 — translation
(0.55, 0, 0.303), rotation = level camera × the 2026-09-08 ground-plane
fit — re-measured 2026-09-15: roll +1.406°, pitch −0.323°, yaw −0.38°
kept from 09-09; optical axis 1.443° off vertical — and it is
GENERATED, never hand-edited:**
`path_tag_locator/scripts/make_front_cam_extrinsics.py --apply` derives it
from `robot.yaml` (`camera_offset`, `camera_lateral`,
`ground_plane.front_cam` roll/pitch/yaw, `height_m`, and
**`robot.tag_thickness`**). **tz = height_m + tag_thickness = 0.302 + 0.001
(user, 2026-09-15: every laid tag is a 1 mm plate, so the plane the
corners are detected on — what the tag-pair fit measures `height_m`
against — sits 1 mm above the floor / mb origin; a floor tag located
through the chain therefore lands at z = +0.001 in mb, and `robot_sim`
lays its floor tags there).** Navigation is untouched: `pose_z` and the
`z / fx` pixel scale are lens-to-tag-plane distances. tz had moved
0.300 → 0.302 earlier the same day (user: the tape figure and the fit's
lens height must be one value). Before that
the translation was (0.55, 0, 0.300) from 2026-08-21, `(0.547, 0, 0.300)`
from 2026-08-13 and `(0.45, 0, 0.293)` before the base swap — the height
moved only 7 mm across a 374 mm deck drop because the camera is mounted off
the chassis, not the deck. `tx` must stay equal to `robot.yaml`
`camera_offset`, the only key in that block any code reads.

⚠️ **Two front_cam frames now exist, and the chain must use the one its
detections are in.** `robot_camera_node` re-images front_cam's detections
through a LEVEL virtual camera while the ground-plane correction is on,
so a chain fed from `/front_cam/tag_detections` needs
`T_mb2fc_level = T_mb2fc @ T_tilted_to_level(fit)` (rotation exactly
diag(1, −1, −1), same lens centre) — feeding it the stored tilted matrix
applies the tilt twice (8 mm / 1.38° on a floor tag). Only a consumer
that re-detects RAW frames (`verify_arm_pointing.py`) takes the stored
matrix. `path_tag_locator.constants.load_extrinsics_full()` derives the
level frame (`ground_plane.T_tilted_to_level`), picks per
`locator.yaml detector.front_cam_frame` (`auto` = follow
`ground_plane.enabled`), and REFUSES a yaml whose rotation or tz no
longer matches `robot.yaml` (0.01° / 1e-6 m) — so an edit to the fit
without re-running the generator fails the calibration nodes at start
instead of shifting every result. `robot_sim` renders through the same
choice. `scripts/check_front_cam_extrinsics.py` (22) pins all of it,
including the tilt's sign against `GroundPlane.project` itself.

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
  for `from_matrix`/`from_dcm`). **Two scipys live on the robot PC**
  (checked 2026-09-02): the Ubuntu system package is **1.3.3** (no
  `as_matrix`), and a user-site **1.10.1** under
  `~/.local/lib/python3.8/site-packages` (installed 2026-08-05) shadows it
  for the default `python3`, which is what roslaunch and every node run.
  So bare `as_matrix()` works today *only* because of that user-site
  install — `python3 -s`, a different user, a `PYTHONNOUSERSITE` env, or a
  `pip uninstall --user` all drop back to 1.3.3 and it crashes at runtime.
  That is the failure the 2026-09-01 review found (an empty
  `/…/tag_detections` array on every frame) and patched in
  `robot_camera_node.py`, `arm_transform.py`, `arm_controller.py`; the
  earlier Work Log claim that "this machine runs 1.3.3" was made against the
  system interpreter, not the default one. Runtime code stays compat.
  `tools/*.py` still hold bare calls, and `tools/arm_controller_sdk.py` is a
  controller variant `arm_node` can be pointed at — make it compat before
  switching to it.

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
| Wherever the 655-point fit appears | its data (`task/csv/calib_data*`) was **deleted 2026-09-11** — old-base, and the base was replaced 2026-08-13. The tilt ≈ 0.0001/0.0007 rad number survives as a note in `robot.yaml` / `arm_transform.py`; do not point readers at the files. |
| Task list / §5 | every `scan_joints_line*` / `scan_grid_line*` / `scan_full_*` task and CSV was deleted 2026-09-11. Current tasks are `scan_grid_standoff{010,030,050}` and `scan_g104_standoff010` (pose mode); the RRT joint tasks are commented out pending a group→tag fix. |
| Appendix B checklist | predates `lifter_node`. (The "`mm_calibrated` is false" caveat it was missing is now moot — measured 2026-08-13, it is true.) |
| Throughout | "homing" is still used for both senses. The workspace now separates **리프트 원점복귀** (lift origin homing) from **매니퓰레이터 홈 자세** (arm home pose). |
| Missing entirely | the `lift_height` CSV column and the task flow it drives; that a task ends with lift origin homing and then **stays put** (`go_home` is a separate task); that absolute lift moves are refused before origin homing. |
| Everywhere | **node names renamed 2026-08-11**: `arm_controller_node` → `arm_node`, `base_lifter_node` → `lifter_node`. Also `robot_controller.py` → `mobile_controller.py` and `RobotController` → `MobileController`. Affects §2.1, §2.4 (line 268 sample output), §5, §7 topic/service tables and the troubleshooting table. |
| Everywhere | **`/base_lifter/*` → `/lifter/*`** in the same pass, and `robot.yaml`'s `base_lifter:` block key is now `lifter:`. Appendix A must follow. |
| Wherever output paths appear | **Every result / record lives in the workspace since 2026-09-14**: `log/apriltag_nav/ra_maps` (moved from `results/ra_maps` 2026-09-15), `results/scan_images/<run>/`, `log/apriltag_nav/nav_log`, `log/path_tag_locator/…`, `log/ros` (`ROS_LOG_DIR`). `~/.ros/…`, `~/scan_results`, `/tmp/robot_ui_captures` and result CSVs in `task/csv` are all gone. |
| §2.1 node table + line 295 | node count is now **8개 중 7개 필수** — `mobile_node` was added 2026-08-11 and is required. |
| §7 topic tables | `/cmd_vel` and `/robot_pose` are published by **`mobile_node`**, not `mobile_manipulator_system`. New: `/mobile/goto_tag`, `/mobile/state`, `/mobile/busy` and the `/mobile/{stop,cancel,clear_stop}` services. |
| Missing entirely | that `task_executor` now owns **no device at all** — drive, lift and arm are each reached through a client proxy. Worth a short section; it is the main structural change since the guide was written. |
| Task list / §5 | `scan_joints_line1_lift` no longer exists (retired 2026-08-13). Both `optimized_joints_line*.csv` now carry `lift_height: 150`, so **every joint-mode scan raises the lift 150 mm** and pose-mode scans still do not. |
| Appendix / §7 | robot footprint is **0.90 x 0.70 m** (was 0.80 x 0.50), `wheel_radius` 0.0825 / `wheel_separation` 0.65, and `T_mb2fc` is **(0.55, 0, 0.303) with the 1.3° tilt in its rotation, generated from robot.yaml** since 2026-09-15 (was `(0.45, 0, 0.293)`); tz = lens 0.302 above the tag top + 1 mm tag thickness. |
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

### 2026-09-18 (afternoon) — Bootstrap sweep ran on the robot twice; the compute needed two more fixes; new-mount hand-eye in use, absolute check open

User: "다시 해봤어요 검증해줘". `run_20260918_144420`: **14:46, from
1.2 m** — file diverged after ONE step (287 → 405 mm, tilt 4.4 → 7.4°),
retreat, bootstrap 7/7 views (HORAUD, t = (−86, −315, 157)), square-up
6 iterations 286 → 155 mm ending at z 0.72 (unconverged, 10 cm clamp),
4 views planned / 20 rejected by clearance, 5 captured. **14:48, from
0.62 m** (where the first left the arm) — the FILE was tried again and
diverged again (the node loads it per call), bootstrap 7/7 (t = (19,
−279, −21)), square-up converged in 5 (105 → 2.7 mm, z 0.349), 12
planned / 12 rejected, 13 captured. `compute` over all 32: ANDREFF
0.0676, t = (35, −337, −128). So the divergence guard, the retreat and
the bootstrap all did exactly what the plant said. Two things the plant
had NOT shown:

1. **The bootstrap samples poison the compute.** Re-detecting the 32
   archived frames: with the node's file the fixed tag re-projects with
   **11.1 mm rms / 22 max** scatter (09-14: 2.1 / 3.2). Per sample, the
   seven 1.2 m bootstrap frames (tag ~70 px) sit 17–22 mm off each, the
   0.62 m ones 2–9, the 18 sweep views 1–5. The two bootstrap solves
   differed by 200 mm between themselves — fine for AIMING (both
   square-ups then converged), useless as calibration data. Now
   `bootstrap_keep_samples: false` (default): the node drops them from
   the set once the provisional hand-eye is solved (`discard(since)`;
   they stay archived on disk).
2. **The closed-form solve is not what the chain needs.** `calibrate()`
   now REFINES the best OpenCV result by minimising the fixed tag's
   re-projection scatter (position + normal) over the 6 parameters
   (`refine_hand_eye`, scipy least_squares), keeps it only if the
   scatter drops, and reports both (`CalibResult.scatter_*`,
   `summarize`, `result.yaml`). On today's 18 sweep samples: PARK 3.1 /
   6.2 mm → refined **2.3 / 4.8 mm**, normal 0.82°; over all 32: 11.1 →
   5.3. Synthetic A/B (`check_handeye_refine.py`, 14): the refinement
   never raises the fit scatter, lowers the located-tag bias and
   per-view rms on HELD-OUT views (4.9 → 3.1 / 7.5 → 6.3 mm at 0.7° /
   3 mm noise) and costs ≤ 1 mm / 0.1° of hand-eye truth — the trade
   the locator wants; and a 2026-09-18-shaped mixed set is no better
   than the sweep views alone, which is the case for dropping them.

**File in use: the sweep-only refined solve, written offline** from the
run's 18 sweep samples — t = (37, −340, −153) mm, rpy (0.46, −0.47,
−179.3)°: the camera is 0.37 m from the flange and ~180° spun from the
09-14 mount (which is why that file diverged). Jackknife sd 3.5 / 3.6 /
1.7 mm on t, 0.5–1.2 mm on the located tag. The node's 14:49 file is
`T_hc2ee_2026-09-18_node_all32_andreff.npz` (25 mm / 1.4° away), the
09-14 one `T_hc2ee_2026-09-14_old_mount.npz`. ⚠️ **Absolute check not
closed:** tag 0 through the new file lands (−389.5, 992.4, −580.2) mm,
base aligned on 102 to 2.3 mm, vs map.yaml's (−400, 1010, −571.5):
**10 / −18 / −9 mm**, where 09-14 agreed to 1 / 0 / 5. Stable to ~1 mm
under the jackknife, so systematic — fewer views than 09-14 (18 vs 23,
only two tilt-22 views, a longer lever) or the base/map side. Next on
the robot: one more sweep from ~0.6 m (aims from the new file now, no
bootstrap expected), Compute over both sessions' sweep samples, re-check
that number; restart the calibration nodes (they cache the npz).
`check_handeye_sweep.py` 71 → 74, `check_repose_from_corners` 10.

### 2026-09-18 — Hand camera remounted: the sweep's square-up diverged on the old hand-eye; now it retreats and bootstraps its own aiming estimate

User: "현재 handcam 위치 바꿔서 핸드아이 캘리브레이션 다시할려고하는대
초기 이동으로 tag 위치인식하지 못함". The 14:23 sweep
(`log/ros/…/handeye_calib-3.log`): align 1 xy 289 mm / tilt 2.98° at
z 1.20 m → one 10 cm step → align 2 xy **373.5** mm / tilt **7.73°** →
second step → `tag lost during square-up (iteration 2)`, 0 captured.
The step made the error LARGER: the 2026-09-14 `T_hc2ee.npz` describes
the mount the camera was on THEN, and `compute_target_ee_pose` applies
the camera-frame correction through it, so a moved camera turns the
correction the wrong way — the same signature as 2026-09-02's
180°-spun May file (66→131 mm, 5→8.5°). The samples themselves never
depended on that file (it only AIMS), but the square-up did, so a
remount left the auto sweep unusable and the operator with the manual
jog-and-capture procedure.

Two changes in `handeye_sweep.py`, config `handeye_calib.yaml auto:`:

- **Divergence guard in `_square_up`.** The mm-equivalent error
  (`xy_mm + 10·tilt_deg`) after every step is compared with the BEST so
  far: growth by `align_diverge_ratio` 1.25 AND more than
  `align_diverge_min_growth_mm` 30 (so noise near convergence cannot
  trip it) is a divergence; a tag lost right after a step is one; and
  running out of iterations with less than `align_stall_min_improvement`
  25 % of the initial error removed is one too — the plant showed a
  90°-spun hand-eye at 1.2 m moves the camera SIDEWAYS to the error, so
  the number never changes and the old loop handed a wrong hand-eye to
  the planner after 6 quiet iterations. All three retreat to the pose
  where the best error was measured (the tag was in view there) and
  raise `AimDiverged`. Today's numbers trip the first rule after ONE
  step (319 → 451); the 10 cm / 4° "interim file" case of the existing
  check does not trip any.
- **Bootstrap (`bootstrap: auto`, also `always` / `never`).** On
  divergence — or when no file loads — the runner makes its own aiming
  estimate at that pose: captures the current view and six ±10°
  rotations about the FLANGE axes (`BOOTSTRAP_AXES`; pure flange
  rotations, so they need no hand-eye to plan and move the camera only
  lever × angle), returns to the start, and `solve(since)` — the node's
  `calibrate()` over just those samples, `bootstrap_min_samples` 5 —
  gives a provisional `T_hc2ee`. Aim-grade is all it needs: the
  square-up re-measures every step, the views re-detect, and the plan's
  clearance gets `bootstrap_clearance_extra_m` 50 mm on top. The seven
  samples stay in the set (real samples of the new mount). Precondition
  `bootstrap_min_depth_m` 0.5: the camera at least that far from the tag
  so a 10° flange rotation (the tool tip moves a few cm) has room —
  both real sweeps started at 1.2 m. `never` fails naming the manual
  procedure; a second divergence with the bootstrap estimate does too.
  Progress phases `diverged` / `bootstrap`; `SweepResult.aim_source`,
  `bootstrapped`, `n_bootstrap`, `diverged`; the start event carries
  `aim_source`; both UI fronts render them.

Verified offline: `check_handeye_sweep.py` 36 → **71** — the fake solve
is the real `cv2.calibrateHandEye` over the fake's (T_ab2ee, T_cam2tag)
pairs with 0.7° / 3 mm per-frame noise: against a quarter-turned +
15 cm-moved camera the file diverges (or stalls at 1.2 m), the arm is
back at the start with the tag in view, `never` names the procedure and
captures nothing; `auto` captures 7/7 bootstrap views at both 0.55 and
1.2 m starts, the provisional hand-eye is within **40 mm / 3.7°** of the
truth (worst of 6 seeds; the file was 155 mm / 90°), the square-up then
converges in 3–6 iterations and every seed ends with 21–23 samples, no
capture off-tag, tool ≥ 189 mm above the plate, arm back at the start;
`always` and no-file bootstrap; a 0.30 m start is refused before any
bootstrap move. A scan of mount changes: spins ≥ 90°, tilts ≥ 45° are
all caught; a 45° spin and pure translations of 0.3–0.5 m still converge
and are left alone. `check_task_list_ui.py` 85 → 86, `check_web_ui.py`
106 → 109. Not run on the robot: restart the calibration launch
(`use_handeye_calib:=true`), drive hand_cam over cross tag 0 from
~0.6–1.2 m up, Auto-sample, watch for `sweep: align 2: … WORSE …
retreating`, then `bootstrap: provisional T_hc2ee from 7 samples`, then
the normal `N views planned`; `Compute & save` afterwards. Rename the
09-14 file `T_hc2ee_2026-09-14_old_mount.npz` when the new one is in.

### 2026-09-15 — Web Task combo: the dropdown showed ONE task; now a real dropdown that always lists them all

User (pinyin): "下拉框中只有一个" — the Task tab's task dropdown offered a
single entry although /task_list carries nine. Cause is the widget, not the
data: the web UI used a plain `<input list=datalist>`, and a browser's
native datalist only suggests options whose value contains the input's
CURRENT text as a substring. The field is pre-filled with the first task's
full name on load (and holds a full name after any pick), and no other task
name contains that whole string — so opening it showed exactly one
self-match and the other eight silently vanished. (The Qt window's
editable `QComboBox` lists everything regardless of the edit text, so it
never had this; the web port inherited the browser's filtering.)

Replaced with our own dropdown (`renderTaskDropdown` /
`openTaskDropdown` in `web/app.js`, `.combo` / `.combo-list` in
`style.css`, `#task-combo` / `#task-dropdown` in `index.html`; the
datalist is gone): a click or focus opens it UN-filtered — every task,
always, each with its one-line summary underneath — and only typing
afterwards narrows it by substring; a pick sets the field, closes it and
updates the detail view; an outside click or Escape closes it; a
`/task_list` republish while it is open re-renders it with the current
filter instead of resetting under the cursor. The item handler is on
`mousedown`, not `click`, so the pick lands before the input's blur /
the outside-click close could drop it on the same gesture. Free typing of
a name the list does not show still works (same field, same `input`
handler).

Verified: `check_web_ui_browser.py` 76 → **86** — a real headless Chrome
reproduces the failure state (field already holding a full task name)
and asserts the opened dropdown lists all 4 fixture tasks including one
whose name shares nothing with the field text; typing narrows to the one
substring match; a pick sets the field, closes the list and updates the
detail; an outside mousedown closes it. Two harness lessons on the way,
recorded in the test's comments: headless Chrome does not move DOM focus
on a programmatic `.focus()` (no tab activation), so the test dispatches
the click the handler also listens for; and `.click()` synthesises only a
`click`, never the `mousedown` a real click starts with, so the
outside-close test dispatches `mousedown` explicitly. Then **live against
the running stack**: the field pre-filled with
`scan_joint_errorX_p000mm_standoff_010mm_height_652mm`, a click on it
listed all **9** real tasks. Web-only change; `check_web_ui.py` 106 and
the Qt `check_task_list_ui.py` 85 unchanged. Browser tabs pick the new
`app.js` up on reload (served `no-store`); no node restart needed.

### 2026-09-15 — Task tab shows what a task actually IS; Ra map CSVs moved from results/ to log/

Two small user requests about the web UI (pinyin): what does a listed scan
task option actually do — the names are long, cryptic RRT-set strings
(`scan_joint_errorX_p000mm_standoff_010mm_height_652mm`) and the old detail
line squeezed mode/tags/points/lift/files into one dense `·`-joined row —
and don't write each scan's result CSV under `results/`, put it under
`log/`.

**Task detail is now field-by-field, not one line.** New
`taskDetailHtml()` (web, `robot_ui/web/app.js`) /
`MainWindow.task_detail_html()` (Qt, mirrored so both fronts show the same
thing — same source data, `/task_list`, just rendered by each toolkit):
a plain-language mode line (pose / joint / move-only / system, spelled out
rather than the raw enum), then a small table — tags (with "drives to each
stop, in order"), points (scan vs. traverse, traverse marked "driven
through, not scanned"), lift, source file(s), paired file (labelled *why*
it's paired — IK seed for pose mode, world xyz for joint mode), IK-seeded /
world-xyz point counts when present, an explicit subset note for
`groups_filter`, and — only for joint-mode tasks — the reach/collision
warning from CLAUDE.md's RRT-dialect section, now always visible next to
the task rather than only in a log line at registration. `taskSummary()` /
`task_summary()` (the compact one-liner) survive unchanged as the
`<option>` tooltip on the datalist / combo, which can't render multi-line
HTML. Qt's `QLabel` needed `setTextFormat(Qt.RichText)` set explicitly —
relying on its HTML auto-detection would have missed any task name that
doesn't start with a tag-like token, i.e. almost every real name.

**`paths.RA_MAP_DIR` moved: `<ws>/results/ra_maps` → `<ws>/log/apriltag_nav/
ra_maps`.** Same file (`<task>_ra_map_<ts>.csv`), same 13-column format,
same writer (`task_executor` → `arm_node`'s `ScanResultWriter`), still
versioned — just filed next to the other per-run RECORDS (`nav_log`,
`path_tag_locator`) instead of sitting in `results/` beside the large,
unversioned `scan_images` bulk. `tools/ra_map_plotter.py` takes the csv
path as an argument, so it needed no change. The three CSVs already
tracked under `results/ra_maps/` were `git mv`'d, not copied, so their
history follows them. Updated in the same pass: `paths.py` (the one
definition), `task_manager.py`'s docstring, `mobile_manipulator.launch`'s
comment, `results/README.md`, `.gitignore`'s comment (the pattern itself
needed no change — nothing under `log/` was excluding `*.csv`), and this
file's *Where run output lives* table + the Korean-guide backlog row.
Left alone, per the standing rule that Work Log entries are historical
narrative: the 2026-09-14 entries that named `results/ra_maps` as where
that day's CSVs landed — they were true when written.

Verified offline: `tools/check_web_ui.py` 106 → **still 106** (task-detail
rendering isn't covered by the fake-bridge suite, since it never emits a
`/task_list` payload with `scan_mode`/`kind` set — task-detail is instead
exercised through `check_web_ui_browser.py`, which now asserts, against a
real headless Chrome, the plain-language mode line for pose / joint /
system tasks, the tags/points/lift/paired-file table rows (traverse points
called out as "not scanned", the paired file naming *why* it's paired),
the joint-only reach/collision warning present on a joint task and absent
on a pose/system one, and that a task name or file name containing `<`/`&`
comes through HTML-escaped rather than as live markup: 65 → **76**.
`check_task_list_ui.py` (Qt) got the same coverage against the same
`PAYLOAD` fixture, plus `lbl_task_detail.textFormat() == Qt.RichText`
(explicit, not relying on QLabel's HTML auto-detection — which a name not
starting with a tag-like token, i.e. almost every real one, would miss):
71 → **85**. `paths.py`'s `RA_MAP_DIR` change needs no test beyond the
existing suites resolving through it — confirmed by re-running
`check_task_discovery.py` (47/48, unchanged from before this change: the
one failure is the pre-existing "assumes three file pairs" mismatch noted
in the 2026-09-14 entry, not a directory-path assertion). Not run against
the live stack; `task_executor` restart required before the next TASK run
writes to the new location — its own `RA_MAP_DIR` import is process-start-
time, so an
already-running node keeps writing to the old path until restarted.

### 2026-09-15 — robot_ui on the web: every feature of the PyQt window as a page any LAN computer can open

User (pinyin): "robot ui 重构, 保留所有功能, 把 UI 显示在 web 上, 使局域网内
其他人可以连接". The site LAN has no internet and the robot PC has no
Node, so the constraint was: nothing new to install anywhere. Tornado is
already on the PC (rosbridge_server depends on it) and the page is plain
HTML/JS/CSS served from `robot_ui/web/` — no CDN, no build step.

**Shape.** Three layers, and the split is the point:
- `RosBridge` lost its Qt: it was a `QObject` with `pyqtSignal`s, i.e. the
  one ROS boundary of the package could only feed a Qt event loop. Its
  outputs are now `robot_ui.signals.Signal` (connect/emit, synchronous on
  rospy's thread; `SIGNALS` / `STATE_SIGNALS` name them, the cache is keyed
  by name, `cached_states()` is the web replay). `MainWindow` marshals
  every handler through ONE `pyqtSignal(object, object)` — the same
  mechanism `append_log` already used — so the desktop window is
  unchanged in behaviour (`check_task_list_ui.py` 71/71 still).
- `web_ui.UiController` is `MainWindow` minus the widgets: capture → save →
  Ra with the preview PAUSED (not released) for the shot, the 5 Hz
  lamp-off preview loop that skips a tick while any call is in flight,
  jog / MOVE with blank axes taken from the live pose / standoff, task +
  lift, manual base moves with the in-flight flags, the map-calibration
  session with the plate↔ref pairing and the yaw-sweep warning, hand-eye,
  plugins (hot-reload, run ON THE ROBOT PC whoever pressed RUN), STOP ALL,
  ROI in image pixels persisted to `roi_config.json`, a 500-line log ring.
  It talks to a `sink` (state / event / ui-patch / log / frame) and holds
  the SHARED ui dict every browser renders — so two tabs on two PCs show
  one robot and either may act (last press wins, documented).
- `web_server` (tornado, one port): static page, `/ws` JSON protocol
  (hello = replay of cached states + ui + log history; state / event / ui
  / log broadcasts; `call` → `api_<name>` on a pool → `reply` to the
  caller only), `/api/health`. Frames: the encoder thread JPEGs the NEWEST
  frame per camera once (downscaled to `stream_max_width` 1400, capped at
  `stream_fps` 10, nothing encoded with no client), fanned out with ONE
  outstanding frame per camera per client — the browser acks after
  decode, so a slow Wi-Fi link drops frames instead of lagging. The
  full-res Basler frame stays server-side for the ROI crop, the PNG and
  inference.

**Verified.** `tools/check_web_ui.py` **106** (controller + real tornado
server over a WebSocket client: replay, broadcast to two clients, reply
routing, unknown/private methods refused, packet format, the ack holding
the second frame and releasing frame #3 not #2, late joiner gets the last
frame) and `tools/check_web_ui_browser.py` **65** — a REAL headless
Chrome driven over DevTools against the same server: chips from live
states, every tab's buttons producing the bridge calls, a pushed frame
drawn on the canvas and acknowledged, ROI drag landing on the server in
image pixels, thumbnail click / double-click maximise, STOP ALL, zero JS
exceptions; screenshot in `src/robot_ui/tools/web_ui_screenshot.png`.
Then **the real node against the running stack, view-only** (robot idle
on dock 500, camera closed): all ten states rendered, the nine real tasks
listed with their detail lines, calibration nodes ONLINE / hand-eye
OFFLINE, front_cam overlay on tag 500 + side/hand cams at 8 fps in the
browser, ~50 % of one core with one client (the three 30 Hz cv_bridge
conversions the Qt window also paid, plus JPEG), clean SIGINT exit with
the port released. Not yet used by an operator from another PC; the
Phoenix bridge needs a TCP 8080 forward before the Windows PC can reach
it. `catkin_make` clean (`web/` and the new script are installed).

### 2026-09-15 — Pose calibration on the 90 mm tags 149/150; tags are 1 mm plates (tz = height_m + tag_thickness, cross tags z +0.001)

User: the 60 mm pair 15/16 is gone, four 90 mm tags 147–150 are on hand;
and every tag is a 1 mm thick plate, "로봇 베이스로부터 태그는 z축으로 1mm
위에 있다고 보면 돼".

**Thickness.** The tag-pair fit measures the lens above the plane the
corners lie on — the tag TOP — so `height_m` (0.302) is not the lens
height above the floor. New `robot.tag_thickness: 0.001`;
`make_front_cam_extrinsics.py` writes tz = height_m + thickness (0.303),
`load_extrinsics_full` checks that sum (the old tz == height_m is now
refused, pinned by `check_front_cam_extrinsics.py` 22 → **23**, which
also renders the floor tag at z = +0.001 and asserts the level chain
returns exactly that), `robot_sim` lays floor tags at floor + thickness.
Nothing in navigation reads the sum. **The cross tags too (user, same
message): they sit in machined slots but their top face is 1 mm above
the plate top, so `reference_tags.yaml` / `_plate2.yaml` z went 0 →
+0.001** (the chain measures that face; the expected path-tag z is now
−0.079, the floor tags' top face). The calibration plans were NOT
regenerated — the generator would reset plate 1's session-measured seeds
for a 1 mm change the align loop absorbs anyway.

**The tool** (`calib_front_cam_pose.py`, doc §1–§4 rewritten): defaults
149 → 150 (the user's pick), 0.090; `--spacing` has NO default (it is the scale ruler;
measure (outer extent + inner gap) / 2 so the print size drops out).
Three changes the bigger tags forced or allowed, each measured on the
synthetic plant (`check_front_cam_pose_calib.py` 16 → **27**):
- **Frame room.** A 90 mm tag is 271 px at 0.30 m; a 0.12 m pair spans
  half the view and a 0.12 m drive would push a tag out. `check` prints
  the room (fwd / rev / left / right, from the corners), `collect` caps
  every scan move and drive to the room of the moment (`frame_room_m` /
  `cap_distance`); the plant confirms zero lost frames with the cap and
  733 without.
- **Laying angle is fitted.** `fit_ground` gained one in-plane rotation
  per tag, so the tags no longer need parallel edges and a tag laid a
  quarter turn round fits with a 90° angle instead of scrambling the
  corner order (91.5° / −2.0° recovered to 0.005°; roll / pitch / tx /
  yaw unchanged). On the 2026-09-08 record this absorbs tag 15's known
  0.4° skew: rms 0.33 → 0.22 px, roll +1.206 / pitch −0.495 / 302.1 mm
  (0.02° from the values in robot.yaml — inside the stated precision,
  robot.yaml left as is since the user is re-calibrating anyway).
- **Rotation centre as one linear least squares** over all pivot
  snapshots (`fit_rotation_centre`: l_i + R(−φ_i)·centre_C = c_T) with
  the pivot pattern +3 +3 −3 −3 −3 −3 +3 +3 (nine snapshots, ~9° of
  spread, lens ≤ 5 cm off its line — the ±4°×3 pattern gave 3.5°), and
  30-frame snapshots instead of 15. Over eight noise seeds: tx rms 0.3 /
  max 0.6 mm (15 frames: 0.85 / 1.5), ty 0.5 / 1 mm, yaw 0.04 / 0.1°;
  a single 0.09 m track is ±0.5° — the mean is the number, and the doc
  now says so.
**Same afternoon, at the robot: the bumper hides the left third of
front_cam's image** (user; the first `check` showed the pair 7 cm ahead
of the nadir for exactly that reason). `--left-edge-px` (430) bounds the
usable image; `check` prints the room with BOTH tags (snapshots) and with
ONE tag (drive tracks) and the pair's offset from the usable middle;
`collect` recentres the pair there before the pivots and before the
drives, caps scan moves to the both-tag room and drives to the one-tag
room, and the yaw fit places a frame from a single tag (`square_angle` of
its four edges + the fitted laying angle, `pair_pose_any`), so a track
runs ~0.15 m across the usable width instead of the ~0.04 m both tags
allow. Pivots are ±2° (cumulative ±4): with the pair 7 cm ahead the far
tag sits 0.68 m from the rotation centre and ±6° put it past the 0.07 m
of lateral room in the plant. Eight seeds on the occluded plant: tx rms
0.7 / max 1.3 mm, ty 0.5 / 0.9 mm, yaw 0.05 / 0.08°. One sign bug caught
by the plant on the way: the square's edges turn −90° per corner, so
edge k is brought back by +k·90°, not −.
**First real session (13:56–13:58, `calib_pair_20260915_a`): the fit
is excellent (17 snapshots, rms 0.133 px, roll +1.334 / pitch −0.277 /
302.0 mm, tag size 89.71 mm, laying angles 0.10 / 0.18°) and the eight
drive tracks ran 145 mm each with single-tag frames doing real work
(105 of 168 frames) — but the commanded ±2° pivots executed 0.3–0.8°
each, 2.1° of spread in all, giving a centre of (−548.8, +3.2) mm with a
jackknife sd of (2.2, 9.0) mm: unusable.** The pivots are therefore a
CLOSED LOOP now: cumulative executed targets +3 +6 +3 0 −3 −6 −3 0°
measured on the pair angle at rest, gain (commanded / executed) learned
per attempt, sign included, each command capped by the lateral room
(`max_pivot_deg`), and `collect --only piv|scan|drive` redoes one phase
into the existing directory (old files renamed `old_*`). Fake-base test
(stiction under 1°, 25–75 % execution): 9 snapshots at 0 / −3.4 / −5.7 /
−3.0 / −0.3 / +3.3 / +6.1 / +2.7 / −0.3°, 21 commands. **User then asked
for bigger pivots: the sweep is physically bounded by the 0.24 m tall
view — a tag 0.6 m from the rotation centre swings 10 mm per degree and
has ±75 mm of room, so ~±6–7° even keeping only ONE tag in view.** The
targets are now ±F/2, ±F with F = `max_pivot_from_corners(keep='one')`
read off the frame (≤ `--pivot-max` 12), the feedback is per-tag
(`square_angle` of whichever tags are visible, referenced per tag), and
pivot snapshots may show one tag (`pair_pose_any`). Plant: F 5.4°,
spread 10.8°, eight seeds tx rms 0.36 / max 0.62 mm, ty 0.5 / 1.2, yaw
0.05 / 0.11°. Going beyond needs a different layout (tags offset
laterally), not a bigger command. **Second closed-loop run (14:22):**
reached +8.7° (7.5° commanded → 4.3° executed, twice) and then stuck —
the lateral cap took the smaller of the top/bottom rooms, so with the
tags at the bottom edge it forbade the way BACK too, and the sweep ran
at mobile_node's default 0.2 rad/s ("too fast"). Fixed: the cap is
directional (image angle + ⇒ tags move DOWN the image, verified on the
run's snapshots: +8.7° moved tag 149 from row 356 to 576), margin 12 mm
(at 8.7° a tag 6 mm from the edge was still detected), the sweep keeps
BOTH tags by default (F ≈ 6.4° here; `--pivot-one-tag` for ±8 with
single-tag extremes), `--pivot-speed` 0.08 rad/s, a levelling pivot
first (the tags back onto the principal row, first command capped to 3°
until the sign is learned), and a pivot mobile_node reports short of its
ODOM target (its `shortfall_frac` check, the normal case for this base)
is progress rather than an abort. **Third run (14:29) completed: 9 pivot
snapshots over 12.5° of executed spread, all two-tag.** Solve
(`--spacing 0.120`, 17 snapshots rms 0.237 px): roll +1.406 / pitch
−0.323°, lens 301.8 mm, tag size 89.82; rotation centre (−550.3, +4.6)
mm from the nadir (jackknife sd 1.3 / 3.8 mm, rms 0.32 mm) ⇒ **tx 0.5504,
ty +1.7 mm**; yaw −0.305° (fwd tracks −0.03, rev −0.59). Analysis before
trusting it: (1) the tilt subsets agree in roll (1.33–1.41) less in
pitch (−0.26 scans / −0.42 pivots); vs 09-08 (+1.228 / −0.504) both
components moved ~0.18° and the optical axis by 0.25° — floor-flatness /
chassis-loading territory (two different floor spots, 3 mm/m), not
evidence of the camera moving; the practical difference in the current
pipeline is 0.09° of edge angle at the aim position and 1.4 mm of
position. (2) The lever depends on which tilt is used (0.5504 with
today's, 0.5465 with 09-08's on the same pivots), so its honest
uncertainty is ~3 mm; 0.550 / 0.552 / 0.5504 all say `camera_offset`
0.55 stands; ty +1.7 ± 3.8 vs 09-08 −4.5 ± 4.9 ⇒ lens on the centreline
within the noise, `camera_lateral` stays 0. (3) The fwd/rev yaw split
is NOT the single-tag reconstruction (bias ±0.03° / ±0.4 mm, random,
checked on 29 at-rest snapshots across x 586–1086): the rotation
centre's body-frame lateral position along every track is flat with a
+1 mm step (to the left) at the same floor position in both directions
— a floor feature — and 1 mm over 0.145 m is 0.4°, so a straight-drive
yaw on this floor is ±0.3° at best; the both-tag middle segments alone
give −0.71 (fwd −0.80 / rev −0.60, consistent), the whole tracks −0.31.
Verdict: yaw −0.3…−0.7, the configured −0.38 is inside it. **Applied
(user's call): roll +1.406 / pitch −0.323 / height_m 0.302 /
camera_offset 0.550, camera_lateral kept 0 (ty is noise), yaw_deg kept
at the driving-verified −0.38; extrinsics.yaml regenerated (tz 0.303),
`ground_plane.front_cam.enabled` back to true.** Effect on driving: no
code changed and `camera_offset` is unchanged, only the corrected
detections move — a square-laid tag reads 0.03° / 1 mm differently on
the crosshair (stop align) and 0.09° / 1.4 mm at the aim position
(0.2 m ahead), all under the 0.2° align band. `robot_camera_node` and
`mobile_node` restart required; the calibration nodes re-read
extrinsics at launch.

**Same afternoon, user question: is the camera-latency compensation
applied?** Yes (`camera_latency_compensation: true`, every record carries
`tag_age_s` / `latency_comp_px`) — but its frame-age cap was a hard-coded
0.3 s and it was BINDING: 170 of the 173 records of 09-14/15 read exactly
0.3. Probed live: the image arrives 0.09 s after its stamp, the detection
0.33 s (0.29–0.36) — ~0.24 s inside `robot_camera_node`, which was at
350 % CPU with two `tag_overlay` subscribers (robot_ui and the new
robot_ui_web) and 21.5 Hz of detections vs 30 Hz of frames; the 2026-08
figure was 109 ms end to end. Cost of the cap: 0.03 s × 0.01 m/s = 0.3 mm
at the stop, ~1 mm while steering at 0.033 m/s. Now
`camera_latency_max_s` (0.5) in robot.yaml and `tag_age_raw_s` in the
records, so the next records show the true age. Not fixed: the node's
latency itself — check it with one overlay subscriber, and whether the
rectified overlay (`GroundPlane.rectify`, cv2.remap per frame per
subscriber) is what costs the 0.24 s. **A/B driven the same hour (user):
with the compensation OFF the forward stops rested +1.3…+3.3 mm PAST the
tag (mean +2.3; reverse 3.5–4.6 mm further than the trigger) against
−0.2…−2.6 mm (mean −1.0) with it ON on the same lane an hour earlier —
a 3.3 mm difference = the 0.33 s frame age × the 0.01 m/s stop speed,
exactly. The raw ages now recorded: 0.30–0.36 s. Kept ON.** Then
`stop_latency_linear_s` 0.22 → 0.15 → **0.08** (the post-trigger roll at
0.010 m/s is ~0.7 mm / 1.3 px now, not the 2.1 mm fitted on 09-09 under
the capped age) and `center_x_stop_tolerance` 4 → **2 px**: with a 2 px
lead the 4 px floor was the firing line, and the 112→109→112 check run
(4 fwd, 3 rev) rested 1.5–4.4 px short of its column both ways, lateral
+0.5 ± 1.4 / +0.2 ± 0.5 mm, yaw within ±0.2°, 1–3 align passes. **Driven
(15:26–15:28, 112→109→112, after the tolerance change and the camera-node
restart): at rest forward −2.6 / −1.2 / −0.3 / +1.9 px from the column
(mean −0.6 px = −0.2 mm), reverse +0.5 / +1.0 / +0.6 px; lateral
−1.2…+2.5 mm; yaw ±0.16°; align 1–2 passes; frame ages 0.11–0.22 s (one
0.41 s frame, compensated to +1.9 px). The stop is on the column within
1 mm both ways.**
Also noted in the doc: 147–150 are zone-E map ids, harmless for the tool
(manual moves only) but `/robot_pose` and `last_known_tag` will point at
zone E until `mobile_node` is restarted, which the procedure does anyway;
147/148 serve the §6 mechanical yaw cross-check (one frame cannot hold
two tags 0.5 m apart, so two snapshots). Verified offline only; the
suites that read the loader still pass (ground_plane 17, repose 10,
error_budget, yaw-sweep 4/4). `catkin_make` not needed. Not driven.

### 2026-09-15 — front_cam ↔ hand_cam chain calibration from two floor tags (`calib_fc_hc_chain.py`)

User: the matrices between front_cam and hand_cam are a black box — no
way to tell any of them from its ideal value — so measure the whole
T_fc2hc against two 90 mm tags (149 / 150) laid at a known spacing, one
under each camera, and use the ideal-vs-measured difference as a
correction. Built as its own package **`src/chain_calib`** (2026-09-15 evening:
`solver.py` pure numpy, `session.py` persistence + coverage advice,
`scripts/chain_calib.py` check / capture / status / drop / solve,
`README.md` = the operator guide, sessions under `log/chain_calib/`).
**Operator-jogged only**: the automatic sweep collided the arm (below)
and was removed; `capture` saves one sample at the current pose and
prints the view (tilt, direction, spin), that view's raw chain error
and a coverage line saying which tilt directions / spins are still
missing for a determined hand/base split. `catkin_make` once for
`rosrun chain_calib`.

The design point: with the base still, tag B under front_cam is one
constant observation and the arm poses are the only excitation, which
makes this the AX = YB (robot-world / hand-eye) problem — `inv(H)·S_i =
D·(A_i·B)·F` with D a hand-side (hand-eye) correction and F a base-side
one (arm mount T_ab2mb, T_mb2fc and the front-tag observation, which one
base pose cannot separate). One pose cannot tell D from F; views with
rotation DIVERSITY (tilts about two axes + spins — the hand-eye sweep's
own planner and safety rules, tag plane = the floor) can, which is
precisely what the 09-11 sessions lacked. `solve` fits raw / hand-only /
base-only / joint and reads the attribution off the residuals; the laid
truth is snapped to the quarter-turn ambiguities of two collinear-edge
tags, so the print orientation need not be known. Nothing is applied
automatically: `--write-hand-eye` writes a dated npz for locator.yaml, a
base-side F is printed folded into T_ab2mb (also robot.yaml
arm_calibration's matrix — pose-mode IK — so a separate decision).

Verified offline: `scripts/check_chain_calib.py` (23 — synthetic chain
through the real `plan_sweep` with the real hand-eye / extrinsics, tag
corners rendered and re-solved: a planted 2° / 15 mm hand-eye error is
recovered to 0.8 mm / 0.06° with the base fit 10× worse, a planted 1° /
8 mm mount error to 0.3 mm / 0.04° the other way round, both at once by
the joint fit; the per-sample noise floor is 2–3 mm / 0.3° because a
120 px tag's out-of-plane tilt is only 0.7° per FRAME and rides the
0.7 m A→B lever — hence 20-frame means per camera per view, without
which D is only good to ~4 mm). Live `check` ran against the master
(arm state, extrinsics, both K, hand_cam topic) and failed correctly on
the unlaid tag.

**Run on the robot the same evening (three layouts).** Two lessons and
one result. (1) The user's E / g tape readings came out 100 mm over the
chain twice (left: chain 790.7 vs 889.5; right: 1005 vs 1100) — the
SAME sign on both sides, which an arm-mount offset cannot produce
(it would flip) — and a direct centre-to-centre read agreed with the
chain; the layout is now read as outer-edge-to-outer-edge minus the
fitted 89.75 mm tag width (spacing 1.01025 m, tag 150 under hand_cam,
149 under front_cam, 150 to the robot's RIGHT). `check` at three arm
poses gave the chain 10.9–13.2 mm / 1.0° off, almost all of it +8…+14 mm
in HEIGHT and constant under a 90° wrist spin — so not a hand-eye
lateral error. (2) **The sweep collided the arm** (`20260915_c`: 12
planned, 6 rejected, 2 move failures, 11 captured of which v09/v10 are
the collision, 414 mm residual): the planner's safety rules model only
the flange and vision tip against the TAG PLANE — the mobile base body,
lift and the arm's own elbow are not modelled at all — and with the tag
1 m beside the base near the reach limit the tilted views swung the
elbow into the robot. ⚠️ Do not run `collect` again in that geometry
without a base-body / link clearance model, or take the tilted views by
hand (jog + capture). `solve` now drops outliers (> 4× the median raw
residual, `--exclude`). Nine clean samples: raw 11.0 mm / 0.94° rms,
hand-only 7.4, base-only 6.3, joint 4.3 (jackknife 7–16 mm) — **verdict
UNDETERMINED**: the nine views were 8 tilted (11–23°) but ALL toward
the tag's −x / −y sides (the +x / +y views were the rejected / failed
ones), and without the opposite tilts a hand-eye offset along the
optical axis and an arm-base height offset are the same thing. What stands: no 100 mm-class
error anywhere (mount −100, camera_offset 0.55, hand-eye all mm-true);
the chain is ~5 mm in the floor plane and ~12 mm in height, the latter
consistent with the 09-14 hand-eye absolute check (z 5 mm). Nothing
applied. The arm controller refused connections from 18:06 (fault after
the collision) — recover on the pendant first.

### 2026-09-15 — Collect tab VISION lamp switch; black frames in bursts explained and fixed

User: "UI 收集数据上添加 vision 的照明开关，还有在连续拍照时会发生一部分完全黑
的图像是怎么回事". Two things, the second more important than it looked.

**The black frames.** Read from the capture path, then checked against
data: `camera_interface` runs the Basler free-running (Continuous, 5 fps,
4 ms exposure, gain 0) with `GrabStrategy_LatestImageOnly`; the node turns
the lamp on, sleeps `warmup_s` 150 ms and calls `RetrieveResult`, which
hands back the newest COMPLETED frame — and on this 20 MB GigE part a frame
completes ~200 ms after it was exposed, so that frame was exposed before
the lamp: black. In a burst only the first frame is affected; over many
single-frame captures it is a fraction of them. The survey of yesterday's
scan frames (`results/scan_images/20260914_flat_three_runs`, 240 sampled)
puts it at **7.1 % black**, and every black frame carries **Ra ≈ 0.082**
(0.0802–0.0897): the ONNX model's constant answer to a black input. So
the same defect has been in every TASK scan, silently — a Ra map row at
~0.082 with `success` is the signature. Fix: flush `lamp_flush_frames`
after lamp-on plus a mean-intensity dark check with bounded re-grabs;
promoted to the camera-lifecycle section. Not changed: the exposure — the
frames are dim overall (median mean 18/255) but that is what the model was
trained on, and the user did not ask.

**Follow-up, same day: "相机在 live 下无法 capture，快速点击 capture 时还是
有部分图像中有黑色部分".** Two more defects, both fixed and promoted into
the camera-lifecycle section: (1) the CAPTURE button was disabled by the
preview's own grabs (`_update_busy` counted every pooled call) — preview
calls are counted separately and a capture pauses the preview instead of
releasing the device; a worker-GC bug that could leak the same counter
for a whole session was found by the offscreen check and fixed with
strong references (`_live_workers`). (2) PART of a frame black = a
rolling-shutter frame that straddled the lamp switch; the dark check is
now per band (8 rows bands) and the offline camera model is a rolling
shutter — with flush alone the second frame comes out half lit (mean
~90) and the band check re-grabs it. `check_basler_lamp.py` 18 → **20**,
`check_task_list_ui.py` 61 → **71**.

**The lamp switch.** `basler_camera_node` stays the sole owner:
`/camera/set_lamp` (Bool) holds the lamp on, `/camera/lamp_state` (latched
Bool) reports it, the hold opens the device and ends whenever the device
closes. robot_ui: "VISION lamp" checkbox next to Live preview (follows
`lamp_state`; STOP ALL releases it), bridge `set_vision_lamp` /
`lamp_state`. Under a hold a capture uses the lamp as it is (no toggling,
no flush, message `lamp held`).

Verified offline only: new `tools/check_basler_lamp.py` (18 — the old
behaviour reproduces a black first frame against the delayed-camera
model; flush fixes it; a relay that lights 2 frames late is caught by the
re-grab; retries bounded; lamp-off frames never re-grabbed; hold opens /
lights / publishes, capture under hold neither toggles nor flushes,
release / close / shutdown all end with the lamp off) and
`check_task_list_ui.py` 55 → **61**. Not run on the robot;
`basler_camera_node` and robot_ui restart required. First thing to watch:
`captured 1/1 (flushed 1 pre-lamp frame)` in the node log, and no more
Ra ≈ 0.082 rows in a scan CSV. ⚠️ The three Ra maps of 2026-09-14 in
`results/ra_maps` should be re-checked: rows at Ra ≈ 0.082 are black
frames, not measurements.

### 2026-09-15 — front_cam pose calibration as a procedure: `tools/calib_front_cam_pose.py` (tx, ty, yaw, roll, pitch, height) + the rotation-centre question

User: is there a record of the 2026-09-08 lever measurement, how to know
whether the rotation centre is the chassis centre, and a full T_mb2fc
calibration procedure to run now. The record is
`log/apriltag_nav/calib_pair/` (15 snapshots + `front_cam_fit.npy`);
re-running `fit_front_cam_ground.py` on it reproduces roll +1.228 /
pitch −0.504 / 302.0 mm / lever 0.552 ± 0.001 m from six ±4° pivot pairs.
Solving the arc CENTRE (not just its radius) from the same pivots puts the
rotation centre (−552.2, −4.5) mm from the nadir in the level camera frame,
sd (1.4, 4.9) — the lateral term is inside its noise.

**The tool** (`collect` / `snap` / `record` / `solve [--apply]`, doc
`docs/FRONT_CAM_POSE_CALIBRATION_kr.md`): snapshots are 15-frame corner
means (single frames cost the 09-08 fit ~0.07° of roll / 2 mm of lever in
the plant); tx, ty come from the pivot arc centre; **yaw from straight
drive tracks by a cumulative-lateral least squares** — the rotation
centre `c_i = l_i + R(−φ_i)·centre_C` may only move along the body x axis,
and yaw is the one rotation that makes the accumulated lateral drift
vanish. Two things the plant taught: per-step differencing amplifies
corner noise 20× (drop it), and the pair angle's 0.04° frame noise times
the 0.55 m lever is 0.4 mm of centre position per frame — smooth φ over
~21 frames before placing the centre. With that, 0.3 px noise and six
0.12 m tracks give yaw ±0.05–0.08°; per-track spread is the honest
uncertainty, and **lateral slip of the rotation centre is
indistinguishable from camera yaw** (1 mm per 0.1 m = 0.57°), so several
tracks both ways and a mechanical cross-check (doc §6) are part of the
procedure. `robot.yaml` gained `camera_lateral` (ty; read by the
extrinsics generator only — `mobile_controller` still assumes the lens
on the centreline), `MobileClient.pivot_angle` was added beside
`drive_distance`, and `fit_front_cam_ground.py` exposes
`load_snapshots` / `fit_ground` for reuse (output unchanged).

**Rotation centre vs geometric centre** is a mechanical question (doc
§5): plumb the front/rear bumper centres to the floor, pivot 90°, plumb
again; the perpendicular bisectors meet at the rotation centre, the
bumper midpoint is the geometric centre. The camera's tx is to the
rotation centre; `T_ab2mb` and the cell's stop poses are chassis-centre
figures, so a difference goes 1:1 into the arm's world position.

Verified offline only: `tools/check_front_cam_pose_calib.py` (16 —
synthetic plant with the real D, tilt, yaw −0.4 / +0.7 / 0, lens 4 mm
right / 8 mm left / on-centre, under-executed moves, heading wander,
0.3 px noise: roll/pitch 0.02°, height 0.5 mm, tx 1 mm, ty with the right
sign, yaw within 0.08° and the model exact at zero noise; `--apply`
round-trips a copy of robot.yaml through the generator and the loader).
`check_front_cam_extrinsics.py` 22 still pass with `camera_lateral` in the
generator. `collect --dry-run` against the live master refused correctly
(tags 15/16 not laid, correction on). Not driven.

### 2026-09-15 — robot_ui: distance-sensor assist (live Keyence standoff + "Auto standoff" from the current pose)

User: "在 UI 中添加使用距离传感器进行辅助的功能". The Keyence DL-EN1 was
only ever read inside a TASK scan; an operator jogging the tool over a
surface by hand had no standoff readout and no way to let the sensor
finish the approach. Now, on the Arm tab, group *Distance sensor assist*:

- **Live line** from a new `/arm/standoff_state` (String JSON, one per
  `/keyence/value` message, published by `ArmController.keyence_cb` via
  `standoff_state(raw)`): `standoff: 12.21 mm (target 10: 2.21 mm too
  far) raw −3.00`, `ON TARGET` inside 0.2 mm, or `OUT OF RANGE (too far /
  too close)` from the ±99999 sentinel's sign, with an age-out after
  1.5 s. Computed in the controller, not the UI, so the beam projection
  (cos 42.6°), the sensor zero and the live target stay in ONE place:
  `perp = raw·cos(beam)`, `standoff = zero − perp`, `err = target −
  standoff` (approach-positive, the loop's own convention).
- **Auto standoff** → `/arm/standoff` `{"target_mm": t}` →
  `arm_node._run_standoff` → `ArmController.adjust_standoff(t)`: the
  scan's own `StandoffController` run from wherever the arm is (fresh
  median-of-5 readings after every move, approach capped at half the
  measured gap / 1 mm fine, 25 mm travel budget, response check), result
  through `motion_seq` like move_cart / jog, cancel through `/arm/cancel`.
  The target typed in the UI (1–30 mm, default 10) is scoped to that one
  call — `keyence_setpoint_mm` / `target_distance_mm` are restored
  afterwards, so an experiment cannot change what the next TASK scans
  at. Refused while a scan holds the worker, like jog.
- Bridge: `standoff_state` signal (cached + replayed like `/lifter/state`),
  `arm_standoff(target_mm, timeout)`.

Verified offline only: `tools/check_scan_progress.py` 37 → **49**
(projection and sign of `standoff_state`, both sentinels, `keyence_cb`
publishing + sequence, `adjust_standoff` scoping the setpoint to −2 mm for
a 12 mm target and restoring 0 / 10 afterwards, stale cancel cleared,
non-converged result reported with its reason) and
`src/robot_ui/tools/check_task_list_ui.py` 42 → **55** (label rendering for
valid / on-target / sentinel, age-out, button disabled in flight and
re-enabled, `arm_standoff` called once with the spinbox value, log line,
Cancel → `arm_cancel`). Not run on the robot; `arm_node` and robot_ui
restart required. First thing to watch: with the tool over the plate the
line should read a plausible 8–12 mm and flip to OUT OF RANGE when lifted;
the loop's `seek_enabled` is still off, so Auto standoff needs the surface
inside the sensor's window before it will move.

### 2026-09-15 — The TF chain: no ROS TF exists; extrinsics.yaml now carries the physical (tilted) front_cam, and the chain picks the frame its detections are in

User: "모바일 로봇의 tf 체인이 있잖아. 확인해줘", then "보정값들 다 tf
matrix에 반영해줘 … tz 0.302 … 1.3° 틸트 반영, path_tag_locator도 수정".

**What the check found (read-only on the live master).** There is no ROS
TF chain for this robot: `/tf` is four disconnected islands — navifra's
`odom → base_link` (50 Hz) and each camera driver's own static subtree
(`front_cam_link`, `side_cam_link`, `hand_cam_link` roots) — no `map`,
no `base_link → <camera>`, no arm (`robot_state_publisher`, `/joint_states`
and `robot_description` are absent), no lift. Nothing in this workspace
publishes TF. The actual chain lives in config matrices consumed by code:
`map.yaml` tag → `/robot_pose` (`calculate_robot_pose`), `T_mb2fc`,
`T_ab2mb` (= `robot.yaml arm_calibration` = the planner URDF's
`mobile_to_base` under the 180° frame convention), `T_hc2ee.npz`, the
URDF `vision_tip_joint` = `vision_tip_offset_mm`, and `/lifter/height`
added at run time. Two inconsistencies: `T_mb2fc` tz 0.300 vs
`ground_plane.height_m` 0.302, and no tilt in `T_mb2fc`. ⚠️ Also noted, not
changed: navifra's `base_link` is the MOBILE base while the planner URDF's
`base_link` is the ARM base (child of `mobile_base`) — running
`robot_state_publisher` on that URDF next to the driver would give
`base_link` two parents.

**What changed.** `T_mb2fc` is now the physical camera: t (0.55, 0,
0.302), R = diag(1,−1,−1) · inv(rot_xyz(roll, pitch, yaw)) with the
2026-09-08 fit — GENERATED by `make_front_cam_extrinsics.py` from
robot.yaml, comment in the yaml says so. The trap this creates and how it
is closed is promoted to *Transform Parameters*: robot_camera_node's
detections are LEVEL-frame while the correction is on, so
`load_extrinsics_full()` derives `T_mb2fc_level`
(`ground_plane.T_tilted_to_level`, new), selects per
`locator.yaml detector.front_cam_frame` (`auto` → `ground_plane.enabled`),
and refuses a stale yaml. Switched to it: `path_tag_locator_node`,
`map_calibrator_node`, `error_budget.py`, `analyse_yaw_sweep.py`,
`robot_sim/sim_node.py` (renders through the same choice);
`verify_arm_pointing.py` keeps the physical matrix (raw frames) and says
why. `load_extrinsics()` still returns the stored matrix, so
`generate_calibration_artifacts.py` (T_ab2mb only) is untouched. Numerically
the level frame is bit-for-bit the pre-change matrix except tz, so every
calibration path behaves as before plus 2 mm of lens height.

The tilt's sign was the thing to get right and it was settled against
`GroundPlane` itself, not by reading: `to_ground` maps a camera ray r to
ground coordinates Rᵀr, so the level camera's axes in the physical frame
are the columns of R = rot_xyz(roll, pitch, yaw) — `T_tilted_to_level`.
Verified offline only: `check_front_cam_extrinsics.py` (22 — stored ==
generator to 3e-10; tx/tz invariants; auto/override selection; a level
matrix, a 0.300 tz and a tilted matrix without a fit are all refused; a
plain pinhole render through the physical matrix equals
`GroundPlane.project` to 4e-7 px; RAW corners + physical and CORRECTED
corners + level both land 40 random floor tags to 0.0000 mm / 0.00000°;
the cross pairing is off 8.1 mm / 1.38°), plus `check_repose_from_corners`
10, `check_ground_plane` 17, `error_budget.py -n 200` and
`analyse_yaw_sweep.py --self-test` 4/4 under the new loader. Not run on the
robot; the calibration nodes (`path_tag_locator.launch`) must be restarted
to pick the yaml up. `catkin_make` is needed once for the two new
`install(PROGRAMS)` entries, not for behaviour (Python only).

### 2026-09-14 — Run output moved into the workspace; old output kept or deleted

User: "이제부터 결과물 파일 그리고 로그 다 워크스페이스에 저장, 이전 거도
필요한 거는 남기고 나머지 지우기". Until now the writers were scattered:
Ra maps into `task/csv` next to the INPUT path files (the `_ra_map` stem
filter existed only to keep them from being re-read as tasks), frames
into `$HOME/scan_results` (flat — 1273 frames of three runs in one
folder), nav records and every calibration artifact into `~/.ros/…`,
robot_ui captures into `/tmp`, node logs into `~/.ros/log` (1710 entries
since July). The 2026-09-09 `log/` copy of `~/.ros` had already diverged
(today's nav records, three hand-eye runs, one 09-09 file cut mid-command).

Landed — standing section *Where run output lives* has the layout:
`paths.WS_DIR / LOG_DIR / RESULTS_DIR / …` in apriltag_nav (+ the same in
`path_tag_locator/__init__.py` and `robot_ui/paths.py`), the catkin env
hook exporting `MM_WS` + `ROS_LOG_DIR`, `${MM_WS}` in the three yaml
configs, `ws_dir` in the launch, `TaskManager(result_dir=)`, and
`arm_node` saving frames into one `<ra_map stem>/` folder per run
(`_image_dir_base`). Docs: README, HANDOVER, ROSBRIDGE_kr, the
path_tag_locator README / USAGE / TROUBLESHOOTING / CALIBRATION guides.

Data, with `diff -rq` before every delete: **kept (moved in)** today's
nav records + the complete 09-09 `GOTO_500` file, the three hand-eye runs
(12 MB), `~/calib_pair` (the tilt-fit inputs), the three Ra maps of the day
(→ `results/ra_maps`, now tracked), today's 1273 frames / 12.8 GB (→
`results/scan_images/20260914_flat_three_runs/`, unattributable, README
inside), the 10 September roslaunch runs + 190 loose node logs (→
`log/ros`, 188 MB). **Deleted:** `~/.ros/apriltag_nav` and
`~/.ros/path_tag_locator` (byte-identical to `log/`), 214 frames / 1.1 GB
of 2026-08-05..24 (retired base, old cell, pre-crop Ra), 146 roslaunch runs
+ 1363 loose logs of 2026-07-13..08-28, `~/.ros/Log` (08-05 orbbec crash
traces). Disk: 89 → 88 GB used.

⚠️ **Two live writers were left in place**: the stack and the calibration
nodes were RUNNING (a `GOTO 102` record was growing and hand-eye run
`run_20260914_185621` was open), so `~/.ros/apriltag_nav/nav_log/20260914/`
and `~/.ros/path_tag_locator/handeye_calib/run_20260914_185621/` still
exist and receive that session's output until the next relaunch. After
it: `cp -a ~/.ros/apriltag_nav/nav_log/. log/apriltag_nav/nav_log/ &&
cp -a ~/.ros/path_tag_locator/handeye_calib/. log/path_tag_locator/
handeye_calib/ && rm -rf ~/.ros/apriltag_nav ~/.ros/path_tag_locator`.

Verified: `catkin_make` clean, the hook exports both variables in a fresh
shell, every new path resolves to the workspace with and without `MM_WS`,
`roslaunch --dump-params` shows `output_dir` / `save_dir` under the ws,
suites 37 / 19 / 42 / 36 / 12 pass; `check_task_discovery.py` 47 ok, 1 fail
— pre-existing, it assumes three file pairs and the user added a `_plate2`
pair today. Not yet run on the robot: the running stack predates every
change, restart required (`arm_node`, `mobile_node`, `task_executor`, the
three calibration nodes, robot_ui — i.e. everything, in a freshly sourced
shell so `ROS_LOG_DIR` is set).

### 2026-09-14 — Hand-eye calibration from robot_ui, with an automatic sample sweep

User: "이 기능도 robot ui 추가, 그리고 촬영 포인트로 로봇 이동하고 충돌
없이 샘플해줘". `handeye_calib_node` (services capture / compute /
status / reset / load_latest, unchanged) gained **`~auto_sample`**,
**`~cancel`** and a **`~progress`** JSON stream, implemented in
`path_tag_locator/handeye_sweep.py` (pure numpy, offline-tested):

- Precondition: the operator has cross tag 0 in the hand camera (base
  on 102/103, Arm tab jog or a plan seed). The node squares up with the
  existing align maths (`compute_target_ee_pose`, tag centred, zero
  tilt, at `distances_m[0]`), takes that as sample 0 and as the frame
  the sweep is planned in (`T_ab2tag = T_ab2ee · inv(T_hc2ee) ·
  T_cam2tag`).
- Views: distance × tilt × azimuth on a sphere around the tag centre,
  optical axis through it, spins round-robin — 3 × (1 + 2×4) = 27,
  evenly subsampled to `max_samples` 24. Rotation diversity is the
  point (a hand-eye solve is blind to pure translations).
- **Collision rules, evaluated in the tag plane's frame (the plate IS
  the tag plane):** every `tool_points_mm` point — the flange and the
  vision tip / Keyence bracket (0, −253, 225.2), the lowest part of the
  tool — must stay `min_clearance_m` 0.12 above the plate; the flange
  stays within `max_xy_from_start_m` 0.30 of the start pose (the sweep
  never leaves the plate area the operator chose) and inside
  `max_flange_reach_m` 1.25; rejected views are counted by reason.
  Moves between views are straight `MoveL` chunks of ≤ 0.20 m / 30°
  (both ends inside a convex safe region ⇒ so is the straight path),
  nearest-first ordering. Each view: settle, median-of-5 re-detection
  on robot_camera_node's hand_cam topic → not seen = skipped, never a
  blind capture; a failed move skips that view; cancel is honoured
  before every move; the arm returns to the start pose at the end.
  Compute / reset are refused while a sweep runs.
- The interim 2026-09-02 hand-eye AIMS the sweep only: an error there
  mis-centres the tag (caught by the re-detection) and is why the
  clearance margin is generous. `detect.py`'s own detector now uses
  `quad_decimate` 1.0 (was the library's 2.0 — the exact setting that
  made hand_cam miss the 90 mm tag on 2026-09-02; this detector is what
  `calibrate()` re-runs over the archived samples).
- robot_ui: Calibration tab → Hand-eye group (node ONLINE/OFFLINE line
  with the launch flag, Auto-sample / Cancel / Capture here / Compute &
  save / Load latest / Reset / Status, sample count and a progress
  line; skipped views and the summary go to the log). Bridge:
  `handeye_{auto_sample,cancel,reset,load_latest,online}` +
  `handeye_progress` signal.

Verified offline only: `path_tag_locator/scripts/check_handeye_sweep.py`
(36 — view geometry: tag on the optical axis at the requested range and
polar angle, spin leaves the camera position; plan round trip: every
accepted flange target re-images the tag centred at its tilt, lowest
tool point 151 mm above the plate, each rule rejects with its reason,
longest hop 0.32 m; runner against a fake arm + limited-FOV camera:
squares up, 24 captured incl. the square view, no chunk over the
clamps, tilts 0/12/22 present, returns to start; a failed move and a
cancel handled; a 10 cm / 4° WRONG aiming hand-eye still captures 20
usable samples with nothing under 167 mm; refuses without moving when
the tag is not in view) and `check_task_list_ui.py` 34 → **42**. Not run
on the robot: first sweep with a hand on the e-stop, watching
`handeye_calib: sweep: N views planned` and the per-view lines.

**Run on the robot the same evening (18:38–18:44, base on tag 102, the
arm driven over cross tag 0 by hand):** the square-up ran its 6
iterations without reaching the 3 mm tolerance (started 250 mm off at
z 1.2 m, 0.10 m steps: 250 → 88 mm) — harmless, the views are planned
from the measured tag pose and re-detected — then **22 views planned
(2 rejected by the xy window), 23/23 captured, 0 skipped, 0 move
failures, 2 min**, arm back at the start. `compute`: DANIILIDIS,
residual 0.0173, `T_hc2ee` t = (+26.3, +165.2, −157.1) mm, rpy
(+0.45, +0.72, +1.57)°. Quality, measured rather than assumed:
- **Consistency:** the fixed tag re-projected through all 23 poses
  scatters **2.1 mm rms / 3.2 mm max**, normal 0.5° rms; the 09-02
  fitted file gives 15.1 / 24.7 mm on the very same samples, the
  spun-only one 34 / 50 mm.
- **Absolute:** tag 0 lands at (−0.399, 1.010, −0.577) m in the arm
  frame; `transform_world_to_arm` at the tag-102 stop pose predicts
  (−0.400, 1.010, −0.652 + 0.080 plate datum = −0.572): **1 / 0 / 5 mm.**
  That is the whole chain — map.yaml, the aligned base pose, `T_ab2mb`,
  the new hand-eye — agreeing to millimetres.
- New vs the file in use since 09-02: 45 mm / 2.3°, consistent with the
  ~2.5–3.3° rotation error the 09-11 analysis attributed to hand-eye or
  front_cam and could not separate. The 09-02 caveats are closed.
Follow-ups: restart the calibration nodes (they cache the npz); the
plate plans' design seeds move ~5 cm with the hand-eye — regenerate
(`generate_calibration_artifacts.py`) or keep the session-measured
plate-1 seeds, which are real TCP poses and still valid starting points.
Square-up tuning: `align_max_iterations` 6 → more, or start the sweep
from ~0.5 m above the tag.

### 2026-09-14 — rosbridge in the launch: the Windows PC reaches the stack over WebSocket through the Phoenix AP bridge

User: "rosbridge, 웹소켓으로 로봇PC와 외부 컴퓨터를 연결하고 싶은데 방법을
모르겠어" — then, once it worked by hand, "rosbridge를
mobile_manipulator.launch에 넣어줘". Topology found by probing from this
PC (read-only): default route → the Phoenix Contact AP bridge at
192.168.1.10, which is a **NAT router** between LAN 4 (192.168.1.x) and
the site router's 192.168.0.x — it holds 192.168.0.20 on its WLAN side,
and 192.168.0.1 answers pings from here while nothing from 192.168.0.x
can reach 192.168.1.100 unaided. Native ROS (`ROS_MASTER_URI`) is
therefore impossible from the Windows PC (each node needs a reverse
TCPROS connection); rosbridge is one inbound TCP port. Steps that
worked: `ros-noetic-rosbridge-server` was already installed; ufw is
inactive; a hand-launched `rosbridge_websocket` answered
`/rosapi/topics` (80 topics) and `/odom` locally; the user added the
bridge's port forward WLAN 9090 → 192.168.1.100:9090 and the Windows
client (`python -m pip install roslibpy websocket-client`; `pip` itself
is not on PATH with python.org's 3.14) connected to
`ws://192.168.0.20:9090` and listed the topics. A first local test that
subscribed `/task_state` "printed nothing" — `task_executor` was not
running, so the latched topic had no publisher; use `/rosapi/topics` as
the liveness check, not a stack topic.

Landed: `use_rosbridge` (true) / `rosbridge_port` (9090) args and the
`rosbridge_websocket.launch` include at the end of
`mobile_manipulator.launch`, `rosbridge_server` as an `exec_depend`,
`/rosbridge_websocket` + `/rosapi` as optional nodes in
`test_all_devices.py`, and the *External access* paragraph in the
Architecture section (ownership rules apply to external clients too; no
images; no auth). Verified: `roslaunch --nodes` resolves the include and
lists both nodes, `use_rosbridge:=false` drops them, `rosbridge_port`
reaches the node's `~port`; XML and Python parse. Not yet launched as
part of the stack — the hand-started rosbridge in the user's terminal
must be Ctrl-C'd first or the launch's copy fails to bind 9090.

**Later the same evening.** Exactly that happened: the user relaunched the
stack with the hand-started rosbridge still up, the launch's copy looped
on `Address already in use` and was then shut down by the master as a
duplicate name, leaving a hung process; killed it and started a
detached rosbridge by hand for the session (`pkill -f
rosbridge_websocket` before the next stack launch; moot after a
reboot). Then the first command from Windows through it: `STATE`
(answered `[STATE] IDLE`) and **`CHARGE`** — received, `battery_return`
ran, but `mobile_node` said "Already at target tag 500", no motion, relay
ON, and **no BMS current within 15 s → `dock_failed`**. Reconstructed
from the logs: the 16:59 CHARGE (after an UNDOCK) drove 0.095 m onto the
dock and was confirmed at 46 %; at 17:43:50 the user's Ctrl-C of the
stack ran `devices.shutdown()`, which **drops the charge relay by
design**, ending that charge at ~71 %; five restarts followed, the
manager never resumes charging on restart, and at 18:21 relay-ON alone
(base unmoved since 16:59) produced no current. Whether that is the
charger's state after a cut under load or the contacts is not knowable
from software; the user was told to check the charger and re-dock with
`UNDOCK` → `CHARGE`. Open decision for the user: keep the relay on at
shutdown while the BMS confirms charging, or resume `charging` on
restart when parked on 500 with the relay on — neither changed.
Delivered on top: `tools/rosbridge/robot_cmd.py` (merged from two
scratch scripts; every subcommand exercised read-only against the live
stack, incl. the fix for lowercase `state` being sent as a STATE
command) and `docs/ROSBRIDGE_kr.md`.

### 2026-09-14 — CHARGE / UNDOCK: dock-and-charge as an operator command, with robot_ui buttons

User: "주차태그 이동 및 충전 task robot_ui 추가 — `rostopic pub -1
/crevis/charging std_msgs/Bool "data: true"` 이것을 보내야 충전이 됨,
충전도크는 500번". The 2026-09-09 charging manager already does exactly
that sequence as its internal `battery_return` task (lift origin home →
`move_to_tag(dock_tag 500)` → `/crevis/charging true` → wait
`charge_confirm_s` for BMS current), but only when its own rules fire.
Now `/task_command` accepts **`CHARGE`** (queues `battery_return`, phase
`returning`) and **`UNDOCK`** (queues `battery_undock`: `/crevis/charging
false` → `undock_forward_m` forward; phase `stopped`, i.e. plain idle
lamp and no unattended return until the next user task — `full` would
have lit the "charged" colour for a pack that was merely unplugged).
Both preempt a running task like `TASK` does and need no
`charging.enabled`, which gates only the automatic rules. `/task_state`
gained `charge_phase` / `charging` / `battery_pct`, republished whenever
the lamp key changes, so the UI sees a charge start or stop while IDLE.
robot_ui: Task tab row "Dock & charge (tag 500)" / "Undock / stop
charging" with a phase line, and a `CHARGE` status chip (magenta tint
while the BMS reports current, red on `dock_failed`, unchanged for an
older executor that sends no fields).

Verified offline only: `tools/check_charging_manager.py` 13 → **19**
(CHARGE with the manager disabled runs lift home → goto 500 → relay
true → confirmed; UNDOCK drops the relay, drives 0.10 m, no return
afterwards; the /task_state fields) and `check_task_list_ui.py` 29 →
**34**. Not driven; `task_executor` and robot_ui restart required. The
2026-09-09 caveat stands: whether the designed stop pose on 500 actually
makes the charger contacts is unverified — `dock_reverse_m` is the knob.

### 2026-09-14 — "拍照后出结果为什么这么慢": inference + PNG moved off the arm's thread; mono frames end to end

User question after the first 105-point pose run (15:16, `scan_pose_
errorX_…050mm`): why the wait after the shutter. Read off that run's
arm_node log, per point (6.6 s cycle): MoveJ 0.96 s, `stabilization_time`
0.5 s, Keyence 0.48 s (five out-of-range readings → "0 steps"), a fixed
0.5 s sleep, capture 0.39 s, **ONNX 1.7–1.9 s**, then **1.87 s between
the `Sample` line and `Ra map saved`** — `_capture_frames` decoded the
mono8 frame as `bgr8` (5472×3648×3 = 60 MB), published it on
`/scan/image` with nobody subscribed, and `cv2.imwrite` PNG-encoded the
three channels (measured 1.31 s here; the same frame single-channel 0.62 s,
BMP 0.04 s). So of the ~4.2 s after the shutter, 1.9 s was scoring and
1.9 s was bookkeeping that never needed the arm to wait.

Three changes, all in the scan loop / pipeline, none touching motion:

1. **Capture and processing are split** (`RaScanPipeline.capture` /
   `process`; `scan_point` = both, kept for tools). `execute_scan_points`
   captures with the arm at rest, publishes `done`, enqueues the frames and
   moves to the next row; one `scan-infer` worker per scan (FIFO, bounded
   `~infer_queue_max` 4 so a slow model throttles the arm rather than
   growing memory) runs inference + `/scan/*` publishes + the PNG, fills
   the row in place under `_results_lock`, rewrites the CSV and publishes a
   new **`result`** event (ra_mean / ra_std / num_samples). The end-of-scan
   home move overlaps the last inference; `finished` / `/scan_finished`
   wait for the queue to drain, on cancel too (frames already taken are
   scored, so the CSV never ends with blank rows for measured points). A
   raise inside the worker marks that row `(no Ra)` and continues.
   robot_ui logs `OK` on `done` and `ra=` on `result`; the SCAN chip counts
   only on `done`/`failed`.
2. **Mono end to end.** `imgmsg_to_cv2(…, 'passthrough')` for mono8, the
   model's own GRAY→RGB widening (checked on a real frame against the
   pre-widened BGR: Ra 0.258414 both, bit-identical), `/scan/image` as
   mono8 and only when subscribed, single-channel PNG. The saved files are
   the same pixels, one channel instead of three identical ones.
3. **The 0.5 s after the standoff loop is skipped when the loop took no
   step** (`travel_mm == 0`); when it did move, its own `settle_s` per
   MoveL plus this 0.5 s stay.

Expected cycle ≈ move 1.0 + stabilise 0.5 + Keyence 0.5 + capture 0.4 ≈
2.5–3 s with the ~2.5 s of scoring hidden behind the next move (the
worker keeps up: 8-core, 4 ORT threads, 1.3–1.9 s per frame). Verified
offline only: `tools/check_scan_progress.py` 28 → **37** (Ra arrives in a
`result` after `done` and before `finished`, on the `scan-infer` thread;
row filled before `execute_scan_points` returns; worker survives an
inference raise; settle only when the standoff moved) and
`src/robot_ui/tools/check_task_list_ui.py` 26 → **29**. `arm_node` and
robot_ui restart required. Found on the way and fixed:
`check_task_discovery.py` hard-coded the errorY files' tags and counts
and broke when the user swapped in errorX; it now reads its expectations
from the CSVs with pandas (48 ok).

⚠️ Seen in the same log and NOT fixed: every point read `standoff NOT
corrected: out of range on the far side` — the Keyence saw no surface
within its ±20 mm window at any of the 105 points (`seek_enabled` off),
so those Ra values were taken at the CSV's raw height with no standoff
correction. Worth checking the standoff geometry before trusting them.

### 2026-09-14 — "joint 값은 정확한데 _exec_pose는 같은 지점에 안 간다": the controller's active tool is the flange, and the new CSV's euler is ZYX

User (pinyin): the two execution paths do not reach the same place; the
joint values are correct. Settled by forward kinematics, not by reading
conventions: an offline FK of the FR10v6 URDF (base_link → tool_Link,
`tools/check_pose_vs_joint.py`) reproduces the robot's own home TCP
(−159, 700, 774) mm to 0.0 mm, so it is trusted as the flange; the
planner's `assigned_workpoints_*` rows equal URDF-flange + (0, −253,
225.2) of their paired `rrt_final_path_*` joint rows to **0.000 m over all
943 points** (the 13:42 export; the 13:01 one was in another frame — the
user re-exported after the 13:20 diagnosis). So the planner is internally
consistent and `transform_world_to_arm` reproduces its positions exactly.
The two paths still diverge, for two reasons, both fixed:

1. **Tool frame.** `/arm/state` at home == the URDF FLANGE ⇒ the
   controller's active tool offset is zero; tool 1 from `set_tool_tcp.py`
   is not in effect (at least since 2026-09-04, when the same home TCP
   was recorded). `GetInverseKin` uses the active tool, so the tip
   coordinates were solved as flange targets: tip 338.7 mm off (+232 mm
   z), and many targets "unreachable" at 1.6 m that are fine for the tip.
   Fix in software (`_probe_tool_frame` / `_tip_to_flange`, see the
   Coordinate Frames section for why the controller was NOT changed).
2. **Euler convention.** The planner's `[rx, ry, rz]` are ZYX-intrinsic
   (rx = yaw); the code's hard-coded `'zyx'` was 180° off. Now
   `arm_calibration.csv_euler` (default `"ZYX"`), with the 2026-09-11
   finding recorded as true for the old generator only.

**The planner's URDF is
`frcobot_description/urdf/fr10v6_mobile_vision_0317_test.urdf`** (user,
same day: "이것이 현재 사용하는 urdf"; untracked until this commit). It
settles the frames by construction: same j1–j6 + `tool` (0.106) chain as
`fr10v6_vision.urdf`, `vision_tip_joint` = tool_Link + (0, −0.25299,
0.2252) with rpy 0 — exactly tool 1 / `vision_tip_offset_mm`, so the CSV
x y z are the `vision_tip` link and its rx ry rz the flange orientation —
and `mobile_to_base` = (0, +0.1, 0.652), rpy 0, which is `T_ab2mb`
(0, −0.100, −0.652) seen from a mobile_base frame yawed 180° from the
code's body frame. `check_pose_vs_joint.py` reads that file for its FK
chain and asserts both offsets against `robot.yaml`.

Verified offline only: `tools/check_pose_vs_joint.py` (24 — the URDF's
tip and mount offsets; embedded
paired rows for tags 106 and 119: ZYX 0.000°, zyx 180°, position 0.01 mm;
`_exec_pose` against a fake Fairino sends exactly the URDF flange pose of
the joint row when `GetTCPOffset` is zero (0.01 mm / 0.000°), the tip when
the tip is active, refuses on anything else; probe verdicts), plus
`check_scan_progress.py` 28 and `check_lift_compensation.py` 12 still
pass. Not run on the robot; `arm_node` restart required. **First thing to
watch at startup:** `[Arm REAL] Active tool frame is the FLANGE … will be
converted` — then a pose task's `tip target … -> flange target …` lines,
and the tip physically where the joint replay put it.

### 2026-09-14 — Pose scan: every point "cannot unpack non-iterable int object" — IK failures were invisible, and would have been "Success"

User, 13:20, `TASK scan_pose_errorX_p000mm_standoff_050mm_height_652mm` at
tag 106: `[Arm REAL] Move failed at point 110: cannot unpack non-iterable
int object`, 110 of 110 attempted points, then STOP. Two layers:

1. **The message is a TypeError, not the reason.** `_exec_pose` did
   `ret, joints = self.robot.GetInverseKinRef(...)`; the Fairino SDK
   returns a BARE INT error code when IK has no solution (the tuple only on
   success), so the unpack raised. That raise is the only reason the
   points were recorded as failures at all — see 3.
2. **The reason: the CSV's points are 1.65–2.56 m from the arm base.**
   Reproduced exactly: the file's group-106 first row, world
   (0.285, 1.714, 0.385) m, through the real `transform_world_to_arm` at
   the logged `/robot_pose` (x −0.000, y 1.711, θ 90) gives the logged
   target (−1714, 1896, −267) mm; FR10 reach is 1.40 m. The CSV world
   frame IS the `map.yaml` frame (origin 정반 1 centre — `calculate_robot_pose`
   swaps/negates into the manipulator frame and `arm_transform` swaps back),
   so at tag 106 the base is at (−1.711, 0.0) and the points would have to
   lie within ~1.2 m of it; the file puts them near the plate centre and
   1.2–1.7 m up the lane. Same finding as 2026-09-11 (group → tag
   assignment / planner frame), now on the user's 13:01 `errorX` export;
   the user confirmed this reading ("2번이 맞습니다"). Planner-side.
3. **Latent and worse: `_exec_joint` / `_exec_pose` logged failures and
   RETURNED.** `execute_scan_points` then marked the point `Success`, ran
   the Keyence loop and captured wherever the arm was. Both now RAISE
   (`RuntimeError`), the per-point `except` records the message, and the
   scan continues — the contract that was already assumed. The IK message
   names the code, the arm-frame target, its distance from the base, the
   CSV world point and the robot pose, so the next such failure diagnoses
   itself. `_ik_result()` normalises the SDK's int-or-tuple return.
   `tools/arm_controller_sdk.py` still has the old shape (already flagged
   as needing work before `arm_node` is pointed at it).

**"UI에 로그 보이게" + "로봇팔 실시간 state 안 보임" (user, same
message):**
- `/arm/scan_progress` (String JSON from `ArmController`: `start` with
  totals, `move` per point, `done` with `ra_mean`/message, `failed` with
  the reason, `finished` with `n_ok`/`n_fail`/`cancelled`; every event
  after the first successful move carries `tcp_pose`). robot_ui: a `SCAN
  i/N ok a fail b` chip in the status bar (red once anything fails) and
  one log line per work point (`[scan] 110/468 pt 110 g106: FAIL — IK
  failed (code 112): …`); traverse rows update the chip only.
- **`/arm/state` was blind for the whole scan.** arm_node's state timer
  skips the pose read whenever a motion holds the executor lock — and a
  scan holds it for minutes — so `pose_valid` was false and the Arm tab
  showed `—` throughout. The worker now snapshots pose + joints after
  every MoveJ / standoff MoveL / capture (`_refresh_live_pose`, between
  its own RPC calls, so no socket collision), and the timer serves that
  snapshot as valid while it cannot take the lock (`~live_pose_max_age_s`
  60). Exact between moves, a few seconds stale during one.

Verified offline only: `tools/check_scan_progress.py` (28 — real
`ArmController` without `__init__` against a fake Fairino whose IK returns
a bare int: the verbatim CSV row fails with the full reason and no MoveJ /
capture, the reachable point completes with its Ra, event sequence and
counts, live pose after the first move, MoveJ error code, traverse rows,
5-value row, seedless IK path, `_ik_result` shapes) and
`src/robot_ui/tools/check_task_list_ui.py` (26: chip texts per phase,
failure reason in the log, Ra line, traverse silent, finished summary).
Not run on the robot; `arm_node` and the UI must be restarted.

### 2026-09-14 — Tasks come from the files in task/csv; robot_ui reads /task_list

User: "task 디렉토리에 있는 경로데이터 기반으로 동작할 수 있도록 수정해줘" —
`assigned_workpoints_*` are end-effector pose paths (x y z rx ry rz),
`rrt_final_path_*` are joint-angle paths — "수정되면 당연히 robot_ui도 수정".
Before this, `TASK_DEFS` named the six 2026-09-11 files by hand, the three
joint entries were commented out, and robot_ui's Task combo still listed
`scan_joints_line1` & co. — names deleted three days earlier.

- **`TaskManager.discover_task_defs(task_dir)`**: `assigned_workpoints_<key>.csv`
  → `scan_pose_<key>` (pose mode, `joint_file` = the rrt file of the same key
  when present, for the IK seed), `rrt_final_path_<key>.csv` →
  `scan_joint_<key>` (joint mode, `pose_file` for the Ra map's world x y z).
  Each file registers on its own; pairing is by key. Result CSVs are
  `<task>_ra_map_<ts>.csv` and stems containing `_result` / `_ra_map` are
  skipped, so results written into task/csv (as the 2026-08 runs were) never
  come back as path data. `TASK_DEFS` is kept as an explicit-extras layer
  merged OVER the discovered set (same name wins, announced), normally empty;
  the 2026-09-11 `groups: [104]` bring-up entry is the commented example.
- **Joint tasks are registered again**, on the user's instruction. The
  2026-09-11 finding (group → tag assignment does not check out; MoveJ
  replay has no reach/collision check) is unchanged and is now a `logwarn`
  per `scan_joint_*` task at load plus the scan-CSV section above. Run the
  `scan_pose_*` twin of a new pair first.
- **`/task_list`** (latched JSON from `task_executor`, `TaskManager.
  describe_tasks()`): per task name, kind, scan_mode, tags in order, work /
  traverse point counts, IK-seeded or xyz-paired counts, lift height, files,
  result name. **`RELOAD_TASKS`** re-scans the directory (refused while a
  task is running or pending — the running task holds the old manager's
  scan points).
- **robot_ui** fills the Task combo from `/task_list` (cached + replayed
  like `/task_state`), shows a detail line (`mode · tags · N pts (+M
  traverse) · lift · file · paired file`) and per-item tooltips, preserves
  a hand-typed name across republishes, and has a "Reload tasks" button.
  `readme.txt`, `tools/send_debug_cmd.py` and a `lifter_node` warning string
  lost their stale names.

**Same day, on the robot: "매니퓰레이터의 첫 home pose에서 경로데이터의 첫
point_id로 이동할 때 뚝뚝 끊기면서 움직여" — diagnosed, "fixed", and the fix
BACKED OUT on the user's word.** From the 11:19 arm_node log: group 104's
home → point 14 is 1 home + 12 transition rows, each an exact 5.72° step on
every joint (a straight joint-space line, sum 74.4° = the direct distance),
and each row went out as its own blocking `MoveJ(..., blendT=-1)` taking
0.88 s — 12 stop-starts over 11.5 s. A load-time fold of collinear traverse
runs into one `MoveJ` (all 728 runs in the three files are straight to
0.0000°) was implemented and verified offline, then removed within the
hour: the user confirmed the row-by-row motion is the intended behaviour
("경로데이터대로 잘 가고 있는 거였어"). Nothing of it remains in the code;
the RRT-dialect section now says so, so it is not rebuilt.

**Speeds halved in all six path files (user instruction, same day):** the
`speed` column's 60 → 30 and 30 → 10, in place, nothing else touched (BOM,
CRLF and the scientific-notation cells preserved; verified field by field
against HEAD for the rrt files). `speed` feeds `SetSpeed(percent)` per row
in `execute_scan_points`, so every move — work points and transitions — now
runs at half the planner's percentage. `test_move_57.csv` (2026-08-12
bring-up scaffolding, no task referenced it) deleted on the user's
instruction; the `*_result_*.csv` files that sat beside it were gitignored
and are gone too.

Verified offline only: `tools/check_task_discovery.py` (48 checks — real
task dir: 3 + 3 tasks, 1035 work points each, every pose point IK-seeded,
joint path 1035 work + 422 traverse with world xyz on every work point and
none on traverse rows, per-row speed 10/30, standoff-050 tags differ;
scratch dirs: result files ignored, unpaired pose / joint file registers
alone, explicit TASK_DEFS with `groups` overrides and is announced, a
disagreeing `lift_mm` refuses that file only, empty CSVs register nothing)
and `src/robot_ui/tools/check_task_list_ui.py` (offscreen MainWindow +
fake bridge, 20 checks). Not run on the robot; `task_executor` and the UI
must be restarted. Task names on the wire are now long
(`scan_pose_errorY_p000mm_standoff_010mm_height_652mm`) — exact, on
purpose; shorten them by renaming the files' key, not by editing code.

### 2026-09-11 — Pose-mode IK finally tracks the lift; and the CSV euler convention was documented backwards

Second half of the day. Asked what still limits driving the arm to a WORLD
point now that navigation is good. Answer had four parts, of which this
entry does the one that was an outright bug.

**`arm_base_z` now tracks the lift.** `transform_world_to_arm(g, msg,
lift_m)`; `_exec_pose` feeds it `/lifter/height` through a new read-only
`lift_height.LiftHeightListener`. Detail promoted to its own section above.

Three decisions inside it worth keeping:

- **Read-only listener, not `LiftClient`.** `LiftClient` already exposes
  `height_mm`, but it is the COMMAND proxy — it publishes
  `/lifter/height_cmd` and holds home/stop service proxies. Handing
  `arm_node` the ability to move the lift to answer "how high is it" is
  more authority than the question needs, and CLAUDE.md's rule is that
  reading stays open while `lifter_node` is the sole writer.
- **Unknown ≠ origin.** `height_m()` returns None before the first
  message. `~require_lift_height` picks the policy: default false keeps
  today's assume-the-origin behaviour (correct by convention — every task
  ends with lift origin homing and pose CSVs carry no `lift_height`) but
  logs it at **error** level; true fails the move, for unattended runs.
  Defaulting to false was deliberate: with `lifter_node` in the launch the
  unknown case is degenerate, and introducing a new way to fail a working
  task was not worth it.
- **The direction is the opposite of what "the arm will crash" intuition
  says.** No mount tilt ⇒ `R_AW` is a pure z-rotation ⇒ omitting the lift
  makes `p_arm[2]` too LARGE and the TCP lands `lift_m` **ABOVE** target,
  away from the plate. Wrong measurement, not collision — and not
  self-correcting, since the Keyence loop is clamped to 1 mm/step and
  cannot close 150–300 mm.

⚠️ **Found while reading that code: the CSV euler convention is documented
backwards, and the CODE is the correct one.** `arm_transform.py` uses
`from_euler('zyx', [rx, ry, rz])` — scipy lowercase, i.e. extrinsic, i.e.
`XYZ`-intrinsic(rz, ry, rx) — while this file and the module docstring both
claimed "ZYX intrinsic". Settled physically, not by argument: over
`grid_path_line1.csv` the code's reading puts the tool z-axis at
(−0.089, −0.037, −0.995), down at the plate, and the documented reading
gives (0.036, 0.999, −0.003), horizontal — **116–124° apart across the
file**. Docs fixed at both sites with a "do not 'fix' this line to match a
doc" note. Nobody has been bitten yet because the only consumers are CSVs
generated by the same convention; the trap is for the NEXT generator or a
hand-written world-point command.

**Verified offline; nothing ran on hardware.** New
`tools/check_lift_compensation.py`, 14 checks: `lift_m=0` reproduces the
old result bit-for-bit (max diff exactly 0); only arm-frame z moves and by
exactly the height (x/y 0.0, orientation 0.0); the compensated TCP lands on
the requested world z to 7e-14 mm across four base poses × four targets ×
five heights, while the uncompensated one is off by the full 343.35 mm
stroke; a flipped sign would be 2× the lift off, so the sign is genuinely
tested rather than assumed; the unknown-height policy in both directions.
The three other suites plus the morning's still pass (ground_plane 15,
nav_sequencing 14, charging 14, repose_from_corners 10).

Not done, and the reason: `tools/arm_controller_sdk.py` holds a second copy
of this geometry that did not get the fix. It is a variant `arm_node` can
be pointed at but already needed scipy-compat work before any such switch,
so it got a docstring warning listing both rather than a silent divergence.

**What this does NOT fix**, from the same question — the remaining error in
driving the arm to a world point, roughly ranked:
1. `/robot_pose` is anchored to **map.yaml's** tag coordinate
   (`calculate_robot_pose` reads `tag_info['x'/'y']`), so navigation being
   accurate relative to a tag does not make the reported WORLD coordinate
   accurate. The arm inherits map.yaml-vs-reality 1:1. Blocked on the
   calibration work in the entry below.
2. align converges to 0.2°, which over a 0.7–1.2 m arm reach is 2.4–4.2 mm.
3. `camera_offset` 0.55 vs the 0.547 tape measure, still unreconciled.
4. arm absolute accuracy + the vision_tip TCP — repeatability (±0.05 mm) is
   not accuracy, and neither has been measured on this robot.

### 2026-09-11 — front_cam's pose fields are not a 6-DOF measurement; the locator chain now re-solves from the corners

Started from "how do I make map calibration more accurate". Answered from
the 9 archived plate-1 sessions (09-08/09-09, 25–26 tags each) rather than
by reasoning, and the answer moved twice.

**Random error is not the bottleneck.** Session-to-session repeatability is
sd_xy median **2.9 mm** (P95 4.3), sd_yaw 0.09°. The 0.5 m view height and
the 5-frame mean are doing their job; tuning detection further buys little.

**The dominant term is a ~2.5–3.3° ROTATION error, and the world frame is
cleanly ruled out.** Expressing each tag's normal error as a vector in
three candidate frames and asking which one holds it constant:

| frame | explains |
|---|---|
| **world** (sloped floor / tilted ref tag) | **0.2 %** |
| hand-cam (hand-eye rotation) | 85.0 % |
| mobile-base (front_cam / T_ab2mb) | 87.8 % |

So `reference_tags.yaml`'s face-up assumption is **confirmed** (its
*positions* are still unverified). But hand-eye vs base frame is 85 vs 88 —
**undecidable from this data**, because a calibration session only ever
spins the camera about its own optical axis, and that is precisely the
motion a vertical-tilt signature cannot see. Corroborating signature: the
400 mm spacing error is a clean function of the plan's per-entry camera
yaw (Δyaw +45° → −7.3 ± 0.5 mm, n=4, consistent across zones B and C;
r = −0.81), and it is repeatable to 0.4–1.3 mm across the 9 sessions.

⚠️ **An offline hand-eye refit was built, validated and NOT applied.** It
fits well (three independent subsets agree to 0.1° in rx, condition number
3.9 after re-parameterising in the CAMERA frame, held-out tilt 3.29 → 1.57°
and spacing rms 4.78 → 2.40 mm) — and it still makes the **absolute**
position worse in every variant (zone B/C differential dx +6.6 → +15.6 mm,
z −63 → −109 against a design −80). That is the fit being forced to
attribute 3.1° to one of two frames the data cannot separate. Shipping it
would have looked like an improvement on exactly the metrics it was fitted
to. Scripts are in the session scratchpad, not the repo.

**Then the real defect surfaced, and it is upstream of all of that.**
`robot_camera_node` with `robot_camera.ground_plane.front_cam` enabled
publishes a **navigation-shaped** detection: `pose_x/pose_y` are ground
coordinates relative to the lens nadir, `pose_z` is the **configured lens
height** (the measured depth is discarded), and the orientation fields keep
dt_apriltags' **raw, uncorrected** values. `detection_to_T_cam2tag` was
consuming that as `T_fc2B` — corrected position + uncorrected rotation +
an asserted depth.

Confirmed from the archive, not inferred: front_cam `pose_z` has 25 distinct
values per session on 09-08 (sd 1.26 mm) and is **exactly 0.302000, one
value, 25 times** on 09-09. So the five 09-09 sessions ran on a corrupted
`T_fc2B`; 09-08 is the methodologically clean set (it gives the same
picture: 2.68° tilt, world frame 0.8 %).

⚠️ **This is structural, not a coding slip** — `GroundPlane.to_ground()`
intersects every ray with the floor plane, so depth is an *input
assumption* and no 3D rotation is ever formed. The module is a 2D floor
rectifier for navigation and **cannot** produce a 6-DOF pose. It is right
for `mobile_controller`, which works in pixels with `z / fx`; it was never
meant to be metrology. The navigation result it was built for stands
(108 hops, lateral +8.8/−11.6 → +1.9/−3.3 mm).

**Fix: the consumer re-solves.** The *corrected corners* are published, and
they are a level, distortion-free virtual camera's pixels at the SAME `K`,
so `detections.pose_from_corners()` (cv2 `solvePnP`) recovers the honest
6-DOF pose. `detection_to_T_cam2tag(..., camera_K=)` routes to it;
`detector.front_cam_repose_from_corners` (default true) plus the existing
`front_cam_info` topic drive it, K fetched once per session and cached. A
missing CameraInfo degrades to the old path with a loud warning rather than
failing a session.

Deliberately NOT done: changing `robot_camera_node`, the `.msg`, or the
navigation fields. That keeps a verified-on-hardware path untouched, needs
no `catkin_make`, and puts the metrology where the metrology consumer is.
hand_cam is unaffected (no ground_plane) and keeps the euler path.

⚠️ **The corner ordering is the whole correctness argument** and was
established empirically, not read off a doc: the object points must be
`(-h,+h), (+h,+h), (+h,-h), (-h,-h)`. The mirrored ordering is **42–180°**
out, not subtly wrong — it would have silently corrupted every calibrated
position. `scripts/check_repose_from_corners.py` (10 checks) pins it,
including against dt_apriltags itself via a rendered `cv2.aruco`
36h11 tag: the re-solve reproduces the library's own pose from the
library's own corners to **0.070° / 0.040 mm**, i.e. it is not worse than
the library.

**Verified offline; nothing ran on hardware.** New check 10/10; the three
existing suites still pass (ground_plane 15, nav_sequencing 14, charging
14); config loads with and without the new key (old configs default to
true); end-to-end through the real `detection_to_T_cam2tag` on a
synthetic-but-exact tilted-front_cam scene, the old path leaves a constant
**1.327°** false tag tilt in `T_fc2B` and the new path leaves **0.000°**.

**Falsifiable prediction for the next on-robot session:** the base-frame
tilt should drop by ~1.33° (≈2.5° → ≈1.2°). If it does, the remainder is
hand-eye and the 85/88 ambiguity is resolved by subtraction — i.e. this fix
doubles as the disambiguating experiment. If it does not, `T_mb2fc` itself
is wrong by more than the ground-plane fit says.

### 2026-09-11 — Scan CSVs replaced: RRT-planned paths for the new cell

The user supplied `Test_path_3_0911.zip` — six CSVs re-solved for the
REPLACEMENT base — and asked to swap them in and delete the old ones. The
old ones went; the new ones needed loader work first, so this is not a
file swap.

**What they are.** Three standoffs (10/30/50 mm), each a pair:
`rrt_final_path_*` (joint) + `assigned_workpoints_*` (pose). Every file
carries `base_height_mm` 652 and `lift_mm` 0, and the group ids are valid
new-cell tags. Registered as six tasks — the standoffs are SEPARATE because
the standoff changes which tag each work point is assigned to, not just the
offset (010 uses {104-107,118,119}, 050 uses {104,105,106,119,120}).

**The joint files are PATHS, not point lists.** ~30 % of their rows are
`transition`/`home` waypoints. The arm must DRIVE THROUGH them — that is the
collision-free route, and skipping them would send it straight between work
points — but must not settle, run Keyence, capture or record there. So the
scan loop gained a `scan` flag: the move executes, then `continue`. Keyence
is the one that would actually misbehave, chasing a meaningless reading at a
pose nowhere near the surface.

**Three dialect differences, all silent failures if missed:**
- pairing key is `source_point_id`, not `point_id` — the RRT file renumbers
  `point_id` along the path. Pairing on `point_id` matched 797 of 1035 and
  dropped the rest without a word.
- `lift_mm` not `lift_height` — without an alias the file reads as "no lift
  column" and the lift is never commanded. Harmless at 0, wrong at anything
  else.
- `assigned_workpoints_*` writes integer columns in scientific notation
  (`0.000000000000000000e+00`), which plain `int()` rejects outright.

Found while answering a follow-up: `ScanResultWriter.begin` seeded a result
row for EVERY queued point, so a run would have written 422 permanently-empty
rows into the Ra map — and a transition's `point_id` is a path index that can
collide with a real work point's key and overwrite its metadata. Now skips
non-scan points.

🛑 **target_line 2 is not trustworthy and is documented as such rather than
withheld.** Groups 118/119/120 sit 3.7-4.6 m from the arm base of the zone-C
tags they are assigned to — they are in zone B's region. Line 1 is 0.6-2.0 m,
matching the known-good older files. Pose mode fails IK there (safe); joint
mode drives a valid planned trajectory to the wrong place. Needs the
generator's author; run line 1 first.

⚠️ **A methodology note, because two of my checks were wrong before they were
right.** A flange-reach screen said 92.7 % of the new points were
unreachable — but running the same screen on the OLD CSVs with the OLD map
coordinates, a combination that demonstrably drove the robot, gave 3.2-6.6 m.
A control that fails on known-good data invalidates the method, not the data;
the claim was withdrawn. What survived is a frame-assumption-free check
(horizontal distance from the arm base implied by `world_x = -msg.y`), which
reproduces 0.90-1.43 m on the old known-good line 1 and is what flags line 2.

Offline only. 6 tasks register with 0 errors, routing verified from
`START_TAG` 500, traverse points carry no world coords and seed no result
rows, older CSV dialects load unchanged.

### 2026-09-11 — Chain-error diagnosis: a camera-yaw sweep; dual-anchor built then removed

Third part of the day. Question was "how do I decide whether the ~2.5-3.3 deg
rotation error is hand-eye or front_cam" — 79.6 % vs 82.3 % on the clean
09-08 data, a tie, with both fixes expensive.

**The answer is a CAMERA-YAW SWEEP**: one path tag, one ref tag, N camera
yaws, base stationary. `T_ab2mb @ T_mb2fc @ T_fc2B` is then identical in
every entry and cancels, so front_cam CANNOT appear in the variation; a
hand-eye error is fixed in the EE frame and the yaw rotates the EE, so the
computed position traces a circle whose RADIUS is the arm-side error and
needs no ground truth. Forward-modelled through the real chain: front_cam
rotation 2 deg -> 0.00 mm of spread, hand-eye rotation 2 deg -> 22.20, and
hand-eye translation 20 mm -> 21.79. Plans + `analyse_yaw_sweep.py` (circle
fit, verdict, 4/4 self-test through the real chain), robot_ui plan selector.

⚠️ **A DUAL-ANCHOR design was built first, then removed the same day** —
same tag from two ref tags at a PINNED camera yaw. It was right for the
premise it was designed under (the cross tags were the suspect, so pin the
yaw to cancel the chain and expose the anchors) and **exactly backwards**
once the user said the cross tags are embedded in precision-machined slots
and the CHAIN became the target: the pinned yaw cancels the very thing being
measured. Measured on the same forward model it moves 0.47 mm for a 2 deg
hand-eye error and 0.00 for hand-eye translation. Deleted along with
`solve_reference_yaws.py`, `fit_reference_tags.py` and the hand-survey doc;
git has them if the cross tags turn out to be printed inserts (registration
gives ±0.13 deg of ref yaw, which dual-anchor sees and the sweep cannot).

**The lesson, since it cost three wrong turns in one day:** I carried a
design across a premise change without re-deriving it. The same session also
produced a wrong gauge-freedom claim (a common in-place spin of a ref column
IS observable — the anchors sit in different places) and a "negligible"
verdict computed from the MEAN of a displacement vector that the varying
camera yaw had cancelled to ~zero. All three were caught by simulating the
real chain rather than reasoning; that is the habit worth keeping.

⚠️ **`install(PROGRAMS)` did not list the scripts the docs tell operators to
`rosrun`** — `analyse_yaw_sweep.py` was named in the plan headers and in the
robot_ui note while not being installed, and `error_budget.py` /
`update_plan_seeds_from_session.py` had the same pre-existing gap. Fixed,
with a comment at the list saying why it has to stay in step.

Also quantified, for whoever chases the open 17 mm z: `pose_t` is linear in
tag size, so **1 % of hand_cam scale = 4.75 mm of path-tag z**, and −80
needs 0.9658 (a 86.9 mm black border, or fx 3.54 % off) — a ten-minute
check. And hand_cam's principal point is **0.79 mm of path-tag error per
pixel**, 1:1, which shows up as per-tag SCATTER rather than bias because the
camera yaw rotates it differently per entry: invisible to any mean-deviation
check and to session repeatability. Reasoning and procedure in
`path_tag_locator/docs/chain_error_diagnosis.md`.

### 2026-09-09 — Charging manager: 85 % undock, 20 % return, return after every task, /crevis/charging true after docking

User request over `/bms/state`: stop charging at 85 % and come forward
a little, at 20 % (first said 30, changed the same day) stop work and go
back to the charger, a distinct lamp colour while
charging and when charged (first red / green, then magenta / white);
then (same day) also return to the
charger after a task completes, and — the operating fact that shaped
the design — `rostopic pub /crevis/charging true/false` is the charge
start/stop command and the charger only starts on an explicit true
after docking. Built into `task_executor` as two internal tasks queued
by a per-tick rule evaluation (see the *Charging manager* section);
`NavifraDevices.charging_by_bms()` and `MobileClient.drive_distance()`
were added for it, `run()` was split into `_tick()` so the manager is
testable offline. The robot was charging at 57 % (+16 A) when this was
written; nothing has been driven. First things to watch: after a `GOTO
500` the log line `[Charge] docked — /crevis/charging true` followed
within 15 s by `charging confirmed by the BMS`; at 85 % `/crevis/charging
false`, the BMS current dropping, then a 0.10 m forward move and a green
lamp. `task_executor` restart required.

### 2026-09-08 — Session index (evening): what changed today and what tomorrow starts with

Six things landed on `real` today, all offline-verified, the first three
also driven; details are in the dated entries below this one and in the
standing sections named here:

| commit | change | driven? |
|---|---|---|
| `4388538` | pivot finishes ON its exit tag (odom → tag error → slow inside 5° → delay-led stop) | yes, 19 hops |
| `b310326` | the FIRST hop of every command aligns on its start tag first (move, pivot, dock alike) | yes |
| `5401ace` | forward/reverse arrivals: `steer_mode: aim_and_drive` — stop at first sight, aim the base centre at the stop pose (plate-wall cap), drive straight, stop align lands the lens on the tag | yes — aims converged, but ±10 mm lateral bias → next row |
| `f738a10` | front_cam is tilted 1.3° (roll +1.228, pitch −0.504, lens 302 mm): `robot_camera_node` now re-images detections through a level virtual camera; lever confirmed 0.552 m | **driven 2026-09-09, 108 hops: aligned lateral +1.9 ± 1.7 mm forward / −3.3 ± 2.2 mm reverse (was +8.8 / −11.6), yaw ±0.2°, no wall cap hit.** The residual, still direction-signed, is next (camera yaw −0.38° uncorrected ≈ 1.3 mm; the rest unexplained) |

**2026-09-09 (later):** the linear stop lead got its own constant,
`stop_latency_linear_s: 0.22` — 247 stops at 0.010 m/s rolled 2.1 mm
forward / 2.6 mm reverse after the trigger, not the 5.5 mm the 0.55 s
lead assumed, so every stop rested ~3 mm short (user: "기준선보다 아주
미세하게 덜 도착"); yaw keeps 0.55 (validated align). Also used by the
manual drive's braking lead. Not driven.

**2026-09-09:** overlay relaid out on the user's request (see *Reading tag
ID and orientation off the image*): pixel text gone, stop columns in mm,
per-tag `ID / offset (mm) / degree` block top-left, frame rectified to the
level virtual camera when the correction is on. Then the measured camera
yaw (−0.38°) was added to the correction as `ground_plane.front_cam.yaw_deg`
(user request); not yet driven — `robot_camera_node` restart required.
Same day, user rule: **a forward hop after a reverse arrival first
re-seats the start tag from the REV column onto the FWD column with the
normal arrival algorithm, then aligns, then hops** (`go_to_next_tag`
move branch, `reseat_*` keys). The lost scratch suites were replaced by
two repo-resident checks: `tools/check_ground_plane.py` (17) and
`tools/check_nav_sequencing.py` (10: re-seat sequencing + plant run,
first-hop rules, pivot). Not driven; `mobile_node` restart required.
**Then (user, calibration order):** the re-seat also runs at the END of
a command whose last hop arrived in reverse (`reseat_at_command_end`),
so the arm — calibration view move or scan — only ever works with the
tag on the FWD column; `check_nav_sequencing.py` → 14 (a reverse-ending
`move_to_tag` returns after `align → pp(rev) → align → pp(fwd 0.16) →
align` with the tag on the crosshair; the next forward command has
nothing to re-seat; key off restores the REV-column finish).

Also today, other sessions: the Keyence standoff rewrite (uncommitted in
this checkout) and the calibration ref-tag re-pairing (uncommitted). A
dev worktree (`~/mobile_manipulator_ws_dev`, branch `dev-20260908`) was
made in the morning and retired the same day — work goes into this
checkout directly; the worktree can be removed.

Standing rules that came out of today, promoted into the sections above:
every stop aligns and every command's first hop aligns first (align
section); `aim_and_drive` and its wall cap (align section + `robot.yaml
robot:`); the front_cam ground-plane correction and how to re-fit it
(*front_cam ground-plane correction* section); calibrate with a precisely
laid tag pair as the ruler and the base's motion only as excitation —
the base under-executes small commands and yaws on its own.

### 2026-09-08 — front_cam tilt found and corrected (ground-plane projection); lever confirmed 0.552 m

First robot run of aim-and-drive (21:15–21:24, `GOTO 117/120/123/114`,
10 forward + 9 reverse hops): the aim pivots converged in one pass, yaw
at rest ±0.18°, fore-aft +2…+5 mm forward / 155–158 mm reverse, hop time
24 s — but the lens lateral after the stop align was **+8.8 ± 1.3 mm on
every forward hop and −11.6 ± 1.6 mm on every reverse hop**, independent
of the aim angle. A constant, direction-flipping bias with 1.5 mm scatter
is a model error, not control. (With state_feedback the same corridor had
shown +12 → +27 mm growing along the run.)

Chased in three steps, each using the user's precisely laid tags as the
ruler and the robot's imprecise motion only as excitation:
1. Straight-drive test on tag 118 (7 records, reverse/forward 0.15 m ×3):
   camera yaw vs travel axis −0.38 ± 0.22° — too small (1.3 mm) to be the
   cause; the base translates straight (residual < 0.8 mm per 0.14 m) but
   yaws 0.1–0.8° per segment on its own.
2. Tag pair (60 mm, 0.150 m apart) in one frame: pair spacing measured
   150.82 ± 0.08 mm at all positions (scale fine), but the two tags'
   `yaw` readings differed by an amount that GREW with image position
   (+2.2°/1000 px) — not lens distortion (undistorting with D changed
   edge angles by 0.05°), and a single tag's two parallel edges converged
   by 0.2–0.4°: perspective, i.e. the optical axis is not vertical.
3. Full fit (15 snapshots incl. 6 pivots, 120 corners): roll +1.228°,
   pitch −0.504°, height 302 mm, rms 0.33 px. The uncorrected pipeline
   reads a square-laid tag's edge at +0.67° when the tag is 0.2 m ahead
   (the aim's measuring position), +0.50° at the reverse column, −0.21°
   on the crosshair. 0.67° × the 0.75 m tag-to-base lever = **8.8 mm** —
   the forward bias exactly; the reverse column's +0.50° (align leaves the
   body −0.5° yawed, lens swings 4.8 mm, tag 0.155 m ahead adds 1.4) plus
   the −0.3° at the reverse aim position give ≈ −9 mm vs −11.6 measured.
   The lever from the pivots: 0.552 ± 0.001 m.

Fix: `apriltag_nav/ground_plane.py` + `robot_camera_node` re-imaging
(see the new section above), `robot.yaml robot_camera.ground_plane`,
`tools/fit_front_cam_ground.py` (the fit, reusable). Verified offline
(`t_ground.py`, 27 checks): identity for a level undistorted camera;
with the fitted numbers a square-laid tag reads edge 0.000° and its true
floor position anywhere in the frame; the real scans' tag-16 edge vs the
pair line goes from +0.27…+0.77° (raw) to −0.06…+0.12°; spacing
149.7–150.3 mm. Not yet driven with the correction on;
`robot_camera_node` restart required (the launch's `~driver_*` params
persist on the master, so `rosnode kill /robot_camera_node` + `rosrun`
works, as on 2026-09-02). Expect the aligned lateral bias to drop from
±10 mm to a few mm in both directions; `aim_and_drive` needs no change.

Also seen and not yet fixed: a manual `drive_distance` of 0.02 m executes
~11 mm and `pivot_angle` 5° executes ~3.7° — the stop-latency lead in
the manual profile over-compensates at tiny moves (fine for the
calibration, which never trusts the command).

### 2026-09-08 — Arrival rework: aim-and-drive — stop at first sight, point the base centre at the stop pose, drive straight; smooth path kept as an option

User: on a forward hop (105→106) the moving align is not meaningful —
the tag can be dead on the crosshair at the stop and the stop align then
rotates it off, and the base cannot crab. Diagnosed from the geometry:
the stop align turns about the BASE CENTRE, the lens is 0.55 m ahead,
so a yaw fix of θ moves the lens 0.55·sin θ (1° → 9.6 mm); the lens can
only end ON the tag if the base centre is already on the tag's line when
the stop align runs. `state_feedback` (gains 32 / 0.6) is far too slow
for that in the 0.2 m of tag visibility (closed loop ζ 1.6, slow pole
0.066 rad/s: a 20 mm base offset shrinks 40 %), and the blind-phase
predictive centering assumes the hop STARTS on the lane, so an initial
offset is never corrected.

**Two designs were built and measured on a 2-D plant** (unicycle base,
0.55 s command delay, 20 Hz, front_cam geometry with the lens 0.55 m
ahead, 0.1 s image latency, 3-frame median, bumper occlusion, whole-tag
visibility; `t_plan.py`, 72 checks). Lens lateral error after the stop
align, forward / reverse:

| start offset | state_feedback | tag_line_plan (quintic) | **aim_and_drive** |
|---|---|---|---|
| 10 mm | 9.3 / 9.3 | 0.1 / 0.1 | **0.5 / 0.6** |
| 20 mm | 18.6 / 18.1 | 3.5 / 3.2 | **1.0 / 0.2** |
| 30 mm | 28.1 / 27.6 | 8.0 / 8.6 | 15.0 / 13.6 (wall-capped) |
| 40 mm | 36.9 / 36.5 | 11.7 / 16.1 (body 0.53 m!) | 28.5 / 28.2 (wall-capped) |

- **`tag_line_plan`** (kept, selectable): once the target tag has been
  in view for 3 frames, a quintic path from the base centre's measured
  (offset, slope, curvature) relative to the TARGET tag's line — the
  proper perpendicular distance `ty cos θ − tx sin θ − 0.55 sin θ`,
  which `lateral = tag y` alone misses by ~9 mm at first sight — to
  (0, 0, 0) at the stop is planned once and tracked by odom travel
  (curvature feed-forward led by the delay + heading loop + a deviation
  term with the 6/L² gain), re-planned on a 3 mm deviation, at a
  per-plan speed keeping the peak ω inside 0.04 rad/s. Two earlier
  versions failed instructively: re-planning every tick with a floored
  horizon left the base ON the line but 6° yawed (the turn-back never
  finished), and planning from the current roll-lead-shortened
  horizon while the base was still doing 0.09 m/s spiralled. The
  residual that remains is the launch delay's lost lateral progress —
  and the path swings the body to 0.455–0.53 m from the lane at
  20–40 mm, PAST the plate margin. Not the default.
- **`aim_and_drive`** (DEFAULT; the user's two-straight-lines idea with
  the corner placed at the stop pose itself): at first sight the base
  STOPS, settles, measures the tag at rest (median 5), computes the
  stop pose S = tag − (0.55 + f)·line direction (f = the fore distance
  the column leaves, 0 forward / 0.16 reverse), pivots so its CENTRE
  points at S (`_align_to_tag_continuous` with `target_deg`, delay-led,
  verified at rest, `record=False`), drives that straight line at
  `aim_drive_speed` holding the pivoted heading on the tag (odom if it
  drops out; the launch hold is re-anchored to the aim heading), stops
  on the column corrected for the yaw (`(0.55+f) cos θ − 0.55`), and
  the mandatory stop align turns the yaw back about the base centre —
  with the centre on the line, the lens lands ON the tag. Accuracy is
  the align's; the base error at the stop was ±0.1 mm in every
  uncapped case.

**The plate wall caps the aim (user's constraint: the plate face is
~460 mm from the tag).** A 0.90 × 0.70 body spun about its centre by ψ
reaches `0.45 sin|ψ| + 0.35 cos|ψ|` sideways — the same for either
sign of ψ (CCW swings the rear-right corner out, CW the front-right) —
plus the base's own offset toward the plate; that must stay inside
`aim_wall_dist_m` 0.45 − `aim_wall_margin_m` 0.03. The plate is on the
robot's RIGHT on every lane (`aim_wall_side`), so an offset AWAY from
it buys angle (then `aim_max_deg` 9, the tag-visibility limit, binds);
the other side is bounded by `aim_free_side_dist_m` 0.60. With the
base 20 mm toward the plate the cap is 6.7° — just enough for 20 mm
over the ~0.17 m left after the first-sight stop — and 30 / 40 mm are
capped at 5.3 / 3.9°, leaving 13–15 / 28 mm for the NEXT hop (logged
as `[Aim] pivot limited`). The plant's body reach never exceeded
0.421 m. ⚠️ Only the chassis rectangle is modelled: if the arm at
home, the lift or a cable chain overhangs the 0.70 m width toward the
plate, raise `aim_body_half_width_m`.

**Cost:** ~+11 s per 0.4 m hop in the plant (20.6 vs 9.2 s forward,
22.7 vs 11.7 reverse): the pre-slow to `aim_drive_speed` 0.03 from
`plan_prepare_dist` 0.28 m of odom distance (the base must be slow when
the tag appears — at 0.09 m/s the stop roll ate 5 cm of the 0.2 m
approach), the stop + settle + measure (~1.1 s), the pivot (~2–3 s).
The user accepted slowing the visible approach.

**Open:** offsets over ~20 mm toward the plate cannot be removed in one
hop under the wall constraint — a launch aim from the START tag (known
to ~1 mm at rest after the launch align, over the whole 0.4 m hop, so
4.3° for 30 mm) would take most of it before the target tag appears
and leave the target-tag aim as a trim; not built, proposed. Not driven
on the robot; `mobile_node` restart required. First things to watch:
`[Aim] at rest: … aim +x deg (cap …)`, the pivot converging in one
pass, and the `aligned` record's lateral within a few mm on a hop that
started visibly off the lane.

### 2026-09-08 — Pivot turns finish on the exit tag; align on both pivot tags (dev worktree)

Done in `~/mobile_manipulator_ws_dev` (branch `dev-20260908`, a worktree
made this session so the running `ws_20260902` stack is untouched — merge
back with `git merge --ff-only dev-20260908` once the robot has been
restarted for it). User requirement: while pivoting, once the next tag is
in view and the align error is inside **5°**, slow the turn right down and
stop inside **±0.2°**; and align on **both** the start and the end tag of
every pivot, 0.2° each. **Refined by the user the same day, twice:** since every
arrival already aligns, a MID-ROUTE hop starts on an aligned tag and
needs no extra start align; but the **first hop of every command — move,
pivot, or from the dock 500 alike — must align on its start tag before
setting off**, because the base may have been put on that tag by hand, or
be resuming after a stop, with nothing squared up
(`go_to_next_tag(first_hop=…)`, set by `move_to_tag` for `path[1]`; the
align runs before the edge-type dispatch, so move and pivot share it).
Cost when the previous command already aligned there: one at-rest
measurement, ~0.85 s. Cost when the base is parked OFF its tag: the
command fails on `align_timeout_s` (20 s) and the base never drives —
previously it would have set off blind from `last_known_tag`.

**What it was:** `execute_pivot` was odom-only — raw P (`pivot_gain` 1.5)
on `±90°`, stop at 1° of ODOM error, no use of the tag at all, no delay
compensation — followed by a separate `align_to_tag` on the exit tag. In
the yaw plant with the robot's 0.55 s command delay that loop parks
**+4.2° past** the target before the align starts (loop gain × delay =
0.83 rings). And a pivot that was the FIRST hop of a command (base parked
on 501, `GOTO 505`) never aligned on the start tag: only the previous hop's
tail did that.

**What it is now** (`execute_pivot(direction, exit_tag_id)`, `go_to_next_tag`
pivot branch; new `robot.yaml` keys `pivot_tag_slow_deg: 5.0`,
`pivot_tag_slow_max_angular: 0.05`, `pivot_timeout_s: 30`;
`pivot_threshold_deg` is finally read; the never-read `pivot_slowdown_deg`
is gone): `align_to_tag(start)` (first hop of a command only, move hops too) →
turn → `align_to_tag(exit)`. The turn
has three phases, every one acting on the PREDICTED error (current minus
the rotation still pending from the last `stop_latency_s` of commands —
the same Smith-predictor lead the drive stop and the align use): odom P
until the exit tag is detected; from then the error is the tag's edge
angle, so a tag laid 2–3° off the odom quarter turn is turned TO, not
past; inside 5° predicted the command drops to the align law
(−angle × `align_gain`, cap 0.05 rad/s, floor `align_min_angular_speed`);
the stop fires when the predicted settled angle reaches ~0 (the align's
lead-target rule). The at-rest measurement / settle / re-pass that
certify 0.2° are deliberately NOT duplicated — they are `align_to_tag`,
which runs right after and shares the command history. Exit tag never
seen → stops on the odom band and the align's 20 s timeout fails the hop,
as before. Tag lost inside the slow phase → stop, align takes over.

Verified offline only (`t_pivot.py`, 33 checks, real controller + real
robot.yaml, yaw plant with 0.55 s delay and a front_cam that sees the
exit tag only inside 15° of its heading): CCW/CW pivots reach the 0.25
cap, enter the slow phase, never exceed 0.05 rad/s after it, stop 0.5°
short and settle at +0.04° BEFORE the align, and end at ±0.04° after it
in one pass (8.2 s turn); an exit tag laid +2 / −3° off the odom turn is
turned to (base at 92.0 / 87.0°, error ≤ 0.04°); no exit tag → odom
finish at 89.1°; preempt → False; a first-hop pivot runs
align(501) → pivot(ccw, exit 505) → align(505) and a start tag 1.5° off
is inside 0.2° when the turn begins, a mid-route pivot runs pivot → align
only, and `move_to_tag` over 500→501→505 gives align(500) → pp(501) →
align(501) → pivot → align(505) with no second align on 501; a first hop
that is a forward / reverse move or leaves the dock aligns its start tag
from 1.2° to ≤ 0.2° before Pure Pursuit is called, a mid-route move hop
does not, and a first hop whose start tag is out of view fails without
driving; a plant with an extra 0.3 s lag and
0.15° measurement noise still ends inside 0.1°. A sweep of the tag
visibility window (15 / 10 / 7 / 4°) × placement (0 / ±2°) ended every
case inside 0.1° after the align — the odom phase's predicted-error P has
already slowed the turn by the time the tag appears, so the window size
barely matters. Not driven on the robot; `mobile_node` restart required.
First thing to watch: the `[Pivot] exit tag N in view` line should
appear several degrees before the end, then `slow phase`, then the
align finishing in one pass.

### 2026-09-08 — Keyence standoff loop rewritten: whole sensor range, gap-proportional steps, fresh median readings, outcome recorded

User: "거리센서 현재 측정범위는 너무 짧아, 조정 범위도 금형과 가까우니 이
알고리즘을 최적화해줘". Two things were wrong with the inline loop in
`arm_controller._adjust_distance_to_surface`: it refused anything beyond a
5 mm window and scanned anyway with `execution_message: Success`, and
inside that window it took fixed 1 mm blind steps against a CACHED reading
with a fixed 1 s sleep — a sensor whose value stopped updating would have
been stepped toward the mould ten times. The loop is now
`src/apriltag_nav/keyence_standoff.py` (`StandoffController`, pure logic in
approach-positive perpendicular mm); `ArmController` keeps the two
conversions (`perp = reading × cos(beam)`, `toolZ = approach × −dir`), a
sequence-counted `keyence_cb` so readings are known to be fresh, and the
MoveL. Full before/after table in `docs/keyence_scan_chain.md`.

The design, in one paragraph: engage whenever |err| < 20 mm (the sensor's
own range binds first; the ±99999 sentinel is rejected, never read as a
distance); approach by at most half the MEASURED gap while far and by
≤ 1 mm inside the last ~2 mm (a reading would have to be 2× wrong to reach
the surface, and it is re-measured after every move); retreat up to 3 mm;
25 mm total budget; every decision the median of 5 readings that arrived
after the move settled (0.3 s); a step ≥ 0.3 mm that moves the reading by
< 25 % of itself twice in a row aborts ("reading does not follow the
motion"); the next step is divided by the sensitivity measured from the
last one, clamped [1, 4] — the doc's open issue 1 (spot walk on slopes,
k_eff 7.5 observed) handled in software, the 1 mm clamp kept as backstop;
`keyence.target_distance_mm` is live against `sensor_zero_mm` (both 10, so
nothing moves by default — raising the target scans further from the mould
but the Basler focus / Ra model were set at 10); the result goes into the
CSV row (`Success (standoff ok (err −0.04 mm, 3 steps, travel 2.0 mm))` /
`Success (standoff NOT corrected: …)`), and `require_converged: true` fails
the point instead. `seek_enabled` (step toward the side an out-of-range
sentinel names) exists but is OFF: it approaches on no measurement, and the
sentinel's sign has not been confirmed on this sensor.

Verified offline only: `t_standoff.py` (44 checks — surface plant with the
42.6° spot walk, signed sentinel, dropouts, frozen / silent sensor; flat
4–20 mm starts converge in 2–5 steps where the old loop refused 4 / 16 / 20;
slopes k_eff up to 3.2 converge in ≤ 3 steps with ≤ 1 reversal where the
old and non-adaptive loops hit the budget; 20 % dropout converges; frozen
value stops after 2 mm vs the old loop's 10 mm walk) and `t_arm_wiring.py`
(20 checks on the real `ArmController` methods against a fake Fairino with
the real tool-Z geometry, incl. `execution_message` / `require_converged`).
One bug found by the plant: the sentinel was being projected by cos and
slipped under the invalid threshold — it now passes unprojected. Config:
`robot.yaml keyence:` (new keys documented inline, `activate_threshold`
5 → 20, `max_steps` 10 → 15), launch `keyence_max_steps` 15,
`tools/test_scan_chain.py` defaults. Not run on the robot; `arm_node`
restart required.

### 2026-09-08 — Calibration ref-tag pairing changed to explicit ranges; eight view seeds recomputed

User instruction: pair the drive (WORK) tags with the cross (ref) tags
as **100-104→0, 105-107→1, 108-112→2, 113-117→3, 118-120→4,
121-125→5** and **126-130→0, 131-133→1, 134-137→2, 138-142→3,
143-145→4, 146-150→5**, and recompute the initial
`arm_view_tcp_mm_deg` to match. The generator's old rule was
"nearest cross tag by TAG y", which split each column 4/3/6; the new
split is 5/3/5 and — checked against every tag — equals "nearest
cross tag to the robot's STOP pose" (tag y − 0.55 m along the
heading). It is kept as an explicit table (`REF_RANGES` in
`generate_calibration_artifacts.py`), not as that rule, because the
table is what the user specified; the generator refuses a ref that is
not on the column facing the tag's corridor.

**What changed, per plate: exactly four entries** (plate 1: 104 1→0,
107 2→1, 117 4→3, 120 5→4; plate 2: 130 1→0, 133 2→1, 142 4→3,
145 5→4). Their design view TCPs moved from x ≈ −668 to x ≈ +377 mm in
the arm frame (the ref tag is now 0.4 m on the other side of the
stop) and their flange reach dropped 0.93 → 0.74 m. Every other
entry's design TCP is byte-identical to before (the pre-change
generator was run first and diffed). `docs/all_tags_position.csv`
regenerated — it was open in LibreOffice at the time (lock file
present), so reload it there.

**Seeds.** Plate 1 keeps its 22 session-measured seeds from
`20260904_153218`; the four re-paired entries get design + the
same-ref session median (−20…−28 mm x, −6…−24 mm y), since a pose
converged over the OLD ref is 1.2 m off for the new one.
`update_plan_seeds_from_session.py` now detects a session ref that
differs from the plan's and falls back to that estimate instead of
copying the stale measurement — needed for exactly this, and for any
future re-pairing. Plate 2 stays on design seeds (as committed);
it has since been run four times (09-04 16:22: 25/25 ok; 09-08 11:37:
24/24; 12:00: 24/24; 12:55: 12 ok of 38 attempts) and any of the
good sessions can be fed to that script.

Verified offline only: both plans load through the real
`load_calibration_plan` with every entry matching `REF_RANGES`; the
generator's diff against its own pre-change output is the header +
the eight entries; the seed script's dry run reproduces all 22
unchanged plate-1 seeds exactly. Not run on the robot. The
calibration nodes read the plan per session, so no restart is needed.

### 2026-09-04 — Forward stop column back to the crosshair (user request); what still covers the forward launch

`center_x_stop_offset` 300 → **0** on the user's instruction ("전진 태그
목표점을 원래 위치로"): the far forward column cost ~12 cm of stop
position on every forward tag, and the crosshair was preferred. Kept
from the same afternoon: reverse +400, the 500-series crosshair rule in
both directions, the `/robot_pose` fore-aft term (0 at the crosshair, so
no change for forward), the whole-hop heading hold, and the signed
reverse curvature term.

Forward vs reverse after the revert, stated plainly: at a
forward-after-forward launch the start tag leaves the frame after ~3 cm
(bumper occlusion), so that launch is held on odom yaw anchored to the
tag's last frames plus the backlash feed-forward; a forward-after-reverse
launch (start tag at +400) and every reverse launch keep the tag ~19 cm.
A reverse hop INTO a 500-series tag (112→505, 137→507) sees its target
only from ~3 cm out — the pre-2026-09-02 reverse geometry — and relies
on the odom creep zone (0.015 m/s inside the last 15 cm) and the roll
lead; today's records show those arrivals within a few px. Nothing else
in the loop is direction-dependent beyond the items listed in the
difference table above. All suites pass with the revert (60 / 18 / 7 /
11 / 15 / 10 / 10 / 7). Not driven; `mobile_node` restart required.

### 2026-09-04 — Forward made like reverse: far forward column, /robot_pose fore-aft term, whole-hop heading hold, reverse-prediction sign

User: "후진은 너무 잘되니 후진을 기준으로 전진을 수정". Forward-only
look at the records first: the forward arrival yaw is bad on the ZONE A
LANE (−0.6 … −1.7°, map prediction off there so the blind part drove with
omega = 0) and fine in zone C (−0.15 … −0.28°); reverse was fine on both.
What reverse has that forward lacked: the tag is left 16 cm AHEAD of the
lens at the stop, so the next launch keeps the start tag in view ~19 cm
and the heading is held on the TAG (a wheel-play twist the encoders never
see is still corrected). Forward stopped on the crosshair and lost its
start tag after 3 cm.

Changes:
- **`center_x_stop_offset` 0 → 300 px** (tag ~12 cm ahead of the lens at
  a forward stop): start tag visible ~15 cm at the next launch; the
  target, which appears ~25 cm out going forward, still gives ~13 cm of
  tag-visible approach (400 would leave 9). Sim: start-tag-in-view ticks
  at launch 50 → 96 (2.5 → 4.7 s at 0.02 m/s). ⚠️ Every forward stop is
  now ~12 cm short of the design pose.
- **`stop_offset_skip_tag_ranges: [[500, 599]]`** replaces the reverse-
  only key (alias kept): those targets stop on the CROSSHAIR (offset 0) in
  both directions — a pivot needs the base on the designed pose.
- **`/robot_pose` fore-aft term** (`robot_pose_use_fore_aft`): the pose
  used to assume the lens over the tag (map tag + lateral + camera
  offset); it now subtracts the tag's fore-aft distance along the zone
  heading, so the reported pose and the pose-mode arm transform are right
  whatever column the base stopped on. Verified per zone (A/B/C) against
  hand-computed world positions.
- **Heading hold over the whole hop** (`launch_yaw_hold_dist` 0.15 → 10):
  odom yaw against the tag-anchored reference for the entire blind part —
  not map prediction, so it stays on for the lane. Sim gain was modest
  (0.22 → 0.15° with a 2 % slow wheel) but it is what covers the lane.
- **Reverse predictive-centering sign** (`_predictive_centering_omega`):
  `|speed| × curvature` → `speed × curvature`. Same reason as the base-
  referenced lateral: the base's lateral motion is v·sin(yaw), so the
  curvature term flips in reverse. Sim (zone C reverse, map prediction
  on, a 3 % slow left wheel as the in-hop disturbance — prediction reads a
  START heading error as a rotated corridor, since it assumes the hop
  starts squared up, which the mandatory align guarantees): lateral when
  the tag appears 2.0 → 1.0 mm, yaw −0.61 → −0.16°; modest, because the
  heading term dominates the blind part either way.
- **Median needs three samples**: with two, numpy's median is their mean
  and a spike leaks in at half strength — `_tag_view` now filters only
  from three frames on and uses the newest frame unfiltered below that.

Suites after all of it: 60 / 18 / 7 / 11 / 15 / 10 / 7 / 10 pass
(`t_fwd.py` is the new one). Not driven; `mobile_node` + `robot_camera_node` restart required. First
forward run to watch: the tag must rest on the FWD line (+300) with the
whole tag in frame, and the following launch should show
`[LaunchYawHold] … (tag N)` for several seconds instead of `(odom)`.

### 2026-09-04 — Are the FWD / REV lines where the base actually stops? Yes — and the overlay's crosshair was 2 px off them

User asked to verify that the forward and reverse target tag positions
are exactly on the lines drawn on the screen. Two things checked.

**Reference.** `mobile_controller` stops on `camera_params[2] + offset`
and `robot_camera_node` draws the FWD / REV lines at `camera_params[2] +
offset` — both `K[2]` from the same CameraInfo (638.2 on front_cam). The
crosshair and the per-tag `off` text, however, were measured from the
frame's geometric centre `w//2` = 640, so a tag resting exactly on the
REV line read `off +398` and the FWD line sat 2 px beside the crosshair.
Fixed: the crosshair, the columns and every printed offset now use the
calibrated principal point (fallback w/2, h/2 without CameraInfo).

**Result, from today's 130 arrival→aligned pairs** (at-rest tag column
minus the target line): forward n=65 mean **+1.35 px** (sd 4.9, −7.8 …
+12.8), reverse n=65 mean **−3.0 px** (sd 3.8, −11.0 … +3.8); at the stop
TRIGGER the tag is +13.0 / −13.2 px from the line, i.e. the 0.55 s roll
lead is right to a pixel in both directions. 1 px ≈ 0.4 mm at this
depth, so the base rests within ~0.5 mm (mean) / 2 mm (sd) of the line
either way. `robot_camera_node` restart required for the overlay change.

### 2026-09-04 — Four deterministic accuracy changes: state-feedback steering, predicted heading, median measurement, camera-latency compensation

User asked for accuracy improvements that are LOGIC, not learning
("학습이 아닌 로직이 필요"), and chose these four. All in
`mobile_controller`, all constants in `robot.yaml`, all off-switchable.

1. **`steer_mode: state_feedback`** (was Pure Pursuit) while the target
   tag is in view: `omega = −(v·sf_lateral_gain)·e_y − sf_heading_gain·e_θ`
   with `e_y` the base-referenced lateral (m) and `e_θ` the PREDICTED
   heading (below). The lateral gain scales with speed as PP did
   (32 = 2/0.25², the PP curvature at L 0.25); the heading gain (0.6)
   does not, which is what gives it authority in the 0.01–0.03 m/s last
   25 cm where PP's `omega ∝ speed` had none. Closed loop `wn = v√k_y`,
   `ζ = k_θ/(2v√k_y)` = 1.6 at 0.033 m/s, 0.53 at 0.1 m/s; below
   `sf_min_speed_for_gain` (0.02) the lateral gain is held. The
   prediction-segment heading term and the launch hold's target-tag term
   are folded into it (no double-adding). `'pure_pursuit'` restores the
   old law.
2. **Predicted heading everywhere.** The pending rotation from the last
   `stop_latency_s` of angular commands (`_pending_yaw_rad`, already used
   by the align stop) is now added to the heading error the steering law
   and the launch hold act on — the Smith-predictor idea with the known
   0.55 s delay, no learning.
3. **Median measurement.** `detections_callback` keeps a per-tag history;
   `_tag_view()` returns the median of the last `stop_measure_frames` (3)
   detections (column, row, pose, edge angle). Every steering/stop/final-
   approach/record decision uses that view, so one bad frame cannot fire
   the stop (sim: a spike frame claiming the tag at the column fired the
   old single-frame stop 8 cm early; the median ignored it).
4. **Camera-latency compensation.** Detections carry the IMAGE stamp;
   `_tag_view` extrapolates the fore-aft column by the executed speed
   (odom twist, else the command from `stop_latency_s` ago) × frame age
   (capped 0.3 s). Arrival records carry `tag_age_s` / `latency_comp_px` /
   `steer_mode`. Sim with a 0.11 s image delay: forward stop 1.7 → 0.5 mm
   from the tag, reverse column error unchanged-or-better within 2 mm.

**Reverse steering sign, found while listing forward/reverse differences
for the user (same day):** the base-referenced lateral error obeys
`e_y' = v·sin θ`, so its steering sign must flip with the direction of
travel — reversing, the base centre moves the other way for the same yaw,
like a car backing up. The lens-referenced error never needed this (the
lens sits 0.55 m ahead of the pivot and that lever moved it toward the
tag under a CW turn whichever way the base rolled), so switching to
`pp_lateral_reference: base` earlier today silently made the REVERSE
lateral loop positive feedback; the short reverse test hops passed on the
heading term alone. Fixed: the lateral term is multiplied by
`move_dir_sign` when base-referenced (both steering laws). New check: a
1.0 m reverse hop from 30 mm off now shrinks the base lateral while the
tag is in view (30 → 26 mm state feedback / 24.5 mm PP, yaw +1.9 / +4.2°
toward the line, no divergence).

`t_sf.py` (10 checks, wheel-level plant, 0.55 s command delay, detections
through `_store_detections` with image latency): state feedback vs PP
from 30 / 60 mm off — arrival yaw −2.38 → −1.66° / −4.73 → −3.32°, zero
omega sign flips, base lateral unchanged; latency and median cases as
above. All earlier suites pass (59 / 18 / 7 / 11 / 15). The base-lateral
numbers restate the geometric limit: a 0.5 m hop with the tag visible for
its last 25 cm cannot centre the BASE from 30 mm off — the arrival yaw is
the base pointing at the line; the following align swings the lens onto
the tag (pp_lateral_reference: base) and the next hop continues the
convergence. Not driven on the robot; `mobile_node` restart required.

### 2026-09-04 — "여전히 얼라인이 과하게 돌아": it was the lens swinging, not the yaw — PP now steers the base centre

First robot run with the lead+settle align (`GOTO 120` 14:33:56, 16 s
after the `mobile_node` restart, and `GOTO 400` 14:30 — the records now
carry `align_passes`, i.e. the 'aligned' record is taken AT REST after
the settle). **Every aligned yaw at rest is inside the band**: −0.08,
+0.11, −0.05, +0.17, −0.02, −0.06, +0.07, −0.17 … (band 0.2°), 1–3
passes. Fore-aft at rest is within ±1 mm on both columns (forward −0.5 …
+2.6 px around cx; reverse 399.4 … 401.5 around +400), and the arrival
records sit 13 px SHORT of the column with a 13.6–14.1 px lead, i.e. the
0.55 s roll model is right to a pixel at 0.010 m/s.

**What the operator sees as over-rotation is the LATERAL jump during the
align.** Tag 404: arrival lateral +3.6 px (1.5 mm), yaw −1.66° → after
the align +48 px (19.5 mm). The align rotates about the base centre and
the lens is 0.55 m ahead of it: 0.55 × sin(1.83°) = 17.6 mm, matches.
Forward runs then show a steady −11 … −20 mm lateral bias at rest, and
the next hop arrives yawed −0.9 … −1.7° because Pure Pursuit homes the
LENS back onto the line at an angle — a cycle: align swings the lens
out, PP steers it back in at a yaw, align swings it out again.

**Fix: `pp_lateral_reference: base`** — PP steers on
`lateral − camera_offset × sin(yaw_error)`, the BASE CENTRE's offset
from the line, so the align lands the lens ON the tag instead of off it.
Sign verified against the records (predicted 17.4 mm for tag 404,
measured 19.5 incl. the residual). `'lens'` restores the old behaviour.

**Pulse align (`align_mode: pulse`) added but NOT made the default.**
Written as the delay-immune alternative (measure at rest → one burst of
a computed duration → stop, settle, re-measure → learn achieved/commanded
into a gain kept across hops); on a plant with 0.55 s delay + stiction +
1.6× release lurch + 1.3× gain it converges from ±3 / 1 / −0.5 / 0.35 / 8°
to within the band (`t_pulse.py`, 15 checks) but takes 4.5–9.5 s and 1–2
reversals where the continuous loop takes 1.6–5 s and lands ±0.05 … 0.2°
on the same plant — and the continuous loop is the one the robot has now
shown working. Switch to pulse only if continuous misbehaves on the
robot. `launch_peak_yaw_err_deg` is now SIGNED so the backlash
feed-forward can be fitted from it.

Suites: 59 / 18 / 7 / 11 / 15 all pass; `mobile_node` restart required.

### 2026-09-04 — Backlash feed-forward at launch; Pure Pursuit steering limits raised

Two more user requests the same afternoon. (1) "얼라인 때 후진을 줬던 바퀴가
출발 때 약간 반응이 늦다": the in-place align turns one wheel backward; on
the next hop that wheel has to REVERSE and sits idle through its gear play
while the other wheel already rolls, so the base twists toward the late
wheel by play / track (0.7° per 8 mm on the 0.65 m track) — and a
motor-side encoder never sees it, so the odom-referenced launch hold from
the entry below would faithfully hold the TWISTED heading. (2) "퓨어퍼슛
조향 각도 제한이 너무 작다".

**Backlash feed-forward.** `align_to_tag` records the sign of its last
non-zero turn (`_last_align_turn_sign`). The twist direction follows from
it: a CCW align backs the left wheel → forward hop: left is late → CCW
twist; reverse hop: the RIGHT wheel is the one reversing → v_L = −v,
v_R = 0 → also CCW. So for the first `launch_backlash_ff_s` (0.3 s) of
every hop `omega += −sign × launch_backlash_ff_omega` (0.02 rad/s, ≈0.34°
of pre-rotation), on top of the hold. The magnitude is a guess to be
fitted on the robot: every arrival record now carries
`launch_peak_yaw_err_deg` (tag-measured while the start tag is in view),
`launch_ff_applied` and `last_align_turn_sign` — raise the value while the
peak keeps the align's sign, lower it once it flips. 0 disables.

**Launch hold re-anchored to the START tag.** The first version used
`heading_error_deg`, which exists only while the TARGET tag is in view —
never at launch. It now reads the start tag's edge angle (in view for the
first ~3 cm forward, longer in reverse), else the target's, else odom yaw
against a reference that is re-anchored to the tag whenever one is seen
(`launch_theta_ref = theta − edge_angle`), so an encoder-invisible twist
is still held after the tag has left the frame.

**PP limits.** `move_max_angular_speed` 0.12 → **0.25** rad/s (= the pivot
cap) and `look_ahead_base` 0.4 → **0.25** m: PP's path curvature is
2·lateral/L², so with L = 0.4 a lateral error decayed over ~0.4 m — a whole
corridor hop — and the tag was never centred before the stop; 0.25 gives
2.6× the steering. ⚠️ What the sim made explicit: the lens is 0.55 m ahead
of the base centre, so PP homes the LENS onto the tag; a 30 mm base
offset is reported as 9 mm camera lateral once the base has yawed −2°,
and the following in-place align swings the lens 0.55 × sin(2°) ≈ 19 mm
back out. Centring the BASE on the lane within one 0.4 m hop is not
something this PP can do; the corridor prediction (blind heading) is what
does that over several hops.

Verified offline (`t_backlash.py`, 11 checks; wheel-level plant with
8 mm of play per wheel taken up on every direction reversal, MOTOR-side
odom, 0.55 s delay; align from ∓1.5° then a hop): uncorrected mean |yaw|
0.38° / 4.5 mm lateral → hold only 0.08° / 0.3 mm → hold + ff 0.05° /
0.2 mm (4 % of uncorrected); reverse hop after a CW align 0.63° → 0.03°;
with NO play the ff is a 0.34° perturbation the hold removes (0.05°,
0.2 mm); arrival record carries the three new fields. PP: from 30 / 60 mm
off, camera lateral at the stop 15.7 → 9.0 / 31.5 → 18.4 mm (old → new
L), max |omega| 0.043 — the new cap never binds in the sim — and zero
omega sign flips with the 0.55 s delay. Earlier suites (59 / 18 / 7)
unchanged. Not driven; `mobile_node` restart required.

### 2026-09-04 — Launch yaw hold: the base no longer sets off twisted after an align

User: "제자리 얼라인 후에 직진을 하면 한쪽 바퀴가 살짝 늦게 도는지 약간 요가
기울면서 출발하는 경향". Cause on the software side: Pure Pursuit's
steering is `omega = |speed| × curvature × gain` — proportional to the
speed (≈0 at launch) and to the LATERAL offset only, so a yaw twist from a
late wheel is invisible to it until it has grown into a lateral error;
the heading term that exists is only added on hops with a calibrated
prediction segment. New `launch_yaw_hold_dist: 0.15` / `launch_yaw_gain:
1.0` (`robot:`): inside the first 15 cm of every move hop an extra
`omega = −gain × yaw_error` holds the heading the align established.
`yaw_error` is the tag's edge angle while the tag is in view (only ~3 cm
going forward — the bumper hides the floor behind the lens — longer in
reverse) and otherwise odom yaw minus the yaw at the hop start; a late
wheel IS a wheel-speed differential, so the encoders see it. Same sign
convention as `align_to_tag` (edge angle moves with the base yaw,
positive removed by turning CW), direction-independent, not double-added
when the calibrated-segment branch already used the tag heading. 0
disables.

Verified offline (`t_launch.py`, 7 checks: 2-D unicycle with a 0.55 s
command delay and a late LEFT wheel modelled as +0.05 rad/s for the first
0.6 s of motion, tag corners rotate with yaw). Forward 0.5 m hop: mean
|yaw| while moving 1.21° → **0.18°**, lateral excursion 13.0 → **0.7 mm**;
reverse 1.67° → 0.14°, 19.7 → 0.7 mm; a clean launch is untouched
(0.000°); the following align still ends at −0.02°. The PEAK twist
(1.7°) is unchanged in the sim — the correction cannot arrive before the
0.55 s delay, so what the hold buys is a fast recovery instead of a
lateral drift. Not driven on the robot; `mobile_node` restart required.
If the twist is pure slip (no encoder differential) the odom term cannot
see it and only the tag term (3 cm forward) helps — then look at the
wheel drives, not here.

### 2026-09-04 — Skid-steer align overshoot: lead the stop, settle, re-measure; deceleration zones lengthened

User: "로봇이 스키드 조향인 점을 인지하고, 제자리 얼라인할 때 오차보다 좀
더 회전해서 얼라인이 틀어지는 문제를 해결, 감속 모드를 살짝 더 길게".

**The mechanism, from the numbers already in this file.** The base
executes commands ~`stop_latency_s` (0.55 s) late — measured on the
linear stop on 2026-09-02 — and an in-place turn is no different. The
align loop commanded at least the 0.01 rad/s floor until the raw edge
angle read inside the 0.2° band and then sent zero; the base then turned
another 0.55 × 0.01 rad = **0.32°**, i.e. it always parked OUTSIDE the band
on the far side (the "twist"). Skid steer adds a lurch on release, so the
real figure is larger than the pure-delay 0.32°.

**Fix, in `align_to_tag`:** (1) the command history now carries the signed
angular command too, and `_pending_yaw_rad()` integrates the last
`stop_latency_s` of it — the angular twin of `_pending_roll_m()`; the stop
fires when the PREDICTED settled angle (`angle + pending`; the edge angle
moves WITH the base yaw, since the loop removes a positive angle by
turning CW) is within `align_lead_target_ratio` × band (0.05°) or would
cross zero — aiming at zero, not the band edge, which a first version did
and parked at +0.17°. (2) zero command for `stop_latency_s +
align_settle_s` (0.85 s), then the tag is re-read AT REST. (3) inside the
band → done; otherwise another pass, up to `align_max_passes` (3), after
which the residual is accepted with a warning and
`align_residual_accepted: true` in the record. Only a measurement taken
at rest is ever reported as "aligned"; records carry `align_passes`.

Two bugs found by the yaw plant on the way: the predicted-angle sign was
first written as `angle − pending` (diverged to −85°); and
`_integrate_pending` extended the FIRST in-window sample back to the
window start, which after ONE tick of command claimed a whole
`rate × latency` was pending and stopped the align after a single 0.057°
step (−0.35° → −0.29 → −0.24, three passes to reach −0.18). It now holds
the last sample published BEFORE the window, exact for the delay model.
`_pending_roll_m` shares the integrator; at the drive's stop the history
is dense so its value is unchanged (59-check drive suite identical).

**Deceleration zones "살짝 더 길게":** `final_approach_dist` 0.06 → **0.08**
m and `blind_approach_dist` 0.12 → **0.15** m. Config only; the solved
deceleration and the creep zone follow.

Verified offline (`t_align.py`, 18 checks, yaw plant with a 0.55 s
transport delay, edge angle = +yaw): old rule ends at −0.32° from a +3°
start; new rule ends at +0.05 / −0.05 / +0.06 / −0.006 / +0.05 / +0.16°
from +3 / −3 / +0.4 / −0.35 / +1.5 / +8°, one pass each, 3–4.5 s; a plant
with 0.9 s lag against the 0.55 s config recovers by extra passes;
already-square = one settle; the integrator reads 0.055 rad for 0.55 s of
0.1 rad/s and leaves the linear roll at 0. The large-error residual
(+0.16° from 8°) is the 10 Hz tick — the executed angle moves up to 0.6°
per tick at the 0.12 rad/s clamp. Not driven on the robot; `mobile_node`
restart required. If the robot still twists, `stop_latency_s` for the
turn may differ from the drive's — fit it from `(arrival yaw − aligned
yaw)` in the nav_log records.

### 2026-09-04 — Calibration + nav_log yaml wiped; one record file per command, created lazily

User request: delete every yaml the calibration code produced, delete
every tag-accuracy yaml from driving, and make the tag-accuracy record
"one yaml per TASK / GOTO command".

**Deleted (user's explicit instruction), yaml only:** 940 files under
`~/.ros/path_tag_locator` — 428 `calibrate/<ts>/entries/*.yaml`, 11
`session.yaml`, 9 session `map_world.yaml` copies, the 9 top-level
`map_world_20260902_*.yaml` + the `_handeye_corrected` one, and every
`locate/<ts>/run_*/request|result.yaml`; plus all 32
`~/.ros/apriltag_nav/nav_log/<day>/*.yaml` (2026-09-02 and 09-04).
Empty directories were removed. **Kept:** the non-yaml artifacts — 56
`result.npz`, 112 png, 13 csv (incl. `compare_map_world_…csv`) — the
npz are what the 2026-09-02 offline hand-eye re-fit was computed from.
Consequence: `predictive_centering.map_world_path: "latest"` now finds
nothing, so blind steering uses map.yaml everywhere and records carry
`calibrated_reference: null` — the documented fallback, not an error.

**Why one command sometimes had two files:** the design was already one
file per command, but `begin_nav_session` wrote the file immediately, so
a GOTO that failed before any arrival (tag not visible, preempt) and was
re-issued left a `records: []` twin — 7 of the 32 deleted files were
such empties (e.g. `110106_GOTO_503` empty, `110110_GOTO_503` real).
Now the file is **created at the first record**; a command that never
arrives anywhere leaves nothing (the failure is in rosout). The
same-second suffix rule is re-checked at that first write. Second fix,
in `mobile_node`: the task's FIRST `goto_tag` could race the latched
`/task_state` and open a second `goto_<n>` file next to the TASK file —
`_cmd_session_pending` (set by `/task_command`, cleared by the first
goto) now keeps that hop in the command's file regardless of timing.

Verified offline (12 checks: no file without an arrival; re-issued
GOTO → one file; TASK with two tag arrivals → one file; next command →
new file, old untouched; bare record → `goto_<tag>`; same-second twins
suffixed; mobile_node: TASK → first goto before `/task_state` reuses the
session, second hop too, a bare goto outside a task gets its own) plus
the 59-check suite. Not run on the robot; `mobile_node` restart needed.

### 2026-09-04 — Stop-then-align on EVERY hop; the 400-lane skip is retired

User rule: "모든 주행에서 태그 검출해서 퓨어퍼슛한 다음 정지 얼라인 —
어떤 상황이든 정지해서 얼라인은 필수". The drive rule (tag visible →
Pure Pursuit, stop line → stop → align) already held everywhere except
the one exception added on 2026-09-02, `align_skip_tag_ranges: [[400,
499]]`, which let 400→401 … 410→411 lane hops end without the in-place
align. That exception is gone: `_align_after_arrival` no longer consults
any tag range, the key was removed from robot.yaml, and a stale copy is
ignored with a `logwarn` so a config merge cannot quietly bring the skip
back. Pivots already aligned to their exit tag; manual `drive_distance` /
`pivot_angle` have no target tag and are unchanged. The only skip left is
a temporarily-missing VIRTUAL tag (`temporary_missing_tags.enabled:
false`) — nothing physical to align to. Cost: each 400-lane hop now
spends the align time (typically < 1 s when the tag is already square;
up to `align_timeout_s` 20 s and a FAILED hop if the tag is out of view
at rest — that is the intended behaviour, not a regression).

Verified offline (7 checks on the real controller against the corridor
plant: 400→401→402→502 all forward, `align_to_tag` called after each of
the three hops; a stale key produces the warning and still aligns;
robot.yaml no longer carries the key) plus the earlier 59-check suite
unchanged. Not driven on the robot; `mobile_node` restart required.

### 2026-09-04 — Five pivot tags re-laid and tape-measured; 정반 spacing 3.89 → 3.90 m

The user physically moved **503, 505, 506, 507, 508** and measured each
against its neighbouring WORK tag, which stayed put; every other tag keeps
its design value (user instruction, same message: "나머지 태그들은 기존값
유지"). A first pass that also shifted the D/E lanes and 504 by the new
plate spacing was backed out for that reason. Only the measured axis of
each moved tag changed:

| pair | measured | map before | map after |
|---|---|---|---|
| 111 ↔ 505 | 1.03 | 1.02 | 505 y 3.57 → **3.58** |
| 113 ↔ 506 | 0.64 | 0.62 | 506 y 2.47 → **2.49** |
| 137 ↔ 507 | 1.04 | 1.02 | 507 y 3.57 → **3.59** |
| 138 ↔ 508 | 0.62 | 0.62 | 508 unchanged (5.6, 2.47) |
| 502 ↔ 503 | 0.50 | 0.47 | 503 x 2.73 → **2.76** |

Consequences worth knowing: 503's stop pose is now x 2.21, 3 cm east of
the zone D lane (2.18) where 507 still sits, so the 503→507 pivot lands
front_cam ~3 cm beside 507 — align + Pure Pursuit absorb it. The exit tags
are no longer exactly 0.55 from the y 3.02 lane (505/507 0.56/0.57 north,
506 0.53 south), which only changes those hops' odom distances
(505→112 0.63, 506→113 0.64, 507→137 1.04 — from the real `MapManager`).
All four `500→…` routes and the return route are unchanged; 72 tags /
142 edges. The old map is in git, not in a .bak.

**Plate spacing 3.90 m** (user: "정반 사이 거리 390 cm", design 3.89):
applied to `reference_tags_plate2.yaml` (cross tags ON 정반 2: x 3.290 →
**3.300**, 4.490 → **4.500**), the generator comment, `robot_sim`'s plate
outline, `CALIBRATION_GUIDE_kr.md`, `find_cross_tags.py`, HANDOVER — and
NOT to the D/E floor lanes, per the instruction above. Plans regenerated:
`calibration_plan_plate2.yaml` seeds moved +10 mm along the arm-frame
axis that maps to world x (e.g. tag 126 `931.3 → 941.3`), plate 1 plan
byte-identical, `docs/all_tags_position.csv` regenerated, reach still
0/51 over 1.40 m. ⚠️ A plate-2 calibration session run before today
against the +3.890 refs is offset 10 mm in x from one run now — the two
archived 09-02 map_world files carry the old assumption.

Verified offline only: map parses, the five measured distances reproduce
exactly, routes/hop lengths via the real `MapManager`. Not driven.
`mobile_node` must be restarted to load the new map; the calibration
nodes reload the refs per session.

### 2026-09-04 — Reverse hops stop the tag at the far right of the frame; both columns drawn on the overlay

User request: "후진할 때 전진과 동일한 태그 목표 위치를 쓰고 있는데, 후진할
때만 화면 기준으로 최대한 오른쪽에 목표를 두고, 그 목표도 화면에서 보이게".
New `robot.center_x_stop_offset_reverse: 400.0` (px from the calibrated
cx, same sign convention as `center_x_stop_offset`; absent/null = the old
shared column). `execute_pure_pursuit` picks the column by
`move_dir_sign`; forward is untouched. **How far right is possible:** with
cx ≈ 638 of 1280 px, fx ≈ 750 and the 90 mm tag ~222 px wide at the 0.30 m
camera height, the tag centre must stay under ~1100 px for the detector
(whole tag + a bit of quiet zone in frame), i.e. offset ≤ ~460; 400 leaves
~50 mm of margin for the roll and lateral error. The numbers come from
today's nav_log (13.1 px ↔ 5.31 mm at depth 0.3036 m).

**Overlay:** `draw_overlay` takes `stop_columns={'FWD': px, 'REV': px}`
(front_cam only, read in the node from the same two `robot:` keys) and
draws a dashed vertical line per column at the CALIBRATED cx + offset —
the column the stop test actually uses, ~2 px off the geometric crosshair
— with a `FWD stop +0px` / `REV stop +400px` label at the bottom; equal
offsets collapse to one line with both labels. robot_ui shows it
unchanged.

**The consequence that needed code, found by the offline plant:** a
reverse arrival now rests the lens ~162 mm short of the tag, `/robot_pose`
carries no fore-aft term, and the map edge is stop-pose-to-stop-pose — so
the NEXT hop's odom distance is wrong by that much (longer forward, and
0.16 m longer for a reverse hop that follows a forward arrival). The first
sim run of a bare 0.40 m reverse hop ended on the "near target, tag not
visible → min_speed" crawl × the 1/3 tag-visible factor = 0.0067 m/s and
hit the 30 s watchdog. `go_to_next_tag` now plans
`edge + dir × (fore_prev − fore_target)`: `fore_prev` = the tag's fore-aft
distance from the lens at the start (read LIVE from the start tag when in
view — with the far-right column it is — else remembered from the last
arrival/align; manual moves reset it), `fore_target` = offset × depth / fx
for the hop's direction. Arrival records gained `direction`,
`stop_offset_px`, `stop_target_x_px`, `tag_offset_from_target_px`, so the
roll fit still works against the right column.

**Same day, user rule: "500번대 태그가 목표 명령이면 기존 목표 태그 위치에
멈춰야 해".** New `center_x_stop_offset_reverse_skip_tag_ranges: [[500,
599]]` — matched on the TARGET tag only; a reverse hop into 500–599 uses
the forward column (`_stop_offset_px`), and the odom-distance correction
follows (a reverse arrival on 112 then reverse to 505 plans 0.40 − 0.162 =
0.238 m, since the lens starts 162 mm short of 112 and ends ON 505). The
overlay's REV label carries the note `(tags 500-599: FWD)`. Reason the
500-series are the exception: they are the dock and the corridor pivot
tags, and a pivot about the base centre needs the base on the designed
pose, not 162 mm short of it.

Verified offline only (`t_overlay.py` 7 checks, incl. the label note;
`t_rev_stop.py` 59 checks — the 39 below plus: reverse→505/500 = forward
column, reverse→111 = reverse column, forward→505 unchanged, a 6-hop
505→112→111→112→505→112→505 corridor landing every hop on its column with
the planned distances 0.56/0.40/0.56/0.40/0.56/0.56 m, reverse 112→505
landing +0.9 px of cx, malformed range entries skipped. Original suite: both lines at the right
columns, none on side_cam, shared-column collapse, off-frame safe;
`t_rev_stop.py` 39 checks on the real controller against a 0.55 s
transport-delay plant + simulated front_cam with bumper occlusion and
frame-edge visibility: forward lands +1 px of cx, reverse +399 px with the
whole tag in frame and the lens 161.6 mm short, key absent/null → shared
column, forward ignores the reverse key, +450 still in frame, 1.0 m reverse
same column; a 5-hop corridor through `go_to_next_tag` F,F,R,R,F lands
every hop on its column with planned odom distances 0.40 / 0.40 / 0.56 /
0.40 / 0.56 m and every hop + align under 16 s). Not driven on the robot;
`mobile_node` and `robot_camera_node` must be restarted. ⚠️ The test's
first run wrote its sim records into the REAL `~/.ros/apriltag_nav/nav_log`
(the dir key sits under `robot.predictive_centering`, not `robot:`); the
twelve 13:38–13:41 `GOTO_401`/`DIAG` files were deleted, the 10:15–11:37
robot records are untouched.

### 2026-09-04 — Plate-1 calibration: 6/26 entries lost to the initial-move clamp; fixed three ways

First live plate-1 session with the 2026-09-03 workflow
(`calibrate/20260904_153218`): 20 ok, 6 failed — 100, 111, 112, 113,
124, 125, every one with "tag A not detected at iteration 1", and every
retry failing identically. The 20 successes calibrated to within
5–15 mm of design (x ≈ −1.71 / +1.70, 0.4 m pitch, z −0.05…−0.09).

**Root cause, from the map_calibrator log: the initial view move was
clamped to one 0.80 m step.** Since 2026-09-03 the arm HOMES before
every base move, so every seed is approached from the home TCP
(≈ (−160, 700, 774) mm — z alone is 0.71 m above the seeds' z 60).
Seeds within 0.8 m of home (x −668…+377) were reached or nearly so;
the six at x −1052 / +734 / +1109 are 1.15–1.17 m away, the single
clamped step left the arm 0.35 m short with the camera still 0.28 m
high, and the tag was out of view. The seeds themselves are fine — the
successes converged 8–43 mm from them. A side effect worth knowing:
entries whose step was clamped but still saw the tag (101, 104, 107,
110, …) aligned at **z ≈ 135 mm instead of 60**, because the align
loop keeps whatever depth the initial move ended at
(`target_distance_m 0`) — they measured from 0.63 m, not 0.55.

**Fix 1 — the initial move is chunked** (`align_runner.approach_pose`,
`align.max_initial_steps: 4`): up to N clamped steps of
`max_initial_step_m` until the target is reached; still bounded, still
logs every clamp, and says so if it ends short. Used by both
`run_auto_align` and the `align_required: false` path. Offline: the
real tag-111 seed from the home pose is reached in 2 steps (first step
exactly 0.8 m); `max_initial_steps: 1` reproduces the 0.354 m short
stop from the log.

**Fix 2 — a retry after "tag not seen from the seed" re-estimates the
seed instead of re-sending it** (user: "초기자세로 안되면 다른 태그로부터
초기자세 추정"). `CalibrationOrchestrator._retry_view_tcp`, one
strategy per retry, first available wins: (1) plan seed + median
(aligned − seed) of the session's successes, same ref tag first
(`_seed_corrections`, learned from every entry that succeeded from its
plan seed; recorded per entry as `seed_delta_mm`); (2) the existing
anchor bootstrap (last chain's base pose + map.yaml offsets + odom
heading), which the per-entry override had always pre-empted; (3) plan
seed with the camera raised `retry_raise_m` 0.25 m (half-FOV 0.35 →
0.53 m) when nothing has succeeded yet. Only a seed failure
(`is_seed_failure`: "not detected at iteration 1" / "initial move")
switches seeds; a nav or chain failure retries the same pose. The
record's `view_tcp_source` names the strategy. `retry_count: 1` gives
one alternative; raise it to reach the later ones.

**Fix 3 — the plate-1 plan seeds are now the session's data**
(`scripts/update_plan_seeds_from_session.py <session_dir> <plan> --apply`).
Successful entries get their converged x, y; failed ones get design x, y
+ the same-ref median (−15…−28, −24…+1 mm); **z and orientation stay
design** on purpose, because the recorded z carries the 80 mm clamp
artefact above. Text-level edit: comments survive, each line says where
its seed came from, a header names the session; the generator resets
it. Plate 2 is untouched (no data). Verified offline (31 checks: chunked
approach incl. exception propagation, strategy order / pooling /
attempt indexing / anchor failure fall-through / raise disabled, script
on the real session dir — xy-only corrections, z kept, comments and
header idempotent). Not yet run on the robot.

### 2026-09-04 — RViz out of the launch; robot_ui becomes the camera viewer

User request, same day: stop the main launch opening RViz for the camera
images / front_cam tag view; instead show tag detection for the three
non-Basler cameras in robot_ui with a per-camera use/not-use tick, and let a
click on any of the four camera panes make it the big one.

- **`camera_viewer_node` removed from `mobile_manipulator.launch`** (the
  2026-09-03 pull had it at `auto_start: true`, so every bring-up threw an
  RViz window). The script stays for `rosrun`; `test_all_devices.py` keeps
  it as optional. Eight nodes in the launch now, seven required.
- **Camera panel rebuilt as one main slot + a strip of "cells"** (view +
  its control row). A single **click** on a thumbnail moves that cell into
  the main slot and the previous main cell back down; cells are re-parented
  between layouts, not re-created, so frames, the Basler ROI and checkbox
  state survive a swap. Double-click keeps its maximise meaning.
  `ImageView` gained a `clicked` signal — press/release within 6 px — and a
  plain click **no longer clears the Basler ROI** (it used to count as a
  degenerate drag), otherwise selecting the Basler thumbnail would wipe the
  operator's crop.
- **Tag detection in the UI comes from `robot_camera_node`, not a second
  detector.** Each tag camera subscribes to `/<cam>/tag_overlay` by default
  (the node renders it only while subscribed, so an unticked box costs
  nothing); the `tags` box swaps that subscription for the raw stream
  (`RosBridge.set_stream_source`, one subscription per camera either way),
  the `on` box is the existing `set_enabled` (detector + vendor stream),
  and a `tags: …` label lists the IDs of the latest detection frame
  (`RosBridge.tag_ids`, side_cam detections now subscribed too; ages out
  after 1.5 s without a frame). The Collect tab's duplicate "Tag cameras"
  on/off box was removed.

Verified offline only: UI offscreen (40 checks — initial layout, click
swaps incl. no-op on the main view, Live tab raised, ROI + frame kept, a
real drag still sets an ROI, checkbox → bridge arguments, tag label incl.
age-out, image routing) and bridge (21 — overlay default, raw/overlay swap
drops the old subscription, idempotent, unknown camera refused, `tag_ids`
from a real `AprilTagDetectionArray`, cv_bridge frame routed by name).
Launch XML parses. Not run on hardware; needs the UI restarted and the
stack relaunched (to drop RViz).

### 2026-09-04 — robot_ui Mobile tab: drive a distance / pivot an angle, through mobile_node

User request: "robot ui 에 모바일 로봇 직진 거리하고 회전 각도 제어 할수있게
추가". The UI gets a **Mobile** tab (Task and Calibration sit either side of
it) with a distance spinbox (m, 1 mm resolution, Forward / Reverse), an angle
spinbox (deg, Turn left = CCW / Turn right = CW), a speed spinbox for each,
**Stop base** (preempt cancel) and **Clear stop latch**, plus a `BASE …`
chip in the status bar fed by `/mobile/state`.

**The UI does not publish `/cmd_vel`.** That was the one design constraint:
`mobile_node` is the sole publisher and `tools/vw_drive.py` — which already
does exactly this kind of odometry-closed move — is a second publisher that
refuses to start while the stack is up. So the tool's motion logic was
**ported into `MobileController`** as `drive_distance(m, speed)` and
`pivot_angle(deg, speed)` (both return `(ok, message)`), and `mobile_node`
exposes them on a new `/mobile/move_cmd` JSON topic
(`{"type":"move","distance_m":0.3,"speed":0.05}` /
`{"type":"pivot","angle_deg":90}`), completed through the same `seq` /
`result` handshake as `goto_tag` (`result.tag` is `null`, `result.what`
names the move). The busy guard is shared: a manual move is refused while a
GOTO / TASK / calibration hop is driving and vice versa. `RosBridge` gained
`mobile_drive` / `mobile_pivot` (blocking, `seq`-keyed, with the same
ack-then-complete phases as `MobileClient`), `mobile_cancel`,
`mobile_clear_stop`. `vw_drive.py` itself is unchanged and still the
stack-down bring-up tool.

What was kept from the tool, and why: the trapezoid
`v = min(top, sqrt(2·a·remaining))` with hand-off to a constant-accel
ramp-out when the commanded speed's stopping distance equals what is left
(the 2026-08-14 judder fix), the 2.5 s odom-stall abort, the shortfall
check that catches a base that never moved, and the stale-/no-odom refusal.
Two things are new:

- **The hand-off leads by the base's transport delay.** Simulated against a
  plant with the robot's measured `stop_latency_s` 0.55 s, the plain
  trapezoid overshot by +21 mm at 0.05 m/s, +48 mm at 0.1 m/s and +4° on a
  90° pivot — the same roll the tag stop compensates. The braking distance
  now includes `|cmd| × stop_latency_s`, which is exact for a pure delay, and
  the move **settles for `stop_latency_s` after the ramp-out** before
  verifying and returning, so `mobile_node` reports completion only once
  the base is physically at rest. With the lead in place all delayed cases
  land within 1 mm / 0.1°. The cost of a mis-measured latency is stated in
  the test: with the lead on but a delay-free plant the move ends 27 mm
  short and is reported as such, not as success.
- **Stop-latch rule follows the calibration `BaseInterface`:** an EMERGENCY
  latch (STOP ALL, `/mobile/stop`) refuses the move and needs an explicit
  Clear; a leftover preempt latch is cleared by the new command.

Limits and tuning live in a new `robot.yaml` `robot.manual_move:` block
(tolerances, stall grace, `max_distance_m` 5, `max_angle_deg` 360, default
speeds); requested speeds are capped by `max_linear_speed` /
`max_angular_speed`.

Also fixed on the way: `MainWindow._run` gained `on_error` — the mobile
buttons must re-enable when the bridge call *raises*, but every existing
`on_done` consumer unpacks its result and would not survive a `None`, so a
separate hook was the safe change. And `RosBridge` reads `/mobile/state`
through its own callback now: a first version looked the cached dict up by
bound signal, and PyQt hands out a fresh bound-signal object on every
attribute access, so the lookup always missed.

**Verified offline only, nothing on hardware:** controller (62 checks,
real `MobileController` + real `robot.yaml`, stubbed rospy with a fake clock,
unicycle plant with optional transport delay / frozen odom — forward,
reverse, triangle profile, ±90/270/30° pivots, speed caps, preempt abort,
stall abort, emergency refusal, stale odom, limits, the 0.55 s delayed
cases), `mobile_node` dispatch (16: JSON parse, busy refusal both ways,
seq/result shape, goto unchanged), bridge (12: ack / completion / timeouts /
stale-seq), and the window offscreen against a fake bridge (21: button →
call arguments and signs, in-flight disable/re-enable on both success and
exception, stop/clear/STOP ALL wiring, chip states). `catkin_make` not
needed — Python only, no message change; `mobile_node` and the UI must be
restarted to pick it up.

### 2026-09-03 — robot_sim: the real nav + calibration stacks close the loop off-robot

New package `robot_sim` (dev machine only — it publishes the real driver
topic names). One node simulates ONLY the hardware-owner boundary:
unicycle base with a first-order velocity lag matched to the measured
0.55 s stop roll, pinhole cameras over map.yaml floor tags + both
plates' cross tags (exact robot_camera_node euler encoding, detector
tag-size rescale, all-corners-visible gating), the arm's motion_seq
protocol with the literal "move_cart ok" attribution string, the lifter,
`/safety/estop`. `mobile_node` and the whole path_tag_locator pipeline
run UNMODIFIED against it.

Zero-noise baseline (test_nav.py / test_calibration.py, rerunnable):
single hop 0.1 mm/0.00°; pivot + reverse corridor entry + 8 hops in
80 s at 0.2 mm/0.00°; full calibration of tags 105 and 106 (real
orchestrator → auto-align at 0.8 m → chain) at **0.00–0.03 mm** with
z = −0.080 exactly. Sim forward model and chain inverse are independent
code paths, so this validates both.

**Reach-degradation (user request, sim-verified):** an align move that
fails on IK/reach no longer fails the entry when the ref tag is still
observable — the chain runs from the LAST REACHABLE pose
(`align.continue_on_move_failure`, default on; the chain needs a 6-DOF
observation, not a square view). The result carries a `degraded` marker
through progress / session records / robot_ui ("(DEGRADED)") — verify
those entries' residuals before trusting them, since an oblique view
degrades real-camera detection accuracy. E2E-tested in the sim with an
over-reach seed (sim got an `~arm_reach_mm` IK-rejection model for
this): initial move rejected → degraded → chain exact to 0.00 mm.
Building the test also caught a sim-only deadlock (bump inside the
non-reentrant state lock on the rejection path).

**Error budget in the deliverable metric (scripts/error_budget.py,
user request):** Monte-Carlo through the real chain, everything expressed
as PATH-TAG world-position error (entry 105 geometry, 1.12 m A->B lever,
0.5 px / 0.5 deg tilt / 0.2 deg yaw / Fairino-repeatability defaults):
sigma_xy 4.0 mm (P95 7.8), sigma_z 10.8 mm. Decomposition: **xy is
dominated by the hand-cam observation's IN-PLANE YAW times the A->B
lever** (19.6 mm per deg; tilt hardly touches xy — it leaks into z via
the 0.8 m view height, and z also carries the ~8 mm depth-from-size
noise). front-cam rotation contributes 0.00 mm to B position (last
chain factor). Systematics: every extrinsic/hand-eye mm -> 1 mm; every
0.1 deg of hand-eye or REF-TAG yaw -> ~2 mm — the reference_tags.yaml
yaw-0 assumption is a 19 mm/deg lever and must be checked against the
first session's relative-geometry residuals. Mitigations, quantified:
yaw noise shrinks with tag pixel size (closer view) and with per-frame
medians; at 0.05 deg yaw the budget drops to sigma_xy 1.5 mm / P95 2.7.

**Budget-driven optimization (same day):** three changes, all aimed at
the dominant yaw-x-lever term. (1) View height 0.8 -> 0.5 m
(locator.yaml auto_view_distance_m; plans regenerated) — tag pixels
+80%, angular noise ~x0.55, depth noise ~x0.39, AND flange reach
improved to 0.67-1.20 m (camera lower = closer to the base). (2) The
FINAL chain observations are now the mean of samples_per_iteration
frames (detections.mean_detection — sin/cos-safe angle averaging since
roll sits near ±180; the align loop keeps the median pick, which only
rejects spikes). Before this, the one measurement the result is
computed from was a single frame while only the align loop was
multi-frame. (3) Expected budget at the new settings: **sigma_xy
1.1 mm / P95 2.2 / sigma_z 2.3** (was 4.0/7.8/10.8). Sim E2E
re-verified at 0.00-0.08 mm with the new plans and averaging path.

**robot_ui Calibration tab (user request):** map calibration moved from
edit-a-plugin-constant to a proper panel — plate selector (plan+ref
files switch together), dry-run toggle, START/Cancel, live ok/fail/
degraded counts and last-entry line fed by calib_progress, plus a
single-tag locate row. Session runs on the CallWorker pool; START is
disabled while one runs. Offscreen-tested against the sim: a real
dry-run session streamed all 26 per-tag lines through the actual button
path. The Scripts-tab plugin remains as the automation variant. First field
use immediately hit "timeout waiting for service" — the calibration
nodes were not running (they are NOT in the main launch). Follow-up: a
3 s master-registry poll drives a red "nodes: OFFLINE — run: roslaunch
path_tag_locator path_tag_locator.launch" line on the panel, START
refuses instantly with the same instruction, and the bridge's
locate/run error strings carry the hint too.

**Home-before-nav (user rule):** every base move in a calibration
session is now preceded by a synchronous arm home
(ArmInterface.move_home -> /arm/move_home; `arm.home_before_nav`,
default on) — routes include pivots and reverse corridor entries, and
driving them with the arm extended at the previous view pose was a
collision risk. A failed home fails the entry. Sim E2E re-verified
(0.02 mm), `arm_homed_before_nav` recorded per attempt.

**Yaw is a deliverable too (user correction):** error_budget.py now
reports B's laid-yaw error alongside position. The structure INVERTS
for yaw: front-cam rotation — exactly 0.00 mm on position — enters yaw
1:1 and matches the hand-cam contribution (~0.05 deg each at optimized
settings; combined sigma 0.073 deg / P95 0.14, was 0.28/0.55 baseline).
Front-cam quality is therefore NOT dispensable when yaw matters; the
frame averaging is what carries it. Hand-eye yaw and the
reference_tags.yaml yaw assumption are exact 1:1 biases on EVERY
calibrated yaw — a constant offset over all tags in the output is the
fingerprint of a wrong ref/hand-eye yaw.

Two convention traps the sim caught (regression value, documented in its
README): the corner ORDER is the alignment convention — corner0→corner1
must image at raw +90° when aligned; the +x ordering made the 09-02
heading-correction term servo the robot ~6° off and push the tag's
corners past the frame edge (reproducing the real 2026-08-14 "align
waits forever" failure). And an angular lag that is too slow does the
same through arrival heading error.

### 2026-09-02 — Workspace check on the robot PC; four doc/repo drifts fixed

First session in this checkout (`mobile_manipulator_ws_20260902`, host
`abc-desktop`, the robot PC — `~/navifra` present, `navifra-robot` active).
Read-only against the robot: `rosnode list` showed only the driver's nodes,
`/safety/estop` false, battery 66.5 %. **No motion command was sent.**

**Checked and found sound:** git clean and level with `origin/real`; every
Python file byte-compiles; all `scripts/` entry points executable; every
`config/*.yaml` and `*.launch` parses; `catkin_make` from scratch exits 0
with zero errors (realsense2_camera, orbbec_camera, robot_msgs, robot_ui all
built); apt state right (`ros-noetic-ddynamic-reconfigure` marked manual,
apt `realsense2_camera` absent); the real `MapManager` routes `500→105` as
`[500, 501, 505, 112 … 106, 105]` and the real `TaskManager` registers the
same six tasks at the same lift heights the sections above describe, with
`scan_full_joints` refused for the documented `[150, 300]` disagreement.

**Fixed, on the user's instruction:**

- **`docs/HANDOVER.md` rewritten.** It dated from 2026-07-31 and this file
  told every reader to start there — yet it still said 4 nodes,
  `arm_base_z` 1.025, "the code never commands the lift", `width 0.50`,
  motor 2 under repair, and pointed at `docs/USAGE_kr.md`, deleted
  2026-08-07. Every one of those had been reversed by later work. It is now a
  current-state checklist (verified-on-hardware / open / interim rules /
  document map) that defers to this file for reasoning.
- **The scipy convention corrected** — see Coding Conventions. The
  2026-09-01 finding was real but its premise was half right: the *system*
  scipy is 1.3.3, the *default* one is a user-site 1.10.1. Compat stays
  mandatory for exactly that reason.
- **`src/path_tag_locator.zip` removed from git.** A May-2026 pre-refactor
  copy (still containing the deleted `nav/` tree and `tcp_pose.py`),
  committed 2026-07-30; HANDOVER had it listed as "untracked, confirm and
  delete". Unpacking it over the live package would have resurrected both
  deleted modules.
- **`inference_node` added to the node table** (nine nodes, not eight —
  it has been in the launch since 2026-08-12) and to
  `tools/test_all_devices.py` as optional, with `/inference/predict`.

Not touched: `docs/GUIDE_kr.md` (standing rule). Correction to the
2026-09-01 entry below: `reference_tags.yaml` does NOT still hold example
poses — the six cross tags were filled from the design record in that same
commit (the user pointed this out). What is still open is verifying those
design poses against the physical tags on the first live session. Two
stale comments in that file (an "examples below" line, and a plate-2 note
still quoting the retracted +3.420 m offset) were fixed.

**Lane hops 400→400 skip the in-place align; any hop touching a 100-/500-
series tag aligns (user request, same day). ⚠️ SUPERSEDED 2026-09-04 —
every hop aligns, the key is ignored.** `robot.align_skip_tag_ranges:
[[400, 499]]`; `_align_after_arrival(target, start)` returns without
`align_to_tag` only when BOTH endpoints are in a range. So 400→401 …
410→411 are stop-and-go on Pure Pursuit alone, while 501→400, 405→502,
503→406, 411→504, every corridor hop and every pivot still align. The base
still STOPS on each lane tag (arrival detection is per hop); a true
drive-through would be a route-level change and was not asked for.
Offline (65 checks): six real edges checked both ways. Not driven on the
robot.

**"전진도 자꾸 지나쳐서 멈춰" — the stop now leads by the base's roll
(same day).** Quantified from every hop after the 16:27 restart, using
`arrival` (trigger instant) vs `aligned` (base at rest; align itself took
0.0 s on most hops so it is pure roll): **forward rolled 5.8 mm median
(0.7–9.0, 23 hops) at a 0.011 m/s command, reverse 12.6 mm (17 hops) at
0.022 m/s** — the same ~0.55 s of continued motion either way, i.e. the
base executes commands with a transport delay. New `stop_latency_s: 0.55`:
`_publish_vel` keeps a command history and `_pending_roll_m()` integrates
the last 0.55 s of it (exact for a pure delay; falls back to
|cmd| × latency without history); the stop fires that far early along the
direction of travel, and `center_x_stop_tolerance` becomes a floor under
that lead (`fire_px = max(tol, lead)`), so the base ends ON the target
column instead of `tol` short. The final-approach envelope aims at the
same `fire_px`. Every `arrival` record now carries
`commanded_speed_at_stop_mps`, `stop_lead_px`, `stop_lead_mm` so the
latency can be re-fitted from `(arrival − aligned) / speed`. Verified
offline (57 checks) against a plant with a 0.55 s transport delay: forward
and reverse end within 0.0 mm of the tag (without the lead: 4.3 mm past,
matching the robot), the realistic reverse case — tag hidden beyond 2.5 cm
plus the delay — ends +1.2 mm (without creep zone + lead: +13.5 mm,
matching the robot's 6–17). Not driven on the robot.

**"자꾸 태그를 조금 지나쳐서 멈춰" — root-caused to REVERSE hops, fixed
with an odom creep zone (same day).** The day's records after the 16:27
restart (offset 0, tol 4) show arrivals dead on the crosshair in both
directions (±4 px), but the `aligned` record — taken after the base has
physically stopped — sits **+6…17 mm past the tag on reverse hops** and
only 1–2 mm on forward ones. The mobile_node log explains it: reversing,
the target tag is first detected when it is **~3 cm from the lens** (the
lens is 100 mm ahead of the bumper, so the bumper hides the floor behind
it; the docs already note "the bumper appears on the LEFT of the image"),
the final approach latches at 2.5–2.9 cm from 0.030–0.037 m/s and the stop
fires at 0.020–0.024 m/s. Forward sees the tag 26 cm out and stops at
0.010. Odom remaining distance was within 3 mm of the tag stop on those
0.40 m hops, so new `blind_approach_dist: 0.12` / `blind_approach_speed:
0.015` cap the command inside the last 12 cm of ODOM distance regardless
of visibility; the tag-based final approach then only has to go
0.015 → 0.010 over the last few cm. Applied after the backup `min_speed`
branch and before the final-approach override. Costs ~8 s per hop. The
1 m pivot-exit hop (507→137) showed ~6 cm of odom error, hence the 12 cm
margin. Offline (49 checks): a reverse hop whose tag is hidden beyond
2.5 cm now enters the final approach at 0.015 m/s instead of 0.033 (the
sim reproduces the robot's entry speed without the zone), stops from
≤0.012, survives a +6 cm odom over-estimate; forward unchanged. Not
driven on the robot.

**Pure Pursuit on a visible tag runs at 1/3 speed (user request, same
day: "너무 빨라").** New `robot.tag_visible_speed_factor: 0.333`, applied
in `execute_pure_pursuit` to the profile speed whenever the target tag is in
`detected_tags` — so at `max_linear_speed` 0.1 the visual approach closes
at ~0.033 m/s while blind driving keeps the full profile, and the 6 cm final
approach now latches from that lower entry speed (gentler solved
deceleration). Applied before the final-approach override, which still
wins. Offline (42 checks): a 0.80 m hop reaches 0.1 blind, never exceeds
0.0333 once the tag is seen, cruises at 0.033, arrives; factor 1.0 restores
the old behaviour. Not driven on the robot.

**`center_x_stop_offset` 50 → 0 (user request, same day: "태그 중심이
목표 중심점에 와야 정지").** The stop target column is now the optical axis
/ overlay crosshair, for both directions. Config only; the 4 px tolerance
and the 6–8 mm post-trigger roll noted below still apply, so the tag will
sit a little past the crosshair at rest. Offline: forward stops at cx+3.8,
reverse at cx−3.8 (sim, 36 checks). Not driven on the robot.

**Align and stop-centring tightened, from the day's hardware record (user
request, same day: "얼라인 기준하고 태그 정중앙 정렬 기준을 더 빡세게").**
`align_threshold_deg` 0.5 → **0.2**, `center_x_stop_tolerance` 10 → **4 px**,
plus a new `align_min_angular_speed: 0.01` rad/s floor inside
`align_to_tag`. The numbers came from the 15:33 `GOTO 400` run (194 aligns
in the mobile_node log): finals were median 0.24°, and every align that took
~2 s ended at 0.43–0.49° — the loop was exiting on first contact with 0.5,
not converging, so 0.2 is reachable. The floor exists because the P term at
0.2° is 0.0028 rad/s, which a skid base likely cannot execute; 0.01 is above
the 0.0063 rad/s the base was observed still turning at, and one 10 Hz tick
at the floor is 0.057°, inside the band. `approach_yaw_tolerance_deg` is set
explicitly (0.5) so the approach slowdown is unaffected. ⚠️ Found while
reading the same records: arrivals land at **+40…46 px**, not the +54…60 the
trigger fires at — the base rolls **6–8 mm past the trigger line** before it
actually stops. Tolerance therefore buys ≤2.4 mm; centring the tag exactly on
the target column means raising `center_x_stop_offset` by ~18 px, which was
NOT done (user's call). Also seen: the `aligned` record's fore-aft offset is
20–80 px larger than the `arrival` one on most tags — the in-place rotation
translates the lens, so align is not free of position error either. Verified
offline (35 checks total): converges to <0.2° from 3° and from 0.4°, no
non-zero command below the floor, floor 0 restores the pure P term, forward
trigger within 4 px + one tick of the target. Not driven on the robot.

**Zone A lane (tags 400–499) drives WITHOUT prediction (user request,
same day).** `predictive_centering.disabled_tag_ranges: [[400, 499]]` (+
`disabled_tag_ids`) in robot.yaml; `_make_prediction_segment` returns None
when EITHER endpoint of the hop matches, so 501→400 and 405→502 are excluded
along with 400→401…410→411. With no segment the existing code already does
exactly the three things asked for: tag visible → plain Pure Pursuit (no
approach-heading blend, no ApproachBalance slowdown), tag not visible →
`omega = 0`, a straight-line command at profile speed, stop line →
`align_to_tag`. Zone B–E hops keep map_world / map.yaml prediction.
Verified offline (9 checks added to `t_stop_align.py`, 30 total): the
segment is None for the three hop kinds and present for 505→112; a
0.50 m 400→401 drive with a 30 mm lateral offset commands omega exactly 0
on all 78 blind ticks, arrives, and still aligns. Not driven on the robot.

**Every hop — pivots included — now ends with an in-place align, and the
reverse stop line stays on the existing target column (user request, same
day).** Changes in `mobile_controller`, verified offline against the real
class (21 checks, stubbed rospy + unicycle plant + simulated front_cam,
scratch `t_stop_align.py`); **not driven on the robot**, needs `mobile_node`
restarted. (1) Stop line: a direction-mirrored variant
(`cx ± center_x_stop_offset`, reverse stopping with the tag 24 mm short of
the lens on the image-LEFT side) was implemented first and **backed out
within the hour** on the user's instruction that reverse must stop at the
previously established target centre. So `target_x = cx + offset` for both
directions, as before: forward brings the tag to that column from the right
(first trigger ~698 px), reverse from the left (~678 px), same physical spot
over the tag to within the 10 px tolerance. No behaviour change on this
point. (2) `go_to_next_tag`'s pivot branch used to return straight from the
odometry-only `execute_pivot`; it now goes through the same
`_align_after_arrival(target_id)` tail as move hops, squaring up to the
exit tag (501→505 pivots then aligns on 505; sim: 90° pivot + 3° tag error
converged to −0.48°). (3) `align_timeout_s: 20.0` — the 2026-08-12 open
defect — because a pivot can land with the exit tag out of frame, and an
unbounded wait there would now hang every corridor entry. Timeout → `stop()`,
`logerr` with reason, hop returns False (the task fails, which is what the
600 s client timeout did anyway, ten minutes later).

**First launch of `path_tag_locator.launch` on the robot died at parameter
load** — the user ran it and hit `cannot marshal None unless allow_none is
enabled`. Cause: `map_calibrator.yaml` had `lift_height_mm: null`, and that
file goes through `<rosparam command="load">`, whose XML-RPC transport has
no encoding for None; roslaunch aborts before starting any node. (The
`null`s in `robot.yaml` are harmless only because nothing loads that file
with rosparam — the nodes read it as a file.) Fix: the key is now **−1**,
`map_calibrator_node` treats negative/absent as "leave the lift alone", and
the yaml says why it must not be `null`. Verified by marshalling all three
package configs through `xmlrpc.client.dumps` offline and by a real
`rosparam load` of the file into a scratch namespace on the live master
(then deleted). The launch itself still has not run to completion on the
robot — the main stack was not up at the time.

**Are the calibration results usable? Not as produced — but the archived
observations let the bias be removed offline, and the corrected map is
now the best estimate of the ACTUAL tag positions.** Two plate-2 sessions
(14:24 → `map_world_20260902_142402.yaml`, 14:53 → `…_145354.yaml`)
agree with each other to 2–3 mm mean / 7–8 mm sd in xy (z: 27 mm sd) —
the measurement is repeatable. Against `map.yaml` they are biased: zone D
dy +65 mm, zone E dy −49 mm (sign flips with robot heading ⇒ an offset
fixed in the arm frame), dx cycling with the plan's 3-tag camera-yaw
pattern, adjacent spacing 359–464 mm on a 400 mm grid (6/40 edges over
3 cm). The user's point stands — map.yaml is the PLAN and calibration
exists to find where the tags really are — so the question was whether
that deviation is placement or chain bias. Test without design
coordinates: recompute the chain from the 22 archived `result.npz` with
a 4-parameter hand-eye correction (camera-frame translation + yaw)
fitted only to "400 mm spacing + straight corridor". Result: dt =
(+101, −3, +24) mm, yaw +1.55°; physical rms 26 → 11 mm; and the
positions then sit +5±3 / +9±3 mm (zone E) and +18±10 / +8±13 mm
(zone D) from the plan — the plan was never asked for, so this is the
tags actually being close to it. Chain bias, not placement. Outputs:
`map_world_20260902_145354_handeye_corrected.yaml` (with
`deviation_from_design_mm` per tag; 4/40 edges still over 3 cm, all
touching tag 131, the one entry whose align wobbled), the corrected
hand-eye applied to `T_hc2ee.npz` (spun-only file kept), plans
regenerated (seeds moved with the camera offset), comparison CSV
`compare_map_world_142402_vs_145354_vs_map.csv`. Caveat: a 4-parameter
fit from one session is an estimate; `handeye_calib` on this robot is
still the real fix. Until then, `predictive_centering.map_world_path:
"latest"` makes mobile_node steer blind segments from whichever
map_world is newest — the biased 14:53 file by timestamp — consider
`use_map_world: false` or pointing it at the corrected file.

**Full plate-2 session ran (25 entries, 21 ok, 17 min) — and the align
got SLOWER as it went. Cause: MoveL, i.e. the change requested an hour
earlier; fixed by splitting move types.** Per-move durations from the
arm_node log, same kind of move: the ~300 mm / 20°-yaw initial view move
took **5–6 s** for tags 128–137 and **22–34 s** for 138–145; small
correction steps 0.4–2.2 s early vs 2–9 s late; two 740–800 mm / 40–90°
moves hit the SDK's **60 s RPC timeout**, and after such a timeout the
controller keeps executing while the SDK has returned — a 0 mm "move"
then took 14 s queued behind it. A straight-line Cartesian move that
reorients the wrist is joint-speed-limited near the wrist singularity,
and zone E's mirrored seeds put the arm there; MoveCart (joint
interpolation) to the same pose does not care. So `move_cart` now takes
`linear` (JSON key on `/arm/move_cart`, default true = MoveL as the
user asked for the UI): the calibration's **initial view-pose move and
the align_required=false move send `linear=false` (MoveCart)**, the
small correction steps stay MoveL. Also: on an RPC "timed out" the
controller now polls `GetRobotMotionDone` (≤120 s) before releasing
`busy`, so the next command cannot be sent into a moving arm.

Second time sink in the same session: **most entries ran all 5 align
iterations** with xy already at 1–5 mm because the tilt jitters around
the 1.0° tolerance (std 0.5°, spikes to 3–4°, one 34° first frame). New
`align.samples_per_iteration: 5` — `wait_for_tag_detections` collects
5 frames (~0.2 s) and the **median-tilt frame** is used (a real frame,
no rotation averaging). Verified offline (8 checks: MoveL/MoveCart
dispatch by flag, timeout → motion-done wait → failure reported, median
pick ignores the spike, config loads, payload carries `linear`). Needs
arm_node restarted (main stack) to honour `linear`; until then every
move is still MoveL.

**Live test of the align fix — a SECOND root cause found and fixed:
`quad_decimate`. Converged on the robot.** First live attempt with the
spun hand-eye, from the plan's seed for tag 129 (robot standing on 129,
front_cam centred within 15 mm): iter 1 read xy 56 mm but **tilt 32.6°**
at z 0.68 m, the 15°-clamped step then pushed xy to 171 mm and the tag was
lost. A flat tag under a vertical camera cannot tilt 32°, so the detection
was wrong, not the geometry. Re-parked at the seed, hand_cam then
detected NOTHING in 150 frames; the saved frame shows the tag dead centre.
Offline on that frame with the live intrinsics: dt_apriltags
`quad_decimate` 1.0 → id 0, z 0.796, **tilt 2.74°**; 1.5 → tilt 20.7°;
2.0 (the library default, and what `robot_camera_node` used) → no
detection. The 69 px tag halves to ~35 px under decimation and the border
merges with the plate seams/glare. `robot_camera_node` now takes
`robot_camera.quad_decimate.<cam>` from robot.yaml (front_cam 2.0 —
unchanged, its 30 Hz/109 ms numbers were measured there; side_cam and
hand_cam 1.0). Restarted only that node (`rosnode kill` + `rosrun`; the
launch's `~driver_*` params persist on the master) — hand_cam then saw
tag 0 in 95/95 frames, z 0.797, tilt 1.75 ± 0.5°. **Second align run from
the same seed: xy 62 → 3.1 → 3.4 mm, tilt 3.0 → 2.6 → 0.59°, converged
in 3 iterations, full locate succeeded** (tag 129 came out 16 / 67 / 36 mm
from its design offset in x / y / z relative to the ref tag — the
cm-level residual expected from the May hand-eye translation). MoveL was
in the loop for every step. So the morning's divergence had two
independent causes stacked: the 180° hand-eye (direction) and
decimation-corrupted tilt (magnitude); both had to go.

**`move_cart` now drives `MoveL`, not `MoveCart` (user request, same
day).** `ArmController.move_cart` — the path behind `/arm/move_cart`, so
both the UI's absolute/jog moves and the calibration align steps — used
`MoveCart`, which interpolates in JOINT space to a Cartesian endpoint and
lets the controller choose the configuration; the TCP can arc off the
straight line on the way. `MoveL(target, TOOL_ID, 0, vel=, acc=)` keeps
the TCP path linear, which is what a camera-frame correction of ≤100 mm
assumes. Trade-off recorded at the call site: a straight path through a
singularity or joint limit is refused (error) where MoveCart would have
gone round it. Result strings keep the `move_cart` wording on purpose —
`ArmInterface` attributes completions by that substring. The Keyence
standoff loop already used MoveL; this was checked against the SDK's
actual signature (`MoveL(desc_pos, tool, user, joint_pos=…, vel=20.0,
acc=0.0, …)`). Not run on hardware yet. Note: this was NOT the cause of
the align divergence — see the next entry — but it removes one source of
path uncertainty from the loop.

**`auto_align` diverged on the real robot — hand-eye was 180° off (found
and patched, same day).** First live calibration attempt (plate 2, tags
126–150): every entry failed. The per-iteration log is the tell — xy offset
66 → 131 → 258 mm and tilt 5.1 → 8.5 → 17.1°, doubling each step in three
separate runs, until the clamped step became unreachable (Fairino 112) and
the entry timed out. Diagnosis, all read-only against the robot: (1) the
euler encode/decode round trip is exact (1e-15), so the detector path is
not it; (2) the live hand-cam observation at the arm home pose puts ref tag
0 at z = −0.584 m in the arm frame with the hand-eye as loaded — the plate
top is at −0.572 — so the hand-eye's z / direction is right; (3) the same
observation lands on the map position (arm-frame y = +1.01 m) only if the
hand-eye is spun 180° about the optical axis (+0.995 m; as loaded +0.778;
EE-spun +0.621); (4) simulating the real align functions with exactly that
mounting error reproduces the log (65 → 116 → 183 mm, 5 → 10 → 20°), while
a transposed detector rotation does not. So the May-2026 `T_hc2ee.npz`
describes a mount rotated 180° about the optical axis from today's.
**Interim fix:** `T_hc2ee.npz` := `Rz(180°) @ old` (old kept as
`T_hc2ee_2026-05-27_old_mount.npz`); plans regenerated — the seed TCPs
came out numerically identical (the planner's free camera-yaw sweep
absorbs the spin), only the annotated cam yaw moved by 180°. Offline, the
loop now converges in one step from every seed with a 65 mm base-stop
error. **The translation is still May's** — a proper `handeye_calib`
session on this robot is the real fix (HANDOVER §2-2).

Also fixed: `ArmInterface.move_cart` attributed a refused move as a
foreign motion — arm_node reports success as `move_cart ok` but a refusal
as `MoveCart error 112`, and the check was a case-sensitive
`'move_cart' in message` — so every unreachable align pose cost the full
60 s timeout instead of failing at once. Now matches both spellings.

Not verified on hardware yet: the corrected hand-eye has not driven an
align loop. Note the align-history is lost when `run_auto_align` raises
mid-loop (the session record shows `auto_align: null` for those attempts)
— the iteration numbers above came from rosout, not the record.

**Drive rule made unconditional (user request, same day): tag visible →
Pure Pursuit; tag not visible → predictive drive on map geometry; tag
reaches the stop line → stop, then ALWAYS align.** (Exception added later
the same day: hops touching tags 400–499 skip the predictive part and drive
straight when blind — see the entry above.) Two gates in
`mobile_controller` made this conditional. (1) `_prediction_xy` returned
nothing for a tag missing from `map_world` whenever a map_world file
existed and `fallback_to_map_yaml` was false (it was) — so with one
calibrated endpoint the segment was `None` and the blind part of the hop
had NO steering at all; map.yaml is now always the fallback (calibrated
positions still win when both endpoints have them), and the only `None`
left is a tag in neither file. (2) `go_to_next_tag` skipped
`align_to_tag` after arrival when `post_move_align_enabled` was false or
the tag's centre-row error exceeded `post_move_align_y_tolerance_px`;
both branches and both keys are gone — every `move` hop now ends
stop → `align_to_tag`, no exceptions (the only remaining skip is a
temporary-missing VIRTUAL tag, which has nothing to align to). Verified
offline (7 checks on the real methods with the real map: map.yaml when
nothing / one endpoint is calibrated, map_world when both are, align
called after every successful move even with a 140 px row error, none
after a failed move, pivot untouched). `fallback_to_map_yaml` is kept in
robot.yaml but no longer read. Not driven on the robot.

**Calibration starts at each plate's first tag, not the dock (user
request, same day).** Both generated plans carried `nav_start_id: 500` on
their first entry, so a session first drove to DOCK 500 and then to the
plate's first tag — for plate 2 that is a 9 m detour through zone A and
the 507 pivot before the first measurement. The generator
(`generate_calibration_artifacts.py`) no longer emits it; both plans were
regenerated (only that line changed — verified by diff) and now start
from wherever the base is parked, which must be ON tag **100** (plate 1)
or **126** (plate 2) with front_cam seeing it, since `move_to_tag` takes
its start node from the visible / last-known tag. README updated at all
four places that said "park at DOCK 500".

**Per-command navigation record (user request, same day: "a new file
per task command, never overwritten").** `mobile_controller` used to
write ONE `~/.ros/path_tag_locator/tag_alignment_results.yaml`, keyed by
tag — so every arrival at tag 105 overwrote the last one, every task
shared the file, and with no `map_world_*.yaml` present (i.e. today) it
recorded nothing at all and just warned. Replaced by
`begin_nav_session(label)` + `_record_tag_offset(tag, stage)`: each
`TASK …` / `GOTO n` on `/task_command` opens a new
`~/.ros/apriltag_nav/nav_log/<YYYYMMDD>/<YYYYMMDD_HHMMSS>_<cmd>.yaml`
(`mobile_node` subscribes to `/task_command` for exactly this; a
`/mobile/goto_tag` published outside a task_executor task — a calibration
session, a direct publish — gets its own `goto_<tag>` file, decided from
`/task_state`), and every arrival appends a record: `arrival` when the
pure-pursuit stop fires (new — previously nothing was recorded there)
and `aligned` after `align_to_tag`. Each record carries the camera-frame
error (`image_offset_px`, `tag_pose_m` fore-aft/lateral/depth,
`camera_center_offset_from_tag_mm`, px→mm through depth/fx, the tag-edge
yaw error, odom travel) and, only when a map_world exists, the old
world-frame `calibrated_reference` block. Config key
`alignment_result_dir` replaces `alignment_result_path`. Same-second
collisions get a numeric suffix. Verified offline (14 checks on the real
controller methods: per-command files, append order, old file untouched
by the next command, suffixing, GOTO/goto labels, disabled → nothing,
no-map_world → recorded anyway, map_world → reference rotated by the
tag's yaw). Not yet driven on the robot.

**Camera-frame error recording (user request, same day).** `result.yaml`
from `locate_path_tag` used to record the alignment error as two scalars
(`xy_offset_m`, `tilt_deg`), and nothing at all about where the tag sat
in the image when `auto_align` was off. It now records the error **in the
camera optical frame** — `position_m` (x = image right, y = image down,
z = range along the optical axis) and `rpy_deg` of the tag in that frame
— under `observations:` for BOTH observations the result is built from
(hand_cam → tag A, front_cam → tag B, recorded on every successful call),
and under `auto_align.tag_in_cam` (final) + `auto_align.history` (one per
iteration) when auto_align ran. The axis definition is written once at the
top of the file as `camera_frame_note`. `AlignMetrics` gained the raw
`t_cam_m` / `rpy_cam_deg` it always derived the scalars from (defaults
keep old constructors valid); `is_converged` and the service response are
unchanged, so no `.srv` rebuild. `persistence._plain()` coerces numpy in
nested reports. Verified offline (10 checks: rounding, square-on tag reads
rpy `[0, 0, spin]` with tilt 0, the note appears exactly once, numpy
coerced, a call without observations writes the old file shape).
`map_calibrator` no longer drops them — see the next paragraph.

**Append-only calibration session record (user request: "record
everything, overwrite nothing, number it 1, 2, 3 in order").** Before,
a session left only `map_world_<ts>.yaml` (per-tag upsert, so a retry
overwrote its predecessor and the align report was discarded), the
timestamped `locate/` run dirs, and a `CalibReport` that never reached
disk. New `calibration/session_log.py` (`SessionRecorder`) writes
`<save_dir>/calibrate/<YYYYMMDD_HHMMSS>/` per session with one file PER
ATTEMPT under `entries/`, named `NNN_tag<id>_attempt<n>_<ok|fail>.yaml`
in execution order — a retry is a new number, a failed attempt keeps
everything up to the failure (`_run_one` now fills a `rec` dict
progressively), a missing-ref entry is recorded without motion — plus
`session.yaml` (ordered index, rewritten atomically after every attempt),
`entries_log.csv` and a `map_world.yaml` copy. Each entry carries nav
state, view pose + source, the full auto-align report (camera-frame
`tag_in_cam` + per-iteration history), both camera-frame observations,
TCP, lift height, the world result and the 4x4 transforms, and a pointer
to its `locate/` run dir. The user's `map_out_path` keeps its old
semantics; the session copy is the one guaranteed to survive. Dry runs
record nothing. Progress messages gained a `seq` field (robot_ui ignores
unknown keys). Verified offline (20 checks, orchestrator driven end to
end against fakes: retry numbering, failed-attempt contents, second
session in a new dir with the first untouched, dry run silent).

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
`map_calibrator.yaml lift_height_mm` (**−1** = leave the lift alone; it was
`null` until 2026-09-02, which roslaunch cannot load — see that entry) makes
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
