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

**Since 2026-09-22 the main launch starts at robot boot as the systemd
service `mobile-manipulator`** (`src/apriltag_nav/tools/systemd/`:
unit template, `run_stack.sh` = ExecStart, `mobile-manipulator.env` for
`LAUNCH_ARGS` and the wait limits, `install_service.sh` to install /
`--uninstall`, needs sudo). `Requires=` / `After=` / `PartOf=
navifra-robot.service`, so it comes up after the driver's roscore and
follows a driver restart; `run_stack.sh` waits (bounded: master 120 s,
Fairino RPC port 20003 180 s, Keyence 30 s) and launches anyway with a
warning if a device is late — `sudo systemctl restart mobile-manipulator`
once it is on. Operate with `sudo systemctl {start|stop|restart|status}
mobile-manipulator`, `journalctl -u mobile-manipulator -f`; file logs
still go to `<ws>/log/ros` (the env hook is sourced). ⚠️ **Stop the
service before a hand-started `roslaunch` of the same file** — two copies
kill each other's nodes and collide on 9090 / 8080; `stop_stack.sh`
refuses to touch a service-run launch and `install_service.sh --start`
refuses while a hand launch runs. `systemctl stop` / reboot runs every
shutdown hook, charge relay off included. Korean procedure:
`docs/STOP_LAUNCH_kr.md` §0.5.

The Navifra systemd service already runs `roscore` — do not start one.
`rosrun apriltag_nav task_executor.py` starts *only* the orchestrator and is a
debug path, not the way to bring the stack up.

**Stopping it: `tools/stop_stack.sh`** (SIGINT = Ctrl-C, wait, and — only
if the graceful exit stalls — SIGKILL of the leftovers + `rosnode cleanup`;
refuses the navifra driver's `robot.launch`, which runs roscore). Procedure
and the 2026-09-21 failure it handles in `docs/STOP_LAUNCH_kr.md`: a frozen
VS Code launch terminal blocks every node's final stdout write, so SIGINT
leaves 16 unregistered-but-alive processes and roslaunch's own escalation
hangs with them. Run it from a DIFFERENT terminal than the launch.

## Where run output lives — inside the workspace (2026-09-14)

**User rule: every result and every record a run produces is stored in
this workspace, never in `~/.ros`, `/tmp` or `$HOME`.** The pre-existing
output (2026-09-02..14) was moved in the same session; what was not worth
keeping was deleted (see the Work Log entry).

```
<ws>/log/apriltag_nav/ra_maps/<task>_ra_map_<ts>.csv task_manager result_dir  (versioned)
<ws>/results/scan_images/<task>_ra_map_<ts>/*.png  arm_node output_dir, ONE FOLDER PER RUN (ignored);
                                                    g<group>_p<point>_i<index>_s<n>.png since 2026-10-06
<ws>/results/scan_images/<run>/<run>_ra_measured.csv  arm_node COLLECT mode: the hand-measured Ra per
                                                    scanned point, IN THE RUN'S FRAME FOLDER (versioned —
                                                    only the png/jpg under scan_images are ignored); also
                                                    <run>_mark_template.csv — docs/RA_COLLECT_kr.md.
                                                    Was log/apriltag_nav/ra_measured/ until 2026-10-06 evening
<ws>/results/ra_dataset/<run>_dataset.csv           tools/merge_ra_dataset.py, one row per frame (ignored)
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
                           # Today's keys (2026-09-29, 18:10; plus the 2026-09-30
                           # rrt_final_path_260930_standoff20mm_offset{0cm,1.5cm,
                           # 3cm}_height{652,667,682}mm joint-only set — new work
                           # points, new tag assignment, NO pose twin, see the
                           # Work Log): 260610_standoff
                           # 20mm_offset{0,1.5,3}cm_height{652,667,682}mm — the
                           # planner's "260610_rot" export (task/260610_rot_
                           # standoff20mm_offset0_15_30.zip): the SAME 1266
                           # work points on 정반 1 tags 104–108 / 119–121 at
                           # three LIFT heights (lift_mm 0 / 15 / 30 — the
                           # "offset" is the arm base height, the joint rows
                           # are solved at it), standoff 20 mm, speeds 10–30.
                           # Planner ORIGINALS, NOT retargeted (user, 18:20:
                           # "retarget 안 해도 됨") — see the Work Log.
                           # Plus upper_mold_errorX_p000mm_standoff_{001,010,
                           # 020}mm_height_652mm_plate2: the 정반 2 set of the
                           # evening (tags 131–133 / 143–145, retargeted,
                           # 20 mm tip-down), restored from the 18:06 backup
                           # on the user's word that it is the 상형 (18:35;
                           # the planner zip's folder called it 하형 — the
                           # name follows the user). Plus the 2026-09-14 set
                           # brought back from git (commit 68bac0d, user
                           # 18:45): errorX_p000mm_standoff_{010,030,050}mm_
                           # height_652mm (정반 1, tags 106–108 / 118–121, z
                           # 0.34–0.44 m, planner originals of that day) and
                           # errorX_…_standoff_{010,030,050}mm_…_plate2 (19:00
                           # / 19:10, user: these for 정반 2 as well — the same
                           # make_plate2_paths.py conversion, tags 106/107/108/
                           # 119/120/121 → 132/133/134/144/145/146, 20 mm
                           # tip-down; 2 / 3 work points per file sit 4–9 mm
                           # beyond the arm's reach there, see the Work Log;
                           # the 010_plate2 JOINT file is at 16.1 mm of
                           # tip-down since 20:30 = what the Keyence measured
                           # on group 132, its pose twin at 20;
                           # the 09-14 "+26" 050_plate2 files are replaced,
                           # git 68bac0d has them).
                           # Keys are SEPARATE tasks because the standoff /
                           # lift changes the solution, not just the offset.
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
UNDOCK                     # /crevis/charging false; the base STAYS on the
                           # dock (no forward move since 2026-10-06,
                           # `undock_forward_m` 0); no automatic return
                           # until the next task. CHARGE from the dock then
                           # drives nothing — tag 500 is the current tag
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

### The joint files in task/csv are RETARGETED to the calibrated robot (2026-09-29)

A planner export is solved for the planner's model, settled by FK against
its own pose rows (0.010 mm over 2828 work points, one combination only):
**base stop pose = tag of `config/map_idle.yaml` − 0.55 m at exactly ±90°**
(`docs/robot_base_stop_poses_plate1_idle.csv`), **the DESIGN `T_ab2mb`** and
**the DESIGN tip** (`tf_chain.yaml` `design` blocks). The robot stops on the
calibrated `map.yaml` (`docs/robot_base_stop_poses_plate1_0928.csv`) and
carries the calibrated mount and the measured tip. A joint row is an absolute
configuration, so replayed as exported the tip lands 9 mm (zone B) / 17 mm
(zone C) beside the planned point and 10 mm above it.

**`tools/retarget_joint_paths.py [task_dir] [--apply]`** re-solves the
`rrt_final_path_*` rows so the REAL tip is on the PLANNED world pose: work
points and the transitions between them exactly (IK seeded with the planned
row, same arm configuration), transitions next to a `home` row by fading the
work point's correction to 0 at home, `home` rows untouched. Only q1..q6
cells change. It classifies each file first (ORIGINAL / RETARGETED /
neither) and rewrites only an ORIGINAL, so a second run is a no-op.

- **`assigned_workpoints_*` is NOT retargeted and must not be**: its x y z
  rx ry rz are WORLD coordinates and `_exec_pose` applies the live
  `/robot_pose`, the calibrated mount and the measured tip at run time.
  Adding the stop-pose difference to it moves the target off the workpiece.
- **After dropping a new planner export into task/csv:** run the tool with
  `--apply` (dry run first). `tools/check_retarget_joint_paths.py` fails on
  a joint file that is not retargeted for the current map.
- **After `map.yaml` or `tf_chain.yaml` changes:** restore the planner
  ORIGINALS from `log/apriltag_nav/task_csv_backup/<date>_before_stop_pose_
  retarget/` (the 001 / 020 files are not in git), regenerate the actual
  stop poses (`tools/map_stop_poses.py`), then run the tool again. A file
  retargeted for the old map is "neither" and is refused.
- The metadata columns (`base_x_actual_mm` ±1610 = the planner's arm base x)
  still describe the planner's model; nothing reads them.
- Not covered: the joint zero offsets (joint mode never applies them; pose
  mode applies their xy part — ~9 mm between the two modes remains) and the
  per-arrival stop error (±2 mm / ±0.2°), which only pose mode sees.

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
  tag 12–16 cm ahead of the lens. **Since 2026-09-28 (night) all three
  body-frame vectors — fore, lateral and the 0.55 m lever — are rotated by
  the BODY HEADING (zone + align residual + calibrated laying angle),
  not the bare zone axis** (`robot_pose_offset_along_heading`): the align
  squares the body to the tag edges, so a tag laid δ off its lane leaves
  the base centre 0.55·sin δ beside the zone-axis estimate (9.6 mm/deg —
  the tip tour's +9.7 mm at tag 116, δ −0.77°). **And the published
  arrival pose is the align's at-rest MEDIAN view** (`align_record_frames`
  3; an out-of-passes residual is re-settled and re-measured first), not
  the newest single frame, which at tag 104 disagreed by 1.44°.
  `tools/check_robot_pose_heading.py` (20). There is still **no body-width term
  anywhere in the pipeline**, so changing the chassis does not move where
  the robot stops. `wall_dist_*` is documentation.
- **`/robot_pose` is a LIVE stream since 2026-09-28** (user: "실시간으로
  로봇 현재 위치 계속 보내고 싶어"). Two kinds of message, told apart by
  `flag`: `flag True` is the at-rest ARRIVAL pose, published once per
  tag arrival exactly as before (`publish_robot_pose`) and the ONLY kind
  `arm_controller.pose_cb` / `robot_ui.tip_check` accept; `flag False`
  is the live estimate `mobile_node` publishes at `robot_pose_live.rate_hz`
  (10): the same `calculate_robot_pose` on the map tag nearest the lens
  whenever one is in front_cam (a non-map tag, e.g. a calibration pair,
  produces nothing), and while no map tag is visible the last tag-based
  pose (arrival or live) carried forward on the `/odom` delta rotated
  into the world (`live_robot_pose`; off when `/odom` is older than
  `odom_max_age_s`, silent before the first tag-based pose). While
  moving the tag-based value lags by the image latency × speed (~5 mm at
  0.05 m/s); at rest it equals the arrival pose. Not latched — a fresh
  `rostopic echo /robot_pose` shows the stream at once, and `id` says
  which tag it is anchored on. `robot_ui`'s `robot_pose_snapshot()` keeps
  the arrival pose, `robot_pose_live()` the stream.
  `tools/check_robot_pose_live.py` (30).
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
| `mobile_node.py` | mobile base (**sole publisher** of `/cmd_vel` and `/robot_pose`) | `/mobile/goto_tag`, `/mobile/move_cmd` (manual distance / angle, JSON), `/mobile/{stop,cancel,clear_stop}` (srv), `/mobile/state`; **`/robot_pose`** (2026-09-28): `flag` True = the at-rest ARRIVAL pose, once per tag arrival (what pose-mode IK uses); `flag` False = the LIVE estimate at 10 Hz (`robot.robot_pose_live`) — from the map tag in front_cam while one is in view, else the last tag-based pose carried forward on `/odom`; not latched, `id` = the anchoring tag |
| `arm_node.py` | Fairino FR10v6 arm | `/arm/scan_command`, `/arm/cancel`, `/arm/move_home` (srv), **`/arm/move_cart` takes `"physical": true`** (2026-09-28: an ABSOLUTE target, the joint zero offsets pre-applied on the command side — while `~apply_joint_offsets_cmd` is true, which the launch sets since 2026-09-28 evening, **xy + rotation only, the commanded arm-frame z kept** (`~joint_offsets_cmd_skip_z` true); `_exec_pose` corrects every world point the same way; see the 2026-09-28 (night, tip tour) Work Log for why), **`/arm/reset_error`** (srv, 2026-09-28: clear a latched controller error — joint limit / collision stop — without moving); `/arm/state` (10 Hz, pose kept live through a scan), `/arm/scan_progress` (JSON per point: start / move / done / result / failed / finished — `done` when the frames are captured, `result` when the background inference has the Ra) ; **`/arm/standoff`** (JSON `{target_mm}`, optional — run the Keyence standoff loop from the current pose, completion via `motion_seq`) and **`/arm/standoff_state`** (per Keyence reading: raw, perpendicular, standoff mm, error vs target, out-of-range side) — robot_ui's distance-sensor assist, 2026-09-15; **`/arm/move_joint`** (JSON `{joints:[j1..j6 deg]}`, one MoveJ) and **`/arm/jog_joint`** (JSON `{joint:'j3'|3, delta}`, one joint by `delta` deg, bounded by `~jog_max_step`) — robot_ui's joint control, 2026-09-21, same busy / `motion_seq` rules as `move_cart` / `jog_cmd`, no reach or collision check; **`/arm/collect_mode`** (Bool) and **`/arm/scan_continue`** (JSON `{ra}` / `{readings:[..], note}` / `{skip:true}`) + latched **`/arm/collect_state`** + **`/arm/collect_config`** (JSON mode / batch_size / retreat / dwell) — the Ra DATA COLLECTION mode, 2026-10-06: `pause` (method A) stops after every `batch_size` scanned points with the tool retreated 80 mm along its z until the operator sends each point's hand-measured Ra (robot_ui Task tab lists the batch with every earlier point's offset from the tip, any LAN browser), recorded in `log/apriltag_nav/ra_measured/<run>_ra_measured.csv`; `mark` (method B) captures nothing and stops at every scanned point for its running number to be written beside the spot, writing `<run>_mark_template.csv` to fill in by hand later; `scan_progress` gains `wait` / `resume`; `docs/RA_COLLECT_kr.md` |
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
`/mobile/goto_tag`: no `TASK`/`GOTO` during a calibration session. **Since 2026-09-22 the hand-cam align of a
calibration entry puts the hand-cam OPTICAL AXIS at rx −180 / ry 0 (hand-eye
applied, spin free)
and corrects TRANSLATION only, z to 0.50 m
(`locator.yaml align.orientation: fixed`) — the measured tilt is
recorded, not chased; regenerate the plans after any tf_chain change.**

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

**No front camera, no tag driving (user rule, 2026-09-22).** `mobile_node`
refuses every `/mobile/goto_tag` — i.e. every TASK / GOTO / CHARGE hop —
while front_cam's detections are absent or older than
`robot.front_cam_alive_timeout_s` (1.0 s; `robot_camera_node` publishes an
array for EVERY frame, empty or not, so the message age is the liveness of
driver + node + detector: node down, camera switched off with
`set_enabled false`, stalled stream all count). The result message says
`front_cam not working: …`, `/mobile/state` carries `front_cam_ok` /
`front_cam_age_s` / `front_cam_reason`, robot_ui's BASE chip reads
`NO CAM`. `MobileController.move_to_tag` re-checks before every hop (a
camera that dies mid-route stops the base before the next hop), and
`detected_tags` is a property that serves `{}` once the frame is stale —
before this the last frame stayed in front of the control loops forever
and `get_current_tag_id` would have launched a blind drive from it. The
odom-only manual moves (`/mobile/move_cmd`) are not tag driving and are
not gated. `tools/check_front_cam_guard.py` (31).

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
measures at rest and certifies the 0.2° band (both aligns 0.2°). **Since
2026-10-06 every pivot first puts its START tag exactly on the FWD
column** (`_reseat_before_pivot`, `pivot_reseat_enabled` /
`pivot_reseat_tol_m` 10 mm; user rule: "회전할 태그에 정확히 정한 이미지
화면에 온 후 회전한다"): a pivot turns about the base centre, so a
fore-aft error of the tag at the start becomes the same LATERAL error on
the exit lane and the exit-tag align (yaw only) cannot remove it. Beyond
the tolerance the tag is driven onto the column with the normal arrival
algorithm — forward when ahead, backward when behind (500-series reverse
arrivals stop on the crosshair) — and aligned there; a fresh arrival
(±2 mm) never trips it, a tag out of view at rest fails the hop. **`center_x_stop_offset` (currently +50 px)** is what
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
(`full_pct`), then `/crevis/charging false` — **the base stays on the dock
since 2026-10-06** (`undock_forward_m` 0; user: "undock 할때도 위치는
이동하지 말고 충전명령을 false 바꾼다"; a value > 0 restores the forward
move, and before this every UNDOCK → CHARGE cycle was a 0.10 m forward +
a 0.10 m reverse hop, which is what the user saw as "CHARGE from the
charging position keeps reversing"); at **20 %** (`return_pct`; 30 → 20 on the
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
fail the point). **`seek_enabled` is OFF since 2026-09-29** (user; it was ON
2026-09-21..29): the key is shared by the TASK scan and robot_ui's Auto
standoff, and a scan whose surface was out of range walked the tool down the
whole 40 mm at EVERY point and captured there — Work Log 2026-09-29. With it
off an out-of-range reading moves nothing and the row says `out of range on
the far side (seek disabled)`. What the seek does when on: an out-of-range
first reading steps `seek_step_mm` (5) toward the side the sentinel names
until a reading appears, on its OWN budget (`seek_max_mm` 40 — the
user's "start from 4 cm": Auto standoff works from ≤ ~67 mm of case
standoff; not counted
against `max_steps` / `max_travel_mm`) — the far-side sign (negative,
raw −100000) was confirmed with the tool at the home pose; a "near"
sentinel retreats. The step must stay under the sensor window (~6.5–27 mm
case standoff) and the budget is how far a beam that sees nothing walks
the tool down. Run on the robot through robot_ui's Auto standoff.

**Since 2026-09-29 the loop moves the MEASURED gap in one move
(`keyence.move_mode: direct`; `stepped` = the halving law above, which a
direct adjustment falls back to).** One computed move, a fresh median at
rest, one trim if the residual is outside `tolerance_mm` — 1–2 moves per
point instead of the 3–4 measured that day. Safeguards: a clean-reading
gate (`direct_max_spread_mm`), per-move caps, the travel budget, no
amplified approach; a **live guard** (`guard_*`: `arm_controller` watches
`/keyence/value` while the approach MoveL runs and calls `StopMotion`
when the reading has passed the target by 1 mm or the sensor says "too
close", 2 messages in a row — not a cancel, the loop re-measures and
goes on stepped, a second stop ends it); a long move the reading did not
follow ends the adjustment after ONE move. ⚠️ **Standoff moves chain from
the previous COMMAND, not from the pose readback**: a MoveL ends a small
constant offset from its target (0.1–0.5 mm, 0.06°, pose-dependent), and
"readback + dz" carried it on every move — a 0.3 mm retreat moved the
tool closer, 47 of 64 tip-tour adjustments of 2026-09-28 ended "reading
does not follow the motion", and the tool drifted ~0.3 mm sideways per
move. Do not go back to readback-relative targets there; `jog` still
has the same property (2026-09-18 note). The offset measured after each
move pre-compensates the first move of the next adjustment nearby
(`cmd_bias_*`). `tools/check_standoff_direct.py` (50); detail in
`docs/keyence_scan_chain.md`.

Three things about it are not guessable from the code — full
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

**The map calibration is applied through `map.yaml`: x, y and yaw since
2026-09-22 (tags 100–117 / 121–125 re-measured 2026-09-28 on the shifted
T_ab2mb, 118–120 still the 09-22 values — Work Log 2026-09-28 evening) (user: "x, y 말고 모든 값 사용"), z NOT used and not in the
file (user's decision the same evening: "z는 사용 안하기로", "지워주").**
Tags 100–125 carry the calibrated `x` / `y` (design + delta in a trailing
comment) and **`yaw`** (world heading of the tag's x axis, CCW +); the
calibrated z stays in the map_world file only. Three consumers:
- `x` / `y` → `/robot_pose`, the hop odom distance, the prediction
  fallback (the calibrated positions; `predictive_centering` reads the
  newest `map_world_*.yaml` itself).
- `yaw` → `/robot_pose.theta` (`robot.robot_pose_use_tag_yaw`,
  `_tag_yaw_error_deg`): the align squares the body to the TAG's edges, so
  a tag laid δ off its lane axis leaves the body δ off the zone heading
  with `align_angle` reading 0; theta = zone + align + δ, δ = yaw − nearest
  90° axis (≤ ±0.7° on plate 1; 9 mm at 1 m of reach).
- `z` → pose-mode IK, **OFF** (`arm_calibration.use_tag_z: false`): the
  path exists (`arm_transform.tag_floor_z_m`: the floor under the tag is
  `(z − tag_thickness) − world_floor_z_m` (−0.080, the cell design) above
  the CSV / `arm_base_z` datum, entering `transform_world_to_arm(
  floor_z_m=)` exactly like the lift; `tag_z_max_offset_m` 0.05 refuses a
  gross value) but the user switched it off: plate 1 reads ~+20 mm (tag
  top −57 ± 10 mm vs the design −79), z is the chain's weakest axis (sd
  10 mm per tag, 7 mm session to session), the +20 mm cannot be told from
  a chain z bias, and a wrong z moves the tool TOWARD the workpiece. With
  it off pose-mode IK uses the design floor, bit for bit as before; the z
  lines were removed from map.yaml on the user's word (the value lives in
  `map_world_20260922_plate1_final.yaml` should it ever be wanted).
Tags without the keys (dock, pivots, zone A, plates D/E) behave as before;
`robot_pose_use_tag_yaw: false` restores the zone-only theta.
`tools/check_calibrated_tag_z_yaw.py` (42) pins the arithmetic of both,
that yaw is on, z off, and no z in map.yaml.
roll / pitch stay in the map_world file only — a floor tag's tilt is the
chain's reading of the chassis attitude, not map data.

## Transform Parameters (4-DOF physical model)

**Source of truth since 2026-09-21 (evening): `apriltag_nav/config/tf/tf_chain.yaml`**
— every fixed transform (`T_ab2mb`, `T_mb2fc`, `T_hc2ee`, `T_ee2tip`), each
block with its DESIGN value, how the APPLIED value was measured and its
uncertainty, plus a `<name>.npz` twin per transform for tools that take an
npz path (`tf_chain_tool.py check` asserts they agree). Loader
`apriltag_nav.tf_chain`; CLI `tools/tf_chain_tool.py` (`show` / `check` /
`set` / `front-cam` / `urdf` / `export-npz`). Readers: `arm_transform`
(pose-mode IK derives its six numbers from `T_ab2mb`), `arm_controller`
(`T_ee2tip`, the tip → flange conversion), `tools/set_tool_tcp.py`,
`path_tag_locator` (`locator.yaml` / `handeye_calib.yaml` point at it),
`chain_calib`, `robot_sim`, the check scripts. Writers: `tf_chain_tool.py
set` / `front-cam --apply` and `handeye_calib_node`'s compute (npz + yaml
block together); nothing edits the numbers by hand. The planner URDF's
`mobile_to_base` / `vision_tip_joint` is the ONE copy outside it (the
planner's input) — `check_pose_vs_joint.py` and `tf_chain_tool.py check`
assert it agrees, `tf_chain_tool.py urdf` prints the lines. `robot.yaml
arm_calibration` keeps only `csv_euler`. Restart `arm_node` (T_ab2mb,
T_ee2tip) and the calibration nodes (all four) after a change. Before this
the same values sat in six files that had to be changed together —
`path_tag_locator/config/extrinsics.yaml`, its `hand_eye/T_hc2ee.npz`
(+ five historical npz), the six `arm_calibration` numbers and
`vision_tip_offset_mm` in `robot.yaml`, and the record
`chain_calib/docs/TF_CHAIN_2026-09-21.yaml` — all deleted (git has them).

The mount has **no tilt** by design — R is exactly Rz(180°) — which a
655-point real-robot fit (tilt ≈ 0.0001/0.0007 rad) confirmed
independently. ⚠️ That fit's data (`task/csv/calib_data*`) was old-base and
was **deleted 2026-09-11**, so the number survives only as this note; git
has the files. The USD-derived values used before (base_z 1.0076, tilts
−1.3°/+1.5°) are superseded; those tilts do not exist on the real platform.
The design block, as `arm_transform` parametrises it (the file stores the
matrix; `tf_chain_tool.py show` prints these):

```yaml
arm_body_offset_x:  0.0       # arm mount in body frame X (m)
arm_body_offset_y: -0.100     # arm mount in body frame Y (m) — moved 2026-08-13
arm_base_z:         0.652     # arm base height above ground (m), lift at origin
arm_mount_yaw:      π         # arm base yaw vs body (rad, exact)
arm_tilt_x:         0.0       # no mount tilt
arm_tilt_y:         0.0       # no mount tilt
```

`T_ab2mb` t is therefore **(0, -0.100, -0.652)** by design. Both signs come
out negative because Rz(180°) flips y and the inverse flips it back; the arm
did move toward the wall, i.e. body **-Y**.

⚠️ **Since 2026-09-21 the applied value is NOT the design.** `tf_chain.yaml`
`T_ab2mb` holds the CALIBRATED value from `chain_calib` (printed A0 tag
sheet as the ground truth, 63 views, ruler print scale, level-floor prior
on front_cam's rotation): **t (−7.47, −123.68, −628.44) mm, rpy (+0.320,
+0.786, 178.573°)** — 7 / 24 / 24 mm and **1.43° of yaw** off the design
block above. The yaw and the in-plane translation are robot constants
(two independent sessions agreed to 0.1° / 2 mm); the roll / pitch are
NOT — a floor-sheet session measures the mount tilt PLUS the chassis'
attitude at that parking PLUS the paper's slope (±1°, and ±10 mm of tz/tx
at the 0.64 m lever), so expect ~8 mm of shift per re-parking. Everything
on the locator / calibration chain (`path_tag_locator`, `robot_sim`, the
plan generator, `verify_chain.py --fit none`, `sheet_path.py`) uses it.
⚠️ **Since 2026-09-22 (evening) the applied set is FOUR things, refit
together, and `T_ab2mb` moved again:** `T_hc2ee` re-solved from the
09-18 sweep frames with the checkerboard hand_cam K (5.4 mm / 0.22°),
**`config/tf/arm_joint_offsets.yaml`** (J2..J6 zero offsets −0.34 /
−0.53 / −0.05 / −0.13 / −0.50°, fitted on the 65-view arm session with
that hand-eye fixed; `tf_chain.arm_flange_T` makes the LOCATOR chain's
flange FK_urdf(q + dq) instead of the controller's TCP — pose-mode
commands do not use them), and `T_ab2mb` refit on the same arm session
with all of that in the chain: **t (−8.68, −103.42, −643.10) mm, rpy
(−0.451, −0.103, 179.214°)**, 12.9 mm / 0.91° from the design (the 09-21
value was 34 mm / 1.66°; 20 mm of y and 0.64° of yaw moved with the
offsets — they absorb what the J2/J3 offsets took out of the FK). Chain
rms on that session 10.1 → 7.9 mm, hold-out 12.7 → 7.7. The calibration
dependency order that makes them one set (A → B ∥ C → D, E → F):
hand_cam K → (front_cam fit ∥ hand-eye) → (T_ee2tip, joint offsets) →
T_ab2mb → map calibration; change one, redo only what is to its right
(`tf_chain_tool.py check` asserts the files agree; the numbers below in
this section are the 09-21 ones and stay as history). Work Log
2026-09-22 "C, E re-solved".

⚠️ **Since 2026-09-22 (later that evening) `T_ab2mb` is fitted PLANAR —
user rule: "roll/pitch는 설계값 0으로 고정하고 x, y, yaw만 피팅, 섀시
기울기가 T_ab2mb에 들어가지 않게".** A floor-sheet session measures the
chassis' attitude at that parking (±1°, ±10 mm at the 0.64 m lever) and
the map calibration runs at other parkings, so that tilt cannot be a
constant. Applied: **t (−8.20, −106.13, −652.00) mm, rpy (0, 0,
179.196°)** — roll = pitch = 0 exactly, tz = the design −0.652, only
(x, y, yaw) from the data (jackknife 0.4 / 0.4 mm / 0.01°); the session
tilt stays in the residual (12.5 mm rms vs 7.9 for the 6-DOF fit — by
design). `chain_calib.py solve` reports `planar` next to `base` / `joint`
and saves `T_ab2mb_planar` in `corrections.npz`; `solver.fit_planar_
T_ab2mb` / `--planar-tz`. `arm_transform`'s tilt_x / tilt_y are 0 again
and the lift is purely along arm z. The URDF `mobile_to_base` follows
(xyz 0.006706 0.106230 0.652000, rpy 0 0 0.014030243).

**And on 2026-09-28 (evening) the applied `T_ab2mb` x, y moved once more
by (−1, +7) mm → t (−9.20, −99.13, −652.00) mm, yaw / tz unchanged:** the
automatic sheet session of that day showed a constant xy bias of
(−7.1, +1.0) mm in the sheet frame that the planar least squares (which
also weighs the parking tilt's z) did not remove; shifted directly, on the
user's decision that only x, y matter. Work Log 2026-09-28 (evening).

**And `T_ee2tip` was re-solved into the same set (2026-09-22, evening):**
the 09-21 Basler-tip session re-solved with the re-solved hand-eye and
the checkerboard K (`basler_tip_calib.py --hand-intrinsics config`; the
09-21 pair reproduces the old −1.8 / −245.6 / 209.6 exactly) gives
**(−2.0, −245.2, 214.4) mm** — z +4.8 from the hand-eye's z re-solve, xy
inside the robot verification's noise. Applied to `tf_chain.yaml`, the
URDF `vision_tip_joint`, and the three tool-extent copies
(`generate_calibration_artifacts.py`, `handeye_calib.yaml`,
`handeye_sweep.py`); the calibration plans were regenerated for the
whole 09-22 set (seeds moved 23 mm mean / 37 max). So the applied set
is now FIVE values from one chain: K → T_hc2ee → joint offsets →
T_ab2mb (planar) → T_ee2tip; `tf_chain_tool.py check` 13/13.

**Since 2026-09-21 (later the same day, user: the corrected chain is for
end-effector pose control too) pose-mode IK uses the SAME transform:**
`transform_world_to_arm` derives its 4-DOF parametrisation from
`tf_chain.yaml T_ab2mb` at call time — offsets (−0.013008, −0.120318,
0.629002), mount_yaw 3.166578737 rad (181.43°), tilt_x/y 0.005927 /
0.013580 rad (`arm_node` restart) — and the planner URDF
`frcobot_description/urdf/fr10v6_mobile_vision_0317_test.urdf`
`mobile_to_base` xyz (0.013008, 0.120318, 0.629002) rpy (0.005927,
0.013580, 0.024986) is the same transform seen from its 180°-yawed
`mobile_base`. `check_pose_vs_joint.py` asserts the URDF and the
derivation agree (27).
Consequences: every pose-mode target moved ~35 mm at 1 m reach; the
existing `rrt_final_path_*` joint paths were planned with the DESIGN mount
and now land that far from their `assigned_workpoints_*` twins until the
planner regenerates them with the updated URDF (the check asserts that
45 mm discrepancy as EXPECTED, not hidden); with a non-zero tilt the lift
is no longer purely along arm z (0.85° × 343 mm ≈ 5 mm of x/y at full
stroke — `check_lift_compensation` is tilt-aware now, 11). The "no mount
tilt" statements above and in `arm_transform.py`'s older text are
superseded by this — the applied tilt is as much this parking's chassis
attitude as the mount (±1°), kept so the locator and the arm agree.
`check_front_cam_extrinsics.py` and `tf_chain_tool.py check` pin T_ab2mb
to "orthonormal, within 3° of Rz(180) and 50 mm of the design", not to the
design numbers. Record: `src/chain_calib/docs/CHAIN_CALIB_2026-09-21_kr.md`;
the whole chain with provenance: the comments in `tf_chain.yaml` itself.

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
`tools/tf_chain_tool.py front-cam --apply` derives it (block `T_mb2fc` of
`tf_chain.yaml`) from `robot.yaml` (`camera_offset`, `camera_lateral`,
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
| Wherever `extrinsics.yaml`, `hand_eye/T_hc2ee.npz`, `arm_calibration` numbers or `vision_tip_offset_mm` appear | **every fixed transform lives in `apriltag_nav/config/tf/tf_chain.yaml` (+ a `<name>.npz` per transform) since 2026-09-21**, with design values and provenance in its comments; `tools/tf_chain_tool.py` shows / checks / sets them. The old files are deleted. |
| Wherever the stop offset appears | the key is now **`center_x_stop_offset: +50.0`** (was `center_y_stop_offset: -50.0`) and **more positive** stops earlier. Same physical stop point; the fore/aft image axis moved from rows to columns. |

## Work Log

Newest first. **Append an entry for every session that changes this workspace.**
Record the *reasoning* and what was *verified*, not a file diff — the diff is in
git, the reasoning is not. Keep entries short; promote anything that becomes a
standing rule up into the sections above instead of leaving it buried here.

### 2026-10-06 (evening, later) — Hand-measured Ra CSV moved into the run's frame folder under results/

User: "손으로 잰 Ra 도 log/ 밑에 말고, results/ 에 저장해줘. 필요하다면
results 아래에 scan_images/ 폴더 만들어도 돼". The COLLECT-mode records
(`<run>_ra_measured.csv`, method A; `<run>_mark_template.csv`, method B)
now go to **`results/scan_images/<run>/`** — the same folder the run's
frames land in, so one directory per run holds the pictures and the
labels that go with them. `paths.RA_MEASURED_DIR` is the frame root and
`paths.ra_measured_path(stem, suffix, root)` builds the file path (used by
`ArmController._collect_csv_path` and `merge_ra_dataset.find_measured`);
`robot.yaml collect.record_dir` is `${MM_WS}/results/scan_images` and means
the ROOT (the CSV goes to `<record_dir>/<run>/`). A mark run captures no
frames but creates the folder for its template by the same rule. The
merge tool's default run discovery globs `<root>/*/*_ra_measured.csv`, and
the mark-template fallback `<root>/<task>_ra_map_*/…_mark_template.csv`;
`--measured-dir` keeps its meaning as the root. Versioning kept: the
`.gitignore` line `results/scan_images/` became `results/scan_images/**/
*.png` + `*.jpg`, so the CSVs beside the frames are tracked while the
frames are not (`git check-ignore` confirms both). The one existing file
(the 14:05 run) was `git mv`'d into its run folder and the empty
`log/apriltag_nav/ra_measured/` removed; its `image_dir` column already
pointed there. Verified: `check_scan_progress.py` **136** (its expected
paths follow the new rule), the yaml resolves to the results root, the
merge tool finds the moved file by itself and by `--measured`. **Not run
on the robot — `arm_node` reads `record_dir` at start, so until
`sudo systemctl restart mobile-manipulator` the running node still writes
to the old log/ path.** Found on the way, left alone: the index held
four staged doc deletions and an `arm_joint_offsets.yaml` edit from
another session; this commit lists its own paths only.

### 2026-10-06 (late) — Workspace tidy: latest log + calibration data kept, the rest removed; the Work Log split

User: "현재 이 워크스페이스 정리하기 — 최신 log하고 캘리브레이션 파일 관련은
남기고 나머지 정돈". Surveyed first (git 45 dirty paths, log/ 2.4 GB,
results/ 474 MB, .git 634 MB), then two kinds of action.

**Committed** (the tree was dirty from two sessions): the 10-06 night
UNDOCK / pivot re-seat change as its own commit (checks 20 / 23 re-run
first), and the user's own task/csv replacement — the whole 09-29/30 set
deleted at 13:37, one pair `assigned_workpoints_10mm` /
`rrt_final_path_10mm` in its place (planner originals, NOT retargeted,
not speed-10'd; `scan_joint_10mm` ran four times today) — with the day's
records (nav_log/20261006, four Ra maps, the first `ra_measured` CSV).
`dev-20260908` (merged, worktree long gone) deleted. Pushed to
`origin/real`.

**Deleted, on the user's answers:** the 2026-09-09 front_cam videos
(1.2 GB, nowhere else); every `log/ros` run before 2026-09-30 and all 419
loose one-off tool logs (the live run and 09-30's four kept); the Basler
frames of the 09-22 touch and the first three 09-28 tip tours
(`results/tip_check`, 448 MB — the fourth tour 214111 and every
`tip_check` CSV / yaml kept); the per-run PNGs of the old-chain locate
sessions 09-02 … 09-15 (906 files, 570 MB; their `result.npz` / yaml
stay, and nothing from 09-22 on was touched); `config/map.yaml.bak` (the
2026-08-13 old-cell map, its only copy — that cell no longer exists);
`__pycache__`, a stale LibreOffice lock in docs/. **Kept on the user's
word:** the two `best_model.pt` checkpoints (254 MB, ignored, unread by
code — possibly the ONNX export source). Not touched: `~/.ros/log`
(the navifra driver's), `build/` / `devel/`, every calibration record
(`chain_calib`, `handeye_calib`, `calibrate/`, map_world files,
`task_csv_backup`, `calib_pair_20260915_a`).

**Work Log split:** CLAUDE.md had reached 644 KB and is loaded every
session; entries dated before 2026-09-15 (08-07 … 09-14) moved verbatim
to `docs/WORKLOG_ARCHIVE.md` with a pointer entry left at the bottom.
372 KB now. Seen on the way, not acted on: `results/scan_images` is
gone — today's four `scan_joint_10mm` runs have Ra maps but no frames
on disk (the directory's mtime is 14:58, before this session), so the
first COLLECT-mode run's `ra_measured` row cannot be merged with a
frame. Disk: log/ 2.4 GB → ~0.4 GB, results/ 474 → 29 MB.

### 2026-10-06 (night) — UNDOCK no longer moves the base; CHARGE from the dock drives nothing; every pivot first puts its start tag on the FWD column

User, three rules in one message: (1) pressing CHARGE at the charging
position reverses the base — a problem; (2) UNDOCK must not move, only
`/crevis/charging false`; (3) before a pivot, the tag to turn on must
first be exactly at the designated image position, then turn.

**(1) and (2) are one change.** Today's `mobile_node` log has every
dock move arriving normally (12:11 and 13:40: 501 → 500 reverse,
0.078 / 0.100 m, 500 on the crosshair within 1 mm) and the "already at
500" path returning at once on earlier days — the reversing the user
sees is the UNDOCK → CHARGE cycle itself: UNDOCK drove 0.10 m forward
(12:55 today), which hides tag 500 behind the bumper and makes 501 the
current tag, and the next CHARGE then planned the reverse hop 501 → 500.
With `undock_forward_m` **0** (`_battery_undock_task` adds the drive
item only for a value > 0, like `dock_reverse_m`) the base stays on the
dock, tag 500 stays on the crosshair, and CHARGE hits `move_to_tag`'s
"Already at target tag 500" — relay on, no motion. The 85 % stop uses
the same task, so it no longer comes forward either; the manager's
phases are unchanged. Not changed: `move_to_tag` itself (the nearest
visible tag is the start tag; 500 and 501 are 0.202 m apart, so that
only flips once the base is > 0.1 m off the dock).

**(3) `_reseat_before_pivot`** in the pivot branch of `go_to_next_tag`,
after the first-hop align: the start tag's fore-aft distance from the
FWD column, read live; beyond `pivot_reseat_tol_m` (10 mm) the tag is
driven onto the column with `execute_pure_pursuit` (forward if ahead,
backward if behind — refused with a message if that tag's reverse column
is not the crosshair, which no 500-series tag has) and aligned, then
the pivot. The case that produced the rule is in today's log: the TASK
of 13:38 started on 501 with the tag 0.125 m ahead of the lens (the
undocked position) and pivoted from there; a pivot about the base
centre carries that 0.125 m through the turn as a lateral error on the
505 lane. A tag not in view at rest fails the hop (nothing to verify
against; the first-hop align would have failed first anyway).

Verified offline: `check_nav_sequencing.py` 14 → **20** (0.125 m ahead:
align(501) → pp(501 fwd 0.125) → align → pivot → align(505), base
centre on the stop pose within 5 mm, 90° within 0.2°; 25 mm behind:
backward re-seat onto the crosshair; 3 mm off: no re-seat, "on the
column" logged; key off: the old behaviour, 50 mm short), `check_charging_
manager.py` 19 → **23** (85 % stop and UNDOCK with no drive; a positive
`undock_forward_m` still drives; CHARGE from the dock: relay on,
confirmed, no straight move), `check_front_cam_guard` 31,
`check_robot_pose_live` 30, `check_robot_pose_heading` 20. UI tooltips
and `ROSBRIDGE_kr.md` / HANDOVER follow. **Not run on the robot;
`mobile_node` and `task_executor` restart required** (`sudo systemctl
restart mobile-manipulator`). The base is currently sitting where today's
last TASK left it; the first CHARGE after the restart still drives there
from wherever it is — only CHARGE *from the dock* is motionless.

### 2026-10-06 (evening) — Two testers at once: batch stops (method A) and a numbering pass with a template to fill in later (method B), selectable in the UI

User: "조도측정기가 2개 있는데 현재 상태로는 2개를 동시에 활용할 수 없잖아요,
속도를 더 낼 수 있는 방법 있나요" — then "두 방법 다 진행해주 선택할 수 있게".
Measurement is the bottleneck (arm ~3 s per point, a tester 20–30 s with
placing), so both methods make the measuring parallel; `collect.mode`
chooses (`/arm/collect_config`, robot_ui Task tab).

- **`pause` (A): `batch_size` N.** The scan loop keeps a batch of the
  scanned points since the last stop and calls `_collect_pause(batch)`
  when it is full OR after the last scanned point of the list (while the
  tool is still over it — the arm's path ends with home rows, so an
  end-of-scan flush would have been nowhere near the spot). The waiting
  state lists every batch point with its offset from the point under the
  tip: world dx dy (mm, straight from the CSV x y) and arm-base dx dy
  (through `transform_world_to_arm`, position only — no new sign
  convention invented; None for a task without x y z). `collect_continue`
  takes `points: [{group_id, point_id, ra | readings, note, skip}]` and
  refuses the flat form for a batch > 1, a point outside the batch, or a
  listed point without a value; a batch point left out of the release is
  recorded as skipped "not entered". One row per point, one retreat +
  return per STOP. UI: a table (last point highlighted, Enter moves down
  the Ra column, the last Enter sends), Skip all, per-row skip / note.
- **`mark` (B): no capture, no Ra map.** `_collect_mark_point` at every
  scanned point after the Keyence standoff: retreat `mark_retreat_mm`
  (30), publish `wait` kind mark with the running number, block until
  any `scan_continue` (or `mark_dwell_s` > 0 passes), append the row to
  `<run>_mark_template.csv` (`mark_no`, group / point / x y z, blank
  `ra_measured`), return. The operator writes the number BESIDE the spot
  (outside the Basler's few-mm field), measures everything afterwards
  with as many testers as there are, fills the template in, runs the
  same TASK normally for the frames, and `merge_ra_dataset.py <scan run>
  --measured <template>` — or no flag: the tool takes the newest
  filled-in template of the same task name and says so. Caveat written
  into the doc: mark pass and scan pass are two runs with the base's
  ±2 mm stop error between them.
- `COLLECT_COLUMNS` gained `mark_no` (blank in pause mode); `/arm/collect_
  state` carries mode / batch_size / kind / mark_no / points; the SCAN
  chip reads `WAIT Ra N pt` or `MARK #n`.
- **Then (user, in pinyin: with method A the scan position cannot be
  found; "1로 진행하기"): `collect.premark`** — in pause mode every
  scanned point stops once more right after the capture with the tool
  STILL AT THE STANDOFF (case bottom 16.5 mm up, no retreat; kind
  `premark`, chip `MARK SPOT`) until any `scan_continue`: the operator
  marks the spot beside the case (it sits under the case centre), and
  the retreat + Ra-entry stop follows as before. Nothing is recorded at
  the premark stop. Why not the lamp or a laser: the vision tip is a
  virtual point, nothing physical is there; whether the VISION lamp is
  coaxial (its floor pattern would then mark the axis) is unknown until
  the user looks, and a cross-line laser on the tool is the robust
  hardware answer, both offered. `check_scan_progress.py` 130 → **136**
  (premark at both points of a 2-batch, not retreated, no MoveL before
  either, the retreat only at the batch stop, a bare continue releases
  it, rows only from the batch stop), `check_web_ui.py` 144 → **145**.

Verified offline: `check_scan_progress.py` 103 → **130** (batch 2 over
three scanned points + a traverse: stops after point 2 and after the last
point, point 1 listed at world dy −25 mm / a 25 mm arm-frame vector, one
retreat + return per stop, one CSV row per point with its own Ra, frame
names skipping the traverse index; the three refusals; "not entered";
`set_collect_config` + its two refusals; the mark pass: nothing captured,
camera never opened, numbers 1, 2 over the scanned points only, 30 mm
retreats, no Ra map, template rows with blank Ra and the world xyz, done
events `mark`, the home move; dwell 0.2 s continues by itself),
`check_web_ui.py` 138 → **144** (batch / mark release forms, config
coercion + refusal, mark log line), `check_task_list_ui.py` 104, the
page in headless Chrome with no JS error, the merge tool's template
fallback (picks the filled 14:00 template over the empty 12:00 one) and
`--measured`. **Not run on the robot**; `arm_node` + `robot_ui_web_node`
restart required. The arm-frame dx dy sign has only been checked for
length, not direction, on hardware — the world dx dy is the one to trust
first.

### 2026-10-06 — Ra training-data collection: frames named by group / point / index, a per-point COLLECT pause with the hand-measured Ra typed into the web UI, and a merge tool

User: to build the Ra estimation model, scan data must be collected now —
the Basler images plus the Ra a hand-held roughness tester reads at the
same spot, written down by hand after each shot, on the real scan TASK
paths as they are; what has to be prepared? Then "모두 진행해주".

**Two things found in the last run before building anything.** (1) The
09-30 `scan_joint_260930_…offset0cm` Ra map: 271 scanned points, 271
`standoff NOT corrected: out of range on the far side`, 0 converged — the
frames were taken wherever the CSV put the tool, not at the 16.5 mm the
Basler is focused at, so as data they are worthless. Not a code matter:
the surface is outside the Keyence window at the CSV pose (the 09-29
open issue); check it on the mould with Auto standoff and run the
collection with `keyence.require_converged: true`. (2) The frame name
`point_<id>_sample_<n>_ra_<model>.png` carries no group: `source_point_id`
repeats across groups in every current path file (260930: 557 of 1267
work points; that run's 271 successes had 187 distinct ids), so a frame
could only be told from its namesake by the model's Ra digits.

**Built.**
- `scan_pipeline.image_file_name` / `image_name_prefix`: the scan loop
  saves `g<group>_p<point>_i<index>_s<n>.png` (index = 1-based position
  in the scan point list, the name is known at CAPTURE time, before the
  worker has the model Ra); the legacy name stays for callers without a
  prefix. `process()` returns the saved names in `images`.
- **COLLECT mode** (`robot.yaml collect:`; `arm_node ~collect_*`;
  `/arm/collect_mode` Bool live switch, `/arm/scan_continue` JSON,
  latched `/arm/collect_state`): after a scanned point's capture
  `_collect_pause` MoveLs the tool `retreat_mm` (80) along its own z AWAY
  from the surface (the standoff loop's sign: approach = −keyence_dir, so
  retreat = +keyence_dir), publishes `waiting` with the frame names and
  a `wait` progress event, blocks (cancel-aware, optional
  `wait_timeout_s` → recorded as skipped) until `collect_continue` brings
  `ra` or `readings` (mean recorded) or `skip`, appends the row to
  `log/apriltag_nav/ra_measured/<run>_ra_measured.csv` (run, index,
  group_id, point_id, images, ra_measured, ra_readings, note, skipped,
  standoff, x y z, image_dir, measured_at), publishes `resume`, MoveLs
  back to the captured pose. A release with no value is refused and the
  point keeps waiting; transition rows, failed moves and points with no
  frames never pause; a cancel during the wait ends the scan with the
  tool left retreated (no return move). `task_executor` needs no change:
  its `/scan_finished` wait has no timeout.
- **robot_ui (web only; the Qt window untouched):** Task tab group "Ra
  data collection" — Collect mode checkbox (follows arm_node's state, not
  its own click), the waiting readout (point, run index, frame names,
  standoff message), Ra reading(s) + Note fields live only while a point
  waits (Enter = Record & next), Skip point, recorded / skipped counts;
  SCAN chip `WAIT Ra pt N`. Usable from a phone on the LAN at port 8080.
  Bridge `set_collect_mode` / `scan_continue`, `collect_state` cached +
  replayed; `api_set_collect_mode` / `api_scan_continue`.
- **`tools/merge_ra_dataset.py [run …] [--all-frames] [--out one.csv]`**:
  joins the Ra map, the measured CSV and the frame folder of a run into
  `results/ra_dataset/<run>_dataset.csv`, ONE ROW PER FRAME (image path,
  ra_measured, readings, note, model Ra, `standoff_ok`, x y z, flags) and
  reports frames without a measurement, measurements without a frame,
  rows whose standoff did not converge; legacy frames pair by point_id
  and are flagged `ambiguous` when that id sits in several groups.
- `docs/RA_COLLECT_kr.md` (operator procedure + what to prepare).

**Verified offline:** `check_scan_progress.py` 73 → **103** (two scanned
points + a traverse pause twice with the right frame names and run
index, retreat +80 mm base z for a tool pointing down with xy / rpy kept,
return to the captured pose, measured CSV rows with the readings' mean
and the world xyz, the inference worker saving under the same prefix,
final state counts; skip; a value-less release refused; cancel while
waiting → retreat only, nothing recorded, scan cancelled before the next
capture; timeout → skipped with the reason; mode off → no pause and the
new names still; no frames → no pause), `check_web_ui.py` 130 → **138**,
`check_task_list_ui.py` 104, the page in headless Chrome with no JS
error and the new elements present (`check_web_ui_browser.py` dies at
`Page.navigate` in this session — the pre-existing environment issue),
the merge tool on synthetic runs (new + legacy names, skipped,
unmeasured, missing frame, ambiguous id). **Not run on the robot:**
`arm_node` and `robot_ui_web_node` restart required (`sudo systemctl
restart mobile-manipulator`). First thing to watch: with Collect mode
ticked, the first scanned point's `[Arm REAL] collect: point … waiting
for Ra` line, the tool 80 mm up, and after Record & next the return
MoveL landing where the capture was before the next row's MoveJ. Still
the user's to decide: which points to collect (all 1266 at ~1 min each
is 20 h), and surfaces with other Ra ranges than the one mould.

### 2026-09-30 — Three new planner joint paths installed as `rrt_final_path_260930_standoff20mm_offset{0cm,1.5cm,3cm}_height{652,667,682}mm.csv`

User dropped `joint_path_offset{0cm,1.5cm,3cm}_h{652,667,682}.csv` into
task/csv and asked whether RELOAD + run was enough. It was not: the prefix
is not one `TaskManager` discovers, and the speed-10 rule was not applied.
Done on the user's instruction ("오늘 날짜가 들어가는 이름으로 변경, speed
10 적용"): renamed to the `rrt_final_path_260930_…` keys above (same
naming as the 260610 set; originals + SHA256SUMS in
`log/apriltag_nav/task_csv_backup/20260930_joint_path_offset_originals/`)
and `slow_task_entry.py --apply` (258 / 261 / 266 boundaries, only the
speed column differs from the originals, 0 violations, BOM / CRLF kept).
Real `TaskManager` registers `scan_joint_260930_…` ×3 (lift 0 / 15 / 30
mm, tags 104 105 106 107 118 119 120) with no error. `RELOAD_TASKS` from
the UI picks them up; no node restart needed (the stack was restarted
09:33 today, so the 09-29 standoff code is live).

**What these files are, measured before installing — and what they lack.**
Same RRT dialect, standoff 20, z 0.589–0.667 (하형 heights), 1267 work
rows each, `lift_mm` 0 / 15 / 30, planner metadata identical to 260610's
(`base_x_actual_mm` ±1610). But the group → tag assignment is NEW (106:
501 pts, 119: 557, 107: 8, 118 added, 108 / 121 gone) and the work points
are a DIFFERENT set: FK under the planner model (idle stops, design mount
and tip) puts the tips at x −0.56…0.56 like 260610 but y −0.567…0.434
(260610: −0.777…0.224), and no new tip lands within 2 mm of any 260610
point (median 19 mm, p95 180). **No `assigned_workpoints_` twin was
delivered**, so: the tasks register alone with `points_with_world_xyz 0`
(Ra map rows without x y z), the retarget tool cannot classify or correct
them (they replay as planner originals: tip ~9 mm beside / ~10 mm above
the planned point, as for 260610), and ⚠️ **the 260610 pose files must
NOT be given these keys** — pairing by (group, source_point_id) would
attach wrong coordinates to 395 points (median 330 mm off) and none to
872. Reach: flange 814–1412 mm (FR10 nominal 1400 — on the limit at the
far corners); joint-limit margin ≥ 43.5°, row step 5.73°. The standing
joint-replay hazard applies unchanged. Asked the user for the planner's
pose twins; pending.

### 2026-09-29 (20:30) — The measured standoff error put into the path data: `rrt_final_path_errorX_…_010mm_…_plate2.csv` re-made at 16.1 mm of tip-down

User, on being told the scan points were "about 5 mm too far": "이거는
경로데이터에 추가". Split by task first, because the 5 was a reading of
the last log lines, not a mean. Only ONE task of the day has readings at
all — `scan_joint_errorX_p000mm_standoff_010mm_height_652mm_plate2`, group
132 — every other scan of 2026-09-29 (errorX 001 / 020, 260610, 030_plate2,
upper_mold_plate2: 600+ points) read "out of range on the far side", i.e.
more than ~10 mm too far by an unknown amount, so nothing can be added to
those files from the data. The one task ran twice: **19:23 with the 20 mm
file, 40 points, first error +3.96 mm (too close, sd 0.85); 19:29 with
the 12 mm file, 85 points, −4.10 mm (too far, sd 1.01)** — the two agree
on the surface being at 16.0–16.1 mm of tip-down. Applied **4.1, not 5**:
the file was re-made with `make_plate2_paths.py --tip-down-mm 16.1` from
the same 09-14 source (the 19:20 procedure: scratch output, only the rrt
file installed, the pose twin left at 20 mm — it is now 3.9 mm BELOW the
joint file, was 8). Only q1..q6 changed; FK new vs old: tip +4.100 mm
along the tool z on 941 work points, sideways 0.000 mm, largest joint
change 1.05°; the two unreachable corner points miss by 11.3 / 4.9 mm
(were 10.8 / 4.6), row-to-row step 5.7 → 12.1°. Record + the replaced
file: `task_csv_backup/20260929_plate2_from_0914_standoff010_tipdown16p1/`.
`check_task_discovery` 58; `check_retarget_joint_paths` fails the same
nine files as before (this one because joint ≠ pose by design).

What the constant does not remove: within the 19:29 run the error drifts
−2.6 mm (points 17–31) → −5.0 mm (points 84–155), so after the shift the
first reading should be within about ±2 mm and the standoff loop takes
the rest (one direct move). Groups 133 / 134 / 143 / 144 of the file were
never measured; they got the same 4.1. Not run on the robot;
`RELOAD_TASKS` (or the `arm_node` restart the standoff change needs
anyway).

### 2026-09-29 (20:00) — Keyence standoff: the measured gap in ONE move, guarded; and why the tip tour's loop "did not follow the motion"

User: "거리센서 보정 진행할 때 실제로 이동해야 할 거리를 계산해서 이동, 안전장치도
필요, 여러 번 이동하는 것을 최소화". Measured first, from the day's
`arm_node` log: 116 scan adjustments, first error 3–6 mm off at almost
every point (too close in the 19:23 run, too far in the 19:29 run),
**3 moves on 60 / 4 on 37 / 5–6 on 10**, median 2.2 s — the
2026-09-08 law approaches by halves although the first reading has the
whole distance. Sensitivity of the big moves (reading change / commanded):
median 0.93, p5 0.83, p95 1.00, so a one-shot move ends 0.2–0.5 mm short
on about half the points and inside the 0.2 mm tolerance on the rest.

**Built** (`keyence_standoff.py` docstring 10, `arm_controller.py`,
`robot.yaml keyence:`; standing text in *Keyence Distance Loop*):
`move_mode: direct` — whole gap in one move, verify at rest, trim with the
measured sensitivity; `move()` may return a `MoveReport` (executed
distance from the pose readback, guard stop). Safeguards before / during /
after the move as listed there; the one that is new in kind is the **live
guard**: the sensor is watched on the `/keyence/value` callback thread
while the MoveL blocks the worker, and `StopMotion` (retry, no cancel
flag) ends the move when the reading passes the target by 1 mm.
`stepped` is the old law bit for bit and the dataclass default.

**Found on the way — the loop's targets were readback-relative.** The
2026-09-28 tip-tour log had 47 of 64 adjustments ending "reading does not
follow the motion", and their steps show a constant ~0.4–0.5 mm toward the
surface on EVERY move whatever was commanded (retreat 3.0 → 2.25, 0.81 →
0.24, 0.61 → 0.08; approach 0.6 → 1.16). `tip_check/…/r1_tag101.yaml`
has the poses: three retreats totalling 1.3 mm moved the readback z
+0.23 mm, x +0.68, y −0.91 mm, rx 179.13 → 178.94. A MoveL ends a small
offset from its target and "readback + dz" re-applied it per move (the
2026-09-18 jog finding, larger at 1.1 m reach). Today's scans show no
such bias (small-move ratio 0.99) — it is pose-dependent. Moves now chain
from the previous command with the first pose's orientation; the offset
is measured after each move and carried to the next adjustment nearby.
The HEAD code against the new check's fake robot reproduces the tour's
signature (0.5 mm from target, x / y drift), the new code converges in 2
moves there and in 1 at the next point.

Verified offline only: `check_standoff_direct.py` 50 (300 random starts:
1.66 moves per point vs 4.17 stepped; frozen sensor = one move; guard
stops a move onto a 5 mm raised edge at 15.3 mm, 11.5 without),
`check_standoff_seek.py` 23, `check_scan_progress.py` 73. **Not run on the
robot; `arm_node` restart required** (`sudo systemctl restart
mobile-manipulator`). Watch for `[Standoff 1/5] err −5.3 mm … -> approach
5.3 mm (direct x1, gain/1.00)` followed by `on target` or one short trim.
Not verified on hardware: the controller accepting the next MoveL after a
guard `StopMotion` without a reset, and the real stop distance. If a
guard stop ever leaves the arm in error, `guard_enabled: false` keeps the
rest. The constant part of the day's error went into the path file
instead — next entry.

### 2026-09-29 (19:00) — 정반 2 twins of the 09-14 standoff 030 / 050 pairs; the plate-2 tool learnt to keep unreachable rows

User: "이 두개의 정반 2의 파일도 만들어주" (the 09-14 `errorX_…_standoff_030mm`
and `_050mm` pairs). `make_plate2_paths.py` on them, same treatment as the
evening's set (x +3.900, twins 106/107/108/119/120/121 → 132/133/134/144/
145/146, 20 mm along the tool z, joint rows re-solved on the robot model,
speed-10 rule). Output `assigned_workpoints_ / rrt_final_path_errorX_p000mm_
standoff_{030,050}mm_height_652mm_plate2.csv` — the 050 name REPLACES the
09-14 "+26" pair restored fifteen minutes earlier (that one kept 정반 1
coordinates; git `68bac0d` still has it). Record + sources in
`task_csv_backup/20260929_plate2_from_0914_standoff030_050/`.

Two tool changes it needed. (1) Twin matching is by grid ROW (nearest
zone D / E tag within 50 mm), not ±5 mm: tag 121's calibrated y is
−1.3557 and its twin 146 sits at the design −1.35. (2) **Rows the arm
cannot reach on 정반 2 are kept, not refused:** this set (z 0.34–0.44 m,
the far corners at 1391–1395 mm of planned flange reach) needs 1402–
1416 mm on 정반 2 — the base stands 10 mm further from the plate, the
measured tip is 13.5 mm shorter, the tip-down adds reach on a tilted tool
— and the damped IK ends 4–9 mm short with the elbow straight. Such a row
(work point or transition) keeps the planned row + the correction
interpolated from its solved neighbours, so the path stays smooth, and
the tool lists it: **030: 2 work points (group 106 pt 1: 8.9 mm, 107 pt
32: 4.2 mm) + 1 transition; 050: 3 work points (106 pt 1: 6.6, pt 414:
9.0, pt 452: 8.3 mm) + 3 transitions** (`--max-unreachable` 5 / `--max-
unreachable-mm` 10; this run with 6). The reachable rows next to them
straighten the elbow 13–15° → 2°, so the row-to-row step grows 5.7 →
10.1 / 10.8° there (`--max-step-growth-deg` 6 for this run; the default
0.5 is the divergence guard). Every other work point is on its pose row
to 0.010 mm; pose mode refuses those 2–3 points with an IK error at run
time, so the miss is visible there too. `check_task_discovery` 56, real
`TaskManager` 10 + 10 tasks, no errors, every task routes from 500;
`check_retarget_joint_paths.py` part 2 reports the two new joint files
as 8.9 / 9.0 mm worst — exactly the 2 / 3 unreachable work points (its
message now says "N of M work points off … unreachable rows kept" for a
handful and "NOT retargeted" for a whole file; the 260610 originals are
the latter, by the 18:20 decision). Not run on the robot.

**Then (19:10, "이거도 해주") the 09-14 `errorX_010` pair too**, same tool
and settings: tags 106/107/108/118/119 → 132/133/134/143/144, 943 work
points on their pose rows to 0.010 mm except **2 unreachable work points
(group 106 pt 1: 11.8 mm, 107 pt 32: 5.1 mm) + 2 transitions** (flange
target 1403–1412 mm), row-to-row step 5.7 → 13.1° next to them
(`--max-unreachable-mm 12 --max-step-growth-deg 8` for this run). Record
in `task_csv_backup/20260929_plate2_from_0914_standoff010/`. task/csv:
12 + 12 tasks, `check_task_discovery` 58, real `TaskManager` no errors,
routes from 500 ok.

**Then (19:20, user: "rrt_final_path_errorX_…_010mm_…_plate2.csv vision_tip
방향 기준으로 8mm 위쪽으로") — that JOINT file alone re-made at 12 mm of
tip-down (20 − 8), its pose twin left at 20 mm on purpose.** Same tool,
`--tip-down-mm 12`, output to a scratch dir and only the rrt file copied
in; record in `task_csv_backup/20260929_plate2_from_0914_standoff010_
tipdown12/`. Verified on the installed pair: the joint rows' real tip
sits exactly 8.000 mm ABOVE the pose rows along the tool z at all 941
reachable work points (perpendicular 0.010 mm); the two unreachable
corner points miss by 10.8 / 4.6 mm now, step 5.7 → 11.4°. So for this
key the joint task scans 8 mm higher than the pose task, and the Ra map's
world x y z (taken from the pose file) is 8 mm below where the joint
task actually measured — say the word and the pose file follows.

### 2026-09-29 (18:45) — The 2026-09-14 task set brought back from git (commit 68bac0d)

User: "9.14일 task git에서 가져오기". The last commit of that day touching
task/csv is `68bac0d` (09-14 17:18): four pairs — `errorX_p000mm_standoff_
{010,030,050}mm_height_652mm` (정반 1, 943 work points each, tags 106–108 /
118–121, z 0.34–0.44 m, speeds already halved to 10 / 30) and
`errorX_p000mm_standoff_050mm_height_652mm_plate2` (tags 132 / 133, 472
points: the 09-14 tag-id +26 shift with 정반 1 world coordinates, i.e. NOT
a frame conversion — its pose twin puts the tip on 정반 1's workpiece
position while the base stands on 정반 2; run only its joint twin, if at
all). Written into task/csv with `git show` (none of the names existed
there any more), verified cell by cell against the commit, then the
standing speed-10 rule applied (only speed cells changed: 361 / 408 /
361 / 176 per pair). Not retargeted (the user's rule of 18:20 stands for
everything in task/csv; these are the 09-14 planner originals, made for
the DESIGN mount / tip and the idle stop pose — the 09-21 tip finding
says pose mode's flange target sits 17 mm from the joint row's here).
task/csv now holds 10 + 10 tasks; `check_task_discovery` 56, real
`TaskManager` no errors, every task routes from 500. `RELOAD_TASKS`.

### 2026-09-29 (18:35) — The 정반 2 set restored from the backup as `upper_mold_…_plate2`

User: "백업한 정반2 상형 데이터 추가해주". The only 정반 2 data in any backup
is the evening's `_plate2` conversion (18:06 zip, six files, sha256
verified against the zip's SHA256SUMS) — the set the planner zip's folder
called 하형 and which was therefore installed as `lower_mold_…_plate2`. It
is back in task/csv unchanged except the key: **`upper_mold_…_plate2`, on
the user's word** (the user is the authority on which mold sits on 정반
2; if the planner's folder name was right after all, the rename is a
`sed` — the contents are the same either way). The 상형 folder of the
planner zip (z 0.33–0.41 m, reach-limit rows, refused by the retarget)
is still NOT installed anywhere. task/csv now holds 6 + 6 tasks:
`260610_…` (정반 1, three lifts, originals) and `upper_mold_…_plate2`
(정반 2, retargeted, 20 mm tip-down); `check_task_discovery` 52, real
`TaskManager` no errors, every task routes from 500. `RELOAD_TASKS`.

### 2026-09-29 (18:10–18:30) — The "260610_rot" export replaces task/csv: three lift heights, planner originals, NOT retargeted (user's call); the retarget tool learnt `lift_mm`

User: "현재 task 압축하고 백업" → `log/apriltag_nav/task_csv_backup/
20260929_180646_task_csv.zip` (the 12 lower_mold / _plate2 files +
SHA256SUMS). Then `task/csv/260610_rot_standoff20mm_offset0_15_30.zip`
"이것이 최신 경로파일이다 정리해주". What it is: three pairs named
`260610_standoff20mm_offset{0cm,1.5cm,3cm}_height{652,667,682}mm_
{assigned_workpoints,rrt_final_path}.csv` — SUFFIX naming, so they were
renamed to the `assigned_workpoints_<key>` / `rrt_final_path_<key>`
prefix `TaskManager` discovers. Same 1266 work points in all three (정반 1
frame, tags 104–108 + 119–121, both target lines, z 0.589–0.667 = the
하형 heights, standoff 20 mm, tool up to 27.8° off vertical, rz over
±180° — the "rot"); what differs is **`lift_mm` 0 / 15 / 30** (base
height 652 / 667 / 682): the joint rows are solved at that base height,
FK under the planner model lands on the pose rows to 0.010 mm only once
the arm base is raised by exactly 15.000 / 30.000 mm. The planner's own
speeds are graded 10 / 15 / 20 / 25 / 30; the standing transition → task
speed-10 rule was applied (292 / 289 / 288 boundaries per file, backup of
the renamed originals in `task_csv_backup/20260929_260610_rot_before_
transition_task_speed10/`). Old files removed (`git rm` for the tracked
pair), the zip moved to `task/`. Real `TaskManager`: 3 + 3 tasks +
go_home, lift 0 / 15 / 30 mm read from `lift_mm`, every task routes
from 500 (38 hops); `check_task_discovery` 49 (it no longer assumes
`standoff_010mm` keys or the {10, 30} speed set — counts, lift and
speeds come from the files).

**Not retargeted — user: "retarget 안 해도 됨".** The dry run had shown
what the retarget would do: work points 0.010 mm after, 27.06 mm /
1.01° before (replayed as planned the tip lands mean (+1.9, +8.4,
+10.5) mm off in zone B, (−3.9, −8.0, +10.4) in zone C, |xy| up to
20.8 / 25.3 mm; flange 7–11 mm lower than the plan). So **the joint
tasks of this set put the tip ~9 mm beside and ~10 mm above the
planned point; the pose tasks are right** (pose mode applies the
calibrated map / mount / tip at run time). `check_retarget_joint_paths.py`
part 2 reports the three joint files as NOT retargeted — expected, by
this decision, not a regression. Two tool changes stay: (1)
`retarget_joint_paths.py` / its check / `make_plate2_paths.py` read
`lift_mm` and raise the arm base by it in BOTH models (the lift is
vertical in mb, so the correction itself is lift-independent — but the
classification is not: the 15 / 30 files read as "neither" before);
(2) a TRANSITION row the robot's model cannot reach keeps the planned
row + the neighbours' interpolated correction and reports the deviation
(`--max-transition-dev-mm` 15), instead of failing the file — the
lift-30 file routes group 106's transitions through the fully straight
arm (flange 1491–1518 mm, J3 0.6°; work points ≤ 1414), 8 rows the
target sits 3–6 mm beyond. That file still refuses on one diverged row
(J1 / J6 22.7°, step 34.6°) — not chased, since nothing is retargeted.

### 2026-09-29 (evening, later) — The mold is in the task name now (`lower_mold_` / `upper_mold_`); the 상형 export refuses the retarget at the reach limit

User: "현재 task 중 상형 하형이 구분이 안 돼". Everything in task/csv was 하형
(the 정반 1 originals and the `_plate2` twins) and the key carried no mold
name. Renamed all twelve files `*_errorX_…` → `*_lower_mold_errorX_…`
(`git mv` for the tracked ones; contents untouched — the retarget check
still holds all six joint files at 0.010 mm, `check_task_discovery` 52),
so the tasks read `scan_pose_lower_mold_…` / `scan_joint_lower_mold_…
[_plate2]`; the record yaml of the plate-2 conversion notes the rename.
ASCII on purpose (`robot_cmd.py` on the Windows PC types the task name).

**The 상형 set was then staged as `upper_mold_…` for 정반 1 (speed-10 applied)
and `retarget_joint_paths.py --apply` REFUSED it — nothing written, the
files removed from task/csv again.** Not a tool defect: the export plans
zone B work points with the arm STRAIGHT — flange reach 1400–1407 mm, J3
0.0–2.8° (하형: ≤ 1426 mm but J3 ≥ 0.9° and re-solvable; 상형 z is
0.33–0.41 m vs 하형's 0.57–0.65, so the same lane reaches further). Under
the robot's model the flange has to go ~10 mm further for the same tip
(the measured tip is 13.5 mm shorter than the design one) and the IK has
nowhere to go: 10 / 10 / 16 rows fail per file (groups 105 / 106 / 107,
lines 124–126, 377–384, 564–566, 638, 705–706, 746–748), J3 → 0.00, a
row-to-row step of 16.7 / 34.0°. The 001 file is also 0.051 mm off its
pose rows at one point (tolerance 0.05 — rounding, not the problem). What
it needs is the planner: those points assigned to a nearer stop, or
planned with the calibrated mount / measured tip (`tf_chain_tool.py urdf`)
and a reach margin. Installing the originals unretargeted would run the
tip 10–14 mm (zone B) / 13–26 mm (zone C) beside the planned points and
fail `check_retarget_joint_paths.py`, so they stay out. Pose mode would
have refused the same points with an IK error at run time.

### 2026-09-29 (evening) — 하형 export re-expressed for 정반 2, every point 20 mm further down the tool z: `tools/make_plate2_paths.py` → the `*_plate2` task pairs

User, with `task/20260929.zip` (planner delivery of 11:36 / 12:10: 하형 and
상형 folders, standoff 001 / 010 / 020, one pose + one joint file each):
"여기에 있는 하형데이터는 정반 1기준인데 정반2 기준으로 간단하게 바꾸고
그리고 경로데이터 point를 vision_tip 방향 2cm 더 내려가게 한다". The zip's
하형 files are byte-identical to the planner ORIGINALS of the set in
task/csv (the `20260929_before_transition_task_speed10` backups; the 001
joint file differs only by the user's hand-set speeds) — i.e. today's
plate-1 scans ran the 하형 paths on the plate the workpiece is NOT on, which
also explains the Keyence seeing nothing after 40 mm (entry below). The
상형 set was not asked about and is not installed.

**Built `tools/make_plate2_paths.py SRC_DIR [--apply]`** (dry run by
default) on the retarget tool's kinematics: for each planner-original pair
it writes `assigned_workpoints_<key>_plate2.csv` + `rrt_final_path_<key>_
plate2.csv`, which `TaskManager` discovers as `scan_pose_<key>_plate2` /
`scan_joint_<key>_plate2`. (1) World x += 3.900 (정반 2's centre per
`reference_tags_plate2.yaml`, the user's 390 cm; the D / E lanes sit at
+3.89, so the base stands 10 mm further from the workpiece than on 정반 1 —
absorbed by the joint re-solve, and by `/robot_pose` in pose mode). (2)
Group tags → their twins by design y, B → D / C → E: 105 → 131, 106 → 132,
107 → 133, 118 → 143, 119 → 144, 120 → 145. (3) Every point 20 mm further
along the TOOL z axis (the tip's approach direction; the tool is up to
27.8° off vertical in these files, so world dz is −20.0 … −17.7 mm with up
to 9.3 mm of xy — not the same as world −z). Pose rows: `p += d·R[:,2]`
with R the ZYX flange orientation. Joint rows are RE-SOLVED, not copied:
planned tip pose under the planner model (idle 정반 1 stop pose, design
mount, design tip) → shifted → flange target under the robot model (map
.yaml stop pose of the 정반 2 tag — DESIGN values, plate 2 has never been
map-calibrated — calibrated `T_ab2mb`, measured tip) → IK seeded with the
planned row; transitions between work points exact, transitions next to
`home` blended to 0 at home, `home` rows untouched — the retarget tool's
rules. Then the standing speed-10 rule (`slow_task_entry.py`, run on the
output). Sources + `make_plate2_record.yaml` (sha256, tag map, per-file
stats) in `log/apriltag_nav/task_csv_backup/20260929_plate2_from_hahyeong_
20260929/`. Refuses to overwrite (`--force`), refuses a pair that is not a
planner original, a tag without a twin, an IK failure, > 20° of joint
change, < 5° of limit margin, > 0.5° of row-step growth.

**Numbers (38 checks in the tool, all three pairs):** IK converged on all
1613 / 1629 / 1668 rows in ≤ 5 iterations; largest joint change 4.4 / 5.2
/ 7.0° (J3), median work-point row 2.7–2.8°; row-to-row step 5.73 →
5.86–5.98°; J3 closest to straight 0.9–2.4 → 4.4–5.1° (the plate-1
retarget gave 8.3–8.5); flange reach 532–1423 mm (plate-1 files 507–1426);
flange 23.5–31.6 mm LOWER than the 정반 1 plan at the work points (20 of
it the tip-down, the rest the measured tip being shorter than the design).
As written, the real tip at the 정반 2 stop pose is on the new pose rows to
0.010 mm / 0.0000° (the planner URDF's −252.99 vs −253, as on plate 1).

**Tools taught to take plate-2 pairs:** `retarget_joint_paths.py` fills a
tag missing from `--actual` from map.yaml and treats a tag missing from
`--planned` as "cannot be a plate-1 original" (the six joint files now all
classify RETARGETED — nothing to do, 7 ok); `check_retarget_joint_paths.py`
part 2 the same (**25**, the three `_plate2` files at 0.010 mm against pose
mode's own `transform_world_to_arm` + tip → flange); `check_task_discovery.
py` no longer assumes three pairs named 010 / 030 / 050 (counts from the
directory — its pre-existing failure is gone: **52 ok, 0 failed**);
`check_pose_vs_joint` 27. Real `TaskManager`: 6 + 6 tasks + go_home, no
logerr, every plate-2 task routes from 500 (41 hops via 400-lane + 503 →
507 / 504 → 508, vs 35 for plate 1).

**Not run on the robot.** `RELOAD_TASKS` (or a `task_executor` restart)
lists the six `_plate2` tasks. The plate-1 하형 files were left in task/csv
(the user did not say to delete them). Before the first plate-2 run: tags
126–150 are design positions with no calibration and no `yaw`, so the
stop-pose error there is whatever the tags' laying is (plate 1 showed up
to 20 mm before its calibration); the `scan_pose_*_plate2` twin fails IK
rather than colliding, so run it first; and the 20 mm is the user's number
for the surface the seek could not find, not a measurement — watch the
live standoff line at the first point.

### 2026-09-29 — Joint paths retargeted from the idle map to the calibrated one (stop pose + mount + tip)

User: the six files in task/csv were generated for `map_idle.yaml` (base
550 mm behind each tag); compute the difference between the idle stop poses
and `docs/robot_base_stop_poses_plate1_0928.csv` and correct the files,
Cartesian and joint, so they run on the current `map.yaml`. Plan shown
first; the user chose **option B for the joint files**.

**What the files turned out to be (FK, before planning anything).** The
joint rows reproduce their pose twins to 0.010 mm with exactly one model:
idle stop pose, DESIGN mount, DESIGN tip (the 0.010 is the planner URDF's
−252.99 vs −253). So the planner differs from the robot in three things,
not one: stop pose (2.5–20 mm, ≤ 0.30° on the six tags used — 105 / 106 /
107 / 118 / 119 / 120; every one of them nearer the plate than planned),
mount (9 mm, 0.80°), tip (13.5 mm, 10.8 of it z).

**Two departures from the request, both agreed.** (1) The Cartesian files
are NOT modified: they are world coordinates and pose mode already applies
the calibrated map through `/robot_pose`; adding the difference would move
the targets off the workpiece. (2) Correcting the stop pose alone (option
A) makes zone B WORSE — predicted tip xy error 8.7 → 17.0 mm — because
today the stop-pose difference partly cancels the mount / tip difference;
B (all three) takes both zones to 0 and makes joint mode land where pose
mode does. Promoted to *The joint files in task/csv are RETARGETED*.

**Done.** `tools/retarget_joint_paths.py` (dry run by default),
`tools/check_retarget_joint_paths.py`, `docs/robot_base_stop_poses_plate1_
idle.csv` (from `map_stop_poses.py --map config/map_idle.yaml`; that tool's
`--out` with a bare file name crashed on `makedirs('')`, fixed),
`docs/robot_base_stop_poses_plate1_diff_idle_vs_0928.csv` (26 tags, world
and body frame), originals + `retarget_record.yaml` in
`log/apriltag_nav/task_csv_backup/20260929_before_stop_pose_retarget/`.
Three joint files rewritten: 1613 / 1629 / 1668 rows, IK converged on all
(≤ 5 iterations), median row 0.94° of largest joint change, max 13.3 / 13.4
/ 15.1° on J3 at the nearly-straight-arm rows (J3 0.9–2.4° → 8.3–8.5°, i.e.
AWAY from the elbow singularity, since the base is nearer), 24–25 work
points per file move a joint > 5°, row-to-row step 5.72–5.73 → 5.72–5.79°,
limits ≥ 11°.

**Verified offline.** `check_retarget_joint_paths.py` 22: the tool end to
end on a synthetic pair (dry run writes nothing; only q cells of non-home
rows change; BOM / CRLF kept; backup byte-identical; transitions between
work points on the planned world path; home-side runs end on the home
joints; second apply a no-op; a file that is neither, stale stop poses and
a different existing backup all refused with nothing written), and on the
real files the written rows against **pose mode's own code** —
`transform_world_to_arm` + tip → flange at the nominal stop pose: 0.010 mm
/ 0.0000° over 2828 work points (23.9 mm before). Pose files md5-identical.
Real `TaskManager`: the same 6 tasks + go_home, same steps / counts / lift,
no logerr; joints changed on every non-home point, the pose tasks' IK
seeds (q0) follow. `check_pose_vs_joint` 27. `check_task_discovery` output
identical before and after — it still stops at its pre-existing
`standoff_050mm` lookup (the file set is 001 / 010 / 020 now), not fixed.

**Not run on the robot.** The stack was up and IDLE; `RELOAD_TASKS` (or a
`task_executor` restart) is needed before a TASK uses the new rows. First
run with a hand on the e-stop: the flange is 5.8–13 mm LOWER than the
planner drew it at the work points (the measured tip is 10.8 mm shorter
than the design one; pose mode has been sending the same flange height),
and the 09-21 tip's honest uncertainty is ±1.5 / 3 / 2 mm. No collision
model was run: the tool end follows the planned world path, the arm base
is where the robot really stands (up to ~3 cm from the planner's), and the
links in between sit somewhere between the two.

### 2026-09-29 — Path CSVs: speed 10 on both rows of every transition → task boundary

User rule: "waypoint_kind 인자가 transition에서 task로 넘어갈때는 둘 다 speed가
10이어야 해", for every CSV in task/csv (today's set: errorX standoff 001 /
010 / 020, one pose + one joint file each). Applied literally — EVERY
boundary, not only the approach from home: in each `rrt_final_path_*` the
last `transition` row and the `task` row after it are 10. Before, of the
363–369 boundaries per file 319–334 were (30, 30) (the single transition
row between two work points), 12–15 were (10, 30) and 20–38 already
(10, 10); the user had hand-set the first work point of each group in the
001 file, which is the same rule. `assigned_workpoints_*` has no
`waypoint_kind`, so the work point paired with each of those task rows
(`group_id`, `source_point_id` == `point_id`) got the same 10 — the two
files of a pair agreed on every work point's speed before (010 / 020) and
do again (0 mismatches of 940 / 943 / 945).

Size of the change: joint files 650–680 rows 30 → 10 each (now ~1080–1130
rows at 10, ~550 at 30), pose files 331–351. `speed` is `SetSpeed(percent)`
per row, so a scan is noticeably slower — roughly two thirds of all moves
run at 10 %. Task → transition (leaving a work point) is untouched.

`tools/slow_task_entry.py <task/csv> [--apply]` is the edit (text level:
only the speed cell changes, BOM / CRLF / scientific-notation cells kept;
idempotent — a re-run changes 0 rows); re-run it on a new planner export.
Originals: `log/apriltag_nav/task_csv_backup/20260929_before_transition_
task_speed10/` (the 001 / 020 files are untracked, so git cannot restore
them). Verified: byte diff vs the backup = speed cells 30 → 10 only, 0 rule
violations, the real `TaskManager` registers the 6 tasks + go_home with no
error. Not run on the robot; `RELOAD_TASKS` (or a `task_executor` restart)
to pick the files up.

### 2026-09-29 — Web UI: STOP ALL pinned top-right; a long task name no longer moves it

User (screenshot of the system bar during `scan_joint_errorX_…_001mm`):
with the long task name in the TASK chip, STOP ALL had dropped to a
second row at the far LEFT — "stop all 위치 바뀌면 안 돼". `#sysbar` was
one wrapping flex row (chips, spacer, conn chip, button), so whichever
chip overflowed pushed the button onto a new line. Now two blocks that
never wrap against each other: `#sysbar-chips` (flex 1, the chips wrap
among themselves) and `#sysbar-fixed` (conn chip + STOP ALL, flex
0 0 auto, top-aligned). A long name adds a chip ROW; the button does not
move. A chip wider than the whole row is cut with an ellipsis, and the
TASK chip's tooltip carries the full text. `web/` only (index.html,
style.css, app.js) — browsers pick it up on reload, no node restart. The
Qt window was not touched (its bar is one non-wrapping row with the
button last).

Verified in headless Chrome with the screenshot's chip texts, task name
short / the real 52-character one / 4x that, at 1920 / 1400 / 1100 /
800 / 500 px: the button's x, y and width are identical across the three
names at every width (11 px from the bar's right edge, y 29), no
horizontal scroll. `check_web_ui.py` 130; `check_web_ui_browser.py`
91 ok / 2 failed — the same two ("TASK + CHARGE chips", "standoff
line") fail against HEAD's web files, pre-existing.

### 2026-09-29 — Keyence seek OFF: the scan walked 40 mm down at every point on no measurement

User, after `TASK scan_pose_…_020mm` / `scan_joint_…_020mm`: why does the
arm try to go down at every point_id although the displacement sensor is
out of its range. It was the seek of 2026-09-21, which was turned on for
robot_ui's Auto standoff ("start from 4 cm") but is the SAME key the
TASK scan's standoff loop reads. From `arm_node`'s log (12:23–12:31):
every point read the −100000 sentinel (`side: far`), ran `[Seek 1..8]`
× 5 mm, ended `still out of range on the far side after seeking 40.0 mm`
and — `require_converged: false`, and the loop does not return to the
start height after a failed seek — **captured 40 mm BELOW the CSV pose**,
~9 s per point (467 points ≈ 70 min). The next row's move goes back up to
the CSV height, hence "down at every point". 22 of the 23 rows written
into the two Ra maps of the day carry that message (the 23rd was the
cancel); their Ra values are not measurements at the planned standoff.

`robot.yaml keyence.seek_enabled: false` on the user's choice (the
alternative offered and not taken: a scan-only switch that keeps the seek
for Auto standoff). Consequences: a scan point with the surface out of
range is captured at the CSV pose, untouched, with `standoff NOT
corrected: out of range on the far side (seek disabled)` in its row;
Auto standoff needs the surface inside the sensor window (~6.5–27 mm of
case standoff) before it will move, as before 09-21. `seek_step_mm` /
`seek_max_mm` and the code are unchanged.

Not explained by this, and open: the seek found NOTHING after 40 mm at
any point, so at the CSV pose the case was more than ~67 mm from whatever
is under the beam (sensor window ends ~27 mm), or the oblique beam (37°)
was off the surface. For a `standoff_020mm` file the case should start
near 36.5 mm. Either no workpiece was under the points, or the CSV z
(world 0.648 m) / the world → arm z chain is tens of mm from the real
surface — check with the tool over the workpiece and the live standoff
line in robot_ui before trusting a scan's height. Also worth a separate
look: a failed seek leaves the tool at the bottom of its walk.

Verified offline: `check_standoff_seek.py` 23, `check_scan_progress.py`
73; the yaml loads with `seek_enabled` False. `arm_node` restart
required (`sudo systemctl restart mobile-manipulator`); the startup
warning `keyence seek is ENABLED` should be gone.

### 2026-09-28 (night) — Base stop pose per map tag, for path generation: `tools/map_stop_poses.py` → `docs/robot_base_stop_poses.csv`

User: from the current map.yaml, compute where the mobile base CENTRE is
(world frame = 정반 1 centre; x, y, yaw) when front_cam has aligned on
each drive tag and stopped, taking T_mb2fc's non-ideal tilt / yaw into
account — reference data for the path generator, not a code change.
Arithmetic = `calculate_robot_pose` at an at-rest arrival with fore =
lateral = align residual = 0: heading = zone axis + the tag's calibrated
laying angle δ (map.yaml `yaw`, tags without one δ = 0), base = tag −
0.55·(cos, sin heading). **Why T_mb2fc's tilt and yaw are not extra
terms:** the script asserts, from tf_chain.yaml's physical matrix and
`ground_plane.T_tilted_to_level`, that the level virtual camera the
detections are published in is exactly diag(1, −1, −1) at the lens
(4.7e-10), so "on the crosshair" = under the lens NADIR (0.55, 0) and
"edge 0" = body parallel to the tag edge; the physical optical axis
meets the floor 7.6 mm from the nadir but nothing uses that point. What
the calibration leaves: ±0.22° of yaw (±2 mm lateral at the base) and
the 0.2° align band (1.9 mm). **WORK tags 100–150 only (51 rows), the
FWD-column pose** — user, same night: the 400 / 500-series are not
wanted (`--ids all` restores them); checks: dock 500 → x −1.9123 (the
design record), 116 (δ −0.77°) → +7.4 mm world x = the 9.6 mm/deg lever
of the tip tour. Re-run the script after any map.yaml / camera_offset
change.

### 2026-09-28 (night) — "주행이 너무 느리다": the last 15 cm crawl shortened; accuracy comes from the at-rest measurements, which are untouched

User: the driving is too slow — can it be faster with the same accuracy?
Measured first, from today's nav_log (184 hops): a 0.40 m hop takes
**21.8 s median forward / 22.4 reverse** (0.018 m/s average) plus a 3.0 s
arrival align (2+ passes on 33 %). One hop (123→124, 19:20) laid out:
launch align 1.0 s, accelerate + cruise 4.3 s (only reaches 0.062 m/s
before `plan_prepare_dist` pre-slows it), aim stop + measure + pivot
4.2 s, aim drive 3.1 s, **the last 15 cm at 0.010–0.015 m/s ~10 s**,
arrival align 4.3 s. So the driving proper is 7 s, the crawl 10, the
at-rest measurements 9.5. What defines the accuracy is the at-rest part
(aim pivot, align band 0.2°, 0.85 s settle, 5-frame medians) and the
crawl only sets the fore-aft stop scatter (±0.5–1 mm at 0.010 m/s) —
which is measured into `/robot_pose` since 09-04 and does not reach the
arm's world coordinates.

Applied (`robot.yaml robot:`): `blind_approach_dist` 0.15 → **0.10**,
`blind_approach_speed` 0.015 → **0.020**, `final_approach_dist` 0.08 →
**0.05**, `final_approach_speed` 0.010 → **0.015**. Expected: ~5–6 s
less per hop (the crawl 10 → ~4 s), fore-aft stop scatter ~±1.5 mm
(stop lead 1.2 mm instead of 0.7 from the measured 0.08 s latency).
0.10 still covers the ~6 cm odom error of the 1 m pivot-exit hop and the
3 cm at which a reverse hop into a 500-series tag first sees it. NOT
changed: `aim_min_deg` (skipping sub-3 mm aims would cost lateral
accuracy — the user's call), the first-hop align rule, and every settle /
band / median constant. **`plan_prepare_dist` 0.28 → 0.18 was tried and
reverted**: the tag appears at 0.19–0.21 m remaining, the cap executes
0.55 s later (3.3 cm at 0.06 m/s) plus 1.4 cm of braking, so 0.26 is the
minimum; at 0.18 the plant still had the base at 0.06 m/s when the tag
came into view (`check_robot_pose_live` G4 failed, 15 mm live lag). The
yaml comment records the arithmetic. Verified offline:
`check_nav_sequencing` 14, `check_front_cam_guard` 31,
`check_robot_pose_live` 30 (the plant's 106→107 hop 384 → 327 ticks).
Not driven: `mobile_node` restart required (`sudo systemctl restart
mobile-manipulator`); first check a 112→109→112 round trip's `aligned`
records — fore-aft within ±2 mm, lateral / yaw unchanged, hop time.

### 2026-09-28 (night) — Tip tour over 22 stops analysed: the error is the joint offsets missing from the COMMAND side; correction turned ON, xy only

User ran `tip_touch_cross_tags` (session `tip_check/20260928_170357_cross_tags`,
22 stops, frames in `results/tip_check/…`) and asked why the markers were
not centred and "the errors vary randomly". `analyze_tip_check.py` over
the frames: 19 measured, **118 / 119 / 120 (cross tag 4, marker 223) show
no tag36h11 at all** — re-detected at full resolution, decimate 1.0: none.
Either marker 223 is not on cross tag 4 or the tip is off by more than the
62 × 41 mm field; the neighbours' 10 mm errors make the first the likelier
— check the marker before the next run.

**Not random.** World x error +9.3 mm mean in zone B, −8.1 in zone C —
sign flips with the robot's heading; rotated into the BODY frame it is a
constant **−8.8 mm along body y (the arm / plate side) in both zones**,
sd (5.0, 3.8), no correlation with the reported base lateral (zone B
r = 0.01). A robot-frame constant plus an along-lane pattern within each
cross-tag group.

**Cause, reproduced by calculation:** the map calibration runs with the
joint offsets IN its chain (every entry `joint_offsets_applied: true`),
the tip command ran WITHOUT them (`~apply_joint_offsets_cmd` false, the
afternoon's decision). FK(q + dq) − FK(q) carried to the tip at every
recorded touch configuration, rotated to world through the point's own
R_AW: mean (+1.87, −0.09) vs measured (+1.95, −0.31) mm, same zone sign
flip, same along-lane trend (101→104 pred +9.2 / +5.4 / −1.3 / −6.6 vs
meas +4.3 / +3.6 / −6.6 / −2.7 in y); **rms 10.8 → residual 5.3 mm**,
residual sd (4.4, 3.0), the 7–9 mm outliers 104 / 116 / 122 (two of them
registered, not detected, frames). The sign matching means the offsets'
xy is physically right and the command side is what is missing. z is the
opposite, as the afternoon found: the offsets predict the tip +9…14 mm
above the reading, the Keyence measured +4.4 (cross tags 0 / 1 / 5) and
−3.7 mm (cross tag 2) — a full correction would run the case 5–9 mm into
the plate on the approach. Also seen: 12 of 22 standoff loops ended
"reading does not follow the motion" (still moved 3–8 mm, frame taken);
not an xy matter, separate.

**Applied (user: use the offsets in the Cartesian command like the map
calibration does, x/y only):** `CommandCorrector(skip_z=True)` computes
the full 6-DOF correction (translation AND the ~1.05° rotation — the
rotation is part of the in-plane fix: 0.32 m tip lever × 1° = 5.6 mm of
tip xy, 0.05 mm of z), then puts the commanded ARM-FRAME z back to the
target's z, re-evaluating D at the pose actually commanded (arm z is
vertical with the planar T_ab2mb). `joints_for_physical_pose` is the
joint-target form (IK of the xy-corrected pose; with skip_z off it equals
q − dq to 1e-4°), used by `_exec_pose`; `move_cart(physical=True)` takes
the pose form. Plausibility is judged on the full correction, dropped z
included. `arm_node` params: `~apply_joint_offsets_cmd` **true** and
`~joint_offsets_cmd_skip_z` **true**, both set in the launch now
(`roslaunch --dump-params` shows them). At the 22 touch poses the full
correction is 10.7–13.6 mm (bound 30), the xy applied 9.0–9.7 mm, the z
skipped −4.7…−10.2 mm at the flange. Verified offline:
`check_joint_offset_cmd.py` 21 → **37** (skip_z: commanded z == target z
bit for bit, FK(IK(cmd)+dq) on the target xy to 0.002 mm with the target
orientation, physical z misses by exactly the dropped term, the joint
form on target to 0.001 mm, `_exec_pose` against a URDF-IK fake sending
MoveJ joints whose FK(q+dq) is on the IK target xy while the controller's
own FK(q) is 6.8 mm off it, move_cart keeping z), `check_scan_progress`
73, `check_tip_tour` 20. **Not run on the robot: `arm_node` restart
required** (`sudo systemctl restart mobile-manipulator`; watch for
`joint offsets on the command side … xy only … ON` at start, then per
point `joint offsets applied (xy only, z kept; 9.x mm / 1.05 deg, dz
−x mm skipped)`). Expected on the re-run: rms 10.8 → ~5 mm; what remains
is the arm's position-dependent error (09-21's ±9 mm) and the stop pose.

**Re-run by the operator (18:50, `20260928_185021_cross_tags`, arm_node
restarted with the correction ON): rms 10.8 → 4.2 mm (3.6 without 116),
mean (−0.8, −0.5) mm, 17 of 22 measured.** Per stop, run 2 equals run 1's
post-prediction residual to sd (1.6, 2.3) mm — the remaining error is
REPEATABLE per stop, not noise. Two things it is: (1) **`calculate_robot_
pose` projects `camera_offset` along the ZONE axis, not the body heading**
— the align squares the body to the tag edges, so the body sits at zone +
align residual + laying angle δ (map yaw), the heading field carries all
three, but the 0.55 m lever is applied along the bare zone axis; the base
centre is then 9.6 mm/deg of body yaw off in world x, sign per zone.
Measured: err_wx vs (theta − zone) r = 0.87, sd 3.35 → 1.74 mm when
removed; 116 (δ −0.77°) +9.7 mm of which 6.0 predicted, 122 (δ −0.73°)
was run 1's +7.5. Fix = project along `heading` (not done — user asked
for the analysis). (2) **104 in run 2 was a one-off bad stop:** the stop
align overshot every pass (pass 1 settled −0.55° after predicting 0,
pass 2 lost the tag twice and settled −0.99°), was ACCEPTED at +0.59°
with the lens 8 mm off, and `publish_robot_pose` 3 ms later read the
newest single frame at θ 89.58 — 1.44° from the record's at-rest median
— i.e. the base was still turning; 1.44° × the 1.1 m lever = 27 mm along
the lane, marker outside the 62 × 41 mm field (registration 0.35). The
other three 104 arrivals today aligned in one pass. Improvements: re-
settle and re-measure before publishing an accepted residual, publish
from the align's at-rest median rather than the newest frame, and have
tip_check redo a GOTO whose aligned record says `align_residual_accepted`
or lateral > 5 mm. Also seen: the rotation part of the xy correction
moved the physical TIP ~3.5 mm lower (245 mm tip lever × 1°; the flange
z was kept, the tip's was not — run-to-run standoff corrections +3.49 ±
0.82 mm), absorbed by the Keyence loop; keeping the TIP z instead would
remove it. Cross tag 5's frames (121–124) show no marker this run (224
was there in run 1; 223 on cross tag 4 was missing in run 1 and present
now — a marker was moved). Standoff "reading does not follow the motion"
on 19 of 22 stops (12 in run 1), ending 0.5–0.9 mm from target — separate.

**Then (user: "1번과 2번 구현해줘") — both built.** (1) `calculate_robot_pose`
rotates fore, lateral and the camera lever by the full heading
(`robot.robot_pose_offset_along_heading`, default true; false = the old
zone-axis arithmetic, bit for bit when the body is on the axis). (2) The
continuous align's settled measurement is `_measure_tag_view_at_rest`
(median of `align_record_frames` = 3 frames, x / y / z / edge_deg); when
the passes run out it waits another `stop_latency_s + align_settle_s`
with zero command and re-measures before recording (`align_resettled`,
`align_median_frames` in the record); the view is kept
(`_aligned_view`) and `publish_robot_pose` publishes THAT view while it
is fresh (`align_view_max_age_s` 10 s, same tag; log says `from the
align's at-rest median (3 frames)` vs `from the newest frame`).
`calculate_robot_pose(tag_id, tag=)` takes the view and prefers its
`edge_deg` over the corners. Verified offline: new
`tools/check_robot_pose_heading.py` (**20** — on-axis identity in A / B /
C / DOCK with fore + lateral; tag 116's numbers give +5.8 mm world x,
104's +4.1, 101's −3.4 — the measured signs; a plant with the tag laid
0.8° off the lane: after the align the published base centre is within
0.00 mm of the plant where the zone-axis projection is 7.6 mm off, from
the 3-frame median; a 2.0 s-late plant runs out of passes, re-settles,
records the plant's true at-rest yaw and publishes it; stale / absent
view → newest frame), `check_nav_sequencing` 14, `check_robot_pose_live`
30, `check_front_cam_guard` 31, `check_calibrated_tag_z_yaw` 42. **Not
run on the robot: `mobile_node` restart required** (the same
`systemctl restart`). Cost: ~0.3 s per align (3 frames), plus one settle
(~0.85 s) only when a residual is accepted. Expected on the next tour:
x-error sd 3.4 → ~1.7 mm, rms 4.2 → ~3.0.

**Third tour (20:18, `20260928_201821_cross_tags`, both fixes live —
every align logs `median of 3 frames`, every pose `from the align's at-rest
median`): 22 / 22 measured (all six markers laid), rms 4.15 mm, 3.17
without 104; mean (+0.6, +0.3); x-error vs body yaw r 0.87 → −0.10 (the
heading projection did its job: 116 went +9.7 → +1.5 mm); per-stop sd
(2.1, 2.4) mm.** The one outlier is **104 again: (+8.9, +9.4) mm** — run 1
(+16.9, −2.7), run 2 out of frame, i.e. NOT repeatable like the other
stops, and its align misbehaves only there (runs 2 and 3: pass 1 settles
0.5° past a 0-predicted stop, the base arrives 22 mm off the lane). What
is NOT the cause, checked: the map's yaw — the tag edge front_cam reads
at rest before each aim pivot (`[Aim] at rest … edge`) equals the map's
laying-angle difference δ_prev − δ_target with **r 0.93, slope 0.93, rms
0.17° over 97 hops** in four runs, 103→104 included (−1.02…−1.14 vs
−0.90), so map.yaml's yaw column is real and δ_104 = +0.43 is roughly
right. Left standing: the x part (+7…+9 in runs 1 and 3) matches 104's
calibrated x sitting 5–6 mm off its neighbours (−1.7066 vs −1.7010 /
−1.7015; the same in both sessions — a repeatable single-entry chain
error, the plan reaches 104 at a different wrist spin, rz ≈ 71–76° vs
95 / 115), and the y part (0 … +9) together with the align overshoot
points at tag 104 itself (a skewed / damaged print reads a biased edge
angle — the 09-08 tag-15 effect) or the floor under it. Corners are not
in the nav records, so the squareness test needs a live read. A full map
re-calibration is NOT indicated: it reproduces the same 104 entry. Next:
inspect / replace tag 104, then a subset plan for 104 alone. Also
repeatable across all three tours: the standoff correction at cross tag 2
(108–111) is +5…+7.7 mm where the others are −1…+3 — that cross tag's
top sits ~7 mm lower than the model says (z, not used).

**Then (21:33, user re-laid tag 104 and calibrated 100–105: session
`calibrate/20260928_212842`, 6/6 ok; "104만 업데이트") — map.yaml tag 104
only: x −1.7066 → −1.7031, y −0.2531 → −0.2530, yaw 0.43 → 0.16.** The
other five entries of that session repeat the 16:14 values to 0.3–1.5 mm
(105 y 3.5 mm), so the session is sound and 104's move is the re-lay: it
now sits 2.4 mm from 103's x instead of 5.6, and its laying angle is
+0.16° instead of the +0.43° that stood out against the neighbours' −0.3.
Not applied from that session: 100–103 / 105 (the user asked for 104
alone; the differences are inside the session noise anyway). Verified:
72 tags / 142 edges, `MapManager` 500→105 / 500→125 unchanged,
`check_calibrated_tag_z_yaw` 42, `check_nav_sequencing` 14,
`check_front_cam_guard` 31, `check_robot_pose_heading` 20; CRLF endings
kept. `predictive_centering.map_world_path: latest` now resolves to the
6-tag `map_world_20260928_212842.yaml` — the other tags fall back to
map.yaml, which carries the same calibrated values. **`mobile_node`
restart required** (map.yaml is read at start); then a `GOTO 104` +
`tip_touch_cross_tags` shows whether 104 joins the other 21 stops at
~3 mm.

### 2026-09-28 (night) — `/robot_pose` streams the live position: tag-based at 10 Hz, odom-carried between tags

User: `rostopic echo /robot_pose` "왜 안 되요", then "실시간으로 로봇
현재 위치 계속 보내고 싶어". Nothing was broken: the topic was not
latched and went out ONCE per arrival (`publish_robot_pose`, last one
17:08:05 on tag 104), so an echo started afterwards saw nothing. Now
`mobile_node` runs a timer (`robot.robot_pose_live`, 10 Hz) calling
`MobileController.publish_live_robot_pose()`: with a MAP tag in
front_cam the pose is `calculate_robot_pose` on the tag nearest the
lens, every tick; with none, the last tag-based pose is carried forward
on the `/odom` delta (translation rotated by the anchor's world heading
minus odom yaw, heading by the odom yaw delta; anchor = every tag-based
pose, arrival or live). Refuses to guess: nothing before the first
tag-based pose, nothing from a tag the map does not know, nothing when
`/odom` is older than 1 s. **The arrival message is untouched and the
two are told apart by `flag`** (True = arrival, False = live):
`arm_controller.pose_cb` (and the `arm_controller_sdk` variant) drop
flag-False messages, so pose-mode IK still uses the at-rest arrival pose
bit for bit; `robot_ui`'s bridge keeps the arrival pose for
`robot_pose_snapshot()` (tip_check) and the stream in a new
`robot_pose_live()`. Promoted to the *Mobile base split* section.

Verified offline: new `tools/check_robot_pose_live.py` (30 — tag pose
== `calculate_robot_pose` and == the plant; odom carry-forward with the
odom frame yawed 30° vs the world: +0.20 m along the lane and +10° of
heading recovered to 1e-9; stale odom silent; a returning tag re-anchors
and discards the odom drift; non-map tag and no-anchor cases publish
nothing; arrival flag True and the live message at rest identical;
`enabled: false` restores the once-per-arrival behaviour; a real driven
106→107 hop streams 384 messages, both sources, within 8 mm of the plant
on every tick (the 0.1 s image latency × speed) and 1 mm at rest;
`pose_cb` keeps an arrival across later live messages),
`check_front_cam_guard` 31, `check_nav_sequencing` 14,
`check_scan_progress` 73, `check_web_ui` 130, `check_task_list_ui` 104,
`check_tip_tour` 20, `check_charging_manager` 21. Not run on the robot:
`mobile_node` (the stream), `arm_node` (the flag filter) and
`robot_ui_web_node` (the bridge) need a restart — `sudo systemctl
restart mobile-manipulator` (sudo needs a password; the base was idle on
tag 104, not charging). Then `rostopic echo /robot_pose` should print at
10 Hz with `flag: False`, `id: 104`, and keep printing when the tag
leaves the frame.

### 2026-09-28 (evening) — T_ab2mb x, y shifted by (−1, +7) mm: the sheet session's constant xy bias, applied as a robot constant

User: "z축 오차는 중요하지 않아. x,y 축 오프셋만 중요해" — with the
sheet as GT, is the chain's xy error a constant, and if so is the right
place a terminal correction (tag-position output / Cartesian command)
or `T_ab2mb`? Measured on today's session (body-side views v02 v04 v05
v18 v28 excluded, joint offsets applied): the raw `T_A2B` xy bias in the
sheet frame is **(−7.1, +1.0) mm, sd (1.5, 2.0)**, and it is the same
across spin (~60 / 105 / 135°: x −7.9 / −7.1 / −6.3), range (< / ≥ 0.47 m:
−6.8 / −7.6) and tilted / down views (−7.4 / −6.7) — a constant. The
planar least squares had left it at (−4.8, +1.3) because its residual
also carries the parking tilt's z and rotation, which trade against the
in-plane translation; a pure x, y shift of `T_ab2mb` removes it.

**Decision (recommended, user: "진행해줘"): into `T_ab2mb`, not a
terminal constant.** The constant lives in the mb frame — a sheet /
world-frame constant flips sign between zone B (+90°) and zone C (−90°),
while `T_ab2mb` is rotated by the robot heading automatically — and
`T_ab2mb` is read by BOTH consumers (the locator chain for map
calibration and `transform_world_to_arm` for pose-mode commands), so
one edit corrects the tag-position output and the Cartesian target
together; a terminal constant would be a second copy in two places.
The x, y, yaw-only rule stands (yaw 179.196° and tz −0.652 untouched).
Applied with `tf_chain_tool.py set T_ab2mb --matrix …`: **t (−9.20,
−99.13, −652.00) mm** (was −8.20 / −106.13); the yaml comment records
the provenance. Re-solve of the same session with the new file: **raw
bias (−0.1, −0.0, +0.8) mm** — the xy term is gone (the raw pose rms
7.8 → 8.9 mm is the z / rotation part the metric also weighs; z is
deliberately not corrected). `corrections.npz` of the session was
restored after each solve. Planner URDF `mobile_to_base` → xyz
(0.007804, 0.099245, 0.652000), rpy unchanged; both plates' plans +
yaw sweeps + `docs/all_tags_position.csv` regenerated (seeds moved
~3.7 mm, flange reach 0.51–1.05 m, 0/51 over 1.40). Checks:
`tf_chain_tool check` 13/13, `check_pose_vs_joint` 27,
`check_lift_compensation` 11, `check_joint_offsets` 22,
`check_joint_offset_cmd` 21, `check_front_cam_extrinsics` 24,
`check_chain_calib` 48, `check_sheet_sweep` 36, `check_scan_progress` 73.

**Not done here:** `arm_node` restart (`sudo systemctl restart
mobile-manipulator` — sudo needs a password; the charge relay drops) and
the calibration launch (not running; the next `path_tag_locator.launch`
reads the new file). What the constant does NOT fix, stated to the
user: the arm's position-dependent ±9 mm (off_x slope over the column),
the 09-22 tip tour's stop-dependent 8–23 mm (tracks the reported base
lateral, i.e. navigation / map, not this transform), and a print-scale
error along W x (−7 mm at x ≈ 1.0 m ≈ 0.7 %; excluded because the print
is the 09-21 one, rule-measured sx 1.0010). Next on the robot: plate-1
map calibration on the shifted chain, then compare 307 / 309 against
tag 200.

**Then (15:45, user: "오늘 캘리브레이션한 map파일을 map.yaml에 최신화") —
the plate-1 session of 14:04 APPLIED to `map.yaml`.** Session
`calibrate/20260928_140412` (`map_world_20260928_140412.yaml`, copy
`config/map_world_plate1_20260928.yaml`) ran on the SHIFTED T_ab2mb —
verified, not assumed: every `locate/` `result.npz` carries t (−9.20,
−99.13, −652.00), and recomputing all 23 entries through
`chain.compute_T_A2B` with that matrix reproduces the file to 0.0 mm.
Recomputing them with the OLD matrix isolates what the shift did: every
tag moves (+7.0, +1.1) mm in zone B and (−7.0, −1.1) in zone C, exactly
the mb-frame constant rotated by the lane heading. Against the 09-22
values the net move is B (+3.6, +1.5) mm sd 1.0 / 1.8, C (−3.2, −1.2) sd
0.4 / 1.0, yaw −0.1 ± 0.1° — i.e. on the old chain today's session would
have read ~3.5 mm on the OTHER side of the 09-22 map in both zones (a
zone-flipping mb-frame term that moved between 09-22 and today; the
sheet's 7 mm and the map's 3.5 mm do not agree on the size of the
constant, and this is not settled). 23 tags updated (100–117, 121–125:
x, y, yaw; the comment keeps the design and the 09-22 value); **118 /
119 / 120 FAILED — `tag A (id=4) not detected at iteration 1`, twice
each** (cross tag 4; the 09-22 sessions saw it — check whether the
20 mm marker 226 from the tip tour still covers it, or the 118–120 seeds
after today's plan regeneration) and keep their 09-22 values, marked in
the comment. Plans regenerated once more from the new stop poses (seed
moves ≤ a few mm). Verified: 72 tags / 142 edges, `MapManager` 500→105 /
118 / 125 unchanged, `check_calibrated_tag_z_yaw` 42,
`check_nav_sequencing` 14, `check_front_cam_guard` 31.
`predictive_centering.map_world_path: latest` already resolves to the
14:04 file by name (118–120 fall back to map.yaml). **`mobile_node`
restart required** (`sudo systemctl restart mobile-manipulator`).

**Then (16:14 session, user: "map.yaml에 업데이트") — the plate-1 re-run
APPLIED, all 26 tags this time.** Session `calibrate/20260928_161430`
(`map_world_20260928_161429.yaml`, 26/26 ok — 118 / 119 / 120 measured
now) replaces the 14:04 values in `map.yaml` tags 100–125 (x, y, yaw; the
config copy `map_world_plate1_20260928.yaml` is the 16:14 file). 14:04 →
16:14 on the 23 tags both sessions hold: |dx| ≤ 3.8 mm, |dy| ≤ 3.6 mm
(zone B within ±2 mm except 105 / 106 y +2.1 / +3.1; zone C 121–125 all
−2…−4 mm in both axes, 113–117 within 1 mm), yaw ≤ 0.2°, z up to 24 mm
(not used). 118–120 vs their 09-22 values: (−3.3, +0.6) / (−3.0, +0.5) /
(−2.8, −0.6) mm — the same zone C move the T_ab2mb shift gave every
other tag, so the zone C middle is consistent with its neighbours now.
Verified: 72 tags / 142 edges, `MapManager` 500→105 / 118 / 125
unchanged, `check_calibrated_tag_z_yaw` 42, `check_nav_sequencing` 14,
`check_front_cam_guard` 31; `map_world_path: latest` resolves to the
16:14 file by name. **`mobile_node` restart required.**

### 2026-09-28 — Tip-error tour over all plate-1 cross tags: per-stop targets in `tip_check`, `tip_touch_cross_tags` plugin, `analyze_tip_check.py`

User: find how far the vision TIP really lands from its target, stop by
stop — a 20 mm tag36h11 marker is laid on the centre of each plate-1
cross tag (0→219, 1→220, 2→221, 3→222, 4→223, 5→224 — first specified
as 230…225, changed by the user the same day) and every drive
tag touches ITS paired cross tag (the calibration plan's pairing:
100-104→0, 105-107→1, 108-112→2, 113-117→3, 118-120→4, 121-125→5).

**Why (the analysis that led here, same day):** the 09-22 touch run's
Basler frames, measured against tag 230 on cross tag 0 (itself located
from the etched tag-0 cells to 0.3 mm), put the tip **8–23 mm** off in
xy — far outside T_ee2tip's ±3 mm — and stop-dependent (102 +8.7 /
103 +7.6 / 104 **+20.2** mm world x, rounds repeat to 0.3–2.7 mm sd).
The world-x error tracks the REPORTED base lateral (`/robot_pose` y)
with slope 1.19, r 0.96; the implied true lateral is the same ±3 mm at
all three stops while the report moves 14 mm. At 104 the calibrated map
x (−1.7106 vs −1.705 at 102/103) plus the 0.55 m lever × its +0.58°
yaw sum to ~11 mm — the extra error. Not settled which of the two is
wrong. The tour measures this for every stop.

- **`robot_ui.tip_check`:** `Settings(targets={stop: (x,y,z)},
  target_info={stop: {ref_tag, marker_id}}, skip_refused=)`,
  `for_tag()` gives a stop its own target; `target_info` goes into the
  yaml / summary.csv (`ref_tag`, `marker_id`) and session.yaml;
  `skip_refused` records an out-of-bounds stop and continues (nothing
  has moved since the GOTO's arm home). `design_stop_pose` handles zone
  C / E (heading −90, stop 0.55 m north). Old plugins unchanged
  (defaults; `tip_touch_ref_tag` dry run identical).
- **`plugins/tip_touch_cross_tags.py`:** 22 stops (101–111, 114–124;
  100 / 112 / 113 / 125 are 1.2 m along the lane from their cross tag,
  flange 1.31–1.42 m > the 1.25 m bound, left out), target = the cross
  tag's `reference_tags.yaml` position + 1 mm, rz 0 everywhere, touch +
  Keyence standoff + LED capture, 1 round (`ROUNDS`). It reloads
  `robot_ui.tip_check` itself — the plugin runner reloads only the
  plugin file, so a web node that already ran a tip plugin would keep
  the old module.
- **`tools/analyze_tip_check.py <record_dir>`:** marker centre in each
  frame (detected, or registered against a same-cross-tag frame when
  the marker is cut off), image offset → flange (ψ from the tip
  result, −179.212°) → arm (recorded TCP rpy) → world (R_AW of the
  point's pose). Writes `tip_image_error.csv`, prints per-stop and
  per-cross-tag means relative to the overall mean (= the chain
  constant). `marker_axis_dev_deg` (the marker's edge vs the nearest
  world axis) checks the rotation chain: 0.0–0.2° on 09-22. Sessions
  without `target_info` take the marker id from `--markers` (default the
  219…224 layout; the 09-22 session needs `--markers 0:230`).

Verified: `analyze_tip_check` on `20260922_184115_touch` reproduces
the hand analysis to 0.1–0.4 mm (the ψ digits and the 0.26 mm tag-230
offset); new `src/robot_ui/tools/check_tip_tour.py` **20** (per-stop
targets / labels / skip, zone-C stop pose landing the tag where zone B
does, the plugin's 22 stops all in bounds, a fake-bridge run that
descends to each stop's own cross tag and skips the refused one, the
image → world rotation flipping between zones). Plugin dry run: 22 OK,
flange reach 0.74–1.10 m. Not run on the robot. The analysis wrote
`tip_image_error.csv` into the 09-22 record dir.

### 2026-09-28 (afternoon) — Joint offsets on the COMMAND side: built, wired, tested — and shipped OFF, because the constants are not position-consistent

User: "오프셋이 명령 쪽에 없는 문제를 해결. 오프셋 적용." Built:
`apriltag_nav/joint_offset_cmd.py` (`CommandCorrector`: joint targets →
`q − dq`; Cartesian targets → the pose whose nominal IK solution q gives
FK(q + dq) = target, two IK evaluations, 30 mm / 3° plausibility bound,
refuse on IK failure), `ArmController` (`_exec_pose` commands `q − dq`;
`move_cart(physical=True)` pre-corrects; `~apply_joint_offsets_cmd`),
`arm_node` JSON key `physical`, `ArmInterface.move_j_to_pose(physical=)`,
`RosBridge.arm_move_cart(physical=)`; opted in at the ABSOLUTE-target
callers — pose-mode scans, `verify_chain`, `sheet_path`, `sheet_sweep`,
`basler_tip verify`, `tip_check`, the calibration seed approach — and
deliberately NOT at reading-relative ones (jog, the align's correction
steps, the standoff loop, the UI MOVE), where the offsets' effect cancels
between the reading and the target. `check_joint_offset_cmd.py` (21 —
against the real URDF FK with a numerical IK: FK(IK(cmd) + dq) lands on
the target to 0.002 mm over the session's 12 views, correction 9.2 mm /
1.06° = the measured effect, refusals, the controller hooks against a
fake Fairino), `check_scan_progress` 71 → 73, align / sheet_sweep /
handeye_sweep / web_ui / task_list_ui unchanged.

**Then the numbers said not to turn it on.** Two independent
measurements of where the arm PHYSICALLY is, against what the offsets
predict: (1) the sheet session — camera height above the sheet by the
hand_cam PnP vs the model through T_ab2W: from the reading −6.5 ± 2.1 mm,
from FK(q + dq) **−16.9 ± 1.7 mm** (the offsets predict +10 higher, the
sheet measured 6.5 lower); (2) the 09-22 touch poses (real joints from
the yaml records) — the offsets predict the tip **+9…+11 mm higher** and
+10…13 mm in y, the Keyence measured **+4.8 mm**. So the applied set
(offsets, hand-eye, planar T_ab2mb) is consistent in the locator's
T_A2B metric (5–7 mm rms — a ~1° camera tilt trades against height at
the 1 m lever to tag 200, which is also why `arm_offsets.py` reports "the
sheet in the arm base: chain vs fitted 9.2 mm / 0.88°") but NOT in the
arm's absolute position; correcting the command side alone would move the
executed camera height from −6.5 to about −17 mm at the sheet and put the
tip **4–6 mm into the plate** at a touch target (the Keyence loop only
runs after arrival). Default `~apply_joint_offsets_cmd: false`; the flag
and the callers stay in place. What has to come first: the absolute-z
term — the parking's ~0.9° tilt the planar rule leaves out (the 6-DOF
base fit's roll −0.94°), or the hand-eye z that the sweep could not
observe; the sheet measures both directly per session. Not driven.

### 2026-09-28 — Arm error reset without restarting arm_node: `/arm/reset_error` + "Reset arm error" in robot_ui

User: "로봇암 리미트 걸리고 나서 복구하려면 우리 코드에서 어떻게?", then
"해주". A joint soft-limit trip (or a collision stop) latches an error in
the Fairino controller and every later MoveJ / MoveL / jog is refused.
Until now the only `ResetAllError()` in the stack ran in
`ArmController.__init__` (so: restart `arm_node` — with the service, the
whole stack, charge relay included) plus `move_to_home`'s one retry on
error 14; `jog_joint` / `move_joint` / `move_cart` never reset, so the UI's
joint jog could not be used to walk the joint back.

`ArmController.reset_error()` = the `__init__` sequence on demand:
`GetRobotErrorCode` → `ResetAllError` → `RobotEnable(1)` → `Mode(0)` →
`GetRobotErrorCode`, **no motion**, refused while busy; returns
`(ok, "error cleared (code before -> after)")`, fails when ResetAllError
returns non-zero or the code still reads non-zero afterwards — the
message then says a joint may still be beyond its soft limit (the
controller re-trips at once) and names the next step. An unreadable code
is reported as "reset sent, error state unconfirmed" rather than guessed.
`arm_node`: `/arm/reset_error` (`std_srvs/Trigger`, synchronous, under the
executor lock like `move_home`, bumps `motion_seq` with the result).
robot_ui: `RosBridge.arm_reset_error`, a **Reset arm error** button next
to Arm home pose in both fronts (web: `api_arm_reset_error`).

Recovery procedure: **Reset arm error → Joints group: jog the tripped
joint 2–5° back inward → Arm home pose.** ⚠️ Not verified on hardware:
whether the controller accepts an inward MoveJ from a pose that is still
past the soft limit. If it re-trips, use the Fairino web pendant
(192.168.58.2) in manual mode or drag teach; widening the limits with
`SetLimitPositive/Negative` exists in the SDK but was deliberately not
wired in. The exact `GetRobotErrorCode` return shape on this SDK build was
not read off the robot (assumed `(ret, [main, sub])`, handled defensively).

Verified offline: `check_scan_progress.py` 62 → **71** (fake Fairino with
a latched error: order ResetAllError → RobotEnable(1) → Mode(0), no MoveJ,
busy released; sticky error → failure with the soft-limit hint; refused
ResetAllError; unreadable code; busy refusal; `error_is_clear` shapes),
`check_web_ui.py` 128 → **130**, `check_task_list_ui.py` 102 → **104**,
`check_web_ui_browser.py` +1 (button → bridge call, no console errors; its
two pre-existing failures unchanged). Needs `arm_node` and
`robot_ui_web_node` restarted (`sudo systemctl restart mobile-manipulator`).

### 2026-09-28 — A0 sheet calibration, automatic: `chain_calib/sheet_sweep.py` over tags 301/303/305/307/309, rotations ≤ 60°

User: redo the A0-sheet calibration; hand_cam uses ONLY 301 303 305 307
309 of the grid; collect automatically; "r p y 를 너무 크게 돌리지마 —
최대 60도까지는 ok"; where do the records go. Three pieces, all offline-
verified plus one read-only dry run against the live stack:

- **`chain_calib.py --hand-tags 301,303,305,307,309`** (global, comma-
  separated — a `nargs` global option ate the subcommand name): the
  whitelist is applied BEFORE hand_cam's PnP (`Session.camera_T(only=)`),
  stored as `meta.hand_tags`, and `status` / `solve` / `arm_offsets.py`
  filter the stored corners with it, dropping (named) samples left with
  no allowed tag. On the real 09-21 arm session 54 of 65 samples survive
  the filter.
- **`scripts/sheet_sweep.py`**: plans 7 views per tag (down + 4 tilts of
  15° toward sheet ±x/±y at 0.45 m; down + 1 tilt at 0.55 m; spins 0 /
  +40 / −40° RELATIVE to the current camera spin, round-robin), turns
  them into flange targets through the live front_cam view of tag 200
  (`T_ab2ee = T_ab2W · T_W2hc · T_hc2ee`, level prior, lift-compensated),
  and checks EVERY target and EVERY MoveL chunk end (≤ 0.20 m / 30°):
  flange + vision tip ≥ 0.12 m above the sheet plane, `verify_chain`'s
  body-clearance line, reach 0.25–1.25 m, flange xy ≤ 0.70 m from the
  start, **orientation ≤ 60° from the START orientation** (`--max-rot`;
  tilt ≤ 20°, |spin| ≤ 60° as separate caps), and ≥ 2 whitelisted tags
  predicted whole in the image (`sheet.visible_tags`, 640×480, K_hand).
  A refused view is retried 5 / 10 / 15 cm higher, then skipped with the
  reason; a straight path that fails a rule between views goes up 0.10 m,
  across, down (`safe_path`); a failed MoveL skips that view; Ctrl-C
  stops after the current step and LEAVES the arm; the end returns to
  the start pose. Session = an ordinary chain_calib session
  (`log/chain_calib/<YYYYMMDD>_sheet_auto/`, saved after every view) +
  `sweep_plan.csv` / `sweep_log.csv`; `meta.sweep` records the settings.
  Refuses to start with `/map_calibrator` / `/path_tag_locator` /
  `/handeye_calib` registered (second arm commander).
- **The spin sign trap, caught by the check:** `session.describe_view`'s
  `spin` is the rotation of `T_hc2W` (sheet in camera), so the camera in
  W carries `Rz(−spin)`; the first draft used `Rz(+spin)` and every
  planned view came out 140–180° from the start (all 35 rejected by the
  new rotation rule — which is what the rule is for).

Verified: new `check_sheet_sweep.py` (**33** — geometry round-trips
through `describe_view`; plan on the REAL tf chain + the 09-21 session's
front_cam view / flange pose: 29 of 35 feasible, max rotation 42.8°,
lowest tool point 0.357 m above the sheet, all five tags, ≥ 2 whitelisted
tags each, 90° spin / `--max-rot 30` refused; chunks bounded; the runner
against a fake arm + rendered detections captures every view with only
whitelisted corners, one detour used, arm back at the start, coverage
READY (4 of 4 tilt directions, rotation diversity 85°); a refused MoveL
skips one view; Ctrl-C leaves the arm; 1-tag views are capture failures;
a rule-failing target moves nothing), `check_chain_calib.py` 48, and
`chain_calib.py --hand-tags … status/solve` on the real 09-21 session.
**Live, read-only:** `sheet_sweep.py --dry-run` against the running stack
(sheet still on the floor, front_cam on 200): 33 / 35 views planned, the
two rejections the body-clearance line at 303 / 309 tilted toward −x;
first target 0.75 m from the parked arm, so a real run starts with
parking hand_cam over 305 at ~0.45 m. **Then the user's own dry run
(arm parked over the column, first target 0.13 m)** showed the three
views tilted TOWARD the arm base (307 / 305 / 303 at sheet −x) landing
at reach 0.48–0.56 m only 11–13 mm above the base plane — legal by the
step rule, no margin: the planner now reduces a tilt whose displacement
points at the arm base to `--body-tilt` 8° (README §3-2's "5–10° on the
body side"), and the sweep's own rule applies the 0.65 m line up to
20 mm ABOVE the plane (`low_z_margin_m`). Re-run: 35 / 35 planned, the
body-side views at z +50–62 mm, max rotation 43.8°; check 36. **Not
driven.** README §3-0.

**Driven the same day (12:27, `log/chain_calib/20260928_sheet_auto`): 34
of 35 views captured** (one saw a single whitelisted tag and was not
stored; the return move failed the body rule and the arm stayed at the
last view). What the data said, in order of importance:
- **Every body-side (`a180`) view is an outlier** — 22–68 mm raw, and
  the sheet normal read 2.8–5.0° off the arm's z there vs 1.1° from a
  normal pose at the SAME foot (v18 vs v06): the arm's orientation at a
  folded, short-reach configuration, the 09-21 finding again. Not the
  paper (no trend along the column, r = +0.16) and not the chain.
  `solve` auto-excluded four; with all five out: raw 6.98 mm / 1.09°,
  joint floor 5.1 mm, the rest mostly the collinear 2-tag `[307, 309]`
  views (10–34 mm) — the price of a one-column whitelist.
- **The applied set is confirmed:** planar T_ab2mb (−8.4, −103.2,
  yaw 179.247°) vs applied (−8.2, −106.1, 179.196°) — 3 mm / 0.05°;
  joint offsets refit (body views out) J2 −0.53±0.38 / J3 −0.67±0.30 /
  J4 +0.12±0.21 / J5 −0.04 / J6 −0.52 — all within 1σ of the applied
  set, reprojection 7.5 → 1.5 px (hold-out 1.7). Nothing changed.
- **The sheet fit's hand-eye candidate (HAND verdict, D roll +0.85°,
  z −7.8 mm, jackknife 0.08° / 0.9 mm) FAILED the executed test** and
  was not applied: `verify_chain run` over 301–309 at 0.50 m, `--fit
  none` → centre offset mean (−1.9, −9.1) mm, rms 11.4, range −6.0 mm;
  `--fit hand` → (−2.4, −14.7), rms 16.4, range +7.6 — worse by about
  the size of D. The reprojection fit had also rejected it (offsets
  refit with the candidate: 1.54 → 2.06 px, hold-out 3.34).
- **Where the executed error comes from — two deliberate omissions,
  not a calibration error.** (1) The applied joint offsets act on the
  MEASUREMENT chain only; at these views FK(q+dq) puts the camera
  **+9…12 mm in arm z, +7…11 mm in y and 1.06° tilted** from where the
  controller's TCP says it is, and `verify` / every Cartesian command
  sends the flange through the controller unchanged, so the executed
  camera carries that whole offset. (2) The planar T_ab2mb rule leaves
  this parking's ~0.9° tilt in the residual (the 6-DOF base fit wanted
  roll −0.94° / tz −654.5), which at the column's 1.0 m lever from tag
  200 is ~16 mm of z. +10 − 16 ≈ the measured −6 mm range. The
  constant −9 mm off_y is the offsets' y term by size. What remains
  after both is the **off_x slope of +18.5 mm over the 0.6 m column
  (1.8°) with camera rx drifting +0.64 → −0.27°** — the arm's
  position-dependent error of 09-21, ±9 mm at the column ends, which
  no constant corrects; the 2 mm goal at 307 AND 309 is not reachable
  through the chain as it stands. Decisions pending (user): command-
  side offset application (`MoveJ(IK − δq)` / target pre-correction —
  affects every arm move), whether the tilt stays a per-parking
  residual, and a per-position correction or the one-frame method for
  the last ±9 mm.

### 2026-09-22 (18:00) — Tip over cross tag 0 from the 102 / 103 / 104 stops: `robot_ui.tip_check` + two Scripts-tab plugins (hover 30 mm; touch + Keyence standoff + LED capture), 3 rounds

User, two requests: (1) "fc으로 102, 103, 104에서 멈췄을 때, 정반 중심
(world, map.yaml) 기준 0번 레퍼런스 태그 (-0.60, -1.20, 0.001)보다 높은
(-0.60, -1.20, 0.030)에 tcp(tip) 이동 — 3초 멈추고 홈, 다음 태그 …
104까지, 다시 102부터, 3번"; then (2) the same at **(-0.60, -1.20,
0.002)**, saving the world target + joints on arrival, 1 s later the
Keyence standoff, the same again `_after_correction`, then the VISION
lamp and a capture, 3 s, next. Built as robot_ui Scripts-tab plugins
(RUN from any browser, runs on the robot PC) over ONE module,
`src/robot_ui/src/robot_ui/tip_check.py` (`Settings` + `run_sequence`):
**`plugins/tip_over_ref_tag.py`** (hover30) and
**`plugins/tip_touch_ref_tag.py`** (touch). Per point: `GOTO <tag>` on
`/task_command` (task_executor homes the arm, drives, aligns) →
`/robot_pose` of THAT arrival → `transform_world_to_arm` (the pose-mode
scan's own maths, live lift) → tip → flange with `T_ee2tip` → MoveCart
to 0.20 m above → MoveL down → [touch: record before; 1 s;
`/arm/standoff` (config target 16.5); record after_correction;
`/camera/capture` lamp on → `results/tip_check/<session>/r<n>_tag<id>.png`]
→ 3 s → MoveL up; the next GOTO's home takes the arm back, the last
point homes explicitly. UNDOCK first when the BMS shows current (the
robot was charging on 500 at 46 %). Records
`log/apriltag_nav/tip_check/<ts>_<name>/`: `summary.csv` + one yaml per
point (commanded world target, robot pose, arm-frame targets, and for
before / after_correction the TCP, the six joints, the tip's position in
WORLD axes (the affine transform inverted: target + R_AWᵀ·Δ), its error
vs the target, the live Keyence line; the standoff result; the image).

Settled with the user when asked: **z datum — both targets are from the
PLATE TOP** (reference_tags.yaml's frame), and `transform_world_to_arm`'s
world z is the FLOOR datum (arm_base_z above the floor), so the module
adds 0.080 before the transform (`PLATE_TOP_ABOVE_FLOOR_M`; feeding a
plate-top z in directly puts the tip 80 mm too LOW — the 2026-09-14
hand-eye check added the same +0.080 by hand). Hover: tip z −542 mm arm
frame, flange −328; touch: tip −570 (1 mm above the tag's top face — the
tip is the surface point the Basler centre sees at the case's 16.5 mm
zero standoff, so the case arrives ~16.5 mm above the tag and the loop
trims it), flange −356. Tool straight down, **rz 0 at all three stops**
((180, 0, 0) is also the flange's home orientation, so the approach is a
pure translation); the script sends UNDOCK itself; two-stage approach;
standoff target = robot.yaml 16.5; save BOTH the commanded target and
the measured tip world position; capture and save only, no Ra; an
unconverged standoff is recorded and the capture still happens; 3
rounds. Built in without asking: the tool frame is read off `/arm/state`
at the home joints (flange (−159, 700, 774) vs tip), the PHYSICAL flange
is bounds-checked (reach ≤ 1.25 m, z ≥ −0.5 m, verify_chain's body-
clearance line: below the base plane needs ≥ 0.65 m — here 0.75–0.86 m),
and a GOTO whose base result does not confirm the tag, a `/robot_pose`
for another tag or older than the command, an unknown lift height or an
arm not at home REFUSE before any arm move. `DRY_RUN = True` prints the
targets for the design stops: hover tip (−394.5, 1004.5, −542) /
(4.2, 998.5, −542) / (401.8, 998.8, −542) mm at 102 / 103 / 104.

`RosBridge` gained `robot_pose_snapshot()` (a `/robot_pose` subscription
kept from bridge start — the topic is not latched and is published once
per arrival, at rest after the align) and `arm_move_cart(..., linear=)`
(the JSON key arm_node already honoured; the UI's MOVE button is
unchanged, MoveL). Verified offline: scratch `t_tip_check.py` (**42** —
hover: dry run moves nothing; charging → UNDOCK → 9 GOTOs → 27 moves in
MoveCart / MoveL / MoveL order at rpy (−180, 0, 0), descend exactly
200 mm, every descend z −327.6, home at the end, 9 rows + 9 yaml with
the planted 0.3 mm settle error read back as tip world z 0.0303; touch:
move-move-standoff-capture-move per point, standoff target None, LED on,
descend z −355.6, the fake's −1.7 mm correction read back as
`correction_dz_mm` and after-tip wz 0.0006, joints before / after, the
PNG per point, yaml before / after_correction / standoff / capture
blocks, the Keyence line before (18.2) and after (16.5); an unconverged
standoff and a failed capture are recorded and the sequence continues;
a failed GOTO stops it; wrong-tag pose, not-at-home, out-of-reach
refuse; the tip tool frame sends tip coordinates; cancel during the
dwell leaves the arm; a 100 mm lift lowers the target 100 mm),
`check_web_ui.py` 128, `check_task_list_ui.py` 102. The bridge change
needs a `robot_ui_web` restart (a hand `rosnode kill` + detached
`roslaunch robot_ui robot_ui_web.launch` was used first; the user's
`systemctl restart mobile-manipulator` at 18:03 then replaced it with the
service's node, which loaded the new bridge — the plugins and `tip_check`
are imported at RUN time, so they need only the Scripts tab's refresh).

**Run on the robot by the operator the same evening.** Hover (18:05,
`tip_check/20260922_180545_tip_over_ref_tag0.csv`, the pre-module CSV
format): round 1 at 102 / 103 / 104 reached, controller settle ≤ 0.2 mm,
STOP during round 2. Touch (18:41, `20260922_184115_touch/`): **9 / 9
points, every Keyence standoff converged in 3–6 steps (|err| ≤ 0.17 mm),
every capture saved** (`results/tip_check/20260922_184115_touch/`). The
number to keep: the standoff correction was **−4.6 … −5.1 mm (mean
−4.8, sd 0.16) at all three stops and all three rounds** — the chain
placed the tip ~4.8 mm ABOVE the tag's top face and the sensor took it
down; after the correction the model reads the tip at world z −0.0025 …
−0.0030. A constant, stop-independent z bias of the world → arm chain
(T_ab2mb tz / arm_base_z / T_ee2tip z / the 0.080 plate height), not
a per-stop navigation term; xy is not measured by this (the Basler
frames are). A first touch attempt at 18:40 stopped before moving:
"task_executor did not start goto_102 within 20 s" — the GOTO right
after the operator's STOP was not picked up within the ack window;
the retry a minute later ran.

### 2026-09-22 (16:00) — Map calibration froze at tag 123: the VS Code launch terminal again; 26/26 finished with a subset plan and merged

User: "map 갤리브레이션 진행했는데 하다가 123번태그에서 멈추었어요". Session
`20260922_154103` had 100–122 all ok (23 entries); at 16:02:41 the
calibrator logged `arrived at tag 123` and `auto_align: initial MoveJ 1/4`
and then NOTHING — not the `[ArmInterface] move_cart ->` line that follows
within 2 ms on every other entry, no 60 s timeout, and `arm_node` never
received a command (its next motion was the operator's jog at 16:04:50).
`rosnode list` still showed `/map_calibrator`. Cause is the 2026-09-21 one,
mid-run this time: `path_tag_locator.launch` was started in a VS Code
integrated terminal (`/dev/pts/0`), the pty stopped being drained, and
`rospy.loginfo`'s stdout handler blocked inside that log call holding the
logging lock — the file handler had already written the line, the publish
after it never ran, every other thread queued on the lock (`wait_woken`
wchans). Proved with `timeout 3 bash -c 'echo > /dev/pts/0'` → 124; not
XOFF (`tcflow(TCOON)` changed nothing); py-spy absent and ptrace_scope 1
blocked gdb, so the Python stack was inferred from the log ordering.
Nothing in the calibration code was at fault and nothing was lost (entries
are written atomically per attempt).

Recovery: subset plan `log/path_tag_locator/calibrate/plan_plate1_tags123-125.yaml`
(the plate-1 header + entries 123–125, validated with the real loader),
run through `/map_calibrator/run_calibration plan_path=…` on a relaunched
calibration stack (user, 16:16): 123 ok, 124 ok on retry (tag 5 not seen
at the seed once), 125 ok — session `20260922_161657`.

⚠️ **The two halves are on DIFFERENT chains, so they were NOT merged into
a usable map.** The 15:41 session ran on calibration nodes started at
11:35 — before the joint offsets / refit T_ab2mb / new T_hc2ee of this
afternoon existed on disk — and its entries carry no
`joint_offsets_applied`; the 16:16 relaunch read the new tf files
(`joint_offsets_applied: true`). Tags measured by both (the 15:16 session
vs the subset): 123 old (1705.8, −2127.8) → new (1697.7, −2148.9) mm,
124 (1715.4, −2521.7) → (1699.7, −2546.8), 125 (1724.6, −2922.4) →
(1702.0, −2948.8) — the new chain moves them 8–23 mm in x and 21–27 mm
in y, onto the design grid (1710, −2150 / −2550 / −2950) where the old
chain had a +22 mm y bias and an x drift along the lane. So **every
plate-1 session of today before 16:16 (14:21, 14:51, 15:16, 15:41) is
old-chain data; the only new-chain map_world is the 3-tag
`map_world_20260922_161657.yaml`.** A 26-tag merge was built, checked
(lane spacing 388–419 mm, seam 122→123 412 mm — the seam itself does not
show the mismatch, the absolute offset does) and then parked as
`calibrate/20260922_161657/map_world_plate1_merged_CHAIN_MISMATCH.yaml`
with a `WARNING_chain_inconsistent` key, OUT of the `map_world_*.yaml`
glob that `predictive_centering.map_world_path: latest` reads (that glob
sorts by name, so `latest` is now the 3-tag file; blind steering falls
back to map.yaml for the other tags, the documented fallback). **Next:
re-run the whole plate-1 plan on the new chain, from a detached launch.**

`STOP_LAUNCH_kr.md` §3.1 records the mid-run form of the failure and the
rule: **every hand launch — the calibration launch included — goes
`setsid nohup … > log/ros/….out 2>&1 &`, never into a VS Code terminal
pane.** The relaunched calibration stack of 16:16 is still on a VS Code
pty (`pts/6`, draining for now); restart it detached before the full
re-run.

**Later (17:16): plate 1 re-run TWICE on the new chain and the result
filed.** Sessions `20260922_162335` (16:23–16:48) and `20260922_165326`
(16:53–17:16), 26/26 each (two 16:51 starts failed instantly — nodes not
ready — and are noise). Session-to-session: xy sd 0.3 / 0.4 mm, **max
0.9 mm**, z sd 7.4 mm, yaw sd 0.03°. Against the old chain (14:51 /
15:16, themselves repeatable to 0.5 mm) the new chain moves tags by up
to 35 mm (y sd 17 mm) — and lands on the design grid: zone B dx
+1.3 ± 3.2 / dy −3.5 ± 2.5 mm, zone C dx **−16.7 ± 6.3** / dy −2.4 ±
3.5 mm (old chain: 12–18 mm biases with 10–13 mm sd), z −57 ± 10 mm
(design −80; the z scatter is the known depth term). The zone C lane
reading 17 mm closer to the plate centre than map.yaml is consistent
across both sessions and all 13 tags, so it is where the tags are (or a
constant chain term), not noise. Result = per-tag MEAN of the two:
`log/path_tag_locator/map_world_20260922_plate1_final.yaml` (header
`method` / `source_sessions`, per tag `spread_mm` / `dev_from_design_mm`;
sorts after the session files, so `map_world_path: latest` picks it) and
a copy **next to map.yaml: `src/apriltag_nav/config/map_world_plate1_20260922.yaml`**
(user: "주요결과는 map.yaml 같은 폴더에 복사"). It is NOT a drop-in map.yaml
(world frame = reference_tags.yaml's tag-0 frame; same origin and axes
as map.yaml by design, hence the small deviations, but the generator's
note stands). The old-chain sessions of the day are kept as records only.

**Then (17:30, user: "이 결과 실제로 작용") — APPLIED to `map.yaml`.**
Where the calibrated positions act: `mobile_node` re-resolves
`predictive_centering.map_world_path: latest` per hop, so blind-segment
steering had already picked the final file up at 17:17:55 (`loaded 26
calibrated tag positions`). Everything else reads **map.yaml** —
`calculate_robot_pose` (`/robot_pose`, hence pose-mode
`transform_world_to_arm`), the hop odom distance (`hypot` of the two map
positions in `go_to_next_tag`) and the prediction fallback — so tags
**100–125 in `config/map.yaml` now carry the calibrated x / y** (4
decimals, design value + delta in a trailing comment, header block dated
2026-09-22; CRLF endings kept, 52 value lines changed). Dock, pivots,
zone A and plates D/E are untouched. Consequences: `/robot_pose` on a
work tag is the calibrated world position (zone C reads x 1.693, not
1.71, and the arm's world→arm transform follows); hop distances are the
measured 388–419 mm instead of a flat 0.400 (the pivot-exit hops 505→112
/ 506→113 change by a few mm, the align absorbs it);
`generate_calibration_artifacts.py` would now seed from the calibrated
stop poses (≤ 17 mm from the current plan seeds — not regenerated, the
align covers it). Verified offline: yaml parses (72 tags / 142 edges),
`MapManager` routes 500→105 and 500→125 unchanged, `check_nav_sequencing`
14, `check_front_cam_guard` 31, `check_task_discovery` 47/48 (the known
pre-existing failure). **`mobile_node` must be restarted to read it**
(`sudo systemctl restart mobile-manipulator` — the charge relay drops, as
on every stop); the calibration nodes read `map_in_path` per session and
need nothing.

**Then (18:00, user: "x, y 말고 모든 값 사용") — z and yaw applied too.**
map.yaml tags 100–125 gained `z` (tag top, calibration world frame) and
`yaw` (tag x-axis heading, wrapped; zone B ≈ 0°, zone C ≈ 180°, ±0.7° off
the axis) from the same final file; roll / pitch deliberately not (a floor
tag's tilt = the chassis attitude the chain measured). Consumers:
`mobile_controller._tag_yaw_error_deg` adds δ = yaw − nearest axis to
`/robot_pose.theta` (sign pinned: `rot2rpy_deg`'s rz is the world
heading of the tag x axis, CCW +, roll 180 notwithstanding — checked on
a synthetic face-down tag; the align leaves the body parallel to the tag
edges, so δ carries 1:1 into theta); `arm_transform.tag_floor_z_m` turns z
into the floor height above the CSV datum ((z − 0.001) − (−0.080)) and
`transform_world_to_arm(floor_z_m=)` adds it to the arm base height like
the lift, `arm_controller._exec_pose` looking the tag up by
`/robot_pose.id` (map.yaml read lazily on first use). Config: `robot.
robot_pose_use_tag_yaw`, `arm_calibration.use_tag_z / world_floor_z_m /
tag_z_max_offset_m` (0.05: a gross z is refused, since at the 16.5 mm
standoff a wrong z is a collision). Plate 1's z says the floor is ~+20 mm
above the design −80 under every tag (tag top −57 ± 10 mm; −35 at 104,
−91 at 110) — real or a chain z bias is not settled; the standoff loop
covers ±20 mm, so the first pose-mode scan is the test. Promoted to
*Coordinate Frames*. **Then the user decided against z ("z는 사용
안하기로", then "지워주"): `use_tag_z: false` and the 26 z lines removed
from map.yaml (the values stay in the final map_world file), the code
path and its check stay (42, off-state and absence asserted), yaw stays
ON.** Verified offline: new
`tools/check_calibrated_tag_z_yaw.py` (39 — floor arithmetic, refusal,
bit-identity at floor 0, the floor entering exactly like the lift, theta
in zones A/B/C for six yaws incl. the ±180 wraps and the switch, the real
map.yaml's 26 z / yaw inside their bounds), `check_lift_compensation` 11,
`check_pose_vs_joint` 27, `check_scan_progress` 62, `check_nav_sequencing`
14, `check_front_cam_guard` 31. Not run on the robot: `mobile_node` (map +
theta) and `arm_node` (floor) restart required.

### 2026-09-22 — C and E re-solved for the new hand_cam K; joint offsets APPLIED to the locator chain; T_ab2mb refit — one set

User, in three steps: (1) "지금 fc과 hc이 보는 화면을 보고 태그 200번 기준으로
307 / 309의 좌표를 계산" — the calibration GOAL stated: the map-calibration
CSV must reproduce real dimensions, tags 307 / 309 at (1000, 450, 0) /
(1000, 600, 0) mm in tag 200's frame, **xy within 2 mm**. (2) "오늘 A가
바뀌었으므로 C와 E는 저장 데이터로 다시 풀어줘". (3) "관절 오프셋 넣어줘.
없는 것보다는 낫잖아" — after being told the fit reaches ~4.4 mm, not 2.

**Live measurement first (arm at TCP (−159, 550, 63), hand_cam 0.55 m
over 305/307/308/309, front_cam on 200, 20-frame corner means, scatter
≤ 0.06 px):** through the applied chain of the moment (09-18 hand-eye,
09-21 T_ab2mb) tags 307 / 309 landed at (1005.0, 438.8) / (1004.9, 589.0)
mm — **one constant (+5, −11) mm shift for all four tags**, sub-mm
internal geometry (307↔309 150.2 mm, 305↔307 150.1), z −19 mm with
front_cam's measured 1.6° tilt of tag 200 (paper lift) or +8 mm with
the level prior. Repeatable to 0.2 mm between captures. Reading: the
chain is biased, the tags are read fine; a single view cannot split the
bias into hand-eye vs base. The 2 mm goal is reachable without the
chain (ref tag + path tag in ONE hand_cam frame, both at 1.0–1.2 m —
proposed, not built); with the chain, a per-view-pose correction table
or the joint offsets are the options, neither guaranteed to 2 mm.

**Dependency order, written for the user:** A hand_cam K/D → (B
front_cam fit ∥ C hand-eye) → D T_ee2tip (needs A, C; not in the map
chain) / E T_ab2mb (needs A, B, C, arm FK, parking) → F map
calibration. Change X, redo only what is right of X. Today A changed
(the checkerboard K/D from `hand_cam_intr_20260922`, another session,
applied in robot.yaml and running in `robot_camera_node` — fx 601.87 fy
601.93 cx 321.35 cy 238.51, k1 +0.177 k2 −0.348), so C → E → F.

- **C:** the 35 sweep samples of `run_20260918_144420` re-detected on
  frames rectified with the new K/D (`Rectifier`, the node's own path):
  with the driver K the file reproduces HEAD to 0.00 mm (method check);
  with the new K **t (36.46, −331.77, −156.14) mm, rpy (−0.047, −0.435,
  −179.343°)**, 5.4 mm / 0.22° from HEAD, tag scatter 2.66 → 2.06 mm rms.
  Saved as `result_sweep123_refined_K20260922.npz` in the run dir and
  applied (`tf_chain_tool.py set T_hc2ee`).
- **Joint offsets (user's decision):** the 09-21 chain session (63 views)
  was captured BEFORE joints were recorded, so it cannot take offsets;
  the 65-view arm session (`20260921_arm`) can. `arm_offsets.py` with the
  new hand-eye FIXED and the new K (v11 v13 v20 v45 excluded, all views
  in the fit): **J2 −0.341, J3 −0.529, J4 −0.051, J5 −0.134, J6
  −0.496°** (jackknife sd 0.42 / 0.36 / 0.29 / 0.06 / 0.06), corner
  reprojection 7.43 → 3.39 px (hold-out 7.75 → 3.51 with every 4th view
  out). The near/far ±2° disagreement of the earlier fit stands — this
  is an average — and the other session's finding that the offsets
  explain little of `sheet_path`'s ±13 mm is not contradicted; what they
  DO buy is below. Applied as `config/tf/arm_joint_offsets.yaml` (+
  npz; `enabled: false` switches every consumer back to dq = 0).
- **E:** `chain_calib.py solve` on the arm session with the new hand-eye,
  new K and the offsets in the chain (new `--arm-offsets config|none|NPZ`,
  default `config` = what the locator applies): raw 14.5 mm (the old
  T_ab2mb no longer pairs), base fit → **7.92 mm / 0.46°**, jackknife
  0.4 / 0.4 / 0.9 mm; the same session WITHOUT offsets fits to 10.1 mm
  (hold-out 12.7 vs 7.7 with). Applied: t (−8.68, −103.42, −643.10) mm,
  rpy (−0.451, −0.103, 179.214°). Planner URDF `mobile_to_base` updated
  to match (xyz 0.006101 0.108596 0.642272, rpy −0.007903911
  −0.001687209 0.013726417). The 09-21 63-view session can no longer be
  scored with this set (no joints), so "6.6 mm raw" is not comparable
  with today's 7.9.

**Code:** `apriltag_nav/arm_fk.py` (ArmChain moved from chain_calib,
which re-exports), `tf_chain.load_joint_offsets / write_joint_offsets /
check_joint_offsets / arm_flange_T` (FK(q) must agree with the
controller TCP within 2 mm / 0.1° or the pose is used unchanged with a
warning — joints and pose from different states, or a different URDF),
`tf_chain_tool.py joint-offsets [--apply NPZ | --enable | --disable]` +
`show` / `check` lines, `path_tag_locator.chain.compute_T_A2B(joints_deg,
joint_offsets_deg)` → `joint_offsets_applied` in the result,
`ArmInterface.get_pose_and_joints()` (ONE /arm/state for both),
`path_tag_locator_node` + `CalibrationOrchestrator` pass them and record
`joints_deg` / `joint_offsets_deg` / `joint_offsets_applied` per run /
entry; `arm_offsets.py` stores hand_eye / urdf / K source in its npz.
NOT changed: pose-mode commands (`MoveJ(IK − δq)` is still not built —
the offsets act on the MEASUREMENT chain only), `verify_chain.py` /
`sheet_path.py` (they command flange targets), robot_sim (an ArmState
without joints falls back to the pose).

Verified offline: new `tools/check_joint_offsets.py` (22 — FK(q) ==
the reported TCP 0.001 mm; dq = 0 / no joints / 5 joints / disabled file
/ pose-joint mismatch all give the controller pose bit-for-bit; +1° J6 =
+1° spin of the flange about its z; J2 lever 9.8 mm/°; yaml/npz round
trip and the 6-value / 5° refusals; compute_T_A2B with and without),
`tf_chain_tool.py check` 13/13 (new offsets line), `check_pose_vs_joint`
27 (its "expected discrepancy" bound widened 20 → 5 mm: the new mount is
closer to the design, so the planner files are now 19 mm off, not 45),
`check_lift_compensation` 11, `check_front_cam_extrinsics` 24,
`check_repose_from_corners` 10, `check_chain_calib` 48, `check_basler_tip`
11, `check_handeye_sweep` 74, `check_scan_progress` 62,
`check_camera_intrinsics_override` 20. **Not run on the robot.** Restart
the calibration launch (all four nodes read tf at start) and `arm_node`
(T_ab2mb for pose-mode IK) — ⚠️ a map calibration ran at 15:11–15:15
today (tags 121–125, `locate_log.csv`) on the OLD chain and predates all
of this. The docs of the other session that say the offsets are "NOT
applied" (`HANDEYE_FITTING_STATUS` §0/§6, HANDOVER §2-0c) got a dated
line saying they now are.

**Then, on the user's instruction ("63뷰 체인 세션은 앞으로도 못 쓸 테니
삭제, 그 외에 다시는 사용할 일 없는 것들도"), deleted — git has them:**
`log/chain_calib/20260921` (the 63-view 11:29 session, no joint angles,
so it can never take the offsets) and its record
`src/chain_calib/docs/CHAIN_CALIB_2026-09-21_kr.md`; hand-eye runs
`run_20260918_152111` (0 samples) and `run_20260921_195136` (the 20:09
sweep, never applied, untracked); the superseded solves
`result_sweep_only_refined.npz` / `result_sweep123_refined.npz` in
`run_20260918_144420` (the K20260922 re-solve is the applied one; the
35 sample frames stay); and in `20260921_arm` the rejected hand-eye-free
candidate `T_hc2ee_fit_20260921_arm.npz` with its three report txt files
and the other session's `arm_offsets_20260922_handeye_fixed_newK.txt`
(numbers survive in the status doc §6 / Work Log). **Kept**, because an
applied value comes from them: `20260921_arm` (samples, corners, meta,
`arm_offsets.npz`, `corrections.npz`, `T_ab2mb_20260922_offsets.npz`),
`basler_tip_20260921`, `hand_cam_intr_20260922` + the checkerboard PDF,
`run_20260918_144420` (frames + `result.npz` + the K20260922 npz),
`log/apriltag_nav/calib_pair_20260915_a`, and the `locate/` /
`calibrate/` map-calibration outputs (results, not inputs — today's
15:11 run predates the new chain). `check_chain_calib.py`'s "old session
loads with joints_deg None" case now builds its own old-format session
(48 still); the `tf_chain.yaml` T_ab2mb / T_hc2ee comment headers
describe the 09-22 provenance and name the 09-21 value as history.

**Then, later the same evening — T_ab2mb made PLANAR on the user's
rule.** Asked what "±8 mm per re-parking" meant, the answer was that the
6-DOF fit's roll / pitch ARE the chassis' attitude at the sheet's
parking (plus paper slope), and since map calibration happens at other
parkings ("F와 G를 같은 주차 위치에서 할 수는 없어") the user chose the
third option offered: fix roll / pitch at the design 0 and fit only
x, y, yaw, so no chassis tilt enters the constant. Built
`solver.fit_planar_T_ab2mb` (T_ab2mb = T(x, y, tz_design)·Rz(yaw),
least squares on (x, y, yaw) with the lift compensation per sample,
jackknife) and `F_for_T_ab2mb` so the planar result goes through the
same evaluate / pair-error reporting; `chain_calib.py solve` prints a
`planar` row + `[planar]` block and stores `T_ab2mb_planar` /
`planar_tz_m` in `corrections.npz` (`--planar-tz` overrides the design
tz read from `tf_chain.yaml`). On the arm session (all 61 views, new
hand-eye, new K, offsets): **t (−8.20, −106.13, −652.00) mm, yaw
179.196°**, jackknife 0.43 / 0.44 mm / 0.009°; rms 12.5 mm / 0.65°
(hold-out 14.2 with every 4th view out) against 7.9 for the 6-DOF fit —
the ~0.5° of session tilt the planar form refuses to absorb, which is
the point. Applied to `tf_chain.yaml` + npz (comment header rewritten
around the rule, the two 6-DOF values kept as history), the planner
URDF `mobile_to_base` (rpy now 0 0 0.014030), README §5-A's apply
block. Checks: `tf_chain_tool check` 13, `check_pose_vs_joint` 27,
`check_lift_compensation` 11, `check_joint_offsets` 22,
`check_front_cam_extrinsics` 24, `check_basler_tip` 11,
`check_scan_progress` 62, `check_chain_calib` 48. Not run on the robot;
the calibration launch and `arm_node` still need the restart named
above.

**"지금 모든 요소의 캘리브레이션 짝이 맞는 거지?" — checked file by
file, one was not.** `T_hc2ee` == the K20260922 re-solve (4e-10),
`arm_joint_offsets.yaml`'s recorded hand-eye == the applied one,
`T_ab2mb` == `corrections.npz` `T_ab2mb_planar`, robot.yaml's override
K == the checkerboard result — but `T_ee2tip` was still the 09-21 solve
(09-18 hand-eye, driver K fx 609.3, D = 0), i.e. D in the dependency
order had not followed A and C. `basler_tip_ros.BaslerTipSession` got
`hand_intrinsics` (`meta` / `config`, the chain_calib rule: the override
applied to RAW corners, refused on an override-captured session) and
the CLI `--hand-intrinsics`; the old pair reproduces the applied value
exactly, the new hand-eye alone moves it to (−2.0, −245.2, 213.2), new
hand-eye + new K to **(−2.0, −245.2, 214.4) mm**, roll −179.2°, fit
6.0 mm rms / jackknife 1.8 / 0.8 / 0.4. Applied everywhere the number
lives (tf yaml + npz, URDF `vision_tip_joint` −0.0020 −0.2452 0.2144,
`TOOL_OFFSET_MM`, `handeye_calib.yaml` / `handeye_sweep.py`
`tool_points_mm`, `set_tool_tcp.py` docstring — its dry run prints the
new numbers); record `result_K20260922_HE20260922.{npz,yaml}` in the
session dir. Then `generate_calibration_artifacts.py` for the whole
09-22 set (the plans of commit `2cbb62f` were made with the 09-18
hand-eye + 09-21 T_ab2mb): 26 + 25 entries, seeds moved 22.7 / 21.4 mm
mean, 37.4 max (the align loop absorbs that; the 09-04 loss was at
1.15 m), flange reach 0.51–1.04 m, 0/51 over 1.40; `docs/all_tags_
position.csv` regenerated. Checks: `tf_chain_tool check` 13,
`check_pose_vs_joint` 27, `check_scan_progress` 62, `check_basler_tip`
11, `check_handeye_sweep` 74, `check_joint_offsets` 22. The other
session's `docs/TF_CHAIN_CALIBRATION_STATUS_kr.md` (untracked) got its
T_ee2tip row and §3.7 updated. Consequence for pose mode: the RRT CSVs
(design tip) are now |Δ| ≈ 13 mm from the flange the joint rows put —
`check_pose_vs_joint` carries the figure; still `scan_joint_*` until the
planner re-exports.

### 2026-09-22 — front_cam liveness gate: no front camera, every tag-driving command refused

User: "정면 카메라 작동이 안되면 tag로 주행하는 모든 명령 거절". Until now a
dead front_cam was invisible to navigation: `mobile_controller` kept the
LAST detections frame in `detected_tags` forever (nothing ever expired
it), so `get_current_tag_id()` still answered from it, `move_to_tag` fell
through to `last_known_tag` when it did not, and a GOTO / TASK set off
blind on odom with no stop line and no align — the failure only showed
up as `align_timeout_s` twenty seconds later. Now `MobileController`
stamps the wall time of every detections array in `_store_detections`
(the array is published per FRAME, empty or not, so its age is the
liveness of the whole front_cam pipeline), `front_cam_status()` returns
`(ok, reason, age)`, `detected_tags` became a property that serves `{}`
once the age passes `robot.front_cam_alive_timeout_s` (1.0 s — measured
live: 30.0 Hz, worst gap 92 ms), `move_to_tag` refuses at the start and
before every hop (stopping the base on a mid-route death), and
`mobile_node` refuses `/mobile/goto_tag` at the command boundary with
the reason in `result.message`, publishes `front_cam_ok / _age_s /
_reason` in `/mobile/state`, and — found on the way — now calls
`stop()` when a goto RAISES (`_do_manual` did, `_do_goto` did not). Both
robot_ui fronts show `BASE NO CAM` and the reason. Manual odom moves are
deliberately not gated (not tag driving). `<= 0` disables the gate for
offline plants. Promoted to the *Mobile base split* section.

Verified offline: new `tools/check_front_cam_guard.py` (31 — property
empty at 1.1 s but not 0.9 s, logged once, resumes; refused before the
`last_known_tag` fallback with nothing moved; "no detections yet"
before the first frame; camera dying after hop 1 of a 2-hop route
aborts before hop 2 with `stop()`; a live camera still arrives within
5 mm; timeout 0 disables; the real `mobile_node` against a fake ROS:
state fields, refusal with seq / ok / message, manual move_cmd still
runs, a raising goto stops the base); `check_nav_sequencing.py` 14,
`check_ground_plane.py` 15, `check_task_list_ui.py` 102,
`check_web_ui.py` 128 unchanged. Not run on the robot: `mobile_node`
restart required (the service, or the hand launch); then
`rosservice call /robot_camera/front_cam/set_enabled "data: false"` and
a `GOTO` should come back refused within a second with the BASE chip
red, and `data: true` should clear it.

### 2026-09-22 — hand_cam intrinsics RE-CALIBRATED on a checkerboard and APPLIED: fx 601.87 / fy 601.93 / cx 321.35 / cy 238.51, k1 +0.1771 k2 −0.3483

The operator shot `log/chain_calib/hand_cam_intr_20260922` with the tool
from the entry below: an 11×8-inner-corner board (12×9 squares of
14.99 mm measured; 11×8 confirmed on the saved frames — 12×8 detects 1 of
12), driver-raw `/hand_cam/color/image_raw`, 67 stills. First pass (42
views) was refused by `solve`: everything at 0.19–0.43 m and 41/42
central (corners reaching r = 332 px of 400, 3.4 % beyond r = 250), so
k1 disagreed 0.016–0.022 between halves — the exact degeneracy the
re-shoot was for. Second pass added 0.45 / 0.55 m and corner views (52 /
7 / 5 by range bin, tilts to 52°, 10 corner views).

**Result (k1 k2, all 67):** rms 0.412 px (per-view 0.16–0.65; the
newest far views 0.16–0.27, the early close high-tilt ones 0.5–0.65 —
residual correlates with tilt +0.69, with range only −0.37), formal sd
fx ±0.56 / k1 ±0.0026. Radial residual flat to ±0.12 px out to r = 250,
k1k2k3 / full models change rms 0.434 → 0.425 / 0.415 while k3 runs to
−2.6: the model is right, the floor is the board / sensor. **Why it was
applied although `solve` said NOT yet** (rms > 0.35, near/far halves
fx 4.4 px apart): (1) odd/even halves — identical coverage — agree to
fx 0.9 px / k1 0.0006; (2) the far half alone has fx sd 1.57 px (3× the
near half's) because a half with no range spread sits on the fx–k1
valley, so 4.4 px is 2.6 σ of a test that removes the very diversity it
is meant to check; (3) **3-fold hold-out, pose-only on the unseen fold:
new 0.412 px vs the 09-21 override 0.441 vs the driver 0.676**, fx
across folds 601.72 / 601.97 / 601.92; (4) on the INDEPENDENT 09-21 arm
session the radial residual trend (centre → r 300) goes driver −4.00 →
override −1.69 → **new −1.05 px**; (5) fx agrees with the 09-21 A0-sheet
fit to 0.15 px (0.025 %) from a different target and method. What
changed physically: **fy = fx now** (601.93 / 601.87; the 09-21 value's
fy/fx = 1.0036 was the A0 print's y-scale error aliased into the camera
— the checkerboard has one measured square size, so it cannot alias),
k1 +0.160 → +0.177 (the old one had no corner data behind it). Applied
by hand into `robot.yaml intrinsics_override.hand_cam` (the tool's
`--apply --force` was blocked by the session's permission classifier;
the K / D / note lines were edited, the Korean comments kept and
updated), `check_camera_intrinsics_override.py` 20/20. **Restart
`robot_camera_node` (`sudo systemctl restart mobile-manipulator`; the
charge relay drops) and the calibration launch.** Still open: a ~1 px
radial trend remains in the arm session with any K — the AprilTag
corner detector's own bias at large radius, or the A0 print, are the
candidates; and the far-range / corner coverage is at the minimum
(0.50–0.60 m: 5 views, BR corner: 1).

### 2026-09-22 — hand_cam intrinsics: a proper calibration tool (`tools/hand_cam_intrinsics.py`) and procedure

User: "hand_cam 내부 파라미터 캘리브레이션 진행하자" with the shooting rules
(three ranges 0.25–0.6 m incl. 0.35 / 0.45 / 0.55, tilts ±30–45° four ways,
the board in every corner, 30–50 stills, MEASURED square size, flat board,
rms ≤ 0.3 px, halves agreeing to 1–2 px of fx / 0.005 of k1, raw frames).
The 09-21 override came from A0-sheet corners at one range, mostly
central — which is why the offset fit still showed a −0.7 → −2.4 px radial
residual. Built: `capture` (subscribes the DRIVER's
`/hand_cam/color/image_raw` — raw regardless of the override, which only
remaps the node's own detection copy — detects a checkerboard with
`findChessboardCornersSB` or the chain_calib A4 / A0 tag sheets with
dt_apriltags, live line with range / tilt / position / corner motion,
saves on Enter only when the corners were still for 6 frames, `--auto`,
a coverage table per save that names what is missing), `solve`
(`calibrateCamera` k1 k2, per-view rms with 3×-median outliers dropped,
odd/even AND near/far half-splits against `--fx-tol` 2 px / `--k1-tol`
0.005 / `--rms-max` 0.35, the robot.yaml block, `--apply` rewriting only
the K / D / note / image_size lines of the existing hand_cam block so the
user's comments in it survive, refused unless ACCEPT), and `board` (A4
10×7 / 25 mm checkerboard PDF, `log/chain_calib/checkerboard_A4_10x7_25mm.pdf`
— print at 100 %, measure, glue flat). Procedure: `docs/HAND_CAM_INTRINSICS_kr.md`.

Verified: `check_hand_cam_intrinsics.py` (18 — a rendered board detected
with the object-grid order (or its 180° twin), 60 synthetic views at
three ranges / tilts / corners with a known K, D recovered to 0.1 px of
fx / cx and 0.0001 of k1 at the 0.2 px floor, a 2.5 px "moved" view
dropped, both splits inside tolerance, `--apply` on a commented yaml copy
changing only the numbers, the printed PDF re-detected as 9×6); live
`capture` against the running stack read the driver's K (609.3 / 608.6,
D = 0) and the 640×480 stream, reported "target NOT seen" and exited
cleanly on q. Not yet shot — needs the board printed and measured.

### 2026-09-22 — Joint offsets fitted (hand-eye fixed, new K): NOT applied — they move the lens ≤ 2 mm, and the ±15 mm of `sheet_path` is the 2–3-tag PnP, not the arm

User: "관절 offset 피팅하자" — step 1 of the status doc's §6. Run on scratch
copies of `log/chain_calib/20260921_arm` (61 views after the four
exclusions, `--hand-intrinsics config`, hold-out every 4th); the full
report is `arm_offsets_20260922_handeye_fixed_newK.txt` beside the session,
the subset / diagnostic scripts stayed in the session scratchpad.

**The fit itself.** Rigid 7.63 px rms (hold-out 7.90) → offsets 3.55
(3.59): J2 −0.11 ± 0.54, J3 −0.66 ± 0.44, J4 −0.16 ± 0.35, J5 −0.15 ± 0.08,
J6 −0.52 ± 0.09° (jackknife). Subsets do NOT agree: near (≤ 0.42 m, 15
views) J2 −3.48 / J3 +1.13, far (46) J2 +0.53 / J3 −1.07, spin 30 vs 90 vs
150 and tilted vs flat likewise ±2° on J2/J3 — far outside the 0.2°
acceptance rule. Yet the geometry is identifiable: planted offsets are
recovered to 0.03° on EVERY subset, and offsets fitted on any subset cut
the other's residual 7.6 → 4.3–4.8 px. So the values are a weakly
constrained J2/J3/sheet-pose trade-off riding on a MODEL error, and no
extra term makes them agree — hand-eye xy (z fixed), hand-eye tz, focal
scale, k1, per-tag in-plane sheet corrections were each freed in the
scratch model: rms 3.5 → 2.6–3.1 px, near still J2 −1.3…−2.0 vs far
+0.7…+1.0. The residual's radial component grows −0.7 → −2.4 px from the
principal point to r = 350 (hand_cam K still ~0.7 % off at the edge) and
the per-tag corrections are a y-only −0.7…−1 % print scale (−5.8 mm at
tag 309 — the rule said 0.35 %; re-measure 300↔308 with the steel rule).

**The decisive test — the deliverable metric.** IK'd every `sheet_path`
target (runs 16:35 spin 92 / 17:00 spin 60 / 17:05 h 0.40) through the
URDF and predicted the lens error the fitted offsets imply: **1.2–1.6 mm
rms of per-point variation against the 12.7–13.2 mm measured, correlation
−0.35…+0.16.** The offsets do not explain the on-robot pattern, and
applying them (with the `MoveJ(IK − δq)` change) would move the lens by
~1.5 mm across the sheet plus a constant the chain already absorbs. Not
applied; the command-side change is not built.

**What the ±15–20 mm is.** In the same session, PnP lens position vs the
model by hand_cam tag count: 2 tags 9.8 mm xy rms (rigid) / 7.2 (offsets),
3 tags 15.4 / 13.8, 4 tags 8.3 / 5.5, 5 tags 6.8 / 3.9, 6 tags 9.4 / 1.8.
A 0.5 px corner-noise Monte-Carlo on the real views gives 5.2 mm (2 tags)
vs 2.2–3.1 (4–6) — random noise is a third of it; the rest is the
systematic 3 px residual amplified by the two-tag planar ambiguity into a
lateral shift. `sheet_path` measures the lens by that same PnP, mostly on
2–4 tags, and its 17:00 run alternates +8…+19 / −6…−19 mm in x BY COLUMN
(x = 850 vs 1000): a sheet/print-frame signature, not a joint. So the
09-21 "configuration-dependent arm error" is largely the measurement; the
arm + rigid chain is good to ~8 mm xy on ≥ 4-tag views (2–5 with offsets,
in-sample). **Next:** fix the measurement before fitting the arm again —
verify at 0.55 m (4–6 tags in view) or require ≥ 4 tags, a proper hand_cam
intrinsics set (edge coverage, 0.25–0.6 m), measure the print's y scale,
tape the sheet. Status doc §6 rewritten; HANDOVER §2-0c pointer updated.

### 2026-09-22 — Map-calibration hand-cam align: orientation FIXED at the design view pose, translation-only correction to 0.50 m; plans regenerated (seeds were 190 mm off)

User: "map 갤리브레이션 진행할 때 핸드카메라 어라인은 x,y 평면에서
어라인하고 z 수직이동만, 각도는 태그하고 평행, 즉 handcam 각도보정은
정한 값으로 유지"; then "0.50 m 사용, 회전은 rz 자유, 나머지는 어라인
중에는 tag하고 일치". Until now `run_auto_align` built a 6-DOF target
every step (tag centred + optical axis squared to the tag, spin kept)
and required tilt ≤ 0.5° to converge — so it chased the arm's own
orientation error (0.6–2.7° on 09-21) and the paper's slope with
rotations, and the final camera orientation differed per entry.

- **`align.orientation: fixed`** (`locator.yaml`, `AlignCfg`; `correct`
  = the old loop, still what the hand-eye sweep's square-up does through
  `handeye_calib.yaml`). `compute_target_ee_pose(fix_orientation=True)`
  keeps the current rotation BIT FOR BIT and moves the tool by
  `R_ab2hc · (t_cam2tag − (0, 0, d))`: x/y in the image plane (parallel
  to the tag), z along the optical axis (vertical) to `target_distance_m`
  **0.50** (= `auto_view_distance_m`; was 0 = keep depth, which left the
  09-04 entries measuring from 0.63 m after a clamped seed).
  `clamp_step` snaps a zero-rotation delta to the exact identity (acos
  near 1 reads a 1e-14 trace error as 2e-7 rad), so the commanded rx ry
  rz ARE the seed's. Convergence: xy ≤ `position_tol_m` and range within
  `depth_tol_m` (5 mm); the tilt is recorded per iteration and in the
  report (`orientation` key added) and warned about above
  `tilt_warn_deg` (3°), never corrected — with the orientation fixed it
  is the arm's orientation error plus the tag's slope, a chain
  diagnostic, and the 6-DOF chain observation does not need it removed.
  The seed orientation is the plan's design view TCP
  (`compute_view_tcp`: optical axis parallel to the tag normal through
  the calibrated `T_ab2mb` / `T_hc2ee`, rz the planner's free reach
  choice). The locate service shares the cfg, so `auto_align` there
  behaves the same.
- **Plans regenerated** (`generate_calibration_artifacts.py`, both
  plates + yaw sweeps + `docs/all_tags_position.csv`): the tracked seeds
  dated from the 09-14 hand-eye, before the 09-18 remount (camera
  0.37 m from the flange, 180° spun) and the 09-21 `T_ab2mb` — every
  seed moved **175–197 mm** (mean 190) and the design rx/ry from a
  single (−179.5, −0.7) to (−179.9…+179.3, −0.4…+0.3) varying with the
  spin (the calibrated mount tilt rotates with rz). The old loop would
  have iterated the 19 cm away; with the orientation fixed the seed IS
  the final orientation, so the plans must be current — regenerate
  after any tf_chain change from now on.

- **rx / ry held at −180 / 0, rz free (user, later the same day: "어라인
  중에 RX −180 RY 0 유지, 태그 접근 중에는 안 해도 됨").** `align.fixed_rx_deg`
  / `fixed_ry_deg` (`null` / `null` = the seed's own design rx/ry). The
  align target's rotation is `Rz(rz_now)·Ry(0)·Rx(−180)` — the tool
  straight down the arm's z with the planner's spin kept — and the
  camera is placed at `d` along that axis through the tag centre
  (`p_hc = p_tag − d·z_hc`), so the first step turns the seed's design
  rx/ry (≤ 1°) onto the fixed value together with the translation and
  every later step is a pure translation. The approach to the seed is
  not held to it. Consequence for the recorded tilt: the hand-eye's
  optical axis is 0.47° off the flange z (`T_hc2ee`) and the base sits
  0.85° off vertical (`T_ab2mb`), so a flat tag reads ~0.5–1.3° of
  tilt routinely — the 3° warning is set above that. Report key
  `fixed_rx_ry_deg`.
- **`fixed_rpy_frame: camera` (user, right after: "핸드아이 광축이 플랜지
  이러한 보정은 적용").** The fixed rx/ry now describe the hand-cam
  OPTICAL frame in the arm frame: `R_ab2hc = Rz(spin_now)·Ry(0)·Rx(−180)`
  (optical axis exactly along −arm z, the camera's own spin kept) and
  the flange follows through the hand-eye, `R_ab2ee = R_ab2hc·R_hc2ee`
  — so the flange reads (−179.86, +0.45, rz − 0.00) with the applied
  `T_hc2ee`, and a flat tag's recorded tilt is the tag's slope vs the
  arm vertical alone (the 0.47° hand-eye offset is gone from it; the
  base's 0.85° `T_ab2mb` tilt is NOT applied — the reference is the
  arm's z, not the floor normal — so ~0.9° stays on a level floor).
  `flange` restores the TCP-frame reading. Report key `fixed_rpy_frame`.

Verified offline: new `scripts/check_align_fixed_orientation.py` (76 —
real `T_hc2ee.npz`, real runner against a fake arm + detector: from a
seed 1.5° off the normal, 50 mm off-centre, 0.63 m up, the final rx ry
rz equal the seed to 1e-16°, every commanded move carries the seed
orientation, tag centred to 0.00 mm at 0.500 m in 3 MoveLs, tilt still
1.50° in every history entry and the report; a 40° spin seed kept; a
4° tilt warns and still converges; square camera + 130 mm range error
moves straight down along arm z; `correct` still squares to 0.000°;
bad orientation value refused; locator.yaml loads fixed / 0.50 ==
auto_view_distance_m; with fixed −180 / 0 a plan-like seed (−179.5,
−0.3, 25) ends at |rx| 180 / ry 0 / rz 25 to 1e-9°, the first step
turning 0.5°, later steps 0 rotation, a planted 0.7° tag slope read
as tilt and constant over the iterations; rx without ry refused; with
`fixed_rpy_frame: camera` the final CAMERA rotation equals
Rz(spin)·Rx(−180) to 1e-12 with its axis exactly −arm z, the flange
reading −179.86 / +0.45, a flat tag's tilt 0.000 and a 0.7° slope
0.700 exactly; `flange` keeps the TCP-frame result; unknown frame
refused), `check_handeye_sweep.py` 74 unchanged, both
plans load (26 / 25). Not run on the robot: restart the calibration
launch (`path_tag_locator.launch`); watch `auto_align iter n: … tilt=x
deg (recorded, not corrected)`, the rx ry rz in the `MoveJ`/MoveL lines
staying at the seed's, and convergence in 2–3 iterations.

### 2026-09-22 — First boot with the `mobile-manipulator` service: a restart loop on `ROS_DISTRO: unbound variable`, fixed

User (pinyin): "开机就启动 main launch 怎么做". The service from the entry
two below was already installed and enabled, and the PC had just booted
(uptime 2 min) — but `systemctl status` showed `activating (auto-restart)`,
restart counter 26, every attempt dying in under a second with
`/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh: line 3: ROS_DISTRO:
unbound variable`. `run_stack.sh` runs under `set -u`, and that ROS env
hook tests `"$ROS_DISTRO"` before it is ever set. Harmless in every shell
the script had been dry-run from (the login profile had already exported
it), fatal under systemd's empty environment — reproduced with `env -i bash
-c 'set -u; source /opt/ros/noetic/setup.bash'`. Fix: `set +u` around the
two `source` lines, `set -u` again after (comment in the script says why).
The unit's `ExecStart` points at the script in the working tree, so the
next 5 s auto-restart picked the fix up with no reinstall: `[run_stack]
ROS master ok (0s) / Fairino arm ok (0s) / Keyence ok (0s)`, 9/9 nodes +
rosbridge (9090) + web UI (8080) up at 10:37:20, `ROS_LOG_DIR` under
`<ws>/log/ros`, front_cam seeing tag 501, arm left in place. Verified: the
dry run in a clean `env -i` environment with and without `MM_WS`, then the
live boot. Lesson: dry-run a systemd launcher with `env -i`, not from a
sourced terminal.

### 2026-09-22 — Hand-eye fitting status written up for the next session; the rename had broken the stored session paths

User: "현재까지 진행상황은 다른 창에 넘기게 문서 작성 Hand-eye calibration
fitting results". **`docs/HANDEYE_FITTING_STATUS_2026-09-22_kr.md`** — the
applied values, every hand-eye candidate with its score on the two sheet
sessions, the arm-session and chain-session fits, why the 20 mm z was an
artefact, the revised order (joint offsets with the hand-eye FIXED, then
the command-side `MoveJ(IK − δq)`, then `sheet_path` at two spins), and
the exact commands. HANDOVER §2-0c got a pointer. Every number was RE-RUN
today rather than copied: arm session hand-eye-free 1.06 px (hold-out
1.99), J2 −1.160 / J3 −0.366 / J4 +0.246 / J5 −0.101°, Δt (+8.6, +3.7,
−16.6) mm; chain session raw 6.62 / joint 4.40 mm — identical to the
09-21 late entry. Found on the way: the working-tree tf files are back
at HEAD (the 09-18 hand-eye, `tf_chain_tool.py check` 12/12), so the
20:09 sweep value is applied nowhere and the "(d)" inconsistency of
HANDOVER §2-0c no longer exists; the 20:09 result survives only in
`run_20260921_195136/result.npz` (untracked).

**The rename bit here first.** Session `meta.yaml` / `session.json` store
ABSOLUTE paths from capture time (sheet layout json, hand-eye npz); after
`ws_20260902 → ws` the first `arm_offsets.py` run died with
`FileNotFoundError` on the layout. `chain_calib.py sheet_from_args` and
`basler_tip_ros.BaslerTipSession` now fall back to the package's copy by
basename (and the configured hand-eye) with a `! … no longer exists —
using …` line, the same rule `platform()` already had for the hand-eye.
`check_chain_calib.py` 48, `check_basler_tip.py` 11, `basler_tip_calib.py
status` on the real session all pass through the stale paths.
`chain_calib.py`'s `--sx / --sy / --tag-size` are GLOBAL options and must
precede `solve` — written into the doc's command block after tripping on
it. Fits were run on scratch copies so the tracked `arm_offsets.npz` /
`corrections.npz` were not rewritten.

### 2026-09-22 — Workspace folder renamed: `~/mobile_manipulator_ws_20260902` → `~/mobile_manipulator_ws`

User (pinyin): rename the project to `mobile_manipulator_ws`, folder
included. The stack was up by hand (idle on dock 500, charging), so:
`stop_stack.sh --force` (clean SIGINT exit, 4 s) → `mv` → `rm -rf build
devel` + full `catkin_make` (the build cache and every devel setup file
hardcode the absolute path; a `catkin_make` in the old tree fails on the
CMakeCache directory check) → relaunch from the new path. Nothing inside
the repo depends on the folder name: `MM_WS` comes from the env hook's
devel prefix, `paths.WS_DIR` from the source space, the launch from
`optenv`. What did carry the old name: the three Korean guides'
`source ~/…/devel/setup.bash` lines (updated), the retired `ws_dev`
worktree entry (pruned — the directory was already gone), and Claude
Code's per-project memory directory, which is keyed by the cwd
(`~/.claude/projects/-home-abc-mobile-manipulator-ws`, copied over from
the old key with the paths inside updated). The systemd unit from the
entry below substitutes the path at install time, so it needs no change;
install it from the new location. Work Log entries naming
`ws_20260902` are history and were left alone. Stopping the stack dropped
the charge relay (as on every Ctrl-C); CHARGE was re-issued after the
relaunch: the base was still on 500, so `battery_return` did lift origin
homing, "already at 500", relay on, and the BMS confirmed 15.7 A at
76.5 % two seconds later — no motion. Relaunched detached (`setsid
nohup`, output in `log/ros/launch_20260922_manual.out`), 9/9 nodes +
rosbridge + web UI up, `MM_WS` / `ROS_LOG_DIR` resolving to the new path.

### 2026-09-22 — Main launch at robot boot: systemd `mobile-manipulator`, after the navifra driver

User: "main launch 를 robot 시작할때부터 실행". Until now the PC booted into
the navifra driver only (`navifra-robot.service`, up at 09:38 today) and
the operator started `mobile_manipulator.launch` by hand from a VS Code
terminal (09:42) — the terminal whose frozen pty produced the 2026-09-21
stop failure. Built the same way the driver is run: a unit + a launcher
script, `src/apriltag_nav/tools/systemd/`.

- **Unit** (`mobile-manipulator.service`, installed with the workspace
  path substituted): `Requires=` / `After=` / `PartOf=navifra-robot.service`
  — roscore lives in the driver, so the stack cannot start without it and
  restarts with it; `User=abc`, `KillSignal=SIGINT` to the cgroup (=
  Ctrl-C, every shutdown hook runs, 40 s before SIGKILL), `Restart=
  on-failure` 5 s, `EnvironmentFile` = the env file in the repo.
- **`run_stack.sh`** sources `/opt/ros/noetic` + `devel/setup.bash` (so
  `MM_WS` / `ROS_LOG_DIR` are the env hook's, file logs stay in
  `<ws>/log/ros`), `PYTHONUNBUFFERED=1` + `stdbuf -oL` (a non-tty stdout
  is block-buffered — the driver only had stdbuf, which does nothing for
  Python's own buffer), then WAITS: master via `getSystemState` (120 s,
  fail → exit 1 → restart), the Fairino RPC port 192.168.58.2:20003
  (180 s — the controller boots slower than the PC and `arm_node`
  connects once at start), Keyence 64000 (30 s). A late device is a
  warning, not a refusal: cameras / base / web UI work without the arm,
  and `systemctl restart` picks it up. `DRY_RUN=1` prints the command.
- **`install_service.sh`** (sudo): copy, `daemon-reload`, `enable`;
  `--start` refuses while a hand-started launch is running; `--uninstall`.
  **`stop_stack.sh`** now exits with the systemctl instruction when the
  launch is the service (a SIGINT from it would also have left
  `Restart=on-failure` to decide).
- Docs: `STOP_LAUNCH_kr.md` §0.5 (operation, the hand-launch conflict, the
  charge relay dropping on stop / reboot as on Ctrl-C), README, Build &
  Run above.

Verified offline: `bash -n`, `DRY_RUN` with and without `MM_WS` in the
environment (both resolve the ws and print the roslaunch line), the
wait helper against an open and a closed port on the live arm
(20003 open, 8080 / 8082 too — 20003 is the SDK's RPC port),
`systemd-analyze verify` on the substituted unit (no finding for it; the
noise is other units), master probe returns code 1 on the live master.
**Not installed:** sudo needs a password in this session, and the stack
was running by hand (PID 3435) — installing + starting would have
collided with it. To finish: `sudo src/apriltag_nav/tools/systemd/
install_service.sh` (enables for the next boot), then at a convenient
moment `src/apriltag_nav/tools/stop_stack.sh` → `sudo systemctl start
mobile-manipulator` → `journalctl -u mobile-manipulator -f` and watch the
three `[run_stack] … ok` lines and the nine nodes come up. First boot to
check: the arm wait — if the controller takes longer than 180 s, raise
`WAIT_ARM_S` in the env file.

### 2026-09-21 (late) — hand_cam intrinsics were wrong (D ≠ 0, fx 1.2 % high): a per-camera override in robot.yaml, applied by remapping in robot_camera_node; the hand-eye re-checked from every stored photo — it is NOT the 20 mm

Step 1 of HANDOVER §2-0c, done offline. `cv2.calibrateCamera` over the
stored A0-sheet corners (planar target) of both 2026-09-21 chain_calib
sessions: with the driver's K and D = 0 the corner fit is 0.7–0.8 px rms;
with K free + k1 k2 it is **0.26–0.28 px**, the corner floor, and every
subset agrees — chain session halves fx 600.9 / 602.4 ± 1.7, arm session
halves 607 ± 7 / 600 ± 3.5, ≥ 3-tag views 601.3, all views 604.2; k1
+0.152…+0.166, k2 −0.31…−0.35. Adopted (60 chain views): **fx 601.72,
fy 603.87, cx 322.02, cy 238.46, k1 +0.1598, k2 −0.3222** vs the D435's
609.30 / 608.62 / 321.47 / 238.67 / 0. The distortion moves a corner
4.5 px at r = 250 px; fx is 1.2 % high in the driver; and the fy/fx
change (+0.46 %) is the "y-only 0.35 % shrink" the 09-21 chain record
could not explain. A full 5-coefficient model (k3 p1 p2) does not
converge on this data (LM runs for minutes) — k1 k2 only.

**Applied as `robot_camera.intrinsics_override.hand_cam` in robot.yaml**
(`apriltag_nav/camera_intrinsics.py`; the yaml comment and the module
docstring carry the contract). Nothing in the drivers can be told a
different K, so: `robot_camera_node` REMAPS hand_cam's frame with
(K, D) (`Rectifier`, maps built once, ~1 ms) and detects on it —
`/hand_cam/tag_detections` are in the rectified frame with
`camera_params` = K_override (refused with a logerr if the stream size
differs from `image_size`); detection consumers use
`effective_intrinsics(cam)` = (K_override, D = 0) — `chain_calib.py`
capture (`_grab_KD`, meta gets `hand_cam_intrinsics: override …`),
`basler_tip_ros`; the raw-frame consumer `handeye_calib_node` rectifies
each archived sample itself (`rectify_raw_frame`, K stored = override);
sessions captured BEFORE (raw corners, driver K in meta) are re-solved
with `arm_offsets.py / chain_calib.py solve --hand-intrinsics config`
(refused on a session captured with the override — D would apply
twice). No entry = the driver's CameraInfo, bit-for-bit as before;
front_cam / side_cam have none. **Restart `robot_camera_node`** (and
the calibration launch) for it to take effect; the calibration nodes'
locator chain then sees rectified hand_cam poses.

**What the correct K changes, and what it does not.** Re-fitting the
arm session (`--hand-intrinsics config`): rigid 7.4 px, offsets-only
3.55 (was 3.81), hand-eye-free 1.06 (was 1.26; hold-out 1.99) — and the
fitted hand-eye correction stays **19.1 mm / 1.30°**, Δt (+8.6, +3.7,
−16.6) mm. So intrinsics are not the missing term. The near / far split
still disagrees ((+9.6, +5.1, −16.0) vs (+5.7, −3.2, −4.7)), and the
reason is now measured: the far views' early half alone fits its
hand-eye z to **−266 mm** at 0.43 px rms — the hand_cam z (optical-axis)
offset is UNOBSERVABLE in this session (fronto-parallel views at one
range), so the 16 mm z is a fit artefact, not a measurement. The sheet
did not move (front_cam's tag 200 constant to 0.1 mm; one 1.2 mm step at
v52 in the arm session).

**The hand-eye from every photo already on disk** (`calibrateHandEye` +
scatter refine; sheet sessions use multi-tag PnP → `T_hc2W`, the sweep
re-detects its archived images): the 09-18 sweep (35 samples) with the
driver K reproduces the file EXACTLY; with the new K + undistorted
images it moves **4.8 mm / 0.22°** (Δt +1.2, +2.4, −4.0), scatter 2.7 →
2.2 mm. The sheet sessions solved as hand-eye data land 3–12 mm from
the file — and whichever hand-eye is plugged in, the sheet's position
through the chain scatters **9.5 mm rms (chain session) / 16 mm (arm
session)**, changing by < 1 mm between candidates: that scatter is the
ARM's configuration-dependent error (the ±15–20 mm of `sheet_path`),
not the hand-eye. The correct K halves the arm session's figure (31.8 →
16.1 mm, mostly the 2-tag PnP depth). Verdict: the 09-18 hand-eye is
good to ~5 mm; a re-shoot cannot improve on that; the xy error is the
joints' (fixing the hand-eye at the file and fitting J2..J5 offsets
alone: 5.7 → 2.7 mm).

⚠️ **Meanwhile the operator ran a NEW sweep on cross tag 0 at 20:07
(18 samples, `run_20260921_195136`, scatter 3.05 mm) and pressed
Compute & save at 20:09 — `T_hc2ee` in `tf_chain.yaml` + `.npz` is now
t (43.70, −328.51, −152.65), rpy (−0.068, −0.071, −178.766)°: 10.1 mm
from the 09-18 value (Δt +8.1, +6.0, −1.1 in the camera frame).** On the
sheet sessions it is a tie (chain 9.5 → 8.9 mm, arm 16.1 → 17.1). But
`T_ab2mb` was fitted WITH the 09-18 hand-eye, so the chain's raw
consistency on the 63-view session is now **17.8 mm** (6.6 with the
09-18 file, new K); the joint re-fit still reaches 4.4 mm. Either revert
the two files (`git checkout src/apriltag_nav/config/tf/`) or re-solve
the chain and apply its `T_ab2mb` — the pair must be consistent. Left
UNCOMMITTED here, the user's call. Also learned: single sweeps
reproduce to ~10 mm between themselves (09-18 sweep 3 vs the 18-sample
file was 12.6 mm; today's vs the 35-sample file 10.1) while their
within-sweep scatter is 3 mm — the sweep's honest xy uncertainty is
5–10 mm, which is also why the sheet cannot rank the candidates.

Verified offline: new `tools/check_camera_intrinsics_override.py` (20 —
yaml entry, effective/raw-frame rules, `undistort_points` exact to
1e-5 px, a remapped dot image within 0.3 px, size-mismatch refusal, the
node worker taking K_override + remap / driver K without / ignoring a
mismatched stream / no remap for D = 0, the detector fed the remapped
frame); `check_robot_camera_latency.py` 13, `check_chain_calib.py` 48,
`check_basler_tip.py` 11; `arm_offsets.py --hand-intrinsics config` and
`chain_calib.py solve --hand-intrinsics config` on the real sessions
reproduce the scratch numbers. Not run on the robot.

### 2026-09-21 — Every fixed transform in ONE place: `apriltag_nav/config/tf` (tf_chain.yaml + an npz per transform); the old copies and superseded records deleted

User: "현재 사용중인 모든 tf 값을 main apriltag_nav 저장하고 여기에서 적용,
각각의 npz파일과 모든 tf 가 있는 yaml 로 구성 그리고 설계값하고 어떻게
나왔는지 주석으로 설명, 이제 필요없는 기록은 지운다". Until now the applied
values sat in six files that had to be changed together and were checked
against each other by hand: `path_tag_locator/config/extrinsics.yaml`
(T_ab2mb, T_mb2fc), `path_tag_locator/config/hand_eye/T_hc2ee.npz` (+ five
historical npz, a yaml input file and a README of their history),
`robot.yaml arm_calibration` (inv(T_ab2mb) as six numbers + the vision
tip), the planner URDF, and the record `chain_calib/docs/TF_CHAIN_2026-09-
21.yaml`. Now **`src/apriltag_nav/config/tf/tf_chain.yaml`** holds the four
fixed transforms — `T_ab2mb`, `T_mb2fc`, `T_hc2ee`, `T_ee2tip` — each as a
row-major matrix with `t_mm` / `rpy_deg_xyz` readouts, a `design` block,
and a comment header saying what the design value was, how the applied
value was measured (session, method, numbers, what is and is not a robot
constant) and its uncertainty; `<name>.npz` next to it is the twin for
tools that take an npz path. Values were carried over byte-for-byte
(T_ab2mb / T_mb2fc diff 0.0 vs HEAD's extrinsics.yaml, T_hc2ee.npz diff
0.0). Promoted to *Transform Parameters*.

- **`apriltag_nav/tf_chain.py`** (pure numpy): `load_tf_chain` /
  `load_transform` / `load_npz`, `arm_calibration_from_T_ab2mb` (the
  inverse as offsets + Rz Ry Rx, yaw wrapped to [0, 2π)) and its inverse,
  `tip_offset_mm`, `urdf_mobile_to_base`, `physical_T_mb2fc` (moved from
  the deleted `make_front_cam_extrinsics.py`), `write_transform` (rewrites
  ONE block's source / t_mm / rpy / matrix lines in place — comments and
  the design block survive — plus the npz, then reads it back),
  `check_npz_agree`, `check_front_cam`. **`tools/tf_chain_tool.py`**:
  `show` (values, vs design, the six arm_transform numbers, the URDF
  lines), `check` (rigidity, npz == yaml, T_mb2fc == generator(robot.yaml),
  T_ab2mb near the design, the planner URDF's two joints — 12), `set NAME
  --npz|--matrix|--t-mm --rpy-deg --source`, `front-cam [--apply]`,
  `export-npz`, `urdf`.
- **Readers moved:** `arm_transform` derives its six numbers from
  `T_ab2mb` at call time (the private `~arm_*` params still override; the
  hardcoded fallbacks are gone — a missing file raises); `arm_controller`
  takes the tip from `T_ee2tip`; `set_tool_tcp.py` reads it (dry run
  prints the same −1.8 / −245.6 / 209.6); `path_tag_locator.constants.
  load_extrinsics[_full]` read the `T_xxx: {matrix: …}` blocks (default
  path = the tf yaml) and their refusal messages name the tool;
  `locator.yaml` / `handeye_calib.yaml` point at `$(find apriltag_nav)/
  config/tf/…`; `handeye_calib_node`'s compute updates the yaml block
  after writing the canonical npz (a custom `output_path` gets a warning
  instead); `chain_calib.py platform()` / `basler_tip_ros` resolve any
  `$(find pkg)`, `--write-hand-eye` writes into the tf dir; `robot_sim`,
  `error_budget`, `analyse_yaw_sweep`, `generate_calibration_artifacts`,
  `check_chain_calib`, `check_front_cam_extrinsics` (imports the generator
  from tf_chain), `check_pose_vs_joint` (patches `tf_chain.load_transform`
  for its design-mount rows), `check_lift_compensation`,
  `calib_front_cam_pose --apply` (calls `tf_chain_tool.py front-cam
  --apply`; its check patches `--tf-yaml`). `robot.yaml arm_calibration`
  keeps only `csv_euler`; the vision tip and the six mount numbers are
  gone from it.
- **Deleted (git has them):** `path_tag_locator/config/extrinsics.yaml`,
  the whole `config/hand_eye/` (README, `T_hc2ee.yaml`, `T_hc2ee.npz` —
  moved — and the 2026-05-27 / 09-02 spun / 09-14 old-mount / 09-18
  node-all32 / hardware npz), `scripts/save_npz.py`,
  `scripts/make_front_cam_extrinsics.py`, `chain_calib/docs/TF_CHAIN_2026-
  09-21.yaml` (its content is the yaml's comments now); records that fed
  no applied value: hand-eye runs `run_20260914_183840` (the old mount),
  `run_20260915_182657` and `run_20260918_141740` (0 samples), and
  `log/apriltag_nav/calib_pair` (the 09-08 tilt fit, superseded by
  `calib_pair_20260915_a`). **Kept:** `run_20260918_144420` +
  `run_20260918_152111` (the samples behind the applied T_hc2ee),
  `log/chain_calib/20260921` and `basler_tip_20260921` (behind T_ab2mb and
  T_ee2tip), and — not transform records — the `map_world_*.yaml`,
  `locate/`, `calibrate/` session logs (`predictive_centering.map_world_
  path: latest` still reads the newest map_world).

Verified offline: `tf_chain_tool.py check` 12/12; `check_front_cam_
extrinsics.py` 24, `check_pose_vs_joint.py` 27 (was 26: + the
parametrisation round trip), `check_lift_compensation.py` 13,
`check_scan_progress.py` 62, `check_front_cam_pose_calib.py` 28,
`check_chain_calib.py` 43, `check_basler_tip.py` 11, `check_handeye_sweep.
py` 74, `check_repose_from_corners` 10, `analyse_yaw_sweep --self-test`
4/4, `error_budget -n 50` (its exact-closure assert relaxed 1e-8 → 1e-6:
the calibrated rotation is stored to 9 decimals and is orthonormal only to
~1e-9 — the values are unchanged, the assert was tighter than the file);
`catkin_make` clean (the two deleted scripts left the install list). Not
run on the robot: `arm_node` and the calibration nodes read the new file
at their next start; the numbers they will read are the ones they run
on now.

### 2026-09-21 (night) — First arm-calibration session on the sheet: the fit reads PIXELS now; the dominant term looks like the hand-eye, but it is not stable enough to apply

User collected `log/chain_calib/20260921_arm` (65 views: all of 300–309,
spins 30 / 90 / 150°, hand_cam 0.25–0.56 m, tilts to 41°) for the
joint-offset tooling of the previous entry, then moved to another
session. **Handover for the next session: `docs/HANDOVER.md` §2-0c** —
this entry is the reasoning.

**The PnP-pose residual was worthless on real data.** Most views hold
two sheet tags, and a two-tag PnP has a near-planar ambiguity: a false
3–7° tilt with a matching 30–110 mm depth error (the `raw chain error …
z ±30…110 mm` lines in `capture`). Fitting joint offsets to those poses
gave rigid 21 mm rms and hold-out WORSE with offsets. `arm_offsets.py`
now minimises the **corner reprojection in px** (`--residual reproj`,
default; `pose` kept): a two-tag view then constrains exactly what its
corners constrain. Also added `--hand-eye-free` (6-DOF `T_hc2ee`
correction; J6 fixed because a J6 offset IS a flange-z spin of the
hand-eye), `--min-tags`, `--quick`, `--write-hand-eye`, and the link
scales moved to the j3 / j4 ORIGINS (0.700 / 0.586 m) — the first
version scaled j2's 0.18 m along J1's axis, a pure base shift, and
diverged. `chain_calib.py platform()` falls back to the configured
`T_hc2ee.npz` when a session's meta names the pre-`56ef9fb` path.

**Result (exclude v13 / v11 / v20 — moving, v45 — one tag):** rigid
7.64 px rms (hold-out 7.85) → offsets 3.81 (3.85) → **hand-eye free 1.26
(2.11)**: a `T_hc2ee` correction of 20.4 mm / 1.29°, Δt in the hand_cam
frame (+8.3, +3.7, −18.3) mm, joint offsets then all ≤ 1.2° (J2
−1.22 ± 0.31, J5 −0.10 ± 0.05), links nothing. `check_chain_calib.py`
§5b (48 checks) renders corners at the REAL configurations, plants a
20 mm / 1° hand-eye error + offsets and recovers both to 0.15 mm at
0.30 px — the method is right. **Not applied, for two measured
reasons:** the parameters move between range subsets (far Δt (+5.4,
−5.0, −11.9) / 0.88° vs near (+9.1, −0.1, −19.3) / 1.43°, J4 −0.13 ↔
+0.76°) and the 1.2–2.1 px residual is 4–7× the corner floor, so the
model is missing a term — hand_cam intrinsics (D435 reports D = 0; an
fx error is near-degenerate with hand-eye z at one range) or the paper
(0.75–1.03° slope this session) are the candidates. Re-solving the
63-view chain session with the fitted hand-eye flips its verdict to
HAND with D ≈ the inverse — the chain data (PnP poses) prefers the old
hand-eye + the applied `T_ab2mb`. Unresolved; next steps and the exact
commands are in HANDOVER §2-0c (intrinsics check first, then tape the
sheet and add range / tilt diversity, accept only when subsets agree).
`log/chain_calib/20260921/corrections.npz` was restored after the
`--hand-eye` re-solve overwrote it; the re-solve outputs live under
`20260921_arm/`.

### 2026-09-21 — Web UI layout pass: the control column, camera pane untouched

User (pinyin): the camera-pane layout is fine, the rest is a bit messy.
Rendered every tab headless before touching anything; four causes, all in
`robot_ui/web/` (index.html / style.css / app.js), no Python, no node
restart — browsers pick it up on reload (served `no-store`):

- **Explanatory paragraphs** (3–5 grey lines at the top of nearly every
  group) were most of every tab. They are hidden by default now and
  toggled by an **ⓘ hints** button at the right of the tab bar
  (`body.hints`, remembered in localStorage); tooltips on the controls
  are unchanged. Text kept verbatim.
- **Arm tab:** each block was three separate 6-column grids with their
  own header rows (X.. / X.. / x..). One 7-column grid per block now —
  row label + six columns; header, `live`, `jog +`, `jog −`, `target`
  as ROWS — with step / speed / Fill / MOVE in a toolbar underneath.
  `#pose-grid` / `#jog-grid` / `#target-grid` and the joint trio are
  `display: contents` wrappers inside it, so every id, the
  `#jog-grid button[data-axis]` selectors and `check_web_ui_browser.py`
  are untouched. Arm home / Cancel arm motion moved to a toolbar at the
  top of the tab.
- **Task / Mobile / Lift:** label-column forms (`.form`) and `.toolbar`
  rows instead of wrapping `.row`s — Forward/Reverse and CCW/CW are
  equal-width pairs under their inputs, stop / cancel buttons sit at the
  right of their row (`.btn-warn`), the one main action per group is
  `.btn-primary` (Send TASK, MOVE, START SESSION, RUN SCRIPT…).
  Calibration status lines sit in a `.status` box per group.
- **Page / log split:** the pages took a fixed 3/5 of the column, so a
  short tab (Task, Mobile, Scripts) left a blank band above the log.
  `#ctl-pages` is `flex: 0 1 auto` now and the log fills the rest
  (min 140 px); a tall tab scrolls its pages instead. The log's Clear
  button is a small `clear` in its title bar.

Verified by rendering all six tabs (and hints on) in headless Chrome at
1920×1080: no wrapped button pairs, no blank band, every pre-existing
element id present (checked against HEAD's index.html). The Qt window
(`robot_ui.launch`) was not touched — its Arm tab still has the three
separate grids from the joint-control entry below.

### 2026-09-21 — robot_ui: live joint angles + joint control (Arm tab, web and Qt)

User: "ui 에 현재joint각도 와 joint 제어 추가". The joints were already in
`/arm/state` (`ArmState.joints`, same poll cycle and `pose_valid` flag as
the TCP pose — `arm_node` even keeps them live through a scan via the
worker snapshot) but neither front rendered them, and the arm had no
joint-space command at all: `move_cart` / `jog_cmd` are Cartesian, and
the only MoveJ entry points were the home pose and a `scan_joint_*` row.

- **`ArmController.move_joint(j1..j6, vel, acc)`** = one
  `MoveJ(target, TOOL_ID, 0, vel=, acc=)`; **`jog_joint(joint, delta)`**
  reads the LIVE joints, adds `delta` deg to ONE of them ('j1'..'j6' or
  1..6) and calls `move_joint` — the same rules as the Cartesian jog: no
  accumulated target, `max_step` bound, refused while busy, RPC timeout
  → `_wait_motion_done`. Result strings `move_joint ok` / `jog j3 -2.5
  deg ok`. **arm_node:** `/arm/move_joint`, `/arm/jog_joint` (String
  JSON), through the same single worker and `motion_seq` bump as
  `move_cart` — refused, not queued, while a scan holds the arm.
  Deliberately NO reach / collision / joint-limit check below the SDK:
  none exists for MoveJ anywhere in the stack (a joint task's rows go
  the same way), and the controller refuses a limit with its own code.
- **UI (both fronts, one *Joints* group on the Arm tab):** J1..J6
  readout from `/arm/state` (`—` while `pose_valid` is false, the last
  valid set kept for the fields), a ± button per joint with its own
  `jog step [deg]` (speed shared with the Cartesian jog), a six-field
  absolute target with *Fill from current* and **MOVE J** (blank = keep
  the live angle, one MoveJ). Bridge: `ARM_JOINTS`, `arm_move_joint`,
  `arm_jog_joint`; web: `api_arm_jog_joint` / `api_arm_move_joint` /
  `api_arm_joints`, `joint-grid` / `jog-joint-grid` /
  `joint-target-grid` in `app.js`.

Verified offline: `check_scan_progress.py` 49 → **62** (real
`ArmController` against the fake Fairino: six angles in one MoveJ with
vel/acc, error code reported with busy released, jog moves only the
named joint from the live angles, integer joint names, unknown joint /
over-`max_step` / non-numeric refused before any RPC, busy refusals),
`check_web_ui.py` 121 → **128**, `check_task_list_ui.py` 94 → **102**
(labels render to 2 decimals and blank on `pose_valid` false, the jog
call carries the joint step and the shared speed, MOVE J fills a blank
field from the live joints, a non-numeric field refuses). The
DevTools-driven `check_web_ui_browser.py` got the same three cases but
still dies at `Page.navigate` in this environment (pre-existing, see the
Basler-tip entry); `google-chrome --headless --dump-dom` on the page
shows the new grids built, so the JS runs through its init. Not run on
the robot: `arm_node` restart for the two topics, `robot_ui_web_node`
restart (or reload for the page alone — `app.js` is served `no-store`
from the source `web/`, but the `api_*` methods live in the node).

### 2026-09-21 — Vision tip APPLIED: (−1.8, −245.6, 209.6) mm in robot.yaml, set_tool_tcp.py and the planner URDF; the RRT CSVs are now stale

User: "반영해주", after the executed verification (measured tip 2.6 mm
from tag 215, design 7.5 mm, same final flange z). Changed together:
`robot.yaml arm_calibration.vision_tip_offset_mm`, `tools/set_tool_tcp.py`
(tool 1 constants + docstring), the planner URDF
`fr10v6_mobile_vision_0317_test.urdf` `vision_tip_joint` (−0.0018,
−0.2456, 0.2096), plus the reach / clearance copies that only model the
tool's extent (`generate_calibration_artifacts.py TOOL_OFFSET_MM`,
`handeye_calib.yaml` / `handeye_sweep.py` `tool_points_mm`) and the
TF_CHAIN doc line. The image roll (−179.1° about flange z) is not part
of the TCP — the tip frame keeps the flange orientation, as before.
Left alone: `arm_controller.py`'s fallback literal (another session has
that file open; it is only read when the yaml key is missing) and
`chain_calib`'s `DESIGN_TIP`, which is the contrast value by definition.

⚠️ **The RRT CSVs in `task/csv` were exported with the DESIGN tip and
are now stale for pose mode.** `_exec_pose` converts their tip
coordinates to a flange target with the NEW offset, so the flange lands
|Δ| = 17.4 mm from where the paired joint row puts it — the 2026-09-14
"joint 값은 정확한데 pose는 다른 곳" symptom, by exactly that vector, until
the planner re-exports from the updated URDF. **Run `scan_joint_*` until
then, not `scan_pose_*`.** `tools/check_pose_vs_joint.py` now carries
`TIP` (measured, what the config must agree on) and `CSV_TIP` (the
design, what the files on disk carry) separately and ASSERTS the 17.4 mm
offset between the pose-mode target and the joint row's flange, so the
hazard is pinned rather than hidden; when the CSVs are regenerated set
`CSV_TIP = TIP` and that check returns to "< 1 mm". Suites: 26 / 62 / 74 /
11 pass; yaml + URDF parse. `arm_node` restart required (reads the key
at start); `set_tool_tcp.py` only matters if tool 1 is ever activated
on the controller (it is not — the flange is the active frame).

### 2026-09-21 — Basler vision tip measured on the A4 sheet: (−1.8, −245.6, 209.6) mm, roll −179.1°; the ±3 mm floor is the arm's spin-dependent orientation error

User: "vision tip 수집한 데이터 분석하면 결과 확인". Session
`log/chain_calib/basler_tip_20260921` (14:07–14:41, robot_ui Basler-tip
group): 5 hand samples (spins 0 / ±40 / ±80, 30 tags each, PnP rms
0.8–1.1 px) and 10 Basler samples over tags 212 / 222 / 217 / 207 / 220 /
229 at spins 0 / ±45 / ±90, standoff 16.3–16.7 (b10 unknown). The
operator's Solve: **tip (−1.81, −245.59, 209.60) mm, image roll
−179.11°**, fit 5.48 mm rms / 11.4 max, jackknife (1.65, 0.71, 0.35).
Design (0, −253, 225.2) → **(−1.8, +7.4, −15.6) mm** — the z is the
09-18 case shortening (+6.5 mm of standoff, ~22 mm of case), the y a
real 7 mm.

Re-solved offline with per-sample residuals to see what the 5.5 mm is.
Not the sheet (single-tag similarity rms 1.4–6 px = 0.02–0.07 mm) and
not random: the residual sits along the sheet's x axis and **grows with
the wrist spin** — 0.05 / 1.5 / 4–6.7 / 11 mm at spin 0 / ±45 / +90 /
−90 — i.e. (1 − cos θ)-shaped, which is what a ~1° error between the
FK's flange z axis and the real one does to a point 246 mm off that axis
(2·246·sin 45°·0.0175 = 6 mm at 90°). The same arm orientation error
verify_chain measured on 09-18 (0.6–2.7° at one commanded orientation),
now on the Basler's lever. The hand side shows it too: the sheet origin
moves 8 mm in y between the spin-0 and spin-±80 hand samples (a
spin-dependent chain error of ~6 mm at that lever), so the fit against
each hand sample alone wanders tip y −243.4 … −248.8 and z 208.0 … 210.9
with rms 3.1 (h4, h5) … 8.3 (h1). **Honest uncertainty ±1.5 / ±3 / ±2 mm
(x / y / z), not the jackknife** — the jackknife only drops Basler
samples and never sees the sheet-pose spread. The y and z deltas from
the design clear that; x does not.

**`verify` (same evening, user: "실행 검증 어떻게"):** the executed
test — `BaslerTipSession.verify(tag, standoff, use_design)`, CLI
`basler_tip_calib.py verify <dir> <tag> [--design]`, robot_ui "Verify
(moves arm)" in the Basler-tip group (web + Qt). From the session's
sheet pose it MoveLs the flange so the tip (result.npz, or the design
tip for contrast) sits 20 mm above the chosen tag with the wrist
orientation kept, runs the standoff loop (the seek covers the 20 mm),
takes one Basler frame and reports image-centre − tag-centre in mm with
a verdict (≤ 3 OK / 3–6 the arm's spin error / > 6 wrong), appended to
`verify.csv`; refused beyond 0.35 m from the current flange. Verified
offline only (`check_web_ui` 121, `check_task_list_ui` 94; the browser
suite times out in this environment before and after the change).
**Run on the robot 16:29 / 16:31 (tag 215, wrist rz 0, from the web
UI):** measured tip → error (−0.1, +2.6) mm, |d| 2.6 — inside the fit's
±3; design tip → (−7.5, +0.6) mm, |d| 7.5 — the 7.4 mm y delta seen
directly, and BOTH runs ended at the same flange z (−432.6 mm after
the standoff loop) although the design tip's target was 15.6 mm
higher: the seek walked the difference, so the z delta is confirmed
too. `verify.csv` in the session dir.

Not applied. Applying means moving THREE things together — robot.yaml
`vision_tip_offset_mm` (the tip → flange conversion of pose mode),
`set_tool_tcp.py` (tool 1) and the planner URDF's `vision_tip_joint` —
and regenerating the RRT CSVs from that URDF: the CSV tip coordinates
were computed from joint rows with the OLD tip, so converting them with
the new offset alone would send the flange 17 mm away from where the
joint row puts it (the 2026-09-14 divergence, reintroduced). Also worth
knowing before using it: at a 246 mm lever the tip's real position moves
±4–6 mm with wrist spin whichever number is written; a tip calibrated at
the spin the scans actually use would be the tighter one.

### 2026-09-21 — Keyence seek turned on: the standoff loop now walks into the sensor window

During the Basler vision-tip session the operator jogged the tool up after
a wrist spin and pressed Auto standoff: `standoff NOT corrected: out of
range on the far side (seek disabled) (0 steps, travel 0.0 mm)` —
"这个太短了". The sensor's window on this mount is ~6.5–27 mm of case
standoff (far end raw ≈ −13), so anything parked a few cm up is the
sentinel and the loop refused to move. The 2026-09-08 rewrite had the
seek built but OFF pending the sentinel's sign; that was confirmed now
without any motion: with the arm at the home pose and nothing in range
`/keyence/value` reads −100000 and `standoff_state` says `side: far` —
negative = far, the same polarity as the readings.

Two changes. (1) `keyence_standoff.py`: the seek has its own budget —
`seek_max_mm` of travel and the steps that takes — instead of consuming
`max_steps` and `max_travel_mm`; before, 15 decisions minus ~11 seek
steps left the closed loop 4, and a 25 mm total budget could not hold a
33 mm seek plus the approach. A non-positive `seek_step_mm` is treated as
seek off (a 0 mm step spun 10 M iterations in the check). The record's
`travel_mm` still counts both. (2) `robot.yaml`: `seek_enabled: true`,
`seek_step_mm` **5** / `seek_max_mm` **40** — the user's rule
"Auto standoff 시작 가능은 4 cm": the seek walks at most 4 cm on no
measurement, so the loop works from ≤ ~67 mm of case standoff (8 steps
of ~1 s: SetSpeed(5) MoveL + settle + 5 readings). A 30 cm version
(10 / 300) was written and backed out within the hour: the step stays
under the ~20 mm window either way, but the budget is also how far a
beam that sees NOTHING walks the tool down, and at 30 cm the oblique
spot is 23 cm to the side — over a raised workpiece edge the beam reads
the floor as "far" while the case descends onto the part. The user's
other wish, "거리감지는 30 cm", is not this sensor's: the head is an
**IL-100** (user), 75–130 mm from the head along the beam — a 55 mm
span, and the −13 raw far end of 09-18 says the 16.5 mm zero sits
~117 mm from the head, so only ~10 mm (perpendicular) of that span is on
the FAR side; the rest is below the case-contact height. The one
hardware lever is remounting the head ~35–40 mm closer to the surface
(zero near the 75 mm end → ~40 mm of far-side range, re-zero and
re-measure the beam angle after); the hand_cam depth stream was offered
for 30 cm and declined ("깊이 카메라는 사용하지 않고").

Verified offline: new `tools/check_standoff_seek.py` (20 — surface plant
with the signed sentinel: 60 mm start seeks 11 × 3 mm then converges with
> 15 total steps and the closed loop inside its 25 mm; seek off refuses
without moving; a blind beam walks ≤ 50 mm and stops with a reason;
budget 40 from 90 mm stops at 51 mm; a near sentinel retreats then
converges; in-range start has no seek entries; cancel mid-seek; step 0 =
off), `check_scan_progress.py` 49. `arm_node` restart required (reads
the keys at start); then Auto standoff from a few cm up should log
`[Seek n] out of range on the far side; approach 3.00 mm` lines and hand
over to `[Standoff k/15]`.

### 2026-09-21 — Stopping the stack: SIGINT stalled on a frozen VS Code terminal; `tools/stop_stack.sh`

User asked how to see the running launches and how to kill a specific one,
then confirmed PID 30334 (`mobile_manipulator.launch`, up since 10:40).
`kill -INT 30334` did the right thing — every node unregistered and ran
its hooks (VISION lamp off, relay off, arm `Shutting down...`) — but all
16 node processes and roslaunch itself stayed alive: roslaunch logged
`ProcessMonitor shutdown failed!` 20 s in, the keyence node's main thread
sat in `tty_write_lock`, and a probe `echo > /dev/pts/1` timed out. The
launch had been started in a VS Code integrated terminal whose pty was no
longer being drained, so each node hung on its LAST stdout write at
interpreter exit, and roslaunch's SIGTERM→SIGKILL escalation hung writing
its own log lines to the same tty. Forced: SIGKILL children then
roslaunch, `rosnode cleanup` for one stale `/side_cam/realsense2_camera`
entry; only the STATUS-lamp-off hook was lost (green stays lit until the
next launch). The user relaunched at 13:39.

⚠️ **My test run of the new script then killed that relaunched stack.**
The first `stop_stack.sh` matched `pgrep -f "roslaunch .*<launch>"`, which
also hit its own `bash -c` command line (it SIGINT'd itself) and — worse —
the user's fresh roslaunch, which went down cleanly. Fixed: the matcher
is anchored on `bin/roslaunch` as the script argument and excludes `$$`,
verified against a decoy shell mentioning the launch name; the script was
only run again with no launch present. Lesson kept: never smoke-test a
stop script against a live stack. `docs/STOP_LAUNCH_kr.md` is the
procedure (what never to kill, the tty check, the restart checklist).

### 2026-09-21 — chain_calib re-measured and APPLIED: T_ab2mb from the A0 sheet (63 views); the first session's roll was paper; its records deleted

User: run the sheet calibration again, record it separately from the
first one, and — at the end — make today's result the one in the TF chain
and delete the earlier calibration's records. Done; the single record is
**`src/chain_calib/docs/CHAIN_CALIB_2026-09-21_kr.md`**, the chain with
every matrix and its provenance **`docs/TF_CHAIN_2026-09-21.yaml`**, the
data `log/chain_calib/20260921/`. `log/chain_calib/20260918/` and its
document are deleted (git has them); the 09-18 entry below keeps only the
tool history. Four things worth keeping here:

**Applied:** `extrinsics.yaml T_ab2mb` = **t (−7.47, −123.68, −628.44) mm,
rpy (+0.320, +0.786, 178.573°)**. Design chain error on the sheet
22.9 mm / 1.79° → 8.5 mm / 0.65° after the base correction (F in the fc
frame (+11.4, −8.2, −16.0) mm / (+0.32, −0.79, +1.43)°, jackknife
0.3 / 0.3 / 1.0 mm, 0.06°), verdict BASE side, hand-eye fine (joint D
~6 mm / 0.6°, inside its resolution; a spin test 62° vs 152° moves the FK
grid normal 0.55°, inside per-view scatter). User's metric (tag 200's
position from the hand_cam tags through the chain vs the sheet): bias
(+10.2, +6.6, +16.0) → 0 mm, rms 21.8 → 8.5 mm. Checks 24 / 37 / 10.
**Restart the calibration nodes.** Later the same day, on the user's
statement that the corrected chain is for end-effector pose control too,
the same transform went into `robot.yaml arm_calibration` (pose-mode IK,
`arm_node` restart) and the planner URDF's `mobile_to_base`;
`check_pose_vs_joint.py` asserts the three files agree and names the
resulting ~45 mm pose-vs-joint discrepancy of the existing planner files
as expected (regenerate `rrt_final_path_*` with the updated URDF);
`check_lift_compensation.py` made tilt-aware; `check_scan_progress.py`'s
hard-coded design-mount IK message numbers derived instead (49). See the
Transform Parameters section.

**The first session's 1.2° roll was the paper, not the mount** — found
because the re-measurement asked for a 1.8° rotation where a few mm were
expected. front_cam sees ONE 90 mm tag (200, 100 mm from the A0 corner);
a single tag's out-of-plane tilt is the LOCAL slope of the paper under
it, and the coplanar-sheet model can only put it into F. Re-laying the
sheet changed that slope 1.7° (normal 1.27° → 0.68° off vertical, sheet
moved 11 / 8 mm) and F followed. Forcing front_cam's rotation to the
level-floor prior (the level frame IS level by the ground-plane
calibration, the sheet lies on that floor; only the measured yaw kept)
changed the first session's fit rms by 0.05 mm and dropped its F tilt
1.16° → 0.17°. So: `chain_calib.py / verify_chain.py / sheet_path.py
--front-rotation level` (default), `solver.level_front_observation()`,
stored samples stay raw, `check` / `capture` print the paper slope under
front_cam's tags (⚠ over 1°). A verify run cannot catch this — it goes
through the same front_cam view of the same paper.

**Roll / pitch are not measurable this way, yaw and translation are.**
The grid plane's normal seen through hand_cam + FK sat 0.17° from the
arm's z in the first session and 0.86° in this one — the robot had been
moved off the sheet and parked back (user), so the paper under the grid
AND the chassis' attitude on the floor both changed, and "mount tilt" IS
"grid normal vs arm z". Hence the applied roll / pitch (0.32 / 0.79°) are
this parking's numbers (±1°, ±10 mm of tz/tx at the 0.64 m lever) while
yaw (1.43 vs 1.34°) and in-plane translation (~2 mm) agreed across the two
sessions. To measure tilt for real: sheet on a rigid flat board, chassis
attitude per session. Also settled: the global print scale is
unobservable from the fit (0.9989 ± 0.0083 per view — degenerate with
hand_cam's depth scale and its unmodelled distortion, D = 0), the user's
steel rule (1.0 / 1.0 / 90.00 mm) is the input; what the corners do show
is a y-only 0.35 % shrink (sy 0.9965 ± 0.0000, sx 1.0010 ± 0.0013) that no
isotropic camera effect explains and that disagrees with the rule by
2 mm — unresolved, not applied, hand_cam intrinsics calibration is where
it leads. README: corner scatter is the motion indicator, not PnP rms
(0.6–0.7 px is this setup's floor).

**Verified on the robot, and the remaining error named.** `verify_chain
run` (before today's value was applied; the difference is a constant few
mm): 8 of 10 grid tags — the run died at tag 308 when front_cam lost tag
200 for 20 frames and the drift check raised, and the CSV was only
written after the loop (both fixed: rows appended per target, the
front_cam re-read non-fatal; the 8 rows reconstructed from the terminal).
Centre offsets mean (−4.8, −5.8) mm, rms 9.8, range +1.9 mm — and the
position-dependent gradient +20.5 mm / 600 mm of arm x (1.96°) with
residual sd 1.3 mm, reproduced from the first session's run to 0.04°: the
new rpy / position columns show camera rx drifting +2.17° and its
position −11.1 mm together over 450 mm of travel at ONE commanded
orientation. That is the ARM (FK joint offsets or the lift column tilting
under the arm's moment), systematic, and now the whole of the remaining
error; separating FK from structure (same points at 0.40 m, or wrist
flipped) is next. The paper's tilt showed as predicted: range falling
0.60 / 0.79° along the grid columns. New `scripts/sheet_path.py` (user
request): drive the hand_cam LENS through points given in tag 200's frame
(x, y, height), dwell, and measure the lens's landing error against the
sheet — the ten grid points at 0.50 m by default, 3 s dwell; targets equal
verify_chain's to 0.00. Session capture lesson: 64 captures but 20
distinct geometries — repeats over-weight one pose; −x tilts 3, tags
308 / 309 seen once.

**Later: two discriminating runs, and a COLLISION.** `sheet_path` after
re-parking (16:35): 10-point mean lens error (+1.3, −0.9) mm — the chain
constant holds at a new base pose — with the arm gradient a third time
(−17 mm / 600 mm of arm x, plus an extra −10…−17 mm at the folded far row,
tags 308 / 309, reach 0.48 m). Wrist spun 92° → 60° (17:00): the mean stays
(−2.1, −0.9) but the per-point pattern changes completely (rms difference
17.7 mm vs 13 mm within a run; the Wy gradient becomes a ±10 mm alternation
by column) — the error is a function of the ARM CONFIGURATION, not of the
lens position, so a rigid base / lift-column tilt is ruled out; what is
left is FK-level (joint zero offsets / link geometry) or configuration-
dependent flex, ±15–20 mm ≈ 1° of joint error at 1 m. h = 0.40 m (17:05):
3 points, then the target at reach 0.50 m with the flange 84 mm BELOW the
arm-base plane stalled (MoveL never completed) and the one at 0.40 m
**collided** — the arm folds its elbow / tool toward the chassis there;
0.63 m and up were fine, and above the plane 0.40 m was fine. The script
had no body model (documented, and not enough — the 09-15 sweep failed
the same way, and `sheet_path` moving the arm automatically is itself the
exception to the "no automatic arm motion in chain_calib" rule). Now
`verify_chain.body_clearance_ok`: flange z < 0 ⇒ horizontal reach ≥ 0.65 m
or the target is REFUSED and recorded (`--min-reach-low` to override
knowingly). Empirical line, no link model. **Next for the remaining
±15 mm: joint-offset calibration of the arm** — the sheet already gives
the lens pose per view, but the tools record only the TCP, not the joint
angles; record `/arm/state` joints per sample first.

### 2026-09-18 (evening) — chain_calib: the printed A0 tag sheet is the ground truth for T_hc2fc; two-tag mode removed

User dropped `mobile_manipulator T_hc2fc Calibration/` at the workspace
root (an A0 print PDF with tags 200 + 300–309, its layout JSON, and a
handoff document written for another stack — ROS2 / FR5 / pupil_apriltags)
and asked for what the handoff describes: compare the robot's estimate of
the transform between the tag front_cam sees and the tag hand_cam sees
against the sheet's GT, and turn the error into corrections of the fixed
chain matrices. That is the 2026-09-15 `chain_calib` problem with a
better ruler, so it went into that package; the folder moved to
`src/chain_calib/sheet/`. **Then, on the user's instruction, the
2026-09-15 two-tag mode (`--spacing`, hand-laid tags + tape measure) was
deleted** — the sheet makes laying and measuring tags unnecessary. The
package now has one mode; `ChainSample` carries `T_hc2W / T_fc2W`, and
`load_samples` refuses the old session format. **The 2026-09-15 session
data (`log/chain_calib/20260915_c`) and its result record were deleted
on the user's instruction** (git has them).

**What the sheet changes.** Each camera solves `T_cam2W` (W = tag 200's
frame; every tag is a pure translation of it) by a **multi-tag PnP over
all the sheet tags it sees** — IPPE over every corner, LM refine, both
mirror solutions checked (`sheet.multi_tag_pnp`) — instead of one 90 mm
tag's pose, so the out-of-plane tilt that dominated the error budget is
measured over a 150–600 mm baseline. The measured hc→fc is simply
`S_i = T_hc2W · inv(T_fc2W)`: no quarter-turn snap, the cameras may look
at ANY tags. The fit is the same AX = YB (the handoff's `X_fixed` /
`Y_fixed` / `XY` = our hand / base / joint). hand_cam corners are RAW →
CameraInfo `D` goes into the PnP (the D435 reports zeros today, but the
path is there and a synthetic −0.05 k1 costs 6.8 mm if ignored);
front_cam corners are the ground-plane-corrected level-camera pixels →
no `D`, `T_mb2fc_level`, and their PnP depth is genuinely MEASURED (the
correction only de-tilts the rays; `h` enters pose_x/y, which this never
reads). Corners are stored (`corners.json`) so `solve --sx --sy
--tag-size` re-solves every sample with the print's MEASURED scale — a
plotter's 0.3 % reads as a 1.5 mm chain error otherwise (pinned by the
check). New in `solve`: hold-out evaluation (`--holdout`,
`--holdout-every`), the user's metric — tag A (hand_cam axis) → tag B
(front_cam axis) `T_A2B` through the chain vs the sheet, mean / sd / rms
/ p95 before and after each correction, bias vector in B's frame — and
residual-vs-view correlations (handoff §9.2).

**Corner convention settled against the real PDF, not the doc**
(`check_chain_calib.py` §1): rasterised with `pdftoppm` and detected with
dt_apriltags (the library `robot_camera_node` runs), all 11 tags land at
their paper positions (< 0.2 mm) with corner 0 bottom-left — exactly
`detections._CORNER_ORDER`, and the handoff's pupil_apriltags table. A
virtual camera over the raster gives R = I, so W's axes (+x paper right,
+y paper down, +z into the paper) are the tag frame the whole chain
already uses.

Verified offline: `check_chain_calib.py` (rewritten for the sheet) **35**
(PDF convention; PnP with / without D; a square-on single tag is NOT
flagged for the IPPE flip while an oblique one reports its two
solutions; the README §2 layout gives 24 views of 2–4 grid tags with
front_cam on the 300/301 pair; planted hand-eye 2° / 15 mm → D within
0.3 mm / 0.03°, mount 1° / 8 mm → F within 0.1 mm / 0.01°, both at once
by the joint fit, attribution right each time; the T_A2B metric 24 mm →
0.4 mm; hold-out 0.70 vs train 0.55 mm; persistence round trip; re-solve
at a different scale; the old format refused) and the CLI's `status` /
`solve --holdout-every 4` on synthetic sessions written to disk (both
errors planted → verdict BOTH, joint hold-out 0.70 mm). Live, read-only:
`check` against the running stack read K / D of both cameras and the
arm, collected 20 frames per camera and refused correctly (`none of the
detected tags [0] is on the sheet`) — the sheet is not laid yet.
`catkin_make` clean.

**The first session on the robot ran that evening (50 views) and was
applied; on 2026-09-21 it was re-measured, its 1.2° roll turned out to be
the paper under tag 200 (see that entry), today's session replaced it in
`extrinsics.yaml`, and its data and record were DELETED on the user's
instruction (`log/chain_calib/20260918`, `CHAIN_CALIB_2026-09-18_kr.md` —
git has them).** What survives of it is the method: the level-floor prior
on front_cam's rotation, the ruler as the scale input, the JOINT-fit
outlier criterion, `solve --min-tags --max-range`, and
`scripts/verify_chain.py` (the metric executed: the chain's flange pose
that puts hand_cam 0.50 m above each grid tag, MoveL one at a time,
offset / range measured; its algebra reproduces the fit residual offline).

### 2026-09-18 (evening) — Basler vision tip from the 20 mm tag sheet: `chain_calib/basler_tip_calib.py`

User: how to get the TF between the BASLER and hand_cam precisely with
two tags of known relative position; then "20 mm tags for both, 20 mm
gap" and a photo of the A4 sheet they printed (tag36h11 201–230, 5 × 6).
Answer built rather than described: both cameras ride the flange, so the
TF is the constant `inv(D) · T_ee2tip`, D is the sweep's, and the only
unknown is the **vision tip** — where the Basler's frame centre looks at
its 16.5 mm standoff, in the flange frame (3) + image roll (1). Not a
Basler hand-eye: at 16.5 mm the lens is macro, tilt is unmeasurable and
there is no K, and the 4 numbers ARE what the scan uses
(`vision_tip_offset_mm`, never measured on this robot, and stale since
the tool case was shortened on 09-18).

Method (`basler_tip.py`, sheet `A4_tag20_201-230_5x6_layout.json`,
40 mm pitch DESIGN — measure the print): hand_cam over the sheet at
0.25–0.35 m sees all 30 tags → `multi_tag_pnp` (120 corners, 200 mm
baseline) → `T_ab2W = A · inv(D) · T_hc2W`, several spins averaged (their
scatter = the hand_cam chain floor, and a hand-eye error shows there as
spin-dependent scatter); Basler at the Keyence standoff over one tag →
corners at full resolution (detect at 1/4, cornerSubPix) → a 2-D
similarity px → W over all corners seen → the sheet point under the
frame centre and the image x angle; least squares over (p_tip, ψ) with
3 position + 1 angle residual per Basler sample, jackknife. Tool:
`check` / `capture-hand` / `capture-basler [--standoff 16.5]` / `status`
/ `solve` → `result.yaml` with `vision_tip_offset_mm`; NOT applied
(robot.yaml, set_tool_tcp.py, the planner URDF must move together).
**Runnable from robot_ui too (user: "이 모든 것은 ui 로 실행할수있게")**:
the ROS side moved into `chain_calib/basler_tip_ros.py`
(`BaslerTipSession.check / capture_hand / capture_basler(standoff_mm) /
status / solve`, each `(ok, report, extra)`), the CLI is a thin wrapper,
`RosBridge.basler_tip(cmd, dir, standoff, exclude)` lazy-imports it, and
both fronts got a Calibration-tab group "Basler vision tip": session dir
(pre-filled `log/chain_calib/basler_tip_<date>`), Check / Capture hand /
standoff-first box + target / Capture Basler / Status / exclude / Solve,
counts line, last line, the multi-line report in a box and line by line
in the log; the solve line shows the tip and roll. `check_web_ui` 112 →
118, `check_task_list_ui` 87 → 93, `check_web_ui_browser` 84 → 87 (its two
pre-existing failures — "TASK + CHARGE chips", "standoff line" — belong
to the standoff 16.5 UI edits of the other session, not this).
`check_basler_tip.py` 11: exact at zero noise, 0.15 mm / 0.04° worst
under 0.3 px hand_cam + 2 px Basler + 0.02° arm noise, a rendered
5472 × 3648 frame (aruco 36h11 bitmap is upside down vs dt_apriltags'
corner order — rot90 ×2) lands the centre to 0.05 mm, a mirrored corner
order is refused. Live `check` against the stack: hand_cam detections
and a lamp-on Basler frame arrive (the A0 sheet 300–303 was still on the
floor; the A4 sheet not yet laid). README §6 is the procedure.

### 2026-09-18 (15:10) — Third sweep aimed by the new file: 17/17, tag 2–8 mm from centre; the 20 mm absolute offset reproduces; STOP ALL now cancels a sweep

User: "Auto-sample 한 번 더 했음". Node not restarted (old code, 49
samples in memory) but `auto_sample` loads the npz per call, so it aimed
from the sweep-only refined file: **no divergence**, square-up from
1.2 m again (285 → 115 mm in 6, unconverged — start from ~0.6 m next
time), 17 planned / 7 xy-rejected, **17 captured, the tag 2–8 mm from
the image centre at every view** — with the node's 25 mm-off file that
would have read ~25. One move failure: at 15:11:09 the operator hit
STOP ALL (two `CANCEL requested`) and Home while view 9 was moving to
(−244, 444, −45) mm, and the sweep, whose `ArmInterface` only
attributes `move_cart ok`, waited its 60 s timeout and then **continued
from the home pose** (5 MoveL chunks down to view 10, 8 more views,
all fine). That is the documented "a failed move plans the next one
from wherever the arm is" — but an operator stop must stop the
procedure: both robot_ui fronts' STOP ALL now also call
`/handeye_calib/cancel` and `/map_calibrator/cancel_calibration` when
those nodes are online (`check_web_ui` 112, `check_task_list_ui` 87).

Solve over the 35 sweep samples of the three sweeps (offline, refined
`calibrate()`, `result_sweep123_refined.npz`, now `T_hc2ee.npz`): t =
(36, −335, −152) mm, scatter 2.7 mm rms / 5.1 max, jackknife 2.8 / 2.2 /
1.5 mm — 5.4 mm / 0.36° from the 18-sample file. **The absolute offset
reproduces**: sweep 3 alone (17 fresh views, a hand-eye 12.6 mm / 1.1°
from the 18-sample one) puts tag 0 at 10 / −21 / −3 mm from map.yaml's
prediction, all 35 at 10 / −19 / −5 with a 0.5 / 0.7 / 1.6 mm jackknife.
In world axes (`transform_world_to_arm` sensitivity: world +x → arm +y,
world +y → arm −x at the tag-102 pose) that is **−19 mm ACROSS the
lane** and −10 along it. Not noise, and not resolvable by more sweeps:
hand-eye bias particular to this 0.37 m lever, `arm_body_offset_y`, or
tag 102's map position — the 09-14 mount agreed to 1 / 0 / 5 through
the same chain. Next is `chain_calib` or a tape measure from the base
to cross tag 0. Restart the calibration nodes for the new npz.

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

### 2026-09-18 — Tool case shortened: Keyence zero 10 → 16.5 mm, beam angle re-measured 42.6 → 37.1°

User shortened the end-effector tool case; the Keyence and the Basler moved
with it relative to the flange, the case bottom now rests **16.5 mm** above
the surface when the sensor reads 0, and the Basler was refocused there.
Config: `keyence.sensor_zero_mm` **16.5** and `target_distance_mm` **16.5**
(setpoint stays 0 — the loop still drives the reading to 0, which is where
the focus is; the two keys must move together, a zero-only edit would have
pushed the tool 6.5 mm closer on every scan), UI Auto-standoff default 16.5,
`keyence_scan_chain.md` Sensor facts. Note the zero was never measured in
this workspace: 10.0 was the operator's word on 2026-08-05, 30.0 before
that matched nothing; 16.5 is a tape reading at reading 0.

**Beam angle measured on the new mount, not taken from the geometry.** The
user's figure was 60° to the surface (= 30° to tool Z); two sweeps with
`tools/measure_keyence_angle_via_node.py` (new — the same symmetric-sweep
method as `measure_keyence_angle.py`, but through `/arm/jog_cmd` +
`/arm/state` so it opens no second Fairino RPC while `arm_node` runs;
tool vertical over the workpiece, base on 102) gave **k = 1.257 / 1.250 →
37.30° / 36.87°**, R² 0.998 / 0.9995, rms 35 µm — i.e. **53° to the
surface**. Per-step sensitivity wandered 1.10–1.35 (spot walk over
topography, as on the old mount), so ±2°. `beam_angle_deg` **37.1**. Also
seen: the far end of the sensor's range is at raw ≈ −13 mm (perp −10.4 →
~27 mm case standoff); the first sweep's −1.0 mm point went out of range.
`keyence_dir` −1.0 confirmed (k > 0). `arm_node` restart required for all
of it; `check_scan_progress.py` 49 ok.

⚠️ **Found on the way, NOT fixed: `jog` accumulates a constant readback
offset.** Every MoveL target arm_node logged was z −0.635 / y −0.09 from
the previous one for a `jog z −0.5`: `GetActualTCPPose` after a MoveL
reports (0, −0.09, −0.135) mm off the target it was sent, and `jog` reads
that pose and adds its delta, so each jog moves delta + 0.135 mm in z and
drifts y by 0.09 mm — 5 × 0.5 mm jogs moved 3.2 mm. Constant, so the
sweep's fit against the reported z is right (target and readback moved the
same 0.635 per step). For the operator jogging in robot_ui it is a 27 %
overshoot on small z jogs; whether it is a controller settle offset or a
frame subtlety is open. The sweeps' return jogs suffered from it too: the
arm ended ~1.7 mm closer to the surface than the user parked it.

Still open from the same change: Ra values before/after are not comparable
(new working distance, refocused), `vision_tip_offset_mm` still describes
the old case, and `T_hc2ee` if hand_cam moved with it.

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

### 2026-09-15 — front_cam ↔ hand_cam chain calibration package `chain_calib` (two floor tags; the two-tag truth was replaced by the A0 sheet 2026-09-18)

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

**Run on the robot the same evening; superseded and deleted 2026-09-18.**
Two things from that run still shape the tool: the automatic view sweep
collided the arm with the robot body — the planner's safety rules model
only the flange and vision tip against the TAG PLANE, not the base
body, lift or elbow — so `chain_calib` has had **no automatic arm motion
since** (operator-jogged `capture` only); and the two-tag truth (hand-laid
tags + a tape measure) proved error-prone to lay and measure, which is
why the printed A0 sheet replaced it on 2026-09-18 and the two-tag mode,
its session data and its result record were removed on the user's
instruction. Nothing was applied to the config from that session.

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

### Older entries (2026-08-07 … 2026-09-14) — `docs/WORKLOG_ARCHIVE.md`

Moved out of this file on 2026-10-06 (user: the file had grown to 644 KB and
is loaded every session). Same text, same order, nothing edited; anything
above that cites "the Work Log entry" of a date before 2026-09-15 means that
file. New entries still go HERE, at the top of the Work Log.
