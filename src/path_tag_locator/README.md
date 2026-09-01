# path_tag_locator

Online localization of human-placed ground-path AprilTags in the world
frame, using a user-supplied reference tag whose accurate world pose is
known.

**2026-09-01 refactor — no standalone HARDWARE mode.** The nodes keep
their own `launch/path_tag_locator.launch` (run it alongside the main
stack, which must already be up; `use_handeye_calib:=true` adds the
hand-eye node) but own no hardware: the arm is reached through `arm_node`
(`/arm/state` + `/arm/move_cart`), the base through `mobile_node`
(`MobileClient`), and tag observations come from `robot_camera_node`'s
`/hand_cam/tag_detections` / `/front_cam/tag_detections`. The former
in-package nav stack (`nav/`), the Fairino SDK client (`tcp_pose.py`),
and the map/robot_nav config copies were deleted with it.
⚠️ `map_calibrator` is a second commander of `/mobile/goto_tag` after
`task_executor` — do not issue `TASK`/`GOTO` during a calibration
session.

Korean field guide: [docs/CALIBRATION_GUIDE_kr.md](docs/CALIBRATION_GUIDE_kr.md).

The workflow is also drivable from **robot_ui**: the `map_calibration` and
`locate_tag` scripts in the Scripts tab call these services through
`RosBridge`, and per-tag session progress streams into the UI log.

## Method

```
T_A2B = T_A2hc · T_hc2ee · T_ee2ab · T_ab2mb · T_mb2fc · T_fc2B
T_B_world = T_A_world · T_A2B
```

- **A**: reference tag, accurate `T_A_world` provided by user.
  Observed by **hand camera** (arm end-effector).
- **B**: path tag, observed by **front/floor camera** (mobile base).
- `T_hc2ee`: hand-eye calibration (npz).
- `T_ee2ab`: from live Fairino TCP pose.
- `T_ab2mb`, `T_mb2fc`: platform-fixed geometry (yaml).

## Files

| Path | Purpose |
|------|---------|
| `srv/LocatePathTag.srv` | Service: request tag_b_id (or default), optional T_A_world override, optional save |
| `config/locator.yaml` | Detections/arm topics, tag IDs/sizes, detector sizes, file paths |
| `config/reference_tag.yaml` | Default T_A_world (pose or 4x4) |
| `config/extrinsics.yaml` | T_AB2MB / T_MB2FC (row-major 4x4) |
| `config/handeye_calib.yaml` | Hand-eye calibration node config |
| `config/hand_eye/T_hc2ee.npz` | 4x4 hand-eye calibration (produced by handeye_calib_node) |
| `config/reference_tags.yaml` | Multi-ref-tag ground truth for batch map calibration |
| `config/calibration_plan.yaml` | Ordered path_tag → ref_tag plan |
| `config/map_calibrator.yaml` | Orchestrator default file paths |
| `srv/RunMapCalibration.srv` | Batch calibration service |
| `scripts/path_tag_locator_node.py` | Locator ROS node |
| `scripts/handeye_calib_node.py` | Hand-eye calibration ROS node |
| `scripts/map_calibrator_node.py` | Batch map-calibration orchestrator |
| `src/path_tag_locator/align.py` | Auto-align logic (target EE pose, per-step clamp, metrics) |
| `src/path_tag_locator/align_runner.py` | Shared iterative align driver (used by both locate and calibrator) |
| `src/path_tag_locator/arm_interface.py` | arm_node proxy (get_tcp_pose / move_j_to_pose over `/arm/state` + `/arm/move_cart`) |
| `src/path_tag_locator/base_interface.py` | mobile_node proxy (MobileClient + /odom heading) |
| `src/path_tag_locator/detections.py` | shared-detector observations -> T_cam2tag (with tag-size rescale) |
| `src/path_tag_locator/lift_listener.py` | live `/lifter/height` for T_ab2mb lift compensation in the chain |
| `src/path_tag_locator/calibration/` | plan_io + map_io + orchestrator for batch calibration |
| `src/path_tag_locator/` | Python module (chain, geometry, detect, handeye_calib, align, ...) |

## Usage

```bash
# Build
cd ~/mobile_manipulator_ws
catkin_make --pkg path_tag_locator
source devel/setup.bash
```

### Step 0 — Hand-eye calibration (one-off)

There are two ways to produce `config/hand_eye/T_hc2ee.npz`:

- **A. Run a calibration session** with `handeye_calib_node` (recommended
  when you have the equipment to capture diverse hand-cam views of the
  calibration tag — see below).
- **B. Direct input** via `config/hand_eye/T_hc2ee.yaml` — edit
  `position_m` + `rpy_deg` (or `matrix_4x4`) in that file with values
  measured externally (CAD, datasheet, prior calibration), then convert
  to npz with `scripts/save_npz.py`. Useful for bootstrapping or when
  the nominal mounting offset is good enough.

```bash
# Edit the values in config/hand_eye/T_hc2ee.yaml first, then:
rosrun path_tag_locator save_npz.py            # uses the yaml + writes npz
# (refuses to overwrite an existing npz; add --force to replace)
rosrun path_tag_locator save_npz.py --force
```

Restart `path_tag_locator_node` after rewriting the npz — the locator
caches the matrix at startup.

#### Path A — Real calibration via `handeye_calib_node`

`T_hc2ee.npz` is hardware-specific and must be produced on this robot.
Each capture is immediately archived to disk, so you can interrupt the
session and resume later (see "Reusing previous samples" below).

```bash
# Edit config/handeye_calib.yaml (topic names, tag_id/size,
# output_path, min_samples). Then:
roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true

# Move the arm so the hand-cam sees the calibration tag from a new pose,
# then capture; repeat ~15-30 times with diverse orientations:
rosservice call /handeye_calib/capture     "{}"
rosservice call /handeye_calib/status      "{}"   # check progress
rosservice call /handeye_calib/compute     "{}"   # writes T_hc2ee.npz
# rosservice call /handeye_calib/reset     "{}"   # discard and start over
# rosservice call /handeye_calib/load_latest "{}" # reuse most recent run
```

**Reusing previous samples.** Restarting the node clears in-memory
samples but disk archives under
`~/.ros/path_tag_locator/handeye_calib/run_*/` are preserved. Two ways
to re-feed them into `/compute`:

```bash
# (a) after node start, append the latest prior run into memory
rosservice call /handeye_calib/load_latest "{}"
rosservice call /handeye_calib/compute     "{}"

# (b) configure specific dirs to preload at launch via
#     config/handeye_calib.yaml → io.load_samples_dirs (list of paths)
```

You may freely mix loaded + new `/capture` samples before `/compute`.

### Step 1 — Locate path tags

```bash
# 1. Edit config/reference_tag.yaml with the accurate T_A_world for your
#    installation (position_m + rpy_deg, or matrix_4x4).
# 2. Ensure your hand-cam and front-cam publish on the topics named in
#    config/locator.yaml (or update the topic names there).
# 3. Make sure arm_node and robot_camera_node are up (main launch).

# Run
roslaunch path_tag_locator path_tag_locator.launch   # main stack must already be up

# Call service (use yaml defaults, no auto-align)
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: -1,
  override_ref: false,
  save_result: true,
  save_dir: '',
  auto_align: false,
  align_initial_tcp_mm_deg: [0,0,0,0,0,0]
}"

# Subscribe to latched topic
rostopic echo -n 1 /path_tag_locator/tag_world_pose
```

### Auto-align before locating

If you do not have a fixed observation pose for tag A, set
`auto_align: true` and provide an approximate base-frame TCP pose where
the hand-cam can see tag A. The node will:

1. Move to the supplied pose (`MoveJ` after IK, clamped by
   `align.max_initial_step_*`).
2. Detect tag A in the hand-cam image, then iteratively move the arm so
   that the tag appears at image center with the camera optical axis
   perpendicular to the tag plane. Each step is clamped by
   `align.max_step_m` / `align.max_step_deg` for safety.
3. Stop when both `xy_offset ≤ position_tol_m` and `tilt ≤ angle_tol_deg`,
   or after `max_iterations` (set `max_iterations: 1` for one-shot).
4. Then run the usual locate computation on the aligned pose.

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: -1,
  override_ref: false,
  save_result: true,
  save_dir: '',
  auto_align: true,
  align_initial_tcp_mm_deg: [300, 0, 400, 180, 0, 0]   # approximate, mm/deg, FR5 ZYX
}"
```

The response includes `align_iterations_used`,
`align_final_xy_offset_m`, `align_final_tilt_deg`, and
`align_final_tcp_mm_deg` for inspection.

To override T_A_world at call time, set `override_ref: true` and fill
`ref_pose` (a `geometry_msgs/Pose` with position in meters and a unit
quaternion).

### Step 2 — Batch map calibration (`map_calibrator_node`)

For the larger workflow of **recalibrating every path tag in `map.yaml`**
against multiple user-provided reference tags, use the
`map_calibrator_node`. This drives the base through `mobile_node` (apriltag_nav's
`MobileClient` — since the 2026-09-01 refactor there is no in-package
navigation) and the per-tag locate call into a single autonomous session
that emits a world-frame `map_world_<ts>.yaml`.

#### Config files (`config/`)

| File | Purpose |
|------|---------|
| `reference_tags.yaml` | Multiple ref tags with **accurate** world poses (6-DOF: position_m + rpy_deg, or a 4×4 matrix). User-measured ground truth — these are what every path tag is calibrated against. |
| `calibration_plan.yaml` | Ordered list of `{path_tag_id, ref_tag_id}` assignments. Per-entry can override `arm_view_tcp_mm_deg` (initial arm pose so hand-cam sees the assigned ref tag) and `nav_start_id` (base nav starting tag for BFS). |
| `map_calibrator.yaml` | Default file paths for the orchestrator (overridable per service call). |

#### Workflow

```bash
# Park the base at a known map tag (e.g. DOCK 500) BEFORE launching, so
# the very first nav has a known starting point.
# The MAIN STACK MUST ALREADY BE UP (arm_node / mobile_node /
# robot_camera_node are the hardware owners). Do not send TASK/GOTO
# while a session runs.
roslaunch path_tag_locator path_tag_locator.launch

# Dry-run: parses plan + ref tags + map.yaml without moving anything.
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: true
}"

# Real run.
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: false
}"

# Monitor (per-tag JSON status):
rostopic echo /map_calibrator/progress
rostopic echo /map_calibrator/current_target_tag
```

The service response carries `num_succeeded`, `num_failed`, and the
`output_yaml_path`. Failed entries do **not** abort the session; they are simply absent
from the output's `tags:` (map.yaml itself is never written), and a
`run_*_FAILED/` directory under `~/.ros/path_tag_locator/locate/` holds
the error and request echo.

#### Per-entry execution (what happens for each plan row)

1. Base navigates to `nav_start_id` (if given), then to `path_tag_id`
   via the BFS over `map.yaml`'s `edges`.
2. Arm `MoveJ` to `arm_view_tcp_mm_deg` (a coarse "look at ref tag"
   pose; defaults to `plan.defaults.arm_view_tcp_mm_deg`).
3. `run_auto_align` iteratively centers tag A in the hand-cam image and
   makes the camera optical axis perpendicular to the tag plane
   (reuses the same algorithm as `~/locate_path_tag`'s `auto_align`).
4. Capture fresh hand-cam + front-cam images, read live TCP pose.
5. Run the chain (`compute_T_A2B` → `compute_T_B_world`) with the
   trusted `T_A_world` from `reference_tags.yaml`.
6. Upsert the tag's world pose into the in-memory output and
   **atomically** rewrite `map_world_<ts>.yaml` (tmp + `os.replace`).
7. Persist the full 6-DOF result + raw images via
   `persistence.save_locate_run` (same archive layout as the locator).

#### Why this works despite `map.yaml` being inaccurate

The arm "viewing pose" is derived from `map.yaml`'s current (possibly
off by cm) path-tag positions — so the initial arm move is approximate.
This is OK because:

- The chain (`T_A2B`) is anchored on **direct AprilTag observations** of
  ref tag A and path tag B, plus the calibrated transforms
  (`T_hc2ee`, `T_ab2mb`, `T_mb2fc`) and the live TCP. None of these
  depend on `map.yaml`.
- `T_B_world = T_A_world · T_A2B` uses the user-supplied accurate
  `T_A_world`, so the final result is independent of `map.yaml`'s
  errors.
- `run_auto_align` absorbs the approximate-pose noise by re-centering
  the hand camera on the ref tag before the chain is evaluated.

So `map.yaml` is used only as a coarse seed for arm positioning; the
calibration result is bounded by `T_A_world` accuracy + hand-eye
calibration residual + platform extrinsics + tag detection precision.

#### First calibration — step-by-step

This walks through the very first calibration of **one path tag** end-to-end.
Once it succeeds, scaling to all path tags is just adding entries to the plan.

**Phase 0 — Physical setup (one-time)**

1. Stick AprilTag `id=0` at a fixed location that defines your **world
   origin** (wall corner, floor mark, fixture).
2. Stick the other ref tags (e.g. id 100, 502) and **measure** their
   `(Δx, Δy, Δz, yaw)` relative to tag 0 with a tape measure.
3. Place at least one path tag on the floor (e.g. id 101) where the
   base can drive to it and the front camera can see it.

**Phase 1 — Hand-eye `T_hc2ee.npz` (one-time)**

Two paths — pick one:

```bash
# A. Quick bootstrap: type measured / design values into the yaml.
nano src/path_tag_locator/config/hand_eye/T_hc2ee.yaml
rosrun path_tag_locator save_npz.py             # refuses to overwrite
rosrun path_tag_locator save_npz.py --force     # force overwrite

# B. Real calibration: 15-30 captures + cv2.calibrateHandEye.
roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true
rosservice call /handeye_calib/capture "{}"     # repeat per arm pose
rosservice call /handeye_calib/compute "{}"
```

**Phase 2 — Fill the configs**

`config/reference_tags.yaml` — measured ground truth:

```yaml
format: "pose"
reference_tags:
  - id: 0
    position_m: [0.0, 0.0, 0.0]
    rpy_deg:    [0.0, 0.0, 0.0]
  - id: 100
    position_m: [1.500, 0.200, 0.0]
    rpy_deg:    [0.0, 0.0, 0.0]
```

`config/calibration_plan.yaml` — start with ONE entry:

```yaml
defaults:
  align_required: true
  arm_view_tcp_mm_deg: [350.0, 0.0, 250.0, 180.0, 0.0, 0.0]   # placeholder; replaced in Phase 3
plan:
  - path_tag_id: 101
    ref_tag_id:  0
    nav_start_id: 500       # base-nav start tag (e.g. DOCK)
```

`config/locator.yaml` / `config/handeye_calib.yaml` — make sure the
detections topics match robot_camera_node's outputs
(`/hand_cam/tag_detections`, `/front_cam/tag_detections`) and the
`detector:` tag sizes mirror robot.yaml `robot_camera.tag_size`.

**Phase 3 — Capture `arm_view_tcp_mm_deg`**

This is the only manual step. Find an arm pose where the hand camera
roughly sees ref tag 0, then record its TCP:

```bash
# 1. Drive the base to in front of path_tag 101 (front-cam sees it).
#    Use the teach pendant / joystick / a one-off apriltag_nav run.

# 2. Once the base is parked, jog the arm via the teach pendant until
#    the hand camera sees ref tag 0 (use rqt_image_view on
#    /hand_cam/color/image_raw to confirm — tag 0 just needs to be
#    visible; auto_align will center it later).

# 3. Read the live TCP pose from arm_node (never open a second RPC
#    connection to the arm):
rostopic echo -n1 /arm/state/tcp_pose

# 4. Paste those 6 numbers into calibration_plan.yaml — either
#    defaults.arm_view_tcp_mm_deg or the entry's override.
```

**Phase 4 — Launch the calibrator**

```bash
# Keep the main stack RUNNING (mobile_node drives the base for the
# session); just make sure no TASK/GOTO is in flight.

# Park the base in front of nav_start_id (DOCK 500). The first goto needs
# a tag the front cam can see, otherwise BFS has no starting node.

# Launch:
roslaunch path_tag_locator path_tag_locator.launch   # main stack must already be up
```

**Phase 5 — Dry-run, then real run**

```bash
# Dry-run: parses plan + ref tags + map.yaml without moving anything.
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: true
}"
# Expected: success=true, num_succeeded=0, num_failed=0 (dry runs don't count).

# Monitor (separate terminal):
rostopic echo /map_calibrator/progress

# Real run:
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: false
}"
```

Inside, for that one entry:
1. base nav: `500 → … → 101` (front-cam keeps the tag in view, decelerates).
2. arm `MoveJ` to your `arm_view_tcp_mm_deg`.
3. `auto_align` (~3–5 iterations) — hand-cam centers ref tag 0:
   ```
   auto_align iter 1/5: xy=0.040 m, tilt=3.2 deg, z=0.250 m
   auto_align iter 3/5: xy=0.003 m, tilt=0.4 deg
   auto_align: converged at iteration 3
   ```
4. fresh hand-cam + front-cam images + TCP read.
5. chain math → `map_world_<ts>.yaml` updated.

**Phase 6 — Verify**

```bash
# The yaml output:
ls -lt ~/.ros/path_tag_locator/map_world_*.yaml | head -1
cat <that file>
# Expect tag 101 with position_m / rpy_deg, ref_tag_id=0, map_xy from map.yaml.

# Per-tag archive (raw images for sanity check):
ls ~/.ros/path_tag_locator/locate/$(date +%Y%m%d)/run_*_tag101/
# hand_cam.png: ref tag 0 should be near image center, tilt < ~5°
# front_cam.png: path tag 101 should be clearly visible
```

**Phase 7 — Scale to the full map (auto view-pose bootstrap)**

After the **first** successful entry the orchestrator captures a base
anchor (mb's world-frame pose, /odom yaw, the path tag's map.yaml xy)
and uses it to **auto-compute `arm_view_tcp_mm_deg`** for every later
entry that does not supply one explicitly. The estimate uses

- `T_world2mb` from the anchor + Δ(path_tag map.yaml xy) + Δ(/odom yaw),
- the trusted ref tag world pose, plus the calibrated `T_hc2ee` and
  platform extrinsics.

It does NOT need a `T_map2world` transform — the bootstrap relies on
**relative** map.yaml geometry (which is approximately correct even when
the absolute map↔world transform is unknown). `run_auto_align` absorbs
any residual cm/deg-level error in the estimate.

Override precedence per entry:

1. `entry.arm_view_tcp_mm_deg` (per-entry yaml override) — always wins.
2. Auto-bootstrap estimate (when `align.auto_view_pose: true` and at
   least one prior entry succeeded). Logged as `view_tcp_source:
   auto-bootstrap`.
3. `defaults.arm_view_tcp_mm_deg` (yaml).

Switch off the bootstrap with `align.auto_view_pose: false` in
`locator.yaml` to force always-defaults behavior.

So the practical plan for adding more tags:

```yaml
plan:
  - path_tag_id: 101
    ref_tag_id: 0
    nav_start_id: 500
  - path_tag_id: 102            # arm_view_tcp_mm_deg auto-estimated
    ref_tag_id: 0
  - path_tag_id: 103            # auto-estimated again
    ref_tag_id: 0
  - path_tag_id: 405
    ref_tag_id: 502             # auto-estimate still works after switching ref
```

If a particular entry's auto-estimate puts the cam in a poor pose (e.g.
the ref tag ends up at the edge of the image), add a per-entry override
for just that row:

```yaml
  - path_tag_id: 405
    ref_tag_id: 502
    arm_view_tcp_mm_deg: [420.0, -10.0, 320.0, 175.0, 0.0, 5.0]
```

#### Convention warning: tag orientation in `reference_tags.yaml`

The chain follows the AprilTag library convention: a tag's local
**z-axis points INTO the tag**, away from the printed face. Match the
`rpy_deg` you declare to each tag's PHYSICAL mounting:

| Physical mounting | tag's +z direction | `rpy_deg` (no yaw) |
|-------------------|--------------------|--------------------|
| Face DOWN (visible from below; e.g. ceiling-mounted — **this project's default**) | world +z (up)  | `[0.0, 0.0, 0.0]`  |
| Face UP   (visible from above; common for floor stickers) | world -z (down)| `[180.0, 0.0, 0.0]`|

The bundled `reference_tags.yaml` holds the REAL cross tags — face-up
on the plate, `[180, 0, 0]`. Getting this wrong produces a 180° error in the chain
output, usually visible as path-tag positions mirrored about the tag
plane.

(Continuing the walkthrough with this convention in mind:)

```yaml
plan:
  - path_tag_id: 101
    ref_tag_id: 0
    nav_start_id: 500
  - path_tag_id: 102
    ref_tag_id: 0
  - path_tag_id: 405
    ref_tag_id: 502                  # use a closer ref tag for far locations
    arm_view_tcp_mm_deg: [...]       # different jog pose for this base spot
```

Re-running calibrator writes a fresh `map_world_<ts2>.yaml`; merge with
the previous one manually (yq, jq, or hand) when you're done.

#### Common first-time issues

| Symptom | Fix |
|---------|-----|
| `base nav … failed` on first entry | base isn't in front of `nav_start_id`; front-cam sees no tag |
| `tag A (id=0) not detected` | jog drifted too far from where you captured `arm_view_tcp_mm_deg`; recapture per Phase 3 |
| `auto_align: clamped` repeats | per-step throttling — usually fine (next iter continues) |
| `auto_align` xy stays >5 cm | `T_hc2ee` is inaccurate → redo Phase 1; or relax `align.position_tol_m` |
| result `position_m` off by >1 m | ref_tags measurement error; yaw flipped; hand_cam.png shows the wrong tag |
| Fairino `MoveJ` failure | `tcp_index=1` tool not active on the controller; or `arm_view_tcp_mm_deg` unreachable |

#### Frames: `map.yaml` ≠ user world frame

`map.yaml` lives in the Manipulator/Map frame (the 2D `(x, y)` system
that `apriltag_nav` and `/robot_pose` use). The user-supplied
`reference_tags.yaml` lives in a separate **world frame** (typically
chosen with one ref tag as the origin — see the example with `id: 0`
having identity pose). The transform between these two frames is
generally **not measurable** by this package, so the orchestrator does
not attempt to align them.

Consequences:
- `map.yaml` is read-only — the calibrator never writes to it and never
  produces a drop-in replacement for it.
- The base navigation still uses `map.yaml` (its (x, y) are good enough
  for Pure-Pursuit BFS; cm-level errors are absorbed by visual
  servoing).
- The chain math (`T_B_world = T_A_world · T_A2B`) produces results in
  the **world frame**, regardless of map.yaml's accuracy.

#### Post-calibration verification

After a session finishes you can run any of these (most without
touching the robot):

| Script | Robot needed | Purpose |
|--------|-------------|---------|
| `verify_map_world.py` | No | Per-tag summary + relative-distance check against `map.yaml`'s edge graph. Flags entries whose inter-tag distance differs by more than `--threshold-m` (default 5 cm). |
| `test_repeatability.py` | Yes (full re-run) | Calls `/map_calibrator/run_calibration` twice, diffs the two outputs. Per-tag mean |Δxy| is the actual repeatability of the chain. |
| `verify_arm_pointing.py` | Yes (arm + cameras + base parked) | For each calibrated tag, compute the view pose using its world coordinates, MoveJ there, re-detect with the hand camera, report residual offset. Direct closed-loop check that the saved coordinates actually point the arm at the tag. |
| `visualize_map_world.py` | RViz only | Publishes a `MarkerArray` of ref tags (red) + calibrated path tags (green) + world origin axes for visual inspection. |

```bash
# Offline (no robot)
rosrun path_tag_locator verify_map_world.py
rosrun path_tag_locator verify_map_world.py --threshold-m 0.10  # looser

# Repeatability (robot has to re-do the full plan; base back to start)
rosrun path_tag_locator test_repeatability.py

# Arm pointing — drive base to the tag, then:
rosrun path_tag_locator verify_arm_pointing.py --tag-id 101
# or after the bulk run, with base parked near several tags in sequence:
rosrun path_tag_locator verify_arm_pointing.py --all

# RViz visualization (start `rviz` in another terminal with Fixed Frame = world)
rosrun path_tag_locator visualize_map_world.py
```

Inspect failure cases by opening the per-tag archive:
`~/.ros/path_tag_locator/locate/<YYYYMMDD>/run_<ts>_tag<id>/{hand_cam,front_cam}.png`.

#### Output

- `~/.ros/path_tag_locator/map_world_<ts>.yaml` (**world-frame** output;
  schema explicitly different from `map.yaml`, with a `frame: world`
  banner and a `note:` warning against using it as a map.yaml
  replacement). Each calibrated tag has:
  - `position_m: [x, y, z]` and `rpy_deg: [rx, ry, rz]` (full 6-DOF)
  - `ref_tag_id` used for this calibration
  - `map_xy` (the (x, y) the tag had in map.yaml at run-time, for
    cross-reference)
  - `type` / `zone` / `name` copied from map.yaml
- Per-tag archive identical to single-call locate output (see
  "Persistence layout" below).

## Output

- **Topic** `~/tag_world_pose` (`geometry_msgs/PoseStamped`, frame_id="world",
  latched).
- **Service response** includes `tag_b_world_pose`, `position_m`,
  `rpy_deg` (ZYX intrinsic, degrees), and `t_a2b_row_major` for debug.
- **Files** — every call (success and failure) is persisted; the
  `save_result` request flag is now informational only. See next section.

## Persistence layout

Every `locate_path_tag` call writes a self-contained run directory so
that the result can be reproduced offline:

```
~/.ros/path_tag_locator/locate/<YYYYMMDD>/run_<ts>_tag<id>/
    hand_cam.png            BGR image fed to detector
    front_cam.png           BGR image fed to detector
    K_hc.npz / K_fc.npz     intrinsics actually used
    result.npz              T_B_world, T_A2B, T_A_world, T_hc2ee,
                            T_ab2mb, T_mb2fc, tcp_pose_mm_deg, K_*,
                            position_m, rpy_deg
    result.yaml             human-readable summary (incl. auto_align report)
    request.yaml            full service request echo
~/.ros/path_tag_locator/locate/locate_log.csv   append-only index
```

Failed calls land in `..._FAILED/` directories with `result.yaml`
holding the error message and `request.yaml` echoing the input, and the
log row carries `success=0`.

Hand-eye calibration archives each capture as it happens:

```
~/.ros/path_tag_locator/handeye_calib/run_<ts>/
    samples/0000_image.png    samples/0000_pose.npz  (tcp_pose, K)
    samples/0001_image.png    ...
    samples_index.csv         row per capture (tcp, file paths)
    result.npz / result.yaml  written on /compute
```

`/handeye_calib/reset` starts a fresh `run_<ts>/` directory, so
historical attempts are never lost. Any prior `run_*/` directory can be
re-fed into a new session with `/handeye_calib/load_latest` or via the
`io.load_samples_dirs` yaml field.

## Troubleshooting

For symptom-based diagnostics (100m-scale results, auto_align non-convergence,
SDK signature errors, reach-limit issues, calibration residual, etc.) and
copy-paste verification commands, see
[docs/TROUBLESHOOTING_kr.md](docs/TROUBLESHOOTING_kr.md).

## Notes

- Coordinate conventions: lengths in meters, RPY in degrees (ZYX intrinsic).
  This matches the FR5 TCP-pose convention used by the Fairino SDK.
- Transform notation: `T_X2Y` = pose of frame Y expressed in frame X
  (i.e. it transforms Y-frame coordinates to X-frame coordinates). All
  matrices in `config/extrinsics.yaml` and `T_hc2ee.npz` follow this
  convention.
- `T_hc2ee` is the pose of EE in the hand-cam frame. OpenCV's
  `calibrateHandEye` returns `R_cam2gripper, t_cam2gripper` (the inverse
  direction); `handeye_calib.py` already inverts it before saving.
- Restart the node after editing `T_hc2ee.npz` or any yaml file —
  the locator caches them at startup.
- The detector uses `dt_apriltags` (Duckietown). Install with
  `pip install dt_apriltags` if missing.
- `scripts/save_npz.py` writes a hand-coded identity-ish T_hc2ee for
  smoke-testing only; for real use it refuses to overwrite an existing
  file unless given `--force`. Always run `handeye_calib_node` for a
  real calibration.
