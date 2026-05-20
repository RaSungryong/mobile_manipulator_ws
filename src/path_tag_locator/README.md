# path_tag_locator

Online localization of human-placed ground-path AprilTags in the world
frame, using a user-supplied reference tag whose accurate world pose is
known. This package is independent of `apriltag_nav` (no imports, no
shared messages, no shared topics).

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
| `config/locator.yaml` | Topics, tag IDs/sizes, robot IP, file paths |
| `config/reference_tag.yaml` | Default T_A_world (pose or 4x4) |
| `config/extrinsics.yaml` | T_AB2MB / T_MB2FC (row-major 4x4) |
| `config/handeye_calib.yaml` | Hand-eye calibration node config |
| `config/hand_eye/T_hc2ee.npz` | 4x4 hand-eye calibration (produced by handeye_calib_node) |
| `scripts/path_tag_locator_node.py` | Locator ROS node |
| `scripts/handeye_calib_node.py` | Hand-eye calibration ROS node |
| `src/path_tag_locator/align.py` | Auto-align logic (target EE pose, per-step clamp, metrics) |
| `src/path_tag_locator/` | Python module (chain, geometry, detect, handeye_calib, align, ...) |

## Usage

```bash
# Build
cd ~/mobile_manipulator_ws
catkin_make --pkg path_tag_locator
source devel/setup.bash
```

### Step 0 — Hand-eye calibration (one-off)

`T_hc2ee.npz` is hardware-specific and must be produced on this robot.

```bash
# Edit config/handeye_calib.yaml (topic names, tag_id/size, robot_ip,
# output_path, min_samples). Then:
roslaunch path_tag_locator handeye_calib.launch

# Move the arm so the hand-cam sees the calibration tag from a new pose,
# then capture; repeat ~15-30 times with diverse orientations:
rosservice call /handeye_calib/capture "{}"
rosservice call /handeye_calib/status  "{}"   # check progress
rosservice call /handeye_calib/compute "{}"   # writes T_hc2ee.npz
# rosservice call /handeye_calib/reset "{}"   # discard and start over
```

### Step 1 — Locate path tags

```bash
# 1. Edit config/reference_tag.yaml with the accurate T_A_world for your
#    installation (position_m + rpy_deg, or matrix_4x4).
# 2. Ensure your hand-cam and front-cam publish on the topics named in
#    config/locator.yaml (or update the topic names there).
# 3. Make sure the Fairino SDK path in config/locator.yaml is correct.

# Run
roslaunch path_tag_locator path_tag_locator.launch

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

## Output

- **Topic** `~/tag_world_pose` (`geometry_msgs/PoseStamped`, frame_id="world",
  latched).
- **Service response** includes `tag_b_world_pose`, `position_m`,
  `rpy_deg` (ZYX intrinsic, degrees), and `t_a2b_row_major` for debug.
- **Files** (when `save_result: true`): under
  `~/.ros/path_tag_locator/path_tag_<id>_<timestamp>.{npz,yaml}`.

## Notes

- Coordinate conventions: lengths in meters, RPY in degrees (ZYX intrinsic).
  This matches the FR5 TCP-pose convention used by the Fairino SDK.
- The hand-eye `T_hc2ee` is the OpenCV `calibrateHandEye` "gripper-to-cam"
  result — i.e. the pose of EE expressed in the hand-cam frame.
- The detector uses `dt_apriltags` (Duckietown). Install with
  `pip install dt_apriltags` if missing.
