# Handover — current state of the workspace

Rewritten 2026-09-02, branch `real`. Audience: whoever (human or AI
assistant) picks up this workspace next. This is the **checklist**: what is
verified on hardware, what is still open, and the rules that hold in the
meantime. The *reasoning* behind every line lives in `CLAUDE.md` (its
sections and its dated Work Log) — read that when a line here surprises you.

The previous version of this file (2026-07-31) described the state right
after the base swap. Everything it listed as open has since been either done
or superseded; it is in git history if you need it.

---

## 1. Verified on hardware

| Item | State | Where the evidence is |
|------|-------|-----------------------|
| Mobile base | Navifra KU Polishing Robot Driver v0.16, both drive motors active (`~/navifra/param.yaml` `drive_motor_ids: [1, 2]`). Navigation, pure pursuit, the odom S-curve and the 6 cm final approach have all driven on the real robot | Work Log 2026-08-12, 2026-08-14 |
| front_cam | Rotated −90° about its optical axis; `mobile_controller` ported and confirmed by three arrivals on tag 105 | Work Log 2026-08-13/14 |
| Lift | `lifter_node` owns it; scale measured (343.2 mm at count 6897, `mm_per_count` 0.04976077, `soft_max_counts` 6900); tasks set it through the CSV `lift_height` column; every task ends with lift origin homing | CLAUDE.md *The base lift* |
| Arm mount | `arm_base_z` **0.652 m** at lift origin (cell design record), `arm_body_offset_y` −0.100, no tilt, yaw exactly 180° | CLAUDE.md *Transform Parameters* |
| Base geometry | 0.90 × 0.70 m footprint, wheel radius 0.0825 / separation 0.65 (owned by `param.yaml`), front_cam at (0.55, 0, 0.300) | CLAUDE.md *Drivetrain geometry* |
| Arm node interfaces | `/arm/state`, `/arm/move_cart`, `/arm/jog_cmd` exercised from `robot_ui` (33 motions), cancel-with-retry confirmed to stop a moving arm | Work Log 2026-08-12 |
| Wrist camera + Ra | Basler mono8 at the sensor's 5 fps; ONNX inference in-process for scans and via `inference_node` for the UI, bit-identical | Work Log 2026-08-12 |
| Build | `catkin_make` clean from scratch on the robot PC (2026-09-02) | Work Log 2026-09-02 |

---

## 2. Open items

### 2-0. ▶ NEXT ON-ROBOT SESSION — do these first (written 2026-09-03, dev side)

The user asked to be reminded of this when back on the robot. Commit
`9b2be76` (dev machine, sim-verified) changed the calibration workflow;
pull + `catkin_make` before anything below.

1. **정반 2 mass "tag not detected" — root-cause with the new survey
   tool before re-running anything.** Suspect: physical cross tags laid
   by the "2번정반 중심" CSV origin (+3.420) while config assumes the
   plate's geometric centre (+3.900, measured 2026-09-04) — a 0.48 m offset. Procedure: park
   at a D-corridor stop, arm to that entry's plan seed pose, robot_ui →
   Scripts → `find_cross_tags` → RUN (sweeps ±0.6 m, reports sightings;
   sim-blind-tested to 1 mm on a manufactured 0.47 m offset).
   - sighting at ≈±470 mm → shift `reference_tags_plate2.yaml` x by
     −0.470 and re-run `generate_calibration_artifacts.py`
   - sighting at ≈0 → placement fine; investigate lighting/occlusion
   - nothing → tags not installed/covered
2. **⚠️ View height is now 0.5 m** (was 0.8; error-budget optimization,
   plans regenerated): half-FOV ≈ 0.35 m, so a 0.47 m placement error
   now sees NOTHING at the seed (at 0.8 m it was border-visible — which
   is exactly why 정반 2 "mostly" failed rather than always). The
   survey is mandatory before the next plate-2 session.
3. Other 9b2be76 changes that alter on-robot behaviour: arm HOMES
   before every base move of a session (`arm.home_before_nav`); final
   chain observations are now 5-frame means; align move failures
   degrade instead of failing the entry when the ref tag is visible
   (results marked `degraded` — check those residuals); robot_ui has a
   Calibration tab (plate selector, dry-run, cancel, online lamp).
4. **Plate 1 ran 2026-09-04 (20/26 ok).** The 6 failures were the
   initial-move clamp (one 0.8 m step from the home TCP, seed 1.15 m
   away) — fixed: chunked approach (`align.max_initial_steps`), retry
   with a re-estimated seed (session correction → anchor → raised), and
   `calibration_plan_plate1.yaml` seeds rewritten from the session
   (`scripts/update_plan_seeds_from_session.py`). Re-run plate 1 to pick
   up the six; expect the retry path to be exercised for the first time.
5. First good session: extract the REAL hand-cam yaw noise from the
   session archive `history` and feed it to
   `path_tag_locator/scripts/error_budget.py` (assumed 0.2° today; the
   xy budget is dominated by yaw × the A→B lever).

### 2-1. 🛑 Every scan CSV is invalid — the cell was replaced 2026-08-21

- `map.yaml` is the new cell (72 tags, origin at the centre of 정반 1).
  Every `task/csv/*` still encodes the OLD cell: joint-mode angles were
  solved for the old arm-over-plate geometry and the old base height
  (1.025 vs 0.652 — the 373 mm drop exceeds the whole 343 mm lift stroke),
  pose-mode grids still describe the old workpiece position.
- `_exec_joint` is a bare `MoveJ` with no reachability or collision check.
  **Running `scan_joints_line*` on the new cell is a collision path.**
- Fix: re-solve every joint CSV; regenerate the grid CSVs for the new
  workpiece. No transform or `lift_height` value rescues the old files.
- Side effect: `scan_full_joints` is unregistered (line1 300 mm vs line2
  150 mm) until both CSVs agree.

### 2-2. Tag installation and calibration of the new cell

- As of 2026-08-24 only zone B's 16 tags (100–112, 500, 501, 505) were
  physically installed. Zone B routes end to end on them; check what has
  been added since before driving elsewhere.
- `path_tag_locator` was refactored into the main stack 2026-09-01
  (`path_tag_locator.launch` runs alongside `mobile_manipulator.launch`,
  owns no hardware). **Verified offline only — no live calibration session
  has run yet.** First session needs: `move_cart` under the align loop,
  detection latency, single-commander discipline (no `TASK`/`GOTO` while
  it runs).
- `path_tag_locator/config/reference_tags.yaml` holds the six 90 mm cross
  tags at their **design-record** poses (±0.600 x, 0/±1.200 y, on the plate
  top, face up, yaw assumed 0). Not yet checked against the physical tags —
  `verify_map_world.py`'s relative-geometry check on the first session is
  what confirms them (a wrong yaw rotates every result about that tag).
  Plate 2 uses `reference_tags_plate2.yaml` (= plate 1 + 3.900 m x) and its
  own plan — plan and ref file swap together.
- `camera_offset` 0.55 (map design value) vs 0.547 (tape measure) is
  unreconciled. If 547 is right, move the tags, not the key.
- **`robot_camera_node` must run with `quad_decimate: 1.0` on hand_cam /
  side_cam** (robot.yaml `robot_camera.quad_decimate`, 2026-09-02) — at the
  library default 2.0 the 640x480 cameras drop or corrupt small tags.
  Verified live: align converged in 3 iterations once set.
- **Calibrated maps so far are biased by the hand-eye translation** —
  `map_world_20260902_145354_handeye_corrected.yaml` (offline reprocess
  with a fitted correction) is the best current estimate of the actual
  tag positions (1–2 cm of plan, tag 131 suspect). Raw `map_world_*.yaml`
  from before the correction carry a 5–7 cm bias; do not feed them to
  navigation.
- **Hand-eye `T_hc2ee.npz` is interim** (2026-09-02): the May-2026 value was
  180° off about the optical axis and made `auto_align` diverge on the
  robot; it was spun by 180° as a stop-gap. Re-run `handeye_calib` on this
  robot (`path_tag_locator.launch use_handeye_calib:=true`, 15–30 captures)
  before trusting calibrated positions to better than a few cm.

### 2-3. Navigation defects known since 2026-08-12

- ~~`align_to_tag()` has no timeout~~ — **fixed 2026-09-02**
  (`align_timeout_s: 20.0`, hop fails on expiry). Same day: align now runs
  after pivots too. **Since 2026-09-04 the stop column depends on the
  direction:** forward `cx + center_x_stop_offset` (0), reverse
  `cx + center_x_stop_offset_reverse` (+400 px = tag at the far right of
  the frame, ~162 mm ahead of the lens; remove the key for one shared
  column) — EXCEPT a reverse hop whose target is a 500-series tag, which
  keeps the forward column (`center_x_stop_offset_reverse_skip_tag_ranges:
  [[500, 599]]`; dock / pivot tags need the base on the designed pose).
  Both columns are drawn on `/front_cam/tag_overlay` (FWD cyan,
  REV magenta). `go_to_next_tag` corrects the next hop's odom distance
  for where the lens actually rests (see the 2026-09-04 Work Log entry).
  Offline-verified only; `mobile_node` AND `robot_camera_node` must be
  restarted, and the first reverse hop watched on the overlay: the whole
  tag must still be in frame at rest. Also 2026-09-02: hops touching
  tags 400–499 (zone A lane) run with prediction OFF — Pure Pursuit when
  the tag is seen, straight command when blind, align at the stop
  (`predictive_centering.disabled_tag_ranges`). **Since 2026-09-04 EVERY
  hop ends with stop → align_to_tag** — the 400→400 align skip
  (`align_skip_tag_ranges`) is retired and the key is ignored with a
  warning; a tag out of view at rest fails the hop on `align_timeout_s`.
  Same day: the align stop is led by the pending rotation of the 0.55 s
  command delay, then settles 0.85 s and re-measures at rest (skid-steer
  overshoot fix, `align_settle_s` / `align_max_passes` /
  `align_lead_target_ratio`); `final_approach_dist` 0.08 and
  `blind_approach_dist` 0.15 (were 0.06 / 0.12); and a launch yaw hold
  (`launch_yaw_hold_dist` 0.15 m, `launch_yaw_gain` 1.0) holds the
  aligned heading through the first 15 cm of every hop — the "sets off
  twisted, one wheel late" symptom — plus a backlash feed-forward
  (`launch_backlash_ff_omega` 0.02 rad/s for 0.3 s, opposing the wheel the
  align left backing; fit it from `launch_peak_yaw_err_deg` in the
  arrival records). PP steering: `move_max_angular_speed` 0.25 (was 0.12),
  `look_ahead_base` 0.25 (was 0.4), and `pp_lateral_reference: base` —
  PP puts the BASE CENTRE on the line so the align lands the lens on the
  tag (the 14:33 records showed the align swinging the lens 10–20 mm
  sideways: 0.55 m × sin(yaw)). Aligned yaw at rest on that run: all
  within ±0.19°. `align_mode: pulse` exists as a fallback. Later the
  same day: `steer_mode: state_feedback` (lateral + PREDICTED heading
  law replacing Pure Pursuit while the tag is in view), median of
  `stop_measure_frames` 3 detections for every decision, and
  `camera_latency_compensation` (image stamp × executed speed). All
  offline-only; watch the first hops for steering oscillation. With the
  base-referenced lateral the steering sign flips in REVERSE (fixed the
  same day; a reverse hop that drifts sideways while the tag is in view
  would be the symptom of getting this wrong). Then, to make forward
  behave like reverse: `center_x_stop_offset` 300 px (forward stops with
  the tag ~12 cm ahead of the lens so the next launch sees it),
  `stop_offset_skip_tag_ranges` (500-series stop on the crosshair both
  ways), `/robot_pose` gained the fore-aft term, the heading hold covers
  the whole hop, and the reverse predictive-centering sign was fixed.
- **Reverse hops see their target tag only ~3 cm out** (bumper hides the
  floor behind the lens). Fixed 2026-09-02 with an odom creep zone
  (`blind_approach_dist` 0.12 m at 0.015 m/s); before it reverse stops
  overshot by 6–17 mm vs 1–2 mm forward. Offline-verified only.
- **The base rolls ~0.55 s after a stop command** (5.8 mm at 0.011 m/s,
  12.6 mm at 0.022). `stop_latency_s: 0.55` fires the stop early by the
  integral of the last 0.55 s of commands. Re-fit from nav_log
  (`arrival` − `aligned` fore-aft, ÷ `commanded_speed_at_stop_mps`) after
  the first live run.
- A frozen `/odom` deadlocks `execute_pure_pursuit()` at minimum speed with
  no error. Still unwritten.

### 2-4. `arm_base_z` does not track the lift

- Pose-mode IK is offset by whatever the lift has travelled (up to 343 mm).
  Joint mode is unaffected. `path_tag_locator` compensates its own chain
  from `/lifter/height`; the main stack deliberately does not yet.
  See `docs/lift_arm_base_z_analysis.md`.

### 2-5. Config placeholders and guards

- `vision_stop.stop_tag_ids: []` — vision soft-stop inert until filled.
- `navifra.require_safety_link: false` — warn-only; set `true` for
  production.
- `keyence_max_step_mm: 1.0` — bring-up clamp doing real work; read
  `docs/keyence_scan_chain.md` before raising it.
- `grid_path_line{1,2}_-5.csv` use `group_id` 4/5, valid in no map.

### 2-6. Documentation debt

- `docs/GUIDE_kr.md` + its PDF are frozen; the correction backlog is the
  *Deferred* table in `CLAUDE.md`. Do not edit the guide without asking.
- `tools/*.py` still use bare scipy `as_matrix()`; they only work because a
  user-site scipy 1.10.1 shadows the system 1.3.3 (see *Coding Conventions*).

---

## 3. Interim operating rules

1. **No `scan_joints_*` / `scan_grid_*` / `scan_full_*` on the new cell**
   (§2-1). `GOTO` within installed tags and `go_home` are fine.
2. One commander of the base at a time: no `TASK`/`GOTO` during a
   calibration session; never run `tools/navigate.py`, `tools/vw_drive.py`
   or `tools/lift_calib_ui.py` while the stack is up (second writers).
   Manual "go N m / turn N deg" moves belong in **robot_ui's Mobile tab**
   (2026-09-04): they run inside `mobile_node` via `/mobile/move_cmd`, so
   they are safe with the stack up — unlike the tools above.
3. `/lifter/*` (ours, guarded) ≠ `/lift/*` (raw driver). Read twice before
   `rostopic pub`. Absolute lift moves are refused until lift origin homing;
   `/lift/homed == true` can still need a re-home (command logged, position
   frozen, no alarm → home again).
4. Startup never moves the arm or the lift. `move_to_home` is explicit.
5. Never `apt install ros-noetic-realsense2-camera`; keep
   `ros-noetic-ddynamic-reconfigure` marked manual.
6. Emergency stop is the PILZ hardware button; `STOP` is secondary.
7. All tuning in `config/robot.yaml` (manipulator) / `~/navifra/param.yaml`
   (base); a `~param` in `mobile_manipulator.launch` overrides `robot.yaml`
   (`soft_max_counts` is one — change both).

---

## 4. Repository / document map

| Where | What |
|-------|------|
| branch `real` (remote `origin/real`) | the whole real-robot effort |
| `CLAUDE.md` | architecture, frames, policies, dated Work Log — the source of reasoning |
| `README.md` | technical documentation (loses to `CLAUDE.md` + `robot.yaml` on any conflict) |
| `docs/GUIDE_kr.md` / `mobile_manipulator_guide_kr.pdf` | Korean operator guide — frozen, see the Deferred table |
| `docs/keyence_scan_chain.md` | Keyence standoff loop record |
| `docs/lift_arm_base_z_analysis.md` | lift-vs-transform analysis |
| `docs/architecture_slides_kr.md` | presentation material |
| `docs/all_tags_position.csv` | generated design positions for all 78 tags |
| `src/path_tag_locator/docs/{USAGE_kr,TROUBLESHOOTING_kr,CALIBRATION_GUIDE_kr}.md` | tag-calibration tool docs |
| `src/path_tag_locator/config/extrinsics.yaml` | measured truth for `T_ab2mb` / `T_mb2fc` (lift at origin) |
| cell design record ("the parent directory's CLAUDE.md" in `CLAUDE.md`) | tag layout, Z datum, mounts. **Not in this checkout's parent** — on the robot PC it is `~/mobile_manipulator_ws_20260824/CLAUDE.md`, next to `make_plate_frame_csvs.py` / `tags_plate{1,2}_frame.csv` |
| `~/navifra/` | base driver install, interface guide PDF, `param.yaml` field tuning |
