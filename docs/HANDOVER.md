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

### 2-0. ▶ NEXT ON-ROBOT SESSION — navigation (written 2026-09-08 evening, on the robot)

Four commits landed on `real` today (`4388538`, `b310326`, `5401ace`,
`f738a10`); only the first three have been driven. Do these in order:

1. **Restart `robot_camera_node`** so the front_cam ground-plane
   correction (`f738a10`) is live: `rosnode kill /robot_camera_node`,
   then `rosrun apriltag_nav robot_camera_node.py` (the launch's
   `~driver_*` params persist on the master). The startup log must show
   `ground-plane correction ON — roll +1.228 pitch -0.504 deg, height
   302.0 mm, D applied`. If it says `disabled:` the CameraInfo D was not
   parsed — stop and report.
2. **Drive the same corridor as 2026-09-08 21:15** (`GOTO 117 → 120 →
   123`, then back to `114`) and compare the `aligned` records in
   `log/apriltag_nav/nav_log/<day>/` with yesterday's: forward lateral
   was +8.8 ± 1.3 mm and reverse −11.6 ± 1.6 mm on EVERY hop; both should
   now be within a few mm with no direction-dependent sign. Yaw at rest
   (±0.2°) and fore-aft (+2…+5 mm forward, 155–158 mm reverse) were
   already fine and must stay so. If a bias remains, its sign per
   direction is the diagnostic — bring the yaml files.
3. Watch the `[Aim]` lines: `at rest: … aim ±x deg (cap …)` on every
   arrival, the pivot converging in one pass, no `LIMITED` unless the
   base was > ~20 mm toward the plate. Hop time is ~24 s (was 9–12);
   `plan_prepare_dist` 0.28 / `aim_drive_speed` 0.03 are the knobs.
3b. **2026-09-09 additions, not yet driven** — restart `mobile_node`
   AND `robot_camera_node`: (a) `ground_plane.front_cam.yaw_deg −0.38`
   (camera yaw vs travel axis); (b) a forward hop after a reverse
   arrival now re-seats the start tag onto the FWD column first (same
   arrival algorithm, then align, then the hop), and a command ENDING on
   a reverse arrival re-seats before it returns (`reseat_at_command_end`:
   calibration / scan arm work happens at the FWD column) — watch for
   `[Reseat] start tag N is 0.16 m ahead …` followed by an `[Aim]` on
   that tag and a second `[Aim]` on the target; (c) the overlay is in mm.
   `tools/check_nav_sequencing.py` and `tools/check_ground_plane.py` are
   the offline regression checks (10 + 17).
3c. **Charging manager (2026-09-09, not yet driven)** — restart
   `task_executor`. Watch: `GOTO 500` → `[Charge] docked — /crevis/charging
   true` → `charging confirmed by the BMS` within 15 s (else
   `dock_failed`, no retry: the contacts / `dock_reverse_m`); at 85 %
   `/crevis/charging false` → current drops → 0.10 m forward → green;
   below 20 % a running task is preempted and the robot returns; after
   any completed task it returns. `tools/check_charging_manager.py`
   (13) is the offline check. `navifra.charging.enabled: false` disables.
4. **Known and not yet fixed** (do not chase as new bugs): manual
   `drive_distance` 0.02 m executes ~11 mm and `pivot_angle` 5° ~3.7°
   (stop-latency lead over-compensates on tiny moves); offsets > ~20 mm
   toward the plate cannot be removed in one hop under the wall cap (a
   launch aim from the START tag is the proposed follow-up); tag 15 of
   the calibration pair has skewed corners — use the pair's centre line,
   never one tag's edge, as an angular reference.
5. The `docs/all_tags_position.csv` LibreOffice lock file and the
   Keyence standoff work (`keyence_standoff.py`, `arm_controller.py`,
   parts of `robot.yaml` / `CLAUDE.md`) are another session's
   UNCOMMITTED changes in this checkout — leave them alone.

### 2-0a. ▶ NEXT ON-ROBOT SESSION — calibration accuracy (2026-09-11, dev side)

Read the 2026-09-11 Work Log entry for the evidence; the checklist:

1. **Restart `path_tag_locator` / `map_calibrator`** to pick up the corner
   re-solve. The log must show `front_cam pose solved from corners` once
   per session. If it warns about missing CameraInfo instead, the run
   silently falls back to the broken path — stop and fix the topic.
2. 🛑 **Do NOT trust the five 09-09 `map_world_*.yaml`.** They ran with
   front_cam's ground-plane correction on and the OLD consumer, so their
   `T_fc2B` was a corrected position + an UNCORRECTED rotation + a
   constant asserted depth (`pose_z` is literally `0.302000` on all 25
   entries). The four 09-08 files are the clean ones. Nothing before
   today is free of the 1.33° false tag tilt.
3. **Re-run one plate-1 session and check the prediction**: the
   base-frame component of the tag-normal tilt should fall ~1.33°
   (≈2.5° → ≈1.2°). Whatever remains is hand-eye, which resolves the
   open 85 %-vs-88 % attribution by subtraction — this run IS the
   disambiguating experiment, so do it before any hand-eye work.
4. Accuracy context, so effort goes to the right place: repeatability is
   already **sd_xy 2.9 mm**; the error is systematic, not noise. An
   offline hand-eye refit was built and **deliberately not applied** —
   it improves the metrics it is fitted to and degrades absolute
   position, because a session only spins the camera about its own
   optical axis and so cannot separate hand-eye from front_cam.
5. ✅ **The six cross tags are NOT a suspect** — they are embedded in
   **precision-machined slots in the 정반**, so `reference_tags.yaml`'s
   ±0.600 / 0, ±1.200 is machined geometry. The face-up orientation is
   separately confirmed from the data (world-frame tilt explains 0.2 %).
   ⚠️ The one term machining does not cover is **print registration** —
   ±0.2 mm on a 90 mm tag is ±0.13° of yaw (~2.8 mm at the lever). Check
   whether the tags are printed inserts or etched.
   **Consequence: the 4–8 mm that re-anchored tags move is CHAIN error**,
   which corroborates the rotation finding and raises the priority of
   step 3.
6. **The tie-breaker is a CAMERA-YAW SWEEP** — one tag, one cross tag, 6
   yaws, base stationary. front_cam cancels by construction, so the
   circle radius is arm-side only: large ⇒ hand-eye, ~0 ⇒ front_cam.
   ~4 min. robot_ui → Calibration → Plan = `정반 1 YAW SWEEP`, then
   `rosrun path_tag_locator analyse_yaw_sweep.py <session_dir>`.
   Full reasoning: **`path_tag_locator/docs/chain_error_diagnosis.md`**.
7. Two cheap checks that may close the 17 mm z: measure the cross tag's
   black border against `tag_a_size_m: 0.090` (1 % scale = 4.75 mm of z;
   −80 needs 86.9 mm), and `rostopic echo -n1
   /hand_cam/color/camera_info` for a stale fx after a resolution change.

### 2-0b. ▶ NEXT ON-ROBOT SESSION — calibration (written 2026-09-03, dev side; still open)

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
   **2026-09-08: the ref-tag pairing was changed on the user's
   instruction** (`REF_RANGES` in the generator — 5/3/5 tags per cross
   tag per column, on both plates). Four entries per plate moved to a
   different cross tag (plate 1: 104→0, 107→1, 117→3, 120→4; plate 2:
   130→0, 133→1, 142→3, 145→4) and got NEW design seeds — the four
   plate-1 ones carry an estimated seed (design + that ref's session
   median), not a measured one, so watch them on the next run. The
   other 22 plate-1 seeds are unchanged. Plate 2 still carries design
   seeds; sessions 20260904_162230 (25/25) and 20260908_120034 (24/24)
   can be fed to `update_plan_seeds_from_session.py` (it now ignores a
   measurement taken against a since-changed ref).
5. First good session: extract the REAL hand-cam yaw noise from the
   session archive `history` and feed it to
   `path_tag_locator/scripts/error_budget.py` (assumed 0.2° today; the
   xy budget is dominated by yaw × the A→B lever).

### 2-1. ✅ Scan CSVs re-solved for the new cell (2026-09-11) — line 2 still open

- The pre-cell-swap CSVs are **deleted**. They were solved for the retired
  base and the old cell; `_exec_joint` is a bare `MoveJ` with no
  reachability or collision check, so running them was a collision path.
- Replaced by the 2026-09-11 RRT set: `base_height_mm` 652, `lift_mm` 0,
  three standoffs. Old task names no longer exist.
- **Pose mode = joint mode again (2026-09-14):** the controller's ACTIVE
  tool frame is the FLANGE (tool 1 from `set_tool_tcp.py` is not in
  effect), so `ArmController` converts the CSV's vision-tip targets to
  flange targets before IK, and the new planner's `rx ry rz` are read as
  `csv_euler: "ZYX"`. Verified only by URDF FK offline
  (`tools/check_pose_vs_joint.py`); watch the startup line `Active tool
  frame is the FLANGE` and compare a pose point with its joint twin on
  the robot. Do not activate tool 1 on the controller without re-expressing
  the hand-eye and the calibration seeds (CLAUDE.md, Coordinate Frames).
- **Hand-eye calibration from robot_ui (2026-09-14):** Calibration tab →
  Hand-eye group. `Auto-sample (sweep)` makes `handeye_calib_node` square
  up on cross tag 0 and orbit it (24 views, tilt/spin diversity), capturing
  where the tag is seen, inside clearance / xy-window / reach rules
  (`handeye_calib.yaml auto:`); `Compute & save` overwrites
  `config/hand_eye/T_hc2ee.npz`. Needs
  `path_tag_locator.launch use_handeye_calib:=true`. **Run on the robot
  2026-09-14 18:41 (base on 102): 23/23 views captured, no skips, no move
  failures; the result puts cross tag 0 within 1 / 0 / 5 mm of map.yaml's
  prediction and its scatter over the 23 poses is 2.1 mm rms (was 15 mm
  with the 09-02 file).** Restart the calibration nodes (they cache the
  npz) and treat the 2026-09-02 hand-eye caveats as closed.
- **`CHARGE` / `UNDOCK` on `/task_command` (2026-09-14, robot_ui Task tab):**
  operator versions of the charging manager's dock-and-charge / undock
  tasks; the charger starts only on the `/crevis/charging true` that
  CHARGE sends after arriving at tag 500. Not yet driven.
- **Inference runs behind the arm (2026-09-14):** after the capture the
  scan loop hands the frame to a worker thread and moves to the next row;
  the Ra arrives as a `result` event on `/arm/scan_progress` and the CSV
  row is filled in a few seconds later. A cancel stops the arm at once but
  `/scan_finished` waits for the frames already taken to be scored. Not
  yet run on the robot — expect the per-point cycle to drop from ~6.6 s
  to ~3 s; if the SCAN log shows `OK` lines but no `ra=` lines, the worker
  is dead (arm_node rosout says why).
- **Task names come from the files** (2026-09-14): `assigned_workpoints_
  <key>.csv` → `scan_pose_<key>` (end-effector poses, IK per point) and
  `rrt_final_path_<key>.csv` → `scan_joint_<key>` (joint-angle path, MoveJ
  replay). Nothing in `TASK_DEFS` names a file any more; `rostopic echo
  /task_list` or robot_ui's Task tab lists what is registered, and
  `RELOAD_TASKS` re-scans task/csv. ⚠️ The joint tasks ARE registered
  again (user instruction) although the assignment problem below is
  unresolved — a `logwarn` per `scan_joint_*` task says so at load. Run
  the `scan_pose_*` twin first.
- ⚠️ The joint files are PATHS: ~30 % of rows are transition/home
  waypoints the arm drives through but must not scan. Handled via
  `is_task_waypoint` → a `scan` flag; pairing with the pose file is on
  `source_point_id`, not `point_id` (that matches only 77 %).
- 🛑 **The group → tag assignment does not survive checking.** Measured
  from the robot's STOP pose (tag − 0.55 m), every group's work points are
  nearest to a DIFFERENT tag than the one it is assigned to, and all of
  them cluster near tags 102–104:

  | group | assigned, max dist | nearest tag, max dist |
  |---|---|---|
  | 104 | 104, 1.00 m | 104, 1.00 m ✓ |
  | 105 | 105, 1.29 m | 103, 0.92 m |
  | 106 | 106, 1.48 m | 103, 0.72 m |
  | 107 | 107, 1.72 m | 103, 0.16 m |
  | 118 | 118, 4.80 m | 102, 0.94 m |
  | 119 | 119, 4.55 m | 103, 0.94 m |

  The work points span only ~0.9 × 0.8 m in total with heavily overlapping
  per-group boxes — ONE area, spread across tags up to 6.4 m apart.
  **Only group 104 is reachable.** Needs the generator's author.
- ⚠️ **Why the joint tasks are disabled and the pose ones are not.** Joint
  angles are relative to the arm base: replay them from a base that is not
  where the planner assumed and the arm's shape is unchanged but its
  absolute position is offset, so "collision-free" does not transfer.
  `_exec_joint` is a bare `MoveJ` with no checks. Pose mode solves IK per
  point, so a mis-assigned group fails the move and reports it — that
  failure is the diagnostic, not a hazard.
- Not yet run on hardware — offline registration only (6 tasks, 0 errors,
  routing verified from `START_TAG` 500).

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
  ⚠️ Its **out-of-plane tilt is still exactly May's 1.312°** — neither the
  180° spin nor the 2026-09-02 4-parameter fit (translation + yaw only)
  ever touched that component, and it is a prime suspect for the residual
  rotation error. Do §2-0a step 3 first: it tells you how much of the
  ~2.5° is hand-eye rather than front_cam, which decides whether a
  `handeye_calib` run is the right fix at all.

### 2-3. Navigation defects known since 2026-08-12

- ~~`align_to_tag()` has no timeout~~ — **fixed 2026-09-02**
  (`align_timeout_s: 20.0`, hop fails on expiry). Same day: align now runs
  after pivots too. **Since 2026-09-08 (dev branch `dev-20260908`, not yet
  merged / driven) the FIRST hop of every command — move, pivot or
  from the dock — aligns on its start tag before setting off (start tag
  must be in view; parked off the tag = command fails, no blind drive),
  and a pivot finishes on the exit tag** — odom until the exit tag is
  in view, tag error from then, slow
  (0.05 rad/s) inside 5°, delay-led stop, then the exit align certifies
  0.2° at rest; keys `pivot_tag_slow_deg` / `pivot_tag_slow_max_angular` /
  `pivot_timeout_s`. **Same evening: forward / reverse arrivals are
  `steer_mode: aim_and_drive`** — stop at first sight of the target tag,
  pivot the base centre onto a straight line to the stop pose on the
  tag's line (capped by the plate wall: `aim_wall_dist_m` 0.45,
  `aim_wall_margin_m` 0.03, plate on the robot's right), drive it, and
  the stop align lands the lens on the tag. Plant: 10–20 mm start
  offsets end within ~1 mm; ≥ 30 mm toward the plate is wall-capped and
  spills into the next hop; ~+11 s per hop. **Driven 21:15 the same
  day:** aims converge, but every stop carried a +8.8 mm (fwd) /
  −11.6 mm (rev) lateral bias — traced to **front_cam being tilted 1.3°**
  (edge angle reads +0.67° for a tag 0.2 m ahead). Fixed by ground-plane
  re-imaging in `robot_camera_node` (`robot_camera.ground_plane`,
  fitted from a tag pair by `tools/fit_front_cam_ground.py`; lever
  confirmed 0.552 m). Correction NOT yet driven — restart
  `robot_camera_node` and re-run a corridor; the `aligned` lateral should
  land within a few mm both ways. **Since 2026-09-04 the stop column depends on the
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
  behave like reverse: `center_x_stop_offset` was raised to 300 px and
  **reverted to 0 the same evening** (user: keep the forward stop on the
  crosshair),
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
- `keyence_max_step_mm: 1.0` — the standoff loop's APPROACH fine step
  (loop rewritten 2026-09-08, `keyence_standoff.py`: whole-range
  engagement, gap-proportional steps, fresh median readings, outcome in
  the CSV row; not yet run on the robot). Read `docs/keyence_scan_chain.md`
  before raising it. `keyence.seek_enabled` stays false until the ±99999
  sentinel's sign is confirmed on the real sensor.
- `grid_path_line{1,2}_-5.csv` use `group_id` 4/5, valid in no map.

### 2-6. Documentation debt

- `docs/GUIDE_kr.md` + its PDF are frozen; the correction backlog is the
  *Deferred* table in `CLAUDE.md`. Do not edit the guide without asking.
- `tools/*.py` still use bare scipy `as_matrix()`; they only work because a
  user-site scipy 1.10.1 shadows the system 1.3.3 (see *Coding Conventions*).

---

## 3. Interim operating rules

1. **Run `scan_pose_*` before its `scan_joint_*` twin on any new path
   pair** (§2-1): pose mode fails IK loudly on a mis-assigned group, a
   joint replay drives it. `GOTO` within installed tags and `go_home` are
   fine.
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
8. **Every result and record lives in the workspace (2026-09-14):**
   `results/ra_maps`, `results/scan_images/<run>/`, `log/apriltag_nav/
   nav_log`, `log/path_tag_locator/…`, `log/ros` (node logs — needs a shell
   that sourced `devel/setup.bash` AFTER the 2026-09-14 `catkin_make`, which
   exports `MM_WS` / `ROS_LOG_DIR`). Nothing goes to `~/.ros`, `/tmp` or
   `$HOME`; `~/.ros/log` is the navifra driver's own and stays.

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
| `docs/ROSBRIDGE_kr.md` / `docs/ROBOT_UI_WEB_kr.md` | operator guides for the two external interfaces: rosbridge JSON on :9090 (Windows `robot_cmd.py`) and the **web operator UI on :8080** (any LAN browser, 2026-09-15 — the whole robot_ui, shared state across tabs) |
| `results/` (`README.md` there) | Ra maps (versioned), per-run scan frames, robot_ui captures |
| `log/` | nav records, calibration sessions / locate runs / hand-eye runs, `ros/<run_id>` node logs, `apriltag_nav/calib_pair` |
| `docs/all_tags_position.csv` | generated design positions for all 78 tags |
| `src/path_tag_locator/docs/{USAGE_kr,TROUBLESHOOTING_kr,CALIBRATION_GUIDE_kr}.md` | tag-calibration tool docs |
| `src/path_tag_locator/config/extrinsics.yaml` | measured truth for `T_ab2mb` / `T_mb2fc` (lift at origin) |
| cell design record ("the parent directory's CLAUDE.md" in `CLAUDE.md`) | tag layout, Z datum, mounts. **Not in this checkout's parent** — on the robot PC it is `~/mobile_manipulator_ws_20260824/CLAUDE.md`, next to `make_plate_frame_csvs.py` / `tags_plate{1,2}_frame.csv` |
| `~/navifra/` | base driver install, interface guide PDF, `param.yaml` field tuning |
