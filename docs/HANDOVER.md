# Handover — current state of the workspace

Rewritten 2026-10-06, branch `real` (remote `origin/real`). Audience: whoever
(human or AI assistant) picks up this workspace next. This is the
**checklist**: what is verified on hardware, what is still open, and the
rules that hold in the meantime. The *reasoning* behind every line lives in
`CLAUDE.md` (its standing sections and its dated Work Log from 2026-09-15 on;
older entries are in `docs/WORKLOG_ARCHIVE.md`) — read that when a line here
surprises you. Previous versions of this file (2026-07-31, 2026-09-02 and
the session notes appended to it through 2026-10-06) are in git history;
everything they listed as open is either done or restated below.

---

## 0. The robot as it stands (2026-10-06)

- **Stack:** boots as the systemd service `mobile-manipulator` after the
  navifra driver (`navifra-robot`); eight `apriltag_nav` nodes + rosbridge
  (:9090) + the web operator UI (:8080). Operate with `sudo systemctl
  {start|stop|restart|status} mobile-manipulator`; never a second hand
  launch while it runs; `tools/stop_stack.sh` for a hand launch. Stopping
  it drops the charge relay.
- **Code on disk vs code running:** the 2026-10-06 night changes (UNDOCK
  without a forward move, every pivot re-seating its start tag on the FWD
  column) and the 2026-10-06 code tidy (dead code removed, `robot_sim`
  fix, dead config keys dropped) have NOT been run on the robot. A
  `sudo systemctl restart mobile-manipulator` is required before the next
  task; the first CHARGE after it still drives from wherever the base is.
- **RESUME (2026-10-06 night, not yet run on the robot):** an interrupted
  TASK continues in its own result files — `RESUME [<task>_ra_map_<ts>]`
  on `/task_command` / Task tab "Resume interrupted run"; finished points
  (Ra map row + answered collect row) are skipped, the rest is scanned
  into the same Ra map / frame folder / `_ra_measured.csv`. Needs the
  same restart. The 17:56 `offset50mm` run (group 105 done) is the first
  candidate. **Group selection (2026-10-07, also not yet run):** `TASK
  <name> groups=105,106` / `RESUME [<run>] groups=…` — the Task tab's
  Groups checkboxes; unticked groups are skipped, not done. **Method B
  (`mark`) redone the same day:** capture + numbered marking stop per
  point + one Ra entry stop per group, rows in `_ra_measured.csv` with
  `mark_no` — no template any more; `arm_node` restart. **Since the
  10-07 evening the number is written at the marking stop as a PENDING
  row (Ra blank), so a RESUME keeps an interrupted group's photos and
  numbers (driven through, not re-shot) and lists them at that group's
  entry stop; a group whose entry stop was the interruption is resumed
  for the entry alone.** **Lift per
  group (same day):** offset50mm group 119 has `lift_mm` 0 (the rest 10);
  `TaskManager` / `task_executor` set the lift per group (descents via
  origin homing) — `task_executor` restart + `RELOAD_TASKS` before the
  file's 0 is honoured.
- **Tasks:** names are derived from the files in `task/csv`. The set
  changed twice on 2026-10-06: a `10mm` pose + joint pair at 13:37
  (`scan_joint_10mm` ran four times; its frames are gone from
  `results/scan_images`), then at 15:54 / 16:42 FIVE JOINT-ONLY files,
  renamed 17:30–18:00 to the user's naming rule
  `<kind>_<product>_<mold>_<plate>_<offset>.csv` (kind = joint | pose IS
  the discovery prefix now — `TaskManager` reads `joint_` / `pose_`, the
  old `rrt_final_path_` / `assigned_workpoints_` are gone, not aliased):
  `joint_hoodouter_lower_plate1_offset{0,10,20,30,40}mm.csv`
  → `scan_joint_hoodouter_lower_plate1_offset<N>mm` (1267 work points
  each, groups 104–107 / 118–120, standoff 17, planner speeds 10–30,
  `lift_mm` set to 0 in ALL FIVE for a test — the exported 0 / 10 / 20 /
  30 / 40 originals are in `task/csv/task_csv_backup/20261006_base_
  height_652_originals/` under the same names; NO `pose_`
  twin, so no pose task and no world x y z in the Ra map; "lower" is
  inferred from the 260610 heights, not confirmed by the user). **Planner originals: not retargeted, the
  speed-10 boundary rule not applied** — `tools/retarget_joint_paths.py`
  cannot classify a file without its pose twin, and `check_task_discovery.py`
  fails on a joint-only set (it assumes pairs). Ask the user for the pose
  twins before scanning; a joint replay has no reach / collision check.
- **Calibrated chain** (`src/apriltag_nav/config/tf/tf_chain.yaml`,
  `tools/tf_chain_tool.py show / check` 13/13): `T_ab2mb` planar, t
  (−9.20, −99.13, −652.00) mm, yaw 179.196° (09-22 fit, xy shifted
  09-28); `T_mb2fc` (0.55, 0, 0.303) with roll +1.406 / pitch −0.323 /
  yaw −0.38° GENERATED from robot.yaml; `T_hc2ee` t (36.46, −331.77,
  −156.14) mm (09-18 sweep re-solved with the checkerboard K);
  `T_ee2tip` (−2.0, −245.2, 214.4) mm; joint offsets J2..J6 −0.34 /
  −0.53 / −0.05 / −0.13 / −0.50° applied in the locator FK AND, since
  09-28 night, on the arm command side (xy only, z kept); hand_cam K
  601.87 / 601.93 / 321.35 / 238.51, k1 +0.177 k2 −0.348.
- **map.yaml:** tags 100–125 carry calibrated x / y / yaw from the
  2026-09-28 16:14 session (tag 104 re-laid and re-measured 21:28); z not
  used. Plates D / E (126–150) are design values, never calibrated.
- **Keyence:** zero and target 16.5 mm (case shortened 2026-09-18), beam
  37.1°, `move_mode: direct` with the live guard, `seek_enabled` false,
  `activate_threshold` 45 mm.

---

## 1. Verified on hardware

| Item | State | Evidence |
|------|-------|----------|
| Navigation | aim-and-drive arrivals, mandatory align on every hop and on the first hop of every command, delay-led stop, ground-plane-corrected front_cam: 108 hops with lateral +1.9 ± 1.7 mm fwd / −3.3 ± 2.2 mm rev, yaw ±0.2° (09-09); stops on the column within 1 mm both ways (09-15); heading-projected `/robot_pose` and the align's at-rest median pose (09-28, tip tour 3) | Work Log 09-09, 09-15, 09-28 |
| `/robot_pose` | live 10 Hz stream (`flag` False) + per-arrival pose (`flag` True); arm_node uses only the arrival pose | Work Log 09-28 (night) |
| Map calibration | plate 1 measured twice on 09-22 (0.9 mm max session spread) and twice on 09-28, applied to map.yaml; tip tour over all 22 plate-1 stops rms 4.2 mm (3.2 without tag 104, which was then re-laid) | Work Log 09-22, 09-28 |
| Pose-mode IK | tip → flange conversion (controller's active tool is the FLANGE), `csv_euler ZYX`, live lift compensation, calibrated `T_ab2mb`, tag yaw in theta, joint offsets on the command side xy | Work Log 09-14, 09-21, 09-22, 09-28 |
| Lift | `lifter_node`; scale 0.04976077 mm/count (343.2 mm at 6897), `soft_max_counts` 6900, tasks set it via `lift_mm` / `lift_height`; re-home signature known (command logged, position frozen, no alarm) | CLAUDE.md *The base lift* |
| Arm node | `/arm/state`, `move_cart` (MoveL), `jog_cmd`, `move_joint` / `jog_joint`, `standoff`, `reset_error`, cancel-with-retry stops a moving arm | Work Log 08-12, 09-15, 09-21, 09-28 |
| Keyence standoff | 2026-09-22 touch tour: 9/9 converged in 3–6 steps; direct mode + guard live since the 09-30 restart (first-move hit rate not yet measured on a scan) | Work Log 09-22, 09-29 |
| Basler + Ra | mono8 at 5 fps; pre-lamp black frames fixed (flush + per-band dark check); inference in-process for scans and via `inference_node` for the UI, bit-identical | Work Log 08-12, 09-15 |
| Charging | CHARGE / UNDOCK driven; 85 % stop; return after a user TASK; dock without BMS current → `dock_failed`, not retried | Work Log 09-09, 09-14, 10-06 |
| External interfaces | rosbridge reached from the Windows PC through the Phoenix bridge (`tools/rosbridge/robot_cmd.py`); web UI used by the operator; both in the main launch | Work Log 09-14, 09-15 |
| Ra data collection | COLLECT mode (pause after capture, Ra typed in the web UI) ran once on 2026-10-06 (14:05 run, one `ra_measured` CSV); batch / mark / premark variants built the same evening; method B ran 2026-10-07 (17:56 run, groups 105/118/119); **Save / Enter writes the typed Ra to the CSV mid-entry** (2026-10-07 evening, `/arm/collect_save`), not yet run on the robot | Work Log 10-06 |
| Build | `catkin_make` clean on the robot PC | 2026-10-06 |

---

## 2. Open items

1. **The two `offset*` joint files are planner originals without pose
   twins** (see §0): the tip lands ~9 mm beside / ~10 mm above the planned
   point in zone B when replayed as exported (09-29 analysis). Get the
   `pose_` twins, decide retarget / speed-10, then
   `RELOAD_TASKS`.
2. **Scan height vs the Keyence window.** The 09-30 run read "out of
   range on the far side" at all 271 points — the surface is more than
   ~27 mm of case standoff below the CSV pose at those points. Check with
   robot_ui's Auto standoff over the workpiece before trusting a scan's
   frames, and collect with `keyence.require_converged: true`.
3. **Frozen `/odom` deadlocks `execute_pure_pursuit` at the floor speed
   with no error** (seen 2026-08-12). Only the manual `/mobile/move_cmd`
   moves have a stall check. Still unwritten.
4. **Arm absolute error ±9 mm, position-dependent** (off_x slope over a
   0.6 m column, 09-21 / 09-28 sheet sessions). No constant fixes it;
   candidates are a per-position correction or the one-frame method
   (ref tag + path tag in one hand_cam frame). The chain's constants are
   now consistent to ~3 mm (tip tour).
5. **Per-parking chassis tilt** is left in the residual by the planar
   `T_ab2mb` rule (≈ 0.9°, ~16 mm of z at a 1 m lever); z is deliberately
   not corrected (user: only x, y matter).
6. **hand_cam intrinsics** far-range / corner coverage is at the minimum
   (0.50–0.60 m: 5 views); a ~1 px radial trend remains in the arm session.
7. **Config placeholders:** `vision_stop.stop_tag_ids: []` (soft stop
   inert); `navifra.require_safety_link: false` (warn-only).
8. **Documentation debt:** `docs/GUIDE_kr.md` is frozen with the
   *Deferred* backlog table in `CLAUDE.md` (the PDF it names does not
   exist on disk; `docs/build_guide_pdf.py` makes it). `tools/*.py` hold
   bare scipy calls that only work through the user-site scipy 1.10.1.
9. **Plates D / E** have never been map-calibrated; the `_plate2` task
   conversions of 2026-09-29 were deleted with the rest of that set and
   would have to be regenerated from `task_csv_backup` if wanted.
10. **The arm Ethernet link (PC `enp2s0` ↔ FR10 controller 192.168.58.2)
   drops for ~12 s 5–23 times a day** (kernel `NIC Link is Down`, every
   working day since 2026-09-02, always back at 100 Mbps; the robot idle
   or not). Each drop freezes whatever Fairino RPC is in flight — a scan
   stood still 13.5 s / 14.7 s twice on 2026-10-06. Hardware job: patch
   cable / connectors on LAN 3 first, then PHY negotiation (`ethtool` not
   installed). Since 2026-10-06 night `/arm/state` carries `link_*` /
   `rpc_*` and the ARM chip reads `LINK DOWN` / `RPC STALL`, so the next
   one is visible without the journal. Unverified: what the controller
   does when a drop lands inside a MoveJ.

---

## 3. Interim operating rules

1. **Run `scan_pose_*` before its `scan_joint_*` twin on any new path
   pair**: pose mode fails IK loudly on a mis-assigned group, a joint
   replay drives it (no reach / collision check anywhere below `MoveJ`).
2. One commander of the base at a time: no `TASK`/`GOTO` during a
   calibration session; never run `tools/navigate.py`, `tools/vw_drive.py`
   or `tools/lift_calib_ui.py` while the stack is up (second writers).
   Manual "go N m / turn N deg" moves belong in robot_ui's Mobile tab
   (they run inside `mobile_node`).
3. No front camera, no tag driving: `mobile_node` refuses every goto while
   front_cam's detections are absent or stale (BASE chip `NO CAM`).
4. `/lifter/*` (ours, guarded) ≠ `/lift/*` (raw driver). Absolute lift
   moves are refused until lift origin homing; `/lift/homed == true` can
   still need a re-home.
5. Startup never moves the arm or the lift; `move_to_home` is explicit.
6. Never `apt install ros-noetic-realsense2-camera`; keep
   `ros-noetic-ddynamic-reconfigure` marked manual.
7. Emergency stop is the PILZ hardware button; `STOP` is secondary. After
   a power-up (and after any e-stop / bumper event) the PNOZ keeps the
   traction power / STO outputs OFF until the panel **RESET** button is
   pressed, and `/safety/estop` reads true until then — the stack boots
   into ERROR (red lamp) when the reset comes after it. Since 2026-10-07
   the release takes ERROR back to IDLE (green) by itself when nothing is
   running; before that it stayed red until the first command.
8. Tuning lives in `config/robot.yaml` (manipulator) / `~/navifra/param.yaml`
   (base); a `~param` in `mobile_manipulator.launch` overrides `robot.yaml`
   (`soft_max_counts`, the Keyence loop keys — change both).
9. Every result and record lives in the workspace: `log/apriltag_nav/
   {ra_maps,nav_log,tip_check,task_csv_backup}`, `results/scan_images/<run>/`
   (frames + the COLLECT-mode CSV), `log/path_tag_locator/…`,
   `log/chain_calib/…`, `log/ros/<run_id>/`. Nothing goes to `~/.ros`,
   `/tmp` or `$HOME`; `~/.ros/log` is the navifra driver's own.
10. Hand launches go detached (`setsid nohup … > log/ros/….out 2>&1 &`),
    never into a VS Code terminal pane (two frozen-pty incidents).
11. After a `tf_chain.yaml` or `map.yaml` change: restart `arm_node` /
    `mobile_node` (the service), regenerate the calibration plans, and
    re-check the joint path files (`tools/check_retarget_joint_paths.py`).

---

## 4. Repository / document map

| Where | What |
|-------|------|
| `CLAUDE.md` | architecture, frames, policies, Work Log from 2026-09-15 — the source of reasoning |
| `docs/WORKLOG_ARCHIVE.md` | Work Log entries 2026-08-07 … 09-14, verbatim |
| `README.md` | Korean quick reference of commands (loses to `CLAUDE.md` + `robot.yaml` on any conflict) |
| `docs/STOP_LAUNCH_kr.md` | systemd service, stopping a hand launch, the frozen-terminal failure |
| `docs/ROSBRIDGE_kr.md` / `docs/ROBOT_UI_WEB_kr.md` | the two external interfaces: rosbridge JSON on :9090 (Windows `robot_cmd.py`) and the web operator UI on :8080 |
| `docs/RA_COLLECT_kr.md` | Ra data collection, the one operator guide: methods A / B, options, RESUME, merge (rewritten short 2026-10-07) |
| `docs/TF_CHAIN_CALIBRATION_STATUS_kr.md` | the calibration chain: frames, every applied value with provenance, error sources, dependency order (as of 09-22 with the 09-28 changes marked) |
| `docs/FRONT_CAM_POSE_CALIBRATION_kr.md` / `docs/HAND_CAM_INTRINSICS_kr.md` | the two camera calibration procedures |
| `docs/keyence_scan_chain.md` | Keyence standoff loop record |
| `docs/lift_arm_base_z_analysis.md` | historical analysis (fixed 2026-09-11); kept for the reasoning |
| `docs/GUIDE_kr.md` | Korean operator guide — frozen, see the Deferred table in CLAUDE.md |
| `docs/all_tags_position.csv`, `docs/robot_base_stop_poses_plate1_*.csv` | generated: tag positions / the base stop poses the retarget tool reads |
| `src/apriltag_nav/config/tf/tf_chain.yaml` (+ npz, `arm_joint_offsets.yaml`) | every fixed transform with design values and provenance |
| `src/path_tag_locator/README.md`, `docs/CALIBRATION_GUIDE_kr.md`, `docs/TROUBLESHOOTING_kr.md` | map-calibration package docs |
| `src/chain_calib/README.md` | A0-sheet chain calibration, hand_cam intrinsics, Basler tip |
| `results/README.md` | what a run writes under `results/` |
| `log/` | records: nav_log, Ra maps, tip_check, task_csv_backup, calibration sessions, `ros/<run_id>` |
| `~/navifra/` | base driver install, interface guide PDF (copy in `docs/`), `param.yaml` field tuning |
| cell design record | the parent directory's CLAUDE.md on the machine the cell was designed on; not in this checkout — the numbers it fixed (origin at the centre of 정반 1, z datum, 652 mm arm base) are in CLAUDE.md |
