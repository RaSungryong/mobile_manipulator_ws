# Base-Swap Transition Status (handover notes)

Written 2026-07-31, branch `real`. Audience: whoever (human or AI assistant)
picks up this workspace next. The point of this file is the **line between
what is verified-done and what is still open** after the mobile-base swap
(old base → Navifra KU Polishing Robot Driver v0.16). Day-to-day operating
procedures live in `docs/USAGE_kr.md` (Korean, for operators).

---

## 1. Done and verified

| Item | State |
|------|-------|
| Simulation removal | All Isaac Sim support deleted; real-robot-only codebase |
| Architecture | Node-per-device split (4 nodes, same pattern as the Navifra driver); the stale parallel `scripts_ros/` tree deleted |
| Arm control | Single controller `arm_controller.py` (TOOL_ID=1 vision_tip, q0-seeded IK, 4-DOF transform, Keyence closed loop); the two old variants deleted |
| Camera | Always-on streaming → on-demand capture service. Closed at rest, pre-opened at scan-point start (latency hidden under arm motion), VISION lamp bracketed with the shutter |
| New peripherals | E-stop detection (subscriber-thread abort), STATUS lamp from task state, battery warning, full lift API (integration scope: §2-2) |
| Calibration | Arm mount transform switched to the measured truth (`extrinsics.yaml`: base_z 1.025 m, zero tilt, yaw exactly 180°); USD-derived old values retired |
| Verification | 105 automated behaviour checks pass (JSON round-trips, e-stop paths, camera lifecycle, transform numerical equivalence, scan sequencing) |

⚠️ **No integrated on-robot drive/scan validation has been performed yet** —
blocked by the motor repair (§2-1).

---

## 2. Open items (transition checklist)

### 2-1. Drive motor 2 under repair — blocks drive validation

- Symptom: `~/navifra/param.yaml` → `drive_motor_ids: [1]` (only motor 1
  drives). Straight-line travel and in-place rotation are not achievable in
  this state (driver guide §6).
- On repair completion:
  1. In `~/navifra/param.yaml`: delete the `drive_motor_ids: [1]` line and
     restore `motor_directions: [-1, 1]` → `sudo systemctl restart navifra-robot`
  2. Verify straight/rotation per driver guide §4.2
  3. Re-tune AprilTag navigation / Pure Pursuit on the real vehicle
     (`robot.yaml`: `look_ahead_base`, `pp_gain_*`, ...)

### 2-2. Lift — API exists, task integration deliberately deferred

- Current code **never commands the lift**. The driver auto-homes it to the
  bottom limit at launch; it stays there.
- The system is self-consistent **as long as the lift stays at home** —
  `arm_base_z` corresponds to that height. Transition rule: do not move the
  lift manually; doing so silently offsets pose-mode scans (joint mode is
  unaffected).
- Activation sequence (order fixed; details in `docs/lift_arm_base_z_analysis.md`):
  1. [field] choose the scan working height → fill `robot.yaml`
     `navifra.scan_height_counts`
  2. [field] recalibrate `arm_base_z` at that height
     (`tools/collect_calib_data.py` + `tools/calibrate_transform.py`)
  3. [code] wire `lift_home()` → `lift_goto()` into task start and set
     `scan_height_guard: "refuse"` (small change, ready to do on request)

### 2-3. arm_base_z re-validation — required before precision pose scans

- Open discrepancy: the 655-point fit gave 0.9541 m vs measured extrinsics
  1.025 m (71 mm). Either the fit lineage (0428, previously judged
  untrustworthy) or a different lift height at measurement time.
- The old "12 mm mean residual" figure belonged to the retired values —
  **the residual of the current values is unmeasured**. Re-validate with
  paired joint/pose data at a known lift height.
- Until then: trust joint-mode scans (`scan_joints_*`); treat pose-mode
  (`scan_grid_*`, `scan_full_pose`) results as indicative only.

### 2-4. Base dimensions unmeasured — wall-clearance margin affected

- `robot.yaml` `robot.width: 0.50` is the OLD base. The new base's wheel
  separation alone is 0.65 m, so width must exceed that → current
  wall-clearance math is optimistic.
- Measure length/width/camera offset of the new base, update `robot.yaml`,
  recompute `wall_dist_*` / `min_wall_clearance`.

### 2-5. Safety link enforcement — recommended for production

- Currently `navifra.require_safety_link: false` (warn-only when the Safety
  PLC is not confirmed) — a development convenience.
- Switch to `true` for production: task start is then refused unless the PLC
  link is confirmed.

### 2-6. Misc

- `polishing_env/` (608 MB USD assets) is out of git — **exists only on this
  PC**. Copy manually if another machine needs it.
- `src/path_tag_locator.zip`: untracked, presumed manual backup of the
  sibling source dir — confirm and delete.
- Korean operator docs: `docs/USAGE_kr.md` (this system),
  `src/path_tag_locator/docs/USAGE_kr.md` + `TROUBLESHOOTING_kr.md`
  (tag-calibration tool).

---

## 3. Interim operating rules

1. **Do not move the lift** (keep the driver's auto-home position).
2. **Only joint-mode scan data is authoritative** — pose mode is indicative
   until §2-3 is done.
3. No driving (GOTO) until motor 2 is back (§2-1).
4. Emergency: the physical e-stop button (PLC cuts motor power); software
   `STOP` is secondary. Recovery: `USAGE_kr.md` §4.
5. All tuning goes in `config/robot.yaml` (manipulator) /
   `~/navifra/param.yaml` (base) — never hardcode.

---

## 4. Repository / document map

| Where | What |
|-------|------|
| branch `real` | the whole transition effort (remote: `origin/real`) |
| `CLAUDE.md` | technical summary — architecture, frames, camera/arm policies |
| `README.md` | detailed technical documentation |
| `docs/USAGE_kr.md` | operator guide (Korean) |
| `docs/lift_arm_base_z_analysis.md` | lift-vs-transform analysis + calibration method |
| `~/navifra/` | base driver (interface guide PDF, `param.yaml` field tuning) |
| `src/path_tag_locator/config/extrinsics.yaml` | measured truth for the arm mount transform |
