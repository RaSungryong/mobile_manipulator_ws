# Lift vs `arm_base_z` — problem analysis and calibration method

**Status: analysis only. No transform code has been changed.**
Written 2026-07-30 against Navifra *KU Polishing Robot Driver* v0.16.

The new mobile base adds a lift under the arm. The arm transform still assumes
the arm base sits at a fixed height. This document records exactly why that is
now wrong, how big the error is, and what has to be measured before it can be
fixed. It is the reference for the `TODO(lift-transform)` marker in
`src/apriltag_nav/config/robot.yaml`.

---

## 1. The problem

`arm_base_z` is a **compile-time constant** that flows straight into the arm
base origin in the world frame:

```
robot.yaml  arm_calibration.arm_base_z: 1.0076
   │
   ▼
arm_controller.py:636
   body_off_z = rospy.get_param('~arm_base_z', _calib.get('arm_base_z', 1.0076))
   │
   ▼
arm_controller.py:648-652
   p_A_W = np.array([ x_base + ..., y_base + ..., body_off_z ])
                                                 ^^^^^^^^^^
                              arm base Z in world — never updated
```

`p_A_W` is the arm base origin used to map every CSV world pose into the arm
base frame. So `arm_base_z` is, literally, "how high the arm base is".

On the old base that was a fixed mechanical mount, so a constant was correct.
On the new base the arm base height is
`z0 + (metres per count) x /lift/position`, and **nothing in the code reads
`/lift/position`**. Consequences:

- Move the lift, and every pose-mode target is offset in Z by exactly the lift
  travel. Nothing errors — IK still succeeds, the arm just goes to the wrong
  height. Silent, systematic, full-magnitude.
- Joint-mode tasks (`scan_joints_*`) are **unaffected**: they replay recorded
  joint angles and never touch `_transform_pose`. Only pose-mode
  (`scan_grid_*`, `scan_full_pose`, `TEST_POSE`) is affected.
- The Keyence closed-loop distance correction will partly mask small offsets by
  driving the tool back to 30 mm standoff — which makes the bug *harder to
  notice* while corrupting the recorded pose, and it cannot absorb an offset
  larger than `keyence_activate_threshold` (5 mm).

### Magnitude

Full lift stroke is 0..~7000 counts (driver guide 3.5). The count-to-metre
scale is unmeasured, but the error is *the entire lift displacement* — for any
plausible stroke that is centimetres to tens of centimetres, i.e. 10x-100x the
12 mm mean residual the 4-DOF model was validated to. This is not a tuning
nuisance; it invalidates the pose pipeline.

---

## 2. Interim mitigation (already in place)

Since the transform is unchanged, a guard was added instead of a fix:

- `robot.yaml` `navifra.scan_height_counts` — the lift position the transform
  was calibrated at. `null` = unknown, guard disabled.
- `robot.yaml` `navifra.scan_height_guard` — `warn` (default) | `refuse` | `off`.
- `task_executor._check_lift_scan_height()` runs before every scan and logs or
  refuses when the lift is not at that position.

This does not correct anything. It only stops a wrong-height scan from being
recorded as valid data. **`scan_height_counts` must be filled in before the
guard does anything at all.**

---

## 3. Two viable fixes

### Option A — pin the lift (low risk, recommended first)

Treat the lift as a fixture, not a degree of freedom: home it once per power
cycle, drive it to one scan height, and keep `arm_base_z` a constant
recalibrated at that height.

1. `devices.lift_home()` at task start (`/lift/home`, homes to the lower limit).
2. `devices.lift_goto(scan_height_counts)`.
3. Recalibrate `arm_base_z` at that exact height (see section 4).
4. Set `scan_height_guard: refuse` so a drifted lift blocks the scan.

Keeps the 4-DOF model and its 12 mm validation intact. Costs the lift's working
range.

### Option B — make `arm_base_z` dynamic

`arm_base_z = z0 + k x lift_position`, subscribing `/lift/position`.

Needs, beyond the scale factor `k`:
- **Latching per scan point.** Sample `/lift/position` once when the target is
  computed and reuse it, so a mid-motion lift change cannot desynchronise the
  pose from the transform used to build it.
- **A staleness gate.** `/lift/position` missing or stale must fail the scan,
  not fall back to a constant.
- **Re-validation.** The 328-row paired joint/pose validation was taken at one
  height. It must be repeated at 2-3 heights, or the residual figure quoted in
  `CLAUDE.md` no longer describes the shipped model.

More capable, but it turns a validated 4-DOF model into a 5-DOF one that is
currently validated at exactly one point.

---

## 4. Calibration method (required by both options)

### 4.1 Establish the count origin

`/lift/position` is an **incremental encoder count and is lost on every power
cycle** (driver guide 3.5, 6). `/lift/home` drives to the physical lower limit
switch, which the guide states is always accurate regardless of accumulated
count error. So: home first, always, and treat count 0 as the only trustworthy
absolute reference.

### 4.2 Measure `k` (metres per count)

At a *fixed* base pose, with the arm holding a *fixed* joint configuration:

1. `/lift/home` -> confirm `/lift/homed` true, `/lift/position` ~ 0.
2. Record tool Z in the arm base frame via FK (or Keyence standoff against a
   fixed reference surface).
3. `/lift/position_cmd` to a series of counts, e.g. 1000, 2000, ..., 6000.
4. At each, record `/lift/position` (actual, not commanded) and the measured Z.
5. Least-squares fit `Z = z0 + k x counts`. Report the residual — if it is not
   near-linear, the drive has backlash and Option A becomes the only safe path.

Use the **actual** reported count, never the commanded one, because of 4.3.

### 4.3 The count error that makes this dangerous

Driver guide 3.5 carries a measured warning (2026-07-28):

> Starting a descent while pressed against the **upper** limit counts backwards
> for the first few seconds (about +500). Over a full up-then-down stroke the
> error accumulates to roughly -1000 to -1800 counts past zero.

At any plausible `k`, 1000-1800 counts of count error is a large Z error. This
is the strongest argument for Option A, and for these rules if Option B is
chosen anyway:

- Re-home before every scan session; never trust counts across a direction
  reversal at the upper limit.
- Never drive to the upper hard limit during a scan cycle. The upper limit has
  **no limit switch** — the driver detects it as a stall and raises `CTRL_FAIL`,
  and that is exactly the state that corrupts the count.
- Cross-check `/lift/position` against `/lift/homed` and `/lift/status`
  (`di=` DIR bit shows the lower limit) before trusting a value.

### 4.4 Homing repeatability

Before either option is trusted, quantify the origin: home 10 times, record
`/lift/position` and a physically measured Z each time. The spread is the noise
floor of everything above. If it exceeds the 12 mm the 4-DOF model was
validated to, the lift is the dominant error source and neither option reaches
the current accuracy without mechanical work.

---

## 5. Where the code would change

| What | File / line |
|---|---|
| Constant read | [arm_controller.py:636](../src/apriltag_nav/src/apriltag_nav/arm_controller.py:636) |
| Used as arm base Z | [arm_controller.py:648](../src/apriltag_nav/src/apriltag_nav/arm_controller.py:648) |
| Config value | [robot.yaml `arm_calibration.arm_base_z`](../src/apriltag_nav/config/robot.yaml) |
| Lift state / commands | [navifra_devices.py](../src/apriltag_nav/src/apriltag_nav/navifra_devices.py) — `lift_position`, `lift_home()`, `lift_goto()`, `lift_at()` |
| Interim guard | [task_executor.py `_check_lift_scan_height`](../src/apriltag_nav/scripts/task_executor.py) |
| Offline re-fit | [calibrate_transform.py](../src/apriltag_nav/tools/calibrate_transform.py) — already solves for `arm_base_z`; would need a lift-position column added to the paired CSV |

Note `calibrate_transform.py` already treats `arm_base_z` as a free parameter
with bounds 0.80-1.20 m, so re-fitting at a new fixed height (Option A) needs
no solver change — only a fresh data capture at that height.

---

## 6. Open items

- [ ] Measure `k` (metres per count) — blocks both options.
- [ ] Choose the scan height and fill `navifra.scan_height_counts`.
- [ ] Recalibrate `arm_base_z` at that height; update `robot.yaml` and the
      residual figure in `CLAUDE.md`.
- [ ] Quantify homing repeatability (section 4.4).
- [ ] Decide Option A vs B and set `scan_height_guard` to `refuse`.
- [ ] Unrelated but blocking field validation: drive motor 2 is out for repair
      (`navifra param.yaml` `drive_motor_ids: [1]`), so differential straight
      travel and in-place rotation are not achievable yet.
