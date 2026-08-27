# Keyence distance loop + scan chain — calibration and bring-up

Record of the 2026-08-05 bring-up: the Keyence closed loop had never actually
run on this robot, and bringing it up surfaced one collision-risk bug plus
several unit/config inconsistencies. This documents what was measured, what was
changed, and what is still open.

## TL;DR

- The distance loop **had never executed** — `keyence_dlen1_node` was commented
  out in the launch file, so `_adjust_distance_to_surface()` always hit its
  "Keyence value NOT available" path and returned. Scans ran at whatever
  standoff the arm stopped at, with **no error recorded in the scan result**.
- Because it never ran, `keyence_dir` had been **wrong since it was written**
  (+1.0 instead of −1.0). Enabling the node without fixing it would have driven
  the tool into the workpiece, amplifying the error by 1.8× per step.
- The laser is mounted **oblique at 42.6°**, which nothing in the code modelled.
- All of it is fixed and verified on hardware; the loop now converges. Two
  issues remain open — see [Open issues](#open-issues).

## Sensor facts (measured, not from a datasheet)

| Property | Value | How established |
|---|---|---|
| Zero point | reading 0 at **10 mm** standoff | operator, confirms `target_distance_mm` |
| Polarity | too far → **negative**, too close → **positive** | operator |
| Beam angle vs tool Z | **42.6°** (`cos` = 0.7361) | `tools/measure_keyence_angle.py` |
| Sensitivity `k` = d(reading)/d(toolZ) | **+1.358** mm/mm | same, two runs |
| Noise | sd ≈ 3–5 µm | 20 s at rest |
| Publish rate | 30 Hz | `/keyence/value` |

The two polarity statements agree and are the *same fact*: `k > 0` means the
reading rises as the tool approaches, which is what "too close → positive"
says. That is what forces `keyence_dir = -sign(k) = -1.0`.

### Beam angle measurement

`measure_keyence_angle.py` sweeps the tool symmetrically along its own Z axis
and fits reading vs displacement. The slope is the sensitivity; the angle is
just its interpretation, `theta = acos(1/k)`. `k >= 1` always — a normal-
incidence beam gives exactly 1, and any tilt makes the reading move *faster*
than the tool, so a fitted `k < 1` means the measurement is invalid.

| Sweep | k | angle | R² |
|---|---|---|---|
| ±1 mm, 5 pts | 1.3717 | 43.20° | 0.99899 |
| ±2 mm, 5 pts | 1.3450 | 41.97° | 0.99911 |
| **used** | **1.3583** | **42.59°** | |

The 1.2° spread is surface topography, not sensor noise: the oblique beam makes
the spot **translate across the surface** as Z changes, so each sample sees
slightly different material.

⚠️ **Calibrate only on a flat spot.** A third attempt at a different height gave
66.34° with R² 0.929 — invalid, because that spot was sloped. Verify flatness by
moving *parallel* to the surface (base X/Y with the tool vertical): on a flat
surface the reading must not change. That spot measured +1.31 and −0.49 mm per
mm of lateral travel, and the model predicts the bogus result exactly:

```
apparent k = k0 + s*tan(theta) = 1.3583 + 1.3065*0.919 = 2.559   (measured 2.49 / 2.53)
```

## What was wrong, and what changed

| # | Problem | Fix |
|---|---|---|
| 1 | `keyence_dlen1_node` commented out → loop never ran, silently | uncommented in `mobile_manipulator.launch` |
| 2 | `keyence_dir = +1.0` — **wrong sign**, error ×(1+kp) per step, drives into the part | `-1.0`, with the derivation in the launch comment |
| 3 | Oblique beam never compensated — the reading was treated as a perpendicular distance | `keyence.beam_angle_deg`, projected by `cos` in `arm_controller` |
| 4 | `target_distance_mm: 30.0` — read by nothing, matched no hardware setting | `10.0`, marked informational-only |
| 5 | `tol` / `activate_threshold` compared against the **raw beam reading** while the step was perpendicular — mixed units | project first, then everything is perpendicular mm |
| 6 | `robot.yaml` said tol 0.5 / kp 0.5 / max_steps 5 while the launch used 0.2 / 0.8 / 10 | aligned; precedence documented |
| 7 | `max_step_mm`, `activate_threshold` had no yaml fallback at all | added |
| 8 | `keyence.timeout_s` read by nothing (the node has its own `~timeout_s`) | removed |

### Units, once and for all

`keyence_tol`, `keyence_max_step_mm` and `keyence_activate_threshold` are all
**perpendicular standoff millimetres**. The raw reading is projected by
`cos(beam_angle_deg)` *first*, then everything downstream works in that one
unit. At 42.6° the raw reading is 36% larger than the error it represents.

### Loop gain

With the projection applied the reading's `1/cos` factor cancels, so on a flat
surface the per-step residual is `|1 - kp|` — the beam angle drops out. It only
re-enters if `beam_angle_deg` is left at 0, and then the true gain is `kp/cos`,
diverging past 66.4° at kp 0.8.

Note the uncorrected loop was *accidentally* near-optimal: `kp/cos = 0.8/0.7361
= 1.087`, almost deadbeat. That coincidence is why nobody noticed, and it would
have broken the moment kp was retuned or the sensor remounted.

## Current tuning

| Param | Value | Unit | Set in |
|---|---|---|---|
| `beam_angle_deg` | 42.6 | deg | `robot.yaml` |
| `keyence_dir` | −1.0 | sign | launch |
| `keyence_kp` | 0.8 | — | launch |
| `keyence_tol` | 0.2 | perp mm | launch |
| `keyence_max_steps` | 10 | — | launch |
| `keyence_max_step_mm` | **1.0** | perp mm | launch — *temporary bring-up value* |
| `keyence_activate_threshold` | 5.0 | perp mm | `robot.yaml` |

Precedence is `~param` on `arm_node` > `robot.yaml keyence:` >
hardcoded default. `robot.yaml` now mirrors the launch values so both paths
behave identically.

## Verification on hardware

First real closed-loop run (`test_scan_chain.py --move`), 24 PASS / 0 FAIL:

```
t+0.08   -2.018 mm    negative = too far
t+0.75   +1.897 mm    reading rose = tool moved closer   <- correct direction
...
t+6.0    ~0.00 mm     converged
t+7.086  capture      lamp ON, bracketed, 0.27 s
t+9.75   Ra = 0.1718
```

The loop converged before the shutter opened, which is the ordering the scan
depends on.

## Open issues

### 1. Surface slope couples into the loop — one step entered the divergent regime

Because the beam is oblique, a correction of `dz` walks the spot
`0.919*dz` sideways, so each step lands on different topography and the
*effective* sensitivity is not `k0`. Reconstructed from that run:

| step | reading | dz | Δreading | effective k | regime |
|---|---|---|---|---|---|
| 1 | −2.05 | +1.000 | +3.35 | 3.35 | oscillating (limit 3.40) |
| 2 | +1.30 | −0.766 | −2.00 | 2.61 | oscillating |
| 3 | −0.70 | +0.412 | +3.10 | **7.52** | **divergent** |
| 4 | +2.40 | −1.000 | −3.00 | 3.00 | oscillating |
| 5 | −0.60 | +0.353 | +0.35 | 0.99 | monotonic |
| 6 | −0.25 | +0.147 | +0.27 | 1.83 | oscillating |

With `g = kp*cos*k`: `k = 1.70` is deadbeat, **`k >= 3.40` diverges**. Steps 1
and 3 were at or past that. It converged anyway because the 1.0 mm clamp
truncated the runaway steps (1 and 4 both hit it) until the spot reached
flatter ground.

**So `keyence_max_step_mm = 1.0` is load-bearing, not just cautious — do not
restore 5.0 yet.** Candidate mitigations, in increasing order of effort:

- lower `keyence_kp` 0.8 → 0.3–0.4, moving the divergence limit to k ≈ 6.8–9.1
  (costs ~5–6 steps instead of 3 on flat surfaces);
- keep the clamp small so the lateral walk stays small;
- **structural**: compensate along the *beam* instead of tool Z, so the spot
  stays put during a correction. This is a controller change and has not been
  attempted.

### 2. `/odom` publishes nothing

`/base_controller` advertises `/odom` and `mobile_manipulator_system`
subscribes, but no message is ever sent (confirmed independently with
`rostopic hz`). `mobile_controller.py` needs it for Pure Pursuit and the S-curve
profile, so navigation cannot work in this state. Unrelated to the Keyence
work; found by `test_all_devices.py`.

### 3. Reflective dropout

On the polished surface the reading intermittently drops to the ±99999
out-of-range sentinel. `_adjust_distance_to_surface()` treats a missing value
as "skip", so a dropout mid-loop silently ends the correction. Worth confirming
the reading is stable before trusting a scan.

## Test tooling

All three live in `src/apriltag_nav/tools/`, print a `PASS/FAIL/WARN/SKIP`
summary, and exit non-zero if any required check failed.

| Script | Moves the arm? | Purpose |
|---|---|---|
| `test_all_devices.py` | no | network map, arm, Keyence, Basler, USB cameras, and per-topic **data reception** (rate + decoded sample), not just "is it advertised" |
| `test_scan_chain.py` | `--move` only | Basler + VISION lamp + Keyence + arm in scan-point order; probe mode verifies the lamp brackets the shutter without moving |
| `measure_keyence_angle.py` | ±1 mm | the beam-angle calibration above |

```bash
source devel/setup.bash
python3 src/apriltag_nav/tools/test_all_devices.py          # read-only sweep
python3 src/apriltag_nav/tools/test_scan_chain.py           # linkage, no motion
python3 src/apriltag_nav/tools/test_scan_chain.py --move    # one real scan point
```

`--move` runs the full production path including `move_to_home()` at the end,
which is a large motion. It prints the live per-joint deltas and requires typing
`yes`.

## If the sensor is remounted or reconfigured

1. Put the tool in range on a **flat** part of the workpiece; confirm flatness
   by sweeping laterally.
2. `measure_keyence_angle.py --dry-run`, then without it.
3. Write the reported angle to `robot.yaml keyence.beam_angle_deg`.
4. Apply the reported `keyence_dir` — the script computes `-sign(k)` and flags a
   mismatch against the running value.
5. Restart `arm_node`, then `test_scan_chain.py` before `--move`.
