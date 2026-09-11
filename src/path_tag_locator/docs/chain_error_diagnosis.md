# Diagnosing the calibration chain's rotation error

Written 2026-09-11, dev side. **Not yet run on the robot.**

## The problem

Map calibration is repeatable but systematically wrong. From the nine
archived plate-1 sessions (2026-09-08/09):

| | |
|---|---|
| session-to-session repeatability | sd_xy **2.9 mm** — not the bottleneck |
| tag-normal tilt (clean 09-08 data) | **2.68°** — a rotation error in the chain |
| deviation from design, frame offset removed | rms **6.63 mm** |
| path-tag z | **−63.3** vs design −80 (17 mm, unexplained) |

Floor tags lie flat, so a calibrated tag's normal must come out vertical.
It does not. Expressing that error as a vector and asking which frame holds
it **constant** localises it:

| frame | explains |
|---|---|
| world (sloped floor / tilted ref tag) | **0.2 %** |
| hand-cam (hand-eye rotation) | 79.6 % |
| mobile-base (front_cam / `T_ab2mb`) | 82.3 % |

The world frame is cleanly ruled out — the floor and the cross tags are
fine. But **79.6 vs 82.3 is a tie**, and the two candidate fixes are both
expensive: `handeye_calib` is 15–30 captures, and re-fitting `T_mb2fc`
invalidates a 108-hop navigation verification. Guessing is not an option.

The tie is structural, not bad luck: a calibration session only ever spins
the camera about its own optical axis, and that is precisely the motion a
vertical-tilt signature cannot separate.

## The experiment that breaks the tie — a camera-yaw sweep

**One path tag, one ref tag, N camera yaws, base never moves.**

Then `T_ab2mb @ T_mb2fc @ T_fc2B` is identical in every entry and cancels —
**front_cam cannot appear in the variation at all**. A hand-eye error is
fixed in the EE frame, and sweeping the camera yaw rotates the EE about the
vertical, so the computed tag position traces a **circle**:

- **radius** = the arm-side (hand-eye) error magnitude — needs **no ground
  truth**, which is what makes it usable when the truth is the unknown
- **centre** = everything that does not rotate with the yaw

Sensitivity, forward-modelled through the real chain:

| injected | sweep spread | absolute error |
|---|---|---|
| front_cam rotation +2° | **0.00 mm** | 10.47 mm |
| hand-eye rotation +2° | **22.20 mm** | 20.54 mm |
| hand-eye translation +20 mm | **21.79 mm** | 20.00 mm |

```
radius large                       -> HAND-EYE     -> run handeye_calib
radius ~0, but the normal session
  shows a large absolute error     -> FRONT_CAM    -> re-fit T_mb2fc
```

## Running it

```bash
# plans (already generated; regenerate only if extrinsics/hand-eye change)
#   ⚠️ --diagnostics-only. A full run RESETS the main plans' per-entry seeds
#   to design values, discarding the measured ones.
rosrun path_tag_locator generate_calibration_artifacts.py --diagnostics-only
```

`config/calibration_plan_plate{1,2}_yawsweep.yaml` — plate 1 is tag **103**
/ ref **0**, 6 yaws spanning 355°; plate 2 is tag **141** / ref **3**. The
tag is chosen for the widest *reachable* yaw span, because the resolving
power is angular coverage, not entry count.

In robot_ui: Calibration tab → Plan = `정반 1 YAW SWEEP`. **Park the base on
the tag and leave it there** — a base that moves breaks the common factor
the whole method rests on. ~4 min, 6 entries.

```bash
rosrun path_tag_locator analyse_yaw_sweep.py <session_dir>
```

It fits the circle, prints the per-entry radius, and gives the verdict.
`--self-test` drives the real chain forward with known errors and checks all
four classifications. ⚠️ It reads the per-attempt `entries/`, **not**
`map_world.yaml`: every entry here is the same tag, so that file would keep
only the last one and destroy the measurement.

Thresholds: radius ≥ 6 mm ⇒ hand-eye, ≤ 3 mm ⇒ not the hand-eye (3 mm is
about what this rig scatters anyway), between ⇒ reported as inconclusive
rather than guessed.

## Two other numbers worth closing first

**The 17 mm z.** `pose_t` is strictly linear in tag size, so **1 % scale
error = 4.75 mm of path-tag z**, and landing on −80 needs scale 0.9658 —
equivalently a cross tag whose black border is 86.9 mm rather than 90, or a
hand_cam `fx` 3.54 % off. Both are ten-minute checks: measure the border,
and `rostopic echo -n1 /hand_cam/color/camera_info` (a stale CameraInfo
after a resolution change is the classic cause).

**hand_cam's principal point.** 1 px of `cx` error ⇒ **0.79 mm** of path-tag
displacement, 1:1 (= z/fx at the 0.5 m view). ⚠️ It shows up as per-tag
*scatter*, not a bias: the camera yaw differs per entry, so the world-frame
error vector rotates and **averages to ~zero**. Any check that looks at mean
deviation is blind to it, and so is session-to-session repeatability, since
it is deterministic per entry. Not worth chasing until the rotation error
above is fixed — it is the smaller term.

## What was tried and discarded

**A dual-anchor design** — the same path tag measured from two ref tags at a
**pinned** camera yaw — was built first, then removed. It was correct for
the premise it was designed under (the cross tags were suspect, so pinning
the yaw cancels the chain and exposes the anchors) and exactly backwards
once the cross tags turned out to be machined into the 정반 and the chain
became the target: the pinned yaw cancels the very thing being measured.
Forward-modelled, it moves **0.47 mm** for a 2° hand-eye error and **0.00**
for hand-eye translation — near-blind.

It is in git history (removed 2026-09-11, with `solve_reference_yaws.py` and
`fit_reference_tags.py`). Restore it only if the cross tags turn out to be
**printed inserts** rather than etched: print registration of ±0.2 mm on a
90 mm tag is ±0.13° of ref yaw, which dual-anchor can see and this sweep
cannot. At the 1.24 m lever that is ~2.8 mm — below today's dominant term,
so not worth acting on until the chain is fixed.

## Sensitivities, for reference

The A→B lever (ref tag → path tag) runs 1.12–2.07 m, median 1.24 m.

| quantity | transfers as |
|---|---|
| ref-tag position | 1 : 1 |
| ref-tag yaw | × lever = **21.6 mm/deg** median, 36.2 worst |
| hand_cam `cx`/`cy` | 0.79 mm/px |
| hand_cam scale (fx or tag_size) | 4.75 mm of z per 1 % |

These are properties of the geometry, not of any particular calibration —
they are what make the sweep a sensitive probe in the first place.
