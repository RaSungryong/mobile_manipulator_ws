# Hand-Eye Calibration File

> ✅ **2026-09-14 — `T_hc2ee.npz` IS a real calibration of this robot.**
> Produced by `handeye_calib_node`'s automatic sweep (run
> `$MM_WS/log/path_tag_locator/handeye_calib/run_20260914_183840`, 23 samples,
> tilts 0/12/22°, spins ±30°, three distances; DANIILIDIS, AX=XB residual
> 0.0173). Checked two ways: the fixed cross tag 0 re-projected through
> all 23 poses scatters **2.1 mm rms / 3.2 mm max** (the 09-02 file gave
> 15.1 / 24.7 on the same samples), and its absolute position in the arm
> frame, base aligned on tag 102, is (−0.399, 1.010, −0.577) m against
> map.yaml's prediction (−0.400, 1.010, −0.572): **1 / 0 / 5 mm**. The
> 09-02 file (git history, commit 68bac0d) was 45 mm / 2.3° away from it.
> Everything below the line is the history of how it got there.
>
> ⚠️ **2026-09-02 — `T_hc2ee.npz` was an INTERIM value, not a fresh calibration.**
> The file committed 2026-05-27 (kept here as
> `T_hc2ee_2026-05-27_old_mount.npz`) describes a hand-cam mount that is
> rotated **180° about the optical axis** relative to the camera as it is
> mounted today. Symptom on the robot: `auto_align` DIVERGED — xy offset and
> tilt doubled every iteration (66→131→258 mm, 5→8.5→17°) until the pose
> became unreachable (Fairino error 112). A simulation of the align loop with
> that exact mounting error reproduces the numbers, and the live hand-cam
> observation of ref tag 0 lands on the map position only with the spun
> transform (15 mm vs 230 mm). The current file is therefore
> `Rz(180°) @ old` — right rotation, translation inherited from May.
> **Re-run the hand-eye calibration on this robot** (steps below) and
> overwrite it; until then treat every calibrated position as carrying the
> May translation's error budget.
>
> **Same day, second correction — translation fitted from field data.**
> The spun-only file (kept as `T_hc2ee_2026-09-02_spun_only.npz`) gave a
> repeatable (7–8 mm) but biased map: +65 mm in zone D, −49 mm in zone E,
> adjacent tag spacing 359–464 mm for a 400 mm grid. Fitting a 4-parameter
> hand-eye correction (camera-frame translation + yaw) from PHYSICAL
> constraints only — 400 mm spacing and straight corridors, no design
> coordinates — gave dt = (+101, −3, +24) mm, yaw +1.55°, and with it the
> same 22 observations land within 1–2 cm of the design positions. That is
> the file now in use. Still not a proper calibration: `handeye_calib` on
> this robot remains the real fix.

> ⚠️ **2026-09-18 — the hand camera was MOVED, so the 09-14 file describes
> the OLD mount and is only the aiming estimate until `compute` overwrites
> it.** First sweep attempt that afternoon: the square-up DIVERGED (xy
> 289 → 373 mm, tilt 3.0 → 7.7°, tag lost on the second step) — the
> 09-02 signature again, a different mount. The sweep now catches that
> (retreats to the pose where the tag was seen) and **bootstraps its own
> aiming hand-eye**: it captures the start view and six ±10° rotations
> about the flange axes, solves a provisional T_hc2ee from just those,
> squares up with it and runs the normal sweep (`handeye_calib.yaml
> auto.bootstrap: auto`). Nothing to configure after a remount — start
> the sweep from ~0.6–1.2 m above the tag with the tag in view, as before.
> `compute` then writes the real file. The 09-14 file is kept as
> `T_hc2ee_2026-09-14_old_mount.npz`.
>
> ✅ **2026-09-18 15:01 — `T_hc2ee.npz` is the NEW mount** (run
> `run_20260918_144420`, two sweeps in one node session: 14:46 from 1.2 m
> and 14:48 from 0.62 m, both bootstrapped). t = (37, −340, −153) mm, rpy
> (0.46, −0.47, −179.3)° — the camera is now 0.37 m from the flange and
> spun ~180° from the 09-14 mount, which is exactly why the old file
> diverged. Computed OFFLINE from the **18 sweep samples** with the
> refined `calibrate()` (`result_sweep_only_refined.npz` in the run dir):
> tag 0 re-projected through the 18 poses scatters **2.3 mm rms / 4.8 max**,
> normal 0.82° rms — the 09-14 quality (2.1 / 3.2); jackknife sd 3.5 /
> 3.6 / 1.7 mm on t, 0.5–1.2 mm on the located tag. The node's own
> 14:49 `compute` (ANDREFF over all 32 samples, the 14 bootstrap
> rotations included — 7 of them at 1.2 m where the tag is ~70 px and
> each scattered 17–22 mm) is kept as
> `T_hc2ee_2026-09-18_node_all32_andreff.npz`: it scattered the tag
> 11.1 mm rms and sits **25 mm / 1.4°** from the file in use. Since then
> the node drops the bootstrap samples after the provisional solve and
> refines every compute, so the next `Compute & save` gives this quality
> directly. ⚠️ Absolute check NOT closed: tag 0 lands at (−389.5, 992.4,
> −580.2) mm in the arm frame, base aligned on 102 (2.3 mm from the
> tag), against map.yaml's (−400, 1010, −571.5): **10 / −18 / −9 mm**,
> where the 09-14 file agreed to 1 / 0 / 5. The jackknife says the
> 18-sample solution is stable to ~1 mm on that number, so the 20 mm is
> systematic — hand-eye bias from fewer views (18 vs 23, two tilt-22
> views, 0.37 m lever) or something in the base/map side.
>
> **15:10 — third sweep, aimed by the new file: no divergence, 17/17
> views, the tag 2–8 mm from the image centre at every view** (the
> aim's own confirmation). File updated to the solve over all **35 sweep
> samples** (`result_sweep123_refined.npz`): t = (36, −335, −152) mm,
> rpy (0.14, −0.45, −179.4)°, tag scatter 2.7 mm rms / 5.1 max, jackknife
> 2.8 / 2.2 / 1.5 mm on t. The independent third sweep alone solves to
> 12.6 mm / 1.1° from the 18-sample file and puts tag 0 at the SAME place
> vs the map: **10 / −21 / −3 mm** (all 35: 10 / −19 / −5, jackknife
> 0.5 / 0.7 / 1.6 mm). So the offset is not sample noise: in world axes
> it is −19 mm ACROSS the lane (world −x, the tag 19 mm nearer the
> robot's lane than map.yaml says) and −10 mm along it. Candidates that
> software cannot separate: a hand-eye bias particular to this lever, the
> arm mount's `arm_body_offset_y`, or tag 102's map position — the 09-14
> mount agreed to 1 / 0 / 5 with the same chain, so at least one of them
> changed with the remount. `chain_calib` (2026-09-15) or a tape measure
> from the base to cross tag 0 is the next step, not another sweep.

This directory holds the hand-eye calibration result that the locator
node loads at startup.

**Expected file**: `T_hc2ee.npz` — a 4x4 numpy array (any single key) storing
the pose of the end-effector expressed in the hand-camera frame
(equivalent to OpenCV `calibrateHandEye` output `T_gripper2cam`).

The file is intentionally *not* shipped with the package because it is
specific to each physical robot/camera mounting. Produce it once per
installation with the calibration node provided in this package:

```bash
# 1. Edit config/handeye_calib.yaml (topics, tag_id, tag_size, robot_ip,
#    output_path, ...).
# 2. Launch the calibration node:
roslaunch path_tag_locator handeye_calib.launch

# 3. For each calibration pose (move the arm so the hand-cam sees the
#    calibration tag from a different angle; ~15-30 distinct views):
rosservice call /handeye_calib/capture "{}"

# 4. Check progress:
rosservice call /handeye_calib/status "{}"

# 5. When enough samples are collected, run calibration:
rosservice call /handeye_calib/compute "{}"
# -> writes T_hc2ee.npz to the output_path configured in the yaml.

# 6. (Optional) clear samples and restart:
rosservice call /handeye_calib/reset "{}"
```

Once `T_hc2ee.npz` exists, `path_tag_locator_node.py` will load it on
startup.
