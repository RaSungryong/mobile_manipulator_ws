# Hand-Eye Calibration File

> ⚠️ **2026-09-02 — `T_hc2ee.npz` is an INTERIM value, not a fresh calibration.**
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
