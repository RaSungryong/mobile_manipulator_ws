# Hand-Eye Calibration File

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
