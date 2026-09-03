# robot_sim

Kinematic simulator for testing the navigation and map-calibration
stacks WITHOUT the robot. It emulates only the hardware-owner boundary;
the code under test — `mobile_node`/`mobile_controller` and the whole
`path_tag_locator` calibration pipeline — runs **unmodified**.

⚠️ DEV MACHINE ONLY: sim_node publishes the real driver topic names on
purpose (`/odom`, `/front_cam/tag_detections`, `/arm/state`, …). On the
robot PC it would fight the real stack.

## What is simulated

| Boundary | Simulation |
|---|---|
| base drive | unicycle kinematics from `/cmd_vel` → `/odom`; first-order velocity lag (`~vel_tau_s` 0.55) reproducing the measured post-stop roll that `stop_latency_s` compensates |
| front camera | pinhole projection of map.yaml floor tags → `AprilTagDetectionArray` + `CameraInfo`, exact robot_camera_node euler encoding, corner order matching the hardware-verified `tag_edge_angle_deg` convention, all-corners-visible gating (reproduces the real "tag past the frame edge" failure mode) |
| hand camera | same, viewing cross tags through base ∘ T_ab2mb(lift-compensated) ∘ TCP ∘ inv(hand-eye) |
| arm | `/arm/state` + `/arm/move_cart`/`jog` with the motion_seq protocol and the exact `"move_cart ok"` attribution string; `/arm/move_home` |
| lifter | `/lifter/state` (JSON), `/lifter/height`, `height_cmd`, `home`/`stop` services |
| safety | `/safety/estop` latched false |

Ground truth streams on `/robot_sim/ground_truth` (JSON); tag markers on
`/robot_sim/markers` for rviz. Noise knobs: `~noise_px`, `~noise_t_m`,
`~odom_noise` (default 0 = perfect sensors).

## Usage

```bash
roslaunch robot_sim sim.launch          # sim + REAL mobile_node + live viewer
roslaunch path_tag_locator path_tag_locator.launch   # for calibration tests

rosrun robot_sim test_nav.py            # E2E navigation (501, then 105)
rosrun robot_sim test_calibration.py 105    # E2E calibration chain
```

### Visualization

`sim.launch` starts a live top-down viewer (`viz:=false` to disable):
floor tags colored by zone, cross tags magenta `+`, plates dashed, robot
body + heading arrow, front (orange) / hand (red) camera positions, an
orange/red ring around every tag each camera currently detects, and the
driven trail. Headless capture:

```bash
rosrun robot_sim viz_node.py --snapshot route.png --watch 120
```

Manual driving / calibrating: exactly the real interfaces —
`/mobile/goto_tag`, `/map_calibrator/run_calibration`, robot_ui also
works against the sim.

## Verified baseline (2026-09-03, zero noise)

- goto 501 (single hop + align): error **0.1 mm / 0.0 mm / 0.00°**
- goto 105 (pivot + reverse corridor entry + 8 hops, 80 s):
  **0.2 mm / 0.0 mm / 0.00°**
- calibration of tag 105 vs ref 1 (auto-align at 0.8 m + full chain):
  **0.00 mm in x, y AND z** (z = −0.080 exactly — the health check)

The sim's forward model and the calibration chain's inverse are fully
independent code paths, so agreement at this level validates both. Two
convention traps the sim caught while being built (kept as regression
value): the corner order IS the alignment convention (wrong order made
the 9/2 heading-correction servo the robot ~6° off), and an angular lag
that is too slow pushes the tag's corners past the frame edge at arrival
— the same failure the real robot showed on 2026-08-14.
