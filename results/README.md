# results/ — task output (since 2026-09-14)

Everything a run PRODUCES lives in the workspace: results here, records under
`../log/`. Nothing is written to `~/.ros`, `/tmp` or `$HOME` any more.

| path | writer | versioned |
|---|---|---|
| `ra_maps/<task>_ra_map_<ts>.csv` | `task_executor` → `arm_node` (`ScanResultWriter`), one per TASK run | yes — the deliverable, ~100 KB |
| `scan_images/<task>_ra_map_<ts>/point_<id>_sample_<n>_ra_<x>.png` | `arm_node` (`save_images`, one folder per run) | no — ~10 MB per frame |
| `captures/` | robot_ui Collect tab | no |

`scan_images/20260914_flat_three_runs/` holds the 1273 frames of the three
runs of 2026-09-14, from before frames were split per run (see its README).

Records (`../log/`): `apriltag_nav/nav_log/<day>/` one yaml per TASK / GOTO,
`path_tag_locator/{calibrate,locate,handeye_calib,map_world_*.yaml}`,
`ros/<run_id>/` roslaunch and node logs (`ROS_LOG_DIR`, set by the catkin env
hook in `devel/setup.bash`), `apriltag_nav/calib_pair/` the 2026-09-08
front_cam tilt-fit snapshots. Layout and the reasoning: CLAUDE.md, *Where run
output lives*.
