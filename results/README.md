# results/ — task output (since 2026-09-14)

Everything a run PRODUCES lives in the workspace: results here, records under
`../log/`. Nothing is written to `~/.ros`, `/tmp` or `$HOME` any more.

| path | writer | versioned |
|---|---|---|
| `scan_images/<task>_ra_map_<ts>/point_<id>_sample_<n>_ra_<x>.png` | `arm_node` (`save_images`, one folder per run) | no — ~10 MB per frame |
| `captures/` | robot_ui Collect tab | no |

`scan_images/20260914_flat_three_runs/` holds the 1273 frames of the three
runs of 2026-09-14, from before frames were split per run (see its README).

**The Ra map CSV moved to `../log/apriltag_nav/ra_maps/` on 2026-09-15**
(user request) — same file (`<task>_ra_map_<ts>.csv`, unchanged format,
`task_executor` → `arm_node`'s `ScanResultWriter`), just filed as a per-run
RECORD next to `nav_log` etc. instead of sitting here beside the large,
unversioned `scan_images` bulk. Still the deliverable, still versioned
(`paths.RA_MAP_DIR`); `tools/ra_map_plotter.py` takes the csv path as an
argument and needed no change.

Records (`../log/`): `apriltag_nav/nav_log/<day>/` one yaml per TASK / GOTO,
`apriltag_nav/ra_maps/` the Ra map CSVs above,
`path_tag_locator/{calibrate,locate,handeye_calib,map_world_*.yaml}`,
`ros/<run_id>/` roslaunch and node logs (`ROS_LOG_DIR`, set by the catkin env
hook in `devel/setup.bash`), `apriltag_nav/calib_pair/` the 2026-09-08
front_cam tilt-fit snapshots. Layout and the reasoning: CLAUDE.md, *Where run
output lives*.
