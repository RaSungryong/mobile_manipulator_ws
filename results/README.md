# results/ — task output (since 2026-09-14)

Everything a run PRODUCES lives in the workspace: results here, records under
`../log/`. Nothing is written to `~/.ros`, `/tmp` or `$HOME` any more.

| path | writer | versioned |
|---|---|---|
| `scan_images/<task>_ra_map_<ts>/g<group>_p<point>_sp<source>_i<index>_s<n>.png` (`_sp` = source_point_id since 2026-10-07) | `arm_node` (`save_images`, one folder per run; `point_<id>_sample_<n>_ra_<x>.png` before 2026-10-06) | no — ~10 MB per frame |
| `scan_images/<run>/<run>_ra_measured.csv`, `…/<run>_mark_template.csv` | `arm_node` COLLECT mode (`collect.record_dir`): the hand-measured Ra per scanned point, in the run's own frame folder (`../log/apriltag_nav/ra_measured/` before 2026-10-06 evening) — `../docs/RA_COLLECT_kr.md` | **yes** — only the png / jpg under `scan_images/` are ignored |
| `captures/` | robot_ui Collect tab | no |
| `tip_check/<session>/r<n>_tag<id>.png` | robot_ui `tip_check` plugins (one Basler frame per stop; the records are in `../log/apriltag_nav/tip_check/`) | no |
| `ra_dataset/<run>_dataset.csv` | `tools/merge_ra_dataset.py` — one row per frame with the hand-measured Ra (the run folder's CSV above) and the Ra map joined | no — regenerate from the versioned inputs |

`scan_images/` is created by the next run; every frame up to 2026-10-06 (the
2026-09-14 bulk included) was deleted on 2026-10-06.

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
hook in `devel/setup.bash`), `apriltag_nav/calib_pair_20260915_a/` the
2026-09-15 front_cam tilt-fit snapshots, `apriltag_nav/tip_check/` the tip-tour
records, `apriltag_nav/task_csv_backup/` the planner originals of every task set,
`chain_calib/` the A0-sheet sessions. Layout and the reasoning: CLAUDE.md, *Where run
output lives*.
