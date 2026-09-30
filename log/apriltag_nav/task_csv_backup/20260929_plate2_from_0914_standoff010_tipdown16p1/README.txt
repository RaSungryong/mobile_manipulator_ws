2026-09-29 20:30 — rrt_final_path_errorX_p000mm_standoff_010mm_height_652mm_plate2.csv
re-made at 16.1 mm of tip-down (was 12.0).

Why: the Keyence's first reading at every scan point of the two runs of
scan_joint_errorX_p000mm_standoff_010mm_height_652mm_plate2 (group 132):
  19:23 run, the 20 mm file: 40 points, mean +3.96 mm (too CLOSE), sd 0.85
  19:29 run, the 12 mm file: 85 points, mean -4.10 mm (too FAR),  sd 1.01
Both say the surface is at ~16.1 mm of tip-down (20 - 3.96 = 16.04,
12 + 4.10 = 16.10). Within the 19:29 run the error drifts from -2.6 mm
(points 17-31) to -5.0 mm (points 84-155); a constant cannot remove that,
the standoff loop does.

How: tools/make_plate2_paths.py <this dir> --tip-down-mm 16.1
     --max-unreachable-mm 14 --max-step-growth-deg 8 --apply, output to a
     scratch directory, ONLY the rrt file installed (the pose twin stays at
     20 mm, as on 19:20). make_plate2_record.yaml is that run's record; the
     assigned_workpoints output it lists was not installed.
replaced/ holds the 12 mm file that was in task/csv.
Verified: only q1..q6 differ from the 12 mm file; FK of new vs old rows:
tip +4.100 mm along the tool z at 941 work points (4.07 at the 2
unreachable ones), sideways 0.000 (max 0.48) mm, largest joint change
1.05 deg.
