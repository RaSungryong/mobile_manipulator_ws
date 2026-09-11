#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import rospy
from collections import defaultdict
from typing import Dict, List, Optional


# ---------------------------------------------------------------------------
# CSV dialect helpers (2026-09-11)
#
# The RRT-planned scan CSVs interleave the collision-free path between work
# points with the work points themselves, and renumber `point_id` along the
# whole path. Two optional columns carry the difference; both default to the
# original behaviour when absent, so older CSVs load unchanged.
# ---------------------------------------------------------------------------
def _is_scan_row(row) -> bool:
    """Is this row a work point, or just a waypoint to drive through?

    `is_task_waypoint` 0 marks a transition/home pose: the arm must MOVE
    there (it is the planned collision-free route) but must not settle,
    run the Keyence standoff loop or capture — none of which mean anything
    away from the surface. Absent column = every row is a work point.
    """
    v = str(row.get("is_task_waypoint", "")).strip()
    return True if v == "" else v not in ("0", "0.0", "False", "false")


def _as_int(value, default=0) -> int:
    """Integer from a CSV cell, tolerating float spelling.

    The assigned_workpoints_* files write integer columns in scientific
    notation ('0.000000000000000000e+00'), which plain int() rejects. Going
    through float() accepts both dialects; an empty or unparsable cell falls
    back to `default` rather than killing the whole task load.
    """
    try:
        return int(float(str(value).strip()))
    except (TypeError, ValueError):
        return int(default)


def _work_point_id(row) -> int:
    """The work-point index a row belongs to.

    `source_point_id` when present (RRT dialect), else `point_id`. This is
    the key that pairs a joint CSV with its pose CSV; using `point_id` on an
    RRT file pairs only ~77 % of the rows, silently dropping the rest.
    """
    v = str(row.get("source_point_id", "")).strip()
    if v:
        return _as_int(v)
    return _as_int(row.get("point_id", 0))


class TaskManager:
    """
    TaskManager (Explicit + System Tasks)
    ====================================
    Responsibilities:
    - Load explicitly-defined CSV tasks
    - Register system tasks (no CSV)
    - Build dynamic runtime tasks (GOTO)
    - NEVER guess task type or scan mode

    Task step format:
        {
            "tag": int,
            "scan": bool
        }

    Scan point format:
        pose:
            {
              "mode": "pose",
              x,y,z, rx,ry,rz, speed
            }
        joint:
            {
              "mode": "joint",
              joints[6], speed
            }
    """

    # ==================================================
    # USER CONFIG
    # ==================================================

    START_TAG = 500   # Home tag (dock)

    TASK_DEFS = {

        # ---------------- scan: RRT-planned, 2026-09-11 ----------------
        # Re-solved for the REPLACEMENT base (base_height_mm 652, lift_mm 0)
        # after the 2026-08-21 cell swap invalidated everything before them.
        # Three standoffs, registered separately because the standoff changes
        # which tag each work point is assigned to, not just the offset.
        #
        # 🛑 THE JOINT TASKS ARE DISABLED — the group -> tag assignment in
        # these files does not survive checking, and replaying an RRT
        # trajectory from the wrong base position is a COLLISION path, not a
        # bad-data one. Joint angles are relative to the arm base: if the base
        # is not where the planner assumed, the arm's shape is unchanged but
        # its absolute position in the cell is offset by that error, so
        # "collision-free" does not transfer. _exec_joint is a bare MoveJ with
        # no reachability or collision check, and the Keyence loop only runs
        # after the move lands, so nothing downstream can intervene.
        #
        # The evidence (2026-09-11, offline): every group's work points are
        # nearest to a DIFFERENT tag than the one it is assigned to, and all
        # of them cluster around tags 102-104 --
        #
        #   group   assigned   max dist   nearest tag   max dist
        #     104        104      1.00 m          104     1.00 m   ok
        #     105        105      1.29 m          103     0.92 m
        #     106        106      1.48 m          103     0.72 m
        #     107        107      1.72 m          103     0.16 m
        #     118        118      4.80 m          102     0.94 m
        #     119        119      4.55 m          103     0.94 m
        #
        # The work points span only ~0.9 x 0.8 m in total and the per-group
        # bounding boxes overlap heavily, i.e. this is ONE area subdivided,
        # yet the groups place it across tags up to 6.4 m apart. Distances use
        # the robot's STOP pose (tag - 0.55 m camera_offset), not the tag.
        #
        # The POSE tasks below stay enabled: they solve IK per point, so a
        # mis-assigned group fails the move and reports it instead of driving
        # somewhere planned for a different base position.
        #
        # To re-enable once the generator's author has confirmed the
        # assignment: uncomment the three entries. Nothing else changes --
        # the loader already handles the dialect.

        # "scan_rrt_standoff010": {
        # "file": "rrt_final_path_errorY_p000mm_standoff_010mm_height_652mm.csv",
        # "pose_file": "assigned_workpoints_errorY_p000mm_standoff_010mm_height_652mm.csv",
        # "type": "scan",
        # "scan_mode": "joint",
        # "result_name": "scan_rrt_standoff010_ra_map.csv",
        # },

        # "scan_rrt_standoff030": {
        # "file": "rrt_final_path_errorY_p000mm_standoff_030mm_height_652mm.csv",
        # "pose_file": "assigned_workpoints_errorY_p000mm_standoff_030mm_height_652mm.csv",
        # "type": "scan",
        # "scan_mode": "joint",
        # "result_name": "scan_rrt_standoff030_ra_map.csv",
        # },

        # "scan_rrt_standoff050": {
        # "file": "rrt_final_path_errorY_p000mm_standoff_050mm_height_652mm.csv",
        # "pose_file": "assigned_workpoints_errorY_p000mm_standoff_050mm_height_652mm.csv",
        # "type": "scan",
        # "scan_mode": "joint",
        # "result_name": "scan_rrt_standoff050_ra_map.csv",
        # },

        # ---------------- scan: pose mode (the only enabled ones) -------
        # IK-solved per point from the world coordinates. This is what makes
        # them safe to leave registered while the group assignment is in
        # doubt: an unreachable point FAILS the move and says so, rather than
        # executing a trajectory planned for a base that is somewhere else.
        # Expect group 104 to work (max 1.00 m from its stop) and 105 onward
        # to fail progressively — that failure IS the diagnostic.

        "scan_grid_standoff010": {
            "file": "assigned_workpoints_errorY_p000mm_standoff_010mm_height_652mm.csv",
            "joint_file": "rrt_final_path_errorY_p000mm_standoff_010mm_height_652mm.csv",
            "type": "scan",
            "scan_mode": "pose",
            "result_name": "scan_grid_standoff010_ra_map.csv",
        },

        "scan_grid_standoff030": {
            "file": "assigned_workpoints_errorY_p000mm_standoff_030mm_height_652mm.csv",
            "joint_file": "rrt_final_path_errorY_p000mm_standoff_030mm_height_652mm.csv",
            "type": "scan",
            "scan_mode": "pose",
            "result_name": "scan_grid_standoff030_ra_map.csv",
        },

        "scan_grid_standoff050": {
            "file": "assigned_workpoints_errorY_p000mm_standoff_050mm_height_652mm.csv",
            "joint_file": "rrt_final_path_errorY_p000mm_standoff_050mm_height_652mm.csv",
            "type": "scan",
            "scan_mode": "pose",
            "result_name": "scan_grid_standoff050_ra_map.csv",
        },

        # ---------------- single-group bring-up ----------------
        # Group 104 is the ONE group whose work points are actually within
        # reach of the tag it is assigned to (max 1.00 m from the stop pose;
        # every other group is nearest to a different tag — see the table
        # above). This task exists to exercise the whole chain end to end —
        # navigate to 104, arm to each point, Keyence standoff, capture,
        # inference, result CSV — on data that should work, before anyone
        # tries to interpret a failure from the mis-assigned groups.
        #
        # Pose mode on purpose: IK is solved per point, so anything out of
        # reach fails the move and says so. Expect 182 of 183 points to
        # solve; the one at 1.41 m is just past the 1.40 m flange reach and
        # failing there is correct behaviour, not a bug.
        "scan_g104_standoff010": {
            "file": "assigned_workpoints_errorY_p000mm_standoff_010mm_height_652mm.csv",
            "joint_file": "rrt_final_path_errorY_p000mm_standoff_010mm_height_652mm.csv",
            "groups": [104],
            "type": "scan",
            "scan_mode": "pose",
            "result_name": "scan_g104_standoff010_ra_map.csv",
        },

        # ---------------- move-only CSV  --------
        # "move_route_A": {
        #     "file": "move_route_A.csv",
        #     "type": "move",
        # },

    }

    # ==================================================
    # INIT
    # ==================================================
    def __init__(self, task_dir: str):

        self.task_dir = task_dir

        # task_name -> [{tag, scan}]
        self.tasks: Dict[str, List[dict]] = {}

        # task_name -> tag_id -> [scan points]
        self.scan_points: Dict[str, Dict[int, List[dict]]] = defaultdict(dict)

        # task_name -> lift height [mm], or None when the CSV has no
        # lift_height column. See _extract_lift_height.
        self.lift_heights: Dict[str, Optional[float]] = {}

        rospy.loginfo(f"[TaskManager] Loading tasks from: {task_dir}")

        self._load_csv_tasks()
        self._register_system_tasks()

    # ==================================================
    # LOAD CSV TASKS
    # ==================================================
    def _load_csv_tasks(self):

        if not os.path.isdir(self.task_dir):
            rospy.logerr(f"[TaskManager] Task dir not found: {self.task_dir}")
            return

        for task_name, cfg in self.TASK_DEFS.items():

            # Input CSVs: accept `files` (list) or `file` (single), concat in order
            input_files = cfg.get("files") or (
                [cfg["file"]] if cfg.get("file") else []
            )
            if not input_files:
                rospy.logerr(f"[TaskManager] Task '{task_name}' has no input file(s)")
                continue

            all_rows = []
            missing = False
            for fname in input_files:
                fpath = os.path.join(self.task_dir, fname)
                if not os.path.isfile(fpath):
                    rospy.logerr(f"[TaskManager] CSV not found: {fpath}")
                    missing = True
                    break
                all_rows.extend(self._read_csv(fpath))
            if missing or not all_rows:
                if not all_rows:
                    rospy.logwarn(f"[TaskManager] Empty input for '{task_name}'")
                continue

            # Optional `groups` filter: run only these group_ids from the
            # CSV. Lets one known-good group be exercised end to end without
            # copying the file — a derived CSV is a second source of truth
            # that drifts the moment the original is regenerated.
            want = cfg.get("groups")
            if want:
                want = {int(g) for g in want}
                kept = [r for r in all_rows if _as_int(r.get("group_id")) in want]
                if not kept:
                    rospy.logerr(
                        f"[TaskManager] Task '{task_name}': groups {sorted(want)} "
                        f"match no row. Refusing to register.")
                    continue
                rospy.loginfo(
                    f"[TaskManager] Task '{task_name}': groups filter "
                    f"{sorted(want)} kept {len(kept)} of {len(all_rows)} rows")
                all_rows = kept

            # Lift height for the whole task. A disagreement is fatal for the
            # task, so resolve it before anything gets registered.
            lift_ok, lift_mm = self._extract_lift_height(task_name, all_rows)
            if not lift_ok:
                continue

            # Output path: explicit `result_name` or `{first_input_stem}_result.csv`
            result_name = cfg.get("result_name")
            if result_name:
                result_csv_path = os.path.join(self.task_dir, result_name)
            else:
                base, ext = os.path.splitext(input_files[0])
                result_csv_path = os.path.join(self.task_dir, f"{base}_result{ext}")

            task_type = cfg["type"]

            if task_type == "scan":
                scan_mode = cfg["scan_mode"]

                # Paired joint CSV(s) for IK seed (pose mode)
                joint_rows = None
                if scan_mode == "pose":
                    joint_files = cfg.get("joint_files") or (
                        [cfg["joint_file"]] if cfg.get("joint_file") else []
                    )
                    if joint_files:
                        joint_rows = []
                        for jf in joint_files:
                            jpath = os.path.join(self.task_dir, jf)
                            if os.path.isfile(jpath):
                                joint_rows.extend(self._read_csv(jpath))
                        rospy.loginfo(
                            f"[TaskManager] Loaded IK seeds from {len(joint_files)} "
                            f"file(s) ({len(joint_rows)} rows)"
                        )

                # Paired pose CSV(s) for (x, y, z) in joint mode Ra map
                pose_lookup = None
                if scan_mode == "joint":
                    pose_files = cfg.get("pose_files") or (
                        [cfg["pose_file"]] if cfg.get("pose_file") else []
                    )
                    if pose_files:
                        pose_lookup = {}
                        for pf in pose_files:
                            ppath = os.path.join(self.task_dir, pf)
                            if os.path.isfile(ppath):
                                for r in self._read_csv(ppath):
                                    key = (_as_int(r["group_id"]),
                                           _work_point_id(r))
                                    pose_lookup[key] = (
                                        float(r["x"]), float(r["y"]), float(r["z"])
                                    )
                        rospy.loginfo(
                            f"[TaskManager] Loaded world coords from {len(pose_files)} "
                            f"pose file(s) ({len(pose_lookup)} keys)"
                        )

                self._build_scan_task(
                    task_name,
                    all_rows,
                    scan_mode=scan_mode,
                    joint_rows=joint_rows,
                    csv_path=result_csv_path,
                    pose_lookup=pose_lookup,
                )

            elif task_type == "move":
                self._build_move_only_task(task_name, all_rows)

            else:
                rospy.logerr(
                    f"[TaskManager] Unknown task type '{task_type}' "
                    f"for task '{task_name}'"
                )

            self.lift_heights[task_name] = lift_mm

            rospy.loginfo(
                f"[TaskManager] Task '{task_name}' loaded "
                f"(steps={len(self.tasks.get(task_name, []))}, "
                f"lift_height={'none' if lift_mm is None else f'{lift_mm} mm'})"
            )

    # ==================================================
    # LIFT HEIGHT
    # ==================================================
    def _extract_lift_height(self, task_name, rows):
        """Read the optional `lift_height` column [mm]. Returns (ok, value).

        The lift is set once per task and held until it finishes, so ONE value
        has to cover every row — the joint angles in a scan CSV were solved at
        a specific base height, and running them at a different one drives the
        arm somewhere else entirely. A file that disagrees with itself is
        therefore rejected outright rather than resolved by picking a winner:
        there is no safe way to guess which rows are the wrong ones.

        (ok=False) means the task must not be registered at all. A CSV with no
        such column returns (True, None) — the lift is simply not commanded,
        which is how every pre-existing task keeps working.
        """
        # `lift_mm` is the RRT dialect's spelling of the same column. Without
        # this alias such a CSV loads as "no lift column" and the lift is
        # silently never commanded — harmless at 0 mm, wrong at any other.
        for r in rows:
            if 'lift_height' not in r and 'lift_mm' in r:
                r['lift_height'] = r['lift_mm']

        present = [r for r in rows if str(r.get('lift_height', '')).strip()]
        if not present:
            if any('lift_height' in r for r in rows):
                rospy.logerr(
                    f"[TaskManager] Task '{task_name}': lift_height column is "
                    "present but every cell is empty — refusing to load. "
                    "Remove the column or fill it in."
                )
                return False, None
            return True, None

        if len(present) != len(rows):
            rospy.logerr(
                f"[TaskManager] Task '{task_name}': lift_height is set on "
                f"{len(present)} of {len(rows)} rows. Refusing to load — a "
                "blank cell is not the same as 0 mm and guessing which is "
                "meant is not safe."
            )
            return False, None

        try:
            values = {float(r['lift_height']) for r in present}
        except ValueError as e:
            rospy.logerr(
                f"[TaskManager] Task '{task_name}': lift_height is not "
                f"numeric ({e}). Refusing to load."
            )
            return False, None

        if len(values) > 1:
            rospy.logerr(
                f"[TaskManager] Task '{task_name}': lift_height disagrees "
                f"across rows ({sorted(values)}). The lift is set once per "
                "task and held, so the CSV must name a single height. "
                "Refusing to load."
            )
            return False, None

        return True, values.pop()

    # ==================================================
    # SYSTEM TASKS (NO CSV)
    # ==================================================
    def _register_system_tasks(self):
        """
        Built-in tasks that do not rely on CSV
        """

        # ---- go home ----
        self.tasks["go_home"] = [
            {"tag": self.START_TAG, "scan": False}
        ]
        self.scan_points["go_home"] = {}
        self.lift_heights["go_home"] = None

        rospy.loginfo(
            f"[TaskManager] System task registered: go_home → tag {self.START_TAG}"
        )

    # ==================================================
    # DYNAMIC TASKS (RUNTIME)
    # ==================================================
    def build_goto_task(self, tag_id: int) -> List[dict]:
        """
        Build a move-only task at runtime:
            GOTO <tag_id>
        """
        rospy.loginfo(f"[TaskManager] Build dynamic GOTO task → tag {tag_id}")
        return [
            {"tag": int(tag_id), "scan": False}
        ]

    # ==================================================
    # BUILD TASKS
    # ==================================================
    def _build_scan_task(self, task_name, rows, scan_mode, joint_rows=None,
                         csv_path="", pose_lookup=None):
        """Turn CSV rows into task steps + per-tag scan points.

        Two CSV dialects are accepted, distinguished only by which optional
        columns are present (see `_is_scan_row` / `_work_point_id`):

        * the original one — every row is a work point, identified by
          `point_id`;
        * the RRT-planned one (2026-09-11) — rows are path waypoints,
          `is_task_waypoint` says which are work points and
          `source_point_id` carries the work-point index.
        """

        task_steps = []
        scan_points_by_tag = defaultdict(list)

        # sort for deterministic execution
        rows.sort(
            key=lambda r: (
                _as_int(r.get("order", 0)),
                _as_int(r.get("group_id", 0)),
                _as_int(r.get("point_id", 0)),
            )
        )

        # Build joint lookup: (group_id, work-point id) -> [q1..q6]
        #
        # ⚠️ The key is `source_point_id` when the CSV has it. An RRT-planned
        # path renumbers `point_id` along the whole path — transitions
        # included — and keeps the original work-point index in
        # `source_point_id`. Pairing on `point_id` instead matched only 77 %
        # of the 2026-09-11 files (797 of 1035); on `source_point_id` it is
        # 1035/1035. Files without the column are unaffected.
        joint_lookup = {}
        if joint_rows:
            for jr in joint_rows:
                if not _is_scan_row(jr):
                    continue          # transitions have no work-point identity
                joint_lookup[(_as_int(jr["group_id"]), _work_point_id(jr))] = [
                    float(jr["q1"]), float(jr["q2"]), float(jr["q3"]),
                    float(jr["q4"]), float(jr["q5"]), float(jr["q6"]),
                ]

        prev_gid = None

        for r in rows:
            gid = _as_int(r["group_id"])

            # ---- task step ----
            if gid != prev_gid:
                task_steps.append({
                    "tag": gid,
                    "scan": True
                })
                prev_gid = gid

            speed = float(r.get("speed", 80))

            # ---- metadata from CSV ----
            pid = _as_int(r.get("point_id", 0))
            is_disc = _as_int(r.get("is_discontinuous", 0))

            # ---- scan point ----
            if scan_mode == "joint":
                point = {
                    "mode": "joint",
                    "joints": [
                        float(r["q1"]), float(r["q2"]), float(r["q3"]),
                        float(r["q4"]), float(r["q5"]), float(r["q6"]),
                    ],
                    "speed": speed,
                    "point_id": pid,
                    "group_id": gid,
                    "csv_path": csv_path,
                    "is_discontinuous": is_disc,
                    # False = drive through it, do not scan (see _is_scan_row)
                    "scan": _is_scan_row(r),
                }
                # Attach world (x, y, z) from paired pose CSV — used for Ra map
                # output. Keyed on the WORK-POINT id, same reason as the joint
                # lookup above. Scan rows only: a transition has no
                # source_point_id, so the key would fall back to its path
                # `point_id` and collide with an unrelated work point.
                if pose_lookup is not None and point["scan"]:
                    xyz = pose_lookup.get((gid, _work_point_id(r)))
                    if xyz is not None:
                        point["x"], point["y"], point["z"] = xyz
                scan_points_by_tag[gid].append(point)

            elif scan_mode == "pose":
                point = {
                    "mode": "pose",
                    "x": float(r["x"]),
                    "y": float(r["y"]),
                    "z": float(r["z"]),
                    "rx": float(r["rx"]),   # rad
                    "ry": float(r["ry"]),
                    "rz": float(r["rz"]),
                    "speed": speed,
                    "point_id": pid,
                    "group_id": gid,
                    "csv_path": csv_path,
                    "is_discontinuous": is_disc,
                }
                # Attach IK seed from paired joint CSV
                q0 = joint_lookup.get((gid, pid))
                if q0 is not None:
                    point["q0"] = q0
                scan_points_by_tag[gid].append(point)

        self.tasks[task_name] = task_steps
        self.scan_points[task_name] = scan_points_by_tag

    def _build_move_only_task(self, task_name, rows):

        task_steps = []

        for r in rows:
            gid = int(r["group_id"])
            task_steps.append({
                "tag": gid,
                "scan": False
            })

        self.tasks[task_name] = task_steps
        self.scan_points[task_name] = {}

    # ==================================================
    # CSV READ
    # ==================================================
    def _read_csv(self, csv_path: str) -> List[dict]:

        rows = []
        try:
            with open(csv_path, newline='', encoding="utf-8-sig") as f:
                reader = csv.DictReader(f)
                for r in reader:
                    rows.append(r)
        except Exception as e:
            rospy.logerr(f"[TaskManager] Failed to read {csv_path}: {e}")
        return rows

    # ==================================================
    # PUBLIC API
    # ==================================================
    def get_task(self, task_name: str) -> List[dict]:
        return self.tasks.get(task_name, [])

    def get_scan_points(self, task_name: str, tag_id: int) -> List[dict]:
        return self.scan_points.get(task_name, {}).get(tag_id, [])

    def get_lift_height(self, task_name: str) -> Optional[float]:
        """Lift height [mm] the task runs at, or None to leave the lift alone."""
        return self.lift_heights.get(task_name)

    def get_all_task_names(self) -> List[str]:
        return list(self.tasks.keys())
