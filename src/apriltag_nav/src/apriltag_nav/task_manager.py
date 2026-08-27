#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import rospy
from collections import defaultdict
from typing import Dict, List, Optional


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

        # ---------------- scan (joint) ----------------
        "scan_joints_line1": {
            "file": "optimized_joints_line1.csv",
            "type": "scan",
            "scan_mode": "joint",
        },

        "scan_joints_line2": {
            "file": "optimized_joints_line2.csv",
            "type": "scan",
            "scan_mode": "joint",
        },

        # `scan_joints_line1_lift` / `joints_line1_lift.csv` were retired
        # 2026-08-13: optimized_joints_line1.csv now carries the lift_height
        # column itself, so the two were the same task. The CSV outlived the
        # task def in this checkout and was deleted 2026-08-23, after
        # confirming it was byte-identical to optimized_joints_line1.csv
        # (328 rows, all 10 columns, zero value differences).

        # ---------------- scan (pose) ----------------
        "scan_grid_line1": {
            "file": "grid_path_line1.csv",
            "type": "scan",
            "scan_mode": "pose",
            "joint_file": "optimized_joints_line1.csv",
        },

        "scan_grid_line2": {
            "file": "grid_path_line2.csv",
            "type": "scan",
            "scan_mode": "pose",
            "joint_file": "optimized_joints_line2.csv",
        },



        # ---------------- scan (joint, new) ----------------
        # "scan_joints_line1_new": {
        #     "file": "optimized_joints_line1_new.csv",
        #     "type": "scan",
        #     "scan_mode": "joint",
        # },

        # ---------------- full scan (line1 + line2, joint) ----------------
        "scan_full_joints": {
            "files": ["optimized_joints_line1.csv", "optimized_joints_line2.csv"],
            "pose_files": ["grid_path_line1.csv", "grid_path_line2.csv"],
            "type": "scan",
            "scan_mode": "joint",
            "result_name": "scan_full_joints_ra_map.csv",
        },

        # ---------------- full scan (line1 + line2, pose) ----------------
        "scan_full_pose": {
            "files": ["grid_path_line1.csv", "grid_path_line2.csv"],
            "joint_files": ["optimized_joints_line1.csv", "optimized_joints_line2.csv"],
            "type": "scan",
            "scan_mode": "pose",
            "result_name": "scan_full_pose_ra_map.csv",
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
                                    key = (int(r["group_id"]), int(r["point_id"]))
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

        task_steps = []
        scan_points_by_tag = defaultdict(list)

        # sort for deterministic execution
        rows.sort(
            key=lambda r: (
                int(r.get("order", 0)),
                int(r.get("group_id", 0)),
                int(r.get("point_id", 0)),
            )
        )

        # Build joint lookup: (group_id, point_id) -> [q1..q6]
        joint_lookup = {}
        if joint_rows:
            for jr in joint_rows:
                key = (int(jr["group_id"]), int(jr["point_id"]))
                joint_lookup[key] = [
                    float(jr["q1"]), float(jr["q2"]), float(jr["q3"]),
                    float(jr["q4"]), float(jr["q5"]), float(jr["q6"]),
                ]

        prev_gid = None

        for r in rows:
            gid = int(r["group_id"])

            # ---- task step ----
            if gid != prev_gid:
                task_steps.append({
                    "tag": gid,
                    "scan": True
                })
                prev_gid = gid

            speed = float(r.get("speed", 80))

            # ---- metadata from CSV ----
            pid = int(r.get("point_id", 0))
            is_disc_val = str(r.get("is_discontinuous", "0")).strip()
            is_disc = int(is_disc_val) if is_disc_val else 0

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
                }
                # Attach world (x, y, z) from paired pose CSV — used for Ra map output
                if pose_lookup is not None:
                    xyz = pose_lookup.get((gid, pid))
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
