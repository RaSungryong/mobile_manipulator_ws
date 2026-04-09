#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import rospy
from collections import defaultdict
from typing import Dict, List


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

    START_TAG = 508   # Home tag

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

            csv_path = os.path.join(self.task_dir, cfg["file"])
            if not os.path.isfile(csv_path):
                rospy.logerr(f"[TaskManager] CSV not found: {csv_path}")
                continue

            rows = self._read_csv(csv_path)
            if not rows:
                rospy.logwarn(f"[TaskManager] Empty CSV: {csv_path}")
                continue

            task_type = cfg["type"]

            if task_type == "scan":
                # Load paired joint CSV for IK seed (pose mode only)
                joint_rows = None
                jf = cfg.get("joint_file")
                if jf and cfg.get("scan_mode") == "pose":
                    jpath = os.path.join(self.task_dir, jf)
                    if os.path.isfile(jpath):
                        joint_rows = self._read_csv(jpath)
                        rospy.loginfo(
                            f"[TaskManager] Loaded IK seed: {jf} "
                            f"({len(joint_rows)} rows)"
                        )

                self._build_scan_task(
                    task_name,
                    rows,
                    scan_mode=cfg["scan_mode"],
                    joint_rows=joint_rows,
                )

            elif task_type == "move":
                self._build_move_only_task(task_name, rows)

            else:
                rospy.logerr(
                    f"[TaskManager] Unknown task type '{task_type}' "
                    f"for task '{task_name}'"
                )

            rospy.loginfo(
                f"[TaskManager] Task '{task_name}' loaded "
                f"(steps={len(self.tasks.get(task_name, []))})"
            )

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
    def _build_scan_task(self, task_name, rows, scan_mode, joint_rows=None):

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

            # ---- scan point ----
            if scan_mode == "joint":
                scan_points_by_tag[gid].append({
                    "mode": "joint",
                    "joints": [
                        float(r["q1"]), float(r["q2"]), float(r["q3"]),
                        float(r["q4"]), float(r["q5"]), float(r["q6"]),
                    ],
                    "speed": speed
                })

            elif scan_mode == "pose":
                point = {
                    "mode": "pose",
                    "x": float(r["x"]),
                    "y": float(r["y"]),
                    "z": float(r["z"]),
                    "rx": float(r["rx"]),   # rad
                    "ry": float(r["ry"]),
                    "rz": float(r["rz"]),
                    "speed": speed
                }
                # Attach IK seed from paired joint CSV
                pid = int(r.get("point_id", 0))
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

    def get_all_task_names(self) -> List[str]:
        return list(self.tasks.keys())
