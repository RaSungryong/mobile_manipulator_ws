#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Incremental scan-result CSV persistence (13-column Ra map schema).

Split out of arm_controller.py — writing CSVs is not motion control.
Schema (see CLAUDE.md "Task Commands"):
    group_id, point_id, x, y, z,
    ra_mean, ra_std, ra_min, ra_max, num_samples,
    success, execution_message, validated_at

Usage: call begin(scan_points) once per scan to register the queued points
(seeds every row so partial results stay visible), then save(csv_path,
results) after each point — incremental, so a cancel mid-scan loses nothing.
"""

import os
from datetime import datetime

import pandas as pd

import rospy

COLUMNS = ['group_id', 'point_id', 'x', 'y', 'z',
           'ra_mean', 'ra_std', 'ra_min', 'ra_max', 'num_samples',
           'success', 'execution_message', 'validated_at']


class ScanResultWriter:

    def __init__(self):
        self._scan_meta = {}   # (group_id, point_id) → {x, y, z, csv_path}

    def begin(self, scan_points):
        """Register the queued points; used to seed the CSV on first write."""
        self._scan_meta = {}
        for i, p in enumerate(scan_points):
            key = (int(p.get("group_id", -1)), int(p.get("point_id", i)))
            self._scan_meta[key] = {
                "x":        float(p["x"]) if "x" in p else None,
                "y":        float(p["y"]) if "y" in p else None,
                "z":        float(p["z"]) if "z" in p else None,
                "csv_path": p.get("csv_path", ""),
            }

    def save(self, csv_path, results):
        """Persist per-point results to csv_path incrementally.

        On first call the file is seeded from the registered points so every
        queued scan point gets a row (with x/y/z populated). Subsequent calls
        update rows in place.
        """
        if not csv_path:
            return

        cols = COLUMNS

        if os.path.exists(csv_path):
            df = pd.read_csv(csv_path)
            for c in cols:
                if c not in df.columns:
                    df[c] = None
            # Add rows for new scan points (current group) not yet in the CSV
            existing_keys = set()
            for _, row in df[['group_id', 'point_id']].iterrows():
                try:
                    existing_keys.add((int(row['group_id']), int(row['point_id'])))
                except (TypeError, ValueError):
                    pass
            new_rows = [{
                'group_id': gid, 'point_id': pid,
                'x': m['x'], 'y': m['y'], 'z': m['z'],
                'ra_mean': None, 'ra_std': None, 'ra_min': None,
                'ra_max': None, 'num_samples': 0,
                'success': None, 'execution_message': None,
                'validated_at': None,
            } for (gid, pid), m in self._scan_meta.items()
              if (gid, pid) not in existing_keys]
            if new_rows:
                df = pd.concat([df, pd.DataFrame(new_rows, columns=cols)], ignore_index=True)
        else:
            # Seed with all queued points so partial results are visible
            rows = [{
                'group_id': gid, 'point_id': pid,
                'x': m['x'], 'y': m['y'], 'z': m['z'],
                'ra_mean': None, 'ra_std': None, 'ra_min': None,
                'ra_max': None, 'num_samples': 0,
                'success': None, 'execution_message': None,
                'validated_at': None,
            } for (gid, pid), m in self._scan_meta.items()]
            df = pd.DataFrame(rows, columns=cols)

        result_dict = {(r['group_id'], r['point_id']): r for r in results}
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        def _update(row):
            try:
                key = (int(row['group_id']), int(row['point_id']))
            except (TypeError, ValueError):
                return row
            if key in result_dict:
                r = result_dict[key]
                row['success']           = r['success']
                row['execution_message'] = r['execution_message']
                row['ra_mean']           = r['ra_mean']
                row['ra_std']            = r['ra_std']
                row['ra_min']            = r['ra_min']
                row['ra_max']            = r['ra_max']
                row['num_samples']       = r['num_samples']
                if r['success'] is not None and pd.isna(row.get('validated_at')):
                    row['validated_at'] = now
            return row

        df = df.apply(_update, axis=1)
        df.to_csv(csv_path, index=False)
        rospy.loginfo(f"[Arm REAL] Ra map saved → {csv_path}")
