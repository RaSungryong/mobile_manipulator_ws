"""
session_log.py
==============
Append-only record of ONE map-calibration session.

    <save_dir>/calibrate/<YYYYMMDD_HHMMSS>/
        session.yaml            header (plan / ref / map / lift / dry_run)
                                + an ordered ``entries`` index, rewritten
                                atomically after every attempt
        entries/001_tag105_attempt1_fail.yaml
        entries/002_tag105_attempt2_ok.yaml     one file PER ATTEMPT, in
        entries/003_tag106_attempt1_ok.yaml     execution order
        entries_log.csv         append-only one-line-per-attempt index
        map_world.yaml          the world-frame result as of the last write

Nothing in here is ever overwritten by a later attempt or a later
session: the sequence number only grows, a retry gets its own file, and
each session gets its own timestamped directory. (The orchestrator's
``map_out_path`` is a separate, user-chosen file and keeps its old
semantics; the copy here is the one that is guaranteed to survive.)
"""
import csv
import datetime as _dt
import os
from pathlib import Path
from typing import Optional

from ..persistence import _plain
from .map_io import atomic_write


def _expand(p):
    return Path(os.path.expandvars(os.path.expanduser(str(p))))


def _now_iso():
    return _dt.datetime.now().isoformat(timespec="milliseconds")


CSV_FIELDS = ["seq", "recorded_at", "path_tag_id", "ref_tag_id", "attempt",
              "status", "x_m", "y_m", "z_m", "error", "file"]


class SessionRecorder:
    def __init__(self, root_dir, meta: Optional[dict] = None):
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = _expand(root_dir) / "calibrate"
        self.dir = base / stamp
        # Two sessions inside one second: never reuse a directory.
        n = 1
        while self.dir.exists():
            n += 1
            self.dir = base / f"{stamp}_{n}"
        self.entries_dir = self.dir / "entries"
        self.entries_dir.mkdir(parents=True, exist_ok=False)
        self.csv_path = self.dir / "entries_log.csv"
        self.seq = 0
        self.session = {
            "started_at": _now_iso(),
            "status": "running",
            "note": ("One file per attempt under entries/, numbered in "
                     "execution order; a retry is a new number, nothing "
                     "is overwritten. Camera-frame errors: see "
                     "camera_frame_note inside each entry file."),
        }
        self.session.update(_plain(meta or {}))
        self.session["entries"] = []
        self._write_session()
        with open(self.csv_path, "w", newline="") as fh:
            csv.DictWriter(fh, fieldnames=CSV_FIELDS).writeheader()

    # ------------------------------------------------------------------
    def _write_session(self):
        atomic_write(self.session, self.dir / "session.yaml")

    def record(self, payload: dict) -> int:
        """Persist one attempt. Returns its sequence number (1-based)."""
        self.seq += 1
        tag = int(payload.get("path_tag_id", -1))
        attempt = int(payload.get("attempt", 1))
        status = str(payload.get("status", "unknown"))
        name = f"{self.seq:03d}_tag{tag}_attempt{attempt}_{status}.yaml"
        path = self.entries_dir / name
        k = 1
        while path.exists():          # cannot happen (seq grows) — belt and braces
            k += 1
            path = self.entries_dir / f"{name[:-5]}_{k}.yaml"
        body = {"seq": self.seq, "recorded_at": _now_iso()}
        body.update(_plain(payload))
        atomic_write(body, path)

        world = (payload.get("result_world") or {}).get("position_m") or [None] * 3
        row = {
            "seq": self.seq, "recorded_at": body["recorded_at"],
            "path_tag_id": tag, "ref_tag_id": payload.get("ref_tag_id"),
            "attempt": attempt, "status": status,
            "x_m": world[0], "y_m": world[1], "z_m": world[2],
            "error": payload.get("error") or "", "file": path.name,
        }
        with open(self.csv_path, "a", newline="") as fh:
            csv.DictWriter(fh, fieldnames=CSV_FIELDS).writerow(row)

        self.session["entries"].append(_plain({
            "seq": self.seq, "path_tag_id": tag,
            "ref_tag_id": payload.get("ref_tag_id"), "attempt": attempt,
            "status": status, "position_m": world if world[0] is not None else None,
            "error": payload.get("error"), "file": f"entries/{path.name}",
        }))
        self._write_session()
        return self.seq

    def mark_cancelled(self, before_tag: int):
        self.session["cancelled_before_tag"] = int(before_tag)
        self._write_session()

    def write_map_copy(self, world_data: dict):
        atomic_write(world_data, self.dir / "map_world.yaml")

    def finish(self, num_succeeded: int, num_failed: int,
               output_yaml_path: str, world_data: Optional[dict] = None):
        self.session["status"] = "finished"
        self.session["finished_at"] = _now_iso()
        self.session["num_succeeded"] = int(num_succeeded)
        self.session["num_failed"] = int(num_failed)
        self.session["num_attempts"] = int(self.seq)
        self.session["map_out_path"] = str(output_yaml_path)
        if world_data is not None:
            self.write_map_copy(world_data)
        self._write_session()
