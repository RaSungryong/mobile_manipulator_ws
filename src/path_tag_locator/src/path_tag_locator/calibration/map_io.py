"""
map_io.py
=========
Two-frame I/O:

- :func:`load_map` reads the input ``map.yaml`` (used by ``apriltag_nav``).
  Its (x, y) live in the Manipulator/Map frame; the file is opened
  read-only — the orchestrator uses it to drive the base via Pure
  Pursuit + BFS but never mutates it.

- :func:`save_map_world` writes a SEPARATE ``map_world.yaml`` whose
  coordinates are in the **world frame** (the same frame the user
  declared in ``reference_tags.yaml``). The schema is intentionally
  different from ``map.yaml`` so the output is NOT a drop-in for
  apriltag_nav — that mismatch would silently break navigation, since
  apriltag_nav expects Manipulator-frame coordinates.

Why the split: the world frame the user picks (commonly with one of
their ref tags as the origin) is unrelated to map.yaml's frame, and a
robust transform between the two cannot be measured by this package.
Mixing them in a single file would invite future-someone to use the
wrong one.
"""
import datetime as _dt
import copy
import os
import tempfile
from pathlib import Path
from typing import Optional

import yaml


def _expand(p):
    return Path(os.path.expandvars(os.path.expanduser(str(p))))


def load_map(yaml_path) -> dict:
    """Read map.yaml into a plain dict (deep enough for safe mutation)."""
    p = _expand(yaml_path)
    with open(p, "r") as fh:
        data = yaml.safe_load(fh) or {}
    if "tags" not in data:
        raise ValueError(f"{p}: map yaml has no 'tags' key")
    # Deep-copy so callers can mutate freely.
    return copy.deepcopy(data)


def make_world_map(origin_tag_id: Optional[int] = None,
                   reference_tags_path: Optional[str] = None,
                   calibration_plan_path: Optional[str] = None,
                   map_in_path: Optional[str] = None) -> dict:
    """Construct the empty container for a world-frame output file."""
    return {
        "frame": "world",
        "origin_tag_id": int(origin_tag_id) if origin_tag_id is not None else None,
        "generated_at": _dt.datetime.now().isoformat(timespec="seconds"),
        "reference_tags_path": reference_tags_path,
        "calibration_plan_path": calibration_plan_path,
        "map_in_path": map_in_path,
        "note": (
            "Coordinates here are in the world frame defined by "
            "reference_tags.yaml. They are NOT in the same frame as "
            "the input map.yaml (apriltag_nav's Manipulator frame) and "
            "must NOT be used as a drop-in replacement for it."
        ),
        "tags": {},
    }


def upsert_world_tag(world_data: dict,
                     tag_id: int,
                     position_m,
                     rpy_deg,
                     ref_tag_id: int,
                     map_xy=None,
                     tag_type: Optional[str] = None,
                     zone: Optional[str] = None,
                     name: Optional[str] = None) -> None:
    """Add / overwrite one calibrated tag in the world-frame output."""
    entry = {
        "position_m": [float(v) for v in position_m],
        "rpy_deg":    [float(v) for v in rpy_deg],
        "ref_tag_id": int(ref_tag_id),
    }
    if map_xy is not None:
        entry["map_xy"] = [float(map_xy[0]), float(map_xy[1])]
    if tag_type is not None:
        entry["type"] = str(tag_type)
    if zone is not None:
        entry["zone"] = str(zone)
    if name is not None:
        entry["name"] = str(name)
    world_data.setdefault("tags", {})[int(tag_id)] = entry


def atomic_write(map_data: dict, out_path) -> Path:
    """Write a yaml dict to ``out_path`` atomically via tmp + os.replace.

    Used for both ``save_map_world`` and any other yaml the orchestrator
    persists (echo, summary).

    Returns the resolved output path.
    """
    p = _expand(out_path)
    p.parent.mkdir(parents=True, exist_ok=True)
    # NamedTemporaryFile in the same dir so os.replace is atomic on
    # POSIX (cross-device renames would not be).
    fd, tmp_path = tempfile.mkstemp(
        prefix=p.name + ".",
        suffix=".tmp",
        dir=str(p.parent),
    )
    try:
        with os.fdopen(fd, "w") as fh:
            yaml.safe_dump(map_data, fh, default_flow_style=False,
                           sort_keys=False, allow_unicode=True)
        os.replace(tmp_path, str(p))
    except Exception:
        try:
            os.remove(tmp_path)
        except OSError:
            pass
        raise
    return p
