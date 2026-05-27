"""
plan_io.py
==========
Yaml schema loaders for the map-calibration workflow.

Two files:
1. ``reference_tags.yaml`` — multiple basis tags with accurate world
   poses provided by the user.
2. ``calibration_plan.yaml`` — ordered list of path_tag -> ref_tag
   assignments, plus per-entry overrides.
"""
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml

from ..geometry import assert_rigid, rpy_deg_to_R


# ---------------------------------------------------------------------------
@dataclass
class RefTag:
    id: int
    T_world: np.ndarray            # 4x4 pose of ref tag in world
    size_m: Optional[float] = None # None = use locator.yaml default


@dataclass
class PlanEntry:
    path_tag_id: int
    ref_tag_id: int
    arm_view_tcp_mm_deg: Optional[List[float]] = None
    nav_start_id: Optional[int] = None
    retry_count: Optional[int] = None
    align_required: Optional[bool] = None


@dataclass
class CalibrationPlan:
    defaults: dict
    entries: List[PlanEntry] = field(default_factory=list)

    def resolved(self, entry: PlanEntry) -> dict:
        """Merge ``defaults`` with per-entry overrides; per-entry wins."""
        out = dict(self.defaults)
        if entry.arm_view_tcp_mm_deg is not None:
            out["arm_view_tcp_mm_deg"] = entry.arm_view_tcp_mm_deg
        if entry.retry_count is not None:
            out["retry_count"] = int(entry.retry_count)
        if entry.align_required is not None:
            out["align_required"] = bool(entry.align_required)
        if entry.nav_start_id is not None:
            out["nav_start_id"] = int(entry.nav_start_id)
        return out


# ---------------------------------------------------------------------------
def _expand(p):
    return os.path.expandvars(os.path.expanduser(str(p)))


def load_reference_tags(yaml_path) -> Dict[int, RefTag]:
    """Parse reference_tags.yaml -> {tag_id: RefTag}.

    Schema::

        format: "pose" | "matrix"   (optional, default "pose")
        reference_tags:
          - id: 100
            position_m: [x, y, z]
            rpy_deg:    [rx, ry, rz]
            size_m: 0.060   # optional
          # or
          - id: 200
            matrix_4x4: [r00, r01, ..., 1]
    """
    with open(_expand(yaml_path), "r") as fh:
        data = yaml.safe_load(fh) or {}
    default_format = str(data.get("format", "pose"))
    entries = data.get("reference_tags") or []
    out: Dict[int, RefTag] = {}
    for raw in entries:
        tag_id = int(raw["id"])
        fmt = str(raw.get("format", default_format))
        if fmt == "matrix":
            T = np.asarray(raw["matrix_4x4"], dtype=np.float64).reshape(4, 4)
        else:
            pos = np.asarray(raw["position_m"], dtype=np.float64)
            rpy = raw["rpy_deg"]
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = rpy_deg_to_R(float(rpy[0]), float(rpy[1]), float(rpy[2]))
            T[:3, 3] = pos
        assert_rigid(T, name=f"reference_tags[{tag_id}]")
        size = raw.get("size_m")
        out[tag_id] = RefTag(
            id=tag_id, T_world=T,
            size_m=float(size) if size is not None else None,
        )
    if not out:
        raise ValueError(
            f"{yaml_path}: reference_tags list is empty")
    return out


def load_calibration_plan(yaml_path) -> CalibrationPlan:
    """Parse calibration_plan.yaml -> :class:`CalibrationPlan`.

    Schema::

        defaults:
          retry_count: 1
          align_required: true
          arm_view_tcp_mm_deg: [mm, mm, mm, deg, deg, deg]
        plan:
          - path_tag_id: 101
            ref_tag_id: 100
            # optional per-entry overrides:
            arm_view_tcp_mm_deg: [...]
            nav_start_id: 508
            retry_count: 2
            align_required: false
    """
    with open(_expand(yaml_path), "r") as fh:
        data = yaml.safe_load(fh) or {}

    defaults = dict(data.get("defaults") or {})
    defaults.setdefault("retry_count", 1)
    defaults.setdefault("align_required", True)
    if "arm_view_tcp_mm_deg" in defaults:
        defaults["arm_view_tcp_mm_deg"] = [
            float(v) for v in defaults["arm_view_tcp_mm_deg"]]

    raw_entries = data.get("plan") or []
    entries: List[PlanEntry] = []
    for i, raw in enumerate(raw_entries):
        if "path_tag_id" not in raw or "ref_tag_id" not in raw:
            raise ValueError(
                f"{yaml_path}: plan entry {i} missing "
                f"'path_tag_id' or 'ref_tag_id'")
        atcp = raw.get("arm_view_tcp_mm_deg")
        if atcp is not None:
            atcp = [float(v) for v in atcp]
            if len(atcp) != 6:
                raise ValueError(
                    f"{yaml_path}: plan entry {i} arm_view_tcp_mm_deg "
                    f"must have 6 floats, got {len(atcp)}")
        entries.append(PlanEntry(
            path_tag_id=int(raw["path_tag_id"]),
            ref_tag_id=int(raw["ref_tag_id"]),
            arm_view_tcp_mm_deg=atcp,
            nav_start_id=(int(raw["nav_start_id"])
                          if raw.get("nav_start_id") is not None else None),
            retry_count=(int(raw["retry_count"])
                         if raw.get("retry_count") is not None else None),
            align_required=(bool(raw["align_required"])
                            if raw.get("align_required") is not None else None),
        ))
    if not entries:
        raise ValueError(f"{yaml_path}: plan list is empty")
    return CalibrationPlan(defaults=defaults, entries=entries)
