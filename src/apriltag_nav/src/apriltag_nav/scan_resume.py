#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Resume an interrupted scan run — the remaining points of ITS OWN result
files (2026-10-06, user: "중간에 끊겨도 이어서 하는 기능").

A TASK that stops half-way (STOP, e-stop, link drop, navigation error)
leaves everything it finished on disk, per point: the Ra map row
(`log/apriltag_nav/ra_maps/<run>.csv`, `success` + `validated_at`), the
frames (`results/scan_images/<run>/g<g>_p<p>_i<i>_s<n>.png`) and — in
COLLECT mode — the hand-measured row (`<run>/<run>_ra_measured.csv`).
Re-sending the TASK would start a NEW run and redo every point. `RESUME
[<run>]` on /task_command instead builds the same task with the finished
points taken out and the run's own csv_path kept, so the arm scans only
what is left and every writer appends to the same files:

  * `ScanResultWriter.save` updates an existing Ra map in place and adds
    rows only for keys it has not seen — the finished rows stay as they are;
  * the frame folder and the collect CSV are named from the csv_path stem,
    so they are the run's own;
  * the frame INDEX is the point's position in its group's list, which is
    kept by leaving the finished points IN the list (joint mode) rather
    than removing them.

What counts as DONE, per (group_id, point_id):
  * the Ra map row has `success` true and a `ra_mean`; AND
  * when the run has a `_ra_measured.csv` (a COLLECT-mode run), the point
    has a row there — recorded or skipped by the operator, OR a PENDING
    mark row (method B writes `mark_no` + frames at the marking stop with
    the Ra blank; user rule 2026-10-07: the photo and the number written
    on the case are KEPT, the point is not re-shot). A method-A point whose
    frame was taken but whose pause was cancelled before the operator
    answered (the 2026-10-06 20:07 e-stop: g106 p49) has no row, is NOT
    done and is scanned again, so the operator gets its stop back.
  A failed point (`success` false) is not done and is retried.
  `pending_points` lists the pending mark rows: their Ra is still owed, so
  a group whose points are all done but some pending is NOT dropped — the
  arm drives its path through (nothing captured) and `arm_controller`
  runs the group's entry stop for them at the end (`n_pending`).

What the plan does with a task step (one per group tag):
  * every scanned point of the group done, none pending -> the step is
    DROPPED (no drive); with pending points it is kept, all `scan: False`
    (pose mode keeps the last done point as the one move, for the run stem);
  * some left, JOINT mode                  -> the step stays; done work points
    become `scan: False` — the arm drives THROUGH them (the planned path
    stays intact and the frame index stays the same) and does not settle,
    capture or pause there;
  * some left, POSE mode                   -> the done points are removed
    (pose rows are independent targets);
  * a step without a scan (move-only)      -> kept as is.
Every remaining point's `csv_path` is the run's Ra map path.

GROUP SELECTION (2026-10-07, user: "그룹 id까지 선택해서 수집"): both
`TASK <name>` and `RESUME [<run>]` take a trailing `groups=104,105` token
(`parse_groups_arg`). A task step whose tag is not listed is DROPPED from
the plan (not driven to, nothing scanned) — the group is SKIPPED, not
done, so a later RESUME of the same run offers it again. A group id the
task does not have is an error (the command is refused, nothing moves).
In the plan `groups_filter` is the selection and `groups_skipped` the
dropped unfinished groups; `n_remaining` counts only the selected groups.

Pure python, no ROS: `task_executor` calls `plan_resume`; the check
script `tools/check_scan_resume.py` drives it offline.
"""
import csv
import glob
import os
import re
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

Key = Tuple[int, int]

_RUN_RE = re.compile(r'^(?P<task>.+)_ra_map_(?P<ts>\d{8}_\d{6})$')


def task_of(run_stem: str) -> Optional[str]:
    """`scan_joint_x_ra_map_20261006_175616` -> `scan_joint_x`; None if the
    stem is not a run name."""
    m = _RUN_RE.match(run_stem)
    return m['task'] if m else None


def run_stem_of(path: str) -> str:
    return os.path.splitext(os.path.basename(path))[0]


def _truthy(v) -> bool:
    return str(v).strip().lower() in ('1', 'true', 'yes', 'y')


def _as_key(row: dict) -> Optional[Key]:
    try:
        return (int(float(row['group_id'])), int(float(row['point_id'])))
    except (KeyError, TypeError, ValueError):
        return None


def _read_rows(path: str) -> List[dict]:
    with open(path, newline='', encoding='utf-8-sig') as f:
        return list(csv.DictReader(f))


def done_points(ra_map_path: str, measured_path: Optional[str] = None) -> Set[Key]:
    """The (group_id, point_id) keys the run has finished — see the module
    docstring for the rule. A missing Ra map means nothing is done."""
    if not ra_map_path or not os.path.isfile(ra_map_path):
        return set()
    done: Set[Key] = set()
    for r in _read_rows(ra_map_path):
        k = _as_key(r)
        if k is None or not _truthy(r.get('success')):
            continue
        ra = str(r.get('ra_mean', '')).strip()
        if ra == '' or ra.lower() == 'nan':
            continue
        done.add(k)
    if measured_path and os.path.isfile(measured_path):
        answered = {k for k in (_as_key(r) for r in _read_rows(measured_path)) if k}
        done &= answered
    return done


def _row_pending(r: dict) -> bool:
    ra = str(r.get('ra_measured', '') or '').strip()
    return ra == '' and not _truthy(r.get('skipped'))


def pending_points(measured_path: Optional[str]) -> Set[Key]:
    """The (group_id, point_id) keys of the run's PENDING mark rows: in the
    measured CSV with a blank `ra_measured` and not skipped — captured and
    numbered (method B), Ra not yet entered. Empty without the file."""
    if not measured_path or not os.path.isfile(measured_path):
        return set()
    return {k for k in (_as_key(r) for r in _read_rows(measured_path) if _row_pending(r)) if k}


def parse_groups_arg(parts: List[str]) -> Tuple[List[str], Optional[Set[int]], Optional[str]]:
    """Split a `groups=104,105` token (any position, case-insensitive key,
    `,` / `+` / `;` separated, spaces allowed after the `=` when the caller
    joined them) out of a command's words. Returns (other words, the set
    or None when absent, an error message or None). An empty or malformed
    list is an error, not "no filter"."""
    rest: List[str] = []
    groups: Optional[Set[int]] = None
    i = 0
    while i < len(parts):
        w = parts[i]
        low = w.lower()
        if low.startswith('groups=') or low.startswith('group=') or low.startswith('g='):
            text = w.split('=', 1)[1]
            # `groups= 104,105` — the list in the next word
            if text == '' and i + 1 < len(parts):
                i += 1
                text = parts[i]
            ids: Set[int] = set()
            for tok in re.split(r'[,;+\s]+', text.strip()):
                if tok == '':
                    continue
                try:
                    ids.add(int(tok))
                except ValueError:
                    return rest, None, f"bad group id '{tok}' in '{w}' (want groups=104,105)"
            if not ids:
                return rest, None, f"'{w}' lists no group (want groups=104,105)"
            groups = ids if groups is None else groups | ids
        else:
            rest.append(w)
        i += 1
    return rest, groups, None


def task_groups(task_steps: Iterable[dict]) -> List[int]:
    """The group ids (= scan-step tags) of a registered task, in order."""
    return [int(st['tag']) for st in task_steps if 'tag' in st and st.get('scan', False)]


def check_groups(task_steps: Iterable[dict], groups: Optional[Set[int]]) -> Optional[str]:
    """None when every selected group is one of the task's scan groups, else
    the refusal text."""
    if not groups:
        return None
    have = task_groups(task_steps)
    unknown = sorted(g for g in groups if g not in have)
    if unknown:
        return f"groups {unknown} are not in this task (its groups: {have})"
    return None


def filter_steps(task_steps: Iterable[dict], groups: Optional[Set[int]]) -> List[dict]:
    """A fresh TASK with a group selection: the scan steps of other groups
    dropped, move-only / service items kept. Copies the steps."""
    out: List[dict] = []
    for st in task_steps:
        if groups and 'tag' in st and st.get('scan', False) and int(st['tag']) not in groups:
            continue
        out.append(dict(st))
    return out


class ResumePlan:
    """What `plan_resume` worked out for one run."""

    def __init__(self, run_stem: str, task_name: str, csv_path: str):
        self.run_stem = run_stem
        self.task_name = task_name
        self.csv_path = csv_path
        self.steps: List[dict] = []
        self.points_by_tag: Dict[int, List[dict]] = {}
        self.n_done = 0              # scanned points already in the files
        self.n_remaining = 0         # scanned points the resume will do
        self.n_total = 0             # scanned points of the whole task
        self.n_pending = 0           # captured + numbered, Ra entry still owed (mark pass)
        self.groups_done: List[int] = []
        self.groups_kept: List[int] = []
        self.groups_filter: Optional[List[int]] = None   # the selection, sorted
        self.groups_skipped: List[int] = []              # unfinished, not selected

    @property
    def nothing_left(self) -> bool:
        return self.n_remaining == 0 and self.n_pending == 0

    def summary(self) -> str:
        text = (f"{self.run_stem}: {self.n_done} of {self.n_total} points done, "
                f"{self.n_remaining} left; groups done {self.groups_done}, "
                f"to scan {self.groups_kept}")
        if self.n_pending:
            text += f"; {self.n_pending} captured point(s) await their Ra entry"
        if self.groups_filter is not None:
            text += f"; selected {self.groups_filter}, skipped {self.groups_skipped}"
        return text


def plan_resume(run_stem: str, task_name: str, task_steps: Iterable[dict],
                points_by_tag: Dict[int, List[dict]], done: Set[Key],
                csv_path: str, groups: Optional[Set[int]] = None,
                pending: Optional[Set[Key]] = None) -> ResumePlan:
    """Build the resumed task. Inputs are TaskManager's registered steps /
    points (never modified — every point is copied), the done set and the
    pending set (mark rows whose Ra is still owed; a subset of done).
    `groups` (optional) keeps only those group tags — see the docstring."""
    plan = ResumePlan(run_stem, task_name, csv_path)
    pending = set(pending or ())
    if groups:
        plan.groups_filter = sorted(int(g) for g in groups)
    for step in task_steps:
        if 'tag' not in step or not step.get('scan', False):
            plan.steps.append(dict(step))
            continue
        tag = int(step['tag'])
        pts = points_by_tag.get(tag, [])
        scan_keys = [(int(p.get('group_id', tag)), int(p.get('point_id', i)))
                     for i, p in enumerate(pts) if p.get('scan', True)]
        n_done = sum(1 for k in scan_keys if k in done)
        n_pend = sum(1 for k in scan_keys if k in done and k in pending)
        plan.n_total += len(scan_keys)
        plan.n_done += n_done
        if scan_keys and n_done == len(scan_keys) and n_pend == 0:
            plan.groups_done.append(tag)
            continue
        if groups and tag not in groups:
            plan.groups_skipped.append(tag)   # unfinished, but not selected
            continue
        kept: List[dict] = []
        last_done: Optional[dict] = None
        for i, p in enumerate(pts):
            q = dict(p)
            q['csv_path'] = csv_path
            key = (int(p.get('group_id', tag)), int(p.get('point_id', i)))
            if p.get('scan', True) and key in done:
                q['scan'] = False            # on the path: drive through it
                if q.get('mode') == 'pose':
                    last_done = q            # an independent target: skip it
                    continue
            kept.append(q)
        if not kept and last_done is not None:
            # pose mode, everything captured, Ra entries owed: one move to
            # the last done point (not scanned) carries the run's csv_path
            # to the arm, whose end-of-list entry stop lists the pending rows
            kept.append(last_done)
        plan.points_by_tag[tag] = kept
        plan.steps.append(dict(step))
        plan.groups_kept.append(tag)
        plan.n_remaining += len(scan_keys) - n_done
        plan.n_pending += n_pend
    return plan


def list_runs(ra_map_dir: str) -> List[str]:
    """Run stems under ra_map_dir, newest first (the name carries the stamp)."""
    stems = [run_stem_of(p) for p in glob.glob(os.path.join(ra_map_dir, '*_ra_map_*.csv'))]
    return sorted((s for s in stems if task_of(s)), reverse=True)


def find_resumable(ra_map_dir: str, make_plan: Callable[[str], Optional[ResumePlan]],
                   limit: int = 20) -> Optional[ResumePlan]:
    """The newest run with points left, among the `limit` newest Ra maps.
    `make_plan(stem)` returns a ResumePlan or None (task not registered)."""
    for stem in list_runs(ra_map_dir)[:limit]:
        plan = make_plan(stem)
        if plan is not None and not plan.nothing_left:
            return plan
    return None
