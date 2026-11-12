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

START POINT (2026-11-12): `TASK <name> start=<group>:<source_point_id>`
starts a FRESH run at that work point — `start_done_points` marks every
scanned point before it as done and the same `plan_resume` does the rest
(earlier groups not driven to; pose mode: earlier points dropped). In a
JOINT path the start group then goes STRAIGHT there (`direct_start`): its
home row, a joint-linear approach home -> start point, the start point and
the rest — the rows before the start point are not driven. `RESUME … start=g:p` adds
the same set to the run's own done set, so a run that was started at a
point is not offered the points before it again (a RESUME still drives
through, it does not go straight).

Pure python, no ROS: `task_executor` calls `plan_resume`; the check
script `tools/check_scan_resume.py` drives it offline.
"""
import csv
import glob
import math
import os
import re
from typing import Callable, Dict, Iterable, List, Optional, Set, Tuple

Key = Tuple[int, int]

# A run stem is `<task>_ra_map_<stamp>` (task_executor's own stamp is
# `YYYYMMDD_HHMMSS`), or — since 2026-10-13, `TASK <name> run=<stem>` — any
# `<task>[_<label>]_ra_map_<text>` the operator chose (e.g.
# `..._offset50mm_roughing_ra_map_20261013-1014`): the stem names the Ra map,
# the frame folder and the collect CSV, so one task can be scanned into
# separate result sets (roughing / finishing mould). The split is at the LAST
# `_ra_map_`; the head is the task name, possibly followed by a label.
_RUN_RE = re.compile(r'^(?P<head>.+)_ra_map_(?P<ts>[^/\s]+)$')
_RUN_STEM_CHARS = re.compile(r'^[A-Za-z0-9._-]+$')


def run_head_of(run_stem: str) -> Optional[str]:
    """Everything before the last `_ra_map_`, or None if the stem is not a
    run name. `scan_joint_x_roughing_ra_map_20261013-1014` -> `scan_joint_x_roughing`."""
    m = _RUN_RE.match(run_stem)
    return m['head'] if m else None


def task_of(run_stem: str, tasks: Optional[Iterable[str]] = None) -> Optional[str]:
    """The task a run stem belongs to; None if the stem is not a run name.

    `scan_joint_x_ra_map_20261006_175616` -> `scan_joint_x`. With `tasks`
    (the registered task names) a LABELLED stem resolves too: the longest
    registered name that is the head itself or a `_`-separated prefix of it
    (`scan_joint_x_roughing_ra_map_20261013-1014` -> `scan_joint_x`), None
    when no registered task matches. Without `tasks` the whole head is
    returned (the pre-label convention, which every stem the executor
    stamped itself still follows)."""
    head = run_head_of(run_stem)
    if head is None:
        return None
    if tasks is None:
        return head
    best = None
    for t in tasks:
        if head == t or head.startswith(t + '_'):
            if best is None or len(t) > len(best):
                best = t
    return best


def parse_run_arg(parts: List[str]) -> Tuple[List[str], Optional[str], Optional[str]]:
    """Split a `run=<stem>` token (any position, case-insensitive key) out of
    a command's words — the operator's own name for the result set of a
    fresh TASK. Returns (other words, the stem or None, an error or None).
    The stem may carry `.csv`, which is dropped; it must be one word of
    `[A-Za-z0-9._-]` containing `_ra_map_` (what every reader splits on)."""
    rest: List[str] = []
    stem: Optional[str] = None
    i = 0
    while i < len(parts):
        w = parts[i]
        low = w.lower()
        if low.startswith('run=') or low.startswith('run_name=') or low.startswith('name='):
            text = w.split('=', 1)[1].strip()
            if text == '' and i + 1 < len(parts):
                i += 1
                text = parts[i].strip()
            if text.lower().endswith('.csv'):
                text = text[:-4]
            if text == '':
                return rest, None, f"'{w}' names no run (want run=<task>[_<label>]_ra_map_<stamp>)"
            if not _RUN_STEM_CHARS.match(text):
                return rest, None, (f"run name '{text}' may only use letters, digits, '.', '_', '-' "
                                    "(no spaces, no '/')")
            if run_head_of(text) is None:
                return rest, None, f"run name '{text}' must contain '_ra_map_' (<task>[_<label>]_ra_map_<stamp>)"
            if stem is not None and stem != text:
                return rest, None, f"two run names given ('{stem}', '{text}')"
            stem = text
        else:
            rest.append(w)
        i += 1
    return rest, stem, None


def check_run_stem(stem: str, task_name: str, tasks: Iterable[str]) -> Optional[str]:
    """Why `stem` cannot be the run name of a fresh `task_name` TASK, or
    None when it can: it must resolve (task_of with the registered names)
    to exactly that task — i.e. `<task_name>_ra_map_…` or
    `<task_name>_<label>_ra_map_…`, and not to a longer registered name."""
    head = run_head_of(stem)
    if head is None:
        return f"run name '{stem}' must contain '_ra_map_'"
    if not (head == task_name or head.startswith(task_name + '_')):
        return (f"run name '{stem}' does not start with the task name — want "
                f"'{task_name}_ra_map_<stamp>' or '{task_name}_<label>_ra_map_<stamp>'")
    owner = task_of(stem, tasks)
    if owner != task_name:
        return (f"run name '{stem}' would belong to task '{owner}' (a longer registered name) "
                f"— a later RESUME would run that task's path; choose another label")
    return None


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


def parse_start_arg(parts: List[str]) -> Tuple[List[str], Optional[Tuple[int, int]], Optional[str]]:
    """Split a `start=<group>:<source_point_id>` token (2026-11-12, user:
    "어느 point부터 시작하는 기능") out of a command's words. `:` `/` `,`
    separate the two numbers. Returns (other words, (group, point) or None,
    error or None)."""
    rest: List[str] = []
    start: Optional[Tuple[int, int]] = None
    for w in parts:
        if w.lower().startswith('start='):
            text = w.split('=', 1)[1].strip()
            m = re.match(r'^(\d+)\s*[:/,]\s*(\d+)$', text)
            if not m:
                return rest, None, f"bad '{w}' (want start=<group>:<source_point_id>, e.g. start=106:651)"
            start = (int(m.group(1)), int(m.group(2)))
        else:
            rest.append(w)
    return rest, start, None


def _work_id(p: dict, i: int) -> Optional[int]:
    """The work-point number of a scan point (joint: source_point_id; pose:
    point_id, which is the work point)."""
    v = p.get('source_point_id')
    if v is None and p.get('mode') == 'pose':
        v = p.get('point_id', i)
    try:
        return None if v is None else int(v)
    except (TypeError, ValueError):
        return None


def start_done_points(task_steps: Iterable[dict], points_by_tag: Dict[int, List[dict]],
                      start: Tuple[int, int]) -> Tuple[Set[Key], Optional[str]]:
    """The scanned points that come BEFORE `start` = (group, source_point_id)
    in the task's order, as `plan_resume` done keys: every scanned point of
    the groups before the start group, and those of the start group listed
    before the start point. `plan_resume` with this set then skips the
    earlier groups and, in a joint path, drives THROUGH the earlier points of
    the start group (scan False, planned route intact). (set, error)."""
    g0, sp0 = int(start[0]), int(start[1])
    groups = task_groups(task_steps)
    if g0 not in groups:
        return set(), f"start group {g0} is not in this task (its groups: {groups})"
    done: Set[Key] = set()
    for tag in groups:
        pts = points_by_tag.get(tag, [])
        keys = [((int(p.get('group_id', tag)), int(p.get('point_id', i))), _work_id(p, i))
                for i, p in enumerate(pts) if p.get('scan', True)]
        if tag != g0:
            done.update(k for k, _ in keys)
            continue
        ids = [w for _, w in keys]
        if sp0 not in ids:
            lo = min((w for w in ids if w is not None), default=None)
            hi = max((w for w in ids if w is not None), default=None)
            return set(), (f"group {g0} has no scanned work point {sp0} "
                           f"(its source_point_id run {lo}..{hi})")
        for k, w in keys:
            if w == sp0:
                break
            done.add(k)
        return done, None
    return done, None


def direct_start(points: List[dict], start_sp: int, step_rad: float = 0.1
                 ) -> Tuple[List[dict], int, int]:
    """A JOINT group list that goes STRAIGHT to the start point (2026-11-12,
    user: "앞에 있는 경로 무시하고 직접 가는 추가 경로"): the group's first row
    (its home row) is kept, every row between it and the start point is
    dropped, and a joint-linear approach home -> start point is inserted,
    cut into `step_rad` pieces (speed 10, not scanned) — the same shape the
    planner gives every approach from home. The start point and everything
    after it are unchanged (the start point's speed set to 10, it is now the
    end of an approach). Returns (points, rows dropped, rows inserted); the
    list unchanged when the group is not joint mode, its first row is a
    work point, or the start point is already the group's first work point. Copies, never modifies the input."""
    idx = next((i for i, p in enumerate(points)
                if p.get('scan', True) and p.get('source_point_id') is not None
                and int(p['source_point_id']) == int(start_sp)), None)
    # the group's first WORK point (a plan marks the ones before the start
    # scan False but keeps their source_point_id)
    first_scan = next((i for i, p in enumerate(points) if p.get('source_point_id') is not None), None)
    if (idx is None or not points or points[0].get('mode') != 'joint'
            or points[0].get('scan', True) or idx == first_scan):
        return [dict(p) for p in points], 0, 0
    origin, target = points[0], points[idx]
    q0 = [float(v) for v in origin['joints']]
    q1 = [float(v) for v in target['joints']]
    n = max(1, int(math.ceil(max(abs(b - a) for a, b in zip(q0, q1)) / step_rad - 1e-9)))
    approach = []
    for s in range(1, n):
        p = {k: v for k, v in target.items() if k not in ('x', 'y', 'z')}
        p['joints'] = [a + (b - a) * s / n for a, b in zip(q0, q1)]
        p['scan'] = False
        p['source_point_id'] = None
        p['point_id'] = origin.get('point_id')
        p['speed'] = 10.0
        approach.append(p)
    tgt = dict(target)
    tgt['speed'] = 10.0
    out = [dict(origin)] + approach + [tgt] + [dict(p) for p in points[idx + 1:]]
    return out, idx - 1, len(approach)


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
