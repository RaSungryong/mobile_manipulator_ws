"""
orchestrator.py
===============
The map calibration session: drive base + arm through a plan of
(path_tag, ref_tag) pairs, calling the existing chain to compute each
path tag's true world (x, y).

Reuses (NEVER reimplements):
- ``chain.compute_T_A2B`` + ``compute_T_B_world``
- ``align_runner.run_auto_align``
- ``persistence.save_locate_run`` + ``save_locate_failure``
- ``detections`` helpers (shared-detector observations)
- ``arm_interface.ArmInterface`` (injected as ``tcp_client``)
- ``base_interface.BaseInterface`` (injected as ``base``)
- ``geometry`` helpers

Hardware is reached exclusively through the main stack's owner nodes
(arm_node / mobile_node / robot_camera_node) via the two injected
interfaces — this package publishes no /cmd_vel and opens no SDK
connection of its own.
"""
import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np
import rospy
from std_msgs.msg import Int32, String

from ..align import clamp_step, tag_in_cam_report
from ..align_runner import run_auto_align, AutoAlignError, approach_pose
from ..geometry import matrix_m_to_pose_fr5, pose_fr5_to_matrix_m
from ..chain import compensate_T_ab2mb, compute_T_A2B, compute_T_B_world
from ..detections import (detection_to_T_cam2tag, mean_detection,
                          wait_for_tag_detections)
from ..geometry import rot2rpy_deg
from ..persistence import save_locate_failure, save_locate_run
from ..ros_image import grab_image
from .session_log import SessionRecorder
from .map_io import (
    atomic_write,
    load_map,
    make_world_map,
    upsert_world_tag,
)
from .plan_io import (
    CalibrationPlan,
    PlanEntry,
    RefTag,
    load_calibration_plan,
    load_reference_tags,
)
from .view_pose import (
    BaseAnchor,
    compute_T_world2mb_from_chain,
    compute_view_tcp,
    propagate_world_mb,
)


@dataclass
class OrchestratorCfg:
    """Bundle of paths + parameters consumed by the orchestrator. The
    fields cover everything the orchestrator needs that is NOT already
    on the existing path_tag_locator AlignCfg / FairinoTCPClient."""
    # File paths
    ref_tags_yaml: str
    plan_yaml: str
    map_in_path: str
    map_out_path: str
    save_dir: str                         # persistence root (~/.ros/...)
    # Shared-detector observations (reused from locator.yaml)
    hand_cam_detections_topic: str
    front_cam_detections_topic: str
    hand_cam_detector_size_m: float
    front_cam_detector_size_m: float
    detection_wait_timeout_s: float
    tag_a_size_m_default: float
    tag_b_size_m: float
    # arm
    align_cfg: Any                        # AlignCfg dataclass
    # Raw-frame snapshot topics for the persistence record (optional,
    # best-effort; empty string disables)
    hand_cam_image_topic: str = ""
    front_cam_image_topic: str = ""
    # Metadata only (the shared detector owns the actual family setting)
    tag_family: str = "tag36h11"
    # Behavior
    dry_run: bool = False
    # Tuck the arm home before every base move (user rule 2026-09-03:
    # never drive/pivot with the arm extended at a view pose).
    home_before_nav: bool = True


@dataclass
class CalibReport:
    num_succeeded: int
    num_failed: int
    output_yaml_path: str
    entries: List[Dict[str, Any]] = field(default_factory=list)
    session_dir: str = ""           # append-only per-attempt record


# ---------------------------------------------------------------------------
class CalibrationOrchestrator:
    """Loads everything once, then runs ``run()`` over the plan.

    Construction arguments are intentionally explicit (no global state) so
    the orchestrator can be smoke-tested in isolation.
    """

    def __init__(self,
                 cfg: OrchestratorCfg,
                 T_hc2ee,
                 T_ab2mb,
                 T_mb2fc,
                 tcp_client,
                 base=None,
                 lift=None,
                 cancel_check=None,
                 progress_pub: Optional[rospy.Publisher] = None,
                 target_pub: Optional[rospy.Publisher] = None):
        self.cfg = cfg
        self.T_hc2ee = T_hc2ee
        self.T_ab2mb = T_ab2mb        # lift-at-origin (extrinsics.yaml)
        self.T_mb2fc = T_mb2fc
        self.tcp_client = tcp_client   # arm_interface.ArmInterface
        self.base = base               # base_interface.BaseInterface
        self.lift = lift               # lift_listener.LiftHeightListener
        self._cancel_check = cancel_check   # callable -> bool, or None
        self._progress_pub = progress_pub
        self._target_pub = target_pub

        # Eager loads — any schema error surfaces before the first move.
        self.ref_tags: Dict[int, RefTag] = load_reference_tags(cfg.ref_tags_yaml)
        self.plan: CalibrationPlan = load_calibration_plan(cfg.plan_yaml)
        # Read map.yaml read-only for cross-reference metadata + base nav
        # supports. We never mutate or echo it into the output file —
        # map.yaml lives in the Manipulator/Map frame and the output
        # lives in the World frame (defined by reference_tags.yaml), so
        # mixing them would be a coordinate-frame bug.
        self.map_data: dict = load_map(cfg.map_in_path)

        # Fresh world-frame output container.
        # If reference_tags.yaml has a tag whose pose is the identity,
        # advertise it as the origin; otherwise leave None.
        origin_id = None
        for tid, rt in self.ref_tags.items():
            if np.allclose(rt.T_world, np.eye(4), atol=1e-9):
                origin_id = tid
                break
        self.world_data: dict = make_world_map(
            origin_tag_id=origin_id,
            reference_tags_path=str(cfg.ref_tags_yaml),
            calibration_plan_path=str(cfg.plan_yaml),
            map_in_path=str(cfg.map_in_path),
        )

        # Bootstrap anchor for auto-view-pose. Populated after the first
        # successful chain; used to estimate arm_view_tcp_mm_deg for
        # subsequent entries that don't supply one explicitly.
        self._anchor: Optional[BaseAnchor] = None
        # Seed corrections learned in THIS session: for every successful
        # entry that started from a plan seed, (aligned TCP - seed) in the
        # arm frame, keyed by ref tag. A retry after "tag not seen from the
        # seed" applies the median to the failing entry's seed — i.e. the
        # initial pose is estimated from the tags already measured.
        self._seed_corrections: Dict[int, List[np.ndarray]] = {}
        # Session-start lift height (set in run(); drift-guard baseline).
        self._session_lift_m: Optional[float] = None
        # Append-only per-attempt record (session_log.SessionRecorder);
        # None during dry runs, which write nothing by design.
        self._session: Optional[SessionRecorder] = None

        rospy.loginfo(
            "[Calibrator] loaded %d ref tags (origin=%s), %d plan entries; "
            "writing world-frame output to %s (dry_run=%s)",
            len(self.ref_tags), origin_id, len(self.plan.entries),
            cfg.map_out_path, cfg.dry_run)

        # Base navigation goes through mobile_node (injected interface);
        # required unless dry_run.
        if not cfg.dry_run and self.base is None:
            raise RuntimeError(
                "CalibrationOrchestrator needs a BaseInterface unless "
                "dry_run=true")

    # ------------------------------------------------------------------
    def _publish_progress(self, payload: dict):
        if self._progress_pub is None:
            return
        try:
            self._progress_pub.publish(String(data=json.dumps(payload)))
        except Exception as e:
            rospy.logwarn("[Calibrator] progress publish failed: %s", e)

    def _publish_target(self, tag_id: int):
        if self._target_pub is None:
            return
        try:
            self._target_pub.publish(Int32(data=int(tag_id)))
        except Exception:
            pass

    # ------------------------------------------------------------------
    def _lift_height_m(self) -> float:
        """Live lift extension for T_ab2mb compensation; 0.0 when no
        listener was injected (assume lift at origin, as before)."""
        return float(self.lift.height_m()) if self.lift is not None else 0.0

    # ------------------------------------------------------------------
    def _snapshot(self, image_topic):
        """Best-effort raw-frame grab for the persistence record. Returns
        None on empty topic name or timeout — never raises."""
        if not image_topic:
            return None
        try:
            return grab_image(image_topic, timeout=1.0)
        except Exception:
            return None

    # ------------------------------------------------------------------
    def _resolve_ref_size(self, ref: RefTag) -> float:
        return (ref.size_m if ref.size_m is not None
                else self.cfg.tag_a_size_m_default)

    @staticmethod
    def is_seed_failure(error) -> bool:
        """Did an attempt fail because the ref tag was not seen from the
        initial (seed) pose? Those are the failures a different seed can
        fix; a nav or chain failure is retried with the same seed."""
        text = str(error or "")
        return ("not detected at iteration 1" in text
                or "initial move" in text)

    def _seed_correction(self, ref_tag_id: int):
        """Median (aligned TCP - seed) translation, mm, from this
        session's successes — same ref tag first, any ref as fallback.
        (None, 0, False) until at least one entry succeeded from a plan seed."""
        same = self._seed_corrections.get(int(ref_tag_id)) or []
        pool = same if same else [d for ds in self._seed_corrections.values()
                                  for d in ds]
        if not pool:
            return None, 0, bool(same)
        return np.median(np.array(pool), axis=0), len(pool), bool(same)

    def _anchor_view_tcp(self, entry: PlanEntry, ref: RefTag):
        """Bootstrap estimate from the last successful entry's anchor
        (base pose in world from the chain) + map.yaml offsets + odom
        heading; None when no anchor / no base / disabled."""
        if not (bool(getattr(self.cfg.align_cfg, "auto_view_pose", False))
                and self._anchor is not None and self.base is not None):
            return None
        map_xy_now = self._lookup_path_map_xy(entry.path_tag_id)
        theta_now = float(self.base.current_theta)
        T_world2mb_estimate = propagate_world_mb(
            self._anchor, map_xy_now, theta_now)
        tcp = compute_view_tcp(
            T_A_world=ref.T_world,
            T_world2mb=T_world2mb_estimate,
            T_ab2mb=compensate_T_ab2mb(self.T_ab2mb,
                                       self._lift_height_m()),
            T_hc2ee=self.T_hc2ee,
            view_distance_m=float(
                self.cfg.align_cfg.auto_view_distance_m),
        )
        rospy.loginfo(
            "[Calibrator] anchor view_tcp for tag=%d (ref=%d) -> %s "
            "(Δmap=[%.3f,%.3f], Δtheta=%.2fdeg from anchor tag=%d)",
            entry.path_tag_id, entry.ref_tag_id,
            ["%.2f" % v for v in tcp],
            map_xy_now[0] - self._anchor.path_map_xy[0],
            map_xy_now[1] - self._anchor.path_map_xy[1],
            np.degrees(theta_now - self._anchor.odom_theta),
            self._anchor.path_tag_id)
        return [float(v) for v in tcp]

    def _retry_view_tcp(self, entry: PlanEntry, resolved: dict,
                        ref: RefTag, attempt: int):
        """Seed for a RETRY after the ref tag was not seen from the seed.

        Re-sending the identical pose (what happened on 2026-09-04: every
        attempt 2 failed exactly like attempt 1) is pointless, so the
        initial pose is re-estimated from what the session already
        knows, in this order, one strategy per retry:

          1. plan seed + median (aligned - seed) of the entries that
             already succeeded (same ref tag first) — the base stops with
             a repeatable offset, so the tags already measured say where
             this one's seed really is;
          2. anchor bootstrap: base pose from the last successful chain +
             map.yaml offsets + odom heading (compute_view_tcp);
          3. the plan seed with the camera raised `retry_raise_m` — a
             wider field of view when nothing has succeeded yet.

        Returns (tcp, source) or None to keep the attempt-1 pose.
        """
        seed = ([float(v) for v in entry.arm_view_tcp_mm_deg]
                if entry.arm_view_tcp_mm_deg is not None
                else resolved.get("arm_view_tcp_mm_deg"))
        candidates = []
        corr, n, same = self._seed_correction(entry.ref_tag_id)
        if seed is not None and corr is not None:
            tcp = list(seed)
            for i in range(3):
                tcp[i] = float(seed[i] + corr[i])
            candidates.append((tcp, "entry-override+session-correction"
                               f"(ref {'same' if same else 'any'}, n={n}, "
                               f"Δ=[{corr[0]:+.1f},{corr[1]:+.1f},{corr[2]:+.1f}] mm)"))
        try:
            tcp = self._anchor_view_tcp(entry, ref)
        except Exception as e:
            rospy.logwarn("[Calibrator] anchor view_tcp failed (%s)", e)
            tcp = None
        if tcp is not None:
            candidates.append((tcp, "anchor-bootstrap"))
        raise_m = float(getattr(self.cfg.align_cfg, "retry_raise_m", 0.0))
        if seed is not None and raise_m > 0:
            tcp = list(seed)
            tcp[2] = float(seed[2] + raise_m * 1000.0)
            candidates.append((tcp, f"entry-override+raised({raise_m:.2f} m)"))
        if not candidates:
            return None
        idx = min(max(0, int(attempt) - 2), len(candidates) - 1)
        return candidates[idx]

    def _resolve_view_tcp(self, entry: PlanEntry, resolved: dict,
                          ref: RefTag, attempt: int = 1,
                          seed_failed: bool = False):
        """Return (view_tcp_mm_deg, source_str).

        Source order (attempt 1, or a retry after a non-seed failure):
            1. Per-entry override (entry.arm_view_tcp_mm_deg)
            2. Bootstrap auto-estimate (if anchor available + auto_view_pose)
            3. Defaults from calibration_plan.yaml
        A retry after "tag not seen from the seed" goes through
        _retry_view_tcp instead (session correction -> anchor -> raised).
        """
        if attempt >= 2 and seed_failed:
            alt = self._retry_view_tcp(entry, resolved, ref, attempt)
            if alt is not None:
                rospy.logwarn(
                    "[Calibrator] tag=%d retry %d: seed re-estimated (%s)",
                    entry.path_tag_id, attempt, alt[1])
                return alt
            rospy.logwarn(
                "[Calibrator] tag=%d retry %d: no alternative seed "
                "available, re-using the attempt-1 pose",
                entry.path_tag_id, attempt)

        # 1. Per-entry override beats everything.
        if entry.arm_view_tcp_mm_deg is not None:
            return [float(v) for v in entry.arm_view_tcp_mm_deg], "entry-override"

        # 2. Bootstrap auto-estimate.
        if (bool(getattr(self.cfg.align_cfg, "auto_view_pose", False))
                and self._anchor is not None and self.base is not None):
            try:
                map_xy_now = self._lookup_path_map_xy(entry.path_tag_id)
                theta_now = float(self.base.current_theta)
                T_world2mb_estimate = propagate_world_mb(
                    self._anchor, map_xy_now, theta_now)
                tcp = compute_view_tcp(
                    T_A_world=ref.T_world,
                    T_world2mb=T_world2mb_estimate,
                    T_ab2mb=compensate_T_ab2mb(self.T_ab2mb,
                                               self._lift_height_m()),
                    T_hc2ee=self.T_hc2ee,
                    view_distance_m=float(
                        self.cfg.align_cfg.auto_view_distance_m),
                )
                rospy.loginfo(
                    "[Calibrator] auto view_tcp for tag=%d (ref=%d) -> %s "
                    "(Δmap=[%.3f,%.3f], Δtheta=%.2fdeg from anchor tag=%d)",
                    entry.path_tag_id, entry.ref_tag_id,
                    ["%.2f" % v for v in tcp],
                    map_xy_now[0] - self._anchor.path_map_xy[0],
                    map_xy_now[1] - self._anchor.path_map_xy[1],
                    np.degrees(theta_now - self._anchor.odom_theta),
                    self._anchor.path_tag_id)
                return tcp, "auto-bootstrap"
            except Exception as e:
                rospy.logwarn(
                    "[Calibrator] auto view_tcp failed (%s); "
                    "falling back to defaults", e)

        # 3. Defaults.
        default = resolved.get("arm_view_tcp_mm_deg")
        if default is None:
            raise RuntimeError(
                "no arm_view_tcp_mm_deg available; supply one in "
                "defaults.arm_view_tcp_mm_deg, per-entry override, OR "
                "rely on the auto-bootstrap (needs one prior successful "
                "entry + align.auto_view_pose=true).")
        return [float(v) for v in default], "defaults"

    def _lookup_path_map_xy(self, path_tag_id: int):
        """Read (x, y) of a path tag from the loaded map.yaml. Raises
        KeyError if the tag is missing or has no coordinates."""
        info = self.map_data.get("tags", {}).get(int(path_tag_id))
        if info is None or "x" not in info or "y" not in info:
            raise KeyError(
                f"path_tag_id={path_tag_id} missing (x, y) in map.yaml")
        return (float(info["x"]), float(info["y"]))

    def _update_anchor(self, entry: PlanEntry, T_fc2B: np.ndarray,
                        T_B_world: np.ndarray):
        """After a successful chain, snapshot (mb in world, odom_theta,
        path_xy_map) for use as a seed by subsequent entries."""
        T_world2mb = compute_T_world2mb_from_chain(
            T_B_world=T_B_world, T_fc2B=T_fc2B, T_mb2fc=self.T_mb2fc)
        theta = (float(self.base.current_theta)
                 if self.base is not None else 0.0)
        map_xy = self._lookup_path_map_xy(entry.path_tag_id)
        self._anchor = BaseAnchor(
            T_world2mb=T_world2mb,
            odom_theta=theta,
            path_tag_id=int(entry.path_tag_id),
            path_map_xy=map_xy,
        )
        rospy.loginfo(
            "[Calibrator] anchor updated: tag=%d, base in world=(%.3f, %.3f, %.3f), "
            "odom_theta=%.2fdeg",
            entry.path_tag_id,
            T_world2mb[0, 3], T_world2mb[1, 3], T_world2mb[2, 3],
            np.degrees(theta))

    def run(self) -> CalibReport:
        report = CalibReport(
            num_succeeded=0, num_failed=0,
            output_yaml_path=str(self.cfg.map_out_path))

        # Fixed-height sessions: remember where the lift started so any
        # mid-session drift is flagged. The chain always compensates by
        # the LIVE height, so accuracy only suffers if the encoder LIES —
        # which is exactly what a silent height change suggests.
        self._session_lift_m = (self._lift_height_m()
                                if not self.cfg.dry_run else None)
        if self._session_lift_m:
            rospy.loginfo("[Calibrator] session lift height: %.1f mm",
                          self._session_lift_m * 1000.0)

        # Every attempt of every entry gets its own numbered file, in
        # execution order, under <save_dir>/calibrate/<ts>/ — retries and
        # failures included. Nothing there is ever overwritten.
        self._session = None
        if not self.cfg.dry_run:
            try:
                self._session = SessionRecorder(self.cfg.save_dir, meta={
                    "plan_yaml": str(self.cfg.plan_yaml),
                    "ref_tags_yaml": str(self.cfg.ref_tags_yaml),
                    "map_in_path": str(self.cfg.map_in_path),
                    "map_out_path": str(self.cfg.map_out_path),
                    "session_lift_m": self._session_lift_m,
                    "num_plan_entries": len(self.plan.entries),
                    "dry_run": False,
                })
                report.session_dir = str(self._session.dir)
                rospy.loginfo("[Calibrator] session record -> %s",
                              self._session.dir)
            except Exception as e:
                rospy.logwarn("[Calibrator] session record disabled: %s", e)

        for entry in self.plan.entries:
            if self._cancel_check is not None and self._cancel_check():
                rospy.logwarn('[Calibrator] session cancelled before '
                              'tag=%d — stopping (partial output kept)',
                              entry.path_tag_id)
                self._publish_progress({'tag': entry.path_tag_id,
                                        'status': 'cancelled'})
                if self._session is not None:
                    self._session.mark_cancelled(entry.path_tag_id)
                break
            self._publish_target(entry.path_tag_id)
            resolved = self.plan.resolved(entry)
            ref = self.ref_tags.get(entry.ref_tag_id)
            if ref is None:
                msg = (f"ref_tag_id {entry.ref_tag_id} not in "
                       f"reference_tags.yaml")
                seq = self._record(entry, resolved, 1, "fail", {}, error=msg)
                report.entries.append({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "fail", "error": msg, "seq": seq})
                report.num_failed += 1
                self._publish_progress({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "fail", "error": msg, "seq": seq})
                if not self.cfg.dry_run:
                    save_locate_failure(
                        root_dir=self.cfg.save_dir,
                        tag_b_id=entry.path_tag_id,
                        message=msg,
                        request_echo={"entry": resolved})
                continue

            try:
                if self.cfg.dry_run:
                    rospy.loginfo(
                        "[Calibrator] DRY_RUN entry tag=%d ref=%d resolved=%s",
                        entry.path_tag_id, entry.ref_tag_id, resolved)
                    report.entries.append({
                        "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                        "status": "dry_run"})
                    # Publish too — consumers (robot_ui log) show per-tag
                    # lines for dry runs as well, not just live sessions.
                    self._publish_progress({
                        "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                        "status": "dry_run"})
                    continue

                # retry_count from the plan (defaults block / per entry):
                # N retries AFTER the first failed attempt. Re-runs the
                # whole entry including nav + align.
                attempts = 1 + max(0, int(resolved.get("retry_count", 0)))
                seq = None
                seed_failed = False
                for attempt in range(1, attempts + 1):
                    # `rec` is filled progressively by _run_one so a failed
                    # attempt still records everything up to the failure.
                    rec: Dict[str, Any] = {}
                    try:
                        self._run_one(entry, resolved, ref, rec,
                                      attempt=attempt,
                                      seed_failed=seed_failed)
                        seq = self._record(entry, resolved, attempt, "ok", rec)
                        break
                    except Exception as e:
                        seq = self._record(entry, resolved, attempt, "fail",
                                           rec, error=str(e))
                        seed_failed = self.is_seed_failure(e)
                        if attempt >= attempts:
                            raise
                        rospy.logwarn(
                            "[Calibrator] tag=%d attempt %d/%d failed "
                            "(%s) — retrying", entry.path_tag_id,
                            attempt, attempts, e)

                world_entry = self.world_data["tags"][entry.path_tag_id]
                pos_x, pos_y, pos_z = world_entry["position_m"]
                degraded = (rec.get("auto_align") or {}).get("degraded") \
                    if isinstance(rec.get("auto_align"), dict) else None
                if degraded:
                    rospy.logwarn(
                        "[Calibrator] tag=%d OK but DEGRADED (%s) — "
                        "verify this entry's residual in "
                        "verify_map_world.py before trusting it",
                        entry.path_tag_id, degraded)
                entry_result = {
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "ok", "x": pos_x, "y": pos_y, "z": pos_z,
                    "seq": seq}
                if degraded:
                    entry_result["degraded"] = degraded
                report.entries.append(entry_result)
                report.num_succeeded += 1
                self._publish_progress(dict(entry_result))
            except Exception as e:
                rospy.logerr("[Calibrator] entry tag=%d failed: %s",
                             entry.path_tag_id, e)
                try:
                    save_locate_failure(
                        root_dir=self.cfg.save_dir,
                        tag_b_id=entry.path_tag_id,
                        message=str(e),
                        request_echo={"entry": resolved})
                except Exception as save_err:
                    rospy.logwarn(
                        "[Calibrator] failed to persist failure: %s", save_err)
                report.entries.append({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "fail", "error": str(e)})
                report.num_failed += 1
                self._publish_progress({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "fail", "error": str(e)})
                # continue; failure does NOT abort the whole session

        # Final write — even when every entry failed we still emit an
        # output file (with the bookkeeping header and `tags: {}`), which
        # signals "ran but produced nothing" rather than ambiguity.
        if not self.cfg.dry_run:
            atomic_write(self.world_data, self.cfg.map_out_path)
            rospy.loginfo(
                "[Calibrator] final map_world.yaml -> %s "
                "(succeeded=%d, failed=%d)",
                self.cfg.map_out_path,
                report.num_succeeded, report.num_failed)
        if self._session is not None:
            try:
                self._session.finish(report.num_succeeded, report.num_failed,
                                     str(self.cfg.map_out_path),
                                     world_data=self.world_data)
            except Exception as e:
                rospy.logwarn("[Calibrator] session record finish failed: %s", e)
        return report

    # ------------------------------------------------------------------
    def _record(self, entry: PlanEntry, resolved: dict, attempt: int,
                status: str, rec: dict, error: Optional[str] = None):
        """Write one attempt to the session record. Never raises; returns
        the sequence number or None (dry run / recorder unavailable)."""
        if self._session is None:
            return None
        payload = {
            "path_tag_id": int(entry.path_tag_id),
            "ref_tag_id": int(entry.ref_tag_id),
            "attempt": int(attempt),
            "status": status,
            "error": error,
            "plan_entry": dict(resolved),
        }
        payload.update(rec)
        try:
            return self._session.record(payload)
        except Exception as e:
            rospy.logwarn("[Calibrator] session record write failed: %s", e)
            return None

    # ------------------------------------------------------------------
    def _run_one(self, entry: PlanEntry, resolved: dict, ref: RefTag,
                 rec: Optional[dict] = None, attempt: int = 1,
                 seed_failed: bool = False):
        """Process a single plan entry; raises on any failure.

        ``rec`` (if given) is filled step by step with everything worth
        keeping — nav, view pose, the auto-align report, the camera-frame
        observations, the chain result — so the session record of a
        FAILED attempt still shows how far it got.
        """
        if rec is None:
            rec = {}

        # 0. Tuck the arm to its home pose BEFORE the base moves: routes
        # include pivots and reverse corridor entries, and driving them
        # with the arm extended at the previous entry's view pose is a
        # collision risk. Synchronous — nav starts only once the arm is
        # home. Failure fails the entry (an arm that will not home is
        # not an arm to drive around with).
        if self.cfg.home_before_nav and hasattr(self.tcp_client,
                                                'move_home'):
            ok, msg = self.tcp_client.move_home()
            if not ok:
                raise RuntimeError(f"arm home before nav failed: {msg}")
            rec["arm_homed_before_nav"] = True

        # 1. Navigate base through mobile_node. If nav_start_id is
        # supplied, go there first so the controller has a known starting
        # tag for path finding to path_tag_id.
        if resolved.get("nav_start_id") is not None:
            ok = self.base.goto(resolved["nav_start_id"])
            if not ok:
                raise RuntimeError(
                    f"base nav to nav_start_id={resolved['nav_start_id']} failed")
        ok = self.base.goto(entry.path_tag_id)
        if not ok:
            raise RuntimeError(
                f"base nav to path_tag_id={entry.path_tag_id} failed")
        rec["nav"] = {
            "nav_start_id": resolved.get("nav_start_id"),
            "arrived_tag_id": int(entry.path_tag_id),
            "base_theta_deg": float(np.degrees(self.base.current_theta)),
        }

        # 2. Resolve arm viewing pose. Order of precedence:
        #    (a) explicit override in this entry (plan.yaml)
        #    (b) bootstrap estimate from previous anchor (if available)
        #    (c) defaults.arm_view_tcp_mm_deg
        #    Raises if none of the above are present.
        view_tcp, view_source = self._resolve_view_tcp(
            entry, resolved, ref, attempt=attempt, seed_failed=seed_failed)
        rec["view_tcp_mm_deg"] = [float(v) for v in view_tcp]
        rec["view_tcp_source"] = view_source
        rec["attempt"] = int(attempt)

        align_required = bool(resolved.get("align_required", True))
        rec["align_required"] = align_required
        rec["auto_align"] = None
        if align_required:
            try:
                rec["auto_align"] = run_auto_align(
                    align_cfg=self.cfg.align_cfg,
                    tcp_client=self.tcp_client,
                    T_hc2ee=self.T_hc2ee,
                    tag_a_id=entry.ref_tag_id,
                    tag_a_size_m=self._resolve_ref_size(ref),
                    hand_cam_detections_topic=self.cfg.hand_cam_detections_topic,
                    hand_cam_detector_size_m=self.cfg.hand_cam_detector_size_m,
                    detection_wait_timeout_s=self.cfg.detection_wait_timeout_s,
                    initial_tcp_mm_deg=view_tcp,
                )
            except AutoAlignError as e:
                rec["auto_align"] = e.report     # keep the partial history
                raise
        else:
            # align_required=false skips run_auto_align — which is also
            # where the initial-step CLAMP lives. Use the same chunked,
            # clamped approach so a bad per-entry pose or a wild
            # bootstrap estimate cannot go to the arm as one unbounded
            # MoveJ, while a seed farther than one step is still reached.
            approach_pose(self.tcp_client, view_tcp, self.cfg.align_cfg,
                          what="view")

        # 3. Capture fresh observations from the shared detector and run
        # the chain. AVERAGED over samples_per_iteration frames: this is
        # the measurement the result is computed from, and the error
        # budget shows its in-plane yaw noise (x the A->B lever) is the
        # dominant path-tag position error — averaging divides it by
        # ~sqrt(n). (The align loop uses the median pick; here the mean
        # is right: one synthetic low-noise observation.)
        n_samp = max(1, int(getattr(self.cfg.align_cfg,
                                    'samples_per_iteration', 1)))
        det_a = mean_detection(wait_for_tag_detections(
            self.cfg.hand_cam_detections_topic, int(entry.ref_tag_id),
            n_samp, timeout=self.cfg.detection_wait_timeout_s))
        T_hc2A = detection_to_T_cam2tag(
            det_a, self._resolve_ref_size(ref),
            self.cfg.hand_cam_detector_size_m)
        det_b = mean_detection(wait_for_tag_detections(
            self.cfg.front_cam_detections_topic, int(entry.path_tag_id),
            n_samp, timeout=self.cfg.detection_wait_timeout_s))
        T_fc2B = detection_to_T_cam2tag(
            det_b, self.cfg.tag_b_size_m,
            self.cfg.front_cam_detector_size_m)
        tcp_pose = self.tcp_client.get_tcp_pose()
        # Remember how far the aligned pose sits from the plan seed, so a
        # later entry whose seed misses its tag can be re-seeded from the
        # tags already measured (see _retry_view_tcp). Only entries that
        # started from the plan seed itself contribute; raised or
        # corrected retries would double-count.
        if (entry.arm_view_tcp_mm_deg is not None
                and view_source == "entry-override"
                and (rec.get("auto_align") or {}).get("final_tcp")):
            delta = (np.array(tcp_pose[:3], dtype=float)
                     - np.array(entry.arm_view_tcp_mm_deg[:3], dtype=float))
            self._seed_corrections.setdefault(
                int(entry.ref_tag_id), []).append(delta)
            rec["seed_delta_mm"] = [float(v) for v in delta]
        # Camera-frame error of both observations (see align.CAM_FRAME_NOTE).
        observations = {
            "hand_cam_to_tag_a": dict(tag_id=int(entry.ref_tag_id),
                                      **tag_in_cam_report(T_hc2A)),
            "front_cam_to_tag_b": dict(tag_id=int(entry.path_tag_id),
                                       **tag_in_cam_report(T_fc2B)),
        }
        rec["observations"] = observations
        rec["tcp_pose_mm_deg"] = [float(v) for v in tcp_pose]

        lift_height_m = self._lift_height_m()
        rec["lift_height_m"] = float(lift_height_m)
        rec["session_lift_m"] = self._session_lift_m
        if lift_height_m:
            rospy.loginfo("[Calibrator] compensating chain for lift "
                          "height %.1f mm", lift_height_m * 1000.0)
        if (self._session_lift_m is not None
                and abs(lift_height_m - self._session_lift_m) > 0.005):
            rospy.logwarn(
                "[Calibrator] lift height drifted %.1f mm from session "
                "start (%.1f -> %.1f mm). Compensation follows the live "
                "value, but on this lift a silent change usually means "
                "the ENCODER drifted — consider re-homing and re-running "
                "affected entries.",
                (lift_height_m - self._session_lift_m) * 1000.0,
                self._session_lift_m * 1000.0, lift_height_m * 1000.0)
        out = compute_T_A2B(
            T_hc2A=T_hc2A, T_fc2B=T_fc2B,
            tcp_pose_mm_deg=tcp_pose,
            T_hc2ee=self.T_hc2ee,
            T_ab2mb=self.T_ab2mb, T_mb2fc=self.T_mb2fc,
            lift_height_m=lift_height_m,
        )
        T_A2B = out["T_A2B"]
        T_B_world = compute_T_B_world(ref.T_world, T_A2B)
        pos_m = T_B_world[:3, 3]
        rpy_deg = rot2rpy_deg(T_B_world[:3, :3])
        rec["result_world"] = {
            "position_m": [float(v) for v in pos_m],
            "rpy_deg": [float(v) for v in rpy_deg],
            "ref_tag_id": int(entry.ref_tag_id),
        }
        rec["T_A2B_row_major"] = [float(v) for v in T_A2B.flatten()]
        rec["T_B_world_row_major"] = [float(v) for v in T_B_world.flatten()]

        # 4. Persist the full 6-DOF result (observations + transforms +
        # CSV; raw-frame snapshots are best-effort extras).
        run_dir = save_locate_run(
            root_dir=self.cfg.save_dir,
            tag_b_id=entry.path_tag_id,
            image_hc=self._snapshot(self.cfg.hand_cam_image_topic),
            image_fc=self._snapshot(self.cfg.front_cam_image_topic),
            T_hc2A=T_hc2A, T_fc2B=T_fc2B,
            T_A_world=ref.T_world,
            T_hc2ee=self.T_hc2ee,
            T_ab2mb=self.T_ab2mb, T_mb2fc=self.T_mb2fc,
            tcp_pose_mm_deg=tcp_pose,
            T_A2B=T_A2B, T_B_world=T_B_world,
            position_m=(float(pos_m[0]), float(pos_m[1]), float(pos_m[2])),
            rpy_deg=(float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2])),
            tag_a_id=entry.ref_tag_id,
            tag_a_size_m=self._resolve_ref_size(ref),
            tag_b_size_m=self.cfg.tag_b_size_m,
            family=self.cfg.tag_family,
            request_echo={"plan_entry": {
                "path_tag_id": entry.path_tag_id,
                "ref_tag_id": entry.ref_tag_id,
                "nav_start_id": entry.nav_start_id,
                "arm_view_tcp_mm_deg": view_tcp,
                "view_tcp_source": view_source,
                "lift_height_m": lift_height_m,
            }},
            success=True,
            message="ok",
            observations=observations,
            auto_align_report=rec["auto_align"],
        )
        rec["locate_run_dir"] = str(run_dir)

        # 4.5 Update the bootstrap anchor for the next entry. This uses
        # T_fc2B captured during the chain (front-cam saw the path tag
        # just now) to invert the kinematic to find where mb sits in
        # world. Logged only — failure to update the anchor (extremely
        # unlikely) is non-fatal.
        try:
            self._update_anchor(entry, out["intermediates"]["T_fc2B"], T_B_world)
        except Exception as e:
            rospy.logwarn(
                "[Calibrator] failed to update view-pose anchor: %s", e)

        # 5. Update in-memory world-frame output and atomically rewrite
        # map_world.yaml so a mid-run crash still yields the partial set
        # of successful entries.
        map_meta = self.map_data.get("tags", {}).get(entry.path_tag_id, {})
        upsert_world_tag(
            self.world_data,
            tag_id=entry.path_tag_id,
            position_m=(float(pos_m[0]), float(pos_m[1]), float(pos_m[2])),
            rpy_deg=(float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2])),
            ref_tag_id=entry.ref_tag_id,
            map_xy=(map_meta.get("x"), map_meta.get("y"))
                   if "x" in map_meta and "y" in map_meta else None,
            tag_type=map_meta.get("type"),
            zone=map_meta.get("zone"),
            name=map_meta.get("name"),
        )
        atomic_write(self.world_data, self.cfg.map_out_path)
        if self._session is not None:
            try:
                self._session.write_map_copy(self.world_data)
            except Exception as e:
                rospy.logwarn("[Calibrator] session map copy failed: %s", e)
