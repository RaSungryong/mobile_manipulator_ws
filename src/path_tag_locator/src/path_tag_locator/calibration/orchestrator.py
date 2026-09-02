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
from ..align_runner import run_auto_align, AutoAlignError
from ..geometry import matrix_m_to_pose_fr5, pose_fr5_to_matrix_m
from ..chain import compensate_T_ab2mb, compute_T_A2B, compute_T_B_world
from ..detections import detection_to_T_cam2tag, wait_for_tag_detection
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

    def _resolve_view_tcp(self, entry: PlanEntry, resolved: dict,
                          ref: RefTag):
        """Return (view_tcp_mm_deg, source_str).

        Source order:
            1. Per-entry override (entry.arm_view_tcp_mm_deg)
            2. Bootstrap auto-estimate (if anchor available + auto_view_pose)
            3. Defaults from calibration_plan.yaml
        """
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
                for attempt in range(1, attempts + 1):
                    # `rec` is filled progressively by _run_one so a failed
                    # attempt still records everything up to the failure.
                    rec: Dict[str, Any] = {}
                    try:
                        self._run_one(entry, resolved, ref, rec)
                        seq = self._record(entry, resolved, attempt, "ok", rec)
                        break
                    except Exception as e:
                        seq = self._record(entry, resolved, attempt, "fail",
                                           rec, error=str(e))
                        if attempt >= attempts:
                            raise
                        rospy.logwarn(
                            "[Calibrator] tag=%d attempt %d/%d failed "
                            "(%s) — retrying", entry.path_tag_id,
                            attempt, attempts, e)

                world_entry = self.world_data["tags"][entry.path_tag_id]
                pos_x, pos_y, pos_z = world_entry["position_m"]
                report.entries.append({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "ok", "x": pos_x, "y": pos_y, "z": pos_z,
                    "seq": seq})
                report.num_succeeded += 1
                self._publish_progress({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "ok", "x": pos_x, "y": pos_y, "z": pos_z,
                    "seq": seq})
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
                 rec: Optional[dict] = None):
        """Process a single plan entry; raises on any failure.

        ``rec`` (if given) is filled step by step with everything worth
        keeping — nav, view pose, the auto-align report, the camera-frame
        observations, the chain result — so the session record of a
        FAILED attempt still shows how far it got.
        """
        if rec is None:
            rec = {}

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
        view_tcp, view_source = self._resolve_view_tcp(entry, resolved, ref)
        rec["view_tcp_mm_deg"] = [float(v) for v in view_tcp]
        rec["view_tcp_source"] = view_source

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
            # where the initial-step CLAMP lives. Apply the same clamp
            # here so a bad per-entry pose or a wild bootstrap estimate
            # cannot go to the arm as one unbounded MoveJ.
            T_cur = pose_fr5_to_matrix_m(self.tcp_client.get_tcp_pose())
            T_tgt = pose_fr5_to_matrix_m(view_tcp)
            step = clamp_step(
                T_cur, T_tgt,
                max_step_m=self.cfg.align_cfg.max_initial_step_m,
                max_step_deg=self.cfg.align_cfg.max_initial_step_deg)
            if step.clamped:
                rospy.logwarn(
                    "[Calibrator] view move clamped (Δt=%.3f m, "
                    "Δrot=%.2f deg)", step.delta_t_norm_m,
                    step.delta_rot_deg)
            self.tcp_client.move_j_to_pose(
                matrix_m_to_pose_fr5(step.T_ab2ee_step), linear=False)

        # 3. Capture fresh observations from the shared detector and run
        # the chain.
        det_a = wait_for_tag_detection(
            self.cfg.hand_cam_detections_topic, int(entry.ref_tag_id),
            timeout=self.cfg.detection_wait_timeout_s)
        T_hc2A = detection_to_T_cam2tag(
            det_a, self._resolve_ref_size(ref),
            self.cfg.hand_cam_detector_size_m)
        det_b = wait_for_tag_detection(
            self.cfg.front_cam_detections_topic, int(entry.path_tag_id),
            timeout=self.cfg.detection_wait_timeout_s)
        T_fc2B = detection_to_T_cam2tag(
            det_b, self.cfg.tag_b_size_m,
            self.cfg.front_cam_detector_size_m)
        tcp_pose = self.tcp_client.get_tcp_pose()
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
