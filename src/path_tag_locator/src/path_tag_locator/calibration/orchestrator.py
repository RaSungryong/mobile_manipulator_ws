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
- ``ros_image.grab_image_and_K``
- ``tcp_pose.FairinoTCPClient``
- ``geometry`` helpers
- ``nav.Navigator``
"""
import datetime as _dt
import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np
import rospy
from std_msgs.msg import Int32, String

from ..align_runner import run_auto_align
from ..chain import compute_T_A2B, compute_T_B_world
from ..geometry import rot2rpy_deg
from ..nav.navigator import Navigator
from ..persistence import save_locate_failure, save_locate_run
from ..ros_image import grab_image_and_K
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
    # Reused from locator.yaml
    hand_cam_image_topic: str
    hand_cam_info_topic: str
    front_cam_image_topic: str
    front_cam_info_topic: str
    image_wait_timeout_s: float
    tag_family: str
    tag_a_size_m_default: float
    tag_b_size_m: float
    # nav (loaded from robot_nav.yaml)
    robot_nav_yaml: str
    map_yaml_for_nav: str
    # arm
    align_cfg: Any                        # AlignCfg dataclass
    # Behavior
    dry_run: bool = False


@dataclass
class CalibReport:
    num_succeeded: int
    num_failed: int
    output_yaml_path: str
    entries: List[Dict[str, Any]] = field(default_factory=list)


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
                 progress_pub: Optional[rospy.Publisher] = None,
                 target_pub: Optional[rospy.Publisher] = None):
        self.cfg = cfg
        self.T_hc2ee = T_hc2ee
        self.T_ab2mb = T_ab2mb
        self.T_mb2fc = T_mb2fc
        self.tcp_client = tcp_client
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

        rospy.loginfo(
            "[Calibrator] loaded %d ref tags (origin=%s), %d plan entries; "
            "writing world-frame output to %s (dry_run=%s)",
            len(self.ref_tags), origin_id, len(self.plan.entries),
            cfg.map_out_path, cfg.dry_run)

        # Lazy Navigator — only construct if not in dry_run, since it
        # spins up ROS subscribers that bind to live camera topics.
        self.nav: Optional[Navigator] = None
        if not cfg.dry_run:
            self.nav = Navigator(
                robot_cfg=cfg.robot_nav_yaml,
                map_yaml_path=cfg.map_yaml_for_nav,
                wait_for_camera_s=5.0,
            )

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
                and self._anchor is not None and self.nav is not None):
            try:
                map_xy_now = self._lookup_path_map_xy(entry.path_tag_id)
                theta_now = float(self.nav.robot.current_theta)
                T_world2mb_estimate = propagate_world_mb(
                    self._anchor, map_xy_now, theta_now)
                tcp = compute_view_tcp(
                    T_A_world=ref.T_world,
                    T_world2mb=T_world2mb_estimate,
                    T_ab2mb=self.T_ab2mb,
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
        theta = (float(self.nav.robot.current_theta)
                 if self.nav is not None else 0.0)
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

        for entry in self.plan.entries:
            self._publish_target(entry.path_tag_id)
            resolved = self.plan.resolved(entry)
            ref = self.ref_tags.get(entry.ref_tag_id)
            if ref is None:
                msg = (f"ref_tag_id {entry.ref_tag_id} not in "
                       f"reference_tags.yaml")
                report.entries.append({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "fail", "error": msg})
                report.num_failed += 1
                self._publish_progress({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "fail", "error": msg})
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
                    continue

                self._run_one(entry, resolved, ref)

                world_entry = self.world_data["tags"][entry.path_tag_id]
                pos_x, pos_y, pos_z = world_entry["position_m"]
                report.entries.append({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "ok", "x": pos_x, "y": pos_y, "z": pos_z})
                report.num_succeeded += 1
                self._publish_progress({
                    "tag": entry.path_tag_id, "ref": entry.ref_tag_id,
                    "status": "ok", "x": pos_x, "y": pos_y, "z": pos_z})
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
        return report

    # ------------------------------------------------------------------
    def _run_one(self, entry: PlanEntry, resolved: dict, ref: RefTag):
        """Process a single plan entry; raises on any failure."""

        # 1. Navigate base. If nav_start_id is supplied, go there first so
        # the controller has a known starting tag for path finding to
        # path_tag_id.
        if resolved.get("nav_start_id") is not None:
            ok = self.nav.goto(resolved["nav_start_id"])
            if not ok:
                raise RuntimeError(
                    f"base nav to nav_start_id={resolved['nav_start_id']} failed")
        ok = self.nav.goto(entry.path_tag_id)
        if not ok:
            raise RuntimeError(
                f"base nav to path_tag_id={entry.path_tag_id} failed")

        # 2. Resolve arm viewing pose. Order of precedence:
        #    (a) explicit override in this entry (plan.yaml)
        #    (b) bootstrap estimate from previous anchor (if available)
        #    (c) defaults.arm_view_tcp_mm_deg
        #    Raises if none of the above are present.
        view_tcp, view_source = self._resolve_view_tcp(entry, resolved, ref)

        align_required = bool(resolved.get("align_required", True))
        if align_required:
            run_auto_align(
                align_cfg=self.cfg.align_cfg,
                tcp_client=self.tcp_client,
                T_hc2ee=self.T_hc2ee,
                tag_a_id=entry.ref_tag_id,
                tag_a_size_m=self._resolve_ref_size(ref),
                tag_family=self.cfg.tag_family,
                hand_cam_image_topic=self.cfg.hand_cam_image_topic,
                hand_cam_info_topic=self.cfg.hand_cam_info_topic,
                image_wait_timeout_s=self.cfg.image_wait_timeout_s,
                initial_tcp_mm_deg=view_tcp,
            )
        else:
            self.tcp_client.move_j_to_pose(view_tcp)

        # 3. Capture fresh inputs and run the chain.
        hc_img, K_hc = grab_image_and_K(
            self.cfg.hand_cam_image_topic, self.cfg.hand_cam_info_topic,
            timeout=self.cfg.image_wait_timeout_s)
        fc_img, K_fc = grab_image_and_K(
            self.cfg.front_cam_image_topic, self.cfg.front_cam_info_topic,
            timeout=self.cfg.image_wait_timeout_s)
        tcp_pose = self.tcp_client.get_tcp_pose()

        out = compute_T_A2B(
            image_hc=hc_img, image_fc=fc_img,
            tcp_pose_mm_deg=tcp_pose,
            T_hc2ee=self.T_hc2ee,
            K_hc=K_hc, K_fc=K_fc,
            T_ab2mb=self.T_ab2mb, T_mb2fc=self.T_mb2fc,
            tag_a_id=entry.ref_tag_id, tag_b_id=entry.path_tag_id,
            tag_a_size_m=self._resolve_ref_size(ref),
            tag_b_size_m=self.cfg.tag_b_size_m,
            family=self.cfg.tag_family,
        )
        T_A2B = out["T_A2B"]
        T_B_world = compute_T_B_world(ref.T_world, T_A2B)
        pos_m = T_B_world[:3, 3]
        rpy_deg = rot2rpy_deg(T_B_world[:3, :3])

        # 4. Persist the full 6-DOF result (image + K + transforms + CSV).
        save_locate_run(
            root_dir=self.cfg.save_dir,
            tag_b_id=entry.path_tag_id,
            image_hc=hc_img, image_fc=fc_img,
            K_hc=K_hc, K_fc=K_fc,
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
            }},
            success=True,
            message="ok",
        )

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
