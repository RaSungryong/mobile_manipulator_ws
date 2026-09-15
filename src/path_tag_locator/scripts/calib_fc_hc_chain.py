#!/usr/bin/env python3
"""
calib_fc_hc_chain.py — measure the front_cam <-> hand_cam chain against
two floor tags and fit a constant correction (2026-09-15).

Why: the locator chain T_A2B = inv(T_hc2A) · T_hc2ee · T_ee2ab · T_ab2mb ·
T_mb2fc · T_fc2B carries four platform matrices no session has checked
end to end. Two 90 mm tags laid on one floor with their edges collinear
(straightedge) and the centre distance measured give the TRUE T_A2B; the
hand camera then looks at tag A from many orientations while the base
stands still with tag B under front_cam, and every view is one equation
in the unknown corrections (path_tag_locator.chain_calib — the AX = YB
form: D on the hand side = a hand-eye error, F on the base side = an
arm-mount / front_cam / front-tag error; the rotation diversity of the
views is what tells them apart).

Preconditions
  * mobile_manipulator.launch up (arm_node, robot_camera_node,
    lifter_node); path_tag_locator.launch NOT running (it commands the
    arm too); nothing else moving the arm or the base.
  * tag B (default 150) on the floor in front_cam's view, tag A (default
    149) on the same floor within the arm's reach, both edges against ONE
    straightedge, spacing = (outer extent + inner gap) / 2 measured.
  * the operator has hand_cam over tag A (robot_ui Arm tab jog), roughly
    square, 0.35-0.55 m above it. `check` says whether both tags are seen.
  * a hand on the e-stop for `collect`: the arm sweeps ~20 views around
    tag A (the hand-eye sweep's planner and safety rules, tag plane =
    the floor) and returns to where it started.

Commands
  check                       both cameras see their tag; prints the raw
                              chain's T_A2B against the laid truth
  collect DIR --spacing d     the sweep; samples -> DIR/samples.npz (+ yaml)
  solve DIR                   raw / hand / base / joint fits, jackknife,
                              the corrected hand-eye and T_ab2mb;
                              --write-hand-eye writes config/hand_eye/
                              T_hc2ee_chain_<date>.npz (nothing else is
                              applied automatically)

    rosrun path_tag_locator calib_fc_hc_chain.py check --spacing 0.700
    rosrun path_tag_locator calib_fc_hc_chain.py collect log/path_tag_locator/chain_calib/<date> --spacing 0.700
    rosrun path_tag_locator calib_fc_hc_chain.py solve   log/path_tag_locator/chain_calib/<date>
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from path_tag_locator import WS_DIR                                   # noqa: E402
from path_tag_locator import chain_calib as CC                        # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                 # noqa: E402
from path_tag_locator.constants import load_extrinsics_full, load_locator_cfg  # noqa: E402
from path_tag_locator.geometry import rot2rpy_deg                     # noqa: E402
from path_tag_locator.hand_eye import load_T_hc2ee                    # noqa: E402

CFG_DIR = os.path.join(_HERE, "..", "config")
LOCATOR_YAML = os.path.join(CFG_DIR, "locator.yaml")
EXTRINSICS_YAML = os.path.join(CFG_DIR, "extrinsics.yaml")
DEFAULT_OUT = os.path.join(WS_DIR, "log", "path_tag_locator", "chain_calib")


# ----------------------------------------------------------------------
# persistence (numpy only, so solve needs no ROS)
# ----------------------------------------------------------------------
def save_samples(dir_, samples, meta):
    os.makedirs(dir_, exist_ok=True)
    np.savez(os.path.join(dir_, "samples.npz"),
             labels=np.array([s.label for s in samples]),
             tcp=np.array([s.tcp_pose_mm_deg for s in samples], dtype=float),
             lift=np.array([s.lift_height_m for s in samples], dtype=float),
             T_hc2A=np.array([s.T_hc2A for s in samples], dtype=float),
             T_fc2B=np.array([s.T_fc2B for s in samples], dtype=float))
    with open(os.path.join(dir_, "meta.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)


def load_samples(dir_):
    d = np.load(os.path.join(dir_, "samples.npz"))
    meta = yaml.safe_load(open(os.path.join(dir_, "meta.yaml")))
    samples = [CC.ChainSample(str(lab), list(map(float, tcp)), T1, T2, float(lift))
               for lab, tcp, lift, T1, T2 in zip(d["labels"], d["tcp"], d["lift"], d["T_hc2A"], d["T_fc2B"])]
    return samples, meta


def _resolve(p):
    """locator.yaml paths use roslaunch's $(find pkg); resolve without a
    master (this package's own dir) so solve works offline."""
    import re
    return re.sub(r"\$\(find path_tag_locator\)", os.path.normpath(os.path.join(_HERE, "..")), str(p))


def platform(cfg_path=LOCATOR_YAML, extrinsics=EXTRINSICS_YAML, hand_eye=None):
    cfg = load_locator_cfg(cfg_path)
    ext = load_extrinsics_full(extrinsics, front_cam_frame=getattr(cfg.detector, "front_cam_frame", "auto"))
    H = load_T_hc2ee(_resolve(hand_eye or cfg.hand_eye_npz))
    return cfg, ext, H


# ----------------------------------------------------------------------
# ROS side
# ----------------------------------------------------------------------
class Session:
    def __init__(self, cfg, hand_tag, front_tag, frames):
        import rospy
        from path_tag_locator.arm_interface import ArmInterface
        from path_tag_locator.lift_listener import LiftHeightListener
        from path_tag_locator.ros_image import grab_K
        self.rospy = rospy
        self.cfg = cfg
        self.hand_tag, self.front_tag, self.frames = int(hand_tag), int(front_tag), int(frames)
        self.arm = ArmInterface(state_topic=cfg.arm.state_topic, move_cart_topic=cfg.arm.move_cart_topic,
                                home_service=cfg.arm.home_service, motion_timeout_s=cfg.arm.motion_timeout_s)
        self.lift = LiftHeightListener()
        self.K_front = grab_K(cfg.topics.front_cam_info, timeout=5.0)
        self.K_hand = grab_K(cfg.topics.hand_cam_info, timeout=5.0)

    # --- observations, through the SAME functions the locator uses ---
    def hand_T(self, n=None, pick="mean"):
        from path_tag_locator.detections import (detection_to_T_cam2tag, mean_detection,
                                                 median_tilt_detection, wait_for_tag_detections)
        dets = wait_for_tag_detections(self.cfg.topics.hand_cam_detections, self.hand_tag,
                                       n or self.frames, timeout=4.0)
        det = mean_detection(dets) if pick == "mean" else median_tilt_detection(dets)
        return detection_to_T_cam2tag(det, float(self.cfg.tag.tag_a_size_m),
                                      float(self.cfg.detector.hand_cam_tag_size_m)), len(dets)

    def front_T(self, n=None):
        from path_tag_locator.detections import (detection_to_T_cam2tag, mean_detection,
                                                 wait_for_tag_detections)
        dets = wait_for_tag_detections(self.cfg.topics.front_cam_detections, self.front_tag,
                                       n or self.frames, timeout=4.0)
        det = mean_detection(dets)
        K = self.K_front if getattr(self.cfg.detector, "front_cam_repose_from_corners", True) else None
        return detection_to_T_cam2tag(det, float(self.cfg.tag.tag_b_size_m),
                                      float(self.cfg.detector.front_cam_tag_size_m), camera_K=K), len(dets)

    def check(self, ext, H, spacing):
        ok, why = self.arm.wait_for_node(10.0)
        if not ok:
            print("FAIL: %s" % why); return False
        tcp = self.arm.get_tcp_pose()
        lift = self.lift.height_m() or 0.0
        print("arm TCP (mm, deg): %s   lift %.3f m" % (["%.1f" % v for v in tcp], lift))
        print("extrinsics: %s" % ext.note)
        try:
            T_hc2A, n = self.hand_T(n=5)
        except Exception as e:
            print("FAIL: hand_cam does not see tag %d: %s" % (self.hand_tag, e)); return False
        m = CC.pose_error(T_hc2A, np.eye(4))
        print("hand_cam tag %d: %d frames, range %.3f m, xy offset (%+.0f, %+.0f) mm, tilt %.1f deg"
              % (self.hand_tag, n, T_hc2A[2, 3], T_hc2A[0, 3] * 1e3, T_hc2A[1, 3] * 1e3,
                 math.degrees(math.acos(max(-1.0, min(1.0, T_hc2A[2, 2]))))))
        if not 0.30 <= T_hc2A[2, 3] <= 0.65:
            print("  ! bring the camera 0.35-0.55 m above the tag before collect")
        try:
            T_fc2B, n = self.front_T(n=5)
        except Exception as e:
            print("FAIL: front_cam does not see tag %d: %s" % (self.front_tag, e)); return False
        print("front_cam tag %d: %d frames, at (%+.3f, %+.3f) m, depth %.3f m" % (
            self.front_tag, n, T_fc2B[0, 3], T_fc2B[1, 3], T_fc2B[2, 3]))
        s = CC.ChainSample("check", tcp, T_hc2A, T_fc2B, lift)
        raw = CC.raw_chain_T_A2B([s], H, ext.T_ab2mb, ext.T_mb2fc_chain, compensate_T_ab2mb)[0]
        print("raw chain T_A2B: %s" % CC.describe_T(raw, "B in A"))
        if spacing:
            tr = CC.snap_truth(raw, spacing)
            print("laid truth (snapped, B %d quarter turns vs A, %d quarter turns along A's x): %s" % (
                tr.k90, tr.a90, CC.describe_T(tr.T_A2B, "B in A")))
            print("-> raw chain error at this pose: %.1f mm / %.2f deg" % (tr.raw_pos_err_m * 1e3, tr.raw_rot_err_deg))
        print("OK")
        return True

    def collect(self, ext, H, spacing, out_dir, sweep_cfg, dry_run, yes, log):
        from path_tag_locator.handeye_sweep import SweepRunner
        if os.path.isdir(out_dir) and os.path.exists(os.path.join(out_dir, "samples.npz")):
            sys.exit("refusing to overwrite %s" % out_dir)
        if not self.check(ext, H, spacing):
            sys.exit(2)
        print("\nPlan: square up over tag %d, then up to %d views (distances %s m, tilts %s deg, spins %s deg), "
              "%d frames per camera per view, back to the start pose. Output: %s" % (
                  self.hand_tag, sweep_cfg.max_samples, sweep_cfg.distances_m, sweep_cfg.tilts_deg,
                  sweep_cfg.spins_deg, self.frames, out_dir))
        if dry_run:
            return
        if not yes:
            if input("Hand on the e-stop? The ARM will move now. Type 'go' to start: ").strip().lower() != 'go':
                sys.exit("aborted")
        samples = []
        rospy = self.rospy

        def capture():
            try:
                tcp = self.arm.get_tcp_pose()
                rospy.sleep(0.3)
                T_hc2A, n1 = self.hand_T()
                T_fc2B, n2 = self.front_T()
                lift = self.lift.height_m() or 0.0
                samples.append(CC.ChainSample("v%02d" % len(samples), list(tcp), T_hc2A, T_fc2B, lift))
                log("  sample %d: hand %d frames, front %d frames, lift %.3f" % (len(samples), n1, n2, lift))
                return True, "ok"
            except Exception as e:
                return False, str(e)

        runner = SweepRunner(sweep_cfg, get_tcp=self.arm.get_tcp_pose,
                             move=lambda pose: self.arm.move_j_to_pose(pose, linear=True),
                             detect=lambda: self._detect_or_none(), capture=capture,
                             log_info=log, log_warn=log)
        res = runner.run(H)
        log(res.summary())
        if not samples:
            sys.exit("no samples")
        meta = dict(date=time.strftime("%Y-%m-%d %H:%M:%S"), hand_tag=self.hand_tag, front_tag=self.front_tag,
                    spacing_m=float(spacing), frames_per_sample=self.frames,
                    hand_eye_npz=_resolve(self.cfg.hand_eye_npz), extrinsics_note=str(ext.note),
                    K_front=[float(v) for v in np.asarray(self.K_front).ravel()],
                    K_hand=[float(v) for v in np.asarray(self.K_hand).ravel()],
                    sweep=res.summary(), rotation_diversity_deg=float(CC.rotation_diversity_deg(samples)))
        save_samples(out_dir, samples, meta)
        log("saved %d samples -> %s (rotation diversity %.0f deg)" % (len(samples), out_dir, meta["rotation_diversity_deg"]))

    def _detect_or_none(self):
        try:
            return self.hand_T(n=5, pick="median")[0]
        except Exception:
            return None


# ----------------------------------------------------------------------
# commands
# ----------------------------------------------------------------------
def cmd_check(args):
    import rospy
    rospy.init_node("calib_fc_hc_chain", anonymous=True)
    cfg, ext, H = platform(hand_eye=args.hand_eye)
    S = Session(cfg, args.hand_tag, args.front_tag, args.frames)
    sys.exit(0 if S.check(ext, H, args.spacing) else 2)


def cmd_collect(args):
    import rospy
    from path_tag_locator.handeye_sweep import SweepCfg
    rospy.init_node("calib_fc_hc_chain", anonymous=True)
    if not args.spacing:
        sys.exit("collect needs --spacing (measured centre distance, m)")
    cfg, ext, H = platform(hand_eye=args.hand_eye)
    S = Session(cfg, args.hand_tag, args.front_tag, args.frames)
    sweep = SweepCfg(distances_m=list(args.distances), tilts_deg=list(args.tilts),
                     azimuths_deg=[0.0, 90.0, 180.0, 270.0], spins_deg=list(args.spins),
                     max_samples=int(args.max_samples), min_clearance_m=float(args.clearance),
                     max_xy_from_start_m=float(args.max_xy))
    S.collect(ext, H, args.spacing, args.dir, sweep, args.dry_run, args.yes, log=lambda m: print(m, flush=True))


def cmd_solve(args):
    samples, meta = load_samples(args.dir)
    cfg, ext, H = platform(hand_eye=args.hand_eye or meta.get("hand_eye_npz"))
    spacing = args.spacing or float(meta["spacing_m"])
    print("%d samples from %s (tags A=%s under hand_cam, B=%s under front_cam, spacing %.4f m, %d frames/sample, "
          "rotation diversity %.0f deg)" % (len(samples), meta.get("date"), meta.get("hand_tag"), meta.get("front_tag"),
                                            spacing, meta.get("frames_per_sample", 0), CC.rotation_diversity_deg(samples)))
    print("hand-eye: %s\nextrinsics: %s" % (_resolve(args.hand_eye or meta.get("hand_eye_npz")), ext.note))
    # Outliers: a sample taken while the arm was being stopped or pushed
    # (2026-09-15: the collision samples read 414 mm against 10 mm for the
    # rest) would dominate every fit. Drop --exclude'd labels and anything
    # whose RAW residual is > outlier_factor x the median, then re-solve.
    if args.exclude:
        samples = [s for s in samples if s.label not in set(args.exclude)]
    truth0, res0 = CC.fit_corrections(samples, H, ext.T_ab2mb, ext.T_mb2fc_chain, spacing, compensate_T_ab2mb, jackknife=False)
    med = float(np.median([dp for _, dp, _ in res0['raw'].per_sample]))
    bad = [lab for lab, dp, dr in res0['raw'].per_sample if dp > args.outlier_factor * med]
    if bad:
        print("\nexcluding %d outlier sample(s) whose raw residual exceeds %.0fx the median (%.1f mm): %s"
              % (len(bad), args.outlier_factor, med * 1e3, ", ".join(bad)))
        samples = [s for s in samples if s.label not in set(bad)]
    if len(samples) < 4:
        sys.exit("only %d usable samples" % len(samples))
    print("%d samples used, rotation diversity %.0f deg" % (len(samples), CC.rotation_diversity_deg(samples)))
    truth, res = CC.fit_corrections(samples, H, ext.T_ab2mb, ext.T_mb2fc_chain, spacing, compensate_T_ab2mb)
    print("\nlaid truth (snapped: B %d quarter turns vs A, A->B %d quarter turns along A's x): %s"
          % (truth.k90, truth.a90, CC.describe_T(truth.T_A2B, "B in A")))
    print("\n%-6s %-22s %-22s   what it means" % ("fit", "rms (mm / deg)", "max (mm / deg)"))
    notes = {'raw': "the chain as configured", 'hand': "T_hc2ee' = T_hc2ee . D  (hand-eye error)",
             'base': "T_ab2mb' = T_ab2mb . T_mb2fc . F . inv(T_mb2fc)  (mount / front_cam / front tag)",
             'joint': "both"}
    for k in ('raw', 'hand', 'base', 'joint'):
        r = res[k]
        print("%-6s %6.2f / %-13.3f %6.2f / %-13.3f   %s" % (k, r.rms_pos_m * 1e3, r.rms_rot_deg, r.max_pos_m * 1e3, r.max_rot_deg, notes[k]))
    print("\nper-sample residual of the RAW chain (mm / deg):")
    for lab, dp, dr in res['raw'].per_sample:
        print("  %-8s %6.2f / %6.3f" % (lab, dp * 1e3, dr))
    # attribution
    floor = 4e-3
    h, b, j = res['hand'], res['base'], res['joint']
    if h.rms_pos_m <= max(floor, 1.5 * j.rms_pos_m) and h.rms_pos_m < 0.5 * b.rms_pos_m:
        verdict = "HAND side: the hand-eye explains the residual; the base side adds nothing"
    elif b.rms_pos_m <= max(floor, 1.5 * j.rms_pos_m) and b.rms_pos_m < 0.5 * h.rms_pos_m:
        verdict = "BASE side: an arm-mount / front_cam / front-tag error explains it; the hand-eye is fine"
    elif j.rms_pos_m < 0.5 * min(h.rms_pos_m, b.rms_pos_m):
        verdict = "BOTH sides carry error (only the joint fit reaches the floor)"
    elif abs(h.rms_pos_m - b.rms_pos_m) < 0.3 * max(h.rms_pos_m, b.rms_pos_m):
        verdict = "UNDETERMINED: hand and base fits are equally good — not enough rotation diversity in the views"
    else:
        verdict = "mixed — read the table"
    print("\nverdict: %s" % verdict)
    for k in ('hand', 'base', 'joint'):
        r = res[k]
        print("\n[%s]" % k)
        if k != 'base':
            print("  " + CC.describe_T(r.D, "D (ee frame)"))
            if r.jackknife_D_sd is not None:
                print("    jackknife sd: rot %s deg, t %s mm" % (np.round(np.degrees(r.jackknife_D_sd[:3]), 3), np.round(r.jackknife_D_sd[3:] * 1e3, 2)))
            Hc = CC.corrected_hand_eye(H, r.D)
            print("  " + CC.describe_T(Hc, "corrected T_hc2ee"))
        if k != 'hand':
            print("  " + CC.describe_T(r.F, "F (fc frame)"))
            if r.jackknife_F_sd is not None:
                print("    jackknife sd: rot %s deg, t %s mm" % (np.round(np.degrees(r.jackknife_F_sd[:3]), 3), np.round(r.jackknife_F_sd[3:] * 1e3, 2)))
            Tc = CC.corrected_T_ab2mb(ext.T_ab2mb, ext.T_mb2fc_chain, r.F)
            print("  " + CC.describe_T(Tc, "corrected T_ab2mb") + "   (current: %s)" % CC.describe_T(ext.T_ab2mb, "")[2:])
    if args.write_hand_eye:
        k = args.write_hand_eye
        out = os.path.join(CFG_DIR, "hand_eye", "T_hc2ee_chain_%s.npz" % time.strftime("%Y%m%d_%H%M%S"))
        np.savez(out, CC.corrected_hand_eye(H, res[k].D))
        print("\nwrote %s (from the '%s' fit) — point locator.yaml hand_eye.npz_path at it to use it" % (out, k))
    np.savez(os.path.join(args.dir, "corrections.npz"),
             D_hand=res['hand'].D, F_base=res['base'].F, D_joint=res['joint'].D, F_joint=res['joint'].F,
             T_A2B_true=truth.T_A2B)
    print("\ncorrections saved -> %s/corrections.npz" % args.dir)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand-tag", type=int, default=149, help="tag under hand_cam (A)")
    ap.add_argument("--front-tag", type=int, default=150, help="tag under front_cam (B)")
    ap.add_argument("--spacing", type=float, default=None, help="measured centre distance A -> B (m)")
    ap.add_argument("--frames", type=int, default=20, help="frames averaged per camera per sample")
    ap.add_argument("--hand-eye", default=None, help="hand-eye npz to evaluate (default locator.yaml's)")
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("check").set_defaults(fn=cmd_check)
    p = sp.add_parser("collect"); p.add_argument("dir")
    p.add_argument("--max-samples", type=int, default=20)
    p.add_argument("--distances", type=float, nargs="+", default=[0.40, 0.50])
    p.add_argument("--tilts", type=float, nargs="+", default=[0.0, 12.0, 22.0])
    p.add_argument("--spins", type=float, nargs="+", default=[0.0, 90.0, 180.0, -90.0])
    p.add_argument("--clearance", type=float, default=0.12, help="min height of any tool point above the floor (m)")
    p.add_argument("--max-xy", type=float, default=0.30, help="flange stays within this of the start pose (m)")
    p.add_argument("--dry-run", action="store_true"); p.add_argument("--yes", action="store_true")
    p.set_defaults(fn=cmd_collect)
    p = sp.add_parser("solve"); p.add_argument("dir")
    p.add_argument("--exclude", nargs="*", default=None, help="sample labels to drop (e.g. v09 v10)")
    p.add_argument("--outlier-factor", type=float, default=4.0,
                   help="drop samples whose raw residual is more than this times the median")
    p.add_argument("--write-hand-eye", choices=["hand", "joint"], default=None,
                   help="write config/hand_eye/T_hc2ee_chain_<date>.npz from this fit's D")
    p.set_defaults(fn=cmd_solve)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
