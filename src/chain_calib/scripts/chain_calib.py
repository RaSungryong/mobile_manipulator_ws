#!/usr/bin/env python3
"""
chain_calib.py — front_cam <-> hand_cam chain calibration, operator-jogged.

The operator jogs the arm (robot_ui Arm tab) so hand_cam looks at tag A
from a view, runs `capture`, jogs to the next view, captures again. No
automatic arm motion — the automatic sweep that existed on 2026-09-15
collided the arm with the base and is gone. Tag B stays under front_cam
and the base does not move for the whole session.

    rosrun chain_calib chain_calib.py check   --spacing 1.01025
    rosrun chain_calib chain_calib.py capture log/chain_calib/<date> --spacing 1.01025
    rosrun chain_calib chain_calib.py status  log/chain_calib/<date>
    rosrun chain_calib chain_calib.py solve   log/chain_calib/<date> [--write-hand-eye hand]

Defaults: tag A = 150 under hand_cam, tag B = 149 under front_cam
(--hand-tag / --front-tag), 20 frames per camera per capture, session
directories under log/chain_calib/. README.md in this package is the
operator guide.
"""
import argparse
import math
import os
import re
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from chain_calib import solver as CC                                   # noqa: E402
from chain_calib.session import (azimuth_word, coverage, describe_view, load_samples,  # noqa: E402
                                 new_meta, next_label, save_samples)
from path_tag_locator import WS_DIR                                    # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                  # noqa: E402
from path_tag_locator.constants import load_extrinsics_full, load_locator_cfg  # noqa: E402
from path_tag_locator.hand_eye import load_T_hc2ee                     # noqa: E402

PTL_CFG = None      # resolved lazily: path_tag_locator/config
DEFAULT_OUT = os.path.join(WS_DIR, "log", "chain_calib")


def _ptl_cfg_dir():
    """path_tag_locator/config — via rospkg when a master / env is up, else
    the source tree next to this package (the devel-space import of the
    module lands in devel/lib, which holds no config)."""
    global PTL_CFG
    if PTL_CFG is None:
        cand = os.path.normpath(os.path.join(_HERE, "..", "..", "path_tag_locator", "config"))
        if not os.path.isdir(cand):
            try:
                import rospkg
                cand = os.path.join(rospkg.RosPack().get_path("path_tag_locator"), "config")
            except Exception:
                pass
        PTL_CFG = cand
    return PTL_CFG


def _resolve(p):
    """locator.yaml paths use roslaunch's $(find pkg); resolve without a master."""
    return re.sub(r"\$\(find path_tag_locator\)", os.path.normpath(os.path.join(_ptl_cfg_dir(), "..")), str(p))


def platform(hand_eye=None):
    cfg = load_locator_cfg(os.path.join(_ptl_cfg_dir(), "locator.yaml"))
    ext = load_extrinsics_full(os.path.join(_ptl_cfg_dir(), "extrinsics.yaml"),
                               front_cam_frame=getattr(cfg.detector, "front_cam_frame", "auto"))
    H = load_T_hc2ee(_resolve(hand_eye or cfg.hand_eye_npz))
    return cfg, ext, H


# ----------------------------------------------------------------------
# ROS side: read the arm and the two detectors, nothing else
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

    def hand_T(self, n=None):
        from path_tag_locator.detections import (detection_to_T_cam2tag, mean_detection,
                                                 wait_for_tag_detections)
        dets = wait_for_tag_detections(self.cfg.topics.hand_cam_detections, self.hand_tag,
                                       n or self.frames, timeout=4.0)
        det = mean_detection(dets)
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

    def one_sample(self, label):
        """One capture at the current pose: arm state + both cameras."""
        ok, why = self.arm.wait_for_node(10.0)
        if not ok:
            raise RuntimeError(why)
        tcp = self.arm.get_tcp_pose()
        lift = self.lift.height_m() or 0.0
        T_hc2A, n1 = self.hand_T()
        T_fc2B, n2 = self.front_T()
        return CC.ChainSample(label, list(map(float, tcp)), T_hc2A, T_fc2B, float(lift)), n1, n2


def raw_error_at(sample, H, ext, spacing):
    raw = CC.raw_chain_T_A2B([sample], H, ext.T_ab2mb, ext.T_mb2fc_chain, compensate_T_ab2mb)[0]
    tr = CC.snap_truth(raw, spacing)
    E = CC.invert_T(tr.T_A2B) @ raw
    return raw, tr, E[:3, 3] * 1e3


def print_sample(sample, n1, n2, H, ext, spacing):
    v = describe_view(sample.T_hc2A)
    print("  hand_cam tag: %d frames, range %.3f m, %s, spin %+.0f deg, tag %.0f mm off-axis"
          % (n1, v.range_m, azimuth_word(v), v.spin_deg, v.xy_offset_mm))
    print("  front_cam tag: %d frames, at (%+.3f, %+.3f) m" % (n2, sample.T_fc2B[0, 3], sample.T_fc2B[1, 3]))
    print("  lift %.3f m, TCP %s" % (sample.lift_height_m, ["%.1f" % x for x in sample.tcp_pose_mm_deg]))
    if spacing:
        raw, tr, e = raw_error_at(sample, H, ext, spacing)
        print("  raw chain error at this view: %.1f mm / %.2f deg  (along the tags' line %+.1f, across %+.1f, height %+.1f mm)"
              % (tr.raw_pos_err_m * 1e3, tr.raw_rot_err_deg, e[0], e[1], e[2]))


# ----------------------------------------------------------------------
# commands
# ----------------------------------------------------------------------
def cmd_check(args):
    import rospy
    rospy.init_node("chain_calib", anonymous=True)
    cfg, ext, H = platform(hand_eye=args.hand_eye)
    S = Session(cfg, args.hand_tag, args.front_tag, args.frames)
    print("extrinsics: %s" % ext.note)
    try:
        s, n1, n2 = S.one_sample("check")
    except Exception as e:
        print("FAIL: %s" % e)
        sys.exit(2)
    print_sample(s, n1, n2, H, ext, args.spacing)
    v = describe_view(s.T_hc2A)
    if not 0.30 <= v.range_m <= 0.65:
        print("  ! the camera should be 0.35-0.55 m from the tag")
    print("OK")


def cmd_capture(args):
    import rospy
    rospy.init_node("chain_calib", anonymous=True)
    if not args.spacing:
        sys.exit("capture needs --spacing (measured centre distance, m) — the same value for every capture of a session")
    cfg, ext, H = platform(hand_eye=args.hand_eye)
    samples, meta = load_samples(args.dir)
    if meta:
        for k, v in (("hand_tag", args.hand_tag), ("front_tag", args.front_tag)):
            if int(meta.get(k, v)) != int(v):
                sys.exit("%s: this session was started with %s=%s" % (args.dir, k, meta.get(k)))
        if abs(float(meta.get("spacing_m", args.spacing)) - float(args.spacing)) > 1e-6:
            sys.exit("%s: this session was started with spacing %.5f" % (args.dir, meta.get("spacing_m")))
    S = Session(cfg, args.hand_tag, args.front_tag, args.frames)
    if not meta:
        meta = new_meta(args.hand_tag, args.front_tag, args.spacing, args.frames, _resolve(cfg.hand_eye_npz),
                        ext.note, S.K_front, S.K_hand)
    label = args.label or next_label(samples)
    rospy.sleep(0.3)                                     # let the jog settle
    try:
        s, n1, n2 = S.one_sample(label)
    except Exception as e:
        sys.exit("capture failed: %s" % e)
    print("sample %s:" % label)
    print_sample(s, n1, n2, H, ext, args.spacing)
    v = describe_view(s.T_hc2A)
    if v.xy_offset_mm > 60:
        print("  ! tag %.0f mm off the optical axis — fine, but keep it inside the frame" % v.xy_offset_mm)
    samples.append(s)
    save_samples(args.dir, samples, meta)
    ok, lines = coverage(samples)
    print("coverage: " + lines[0])
    print("          " + lines[1])
    print("saved -> %s (%d samples)" % (args.dir, len(samples)))


def cmd_status(args):
    samples, meta = load_samples(args.dir)
    if not samples:
        sys.exit("no samples in %s" % args.dir)
    cfg, ext, H = platform(hand_eye=args.hand_eye or meta.get("hand_eye_npz"))
    spacing = args.spacing or float(meta["spacing_m"])
    print("%d samples, tags A=%s (hand_cam) B=%s (front_cam), spacing %.5f m" % (
        len(samples), meta.get("hand_tag"), meta.get("front_tag"), spacing))
    for s in samples:
        v = describe_view(s.T_hc2A)
        raw, tr, e = raw_error_at(s, H, ext, spacing)
        print("  %-5s range %.2f  %-46s spin %+4.0f  raw err %5.1f mm / %.2f deg (h %+.1f)"
              % (s.label, v.range_m, azimuth_word(v), v.spin_deg, tr.raw_pos_err_m * 1e3, tr.raw_rot_err_deg, e[2]))
    ok, lines = coverage(samples)
    print("coverage: " + lines[0]); print("          " + lines[1])


def cmd_drop(args):
    samples, meta = load_samples(args.dir)
    keep = [s for s in samples if s.label not in set(args.labels)]
    if len(keep) == len(samples):
        sys.exit("no such label(s)")
    save_samples(args.dir, keep, meta)
    print("dropped %s; %d samples left" % (", ".join(args.labels), len(keep)))


def cmd_solve(args):
    samples, meta = load_samples(args.dir)
    if not samples:
        sys.exit("no samples in %s" % args.dir)
    cfg, ext, H = platform(hand_eye=args.hand_eye or meta.get("hand_eye_npz"))
    spacing = args.spacing or float(meta["spacing_m"])
    if args.exclude:
        samples = [s for s in samples if s.label not in set(args.exclude)]
    print("%d samples from %s (tags A=%s under hand_cam, B=%s under front_cam, spacing %.5f m, %d frames/sample)"
          % (len(samples), meta.get("date"), meta.get("hand_tag"), meta.get("front_tag"), spacing,
             meta.get("frames_per_sample", 0)))
    print("hand-eye: %s\nextrinsics: %s" % (_resolve(args.hand_eye or meta.get("hand_eye_npz")), ext.note))
    ok, lines = coverage(samples)
    print("coverage: " + lines[0]); print("          " + lines[1])
    # outliers (a sample taken while the arm was moving / pushed) would dominate every fit
    truth0, res0 = CC.fit_corrections(samples, H, ext.T_ab2mb, ext.T_mb2fc_chain, spacing, compensate_T_ab2mb, jackknife=False)
    med = float(np.median([dp for _, dp, _ in res0['raw'].per_sample]))
    bad = [lab for lab, dp, dr in res0['raw'].per_sample if dp > args.outlier_factor * med]
    if bad:
        print("excluding %d outlier(s) whose raw residual exceeds %.0fx the median (%.1f mm): %s"
              % (len(bad), args.outlier_factor, med * 1e3, ", ".join(bad)))
        samples = [s for s in samples if s.label not in set(bad)]
    if len(samples) < 4:
        sys.exit("only %d usable samples" % len(samples))
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
    print("\nper-sample residual of the RAW chain (mm / deg), with the view:")
    for (lab, dp, dr), s in zip(res['raw'].per_sample, samples):
        v = describe_view(s.T_hc2A)
        print("  %-6s %6.2f / %6.3f   %s, spin %+.0f" % (lab, dp * 1e3, dr, azimuth_word(v), v.spin_deg))
    floor = 4e-3
    h, b, j = res['hand'], res['base'], res['joint']
    if h.rms_pos_m <= max(floor, 1.5 * j.rms_pos_m) and h.rms_pos_m < 0.5 * b.rms_pos_m:
        verdict = "HAND side: the hand-eye explains the residual; the base side adds nothing"
    elif b.rms_pos_m <= max(floor, 1.5 * j.rms_pos_m) and b.rms_pos_m < 0.5 * h.rms_pos_m:
        verdict = "BASE side: an arm-mount / front_cam / front-tag error explains it; the hand-eye is fine"
    elif j.rms_pos_m < 0.5 * min(h.rms_pos_m, b.rms_pos_m):
        verdict = "BOTH sides carry error (only the joint fit reaches the floor)"
    elif abs(h.rms_pos_m - b.rms_pos_m) < 0.3 * max(h.rms_pos_m, b.rms_pos_m):
        verdict = "UNDETERMINED: hand and base fits are equally good — capture more TILTED views (see coverage)"
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
            print("  " + CC.describe_T(CC.corrected_hand_eye(H, r.D), "corrected T_hc2ee"))
        if k != 'hand':
            print("  " + CC.describe_T(r.F, "F (fc frame)"))
            if r.jackknife_F_sd is not None:
                print("    jackknife sd: rot %s deg, t %s mm" % (np.round(np.degrees(r.jackknife_F_sd[:3]), 3), np.round(r.jackknife_F_sd[3:] * 1e3, 2)))
            Tc = CC.corrected_T_ab2mb(ext.T_ab2mb, ext.T_mb2fc_chain, r.F)
            print("  " + CC.describe_T(Tc, "corrected T_ab2mb") + "   (current: %s)" % CC.describe_T(ext.T_ab2mb, "")[2:])
    if args.write_hand_eye:
        k = args.write_hand_eye
        out = os.path.join(_ptl_cfg_dir(), "hand_eye", "T_hc2ee_chain_%s.npz" % time.strftime("%Y%m%d_%H%M%S"))
        np.savez(out, CC.corrected_hand_eye(H, res[k].D))
        print("\nwrote %s (from the '%s' fit) — point locator.yaml hand_eye.npz_path at it to use it" % (out, k))
    np.savez(os.path.join(args.dir, "corrections.npz"),
             D_hand=res['hand'].D, F_base=res['base'].F, D_joint=res['joint'].D, F_joint=res['joint'].F,
             T_A2B_true=truth.T_A2B)
    print("\ncorrections saved -> %s/corrections.npz" % args.dir)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hand-tag", type=int, default=150, help="tag A, under hand_cam")
    ap.add_argument("--front-tag", type=int, default=149, help="tag B, under front_cam")
    ap.add_argument("--spacing", type=float, default=None, help="measured centre-to-centre distance A-B (m)")
    ap.add_argument("--frames", type=int, default=20, help="frames averaged per camera per capture")
    ap.add_argument("--hand-eye", default=None, help="hand-eye npz to evaluate (default: locator.yaml's)")
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("check", help="read both cameras and the arm once; nothing saved").set_defaults(fn=cmd_check)
    p = sp.add_parser("capture", help="save one sample at the current arm pose"); p.add_argument("dir")
    p.add_argument("--label", default=None); p.set_defaults(fn=cmd_capture)
    p = sp.add_parser("status", help="list the session's samples and coverage"); p.add_argument("dir"); p.set_defaults(fn=cmd_status)
    p = sp.add_parser("drop", help="remove samples by label"); p.add_argument("dir"); p.add_argument("labels", nargs="+"); p.set_defaults(fn=cmd_drop)
    p = sp.add_parser("solve", help="fit the corrections"); p.add_argument("dir")
    p.add_argument("--exclude", nargs="*", default=None, help="sample labels to drop")
    p.add_argument("--outlier-factor", type=float, default=4.0)
    p.add_argument("--write-hand-eye", choices=["hand", "joint"], default=None,
                   help="write path_tag_locator/config/hand_eye/T_hc2ee_chain_<date>.npz from this fit's D")
    p.set_defaults(fn=cmd_solve)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
