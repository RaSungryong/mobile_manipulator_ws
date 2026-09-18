#!/usr/bin/env python3
"""
chain_calib.py — front_cam <-> hand_cam chain calibration against the
printed A0 tag sheet, operator-jogged.

The sheet (sheet/A0_landscape_tag200_300-309_5x2_FINAL2.pdf + its layout
JSON) lies on the floor; front_cam sees some of its tags, hand_cam sees
others. The operator jogs the arm (robot_ui Arm tab) so hand_cam looks at
the sheet from a view, runs `capture`, jogs to the next view, captures
again. No automatic arm motion — the automatic sweep that existed on
2026-09-15 collided the arm with the base and is gone. The base does not
move for the whole session.

    rosrun chain_calib chain_calib.py check
    rosrun chain_calib chain_calib.py --sx 1.0012 --sy 0.9991 --tag-size 0.0899 capture log/chain_calib/<date>
    rosrun chain_calib chain_calib.py status  log/chain_calib/<date>
    rosrun chain_calib chain_calib.py solve   log/chain_calib/<date> [--holdout-every 4] [--sx .. --sy .. --tag-size ..]
                                                                     [--write-hand-eye hand|joint]

--sx / --sy / --tag-size are the PRINT'S MEASURED scale (README §2). 20
frames per camera per capture by default; session directories under
log/chain_calib/. README.md in this package is the operator guide.
"""
import argparse
import os
import re
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from chain_calib import sheet as SH                                    # noqa: E402
from chain_calib import solver as CC                                   # noqa: E402
from chain_calib.session import (azimuth_word, coverage, load_samples, new_meta, next_label,  # noqa: E402
                                 sample_view, save_samples)
from path_tag_locator import WS_DIR                                    # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                  # noqa: E402
from path_tag_locator.constants import load_extrinsics_full, load_locator_cfg  # noqa: E402
from path_tag_locator.hand_eye import load_T_hc2ee                     # noqa: E402

PTL_CFG = None      # resolved lazily: path_tag_locator/config
DEFAULT_OUT = os.path.join(WS_DIR, "log", "chain_calib")
DEFAULT_SHEET = os.path.normpath(os.path.join(_HERE, "..", "sheet", "A0_landscape_tag200_300-309_5x2_FINAL2_layout.json"))


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


def sheet_from_args(args, meta=None):
    """The sheet with the print scale: command line first, then the
    session's meta, then the design values (with a warning)."""
    meta = meta or {}
    path = getattr(args, "sheet_json", None) or meta.get("sheet_json", DEFAULT_SHEET)
    if not os.path.exists(path) and path == DEFAULT_SHEET:      # install space: the source tree is elsewhere
        try:
            import rospkg
            path = os.path.join(rospkg.RosPack().get_path("chain_calib"), "sheet", os.path.basename(DEFAULT_SHEET))
        except Exception:
            pass
    sx = args.sx if args.sx is not None else float(meta.get("sheet_sx", 1.0))
    sy = args.sy if args.sy is not None else float(meta.get("sheet_sy", 1.0))
    ts = args.tag_size if args.tag_size is not None else meta.get("sheet_tag_size_m")
    sh = SH.load_sheet(path, sx, sy, ts)
    if abs(sx - 1.0) < 1e-12 and abs(sy - 1.0) < 1e-12 and (ts is None or abs(float(ts) - sh.design_tag_size_m) < 1e-12):
        print("! sheet scale is the DESIGN value (sx = sy = 1, tag %.1f mm) — measure the print and pass "
              "--sx --sy --tag-size (README §2); 0.2 %% of scale is 1-2 mm of chain error" % (sh.design_tag_size_m * 1e3))
    return sh


def resolve_samples(samples, meta, sheet):
    """Re-solve every sample's T_hc2W / T_fc2W from its stored corners
    with THIS sheet (scale may differ from capture time). K / D from meta."""
    K_hand = np.asarray(meta["K_hand"], float).reshape(3, 3)
    K_front = np.asarray(meta["K_front"], float).reshape(3, 3)
    D_hand = np.asarray(meta.get("D_hand") or [], float)
    D_front = None if meta.get("front_cam_frame", "level") == "level" else np.asarray(meta.get("D_front") or [], float)
    for s in samples:
        h = SH.multi_tag_pnp(s.hand_corners, sheet, K_hand, D_hand)
        f = SH.multi_tag_pnp(s.front_corners, sheet, K_front, D_front)
        s.T_hc2W, s.T_fc2W, s.hand_rms_px, s.front_rms_px = h.T_cam2W, f.T_cam2W, h.rms_px, f.rms_px
    return len(samples)


# ----------------------------------------------------------------------
# ROS side: read the arm and the two detectors, nothing else
# ----------------------------------------------------------------------
class Session:
    def __init__(self, cfg, sheet, front_frame, frames):
        import rospy
        from path_tag_locator.arm_interface import ArmInterface
        from path_tag_locator.lift_listener import LiftHeightListener
        self.rospy = rospy
        self.cfg, self.sheet, self.front_frame, self.frames = cfg, sheet, front_frame, int(frames)
        self.arm = ArmInterface(state_topic=cfg.arm.state_topic, move_cart_topic=cfg.arm.move_cart_topic,
                                home_service=cfg.arm.home_service, motion_timeout_s=cfg.arm.motion_timeout_s)
        self.lift = LiftHeightListener()
        self.K_front, self.D_front = self._grab_KD(cfg.topics.front_cam_info)
        self.K_hand, self.D_hand = self._grab_KD(cfg.topics.hand_cam_info)

    def _grab_KD(self, info_topic):
        from sensor_msgs.msg import CameraInfo
        msg = self.rospy.wait_for_message(info_topic, CameraInfo, timeout=5.0)
        K = np.asarray(msg.K, dtype=float).reshape(3, 3)
        D = np.asarray(msg.D, dtype=float).ravel() if msg.D else np.zeros(5)
        return K, D

    def collect_frames(self, topic, n, timeout=4.0):
        """``n`` detection frames from ``topic`` as [{tag id: (4,2) corners}]
        — EVERY tag in the frame, frames with none included (they count
        toward the per-tag visibility fraction)."""
        import threading
        from robot_msgs.msg import AprilTagDetectionArray
        frames, lock, done = [], threading.Lock(), threading.Event()

        def _cb(arr):
            fr = {int(d.id): np.asarray(d.corners, dtype=float).reshape(4, 2) for d in arr.detections}
            with lock:
                if len(frames) < n:
                    frames.append(fr)
                if len(frames) >= n:
                    done.set()
        sub = self.rospy.Subscriber(topic, AprilTagDetectionArray, _cb, queue_size=1)
        try:
            deadline = self.rospy.Time.now() + self.rospy.Duration(float(timeout))
            while not self.rospy.is_shutdown() and not done.wait(0.05):
                if self.rospy.Time.now() >= deadline:
                    break
        finally:
            sub.unregister()
        with lock:
            return list(frames)

    def camera_T(self, topic, K, D, what):
        """(PnPResult, {id: TagCorners}, n frames) for one camera over the sheet."""
        frames = self.collect_frames(topic, self.frames)
        if not frames:
            raise RuntimeError("%s: no detection frames on %s" % (what, topic))
        tags = SH.accumulate_frames(frames)
        if not tags:
            raise RuntimeError("%s does not see any tag (%d frames)" % (what, len(frames)))
        res = SH.multi_tag_pnp({k: t.corners_px for k, t in tags.items()}, self.sheet, K, D)
        return res, tags, len(frames)

    def one_sample(self, label):
        """One capture at the current pose: arm state + both cameras."""
        ok, why = self.arm.wait_for_node(10.0)
        if not ok:
            raise RuntimeError(why)
        tcp = self.arm.get_tcp_pose()
        lift = self.lift.height_m() or 0.0
        h, htags, nh = self.camera_T(self.cfg.topics.hand_cam_detections, self.K_hand, self.D_hand, "hand_cam")
        # front_cam: ground-plane-corrected corners are a distortion-free
        # virtual camera's pixels (D = 0); raw corners need the real D
        Df = None if self.front_frame == "level" else self.D_front
        f, ftags, nf = self.camera_T(self.cfg.topics.front_cam_detections, self.K_front, Df, "front_cam")
        s = CC.ChainSample(label, list(map(float, tcp)), h.T_cam2W, f.T_cam2W, float(lift),
                           hand_corners={k: t.corners_px for k, t in htags.items()},
                           front_corners={k: t.corners_px for k, t in ftags.items()},
                           hand_rms_px=h.rms_px, front_rms_px=f.rms_px)
        return s, dict(hand=h, front=f, hand_tags=htags, front_tags=ftags, n_hand=nh, n_front=nf)


def print_sample(sample, info, H, ext, sheet):
    h, f = info["hand"], info["front"]
    v = sample_view(sample, sheet)
    hstd = max(t.std_px for t in info["hand_tags"].values())
    fstd = max(t.std_px for t in info["front_tags"].values())
    print("  hand_cam : tags %s (%d/%d frames), PnP rms %.2f px, corner scatter %.2f px; range %.3f m, %s, spin %+.0f deg"
          % (h.tag_ids, min(t.n_frames for t in info["hand_tags"].values()), info["n_hand"], h.rms_px, hstd,
             v.range_m, azimuth_word(v), v.spin_deg))
    print("  front_cam: tags %s (%d/%d frames), PnP rms %.2f px, corner scatter %.2f px; sheet origin at (%+.3f, %+.3f, %.3f) m"
          % (f.tag_ids, min(t.n_frames for t in info["front_tags"].values()), info["n_front"], f.rms_px, fstd,
             f.T_cam2W[0, 3], f.T_cam2W[1, 3], f.T_cam2W[2, 3]))
    for r, name in ((h, "hand_cam"), (f, "front_cam")):
        for fl in r.flags:
            print("  ! %s: %s" % (name, fl))
    if hstd > 0.3 or fstd > 0.3:
        print("  ! corner scatter over the frames > 0.3 px — was the arm still moving? (capture again)")
    print("  lift %.3f m, TCP %s" % (sample.lift_height_m, ["%.1f" % x for x in sample.tcp_pose_mm_deg]))
    # the user's metric at this view: the chain's T_A2B vs the sheet's
    (lab, a, b, dp, dr, e) = CC.pair_errors([sample], sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain,
                                            lift_compensate=compensate_T_ab2mb)[0]
    print("  raw chain error at this view: tag %d (hand_cam) -> tag %d (front_cam): %.1f mm / %.2f deg  "
          "(in B's frame: x %+.1f, y %+.1f, z %+.1f mm)" % (a, b, dp * 1e3, dr, e[0], e[1], e[2]))


def _holdout_split(samples, args):
    labels = set(args.holdout or [])
    if args.holdout_every and args.holdout_every > 1:
        labels |= {s.label for i, s in enumerate(samples) if (i % args.holdout_every) == args.holdout_every - 1}
    train = [s for s in samples if s.label not in labels]
    test = [s for s in samples if s.label in labels]
    return train, test


# ----------------------------------------------------------------------
# commands
# ----------------------------------------------------------------------
def cmd_check(args):
    import rospy
    rospy.init_node("chain_calib", anonymous=True)
    cfg, ext, H = platform(hand_eye=args.hand_eye)
    sheet = sheet_from_args(args)
    print("extrinsics: %s" % ext.note)
    print(sheet.describe())
    S = Session(cfg, sheet, ext.front_cam_frame, args.frames)
    try:
        s, info = S.one_sample("check")
    except Exception as e:
        print("FAIL: %s" % e)
        sys.exit(2)
    print_sample(s, info, H, ext, sheet)
    v = sample_view(s, sheet)
    if not 0.30 <= v.range_m <= 0.65:
        print("  ! the camera should be 0.35-0.55 m from the sheet")
    print("OK")


def cmd_capture(args):
    import rospy
    rospy.init_node("chain_calib", anonymous=True)
    samples, meta = load_samples(args.dir)
    cfg, ext, H = platform(hand_eye=args.hand_eye)
    sheet = sheet_from_args(args, meta)
    if meta:
        if os.path.abspath(sheet.path) != meta.get("sheet_json"):
            sys.exit("%s: this session was started with sheet %s" % (args.dir, meta.get("sheet_json")))
        for k, v in (("sheet_sx", sheet.sx), ("sheet_sy", sheet.sy), ("sheet_tag_size_m", sheet.tag_size_m)):
            if abs(float(meta.get(k, v)) - float(v)) > 1e-9:
                print("! %s differs from the session's %s (%.6f): the stored value stays, `solve --sx/--sy/--tag-size` re-solves all samples"
                      % (k, meta.get(k), v))
                sheet = SH.load_sheet(sheet.path, meta["sheet_sx"], meta["sheet_sy"], meta["sheet_tag_size_m"])
                break
        if meta.get("front_cam_frame") != ext.front_cam_frame:
            sys.exit("%s: this session's front_cam detections were %s, now %s (ground_plane toggled?)"
                     % (args.dir, meta.get("front_cam_frame"), ext.front_cam_frame))
    S = Session(cfg, sheet, ext.front_cam_frame, args.frames)
    if not meta:
        meta = new_meta(sheet, args.frames, _resolve(cfg.hand_eye_npz), ext.note, ext.front_cam_frame,
                        S.K_front, S.D_front, S.K_hand, S.D_hand)
    label = args.label or next_label(samples)
    rospy.sleep(0.3)                                     # let the jog settle
    try:
        s, info = S.one_sample(label)
    except Exception as e:
        sys.exit("capture failed: %s" % e)
    print("sample %s:" % label)
    print_sample(s, info, H, ext, sheet)
    hard = [fl for fl in info["hand"].flags + info["front"].flags if "ambiguity" in fl or "behind" in fl]
    if hard and not args.force:
        sys.exit("NOT saved (%s) — move so the camera sees two tags, or --force" % "; ".join(hard))
    samples.append(s)
    save_samples(args.dir, samples, meta)
    ok, lines = coverage(samples, sheet=sheet)
    print("coverage: " + lines[0])
    print("          " + lines[1])
    print("saved -> %s (%d samples)" % (args.dir, len(samples)))


def cmd_status(args):
    samples, meta = load_samples(args.dir)
    if not samples:
        sys.exit("no samples in %s" % args.dir)
    cfg, ext, H = platform(hand_eye=args.hand_eye or meta.get("hand_eye_npz"))
    sheet = sheet_from_args(args, meta)
    resolve_samples(samples, meta, sheet)
    print("%d samples; %s" % (len(samples), sheet.describe()))
    errs = CC.pair_errors(samples, sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain, lift_compensate=compensate_T_ab2mb)
    for s, (lab, a, b, dp, dr, e) in zip(samples, errs):
        v = sample_view(s, sheet)
        print("  %-5s range %.2f  %-46s spin %+4.0f  hand %s front %s  raw err %d->%d %5.1f mm / %.2f deg (z %+.1f)"
              % (s.label, v.range_m, azimuth_word(v), v.spin_deg, sorted(s.hand_corners or []),
                 sorted(s.front_corners or []), a, b, dp * 1e3, dr, e[2]))
    ok, lines = coverage(samples, sheet=sheet)
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
    sheet = sheet_from_args(args, meta)
    n = resolve_samples(samples, meta, sheet)
    print("%d samples from %s (%d frames/sample); %d re-solved from stored corners"
          % (len(samples), meta.get("date"), meta.get("frames_per_sample", 0), n))
    print(sheet.describe())
    print("hand-eye: %s\nextrinsics: %s" % (_resolve(args.hand_eye or meta.get("hand_eye_npz")), ext.note))
    if args.exclude:
        samples = [s for s in samples if s.label not in set(args.exclude)]
    ok, lines = coverage(samples, sheet=sheet)
    print("coverage: " + lines[0]); print("          " + lines[1])
    # outliers (a sample taken while the arm was moving / pushed) would dominate every fit
    res0 = CC.fit_corrections(samples, H, ext.T_ab2mb, ext.T_mb2fc_chain, compensate_T_ab2mb, jackknife=False)
    med = float(np.median([dp for _, dp, _ in res0['raw'].per_sample]))
    bad = [lab for lab, dp, dr in res0['raw'].per_sample if dp > args.outlier_factor * med]
    if bad:
        print("excluding %d outlier(s) whose raw residual exceeds %.0fx the median (%.1f mm): %s"
              % (len(bad), args.outlier_factor, med * 1e3, ", ".join(bad)))
        samples = [s for s in samples if s.label not in set(bad)]
    train, test = _holdout_split(samples, args)
    if len(train) < 4:
        sys.exit("only %d usable training samples" % len(train))
    if test:
        print("hold-out: %d sample(s) kept out of the fit: %s" % (len(test), ", ".join(s.label for s in test)))
    res = CC.fit_corrections(train, H, ext.T_ab2mb, ext.T_mb2fc_chain, compensate_T_ab2mb)

    print("\n%-6s %-22s %-22s   what it means" % ("fit", "rms (mm / deg)", "max (mm / deg)"))
    notes = {'raw': "the chain as configured (T_hc2fc error per view)", 'hand': "T_hc2ee' = T_hc2ee . D  (hand-eye error)",
             'base': "T_ab2mb' = T_ab2mb . T_mb2fc . F . inv(T_mb2fc)  (mount / front_cam)", 'joint': "both"}
    for k in ('raw', 'hand', 'base', 'joint'):
        r = res[k]
        line = "%-6s %6.2f / %-13.3f %6.2f / %-13.3f   %s" % (k, r.rms_pos_m * 1e3, r.rms_rot_deg, r.max_pos_m * 1e3, r.max_rot_deg, notes[k])
        if test:
            ho = CC.evaluate(r.D, r.F, test, H, ext.T_ab2mb, ext.T_mb2fc_chain, compensate_T_ab2mb)
            line += "   | hold-out %6.2f mm / %.3f deg" % (ho.rms_pos_m * 1e3, ho.rms_rot_deg)
        print(line)

    print("\nper-sample residual of the RAW chain (mm / deg), PnP rms (hand / front px), and the view:")
    views = [sample_view(s, sheet) for s in train]
    for (lab, dp, dr), s, v in zip(res['raw'].per_sample, train, views):
        print("  %-6s %6.2f / %6.3f   %4.2f / %4.2f   tags %s | %s   range %.2f, %s, spin %+.0f"
              % (lab, dp * 1e3, dr, s.hand_rms_px, s.front_rms_px, sorted(s.hand_corners or []), sorted(s.front_corners or []),
                 v.range_m, azimuth_word(v), v.spin_deg))
    for k in ('raw', 'joint'):
        c = CC.residual_correlations(res[k], views)
        print("  %s residual vs view: r(range) %+.2f, r(tilt) %+.2f, r(spin) %+.2f%s" % (
            k, c['range_m'], c['tilt_deg'], c['spin_deg'],
            "   <- tracks range/tilt: detection / intrinsics / tag size, not an extrinsic" if max(abs(c['range_m']), abs(c['tilt_deg'])) > 0.7 else ""))

    print("\nthe user's metric — tag A (hand_cam axis) -> tag B (front_cam axis) through the chain vs the sheet:")
    print("  %-8s %-28s %-28s %s" % ("fit", "train pos mm mean/sd/rms/p95", "train rot deg mean/sd/rms/p95", "hold-out pos rms / rot rms"))
    for k in ('raw', 'hand', 'base', 'joint'):
        r = res[k]
        errs = CC.pair_errors(train, sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain, r.D, r.F, compensate_T_ab2mb)
        (pm, ps, pr, p95), (rm, rs, rr, r95) = CC.summarise_errors(errs)
        ho = ""
        if test:
            e2 = CC.pair_errors(test, sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain, r.D, r.F, compensate_T_ab2mb)
            (_, _, pr2, _), (_, _, rr2, _) = CC.summarise_errors(e2)
            ho = "%.2f mm / %.3f deg" % (pr2, rr2)
        print("  %-8s %5.2f/%5.2f/%5.2f/%5.2f          %5.3f/%5.3f/%5.3f/%5.3f          %s"
              % (k, pm, ps, pr, p95, rm, rs, rr, r95, ho))
    raw_pairs = CC.pair_errors(train, sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain, lift_compensate=compensate_T_ab2mb)
    bias = np.mean([e[5] for e in raw_pairs], axis=0); sd = np.std([e[5] for e in raw_pairs], axis=0)
    print("  raw T_A2B translation error in B's frame: bias (%+.1f, %+.1f, %+.1f) mm, sd (%.1f, %.1f, %.1f) mm — only the bias is correctable"
          % (bias[0], bias[1], bias[2], sd[0], sd[1], sd[2]))
    if args.pairs:
        print("\nevery (A in hand_cam, B in front_cam) pair, raw vs the '%s' fit — n, pos rms mm, rot rms deg, raw bias in B's frame (mm):" % args.pairs)
        rf = res[args.pairs]
        raw_all = CC.all_pair_errors(samples, sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain, lift_compensate=compensate_T_ab2mb)
        fix_all = CC.all_pair_errors(samples, sheet, H, ext.T_ab2mb, ext.T_mb2fc_chain, rf.D, rf.F, compensate_T_ab2mb)
        keys = sorted({(e[1], e[2]) for e in raw_all})
        for a, b in keys:
            r = [e for e in raw_all if (e[1], e[2]) == (a, b)]; f = [e for e in fix_all if (e[1], e[2]) == (a, b)]
            (_, _, pr0, _), (_, _, rr0, _) = CC.summarise_errors(r); (_, _, pr1, _), (_, _, rr1, _) = CC.summarise_errors(f)
            bb = np.mean([e[5] for e in r], axis=0)
            print("  %3d -> %3d   n %2d   raw %6.2f / %.3f   %-5s %6.2f / %.3f   bias (%+.1f, %+.1f, %+.1f)"
                  % (a, b, len(r), pr0, rr0, args.pairs, pr1, rr1, bb[0], bb[1], bb[2]))

    floor = 3e-3
    h, b, j = res['hand'], res['base'], res['joint']
    if h.rms_pos_m <= max(floor, 1.5 * j.rms_pos_m) and h.rms_pos_m < 0.5 * b.rms_pos_m:
        verdict = "HAND side: the hand-eye explains the residual; the base side adds nothing"
    elif b.rms_pos_m <= max(floor, 1.5 * j.rms_pos_m) and b.rms_pos_m < 0.5 * h.rms_pos_m:
        verdict = "BASE side: an arm-mount / front_cam error explains it; the hand-eye is fine"
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
             sheet_sx=sheet.sx, sheet_sy=sheet.sy, sheet_tag_size_m=sheet.tag_size_m,
             holdout=np.array([s.label for s in test]))
    print("\ncorrections saved -> %s/corrections.npz" % args.dir)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet-json", default=None, metavar="LAYOUT_JSON", help="layout file (default: sheet/*.json)")
    ap.add_argument("--sx", type=float, default=None, help="print scale along W x (measured / design)")
    ap.add_argument("--sy", type=float, default=None, help="print scale along W y")
    ap.add_argument("--tag-size", type=float, default=None, help="tag black edge as measured on the print (m)")
    ap.add_argument("--frames", type=int, default=20, help="frames averaged per camera per capture")
    ap.add_argument("--hand-eye", default=None, help="hand-eye npz to evaluate (default: locator.yaml's)")
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("check", help="read both cameras and the arm once; nothing saved").set_defaults(fn=cmd_check)
    p = sp.add_parser("capture", help="save one sample at the current arm pose"); p.add_argument("dir")
    p.add_argument("--label", default=None)
    p.add_argument("--force", action="store_true", help="save even a flagged (ambiguous single-tag) PnP")
    p.set_defaults(fn=cmd_capture)
    p = sp.add_parser("status", help="list the session's samples and coverage"); p.add_argument("dir"); p.set_defaults(fn=cmd_status)
    p = sp.add_parser("drop", help="remove samples by label"); p.add_argument("dir"); p.add_argument("labels", nargs="+"); p.set_defaults(fn=cmd_drop)
    p = sp.add_parser("solve", help="fit the corrections"); p.add_argument("dir")
    p.add_argument("--exclude", nargs="*", default=None, help="sample labels to drop")
    p.add_argument("--outlier-factor", type=float, default=4.0)
    p.add_argument("--holdout", nargs="*", default=None, help="sample labels kept OUT of the fit, evaluated after")
    p.add_argument("--holdout-every", type=int, default=0, help="hold out every k-th sample (e.g. 4)")
    p.add_argument("--pairs", choices=["hand", "base", "joint"], default=None,
                   help="also list every (hand_cam tag -> front_cam tag) pair's error, raw vs this fit")
    p.add_argument("--write-hand-eye", choices=["hand", "joint"], default=None,
                   help="write path_tag_locator/config/hand_eye/T_hc2ee_chain_<date>.npz from this fit's D")
    p.set_defaults(fn=cmd_solve)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
