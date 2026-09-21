#!/usr/bin/env python3
"""
verify_chain.py — drive hand_cam over each grid tag THROUGH THE CHAIN and
measure where the tag actually ends up in the image.

front_cam sees tag 200 (the sheet frame W). For every tag k of the grid
the chain predicts the flange pose that puts hand_cam ``--height`` above
k, looking straight down with k on the optical axis:

    T_ab2ee_k = T_ab2mb' · T_mb2fc · T_fc2W(live) · T_W2k · inv(T_hc2k*) · T_hc2ee'

with T_hc2k* = [Rz(spin) | (0, 0, height)] and the primes = the
corrections of a `chain_calib solve` (``--corrections DIR/corrections.npz
--fit base|hand|joint``; ``--fit none`` = the chain as configured, for
the before/after comparison). After each move hand_cam re-detects the
sheet: the tag's offset from the image centre and its range vs
``--height`` ARE the chain error at that pose (the "user's metric",
executed instead of computed).

    rosrun chain_calib verify_chain.py plan log/chain_calib/<date> --fit base
    rosrun chain_calib verify_chain.py run  log/chain_calib/<date> --fit base      # Enter before every move
    rosrun chain_calib verify_chain.py run  log/chain_calib/<date> --fit none      # the uncorrected chain

`plan` writes <dir>/verify_plan_<fit>.csv (tag, flange x y z rx ry rz mm/deg,
reach) and moves nothing. `run` moves the arm — MoveL, one target at a
time, each after the operator presses Enter — and writes
<dir>/verify_result_<fit>.csv. The ten targets share one orientation and
one height (pure 150 mm translations between them); the FIRST target must
be within ``--max-first-move`` (0.35 m) of the current flange position, so
start with hand_cam already over the grid at about the requested height.
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))     # NOT _HERE: scripts/chain_calib.py would shadow the package

from chain_calib import sheet as SH                                    # noqa: E402
from chain_calib import solver as CC                                   # noqa: E402
from chain_calib.session import load_samples                           # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                  # noqa: E402
from path_tag_locator.geometry import invert_T, matrix_m_to_pose_fr5, pose_fr5_to_matrix_m, rot2rpy_deg  # noqa: E402

import importlib.util                                                  # noqa: E402
_spec = importlib.util.spec_from_file_location("chain_calib_tool", os.path.join(_HERE, "chain_calib.py"))
tool = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(tool)   # platform(), Session, sheet_from_args

GRID = [300, 301, 302, 303, 304, 305, 306, 307, 308, 309]


def corrected_chain(ext, H, corrections, fit):
    """(T_ab2mb', T_hc2ee') for the chosen fit of a corrections.npz."""
    if fit == "none" or corrections is None:
        return ext.T_ab2mb, H, "chain as configured (no correction)"
    c = np.load(corrections)
    if fit == "base":
        D, F = np.eye(4), c["F_base"]
    elif fit == "hand":
        D, F = c["D_hand"], np.eye(4)
    else:
        D, F = c["D_joint"], c["F_joint"]
    T_ab2mb = CC.corrected_T_ab2mb(ext.T_ab2mb, ext.T_mb2fc_chain, F)
    Hc = CC.corrected_hand_eye(H, D)
    return T_ab2mb, Hc, "%s fit of %s: %s ; %s" % (fit, corrections, CC.describe_T(D, "D"), CC.describe_T(F, "F"))


def Rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def targets_from(T_ab2mb, T_mb2fc, T_fc2W, lift_m, T_ab2ee_cur, Hc, sheet, height, spin_deg, tags):
    """The pure geometry: flange targets that put hand_cam ``height`` above
    each tag, level, at spin ``spin_deg`` (None = the spin the chain says
    the camera has now). Returns (targets [(tag, pose_mm_deg, reach_m)],
    T_ab2W, spin, spin_cur)."""
    T_ab2W = compensate_T_ab2mb(T_ab2mb, lift_m) @ T_mb2fc @ T_fc2W
    T_hc2W_cur = invert_T(T_ab2ee_cur @ invert_T(Hc)) @ T_ab2W
    spin_cur = math.degrees(math.atan2(T_hc2W_cur[1, 0], T_hc2W_cur[0, 0]))
    spin = spin_cur if spin_deg is None else float(spin_deg)
    T_hc2k = np.eye(4); T_hc2k[:3, :3] = Rz(spin); T_hc2k[2, 3] = float(height)
    out = []
    for k in tags:
        T_ab2ee = T_ab2W @ sheet.T_W2k(k) @ invert_T(T_hc2k) @ Hc
        out.append((k, matrix_m_to_pose_fr5(T_ab2ee), float(np.linalg.norm(T_ab2ee[:3, 3]))))
    return out, T_ab2W, spin, spin_cur


def plan_targets(S, sheet, T_ab2mb, Hc, height, spin_deg, tags, front_rotation="level"):
    """Live T_fc2W + current arm pose -> flange targets. Returns
    (targets [(tag, pose_mm_deg, reach_m)], info dict)."""
    f, ftags, nf = S.camera_T(S.cfg.topics.front_cam_detections, S.K_front,
                              None if S.front_frame == "level" else S.D_front, "front_cam")
    lift = S.lift.height_m() or 0.0
    cur = S.arm.get_tcp_pose()
    # front_cam's ROTATION of the sheet: the same treatment `solve` used for the T_ab2mb in
    # use (the level-floor prior by default, 2026-09-21). A single tag's out-of-plane tilt is
    # the local slope of the paper under it, and taken as the SHEET's orientation it would put
    # the far grid tags at the wrong height: 0.7 deg of it is 10 mm at tag 300 (0.85 m away)
    # and 17 mm at 309. The first (discarded) T_ab2mb happened to carry the equal-and-opposite
    # tilt of that day's paper, which is why its verify run read flat heights — self-consistency.
    T_fc2W = f.T_cam2W if front_rotation == "measured" else CC.level_front_observation(f.T_cam2W)
    slope = math.degrees(math.acos(max(-1.0, min(1.0, abs(f.T_cam2W[2, 2])))))
    out, T_ab2W, spin, spin_cur = targets_from(T_ab2mb, S.ext.T_mb2fc_chain, T_fc2W, lift,
                                               pose_fr5_to_matrix_m(cur), Hc, sheet, height, spin_deg, tags)
    info = dict(front_tags=f.tag_ids, front_rms=f.rms_px, lift=lift, spin=spin, spin_cur=spin_cur,
                cur=cur, T_ab2W=T_ab2W, T_fc2W=T_fc2W, paper_slope_deg=slope, front_rotation=front_rotation)
    return out, info


def print_plan(targets, info, height):
    print("front_cam sees %s (PnP rms %.2f px); lift %.3f m; camera spin %.0f deg (current %.0f)"
          % (info["front_tags"], info["front_rms"], info["lift"], info["spin"], info["spin_cur"]))
    print("front_cam sheet rotation: %s (measured paper slope under its tags %.2f deg%s)"
          % ("level-floor prior, measured yaw kept" if info["front_rotation"] == "level" else "AS MEASURED",
             info["paper_slope_deg"], " — would be %.0f mm of height at tag 309 if believed" % (1.45e3 * math.tan(math.radians(info["paper_slope_deg"]))) if info["front_rotation"] == "level" else ""))
    print("current flange: %s" % ["%.1f" % v for v in info["cur"]])
    print("%-4s %9s %9s %9s %8s %8s %8s  %s" % ("tag", "x_mm", "y_mm", "z_mm", "rx", "ry", "rz", "reach_m"))
    for k, p, r in targets:
        print("%-4d %9.1f %9.1f %9.1f %8.2f %8.2f %8.2f  %.3f%s" % (k, *p, r, "   ! near the 1.25 m reach limit" if r > 1.2 else ""))
    d0 = np.linalg.norm(np.asarray(targets[0][1][:3]) - np.asarray(info["cur"][:3])) / 1e3
    print("first target is %.3f m from the current flange position" % d0)
    return d0


def cmd_plan(args):
    import rospy
    rospy.init_node("verify_chain", anonymous=True)
    cfg, ext, H = tool.platform(hand_eye=args.hand_eye)
    _, meta = load_samples(args.dir)
    sheet = tool.sheet_from_args(args, meta)
    T_ab2mb, Hc, note = corrected_chain(ext, H, args.corrections or os.path.join(args.dir, "corrections.npz"), args.fit)
    print("chain: %s" % note)
    S = tool.Session(cfg, sheet, ext.front_cam_frame, args.frames); S.ext = ext
    targets, info = plan_targets(S, sheet, T_ab2mb, Hc, args.height, args.spin, args.tags, args.front_rotation)
    print_plan(targets, info, args.height)
    out = os.path.join(args.dir, "verify_plan_%s.csv" % args.fit)
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["tag", "x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg", "reach_m"])
        for k, p, r in targets:
            w.writerow([k] + ["%.3f" % v for v in p] + ["%.4f" % r])
    print("plan -> %s (nothing moved)" % out)


def cmd_run(args):
    import rospy
    rospy.init_node("verify_chain", anonymous=True)
    cfg, ext, H = tool.platform(hand_eye=args.hand_eye)
    _, meta = load_samples(args.dir)
    sheet = tool.sheet_from_args(args, meta)
    T_ab2mb, Hc, note = corrected_chain(ext, H, args.corrections or os.path.join(args.dir, "corrections.npz"), args.fit)
    print("chain: %s" % note)
    S = tool.Session(cfg, sheet, ext.front_cam_frame, args.frames); S.ext = ext
    targets, info = plan_targets(S, sheet, T_ab2mb, Hc, args.height, args.spin, args.tags, args.front_rotation)
    d0 = print_plan(targets, info, args.height)
    if d0 > args.max_first_move:
        sys.exit("first target is %.2f m away — jog hand_cam over the grid at ~%.2f m first (or --max-first-move)" % (d0, args.height))
    if max(r for _, _, r in targets) > 1.25:
        sys.exit("a target is beyond the 1.25 m flange reach — lower --height or drop that tag with --tags")
    Kh = S.K_hand; cx, cy = Kh[0, 2], Kh[1, 2]
    rows = []
    # The result file is opened NOW and appended after every target, so a Ctrl-C,
    # an EOF on stdin (pre-fed Enters running out) or a crash keeps what was
    # measured — the 2026-09-21 run lost nine measured targets to an EOF at the
    # tenth prompt because the file was only written after the loop.
    out = os.path.join(args.dir, "verify_result_%s_%s.csv" % (args.fit, time.strftime("%H%M%S")))
    HEADER = ["tag", "x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg", "off_x_mm", "off_y_mm", "range_m", "dpx_x", "dpx_y",
              "tilt_deg", "cam_over_tag_x_mm", "cam_over_tag_y_mm", "cam_rx_deg", "cam_ry_deg", "cam_rz_deg", "note"]
    fh = open(out, "w", newline="")
    w = csv.writer(fh); w.writerow(HEADER); fh.flush()
    print("results -> %s (appended after every target)" % out)

    def keep(row):
        rows.append(row); w.writerow(row); fh.flush()

    print("\nEnter = move to the next target, s = skip it, q = quit. Hand on the e-stop.")
    for k, pose, reach in targets:
        try:
            ans = input("-> tag %d at %s ? " % (k, ["%.1f" % v for v in pose])).strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n   (stdin closed / interrupted — stopping here, %d target(s) kept)" % len(rows)); break
        if ans == "q":
            break
        if ans == "s":
            continue
        try:
            S.arm.move_j_to_pose(pose, linear=True)
        except Exception as e:
            print("   move failed: %s" % e); continue
        rospy.sleep(args.settle)
        try:
            h, htags, nh = S.camera_T(S.cfg.topics.hand_cam_detections, Kh, S.D_hand, "hand_cam")
        except Exception as e:
            print("   hand_cam: %s" % e)
            keep([k] + ["%.3f" % v for v in pose] + [""] * 11 + ["not detected"]); continue
        T_hc2k = h.T_cam2W @ sheet.T_W2k(k)
        seen = k in htags
        if seen:
            c = htags[k].corners_px.mean(axis=0)
            dpx = (c[0] - cx, c[1] - cy)
        else:
            dpx = (float("nan"), float("nan"))
        off = T_hc2k[:3, 3] * 1e3
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(T_hc2k[2, 2])))))
        # the camera's POSITION over the tag (nadir offset, tag frame) and its
        # SIGNED orientation — an off-axis tag is position error + h*tan(tilt),
        # and only these two separate them
        cam_in_k = invert_T(T_hc2k)[:3, 3] * 1e3
        rpy_k = rot2rpy_deg(T_hc2k[:3, :3])
        # front_cam drift check is a DIAGNOSTIC: the arm can occlude tag 200 at the far
        # targets (2026-09-21, tag 308) — that must not end the run.
        try:
            f, _, _ = S.camera_T(S.cfg.topics.front_cam_detections, S.K_front, None if S.front_frame == "level" else S.D_front, "front_cam")
            dfront = np.linalg.norm(f.T_cam2W[:3, 3] - info["T_fc2W"][:3, 3]) * 1e3
        except Exception as e:
            dfront = float("nan"); print("   (front_cam: %s — probably occluded by the arm at this pose; drift not checked)" % e)
        print("   tag %d %s: centre offset %+.1f, %+.1f px = (%+.1f, %+.1f) mm; range %.4f m (target %.3f, %+.1f mm); tilt %.2f deg "
              "(cam rpy in tag %+.2f %+.2f %+.1f); camera over the tag at (%+.1f, %+.1f) mm; hand tags %s rms %.2f px; front_cam 200 moved %.1f mm"
              % (k, "seen" if seen else "NOT in view (offset from its PnP position)", dpx[0], dpx[1], off[0], off[1],
                 T_hc2k[2, 3], args.height, (T_hc2k[2, 3] - args.height) * 1e3, tilt, rpy_k[0], rpy_k[1], rpy_k[2],
                 cam_in_k[0], cam_in_k[1], h.tag_ids, h.rms_px, dfront))
        keep([k] + ["%.3f" % v for v in pose] + ["%.2f" % off[0], "%.2f" % off[1], "%.4f" % T_hc2k[2, 3],
                                                 "%.1f" % dpx[0], "%.1f" % dpx[1], "%.3f" % tilt,
                                                 "%.2f" % cam_in_k[0], "%.2f" % cam_in_k[1],
                                                 "%.3f" % rpy_k[0], "%.3f" % rpy_k[1], "%.3f" % rpy_k[2],
                                                 "seen" if seen else "not seen"])
    fh.close()
    ok = [r for r in rows if r[-1] == "seen"]
    if ok:
        ox = np.array([float(r[7]) for r in ok]); oy = np.array([float(r[8]) for r in ok]); rz_ = np.array([float(r[9]) for r in ok])
        px_ = np.array([float(r[13]) for r in ok]); py_ = np.array([float(r[14]) for r in ok])
        rx_ = np.array([float(r[15]) for r in ok]); ry_ = np.array([float(r[16]) for r in ok])
        print("\n%d tags: centre offset mean (%+.1f, %+.1f) mm, rms %.1f mm, max %.1f mm; range mean %.4f m (%+.1f mm vs target), sd %.1f mm"
              % (len(ok), ox.mean(), oy.mean(), np.sqrt((ox ** 2 + oy ** 2).mean()), np.hypot(ox, oy).max(),
                 rz_.mean(), (rz_.mean() - args.height) * 1e3, rz_.std() * 1e3))
        print("   camera POSITION over the tag: mean (%+.1f, %+.1f) mm, sd (%.1f, %.1f); camera tilt rx/ry: mean (%+.2f, %+.2f) deg, sd (%.2f, %.2f) — "
              "a spread here with ONE commanded orientation is the arm's actual orientation varying with position (FK / structure), not the chain"
              % (px_.mean(), py_.mean(), px_.std(), py_.std(), rx_.mean(), ry_.mean(), rx_.std(), ry_.std()))
    print("results -> %s" % out)


def main():
    common = argparse.ArgumentParser(add_help=False)
    ap = common
    ap.add_argument("dir", help="chain_calib session directory (its meta / corrections.npz)")
    ap.add_argument("--fit", choices=["none", "hand", "base", "joint"], default="base")
    ap.add_argument("--corrections", default=None, help="corrections.npz (default: <dir>/corrections.npz)")
    ap.add_argument("--height", type=float, default=0.50, help="hand_cam height above the tag (m)")
    ap.add_argument("--spin", type=float, default=None, help="camera spin about its axis vs the sheet (deg; default: keep the current)")
    ap.add_argument("--tags", type=int, nargs="*", default=GRID)
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--settle", type=float, default=0.8, help="s to wait after each move before measuring")
    ap.add_argument("--max-first-move", type=float, default=0.35)
    ap.add_argument("--hand-eye", default=None)
    ap.add_argument("--front-rotation", choices=["level", "measured"], default="level",
                    help="front_cam's sheet rotation: the level-floor prior (default; what solve used) or the single-tag PnP as measured")
    ap.add_argument("--sheet-json", default=None); ap.add_argument("--sx", type=float, default=None)
    ap.add_argument("--sy", type=float, default=None); ap.add_argument("--tag-size", type=float, default=None)
    top = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = top.add_subparsers(dest="cmd", required=True)
    sp.add_parser("plan", parents=[common], help="compute and print / save the ten targets; move nothing").set_defaults(fn=cmd_plan)
    sp.add_parser("run", parents=[common], help="move to each target (Enter each), measure").set_defaults(fn=cmd_run)
    a = top.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
