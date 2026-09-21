#!/usr/bin/env python3
"""
sheet_path.py — drive the hand_cam LENS along a list of points given in tag
200's frame (the A0 sheet frame W), pausing at each, and measure how far
the lens actually landed from each point.

    rosrun chain_calib sheet_path.py log/chain_calib/<session>            # the 10 grid points, 3 s dwell
    rosrun chain_calib sheet_path.py log/chain_calib/<session> --dry-run  # plan only, no motion
    rosrun chain_calib sheet_path.py log/chain_calib/<session> --points "0.850,0,0.50;1.000,0,0.50" --dwell 5

The TCP for this script is the CAMERA OPTICAL CENTRE, not the flange: each
point (x, y, h) means "put the hand_cam lens at W (x, y) exactly h metres
ABOVE the sheet, looking straight down" (W's +z points INTO the paper, so
the lens sits at W z = -h). The flange pose that does that comes through
the hand-eye:

    T_ab2ee = T_ab2W · T_W2p · inv(T_hc2p) · T_hc2ee
    T_ab2W  = T_ab2mb(lift) · T_mb2fc · T_fc2W(front_cam's LIVE view of tag 200)
    T_hc2p  = [Rz(spin) | (0, 0, h)]        lens h above p, optical axis along -W z

So the base may stand anywhere front_cam sees tag 200; the sheet's own
rigidity carries the rest. front_cam's single-tag ROTATION is replaced by
the level-floor prior (the same choice `solve` and `verify_chain` make —
one tag's tilt is the paper's local slope, see CHAIN_CALIB_2026-09-21_kr.md).

At each point, after the dwell, hand_cam re-solves the sheet from every tag
it sees and reports the LENS's actual position in W against the commanded
point — xy first, that is the number that matters — and the range. Rows
are appended to <session>/sheet_path_<HHMMSS>.csv as they are measured.

Safety: the arm is never moved automatically until you confirm the printed
plan once; the first target must be within --max-first-move of the current
flange; every target must be inside the 1.25 m flange reach; a failed move
skips that point; Ctrl-C stops after the current point. Hand on the e-stop.
No automatic motion inside chain_calib's calibration itself — this script
is the verification path, like verify_chain.
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

from chain_calib import solver as CC                                   # noqa: E402
from chain_calib.session import load_samples                           # noqa: E402
from path_tag_locator.chain import compensate_T_ab2mb                  # noqa: E402
from path_tag_locator.geometry import invert_T, matrix_m_to_pose_fr5, pose_fr5_to_matrix_m  # noqa: E402

import importlib.util                                                  # noqa: E402
_spec = importlib.util.spec_from_file_location("chain_calib_tool", os.path.join(_HERE, "chain_calib.py"))
tool = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(tool)   # platform(), Session, sheet_from_args
_vspec = importlib.util.spec_from_file_location("verify_chain_tool", os.path.join(_HERE, "verify_chain.py"))
vc = importlib.util.module_from_spec(_vspec); _vspec.loader.exec_module(vc)     # corrected_chain(), Rz()

# The ten grid tags' centres, 0.50 m above the sheet — the user's list of 2026-09-21.
DEFAULT_POINTS = "0.850,0,0.50;1.000,0,0.50;0.850,0.150,0.50;1.000,0.150,0.50;0.850,0.300,0.50;" \
                 "1.000,0.300,0.50;0.850,0.450,0.50;1.000,0.450,0.50;0.850,0.600,0.50;1.000,0.600,0.50"


def parse_points(text):
    pts = []
    for tok in text.replace("\n", ";").split(";"):
        tok = tok.strip()
        if not tok:
            continue
        x, y, h = [float(v) for v in tok.split(",")]
        if h <= 0.05:
            raise SystemExit("point %s: h is the height ABOVE the sheet in metres and must be > 0.05" % tok)
        pts.append((x, y, h))
    return pts


def lens_targets(T_ab2W, Hc, points, spin_deg):
    """Flange poses that put the hand_cam lens at each W point, looking down."""
    out = []
    for (x, y, h) in points:
        T_W2p = np.eye(4); T_W2p[:3, 3] = (x, y, 0.0)
        T_hc2p = np.eye(4); T_hc2p[:3, :3] = vc.Rz(spin_deg); T_hc2p[2, 3] = h
        T_ab2ee = T_ab2W @ T_W2p @ invert_T(T_hc2p) @ Hc
        out.append(((x, y, h), matrix_m_to_pose_fr5(T_ab2ee), float(np.linalg.norm(T_ab2ee[:3, 3]))))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", help="chain_calib session directory (its meta.yaml / corrections.npz)")
    ap.add_argument("--points", default=DEFAULT_POINTS,
                    help="'x,y,h;x,y,h;...' in metres, tag-200 frame; h = lens height ABOVE the sheet (default: the 10 grid tags at 0.50)")
    ap.add_argument("--dwell", type=float, default=3.0, help="seconds to rest at each point before measuring (default 3)")
    ap.add_argument("--spin", type=float, default=None, help="camera spin about its axis vs the sheet (deg; default: keep the current)")
    ap.add_argument("--fit", choices=["none", "hand", "base", "joint"], default="none",
                    help="chain to use: none = extrinsics.yaml as applied (default), or a fit of <dir>/corrections.npz")
    ap.add_argument("--corrections", default=None)
    ap.add_argument("--front-rotation", choices=["level", "measured"], default="level")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--max-first-move", type=float, default=0.35)
    ap.add_argument("--dry-run", action="store_true", help="print the plan, move nothing")
    ap.add_argument("--yes", action="store_true", help="skip the one confirmation prompt")
    ap.add_argument("--hand-eye", default=None)
    ap.add_argument("--sheet-json", default=None); ap.add_argument("--sx", type=float, default=None)
    ap.add_argument("--sy", type=float, default=None); ap.add_argument("--tag-size", type=float, default=None)
    args = ap.parse_args()

    import rospy
    rospy.init_node("sheet_path", anonymous=True)
    cfg, ext, H = tool.platform(hand_eye=args.hand_eye)
    _, meta = load_samples(args.dir)
    sheet = tool.sheet_from_args(args, meta)
    T_ab2mb, Hc, note = vc.corrected_chain(ext, H, args.corrections or os.path.join(args.dir, "corrections.npz"), args.fit)
    points = parse_points(args.points)
    print("chain: %s" % note)
    print("TCP for this run = hand_cam LENS. %d point(s) in tag 200's frame (x, y = on the sheet; h = lens height above it):" % len(points))

    S = tool.Session(cfg, sheet, ext.front_cam_frame, args.frames); S.ext = ext
    f, ftags, nf = S.camera_T(S.cfg.topics.front_cam_detections, S.K_front,
                              None if S.front_frame == "level" else S.D_front, "front_cam")
    slope = math.degrees(math.acos(max(-1.0, min(1.0, abs(f.T_cam2W[2, 2])))))
    T_fc2W = f.T_cam2W if args.front_rotation == "measured" else CC.level_front_observation(f.T_cam2W)
    lift = S.lift.height_m() or 0.0
    cur = S.arm.get_tcp_pose()
    T_ab2W = compensate_T_ab2mb(T_ab2mb, lift) @ ext.T_mb2fc_chain @ T_fc2W
    T_hc2W_cur = invert_T(pose_fr5_to_matrix_m(cur) @ invert_T(Hc)) @ T_ab2W
    spin_cur = math.degrees(math.atan2(T_hc2W_cur[1, 0], T_hc2W_cur[0, 0]))
    spin = spin_cur if args.spin is None else args.spin
    print("front_cam sees %s (PnP rms %.2f px), paper slope under it %.2f deg (%s); lift %.3f m; spin %.0f deg (current %.0f)"
          % (f.tag_ids, f.rms_px, slope, "replaced by the level prior" if args.front_rotation == "level" else "USED as measured",
             lift, spin, spin_cur))
    targets = lens_targets(T_ab2W, Hc, points, spin)
    print("current flange: %s" % ["%.1f" % v for v in cur])
    print("  #  lens @ W (x, y, h) m       flange x      y      z      rx      ry      rz   reach")
    for i, (p, pose, reach) in enumerate(targets):
        print("%3d  (%.3f, %.3f, %.3f)   %8.1f %6.1f %6.1f %7.2f %7.2f %7.2f   %.3f" % (i + 1, *p, *pose, reach))
    d0 = float(np.linalg.norm(np.array(targets[0][1][:3]) - np.array(cur[:3])) / 1e3)
    print("first target is %.2f m from the current flange" % d0)
    if args.dry_run:
        return
    if d0 > args.max_first_move:
        sys.exit("first target is %.2f m away — jog the lens over the grid at ~%.2f m first (or --max-first-move)" % (d0, points[0][2]))
    if max(r for _, _, r in targets) > 1.25:
        sys.exit("a target is beyond the 1.25 m flange reach — lower h or drop that point")
    if not args.yes:
        if input("\nMove through all %d points with %.1f s dwell? [y/N] " % (len(targets), args.dwell)).strip().lower() != "y":
            print("aborted"); return

    out = os.path.join(args.dir, "sheet_path_%s.csv" % time.strftime("%H%M%S"))
    fh = open(out, "w", newline=""); w = csv.writer(fh)
    w.writerow(["idx", "cmd_x_m", "cmd_y_m", "cmd_h_m", "flange_x_mm", "flange_y_mm", "flange_z_mm", "flange_rx", "flange_ry", "flange_rz",
                "lens_x_m", "lens_y_m", "lens_h_m", "err_x_mm", "err_y_mm", "err_xy_mm", "err_h_mm", "tilt_deg",
                "nearest_tag", "tag_centre_dpx_x", "tag_centre_dpx_y", "hand_tags", "hand_rms_px", "note"])
    fh.flush()
    print("results -> %s (appended per point). Ctrl-C stops after the current point. Hand on the e-stop.\n" % out)
    Kh = S.K_hand; cx, cy = Kh[0, 2], Kh[1, 2]
    exy, eh = [], []
    try:
        for i, (p, pose, reach) in enumerate(targets):
            print("-> %d/%d  lens to W (%.3f, %.3f, h %.3f)" % (i + 1, len(targets), *p))
            try:
                S.arm.move_j_to_pose(pose, linear=True)
            except Exception as e:
                print("   move failed: %s — skipping" % e)
                w.writerow([i + 1, *p, *pose] + [""] * 13 + ["move failed: %s" % e]); fh.flush(); continue
            rospy.sleep(args.dwell)
            try:
                h, htags, nh = S.camera_T(S.cfg.topics.hand_cam_detections, Kh, S.D_hand, "hand_cam")
            except Exception as e:
                print("   hand_cam: %s" % e)
                w.writerow([i + 1, *p, *pose] + [""] * 13 + ["hand_cam: %s" % e]); fh.flush(); continue
            T_W2hc = invert_T(h.T_cam2W)                       # lens pose in W, measured against the sheet itself
            lens = T_W2hc[:3, 3]
            err_x, err_y = (lens[0] - p[0]) * 1e3, (lens[1] - p[1]) * 1e3
            err_h = (-lens[2] - p[2]) * 1e3                    # W z points into the paper: height = -z
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(T_W2hc[2, 2])))))
            k = sheet.nearest_tag(p[:2])
            if k in htags:
                c = htags[k].corners_px.mean(axis=0); dpx = (c[0] - cx, c[1] - cy)
            else:
                dpx = (float("nan"), float("nan"))
            exy.append(math.hypot(err_x, err_y)); eh.append(err_h)
            print("   lens landed at W (%.4f, %.4f, h %.4f):  xy error (%+.1f, %+.1f) mm = %.1f mm;  height %+.1f mm;  tilt %.2f deg;  "
                  "tag %d centre %+.0f, %+.0f px from the image centre;  hand tags %s rms %.2f px"
                  % (lens[0], lens[1], -lens[2], err_x, err_y, math.hypot(err_x, err_y), err_h, tilt, k, dpx[0], dpx[1], h.tag_ids, h.rms_px))
            w.writerow([i + 1, *p, *["%.3f" % v for v in pose], "%.5f" % lens[0], "%.5f" % lens[1], "%.5f" % -lens[2],
                        "%.2f" % err_x, "%.2f" % err_y, "%.2f" % math.hypot(err_x, err_y), "%.2f" % err_h, "%.3f" % tilt,
                        k, "%.1f" % dpx[0], "%.1f" % dpx[1], " ".join(map(str, h.tag_ids)), "%.2f" % h.rms_px, "ok"])
            fh.flush()
    except KeyboardInterrupt:
        print("\n(interrupted — stopping after the current point)")
    finally:
        fh.close()
    if exy:
        exy = np.array(exy); eh = np.array(eh)
        print("\n%d point(s): lens xy error mean %.1f mm, rms %.1f mm, max %.1f mm;  height error mean %+.1f mm, sd %.1f mm"
              % (len(exy), exy.mean(), np.sqrt((exy ** 2).mean()), exy.max(), eh.mean(), eh.std()))
    print("results -> %s" % out)


if __name__ == "__main__":
    main()
