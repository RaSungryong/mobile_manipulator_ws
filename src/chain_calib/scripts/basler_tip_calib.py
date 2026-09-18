#!/usr/bin/env python3
"""
basler_tip_calib.py — measure the wrist Basler's vision tip (where its
frame centre looks, in the flange frame) against the A4 20 mm tag sheet
that the hand camera also sees. Maths: chain_calib/basler_tip.py; the
ROS side chain_calib/basler_tip_ros.py — the SAME class robot_ui's
Calibration tab "Basler vision tip" group drives, so either can be used.

    rosrun chain_calib basler_tip_calib.py check                       # both cameras see the sheet? scale? nothing saved
    rosrun chain_calib basler_tip_calib.py capture-hand   [<dir>]      # hand_cam over the sheet (0.25-0.35 m), 20 frames
    rosrun chain_calib basler_tip_calib.py capture-basler [<dir>] [--standoff 16.5]
                                                                       # Basler over ONE tag at its standoff, one lamp-on frame
    rosrun chain_calib basler_tip_calib.py status <dir>
    rosrun chain_calib basler_tip_calib.py solve  <dir> [--exclude b3]

Procedure (base still, sheet flat on the plate, one session): 4-6
capture-hand from different spins about the camera axis, then 6-10
capture-basler — jog the Basler over a tag, standoff (--standoff runs
the Keyence loop first), capture; another tag and/or the wrist spun
30-60 deg between captures. solve prints p_tip / psi against
robot.yaml's vision_tip_offset_mm and writes <dir>/result.yaml.

--sx / --sy / --tag-size: the print's measured scale (default: design,
40 mm pitch / 20 mm tag). Default <dir>: log/chain_calib/basler_tip_<date>.
"""
import argparse
import os
import sys

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from chain_calib.basler_tip_ros import BaslerTipSession, default_session_dir   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sheet-json", default=None)
    ap.add_argument("--sx", type=float, default=None); ap.add_argument("--sy", type=float, default=None)
    ap.add_argument("--tag-size", type=float, default=None, help="measured black edge [m]")
    ap.add_argument("--hand-eye", default=None, help="T_hc2ee npz (default: locator.yaml's)")
    ap.add_argument("--frames", type=int, default=20, help="hand_cam detection frames per sample")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    p = sub.add_parser("capture-hand"); p.add_argument("dir", nargs="?", default=None); p.add_argument("--label")
    p = sub.add_parser("capture-basler"); p.add_argument("dir", nargs="?", default=None); p.add_argument("--label")
    p.add_argument("--standoff", type=float, default=None, help="run /arm/standoff to this mm first (e.g. 16.5)")
    p = sub.add_parser("status"); p.add_argument("dir"); p.add_argument("--exclude", nargs="*", default=[])
    p = sub.add_parser("solve"); p.add_argument("dir"); p.add_argument("--exclude", nargs="*", default=[])
    args = ap.parse_args()

    import rospy
    rospy.init_node("basler_tip_calib", anonymous=True)
    d = getattr(args, "dir", None) or default_session_dir()
    S = BaslerTipSession(d, args.sheet_json, args.sx, args.sy, args.tag_size, args.hand_eye, args.frames)
    if S.design_scale and args.cmd != "status":
        print("! sheet scale is the DESIGN value (40 mm pitch, 20 mm tag) — --sx --sy --tag-size to use the measured print")
    if args.cmd == "check":
        ok, msg, _ = S.check()
    elif args.cmd == "capture-hand":
        ok, msg, _ = S.capture_hand(args.label)
    elif args.cmd == "capture-basler":
        ok, msg, _ = S.capture_basler(args.standoff, args.label)
    elif args.cmd == "status":
        ok, msg, _ = S.status(args.exclude)
    else:
        ok, msg, _ = S.solve(args.exclude)
    print(msg)
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
