#!/usr/bin/env python3
"""
basler_tip_calib.py — measure the wrist Basler's vision tip (where its
frame centre looks, in the flange frame) against the A4 20 mm tag sheet
that the hand camera also sees. Method and maths: chain_calib/basler_tip.py.

    rosrun chain_calib basler_tip_calib.py check                       # both cameras see the sheet? scale? nothing saved
    rosrun chain_calib basler_tip_calib.py capture-hand   <dir>        # hand_cam over the sheet (0.25-0.35 m), 20 frames
    rosrun chain_calib basler_tip_calib.py capture-basler <dir>        # Basler at its standoff over ONE tag, one lamp-on frame
    rosrun chain_calib basler_tip_calib.py status <dir>
    rosrun chain_calib basler_tip_calib.py solve  <dir> [--sx --sy --tag-size]

Procedure (base still, sheet flat on the plate, the whole session in one
go): 4-6 `capture-hand` from different spins about the camera axis, then
6-10 `capture-basler` — jog the Basler over a tag, run the Keyence
standoff (robot_ui Arm tab "Auto standoff" 16.5 mm, or --standoff here),
capture; move to another tag and/or spin the wrist 30-60 deg between
captures (the spins are what determine the tip's lateral components).
`solve` prints p_tip / psi against robot.yaml's vision_tip_offset_mm.

--sx / --sy / --tag-size: the print's MEASURED scale (chain_calib README §2):
sx = (outer-left of 201 to outer-right of 205, mm) / 180, sy = (top of 201
to bottom of 226) / 220, tag-size = one black edge in m. Sessions under
log/chain_calib/basler_tip_<date>/.
"""
import argparse
import json
import os
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))

from chain_calib import basler_tip as BT                               # noqa: E402
from chain_calib import sheet as SH                                    # noqa: E402
from path_tag_locator import WS_DIR                                    # noqa: E402
from path_tag_locator.constants import load_locator_cfg                # noqa: E402
from path_tag_locator.hand_eye import load_T_hc2ee                     # noqa: E402

DEFAULT_SHEET = os.path.normpath(os.path.join(_HERE, "..", "sheet", "A4_tag20_201-230_5x6_layout.json"))
DEFAULT_OUT = os.path.join(WS_DIR, "log", "chain_calib")
DESIGN_TIP = (0.0, -253.0, 225.2)


def _ptl_cfg_dir():
    cand = os.path.normpath(os.path.join(_HERE, "..", "..", "path_tag_locator", "config"))
    if not os.path.isdir(cand):
        import rospkg
        cand = os.path.join(rospkg.RosPack().get_path("path_tag_locator"), "config")
    return cand


def _resolve(p):
    import re
    return re.sub(r"\$\(find path_tag_locator\)", os.path.normpath(os.path.join(_ptl_cfg_dir(), "..")), str(p))


def sheet_from_args(args, meta=None):
    meta = meta or {}
    path = args.sheet_json or meta.get("sheet_json", DEFAULT_SHEET)
    sx = args.sx if args.sx is not None else float(meta.get("sheet_sx", 1.0))
    sy = args.sy if args.sy is not None else float(meta.get("sheet_sy", 1.0))
    ts = args.tag_size if args.tag_size is not None else meta.get("sheet_tag_size_m")
    sh = SH.load_sheet(path, sx, sy, ts)
    if abs(sx - 1.0) < 1e-12 and abs(sy - 1.0) < 1e-12 and (ts is None or abs(float(ts) - sh.design_tag_size_m) < 1e-12):
        print("! sheet scale is the DESIGN value (40 mm pitch, 20 mm tag) — measure the print: --sx --sy --tag-size")
    return sh


def hand_eye_from_args(args, meta=None):
    cfg = load_locator_cfg(os.path.join(_ptl_cfg_dir(), "locator.yaml"))
    path = args.hand_eye or (meta or {}).get("hand_eye_npz") or _resolve(cfg.hand_eye_npz)
    return cfg, path, load_T_hc2ee(path)


# ----------------------------------------------------------------------
class Ros:
    def __init__(self, cfg, frames):
        import rospy
        from path_tag_locator.arm_interface import ArmInterface
        self.rospy = rospy
        self.cfg, self.frames = cfg, int(frames)
        self.arm = ArmInterface(state_topic=cfg.arm.state_topic, move_cart_topic=cfg.arm.move_cart_topic,
                                home_service=cfg.arm.home_service, motion_timeout_s=cfg.arm.motion_timeout_s)
        from sensor_msgs.msg import CameraInfo
        msg = rospy.wait_for_message(cfg.topics.hand_cam_info, CameraInfo, timeout=5.0)
        self.K_hand = np.asarray(msg.K, float).reshape(3, 3)
        self.D_hand = np.asarray(msg.D, float).ravel() if msg.D else np.zeros(5)

    def tcp(self):
        ok, why = self.arm.wait_for_node(10.0)
        if not ok:
            raise RuntimeError(why)
        return list(map(float, self.arm.get_tcp_pose()))

    def hand_frames(self, timeout=6.0):
        import threading
        from robot_msgs.msg import AprilTagDetectionArray
        frames, lock, done = [], threading.Lock(), threading.Event()

        def _cb(arr):
            fr = {int(d.id): np.asarray(d.corners, float).reshape(4, 2) for d in arr.detections}
            with lock:
                if len(frames) < self.frames:
                    frames.append(fr)
                if len(frames) >= self.frames:
                    done.set()
        sub = self.rospy.Subscriber(self.cfg.topics.hand_cam_detections, AprilTagDetectionArray, _cb, queue_size=1)
        try:
            t0 = time.time()
            while not self.rospy.is_shutdown() and not done.wait(0.05) and time.time() - t0 < timeout:
                pass
        finally:
            sub.unregister()
        with lock:
            return list(frames)

    def hand_sample(self, label):
        tcp = self.tcp()
        frames = self.hand_frames()
        if not frames:
            raise RuntimeError("no detection frames on %s" % self.cfg.topics.hand_cam_detections)
        tags = SH.accumulate_frames(frames)
        if not tags:
            raise RuntimeError("hand_cam sees no tag (%d frames)" % len(frames))
        s = BT.HandSample(label, tcp, {k: t.corners_px for k, t in tags.items()})
        return s, tags, len(frames)

    def basler_frame(self):
        """One lamp-on frame from basler_camera_node (the node opens / closes
        the device and brackets the lamp; nothing here touches either)."""
        from robot_msgs.srv import CaptureImages
        self.rospy.wait_for_service("/camera/capture", timeout=5.0)
        resp = self.rospy.ServiceProxy("/camera/capture", CaptureImages)(1, 0.0, True)
        if not resp.success or not resp.images:
            raise RuntimeError("capture failed: %s" % resp.message)
        img = resp.images[0]
        buf = np.frombuffer(img.data, np.uint8)
        if img.encoding in ("mono8", "8UC1"):
            gray = buf.reshape(img.height, img.step)[:, :img.width].copy()
        elif img.encoding in ("bgr8", "rgb8"):
            import cv2
            arr = buf.reshape(img.height, img.step // 3, 3)[:, :img.width]
            gray = cv2.cvtColor(arr, cv2.COLOR_BGR2GRAY)
        else:
            raise RuntimeError("unexpected Basler encoding %s" % img.encoding)
        return gray, resp.message

    def standoff_now(self):
        """The live Keyence standoff line, when arm_node publishes it."""
        from std_msgs.msg import String
        try:
            m = self.rospy.wait_for_message("/arm/standoff_state", String, timeout=1.5)
            d = json.loads(m.data)
            return float(d.get("standoff_mm")) if d.get("valid") else float("nan"), d
        except Exception:
            return float("nan"), None

    def run_standoff(self, target_mm, timeout=60.0):
        """Ask arm_node to run the Keyence standoff loop from here (the
        robot_ui 'Auto standoff' path), wait for the motion to finish."""
        from std_msgs.msg import String
        pub = self.rospy.Publisher("/arm/standoff", String, queue_size=1, latch=True)
        t0 = time.time()
        while pub.get_num_connections() == 0 and time.time() - t0 < 3.0:
            time.sleep(0.05)
        st = self.arm.state
        seq0 = st.motion_seq if st is not None else None
        pub.publish(String(json.dumps({"target_mm": float(target_mm)})))
        t0 = time.time()
        while time.time() - t0 < timeout:                # completion = motion_seq advancing (arm_node's protocol)
            time.sleep(0.2)
            st = self.arm.state
            if seq0 is not None and st is not None and st.motion_seq != seq0:
                break
            if seq0 is None and time.time() - t0 > 8.0:
                break
        time.sleep(0.5)


def print_hand(s, tags, n, sheet, K, D):
    r = SH.multi_tag_pnp(s.corners, sheet, K, D)
    std = max(t.std_px for t in tags.values())
    print("  hand_cam: %d tags (%d/%d frames), PnP rms %.2f px, corner scatter %.2f px, range %.3f m, spin %+.0f deg"
          % (len(tags), min(t.n_frames for t in tags.values()), n, r.rms_px, std, r.T_cam2W[2, 3],
             np.degrees(np.arctan2(r.T_cam2W[1, 0], r.T_cam2W[0, 0]))))
    for fl in r.flags:
        print("  ! " + fl)
    if std > 0.3:
        print("  ! corner scatter > 0.3 px — arm still moving?")
    if not 0.2 <= r.T_cam2W[2, 3] <= 0.45:
        print("  ! hand_cam should be 0.25-0.35 m over the sheet (20 mm tags: ~45-60 px)")
    if len(tags) < 6:
        print("  ! only %d tags — get more of the sheet in view" % len(tags))


def print_basler(b, sheet):
    o, th, ppm, rms, ids = BT.fit_similarity(b.corners, sheet, b.image_wh)
    print("  basler: tags %s, %.1f px/mm (tag %.0f px), similarity rms %.2f px; frame centre over sheet (%.2f, %.2f) mm, "
          "image x at %+.2f deg in W%s" % (ids, ppm, ppm * sheet.tag_size_m * 1e3, rms, o[0] * 1e3, o[1] * 1e3, th,
                                            "" if np.isnan(b.standoff_mm) else "; standoff %.2f mm" % b.standoff_mm))
    if rms > 3.0:
        print("  ! similarity rms %.1f px — perspective/tilt or a wrong tag size; the Basler should be square to the sheet" % rms)
    w, h = b.image_wh
    fov = (w / ppm, h / ppm)
    print("  basler field of view at this standoff: %.1f x %.1f mm" % fov)


# ----------------------------------------------------------------------
def cmd_check(args):
    import rospy
    rospy.init_node("basler_tip_calib", anonymous=True)
    cfg, he_path, H = hand_eye_from_args(args)
    sheet = sheet_from_args(args)
    print(sheet.describe()); print("hand-eye: %s" % he_path)
    R = Ros(cfg, args.frames)
    try:
        s, tags, n = R.hand_sample("check")
        print_hand(s, tags, n, sheet, R.K_hand, R.D_hand)
    except Exception as e:
        print("hand_cam: %s" % e)
    try:
        gray, msg = R.basler_frame()
        det = BT.detect_basler_corners(gray, sheet.family)
        if det:
            b = BT.BaslerSample("check", [0] * 6, det, (gray.shape[1], gray.shape[0]), R.standoff_now()[0])
            print_basler(b, sheet)
        else:
            print("  basler: no tag in the frame (%s) — %dx%d, mean %.0f" % (msg, gray.shape[1], gray.shape[0], gray.mean()))
    except Exception as e:
        print("basler: %s" % e)


def _open(args):
    hand, bas, meta = BT.load_session(args.dir)
    cfg, he_path, H = hand_eye_from_args(args, meta)
    sheet = sheet_from_args(args, meta)
    if meta and os.path.abspath(sheet.path) != meta.get("sheet_json"):
        sys.exit("%s: session started with sheet %s" % (args.dir, meta.get("sheet_json")))
    return hand, bas, meta, cfg, he_path, H, sheet


def cmd_capture_hand(args):
    import rospy
    rospy.init_node("basler_tip_calib", anonymous=True)
    hand, bas, meta, cfg, he_path, H, sheet = _open(args)
    R = Ros(cfg, args.frames)
    if not meta:
        meta = dict(date=time.strftime("%Y-%m-%d %H:%M"), sheet_json=os.path.abspath(sheet.path), sheet_sx=sheet.sx,
                    sheet_sy=sheet.sy, sheet_tag_size_m=sheet.tag_size_m, hand_eye_npz=he_path,
                    K_hand=R.K_hand.tolist(), D_hand=R.D_hand.tolist(), frames_per_hand_sample=args.frames)
    rospy.sleep(0.3)
    label = args.label or "h%d" % (len(hand) + 1)
    s, tags, n = R.hand_sample(label)
    print("hand sample %s:" % label)
    print_hand(s, tags, n, sheet, R.K_hand, R.D_hand)
    hand.append(s)
    BT.save_session(args.dir, hand, bas, meta)
    BT.resolve_hand(hand, sheet, R.K_hand, R.D_hand, H)
    if sum(h.T_ab2W is not None for h in hand) >= 2:
        T, rms, mx, deg = BT.sheet_pose([h for h in hand if h.T_ab2W is not None])
        print("sheet pose over %d hand samples: scatter %.1f mm rms / %.1f max, %.2f deg" % (len(hand), rms, mx, deg))
    print("saved -> %s (%d hand, %d basler)" % (args.dir, len(hand), len(bas)))


def cmd_capture_basler(args):
    import rospy
    import cv2
    rospy.init_node("basler_tip_calib", anonymous=True)
    hand, bas, meta, cfg, he_path, H, sheet = _open(args)
    if not meta:
        sys.exit("capture-hand first (the session's hand_cam intrinsics and sheet scale come from it)")
    R = Ros(cfg, args.frames)
    if args.standoff is not None:
        print("running the Keyence standoff loop to %.1f mm ..." % args.standoff)
        R.run_standoff(args.standoff)
    rospy.sleep(0.3)
    so, sd = R.standoff_now()
    gray, msg = R.basler_frame()
    tcp = R.tcp()
    det = BT.detect_basler_corners(gray, sheet.family)
    if not det:
        sys.exit("no tag in the Basler frame (%s) — jog until one 20 mm tag is inside, at the standoff" % msg)
    label = args.label or "b%d" % (len(bas) + 1)
    os.makedirs(os.path.join(args.dir, "basler"), exist_ok=True)
    img_path = os.path.join("basler", "%s.png" % label)
    cv2.imwrite(os.path.join(args.dir, img_path), gray)
    b = BT.BaslerSample(label, tcp, det, (gray.shape[1], gray.shape[0]), so, img_path)
    print("basler sample %s (%s):" % (label, msg))
    print_basler(b, sheet)
    if sd is not None and not sd.get("valid"):
        print("  ! Keyence out of range — the Basler is probably not at its standoff / focus")
    bas.append(b)
    BT.save_session(args.dir, hand, bas, meta)
    print("saved -> %s (%d hand, %d basler); TCP %s" % (args.dir, len(hand), len(bas), ["%.1f" % v for v in tcp]))


def _solve(args, quiet=False):
    hand, bas, meta, cfg, he_path, H, sheet = _open(args)
    if not hand or not bas:
        sys.exit("need hand and basler samples (%d / %d)" % (len(hand), len(bas)))
    K = np.asarray(meta["K_hand"], float).reshape(3, 3); D = np.asarray(meta.get("D_hand") or [], float)
    BT.resolve_hand(hand, sheet, K, D, H)
    hand = [h for h in hand if h.T_ab2W is not None]
    if args.exclude:
        bas = [b for b in bas if b.label not in set(args.exclude)]
        hand = [h for h in hand if h.label not in set(args.exclude)]
    BT.resolve_basler(bas, sheet)
    return hand, bas, meta, H, he_path, sheet


def cmd_status(args):
    hand, bas, meta, H, he_path, sheet = _solve(args)
    print("%s; %s\nhand-eye %s" % (meta.get("date"), sheet.describe(), he_path))
    T, rms, mx, deg = BT.sheet_pose(hand)
    print("sheet pose from %d hand samples: scatter %.1f mm rms / %.1f max, %.2f deg; origin (tag 201) at (%.1f, %.1f, %.1f) mm in the arm frame"
          % (len(hand), rms, mx, deg, T[0, 3] * 1e3, T[1, 3] * 1e3, T[2, 3] * 1e3))
    for b in bas:
        print("  %-5s tags %-12s centre over (%6.1f, %6.1f) mm  roll %+7.2f  %.1f px/mm  rms %.2f px  standoff %s"
              % (b.label, sorted(b.corners), b.o_W_m[0] * 1e3, b.o_W_m[1] * 1e3, b.theta_W_deg, b.px_per_mm,
                 b.sim_rms_px, "%.2f" % b.standoff_mm if not np.isnan(b.standoff_mm) else "?"))
    spins = sorted(round(b.theta_W_deg / 15.0) * 15 for b in bas)
    print("wrist spins seen (deg, 15 deg bins): %s — need >= 3 distinct for the lateral tip components" % sorted(set(spins)))


def cmd_solve(args):
    hand, bas, meta, H, he_path, sheet = _solve(args)
    res = BT.fit_tip(hand, bas, DESIGN_TIP)
    print("%d hand / %d basler samples; %s\nhand-eye %s" % (len(hand), len(bas), sheet.describe(), he_path))
    print(BT.summarize(res, H, DESIGN_TIP))
    T = BT.T_ee2tip(res)
    out = os.path.join(args.dir, "result.npz")
    np.savez(out, p_tip_mm=res.p_tip_mm, psi_deg=res.psi_deg, T_ee2tip=T, T_hc2tip=np.asarray(H) @ T,
             labels=np.array(res.labels), resid_mm=res.resid_mm, resid_deg=res.resid_deg,
             sheet_T_ab2W=BT.sheet_pose(hand)[0])
    with open(os.path.join(args.dir, "result.yaml"), "w") as fh:
        fh.write("# basler_tip_calib.py solve, %s\n" % time.strftime("%Y-%m-%d %H:%M"))
        fh.write("vision_tip_offset_mm: [%.2f, %.2f, %.2f]\n" % tuple(res.p_tip_mm))
        fh.write("image_roll_deg: %.3f\n" % res.psi_deg)
        fh.write("fit_rms_mm: %.2f\nfit_max_mm: %.2f\nsheet_scatter_mm: %.2f\n" % (res.rms_mm, res.max_mm, res.sheet_scatter_mm))
        if res.jackknife_sd_mm is not None:
            fh.write("jackknife_sd_mm: [%.2f, %.2f, %.2f]\n" % tuple(res.jackknife_sd_mm))
        fh.write("n_hand: %d\nn_basler: %d\nhand_eye_npz: %s\n" % (len(hand), len(bas), he_path))
    print("-> %s, %s" % (out, os.path.join(args.dir, "result.yaml")))
    print("NOT applied: vision_tip_offset_mm lives in robot.yaml arm_calibration, tools/set_tool_tcp.py (tool 1) and the "
          "planner URDF's vision_tip_joint — all three together, on the user's decision.")


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
    p = sub.add_parser("status"); p.add_argument("dir"); p.add_argument("--exclude", nargs="*")
    p = sub.add_parser("solve"); p.add_argument("dir"); p.add_argument("--exclude", nargs="*")
    args = ap.parse_args()
    if getattr(args, "dir", None) is None and args.cmd in ("capture-hand", "capture-basler"):
        args.dir = os.path.join(DEFAULT_OUT, "basler_tip_" + time.strftime("%Y%m%d"))
    {"check": cmd_check, "capture-hand": cmd_capture_hand, "capture-basler": cmd_capture_basler,
     "status": cmd_status, "solve": cmd_solve}[args.cmd](args)


if __name__ == "__main__":
    main()
