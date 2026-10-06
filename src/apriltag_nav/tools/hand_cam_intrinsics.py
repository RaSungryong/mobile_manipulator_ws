#!/usr/bin/env python3
"""hand_cam_intrinsics.py — calibrate the hand_cam (RealSense D435 colour) intrinsics
from a planar target photographed by the RAW stream, and write the result into
robot.yaml ``robot_camera.intrinsics_override.hand_cam``.

Why (2026-09-22): the driver reports D = 0 and an fx 1.2 % high; the override
of 2026-09-21 was fitted from A0-sheet corners taken at ONE range with the
board mostly central, so fx / k1 / hand-eye z are only weakly separated and
the joint-offset fit still shows a −0.7 → −2.4 px radial residual toward the
frame edge. A proper set — three ranges, tilts about both axes, the board in
every corner — is the fix.

    rosrun apriltag_nav hand_cam_intrinsics.py board  --out /tmp/board.pdf        # printable 9x6 / 25 mm checkerboard (A4)
    rosrun apriltag_nav hand_cam_intrinsics.py capture <dir> --chess 9x6 --square-mm 24.9
    rosrun apriltag_nav hand_cam_intrinsics.py capture <dir> --a4 --tag-mm 19.9 --pitch-mm 39.8 39.9
    rosrun apriltag_nav hand_cam_intrinsics.py solve <dir> [--apply]

capture: subscribes ``/hand_cam/color/image_raw`` (the DRIVER's frame — the
node's override remaps only its own detection copy, so this is raw whatever
the override says) and CameraInfo once; detects the target on every frame,
prints a live line (seen / range / tilt / where in the frame / motion) and
saves a view on Enter when the target is detected AND the corners have not
moved > ``--still-px`` (1.0) over the last ``--still-frames`` (4) frames (arm at
rest; the test is corner-order-insensitive, since the symmetric 8x12 grid comes
back reversed on some frames).
``--auto`` saves by itself whenever the view is still and differs from every
saved one (range 4 cm, tilt 6 deg or centroid 60 px). Every save prints the
coverage table (range x tilt x position). Stored: ``frames/vNN.png`` (raw),
``corners.json`` (object mm + image px per view), ``meta.yaml``.

solve: ``cv2.calibrateCamera`` with k1 k2 (``--model``: k1k2 default, k1k2k3,
full), per-view rms with the worst flagged, and the acceptance checks the
procedure asks for — fx of the two halves (odd/even AND near/far) within
``--fx-tol`` px, k1 within ``--k1-tol``, total rms ≤ ``--rms-max`` — then the
robot.yaml block. ``--apply`` rewrites the K / D / note lines of the existing
``intrinsics_override.hand_cam`` block in place (comments kept); restart
``robot_camera_node`` and the calibration launch afterwards.

Target geometry is what YOU measured: ``--square-mm`` for the checkerboard,
``--tag-mm`` / ``--pitch-mm`` for the A4 tag sheet (design 20 / 40). A 0.5 %
error there is 0.5 % of fx.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import threading
import time

import cv2
import numpy as np

TOPIC_IMG = "/hand_cam/color/image_raw"
TOPIC_INFO = "/hand_cam/color/camera_info"
RANGE_BINS = [(0.20, 0.40, "0.25-0.40"), (0.40, 0.50, "0.40-0.50"), (0.50, 0.70, "0.50-0.60")]
TILT_BINS = [(0, 10, "<10"), (10, 25, "10-25"), (25, 90, ">25")]
POS_NAMES = ["TL", "TR", "BL", "BR", "C"]


# ----------------------------------------------------------------------
# targets
# ----------------------------------------------------------------------
class Chessboard:
    kind = "chess"

    def __init__(self, cols, rows, square_mm):
        self.cols, self.rows, self.square = int(cols), int(rows), float(square_mm)
        self.obj = np.array([[c * self.square, r * self.square, 0.0] for r in range(self.rows) for c in range(self.cols)], float)

    def describe(self):
        return "checkerboard %dx%d inner corners, %.3f mm squares" % (self.cols, self.rows, self.square)

    def detect(self, gray):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
        ok, corners = cv2.findChessboardCornersSB(gray, (self.cols, self.rows), flags=flags)
        if not ok:
            return None
        img = corners.reshape(-1, 2).astype(float)
        # An even x even inner grid (8x12) is 180-deg symmetric, and the detector returns the reversed order on
        # some frames (mostly square-on, central views). Either order is a valid calibration view on its own,
        # but the stillness test compares consecutive frames, so canonicalise: first corner = the end nearer
        # the image origin. (still_px() also compares both orders, for the near-diagonal case this rule flips on.)
        if img[0].sum() > img[-1].sum():
            img = img[::-1].copy()
        return self.obj.copy(), img, self.cols * self.rows


class TagSheet:
    """The chain_calib A4 (201-230) or A0 (200, 300-309) sheet as the planar target."""
    kind = "tags"

    def __init__(self, layout_json, tag_mm, pitch_mm=None, sx=1.0, sy=1.0, min_tags=6):
        d = json.load(open(layout_json))
        self.layout = layout_json
        self.tag_mm = float(tag_mm)
        self.min_tags = int(min_tags)
        design_pitch = d.get("pitch_mm")
        if pitch_mm and design_pitch:
            sx, sy = float(pitch_mm[0]) / float(design_pitch[0]), float(pitch_mm[1]) / float(design_pitch[1])
        self.sx, self.sy = float(sx), float(sy)
        self.centres = {int(k): np.array(v["t_W_mm"][:2], float) * [self.sx, self.sy] for k, v in d["tags"].items()}
        h = self.tag_mm / 2.0
        # dt_apriltags corner order: bottom-left, bottom-right, top-right, top-left (path_tag_locator.detections._CORNER_ORDER)
        self.corner_off = np.array([[-h, +h], [+h, +h], [+h, -h], [-h, -h]], float)
        from dt_apriltags import Detector
        self.det = Detector(families=str(d.get("tag_family", "tag36h11")), quad_decimate=1.0, nthreads=2)

    def describe(self):
        return "tag sheet %s: %d tags, tag %.3f mm, scale sx %.5f sy %.5f" % (
            os.path.basename(self.layout), len(self.centres), self.tag_mm, self.sx, self.sy)

    def detect(self, gray):
        dets = [t for t in self.det.detect(gray) if t.tag_id in self.centres and t.hamming == 0]
        if len(dets) < self.min_tags:
            return None
        obj, img = [], []
        for t in sorted(dets, key=lambda t: t.tag_id):
            c = self.centres[t.tag_id]
            obj.append(np.hstack([c + self.corner_off, np.zeros((4, 1))]))
            img.append(np.asarray(t.corners, float).reshape(4, 2))
        return np.vstack(obj), np.vstack(img), len(dets)


def make_target(args):
    if args.chess:
        m = re.match(r"^(\d+)x(\d+)$", args.chess)
        if not m or not args.square_mm:
            sys.exit("--chess COLSxROWS (inner corners) and --square-mm are both required")
        return Chessboard(int(m.group(1)), int(m.group(2)), args.square_mm)
    if args.a4 or args.a0:
        from apriltag_nav.paths import WS_DIR
        name = "A4_tag20_201-230_5x6_layout.json" if args.a4 else "A0_landscape_tag200_300-309_5x2_FINAL2_layout.json"
        path = args.layout or os.path.join(WS_DIR, "src", "chain_calib", "sheet", name)
        tag = args.tag_mm or (20.0 if args.a4 else 90.0)
        return TagSheet(path, tag, args.pitch_mm, args.sx, args.sy, args.min_tags)
    sys.exit("pick a target: --chess COLSxROWS --square-mm S | --a4 | --a0")


# ----------------------------------------------------------------------
# view geometry (for coverage bookkeeping; the driver K is good enough for it)
# ----------------------------------------------------------------------
def view_geometry(obj, img, K, wh):
    ok, rvec, tvec = cv2.solvePnP(obj.reshape(-1, 1, 3) * 1e-3, img.reshape(-1, 1, 2), K, None, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    n_cam = R @ np.array([0, 0, 1.0])                     # board normal in the camera frame
    tilt = math.degrees(math.acos(min(1.0, abs(n_cam[2]))))
    centre_cam = R @ (obj.mean(0) * 1e-3) + tvec.ravel()
    rng = float(centre_cam[2])
    cx, cy = img.mean(0)
    w, h = wh
    u, v = cx / w, cy / h
    if abs(u - 0.5) < 0.2 and abs(v - 0.5) < 0.2:
        pos = "C"
    else:
        pos = ("T" if v < 0.5 else "B") + ("L" if u < 0.5 else "R")
    # how much of the frame the board spans, and how close its extreme corners get to the edge
    x0, y0 = img.min(0); x1, y1 = img.max(0)
    edge = float(min(x0, y0, w - x1, h - y1))
    return dict(range_m=rng, tilt_deg=tilt, pos=pos, u=float(u), v=float(v), edge_px=edge,
                span=float(max(x1 - x0, y1 - y0)))


def bin_of(x, bins):
    for lo, hi, name in bins:
        if lo <= x < hi:
            return name
    return "out"


def coverage_table(views):
    lines = []
    lines.append("  views: %d   (target 30-50)" % len(views))
    for title, key, bins in (("range [m]", "range_m", RANGE_BINS), ("tilt [deg]", "tilt_deg", TILT_BINS)):
        counts = {name: 0 for _, _, name in bins}; counts["out"] = 0
        for v in views:
            counts[bin_of(v[key], bins)] += 1
        lines.append("  %-11s " % title + "  ".join("%s: %d" % (k, n) for k, n in counts.items() if not (k == "out" and n == 0)))
    counts = {p: 0 for p in POS_NAMES}
    for v in views:
        counts[v["pos"]] += 1
    lines.append("  position    " + "  ".join("%s: %d" % (k, n) for k, n in counts.items()))
    need = []
    for lo, hi, name in RANGE_BINS:
        if sum(1 for v in views if lo <= v["range_m"] < hi) < 8:
            need.append("range %s" % name)
    if sum(1 for v in views if v["tilt_deg"] >= 25) < 8:
        need.append("tilt >= 25 deg (both axes, four directions)")
    for p in POS_NAMES[:4]:
        if counts[p] < 3:
            need.append("board in the %s corner" % p)
    lines.append("  still missing: " + ("; ".join(need) if need else "nothing — solve"))
    return "\n".join(lines)


# ----------------------------------------------------------------------
# capture
# ----------------------------------------------------------------------
def cmd_capture(args):
    import rospy
    import yaml
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CameraInfo, Image

    target = make_target(args)
    # RAW-frame guard (user rule, 2026-09-22): the override in robot.yaml makes robot_camera_node REMAP its own
    # detection copy of the hand_cam frame; the node publishes no image but /hand_cam/tag_overlay. The driver's
    # /hand_cam/color/image_raw is therefore raw whatever the override says — and the overlay is not a frame to
    # calibrate on. Refuse anything that is not a driver image topic.
    if "tag_overlay" in args.topic or not args.topic.endswith("image_raw"):
        sys.exit("--topic must be the DRIVER's raw stream (…/image_raw), not %s" % args.topic)
    try:
        from apriltag_nav.camera_intrinsics import load_config_block
        ov = (load_config_block().get("hand_cam") or {})
        ov_state = "ENABLED" if ov.get("enabled", True) and ov else "off"
    except Exception:  # noqa: BLE001
        ov_state = "unknown"
    print("frames come from %s = the DRIVER's raw stream (robot.yaml intrinsics_override.hand_cam is %s — it "
          "remaps only robot_camera_node's own detection copy and is NOT in these frames)" % (args.topic, ov_state))
    os.makedirs(os.path.join(args.dir, "frames"), exist_ok=True)
    cpath = os.path.join(args.dir, "corners.json")
    views = json.load(open(cpath)) if os.path.exists(cpath) else {}
    rospy.init_node("hand_cam_intrinsics_capture", anonymous=True, disable_signals=True)
    info = rospy.wait_for_message(args.info_topic, CameraInfo, timeout=10.0)
    K = np.array(info.K, float).reshape(3, 3); wh = (info.width, info.height)
    meta = dict(topic=args.topic, info_topic=args.info_topic, image_size=[info.width, info.height],
                driver_K=[float(x) for x in info.K], driver_D=[float(x) for x in info.D],
                raw_frames=True, target=target.describe(), target_kind=target.kind, started=time.strftime("%Y-%m-%d %H:%M:%S"))
    if target.kind == "chess":
        meta.update(chess=[target.cols, target.rows], square_mm=target.square)
    else:
        meta.update(layout=target.layout, tag_mm=target.tag_mm, sx=target.sx, sy=target.sy)
    with open(os.path.join(args.dir, "meta.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)
    print("target: %s\nstream %s %dx%d, driver K fx %.1f fy %.1f cx %.1f cy %.1f, D %s"
          % (target.describe(), args.topic, info.width, info.height, K[0, 0], K[1, 1], K[0, 2], K[1, 2], list(info.D)))
    print("%d view(s) already in %s" % (len(views), args.dir))
    print("Enter = save the current view (needs the target seen and still)%s; q + Enter = quit"
          % ("; --auto is ON" if args.auto else ""))

    bridge = CvBridge()
    state = dict(frame=None, stamp=0.0, seq=0)
    lock = threading.Lock()

    def cb(msg):
        try:
            im = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            rospy.logwarn_throttle(5, "cv_bridge: %s" % e); return
        with lock:
            state["frame"] = im; state["stamp"] = msg.header.stamp.to_sec(); state["seq"] += 1
    sub = rospy.Subscriber(args.topic, Image, cb, queue_size=1, buff_size=2 ** 22)

    keys = []
    def reader():
        for line in sys.stdin:
            keys.append(line.strip())
    threading.Thread(target=reader, daemon=True).start()

    history = []            # recent (corners) for the stillness test
    last_seq = -1; last_auto_geo = None; last_line = ""
    def still_px(cur):
        """Largest corner displacement vs the last `still_frames` frames, ORDER-INSENSITIVE: a frame whose
        corners came back reversed (180-deg twin of a symmetric board) is compared reversed too."""
        if len(history) < args.still_frames:
            return float("inf")
        ref = [h for h in history[-args.still_frames:] if h.shape == cur.shape]
        if len(ref) < args.still_frames:
            return float("inf")
        return float(max(min(np.abs(h - cur).max(), np.abs(h - cur[::-1]).max()) for h in ref))

    while not rospy.is_shutdown():
        with lock:
            frame, seq = state["frame"], state["seq"]
        if frame is None or seq == last_seq:
            time.sleep(0.02); continue
        last_seq = seq
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        det = target.detect(gray)
        want = any(k == "" for k in keys)
        quit_ = any(k.lower() == "q" for k in keys)
        keys.clear()
        if quit_:
            break
        if det is None:
            history.clear()
            line = "target NOT seen                                      "
        else:
            obj, img, n = det
            motion = still_px(img)
            history.append(img); del history[:-max(args.still_frames, 1)]
            geo = view_geometry(obj, img, K, wh) or {}
            still = motion <= args.still_px
            line = "seen %2d  range %.2f m  tilt %4.1f  pos %-2s (u %.2f v %.2f)  edge %3.0f px  motion %5.2f px %s" % (
                n, geo.get("range_m", 0), geo.get("tilt_deg", 0), geo.get("pos", "?"), geo.get("u", 0), geo.get("v", 0),
                geo.get("edge_px", 0), motion if motion != float("inf") else 99.0, "STILL" if still else "moving")
            auto = False
            if args.auto and still and geo:
                new = True
                for v in views.values():
                    if (abs(v["range_m"] - geo["range_m"]) < 0.04 and abs(v["tilt_deg"] - geo["tilt_deg"]) < 6
                            and math.hypot(v["u"] - geo["u"], v["v"] - geo["v"]) * wh[0] < 60):
                        new = False; break
                auto = new and (last_auto_geo is None or time.time() - last_auto_geo > 1.5)
            if want and not still:
                print("\n  not saved: the corners moved %.2f px over the last %d frames — let the arm settle" % (motion, args.still_frames))
            if (want and still) or auto:
                label = "v%02d" % len(views)
                cv2.imwrite(os.path.join(args.dir, "frames", label + ".png"), frame)
                views[label] = dict(obj_mm=obj.tolist(), img_px=img.tolist(), n=n, motion_px=motion, **geo)
                json.dump(views, open(cpath, "w"))
                last_auto_geo = time.time(); history.clear()
                print("\n  saved %s: %s\n%s" % (label, line.strip(), coverage_table(list(views.values()))))
        if line != last_line:
            sys.stdout.write("\r" + line); sys.stdout.flush(); last_line = line
    sub.unregister()
    print("\n%d views in %s" % (len(views), args.dir))
    print(coverage_table(list(views.values())))


# ----------------------------------------------------------------------
# solve
# ----------------------------------------------------------------------
def calibrate(views, wh, model, K0=None):
    obj = [np.asarray(v["obj_mm"], np.float32) * 1e-3 for v in views]
    img = [np.asarray(v["img_px"], np.float32) for v in views]
    flags = cv2.CALIB_USE_INTRINSIC_GUESS if K0 is not None else 0
    if model == "k1k2":
        flags |= cv2.CALIB_FIX_K3 | cv2.CALIB_ZERO_TANGENT_DIST
    elif model == "k1k2k3":
        flags |= cv2.CALIB_ZERO_TANGENT_DIST
    K = K0.copy() if K0 is not None else None
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(obj, img, wh, K, None, flags=flags, criteria=crit)
    per = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, r, t, K, D)
        per.append(float(np.sqrt(((p.reshape(-1, 2) - i) ** 2).sum(1).mean())))
    return float(rms), K, D.ravel(), np.array(per)


def fmt_K(K, D):
    return "fx %.2f fy %.2f cx %.2f cy %.2f  k1 %+.4f k2 %+.4f p1 %+.4f p2 %+.4f k3 %+.4f" % (
        K[0, 0], K[1, 1], K[0, 2], K[1, 2], *(list(D) + [0.0] * 5)[:5])


def cmd_solve(args):
    import yaml
    meta = yaml.safe_load(open(os.path.join(args.dir, "meta.yaml")))
    views_all = json.load(open(os.path.join(args.dir, "corners.json")))
    wh = tuple(int(x) for x in meta["image_size"])
    K_drv = np.array(meta["driver_K"], float).reshape(3, 3)
    labels = [k for k in sorted(views_all) if k not in set(args.exclude or [])]
    views = [views_all[k] for k in labels]
    print("%s: %d views (%d excluded), %s\n%s" % (args.dir, len(views), len(views_all) - len(views), meta["target"],
                                                 coverage_table(views)))
    if len(views) < 10:
        sys.exit("need at least 10 views")
    # 1. all views, then drop the worst per-view outliers once (a moved arm shows as a 3x rms view)
    rms, K, D, per = calibrate(views, wh, args.model, K_drv)
    med = float(np.median(per))
    bad = [labels[i] for i in range(len(views)) if per[i] > max(args.outlier_x * med, 0.6)]
    if bad and not args.keep_outliers:
        print("dropping %d outlier view(s) (> %.1fx the median %.2f px): %s" % (len(bad), args.outlier_x, med,
              ", ".join("%s %.2f" % (b, per[labels.index(b)]) for b in bad)))
        keep = [i for i in range(len(views)) if labels[i] not in bad]
        labels = [labels[i] for i in keep]; views = [views[i] for i in keep]
        rms, K, D, per = calibrate(views, wh, args.model, K_drv)
    print("\n== result (%s model, %d views) ==\n  rms %.3f px   per-view median %.2f max %.2f (%s)" % (
        args.model, len(views), rms, np.median(per), per.max(), labels[int(np.argmax(per))]))
    print("  " + fmt_K(K, D))
    print("  driver:   " + fmt_K(K_drv, np.array(meta.get("driver_D") or [0] * 5)))
    try:
        from apriltag_nav.camera_intrinsics import load_config_block, IntrinsicsOverride
        cur = IntrinsicsOverride.from_dict("hand_cam", load_config_block().get("hand_cam"))
        if cur is not None:
            print("  override: " + fmt_K(cur.K, cur.D))
    except Exception as e:  # noqa: BLE001
        print("  (current override not readable: %s)" % e)
    r = math.hypot(wh[0] / 2, wh[1] / 2) * 0.8
    xn = r / K[0, 0]
    print("  distortion at r = %.0f px: %.1f px" % (r, r * (D[0] * xn ** 2 + D[1] * xn ** 4)))
    print("  per view: " + "  ".join("%s %.2f" % (l, p) for l, p in zip(labels, per)))

    # 2. acceptance: halves by index and by range
    print("\n== split checks (fx within %.1f px, k1 within %.3f) ==" % (args.fx_tol, args.k1_tol))
    ok_all = rms <= args.rms_max
    if not ok_all:
        print("  ! rms %.3f > %.2f px" % (rms, args.rms_max))
    order = np.argsort([v["range_m"] for v in views])
    splits = {"odd / even": ([i for i in range(len(views)) if i % 2 == 0], [i for i in range(len(views)) if i % 2 == 1]),
              "near / far": (list(order[:len(order) // 2]), list(order[len(order) // 2:]))}
    for name, (a, b) in splits.items():
        ra, Ka, Da, _ = calibrate([views[i] for i in a], wh, args.model, K_drv)
        rb, Kb, Db, _ = calibrate([views[i] for i in b], wh, args.model, K_drv)
        dfx, dfy, dk1 = abs(Ka[0, 0] - Kb[0, 0]), abs(Ka[1, 1] - Kb[1, 1]), abs(Da[0] - Db[0])
        ok = dfx <= args.fx_tol and dfy <= args.fx_tol and dk1 <= args.k1_tol
        ok_all &= ok
        print("  %-11s %s  |dfx| %.1f |dfy| %.1f px  |dk1| %.4f  |dcx| %.1f |dcy| %.1f px   (rms %.2f / %.2f)" % (
            name, "OK  " if ok else "FAIL", dfx, dfy, dk1, abs(Ka[0, 2] - Kb[0, 2]), abs(Ka[1, 2] - Kb[1, 2]), ra, rb))
        print("     A: %s\n     B: %s" % (fmt_K(Ka, Da), fmt_K(Kb, Db)))
    print("\nVERDICT: %s" % ("ACCEPT" if ok_all else "NOT yet — add views where the coverage table says, or check the target"))

    # 3. the block
    note = "calibrateCamera %s over %s (%d views, rms %.2f px, %s; fx/k1 halves within %.1f px / %.4f)" % (
        args.model, os.path.basename(os.path.normpath(args.dir)), len(views), rms, meta["target"].split(",")[0],
        max(abs(calibrate([views[i] for i in a], wh, args.model, K_drv)[1][0, 0] - calibrate([views[i] for i in b], wh, args.model, K_drv)[1][0, 0]) for a, b in splits.values()),
        max(abs(calibrate([views[i] for i in a], wh, args.model, K_drv)[2][0] - calibrate([views[i] for i in b], wh, args.model, K_drv)[2][0]) for a, b in splits.values()))
    block = ("      K: [%.2f, 0.0, %.2f,\n          0.0, %.2f, %.2f,\n          0.0, 0.0, 1.0]\n"
             "      D: [%.4f, %.4f, %.4f, %.4f, %.4f]\n      note: \"%s\"\n") % (
        K[0, 0], K[0, 2], K[1, 1], K[1, 2], D[0], D[1], D[2] if len(D) > 2 else 0, D[3] if len(D) > 3 else 0,
        D[4] if len(D) > 4 else 0, note)
    print("\nrobot.yaml robot_camera.intrinsics_override.hand_cam:\n" + block)
    result = dict(K=K.tolist(), D=[float(x) for x in D], rms=rms, n=len(views), model=args.model, note=note,
                  image_size=list(wh), views=labels)
    with open(os.path.join(args.dir, "result.yaml"), "w") as fh:
        yaml.safe_dump(result, fh, sort_keys=False)
    if args.apply:
        if not ok_all and not args.force:
            sys.exit("not applied: the checks failed (use --force to apply anyway)")
        apply_to_robot_yaml(args.config, K, D, note, wh)


def apply_to_robot_yaml(path, K, D, note, wh):
    """Rewrite the numeric lines of the existing intrinsics_override.hand_cam block in place — comments survive."""
    from apriltag_nav.paths import CONFIG_PATH
    path = path or CONFIG_PATH
    text = open(path).read()
    m = re.search(r"(intrinsics_override:\s*\n(?:.*\n)*?\s+hand_cam:\s*\n)", text)
    if not m:
        sys.exit("no intrinsics_override.hand_cam block in %s — add one by hand first" % path)
    start = m.end()
    # the block ends at the next line that is indented LESS than the hand_cam keys (or a top-level key)
    rest = text[start:]
    indent = None; end = len(rest)
    for mm in re.finditer(r"^( *)(\S.*)$", rest, re.M):
        if mm.group(2).lstrip().startswith("#"):
            continue
        if indent is None:
            indent = len(mm.group(1))
        elif len(mm.group(1)) < indent:
            end = mm.start(); break
    block = rest[:end]
    pad = " " * (indent or 6)
    newK = "%sK: [%.2f, 0.0, %.2f,\n%s    0.0, %.2f, %.2f,\n%s    0.0, 0.0, 1.0]" % (pad, K[0, 0], K[0, 2], pad, K[1, 1], K[1, 2], pad)
    newD = "%sD: [%.4f, %.4f, %.4f, %.4f, %.4f]" % ((pad,) + tuple((list(D) + [0.0] * 5)[:5]))
    block2, nk = re.subn(r"^ *K: \[[^\]]*\]", newK, block, count=1, flags=re.M | re.S)
    block2, nd = re.subn(r"^ *D: \[[^\]]*\]", newD, block2, count=1, flags=re.M)
    block2, nn = re.subn(r"^( *note:).*$", r"\1 \"%s\"" % note.replace("\\", "").replace('"', "'"), block2, count=1, flags=re.M)
    block2, ns = re.subn(r"^( *image_size:).*$", r"\1 [%d, %d]" % tuple(wh), block2, count=1, flags=re.M)
    if not (nk and nd):
        sys.exit("could not find the K / D lines inside the hand_cam block")
    open(path, "w").write(text[:start] + block2 + rest[end:])
    import yaml
    chk = yaml.safe_load(open(path))["robot_camera"]["intrinsics_override"]["hand_cam"]
    assert abs(chk["K"][0] - K[0, 0]) < 0.01 and abs(chk["D"][0] - D[0]) < 1e-4, "re-read mismatch"
    print("applied to %s — restart robot_camera_node (and the calibration launch) to use it" % path)


# ----------------------------------------------------------------------
# printable board
# ----------------------------------------------------------------------
def cmd_board(args):
    from PIL import Image as PILImage
    cols, rows = args.squares
    dpi = 300
    px = args.square_mm / 25.4 * dpi
    W, H = int(round(297 / 25.4 * dpi)), int(round(210 / 25.4 * dpi))        # A4 landscape
    im = np.full((H, W), 255, np.uint8)
    bw, bh = cols * px, rows * px
    x0, y0 = (W - bw) / 2, (H - bh) / 2
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                xa, ya = int(round(x0 + c * px)), int(round(y0 + r * px))
                xb, yb = int(round(x0 + (c + 1) * px)), int(round(y0 + (r + 1) * px))
                im[ya:yb, xa:xb] = 0
    label = "checkerboard %dx%d squares (%dx%d inner corners), %.1f mm design — print at 100 %%, MEASURE, glue flat" % (
        cols, rows, cols - 1, rows - 1, args.square_mm)
    cv2.putText(im, label, (int(x0), int(y0) - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, 0, 2)
    pil = PILImage.fromarray(im)
    pil.save(args.out, resolution=dpi) if args.out.lower().endswith(".pdf") else pil.save(args.out, dpi=(dpi, dpi))
    print("%s -> %s\ncapture with: --chess %dx%d --square-mm <measured>" % (label, args.out, cols - 1, rows - 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def target_args(p):
        p.add_argument("--chess", help="COLSxROWS inner corners, e.g. 9x6")
        p.add_argument("--square-mm", type=float, help="MEASURED square size")
        p.add_argument("--a4", action="store_true", help="the chain_calib A4 20 mm tag sheet (201-230)")
        p.add_argument("--a0", action="store_true", help="the chain_calib A0 sheet (200, 300-309)")
        p.add_argument("--layout", default=None)
        p.add_argument("--tag-mm", type=float, default=None, help="MEASURED black-edge tag size")
        p.add_argument("--pitch-mm", type=float, nargs=2, default=None, metavar=("PX", "PY"), help="MEASURED tag pitch x y")
        p.add_argument("--sx", type=float, default=1.0); p.add_argument("--sy", type=float, default=1.0)
        p.add_argument("--min-tags", type=int, default=6)

    c = sub.add_parser("capture"); c.add_argument("dir"); target_args(c)
    c.add_argument("--topic", default=TOPIC_IMG); c.add_argument("--info-topic", default=TOPIC_INFO)
    c.add_argument("--auto", action="store_true")
    c.add_argument("--still-px", type=float, default=1.0, help="max corner motion over the window to count as still "
                   "(0.5-0.8 px is exposure / corner noise; a moving arm is several px)")
    c.add_argument("--still-frames", type=int, default=4)
    c.set_defaults(fn=cmd_capture)

    s = sub.add_parser("solve"); s.add_argument("dir")
    s.add_argument("--model", choices=["k1k2", "k1k2k3", "full"], default="k1k2")
    s.add_argument("--exclude", nargs="*", default=None); s.add_argument("--keep-outliers", action="store_true")
    s.add_argument("--outlier-x", type=float, default=3.0)
    s.add_argument("--fx-tol", type=float, default=2.0); s.add_argument("--k1-tol", type=float, default=0.005)
    s.add_argument("--rms-max", type=float, default=0.35)
    s.add_argument("--apply", action="store_true"); s.add_argument("--force", action="store_true")
    s.add_argument("--config", default=None, help="robot.yaml to rewrite (default: the package's)")
    s.set_defaults(fn=cmd_solve)

    b = sub.add_parser("board"); b.add_argument("--out", required=True)
    b.add_argument("--squares", type=int, nargs=2, default=[10, 7], metavar=("COLS", "ROWS"))
    b.add_argument("--square-mm", type=float, default=25.0)
    b.set_defaults(fn=cmd_board)

    args = ap.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
