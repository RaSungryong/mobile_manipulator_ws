"""
basler_tip_ros.py
=================
The ROS side of the Basler vision-tip measurement (basler_tip.py holds
the maths): one class, ``BaslerTipSession``, driven by BOTH the command
line (scripts/basler_tip_calib.py) and robot_ui (RosBridge.basler_tip),
so the operator gets the same numbers and the same words whichever way
the buttons are pressed. Every method returns ``(ok, message, extra)``;
``message`` is the multi-line text the CLI prints and the UI logs.

The caller owns the rospy node (init_node) — this module only makes
subscribers / service proxies on it. Nothing here moves the arm except
``capture_basler(standoff_mm=…)``, which publishes the same
``/arm/standoff`` request robot_ui's "Auto standoff" button does.
"""
import json
import os
import re
import time

import numpy as np

from . import basler_tip as BT
from . import sheet as SH
from path_tag_locator.geometry import pose_fr5_to_matrix_m

_HERE = os.path.dirname(os.path.realpath(__file__))
DESIGN_TIP = (0.0, -253.0, 225.2)


def _ptl_cfg_dir():
    cand = os.path.normpath(os.path.join(_HERE, "..", "..", "..", "path_tag_locator", "config"))
    if not os.path.isdir(cand):
        import rospkg
        cand = os.path.join(rospkg.RosPack().get_path("path_tag_locator"), "config")
    return cand


def _resolve(p):
    return re.sub(r"\$\(find path_tag_locator\)", os.path.normpath(os.path.join(_ptl_cfg_dir(), "..")), str(p))


def default_sheet_json():
    cand = os.path.normpath(os.path.join(_HERE, "..", "..", "sheet", "A4_tag20_201-230_5x6_layout.json"))
    if not os.path.exists(cand):
        try:
            import rospkg
            cand = os.path.join(rospkg.RosPack().get_path("chain_calib"), "sheet", "A4_tag20_201-230_5x6_layout.json")
        except Exception:
            pass
    return cand


def default_session_dir():
    from path_tag_locator import WS_DIR
    return os.path.join(WS_DIR, "log", "chain_calib", "basler_tip_" + time.strftime("%Y%m%d"))


# ----------------------------------------------------------------------
class _Ros:
    """Subscribers / proxies on the caller's node."""

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

    def basler_frame(self):
        """One lamp-on frame from basler_camera_node (which opens / closes
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
        from std_msgs.msg import String
        try:
            m = self.rospy.wait_for_message("/arm/standoff_state", String, timeout=1.5)
            d = json.loads(m.data)
            return (float(d["standoff_mm"]) if d.get("valid") else float("nan")), d
        except Exception:
            return float("nan"), None

    def run_standoff(self, target_mm, timeout=60.0):
        """The Keyence standoff loop from here (robot_ui's Auto standoff);
        completion = arm_node's motion_seq advancing."""
        from std_msgs.msg import String
        pub = self.rospy.Publisher("/arm/standoff", String, queue_size=1, latch=True)
        t0 = time.time()
        while pub.get_num_connections() == 0 and time.time() - t0 < 3.0:
            time.sleep(0.05)
        st = self.arm.state
        seq0 = st.motion_seq if st is not None else None
        pub.publish(String(json.dumps({"target_mm": float(target_mm)})))
        t0 = time.time()
        while time.time() - t0 < timeout:
            time.sleep(0.2)
            st = self.arm.state
            if seq0 is not None and st is not None and st.motion_seq != seq0:
                break
            if seq0 is None and time.time() - t0 > 8.0:
                break
        time.sleep(0.5)


# ----------------------------------------------------------------------
class BaslerTipSession:
    def __init__(self, dir_=None, sheet_json=None, sx=None, sy=None, tag_size_m=None, hand_eye=None, frames=20):
        from path_tag_locator.constants import load_locator_cfg
        from path_tag_locator.hand_eye import load_T_hc2ee
        self.dir = dir_ or default_session_dir()
        self.hand, self.bas, self.meta = BT.load_session(self.dir)
        self.cfg = load_locator_cfg(os.path.join(_ptl_cfg_dir(), "locator.yaml"))
        self.hand_eye_path = hand_eye or self.meta.get("hand_eye_npz") or _resolve(self.cfg.hand_eye_npz)
        self.H = load_T_hc2ee(self.hand_eye_path)
        path = sheet_json or self.meta.get("sheet_json") or default_sheet_json()
        sx = sx if sx is not None else float(self.meta.get("sheet_sx", 1.0))
        sy = sy if sy is not None else float(self.meta.get("sheet_sy", 1.0))
        ts = tag_size_m if tag_size_m is not None else self.meta.get("sheet_tag_size_m")
        self.sheet = SH.load_sheet(path, sx, sy, ts)
        self.design_scale = (abs(sx - 1.0) < 1e-12 and abs(sy - 1.0) < 1e-12
                             and (ts is None or abs(float(ts) - self.sheet.design_tag_size_m) < 1e-12))
        self.frames = int(frames)
        self._ros = None

    # -- helpers ---------------------------------------------------------
    def ros(self):
        if self._ros is None:
            self._ros = _Ros(self.cfg, self.frames)
        return self._ros

    def counts(self):
        return {"dir": self.dir, "n_hand": len(self.hand), "n_basler": len(self.bas)}

    def _hand_text(self, s, tags, n):
        R = self.ros()
        r = SH.multi_tag_pnp(s.corners, self.sheet, R.K_hand, R.D_hand)
        std = max(t.std_px for t in tags.values())
        lines = ["hand_cam: %d tags (%d/%d frames), PnP rms %.2f px, corner scatter %.2f px, range %.3f m, spin %+.0f deg"
                 % (len(tags), min(t.n_frames for t in tags.values()), n, r.rms_px, std, r.T_cam2W[2, 3],
                    np.degrees(np.arctan2(r.T_cam2W[1, 0], r.T_cam2W[0, 0])))]
        lines += ["! " + fl for fl in r.flags]
        if std > 0.3:
            lines.append("! corner scatter > 0.3 px — arm still moving? capture again")
        if not 0.2 <= r.T_cam2W[2, 3] <= 0.45:
            lines.append("! hand_cam should be 0.25-0.35 m over the sheet (20 mm tags ~45-60 px)")
        if len(tags) < 6:
            lines.append("! only %d tags — get more of the sheet in view" % len(tags))
        return lines

    def _basler_text(self, b):
        o, th, ppm, rms, ids = BT.fit_similarity(b.corners, self.sheet, b.image_wh)
        w, h = b.image_wh
        lines = ["basler: tags %s, %.1f px/mm (tag %.0f px), similarity rms %.2f px; frame centre over sheet (%.1f, %.1f) mm, "
                 "image x %+.1f deg in W%s" % (ids, ppm, ppm * self.sheet.tag_size_m * 1e3, rms, o[0] * 1e3, o[1] * 1e3, th,
                                               "" if np.isnan(b.standoff_mm) else "; standoff %.2f mm" % b.standoff_mm),
                 "basler field of view at this standoff: %.1f x %.1f mm" % (w / ppm, h / ppm)]
        if rms > 3.0:
            lines.append("! similarity rms %.1f px — tilt or wrong tag size; the Basler should be square to the sheet" % rms)
        return lines

    def _sheet_pose_line(self):
        R = self.ros() if self._ros else None
        K = np.asarray(self.meta["K_hand"], float).reshape(3, 3) if "K_hand" in self.meta else R.K_hand
        D = np.asarray(self.meta.get("D_hand") or (R.D_hand if R else []), float)
        BT.resolve_hand(self.hand, self.sheet, K, D, self.H)
        good = [h for h in self.hand if h.T_ab2W is not None]
        if len(good) < 2:
            return None
        T, rms, mx, deg = BT.sheet_pose(good)
        return "sheet pose over %d hand samples: scatter %.1f mm rms / %.1f max, %.2f deg" % (len(good), rms, mx, deg)

    # -- commands --------------------------------------------------------
    def check(self):
        lines = [self.sheet.describe(), "hand-eye: %s" % self.hand_eye_path]
        if self.design_scale:
            lines.append("sheet scale = DESIGN (40 mm pitch, 20 mm tag)")
        ok = True
        try:
            R = self.ros()
            frames = R.hand_frames()
            tags = SH.accumulate_frames(frames) if frames else {}
            on_sheet = {k: t for k, t in tags.items() if k in self.sheet.t_W_m}
            if on_sheet:
                s = BT.HandSample("check", [0] * 6, {k: t.corners_px for k, t in on_sheet.items()})
                lines += self._hand_text(s, on_sheet, len(frames))
            else:
                lines.append("hand_cam: no A4 sheet tag in view (%d frames; other tags seen: %s) — jog it 0.25-0.35 m over the sheet"
                             % (len(frames), sorted(tags))); ok = False
        except Exception as e:
            lines.append("hand_cam: %s" % e); ok = False
        try:
            R = self.ros()
            gray, msg = R.basler_frame()
            det = BT.detect_basler_corners(gray, self.sheet.family)
            if det:
                b = BT.BaslerSample("check", [0] * 6, det, (gray.shape[1], gray.shape[0]), R.standoff_now()[0])
                lines += self._basler_text(b)
            else:
                lines.append("basler: no tag in the frame (%s) — %dx%d, mean %.0f (normal before the Basler is over the sheet)"
                             % (msg, gray.shape[1], gray.shape[0], gray.mean()))
        except Exception as e:
            lines.append("basler: %s" % e); ok = False
        return ok, "\n".join(lines), self.counts()

    def capture_hand(self, label=None):
        R = self.ros()
        if not self.meta:
            self.meta = dict(date=time.strftime("%Y-%m-%d %H:%M"), sheet_json=os.path.abspath(self.sheet.path),
                             sheet_sx=self.sheet.sx, sheet_sy=self.sheet.sy, sheet_tag_size_m=self.sheet.tag_size_m,
                             hand_eye_npz=self.hand_eye_path, K_hand=R.K_hand.tolist(), D_hand=R.D_hand.tolist(),
                             frames_per_hand_sample=self.frames)
        time.sleep(0.3)
        tcp = R.tcp()
        frames = R.hand_frames()
        if not frames:
            return False, "no detection frames on %s" % self.cfg.topics.hand_cam_detections, self.counts()
        tags = {k: t for k, t in SH.accumulate_frames(frames).items() if k in self.sheet.t_W_m}
        if not tags:
            return False, "hand_cam sees no A4 sheet tag (%d frames)" % len(frames), self.counts()
        label = label or "h%d" % (len(self.hand) + 1)
        s = BT.HandSample(label, tcp, {k: t.corners_px for k, t in tags.items()})
        lines = ["hand sample %s:" % label] + self._hand_text(s, tags, len(frames))
        self.hand.append(s)
        BT.save_session(self.dir, self.hand, self.bas, self.meta)
        sp = self._sheet_pose_line()
        if sp:
            lines.append(sp)
        lines.append("saved -> %s (%d hand, %d basler)" % (self.dir, len(self.hand), len(self.bas)))
        return True, "\n".join(lines), self.counts()

    def capture_basler(self, standoff_mm=None, label=None):
        import cv2
        if not self.meta:
            return False, "capture-hand first (the session's hand_cam intrinsics come from it)", self.counts()
        R = self.ros()
        lines = []
        if standoff_mm is not None:
            lines.append("Keyence standoff loop to %.1f mm …" % float(standoff_mm))
            R.run_standoff(float(standoff_mm))
        time.sleep(0.3)
        so, sd = R.standoff_now()
        gray, msg = R.basler_frame()
        tcp = R.tcp()
        det = BT.detect_basler_corners(gray, self.sheet.family)
        if not det:
            return False, "no tag in the Basler frame (%s) — jog until one 20 mm tag is inside, at the standoff" % msg, self.counts()
        label = label or "b%d" % (len(self.bas) + 1)
        os.makedirs(os.path.join(self.dir, "basler"), exist_ok=True)
        img_path = os.path.join("basler", "%s.png" % label)
        cv2.imwrite(os.path.join(self.dir, img_path), gray)
        b = BT.BaslerSample(label, tcp, det, (gray.shape[1], gray.shape[0]), so, img_path)
        lines.append("basler sample %s (%s):" % (label, msg))
        lines += self._basler_text(b)
        if sd is not None and not sd.get("valid"):
            lines.append("! Keyence out of range — the Basler is probably not at its standoff / focus")
        self.bas.append(b)
        BT.save_session(self.dir, self.hand, self.bas, self.meta)
        lines.append("saved -> %s (%d hand, %d basler); TCP %s" % (self.dir, len(self.hand), len(self.bas),
                                                                   ["%.1f" % v for v in tcp]))
        return True, "\n".join(lines), self.counts()

    def _resolved(self, exclude=()):
        if not self.hand or not self.bas:
            raise RuntimeError("need hand and basler samples (%d / %d)" % (len(self.hand), len(self.bas)))
        K = np.asarray(self.meta["K_hand"], float).reshape(3, 3)
        D = np.asarray(self.meta.get("D_hand") or [], float)
        BT.resolve_hand(self.hand, self.sheet, K, D, self.H)
        hand = [h for h in self.hand if h.T_ab2W is not None and h.label not in set(exclude)]
        bas = [b for b in self.bas if b.label not in set(exclude)]
        BT.resolve_basler(bas, self.sheet)
        return hand, bas

    def status(self, exclude=()):
        try:
            hand, bas = self._resolved(exclude)
        except Exception as e:
            return False, str(e), self.counts()
        T, rms, mx, deg = BT.sheet_pose(hand)
        lines = ["%s; %s" % (self.meta.get("date"), self.sheet.describe()),
                 "sheet pose from %d hand samples: scatter %.1f mm rms / %.1f max, %.2f deg; tag 201 at (%.1f, %.1f, %.1f) mm in the arm frame"
                 % (len(hand), rms, mx, deg, T[0, 3] * 1e3, T[1, 3] * 1e3, T[2, 3] * 1e3)]
        for b in bas:
            lines.append("  %-5s tags %-12s centre over (%6.1f, %6.1f) mm  roll %+7.2f  %.1f px/mm  rms %.2f px  standoff %s"
                         % (b.label, sorted(b.corners), b.o_W_m[0] * 1e3, b.o_W_m[1] * 1e3, b.theta_W_deg, b.px_per_mm,
                            b.sim_rms_px, "%.2f" % b.standoff_mm if not np.isnan(b.standoff_mm) else "?"))
        spins = sorted(set(round(b.theta_W_deg / 15.0) * 15 for b in bas))
        lines.append("wrist spins seen (15 deg bins): %s — need >= 3 distinct for the lateral tip components" % spins)
        return True, "\n".join(lines), self.counts()

    # ------------------------------------------------------------------
    def verify(self, tag_id, standoff_mm=16.5, use_design=False, approach_mm=20.0, max_move_m=0.35,
               exclude=()):
        """EXECUTED check of the vision tip (user, 2026-09-21: "실행 검증").

        From the session's sheet pose (hand samples) the flange pose that
        puts the TIP on tag ``tag_id``'s centre is computed with the current
        wrist orientation kept (so the spin is whatever the operator set),
        raised ``approach_mm`` along the sheet normal; one MoveL there; the
        Keyence loop brings the case to ``standoff_mm`` (the seek covers the
        approach); one lamp-on Basler frame; the sheet point under the image
        centre vs the tag centre is the error, in mm. ``use_design`` runs
        the same test with robot.yaml's design tip for contrast. The move is
        refused when the target is farther than ``max_move_m`` from the
        current flange (jog closer first) — same rule as verify_chain. One
        row per run is appended to <dir>/verify.csv."""
        try:
            hand, _bas = self._resolved(exclude)
        except Exception as e:
            return False, str(e), self.counts()
        tag_id = int(tag_id)
        if tag_id not in self.sheet.t_W_m:
            return False, "tag %d is not on the sheet %s" % (tag_id, sorted(self.sheet.t_W_m)), self.counts()
        if use_design:
            tip, src = np.asarray(DESIGN_TIP, float), "DESIGN"
        else:
            rp = os.path.join(self.dir, "result.npz")
            if not os.path.exists(rp):
                return False, "no result.npz in %s — Solve first (or tick 'design tip')" % self.dir, self.counts()
            tip, src = np.asarray(np.load(rp)["p_tip_mm"], float), "measured"
        T_ab2W, sc, _mx, _deg = BT.sheet_pose(hand)
        R_W, t_W = T_ab2W[:3, :3], T_ab2W[:3, 3]
        P_W = np.array([self.sheet.t_W_m[tag_id][0], self.sheet.t_W_m[tag_id][1], 0.0])
        P = R_W @ P_W + t_W                       # tag centre, arm frame (m)
        up = -R_W[:, 2]                            # W +z is INTO the paper
        R = self.ros()
        cur = R.tcp()
        A = pose_fr5_to_matrix_m(cur)
        t = P + up * (float(approach_mm) / 1e3) - A[:3, :3] @ (tip / 1e3)
        target = [t[0] * 1e3, t[1] * 1e3, t[2] * 1e3, cur[3], cur[4], cur[5]]
        dist = float(np.linalg.norm(t - A[:3, 3]))
        lines = ["verify tag %d with the %s tip (%.1f, %.1f, %.1f) mm; sheet from %d hand samples (scatter %.1f mm)"
                 % (tag_id, src, tip[0], tip[1], tip[2], len(hand), sc),
                 "tag centre in the arm frame (%.1f, %.1f, %.1f) mm; flange target %s (%.0f mm from here, +%.0f mm above)"
                 % (P[0] * 1e3, P[1] * 1e3, P[2] * 1e3, ["%.1f" % v for v in target], dist * 1e3, approach_mm)]
        if dist > float(max_move_m):
            lines.append("REFUSED: target %.2f m from the current flange (> %.2f m) — jog the tool near tag %d first"
                         % (dist, max_move_m, tag_id))
            return False, "\n".join(lines), self.counts()
        try:
            R.arm.move_j_to_pose(target, linear=True)
        except Exception as e:
            lines.append("move failed: %s" % e)
            return False, "\n".join(lines), self.counts()
        lines.append("Keyence standoff loop to %.1f mm …" % float(standoff_mm))
        R.run_standoff(float(standoff_mm))
        time.sleep(0.3)
        so, sd = R.standoff_now()
        tcp = R.tcp()
        gray, msg = R.basler_frame()
        det = BT.detect_basler_corners(gray, self.sheet.family)
        if not det:
            lines.append("no tag in the Basler frame (%s): the tip is off by more than the ~30 mm field, or not at focus" % msg)
            return False, "\n".join(lines), self.counts()
        b = BT.BaslerSample("v", tcp, det, (gray.shape[1], gray.shape[0]), so)
        BT.resolve_basler([b], self.sheet)
        d_mm = (np.asarray(b.o_W_m[:2]) - P_W[:2]) * 1e3      # sheet frame: image centre - tag centre
        # where the tip is according to FK + this offset, for the record
        Af = pose_fr5_to_matrix_m(tcp)
        tip_ab = Af[:3, :3] @ (tip / 1e3) + Af[:3, 3]
        tip_W = R_W.T @ (tip_ab - t_W) * 1e3
        lines.append("Basler sees tags %s; image centre over (%.1f, %.1f) mm, tag %d centre at (%.1f, %.1f) mm"
                     % (sorted(det), b.o_W_m[0] * 1e3, b.o_W_m[1] * 1e3, tag_id, P_W[0] * 1e3, P_W[1] * 1e3))
        lines.append("ERROR (image centre - tag centre, sheet frame): dx %+.1f  dy %+.1f  |d| %.1f mm;  "
                     "tip via FK sits %.1f mm %s the sheet;  standoff %s;  wrist rz %.1f;  %.1f px/mm"
                     % (d_mm[0], d_mm[1], float(np.hypot(*d_mm)), abs(tip_W[2]), "above" if tip_W[2] < 0 else "below",
                        "%.2f" % so if not np.isnan(so) else "?", tcp[5], b.px_per_mm))
        if sd is not None and not sd.get("valid"):
            lines.append("! Keyence out of range — the Basler is not at its standoff / focus, the numbers above are not a verdict")
        verdict = "OK (inside the +/-3 mm the fit is good to)" if np.hypot(*d_mm) <= 3.0 else \
            ("marginal (3-6 mm: the arm's spin-dependent orientation error at this lever)" if np.hypot(*d_mm) <= 6.0
             else "OFF — the tip (or the sheet pose) is wrong by more than the arm can explain")
        lines.append("verdict: " + verdict)
        try:
            csvp = os.path.join(self.dir, "verify.csv")
            new = not os.path.exists(csvp)
            with open(csvp, "a") as fh:
                if new:
                    fh.write("time,tag,tip_source,tip_x,tip_y,tip_z,dx_mm,dy_mm,d_mm,standoff_mm,rz_deg,px_per_mm,"
                             "tcp_x,tcp_y,tcp_z,tcp_rx,tcp_ry,tcp_rz\n")
                fh.write("%s,%d,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%.1f,%.1f,%s\n"
                         % (time.strftime("%Y-%m-%d %H:%M:%S"), tag_id, src, tip[0], tip[1], tip[2], d_mm[0], d_mm[1],
                            float(np.hypot(*d_mm)), "%.2f" % so if not np.isnan(so) else "", tcp[5], b.px_per_mm,
                            ",".join("%.2f" % v for v in tcp)))
            lines.append("-> %s" % csvp)
        except Exception as e:
            lines.append("(verify.csv not written: %s)" % e)
        extra = self.counts()
        extra.update(verify_tag=tag_id, verify_dx_mm=float(d_mm[0]), verify_dy_mm=float(d_mm[1]),
                     verify_d_mm=float(np.hypot(*d_mm)), verify_tip_source=src)
        return True, "\n".join(lines), extra

    def solve(self, exclude=()):
        try:
            hand, bas = self._resolved(exclude)
            res = BT.fit_tip(hand, bas, DESIGN_TIP)
        except Exception as e:
            return False, str(e), self.counts()
        lines = ["%d hand / %d basler samples; hand-eye %s" % (len(hand), len(bas), self.hand_eye_path),
                 BT.summarize(res, self.H, DESIGN_TIP)]
        T = BT.T_ee2tip(res)
        np.savez(os.path.join(self.dir, "result.npz"), p_tip_mm=res.p_tip_mm, psi_deg=res.psi_deg, T_ee2tip=T,
                 T_hc2tip=np.asarray(self.H) @ T, labels=np.array(res.labels), resid_mm=res.resid_mm,
                 resid_deg=res.resid_deg, sheet_T_ab2W=BT.sheet_pose(hand)[0])
        with open(os.path.join(self.dir, "result.yaml"), "w") as fh:
            fh.write("# basler_tip solve, %s\n" % time.strftime("%Y-%m-%d %H:%M"))
            fh.write("vision_tip_offset_mm: [%.2f, %.2f, %.2f]\n" % tuple(res.p_tip_mm))
            fh.write("image_roll_deg: %.3f\n" % res.psi_deg)
            fh.write("fit_rms_mm: %.2f\nfit_max_mm: %.2f\nsheet_scatter_mm: %.2f\n" % (res.rms_mm, res.max_mm, res.sheet_scatter_mm))
            if res.jackknife_sd_mm is not None:
                fh.write("jackknife_sd_mm: [%.2f, %.2f, %.2f]\n" % tuple(res.jackknife_sd_mm))
            fh.write("n_hand: %d\nn_basler: %d\nhand_eye_npz: %s\n" % (len(hand), len(bas), self.hand_eye_path))
        lines.append("-> %s/result.yaml   (NOT applied: robot.yaml vision_tip_offset_mm, set_tool_tcp.py tool 1 and the "
                     "planner URDF vision_tip_joint move together, on your decision)" % self.dir)
        extra = self.counts()
        extra.update(p_tip_mm=[float(v) for v in res.p_tip_mm], psi_deg=float(res.psi_deg), rms_mm=float(res.rms_mm),
                     max_mm=float(res.max_mm), sheet_scatter_mm=float(res.sheet_scatter_mm),
                     jackknife_sd_mm=[float(v) for v in res.jackknife_sd_mm] if res.jackknife_sd_mm is not None else None)
        return True, "\n".join(lines), extra
