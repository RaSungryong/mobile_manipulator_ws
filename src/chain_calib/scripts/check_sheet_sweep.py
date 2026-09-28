#!/usr/bin/env python3
"""
check_sheet_sweep.py — offline check of sheet_sweep.py's planner and runner
against the REAL tf chain (tf_chain.yaml), the real sheet layout and a
realistic geometry taken from the 2026-09-21 arm session (front_cam's
view of tag 200, a flange pose over the 301/303 pair). No ROS.

    python3 src/chain_calib/scripts/check_sheet_sweep.py
"""
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", "src"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "path_tag_locator", "src"))
sys.path.insert(0, os.path.join(_HERE, "..", "..", "apriltag_nav", "src"))

import importlib.util                                                  # noqa: E402
_spec = importlib.util.spec_from_file_location("sheet_sweep_tool", os.path.join(_HERE, "sheet_sweep.py"))
sw = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(sw)
tool, vc = sw.tool, sw.vc

from chain_calib import sheet as SH                                    # noqa: E402
from chain_calib import solver as CC                                   # noqa: E402
from chain_calib.session import coverage                               # noqa: E402
from path_tag_locator.geometry import invert_T, matrix_m_to_pose_fr5, pose_fr5_to_matrix_m  # noqa: E402

N_OK = N_FAIL = 0


def check(name, ok, detail=""):
    global N_OK, N_FAIL
    N_OK += ok; N_FAIL += (not ok)
    print("  %s %s%s" % ("ok  " if ok else "FAIL", name, ("  [%s]" % detail) if detail else ""))


cfg_l, ext, H = tool.platform()
sheet = SH.load_sheet(sw.tool.DEFAULT_SHEET)
K_hand = np.array([[601.87, 0, 321.35], [0, 601.93, 238.51], [0, 0, 1.0]]); D_hand = np.zeros(5)
K_front = np.array([[750.24, 0, 638.20], [0, 749.77, 352.98], [0, 0, 1.0]])
sess = os.path.join(tool.WS_DIR, "log", "chain_calib", "20260921_arm", "samples.npz")
if os.path.exists(sess):
    d = np.load(sess)
    T_fc2W = CC.level_front_observation(d["T_fc2W"][1]); start = list(map(float, d["tcp"][1]))
else:                                                       # the same geometry, typed in
    T_fc2W = np.array([[0.001, -0.9999, -0.0128, 0.0023], [1.0, 0.001, 0.0035, 0.0065], [-0.0034, -0.0128, 0.9999, 0.3031], [0, 0, 0, 1.0]])
    T_fc2W = CC.level_front_observation(T_fc2W); start = [-568.95, 560.36, -36.20, -179.93, -0.02, -0.07]
T_ab2W = ext.T_ab2mb @ ext.T_mb2fc_chain @ T_fc2W
T_start = pose_fr5_to_matrix_m(start)
T_hc2W_cur = invert_T(T_start @ invert_T(H)) @ T_ab2W
spin_cur = math.degrees(math.atan2(T_hc2W_cur[1, 0], T_hc2W_cur[0, 0]))
print("geometry: camera %.3f m over W (%.3f, %.3f), spin %.0f deg" % (-invert_T(T_hc2W_cur)[2, 3], invert_T(T_hc2W_cur)[0, 3], invert_T(T_hc2W_cur)[1, 3], spin_cur))

print("\n== 1. camera_pose_W geometry ==")
T = sw.camera_pose_W((1.0, 0.3), 0.45, 0.0, 0.0, 0.0)
check("square view: camera 0.45 m above the tag, R = I", np.allclose(T[:3, 3], [1.0, 0.3, -0.45]) and np.allclose(T[:3, :3], np.eye(3)))
for az, want in ((0.0, (1, 0)), (90.0, (0, 1)), (180.0, (-1, 0)), (270.0, (0, -1))):
    T = sw.camera_pose_W((1.0, 0.3), 0.45, 15.0, az, 0.0)
    disp = T[:2, 3] - np.array([1.0, 0.3])
    axis_hits = invert_T(T)
    check("tilt 15 toward az %.0f displaces the camera toward %s and keeps the tag on the optical axis" % (az, want),
          np.dot(disp / np.linalg.norm(disp), want) > 0.99 and abs(np.linalg.norm(disp) - 0.45 * math.sin(math.radians(15))) < 1e-9
          and abs(math.degrees(math.acos(abs(T[2, 2]))) - 15.0) < 1e-9)
T = sw.camera_pose_W((1.0, 0.3), 0.45, 0.0, 0.0, 40.0)
from chain_calib.session import describe_view
check("spin 40 reads back as spin 40 through session.describe_view", abs(describe_view(invert_T(T)).spin_deg - 40.0) < 1e-9,
      "%.1f" % describe_view(invert_T(T)).spin_deg)
Tt = sw.camera_pose_W((1.0, 0.3), 0.45, 15.0, 90.0, 40.0)
vv = describe_view(invert_T(Tt), [(1.0, 0.3)])
check("tilt 15 / az +y / spin 40 reads back as tilt 15, azimuth 90, offset 0", abs(vv.tilt_deg - 15) < 1e-6 and abs(vv.azimuth_deg - 90) < 1e-6
      and vv.xy_offset_mm < 1e-6, "tilt %.1f az %.1f off %.2f mm" % (vv.tilt_deg, vv.azimuth_deg, vv.xy_offset_mm))

print("\n== 2. plan over the whitelist with the real chain ==")
cfg = sw.SweepCfg()
targets, rejected, rules = sw.plan(T_ab2W, H, T_start, sheet, K_hand, D_hand, cfg, spin_cur)
print("  %d accepted, %d rejected" % (len(targets), len(rejected)))
for v in rejected:
    print("     rejected %s: %s" % (v.label, v.reason))
check("at least 25 of the 35 views are feasible", len(targets) >= 25, "%d" % len(targets))
if not targets:
    sys.exit("no targets — nothing else can be checked")
check("every accepted view's orientation is within 60 deg of the start", all(v.rot_from_start_deg <= 60.0 + 1e-6 for v in targets),
      "max %.1f" % max(v.rot_from_start_deg for v in targets))
check("… and the tilted + spun ones are genuinely rotated (>= 30 deg for the 40-deg spins)",
      any(v.rot_from_start_deg > 30 for v in targets))
check("every accepted view passes check_T", all(sw.check_T(v.T_ab2ee, rules) is None for v in targets))
check("every accepted view predicts >= 2 whitelisted tags, none outside the whitelist",
      all(len(v.predicted_tags) >= 2 and set(v.predicted_tags) <= set(cfg.tags) for v in targets))
body = [v for v in targets + rejected if v.label.endswith("_body") or "_body_" in v.label]
check("tilts toward the arm base are reduced to 8 deg (one direction per tag)", len(body) >= 5 and all(v.tilt_deg <= 8.0 for v in body),
      "%d views: %s" % (len(body), sorted({v.azimuth_deg for v in body})))
tags_covered = {v.tag for v in targets}
check("all five tags are viewed", tags_covered == set(cfg.tags), "%s" % sorted(tags_covered))
# the tool points really are above the sheet
worst = min(float(np.dot(rules.up_ab, (v.T_ab2ee[:3, :3] @ p + v.T_ab2ee[:3, 3]) - rules.origin_ab)) for v in targets for p in rules.tool_pts)
check("lowest tool point over all views >= 0.12 m above the sheet", worst >= 0.12, "%.3f m" % worst)
check("body-clearance rule holds on every target", all(vc.body_clearance_ok(v.pose)[0] for v in targets))
# a rule that would bite: a 90 deg spin is refused at plan time
try:
    sw.plan(T_ab2W, H, T_start, sheet, K_hand, D_hand, sw.SweepCfg(spins_rel_deg=[0, 90]), spin_cur); check("a 90 deg spin is refused", False)
except SystemExit as e:
    check("a 90 deg spin is refused", "60" in str(e), str(e))
cfg30 = sw.SweepCfg(max_rot_from_start_deg=30.0)
t30, r30, _ = sw.plan(T_ab2W, H, T_start, sheet, K_hand, D_hand, cfg30, spin_cur)
check("--max-rot 30 rejects the 40-deg spins by the rotation rule", any("orientation" in (v.reason or "") for v in r30) and len(t30) < len(targets),
      "%d accepted" % len(t30))
# nearest-first: the first target is close to the start
d0 = float(np.linalg.norm(targets[0].T_ab2ee[:3, 3] - T_start[:3, 3]))
check("first target within 0.50 m of the start flange", d0 <= 0.50, "%.2f m" % d0)

print("\n== 3. chunks ==")
c = sw.chunks(T_start, targets[-1].T_ab2ee, cfg)
steps = [np.linalg.norm(b[:3, 3] - a[:3, 3]) for a, b in zip([T_start] + c[:-1], c)]
angs = [sw.rot_angle_deg(a[:3, :3], b[:3, :3]) for a, b in zip([T_start] + c[:-1], c)]
check("chunks are <= 0.20 m / 30 deg and end exactly at the target", max(steps) <= 0.2 + 1e-9 and max(angs) <= 30 + 1e-6
      and np.allclose(c[-1], targets[-1].T_ab2ee), "%d chunks, max %.3f m / %.1f deg" % (len(c), max(steps), max(angs)))
check("a move to itself is one chunk", len(sw.chunks(T_start, T_start, cfg)) == 1)

print("\n== 4. the runner against a fake arm + rendered detections ==")


class FakeArm:
    def __init__(self, T, fail_label=None):
        self.T = T.copy(); self.moves = []; self.fail_at = None
    def get_T(self):
        return self.T.copy()
    def move(self, pose):
        T = pose_fr5_to_matrix_m(pose)
        d = np.linalg.norm(T[:3, 3] - self.T[:3, 3]); a = sw.rot_angle_deg(self.T[:3, :3], T[:3, :3])
        self.moves.append((d, a))
        if self.fail_at is not None and np.allclose(T, self.fail_at):
            raise RuntimeError("MoveL error 112")
        self.T = T


def make_capture(arm, whitelist, rng):
    def capture(label):
        T_hc2W = H @ invert_T(arm.T) @ T_ab2W
        vis = SH.visible_tags(T_hc2W, sheet, K_hand, (640, 480), D_hand, 12.0)
        allowed = [k for k in vis if k in set(whitelist)]         # what Session.camera_T(only=) leaves
        if not allowed:
            raise RuntimeError("hand_cam sees %s but none of the allowed tags" % vis)
        hc = {k: v + rng.normal(0, 0.05, (4, 2)) for k, v in SH.project_sheet(T_hc2W, sheet, K_hand, D_hand, allowed).items()}
        fc = SH.project_sheet(T_fc2W, sheet, K_front, None, [200])
        h = SH.multi_tag_pnp(hc, sheet, K_hand, D_hand); f = SH.multi_tag_pnp(fc, sheet, K_front, None)
        s = CC.ChainSample(label, matrix_m_to_pose_fr5(arm.T), h.T_cam2W, f.T_cam2W, 0.0, hand_corners=hc, front_corners=fc,
                           hand_rms_px=h.rms_px, front_rms_px=f.rms_px, joints_deg=[0.0] * 6)
        tc = lambda cs: {k: SH.TagCorners(k, v, 20, 0.05) for k, v in cs.items()}
        return s, dict(hand=h, front=f, hand_tags=tc(hc), front_tags=tc(fc), n_hand=20, n_front=20)
    return capture


rng = np.random.RandomState(1)
arm = FakeArm(T_start)
got = []
logs = []
res = sw.execute(targets, rules, cfg, get_T=arm.get_T, move=arm.move, capture=make_capture(arm, cfg.tags, rng),
                 on_sample=lambda s, info, v: got.append((s, v)), settle=lambda: None, log=logs.append,
                 return_to=T_start, front_ref_t=T_fc2W[:3, 3])
check("every planned view captured", res["ok"] == len(targets) and res["move_failed"] == 0 and res["capture_failed"] == 0, "%s" % res)
# the detour: two legal poses whose straight line crosses the body-clearance line
Ta = np.eye(4); Ta[:3, :3] = T_start[:3, :3]; Ta[:3, 3] = [-0.39, 0.39, 0.03]        # reach 0.55, above the plane + margin
Tb = np.eye(4); Tb[:3, :3] = T_start[:3, :3]; Tb[:3, 3] = [-0.51, 0.51, -0.10]      # reach 0.72, below it
rules2 = sw.make_rules(T_ab2W, Ta, cfg)
check("both detour endpoints are legal", sw.check_T(Ta, rules2) is None and sw.check_T(Tb, rules2) is None,
      "%s / %s" % (sw.check_T(Ta, rules2), sw.check_T(Tb, rules2)))
path, why = sw.safe_path(Ta, Tb, rules2, cfg)
check("a straight path that dips under the line at short reach becomes an up-across-down detour",
      why is None and path[0] == "via" and all(sw.check_T(T, rules2) is None for T in path[1]) and np.allclose(path[1][-1], Tb),
      "%s" % (why or "%d chunks" % len(path[1])))
path2, why2 = sw.safe_path(Ta, Ta, rules2, cfg)
check("a legal straight path is used as is", why2 is None and path2[0] != "via")
check("every sample's hand corners are whitelisted tags only, >= 2 of them",
      all(set(s.hand_corners) <= set(cfg.tags) and len(s.hand_corners) >= 2 for s, _ in got))
check("no MoveL chunk exceeded 0.20 m / 30 deg", max(d for d, _ in arm.moves) <= 0.2 + 1e-9 and max(a for _, a in arm.moves) <= 30 + 1e-6,
      "%d moves, max %.3f m / %.1f deg" % (len(arm.moves), max(d for d, _ in arm.moves), max(a for _, a in arm.moves)))
check("the arm is back at the start pose", np.allclose(arm.T, T_start, atol=1e-9))
check("no front_cam drift flagged with a still base", not any("moved" in m for m in logs))
ok_cov, lines = coverage([s for s, _ in got], sheet=sheet)
check("coverage says READY to solve", ok_cov, lines[0] + " / " + lines[1])
rot = CC.rotation_diversity_deg([s for s, _ in got])
check("rotation diversity between views >= 40 deg (the hand / base split needs tens of degrees)", rot >= 40, "%.0f deg" % rot)

# a failed move skips that view and the run goes on; Ctrl-C leaves the arm
arm2 = FakeArm(T_start); arm2.fail_at = targets[3].T_ab2ee
got2 = []; logs2 = []
res2 = sw.execute(targets[:6], rules, cfg, get_T=arm2.get_T, move=arm2.move, capture=make_capture(arm2, cfg.tags, rng),
                  on_sample=lambda s, info, v: got2.append(v.label), settle=lambda: None, log=logs2.append, return_to=None)
check("a refused MoveL skips that view only", res2["ok"] == 5 and res2["move_failed"] == 1 and targets[3].label not in got2
      and any("move failed" in m for m in logs2), "%s" % res2)
check("--no-return leaves the arm at the last view", np.allclose(arm2.T, targets[5].T_ab2ee))


class Interrupting(FakeArm):
    def move(self, pose):
        if len(self.moves) >= 3:
            raise KeyboardInterrupt
        super().move(pose)


arm3 = Interrupting(T_start); n_before = None
res3 = sw.execute(targets[:6], rules, cfg, get_T=arm3.get_T, move=arm3.move, capture=make_capture(arm3, cfg.tags, rng),
                  on_sample=lambda s, info, v: None, settle=lambda: None, log=lambda m: None, return_to=T_start)
check("Ctrl-C stops the run, no return move", res3["stopped"] and len(arm3.moves) == 3 and not np.allclose(arm3.T, T_start))

# a capture that sees one whitelisted tag only is not stored
arm4 = FakeArm(T_start)
cap = make_capture(arm4, [301], rng)
res4 = sw.execute(targets[:2], rules, cfg, get_T=arm4.get_T, move=arm4.move, capture=cap, on_sample=lambda *a: None,
                  settle=lambda: None, log=lambda m: None, return_to=None)
check("a view with < 2 whitelisted tags in the image is a capture failure, not a sample", res4["ok"] == 0 and res4["capture_failed"] == 2)

# a path whose chunk would dip under the body-clearance line is skipped before any move
low = targets[0].T_ab2ee.copy(); low[2, 3] = -0.30; low[0, 3] = 0.20; low[1, 3] = 0.20
bad_view = sw.View("bad", 301, 0.45, 0, 0, 0); bad_view.T_ab2ee = low; bad_view.pose = matrix_m_to_pose_fr5(low)
bad_view.rot_from_start_deg = 0.0
arm5 = FakeArm(T_start)
res5 = sw.execute([bad_view], rules, cfg, get_T=arm5.get_T, move=arm5.move, capture=make_capture(arm5, cfg.tags, rng),
                  on_sample=lambda *a: None, settle=lambda: None, log=lambda m: None, return_to=None)
check("a target failing the rules is skipped without moving", res5["move_failed"] == 1 and not arm5.moves)

print("\n%d ok, %d failed" % (N_OK, N_FAIL))
sys.exit(1 if N_FAIL else 0)
