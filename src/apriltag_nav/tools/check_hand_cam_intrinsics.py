#!/usr/bin/env python3
"""Offline check of tools/hand_cam_intrinsics.py: a rendered checkerboard is detected with the right corner
order, synthetic views with a known (K, D) are recovered by `solve` within its own acceptance rule, an
outlier view is dropped, the split checks pass, and --apply rewrites only the numbers of the robot.yaml block."""
import importlib.util, json, os, sys, tempfile, subprocess
import numpy as np, cv2, yaml

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("hci", os.path.join(HERE, "hand_cam_intrinsics.py"))
hci = importlib.util.module_from_spec(spec); spec.loader.exec_module(hci)
n_ok = 0; n_fail = 0
def check(name, cond, detail=""):
    global n_ok, n_fail
    n_ok += cond; n_fail += (not cond)
    print("  %s %s %s" % ("ok  " if cond else "FAIL", name, detail))

K_true = np.array([[600.0, 0, 323.0], [0, 602.0, 237.0], [0, 0, 1]]); D_true = np.array([0.15, -0.30, 0, 0, 0])
WH = (640, 480)
rs = np.random.RandomState(3)
board = hci.Chessboard(9, 6, 25.0)

def render_view(rvec, tvec, noise=0.15):
    p, _ = cv2.projectPoints(board.obj * 1e-3, rvec, tvec, K_true, D_true)
    return p.reshape(-1, 2) + rs.normal(0, noise, (len(board.obj), 2))

# 1. detection on a rendered board: draw the checkerboard through a homography, detect, compare corner order
print("1. rendered checkerboard")
tile = 40; cols, rows = 10, 7
img = np.full(((rows + 2) * tile, (cols + 2) * tile), 255, np.uint8)
for r in range(rows):
    for c in range(cols):
        if (r + c) % 2 == 0:
            img[(r + 1) * tile:(r + 2) * tile, (c + 1) * tile:(c + 2) * tile] = 0
src = np.float32([[tile, tile], [(cols + 1) * tile, tile], [(cols + 1) * tile, (rows + 1) * tile], [tile, (rows + 1) * tile]])
dst = np.float32([[120, 90], [520, 110], [500, 400], [140, 380]])
Hm = cv2.getPerspectiveTransform(src, dst)
view = cv2.warpPerspective(img, Hm, WH, borderValue=255)
view = cv2.GaussianBlur(view, (3, 3), 0)
det = board.detect(view)
check("board detected", det is not None)
if det is not None:
    obj, im, n = det
    check("54 corners", n == 54 and im.shape == (54, 2))
    # inner corners in board pixels -> through H: expected positions; detection may start from either end (180 deg ambiguity is broken by odd x even)
    inner = np.array([[(c + 2) * tile, (r + 2) * tile] for r in range(rows - 1) for c in range(cols - 1)], np.float32)
    exp = cv2.perspectiveTransform(inner.reshape(-1, 1, 2), Hm).reshape(-1, 2)
    d = np.linalg.norm(im - exp, axis=1)
    d2 = np.linalg.norm(im - exp[::-1], axis=1)
    check("corner order matches the object grid (or its 180 deg twin)", min(d.max(), d2.max()) < 1.5, "max %.2f / %.2f px" % (d.max(), d2.max()))

# 2. synthetic session: 40 views over three ranges, tilts, corners
# 1b. canonical order + order-insensitive stillness (the symmetric 8x12 grid comes back reversed on some frames)
print("1b. corner order")
a = np.array([[100.0 + 10 * c, 80.0 + 10 * r] for r in range(8) for c in range(12)])
hist = [a + 0.1 * k for k in range(4)]
def still_px(cur):
    return float(max(min(np.abs(h - cur).max(), np.abs(h - cur[::-1]).max()) for h in hist[-4:]))
check("reversed-order frame reads as still", still_px(a[::-1] + 0.3) < 1.0, "%.2f px" % still_px(a[::-1] + 0.3))
check("a moved frame is not still", still_px(a + 3.0) > 1.0)
if det is not None:
    check("canonical order: first corner nearer the origin than the last", im[0].sum() <= im[-1].sum())
print("2. synthetic views -> solve")
tmp = tempfile.mkdtemp(); os.makedirs(os.path.join(tmp, "frames"))
views = {}
i = 0
for rng in (0.30, 0.45, 0.55):
    spread = 0.12 if rng < 0.4 else (0.22 if rng < 0.5 else 0.27)     # keep the board inside the frame at 0.30 m
    for tilt in (0, 20, 35):
        for (u, v) in ((0.5, 0.5), (0.5 - spread, 0.5 - spread), (0.5 + spread, 0.5 - spread),
                       (0.5 - spread, 0.5 + spread), (0.5 + spread, 0.5 + spread), (0.5, 0.5 - spread), (0.5, 0.5 + spread)):
            ax = rs.normal(size=3); ax /= np.linalg.norm(ax); ax[2] *= 0.2
            rvec = ax * np.radians(tilt) + np.array([0, 0, rs.uniform(-0.3, 0.3)])
            # place the board centre at pixel (u, v) at range rng
            cx = (u * WH[0] - K_true[0, 2]) / K_true[0, 0] * rng; cy = (v * WH[1] - K_true[1, 2]) / K_true[1, 1] * rng
            R, _ = cv2.Rodrigues(rvec)
            tvec = np.array([cx, cy, rng]) - R @ (board.obj.mean(0) * 1e-3)
            im = render_view(rvec, tvec)
            if im.min() < 0 or im[:, 0].max() > WH[0] or im[:, 1].max() > WH[1]:
                continue
            geo = hci.view_geometry(board.obj, im, K_true, WH)
            views["v%02d" % i] = dict(obj_mm=board.obj.tolist(), img_px=im.tolist(), n=54, motion_px=0.0, **geo)
            i += 1
# one moved-arm outlier
bad = dict(views["v03"]); bad["img_px"] = (np.asarray(bad["img_px"]) + rs.normal(0, 2.5, (54, 2))).tolist()
views["v%02d" % i] = bad
json.dump(views, open(os.path.join(tmp, "corners.json"), "w"))
yaml.safe_dump(dict(image_size=list(WH), driver_K=[609.3, 0, 321.5, 0, 608.6, 238.7, 0, 0, 1], driver_D=[0] * 5,
                    target="checkerboard 9x6 inner corners, 25.000 mm squares", target_kind="chess"),
               open(os.path.join(tmp, "meta.yaml"), "w"))
check("view_geometry range / tilt sane", abs(views["v00"]["range_m"] - 0.30) < 0.01 and views["v00"]["tilt_deg"] < 5.0,
      "range %.3f tilt %.1f" % (views["v00"]["range_m"], views["v00"]["tilt_deg"]))
tilted = [v for v in views.values() if v["tilt_deg"] > 25]
check("coverage table sees tilts and corners", len(tilted) >= 8 and sum(1 for v in views.values() if v["pos"] == "TL") >= 3)

# 3. --apply on a robot.yaml copy with comments inside the block
cfg = os.path.join(tmp, "robot.yaml")
open(cfg, "w").write("""robot_camera:
  # leading comment
  intrinsics_override:
    hand_cam:
      enabled: true
      image_size: [640, 480]

      K: [601.72, 0.0, 322.02,
          0.0, 603.87, 238.46,
          0.0, 0.0, 1.0]
      # a comment between K and D — must survive
        # indented note comment
      D: [0.1598, -0.3222, 0.0, 0.0, 0.0]
      note: "old note"
  quad_decimate:
    front_cam: 2.0
""")
out = subprocess.run([sys.executable, os.path.join(HERE, "hand_cam_intrinsics.py"), "solve", tmp, "--apply", "--config", cfg],
                     capture_output=True, text=True)
print("\n".join("    | " + l for l in out.stdout.splitlines()[-22:]))
check("solve exits 0", out.returncode == 0, out.stderr[-300:])
res = yaml.safe_load(open(os.path.join(tmp, "result.yaml")))
K = np.array(res["K"]); D = np.array(res["D"])
check("fx / fy within 1 px", abs(K[0, 0] - 600) < 1 and abs(K[1, 1] - 602) < 1, "fx %.2f fy %.2f" % (K[0, 0], K[1, 1]))
check("cx / cy within 1 px", abs(K[0, 2] - 323) < 1 and abs(K[1, 2] - 237) < 1, "cx %.2f cy %.2f" % (K[0, 2], K[1, 2]))
check("k1 / k2 within 0.005 / 0.02", abs(D[0] - 0.15) < 0.005 and abs(D[1] + 0.30) < 0.02, "k1 %.4f k2 %.4f" % (D[0], D[1]))
check("rms at the noise floor", res["rms"] < 0.25, "%.3f" % res["rms"])
check("the outlier view was dropped", "v%02d" % i not in res["views"] and "dropping 1 outlier" in out.stdout)
check("VERDICT ACCEPT", "VERDICT: ACCEPT" in out.stdout)
txt = open(cfg).read()
c2 = yaml.safe_load(txt)["robot_camera"]
check("applied K / D in the yaml", abs(c2["intrinsics_override"]["hand_cam"]["K"][0] - K[0, 0]) < 0.01
      and abs(c2["intrinsics_override"]["hand_cam"]["D"][1] - D[1]) < 1e-4)
check("comments survived", "# a comment between K and D" in txt and "# leading comment" in txt and "# indented note comment" in txt)
check("other keys untouched", c2["quad_decimate"]["front_cam"] == 2.0 and c2["intrinsics_override"]["hand_cam"]["enabled"] is True)
check("note replaced", "old note" not in txt and "calibrateCamera" in c2["intrinsics_override"]["hand_cam"]["note"])

# 4. board file
bp = os.path.join(tmp, "board.png")
out = subprocess.run([sys.executable, os.path.join(HERE, "hand_cam_intrinsics.py"), "board", "--out", bp], capture_output=True, text=True)
bimg = cv2.imread(bp, 0)
check("board png written at A4 300 dpi", out.returncode == 0 and bimg is not None and bimg.shape == (2480, 3508), str(None if bimg is None else bimg.shape))
if bimg is not None:
    small = cv2.resize(bimg, (877, 620))
    check("printed board detected as 9x6", cv2.findChessboardCornersSB(small, (9, 6))[0])

print("\n%d ok, %d fail" % (n_ok, n_fail)); sys.exit(1 if n_fail else 0)
