#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Measure the Keyence beam angle by moving the tool and watching the reading.

Why this is measurable rather than lookup-able
----------------------------------------------
The DL-EN1 measures along its LASER BEAM, but _adjust_distance_to_surface()
corrects by translating along TOOL Z. The beam is mounted oblique, so the two
differ by a projection. No URDF, TCP or config in this workspace records the
mounting angle, so it has to come from the robot itself.

What the loop actually needs is the sensitivity

    k = d(reading) / d(tool Z)        [mm of reading per mm of motion]

because the correct step for a reading `val` is simply val / k. The angle is
just an interpretation of the same number:

    reading = perpendicular_error / cos(theta)   ->   k = 1 / cos(theta)
    theta   = acos(1 / k)

k >= 1 always: a beam at normal incidence gives k = 1, and any tilt makes the
reading move FASTER than the tool. A fitted k < 1 therefore means something is
wrong (wrong axis, surface moved, sensor scaling) rather than a small angle.

Method
------
A symmetric sweep about the starting pose: the tool visits -2d, -d, 0, +d, +2d
along its own Z axis, averaging the reading at each stop, then returns exactly
to where it started. Least squares through the five points gives k.

Symmetric on purpose: the excursion is tiny (default +/-1.0 mm) and centred on
a pose you already put in range, so neither direction can drive the tool into
the surface, and the fit does not depend on knowing in advance which way is
"away". The sign of k also settles the keyence_dir convention.

Caveat: this measures the EFFECTIVE angle between the beam and tool Z at this
pose, which folds in any tilt of the surface relative to tool Z. Measure on the
real workpiece, at a pose representative of your scan points, ideally with the
tool normal to the surface.

Usage:
    python3 measure_keyence_angle.py                 # +/-1.0 mm, 5 points
    python3 measure_keyence_angle.py --step 0.3      # smaller excursion
    python3 measure_keyence_angle.py --dry-run       # check the reading only
"""

import argparse
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'src'))

import numpy as np                                        # noqa: E402
from scipy.spatial.transform import Rotation as R         # noqa: E402

ARM_IP = '192.168.58.2'
TOOL_ID = 1          # vision_tip — same tool the scan loop uses
SENTINEL = 99999.0   # |reading| at/over this is the out-of-range value


def _avg_reading(watch, seconds, min_samples=5):
    """Mean reading over a window; None if it went out of range or went quiet."""
    import rospy
    n0 = watch.count
    vals = []
    t_end = time.time() + seconds
    watch.samples = vals
    while time.time() < t_end and not rospy.is_shutdown():
        time.sleep(0.02)
    watch.samples = None
    if watch.count - n0 < min_samples:
        return None, f"only {watch.count - n0} samples in {seconds:.1f}s"
    if not vals:
        return None, 'no samples captured'
    arr = np.array(vals, dtype=float)
    if np.any(np.abs(arr) >= SENTINEL):
        return None, 'out-of-range sentinel during the window'
    return float(arr.mean()), f"n={len(arr)} sd={arr.std():.4f}"


class Watch:
    """Collects keyence/value, and mirrors into .samples while a window is open."""

    def __init__(self):
        self.count = 0
        self.last = None
        self.samples = None

    def cb(self, msg):
        v = float(msg.data)
        self.count += 1
        self.last = v
        if self.samples is not None:
            self.samples.append(v)


def _tool_z_axis(pose):
    """Unit tool-Z in base frame, from a Fairino TCP pose (mm + deg, xyz euler)."""
    _, _, _, rx, ry, rz = pose
    return R.from_euler('xyz', [rx, ry, rz], degrees=True).as_matrix()[:, 2]


def _offset_pose(pose, z_vec, delta_mm):
    x, y, z, rx, ry, rz = pose
    return [x + z_vec[0] * delta_mm,
            y + z_vec[1] * delta_mm,
            z + z_vec[2] * delta_mm,
            rx, ry, rz]


def main():
    ap = argparse.ArgumentParser(
        description='Measure the Keyence beam angle / sensitivity on the robot.')
    ap.add_argument('--step', type=float, default=0.5,
                    help='spacing between sample points in mm (default 0.5, '
                         'so the sweep spans +/-1.0 mm)')
    ap.add_argument('--points', type=int, default=5,
                    help='number of sample points, odd (default 5)')
    ap.add_argument('--settle', type=float, default=1.0,
                    help='seconds to wait after each move before averaging')
    ap.add_argument('--window', type=float, default=0.8,
                    help='seconds to average the reading at each stop')
    ap.add_argument('--speed', type=int, default=5,
                    help='arm speed percent for the probe moves (default 5)')
    ap.add_argument('--dry-run', action='store_true',
                    help='report the current reading and planned sweep, no motion')
    ap.add_argument('--yes', action='store_true', help='skip the confirmation')
    args = ap.parse_args()

    if args.points < 3 or args.points % 2 == 0:
        ap.error('--points must be an odd number >= 3')

    import rospy
    import rosgraph
    from std_msgs.msg import Float32
    from apriltag_nav.paths import add_fairino_sdk_to_path

    if not rosgraph.is_master_online():
        print('[FAIL] ROS master offline')
        sys.exit(1)
    rospy.init_node('measure_keyence_angle', anonymous=True, disable_signals=True)

    # ---------- sensor ----------
    watch = Watch()
    rospy.Subscriber('/keyence/value', Float32, watch.cb, queue_size=20)
    print('Waiting for /keyence/value ...')
    t_end = time.time() + 5.0
    while watch.count == 0 and time.time() < t_end:
        time.sleep(0.05)
    if watch.count == 0:
        print('[FAIL] no data on /keyence/value — start keyence_dlen1_node')
        sys.exit(1)
    if abs(watch.last) >= SENTINEL:
        print(f"[FAIL] reading is {watch.last:.1f} mm (out-of-range sentinel).")
        print('       Move the tool into the sensor range first, then re-run.')
        sys.exit(1)
    print(f"[ OK ] reading in range: {watch.last:.3f} mm")

    # ---------- arm ----------
    if not add_fairino_sdk_to_path():
        print('[FAIL] Fairino SDK not found')
        sys.exit(1)
    from fairino import Robot
    robot = Robot.RPC(ARM_IP)
    time.sleep(0.5)
    ret, start_pose = robot.GetActualTCPPose()
    if ret != 0:
        print(f"[FAIL] GetActualTCPPose returned {ret}")
        sys.exit(1)
    z_vec = _tool_z_axis(start_pose)
    print(f"[ OK ] TCP pose: {[round(v, 2) for v in start_pose]}")
    print(f"       tool Z in base frame: {[round(v, 4) for v in z_vec]}")

    half = (args.points - 1) // 2
    offsets = [round((i - half) * args.step, 4) for i in range(args.points)]
    print(f"\n  sweep along tool Z (mm): {offsets}")
    print(f"  max excursion from the current pose: {half * args.step:.2f} mm")

    if args.dry_run:
        print('\n--dry-run: no motion performed.')
        return

    # ---------- confirm ----------
    print('\n' + '!' * 68)
    print('  THE ARM WILL MOVE')
    print('!' * 68)
    print(f"\n  {args.points} MoveL steps of {args.step} mm along the TOOL Z axis,")
    print(f"  at speed {args.speed}%, staying within "
          f"+/-{half * args.step:.2f} mm of the current pose.")
    print('  Orientation is preserved; the tool returns to the start pose at the end.')
    print('\n  Check: the tool is over the workpiece and nothing can collide.')
    if not args.yes:
        try:
            if input("\n  Type 'yes' to proceed: ").strip().lower() != 'yes':
                print('  aborted.')
                return
        except EOFError:
            print('  no tty and --yes not given — aborting.')
            return

    # ---------- sweep ----------
    robot.SetSpeed(int(args.speed))
    measured = []      # (offset_mm, reading_mm)
    failures = []
    try:
        for off in offsets:
            target = _offset_pose(start_pose, z_vec, off)
            ret = robot.MoveL(target, tool=TOOL_ID, user=0)
            if ret != 0:
                failures.append((off, f"MoveL returned {ret}"))
                print(f"  offset {off:+.2f} mm -> MoveL FAILED ({ret})")
                continue
            time.sleep(args.settle)
            val, note = _avg_reading(watch, args.window)
            if val is None:
                failures.append((off, note))
                print(f"  offset {off:+.2f} mm -> reading unusable ({note})")
                continue
            measured.append((off, val))
            print(f"  offset {off:+.2f} mm -> reading {val:+.4f} mm   ({note})")
    finally:
        # Always put the tool back, including on Ctrl-C or a mid-sweep failure.
        print('\n  returning to the start pose ...')
        ret = robot.MoveL(list(start_pose), tool=TOOL_ID, user=0)
        print('  returned.' if ret == 0 else f"  [WARN] return MoveL returned {ret}")

    # ---------- fit ----------
    print('\n' + '=' * 68)
    print('Result')
    print('=' * 68)
    if len(measured) < 3:
        print(f"[FAIL] only {len(measured)} usable points — cannot fit.")
        for off, why in failures:
            print(f"       offset {off:+.2f}: {why}")
        sys.exit(1)

    x = np.array([m[0] for m in measured])
    y = np.array([m[1] for m in measured])
    k, intercept = np.polyfit(x, y, 1)
    resid = y - (k * x + intercept)
    ss_res = float(np.sum(resid ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    print(f"  points used     : {len(measured)}/{args.points}")
    print(f"  sensitivity k   : {k:+.4f} mm reading per mm tool-Z")
    print(f"  fit quality     : R^2 = {r2:.5f}, max residual "
          f"{np.max(np.abs(resid)):.4f} mm")

    if r2 < 0.98:
        print("\n  [WARN] poor linearity — the surface may be tilted, the tool")
        print("         may be moving off the measurement spot, or the sweep is")
        print("         too small relative to sensor noise. Try a larger --step.")

    ak = abs(k)
    if ak < 0.98:
        print(f"\n  [FAIL] |k| = {ak:.4f} < 1, which is geometrically impossible")
        print("         for a rigid beam and surface. Check that the sweep really")
        print("         moved along tool Z and that the surface did not shift.")
        sys.exit(1)

    cos_theta = 1.0 / ak
    theta = float(np.degrees(np.arccos(min(1.0, cos_theta))))
    print(f"\n  cos(theta)      : {cos_theta:.4f}")
    print(f"  BEAM ANGLE      : {theta:.2f} deg  <-- keyence.beam_angle_deg")

    # k = 1/cos is flat near 0 deg (dk/dtheta = sin/cos^2 -> 0), so a small
    # angle is poorly determined — but it is also the case where the correction
    # barely matters, so the method is most precise exactly where it counts.
    span = float(x.max() - x.min())
    if theta < 10.0:
        print("                    (near-normal incidence: the angle is "
              "noise-dominated here,")
        print(f"                     but cos = {cos_theta:.4f} so the "
              "correction is negligible anyway)")
    elif span < 3.0:
        # FR10 repeatability is ~+/-0.03 mm, which is the dominant error term:
        # a 2 mm span puts ~1.5% on k, i.e. roughly a degree at 45 deg.
        print(f"                    (swept {span:.1f} mm; arm repeatability "
              "~0.03 mm dominates the")
        print("                     error budget — re-run with a larger --step "
              "for a tighter estimate,")
        print("                     as long as the reading stays in range)")
    # --- the tuning the controller really uses ---
    def _param(name, fallback_key, default):
        try:
            return float(rospy.get_param(f"/arm_node/{name}")), '~param'
        except Exception:
            from apriltag_nav.paths import load_yaml_block
            return float(load_yaml_block('keyence').get(fallback_key, default)), \
                'robot.yaml'

    kp, kp_src = _param('keyence_kp', 'kp', 0.8)
    cur_dir, dir_src = _param('keyence_dir', 'dir', 1.0)

    # keyence_dir is the OPPOSITE sign of k. The controller steps by
    # val*dir*kp*cos, and val responds to a tool-Z move with slope k, so one step
    # leaves val*(1 + dir*kp). Convergence needs dir*kp < 0, i.e. dir = -sign(k).
    # Backwards does not merely fail to converge — it AMPLIFIES the error by
    # (1 + kp) every step, driving the tool into the surface until the step clamp
    # and max_steps run out.
    want_dir = -1.0 if k > 0 else +1.0
    print(f"\n  direction       : reading {'GROWS' if k > 0 else 'SHRINKS'} as the "
          f"tool moves +Z, so a positive")
    print(f"                    reading means too "
          f"{'CLOSE' if k > 0 else 'FAR'}, and the fix is to move "
          f"{'-' if k > 0 else '+'}Z.")
    print(f"  keyence_dir     : MUST be {want_dir:+.1f}   "
          f"(currently {cur_dir:+.1f} from {dir_src})")

    step_factor = abs(1.0 + cur_dir * kp)
    if cur_dir != want_dir:
        print("\n  *** [FAIL] keyence_dir has the WRONG SIGN. ***")
        print(f"      Each step would multiply the error by {step_factor:.1f}x")
        print("      and drive the tool INTO the workpiece. Fix it in")
        print("      mobile_manipulator.launch before enabling the loop.")
    else:
        print("  [ OK ] sign is correct.")

    # Loop gain. With the projection applied the reading's 1/cos factor cancels,
    # so the per-step residual is |1 + dir*kp| = |1 - kp| — the beam angle drops
    # out entirely. It only re-enters if beam_angle_deg is left at 0.
    print(f"\n  loop gain       : kp={kp} from {kp_src}")
    print(f"  residual/step   : |1 - kp| = {abs(1 - kp):.2f}"
          f"  (with the projection active)")
    if not 0.0 < kp < 2.0:
        print("  [FAIL] kp outside (0, 2) — does not converge.")
    elif kp > 1.5:
        print("  [WARN] kp > 1.5 — will overshoot and oscillate.")
    else:
        print("  [ OK ] stable.")
    print("\n  If beam_angle_deg is left at 0 instead, the true gain becomes")
    print(f"  kp/cos = {kp / cos_theta:.3f}, which diverges past "
          f"{np.degrees(np.arccos(min(1.0, kp / 2.0))):.1f} deg.")

    print("\n  Apply with, in config/robot.yaml:")
    print(f"      keyence:\n        beam_angle_deg: {theta:.2f}")
    print('=' * 68)


if __name__ == '__main__':
    main()
