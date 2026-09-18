#!/usr/bin/env python3
"""Keyence beam-angle sweep THROUGH arm_node (/arm/jog_cmd + /arm/state),
so no second Fairino RPC connection is opened while the stack is up.

Same method as tools/measure_keyence_angle.py: symmetric sweep about the
current pose along TOOL Z (here: base z, the tool must be vertical), mean
reading at each stop, least squares -> k = d(reading)/d(toolZ), theta = acos(1/k).
"""
import argparse, json, math, sys, time, threading
import numpy as np
import rospy
from std_msgs.msg import Float32, String
from robot_msgs.msg import ArmState

SENTINEL = 90.0  # |reading| >= this = out of range (robot.yaml keyence.invalid_abs_mm)

class Watch:
    def __init__(self):
        self.lock = threading.Lock(); self.count = 0; self.samples = None
        self.state = None; self.last = None
    def cb(self, m):
        with self.lock:
            self.count += 1; self.last = m.data
            if self.samples is not None: self.samples.append(m.data)
    def st(self, m): self.state = m

def avg(w, seconds):
    vals = []
    with w.lock: n0 = w.count; w.samples = vals
    time.sleep(seconds)
    with w.lock: w.samples = None; n = w.count - n0
    if n < 5: return None, f'only {n} samples'
    a = np.array(vals)
    if np.any(np.abs(a) >= SENTINEL): return None, 'out of range inside window'
    return (a.mean(), a.std(), n), None

def jog(pub, w, delta, vel, timeout=30):
    seq0 = w.state.motion_seq
    pub.publish(String(json.dumps({'axis': 'z', 'delta': float(delta), 'vel': vel, 'acc': 20})))
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = w.state
        if s.motion_seq > seq0:
            return bool(s.result_success), s.result_message
        time.sleep(0.02)
    return False, 'timeout waiting for motion_seq'

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', type=float, default=0.5)
    ap.add_argument('--points', type=int, default=5)
    ap.add_argument('--settle', type=float, default=1.0)
    ap.add_argument('--window', type=float, default=0.8)
    ap.add_argument('--vel', type=float, default=5)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--yes', action='store_true')
    ap.add_argument('--offsets', type=str, default=None, help='comma list of tool-Z offsets (mm) instead of the symmetric sweep')
    a = ap.parse_args()
    rospy.init_node('keyence_angle_via_node', anonymous=True)
    w = Watch()
    rospy.Subscriber('/keyence/value', Float32, w.cb, queue_size=20)
    rospy.Subscriber('/arm/state', ArmState, w.st, queue_size=1)
    pub = rospy.Publisher('/arm/jog_cmd', String, queue_size=1)
    t0 = time.time()
    while (w.state is None or w.count < 5) and time.time() - t0 < 5: time.sleep(0.05)
    if w.state is None: sys.exit('no /arm/state'); 
    if w.count < 5: sys.exit('no /keyence/value')
    s = w.state
    if not s.pose_valid or s.busy: sys.exit(f'arm pose_valid={s.pose_valid} busy={s.busy}')
    x, y, z, rx, ry, rz = s.tcp_pose
    # tool Z in base frame from the Fairino rpy (Rz*Ry*Rx intrinsic)
    from scipy.spatial.transform import Rotation as R
    Rm = R.from_euler('ZYX', [rz, ry, rx], degrees=True)
    Rm = Rm.as_matrix() if hasattr(Rm, 'as_matrix') else Rm.as_dcm()
    tz = Rm @ np.array([0, 0, 1.0])
    print(f'TCP z={z:.2f} mm  rpy=({rx:.2f},{ry:.2f},{rz:.2f})  tool Z in base = {np.round(tz,3)}')
    if abs(tz[2]) < 0.995:
        sys.exit('tool is not vertical (|toolZ.z| < 0.995) — this script sweeps base z only')
    sign = 1.0 if tz[2] > 0 else -1.0   # base-z jog per +1 mm of tool Z
    r0, err = avg(w, 1.0)
    if r0 is None: sys.exit(f'reading not usable at the start: {err}  (last {w.last})')
    print(f'reading at start: {r0[0]:+.3f} mm  (sd {r0[1]*1000:.0f} um, {r0[2]} samples)')
    if a.dry_run: return
    half = (a.points - 1) // 2
    offs = [(i - half) * a.step for i in range(a.points)]
    if a.offsets: offs = [float(v) for v in a.offsets.split(',')]
    print(f'sweep tool-Z offsets {offs} mm (+ = toward the surface), base z jog sign {sign:+.0f}, vel {a.vel}%')
    if not a.yes and input('proceed? [y/N] ').strip().lower() != 'y': return
    cur = 0.0; rows = []; failures = []
    z_start = z
    try:
        for off in offs:
            d = off - cur
            if abs(d) > 1e-6:
                ok, msg = jog(pub, w, sign * d, a.vel)
                if not ok: failures.append((off, msg)); print(f'  {off:+.2f}: jog FAILED {msg}'); continue
                cur = off
            time.sleep(a.settle)
            zt = (w.state.tcp_pose[2] - z_start) * sign   # actual tool-Z offset from the arm
            r, err = avg(w, a.window)
            if r is None: failures.append((off, err)); print(f'  {off:+.2f}: {err}'); continue
            rows.append((off, zt, r[0], r[1], r[2]))
            print(f'  cmd {off:+.2f}  actual {zt:+.3f}  reading {r[0]:+.3f} mm  sd {r[1]*1000:.0f} um  n={r[2]}')
    finally:
        if abs(cur) > 1e-6:
            ok, msg = jog(pub, w, -sign * cur, a.vel)
            print('returned to start.' if ok else f'[WARN] return jog: {msg}')
    if len(rows) < 3: sys.exit(f'not enough points ({len(rows)}); failures {failures}')
    zz = np.array([r[1] for r in rows]); vv = np.array([r[2] for r in rows])
    k, b = np.polyfit(zz, vv, 1)
    res = vv - (k * zz + b); r2 = 1 - res.var() / vv.var() if vv.var() > 0 else float('nan')
    print(f'\n  k = d(reading)/d(toolZ) = {k:+.4f} mm/mm   R^2 {r2:.5f}   rms {res.std()*1000:.0f} um')
    if abs(k) < 1: print('  |k| < 1: NOT a valid beam angle (wrong axis / surface moved / scaling)'); return
    th = math.degrees(math.acos(1 / abs(k)))
    print(f'  beam angle vs tool Z = {th:.2f} deg  (vs surface {90-th:.2f} deg)   cos = {1/abs(k):.4f}')
    print(f'  keyence_dir must be {-math.copysign(1, k):+.1f}')
    if failures: print(f'  failures: {failures}')

if __name__ == '__main__': main()
