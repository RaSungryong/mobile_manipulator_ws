#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Scan-point chain test: Basler + VISION lamp + Keyence + arm, in the linkage
order a real scan point uses.

The production sequence inside ArmController.execute_scan_points() is:

    pipeline.preopen()          -> /camera/set_active true (device opens, lamp OFF)
    SetSpeed + MoveJ            -> arm travels to the point
    sleep(stabilization_time)
    _adjust_distance_to_surface -> reads keyence/value, MoveL along tool Z, loop
    sleep(0.5)
    pipeline.scan_point()       -> /camera/capture (lamp ON -> grab -> lamp OFF)
                                   -> ONNX infer -> /scan/ra_value
    (after the last point)      -> move_to_home()
    pipeline.release()          -> /camera/set_active false (device closes)

Two modes, because only one of them moves the robot:

  probe (default)  NO ARM MOTION. Verifies the wiring at whatever pose the arm
                   is already in: nodes/services present, Keyence streaming,
                   and — the actual point of this script — that the VISION lamp
                   is bracketed around the shutter rather than merely toggled
                   somewhere nearby. Safe to run any time.

  point --move     Sends ONE real scan point through /arm/scan_command, i.e. the
                   production path, and records the whole timeline. THE ARM
                   MOVES. Read the warning it prints before confirming.

Usage:
    python3 test_scan_chain.py                    # probe, no motion
    python3 test_scan_chain.py --no-infer         # skip ONNX (faster)
    python3 test_scan_chain.py --move             # one scan point at the CURRENT pose
    python3 test_scan_chain.py --move --joints "-90.4,-144.6,144.9,-81.2,-90,0"
    python3 test_scan_chain.py --move --yes       # skip the interactive prompt

Exit code 0 when every required link in the chain checked out.
"""

import argparse
import json
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'src'))

from test_all_devices import Report, header, _info  # noqa: E402  (sibling tool)


# ============================================================
# EVENT TIMELINE
# ============================================================
class Timeline:
    """Timestamped events from several topics, printed in interleaved order.

    Ordering is what this test is about: 'lamp went on' and 'frames came back'
    are each individually meaningless — the question is whether the lamp
    bracketed the shutter.
    """

    def __init__(self):
        self.t0 = time.time()
        self.events = []      # (dt, source, text)

    def mark(self, source, text):
        self.events.append((time.time() - self.t0, source, text))

    def reset(self):
        self.t0 = time.time()
        self.events = []

    def dump(self, title='Timeline'):
        print(f"\n  --- {title} (t=0 at start) ---")
        for dt, source, text in self.events:
            print(f"    t+{dt:6.3f}s  {source:<18} {text}")

    def first(self, source, pred=None):
        """dt of the first event from source (optionally matching pred)."""
        for dt, s, text in self.events:
            if s == source and (pred is None or pred(text)):
                return dt
        return None

    def last(self, source, pred=None):
        found = None
        for dt, s, text in self.events:
            if s == source and (pred is None or pred(text)):
                found = dt
        return found


class KeyenceWatch:
    """Aggregates keyence/value — it streams at 30-60 Hz, far too fast to log raw."""

    def __init__(self, timeline=None, label='keyence'):
        self.count = 0
        self.last = None
        self.vmin = None
        self.vmax = None
        self.t_first = None
        self.timeline = timeline
        self.label = label
        self._last_logged = None

    def cb(self, msg):
        v = float(msg.data)
        self.count += 1
        self.last = v
        if self.t_first is None:
            self.t_first = time.time()
        self.vmin = v if self.vmin is None else min(self.vmin, v)
        self.vmax = v if self.vmax is None else max(self.vmax, v)
        # Log only meaningful changes, so the closed loop's steps stay visible.
        if self.timeline is not None:
            if self._last_logged is None or abs(v - self._last_logged) > 0.05:
                self._last_logged = v
                self.timeline.mark(self.label, f"value = {v:.3f} mm")


# ============================================================
# PRECONDITIONS
# ============================================================
def check_preconditions(rep, need_keyence):
    """Returns True when the chain can be exercised at all."""
    import rospy
    import rosnode

    header("1. Preconditions — nodes and services")

    try:
        alive = set(rosnode.get_node_names())
    except Exception as e:
        rep.add('pre', 'node list', 'FAIL', f"{type(e).__name__}: {e}")
        return False

    ok = True
    for node, required in (('/arm_node', True),
                           ('/basler_camera_node', True),
                           ('/crevis_io_node', True),
                           ('/keyence_dlen1_node', need_keyence)):
        if node in alive:
            rep.add('pre', f"node {node}", 'PASS', 'running')
        else:
            rep.add('pre', f"node {node}", 'FAIL' if required else 'WARN',
                    'not running')
            if required:
                ok = False

    for svc in ('/camera/capture',):
        try:
            rospy.wait_for_service(svc, timeout=2.0)
            rep.add('pre', f"service {svc}", 'PASS', 'advertised')
        except Exception:
            rep.add('pre', f"service {svc}", 'FAIL', 'not advertised')
            ok = False

    # A publisher-less keyence/value is the failure mode that matters here: the
    # closed loop degrades to "Keyence value NOT available, skipping adjustment"
    # and the scan silently proceeds at whatever distance the arm happened to
    # stop at, with no error anywhere in the scan result.
    try:
        import rosgraph
        master = rosgraph.Master('/test_scan_chain')
        state = master.getSystemState()
        publishers = {t: n for t, n in state[0]}
        who = publishers.get('/keyence/value', [])
        if who:
            rep.add('pre', '/keyence/value publisher', 'PASS', ', '.join(who))
        else:
            rep.add('pre', '/keyence/value publisher',
                    'FAIL' if need_keyence else 'WARN',
                    'NO PUBLISHER — the Keyence link in the chain is dead')
            if need_keyence:
                ok = False
    except Exception as e:
        rep.add('pre', '/keyence/value publisher', 'WARN',
                f"could not query master: {e}")

    return ok


# ============================================================
# STEP — KEYENCE STREAM
# ============================================================
def _effective_keyence_params(node='/arm_node'):
    """The tuning the CONTROLLER actually uses, not what robot.yaml says.

    mobile_manipulator.launch sets keyence_kp / keyence_tol / keyence_max_steps
    as ~params on arm_node, and a private param beats the robot.yaml
    fallback inside ArmController. Reading only the yaml would report kp 0.5
    when the running controller is using 0.8.
    """
    import rospy
    from apriltag_nav.paths import load_yaml_block

    y = load_yaml_block('keyence')
    out = {
        'kp':  float(y.get('kp', 0.8)),
        'tol': float(y.get('tolerance_mm', 0.2)),
        'beam_angle_deg': float(y.get('beam_angle_deg', 0.0)),
        'max_steps': float(y.get('max_steps', 10)),
        'max_step_mm': float(y.get('max_step_mm', 1.0)),
        'activate_threshold': float(y.get('activate_threshold', 5.0)),
    }
    src = 'robot.yaml'
    overridden = []
    for key, param in (('kp', 'keyence_kp'),
                       ('tol', 'keyence_tol'),
                       ('beam_angle_deg', 'keyence_beam_angle_deg'),
                       ('max_steps', 'keyence_max_steps'),
                       ('max_step_mm', 'keyence_max_step_mm'),
                       ('activate_threshold', 'keyence_activate_threshold')):
        try:
            val = rospy.get_param(f"{node}/{param}")
        except Exception:
            continue
        if val is not None and float(val) != out[key]:
            out[key] = float(val)
            overridden.append(param)
    if overridden:
        src = f"robot.yaml + {node} ~params ({', '.join(overridden)})"
    return out, src


def check_keyence(rep, duration):
    import rospy
    from std_msgs.msg import Float32

    header("2. Keyence stream — is the closed loop fed?")

    watch = KeyenceWatch()
    sub = rospy.Subscriber('/keyence/value', Float32, watch.cb, queue_size=10)
    print(f"  sampling /keyence/value for {duration:.1f}s ...")
    time.sleep(duration)
    sub.unregister()

    if watch.count == 0:
        rep.add('keyence', 'stream', 'FAIL',
                'no data — _adjust_distance_to_surface() will skip silently')
        _info("start it with:")
        _info("  rosrun apriltag_nav keyence_dlen1_node.py "
              "_host:=192.168.100.105 _port:=64000")
        return

    hz = watch.count / duration
    rep.add('keyence', 'stream', 'PASS',
            f"{watch.count} msg  {hz:.1f} Hz  last={watch.last:.3f} mm "
            f"(min {watch.vmin:.3f} / max {watch.vmax:.3f})")

    # The controller only engages the loop when |val| < keyence_activate_threshold
    # (default 5 mm) and stops once |val| <= tolerance. Outside that band the
    # scan runs with NO distance correction at all, which is the quiet failure.
    #
    # The published value is a deviation measured ALONG THE LASER BEAM, zero at
    # the sensor's own 10 mm standoff setting. The beam is mounted oblique, so
    # it is not the perpendicular distance to the surface — see robot.yaml
    # keyence.beam_angle_deg.
    import math
    cfg, src = _effective_keyence_params()
    _info(f"tuning source: {src}")
    tol = float(cfg['tol'])
    kp = float(cfg['kp'])
    beam_deg = float(cfg['beam_angle_deg'])
    threshold = float(cfg['activate_threshold'])
    cos_beam = math.cos(math.radians(beam_deg))
    v = watch.last

    # With the projection applied the reading's 1/cos cancels, so the loop gain
    # is kp alone. Without it the true gain is kp/cos(real angle) — and the real
    # angle is unknown by definition, which is what makes 0.0 the risky setting.
    if beam_deg == 0.0:
        rep.add('keyence', 'beam angle', 'WARN',
                f"0.0 — oblique mount NOT compensated; true gain is kp/cos = "
                f"{kp:.2f}/cos, diverging past "
                f"{math.degrees(math.acos(min(1.0, kp / 2.0))):.1f} deg. "
                "Measure with tools/measure_keyence_angle.py")
    else:
        rep.add('keyence', 'beam angle', 'PASS',
                f"{beam_deg} deg, cos={cos_beam:.4f} — projection active")
    rep.add('keyence', 'loop gain',
            'PASS' if 0 < kp <= 1.5 else ('WARN' if kp < 2.0 else 'FAIL'),
            f"kp={kp} -> per-step residual |1-kp| = {abs(1 - kp):.2f}"
            + ('' if 0 < kp < 2.0 else '  DOES NOT CONVERGE'))

    if abs(v) >= 99999.0:
        rep.add('keyence', 'closed loop would engage', 'WARN',
                f"{v:.1f} mm is the out-of-range sentinel — no target in view; "
                "adjustment will be skipped")
        return

    # Every threshold is perpendicular standoff mm, so project before comparing
    # — exactly what _adjust_distance_to_surface() does.
    perp = v * cos_beam
    shown = f"{v:.3f} mm beam -> {perp:.3f} mm perpendicular"

    if abs(perp) >= threshold:
        rep.add('keyence', 'closed loop would engage', 'WARN',
                f"{shown}; |perp| >= {threshold} mm activate threshold — "
                "adjustment will be SKIPPED at this pose")
    elif abs(perp) <= tol:
        rep.add('keyence', 'closed loop would engage', 'PASS',
                f"{shown}; |perp| <= {tol} mm tolerance — already on target")
    else:
        rep.add('keyence', 'closed loop would engage', 'PASS',
                f"{shown}; inside the {threshold} mm band and outside the "
                f"{tol} mm tolerance — the loop would correct")


# ============================================================
# STEP — LAMP / SHUTTER BRACKETING
# ============================================================
def check_lamp_shutter(rep, args):
    """Call /camera/capture and verify the lamp bracketed the grab."""
    import rospy
    from std_msgs.msg import Bool, String
    from robot_msgs.srv import CaptureImages

    header("3. Illumination linkage — is VISION bracketed around the shutter?")

    tl = Timeline()

    def led_cb(msg):
        tl.mark('led/vision', f"commanded {'ON' if msg.data else 'OFF'}")

    def ledall_cb(msg):
        # Hardware-side echo from crevis_io_node at ~10 Hz — independent
        # confirmation that the IO module actually applied the command.
        txt = str(msg.data)
        for tok in txt.split():
            if tok.startswith('vision='):
                state = tok.split('=', 1)[1]
                if getattr(ledall_cb, '_last', None) != state:
                    ledall_cb._last = state
                    tl.mark('led_state_all', f"hardware vision={state}")

    def state_cb(msg):
        tl.mark('camera/state', str(msg.data))

    subs = [
        rospy.Subscriber('/crevis/led/vision', Bool, led_cb, queue_size=20),
        rospy.Subscriber('/crevis/led_state_all', String, ledall_cb, queue_size=20),
        rospy.Subscriber('/camera/state', String, state_cb, queue_size=20),
    ]
    active_pub = rospy.Publisher('/camera/set_active', Bool, queue_size=1)
    time.sleep(1.0)          # let latched state arrive before t=0
    tl.reset()

    # --- replicate the production order: preopen, capture, release ---
    tl.mark('test', 'preopen -> /camera/set_active true')
    active_pub.publish(Bool(True))
    time.sleep(args.preopen_wait)

    try:
        proxy = rospy.ServiceProxy('/camera/capture', CaptureImages)
        tl.mark('test', f"calling /camera/capture (n={args.samples})")
        t_call = time.time()
        resp = proxy(num_samples=args.samples,
                     delay_between_s=-1.0,
                     use_vision_led=True)
        dt_call = time.time() - t_call
        tl.mark('test', f"capture returned in {dt_call*1000:.0f} ms")
    except Exception as e:
        for s in subs:
            s.unregister()
        rep.add('lamp', 'capture call', 'FAIL', f"{type(e).__name__}: {e}")
        return None

    time.sleep(0.5)          # catch the trailing lamp-off + state events
    tl.mark('test', 'release -> /camera/set_active false')
    active_pub.publish(Bool(False))
    time.sleep(0.5)
    for s in subs:
        s.unregister()

    tl.dump('capture timeline')

    # --- verdicts ---
    if not resp.success:
        rep.add('lamp', 'capture', 'FAIL', f"success=false: {resp.message}")
        return None
    if not resp.images:
        rep.add('lamp', 'capture', 'FAIL', 'success but zero frames returned')
        return None

    img = resp.images[0]
    rep.add('lamp', 'capture', 'PASS',
            f"{len(resp.images)} frame(s) {img.width}x{img.height} {img.encoding}")

    on = tl.first('led/vision', lambda t: 'ON' in t)
    off = tl.last('led/vision', lambda t: 'OFF' in t)
    t_capture = tl.first('test', lambda t: 'calling' in t)
    t_returned = tl.first('test', lambda t: 'returned' in t)

    if on is None:
        rep.add('lamp', 'VISION on', 'FAIL',
                'lamp never commanded ON during capture')
    elif t_capture is not None and on < t_capture:
        rep.add('lamp', 'VISION on', 'WARN',
                f"lamp went ON at t+{on:.3f}s, BEFORE the capture call "
                f"(t+{t_capture:.3f}s) — something else is driving it")
    else:
        rep.add('lamp', 'VISION on', 'PASS', f"t+{on:.3f}s, inside the capture call")

    if off is None:
        rep.add('lamp', 'VISION off', 'FAIL',
                'lamp never commanded OFF — it is still lit')
    elif on is not None and off <= on:
        rep.add('lamp', 'VISION off', 'FAIL',
                f"OFF at t+{off:.3f}s precedes ON at t+{on:.3f}s")
    elif t_returned is not None and off > t_returned + 0.5:
        rep.add('lamp', 'VISION off', 'WARN',
                f"lamp stayed lit {off - t_returned:.3f}s after capture returned")
    else:
        rep.add('lamp', 'VISION off', 'PASS',
                f"t+{off:.3f}s, lit for {off - on:.3f}s")

    # Hardware echo — proves the IO module applied it, not just that we asked.
    hw_on = tl.first('led_state_all', lambda t: 'vision=1' in t)
    if hw_on is not None:
        rep.add('lamp', 'hardware echo', 'PASS',
                f"crevis reported vision=1 at t+{hw_on:.3f}s")
    else:
        # /crevis/led_state_all runs at ~10 Hz; a short burst can fall between
        # two samples, so a miss is inconclusive rather than a failure.
        rep.add('lamp', 'hardware echo', 'WARN',
                'no vision=1 seen in led_state_all (10 Hz may have missed a '
                'short burst — not conclusive)')

    opened = tl.first('camera/state', lambda t: 'open' in t)
    capturing = tl.first('camera/state', lambda t: 'capturing' in t)
    if capturing is not None:
        rep.add('lamp', 'device state', 'PASS',
                f"open t+{opened:.3f}s -> capturing t+{capturing:.3f}s"
                if opened is not None else f"capturing at t+{capturing:.3f}s")
    else:
        rep.add('lamp', 'device state', 'WARN',
                'no capturing state seen on /camera/state')

    return resp.images[0]


# ============================================================
# STEP — INFERENCE
# ============================================================
def check_inference(rep, img_msg):
    header("4. Inference — does the captured frame produce an Ra value?")

    if img_msg is None:
        rep.add('infer', 'Ra inference', 'SKIP', 'no frame from the capture step')
        return

    try:
        from cv_bridge import CvBridge
        from apriltag_nav.inference_interface import InferenceInterface
        from apriltag_nav.paths import MODEL_PATH
    except Exception as e:
        rep.add('infer', 'Ra inference', 'SKIP', f"import failed: {e}")
        return

    if not os.path.exists(MODEL_PATH):
        rep.add('infer', 'Ra inference', 'SKIP', f"model not found: {MODEL_PATH}")
        return

    try:
        frame = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
    except Exception as e:
        rep.add('infer', 'frame decode', 'FAIL', f"{type(e).__name__}: {e}")
        return
    rep.add('infer', 'frame decode', 'PASS', f"{frame.shape[1]}x{frame.shape[0]}")

    inf = InferenceInterface()
    t0 = time.time()
    if not inf.load_model(MODEL_PATH):
        rep.add('infer', 'model load', 'FAIL', MODEL_PATH)
        return
    rep.add('infer', 'model load', 'PASS', f"{(time.time()-t0)*1000:.0f} ms")

    t0 = time.time()
    try:
        ra = inf.infer(frame)
    except Exception as e:
        rep.add('infer', 'Ra inference', 'FAIL', f"{type(e).__name__}: {e}")
        return
    dt = (time.time() - t0) * 1000
    if ra is None:
        rep.add('infer', 'Ra inference', 'FAIL', 'returned None')
    else:
        rep.add('infer', 'Ra inference', 'PASS', f"Ra={ra:.4f}  ({dt:.0f} ms)")


# ============================================================
# MODE — ONE REAL SCAN POINT (ARM MOVES)
# ============================================================
def _read_current_joints():
    """Current joint angles in degrees, straight from the SDK."""
    from apriltag_nav.paths import add_fairino_sdk_to_path
    if not add_fairino_sdk_to_path():
        return None
    try:
        from fairino import Robot
        robot = Robot.RPC('192.168.58.2')
        time.sleep(0.4)
        ret, joints = robot.GetActualJointPosRadian()
        if ret != 0:
            return None
        return [j * 57.29578 for j in joints]
    except Exception:
        return None


def confirm_motion(args, joints_deg):
    """Spell out every motion before anything moves. Returns True to proceed."""
    from apriltag_nav.paths import load_yaml_block

    home = load_yaml_block('arm_home').get('joints_rad',
                                           [-1.5708, -1.5708, 1.5708,
                                            -1.5708, -1.5708, 0.0])
    home_deg = [h * 57.29578 for h in home]
    current = _read_current_joints()

    print("\n" + "!" * 68)
    print("  THE ARM WILL MOVE")
    print("!" * 68)
    # Read the live tuning rather than hardcoding it — these values have moved
    # (max_step_mm 5.0 -> 1.0) and a stale prompt understates what will happen.
    kx, _ = _effective_keyence_params()
    print("\n  This sends one scan point through /arm/scan_command, which runs")
    print("  the production path in ArmController.execute_scan_points():\n")
    print("    1. MoveJ to the scan point")
    print(f"    2. Keyence closed loop — up to {int(kx['max_steps'])} MoveL steps "
          f"along tool Z, each clamped")
    print(f"       to {kx['max_step_mm']:.1f} mm, skipped entirely if the "
          f"perpendicular error is >= {kx['activate_threshold']:.1f} mm")
    print("    3. capture + inference (no motion)")
    print("    4. move_to_home() — ALWAYS runs after the last point\n")

    def fmt(v):
        return '[' + ', '.join(f"{x:7.2f}" for x in v) + ']'

    if current is not None:
        print(f"  current joints (deg): {fmt(current)}")
    print(f"  scan point    (deg): {fmt(joints_deg)}")
    print(f"  home          (deg): {fmt(home_deg)}   <- step 4 ends here")

    if current is not None:
        d_point = max(abs(a - b) for a, b in zip(current, joints_deg))
        d_home = max(abs(a - b) for a, b in zip(current, home_deg))
        print(f"\n  largest joint delta to the scan point: {d_point:.2f} deg")
        print(f"  largest joint delta to home:           {d_home:.2f} deg")
        if d_home > 5.0:
            print("\n  *** Step 4 is a REAL move, not a no-op. Make sure the arm")
            print("      has a clear path to home before continuing. ***")

    print("\n  Check: workspace clear, e-stop within reach, nobody near the arm.")

    if args.yes:
        print("\n  --yes given, proceeding.")
        return True
    try:
        ans = input("\n  Type 'yes' to proceed: ").strip().lower()
    except EOFError:
        print("  no tty and --yes not given — aborting.")
        return False
    return ans == 'yes'


def run_scan_point(rep, args):
    import rospy
    from std_msgs.msg import Bool, String, Float32

    header("5. One real scan point through /arm/scan_command")

    if args.joints:
        try:
            joints_deg = [float(x) for x in args.joints.split(',')]
        except ValueError:
            rep.add('point', 'joints arg', 'FAIL', f"unparsable: {args.joints}")
            return
        if len(joints_deg) != 6:
            rep.add('point', 'joints arg', 'FAIL',
                    f"need 6 values, got {len(joints_deg)}")
            return
    else:
        joints_deg = _read_current_joints()
        if joints_deg is None:
            rep.add('point', 'current joints', 'FAIL',
                    'could not read from the SDK — pass --joints explicitly')
            return
        _info("no --joints given; using the CURRENT pose, so step 1 is a no-op "
              "move and only step 4 (home) actually travels")

    if not confirm_motion(args, joints_deg):
        rep.add('point', 'scan point', 'SKIP', 'not confirmed by the operator')
        return

    tl = Timeline()
    kw = KeyenceWatch(timeline=tl)
    done = {'flag': False}

    def led_cb(m):    tl.mark('led/vision', 'ON' if m.data else 'OFF')
    def state_cb(m):  tl.mark('camera/state', str(m.data))
    def status_cb(m): tl.mark('arm/status', str(m.data))
    def ra_cb(m):     tl.mark('scan/ra_value', f"Ra = {m.data:.4f}")

    def result_cb(m):
        try:
            d = json.loads(m.data)
            tl.mark('scan/point_result',
                    f"mean={d.get('ra_mean'):.4f} n={d.get('num_samples')}")
        except Exception:
            tl.mark('scan/point_result', str(m.data)[:60])

    def finished_cb(m):
        tl.mark('scan_finished', f"data={m.data}")
        done['flag'] = True

    subs = [
        rospy.Subscriber('/crevis/led/vision', Bool, led_cb, queue_size=20),
        rospy.Subscriber('/camera/state', String, state_cb, queue_size=20),
        rospy.Subscriber('/arm/status', String, status_cb, queue_size=20),
        rospy.Subscriber('/keyence/value', Float32, kw.cb, queue_size=20),
        rospy.Subscriber('/scan/ra_value', Float32, ra_cb, queue_size=20),
        rospy.Subscriber('/scan/point_result', String, result_cb, queue_size=20),
        rospy.Subscriber('/scan_finished', Bool, finished_cb, queue_size=5),
    ]
    cmd_pub = rospy.Publisher('/arm/scan_command', String, queue_size=1)
    time.sleep(1.0)
    tl.reset()

    # csv_path is deliberately empty: ScanResultsWriter.save() returns early on
    # a falsy path, so a diagnostic run cannot corrupt a real scan CSV.
    point = [{
        'point_id':  9999,
        'group_id':  -1,
        'mode':      'joint',
        'joints':    [d / 57.29578 for d in joints_deg],   # controller wants rad
        'speed':     int(args.speed),
        'csv_path':  '',
    }]

    tl.mark('test', f"publishing scan point (speed={args.speed})")
    cmd_pub.publish(String(json.dumps(point)))

    print(f"\n  waiting up to {args.timeout:.0f}s for /scan_finished ...")
    t_end = time.time() + args.timeout
    while time.time() < t_end and not done['flag'] and not rospy.is_shutdown():
        time.sleep(0.05)
    for s in subs:
        s.unregister()

    tl.dump('scan point timeline')

    if not done['flag']:
        rep.add('point', '/scan_finished', 'FAIL',
                f"not received within {args.timeout:.0f}s — scan may still be running")
        return
    rep.add('point', '/scan_finished', 'PASS',
            f"t+{tl.first('scan_finished'):.1f}s")

    # --- per-link verdicts, in chain order ---
    if tl.first('camera/state', lambda t: 'open' in t) is not None:
        rep.add('point', 'camera preopen', 'PASS', 'device opened before capture')
    else:
        rep.add('point', 'camera preopen', 'WARN', 'no open state seen')

    if kw.count:
        rep.add('point', 'keyence during scan', 'PASS',
                f"{kw.count} msg, {kw.vmin:.3f}..{kw.vmax:.3f} mm")
    else:
        rep.add('point', 'keyence during scan', 'FAIL',
                'no data — the distance adjustment was skipped')

    on = tl.first('led/vision', lambda t: t == 'ON')
    off = tl.last('led/vision', lambda t: t == 'OFF')
    if on is not None and off is not None and off > on:
        rep.add('point', 'VISION bracketed shutter', 'PASS',
                f"ON t+{on:.1f}s -> OFF t+{off:.1f}s ({off-on:.2f}s lit)")
    else:
        rep.add('point', 'VISION bracketed shutter', 'FAIL',
                f"ON={on} OFF={off}")

    ra = tl.first('scan/ra_value')
    if ra is not None:
        rep.add('point', 'Ra published', 'PASS', f"t+{ra:.1f}s")
    else:
        rep.add('point', 'Ra published', 'FAIL', 'no /scan/ra_value')

    if tl.first('scan/point_result') is not None:
        rep.add('point', 'point result', 'PASS', 'published')
    else:
        rep.add('point', 'point result', 'FAIL', 'no /scan/point_result')


# ============================================================
# MAIN
# ============================================================
def main():
    ap = argparse.ArgumentParser(
        description='Scan-point chain test: Basler + VISION lamp + Keyence + arm.',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--move', action='store_true',
                    help='run one REAL scan point (THE ARM MOVES)')
    ap.add_argument('--yes', action='store_true',
                    help='skip the interactive motion confirmation')
    ap.add_argument('--joints',
                    help='scan point joints in deg, comma separated '
                         '(default: current pose)')
    ap.add_argument('--speed', type=int, default=20,
                    help='arm speed percent for the point (default 20)')
    ap.add_argument('--samples', type=int, default=1,
                    help='frames per capture (default 1)')
    ap.add_argument('--keyence-window', type=float, default=3.0,
                    help='seconds to sample keyence/value (default 3.0)')
    ap.add_argument('--preopen-wait', type=float, default=1.0,
                    help='seconds between preopen and capture (default 1.0)')
    ap.add_argument('--timeout', type=float, default=180.0,
                    help='seconds to wait for /scan_finished (default 180)')
    ap.add_argument('--no-infer', action='store_true',
                    help='skip the ONNX inference step')
    args = ap.parse_args()

    try:
        import rospy
        import rosgraph
    except ImportError as e:
        print(f"[FAIL] {e} — source devel/setup.bash first")
        sys.exit(1)

    if not rosgraph.is_master_online():
        print(f"[FAIL] ROS master offline at "
              f"{os.environ.get('ROS_MASTER_URI', '?')}")
        sys.exit(1)

    rospy.init_node('test_scan_chain', anonymous=True, disable_signals=True)

    rep = Report()
    mode = 'point (ARM MOVES)' if args.move else 'probe (no motion)'
    print(f"Scan chain test — mode: {mode}")

    # In probe mode a missing Keyence node is reported but not fatal: the lamp
    # and camera links are still worth checking on their own.
    if not check_preconditions(rep, need_keyence=args.move):
        print("\n  preconditions failed — stopping before touching hardware")
        rep.summary()
        sys.exit(1)

    check_keyence(rep, args.keyence_window)
    img = check_lamp_shutter(rep, args)
    if not args.no_infer:
        check_inference(rep, img)

    if args.move:
        run_scan_point(rep, args)

    rep.summary()
    sys.exit(0 if rep.ok() else 1)


if __name__ == '__main__':
    main()
