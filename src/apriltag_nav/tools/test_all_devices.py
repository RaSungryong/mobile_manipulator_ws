#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Full-system device connection + data reception test.

Complements tools/test_hardware.py, which only opens the three directly-attached
devices (arm / Keyence / Basler). This script also covers the network map, the
USB cameras, and — the part that actually matters once the stack is running —
whether every topic is *delivering data*, not merely advertised.

Read-only by design. It never publishes /cmd_vel, never homes or moves the arm,
never drives an LED, and never calls /camera/capture unless --capture is given
(that service lights the VISION lamp). Safe to run while the stack is up.

Sections:
    net       TCP/ICMP reachability for every device in the network map
    arm       Fairino FR10v6 via SDK — joints, TCP pose, mode
    keyence   Keyence DL-EN1 — one M0 measurement over TCP
    basler    Wrist Basler via PyPylon — enumerate + grab one frame
    usb       Orbbec / RealSense USB presence + serial match vs the launch file
    ros       ROS master, expected nodes, services, and per-topic receive rate

Usage:
    python3 test_all_devices.py                 # everything
    python3 test_all_devices.py ros             # one section
    python3 test_all_devices.py net arm keyence # several
    python3 test_all_devices.py ros -d 5.0      # longer topic sampling window
    python3 test_all_devices.py --capture       # also exercise /camera/capture

Exit code 0 only when every required check passed. Optional checks (cameras
whose driver is intentionally off, an unbuilt robot_msgs, ...) report SKIP and
do not fail the run.
"""

import argparse
import os
import socket
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

try:
    from apriltag_nav.paths import (load_config, add_fairino_sdk_to_path,
                                    FAIRINO_SDK_PATH)
except Exception:  # running without the package on sys.path
    load_config = None
    FAIRINO_SDK_PATH = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', 'fairino_sdk', 'fairino-python-sdk', 'Linux')

    def add_fairino_sdk_to_path():
        if not os.path.isdir(FAIRINO_SDK_PATH):
            return False
        if FAIRINO_SDK_PATH not in sys.path:
            sys.path.append(FAIRINO_SDK_PATH)
        return True


# ============================================================
# CONFIG
# ============================================================
ARM_IP = '192.168.58.2'
ARM_PORT = 8080

# Network map per the Navifra driver guide §1 (see CLAUDE.md). Every LAN-hub
# device lives on 192.168.100.x; 192.168.1.x is reserved for the wireless AP.
# port None = ICMP ping only (device answers no known TCP port).
NETWORK_MAP = [
    # (label,               host,              port,  required)
    ('Front LiDAR',         '192.168.100.101', None,  False),
    ('Rear LiDAR',          '192.168.100.102', None,  False),
    ('Safety PLC (PNOZ)',   '192.168.100.103', None,  False),
    ('Crevis GN-9289 IO',   '192.168.100.104', None,  False),
    ('Keyence DL-EN1',      '192.168.100.105', 64000, True),
    ('Manipulator camera',  '192.168.200.106', None,  False),
    ('Fairino FR10v6 arm',  ARM_IP,            8080,  True),
    ('Wireless AP',         '192.168.1.10',    None,  False),
]

# Serials pinned in mobile_manipulator.launch. Three RealSense-class devices
# share this PC's USB bus, so a wrapper without a serial opens whichever it
# enumerates first — side_cam and hand_cam would silently swap intrinsics.
RS_SERIALS = {
    'side_cam': '409122271601',   # D405
    'hand_cam': '935322071325',   # D435
}
USB_VENDORS = {
    'Orbbec (front_cam Femto Bolt)': '2bc5',
    'Intel RealSense (side/hand_cam)': '8086',
}

# All nine are started by mobile_manipulator.launch. robot_camera_node is
# required because navigation consumes its detections — without it /robot_pose
# never publishes and GOTO cannot work, even though vision-stop stays inert
# while stop_tag_ids is the empty placeholder.
EXPECTED_NODES = [
    ('/mobile_manipulator_system', True),
    ('/mobile_node',               True),
    ('/arm_node',                  True),
    ('/basler_camera_node',        True),
    ('/keyence_dlen1_node',        True),
    ('/robot_camera_node',         True),
    ('/lifter_node',               True),
    ('/camera_viewer_node',        False),   # debug aid; absence is not a fault
    ('/inference_node',            False),   # on-demand Ra for robot_ui; TASK scans score in-process
]

EXPECTED_SERVICES = [
    ('/camera/capture',                     True),
    ('/arm/move_home',                      True),
    ('/mobile/stop',                        True),
    ('/mobile/cancel',                      True),
    ('/mobile/clear_stop',                  True),
    ('/lifter/home',                        True),
    ('/lifter/stop',                        True),
    ('/lifter/reset',                       False),
    ('/lifter/goto_scan_height',            False),
    ('/robot_camera/front_cam/set_enabled', False),
    ('/robot_camera/side_cam/set_enabled',  False),
    ('/robot_camera/hand_cam/set_enabled',  False),
    ('/camera_viewer/set_enabled',          False),
    ('/inference/predict',                  False),
]


# ============================================================
# REPORTING
# ============================================================
class Report:
    """Collects per-check results so the summary can be printed once at the end."""

    def __init__(self):
        self.rows = []      # (section, name, status, detail)

    def add(self, section, name, status, detail=''):
        self.rows.append((section, name, status, detail))
        tag = {'PASS': '[PASS]', 'FAIL': '[FAIL]',
               'SKIP': '[SKIP]', 'WARN': '[WARN]'}[status]
        line = f"  {tag} {name}"
        if detail:
            line += f" — {detail}"
        print(line)

    def ok(self):
        return not any(s == 'FAIL' for _, _, s, _ in self.rows)

    def summary(self):
        print("\n" + "=" * 68)
        print("Summary")
        print("=" * 68)
        counts = {'PASS': 0, 'FAIL': 0, 'SKIP': 0, 'WARN': 0}
        section = None
        for sec, name, status, detail in self.rows:
            if sec != section:
                section = sec
                print(f"\n  {sec}")
            counts[status] += 1
            print(f"    {status:<5} {name:<34} {detail}")
        print("\n" + "-" * 68)
        print(f"  PASS {counts['PASS']}   FAIL {counts['FAIL']}   "
              f"WARN {counts['WARN']}   SKIP {counts['SKIP']}")
        print("=" * 68)


def header(title):
    print(f"\n{'=' * 68}\n{title}\n{'=' * 68}")


def _info(msg):
    print(f"  [INFO] {msg}")


# ============================================================
# SECTION 1 — NETWORK
# ============================================================
def _ping(host, timeout_s=1.0):
    try:
        r = subprocess.run(['ping', '-c', '1', '-W', str(int(max(1, timeout_s))), host],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=timeout_s + 2.0)
        return r.returncode == 0
    except Exception:
        return False


def _tcp(host, port, timeout_s=2.0):
    try:
        s = socket.create_connection((host, port), timeout=timeout_s)
        s.close()
        return True, ''
    except Exception as e:
        return False, str(e)


def test_net(rep, args):
    header("1. Network — device reachability")
    for label, host, port, required in NETWORK_MAP:
        name = f"{label} ({host})"
        if port is not None:
            up, err = _tcp(host, port, args.net_timeout)
            if up:
                rep.add('net', name, 'PASS', f"TCP {port} open")
                continue
            # A device can be alive but with the port closed — distinguish the
            # two, because "wrong port" and "unplugged" need different fixes.
            if _ping(host, args.net_timeout):
                rep.add('net', name, 'FAIL' if required else 'WARN',
                        f"ping OK but TCP {port} refused: {err}")
            else:
                rep.add('net', name, 'FAIL' if required else 'WARN',
                        'unreachable (no ping, no TCP)')
        else:
            if _ping(host, args.net_timeout):
                rep.add('net', name, 'PASS', 'ping OK')
            else:
                rep.add('net', name, 'FAIL' if required else 'WARN',
                        'no ping response')


# ============================================================
# SECTION 2 — FAIRINO ARM
# ============================================================
def test_arm(rep, args):
    header("2. Fairino FR10v6 arm — SDK connection + data")
    _info(f"Target: {ARM_IP}:{ARM_PORT}")

    up, err = _tcp(ARM_IP, ARM_PORT, 3.0)
    if not up:
        rep.add('arm', 'TCP 8080', 'FAIL', err)
        return
    rep.add('arm', 'TCP 8080', 'PASS', 'reachable')

    if not add_fairino_sdk_to_path():
        rep.add('arm', 'SDK import', 'FAIL',
                f"SDK not found at {FAIRINO_SDK_PATH}")
        return

    try:
        from fairino import Robot
    except Exception as e:
        rep.add('arm', 'SDK import', 'FAIL', f"{type(e).__name__}: {e}")
        return

    try:
        robot = Robot.RPC(ARM_IP)
        time.sleep(0.5)
    except Exception as e:
        rep.add('arm', 'SDK connect', 'FAIL', f"{type(e).__name__}: {e}")
        return
    rep.add('arm', 'SDK connect', 'PASS', 'RPC established')

    # Joint feedback is the real "is data flowing" check — the RPC handshake
    # succeeds even when the controller is faulted.
    try:
        ret, joints = robot.GetActualJointPosRadian()
        if ret == 0:
            deg = [round(j * 57.29578, 2) for j in joints]
            rep.add('arm', 'joint feedback', 'PASS', f"deg={deg}")
        else:
            rep.add('arm', 'joint feedback', 'FAIL', f"ret={ret}")
    except Exception as e:
        rep.add('arm', 'joint feedback', 'FAIL', f"{type(e).__name__}: {e}")

    try:
        ret, tcp = robot.GetActualTCPPose()
        if ret == 0:
            rep.add('arm', 'TCP pose', 'PASS',
                    str([round(v, 2) for v in tcp]))
        else:
            rep.add('arm', 'TCP pose', 'WARN', f"ret={ret}")
    except Exception as e:
        rep.add('arm', 'TCP pose', 'WARN', f"{type(e).__name__}: {e}")

    for label, call in (('robot mode', 'GetRobotMotionDone'),
                        ('error code', 'GetRobotErrorCode')):
        fn = getattr(robot, call, None)
        if fn is None:
            rep.add('arm', label, 'SKIP', f"{call} not in this SDK build")
            continue
        try:
            res = fn()
            rep.add('arm', label, 'PASS', f"{call} -> {res}")
        except Exception as e:
            rep.add('arm', label, 'WARN', f"{type(e).__name__}: {e}")


# ============================================================
# SECTION 3 — KEYENCE DL-EN1
# ============================================================
def test_keyence(rep, args):
    header("3. Keyence DL-EN1 — TCP measurement")

    cfg = (load_config() or {}).get('keyence', {}) if load_config else {}
    host = cfg.get('ip', '192.168.100.105')
    port = int(cfg.get('port', 64000))
    _info(f"Target: {host}:{port}")

    try:
        s = socket.create_connection((host, port), timeout=3.0)
    except Exception as e:
        rep.add('keyence', 'TCP connect', 'FAIL', str(e))
        return
    rep.add('keyence', 'TCP connect', 'PASS', f"port {port} open")

    try:
        s.settimeout(1.0)
        s.sendall(b"M0\r\n")
        try:
            s.shutdown(socket.SHUT_WR)
        except OSError:
            pass

        chunks = []
        t0 = time.time()
        while time.time() - t0 < 1.0:
            try:
                b = s.recv(256)
            except socket.timeout:
                break
            if not b:
                break
            chunks.append(b)
            if b"\r" in b or b"\n" in b:
                break
        s.close()

        text = b"".join(chunks).decode(errors='ignore').strip()
        if not text:
            rep.add('keyence', 'M0 measurement', 'FAIL', 'no response')
            return
        if text.startswith('ER,'):
            rep.add('keyence', 'M0 measurement', 'FAIL', f"sensor error: {text}")
            return

        parts = text.split(',')
        if len(parts) >= 2:
            try:
                raw = int(parts[1].strip())
                # The DL-EN1 answers with a +/-99999xxx sentinel when the target
                # is out of range. The link is fine — the reading is not usable,
                # and a closed-loop scan would chase a garbage setpoint.
                if abs(raw) >= 99999000:
                    rep.add('keyence', 'M0 measurement', 'WARN',
                            f"raw={raw} — out-of-range sentinel, no target in view")
                else:
                    rep.add('keyence', 'M0 measurement', 'PASS',
                            f"raw={raw}  {raw * 0.001:.3f} mm")
                return
            except ValueError:
                pass
        rep.add('keyence', 'M0 measurement', 'PASS', f"response '{text}'")
    except Exception as e:
        rep.add('keyence', 'M0 measurement', 'FAIL', f"{type(e).__name__}: {e}")


# ============================================================
# SECTION 4 — BASLER
# ============================================================
def test_basler(rep, args):
    header("4. Wrist Basler camera — PyPylon enumerate + grab")

    try:
        from pypylon import pylon
    except ImportError:
        rep.add('basler', 'pypylon import', 'FAIL',
                'not installed — pip install pypylon')
        return
    rep.add('basler', 'pypylon import', 'PASS')

    try:
        factory = pylon.TlFactory.GetInstance()
        devices = factory.EnumerateDevices()
    except Exception as e:
        rep.add('basler', 'enumerate', 'FAIL', f"{type(e).__name__}: {e}")
        return

    if not devices:
        rep.add('basler', 'enumerate', 'FAIL', 'no camera found')
        return
    for i, d in enumerate(devices):
        _info(f"[{i}] {d.GetModelName()} S/N {d.GetSerialNumber()} "
              f"({d.GetDeviceClass()})")
    rep.add('basler', 'enumerate', 'PASS', f"{len(devices)} device(s)")

    # basler_camera_node keeps the device Close()d between captures and owns it
    # exclusively, so opening it here fails while the stack is running. That is
    # the correct answer, not a defect — report it as such.
    camera = None
    try:
        camera = pylon.InstantCamera(factory.CreateDevice(devices[0]))
        camera.Open()
        rep.add('basler', 'open', 'PASS',
                camera.GetDeviceInfo().GetModelName())

        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        result = camera.RetrieveResult(3000, pylon.TimeoutHandling_ThrowException)
        if result.GrabSucceeded():
            arr = converter.Convert(result).GetArray()
            rep.add('basler', 'grab frame', 'PASS',
                    f"{arr.shape[1]}x{arr.shape[0]} {arr.dtype}")
        else:
            rep.add('basler', 'grab frame', 'FAIL', result.ErrorDescription)
        result.Release()
        camera.StopGrabbing()
    except Exception as e:
        msg = f"{type(e).__name__}: {e}"
        if 'exclusive' in msg.lower() or 'access' in msg.lower():
            rep.add('basler', 'open', 'SKIP',
                    'device held by basler_camera_node (expected while running)')
        else:
            rep.add('basler', 'open', 'FAIL', msg)
    finally:
        try:
            if camera is not None and camera.IsOpen():
                camera.Close()
        except Exception:
            pass


# ============================================================
# SECTION 5 — USB CAMERAS (Orbbec / RealSense)
# ============================================================
def _realsense_serials():
    """{serial: model} for every connected RealSense, plus how it was obtained.

    pyrealsense2 is not installed on the robot PC (the driver is the C++
    realsense2_camera built from source), so the librealsense CLI is the
    practical path. Returns (None, reason) when neither is available.
    """
    try:
        import pyrealsense2 as rs
        found = {}
        for dev in rs.context().query_devices():
            found[dev.get_info(rs.camera_info.serial_number)] = \
                dev.get_info(rs.camera_info.name)
        return found, 'pyrealsense2'
    except ImportError:
        pass
    except Exception as e:
        return None, f"pyrealsense2 error: {type(e).__name__}: {e}"

    try:
        out = subprocess.run(['rs-enumerate-devices', '-s'],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             timeout=20.0)
    except FileNotFoundError:
        return None, ('neither pyrealsense2 nor rs-enumerate-devices available '
                      '— serials unverified')
    except Exception as e:
        return None, f"rs-enumerate-devices failed: {type(e).__name__}: {e}"

    # Columns are "Device Name | Serial Number | Firmware Version", space
    # padded; the model name itself contains spaces, so split from the right.
    found = {}
    for line in out.stdout.decode(errors='ignore').splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 3 and parts[-2].isdigit():
            found[parts[-2]] = ' '.join(parts[:-2])
    return found, 'rs-enumerate-devices'


def test_usb(rep, args):
    header("5. USB cameras — Orbbec front_cam / RealSense side+hand_cam")

    try:
        out = subprocess.run(['lsusb'], stdout=subprocess.PIPE,
                             stderr=subprocess.DEVNULL, timeout=10.0)
        lsusb = out.stdout.decode(errors='ignore')
    except Exception as e:
        lsusb = ''
        rep.add('usb', 'lsusb', 'SKIP', f"{type(e).__name__}: {e}")

    if lsusb:
        for label, vid in USB_VENDORS.items():
            hits = [ln for ln in lsusb.splitlines()
                    if f"ID {vid}:" in ln.lower() or f":{vid}:" in ln.lower()
                    or f" {vid}:" in ln.lower()]
            if hits:
                rep.add('usb', label, 'PASS', f"{len(hits)} device(s) on bus")
                for h in hits:
                    _info(h.strip())
            else:
                rep.add('usb', label, 'WARN', f"no VID {vid} on the USB bus")

    # Serial check is the one that matters: without it side_cam and hand_cam
    # can swap and each gets the other's intrinsics.
    found, how = _realsense_serials()
    if found is None:
        rep.add('usb', 'RealSense serials', 'SKIP', how)
        return
    _info(f"serials via {how}")

    if not found:
        rep.add('usb', 'RealSense serials', 'FAIL', 'no RealSense device found')
        return
    for name, serial in RS_SERIALS.items():
        if serial in found:
            rep.add('usb', f"{name} serial {serial}", 'PASS', found[serial])
        else:
            rep.add('usb', f"{name} serial {serial}", 'FAIL',
                    f"not present; connected: {sorted(found)}")


# ============================================================
# SECTION 6 — ROS TOPICS / NODES / SERVICES
# ============================================================
def _fmt_bool(m):        return f"data={m.data}"
def _fmt_scalar(m):      return f"data={m.data}"
def _fmt_string(m):      return f"'{str(m.data)[:48]}'"


def _fmt_odom(m):
    p = m.pose.pose.position
    t = m.twist.twist
    return (f"xy=({p.x:.3f}, {p.y:.3f}) "
            f"v=({t.linear.x:.3f} m/s, {t.angular.z:.3f} rad/s)")


def _fmt_battery(m):
    pct = m.percentage * 100.0 if m.percentage <= 1.0 else m.percentage
    return f"{pct:.1f}%  {m.voltage:.2f} V  {m.current:.2f} A"


def _fmt_image(m):
    return f"{m.width}x{m.height} {m.encoding}"


def _fmt_caminfo(m):
    return f"{m.width}x{m.height} fx={m.K[0]:.1f} fy={m.K[4]:.1f}"


def _fmt_tags(m):
    ids = [d.id for d in m.detections]
    return f"{m.camera_name} {m.image_width}x{m.image_height} tags={ids}"


def _fmt_twist(m):
    return f"lin.x={m.linear.x:.3f} ang.z={m.angular.z:.3f}"


def _fmt_pose2d(m):
    return f"x={m.x:.3f} y={m.y:.3f} theta={m.theta:.1f}deg flag={m.flag}"


def _topic_table():
    """(group, topic, msg_type, required, formatter, min_hz) — msg types resolved
    lazily so a missing robot_msgs build degrades to SKIP instead of ImportError."""
    from std_msgs.msg import Bool, String, Int32, Float32
    from sensor_msgs.msg import BatteryState, Image, CameraInfo
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    table = [
        # --- Navifra base driver -----------------------------------------
        ('base',   '/odom',                    Odometry,     True,  _fmt_odom,   5.0),
        ('base',   '/cmd_vel',                 Twist,        False, _fmt_twist,  0.0),
        # --- Safety (fail-safe: true at startup and on PLC comms loss) ----
        ('safety', '/safety/estop',            Bool,         True,  _fmt_bool,   0.0),
        ('safety', '/safety/connected',        Bool,         False, _fmt_bool,   0.0),
        # --- Battery ------------------------------------------------------
        ('power',  '/bms/state',               BatteryState, True,  _fmt_battery, 0.0),
        ('power',  '/bms/soc',                 Float32,      False, _fmt_scalar, 0.0),
        ('power',  '/crevis/charging',         Bool,         False, _fmt_bool,   0.0),
        ('power',  '/crevis/charge_port_on',   Bool,         False, _fmt_bool,   0.0),
        # --- Crevis IO / lighting ----------------------------------------
        ('io',     '/crevis/connected',        Bool,         False, _fmt_bool,   0.0),
        ('io',     '/crevis/led_state_all',    String,       False, _fmt_string, 0.0),
        # --- Lift (incremental encoder; homing needed once per power cycle)
        ('lift',   '/lift/position',           Int32,        False, _fmt_scalar, 0.0),
        ('lift',   '/lift/status',             String,       False, _fmt_string, 0.0),
        ('lift',   '/lift/homed',              Bool,         False, _fmt_bool,   0.0),
        ('lift',   '/lift/error',              Bool,         False, _fmt_bool,   0.0),
        ('lift',   '/lift/alarm',              String,       False, _fmt_string, 0.0),
        # lifter_node's own view of the same axis. Required (latched), and
        # it is the topic to look at first: it carries homed / soft limits /
        # busy together, which the raw /lift/* topics do not.
        ('lift',   '/lifter/state',            String,       True,  _fmt_string, 0.0),
        ('lift',   '/lifter/ready',            Bool,         False, _fmt_bool,   0.0),
        # --- Drive owner --------------------------------------------------
        # mobile_node's state is what MobileClient polls to turn a fire-and-
        # forget /mobile/goto_tag into a blocking call, so a silent one here
        # means every GOTO would time out rather than fail visibly.
        ('drive',  '/mobile/state',            String,       True,  _fmt_string, 0.0),
        ('drive',  '/mobile/busy',             Bool,         False, _fmt_bool,   0.0),
        # --- Cameras (images) --------------------------------------------
        ('camera', '/front_cam/color/image_raw',  Image,      False, _fmt_image,   1.0),
        ('camera', '/front_cam/color/camera_info', CameraInfo, False, _fmt_caminfo, 1.0),
        ('camera', '/side_cam/color/image_raw',   Image,      False, _fmt_image,   1.0),
        ('camera', '/side_cam/color/camera_info',  CameraInfo, False, _fmt_caminfo, 1.0),
        ('camera', '/hand_cam/color/image_raw',   Image,      False, _fmt_image,   1.0),
        ('camera', '/hand_cam/color/camera_info',  CameraInfo, False, _fmt_caminfo, 1.0),
        # --- Sensors / node output ---------------------------------------
        ('sensor', '/keyence/value',           Float32,      False, _fmt_scalar, 0.0),
        ('sensor', '/keyence/raw',             Int32,        False, _fmt_scalar, 0.0),
        ('node',   '/arm/status',              String,       False, _fmt_string, 0.0),
    ]

    try:
        from robot_msgs.msg import AprilTagDetectionArray, Pose2DWithFlag
        table += [
            ('vision', '/front_cam/tag_detections', AprilTagDetectionArray,
             False, _fmt_tags, 0.0),
            ('vision', '/side_cam/tag_detections', AprilTagDetectionArray,
             False, _fmt_tags, 0.0),
            ('vision', '/hand_cam/tag_detections', AprilTagDetectionArray,
             False, _fmt_tags, 0.0),
            ('node',   '/robot_pose', Pose2DWithFlag, False, _fmt_pose2d, 0.0),
        ]
    except Exception:
        table.append(('vision', 'robot_msgs topics', None, False, None, 0.0))

    return table


class _Collector:
    """Counts messages on one topic and keeps the last one for the sample line."""

    def __init__(self):
        self.count = 0
        self.last = None
        self.first_stamp = None
        self.last_stamp = None

    def cb(self, msg):
        now = time.time()
        if self.first_stamp is None:
            self.first_stamp = now
        self.last_stamp = now
        self.count += 1
        self.last = msg

    def hz(self, window_s):
        if self.count < 2 or self.last_stamp is None:
            return self.count / window_s if window_s > 0 else 0.0
        span = self.last_stamp - self.first_stamp
        return (self.count - 1) / span if span > 0 else 0.0


def test_ros(rep, args):
    header("6. ROS — master, nodes, services, topic data reception")

    try:
        import rospy
        import rosgraph
        import rosnode
    except ImportError as e:
        rep.add('ros', 'rospy import', 'FAIL',
                f"{e} — source devel/setup.bash first")
        return

    if not rosgraph.is_master_online():
        rep.add('ros', 'roscore', 'FAIL',
                f"master offline at {os.environ.get('ROS_MASTER_URI', '?')}")
        return
    rep.add('ros', 'roscore', 'PASS', os.environ.get('ROS_MASTER_URI', ''))

    rospy.init_node('test_all_devices', anonymous=True, disable_signals=True)

    # ---- nodes ----
    try:
        alive = set(rosnode.get_node_names())
    except Exception as e:
        alive = set()
        rep.add('ros', 'node list', 'WARN', f"{type(e).__name__}: {e}")
    for name, required in EXPECTED_NODES:
        if name in alive:
            rep.add('ros', f"node {name}", 'PASS', 'running')
        else:
            rep.add('ros', f"node {name}", 'FAIL' if required else 'SKIP',
                    'not running')

    # ---- services ----
    for name, required in EXPECTED_SERVICES:
        try:
            rospy.wait_for_service(name, timeout=1.0)
            rep.add('ros', f"service {name}", 'PASS', 'advertised')
        except Exception:
            rep.add('ros', f"service {name}", 'FAIL' if required else 'SKIP',
                    'not advertised')

    # ---- topic data reception ----
    table = _topic_table()
    subs, collectors = [], {}
    for group, topic, msg_type, required, fmt, min_hz in table:
        if msg_type is None:
            rep.add('ros', topic, 'SKIP',
                    'robot_msgs not built/sourced — detection topics unchecked')
            continue
        c = _Collector()
        collectors[topic] = c
        subs.append(rospy.Subscriber(topic, msg_type, c.cb, queue_size=10))

    print(f"\n  Sampling {len(collectors)} topics for {args.duration:.1f}s ...\n")
    t_end = time.time() + args.duration
    while time.time() < t_end and not rospy.is_shutdown():
        time.sleep(0.05)
    for s in subs:
        s.unregister()

    for group, topic, msg_type, required, fmt, min_hz in table:
        if msg_type is None:
            continue
        c = collectors[topic]
        label = f"{topic}"
        if c.count == 0:
            # /cmd_vel idling silently is correct, not a fault.
            if topic == '/cmd_vel':
                rep.add('ros', label, 'PASS', 'no traffic (robot idle) — expected')
            else:
                rep.add('ros', label, 'FAIL' if required else 'SKIP',
                        'no data received')
            continue
        hz = c.hz(args.duration)
        try:
            sample = fmt(c.last)
        except Exception as e:
            sample = f"<unformattable: {type(e).__name__}>"
        detail = f"{c.count} msg  {hz:.1f} Hz  |  {sample}"
        if min_hz > 0 and hz < min_hz:
            rep.add('ros', label, 'WARN', f"{detail}  (below {min_hz:.1f} Hz)")
        else:
            rep.add('ros', label, 'PASS', detail)

    # ---- optional: exercise the capture service ----
    if args.capture:
        _test_capture(rep)


def _test_capture(rep):
    """Opt-in: /camera/capture lights the VISION lamp, so it is never automatic."""
    import rospy
    try:
        from robot_msgs.srv import CaptureImages
    except Exception as e:
        rep.add('ros', '/camera/capture call', 'SKIP',
                f"robot_msgs unavailable: {e}")
        return
    try:
        rospy.wait_for_service('/camera/capture', timeout=5.0)
        proxy = rospy.ServiceProxy('/camera/capture', CaptureImages)
        resp = proxy(num_samples=1, delay_between_s=-1.0, use_vision_led=True)
    except Exception as e:
        rep.add('ros', '/camera/capture call', 'FAIL', f"{type(e).__name__}: {e}")
        return
    if resp.success and resp.images:
        img = resp.images[0]
        rep.add('ros', '/camera/capture call', 'PASS',
                f"{len(resp.images)} frame(s), {img.width}x{img.height} {img.encoding}")
    else:
        rep.add('ros', '/camera/capture call', 'FAIL',
                f"success={resp.success} msg='{resp.message}' "
                f"frames={len(resp.images)}")


# ============================================================
# MAIN
# ============================================================
SECTIONS = {
    'net':     test_net,
    'arm':     test_arm,
    'keyence': test_keyence,
    'basler':  test_basler,
    'usb':     test_usb,
    'ros':     test_ros,
}


def main():
    ap = argparse.ArgumentParser(
        description='Full-system device connection + data reception test.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='sections: ' + ', '.join(SECTIONS))
    ap.add_argument('sections', nargs='*', default=[],
                    help='sections to run (default: all)')
    ap.add_argument('-d', '--duration', type=float, default=3.0,
                    help='topic sampling window in seconds (default 3.0)')
    ap.add_argument('--net-timeout', type=float, default=2.0,
                    help='per-host network timeout in seconds (default 2.0)')
    ap.add_argument('--capture', action='store_true',
                    help='also call /camera/capture (lights the VISION lamp)')
    args = ap.parse_args()

    selected = args.sections or list(SECTIONS)
    unknown = [s for s in selected if s not in SECTIONS]
    if unknown:
        ap.error(f"unknown section(s): {', '.join(unknown)}")

    rep = Report()
    print(f"Mobile manipulator device test — sections: {', '.join(selected)}")
    for name in selected:
        try:
            SECTIONS[name](rep, args)
        except KeyboardInterrupt:
            print("\n  interrupted")
            break
        except Exception as e:
            rep.add(name, f"{name} section", 'FAIL',
                    f"unhandled {type(e).__name__}: {e}")

    rep.summary()
    sys.exit(0 if rep.ok() else 1)


if __name__ == '__main__':
    main()
