#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware connection test script.
Tests Fairino arm, Keyence DL-EN1, and Basler camera independently.

Usage:
    python3 test_hardware.py             # test all
    python3 test_hardware.py arm         # test arm only
    python3 test_hardware.py keyence     # test keyence only
    python3 test_hardware.py basler      # test basler only
"""

import sys
import os
import socket
import time

# ============================================================
# CONFIG — edit if your IPs differ
# ============================================================
ARM_IP       = '192.168.58.2'
KEYENCE_HOST = '192.168.100.105'
KEYENCE_PORT = 64000

# Fairino SDK path
_FAIRINO_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', 'fairino_sdk', 'fairino-python-sdk', 'Linux'
)

# ============================================================
# HELPERS
# ============================================================
def _ok(msg):  print(f"  [PASS] {msg}")
def _fail(msg): print(f"  [FAIL] {msg}")
def _info(msg): print(f"  [INFO] {msg}")


# ============================================================
# TEST 1 — FAIRINO ARM
# ============================================================
def test_arm():
    print("\n=== Fairino FR10v6 Arm ===")
    print(f"  Target: {ARM_IP}:8080")

    # Quick TCP reachability check first
    try:
        s = socket.create_connection((ARM_IP, 8080), timeout=3.0)
        s.close()
        _ok("TCP port 8080 reachable")
    except Exception as e:
        _fail(f"TCP port 8080 not reachable: {e}")
        return False

    # Fairino SDK connection
    try:
        if _FAIRINO_PATH not in sys.path:
            sys.path.append(_FAIRINO_PATH)
        from fairino import Robot

        _info("Connecting via Fairino SDK...")
        robot = Robot.RPC(ARM_IP)
        time.sleep(0.5)

        ret, joints = robot.GetActualJointPosRadian()
        if ret == 0:
            deg = [round(j * 57.296, 2) for j in joints]
            _ok(f"SDK connected — joint pos (deg): {deg}")
        else:
            _fail(f"GetActualJointPosRadian returned: {ret}")
            return False

        ret, tcp = robot.GetActualTCPPose()
        if ret == 0:
            _ok(f"TCP pose: {[round(v, 3) for v in tcp]}")
        else:
            _info(f"GetActualTCPPose returned: {ret} (non-critical)")

        return True

    except Exception as e:
        _fail(f"SDK error: {e}")
        return False


# ============================================================
# TEST 2 — KEYENCE DL-EN1
# ============================================================
def test_keyence():
    print("\n=== Keyence DL-EN1 ===")
    print(f"  Target: {KEYENCE_HOST}:{KEYENCE_PORT}")

    # TCP reachability
    try:
        s = socket.create_connection((KEYENCE_HOST, KEYENCE_PORT), timeout=3.0)
        _ok(f"TCP port {KEYENCE_PORT} reachable")
    except Exception as e:
        _fail(f"TCP connection failed: {e}")
        return False

    # Send measurement command M0 + CRLF
    try:
        payload = b"M0\r\n"
        s.settimeout(1.0)
        s.sendall(payload)
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

        resp = b"".join(chunks)
        text = resp.decode(errors="ignore").strip()

        if not text:
            _fail("No response from sensor")
            return False

        if text.startswith("ER,"):
            _fail(f"Sensor returned error: {text}")
            return False

        # Parse value
        parts = text.split(",")
        if len(parts) >= 2:
            try:
                raw = int(parts[1].strip())
                value_mm = raw * 0.001
                _ok(f"Measurement received — raw: {raw}  value: {value_mm:.3f} mm")
                _ok(f"Full response: '{text}'")
                return True
            except ValueError:
                pass

        _ok(f"Response: '{text}' (parse OK)")
        return True

    except Exception as e:
        _fail(f"Communication error: {e}")
        return False


# ============================================================
# TEST 3 — BASLER CAMERA
# ============================================================
def test_basler():
    print("\n=== Basler Camera (PyPylon) ===")

    try:
        from pypylon import pylon
    except ImportError:
        _fail("pypylon not installed — run: pip install pypylon")
        return False

    try:
        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()
    except Exception as e:
        _fail(f"TlFactory error: {e}")
        return False

    if len(devices) == 0:
        _fail("No Basler camera found (check USB/GigE connection)")
        return False

    for i, d in enumerate(devices):
        _info(f"Camera [{i}]: {d.GetModelName()} — S/N {d.GetSerialNumber()} — {d.GetDeviceClass()}")

    # Open first camera and grab one frame
    try:
        camera = pylon.InstantCamera(tlFactory.CreateDevice(devices[0]))
        camera.Open()
        _ok(f"Camera opened: {camera.GetDeviceInfo().GetModelName()}")

        # Apply settings
        camera.UserSetSelector.SetValue("Default")
        camera.UserSetLoad.Execute()
        camera.AcquisitionFrameRateEnable.SetValue(True)
        camera.GainRaw.SetValue(0)
        camera.ExposureAuto.SetValue('Off')
        camera.ExposureTimeAbs.SetValue(4000.0)
        camera.AcquisitionMode.SetValue("Continuous")

        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

        result = camera.RetrieveResult(3000, pylon.TimeoutHandling_ThrowException)
        if result.GrabSucceeded():
            img = converter.Convert(result)
            arr = img.GetArray()
            _ok(f"Frame grabbed: {arr.shape[1]}x{arr.shape[0]} px  dtype={arr.dtype}")
        else:
            _fail(f"Grab failed: {result.ErrorDescription}")
            result.Release()
            camera.StopGrabbing()
            camera.Close()
            return False

        result.Release()
        camera.StopGrabbing()
        camera.Close()
        return True

    except Exception as e:
        _fail(f"Camera error: {e}")
        return False


# ============================================================
# MAIN
# ============================================================
def main():
    targets = sys.argv[1:] if len(sys.argv) > 1 else ['arm', 'keyence', 'basler']

    results = {}

    if 'arm' in targets:
        results['arm'] = test_arm()

    if 'keyence' in targets:
        results['keyence'] = test_keyence()

    if 'basler' in targets:
        results['basler'] = test_basler()

    # Summary
    print("\n" + "=" * 40)
    print("Summary")
    print("=" * 40)
    all_pass = True
    for name, ok in results.items():
        status = "PASS" if ok else "FAIL"
        print(f"  {name:<12} {status}")
        if not ok:
            all_pass = False

    print("=" * 40)
    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
