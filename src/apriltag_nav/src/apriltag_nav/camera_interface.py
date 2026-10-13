#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import resource

import rospy
from pypylon import pylon


# pylon 3.0.1 (pypylon 3.0.1) LEAKS file descriptors (2026-10-13): every
# TlFactory.CreateDevice + InstantCamera construction keeps 4 descriptors on
# /proc/self/status open for good (DestroyDevice does not return them) and
# every Open() one more; StartGrabbing / RetrieveResult leak nothing. The
# node used to enumerate + create + open the device at EVERY scan point
# (pre-open, idle close 5 s later while the operator writes the number), so
# a collection run burnt ~5 descriptors per point and the default soft limit
# of 1024 was reached after ~180 points — the grab then fails with
# `WaitObject duplicate failed: Too many open files` and the scan walks on
# without frames. Two defences, both here: the device object is created
# ONCE and only Open()ed / Close()d afterwards (1 descriptor per cycle), and
# the process raises its soft RLIMIT_NOFILE to the hard limit (524288 on the
# robot PC) at start, so the remaining leak is a restart-every-few-months
# matter, not a per-run one. `fd_count()` / the warning in _initialize_basler
# make it visible before it bites.

def ensure_fd_headroom(min_soft=65536):
    """Raise this process's soft open-files limit to the hard limit (or at
    least `min_soft` when the hard limit allows). Returns (soft, hard) after
    the change; never raises."""
    try:
        soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
        want = hard if hard != resource.RLIM_INFINITY else max(soft, min_soft)
        if want > soft:
            resource.setrlimit(resource.RLIMIT_NOFILE, (want, hard))
            soft = want
        return soft, hard
    except Exception as e:                       # pragma: no cover
        rospy.logwarn(f"[Camera] could not raise the open-files limit: {e}")
        try:
            return resource.getrlimit(resource.RLIMIT_NOFILE)
        except Exception:
            return (0, 0)


def fd_count():
    """Open descriptors of this process (0 when /proc is unreadable)."""
    try:
        return len(os.listdir('/proc/self/fd'))
    except Exception:
        return 0


class CameraInterface:
    """Basler (pypylon) camera interface — no webcam fallback.

    The pylon device object (`_device_cam`) is created on the first
    initialize() and REUSED by every later one (Open/Close only) — see the
    module note on the descriptor leak. `camera` is the open handle (None
    while closed), which is what every other method and the node test on.
    """

    def __init__(self):
        self.camera = None
        self.camera_type = None
        self.converter = None
        self._device_cam = None      # the pylon InstantCamera, kept across close()
        self._fd_warned = False
        soft, hard = ensure_fd_headroom()
        rospy.loginfo(f"[Camera] open-files limit: soft {soft} / hard {hard}, "
                      f"{fd_count()} in use")
        # 'mono8' or 'bgr8' — the ROS encoding grab_frame() returns. Decided
        # from the sensor at open time, not assumed. See _configure_converter.
        self.encoding = 'bgr8'

        try:
            self.converter = pylon.ImageFormatConverter()
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        except Exception as e:
            rospy.logwarn(f"[Camera] ImageFormatConverter init failed: {e}")

    def initialize(self):
        return self._initialize_basler()

    def _create_device(self):
        """Enumerate + create the pylon device object (leaks 4 descriptors per
        call in pylon 3.0.1 — hence done once and reused)."""
        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()
        if len(devices) == 0:
            rospy.logwarn("[Camera] No Basler camera found")
            return None
        return pylon.InstantCamera(tlFactory.CreateDevice(devices[0]))

    def _discard_device(self):
        cam, self._device_cam = self._device_cam, None
        if cam is None:
            return
        for step in ('Close', 'DestroyDevice'):
            try:
                getattr(cam, step)()
            except Exception:
                pass

    def _check_fd_headroom(self):
        try:
            soft, _ = resource.getrlimit(resource.RLIMIT_NOFILE)
        except Exception:
            return
        n = fd_count()
        if soft and n > 0.9 * soft and not self._fd_warned:
            self._fd_warned = True
            rospy.logerr(f"[Camera] {n} of {soft} file descriptors in use — pylon leaks "
                         "~1 per device open; restart basler_camera_node soon or captures "
                         "will fail with 'Too many open files'")

    def _initialize_basler(self):
        try:
            self._check_fd_headroom()
            if self._device_cam is None:
                self._device_cam = self._create_device()
                if self._device_cam is None:
                    return False
            self.camera = self._device_cam
            try:
                self.camera.Open()
            except Exception as e:
                # The cached device may be stale (camera power-cycled, link
                # lost): drop it and create a fresh one, once.
                rospy.logwarn(f"[Camera] reopen of the cached device failed ({e}); recreating")
                self.camera = None
                self._discard_device()
                self._device_cam = self._create_device()
                if self._device_cam is None:
                    return False
                self.camera = self._device_cam
                self.camera.Open()

            # Reset to factory defaults to ensure clean state
            self.camera.UserSetSelector.SetValue("Default")
            self.camera.UserSetLoad.Execute()

            self.camera.AcquisitionFrameRateEnable.SetValue(True)
            self.camera.GainRaw.SetValue(0)
            self.camera.ExposureAuto.SetValue('Off')
            self.camera.ExposureTimeAbs.SetValue(4000.0)  # 4ms
            self.camera.BlackLevelRaw.SetValue(0)
            self.camera.AcquisitionMode.SetValue("Continuous")
            self.camera_type = 'basler'
            self._configure_converter()
            rospy.loginfo(
                f"[Camera] Basler: {self.camera.GetDeviceInfo().GetModelName()} "
                f"-> {self.encoding}")
            return True
        except Exception as e:
            rospy.logwarn(f"[Camera] Basler init failed: {e}")
            if self.camera:
                try:
                    self.camera.Close()
                except Exception:
                    pass
                self.camera = None
            return False

    def _configure_converter(self):
        """Pick the output format from what the sensor actually delivers.

        ⚠️ A MONO SENSOR MUST NOT BE EXPANDED TO BGR. This node's camera is an
        acA5472-5gm — mono — and the converter used to force BGR8packed
        unconditionally, so every frame left here as 5472x3648x3 = 59.9 MB with
        three identical channels. Verified on the real camera: B == G == R
        exactly, on every pixel. That is 40 MB per frame of duplicated bytes,
        paid twice (the service reply and the /basler/image_raw echo), on a
        5 fps part where the whole budget is 200 ms.

        Nothing downstream needs the expansion. RaPredictor.preprocess accepts a
        2-D frame and does GRAY2RGB itself, and the model input tensor is
        bit-identical either way (verified: max |diff| = 0.0). cv_bridge widens
        mono8 for any consumer that asks for bgr8.

        A colour camera still gets BGR8packed — the choice follows the sensor.
        """
        if self.converter is None:
            return
        try:
            fmt = str(self.camera.PixelFormat.GetValue())
        except Exception as e:
            rospy.logwarn(f"[Camera] PixelFormat unreadable ({e}); keeping bgr8")
            return

        if fmt.lower().startswith('mono'):
            self.converter.OutputPixelFormat = pylon.PixelType_Mono8
            self.encoding = 'mono8'
        else:
            self.converter.OutputPixelFormat = pylon.PixelType_BGR8packed
            self.encoding = 'bgr8'
        rospy.loginfo(f"[Camera] sensor PixelFormat={fmt} -> ROS {self.encoding}")

    def start_grabbing(self):
        if self.camera is None:
            return False
        if self.camera_type == 'basler' and self.camera.IsOpen():
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        return True

    def grab_frame(self, timeout=5000):
        if self.camera is None:
            return None
        try:
            return self._grab_basler(timeout)
        except Exception as e:
            rospy.logerr(f"[Camera] grab error: {e}")
        return None

    def _grab_basler(self, timeout):
        if not self.camera.IsGrabbing():
            return None
        result = self.camera.RetrieveResult(timeout, pylon.TimeoutHandling_ThrowException)
        if result.GrabSucceeded():
            img = self.converter.Convert(result).GetArray()
            result.Release()
            return img
        result.Release()
        return None

    def stop_grabbing(self):
        if self.camera is None:
            return
        try:
            if self.camera_type == 'basler' and self.camera.IsGrabbing():
                self.camera.StopGrabbing()
        except Exception as e:
            rospy.logwarn(f"[Camera] stop error: {e}")

    def close(self):
        """Stop grabbing + Close() the device (powers the sensor down). The
        pylon device object is KEPT for the next initialize() — re-creating
        it is what leaked descriptors (module note)."""
        if self.camera is None:
            return
        try:
            if self.camera.IsGrabbing():
                self.camera.StopGrabbing()
            if self.camera.IsOpen():
                self.camera.Close()
            rospy.loginfo("[Camera] Basler closed")
        except Exception as e:
            rospy.logwarn(f"[Camera] close error: {e}")
        self.camera = None
        self.camera_type = None
