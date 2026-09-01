#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from pypylon import pylon


class CameraInterface:
    """Basler (pypylon) camera interface — no webcam fallback."""

    def __init__(self):
        self.camera = None
        self.camera_type = None
        self.converter = None
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

    def _initialize_basler(self):
        try:
            tlFactory = pylon.TlFactory.GetInstance()
            devices = tlFactory.EnumerateDevices()
            if len(devices) == 0:
                rospy.logwarn("[Camera] No Basler camera found")
                return False
            self.camera = pylon.InstantCamera(tlFactory.CreateDevice(devices[0]))
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
