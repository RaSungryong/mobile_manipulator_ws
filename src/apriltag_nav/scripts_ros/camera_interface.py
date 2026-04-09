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
            rospy.loginfo(f"[Camera] Basler: {self.camera.GetDeviceInfo().GetModelName()}")
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
