#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera viewer node — on-demand RViz window showing all three robot cameras.

A debug aid, nothing else. It owns no device, publishes no topic and is never
required for the robot to work: it launches RViz as a child process against a
prepared config (config/robot_cameras.rviz) that shows front_cam, side_cam and
hand_cam side by side, and it kills that process again on request.

Why a node instead of just running `rviz` in another terminal: lifetime. As a
node inside mobile_manipulator.launch, roslaunch owns it, so the viewer cannot
outlive the stack it is inspecting — and because RViz is a *child* of this
node, the window goes away with it too (see _stop for why that needs care).

The three cameras are shown together on purpose. Turning individual cameras on
and off is robot_camera_node's job (/robot_camera/<name>/set_enabled); this
node only decides whether a window exists. A camera that is switched off there
simply shows no image here.

Interface
---------
Serves:
  /camera_viewer/set_enabled   std_srvs/SetBool   true = open, false = close
Params (~):
  auto_start   bool  open the window at launch (default False — the whole
                     point is that the stack comes up without a GUI)
  rviz_config  str   path to the .rviz file (default: package config/)
"""

import os
import signal
import subprocess

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

from apriltag_nav.paths import PKG_DIR

DEFAULT_CONFIG = os.path.join(PKG_DIR, 'config', 'robot_cameras.rviz')


class CameraViewerNode:
    def __init__(self):
        rospy.init_node('camera_viewer_node', anonymous=False)

        self.config = rospy.get_param('~rviz_config', DEFAULT_CONFIG)
        self.proc = None

        if not os.path.isfile(self.config):
            rospy.logwarn(f"[CameraViewer] config not found: {self.config} "
                          "— RViz will open with its own default layout")

        rospy.Service('/camera_viewer/set_enabled', SetBool, self._srv)
        # Kill the child on the way out. Without this the RViz window survives
        # Ctrl-C: roslaunch signals its own nodes, not their grandchildren.
        rospy.on_shutdown(self._stop)

        if rospy.get_param('~auto_start', False):
            self._start()

        rospy.loginfo("[CameraViewer] Ready — closed. Open with: "
                      "rosservice call /camera_viewer/set_enabled \"data: true\"")

    # ------------------------------------------------------------------
    def _srv(self, req):
        try:
            msg = self._start() if req.data else self._stop()
            return SetBoolResponse(success=True, message=msg)
        except Exception as e:
            rospy.logerr(f"[CameraViewer] {e}")
            return SetBoolResponse(success=False, message=str(e))

    def _running(self):
        return self.proc is not None and self.proc.poll() is None

    def _start(self):
        if self._running():
            return "viewer already open"

        cmd = ['rviz', '-d', self.config] if os.path.isfile(self.config) else ['rviz']
        # start_new_session puts RViz in its own process group so _stop can
        # signal the whole group: RViz spawns helpers, and killing only the
        # parent pid leaves them behind holding the X window.
        self.proc = subprocess.Popen(cmd, start_new_session=True)
        rospy.loginfo(f"[CameraViewer] opened (pid {self.proc.pid})")
        return f"viewer opened (pid {self.proc.pid})"

    def _stop(self):
        if not self._running():
            self.proc = None
            return "viewer already closed"

        pid = self.proc.pid
        try:
            # SIGINT first: RViz saves its window state and exits cleanly.
            os.killpg(os.getpgid(pid), signal.SIGINT)
            self.proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            rospy.logwarn("[CameraViewer] RViz ignored SIGINT — killing")
            try:
                os.killpg(os.getpgid(pid), signal.SIGKILL)
                self.proc.wait(timeout=5)
            except Exception as e:
                rospy.logerr(f"[CameraViewer] could not kill {pid}: {e}")
        except ProcessLookupError:
            pass  # already gone

        self.proc = None
        rospy.loginfo(f"[CameraViewer] closed (was pid {pid})")
        return f"viewer closed (was pid {pid})"


if __name__ == '__main__':
    node = CameraViewerNode()
    rospy.spin()
