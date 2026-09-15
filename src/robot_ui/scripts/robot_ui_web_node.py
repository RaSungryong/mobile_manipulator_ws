#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Entry point for the WEB operator UI (2026-09-15).

    rosrun robot_ui robot_ui_web_node.py
    roslaunch robot_ui robot_ui_web.launch          # or as part of
    roslaunch apriltag_nav mobile_manipulator.launch  (use_web_ui, default true)

Same contract as robot_ui_node.py (the PyQt window): a ROS node that owns
no device — every button is a topic or a service on the node that owns the
hardware, through the same RosBridge. The difference is where the buttons
are: this node serves the page over HTTP and streams state and camera
frames over one WebSocket, so any computer on the site LAN opens
http://<robot PC>:8080 in a browser and every open tab sees the same
robot. Nothing on the client side is installed. No X display is needed,
so it can run from a systemd unit or an ssh session.

ROS parameters
--------------
  ~port                 TCP port (default 8080)
  ~address              bind address (default 0.0.0.0 = every interface)
  ~plugin_dir           hot-reloadable operator scripts (robot_ui/plugins)
  ~save_dir             default capture folder (<ws>/results/captures)
  ~stream_max_width     frames are downscaled to this width for the wire
                        (default 1400: a 5472 px Basler frame becomes 1400)
  ~stream_fps           per-camera frame cap on the wire (default 10)
  ~jpeg_quality         default 80

Two tabs pressing buttons at once is the same as two operators at one
window: last press wins. There is no authentication — keep the port inside
the site LAN, like rosbridge's 9090.
"""

import signal
import sys
import time

import rospy

from robot_ui import paths
from robot_ui.ros_bridge import RosBridge
from robot_ui.web_server import WebServer
from robot_ui.web_ui import UiController


def main():
    bridge = RosBridge(node_name='robot_ui_web')
    port = int(rospy.get_param('~port', 8080))
    address = str(rospy.get_param('~address', '0.0.0.0'))
    plugin_dir = rospy.get_param('~plugin_dir', paths.PLUGIN_DIR)
    save_dir = rospy.get_param('~save_dir', paths.DEFAULT_SAVE_DIR)
    max_width = int(rospy.get_param('~stream_max_width', 1400))
    fps = float(rospy.get_param('~stream_fps', 10.0))
    quality = int(rospy.get_param('~jpeg_quality', 80))

    def make_controller(sink):
        return UiController(bridge, sink, plugin_dir=plugin_dir,
                            save_dir=save_dir, stream_max_width=max_width,
                            stream_fps=fps, jpeg_quality=quality)

    server = WebServer(make_controller, port=port, address=address)
    try:
        server.start_in_thread()
    except OSError as e:
        rospy.logfatal(f'[web_ui] cannot bind {address}:{port}: {e} — '
                       'another web UI (or web_video_server) on that port? '
                       'Set ~port.')
        sys.exit(1)
    rospy.loginfo(f'[web_ui] serving http://{address}:{server.bound_port}/ '
                  f'(ws on /ws) — plugins {plugin_dir}, captures {save_dir}')

    # rospy.init_node ran with disable_signals=True (RosBridge), so Ctrl-C
    # is ours to handle: stop the server (which releases the Basler hold),
    # then tell rospy.
    def _stop(*_):
        rospy.signal_shutdown('web ui stopped')

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)
    try:
        while not rospy.is_shutdown():
            time.sleep(0.25)
    finally:
        server.stop()


if __name__ == '__main__':
    main()
