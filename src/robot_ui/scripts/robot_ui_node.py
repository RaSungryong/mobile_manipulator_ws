#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Entry point for the operator UI.

    rosrun robot_ui robot_ui_node.py
    roslaunch robot_ui robot_ui.launch

This is a ROS node that happens to have a window. It owns no device: every
capability it offers is a topic or a service on the node that owns the
hardware. It is safe to start and stop at any time — nothing else depends on
it, and killing it cannot leave a device half-configured. The one thing it asks
another node to hold on its behalf is the Basler staying open for preview, and
RosBridge.shutdown() releases that.

ROS parameters
--------------
  ~plugin_dir  directory of hot-reloadable operator scripts
               (default robot_ui/plugins)
  ~save_dir    default capture folder (default /tmp/robot_ui_captures)
"""

import os
import signal
import sys

import rospy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from robot_ui import paths
from robot_ui.main_window import MainWindow
from robot_ui.ros_bridge import RosBridge


# Qt platform plugins that draw nothing and therefore need no X display.
HEADLESS_PLATFORMS = ('offscreen', 'minimal', 'minimalegl', 'linuxfb', 'vnc')


def check_display():
    """Fail with a usable message when there is no X display.

    Without this, Qt aborts with

        qt.qpa.xcb: could not connect to display
        Could not load the Qt platform plugin "xcb" ...
        Reinstalling the application may fix this problem.

    which sends people to reinstall packages over an unset environment
    variable. roslaunch then reports only "process has died ... exit code -6".
    The realistic causes are an ssh session, a systemd unit, or a terminal that
    lost DISPLAY — the X server is usually running fine.

    Returns True when it is safe to build a QApplication.
    """
    platform = os.environ.get('QT_QPA_PLATFORM', '')
    if platform in HEADLESS_PLATFORMS:
        return True
    if os.environ.get('DISPLAY'):
        return True

    sockets = []
    try:
        sockets = sorted(f':{n[1:]}' for n in os.listdir('/tmp/.X11-unix')
                         if n.startswith('X'))
    except OSError:
        pass

    hint = (f'A display server IS running on {", ".join(sockets)} — this shell '
            f'just has no DISPLAY set.\n         Try:  DISPLAY={sockets[0]} '
            f'roslaunch robot_ui robot_ui.launch'
            if sockets else
            'No X socket found under /tmp/.X11-unix either, so there is '
            'probably no desktop session on this machine.')

    rospy.logfatal(
        'robot_ui needs an X display and DISPLAY is not set.\n'
        f'         {hint}\n'
        '         The launch file normally handles this via '
        '$(optenv DISPLAY :0); it was likely overridden.\n'
        '         For a windowless run (tests, CI): '
        'QT_QPA_PLATFORM=offscreen')
    return False


def main():
    if not check_display():
        sys.exit(1)

    app = QApplication(sys.argv)

    # rospy.init_node is called with disable_signals=True inside RosBridge, so
    # Ctrl-C reaches Qt rather than rospy's handler. Without this default
    # restored, SIGINT is ignored entirely while the Qt event loop runs.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    bridge = RosBridge()
    window = MainWindow(
        bridge,
        plugin_dir=rospy.get_param('~plugin_dir', paths.PLUGIN_DIR),
        save_dir=rospy.get_param('~save_dir', paths.DEFAULT_SAVE_DIR),
    )
    window.show()

    # Let the Qt loop notice a rosmaster shutdown (roslaunch killing us, or
    # rosnode kill) instead of leaving an orphan window on screen.
    watchdog = QTimer()
    watchdog.timeout.connect(
        lambda: app.quit() if rospy.is_shutdown() else None)
    watchdog.start(500)

    exit_code = app.exec_()
    try:
        bridge.shutdown()
    except Exception:
        pass
    rospy.signal_shutdown('ui closed')
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
