"""
navigator.py
============
Thin facade over MapManager + RobotController so the calibrator can issue
``goto(tag_id)`` without reimplementing the navigation glue from
apriltag_nav/scripts/navigate.py.

Intentionally drops: CLI argparse, Excel waypoint loading, mode 1/2/3
menus, task_command/String publisher, and scan-delegation handshake.
"""
import time

import rospy
import yaml

from .map_manager import MapManager
from .robot_controller import RobotController


class Navigator:
    """High-level wrapper bundling ``MapManager`` + ``RobotController``.

    Parameters
    ----------
    robot_cfg : dict | str
        Either an already-loaded yaml dict or a path to robot_nav.yaml
        (the subset of apriltag_nav/config/robot.yaml that holds
        ``robot.*`` + ``topics.*``).
    map_yaml_path : str
        Path to map.yaml.
    wait_for_camera_s : float
        Block up to this many seconds inside ``__init__`` waiting for the
        first CameraInfo callback (so ``RobotController.camera_params``
        is populated before the first ``goto``). 0 disables the wait.
    """

    def __init__(self, robot_cfg, map_yaml_path: str,
                 wait_for_camera_s: float = 5.0,
                 enable_scan_signal: bool = False):
        if isinstance(robot_cfg, str):
            with open(robot_cfg, "r") as fh:
                robot_cfg = yaml.safe_load(fh)
        self.cfg = robot_cfg
        self.map_mgr = MapManager(map_yaml_path)
        self.robot = RobotController(
            robot_cfg, self.map_mgr, enable_scan_signal=enable_scan_signal)
        if wait_for_camera_s > 0:
            self._wait_for_camera(wait_for_camera_s)

    # ------------------------------------------------------------------
    def _wait_for_camera(self, timeout_s: float):
        rospy.loginfo("[Navigator] waiting for camera_info (≤ %.1fs) ...",
                      timeout_s)
        deadline = time.time() + timeout_s
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.robot.camera_params is not None:
                rospy.loginfo("[Navigator] camera ready.")
                return
            rate.sleep()
        rospy.logwarn(
            "[Navigator] camera_info NOT received within %.1fs — "
            "first goto may stall.", timeout_s)

    # ------------------------------------------------------------------
    def goto(self, target_id: int, start_id=None) -> bool:
        """Drive the base to ``target_id``. Returns True on success."""
        if start_id is not None:
            self.robot.last_known_tag = int(start_id)
        return bool(self.robot.move_to_tag(int(target_id)))

    def current_tag_id(self):
        return self.robot.get_current_tag_id()

    def stop(self):
        self.robot.stop()

    def shutdown(self):
        try:
            self.robot.stop()
        except Exception:
            pass
