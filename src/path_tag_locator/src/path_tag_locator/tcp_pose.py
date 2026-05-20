"""
tcp_pose.py
===========
Fairino arm client.

Provides:
  - get_tcp_pose()        -> [x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg]
  - enable()              -> RobotEnable(1) + Mode(1) (idempotent)
  - move_j_to_pose(pose)  -> linear-ish move via GetInverseKinRef + MoveJ

Move strategy mirrors the convention used in apriltag_nav: resolve a target
Cartesian descriptor pose to joint angles via the Fairino IK service, then
``MoveJ`` to those joints. The current joint configuration is used as a
seed (``GetInverseKinRef``) to reduce ambiguity and avoid large joint
flips. MoveJ runs in joint space, but with a close seed and small steps
the end-effector path is approximately linear.
"""
import sys
import time
from pathlib import Path


class FairinoTCPClient:
    """Thin wrapper around the Fairino RPC client.

    Parameters
    ----------
    robot_ip : str
        IP of the controller.
    sdk_path : str | None
        Inserted at the front of ``sys.path`` before importing ``fairino``.
    tcp_index : int
        Flag passed to ``GetActualTCPPose`` (typically 1).
    default_vel : float
        Default MoveJ velocity percentage (0-100).
    default_acc : float
        Default MoveJ acceleration percentage (0-100).
    default_ovl : float
        Default MoveJ overlap percentage (0-100, 100 = no blending).
    """

    def __init__(self, robot_ip: str, sdk_path=None, tcp_index: int = 1,
                 default_vel: float = 20.0,
                 default_acc: float = 20.0,
                 default_ovl: float = 100.0):
        self.robot_ip = robot_ip
        self.tcp_index = tcp_index
        self.default_vel = float(default_vel)
        self.default_acc = float(default_acc)
        self.default_ovl = float(default_ovl)
        if sdk_path:
            sp = str(Path(sdk_path).expanduser())
            if sp not in sys.path:
                sys.path.insert(0, sp)
        self._robot = None
        self._enabled = False

    # ------------------------------------------------------------------
    def _connect(self):
        if self._robot is None:
            from fairino import Robot  # lazy import
            self._robot = Robot.RPC(self.robot_ip)
        return self._robot

    def enable(self):
        """Enable the robot and switch to remote-program mode. Idempotent."""
        robot = self._connect()
        if not self._enabled:
            err = robot.RobotEnable(1)
            if err and err != 0:
                raise RuntimeError(f"RobotEnable failed (err={err})")
            err = robot.Mode(1)
            if err and err != 0:
                raise RuntimeError(f"Mode(1) failed (err={err})")
            self._enabled = True

    # ------------------------------------------------------------------
    def get_tcp_pose(self):
        """Return [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]."""
        robot = self._connect()
        err, tcp = robot.GetActualTCPPose(self.tcp_index)
        if err != 0:
            raise RuntimeError(f"GetActualTCPPose failed (err={err})")
        return [float(v) for v in tcp]

    def get_joints(self):
        """Return current joint angles (deg) as a list."""
        robot = self._connect()
        err, joints = robot.GetActualJointPosDegree(self.tcp_index)
        if err != 0:
            raise RuntimeError(f"GetActualJointPosDegree failed (err={err})")
        return [float(v) for v in joints]

    # ------------------------------------------------------------------
    def move_j_to_pose(self, target_pose_mm_deg,
                       vel=None, acc=None, ovl=None,
                       settle_s: float = 0.2):
        """Resolve ``target_pose_mm_deg`` to joints via IK (using the
        current joint state as a reference) and execute a blocking MoveJ.

        Raises ``RuntimeError`` on any non-zero SDK return.
        """
        self.enable()
        robot = self._connect()
        cur_joints = self.get_joints()
        target_pose = [float(v) for v in target_pose_mm_deg]

        ret = robot.GetInverseKinRef(0, target_pose, cur_joints, -1)
        # SDK varies: some return (err, joints); some (err) writing into a
        # mutable arg; some return joints only. Normalize:
        if isinstance(ret, tuple) and len(ret) == 2:
            err_ik, joints = ret
        else:
            # Try the older single-return-list shape.
            err_ik, joints = 0, ret
        if err_ik and int(err_ik) != 0:
            # Fallback: untreaded IK without seed.
            ret = robot.GetInverseKin(0, target_pose, -1)
            if isinstance(ret, tuple) and len(ret) == 2:
                err_ik, joints = ret
            else:
                err_ik, joints = 0, ret
            if err_ik and int(err_ik) != 0:
                raise RuntimeError(f"GetInverseKin* failed (err={err_ik})")

        v = float(self.default_vel if vel is None else vel)
        a = float(self.default_acc if acc is None else acc)
        o = float(self.default_ovl if ovl is None else ovl)

        err = robot.MoveJ(joints, target_pose, 0, 0,
                          v, a, o, [0.0] * 6, 0, 0)
        if err and int(err) != 0:
            raise RuntimeError(f"MoveJ failed (err={err})")
        if settle_s > 0:
            time.sleep(settle_s)
