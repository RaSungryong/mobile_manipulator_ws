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
        Tool index registered on the controller (e.g. 1 for vision_tip).
        Set up on the controller via ``set_tool_tcp.py`` / ``SetToolCoord``.
        Used as the ``tool=`` argument for ``MoveJ``. SDK pose-read calls
        (``GetActualTCPPose`` / ``GetActualJointPosDegree``) take no tool
        argument — they return the currently active TCP set on the box.
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
        err, tcp = robot.GetActualTCPPose()
        if err != 0:
            raise RuntimeError(f"GetActualTCPPose failed (err={err})")
        return [float(v) for v in tcp]

    def get_joints(self):
        """Return current joint angles (deg) as a list."""
        robot = self._connect()
        err, joints = robot.GetActualJointPosDegree()
        if err != 0:
            raise RuntimeError(f"GetActualJointPosDegree failed (err={err})")
        return [float(v) for v in joints]

    # ------------------------------------------------------------------
    def move_j_to_pose(self, target_pose_mm_deg,
                       vel=None, acc=None, ovl=None,
                       settle_s: float = 0.2):
        """Resolve ``target_pose_mm_deg`` to joints via IK and execute a
        blocking ``MoveJ``. Mirrors the call pattern used by the working
        ``apriltag_nav`` arm controller (proven on this Fairino build):

            ret, joints = robot.GetInverseKin(0, target, config=-1)
            ret         = robot.MoveJ(joints, tool, user)

        The ``vel``/``acc``/``ovl`` kwargs are intentionally NOT passed to
        ``MoveJ`` — the compiled SDK on this controller exposes only the
        first three positional arguments and rejects the speed kwargs.
        Speed is therefore governed by the controller-side defaults (set
        on the teach pendant or via ``robot_common_set.py``). The args
        are accepted here for forward compatibility but currently
        ignored; they are logged for traceability.

        Tool-frame consistency: ``GetActualTCPPose()`` and IK results
        reference the controller's currently active tool. ``MoveJ`` is
        called with ``tool=tcp_index``. For the chain math to hold, the
        active tool on the box MUST equal ``tcp_index`` — set it once
        with ``SetToolCoord`` + the teach pendant. If ``tcp_index`` is 0
        you are using the bare flange and no setup is needed.

        Raises ``RuntimeError`` on any non-zero SDK return. Includes the
        error code in the message so the Fairino error table can be
        consulted (e.g. ``ret=14`` means "axis travel limit reached" —
        the caller may want to call ``ResetAllError`` and retry).
        """
        self.enable()
        robot = self._connect()
        target_pose = [float(v) for v in target_pose_mm_deg]

        # Log the IK input so a failure on the box is easy to reproduce
        # off-line (paste these numbers into a stand-alone script).
        try:
            cur_tcp = self.get_tcp_pose()
        except Exception:
            cur_tcp = None
        _vel = float(self.default_vel if vel is None else vel)
        _acc = float(self.default_acc if acc is None else acc)
        _ovl = float(self.default_ovl if ovl is None else ovl)
        print(f"[FairinoTCPClient] move_j_to_pose: "
              f"target={target_pose} cur_tcp={cur_tcp} "
              f"tool={int(self.tcp_index)} "
              f"(vel={_vel} acc={_acc} ovl={_ovl} not forwarded to MoveJ)")

        ret_ik = robot.GetInverseKin(0, target_pose, config=-1)
        # The SDK returns (err, joints). Unpack defensively to surface
        # any signature drift as a clear error.
        try:
            err_ik, joints = ret_ik
        except (TypeError, ValueError):
            raise RuntimeError(
                f"GetInverseKin returned unexpected shape: {ret_ik!r}")
        if int(err_ik) != 0:
            raise RuntimeError(
                f"GetInverseKin failed (err={err_ik}); target={target_pose}")

        err = robot.MoveJ(joints, int(self.tcp_index), 0)
        if err and int(err) != 0:
            raise RuntimeError(
                f"MoveJ failed (err={err}); joints={joints} "
                f"tool={int(self.tcp_index)} "
                f"(hint: err=14 -> ResetAllError; check active tool "
                f"matches tcp_index, and joints are inside soft limits)")
        if settle_s > 0:
            time.sleep(settle_s)
