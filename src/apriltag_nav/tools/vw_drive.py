#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Manual (v, w) drive tool — bring-up and debugging only.

Drives the base by publishing geometry_msgs/Twist directly, with odometry
closing the loop so a command can be given as a DISTANCE or an ANGLE rather
than as a velocity held for a guessed number of seconds:

    forward 0.3      drive 0.30 m ahead          (relative, odom-closed)
    back 0.3         drive 0.30 m in reverse
    left 90          pivot 90 deg CCW            (relative, odom-closed)
    right 90         pivot 90 deg CW
    vw 0.03 0.1 2    raw: v=0.03 m/s, w=0.1 rad/s, for 2 s   (open loop)

⚠️ THIS IS A SECOND /cmd_vel PUBLISHER. Same rule as tools/navigate.py:
`mobile_node` is the sole publisher in the running stack and there is no
arbitration below it — navifra's `base_controller` obeys whichever message
arrived last, so the two would fight at 50 Hz with no error anywhere.
**Never run this while mobile_manipulator.launch is up.** The tool refuses to
start when it sees `mobile_node`; `--force` overrides that, and is only for the
case where you know the stack is down and the node name is stale.

Three things it does that a bare `rostopic pub` cannot, all of them from
hardware behaviour recorded in CLAUDE.md:

  * Publishes at 20 Hz. `base_controller`'s `cmd_vel_timeout` is 0.5 s, so a
    single message is a twitch, not a move.
  * Aborts on a STALLED /odom. A frozen odom is how `MOTOR_FEEDBACK_TIMEOUT`
    presented on 2026-08-12: the base is dead, nothing errors, and the loop
    happily commands a velocity for its whole timeout. Here it stops and says
    so, which is the whole point of a debug tool.
  * Aborts on /safety/estop. The PILZ circuit has already cut motor power at
    that point; continuing to command a base that cannot move only hides why.

Motion profile
--------------
Trapezoidal, and closed on odom the whole way:

    v_cmd = min(v_top, sqrt(2 * accel * remaining))

then through a slew-rate limiter at `linear_accel` / `angular_accel`. Cruise
happens where the braking curve is above `v_top`, braking where it is below;
a move too short to reach `v_top` degrades to a triangle on its own. There are
no phase ratios to tune, and deceleration is CONSTANT, i.e. the velocity falls
linearly in time — which is what a smooth stop looks and feels like.

⚠️ **This replaced a copy of `MobileController._smooth_speed_factor`**, which
ramps down as `(remaining/span)**2` — quadratic in DISTANCE, evaluated against
a limiter that works in TIME. That combination judders into a stop, and both
reasons were measured in simulation against the real code:

  1. `v` proportional to `remaining**2` integrates to `remaining(t) = 1/(kt+C)`
     — hyperbolic, so it never reaches zero. It flattens onto the minimum
     speed and creeps there, and the move ends with a STEP from that speed to
     zero because `stop()` bypasses the ramp.
  2. The curve's steepest demand scales with the SQUARE of the top speed. At
     `max_linear_speed` 0.05 it asked 0.032 m/s2 of a 0.05 limit and never
     bound; at 0.1 it asks 0.125 — 2.5x over — so the limiter saturates, the
     real speed runs above the plan for the whole braking phase, and the plan
     answers by demanding an even steeper drop.

The envelope above has neither problem, because it IS the braking curve: a
speed on it satisfies `v**2/(2a) == remaining`, so the constant-accel ramp-out
lands exactly on target.

⚠️ Do not read the above as a description of navigation. **`_smooth_speed_factor`
is dead code — nothing calls it.** `execute_pure_pursuit` uses a LINEAR-in-
distance ramp, whose peak demand is `v_top*(v_top-v_floor)/(0.4*D)`: milder,
but still 1.0x the limit on a 0.40 m move and 1.95x on a 0.20 m one at the
current top speed. See CLAUDE.md.

Speed
-----
Defaults come from config/robot.yaml `max_linear_speed` / `max_angular_speed`
— deliberately the same ceiling navigation uses. There is no floor; see the
note in robot.yaml about the clamp that was added and removed on 2026-08-14.

Two ways to change the ceiling, and they are not symmetric:

  * SLOWER — a trailing argument on the command: `forward 0.3 0.02` drives at
    0.02 m/s, `left 90 0.1` pivots at 0.1 rad/s. Always available.
  * FASTER — `--vmax` / `--wmax`, which raise the ceiling itself and log a
    warning saying so. Bounded by `base_controller`'s own `max_linear_vel` /
    `max_angular_vel` (2.0 m/s / 20 rad/s) read off the param server, because
    past that the driver clamps anyway and the tool would be reporting a
    velocity it never sent.

⚠️ Those driver limits are 40x the configured speed and are NOT a safety
margin. Nothing in this system has ever driven that fast; `--vmax` exists for
bring-up questions like "does it move at all above stiction", not for getting
somewhere sooner.

Usage
-----
    python3 vw_drive.py                       # interactive prompt
    python3 vw_drive.py forward 0.3           # one command, then exit
    python3 vw_drive.py forward 0.3 0.02      # ... at 0.02 m/s
    python3 vw_drive.py --vmax 0.15 forward 0.3   # raise the ceiling first
    python3 vw_drive.py --exec "forward 0.3; left 90; back 0.3"
    python3 vw_drive.py --dry-run forward 0.3 # print, publish nothing
"""

import argparse
import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from apriltag_nav import utils
from apriltag_nav.paths import CONFIG_PATH

RATE_HZ = 20.0          # >> the 2 Hz floor set by cmd_vel_timeout 0.5 s
ODOM_STALE_S = 1.0      # no odom for this long = the driver is not talking
STALL_GRACE_S = 2.5     # commanding motion this long with no odom change = stall
STALL_EPS_M = 0.005     # movement below this does not count as moving
STALL_EPS_RAD = math.radians(0.5)

# Arrival tolerance. The loop stops steering here and hands off to the
# constant-accel ramp-out, which covers the remainder — see _ramp_to_zero.
ARRIVE_TOL_M = 0.003
RAMP_DONE_EPS = 1e-3    # commanded velocity this small counts as stopped
RAMP_OUT_MAX_S = 5.0    # bound on the ramp-out, in case the limiter never lands
ARRIVE_SHORTFALL_FRAC = 0.05    # finishing this far short of target = a failure


def _yaw_from_quat(q):
    """Planar yaw only — the base has no roll or pitch to speak of."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class VWDriver:
    """Odometry-closed (v, w) driving. Owns no state the caller must reset."""

    def __init__(self, cfg, dry_run=False, vmax=None, wmax=None):
        robot = cfg.get('robot', {}) or {}
        topics = cfg.get('topics', {}) or {}

        self.max_linear = float(robot.get('max_linear_speed', 0.05))
        self.max_angular = float(robot.get('max_angular_speed', 0.25))
        if vmax is not None or wmax is not None:
            self.max_linear, self.max_angular = self._raise_limits(vmax, wmax)
        self.linear_accel = float(robot.get('linear_accel', 0.05))
        self.angular_accel = float(robot.get('angular_accel', 0.3))
        self.ramp_enabled = bool(robot.get('ramp_enabled', True))
        # robot.yaml's s_curve_* ratios and min factors are deliberately NOT
        # read — see the Motion profile section of the module docstring. The
        # trapezoid needs no phase ratios: min(top, sqrt(2*a*remaining))
        # produces the accel / cruise / decel split on its own, and produces a
        # triangular profile automatically when the move is too short to cruise.
        self.pivot_tol = math.radians(
            float(robot.get('pivot_threshold_deg', 1.0)))

        # ⚠️ No speed floor. A min_drive_speed clamp lived here for part of
        # 2026-08-14 and was removed with navigation's — nothing measured
        # backs a stiction limit on this drive, see robot.yaml. It was very
        # nearly inert anyway: the braking envelope only falls below 0.015 m/s
        # inside the last 2.25 mm (0.015**2 / 2a), which is under ARRIVE_TOL_M,
        # so the loop has already handed off to the ramp-out by then.

        self.dry_run = dry_run

        # Odom state, written from the subscriber thread.
        self.odom_x = None
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_stamp = None

        self.estop = None       # None = never heard from the safety node

        # Ramp state, mirroring MobileController.send_vel.
        self._cur_v = 0.0
        self._cur_w = 0.0
        self._last_vel_time = None

        cmd_topic = topics.get('cmd_vel', '/cmd_vel')
        odom_topic = topics.get('odom', '/odom')
        self.cmd_pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self._cb_odom, queue_size=1)
        rospy.Subscriber('/safety/estop', Bool, self._cb_estop, queue_size=1)

    def _raise_limits(self, vmax, wmax):
        """Apply --vmax / --wmax, bounded by the driver's own hard limits.

        The absolute ceiling is `base_controller`'s `max_linear_vel` /
        `max_angular_vel` on the param server (2.0 m/s / 20 rad/s), not a
        number invented here — past that the driver clamps anyway and the tool
        would be lying about what it commanded. Nothing below robot.yaml's
        0.05 / 0.25 needs this flag: the per-command speed argument already
        goes slower.
        """
        hard_v = float(rospy.get_param('/base_controller/max_linear_vel', 2.0))
        hard_w = float(rospy.get_param('/base_controller/max_angular_vel', 20.0))

        v = self.max_linear if vmax is None else min(abs(float(vmax)), hard_v)
        w = self.max_angular if wmax is None else min(abs(float(wmax)), hard_w)

        for label, new, cfg_val, hard in (('linear', v, self.max_linear, hard_v),
                                          ('angular', w, self.max_angular, hard_w)):
            if new > cfg_val:
                rospy.logwarn(
                    f"[vw] {label} limit raised {cfg_val} -> {new}, ABOVE "
                    f"robot.yaml. Navigation never commands this much; the "
                    f"driver's own ceiling is {hard}.")
        return v, w

    # ==========================================================
    # CALLBACKS
    # ==========================================================
    def _cb_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_x = p.x
        self.odom_y = p.y
        self.odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)
        # Wall clock, not the message stamp: staleness here means "the driver
        # stopped publishing", and a driver that freezes mid-message can leave
        # a stamp that never advances either.
        self.odom_stamp = rospy.Time.now()

    def _cb_estop(self, msg):
        self.estop = msg.data

    # ==========================================================
    # VELOCITY OUTPUT
    # ==========================================================
    def _publish(self, v, w):
        if self.dry_run:
            rospy.loginfo_throttle(0.5, f"[dry-run] v={v:+.3f} w={w:+.3f}")
            return
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.cmd_pub.publish(t)

    def send_vel(self, v, w):
        """Publish (v, w) through the configured acceleration limits."""
        v = max(-self.max_linear, min(self.max_linear, v))
        w = max(-self.max_angular, min(self.max_angular, w))

        if not self.ramp_enabled:
            self._cur_v, self._cur_w = v, w
            self._publish(v, w)
            return

        now = rospy.Time.now()
        if self._last_vel_time is None:
            dt = 1.0 / RATE_HZ
        else:
            dt = max(0.001, min((now - self._last_vel_time).to_sec(), 0.2))
        self._last_vel_time = now

        for attr, target, accel in (('_cur_v', v, self.linear_accel),
                                    ('_cur_w', w, self.angular_accel)):
            cur = getattr(self, attr)
            step = accel * dt
            diff = target - cur
            setattr(self, attr,
                    cur + math.copysign(step, diff) if abs(diff) > step
                    else target)

        self._publish(self._cur_v, self._cur_w)

    def stop(self):
        """Zero velocity and drop the ramp state."""
        self._cur_v = 0.0
        self._cur_w = 0.0
        self._last_vel_time = None
        # A single zero can race the last non-zero message; repeat past one
        # cmd_vel_timeout so the base ends up stopped either way.
        for _ in range(5):
            self._publish(0.0, 0.0)
            if self.dry_run:
                break
            rospy.sleep(0.05)

    # ==========================================================
    # GUARDS
    # ==========================================================
    def wait_for_odom(self, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.odom_x is None:
            if rospy.Time.now() > deadline:
                return False
            rate.sleep()
        return self.odom_x is not None

    def _blocked(self):
        """Reason the base must not be commanded right now, or None."""
        if self.estop:
            return ("/safety/estop is ACTIVE — the PILZ circuit has cut motor "
                    "power. Reset the safety circuit first.")
        if self.odom_stamp is None:
            return "no /odom received yet"
        age = (rospy.Time.now() - self.odom_stamp).to_sec()
        if age > ODOM_STALE_S:
            return f"/odom is stale by {age:.1f}s — is the navifra driver up?"
        return None

    # ==========================================================
    # MOTIONS
    # ==========================================================
    def drive_distance(self, distance, speed=None):
        """Drive `distance` m along body X. Negative = reverse.

        Closed on odom travel, trapezoidal profile — see the module docstring.
        """
        target = abs(float(distance))
        if target < ARRIVE_TOL_M:
            rospy.logwarn(f"distance below the {ARRIVE_TOL_M*1000:.0f} mm "
                          f"arrival tolerance, nothing to do")
            return True
        sign = 1.0 if distance > 0 else -1.0
        v_max = min(abs(speed) if speed else self.max_linear, self.max_linear)
        accel = self.linear_accel

        start = (self.odom_x, self.odom_y)
        timeout = rospy.Time.now() + rospy.Duration(
            self._timeout_for(target, v_max, accel))

        def travelled():
            return math.hypot(self.odom_x - start[0], self.odom_y - start[1])

        rospy.loginfo(f"[vw] {'forward' if sign > 0 else 'reverse'} "
                      f"{target:.3f} m at up to {v_max:.3f} m/s")

        ok = self._run_loop(
            progress=travelled,
            target=target,
            tol=ARRIVE_TOL_M,
            eps=STALL_EPS_M,
            timeout=timeout,
            command=lambda rem: (
                sign * self._envelope(rem, v_max, accel), 0.0),
            braking=lambda: self._cur_v ** 2 / (2.0 * accel),
            what='move')
        if not ok:
            return False
        short = target - travelled()
        if not self._arrived(short, target, 3 * ARRIVE_TOL_M, 'move',
                             f"{short*1000:.0f} mm"):
            return False
        rospy.loginfo(f"[vw] done: travelled {travelled():.3f} m "
                      f"(target {target:.3f} m)")
        return True

    def pivot_angle(self, degrees, speed=None):
        """Pivot in place by `degrees`. Positive = CCW (left)."""
        target = abs(math.radians(float(degrees)))
        if target < self.pivot_tol:
            rospy.logwarn("angle below the pivot tolerance, nothing to do")
            return True
        sign = 1.0 if degrees > 0 else -1.0
        w_max = min(abs(speed) if speed else self.max_angular, self.max_angular)
        accel = self.angular_accel

        start_yaw = self.odom_yaw
        turned = [0.0]      # accumulated, so a pivot past +-180 deg still works
        last_yaw = [start_yaw]
        timeout = rospy.Time.now() + rospy.Duration(
            self._timeout_for(target, w_max, accel))

        def progress():
            # Integrate wrapped deltas rather than comparing against start:
            # a single wrap would otherwise read as a jump backwards.
            turned[0] += abs(_wrap(self.odom_yaw - last_yaw[0]))
            last_yaw[0] = self.odom_yaw
            return turned[0]

        rospy.loginfo(f"[vw] pivot {'CCW' if sign > 0 else 'CW'} "
                      f"{math.degrees(target):.1f} deg at up to "
                      f"{w_max:.3f} rad/s")

        ok = self._run_loop(
            progress=progress,
            target=target,
            tol=self.pivot_tol,
            eps=STALL_EPS_RAD,
            timeout=timeout,
            command=lambda rem: (
                0.0, sign * self._envelope(rem, w_max, accel)),
            braking=lambda: self._cur_w ** 2 / (2.0 * accel),
            what='pivot')
        if not ok:
            return False
        # One more sample AFTER the ramp-out. Unlike travelled(), which reads
        # odom directly, this accumulator only advances when it is called — and
        # the ramp-out is real rotation the loop never sampled.
        short = target - progress()
        if not self._arrived(short, target, 3 * self.pivot_tol, 'pivot',
                             f"{math.degrees(short):.1f} deg"):
            return False
        rospy.loginfo(f"[vw] done: turned {math.degrees(turned[0]):.2f} deg "
                      f"(error {math.degrees(short):+.2f} deg)")
        return True

    def hold_vw(self, v, w, duration):
        """Publish a fixed (v, w) for `duration` seconds. Open loop.

        The one command here that does NOT close on odom — it is for answering
        "does the base respond to this twist at all", which is exactly the
        question you have when odom is the thing you suspect.
        """
        v = max(-self.max_linear, min(self.max_linear, float(v)))
        w = max(-self.max_angular, min(self.max_angular, float(w)))
        end = rospy.Time.now() + rospy.Duration(float(duration))
        rospy.loginfo(f"[vw] holding v={v:+.3f} w={w:+.3f} for {duration:.1f}s")

        start = (self.odom_x, self.odom_y, self.odom_yaw)
        rate = rospy.Rate(RATE_HZ)
        while not rospy.is_shutdown() and rospy.Time.now() < end:
            reason = self._blocked()
            if reason:
                rospy.logerr(f"[vw] aborted — {reason}")
                self.stop()
                return False
            self.send_vel(v, w)
            rate.sleep()
        self.stop()
        rospy.loginfo(
            f"[vw] moved {math.hypot(self.odom_x - start[0], self.odom_y - start[1]):.3f} m, "
            f"turned {math.degrees(_wrap(self.odom_yaw - start[2])):+.2f} deg")
        return True

    # ==========================================================
    # SHARED MOTION LOOP
    # ==========================================================
    @staticmethod
    def _envelope(remaining, top, accel):
        """Fastest speed from which a constant-`accel` ramp still stops on target.

        `sqrt(2*a*remaining)`, clipped to `top`. This is the braking curve, not
        a shape chosen for looks, and the identity that makes it the right one
        is that it is self-consistent: a speed on the envelope satisfies
        `v**2 / (2*a) == remaining` exactly, so at ANY point on it, handing off
        to a constant-`a` ramp-down covers precisely the distance that is left.
        That is what `_ramp_to_zero` relies on.

        Taking `min` with `top` produces the whole trapezoid — cruise where the
        curve is above `top`, brake where it is below — with no phase ratios to
        configure, and it degrades to a triangular profile on a move too short
        to reach `top` instead of needing a special case.

        """
        return min(top, math.sqrt(max(0.0, 2.0 * accel * remaining)))

    @staticmethod
    def _timeout_for(target, top, accel):
        """Generous bound on a trapezoid's duration. Bounds a hang, not a spec."""
        return 3.0 * (target / max(top, 1e-6) + top / max(accel, 1e-6)) + 5.0

    def _arrived(self, shortfall, target, floor_tol, what, pretty):
        """Did the base actually get there? Returns False (and shouts) if not.

        The stall check inside the loop is not sufficient on its own here. The
        braking hand-off triggers once the COMMANDED speed's stopping distance
        covers what is left, and on a base that is not moving the commanded
        speed keeps climbing until that is true of the whole move — which can
        happen sooner than STALL_GRACE_S on a short one. Caught in a live
        dry-run: `forward 0.05` reported success having travelled 0.000 m.
        """
        if self.dry_run:
            return True                # nothing is published, so nothing moves
        if shortfall <= max(ARRIVE_SHORTFALL_FRAC * target, floor_tol):
            return True
        rospy.logerr(f"[vw] {what} ended {pretty} short of the target — the "
                     f"base did not follow the command (check /motor/error, "
                     f"/motor/alarm, and whether /odom is advancing).")
        return False

    def _ramp_to_zero(self, what):
        """Bring the commanded velocity down to zero at the accel limit.

        NOT `stop()`. That assigns zero and publishes it — a step, and the step
        is exactly the jolt this whole profile exists to avoid. Because the
        loop leaves the envelope, `v**2 / (2*a)` equals the distance still to
        go, so ramping down here is both the smooth ending and the accurate
        one: the robot coasts out the last few millimetres instead of being
        cut off short of them.

        `stop()` stays the right call for an abort — an e-stop or a stalled
        drive is not an occasion for a graceful ramp.
        """
        rate = rospy.Rate(RATE_HZ)
        for _ in range(int(RATE_HZ * RAMP_OUT_MAX_S)):
            if rospy.is_shutdown() or self._blocked():
                break
            if (abs(self._cur_v) < RAMP_DONE_EPS
                    and abs(self._cur_w) < RAMP_DONE_EPS):
                break
            self.send_vel(0.0, 0.0)
            rate.sleep()
        self.stop()
        return True

    def _run_loop(self, progress, target, tol, eps, timeout, command, braking,
                  what):
        """Drive until it is time to brake, then ramp out at the accel limit.

        `braking()` returns the distance the CURRENTLY COMMANDED velocity needs
        in order to reach zero at the accel limit. Handing off exactly when
        that equals the remaining distance is what lands the move on target:
        testing the remaining distance alone would hand off at whatever speed
        the limiter happened to be at, which overshoots when it is lagging
        above the envelope, and stops short when below.
        """
        rate = rospy.Rate(RATE_HZ)
        stall_ref = progress()
        stall_since = rospy.Time.now()

        while not rospy.is_shutdown():
            reason = self._blocked()
            if reason:
                rospy.logerr(f"[vw] {what} aborted — {reason}")
                self.stop()
                return False

            done = progress()
            remaining = target - done
            # Leave when braking now would land on target, or on the tolerance
            # — never by waiting for the envelope to reach zero, which would
            # creep below the stiction threshold and trip the stall check.
            if remaining <= max(tol, braking()):
                break

            now = rospy.Time.now()
            if now > timeout:
                rospy.logerr(f"[vw] {what} timed out at {done:.3f} of "
                             f"{target:.3f}")
                self.stop()
                return False

            # Stall check. Commanding a base that is not moving is the 2026-08-12
            # failure mode; without this the loop would sit at low speed for the
            # whole timeout writing to a dead drive.
            if abs(done - stall_ref) > eps:
                stall_ref = done
                stall_since = now
            elif (now - stall_since).to_sec() > STALL_GRACE_S:
                rospy.logerr(
                    f"[vw] {what} aborted — /odom has not advanced in "
                    f"{STALL_GRACE_S:.1f}s while commanding motion. The drive "
                    f"is not moving (check /motor/error, /motor/alarm).")
                self.stop()
                return False

            v, w = command(remaining)
            self.send_vel(v, w)
            rate.sleep()

        return self._ramp_to_zero(what)

    # ==========================================================
    # STATUS
    # ==========================================================
    def report(self):
        age = ('never' if self.odom_stamp is None
               else f"{(rospy.Time.now() - self.odom_stamp).to_sec():.2f}s ago")
        estop = {None: 'unknown (no message yet)',
                 True: 'ACTIVE', False: 'clear'}[self.estop]
        rospy.loginfo(
            f"[vw] odom x={self.odom_x} y={self.odom_y} "
            f"yaw={math.degrees(self.odom_yaw):+.2f} deg, updated {age}; "
            f"estop {estop}; limits v<={self.max_linear} m/s "
            f"w<={self.max_angular} rad/s")


# ==============================================================
# COMMAND PARSING
# ==============================================================
HELP = """
  forward <m> [v]      drive ahead, closed on odom      e.g. forward 0.3
  back <m> [v]         drive in reverse                 e.g. back 0.3
  left <deg> [w]       pivot CCW, closed on odom        e.g. left 90
  right <deg> [w]      pivot CW                         e.g. right 90
  pivot <deg> [w]      pivot, + = CCW                   e.g. pivot -45
  vw <v> <w> <sec>     raw twist, open loop             e.g. vw 0.03 0.1 2
  stop                 zero velocity now
  status               odom / estop / limits
  help, quit

  The trailing [v] / [w] goes SLOWER than the ceiling; to go faster, restart
  with --vmax / --wmax. 'status' prints the ceiling currently in force.
"""


def run_command(drv, line):
    """Execute one command line. Returns False to end the session."""
    parts = line.replace(',', ' ').split()
    if not parts:
        return True
    cmd, args = parts[0].lower(), parts[1:]

    try:
        if cmd in ('q', 'quit', 'exit'):
            return False
        if cmd in ('h', 'help', '?'):
            print(HELP)
        elif cmd == 'stop':
            drv.stop()
            rospy.loginfo("[vw] stopped")
        elif cmd in ('status', 'odom'):
            drv.report()
        elif cmd in ('forward', 'f', 'back', 'b'):
            sign = -1.0 if cmd in ('back', 'b') else 1.0
            drv.drive_distance(sign * float(args[0]),
                               float(args[1]) if len(args) > 1 else None)
        elif cmd in ('left', 'l', 'right', 'r', 'pivot', 'p'):
            deg = float(args[0])
            if cmd in ('left', 'l'):
                deg = abs(deg)
            elif cmd in ('right', 'r'):
                deg = -abs(deg)
            drv.pivot_angle(deg, float(args[1]) if len(args) > 1 else None)
        elif cmd == 'vw':
            drv.hold_vw(float(args[0]), float(args[1]), float(args[2]))
        else:
            rospy.logwarn(f"unknown command: {cmd!r} — try 'help'")
    except (IndexError, ValueError):
        rospy.logerr(f"bad arguments for {cmd!r} — try 'help'")
    return True


def stack_is_up():
    """True if mobile_node is registered — i.e. something else owns /cmd_vel."""
    try:
        import rosnode
        return any(n.endswith('mobile_node') for n in rosnode.get_node_names())
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser(
        description='Manual (v, w) drive tool — bring-up and debugging only.')
    ap.add_argument('command', nargs='*',
                    help='one command to run, then exit (e.g. forward 0.3)')
    ap.add_argument('--exec', dest='script', default=None,
                    help='semicolon-separated commands to run, then exit')
    ap.add_argument('--dry-run', action='store_true',
                    help='print velocities instead of publishing them')
    ap.add_argument('--force', action='store_true',
                    help='run even though mobile_node is registered (DANGEROUS '
                         '— two publishers fight over /cmd_vel)')
    ap.add_argument('--vmax', type=float, default=None,
                    help='override the linear speed ceiling [m/s] (robot.yaml '
                         'says 0.05). Only needed to go FASTER — the per-command '
                         'speed argument already goes slower.')
    ap.add_argument('--wmax', type=float, default=None,
                    help='override the angular speed ceiling [rad/s] '
                         '(robot.yaml says 0.25)')
    args = ap.parse_args()

    rospy.init_node('vw_drive', anonymous=True)

    if stack_is_up() and not (args.force or args.dry_run):
        rospy.logfatal(
            "mobile_node is running — it is the sole /cmd_vel publisher and "
            "this tool would fight it at 50 Hz with no error anywhere. Shut "
            "the stack down, or use GOTO / /mobile/goto_tag instead. --force "
            "overrides.")
        return 1

    drv = VWDriver(utils.load_config(CONFIG_PATH), dry_run=args.dry_run,
                   vmax=args.vmax, wmax=args.wmax)
    rospy.on_shutdown(drv.stop)

    if not drv.wait_for_odom():
        rospy.logfatal("no /odom within 5s — the navifra driver is not "
                       "publishing. Nothing was commanded.")
        return 1
    drv.report()

    if args.script or args.command:
        source = args.script or ' '.join(args.command)
        for line in source.split(';'):
            if not run_command(drv, line.strip()):
                break
        drv.stop()
        return 0

    print(HELP)
    while not rospy.is_shutdown():
        try:
            line = input('vw> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not run_command(drv, line):
            break
    drv.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
