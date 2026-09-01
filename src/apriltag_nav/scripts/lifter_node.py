#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Base lifter node — sole owner of the manipulator base lift.

The Navifra base carries a vertical lift under the arm (MDROBOT DC drive over
RS485, driven by the `lift_driver` package in ~/navifra). Stroke is 0 to 6900
hole counts = 343.3 mm of arm-base extension. This node is the ONLY thing in
apriltag_nav that commands it; everything else reads its state over topics.
Same one-owner-per-device split as basler_camera_node / arm_node.

It commands through `NavifraDevices`, not raw `/lift/*` publishers — that
wrapper stays the single point of contact with the Navifra driver (CLAUDE.md).
Read-only use of `NavifraDevices.lift_position` / `lift_at()` elsewhere (e.g.
task_executor's scan-height guard) is fine and unaffected; only *commands*
are owned here.

Terminology — "homing" means two unrelated things in this workspace
-------------------------------------------------------------------
Never write plain "homing" in lift code or lift docs. Always qualify it:

  **LIFT ORIGIN HOMING** (this node, `/lifter/home`, driver `/lift/home`)
      The lift descends to the physical LOWER LIMIT SWITCH and the encoder
      origin is reset to count 0. It is about re-establishing a measurement
      reference, and it always moves DOWN. Korean: 리프트 원점복귀.

  **ARM HOME POSE** (arm_node, `/arm/move_home`,
      `ArmController.move_to_home()`)
      The Fairino arm does a MoveJ to a stored home JOINT configuration.
      Nothing is zeroed, no reference is established, and it has nothing to do
      with the lift. Korean: 매니퓰레이터 홈 자세.

They are different devices, different nodes, different services, and one of
them moves the base under the other. `~auto_home_on_start` on this node means
lift origin homing only; it never touches the arm.

Why the lift needs a node of its own
------------------------------------
Four hardware facts make raw `rostopic pub` to `/lift/*` genuinely unsafe,
and all four are handled here rather than being left to every caller
(driver guide 3.5 / 6, measured 2026-07-28):

1. **The upper end has no limit switch.** Only the lower one is wired. Driving
   up runs the mechanism into a hard stop, which the drive sees as a stall;
   `auto_release_on_stop` then drops the velocity command to dodge a
   CTRL_FAIL alarm. So every upward move here is clamped to
   `soft_max_counts` — 6900, the top of travel as re-measured 2026-08-14 —
   and open-ended jogs are bounded by both position and a timeout.
2. **The count is incremental and drifts.** It is lost on power cycle, and a
   descent started while pressed against the upper stop counts *backwards*
   for the first few seconds (about +500), accumulating 1000-1800 counts of
   error per full up-down cycle. Lift origin homing (to the lower limit
   switch, which is always accurate) is the only way to restore the origin.
3. **Absolute position commands are silently ignored before origin homing.**
   Use the relative jog instead — that is what `/lifter/jog_cmd` is for.
4. **The drive has mechanical backlash.** A given count therefore corresponds
   to two different physical heights depending on which way the lift last
   moved. See below.

Backlash, and why there is no origin pass
-----------------------------------------
Because of point 4, a count read after an ascent and the same count read after
a descent are not the same physical height. The node used to defend against
that by routing every descending absolute move through the origin, so the last
motion before stopping was always an ascent. **That was removed on 2026-08-10**
— it cost up to 56 s per move and defended a case the task flow cannot produce.

The reason it cannot: every task ends with lift origin homing, and nothing
homes or lowers the lift mid-task, so an absolute move only ever starts from
the origin and only ever goes up. A descent is therefore not an expected
request. If one is made anyway it is now **executed as asked** — the drive goes
straight down and stops there, loaded in the descending direction.

⚠️ So a position reached by descending is loaded the opposite way to one
reached by climbing, and the same is true of `jog` / `up` / `down`, which have
no target at all. Do not use any of them to reach a height that will be
scanned at; run `/lifter/home` and climb to it instead.

An absolute move while the origin is unknown is **refused**, not silently
homed: point 3 means the driver would ignore it, so reporting success would be
a lie. Use `/lifter/home` first, or `/lifter/jog_cmd` for a relative
move that works un-homed.

Startup does not move the lift, for the same reason arm_node does
not move the arm to its home pose: bringing the stack up is not a request to
move hardware, and the arm may be inside a fixture. `~auto_home_on_start`
exists but defaults false. NOTE the *driver's* own `auto_home_on_start` is set
**true** in ~/navifra/param.yaml on this robot, so the lift may already have
origin-homed itself on driver start, before this node ever ran — this node
reports that state rather than repeating it.

Interface
---------
Services (std_srvs, no custom messages):
  /lifter/home              Trigger  LIFT ORIGIN HOMING — descend to the
                                          lower limit switch, zero the count
                                          (blocking; this is NOT the arm's
                                          home pose, that is /arm/move_home)
  /lifter/stop              Trigger  stop now; also cancels a blocked move
  /lifter/reset             Trigger  manual alarm reset
  /lifter/goto_scan_height  Trigger  go to navifra.scan_height_counts
Topics in:
  /lifter/command      std_msgs/String   up | down | stop | reset |
                                              home  (= lift origin homing) |
                                              scan_height | goto <counts> |
                                              jog <counts> | mm <millimetres>
  /lifter/position_cmd std_msgs/Int32    absolute target [counts]
  /lifter/height_cmd   std_msgs/Float32  absolute target [mm above origin]
  /lifter/jog_cmd      std_msgs/Int32    relative offset [counts]
Topics out:
  /lifter/state    std_msgs/String   JSON, everything below at once
  /lifter/height   std_msgs/Float32  height above the origin [mm] (see
                                          the mm_per_count warning below)
  /lifter/ready    std_msgs/Bool     origin-homed, no alarm, no e-stop,
                                          idle
  /lifter/busy     std_msgs/Bool     a motion is in progress here

ROS parameters (~, defaults from robot.yaml `lifter:`):
  soft_min_counts / soft_max_counts  travel clamp [counts]
  mm_per_count                       count -> mm scale
  mm_calibrated                      set true only after measuring the scale
  auto_home_on_start                 lift origin homing once at node start
                                     (default false)
  jog_timeout_s                      bound on an open-ended up/down jog
  publish_rate_hz                    state publish rate

`mm_per_count` is 343.2 mm / 6897 counts = 0.04976077, re-measured 2026-08-14
at the top of travel: commanded to 6900 the drive settled at count 6897, and
the gauge read 343.2 mm of extension there. The arm base therefore sits 652 mm
above the ground at the origin and ~995 mm at the top of the stroke.
`mm_calibrated` is true and the old startup warning about an unmeasured scale
no longer applies. (It was 341 mm / 7000 = 0.0487143 until 2026-08-14 — 7.2 mm
low over the stroke.)
"""

import json
import threading

import rospy
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger, TriggerResponse

from apriltag_nav.navifra_devices import NavifraDevices
from apriltag_nav.paths import load_yaml_block as _load_yaml_block

# The count -> mm scale comes from ONE measured pair, taken at the top of
# travel on 2026-08-14: commanded to 6900 the drive settled at 6897 (inside the
# 20-count settle tolerance) and the gauge read 343.2 mm of arm-base extension.
# The scale is derived from the count actually reached, not the commanded one.
CALIB_COUNTS = 6897
CALIB_MM = 343.2
MM_PER_COUNT = CALIB_MM / CALIB_COUNTS          # 0.04976077

# Usable top of travel, and the ceiling the soft clamp is checked against.
# Was 7000 paired with 341 mm (2026-08-13) — that scale ran 7.2 mm low over the
# stroke, and there is still no upper limit switch to stop against.
STROKE_COUNTS = 6900
STROKE_MM = STROKE_COUNTS * MM_PER_COUNT        # 343.35

DEFAULTS = {
    'soft_min_counts':     0,
    'soft_max_counts':     STROKE_COUNTS,
    'mm_per_count':        MM_PER_COUNT,
    'mm_calibrated':       True,
    'auto_home_on_start':  False,
    'jog_timeout_s':       35.0,
    'publish_rate_hz':     2.0,
}


class BaseLifterNode:
    def __init__(self):
        rospy.init_node('lifter_node', anonymous=False)

        cfg = dict(DEFAULTS)
        cfg.update(_load_yaml_block('lifter') or {})
        # ~params win over robot.yaml, as everywhere else in this package.
        self.soft_min = int(rospy.get_param('~soft_min_counts',
                                            cfg['soft_min_counts']))
        self.soft_max = int(rospy.get_param('~soft_max_counts',
                                            cfg['soft_max_counts']))
        self.mm_per_count = float(rospy.get_param('~mm_per_count',
                                                  cfg['mm_per_count']))
        self.mm_calibrated = bool(rospy.get_param('~mm_calibrated',
                                                  cfg['mm_calibrated']))
        self.jog_timeout_s = float(rospy.get_param('~jog_timeout_s',
                                                   cfg['jog_timeout_s']))
        rate_hz = float(rospy.get_param('~publish_rate_hz',
                                        cfg['publish_rate_hz']))

        if self.soft_max > STROKE_COUNTS:
            rospy.logwarn(f"[BaseLifter] soft_max_counts {self.soft_max} is "
                          f"above the measured stroke {STROKE_COUNTS} — the "
                          "lift will stall against the unswitched upper stop")
        if self.soft_min >= self.soft_max:
            raise ValueError(f"soft_min_counts ({self.soft_min}) must be below "
                             f"soft_max_counts ({self.soft_max})")

        navifra_cfg = _load_yaml_block('navifra')
        self.devices = NavifraDevices(navifra_cfg, on_estop=self._on_estop)

        # The lift position the arm transform was calibrated at, if known.
        # Read-only here: this node warns when it moves the lift away from it,
        # task_executor owns the actual pre-scan guard.
        sh = navifra_cfg.get('scan_height_counts', None)
        self.scan_height = None if sh is None else int(sh)

        # One motion at a time. Held for the whole blocking move, so a second
        # request is rejected rather than queued — two callers fighting over
        # one axis is worse than a clear "busy".
        self._motion_lock = threading.Lock()
        self._busy_what = None

        self.pub_state  = rospy.Publisher('/lifter/state', String,
                                          queue_size=1, latch=True)
        self.pub_height = rospy.Publisher('/lifter/height', Float32,
                                          queue_size=1, latch=True)
        self.pub_ready  = rospy.Publisher('/lifter/ready', Bool,
                                          queue_size=1, latch=True)
        self.pub_busy   = rospy.Publisher('/lifter/busy', Bool,
                                          queue_size=1, latch=True)

        rospy.Service('/lifter/home', Trigger, self._srv_home)
        rospy.Service('/lifter/stop', Trigger, self._srv_stop)
        rospy.Service('/lifter/reset', Trigger, self._srv_reset)
        rospy.Service('/lifter/goto_scan_height', Trigger,
                      self._srv_scan_height)

        rospy.Subscriber('/lifter/command', String, self._cb_command,
                         queue_size=5)
        rospy.Subscriber('/lifter/position_cmd', Int32,
                         self._cb_position_cmd, queue_size=1)
        rospy.Subscriber('/lifter/height_cmd', Float32,
                         self._cb_height_cmd, queue_size=1)
        rospy.Subscriber('/lifter/jog_cmd', Int32, self._cb_jog_cmd,
                         queue_size=1)

        rospy.on_shutdown(self._on_shutdown)

        if not self.mm_calibrated:
            rospy.logwarn(
                "[BaseLifter] mm_per_count=%.6f = %.0f mm / %d counts, but "
                "mm_calibrated is false — the millimetre side has not been "
                "put against a height gauge, so /lifter/height is indicative "
                "only. See docs/lift_arm_base_z_analysis.md section 4.2."
                % (self.mm_per_count, STROKE_MM, STROKE_COUNTS))

        rospy.loginfo(f"[BaseLifter] Ready — travel clamped to "
                      f"[{self.soft_min}, {self.soft_max}] counts "
                      f"(stroke {STROKE_COUNTS}); scan_height="
                      f"{self.scan_height}")

        if bool(rospy.get_param('~auto_home_on_start',
                                cfg['auto_home_on_start'])):
            rospy.logwarn("[BaseLifter] ~auto_home_on_start is true — LIFT "
                          "ORIGIN HOMING now. The lift WILL descend to the "
                          "lower limit switch. (This is the lift, not the "
                          "arm's home pose.)")
            threading.Thread(target=self._do_home, daemon=True).start()
        else:
            rospy.loginfo("[BaseLifter] No lift origin homing at start — "
                          "absolute moves are refused until "
                          "/lifter/home succeeds or the driver reports "
                          "/lift/homed. Relative jogs work either way.")

        self._timer = rospy.Timer(rospy.Duration(1.0 / max(rate_hz, 0.1)),
                                  self._publish_state)

    # ==========================================================
    # HELPERS
    # ==========================================================
    def _on_estop(self):
        """Fires in the subscriber thread on every false->true e-stop edge.

        The hardware PILZ circuit already cut motor power; this only makes the
        software side agree, so a blocked move returns instead of waiting out
        its timeout and the drive is not left holding a velocity command.
        """
        rospy.logerr("[BaseLifter] E-stop — cancelling lift motion")
        try:
            self.devices.lift_stop()
        except Exception as e:
            rospy.logerr(f"[BaseLifter] stop-on-estop failed: {e}")

    def counts_to_mm(self, counts):
        return float(counts) * self.mm_per_count

    def mm_to_counts(self, mm):
        return int(round(float(mm) / self.mm_per_count))

    def _clamp(self, counts):
        """Clamp to the soft travel range. Returns (clamped, was_clamped)."""
        c = int(counts)
        cl = max(self.soft_min, min(self.soft_max, c))
        return cl, (cl != c)

    def _warn_leaving_scan_height(self, target):
        """Pose-mode IK assumes a constant arm_base_z. Say so out loud."""
        if self.scan_height is None:
            return
        if abs(int(target) - self.scan_height) <= self.devices.lift_settle_tol:
            return
        rospy.logwarn(
            f"[BaseLifter] Moving to {target} counts, away from the "
            f"calibrated scan height {self.scan_height}. Pose-mode scans "
            "(scan_grid_*, scan_full_pose, TEST_POSE) will be offset in Z — "
            "arm_base_z is a constant and does not track the lift.")

    def _begin(self, what):
        """Take the motion lock. Returns True if this caller now owns it."""
        if not self._motion_lock.acquire(blocking=False):
            return False
        self._busy_what = what
        self._publish_busy()
        return True

    def _end(self):
        self._busy_what = None
        self._motion_lock.release()
        self._publish_busy()

    def _busy_reply(self):
        return False, f"busy: {self._busy_what or 'motion in progress'}"

    # ==========================================================
    # MOTIONS — each returns (ok, message) and holds the motion lock
    # ==========================================================
    def _home_locked(self):
        """Lift origin homing. Caller must already hold the motion lock."""
        rospy.loginfo("[BaseLifter] Lift origin homing — descending to the "
                      "lower limit switch (this is the LIFT, not the arm)")
        return self.devices.lift_home()

    def _do_home(self):
        if not self._begin('lift origin homing'):
            return self._busy_reply()
        try:
            return self._home_locked()
        finally:
            self._end()

    def _do_goto(self, counts):
        """Absolute move. Refused before origin homing; see the backlash note.

        A descending target is carried out as asked rather than routed via the
        origin — the task flow never produces one, and paying up to 56 s to
        defend against a case that cannot happen was not worth the complexity.
        """
        target, clamped = self._clamp(counts)
        if clamped:
            rospy.logwarn(f"[BaseLifter] Target {int(counts)} clamped to "
                          f"{target} counts (soft travel limit)")

        if not self.devices.lift_homed:
            return False, ("lift not origin-homed — absolute position is "
                           "meaningless and the driver ignores it. Call "
                           "/lifter/home, or use /lifter/jog_cmd "
                           "for a relative move.")

        pos = self.devices.lift_position
        if pos is None:
            return False, "lift position unknown (lift_driver running?)"

        if not self._begin(f'goto {target}'):
            return self._busy_reply()
        try:
            if abs(pos - target) <= self.devices.lift_settle_tol:
                return True, f"already at {pos} counts — no motion"

            if target < pos:
                # Not expected from a task; see the module docstring.
                rospy.logwarn(
                    f"[BaseLifter] goto {target} descends from {pos}. The "
                    "lift will stop loaded in the descending direction, so "
                    "this count is not the same physical height it would be "
                    "after a climb. Do not scan here without homing first.")

            self._warn_leaving_scan_height(target)
            return self.devices.lift_goto(target)
        finally:
            self._end()

    def _do_jog(self, delta):
        """Relative move. Allowed before origin homing — that is the point.

        There is no absolute target here, so the drive can be left loaded
        either way. Do not jog to a height that will be scanned at.
        """
        pos = self.devices.lift_position
        if pos is None:
            return False, "lift position unknown (lift_driver running?)"

        target, clamped = self._clamp(pos + int(delta))
        applied = target - pos
        if clamped:
            rospy.logwarn(f"[BaseLifter] Jog {int(delta):+d} clamped to "
                          f"{applied:+d} counts (soft travel limit)")
        if applied == 0:
            return True, f"already at the travel limit ({pos} counts)"
        if not self._begin(f'jog {applied:+d}'):
            return self._busy_reply()
        try:
            if not self.devices.lift_homed:
                rospy.logwarn("[BaseLifter] Jogging before lift origin homing "
                              "— the count is not a repeatable reference yet")
            self._warn_leaving_scan_height(target)
            return self.devices.lift_jog(applied)
        finally:
            self._end()

    def _do_manual(self, direction):
        """Bounded up/down jog.

        `up`/`down` on the driver run until stopped. Up especially must never
        be left running: there is no upper limit switch, so it would drive into
        the hard stop. This bounds it by the soft travel limit and by
        jog_timeout_s, and always sends a stop on the way out.

        Like jog, this has no absolute target, so a `down` in particular
        leaves the drive train loaded the opposite way to a climb.
        """
        pos = self.devices.lift_position
        if pos is None:
            return False, "lift position unknown (lift_driver running?)"
        bound = self.soft_max if direction == 'up' else self.soft_min

        ok, why = self.devices.safe_to_move()
        if not ok:
            return False, f"{direction} refused: {why}"

        if not self._begin(direction):
            return self._busy_reply()
        try:
            if direction == 'up':
                if pos >= self.soft_max:
                    return True, f"already at the upper soft limit ({pos})"
                self.devices.lift_up()
            else:
                if pos <= self.soft_min:
                    return True, f"already at the lower soft limit ({pos})"
                self.devices.lift_down()

            deadline = rospy.get_time() + self.jog_timeout_s
            rate = rospy.Rate(10)
            while not rospy.is_shutdown():
                p = self.devices.lift_position
                if self.devices.estop_active:
                    return False, "e-stop during manual jog"
                if self.devices.lift_error:
                    return False, f"lift alarm: {self.devices.lift_status}"
                if p is not None and ((direction == 'up' and p >= bound) or
                                      (direction == 'down' and p <= bound)):
                    return True, f"reached the soft limit at {p} counts"
                if rospy.get_time() >= deadline:
                    return True, (f"jog timeout after {self.jog_timeout_s:.0f}s"
                                  f" — stopped at {p}")
                rate.sleep()
            return False, "shutdown during manual jog"
        finally:
            # Unconditional: a return on any path above must leave the drive
            # stopped, never coasting toward the unswitched upper stop.
            try:
                self.devices.lift_stop()
            except Exception:
                pass
            self._end()

    def _do_scan_height(self):
        """Go to the lift height the arm transform was calibrated at.

        Just an absolute move, so _do_goto's origin pass already guarantees it
        is reached on an ascent — which is exactly what a scan height needs,
        since arm_base_z is a constant fitted at one physical height.
        """
        if self.scan_height is None:
            return False, ("navifra.scan_height_counts is null in robot.yaml — "
                           "no calibrated scan height to go to. See "
                           "docs/lift_arm_base_z_analysis.md.")
        return self._do_goto(self.scan_height)

    # ==========================================================
    # SERVICES
    # ==========================================================
    def _reply(self, result):
        ok, msg = result
        (rospy.loginfo if ok else rospy.logwarn)(f"[BaseLifter] {msg}")
        return TriggerResponse(success=bool(ok), message=str(msg))

    def _srv_home(self, _req):
        return self._reply(self._do_home())

    def _srv_stop(self, _req):
        # Deliberately does NOT take the motion lock: stop must work while a
        # move holds it. devices.lift_stop() also trips the cancel flag the
        # blocking waiter polls, so the mover returns immediately.
        try:
            self.devices.lift_stop()
            return self._reply((True, "stop sent"))
        except Exception as e:
            return self._reply((False, f"stop failed: {e}"))

    def _srv_reset(self, _req):
        try:
            self.devices.lift_reset_alarm()
            return self._reply((True, "alarm reset sent (/lift/reset)"))
        except Exception as e:
            return self._reply((False, f"reset failed: {e}"))

    def _srv_scan_height(self, _req):
        return self._reply(self._do_scan_height())

    # ==========================================================
    # TOPIC COMMANDS
    # ==========================================================
    def _run_async(self, fn, *args):
        """Topic commands must not block the subscriber thread for a minute."""
        threading.Thread(target=lambda: self._reply(fn(*args)),
                         daemon=True).start()

    def _cb_command(self, msg):
        parts = (msg.data or '').strip().split()
        if not parts:
            return
        verb = parts[0].lower()
        arg = parts[1] if len(parts) > 1 else None

        try:
            if verb == 'stop':
                self.devices.lift_stop()
                rospy.loginfo("[BaseLifter] stop sent")
            elif verb in ('up', 'down'):
                self._run_async(self._do_manual, verb)
            elif verb == 'home':
                self._run_async(self._do_home)
            elif verb == 'reset':
                self.devices.lift_reset_alarm()
                rospy.loginfo("[BaseLifter] alarm reset sent")
            elif verb == 'scan_height':
                self._run_async(self._do_scan_height)
            elif verb == 'goto' and arg is not None:
                self._run_async(self._do_goto, int(float(arg)))
            elif verb == 'jog' and arg is not None:
                self._run_async(self._do_jog, int(float(arg)))
            elif verb == 'mm' and arg is not None:
                self._run_async(self._do_goto, self.mm_to_counts(float(arg)))
            else:
                rospy.logwarn(f"[BaseLifter] Unknown command '{msg.data}'. "
                              "Use: up|down|stop|home|reset|scan_height|"
                              "goto <counts>|jog <counts>|mm <millimetres>")
        except ValueError:
            rospy.logwarn(f"[BaseLifter] Bad numeric argument in "
                          f"'{msg.data}'")

    def _cb_position_cmd(self, msg):
        self._run_async(self._do_goto, int(msg.data))

    def _cb_height_cmd(self, msg):
        self._run_async(self._do_goto, self.mm_to_counts(float(msg.data)))

    def _cb_jog_cmd(self, msg):
        self._run_async(self._do_jog, int(msg.data))

    # ==========================================================
    # STATE PUBLISHING
    # ==========================================================
    def _publish_busy(self):
        try:
            self.pub_busy.publish(Bool(self._busy_what is not None))
        except Exception:
            pass

    def _publish_state(self, _event=None):
        d = self.devices
        pos = d.lift_position
        state = {
            'position':      pos,
            'height_mm':     None if pos is None else round(self.counts_to_mm(pos), 1),
            'mm_calibrated': self.mm_calibrated,
            'homed':         d.lift_homed,
            'error':         d.lift_error,
            'alarm':         d.lift_status,
            'busy':          self._busy_what,
            'estop':         d.estop_active,
            'soft_min':      self.soft_min,
            'soft_max':      self.soft_max,
            'scan_height':   self.scan_height,
        }
        try:
            self.pub_state.publish(String(json.dumps(state)))
            if pos is not None:
                self.pub_height.publish(Float32(self.counts_to_mm(pos)))
            self.pub_ready.publish(Bool(
                bool(d.lift_homed) and not d.lift_error
                and not d.estop_active and self._busy_what is None))
        except Exception:
            pass

    # ==========================================================
    # SHUTDOWN
    # ==========================================================
    def _on_shutdown(self):
        """Never leave the drive moving. Do NOT origin-home or park on the way
        out — the arm may be extended, and shutdown is not a licence to move
        the base underneath it."""
        try:
            self.devices.lift_stop()
        except Exception:
            pass


if __name__ == '__main__':
    node = BaseLifterNode()
    rospy.spin()
