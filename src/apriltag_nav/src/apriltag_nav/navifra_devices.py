#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NavifraDevices — peripheral interface to the Navifra KU Polishing Robot Driver.

Target driver: KU Polishing Robot Driver v0.16 (ROS1 Noetic), packages
motor_driver / bms_driver / lift_driver / crevis_io_driver / safety_io_driver.

This wraps the driver's peripheral topics so the rest of apriltag_nav never
talks to raw driver topics. Drive control (/cmd_vel, /odom) is deliberately
NOT here — mobile_node owns that through MobileController.

Covered subsystems:
  safety   : /safety/estop, /safety/connected        (read-only, hardware PLC)
  lighting : /crevis/led/*, /crevis/led_state_all    (VISION lamp + status RGB)
  battery  : /bms/state, /bms/soc
  lift     : /lift/command|home|position_cmd|reset, /lift/position|status|...
  charging : /crevis/charging, /crevis/charge_port_on

Safety note — the driver's /safety/estop is FAIL-SAFE: it publishes true at
node start and whenever Safety PLC comms drop. Emergency stop itself is a
hardware circuit (PILZ PNOZmulti 2) that cuts motor power independently of
ROS; this topic is state feedback only and must never be treated as the
stopping mechanism. The software /estop topic was removed in driver v0.11.
"""

import time

import rospy
from std_msgs.msg import Bool, String, Int32, Int16, Float32
from sensor_msgs.msg import BatteryState


# STATUS LED is one RGB lamp driven by three independent Bool channels.
# Channels not part of a colour MUST be driven false or the colour mixes
# with whatever was left on (driver guide 3.4).
STATUS_COLORS = {
    'off':     (False, False, False),
    'red':     (True,  False, False),
    'green':   (False, True,  False),
    'blue':    (False, False, True),
    'yellow':  (True,  True,  False),
    'cyan':    (False, True,  True),
    'magenta': (True,  False, True),
    'white':   (True,  True,  True),
}

_DEFAULT_TOPICS = {
    'estop':            '/safety/estop',
    'safety_connected': '/safety/connected',
    'led_vision':       '/crevis/led/vision',
    'led_front':        '/crevis/led/front',
    'led_side':         '/crevis/led/side',
    'led_status_red':   '/crevis/led/status_red',
    'led_status_green': '/crevis/led/status_green',
    'led_status_blue':  '/crevis/led/status_blue',
    'led_state_all':    '/crevis/led_state_all',
    'crevis_connected': '/crevis/connected',
    'charging':         '/crevis/charging',
    'charge_port_on':   '/crevis/charge_port_on',
    'bms_state':        '/bms/state',
    'bms_soc':          '/bms/soc',
    'lift_command':     '/lift/command',
    'lift_home':        '/lift/home',
    'lift_position_cmd': '/lift/position_cmd',
    'lift_inc_position_cmd': '/lift/inc_position_cmd',
    'lift_velocity_cmd': '/lift/velocity_cmd',
    'lift_reset':       '/lift/reset',
    'lift_position':    '/lift/position',
    'lift_status':      '/lift/status',
    'lift_homed':       '/lift/homed',
    'lift_error':       '/lift/error',
    'lift_alarm':       '/lift/alarm',
}


class NavifraDevices:
    """Peripheral interface to the Navifra driver. Construct after init_node()."""

    def __init__(self, config=None, on_estop=None):
        """
        on_estop: optional callable invoked from the ROS subscriber thread on
                  every false->true e-stop transition. Use this rather than
                  polling — callers blocked in a motion/scan wait would not
                  otherwise notice the e-stop until the blocking call returns.
        """
        cfg = (config or {})
        self.cfg = cfg
        self._on_estop = on_estop

        topics = dict(_DEFAULT_TOPICS)
        topics.update(cfg.get('topics', {}) or {})
        self.topics = topics

        # ---------- Policy ----------
        # require_safety_link: when true, a missing/stale /safety/estop counts as
        # unsafe. Default false so the stack still runs if safety_io_driver was
        # not launched — but then safety_link_ok() is false and callers warn.
        self.require_safety_link = bool(cfg.get('require_safety_link', False))
        self.safety_timeout_s    = float(cfg.get('safety_timeout_s', 3.0))
        self.low_battery_pct     = float(cfg.get('low_battery_pct', 20.0))
        self.lift_settle_tol     = int(cfg.get('lift_settle_tol', 20))

        # ---------- Safety state ----------
        self._estop = None                 # None = never received
        self._estop_stamp = 0.0
        self._safety_connected = None
        self._estop_edge_seen = False      # latches a false->true transition

        # ---------- Battery state ----------
        self._battery = None
        self._battery_stamp = 0.0
        self._soc = None

        # ---------- Lift state ----------
        self._lift_position = None
        self._lift_status = None
        self._lift_homed = None
        self._lift_error = None
        self._lift_alarm = None
        # Set by lift_stop() so a blocking lift_home()/lift_goto() running in
        # another thread returns at once instead of sitting out its timeout.
        # Without this a stop request looks like it did nothing: the drive does
        # halt, but the waiter keeps polling until lift_move_timeout_s expires.
        self._lift_cancel = False

        # ---------- Lighting / charging state ----------
        self._led_state_all = None
        self._crevis_connected = None
        self._charge_port_on = None

        # ---------- Publishers ----------
        def pub(key, msg_type, latch=False):
            return rospy.Publisher(topics[key], msg_type, queue_size=1, latch=latch)

        self._pub_led = {
            'vision':       pub('led_vision', Bool, latch=True),
            'front':        pub('led_front', Bool, latch=True),
            'side':         pub('led_side', Bool, latch=True),
            'status_red':   pub('led_status_red', Bool, latch=True),
            'status_green': pub('led_status_green', Bool, latch=True),
            'status_blue':  pub('led_status_blue', Bool, latch=True),
        }
        self._pub_charging = pub('charging', Bool)
        self._pub_lift_command      = pub('lift_command', String)
        self._pub_lift_home         = pub('lift_home', Bool)
        self._pub_lift_position_cmd = pub('lift_position_cmd', Int32)
        self._pub_lift_inc_cmd      = pub('lift_inc_position_cmd', Int32)
        self._pub_lift_velocity_cmd = pub('lift_velocity_cmd', Int16)
        self._pub_lift_reset        = pub('lift_reset', Bool)

        # ---------- Subscribers ----------
        def sub(key, msg_type, cb):
            return rospy.Subscriber(topics[key], msg_type, cb, queue_size=1)

        sub('estop', Bool, self._cb_estop)
        sub('safety_connected', Bool, self._cb_safety_connected)
        sub('bms_state', BatteryState, self._cb_bms)
        sub('bms_soc', Float32, self._cb_soc)
        sub('lift_position', Int32, self._cb_lift_position)
        sub('lift_status', String, self._cb_lift_status)
        sub('lift_homed', Bool, self._cb_lift_homed)
        sub('lift_error', Bool, self._cb_lift_error)
        sub('lift_alarm', String, self._cb_lift_alarm)
        sub('led_state_all', String, self._cb_led_state_all)
        sub('crevis_connected', Bool, self._cb_crevis_connected)
        sub('charge_port_on', Bool, self._cb_charge_port_on)

        rospy.loginfo("[Navifra] Peripheral interface ready "
                      f"(require_safety_link={self.require_safety_link})")

    # ==========================================================
    # CALLBACKS
    # ==========================================================
    def _cb_estop(self, msg):
        prev = self._estop
        self._estop = bool(msg.data)
        self._estop_stamp = time.time()
        if self._estop and prev is not True:
            self._estop_edge_seen = True
            rospy.logerr("[Navifra] HARDWARE E-STOP ACTIVE (/safety/estop=true)")
            if self._on_estop is not None:
                # Fires in the subscriber thread so a caller blocked in a
                # motion/scan wait is interrupted immediately.
                try:
                    self._on_estop()
                except Exception as e:
                    rospy.logerr(f"[Navifra] on_estop handler raised: {e}")
        elif prev is True and not self._estop:
            rospy.logwarn("[Navifra] E-stop cleared")

    def _cb_safety_connected(self, msg):
        was = self._safety_connected
        self._safety_connected = bool(msg.data)
        if was is True and not self._safety_connected:
            rospy.logerr("[Navifra] Safety PLC link lost")

    def _cb_bms(self, msg):
        self._battery = msg
        self._battery_stamp = time.time()

    def _cb_soc(self, msg):
        self._soc = float(msg.data)

    def _cb_lift_position(self, msg):
        self._lift_position = int(msg.data)

    def _cb_lift_status(self, msg):
        self._lift_status = msg.data

    def _cb_lift_homed(self, msg):
        self._lift_homed = bool(msg.data)

    def _cb_lift_error(self, msg):
        was = self._lift_error
        self._lift_error = bool(msg.data)
        if self._lift_error and not was:
            rospy.logerr(f"[Navifra] Lift alarm active: {self._lift_alarm}")

    def _cb_lift_alarm(self, msg):
        self._lift_alarm = msg.data

    def _cb_led_state_all(self, msg):
        self._led_state_all = msg.data

    def _cb_crevis_connected(self, msg):
        self._crevis_connected = bool(msg.data)

    def _cb_charge_port_on(self, msg):
        self._charge_port_on = bool(msg.data)

    # ==========================================================
    # SAFETY
    # ==========================================================
    @property
    def estop_active(self):
        """True only if the driver actually reported an active e-stop.

        A never-received topic is NOT reported as active — otherwise the whole
        stack would refuse to run whenever safety_io_driver is not launched.
        Use safety_link_ok() to detect that case instead.
        """
        return self._estop is True

    def safety_link_ok(self):
        """True if /safety/estop is present and fresh, and the PLC is connected."""
        if self._estop is None:
            return False
        if (time.time() - self._estop_stamp) > self.safety_timeout_s:
            return False
        return self._safety_connected is not False

    def safe_to_move(self):
        """(bool, reason) — gate to call before starting motion or a scan."""
        if self.estop_active:
            return False, "hardware e-stop active (/safety/estop=true)"
        if not self.safety_link_ok():
            reason = "safety PLC link not confirmed (safety_io_driver running?)"
            if self.require_safety_link:
                return False, reason
            rospy.logwarn(f"[Navifra] {reason} — proceeding "
                          "(navifra.require_safety_link=false)")
        return True, "ok"

    def take_estop_edge(self):
        """Consume a latched false->true e-stop transition. True once per event."""
        if self._estop_edge_seen:
            self._estop_edge_seen = False
            return True
        return False

    # ==========================================================
    # LIGHTING
    # ==========================================================
    def _set_led(self, name, on):
        try:
            self._pub_led[name].publish(Bool(bool(on)))
        except Exception as e:
            rospy.logwarn(f"[Navifra] LED {name} publish failed: {e}")

    def vision_led(self, on):
        """VISION lamp — scan illumination. Single fixed-colour lamp, on/off only."""
        self._set_led('vision', on)

    def front_led(self, on):
        self._set_led('front', on)

    def side_led(self, on):
        self._set_led('side', on)

    def set_status_color(self, color):
        """Drive the RGB STATUS lamp. Always writes all three channels."""
        key = (color or 'off').lower()
        if key not in STATUS_COLORS:
            rospy.logwarn(f"[Navifra] Unknown status colour '{color}' — using off")
            key = 'off'
        r, g, b = STATUS_COLORS[key]
        self._set_led('status_red', r)
        self._set_led('status_green', g)
        self._set_led('status_blue', b)

    def status_leds_off(self):
        """Darken only the STATUS lamp. Leaves VISION alone.

        Use this from a node that does not own the VISION lamp — killing it
        mid-capture would desynchronise it from the shutter.
        """
        self.set_status_color('off')

    def all_leds_off(self):
        """Darken every lamp. Only for a node that owns all of them."""
        for name in ('vision', 'front', 'side'):
            self._set_led(name, False)
        self.set_status_color('off')

    @property
    def led_state_all(self):
        return self._led_state_all

    @property
    def crevis_connected(self):
        return self._crevis_connected

    # ==========================================================
    # BATTERY
    # ==========================================================
    @property
    def battery_percent(self):
        """Battery charge [%], or None. /bms/state.percentage is 0.0-1.0."""
        if self._soc is not None:
            return self._soc
        if self._battery is not None and self._battery.percentage is not None:
            return float(self._battery.percentage) * 100.0
        return None

    @property
    def battery_state(self):
        return self._battery

    def battery_low(self):
        pct = self.battery_percent
        return pct is not None and pct < self.low_battery_pct

    def charging(self):
        """True only if the relay is on AND current is actually flowing.

        charge_port_on alone is just relay output state — the driver guide is
        explicit that it does not by itself mean charging.
        """
        if not self._charge_port_on:
            return False
        b = self._battery
        return b is not None and b.current is not None and b.current > 0

    def set_charging(self, on):
        try:
            self._pub_charging.publish(Bool(bool(on)))
        except Exception as e:
            rospy.logwarn(f"[Navifra] charging publish failed: {e}")

    # ==========================================================
    # LIFT
    # ==========================================================
    @property
    def lift_position(self):
        return self._lift_position

    @property
    def lift_homed(self):
        return self._lift_homed

    @property
    def lift_error(self):
        return self._lift_error

    @property
    def lift_status(self):
        return self._lift_status

    def lift_up(self):
        """Start moving up. Runs until lift_stop() — the upper end has NO limit
        switch, so the caller must bound this. See lifter_node."""
        self._pub_lift_command.publish(String("up"))

    def lift_down(self):
        """Start moving down. The lower limit switch stops the drive itself."""
        self._pub_lift_command.publish(String("down"))

    def lift_stop(self):
        self._lift_cancel = True
        self._pub_lift_command.publish(String("stop"))

    def lift_velocity(self, rpm):
        """Raw velocity command [rpm], sign = direction. Manual mode only.

        Same unbounded-at-the-top caveat as lift_up(); prefer lift_goto().
        """
        self._pub_lift_velocity_cmd.publish(Int16(int(rpm)))

    def lift_reset_alarm(self):
        """Manual alarm reset (PID_ALARM_RESET). Driver may auto-reset first."""
        self._pub_lift_reset.publish(Bool(True))

    def lift_home(self, timeout_s=None):
        """Home the lift to the lower limit (absolute origin 0).

        Position is incremental (encoder) and is lost on power cycle, so this
        must run once per power cycle before any absolute position command.
        Returns (bool, str).
        """
        timeout_s = float(timeout_s if timeout_s is not None
                          else self.cfg.get('lift_home_timeout_s', 130.0))
        ok, why = self.safe_to_move()
        if not ok:
            return False, f"lift_home refused: {why}"

        self._lift_homed = None
        self._lift_cancel = False
        self._pub_lift_home.publish(Bool(True))
        rospy.loginfo("[Navifra] Lift homing started")

        deadline = time.time() + timeout_s
        rate = rospy.Rate(5)
        while time.time() < deadline and not rospy.is_shutdown():
            if self._lift_cancel:
                return False, "lift homing cancelled by stop"
            if self.estop_active:
                return False, "e-stop during lift homing"
            if self._lift_error:
                return False, f"lift alarm during homing: {self._lift_alarm}"
            if self._lift_homed:
                rospy.loginfo(f"[Navifra] Lift homed at {self._lift_position}")
                return True, "homed"
            rate.sleep()
        return False, f"lift homing timeout after {timeout_s:.0f}s"

    def lift_goto(self, counts, timeout_s=None):
        """Move the lift to an absolute encoder count. Requires prior homing."""
        timeout_s = float(timeout_s if timeout_s is not None
                          else self.cfg.get('lift_move_timeout_s', 60.0))
        ok, why = self.safe_to_move()
        if not ok:
            return False, f"lift_goto refused: {why}"
        if not self._lift_homed:
            return False, ("lift not homed — absolute position is meaningless "
                           "until lift_home() runs this power cycle")

        target = int(counts)
        self._lift_cancel = False
        self._pub_lift_position_cmd.publish(Int32(target))
        rospy.loginfo(f"[Navifra] Lift -> {target} counts")
        return self._wait_lift_settle(target, timeout_s)

    def lift_jog(self, delta_counts, timeout_s=None):
        """Move the lift by a RELATIVE count offset (/lift/inc_position_cmd).

        Unlike lift_goto() this does not require homing — the driver ignores
        absolute position commands until homed, but relative ones work off the
        raw incremental count. Use it to reach a working height before the
        first homing of a power cycle, not to reach a repeatable position:
        the count drifts 1000-1800 per full up-down cycle (driver guide 3.5).
        """
        timeout_s = float(timeout_s if timeout_s is not None
                          else self.cfg.get('lift_move_timeout_s', 60.0))
        ok, why = self.safe_to_move()
        if not ok:
            return False, f"lift_jog refused: {why}"
        if self._lift_position is None:
            return False, "lift position unknown (lift_driver running?)"

        target = int(self._lift_position) + int(delta_counts)
        self._lift_cancel = False
        self._pub_lift_inc_cmd.publish(Int32(int(delta_counts)))
        rospy.loginfo(f"[Navifra] Lift {int(delta_counts):+d} counts "
                      f"-> ~{target}")
        return self._wait_lift_settle(target, timeout_s)

    def _wait_lift_settle(self, target, timeout_s):
        """Block until the lift is within lift_settle_tol of target."""
        deadline = time.time() + timeout_s
        rate = rospy.Rate(5)
        while time.time() < deadline and not rospy.is_shutdown():
            if self._lift_cancel:
                return False, "lift move cancelled by stop"
            if self.estop_active:
                return False, "e-stop during lift move"
            if self._lift_error:
                return False, f"lift alarm during move: {self._lift_alarm}"
            pos = self._lift_position
            if pos is not None and abs(pos - target) <= self.lift_settle_tol:
                return True, f"at {pos}"
            rate.sleep()
        return False, (f"lift move timeout after {timeout_s:.0f}s "
                       f"(pos={self._lift_position}, target={target})")

    def lift_at(self, counts, tol=None):
        """True if the lift is within tol counts of the given position."""
        if self._lift_position is None:
            return False
        t = self.lift_settle_tol if tol is None else int(tol)
        return abs(self._lift_position - int(counts)) <= t

    # ==========================================================
    # SHUTDOWN
    # ==========================================================
    def shutdown(self, leds='status'):
        """Leave the machine dark and the charge relay off.

        leds='status' (default) darkens only the STATUS lamp — correct for
        task_executor, which does not own VISION. leds='all' is for a node that
        owns every lamp.
        """
        try:
            if leds == 'all':
                self.all_leds_off()
            else:
                self.status_leds_off()
            self.set_charging(False)
        except Exception:
            pass
