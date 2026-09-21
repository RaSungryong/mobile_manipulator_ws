#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keyence_standoff — the Keyence DL-EN1 standoff loop as pure logic.

ArmController used to hold this loop inline (a 5 mm engage window, a fixed
1 mm step, a fixed 1 s sleep, one raw sample per step, and no record of the
outcome). It is now this class so it can be run offline against a surface
model, and so the behaviour is described in one place.

Everything in here is in ONE unit: perpendicular standoff millimetres,
"approach-positive" — a positive motion moves the tool TOWARD the surface,
a positive error means the tool is CLOSER than the target. The caller
(ArmController) owns the two conversions that make that true on the robot:

  * reading -> perpendicular:  perp = reading * cos(beam_angle)
  * approach mm -> tool Z:     dz_toolZ = approach_mm * (-keyence_dir)

so the loop never sees the beam angle or the sign convention.

What changed against the inline loop, and why
---------------------------------------------
1. ENGAGE RANGE — was |perp| < 5 mm, else "skip" and scan anyway. Now the
   loop engages up to `activate_threshold_mm` (20 by default) — in practice
   the sensor's own measuring range, since out-of-range readings arrive as
   the +/-99999 sentinel and are rejected as invalid, not as "far".
2. STEP SIZE is direction-aware. Approaching the mould is the dangerous
   direction, so an approach step is capped at
   max(max_step_mm, approach_fraction * |gap|): at most half the MEASURED
   remaining gap while far away (a reading would have to be wrong by 2x to
   reach the surface, and it is re-measured every step), and the old 1 mm
   fine step inside the last 2 mm. Retreating is always safe, so a retreat
   step may be up to `retreat_step_mm` (3 mm).
3. TOTAL TRAVEL BUDGET (`max_travel_mm`) — the loop cannot walk further
   than this in total, whatever the readings say.
4. FRESH, MEDIAN READINGS. Each decision uses the median of `samples` NEW
   readings taken after the previous move settled; a value that is the
   out-of-range sentinel (|v| >= invalid_abs_mm) is dropped. A stale cached
   value can no longer drive the loop (the old code would happily step ten
   times against a reading that stopped updating), and one reflective
   dropout frame cannot end or misdirect a correction.
5. RESPONSE CHECK. After a step of >= `response_min_step_mm`, the reading
   must move by at least `min_response_ratio` of the commanded distance.
   Two consecutive non-responses abort the loop: the sensor is frozen, the
   arm did not move, or the beam is not on the surface it thinks it is —
   none of which should be answered with more approach steps.
6. ADAPTIVE GAIN (opt-out). The oblique beam walks the laser spot sideways
   as the tool moves, so on sloped or rough material the reading changes
   MORE than the tool moved (docs/keyence_scan_chain.md open issue 1 — an
   effective sensitivity of 7.5 was reconstructed from the first live run,
   past the divergence limit of 3.4). The loop measures that ratio from the
   last step and divides the next step by it, clamped to [1, gain_ratio_max]
   — it only ever REDUCES the gain, never amplifies.
7. TARGET STANDOFF is a parameter. `setpoint_mm` is the perpendicular
   reading the loop drives to (0 = the sensor's own zero, i.e. the DL-EN1's
   configured 10 mm); ArmController derives it from
   keyence.target_distance_mm - keyence.sensor_zero_mm, so the scan
   standoff can be moved away from the mould without reconfiguring the
   sensor. NOTE the Basler focus / Ra model were established at the 10 mm
   standoff; changing the target changes the images.
8. SEEK (opt-in; ON in robot.yaml since 2026-09-21). When a reading is out
   of range the sentinel's SIGN says which side (negative = too far,
   positive = too close, matching the sensor polarity). With `seek_enabled`
   the loop steps `seek_step_mm` in the indicated direction — never on a
   mixed/ambiguous sentinel — until a reading appears, then hands over to
   the closed loop. The seek has ITS OWN budget: `seek_max_mm` of travel
   and the steps that takes; it consumes neither `max_steps` nor
   `max_travel_mm`, which bound the closed loop that follows (the record's
   `travel_mm` still counts both). Approaching on no measurement is what
   this loop exists to avoid, so two things bound the risk: the step is
   smaller than the sensor's window (the window cannot be jumped over —
   the first in-range reading stops the seek), and `seek_max_mm` caps how
   far a beam that sees NOTHING (spot off the plate edge, reported as "far")
   can walk the tool down. The far-side sign was confirmed on the robot
   2026-09-21: tool at the home pose, nothing in range, raw -100000 =
   negative = "far". A "near" sentinel makes the seek RETREAT, always safe.
9. THE OUTCOME IS RETURNED, not logged and forgotten: converged / reason /
   final error / steps / travel, so the scan CSV can carry it.
"""

from dataclasses import dataclass, field
from typing import Callable, List, Optional


SENTINEL_ABS_MM = 90.0     # |reading| at/above this is the +/-99999 sentinel


@dataclass
class StandoffConfig:
    tolerance_mm: float = 0.2          # converged when |err| <= this
    kp: float = 0.8                    # proportional gain (per-step residual |1-kp|)
    max_steps: int = 15                # decision steps before giving up
    max_step_mm: float = 1.0           # approach fine-step cap (near the target)
    approach_fraction: float = 0.5     # far-regime approach cap = fraction * |gap|
    retreat_step_mm: float = 3.0       # retreat cap (moving away is always safe)
    activate_threshold_mm: float = 20.0  # |err| >= this: reading not trusted, stop
    max_travel_mm: float = 25.0        # total |motion| budget for one adjustment
    invalid_abs_mm: float = SENTINEL_ABS_MM
    samples: int = 5                   # fresh readings per decision (median)
    read_timeout_s: float = 1.0        # wait for those samples
    adaptive_gain: bool = True
    gain_ratio_max: float = 4.0        # adaptive divisor clamp [1, this]
    response_min_step_mm: float = 0.3  # response check only after steps >= this
    min_response_ratio: float = 0.25   # |d(reading)| / |step| below this = no response
    max_nonresponse: int = 2           # consecutive non-responses before abort
    invalid_retries: int = 2           # consecutive invalid reads before abort
    setpoint_mm: float = 0.0           # perpendicular reading to hold (0 = sensor zero)
    seek_enabled: bool = False
    seek_step_mm: float = 2.0          # < the sensor window, so it cannot be jumped
    seek_max_mm: float = 10.0          # seek travel budget, separate from max_travel_mm


@dataclass
class StandoffResult:
    converged: bool
    reason: str
    steps: int = 0
    travel_mm: float = 0.0
    final_err_mm: Optional[float] = None
    history: List[dict] = field(default_factory=list)

    def summary(self) -> str:
        err = ('n/a' if self.final_err_mm is None
               else f"{self.final_err_mm:+.2f} mm")
        if self.converged:
            return (f"standoff ok (err {err}, {self.steps} step"
                    f"{'s' if self.steps != 1 else ''}, "
                    f"travel {self.travel_mm:.1f} mm)")
        return (f"standoff NOT corrected: {self.reason} "
                f"(err {err}, {self.steps} steps, travel {self.travel_mm:.1f} mm)")


class _Reading:
    """One decision's measurement: median of the valid samples, or the
    sentinel side when there was no valid sample."""
    __slots__ = ('perp', 'n_valid', 'n_total', 'sentinel_sign')

    def __init__(self, perp, n_valid, n_total, sentinel_sign):
        self.perp = perp                  # float or None
        self.n_valid = n_valid
        self.n_total = n_total
        self.sentinel_sign = sentinel_sign  # +1 / -1 when ALL invalid agree, else 0


def _median(vals: List[float]) -> float:
    s = sorted(vals)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


class StandoffController:
    """Closed-loop standoff correction, in approach-positive perpendicular mm.

    read(n, timeout_s) -> list of FRESH perpendicular readings (already
        projected by the caller); may be shorter than n on timeout, empty
        when the sensor is silent.
    move(approach_mm) -> bool; positive moves toward the surface. The
        caller settles and returns only once the arm is at rest.
    cancelled() -> bool
    log_info / log_warn: optional text sinks.
    """

    def __init__(self, cfg: StandoffConfig,
                 read: Callable[[int, float], List[float]],
                 move: Callable[[float], bool],
                 cancelled: Optional[Callable[[], bool]] = None,
                 log_info: Optional[Callable[[str], None]] = None,
                 log_warn: Optional[Callable[[str], None]] = None):
        self.cfg = cfg
        self._read = read
        self._move = move
        self._cancelled = cancelled or (lambda: False)
        self._info = log_info or (lambda s: None)
        self._warn = log_warn or (lambda s: None)

    # ------------------------------------------------------------------
    def _measure(self) -> _Reading:
        n = max(1, int(self.cfg.samples))
        vals = list(self._read(n, float(self.cfg.read_timeout_s)) or [])
        valid = [v for v in vals if abs(v) < self.cfg.invalid_abs_mm]
        invalid = [v for v in vals if abs(v) >= self.cfg.invalid_abs_mm]
        # Majority of the requested samples must be valid; otherwise the
        # frame is a dropout / out-of-range decision, not a measurement.
        need = (n + 1) // 2
        if len(valid) >= need:
            return _Reading(_median(valid), len(valid), len(vals), 0)
        sign = 0
        if invalid and not valid:
            if all(v > 0 for v in invalid):
                sign = +1
            elif all(v < 0 for v in invalid):
                sign = -1
        return _Reading(None, len(valid), len(vals), sign)

    def _approach_cap(self, err_abs: float) -> float:
        return max(float(self.cfg.max_step_mm),
                   float(self.cfg.approach_fraction) * err_abs)

    # ------------------------------------------------------------------
    def run(self) -> StandoffResult:
        cfg = self.cfg
        res = StandoffResult(converged=False, reason='not started')
        ratio = 1.0                 # adaptive gain divisor
        nonresp = 0
        invalid_run = 0
        seek_travel = 0.0           # seek budget (own), see docstring 8
        loop_travel = 0.0           # closed-loop travel against max_travel_mm
        prev_err = None             # error before the last motion
        prev_cmd = None             # last commanded approach (mm)

        step = 0                    # closed-loop decisions; seek steps do not count
        # A non-positive seek step is "seek off" (a 0 mm step would spin
        # without moving); otherwise bound the seek in steps too, so a huge
        # budget cannot loop past what it can travel.
        seek_on = bool(cfg.seek_enabled) and float(cfg.seek_step_mm) > 0.0
        seek_steps_max = (int(cfg.seek_max_mm / float(cfg.seek_step_mm)) + 2
                          if seek_on else 0)
        seek_steps = 0
        while step < int(cfg.max_steps):
            if self._cancelled():
                res.reason = 'cancelled'
                return res

            m = self._measure()

            # ---------- no usable measurement ----------
            if m.perp is None:
                if m.n_total == 0:
                    res.reason = 'no Keyence data (sensor/node silent)'
                    self._warn(f"[Standoff] {res.reason}")
                    return res
                invalid_run += 1
                # Seek: only on an unambiguous sentinel side, only when
                # enabled, only within its own budget.
                if seek_on and m.sentinel_sign != 0:
                    side = 'near' if m.sentinel_sign > 0 else 'far'
                    if (seek_travel + cfg.seek_step_mm > cfg.seek_max_mm
                            or seek_steps >= seek_steps_max):
                        res.reason = (f"still out of range on the {side} side "
                                      f"after seeking {seek_travel:.1f} mm "
                                      f"(seek_max_mm {cfg.seek_max_mm})")
                        self._warn(f"[Standoff] {res.reason}")
                        return res
                    # negative sentinel = too far -> approach (+); positive
                    # = too close -> retreat (-). Same polarity as readings.
                    cmd = -m.sentinel_sign * float(cfg.seek_step_mm)
                    self._info(f"  -> [Seek {seek_steps+1}] out of range on the "
                               f"{side} side; "
                               f"{'retreat' if cmd < 0 else 'approach'} "
                               f"{abs(cmd):.2f} mm ({seek_travel:.1f}/"
                               f"{cfg.seek_max_mm} mm used)")
                    if not self._move(cmd):
                        res.reason = 'move failed while seeking'
                        return res
                    res.steps += 1
                    res.travel_mm += abs(cmd)
                    seek_travel += abs(cmd)
                    seek_steps += 1
                    invalid_run = 0     # a seek step is progress, not a retry
                    prev_err, prev_cmd = None, None
                    res.history.append(dict(step=res.steps, kind='seek',
                                            cmd_mm=cmd, sentinel=m.sentinel_sign))
                    continue
                if invalid_run > int(cfg.invalid_retries):
                    if m.sentinel_sign != 0 and not seek_on:
                        res.reason = (f"out of range on the "
                                      f"{'near' if m.sentinel_sign > 0 else 'far'}"
                                      f" side (seek disabled)")
                    else:
                        res.reason = (f"no valid reading ({m.n_valid}/{m.n_total} "
                                      "samples valid — dropout or out of range)")
                    self._warn(f"[Standoff] {res.reason}")
                    return res
                self._warn(f"[Standoff] invalid reading "
                           f"({m.n_valid}/{m.n_total} valid), retrying")
                continue

            invalid_run = 0
            err = float(m.perp) - float(cfg.setpoint_mm)
            res.final_err_mm = err
            res.history.append(dict(step=step + 1, kind='measure', err_mm=err,
                                    n_valid=m.n_valid))

            # ---------- response check / adaptive gain ----------
            if prev_cmd is not None and prev_err is not None \
                    and abs(prev_cmd) >= cfg.response_min_step_mm:
                observed = err - prev_err           # approach raises err
                r = observed / prev_cmd
                res.history[-1]['response_ratio'] = r
                if r < cfg.min_response_ratio:
                    nonresp += 1
                    self._warn(f"[Standoff] reading moved {observed:+.2f} mm for a "
                               f"{prev_cmd:+.2f} mm step (ratio {r:.2f}) — "
                               f"no response {nonresp}/{cfg.max_nonresponse}")
                    if nonresp >= int(cfg.max_nonresponse):
                        res.reason = ('reading does not follow the motion '
                                      '(frozen sensor / arm not moving / '
                                      'beam off the surface)')
                        self._warn(f"[Standoff] {res.reason}")
                        return res
                else:
                    nonresp = 0
                    if cfg.adaptive_gain:
                        ratio = min(max(r, 1.0), float(cfg.gain_ratio_max))
                    else:
                        ratio = 1.0

            # ---------- convergence / trust ----------
            if abs(err) <= cfg.tolerance_mm:
                res.converged = True
                res.reason = 'converged'
                self._info(f"[Standoff] on target: err {err:+.3f} mm "
                           f"after {res.steps} step(s)")
                return res
            if abs(err) >= cfg.activate_threshold_mm:
                res.reason = (f"|err| {abs(err):.1f} mm >= activate threshold "
                              f"{cfg.activate_threshold_mm} mm (reading not trusted)")
                self._warn(f"[Standoff] {res.reason}")
                return res

            # ---------- step ----------
            cmd = -err * float(cfg.kp) / ratio        # approach-positive
            if cmd > 0:
                cap = self._approach_cap(abs(err))
                cmd = min(cmd, cap)
            else:
                cmd = max(cmd, -float(cfg.retreat_step_mm))
            if loop_travel + abs(cmd) > cfg.max_travel_mm:
                res.reason = f"travel budget {cfg.max_travel_mm} mm exhausted"
                self._warn(f"[Standoff] {res.reason} (err {err:+.2f} mm)")
                return res

            self._info(f"  -> [Standoff {step+1}/{cfg.max_steps}] err {err:+.3f} mm "
                       f"({m.n_valid}/{m.n_total} samples) -> "
                       f"{'approach' if cmd > 0 else 'retreat'} {abs(cmd):.3f} mm "
                       f"(kp {cfg.kp}, gain/{ratio:.2f})")
            if not self._move(cmd):
                res.reason = 'move failed'
                self._warn(f"[Standoff] {res.reason}")
                return res
            res.steps += 1
            res.travel_mm += abs(cmd)
            loop_travel += abs(cmd)
            res.history[-1]['cmd_mm'] = cmd
            prev_err, prev_cmd = err, cmd
            step += 1

        # Budget of steps used up: take one last measurement so the record
        # says where it ended, then report.
        m = self._measure()
        if m.perp is not None:
            res.final_err_mm = float(m.perp) - float(cfg.setpoint_mm)
            if abs(res.final_err_mm) <= cfg.tolerance_mm:
                res.converged = True
                res.reason = 'converged'
                return res
        res.reason = f"not converged within {cfg.max_steps} steps"
        self._warn(f"[Standoff] {res.reason} (err "
                   f"{res.final_err_mm if res.final_err_mm is not None else float('nan'):+.2f} mm)")
        return res
