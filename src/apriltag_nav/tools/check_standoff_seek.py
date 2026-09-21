#!/usr/bin/env python3
"""Offline check of the Keyence standoff loop's SEEK (2026-09-21).

Plant: a flat surface, the tool case bottom at standoff S mm. The sensor
sees the surface only inside its window [near, far] and reports the signed
+/-100000 sentinel outside it (negative = far, as confirmed on the robot
with the tool at the home pose). read() returns the PERPENDICULAR reading
the controller would hand the loop (zero - S), the sentinel unprojected.
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from apriltag_nav.keyence_standoff import StandoffController, StandoffConfig

ZERO, NEAR, FAR = 16.5, 6.5, 27.0
SENT = 100000.0
n_ok = n_fail = 0


def check(name, cond):
    global n_ok, n_fail
    if cond:
        n_ok += 1
    else:
        n_fail += 1
        print(f"FAIL: {name}")


class Plant:
    def __init__(self, s0, blind=False, near=NEAR, far=FAR):
        self.s = float(s0); self.blind = blind; self.near = near; self.far = far
        self.moves = []; self.min_s = self.s

    def read(self, n, timeout):
        if self.blind or self.s > self.far:
            return [-SENT] * n
        if self.s < self.near:
            return [SENT] * n
        return [ZERO - self.s] * n

    def move(self, approach):
        self.s -= approach
        self.min_s = min(self.min_s, self.s)
        self.moves.append(approach)
        return True


def cfg(**kw):
    base = dict(tolerance_mm=0.2, kp=0.8, max_steps=15, max_step_mm=1.0,
                approach_fraction=0.5, retreat_step_mm=3.0,
                activate_threshold_mm=45.0, max_travel_mm=25.0,
                invalid_abs_mm=90.0, samples=5,
                seek_enabled=True, seek_step_mm=3.0, seek_max_mm=50.0)
    base.update(kw)
    return StandoffConfig(**base)


def run(plant, c, cancelled=None):
    return StandoffController(c, plant.read, plant.move, cancelled=cancelled).run()


# 1. far start, seek then converge; seek steps do not eat max_steps
p = Plant(60.0); r = run(p, cfg())
seek = [h for h in r.history if h['kind'] == 'seek']
check('far 60: converged', r.converged and abs(p.s - ZERO) <= 0.2)
check('far 60: seek ~11 steps of 3 mm', 10 <= len(seek) <= 12 and all(h['cmd_mm'] == 3.0 for h in seek))
check('far 60: seek stopped at first in-range reading', p.moves[len(seek) - 1] == 3.0 and FAR - 3.0 <= 60.0 - 3.0 * len(seek) <= FAR)
check('far 60: closed loop had its own budget (total steps > 15)', r.steps > 15)
check('far 60: closed-loop travel <= max_travel', sum(abs(m) for m in p.moves[len(seek):]) <= 25.0 + 1e-9)
check('far 60: never touched the surface', p.min_s > 5.0)
check('far 60: record travel counts both', abs(r.travel_mm - sum(abs(m) for m in p.moves)) < 1e-9)

# 2. seek disabled: refuse, no motion
p = Plant(60.0); r = run(p, cfg(seek_enabled=False))
check('disabled: refused with side named', (not r.converged) and 'far side (seek disabled)' in r.reason)
check('disabled: no motion', p.moves == [])

# 3. seek budget: beam sees nothing -> walks exactly the budget, then stops
p = Plant(200.0, blind=True); r = run(p, cfg(seek_max_mm=50.0, seek_step_mm=3.0))
check('blind: stops with reason', (not r.converged) and 'still out of range on the far side' in r.reason)
check('blind: travel <= seek_max', r.travel_mm <= 50.0 + 1e-9 and r.travel_mm >= 45.0)

# 4. too far for the budget: stops short, no contact
p = Plant(90.0); r = run(p, cfg(seek_max_mm=40.0))
check('90 mm, budget 40: not converged, 13 x 3 mm = 39 <= 40 then stops', (not r.converged) and abs(p.s - 51.0) < 1e-9 and r.travel_mm <= 40.0)

# 5. near sentinel: retreat until in range, then converge
p = Plant(3.0); r = run(p, cfg())
seek = [h for h in r.history if h['kind'] == 'seek']
check('near 3: seek retreats', all(h['cmd_mm'] == -3.0 and h['sentinel'] > 0 for h in seek) and len(seek) >= 1)
check('near 3: converged', r.converged and abs(p.s - ZERO) <= 0.2)

# 6. in range from the start: no seek at all (unchanged behaviour)
p = Plant(22.0); r = run(p, cfg())
check('in range: no seek entries', not any(h['kind'] == 'seek' for h in r.history))
check('in range: converged', r.converged and abs(p.s - ZERO) <= 0.2)

# 7. cancel during the seek
p = Plant(60.0); calls = [0]
def cancelled():
    calls[0] += 1
    return calls[0] > 4
r = run(p, cfg(), cancelled=cancelled)
check('cancel: reason', r.reason == 'cancelled' and not r.converged)
check('cancel: <= 4 moves', len(p.moves) <= 4)

# 8. step larger than the window is the misconfiguration to avoid: the
#    plant is jumped over and the near sentinel then walks it back — still
#    no contact, but document that seek_step_mm must stay under the window.
p = Plant(60.0); r = run(p, cfg(seek_step_mm=2.0))
check('2 mm step: converged too', r.converged)

# 9. pathological config cannot spin: step 0 -> bounded, returns
p = Plant(60.0); r = run(p, cfg(seek_step_mm=0.0, seek_max_mm=10.0))
check('step 0: treated as seek off, no motion', not r.converged and p.moves == [] and 'seek disabled' in r.reason)

# 10. the robot.yaml values: 5 mm step, 40 mm budget -> works from 67 mm
p = Plant(66.0); r = run(p, cfg(seek_step_mm=5.0, seek_max_mm=40.0))
seek = [h for h in r.history if h['kind'] == 'seek']
check('66 mm start: converged', r.converged and abs(p.s - ZERO) <= 0.2)
check('66 mm start: <= 8 seek steps, never touched', len(seek) <= 8 and p.min_s > 4.0)
p = Plant(80.0); r = run(p, cfg(seek_step_mm=5.0, seek_max_mm=40.0))
check('80 mm start: out of budget, stops at 40 mm, no contact', (not r.converged) and abs(p.s - 40.0) < 1e-9)

print(f"{n_ok} ok, {n_fail} fail")
sys.exit(1 if n_fail else 0)
