#!/usr/bin/env python3
"""Offline checks of the VIA-tag rule (robot.yaml `via_tag`, 2026-11-11):
intermediate tags of a route are passed without the aim stop / pivot and
without the arrival align while the base is on the tag's line and square
to it; the command's last tag, pivot tags, direction changes and the
500-series keep the full aim + stop + align.

    python3 src/apriltag_nav/tools/check_via_tags.py

Runs the real MobileController with the real robot.yaml against the 2-D
plant of check_nav_sequencing.py (0.55 s angular / 0.25 s linear command
delay, 0.1 s image latency, bumper occlusion), optionally with corner
noise on the detections. Exit status 1 on failure.
"""
import copy
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_nav_sequencing as H  # noqa: E402  (stubs rospy, loads the controller)

CAM, CLK, LOG, Rate = H.CAM, H.CLK, H.LOG, H.Rate
checks = []


def check(name, cond, detail=''):
    checks.append(bool(cond))
    print(('PASS ' if cond else 'FAIL ') + name + (f'  [{detail}]' if detail else ''))


def cfg_with(**via):
    cfg = copy.deepcopy(H.CFG0)
    cfg['robot'].setdefault('via_tag', {})
    cfg['robot']['via_tag'].update(via)
    return cfg


def add_noise(p, sigma_px, seed):
    """Gaussian noise on every detected corner / centre (px)."""
    rng = np.random.RandomState(seed)
    real_view = p.view

    def view(x, y, psi):
        dets = real_view(x, y, psi)
        for d in dets.values():
            d['corners'] = d['corners'] + rng.normal(0.0, sigma_px, d['corners'].shape)
            d['center_x'] += rng.normal(0.0, sigma_px)
            d['center_y'] += rng.normal(0.0, sigma_px)
            d['x'] = (d['center_x'] - H.CX) * H.Z / H.FX
            d['y'] = (d['center_y'] - H.CY) * H.Z / H.FY
        return dets
    p.view = view


def run_lane(ids, tags, reverse=False, cfg=None, y0=0.0, noise=None):
    """Drive the lane `ids` (in route order) with move_to_tag. `tags` maps
    id -> (X, Y[, heading]). Forward: the base starts with the lens over
    the first tag; reverse: the same (as a forward command left it), then
    backs along -x. Returns a result dict."""
    x_start = tags[ids[0]][0]
    c, p = H.make(x_start - CAM, y0, 0.0, dict(tags), cfg)
    if noise:
        add_noise(p, noise[0], noise[1])
    direction = 'backward' if reverse else 'forward'
    for a, b in zip(ids, ids[1:]):
        c.map_mgr.edges[(a, b)] = {'type': 'move', 'direction': direction}
    c.map_mgr.get_tag_info = lambda t: {'x': tags[t][0], 'y': tags[t][1], 'zone': 'A'}
    c.map_mgr.path = list(ids)
    c.last_known_tag = ids[0]
    calls = []
    H.spy(c, calls)
    turn_signs = []
    real_pp = c.execute_pure_pursuit

    def pp(**kw):
        turn_signs.append((kw['target_id'], c._last_align_turn_sign))
        return real_pp(**kw)
    c.execute_pure_pursuit = pp
    ymax = [0.0]
    vtrace = []                      # (t, executed linear speed) of the plant
    stops = [0]
    real_stop = c.stop

    def stop():
        stops[0] += 1
        real_stop()
    c.stop = stop
    step = H.STEP_HOOK[0]

    def hook(dt):
        step(dt)
        ymax[0] = max(ymax[0], abs(p.y))
        vtrace.append((CLK.t, abs(p.last_v)))
    H.STEP_HOOK[0] = hook
    t0 = CLK.t
    ok = c.move_to_tag(ids[-1])
    dur = CLK.t - t0
    for _ in range(20):
        Rate(20).sleep()
    last = tags[ids[-1]]
    hdg = last[2] if len(last) > 2 else 0.0
    lx, ly = p.lens()
    return {'ok': ok, 'calls': calls, 'dur': dur, 'ymax_mm': ymax[0] * 1000.0,
            'lens_err_mm': math.hypot(lx - last[0], ly - last[1]) * 1000.0,
            'yaw_err_deg': math.degrees(H.wrap(p.psi - hdg)),
            'aims_at_rest': [l for l in LOG if l.startswith('[Aim] at rest')],
            'via_log': [l for l in LOG if l.startswith('[Via]')],
            'turn_signs': turn_signs, 'c': c, 'p': p, 'stops': stops[0], 'vtrace': vtrace,
            'arrive_cmd': [float(l.split('cmd ')[1].split(' m/s')[0])
                           for l in LOG if l.startswith('Arrived at')],
            'final_approach': sum(l.startswith('[Navigation] Final approach') for l in LOG),
            't0': t0}


def seq(calls):
    return [x[:2] for x in calls]


def main():
    ids = [101, 102, 103, 104, 105, 106]
    straight = {t: (0.40 * i, 0.0) for i, t in enumerate(ids)}
    on, off = cfg_with(enabled=True), cfg_with(enabled=False)
    # the 2026-11-11 first step: stops removed, speed caps and the zero
    # command at every column kept
    slow = cfg_with(enabled=True, approach_speed=0.0, pass_without_stop=False)
    vs = float(on['robot']['via_tag']['approach_speed'])

    # ---- A. forward lane, base on the line: nothing happens at the via tags
    r = run_lane(ids, straight, cfg=on)
    exp = [('align', 101)] + [('pp', t) for t in ids[1:]] + [('align', 106)]
    check('A1 forward 5 hops: align(101), five drives, ONE align at the last tag',
          r['ok'] and seq(r['calls']) == exp, str(seq(r['calls'])))
    check('A2 no aim at rest at the four via tags, the normal aim at the last tag',
          len(r['aims_at_rest']) == 1 and
          sum('no stop, no pivot' in l for l in r['via_log']) == 4 and
          sum('align skipped' in l for l in r['via_log']) == 4,
          f"aims {len(r['aims_at_rest'])} via {len(r['via_log'])}")
    check('A3 the command still ends with the lens on the last tag within 5 mm and 0.2 deg',
          r['lens_err_mm'] < 5.0 and abs(r['yaw_err_deg']) <= 0.2,
          f"{r['lens_err_mm']:.1f} mm {r['yaw_err_deg']:+.2f} deg")
    check('A4 no in-place turn before a via launch -> the backlash feed-forward sign is cleared',
          all(s == 0 for t, s in r['turn_signs'][2:]), str(r['turn_signs']))
    r0 = run_lane(ids, straight, cfg=off)
    exp0 = [('align', 101)]
    for t in ids[1:]:
        exp0 += [('pp', t), ('align', t)]
    check('A5 via_tag.enabled false: every hop aims and aligns (the old sequence)',
          r0['ok'] and seq(r0['calls']) == exp0 and len(r0['aims_at_rest']) == 5
          and not r0['via_log'], f"{seq(r0['calls'])} aims {len(r0['aims_at_rest'])}")
    check('A6 forward route is faster with via tags',
          r['dur'] < r0['dur'] - 4 * 2.0, f"{r['dur']:.1f} s vs {r0['dur']:.1f} s "
          f"({(r0['dur'] - r['dur']) / 4:.1f} s saved per via tag)")
    fwd_on, fwd_off = r['dur'], r0['dur']
    check('A7 the four via columns are crossed at approach_speed, the last tag at the final-approach crawl',
          len(r['arrive_cmd']) == 5 and all(abs(v - vs) < 1e-6 for v in r['arrive_cmd'][:4])
          and r['arrive_cmd'][4] <= 0.0151, str(r['arrive_cmd']))
    check('A8 no final approach is latched at a via tag (once, at the last tag)',
          r['final_approach'] == 1, str(r['final_approach']))
    # from 2 s after the launch until the last via column is crossed the base
    # never comes to rest: no zero command was sent at the via columns
    passed = [l for l in r['via_log'] if 'passed without stopping' in l]
    check('A9 all four via tags are passed WITHOUT a zero command', len(passed) == 4, str(len(passed)))
    arr_t = r['c']._via_arrival          # None: the last hop is not via
    v_mid = [v for t, v in r['vtrace'] if r['t0'] + 6.0 < t < r['t0'] + r['dur'] - 25.0]
    check('A10 ... and between the launch and the last hop the base never drops below 0.02 m/s',
          arr_t is None and len(v_mid) > 100 and min(v_mid) >= 0.02 - 1e-9,
          f'min {min(v_mid):.4f} m/s over {len(v_mid)} ticks')
    rs = run_lane(ids, straight, cfg=slow)
    check('A11 approach_speed 0 + pass_without_stop false: via columns reached at the crawl, a zero command at each',
          rs['ok'] and seq(rs['calls']) == exp and all(v <= 0.0151 for v in rs['arrive_cmd'])
          and rs['stops'] >= r['stops'] + 4 and r['dur'] < rs['dur'] - 4 * 2.0,
          f"cmd {rs['arrive_cmd']} stops {rs['stops']} vs {r['stops']}; {rs['dur']:.1f} s vs {r['dur']:.1f} s")
    rk = run_lane(ids, straight, cfg=cfg_with(enabled=True, pass_without_stop=False))
    check('A12 pass_without_stop false alone: approach_speed held, but a zero command at every via column',
          rk['ok'] and all(abs(v - vs) < 1e-6 for v in rk['arrive_cmd'][:4])
          and rk['stops'] >= r['stops'] + 4 and not any('without stopping' in l for l in rk['via_log']),
          f"stops {rk['stops']} vs {r['stops']}; {rk['dur']:.1f} s")
    fwd_slow = rs['dur']

    # ---- B. reverse lane: same rule; the command end re-seats as before
    rids = ids[::-1]
    r = run_lane(rids, straight, reverse=True, cfg=on)
    exp = ([('align', 106)] + [('pp', t) for t in rids[1:]]
           + [('align', 101), ('pp', 101), ('align', 101)])
    check('B1 reverse 5 hops: via tags passed, align + re-seat only at the last tag',
          r['ok'] and seq(r['calls']) == exp and r['calls'][-2][2] == 'forward', str(seq(r['calls'])))
    check('B2 reverse: no aim at rest at the via tags (one at the last tag, one in its re-seat)',
          len(r['aims_at_rest']) <= 2 and sum('align skipped' in l for l in r['via_log']) == 4,
          f"aims {len(r['aims_at_rest'])}")
    check('B3 reverse command ends on the FWD column within 5 mm and 0.2 deg',
          r['lens_err_mm'] < 5.0 and abs(r['yaw_err_deg']) <= 0.2,
          f"{r['lens_err_mm']:.1f} mm {r['yaw_err_deg']:+.2f} deg")
    r0 = run_lane(rids, straight, reverse=True, cfg=off)
    check('B4 reverse route is faster with via tags',
          r0['ok'] and r['dur'] < r0['dur'] - 4 * 2.0, f"{r['dur']:.1f} s vs {r0['dur']:.1f} s "
          f"({(r0['dur'] - r['dur']) / 4:.1f} s saved per via tag)")
    rev_on, rev_off = r['dur'], r0['dur']
    check('B5 reverse: the four via columns crossed at approach_speed without a zero command',
          all(abs(v - vs) < 1e-6 for v in r['arrive_cmd'][:4])
          and sum('passed without stopping' in l for l in r['via_log']) == 4, str(r['arrive_cmd']))
    rev_slow = run_lane(rids, straight, reverse=True, cfg=slow)['dur']

    # ---- C. base 12 mm beside the lane: the FIRST via tag gets the full aim + align
    r = run_lane(ids, straight, cfg=on, y0=0.012)
    s = seq(r['calls'])
    check('C1 12 mm off the line: aim at rest at tag 102, arrival yawed by it -> align(102)',
          r['ok'] and s[:3] == [('align', 101), ('pp', 102), ('align', 102)]
          and any('tag 102' in l and 'aiming at rest' in l for l in r['via_log'])
          and any('tag 102' in l and 'aligning' in l for l in r['via_log']),
          f"{s[:4]} {r['via_log'][:2]}")
    check('C2 ... after which the base is on the line and the remaining via tags are passed',
          s[3:] == [('pp', 103), ('pp', 104), ('pp', 105), ('pp', 106), ('align', 106)]
          and abs(r['p'].y) < 0.004, f"{s[3:]} y {r['p'].y * 1000:+.1f} mm")
    check('C3 ... and the last tag is reached within 5 mm / 0.2 deg',
          r['lens_err_mm'] < 5.0 and abs(r['yaw_err_deg']) <= 0.2,
          f"{r['lens_err_mm']:.1f} mm {r['yaw_err_deg']:+.2f} deg")
    r = run_lane(rids, straight, reverse=True, cfg=on, y0=0.012)
    check('C4 same in reverse: aim + align at the first via tag (105), last tag within 5 mm',
          r['ok'] and seq(r['calls'])[:3] == [('align', 106), ('pp', 105), ('align', 105)]
          and r['lens_err_mm'] < 5.0, f"{seq(r['calls'])[:4]} {r['lens_err_mm']:.1f} mm")

    # ---- D. a real-looking lane: tags laid +-0.7 deg and +-3 mm off, corner noise
    worst = {'fwd': 0.0, 'rev': 0.0}
    all_ok, lens_worst, n_aims = True, 0.0, 0
    for seed in range(4):
        rng = np.random.RandomState(100 + seed)
        laid = {t: (0.40 * i, rng.uniform(-0.003, 0.003), math.radians(rng.uniform(-0.7, 0.7)))
                for i, t in enumerate(ids)}
        for rev in (False, True):
            r = run_lane(rids if rev else ids, laid, reverse=rev, cfg=on, noise=(0.3, seed))
            all_ok &= bool(r['ok'])
            worst['rev' if rev else 'fwd'] = max(worst['rev' if rev else 'fwd'], r['ymax_mm'])
            lens_worst = max(lens_worst, r['lens_err_mm'])
            n_aims += sum('aiming at rest' in l for l in r['via_log'])
    check('D1 8 noisy lanes (tags +-0.7 deg, +-3 mm, 0.3 px corner noise) all complete',
          all_ok)
    check('D2 ... the base centre stays within 15 mm of the lane the whole way, both directions',
          worst['fwd'] < 15.0 and worst['rev'] < 15.0,
          f"fwd {worst['fwd']:.1f} mm, rev {worst['rev']:.1f} mm; "
          f"{n_aims} of 32 via tags needed an aim")
    check('D3 ... and every command ends within 5 mm of its last tag', lens_worst < 5.0,
          f'{lens_worst:.1f} mm')

    # ---- E. which hops are via
    c, p = H.make(-CAM, 0.0, 0.0, {101: (0.0, 0.0)}, on)
    E = c.map_mgr.edges
    E[(101, 102)] = E[(102, 103)] = {'type': 'move', 'direction': 'forward'}
    E[(103, 102)] = {'type': 'move', 'direction': 'backward'}
    E[(112, 505)] = {'type': 'move', 'direction': 'forward'}
    E[(505, 501)] = {'type': 'pivot', 'direction': 'turn_cw_90'}
    E[(111, 112)] = {'type': 'move', 'direction': 'forward'}
    E[(501, 400)] = E[(400, 401)] = {'type': 'move', 'direction': 'forward'}
    E[(401, 501)] = E[(501, 500)] = {'type': 'move', 'direction': 'backward'}
    check('E1 move -> move, same direction: via', c._is_via_hop(101, 102, 103))
    check('E2 the last tag of the command is never via', not c._is_via_hop(101, 102, None))
    check('E3 a tag where the direction flips is not via', not c._is_via_hop(102, 103, 102))
    check('E4 a tag the base pivots on is not via', not c._is_via_hop(112, 505, 501))
    check('E5 a 500-series tag is not via even between two moves', not c._is_via_hop(401, 501, 500))
    check('E6 the hop into the tag before a 500-series tag is via (112 before 505)',
          c._is_via_hop(111, 112, 505))
    check('E7 zone A lane tags are via', c._is_via_hop(501, 400, 401))
    c2, _ = H.make(-CAM, 0.0, 0.0, {101: (0.0, 0.0)}, off)
    c2.map_mgr.edges = dict(E)
    check('E8 disabled: nothing is via', not c2._is_via_hop(101, 102, 103))
    cfg = cfg_with(enabled=True); cfg['robot']['steer_mode'] = 'state_feedback'
    c3, _ = H.make(-CAM, 0.0, 0.0, {101: (0.0, 0.0)}, cfg)
    c3.map_mgr.edges = dict(E)
    check('E9 another steer_mode: nothing is via', not c3._is_via_hop(101, 102, 103))

    # ---- F. align band 0: the aim is still skipped, the align runs at every tag
    r = run_lane(ids, straight, cfg=cfg_with(enabled=True, align_skip_deg=0.0))
    check('F1 align_skip_deg 0: every via tag is aligned, none is aimed at rest',
          r['ok'] and seq(r['calls']) == exp0 and len(r['aims_at_rest']) == 1,
          f"{seq(r['calls'])} aims {len(r['aims_at_rest'])}")
    check('F2 ... and none is passed without stopping (the heading is never inside a 0 band)',
          not any('without stopping' in l for l in r['via_log'])
          and r['lens_err_mm'] < 5.0, f"{r['lens_err_mm']:.1f} mm")
    # ---- G. offset threshold 0: every via tag is aimed at rest (and then aligned)
    r = run_lane(ids, straight, cfg=cfg_with(enabled=True, aim_skip_offset_mm=0.0), noise=(0.3, 7))
    check('G1 aim_skip_offset_mm 0: the aim at rest runs at every tag',
          r['ok'] and len(r['aims_at_rest']) == 5, f"aims {len(r['aims_at_rest'])}")

    # ---- H. a stop request during a via hop ends the command
    c, p = H.make(-CAM, 0.0, 0.0, dict(straight), on)
    for a, b in zip(ids, ids[1:]):
        c.map_mgr.edges[(a, b)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': straight[t][0], 'y': 0.0, 'zone': 'A'}
    c.map_mgr.path = list(ids); c.last_known_tag = 101
    step = H.STEP_HOOK[0]

    def hook(dt):
        step(dt)
        if p.x + CAM > 0.95 and not c.stop_requested:
            c.preempt_stop_robot()
    H.STEP_HOOK[0] = hook
    ok = c.move_to_tag(106)
    for _ in range(40):
        Rate(20).sleep()
    x_rest = p.x
    for _ in range(20):
        Rate(20).sleep()
    check('H1 preempt during a via hop: move_to_tag returns False and the base is at rest',
          (not ok) and abs(p.x - x_rest) < 1e-6 and p.x + CAM < 1.05,
          f'lens x {p.x + CAM:.3f}')

    # ---- I. the hop after a passed via tag fails: the base must not keep rolling
    c, p = H.make(-CAM, 0.0, 0.0, dict(straight), on)
    for a, b in zip(ids, ids[1:]):
        c.map_mgr.edges[(a, b)] = {'type': 'move', 'direction': 'forward'}
    c.map_mgr.get_tag_info = lambda t: {'x': straight[t][0], 'y': 0.0, 'zone': 'A'}
    c.map_mgr.path = list(ids); c.last_known_tag = 101
    real_edge = c.map_mgr.get_edge
    asked = [0]

    def get_edge(a, b):
        if (a, b) == (102, 103):
            asked[0] += 1
            if asked[0] > 1:          # 1st: the via classification; then the hop itself
                return None
        return real_edge(a, b)
    c.map_mgr.get_edge = get_edge
    ok = c.move_to_tag(106)
    for _ in range(30):
        Rate(20).sleep()
    x_rest = p.x
    for _ in range(20):
        Rate(20).sleep()
    check('I1 the hop after a passed via tag fails (no edge): move_to_tag False and the base at rest',
          (not ok) and any('tag 102 passed without stopping' in l for l in LOG)
          and abs(p.x - x_rest) < 1e-9 and abs(p.last_v) < 1e-9,
          f'lens x {p.x + CAM:.3f} v {p.last_v:.4f}')

    print(f"\nsimulated 5-hop lane (4 via tags), every tag aimed + aligned -> stops removed -> "
          f"+ approach_speed {vs:.2f} and no stop:\n  forward {fwd_off:.0f} -> {fwd_slow:.0f} -> "
          f"{fwd_on:.0f} s, reverse {rev_off:.0f} -> {rev_slow:.0f} -> {rev_on:.0f} s")
    n = sum(1 for x in checks if not x)
    print(f'{len(checks) - n}/{len(checks)} checks passed')
    return 1 if n else 0


if __name__ == '__main__':
    sys.exit(main())
