#!/usr/bin/env python3
"""Offline check of task_executor's charging manager (2026-09-09).

    python3 src/apriltag_nav/tools/check_charging_manager.py

Loads the real task_executor with rospy / messages stubbed and the device
clients replaced by fakes that record calls; drives `_tick()` with a fake
battery. Exit status 1 on failure.
"""
import importlib.util
import os
import sys
import types

WS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
sys.path.insert(0, os.path.join(WS, 'src'))


class _Clock:
    t = 1000.0


CLK = _Clock()
LOG = []


def _log(*a, **k):
    try:
        msg = a[0] % a[1:] if len(a) > 1 else a[0]
    except Exception:
        msg = ' '.join(str(x) for x in a)
    LOG.append(str(msg))


class _Time:
    def __init__(self, s): self.s = float(s)
    @staticmethod
    def now(): return _Time(CLK.t)
    def to_sec(self): return self.s


def _sleep(s):
    CLK.t += float(s)


class _Rate:
    def __init__(self, hz): self.dt = 1.0 / hz
    def sleep(self): CLK.t += self.dt


rospy = types.ModuleType('rospy')
rospy.Time = _Time
rospy.Duration = lambda s: types.SimpleNamespace(s=float(s), to_sec=lambda: float(s))
rospy.Rate = _Rate
rospy.sleep = _sleep
rospy.get_time = lambda: CLK.t
rospy.is_shutdown = lambda: False
rospy.init_node = lambda *a, **k: None
rospy.get_param = lambda k, d=None: d
rospy.on_shutdown = lambda f: None
rospy.Publisher = lambda *a, **k: types.SimpleNamespace(publish=lambda m: None, get_num_connections=lambda: 0)
rospy.Subscriber = lambda *a, **k: None
rospy.ServiceProxy = lambda *a, **k: (lambda *x, **y: types.SimpleNamespace(success=True, message='ok'))
rospy.wait_for_service = lambda *a, **k: None
for n in ('loginfo', 'logwarn', 'logerr', 'logdebug'):
    setattr(rospy, n, _log)
    setattr(rospy, n + '_throttle', lambda period, *a, **k: _log(*a))
sys.modules['rospy'] = rospy


class _Msg:
    def __init__(self, *a, **k):
        self.data = a[0] if a else None


def _msgmod(name, *classes):
    m = types.ModuleType(name)
    for c in classes:
        setattr(m, c, type(c, (_Msg,), {}))
    sys.modules[name] = m


_msgmod('std_msgs'); _msgmod('std_msgs.msg', 'Bool', 'String', 'Int32', 'Int16', 'Float32')
_msgmod('sensor_msgs'); _msgmod('sensor_msgs.msg', 'BatteryState', 'Image', 'CameraInfo')
_msgmod('std_srvs'); _msgmod('std_srvs.srv', 'Trigger', 'TriggerResponse', 'SetBool', 'SetBoolResponse')
_msgmod('geometry_msgs'); _msgmod('geometry_msgs.msg', 'Twist')
_msgmod('nav_msgs'); _msgmod('nav_msgs.msg', 'Odometry')
_msgmod('robot_msgs'); _msgmod('robot_msgs.msg', 'Pose2DWithFlag', 'AprilTagDetectionArray', 'AprilTagDetection', 'ArmState')
_msgmod('robot_msgs.srv', 'PredictRa', 'CaptureImages')

spec = importlib.util.spec_from_file_location('task_executor', os.path.join(WS, 'scripts', 'task_executor.py'))
te = importlib.util.module_from_spec(spec)
spec.loader.exec_module(te)


# ---------------------------------------------------------------- fakes
class FakeMobile:
    """Arriving at the dock tag makes the charger contacts (unless dock_ok is
    False); any forward move breaks them."""
    def __init__(self): self.calls = []; self.fail_goto = False; self.dock_ok = True; self.battery = None
    def move_to_tag(self, tag):
        self.calls.append(('goto', int(tag))); CLK.t += 5.0
        if int(tag) == 500 and self.battery is not None:
            self.battery.docked = bool(self.dock_ok)
        return not self.fail_goto
    def drive_distance(self, d, speed=None):
        self.calls.append(('drive', round(float(d), 3))); CLK.t += 2.0
        if d > 0 and self.battery is not None:
            self.battery.docked = False
        return True
    def emergency_stop_robot(self): self.calls.append(('estop',))
    def preempt_stop_robot(self): self.calls.append(('preempt',))
    def clear_stop_flag(self): pass


class FakeArm:
    def __init__(self): self.calls = []
    def move_to_home(self): self.calls.append('home')
    def cancel(self): self.calls.append('cancel')
    def execute_scan_points(self, pts): self.calls.append(('scan', len(pts)))
    current_pose_msg = None


class FakeLift:
    def __init__(self): self.calls = []
    def home(self): self.calls.append('home'); return True, 'lift at origin'
    def goto_mm(self, mm): self.calls.append(('goto', mm)); return True, 'ok'


class FakeDevices:
    def __init__(self): self.battery_state = None; self._b = None
    @property
    def battery_percent(self): return self._b.pct if self._b else None
    def shutdown(self, **k): pass


class Battery:
    """Fake BMS + charger: current flows only after /crevis/charging true was sent AND the base is docked."""
    def __init__(self, ex, pct):
        self.pct = pct; self.relay = False; self.docked = True; self.ex = ex
        ex.devices._b = self
        ex.devices.set_charging = self.set_charging
        ex.devices.charging_by_bms = lambda min_a=0.5: self.charging()
        ex.devices.battery_low = lambda: self.pct < 20.0
        ex.devices.safe_to_move = lambda: (True, 'ok')
        ex.devices.take_estop_edge = lambda: False
        self.lamp = []
        ex.devices.set_status_color = lambda c: self.lamp.append(c)
    def set_charging(self, on):
        self.relay = bool(on); LOG.append(f'/crevis/charging <- {on}')
    def charging(self):
        return self.relay and self.docked


def make(pct, docked=True):
    CLK.t = 1000.0; LOG.clear()
    ex = te.MobileManipulatorTaskExecutor.__new__(te.MobileManipulatorTaskExecutor)
    # minimal init without ROS: reuse the real constructor pieces we need
    import yaml
    cfg = yaml.safe_load(open(os.path.join(WS, 'config', 'robot.yaml')))
    ex._navifra_cfg = cfg['navifra']
    ex._status_colors = ex._navifra_cfg['status_colors']
    ex._charge_cfg = ex._navifra_cfg['charging']
    ex._charge_enabled = True
    ex._charge_phase = 'unknown'; ex._charge_next_eval = 0.0; ex._lamp_key = None; ex._task_completed = False
    ex._state = te.MobileManipulatorState.IDLE
    ex.scan_done = False
    ex._current_task_name = None; ex._current_task = None
    ex._pending_task_name = None; ex._pending_task = None
    ex._stop_requested = False; ex._task_running = False
    ex._battery_warned = False
    ex._progress_index = 0; ex._progress_total = 0; ex._progress_tag = -1; ex._progress_note = None
    ex._lift_home_on_finish = True
    ex._debug_mode = False
    ex._task_state_pub = types.SimpleNamespace(publish=lambda m: None)
    import threading
    ex._scan_done_event = threading.Event()
    ex.devices = FakeDevices()
    ex.task_mgr = types.SimpleNamespace(START_TAG=500, scan_points={}, lift_heights={},
                                        get_task=lambda n: [{'tag': 106, 'scan': True}] if n == 'scan_test' else [],
                                        get_scan_points=lambda n, t: [{'x': 0}],
                                        get_lift_height=lambda n: None,
                                        build_goto_task=lambda t: [{'tag': int(t), 'scan': False}])
    ex.mobile, ex.arm, ex.lift = FakeMobile(), FakeArm(), FakeLift()
    b = Battery(ex, pct); b.docked = docked
    ex.mobile.battery = b
    return ex, b


checks = []


def check(name, cond, detail=''):
    checks.append(bool(cond))
    print(('PASS ' if cond else 'FAIL ') + name + (f'  [{detail}]' if detail else ''))


def ticks(ex, n):
    for _ in range(n):
        ex._tick(); CLK.t += 0.2


def main():
    # ---- 1. docked and charging at 57 %: lamp red, nothing queued
    ex, b = make(57.0); b.relay = True
    ticks(ex, 15)
    check('1 charging at 57%: phase charging, lamp red, no task', ex._charge_phase == 'charging' and b.lamp and b.lamp[-1] == 'red'
          and ex.mobile.calls == [], f'phase {ex._charge_phase} lamp {b.lamp[-1:]} calls {ex.mobile.calls}')

    # ---- 2. reaches 85 %: charging false, forward 0.10, phase full, lamp green
    b.pct = 85.2
    ticks(ex, 20)
    check('2 85%: /crevis/charging false, forward 0.10 m, phase full, lamp green',
          ex._charge_phase == 'full' and b.relay is False and ('drive', 0.1) in ex.mobile.calls and b.lamp[-1] == 'green',
          f'phase {ex._charge_phase} relay {b.relay} calls {ex.mobile.calls} lamp {b.lamp[-1:]}')
    n_drives = ex.mobile.calls.count(('drive', 0.1))
    ticks(ex, 30)
    check('3 stays full afterwards (no repeated undock, no return)', ex.mobile.calls.count(('drive', 0.1)) == n_drives and ('goto', 500) not in ex.mobile.calls)

    # ---- 4. user task running, battery drops to 29 %: preempt, lift home, goto 500, charging true, confirmed
    ex, b = make(60.0, docked=False); ex._charge_phase = 'full'
    # a scan task whose scan never finishes on its own; the preempt must break it
    ex._pending_task_name, ex._pending_task = 'scan_test', ex.task_mgr.get_task('scan_test')
    import threading
    def run_task():
        ex._tick()          # activates and runs scan_test (blocks in the scan wait)
    th = threading.Thread(target=run_task, daemon=True); th.start()
    for _ in range(30):
        if ex._task_running: break
        _sleep(0.05); __import__('time').sleep(0.01)
    b.pct = ex._charge_cfg['return_pct'] - 1.0
    # the manager runs from the main loop while the task thread blocks
    for _ in range(40):
        ex._charge_tick(); CLK.t += 0.3; __import__('time').sleep(0.01)
        if ex._stop_requested: break
    check('4a below return_pct during a task: task preempted for battery_return', ex._stop_requested and ex._pending_task_name == 'battery_return',
          f'stop {ex._stop_requested} pending {ex._pending_task_name}')
    th.join(timeout=5.0)
    b.docked = True
    # run the queued return task to completion
    ticks(ex, 5)
    check('4b return task: lift home -> goto 500 -> /crevis/charging true -> confirmed charging',
          ex.lift.calls == ['home'] and ('goto', 500) in ex.mobile.calls and b.relay is True and ex._charge_phase == 'charging',
          f'lift {ex.lift.calls} mobile {ex.mobile.calls} relay {b.relay} phase {ex._charge_phase}')
    check('4c lamp red while charging', b.lamp[-1] == 'red', str(b.lamp[-2:]))

    # ---- 5. user task completes normally at 60 %: return after task
    ex, b = make(60.0, docked=False); ex._charge_phase = 'full'
    ex.task_mgr.get_task = lambda n: [{'tag': 106, 'scan': False}]
    ex._pending_task_name, ex._pending_task = 'goto_106', [{'tag': 106, 'scan': False}]
    ticks(ex, 3)
    b.docked = True
    ticks(ex, 6)
    check('5 completed task -> battery_return queued and run -> charging',
          ex.mobile.calls[:1] == [('goto', 106)] and ('goto', 500) in ex.mobile.calls and ex._charge_phase == 'charging' and b.relay,
          f'{ex.mobile.calls} phase {ex._charge_phase}')

    # ---- 6. return_after_task off: no return
    ex, b = make(60.0, docked=False); ex._charge_phase = 'full'
    ex._charge_cfg = dict(ex._charge_cfg, return_after_task=False)
    ex._pending_task_name, ex._pending_task = 'goto_106', [{'tag': 106, 'scan': False}]
    ticks(ex, 8)
    check('6 return_after_task false: stays where the task ended', ex.mobile.calls == [('goto', 106)] and not b.relay, str(ex.mobile.calls))

    # ---- 7. STOP disarms the auto-return; a new task re-arms it
    ex, b = make(15.0, docked=False); ex._charge_phase = 'full'
    ex._command_cb(types.SimpleNamespace(data='STOP'))
    ticks(ex, 15)
    check('7a after STOP at 15%: no auto-return', ex._charge_phase == 'stopped' and ('goto', 500) not in ex.mobile.calls, f'{ex._charge_phase} {ex.mobile.calls}')
    ex._pending_task_name, ex._pending_task = 'goto_106', [{'tag': 106, 'scan': False}]
    ticks(ex, 3); b.docked = True; ticks(ex, 6)
    check('7b a user task re-arms it: after the task, return + charge', ('goto', 500) in ex.mobile.calls and ex._charge_phase == 'charging', f'{ex.mobile.calls} {ex._charge_phase}')

    # ---- 8. docking produces no current: ERROR, dock_failed, no retry
    ex, b = make(15.0, docked=False); ex._charge_phase = 'working'
    ex.mobile.dock_ok = False   # contacts never made
    ticks(ex, 3)
    n_goto = ex.mobile.calls.count(('goto', 500))
    check('8a no current after docking: dock_failed, error logged, relay left true for the operator to see',
          ex._charge_phase == 'dock_failed' and n_goto == 1 and any('no charging current' in l for l in LOG),
          f'phase {ex._charge_phase} gotos {n_goto}')
    ticks(ex, 30)
    check('8b not retried', ex.mobile.calls.count(('goto', 500)) == 1)

    # ---- 9. manager disabled: nothing happens at 25 %
    ex, b = make(15.0, docked=False); ex._charge_enabled = False
    ticks(ex, 15)
    check('9 disabled: no motion, no relay', ex.mobile.calls == [] and not b.relay)

    n = sum(1 for c in checks if not c)
    print(f'\n{len(checks) - n}/{len(checks)} checks passed')
    return 1 if n else 0


if __name__ == '__main__':
    sys.exit(main())
