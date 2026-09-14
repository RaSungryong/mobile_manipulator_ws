#!/usr/bin/env python3
"""robot_cmd.py — talk to the robot from a PC WITHOUT ROS, over rosbridge.

Runs on the Windows operator PC (python.org Python 3.x). Needs only:
    python -m pip install websocket-client

The robot PC runs rosbridge_websocket on port 9090 (started by
mobile_manipulator.launch since 2026-09-14). The Windows PC reaches it as
ws://192.168.0.20:9090 — the Phoenix Contact AP bridge's WLAN address,
port-forwarded to the robot PC's 192.168.1.100:9090.

USAGE
    python robot_cmd.py "GOTO 105"                       task command (TASK/GOTO/STOP/STATE/CHARGE/UNDOCK/RELOAD_TASKS)
    python robot_cmd.py state [seconds]                  watch /task_state
    python robot_cmd.py topics | services                list what exists
    python robot_cmd.py type /bms/state                  message type of a topic
    python robot_cmd.py fields sensor_msgs/BatteryState  field names of a type
    python robot_cmd.py sub /bms/state [n]               print n messages (default 1)
    python robot_cmd.py pub /crevis/charging std_msgs/Bool "{\"data\": true}"
    python robot_cmd.py call /arm/move_home ['{"json":"args"}']
    python robot_cmd.py param /mobile_node/state_rate_hz
    python robot_cmd.py --host 127.0.0.1 topics          any command, other address

Set ROBOT_HOST in the environment to change the default address.

RULES (same as for code on the robot PC — see CLAUDE.md):
  * never publish /cmd_vel (mobile_node is the only publisher, no arbitration)
  * never publish /lift/* (lifter_node is the only writer; no soft limits below it)
    — use /lifter/height_cmd and the /lifter/* services instead
  * do not `sub` image topics (one Basler frame is ~27 MB of JSON)
  * /crevis/charging written directly bypasses task_executor's charging
    manager; for operation use the CHARGE / UNDOCK commands
"""
import json
import os
import sys
import time

try:
    import websocket
except ImportError:
    sys.exit("websocket-client is missing:  python -m pip install websocket-client")

DEFAULT_HOST = os.environ.get('ROBOT_HOST', '192.168.0.20')
PORT = 9090
TASK_WORDS = ('TASK', 'GOTO', 'STOP', 'STATE', 'CHARGE', 'UNDOCK', 'RELOAD_TASKS',
              'TEST_POSE', 'EXEC', 'EVAL')

_id = 0


# ----------------------------------------------------------------- transport
def connect(host=DEFAULT_HOST, port=PORT, timeout=10):
    try:
        return websocket.create_connection(f'ws://{host}:{port}', timeout=timeout)
    except Exception as e:
        sys.exit(f"cannot connect to ws://{host}:{port}: {e}\n"
                 f"  1) is the robot stack (or rosbridge) running on the robot PC?\n"
                 f"  2) ping {host}  /  Test-NetConnection {host} -Port {port}")


def _recv(ws):
    """One JSON message, or None on a quiet socket (timeout)."""
    try:
        return json.loads(ws.recv())
    except websocket.WebSocketTimeoutException:
        return None


def call(ws, service, args=None):
    """Call a ROS service; returns its response values (dict)."""
    global _id
    _id += 1
    rid = f'c{_id}'
    ws.send(json.dumps({"op": "call_service", "service": service,
                        "args": args or {}, "id": rid}))
    while True:
        r = _recv(ws)
        if r is None:
            continue
        if r.get("op") == "service_response" and r.get("id") == rid:
            if not r.get("result", True):
                raise RuntimeError(f"{service} failed: {r.get('values')}")
            return r["values"]


def pub(ws, topic, msg_type, msg):
    """Publish one message. advertise first, then a short pause so the ROS-side
    subscribers have attached — a message sent before that is dropped."""
    ws.send(json.dumps({"op": "advertise", "topic": topic, "type": msg_type}))
    time.sleep(0.5)
    ws.send(json.dumps({"op": "publish", "topic": topic, "msg": msg}))
    time.sleep(0.2)


def sub(ws, topic, n=1, seconds=10.0, on_msg=None):
    """Collect up to n messages from topic within `seconds`. If on_msg is given
    it is called for each message and nothing is collected."""
    ws.send(json.dumps({"op": "subscribe", "topic": topic}))
    out, end = [], time.time() + seconds
    try:
        while len(out) < n and time.time() < end:
            r = _recv(ws)
            if r is None or r.get("op") != "publish" or r.get("topic") != topic:
                continue
            if on_msg:
                on_msg(r["msg"])
            else:
                out.append(r["msg"])
    finally:
        ws.send(json.dumps({"op": "unsubscribe", "topic": topic}))
    return out


# ----------------------------------------------------------------- commands
def task_command(ws, text):
    pub(ws, '/task_command', 'std_msgs/String', {"data": text})
    print(f"sent: {text}")


def watch_state(ws, seconds=5.0):
    """Print /task_state as it changes (it is latched + published on change)."""
    def show(m):
        s = json.loads(m["data"])
        print(f"state: {s.get('state'):<10} task: {str(s.get('task')):<20} "
              f"group: {s.get('current_group')}  charge: {s.get('charge_phase')}  "
              f"battery: {s.get('battery_pct')}")
    sub(ws, '/task_state', n=10**9, seconds=seconds, on_msg=show)


def main(argv):
    host = DEFAULT_HOST
    if argv[:1] == ['--host']:
        host, argv = argv[1], argv[2:]
    if not argv:
        print(__doc__)
        return 2
    ws = connect(host)
    a = argv
    try:
        word = a[0].split()[0] if a[0].strip() else ''
        if word in TASK_WORDS:                       # UPPERCASE as typed: python robot_cmd.py "GOTO 105"
            task_command(ws, a[0])
            watch_state(ws, float(a[1]) if len(a) > 1 else 5.0)
        elif a[0] == 'state':
            watch_state(ws, float(a[1]) if len(a) > 1 else 10.0)
        elif a[0] == 'topics':
            print("\n".join(sorted(call(ws, '/rosapi/topics')['topics'])))
        elif a[0] == 'services':
            print("\n".join(sorted(call(ws, '/rosapi/services')['services'])))
        elif a[0] == 'type':
            print(call(ws, '/rosapi/topic_type', {"topic": a[1]})['type'])
        elif a[0] == 'fields':
            for t in call(ws, '/rosapi/message_details', {"type": a[1]})['typedefs']:
                print(t['type'], ':', ', '.join(
                    f"{n} ({ty})" for n, ty in zip(t['fieldnames'], t['fieldtypes'])))
        elif a[0] == 'sub':
            n = int(a[2]) if len(a) > 2 else 1
            msgs = sub(ws, a[1], n)
            if not msgs:
                print(f"(nothing on {a[1]} within 10 s — no publisher, or the topic name is wrong; try: topics)")
            for m in msgs:
                print(json.dumps(m))
        elif a[0] == 'pub':
            pub(ws, a[1], a[2], json.loads(a[3]))
            print(f"published {a[1]} {a[3]}")
        elif a[0] == 'call':
            print(call(ws, a[1], json.loads(a[2]) if len(a) > 2 else None))
        elif a[0] == 'param':
            print(call(ws, '/rosapi/get_param', {"name": a[1]})['value'])
        else:
            print(f"unknown command: {a[0]}\n{__doc__}")
            return 2
    finally:
        ws.close()
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
