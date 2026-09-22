# 외부 PC(Windows)에서 로봇 제어하기 — rosbridge 사용 안내

작성 2026-09-14. 대상: 로봇 PC 옆이 아니라 **Windows 노트북에서** 로봇에 명령을
보내고 상태를 보려는 운영자. ROS를 몰라도 되고, Windows에 ROS를 설치하지
않는다.

---

## 1. 구조

```
Windows PC (192.168.0.x) ─ 공유기(192.168.0.1) ─ 무선 ─ 피닉스 AP브릿지 ─ 로봇 PC
                                                   WLAN 192.168.0.20     192.168.1.100
                                                   LAN  192.168.1.10
                                          ws://192.168.0.20:9090 ──포트포워딩──▶ :9090
```

- 로봇 PC에서는 `rosbridge_websocket`이 ROS 안의 모든 토픽·서비스를
  **JSON/WebSocket(9090 포트)** 으로 열어 준다. 스택(`mobile_manipulator.launch`)을
  띄우면 같이 뜬다 (2026-09-14부터).
- 피닉스 AP브릿지는 192.168.1.x(로봇)와 192.168.0.x(사무실) 사이의 **NAT 라우터**다.
  그래서 Windows는 로봇 IP(192.168.1.100)가 아니라 **피닉스의 WLAN 주소
  192.168.0.20** 으로 접속하고, 피닉스가 9090 포트를 로봇 PC로 넘겨 준다.
  (ROS 고유 방식의 원격 연결은 이 NAT를 넘지 못한다. rosbridge를 쓰는 이유.)
- Windows 쪽은 `robot_cmd.py` 하나로 명령·상태·임의 토픽을 다룬다.

## 2. 두 PC를 재부팅했을 때의 절차

### 2-1. 로봇 PC

1. 전원을 켜면 navifra 서비스가 **roscore와 베이스 드라이버를 자동으로** 띄운다.
   `roscore`를 따로 실행하지 않는다.
2. 터미널에서 스택을 올린다. rosbridge가 함께 뜬다.
   ```bash
   source ~/mobile_manipulator_ws/devel/setup.bash
   roslaunch apriltag_nav mobile_manipulator.launch
   ```
3. 다른 터미널에서 rosbridge가 듣고 있는지 확인한다.
   ```bash
   ss -ltn | grep 9090        # 0.0.0.0:9090 LISTEN 이 보여야 함
   ```
   안 보이면 → 6장 "문제 해결".

⚠️ 스택을 **재부팅 없이** 다시 올리는 경우, 손으로 띄운 rosbridge
(`roslaunch rosbridge_server …`)가 남아 있으면 launch 쪽 rosbridge가 9090을
못 잡고 죽는다. 먼저 `pkill -f rosbridge_websocket` 한 뒤 launch 한다.
재부팅하면 이 문제는 없다.

### 2-2. 피닉스 AP브릿지

할 일 없음. 포트 포워딩(TCP 9090 → 192.168.1.100:9090, 22 → 192.168.1.100:22)은
기기에 저장돼 있다. 설정 화면은 로봇 PC에서 `https://192.168.1.10`, Windows에서
`https://192.168.0.20`.

### 2-3. Windows PC

1. 로봇과 **같은 공유기**에 붙어 있는지 확인 (`ipconfig` → IPv4가 192.168.0.x).
2. PowerShell에서 순서대로:
   ```powershell
   ping 192.168.0.20                            # 피닉스까지
   Test-NetConnection 192.168.0.20 -Port 9090   # TcpTestSucceeded : True
   ```
3. `robot_cmd.py`가 있는 폴더에서:
   ```powershell
   python robot_cmd.py topics                   # 토픽 목록이 쭉 나오면 연결 완료
   python robot_cmd.py state                    # 로봇 상태
   ```

`websocket-client`는 처음 한 번만 설치한다 (재부팅해도 남는다):
```powershell
python -m pip install websocket-client
```
(`pip`만 치면 "인식되지 않습니다"가 나오니 반드시 `python -m pip`.)

## 3. robot_cmd.py

위치: 저장소 `src/apriltag_nav/tools/rosbridge/robot_cmd.py`. Windows의 아무
폴더에 복사해서 쓴다. 접속 주소는 기본 `192.168.0.20`이고, 바꾸려면
`--host 주소`를 앞에 붙이거나 환경변수 `ROBOT_HOST`를 설정한다.

### 3-1. 명령 보내기 (대문자 명령어를 따옴표로)

```powershell
python robot_cmd.py "STATE"            # 상태 조회, 모션 없음 — 첫 테스트용
python robot_cmd.py "GOTO 105"         # 태그 105로 이동
python robot_cmd.py "TASK scan_pose_errorX_p000mm_standoff_050mm_height_652mm"
python robot_cmd.py "STOP"             # 소프트 정지
python robot_cmd.py "CHARGE"           # 500번 도킹 → 충전 시작
python robot_cmd.py "UNDOCK"           # 충전 중단 → 0.10 m 전진
python robot_cmd.py "RELOAD_TASKS"     # task/csv 다시 읽기
python robot_cmd.py "GOTO 105" 60      # 뒤의 숫자 = 상태를 지켜볼 시간(초), 기본 5
```

명령을 보낸 뒤 `/task_state`가 바뀔 때마다 한 줄씩 찍힌다:
```
sent: GOTO 105
state: IDLE       task: None    group: None  charge: working  battery: 68.3
state: MOVING     task: goto    ...
state: ARRIVED    ...
```
지켜보는 시간이 지나면 프로그램은 끝나지만 **로봇은 계속 동작한다**. 끝났는지
다시 보려면 `python robot_cmd.py state`.

TASK 이름은 `task/csv`의 파일명에서 나온다. 목록은
`python robot_cmd.py sub /task_list` 로 본다.

### 3-2. 상태 보기

```powershell
python robot_cmd.py state              # /task_state 변화를 10 s 동안 표시
python robot_cmd.py state 60           # 60 s 동안
```

### 3-3. 어떤 토픽·서비스가 있는지 알아보기

```powershell
python robot_cmd.py topics                        # 토픽 전체
python robot_cmd.py services                      # 서비스 전체
python robot_cmd.py type /bms/state               # 토픽의 메시지 타입
python robot_cmd.py fields sensor_msgs/BatteryState   # 타입의 필드 이름
```

### 3-4. 아무 토픽 읽기 / 쓰기, 서비스 호출, 파라미터

```powershell
python robot_cmd.py sub /bms/state                # 메시지 1개
python robot_cmd.py sub /odom 5                   # 메시지 5개
python robot_cmd.py pub /crevis/charging std_msgs/Bool '{"data": true}'
python robot_cmd.py pub /lifter/height_cmd std_msgs/Float32 '{"data": 150.0}'
python robot_cmd.py call /arm/move_home           # std_srvs/Trigger (인자 없음)
python robot_cmd.py call /lifter/home
python robot_cmd.py param /mobile_node/state_rate_hz
```

JSON 따옴표: **PowerShell**에서는 작은따옴표로 감싸면 그대로 넘어간다.
**cmd.exe**에서는 `"{\"data\": true}"` 처럼 써야 한다.

### 3-5. 내 파이썬 코드에서 쓰기

```python
from robot_cmd import connect, pub, sub, call
ws = connect('192.168.0.20')
print(sub(ws, '/bms/state')[0]['percentage'])
pub(ws, '/task_command', 'std_msgs/String', {"data": "GOTO 105"})
print(call(ws, '/arm/move_home'))
ws.close()
```

## 4. 자주 쓰는 토픽·서비스

| 용도 | 이름 | 타입 / 내용 |
|---|---|---|
| 명령 | `/task_command` | String: `TASK <name>`, `GOTO <tag>`, `STOP`, `STATE`, `CHARGE`, `UNDOCK`, `RELOAD_TASKS` |
| 로봇 상태 | `/task_state` | String(JSON): state, task, current_group, charge_phase, charging, battery_pct |
| task 목록 | `/task_list` | String(JSON): 이름, 모드, 태그, 점 수, 파일 |
| 팔 | `/arm/state` | 10 Hz, TCP 자세·관절·busy |
| 스캔 진행 | `/arm/scan_progress` | 점마다 start/move/done/result(Ra)/failed/finished |
| 베이스 | `/mobile/state` | busy, seq, result, last_known_tag, visible_tags |
| 수동 주행 | `/mobile/move_cmd` | String(JSON): `{"type":"move","distance_m":0.3,"speed":0.05}` / `{"type":"pivot","angle_deg":90}` |
| 리프트 | `/lifter/state`, `/lifter/height_cmd` (Float32 mm), `/lifter/home` `/lifter/stop` (srv) | |
| 팔 서비스 | `/arm/move_home`, `/arm/cancel` (Trigger) | |
| 베이스 서비스 | `/mobile/stop`, `/mobile/cancel`, `/mobile/clear_stop` (Trigger) | |
| 배터리 | `/bms/state` | BatteryState: `percentage` 0~1, `current` +면 충전 중, `power_supply_status` 1=CHARGING 2=DISCHARGING |
| 충전 릴레이 | `/crevis/charging` (쓰기, Bool), `/crevis/charge_port_on` (읽기) | |
| 비상정지 | `/safety/estop` | Bool, true = 걸림 |
| 태그 | `/front_cam/tag_detections` | 보이는 태그 ID·offset·각도 |
| 램프 | `/crevis/led/vision`, `/crevis/led/status_{red,green,blue}` | Bool |

## 5. 하면 안 되는 것

- **`/cmd_vel`에 발행 금지.** `mobile_node`가 유일한 발행자이고 중재가 없다.
  두 발행자가 있으면 베이스가 마지막 메시지를 따라 오락가락한다.
- **`/lift/*`에 쓰기 금지** (`/lift/position_cmd`, `/lift/home`, `/lift/command` …).
  드라이버 raw 토픽이라 소프트 리밋·원점 확인이 없다. 리프트는
  `/lifter/height_cmd`, `/lifter/home` 으로. 읽기(`/lift/position`, `/lift/homed`)는 된다.
- **이미지 토픽 `sub` 금지** (`/basler/image_raw`, `/front_cam/color/image_raw` …).
  한 장이 수십 MB JSON이라 연결이 막힌다.
- `/crevis/charging`을 직접 쓰면 `task_executor`의 충전 매니저는 그 사실을 모른다
  (램프·`/task_state`의 충전 표시가 실제와 어긋남). 운용은 `CHARGE`/`UNDOCK`.
- 9090 포트는 **사무실 내부망까지만**. 인증이 없어서 접속하는 누구나 명령을 보낼 수 있다.

## 6. 문제 해결

| 증상 | 확인 | 원인 / 조치 |
|---|---|---|
| `cannot connect … No route to host` / `timed out` | Windows `ping 192.168.0.20` | Windows가 다른 네트워크에 있음 |
| ping은 되는데 `Test-NetConnection … False` | 로봇 PC `ss -ltn \| grep 9090` | rosbridge 안 뜸 → 스택 launch (2-1). 떠 있는데도 False면 피닉스 포트 포워딩 확인 |
| 로봇 PC에서 9090이 안 보임 | `rosnode list \| grep rosbridge` | 수동 rosbridge와 충돌 → `pkill -f rosbridge_websocket` 후 스택 재launch |
| `sub /task_state` 가 아무것도 안 찍음 | `python robot_cmd.py sub /odom` | `/odom`은 되면 연결은 정상, `task_executor`가 안 떠 있는 것 |
| `"GOTO 105"` 보냈는데 `state:` 줄이 안 나옴 | `python robot_cmd.py state` | 명령은 갔음. 상태가 5 s 안에 안 바뀌었을 뿐 |
| `CHARGE` 후 `no charging current within 15s` | 로봇 PC `grep '\[Charge\]' $MM_WS/log/ros/latest/mobile_manipulator_system*.log` | 릴레이는 켜졌는데 전류가 없음. 충전기 표시등·접점 확인, `UNDOCK` → `CHARGE`로 재도킹 |
| `pip` 인식 안 됨 | | `python -m pip …` |

⚠️ **스택을 Ctrl-C로 끄면 충전 릴레이도 꺼진다** (`task_executor` 종료 루틴의
설계 동작). 충전 중에 스택을 재시작하면 충전이 멈추고, 다시 올려도 저절로
재개되지 않는다 — 다시 `CHARGE`를 보내야 한다. 2026-09-14에 이것 때문에
46 %→71 %에서 충전이 끊겼다.

## 7. 직접 프로그램을 짤 때 — rosbridge JSON 프로토콜 요약

WebSocket으로 JSON 한 줄씩 주고받는다. `robot_cmd.py`가 하는 일도 이 네 가지뿐이다.

| 하고 싶은 것 | 보내는 JSON |
|---|---|
| 토픽 발행 | `{"op":"advertise","topic":"/task_command","type":"std_msgs/String"}` 한 번, 0.5 s 뒤 `{"op":"publish","topic":"/task_command","msg":{"data":"GOTO 105"}}` |
| 토픽 구독 | `{"op":"subscribe","topic":"/bms/state"}` → 이후 `{"op":"publish","topic":"/bms/state","msg":{…}}` 가 계속 옴. `"throttle_rate":500` (ms) 로 줄일 수 있음 |
| 서비스 호출 | `{"op":"call_service","service":"/arm/move_home","args":{},"id":"1"}` → `{"op":"service_response","id":"1","result":true,"values":{…}}` |
| 목록 조회 | `call_service` 로 `/rosapi/topics`, `/rosapi/topic_type`, `/rosapi/message_details`, `/rosapi/services`, `/rosapi/get_param` |

advertise 직후 바로 publish 하면 첫 메시지가 버려진다 (ROS 쪽 구독자가 붙기
전). 0.5 s 기다린다.

브라우저에서 만들 때는 `roslib.js`, 파이썬에서 라이브러리를 쓰려면 `roslibpy`
(둘 다 같은 프로토콜). `robot_cmd.py`는 라이브러리 없이 `websocket-client`만 쓴다.
