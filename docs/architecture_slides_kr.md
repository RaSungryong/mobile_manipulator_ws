# 모바일 매니퓰레이터 시스템 구조 — 발표 자료

> 작성 2026-08-11. 코드 기준 스냅샷이며, 구조가 바뀌면 이 문서도 같이 갱신해야 한다.
> 슬라이드 1장 = 아래 `## Slide N` 하나. 다이어그램은 그대로 옮겨 쓰거나 도형으로 다시 그리면 된다.

---

## Slide 1 — 한 장 요약

**두 개의 워크스페이스가 역할로 나뉘어 있다.**

| | `~/navifra` | `~/…/mobile_manipulator_ws` |
|---|---|---|
| 역할 | **하드웨어 드라이버 계층** | **응용 / 작업 계층** |
| 제공자 | Navifra (KU Polishing Robot Driver v0.16) | 자체 개발 |
| 형태 | **바이너리 install 공간 (`src/` 없음)** | catkin 소스 워크스페이스 |
| 실행 | systemd `navifra-robot.service` (부팅 시 자동) | `roslaunch apriltag_nav mobile_manipulator.launch` (수동) |
| roscore | **소유** | 붙기만 함 |
| 아는 것 | 모터·리프트·BMS·IO 의 전기적 프로토콜 | 태그, 스캔 순서, 작업 정의 |
| 모르는 것 | "작업(task)"이라는 개념 자체 | RS485 / CANopen / Modbus 프레임 |

경계는 **ROS 토픽**이다. 위쪽은 `/lift/position_cmd` 를 쏘면 되고,
아래쪽은 그게 왜 필요한지 알 필요가 없다.

---

## Slide 2 — navifra 워크스페이스 구조

```
~/navifra/                          ← 배포된 바이너리. 소스 없음
├── run_robot.sh                    ← 진입점. ROS + install 소싱, param.yaml 생성
├── navifra-robot.service           ← systemd 유닛 (After/Requires = caninit.service)
├── param.yaml                      ← ★ 현장 튜닝값. 여기만 사람이 편집한다
├── param.default.yaml              ← 최신 템플릿 (신규 항목 확인용)
└── install/
    ├── lib/                        ← 실행 바이너리 + libcan_interface.so
    └── share/
        ├── motor_driver/           ← robot.launch (최상위), bringup.launch
        ├── lift_driver/
        ├── bms_driver/
        ├── crevis_io_driver/
        ├── safety_io_driver/
        └── can_interface/          ← 공용 CAN 라이브러리
```

**패키지 6개 = 물리 장치 6종에 1:1 대응**

| 패키지 | 담당 하드웨어 | 물리 인터페이스 |
|---|---|---|
| `motor_driver` | 주행 모터 2개 + `base_controller` | CAN 500K (CANopen) |
| `lift_driver` | MDROBOT DC 리프트 | RS485 `/dev/ttyS0` |
| `bms_driver` | Daly BMS | CAN **250K** (별도 버스) |
| `crevis_io_driver` | Crevis GN-9289 I/O (LED·충전) | Modbus/TCP `192.168.100.104` |
| `safety_io_driver` | PILZ PNOZmulti 2 안전 PLC | Modbus/TCP :502 |
| `can_interface` | 위 CAN 노드들이 공유 | — |

**설정이 덮이는 순서 (중요)**

```
install/share/<pkg>/config/*.yaml     ← 패키지 기본값 (배포본, 건드리지 않음)
              ↓  robot.launch 가 include 를 먼저 실행
        ~/navifra/param.yaml          ← include 이후에 rosparam load → ★ 항상 이긴다
```
현장에서는 `param.yaml` 만 고치고 드라이버를 재시작한다.
최상위 키는 노드 이름과 정확히 일치해야 한다: `base_controller` / `motor_driver_node` / `lift_driver`.

---

## Slide 3 — navifra 가 노출하는 ROS 인터페이스

```
            ┌──────────────── navifra 드라이버 (roscore 소유) ────────────────┐
  구동      │  /cmd_vel      (sub, Twist)          → 모터 rpm 변환          │
            │  /odom         (pub)                                          │
            │  /motor/cmd, /motor/alarm                                     │
            ├───────────────────────────────────────────────────────────────┤
  리프트    │  [명령]  /lift/command          (String: up/down/stop …)      │
            │          /lift/home             (Bool  : 하부 리미트 원점화)   │
            │          /lift/position_cmd     (절대 카운트)                  │
            │          /lift/inc_position_cmd (상대 카운트)                  │
            │          /lift/velocity_cmd, /lift/reset                       │
            │  [상태]  /lift/position  /lift/status  /lift/homed             │
            │          /lift/error     /lift/alarm                           │
            ├───────────────────────────────────────────────────────────────┤
  배터리    │  /bms/state  /bms/soc                                          │
  I/O       │  /crevis/di  /crevis/led/*  /crevis/charging  /crevis/connected│
  안전      │  /safety/estop  /safety/input/*  /safety/output/*             │
            └───────────────────────────────────────────────────────────────┘
```

> 리프트 핵심 물성 (발표 시 언급용)
> · 스트로크 0 – 6900 count = 343 mm (2026-08-14 재실측: count 6897 에서 343.2 mm) · 전 구간 약 27.8 s
> · 매니퓰레이터 베이스 높이: 원점 652 mm → 최상단 약 995 mm (지면 기준)
> · joint 모드 스캔은 **150 mm(3014 count, 약 12.1 s) 상승 후 작업**, 끝나면 팔 홈 자세 → 리프트 원점복귀
> · **위치는 증분 홀카운트** → 재부팅하면 원점 소실 → `/lift/home` 필수
> · 백래시 존재 → 같은 카운트라도 "올라와서 도달"과 "내려와서 도달"의 실제 높이가 다르다

---

## Slide 4 — mobile_manipulator_ws 워크스페이스 구조

```
mobile_manipulator_ws/
└── src/
    ├── apriltag_nav/       ★ 자체 개발 본체
    ├── robot_msgs/           커스텀 msg/srv (Pose2DWithFlag, CaptureImages.srv …)
    ├── fairino_sdk/          Fairino FR10v6 파이썬 SDK (벤더 제공)
    ├── frcobot_ros/          Fairino ROS 패키지 (벤더 제공)
    ├── orbbec_camera/        Femto Bolt 드라이버 (벤더)
    ├── realsense-ros/        D405 / D435 드라이버 (벤더)
    └── path_tag_locator/     AprilTag 경로 유틸
```

**`apriltag_nav` 내부 — "노드"와 "라이브러리"를 분리했다**

```
apriltag_nav/
├── scripts/          ← 실행 노드 8개 (rosrun 대상)
│   ├── task_executor.py       ★ 총괄 상태기계. 유일한 지휘자
│   ├── mobile_node.py         ★ 주행 단독 소유자
│   ├── lifter_node.py         ★ 리프트 단독 소유자
│   ├── arm_node.py            ★ 로봇팔 단독 소유자
│   ├── robot_camera_node.py     카메라 3대 소유·중계
│   ├── basler_camera_node.py    Basler 산업카메라
│   ├── keyence_dlen1_node.py    Keyence 변위센서
│   └── camera_viewer_node.py    디버그 뷰어
│
└── src/apriltag_nav/ ← import 되는 순수 라이브러리 (노드 아님)
    ├── task_manager.py      task 정의 CSV 로딩/검증
    ├── map_manager.py       태그 좌표 맵
    ├── mobile_controller.py 주행 제어 본체 (→ /cmd_vel)
    ├── navifra_devices.py   ★ navifra 토픽을 감싸는 유일한 래퍼
    ├── mobile_client.py     mobile_node 원격 호출 대리자
    ├── arm_client.py        arm_node 원격 호출 대리자
    ├── lift_client.py       lifter_node 원격 호출 대리자
    ├── arm_controller.py    Fairino SDK 모션 제어 본체
    ├── arm_transform.py     world → arm-base 좌표 변환 (순수 기하)
    ├── scan_pipeline.py     촬영 + ONNX Ra 추론 + /scan/* 발행
    ├── scan_results.py      결과 CSV 증분 저장
    └── paths.py / utils.py
```

**설계 규칙 — 장치 하나에 주인 노드 하나**

| 장치 | 유일한 소유자 | 다른 노드의 접근 방법 |
|---|---|---|
| 주행 | `mobile_node` | `MobileClient` → `/mobile/*` |
| 리프트 | `lifter_node` | `LiftClient` → `/lifter/*` |
| 로봇팔 | `arm_node` | `ArmClient` → `/arm/*` |
| 카메라 3대 | `robot_camera_node` | `/camera/set_active`, `/camera/capture` |

읽기(상태 구독)는 누구에게나 열려 있다. **막는 것은 쓰기뿐이다.**

> 2026-08-11 에 주행도 이 표에 들어왔다. 그 전까지 `task_executor` 가
> `MobileController` 를 직접 들고 `/cmd_vel` 을 쐈고, 주행만 주인 노드가 없는
> 비대칭이 남아 있었다. 아래 슬라이드 8 참고.

---

## Slide 5 — 명령은 어떻게 들어오는가

**입구는 단 하나: `/task_command` (std_msgs/String)**

```bash
rostopic pub -1 /task_command std_msgs/String "data: 'TASK scan_joints_line1'"
```

| 명령 | 의미 |
|---|---|
| `TASK <이름>` | CSV 에 정의된 작업 실행 |
| `TASK go_home` | 시작 태그로 복귀 (**이것도 그냥 하나의 task 다**) |
| `GOTO <태그번호>` | 해당 태그로 단순 이동 |
| `STOP` | 즉시 중단 |
| `STATE` | 현재 상태 조회 |
| `TEST_POSE` / `EXEC` / `EVAL` | 디버그용 |

**작업은 조립 가능한 블록이다 (설계상 표준)**

작업이 끝나도 로봇은 **그 자리에 선다.** 복귀는 별개의 `go_home` 작업이다.
그래서 `scan → scan → go_home` 처럼 임의 순서로 큐잉할 수 있고,
이것이 다음 슬라이드의 GUI 블록 코딩이 성립하는 전제다.

```
TASK scan_line1  →  TASK scan_line2  →  TASK go_home
   (끝나면 IDLE)      (끝나면 IDLE)       (끝나면 IDLE)
```

---

## Slide 6 — 명령 한 줄이 하드웨어까지 내려가는 전 경로

```
 사용자 / (향후) GUI
        │  rostopic pub  /task_command  "TASK scan_joints_line1"
        ▼
┌───────────────────────────────────────────────────────────────┐
│  task_executor.py   ── 상태기계: IDLE→MOVING→ARRIVED→SCANNING │
│                        →SCAN_DONE→(ERROR)                     │
│  TaskManager 가 읽은 CSV 한 줄 = "태그 105 로 가서, 리프트를   │
│  150 mm 올리고, 스캔점 169개를 찍어라"                          │
└──┬───────────────┬────────────────┬───────────────────────────┘
   │ MobileClient  │ LiftClient     │ ArmClient      ← 전부 대리자
   ▼               ▼                ▼
 /mobile/goto_tag   /lifter/height_cmd   /arm/scan_command
 /mobile/stop(srv)  /lifter/home(srv)    /arm/move_home(srv)
 /mobile/state(구독) /lifter/state(구독)  /arm/status(구독)
   │               │                     │
   ▼               ▼                     ▼
┌──────────────┐  ┌──────────────┐  ┌───────────────────┐
│ mobile_node  │  │ lifter_node  │  │    arm_node       │
│ Pure Pursuit │  │ 모션락·클램프│  │  ArmController    │
│ S-커브 가감속│  │ 원점복귀     │  │  + Keyence 보정   │
│ 비전정지 판단│  │ 안전검사     │  │  + scan_pipeline  │
└──────┬───────┘  └──────┬───────┘  └─────────┬─────────┘
       │ /cmd_vel        │ NavifraDevices     │ Fairino SDK (TCP, ROS 아님)
       ▼                 ▼                    ▼
       │          /lift/position_cmd    Fairino FR10v6 컨트롤러
       │          /lift/home
       ▼                 ▼
╔══════════════════════════════════════════════════════════════╗
║  navifra 드라이버 (systemd, roscore 소유)                     ║
║   motor_driver ─CAN500K→ 주행모터   lift_driver ─RS485→ 리프트 ║
║   bms_driver ─CAN250K→ BMS   crevis/safety ─Modbus/TCP→ IO/PLC ║
╚══════════════════════════════════════════════════════════════╝
```

**계층은 3단이다.** 지휘(task_executor) / 장치 소유(각 노드) / 하드웨어(navifra).
각 계층은 **바로 아래 한 층만** 안다.

세 갈래가 완전히 같은 모양이라는 점이 핵심이다. `task_executor` 는 어느
장치든 `<Device>Client` 만 호출하고, 그 아래에서 프로세스가 나뉘는지 여부를
모른다. 그래서 주행을 `mobile_node` 로 떼어낼 때 `task_executor` 의 호출부는
**한 줄도 바뀌지 않았다** (`self.mobile.move_to_tag(tag_id)` 그대로).

---

## Slide 7 — 통신 방식을 무엇으로 골랐나 (그리고 왜)

| 성격 | 고른 방식 | 예 |
|---|---|---|
| 짧고 즉답이 있음 | **서비스** | `/lifter/home`, `/mobile/stop`, `/arm/move_home` (Trigger) |
| 길고 중간에 멈춰야 함 | **토픽 + 상태 폴링** | `/lifter/height_cmd` → `/lifter/state` 의 `busy` 감시<br>`/mobile/goto_tag` → `/mobile/state` 의 `seq` 감시 |
| 계속 흐르는 상태 | **토픽** | `/lift/position`, `/arm/status`, `/safety/estop` |
| 연속 제어 | **토픽** | `/cmd_vel` (발행자는 `mobile_node` 하나뿐) |

⚠️ 리프트와 주행은 같은 토픽+폴링 방식이지만 **완료를 판정하는 방법이 다르다.**
리프트는 "목표 높이에 도달했는가"라는 측정 가능한 종료 상태가 있다. 주행에는
그런 게 없다 — 도착 판정 자체가 `MobileController` 안에 있다. 그래서
`mobile_node` 는 끝난 이동마다 `seq` 를 1 올리고 결과를 실어 두고,
`MobileClient` 는 위치가 아니라 **`seq` 가 올라가기를** 기다린다.

**판단 기준**: ① 얼마나 오래 걸리나 ② 중간에 취소해야 하나
③ 돌려줄 "결과"가 있나 ④ 상대 노드가 멈췄을 때 내가 어떻게 되나

⚠️ 정직하게 덧붙일 점: 리프트 이동을 토픽+폴링으로 한 이유로 흔히
"서비스는 취소가 안 되니까"를 들지만 **그건 틀렸다.** 하드웨어 정지 경로는
양쪽 다 `/lifter/stop` 을 별도 스레드에서 부르는 것으로 동일하다.
실제로 남는 차이는 **rospy 서비스 호출에는 in-flight 타임아웃이 없다**는 것뿐이다
(상대 노드가 굳으면 호출자가 영원히 블록됨. 폴링 루프는 자기 `move_timeout_s` 를 가진다).
장기적으로 **정답은 `actionlib`** 이다 — 길고, 취소 가능하고, 진행률이 있는 동작을 위해 ROS1 이 만든 원시타입.

---

## Slide 8 — 모듈화, 정직한 평가

### 잘 되어 있는 것

- **장치 1개 = 주인 노드 1개.** `/lift/*` 쓰기는 `lifter_node` 만, Fairino SDK 는 `arm_node` 만, `/cmd_vel` 은 `mobile_node` 만 잡는다. 코드 전역 grep 으로 확인됨.
- **클라이언트 대리자 패턴.** `MobileClient` / `ArmClient` / `LiftClient` 가 원격 호출을 감싸서, `task_executor` 의 호출부는 "같은 프로세스 안에 있던 시절"과 문자 그대로 동일하다. 구현을 갈아끼워도 지휘자는 안 바뀐다.
- **노드 / 라이브러리 분리.** `scripts/` 는 ROS 배선만, `src/apriltag_nav/` 는 순수 로직. 그래서 ROS 마스터 없이 스텁만으로 오프라인 테스트가 돌아간다 (실제로 이번 리프트 변경도 그렇게 검증했다).
- **작업의 조립 가능성.** 어떤 task 도 자기 마음대로 복귀하지 않는다 → 임의 순서 조합 가능.
- **워크스페이스 경계가 토픽이다.** navifra 를 새 버전 바이너리로 통째로 교체해도 위층은 무영향.

### 아직 깨끗하지 않은 것 (숨기지 말 것)

| # | 문제 | 영향 |
|---|---|---|
| ~~1~~ | ~~**주행만 주인 노드가 없다.**~~ → **2026-08-11 해소.** `mobile_node` + `MobileClient` 를 추가해 팔·리프트와 같은 모양이 됐다 | 이제 세 장치가 완전 대칭 |
| 2 | **`arm_transform.py` 의 `arm_base_z` 가 상수다.** 리프트를 올려도 world→arm 변환이 따라가지 않는다. | 2026-08-13 기준 **설계로 격리됨**: 리프트를 올리는 task 는 전부 joint 모드(=이 변환을 안 탄다)이고, pose 모드 CSV 에는 `lift_height` 가 없다. 둘을 섞는 순간 다시 위험해진다 (`docs/lift_arm_base_z_analysis.md`) |
| 3 | ~~`mm_calibrated: false`~~ → 2026-08-13 실측, 2026-08-14 재실측 완료 (343.2 mm / 6897 count), 이제 `true` | 해소됨 |
| 4 | 세 Client 모두 `actionlib` 대신 손으로 만든 토픽+폴링을 쓴다 | ROS1 이 이 용도로 만든 원시타입을 안 쓰고 같은 패턴을 세 번 반복했다. 네 번째가 필요해지면 셋 다 바꿀 것 |
| 5 | `navifra-robot.service` 가 `User=abc`, `HOME=/home/abc` 고정 | 배포 계정 가정이 박혀 있음. 다른 계정 로봇에 그대로 못 쓴다 |
| 6 | 워크스페이스가 둘이라 `roscore` 소유·기동 순서가 암묵적 계약 | navifra 가 먼저 떠 있어야 함 |
| 7 | ⚠️ **`/lifter/*` (우리) 와 `/lift/*` (navifra) 가 한 글자 차이** | 현장에서 `rostopic pub` 을 칠 때 헷갈릴 위험. `/lift/*` 는 원시 드라이버라 안전장치가 없다 |
| ~~8~~ | ~~**front_cam 이 광축 기준 −90° 회전됐는데 `mobile_controller.py` 가 안 따라갔다**~~ → **2026-08-13 당일 해소** | 좌표계 확정: `fc.x = +mb.x`(화면 오른쪽 = 전진), `fc.y = -mb.y`(화면 아래 = 우측/벽). 세 곳 모두 수정 — `lateral = tag['x']` → `tag['y']`(부호는 그대로), `center_y` 정지조건 → `center_x` + **부등호 반전** + 키 이름/부호 변경(`center_x_stop_offset: +50.0`), 코너각은 공용 `tag_edge_angle_deg()` 로 **−90° 보정**. 오프라인 38개 검증 통과 — 특히 *회전 이전* 카메라와 *구* 로직을 재구성해 정지거리를 비교: 전진 0.567 m / 후진 0.561 m 로 **1e-9 이내 동일**(재튜닝이 아니라 좌표 변환임을 증명). ⚠️ `path_tag_locator` 의 사본은 미수정 — 별도 launch 이므로 메인 스택엔 영향 없지만 그 launch 는 돌리지 말 것 |
| 9 | 🛑 **joint 모드 스캔이 100 mm 어긋나 있다** (2026-08-13) | 차체가 200 mm 넓어져 중심이 벽에서 100 mm 멀어졌고, 팔은 벽쪽으로 100 mm 옮겼다. pose 모드는 `arm_body_offset_y` 로 자동 보정되지만 joint 모드 CSV 는 변환을 안 타서 못 따라간다. **태그를 100 mm 옮기고 `map.yaml` 을 갱신하면 해소** (사용자 예정) |

**한 줄 결론**: 역할 분담과 모듈 경계는 명확하고 실제로 코드가 그걸 지키고 있다.
2026-08-11 로 세 장치(주행·리프트·팔)가 전부 같은 3층 구조를 갖게 됐다.
구조적 최대 부채는 여전히 **리프트–팔 좌표 결합(#2)** 이지만,
2026-08-13 현재 **당장 로봇을 돌리기 전에 남은 것은 #9 하나**다 (#8 은 같은 날 해소).
#9 는 코드가 아니라 **태그를 물리적으로 옮기고 `map.yaml` 을 다시 재는 작업**이라
그때까지 joint 모드 스캔 결과는 신뢰하면 안 된다.

---

## Slide 9 — 향후 계획 ①: 두 워크스페이스 통합

### 왜

- 워크스페이스가 둘이면 소싱 순서·`roscore` 소유·기동 순서가 전부 암묵적 계약이 된다.
- navifra 쪽은 `src/` 가 없는 바이너리라 디버깅 시 소스를 못 본다.
- 배포가 두 갈래(systemd + 수동 roslaunch)라 "지금 무엇이 떠 있는가"가 불투명하다.

### 어떻게 (단계적, 되돌릴 수 있게)

```
1단계  기동 통합 — 코드는 안 건드린다
       navifra 를 systemd 로 두되, 상위 launch 에 조건부 include 를 두어
       "드라이버가 이미 떠 있으면 붙고, 아니면 같이 띄운다" 로 만든다.
       → 기동 순서 계약을 launch 파일에 명시적으로 적는다

2단계  인터페이스 고정 — robot_msgs 로 계약을 문서화
       지금 String/Bool/Float32 로 오가는 리프트·작업 명령을
       robot_msgs 의 정식 msg/srv 로 승격한다
       (robot_msgs 는 이미 message_generation + add_service_files 를 갖고 있고
        CaptureImages.srv 가 float32 를 싣고 있다 — 새 srv 추가에 장애물 없음)
       예: SetLiftHeight.srv, RunTask.action

3단계  소스 편입 — navifra 소스를 받을 수 있을 때
       navifra/src/<pkg> 를 mobile_manipulator_ws/src/navifra_drivers/ 아래로 옮기고
       한 번의 catkin_make 로 전체 빌드. install 바이너리는 폐기
       param.yaml 은 그대로 유지 (현장 튜닝값이므로 소스 밖에 남긴다)

4단계  기동 일원화
       roslaunch mobile_manipulator bringup.launch  한 줄로 드라이버 + 응용 전부
       systemd 유닛은 이 한 줄만 부른다. User/HOME 하드코딩도 이때 제거
```

> 소스를 못 받는 경우 3단계는 생략하고, navifra install 공간을
> `mobile_manipulator_ws` 안에 서브디렉터리로 넣어 **소싱만** 통합해도
> 1·2·4단계의 이득은 대부분 얻는다.

---

## Slide 10 — 향후 계획 ②: GUI (블록 코딩 방식)

### 개념

```
┌──────────────────────────────────────────────────┐
│  ┌────────────────┐ → ┌────────────┐ → ┌───────┐ │
│  │scan_joints_line1│   │scan_line2  │   │go_home│ │
│  └────────────────┘   └────────────┘   └───────┘ │
│                                                  │
│  사용 가능한 블록:  [scan_line1] [scan_line2]     │
│                     [go_home]  [goto 105] …      │
│                                                  │
│              ┌─────────┐  ┌──────┐               │
│              │  START  │  │ STOP │               │
│              └─────────┘  └──────┘               │
└──────────────────────────────────────────────────┘
```

- 빈 슬롯에 **모듈화된 task 블록**을 순서대로 끼운다.
- START 를 누르면 순차 실행. 하나가 끝나면 **IDLE 로 대기**했다가 다음으로 넘어간다.
- 블록 목록은 task 정의 CSV 에서 **자동 생성**한다 — GUI 에 task 이름을 하드코딩하지 않는다.

### 왜 지금 구조에서 가능한가

이미 갖춰진 전제 세 가지:

1. **모든 task 가 자기 완결적이다** — 끝나도 복귀하지 않으므로 어떤 순서로도 이을 수 있다.
2. **입구가 `/task_command` 하나다** — GUI 는 이 토픽에 문자열만 쏘면 된다. 새 통신 경로 불필요.
3. **끝을 알 수 있다** — task 종료 시 상태가 `IDLE` 로 돌아오므로 "다음 블록 실행" 시점 판정이 가능하다.

### 구현 시 손봐야 할 것

- 지금은 상태 판정을 `/lifter/state` 처럼 개별 토픽으로 봐야 한다 →
  **시스템 전체 상태 토픽 하나**(현재 task, 진행률, 남은 블록)를 새로 발행하는 편이 낫다.
- 순차 실행 큐를 **GUI 쪽에 둘지 `task_executor` 안에 둘지** 결정 필요.
  → `task_executor` 안이 낫다. GUI 가 죽어도 작업이 이어지고, GUI 는 표시만 담당하게 된다.
- 이때 `/task_command` 를 **`RunTaskSequence.action`** 으로 승격하면 진행률·취소·결과가
  한 번에 해결된다 (슬라이드 7 의 actionlib 이야기와 같은 결론).

---

## Slide 11 — 마무리

- **지금**: 드라이버 계층(navifra) / 응용 계층(mobile_manipulator_ws) 이 ROS 토픽으로 분리되어 있고, 장치마다 주인 노드가 하나씩 있다. 주행·리프트·팔 세 장치가 **완전히 같은 3층 구조**(제어 로직 / 주인 노드 / 대리자)를 쓴다.
- **남은 부채**: 리프트–팔 좌표 결합(`arm_base_z`), 캘리브레이션 미검증, 세 대리자 모두 `actionlib` 미사용.
- **다음**: 워크스페이스 통합(기동 → 인터페이스 → 소스 → 일원화) → 블록 코딩 GUI.
- 통합과 GUI 는 **같은 방향의 작업**이다. 둘 다 "명시적인 인터페이스 계약"을 요구하고,
  그 계약을 만드는 순간 나머지는 따라온다.
