# 1. 개요

Fairino FR10v6 6축 협동로봇 팔을 Navifra 이동 베이스에 얹은 **모바일 매니퓰레이터**
시스템의 운용 가이드다. 시스템은 다음 순서로 동작한다.

1. AprilTag 시각 내비게이션으로 지정 태그까지 이동
2. 도착 위치에서 로봇 팔이 정해진 스캔 점들을 순회
3. 각 점에서 Basler 카메라로 촬영 → ONNX 추론 → **표면조도(Ra)** 예측
4. 결과를 Ra 맵 CSV로 증분 저장

- 대상 브랜치: `real` / 대상 패키지: `apriltag_nav`
- 시뮬레이션 지원은 제거되었다. **실 로봇 전용**이다.
- 이동 베이스 드라이버는 별도 설치물이다: **Navifra KU Polishing Robot Driver v0.16**
  (`~/navifra`, systemd 유닛 `navifra-robot`). 주행·리프트·조명·배터리·Safety PLC를
  드라이버가 소유하며, 현장 튜닝 값은 이 워크스페이스가 아니라 `~/navifra/param.yaml`에 있다.

## 1.1 기술 스택

| 구분 | 내용 |
|---|---|
| ROS | Noetic / Ubuntu 20.04 |
| 언어 | Python 3.8, C++17 |
| 빌드 | Catkin (`catkin_make`) |
| 팔 제어 | Fairino SDK (`GetInverseKin`, `GetInverseKinRef`, `MoveJ`, `MoveL`) |
| 내비게이션 | dt_apriltags, Pure Pursuit, S-curve 속도 프로파일 |
| 스캔 카메라 | Basler (PyPylon) — 손목 장착, 폴백 없음(하드 의존) |
| 태그 카메라 | Orbbec Femto Bolt(front) / RealSense D405(side) / D435(hand) |
| 거리 센서 | Keyence DL-EN1 (TCP) |
| 추론 | ONNX Runtime (CPU), ResNet3D |
| 변환 | scipy(≥1.4), numpy(≥1.21.6) |

## 1.2 문서 구성

| 문서 | 내용 |
|---|---|
| **본 가이드** | 한국어 운용 가이드 + 핵심 기술 요약 |
| `CLAUDE.md` | 영문 요약 — 설계 의사결정과 함정 위주 |
| `README.md` | 영문 상세 기술 문서 |
| `docs/keyence_scan_chain.md` | Keyence 거리 폐루프 상세 |
| `docs/lift_arm_base_z_analysis.md` | 리프트가 `arm_base_z`를 깨뜨리는 문제 분석 |

설정 값이 문서와 어긋나면 **`config/robot.yaml`과 launch 파일이 항상 우선**한다.
튜닝 값은 그 두 곳에만 존재하며, 문서는 그것을 설명할 뿐이다.

---

# 2. 시스템 구성

## 2.1 노드 구조

하드웨어 장치 1개당 소유 노드 1개 — Navifra 드라이버와 같은 패턴이다. 다른 노드는
토픽/서비스로만 접근한다.

| 노드 | 소유 대상 | 주요 인터페이스 |
|---|---|---|
| `mobile_manipulator_system` (`task_executor.py`) | 오케스트레이션, STATUS 램프, 비상정지 감지, 배터리 감시 | `/task_command` |
| `arm_controller_node` | Fairino FR10v6 팔 | `/arm/scan_command`, `/arm/cancel`, `/arm/move_home`(srv), `/arm/status` |
| `basler_camera_node` | 손목 Basler **+ VISION 램프** | `/camera/capture`(srv), `/camera/set_active`, `/camera/state` |
| `keyence_dlen1_node` | Keyence DL-EN1 | `keyence/value`, `keyence/raw` |
| `robot_camera_node` | front/side/hand 카메라 AprilTag 검출 | `/<cam>/tag_detections`, `/<cam>/tag_overlay`, `/robot_camera/<cam>/set_enabled`(srv) |
| `camera_viewer_node` | RViz 디버그 뷰어 (장치 소유 없음) | `/camera_viewer/set_enabled`(srv) |

`mobile_manipulator.launch`가 위 6개 노드와 벤더 카메라 드라이버 3개(Orbbec 1, RealSense 2)를
함께 기동한다.

**6개 중 5개는 필수다.** 선택 항목은 `camera_viewer_node` 하나뿐이며, 장치를 소유하지
않는 디버그 뷰어라 없어도 운용에 지장이 없다.

> **`robot_camera_node`는 비전 정지를 안 쓰더라도 필수다.** 비어 있는
> `vision_stop.stop_tag_ids`가 비활성화하는 것은 *정지 판단*뿐이고, 이 노드 자체는
> 여전히 내비게이션의 입력원이다. 이 노드가 없으면 `detected_tags`가 비어 있고,
> `/robot_pose`가 발행되지 않으며, 결과적으로 **`GOTO`가 동작하지 않는다.**
> 같은 이유로 주행 중에 front_cam을 끄면 안 된다(§7.3).

## 2.2 소유권 원칙 — 반드시 지킬 것

- **VISION 램프**는 `basler_camera_node` 전용이다. 셔터와 묶여 촬영 순간에만 점등된다.
  다른 노드가 `/crevis/led/vision`을 발행하면 안 된다.
- **STATUS 램프(RGB)**는 `task_executor` 전용이다.
- **Navifra 드라이버 토픽을 직접 만지지 않는다.** `apriltag_nav` 안에서는
  `navifra_devices.py`(`NavifraDevices`)를 통해서만 접근한다.
- `robot_camera_node`는 **발행만 한다.** `/cmd_vel`이나 정지 메서드를 직접 호출하지 않는다.
  정지 판단은 `robot_controller.py`가 한다.

### STATUS 램프 색상

| 색 | 상태 |
|---|---|
| 초록 | IDLE (대기) |
| 파랑 | MOVING (이동 중) |
| 시안 | SCANNING (스캔 중) |
| 빨강 | ERROR (오류 / 비상정지 후) |

`ARRIVED`, `SCAN_DONE`은 전용 색이 없어 초록으로 표시된다.

## 2.3 워크스페이스 레이아웃

```
mobile_manipulator_ws/
├── CLAUDE.md, README.md
├── docs/                      # 본 가이드 및 분석 문서
└── src/
    ├── apriltag_nav/          # 핵심 패키지
    │   ├── scripts/           # ROS 노드 진입점만 (장치당 1개)
    │   ├── src/apriltag_nav/  # import 가능한 라이브러리 (catkin_python_setup)
    │   ├── tools/             # 독립 실행 스크립트 (설치 안 됨)
    │   ├── config/            # robot.yaml, map.yaml, robot_cameras.rviz
    │   ├── task/csv/          # 스캔 태스크 CSV 및 결과
    │   ├── model/             # ONNX 가중치 (gitignore)
    │   └── launch/            # mobile_manipulator.launch
    ├── robot_msgs/            # 커스텀 메시지
    ├── fairino_sdk/           # Fairino SDK 래퍼
    ├── frcobot_ros/           # URDF, MoveIt, ros_control HW 인터페이스
    ├── orbbec_camera/         # Femto Bolt 드라이버 (GitHub vendoring)
    ├── realsense-ros/         # RealSense 드라이버 (소스 빌드, ros1-legacy)
    └── path_tag_locator/      # 외부 캘리브레이션 기준값 보유
```

라이브러리는 `from apriltag_nav.map_manager import MapManager` 형태로 import한다.

---

# 3. 빌드

```bash
cd ~/mobile_manipulator_ws
catkin_make
source devel/setup.bash
```

메시지 패키지를 먼저 찾지 못하는 경우:

```bash
catkin_make --only-pkg-with-deps robot_msgs
catkin_make
```

## 3.1 RealSense 드라이버는 반드시 소스 빌드

`src/realsense-ros`를 이 워크스페이스에서 함께 빌드해야 한다. **apt 패키지를 쓰면
D405가 동작하지 않는다.**

이유는 이렇다.

- D405는 비교적 나중에 나온 모델이고, `librealsense`는 **지원 카메라 목록을 라이브러리
  안에 USB Product ID로 하드코딩**해 둔다.
- Ubuntu 20.04 apt 저장소의 `ros-noetic-realsense2-camera`는 **librealsense 2.50**에
  묶여 있는데, 이 버전의 목록에는 D405의 ID인 `0x0B5B`가 없다.
- 그래서 카메라를 꽂아도 드라이버가 `Unsupported device! Product ID: 0x0B5B`를 내며
  거부한다. **케이블이나 udev 권한 문제가 아니라 버전 문제**다.

해결책은 D405를 아는 최신 realsense-ros를 워크스페이스에서 직접 빌드하는 것이고,
현재 그렇게 되어 있다. `src/realsense-ros/`가 존재하고 apt 패키지는 설치되어 있지 않다.

```bash
dpkg -l | grep realsense     # 출력이 없어야 정상
ls src/realsense-ros         # realsense2_camera / realsense2_description 있어야 정상
```

> **`sudo apt install ros-noetic-realsense2-camera`를 실행하지 말 것.** apt 버전이
> 소스 빌드본보다 먼저 잡혀서, 잘 되던 D405가 다시 죽는다. 이미 설치했다면 제거한 뒤
> `catkin_make`를 다시 돌리고 `source devel/setup.bash`를 새로 해야 한다.

---

# 4. 기동 절차

## 4.1 순서

```bash
# 1) Navifra 베이스 드라이버 (부팅 시 자동 기동)
sudo systemctl start navifra-robot
systemctl status navifra-robot          # active 확인
# ※ 이 서비스가 roscore 를 이미 띄운다. 별도 roscore 실행 불필요.

# 2) 매니퓰레이터 스택
cd ~/mobile_manipulator_ws
source devel/setup.bash
roslaunch apriltag_nav mobile_manipulator.launch
```

## 4.2 기동 확인

```bash
rostopic echo -n1 /arm/status      # "idle" 이면 팔 노드 정상
rostopic echo -n1 /camera/state    # "closed" 가 정상 (카메라는 평소 꺼져 있음)
rostopic echo -n1 /safety/estop    # false 여야 정상
rostopic list | grep tag_detections # front/side/hand 검출 토픽 확인
```

## 4.3 launch 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `use_front_cam` | `true` | Orbbec 드라이버 + front 검출기 동시 게이트 |
| `use_side_cam` | `true` | RealSense D405 드라이버 + 검출기 |
| `use_hand_cam` | `true` | RealSense D435 드라이버 + 검출기 |
| `front_cam_width` / `_height` / `_fps` | `1280` / `720` / `30` | front_cam 컬러 해상도 |
| `debug_mode` | `false` | `/task_command`의 `EXEC`/`EVAL` 채널 개방 |

```bash
# 예: hand_cam 없이 한 번만 기동
roslaunch apriltag_nav mobile_manipulator.launch use_hand_cam:=false
```

`use_<name>_cam:=false`는 벤더 드라이버와 검출기를 **함께** 끄므로 USB 대역폭과 CPU가
실제로 반환된다. `robot.yaml`의 `robot_camera.enabled`만 끄면 드라이버는 계속 돌아
대역폭을 소모하므로, 다른 프로그램이 그 카메라 영상을 써야 할 때만 의미가 있다.

> **`debug_mode`는 운용 중 금지.** `EXEC`/`EVAL`은 인증 없는 토픽에서 임의의 파이썬
> 코드를 노드 프로세스 안에서 실행한다. 디버그 세션에서만 켠다.

## 4.4 전 장치 점검 — `test_all_devices.py`

각 장치가 **연결되어 있는지**뿐 아니라 **실제로 데이터가 흐르는지**까지 확인하는
진단 스크립트다. 토픽이 "광고만 되고" 데이터는 안 오는 상태를 잡아내는 것이 핵심이다.

**읽기 전용으로 설계되었다.** `/cmd_vel`을 발행하지 않고, 팔을 홈으로 보내거나 움직이지
않으며, LED를 켜지 않는다. `--capture`를 명시하지 않는 한 `/camera/capture`도 호출하지
않는다(그 서비스는 VISION 램프를 켜기 때문). **스택이 돌아가는 중에 그대로 실행해도
안전하다.**

### 실행

`tools/`는 설치되지 않으므로 직접 실행한다.

```bash
cd ~/mobile_manipulator_ws
source devel/setup.bash
cd src/apriltag_nav/tools

python3 test_all_devices.py                  # 전체 6개 섹션
python3 test_all_devices.py ros              # 한 섹션만
python3 test_all_devices.py net arm keyence  # 여러 섹션
python3 test_all_devices.py ros -d 5.0       # 토픽 샘플링 창을 5초로
python3 test_all_devices.py --capture        # /camera/capture까지 실제로 호출
```

### 섹션

| 섹션 | 점검 내용 |
|---|---|
| `net` | 네트워크 맵의 모든 장치에 TCP/ICMP 도달 확인 |
| `arm` | Fairino FR10v6 SDK 연결 — 관절값, TCP 포즈, 모드 |
| `keyence` | Keyence DL-EN1에 `M0` 측정 1회 요청 |
| `basler` | 손목 Basler 열거 + 프레임 1장 grab |
| `usb` | Orbbec / RealSense USB 존재 + **시리얼이 launch와 일치하는지** |
| `ros` | 마스터, 예상 노드, 서비스, 토픽별 수신율 |

### 결과 읽는 법

각 검사는 네 가지 상태 중 하나로 보고된다.

| 상태 | 의미 |
|---|---|
| `PASS` | 정상 |
| `FAIL` | 필수 항목 실패 — 종료 코드가 1이 된다 |
| `WARN` | 동작은 하지만 확인이 필요 (예: Keyence가 측정 범위 밖) |
| `SKIP` | 선택 항목이라 건너뜀 (예: 일부러 꺼 둔 카메라) |

마지막에 요약표가 출력되고, **`FAIL`이 하나도 없을 때만 종료 코드 0**이다.

```
========================================================
6. ROS — master, nodes, services, topic data reception
========================================================
  [PASS] roscore — http://localhost:11311
  [PASS] node /arm_controller_node — running
  [PASS] node /keyence_dlen1_node — running
  [SKIP] node /camera_viewer_node — not running

  Sampling 24 topics for 3.0s ...

  [PASS] /odom — 61 msg  20.0 Hz  |  xy=(0.000, 0.000) v=(0.000 m/s, 0.000 rad/s)
  [PASS] /cmd_vel — no traffic (robot idle) — expected
  [PASS] /front_cam/color/image_raw — 90 msg  30.0 Hz  |  1280x720 rgb8
  [PASS] /front_cam/tag_detections — 89 msg  29.7 Hz  |  front_cam 1280x720 tags=[104]
  [SKIP] /side_cam/tag_detections — no data received

--------------------------------------------------------
  PASS 31   FAIL 0   WARN 1   SKIP 6
========================================================
```

읽을 때 알아둘 것:

- `/cmd_vel`에 트래픽이 없는 것은 **정상**이다. 로봇이 정지 중이면 아무도 발행하지 않는다.
- Basler `open`이 `SKIP`으로 나오고 "device held by basler_camera_node"라고 하면
  **정상**이다. 스택이 돌고 있으면 그 노드가 장치를 독점하고 있다.
- Keyence가 `raw=±99999xxx`로 `WARN`을 내면 통신은 정상이고 **측정 대상이 범위 밖**이라는
  뜻이다. 이 상태로 폐루프 스캔을 돌리면 쓰레기 값을 쫓아간다.
- `usb` 섹션의 시리얼 불일치는 반드시 고쳐야 한다. side_cam과 hand_cam이 뒤바뀌면
  서로의 내부 파라미터와 짝지어져 태그 자세가 조용히 틀리게 나온다(§7.2).

`ros` 섹션은 launch가 띄우는 6개 노드를 모두 점검한다. `camera_viewer_node`만 선택
항목이라 없어도 `SKIP`이고, 나머지 5개는 없으면 `FAIL`이다.

---

# 5. 태스크 명령

`/task_command` (`std_msgs/String`) 토픽으로 전송한다.

| 명령 | 설명 |
|---|---|
| `TASK <name>` | 등록된 태스크 실행 |
| `GOTO <tag_id>` | 지정 AprilTag로 이동 (스캔 없음) |
| `TEST_POSE x y z [rx ry rz]` | 월드 좌표 포즈 제어 테스트 (디버그) |
| `STOP` | 즉시 정지 — 주행과 스캔을 함께 중단 |
| `STATE` | 현재 상태 조회 |
| `EXEC <code>` / `EVAL <expr>` | 디버그 실행 (`debug_mode:=true` 필요) |

```bash
# 스캔 (라인별)
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_grid_line1"

# 병합 스캔 → 통합 Ra 맵 CSV 생성
rostopic pub -1 /task_command std_msgs/String "TASK scan_full_joints"
rostopic pub -1 /task_command std_msgs/String "TASK scan_full_pose"

# 홈 복귀 / 태그 이동 / 상태 / 정지
rostopic pub -1 /task_command std_msgs/String "TASK go_home"
rostopic pub -1 /task_command std_msgs/String "GOTO 104"
rostopic pub -1 /task_command std_msgs/String "STATE"
rostopic pub -1 /task_command std_msgs/String "STOP"

# 포즈 테스트 (디버그)
rostopic pub -1 /task_command std_msgs/String "TEST_POSE 0.737 2.14 0.704"
```

## 5.1 등록된 태스크

`task_manager.py`의 `TASK_DEFS` 기준이다.

| 태스크 | 모드 | 입력 CSV | 결과 파일 |
|---|---|---|---|
| `scan_joints_line1` | joint | `optimized_joints_line1.csv` | `..._result.csv` |
| `scan_joints_line2` | joint | `optimized_joints_line2.csv` | `..._result.csv` |
| `scan_grid_line1` | pose | `grid_path_line1.csv` (+ joint CSV를 q0 시드로) | `..._result.csv` |
| `scan_grid_line2` | pose | `grid_path_line2.csv` (+ joint CSV를 q0 시드로) | `..._result.csv` |
| `scan_full_joints` | joint | line1 + line2 joint CSV (+ pose CSV에서 xyz) | `scan_full_joints_ra_map.csv` |
| `scan_full_pose` | pose | line1 + line2 pose CSV (+ joint CSV에서 q0) | `scan_full_pose_ra_map.csv` |
| `go_home` | move | — (태그 508로 이동) | — |

- **joint 모드**: CSV에 기록된 관절각(라디안)을 그대로 재생한다.
- **pose 모드**: CSV의 월드 좌표 포즈(m, ZYX 내인 오일러 rad)를 팔 base 좌표로 변환해
  IK로 푼다. 짝을 이루는 joint CSV 값은 IK 시드(`q0`)로만 쓰인다.

## 5.2 상태 머신

```
IDLE ──TASK/GOTO──▶ MOVING ──도착──▶ ARRIVED ──스캔 시작──▶ SCANNING
                                                                │
  ▲                                                             ▼
  └────────────────────── IDLE ◀───────────────── SCAN_DONE ────┘

  (어느 단계에서든 타임아웃·IK 실패·비상정지 → ERROR)
```

## 5.3 실행 흐름 (`scan_full_pose` 예시)

1. `task_manager`가 line1 + line2 pose CSV를 이어붙이고, 대응 joint CSV에서 q0 시드를 읽는다.
2. `group_id` 오름차순(105 → 106 → 117 → 118)으로 처리한다.
3. 각 그룹마다:
   1. `robot_controller`가 해당 태그까지 이동하고 `/robot_pose`를 1회 발행한다.
   2. `arm_controller`가 4-DOF 변환으로 각 스캔 점의 팔 base 목표를 만든다.
   3. 점마다: 카메라 선오픈 → 팔 이동 → 안정화 → Keyence 거리 보정 →
      `/camera/capture` → ONNX 추론 → CSV 1행 append
4. 팔을 홈으로 복귀시키고 `/scan_finished`를 발행한다.

---

# 6. 스캔 결과

## 6.1 Ra 맵 CSV (13열)

병합 스캔(`scan_full_*`)은 다음 13열 CSV를 만든다.

```
group_id, point_id, x, y, z,
ra_mean, ra_std, ra_min, ra_max, num_samples,
success, execution_message, validated_at
```

- **점별 증분 저장**이다. 중간에 `STOP` 하거나 취소해도 이미 끝난 점의 결과는 보존된다.
- joint 모드의 `x, y, z`는 짝을 이루는 pose CSV(`pose_files`)에서 가져온다.
- 저장 위치는 `src/apriltag_nav/task/csv/` 이다.
- 촬영 이미지는 `~/scan_results`에 저장된다(`arm_controller_node`의 `~output_dir`,
  `~save_images` 파라미터).

## 6.2 시각화

```bash
python3 src/apriltag_nav/tools/ra_map_plotter.py \
        src/apriltag_nav/task/csv/scan_full_pose_ra_map.csv --interpolate
```

옵션: `--interpolate`(보간), `--metric ra_std`(표시할 지표 변경). 결과 PNG는 CSV 옆에
생성된다.

## 6.3 CSV 입력 형식

**joint 모드**

```csv
group_id,point_id,q1,q2,q3,q4,q5,q6,collision_detected
106,1,-1.5808,-0.0847,0.6084,-2.0974,-1.6656,-0.0101,FALSE
```

**pose 모드**

```csv
group_id,point_id,x,y,z,rx,ry,rz,speed,comment
106,1,1.1766,2.5154,0.5712,1.5741,-0.0891,3.1049,30,
```

위치는 미터, 자세는 **ZYX 내인(intrinsic) 오일러 라디안**이다. `speed`는 스캔 이동
속도 계수(0–100, 현재 데이터셋은 30)다.

`is_discontinuous`가 1인 점은 이동 전에 팔이 **홈을 경유**한다. 멀리 떨어진 자세 사이의
위험한 직선 경로를 막기 위한 것이다.

---

# 7. 카메라

## 7.1 손목 Basler — 상시 켜두지 않는다

발열·센서 수명·전력 때문에 Basler는 촬영 사이에 **`Close()` 상태**로 둔다. 단순히
"grab 안 함"이 아니라 디바이스를 닫는다.

- 평소 `/camera/state` = `"closed"`
- 촬영은 **스트림이 아니라 서비스**(`/camera/capture`)다. 응답에 프레임이 실려 오므로
  스캔 점이 낡은 토픽 프레임과 짝지어질 수 없다.
- **선(先)오픈** — 스캔할 점 하나를 처리하기 시작할 때, **팔이 움직이기 전에** 먼저
  `/camera/set_active` true를 보내 카메라를 연다. 디바이스 `Open()`에는 시간이 걸리는데,
  그동안 팔은 목표 자세로 이동하고 안정화하고 Keyence 거리 보정까지 수행한다. 즉 오픈
  대기 시간이 촬영 직전에 **따로 더해지지 않고** 팔 동작 시간 안에 묻힌다. 선오픈하지
  않으면 촬영을 요청한 순간부터 오픈이 시작되어 그만큼 점당 시간이 늘어난다.
- 선오픈은 VISION 램프를 켜지 않는다. 램프는 촬영 순간에만 점등된다.
- `camera.idle_close_sec`(기본 5.0초)는 촬영 후 이 시간 동안 디바이스를 열어 둔다.
  다만 이동 + 안정화 + Keyence 보정은 보통 5초보다 오래 걸리므로, 실제로는 점과 점
  사이에 한 번 닫혔다가 다음 점의 선오픈으로 다시 열리는 것이 정상 동작이다.
- VISION 램프는 **촬영 순간에만** 점등된다(셔터와 묶임).

점 하나의 흐름: 선오픈 → 이동 → 안정화 → Keyence 거리 보정 →
`/camera/capture`(램프 점등 → 촬영 → 소등) → 추론.
스캔 전체가 끝나거나 취소되면 `/camera/set_active` false로 즉시 닫는다.

> 선오픈은 **점 단위**로 호출된다. 그룹 ID 단위로 한 번만 여는 것이 아니다 —
> 주행 영역 태그마다 스캔할 그룹 ID가 정해져 있지만, 카메라 개폐는 그 그룹 경계가
> 아니라 개별 스캔 점의 시작에 맞춰 일어난다.

진단용 수동 개폐:

```bash
rostopic pub -1 /camera/set_active std_msgs/Bool "data: true"
```

## 7.2 태그 카메라 3대

| 이름 | 장치 | 역할 |
|---|---|---|
| `front_cam` | Orbbec Femto Bolt | 내비게이션 태그 검출 + 비전 정지 판단의 입력 |
| `side_cam` | RealSense D405 (S/N 409122271601) | 검출 발행만 — 아직 소비자 없음 |
| `hand_cam` | RealSense D435 (S/N 935322071325) | 검출 발행만 — 아직 소비자 없음 |

`robot_camera_node`가 카메라마다 독립된 `dt_apriltags` 검출기를 돌리므로, 하나를 꺼도
나머지는 영향을 받지 않는다.

> **serial_no는 필수다.** 이 PC의 USB 버스에 RealSense 계열 3대가 물려 있어 시리얼이
> 없으면 각 래퍼가 먼저 열거된 장치를 잡는다. side_cam과 hand_cam이 뒤바뀌면 서로의
> 내부 파라미터와 짝지어져 **태그 자세가 조용히 틀리게** 나온다. 카메라 교체 시
> `rs-enumerate-devices -s`로 확인해 갱신할 것.

### 검출기는 하나, 소비자는 둘

내비게이션은 자체 검출을 하지 않고 `robot_camera_node`의 출력을 소비한다.

```
front_cam ─▶ robot_camera_node ─▶ /front_cam/tag_detections ─┬─▶ detected_tags (내비게이션)
                                                             └─▶ 비전 정지 판단
```

구독이 하나이므로 정지 판단과 주행 조향이 **항상 같은 프레임**을 본다.
`corners`(`float64[8]`)가 메시지에 함께 실려 오고, 정렬 각도는 corner0→corner1 변에서
계산된다. 이 계산은 튜닝된 것이므로 yaw로부터 근사해 다시 만들지 말 것.

### front_cam 해상도

**1280x720**을 쓴다. 드라이버 기본값인 1920x1080은 MJPG 디코드만으로 140 ms 지연을
만들고 30 Hz 스트림 대비 검출을 17 Hz로 떨어뜨린다. 720p는 RGB888로 발행되어 검출
30 Hz를 유지하고 종단 지연이 109 ms다. 1280x720은 Femto Bolt가 제공하는 **가장 낮은**
컬러 프로파일이다.

## 7.3 카메라 켜고 끄기 — 수명별 3가지 방법

| 범위 | 방법 |
|---|---|
| 재기동 없이 즉시 | `rosservice call /robot_camera/<name>/set_enabled "data: false"` |
| 1회 실행 | `roslaunch … use_<name>_cam:=false` (드라이버까지 함께 끔) |
| 영구 기본값 | `robot.yaml` → `robot_camera.enabled.<name>` |

### 실행 중 켜고 끄기 — launch를 내리지 않는다

`roslaunch`가 돌아가는 터미널은 그대로 두고, **새 터미널을 하나 더 열어** 서비스를
호출하면 된다. 노드 재기동도, launch 재실행도 필요 없다.

```bash
# 새 터미널
cd ~/mobile_manipulator_ws
source devel/setup.bash

# 끄기
rosservice call /robot_camera/front_cam/set_enabled "data: false"
rosservice call /robot_camera/side_cam/set_enabled  "data: false"
rosservice call /robot_camera/hand_cam/set_enabled  "data: false"

# 켜기
rosservice call /robot_camera/front_cam/set_enabled "data: true"
rosservice call /robot_camera/side_cam/set_enabled  "data: true"
rosservice call /robot_camera/hand_cam/set_enabled  "data: true"
```

응답은 `std_srvs/SetBool`의 표준 형식이다.

```
success: True
message: "side_cam disabled"
```

껐는지 확인하는 방법:

```bash
rosservice list | grep set_enabled            # 서비스가 살아 있는지
rostopic hz /side_cam/tag_detections          # 껐으면 발행이 멈춘다
rostopic hz /side_cam/color/image_raw         # 벤더 스트림도 함께 멈춘다
```

세 대는 서로 독립이다. side_cam을 꺼도 front_cam의 내비게이션 검출과 비전 정지 판단은
그대로 동작한다. 다만 **front_cam을 끄면 내비게이션이 태그를 못 본다** — 주행 중에는
끄지 말 것.

`camera_viewer`(§7.5)와 혼동하지 말 것. 뷰어 서비스는 창을 열고 닫을 뿐이고, 여기서
끈 카메라는 뷰어를 열어도 영상이 나오지 않는다.

### 서비스가 실제로 하는 일

서비스는 검출기와 **벤더 스트림을 함께** 정지시킨다. 괄호 안의 표기는 그 "벤더 스트림"을
끄기 위해 이 노드가 대신 호출해 주는 **벤더 드라이버 쪽 서비스 이름**이다.

| 카메라 | 벤더 드라이버 | 실제로 호출되는 서비스 | 타입 |
|---|---|---|---|
| front_cam | orbbec_camera | `/front_cam/toggle_color` | `std_srvs/SetBool` |
| side_cam | realsense2_camera | `/side_cam/enable` | `std_srvs/SetBool` |
| hand_cam | realsense2_camera | `/hand_cam/enable` | `std_srvs/SetBool` |

이 이름들은 `robot.yaml`의 `robot_camera.driver_toggle`에 설정되어 있다. 벤더 launch를
커스터마이즈해 네임스페이스가 달라졌을 때만 수정하면 된다.

**벤더 스트림**이란 카메라 제조사가 제공하는 ROS 드라이버 노드(Orbbec의 `orbbec_camera`,
Intel의 `realsense2_camera`)가 USB로 장치에서 프레임을 받아 `/<cam>/color/image_raw`로
발행하는 그 영상 흐름 자체를 말한다. 우리 코드가 아니라 벤더 코드가 만들어 내는
스트림이라 "벤더 스트림"이라고 부른다.

검출기만 끄고 스트림을 그대로 두면 CPU의 태그 검출 비용은 사라지지만 USB 대역폭과
드라이버의 디코드 비용은 계속 나간다. 그래서 이 서비스는 둘을 함께 끈다.

> **기동 시점에 드라이버를 토글하면 안 된다.** launch가 이미 요청된 상태로 띄웠고,
> 재확인은 무해한 동작이 아니다. RealSense는 두 번째 `enable`에
> `open(...) failed. UVC device is streaming!`로 답하고 이미 서비스 중이던 스트림을
> 끊을 수 있다. 실제 상태 전이일 때만 드라이버를 건드린다.

## 7.4 태그 오버레이

`robot_camera_node`는 `/<cam>/tag_overlay`를 발행한다. 광축에 호박색 십자선을 그리고,
태그마다 ID, 십자선 대비 오프셋(**픽셀과 방위각(도) 양쪽**), roll/pitch/yaw, 기울기,
거리를 표시한다. 초점거리가 다른 카메라 사이에서는 픽셀만으로 비교가 안 되므로 둘 다 낸다.

구독자가 있을 때만 렌더링하므로 뷰어를 열지 않으면 비용이 0이다.

> **정면으로 마주 본 태그가 rpy 0,0,0으로 읽히지 않는다.** 태그의 +Z가 카메라 쪽을
> 향하므로 어느 한 각도가 ±180에 놓인다. 어느 각도가 뒤집힘을 떠안는지는 오일러 분해에
> 따라 달라지므로(관측상 roll인 경우와 yaw인 경우 모두 있었다) **한 각도만 보고
> 정면 여부를 판단하지 말 것.** 그 용도의 값은 **`tilt_from_normal`**(태그 법선과 광축
> 사이 각)이다. 0이면 완전 정면이고, 태그를 자기 평면 안에서 돌려도 값이 변하지 않는다
> (45° 면내 회전에서도 tilt 0으로 검증됨).

### 내비게이션이 태그의 Z축 회전만 보는 이유

front_cam은 **바닥을 수직 아래로 내려다보도록 설치**되어 있고, 태그는 바닥에 평면으로
붙어 있다. 따라서 태그 법선과 카메라 광축은 원칙적으로 평행하며, `tilt_from_normal`은
0에 가깝다고 **가정**한다. 이 가정 아래에서는 태그의 roll/pitch가 담는 정보가 없고
남는 자유도는 **광축을 중심으로 한 회전(태그의 Z축 회전)** 하나뿐이므로, 내비게이션은
그 각도만 사용해 로봇의 방위를 구한다.

`corners`에서 corner0→corner1 변으로 정렬 각도를 계산하는 것도 같은 이유다 — 면내
회전만 남은 상황에서 가장 직접적이고 잡음에 강한 측정이 태그 변의 방향이다.

> 이 가정이 깨지는 경우를 알아두는 것이 중요하다. 카메라 마운트가 틀어지거나 태그가
> 들뜨거나 바닥이 기울면 `tilt_from_normal`이 0에서 벗어나고, 그때부터 면내 회전각에
> 실제 기울기가 섞여 들어가 방위가 조용히 틀어진다. 태그 인식이 이상할 때
> `/front_cam/tag_overlay`에서 **`tilt_from_normal`이 0 근처인지 먼저 확인**하는 것이
> 가정 위반을 잡아내는 가장 빠른 방법이다.

## 7.5 디버그 뷰어

```bash
rosservice call /camera_viewer/set_enabled "data: true"    # 열기
rosservice call /camera_viewer/set_enabled "data: false"   # 닫기
```

`camera_viewer_node`는 장치를 소유하지 않고 아무것도 발행하지 않는다. `config/robot_cameras.rviz`
레이아웃으로 RViz를 띄우고 내리기만 한다. **닫힌 상태로 시작**한다 — 스택 기동이
화면에 창을 띄워서는 안 되고, 헤드리스 부팅에는 디스플레이가 없을 수도 있기 때문이다.
디버그 세션에서만 `~auto_start:=true`로 바꾼다.

카메라별 on/off는 이 노드가 아니라 `robot_camera_node` 소관이다. 거기서 끈 카메라는
뷰어에서도 영상이 나오지 않는다.

---

# 8. 안전

## 8.1 비상정지 — 하드웨어다

- 비상정지는 **PILZ PNOZmulti 2 안전 PLC**가 담당한다. 비상버튼·범퍼가 눌리면 PLC가
  ROS와 무관하게 모터 전원을 물리적으로 차단한다.
- `/safety/estop`은 **읽기 전용 상태 피드백**일 뿐 정지 수단이 아니다. `task_executor`가
  이를 감지해 진행 중 태스크를 중단하고 ERROR(빨강 램프)로 전환한다.
- 소프트웨어 `/estop` 토픽은 드라이버 v0.11에서 제거되었다. 찾지 말 것.
- `/safety/estop`은 **fail-safe**다(기동 직후와 PLC 통신 두절 시 true). 따라서
  `estop_active`는 실제 메시지를 한 번 받은 뒤에만 true를 보고하고, "드라이버 미기동"
  상황은 `safety_link_ok()`가 별도로 다룬다.
- 해제 후 재기동: 물리 리셋 → 새 `TASK` 명령 전송.

`robot.yaml`의 `navifra.require_safety_link`가 `false`이면 안전 링크가 없어도 경고만
하고 스택이 동작한다(`safety_io_driver` 미기동 환경 대응). 실 운용에서는 `true` 권장.

## 8.2 리프트 주의 — pose 모드가 조용히 틀어진다

팔 변환의 `arm_base_z`는 **상수**이며 리프트 위치를 추적하지 **않는다.** 따라서 리프트를
움직이면 pose 모드 스캔(`scan_grid_*`, `scan_full_pose`, `TEST_POSE`)의 목표 높이가
리프트 이동량만큼 **오류 없이** 틀어진다.

- joint 모드(`scan_joints_*`)는 기록된 관절각 재생이므로 **영향 없다.**
- 보정된 스캔 높이는 `robot.yaml` → `navifra.scan_height_counts`에 기록한다.
  `null`이면 가드가 비활성이다. 가드 동작은 `navifra.scan_height_guard`
  (`warn` | `refuse` | `off`)로 정한다.
- 리프트 위치는 **증분(엔코더)** 이라 전원 재투입 시 소실된다. 절대 위치 명령 전에
  전원 사이클당 한 번 홈잉이 필요하다(`/lift/home` → 하부 리미트 원점).
  스트로크는 실측 0 ~ 약 7000 카운트, 전 구간 약 28초(2000 rpm)다.
- 상세 분석: `docs/lift_arm_base_z_analysis.md`

## 8.3 기동은 팔을 움직이지 않는다

`ArmController.__init__`는 폴트를 클리어하고 `Mode(0)`을 설정하지만 **홈잉하지 않는다.**
스택을 띄우는 것은 움직이라는 요청이 아니다 — 팔이 지그 안이나 공작물에 닿은 채로
전원이 들어왔을 수 있고, 아무도 움직임을 예상하지 않는 시점의 `MoveJ`는 충돌 위험이다.

홈잉은 명시적일 때만 일어난다.

- `task_executor`가 매 태스크 전에 `move_to_home()`을 호출(멱등)
- `/arm/move_home` (`std_srvs/Trigger`) 서비스 호출

홈 관절각: `[-1.5708, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]` rad (`robot.yaml` `arm_home`).

## 8.4 배터리

`/bms/state`의 잔량이 `navifra.low_battery_pct`(기본 20%) 미만이면 경고 로그를 남긴다.

---

# 9. 좌표계와 변환

## 9.1 프레임 체인

```
World Frame (CSV 포즈, 원점은 폴리싱 셀)
  │  world_x = -msg.y,  world_y = -msg.x
  ▼
Manipulator Frame (/robot_pose: x, y [m], theta [deg])
  │  p_A_W = (x_base, y_base, arm_base_z)                    # 2-DOF 병진
  │  R_AW  = Rz(-(θ+mount_yaw))·Ry(-tilt_y)·Rx(-tilt_x)      # 4-DOF 회전
  ▼
Arm base_link (IK 입력: Fairino SDK 단위 mm + deg)
```

CSV 자세는 **ZYX 내인 오일러(라디안)** 이며, IK 목표 자세는 `process_transforms`를 거쳐
**CSV 값을 그대로** 쓴다. 스캔 패턴 전체에서 엔드이펙터 자세가 거의 변하지 않기 때문에
CSV 값이 가장 깨끗한 출처다. `q0`(짝 joint CSV)는 IK 시드로만 쓴다.

## 9.2 4-DOF 물리 마운트 모델

기준값 출처는 **`path_tag_locator/config/extrinsics.yaml`의 `T_ab2mb`**(플랫폼 실측)이다.
마운트에는 **틸트가 없다** — R은 정확히 Rz(180°)다. 655점 실 로봇 피팅
(`task/csv/calib_data_params.yaml`, tilt ≈ 0.0001 / 0.0007 rad)이 이를 독립적으로 확인했다.

| 파라미터 | ROS `~` 파라미터 | 값 | 의미 |
|---|---|---|---|
| Body offset X | `~arm_body_offset_x` | 0.0 m | 바디 프레임 기준 팔 마운트 X |
| Body offset Y | `~arm_body_offset_y` | 0.0 m | 바디 프레임 기준 팔 마운트 Y |
| Base Z | `~arm_base_z` | 1.025 m | mb 원점 기준 팔 base 높이 |
| Mount yaw | `~arm_mount_yaw` | π (정확) | 바디 대비 팔 base yaw |
| Tilt X | `~arm_tilt_x` | 0.0 | 마운트 틸트 없음 |
| Tilt Y | `~arm_tilt_y` | 0.0 | 마운트 틸트 없음 |

모두 `robot.yaml`의 `arm_calibration` 블록에 있고 ROS `~` 프라이빗 파라미터로 오버라이드
가능하다. **코드에 하드코딩 금지.**

> **미해결 불일치:** 655점 피팅은 `arm_base_z`를 **0.9541**로 냈다 — extrinsics 값보다
> 71 mm 낮다. 새 베이스에서는 리프트 높이가 달랐을 가능성이 크다
> (`navifra.scan_height_counts` 참조). **정밀 pose 스캔 전에 알려진 리프트 위치에서
> 짝지은 joint/pose 데이터로 재검증할 것.** 재검증 전까지 pose 모드의 위치 잔차는
> 미확정이다.

## 9.3 관절 한계 (FR10v6)

| 관절 | 최소 (rad) | 최대 (rad) |
|---|---|---|
| J1 | −3.0543 | 3.0543 |
| J2 | −4.6251 | 1.4835 |
| J3 | −2.8274 | 2.8274 |
| J4 | −4.6251 | 1.4835 |
| J5 | −3.0543 | 3.0543 |
| J6 | −3.0543 | 3.0543 |

속도 한계: `[3.15, 3.15, 3.15, 3.2, 3.2, 3.2]` rad/s.
실행 전 `validate_joint_values()` / `validate_velocities()`가 검사한다.

---

# 10. Keyence 거리 폐루프

`arm_controller.py`의 `_adjust_distance_to_surface()`가 매 촬영 직전에 툴을 툴 Z 방향으로
미세 이동시킨다. 코드만 봐서는 알 수 없는 세 가지가 있다(전체 기록:
`docs/keyence_scan_chain.md`).

## 10.1 레이저는 비스듬히 달려 있다

빔은 툴 Z에서 **42.6° 기울어져** 있다(실측값이며 URDF나 TCP 어디에도 문서화되어 있지 않다).
따라서 센서 값은 **빔을 따라 잰 거리**이고, 무엇보다 먼저 `cos(beam_angle_deg)`로 투영된다.
투영 이후 `keyence_tol`, `keyence_max_step_mm`, `keyence_activate_threshold`는 모두
**수직 스탠드오프 mm**다. **원시 값과 비교하지 말 것.** 42.6°에서 원시 값은 그것이
나타내는 수직 오차보다 36% 크다.

센서가 재장착되면 `tools/measure_keyence_angle.py`로 다시 구한다.

## 10.2 `keyence_dir`는 −1.0이어야 한다

센서 극성(DL-EN1 본체에 설정되어 있음):

| 실제 거리 | 센서 값 |
|---|---|
| 10 mm 초과 (너무 멂) | 음수 |
| 10 mm (기준점) | 0 |
| 10 mm 미만 (너무 가까움) | 양수 |

툴이 접근할수록 값이 커지므로 `k > 0`이고, `dir`은 **`-sign(k) = -1.0`** 이다.
부호가 반대이면 매 스텝 오차가 `(1 + kp)`배로 증폭되어 **툴이 공작물로 파고든다.**
센서를 재장착하거나 극성을 다시 설정하면 `tools/measure_keyence_angle.py`로 `k`를
다시 재고 부호를 유도할 것.

## 10.3 `keyence_max_step_mm` = 1.0은 임시값이며 실제로 일한다

비스듬한 빔 때문에 보정 1회마다 레이저 스팟이 옆으로 `0.919 × dz`만큼 이동한다.
경사면에서는 실효 감도가 캘리브레이션 값 1.358보다 훨씬 커져 발산 한계 3.40을 넘는
스텝이 관측된다. 이 클램프가 폭주를 막는다.

작동 임계값이 5 mm이므로 최악의 경우에서도 클램프된 5스텝이면 수렴한다
(`keyence_max_steps` 10 이내). `docs/keyence_scan_chain.md`의 미해결 항목을 읽지 않고
**5.0으로 되돌리지 말 것.**

## 10.4 튜닝 파라미터

**우선순위:** `arm_controller`는 `arm_controller_node`의 `~params`를 **먼저** 읽고,
없으면 `robot.yaml`의 `keyence:` 블록으로 폴백한다. launch 파일이 `keyence_tol`, `keyence_kp`,
`keyence_max_steps`, `keyence_max_step_mm`, `keyence_dir`를 설정하므로 **정상 기동에서는
launch 값이 이긴다.**

| 항목 | `~` 파라미터 | 값 | 단위/의미 |
|---|---|---|---|
| 허용 오차 | `~keyence_tol` | 0.2 | 수직 mm — 이하이면 수렴 |
| 비례 게인 | `~keyence_kp` | 0.8 | 스텝당 잔차는 \|1 − kp\| |
| 최대 스텝 수 | `~keyence_max_steps` | 10 | 초과 시 포기 |
| 스텝 클램프 | `~keyence_max_step_mm` | 1.0 | 임시 (정상값 5.0) |
| 작동 임계 | `~keyence_activate_threshold` | 5.0 | 이상이면 루프 생략(값 신뢰 불가) |
| 빔 각도 | `~keyence_beam_angle_deg` | 42.6 | 툴 Z 대비 빔 각(도) |
| 방향 부호 | `~keyence_dir` | −1.0 | `-sign(k)` |

노드가 없으면 `_adjust_distance_to_surface()`는 `"Keyence value NOT available"` 경고를
**한 번** 남기고 반환한다. 스캔은 팔이 멈춘 그 스탠드오프에서 그대로 진행되며
**결과 CSV에는 아무 오류도 기록되지 않는다.**

---

# 11. 설정 레퍼런스

모든 튜닝 값은 `src/apriltag_nav/config/robot.yaml`에 있다.

## 11.1 주행 (`robot:`)

| 파라미터 | 값 | 설명 |
|---|---|---|
| `max_linear_speed` | 0.05 m/s | 최대 직진 속도 |
| `max_angular_speed` | 0.25 rad/s | 최대 회전 속도 |
| `slow_factor` | 0.3 | 도킹/정밀 구간 속도 배수 |
| `camera_offset` | 0.45 m | 로봇 중심 ↔ 카메라 렌즈 거리 |
| `tag_size` | 0.06 m | AprilTag 실물 크기 |
| `tag_family` | `tag36h11` | 태그 패밀리 |
| `navigation_timeout` | 8.0 s | 초과 시 ERROR |
| `wall_dist_work_zone` | 0.35 m | 작업 구역(B/C/D/E) 벽 거리 |
| `wall_dist_zone_a` | 0.6275 m | A 구역 벽 거리 |
| `min_wall_clearance` | 0.10 m | 최소 벽 여유 |

> **`robot.length` 0.80 / `robot.width` 0.50은 구 베이스 값이며 새 Navifra 베이스에서
> 재측정되지 않았다.** 새 베이스의 `wheel_separation`이 0.65 m이므로 폭은 0.65보다 커야
> 한다. 이 값들이 `wall_dist_*` / `min_wall_clearance`로 흘러가므로 **벽 간섭 여유가
> 현재 낙관적으로 잘못 계산된다.** 실측 후 갱신 필요.

## 11.2 지도 (`config/map.yaml`)

AprilTag 토폴로지 지도를 BFS 최단경로로 탐색한다.

- 태그 ID: 100–147, 400–415, 500–508
- 태그 종류: DOCK / PIVOT / MOVE / WORK
- 작업 구역: A(입구 통로), B·D(좌측), C·E(우측)
- 홈 태그: **508** (`task_manager.START_TAG`)

## 11.3 네트워크 맵

| 로봇 PC 포트 | 로봇 PC IP | 장치 | 장치 IP |
|---|---|---|---|
| LAN 1 | 192.168.100.100 | 전방 / 후방 LiDAR | .101 / .102 |
| | | Safety PLC (PNOZ) | .103 |
| | | Crevis GN-9289 IO | .104 |
| | | **Keyence DL-EN1** | **.105** |
| LAN 2 | 192.168.200.100 | 매니퓰레이터 카메라 | .106 |
| LAN 3 | 192.168.58.100 | Fairino FR10v6 팔 | 192.168.58.2 |
| LAN 4 | 192.168.1.100 | 무선 AP | 192.168.1.10 |

192.168.1.x는 AP 전용이다. LAN 허브에 붙는 장치는 전부 192.168.100.x에 속한다.
Keyence는 이에 따라 192.168.1.5 → **192.168.100.105**로 이전되었다.

## 11.4 추론 (`inference:`)

| 파라미터 | 값 |
|---|---|
| 입력 크기 | 900 × 900 |
| 정규화 mean / std | [0.5, 0.5, 0.5] / [0.5, 0.5, 0.5] |

모델 가중치는 gitignore 대상이므로 별도로 관리한다.

```
src/apriltag_nav/model/exported/resnet3D.onnx
src/apriltag_nav/model/exported/resnet3D_gray.onnx
```

---

# 12. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| 스캔 점마다 `no frames captured` | `basler_camera_node` 미기동 또는 카메라 미연결. `/camera/state` 확인 |
| 태스크가 SCANNING에서 멈춤 | `arm_controller_node` 미기동. `/arm/status` 확인 |
| `pose scan requires /robot_pose` | 내비게이션이 위치를 발행하지 않음. `GOTO`로 태그 도착 후 스캔 |
| 기동 직후 `/safety/estop` = true | fail-safe 정상 동작. Navifra `safety_io_driver` 미기동 또는 PLC 통신 두절 |
| 팔이 엉뚱한 높이로 이동 (pose 모드) | 리프트가 보정 높이를 벗어남 (§8.2 참조) |
| Keyence 값 안 들어옴 | 192.168.100.105:64000 확인(LAN1 허브), `keyence_dlen1_node` 로그 확인 |
| 직진이 휘거나 제자리회전 실패 | 구동모터 상태 확인 — `~/navifra/param.yaml`의 `drive_motor_ids`가 `[1]` 뿐이면 2번 모터 수리 중 |
| 태그 검출 불안정 | `tag_size`가 실물과 일치하는지, `camera_info`가 발행되는지 확인 |
| 내비게이션 타임아웃 (8s) | `robot.yaml`의 `navigation_timeout` 조정. 초과 시 ERROR 상태 진입 |
| pose 모드 IK 실패 | 로그의 `dist=` 값 확인. 1.5 m 초과면 변환이 작업 영역 밖 목표를 만들고 있다. `/robot_pose` 값과 캘리브레이션 파라미터 점검 |
| `catkin_make`가 `robot_msgs`를 못 찾음 | `catkin_make --only-pkg-with-deps robot_msgs` 먼저 실행 |
| D405가 `Unsupported device! Product ID: 0x0B5B` | apt의 realsense2_camera(librealsense 2.50)를 쓰고 있다. 제거 후 `src/realsense-ros` 소스 빌드 (§3.1) |
| RealSense `UVC device is streaming!` | 이미 스트리밍 중인 장치에 `enable`을 재차 호출했다. 기동 시 드라이버 토글 금지 (§7.3) |
| side_cam / hand_cam 영상이 서로 바뀜 | launch의 `serial_no` 확인. `rs-enumerate-devices -s`로 대조 |

---

# 13. 개발 규약

## 13.1 경로

**`__file__`로 경로를 계산하지 말 것.** `apriltag_nav.paths`를 쓴다.

```python
from apriltag_nav.paths import PKG_DIR, CONFIG_PATH, MAP_PATH, TASK_DIR, MODEL_PATH
from apriltag_nav.paths import add_fairino_sdk_to_path
```

기존의 파일별 `__file__` 역추적은 "이 파일은 `<pkg>/scripts/`에 있다"를 전제했기 때문에
파일이 이동하자 깨졌다.

## 13.2 팔 컨트롤러 교체

| 파일 | IK 엔진 | 스캔 |
|---|---|---|
| `src/apriltag_nav/arm_controller.py` | Fairino SDK + q0 레퍼런스 | 있음 (기본) |
| `tools/arm_controller_sdk.py` | Fairino SDK (기본형) | 없음 |

교체는 `scripts/arm_controller_node.py`의 import 한 줄만 바꾼다.

> **감싸라, 다시 쓰지 말라.** `arm_controller_node.py`는 `ArmController`를 그대로
> **래핑**한다. 그 컨트롤러가 `TOOL_ID=1`(vision_tip TCP), q0 IK 시드, 4-DOF 변환,
> 13열 CSV, Keyence 루프를 모두 쥐고 있기 때문이다. 재구현하면 이들을 조용히 잃는다 —
> 특히 `TOOL_ID=1` 대신 `tool=0`(플랜지)을 쓰면 TCP 오프셋이 통째로 틀어진다.

> **`ArmController`의 모든 `~params`는 `arm_controller_node`의 프라이빗 네임스페이스에
> 두어야 한다** (`num_samples`, `save_images`, `output_dir`, `keyence_*` 등).
> `ArmController`가 그 노드 안에서 살기 때문이다. 다른 노드에 두면 읽히지 않고
> **조용히 기본값으로 폴백한다.**

## 13.3 task_executor ↔ 팔 노드

`task_executor`는 `ArmClient`(`src/apriltag_nav/arm_client.py`)를 통해 팔 노드와 통신한다.
두 가지만 알아 두면 된다.

- `execute_scan_points()`는 **비동기**다 — 발행 즉시 반환한다. 완료는 `/scan_finished`로
  오고, `task_executor`가 이를 기다린다. `move_to_home()`은 서비스라 동기다.
- 스캔 점은 `/arm/scan_command`에 JSON으로 실린다. 태스크 점이 pandas에서 오기 때문에
  `ArmClient`가 numpy 스칼라를 먼저 변환한다 — 원시 `json.dumps`는 `numpy.float64`에서 실패한다.

## 13.4 코딩 컨벤션

- Python: 클래스 `PascalCase`, 메서드 `snake_case`, 상수 `UPPER_CASE`
- **주석은 영어로 작성**
- ROS 토픽: 소문자 + `/` 구분, 스캔 결과는 `/scan/` 아래
- 튜닝 값은 전부 `config/robot.yaml`에 — **코드에 하드코딩 금지**
- **scipy는 ≥ 1.4가 필요하며 `as_matrix()` / `from_matrix()`를 쓴다.** 구 표기
  `as_dcm()` / `from_dcm()`은 scipy < 1.6에만 존재하는데, 그렇게 낮은 scipy는 여기서
  쓸 수 없다. numpy는 `onnxruntime` 때문에 ≥ 1.21.6이어야 하고, scipy < 1.6은 numpy가
  1.21에서 제거한 `np.typeDict`를 호출하므로 `as_dcm()`이 있는 scipy는 전부 import에
  실패한다.

## 13.5 도구 스크립트 (`tools/`)

설치되지 않으며, 워크스페이스를 source한 상태에서 직접 실행한다.

| 스크립트 | 용도 |
|---|---|
| `ra_map_plotter.py` | Ra 맵 CSV → 히트맵 PNG |
| `measure_keyence_angle.py` | 비스듬한 레이저 빔 각도 실측 |
| `test_all_devices.py` | 전 장치 + ROS 토픽 수신 점검 (읽기 전용) |
| `test_scan_chain.py` | Basler + VISION 램프 + Keyence + 팔 연동 점검 |
| `calibrate_transform.py`, `collect_calib_data.py` | 변환 캘리브레이션 |
| `validate_transform.py`, `validate_compare.py` | 오프라인 FK 검증 (roboticstoolbox) |
| `set_tool_tcp.py` | 툴 TCP 설정 |
| `send_debug_cmd.py`, `navigate.py`, `FrCmd.py` | 디버그 명령 전송 |

---

# 부록 A. ROS 인터페이스 요약

## A.1 이 워크스페이스가 소유하는 토픽

노드 이름은 실제 ROS 그래프 이름이다. `task_executor.py`가 띄우는 노드의 이름은
**`/mobile_manipulator_system`**이다(파일명과 다르므로 주의).

| 토픽 | 타입 | 발행자 | 구독자 | 설명 |
|---|---|---|---|---|
| `/task_command` | `std_msgs/String` | **사용자** (`rostopic pub`, 외부 UI) | `mobile_manipulator_system` | 태스크 명령 |
| `/robot_pose` | `robot_msgs/Pose2DWithFlag` | `mobile_manipulator_system` (RobotController) | `arm_controller_node` | 베이스 위치(매니퓰레이터 프레임) |
| `/arm/scan_command` | `std_msgs/String` | `mobile_manipulator_system` (ArmClient) | `arm_controller_node` | 스캔 점 JSON |
| `/arm/cancel` | `std_msgs/Bool` | `mobile_manipulator_system` (ArmClient) | `arm_controller_node` | 스캔 취소 |
| `/scan_finished` | `std_msgs/Bool` | `arm_controller_node` | `mobile_manipulator_system` | 스캔 완료 신호 |
| `/arm/status` | `std_msgs/String` | `arm_controller_node` | 없음 (진단용) | 팔 상태 |
| `/camera/set_active` | `std_msgs/Bool` | `arm_controller_node` (ScanPipeline) | `basler_camera_node` | 선오픈 / 강제 개폐 |
| `/camera/state` | `std_msgs/String` | `basler_camera_node` | 없음 (진단용) | `open` / `closed` |
| `/basler/image_raw` | `sensor_msgs/Image` | `basler_camera_node` | 없음 (디버그 뷰어용) | 마지막 촬영 프레임 |
| `keyence/value` | `std_msgs/Float32` | `keyence_dlen1_node` | `arm_controller_node` | 거리 측정값 |
| `keyence/raw` | `std_msgs/Int32` | `keyence_dlen1_node` | 없음 (진단용) | 원시 값 |
| `/<cam>/tag_detections` | `robot_msgs/AprilTagDetectionArray` | `robot_camera_node` | front만 `mobile_manipulator_system` (내비 + 비전 정지). side/hand는 소비자 없음 | 태그 검출 결과 |
| `/<cam>/tag_overlay` | `sensor_msgs/Image` | `robot_camera_node` | RViz 등 뷰어 | 구독자가 있을 때만 렌더 |
| `/scan/ra_value` | `std_msgs/Float32` | `arm_controller_node` (ScanPipeline) | 없음 (외부 소비용) | Ra 예측값 |
| `/scan/point_result` | `std_msgs/String` | `arm_controller_node` (ScanPipeline) | 없음 (외부 소비용) | 점별 결과 JSON |
| `/scan/image` | `sensor_msgs/Image` | `arm_controller_node` (ScanPipeline) | 없음 (외부 소비용) | 촬영 이미지 |

"없음"은 이 워크스페이스 안에 구독자가 없다는 뜻이다. 외부 도구나 `rostopic echo`로
관찰하라고 발행하는 토픽이며, 구독자가 없어도 정상이다.

`RobotController` / `TaskManager` / `MapManager` / `ArmClient`는 별도 노드가 아니라
`mobile_manipulator_system` 프로세스 **안에서** 도는 라이브러리다. 표의 괄호는 그
프로세스 안의 어느 객체가 실제로 발행하는지를 가리킨다.

## A.2 서비스

| 서비스 | 타입 | 서버(제공) | 클라이언트(호출) |
|---|---|---|---|
| `/camera/capture` | `robot_msgs/CaptureImages` | `basler_camera_node` | `arm_controller_node` (ScanPipeline) |
| `/arm/move_home` | `std_srvs/Trigger` | `arm_controller_node` | `mobile_manipulator_system` (ArmClient), 사용자 |
| `/robot_camera/<cam>/set_enabled` | `std_srvs/SetBool` | `robot_camera_node` | **사용자** (`rosservice call`) |
| `/camera_viewer/set_enabled` | `std_srvs/SetBool` | `camera_viewer_node` | **사용자** (`rosservice call`) |
| `/front_cam/toggle_color` | `std_srvs/SetBool` | orbbec 벤더 드라이버 | `robot_camera_node` |
| `/<side\|hand>_cam/enable` | `std_srvs/SetBool` | realsense 벤더 드라이버 | `robot_camera_node` |

## A.3 벤더 카메라 드라이버가 발행하는 토픽

| 토픽 | 타입 | 발행자 | 구독자 |
|---|---|---|---|
| `/<cam>/color/image_raw` | `sensor_msgs/Image` | 벤더 드라이버 (orbbec / realsense) | `robot_camera_node` |
| `/<cam>/color/camera_info` | `sensor_msgs/CameraInfo` | 벤더 드라이버 | `robot_camera_node`, front만 `mobile_manipulator_system` |

## A.4 Navifra 드라이버와 주고받는 토픽

이 워크스페이스 쪽은 전부 `NavifraDevices`(`navifra_devices.py`)가 감싼다.
`apriltag_nav` 안의 다른 코드가 직접 만지면 안 된다.

| 토픽 | 타입 | 발행자 | 구독자 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | `mobile_manipulator_system` (RobotController) | Navifra `base_controller` |
| `/odom` | `nav_msgs/Odometry` | Navifra `base_controller` | `mobile_manipulator_system` (RobotController) |
| `/safety/estop` | `std_msgs/Bool` | Navifra 안전 PLC 드라이버 | `mobile_manipulator_system` (중단 판단) |
| `/crevis/led/vision` | `std_msgs/Bool` | `basler_camera_node` | Navifra Crevis IO 드라이버 |
| `/crevis/led/status_{red,green,blue}` | `std_msgs/Bool` | `mobile_manipulator_system` | Navifra Crevis IO 드라이버 |
| `/bms/state` | `sensor_msgs/BatteryState` | Navifra BMS 드라이버 | `mobile_manipulator_system` (저전압 경고) |
| `/lift/*` | Bool / String / Int32 / Int16 | 양방향 (명령은 우리, 상태는 드라이버) | `navifra_devices.py` |

> VISION 램프와 STATUS 램프의 **발행자가 다르다**. VISION은 `basler_camera_node`가
> 셔터와 묶어 소유하고, STATUS는 `mobile_manipulator_system`이 태스크 상태에 따라
> 소유한다. 소유하지 않은 램프를 건드리면 두 노드가 서로 덮어쓴다(§7.1).

## A.5 실제 그래프 확인

```bash
rostopic info /front_cam/tag_detections   # Publishers / Subscribers 실측
rosnode info /robot_camera_node           # 한 노드의 발행·구독·서비스 전체
rqt_graph                                 # 그래프 시각화
```

## A.6 커스텀 메시지

**`robot_msgs/Pose2DWithFlag`**

```
std_msgs/Header header
float64 x          # 매니퓰레이터 프레임 X (m)
float64 y          # 매니퓰레이터 프레임 Y (m)
float64 theta      # 월드 프레임 방위 (deg)
float64 theta_web  # 웹 표시용 방위
bool    flag       # 유효성 플래그
int32   id         # 태그 ID
```

**`robot_msgs/AprilTagDetection`** (`AprilTagDetectionArray`가 프레임당 한 묶음으로 실어 발행)

```
int32      id                # AprilTag ID
float64    center_x, center_y   # 영상 내 태그 중심 (px)
float64    pose_x, pose_y, pose_z  # dt_apriltags pose_t (m)
float64    roll, pitch, yaw   # 카메라 프레임 기준 태그 자세, ZYX 내인 오일러 (deg)
float64[8] corners           # [x0,y0, ... x3,y3] px, dt_apriltags 순서
float64    tilt_from_normal  # 태그 법선과 광축 사이 각 (deg), 0이면 정면
```

그 외: `robot_msgs/NavDebugStatus`(내비게이션 디버그 상태), `frcobot_hw/status`(팔 하드웨어 상태).
서비스 정의 `robot_msgs/CaptureImages`는 `/camera/capture`가 쓴다.

---

# 부록 B. 미해결 항목 체크리스트

실 로봇 운용 전에 확인해야 할 것들이다.

| # | 항목 | 위치 | 상태 |
|---|---|---|---|
| 1 | `vision_stop.stop_tag_ids`가 빈 플레이스홀더 | `robot.yaml` | 태그를 봐도 정지하지 않음. 추측한 ID로 실 로봇 테스트 금지 |
| 2 | `arm_base_z` 1.025 vs 0.9541 (71 mm 차이) | `robot.yaml` `arm_calibration` | 알려진 리프트 위치에서 재검증 필요 |
| 3 | `navifra.scan_height_counts`가 `null` | `robot.yaml` | 스캔 높이 가드 비활성 |
| 4 | `robot.length` / `robot.width`가 구 베이스 값 | `robot.yaml` | 벽 여유 계산이 낙관적으로 잘못됨 |
| 5 | `keyence_max_step_mm` 1.0이 임시 브링업 값 | launch | 루프 신뢰 후 5.0 복원 |
| 6 | `navifra.require_safety_link`가 `false` | `robot.yaml` | 실 운용에서는 `true` 권장 |
| 7 | side_cam / hand_cam 검출 결과에 소비자 없음 | `robot_camera_node` | 두 카메라의 검출만 미사용. front_cam은 내비게이션이 소비하므로 노드 자체는 필수(§2.1) |
