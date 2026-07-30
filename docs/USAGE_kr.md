# Mobile Manipulator 사용 가이드 (한국어)

Fairino FR10v6 6축 로봇팔 + Navifra 이동 베이스 통합 시스템의 운용 가이드.
AprilTag 시각 내비게이션 → 지정 위치에서 팔 스캔 → 표면조도(Ra) 예측을 수행한다.

- 대상 코드: `real` 브랜치, `apriltag_nav` 패키지
- 이동 베이스 드라이버: **Navifra KU Polishing Robot Driver v0.16**
  (별도 설치 `~/navifra`, 상세 스펙은 그쪽의 인터페이스 가이드 PDF 참조)
- 영문 기술 문서: `CLAUDE.md`(요약) / `README.md`(상세)

---

## 1. 시스템 구성

하드웨어 장치별로 독립 ROS 노드 1개씩 (Navifra 드라이버와 같은 패턴):

| 노드 | 담당 | 인터페이스 |
|------|------|-----------|
| `task_executor` | 태스크 오케스트레이션, STATUS 램프, 비상정지 감지, 배터리 감시 | `/task_command` |
| `arm_controller_node` | Fairino 팔 (TOOL_ID=1 vision_tip, q0 시드 IK, Keyence 폐루프) | `/arm/scan_command`, `/arm/cancel`, `/arm/move_home`(srv) |
| `basler_camera_node` | 손목 Basler 카메라 **+ VISION 조명** | `/camera/capture`(srv) |
| `keyence_dlen1_node` | Keyence DL-EN1 거리 센서 (192.168.100.105) | `keyence/value` |

이동 베이스(`/cmd_vel`·`/odom`), 리프트, 배터리(BMS), Safety PLC, Crevis 조명은
**Navifra 드라이버가 소유**한다. 이 워크스페이스는 `navifra_devices.py` 를 통해
토픽으로만 접근한다.

### 조명(LED) 소유권 — 반드시 지킬 것

- **VISION 램프** → `basler_camera_node` 전용. 셔터와 묶여서 촬영 중에만 점등.
  다른 노드가 `/crevis/led/vision` 을 발행하면 안 됨.
- **STATUS 램프(RGB)** → `task_executor` 전용. 상태 표시:

| 색 | 의미 |
|----|------|
| 초록 | IDLE (대기) |
| 파랑 | 이동 중 / 도착 |
| 시안 | 스캔 중 |
| 빨강 | 오류 / 비상정지 후 |

---

## 2. 기동 절차

```bash
# 1. Navifra 베이스 드라이버 (부팅 시 자동. 수동으로는:)
sudo systemctl start navifra-robot
systemctl status navifra-robot        # active 확인
# ※ 이 서비스가 roscore 를 이미 띄운다. 별도 roscore 실행 불필요.

# 2. 매니퓰레이터 스택 (4개 노드 일괄 기동)
cd ~/mobile_manipulator_ws
source devel/setup.bash
roslaunch apriltag_nav mobile_manipulator.launch
```

기동 확인:

```bash
rostopic echo -n1 /arm/status      # "idle" 이면 팔 노드 정상
rostopic echo -n1 /camera/state    # "closed" 가 정상 (카메라는 평소 꺼져 있음)
rostopic echo -n1 /safety/estop    # false 여야 정상
```

파라미터 오버라이드 예:

```bash
roslaunch apriltag_nav mobile_manipulator.launch keyence_tol:=0.1 num_samples:=3
```

---

## 3. 태스크 명령

`/task_command` (std_msgs/String) 토픽으로 전송:

```bash
# 스캔 태스크 (라인별)
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_grid_line1"
# 병합 스캔 → *_ra_map.csv 생성
rostopic pub -1 /task_command std_msgs/String "TASK scan_full_joints"
rostopic pub -1 /task_command std_msgs/String "TASK scan_full_pose"

# 홈 복귀 / 태그 이동(스캔 없음)
rostopic pub -1 /task_command std_msgs/String "TASK go_home"
rostopic pub -1 /task_command std_msgs/String "GOTO 104"

# 상태 조회 / 즉시 정지
rostopic pub -1 /task_command std_msgs/String "STATE"
rostopic pub -1 /task_command std_msgs/String "STOP"

# 포즈 테스트 (디버그)
rostopic pub -1 /task_command std_msgs/String "TEST_POSE 0.737 2.14 0.704"
```

`EXEC` / `EVAL` 디버그 명령은 기본 비활성 (`~debug_mode:=true` 필요, 운용 중 금지).

### 스캔 결과

13열 CSV (`group_id, point_id, x, y, z, ra_mean, ra_std, ra_min, ra_max,
num_samples, success, execution_message, validated_at`). 점별 증분 저장이므로
**중간에 STOP 해도 완료된 점의 결과는 보존**된다. 시각화:

```bash
python3 src/apriltag_nav/tools/ra_map_plotter.py <csv> --interpolate
```

---

## 4. 안전

### 비상정지

- 비상정지는 **하드웨어 회로**(PILZ PNOZmulti 2)가 담당한다. 비상버튼·범퍼가
  눌리면 PLC 가 모터 전원을 물리적으로 차단한다 — ROS 와 무관하게 동작.
- ROS 측(`/safety/estop`)은 **상태 피드백일 뿐**이다. task_executor 는 이를
  감지해 진행 중 태스크를 중단하고 ERROR 상태(빨강 램프)로 전환한다.
- 해제 후 재기동: 물리 리셋 → 새 TASK 명령 전송.

### 리프트 사용 시 주의 (중요)

팔 변환의 `arm_base_z` 는 **상수**다. 리프트를 움직이면 pose 모드 스캔
(`scan_grid_*`, `scan_full_pose`, `TEST_POSE`)의 목표 위치가 리프트 이동량만큼
**조용히** 틀어진다 (오류 없이 잘못된 높이로 이동함).

- joint 모드(`scan_joints_*`)는 기록된 관절각 재생이므로 **영향 없음**.
- 보정된 스캔 높이는 `robot.yaml` → `navifra.scan_height_counts` 에 기록
  (미기록 `null` 이면 가드 비활성). 상세 분석: `docs/lift_arm_base_z_analysis.md`
- 리프트는 전원 재투입마다 홈잉 필요 (`/lift/home` → 하부 리미트 원점).

### 배터리

`/bms/state` 잔량이 20% 미만이면 경고 로그. 임계값: `robot.yaml` →
`navifra.low_battery_pct`.

---

## 5. 카메라 동작 방식

Basler 는 발열·수명·전력 때문에 **상시 켜두지 않는다**:

- 평소 디바이스 **Close** 상태 (`/camera/state` = "closed").
- 스캔 점 시작 시 **선(先)오픈**: 팔이 이동을 시작하기 전에 카메라를 미리
  열어 두어, 오픈 지연이 이동+Keyence 조정 시간과 겹쳐 숨겨진다.
- 점별 흐름: 선오픈 → 이동 → 안정화 → Keyence 거리 조정 →
  `/camera/capture` (VISION 램프 점등 → 촬영 → 소등) → 추론.
- 스캔 종료/취소 시 즉시 닫힘. VISION 램프는 선오픈과 무관하게 **촬영
  순간에만** 점등된다 (셔터와 묶임).
- 수동 강제 개폐(진단용): `rostopic pub -1 /camera/set_active std_msgs/Bool "data: true"`

---

## 6. 문제 해결

| 증상 | 원인 / 조치 |
|------|------------|
| 스캔 점마다 "no frames captured" | `basler_camera_node` 미기동 또는 카메라 미연결. `/camera/state` 확인 |
| 태스크가 SCANNING 에서 멈춤 | `arm_controller_node` 미기동. `/arm/status` 확인 |
| "pose scan requires /robot_pose" | 내비게이션이 위치를 발행하지 않음. GOTO 로 태그 도착 후 스캔 |
| 기동 직후 `/safety/estop` = true | fail-safe 정상 동작: Navifra `safety_io_driver` 미기동이거나 PLC 통신 두절 |
| 팔이 엉뚱한 높이로 이동 (pose 모드) | 리프트가 보정 높이에서 벗어남 (§4 리프트 주의 참조) |
| Keyence 값 안 들어옴 | IP 192.168.100.105:64000 확인 (LAN1 허브), `keyence_dlen1_node` 로그 확인 |
| 직진이 휘거나 제자리회전 실패 | 구동모터 상태 확인 — `~/navifra/param.yaml` 의 `drive_motor_ids` 가 [1] 뿐이면 2번 모터 수리중 상태 |

---

## 7. 캘리브레이션 메모

- 팔 마운트 변환의 기준값: `path_tag_locator/config/extrinsics.yaml` `T_ab2mb`
  (실측: yaw 180° 정확, **틸트 없음**, 높이 1.025 m). `robot.yaml`
  `arm_calibration` 이 이 값을 따른다.
- 과거 USD 시뮬레이션 유래 값(높이 1.0076, 틸트 ±1.5°)은 폐기됨 — 실제
  플랫폼에 그 틸트는 존재하지 않는다.
- 리프트 높이별 재검증 전까지 정밀 pose 스캔의 잔차는 미확정 상태다.

수정할 때: 값은 전부 `config/robot.yaml` 에 있고 ROS `~` 프라이빗 파라미터로
오버라이드 가능. **코드에 하드코딩 금지.**
