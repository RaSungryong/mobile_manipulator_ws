# 프레임과 고정 기하 — 한 장 요약

이 문서는 모바일 매니퓰레이터의 **좌표 프레임, 그 사이의 고정 변환, 각 숫자가 사는 파일, 바꿀 때의 절차**를 한 곳에 모은 것이다. 숫자 자체는 여기 적지 않는다 — 숫자는 `src/apriltag_nav/config/tf/tf_chain.yaml`이 원본이고, `python3 src/apriltag_nav/tools/tf_chain_tool.py show`가 현재 값을 보여 준다. (작성 2026-10-13)

## 1. 프레임 트리

```
world  (셀/정반 좌표. 경로 CSV의 x y z, 맵의 태그 위치가 이 프레임)
  │  map.yaml  x, y, yaw  (+ log/…/map_world_<ts>.yaml  z, rpy)         ← 맵 캘리브레이션
  ▼
tag_<id>  (바닥의 AprilTag. 태그 윗면이 z = +tag_thickness)
  │  front_cam 검출 + /robot_pose                                       ← 라이브 (주행)
  ▼
mb   대차 몸체.  x 앞, y 왼쪽, z 위, 원점은 섀시 중심 아래 바닥
  ├─ T_mb2fc ─────────▶ fc        front_cam 실제 광학 프레임 (1.4° 기울어짐)
  │                      └─(ground_plane fit)─▶ fc_level  수평 가상 카메라 — 검출 토픽은 이 프레임
  └─ inv(T_ab2mb) + lift ─▶ ab    암 base_link (Fairino base)           ← lift는 라이브 (/lifter/height)
                              │  FK_urdf(q + dq_deg)   q = 컨트롤러 조인트 읽기, dq = 영점 보정
                              ▼
                             ee   플랜지 = URDF tool_Link = Fairino tool 0 = /arm/state 가 보고하는 자세
                              ├─ inv(T_hc2ee) ─▶ hc   hand_cam (D435) 광학 프레임
                              └─ T_ee2tip ─────▶ tip  vision_tip: Basler 프레임 중심이 스탠드오프에서 보는 점
```

## 2. 프레임 정의

| 이름 | 무엇 | 축 | 비고 |
|---|---|---|---|
| `world` | 셀 좌표. 경로 CSV·맵의 기준 | z 위, 정반 윗면 z = 0, 바닥 z = −0.080 | `/robot_pose` 메시지와 축이 다름: `world_x = −msg.y`, `world_y = −msg.x` (CLAUDE.md "Coordinate Frames") |
| `mb` | 대차 몸체 | x 앞, y 왼쪽, z 위, 원점 바닥 | `/robot_pose`가 고정하는 프레임 |
| `mobile_base` (URDF) | 플래너 URDF의 대차 링크 | **mb와 yaw 180° 다름** (+x가 뒤) | `mobile_to_base` 조인트 = T_ab2mb를 이 프레임에서 본 것. `tf_chain_tool.py urdf`가 변환해 출력 |
| `ab` | 암 base_link | Fairino 규약 | 암은 대차에 뒤로(180°) 얹혀 있음 |
| `j1_Link … j6_Link` | 암 링크 | URDF | 조인트 원점 = 조인트 축. j6 원점에서 플랜지까지 106 mm (DH d6) |
| `ee` = `tool_Link` | **플랜지 면** | j6와 같은 자세 | 컨트롤러의 활성 툴(tool 0). tool 1(tip)은 등록만 되고 비활성 (CLAUDE.md 2026-09-14) |
| `tip` = `vision_tip` | Basler 초점 | 플랜지와 같은 자세, 평행이동만 | pose 모드 CSV의 x y z는 이 점. `arm_controller`가 tip→플랜지로 되돌려 IK |
| `hc` | hand_cam 광학 | x 이미지 오른쪽, y 아래, z 광축 | 캘리브레이션 전용 |
| `fc` / `fc_level` | front_cam 실제 / 수평 가상 | 광학 규약 | `robot_camera.ground_plane.front_cam`이 켜져 있으면 검출은 `fc_level` |
| lift | ab가 mb에 대해 z로 움직이는 축 | — | **파일에 없음.** `T_ab2mb`는 리프트 원점에서 측정한 값이고 소비자가 라이브 높이를 더함 (`arm_transform lift_m`, `chain.compensate_T_ab2mb`) |

## 3. 규약

- **`T_X2Y` = 프레임 Y의 자세를 X에서 표현.** Y의 점 p는 `T_X2Y @ p`로 X가 된다. 4×4 행 우선, 미터. (`path_tag_locator.geometry`)
- yaml의 `t_mm` / `rpy_deg_xyz`는 사람이 읽는 표기(scipy extrinsic 'xyz', deg). **코드는 `matrix`만 읽는다.** 세 줄은 `tf_chain_tool.py set`이 항상 같이 쓴다.
- URDF `rpy`는 고정축 x→y→z (`R = Rz·Ry·Rx`), 미터·라디안. Fairino SDK는 mm·deg.
- 조인트 영점 `dq_deg`는 컨트롤러 읽기에 **더해서** 물리 각도가 된다. J1은 0 (J1 오프셋은 베이스 yaw = T_ab2mb가 흡수).
- `(T_hc2ee, dq_deg, T_ab2mb)`는 **한 세트**: 핸드아이나 hand_cam K가 바뀌면 영점과 T_ab2mb를 그 뒤에 다시 맞춘다.
- 경로 CSV의 `rx ry rz` 해석은 생성기마다 다름 — `robot.yaml arm_calibration.csv_euler` (현 플래너 `"ZYX"`). 새 생성기는 `tools/check_pose_vs_joint.py`로 확인하고 키만 바꾼다.

## 4. 숫자가 사는 곳

| 수치 | 원본 (손으로 적는 유일한 곳) | 종류 | 쓰는 쪽 | 읽는 쪽 | 자동 복사본 |
|---|---|---|---|---|---|
| `T_ab2mb` | `tf_chain.yaml` | calibrated | `chain_calib` → `tf_chain_tool.py set` | `arm_transform`(arm_node), locator 노드들 | `T_ab2mb.npz`, 플래너 URDF `mobile_to_base` |
| `T_mb2fc` | **`robot.yaml`** `robot.camera_offset`, `camera_lateral`, `tag_thickness`, `robot_camera.ground_plane.front_cam` | generated | `tf_chain_tool.py front-cam --apply` | locator 노드들 (`mobile_node`는 robot.yaml을 직접 읽음) | `tf_chain.yaml` 블록, `T_mb2fc.npz` |
| `T_hc2ee` | `tf_chain.yaml` | calibrated | `handeye_calib_node` compute | locator, chain_calib, handeye sweep | `T_hc2ee.npz` (locator.yaml / handeye_calib.yaml이 경로로 가리킴) |
| `T_ee2tip` | `tf_chain.yaml` | **design** (2026-10-12; 마지막 측정은 `measured_2026_09_22` 하위 블록) | `tf_chain_tool.py set` | `arm_controller` (pose 모드 tip→플랜지), `set_tool_tcp.py`, `generate_calibration_artifacts.py`, `handeye_sweep` | `T_ee2tip.npz`, 플래너 URDF `vision_tip_joint` |
| `dq_deg` | `config/tf/arm_joint_offsets.yaml` | calibrated | `tf_chain_tool.py joint-offsets --apply` | `tf_chain.arm_flange_T` (locator 체인), `joint_offset_cmd` (명령 측) | `.npz` |
| 암 링크 길이 (DH) | 플래너 URDF `j1…j6`, `tool` | vendor | — | `arm_fk.py`, 플래너 | — |
| 태그 맵 | `map.yaml` (x,y,yaw) | calibrated | `map_calibrator` | `map_manager`/`mobile_node`, locator | `log/…/map_world_<ts>.yaml` (z, rpy — 주행은 `latest`) ; `map_idle.yaml` = 설계 |
| 태그 크기·두께, 바닥 z | `robot.yaml` (`tag_size`, `robot_camera.tag_size`, `tag_thickness`, `arm_calibration.world_floor_z_m`) | design | 손 | 주행, 검출, locator | `locator.yaml tag.*`, `handeye_calib.yaml tag.*` (같은 값을 따로 들고 있음) |
| 스탠드오프 | `robot.yaml keyence.sensor_zero_mm` (16.5) | design | 손 | Keyence 루프 | `T_ee2tip` 설계값의 "케이스 끝 + 16 mm"가 이 값에 기댐 |

플래너 URDF = `src/frcobot_ros/frcobot_description/urdf/fr10v6_mobile_vision_0317_test.urdf`. 런타임 노드는 읽지 않는다 — Windows 플래너와 오프라인 도구만. **플래너 PC의 복사본은 손으로 옮긴다.**

## 5. 바꿀 때의 절차

공통: 바꾼 뒤 `python3 src/apriltag_nav/tools/tf_chain_tool.py check` (0 failed), 재시작은 아래 표. 로봇 PC: `sudo systemctl restart mobile-manipulator`.

| 바꾸는 것 | 절차 | 재시작 |
|---|---|---|
| `T_ab2mb` 재캘리브 | `chain_calib` 세션 → `tf_chain_tool.py set T_ab2mb --npz <result>` → `check` → `urdf`가 출력하는 `mobile_to_base` 줄을 플래너 URDF에 → 플래너 PC에 복사 | arm_node, 캘리브레이션 노드 |
| `T_hc2ee` 재캘리브 | `handeye_calib_node` compute (자동 기록) → 그 뒤 `dq_deg`와 `T_ab2mb`를 다시 맞춤 (한 세트) | 캘리브레이션 노드 |
| `T_ee2tip` 재측정 | `basler_tip_calib` → `tf_chain_tool.py set T_ee2tip --t-mm x y z` → `check` → `urdf`의 `vision_tip_joint` 줄을 플래너 URDF에 → **플래너가 경로 CSV를 다시 내보냄** → (tool 1을 쓸 거면) `set_tool_tcp.py` | arm_node |
| `T_mb2fc` (front_cam 재장착·재피팅) | `robot.yaml`의 `camera_offset` / `ground_plane.front_cam` 수정 → `tf_chain_tool.py front-cam --apply` | mobile_node, robot_camera_node, 캘리브레이션 노드 |
| 맵 | `map_calibrator` 세션 → `map.yaml` + `map_world_<ts>.yaml` 자동 | mobile_node |
| 조인트 영점 | `chain_calib/arm_offsets.py` → `tf_chain_tool.py joint-offsets --apply <session>/arm_offsets.npz` | arm_node, 캘리브레이션 노드 |
| **새 고정 변환 추가** (예: side_cam) | ① `tf_chain.yaml`에 블록을 한 번 손으로 만든다 (`kind`, `source`, `t_mm`, `rpy_deg_xyz`, `matrix` 줄 — 기존 블록 복사) ② `apriltag_nav/tf_chain.py`의 `NAMES`에 이름 추가 ③ 그 뒤로는 `set`으로 갱신 | — |

`tf_chain.yaml` 블록의 `kind:` 는 `calibrated` / `generated` / `design` 중 하나 — **generated 블록은 손으로 고치지 않는다** (다음 `front-cam --apply`가 덮어쓴다).

## 6. rviz로 보기 (2026-10-13, config 2단계)

`tf_chain_publisher.py`가 위 체인을 그대로 `/tf`, `/tf_static`으로 내보낸다 — **기본은 꺼져 있고**, 켜도 프레임이 추가될 뿐 스택의 동작은 바뀌지 않는다 (아무 노드도 구독하지 않음).

```bash
roslaunch apriltag_nav mobile_manipulator.launch use_tf_publish:=true   # 또는 스택 옆에서
rosrun apriltag_nav tf_chain_publisher.py
rviz -d $(rospack find apriltag_nav)/config/tf_chain.rviz               # 고정 프레임 mb
```

| TF 프레임 | 출처 | 갱신 |
|---|---|---|
| `world → mb` | `/robot_pose` (x, y, theta; `world_x = −y`, `world_y = −x`, mb 원점 z = `world_floor_z_m`) | 10 Hz |
| `mb → ab` | `inv(T_ab2mb)` + `/lifter/state`의 `height_mm` (리프트만큼 ab가 mb z로 올라감) | 10 Hz |
| `ab → ee` | `/arm/state`의 joints로 `FK(q + dq)` (locator 체인과 같은 `arm_flange_T`; joints와 pose가 안 맞으면 컨트롤러 pose로) | 10 Hz |
| `ab → ee_ctrl` | 컨트롤러가 보고한 TCP pose — dq가 적용 중일 때만, `ee`와 나란히 (둘의 차이 = 영점 보정 효과) | 10 Hz |
| `mb → fc`, `mb → fc_level` | `T_mb2fc`, 그리고 ground-plane 기울기를 뺀 수평 가상 카메라 | static |
| `mb → mobile_base_urdf` | Rz(180°) — 플래너 URDF의 대차 프레임 | static |
| `ee → tip`, `ee → hc` | `T_ee2tip`, `inv(T_hc2ee)` | static |

수학은 `apriltag_nav/tf_chain_frames.py`(순수 numpy), 노드는 `scripts/tf_chain_publisher.py`, 오프라인 검증은 `tools/check_tf_chain_publisher.py` (런타임이 쓰는 로더·함수와 프레임마다 대조: 27 checks).

## 7. 확인 도구

```bash
python3 src/apriltag_nav/tools/tf_chain_tool.py show      # 현재 값, 설계 대비, URDF가 가져야 할 줄
python3 src/apriltag_nav/tools/tf_chain_tool.py check     # yaml↔npz, robot.yaml↔T_mb2fc, URDF↔yaml, 강체성
python3 src/apriltag_nav/tools/check_pose_vs_joint.py     # 경로 CSV의 pose 행 ↔ joint 행 (FK), URDF 오프셋
python3 src/apriltag_nav/tools/check_joint_offsets.py     # 영점 보정 체인
python3 src/path_tag_locator/scripts/check_handeye_sweep.py
python3 src/apriltag_nav/tools/check_tf_chain_publisher.py   # TF 뷰어의 프레임 == 런타임 로더
```

관련: `tf_chain.yaml` 헤더(규약 원문), CLAUDE.md "Coordinate Frames" / "Transform Parameters", `docs/TF_CHAIN_CALIBRATION_STATUS_kr.md`, `src/path_tag_locator/docs/CALIBRATION_GUIDE_kr.md`.
