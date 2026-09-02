# path_tag_locator 사용 가이드

> **현장 작업은 [CALIBRATION_GUIDE_kr.md](CALIBRATION_GUIDE_kr.md)를 먼저 보세요** —
> 최종 아키텍처 기준의 사용 가이드입니다. 아래 본문은 상세 참고용이며
> 일부 구버전 내용이 남아 있습니다.
>
> **⚠️ 2026-09-01 리팩터링:** 이 패키지는 하드웨어를 직접 소유하지 않습니다.
> 전용 `path_tag_locator.launch`로 시작하되 **메인 스택(mobile_manipulator.launch)이
> 먼저 떠 있어야** 하며, 하드웨어는 arm_node / mobile_node / robot_camera_node를
> 통해서만 접근합니다. 아래 문서의 세부 절차 중 일부(독립 launch, SDK 직결,
> robot_nav.yaml)는 과거 방식 기준이니 명령어는 이 머리말과 README를 우선하세요.
> ⚠️ 캘리브레이션 세션 중에는 TASK/GOTO를 내리지 마세요(/mobile/goto_tag 이중 지휘 금지).


## 0. 최초 빌드

```bash
cd ~/mobile_manipulator_ws
catkin_make --pkg path_tag_locator
source devel/setup.bash
```

의존성 확인 (없는 항목만 설치):

```bash
python3 -c "import dt_apriltags, cv_bridge, cv2, numpy, yaml, rospkg"
pip install dt_apriltags        # 누락 시
```

(2026-09-01 이후 SDK 직결 없음 — 팔은 arm_node 의 /arm/state + /arm/move_cart 경유.)

---

## 1. Hand-eye 캘리브레이션 (`T_hc2ee.npz`, 로봇 1대당 1회)

`config/hand_eye/T_hc2ee.npz` 를 만드는 두 가지 경로:

- **A. 정상 보정 세션**: `handeye_calib_node` 로 실제 캡처를 모아 `cv2.calibrateHandEye` 를 돌린다 (다음 절). 권장.
- **B. 직접 입력**: 외부 (CAD, 데이터시트, 이전 보정값) 에서 측정한 값을 `config/hand_eye/T_hc2ee.yaml` 에 채워서 변환. 빠른 부트스트랩 / nominal mounting offset 으로 시작할 때 유용.

```bash
# 1. config/hand_eye/T_hc2ee.yaml 의 position_m + rpy_deg (또는 matrix_4x4) 수정
# 2. yaml -> npz 변환:
rosrun path_tag_locator save_npz.py             # 기존 npz 가 있으면 거부 (덮어쓰지 않음)
rosrun path_tag_locator save_npz.py --force     # 덮어쓰기 허용
```

> **주의**: `T_hc2ee.npz` 를 새로 쓴 뒤에는 `path_tag_locator_node` 를 **재시작**해야 반영됨 (노드가 기동 시 1회 로드).

### 1.A 정상 보정 세션 (권장)

### 1a. `config/handeye_calib.yaml` 설정

```yaml
handeye_calib:
  topics:
    hand_cam_image:  "/hand_cam/image_raw"      # ← 실제 토픽으로 변경
    hand_cam_info:   "/hand_cam/camera_info"
  tag:
    id:        58                                # ← 캘리브레이션용 태그 (= 이후 기준 태그 A)
    size_m:    0.090                             # ← 태그 한 변 길이 (m, 2026-09 90mm)
  robot:
  # (robot: 블록 삭제됨 — TCP pose 는 /arm/state 에서 읽음)
  io:
    output_path: "$(find path_tag_locator)/config/hand_eye/T_hc2ee.npz"
    min_samples: 8                               # ← 권장 ≥ 15
```

### 1b. 노드 실행

```bash
roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true
```

### 1c. 샘플 수집 (각 자세마다 capture 1회)

암을 태그가 보이는 다양한 자세로 옮기면서 (각도/거리를 분산시킬수록 좋음), 매번:

```bash
# 1회 캡처 — 즉시 디스크 아카이브 (~/.ros/path_tag_locator/handeye_calib/run_<ts>/) 됨
rosservice call /handeye_calib/capture "{}"

# 진행 상황 확인
rosservice call /handeye_calib/status  "{}"

# 메모리 + 새 run_<ts>/ 시작 (이전 디스크 아카이브는 보존됨)
rosservice call /handeye_calib/reset   "{}"
```

### 1d. 캘리브레이션 실행 (npz 저장)

```bash
rosservice call /handeye_calib/compute "{}"
```

응답에는 BEST method, residual, T_hc2ee 의 translation 과 RPY 가 포함됨.
결과 파일은 `config/hand_eye/T_hc2ee.npz` 에 저장됨.

> **참고**: `T_hc2ee.npz`는 노드가 시작 시 1회만 로드합니다. 새로 보정한 뒤에는 `path_tag_locator` 노드를 **재시작**해야 적용됩니다.

### 1e. 기존 샘플 재사용 (resume)

캡처 도중 노드를 종료해도 디스크 아카이브는 남아 있습니다. 재시작 후 두 가지 방법으로 이어서 진행할 수 있습니다.

**방법 A — 가장 최근 run을 메모리에 적재 (가장 간단)**

```bash
roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true

# 이전 세션의 가장 최신 run_<ts>/ 를 자동 검색해서 적재
rosservice call /handeye_calib/load_latest "{}"
# → loaded N sample(s) from .../run_20260521_133900 (total in memory: N)

rosservice call /handeye_calib/status  "{}"   # samples: N (>= min_samples 인지 확인)
rosservice call /handeye_calib/compute "{}"   # 바로 보정
```

**방법 B — 특정 디렉터리 지정 (`config/handeye_calib.yaml`)**

```yaml
handeye_calib:
  io:
    load_samples_dirs:
      - "~/.ros/path_tag_locator/handeye_calib/run_20260521_133900"
      - "~/.ros/path_tag_locator/handeye_calib/run_20260521_142500"   # 여러 세션 합치기 가능
```

노드 시작 시 자동으로 메모리에 적재됩니다. 각 항목은 `run_<ts>/` 디렉터리 또는 그 아래의 `samples/` 디렉터리 모두 지원합니다.

**방법 C — 적재 후 추가 캡처**

`/load_latest` 또는 `load_samples_dirs`로 적재한 다음에 `/capture`를 더 호출하면 두 소스가 합쳐져 `/compute`에 들어갑니다. 자세 다양성이 부족할 때 유용합니다.

```bash
rosservice call /handeye_calib/load_latest "{}"   # 과거 8개 적재
rosservice call /handeye_calib/capture "{}"       # 새 자세 추가
rosservice call /handeye_calib/capture "{}"
rosservice call /handeye_calib/status  "{}"       # samples: 10
rosservice call /handeye_calib/compute "{}"
```

> **주의**: 적재된 샘플은 새 `recorder` run에 **다시 저장되지 않습니다** (중복 방지). `/compute` 결과 `result.npz`는 새 `run_<ts>/`에 별도로 저장됩니다.

---

## 2. 기준 태그 및 플랫폼 기하 설정

### 2a. 기준 태그 월드 좌표 `config/reference_tag.yaml`

```yaml
reference_tag:
  format: "pose"                         # 또는 "matrix"
  position_m: [1.234, 0.567, 0.890]      # ← 사용자가 측정한 태그 A 의 실제 월드 좌표
  rpy_deg:    [0.0, 0.0, 90.0]           # ZYX 내재 (intrinsic)
  # 또는:
  # format: "matrix"
  # matrix_4x4: [r00, r01, ..., 1]
```

### 2b. 플랫폼 기하 `config/extrinsics.yaml`

`T_ab2mb` (mb의 ab 표현), `T_mb2fc` (fc의 mb 표현) 의 기본값이 채워져 있음. **좌표 규약**: `T_X2Y` = "Y 프레임의 X 프레임 내 표현" = Y → X 좌표 변환. 플랫폼 개조(arm 마운트/카메라 마운트 변경) 시에만 수정.

기본값은 USD 모델 기준이므로 실제 하드웨어와 차이가 있으면 재측정하세요(`T_ab2mb`는 arm base 가 mobile base 위 어디에 있는지, `T_mb2fc`는 front-cam이 mobile base 어디에 어떤 방향으로 마운트됐는지).

### 2c. 메인 설정 `config/locator.yaml`

| 필드 | 의미 |
|------|------|
| `topics.hand_cam_*` / `front_cam_*` | 양쪽 카메라 토픽 |
| `tag.tag_a_id` / `tag_b_id` | 기준 / 경로 태그 ID |
| `tag.tag_a_size_m` / `tag_b_size_m` | 태그 한 변 길이 (m) |
| `arm.state_topic` / `arm.move_cart_topic` | arm_node 프록시 |
| `align.*` | 자동 정렬 허용 오차 및 step 크기 |

---

## 3. Localization 노드 실행

```bash
roslaunch path_tag_locator path_tag_locator.launch  # 메인 스택이 먼저 떠 있어야 함
```

노드가 제공하는 인터페이스:
- **Service**: `/path_tag_locator/locate_path_tag` (타입 `path_tag_locator/LocatePathTag`)
- **Topic** (latched): `/path_tag_locator/tag_world_pose` (`geometry_msgs/PoseStamped`)

`/handeye_calib` 노드 인터페이스(전부 `std_srvs/Trigger`):

| 서비스 | 동작 |
|--------|------|
| `~capture` | 현재 이미지+K+TCP 한 세트 캡처. 디스크에도 즉시 저장. |
| `~compute` | 메모리의 모든 샘플로 `calibrateHandEye` 실행. `T_hc2ee.npz` 갱신 + 새 `run_<ts>/result.{npz,yaml}` 저장. |
| `~status`  | 현재 샘플 수, 마지막 결과 요약. |
| `~reset`   | 메모리 비움 + 새 `run_<ts>/` 디렉터리 시작 (디스크 기록은 보존). |
| `~load_latest` | `run_root` 아래 **현 세션을 제외한 가장 최신** `run_*/` 디렉터리에서 샘플을 메모리에 적재. |

---

## 4. 서비스 호출

### 4a. 가장 단순한 형태: yaml 기본값 사용, 자동 정렬 끔

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: -1,
  override_ref: false,
  ref_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}},
  save_result: true,
  save_dir: '',
  auto_align: false,
  align_initial_tcp_mm_deg: [0,0,0,0,0,0]
}"
```

### 4b. 특정 경로 태그 지정

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: 12,
  override_ref: false,
  ref_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}},
  save_result: false,
  save_dir: '',
  auto_align: false,
  align_initial_tcp_mm_deg: [0,0,0,0,0,0]
}"
```

### 4c. 기준 태그 월드 좌표를 일시적으로 덮어쓰기

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: -1,
  override_ref: true,
  ref_pose: {
    position: {x: 1.500, y: 0.200, z: 1.000},
    orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}
  },
  save_result: true,
  save_dir: '',
  auto_align: false,
  align_initial_tcp_mm_deg: [0,0,0,0,0,0]
}"
```

### 4d. 자동 정렬 활성화

로봇이 먼저 `align_initial_tcp_mm_deg` 로 MoveJ → 태그 A 를 화면 중앙으로, 카메라가 태그에 수직이 되도록 반복 정렬 → 그 다음 결과 계산:

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: -1,
  override_ref: false,
  ref_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}},
  save_result: true,
  save_dir: '',
  auto_align: true,
  align_initial_tcp_mm_deg: [300, 0, 400, 180, 0, 0]
}"
```

`align_initial_tcp_mm_deg` 단위는 `mm, mm, mm, deg, deg, deg` (FR5 ZYX). hand-cam 이 태그 A 를 볼 수 있는 대략적인 위치만 주면 됨. 응답의 `align_iterations_used`, `align_final_xy_offset_m`, `align_final_tilt_deg` 로 정렬 결과를 확인.

저장되는 `result.yaml`(`~/.ros/path_tag_locator/locate/<날짜>/run_*/`)에는 **카메라 좌표계 기준** 오차가 기록된다
(2026-09-02). `observations:` 블록에 hand_cam→태그 A, front_cam→태그 B 각각의 `position_m` (x=영상 오른쪽,
y=영상 아래, z=광축 방향 거리) 와 `rpy_deg` 가 들어가고, `auto_align: true` 였으면 `auto_align.tag_in_cam` (최종)
과 `auto_align.history` (반복마다) 가 추가된다. 축 정의는 파일 상단 `camera_frame_note` 에 적혀 있다.

정렬 동작 조정: `config/locator.yaml` 의 `align:` 블록 수정:
- `max_iterations: 1` → one-shot 모드
- `position_tol_m: 0.005` / `angle_tol_deg: 1.0` → 수렴 임계값
- `max_step_m: 0.10` / `max_step_deg: 15.0` → 1 step 안전 clamp

---

## 5. 지도 일괄 보정 (`map_calibrator_node`)

`map.yaml`에 있는 **모든 path tag 의 (x, y)를 한 세션으로 재보정**해서
`map_updated.yaml`을 산출하는 자율 워크플로. 단일 `locate_path_tag` 호출을
플랜에 따라 순차로 호출하면서, base 이동까지 패키지 내부 nav 로직(`apriltag_nav`
런타임 의존 없음)으로 처리한다.

### 5a. 설정 파일

| 파일 | 용도 |
|------|------|
| `config/reference_tags.yaml` | **사용자가 측정한 정확한** 기준 tag 들의 월드 6-DOF. (id → position_m + rpy_deg, 또는 4×4 matrix). 모든 path tag 가 이 기준에 대해 보정됨. |
| `config/calibration_plan.yaml` | `path_tag_id` → `ref_tag_id` 순차 매핑. 항목별 `arm_view_tcp_mm_deg` (hand-cam이 ref tag를 볼 초기 자세) / `nav_start_id` (base nav BFS 시작 tag) override 가능. |
| (map.yaml 로컬 복사본 삭제됨) | `$(find apriltag_nav)/config/map.yaml` 이 단일 기준 (map_calibrator.yaml) |
| `config/map_calibrator.yaml` | 오케스트레이터의 기본 파일 경로들 (service call 시 override 가능). |

**세션 기록 (2026-09-02):** 세션마다 `~/.ros/path_tag_locator/calibrate/<YYYYMMDD_HHMMSS>/` 폴더가
새로 생기고, **시도(attempt)마다** `entries/001_tag105_attempt1_fail.yaml`, `002_tag105_attempt2_ok.yaml`,
`003_tag106_attempt1_ok.yaml` … 처럼 실행 순서대로 번호가 붙은 파일이 남는다. 재시도도 새 번호를 받고,
실패한 시도도 거기까지 진행된 내용(nav, view pose, 정렬 보고, 오류)을 그대로 담는다. 아무것도 덮어쓰지
않는다. 각 파일에는 카메라 좌표계 기준 관측(`observations`), 자동 정렬 보고(`auto_align.tag_in_cam` /
`history`), TCP, 리프트 높이, 월드 결과와 4x4 변환이 들어 있고, `session.yaml`(순서 인덱스),
`entries_log.csv`(한 줄 요약), `map_world.yaml`(세션 결과 사본)이 같은 폴더에 함께 저장된다.
`dry_run: true` 는 아무것도 기록하지 않는다.

### 5b. 실행

```bash
# 사전 조건:
# 메인 스택(mobile_manipulator.launch)은 반드시 떠 있어야 함 — 세션 중 TASK/GOTO 금지.
# - base 는 시작 시점에 플랜의 첫 tag 위에 세워 두어야 함 (정반 1: 100, 정반 2: 126).
#   2026-09-02 부터 플랜에 nav_start_id 가 없어 도크 500 을 거치지 않고 그 자리에서 바로 시작함.
#   front-cam 이 그 tag 를 보고 있어야 첫 이동의 출발 노드가 잡힘

roslaunch path_tag_locator path_tag_locator.launch  # 메인 스택이 먼저 떠 있어야 함

# Dry-run: 실제 이동 없이 plan 파싱과 ref/map 로딩만 검증
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: true
}"

# 실제 보정 실행
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: false
}"

# 진행 상황 모니터링 (항목별 JSON)
rostopic echo /map_calibrator/progress
rostopic echo /map_calibrator/current_target_tag
```

서비스 응답: `success` (= num_failed==0), `message`, `num_succeeded`,
`num_failed`, `output_yaml_path`. 실패한 entry 가 있어도 세션이
중단되지 않고 다음 entry 로 진행한다 — 실패 tag 는 `map_updated.yaml`
에서 원래 (x, y) 를 유지하며, `~/.ros/path_tag_locator/locate/.../`
아래 `run_*_FAILED/` 디렉터리에 에러와 request echo 가 보존된다.

### 5c. 항목 1개당 실행 흐름

```
1. base nav -> entry.nav_start_id (있으면)
2. base nav -> entry.path_tag_id   (map.yaml 의 edges 를 BFS 로 탐색)
3. arm MoveJ -> entry.arm_view_tcp_mm_deg
                (defaults.arm_view_tcp_mm_deg 가 fallback)
4. run_auto_align : hand-cam 을 ref tag 위로 정렬 (locate 시와 동일 알고리즘)
5. hand-cam + front-cam 영상 + live TCP 캡처
6. compute_T_A2B -> compute_T_B_world (= T_A_world · T_A2B)
7. map_data['tags'][path_tag_id]['x','y'] 갱신
8. atomic write -> map_updated.yaml (tmp + os.replace)
9. persistence.save_locate_run(...) 으로 6-DOF + 원본 영상 아카이브
```

### 5d. map.yaml 이 부정확해도 결과가 정확한 이유

체인은 **AprilTag 의 직접 관측**(`T_A2hc`, `T_fc2B`) + **하드웨어 보정값**
(`T_hc2ee`, `T_ab2mb`, `T_mb2fc`) + **실측 TCP** 로 구성된다. `map.yaml`
은 그 어느 항에도 들어가지 않는다. `T_B_world = T_A_world · T_A2B` 에서
`T_A_world` 는 사용자가 준 정확한 값이므로 최종 결과는 `map.yaml`
정확도와 무관하다.

`map.yaml` 의 (부정확한) 값은 단지 2단계의 arm 초기 자세를 계산하는
**대략적 시드** 로만 사용되며, 4단계의 `run_auto_align` 이 그 오차를
실측 기반으로 완전히 흡수한다.

### 5e. 첫 번째 보정 — 단계별 가이드

**path tag 1 개를 처음부터 끝까지** 보정해 보는 가이드. 한번 성공한 뒤에는
plan 에 entry 만 더 넣으면 됨.

#### Phase 0 — 물리 셋업 (1회)

1. **id=0 인 AprilTag** 을 "world 원점" 으로 정한 위치에 부착 (벽 모서리,
   바닥 마킹, 고정 구조물 등).
2. 나머지 ref tag (예: id 100, 502) 도 부착 후, tag 0 기준 상대
   `(Δx, Δy, Δz, yaw)` 을 줄자로 측정.
3. 보정할 path tag 1 개 (예: id 101) 를 base 가 접근 가능하고 front-cam
   으로 보이는 위치에 배치.

#### Phase 1 — Hand-eye `T_hc2ee.npz` (1회)

둘 중 하나:

```bash
# A. 빠른 부트스트랩: yaml 에 측정/설계값 직접 입력
nano src/path_tag_locator/config/hand_eye/T_hc2ee.yaml
rosrun path_tag_locator save_npz.py             # 기존 파일 거부
rosrun path_tag_locator save_npz.py --force     # 덮어쓰기

# B. 실제 보정: 15-30 개 자세 + cv2.calibrateHandEye
roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true
rosservice call /handeye_calib/capture "{}"     # 자세마다 반복
rosservice call /handeye_calib/compute "{}"
```

#### Phase 2 — 설정 파일 작성

`config/reference_tags.yaml` — 측정값 ground truth:

```yaml
format: "pose"
reference_tags:
  - id: 0
    position_m: [0.0, 0.0, 0.0]
    rpy_deg:    [0.0, 0.0, 0.0]
  - id: 100
    position_m: [1.500, 0.200, 0.0]
    rpy_deg:    [0.0, 0.0, 0.0]
```

`config/calibration_plan.yaml` — **첫 entry 1 개만**:

```yaml
defaults:
  align_required: true
  arm_view_tcp_mm_deg: [350.0, 0.0, 250.0, 180.0, 0.0, 0.0]   # placeholder, Phase 3 에서 교체
plan:
  - path_tag_id: 101
    ref_tag_id:  0
    nav_start_id: 500       # base nav 시작 tag (예: DOCK)
```

`config/locator.yaml` / `config/handeye_calib.yaml` — 실제 카메라
검출 토픽 (`/hand_cam/tag_detections`, `/front_cam/tag_detections`)
과 `detector:` 태그 크기가 robot.yaml 과 일치하는지 확인.

#### Phase 3 — `arm_view_tcp_mm_deg` 측정 (수동, 1회당 1번)

이 한 단계만 수동임. hand-cam 이 ref tag 0 을 대략적으로 보는 arm 자세를
찾아 TCP 값을 기록:

```bash
# 1. path_tag 101 앞에 base 를 수동으로 정차 (front-cam 으로 tag 가 보이게).
#    teach pendant / joystick / 임시 apriltag_nav 등 어느 것이든.

# 2. base 정차 후, teach pendant 로 arm 을 jog 해서 hand-cam 이 ref tag 0
#    을 보도록 함 (rqt_image_view /hand_cam/color/image_raw 로 확인).
#    중심에 두려고 애쓸 필요 없음 — auto_align 이 정조정.

# 3. 현재 TCP pose 읽기:
python3 -c "
from fairino import Robot
r = Robot.RPC('192.168.58.2')
err, pose = r.GetActualTCPPose()
print(pose)   # [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
"

# 4. 6 개 숫자를 calibration_plan.yaml 의 defaults.arm_view_tcp_mm_deg
#    (혹은 그 entry 의 override) 에 붙여넣기.
```

#### Phase 4 — 충돌 노드 종료 + calibrator 실행

```bash
# 1. /cmd_vel 발행하는 노드 종료 (apriltag_nav 메인 컨트롤러 등)
rosnode kill /navigate /robot_controller /task_executor 2>/dev/null || true

# 2. base 를 nav_start_id (DOCK 500) 앞에 정차 — front-cam 이 그 tag 를
#    보고 있어야 BFS 의 출발점이 잡힘.

# 3. calibrator 실행
roslaunch path_tag_locator path_tag_locator.launch  # 메인 스택이 먼저 떠 있어야 함
```

#### Phase 5 — Dry-run → 실제 보정

```bash
# 5a. Dry-run: 이동 없이 plan / ref_tags / map.yaml 파싱만 검증
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: true
}"
# 예상: success=true, num_succeeded=0, num_failed=0 (dry_run 은 성공 카운트 X)

# 5b. 진행 모니터링 (다른 터미널)
rostopic echo /map_calibrator/progress

# 5c. 실제 보정
rosservice call /map_calibrator/run_calibration "{
  plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '',
  dry_run: false
}"
```

그 entry 한 개에 대해 내부에서 일어나는 일:
1. base nav: `500 → … → 101` (front-cam tag 보면서 감속해서 정렬)
2. arm `MoveJ` → 너의 `arm_view_tcp_mm_deg`
3. `auto_align` (보통 3-5 iter) — hand-cam 이 tag 0 을 중앙에 맞춤:
   ```
   auto_align iter 1/5: xy=0.040 m, tilt=3.2 deg, z=0.250 m
   auto_align iter 3/5: xy=0.003 m, tilt=0.4 deg
   auto_align: converged at iteration 3
   ```
4. hand-cam + front-cam 신선한 이미지 + TCP 실독
5. 체인 계산 → `map_world_<ts>.yaml` 갱신

#### Phase 6 — 결과 검증

```bash
# yaml 출력
ls -lt ~/.ros/path_tag_locator/map_world_*.yaml | head -1
cat <그 파일>
# 기대: tag 101 의 position_m / rpy_deg, ref_tag_id=0, map_xy

# Per-tag 아카이브 (원본 이미지 확인)
ls ~/.ros/path_tag_locator/locate/$(date +%Y%m%d)/run_*_tag101/
# hand_cam.png: ref tag 0 이 중앙 근처, 기울기 < ~5°
# front_cam.png: path tag 101 이 선명히 보임
```

#### Phase 7 — Plan 확장 (auto view-pose bootstrap)

**첫 entry 가 성공한 뒤로** 오케스트레이터가 base anchor (mb 의 world
pose + /odom yaw + path tag map.yaml xy) 를 저장하고, 이후 entry 에서
`arm_view_tcp_mm_deg` 가 명시되지 않으면 **자동 계산**한다.

추정에 사용되는 정보:
- 앞 anchor + Δ(path tag map.yaml xy) + Δ(/odom yaw) → 새 `T_world2mb`
- 정확한 `T_A_world` + 보정 상수 (`T_hc2ee`, `T_ab2mb`)
- → `arm_view_tcp_mm_deg`

`T_map2world` 가 **필요 없음** — map.yaml 의 *상대* 기하 (절대 frame
정렬은 몰라도 상관없음) 만으로 충분히 정확한 시드를 만들고,
`run_auto_align` 이 cm/deg 단위 잔차를 흡수한다.

entry 별 우선순위:

1. `entry.arm_view_tcp_mm_deg` (yaml override) — 최우선
2. Auto-bootstrap (= `align.auto_view_pose: true` + 직전 entry 성공)
3. `defaults.arm_view_tcp_mm_deg`

`align.auto_view_pose: false` 로 끄면 항상 defaults 사용.

확장 plan 예:

```yaml
plan:
  - path_tag_id: 101
    ref_tag_id: 0
    nav_start_id: 500
  - path_tag_id: 102            # arm_view_tcp_mm_deg 자동 추정
    ref_tag_id: 0
  - path_tag_id: 103            # 또 자동 추정
    ref_tag_id: 0
  - path_tag_id: 405
    ref_tag_id: 502             # ref 가 바뀌어도 bootstrap 동작
```

특정 entry 의 자동 추정이 안 좋으면 (= ref tag 가 화면 가장자리) 그 entry
만 override 추가:

```yaml
  - path_tag_id: 405
    ref_tag_id: 502
    arm_view_tcp_mm_deg: [420.0, -10.0, 320.0, 175.0, 0.0, 5.0]
```

#### `reference_tags.yaml` 의 회전 관습 주의

체인은 AprilTag 라이브러리 관습을 따른다: tag 의 local **z 축은 인쇄면
반대 방향** (tag 안쪽) 을 향한다. 선언하는 `rpy_deg` 는 각 tag 의 **물리적
설치 방향**과 일치해야 한다:

| 물리적 설치 | tag 의 +z 방향 | yaw=0 일 때 `rpy_deg` |
|-------------|------------------|-----------------------|
| Face DOWN (아래를 향함; 천장 / 구조물 부착 — **본 프로젝트 기본 관습**) | world +z (위) | `[0.0, 0.0, 0.0]`     |
| Face UP   (위를 향함; 바닥 스티커 흔한 경우)                | world -z (아래) | `[180.0, 0.0, 0.0]`   |

동봉된 `reference_tags.yaml` 예시는 face-down 기준으로 `[0, 0, 0]` 을
사용한다. 만약 face-up 으로 바꾸면 rpy 도 `[180, 0, 0]` (또는
`[0, 180, 0]`) 으로 함께 바꿔야 한다. 이 부분이 어긋나면 체인 결과가
tag 평면 기준 180° 어긋난 위치로 나온다.

(이전 안내 계속:)

#### 첫 보정 시 흔한 함정

| 증상 | 처방 |
|------|------|
| 첫 entry 에서 `base nav … failed` | base 가 `nav_start_id` 앞에 없거나 front-cam 이 tag 를 못 봄 |
| `tag A (id=0) not detected` | `arm_view_tcp_mm_deg` 가 jog 시점과 다름; Phase 3 재측정 |
| `auto_align: clamped` 반복 | step throttling, 정상 동작 (다음 iter 가 계속 진행) |
| `auto_align` xy 가 >5 cm 에서 멈춤 | `T_hc2ee` 부정확 → Phase 1 재진행; 또는 `align.position_tol_m` 완화 |
| 결과 `position_m` 이 1 m+ 어긋남 | ref_tags 측정 오류; yaw 반대로 설치; hand_cam.png 가 다른 tag 를 봄 |
| Fairino `MoveJ` 실패 | 컨트롤러에서 `tcp_index=1` 도구 비활성; 또는 `arm_view_tcp_mm_deg` 가 reach 밖 |

### 5e+. 표보정 후 검증 스크립트

세션이 끝난 뒤 결과를 검증할 수 있는 4 개 보조 스크립트 (대부분 로봇
없이 동작):

| 스크립트 | 로봇 필요 | 용도 |
|----------|----------|------|
| `verify_map_world.py` | ❌ | 각 tag 의 summary + `map.yaml` 의 edge 그래프 기반 상대 거리 비교. `--threshold-m` (기본 5 cm) 초과 시 표시. |
| `test_repeatability.py` | ✓ (전체 재실행) | `/map_calibrator/run_calibration` 을 두 번 호출하고 두 결과를 diff. 평균 \|Δxy\| 가 체인의 실제 반복정밀도. |
| `verify_arm_pointing.py` | ✓ (arm + 카메라 + base 정차) | 각 tag 의 world 좌표로 view pose 계산 → MoveJ → hand-cam 재검출 → 잔차 측정. 가장 강한 폐루프 검증. |
| `visualize_map_world.py` | RViz 만 | 빨간색 ref tag + 녹색 calibrated tag + world 원점 축을 `MarkerArray` 로 publish. |

```bash
# 오프라인 (로봇 X)
rosrun path_tag_locator verify_map_world.py
rosrun path_tag_locator verify_map_world.py --threshold-m 0.10   # 더 느슨

# 반복성 (같은 plan 재실행, base 출발점으로 되돌리기 필요)
rosrun path_tag_locator test_repeatability.py

# Arm pointing — base 를 해당 tag 앞에 정차 후:
rosrun path_tag_locator verify_arm_pointing.py --tag-id 101
# 또는 여러 tag 순차 점검:
rosrun path_tag_locator verify_arm_pointing.py --all

# RViz 시각화 (다른 터미널에서 rviz, Fixed Frame=world)
rosrun path_tag_locator visualize_map_world.py
```

실패한 entry 의 원인 추적: `~/.ros/path_tag_locator/locate/<YYYYMMDD>/run_<ts>_tag<id>/`
의 `hand_cam.png` / `front_cam.png` 를 열어 확인.

### 5f. 좌표계: `map.yaml` ≠ 사용자의 world frame

`map.yaml` 의 (x, y) 는 apriltag_nav 가 쓰는 **Manipulator/Map frame**.
`reference_tags.yaml` 의 좌표는 사용자가 정의하는 **world frame**
(보통 어떤 ref tag 를 원점으로 잡음 — 예시는 `id: 0` 을 단위행렬로
선언). 두 frame 사이의 변환은 본 패키지가 정확히 측정할 수 없으므로
**합치려 시도하지 않는다**.

따라서:
- `map.yaml` 은 read-only. 보정 결과를 그 위에 덮어쓰지 않음.
- base 이동은 여전히 `map.yaml` 을 사용 (Pure-Pursuit + BFS 의 시드로
  충분하며, cm 단위 오차는 visual servoing 이 흡수).
- 체인 결과 `T_B_world = T_A_world · T_A2B` 는 **world frame**.

### 5g. 출력

- `~/.ros/path_tag_locator/map_world_<ts>.yaml` — **world frame** 의
  보정 결과. 스키마는 `map.yaml` 과 의도적으로 다르며, `frame: world`
  배너와 "map.yaml 대체용 아님" 경고가 포함된다. 각 항목:
  - `position_m: [x, y, z]` + `rpy_deg: [rx, ry, rz]` (전체 6-DOF)
  - `ref_tag_id` (해당 항목에 쓰인 기준 tag)
  - `map_xy` (원본 map.yaml 에서의 (x, y), 참조용)
  - `type` / `zone` / `name` (map.yaml 에서 복사)
- 항목별 6-DOF + 원본 영상은 `~/.ros/path_tag_locator/locate/<날짜>/run_*/`
  아래 단일 `locate_path_tag` 호출과 동일한 형식으로 저장.

---

## 6. 결과 읽기

### Topic (가장 최근 성공 호출이 latched 로 유지됨)

```bash
rostopic echo -n 1 /path_tag_locator/tag_world_pose
```

### 디스크 저장 (자동, 항상 보존됨)

모든 호출 (성공 / 실패) 이 디스크에 기록됨. `save_result` 플래그는 안내용일 뿐 동작에 영향을 주지 않음.

기본 저장 경로: `~/.ros/path_tag_locator/`

#### Localization 호출별 디렉터리 구조

```
~/.ros/path_tag_locator/locate/<YYYYMMDD>/run_<ts>_tag<id>/
    hand_cam.png            검출기에 입력된 BGR 이미지
    front_cam.png           검출기에 입력된 BGR 이미지
    K_hc.npz / K_fc.npz     호출 시점에 실제 사용된 K
    result.npz              T_B_world, T_A2B, T_A_world, T_hc2ee,
                            T_ab2mb, T_mb2fc, tcp_pose_mm_deg, K_*,
                            position_m, rpy_deg
    result.yaml             사람이 읽기 좋은 요약 (auto_align 보고 포함)
    request.yaml            서비스 요청 원본 echo
~/.ros/path_tag_locator/locate/locate_log.csv   append-only 인덱스
```

실패한 호출은 `..._FAILED/` 디렉터리에 저장되며 `result.yaml` 에 에러 메시지, CSV 의 `success` 열이 0 으로 기록됨.

#### Hand-eye 캘리브레이션 아카이브

```
~/.ros/path_tag_locator/handeye_calib/run_<ts>/
    samples/0000_image.png    samples/0000_pose.npz  (tcp_pose, K)
    samples/0001_image.png    ...
    samples_index.csv         캡처 1건당 1 row (tcp, 파일 경로)
    result.npz / result.yaml  /compute 호출 시 작성됨
```

`/handeye_calib/reset` 호출 시 새 `run_<ts>/` 디렉터리가 생성되므로 이전 시도 데이터는 보존됨. 보존된 `run_*/`는 `/handeye_calib/load_latest` 또는 `io.load_samples_dirs` 로 언제든 다시 합칠 수 있음(§ 1e).

#### npz 읽기 예시

```python
import numpy as np
d = np.load('/home/lcl/.ros/path_tag_locator/locate/20260520/run_20260520_173025_tag10/result.npz')
print(d['T_B_world'])   # 4x4
print(d['T_A2B'])
print(list(d.files))    # 모든 키 확인
```

#### CSV 인덱스 빠른 조회

```bash
# 가장 최근 호출 10건
tail -n 10 ~/.ros/path_tag_locator/locate/locate_log.csv

# tag_id=12 의 성공한 호출만
awk -F',' '$2==1 && $3==12' ~/.ros/path_tag_locator/locate/locate_log.csv
```

---

## 7. 자주 발생하는 문제

> 깊은 진단은 [TROUBLESHOOTING_kr.md](TROUBLESHOOTING_kr.md) 참고. 노드 캐시 / 단위 / SDK 시그니처 / reach 경계 / 보정 잔차 등 케이스별 진단 명령어 포함.


| 증상 | 점검 항목 |
|------|---------|
| `Tag A (id=58) not detected in hand-cam image` | hand-cam 이 태그를 향하고 있는가; 태그 크기가 yaml 의 `tag_a_size_m` 과 일치하는가; 태그 family 가 tag36h11 인가 |
| `Tag B not detected in front-cam image` | front-cam 시야; `tag_b_id` 가 올바른가 |
| TCP pose 를 못 읽음 (`no pose_valid /arm/state`) | arm_node 실행 중인지, 팔 전원/네트워크 (`ping 192.168.58.2`), E-stop 상태 확인 |
| `Hand-eye file not found` | 1 단계 캘리브레이션 미완료 |
| `auto_align: ... clamped` | 한 step 이 너무 커서 제한된 것, 정상 (다음 반복에서 계속 접근) |
| 자동 정렬이 떨려서 수렴 안 함 | T_hc2ee 캘리 오차 큼 → 재캘리브레이션; 또는 `align.position_tol_m` 를 크게 |
| `position_m` 결과가 100m 단위로 비정상 | 노드가 시작 시점에 메모리에 캐시한 `T_hc2ee.npz`가 오래된 값 → 노드 **재시작** |
| `GetInverseKinRef() takes exactly 4 positional arguments` | Fairino SDK 버전 차이. (2026-09-01 이후 SDK 직결 제거 — arm_node의 /arm/move_cart 경유. arm_node 쪽 SDK 시그니처를 확인) |
| `T_A_world is identity` warning | `reference_tag.yaml` 미설정 상태로 노드 기동. yaml 채우거나 호출 시 `override_ref: true` 사용 |
| `/compute` 가 `need at least N samples, got M` | `/handeye_calib/load_latest` 또는 `io.load_samples_dirs` 로 이전 세션 샘플을 합치거나 추가 `/capture` |
| `map_calibrator`: 첫 entry `base nav ... failed` | 출발 전에 base 를 map.yaml 의 tag (예: DOCK 500) 앞에 정차 → § 6.1 |
| `map_calibrator`: 모든 entry `ref_tag_id X not in reference_tags.yaml` | yaml 에서 `id:` 값을 정수로 (따옴표 없이) → § 6.7 |
| `map_calibrator`: 중간 중단 후 일부만 보정됨 | 정상. 다음 실행에서 이미 보정된 tag 의 entry 를 plan 에서 제거하면 됨. `map_world.yaml` 끼리 병합은 사용자가 수동으로. → § 6.4 |

---

## 8. 주요 파일 위치

| 파일 | 용도 |
|------|------|
| `launch/path_tag_locator.launch` | Localization 노드 |
| `launch/handeye_calib.launch` | 캘리브레이션 노드 |
| `launch/map_calibrator.launch` | 지도 일괄 보정 오케스트레이터 |
| `config/locator.yaml` | 메인 설정 (단일 locate + auto_align 파라미터) |
| `config/handeye_calib.yaml` | Hand-eye 캘리브레이션 설정 |
| `config/reference_tag.yaml` | 단일 locate 용 T_A_world 기본값 |
| `config/reference_tags.yaml` | 지도 일괄 보정용 다중 기준 tag (정확한 6-DOF) |
| `config/calibration_plan.yaml` | 지도 일괄 보정 plan (path → ref 매핑) |
| `config/map.yaml` | apriltag_nav 의 로컬 복사본 (self-contained nav) |
| `config/map_calibrator.yaml` | 오케스트레이터 기본 파일 경로 |
| `config/extrinsics.yaml` | T_AB2MB / T_MB2FC |
| `srv/LocatePathTag.srv` | 단일 locate 서비스 정의 |
| `srv/RunMapCalibration.srv` | 지도 일괄 보정 서비스 정의 |
| `README.md` | 패키지 내부 README (영문) |
