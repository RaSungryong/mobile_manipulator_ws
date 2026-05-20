# path_tag_locator 사용 가이드

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

`fairino` SDK 경로는 `config/locator.yaml` 의 `robot.fairino_sdk_path` 에서 설정.

---

## 1. Hand-eye 캘리브레이션 (`T_hc2ee.npz`, 로봇 1대당 1회)

### 1a. `config/handeye_calib.yaml` 설정

```yaml
handeye_calib:
  topics:
    hand_cam_image:  "/hand_cam/image_raw"      # ← 실제 토픽으로 변경
    hand_cam_info:   "/hand_cam/camera_info"
  tag:
    id:        58                                # ← 캘리브레이션용 태그 (= 이후 기준 태그 A)
    size_m:    0.060                             # ← 태그 한 변 길이 (m)
  robot:
    robot_ip:  "192.168.58.2"                    # ← 로봇 IP
    fairino_sdk_path: "/home/lcl/mobile_manipulator_ws/fairino_sdk/fairino-python-sdk/Linux"
  io:
    output_path: "$(find path_tag_locator)/config/hand_eye/T_hc2ee.npz"
    min_samples: 8                               # ← 권장 ≥ 15
```

### 1b. 노드 실행

```bash
roslaunch path_tag_locator handeye_calib.launch
```

### 1c. 샘플 수집 (각 자세마다 capture 1회)

암을 태그가 보이는 다양한 자세로 옮기면서 (각도/거리를 분산시킬수록 좋음), 매번:

```bash
# 1회 캡처
rosservice call /handeye_calib/capture "{}"

# 진행 상황 확인
rosservice call /handeye_calib/status  "{}"

# 처음부터 다시
rosservice call /handeye_calib/reset   "{}"
```

### 1d. 캘리브레이션 실행 (npz 저장)

```bash
rosservice call /handeye_calib/compute "{}"
```

응답에는 BEST method, residual, T_hc2ee 의 translation 과 RPY 가 포함됨.
결과 파일은 `config/hand_eye/T_hc2ee.npz` 에 저장됨.

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

T_AB2MB (arm_base → mobile_base), T_MB2FC (mobile_base → front_cam) 의 기본값이 채워져 있음. 플랫폼 개조 시에만 수정.

### 2c. 메인 설정 `config/locator.yaml`

| 필드 | 의미 |
|------|------|
| `topics.hand_cam_*` / `front_cam_*` | 양쪽 카메라 토픽 |
| `tag.tag_a_id` / `tag_b_id` | 기준 / 경로 태그 ID |
| `tag.tag_a_size_m` / `tag_b_size_m` | 태그 한 변 길이 (m) |
| `robot.robot_ip` / `fairino_sdk_path` | Fairino RPC |
| `align.*` | 자동 정렬 허용 오차 및 step 크기 |

---

## 3. Localization 노드 실행

```bash
roslaunch path_tag_locator path_tag_locator.launch
```

노드가 제공하는 인터페이스:
- **Service**: `/path_tag_locator/locate_path_tag` (타입 `path_tag_locator/LocatePathTag`)
- **Topic** (latched): `/path_tag_locator/tag_world_pose` (`geometry_msgs/PoseStamped`)

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

정렬 동작 조정: `config/locator.yaml` 의 `align:` 블록 수정:
- `max_iterations: 1` → one-shot 모드
- `position_tol_m: 0.005` / `angle_tol_deg: 1.0` → 수렴 임계값
- `max_step_m: 0.10` / `max_step_deg: 15.0` → 1 step 안전 clamp

---

## 5. 결과 읽기

### Topic (가장 최근 성공 호출이 latched 로 유지됨)

```bash
rostopic echo -n 1 /path_tag_locator/tag_world_pose
```

### 디스크 저장 (save_result=true 시, 기본 경로 `~/.ros/path_tag_locator/`)

```bash
ls -t ~/.ros/path_tag_locator/ | head
# path_tag_<id>_<timestamp>.npz   — T_B_world, T_A2B, T_A_world, tcp_pose
# path_tag_<id>_<timestamp>.yaml  — position_m, rpy_deg (사람이 보기 쉬움)
```

npz 읽기:
```python
import numpy as np
d = np.load('/home/lcl/.ros/path_tag_locator/path_tag_10_20260520_173025.npz')
print(d['T_B_world'])   # 4x4
print(d['T_A2B'])
```

---

## 6. 자주 발생하는 문제

| 증상 | 점검 항목 |
|------|---------|
| `Tag A (id=58) not detected in hand-cam image` | hand-cam 이 태그를 향하고 있는가; 태그 크기가 yaml 의 `tag_a_size_m` 과 일치하는가; 태그 family 가 tag36h11 인가 |
| `Tag B not detected in front-cam image` | front-cam 시야; `tag_b_id` 가 올바른가 |
| `GetActualTCPPose failed (err=...)` | Fairino IP 도달 가능 (`ping 192.168.58.2`); E-stop 상태; `fairino_sdk_path` 올바른가 |
| `Hand-eye file not found` | 1 단계 캘리브레이션 미완료 |
| `auto_align: ... clamped` | 한 step 이 너무 커서 제한된 것, 정상 (다음 반복에서 계속 접근) |
| 자동 정렬이 떨려서 수렴 안 함 | T_hc2ee 캘리 오차 큼 → 재캘리브레이션; 또는 `align.position_tol_m` 를 크게 |

---

## 7. 주요 파일 위치

| 파일 | 용도 |
|------|------|
| `launch/path_tag_locator.launch` | Localization 노드 |
| `launch/handeye_calib.launch` | 캘리브레이션 노드 |
| `config/locator.yaml` | 메인 설정 |
| `config/handeye_calib.yaml` | 캘리브레이션 설정 |
| `config/reference_tag.yaml` | T_A_world 기본값 |
| `config/extrinsics.yaml` | T_AB2MB / T_MB2FC |
| `srv/LocatePathTag.srv` | 서비스 정의 |
| `README.md` | 패키지 내부 README |
