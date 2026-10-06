# path_tag_locator 트러블슈팅 가이드

증상별 빠른 진단 + 검증 명령어 모음. 다른 세션에서도 이 문서만 보면
문제 위치를 한 번에 추적할 수 있게 정리.

---

## 0. 컨텍스트 (먼저 알아둘 것)

### 변환 체인

```
T_A2B = T_A2hc · T_hc2ee · T_ee2ab · T_ab2mb · T_mb2fc · T_fc2B
T_B_world = T_A_world · T_A2B
```

좌표 규약: **`T_X2Y` = "Y 프레임의 X 프레임 내 표현"** = Y → X 변환.

### 핵심 파일

| 파일 | 역할 |
|------|------|
| `config/locator.yaml` | 토픽/태그ID/arm_node 프록시 토픽/align 파라미터 |
| `config/reference_tag.yaml` | `T_A_world` (기준 태그 월드 좌표) |
| `apriltag_nav/config/tf/tf_chain.yaml` | `T_ab2mb`, `T_mb2fc`, `T_hc2ee`, `T_ee2tip` — 모든 고정 변환 (2026-09-21), **m 단위** |
| `apriltag_nav/config/tf/T_hc2ee.npz` | 손-눈 보정 결과의 npz 쌍둥이 (yaml 블록과 함께 갱신됨) |
| `scripts/path_tag_locator_node.py` | locate 노드 |
| `scripts/handeye_calib_node.py` | 손-눈 보정 노드 |
| `apriltag_nav/tools/tf_chain_tool.py` | 변환 보기/검사/기록 (`show` / `check` / `set`) |
| `src/path_tag_locator/arm_interface.py` | arm_node 프록시 — `/arm/state` 읽기, `/arm/move_cart` 이동 (2026-09-01부터 SDK 직결 없음) |

### 노드는 시작 시 1회 캐시

- `tf_chain.yaml` 의 `T_hc2ee`(npz), `T_ab2mb`, `T_mb2fc`
- `reference_tag.yaml` 의 `T_A_world`
- `locator.yaml` 의 `align.*` 파라미터

**파일 편집 후 노드 재시작 필수.** 한 번 본 메모리 캐시는 절대 다시 안 읽음.

---

## 1. 증상별 진단

### 1.1 `position_m` 결과가 100 m 단위로 비정상

**전형 예**: `position_m: [-162.0, -40.9, -166.9]`

#### 진단

```bash
# 디스크의 T_hc2ee 값을 확인
python3 -c "
import numpy as np
T = np.load('src/apriltag_nav/config/tf/T_hc2ee.npz')['arr_0']
print(T)
print('||t|| =', np.linalg.norm(T[:3,3]), 'm')
"

# 마지막 호출의 result.npz에서 실제 사용된 T_hc2ee 비교
python3 -c "
import numpy as np, glob, os
runs = sorted(glob.glob(os.path.expandvars('$MM_WS/log/path_tag_locator/locate/*/run_*/result.npz')))
T = np.load(runs[-1])['T_hc2ee']
print('used in last run:', T[:3,3], '||t||=', np.linalg.norm(T[:3,3]))
"
```

#### 원인

- 디스크 파일과 **노드 캐시가 다름**. 노드 기동 후에 누가 `tf_chain_tool.py set` 또는 `/handeye_calib/compute`로 값을 갱신해도 노드는 모름.

#### 처방

```bash
# 노드 재시작
# Ctrl-C 후
roslaunch path_tag_locator path_tag_locator.launch
```

### 1.2 `auto_align`이 수렴하지 않음

**전형 예**: `align_iterations_used: 5`, `xy_offset_m: 0.07`, `tilt_deg: 6-7°`

#### 진단 - 1단계: 노드 측 iter 로그

`path_tag_locator` 노드 터미널에 다음 줄이 매 iter마다 찍힘:

```
[INFO] auto_align iter 1/5: xy=0.XXX m, tilt=XX.X deg, z=X.XXX m
[WARN] auto_align: step 1 clamped (Δt=..., Δrot=...)   ← 매 iter 클램프되는지가 핵심
```

#### 진단 - 2단계: 패턴 분류

| iter별 metrics 양상 | 원인 | 처방 |
|---------------------|------|------|
| 단조 감소 중 (iter 5에서도 작아지는 추세) | 단순 반복 부족 | `max_iterations: 15` |
| 매 iter `step ... clamped` + 같은 TCP | **작업영역(reach) 경계** | 안쪽 자세로 초기 위치 변경 |
| 진동 / 증가 | T_hc2ee 회전 오차 | 자세 다양성 ↑ 해서 재보정 |
| 5회 연속 거의 동일 (변화 < 1 mm) | 이동이 거부되는데 에러가 안 보임 / IK 한계 | arm_node 로그의 `move_cart` 결과(`MoveL`/`MoveCart error N`)와 `/arm/state`의 `pose_valid` 확인 |
| iter 1에서 즉시 `RuntimeError: tag A not detected` | 초기 자세에서 시야 밖 | `align_initial_tcp_mm_deg` 재조정 |

#### 진단 - 3단계: reach 경계 확인

```bash
python3 -c "
import math
# 호출에서 준 initial과 saved result의 final tcp 비교
init = [-280, 825, -490]
final = [-224, 866, -162]
print(f'initial dist from base = {math.sqrt(sum((v/1000)**2 for v in init)):.3f} m')
print(f'final   dist from base = {math.sqrt(sum((v/1000)**2 for v in final)):.3f} m')
# FR10v6 공칭 reach 1.40 m (플랜지 기준); 플랜 생성기는 플랜지 1.25 m 를 한계로 쓴다
"
```

플랜지 거리 ≥ 1.25 m 면 reach 경계 의심. 안쪽 자세 시도:

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: 105, override_ref: false, save_result: true, save_dir: '',
  auto_align: true,
  align_initial_tcp_mm_deg: [-200, 600, -300, -179.5, 0, 0]
}"
```

#### 처방 옵션 (locator.yaml 조정)

```yaml
align:
  max_iterations:        15      # 5 → 15
  max_step_m:            0.20    # 0.10 → 0.20 (클램프 완화)
  max_step_deg:          25.0    # 15 → 25
  target_distance_m:     0.30    # 0.0 → 0.30 (z 기준 고정)
  position_tol_m:        0.005   # 1mm → 5mm (수렴 기준 완화; 기본 0.001)
  angle_tol_deg:         1.0     # 0.5 → 1 (기본 0.5; orientation: fixed 에서는 기록만 됨)
```

### 1.3 Fairino SDK 시그니처 오류 (`GetInverseKinRef() takes ...`, `MoveJ(...)`)

2026-09-01 이후 이 패키지는 **SDK 를 직접 호출하지 않는다** — 팔 이동은
`ArmInterface.move_j_to_pose` → `/arm/move_cart`(arm_node) 경유. 이런
오류가 보이면 arm_node(apriltag_nav `arm_controller.py`) 쪽이므로 그 노드의
로그를 본다. 이 패키지에서 확인할 것은 `/arm/state` 가 `pose_valid: true`
로 오는지(§ 2.6)와 `locator.yaml arm.*` 토픽 이름뿐이다.

### 1.4 `Tag A not detected in hand-cam image`

#### 진단 체크리스트

1. **태그 family**: `locator.yaml` `tag.family == "tag36h11"` 인지, 실제 태그도 같은가?
2. **태그 크기**: `tag_a_size_m: 0.090` 이 실제 변 길이(m)와 같고 `detector.hand_cam_size_m` 이 robot.yaml `robot_camera.tag_size.hand_cam` 과 같은가? (mm로 잘못 적으면 90m로 해석 → 거리 폭주)
3. **태그 ID**: `tag_a_id` 와 실제 인쇄된 ID 일치?
4. **카메라 토픽**: `rostopic hz /hand_cam/color/image_raw` 와 `/hand_cam/tag_detections`(robot_camera_node 의 검출, 이 패키지가 소비하는 쪽) 가 도는가? hand_cam 이 `set_enabled false` 면 검출이 없다.
5. **카메라 K 행렬**: `rostopic echo -n 1 /hand_cam/color/camera_info` 로 `K` 가 픽셀 단위인지 확인 (robot.yaml `intrinsics_override.hand_cam` 이 있으면 검출은 그 K 로 보정된 프레임 기준).
6. **밝기/초점**: hand_cam.png를 열어 태그가 명확히 보이는가?

```bash
# 마지막 실패한 호출의 hand_cam.png 위치
ls $MM_WS/log/path_tag_locator/locate/*/run_*FAILED/hand_cam.png 2>/dev/null | tail -1
```

### 1.5 `T_A_world is identity` (월드 좌표 안 잡힘)

#### 원인

`reference_tag.yaml` 의 `position_m`/`rpy_deg`가 전부 0 → `T_A_world = I` → `T_B_world = T_A2B` (월드가 아닌 A-기준).

#### 처방

옵션 1) yaml에 실제 측정값 입력:

```yaml
reference_tag:
  format: "pose"
  position_m: [1.234, 0.567, 0.890]
  rpy_deg:    [0.0, 0.0, 90.0]
```

옵션 2) 호출마다 override (yaml 안 건드림):

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  ..., override_ref: true,
  ref_pose: {
    position: {x: 1.234, y: 0.567, z: 0.890},
    orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}
  }, ...
}"
```

**주의**: `orientation`은 쿼터니언. 전부 0이면 안 됨 (`w: 1.0` 최소). 빈 ref_pose는 `w=0` → `assert_rigid` ValueError.

### 1.6 핸드-아이 보정 잔차가 큼

```bash
# 최근 보정 결과 잔차 확인
cat $(ls -1dt $MM_WS/log/path_tag_locator/handeye_calib/run_*/result.yaml | head -1)
```

| residual | 평가 |
|----------|------|
| < 0.01 | 우수 |
| 0.01 – 0.05 | 양호 |
| 0.05 – 0.1 | 보통 (align 수렴 7cm 정도 한계) |
| > 0.1 | 부족 (재보정) |

#### 개선

- 샘플 수 ↑ (15-30개)
- 자세 다양성 ↑ : 각 자세마다 회전 차이 ≥ 10-20°, 위치 차이 ≥ 5-10cm
- 한 평면에 몰린 자세 ❌ (전체 3축 회전 분산)

기존 샘플 재활용으로 다양성 추가:

```bash
rosservice call /handeye_calib/load_latest "{}"   # 과거 세션 합치기
rosservice call /handeye_calib/capture "{}"        # 새 자세 추가
rosservice call /handeye_calib/compute "{}"
```

---

## 2. 진단 명령어 모음 (복붙용)

### 2.1 디스크 vs 캐시 비교

```bash
# 디스크의 T_hc2ee
python3 -c "
import numpy as np
T = np.load('src/apriltag_nav/config/tf/T_hc2ee.npz')['arr_0']
print('disk T_hc2ee t (m):', T[:3,3])
"

# 노드가 마지막에 사용한 T_hc2ee (result.npz)
python3 -c "
import numpy as np, glob, os
runs = sorted(glob.glob(os.path.expandvars('$MM_WS/log/path_tag_locator/locate/*/run_*/result.npz')))
if runs:
    T = np.load(runs[-1])['T_hc2ee']
    print('used (last run):', T[:3,3])
    print('run:', runs[-1])
"
```

두 값이 다르면 → **노드 재시작**.

### 2.2 보정 placeholder vs real 구분

```bash
python3 -c "
import numpy as np
T = np.load('src/apriltag_nav/config/tf/T_hc2ee.npz')['arr_0']
R = T[:3,:3]
# placeholder: R = 정확히 diag(-1,-1,1), off-diagonal=0
off = abs(R[0,1]) + abs(R[0,2]) + abs(R[1,0]) + abs(R[1,2]) + abs(R[2,0]) + abs(R[2,1])
print('off-diagonal magnitude:', off)
if off < 1e-9:
    print('⚠ 설계/공칭값 (실보정 아님)')
else:
    print('실제 보정값')
"
```

### 2.3 작업영역 거리 계산

```bash
python3 -c "
import math
tcp = [-280, 825, -490]   # ← 검사할 mm 좌표
d = math.sqrt(sum((v/1000)**2 for v in tcp))
print(f'distance from base: {d:.3f} m  (FR10v6 공칭 1.40 m, 플랜 한계 1.25 m — 플랜지 기준)')
print('REACH 경계' if d > 1.25 else 'OK')
"
```

### 2.4 latest run 요약

```bash
LATEST=$(ls -1dt $MM_WS/log/path_tag_locator/locate/*/run_*/ 2>/dev/null | head -1)
echo "$LATEST"
cat "$LATEST/result.yaml"
echo "---"
cat "$LATEST/request.yaml"
```

### 2.5 최근 보정 결과

```bash
cat $(ls -1dt $MM_WS/log/path_tag_locator/handeye_calib/run_*/result.yaml 2>/dev/null | head -1)
```

### 2.6 서비스 살아있나 확인

```bash
rosnode list | grep path_tag
rosservice list | grep -E "path_tag|handeye"
rosservice info /path_tag_locator/locate_path_tag
```

---

## 3. 알려진 함정

### 3.1 값의 출처는 `tf_chain.yaml`의 `source` 줄로 확인한다

`tf_chain_tool.py show`가 변환마다 source(측정 세션·방법)와 설계값 대비 차이를
찍는다. `set`으로 넣은 값은 `--source`가 그대로 기록되므로, 공칭값을 넣었다면
거기에 그렇게 써 있다.

### 3.2 단위 함정

| 곳 | 단위 |
|----|------|
| Fairino TCP pose | **mm, deg** (ZYX intrinsic) |
| 체인 내부 모든 행렬 | **m, rad** |
| `tf_chain.yaml` / npz 의 t | **m** (`t_mm` 읽기용 키만 mm) |
| `reference_tag.yaml` `position_m` | **m** |
| 서비스 응답 `position_m` | **m** |
| 서비스 응답 `rpy_deg` | **degrees (ZYX intrinsic)** |
| `align_initial_tcp_mm_deg` | **mm, deg** |

`pose_fr5_to_matrix_m`이 mm→m 변환을 담당. 다른 경로로 들어오는 매트릭스는 이미 m라고 가정.

### 3.3 ref_pose 쿼터니언

`override_ref: true` 사용 시 `orientation`의 4개 값 중 적어도 하나 ≠ 0 이어야 함. 전부 0이면 norm=0 → 회전 행렬 → assert_rigid 실패. 무회전이면 `{x:0, y:0, z:0, w:1.0}`.

### 3.4 `save_result` 필드는 informational

서비스 srv 의 `save_result` 는 deprecated. 실제로는 **모든 호출이 디스크에 저장됨** (성공/실패 모두). srv를 바꾸진 않았으니 클라이언트는 그대로 두면 됨.

### 3.5 노드 시작 시점 컨피그 캐시

`rospy.get_param("~", {})` 으로 1회 읽음. `$MM_WS/log/...` 에 별도 저장된 yaml이나 디스크의 npz를 갱신해도 자동 반영 안 됨. **무조건 재시작**.

### 3.6 `tf_chain.yaml` 규약

`T_ab2mb` = "mb 의 ab 표현" = mb-coord → ab-coord 변환. 이전 주석은 반대로 해석되도록 쓰여 있었음(이미 수정). 다른 코드에서 같은 yaml을 다른 규약으로 읽으면 안 됨.

### 3.7 도구(Tool) 프레임 — 컨트롤러의 활성 툴은 플랜지

2026-09-14 확인: 컨트롤러의 활성 툴은 **플랜지(tool 0)** 이고 `/arm/state` 의
TCP 도 플랜지다(홈 자세 ≈ (−159, 700, 774) mm). 핸드아이 `T_hc2ee`, 플랜
시드, 체인 전부가 플랜지 기준으로 표현되어 있으므로 **tool 1 (vision_tip)
을 활성화하면 안 된다** — 하면 체인 전체를 다시 표현해야 한다(CLAUDE.md
Coordinate Frames). tool 1 의 등록 자체는 `src/apriltag_nav/tools/set_tool_tcp.py`
가 하며(dry run 이 값을 찍는다), 팁 오프셋은 `tf_chain.yaml T_ee2tip`.

---

## 4. 점검 순서 (체크리스트)

문제 발생 시 위에서 아래로:

- [ ] **노드 재시작** 후에도 재현되는가? (안 되면 캐시 문제 — § 1.1)
- [ ] `T_hc2ee`가 실보정값인가? (§ 2.2, `tf_chain_tool.py show`의 source 줄)
- [ ] 보정 잔차 < 0.1 인가? (§ 1.6)
- [ ] 초기 TCP가 reach 경계 (≥ 0.85 m) 안에 있는가? (§ 2.3)
- [ ] `reference_tag.yaml` 이 identity가 아닌가? (§ 1.5)
- [ ] 핸드캠/프론트캠 토픽 살아있는가? (§ 1.4)
- [ ] 노드 측 iter 로그에 `clamped` 가 매번 뜨는가? (§ 1.2)
- [ ] 활성 툴이 플랜지(tool 0)인가? `/arm/state` 홈 자세 ≈ (−159, 700, 774) mm (§ 3.7)
- [ ] (map_calibrator 사용 시) base 가 플랜의 첫 태그(정반 1: 100, 정반 2: 126) 위에 서서 front-cam 이 그 태그를 보는가? (§ 6.1)
- [ ] (map_calibrator 사용 시) 메인 스택이 떠 있고 세션 중 TASK/GOTO 를 보내지 않았는가? (§ 6.2)
- [ ] (map_calibrator 사용 시) `reference_tags.yaml` 의 `id:` 가 정수인가? (§ 6.7)

---

## 5. (이력) `tcp_pose.py` 의 SDK 시그니처 수정

`tcp_pose.py`(이 패키지의 Fairino SDK 래퍼)는 2026-09-01 리팩터링으로
삭제됐다 — 팔은 arm_node 경유(§ 1.3). 당시의 시그니처 수정 이력은 git
이력에 있고, SDK 호출 코드는 이제 apriltag_nav `arm_controller.py` 한 곳뿐이다.

---

## 6. 지도 일괄 보정 (`map_calibrator_node`) 진단

### 6.1 첫 번째 entry 에서 `base nav to ... failed`

#### 원인
- 세션 시작 시 base 가 어떤 map tag 도 front-cam 으로 보지 못함 →
  `MobileController.get_current_tag_id()` 가 None, `last_known_tag` 도 None
  → `move_to_tag` 가 시작 tag 를 못 정함. (2026-09-08부터 첫 hop 은 서 있는
  태그에 먼저 정렬하므로, 태그가 안 보이면 `align_timeout_s` 로 실패.)
- front_cam 이 죽어 있음 — mobile_node 는 검출이 1 s 넘게 끊기면 모든
  `/mobile/goto_tag` 를 `front_cam not working: …` 로 거부한다(2026-09-22).
- 다른 명령이 base 를 잡고 있음 (TASK/GOTO 진행 중 → mobile_node 가 중복
  이동 거부).

#### 처방
실행 전에 base 를 **플랜의 첫 태그 위**(정반 1: 100, 정반 2: 126)에 세워
front-cam 크로스헤어에 그 태그가 보이게 한다. 생성된 플랜에는 2026-09-02
부터 `nav_start_id` 가 없으므로 도크 500 을 거치지 않고 그 자리에서 시작한다.
`rostopic echo /mobile/state` 의 `front_cam_ok` / `front_cam_reason` 과
`/front_cam/tag_detections` 를 확인.

### 6.2 base 가 세션 중 엉뚱한 곳으로 가거나 이동을 거부함

#### 원인
세션 중에 `TASK`/`GOTO`(또는 CHARGE/UNDOCK)가 들어옴 — task_executor 와
map_calibrator 는 둘 다 `/mobile/goto_tag` 의 지휘자다. (`/cmd_vel` 은
mobile_node 만 발행하므로 스택 안에 두 번째 발행자는 없다; 있다면
`tools/navigate.py` / `tools/vw_drive.py` 같은 독립 도구뿐.)

#### 처방
세션 중 TASK/GOTO 금지. **메인 스택은 그대로 떠 있어야 한다** — 캘리브레이션
노드는 하드웨어를 소유하지 않으므로 arm_node / mobile_node /
robot_camera_node 를 죽이면 세션이 돌지 않는다. 독립 도구가 떠 있는지만
확인: `rosnode list | grep -E "navigate|vw_drive"`.

### 6.3 `auto_align: tag A (id=X) not detected` 가 빈번

#### 원인 (entry 1회 실패는 정상, 매번이면 ↓)
- 그 entry 의 `arm_view_tcp_mm_deg` 가 ref tag 시야 밖
- base 의 실제 정차 위치가 `map.yaml` 의 path tag 값과 너무 다름
  (= cm 보다 큰 어긋남 → arm 시드 자세가 빗나감)

#### 처방
1. (2026-09-04부터 자동) 시드에서 태그를 못 본 재시도는 시드를 다시 추정한다 —
   같은 ref 의 성공 entry 들의 보정량 → anchor 부트스트랩 → 카메라를
   `retry_raise_m` 올린 시드 순(`view_tcp_source` 에 기록). `retry_count` 를
   올리면 뒤 전략까지 간다.
2. `arm_view_tcp_mm_deg` 를 entry 별로 override (현장 jog 로 ref tag 가
   잘 보이는 자세를 찾은 뒤 그 TCP 를 yaml 에 기록), 또는 성공 세션으로
   `update_plan_seeds_from_session.py` 실행.
3. `locator.yaml` 의 `align.max_initial_steps` / `max_initial_step_m` 를
   키워서 첫 접근이 더 멀리 가게 함 (안전 vs 속도 트레이드오프)

### 6.4 `map_world.yaml` 이 일부만 채워짐 (재개 방법)

#### 원인
중간에 노드를 죽였음. 원자 쓰기는 entry 별로 일어나므로 성공한 entry 까지는
`map_world_<ts1>.yaml` 에 기록되어 있음.

#### 처방
`map_in_path` 는 read-only metadata 소스 (apriltag_nav 의 `map.yaml`)
이므로 그것을 바꾸는 방식으로는 재개되지 않는다. 대신:

1. `map_world_<ts1>.yaml` 의 `tags:` 키들을 열어 이미 성공한 path_tag_id
   목록을 확인.
2. `calibration_plan_plate{1,2}.yaml` 의 사본에서 이미 성공한 entry 들을
   제거 (또는 주석 처리) 한 새 plan 을 `plan_path` 로 넘겨 두 번째 실행
   (2026-09-22 의 123–125 부분 플랜이 그 예).
3. 결과는 새 `map_world_<ts2>.yaml` 에 들어가므로, 사용자가 수동으로
   두 yaml 의 `tags:` 섹션을 병합:

```bash
# 수동 병합 예시 (jq 사용시)
yq eval-all '. as $item ireduce ({}; . * $item)' \
    map_world_<ts1>.yaml map_world_<ts2>.yaml > map_world_merged.yaml
```

> 자동 재개 (이전 출력을 읽고 plan 의 이미-완료 entry 를 skip) 는 추후
> 추가 가능. 현재는 수동.

### 6.5 결과가 의도와 다름 (특정 tag 가 1 m 이상 어긋남)

#### 진단 체크리스트
1. **ref tag 측정값이 정확한가** — `reference_tags.yaml` 의 (x, y, z, rpy)
   를 줄자로 재확인. 보통 이게 원인.
2. **그 entry 의 hand-cam 이 다른 tag 를 본 게 아닌가** —
   `$MM_WS/log/path_tag_locator/locate/<date>/run_*_tag<id>/hand_cam.png` 를 열어
   진짜로 ref_tag_id 만 보이는지 확인.
3. **T_A_world rpy_deg 의 회전 부호** — 같은 (x, y, z) 라도 yaw 가 90° 잘못
   되면 path tag 결과가 회전 방향으로 멀리 튐.
4. **Hand-eye 잔차** — § 1.6 참조.

### 6.6 (삭제) `Navigator: camera_info NOT received within 5.0s`

패키지 내부 nav(`nav/`, `robot_nav.yaml`)의 메시지로, 2026-09-01 삭제됐다.
base nav 는 mobile_node(`MobileClient`) 경유이며 front_cam 이 죽으면
mobile_node 가 `front_cam not working: …` 로 goto 를 거부한다 (§ 6.1).

### 6.7 모든 entry 가 `ref_tag_id X not in reference_tags.yaml`

#### 원인
plan 의 `ref_tag_id` 가 `reference_tags.yaml` 의 어떤 항목과도 매칭되지 않음.
yaml 의 id 는 int (예: `100`), plan 도 int 여야 함. 따옴표 (`"100"`) 로 쓰면
string 으로 파싱되어 매칭 실패.

#### 처방
모든 `id:` 값이 따옴표 없이 정수로 적혀 있는지 확인.
