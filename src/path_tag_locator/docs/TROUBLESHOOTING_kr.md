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
| `config/locator.yaml` | 토픽/태그ID/로봇IP/align 파라미터 |
| `config/reference_tag.yaml` | `T_A_world` (기준 태그 월드 좌표) |
| `config/extrinsics.yaml` | `T_ab2mb`, `T_mb2fc` (플랫폼 기하) |
| `config/hand_eye/T_hc2ee.npz` | 손-눈 보정 결과 (mm 아닌 **m 단위**) |
| `scripts/path_tag_locator_node.py` | locate 노드 |
| `scripts/handeye_calib_node.py` | 손-눈 보정 노드 |
| `scripts/save_npz.py` | **placeholder** T_hc2ee 작성 (실보정 아님) |
| `src/path_tag_locator/tcp_pose.py` | Fairino SDK 래퍼 (IK/MoveJ) |

### 노드는 시작 시 1회 캐시

- `T_hc2ee.npz`
- `extrinsics.yaml` 의 `T_ab2mb`, `T_mb2fc`
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
T = np.load('src/path_tag_locator/config/hand_eye/T_hc2ee.npz')['arr_0']
print(T)
print('||t|| =', np.linalg.norm(T[:3,3]), 'm')
"

# 마지막 호출의 result.npz에서 실제 사용된 T_hc2ee 비교
python3 -c "
import numpy as np, glob, os
runs = sorted(glob.glob(os.path.expanduser('~/.ros/path_tag_locator/locate/*/run_*/result.npz')))
T = np.load(runs[-1])['T_hc2ee']
print('used in last run:', T[:3,3], '||t||=', np.linalg.norm(T[:3,3]))
"
```

#### 원인

- 디스크 파일과 **노드 캐시가 다름**. 노드 기동 후에 누가 `save_npz.py` 또는 `/handeye_calib/compute`로 npz를 갱신해도 노드는 모름.

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
| 5회 연속 거의 동일 (변화 < 1 mm) | MoveJ가 실패하지만 에러 안 뜸 / IK 한계 | `tcp_pose.py` MoveJ 반환값 로그 추가 |
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
# FR10v6 reach ≈ 1.0 m
"
```

distance ≥ 0.85 m 면 reach 경계 의심. 안쪽 자세 시도:

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
  position_tol_m:        0.010   # 5mm → 10mm (수렴 기준 완화)
  angle_tol_deg:         2.0     # 1 → 2
```

### 1.3 `GetInverseKinRef() takes exactly 4 positional arguments (5 given)`

#### 원인

Fairino SDK 버전별 시그니처 차이. 표준 시그니처:

```python
err, joints = robot.GetInverseKinRef(0, target_pose, q0_deg)   # 3 args
err, joints = robot.GetInverseKin(0, target_pose, config=-1)   # 2 positional + 1 kw
err = robot.MoveJ(joints, tool=TOOL_ID, user=0, vel=..., acc=..., ovl=...)
```

#### 진단

```bash
grep -n "GetInverseKinRef\|GetInverseKin\|MoveJ" \
  src/path_tag_locator/src/path_tag_locator/tcp_pose.py

# SDK 예제와 대조
grep -n "GetInverseKinRef\|GetInverseKin\|MoveJ" \
  src/fairino_sdk/fairino-python-sdk/examples/*.py
```

#### 처방

[`tcp_pose.py:move_j_to_pose`](../src/path_tag_locator/tcp_pose.py)을 위 시그니처에 맞게 수정. apriltag_nav의 `arm_controllerrealwithscan_v2.py:549-557` 가 동작 검증된 참조.

### 1.4 `Tag A not detected in hand-cam image`

#### 진단 체크리스트

1. **태그 family**: `locator.yaml` `tag.family == "tag36h11"` 인지, 실제 태그도 같은가?
2. **태그 크기**: `tag_a_size_m: 0.060` 이 실제 변 길이(m)와 같은가? (mm로 잘못 적으면 60m로 해석 → 거리 폭주)
3. **태그 ID**: `tag_a_id` 와 실제 인쇄된 ID 일치?
4. **카메라 토픽**: `rostopic hz /vision_cam/color/image_raw` 가 도는가?
5. **카메라 K 행렬**: `rostopic echo -n 1 /vision_cam/color/camera_info` 로 `K` 가 픽셀 단위인지 확인.
6. **밝기/초점**: hand_cam.png를 열어 태그가 명확히 보이는가?

```bash
# 마지막 실패한 호출의 hand_cam.png 위치
ls ~/.ros/path_tag_locator/locate/*/run_*FAILED/hand_cam.png 2>/dev/null | tail -1
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
cat $(ls -1dt ~/.ros/path_tag_locator/handeye_calib/run_*/result.yaml | head -1)
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
T = np.load('src/path_tag_locator/config/hand_eye/T_hc2ee.npz')['arr_0']
print('disk T_hc2ee t (m):', T[:3,3])
"

# 노드가 마지막에 사용한 T_hc2ee (result.npz)
python3 -c "
import numpy as np, glob, os
runs = sorted(glob.glob(os.path.expanduser('~/.ros/path_tag_locator/locate/*/run_*/result.npz')))
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
T = np.load('src/path_tag_locator/config/hand_eye/T_hc2ee.npz')['arr_0']
R = T[:3,:3]
# placeholder: R = 정확히 diag(-1,-1,1), off-diagonal=0
off = abs(R[0,1]) + abs(R[0,2]) + abs(R[1,0]) + abs(R[1,2]) + abs(R[2,0]) + abs(R[2,1])
print('off-diagonal magnitude:', off)
if off < 1e-9:
    print('⚠ PLACEHOLDER (save_npz.py 결과)')
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
print(f'distance from base: {d:.3f} m  (FR10v6 reach ≈ 1.0 m)')
print('REACH 경계' if d > 0.85 else 'OK')
"
```

### 2.4 latest run 요약

```bash
LATEST=$(ls -1dt ~/.ros/path_tag_locator/locate/*/run_*/ 2>/dev/null | head -1)
echo "$LATEST"
cat "$LATEST/result.yaml"
echo "---"
cat "$LATEST/request.yaml"
```

### 2.5 최근 보정 결과

```bash
cat $(ls -1dt ~/.ros/path_tag_locator/handeye_calib/run_*/result.yaml 2>/dev/null | head -1)
```

### 2.6 서비스 살아있나 확인

```bash
rosnode list | grep path_tag
rosservice list | grep -E "path_tag|handeye"
rosservice info /path_tag_locator/locate_path_tag
```

---

## 3. 알려진 함정

### 3.1 `save_npz.py`는 보정 결과가 아니다

오타 그대로 둔 파일. 하드코딩된 4x4를 npz로 저장. `T_hc2ee = diag(-1,-1,1) + 작은 t`인데 우연히 실제 보정값과 비슷할 수 있어 헷갈리기 쉬움.

```bash
# 파일에 --force 가드 있으므로 실수로 덮어쓰진 않음. 그래도 의도치 않게 실행하지 말 것.
```

### 3.2 단위 함정

| 곳 | 단위 |
|----|------|
| Fairino TCP pose | **mm, deg** (ZYX intrinsic) |
| 체인 내부 모든 행렬 | **m, rad** |
| `T_hc2ee.npz` 의 t | **m** |
| `extrinsics.yaml` 의 t | **m** |
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

`rospy.get_param("~", {})` 으로 1회 읽음. `~/.ros/...` 에 별도 저장된 yaml이나 디스크의 npz를 갱신해도 자동 반영 안 됨. **무조건 재시작**.

### 3.6 `extrinsics.yaml` 규약

`T_ab2mb` = "mb 의 ab 표현" = mb-coord → ab-coord 변환. 이전 주석은 반대로 해석되도록 쓰여 있었음(이미 수정). 다른 코드에서 같은 yaml을 다른 규약으로 읽으면 안 됨.

### 3.7 도구(Tool) 활성화

`tcp_index: 1` (= vision_tip)이 컨트롤러에 등록되어 있어야 함:

```bash
# 1회 등록 (필요 시)
python3 /home/ku/mobile_manipulator_ws/scripts/set_tool_tcp.py
```

`GetActualTCPPose()`는 인자 없이 호출 → **현재 활성 tool의 TCP**를 반환. tool=0 이면 flange pose가 나옴. 보정과 locate가 같은 tool 설정에서 돌아야 일관성 유지.

---

## 4. 점검 순서 (체크리스트)

문제 발생 시 위에서 아래로:

- [ ] **노드 재시작** 후에도 재현되는가? (안 되면 캐시 문제 — § 1.1)
- [ ] `T_hc2ee.npz`가 placeholder인가? (§ 2.2)
- [ ] 보정 잔차 < 0.1 인가? (§ 1.6)
- [ ] 초기 TCP가 reach 경계 (≥ 0.85 m) 안에 있는가? (§ 2.3)
- [ ] `reference_tag.yaml` 이 identity가 아닌가? (§ 1.5)
- [ ] 핸드캠/프론트캠 토픽 살아있는가? (§ 1.4)
- [ ] 노드 측 iter 로그에 `clamped` 가 매번 뜨는가? (§ 1.2)
- [ ] tool=1 활성화 됐는가? (§ 3.7)
- [ ] Fairino SDK 호출 시그니처가 SDK 예제와 일치? (§ 1.3)

---

## 5. 알려진 수정 이력 (regression 방지)

`tcp_pose.py` 에 다음 버그들이 있었음:

| 위치 | 잘못 | 수정 |
|------|------|------|
| `GetActualTCPPose(self.tcp_index)` | tcp_index=1 을 flag로 오해 | 인자 없이 호출 |
| `GetActualJointPosDegree(self.tcp_index)` | 동일 | 인자 없이 호출 |
| `GetInverseKinRef(0, p, q, -1)` | 4 args (3 only) | `(0, p, q)` |
| `GetInverseKin(0, p, -1)` | config 위치 인자 | `config=-1` kw |
| `MoveJ(j, p, 0, 0, v, a, o, [0]*6, 0, 0)` | 시그니처 오류 | `(j, tool=1, user=0, vel=v, acc=a, ovl=o)` |

증상이 위 패턴이면 SDK 시그니처 재확인 (`fairino_sdk/.../examples/movej&movel&movecart.py` 참조).
