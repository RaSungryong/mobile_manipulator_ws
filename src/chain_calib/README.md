# chain_calib — front_cam ↔ hand_cam 체인 캘리브레이션 (운영자 가이드)

바닥에 깐 태그 두 장으로 **front_cam과 hand_cam 사이의 변환 체인 오차**를
재고, 그 오차를 상수 보정값으로 풀어내는 패키지입니다. 팔은 **사람이 조그로**
자세를 잡고, 도구는 자세마다 `capture` 한 번으로 샘플을 저장합니다. 팔을
자동으로 움직이는 기능은 없습니다 (2026-09-15의 자동 스윕이 팔을 로봇 몸체에
부딪혀서 뺐습니다).

```
src/chain_calib/
  scripts/chain_calib.py        운영자 도구  check / capture / status / drop / solve
  scripts/check_chain_calib.py  합성 체인으로 solver 검증 (28개 검사)
  src/chain_calib/solver.py     수학 (AX = YB 피팅), ROS 없음
  src/chain_calib/session.py    샘플 저장, 자세 설명, 커버리지 조언, ROS 없음
log/chain_calib/<세션>/         samples.npz, meta.yaml, corrections.npz
```

---

## 1. 무엇을 재는가

locator가 쓰는 체인은 hand_cam이 본 태그 A에서 front_cam이 본 태그 B까지를
네 개의 고정 행렬로 잇습니다 (T_X2Y = X 프레임에서 본 Y의 자세).

```
T_A2B = inv(T_hc2A) · T_hc2ee · T_ee2ab · T_ab2mb · T_mb2fc · T_fc2B
        hand_cam 관측  hand-eye   팔 FK     팔 마운트  front_cam  front_cam 관측
```

- `T_hc2ee` (hand-eye), `T_ab2mb` (팔 마운트), `T_mb2fc` (front_cam)는 사람이
  재거나 계산해서 적어 둔 값이고, 각각이 실제와 같은지 따로 확인할 방법이
  없습니다.
- 두 태그를 같은 바닥에 자로 잰 간격으로 깔면 참값 `T_A2B`를 알게 됩니다.
  로봇 몸체는 세워 둔 채 팔만 여러 자세로 움직여 hand_cam이 A를 보면,
  자세마다 식이 하나씩 생깁니다.

```
inv(T_hc2ee) · S_i = D · (T_ee2ab_i · T_ab2mb · T_mb2fc) · F
```

- **D = hand 쪽 보정** (팔 끝 프레임): hand-eye가 틀렸을 때 남는 값
- **F = base 쪽 보정** (front_cam 프레임): 팔 마운트·front_cam·앞 태그 관측.
  몸체가 안 움직이므로 이 셋은 구분되지 않고 한 덩어리입니다.
- 자세 하나로는 D와 F를 못 가릅니다. **카메라를 여러 방향으로 기울인**
  자세들이 있어야 갈립니다. D는 변하는 팔 자세 앞에, F는 뒤에 있어서
  기울임에 따라 다르게 반응하기 때문입니다.

`solve`는 raw(현재 설정), hand만, base만, 둘 다 네 가지로 풀어 잔차를 나란히
보여주고 귀속을 판정합니다.

| 판정 | 뜻 |
|---|---|
| HAND side | hand만 고쳐도 잔차가 바닥까지 내려감 → hand-eye 오차 |
| BASE side | base만 고쳐도 내려감 → 팔 마운트 / front_cam / 앞 태그 쪽 |
| BOTH | 둘 다 고쳐야 내려감 |
| UNDETERMINED | 둘이 비슷 → 기울인 자세가 부족. 더 찍는다 |

정밀도 (합성 검증, 자세당 20프레임 평균, 18~24 자세): D·F 각각 1~2 mm / 0.1~0.2°.
자세당 잔차 바닥은 2~4 mm입니다. 0.4 m에서 보는 120 px 태그의 기울기가
프레임당 0.7° 노이즈라 20프레임 평균으로 0.15°까지 내려도 A→B 레버(0.5~1 m)에
곱해지면 2~4 mm가 남습니다. 그래서 **프레임 평균이 필수**이고 `--frames`
기본값이 20입니다.

---

## 2. 준비

### 태그

- 90 mm 태그 두 장. 기본값은 **A = 150 (hand_cam)**, **B = 149 (front_cam)**.
  다른 ID를 쓰면 `--hand-tag` / `--front-tag`로 알려줍니다.
- B는 front_cam 화면의 사용 가능 영역(범퍼 오른쪽) 가운데에. 정중앙일 필요는
  없고 여백만 있으면 됩니다.
- A는 팔이 카메라를 그 위 0.35~0.55 m에 두고 **네 방향으로 15~25° 기울일 수
  있는** 자리. 팔 베이스(몸체 중심에서 벽 쪽으로 100 mm)에서 옆으로 0.5~0.8 m가
  무난합니다. 1 m를 넘으면 기울인 자세가 도달 한계에 걸립니다.
- 두 태그의 변이 한 직선에 오도록 직선자(또는 팽팽한 실)를 대고 깝니다.
  인쇄 방향도 같게. 90° 단위 회전은 도구가 raw 체인에 가장 가까운 조합으로
  스스로 맞춥니다.

### 간격 재기 — 여기서 틀리면 결과가 그대로 틀립니다

중심 간 거리를 **1 mm** 안으로 압니다. 중심에는 자를 댈 표시가 없으니
검은 사각형의 **바깥 테두리**(검출기가 코너로 잡는 선)를 씁니다.

```
 |<-------------- E (바깥 변 → 바깥 변) -------------->|
 ┌──────┐                                    ┌──────┐
 │  A   │                                    │  B   │
 └──────┘                                    └──────┘
        |<---------- g (안쪽 변 → 안쪽 변) ---------->|

 검산: E − g = 태그 두 장 폭 ≈ 180 mm
 간격 = (E + g) / 2          (인쇄 크기가 정확히 90이 아니어도 무관)
```

- 줄자를 **0 눈금**에서 시작합니다. 10 cm 눈금에서 시작하는 습관이 있으면 반드시
  100을 뺍니다. 2026-09-15에 양쪽 배치에서 똑같이 100 mm 크게 나와 한참 헤맸습니다.
- 검산이 180에서 5 mm 넘게 벗어나면 어느 한쪽을 다른 변까지 잰 것입니다.
  E만 정확하면 간격 = E − 89.75 (태그 폭은 2026-09-15 피팅값)로도 됩니다.

### 로봇

- `mobile_manipulator.launch` 기동 (arm_node, robot_camera_node, lifter_node).
- **`path_tag_locator.launch`는 내려 둡니다** (팔의 두 번째 명령자).
- 리프트는 원점. **세션 중 몸체는 절대 움직이지 않습니다** — 앞 태그 관측이
  상수라는 것이 계산의 전제입니다.
- 팔 컨트롤러가 정상(연결 거부·폴트 없음)인지 robot_ui Arm 탭에서 자세가
  갱신되는지로 확인합니다.
- 처음 한 번 `catkin_make`를 돌려야 `rosrun chain_calib …`가 됩니다. 그 전에는
  `python3 src/chain_calib/scripts/chain_calib.py …`로 같은 명령을 씁니다.

---

## 3. 절차

### 3-1. 시작 자세와 점검

robot_ui Arm 탭으로 hand_cam을 태그 A 위 **0.40 m** 정도, 똑바로 아래를 보게
가져갑니다. 카메라 창을 `/hand_cam/tag_overlay`로 두면 태그가 보이는지 바로
압니다.

```bash
rosrun chain_calib chain_calib.py --spacing 1.01025 check
```

출력 예:

```
hand_cam tag: 5 frames, range 0.372 m, straight down, spin -2 deg, tag 1 mm off-axis
front_cam tag: 5 frames, at (+0.024, -0.005) m
raw chain error at this view: 10.9 mm / 1.06 deg  (along the tags' line -6.5, across -2.6, height +8.4 mm)
OK
```

`raw chain error`가 **지금 체인의 오차**입니다. 100 mm처럼 크게 나오면 간격을
잘못 쟀거나 태그 ID를 바꿔 넣은 것입니다 (§5).

### 3-2. 자세마다 capture

```bash
rosrun chain_calib chain_calib.py --spacing 1.01025 capture log/chain_calib/$(date +%Y%m%d)
```

찍을 때마다 그 자세의 설명, 그 자세에서의 체인 오차, 그리고 **coverage** 두
줄이 나옵니다.

```
coverage: 5 sample(s); 3 tilted >= 12 deg from 2 of 4 directions; spins [-90, 0]; rotation diversity 48 deg
          still needed: at least 8 samples (have 5); tilts toward more directions (missing: tag +x, tag +y)
```

`READY to solve`가 나올 때까지 자세를 바꿔 가며 반복합니다. 권장 순서 (약 10~12개):

| # | 자세 | 방법 |
|---|---|---|
| 1 | 똑바로 아래, 0.40 m | 시작 자세 |
| 2 | 똑바로 아래, 0.50 m | z 조그 +100 |
| 3~6 | **네 방향**으로 15~25° 기울임 (태그 +x, −x, +y, −y 쪽으로 카메라를 옮기며 기울임) | 기울인 뒤 태그가 화면 안에 들도록 xy 조그 |
| 7~8 | 기울인 자세에서 카메라를 자기 축으로 **90° 스핀** | rz 조그 90 |
| 9~10 | 똑바로 아래에서 스핀 ±90° | rz 조그 |

- 태그가 화면 밖이면 `capture`가 실패 메시지를 내고 저장하지 않습니다.
- 조그가 끝나고 팔이 완전히 멈춘 뒤 찍습니다 (도구가 0.3 s 기다립니다).
- 잘못 찍은 것은 `drop DIR v03`으로 뺍니다. `status DIR`로 목록과 커버리지를
  언제든 봅니다.
- 한 세션 안에서는 `--spacing`, 태그 ID를 바꿀 수 없습니다 (도구가 거부).

### 3-3. 풀이

```bash
rosrun chain_calib chain_calib.py solve log/chain_calib/<세션>
```

읽는 법:

- 표의 `raw` 줄 = 현재 체인 오차 (rms / 최대). 자세별 잔차 표에서 특정 자세만
  4배 넘게 크면 자동으로 제외됩니다 (팔이 밀렸거나 움직이는 중에 찍힌 것).
- `verdict`가 귀속. `UNDETERMINED`면 `coverage`가 말하는 방향의 자세를 더 찍고
  다시 `solve`.
- `[hand]`의 `corrected T_hc2ee`, `[base]`의 `corrected T_ab2mb`가 각 가설의
  보정 결과. `jackknife sd`는 샘플 하나씩 빼고 다시 풀었을 때의 흔들림이며 곧
  불확실도입니다. 보정값보다 sd가 크면 그 값은 의미가 없습니다.

### 3-4. 반영 (자동으로 하지 않습니다)

- **HAND side**: `solve … --write-hand-eye hand`가
  `path_tag_locator/config/hand_eye/T_hc2ee_chain_<날짜>.npz`를 씁니다.
  `locator.yaml`의 `hand_eye.npz_path`를 그 파일로 바꾸고 캘리브레이션 노드를
  재시작합니다. 기존 파일은 그대로 남습니다.
- **BASE side**: `corrected T_ab2mb`를 `extrinsics.yaml`에 넣으면 locator 체인은
  맞지만, **같은 행렬이 `robot.yaml arm_calibration`(pose 모드 IK)에도**
  있습니다. 둘을 같이 바꿔야 하고 부호 규약 확인이 먼저라 별도 작업입니다.
  F의 회전 성분은 "팔 마운트에 틸트가 없다"는 가정의 검증값이기도 합니다.
- 세션 디렉터리의 `corrections.npz`에 D, F, 참값이 항상 남습니다.

---

## 4. 2026-09-15 첫 세션 결과 (`log/chain_calib/20260915_c`)

자동 스윕으로 11개를 찍다가 팔이 몸체와 충돌. 충돌 순간의 v09·v10(잔차 850 mm)
을 제외한 9개로:

| 피팅 | 잔차 rms |
|---|---|
| raw | 11.0 mm / 0.94° |
| hand만 | 7.4 mm |
| base만 | 6.3 mm |
| 둘 다 | 4.3 mm (sd 7~16 mm, 무의미) |

판정 UNDETERMINED. 기울인 자세 8개가 전부 태그 −x·−y 쪽뿐이라 반대 방향이
없었습니다. 확실한 것: 100 mm급 오차는 없고(팔 마운트 −100 mm, camera_offset
0.55, hand-eye 모두 mm 단위로 맞음), 체인은 바닥면 약 5 mm, 높이 약 +8~14 mm
계통 오차. 다음 세션은 +x, +y 방향 기울임 4개와 스핀 2개를 더해 다시 풉니다.

---

## 5. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `raw chain error` ~100 mm, 회전은 1° 미만 | 간격 오독. E − g ≈ 180인지 검산. 좌·우 어느 배치에서도 같은 부호면 자 문제 |
| `hand_cam does not see tag` | 태그 ID (`--hand-tag`) / 카메라 높이 / 화면 밖. overlay로 확인 |
| `FAIL: … /arm/state` | arm_node 없음, 또는 컨트롤러 폴트 (충돌 뒤 `Connection refused`) → 티치펜던트 복구 |
| 잔차 바닥이 5 mm 넘음 | 팔이 흔들리는 중에 찍힘(멈춘 뒤 다시), 태그가 들뜸, `--frames 30` |
| `UNDETERMINED` | 기울임 방향 부족. coverage의 `missing` 방향으로 더 찍는다 |
| 특정 자세만 잔차가 4배 | 자동 제외됨. `status`로 확인하고 `drop` |
| `this session was started with spacing …` | 새 배치는 새 디렉터리 |

## 6. 검증

```bash
python3 src/chain_calib/scripts/check_chain_calib.py     # 28 checks
```

실제 hand-eye·extrinsics로 합성 체인을 만들어 hand-eye에 2° / 15 mm, 마운트에
1° / 8 mm 오차를 심고 되찾는지, 귀속이 맞게 나오는지, 단일 프레임이면 정밀도가
얼마나 나빠지는지, 저장/불러오기와 커버리지 조언이 맞는지 봅니다.
