# chain_calib — front_cam ↔ hand_cam 체인 캘리브레이션 (운영자 가이드)

인쇄된 **A0 태그 시트**(`sheet/`)를 참값(ground truth)으로 삼아 **front_cam과
hand_cam 사이의 변환 체인 오차**를 재고, 그 오차를 체인 안의 고정 행렬에 대한
상수 보정값으로 풀어내는 패키지입니다. 팔은 **사람이 조그로** 자세를 잡고,
도구는 자세마다 `capture` 한 번으로 샘플을 저장합니다. 팔을 자동으로 움직이는
기능은 없습니다 (2026-09-15의 자동 스윕이 팔을 로봇 몸체에 부딪혀서 뺐습니다).

시트는 태그 11장(200, 300~309)이 **한 강체 위에 설계 좌표로** 있는 기준물이라,
사람이 자로 재서 태그를 깔 일이 없습니다. 카메라마다 **보이는 모든 태그의 코너를
한꺼번에 PnP**에 넣어 `T_cam2W`(시트 좌표계 W = 태그 200)를 구하고, 두 카메라가
어느 태그를 봐도 됩니다.

```
src/chain_calib/
  scripts/chain_calib.py        운영자 도구  check / capture / status / drop / solve
  scripts/check_chain_calib.py  오프라인 검증 (PDF 코너 규약 + 합성 세션, 37개 검사)
  scripts/verify_chain.py       보정 검증: 체인으로 계산한 자세로 hand_cam을 태그 위로 보내 실제 중심 편차를 잼
  src/chain_calib/solver.py     수학 (AX = YB 피팅, 홀드아웃 평가, 태그 쌍 지표), ROS 없음
  src/chain_calib/sheet.py      시트 레이아웃, 프레임 누적, multi-tag PnP, ROS 없음
  src/chain_calib/session.py    샘플 저장, 자세 설명, 커버리지 조언, ROS 없음
  sheet/                        A0 시트 PDF(인쇄용) + layout.json(설계 좌표) + 인수인계 문서
log/chain_calib/<세션>/         samples.npz, corners.json, meta.yaml, corrections.npz
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
- 시트 위 태그는 모두 W의 평행이동이므로 참값 `T_A2B`를 JSON에서 바로 알고,
  카메라마다 `T_cam2W`를 구하면 **측정된** `T_hc2fc = T_hc2W · inv(T_fc2W)`가
  자세마다 하나씩 나옵니다. 로봇 몸체는 세워 둔 채 팔만 여러 자세로 움직이면
  자세마다 식이 하나씩 생깁니다.

```
inv(T_hc2ee) · S_i = D · (T_ee2ab_i · T_ab2mb · T_mb2fc) · F
```

- **D = hand 쪽 보정** (팔 끝 프레임): hand-eye가 틀렸을 때 남는 값
- **F = base 쪽 보정** (front_cam 프레임): 팔 마운트·front_cam 쪽. 몸체가 안
  움직이므로 이 둘은 구분되지 않고 한 덩어리입니다.
- 자세 하나로는 D와 F를 못 가릅니다. **카메라를 여러 방향으로 기울인**
  자세들이 있어야 갈립니다. D는 변하는 팔 자세 앞에, F는 뒤에 있어서
  기울임에 따라 다르게 반응하기 때문입니다.

`solve`는 raw(현재 설정), hand만, base만, 둘 다 네 가지로 풀어 잔차를 나란히
보여주고 귀속을 판정합니다. (인수인계 문서 §6.5의 `Y_fixed` = base, `X_fixed`
= hand, `XY` = joint. 문서의 2단계 재투영 BA는 아직 없고 1단계 SE(3) 최소제곱만
있습니다 — 자세당 20프레임 평균으로 합성 검증에서 1 mm 안에 들어와 아직 필요가
없었습니다.)

| 판정 | 뜻 |
|---|---|
| HAND side | hand만 고쳐도 잔차가 바닥까지 내려감 → hand-eye 오차 |
| BASE side | base만 고쳐도 내려감 → 팔 마운트 / front_cam 쪽 |
| BOTH | 둘 다 고쳐야 내려감 |
| UNDETERMINED | 둘이 비슷 → 기울인 자세가 부족. 더 찍는다 |

정밀도 (합성 검증, 자세당 20프레임 평균, 20~24 자세): D·F 각각 0.3 mm / 0.03°
안, 자세당 잔차 바닥 0.5~0.7 mm. 태그 한 장(90 mm)의 변 길이 차이 대신
150~600 mm 기선의 여러 태그로 기울기를 재기 때문입니다. **프레임 평균은
필수**이고 `--frames` 기본값이 20입니다.

---

## 2. 준비

### 인쇄

`sheet/A0_landscape_tag200_300-309_5x2_FINAL2.pdf`를 A0 가로(1189 × 841)로
**원본 크기(100 %)** 인쇄합니다. "용지에 맞춤" 금지. 태그 200이 왼쪽 위,
300~309 격자(5행 2열, 150 mm 간격)가 오른쪽에 있습니다. 시트 평탄도는 고려하지
않습니다 — 바닥에 테이프로 팽팽히 붙입니다.

### 실측 — 안 하면 보정값에 그대로 들어갑니다

플로터는 급지 방향으로 0.1~0.3 % 늘거나 줄고, 600 mm에서 1~2 mm는 체인 오차와
구분되지 않습니다. 검은 사각형 **바깥 변**을 기준으로 강철자로 잽니다.

| 측정 | 설계값 | 넣는 값 |
|---|---|---|
| 200 중심 ↔ 301 중심 (가로) | 1000 mm | `--sx` = 실측 / 1000 |
| 300 중심 ↔ 308 중심, 301 ↔ 309 (세로, 평균) | 600 mm | `--sy` = 실측 / 600 |
| 태그 한 변 (5장 이상 캘리퍼스, 평균) | 90.00 mm | `--tag-size` (m) |

중심에는 표시가 없으니 두 태그의 **바깥 변 → 바깥 변**을 재고 태그 폭(실측)을
뺍니다:

```
 |<-------------- E (바깥 변 → 바깥 변) -------------->|
 ┌──────┐                                    ┌──────┐
 │ 200  │                                    │ 301  │
 └──────┘                                    └──────┘
 중심 간격 = E − 태그 폭.   줄자는 0 눈금에서 (10 cm 눈금에서 시작하는 습관 주의)
```

세 값을 `capture`마다 같이 넘기거나, 찍어 둔 뒤 `solve --sx --sy --tag-size`로
넘겨도 됩니다 — 코너가 `corners.json`에 남아 있어 `solve`가 그 스케일로 전부
다시 풉니다. 안 넘기면 설계값으로 돌고 경고가 뜹니다.

### 배치 (도구는 어느 태그를 봐도 됩니다)

**실제 배치(2026-09-21 세션): front_cam이 200을, hand_cam이 300~309 격자를 봅니다.**
front_cam은 태그 1장(정면 뷰라 뒤집힘 모호성은 없음)이고 그 관측은 세션 내내
상수이므로, 그 오차는 base 쪽 F에 그대로 흡수됩니다 — F를 읽을 때 "front_cam
관측 포함"으로 이해하면 됩니다. `--frames 30`으로 그 잡음을 조금 줄일 수 있습니다.

⚠️ **종이는 평면이 아닙니다 — 이것이 이 방법의 실제 한계입니다 (2026-09-21).**
태그 한 장의 면외 기울기는 시트의 방향이 아니라 **그 자리 종이의 국소 경사**입니다.
태그 200은 A0 모서리에서 100 mm라 잘 뜹니다. 첫 시도에서는 그 자리의 경사 1.3°가 그대로
F의 roll로 들어갔고, 시트를 다시 깐 재측정에서 발각돼 그 시도는 폐기했습니다. 그래서 `solve`/`check`는 기본으로
front_cam 관측의 회전을 **바닥 평행 prior**로 바꿔 씁니다(`--front-rotation level`;
level 프레임은 ground-plane 보정으로 바닥이 z = const인 프레임이고 시트는 그 바닥에
놓이므로 정당합니다; yaw와 이동은 측정값 유지). `check`/`capture`가 찍는
`paper slope under those tags`가 1°를 넘으면 그 자리 종이를 눌러 붙이세요.
**격자 쪽 종이의 경사는 prior로 못 덮습니다** — 그것이 곧 마운트 틸트 측정값이기
때문입니다. 격자는 물리적으로 평평해야 하고, 시트는 **단단한 평판 위에** 붙이는
것이 맞습니다. 600 mm 격자가 9 mm 휘면 pitch 0.9°가 그대로 결과에 들어갑니다.

아래는 그 대안(front_cam이 300/301 두 장을 보는 배치)입니다. front_cam은 렌즈
바로 아래 0.30 m를 보고 사용 가능한 시야가 약 0.34 × 0.29 m(범퍼가 왼쪽 1/3을
가림)라 **150 mm 간격의 태그 두 장**은 들어가지만 200과 격자는 함께 못 봅니다.
팔의 작업 영역은 로봇 오른쪽입니다. 그래서

```
                로봇 전진 →  (mb +x)
      ┌─────────────┐
      │   로봇 몸체   │ 0.90 × 0.70
      │  팔 베이스 ●  │ (0, -0.10)
      └─────────────┘  ▲ front_cam 렌즈 바로 아래 (0.55, 0)
                       │
        [300] [301]  ← 이 두 장이 front_cam 시야에 (시트 W x = mb +x)
        [302] [303]
        [304] [305]  ← hand_cam은 여기 4~8장 위를 0.4~0.5 m 높이에서
        [306] [307]     (팔 베이스에서 0.4~0.8 m, 몸체 앞·오른쪽)
        [308] [309]
   (200은 몸체 아래로 들어가 아무도 안 봄 — 상관없음)
```

즉 시트를 **격자가 로봇 오른쪽으로 뻗도록** 놓고, 300/301 쌍이 front_cam
십자선 근처(300의 중심이 W (0.85, 0)쯤 십자선에 오면 300이 범퍼에 안 가림)에
오게 몸체를 세웁니다. `check`가 front_cam이 보는 태그와 PnP rms를 찍어 주니
그걸 보며 시트를 밀면 됩니다. hand_cam은 **항상 태그 2장 이상**을 보게 합니다
(1장이면 평면 PnP의 뒤집힘 모호성이 있어 `capture`가 거부합니다; `--force`로
저장은 가능).

인수인계 문서 §13의 미확정 항목에 대한 이 워크스페이스의 답: ROS1 Noetic,
검출기는 `robot_camera_node`의 dt_apriltags(코너만 사용, 카메라별 포즈 필드는
안 씀); 프레임은 locator 체인 그대로 — `ab`(팔 베이스), `ee` = `/arm/state`의
**플랜지**, 두 카메라의 optical frame; FK는 `/arm/state`; hand_cam D435는
CameraInfo D를 그대로 적용(현재 펌웨어는 0 보고), front_cam은 ground-plane
보정 코너(D 없음, `T_mb2fc_level`); **받침대 위 8 cm 태그 A 구성은 지원하지
않음** — 시트는 평면이고 로더가 z ≠ 0 태그를 거부합니다; 추정 모드는 셋 다
풀어 판정(§1); 합격 기준은 홀드아웃 ≤ 학습 × 1.5, 잔차 바닥 ~1 mm(합성).

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

robot_ui Arm 탭으로 hand_cam을 격자 위 **0.40 m** 정도, 똑바로 아래를 보게
가져갑니다. 카메라 창을 `/hand_cam/tag_overlay`로 두면 태그가 보이는지 바로
압니다.

```bash
rosrun chain_calib chain_calib.py check
```

출력 예:

```
sheet .../layout.json: 11 tags [...], tag 90.00 mm, scale sx 1.00000 sy 1.00000 (W = tag 200)
  hand_cam : tags [304, 305, 306, 307] (20/20 frames), PnP rms 0.21 px, corner scatter 0.12 px; range 0.412 m, straight down, spin +2 deg
  front_cam: tags [300, 301] (20/20 frames), PnP rms 0.18 px, corner scatter 0.09 px; sheet origin at (-0.851, +0.004, 0.302) m
  lift 0.000 m, TCP [...]
  raw chain error at this view: tag 305 (hand_cam) -> tag 300 (front_cam): 10.9 mm / 1.06 deg  (in B's frame: x -6.5, y -2.6, z +8.4 mm)
OK
```

`raw chain error`가 **지금 체인의 오차**(hand_cam 태그 → front_cam 태그의 상대
자세를 로봇이 계산한 것 vs 시트 참값)입니다. 100 mm처럼 크게 나오면 시트
스케일·하드웨어를 먼저 의심합니다 (§4).

**흔들림을 보는 값은 `corner scatter`이지 `PnP rms`가 아닙니다.** scatter는
프레임끼리 얼마나 일치하는가(0.3 px를 넘으면 팔이 아직 움직이는 중), rms는
시트·카메라 모델이 그 코너에 얼마나 맞는가입니다. 이 셋업의 rms 바닥은
**0.6~0.7 px**로, 2026-09-21 세션의 정상 뷰 전부에서 그랬습니다(원인:
hand_cam의 미교정 왜곡, D435가 D = 0을 보고) — 즉 `rms 0.70 px / scatter
0.04 px`는 정상이고 더 기다려도 안 내려갑니다. rms가 1.2 px를 넘는 뷰는
실제로 이상치이고 `solve`가 알아서 뺍니다.

### 3-2. 자세마다 capture

```bash
rosrun chain_calib chain_calib.py --sx 1.0012 --sy 0.9991 --tag-size 0.0899 capture log/chain_calib/$(date +%Y%m%d)
```

찍을 때마다 그 자세의 설명, 그 자세에서의 체인 오차, 그리고 **coverage** 두
줄이 나옵니다.

```
coverage: 5 sample(s); 3 tilted >= 12 deg from 2 of 4 directions; spins [-90, 0]; rotation diversity 48 deg
          still needed: at least 8 samples (have 5); tilts toward more directions (missing: tag +x, tag +y)
```

`READY to solve`가 나올 때까지 자세를 바꿔 가며 반복합니다. **격자 위치를
옮겨 가며** 찍습니다 — 한 샘플은 그 순간 보이는 태그(2~4장)로 `T_hc2W`를 풀고
모든 샘플이 같은 W로 묶이므로, 한 번에 격자 전체가 보일 필요가 없습니다.
권장: 네 구역 × 4~5개 = 16~20개.

| 구역 (hand_cam 중앙에 둘 태그) | 그 자리에서 (4~5개) |
|---|---|
| 300/301 | 수직 0.40 m → 시트 **+x**로 15~20° 기울임 → **−y**로 15~20° → (안전하면) 스핀 +20° |
| 302/303 | 수직 0.50 m → **−x** 15~20° → **+y** 15~20° |
| 304/305 | 수직 0.40 m → **+x** 15~20° → **+y** 15~20° → (안전하면) 스핀 −20° |
| 306/307 | 수직 0.45 m → **−x** 15~20° → **−y** 15~20° |
| 308/309 | 수직 0.40 m → 네 방향 중 팔이 몸체에서 **멀어지는** 쪽 2개 |

기울임은 "카메라를 그 방향으로 옮기면서 시트 쪽으로 기울이기"입니다 (기울인
뒤 태그 2장 이상이 화면에 들도록 xy 조그). 위치를 옮기는 것만으로는 D/F가
갈리지 않습니다 — 판정에 필요한 건 **회전 다양성**이고, 시트 ±x·±y 네 방향
기울임이 그것을 채웁니다. **스핀은 필수가 아닙니다**: 손목을 90° 돌리는 자세는
팔이 몸체 가까이 있을 때 엔드이펙터가 링크와 부딪힐 수 있어(2026-09-18) 안전한
자리에서 ±20° 정도만, 아니면 생략합니다. 네 방향 기울임이 6개 이상이면
`coverage`가 스핀을 요구하지 않습니다.

⚠️ **몸체 쪽으로 기울이는 자세는 팔이 로봇에 가까워집니다.** 격자가 몸체에
붙어 있으면 몸체 쪽 방향(예: 시트 −x가 몸체 쪽이면 −x)은 5~10°만, 반대쪽은
20°까지. 한 방향이 빠져도 나머지 셋으로 판정은 됩니다.

- 태그가 화면 밖이거나 1장뿐이면 `capture`가 메시지를 내고 저장하지 않습니다.
- 조그가 끝나고 팔이 완전히 멈춘 뒤 찍습니다 (도구가 0.3 s 기다립니다).
- 잘못 찍은 것은 `drop DIR v03`으로 뺍니다. `status DIR`로 목록과 커버리지를
  언제든 봅니다.
- 한 세션 안에서 시트 파일이나 front_cam 보정 상태(`ground_plane`)를 바꾸면
  도구가 거부합니다. 새 배치는 새 디렉터리.

### 3-3. 풀이

```bash
rosrun chain_calib chain_calib.py solve log/chain_calib/<세션>
rosrun chain_calib chain_calib.py solve log/chain_calib/<세션> --holdout-every 4      # 4개마다 1개를 검증용으로 빼고 풂
rosrun chain_calib chain_calib.py solve log/chain_calib/<세션> --sx 1.0012 --sy 0.9991 --tag-size 0.0899   # 스케일을 나중에 반영
```

읽는 법:

- 표의 `raw` 줄 = 현재 체인 오차 (rms / 최대). 자세별 잔차 표에서 특정 자세만
  4배 넘게 크면 자동으로 제외됩니다 (팔이 밀렸거나 움직이는 중에 찍힌 것).
- **hold-out**: 피팅에서 뺀 자세들에 보정값을 적용한 잔차. 학습 잔차의 1.5배
  안이면 과적합이 아닙니다 (인수인계 문서 §9.3). `--holdout v10 v11`로 직접
  고르거나 `--holdout-every k`.
- **the user's metric**: 자세마다 hand_cam 광축에 가장 가까운 태그 A와
  front_cam 광축에 가장 가까운 태그 B의 `T_A2B`를 체인으로 계산한 것 vs 시트
  참값 — 평균 / 표준편차 / rms / 95 %. **평균(bias)만 보정으로 줄어들고
  표준편차(random)는 남습니다.** raw 줄의 bias 벡터(B 프레임)가 "지금 체인이
  어느 쪽으로 얼마나 틀렸나"입니다.
- **residual vs view**: 잔차와 거리·기울기·스핀의 상관 r. |r| > 0.7이면 상수
  외부 파라미터가 아닌 것(검출 / intrinsic / 태그 크기 실측)이 원인입니다.
- `verdict`가 귀속. `UNDETERMINED`면 `coverage`가 말하는 방향의 자세를 더 찍고
  다시 `solve`.
- `[hand]`의 `corrected T_hc2ee`, `[base]`의 `corrected T_ab2mb`가 각 가설의
  보정 결과. `jackknife sd`는 샘플 하나씩 빼고 다시 풀었을 때의 흔들림이며 곧
  불확실도입니다. 보정값보다 sd가 크면 그 값은 의미가 없습니다.

### 3-4. 반영 (자동으로 하지 않습니다)

- 모든 고정 변환은 **`apriltag_nav/config/tf/tf_chain.yaml`** 한 곳에 있고
  (2026-09-21; 변환마다 `<이름>.npz` 쌍둥이), `tools/tf_chain_tool.py`로
  보고/검사/기록합니다. 설계값과 측정 경위는 그 파일의 주석에 있습니다.
- **HAND side**: `solve … --write-hand-eye hand`가
  `apriltag_nav/config/tf/T_hc2ee_chain_<날짜>.npz`를 씁니다(적용 아님).
  적용은 `tf_chain_tool.py set T_hc2ee --npz <그 파일> --source "…"` 뒤
  캘리브레이션 노드 재시작.
- **BASE side**: `corrected T_ab2mb`를 `tf_chain_tool.py set T_ab2mb --matrix …
  --source "…"`로 넣습니다(locator 체인, pose 모드 IK, `robot_sim`, 플랜
  생성기가 읽음; arm_node + 캘리브레이션 노드 재시작). **2026-09-22 팔 세션의
  값이 이렇게 반영되어 있습니다** — 출처와 수치는 `tf_chain.yaml`의 `T_ab2mb`
  블록 주석과 CLAUDE.md Work Log 2026-09-22 (09-21 세션과 그 기록 문서는 관절각이
  없어 오프셋을 적용할 수 없으므로 2026-09-22에 삭제; git에 있음).
  `check_front_cam_extrinsics.py`가 정규직교·설계 근방(3° / 50 mm)을 검사합니다.
  **같은 변환이 `robot.yaml arm_calibration`(pose 모드 IK)과 플래너 URDF의
  `mobile_to_base`에도 들어갑니다**(2026-09-21부터; `check_pose_vs_joint.py`가 셋의
  일치를 검사) — locator가 태그를 위치시키는 체인과 팔이 거기로 가는 체인이 같아야
  왕복 오차가 상쇄됩니다. robot.yaml 값은 inv(T_ab2mb)를 offset + Rz(yaw)Ry(ty)Rx(tx)
  로 분해한 것이고(`arm_node` 재시작), URDF는 180° 돈 mobile_base에서 본 같은
  변환입니다. 기존 `rrt_final_path_*`는 설계 마운트로 계획된 것이라 재생성이 필요합니다.
  ⚠️ F의 roll/pitch는 마운트 틸트가 아니라
  **이번 주차의 차체 자세 + 종이 경사를 포함한 값**(±1°)입니다 — 로봇 상수로
  믿을 것은 yaw와 면내 이동입니다(그 문서 §5-3).
- 세션 디렉터리의 `corrections.npz`에 D, F, 스케일, 홀드아웃 목록이 항상
  남습니다.

---

### 3-5a. 경로 스크립트 — hand_cam 렌즈를 시트 좌표의 점들로 (2026-09-21)

`scripts/sheet_path.py`: **hand_cam 렌즈를 TCP로** 하여 태그 200 프레임의 점 목록
(x, y, h — h는 시트 위 높이)으로 순차 이동, 각 점에서 `--dwell`초 정지 후 hand_cam이
시트를 다시 풀어 **렌즈가 실제로 W 어디에 갔는지**를 명령값과 비교합니다(xy 오차가
주 지표). 기본 점은 격자 10개 태그 위 0.50 m. 플랜 확인 후 한 번 y를 누르면 자동으로
돕니다. 결과는 점마다 `sheet_path_<HHMMSS>.csv`에 즉시 추가.

```bash
rosrun chain_calib sheet_path.py log/chain_calib/<세션> --dry-run          # 플랜만
rosrun chain_calib sheet_path.py log/chain_calib/<세션>                    # 10점, 3 s 대기
rosrun chain_calib sheet_path.py log/chain_calib/<세션> --points "0.925,0.075,0.40;1.000,0.300,0.45" --dwell 5
```

### 3-5. 보정 검증 — 체인으로 팔을 태그 위로 보내 본다

`solve`의 수치가 아니라 **실제 동작**으로 확인합니다: front_cam이 보는 200을
기준으로 체인이 계산한 "태그 k 위 0.50 m, 수직" 자세로 hand_cam을 보내고,
거기서 hand_cam이 k를 화면 어디에 보는지 잽니다. 그 중심 편차와 거리 오차가
곧 그 자세에서의 체인 오차입니다.

```bash
# hand_cam을 격자 위 ~0.5 m, 수직으로 놓고 시작 (첫 목표가 0.35 m 안에 있어야 움직입니다)
rosrun chain_calib verify_chain.py plan log/chain_calib/<세션> --fit base   # 10개 목표 자세 출력·CSV, 이동 없음
rosrun chain_calib verify_chain.py run  log/chain_calib/<세션> --fit base   # 자세마다 Enter → MoveL → 측정
rosrun chain_calib verify_chain.py run  log/chain_calib/<세션> --fit none   # 보정 전 체인으로 같은 것 (대조)
```

열 자세는 방향·높이가 같고 150 mm씩 평행이동만 하므로 MoveL 경로가 단순합니다.
그래도 이동마다 Enter를 받고, `s`로 건너뛰고 `q`로 그만둘 수 있습니다. 결과는
`<세션>/verify_result_<fit>_<시각>.csv`. 기대값: `--fit base`에서 중심 편차
5~10 mm(체인 잔차 9 mm + 자세 잡음), `--fit none`에서 30~40 mm.

## 4. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| `hand_cam does not see any tag` / `none of the detected tags … is on the sheet` | overlay로 확인; 시트 위가 아닌 태그(바닥 경로 태그 등)만 보임 |
| `NOT saved (single tag, IPPE flip ambiguity …)` | hand_cam이 태그 1장만 기울여 봄 → 2장 보이게 옮김. 정말 필요하면 `--force` |
| `! sheet scale is the DESIGN value` | `--sx --sy --tag-size` 미입력. §2 실측 후 `solve`에 넘기면 됨 |
| `this session's front_cam detections were level, now physical` | 세션 도중 robot.yaml `ground_plane.front_cam.enabled`가 바뀜. 새 세션 |
| raw 오차가 스케일과 함께 움직임 (`--sx` 0.1 % 바꾸면 1 mm 움직임) | 정상 — 그래서 실측이 필수 |
| `FAIL: … /arm/state` | arm_node 없음, 또는 컨트롤러 폴트 (충돌 뒤 `Connection refused`) → 티치펜던트 복구 |
| 잔차 바닥이 3 mm 넘음 | 팔이 흔들리는 중에 찍힘(멈춘 뒤 다시), 시트가 들뜸, `--frames 30` |
| `UNDETERMINED` | 기울임 방향 부족. coverage의 `missing` 방향으로 더 찍는다 |
| 특정 자세만 잔차가 4배 | 자동 제외됨. `status`로 확인하고 `drop` |

## 5. 검증

```bash
source devel/setup.bash
python3 src/chain_calib/scripts/check_chain_calib.py     # 35 checks
```

(1) PDF를 래스터화해 **실제 검출기(dt_apriltags)** 로 11장을 검출하고 코너 0이
인쇄물 좌하, 코너 순서가 `_CORNER_ORDER`와 같음을, 가상 카메라 PnP가 R = I를
주는지 확인하고, (2) 렌더링한 코너로 hand_cam(왜곡 포함) / front_cam PnP가
1 mm / 0.1° 안에 들어오는지, (3) 위 §2 배치의 합성 세션에서 hand-eye 2° / 15 mm·
팔 마운트 1° / 8 mm 오차를 심어 되찾고(0.3 mm / 0.03° 안), 사용자 지표가 24 mm
→ 0.4 mm로 내려가고, 홀드아웃이 학습과 같은 수준이고, 인쇄 스케일 0.3 %를
무시하면 1.5 mm의 가짜 체인 오차가 생기며 코너 재풀이로 없어짐을, 저장/불러오기를
확인합니다.

## 5-A. 팔 관절 오프셋 캘리브레이션 — `arm_offsets.py` (2026-09-21)

체인이 맞은 뒤에도 남는 ±15~20 mm는 **팔 자체**의 오차다: 같은 렌즈 위치라도 팔
구성이 다르면 오차가 다르므로(스핀 시험) 강체 마운트 오차가 아니라 **관절 영점
오프셋·링크 기하**다. 시트가 뷰마다 hand_cam 렌즈의 참 자세를 주고 URDF FK가 모델
자세를 주니, 그 차이를 여러 구성에서 모아 J2~J6 오프셋을 최소자승으로 푼다(J1은
체인의 base yaw와 축퇴라 0 고정; 시트의 팔 베이스 내 자세 6개는 자유).

```
T_W2hc_pred = inv(T_ab2W) · FK(q + δq) · inv(T_hc2ee)        URDF FK + hand-eye
residual    = project(K_hand, inv(T_W2hc_pred) · 시트 코너) − 관측 코너 px   (기본, --residual reproj)
            | pose_error(T_W2hc_pred, inv(T_hc2W))                        (--residual pose, 첫 버전)
```

⚠️ **잔차는 코너 재투영(px)이다.** 첫 실제 세션(`20260921_arm`, 65뷰)에서 hand_cam
이 두 태그만 보는 뷰가 대부분이었고, 두 태그 PnP는 평면 모호성 때문에 **가짜 기울기
3~7° + 그에 맞는 깊이 오차 30~110 mm**를 낸다 — `capture`의 `raw chain error … z
±30~110 mm`가 그것이다(팔이 아니라 PnP). PnP 자세를 맞추던 첫 버전은 그 잡음을
피팅했다(강체 21 mm rms, 오프셋이 홀드아웃에서 더 나빠짐). 코너 픽셀을 맞추면 두
태그 뷰는 코너가 구속하는 만큼만(위치·스핀은 잘, 기울기는 약하게) 기여한다. 0.45 m
에서 1 px ≈ 0.75 mm.

### 수집 — 무엇이 다른가

체인 캘리브레이션은 **회전 다양성**(기울임 4방향)이 핵심이었지만, 관절 오프셋은
**관절 공간의 다양성**이 핵심이다. 팔이 실제로 일할 구성 전부를 덮어야 그 안에서
모델이 맞는다.

| | 체인 (`solve`) | 팔 오프셋 (`arm_offsets`) |
|---|---|---|
| 태그 | front_cam 근처 몇 장으로 충분 | **300~309 전부**, 두 열 모두 |
| 높이 | 0.40~0.50 | **0.35 / 0.45 / 0.55 세 단** |
| 스핀 | 선택 | **60~150° 여러 단** (손목 J6 + J4/J5 구성이 바뀜) |
| 기울임 | 4방향 필수 | 각 자리에서 1~2개(J4/J5 구성) |
| 개수 | 16~24 | **30~40, 서로 다른 구성** — 같은 자세 반복은 무의미 |

`capture`는 2026-09-21부터 `/arm/state`의 관절각을 샘플에 함께 저장한다(그 전
세션은 관절각이 없어 이 피팅에 못 쓴다). 수집 자체는 §3-2와 같고(`check` →
`capture` 반복), 한 세션이 `solve`(체인 확인)와 `arm_offsets`(팔) 양쪽에 쓰인다.

⚠️ **수동 조그로만** 움직인다. 플랜지가 팔 베이스 평면 **아래**로 내려가면서 수평
reach가 0.65 m 미만인 자세(베이스 옆으로 접힌 자세)는 팔꿈치/공구가 차체에 닿는다
— 2026-09-21에 충돌했다. 낮은 높이(0.35)는 격자 앞쪽 열(300~305)에서만.

### 풀이

```bash
A="--sx 1.0 --sy 1.0 --tag-size 0.090 --holdout-every 4"
rosrun chain_calib arm_offsets.py log/chain_calib/<세션> $A                 # J2~J6 오프셋
rosrun chain_calib arm_offsets.py log/chain_calib/<세션> $A --links         # + 상완(0.700)/전완(0.586) 길이
rosrun chain_calib arm_offsets.py log/chain_calib/<세션> $A --hand-eye-free # + T_hc2ee 6-DOF 보정 (J6은 고정: hand-eye 스핀과 축퇴)
   --exclude v13 …   --min-tags 3   --quick(jackknife 생략)   --write-hand-eye <npz>(후보 저장, 설치 아님)
```

읽는 법: (1) `URDF FK vs the controller's TCP` — 1 mm 안이어야 URDF가 컨트롤러의
모델이다. (2) `rigid`(오프셋 0, 시트 자세만 자유) vs `offsets` rms — 오프셋이 진짜면
크게 내려가고 홀드아웃도 따라 내려간다. (3) 오프셋 J2~J6과 jackknife sd — sd보다
작은 오프셋은 의미 없다. 손목(J4~J6) 1°는 hand-eye 레버 0.37 m로 렌즈 6.4 mm.
(4) `lens W-x error vs flange x` — 이 작업을 시작하게 한 기울기가 offsets 뒤에 0에
가까워야 한다. 결과 `arm_offsets.npz`는 **어디에도 자동 적용되지 않는다** — 적용은
명령 쪽(목표 자세를 보내기 전에 IK 결과에서 δq를 빼거나 목표를 미리 비트는 것)의
별도 작업이고 숫자를 본 뒤 결정한다.

합성 검증: `check_chain_calib.py` §5 — 1.5 mm / 0.4° 잡음에서 심은 J2~J6 오프셋을
0.3° 안으로 복원; §5b — 실제 세션의 관절 구성에서 코너를 렌더링(0.3 px 잡음, 두 태그
뷰 다수)하고 심은 20 mm / 1° hand-eye 오차 + 오프셋을 0.15 mm로 복원(J6은 hand-eye
스핀으로 접힘), 재투영 rms 0.30 px.

### 첫 실제 세션 결과 (2026-09-21, `log/chain_calib/20260921_arm`) — 적용 보류

65뷰(300~309 전부, 스핀 30/90/150°, 거리 0.25~0.56 m, 대부분 0.45), `--exclude v13
v11 v20 v45`(움직이는 중 캡처 / 태그 1개). 재투영 rms: 강체 **7.64 px** → 오프셋만
**3.81**(홀드아웃 3.85; 오프셋 모두 0.6° 미만) → **`--hand-eye-free` 1.26**(홀드아웃
2.11): 지배항은 **T_hc2ee 보정 20.4 mm / 1.29°**, hand_cam 프레임 Δt (+8.3, +3.7,
−18.3) mm, 그 뒤 J2 −1.22±0.31, J3 −0.32±0.24, J4 +0.23±0.14, J5 −0.10±0.05°, 링크는
0. 그러나 **거리 부분집합에서 답이 흔들린다**(>0.42 m 45뷰: Δt (+5.4, −5.0, −11.9)
/ 0.88°; ≤0.42 m 15뷰: (+9.1, −0.1, −19.3) / 1.43°, J4 −0.13 ↔ +0.76°), 잔차 1.2~2.1 px
가 코너 바닥 0.3 px보다 크므로 모델에 빠진 것이 있다 — hand_cam 내부 파라미터(D435
는 D=0을 보고; fx 오차는 한 거리에서 hand-eye z와 축퇴)와 종이 평탄도(이 세션
0.75~1.03°)가 후보. 체인 `solve`에 이 hand-eye를 넣으면 63뷰 세션은 D≈그 역을 요구
한다(PnP 자세 기반이라 같은 잡음의 영향). **아무것도 적용하지 않았다.** 다음 순서는
`docs/HANDOVER.md` §2-0c: hand_cam 내부 파라미터 확인 → 시트 테이핑 → 거리(0.30/
0.55)·기울임(≥15°, 모든 자리) 추가 수집 → 부분집합이 2 mm / 0.2° 안에서 일치할 때
적용.

### 적용 (2026-09-22) — 측정 체인에만, 설정 파일로

위 "적용은 명령 쪽" 설명은 2026-09-22에 바뀌었다. 오프셋은 **측정 체인**에 들어간다:
`apriltag_nav/config/tf/arm_joint_offsets.yaml`(+ npz)에 저장되고, `path_tag_locator`
(locate / map 캘리브레이션)와 `chain_calib.py solve`는 컨트롤러 TCP 대신
`FK_urdf(q + dq)`를 플랜지로 쓴다 (`apriltag_nav.tf_chain.arm_flange_T`; 관절각이 없는
샘플·상태는 그대로 TCP). 명령 쪽(`MoveJ(IK − δq)`)은 여전히 없다.

```bash
# 1. hand-eye 고정, 현재 K (config) 로 피팅 — 전체 뷰
rosrun chain_calib arm_offsets.py log/chain_calib/<세션> $A --hand-intrinsics config --exclude ...
# 2. 적용 (yaml + npz; --disable 로 끄면 모든 소비자가 dq = 0)
python3 src/apriltag_nav/tools/tf_chain_tool.py joint-offsets --apply log/chain_calib/<세션>/arm_offsets.npz --source "..."
# 3. 같은 세션으로 T_ab2mb 를 다시 풀어 짝을 맞춘다 (--arm-offsets 기본 config).
#    적용하는 값은 [planar] — x, y, yaw 만 피팅, roll = pitch = 0, tz = 설계값 (아래 규칙)
rosrun chain_calib chain_calib.py $A solve log/chain_calib/<세션> --hand-intrinsics config --exclude ...
python3 -c "import numpy as np; c=np.load('log/chain_calib/<세션>/corrections.npz'); np.savez('log/chain_calib/<세션>/T_ab2mb_planar.npz', T_ab2mb=c['T_ab2mb_planar'])"
python3 src/apriltag_nav/tools/tf_chain_tool.py set T_ab2mb --npz log/chain_calib/<세션>/T_ab2mb_planar.npz --source "..."
python3 src/apriltag_nav/tools/tf_chain_tool.py urdf       # 플래너 URDF mobile_to_base 줄을 옮겨 적는다
python3 src/apriltag_nav/tools/tf_chain_tool.py check      # 13/13, URDF mobile_to_base 포함
```

**T_ab2mb 는 planar 로 적용한다 (사용자 규칙, 2026-09-22).** 시트 세션이 재는
roll / pitch 는 "그리드 법선 대 팔 z" — 마운트 기울기 + 그날 그 자리에서 섀시가
바닥에 대해 기울어진 각 + 종이 경사 — 이고, map 캘리브레이션은 다른 주차 위치에서
하므로 그 값이 상수에 들어가면 안 된다. `solve` 는 6-DOF `base` / `joint` 와 함께
`planar` (x, y, yaw 만, roll = pitch = 0, tz 는 `tf_chain.yaml` 의 설계값 —
`--planar-tz` 로 바꿀 수 있음) 를 같이 풀어 `corrections.npz` 의 `T_ab2mb_planar`
에 저장한다. planar 의 잔차가 6-DOF 보다 큰 것은 (12.5 vs 7.9 mm, 2026-09-22 팔
세션) 그 세션의 기울기를 흡수하지 않은 결과이며 정상이다.

(hand-eye, dq, T_ab2mb, hand_cam K)는 **한 세트**다 — 하나가 바뀌면 그 오른쪽을 다시
푼다 (CLAUDE.md *Transform Parameters*, Work Log 2026-09-22). 관절각이 기록되지 않은
세션(2026-09-21 11:29 의 63뷰)은 오프셋을 적용할 수 없으므로 `solve` 가 그 샘플 수를
알려주고 컨트롤러 TCP 로 계산한다. 첫 적용값: J2 −0.341, J3 −0.529, J4 −0.051, J5
−0.134, J6 −0.496° (팔 세션 65뷰, 재투영 7.43 → 3.39 px); 같은 세션의 체인 잔차는
오프셋 없이 10.1 mm, 오프셋 + T_ab2mb 재피팅 후 7.9 mm (홀드아웃 12.7 → 7.7).
부분집합 ±2° 불일치는 그대로이므로 "평균값"이다.

## 6. Basler 비전 팁 측정 — `basler_tip_calib.py` (2026-09-18)

hand_cam이 본 점에 Basler를 정확히 갖다 놓으려면 hand_cam↔Basler 관계가
필요한데, 둘 다 플랜지에 붙어 있으니 그 관계는 상수 `inv(D)·T_ee2tip`이고,
D(hand-eye)는 sweep으로 확정되므로 남는 미지수는 **비전 팁**
(`apriltag_nav/config/tf/tf_chain.yaml`의 `T_ee2tip` = 플랜지 기준으로
Basler 프레임 중심이 초점거리에서 닿는 점 3개 + 이미지 회전 1개)뿐이다.
Basler는 16.5 mm 매크로라 기울기를 못 재고 K도 없으므로 hand-eye를 풀지
않고 이 4개만 푼다 — `src/chain_calib/basler_tip.py` 모듈 docstring에 수식.

**시트**: A4에 인쇄한 tag36h11 201–230, 5×6, 20 mm 태그, 간격 20 mm
(피치 40 mm 설계) — `sheet/A4_tag20_201-230_5x6_layout.json`. 설계값이
아니라 **인쇄물을 재서** 넘긴다: `--sx` = (201 왼쪽 바깥 모서리 → 205 오른쪽
바깥 모서리, mm)/180, `--sy` = (201 위 → 226 아래)/220, `--tag-size` =
검은 변 하나 [m].

**절차** (베이스 정지, 시트를 정반 위에 평평하게, 한 세션에):

```bash
rosrun chain_calib basler_tip_calib.py check                   # 양쪽 카메라가 시트를 보는지, Basler 시야(mm)와 px/mm
# ① hand_cam을 시트 위 0.25-0.35 m에 두고(20 mm 태그 ≈ 45-60 px, 30개 전부 보이게), 카메라 축 스핀을 바꿔 가며 4-6회
rosrun chain_calib basler_tip_calib.py --sx .. --sy .. --tag-size .. capture-hand log/chain_calib/basler_tip_<date>
# ② Basler를 태그 하나 위로 조그 → Keyence standoff 16.5 mm(--standoff 16.5 또는 robot_ui Auto standoff) → 캡처.
#    다른 태그로 옮기고/손목을 30-60° 돌려 가며 6-10회 (스핀이 팁의 옆방향 성분을 결정한다)
rosrun chain_calib basler_tip_calib.py capture-basler log/chain_calib/basler_tip_<date> --standoff 16.5
rosrun chain_calib basler_tip_calib.py status log/chain_calib/basler_tip_<date>   # 본 스핀 구간, 샘플별 rms
rosrun chain_calib basler_tip_calib.py solve  log/chain_calib/basler_tip_<date>   # p_tip / psi, 잔차, jackknife, result.yaml
```

**robot_ui에서도 똑같이** (Calibration 탭 → "Basler vision tip" 그룹, web/Qt
동일): 세션 디렉터리(기본 `log/chain_calib/basler_tip_<날짜>`) → Check →
Capture hand ×4–6 → Capture Basler ×6–10("standoff first" 체크 시 Keyence
루프 16.5 mm 먼저) → Status → Solve. 같은 `BaslerTipSession`
(`chain_calib/basler_tip_ros.py`)이 돌므로 결과·문구가 CLI와 같고, 보고서는
그룹 아래 상자와 로그에 남는다. 명령행은 `catkin_make` 후 `rosrun`.

읽는 법: `sheet pose … scatter`가 hand_cam 체인 자체의 정확도(모든 것의
바닥; 스핀 간 불일치가 크면 hand-eye 오차가 드러난 것), `fit … rms`가
Basler 샘플들의 일관성, `jackknife`가 결과의 불확실도. 결과는 자동 반영되지
않는다 — `tf_chain_tool.py set T_ee2tip …`로 `tf_chain.yaml`에 넣고(pose 모드와
`tools/set_tool_tcp.py`가 거기서 읽음), 플래너 URDF `vision_tip_joint`를 **함께** 바꾼다.
오프라인 검증 `scripts/check_basler_tip.py`(11).

### 6-1. 실행 검증 — `verify` (2026-09-21, 팔이 움직인다)

풀린 tip이 맞는지는 "계산한 tip을 태그 위에 갖다 놓고 Basler가 무엇을
보는가"로 확인한다. 세션의 시트 자세(hand 샘플)로 태그 중심을 팔 좌표로
옮기고, **현재 손목 방향을 유지한 채** tip이 그 중심 위 20 mm에 오는
플랜지 포즈로 MoveL 한 번 → Keyence 루프로 16.5 mm(seek이 20 mm를
내려온다) → Basler 한 장 → 이미지 중심 아래 시트 점과 태그 중심의 차이를
mm로 보고한다(89 px/mm). 현재 플랜지에서 0.35 m 넘게 떨어진 목표는
**거부** — 먼저 그 태그 근처로 jog. 결과는 `<dir>/verify.csv`에 한 줄씩.

```bash
rosrun chain_calib basler_tip_calib.py verify log/chain_calib/basler_tip_<date> 215            # 측정 tip
rosrun chain_calib basler_tip_calib.py verify log/chain_calib/basler_tip_<date> 215 --design   # 설계 tip으로 대조
```

robot_ui: 같은 그룹의 `verify tag` 번호 + `design tip` 체크 + **Verify
(moves arm)**. 읽는 법: `|d|` ≤ 3 mm = fit의 정확도 안(OK); 3–6 mm = 이
레버(246 mm)에서 팔 자세 오차(~1°)가 손목 스핀에 따라 만드는 크기 —
같은 태그에서 스핀 0 / ±45 / ±90°로 반복해 보면 스핀 0°에서 작고 ±90°에서
커지는 것이 보인다; > 6 mm = tip이나 시트 자세가 틀린 것. 설계 tip으로
돌리면 y 7 mm / z 16 mm 차이가 그대로 나와야 한다(≈ 620 px 옆, Keyence가
~16 mm 더 내려감).
