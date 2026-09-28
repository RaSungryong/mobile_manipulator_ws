# hand_cam 내부 파라미터 캘리브레이션 — 따라하기 절차 (2026-09-22)

도구: `src/apriltag_nav/tools/hand_cam_intrinsics.py` (오프라인 검사
`check_hand_cam_intrinsics.py`, 18). 결과는 `robot.yaml`
`robot_camera.intrinsics_override.hand_cam` 에 들어간다.

## 0. 왜, 그리고 raw 프레임 규칙

2026-09-21 의 override 는 A0 시트 코너를 **한 거리(0.45 m), 대체로 화면 중앙**에서
찍은 데이터로 풀었다. 그래서 fx 와 k1, hand-eye z 가 서로 흡수됐고, 관절 offset
피팅에 반경 방향 잔차(중심 −0.7 → 가장자리 −2.4 px)가 남았다. 세 거리 + 두 축
기울기 + 네 구석으로 다시 푼다.

**raw 프레임 규칙.** `robot_camera_node` 는 override 가 켜져 있으면 hand_cam
프레임을 (K, D) 로 리매핑해서 **자기 검출에만** 쓴다. 노드가 내보내는 이미지는
`/hand_cam/tag_overlay` 하나뿐이고, 드라이버의 `/hand_cam/color/image_raw` 는
건드리지 않는다. 캡처 도구는 그 드라이버 토픽만 구독하고(overlay 나 image_raw
가 아닌 토픽은 거부), 시작할 때 `frames come from /hand_cam/color/image_raw =
the DRIVER's raw stream (… override … is ENABLED — … NOT in these frames)` 를
찍는다. 그러니 **override 를 끄지 않아도 되고, 끄면 안 된다** — 끄면
`robot_camera_node` 재시작이 필요하고 얻는 것이 없다.

## 1. 준비물

- 9 × 13 칸 / 15 mm 체커보드 (내부 코너 8 × 12 = 96개). 평판에 붙어 있을 것.
- 강철자 (0.5 mm 눈금).
- 로봇: 메인 스택 동작 중 (`systemctl status mobile-manipulator` = active), robot_ui
  (브라우저 `http://192.168.1.100:8080`) Arm 탭으로 팔을 움직인다. 캘리브레이션
  launch 는 필요 없다.
- 터미널 하나 (로봇 PC 또는 ssh).

## 2. 보드 실측 (2분)

1. 가로: 13칸 전체 길이 L13 을 잰다 (첫 칸의 바깥 모서리 → 마지막 칸의 바깥
   모서리). `a = L13 / 13`.
2. 세로: 9칸 전체 길이 L9. `b = L9 / 9`.
3. `--square-mm` = (a + b) / 2. 예: L13 = 194.8, L9 = 134.9 → a 14.985, b 14.989 →
   **14.99**.
4. a 와 b 가 0.1 mm 이상 다르면 그대로 진행하되 값을 기록해 둔다 (인쇄
   이방성 — 결과의 fx/fy 비율에 나타난다).
5. 판이 평평한지 자를 대각선으로 대 본다. 0.5 mm 이상 뜨면 다른 판.

## 3. 보드 놓기

- 바닥이나 정반 위, 팔이 0.25–0.6 m 위에서 내려다볼 수 있는 곳. hand_cam 이 아래를
  보는 자세(home 근처)에서 시작하면 편하다.
- 조명: 보드 위에 반사(글레어)가 없게. 창·조명이 비치면 위치를 바꾼다.
- 보드는 세션 내내 고정. 뷰는 **팔만** 움직여 바꾼다.

## 4. 캡처 시작

```bash
cd ~/mobile_manipulator_ws && source devel/setup.bash
rosrun apriltag_nav hand_cam_intrinsics.py capture log/chain_calib/hand_cam_intr_$(date +%Y%m%d) \
    --chess 12x8 --square-mm 14.99          # 2절에서 잰 값
```

시작하면 세 줄이 나온다: raw 프레임 안내, `target: checkerboard 12x8 …`,
`stream /hand_cam/color/image_raw 640x480, driver K fx 609.3 …`. 그 뒤 한 줄이
실시간으로 갱신된다:

```
seen 96  range 0.44 m  tilt 21.3  pos TL (u 0.31 v 0.28)  edge  35 px  motion  0.08 px STILL
```

| 항목 | 뜻 |
|---|---|
| `seen 96` | 코너 96개 검출 (`target NOT seen` 이면 보드가 프레임 밖이거나 가려짐/흐림) |
| `range` | 카메라→보드 중심 거리 (드라이버 K 기준, ±2 %) |
| `tilt` | 보드 법선 vs 광축 각도 |
| `pos` | 보드 중심이 있는 화면 영역: TL TR BL BR 구석 / C 중앙 |
| `edge` | 보드의 가장 바깥 코너에서 화면 가장자리까지 픽셀 |
| `motion` | 최근 4프레임의 코너 이동량, 코너 순서(180° 뒤집힘)와 무관. `STILL` (≤ 1.0 px) 일 때만 저장됨 — 0.5–0.8 은 노출/코너 노이즈, 팔이 움직이면 수 px |

## 5. 뷰 찍기 (팔 이동 → 정지 → Enter, 30–50회)

한 뷰의 동작:

1. robot_ui Arm 탭 jog 로 팔을 옮긴다 (`jog step` 20–50 mm, 회전 5–15°).
2. 팔이 멈추면 **1초** 기다린다. 터미널 줄이 `STILL` 로 바뀐다.
3. 터미널에서 **Enter**. `saved v07: …` 와 커버리지 표가 나온다.
   `not saved: the corners moved …` 가 나오면 더 기다렸다 다시 Enter.

찍는 순서 (권장, 총 ~40장):

| 단계 | 거리 | 기울기 | 위치 | 장수 |
|---|---|---|---|---|
| A | 0.45 m | 정면 (< 10°) | 중앙 1, 네 구석 4 (`edge` 20–40 px 까지 밀어 넣기) | 5 |
| B | 0.45 m | 30–45°, 네 방향 (앞·뒤·좌·우로 카메라를 기울임: rx / ry jog ±30°) | 중앙 + 구석 섞어서 | 8 |
| C | 0.35 m | 정면 + 30° 네 방향 | 중앙 2, 구석 4 | 8 |
| D | 0.55–0.60 m | 정면 + 30–40° 네 방향 | 중앙 2, 구석 4 | 8 |
| E | 0.25–0.30 m | 정면 + 20° (가까우니 보드가 화면을 크게 차지) | 중앙 위주 | 5 |
| F | 표가 "still missing" 이라고 하는 것 | | | 나머지 |

- 기울기는 **카메라를 기울이는 것**이다 (툴 rx / ry 를 ±30–45° jog). 보드 중심이
  화면에 남도록 xy 도 같이 옮긴다. 기울인 상태에서 45° 는 `seen` 이 깜빡일 수
  있다 — 40° 정도로.
- 구석: 보드가 화면 **모서리에 걸릴 듯** 놓는 뷰가 왜곡 계수를 결정한다. `edge`
  10 px 미만이면 코너가 잘려 검출이 안 되니 20–40 px 를 노린다.
- 0.25 m 에서는 `range` 가 0.25 아래로 내려가지 않게 (RealSense 컬러 최소 초점
  ~0.2 m).
- 표의 `still missing:` 이 **`nothing — solve`** 가 되면 충분하다. 목표: 거리
  세 구간 각 8+, tilt ≥ 25° 8+, 네 구석 각 3+, 총 30–50.
- 중간에 멈춰야 하면 `q` + Enter. 같은 명령으로 다시 실행하면 v번호가 이어진다.
- `--auto` 를 붙이면 정지 + 새 기하(거리 4 cm / 기울기 6° / 위치 60 px 이상
  다름)일 때 Enter 없이 저장된다. 팔을 옮기고 1초 멈추기만 반복하면 된다.

## 6. 풀이

```bash
rosrun apriltag_nav hand_cam_intrinsics.py solve log/chain_calib/hand_cam_intr_<date>
```

출력을 위에서부터 읽는다:

1. 커버리지 표 (5절과 같은 것).
2. `dropping N outlier view(s)` — 뷰별 rms 가 중앙값의 3배를 넘는 뷰(팔이 움직인
   프레임)는 자동 제외 후 다시 푼다. 3장 넘게 빠지면 촬영 중 흔들림이 잦았던
   것 — `--still-px 0.5` 로 다시 찍는 편이 낫다.
3. `== result ==`: `rms` (정상 0.2–0.3 px), 새 K / D, 그 밑에 `driver:` 와
   `override:` (현재 값) 비교, `distortion at r = 320 px: N px`.
4. `== split checks ==`: 홀/짝 절반과 near/far 절반을 따로 풀어 fx·fy 차이 2 px,
   k1 차이 0.005 이내면 `OK`. 하나라도 `FAIL` 이면 표에서 부족한 뷰를 더 찍고
   다시 `solve`.
5. `VERDICT: ACCEPT` 또는 `NOT yet`.
6. `robot.yaml` 블록 그대로. `result.yaml` 도 세션 디렉터리에 저장된다.

기대값: fx·fy 595–605 부근 (fx ≈ fy), cx·cy 320±5 / 240±5, k1 +0.12…+0.18, k2 −0.2…−0.4
(2026-09-22 적용값 601.87 / 601.93 / 321.35 / 238.51 / +0.177 / −0.348 와 몇 px 이내).

⚠️ near/far 절반 검사는 각 절반이 거리 다양성을 잃어 fx–k1 축퇴로 몇 px 벌어질 수
있다 (09-22: 4.4 px, 홀/짝은 0.9 px). 그때는 3-fold hold-out (CLAUDE.md Work Log
2026-09-22) 로 판단한다 — 이 세션은 그 근거로 `--force` 적용했다.
많이 다르면 `--square-mm` 실측을 다시 확인.

## 7. robot.yaml 에 넣기

```bash
rosrun apriltag_nav hand_cam_intrinsics.py solve log/chain_calib/hand_cam_intr_<date> --apply
```

`--apply` 는 `robot.yaml` 의 `intrinsics_override.hand_cam` 블록에서 **K / D /
note / image_size 줄만** 바꾸고 그 안의 주석은 그대로 둔다. ACCEPT 가 아니면
거부한다 (`--force` 로 강행 가능, 권장하지 않음). 세션 디렉터리와 `solve`
출력을 알려주면 이 단계는 이쪽에서 해도 된다.

그다음 노드 재시작 (그래야 새 K 로 리매핑한다):

```bash
sudo systemctl restart mobile-manipulator      # 메인 스택 전체 (충전 중이면 릴레이가 떨어진다 — CHARGE 재발행)
# 또는 노드 하나만:
rosnode kill /robot_camera_node && rosrun apriltag_nav robot_camera_node.py
# path_tag_locator.launch 가 떠 있으면 그것도 재시작
```

확인:

```bash
rostopic echo -n1 /hand_cam/tag_detections | grep -A4 camera_params   # 새 fx fy cx cy
python3 src/apriltag_nav/tools/check_camera_intrinsics_override.py       # 20
```

## 8. 결과가 바꾸는 것

- 이전 세션(`log/chain_calib/20260921`, `20260921_arm`)은 raw 코너를 저장했으므로
  `--hand-intrinsics config` 로 새 K 로 다시 풀 수 있다:
  `arm_offsets.py log/chain_calib/20260921_arm --sx 1 --sy 1 --tag-size 0.090
  --hand-intrinsics config --exclude v13 v11 v20 v45 --quick` — 반경 방향 잔차가
  사라졌는지가 확인점.
- 새로 찍는 시트 세션은 노드가 이미 리매핑해서 내보내므로 `--hand-intrinsics
  meta` (기본) 로 푼다.
- CLAUDE.md Work Log 에 결과(값, rms, 절반 검사)를 남긴다.

## 9. 문제가 생기면

| 증상 | 원인 / 조치 |
|---|---|
| `target NOT seen` 이 계속 | 보드가 프레임 밖/일부 잘림, 글레어, 초점(0.2 m 미만). 팔을 올려 전체가 보이게 |
| `motion` 이 1.0 px 위에서 흔들림 | 팔 진동 또는 자동노출 깜빡임. 2초 더 기다리거나 `--still-px 1.5` |
| 뷰별 rms 한 장만 1 px 이상 | 그 뷰가 움직인 프레임. 자동 제외되며, `--exclude vNN` 로도 가능 |
| 절반 검사 FAIL, rms 는 정상 | 커버리지 부족 (대개 거리 구간이나 구석). 표대로 추가 |
| rms 0.5 px 이상 전체 | 보드가 평평하지 않거나 `--square-mm` 이 틀림 |
| `wait_for_message … timeout` | 드라이버가 안 떠 있음: `rostopic hz /hand_cam/color/image_raw` |
