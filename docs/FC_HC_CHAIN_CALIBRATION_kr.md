# front_cam ↔ hand_cam 체인 캘리브레이션 (두 태그) — 절차

2026-09-15. 대상: 로봇 앞에서 직접 진행하는 운영자. 도구는
`src/path_tag_locator/scripts/calib_fc_hc_chain.py`, 수학은
`path_tag_locator/chain_calib.py`, 합성 검증은 `scripts/check_chain_calib.py`.

## 0. 무엇을 재는가

locator 체인은 hand_cam이 본 태그 A에서 front_cam이 본 태그 B까지를
네 개의 고정 행렬로 잇는다.

```
T_A2B = inv(T_hc2A) · T_hc2ee · T_ee2ab · T_ab2mb · T_mb2fc · T_fc2B
                      └────────── T_hc2fc (모델) ──────────┘
```

이 안의 `T_hc2ee`(hand-eye), `T_ab2mb`(팔 마운트), `T_mb2fc`(front_cam)는
각각의 "이상값"이 실제와 같은지 확인할 방법이 없었다. 두 태그를 **같은
바닥에 자로 잰 간격으로** 깔면 참값 `T_A2B`를 알게 되고, 베이스를 세워둔 채
팔만 여러 자세로 움직여 hand_cam으로 A를 보면 자세마다 식 하나가 생긴다.

```
inv(T_hc2ee) · S_i = D · (T_ee2ab_i · T_ab2mb · T_mb2fc) · F
```

- `D` = **hand 쪽** 상수 보정 (ee 프레임): hand-eye가 틀렸을 때 남는 것
- `F` = **base 쪽** 상수 보정 (fc 프레임): 팔 마운트·front_cam·앞 태그
  관측 — 베이스가 안 움직이니 이 셋은 구분되지 않고 한 덩어리
- 팔 자세 하나로는 D와 F를 못 가른다. 자세들이 **두 축의 기울임 + 스핀**을
  섞어 회전 다양성을 가지면 갈린다 (D는 변하는 `T_ee2ab_i` 앞, F는 뒤).
  9월 11일 "hand-eye인지 base인지 모르겠다"가 바로 이 조건이 없어서였다.

`solve`는 raw / hand만 / base만 / 둘 다, 네 가지 피팅의 잔차를 나란히
보여준다. **hand만으로 노이즈 바닥에 닿으면 hand-eye 문제, base만으로
닿으면 마운트 쪽, 둘 다여야 닿으면 양쪽, 둘이 비슷하면 회전 다양성 부족.**

정밀도 (합성, 20프레임 평균, 18~24 자세): D·F 각각 1~2 mm / 0.1~0.2°.
자세당 잔차 바닥은 2~3 mm / 0.3° — hand_cam이 0.45 m에서 보는 120 px
태그의 기울기 노이즈(프레임당 0.7°, 20프레임 평균 0.15°)에 A→B 레버
0.7 m가 곱해진 값이다. 그래서 프레임 평균이 필수다.

## 1. 태그 깔기

- 태그 B(기본 **150**)는 front_cam 화면의 사용 가능 영역 가운데 (범퍼
  오른쪽, 렌즈 앞 약 7 cm — 앞 캘리브레이션과 같은 자리).
- 바닥에 **직선자(또는 팽팽한 실)**를 B의 한 변에 대고 놓는다. 그 직선자를
  따라 팔이 닿는 자리까지 밀어가 태그 A(기본 **149**)를 **같은 변이 직선자에
  닿게, 인쇄 방향도 같게** 깐다. 두 태그의 변이 한 직선 위에 있으면 된다.
  둘 다 1 mm 판이라 같은 평면이다.
- A의 위치: 팔이 카메라를 그 위 0.35~0.55 m에 두고 ±22° 기울일 수 있는
  곳. 팔 베이스는 섀시 중심에서 y −100 mm(벽 쪽), 바닥은 베이스 아래
  0.65 m, 도달 1.25 m. 보통 로봇 **오른쪽 옆 0.5~0.7 m** (B 기준 0.5~0.9 m)
  가 편하다. 직선자가 그 방향을 향하도록 B의 어느 변을 쓸지 고른다.
- **간격**: 바깥 변 사이 전체 길이 E와 안쪽 빈틈 g를 자로 재서
  `spacing = (E + g) / 2`. 1 mm 안으로.
- 어느 방향으로 깔았든, 90° 단위 회전은 도구가 raw 체인에 가장 가까운
  조합으로 스스로 맞춘다 (체인 오차가 45°보다 훨씬 작으니 안전).

## 2. 준비

- `mobile_manipulator.launch` 기동 (arm_node, robot_camera_node, lifter).
  **`path_tag_locator.launch`는 내려둔다** (팔의 두 번째 명령자).
- 리프트는 원점. 세션 중 베이스는 절대 움직이지 않는다 (앞 태그 관측이
  상수라는 가정).
- robot_ui Arm 탭으로 hand_cam을 태그 A 위 0.4~0.5 m, 대략 정면으로
  가져간다 (hand-eye 스윕 때와 같은 시작 자세).

```bash
rosrun path_tag_locator calib_fc_hc_chain.py --spacing 0.700 check
```

hand_cam이 149를, front_cam이 150을 보고 `raw chain error at this pose:
N mm / M deg`가 찍히면 준비 끝. 그 숫자가 지금 체인의 오차(첫 자세 기준)다.

## 3. 수집 (자동, 약 3분)

```bash
rosrun path_tag_locator calib_fc_hc_chain.py --spacing 0.700 collect log/path_tag_locator/chain_calib/$(date +%Y%m%d)
```

`go` 입력 후: 태그 A 위에서 정면 정렬 → 거리 0.40/0.50 m × 기울기
0/12/22° × 방위 4 × 스핀 0/±90/180° 격자를 20개로 추린 자세를 가까운
순서로 순회 (hand-eye 스윕과 같은 계획기·안전 규칙: 도구 끝이 바닥 위
0.12 m 이상, 플랜지는 시작점 0.30 m 안, 직선 MoveL 0.2 m/30° 단위) →
자세마다 hand_cam 20프레임 + front_cam 20프레임 평균, TCP, 리프트 높이
기록 → 시작 자세로 복귀. 태그를 못 본 자세는 건너뛴다. e-stop에 손.

옵션: `--max-samples 24`, `--distances 0.4 0.5`, `--tilts 0 12 22`,
`--frames 20`, `--dry-run`.

## 4. 풀이

```bash
rosrun path_tag_locator calib_fc_hc_chain.py solve log/path_tag_locator/chain_calib/<date>
```

읽는 법:

| 줄 | 의미 |
|---|---|
| `raw` | 지금 설정 그대로의 체인 잔차. 이게 곧 "현재 T_fc2hc 오차" |
| `hand` | hand-eye만 고쳤을 때 남는 잔차와 보정 D, 보정된 `T_hc2ee` |
| `base` | base 쪽만 고쳤을 때. 보정 F를 `T_ab2mb`에 접은 값 (`T_mb2fc`는 생성 파일이라 그대로 두고 마운트 행렬에 넣음) |
| `joint` | 둘 다. 한쪽만으로 안 될 때만 의미 |
| `verdict` | 위 규칙으로 고른 귀속 |
| `jackknife sd` | 샘플 하나씩 빼고 다시 풀었을 때의 흔들림 = 불확실도 |

**반영은 자동으로 하지 않는다.**

- hand 쪽이면 `solve … --write-hand-eye hand` 로
  `config/hand_eye/T_hc2ee_chain_<date>.npz`를 쓰고, `locator.yaml`
  `hand_eye.npz_path`를 그 파일로 바꾼 뒤 캘리브레이션 노드를 재시작한다.
  (9월 14일 스윕으로 얻은 현재 hand-eye는 태그 재투영 2.1 mm, 절대 1/0/5 mm
  였으니 D가 몇 mm를 넘으면 그쪽을 의심할 근거가 된다.)
- base 쪽이면 출력된 `corrected T_ab2mb`를 `extrinsics.yaml`에 넣는 것이
  체인에는 맞지만, **같은 행렬이 `robot.yaml arm_calibration`(pose 모드
  IK)에도 있다.** 둘을 같이 바꿔야 하고 `arm_transform`의 부호 규약 확인이
  먼저다 — 별도 작업. F의 회전 성분은 "팔 마운트에 틸트가 없다"는 가정
  (교체 베이스에서 재확인된 적 없음)의 검증값이기도 하다.
- `corrections.npz`(D, F, 참값)는 세션 디렉터리에 항상 남는다.

## 5. 결과가 이상할 때

- raw 잔차가 자세마다 크게 다르고 스핀에 따라 규칙적으로 돌면 hand 쪽,
  자세와 무관하게 일정하면 base 쪽 — `per-sample residual` 표로 보인다.
- `UNDETERMINED`: 기울인 자세가 안전 규칙에 다 걸려 정면 자세만 남은 것.
  카메라를 더 높이거나(0.5 m) A를 팔 베이스에 더 가까이 깐다.
- 잔차 바닥이 5 mm를 넘으면 hand_cam이 흔들렸거나(정지 0.3 s 뒤 촬영)
  태그가 들떠 있는 것. `--frames 30`으로 늘려도 안 줄면 태그를 확인.
