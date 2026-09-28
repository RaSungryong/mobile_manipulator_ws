# Hand-eye 캘리브레이션 피팅 결과 — 현재 상태 (2026-09-22 10:30 기준)

다른 세션(창)이 이어받기 위한 문서. **읽는 순서:** 이 문서 → `docs/HANDOVER.md`
§2-0c (배경, 영어) → CLAUDE.md Work Log 2026-09-18 ~ 09-21 항목. 숫자는 전부
2026-09-22 오전에 새 워크스페이스 경로에서 **다시 돌려서 재현한 값**이다
(§7의 명령 그대로).

## 0. 한 줄 요약

- **적용된 hand-eye(`T_hc2ee`)는 2026-09-18 스윕(35 샘플) 값이고, 정확도 ~5 mm.**
  이보다 나은 후보는 없다. 시트(A0) 세션의 잔차 9.5 / 16 mm는 hand-eye를 어느
  후보로 바꿔도 1 mm도 안 움직인다 — 그 오차는 **팔(FK, 관절 zero offset)의
  자세 의존 오차**다.
- 2026-09-21 밤에 "hand-eye 20 mm 보정" 으로 보였던 항은 hand_cam 광축(z)
  방향인데, 그 세션의 뷰(정면, 한 거리)로는 **관측 불가능**한 축이다 — fit
  artefact. hand_cam 내부 파라미터(K, D)가 틀렸던 것은 찾아서 적용했고, 그래도
  이 결론은 그대로다.
- ~~**다음 할 일은 hand-eye 재촬영이 아니라 관절 offset 적용**이다 (§6).~~
  **2026-09-22 오후 갱신: 관절 offset 은 피팅했고 적용하지 않는다** (§6 참조).
  offset 이 렌즈 위치를 바꾸는 양은 시트 전체에서 ±1.5 mm 뿐이고, `sheet_path`
  의 ±13 mm 는 hand_cam 2–3 태그 PnP 의 측정 오차가 대부분이다. 다음은 측정을
  고치는 것 (≥ 4 태그 뷰, hand_cam 내부 파라미터, 인쇄물 y 스케일 실측).
- **2026-09-22 저녁 갱신: 사용자 결정으로 관절 offset 을 APPLIED** ("없는 것보다는
  낫잖아") — 체커보드 K 로 hand-eye 재해석(5.4 mm) → hand-eye 고정 offset 피팅(J2
  −0.34 / J3 −0.53 / J4 −0.05 / J5 −0.13 / J6 −0.50°) → 팔 세션으로 T_ab2mb 재피팅,
  세 파일 + `config/tf/arm_joint_offsets.yaml` 이 한 세트. 측정 체인(locator)에만
  들어가고 명령 쪽은 그대로. 팔 세션 체인 홀드아웃 12.7 → 7.7 mm. 자세한 것은
  CLAUDE.md Work Log 2026-09-22 "C and E re-solved". 아래 §6 의 "적용하지 않음" 은
  그 시점의 결정이다.

## 1. 지금 적용되어 있는 값 (`src/apriltag_nav/config/tf/tf_chain.yaml`)

| 변환 | 값 | 출처 | 상태 |
|---|---|---|---|
| `T_hc2ee` | t (35.63, −334.50, −151.54) mm, rpy (0.144, −0.449, −179.443)° | 09-18 스윕 35 샘플 (`run_20260918_144420` + 15:10 스윕), PARK + tag-scatter refine | **적용, 커밋됨 (HEAD)** |
| `T_ab2mb` | t (−7.47, −123.68, −628.44) mm, rpy (0.320, 0.786, 178.573)° | 09-21 chain_calib 63뷰, **위 09-18 hand-eye로** 피팅 | 적용, 커밋됨 |
| hand_cam K/D | fx 601.72 fy 603.87 cx 322.02 cy 238.46, k1 +0.1598 k2 −0.3222 (드라이버: 609.3 / 608.6 / 321.5 / 238.7, D = 0) | 09-21 late, 두 시트 세션의 corners로 `calibrateCamera` | 적용 (`robot.yaml robot_camera.intrinsics_override.hand_cam`), 커밋 `002f40f`. **실행 중인 `robot_camera_node`가 이 값으로 remap 중** (10:24 재시작, 로그 `intrinsics OVERRIDE` 확인) |
| `T_ee2tip` | (−1.8, −245.6, 209.6) mm | 09-21 Basler tip 세션 | 적용 |

`tools/tf_chain_tool.py check` 12/12 ok (2026-09-22). **09-21 20:09에 운영자가
Compute & save 로 쓴 18-샘플 스윕 값 t (43.70, −328.51, −152.65) 은 현재
파일에 없다** — working tree 의 tf 파일이 HEAD 와 동일 (다른 세션이 되돌린
것으로 보임). 그 결과는 `log/path_tag_locator/handeye_calib/run_20260921_195136/
result.npz` (untracked) 에만 남아 있다. 따라서 HANDOVER §2-0c 의 "(d) 체인
불일치 17.8 mm" 문제는 **해소된 상태**다 (아래 §3 raw 6.62 mm 가 그 증거).

## 2. hand-eye 후보들과 점수

"점수" = 그 hand-eye 를 꽂고 시트 세션의 체인(front_cam ↔ hand_cam) 잔차를 봤을
때. 09-21 late 항목의 수치.

| 후보 | t (mm) | 09-18 파일과의 거리 | 체인 세션 산포 (63뷰) | 팔 세션 산포 (65뷰) |
|---|---|---|---|---|
| **09-18 스윕 35샘플 (적용)** | (35.6, −334.5, −151.5) | — | 9.5 mm rms | 16.1 mm |
| 09-18 스윕, 새 K + undistort 로 재해석 | Δ (+1.2, +2.4, −4.0) | 4.8 mm / 0.22° | 거의 동일 | 거의 동일 |
| 09-21 20:09 18샘플 스윕 (`run_20260921_195136`) | (43.7, −328.5, −152.7) | 10.1 mm | 8.9 mm | 17.1 mm |
| 팔 세션 `--hand-eye-free` 피팅 (`T_hc2ee_fit_20260921_arm.npz`) | (44.3, −330.8, −168.1) | 19.1 mm / 1.30° (Δ hc frame +8.6, +3.7, **−16.6**) | 체인 판정이 HAND 로 뒤집힘 (= 역보정을 요구) | 1.06 px |
| 체인 세션 hand-only 보정 | (30.6, −334.9, −150.1) | 5.1 mm | 5.75 mm | — |

읽는 법: 후보를 바꿔도 시트 산포가 9.5 ↔ 8.9, 16.1 ↔ 17.1 로 **동률**. 단일
스윕끼리는 ~10 mm 차이나고 (09-18 스윕 3 vs 18샘플 파일 12.6 mm, 20:09 vs 35샘플
10.1 mm) 스윕 내부 산포는 3 mm — **스윕 한 번의 정직한 xy 불확도는 5–10 mm**.
팔 세션 피팅의 −16.6 mm z 는 far-view 절반만 쓰면 z = −266 mm 로 튀는 (0.43 px)
비관측 축의 값이므로 **후보가 아니다**.

## 3. 시트 세션 피팅 결과 (2026-09-22 재현, 새 K, 09-18 hand-eye)

### 3a. 팔 세션 `log/chain_calib/20260921_arm` (65뷰, v13 v11 v20 v45 제외, hold-out 매 4번째)

| 모델 | reproj rms (px) | hold-out (px) |
|---|---|---|
| rigid (시트 자세만 자유, offset 0) | 7.63 | 7.90 |
| + 관절 offset J2..J5 (hand-eye 고정) | 3.55 (09-21 late 기록) | — |
| + hand-eye 6-DOF 자유 | **1.06** | 1.99 |

hand-eye 자유일 때 offset: J2 −1.160, J3 −0.366, J4 +0.246, J5 −0.101° (J1은
베이스 yaw와, J6은 hand-eye spin과 겹쳐 고정). 잔차 1.06 px vs 코너 바닥 0.3 px
— 아직 모델에 빠진 항이 있다 (near/far 서브셋이 J4 −0.13 ↔ +0.76° 로 갈림).
FK(URDF) vs 컨트롤러 TCP: 0.00 mm / 0.001° — FK 자체는 정확.

hand-eye 를 **09-18 파일에 고정**하고 J2..J5 만 피팅하면 시트 산포 5.7 → 2.7 mm
(09-21 late) — 이것이 적용 가치가 있는 방향.

### 3b. 체인 세션 `log/chain_calib/20260921` (63뷰, 새 K)

| fit | rms mm / deg | hold-out |
|---|---|---|
| raw (현재 체인) | **6.62 / 0.578** | 6.55 / 0.625 |
| hand (hand-eye 만 보정) | 5.75 / 0.404 | 5.84 |
| base (마운트만) | 5.83 / 0.397 | 5.79 |
| joint (둘 다) | 4.40 / 0.348 | 3.90 |

판정 BOTH, 바닥 4.4 mm 는 뷰별 노이즈. raw 6.6 mm 가 "09-18 hand-eye + 적용된
T_ab2mb 가 서로 일관" 의 기준값 — 20:09 값을 꽂으면 17.8 mm 가 됐었다.

## 4. 이 결론이 나온 근거 (재검토 시 볼 것)

1. `cv2.calibrateCamera` 를 두 세션 corners(평면 타깃) 에 돌리면 K 자유 + k1 k2
   에서 0.26–0.28 px (드라이버 K, D = 0 은 0.7–0.8 px). 서브셋 전부 fx 601–604 로
   일치. k3 p1 p2 는 수렴 안 함 — k1 k2 만.
2. 올바른 K 로 팔 세션 재피팅: rigid 7.4 → offsets-only 3.55 → hand-eye-free 1.06
   px 인데 hand-eye 보정량은 그대로 19 mm (전부 z). **내부 파라미터는 빠진 항이
   아니었다.**
3. far-view 앞쪽 절반만으로 hand-eye z 가 −266 mm 로 피팅됨 (0.43 px) → z 비관측.
4. 저장된 모든 사진에서 hand-eye 를 다시 풀어도 (09-18 스윕 재검출, 시트 세션
   multi-tag PnP) 시트 산포는 후보 간 < 1 mm 차이.
5. `sheet_path.py` 실측: 손목 spin 92° vs 60° 에서 점별 오차 패턴이 완전히 바뀜
   (rms 차 17.7 mm) → 렌즈 위치가 아니라 **팔 자세의 함수** → FK 급 오차.

## 5. 데이터 / 파일 위치 (새 경로 `~/mobile_manipulator_ws`)

| 무엇 | 어디 |
|---|---|
| 적용 변환 + 주석(설계값, 측정법) | `src/apriltag_nav/config/tf/tf_chain.yaml`, `*.npz` |
| 팔 세션 (65뷰, joints 포함) | `log/chain_calib/20260921_arm/` — `samples.npz`, `corners.json`, `meta.yaml`, `arm_offsets.npz` (적용값), `corrections.npz`, `T_ab2mb_20260922_offsets.npz` (적용값). hand-eye-free 후보 npz 와 그 보고서 txt 들은 2026-09-22 저녁 삭제 (채택 안 함; 수치는 이 문서와 Work Log 에) |
| 체인 세션 (63뷰) | ~~`log/chain_calib/20260921/`~~ **2026-09-22 저녁 삭제** (관절각 없음 → 오프셋 적용 불가; git 에 있음). 적용된 T_ab2mb 의 근거는 이제 팔 세션 |
| Basler tip 세션 | `log/chain_calib/basler_tip_20260921/` |
| hand-eye 스윕 원본 | `log/path_tag_locator/handeye_calib/run_20260918_144420` (35샘플, 새 K 재해석 `result_sweep123_refined_K20260922.npz` = 적용값). `run_20260918_152111` (0 샘플), `run_20260921_195136` (20:09, 미적용), 옛 K 의 refine npz 두 개는 2026-09-22 저녁 삭제 |
| 기록 문서 | `src/chain_calib/README.md`; `CHAIN_CALIB_2026-09-21_kr.md` 는 세션과 함께 삭제 (CLAUDE.md Work Log 2026-09-21 에 요지) |
| 도구 | `src/chain_calib/scripts/{arm_offsets,chain_calib,verify_chain,sheet_path,basler_tip_calib}.py`, `path_tag_locator` 의 `handeye_calib_node` (Auto-sample / Compute) |

⚠️ **세션 `meta.yaml` / `session.json` 에는 캡처 당시의 절대 경로가 박혀 있고,
워크스페이스가 2026-09-22 에 `ws_20260902 → ws` 로 옮겨졌다.** 도구들은 이제
없어진 경로를 만나면 `! … no longer exists — using …` 을 찍고 같은 파일의 새
위치(설정된 hand-eye, 패키지의 sheet/ 레이아웃)로 대체한다 (2026-09-22 수정:
`chain_calib.py sheet_from_args`, `basler_tip_ros.py`). 이 경고는 정상이다.

## 6. 관절 offset 피팅 결과 (2026-09-22 오후) — 적용하지 않음

전체 보고서 ~~`log/chain_calib/20260921_arm/arm_offsets_20260922_handeye_fixed_newK.txt`~~ (2026-09-22 저녁 삭제; 아래 표가 요약)
(hand-eye 고정, 새 K, v13 v11 v20 v45 제외, hold-out 매 4번째). 근거는 CLAUDE.md
Work Log 2026-09-22 "Joint offsets fitted" 항목.

| | rigid | offsets |
|---|---|---|
| 전체 61뷰 reproj rms | 7.63 px | 3.55 px |
| hold-out | 7.90 px | 3.59 px |
| J2 / J3 / J4 / J5 / J6 (jackknife sd) | — | −0.11 (0.54) / −0.66 (0.44) / −0.16 (0.35) / −0.15 (0.08) / −0.52 (0.09)° |
| near (≤ 0.42 m, 15뷰) J2 / J3 | — | −3.48 / +1.13° |
| far (46뷰) J2 / J3 | — | +0.53 / −1.07° |

1. **서브셋이 ±2° 로 갈린다** (near/far, spin 30/90/150, tilt/flat 모두). 기하
   자체는 식별 가능 (심은 offset 을 모든 서브셋에서 0.03° 로 복원, 한 서브셋의
   offset 이 다른 서브셋 잔차를 7.6 → 4.3 px 로 줄임) — 값이 흔들리는 것은
   모델 오차 위에서 J2/J3/시트자세가 서로 바꿔치기하기 때문. hand-eye xy, tz,
   focal, k1, 태그별 시트 보정을 각각 풀어 봐도 (rms 2.6–3.1 px) 일치하지 않는다.
2. **결정적 검증:** `sheet_path` 세 런의 각 점을 URDF IK 로 풀어 offset 이
   예측하는 렌즈 오차를 계산 → 점별 변동 **1.2–1.6 mm rms** (측정은 12.7–13.2 mm,
   상관 −0.35…+0.16). offset 은 로봇에서 본 패턴을 설명하지 못한다.
3. **±15–20 mm 의 정체 = hand_cam PnP.** 같은 세션에서 모델 대비 PnP 렌즈 xy:
   2태그 9.8 mm, 3태그 15.4, 4태그 8.3, 5태그 6.8, 6태그 9.4 (rigid); offset 후
   7.2 / 13.8 / 5.5 / 3.9 / 1.8. 0.5 px 코너 노이즈 몬테카를로만으로 2태그 5.2 mm,
   4–6태그 2.2–3.1 mm — 나머지는 3 px 계통 잔차가 2태그 평면 모호성으로 증폭된
   것. `sheet_path` 17:00 런의 x 오차는 시트 열(x 850 / 1000)에 따라 ±8…19 mm 로
   교대한다 — 인쇄물/측정 프레임 서명이지 관절이 아니다. **팔 + rigid 체인은 ≥ 4
   태그 뷰에서 ~8 mm xy.**
4. 남은 계통 잔차 (3.5 px vs 바닥 0.3): 반경 방향 −0.7 → −2.4 px (r 350) —
   hand_cam K 가 가장자리에서 아직 ~0.7 % 틀림; 태그별 보정은 y 만 −0.7…−1 %
   (309 에서 −5.8 mm; 자로 잰 0.35 % 와 다르다 — 300↔308 을 다시 재라).

**다음 순서 (수정):**

1. 측정을 먼저 고친다 — `sheet_path` / `verify_chain` 을 0.55 m (4–6 태그) 에서
   돌리거나 ≥ 4 태그 뷰만 채점; hand_cam 내부 파라미터를 가장자리까지 덮는
   0.25–0.6 m 세트로 다시; 인쇄물 y 스케일을 자로 실측; 시트를 테이프로 고정.
2. 그 뒤에도 ≥ 4 태그 뷰에서 몇 mm 이상 자세 의존 오차가 남으면 그때 관절
   offset (그때는 range 0.30 + 0.55 m, tilt ≥ 15°, spin 30–150° 로 다시 촬영).
3. `MoveJ(IK − δq)` 명령측 변경은 만들지 않는다 (지금 값으로는 1.5 mm 효과).
4. hand-eye 재촬영 조건은 그대로 (스윕 재현성 10 mm 를 먼저).
5. 새 시트 세션은 `--hand-intrinsics meta` (기본) 로 (override 로 이미 rectified).

## 7. 명령 (새 경로, 그대로 실행 가능)

```bash
cd ~/mobile_manipulator_ws && source devel/setup.bash

# 팔 세션: 관절 offset, hand-eye 고정 (다음 단계 1)
python3 src/chain_calib/scripts/arm_offsets.py log/chain_calib/20260921_arm \
  --sx 1.0 --sy 1.0 --tag-size 0.090 --hand-intrinsics config \
  --exclude v13 v11 v20 v45 --holdout-every 4            # --quick: jackknife 생략
# 같은 것 + hand-eye 6-DOF 자유 (비교용; 1.06 px, 19 mm z — 채택 X)
python3 src/chain_calib/scripts/arm_offsets.py log/chain_calib/20260921_arm \
  --sx 1.0 --sy 1.0 --tag-size 0.090 --hand-intrinsics config \
  --exclude v13 v11 v20 v45 --holdout-every 4 --hand-eye-free --quick

# 체인 재해석은 이제 팔 세션으로 (63뷰 세션은 삭제됨). 오프셋은 --arm-offsets config 가 기본
# ⚠️ --sx/--sy/--tag-size 는 전역 옵션: `solve` 앞에 써야 한다
python3 src/chain_calib/scripts/chain_calib.py --sx 1.0 --sy 1.0 --tag-size 0.090 \
  solve log/chain_calib/20260921_arm --hand-intrinsics config --exclude v13 v11 v20 v45 --holdout-every 4

# 적용 상태 확인
python3 src/apriltag_nav/tools/tf_chain_tool.py check     # 12/12
python3 src/apriltag_nav/tools/tf_chain_tool.py show

# 오프라인 검사
python3 src/chain_calib/scripts/check_chain_calib.py      # 48
python3 src/chain_calib/scripts/check_basler_tip.py       # 11
```

⚠️ `arm_offsets.py` / `chain_calib.py solve` 는 세션 디렉터리에 `arm_offsets.npz` /
`corrections.npz` 를 **덮어쓴다** (git 추적 파일). 비교 실험은 세션을 복사해서
돌리고, 원본은 채택할 때만 갱신.

## 8. 로봇 / 환경 상태 (2026-09-22 10:30)

- 워크스페이스 `~/mobile_manipulator_ws` (이름 변경, 전체 재빌드 완료).
- 메인 스택은 **systemd 서비스 `mobile-manipulator` 로 부팅 시 자동 실행** (10:37
  부터, 9/9 노드; `docs/STOP_LAUNCH_kr.md` §0.5). 서비스 재기동/재부팅은 충전
  릴레이를 떨어뜨리므로 (11:00 BMS −4.4 A, 방전 중) 필요하면 `CHARGE` 재발행. 캘리브레이션 launch
  (`path_tag_locator.launch`) 는 **떠 있지 않다** — hand-eye 노드를 쓰려면
  `use_handeye_calib:=true` 로 띄울 것.
- 로봇: 도크 500, 충전 중 (BMS 15.7 A, 76 %).
- 작업 트리에 다른 세션의 미커밋 변경 다수 (robot.yaml, arm_node, robot_ui …) —
  `git add -A` 금지, hunk 단위로만 stage.
