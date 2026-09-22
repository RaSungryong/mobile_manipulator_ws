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
- **다음 할 일은 hand-eye 재촬영이 아니라 관절 offset 적용**이다 (§6).

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
| 팔 세션 (65뷰, joints 포함) | `log/chain_calib/20260921_arm/` — `samples.npz`, `corners.json`, `meta.yaml`, `arm_offsets_handeye_free.txt` (09-21 밤, **옛 K**), `T_hc2ee_fit_20260921_arm.npz` (후보, 미적용), `arm_offsets.npz`, `corrections.npz` |
| 체인 세션 (63뷰) | `log/chain_calib/20260921/` — 적용된 T_ab2mb 의 근거 |
| Basler tip 세션 | `log/chain_calib/basler_tip_20260921/` |
| hand-eye 스윕 원본 | `log/path_tag_locator/handeye_calib/run_20260918_144420` (35샘플 refine 결과 `result_sweep123_refined.npz` = 적용값), `run_20260918_152111`, `run_20260921_195136` (20:09, untracked) |
| 기록 문서 | `src/chain_calib/docs/CHAIN_CALIB_2026-09-21_kr.md`, `src/chain_calib/README.md` |
| 도구 | `src/chain_calib/scripts/{arm_offsets,chain_calib,verify_chain,sheet_path,basler_tip_calib}.py`, `path_tag_locator` 의 `handeye_calib_node` (Auto-sample / Compute) |

⚠️ **세션 `meta.yaml` / `session.json` 에는 캡처 당시의 절대 경로가 박혀 있고,
워크스페이스가 2026-09-22 에 `ws_20260902 → ws` 로 옮겨졌다.** 도구들은 이제
없어진 경로를 만나면 `! … no longer exists — using …` 을 찍고 같은 파일의 새
위치(설정된 hand-eye, 패키지의 sheet/ 레이아웃)로 대체한다 (2026-09-22 수정:
`chain_calib.py sheet_from_args`, `basler_tip_ros.py`). 이 경고는 정상이다.

## 6. 다음 순서 (HANDOVER §2-0c "Revised order" 그대로, (d) 는 해소)

1. **관절 offset 을 hand-eye 고정으로 피팅** — `arm_offsets.py … --hand-intrinsics
   config` (hand-eye-free 없이). 09-21 late 값: J2..J5 로 시트 산포 5.7 → 2.7 mm.
   near/far, spin 별 서브셋이 ~0.2° 안에서 일치할 때만 채택.
2. **명령 쪽에 적용** — `arm_controller` 에서 `MoveJ(IK(target) − δq)` 또는 목표
   Cartesian 을 미리 왜곡. 설정값 편집으로는 안 들어간다 (HANDOVER §2-0c 4).
   `arm_node` 재시작.
3. **검증** — `sheet_path.py` 를 두 spin(60°, 92°) 에서: 목표는 그리드 태그 위
   렌즈 xy. 지금 ±15–20 mm → 관절 offset 후 몇 mm 가 목표.
4. hand-eye 재촬영은 **스윕 한 번의 10 mm 재현성을 먼저 고친 뒤에만** 의미가
   있다: 스윕당 range 0.30 + 0.55 m, 모든 위치에서 tilt ≥ 15°, spin 30–150°.
   시트는 테이프로 평평하게 (`capture` 의 paper slope < 0.5°).
5. 새 시트 세션을 찍는다면 hand_cam 은 이미 override 로 rectified 되어 나오므로
   `--hand-intrinsics meta` (기본) 로 풀어야 한다 — `config` 는 D 를 두 번 적용하게
   되어 거부된다.

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

# 체인 세션 재해석 (raw 6.62 mm 가 나와야 정상)
# ⚠️ --sx/--sy/--tag-size 는 전역 옵션: `solve` 앞에 써야 한다
python3 src/chain_calib/scripts/chain_calib.py --sx 1.0 --sy 1.0 --tag-size 0.090 \
  solve log/chain_calib/20260921 --hand-intrinsics config --holdout-every 4

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
- 메인 스택은 새 경로에서 **수동 실행 중** (systemd 서비스 `mobile-manipulator` 는
  만들어졌지만 아직 미설치 — `docs/STOP_LAUNCH_kr.md` §0.5). 캘리브레이션 launch
  (`path_tag_locator.launch`) 는 **떠 있지 않다** — hand-eye 노드를 쓰려면
  `use_handeye_calib:=true` 로 띄울 것.
- 로봇: 도크 500, 충전 중 (BMS 15.7 A, 76 %).
- 작업 트리에 다른 세션의 미커밋 변경 다수 (robot.yaml, arm_node, robot_ui …) —
  `git add -A` 금지, hunk 단위로만 stage.
