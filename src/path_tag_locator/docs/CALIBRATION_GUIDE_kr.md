# 태그 맵 캘리브레이션 사용 가이드 (2026-09-01 리팩터링 기준)

십자태그(참조 태그) 기반으로 바닥 태그들의 월드 좌표를 자동 측정하는
절차입니다. 이 문서가 현장 기준이며, 세부 원리는 영문 `README.md`를,
증상별 진단은 `TROUBLESHOOTING_kr.md`를 참조하세요. (옛 `USAGE_kr.md`는
2026-10-06 삭제 — 아직 유효한 참고 내용은 아래 부록으로 옮김.)

## 구성 요약

- **하드웨어 소유 없음**: 팔 = `arm_node`(`/arm/state` + `/arm/move_cart`),
  베이스 = `mobile_node`(MobileClient), 태그 관측 = `robot_camera_node`의
  `/hand_cam/tag_detections` · `/front_cam/tag_detections`.
- **월드 좌표계 = map.yaml 좌표계**: 원점 = 정반 1 중심, +x 동쪽,
  +y 북쪽, z=0 정반 상면. 바닥 태그는 상면 아래 80 mm.
- **십자태그**: 정반마다 6개(ID 0–5, 90 mm), 각 정반 기하 중심 기준
  ±600/±1200 mm. 정반 2 = 정반 1 + x 3.900 m (2026-09-04 실측 390 cm).
- **세션은 정반별로 분리**: 두 정반의 십자태그 ID가 같으므로
  계획 파일과 참조 파일을 반드시 짝으로 교체.

| 세션 | 계획 | 참조 |
|---|---|---|
| 정반 1 (B+C, 26개) | `calibration_plan_plate1.yaml` | `reference_tags.yaml` |
| 정반 2 (D+E, 25개) | `calibration_plan_plate2.yaml` | `reference_tags_plate2.yaml` |

계획에는 태그별로 참조 태그 배정과 **팔 관측 좌표**(설계값 시드)가
들어 있습니다. 손 카메라가 기준(anchor)이라 카메라는 항상 참조 태그
법선 위 0.5 m에 있고(2026-09-03 오차 예산에 따라 0.8→0.5), 표기된 cam yaw는 플랜지가 팔 베이스에 가장
가깝도록(도달거리 최적) 선택된 자유 회전입니다. 51/51 태그 모두
플랜지 기준 1.4 m 이내(0.67–1.20 m).

## 0. 준비 (1회)

```bash
cd ~/mobile_manipulator_ws && catkin_make && source devel/setup.bash
```

- 참조 태그 실측값이 설계값과 다르면 `reference_tags*.yaml` 수정.
- 리프트를 고정 높이에서 쓸 경우 `map_calibrator.yaml`의
  `lift_height_mm` 설정(세션 시작 시 자동 원점복귀 → 단방향 상승;
  인코더 드리프트 대책). 같은 높이로 생성기 재실행 권장:

```bash
rosrun path_tag_locator generate_calibration_artifacts.py --lift-mm 150
```

- 핸드아이(`apriltag_nav/config/tf/tf_chain.yaml`의 `T_hc2ee` + npz)는 카메라/마운트를 물리적으로 건드리지 않는
  한 재보정 불필요. 재보정 절차(2026-09-14): 베이스를 102/103번에 세우고
  Arm 탭으로 hand_cam이 크로스 태그 0을 보게 한 뒤, robot_ui →
  Calibration 탭 → **Hand-eye** 그룹 → "Auto-sample (sweep)". 노드가
  태그에 정렬한 뒤 태그 주위를 돌며(거리 3단 × 기울기 0/12/22° ×
  방위 4 × 스핀 −30/0/+30, 24뷰) 태그가 보이는 자세마다 촬영하고
  시작 자세로 돌아온다(플랜지·비전 팁이 정반 위 12 cm 이상, 시작점
  반경 30 cm, 도달 1.25 m 이내인 뷰만 실행). 그 뒤 "Compute & save".
  이후 캘리브레이션 노드 재시작 + `generate_calibration_artifacts.py`.
  **카메라를 옮긴 뒤(2026-09-18)** 파일의 hand-eye는 옛 마운트 것이라
  정렬이 발산한다(xy 289→373 mm, 두 번째 스텝에 태그 상실). 이제 sweep이
  이를 감지해 태그가 보이던 자세로 되돌아간 뒤, 그 자리에서 플랜지 축
  ±10° 회전 6자세를 스스로 촬영해 임시 hand-eye를 풀고 그것으로 정렬·
  sweep을 진행한다(`handeye_calib.yaml auto.bootstrap: auto`). 시작
  자세는 태그 위 0.5 m 이상(평소처럼 0.6–1.2 m)이면 된다. 끝나면
  "Compute & save"가 진짜 파일을 쓴다.

## 1. 실행 (순서 고정)

```bash
# ① 메인 스택 — 하드웨어 소유 노드들. 반드시 먼저.
roslaunch apriltag_nav mobile_manipulator.launch

# ② 캘리브레이션 노드 — 메인 스택 옆에서 실행.
roslaunch path_tag_locator path_tag_locator.launch
```

핸드아이 재보정 시에만: `... path_tag_locator.launch use_handeye_calib:=true`

## 2. 세션 실행 (robot_ui 권장)

robot_ui → **Calibration** 탭(권장): 정반 선택 + dry run 체크 +
START/Cancel 버튼, 진행 카운트(ok/fail/degraded)와 마지막 항목이
실시간 표시. 단일 태그 locate 도 같은 탭에서 가능.

스크립트 방식(자동화용) — **Scripts** 탭 → `map_calibration`:

1. 처음엔 `DRY_RUN = True`, `PLATE = 1` 그대로 RUN —
   계획/참조/맵 파싱만, 로봇 무동작. 로그에 `[calib] tag N: dry_run`.
2. 스크립트에서 `DRY_RUN = False`로 바꿔 RUN —
   베이스가 B/C 복도 26개 지점을 자동 순회, 로그에
   `[calib] tag 105: OK x=… y=…` 실시간 표시.
3. `PLATE = 2`로 바꿔 정반 2 세션(D/E 25개) — 계획·참조 파일이
   자동으로 짝 맞춰 전환됨.

CLI 동등 명령:

```bash
rosservice call /map_calibrator/run_calibration "{dry_run: true}"   # 정반1 드라이런
rosservice call /map_calibrator/run_calibration "{}"                # 정반1 실행
# 정반 2: plan_path + ref_tags_path를 반드시 짝으로 전달
rosservice call /map_calibrator/run_calibration \
  "{plan_path: '\$(find path_tag_locator)/config/calibration_plan_plate2.yaml',
    ref_tags_path: '\$(find path_tag_locator)/config/reference_tags_plate2.yaml'}"
```

진행 모니터: `rostopic echo /map_calibrator/progress`

### 중단

```bash
rosservice call /map_calibrator/cancel_calibration   # 현재 항목까지 마치고 정지, 부분 결과 유지
```

긴급 시 STOP ALL / E-stop. **비상정지 래치가 걸리면 이후 항목은
스스로 주행을 거부**하며, 래치는 사람이 의도적으로 해제해야 합니다.
개별 항목 실패(IK/미검출/nav)는 스킵하고 세션은 계속됩니다.
동시 세션은 잠금으로 거부됩니다.

**도달 한계 완충(2026-09-03)**: 정렬 이동이 IK/도달 문제로 실패해도
참조 태그가 현재 자세에서 보이면 항목을 실패시키지 않고 **마지막
도달 가능한 자세에서 체인을 계산**합니다(`align.continue_on_move_failure`,
기본 on). 결과에는 `degraded` 표시가 붙으며(robot_ui 로그 "(DEGRADED)"),
해당 항목은 `verify_map_world.py` 잔차를 확인한 뒤 신뢰하세요 —
체인은 정면 뷰를 요구하지 않지만(6-DOF 관측), 기울어진 뷰는 실제
카메라에서 검출 정확도가 떨어집니다.

## 3. 단일 태그 측위 (디버그/재확인)

Scripts의 `locate_tag`(`TAG_B_ID`/`AUTO_ALIGN` 상수 편집), 또는:

```bash
rosservice call /path_tag_locator/locate_path_tag "{tag_b_id: 105, auto_align: false}"
```

전제: 베이스가 해당 바닥 태그 위 정지, 손 카메라가 십자태그를 봄
(`auto_align: true`면 시드 자세로 이동 후 자동 정렬).

## 4. 결과와 검증

- 출력: `$MM_WS/log/path_tag_locator/map_world_<타임스탬프>.yaml`
  (월드 = map.yaml 좌표계, x/y 직접 비교 가능)
- 호출별 아카이브: `$MM_WS/log/path_tag_locator/locate/`
  (관측 행렬, TCP, 리프트 높이, 스냅샷)
- 오차 예산: `python3 scripts/error_budget.py --entry N` — 모든 오차원을
  경로 태그 **위치 + yaw** 오차로 환산(최적화 설정 기준 sig_xy ≈ 1.1 mm,
  sig_yaw ≈ 0.07° 예상). yaw 는 front-cam 회전이 1:1로 기여(위치와 반대)
- 검증: `rosrun path_tag_locator verify_map_world.py`
  — 상대 기하 5 cm 초과 항목 표시
- **첫 세션 건강 체크**: 바닥 태그 z ≈ **−0.079** 이어야 정상 (바닥은 −0.080, 태그가 1 mm 판이라 윗면은 −0.079; 십자 태그 z도 같은 이유로 +0.001).
  전체가 회전되어 보이면 참조 태그 yaw 가정(0°)부터 의심 →
  `reference_tags.yaml` yaw 수정 후 재실행.

**팔 홈 복귀(2026-09-03)**: 세션의 모든 베이스 이동(피벗/후진 포함)
전에 팔을 자동으로 홈 포즈로 복귀시킨 뒤 주행합니다
(`arm.home_before_nav`, 기본 on; 동기 — 홈 도착 후 출발). 관측 자세로
팔을 뻗은 채 주행하는 충돌 위험 제거. 수동 홈: `rosservice call /arm/move_home`.

## 5. 철칙 3가지

1. **세션 중 TASK / GOTO 금지** — `/mobile/goto_tag` 이중 지휘 충돌.
2. **스캔 중 캘리브레이션 호출 금지** — arm_node가 move_cart를 거부.
3. **계획·참조 파일은 항상 짝으로** — 어긋나면 결과 전체가 3.90 m 이동.
   (robot_ui의 `PLATE` 상수를 쓰면 자동 보장.)

## 첫 실기 권장 순서

실기 세션은 2026-09-02부터 돌고 있습니다(정반 2 첫 세션). 정반 1은
2026-09-22와 09-28에 측정되어 `map.yaml`에 적용됨(태그 100–125의 x, y,
yaw; z는 사용 안 함). 새 체인/새 정반에서 처음 돌릴 때의 권장 순서:

1. 드라이런 (`DRY_RUN = True`)
2. 단일 locate, `auto_align: false`
3. 단일 locate, `auto_align: true` (정렬 수렴 확인)
4. 정반 1 배치 세션 → `verify_map_world.py` + z ≈ −0.079 확인
5. 정반 2 배치 세션

설정/계획을 바꿨으면 손으로 고치지 말고 생성기를 다시 돌리세요:

```bash
rosrun path_tag_locator generate_calibration_artifacts.py [--lift-mm N] [--view-m 0.5]
```

---

## 부록 A. 핸드아이 샘플 재사용 (resume)

캡처 도중 노드를 종료해도 디스크 아카이브
(`$MM_WS/log/path_tag_locator/handeye_calib/run_<ts>/`)는 남습니다.

```bash
roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true
rosservice call /handeye_calib/load_latest "{}"   # 현 세션을 제외한 가장 최신 run_*/ 적재
rosservice call /handeye_calib/status  "{}"       # samples ≥ min_samples 확인
rosservice call /handeye_calib/capture "{}"       # 자세 다양성이 부족하면 추가 캡처 (합쳐짐)
rosservice call /handeye_calib/compute "{}"       # npz + tf_chain.yaml 블록 갱신, 새 run_<ts>/result.{npz,yaml}
rosservice call /handeye_calib/reset   "{}"       # 메모리 비움 + 새 run_<ts>/ (디스크 기록 보존)
```

특정 디렉터리를 합치려면 `config/handeye_calib.yaml`의
`io.load_samples_dirs`(목록)에 `run_<ts>/` 또는 그 아래 `samples/`를
적으면 노드 시작 시 적재됩니다. 적재된 샘플은 새 run에 다시 저장되지
않습니다(중복 방지). 새 `T_hc2ee`는 노드가 기동 시 1회 로드하므로
캘리브레이션 노드 **재시작** 후 반영됩니다.

## 부록 B. `locate_path_tag` 호출의 전체 필드

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: -1,                       # -1 = locator.yaml 의 tag_b_id
  override_ref: false,                # true 면 ref_pose 를 T_A_world 로 사용 (쿼터니언 전부 0 금지, 무회전 = w: 1.0)
  ref_pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}},
  save_result: true,                  # 안내용 — 성공/실패 모두 항상 저장됨
  save_dir: '',                       # '' = locator.yaml io.default_save_dir
  auto_align: true,                   # 시드 자세로 이동 후 자동 정렬 (translation 만 보정, 기울기는 기록)
  align_initial_tcp_mm_deg: [300, 0, 400, 180, 0, 0]   # mm, deg (ZYX)
}"
```

응답: `position_m`, `rpy_deg`(ZYX, deg), `t_a2b_row_major`, 정렬 보고
(`align_iterations_used`, `align_final_xy_offset_m`, `align_final_tilt_deg`,
`align_final_tcp_mm_deg`). 마지막 성공 결과는 latched 토픽
`/path_tag_locator/tag_world_pose`(`geometry_msgs/PoseStamped`)에도 남습니다.
정렬 동작은 `config/locator.yaml`의 `align:` 블록(`max_iterations`,
`position_tol_m` 0.001, `angle_tol_deg` 0.5, `max_step_m` / `max_step_deg`
클램프, `orientation: fixed`)으로 조정합니다.

## 부록 C. 기록 구조

```
$MM_WS/log/path_tag_locator/
  locate/<YYYYMMDD>/run_<ts>_tag<id>[_FAILED]/
      hand_cam.png, front_cam.png       검출기에 들어간 영상
      K_hc.npz, K_fc.npz                호출 시점의 K
      result.npz                        T_B_world, T_A2B, T_A_world, T_hc2ee, T_ab2mb,
                                        T_mb2fc, tcp_pose_mm_deg, position_m, rpy_deg
      result.yaml                       요약 + observations (카메라 좌표계: x=영상 오른쪽,
                                        y=영상 아래, z=광축 거리; camera_frame_note 참조)
                                        + auto_align.tag_in_cam / history
      request.yaml                      요청 echo
  locate/locate_log.csv                 append-only 인덱스 (success 열 0 = 실패)
  calibrate/<YYYYMMDD_HHMMSS>/          세션 1개
      entries/NNN_tag<id>_attempt<n>_<ok|fail>.yaml   시도마다 실행 순서대로 번호 (덮어쓰기 없음)
      session.yaml, entries_log.csv, map_world.yaml   순서 인덱스 / 한 줄 요약 / 결과 사본
  map_world_<ts>.yaml                   세션 결과 (월드 = map.yaml 좌표계)
  handeye_calib/run_<ts>/
      samples/NNNN_image.png, NNNN_pose.npz, samples_index.csv, result.{npz,yaml}
```

`dry_run: true`는 아무것도 기록하지 않습니다. 실패한 entry의 원인은
그 run 디렉터리의 `hand_cam.png` / `front_cam.png`(다른 태그를 본 것은
아닌지)와 `result.yaml`의 에러 메시지로 추적합니다.

## 부록 D. 검증 스크립트

| 스크립트 | 로봇 | 용도 |
|---|---|---|
| `verify_map_world.py` | ❌ | 태그별 요약 + `map.yaml` edge 기반 상대 거리 비교(`--threshold-m`, 기본 5 cm) |
| `test_repeatability.py` | ✓ | `/map_calibrator/run_calibration` 두 번 → 두 결과 diff(반복정밀도) |
| `verify_arm_pointing.py` | ✓ | 월드 좌표에서 관측 자세를 계산해 이동 → hand-cam 재검출 → 잔차(폐루프 검증) |
| `visualize_map_world.py` | RViz | ref(빨강) + 보정 태그(초록) + 원점 축을 `MarkerArray`로 publish |
| `error_budget.py` | ❌ | 오차원별 기여를 경로 태그 위치/yaw 오차로 환산 |
