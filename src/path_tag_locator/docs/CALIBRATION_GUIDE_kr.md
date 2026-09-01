# 태그 맵 캘리브레이션 사용 가이드 (2026-09-01 리팩터링 기준)

십자태그(참조 태그) 기반으로 바닥 태그들의 월드 좌표를 자동 측정하는
절차입니다. 이 문서가 현장 기준이며, 세부 원리는 `USAGE_kr.md`(일부
구버전 내용 포함)와 영문 `README.md`를 참조하세요.

## 구성 요약

- **하드웨어 소유 없음**: 팔 = `arm_node`(`/arm/state` + `/arm/move_cart`),
  베이스 = `mobile_node`(MobileClient), 태그 관측 = `robot_camera_node`의
  `/hand_cam/tag_detections` · `/front_cam/tag_detections`.
- **월드 좌표계 = map.yaml 좌표계**: 원점 = 정반 1 중심, +x 동쪽,
  +y 북쪽, z=0 정반 상면. 바닥 태그는 상면 아래 80 mm.
- **십자태그**: 정반마다 6개(ID 0–5, 90 mm), 각 정반 기하 중심 기준
  ±600/±1200 mm. 정반 2 = 정반 1 + x 3.890 m.
- **세션은 정반별로 분리**: 두 정반의 십자태그 ID가 같으므로
  계획 파일과 참조 파일을 반드시 짝으로 교체.

| 세션 | 계획 | 참조 |
|---|---|---|
| 정반 1 (B+C, 26개) | `calibration_plan_plate1.yaml` | `reference_tags.yaml` |
| 정반 2 (D+E, 25개) | `calibration_plan_plate2.yaml` | `reference_tags_plate2.yaml` |

계획에는 태그별로 참조 태그 배정과 **팔 관측 좌표**(설계값 시드)가
들어 있습니다. 손 카메라가 기준(anchor)이라 카메라는 항상 참조 태그
법선 위 0.8 m에 있고, 표기된 cam yaw는 플랜지가 팔 베이스에 가장
가깝도록(도달거리 최적) 선택된 자유 회전입니다. 51/51 태그 모두
플랜지 기준 1.4 m 이내(0.86–1.32 m).

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

- 핸드아이(`T_hc2ee.npz`)는 카메라/마운트를 물리적으로 건드리지 않는
  한 재보정 불필요.

## 1. 실행 (순서 고정)

```bash
# ① 메인 스택 — 하드웨어 소유 노드들. 반드시 먼저.
roslaunch apriltag_nav mobile_manipulator.launch

# ② 캘리브레이션 노드 — 메인 스택 옆에서 실행.
roslaunch path_tag_locator path_tag_locator.launch
```

핸드아이 재보정 시에만: `... path_tag_locator.launch use_handeye_calib:=true`

## 2. 세션 실행 (robot_ui 권장)

robot_ui → **Scripts** 탭 → `map_calibration`:

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

## 3. 단일 태그 측위 (디버그/재확인)

Scripts의 `locate_tag`(`TAG_B_ID`/`AUTO_ALIGN` 상수 편집), 또는:

```bash
rosservice call /path_tag_locator/locate_path_tag "{tag_b_id: 105, auto_align: false}"
```

전제: 베이스가 해당 바닥 태그 위 정지, 손 카메라가 십자태그를 봄
(`auto_align: true`면 시드 자세로 이동 후 자동 정렬).

## 4. 결과와 검증

- 출력: `~/.ros/path_tag_locator/map_world_<타임스탬프>.yaml`
  (월드 = map.yaml 좌표계, x/y 직접 비교 가능)
- 호출별 아카이브: `~/.ros/path_tag_locator/locate/`
  (관측 행렬, TCP, 리프트 높이, 스냅샷)
- 검증: `rosrun path_tag_locator verify_map_world.py`
  — 상대 기하 5 cm 초과 항목 표시
- **첫 세션 건강 체크**: 바닥 태그 z ≈ **−0.080** 이어야 정상.
  전체가 회전되어 보이면 참조 태그 yaw 가정(0°)부터 의심 →
  `reference_tags.yaml` yaw 수정 후 재실행.

## 5. 철칙 3가지

1. **세션 중 TASK / GOTO 금지** — `/mobile/goto_tag` 이중 지휘 충돌.
2. **스캔 중 캘리브레이션 호출 금지** — arm_node가 move_cart를 거부.
3. **계획·참조 파일은 항상 짝으로** — 어긋나면 결과 전체가 3.89 m 이동.
   (robot_ui의 `PLATE` 상수를 쓰면 자동 보장.)

## 첫 실기 권장 순서

전 과정은 아직 오프라인 검증만 완료된 상태입니다.

1. 드라이런 (`DRY_RUN = True`)
2. 단일 locate, `auto_align: false`
3. 단일 locate, `auto_align: true` (정렬 수렴 확인)
4. 정반 1 배치 세션 → `verify_map_world.py` + z ≈ −0.080 확인
5. 정반 2 배치 세션

설정/계획을 바꿨으면 손으로 고치지 말고 생성기를 다시 돌리세요:

```bash
rosrun path_tag_locator generate_calibration_artifacts.py [--lift-mm N] [--view-m 0.8]
```
