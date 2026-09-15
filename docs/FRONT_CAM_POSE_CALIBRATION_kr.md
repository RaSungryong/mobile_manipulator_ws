# front_cam 자세 캘리브레이션 (T_mb2fc 6-DOF) — 절차

2026-09-15. 대상: 로봇 앞에서 직접 진행하는 운영자. 9월 8일 태그 쌍 세션을
도구(`tools/calib_front_cam_pose.py`)로 자동화한 것으로, **모든 값은 정지
상태의 태그 또는 프레임 단위의 태그 관측에서만** 나오고 명령한 이동량·각도는
쓰지 않는다 (베이스는 작은 이동을 ~45 %, 피벗을 ~25 % 덜 수행하고 주행 중
스스로 요잉한다).

| 값 | 어디서 | 정밀도 (0.3 px 노이즈, 90 mm 태그 합성 검증, 8 시드) |
|---|---|---|
| roll, pitch, 렌즈 높이 | 정지 스냅샷의 태그 쌍 피팅 | 0.02°, 0.5 mm |
| tx, ty (회전중심 → 렌즈) | 제자리 피벗 9장의 공동 최소제곱 (호의 중심) | tx 0.7 mm rms (최대 1.3), ty 0.5 mm (최대 0.9) |
| yaw (광축 회전, 주행축 기준) | 태그가 보이는 상태로 직진 (한 장만 보여도 사용) | 0.05° rms, 최대 0.08° (트랙 8개 × ~0.15 m) |

**범퍼 가림 (2026-09-15, 사용자):** front_cam 화면의 왼쪽 1/3(약 430 px까지)은
로봇 몸체와 범퍼에 가려 태그가 안 보인다. 도구는 그 영역을 뺀 나머지를
"사용 가능 화면"으로 보고 여유를 계산하며(`--left-edge-px`, 기본 430),
쌍은 **사용 가능 화면의 가운데** — 렌즈 나디르보다 약 7 cm 앞 — 에 둔다.
직진 트랙은 태그가 한 장만 보여도 프레임을 쓰므로(태그 한 장의 변 방향 +
피팅된 배치 각도로 쌍의 자세를 복원) 트랙이 ~0.15 m까지 나온다.

결과의 mobile-base 프레임 **원점은 베이스의 회전중심**이다. 그것이 섀시의
기하 중심과 같은지는 카메라로는 알 수 없다 — §5.

**태그 두께 (2026-09-15, 사용자):** 모든 태그는 두께 1 mm 판이다
(`robot.yaml robot.tag_thickness: 0.001`). 피팅이 주는 렌즈 높이
`height_m`은 **태그 윗면**까지의 높이다(코너가 놓인 평면). 바닥(= mb 원점)
까지는 그보다 1 mm 더 높고, `extrinsics.yaml`의 `T_mb2fc` tz는 생성기가
`height_m + tag_thickness`로 쓴다(로더가 검사). 내비게이션은 렌즈→태그면
거리만 쓰므로 바뀌는 것이 없다. 이 절차에서 신경 쓸 것은 없다 — 값은
그대로 `height_m`에 들어간다. 정반의 십자 태그도 같은 1 mm 판이라
`reference_tags*.yaml`의 z는 +0.001(정반면 위 윗면)이다.

## 1. 준비물

- 같은 규격으로 인쇄한 AprilTag 2장. **2026-09-15 현재 보유: 90 mm, ID
  147·148·149·150** (60 mm 15·16은 없음). 쌍으로는 **149 → 150**을 쓴다
  (도구 기본값). 바닥에 평평하게 고정(테이프).
- **중심 간격 120 mm 권장** (변 사이 빈틈 30 mm). 90 mm 태그는 0.30 m 높이
  에서 화면 1280 px 중 271 px씩 차지하므로, 간격 150 mm면 쌍이 화면 폭
  0.42 m의 0.24 m를 채워 직진 트랙이 0.07 m밖에 못 된다; 120 mm면 트랙
  ~0.09 m. 간격이 곧 스케일 기준이니 0.1 mm 목표로 잰다. **재는 법: 두
  태그 바깥 변 사이의 전체 길이 E와 안쪽 변 사이 빈틈 g를 자로 재서
  간격 = (E + g) / 2** — 인쇄 크기가 정확히 90이 아니어도 무관하다. 이
  값을 `solve --spacing`에 넣는다(기본값 없음, 필수).
- 변을 평행하게 맞출 필요 없다 (2026-09-15부터 피팅이 태그별 회전을
  같이 푼다 — 90° 돌려 깔아도 된다). 중심 간격만 정확하면 된다.
- 배치 방향: **149 → 150이 로봇의 전진 방향** (화면에서 149가 왼쪽,
  150이 오른쪽). 쌍이 **사용 가능 화면(범퍼 오른쪽)의 가운데** 오도록
  베이스를 세운다 — overlay에서 두 태그 offset 평균이 약 +70 mm (앞쪽),
  또는 `check`의 `from the usable middle` 값이 ±30 mm 안. `collect`가
  피벗 전과 직진 전에 스스로 다시 가운데로 맞춘다.
- 나머지 147·148은 §6 기계적 교차검증에 쓴다.
- 스택 기동 상태 (`mobile_manipulator.launch`), **`path_tag_locator.launch`는
  내려둔다** (베이스의 두 번째 명령자). e-stop에 손.
- ⚠️ 147–150은 map.yaml zone E의 태그 ID이기도 하다. 도구는 수동 이동만
  쓰므로 문제없지만, 세션 중 `/robot_pose`는 zone E 좌표를 가리키고
  `mobile_node`의 last-known tag가 149가 된다 — 세션 후 어차피
  `mobile_node`를 재시작한다(§4).

## 2. robot_camera_node를 RAW 검출로 전환

피팅은 원본 픽셀이 필요하다. 보정된 검출은 도구가 거부한다.

```bash
# robot.yaml: robot_camera.ground_plane.front_cam.enabled: false 로 편집한 뒤
rosnode kill /robot_camera_node
rosrun apriltag_nav robot_camera_node.py      # launch의 ~driver_* 파라미터는 마스터에 남아 있어 그대로 동작
rosrun apriltag_nav calib_front_cam_pose.py check
```

`check`가 `OK: raw detections, both tags, CameraInfo present`와 **프레임
여유**(`room with BOTH tags` / `room with ONE tag`, 그리고 `collect can
drive N m tracks`)를 출력해야 한다. 두 태그 여유가 앞뒤 0.03 m 미만이거나
좌우 0.05 m 미만이면 FAIL — 쌍을 사용 가능 화면 가운데로 다시 세우거나
간격을 줄인다.

## 3. 데이터 수집 (자동, 약 5분)

```bash
rosrun apriltag_nav calib_front_cam_pose.py collect log/apriltag_nav/calib_pair_$(date +%Y%m%d)
```

계획을 출력하고 `go` 입력을 기다린다. 이후 순서 (전부 `mobile_node`의
`/mobile/move_cmd`, 오도메트리 폐루프, 0.03 m/s). **모든 이동·직진은 그
순간의 프레임 여유에 맞춰 자동으로 잘린다** — 태그가 화면 밖으로 나가게
명령하지 않는다 (`capped … by the frame room` 로그):

1. 정지 스냅샷 1 + 소폭 이동 8회(±0.02–0.04 m), 이동마다 1.5 s 정지 후
   30프레임(1 s) 평균 스냅샷 → `scan_*.txt`
2. 쌍을 사용 가능 화면 가운데로 되돌린 뒤 **폐루프 피벗** 8회: 태그로 잰
   실제 회전이 첫 스냅샷 기준 누적 +3, +6, +3, 0, −3, −6, −3, 0°(±0.5°)가
   될 때까지 명령·측정·게인 학습을 반복한다 (베이스는 2° 명령을 0.3~0.8°만,
   5° 명령을 ~3.7° 수행한다 — 2026-09-15 실측). 매 명령은 그 순간의 가로
   여유로 잘린다. 각각 뒤 스냅샷 → `piv_NN_*.txt` (9장).
3. 다시 가운데로 되돌린 뒤 직진 0.15 m(태그 한 장이 남는 여유만큼) 전진/
   후진 × 4, 주행 중 전 프레임 기록 → `drive_*.txt` (8 트랙)

중간에 이동이 실패하면 거기서 멈춘다 — 있는 데이터로 `solve`를 돌릴 수
있다. 한 단계만 다시 하려면 `collect DIR --only piv` (또는 `scan`,
`drive`): 기존 디렉터리에 그 단계 파일만 새로 쓰고 이전 것은 `old_*`로
이름을 바꿔 `solve`에서 제외한다. 수동으로도 가능: `snap DIR scan_01` / `snap DIR piv_01`, 직진은
robot_ui Mobile 탭으로 몰면서 `record DIR 01 --seconds 15`.

## 4. 풀이와 반영

```bash
rosrun apriltag_nav calib_front_cam_pose.py --spacing 0.120 solve log/apriltag_nav/calib_pair_<date>
```

(`--spacing`은 §1에서 잰 값. 태그 ID·크기가 기본값과 다르면 `--tags 147
148 --size 0.09`처럼 같이 준다.) 출력을 읽는 법:

- `rms` 0.3 px 근처, `level camera` rms가 그보다 확실히 커야 틸트가 실제로
  관측된 것. `tags' in-plane angle`은 깔린 각도(90의 배수 + 비뚤어진
  정도) — 정보용. `fitted tag size … off the nominal` 경고가 뜨면
  `--spacing`을 잘못 쟀거나 잘못 넣은 것.
- 회전중심: 피벗 쌍별 (x, y) mm 표와 **전체 피벗 스냅샷 공동 피팅값**
  (`joint fit … spread N deg, rms`). 공동 피팅 rms가 1 mm를 넘거나 쌍별
  sd가 3 mm를 넘으면 바닥이 미끄럽거나 피벗이 안 됐던 것 — 다시.
- yaw: **트랙별 값은 개별로 ±0.5°까지 흔들리는 게 정상**(0.09 m 트랙
  하나의 노이즈); 평균이 답이고 8 트랙 평균은 ±0.1° 안이다. **전진
  트랙과 후진 트랙이 계통적으로 다르면 주행 중 옆으로 미끄러진 것**이라
  yaw로 믿으면 안 된다 (1 mm/0.1 m 미끄럼 = 0.57°).

납득되면:

```bash
rosrun apriltag_nav calib_front_cam_pose.py --spacing 0.120 solve log/apriltag_nav/calib_pair_<date> --apply
```

`--apply`는 `robot.yaml`의 `camera_offset`, `camera_lateral`,
`ground_plane.front_cam.{roll_deg,pitch_deg,height_m,yaw_deg}`만 바꾸고
`make_front_cam_extrinsics.py --apply`로 `extrinsics.yaml`의 `T_mb2fc`를
재생성한다 (tz = height_m + tag_thickness). 그 다음:

1. `robot.yaml` `ground_plane.front_cam.enabled: true`로 되돌리기
2. `robot_camera_node`, `mobile_node`(camera_offset), 캘리브레이션 노드
   (extrinsics) 재시작
3. `python3 src/path_tag_locator/scripts/check_front_cam_extrinsics.py` 23/23

⚠️ `camera_lateral`(ty)은 현재 `T_mb2fc`에만 들어간다. `mobile_controller`는
렌즈가 중심선 위에 있다고 가정하므로, |ty|가 5 mm를 넘게 나오면 그 차이는
`/robot_pose`의 lateral과 정지 위치에 그대로 남는다 — 그때는 컨트롤러에도
넣어야 한다 (별도 작업).

## 5. 회전중심 = 섀시 기하 중심인가 — 바닥 표시 테스트

카메라가 주는 tx는 **회전중심**까지다. `T_ab2mb`(arm 마운트 y −100 mm)와
셀 설계의 정지 자세는 **섀시 기하 중심** 기준이다. 둘이 다르면 그 차이만큼
팔의 절대 위치가 어긋난다. 측정은 기계적으로 한다.

준비: 다림추(또는 레이저 포인터) 2개, 줄자, 바닥 마스킹테이프, 펜.

1. 베이스를 세우고 **앞 범퍼 중앙**과 **뒤 범퍼 중앙**의 바닥 투영점을
   다림추로 찍어 표시한다 (F0, R0). 두 점의 중점이 기하 중심 G, 두 점을 잇는
   선이 섀시 축.
2. 그 상태에서 `snap`을 한 장 찍는다 (또는 overlay에서 태그 149의 offset을
   기록) — 렌즈 나디르와 태그의 관계.
3. robot_ui Mobile 탭으로 **+90° 피벗** (한 번에 큰 각이 좋다). 다시 앞·뒤
   범퍼 중앙을 찍는다 (F1, R1).
4. 회전중심 C = 선분 F0F1의 수직이등분선과 R0R1의 수직이등분선의 교점
   (90°면 교점이 명확). C와 G의 거리·방향이 답이다. 1 mm 자로 읽힌다.
5. 같은 자리에서 90°씩 3번 더 돌려 4점을 얻으면 C의 산포도 나온다.

렌즈 → 기하 중심 거리는 별도로: 태그 149 중심을 overlay `offset: (0, 0)`에
맞춰 렌즈 나디르 바로 아래 두고, 그 태그 중심에서 F0까지 줄자로 잰 값 +
(F0–R0 길이의 절반)이 tx_geometric. 이것과 `solve`의 tx(회전중심 기준)의
차이는 위 4의 C–G 거리와 같아야 한다 — 두 방법이 맞으면 측정을 믿어도 된다.

**결론에 따라 할 일:** C ≈ G (2–3 mm 이내)면 지금 프레임 정의 그대로.
차이가 크면 어느 쪽을 mb 원점으로 삼을지 정해야 한다 — 내비게이션은 C가
자연스럽고(정지·얼라인·aim이 전부 C 기준), 팔 체인은 G 기준으로 측정돼
있으니 `T_ab2mb`의 t를 C 기준으로 옮기는 편이 코드 변경이 없다.

## 6. 기계적 교차검증 (선택)

yaw는 주행으로만 잰다. 주행이 의심스러우면: 나머지 태그 2장(147·148)을
섀시 옆면과 평행하게 (양 끝에서 옆면까지 거리를 자로 같게) 0.5 m 이상
떨어뜨려 깔고 — 두 장이 한 프레임(0.42 × 0.24 m)에 같이 안 들어가므로
베이스를 옆면과 평행하게 둔 채 `snap` 두 장(각 태그 하나씩) 또는
overlay에서 각 태그의 `offset`을 읽는다 — 두 태그 중심을 잇는 선의
각도(offset 차이의 atan)가 섀시축 기준 yaw다. 0.5 mm/0.5 m = 0.06°.
주행 yaw와 0.2° 이상 다르면 바퀴 정렬(주행축 ≠ 섀시축)을 의심한다.
