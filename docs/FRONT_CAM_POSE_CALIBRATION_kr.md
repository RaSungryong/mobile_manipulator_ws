# front_cam 자세 캘리브레이션 (T_mb2fc 6-DOF) — 절차

2026-09-15. 대상: 로봇 앞에서 직접 진행하는 운영자. 9월 8일 태그 쌍 세션을
도구(`tools/calib_front_cam_pose.py`)로 자동화한 것으로, **모든 값은 정지
상태의 태그 또는 프레임 단위의 태그 관측에서만** 나오고 명령한 이동량·각도는
쓰지 않는다 (베이스는 작은 이동을 ~45 %, 피벗을 ~25 % 덜 수행하고 주행 중
스스로 요잉한다).

| 값 | 어디서 | 정밀도 (0.3 px 노이즈, 합성 검증) |
|---|---|---|
| roll, pitch, 렌즈 높이 | 정지 스냅샷의 태그 쌍 피팅 | 0.02°, 0.5 mm |
| tx, ty (회전중심 → 렌즈) | 제자리 피벗 때 렌즈가 그리는 호의 중심 | 1 mm |
| yaw (광축 회전, 주행축 기준) | 태그 쌍이 보이는 상태로 직진 | 0.05–0.08° (트랙 6개) |

결과의 mobile-base 프레임 **원점은 베이스의 회전중심**이다. 그것이 섀시의
기하 중심과 같은지는 카메라로는 알 수 없다 — §5.

## 1. 준비물

- 같은 규격으로 인쇄한 AprilTag 2장 (9월 8일: 60 mm, ID 15·16). 중심 간격
  **150.0 mm**, 변이 평행하게, 바닥에 평평하게 고정 (테이프). 간격 정확도가
  곧 스케일 정확도다 (0.1 mm 목표).
- 배치 방향: **15 → 16이 로봇의 전진 방향**. 쌍이 화면 가운데 오도록 베이스를
  세운다 (`/front_cam/tag_overlay`에서 두 태그 offset이 ±50 mm 안).
- 스택 기동 상태 (`mobile_manipulator.launch`), **`path_tag_locator.launch`는
  내려둔다** (베이스의 두 번째 명령자). e-stop에 손.

## 2. robot_camera_node를 RAW 검출로 전환

피팅은 원본 픽셀이 필요하다. 보정된 검출은 도구가 거부한다.

```bash
# robot.yaml: robot_camera.ground_plane.front_cam.enabled: false 로 편집한 뒤
rosnode kill /robot_camera_node
rosrun apriltag_nav robot_camera_node.py      # launch의 ~driver_* 파라미터는 마스터에 남아 있어 그대로 동작
rosrun apriltag_nav calib_front_cam_pose.py check
```

`check`가 `OK: raw detections, both tags, CameraInfo present`와 프레임 여백
(> 150 px)을 출력해야 한다.

## 3. 데이터 수집 (자동, 약 5분)

```bash
rosrun apriltag_nav calib_front_cam_pose.py collect log/apriltag_nav/calib_pair_$(date +%Y%m%d)
```

계획을 출력하고 `go` 입력을 기다린다. 이후 순서 (전부 `mobile_node`의
`/mobile/move_cmd`, 오도메트리 폐루프, 0.03 m/s):

1. 정지 스냅샷 1 + 소폭 이동 8회(±0.02–0.04 m), 이동마다 1.5 s 정지 후
   15프레임 평균 스냅샷 → `scan_*.txt`
2. 제자리 피벗 6회(±4°), 각각 뒤 스냅샷 → `piv_*.txt`
3. 직진 0.12 m 전진/후진 × 3, 주행 중 전 프레임 기록 → `drive_*.txt`

중간에 이동이 실패하면 거기서 멈춘다 — 있는 데이터로 `solve`를 돌릴 수
있다. 수동으로도 가능: `snap DIR scan_01` / `snap DIR piv_01`, 직진은
robot_ui Mobile 탭으로 몰면서 `record DIR 01 --seconds 15`.

## 4. 풀이와 반영

```bash
rosrun apriltag_nav calib_front_cam_pose.py solve log/apriltag_nav/calib_pair_<date>
```

출력을 읽는 법:

- `rms` 0.3 px 근처, `level camera` rms가 그보다 확실히 커야 틸트가 실제로
  관측된 것.
- 회전중심 표: 피벗 쌍마다 (x, y) mm. **sd가 3 mm를 넘으면** 바닥이 미끄럽거나
  피벗이 너무 작았던 것 — 다시.
- yaw: 트랙별 값의 퍼짐이 불확실도다. ±0.1° 안이면 정상. **한쪽 방향
  트랙만 계통적으로 다르면 주행 중 옆으로 미끄러진 것**이라 yaw로 믿으면 안
  된다 (1 mm/0.1 m 미끄럼 = 0.57°).

납득되면:

```bash
rosrun apriltag_nav calib_front_cam_pose.py solve log/apriltag_nav/calib_pair_<date> --apply
```

`--apply`는 `robot.yaml`의 `camera_offset`, `camera_lateral`,
`ground_plane.front_cam.{roll_deg,pitch_deg,height_m,yaw_deg}`만 바꾸고
`make_front_cam_extrinsics.py --apply`로 `extrinsics.yaml`의 `T_mb2fc`를
재생성한다. 그 다음:

1. `robot.yaml` `ground_plane.front_cam.enabled: true`로 되돌리기
2. `robot_camera_node`, `mobile_node`(camera_offset), 캘리브레이션 노드
   (extrinsics) 재시작
3. `python3 src/path_tag_locator/scripts/check_front_cam_extrinsics.py` 22/22

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
2. 그 상태에서 `snap`을 한 장 찍는다 (또는 overlay에서 태그 15의 offset을
   기록) — 렌즈 나디르와 태그의 관계.
3. robot_ui Mobile 탭으로 **+90° 피벗** (한 번에 큰 각이 좋다). 다시 앞·뒤
   범퍼 중앙을 찍는다 (F1, R1).
4. 회전중심 C = 선분 F0F1의 수직이등분선과 R0R1의 수직이등분선의 교점
   (90°면 교점이 명확). C와 G의 거리·방향이 답이다. 1 mm 자로 읽힌다.
5. 같은 자리에서 90°씩 3번 더 돌려 4점을 얻으면 C의 산포도 나온다.

렌즈 → 기하 중심 거리는 별도로: 태그 15 중심을 overlay `offset: (0, 0)`에
맞춰 렌즈 나디르 바로 아래 두고, 그 태그 중심에서 F0까지 줄자로 잰 값 +
(F0–R0 길이의 절반)이 tx_geometric. 이것과 `solve`의 tx(회전중심 기준)의
차이는 위 4의 C–G 거리와 같아야 한다 — 두 방법이 맞으면 측정을 믿어도 된다.

**결론에 따라 할 일:** C ≈ G (2–3 mm 이내)면 지금 프레임 정의 그대로.
차이가 크면 어느 쪽을 mb 원점으로 삼을지 정해야 한다 — 내비게이션은 C가
자연스럽고(정지·얼라인·aim이 전부 C 기준), 팔 체인은 G 기준으로 측정돼
있으니 `T_ab2mb`의 t를 C 기준으로 옮기는 편이 코드 변경이 없다.

## 6. 기계적 교차검증 (선택)

yaw는 주행으로만 잰다. 주행이 의심스러우면: 태그 2장을 섀시 옆면과 평행하게
(양 끝에서 옆면까지 거리를 자로 같게) 0.5 m 이상 떨어뜨려 깔고 `snap` 한 장
— `solve`가 아니라 overlay의 두 태그 중심을 잇는 선의 각도(`degree`)가
섀시축 기준 yaw다. 0.5 mm/0.5 m = 0.06°. 주행 yaw와 0.2° 이상 다르면 바퀴
정렬(주행축 ≠ 섀시축)을 의심한다.
