# Ra 학습 데이터 수집 절차 (수집 모드, 2026-10-06)

표면조도(Ra) 추정 모델의 학습 데이터 = **실제 스캔 TASK 경로에서 찍은 Basler
이미지 + 같은 자리를 조도측정기로 잰 Ra**. 이 문서는 그 수집을 위해 만든
"수집 모드(collect mode)"의 사용법이다. 설계와 검증 기록은 CLAUDE.md Work Log
2026-10-06.

## 1. 동작

수집 모드가 켜져 있으면 TASK 스캔의 **촬영 포인트마다** 다음이 반복된다.

1. 경로대로 이동 → Keyence 거리 보정 → Basler 촬영 (평소와 같음)
2. 공구가 자기 z축 방향으로 **80 mm 후퇴** (`collect.retreat_mm`)
3. `/arm/collect_state` = waiting, UI의 SCAN 칩이 `WAIT Ra`로 바뀜
4. 작업자: vision tip 바로 아래(방금 촬영한 자리)를 조도측정기로 측정
5. UI Task 탭 → "Ra reading(s)"에 값 입력 (여러 번 잰 값은 띄어쓰기로, 평균이
   기록됨) → **Record & next** (Enter 키도 됨). 잴 수 없으면 **Skip point**
6. 행이 `log/apriltag_nav/ra_measured/<run>_ra_measured.csv`에 추가되고, 공구가
   촬영 자세로 복귀한 뒤 다음 경로 행으로 진행

transition 행, 이동 실패 포인트, 프레임이 없는 포인트는 멈추지 않는다.
대기 중 **Cancel arm motion / STOP ALL**은 스캔을 끝내고 공구는 후퇴한 채로
둔다(복귀 이동 없음).

## 2. 준비

- **거리 보정이 수렴하는지 먼저 확인.** 09-30 run은 271점 전부
  `out of range on the far side`였다. Basler 초점은 16.5 mm 표준거리에 맞춰져
  있어 범위 밖에서 찍은 이미지는 학습에 못 쓴다. 실제 금형 위에서 Arm 탭의
  Auto standoff로 표면 높이를 확인하고 경로 z를 맞출 것. 수집 run은
  `robot.yaml keyence.require_converged: true`로 돌리면 미수렴 포인트가
  `success`로 기록되지 않는다 (해당 포인트는 촬영 생략 → 수집도 생략).
- 포인트 서브셋 결정: 전 경로(1266점)를 점당 ~1분이면 20시간. `TASK_DEFS`의
  `groups` 필터나 경로 파일 축약으로 수집할 포인트를 고른다.
- 측정기 조건 고정: cutoff λc, 측정 길이, 측정 방향(연마 결 대비), 반복 횟수.
  Note 칸이나 별도 메모에 남긴다.
- 조명/카메라 조건은 평소 스캔 그대로(노출 4 ms, gain 0, VISION 램프).
- 디스크: 프레임 1장 20 MB. `num_samples`를 올리면 그만큼 늘어난다.

## 3. 실행

```
sudo systemctl restart mobile-manipulator      # arm_node / web UI가 새 코드를 읽도록 (최초 1회)
```

1. 브라우저(로봇 PC 또는 LAN의 폰/태블릿)에서 `http://192.168.1.100:8080` → Task 탭
2. "Ra data collection" 그룹의 **Collect mode** 체크 (arm_node 상태가 돌아와
   체크가 유지되면 켜진 것; `rostopic echo /arm/collect_state`로도 확인)
3. 평소대로 `Send TASK <이름>`
4. SCAN 칩이 `WAIT Ra pt N`이 되면 측정 → 입력 → Record & next
5. 끝나면 Collect mode 체크 해제 (켜둔 채 다음 TASK를 보내면 또 멈춘다)

`robot.yaml collect.enabled: true`로 두면 부팅부터 켜진다(권장하지 않음 —
운영 스캔이 모두 멈춘다). `wait_timeout_s`를 0보다 크게 두면 그 시간 안에
입력이 없을 때 skip으로 기록하고 진행한다.

## 4. 결과물과 병합

| 파일 | 내용 |
|---|---|
| `log/apriltag_nav/ra_maps/<run>.csv` | Ra map: 포인트별 x y z, 모델 Ra, 거리 보정 결과 |
| `log/apriltag_nav/ra_measured/<run>_ra_measured.csv` | 수기 Ra: run, index, group_id, point_id, images, ra_measured, ra_readings, note, skipped, standoff, x y z, measured_at |
| `results/scan_images/<run>/g<group>_p<point>_i<index>_s<n>.png` | 프레임. 이름만으로 Ra map 행과 수기 행에 1:1 대응 |

```
python3 src/apriltag_nav/tools/merge_ra_dataset.py            # 수기 CSV가 있는 모든 run
python3 src/apriltag_nav/tools/merge_ra_dataset.py <run stem>  # 한 run
```

→ `results/ra_dataset/<run>_dataset.csv`, **프레임 1장당 1행**: image(절대경로),
ra_measured, ra_readings, note, model_ra_mean, standoff_ok, x y z, flags.
기본은 수기 Ra가 있는 프레임만; `--all-frames`는 미측정 프레임도 남긴다
(`flags: unmeasured`); `--out one.csv`는 여러 run을 한 표로. 실행 요약에
"frames unmeasured / measurements without a frame / standoff NOT ok" 수가
찍히니 수집 직후 확인한다. `standoff_not_ok` 행은 초점이 맞지 않은
이미지이므로 학습에서 제외할 것.

2026-10-06 이전의 프레임(`point_<id>_sample_<n>_ra_<x>.png`)은 group이 없어
point_id로만 짝지어지고, 같은 id가 여러 그룹에 있으면 `ambiguous`로 표시된다.

## 5. 토픽 (스크립트/외부 PC용)

- `/arm/collect_mode` (std_msgs/Bool): 켜기/끄기, 다음 촬영 포인트부터 적용
- `/arm/scan_continue` (std_msgs/String, JSON): `{"ra": 0.41}` /
  `{"readings": [0.40, 0.42], "note": "..."}` / `{"skip": true}`
- `/arm/collect_state` (latched JSON): enabled, waiting, index/total, group_id,
  point_id, images, image_dir, record_csv, n_recorded, n_skipped, last
- `/arm/scan_progress`에 `wait` / `resume` 이벤트 추가
