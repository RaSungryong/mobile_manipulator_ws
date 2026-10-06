# Ra 학습 데이터 수집 절차 (수집 모드, 2026-10-06)

표면조도(Ra) 추정 모델의 학습 데이터 = **실제 스캔 TASK 경로에서 찍은 Basler
이미지 + 같은 자리를 조도측정기로 잰 Ra**. 이 문서는 그 수집을 위해 만든
"수집 모드(collect mode)"의 사용법이다. 설계와 검증 기록은 CLAUDE.md Work Log
2026-10-06.

## 1. 두 가지 방법 (UI Task 탭 "Ra data collection"의 Mode에서 선택)

측정이 병목이다(암 점당 ~3초, 측정 자리 잡기 포함 점당 20~30초). 두 방법 모두
측정기 여러 대를 동시에 쓰기 위한 것이다.

### 방법 A — `pause`: N점마다 정지, 그 자리에서 Ra 입력

1. 경로대로 이동 → Keyence 거리 보정 → Basler 촬영 (평소와 같음)을 N점 반복
   (`points per stop`, 기본 1; 측정기 2대면 2 또는 4)
   - **mark spot before retreat** (`collect.premark`)를 켜면 촬영 직후, 공구가
     아직 표준거리(케이스 바닥이 표면에서 16.5 mm)에 있을 때 점마다 한 번 더
     멈춘다(SCAN 칩 `MARK SPOT`). 케이스 옆에 점을 찍거나 케이스 윤곽을 그린다
     — 촬영 지점은 케이스 중심 바로 아래 — 그리고 **Next (spot marked)**.
     tip은 가상의 점이라 80 mm 위의 공구 아래에서는 못 찾으므로, A 방법에서는
     이것을 켜는 것이 기본이다 (점당 ~5초 추가).
2. N점째(또는 마지막 점) 촬영 후 공구가 자기 z축으로 **80 mm 후퇴**하고 대기
   (`collect.retreat_mm`). SCAN 칩이 `WAIT Ra N pt`
3. UI에 N행 표가 뜬다. 마지막 행(파란색)은 vision tip 바로 아래, 나머지 행은
   tip에서 얼마나 떨어진 자리인지 **world dx, dy (mm; 정반 중심 좌표계)** 와
   **arm-base dx, dy (mm; 암 base_link 축)** 로 표시된다. 작업점 간격은 보통
   25 mm라 눈대중으로 찾는다 (±5 mm). joint 전용 task(pose 짝 없음)는 x y z가
   없어 거리를 못 보여준다.
4. 두 사람이 각자 자기 행의 자리를 재고 값 입력 (여러 번 잰 값은 띄어쓰기,
   평균이 기록됨). 못 잰 행은 skip 체크. Enter로 다음 행, 마지막 행에서
   Enter 또는 **Record & next**. **Skip all**은 전부 미측정으로.
5. 행이 `results/scan_images/<run>/<run>_ra_measured.csv`(그 run의 사진
   폴더 안)에 점별로 추가되고, 공구가 촬영 자세로 복귀한 뒤 다음 경로 행으로 진행

### 방법 B — `mark`: 번호만 적고, 측정은 나중에 따로

1. **촬영 없이** 경로를 돈다: 이동 → Keyence 거리 보정 → 공구 **30 mm 후퇴**
   (`collect.mark_retreat_mm`) → 정지. SCAN 칩이 `MARK #n`
2. UI가 `#n = g<group> p<point>`를 보여준다. 작업자는 tip 바로 아래 지점 **옆**
   (카메라 시야 몇 mm 밖, 지점 위에는 쓰지 말 것)에 펜으로 **n**을 쓰고
   **Next (number written)**. `mark dwell [s]`를 0보다 크게 두면 그 시간 뒤
   자동으로 넘어간다
3. run이 끝나면 `results/scan_images/<run>/<run>_mark_template.csv`가
   생긴다 (mark run은 사진이 없지만 폴더는 같은 규칙으로 만든다): mark_no → group_id / point_id / x y z, `ra_measured` 빈칸
4. 측정기 몇 대든, 순서 상관없이 전부 재고 템플릿의 `ra_measured`(필요하면
   `ra_readings`, `note`)를 엑셀/LibreOffice로 채워 **같은 이름으로** 저장
5. 같은 TASK를 수집 모드 **끄고** 평소대로 돌려 프레임을 찍는다 (점당 ~3초)
6. 병합: `merge_ra_dataset.py <스캔 run> --measured <채운 템플릿>` — 생략하면
   같은 task 이름의 가장 최근 "채워진" 템플릿을 자동으로 찾는다

주의: 마크 패스와 촬영 패스는 서로 다른 run이고, 둘 사이에 베이스 정지 오차
(±2 mm)가 들어간다. 표시 번호는 지점 옆에 작게.

공통: transition 행, 이동 실패 포인트, (A에서) 프레임이 없는 포인트는 멈추지
않는다. 대기 중 **Cancel arm motion / STOP ALL**은 스캔을 끝내고 공구는 후퇴한
채로 둔다(복귀 이동 없음).

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
2. "Ra data collection" 그룹: **Mode**(pause / mark), pause면 **points per
   stop**, mark면 **mark dwell** 설정 → **Collect mode** 체크 (arm_node 상태가
   돌아와 체크가 유지되면 켜진 것; `rostopic echo /arm/collect_state`로도 확인)
3. 평소대로 `Send TASK <이름>`
4. 정지할 때마다 측정/마킹 → 입력 → Record & next (또는 Next)
5. 끝나면 Collect mode 체크 해제 (켜둔 채 다음 TASK를 보내면 또 멈춘다;
   방법 B의 촬영 패스는 반드시 끄고 돌릴 것)

`robot.yaml collect.enabled: true`로 두면 부팅부터 켜진다(권장하지 않음 —
운영 스캔이 모두 멈춘다). `wait_timeout_s`를 0보다 크게 두면 그 시간 안에
입력이 없을 때 skip으로 기록하고 진행한다.

## 4. 결과물과 병합

| 파일 | 내용 |
|---|---|
| `log/apriltag_nav/ra_maps/<run>.csv` | Ra map: 포인트별 x y z, 모델 Ra, 거리 보정 결과 |
| `results/scan_images/<run>/<run>_ra_measured.csv` | 방법 A의 수기 Ra (사진과 같은 폴더; git에 올라감): run, mark_no(빈칸), index, group_id, point_id, images, ra_measured, ra_readings, note, skipped, standoff, x y z, measured_at |
| `results/scan_images/<run>/<run>_mark_template.csv` | 방법 B의 템플릿, 같은 컬럼에 mark_no가 채워져 있고 ra_measured는 빈칸 — 손으로 채운다 |
| `results/scan_images/<run>/g<group>_p<point>_i<index>_s<n>.png` | 프레임. 이름만으로 Ra map 행과 수기 행에 1:1 대응 |

```
python3 src/apriltag_nav/tools/merge_ra_dataset.py                       # 수기 CSV가 있는 모든 run (방법 A)
python3 src/apriltag_nav/tools/merge_ra_dataset.py <run stem>             # 한 run
python3 src/apriltag_nav/tools/merge_ra_dataset.py <스캔 run> --measured <채운 템플릿>   # 방법 B
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
- `/arm/collect_config` (String JSON): `{"mode": "pause"|"mark", "batch_size": N,
  "retreat_mm", "mark_retreat_mm", "mark_dwell_s"}` 중 일부
- `/arm/scan_continue` (String JSON): pause — `{"points": [{"group_id", "point_id",
  "ra" 또는 "readings": [..], "note", "skip"}, ...]}` (1점 배치는
  `{"ra": 0.41}` / `{"readings": [..]}` / `{"skip": true}`도 됨); mark — 아무 내용
- `/arm/collect_state` (latched JSON): enabled, mode, batch_size, waiting, kind
  (pause|mark), mark_no, index/total, group_id, point_id, images, points
  (배치의 각 점과 tip 기준 dx/dy), record_csv, n_recorded, n_skipped, last
- `/arm/scan_progress`에 `wait` / `resume` 이벤트 추가
