# Ra 학습 데이터 수집 가이드 (수집 모드 · 방법 A/B · 이어하기 · 병합)

Ra 모델 학습 데이터 = 스캔 TASK가 찍은 Basler 사진 + 같은 자리를 조도측정기로
잰 Ra. **수집 모드**를 켜고 평소 TASK를 보내면 로봇이 측정할 수 있게 멈춰
준다. 웹 UI(`http://192.168.1.100:8080`, 폰도 됨) Task 탭 "Ra data collection"
에서 전부 조작한다. 설계·검증 기록은 CLAUDE.md Work Log 2026-10-06/07.

## 1. 두 방법

| | **A — pause** | **B — mark** |
|---|---|---|
| 멈추는 때 | N점 찍을 때마다 | 점마다(번호 쓰기) + 그룹 끝(Ra 입력) |
| 자리 찾기 | 표의 tip 기준 거리(dx, dy) | 케이스 옆에 쓴 **번호** |
| 맞는 경우 | 점 적을 때, 측정기 1~2대 | 점 많을 때, 측정기 여러 대 |
| 기록 | 멈출 때마다 Ra 입력 | 번호는 즉시 기록(대기 행), Ra는 그룹 끝에 |

## 2. 설정 (Task 탭 "Ra data collection")

| 항목 | 뜻 |
|---|---|
| **Collect mode** 체크 | 수집 모드 켜기. 체크가 유지돼야 켜진 것. ⚠️ `systemctl restart` 하면 꺼지고 Mode도 pause로 돌아간다 — 재시작 뒤 다시 설정 |
| **Mode** | `pause`(A) / `mark`(B). 다음 TASK부터 |
| **points per stop** | A: 몇 점마다 멈출지 (측정기 2대면 2) |
| **mark spot before retreat** | A: 촬영 직후 공구가 내려간 채 한 번 더 멈춤 → 케이스 옆에 점 표시 후 Next. A에서는 켜는 게 기본 (tip은 가상점이라 80 mm 위에선 못 찾음) |
| **mark dwell [s]** | B: 번호 정지를 몇 초 뒤 자동 진행. 0 = Next 누를 때까지 |
| **Groups** (위쪽 줄) | 오늘 할 그룹(=태그)만 체크. 안 한 그룹은 주행 안 하고 다음에 다시 고를 수 있음 |

robot.yaml: `collect.wait_timeout_s` (0 = 입력 올 때까지 대기; >0이면 그 시간
뒤 skip으로 기록하고 진행), `keyence.require_converged: true` 권장(거리 보정
미수렴 점은 찍지 않음 — 초점 안 맞은 사진은 학습에 못 씀).

시작 전 한 번: 금형 위에서 Arm 탭 **Auto standoff**로 표면이 Keyence 범위
안에 있는지 확인. 측정 조건(cutoff, 방향)은 note 칸에.

## 3. 방법 A 절차

1. Mode `pause`, points per stop, premark 체크, Collect mode 체크 → **Send TASK**
2. (premark) 촬영 직후 공구가 내려간 채 멈춤, 칩 `MARK SPOT` → 케이스 옆에 점
   표시 → **Next**
3. N점째 뒤 공구 80 mm 후퇴, 칩 `WAIT Ra N pt`, 표에 N행. 마지막 행이 tip
   바로 아래, 나머지는 거기서 dx, dy mm (joint 전용 task는 거리 없음)
4. 재서 입력: 값 하나 `0.94`, 여러 번은 `0.94 0.97`(평균 기록), 못 잰 행은
   skip 체크. Enter로 다음 행 → **Record & next** (**Skip all** = 전부 미측정)
   **중간 저장 (2026-10-07 저녁):** Enter는 그 행을 CSV에 바로 쓰고 내려간다;
   **Save**는 값/skip이 적힌 행 전부를 쓴다. 정지는 열린 채, saved 열에 ✓.
   탭이 닫히거나 폰이 꺼지거나 스택이 재시작돼도 ✓ 행은 남고, 다시 열면
   표에 미리 채워진다. Record & next는 그때 표의 값으로 덮어쓴다.
   ⚠️ **Skip all은 행이 많을 때 누르지 말 것** — 전부 skip으로 적히고 Resume은
   그 점을 "끝난 점"으로 본다(10-07 16:30에 553점이 그렇게 됐다; §8).
5. 공구 복귀, 계속. 끝나면 **Collect mode 해제**

## 4. 방법 B 절차

1. Mode `mark`, Collect mode 체크 → **Send TASK**. 로그에
   `MARK pass: … (numbers continue from #1)`
2. 점마다 촬영 후 공구가 내려간 채 멈춤, 칩 `MARK #n`, 읽기줄
   `MARK #n = g<group> p<point>`. 케이스 **옆**에 **n**을 쓰고 **Next (number
   written)** (dwell > 0이면 자동). 번호는 그룹 넘어 계속 이어짐
3. 그룹 마지막 점 뒤 공구 80 mm 후퇴, 칩 `WAIT Ra N pt`, 표에 그 그룹 전체가
   mark 열 번호와 함께. 번호 자리마다 재서 입력(§3의 4와 같음) → **Record & next**
4. 다음 그룹으로. 끝나면 **Collect mode 해제**

⚠️ 팔이 움직이는 동안은 정반에 손을 넣지 않는다 — 측정은 `WAIT Ra` 중에만.

## 5. 중간에 멈추기 / 끊김

- **Cancel arm motion / STOP ALL / 비상정지**: 스캔 끝. 공구는 그 자리에
  남는다(번호 정지면 내려간 채, 입력 정지면 후퇴한 채). 베이스를 손으로
  옮기기 전에 **Arm home pose**.
- 그때까지 찍은 점은 모두 디스크에 있다(사진, Ra map 행, 수기 CSV 행). B는
  번호를 쓴 순간 CSV에 "대기 행"(번호 + 사진, Ra 빈칸)이 남으므로 사진과
  번호가 살아 있다.

## 6. 이어하기 (Resume)

**다시 Send TASK 하지 말 것** (새 run이 되어 처음부터, 번호도 #1부터).

1. 원인 정리: 비상정지는 풀고 판넬 RESET(램프 초록 = IDLE). 팔 오류면 Arm 탭
   **Reset arm error**. 베이스는 태그 위에 서 있어야 함(카메라에 태그)
2. §2 설정 재확인 (재시작했으면 Collect mode·Mode가 풀려 있음)
3. Task 탭: 같은 task 선택 → Groups 체크 → **Resume run** 칸은 비워 두고
   (= 남은 점 있는 가장 최근 run) → **Resume interrupted run**
   (원문: `RESUME [<run>] groups=119`)
4. 로그 `[RESUME] … N of M done, K left; … await their Ra entry` 확인.
   B면 `numbers continue from #<다음 번호>`가 떠야 함(#1이면 잘못 잡힌 것)

Resume이 하는 일: 끝난 그룹은 안 간다. 일부만 한 그룹은 가서 경로를 타되
**끝난 점은 정지 없이 통과**하고 남은 점만 찍는다. 끝난 점 = Ra 입력/skip된
점, B에서는 번호까지 쓴 점(다시 찍지 않음 — 그 번호 행은 그 그룹의 입력
정지 표에 같이 올라온다; Save로 저장된 점은 입력된 점이라 안 올라온다).
입력 정지 도중 끊긴 그룹은 통과만 하고 끝에서
입력 정지를 한 번 더 연다. 결과는 **같은 run 파일**에 이어 쌓인다. 며칠에
나눠 해도 되지만 그 사이 **경로 파일(task/csv)은 바꾸지 않는다**.

## 7. 결과와 병합

| 파일 | 내용 |
|---|---|
| `log/apriltag_nav/ra_maps/<run>.csv` | Ra map (모델 Ra, 거리 보정 결과) |
| `results/scan_images/<run>/g<g>_p<p>_sp<s>_i<i>_s1.png` | 사진 |
| `results/scan_images/<run>/<run>_ra_measured.csv` | 수기 Ra (mark_no, group, point, 사진 이름, ra_measured, ra_readings, note, skipped …) |

`<run>` = `<task>_ra_map_<시각>`. 끝났는지: `grep -c "pending: Ra" <수기 CSV>`
가 0.

```
python3 src/apriltag_nav/tools/merge_ra_dataset.py <run>     # → results/ra_dataset/<run>_dataset.csv
```
사진 1장 = 1행 (image, source_point_id, ra_measured, model Ra, standoff_ok …).
요약의 `frames unmeasured`(Ra 없음) / `rows standoff NOT ok`(초점 안 맞음 →
제외)를 본다.

## 8. 자주 겪는 것

| 증상 | 조치 |
|---|---|
| `MARK #` 없이 점마다 `WAIT Ra 1 pt` | Mode가 pause로 돌아감 → STOP ALL, Mode mark, 다시 |
| 멈추지 않고 찍기만 함 | Collect mode 꺼짐 → 체크(다음 점부터) |
| 번호가 #1부터 다시 | 새 run이 시작됨 → STOP ALL, Resume으로 |
| Record & next 안 됨 | 빈 행 있음(로그 `type the Ra or tick skip`) → 값 또는 skip |
| Skip all을 잘못 눌러 입력 못 한 행이 skip으로 적힘 | CSV에서 그 행을 대기 행으로 되돌리고(ra/readings/measured_at 빈칸, skipped False, note `pending: Ra at the group entry stop`; 백업 먼저) Groups에 그 그룹만 체크해 Resume → 입력 정지가 그 행들로 다시 열린다 (10-07 g119 #337–#589) |
| 한 점에서 10초 넘게 정지 | 팔 이더넷 링크 드롭 — 스스로 이어감. 안 되면 §6 |
| 표에 dx, dy 없음 | joint 전용 task. 정상 (B면 번호로 찾음) |

토픽(스크립트용): `/arm/collect_mode` Bool, `/arm/collect_config` JSON
(`mode, batch_size, premark, mark_dwell_s`), `/arm/scan_continue` JSON
(B 번호 정지: 아무 내용; 입력 정지: `{"points":[{"group_id","point_id",
"readings":[..],"note","skip"}]}`), `/arm/collect_save` JSON(같은 `points` 꼴의 일부 — 정지를 열어 둔 채
CSV에 씀), `/arm/collect_state` latched(`saved`, `n_saved` 포함),
`/task_command` `RESUME [<run>] [groups=…]`.
