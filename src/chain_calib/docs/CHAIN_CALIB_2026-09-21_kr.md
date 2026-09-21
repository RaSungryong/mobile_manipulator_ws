# front_cam ↔ hand_cam 체인 캘리브레이션 — 2026-09-21 세션 기록

**2026-09-18 세션과 분리된 기록입니다.** 그 세션(첫 측정, 원시 체인 오차
25 mm → base 보정 적용)은
[CHAIN_CALIB_2026-09-18_kr.md](CHAIN_CALIB_2026-09-18_kr.md)에 있고, 그 문서의
§5-1은 **09-18 데이터의 재분석**(인쇄 스케일 철회, 실측 스케일로 재적용)이지
새 측정이 아닙니다. 이 문서는 **새로 찍은 세션**의 기록입니다.

| | |
|---|---|
| 세션 디렉터리 | `log/chain_calib/20260921/` |
| 목적 | 09-18 보정이 반영된 체인의 **잔차 확인** (재측정·독립 검증) |
| 시트 | 같은 A0 시트, 스케일 **실측값 sx = sy = 1.0000, tag 90.00 mm** (사용자 강철자, 09-21 확정) |
| 시트 태그 높이 | **z = 0** — A0 용지 인쇄물이라 두께 없음. 계산에 들어오지 않음(09-18 문서 §5-1 (1)) |

## 0. 시작 상태 (baseline) — 이 값들 위에서 재는 것

git `17afd81`, `path_tag_locator.launch` 내림, 메인 스택 가동.

| 행렬 | 값 |
|---|---|
| `T_ab2mb` (extrinsics.yaml, **09-18 보정 반영됨**) | t (−14.51, −120.66, −641.21) mm, rpy (−1.110, −0.362, +178.657)° |
| `T_mb2fc_chain` (level frame, 생성값) | t (+550.00, 0, +303.00) mm, rpy (180, 0, 0)° |
| `T_hc2ee` (hand-eye, 09-18 스윕) | t (+35.63, −334.49, −151.54) mm, rpy (+0.144, −0.449, −179.443)° |

⚠️ **이 세션의 `solve`가 내는 D·F는 위 값에 대한 *잔차*입니다** — 09-18처럼
설계값 대비 오차가 아닙니다. 반영할 때는 현재 값 위에 한 번 더 접습니다
(`T_ab2mb · T_mb2fc · F · inv(T_mb2fc)`; 이 접기가 경로 무관하다는 것은 09-18
문서 §5-1에서 소수점까지 확인했습니다).

## 1. 기대값 — 이게 아니면 뭔가 바뀐 것

09-18 데이터를 실측 스케일로 다시 푼 결과가 예고하는 값입니다.

| 지표 | 기대 | 뜻 |
|---|---|---|
| `check`의 raw chain error | 5~10 mm / ~1° | 보정이 살아 있음 (보정 전에는 15~25 mm) |
| `solve`의 raw rms | ≈ 9 mm | 잔차 바닥 (뷰당 hand_cam 자세 잡음) |
| F (base 잔차) | 수 mm, jackknife sd 수준 | **보정할 게 없음 = 09-18 보정 확인** |
| verdict | UNDETERMINED 또는 작은 값 | 상수로 설명할 게 남지 않음 |
| 사용자 지표 raw bias | 수 mm | 09-18 재풀이에서 (+2.2, +1.3, +0.6) mm |

**F가 다시 10 mm를 넘으면** 09-18 이후 물리적으로 바뀐 것이 있다는 뜻입니다
(카메라·팔 마운트 이동, front_cam 재보정, hand-eye 교체). 그때만 반영합니다.

## 2. 실행 명령

```bash
cd ~/mobile_manipulator_ws_20260902 && source devel/setup.bash
S=log/chain_calib/20260921
SCALE="--sx 1.0 --sy 1.0 --tag-size 0.090"

rosrun chain_calib chain_calib.py $SCALE --frames 30 check          # 자세마다 점검
rosrun chain_calib chain_calib.py $SCALE --frames 30 capture $S     # 자세마다 1회, READY 뜰 때까지
rosrun chain_calib chain_calib.py $SCALE status $S
rosrun chain_calib chain_calib.py $SCALE solve  $S --holdout-every 4
rosrun chain_calib chain_calib.py $SCALE solve  $S --min-tags 4 --max-range 0.56   # 좋은 뷰만
```

수집 지침은 [README §3-2](../README.md)와 같고, 이번에는 **태그 4장 이상 /
0.55 m 이내**를 우선합니다(09-18 진단: 2태그 뷰는 10~20 mm 튀고, 4~6태그
0.55 m 이내 뷰는 1~4 mm; 필터링만으로 바닥이 8.6 → 4.3 mm).

## 3. 결과

<!-- 세션 종료 후 채웁니다: 샘플 수 / 커버리지 / fit 표 / verdict / F·D와 jackknife
     / 사용자 지표 / 09-18 대비 변화 / 반영 여부와 그 근거 -->

_아직 실행 전._

## 4. 검증

```bash
rosrun chain_calib verify_chain.py plan $S --fit none    # 현재 적용된 체인 그대로
rosrun chain_calib verify_chain.py run  $S --fit none
```

09-18의 기준선: 중심 편차 평균 (−0.3, −0.7) mm, 거리 0.504 ± 0.002 m, 그리고
팔 x 이동 600 mm에 걸친 ±12 mm 선형 기울기(2.08°). 이번 run부터 태그 위
카메라 실위치와 부호 있는 rpy 열이 기록되므로 그 기울기를 위치 성분과 기울기
성분으로 나눠 볼 수 있습니다.

_아직 실행 전._
