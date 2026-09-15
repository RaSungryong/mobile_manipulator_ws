# 웹 운영 UI (robot_ui_web) — 사용 안내

작성 2026-09-15. 대상: 로봇 PC 화면 앞이 아니라 **같은 네트워크의 다른 PC /
노트북 / 태블릿 브라우저에서** 로봇을 보고 조작하려는 운영자. 설치할 것이
없다 — 브라우저(Chrome / Edge / Firefox)만 있으면 된다.

---

## 1. 무엇인가

`robot_ui`의 PyQt 창(`roslaunch robot_ui robot_ui.launch`)과 **기능이 같은
운영 UI를 웹 페이지로** 제공하는 노드(`robot_ui_web_node.py`)다. 로봇 PC에서
HTTP + WebSocket 서버(기본 **8080 포트**)를 열고, 브라우저는 그 페이지를
연다. 여러 사람이 동시에 열 수 있고, **모든 창이 같은 로봇 상태를 본다**
(프리뷰 ON/OFF, 램프, 캘리브레이션 세션, 스크립트 실행 여부, 로그가 전부
공유된다). 어느 창에서든 버튼을 누를 수 있다.

```
브라우저 A ─┐
브라우저 B ─┼─ http://로봇PC:8080 ─▶ robot_ui_web_node ─▶ (RosBridge) ─▶ arm_node, mobile_node,
Windows PC ─┘   (JSON + JPEG / WebSocket)                        lifter_node, basler_camera_node, …
```

원칙은 PyQt 창과 동일하다: **UI는 어떤 장치도 소유하지 않는다.** 모든
버튼은 그 장치를 소유한 노드의 토픽/서비스 호출이다. PyQt 창과 웹 UI는
같은 `RosBridge`를 쓰며, 카메라 화면은 `robot_camera_node`의 오버레이
(`/<cam>/tag_overlay`)를 그대로 JPEG로 보낸다.

## 2. 시작하기

### 2-1. 로봇 PC

`mobile_manipulator.launch`에 포함돼 있다 (2026-09-15부터, `use_web_ui`
기본 true). 스택을 올리면 같이 뜬다:

```bash
source ~/mobile_manipulator_ws_20260902/devel/setup.bash
roslaunch apriltag_nav mobile_manipulator.launch
```

확인:
```bash
ss -ltn | grep 8080                        # 0.0.0.0:8080 LISTEN
curl -s http://127.0.0.1:8080/api/health   # {"ok": true, "clients": 0, ...}
```

따로 띄우거나(스택은 이미 올라와 있어야 한다) 포트를 바꾸려면:
```bash
roslaunch robot_ui robot_ui_web.launch port:=8090
# 스택 launch에서는:  roslaunch apriltag_nav mobile_manipulator.launch web_ui_port:=8090
# 끄려면:             ... use_web_ui:=false
```

X 디스플레이가 필요 없으므로 ssh 세션이나 systemd에서도 뜬다. PyQt 창을 같이
띄워도 된다(둘 다 같은 노드들을 호출할 뿐이다).

### 2-2. 같은 LAN(192.168.1.x)의 PC

브라우저 주소창에 **`http://192.168.1.100:8080`** (로봇 PC LAN 4 주소).

### 2-3. 사무실 쪽 Windows PC (192.168.0.x, 피닉스 AP브릿지 경유)

rosbridge(9090)와 같은 구조다 — `docs/ROSBRIDGE_kr.md` 1장 참조. 피닉스
브릿지에 **TCP 8080 → 192.168.1.100:8080** 포트 포워딩을 한 번 추가하면
(설정 화면 `https://192.168.0.20`), Windows 브라우저에서
**`http://192.168.0.20:8080`** 으로 연다. 페이지와 WebSocket이 같은 포트를
쓰므로 포워딩은 이 하나면 된다.

## 3. 화면 구성

PyQt 창과 같은 배치다.

| 영역 | 내용 |
|---|---|
| 상단 System 바 | E-STOP / BAT / CHARGE / ARM / LIFT / BASE / TASK / SCAN / CAM 칩, 접속 상태, **STOP ALL (soft)** |
| 왼쪽 카메라 | 큰 화면 1개 + 아래 썸네일 3개. 썸네일 **클릭** → 큰 화면으로 교체, **더블클릭** → 전체 화면(다시 더블클릭으로 복귀). "Last capture" 탭에 마지막 촬영 + Ra |
| 태그 카메라 칸 | `on`(카메라+검출기 켜기/끄기), `tags`(오버레이 ↔ 원본), 현재 검출된 태그 ID |
| Basler 칸 | 드래그로 ROI, 우클릭으로 해제. 초록 사각형 = 추론이 잘라 쓰는 900 px 중앙 영역 |
| Collect | Live preview, VISION lamp 홀드, 촬영(저장 폴더·접두어·장수·램프·저장·Ra 예측·ROI 사용), **CAPTURE** |
| Arm | 현재 TCP 자세, 조그(축별 ±, step/speed), Keyence 거리 보조(실시간 standoff, Auto standoff), 절대 이동(빈 칸 = 현재값 유지), Arm home pose, Cancel |
| Task | `/task_list`의 태스크 선택 + 상세, Send TASK, Reload tasks, GOTO, Dock & charge / Undock, 원문 명령, 리프트(mm 이동·원점복귀·정지) |
| Mobile | 거리 전진/후진, 각도 회전(속도 지정), Stop base, Clear stop latch, 베이스 상태 |
| Calibration | 맵 캘리브레이션 세션(플레이트/야우 스윕 선택, dry run, START/Cancel, 진행 카운트), 핸드-아이(auto-sample, capture, compute, load, reset, status), 단일 태그 locate |
| Scripts | `robot_ui/plugins/*.py` 목록, RUN / Stop (스크립트는 **로봇 PC에서** 실행된다; 어느 브라우저가 눌렀든) |
| Log | 서버 로그(최근 500줄은 접속 시 받아 온다), Clear log는 내 화면만 지운다 |

칩 색: 주황 = 동작 중, 초록 = 정상/대기, 빨강 = 오류/E-STOP, 자홍 = 충전 중.
접속 칩이 빨강(`DISCONNECTED — retrying`)이면 서버가 죽었거나 네트워크가
끊긴 것이며, 자동으로 재접속을 시도한다.

## 4. 규칙

- **인증이 없다.** 9090과 같은 규칙: 8080은 현장 LAN 안에서만 쓴다.
- **여러 창이 동시에 조작할 수 있다.** 창 하나에서 두 사람이 누르는 것과
  같다 — 마지막 명령이 이긴다. 협업할 때는 누가 조작하는지 말로 정한다.
- **STOP ALL은 소프트 정지**다(팔 취소, TASK STOP, 베이스 비상 래치, 리프트
  정지, 스크립트 취소, 프리뷰/램프 해제). 하드웨어 비상정지(PILZ)는 브라우저에서
  닿지 않는다.
- 브라우저 탭을 닫아도 로봇은 멈추지 않는다(PyQt 창과 같다). 프리뷰가 켜져
  있으면 **켜진 채로 남는다** — 마지막 사람이 끄거나 STOP ALL.
- 카메라 프레임은 카메라당 초당 최대 10장, 폭 1400 px로 줄여서 보낸다
  (`stream_fps`, `stream_max_width`). 원본 해상도는 서버에 있고 ROI·저장·추론은
  원본으로 한다. 느린 Wi-Fi에서는 프레임이 밀리지 않고 **빠진다**(한 장씩
  ack 방식).
- 촬영 파일은 **로봇 PC**의 `<ws>/results/captures`에 저장된다(Save folder는
  로봇 PC 경로).

## 5. 문제 해결

| 증상 | 원인 / 조치 |
|---|---|
| 페이지가 안 열림 | `ss -ltn \| grep 8080`. 없으면 launch 로그에서 `robot_ui_web`를 본다. `Address already in use`면 다른 서버(예전 노드, web_video_server)가 8080을 잡고 있다 → `web_ui_port:=8090` |
| Windows에서만 안 열림 | 피닉스 포워딩(8080)이 없거나 다른 공유기. `Test-NetConnection 192.168.0.20 -Port 8080` |
| 칩이 전부 `—` | 스택이 안 떠 있다(노드는 떠 있어도 토픽이 없음). `rosnode list` |
| 카메라가 검게만 나옴 | `robot_camera_node`가 없거나 그 카메라 `on`이 꺼짐. 원본을 보려면 `tags` 해제 |
| Basler 화면이 안 바뀜 | Live preview를 켜야 한다(장치는 평소 닫혀 있음). 5 fps가 센서 한계 |
| 접속 칩 빨강, 새로고침해도 안 됨 | 노드가 죽었다. `roslaunch robot_ui robot_ui_web.launch`로 다시 띄운다 — 스택은 건드릴 필요 없다 |
| Calibration 탭 `nodes: OFFLINE` | `roslaunch path_tag_locator path_tag_locator.launch`가 안 떠 있음(메인 launch에 없음) |
| 버튼 눌렀는데 로그에 `not connected` | 접속이 끊긴 순간. 재접속 후 다시 누른다 |

## 6. 개발 메모

- 코드: `src/robot_ui/src/robot_ui/web_ui.py`(UI 동작 = PyQt `MainWindow`의
  로직, 툴킷 없음), `web_server.py`(tornado), `web/`(index.html, app.js,
  style.css — 외부 CDN 없음, 인터넷 불필요), `scripts/robot_ui_web_node.py`.
- 프로토콜은 `web_server.py` 상단 주석. 새 기능은 `UiController.api_<이름>`
  메서드 하나 + `app.js`의 `call('<이름>', [...])` 한 줄이다.
- 검사: `python3 tools/check_web_ui.py`(컨트롤러 + 서버, 106항목),
  `python3 tools/check_web_ui_browser.py`(헤드리스 Chrome으로 실제 페이지,
  65항목, 스크린샷 저장), PyQt 창은 `QT_QPA_PLATFORM=offscreen python3
  tools/check_task_list_ui.py`(71항목). 셋 다 ROS 마스터 없이 돈다.
