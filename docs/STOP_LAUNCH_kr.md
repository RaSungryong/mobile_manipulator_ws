# launch 깔끔하게 내리기

메인 스택(`roslaunch apriltag_nav mobile_manipulator.launch`)이나 캘리브레이션
launch(`path_tag_locator.launch`)를 종료하는 절차. 2026-09-21에 실제로
겪은 "SIGINT를 보냈는데 노드 16개가 종료 직전에 전부 멈춤" 사례를 기준으로
작성.

## 0. 먼저 알아둘 것

| | |
|---|---|
| **절대 죽이면 안 되는 것** | `roslaunch motor_driver robot.launch …` (navifra 드라이버, systemd `navifra-robot`). **roscore가 그 안에서 돕니다** — 죽이면 마스터가 사라져 모든 노드가 같이 죽음. 드라이버는 `sudo systemctl restart navifra-robot` 로만. |
| **정상 종료 = SIGINT** | launch 터미널의 Ctrl-C 와 동일. roslaunch가 노드들에 순서대로 SIGINT → 각 노드의 rospy 종료 훅 실행 (VISION 램프 off, Basler close, 팔 disconnect, 충전 릴레이 off, STATUS 램프 off). |
| **`kill -9` 는 마지막 수단** | roslaunch만 죽고 노드들이 고아로 남아 마스터에 등록된 채 계속 돎 → 다음 launch 때 이름 충돌, `/cmd_vel` 이중 publisher. |
| **launch를 띄운 터미널과 다른 터미널에서 작업** | launch 터미널이 얼어 있는 경우(§3) 거기서 명령을 치면 같이 얼어붙음. |
| **종료 = 충전 릴레이 off** | `task_executor`의 `devices.shutdown()`이 릴레이를 끊음 (CLAUDE.md 2026-09-14). 충전 중이었다면 재시작 후 `UNDOCK` → `CHARGE`. |

## 0.5. 부팅 시 자동 실행 — systemd `mobile-manipulator` (2026-09-22)

메인 launch는 이제 로봇 PC가 켜질 때 **systemd 서비스로 자동 실행**된다
(`src/apriltag_nav/tools/systemd/`). navifra 드라이버 서비스(`navifra-robot`,
roscore 포함)가 뜬 **뒤에** 시작하고, 드라이버를 재시작하면 같이 재시작된다
(`Requires=` / `After=` / `PartOf=navifra-robot.service`).

```bash
# 설치 (한 번, sudo 필요). 부팅 시 자동 시작으로 등록만 하고 지금은 띄우지 않음
sudo src/apriltag_nav/tools/systemd/install_service.sh
# 지금 바로 띄우려면: 먼저 손으로 띄운 launch를 내리고(§1) 나서
sudo systemctl start mobile-manipulator

sudo systemctl status mobile-manipulator      # 상태
sudo systemctl stop mobile-manipulator        # 종료 (= Ctrl-C: 모든 노드 SIGINT, 종료 훅 실행)
sudo systemctl restart mobile-manipulator     # 재시작 (팔·센서가 늦게 켜졌을 때)
journalctl -u mobile-manipulator -f           # 노드 출력 (output=screen), 실시간
ls log/ros/                                   # roslaunch·노드 로그 (ROS_LOG_DIR, 기존과 동일)
sudo systemctl disable mobile-manipulator     # 자동 시작만 끄기 (설치는 유지)
sudo src/apriltag_nav/tools/systemd/install_service.sh --uninstall
```

| | |
|---|---|
| **시작 순서** | `run_stack.sh`가 ROS master(최대 120 s) → Fairino 컨트롤러 RPC 포트 192.168.58.2:20003(최대 180 s) → Keyence(최대 30 s)를 기다린 뒤 `roslaunch apriltag_nav mobile_manipulator.launch`. 팔·Keyence가 시간 안에 안 켜지면 **경고만 남기고 그냥 띄운다** — 카메라·베이스·웹 UI는 팔 없이도 쓸 수 있고, `arm_node`만 죽어 있다. 팔을 켠 뒤 `sudo systemctl restart mobile-manipulator`. |
| **launch 인자** | `src/apriltag_nav/tools/systemd/mobile-manipulator.env`의 `LAUNCH_ARGS=` (예: `use_hand_cam:=false`). 고친 뒤 restart. 대기 시간도 같은 파일(`WAIT_ARM_S=0` 이면 안 기다림). |
| **손으로 띄울 때** | 서비스가 떠 있는 채로 `roslaunch … mobile_manipulator.launch`를 또 띄우면 노드 이름 충돌로 서로 죽이고 9090/8080 포트가 겹친다. **먼저 `sudo systemctl stop mobile-manipulator`**, 끝나면 `start`. `install_service.sh --start`와 `tools/stop_stack.sh`가 이 경우를 검사한다 (stop_stack은 서비스로 뜬 launch에는 손대지 않고 systemctl을 안내). |
| **종료 = 충전 릴레이 off** | §0과 같다. 리부트·`systemctl stop`도 `task_executor`의 shutdown 훅을 지나므로 충전 중이면 끊긴다. 재시작 후 `UNDOCK` → `CHARGE`. |
| **arm은 움직이지 않는다** | 자동 시작이라도 CLAUDE.md의 규칙 그대로: 팔 홈 자세 이동 없음, 리프트 원점복귀 없음(`lifter.auto_home_on_start` false; 드라이버 자체의 원점복귀는 별개로 navifra가 함). |
| **로그 위치** | stdout은 journal(`journalctl -u mobile-manipulator`), 파일 로그는 `devel/setup.bash`의 env hook이 잡는 `<ws>/log/ros/<run_id>/` — 터미널에서 띄울 때와 같다. |
| **워크스페이스 경로** | 설치 스크립트가 유닛 파일에 이 워크스페이스의 절대 경로를 박아 넣는다. 워크스페이스를 옮기거나 다른 checkout으로 바꾸면 `install_service.sh`를 그 위치에서 다시 실행. |

## 1. 한 줄로: `tools/stop_stack.sh`

```bash
cd ~/mobile_manipulator_ws && source devel/setup.bash
src/apriltag_nav/tools/stop_stack.sh                       # mobile_manipulator.launch
src/apriltag_nav/tools/stop_stack.sh path_tag_locator.launch
src/apriltag_nav/tools/stop_stack.sh --force               # SIGKILL 전 y/n 생략
```

하는 일: launch 프로세스 찾기 (navifra는 거부) → SIGINT → 최대 30 s 대기
(`--timeout N`) → 안 내려가면 launch 터미널 pty가 막혔는지 검사해서 이유를
출력하고, 확인 후 자식 노드 → roslaunch 순으로 SIGKILL → `rosnode cleanup`
→ 남은 노드 목록과 9090/8080 포트 상태 출력.

## 2. 손으로 할 때

```bash
# 1) 무엇이 떠 있는지 — PID, 시작 시각, launch 파일
ps -eo pid,lstart,cmd | grep roslaunch | grep -v grep

# 2) 해당 launch에만 SIGINT (= Ctrl-C)
kill -INT <PID>                       # 또는 pkill -INT -f mobile_manipulator.launch

# 3) 확인 — 스택 노드들이 사라지고 navifra 노드 7개만 남아야 함
rosnode list
ps -eo pid,stat,cmd | grep -E "arm_node|mobile_node|lifter_node|task_executor|robot_ui_web|rosbridge" | grep -v grep
```

노드 하나만 내릴 때는 `rosnode kill /robot_camera_node` — 이 launch는 respawn이
없어 다시 안 뜨며, `rosrun` 으로 다시 올린다 (launch가 마스터에 남긴 `~driver_*`
파라미터는 그대로 유효).

## 3. SIGINT 뒤에도 안 내려갈 때 — 터미널이 원인인지 먼저 본다

증상 (2026-09-21 13:31): `rosnode list` 에서는 이미 사라졌는데 프로세스들은
살아 있고, roslaunch 로그 끝에 `ProcessMonitor shutdown failed!`.

```bash
# roslaunch가 어느 터미널에 붙어 있나
ps -o pid,tty -p <PID>                # 예: pts/1
# 그 터미널에 쓰기가 되나 (124 = 막힘)
timeout 2 sh -c 'echo probe > /dev/pts/1'; echo $?
# 노드들이 어디서 멈춰 있나
for c in $(ps -o pid= --ppid <PID>); do echo "$c $(cat /proc/$c/wchan)"; done
```

`tty_write_lock` / 쓰기 타임아웃이면 **launch 터미널(VS Code 통합 터미널)이 출력을
받아가지 않는 것**이다. 노드들은 마지막 종료 메시지를 stdout에 쓰다가 걸렸고,
roslaunch의 SIGTERM→SIGKILL 에스컬레이션도 같은 터미널에 로그를 쓰려다 같이
걸린다. 이 시점에는 하드웨어 쪽 훅은 이미 끝나 있으므로 (로그와
`rostopic echo -n1 /crevis/led_state_all` 로 `vision=0` 확인) 강제 종료해도 된다:

```bash
kill -KILL $(ps -o pid= --ppid <PID>)   # 자식 노드 먼저
kill -KILL <PID>                         # roslaunch
echo y | rosnode cleanup                 # 마스터에 남은 stale 등록 제거
rosnode list; ss -ltnp | grep -E ':9090|:8080'
```

그 다음 **얼어 있는 VS Code 터미널 탭을 닫고**(휴지통), launch는 새 터미널에서
`source devel/setup.bash` 후 다시 띄운다. 같은 탭에서 다시 띄우면 똑같이 걸린다.
Ctrl-S(XOFF)를 잘못 누른 경우라면 그 탭에서 Ctrl-Q 로 풀린다.

강제 종료 시 실행되지 못하는 훅은 `task_executor`의 STATUS 램프 off 뿐이라
녹색 램프가 켜진 채 남는다 — 다음 launch가 덮어쓴다.

### 3.1. 종료할 때만이 아니라 **실행 중에도** 걸린다 (2026-09-22 16:02)

같은 원인이 map 캘리브레이션 세션을 **도중에** 멈췄다. `path_tag_locator.launch`
를 VS Code 터미널(pts/0)에서 띄우고 캘리브레이션을 돌리던 중, 그 터미널이 출력을
받아가지 않게 되자 `map_calibrator`가 123번 태그의 `initial MoveJ 1/4` 로그 한 줄을
stdout에 쓰다가 그대로 멈췄다 — 파일 로그(`log/ros/…/map_calibrator-2.log`)에는
그 줄이 남았지만 바로 다음의 `/arm/move_cart` publish는 나가지 않았고,
로그 락에 다른 스레드까지 걸려 60 s 타임아웃도 안 찍힌다. `arm_node`에는 명령이
도착한 흔적이 없고 `rosnode list`에는 멀쩡히 남아 있다. 진단은 §3과 같다
(`timeout 3 bash -c 'echo > /dev/pts/N'` 가 124, 스레드 wchan `wait_woken`).
Ctrl-S는 아니었다(`termios.tcflow(TCOON)` 보내도 그대로).

**규칙: 손으로 띄우는 launch(캘리브레이션 launch 포함)는 VS Code 터미널 패널에
붙이지 말고 detached로, 출력은 파일로:**

```bash
cd ~/mobile_manipulator_ws && source devel/setup.bash
setsid nohup roslaunch path_tag_locator path_tag_locator.launch \
    > log/ros/calib_launch_$(date +%Y%m%d_%H%M).out 2>&1 &
tail -f log/ros/calib_launch_*.out      # 보고 싶으면 이렇게
```

멈춘 세션은 살릴 수 없다(kill -9 + `rosnode cleanup`, §3). 세션 기록은 항목마다
원자적으로 저장되므로 잃는 것은 없고, 남은 태그만 subset plan으로 돌린 뒤 두
`map_world_*.yaml`을 합치면 된다 — 단, **재시작한 노드가 읽는 tf 체인이 멈춘
세션의 것과 같은지 먼저 확인**할 것(항목 yaml의 `joint_offsets_applied` 유무,
`config/tf/*` 수정 시각). 2026-09-22에는 그 사이에 체인이 바뀌어 100–122(옛 체인)와
123–125(새 체인)가 20 mm 이상 어긋났고, 병합본은 쓰지 못하고
`calibrate/20260922_161657/map_world_plate1_merged_CHAIN_MISMATCH.yaml` 로
남겨 두었다 — 결국 전체를 새 체인으로 다시 돌려야 한다.

## 4. 재시작 체크리스트

- 새(얼지 않은) 터미널, `source devel/setup.bash` (→ `MM_WS`, `ROS_LOG_DIR=log/ros`)
- 손으로 띄운 `rosbridge_websocket` 이 있으면 먼저 종료 (9090 충돌)
- `roslaunch apriltag_nav mobile_manipulator.launch`
- 충전 중이었으면 `UNDOCK` → `CHARGE`
