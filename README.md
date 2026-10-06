# Mobile Manipulator ROS1 Workspace

ROS1 Noetic / Ubuntu 20.04 환경에서 동작하는 모바일 매니퓰레이터 워크스페이스입니다
(Fairino FR10v6 + Navifra 베이스, AprilTag 주행, 스캔 태스크, Ra 추정).
이 파일은 **명령어 빠른 참조**입니다. 구조·근거·규칙은 `CLAUDE.md`, 현재 상태
체크리스트는 `docs/HANDOVER.md`, 두 파일과 `config/robot.yaml`이 이 파일과
어긋나면 그쪽이 맞습니다.

## 기본 준비

```bash
cd ~/mobile_manipulator_ws
source devel/setup.bash        # MM_WS, ROS_LOG_DIR=<ws>/log/ros 도 여기서 잡힌다
```

빌드:

```bash
catkin_make && source devel/setup.bash
catkin_make --only-pkg-with-deps apriltag_nav      # 특정 패키지만
```

## 메인 실행 — systemd 서비스 (2026-09-22부터)

로봇 PC가 부팅하면 navifra 드라이버(`navifra-robot`, roscore 포함) 뒤에
`mobile_manipulator.launch`가 서비스 `mobile-manipulator`로 자동으로 뜬다.
8개 apriltag_nav 노드 + rosbridge(:9090) + 웹 UI(:8080). `roscore`를 따로
띄우지 않는다.

```bash
sudo systemctl status mobile-manipulator
sudo systemctl restart mobile-manipulator      # 코드·설정 변경 반영 (충전 릴레이가 꺼진다)
journalctl -u mobile-manipulator -f
```

손으로 띄워야 할 때는 **먼저 서비스를 멈추고**, VS Code 터미널이 아닌 분리된
프로세스로 (`docs/STOP_LAUNCH_kr.md`):

```bash
sudo systemctl stop mobile-manipulator
setsid nohup roslaunch apriltag_nav mobile_manipulator.launch > log/ros/launch_$(date +%Y%m%d_%H%M).out 2>&1 &
src/apriltag_nav/tools/stop_stack.sh            # 손으로 띄운 launch 종료 (다른 터미널에서)
```

launch 인자 (서비스로 띄울 때는 `tools/systemd/mobile-manipulator.env`의 `LAUNCH_ARGS`):

```bash
use_front_cam:=true use_side_cam:=true use_hand_cam:=false   # 카메라 일부 끄기
front_cam_width:=1280 front_cam_height:=720 front_cam_fps:=30
use_rosbridge:=false use_web_ui:=false
debug_mode:=true                                              # EXEC / EVAL 허용
```

## 태스크 명령 (`/task_command`)

태스크 이름은 `src/apriltag_nav/task/csv`의 파일에서 나온다:
`assigned_workpoints_<key>.csv` → `scan_pose_<key>`, `rrt_final_path_<key>.csv`
→ `scan_joint_<key>`. 목록은 `rostopic echo /task_list` 또는 웹 UI Task 탭.
새 경로 파일에서는 **`scan_pose_*`를 먼저** 돌린다 (IK 실패가 진단, 조인트
재생은 검사 없이 움직인다).

```bash
rostopic pub -1 /task_command std_msgs/String "TASK scan_joint_offset0mm_h652"    # 2026-10-06 15:54 기준, 조인트 파일만
rostopic pub -1 /task_command std_msgs/String "TASK scan_joint_offset10mm_h662"
rostopic pub -1 /task_command std_msgs/String "TASK go_home"
rostopic pub -1 /task_command std_msgs/String "RELOAD_TASKS"   # task/csv 다시 읽기
rostopic pub -1 /task_command std_msgs/String "GOTO 105"
rostopic pub -1 /task_command std_msgs/String "CHARGE"         # 도크 500 → 충전 on
rostopic pub -1 /task_command std_msgs/String "UNDOCK"         # 충전 off (베이스는 그대로)
rostopic pub -1 /task_command std_msgs/String "STOP"
rostopic pub -1 /task_command std_msgs/String "STATE"
rostopic echo /task_state
```

Ra 학습 데이터 수집(Collect 모드)은 `docs/RA_COLLECT_kr.md`.

## 외부 접속 (ROS 없는 PC)

- rosbridge `ws://192.168.1.100:9090` (윈도우 PC는 피닉스 브리지 경유
  `ws://192.168.0.20:9090`) — `tools/rosbridge/robot_cmd.py`, `docs/ROSBRIDGE_kr.md`.
- 웹 UI `http://192.168.1.100:8080` — `docs/ROBOT_UI_WEB_kr.md`.
- 둘 다 인증 없음, 현장 LAN 안에서만.

## 모바일 베이스

```bash
rostopic pub -1 /mobile/goto_tag std_msgs/Int32 "data: 105"
rostopic pub -1 /mobile/move_cmd std_msgs/String '{"type":"move","distance_m":0.3,"speed":0.05}'
rostopic pub -1 /mobile/move_cmd std_msgs/String '{"type":"pivot","angle_deg":90}'
rosservice call /mobile/stop "{}"        # 비상 정지 래치 (clear_stop 으로 해제)
rosservice call /mobile/cancel "{}"
rosservice call /mobile/clear_stop "{}"
rostopic echo /mobile/state
rostopic echo /robot_pose                # flag True = 도착 포즈, False = 10 Hz 실시간
```

`/cmd_vel`은 `mobile_node`만 publish 한다. `tools/navigate.py`, `tools/vw_drive.py`는
스택이 내려간 상태에서만.

## 카메라

```bash
rosservice call /robot_camera/front_cam/set_enabled "data: false"   # side_cam / hand_cam 도 같은 형식
rostopic hz /front_cam/tag_detections
rostopic echo -n 1 /front_cam/color/camera_info
rostopic echo /camera/state                                          # Basler
rosservice call /camera/set_lamp "data: true"                        # VISION 램프 홀드
```

태그 오버레이(`/<cam>/tag_overlay`)는 웹 UI에서 본다. RViz는 launch에 없다
(`rosrun apriltag_nav camera_viewer_node.py _auto_start:=true` 로 따로).

## 팔

```bash
rosservice call /arm/move_home "{}"
rosservice call /arm/reset_error "{}"                 # 리미트·충돌 후 에러 해제 (움직이지 않음)
rostopic pub -1 /arm/cancel std_msgs/Bool "data: true"
rostopic echo /arm/state
rostopic pub -1 /arm/move_cart std_msgs/String '{"pose":[x,y,z,rx,ry,rz],"vel":20.0}'   # mm / deg, MoveL
rostopic pub -1 /arm/jog_cmd std_msgs/String '{"axis":"z","delta":1.0,"vel":10.0}'
rostopic pub -1 /arm/move_joint std_msgs/String '{"joints":[j1,j2,j3,j4,j5,j6]}'
rostopic pub -1 /arm/jog_joint std_msgs/String '{"joint":"j3","delta":-2.0}'
rostopic pub -1 /arm/standoff std_msgs/String '{"target_mm":16.5}'                     # Keyence 표준거리 보정
rostopic echo /arm/standoff_state
```

## 리프트 (`/lifter/*` — `/lift/*`는 드라이버 원시 토픽, 쓰지 말 것)

```bash
rosservice call /lifter/home "{}"        # 리프트 원점복귀 (하한 스위치까지 하강)
rosservice call /lifter/stop "{}"
rostopic pub -1 /lifter/height_cmd std_msgs/Float32 "data: 150.0"   # mm, 원점복귀 후에만
rostopic pub -1 /lifter/jog_cmd std_msgs/Int32 "data: 100"          # counts, 원점복귀 전에도
rostopic echo /lifter/state
rostopic echo /lifter/height
```

## 맵 캘리브레이션 (path_tag_locator)

메인 스택이 **떠 있는 상태에서** 캘리브레이션 launch를 함께 띄운다 (하드웨어는
스택이 소유; 세션 중 TASK/GOTO 금지). 보통은 웹 UI Calibration 탭에서 시작한다.

```bash
setsid nohup roslaunch path_tag_locator path_tag_locator.launch > log/ros/calib_$(date +%Y%m%d_%H%M).out 2>&1 &
roslaunch path_tag_locator path_tag_locator.launch use_handeye_calib:=true   # 핸드아이 노드까지
```

```bash
rosservice call /map_calibrator/run_calibration "{plan_path: '', ref_tags_path: '', map_in_path: '', map_out_path: '', dry_run: true}"
rostopic echo /map_calibrator/progress
rosservice call /map_calibrator/cancel_calibration "{}"
rosservice call /path_tag_locator/locate_path_tag "{tag_b_id: 105, override_ref: false, save_result: true, save_dir: '', auto_align: true, align_initial_tcp_mm_deg: [0,0,0,0,0,0]}"
rosservice call /handeye_calib/auto_sample "{}"; rosservice call /handeye_calib/compute "{}"
ls -lt log/path_tag_locator/map_world_*.yaml | head -1
ls -lt log/apriltag_nav/nav_log/$(date +%Y%m%d)/          # TASK / GOTO 마다 yaml 하나
```

기본 플랜은 `config/calibration_plan_plate1.yaml` + `reference_tags.yaml`; 정반 2는
`_plate2` 쌍을 같이 바꾼다. 절차: `src/path_tag_locator/docs/CALIBRATION_GUIDE_kr.md`.

## 고정 변환 (tf chain)

모든 고정 변환(T_ab2mb, T_mb2fc, T_hc2ee, T_ee2tip, 관절 offset)은
`src/apriltag_nav/config/tf/`에 있다:

```bash
python3 src/apriltag_nav/tools/tf_chain_tool.py show
python3 src/apriltag_nav/tools/tf_chain_tool.py check
python3 src/apriltag_nav/tools/tf_chain_tool.py set T_hc2ee --npz F --source "..."
python3 src/apriltag_nav/tools/tf_chain_tool.py front-cam --apply      # robot.yaml 에서 T_mb2fc 생성
```

A0 시트 체인 캘리브레이션·hand_cam 내부 파라미터·Basler 팁: `src/chain_calib/README.md`,
`docs/HAND_CAM_INTRINSICS_kr.md`; front_cam 자세: `docs/FRONT_CAM_POSE_CALIBRATION_kr.md`.

## 오프라인 검사 / 도구

```bash
python3 src/apriltag_nav/tools/check_task_discovery.py           # task/csv 등록 확인
python3 src/apriltag_nav/tools/check_retarget_joint_paths.py     # 조인트 파일이 현재 맵용인지
python3 src/apriltag_nav/tools/retarget_joint_paths.py --help    # 새 플래너 export 재해석
python3 src/apriltag_nav/tools/ra_map_plotter.py <ra_map.csv>
python3 src/apriltag_nav/tools/merge_ra_dataset.py <run>
rosrun apriltag_nav test_all_devices.py                          # 노드/토픽/서비스 점검
rosrun path_tag_locator verify_map_world.py
```

`src/*/tools/check_*.py`, `src/*/scripts/check_*.py`는 코드 변경 뒤 돌리는
회귀 검사(roscore 불필요)다.

## Git

작업은 `real` 브랜치, 원격 `origin/real` (`github.com/RaSungryong/mobile_manipulator_ws`).
push 토큰은 워크스페이스 루트의 `token` 파일(gitignore). `--force` 금지.

```bash
git push origin real
```
