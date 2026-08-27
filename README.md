# Mobile Manipulator ROS1 Workspace

ROS1 / Ubuntu 20.04 환경에서 동작하는 모바일 매니퓰레이터 워크스페이스입니다.

## 기본 준비

모든 터미널에서 먼저 실행합니다.

```bash
cd /home/jun/mobile_manipulator_ws_20260819/mobile_manipulator_ws
source devel/setup.bash
```

빌드가 필요하면 아래를 사용합니다.

```bash
catkin_make
source devel/setup.bash
```

특정 패키지만 빌드할 때:

```bash
catkin_make --only-pkg-with-deps apriltag_nav
catkin_make --only-pkg-with-deps robot_msgs
catkin_make --pkg path_tag_locator
```

## 메인 실행

전체 로봇 스택 실행:

```bash
roslaunch apriltag_nav mobile_manipulator.launch
```

카메라 일부를 끄고 실행:

```bash
roslaunch apriltag_nav mobile_manipulator.launch use_front_cam:=true use_side_cam:=true use_hand_cam:=false
roslaunch apriltag_nav mobile_manipulator.launch use_front_cam:=true use_side_cam:=false use_hand_cam:=false
```

front camera 해상도 지정:

```bash
roslaunch apriltag_nav mobile_manipulator.launch front_cam_width:=1280 front_cam_height:=720 front_cam_fps:=30
```

debug command 허용:

```bash
roslaunch apriltag_nav mobile_manipulator.launch debug_mode:=true
```

## 태스크 실행

메인 launch가 실행 중인 상태에서 다른 터미널에 입력합니다.

```bash
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line2"
rostopic pub -1 /task_command std_msgs/String "TASK scan_grid_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_grid_line2"
rostopic pub -1 /task_command std_msgs/String "TASK go_home"
```

특정 AprilTag로 이동:

```bash
rostopic pub -1 /task_command std_msgs/String "GOTO 500"
rostopic pub -1 /task_command std_msgs/String "GOTO 108"
```

정지 및 상태 확인:

```bash
rostopic pub -1 /task_command std_msgs/String "STOP"
rostopic pub -1 /task_command std_msgs/String "STATE"
rostopic echo /task_state
```

## 모바일 베이스 직접 명령

```bash
rostopic pub -1 /mobile/goto_tag std_msgs/Int32 "data: 500"
rostopic pub -1 /mobile/goto_tag std_msgs/Int32 "data: 108"
rosservice call /mobile/stop "{}"
rosservice call /mobile/cancel "{}"
rosservice call /mobile/clear_stop "{}"
rostopic echo /mobile/state
rostopic echo /mobile/busy
```

## 카메라

RViz camera viewer:

```bash
rosservice call /camera_viewer/set_enabled "data: true"
rosservice call /camera_viewer/set_enabled "data: false"
```

AprilTag detector on/off:

```bash
rosservice call /robot_camera/front_cam/set_enabled "data: true"
rosservice call /robot_camera/front_cam/set_enabled "data: false"
rosservice call /robot_camera/side_cam/set_enabled "data: true"
rosservice call /robot_camera/side_cam/set_enabled "data: false"
rosservice call /robot_camera/hand_cam/set_enabled "data: true"
rosservice call /robot_camera/hand_cam/set_enabled "data: false"
```

카메라 확인:

```bash
rostopic hz /front_cam/tag_detections
rostopic echo /front_cam/tag_detections
rostopic echo -n 1 /front_cam/color/camera_info
rostopic echo /camera/state
```

## 팔 제어

```bash
rosservice call /arm/move_home "{}"
rostopic pub -1 /arm/cancel std_msgs/Bool "data: true"
rostopic echo /arm/state
rostopic echo /arm/status
```

Cartesian 이동:

```bash
rostopic pub -1 /arm/move_cart std_msgs/String '{"pose":[0.0,0.0,0.0,0.0,0.0,0.0],"vel":20.0}'
```

Jog:

```bash
rostopic pub -1 /arm/jog_cmd std_msgs/String '{"axis":"z","delta":1.0,"vel":10.0}'
```

## 리프트

```bash
rosservice call /lifter/home "{}"
rosservice call /lifter/stop "{}"
rosservice call /lifter/reset "{}"
rosservice call /lifter/goto_scan_height "{}"
rostopic echo /lifter/state
rostopic echo /lifter/height
rostopic echo /lifter/ready
rostopic echo /lifter/busy
```

높이/위치 명령:

```bash
rostopic pub -1 /lifter/height_cmd std_msgs/Float32 "data: 150.0"
rostopic pub -1 /lifter/position_cmd std_msgs/Int32 "data: 3000"
rostopic pub -1 /lifter/jog_cmd std_msgs/Int32 "data: 100"
```

수동 명령:

```bash
rostopic pub -1 /lifter/command std_msgs/String "up"
rostopic pub -1 /lifter/command std_msgs/String "down"
rostopic pub -1 /lifter/command std_msgs/String "stop"
rostopic pub -1 /lifter/command std_msgs/String "reset"
```

## Map Calibration

주의: `map_calibrator.launch`는 `/cmd_vel`을 사용할 수 있으므로 `apriltag_nav mobile_manipulator.launch`의 mobile controller와 동시에 실행하지 않습니다.

```bash
roslaunch path_tag_locator map_calibrator.launch
```

기본 calibration:

```bash
rosservice call /map_calibrator/run_calibration "{
  plan_path: '',
  ref_tags_path: '',
  map_in_path: '',
  map_out_path: '',
  dry_run: false
}"
```

dry run:

```bash
rosservice call /map_calibrator/run_calibration "{
  plan_path: '',
  ref_tags_path: '',
  map_in_path: '',
  map_out_path: '',
  dry_run: true
}"
```

진행 확인:

```bash
rostopic echo /map_calibrator/progress
rostopic echo /map_calibrator/current_target_tag
```

결과 확인:

```bash
ls -lt ~/.ros/path_tag_locator/map_world_*.yaml | head -1
ls -lt ~/.ros/path_tag_locator/tag_alignment_results.yaml
```

## Path Tag Locator

```bash
roslaunch path_tag_locator path_tag_locator.launch
```

단일 태그 localization:

```bash
rosservice call /path_tag_locator/locate_path_tag "{
  tag_b_id: 101,
  override_ref: false,
  save_result: true,
  save_dir: '',
  auto_align: false,
  align_initial_tcp_mm_deg: [0, 0, 0, 0, 0, 0]
}"
```

## Hand-Eye Calibration

```bash
roslaunch path_tag_locator handeye_calib.launch
rosservice call /handeye_calib/capture "{}"
rosservice call /handeye_calib/status "{}"
rosservice call /handeye_calib/compute "{}"
rosservice call /handeye_calib/reset "{}"
rosservice call /handeye_calib/load_latest "{}"
```

npz 저장:

```bash
rosrun path_tag_locator save_npz.py
rosrun path_tag_locator save_npz.py --force
rosrun path_tag_locator save_npz.py --from-yaml /path/to/T_hc2ee.yaml
rosrun path_tag_locator save_npz.py --out /tmp/T_hc2ee.npz
rosrun path_tag_locator save_npz.py --hardcoded
```

## Calibration 검증

```bash
rosrun path_tag_locator verify_map_world.py
rosrun path_tag_locator verify_map_world.py --threshold-m 0.10
rosrun path_tag_locator visualize_map_world.py
rosrun path_tag_locator verify_arm_pointing.py --tag-id 101
rosrun path_tag_locator verify_arm_pointing.py --all
rosrun path_tag_locator test_repeatability.py
```

## 디버그 / 테스트 툴

```bash
rosrun apriltag_nav send_debug_cmd.py "TASK scan_joints_line1"
rosrun apriltag_nav send_debug_cmd.py "GOTO 108"
rosrun apriltag_nav navigate.py 108
rosrun apriltag_nav test_all_devices.py
rosrun apriltag_nav test_scan_chain.py
rosrun apriltag_nav lift_calib_ui.py
rosrun apriltag_nav measure_keyence_angle.py
rosrun apriltag_nav vw_drive.py
rosrun apriltag_nav validate_transform.py
rosrun apriltag_nav validate_compare.py
```

## GitHub Push

현재 로컬 브랜치 `real`을 GitHub `main`으로 올릴 때:

```bash
cd /home/jun/mobile_manipulator_ws_20260819/mobile_manipulator_ws
git push -u origin real:main
```

GitHub token을 사용하는 경우:

```bash
read -s -p 'GitHub token: ' GITHUB_TOKEN
echo
git push -u "https://pjs0209:${GITHUB_TOKEN}@github.com/pjs0209/mobile_manipulator_ws.git" real:main --force
unset GITHUB_TOKEN
```
