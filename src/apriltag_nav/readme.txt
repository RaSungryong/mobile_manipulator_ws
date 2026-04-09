실행 방법
====실제로봇====================
1. ROS 실행
roscore

2. 전체 시스템 실행 (Keyence 센서 + 팔 컨트롤러 + 태스크 실행기 통합 시작)
cd ~/mobile_manipulator_ws
source devel/setup.bash
roslaunch apriltag_nav mobile_manipulator.launch

※ 위 명령 하나로 아래 3개 노드가 모두 시작됩니다:
   - keyence_dlen1_node  : Keyence DL-EN1 거리 센서 (192.168.1.5:64000, 60Hz)
   - arm_controller      : Basler 카메라 + ONNX 추론 + Keyence 폐루프 제어
   - task_executor       : 메인 태스크 오케스트레이터

Task 실행 명령
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"

상태 확인
rostopic pub -1 /task_command std_msgs/String "STATE"

즉시 중지 (이동 + 스캔 모두 정지)
rostopic pub -1 /task_command std_msgs/String "STOP"

파라미터 오버라이드 예시 (선택)
roslaunch apriltag_nav mobile_manipulator.launch keyence_tol:=0.1 num_samples:=3

주요 토픽 모니터링
rostopic echo /keyence/value          # Keyence 거리 (m 단위)
rostopic echo /arm_controller/status  # 팔 컨트롤러 상태
rostopic echo /task_status            # 태스크 완료 신호

====isaacsim 환경====================
Isaac Sim 실행
play 버튼

cd ~/mobile_manipulator_ws
source devel/setup.bash
roslaunch fr10v6_vision_251219_moveit_config fr10v6_vision_251219_isaac_execution.launch

cd ~/mobile_manipulator_ws
source devel/setup.bash
rosrun apriltag_nav task_executor.py

Task 실행 명령
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line2"

상태 확인
rostopic pub -1 /task_command std_msgs/String "STATE"

즉시 중지 (이동 + 스캔 모두 정지)
rostopic pub -1 /task_command std_msgs/String "STOP"
