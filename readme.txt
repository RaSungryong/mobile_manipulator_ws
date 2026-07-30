실행 방법
====실제로봇====================
1. ROS 실행
cd mobile_manipulator_ws
source devel/setup.bash
roscore

2. TaskExecutor 실행
cd mobile_manipulator_ws
source devel/setup.bash
rosrun apriltag_nav task_executor.py

Task 실행 명령
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"

상태 확인
rostopic pub -1 /task_command std_msgs/String "STATE"

즉시 중지 (이동 + 스캔 모두 정지)
rostopic pub -1 /task_command std_msgs/String "STOP"

# go_home
rostopic pub /task_command std_msgs/String "data: 'TASK go_home'"

# go to tag（no scan）
rostopic pub -1 /task_command std_msgs/String "data: 'GOTO 104'"

