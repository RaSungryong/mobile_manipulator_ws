How to run
==== Real robot ====================
1. Start ROS
cd mobile_manipulator_ws
source devel/setup.bash
roscore

2. Start the system (all nodes: arm, camera, Keyence, task executor)
cd mobile_manipulator_ws
source devel/setup.bash
roslaunch apriltag_nav mobile_manipulator.launch

Run a task
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line2"

Query state
rostopic pub -1 /task_command std_msgs/String "STATE"

Immediate stop (halts both motion and scanning)
rostopic pub -1 /task_command std_msgs/String "STOP"

# go_home
rostopic pub /task_command std_msgs/String "data: 'TASK go_home'"

# go to tag (no scan)
rostopic pub -1 /task_command std_msgs/String "data: 'GOTO 104'"
