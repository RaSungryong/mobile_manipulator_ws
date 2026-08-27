How to run
==== Real robot ====================
1. Start ROS
roscore

2. Start the full system (all four device nodes together)
cd ~/mobile_manipulator_ws
source devel/setup.bash
roslaunch apriltag_nav mobile_manipulator.launch

* This single command starts all four nodes:
   - keyence_dlen1_node   : Keyence DL-EN1 distance sensor (192.168.100.105:64000)
   - basler_camera_node   : wrist Basler camera + VISION lamp (/camera/capture)
   - arm_node  : Fairino arm + ONNX inference + Keyence closed loop
   - task_executor        : main task orchestrator (STATUS lamp, e-stop, battery)

Run a task
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line1"
rostopic pub -1 /task_command std_msgs/String "TASK scan_joints_line2"

Query state
rostopic pub -1 /task_command std_msgs/String "STATE"

Immediate stop (halts both motion and scanning)
rostopic pub -1 /task_command std_msgs/String "STOP"

Parameter override example (optional)
roslaunch apriltag_nav mobile_manipulator.launch keyence_tol:=0.1 num_samples:=3

Useful topics to monitor
rostopic echo /keyence/value     # Keyence distance
rostopic echo /arm/status        # arm node state (idle/busy)
rostopic echo /camera/state      # camera state (closed/open/capturing)
rostopic echo /scan_finished     # scan completion signal
