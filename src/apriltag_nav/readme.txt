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
Task names come from the path-data files in task/csv (rostopic echo /task_list):
  assigned_workpoints_<key>.csv -> scan_pose_<key>   (end-effector poses, IK per point)
  rrt_final_path_<key>.csv      -> scan_joint_<key>  (joint-angle path, MoveJ replay)
rostopic pub -1 /task_command std_msgs/String "TASK scan_pose_errorY_p000mm_standoff_010mm_height_652mm"
rostopic pub -1 /task_command std_msgs/String "TASK scan_joint_errorY_p000mm_standoff_010mm_height_652mm"
rostopic pub -1 /task_command std_msgs/String "RELOAD_TASKS"   # re-scan task/csv

Dock and charge / undock (the charger only starts on /crevis/charging true after docking)
rostopic pub -1 /task_command std_msgs/String "CHARGE"    # lift home -> tag 500 -> /crevis/charging true
rostopic pub -1 /task_command std_msgs/String "UNDOCK"    # /crevis/charging false -> 0.10 m forward

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
