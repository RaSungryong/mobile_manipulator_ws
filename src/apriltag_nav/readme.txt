How to run (real robot)
=======================
roscore is started by the Navifra driver's systemd service (navifra-robot) —
do not start one. Since 2026-09-22 the full stack starts at boot as the
systemd service `mobile-manipulator` (src/apriltag_nav/tools/systemd/):
  sudo systemctl {start|stop|restart|status} mobile-manipulator
  journalctl -u mobile-manipulator -f
Stop the service before a hand launch of the same file (two copies collide):
  cd ~/mobile_manipulator_ws && source devel/setup.bash
  roslaunch apriltag_nav mobile_manipulator.launch

The launch starts the 8 apriltag_nav nodes (task_executor, mobile_node,
arm_node, lifter_node, basler_camera_node, keyence_dlen1_node,
robot_camera_node, inference_node) plus rosbridge on :9090 and the web
operator UI on :8080 (http://192.168.1.100:8080). Stopping: tools/stop_stack.sh.

Run a task
Task names are derived from the path-data files in task/csv
(rostopic echo /task_list, or the web UI's Task tab):
  assigned_workpoints_<key>.csv -> scan_pose_<key>   (end-effector poses, IK per point)
  rrt_final_path_<key>.csv      -> scan_joint_<key>  (joint-angle path, MoveJ replay)
Today (2026-10-06 15:54) the files are two joint-only paths, keys
offset0mm_h652 / offset10mm_h662 (no pose twin, so no scan_pose_* task):
rostopic pub -1 /task_command std_msgs/String "TASK scan_joint_offset0mm_h652"
rostopic pub -1 /task_command std_msgs/String "TASK scan_joint_offset10mm_h662"
rostopic pub -1 /task_command std_msgs/String "RELOAD_TASKS"   # re-scan task/csv

Dock and charge / undock (the charger only starts on /crevis/charging true after docking)
rostopic pub -1 /task_command std_msgs/String "CHARGE"    # lift home -> tag 500 -> /crevis/charging true
rostopic pub -1 /task_command std_msgs/String "UNDOCK"    # /crevis/charging false; the base stays on the dock

Query state / immediate stop (halts both motion and scanning)
rostopic pub -1 /task_command std_msgs/String "STATE"
rostopic pub -1 /task_command std_msgs/String "STOP"

Useful topics to monitor
rostopic echo /task_state        # orchestrator state (latched JSON)
rostopic echo /keyence/value     # Keyence distance
rostopic echo /arm/state         # arm TCP pose / joints / busy
rostopic echo /camera/state      # camera state (closed/open/capturing)
rostopic echo /scan_finished     # scan completion signal
