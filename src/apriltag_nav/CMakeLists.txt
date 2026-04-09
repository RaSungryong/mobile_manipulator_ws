cmake_minimum_required(VERSION 3.0.2)
project(apriltag_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
)

catkin_python_setup()

catkin_package()

install(PROGRAMS
  scripts/navigate.py
  scripts/robot_controller.py
  scripts/map_manager.py
  scripts/utils.py
  scripts/arm_controller.py
  scripts/arm_controller_sdk.py
  scripts/FrCmd.py
  scripts/task_executor.py
  scripts/task_manager.py
  # Plan B — distributed ROS node architecture (real robot)
  scripts_ros/basler_camera_node.py
  scripts_ros/arm_controller_ros.py
  scripts_ros/task_executor_ros.py
  # Plan B — Isaac Sim testing
  scripts_ros/arm_controller_ros_sim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
