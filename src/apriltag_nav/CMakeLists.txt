cmake_minimum_required(VERSION 3.0.2)
project(apriltag_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  # robot_msgs supplies Pose2DWithFlag (long-standing) and CaptureImages.srv.
  # Declaring it is what guarantees robot_msgs' generated Python is built before
  # this package installs — without it the build order is luck.
  robot_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
    robot_msgs
)

# Only node entry points are installed as programs. Library modules live in
# src/apriltag_nav/ and are installed as a python package by
# catkin_python_setup() (see setup.py); tools/ is developer-only and not
# installed. Every PROGRAM lands in one directory, so keep basenames unique.
install(PROGRAMS
  scripts/task_executor.py
  scripts/arm_controller_node.py
  scripts/basler_camera_node.py
  scripts/keyence_dlen1_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
