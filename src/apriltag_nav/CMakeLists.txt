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
# Keep this list equal to scripts/*.py. lifter_node and camera_viewer_node were
# missing here until 2026-08-11 — they ran anyway because roslaunch resolves
# `type=` from the source tree in a devel-space build, so the gap only shows up
# in an installed workspace.
install(PROGRAMS
  scripts/task_executor.py
  scripts/mobile_node.py
  scripts/arm_node.py
  scripts/lifter_node.py
  scripts/basler_camera_node.py
  scripts/keyence_dlen1_node.py
  scripts/robot_camera_node.py
  scripts/camera_viewer_node.py
  scripts/inference_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
