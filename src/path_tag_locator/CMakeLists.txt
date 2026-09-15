cmake_minimum_required(VERSION 3.0.2)
project(path_tag_locator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  message_generation
  robot_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  LocatePathTag.srv
  RunMapCalibration.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    std_srvs
    geometry_msgs
    sensor_msgs
    nav_msgs
    cv_bridge
    message_runtime
    robot_msgs
)

# ⚠️ Every script a doc, a plan header or the UI tells an operator to
# `rosrun` must be listed here, or that instruction is a dead end. The
# last four were missing (2026-09-11): analyse_yaw_sweep is named in the
# yaw-sweep plan headers and in robot_ui's calibration note, and
# error_budget / update_plan_seeds are named in HANDOVER.
install(PROGRAMS
  scripts/path_tag_locator_node.py
  scripts/handeye_calib_node.py
  scripts/map_calibrator_node.py
  scripts/generate_calibration_artifacts.py
  scripts/save_npz.py
  scripts/verify_map_world.py
  scripts/test_repeatability.py
  scripts/verify_arm_pointing.py
  scripts/visualize_map_world.py
  scripts/analyse_yaw_sweep.py
  scripts/check_repose_from_corners.py
  scripts/check_front_cam_extrinsics.py
  scripts/make_front_cam_extrinsics.py
  scripts/calib_fc_hc_chain.py
  scripts/check_chain_calib.py
  scripts/error_budget.py
  scripts/update_plan_seeds_from_session.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
