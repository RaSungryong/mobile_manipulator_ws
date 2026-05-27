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

install(PROGRAMS
  scripts/path_tag_locator_node.py
  scripts/handeye_calib_node.py
  scripts/map_calibrator_node.py
  scripts/save_npz.py
  scripts/verify_map_world.py
  scripts/test_repeatability.py
  scripts/verify_arm_pointing.py
  scripts/visualize_map_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
