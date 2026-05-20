cmake_minimum_required(VERSION 3.0.2)
project(path_tag_locator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

catkin_python_setup()

add_service_files(
  FILES
  LocatePathTag.srv
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
    cv_bridge
    message_runtime
)

install(PROGRAMS
  scripts/path_tag_locator_node.py
  scripts/handeye_calib_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
