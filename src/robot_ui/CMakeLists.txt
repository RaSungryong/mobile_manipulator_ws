cmake_minimum_required(VERSION 3.0.2)
project(robot_ui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  std_srvs
  robot_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs std_srvs robot_msgs
)

catkin_install_python(PROGRAMS
  scripts/robot_ui_node.py
  scripts/robot_ui_web_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# The plugin directory is read at runtime through an rospkg-resolved path
# (robot_ui.paths.PLUGIN_DIR), so it must exist in the install space too, not
# just in devel. Same for web/ (the page the web UI serves, robot_ui.web_server.WEB_DIR).
install(DIRECTORY launch plugins web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
