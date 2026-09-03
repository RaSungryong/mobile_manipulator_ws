cmake_minimum_required(VERSION 3.0.2)
project(robot_sim)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/sim_node.py
  scripts/test_nav.py
  scripts/test_calibration.py
  scripts/viz_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
