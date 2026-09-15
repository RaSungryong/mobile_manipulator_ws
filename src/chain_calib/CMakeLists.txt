cmake_minimum_required(VERSION 3.0.2)
project(chain_calib)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

# Every script an operator `rosrun`s must be listed here.
install(PROGRAMS
  scripts/chain_calib.py
  scripts/check_chain_calib.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
