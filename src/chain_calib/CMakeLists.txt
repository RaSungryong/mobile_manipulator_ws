cmake_minimum_required(VERSION 3.0.2)
project(chain_calib)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

# Every script an operator `rosrun`s must be listed here.
install(PROGRAMS
  scripts/chain_calib.py
  scripts/check_chain_calib.py
  scripts/verify_chain.py
  scripts/sheet_path.py
  scripts/arm_offsets.py
  scripts/basler_tip_calib.py
  scripts/check_basler_tip.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
