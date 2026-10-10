cmake_minimum_required(VERSION 3.0.2)
project(scout_mini_voice_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs
)

catkin_install_python(PROGRAMS
  scripts/voice_recognizer.py
  scripts/voice_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/start_sim.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
