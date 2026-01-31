cmake_minimum_required(VERSION 2.8.3)
project(lab3_turtlesim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/turtle_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
