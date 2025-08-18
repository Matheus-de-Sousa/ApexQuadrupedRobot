cmake_minimum_required(VERSION 3.0.2)
project(apex_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
  nodes/apex_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

