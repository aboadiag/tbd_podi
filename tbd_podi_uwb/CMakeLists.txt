cmake_minimum_required(VERSION 3.0.2)
project(tbd_podi_uwb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tbd_podi_zed
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/uwb_positioning.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
