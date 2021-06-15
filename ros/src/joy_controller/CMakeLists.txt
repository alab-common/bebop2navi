cmake_minimum_required(VERSION 3.0.2)
project(joy_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES joy_controller
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(joy_controller src/joy_controller.cpp)
target_link_libraries(joy_controller ${catkin_LIBRARIES})

