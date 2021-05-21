cmake_minimum_required(VERSION 3.0.2)
project(velocity_selector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES velocity_selector
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velocity_selector src/velocity_selector.cpp)
target_link_libraries(velocity_selector ${catkin_LIBRARIES})
