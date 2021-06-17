cmake_minimum_required(VERSION 3.0.2)
project(pose_fuser)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_fuser
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  "/usr/include"
)

add_executable(pose_fuser src/pose_fuser.cpp src/matrix_generator.cpp)
target_link_libraries(pose_fuser ${catkin_LIBRARIES})
