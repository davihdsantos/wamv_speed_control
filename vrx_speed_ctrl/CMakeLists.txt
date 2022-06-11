cmake_minimum_required(VERSION 3.0.2)
project(vrx_speed_ctrl)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vrx_speed_ctrl
#  CATKIN_DEPENDS rospy std
#  DEPENDS system_lib
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
 include
  ${catkin_INCLUDE_DIRS}  ${GAZEBO_INCLUDE_DIRS} 
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

add_executable(autotune nodes/autotune.cpp)
target_link_libraries(autotune ${catkin_LIBRARIES})