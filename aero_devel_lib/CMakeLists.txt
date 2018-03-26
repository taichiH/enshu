cmake_minimum_required(VERSION 2.8.3)
project(aero_devel_lib)
#add_definitions(-DDUMMY_MODE)
find_package(catkin REQUIRED
  roscpp tf std_msgs sensor_msgs geometry_msgs roslib
  aero_std
  dynamic_reconfigure
  aero_recognition_msgs
  eigen_conversions
  message_generation
)

find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aero_devel_lib
  CATKIN_DEPENDS
  roscpp tf std_msgs sensor_msgs geometry_msgs roslib aero_std
)

add_definitions(-std=c++11)

add_library(aero_devel_lib
  src/devel_lib.cc
)

target_link_libraries(aero_devel_lib
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)