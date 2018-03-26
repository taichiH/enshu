cmake_minimum_required(VERSION 2.8.3)
project(aero_enshu)

find_package(catkin REQUIRED
  roscpp tf std_msgs sensor_msgs geometry_msgs roslib
  aero_std negomo_enshu aero_devel_lib
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
 CATKIN_DEPENDS roscpp tf std_msgs sensor_msgs geometry_msgs roslib aero_devel_lib negomo_enshu aero_std
)

add_definitions(-std=c++11)

add_executable(pick_caffe src/pick_caffe.cc)
target_link_libraries(pick_caffe ${catkin_LIBRARIES})

add_executable(hand_caffe src/hand_caffe.cc)
target_link_libraries(hand_caffe ${catkin_LIBRARIES})

add_executable(detect_hand nodes/detect_hand.cc)
target_link_libraries(detect_hand ${catkin_LIBRARIES})
