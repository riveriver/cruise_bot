cmake_minimum_required(VERSION 3.0.2)
project(cruise_msgs)
add_compile_options(-std=c++17)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
)

add_message_files(
  FILES
  AirSensors.msg
)

add_service_files(
  FILES
  TaskService.srv
)

add_action_files(
  FILES
  TaskExecuter.action
  BehaviorExecuter.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cruise_msgs
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

