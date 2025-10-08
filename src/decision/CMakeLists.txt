cmake_minimum_required(VERSION 3.0.2)
project(cruise_decision)
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
  tf
  tf2_ros
  actionlib
  actionlib_msgs
  ranger_msgs
  sound_play
  roslib
  costmap_2d

)

find_library(JSONCPP_LIBRARY NAMES jsoncpp HINTS /usr/lib/x86_64-linux-gnu)
set(JSONCPP_LIB ${JSONCPP_LIBRARY})

# add_message_files(
#   FILES
#   AirSensors.msg
# )

# add_service_files(
#   FILES
#   TaskService.srv
# )

# add_action_files(
#   FILES
#   TaskExecuter.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cruise_decision
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  src/
  include
)

# add_executable(RobotManager  src/RobotManager.cpp src/cJSON.c)
# add_dependencies(RobotManager ${PROJECT_NAME}_generate_messages_cpp)
# target_link_libraries(RobotManager  
#   ${catkin_LIBRARIES}
#   ${JSONCPP_LIB}
#   mosquitto
# )

add_executable(DeviceManagerNode
        src/DeviceManagerNode.cpp
        src/cJSON.c
  src/DeviceManager.cpp
  src/DeviceManagerInit.cpp
  src/DeviceManagerMotion.cpp
  src/DeviceManagerTask.cpp
  src/DeviceManagerMqtt.cpp)
add_dependencies(DeviceManagerNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(DeviceManagerNode ${catkin_LIBRARIES} ${JSONCPP_LIB} mosquitto)

