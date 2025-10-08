cmake_minimum_required(VERSION 3.0.2)
project(cruise_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  actionlib
  actionlib_msgs
  costmap_2d
  coverage_path_planner
  gazebo_msgs
  geometry_msgs
  message_generation
  mbf_msgs
  nav_msgs
  roslaunch
  sensor_msgs
  tf2
  tf2_ros
)

catkin_package(CATKIN_DEPENDS
        actionlib_msgs
        costmap_2d
        coverage_path_planner
        gazebo_msgs
        geometry_msgs
        message_runtime
        mbf_msgs
        nav_msgs
        roscpp
        sensor_msgs
        tf2
        tf2_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

add_executable(trans_tf_2d src/Trans_TF_2d.cpp)
target_link_libraries(trans_tf_2d ${catkin_LIBRARIES})

add_executable(trans_tf_2d_odom src/Trans_TF_2d_odom.cpp)
target_link_libraries(trans_tf_2d_odom ${catkin_LIBRARIES})

add_executable(trans_odom_2d src/Trans_odom_2d.cpp)
target_link_libraries(trans_odom_2d ${catkin_LIBRARIES})

add_executable(mbf_bridge src/mbf_bridge_node.cpp)
target_link_libraries(mbf_bridge ${catkin_LIBRARIES})

